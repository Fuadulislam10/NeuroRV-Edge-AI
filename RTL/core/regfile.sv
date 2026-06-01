// =============================================================================
// NeuroRV Edge — Integer Register File (RV32: x0–x31)
// File   : rtl/core/regfile.sv
// =============================================================================
`timescale 1ns/1ps
`default_nettype none

module regfile (
    input  logic        clk_i,
    input  logic        rst_ni,
    // Read ports (combinational)
    input  logic [4:0]  rs1_addr_i,
    input  logic [4:0]  rs2_addr_i,
    output logic [31:0] rs1_data_o,
    output logic [31:0] rs2_data_o,
    // Write port
    input  logic [4:0]  rd_addr_i,
    input  logic [31:0] rd_data_i,
    input  logic        rd_we_i
);

    logic [31:0] regs [0:31];
    integer i;

    // Synchronous write, asynchronous read
    // x0 is hardwired to 0
    always_ff @(posedge clk_i) begin
        if (rd_we_i && (rd_addr_i != 5'h0)) begin
            regs[rd_addr_i] <= rd_data_i;
        end
    end

    assign rs1_data_o = (rs1_addr_i == 5'h0) ? 32'h0 : regs[rs1_addr_i];
    assign rs2_data_o = (rs2_addr_i == 5'h0) ? 32'h0 : regs[rs2_addr_i];

    // Simulation init
    // synthesis translate_off
    initial begin
        for (i = 0; i < 32; i = i+1)
            regs[i] = 32'h0;
    end
    // synthesis translate_on

endmodule

`default_nettype wire


// =============================================================================
// NeuroRV Edge — Pipeline Hazard Detection & Data Forwarding Controller
// File   : rtl/core/pipeline_ctrl.sv
// =============================================================================
`timescale 1ns/1ps
`default_nettype none

module pipeline_ctrl (
    // ID/EX stage info
    input  logic [4:0]  id_ex_rs1_addr_i,
    input  logic [4:0]  id_ex_rs2_addr_i,
    input  logic        id_ex_mem_read_i,
    input  logic [4:0]  id_ex_rd_addr_i,
    // EX/MEM stage info
    input  logic [4:0]  ex_mem_rd_addr_i,
    input  logic        ex_mem_reg_write_i,
    // MEM/WB stage info
    input  logic [4:0]  mem_wb_rd_addr_i,
    input  logic        mem_wb_reg_write_i,
    // Other stall sources
    input  logic        muldiv_stall_i,
    input  logic        branch_taken_i,
    input  logic        trap_taken_i,
    // Outputs
    output logic        stall_if_o,
    output logic        stall_id_o,
    output logic        flush_if_o,
    output logic        flush_id_o,
    output logic        flush_ex_o,
    output logic [1:0]  fwd_a_sel_o,   // 00=regfile 01=ex/mem 10=mem/wb
    output logic [1:0]  fwd_b_sel_o
);

    // -------------------------------------------------------------------------
    // Load-use hazard: stall 1 cycle when EX stage is a load
    // and ID stage needs that register
    // -------------------------------------------------------------------------
    logic load_use_hazard;
    assign load_use_hazard = id_ex_mem_read_i &&
                             ((id_ex_rd_addr_i == id_ex_rs1_addr_i) ||
                              (id_ex_rd_addr_i == id_ex_rs2_addr_i)) &&
                             (id_ex_rd_addr_i != 5'h0);

    // -------------------------------------------------------------------------
    // Stall logic
    // -------------------------------------------------------------------------
    assign stall_if_o = load_use_hazard || muldiv_stall_i;
    assign stall_id_o = load_use_hazard || muldiv_stall_i;

    // -------------------------------------------------------------------------
    // Flush logic
    // -------------------------------------------------------------------------
    assign flush_if_o = branch_taken_i || trap_taken_i;
    assign flush_id_o = branch_taken_i || trap_taken_i || load_use_hazard;
    assign flush_ex_o = trap_taken_i;

    // -------------------------------------------------------------------------
    // Data forwarding — EX/MEM → EX (highest priority)
    //                   MEM/WB → EX
    // -------------------------------------------------------------------------
    // Forward A (rs1)
    always_comb begin
        fwd_a_sel_o = 2'b00; // default: register file
        if (ex_mem_reg_write_i && (ex_mem_rd_addr_i != 5'h0) &&
            (ex_mem_rd_addr_i == id_ex_rs1_addr_i))
            fwd_a_sel_o = 2'b01; // EX/MEM forwarding
        else if (mem_wb_reg_write_i && (mem_wb_rd_addr_i != 5'h0) &&
                 (mem_wb_rd_addr_i == id_ex_rs1_addr_i))
            fwd_a_sel_o = 2'b10; // MEM/WB forwarding
    end

    // Forward B (rs2)
    always_comb begin
        fwd_b_sel_o = 2'b00;
        if (ex_mem_reg_write_i && (ex_mem_rd_addr_i != 5'h0) &&
            (ex_mem_rd_addr_i == id_ex_rs2_addr_i))
            fwd_b_sel_o = 2'b01;
        else if (mem_wb_reg_write_i && (mem_wb_rd_addr_i != 5'h0) &&
                 (mem_wb_rd_addr_i == id_ex_rs2_addr_i))
            fwd_b_sel_o = 2'b10;
    end

endmodule

`default_nettype wire


// =============================================================================
// NeuroRV Edge — Multiply/Divide Unit (RV32M Extension)
// File   : rtl/core/muldiv_unit.sv
//
// MUL    funct3=000 → lower 32b of rs1×rs2 (signed×signed)
// MULH   funct3=001 → upper 32b (signed×signed)
// MULHSU funct3=010 → upper 32b (signed×unsigned)
// MULHU  funct3=011 → upper 32b (unsigned×unsigned)
// DIV    funct3=100 → quotient (signed)
// DIVU   funct3=101 → quotient (unsigned)
// REM    funct3=110 → remainder (signed)
// REMU   funct3=111 → remainder (unsigned)
// =============================================================================
`timescale 1ns/1ps
`default_nettype none

module muldiv_unit (
    input  logic        clk_i,
    input  logic        rst_ni,
    input  logic        start_i,
    input  logic [2:0]  op_i,
    input  logic [31:0] op_a_i,
    input  logic [31:0] op_b_i,
    output logic [31:0] result_o,
    output logic        done_o,
    output logic        stall_o
);

    // -------------------------------------------------------------------------
    // Multiply: single-cycle using 64b DSP inference
    // Divide: iterative non-restoring, 33 cycles max
    // -------------------------------------------------------------------------

    typedef enum logic [1:0] {IDLE, MUL_DONE, DIV_RUN, DIV_DONE} state_t;
    state_t state;

    logic [63:0] mul_result;
    logic [31:0] quotient, remainder;
    logic [5:0]  div_iter;
    logic        div_signed;
    logic        div_rem_op;
    logic [31:0] dividend, divisor;
    logic        div_neg_result;
    logic        rem_neg_result;

    // Sign-extended operands for signed operations
    logic signed [32:0] sa, sb;
    assign sa = {op_a_i[31], op_a_i};
    assign sb = {op_b_i[31], op_b_i};

    // 64-bit multiply
    logic [63:0] mul_uu, mul_ss, mul_su;
    assign mul_uu = {32'h0, op_a_i} * {32'h0, op_b_i};
    assign mul_ss = $signed({{32{op_a_i[31]}}, op_a_i}) * $signed({{32{op_b_i[31]}}, op_b_i});
    assign mul_su = $signed({{32{op_a_i[31]}}, op_a_i}) * {32'h0, op_b_i};

    // Divide state machine
    logic [32:0] partial_rem;
    logic [31:0] div_quotient_r;
    logic [32:0] div_dividend_r;

    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            state      <= IDLE;
            done_o     <= 1'b0;
            stall_o    <= 1'b0;
            result_o   <= '0;
            div_iter   <= '0;
        end else begin
            case (state)
                IDLE: begin
                    done_o  <= 1'b0;
                    if (start_i) begin
                        if (op_i[2] == 1'b0) begin
                            // MUL — combinational, 1-cycle
                            case (op_i)
                                3'b000: result_o <= mul_ss[31:0];
                                3'b001: result_o <= mul_ss[63:32];
                                3'b010: result_o <= mul_su[63:32];
                                3'b011: result_o <= mul_uu[63:32];
                                default: result_o <= mul_ss[31:0];
                            endcase
                            done_o  <= 1'b1;
                            stall_o <= 1'b0;
                        end else begin
                            // DIV/REM — iterative
                            stall_o  <= 1'b1;
                            div_iter <= 6'd32;
                            div_signed  <= (op_i == 3'b100 || op_i == 3'b110);
                            div_rem_op  <= op_i[1];

                            // Handle signed negation
                            if (op_i == 3'b100 || op_i == 3'b110) begin
                                dividend <= op_a_i[31] ? (~op_a_i + 1) : op_a_i;
                                divisor  <= op_b_i[31] ? (~op_b_i + 1) : op_b_i;
                                div_neg_result <= op_a_i[31] ^ op_b_i[31];
                                rem_neg_result <= op_a_i[31];
                            end else begin
                                dividend <= op_a_i;
                                divisor  <= op_b_i;
                                div_neg_result <= 1'b0;
                                rem_neg_result <= 1'b0;
                            end
                            div_dividend_r <= 33'h0;
                            div_quotient_r <= '0;
                            state <= DIV_RUN;
                        end
                    end
                end

                DIV_RUN: begin
                    // Non-restoring division step
                    partial_rem = {div_dividend_r[31:0], dividend[31]};
                    dividend    <= {dividend[30:0], 1'b0};

                    if (partial_rem >= {1'b0, divisor}) begin
                        div_dividend_r <= partial_rem - {1'b0, divisor};
                        div_quotient_r <= {div_quotient_r[30:0], 1'b1};
                    end else begin
                        div_dividend_r <= partial_rem;
                        div_quotient_r <= {div_quotient_r[30:0], 1'b0};
                    end

                    div_iter <= div_iter - 6'd1;
                    if (div_iter == 6'd1) begin
                        state <= DIV_DONE;
                    end
                end

                DIV_DONE: begin
                    // Apply sign correction
                    quotient  = div_neg_result ? (~div_quotient_r + 1) : div_quotient_r;
                    remainder = rem_neg_result ? (~div_dividend_r[31:0] + 1) : div_dividend_r[31:0];

                    // Division by zero
                    if (divisor == 32'h0) begin
                        quotient  = 32'hFFFF_FFFF; // -1 for signed, max for unsigned
                        remainder = dividend;
                    end

                    result_o <= div_rem_op ? remainder : quotient;
                    done_o   <= 1'b1;
                    stall_o  <= 1'b0;
                    state    <= IDLE;
                end

                default: state <= IDLE;
            endcase
        end
    end

endmodule

`default_nettype wire
