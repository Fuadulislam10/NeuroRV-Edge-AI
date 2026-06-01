// =============================================================================
// NeuroRV Edge — Execute (EX) Stage
// File   : rtl/core/ex_stage.sv
// =============================================================================

`timescale 1ns/1ps
`default_nettype none

`define ALU_ADD    4'b0000
`define ALU_SUB    4'b0001
`define ALU_AND    4'b0010
`define ALU_OR     4'b0011
`define ALU_XOR    4'b0100
`define ALU_SLL    4'b0101
`define ALU_SRL    4'b0110
`define ALU_SRA    4'b0111
`define ALU_SLT    4'b1000
`define ALU_SLTU   4'b1001
`define ALU_LUI    4'b1010
`define ALU_AUIPC  4'b1011
`define ALU_COPY_B 4'b1100

module ex_stage (
    input  logic        clk_i,
    input  logic        rst_ni,
    input  logic        flush_i,
    // ID/EX inputs
    input  logic [31:0] id_ex_pc_i,
    input  logic [31:0] id_ex_rs1_data_i,
    input  logic [31:0] id_ex_rs2_data_i,
    input  logic [31:0] id_ex_imm_i,
    input  logic [4:0]  id_ex_rs1_addr_i,
    input  logic [4:0]  id_ex_rs2_addr_i,
    input  logic [4:0]  id_ex_rd_addr_i,
    input  logic [3:0]  id_ex_alu_op_i,
    input  logic [2:0]  id_ex_funct3_i,
    input  logic        id_ex_alu_src_a_pc_i,
    input  logic        id_ex_alu_src_b_imm_i,
    input  logic        id_ex_reg_write_i,
    input  logic        id_ex_mem_read_i,
    input  logic        id_ex_mem_write_i,
    input  logic        id_ex_branch_i,
    input  logic        id_ex_jal_i,
    input  logic        id_ex_jalr_i,
    input  logic        id_ex_csr_op_i,
    input  logic [11:0] id_ex_csr_addr_i,
    input  logic        id_ex_valid_i,
    // Forwarding
    input  logic [1:0]  fwd_a_sel_i,
    input  logic [1:0]  fwd_b_sel_i,
    input  logic [31:0] ex_mem_result_i,
    input  logic [31:0] mem_wb_result_i,
    // CSR
    input  logic [31:0] csr_rdata_i,
    // MulDiv
    input  logic        muldiv_stall_i,
    input  logic [31:0] muldiv_result_i,
    output logic        muldiv_start_o,
    output logic [2:0]  muldiv_op_o,
    output logic [31:0] muldiv_op_a_o,
    output logic [31:0] muldiv_op_b_o,
    // EX/MEM pipeline register
    output logic [31:0] ex_mem_pc_o,
    output logic [31:0] ex_mem_alu_result_o,
    output logic [31:0] ex_mem_rs2_data_o,
    output logic [4:0]  ex_mem_rd_addr_o,
    output logic [2:0]  ex_mem_funct3_o,
    output logic        ex_mem_reg_write_o,
    output logic        ex_mem_mem_read_o,
    output logic        ex_mem_mem_write_o,
    output logic        ex_mem_valid_o,
    output logic        ex_mem_branch_taken_o,
    output logic [31:0] ex_mem_branch_target_o,
    output logic [31:0] ex_mem_csr_rdata_o
);

    // -------------------------------------------------------------------------
    // Forwarding mux — select operands
    // -------------------------------------------------------------------------
    logic [31:0] op_a_fwd, op_b_fwd;

    always_comb begin
        unique case (fwd_a_sel_i)
            2'b00:   op_a_fwd = id_ex_rs1_data_i;
            2'b01:   op_a_fwd = ex_mem_result_i;
            2'b10:   op_a_fwd = mem_wb_result_i;
            default: op_a_fwd = id_ex_rs1_data_i;
        endcase
    end

    always_comb begin
        unique case (fwd_b_sel_i)
            2'b00:   op_b_fwd = id_ex_rs2_data_i;
            2'b01:   op_b_fwd = ex_mem_result_i;
            2'b10:   op_b_fwd = mem_wb_result_i;
            default: op_b_fwd = id_ex_rs2_data_i;
        endcase
    end

    // ALU source A: PC or RS1
    logic [31:0] alu_src_a;
    logic [31:0] alu_src_b;
    assign alu_src_a = id_ex_alu_src_a_pc_i  ? id_ex_pc_i   : op_a_fwd;
    assign alu_src_b = id_ex_alu_src_b_imm_i ? id_ex_imm_i  : op_b_fwd;

    // -------------------------------------------------------------------------
    // ALU
    // -------------------------------------------------------------------------
    logic [31:0] alu_result;
    logic [4:0]  shamt;
    assign shamt = alu_src_b[4:0];

    always_comb begin
        unique case (id_ex_alu_op_i)
            `ALU_ADD:    alu_result = alu_src_a + alu_src_b;
            `ALU_SUB:    alu_result = alu_src_a - alu_src_b;
            `ALU_AND:    alu_result = alu_src_a & alu_src_b;
            `ALU_OR:     alu_result = alu_src_a | alu_src_b;
            `ALU_XOR:    alu_result = alu_src_a ^ alu_src_b;
            `ALU_SLL:    alu_result = alu_src_a << shamt;
            `ALU_SRL:    alu_result = alu_src_a >> shamt;
            `ALU_SRA:    alu_result = $signed(alu_src_a) >>> shamt;
            `ALU_SLT:    alu_result = ($signed(alu_src_a) < $signed(alu_src_b)) ? 32'd1 : 32'd0;
            `ALU_SLTU:   alu_result = (alu_src_a < alu_src_b) ? 32'd1 : 32'd0;
            `ALU_LUI:    alu_result = alu_src_b;           // LUI: pass imm
            `ALU_COPY_B: alu_result = alu_src_b;
            default:     alu_result = alu_src_a + alu_src_b;
        endcase
    end

    // -------------------------------------------------------------------------
    // Branch evaluation
    // -------------------------------------------------------------------------
    logic        branch_taken;
    logic [31:0] branch_target;
    logic [31:0] eq_diff;
    logic        branch_eq, branch_lt, branch_ltu;

    assign eq_diff    = op_a_fwd - op_b_fwd;
    assign branch_eq  = (op_a_fwd == op_b_fwd);
    assign branch_lt  = ($signed(op_a_fwd) < $signed(op_b_fwd));
    assign branch_ltu = (op_a_fwd < op_b_fwd);

    always_comb begin
        branch_taken = 1'b0;
        if (id_ex_branch_i && id_ex_valid_i) begin
            unique case (id_ex_funct3_i)
                3'b000: branch_taken = branch_eq;           // BEQ
                3'b001: branch_taken = !branch_eq;          // BNE
                3'b100: branch_taken = branch_lt;           // BLT
                3'b101: branch_taken = !branch_lt;          // BGE
                3'b110: branch_taken = branch_ltu;          // BLTU
                3'b111: branch_taken = !branch_ltu;         // BGEU
                default: branch_taken = 1'b0;
            endcase
        end else if ((id_ex_jal_i || id_ex_jalr_i) && id_ex_valid_i) begin
            branch_taken = 1'b1;
        end
    end

    // Branch target
    always_comb begin
        if (id_ex_jalr_i)
            branch_target = (op_a_fwd + id_ex_imm_i) & ~32'h1; // clear LSB per spec
        else
            branch_target = id_ex_pc_i + id_ex_imm_i;          // PC-relative
    end

    // -------------------------------------------------------------------------
    // MulDiv interface
    // -------------------------------------------------------------------------
    assign muldiv_start_o = id_ex_valid_i && (id_ex_instr_funct7_mext()); // placeholder
    assign muldiv_op_o    = id_ex_funct3_i;
    assign muldiv_op_a_o  = op_a_fwd;
    assign muldiv_op_b_o  = op_b_fwd;

    // Helper function placeholder — in real impl id_ex_muldiv would come from decoder
    function automatic logic id_ex_instr_funct7_mext();
        return 1'b0; // driven by id_ex_muldiv_i signal in full implementation
    endfunction

    // Result selection: normal ALU or muldiv result
    logic [31:0] result;
    assign result = muldiv_stall_i ? muldiv_result_i : alu_result;

    // JAL/JALR: rd = PC+4
    logic [31:0] final_result;
    assign final_result = (id_ex_jal_i || id_ex_jalr_i) ? (id_ex_pc_i + 32'd4) :
                          id_ex_csr_op_i                 ? csr_rdata_i            :
                                                           result;

    // -------------------------------------------------------------------------
    // EX/MEM Pipeline register
    // -------------------------------------------------------------------------
    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            ex_mem_pc_o            <= '0;
            ex_mem_alu_result_o    <= '0;
            ex_mem_rs2_data_o      <= '0;
            ex_mem_rd_addr_o       <= '0;
            ex_mem_funct3_o        <= '0;
            ex_mem_reg_write_o     <= '0;
            ex_mem_mem_read_o      <= '0;
            ex_mem_mem_write_o     <= '0;
            ex_mem_valid_o         <= '0;
            ex_mem_branch_taken_o  <= '0;
            ex_mem_branch_target_o <= '0;
            ex_mem_csr_rdata_o     <= '0;
        end else begin
            if (flush_i) begin
                ex_mem_pc_o            <= '0;
                ex_mem_alu_result_o    <= '0;
                ex_mem_rs2_data_o      <= '0;
                ex_mem_rd_addr_o       <= '0;
                ex_mem_funct3_o        <= '0;
                ex_mem_reg_write_o     <= '0;
                ex_mem_mem_read_o      <= '0;
                ex_mem_mem_write_o     <= '0;
                ex_mem_valid_o         <= '0;
                ex_mem_branch_taken_o  <= '0;
                ex_mem_branch_target_o <= '0;
                ex_mem_csr_rdata_o     <= '0;
            end else begin
                ex_mem_pc_o            <= id_ex_pc_i;
                ex_mem_alu_result_o    <= final_result;
                ex_mem_rs2_data_o      <= op_b_fwd;
                ex_mem_rd_addr_o       <= id_ex_rd_addr_i;
                ex_mem_funct3_o        <= id_ex_funct3_i;
                ex_mem_reg_write_o     <= id_ex_reg_write_i & id_ex_valid_i;
                ex_mem_mem_read_o      <= id_ex_mem_read_i  & id_ex_valid_i;
                ex_mem_mem_write_o     <= id_ex_mem_write_i & id_ex_valid_i;
                ex_mem_valid_o         <= id_ex_valid_i;
                ex_mem_branch_taken_o  <= branch_taken;
                ex_mem_branch_target_o <= branch_target;
                ex_mem_csr_rdata_o     <= csr_rdata_i;
            end
        end
    end

endmodule

`default_nettype wire
