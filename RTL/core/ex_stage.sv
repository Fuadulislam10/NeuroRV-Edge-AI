// ============================================================================
// NeuroRV Edge — Execute Stage (EX)
// FILE: rtl/core/ex_stage.sv
//
// Responsibilities:
//   • Operand forwarding (from MEM and WB stages)
//   • ALU operation
//   • Branch condition evaluation and target address generation
//   • Mul/Div dispatch and stall
//   • CSR read/write dispatch
//   • EX/MEM pipeline register
//
// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2024-2025 NeuroRV Edge Contributors
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

// Import types defined in id_stage (in real flow: package or include)
// Replicated here for self-contained compilation
typedef enum logic [3:0] {
    ALU_ADD  = 4'h0, ALU_SUB  = 4'h1, ALU_AND  = 4'h2,
    ALU_OR   = 4'h3, ALU_XOR  = 4'h4, ALU_SLL  = 4'h5,
    ALU_SRL  = 4'h6, ALU_SRA  = 4'h7, ALU_SLT  = 4'h8,
    ALU_SLTU = 4'h9, ALU_COPY_B = 4'hA
} ex_alu_op_e;

typedef enum logic [1:0] {
    SRC_REG = 2'b00, SRC_IMM = 2'b01, SRC_PC = 2'b10
} ex_src_sel_e;

typedef enum logic [2:0] {
    BR_NONE = 3'b000, BR_BEQ  = 3'b001, BR_BNE  = 3'b010,
    BR_BLT  = 3'b011, BR_BGE  = 3'b100, BR_BLTU = 3'b101, BR_BGEU = 3'b110
} ex_br_type_e;

typedef enum logic [1:0] {
    WB_ALU = 2'b00, WB_MEM = 2'b01, WB_PC4 = 2'b10, WB_CSR = 2'b11
} ex_wb_src_e;

// Forwarding select
typedef enum logic [1:0] {
    FWD_REG = 2'b00,   // from register file
    FWD_MEM = 2'b01,   // from MEM stage alu_result
    FWD_WB  = 2'b10    // from WB stage writeback data
} fwd_sel_e;

module ex_stage (
    input  logic        clk,
    input  logic        rst_n,

    // From ID/EX pipeline register
    input  logic [31:0] id_ex_pc,
    input  logic [31:0] id_ex_rs1_data,
    input  logic [31:0] id_ex_rs2_data,
    input  logic [31:0] id_ex_imm,
    input  logic [4:0]  id_ex_rs1_addr,
    input  logic [4:0]  id_ex_rs2_addr,
    input  logic [4:0]  id_ex_rd_addr,
    input  ex_alu_op_e  id_ex_alu_op,
    input  ex_src_sel_e id_ex_alu_src_a,
    input  ex_src_sel_e id_ex_alu_src_b,
    input  ex_br_type_e id_ex_br_type,
    input  logic        id_ex_is_load,
    input  logic        id_ex_is_store,
    input  logic [2:0]  id_ex_mem_width,
    input  logic        id_ex_rf_we,
    input  ex_wb_src_e  id_ex_wb_src,
    input  logic        id_ex_is_muldiv,
    input  logic [2:0]  id_ex_muldiv_op,
    input  logic        id_ex_csr_en,
    input  logic [11:0] id_ex_csr_addr,
    input  logic [2:0]  id_ex_csr_op,
    input  logic        id_ex_mret,
    input  logic        id_ex_ecall,
    input  logic        id_ex_ebreak,
    input  logic        id_ex_valid,

    // Forwarding inputs
    input  logic        mem_rf_we,
    input  logic [4:0]  mem_rd_addr,
    input  logic [31:0] mem_alu_result,     // MEM stage forwarding value
    input  logic        wb_rf_we,
    input  logic [4:0]  wb_rd_addr,
    input  logic [31:0] wb_rd_data,         // WB stage writeback value

    // CSR interface
    output logic        csr_en,
    output logic [11:0] csr_addr,
    output logic [2:0]  csr_op,
    output logic [31:0] csr_wdata,
    input  logic [31:0] csr_rdata,

    // Stall from muldiv
    output logic        ex_stall,

    // Branch outputs to IF stage
    output logic        branch_taken,
    output logic [31:0] branch_target,

    // Trap signals to pipeline ctrl
    output logic        ex_mret,
    output logic        ex_ecall,
    output logic        ex_ebreak,

    // Stall / Flush
    input  logic        stall_ex,
    input  logic        flush_ex,

    // EX/MEM pipeline register outputs
    output logic [31:0] ex_mem_pc,
    output logic [31:0] ex_mem_alu_result,
    output logic [31:0] ex_mem_rs2_data,     // store data (possibly forwarded)
    output logic [4:0]  ex_mem_rd_addr,
    output logic        ex_mem_is_load,
    output logic        ex_mem_is_store,
    output logic [2:0]  ex_mem_mem_width,
    output logic        ex_mem_rf_we,
    output ex_wb_src_e  ex_mem_wb_src,
    output logic [31:0] ex_mem_csr_rdata,
    output logic        ex_mem_valid,

    // Debug
    output logic [31:0] dbg_alu_a,
    output logic [31:0] dbg_alu_b,
    output logic [31:0] dbg_alu_result
);

    // -----------------------------------------------------------------------
    // Forwarding logic
    // -----------------------------------------------------------------------
    fwd_sel_e fwd_a, fwd_b;

    always_comb begin
        // Forward A (rs1)
        if (mem_rf_we && (mem_rd_addr != '0) && (mem_rd_addr == id_ex_rs1_addr))
            fwd_a = FWD_MEM;
        else if (wb_rf_we && (wb_rd_addr != '0) && (wb_rd_addr == id_ex_rs1_addr))
            fwd_a = FWD_WB;
        else
            fwd_a = FWD_REG;

        // Forward B (rs2)
        if (mem_rf_we && (mem_rd_addr != '0) && (mem_rd_addr == id_ex_rs2_addr))
            fwd_b = FWD_MEM;
        else if (wb_rf_we && (wb_rd_addr != '0) && (wb_rd_addr == id_ex_rs2_addr))
            fwd_b = FWD_WB;
        else
            fwd_b = FWD_REG;
    end

    logic [31:0] rs1_fwd, rs2_fwd;
    always_comb begin
        unique case (fwd_a)
            FWD_MEM: rs1_fwd = mem_alu_result;
            FWD_WB:  rs1_fwd = wb_rd_data;
            default: rs1_fwd = id_ex_rs1_data;
        endcase
        unique case (fwd_b)
            FWD_MEM: rs2_fwd = mem_alu_result;
            FWD_WB:  rs2_fwd = wb_rd_data;
            default: rs2_fwd = id_ex_rs2_data;
        endcase
    end

    // -----------------------------------------------------------------------
    // ALU operand selection
    // -----------------------------------------------------------------------
    logic [31:0] alu_op_a, alu_op_b;

    always_comb begin
        unique case (id_ex_alu_src_a)
            SRC_PC:  alu_op_a = id_ex_pc;
            default: alu_op_a = rs1_fwd;
        endcase
        unique case (id_ex_alu_src_b)
            SRC_IMM: alu_op_b = id_ex_imm;
            default: alu_op_b = rs2_fwd;
        endcase
    end

    // -----------------------------------------------------------------------
    // ALU
    // -----------------------------------------------------------------------
    logic [31:0] alu_result;
    logic [4:0]  shamt;
    assign shamt = alu_op_b[4:0];

    always_comb begin
        unique case (id_ex_alu_op)
            ALU_ADD:    alu_result = alu_op_a + alu_op_b;
            ALU_SUB:    alu_result = alu_op_a - alu_op_b;
            ALU_AND:    alu_result = alu_op_a & alu_op_b;
            ALU_OR:     alu_result = alu_op_a | alu_op_b;
            ALU_XOR:    alu_result = alu_op_a ^ alu_op_b;
            ALU_SLL:    alu_result = alu_op_a << shamt;
            ALU_SRL:    alu_result = alu_op_a >> shamt;
            ALU_SRA:    alu_result = $signed(alu_op_a) >>> shamt;
            ALU_SLT:    alu_result = {31'b0, $signed(alu_op_a) < $signed(alu_op_b)};
            ALU_SLTU:   alu_result = {31'b0, alu_op_a < alu_op_b};
            ALU_COPY_B: alu_result = alu_op_b;
            default:    alu_result = alu_op_a + alu_op_b;
        endcase
    end

    // -----------------------------------------------------------------------
    // Branch condition evaluation
    // -----------------------------------------------------------------------
    logic br_cond;
    always_comb begin
        unique case (id_ex_br_type)
            BR_BEQ:  br_cond = (rs1_fwd == rs2_fwd);
            BR_BNE:  br_cond = (rs1_fwd != rs2_fwd);
            BR_BLT:  br_cond = ($signed(rs1_fwd) < $signed(rs2_fwd));
            BR_BGE:  br_cond = ($signed(rs1_fwd) >= $signed(rs2_fwd));
            BR_BLTU: br_cond = (rs1_fwd < rs2_fwd);
            BR_BGEU: br_cond = (rs1_fwd >= rs2_fwd);
            default: br_cond = 1'b0;
        endcase
    end

    // Branch taken for conditional branches, always taken for JAL/JALR
    logic is_jal, is_jalr;
    assign is_jal  = (id_ex_wb_src == WB_PC4) && (id_ex_alu_src_a == SRC_PC);
    assign is_jalr = (id_ex_wb_src == WB_PC4) && (id_ex_alu_src_a == SRC_REG);

    assign branch_taken  = id_ex_valid && (
                               (id_ex_br_type != BR_NONE && br_cond) ||
                               is_jal || is_jalr);

    // Branch target: ALU computed for branches/JAL/JALR
    // For JALR, clear bit 0 per RISC-V spec
    assign branch_target = is_jalr ? {alu_result[31:1], 1'b0} : alu_result;

    // -----------------------------------------------------------------------
    // Mul/Div unit instantiation
    // -----------------------------------------------------------------------
    logic        muldiv_start;
    logic [31:0] muldiv_result;
    logic        muldiv_busy;
    logic        muldiv_done;

    assign muldiv_start = id_ex_is_muldiv && id_ex_valid && !stall_ex;
    assign ex_stall     = muldiv_busy;

    muldiv_unit u_muldiv (
        .clk      (clk),
        .rst_n    (rst_n),
        .start    (muldiv_start),
        .op       (muldiv_op_e'(id_ex_muldiv_op)),
        .rs1      (rs1_fwd),
        .rs2      (rs2_fwd),
        .result   (muldiv_result),
        .busy     (muldiv_busy),
        .done     (muldiv_done)
    );

    // -----------------------------------------------------------------------
    // CSR interface
    // -----------------------------------------------------------------------
    assign csr_en    = id_ex_csr_en && id_ex_valid;
    assign csr_addr  = id_ex_csr_addr;
    assign csr_op    = id_ex_csr_op;
    // For CSRxI instructions (funct3[2]==1), write data is zero-extended rs1 field
    assign csr_wdata = (id_ex_csr_op[2]) ? {27'b0, id_ex_rs1_addr} : rs1_fwd;

    // -----------------------------------------------------------------------
    // Effective EX result mux (muldiv overrides ALU when done)
    // -----------------------------------------------------------------------
    logic [31:0] ex_result;
    assign ex_result = (id_ex_is_muldiv && muldiv_done) ? muldiv_result : alu_result;

    // -----------------------------------------------------------------------
    // Trap/system signals
    // -----------------------------------------------------------------------
    assign ex_mret   = id_ex_mret   && id_ex_valid;
    assign ex_ecall  = id_ex_ecall  && id_ex_valid;
    assign ex_ebreak = id_ex_ebreak && id_ex_valid;

    // -----------------------------------------------------------------------
    // EX/MEM Pipeline Register
    // -----------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n || flush_ex) begin
            ex_mem_pc          <= '0;
            ex_mem_alu_result  <= '0;
            ex_mem_rs2_data    <= '0;
            ex_mem_rd_addr     <= '0;
            ex_mem_is_load     <= 1'b0;
            ex_mem_is_store    <= 1'b0;
            ex_mem_mem_width   <= '0;
            ex_mem_rf_we       <= 1'b0;
            ex_mem_wb_src      <= WB_ALU;
            ex_mem_csr_rdata   <= '0;
            ex_mem_valid       <= 1'b0;
        end else if (!stall_ex) begin
            ex_mem_pc          <= id_ex_pc;
            ex_mem_alu_result  <= ex_result;
            ex_mem_rs2_data    <= rs2_fwd;    // forwarded store data
            ex_mem_rd_addr     <= id_ex_rd_addr;
            ex_mem_is_load     <= id_ex_is_load;
            ex_mem_is_store    <= id_ex_is_store;
            ex_mem_mem_width   <= id_ex_mem_width;
            ex_mem_rf_we       <= id_ex_rf_we && id_ex_valid && !id_ex_is_muldiv;
                                  // muldiv rf_we set on done cycle
            ex_mem_wb_src      <= id_ex_wb_src;
            ex_mem_csr_rdata   <= csr_rdata;
            ex_mem_valid       <= id_ex_valid && !id_ex_is_muldiv;
        end
    end

    // Debug
    assign dbg_alu_a      = alu_op_a;
    assign dbg_alu_b      = alu_op_b;
    assign dbg_alu_result = alu_result;

endmodule

`default_nettype wire
