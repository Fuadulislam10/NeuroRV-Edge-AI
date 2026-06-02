// ============================================================================
// NeuroRV Edge — Instruction Decode Stage (ID)
// FILE: rtl/core/id_stage.sv
//
// Responsibilities:
//   • Instruction decode (opcode, funct3, funct7, register indices)
//   • Immediate value generation (I, S, B, U, J types)
//   • Register file read (rs1, rs2)
//   • Control signal generation passed to EX stage
//   • Hazard detection (load-use) → stall request
//   • ID/EX pipeline register
//
// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2024-2025 NeuroRV Edge Contributors
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

// ---- Opcode definitions (RV32I/M)
`define OPC_LOAD    7'b000_0011
`define OPC_STORE   7'b010_0011
`define OPC_BRANCH  7'b110_0011
`define OPC_JALR    7'b110_0111
`define OPC_JAL     7'b110_1111
`define OPC_AUIPC   7'b001_0111
`define OPC_LUI     7'b011_0111
`define OPC_OP_IMM  7'b001_0011
`define OPC_OP      7'b011_0011   // includes RV32M when funct7=0000001
`define OPC_SYSTEM  7'b111_0011
`define OPC_FENCE   7'b000_1111

// ---- ALU operation encoding
typedef enum logic [3:0] {
    ALU_ADD  = 4'h0,
    ALU_SUB  = 4'h1,
    ALU_AND  = 4'h2,
    ALU_OR   = 4'h3,
    ALU_XOR  = 4'h4,
    ALU_SLL  = 4'h5,
    ALU_SRL  = 4'h6,
    ALU_SRA  = 4'h7,
    ALU_SLT  = 4'h8,
    ALU_SLTU = 4'h9,
    ALU_COPY_B = 4'hA   // pass rs2/imm through (LUI/AUIPC)
} alu_op_e;

// ---- Branch type encoding
typedef enum logic [2:0] {
    BR_NONE = 3'b000,
    BR_BEQ  = 3'b001,
    BR_BNE  = 3'b010,
    BR_BLT  = 3'b011,
    BR_BGE  = 3'b100,
    BR_BLTU = 3'b101,
    BR_BGEU = 3'b110
} br_type_e;

// ---- Source select for ALU operands
typedef enum logic [1:0] {
    SRC_REG = 2'b00,   // from register file
    SRC_IMM = 2'b01,   // from immediate
    SRC_PC  = 2'b10    // from PC
} src_sel_e;

// ---- Writeback source
typedef enum logic [1:0] {
    WB_ALU  = 2'b00,   // ALU result
    WB_MEM  = 2'b01,   // memory load
    WB_PC4  = 2'b10,   // PC+4 (JAL/JALR)
    WB_CSR  = 2'b11    // CSR read value
} wb_src_e;

module id_stage (
    input  logic        clk,
    input  logic        rst_n,

    // From IF/ID
    input  logic [31:0] if_id_pc,
    input  logic [31:0] if_id_instr,
    input  logic        if_id_valid,

    // Stall / Flush
    input  logic        stall_id,
    input  logic        flush_id,

    // Register file interface
    output logic [4:0]  rf_rs1_addr,
    output logic [4:0]  rf_rs2_addr,
    input  logic [31:0] rf_rs1_data,
    input  logic [31:0] rf_rs2_data,

    // Load-use hazard stall request to pipeline controller
    output logic        load_use_stall,

    // EX stage info needed for load-use detection
    input  logic        ex_is_load,
    input  logic [4:0]  ex_rd_addr,

    // ID/EX pipeline register outputs
    output logic [31:0] id_ex_pc,
    output logic [31:0] id_ex_rs1_data,
    output logic [31:0] id_ex_rs2_data,
    output logic [31:0] id_ex_imm,
    output logic [4:0]  id_ex_rs1_addr,
    output logic [4:0]  id_ex_rs2_addr,
    output logic [4:0]  id_ex_rd_addr,
    output alu_op_e     id_ex_alu_op,
    output src_sel_e    id_ex_alu_src_a,
    output src_sel_e    id_ex_alu_src_b,
    output br_type_e    id_ex_br_type,
    output logic        id_ex_is_load,
    output logic        id_ex_is_store,
    output logic [2:0]  id_ex_mem_width,   // funct3 for load/store
    output logic        id_ex_rf_we,
    output wb_src_e     id_ex_wb_src,
    output logic        id_ex_is_muldiv,
    output logic [2:0]  id_ex_muldiv_op,
    output logic        id_ex_csr_en,
    output logic [11:0] id_ex_csr_addr,
    output logic [2:0]  id_ex_csr_op,
    output logic        id_ex_mret,
    output logic        id_ex_ecall,
    output logic        id_ex_ebreak,
    output logic        id_ex_valid,

    // Debug
    output logic [6:0]  dbg_opcode,
    output logic [2:0]  dbg_funct3
);

    // -----------------------------------------------------------------------
    // Instruction field extraction
    // -----------------------------------------------------------------------
    logic [6:0] opcode;
    logic [4:0] rd, rs1, rs2;
    logic [2:0] funct3;
    logic [6:0] funct7;

    assign opcode = if_id_instr[6:0];
    assign rd     = if_id_instr[11:7];
    assign funct3 = if_id_instr[14:12];
    assign rs1    = if_id_instr[19:15];
    assign rs2    = if_id_instr[24:20];
    assign funct7 = if_id_instr[31:25];

    // Register file read addresses
    assign rf_rs1_addr = rs1;
    assign rf_rs2_addr = rs2;

    // -----------------------------------------------------------------------
    // Immediate generation
    // -----------------------------------------------------------------------
    logic [31:0] imm_i, imm_s, imm_b, imm_u, imm_j;
    logic [31:0] imm_mux;

    // I-type: [31:20]
    assign imm_i = {{20{if_id_instr[31]}}, if_id_instr[31:20]};

    // S-type: [31:25], [11:7]
    assign imm_s = {{20{if_id_instr[31]}},
                    if_id_instr[31:25], if_id_instr[11:7]};

    // B-type: [31], [7], [30:25], [11:8]
    assign imm_b = {{19{if_id_instr[31]}},
                    if_id_instr[31], if_id_instr[7],
                    if_id_instr[30:25], if_id_instr[11:8], 1'b0};

    // U-type: [31:12]
    assign imm_u = {if_id_instr[31:12], 12'b0};

    // J-type: [31], [19:12], [20], [30:21]
    assign imm_j = {{11{if_id_instr[31]}},
                    if_id_instr[31], if_id_instr[19:12],
                    if_id_instr[20], if_id_instr[30:21], 1'b0};

    // CSR zimm (unsigned 5-bit)
    logic [31:0] imm_csr_z;
    assign imm_csr_z = {27'b0, rs1};   // zimm field = rs1 field

    // -----------------------------------------------------------------------
    // Decode: control signal generation
    // -----------------------------------------------------------------------
    alu_op_e  dec_alu_op;
    src_sel_e dec_alu_src_a, dec_alu_src_b;
    br_type_e dec_br_type;
    logic     dec_is_load, dec_is_store;
    logic     dec_rf_we;
    wb_src_e  dec_wb_src;
    logic     dec_is_muldiv;
    logic [2:0] dec_muldiv_op;
    logic     dec_csr_en;
    logic [11:0] dec_csr_addr;
    logic [2:0]  dec_csr_op;
    logic     dec_mret, dec_ecall, dec_ebreak;

    always_comb begin
        // Defaults
        dec_alu_op    = ALU_ADD;
        dec_alu_src_a = SRC_REG;
        dec_alu_src_b = SRC_REG;
        dec_br_type   = BR_NONE;
        dec_is_load   = 1'b0;
        dec_is_store  = 1'b0;
        dec_rf_we     = 1'b0;
        dec_wb_src    = WB_ALU;
        dec_is_muldiv = 1'b0;
        dec_muldiv_op = 3'b000;
        dec_csr_en    = 1'b0;
        dec_csr_addr  = 12'b0;
        dec_csr_op    = 3'b000;
        dec_mret      = 1'b0;
        dec_ecall     = 1'b0;
        dec_ebreak    = 1'b0;
        imm_mux       = imm_i;

        unique case (opcode)
            // ---- R-type (OP): ALU reg-reg
            `OPC_OP: begin
                dec_alu_src_a = SRC_REG;
                dec_alu_src_b = SRC_REG;
                dec_rf_we     = 1'b1;
                dec_wb_src    = WB_ALU;

                if (funct7 == 7'b000_0001) begin
                    // RV32M
                    dec_is_muldiv = 1'b1;
                    dec_muldiv_op = funct3;
                end else begin
                    unique case ({funct7[5], funct3})
                        4'b0000: dec_alu_op = ALU_ADD;
                        4'b1000: dec_alu_op = ALU_SUB;
                        4'b0001: dec_alu_op = ALU_SLL;
                        4'b0010: dec_alu_op = ALU_SLT;
                        4'b0011: dec_alu_op = ALU_SLTU;
                        4'b0100: dec_alu_op = ALU_XOR;
                        4'b0101: dec_alu_op = ALU_SRL;
                        4'b1101: dec_alu_op = ALU_SRA;
                        4'b0110: dec_alu_op = ALU_OR;
                        4'b0111: dec_alu_op = ALU_AND;
                        default: dec_alu_op = ALU_ADD;
                    endcase
                end
            end

            // ---- I-type (OP-IMM): ALU reg-imm
            `OPC_OP_IMM: begin
                dec_alu_src_a = SRC_REG;
                dec_alu_src_b = SRC_IMM;
                dec_rf_we     = 1'b1;
                dec_wb_src    = WB_ALU;
                imm_mux       = imm_i;

                unique case (funct3)
                    3'b000: dec_alu_op = ALU_ADD;
                    3'b010: dec_alu_op = ALU_SLT;
                    3'b011: dec_alu_op = ALU_SLTU;
                    3'b100: dec_alu_op = ALU_XOR;
                    3'b110: dec_alu_op = ALU_OR;
                    3'b111: dec_alu_op = ALU_AND;
                    3'b001: dec_alu_op = ALU_SLL;
                    3'b101: dec_alu_op = (funct7[5]) ? ALU_SRA : ALU_SRL;
                    default: dec_alu_op = ALU_ADD;
                endcase
            end

            // ---- LOAD
            `OPC_LOAD: begin
                dec_alu_src_a = SRC_REG;
                dec_alu_src_b = SRC_IMM;
                dec_alu_op    = ALU_ADD;
                dec_is_load   = 1'b1;
                dec_rf_we     = 1'b1;
                dec_wb_src    = WB_MEM;
                imm_mux       = imm_i;
            end

            // ---- STORE
            `OPC_STORE: begin
                dec_alu_src_a = SRC_REG;
                dec_alu_src_b = SRC_IMM;
                dec_alu_op    = ALU_ADD;
                dec_is_store  = 1'b1;
                dec_rf_we     = 1'b0;
                imm_mux       = imm_s;
            end

            // ---- BRANCH
            `OPC_BRANCH: begin
                dec_alu_src_a = SRC_PC;
                dec_alu_src_b = SRC_IMM;
                dec_alu_op    = ALU_ADD;   // compute branch target
                dec_rf_we     = 1'b0;
                imm_mux       = imm_b;
                unique case (funct3)
                    3'b000: dec_br_type = BR_BEQ;
                    3'b001: dec_br_type = BR_BNE;
                    3'b100: dec_br_type = BR_BLT;
                    3'b101: dec_br_type = BR_BGE;
                    3'b110: dec_br_type = BR_BLTU;
                    3'b111: dec_br_type = BR_BGEU;
                    default: dec_br_type = BR_NONE;
                endcase
            end

            // ---- JAL
            `OPC_JAL: begin
                dec_alu_src_a = SRC_PC;
                dec_alu_src_b = SRC_IMM;
                dec_alu_op    = ALU_ADD;   // target = PC + imm_j
                dec_rf_we     = 1'b1;
                dec_wb_src    = WB_PC4;    // rd ← PC+4
                imm_mux       = imm_j;
                dec_br_type   = BR_NONE;   // unconditional, handled in EX
            end

            // ---- JALR
            `OPC_JALR: begin
                dec_alu_src_a = SRC_REG;
                dec_alu_src_b = SRC_IMM;
                dec_alu_op    = ALU_ADD;   // target = rs1 + imm_i
                dec_rf_we     = 1'b1;
                dec_wb_src    = WB_PC4;
                imm_mux       = imm_i;
            end

            // ---- LUI
            `OPC_LUI: begin
                dec_alu_src_a = SRC_REG;   // rs1 not used
                dec_alu_src_b = SRC_IMM;
                dec_alu_op    = ALU_COPY_B;
                dec_rf_we     = 1'b1;
                dec_wb_src    = WB_ALU;
                imm_mux       = imm_u;
            end

            // ---- AUIPC
            `OPC_AUIPC: begin
                dec_alu_src_a = SRC_PC;
                dec_alu_src_b = SRC_IMM;
                dec_alu_op    = ALU_ADD;
                dec_rf_we     = 1'b1;
                dec_wb_src    = WB_ALU;
                imm_mux       = imm_u;
            end

            // ---- SYSTEM (CSR, ECALL, EBREAK, MRET)
            `OPC_SYSTEM: begin
                dec_csr_addr  = if_id_instr[31:20];
                dec_csr_op    = funct3;

                if (funct3 == 3'b000) begin
                    // PRIV: ECALL, EBREAK, MRET
                    unique case (if_id_instr[31:20])
                        12'h000: dec_ecall  = 1'b1;
                        12'h001: dec_ebreak = 1'b1;
                        12'h302: dec_mret   = 1'b1;
                        default: ;
                    endcase
                end else begin
                    // CSR instruction
                    dec_csr_en = 1'b1;
                    dec_rf_we  = 1'b1;
                    dec_wb_src = WB_CSR;
                    // For CSRxI variants use zero-extended rs1 as immediate
                    imm_mux = (funct3[2]) ? imm_csr_z : imm_i;
                end
            end

            // ---- FENCE (treated as NOP)
            `OPC_FENCE: begin
                dec_rf_we = 1'b0;
            end

            default: begin
                dec_rf_we = 1'b0;
            end
        endcase
    end

    // -----------------------------------------------------------------------
    // Load-use hazard detection
    // Stall if EX stage is a load and its rd matches our rs1 or rs2
    // -----------------------------------------------------------------------
    assign load_use_stall = ex_is_load && (ex_rd_addr != '0) &&
                            ((ex_rd_addr == rs1) || (ex_rd_addr == rs2)) &&
                            if_id_valid;

    // -----------------------------------------------------------------------
    // ID/EX Pipeline Register
    // -----------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n || flush_id) begin
            id_ex_pc         <= '0;
            id_ex_rs1_data   <= '0;
            id_ex_rs2_data   <= '0;
            id_ex_imm        <= '0;
            id_ex_rs1_addr   <= '0;
            id_ex_rs2_addr   <= '0;
            id_ex_rd_addr    <= '0;
            id_ex_alu_op     <= ALU_ADD;
            id_ex_alu_src_a  <= SRC_REG;
            id_ex_alu_src_b  <= SRC_REG;
            id_ex_br_type    <= BR_NONE;
            id_ex_is_load    <= 1'b0;
            id_ex_is_store   <= 1'b0;
            id_ex_mem_width  <= '0;
            id_ex_rf_we      <= 1'b0;
            id_ex_wb_src     <= WB_ALU;
            id_ex_is_muldiv  <= 1'b0;
            id_ex_muldiv_op  <= '0;
            id_ex_csr_en     <= 1'b0;
            id_ex_csr_addr   <= '0;
            id_ex_csr_op     <= '0;
            id_ex_mret       <= 1'b0;
            id_ex_ecall      <= 1'b0;
            id_ex_ebreak     <= 1'b0;
            id_ex_valid      <= 1'b0;
        end else if (!stall_id) begin
            id_ex_pc         <= if_id_pc;
            id_ex_rs1_data   <= rf_rs1_data;
            id_ex_rs2_data   <= rf_rs2_data;
            id_ex_imm        <= imm_mux;
            id_ex_rs1_addr   <= rs1;
            id_ex_rs2_addr   <= rs2;
            id_ex_rd_addr    <= rd;
            id_ex_alu_op     <= dec_alu_op;
            id_ex_alu_src_a  <= dec_alu_src_a;
            id_ex_alu_src_b  <= dec_alu_src_b;
            id_ex_br_type    <= dec_br_type;
            id_ex_is_load    <= dec_is_load;
            id_ex_is_store   <= dec_is_store;
            id_ex_mem_width  <= funct3;
            id_ex_rf_we      <= dec_rf_we & if_id_valid;
            id_ex_wb_src     <= dec_wb_src;
            id_ex_is_muldiv  <= dec_is_muldiv;
            id_ex_muldiv_op  <= dec_muldiv_op;
            id_ex_csr_en     <= dec_csr_en & if_id_valid;
            id_ex_csr_addr   <= dec_csr_addr;
            id_ex_csr_op     <= dec_csr_op;
            id_ex_mret       <= dec_mret & if_id_valid;
            id_ex_ecall      <= dec_ecall & if_id_valid;
            id_ex_ebreak     <= dec_ebreak & if_id_valid;
            id_ex_valid      <= if_id_valid;
        end
        // else: stall — hold all values
    end

    // Debug
    assign dbg_opcode = opcode;
    assign dbg_funct3 = funct3;

endmodule

`default_nettype wire
