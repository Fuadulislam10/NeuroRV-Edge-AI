// =============================================================================
// NeuroRV Edge — Instruction Decode (ID) Stage
// File   : rtl/core/id_stage.sv
// =============================================================================

`timescale 1ns/1ps
`default_nettype none

// ALU operation encoding
`define ALU_ADD   4'b0000
`define ALU_SUB   4'b0001
`define ALU_AND   4'b0010
`define ALU_OR    4'b0011
`define ALU_XOR   4'b0100
`define ALU_SLL   4'b0101
`define ALU_SRL   4'b0110
`define ALU_SRA   4'b0111
`define ALU_SLT   4'b1000
`define ALU_SLTU  4'b1001
`define ALU_LUI   4'b1010
`define ALU_AUIPC 4'b1011
`define ALU_COPY_B 4'b1100

// RISC-V opcodes
`define OPC_LOAD    7'b000_0011
`define OPC_STORE   7'b010_0011
`define OPC_BRANCH  7'b110_0011
`define OPC_JAL     7'b110_1111
`define OPC_JALR    7'b110_0111
`define OPC_LUI     7'b011_0111
`define OPC_AUIPC   7'b001_0111
`define OPC_OP      7'b011_0011   // R-type
`define OPC_OP_IMM  7'b001_0011   // I-type ALU
`define OPC_SYSTEM  7'b111_0011   // SYSTEM (CSR, ECALL, EBREAK, MRET)
`define OPC_MISC_MEM 7'b000_1111  // FENCE

module id_stage (
    input  logic        clk_i,
    input  logic        rst_ni,
    input  logic        stall_i,
    input  logic        flush_i,
    // IF/ID inputs
    input  logic [31:0] if_id_pc_i,
    input  logic [31:0] if_id_instr_i,
    input  logic        if_id_valid_i,
    // Register file read ports
    input  logic [31:0] rf_rdata1_i,
    input  logic [31:0] rf_rdata2_i,
    // ID/EX pipeline register outputs
    output logic [31:0] id_ex_pc_o,
    output logic [31:0] id_ex_instr_o,
    output logic [31:0] id_ex_rs1_data_o,
    output logic [31:0] id_ex_rs2_data_o,
    output logic [31:0] id_ex_imm_o,
    output logic [4:0]  id_ex_rs1_addr_o,
    output logic [4:0]  id_ex_rs2_addr_o,
    output logic [4:0]  id_ex_rd_addr_o,
    output logic [3:0]  id_ex_alu_op_o,
    output logic [2:0]  id_ex_funct3_o,
    output logic        id_ex_alu_src_a_pc_o,
    output logic        id_ex_alu_src_b_imm_o,
    output logic        id_ex_reg_write_o,
    output logic        id_ex_mem_read_o,
    output logic        id_ex_mem_write_o,
    output logic        id_ex_branch_o,
    output logic        id_ex_jal_o,
    output logic        id_ex_jalr_o,
    output logic        id_ex_csr_op_o,
    output logic [11:0] id_ex_csr_addr_o,
    output logic        id_ex_muldiv_o,
    output logic [2:0]  id_ex_muldiv_op_o,
    output logic        id_ex_valid_o
);

    // Instruction fields
    logic [6:0]  opcode;
    logic [4:0]  rs1, rs2, rd;
    logic [2:0]  funct3;
    logic [6:0]  funct7;
    logic [11:0] csr_addr;

    // Immediate generation
    logic [31:0] imm_i, imm_s, imm_b, imm_u, imm_j;
    logic [31:0] imm_out;

    // Decode signals
    logic [3:0]  alu_op;
    logic        alu_src_a_pc;
    logic        alu_src_b_imm;
    logic        reg_write;
    logic        mem_read;
    logic        mem_write;
    logic        branch;
    logic        jal, jalr;
    logic        csr_op;
    logic        muldiv;
    logic [2:0]  muldiv_op;

    // -------------------------------------------------------------------------
    // Instruction decode
    // -------------------------------------------------------------------------
    assign opcode  = if_id_instr_i[6:0];
    assign rd      = if_id_instr_i[11:7];
    assign funct3  = if_id_instr_i[14:12];
    assign rs1     = if_id_instr_i[19:15];
    assign rs2     = if_id_instr_i[24:20];
    assign funct7  = if_id_instr_i[31:25];
    assign csr_addr = if_id_instr_i[31:20];

    // -------------------------------------------------------------------------
    // Immediate generation (sign-extended)
    // -------------------------------------------------------------------------
    assign imm_i = {{20{if_id_instr_i[31]}}, if_id_instr_i[31:20]};
    assign imm_s = {{20{if_id_instr_i[31]}}, if_id_instr_i[31:25], if_id_instr_i[11:7]};
    assign imm_b = {{19{if_id_instr_i[31]}}, if_id_instr_i[31], if_id_instr_i[7],
                    if_id_instr_i[30:25], if_id_instr_i[11:8], 1'b0};
    assign imm_u = {if_id_instr_i[31:12], 12'h0};
    assign imm_j = {{11{if_id_instr_i[31]}}, if_id_instr_i[31], if_id_instr_i[19:12],
                    if_id_instr_i[20], if_id_instr_i[30:21], 1'b0};

    // -------------------------------------------------------------------------
    // Main decoder
    // -------------------------------------------------------------------------
    always_comb begin
        // Defaults
        alu_op        = `ALU_ADD;
        alu_src_a_pc  = 1'b0;
        alu_src_b_imm = 1'b0;
        reg_write     = 1'b0;
        mem_read      = 1'b0;
        mem_write     = 1'b0;
        branch        = 1'b0;
        jal           = 1'b0;
        jalr          = 1'b0;
        csr_op        = 1'b0;
        muldiv        = 1'b0;
        muldiv_op     = 3'b000;
        imm_out       = imm_i;

        unique case (opcode)
            `OPC_OP_IMM: begin // I-type ALU
                reg_write     = 1'b1;
                alu_src_b_imm = 1'b1;
                imm_out       = imm_i;
                unique case (funct3)
                    3'b000: alu_op = `ALU_ADD;  // ADDI
                    3'b010: alu_op = `ALU_SLT;  // SLTI
                    3'b011: alu_op = `ALU_SLTU; // SLTIU
                    3'b100: alu_op = `ALU_XOR;  // XORI
                    3'b110: alu_op = `ALU_OR;   // ORI
                    3'b111: alu_op = `ALU_AND;  // ANDI
                    3'b001: alu_op = `ALU_SLL;  // SLLI
                    3'b101: alu_op = funct7[5] ? `ALU_SRA : `ALU_SRL; // SRAI/SRLI
                    default: alu_op = `ALU_ADD;
                endcase
            end

            `OPC_OP: begin // R-type
                reg_write = 1'b1;
                if (funct7 == 7'b000_0001) begin
                    // M-extension
                    muldiv    = 1'b1;
                    muldiv_op = funct3;
                end else begin
                    unique case (funct3)
                        3'b000: alu_op = funct7[5] ? `ALU_SUB : `ALU_ADD;
                        3'b001: alu_op = `ALU_SLL;
                        3'b010: alu_op = `ALU_SLT;
                        3'b011: alu_op = `ALU_SLTU;
                        3'b100: alu_op = `ALU_XOR;
                        3'b101: alu_op = funct7[5] ? `ALU_SRA : `ALU_SRL;
                        3'b110: alu_op = `ALU_OR;
                        3'b111: alu_op = `ALU_AND;
                        default: alu_op = `ALU_ADD;
                    endcase
                end
            end

            `OPC_LOAD: begin
                reg_write     = 1'b1;
                mem_read      = 1'b1;
                alu_src_b_imm = 1'b1;
                imm_out       = imm_i;
            end

            `OPC_STORE: begin
                mem_write     = 1'b1;
                alu_src_b_imm = 1'b1;
                imm_out       = imm_s;
            end

            `OPC_BRANCH: begin
                branch  = 1'b1;
                imm_out = imm_b;
            end

            `OPC_JAL: begin
                jal           = 1'b1;
                reg_write     = 1'b1;
                alu_src_a_pc  = 1'b1;
                alu_src_b_imm = 1'b1;
                imm_out       = imm_j;
            end

            `OPC_JALR: begin
                jalr          = 1'b1;
                reg_write     = 1'b1;
                alu_src_b_imm = 1'b1;
                imm_out       = imm_i;
            end

            `OPC_LUI: begin
                reg_write     = 1'b1;
                alu_op        = `ALU_LUI;
                alu_src_b_imm = 1'b1;
                imm_out       = imm_u;
            end

            `OPC_AUIPC: begin
                reg_write     = 1'b1;
                alu_op        = `ALU_ADD;
                alu_src_a_pc  = 1'b1;
                alu_src_b_imm = 1'b1;
                imm_out       = imm_u;
            end

            `OPC_SYSTEM: begin
                if (funct3 != 3'b000) begin
                    csr_op    = 1'b1;
                    reg_write = 1'b1; // CSR read
                end
                // ECALL/EBREAK/MRET handled in ex_stage trap logic
            end

            `OPC_MISC_MEM: begin
                // FENCE — treat as NOP in this implementation
                reg_write = 1'b0;
            end

            default: begin
                // Illegal instruction — trap in ex_stage
            end
        endcase
    end

    // -------------------------------------------------------------------------
    // Pipeline register
    // -------------------------------------------------------------------------
    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            id_ex_pc_o          <= '0;
            id_ex_instr_o       <= 32'h0000_0013;
            id_ex_rs1_data_o    <= '0;
            id_ex_rs2_data_o    <= '0;
            id_ex_imm_o         <= '0;
            id_ex_rs1_addr_o    <= '0;
            id_ex_rs2_addr_o    <= '0;
            id_ex_rd_addr_o     <= '0;
            id_ex_alu_op_o      <= `ALU_ADD;
            id_ex_funct3_o      <= '0;
            id_ex_alu_src_a_pc_o  <= '0;
            id_ex_alu_src_b_imm_o <= '0;
            id_ex_reg_write_o   <= '0;
            id_ex_mem_read_o    <= '0;
            id_ex_mem_write_o   <= '0;
            id_ex_branch_o      <= '0;
            id_ex_jal_o         <= '0;
            id_ex_jalr_o        <= '0;
            id_ex_csr_op_o      <= '0;
            id_ex_csr_addr_o    <= '0;
            id_ex_muldiv_o      <= '0;
            id_ex_muldiv_op_o   <= '0;
            id_ex_valid_o       <= '0;
        end else if (!stall_i) begin
            if (flush_i) begin
                // Insert NOP bubble
                id_ex_pc_o          <= '0;
                id_ex_instr_o       <= 32'h0000_0013;
                id_ex_rs1_data_o    <= '0;
                id_ex_rs2_data_o    <= '0;
                id_ex_imm_o         <= '0;
                id_ex_rs1_addr_o    <= '0;
                id_ex_rs2_addr_o    <= '0;
                id_ex_rd_addr_o     <= '0;
                id_ex_alu_op_o      <= `ALU_ADD;
                id_ex_funct3_o      <= '0;
                id_ex_alu_src_a_pc_o  <= '0;
                id_ex_alu_src_b_imm_o <= '0;
                id_ex_reg_write_o   <= '0;
                id_ex_mem_read_o    <= '0;
                id_ex_mem_write_o   <= '0;
                id_ex_branch_o      <= '0;
                id_ex_jal_o         <= '0;
                id_ex_jalr_o        <= '0;
                id_ex_csr_op_o      <= '0;
                id_ex_csr_addr_o    <= '0;
                id_ex_muldiv_o      <= '0;
                id_ex_muldiv_op_o   <= '0;
                id_ex_valid_o       <= '0;
            end else begin
                id_ex_pc_o          <= if_id_pc_i;
                id_ex_instr_o       <= if_id_instr_i;
                id_ex_rs1_data_o    <= rf_rdata1_i;
                id_ex_rs2_data_o    <= rf_rdata2_i;
                id_ex_imm_o         <= imm_out;
                id_ex_rs1_addr_o    <= rs1;
                id_ex_rs2_addr_o    <= rs2;
                id_ex_rd_addr_o     <= rd;
                id_ex_alu_op_o      <= alu_op;
                id_ex_funct3_o      <= funct3;
                id_ex_alu_src_a_pc_o  <= alu_src_a_pc;
                id_ex_alu_src_b_imm_o <= alu_src_b_imm;
                id_ex_reg_write_o   <= reg_write & if_id_valid_i;
                id_ex_mem_read_o    <= mem_read  & if_id_valid_i;
                id_ex_mem_write_o   <= mem_write & if_id_valid_i;
                id_ex_branch_o      <= branch    & if_id_valid_i;
                id_ex_jal_o         <= jal       & if_id_valid_i;
                id_ex_jalr_o        <= jalr      & if_id_valid_i;
                id_ex_csr_op_o      <= csr_op    & if_id_valid_i;
                id_ex_csr_addr_o    <= csr_addr;
                id_ex_muldiv_o      <= muldiv    & if_id_valid_i;
                id_ex_muldiv_op_o   <= muldiv_op;
                id_ex_valid_o       <= if_id_valid_i;
            end
        end
    end

endmodule

`default_nettype wire
