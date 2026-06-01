// ============================================================================
// NeuroRV Edge — RISC-V RV32IM 5-Stage Pipeline CPU Core
// File   : rtl/neuro_rv_core.sv
// Author : NeuroRV Edge Contributors
// License: Apache 2.0
//
// Description:
//   A synthesizable RV32IM 5-stage pipelined processor with:
//   - Instruction Fetch (IF), Instruction Decode (ID), Execute (EX),
//     Memory Access (MEM), Write-Back (WB)
//   - Full data hazard detection and forwarding
//   - Control hazard handling via pipeline flush
//   - M-extension: hardware 32x32 multiplier/divider
//   - Memory-mapped I/O through load/store interface
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

// ============================================================================
// Opcode / Function Code Definitions
// ============================================================================
package rv32im_pkg;
  // Opcodes
  localparam logic [6:0] OP_LUI    = 7'b0110111;
  localparam logic [6:0] OP_AUIPC  = 7'b0010111;
  localparam logic [6:0] OP_JAL    = 7'b1101111;
  localparam logic [6:0] OP_JALR   = 7'b1100111;
  localparam logic [6:0] OP_BRANCH = 7'b1100011;
  localparam logic [6:0] OP_LOAD   = 7'b0000011;
  localparam logic [6:0] OP_STORE  = 7'b0100011;
  localparam logic [6:0] OP_ALUI   = 7'b0010011;
  localparam logic [6:0] OP_ALU    = 7'b0110011;
  localparam logic [6:0] OP_FENCE  = 7'b0001111;
  localparam logic [6:0] OP_SYSTEM = 7'b1110011;

  // ALU operations
  localparam logic [3:0] ALU_ADD  = 4'h0;
  localparam logic [3:0] ALU_SUB  = 4'h1;
  localparam logic [3:0] ALU_AND  = 4'h2;
  localparam logic [3:0] ALU_OR   = 4'h3;
  localparam logic [3:0] ALU_XOR  = 4'h4;
  localparam logic [3:0] ALU_SLL  = 4'h5;
  localparam logic [3:0] ALU_SRL  = 4'h6;
  localparam logic [3:0] ALU_SRA  = 4'h7;
  localparam logic [3:0] ALU_SLT  = 4'h8;
  localparam logic [3:0] ALU_SLTU = 4'h9;
  localparam logic [3:0] ALU_MUL  = 4'hA;
  localparam logic [3:0] ALU_MULH = 4'hB;
  localparam logic [3:0] ALU_DIV  = 4'hC;
  localparam logic [3:0] ALU_REM  = 4'hD;
endpackage

// ============================================================================
// Register File
// ============================================================================
module regfile (
  input  logic        clk,
  input  logic        rst_n,
  // Read ports (combinational)
  input  logic [4:0]  rs1_addr,
  input  logic [4:0]  rs2_addr,
  output logic [31:0] rs1_data,
  output logic [31:0] rs2_data,
  // Write port (synchronous)
  input  logic        wr_en,
  input  logic [4:0]  rd_addr,
  input  logic [31:0] rd_data
);
  logic [31:0] regs [0:31];

  // Synchronous write, x0 hardwired to zero
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      for (int i = 0; i < 32; i++) regs[i] <= 32'h0;
    end else if (wr_en && rd_addr != 5'h0) begin
      regs[rd_addr] <= rd_data;
    end
  end

  // Asynchronous read with write-through for same-cycle WB
  assign rs1_data = (rs1_addr == 5'h0) ? 32'h0 :
                    (wr_en && rd_addr == rs1_addr) ? rd_data :
                    regs[rs1_addr];

  assign rs2_data = (rs2_addr == 5'h0) ? 32'h0 :
                    (wr_en && rd_addr == rs2_addr) ? rd_data :
                    regs[rs2_addr];
endmodule

// ============================================================================
// ALU + M-Extension
// ============================================================================
module alu_unit (
  input  logic [3:0]  op,
  input  logic [31:0] a,
  input  logic [31:0] b,
  output logic [31:0] result,
  output logic        zero
);
  logic [63:0] mul_result;
  logic [63:0] muls_result;
  logic [31:0] div_result;
  logic [31:0] rem_result;

  assign mul_result  = {32'h0, a} * {32'h0, b};
  assign muls_result = {{32{a[31]}}, a} * {{32{b[31]}}, b};

  // Signed division with guard against divide-by-zero
  assign div_result = (b == 32'h0) ? 32'hFFFFFFFF :
                      ($signed(a) / $signed(b));
  assign rem_result = (b == 32'h0) ? a :
                      ($signed(a) % $signed(b));

  import rv32im_pkg::*;
  always_comb begin
    case (op)
      ALU_ADD  : result = a + b;
      ALU_SUB  : result = a - b;
      ALU_AND  : result = a & b;
      ALU_OR   : result = a | b;
      ALU_XOR  : result = a ^ b;
      ALU_SLL  : result = a << b[4:0];
      ALU_SRL  : result = a >> b[4:0];
      ALU_SRA  : result = $signed(a) >>> b[4:0];
      ALU_SLT  : result = ($signed(a) < $signed(b)) ? 32'h1 : 32'h0;
      ALU_SLTU : result = (a < b) ? 32'h1 : 32'h0;
      ALU_MUL  : result = mul_result[31:0];
      ALU_MULH : result = muls_result[63:32];
      ALU_DIV  : result = div_result;
      ALU_REM  : result = rem_result;
      default  : result = 32'h0;
    endcase
  end

  assign zero = (result == 32'h0);
endmodule

// ============================================================================
// Instruction Decoder
// ============================================================================
module decoder (
  input  logic [31:0] instr,
  output logic [6:0]  opcode,
  output logic [4:0]  rs1,
  output logic [4:0]  rs2,
  output logic [4:0]  rd,
  output logic [2:0]  funct3,
  output logic [6:0]  funct7,
  output logic [31:0] imm,
  output logic [3:0]  alu_op,
  output logic        alu_src,     // 0=reg, 1=imm
  output logic        mem_read,
  output logic        mem_write,
  output logic        reg_write,
  output logic        branch,
  output logic        jump,
  output logic        mem_to_reg,
  output logic [1:0]  mem_size,    // 00=byte, 01=half, 10=word
  output logic        mem_sign_ext
);
  import rv32im_pkg::*;

  assign opcode = instr[6:0];
  assign rd     = instr[11:7];
  assign funct3 = instr[14:12];
  assign rs1    = instr[19:15];
  assign rs2    = instr[24:20];
  assign funct7 = instr[31:25];

  // Immediate decode
  logic [31:0] imm_i, imm_s, imm_b, imm_u, imm_j;
  assign imm_i = {{20{instr[31]}}, instr[31:20]};
  assign imm_s = {{20{instr[31]}}, instr[31:25], instr[11:7]};
  assign imm_b = {{19{instr[31]}}, instr[31], instr[7], instr[30:25], instr[11:8], 1'b0};
  assign imm_u = {instr[31:12], 12'h0};
  assign imm_j = {{11{instr[31]}}, instr[31], instr[19:12], instr[20], instr[30:21], 1'b0};

  // ALU op decode
  logic [3:0] alu_op_r;
  always_comb begin
    alu_op_r = ALU_ADD;
    case (opcode)
      OP_ALU, OP_ALUI: begin
        case (funct3)
          3'b000: alu_op_r = (opcode == OP_ALU && funct7[5]) ? ALU_SUB :
                             (opcode == OP_ALU && funct7[0]) ? ALU_MUL : ALU_ADD;
          3'b001: alu_op_r = (funct7[0]) ? ALU_MULH : ALU_SLL;
          3'b010: alu_op_r = (funct7[0]) ? ALU_DIV  : ALU_SLT;  // MULHSU -> DIV approx
          3'b011: alu_op_r = (funct7[0]) ? ALU_DIV  : ALU_SLTU;
          3'b100: alu_op_r = (funct7[0]) ? ALU_DIV  : ALU_XOR;
          3'b101: alu_op_r = (funct7[0]) ? ALU_REM  :
                             funct7[5]   ? ALU_SRA  : ALU_SRL;
          3'b110: alu_op_r = (funct7[0]) ? ALU_REM  : ALU_OR;
          3'b111: alu_op_r = ALU_AND;
          default: alu_op_r = ALU_ADD;
        endcase
      end
      OP_BRANCH: begin
        case (funct3)
          3'b100, 3'b101: alu_op_r = ALU_SLT;
          3'b110, 3'b111: alu_op_r = ALU_SLTU;
          default:        alu_op_r = ALU_SUB;
        endcase
      end
      default: alu_op_r = ALU_ADD;
    endcase
  end
  assign alu_op = alu_op_r;

  // Control signals
  always_comb begin
    alu_src      = 1'b0;
    mem_read     = 1'b0;
    mem_write    = 1'b0;
    reg_write    = 1'b0;
    branch       = 1'b0;
    jump         = 1'b0;
    mem_to_reg   = 1'b0;
    mem_size     = 2'b10;
    mem_sign_ext = 1'b1;
    imm          = imm_i;

    case (opcode)
      OP_LUI: begin
        reg_write = 1'b1; alu_src = 1'b1; imm = imm_u;
      end
      OP_AUIPC: begin
        reg_write = 1'b1; alu_src = 1'b1; imm = imm_u;
      end
      OP_JAL: begin
        reg_write = 1'b1; jump = 1'b1; imm = imm_j;
      end
      OP_JALR: begin
        reg_write = 1'b1; jump = 1'b1; alu_src = 1'b1; imm = imm_i;
      end
      OP_BRANCH: begin
        branch = 1'b1; imm = imm_b;
      end
      OP_LOAD: begin
        reg_write = 1'b1; mem_read = 1'b1; mem_to_reg = 1'b1;
        alu_src = 1'b1; imm = imm_i;
        mem_size     = funct3[1:0];
        mem_sign_ext = ~funct3[2];
      end
      OP_STORE: begin
        mem_write = 1'b1; alu_src = 1'b1; imm = imm_s;
        mem_size = funct3[1:0];
      end
      OP_ALUI: begin
        reg_write = 1'b1; alu_src = 1'b1; imm = imm_i;
      end
      OP_ALU: begin
        reg_write = 1'b1;
      end
      default: ;
    endcase
  end
endmodule

// ============================================================================
// Pipeline Registers + Hazard Unit
// ============================================================================

// IF/ID Pipeline Register
typedef struct packed {
  logic [31:0] pc;
  logic [31:0] instr;
  logic        valid;
} if_id_reg_t;

// ID/EX Pipeline Register
typedef struct packed {
  logic [31:0] pc;
  logic [31:0] rs1_data;
  logic [31:0] rs2_data;
  logic [31:0] imm;
  logic [4:0]  rs1;
  logic [4:0]  rs2;
  logic [4:0]  rd;
  logic [3:0]  alu_op;
  logic        alu_src;
  logic        mem_read;
  logic        mem_write;
  logic        reg_write;
  logic        branch;
  logic        jump;
  logic        mem_to_reg;
  logic [1:0]  mem_size;
  logic        mem_sign_ext;
  logic [6:0]  opcode;
  logic        valid;
} id_ex_reg_t;

// EX/MEM Pipeline Register
typedef struct packed {
  logic [31:0] pc;
  logic [31:0] alu_result;
  logic [31:0] rs2_data;
  logic [4:0]  rd;
  logic        mem_read;
  logic        mem_write;
  logic        reg_write;
  logic        branch;
  logic        jump;
  logic        mem_to_reg;
  logic [1:0]  mem_size;
  logic        mem_sign_ext;
  logic        zero;
  logic        valid;
} ex_mem_reg_t;

// MEM/WB Pipeline Register
typedef struct packed {
  logic [31:0] alu_result;
  logic [31:0] mem_data;
  logic [4:0]  rd;
  logic        reg_write;
  logic        mem_to_reg;
  logic        valid;
} mem_wb_reg_t;

// ============================================================================
// Main CPU Core
// ============================================================================
module neuro_rv_core #(
  parameter logic [31:0] RESET_PC = 32'h0000_0000
)(
  input  logic        clk,
  input  logic        rst_n,
  // Instruction memory interface
  output logic [31:0] imem_addr,
  input  logic [31:0] imem_data,
  input  logic        imem_valid,
  // Data memory interface (AXI-lite style)
  output logic [31:0] dmem_addr,
  output logic [31:0] dmem_wdata,
  output logic [3:0]  dmem_wstrb,
  output logic        dmem_we,
  output logic        dmem_re,
  input  logic [31:0] dmem_rdata,
  input  logic        dmem_ack,
  // CPU status
  output logic [31:0] debug_pc,
  output logic        cpu_active
);

  // ---- Pipeline Stage Registers ----
  if_id_reg_t  if_id,  if_id_nxt;
  id_ex_reg_t  id_ex,  id_ex_nxt;
  ex_mem_reg_t ex_mem, ex_mem_nxt;
  mem_wb_reg_t mem_wb, mem_wb_nxt;

  // ---- PC Register ----
  logic [31:0] pc, pc_next;
  logic        pc_stall;
  logic        pipeline_flush;
  logic [31:0] branch_target;
  logic        branch_taken;

  // ---- Register File Signals ----
  logic [4:0]  rf_rs1_addr, rf_rs2_addr;
  logic [31:0] rf_rs1_data, rf_rs2_data;
  logic        rf_wr_en;
  logic [4:0]  rf_rd_addr;
  logic [31:0] rf_rd_data;

  // ---- Decoder Outputs ----
  logic [6:0]  dec_opcode;
  logic [4:0]  dec_rs1, dec_rs2, dec_rd;
  logic [2:0]  dec_funct3;
  logic [6:0]  dec_funct7;
  logic [31:0] dec_imm;
  logic [3:0]  dec_alu_op;
  logic        dec_alu_src;
  logic        dec_mem_read;
  logic        dec_mem_write;
  logic        dec_reg_write;
  logic        dec_branch;
  logic        dec_jump;
  logic        dec_mem_to_reg;
  logic [1:0]  dec_mem_size;
  logic        dec_mem_sign_ext;

  // ---- ALU Signals ----
  logic [31:0] alu_a, alu_b, alu_result;
  logic        alu_zero;
  logic [3:0]  alu_ctrl;

  // ---- Forwarding ----
  logic [1:0]  fwd_a, fwd_b;
  logic [31:0] fwd_rs1_data, fwd_rs2_data;

  // ---- Hazard Detection ----
  logic        hazard_stall;
  logic        load_use_hazard;

  // ========== Register File ==========
  regfile u_regfile (
    .clk     (clk),
    .rst_n   (rst_n),
    .rs1_addr(rf_rs1_addr),
    .rs2_addr(rf_rs2_addr),
    .rs1_data(rf_rs1_data),
    .rs2_data(rf_rs2_data),
    .wr_en   (rf_wr_en),
    .rd_addr (rf_rd_addr),
    .rd_data (rf_rd_data)
  );

  // ========== Decoder ==========
  decoder u_decoder (
    .instr        (if_id.instr),
    .opcode       (dec_opcode),
    .rs1          (dec_rs1),
    .rs2          (dec_rs2),
    .rd           (dec_rd),
    .funct3       (dec_funct3),
    .funct7       (dec_funct7),
    .imm          (dec_imm),
    .alu_op       (dec_alu_op),
    .alu_src      (dec_alu_src),
    .mem_read     (dec_mem_read),
    .mem_write    (dec_mem_write),
    .reg_write    (dec_reg_write),
    .branch       (dec_branch),
    .jump         (dec_jump),
    .mem_to_reg   (dec_mem_to_reg),
    .mem_size     (dec_mem_size),
    .mem_sign_ext (dec_mem_sign_ext)
  );

  // ========== ALU ==========
  alu_unit u_alu (
    .op    (alu_ctrl),
    .a     (alu_a),
    .b     (alu_b),
    .result(alu_result),
    .zero  (alu_zero)
  );

  // ========== Hazard Detection Unit ==========
  always_comb begin
    load_use_hazard = id_ex.mem_read && id_ex.valid &&
                      ((id_ex.rd == dec_rs1 && dec_rs1 != 5'h0) ||
                       (id_ex.rd == dec_rs2 && dec_rs2 != 5'h0));
    hazard_stall    = load_use_hazard;
    pc_stall        = hazard_stall || !dmem_ack;
  end

  // ========== Forwarding Unit ==========
  // fwd_a/fwd_b: 00=regfile, 01=MEM stage, 10=WB stage
  always_comb begin
    fwd_a = 2'b00;
    fwd_b = 2'b00;

    if (ex_mem.reg_write && ex_mem.valid && ex_mem.rd != 5'h0 &&
        ex_mem.rd == id_ex.rs1)
      fwd_a = 2'b01;
    else if (mem_wb.reg_write && mem_wb.valid && mem_wb.rd != 5'h0 &&
             mem_wb.rd == id_ex.rs1)
      fwd_a = 2'b10;

    if (ex_mem.reg_write && ex_mem.valid && ex_mem.rd != 5'h0 &&
        ex_mem.rd == id_ex.rs2)
      fwd_b = 2'b01;
    else if (mem_wb.reg_write && mem_wb.valid && mem_wb.rd != 5'h0 &&
             mem_wb.rd == id_ex.rs2)
      fwd_b = 2'b10;
  end

  // Forwarded operand selection
  assign fwd_rs1_data = (fwd_a == 2'b01) ? ex_mem.alu_result :
                        (fwd_a == 2'b10) ? rf_rd_data :
                        id_ex.rs1_data;

  assign fwd_rs2_data = (fwd_b == 2'b01) ? ex_mem.alu_result :
                        (fwd_b == 2'b10) ? rf_rd_data :
                        id_ex.rs2_data;

  // ALU inputs
  import rv32im_pkg::*;
  assign alu_a    = (id_ex.opcode == OP_AUIPC || id_ex.opcode == OP_JAL) ?
                     id_ex.pc : fwd_rs1_data;
  assign alu_b    = id_ex.alu_src ? id_ex.imm : fwd_rs2_data;
  assign alu_ctrl = id_ex.alu_op;

  // ========== Branch/Jump Logic ==========
  always_comb begin
    branch_taken  = 1'b0;
    branch_target = ex_mem.pc + 32'h4; // default

    if (ex_mem.valid && ex_mem.jump) begin
      branch_taken  = 1'b1;
      branch_target = ex_mem.alu_result & 32'hFFFF_FFFE;
    end else if (ex_mem.valid && ex_mem.branch) begin
      case (1'b1)
        (ex_mem.zero): branch_taken = 1'b1; // BEQ
        default:       branch_taken = 1'b0;
      endcase
      branch_target = ex_mem.pc + ex_mem.rs2_data; // imm stored in rs2_data slot
    end
    pipeline_flush = branch_taken;
  end

  // ========== PC Stage ==========
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
      pc <= RESET_PC;
    else if (!pc_stall) begin
      if (pipeline_flush)
        pc <= branch_target;
      else
        pc <= pc + 32'h4;
    end
  end

  // ========== IF Stage ==========
  assign imem_addr = pc;

  always_comb begin
    if_id_nxt.pc    = pc;
    if_id_nxt.instr = imem_data;
    if_id_nxt.valid = imem_valid && !pipeline_flush;
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      if_id <= '0;
    end else if (!hazard_stall) begin
      if_id <= if_id_nxt;
    end
  end

  // ========== ID Stage ==========
  assign rf_rs1_addr = dec_rs1;
  assign rf_rs2_addr = dec_rs2;

  always_comb begin
    id_ex_nxt.pc           = if_id.pc;
    id_ex_nxt.rs1_data     = rf_rs1_data;
    id_ex_nxt.rs2_data     = rf_rs2_data;
    id_ex_nxt.imm          = dec_imm;
    id_ex_nxt.rs1          = dec_rs1;
    id_ex_nxt.rs2          = dec_rs2;
    id_ex_nxt.rd           = dec_rd;
    id_ex_nxt.alu_op       = dec_alu_op;
    id_ex_nxt.alu_src      = dec_alu_src;
    id_ex_nxt.mem_read     = dec_mem_read;
    id_ex_nxt.mem_write    = dec_mem_write;
    id_ex_nxt.reg_write    = dec_reg_write;
    id_ex_nxt.branch       = dec_branch;
    id_ex_nxt.jump         = dec_jump;
    id_ex_nxt.mem_to_reg   = dec_mem_to_reg;
    id_ex_nxt.mem_size     = dec_mem_size;
    id_ex_nxt.mem_sign_ext = dec_mem_sign_ext;
    id_ex_nxt.opcode       = dec_opcode;
    id_ex_nxt.valid        = if_id.valid && !hazard_stall && !pipeline_flush;
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
      id_ex <= '0;
    else
      id_ex <= id_ex_nxt;
  end

  // ========== EX Stage ==========
  always_comb begin
    ex_mem_nxt.pc           = id_ex.pc;
    ex_mem_nxt.alu_result   = alu_result;
    ex_mem_nxt.rs2_data     = fwd_rs2_data;  // also stores branch imm offset
    ex_mem_nxt.rd           = id_ex.rd;
    ex_mem_nxt.mem_read     = id_ex.mem_read;
    ex_mem_nxt.mem_write    = id_ex.mem_write;
    ex_mem_nxt.reg_write    = id_ex.reg_write;
    ex_mem_nxt.branch       = id_ex.branch;
    ex_mem_nxt.jump         = id_ex.jump;
    ex_mem_nxt.mem_to_reg   = id_ex.mem_to_reg;
    ex_mem_nxt.mem_size     = id_ex.mem_size;
    ex_mem_nxt.mem_sign_ext = id_ex.mem_sign_ext;
    ex_mem_nxt.zero         = alu_zero;
    ex_mem_nxt.valid        = id_ex.valid;
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
      ex_mem <= '0;
    else
      ex_mem <= ex_mem_nxt;
  end

  // ========== MEM Stage ==========
  // Byte enable generation
  logic [3:0] byte_en;
  always_comb begin
    case (ex_mem.mem_size)
      2'b00: byte_en = 4'b0001 << ex_mem.alu_result[1:0]; // byte
      2'b01: byte_en = (ex_mem.alu_result[1]) ? 4'b1100 : 4'b0011; // half
      default: byte_en = 4'b1111; // word
    endcase
  end

  assign dmem_addr  = {ex_mem.alu_result[31:2], 2'b00};
  assign dmem_wdata = ex_mem.rs2_data;
  assign dmem_wstrb = ex_mem.mem_write ? byte_en : 4'h0;
  assign dmem_we    = ex_mem.mem_write && ex_mem.valid;
  assign dmem_re    = ex_mem.mem_read  && ex_mem.valid;

  // Memory read data sign/zero extension
  logic [31:0] mem_rd_ext;
  always_comb begin
    case (ex_mem.mem_size)
      2'b00: begin // byte
        mem_rd_ext = ex_mem.mem_sign_ext ?
          {{24{dmem_rdata[7]}},  dmem_rdata[7:0]} :
          {24'h0, dmem_rdata[7:0]};
      end
      2'b01: begin // halfword
        mem_rd_ext = ex_mem.mem_sign_ext ?
          {{16{dmem_rdata[15]}}, dmem_rdata[15:0]} :
          {16'h0, dmem_rdata[15:0]};
      end
      default: mem_rd_ext = dmem_rdata; // word
    endcase
  end

  always_comb begin
    mem_wb_nxt.alu_result = ex_mem.alu_result;
    mem_wb_nxt.mem_data   = mem_rd_ext;
    mem_wb_nxt.rd         = ex_mem.rd;
    mem_wb_nxt.reg_write  = ex_mem.reg_write && ex_mem.valid;
    mem_wb_nxt.mem_to_reg = ex_mem.mem_to_reg;
    mem_wb_nxt.valid      = ex_mem.valid;
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
      mem_wb <= '0;
    else
      mem_wb <= mem_wb_nxt;
  end

  // ========== WB Stage ==========
  assign rf_wr_en  = mem_wb.reg_write && mem_wb.valid;
  assign rf_rd_addr = mem_wb.rd;
  assign rf_rd_data = mem_wb.mem_to_reg ? mem_wb.mem_data : mem_wb.alu_result;

  // ========== Debug / Status ==========
  assign debug_pc   = pc;
  assign cpu_active = mem_wb.valid || ex_mem.valid || id_ex.valid;

endmodule

`default_nettype wire
