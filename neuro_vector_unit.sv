// ============================================================================
// NeuroRV Edge — 16-Lane Vector Processing Unit (VPU)
// File   : rtl/neuro_vector_unit.sv
// Author : NeuroRV Edge Contributors
// License: Apache 2.0
//
// Description:
//   A 16-lane SIMD vector accelerator for neural network inference.
//   - 16 x 32-bit parallel lanes
//   - Operations: VADD, VMUL, VRELU, VSIGMOID (piecewise-linear approx)
//   - Vector register file: 16 registers x 512-bit (16x32-bit elements)
//   - Reduction tree: parallel sum and max reduction
//   - MAC accumulator for dot-product operations
//   - Memory-mapped control/status registers
//
// Register Map (base + offset):
//   0x00 : VPU_CTRL   [W] - opcode, src1, src2, dst
//   0x04 : VPU_STATUS [R] - busy, done, error
//   0x08 : VPU_VLOAD  [W] - load vector from SRAM to VRF
//   0x0C : VPU_VSTORE [W] - store vector from VRF to SRAM
//   0x10 : VPU_ACCUM  [R] - 64-bit accumulator result (low 32)
//   0x14 : VPU_ACCUM_H[R] - accumulator result (high 32)
//   0x18 : VPU_REDUCE [R] - reduction result
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

// ============================================================================
// VPU Opcode Definitions
// ============================================================================
package vpu_pkg;
  localparam logic [3:0] VOP_NOP      = 4'h0;
  localparam logic [3:0] VOP_VADD     = 4'h1;  // element-wise add
  localparam logic [3:0] VOP_VSUB     = 4'h2;  // element-wise subtract
  localparam logic [3:0] VOP_VMUL     = 4'h3;  // element-wise multiply (32x32→32)
  localparam logic [3:0] VOP_VRELU    = 4'h4;  // ReLU activation (src1 only)
  localparam logic [3:0] VOP_VSIGMOID = 4'h5;  // Sigmoid approx (src1 only)
  localparam logic [3:0] VOP_VMAC     = 4'h6;  // Multiply-accumulate (dot product)
  localparam logic [3:0] VOP_VSUM     = 4'h7;  // Reduction: sum all lanes
  localparam logic [3:0] VOP_VMAX     = 4'h8;  // Reduction: max all lanes
  localparam logic [3:0] VOP_VLOAD    = 4'h9;  // Load from mem to VRF
  localparam logic [3:0] VOP_VSTORE   = 4'hA;  // Store from VRF to mem
  localparam logic [3:0] VOP_VCOPY    = 4'hB;  // Copy VRF register

  localparam int LANES     = 16;
  localparam int VRF_DEPTH = 16;  // 16 vector registers
endpackage

// ============================================================================
// Single-Lane ALU
// ============================================================================
module vpu_lane_alu (
  input  logic [3:0]  op,
  input  logic [31:0] a,
  input  logic [31:0] b,
  output logic [31:0] result,
  output logic [63:0] mac_out   // full 64-bit product for accumulation
);
  import vpu_pkg::*;

  // Signed multiply (32x32 → 64)
  logic [63:0] mul64;
  assign mul64 = {{32{a[31]}}, a} * {{32{b[31]}}, b};

  // Piecewise-linear sigmoid approximation:
  // sigmoid(x) ≈ 0         if x < -4
  //            ≈ 0.125x+0.5 if -4 ≤ x < 4   (scaled by 2^16)
  //            ≈ 1         if x ≥ 4
  // Input x is Q16 fixed-point: value = a / 65536.0
  logic [31:0] sigmoid_result;
  always_comb begin
    // Threshold at ±4 in Q16 = ±262144
    if ($signed(a) <= -32'sd262144)
      sigmoid_result = 32'h0000;          // 0.0 in Q16
    else if ($signed(a) >= 32'sd262144)
      sigmoid_result = 32'h00010000;      // 1.0 in Q16
    else
      // (x >> 3) + 0x8000 = 0.125*x + 0.5 in Q16
      sigmoid_result = (($signed(a) >>> 3) + 32'sh8000);
  end

  assign mac_out = mul64;

  always_comb begin
    case (op)
      VOP_VADD:     result = a + b;
      VOP_VSUB:     result = a - b;
      VOP_VMUL:     result = mul64[31:0];
      VOP_VRELU:    result = ($signed(a) < 0) ? 32'h0 : a;
      VOP_VSIGMOID: result = sigmoid_result;
      VOP_VMAC:     result = mul64[31:0];
      VOP_VCOPY:    result = a;
      default:      result = 32'h0;
    endcase
  end
endmodule

// ============================================================================
// 16-to-1 Reduction Tree (sum and max, 3-stage)
// ============================================================================
module vpu_reduction_tree (
  input  logic [31:0] in  [0:15],
  output logic [31:0] sum_out,
  output logic [31:0] max_out
);
  // Stage 1: 16→8
  logic [32:0] sum_s1 [0:7];  // 33-bit to prevent overflow
  logic [31:0] max_s1 [0:7];
  genvar i;
  generate
    for (i = 0; i < 8; i++) begin : gen_s1
      assign sum_s1[i] = {1'b0, in[2*i]} + {1'b0, in[2*i+1]};
      assign max_s1[i] = ($signed(in[2*i]) > $signed(in[2*i+1])) ?
                          in[2*i] : in[2*i+1];
    end
  endgenerate

  // Stage 2: 8→4
  logic [34:0] sum_s2 [0:3];
  logic [31:0] max_s2 [0:3];
  generate
    for (i = 0; i < 4; i++) begin : gen_s2
      assign sum_s2[i] = {2'b0, sum_s1[2*i][31:0]} + {2'b0, sum_s1[2*i+1][31:0]};
      assign max_s2[i] = ($signed(max_s1[2*i]) > $signed(max_s1[2*i+1])) ?
                          max_s1[2*i] : max_s1[2*i+1];
    end
  endgenerate

  // Stage 3: 4→2
  logic [35:0] sum_s3 [0:1];
  logic [31:0] max_s3 [0:1];
  generate
    for (i = 0; i < 2; i++) begin : gen_s3
      assign sum_s3[i] = {1'b0, sum_s2[2*i][33:0]} + {1'b0, sum_s2[2*i+1][33:0]};
      assign max_s3[i] = ($signed(max_s2[2*i]) > $signed(max_s2[2*i+1])) ?
                          max_s2[2*i] : max_s2[2*i+1];
    end
  endgenerate

  // Stage 4: 2→1
  assign sum_out = sum_s3[0][31:0] + sum_s3[1][31:0];
  assign max_out = ($signed(max_s3[0]) > $signed(max_s3[1])) ?
                    max_s3[0] : max_s3[1];
endmodule

// ============================================================================
// VPU Top Module
// ============================================================================
module neuro_vector_unit (
  input  logic        clk,
  input  logic        rst_n,

  // CPU control interface (memory-mapped registers)
  input  logic [31:0] ctrl_addr,
  input  logic [31:0] ctrl_wdata,
  input  logic        ctrl_we,
  input  logic        ctrl_re,
  output logic [31:0] ctrl_rdata,

  // Memory interface for vector load/store
  output logic [31:0] mem_addr,
  output logic [31:0] mem_wdata,
  output logic        mem_we,
  output logic        mem_re,
  input  logic [31:0] mem_rdata,
  input  logic        mem_ack,

  // Status
  output logic        vpu_busy,
  output logic        vpu_done,
  output logic        vpu_irq
);
  import vpu_pkg::*;

  // ---- Vector Register File ----
  // 16 registers, each 512 bits = 16 x 32-bit elements
  logic [31:0] vrf [0:VRF_DEPTH-1][0:LANES-1];

  // ---- Control Registers ----
  logic [3:0]  vpu_opcode;
  logic [3:0]  vpu_src1;
  logic [3:0]  vpu_src2;
  logic [3:0]  vpu_dst;
  logic        vpu_start;
  logic        vpu_error;
  logic [63:0] vpu_accumulator;
  logic [31:0] vpu_reduce_result;

  // ---- Lane ALU Outputs ----
  logic [31:0] lane_result [0:LANES-1];
  logic [63:0] lane_mac    [0:LANES-1];

  // ---- Reduction Tree ----
  logic [31:0] reduce_in  [0:LANES-1];
  logic [31:0] reduce_sum;
  logic [31:0] reduce_max;

  // ---- State Machine ----
  typedef enum logic [2:0] {
    VPU_IDLE    = 3'h0,
    VPU_EXEC    = 3'h1,
    VPU_MEM_LD  = 3'h2,
    VPU_MEM_ST  = 3'h3,
    VPU_REDUCE  = 3'h4,
    VPU_DONE    = 3'h5
  } vpu_state_t;

  vpu_state_t vpu_state;

  logic [4:0]  mem_lane_cnt;   // which lane we're loading/storing
  logic [31:0] mem_base_addr;  // base address for load/store

  // ========== Instantiate 16 Lane ALUs ==========
  genvar gi;
  generate
    for (gi = 0; gi < LANES; gi++) begin : gen_lanes
      vpu_lane_alu u_lane (
        .op     (vpu_opcode),
        .a      (vrf[vpu_src1][gi]),
        .b      (vrf[vpu_src2][gi]),
        .result (lane_result[gi]),
        .mac_out(lane_mac[gi])
      );
    end
  endgenerate

  // ========== Reduction Tree ==========
  generate
    for (gi = 0; gi < LANES; gi++) begin : gen_red_in
      assign reduce_in[gi] = vrf[vpu_dst][gi];
    end
  endgenerate

  vpu_reduction_tree u_reduce (
    .in     (reduce_in),
    .sum_out(reduce_sum),
    .max_out(reduce_max)
  );

  // ========== Control Register Interface ==========
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      vpu_opcode    <= VOP_NOP;
      vpu_src1      <= 4'h0;
      vpu_src2      <= 4'h0;
      vpu_dst       <= 4'h0;
      vpu_start     <= 1'b0;
      mem_base_addr <= 32'h0;
    end else begin
      vpu_start <= 1'b0; // auto-clear
      if (ctrl_we) begin
        case (ctrl_addr[4:0])
          5'h00: begin // VPU_CTRL: [31:28]=opcode [27:24]=src1 [23:20]=src2 [19:16]=dst [0]=start
            vpu_opcode <= ctrl_wdata[31:28];
            vpu_src1   <= ctrl_wdata[27:24];
            vpu_src2   <= ctrl_wdata[23:20];
            vpu_dst    <= ctrl_wdata[19:16];
            vpu_start  <= ctrl_wdata[0];
          end
          5'h08: mem_base_addr <= ctrl_wdata; // VPU_VLOAD base addr
          5'h0C: mem_base_addr <= ctrl_wdata; // VPU_VSTORE base addr
          default: ;
        endcase
      end
    end
  end

  // Read back
  always_comb begin
    ctrl_rdata = 32'h0;
    if (ctrl_re) begin
      case (ctrl_addr[4:0])
        5'h00: ctrl_rdata = {vpu_opcode, vpu_src1, vpu_src2, vpu_dst, 15'h0, vpu_busy};
        5'h04: ctrl_rdata = {29'h0, vpu_error, vpu_done, vpu_busy};
        5'h10: ctrl_rdata = vpu_accumulator[31:0];
        5'h14: ctrl_rdata = vpu_accumulator[63:32];
        5'h18: ctrl_rdata = vpu_reduce_result;
        default: ctrl_rdata = 32'h0;
      endcase
    end
  end

  // ========== VPU State Machine ==========
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      vpu_state       <= VPU_IDLE;
      vpu_busy        <= 1'b0;
      vpu_done        <= 1'b0;
      vpu_irq         <= 1'b0;
      vpu_error       <= 1'b0;
      vpu_accumulator <= 64'h0;
      vpu_reduce_result <= 32'h0;
      mem_lane_cnt    <= 5'h0;
      mem_addr        <= 32'h0;
      mem_we          <= 1'b0;
      mem_re          <= 1'b0;
      mem_wdata       <= 32'h0;
    end else begin
      vpu_done <= 1'b0;
      vpu_irq  <= 1'b0;
      mem_we   <= 1'b0;
      mem_re   <= 1'b0;

      case (vpu_state)
        // ------ IDLE ------
        VPU_IDLE: begin
          if (vpu_start) begin
            vpu_busy <= 1'b1;
            vpu_done <= 1'b0;
            case (vpu_opcode)
              VOP_VLOAD:  begin vpu_state <= VPU_MEM_LD; mem_lane_cnt <= 5'h0; end
              VOP_VSTORE: begin vpu_state <= VPU_MEM_ST; mem_lane_cnt <= 5'h0; end
              VOP_VSUM, VOP_VMAX: vpu_state <= VPU_REDUCE;
              default:    vpu_state <= VPU_EXEC;
            endcase
          end
        end

        // ------ EXECUTE (1-cycle ops) ------
        VPU_EXEC: begin
          case (vpu_opcode)
            VOP_VADD, VOP_VSUB, VOP_VMUL,
            VOP_VRELU, VOP_VSIGMOID, VOP_VCOPY: begin
              for (int l = 0; l < LANES; l++) begin
                vrf[vpu_dst][l] <= lane_result[l];
              end
            end
            VOP_VMAC: begin
              // Accumulate dot product across all lanes
              logic [63:0] mac_sum;
              mac_sum = 64'h0;
              for (int l = 0; l < LANES; l++) begin
                mac_sum = mac_sum + lane_mac[l];
              end
              vpu_accumulator <= vpu_accumulator + mac_sum;
            end
            default: ;
          endcase
          vpu_state <= VPU_DONE;
        end

        // ------ VECTOR LOAD (one lane per cycle) ------
        VPU_MEM_LD: begin
          if (mem_lane_cnt < LANES) begin
            mem_addr <= mem_base_addr + {mem_lane_cnt, 2'b00};
            mem_re   <= 1'b1;
            if (mem_ack) begin
              vrf[vpu_dst][mem_lane_cnt[3:0]] <= mem_rdata;
              mem_lane_cnt <= mem_lane_cnt + 5'h1;
            end
          end else begin
            vpu_state <= VPU_DONE;
          end
        end

        // ------ VECTOR STORE (one lane per cycle) ------
        VPU_MEM_ST: begin
          if (mem_lane_cnt < LANES) begin
            mem_addr  <= mem_base_addr + {mem_lane_cnt, 2'b00};
            mem_wdata <= vrf[vpu_src1][mem_lane_cnt[3:0]];
            mem_we    <= 1'b1;
            if (mem_ack) begin
              mem_lane_cnt <= mem_lane_cnt + 5'h1;
            end
          end else begin
            vpu_state <= VPU_DONE;
          end
        end

        // ------ REDUCTION ------
        VPU_REDUCE: begin
          // Reduction tree is combinational — capture result
          case (vpu_opcode)
            VOP_VSUM: vpu_reduce_result <= reduce_sum;
            VOP_VMAX: vpu_reduce_result <= reduce_max;
            default:  vpu_reduce_result <= reduce_sum;
          endcase
          vpu_state <= VPU_DONE;
        end

        // ------ DONE ------
        VPU_DONE: begin
          vpu_busy  <= 1'b0;
          vpu_done  <= 1'b1;
          vpu_irq   <= 1'b1;
          vpu_state <= VPU_IDLE;
        end

        default: vpu_state <= VPU_IDLE;
      endcase
    end
  end

endmodule

`default_nettype wire
