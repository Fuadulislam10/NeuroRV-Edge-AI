// ============================================================================
// FILE: rtl/accelerator/norm_unit.sv
// PROJECT: NeuroRV Edge - Phase 3 Vector AI Accelerator
// MODULE: norm_unit
// DESCRIPTION: Batch normalization inference unit for VEC_LEN parallel lanes.
//
//  Formula (inference): y = (x - mean) * inv_std
//  Fixed-point: mean and inv_std are Q8.8 (16-bit) format
//  Operation per lane:
//    1. Subtract mean   (signed)
//    2. Multiply by inv_std (Q8.8 → right shift 8 for result)
//    3. Saturate to DATA_W output
//
//  mean    [15:0] = Q8.8 signed fixed-point mean
//  inv_std [15:0] = Q8.8 unsigned fixed-point 1/sqrt(var+eps), pre-computed
//
// PIPELINE DEPTH: 2 cycles (subtract → multiply/shift → output)
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

module norm_unit #(
    parameter int VEC_LEN = 256,
    parameter int DATA_W  = 16
)(
    input  logic                  clk,
    input  logic                  rst_n,
    input  logic [15:0]           mean,      // Q8.8 signed
    input  logic [15:0]           inv_std,   // Q8.8 unsigned (1/std)
    input  logic                  valid_in,
    input  logic [DATA_W-1:0]     data_in  [0:VEC_LEN-1],
    output logic                  valid_out,
    output logic [DATA_W-1:0]     data_out [0:VEC_LEN-1]
);

    // =========================================================================
    // Fixed-point parameters
    // FP_FRAC: fractional bits in Q8.8 representation
    // =========================================================================
    localparam int FP_FRAC      = 8;
    localparam int PROD_W       = DATA_W + 16 + 1; // Wide product to avoid overflow
    localparam signed [PROD_W-1:0] CLIP_MAX =
        {{(PROD_W-DATA_W){1'b0}}, {1'b0, {(DATA_W-1){1'b1}}}};
    localparam signed [PROD_W-1:0] CLIP_MIN =
        {{(PROD_W-DATA_W){1'b1}}, {1'b1, {(DATA_W-1){1'b0}}}};

    // =========================================================================
    // Stage 1: Subtract mean from each lane
    // mean is Q8.8: we scale data_in to Q8.8 space (left shift FP_FRAC)
    // sub[i] = (x << FP_FRAC) - mean
    // =========================================================================
    logic signed [DATA_W+FP_FRAC:0] s1_sub   [0:VEC_LEN-1];
    logic                           s1_valid;
    logic [15:0]                    s1_inv_std;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s1_valid   <= 1'b0;
            s1_inv_std <= '0;
            for (int i = 0; i < VEC_LEN; i++) s1_sub[i] <= '0;
        end else begin
            s1_valid   <= valid_in;
            s1_inv_std <= inv_std;
            if (valid_in) begin
                for (int i = 0; i < VEC_LEN; i++) begin
                    // Sign-extend data_in, shift left FP_FRAC bits, subtract mean
                    s1_sub[i] <= ($signed({{1{data_in[i][DATA_W-1]}}, data_in[i]}) <<< FP_FRAC)
                               - $signed({mean[15], mean});
                end
            end
        end
    end

    // =========================================================================
    // Stage 2: Multiply by inv_std (Q8.8), right-shift FP_FRAC, saturate
    // Result = sub * inv_std >> FP_FRAC
    // =========================================================================
    logic signed [PROD_W-1:0] s2_prod  [0:VEC_LEN-1];
    logic                     s2_valid;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s2_valid <= 1'b0;
            for (int i = 0; i < VEC_LEN; i++) s2_prod[i] <= '0;
        end else begin
            s2_valid <= s1_valid;
            if (s1_valid) begin
                for (int i = 0; i < VEC_LEN; i++) begin
                    // Multiply: sub (signed) * inv_std (unsigned), then scale down
                    s2_prod[i] <= ($signed(s1_sub[i]) * $signed({1'b0, s1_inv_std})) >>> FP_FRAC;
                end
            end
        end
    end

    // =========================================================================
    // Stage 3: Saturate and output
    // =========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            valid_out <= 1'b0;
            for (int i = 0; i < VEC_LEN; i++) data_out[i] <= '0;
        end else begin
            valid_out <= s2_valid;
            if (s2_valid) begin
                for (int i = 0; i < VEC_LEN; i++) begin
                    if ($signed(s2_prod[i]) > $signed(CLIP_MAX))
                        data_out[i] <= CLIP_MAX[DATA_W-1:0];
                    else if ($signed(s2_prod[i]) < $signed(CLIP_MIN))
                        data_out[i] <= CLIP_MIN[DATA_W-1:0];
                    else
                        data_out[i] <= s2_prod[i][DATA_W-1:0];
                end
            end
        end
    end

    // =========================================================================
    // Assertions
    // =========================================================================
    // synthesis translate_off
    initial begin
        assert (DATA_W >= 8)
            else $fatal(1, "norm_unit: DATA_W must be >= 8");
    end
    // synthesis translate_on

endmodule

`default_nettype wire
