// ============================================================================
// FILE: rtl/accelerator/mac_array.sv
// PROJECT: NeuroRV Edge - Phase 3 Vector AI Accelerator
// MODULE: mac_array
// DESCRIPTION: 256-lane parallel pipelined MAC array with INT8/INT16 support,
//              saturation arithmetic, and lane-level valid tracking.
// PIPELINE DEPTH: 3 stages (Reg-in → Multiply → Accumulate/Saturate → Reg-out)
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

module mac_array #(
    parameter int VEC_LEN = 256,
    parameter int DATA_W  = 16,
    parameter int ACCUM_W = 40    // Wide accumulator to avoid overflow
)(
    input  logic                    clk,
    input  logic                    rst_n,
    input  logic                    start,
    input  logic [1:0]              dtype,      // 00=INT8, 01=INT16
    input  logic [DATA_W-1:0]       a         [0:VEC_LEN-1],
    input  logic [DATA_W-1:0]       b         [0:VEC_LEN-1],
    input  logic                    valid_in,
    output logic [ACCUM_W-1:0]      result    [0:VEC_LEN-1],
    output logic                    valid_out,
    output logic                    done
);

    // =========================================================================
    // Pipeline Stage Signals
    // =========================================================================

    // Stage 1: Input registration
    logic [DATA_W-1:0]  s1_a       [0:VEC_LEN-1];
    logic [DATA_W-1:0]  s1_b       [0:VEC_LEN-1];
    logic               s1_valid;
    logic [1:0]         s1_dtype;

    // Stage 2: Multiply (partial products)
    // For INT8: sign-extend 8-bit inputs to 16-bit, multiply → 32-bit product
    // For INT16: sign-extend 16-bit inputs, multiply → 32-bit product
    logic signed [31:0] s2_product [0:VEC_LEN-1];
    logic               s2_valid;

    // Stage 3: Accumulate + Saturate
    logic [ACCUM_W-1:0] s3_accum  [0:VEC_LEN-1];
    logic               s3_valid;

    // =========================================================================
    // Accumulator registers (retained across calls via start/done)
    // =========================================================================
    logic [ACCUM_W-1:0] accum_reg [0:VEC_LEN-1];
    logic               accumulating;

    // Saturation limits per dtype
    logic signed [ACCUM_W-1:0] sat_max, sat_min;
    always_comb begin
        if (dtype == 2'b00) begin  // INT8
            sat_max = {{(ACCUM_W-8){1'b0}}, 8'h7F};
            sat_min = {{(ACCUM_W-8){1'b1}}, 8'h80};
        end else begin             // INT16
            sat_max = {{(ACCUM_W-16){1'b0}}, 16'h7FFF};
            sat_min = {{(ACCUM_W-16){1'b1}}, 16'h8000};
        end
    end

    // =========================================================================
    // Stage 1: Input Register
    // =========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s1_valid <= 1'b0;
            s1_dtype <= 2'b00;
            for (int i = 0; i < VEC_LEN; i++) begin
                s1_a[i] <= '0;
                s1_b[i] <= '0;
            end
        end else begin
            s1_valid <= valid_in;
            s1_dtype <= dtype;
            if (valid_in) begin
                for (int i = 0; i < VEC_LEN; i++) begin
                    s1_a[i] <= a[i];
                    s1_b[i] <= b[i];
                end
            end
        end
    end

    // =========================================================================
    // Stage 2: Parallel Multiply
    // INT8 mode:  sign-extend 8-bit operands, 8x8 → 16-bit product
    // INT16 mode: sign-extend 16-bit operands, 16x16 → 32-bit product
    // =========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s2_valid <= 1'b0;
            for (int i = 0; i < VEC_LEN; i++) s2_product[i] <= '0;
        end else begin
            s2_valid <= s1_valid;
            if (s1_valid) begin
                for (int i = 0; i < VEC_LEN; i++) begin
                    if (s1_dtype == 2'b00) begin
                        // INT8 signed multiply - sign-extend 8-bit operands
                        s2_product[i] <= $signed({{24{s1_a[i][7]}}, s1_a[i][7:0]})
                                       * $signed({{24{s1_b[i][7]}}, s1_b[i][7:0]});
                    end else begin
                        // INT16 signed multiply - sign-extend 16-bit operands
                        s2_product[i] <= $signed({{16{s1_a[i][15]}}, s1_a[i][15:0]})
                                       * $signed({{16{s1_b[i][15]}}, s1_b[i][15:0]});
                    end
                end
            end
        end
    end

    // =========================================================================
    // Stage 3: Accumulate with Saturation
    // =========================================================================
    logic signed [ACCUM_W-1:0] sum_temp [0:VEC_LEN-1]; // Temp sum for saturation check

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s3_valid <= 1'b0;
            accumulating <= 1'b0;
            for (int i = 0; i < VEC_LEN; i++) begin
                accum_reg[i] <= '0;
                s3_accum[i]  <= '0;
            end
        end else begin
            s3_valid <= s2_valid;

            if (start && !accumulating) begin
                // Clear accumulator on fresh start
                for (int i = 0; i < VEC_LEN; i++) accum_reg[i] <= '0;
                accumulating <= 1'b1;
            end

            if (s2_valid) begin
                for (int i = 0; i < VEC_LEN; i++) begin
                    sum_temp[i] = $signed(accum_reg[i]) + $signed(s2_product[i]);

                    // Saturate
                    if (sum_temp[i] > $signed(sat_max))
                        s3_accum[i] <= sat_max;
                    else if (sum_temp[i] < $signed(sat_min))
                        s3_accum[i] <= sat_min;
                    else
                        s3_accum[i] <= sum_temp[i][ACCUM_W-1:0];

                    accum_reg[i] <= s3_accum[i];
                end
            end

            if (done) accumulating <= 1'b0;
        end
    end

    // =========================================================================
    // Output: latch results on valid_out
    // =========================================================================
    assign valid_out = s3_valid;
    assign done      = s3_valid; // single-vector: done == valid_out

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int i = 0; i < VEC_LEN; i++) result[i] <= '0;
        end else if (s3_valid) begin
            for (int i = 0; i < VEC_LEN; i++) result[i] <= s3_accum[i];
        end
    end

    // =========================================================================
    // Assertions (Formal / Simulation)
    // =========================================================================
    // synthesis translate_off
    initial begin
        assert (DATA_W == 8 || DATA_W == 16)
            else $fatal(1, "mac_array: DATA_W must be 8 or 16");
        assert (ACCUM_W >= 32)
            else $fatal(1, "mac_array: ACCUM_W must be >= 32");
        assert (VEC_LEN > 0 && (VEC_LEN % 4 == 0))
            else $fatal(1, "mac_array: VEC_LEN must be > 0 and divisible by 4");
    end
    // synthesis translate_on

endmodule

`default_nettype wire
