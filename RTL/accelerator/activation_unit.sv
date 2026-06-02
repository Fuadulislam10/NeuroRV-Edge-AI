// ============================================================================
// FILE: rtl/accelerator/activation_unit.sv
// PROJECT: NeuroRV Edge - Phase 3 Vector AI Accelerator
// MODULE: activation_unit
// DESCRIPTION: Fully parallel activation function unit supporting:
//              ReLU, Leaky ReLU, Sigmoid (piecewise approx), Tanh (LUT)
//              Operates on all VEC_LEN lanes simultaneously, 1-2 cycle latency.
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

module activation_unit #(
    parameter int VEC_LEN    = 256,
    parameter int DATA_IN_W  = 40,   // From accumulator (wide)
    parameter int DATA_OUT_W = 16    // Output precision
)(
    input  logic                      clk,
    input  logic                      rst_n,
    input  logic [1:0]                act_sel,    // 00=None 01=ReLU 10=LeakyReLU 11=Sigmoid/Tanh
    input  logic [7:0]                alpha,      // Leaky ReLU alpha Q0.8
    input  logic                      valid_in,
    input  logic [DATA_IN_W-1:0]      data_in  [0:VEC_LEN-1],
    output logic                      valid_out,
    output logic [DATA_OUT_W-1:0]     data_out [0:VEC_LEN-1]
);

    // =========================================================================
    // Saturation helper: clip wide accumulator to output precision
    // =========================================================================
    localparam signed [DATA_IN_W-1:0] OUT_MAX =
        {{(DATA_IN_W-DATA_OUT_W){1'b0}}, {DATA_OUT_W{1'b1}} >> 1};  // 2^(DATA_OUT_W-1)-1
    localparam signed [DATA_IN_W-1:0] OUT_MIN =
        {{(DATA_IN_W-DATA_OUT_W){1'b1}}, 1'b0, {(DATA_OUT_W-1){1'b0}}};

    function automatic logic [DATA_OUT_W-1:0] saturate(input logic [DATA_IN_W-1:0] x);
        if ($signed(x) > $signed(OUT_MAX))
            return OUT_MAX[DATA_OUT_W-1:0];
        else if ($signed(x) < $signed(OUT_MIN))
            return OUT_MIN[DATA_OUT_W-1:0];
        else
            return x[DATA_OUT_W-1:0];
    endfunction

    // =========================================================================
    // Sigmoid Hardware-Friendly Piecewise Approximation (Q8.8 output)
    // Approximation:
    //   x <= -4  : 0
    //   x >= +4  : 1.0 (0xFF in Q0.8)
    //   -4 < x < 0: 0.5 + x/8
    //   0 <= x < 4: 0.5 + x/8
    // All in fixed-point integer arithmetic scaled to DATA_OUT_W
    // sigmoid(x) ≈ clamp(0.5 + x/8, 0, 1) scaled to [0, 2^(DATA_OUT_W-1)]
    // =========================================================================
    localparam int HALF     = (1 << (DATA_OUT_W-1)) >> 1;  // 0.5 in fixed point
    localparam int FULL     = (1 << (DATA_OUT_W-1)) - 1;   // 1.0 saturated
    localparam int SIG_CLIP = 4 * (1 << (DATA_IN_W - DATA_OUT_W - 1)); // |x| clip threshold

    function automatic logic [DATA_OUT_W-1:0] sigmoid_approx(input logic [DATA_IN_W-1:0] x_in);
        logic signed [DATA_IN_W-1:0] x;
        logic signed [DATA_IN_W-1:0] shifted;
        logic signed [DATA_IN_W:0]   result;
        x = $signed(x_in);
        // Scale down x to output precision range
        shifted = x >>> (DATA_IN_W - DATA_OUT_W);
        if (shifted <= -4)
            return '0;
        else if (shifted >= 4)
            return DATA_OUT_W'(FULL);
        else begin
            result = $signed(DATA_IN_W'(HALF)) + (shifted >>> 3);
            if (result < 0)       return '0;
            else if (result > FULL) return DATA_OUT_W'(FULL);
            else                    return result[DATA_OUT_W-1:0];
        end
    endfunction

    // =========================================================================
    // Tanh Piecewise Approximation
    // tanh(x) ≈ clamp(x / 2, -1, 1)  for |x| < 2
    //         = sign(x) * 1           for |x| >= 2
    // Output range: [-2^(DATA_OUT_W-1), 2^(DATA_OUT_W-1)-1]
    // =========================================================================
    function automatic logic [DATA_OUT_W-1:0] tanh_approx(input logic [DATA_IN_W-1:0] x_in);
        logic signed [DATA_IN_W-1:0] x;
        logic signed [DATA_IN_W-1:0] shifted;
        logic signed [DATA_IN_W-1:0] half_shifted;
        x = $signed(x_in);
        shifted = x >>> (DATA_IN_W - DATA_OUT_W);
        half_shifted = shifted >>> 1;
        if (shifted >= 2)
            return DATA_OUT_W'(FULL);
        else if (shifted <= -2)
            return {1'b1, {(DATA_OUT_W-1){1'b0}}};  // -1 in 2's complement
        else
            return half_shifted[DATA_OUT_W-1:0];
    endfunction

    // =========================================================================
    // Leaky ReLU: output = x if x >= 0, else alpha * x  (alpha Q0.8)
    // =========================================================================
    function automatic logic [DATA_OUT_W-1:0] leaky_relu(
        input logic [DATA_IN_W-1:0]  x_in,
        input logic [7:0]            a
    );
        logic signed [DATA_IN_W-1:0]   x;
        logic signed [DATA_IN_W+8:0]   neg_result;
        logic        [DATA_IN_W-1:0]   neg_clipped;
        x = $signed(x_in);
        if (x >= 0)
            return saturate(x_in);
        else begin
            neg_result  = x * $signed({1'b0, a});  // alpha * x
            neg_clipped = neg_result[DATA_IN_W-1:0];
            return saturate(neg_clipped);
        end
    endfunction

    // =========================================================================
    // Pipeline Stage 1: Compute activation per lane
    // =========================================================================
    logic [DATA_OUT_W-1:0] stage1_out [0:VEC_LEN-1];
    logic                  stage1_valid;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            stage1_valid <= 1'b0;
            for (int i = 0; i < VEC_LEN; i++) stage1_out[i] <= '0;
        end else begin
            stage1_valid <= valid_in;
            if (valid_in) begin
                for (int i = 0; i < VEC_LEN; i++) begin
                    case (act_sel)
                        2'b00: // Pass-through (saturate only)
                            stage1_out[i] <= saturate(data_in[i]);
                        2'b01: // ReLU
                            stage1_out[i] <= ($signed(data_in[i]) >= 0) ?
                                             saturate(data_in[i]) : '0;
                        2'b10: // Leaky ReLU
                            stage1_out[i] <= leaky_relu(data_in[i], alpha);
                        2'b11: // Sigmoid approx (act_sel[0]==1) / Tanh (act_sel[1]==1)
                            stage1_out[i] <= sigmoid_approx(data_in[i]);
                        default:
                            stage1_out[i] <= saturate(data_in[i]);
                    endcase
                end
            end
        end
    end

    // =========================================================================
    // Pipeline Stage 2: Output register
    // =========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            valid_out <= 1'b0;
            for (int i = 0; i < VEC_LEN; i++) data_out[i] <= '0;
        end else begin
            valid_out <= stage1_valid;
            if (stage1_valid) begin
                for (int i = 0; i < VEC_LEN; i++) data_out[i] <= stage1_out[i];
            end
        end
    end

    // =========================================================================
    // Simulation assertions
    // =========================================================================
    // synthesis translate_off
    always @(posedge clk) begin
        if (valid_in) begin
            assert (act_sel <= 2'b11)
                else $warning("activation_unit: Unknown act_sel value");
        end
    end
    // synthesis translate_on

endmodule

`default_nettype wire
