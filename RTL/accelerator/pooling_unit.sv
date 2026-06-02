// ============================================================================
// FILE: rtl/accelerator/pooling_unit.sv
// PROJECT: NeuroRV Edge - Phase 3 Vector AI Accelerator
// MODULE: pooling_unit
// DESCRIPTION: Parallel pooling unit for edge AI inference.
//              Supports 2x2 Max/Average and 4x4 Max/Average pooling.
//              Input: VEC_LEN lanes, treated as flattened 2D window groups.
//              Output: VEC_LEN/4 (2x2) or VEC_LEN/16 (4x4) reduced lanes.
//
// pool_mode[1:0]:
//   00 = Max Pooling  2x2
//   01 = Avg Pooling  2x2
//   10 = Max Pooling  4x4
//   11 = Avg Pooling  4x4
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

module pooling_unit #(
    parameter int VEC_LEN = 256,
    parameter int DATA_W  = 16
)(
    input  logic                  clk,
    input  logic                  rst_n,
    input  logic [1:0]            pool_mode,
    input  logic                  valid_in,
    input  logic [DATA_W-1:0]     data_in  [0:VEC_LEN-1],
    output logic                  valid_out,
    output logic [DATA_W-1:0]     data_out [0:VEC_LEN/4-1]  // Worst-case: 2x2 = VEC_LEN/4
);

    // =========================================================================
    // Constants
    // =========================================================================
    localparam int OUT_LEN_2x2 = VEC_LEN / 4;
    localparam int OUT_LEN_4x4 = VEC_LEN / 16;

    // =========================================================================
    // Helper: signed max of two values
    // =========================================================================
    function automatic logic [DATA_W-1:0] smax2(
        input logic [DATA_W-1:0] a,
        input logic [DATA_W-1:0] b
    );
        return ($signed(a) >= $signed(b)) ? a : b;
    endfunction

    // =========================================================================
    // Stage 1: Group-wise reduction
    // For 2x2: each group of 4 consecutive lanes is reduced to 1
    // For 4x4: each group of 16 consecutive lanes is reduced to 1
    // =========================================================================

    // --- 2x2 Max Pooling ---
    logic [DATA_W-1:0] max2x2 [0:OUT_LEN_2x2-1];
    always_comb begin
        for (int g = 0; g < OUT_LEN_2x2; g++) begin
            max2x2[g] = smax2(
                smax2(data_in[g*4+0], data_in[g*4+1]),
                smax2(data_in[g*4+2], data_in[g*4+3])
            );
        end
    end

    // --- 2x2 Avg Pooling ---
    logic [DATA_W+1:0] avg2x2_sum [0:OUT_LEN_2x2-1]; // Extra bits for sum
    logic [DATA_W-1:0] avg2x2     [0:OUT_LEN_2x2-1];
    always_comb begin
        for (int g = 0; g < OUT_LEN_2x2; g++) begin
            avg2x2_sum[g] = $signed({data_in[g*4+0][DATA_W-1], data_in[g*4+0]})
                          + $signed({data_in[g*4+1][DATA_W-1], data_in[g*4+1]})
                          + $signed({data_in[g*4+2][DATA_W-1], data_in[g*4+2]})
                          + $signed({data_in[g*4+3][DATA_W-1], data_in[g*4+3]});
            avg2x2[g] = avg2x2_sum[g][DATA_W+1:2]; // Divide by 4 = right shift 2
        end
    end

    // --- 4x4 Max Pooling (VEC_LEN must be >= 16) ---
    logic [DATA_W-1:0] max4x4 [0:OUT_LEN_4x4-1];
    logic [DATA_W-1:0] row_max [0:OUT_LEN_4x4-1][0:3]; // Row-wise max for 4x4

    always_comb begin
        for (int g = 0; g < OUT_LEN_4x4; g++) begin
            for (int r = 0; r < 4; r++) begin
                row_max[g][r] = smax2(
                    smax2(data_in[g*16 + r*4 + 0], data_in[g*16 + r*4 + 1]),
                    smax2(data_in[g*16 + r*4 + 2], data_in[g*16 + r*4 + 3])
                );
            end
            max4x4[g] = smax2(
                smax2(row_max[g][0], row_max[g][1]),
                smax2(row_max[g][2], row_max[g][3])
            );
        end
    end

    // --- 4x4 Avg Pooling ---
    logic signed [DATA_W+3:0] avg4x4_sum [0:OUT_LEN_4x4-1]; // 4 extra bits for 16-element sum
    logic        [DATA_W-1:0] avg4x4     [0:OUT_LEN_4x4-1];
    always_comb begin
        for (int g = 0; g < OUT_LEN_4x4; g++) begin
            avg4x4_sum[g] = '0;
            for (int k = 0; k < 16; k++) begin
                avg4x4_sum[g] = avg4x4_sum[g]
                    + $signed({{4{data_in[g*16+k][DATA_W-1]}}, data_in[g*16+k]});
            end
            avg4x4[g] = avg4x4_sum[g][DATA_W+3:4]; // Divide by 16 = right shift 4
        end
    end

    // =========================================================================
    // Stage 2: Output register + MUX
    // =========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            valid_out <= 1'b0;
            for (int i = 0; i < OUT_LEN_2x2; i++) data_out[i] <= '0;
        end else begin
            valid_out <= valid_in;
            if (valid_in) begin
                case (pool_mode)
                    2'b00: begin // Max 2x2
                        for (int i = 0; i < OUT_LEN_2x2; i++)
                            data_out[i] <= max2x2[i];
                    end
                    2'b01: begin // Avg 2x2
                        for (int i = 0; i < OUT_LEN_2x2; i++)
                            data_out[i] <= avg2x2[i];
                    end
                    2'b10: begin // Max 4x4
                        for (int i = 0; i < OUT_LEN_4x4; i++)
                            data_out[i] <= max4x4[i];
                        // Zero-pad unused outputs
                        for (int i = OUT_LEN_4x4; i < OUT_LEN_2x2; i++)
                            data_out[i] <= '0;
                    end
                    2'b11: begin // Avg 4x4
                        for (int i = 0; i < OUT_LEN_4x4; i++)
                            data_out[i] <= avg4x4[i];
                        for (int i = OUT_LEN_4x4; i < OUT_LEN_2x2; i++)
                            data_out[i] <= '0;
                    end
                    default: begin
                        for (int i = 0; i < OUT_LEN_2x2; i++)
                            data_out[i] <= '0;
                    end
                endcase
            end
        end
    end

    // =========================================================================
    // Parameter check
    // =========================================================================
    // synthesis translate_off
    initial begin
        assert (VEC_LEN >= 16 && (VEC_LEN % 16 == 0))
            else $fatal(1, "pooling_unit: VEC_LEN must be >= 16 and divisible by 16 for 4x4 pooling");
    end
    // synthesis translate_on

endmodule

`default_nettype wire
