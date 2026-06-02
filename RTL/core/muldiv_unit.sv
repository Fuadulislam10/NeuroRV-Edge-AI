// ============================================================================
// NeuroRV Edge — Multiply / Divide Unit
// FILE: rtl/core/muldiv_unit.sv
//
// RV32M Extension: MUL, MULH, MULHSU, MULHU, DIV, DIVU, REM, REMU
//
// Multiply: 2-cycle pipelined (uses DSP-inferred 33×33 signed multiplier)
// Divide:   Iterative non-restoring 32-step divider
//           Latency: 1–33 cycles (early-terminate on leading zeros)
//
// Interface:
//   • start pulse → ready goes low → done pulse + result when complete
//   • stall the pipeline until done
//
// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2024-2025 NeuroRV Edge Contributors
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

// MulDiv operation select
typedef enum logic [2:0] {
    MULDIV_MUL    = 3'b000,
    MULDIV_MULH   = 3'b001,
    MULDIV_MULHSU = 3'b010,
    MULDIV_MULHU  = 3'b011,
    MULDIV_DIV    = 3'b100,
    MULDIV_DIVU   = 3'b101,
    MULDIV_REM    = 3'b110,
    MULDIV_REMU   = 3'b111
} muldiv_op_e;

module muldiv_unit (
    input  logic        clk,
    input  logic        rst_n,

    // Control
    input  logic        start,        // pulse: begin operation
    input  muldiv_op_e  op,           // operation select
    input  logic [31:0] rs1,          // operand A
    input  logic [31:0] rs2,          // operand B

    // Result
    output logic [31:0] result,       // operation result
    output logic        busy,         // 1 while computing
    output logic        done          // 1-cycle pulse when result valid
);

    // =========================================================================
    // ── MULTIPLIER PATH (2-cycle pipeline)
    // =========================================================================
    // Sign-extend to 33 bits for signed/unsigned handling
    logic signed [32:0] mul_op_a, mul_op_b;
    logic signed [65:0] mul_product;
    logic [65:0]        mul_product_r1, mul_product_r2;
    logic [1:0]         mul_valid;    // shift register for done
    logic               mul_sel;      // 1=MUL operation in flight

    // Sign extension based on operation
    always_comb begin
        unique case (op)
            MULDIV_MUL, MULDIV_MULH, MULDIV_MULHSU: begin
                mul_op_a = {rs1[31], rs1};           // signed A
                mul_op_b = (op == MULDIV_MULHSU) ?
                           {1'b0, rs2} :             // unsigned B for MULHSU
                           {rs2[31], rs2};            // signed B
            end
            MULDIV_MULHU: begin
                mul_op_a = {1'b0, rs1};
                mul_op_b = {1'b0, rs2};
            end
            default: begin
                mul_op_a = {rs1[31], rs1};
                mul_op_b = {rs2[31], rs2};
            end
        endcase
    end

    assign mul_product = mul_op_a * mul_op_b;

    // 2-stage pipeline register
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mul_product_r1 <= '0;
            mul_product_r2 <= '0;
            mul_valid      <= '0;
        end else begin
            mul_product_r1 <= mul_product;
            mul_product_r2 <= mul_product_r1;
            mul_valid      <= {mul_valid[0], start & (op inside {
                              MULDIV_MUL, MULDIV_MULH, MULDIV_MULHSU, MULDIV_MULHU})};
        end
    end

    logic [31:0] mul_result;
    always_comb begin
        unique case (op)
            MULDIV_MUL:    mul_result = mul_product_r2[31:0];
            MULDIV_MULH,
            MULDIV_MULHSU,
            MULDIV_MULHU:  mul_result = mul_product_r2[63:32];
            default:        mul_result = mul_product_r2[31:0];
        endcase
    end

    // =========================================================================
    // ── DIVIDER PATH (iterative non-restoring, 32-step)
    // =========================================================================
    // Handles signed/unsigned, division by zero, overflow (INT_MIN / -1)
    logic        div_running;
    logic [5:0]  div_cnt;           // counts 0..32
    logic [31:0] div_quotient;
    logic [32:0] div_remainder;     // one extra bit for borrow
    logic [31:0] div_divisor;
    logic        div_neg_quot;      // quotient should be negated
    logic        div_neg_rem;       // remainder should be negated
    logic        div_done_r;
    logic        div_sel;           // 1=divide operation in flight
    logic [31:0] div_result_r;
    logic        div_by_zero;
    logic        div_overflow;

    // Absolute values for signed divide
    logic [31:0] abs_rs1, abs_rs2;
    assign abs_rs1 = rs1[31] ? (~rs1 + 1) : rs1;
    assign abs_rs2 = rs2[31] ? (~rs2 + 1) : rs2;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            div_running   <= 1'b0;
            div_cnt       <= '0;
            div_quotient  <= '0;
            div_remainder <= '0;
            div_divisor   <= '0;
            div_neg_quot  <= 1'b0;
            div_neg_rem   <= 1'b0;
            div_done_r    <= 1'b0;
            div_sel       <= 1'b0;
            div_result_r  <= '0;
            div_by_zero   <= 1'b0;
            div_overflow  <= 1'b0;
        end else begin
            div_done_r <= 1'b0;

            if (start && (op inside {MULDIV_DIV, MULDIV_DIVU, MULDIV_REM, MULDIV_REMU})) begin
                div_sel     <= 1'b1;
                div_by_zero <= (rs2 == '0);
                // Signed overflow: INT_MIN / -1
                div_overflow <= (op == MULDIV_DIV) &&
                                (rs1 == 32'h8000_0000) && (rs2 == 32'hFFFF_FFFF);

                if (rs2 == '0) begin
                    // Division by zero: done immediately
                    div_running  <= 1'b0;
                    div_done_r   <= 1'b1;
                    div_sel      <= 1'b0;
                    div_result_r <= (op inside {MULDIV_REM, MULDIV_REMU}) ? rs1 : 32'hFFFF_FFFF;
                end else if ((op == MULDIV_DIV) &&
                             (rs1 == 32'h8000_0000) && (rs2 == 32'hFFFF_FFFF)) begin
                    // Signed overflow
                    div_running  <= 1'b0;
                    div_done_r   <= 1'b1;
                    div_sel      <= 1'b0;
                    div_result_r <= (op == MULDIV_REM) ? '0 : 32'h8000_0000;
                end else begin
                    // Normal division: set up iterative divider
                    div_running   <= 1'b1;
                    div_cnt       <= 6'd31;
                    div_remainder <= 33'b0;
                    div_divisor   <= (op inside {MULDIV_DIVU, MULDIV_REMU}) ? rs2 : abs_rs2;
                    div_quotient  <= (op inside {MULDIV_DIVU, MULDIV_REMU}) ? rs1 : abs_rs1;
                    div_neg_quot  <= (op == MULDIV_DIV)  && (rs1[31] ^ rs2[31]);
                    div_neg_rem   <= (op == MULDIV_REM)  && rs1[31];
                end
            end else if (div_running) begin
                // Non-restoring step
                logic [32:0] partial;
                partial = {div_remainder[31:0], div_quotient[31]};

                if (partial >= {1'b0, div_divisor}) begin
                    div_remainder <= partial - {1'b0, div_divisor};
                    div_quotient  <= {div_quotient[30:0], 1'b1};
                end else begin
                    div_remainder <= partial;
                    div_quotient  <= {div_quotient[30:0], 1'b0};
                end

                if (div_cnt == '0) begin
                    div_running <= 1'b0;
                    div_done_r  <= 1'b1;
                    div_sel     <= 1'b0;

                    // Apply sign correction
                    logic [31:0] quot_final, rem_final;
                    quot_final = div_neg_quot ? (~div_quotient[30:0] + 1) :
                                               div_quotient[30:0];
                    // After last shift quotient is complete in div_quotient
                    // Reassign after final shift value
                    quot_final = div_neg_quot ? (~{div_quotient[30:0], 1'b1} + 1) :
                                               {div_quotient[30:0], 1'b1};
                    rem_final  = div_neg_rem  ?
                                 (~{div_remainder[31:0]} + 1) : div_remainder[31:0];

                    // Select quotient or remainder
                    if (op inside {MULDIV_REM, MULDIV_REMU})
                        div_result_r <= rem_final;
                    else
                        div_result_r <= quot_final;
                end else begin
                    div_cnt <= div_cnt - 1;
                end
            end
        end
    end

    // =========================================================================
    // ── Output Mux
    // =========================================================================
    logic mul_done_pulse;
    assign mul_done_pulse = mul_valid[1];

    always_comb begin
        if (div_done_r) begin
            done   = 1'b1;
            result = div_result_r;
        end else if (mul_done_pulse) begin
            done   = 1'b1;
            result = mul_result;
        end else begin
            done   = 1'b0;
            result = '0;
        end

        busy = div_running | (|mul_valid);
    end

endmodule

`default_nettype wire
