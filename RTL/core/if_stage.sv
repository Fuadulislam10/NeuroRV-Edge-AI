// ============================================================================
// NeuroRV Edge — Instruction Fetch Stage (IF)
// FILE: rtl/core/if_stage.sv
//
// Responsibilities:
//   • Program counter management (sequential + branch/jump redirect)
//   • Instruction memory request generation
//   • Stall and flush handling
//   • IF/ID pipeline register
//
// Instruction memory interface is word-addressed (32-bit aligned).
// Misaligned fetch support is NOT implemented (compressed ISA out of scope).
//
// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2024-2025 NeuroRV Edge Contributors
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

module if_stage #(
    parameter logic [31:0] RESET_VECTOR = 32'h0010_0000  // Boot ROM base
) (
    input  logic        clk,
    input  logic        rst_n,

    // Stall / Flush from pipeline controller
    input  logic        stall_if,       // hold IF/ID register
    input  logic        flush_if,       // squash current fetch (bubble)

    // Branch/Jump redirection (from EX stage)
    input  logic        branch_taken,   // branch resolved as taken
    input  logic [31:0] branch_target,  // branch/jump target address

    // Trap redirect (from WB/CSR)
    input  logic        trap_taken,
    input  logic [31:0] trap_vector,

    // Trap return (mret)
    input  logic        mret_taken,
    input  logic [31:0] mret_pc,

    // Instruction memory interface (synchronous SRAM)
    output logic        imem_req,       // request valid
    output logic [31:0] imem_addr,      // byte address (word-aligned)
    input  logic [31:0] imem_rdata,     // instruction word
    input  logic        imem_valid,     // data ready (may stall if 0)

    // IF/ID pipeline register outputs
    output logic [31:0] if_id_pc,
    output logic [31:0] if_id_instr,
    output logic        if_id_valid,    // instruction is valid (not bubble)

    // Debug
    output logic [31:0] dbg_pc
);

    // -----------------------------------------------------------------------
    // Program Counter
    // -----------------------------------------------------------------------
    logic [31:0] pc_reg;
    logic [31:0] pc_next;

    // PC selection priority: trap > mret > branch > sequential
    always_comb begin
        if (trap_taken)
            pc_next = trap_vector;
        else if (mret_taken)
            pc_next = mret_pc;
        else if (branch_taken)
            pc_next = branch_target;
        else if (!stall_if && imem_valid)
            pc_next = pc_reg + 32'd4;
        else
            pc_next = pc_reg;
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            pc_reg <= RESET_VECTOR;
        else
            pc_reg <= pc_next;
    end

    // -----------------------------------------------------------------------
    // Instruction memory request
    // -----------------------------------------------------------------------
    assign imem_req  = 1'b1;           // always requesting (simple core)
    assign imem_addr = pc_reg;

    // -----------------------------------------------------------------------
    // IF/ID Pipeline Register
    // -----------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            if_id_pc    <= RESET_VECTOR;
            if_id_instr <= 32'h0000_0013;  // NOP (addi x0, x0, 0)
            if_id_valid <= 1'b0;
        end else if (flush_if || branch_taken || trap_taken || mret_taken) begin
            // Squash: insert NOP bubble
            if_id_pc    <= pc_reg;
            if_id_instr <= 32'h0000_0013;
            if_id_valid <= 1'b0;
        end else if (!stall_if && imem_valid) begin
            if_id_pc    <= pc_reg;
            if_id_instr <= imem_rdata;
            if_id_valid <= 1'b1;
        end
        // else: hold (stall)
    end

    assign dbg_pc = pc_reg;

endmodule

`default_nettype wire
