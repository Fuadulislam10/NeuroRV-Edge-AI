// ============================================================================
// NeuroRV Edge — Pipeline Controller
// FILE: rtl/core/pipeline_ctrl.sv
//
// Coordinates:
//   • Load-use stall insertion
//   • Mul/Div stall propagation
//   • Branch/jump flush (2-cycle penalty)
//   • Trap entry flush and PC redirect
//   • mret flush and PC redirect
//   • Data memory stall
//   • Global stall/flush signal generation for each stage
//
// Stall priority (highest → lowest):
//   1. Data memory stall (mem_stall)
//   2. Mul/Div stall (ex_stall)
//   3. Load-use stall (load_use_stall)
//
// Flush priority (highest → lowest):
//   1. Trap entry
//   2. Mret
//   3. Branch taken
//
// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2024-2025 NeuroRV Edge Contributors
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

module pipeline_ctrl (
    input  logic clk,
    input  logic rst_n,

    // Hazard inputs
    input  logic load_use_stall,    // from ID stage
    input  logic ex_stall,          // from EX (muldiv busy)
    input  logic mem_stall,         // from MEM (dmem not ready)

    // Redirect inputs
    input  logic branch_taken,      // from EX stage
    input  logic ex_mret,           // mret instruction in EX
    input  logic ex_ecall,          // ecall instruction in EX
    input  logic ex_ebreak,         // ebreak instruction in EX

    // CSR/trap
    input  logic [31:0] mtvec,      // trap vector
    input  logic [31:0] mepc,       // exception return PC
    input  logic [31:0] ex_pc,      // PC of instruction in EX

    // Trap enable (from interrupt controller)
    input  logic irq_pending,       // interrupt pending AND MIE set
    input  logic [31:0] irq_cause,  // mcause value for interrupt

    // Stall outputs (1 = hold stage register)
    output logic stall_if,
    output logic stall_id,
    output logic stall_ex,
    output logic stall_mem,

    // Flush outputs (1 = insert bubble into next stage register)
    output logic flush_if,
    output logic flush_id,
    output logic flush_ex,
    output logic flush_mem,

    // Trap interface to IF stage (PC redirect)
    output logic        trap_taken,
    output logic [31:0] trap_vector,
    output logic        mret_taken,
    output logic [31:0] mret_pc,

    // Trap interface to CSR
    output logic        trap_enter,
    output logic [31:0] trap_cause,
    output logic [31:0] trap_pc,
    output logic [31:0] trap_val,

    // Debug
    output logic [3:0]  dbg_pipeline_state  // one-hot: IF|ID|EX|MEM stalled
);

    // -----------------------------------------------------------------------
    // Trap detection
    // -----------------------------------------------------------------------
    // Trap sources (in EX stage): ecall, ebreak, interrupt
    logic do_trap;
    assign do_trap = ex_ecall | ex_ebreak | irq_pending;

    // Trap cause encoding (mcause)
    logic [31:0] trap_cause_sel;
    always_comb begin
        if (irq_pending)
            trap_cause_sel = irq_cause;            // interrupt: bit31=1
        else if (ex_ecall)
            trap_cause_sel = 32'h0000_000B;        // Environment call from M-mode
        else
            trap_cause_sel = 32'h0000_0003;        // Breakpoint
    end

    // -----------------------------------------------------------------------
    // Trap / mret redirect signals
    // -----------------------------------------------------------------------
    assign trap_taken  = do_trap;
    assign trap_vector = mtvec;
    assign mret_taken  = ex_mret;
    assign mret_pc     = mepc;

    // To CSR
    assign trap_enter  = do_trap;
    assign trap_cause  = trap_cause_sel;
    assign trap_pc     = ex_pc;
    assign trap_val    = '0;   // extended: populate for illegal instr, load/store faults

    // -----------------------------------------------------------------------
    // Stall logic
    // -----------------------------------------------------------------------
    // Any downstream stall forces all upstream stages to stall too.
    logic any_stall;
    assign any_stall = mem_stall | ex_stall | load_use_stall;

    // When mem_stall: freeze everything through MEM
    // When ex_stall: freeze IF, ID, EX (MEM can continue — it holds its reg)
    // When load_use_stall: freeze IF and ID, let EX advance (with bubble)

    assign stall_if  = any_stall;
    assign stall_id  = any_stall;
    assign stall_ex  = mem_stall | ex_stall;
    assign stall_mem = mem_stall;

    // -----------------------------------------------------------------------
    // Flush logic
    // -----------------------------------------------------------------------
    // Flush is gated off during stall (can't flush while stalled).
    // Priority: trap > mret > branch
    logic do_flush_branch;
    assign do_flush_branch = branch_taken && !do_trap && !ex_mret;

    // IF flush: insert bubble from next cycle (branch taken / trap)
    assign flush_if  = !any_stall && (do_flush_branch | do_trap | ex_mret);

    // ID flush: squash instruction in decode (1-cycle pipeline flush)
    assign flush_id  = !any_stall && (do_flush_branch | do_trap | ex_mret);

    // EX flush: on trap or mret, squash the trapping instruction itself
    assign flush_ex  = !any_stall && (do_trap | ex_mret);

    // MEM flush: not needed for branch (branch resolves in EX, after MEM is clear)
    assign flush_mem = 1'b0;

    // -----------------------------------------------------------------------
    // Debug state
    // -----------------------------------------------------------------------
    assign dbg_pipeline_state = {stall_mem, stall_ex, stall_id, stall_if};

endmodule

`default_nettype wire
