// ============================================================================
// NeuroRV Edge — Register File
// FILE: rtl/core/regfile.sv
//
// RV32I Integer Register File
//   • 32 × 32-bit general-purpose registers (x0–x31)
//   • x0 hardwired to zero
//   • 2 asynchronous read ports (rs1, rs2)
//   • 1 synchronous write port (rd)
//   • Write-before-read: write data visible on same-cycle read if WB→ID bypass
//     is handled externally (forwarding); the register file itself is simple
//     synchronous write / asynchronous read.
//
// Synthesizable with Yosys; FPGA-compatible (infers distributed RAM or LUT-RAM).
//
// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2024-2025 NeuroRV Edge Contributors
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

module regfile #(
    parameter int unsigned DATA_WIDTH = 32,
    parameter int unsigned ADDR_WIDTH = 5     // log2(32) = 5
) (
    input  logic                  clk,
    input  logic                  rst_n,

    // Read port A (rs1)
    input  logic [ADDR_WIDTH-1:0] rs1_addr,
    output logic [DATA_WIDTH-1:0] rs1_data,

    // Read port B (rs2)
    input  logic [ADDR_WIDTH-1:0] rs2_addr,
    output logic [DATA_WIDTH-1:0] rs2_data,

    // Write port (rd)
    input  logic                  we,
    input  logic [ADDR_WIDTH-1:0] rd_addr,
    input  logic [DATA_WIDTH-1:0] rd_data,

    // Debug: expose full register state
    output logic [DATA_WIDTH-1:0] dbg_regs [0:31]
);

    // -----------------------------------------------------------------------
    // Register array — 32 × 32-bit
    // -----------------------------------------------------------------------
    logic [DATA_WIDTH-1:0] regs [1:31];   // x0 is implicit zero

    // -----------------------------------------------------------------------
    // Synchronous write (x0 writes are silently ignored)
    // -----------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int i = 1; i < 32; i++) regs[i] <= '0;
        end else if (we && (rd_addr != '0)) begin
            regs[rd_addr] <= rd_data;
        end
    end

    // -----------------------------------------------------------------------
    // Asynchronous reads (x0 always reads as zero)
    // -----------------------------------------------------------------------
    assign rs1_data = (rs1_addr == '0) ? '0 : regs[rs1_addr];
    assign rs2_data = (rs2_addr == '0) ? '0 : regs[rs2_addr];

    // -----------------------------------------------------------------------
    // Debug output: full register file view
    // -----------------------------------------------------------------------
    assign dbg_regs[0] = '0;
    for (genvar i = 1; i < 32; i++) begin : gen_dbg
        assign dbg_regs[i] = regs[i];
    end

endmodule

`default_nettype wire
