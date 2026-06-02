// ============================================================================
// NeuroRV Edge — Writeback Stage (WB)
// FILE: rtl/core/wb_stage.sv
//
// Responsibilities:
//   • Select writeback data source (ALU, load data, PC+4, CSR)
//   • Drive register file write port
//   • Instruction retire counter pulse
//   • Forward writeback value to EX stage
//
// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2024-2025 NeuroRV Edge Contributors
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

typedef enum logic [1:0] {
    WB_SRC_ALU = 2'b00,
    WB_SRC_MEM = 2'b01,
    WB_SRC_PC4 = 2'b10,
    WB_SRC_CSR = 2'b11
} wb_wb_src_e;

module wb_stage (
    input  logic        clk,
    input  logic        rst_n,

    // From MEM/WB pipeline register
    input  logic [31:0] mem_wb_alu_result,
    input  logic [31:0] mem_wb_load_data,
    input  logic [4:0]  mem_wb_rd_addr,
    input  logic        mem_wb_rf_we,
    input  wb_wb_src_e  mem_wb_wb_src,
    input  logic [31:0] mem_wb_csr_rdata,
    input  logic [31:0] mem_wb_pc,
    input  logic        mem_wb_valid,

    // Register file write port
    output logic        rf_we,
    output logic [4:0]  rf_rd_addr,
    output logic [31:0] rf_rd_data,

    // Instruction retire pulse to CSR
    output logic        instret_inc,

    // Forwarding: wb data available to EX stage
    output logic [31:0] wb_fwd_data,
    output logic        wb_fwd_we,
    output logic [4:0]  wb_fwd_addr,

    // Debug
    output logic [31:0] dbg_wb_data,
    output logic [4:0]  dbg_wb_addr,
    output logic        dbg_wb_we
);

    // -----------------------------------------------------------------------
    // Writeback data mux
    // -----------------------------------------------------------------------
    logic [31:0] wb_data;

    always_comb begin
        unique case (mem_wb_wb_src)
            WB_SRC_ALU: wb_data = mem_wb_alu_result;
            WB_SRC_MEM: wb_data = mem_wb_load_data;
            WB_SRC_PC4: wb_data = mem_wb_pc + 32'd4;
            WB_SRC_CSR: wb_data = mem_wb_csr_rdata;
            default:    wb_data = mem_wb_alu_result;
        endcase
    end

    // -----------------------------------------------------------------------
    // Register file write
    // -----------------------------------------------------------------------
    assign rf_we      = mem_wb_rf_we && mem_wb_valid;
    assign rf_rd_addr = mem_wb_rd_addr;
    assign rf_rd_data = wb_data;

    // -----------------------------------------------------------------------
    // Instruction retired
    // -----------------------------------------------------------------------
    assign instret_inc = mem_wb_valid;

    // -----------------------------------------------------------------------
    // Forwarding outputs
    // -----------------------------------------------------------------------
    assign wb_fwd_data = wb_data;
    assign wb_fwd_we   = rf_we;
    assign wb_fwd_addr = rf_rd_addr;

    // -----------------------------------------------------------------------
    // Debug
    // -----------------------------------------------------------------------
    assign dbg_wb_data = wb_data;
    assign dbg_wb_addr = rf_rd_addr;
    assign dbg_wb_we   = rf_we;

endmodule

`default_nettype wire
