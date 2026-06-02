// ============================================================================
// NeuroRV Edge — Memory Access Stage (MEM)
// FILE: rtl/core/mem_stage.sv
//
// Responsibilities:
//   • Data memory request generation (load / store)
//   • Load data sign/zero extension (LB, LBU, LH, LHU, LW)
//   • Byte-enable generation for stores (SB, SH, SW)
//   • MEM/WB pipeline register
//
// Data memory interface: word-addressed, synchronous SRAM with byte enables.
//
// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2024-2025 NeuroRV Edge Contributors
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

typedef enum logic [1:0] {
    MEM_WB_ALU = 2'b00, MEM_WB_MEM = 2'b01,
    MEM_WB_PC4 = 2'b10, MEM_WB_CSR = 2'b11
} mem_wb_src_e;

module mem_stage (
    input  logic        clk,
    input  logic        rst_n,

    // From EX/MEM pipeline register
    input  logic [31:0] ex_mem_pc,
    input  logic [31:0] ex_mem_alu_result,   // effective address (load/store) or result
    input  logic [31:0] ex_mem_rs2_data,     // store data
    input  logic [4:0]  ex_mem_rd_addr,
    input  logic        ex_mem_is_load,
    input  logic        ex_mem_is_store,
    input  logic [2:0]  ex_mem_mem_width,    // funct3
    input  logic        ex_mem_rf_we,
    input  mem_wb_src_e ex_mem_wb_src,
    input  logic [31:0] ex_mem_csr_rdata,
    input  logic        ex_mem_valid,

    // Stall / Flush
    input  logic        stall_mem,
    input  logic        flush_mem,

    // Data memory interface
    output logic        dmem_req,
    output logic        dmem_we,
    output logic [3:0]  dmem_be,            // byte enable
    output logic [31:0] dmem_addr,
    output logic [31:0] dmem_wdata,
    input  logic [31:0] dmem_rdata,
    input  logic        dmem_valid,         // data ready

    // MEM/WB pipeline register outputs
    output logic [31:0] mem_wb_alu_result,
    output logic [31:0] mem_wb_load_data,
    output logic [4:0]  mem_wb_rd_addr,
    output logic        mem_wb_rf_we,
    output mem_wb_src_e mem_wb_wb_src,
    output logic [31:0] mem_wb_csr_rdata,
    output logic [31:0] mem_wb_pc,
    output logic        mem_wb_valid,

    // Stall request (data memory not ready)
    output logic        mem_stall,

    // Debug
    output logic [31:0] dbg_dmem_addr,
    output logic [31:0] dbg_dmem_wdata,
    output logic [31:0] dbg_dmem_rdata
);

    // -----------------------------------------------------------------------
    // Address alignment and byte-enable generation
    // -----------------------------------------------------------------------
    logic [1:0] byte_off;
    assign byte_off = ex_mem_alu_result[1:0];

    // Byte enables for store
    logic [3:0] be_mux;
    always_comb begin
        unique case (ex_mem_mem_width[1:0])
            2'b00: begin   // SB
                be_mux = 4'b0001 << byte_off;
            end
            2'b01: begin   // SH
                be_mux = (byte_off[1]) ? 4'b1100 : 4'b0011;
            end
            2'b10: be_mux = 4'b1111;  // SW
            default: be_mux = 4'b1111;
        endcase
    end

    // Store data shifted to correct byte lanes
    logic [31:0] store_data_shifted;
    always_comb begin
        unique case (ex_mem_mem_width[1:0])
            2'b00: store_data_shifted = {4{ex_mem_rs2_data[7:0]}};   // SB: replicate byte
            2'b01: store_data_shifted = {2{ex_mem_rs2_data[15:0]}};  // SH: replicate half
            default: store_data_shifted = ex_mem_rs2_data;            // SW: full word
        endcase
    end

    // -----------------------------------------------------------------------
    // Memory request outputs
    // -----------------------------------------------------------------------
    assign dmem_req   = (ex_mem_is_load | ex_mem_is_store) & ex_mem_valid;
    assign dmem_we    = ex_mem_is_store;
    assign dmem_be    = ex_mem_is_store ? be_mux : 4'b1111;
    assign dmem_addr  = {ex_mem_alu_result[31:2], 2'b00};  // word-aligned
    assign dmem_wdata = store_data_shifted;

    // Stall pipeline if load data not ready
    assign mem_stall = ex_mem_is_load && ex_mem_valid && !dmem_valid;

    // -----------------------------------------------------------------------
    // Load data sign/zero extension
    // -----------------------------------------------------------------------
    logic [31:0] load_data;
    logic [7:0]  load_byte;
    logic [15:0] load_half;

    // Select the correct byte/half from the returned word
    always_comb begin
        unique case (byte_off)
            2'b00: load_byte = dmem_rdata[7:0];
            2'b01: load_byte = dmem_rdata[15:8];
            2'b10: load_byte = dmem_rdata[23:16];
            2'b11: load_byte = dmem_rdata[31:24];
            default: load_byte = dmem_rdata[7:0];
        endcase

        load_half = byte_off[1] ? dmem_rdata[31:16] : dmem_rdata[15:0];
    end

    always_comb begin
        unique case (ex_mem_mem_width)
            3'b000: load_data = {{24{load_byte[7]}},   load_byte};    // LB  — signed
            3'b001: load_data = {{16{load_half[15]}},  load_half};    // LH  — signed
            3'b010: load_data = dmem_rdata;                           // LW
            3'b100: load_data = {24'b0, load_byte};                   // LBU — unsigned
            3'b101: load_data = {16'b0, load_half};                   // LHU — unsigned
            default: load_data = dmem_rdata;
        endcase
    end

    // -----------------------------------------------------------------------
    // MEM/WB Pipeline Register
    // -----------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n || flush_mem) begin
            mem_wb_alu_result <= '0;
            mem_wb_load_data  <= '0;
            mem_wb_rd_addr    <= '0;
            mem_wb_rf_we      <= 1'b0;
            mem_wb_wb_src     <= MEM_WB_ALU;
            mem_wb_csr_rdata  <= '0;
            mem_wb_pc         <= '0;
            mem_wb_valid      <= 1'b0;
        end else if (!stall_mem && !mem_stall) begin
            mem_wb_alu_result <= ex_mem_alu_result;
            mem_wb_load_data  <= load_data;
            mem_wb_rd_addr    <= ex_mem_rd_addr;
            mem_wb_rf_we      <= ex_mem_rf_we;
            mem_wb_wb_src     <= ex_mem_wb_src;
            mem_wb_csr_rdata  <= ex_mem_csr_rdata;
            mem_wb_pc         <= ex_mem_pc;
            mem_wb_valid      <= ex_mem_valid && !ex_mem_is_store;
        end
    end

    // Debug
    assign dbg_dmem_addr  = dmem_addr;
    assign dbg_dmem_wdata = dmem_wdata;
    assign dbg_dmem_rdata = dmem_rdata;

endmodule

`default_nettype wire
