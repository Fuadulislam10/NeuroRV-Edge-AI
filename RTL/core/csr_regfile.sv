// ============================================================================
// NeuroRV Edge — CSR Register File
// FILE: rtl/core/csr_regfile.sv
//
// Machine-mode CSR subset for RV32IM
//   • mstatus, misa, mie, mtvec, mscratch, mepc, mcause, mtval, mip
//   • mcycle / mcycleh, minstret / minstreth
//   • Performance counter inhibit (mcountinhibit)
//   • Read/Write/Set/Clear operations
//   • Illegal CSR access detection
//
// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2024-2025 NeuroRV Edge Contributors
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

// CSR address definitions
`define CSR_MSTATUS      12'h300
`define CSR_MISA         12'h301
`define CSR_MIE          12'h304
`define CSR_MTVEC        12'h305
`define CSR_MCOUNTINH    12'h320
`define CSR_MSCRATCH     12'h340
`define CSR_MEPC         12'h341
`define CSR_MCAUSE       12'h342
`define CSR_MTVAL        12'h343
`define CSR_MIP          12'h344
`define CSR_MCYCLE       12'hB00
`define CSR_MINSTRET     12'hB02
`define CSR_MCYCLEH      12'hB80
`define CSR_MINSTRETH    12'hB82
`define CSR_MVENDORID    12'hF11
`define CSR_MARCHID      12'hF12
`define CSR_MIMPID       12'hF13
`define CSR_MHARTID      12'hF14

// CSR operation encoding (funct3)
`define CSR_RW   3'b001
`define CSR_RS   3'b010
`define CSR_RC   3'b011
`define CSR_RWI  3'b101
`define CSR_RSI  3'b110
`define CSR_RCI  3'b111

module csr_regfile #(
    parameter logic [31:0] HART_ID    = 32'h0,
    parameter logic [31:0] VENDOR_ID  = 32'h4E524556,  // "NREV"
    parameter logic [31:0] ARCH_ID    = 32'h1,
    parameter logic [31:0] IMP_ID     = 32'h0100        // v1.0
) (
    input  logic        clk,
    input  logic        rst_n,

    // CSR access port (from EX stage)
    input  logic        csr_en,          // CSR instruction valid
    input  logic [11:0] csr_addr,        // CSR address
    input  logic [2:0]  csr_op,          // operation (funct3)
    input  logic [31:0] csr_wdata,       // write data (rs1 or zimm)
    output logic [31:0] csr_rdata,       // read data
    output logic        csr_illegal,     // illegal access

    // Trap/interrupt interface
    input  logic        trap_enter,      // entering trap
    input  logic        trap_ret,        // mret instruction
    input  logic [31:0] trap_cause,      // mcause value
    input  logic [31:0] trap_pc,         // PC of trapping instruction
    input  logic [31:0] trap_val,        // mtval value
    output logic [31:0] mtvec_out,       // trap vector base
    output logic [31:0] mepc_out,        // exception PC (for mret)
    output logic        mstatus_mie,     // global interrupt enable

    // External interrupt lines
    input  logic        irq_external,    // MEIP
    input  logic        irq_timer,       // MTIP
    input  logic        irq_software,    // MSIP

    // Instruction retired pulse (from WB)
    input  logic        instret_inc,

    // Debug outputs
    output logic [31:0] dbg_mstatus,
    output logic [31:0] dbg_mcause,
    output logic [31:0] dbg_mepc
);

    // -----------------------------------------------------------------------
    // CSR storage registers
    // -----------------------------------------------------------------------
    logic [31:0] mstatus;
    logic [31:0] mie;
    logic [31:0] mtvec;
    logic [31:0] mscratch;
    logic [31:0] mepc;
    logic [31:0] mcause;
    logic [31:0] mtval;
    logic [31:0] mip;
    logic [31:0] mcountinhibit;
    logic [63:0] mcycle_r;
    logic [63:0] minstret_r;

    // mstatus fields (M-mode only, RV32)
    // [3]  MIE  — machine interrupt enable
    // [7]  MPIE — previous MIE
    // [12:11] MPP = 2'b11 (always M-mode)
    localparam logic [31:0] MSTATUS_MASK = 32'h0000_1888;
    localparam logic [31:0] MISA_VAL     = 32'h4000_1100;
    // MISA: MXL=01 (RV32), I=1 (bit8), M=1 (bit12)

    // -----------------------------------------------------------------------
    // Read multiplexer
    // -----------------------------------------------------------------------
    always_comb begin
        csr_rdata   = '0;
        csr_illegal = 1'b0;

        unique case (csr_addr)
            `CSR_MSTATUS:   csr_rdata = mstatus;
            `CSR_MISA:      csr_rdata = MISA_VAL;
            `CSR_MIE:       csr_rdata = mie;
            `CSR_MTVEC:     csr_rdata = mtvec;
            `CSR_MCOUNTINH: csr_rdata = mcountinhibit;
            `CSR_MSCRATCH:  csr_rdata = mscratch;
            `CSR_MEPC:      csr_rdata = mepc;
            `CSR_MCAUSE:    csr_rdata = mcause;
            `CSR_MTVAL:     csr_rdata = mtval;
            `CSR_MIP:       csr_rdata = mip;
            `CSR_MCYCLE:    csr_rdata = mcycle_r[31:0];
            `CSR_MCYCLEH:   csr_rdata = mcycle_r[63:32];
            `CSR_MINSTRET:  csr_rdata = minstret_r[31:0];
            `CSR_MINSTRETH: csr_rdata = minstret_r[63:32];
            `CSR_MVENDORID: csr_rdata = VENDOR_ID;
            `CSR_MARCHID:   csr_rdata = ARCH_ID;
            `CSR_MIMPID:    csr_rdata = IMP_ID;
            `CSR_MHARTID:   csr_rdata = HART_ID;
            default: begin
                csr_rdata   = '0;
                csr_illegal = csr_en;
            end
        endcase
    end

    // -----------------------------------------------------------------------
    // Write data calculation
    // -----------------------------------------------------------------------
    logic [31:0] wdata_next;

    always_comb begin
        unique case (csr_op)
            `CSR_RW,  `CSR_RWI: wdata_next = csr_wdata;
            `CSR_RS,  `CSR_RSI: wdata_next = csr_rdata | csr_wdata;
            `CSR_RC,  `CSR_RCI: wdata_next = csr_rdata & ~csr_wdata;
            default:             wdata_next = csr_wdata;
        endcase
    end

    logic do_write;
    assign do_write = csr_en && !csr_illegal && !trap_enter && !trap_ret;

    // -----------------------------------------------------------------------
    // Synchronous CSR updates
    // -----------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mstatus      <= 32'h0000_1800;  // MPP=11 (M-mode)
            mie          <= '0;
            mtvec        <= '0;
            mscratch     <= '0;
            mepc         <= '0;
            mcause       <= '0;
            mtval        <= '0;
            mip          <= '0;
            mcountinhibit<= '0;
            mcycle_r     <= '0;
            minstret_r   <= '0;
        end else begin
            // ---- Cycle counter (always counting unless inhibited)
            if (!mcountinhibit[0])
                mcycle_r <= mcycle_r + 1;

            // ---- Instruction retire counter
            if (instret_inc && !mcountinhibit[2])
                minstret_r <= minstret_r + 1;

            // ---- External interrupt pending (level-sensitive)
            mip[11] <= irq_external;   // MEIP
            mip[7]  <= irq_timer;      // MTIP
            mip[3]  <= irq_software;   // MSIP

            // ---- Trap entry (overrides normal CSR write)
            if (trap_enter) begin
                // Save MIE into MPIE, clear MIE
                mstatus[7] <= mstatus[3];   // MPIE ← MIE
                mstatus[3] <= 1'b0;          // MIE  ← 0
                mstatus[12:11] <= 2'b11;     // MPP  ← M-mode
                mepc   <= {trap_pc[31:2], 2'b00};
                mcause <= trap_cause;
                mtval  <= trap_val;
            end

            // ---- Trap return (mret)
            else if (trap_ret) begin
                mstatus[3] <= mstatus[7];    // MIE  ← MPIE
                mstatus[7] <= 1'b1;           // MPIE ← 1
                mstatus[12:11] <= 2'b11;      // MPP  ← M-mode (stay M)
            end

            // ---- Normal CSR write
            else if (do_write) begin
                unique case (csr_addr)
                    `CSR_MSTATUS:   mstatus      <= wdata_next & MSTATUS_MASK
                                                  | (mstatus & ~MSTATUS_MASK);
                    `CSR_MIE:       mie          <= wdata_next & 32'h0000_0888;
                    `CSR_MTVEC:     mtvec        <= {wdata_next[31:2], 1'b0, wdata_next[0]};
                    `CSR_MCOUNTINH: mcountinhibit<= wdata_next & 32'h0000_0005;
                    `CSR_MSCRATCH:  mscratch     <= wdata_next;
                    `CSR_MEPC:      mepc         <= {wdata_next[31:2], 2'b00};
                    `CSR_MCAUSE:    mcause       <= wdata_next;
                    `CSR_MTVAL:     mtval        <= wdata_next;
                    `CSR_MCYCLE:    mcycle_r[31:0]    <= wdata_next;
                    `CSR_MCYCLEH:   mcycle_r[63:32]   <= wdata_next;
                    `CSR_MINSTRET:  minstret_r[31:0]  <= wdata_next;
                    `CSR_MINSTRETH: minstret_r[63:32] <= wdata_next;
                    default: ; // read-only or unimplemented
                endcase
            end
        end
    end

    // -----------------------------------------------------------------------
    // Outputs
    // -----------------------------------------------------------------------
    assign mtvec_out    = mtvec;
    assign mepc_out     = mepc;
    assign mstatus_mie  = mstatus[3];

    // Debug
    assign dbg_mstatus  = mstatus;
    assign dbg_mcause   = mcause;
    assign dbg_mepc     = mepc;

endmodule

`default_nettype wire
