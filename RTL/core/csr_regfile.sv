// =============================================================================
// NeuroRV Edge — CSR Register File (Machine Mode)
// File   : rtl/core/csr_regfile.sv
//
// Implements mandatory CSRs per RISC-V Privileged Spec v1.11:
//   misa, mvendorid, marchid, mimpid, mhartid,
//   mstatus, mtvec, mip, mie,
//   mscratch, mepc, mcause, mtval,
//   mcycle[h], minstret[h]
// =============================================================================
`timescale 1ns/1ps
`default_nettype none

// CSR addresses
`define CSR_MSTATUS    12'h300
`define CSR_MISA       12'h301
`define CSR_MIE        12'h304
`define CSR_MTVEC      12'h305
`define CSR_MSCRATCH   12'h340
`define CSR_MEPC       12'h341
`define CSR_MCAUSE     12'h342
`define CSR_MTVAL      12'h343
`define CSR_MIP        12'h344
`define CSR_MCYCLE     12'hB00
`define CSR_MINSTRET   12'hB02
`define CSR_MCYCLEH    12'hB80
`define CSR_MINSTRETH  12'hB82
`define CSR_MHARTID    12'hF14
`define CSR_MVENDORID  12'hF11
`define CSR_MARCHID    12'hF12
`define CSR_MIMPID     12'hF13

module csr_regfile #(
    parameter int MHARTID   = 0,
    parameter     MVENDORID = 32'h0000_0000,
    parameter     MARCHID   = 32'h0000_0001,  // NeuroRV custom
    parameter     MIMPID    = 32'h0100_0000   // v1.0.0
) (
    input  logic        clk_i,
    input  logic        rst_ni,
    // Read/Write interface (from EX stage)
    input  logic [11:0] csr_addr_i,
    input  logic [31:0] csr_wdata_i,
    input  logic        csr_we_i,
    output logic [31:0] csr_rdata_o,
    // External interrupt sources
    input  logic        irq_external_i,
    input  logic        irq_timer_i,
    input  logic        irq_software_i,
    // Trap interface
    input  logic        trap_taken_i,
    input  logic [31:0] trap_cause_i,
    input  logic [31:0] trap_pc_i,
    input  logic [31:0] trap_val_i,
    // Outputs to pipeline
    output logic [31:0] mtvec_o,
    output logic [31:0] mepc_o,
    output logic        mstatus_mie_o,
    output logic [63:0] mcycle_o,
    output logic [63:0] minstret_o
);

    // -------------------------------------------------------------------------
    // CSR registers
    // -------------------------------------------------------------------------
    logic [31:0] mstatus_r;
    logic [31:0] mtvec_r;
    logic [31:0] mscratch_r;
    logic [31:0] mepc_r;
    logic [31:0] mcause_r;
    logic [31:0] mtval_r;
    logic [31:0] mie_r;
    logic [63:0] mcycle_r;
    logic [63:0] minstret_r;

    // mstatus fields: only MIE (bit3) and MPIE (bit7) implemented
    // MPP (bits 12:11) = 2'b11 (machine mode, always)
    logic mie_bit, mpie_bit;
    assign mie_bit  = mstatus_r[3];
    assign mpie_bit = mstatus_r[7];

    // mip — read-only, driven by external signals
    logic [31:0] mip;
    assign mip = {20'h0, irq_external_i, 3'h0, irq_timer_i, 3'h0, irq_software_i, 3'h0};

    // misa: RV32IM (M=1, I=1), MXL=01 (32-bit)
    logic [31:0] misa;
    assign misa = {2'b01, 4'h0, 26'b000_0000_0001_0001_0000_0000_0000};
    //                              Z...M        I

    // -------------------------------------------------------------------------
    // CSR read
    // -------------------------------------------------------------------------
    always_comb begin
        csr_rdata_o = '0;
        unique case (csr_addr_i)
            `CSR_MSTATUS:   csr_rdata_o = mstatus_r;
            `CSR_MISA:      csr_rdata_o = misa;
            `CSR_MIE:       csr_rdata_o = mie_r;
            `CSR_MTVEC:     csr_rdata_o = mtvec_r;
            `CSR_MSCRATCH:  csr_rdata_o = mscratch_r;
            `CSR_MEPC:      csr_rdata_o = {mepc_r[31:2], 2'b00};
            `CSR_MCAUSE:    csr_rdata_o = mcause_r;
            `CSR_MTVAL:     csr_rdata_o = mtval_r;
            `CSR_MIP:       csr_rdata_o = mip;
            `CSR_MCYCLE:    csr_rdata_o = mcycle_r[31:0];
            `CSR_MCYCLEH:   csr_rdata_o = mcycle_r[63:32];
            `CSR_MINSTRET:  csr_rdata_o = minstret_r[31:0];
            `CSR_MINSTRETH: csr_rdata_o = minstret_r[63:32];
            `CSR_MHARTID:   csr_rdata_o = 32'(MHARTID);
            `CSR_MVENDORID: csr_rdata_o = MVENDORID;
            `CSR_MARCHID:   csr_rdata_o = MARCHID;
            `CSR_MIMPID:    csr_rdata_o = MIMPID;
            default:        csr_rdata_o = '0;
        endcase
    end

    // -------------------------------------------------------------------------
    // CSR write
    // -------------------------------------------------------------------------
    logic [31:0] wdata;
    assign wdata = csr_wdata_i; // CSRRW mode; CSRRS/CSRRC handled in ex_stage

    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            mstatus_r  <= 32'h0000_1800; // MPP=11 (machine mode)
            mtvec_r    <= 32'h0000_0000;
            mscratch_r <= '0;
            mepc_r     <= '0;
            mcause_r   <= '0;
            mtval_r    <= '0;
            mie_r      <= '0;
            mcycle_r   <= '0;
            minstret_r <= '0;
        end else begin
            // Increment counters
            mcycle_r <= mcycle_r + 64'd1;

            // Trap: save state
            if (trap_taken_i) begin
                mepc_r    <= trap_pc_i;
                mcause_r  <= trap_cause_i;
                mtval_r   <= trap_val_i;
                // MPIE = MIE, MIE = 0
                mstatus_r <= {mstatus_r[31:8], mstatus_r[3], mstatus_r[6:4],
                               1'b0, mstatus_r[2:0]};
            end

            // CSR write
            if (csr_we_i) begin
                unique case (csr_addr_i)
                    `CSR_MSTATUS:  mstatus_r  <= wdata & 32'h0000_1888; // mask writable bits
                    `CSR_MIE:      mie_r      <= wdata & 32'h0000_0888;
                    `CSR_MTVEC:    mtvec_r    <= {wdata[31:2], 1'b0, wdata[0]}; // only direct/vectored
                    `CSR_MSCRATCH: mscratch_r <= wdata;
                    `CSR_MEPC:     mepc_r     <= {wdata[31:2], 2'b00};
                    `CSR_MCAUSE:   mcause_r   <= wdata;
                    `CSR_MTVAL:    mtval_r    <= wdata;
                    `CSR_MCYCLE:   mcycle_r[31:0]    <= wdata;
                    `CSR_MCYCLEH:  mcycle_r[63:32]   <= wdata;
                    `CSR_MINSTRET: minstret_r[31:0]  <= wdata;
                    `CSR_MINSTRETH: minstret_r[63:32] <= wdata;
                    default: ; // ignore writes to read-only CSRs
                endcase
            end
        end
    end

    // Instruction retired counter (increment when valid instruction commits)
    // Driven externally or from WB stage — stub here
    // assign minstret_inc = wb_valid; // would be wired from WB

    assign mtvec_o       = mtvec_r;
    assign mepc_o        = mepc_r;
    assign mstatus_mie_o = mstatus_r[3];
    assign mcycle_o      = mcycle_r;
    assign minstret_o    = minstret_r;

endmodule

`default_nettype wire
