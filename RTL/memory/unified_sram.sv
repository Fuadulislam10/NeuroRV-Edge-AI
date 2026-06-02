// ============================================================================
// FILE: rtl/memory/unified_sram.sv
// PROJECT: NeuroRV Edge — Phase 4 Memory Subsystem
// MODULE: unified_sram
// DESCRIPTION: Synchronous dual-port capable SRAM with byte-enable support,
//              optional parity, and single-cycle read/write latency.
//
// PARAMETERS:
//   SRAM_SIZE_BYTES : Total SRAM size in bytes (default 512KB)
//   DATA_W          : Data bus width in bits     (default 32)
//   BE_W            : Byte-enable width = DATA_W/8 (default 4)
//   ADDR_W          : Word address width          (auto from size)
//   PARITY_EN       : 1 = include parity bits per byte lane
//
// PORT A = CPU / primary master  (highest priority)
// PORT B = VXU / DMA secondary   (lower priority, blocked if conflict)
//
// ARBITRATION: Simple two-port with collision detection.
//   On same-cycle address collision: Port A wins; Port B stalls one cycle.
//
// LATENCY:  Read:  1 cycle (data valid cycle after request)
//           Write: 1 cycle (data written on clock edge)
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

module unified_sram #(
    parameter int SRAM_SIZE_BYTES = 512 * 1024,   // 512 KB
    parameter int DATA_W          = 32,
    parameter int PARITY_EN       = 1,
    // Derived
    parameter int BE_W            = DATA_W / 8,
    parameter int WORDS           = SRAM_SIZE_BYTES / (DATA_W / 8),
    parameter int ADDR_W          = $clog2(WORDS)
)(
    input  logic                clk,
    input  logic                rst_n,

    // -----------------------------------------------------------------------
    // Port A — CPU / Primary Master
    // -----------------------------------------------------------------------
    input  logic                pa_req,       // Request (read or write)
    input  logic                pa_wr,        // 1=write 0=read
    input  logic [ADDR_W-1:0]   pa_addr,      // Word address
    input  logic [DATA_W-1:0]   pa_wdata,     // Write data
    input  logic [BE_W-1:0]     pa_be,        // Byte enables (write)
    output logic [DATA_W-1:0]   pa_rdata,     // Read data
    output logic                pa_ack,       // Request granted this cycle
    output logic                pa_stall,     // Port A stalled (should not happen — PA always wins)

    // -----------------------------------------------------------------------
    // Port B — VXU / DMA Secondary Master
    // -----------------------------------------------------------------------
    input  logic                pb_req,
    input  logic                pb_wr,
    input  logic [ADDR_W-1:0]   pb_addr,
    input  logic [DATA_W-1:0]   pb_wdata,
    input  logic [BE_W-1:0]     pb_be,
    output logic [DATA_W-1:0]   pb_rdata,
    output logic                pb_ack,
    output logic                pb_stall,     // Port B stalled due to conflict

    // -----------------------------------------------------------------------
    // Parity error outputs (optional, driven only when PARITY_EN=1)
    // -----------------------------------------------------------------------
    output logic [BE_W-1:0]     pa_parity_err,
    output logic [BE_W-1:0]     pb_parity_err,

    // -----------------------------------------------------------------------
    // Debug
    // -----------------------------------------------------------------------
    output logic [31:0]         dbg_pa_txn_count,   // Total PA transactions
    output logic [31:0]         dbg_pb_txn_count,   // Total PB transactions
    output logic [31:0]         dbg_collision_count, // Address collisions
    output logic [31:0]         dbg_pb_stall_count   // PB stall cycles
);

    // =========================================================================
    // Memory Array
    // DATA_W bits per word + optional parity (1 bit per byte lane)
    // =========================================================================
    localparam int PARITY_W = PARITY_EN ? BE_W : 0;
    localparam int WORD_W   = DATA_W + PARITY_W;

    logic [WORD_W-1:0] mem [0:WORDS-1];

    // =========================================================================
    // Parity generation per byte lane
    // =========================================================================
    function automatic logic byte_parity(input logic [7:0] d);
        return ^d;   // Even parity (XOR of all bits)
    endfunction

    // =========================================================================
    // Collision detection
    // Collision = both ports request the same word address in the same cycle
    // =========================================================================
    logic collision;
    assign collision = pa_req && pb_req && (pa_addr == pb_addr);

    // Port B stall: stall whenever Port A is active (highest priority)
    // For non-conflicting addresses, allow both to proceed simultaneously.
    // For conflicting addresses, Port B is stalled for one cycle.
    logic pb_addr_conflict;
    assign pb_addr_conflict = pa_req && pb_req && (pa_addr == pb_addr);

    assign pa_stall = 1'b0;                   // PA never stalls
    assign pb_stall = pb_addr_conflict;        // PB stalls only on collision

    assign pa_ack = pa_req;
    assign pb_ack = pb_req && !pb_addr_conflict;

    // =========================================================================
    // Port A — Synchronous SRAM Access (always granted)
    // =========================================================================
    always_ff @(posedge clk) begin
        if (pa_req) begin
            if (pa_wr) begin
                // Byte-enable write
                for (int b = 0; b < BE_W; b++) begin
                    if (pa_be[b]) begin
                        mem[pa_addr][b*8 +: 8] <= pa_wdata[b*8 +: 8];
                        if (PARITY_EN)
                            mem[pa_addr][DATA_W + b] <= byte_parity(pa_wdata[b*8 +: 8]);
                    end
                end
            end
        end
    end

    // PA read: registered output (1-cycle latency)
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pa_rdata <= '0;
        end else if (pa_req && !pa_wr) begin
            pa_rdata <= mem[pa_addr][DATA_W-1:0];
        end
    end

    // =========================================================================
    // Port B — Synchronous SRAM Access (stalled on collision)
    // =========================================================================
    always_ff @(posedge clk) begin
        if (pb_ack && pb_wr) begin
            for (int b = 0; b < BE_W; b++) begin
                if (pb_be[b]) begin
                    mem[pb_addr][b*8 +: 8] <= pb_wdata[b*8 +: 8];
                    if (PARITY_EN)
                        mem[pb_addr][DATA_W + b] <= byte_parity(pb_wdata[b*8 +: 8]);
                end
            end
        end
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pb_rdata <= '0;
        end else if (pb_ack && !pb_wr) begin
            pb_rdata <= mem[pb_addr][DATA_W-1:0];
        end
    end

    // =========================================================================
    // Parity Check (read path)
    // =========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pa_parity_err <= '0;
            pb_parity_err <= '0;
        end else begin
            pa_parity_err <= '0;
            pb_parity_err <= '0;
            if (PARITY_EN) begin
                if (pa_req && !pa_wr) begin
                    for (int b = 0; b < BE_W; b++) begin
                        pa_parity_err[b] <= byte_parity(mem[pa_addr][b*8 +: 8])
                                            ^ mem[pa_addr][DATA_W + b];
                    end
                end
                if (pb_ack && !pb_wr) begin
                    for (int b = 0; b < BE_W; b++) begin
                        pb_parity_err[b] <= byte_parity(mem[pb_addr][b*8 +: 8])
                                            ^ mem[pb_addr][DATA_W + b];
                    end
                end
            end
        end
    end

    // =========================================================================
    // Debug Counters
    // =========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dbg_pa_txn_count    <= '0;
            dbg_pb_txn_count    <= '0;
            dbg_collision_count <= '0;
            dbg_pb_stall_count  <= '0;
        end else begin
            if (pa_ack)          dbg_pa_txn_count    <= dbg_pa_txn_count    + 1;
            if (pb_ack)          dbg_pb_txn_count    <= dbg_pb_txn_count    + 1;
            if (collision)       dbg_collision_count <= dbg_collision_count + 1;
            if (pb_stall)        dbg_pb_stall_count  <= dbg_pb_stall_count  + 1;
        end
    end

    // =========================================================================
    // Reset: initialize memory to zero (simulation + FPGA block RAM init)
    // For ASIC: synthesis typically zeros SRAM on power-up via init files
    // =========================================================================
    // synthesis translate_off
    initial begin
        for (int i = 0; i < WORDS; i++) mem[i] = '0;
        $display("unified_sram: initialized %0d words (%0d KB)",
                 WORDS, SRAM_SIZE_BYTES / 1024);
    end
    // synthesis translate_on

    // =========================================================================
    // Parameter assertions
    // =========================================================================
    // synthesis translate_off
    initial begin
        assert (DATA_W % 8 == 0)
            else $fatal(1, "unified_sram: DATA_W must be a multiple of 8");
        assert (SRAM_SIZE_BYTES % (DATA_W/8) == 0)
            else $fatal(1, "unified_sram: SRAM_SIZE_BYTES must be divisible by word size");
        assert (WORDS >= 4)
            else $fatal(1, "unified_sram: Too small — WORDS must be >= 4");
    end
    // synthesis translate_on

endmodule

`default_nettype wire
