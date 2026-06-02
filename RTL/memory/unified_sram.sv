// =============================================================================
// FILE: rtl/memory/unified_sram.sv
// PROJECT: NeuroRV Edge SoC
// MODULE: unified_sram
// DESCRIPTION: 512KB Unified SRAM with byte-enable, optional parity,
//              synchronous R/W, single-cycle latency, dual-port via
//              arbitrated request interface.
// SYNTHESIZABLE: Yes (Yosys + Verilator compatible)
// =============================================================================

`timescale 1ns/1ps

module unified_sram #(
    // Memory parameters
    parameter int unsigned MEM_DEPTH    = 131072,   // 512KB / 4 bytes per word
    parameter int unsigned DATA_WIDTH   = 32,
    parameter int unsigned ADDR_WIDTH   = 17,       // log2(131072) = 17
    parameter int unsigned BYTE_LANES   = 4,        // DATA_WIDTH / 8
    parameter bit          PARITY_EN    = 1'b1,     // Enable parity bit per word
    parameter bit          INIT_ZERO    = 1'b1      // Initialize to zero on reset
)(
    input  logic                    clk,
    input  logic                    rst_n,

    // --- Port A (CPU / Highest Priority) ---
    input  logic                    pa_req,         // Port A request
    input  logic                    pa_we,          // Write enable
    input  logic [ADDR_WIDTH-1:0]   pa_addr,        // Word address
    input  logic [DATA_WIDTH-1:0]   pa_wdata,       // Write data
    input  logic [BYTE_LANES-1:0]   pa_be,          // Byte enables
    output logic [DATA_WIDTH-1:0]   pa_rdata,       // Read data
    output logic                    pa_ack,         // Acknowledge (data valid)
    output logic                    pa_parity_err,  // Parity error flag

    // --- Port B (VXU / DMA / Lower Priority) ---
    input  logic                    pb_req,
    input  logic                    pb_we,
    input  logic [ADDR_WIDTH-1:0]   pb_addr,
    input  logic [DATA_WIDTH-1:0]   pb_wdata,
    input  logic [BYTE_LANES-1:0]   pb_be,
    output logic [DATA_WIDTH-1:0]   pb_rdata,
    output logic                    pb_ack,
    output logic                    pb_parity_err,

    // --- Debug / Status ---
    output logic [31:0]             dbg_access_count,
    output logic [31:0]             dbg_stall_count,
    output logic                    dbg_parity_error_sticky
);

    // =========================================================================
    // Local Parameters
    // =========================================================================
    localparam int unsigned PARITY_BITS = PARITY_EN ? BYTE_LANES : 0;
    localparam int unsigned STORAGE_W   = DATA_WIDTH + PARITY_BITS;

    // =========================================================================
    // Memory Array
    // =========================================================================
    // Each entry stores [parity3|parity2|parity1|parity0 | data[31:0]]
    logic [STORAGE_W-1:0] mem [0:MEM_DEPTH-1];

    // =========================================================================
    // Arbitration Logic (Fixed Priority: Port A > Port B)
    // =========================================================================
    // Port A always wins when both request simultaneously.
    // Port B is stalled exactly one cycle when collision occurs.

    logic sel_a;        // This cycle serves Port A
    logic sel_b;        // This cycle serves Port B
    logic collision;    // Both requested same cycle

    assign collision = pa_req & pb_req;
    assign sel_a     = pa_req;                  // A always served if requesting
    assign sel_b     = pb_req & ~pa_req;        // B served only when A is idle

    // Internal wire mux
    logic                    arb_we;
    logic [ADDR_WIDTH-1:0]   arb_addr;
    logic [DATA_WIDTH-1:0]   arb_wdata;
    logic [BYTE_LANES-1:0]   arb_be;
    logic                    arb_req;

    always_comb begin
        if (sel_a) begin
            arb_we    = pa_we;
            arb_addr  = pa_addr;
            arb_wdata = pa_wdata;
            arb_be    = pa_be;
            arb_req   = 1'b1;
        end else if (sel_b) begin
            arb_we    = pb_we;
            arb_addr  = pb_addr;
            arb_wdata = pb_wdata;
            arb_be    = pb_be;
            arb_req   = 1'b1;
        end else begin
            arb_we    = 1'b0;
            arb_addr  = {ADDR_WIDTH{1'b0}};
            arb_wdata = {DATA_WIDTH{1'b0}};
            arb_be    = {BYTE_LANES{1'b0}};
            arb_req   = 1'b0;
        end
    end

    // =========================================================================
    // Address Range Check
    // =========================================================================
    logic addr_valid;
    assign addr_valid = (arb_addr < ADDR_WIDTH'(MEM_DEPTH));

    // =========================================================================
    // Parity Generation (even parity per byte)
    // =========================================================================
    function automatic logic [BYTE_LANES-1:0] gen_parity(
        input logic [DATA_WIDTH-1:0] data
    );
        logic [BYTE_LANES-1:0] p;
        for (int i = 0; i < BYTE_LANES; i++) begin
            p[i] = ^data[i*8 +: 8];  // Even parity over each byte
        end
        return p;
    endfunction

    function automatic logic [BYTE_LANES-1:0] check_parity(
        input logic [DATA_WIDTH-1:0]  data,
        input logic [PARITY_BITS-1:0] stored_p
    );
        logic [BYTE_LANES-1:0] computed;
        logic [BYTE_LANES-1:0] err;
        computed = gen_parity(data);
        if (PARITY_EN) begin
            err = computed ^ stored_p;
        end else begin
            err = {BYTE_LANES{1'b0}};
        end
        return err;
    endfunction

    // =========================================================================
    // SRAM Read / Write Logic
    // =========================================================================
    // Pipeline register for read data (single-cycle latency)
    logic [DATA_WIDTH-1:0]  rdata_raw;
    logic [PARITY_BITS-1:0] rdata_parity;
    logic                   rdata_valid;
    logic                   rdata_port;     // 0 = Port A, 1 = Port B

    // Write with byte enable and parity update
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            if (INIT_ZERO) begin
                // Synthesis tools treat this as initial state;
                // explicit loop for simulation reset
                for (int i = 0; i < MEM_DEPTH; i++) begin
                    mem[i] <= {STORAGE_W{1'b0}};
                end
            end
        end else if (arb_req && arb_we && addr_valid) begin
            // Byte-granular write with parity update per byte lane
            for (int lane = 0; lane < BYTE_LANES; lane++) begin
                if (arb_be[lane]) begin
                    // Update data byte
                    mem[arb_addr][lane*8 +: 8] <= arb_wdata[lane*8 +: 8];
                    // Update parity bit for this lane if enabled
                    if (PARITY_EN) begin
                        mem[arb_addr][DATA_WIDTH + lane] <= ^arb_wdata[lane*8 +: 8];
                    end
                end
            end
        end
    end

    // Read (synchronous, single-cycle)
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rdata_raw   <= {DATA_WIDTH{1'b0}};
            rdata_parity<= {PARITY_BITS{1'b0}};
            rdata_valid <= 1'b0;
            rdata_port  <= 1'b0;
        end else begin
            rdata_valid <= arb_req & ~arb_we & addr_valid;
            rdata_port  <= sel_b & ~sel_a;  // 1 if Port B was served
            if (arb_req && !arb_we && addr_valid) begin
                rdata_raw    <= mem[arb_addr][DATA_WIDTH-1:0];
                if (PARITY_EN)
                    rdata_parity <= mem[arb_addr][STORAGE_W-1:DATA_WIDTH];
                else
                    rdata_parity <= {PARITY_BITS{1'b0}};
            end
        end
    end

    // =========================================================================
    // Parity Error Detection
    // =========================================================================
    logic [BYTE_LANES-1:0] parity_err_bits;

    always_comb begin
        if (PARITY_EN && rdata_valid) begin
            parity_err_bits = check_parity(rdata_raw, rdata_parity);
        end else begin
            parity_err_bits = {BYTE_LANES{1'b0}};
        end
    end

    logic parity_err_any;
    assign parity_err_any = |parity_err_bits;

    // =========================================================================
    // Output Assignment
    // =========================================================================
    // ACK is registered (same cycle as data due to single-cycle read latency)

    // Port A ACK: served if sel_a was high last cycle
    logic sel_a_q, sel_b_q;
    logic pa_we_q, pb_we_q;
    logic pa_addr_valid_q, pb_addr_valid_q;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sel_a_q          <= 1'b0;
            sel_b_q          <= 1'b0;
            pa_we_q          <= 1'b0;
            pb_we_q          <= 1'b0;
            pa_addr_valid_q  <= 1'b0;
            pb_addr_valid_q  <= 1'b0;
        end else begin
            sel_a_q         <= sel_a;
            sel_b_q         <= sel_b;
            pa_we_q         <= pa_we;
            pb_we_q         <= pb_we;
            pa_addr_valid_q <= addr_valid & sel_a;
            pb_addr_valid_q <= addr_valid & sel_b;
        end
    end

    // Port A outputs
    assign pa_ack        = sel_a_q & pa_addr_valid_q;
    assign pa_rdata      = (~rdata_port & rdata_valid) ? rdata_raw : {DATA_WIDTH{1'b0}};
    assign pa_parity_err = (~rdata_port & rdata_valid & parity_err_any);

    // Port B outputs
    assign pb_ack        = sel_b_q & pb_addr_valid_q;
    assign pb_rdata      = (rdata_port & rdata_valid) ? rdata_raw : {DATA_WIDTH{1'b0}};
    assign pb_parity_err = (rdata_port & rdata_valid & parity_err_any);

    // =========================================================================
    // Debug Counters
    // =========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dbg_access_count       <= 32'd0;
            dbg_stall_count        <= 32'd0;
            dbg_parity_error_sticky<= 1'b0;
        end else begin
            if (arb_req)
                dbg_access_count <= dbg_access_count + 32'd1;
            if (collision)
                dbg_stall_count  <= dbg_stall_count + 32'd1;
            if (parity_err_any & rdata_valid)
                dbg_parity_error_sticky <= 1'b1;
        end
    end

endmodule
// =============================================================================
// END: unified_sram.sv
// =============================================================================
