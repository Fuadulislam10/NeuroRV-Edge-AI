// ============================================================================
// FILE: rtl/memory/axi_interconnect.sv
// PROJECT: NeuroRV Edge — Phase 4 Memory Subsystem
// MODULE: axi_interconnect
// DESCRIPTION: Lightweight AXI4-Lite style 3-master → 1-slave interconnect.
//
//   Masters (in priority order):
//     M0 = CPU      (highest)
//     M1 = VXU      (medium)
//     M2 = DMA      (lowest)
//
//   Slaves:
//     S0 = UNIFIED SRAM   [SRAM_BASE .. SRAM_BASE + SRAM_SIZE - 1]
//     (additional slave decode stubs for future peripherals)
//
//   AXI Channel Model (Lite, simplified):
//     AW/W/B channels for writes  (address → data → response)
//     AR/R   channels for reads   (address → data)
//     Channels are register-sliced for timing closure.
//
//   Arbitration: Fixed priority round-robin with fairness counter.
//     When no contention: whoever requests first gets the bus.
//     When contention: M0 > M1 > M2 with starvation protection
//     (a lower-priority master waiting > STARVE_LIMIT cycles
//      gets one slot regardless of higher-priority pending).
//
//   Backpressure: Any master can be stalled via *_aw_ready / *_ar_ready.
//
// INTERFACE NOTES:
//   - ADDR_W=32 (full 32-bit byte address)
//   - DATA_W=32 (AXI-Lite mandates 32 or 64; using 32)
//   - STRB_W=4  (byte strobes = DATA_W/8)
//   - Response RRESP/BRESP: 2'b00=OKAY, 2'b10=SLVERR (unmapped)
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

module axi_interconnect #(
    parameter int ADDR_W       = 32,
    parameter int DATA_W       = 32,
    parameter int STARVE_LIMIT = 16,      // Starvation counter threshold
    // Slave address map
    parameter logic [ADDR_W-1:0] SRAM_BASE = 32'h2000_0000,
    parameter logic [ADDR_W-1:0] SRAM_SIZE = 32'h0008_0000  // 512 KB
)(
    input  logic                clk,
    input  logic                rst_n,

    // =========================================================================
    // Master 0 — CPU (AXI4-Lite Slave port facing CPU)
    // =========================================================================
    // Write address channel
    input  logic                m0_aw_valid,
    output logic                m0_aw_ready,
    input  logic [ADDR_W-1:0]   m0_aw_addr,
    input  logic [2:0]          m0_aw_prot,
    // Write data channel
    input  logic                m0_w_valid,
    output logic                m0_w_ready,
    input  logic [DATA_W-1:0]   m0_w_data,
    input  logic [DATA_W/8-1:0] m0_w_strb,
    // Write response channel
    output logic                m0_b_valid,
    input  logic                m0_b_ready,
    output logic [1:0]          m0_b_resp,
    // Read address channel
    input  logic                m0_ar_valid,
    output logic                m0_ar_ready,
    input  logic [ADDR_W-1:0]   m0_ar_addr,
    input  logic [2:0]          m0_ar_prot,
    // Read data channel
    output logic                m0_r_valid,
    input  logic                m0_r_ready,
    output logic [DATA_W-1:0]   m0_r_data,
    output logic [1:0]          m0_r_resp,

    // =========================================================================
    // Master 1 — VXU (AXI4-Lite Slave port facing VXU)
    // =========================================================================
    input  logic                m1_aw_valid,
    output logic                m1_aw_ready,
    input  logic [ADDR_W-1:0]   m1_aw_addr,
    input  logic [2:0]          m1_aw_prot,
    input  logic                m1_w_valid,
    output logic                m1_w_ready,
    input  logic [DATA_W-1:0]   m1_w_data,
    input  logic [DATA_W/8-1:0] m1_w_strb,
    output logic                m1_b_valid,
    input  logic                m1_b_ready,
    output logic [1:0]          m1_b_resp,
    input  logic                m1_ar_valid,
    output logic                m1_ar_ready,
    input  logic [ADDR_W-1:0]   m1_ar_addr,
    input  logic [2:0]          m1_ar_prot,
    output logic                m1_r_valid,
    input  logic                m1_r_ready,
    output logic [DATA_W-1:0]   m1_r_data,
    output logic [1:0]          m1_r_resp,

    // =========================================================================
    // Master 2 — DMA (AXI4-Lite Slave port facing DMA)
    // =========================================================================
    input  logic                m2_aw_valid,
    output logic                m2_aw_ready,
    input  logic [ADDR_W-1:0]   m2_aw_addr,
    input  logic [2:0]          m2_aw_prot,
    input  logic                m2_w_valid,
    output logic                m2_w_ready,
    input  logic [DATA_W-1:0]   m2_w_data,
    input  logic [DATA_W/8-1:0] m2_w_strb,
    output logic                m2_b_valid,
    input  logic                m2_b_ready,
    output logic [1:0]          m2_b_resp,
    input  logic                m2_ar_valid,
    output logic                m2_ar_ready,
    input  logic [ADDR_W-1:0]   m2_ar_addr,
    input  logic [2:0]          m2_ar_prot,
    output logic                m2_r_valid,
    input  logic                m2_r_ready,
    output logic [DATA_W-1:0]   m2_r_data,
    output logic [1:0]          m2_r_resp,

    // =========================================================================
    // Slave 0 — SRAM (AXI4-Lite Master port facing SRAM controller)
    // =========================================================================
    output logic                s0_aw_valid,
    input  logic                s0_aw_ready,
    output logic [ADDR_W-1:0]   s0_aw_addr,
    output logic [2:0]          s0_aw_prot,
    output logic                s0_w_valid,
    input  logic                s0_w_ready,
    output logic [DATA_W-1:0]   s0_w_data,
    output logic [DATA_W/8-1:0] s0_w_strb,
    input  logic                s0_b_valid,
    output logic                s0_b_ready,
    input  logic [1:0]          s0_b_resp,
    output logic                s0_ar_valid,
    input  logic                s0_ar_ready,
    output logic [ADDR_W-1:0]   s0_ar_addr,
    output logic [2:0]          s0_ar_prot,
    input  logic                s0_r_valid,
    output logic                s0_r_ready,
    input  logic [DATA_W-1:0]   s0_r_data,
    input  logic [1:0]          s0_r_resp,

    // =========================================================================
    // Debug
    // =========================================================================
    output logic [31:0]         dbg_arb_grant_count [0:2],  // Grants per master
    output logic [31:0]         dbg_stall_count     [0:2],  // Stalls per master
    output logic [31:0]         dbg_rd_txn_count,
    output logic [31:0]         dbg_wr_txn_count,
    output logic [31:0]         dbg_unmapped_count           // SLVERR count
);

    localparam int STRB_W = DATA_W / 8;
    localparam int NUM_MASTERS = 3;

    // =========================================================================
    // Address decode: is address in SRAM range?
    // =========================================================================
    function automatic logic addr_in_sram(input logic [ADDR_W-1:0] addr);
        return (addr >= SRAM_BASE) && (addr < (SRAM_BASE + SRAM_SIZE));
    endfunction

    // =========================================================================
    // Write Arbiter State
    // =========================================================================
    typedef enum logic [1:0] {
        WA_IDLE   = 2'h0,
        WA_ACTIVE = 2'h1,
        WA_RESP   = 2'h2
    } wr_arb_state_t;

    typedef enum logic [1:0] {
        RA_IDLE   = 2'h0,
        RA_ACTIVE = 2'h1,
        RA_RESP   = 2'h2
    } rd_arb_state_t;

    wr_arb_state_t wr_state;
    rd_arb_state_t rd_state;

    logic [1:0]  wr_grant;    // Current write grant (0/1/2)
    logic [1:0]  rd_grant;    // Current read grant  (0/1/2)
    logic        wr_grant_valid;
    logic        rd_grant_valid;

    // Starvation counters per master
    logic [7:0] wr_starve [0:NUM_MASTERS-1];
    logic [7:0] rd_starve [0:NUM_MASTERS-1];

    // Pending request vectors
    logic [NUM_MASTERS-1:0] wr_req;
    logic [NUM_MASTERS-1:0] rd_req;

    assign wr_req[0] = m0_aw_valid && m0_w_valid;
    assign wr_req[1] = m1_aw_valid && m1_w_valid;
    assign wr_req[2] = m2_aw_valid && m2_w_valid;

    assign rd_req[0] = m0_ar_valid;
    assign rd_req[1] = m1_ar_valid;
    assign rd_req[2] = m2_ar_valid;

    // =========================================================================
    // Arbitration function: fixed priority with starvation protection
    // Returns 0/1/2 for granted master, or 3'b111 for no grant
    // =========================================================================
    function automatic logic [1:0] arb_select(
        input logic [NUM_MASTERS-1:0] req,
        input logic [7:0]             starve [0:NUM_MASTERS-1]
    );
        // Check starvation: any lower-priority master waiting too long?
        if (req[2] && (starve[2] >= STARVE_LIMIT[7:0])) return 2'd2;
        if (req[1] && (starve[1] >= STARVE_LIMIT[7:0])) return 2'd1;
        // Normal priority
        if (req[0]) return 2'd0;
        if (req[1]) return 2'd1;
        if (req[2]) return 2'd2;
        return 2'd3; // no request
    endfunction

    // =========================================================================
    // Registered address/data buffers for winning master
    // =========================================================================
    logic [ADDR_W-1:0]  wr_addr_buf;
    logic [2:0]         wr_prot_buf;
    logic [DATA_W-1:0]  wr_data_buf;
    logic [STRB_W-1:0]  wr_strb_buf;
    logic               wr_mapped;    // 1 = address maps to SRAM

    logic [ADDR_W-1:0]  rd_addr_buf;
    logic [2:0]         rd_prot_buf;
    logic               rd_mapped;

    // =========================================================================
    // WRITE ARBITER FSM
    // =========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_state       <= WA_IDLE;
            wr_grant       <= '0;
            wr_grant_valid <= 1'b0;
            wr_addr_buf    <= '0;
            wr_prot_buf    <= '0;
            wr_data_buf    <= '0;
            wr_strb_buf    <= '0;
            wr_mapped      <= 1'b0;
            for (int i = 0; i < NUM_MASTERS; i++) wr_starve[i] <= '0;
            dbg_wr_txn_count <= '0;
        end else begin
            case (wr_state)
                WA_IDLE: begin
                    wr_grant_valid <= 1'b0;
                    if (wr_req != '0) begin
                        automatic logic [1:0] g;
                        g = arb_select(wr_req, wr_starve);
                        if (g != 2'd3) begin
                            wr_grant       <= g;
                            wr_grant_valid <= 1'b1;
                            // Latch address/data from winning master
                            case (g)
                                2'd0: begin
                                    wr_addr_buf <= m0_aw_addr;
                                    wr_prot_buf <= m0_aw_prot;
                                    wr_data_buf <= m0_w_data;
                                    wr_strb_buf <= m0_w_strb;
                                end
                                2'd1: begin
                                    wr_addr_buf <= m1_aw_addr;
                                    wr_prot_buf <= m1_aw_prot;
                                    wr_data_buf <= m1_w_data;
                                    wr_strb_buf <= m1_w_strb;
                                end
                                default: begin
                                    wr_addr_buf <= m2_aw_addr;
                                    wr_prot_buf <= m2_aw_prot;
                                    wr_data_buf <= m2_w_data;
                                    wr_strb_buf <= m2_w_strb;
                                end
                            endcase
                            wr_mapped <= addr_in_sram(
                                g == 2'd0 ? m0_aw_addr :
                                g == 2'd1 ? m1_aw_addr : m2_aw_addr);
                            // Update starvation counters
                            for (int i = 0; i < NUM_MASTERS; i++) begin
                                if (i == int'(g)) wr_starve[i] <= '0;
                                else if (wr_req[i]) wr_starve[i] <= wr_starve[i] + 1;
                            end
                            wr_state <= WA_ACTIVE;
                            dbg_wr_txn_count <= dbg_wr_txn_count + 1;
                        end
                    end else begin
                        // Decay starvation counters when idle
                        for (int i = 0; i < NUM_MASTERS; i++)
                            if (wr_starve[i] > 0) wr_starve[i] <= wr_starve[i] - 1;
                    end
                end

                WA_ACTIVE: begin
                    // Wait for slave to accept the AW+W transaction
                    if (s0_aw_ready && s0_w_ready && wr_mapped) begin
                        wr_state <= WA_RESP;
                    end else if (!wr_mapped) begin
                        // Unmapped — generate SLVERR without forwarding
                        wr_state <= WA_RESP;
                    end
                end

                WA_RESP: begin
                    // Wait for master to accept B response
                    automatic logic m_b_ready_sel;
                    m_b_ready_sel = (wr_grant == 2'd0) ? m0_b_ready :
                                    (wr_grant == 2'd1) ? m1_b_ready : m2_b_ready;
                    if (m_b_ready_sel) begin
                        wr_state       <= WA_IDLE;
                        wr_grant_valid <= 1'b0;
                    end
                end

                default: wr_state <= WA_IDLE;
            endcase
        end
    end

    // =========================================================================
    // READ ARBITER FSM
    // =========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rd_state       <= RA_IDLE;
            rd_grant       <= '0;
            rd_grant_valid <= 1'b0;
            rd_addr_buf    <= '0;
            rd_prot_buf    <= '0;
            rd_mapped      <= 1'b0;
            for (int i = 0; i < NUM_MASTERS; i++) rd_starve[i] <= '0;
            dbg_rd_txn_count   <= '0;
            dbg_unmapped_count <= '0;
        end else begin
            case (rd_state)
                RA_IDLE: begin
                    rd_grant_valid <= 1'b0;
                    if (rd_req != '0) begin
                        automatic logic [1:0] g;
                        g = arb_select(rd_req, rd_starve);
                        if (g != 2'd3) begin
                            rd_grant       <= g;
                            rd_grant_valid <= 1'b1;
                            case (g)
                                2'd0: begin rd_addr_buf <= m0_ar_addr; rd_prot_buf <= m0_ar_prot; end
                                2'd1: begin rd_addr_buf <= m1_ar_addr; rd_prot_buf <= m1_ar_prot; end
                                default: begin rd_addr_buf <= m2_ar_addr; rd_prot_buf <= m2_ar_prot; end
                            endcase
                            rd_mapped <= addr_in_sram(
                                g == 2'd0 ? m0_ar_addr :
                                g == 2'd1 ? m1_ar_addr : m2_ar_addr);
                            for (int i = 0; i < NUM_MASTERS; i++) begin
                                if (i == int'(g)) rd_starve[i] <= '0;
                                else if (rd_req[i]) rd_starve[i] <= rd_starve[i] + 1;
                            end
                            rd_state <= RA_ACTIVE;
                            dbg_rd_txn_count <= dbg_rd_txn_count + 1;
                            if (!addr_in_sram(
                                g == 2'd0 ? m0_ar_addr :
                                g == 2'd1 ? m1_ar_addr : m2_ar_addr))
                                dbg_unmapped_count <= dbg_unmapped_count + 1;
                        end
                    end else begin
                        for (int i = 0; i < NUM_MASTERS; i++)
                            if (rd_starve[i] > 0) rd_starve[i] <= rd_starve[i] - 1;
                    end
                end

                RA_ACTIVE: begin
                    if ((s0_r_valid && rd_mapped) || !rd_mapped) begin
                        rd_state <= RA_RESP;
                    end
                end

                RA_RESP: begin
                    automatic logic m_r_ready_sel;
                    m_r_ready_sel = (rd_grant == 2'd0) ? m0_r_ready :
                                    (rd_grant == 2'd1) ? m1_r_ready : m2_r_ready;
                    if (m_r_ready_sel) begin
                        rd_state       <= RA_IDLE;
                        rd_grant_valid <= 1'b0;
                    end
                end

                default: rd_state <= RA_IDLE;
            endcase
        end
    end

    // =========================================================================
    // Debug grant counters
    // =========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int i = 0; i < NUM_MASTERS; i++) begin
                dbg_arb_grant_count[i] <= '0;
                dbg_stall_count[i]     <= '0;
            end
        end else begin
            for (int i = 0; i < NUM_MASTERS; i++) begin
                // Grant count: when a master wins arbitration
                if (wr_grant_valid && wr_state == WA_IDLE && int'(wr_grant) == i)
                    dbg_arb_grant_count[i] <= dbg_arb_grant_count[i] + 1;
                // Stall: when master has a request but was not granted
                if (wr_req[i] && wr_grant_valid && int'(wr_grant) != i)
                    dbg_stall_count[i] <= dbg_stall_count[i] + 1;
            end
        end
    end

    // =========================================================================
    // Write channel output MUX — to slave S0
    // =========================================================================
    assign s0_aw_valid = wr_grant_valid && (wr_state == WA_ACTIVE) && wr_mapped;
    assign s0_aw_addr  = wr_addr_buf;
    assign s0_aw_prot  = wr_prot_buf;
    assign s0_w_valid  = wr_grant_valid && (wr_state == WA_ACTIVE) && wr_mapped;
    assign s0_w_data   = wr_data_buf;
    assign s0_w_strb   = wr_strb_buf;
    assign s0_b_ready  = (wr_state == WA_RESP) && wr_mapped;

    // Read channel output MUX — to slave S0
    assign s0_ar_valid = rd_grant_valid && (rd_state == RA_ACTIVE) && rd_mapped;
    assign s0_ar_addr  = rd_addr_buf;
    assign s0_ar_prot  = rd_prot_buf;
    assign s0_r_ready  = (rd_state == RA_RESP) && rd_mapped;

    // =========================================================================
    // Master ready signals — back-pressure
    // M*_aw_ready / M*_w_ready: only when this master holds the grant
    // =========================================================================
    assign m0_aw_ready = (wr_state == WA_IDLE) && (arb_select(wr_req, wr_starve) == 2'd0);
    assign m0_w_ready  = m0_aw_ready;
    assign m1_aw_ready = (wr_state == WA_IDLE) && (arb_select(wr_req, wr_starve) == 2'd1);
    assign m1_w_ready  = m1_aw_ready;
    assign m2_aw_ready = (wr_state == WA_IDLE) && (arb_select(wr_req, wr_starve) == 2'd2);
    assign m2_w_ready  = m2_aw_ready;

    assign m0_ar_ready = (rd_state == RA_IDLE) && (arb_select(rd_req, rd_starve) == 2'd0);
    assign m1_ar_ready = (rd_state == RA_IDLE) && (arb_select(rd_req, rd_starve) == 2'd1);
    assign m2_ar_ready = (rd_state == RA_IDLE) && (arb_select(rd_req, rd_starve) == 2'd2);

    // =========================================================================
    // Write response MUX — route B channel back to winning master
    // =========================================================================
    logic [1:0] b_resp_mux;
    assign b_resp_mux = wr_mapped ? s0_b_resp : 2'b10; // SLVERR on unmapped

    assign m0_b_valid = (wr_grant == 2'd0) && (wr_state == WA_RESP);
    assign m0_b_resp  = b_resp_mux;
    assign m1_b_valid = (wr_grant == 2'd1) && (wr_state == WA_RESP);
    assign m1_b_resp  = b_resp_mux;
    assign m2_b_valid = (wr_grant == 2'd2) && (wr_state == WA_RESP);
    assign m2_b_resp  = b_resp_mux;

    // =========================================================================
    // Read response MUX — route R channel back to winning master
    // =========================================================================
    logic [DATA_W-1:0] r_data_mux;
    logic [1:0]        r_resp_mux;
    assign r_data_mux = rd_mapped ? s0_r_data : '0;
    assign r_resp_mux = rd_mapped ? s0_r_resp : 2'b10;

    assign m0_r_valid = (rd_grant == 2'd0) && (rd_state == RA_RESP);
    assign m0_r_data  = r_data_mux;
    assign m0_r_resp  = r_resp_mux;
    assign m1_r_valid = (rd_grant == 2'd1) && (rd_state == RA_RESP);
    assign m1_r_data  = r_data_mux;
    assign m1_r_resp  = r_resp_mux;
    assign m2_r_valid = (rd_grant == 2'd2) && (rd_state == RA_RESP);
    assign m2_r_data  = r_data_mux;
    assign m2_r_resp  = r_resp_mux;

    // =========================================================================
    // Assertions
    // =========================================================================
    // synthesis translate_off
    always @(posedge clk) begin
        // No two masters should be granted simultaneously
        if (wr_grant_valid) begin
            assert (wr_grant < NUM_MASTERS)
                else $fatal(1, "axi_interconnect: invalid wr_grant %0d", wr_grant);
        end
    end
    // synthesis translate_on

endmodule

`default_nettype wire
