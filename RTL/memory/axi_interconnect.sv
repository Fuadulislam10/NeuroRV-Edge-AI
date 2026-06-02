// =============================================================================
// FILE: rtl/memory/axi_interconnect.sv
// PROJECT: NeuroRV Edge SoC
// MODULE: axi_interconnect
// DESCRIPTION: Simplified AXI4-Lite style 3-master → 1-slave interconnect.
//              Masters: CPU (M0, highest), VXU (M1), DMA (M2, lowest).
//              Slave:   Unified SRAM.
//              Features: Fixed-priority arbitration, backpressure, address
//              decoding, read/write channel separation, debug counters.
// SYNTHESIZABLE: Yes (Yosys + Verilator compatible)
// =============================================================================

`timescale 1ns/1ps

module axi_interconnect #(
    parameter int unsigned ADDR_WIDTH  = 32,
    parameter int unsigned DATA_WIDTH  = 32,
    parameter int unsigned STRB_WIDTH  = 4,   // DATA_WIDTH/8

    // Slave address map: SRAM occupies [SRAM_BASE .. SRAM_BASE+SRAM_SIZE-1]
    parameter logic [ADDR_WIDTH-1:0] SRAM_BASE = 32'h0000_0000,
    parameter logic [ADDR_WIDTH-1:0] SRAM_SIZE = 32'h0008_0000  // 512KB
)(
    input  logic clk,
    input  logic rst_n,

    // =========================================================================
    // Master 0: CPU  (AXI4-Lite subordinate ports seen from master side)
    // =========================================================================
    // Write Address Channel
    input  logic                    m0_awvalid,
    output logic                    m0_awready,
    input  logic [ADDR_WIDTH-1:0]   m0_awaddr,
    input  logic [2:0]              m0_awprot,

    // Write Data Channel
    input  logic                    m0_wvalid,
    output logic                    m0_wready,
    input  logic [DATA_WIDTH-1:0]   m0_wdata,
    input  logic [STRB_WIDTH-1:0]   m0_wstrb,

    // Write Response Channel
    output logic                    m0_bvalid,
    input  logic                    m0_bready,
    output logic [1:0]              m0_bresp,

    // Read Address Channel
    input  logic                    m0_arvalid,
    output logic                    m0_arready,
    input  logic [ADDR_WIDTH-1:0]   m0_araddr,
    input  logic [2:0]              m0_arprot,

    // Read Data Channel
    output logic                    m0_rvalid,
    input  logic                    m0_rready,
    output logic [DATA_WIDTH-1:0]   m0_rdata,
    output logic [1:0]              m0_rresp,

    // =========================================================================
    // Master 1: VXU
    // =========================================================================
    input  logic                    m1_awvalid,
    output logic                    m1_awready,
    input  logic [ADDR_WIDTH-1:0]   m1_awaddr,
    input  logic [2:0]              m1_awprot,

    input  logic                    m1_wvalid,
    output logic                    m1_wready,
    input  logic [DATA_WIDTH-1:0]   m1_wdata,
    input  logic [STRB_WIDTH-1:0]   m1_wstrb,

    output logic                    m1_bvalid,
    input  logic                    m1_bready,
    output logic [1:0]              m1_bresp,

    input  logic                    m1_arvalid,
    output logic                    m1_arready,
    input  logic [ADDR_WIDTH-1:0]   m1_araddr,
    input  logic [2:0]              m1_arprot,

    output logic                    m1_rvalid,
    input  logic                    m1_rready,
    output logic [DATA_WIDTH-1:0]   m1_rdata,
    output logic [1:0]              m1_rresp,

    // =========================================================================
    // Master 2: DMA
    // =========================================================================
    input  logic                    m2_awvalid,
    output logic                    m2_awready,
    input  logic [ADDR_WIDTH-1:0]   m2_awaddr,
    input  logic [2:0]              m2_awprot,

    input  logic                    m2_wvalid,
    output logic                    m2_wready,
    input  logic [DATA_WIDTH-1:0]   m2_wdata,
    input  logic [STRB_WIDTH-1:0]   m2_wstrb,

    output logic                    m2_bvalid,
    input  logic                    m2_bready,
    output logic [1:0]              m2_bresp,

    input  logic                    m2_arvalid,
    output logic                    m2_arready,
    input  logic [ADDR_WIDTH-1:0]   m2_araddr,
    input  logic [2:0]              m2_arprot,

    output logic                    m2_rvalid,
    input  logic                    m2_rready,
    output logic [DATA_WIDTH-1:0]   m2_rdata,
    output logic [1:0]              m2_rresp,

    // =========================================================================
    // Slave: SRAM port (direct word-addressed interface to unified_sram)
    // =========================================================================
    output logic                    s_req,
    output logic                    s_we,
    output logic [16:0]             s_addr,         // word address (17-bit for 512KB)
    output logic [DATA_WIDTH-1:0]   s_wdata,
    output logic [STRB_WIDTH-1:0]   s_be,
    input  logic [DATA_WIDTH-1:0]   s_rdata,
    input  logic                    s_ack,

    // =========================================================================
    // Debug
    // =========================================================================
    output logic [31:0]             dbg_m0_wr_count,
    output logic [31:0]             dbg_m0_rd_count,
    output logic [31:0]             dbg_m1_wr_count,
    output logic [31:0]             dbg_m1_rd_count,
    output logic [31:0]             dbg_m2_wr_count,
    output logic [31:0]             dbg_m2_rd_count,
    output logic [31:0]             dbg_arb_stall_count,
    output logic [7:0]              dbg_error_flags   // [7:4]=rd decode err, [3:0]=wr decode err
);

    // =========================================================================
    // Internal Types
    // =========================================================================
    typedef enum logic [1:0] {
        RESP_OKAY   = 2'b00,
        RESP_EXOKAY = 2'b01,
        RESP_SLVERR = 2'b10,
        RESP_DECERR = 2'b11
    } axi_resp_t;

    typedef enum logic [1:0] {
        ARB_IDLE = 2'b00,
        ARB_M0   = 2'b01,
        ARB_M1   = 2'b10,
        ARB_M2   = 2'b11
    } arb_state_t;

    // =========================================================================
    // Address Decode
    // =========================================================================
    function automatic logic addr_in_sram(input logic [ADDR_WIDTH-1:0] addr);
        return (addr >= SRAM_BASE) && (addr < (SRAM_BASE + SRAM_SIZE));
    endfunction

    function automatic logic [16:0] to_word_addr(input logic [ADDR_WIDTH-1:0] addr);
        return addr[18:2];   // byte→word: drop 2 LSBs, take 17 bits from SRAM window
    endfunction

    // =========================================================================
    // Write Arbitration State Machine
    // =========================================================================
    // Handles write-address + write-data channels atomically (AXI4-Lite
    // guarantees AW and W arrive together or we buffer them).
    // We use a simple latch: accept AW when both AW and W are valid.

    arb_state_t wr_arb_state, wr_arb_next;
    arb_state_t rd_arb_state, rd_arb_next;

    // Pending write request per master (latched when AW+W both valid)
    logic [ADDR_WIDTH-1:0]  wr_addr  [0:2];
    logic [DATA_WIDTH-1:0]  wr_data  [0:2];
    logic [STRB_WIDTH-1:0]  wr_strb  [0:2];
    logic                   wr_pend  [0:2];  // write request pending

    logic [ADDR_WIDTH-1:0]  rd_addr  [0:2];
    logic                   rd_pend  [0:2];  // read request pending

    // Write data/response handshake per master
    logic [2:0] wr_active;   // bitmask: which master is currently being served (write)
    logic [2:0] rd_active;   // bitmask: which master is currently being served (read)

    // -------------------------------------------------------------------------
    // Capture write requests (AW+W simultaneous for AXI4-Lite)
    // -------------------------------------------------------------------------
    genvar gi;
    generate
        // Master 0 write capture
        always_ff @(posedge clk or negedge rst_n) begin
            if (!rst_n) begin
                wr_pend[0] <= 1'b0;
                wr_addr[0] <= {ADDR_WIDTH{1'b0}};
                wr_data[0] <= {DATA_WIDTH{1'b0}};
                wr_strb[0] <= {STRB_WIDTH{1'b0}};
            end else begin
                if (m0_awvalid && m0_awready && m0_wvalid && m0_wready) begin
                    wr_addr[0] <= m0_awaddr;
                    wr_data[0] <= m0_wdata;
                    wr_strb[0] <= m0_wstrb;
                    wr_pend[0] <= 1'b1;
                end else if (wr_active[0] && s_ack) begin
                    wr_pend[0] <= 1'b0;
                end
            end
        end

        // Master 1 write capture
        always_ff @(posedge clk or negedge rst_n) begin
            if (!rst_n) begin
                wr_pend[1] <= 1'b0;
                wr_addr[1] <= {ADDR_WIDTH{1'b0}};
                wr_data[1] <= {DATA_WIDTH{1'b0}};
                wr_strb[1] <= {STRB_WIDTH{1'b0}};
            end else begin
                if (m1_awvalid && m1_awready && m1_wvalid && m1_wready) begin
                    wr_addr[1] <= m1_awaddr;
                    wr_data[1] <= m1_wdata;
                    wr_strb[1] <= m1_wstrb;
                    wr_pend[1] <= 1'b1;
                end else if (wr_active[1] && s_ack) begin
                    wr_pend[1] <= 1'b0;
                end
            end
        end

        // Master 2 write capture
        always_ff @(posedge clk or negedge rst_n) begin
            if (!rst_n) begin
                wr_pend[2] <= 1'b0;
                wr_addr[2] <= {ADDR_WIDTH{1'b0}};
                wr_data[2] <= {DATA_WIDTH{1'b0}};
                wr_strb[2] <= {STRB_WIDTH{1'b0}};
            end else begin
                if (m2_awvalid && m2_awready && m2_wvalid && m2_wready) begin
                    wr_addr[2] <= m2_awaddr;
                    wr_data[2] <= m2_wdata;
                    wr_strb[2] <= m2_wstrb;
                    wr_pend[2] <= 1'b1;
                end else if (wr_active[2] && s_ack) begin
                    wr_pend[2] <= 1'b0;
                end
            end
        end
    endgenerate

    // -------------------------------------------------------------------------
    // Capture read requests
    // -------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rd_pend[0] <= 1'b0; rd_addr[0] <= '0;
            rd_pend[1] <= 1'b0; rd_addr[1] <= '0;
            rd_pend[2] <= 1'b0; rd_addr[2] <= '0;
        end else begin
            if (m0_arvalid && m0_arready) begin
                rd_addr[0] <= m0_araddr;
                rd_pend[0] <= 1'b1;
            end else if (rd_active[0] && s_ack) begin
                rd_pend[0] <= 1'b0;
            end

            if (m1_arvalid && m1_arready) begin
                rd_addr[1] <= m1_araddr;
                rd_pend[1] <= 1'b1;
            end else if (rd_active[1] && s_ack) begin
                rd_pend[1] <= 1'b0;
            end

            if (m2_arvalid && m2_arready) begin
                rd_addr[2] <= m2_araddr;
                rd_pend[2] <= 1'b1;
            end else if (rd_active[2] && s_ack) begin
                rd_pend[2] <= 1'b0;
            end
        end
    end

    // =========================================================================
    // AW/W Ready signals — accept when not already pending for that master
    // =========================================================================
    assign m0_awready = ~wr_pend[0];
    assign m0_wready  = ~wr_pend[0];
    assign m1_awready = ~wr_pend[1];
    assign m1_wready  = ~wr_pend[1];
    assign m2_awready = ~wr_pend[2];
    assign m2_wready  = ~wr_pend[2];

    assign m0_arready = ~rd_pend[0];
    assign m1_arready = ~rd_pend[1];
    assign m2_arready = ~rd_pend[2];

    // =========================================================================
    // Write Arbitration FSM (Fixed Priority: M0 > M1 > M2)
    // =========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) wr_arb_state <= ARB_IDLE;
        else        wr_arb_state <= wr_arb_next;
    end

    always_comb begin
        wr_arb_next = wr_arb_state;
        case (wr_arb_state)
            ARB_IDLE: begin
                if      (wr_pend[0]) wr_arb_next = ARB_M0;
                else if (wr_pend[1]) wr_arb_next = ARB_M1;
                else if (wr_pend[2]) wr_arb_next = ARB_M2;
            end
            ARB_M0: if (s_ack) wr_arb_next = ARB_IDLE;
            ARB_M1: if (s_ack) wr_arb_next = ARB_IDLE;
            ARB_M2: if (s_ack) wr_arb_next = ARB_IDLE;
            default: wr_arb_next = ARB_IDLE;
        endcase
    end

    assign wr_active[0] = (wr_arb_state == ARB_M0);
    assign wr_active[1] = (wr_arb_state == ARB_M1);
    assign wr_active[2] = (wr_arb_state == ARB_M2);

    // =========================================================================
    // Read Arbitration FSM (Fixed Priority: M0 > M1 > M2)
    // =========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) rd_arb_state <= ARB_IDLE;
        else        rd_arb_state <= rd_arb_next;
    end

    always_comb begin
        rd_arb_next = rd_arb_state;
        case (rd_arb_state)
            ARB_IDLE: begin
                if      (rd_pend[0]) rd_arb_next = ARB_M0;
                else if (rd_pend[1]) rd_arb_next = ARB_M1;
                else if (rd_pend[2]) rd_arb_next = ARB_M2;
            end
            ARB_M0: if (s_ack) rd_arb_next = ARB_IDLE;
            ARB_M1: if (s_ack) rd_arb_next = ARB_IDLE;
            ARB_M2: if (s_ack) rd_arb_next = ARB_IDLE;
            default: rd_arb_next = ARB_IDLE;
        endcase
    end

    assign rd_active[0] = (rd_arb_state == ARB_M0);
    assign rd_active[1] = (rd_arb_state == ARB_M1);
    assign rd_active[2] = (rd_arb_state == ARB_M2);

    // =========================================================================
    // Slave SRAM Drive
    // Write channel has priority over read when both ready (write-first policy).
    // =========================================================================
    logic wr_grant;
    logic rd_grant;

    assign wr_grant = |wr_active;   // any write master active
    assign rd_grant = |rd_active & ~wr_grant; // read only when no write pending

    // Mux selected write master
    logic [ADDR_WIDTH-1:0] sel_wr_addr;
    logic [DATA_WIDTH-1:0] sel_wr_data;
    logic [STRB_WIDTH-1:0] sel_wr_strb;

    always_comb begin
        sel_wr_addr = wr_addr[0];
        sel_wr_data = wr_data[0];
        sel_wr_strb = wr_strb[0];
        if      (wr_active[0]) begin sel_wr_addr = wr_addr[0]; sel_wr_data = wr_data[0]; sel_wr_strb = wr_strb[0]; end
        else if (wr_active[1]) begin sel_wr_addr = wr_addr[1]; sel_wr_data = wr_data[1]; sel_wr_strb = wr_strb[1]; end
        else if (wr_active[2]) begin sel_wr_addr = wr_addr[2]; sel_wr_data = wr_data[2]; sel_wr_strb = wr_strb[2]; end
    end

    // Mux selected read master
    logic [ADDR_WIDTH-1:0] sel_rd_addr;
    always_comb begin
        sel_rd_addr = rd_addr[0];
        if      (rd_active[0]) sel_rd_addr = rd_addr[0];
        else if (rd_active[1]) sel_rd_addr = rd_addr[1];
        else if (rd_active[2]) sel_rd_addr = rd_addr[2];
    end

    // Address decode
    logic wr_decode_ok;
    logic rd_decode_ok;
    assign wr_decode_ok = addr_in_sram(sel_wr_addr);
    assign rd_decode_ok = addr_in_sram(sel_rd_addr);

    // Drive slave
    always_comb begin
        s_req   = 1'b0;
        s_we    = 1'b0;
        s_addr  = 17'h0;
        s_wdata = {DATA_WIDTH{1'b0}};
        s_be    = {STRB_WIDTH{1'b0}};

        if (wr_grant && wr_decode_ok) begin
            s_req   = 1'b1;
            s_we    = 1'b1;
            s_addr  = to_word_addr(sel_wr_addr);
            s_wdata = sel_wr_data;
            s_be    = sel_wr_strb;
        end else if (rd_grant && rd_decode_ok) begin
            s_req   = 1'b1;
            s_we    = 1'b0;
            s_addr  = to_word_addr(sel_rd_addr);
            s_wdata = {DATA_WIDTH{1'b0}};
            s_be    = {STRB_WIDTH{1'bx}};
        end
    end

    // =========================================================================
    // Write Response Back to Masters
    // =========================================================================
    // B channel fires one cycle after s_ack on a write
    logic wr_resp_valid [0:2];
    logic [1:0] wr_resp_code [0:2];

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int i = 0; i < 3; i++) begin
                wr_resp_valid[i] <= 1'b0;
                wr_resp_code[i]  <= 2'b00;
            end
        end else begin
            for (int i = 0; i < 3; i++) begin
                if (wr_active[i] && s_ack) begin
                    wr_resp_valid[i] <= 1'b1;
                    wr_resp_code[i]  <= wr_decode_ok ? RESP_OKAY : RESP_DECERR;
                end else if (wr_resp_valid[i]) begin
                    // Clear when master accepts response
                    case (i)
                        0: if (m0_bready) wr_resp_valid[i] <= 1'b0;
                        1: if (m1_bready) wr_resp_valid[i] <= 1'b0;
                        2: if (m2_bready) wr_resp_valid[i] <= 1'b0;
                        default: wr_resp_valid[i] <= 1'b0;
                    endcase
                end
            end
        end
    end

    assign m0_bvalid = wr_resp_valid[0];
    assign m0_bresp  = wr_resp_code[0];
    assign m1_bvalid = wr_resp_valid[1];
    assign m1_bresp  = wr_resp_code[1];
    assign m2_bvalid = wr_resp_valid[2];
    assign m2_bresp  = wr_resp_code[2];

    // =========================================================================
    // Read Response Back to Masters
    // =========================================================================
    logic rd_resp_valid [0:2];
    logic [DATA_WIDTH-1:0] rd_resp_data [0:2];
    logic [1:0] rd_resp_code [0:2];

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int i = 0; i < 3; i++) begin
                rd_resp_valid[i] <= 1'b0;
                rd_resp_data[i]  <= {DATA_WIDTH{1'b0}};
                rd_resp_code[i]  <= 2'b00;
            end
        end else begin
            for (int i = 0; i < 3; i++) begin
                if (rd_active[i] && s_ack) begin
                    rd_resp_valid[i] <= 1'b1;
                    rd_resp_data[i]  <= rd_decode_ok ? s_rdata : {DATA_WIDTH{1'b0}};
                    rd_resp_code[i]  <= rd_decode_ok ? RESP_OKAY : RESP_DECERR;
                end else if (rd_resp_valid[i]) begin
                    case (i)
                        0: if (m0_rready) rd_resp_valid[i] <= 1'b0;
                        1: if (m1_rready) rd_resp_valid[i] <= 1'b0;
                        2: if (m2_rready) rd_resp_valid[i] <= 1'b0;
                        default: rd_resp_valid[i] <= 1'b0;
                    endcase
                end
            end
        end
    end

    assign m0_rvalid = rd_resp_valid[0];
    assign m0_rdata  = rd_resp_data[0];
    assign m0_rresp  = rd_resp_code[0];
    assign m1_rvalid = rd_resp_valid[1];
    assign m1_rdata  = rd_resp_data[1];
    assign m1_rresp  = rd_resp_code[1];
    assign m2_rvalid = rd_resp_valid[2];
    assign m2_rdata  = rd_resp_data[2];
    assign m2_rresp  = rd_resp_code[2];

    // =========================================================================
    // Debug Counters
    // =========================================================================
    // Stall counter: any cycle where a request is pending but blocked
    logic arb_stall;
    assign arb_stall = (wr_pend[1] & wr_active[0]) |  // M1 stalled by M0
                       (wr_pend[2] & (wr_active[0] | wr_active[1])) |
                       (rd_pend[1] & rd_active[0]) |
                       (rd_pend[2] & (rd_active[0] | rd_active[1]));

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dbg_m0_wr_count    <= 32'd0;
            dbg_m0_rd_count    <= 32'd0;
            dbg_m1_wr_count    <= 32'd0;
            dbg_m1_rd_count    <= 32'd0;
            dbg_m2_wr_count    <= 32'd0;
            dbg_m2_rd_count    <= 32'd0;
            dbg_arb_stall_count<= 32'd0;
            dbg_error_flags    <= 8'd0;
        end else begin
            if (wr_active[0] && s_ack) dbg_m0_wr_count <= dbg_m0_wr_count + 1;
            if (rd_active[0] && s_ack) dbg_m0_rd_count <= dbg_m0_rd_count + 1;
            if (wr_active[1] && s_ack) dbg_m1_wr_count <= dbg_m1_wr_count + 1;
            if (rd_active[1] && s_ack) dbg_m1_rd_count <= dbg_m1_rd_count + 1;
            if (wr_active[2] && s_ack) dbg_m2_wr_count <= dbg_m2_wr_count + 1;
            if (rd_active[2] && s_ack) dbg_m2_rd_count <= dbg_m2_rd_count + 1;
            if (arb_stall)             dbg_arb_stall_count <= dbg_arb_stall_count + 1;

            // Error flag: decode error on write [3:0] = per-master sticky
            if (wr_active[0] && s_ack && !wr_decode_ok) dbg_error_flags[0] <= 1'b1;
            if (wr_active[1] && s_ack && !wr_decode_ok) dbg_error_flags[1] <= 1'b1;
            if (wr_active[2] && s_ack && !wr_decode_ok) dbg_error_flags[2] <= 1'b1;
            // Error flag: decode error on read [7:4]
            if (rd_active[0] && s_ack && !rd_decode_ok) dbg_error_flags[4] <= 1'b1;
            if (rd_active[1] && s_ack && !rd_decode_ok) dbg_error_flags[5] <= 1'b1;
            if (rd_active[2] && s_ack && !rd_decode_ok) dbg_error_flags[6] <= 1'b1;
        end
    end

endmodule
// =============================================================================
// END: axi_interconnect.sv
// =============================================================================
