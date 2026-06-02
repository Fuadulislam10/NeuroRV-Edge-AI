// =============================================================================
// FILE: rtl/memory/dma_controller.sv
// PROJECT: NeuroRV Edge SoC
// MODULE: dma_controller
// DESCRIPTION: DMA Controller supporting Memory-to-Memory, Memory-to-VXU,
//              and VXU-to-Memory transfer modes. Features: burst transfers,
//              configurable length, start/done interrupt, status registers,
//              memory-mapped CPU register interface, AXI master outputs.
// SYNTHESIZABLE: Yes (Yosys + Verilator compatible)
// =============================================================================

`timescale 1ns/1ps

module dma_controller #(
    parameter int unsigned ADDR_WIDTH    = 32,
    parameter int unsigned DATA_WIDTH    = 32,
    parameter int unsigned STRB_WIDTH    = 4,
    parameter int unsigned LEN_WIDTH     = 20,   // max transfer length in words
    parameter int unsigned BURST_LEN     = 16,   // max words per burst
    parameter int unsigned REG_ADDR_BITS = 4     // 16 internal registers
)(
    input  logic clk,
    input  logic rst_n,

    // =========================================================================
    // CPU Register Interface (memory-mapped, word-addressed)
    // reg[0x0] = DMA_CTRL   : [0]=start, [1]=abort, [3:2]=mode
    // reg[0x1] = DMA_STATUS : [0]=busy, [1]=done, [2]=error, [3]=irq_pending
    // reg[0x2] = DMA_SRC    : source byte address
    // reg[0x3] = DMA_DST    : destination byte address
    // reg[0x4] = DMA_LEN    : transfer length in 32-bit words
    // reg[0x5] = DMA_PROGRESS: words transferred so far (read-only)
    // reg[0x6] = DMA_IRQ_CLR: write 1 to clear irq
    // reg[0x7] = DMA_DBG_CNT: debug transaction count (read-only)
    // =========================================================================
    input  logic                       cfg_req,
    input  logic                       cfg_we,
    input  logic [REG_ADDR_BITS-1:0]   cfg_addr,
    input  logic [DATA_WIDTH-1:0]      cfg_wdata,
    output logic [DATA_WIDTH-1:0]      cfg_rdata,
    output logic                       cfg_ack,

    // =========================================================================
    // DMA AXI Master Read Interface (to interconnect / SRAM)
    // =========================================================================
    output logic                       dma_arvalid,
    input  logic                       dma_arready,
    output logic [ADDR_WIDTH-1:0]      dma_araddr,

    input  logic                       dma_rvalid,
    output logic                       dma_rready,
    input  logic [DATA_WIDTH-1:0]      dma_rdata,
    input  logic [1:0]                 dma_rresp,

    // =========================================================================
    // DMA AXI Master Write Interface (to interconnect / SRAM)
    // =========================================================================
    output logic                       dma_awvalid,
    input  logic                       dma_awready,
    output logic [ADDR_WIDTH-1:0]      dma_awaddr,

    output logic                       dma_wvalid,
    input  logic                       dma_wready,
    output logic [DATA_WIDTH-1:0]      dma_wdata,
    output logic [STRB_WIDTH-1:0]      dma_wstrb,

    input  logic                       dma_bvalid,
    output logic                       dma_bready,
    input  logic [1:0]                 dma_bresp,

    // =========================================================================
    // VXU Streaming Interface (for MEM↔VXU modes)
    // =========================================================================
    // DMA → VXU (memory-to-VXU mode)
    output logic                       vxu_stream_valid,
    input  logic                       vxu_stream_ready,
    output logic [DATA_WIDTH-1:0]      vxu_stream_data,
    output logic                       vxu_stream_last,

    // VXU → DMA (VXU-to-memory mode)
    input  logic                       vxu_source_valid,
    output logic                       vxu_source_ready,
    input  logic [DATA_WIDTH-1:0]      vxu_source_data,
    input  logic                       vxu_source_last,

    // =========================================================================
    // Interrupt Output
    // =========================================================================
    output logic                       dma_irq,

    // =========================================================================
    // Debug
    // =========================================================================
    output logic [31:0]                dbg_wr_count,
    output logic [31:0]                dbg_rd_count,
    output logic [31:0]                dbg_burst_count,
    output logic [31:0]                dbg_stall_count,
    output logic                       dbg_busy
);

    // =========================================================================
    // Transfer Mode Encoding
    // =========================================================================
    typedef enum logic [1:0] {
        DMA_MEM2MEM = 2'b00,    // SRAM → SRAM
        DMA_MEM2VXU = 2'b01,    // SRAM → VXU stream
        DMA_VXU2MEM = 2'b10,    // VXU stream → SRAM
        DMA_RESERVED= 2'b11
    } dma_mode_t;

    // =========================================================================
    // DMA FSM States
    // =========================================================================
    typedef enum logic [3:0] {
        DMA_IDLE         = 4'h0,
        DMA_FETCH_RD_ADDR= 4'h1,   // Issue AXI AR
        DMA_FETCH_RD_DATA= 4'h2,   // Wait AXI R
        DMA_PUSH_VXU     = 4'h3,   // Stream to VXU
        DMA_VXU_RD       = 4'h4,   // Accept VXU source data
        DMA_WRITE_ADDR   = 4'h5,   // Issue AXI AW
        DMA_WRITE_DATA   = 4'h6,   // Issue AXI W
        DMA_WRITE_RESP   = 4'h7,   // Wait AXI B
        DMA_DONE         = 4'h8,
        DMA_ERROR        = 4'h9,
        DMA_ABORT        = 4'hA
    } dma_fsm_t;

    // =========================================================================
    // Registers (memory-mapped)
    // =========================================================================
    logic [1:0]             reg_mode;       // transfer mode
    logic                   reg_start;      // write 1 to start (self-clearing)
    logic                   reg_abort;      // write 1 to abort
    logic [ADDR_WIDTH-1:0]  reg_src;        // source address
    logic [ADDR_WIDTH-1:0]  reg_dst;        // destination address
    logic [LEN_WIDTH-1:0]   reg_len;        // length in words
    logic                   reg_irq_clr;    // IRQ clear pulse

    // Status (read by CPU)
    logic                   stat_busy;
    logic                   stat_done;
    logic                   stat_error;
    logic                   stat_irq;
    logic [LEN_WIDTH-1:0]   stat_progress;

    // =========================================================================
    // Internal DMA State
    // =========================================================================
    dma_fsm_t dma_state, dma_next;
    dma_mode_t dma_mode_reg;

    logic [ADDR_WIDTH-1:0]  cur_src;        // current source pointer
    logic [ADDR_WIDTH-1:0]  cur_dst;        // current dest pointer
    logic [LEN_WIDTH-1:0]   words_left;     // words remaining
    logic [LEN_WIDTH-1:0]   words_done;     // words completed

    // Internal data buffer for MEM2MEM (one word FIFO depth)
    logic [DATA_WIDTH-1:0]  rd_buf;
    logic                   rd_buf_valid;

    // Burst tracking
    logic [7:0]             burst_words;    // words in current burst remaining

    // =========================================================================
    // CPU Register Read/Write
    // =========================================================================
    // Reg address decode constants
    localparam logic [REG_ADDR_BITS-1:0] REG_CTRL     = 4'h0;
    localparam logic [REG_ADDR_BITS-1:0] REG_STATUS   = 4'h1;
    localparam logic [REG_ADDR_BITS-1:0] REG_SRC      = 4'h2;
    localparam logic [REG_ADDR_BITS-1:0] REG_DST      = 4'h3;
    localparam logic [REG_ADDR_BITS-1:0] REG_LEN      = 4'h4;
    localparam logic [REG_ADDR_BITS-1:0] REG_PROGRESS = 4'h5;
    localparam logic [REG_ADDR_BITS-1:0] REG_IRQ_CLR  = 4'h6;
    localparam logic [REG_ADDR_BITS-1:0] REG_DBG_CNT  = 4'h7;

    // Single-cycle ACK
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) cfg_ack <= 1'b0;
        else        cfg_ack <= cfg_req;
    end

    // Write
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            reg_mode    <= 2'b00;
            reg_start   <= 1'b0;
            reg_abort   <= 1'b0;
            reg_src     <= {ADDR_WIDTH{1'b0}};
            reg_dst     <= {ADDR_WIDTH{1'b0}};
            reg_len     <= {LEN_WIDTH{1'b0}};
            reg_irq_clr <= 1'b0;
        end else begin
            reg_start   <= 1'b0;   // self-clear
            reg_abort   <= 1'b0;
            reg_irq_clr <= 1'b0;
            if (cfg_req && cfg_we) begin
                case (cfg_addr)
                    REG_CTRL:    begin
                                    reg_start <= cfg_wdata[0];
                                    reg_abort <= cfg_wdata[1];
                                    reg_mode  <= cfg_wdata[3:2];
                                 end
                    REG_SRC:     reg_src  <= cfg_wdata[ADDR_WIDTH-1:0];
                    REG_DST:     reg_dst  <= cfg_wdata[ADDR_WIDTH-1:0];
                    REG_LEN:     reg_len  <= cfg_wdata[LEN_WIDTH-1:0];
                    REG_IRQ_CLR: reg_irq_clr <= cfg_wdata[0];
                    default: ;
                endcase
            end
        end
    end

    // Read
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cfg_rdata <= {DATA_WIDTH{1'b0}};
        end else begin
            if (cfg_req && !cfg_we) begin
                case (cfg_addr)
                    REG_CTRL:     cfg_rdata <= {{(DATA_WIDTH-4){1'b0}}, reg_mode, 1'b0, stat_busy};
                    REG_STATUS:   cfg_rdata <= {{(DATA_WIDTH-4){1'b0}}, stat_irq, stat_error, stat_done, stat_busy};
                    REG_SRC:      cfg_rdata <= reg_src;
                    REG_DST:      cfg_rdata <= reg_dst;
                    REG_LEN:      cfg_rdata <= {{(DATA_WIDTH-LEN_WIDTH){1'b0}}, reg_len};
                    REG_PROGRESS: cfg_rdata <= {{(DATA_WIDTH-LEN_WIDTH){1'b0}}, stat_progress};
                    REG_DBG_CNT:  cfg_rdata <= dbg_wr_count;
                    default:      cfg_rdata <= 32'hDEAD_BEEF;
                endcase
            end
        end
    end

    // =========================================================================
    // DMA FSM
    // =========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) dma_state <= DMA_IDLE;
        else        dma_state <= dma_next;
    end

    // Next-state logic
    always_comb begin
        dma_next = dma_state;
        case (dma_state)
            DMA_IDLE: begin
                if (reg_start && |reg_len) begin
                    case (reg_mode)
                        DMA_MEM2MEM: dma_next = DMA_FETCH_RD_ADDR;
                        DMA_MEM2VXU: dma_next = DMA_FETCH_RD_ADDR;
                        DMA_VXU2MEM: dma_next = DMA_VXU_RD;
                        default:     dma_next = DMA_IDLE;
                    endcase
                end
            end

            DMA_FETCH_RD_ADDR: begin
                if (dma_arready) dma_next = DMA_FETCH_RD_DATA;
                if (reg_abort)   dma_next = DMA_ABORT;
            end

            DMA_FETCH_RD_DATA: begin
                if (dma_rvalid) begin
                    if (dma_rresp != 2'b00)
                        dma_next = DMA_ERROR;
                    else if (dma_mode_reg == DMA_MEM2VXU)
                        dma_next = DMA_PUSH_VXU;
                    else
                        dma_next = DMA_WRITE_ADDR;
                end
                if (reg_abort) dma_next = DMA_ABORT;
            end

            DMA_PUSH_VXU: begin
                if (vxu_stream_valid && vxu_stream_ready) begin
                    if (words_left == 1)
                        dma_next = DMA_DONE;
                    else
                        dma_next = DMA_FETCH_RD_ADDR;
                end
                if (reg_abort) dma_next = DMA_ABORT;
            end

            DMA_VXU_RD: begin
                if (vxu_source_valid) begin
                    if (dma_rresp == 2'b00) // reuse for error flag via separate path
                        dma_next = DMA_WRITE_ADDR;
                end
                if (reg_abort) dma_next = DMA_ABORT;
            end

            DMA_WRITE_ADDR: begin
                if (dma_awready) dma_next = DMA_WRITE_DATA;
                if (reg_abort)   dma_next = DMA_ABORT;
            end

            DMA_WRITE_DATA: begin
                if (dma_wready) dma_next = DMA_WRITE_RESP;
                if (reg_abort)  dma_next = DMA_ABORT;
            end

            DMA_WRITE_RESP: begin
                if (dma_bvalid) begin
                    if (dma_bresp != 2'b00)
                        dma_next = DMA_ERROR;
                    else if (words_left == 1)
                        dma_next = DMA_DONE;
                    else if (dma_mode_reg == DMA_VXU2MEM)
                        dma_next = DMA_VXU_RD;
                    else
                        dma_next = DMA_FETCH_RD_ADDR;
                end
                if (reg_abort) dma_next = DMA_ABORT;
            end

            DMA_DONE:  dma_next = DMA_IDLE;
            DMA_ERROR: dma_next = DMA_IDLE;
            DMA_ABORT: dma_next = DMA_IDLE;

            default:   dma_next = DMA_IDLE;
        endcase
    end

    // =========================================================================
    // Datapath Registers
    // =========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cur_src      <= {ADDR_WIDTH{1'b0}};
            cur_dst      <= {ADDR_WIDTH{1'b0}};
            words_left   <= {LEN_WIDTH{1'b0}};
            words_done   <= {LEN_WIDTH{1'b0}};
            rd_buf       <= {DATA_WIDTH{1'b0}};
            rd_buf_valid <= 1'b0;
            dma_mode_reg <= DMA_MEM2MEM;
            burst_words  <= 8'h0;
            stat_busy    <= 1'b0;
            stat_done    <= 1'b0;
            stat_error   <= 1'b0;
            stat_irq     <= 1'b0;
            stat_progress<= {LEN_WIDTH{1'b0}};
        end else begin
            // Latch config on start
            if (dma_state == DMA_IDLE && reg_start && |reg_len) begin
                cur_src      <= reg_src;
                cur_dst      <= reg_dst;
                words_left   <= reg_len;
                words_done   <= {LEN_WIDTH{1'b0}};
                dma_mode_reg <= dma_mode_t'(reg_mode);
                stat_busy    <= 1'b1;
                stat_done    <= 1'b0;
                stat_error   <= 1'b0;
            end

            // Advance source pointer after each successful read
            if (dma_state == DMA_FETCH_RD_DATA && dma_rvalid && dma_rresp == 2'b00) begin
                rd_buf       <= dma_rdata;
                rd_buf_valid <= 1'b1;
                cur_src      <= cur_src + 4;
            end else begin
                rd_buf_valid <= 1'b0;
            end

            // For VXU source mode: capture incoming word
            if (dma_state == DMA_VXU_RD && vxu_source_valid) begin
                rd_buf       <= vxu_source_data;
                rd_buf_valid <= 1'b1;
            end

            // Advance dest pointer + decrement counter after write response
            if (dma_state == DMA_WRITE_RESP && dma_bvalid && dma_bresp == 2'b00) begin
                cur_dst    <= cur_dst + 4;
                words_left <= words_left - 1;
                words_done <= words_done + 1;
            end

            // MEM2VXU: advance src and decrement on successful VXU push
            if (dma_state == DMA_PUSH_VXU && vxu_stream_valid && vxu_stream_ready) begin
                cur_src    <= cur_src + 4;
                words_left <= words_left - 1;
                words_done <= words_done + 1;
            end

            // Status updates
            if (dma_state == DMA_DONE) begin
                stat_busy  <= 1'b0;
                stat_done  <= 1'b1;
                stat_irq   <= 1'b1;
            end
            if (dma_state == DMA_ERROR) begin
                stat_busy  <= 1'b0;
                stat_error <= 1'b1;
                stat_irq   <= 1'b1;
            end
            if (dma_state == DMA_ABORT) begin
                stat_busy  <= 1'b0;
            end

            // IRQ clear
            if (reg_irq_clr) begin
                stat_irq  <= 1'b0;
                stat_done <= 1'b0;
            end

            stat_progress <= words_done;
        end
    end

    // =========================================================================
    // AXI AR Channel (Read Address)
    // =========================================================================
    assign dma_arvalid = (dma_state == DMA_FETCH_RD_ADDR);
    assign dma_araddr  = cur_src;

    // =========================================================================
    // AXI R Channel (Read Data)
    // =========================================================================
    assign dma_rready  = (dma_state == DMA_FETCH_RD_DATA);

    // =========================================================================
    // AXI AW Channel (Write Address)
    // =========================================================================
    assign dma_awvalid = (dma_state == DMA_WRITE_ADDR);
    assign dma_awaddr  = cur_dst;

    // =========================================================================
    // AXI W Channel (Write Data)
    // =========================================================================
    assign dma_wvalid  = (dma_state == DMA_WRITE_DATA);
    assign dma_wdata   = rd_buf;
    assign dma_wstrb   = {STRB_WIDTH{1'b1}};   // full-word writes

    // =========================================================================
    // AXI B Channel (Write Response)
    // =========================================================================
    assign dma_bready  = (dma_state == DMA_WRITE_RESP);

    // =========================================================================
    // VXU Stream (MEM→VXU)
    // =========================================================================
    assign vxu_stream_valid = (dma_state == DMA_PUSH_VXU) && rd_buf_valid;
    assign vxu_stream_data  = rd_buf;
    assign vxu_stream_last  = (dma_state == DMA_PUSH_VXU) && (words_left == 1);

    // =========================================================================
    // VXU Source (VXU→MEM)
    // =========================================================================
    assign vxu_source_ready = (dma_state == DMA_VXU_RD);

    // =========================================================================
    // IRQ
    // =========================================================================
    assign dma_irq = stat_irq;
    assign dbg_busy = stat_busy;

    // =========================================================================
    // Debug Counters
    // =========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dbg_wr_count    <= 32'd0;
            dbg_rd_count    <= 32'd0;
            dbg_burst_count <= 32'd0;
            dbg_stall_count <= 32'd0;
        end else begin
            if (dma_state == DMA_WRITE_RESP && dma_bvalid && dma_bresp == 2'b00)
                dbg_wr_count <= dbg_wr_count + 1;

            if (dma_state == DMA_FETCH_RD_DATA && dma_rvalid)
                dbg_rd_count <= dbg_rd_count + 1;

            if (dma_state == DMA_FETCH_RD_ADDR && !dma_arready)
                dbg_stall_count <= dbg_stall_count + 1;

            if (dma_state == DMA_WRITE_ADDR && !dma_awready)
                dbg_stall_count <= dbg_stall_count + 1;

            // Burst count: increments once per burst (every BURST_LEN words)
            if (dma_state == DMA_WRITE_RESP && dma_bvalid)
                if (dbg_wr_count[3:0] == 4'h0)
                    dbg_burst_count <= dbg_burst_count + 1;
        end
    end

endmodule
// =============================================================================
// END: dma_controller.sv
// =============================================================================
