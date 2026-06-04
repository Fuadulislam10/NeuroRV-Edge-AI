// =============================================================================
// FILE: rtl/peripherals/uart_16550.sv
// PROJECT: NeuroRV Edge — Phase 5 Peripheral Subsystem
// MODULE: uart_16550
// DESCRIPTION: 16550-style simplified UART with TX/RX FIFOs, baud rate
//              generator, and AXI-lite register interface.
//
// Register Map (32-bit aligned, byte offset):
//   0x00  TX_REG       [7:0]  Write to transmit data
//   0x04  RX_REG       [7:0]  Read received data
//   0x08  CTRL_REG     [1:0]  [0]=TX_EN, [1]=RX_EN
//   0x0C  STATUS_REG   [3:0]  [0]=TX_READY, [1]=RX_READY, [2]=TX_BUSY, [3]=RX_OVR
//   0x10  BAUD_DIV     [15:0] Baud divider = clk_freq / (baud_rate * 16)
//
// FIFO depth: 16 entries (TX and RX)
// Data format: 8N1 (8 data, no parity, 1 stop)
// Synchronous reset, single clock domain.
// =============================================================================

`timescale 1ns/1ps

module uart_16550 #(
    parameter int CLK_FREQ_HZ  = 50_000_000,
    parameter int BAUD_DEFAULT = 115200,
    parameter int FIFO_DEPTH   = 16
)(
    // Clock and reset
    input  logic        clk,
    input  logic        rst_n,

    // AXI-lite subordinate interface (simplified)
    input  logic [4:0]  s_axil_awaddr,
    input  logic        s_axil_awvalid,
    output logic        s_axil_awready,
    input  logic [31:0] s_axil_wdata,
    input  logic [3:0]  s_axil_wstrb,
    input  logic        s_axil_wvalid,
    output logic        s_axil_wready,
    output logic [1:0]  s_axil_bresp,
    output logic        s_axil_bvalid,
    input  logic        s_axil_bready,
    input  logic [4:0]  s_axil_araddr,
    input  logic        s_axil_arvalid,
    output logic        s_axil_arready,
    output logic [31:0] s_axil_rdata,
    output logic [1:0]  s_axil_rresp,
    output logic        s_axil_rvalid,
    input  logic        s_axil_rready,

    // UART physical interface
    output logic        uart_tx,
    input  logic        uart_rx,

    // Interrupt
    output logic        irq_tx_empty,
    output logic        irq_rx_ready
);

    // -------------------------------------------------------------------------
    // Local parameters
    // -------------------------------------------------------------------------
    localparam int BAUD_DIV_DEFAULT = CLK_FREQ_HZ / (BAUD_DEFAULT * 16);
    localparam int FIFO_PTR_W       = $clog2(FIFO_DEPTH);

    // Register addresses (word-aligned, use bits [4:2])
    localparam logic [2:0] ADDR_TX_REG  = 3'h0;
    localparam logic [2:0] ADDR_RX_REG  = 3'h1;
    localparam logic [2:0] ADDR_CTRL    = 3'h2;
    localparam logic [2:0] ADDR_STATUS  = 3'h3;
    localparam logic [2:0] ADDR_BAUDDIV = 3'h4;

    // -------------------------------------------------------------------------
    // Registers
    // -------------------------------------------------------------------------
    logic [7:0]  ctrl_reg;      // [0]=TX_EN, [1]=RX_EN
    logic [15:0] baud_div_reg;
    logic [3:0]  status_reg;    // [0]=TX_READY,[1]=RX_READY,[2]=TX_BUSY,[3]=RX_OVR

    // -------------------------------------------------------------------------
    // Baud rate generator (16x oversampling)
    // -------------------------------------------------------------------------
    logic [15:0] baud_cnt;
    logic        baud_tick;     // 16x baud tick

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            baud_cnt  <= '0;
            baud_tick <= 1'b0;
        end else begin
            if (baud_cnt == baud_div_reg - 1) begin
                baud_cnt  <= '0;
                baud_tick <= 1'b1;
            end else begin
                baud_cnt  <= baud_cnt + 1'b1;
                baud_tick <= 1'b0;
            end
        end
    end

    // -------------------------------------------------------------------------
    // TX FIFO
    // -------------------------------------------------------------------------
    logic [7:0]           tx_fifo [0:FIFO_DEPTH-1];
    logic [FIFO_PTR_W:0]  tx_wr_ptr, tx_rd_ptr;
    logic                 tx_fifo_empty, tx_fifo_full;
    logic                 tx_fifo_wr_en, tx_fifo_rd_en;
    logic [7:0]           tx_fifo_rdata;

    assign tx_fifo_empty = (tx_wr_ptr == tx_rd_ptr);
    assign tx_fifo_full  = (tx_wr_ptr[FIFO_PTR_W] != tx_rd_ptr[FIFO_PTR_W]) &&
                           (tx_wr_ptr[FIFO_PTR_W-1:0] == tx_rd_ptr[FIFO_PTR_W-1:0]);
    assign tx_fifo_rdata = tx_fifo[tx_rd_ptr[FIFO_PTR_W-1:0]];

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx_wr_ptr <= '0;
        end else if (tx_fifo_wr_en && !tx_fifo_full) begin
            tx_fifo[tx_wr_ptr[FIFO_PTR_W-1:0]] <= s_axil_wdata[7:0];
            tx_wr_ptr <= tx_wr_ptr + 1'b1;
        end
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx_rd_ptr <= '0;
        end else if (tx_fifo_rd_en && !tx_fifo_empty) begin
            tx_rd_ptr <= tx_rd_ptr + 1'b1;
        end
    end

    // -------------------------------------------------------------------------
    // RX FIFO
    // -------------------------------------------------------------------------
    logic [7:0]           rx_fifo [0:FIFO_DEPTH-1];
    logic [FIFO_PTR_W:0]  rx_wr_ptr, rx_rd_ptr;
    logic                 rx_fifo_empty, rx_fifo_full;
    logic                 rx_fifo_wr_en, rx_fifo_rd_en;
    logic [7:0]           rx_fifo_rdata;
    logic [7:0]           rx_byte_in;

    assign rx_fifo_empty = (rx_wr_ptr == rx_rd_ptr);
    assign rx_fifo_full  = (rx_wr_ptr[FIFO_PTR_W] != rx_rd_ptr[FIFO_PTR_W]) &&
                           (rx_wr_ptr[FIFO_PTR_W-1:0] == rx_rd_ptr[FIFO_PTR_W-1:0]);
    assign rx_fifo_rdata = rx_fifo[rx_rd_ptr[FIFO_PTR_W-1:0]];

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_wr_ptr <= '0;
        end else if (rx_fifo_wr_en) begin
            if (!rx_fifo_full) begin
                rx_fifo[rx_wr_ptr[FIFO_PTR_W-1:0]] <= rx_byte_in;
                rx_wr_ptr <= rx_wr_ptr + 1'b1;
            end
        end
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_rd_ptr <= '0;
        end else if (rx_fifo_rd_en && !rx_fifo_empty) begin
            rx_rd_ptr <= rx_rd_ptr + 1'b1;
        end
    end

    // -------------------------------------------------------------------------
    // TX State Machine
    // -------------------------------------------------------------------------
    typedef enum logic [1:0] {
        TX_IDLE  = 2'b00,
        TX_START = 2'b01,
        TX_DATA  = 2'b10,
        TX_STOP  = 2'b11
    } tx_state_e;

    tx_state_e   tx_state;
    logic [7:0]  tx_shift_reg;
    logic [3:0]  tx_bit_cnt;   // counts 16 ticks per bit
    logic [2:0]  tx_data_idx;  // which data bit (0-7)
    logic        tx_busy;

    assign tx_fifo_rd_en = (tx_state == TX_IDLE) && !tx_fifo_empty && ctrl_reg[0];

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx_state    <= TX_IDLE;
            tx_shift_reg<= '1;
            tx_bit_cnt  <= '0;
            tx_data_idx <= '0;
            tx_busy     <= 1'b0;
            uart_tx     <= 1'b1;
        end else begin
            case (tx_state)
                TX_IDLE: begin
                    uart_tx  <= 1'b1;
                    tx_busy  <= 1'b0;
                    if (!tx_fifo_empty && ctrl_reg[0]) begin
                        tx_shift_reg <= tx_fifo_rdata;
                        tx_state     <= TX_START;
                        tx_bit_cnt   <= '0;
                        tx_busy      <= 1'b1;
                    end
                end

                TX_START: begin
                    uart_tx <= 1'b0;  // start bit
                    if (baud_tick) begin
                        if (tx_bit_cnt == 4'd15) begin
                            tx_bit_cnt  <= '0;
                            tx_data_idx <= '0;
                            tx_state    <= TX_DATA;
                        end else begin
                            tx_bit_cnt <= tx_bit_cnt + 1'b1;
                        end
                    end
                end

                TX_DATA: begin
                    uart_tx <= tx_shift_reg[tx_data_idx];
                    if (baud_tick) begin
                        if (tx_bit_cnt == 4'd15) begin
                            tx_bit_cnt <= '0;
                            if (tx_data_idx == 3'd7) begin
                                tx_state <= TX_STOP;
                            end else begin
                                tx_data_idx <= tx_data_idx + 1'b1;
                            end
                        end else begin
                            tx_bit_cnt <= tx_bit_cnt + 1'b1;
                        end
                    end
                end

                TX_STOP: begin
                    uart_tx <= 1'b1;  // stop bit
                    if (baud_tick) begin
                        if (tx_bit_cnt == 4'd15) begin
                            tx_bit_cnt <= '0;
                            tx_state   <= TX_IDLE;
                        end else begin
                            tx_bit_cnt <= tx_bit_cnt + 1'b1;
                        end
                    end
                end

                default: tx_state <= TX_IDLE;
            endcase
        end
    end

    // -------------------------------------------------------------------------
    // RX State Machine (16x oversampling)
    // -------------------------------------------------------------------------
    typedef enum logic [1:0] {
        RX_IDLE  = 2'b00,
        RX_START = 2'b01,
        RX_DATA  = 2'b10,
        RX_STOP  = 2'b11
    } rx_state_e;

    rx_state_e   rx_state;
    logic [7:0]  rx_shift_reg;
    logic [3:0]  rx_bit_cnt;
    logic [2:0]  rx_data_idx;
    logic        rx_ovr;
    logic        uart_rx_sync0, uart_rx_sync1;

    // Double-flop synchronizer for async RX input
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            uart_rx_sync0 <= 1'b1;
            uart_rx_sync1 <= 1'b1;
        end else begin
            uart_rx_sync0 <= uart_rx;
            uart_rx_sync1 <= uart_rx_sync0;
        end
    end

    assign rx_fifo_wr_en = 1'b0; // driven by FSM below
    assign rx_byte_in    = 8'h00;

    // Override above with proper FSM signals via always_ff
    logic        rx_wr_en_int;
    logic [7:0]  rx_byte_int;
    assign rx_fifo_wr_en = rx_wr_en_int;
    assign rx_byte_in    = rx_byte_int;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_state     <= RX_IDLE;
            rx_shift_reg <= '0;
            rx_bit_cnt   <= '0;
            rx_data_idx  <= '0;
            rx_ovr       <= 1'b0;
            rx_wr_en_int <= 1'b0;
            rx_byte_int  <= '0;
        end else begin
            rx_wr_en_int <= 1'b0;  // default: no write

            case (rx_state)
                RX_IDLE: begin
                    rx_ovr <= 1'b0;
                    if (!uart_rx_sync1 && ctrl_reg[1]) begin  // detect start bit
                        rx_state   <= RX_START;
                        rx_bit_cnt <= '0;
                    end
                end

                RX_START: begin
                    // sample at middle of start bit (8 ticks)
                    if (baud_tick) begin
                        if (rx_bit_cnt == 4'd7) begin
                            if (!uart_rx_sync1) begin  // confirm start bit
                                rx_bit_cnt  <= '0;
                                rx_data_idx <= '0;
                                rx_state    <= RX_DATA;
                            end else begin
                                rx_state <= RX_IDLE;  // false start
                            end
                        end else begin
                            rx_bit_cnt <= rx_bit_cnt + 1'b1;
                        end
                    end
                end

                RX_DATA: begin
                    if (baud_tick) begin
                        if (rx_bit_cnt == 4'd15) begin
                            rx_shift_reg[rx_data_idx] <= uart_rx_sync1;
                            rx_bit_cnt <= '0;
                            if (rx_data_idx == 3'd7) begin
                                rx_state <= RX_STOP;
                            end else begin
                                rx_data_idx <= rx_data_idx + 1'b1;
                            end
                        end else begin
                            rx_bit_cnt <= rx_bit_cnt + 1'b1;
                        end
                    end
                end

                RX_STOP: begin
                    if (baud_tick) begin
                        if (rx_bit_cnt == 4'd15) begin
                            rx_bit_cnt <= '0;
                            rx_state   <= RX_IDLE;
                            if (uart_rx_sync1) begin  // valid stop bit
                                if (!rx_fifo_full) begin
                                    rx_wr_en_int <= 1'b1;
                                    rx_byte_int  <= rx_shift_reg;
                                end else begin
                                    rx_ovr <= 1'b1;
                                end
                            end
                        end else begin
                            rx_bit_cnt <= rx_bit_cnt + 1'b1;
                        end
                    end
                end

                default: rx_state <= RX_IDLE;
            endcase
        end
    end

    // -------------------------------------------------------------------------
    // Status and interrupt
    // -------------------------------------------------------------------------
    always_comb begin
        status_reg[0] = !tx_fifo_full;   // TX_READY: can accept data
        status_reg[1] = !rx_fifo_empty;  // RX_READY: data available
        status_reg[2] = tx_busy;          // TX_BUSY
        status_reg[3] = rx_ovr;           // RX_OVERFLOW
    end

    assign irq_tx_empty = tx_fifo_empty;
    assign irq_rx_ready = !rx_fifo_empty;

    // -------------------------------------------------------------------------
    // AXI-lite Write Channel
    // -------------------------------------------------------------------------
    logic        aw_active;
    logic [4:0]  aw_addr_lat;
    logic        w_active;

    assign s_axil_awready = 1'b1;
    assign s_axil_wready  = 1'b1;
    assign s_axil_bresp   = 2'b00;  // OKAY

    // TX FIFO write enable: only when writing to TX_REG
    assign tx_fifo_wr_en = s_axil_wvalid && s_axil_awvalid &&
                           (s_axil_awaddr[4:2] == ADDR_TX_REG);

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ctrl_reg     <= 8'h03;  // TX+RX enabled by default
            baud_div_reg <= BAUD_DIV_DEFAULT[15:0];
            s_axil_bvalid<= 1'b0;
        end else begin
            if (s_axil_bvalid && s_axil_bready)
                s_axil_bvalid <= 1'b0;

            if (s_axil_awvalid && s_axil_wvalid) begin
                s_axil_bvalid <= 1'b1;
                case (s_axil_awaddr[4:2])
                    ADDR_CTRL:    ctrl_reg     <= s_axil_wdata[7:0];
                    ADDR_BAUDDIV: baud_div_reg <= s_axil_wdata[15:0];
                    default: ;
                endcase
            end
        end
    end

    // -------------------------------------------------------------------------
    // AXI-lite Read Channel
    // -------------------------------------------------------------------------
    assign s_axil_arready = 1'b1;
    assign s_axil_rresp   = 2'b00;

    assign rx_fifo_rd_en  = s_axil_arvalid && (s_axil_araddr[4:2] == ADDR_RX_REG);

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s_axil_rvalid <= 1'b0;
            s_axil_rdata  <= '0;
        end else begin
            if (s_axil_rvalid && s_axil_rready)
                s_axil_rvalid <= 1'b0;

            if (s_axil_arvalid && s_axil_arready) begin
                s_axil_rvalid <= 1'b1;
                case (s_axil_araddr[4:2])
                    ADDR_TX_REG:  s_axil_rdata <= 32'h0;
                    ADDR_RX_REG:  s_axil_rdata <= {24'h0, rx_fifo_rdata};
                    ADDR_CTRL:    s_axil_rdata <= {24'h0, ctrl_reg};
                    ADDR_STATUS:  s_axil_rdata <= {28'h0, status_reg};
                    ADDR_BAUDDIV: s_axil_rdata <= {16'h0, baud_div_reg};
                    default:      s_axil_rdata <= 32'hDEADBEEF;
                endcase
            end
        end
    end

endmodule
// =============================================================================
// END OF FILE: uart_16550.sv
// =============================================================================
