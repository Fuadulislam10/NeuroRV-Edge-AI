// =============================================================================
// FILE: rtl/peripherals/uart_16550.sv
// PROJECT: NeuroRV Edge SoC
// MODULE: uart_16550
// DESCRIPTION: 16550-style simplified UART with 8N1 framing, TX/RX FIFOs,
//              baud rate generator, and AXI-lite register interface.
// SYNTHESIZABLE: Yes (Yosys + Verilator compatible)
// =============================================================================

`timescale 1ns/1ps

module uart_16550 #(
    parameter int unsigned FIFO_DEPTH   = 16,    // TX and RX FIFO depth
    parameter int unsigned CLK_FREQ_HZ  = 50_000_000,
    parameter int unsigned DEFAULT_BAUD = 115200,
    parameter int unsigned ADDR_BITS    = 3      // 8 registers
)(
    input  logic        clk,
    input  logic        rst_n,

    // AXI-lite register interface
    input  logic                    reg_req,
    input  logic                    reg_we,
    input  logic [ADDR_BITS-1:0]    reg_addr,
    input  logic [31:0]             reg_wdata,
    output logic [31:0]             reg_rdata,
    output logic                    reg_ack,

    // UART physical pins
    output logic        uart_tx,
    input  logic        uart_rx,

    // Interrupt
    output logic        uart_irq
);

    // =========================================================================
    // Register Map
    // 0x0 = RX_DATA  (RO): received byte, clears RX_READY on read
    // 0x1 = TX_DATA  (WO): byte to transmit
    // 0x2 = STATUS   (RO): [0]=tx_ready, [1]=rx_ready, [2]=tx_busy,
    //                       [3]=rx_busy, [4]=rx_overrun, [5]=frame_err
    // 0x3 = CTRL     (RW): [0]=tx_en, [1]=rx_en, [2]=irq_en
    // 0x4 = BAUD_DIV (RW): baud clock divider (16-bit)
    // 0x5 = IRQ_CLR  (WO): write any to clear irq flags
    // 0x6 = TX_LEVEL (RO): TX FIFO fill level
    // 0x7 = RX_LEVEL (RO): RX FIFO fill level
    // =========================================================================
    localparam logic [ADDR_BITS-1:0]
        REG_RX_DATA  = 3'h0,
        REG_TX_DATA  = 3'h1,
        REG_STATUS   = 3'h2,
        REG_CTRL     = 3'h3,
        REG_BAUD_DIV = 3'h4,
        REG_IRQ_CLR  = 3'h5,
        REG_TX_LEVEL = 3'h6,
        REG_RX_LEVEL = 3'h7;

    localparam int unsigned FIFO_AW = $clog2(FIFO_DEPTH);
    localparam int unsigned DEFAULT_DIV = CLK_FREQ_HZ / (DEFAULT_BAUD * 16);

    // =========================================================================
    // Control / Config registers
    // =========================================================================
    logic        ctrl_tx_en;
    logic        ctrl_rx_en;
    logic        ctrl_irq_en;
    logic [15:0] baud_div;      // baud16 tick every (baud_div) clocks

    // Status flags
    logic        rx_overrun;
    logic        frame_err;
    logic        irq_tx_rdy;   // TX FIFO has room
    logic        irq_rx_rdy;   // RX FIFO has data

    // =========================================================================
    // FIFO implementation (synchronous, single-clock)
    // =========================================================================
    // TX FIFO
    logic [7:0]         tx_fifo  [0:FIFO_DEPTH-1];
    logic [FIFO_AW:0]   tx_wr_ptr, tx_rd_ptr;
    logic               tx_fifo_full, tx_fifo_empty;
    logic [FIFO_AW:0]   tx_level;

    assign tx_level      = tx_wr_ptr - tx_rd_ptr;
    assign tx_fifo_full  = (tx_level == FIFO_AW'(FIFO_DEPTH));
    assign tx_fifo_empty = (tx_level == 0);

    // RX FIFO
    logic [7:0]         rx_fifo  [0:FIFO_DEPTH-1];
    logic [FIFO_AW:0]   rx_wr_ptr, rx_rd_ptr;
    logic               rx_fifo_full, rx_fifo_empty;
    logic [FIFO_AW:0]   rx_level;

    assign rx_level      = rx_wr_ptr - rx_rd_ptr;
    assign rx_fifo_full  = (rx_level == FIFO_AW'(FIFO_DEPTH));
    assign rx_fifo_empty = (rx_level == 0);

    // =========================================================================
    // Baud rate generator (x16 oversampling)
    // =========================================================================
    logic [15:0] baud_cnt;
    logic        baud16_tick;   // fires 16x per bit period

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            baud_cnt    <= 16'd0;
            baud16_tick <= 1'b0;
        end else begin
            baud16_tick <= 1'b0;
            if (baud_cnt == 16'd0) begin
                baud_cnt    <= baud_div - 1;
                baud16_tick <= 1'b1;
            end else begin
                baud_cnt <= baud_cnt - 1;
            end
        end
    end

    // =========================================================================
    // TX Shift Register FSM
    // =========================================================================
    typedef enum logic [2:0] {
        TX_IDLE  = 3'h0,
        TX_START = 3'h1,
        TX_DATA  = 3'h2,
        TX_STOP  = 3'h3
    } tx_state_t;

    tx_state_t   tx_state;
    logic [7:0]  tx_shift;
    logic [2:0]  tx_bit_cnt;
    logic [3:0]  tx_baud_cnt;  // 0..15 sub-bit count
    logic        tx_busy;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx_state    <= TX_IDLE;
            tx_shift    <= 8'hFF;
            tx_bit_cnt  <= 3'd0;
            tx_baud_cnt <= 4'd0;
            uart_tx     <= 1'b1;   // idle high
            tx_rd_ptr   <= '0;
            tx_busy     <= 1'b0;
        end else begin
            if (baud16_tick) begin
                case (tx_state)
                    TX_IDLE: begin
                        uart_tx <= 1'b1;
                        tx_busy <= 1'b0;
                        if (!tx_fifo_empty && ctrl_tx_en) begin
                            tx_shift   <= tx_fifo[tx_rd_ptr[FIFO_AW-1:0]];
                            tx_rd_ptr  <= tx_rd_ptr + 1;
                            tx_state   <= TX_START;
                            tx_baud_cnt<= 4'd0;
                            tx_busy    <= 1'b1;
                        end
                    end
                    TX_START: begin
                        uart_tx <= 1'b0;   // start bit
                        if (tx_baud_cnt == 4'd15) begin
                            tx_baud_cnt <= 4'd0;
                            tx_bit_cnt  <= 3'd0;
                            tx_state    <= TX_DATA;
                        end else begin
                            tx_baud_cnt <= tx_baud_cnt + 1;
                        end
                    end
                    TX_DATA: begin
                        uart_tx <= tx_shift[0];
                        if (tx_baud_cnt == 4'd15) begin
                            tx_baud_cnt <= 4'd0;
                            tx_shift    <= {1'b1, tx_shift[7:1]};
                            if (tx_bit_cnt == 3'd7) begin
                                tx_state <= TX_STOP;
                            end else begin
                                tx_bit_cnt <= tx_bit_cnt + 1;
                            end
                        end else begin
                            tx_baud_cnt <= tx_baud_cnt + 1;
                        end
                    end
                    TX_STOP: begin
                        uart_tx <= 1'b1;   // stop bit
                        if (tx_baud_cnt == 4'd15) begin
                            tx_baud_cnt <= 4'd0;
                            tx_state    <= TX_IDLE;
                        end else begin
                            tx_baud_cnt <= tx_baud_cnt + 1;
                        end
                    end
                    default: tx_state <= TX_IDLE;
                endcase
            end
        end
    end

    // =========================================================================
    // RX Shift Register FSM (x16 oversampling, sample at bit center = count 7)
    // =========================================================================
    typedef enum logic [2:0] {
        RX_IDLE  = 3'h0,
        RX_START = 3'h1,
        RX_DATA  = 3'h2,
        RX_STOP  = 3'h3
    } rx_state_t;

    rx_state_t   rx_state;
    logic [7:0]  rx_shift;
    logic [2:0]  rx_bit_cnt;
    logic [3:0]  rx_baud_cnt;
    logic        rx_busy;

    // 2-FF synchronizer for uart_rx
    logic rx_sync1, rx_sync2, rx_prev;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_sync1 <= 1'b1;
            rx_sync2 <= 1'b1;
            rx_prev  <= 1'b1;
        end else begin
            rx_sync1 <= uart_rx;
            rx_sync2 <= rx_sync1;
            rx_prev  <= rx_sync2;
        end
    end

    logic rx_falling_edge;
    assign rx_falling_edge = rx_prev & ~rx_sync2;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_state    <= RX_IDLE;
            rx_shift    <= 8'h00;
            rx_bit_cnt  <= 3'd0;
            rx_baud_cnt <= 4'd0;
            rx_busy     <= 1'b0;
            rx_overrun  <= 1'b0;
            frame_err   <= 1'b0;
            rx_wr_ptr   <= '0;
        end else begin
            rx_overrun <= 1'b0;  // pulse
            case (rx_state)
                RX_IDLE: begin
                    rx_busy <= 1'b0;
                    if (rx_falling_edge && ctrl_rx_en) begin
                        rx_state    <= RX_START;
                        rx_baud_cnt <= 4'd0;
                        rx_busy     <= 1'b1;
                    end
                end
                RX_START: begin
                    if (baud16_tick) begin
                        if (rx_baud_cnt == 4'd7) begin   // center of start bit
                            if (~rx_sync2) begin          // still low = valid start
                                rx_baud_cnt <= 4'd0;
                                rx_bit_cnt  <= 3'd0;
                                rx_state    <= RX_DATA;
                            end else begin
                                rx_state <= RX_IDLE;      // false start
                            end
                        end else begin
                            rx_baud_cnt <= rx_baud_cnt + 1;
                        end
                    end
                end
                RX_DATA: begin
                    if (baud16_tick) begin
                        if (rx_baud_cnt == 4'd15) begin
                            rx_baud_cnt        <= 4'd0;
                            rx_shift           <= {rx_sync2, rx_shift[7:1]};
                            if (rx_bit_cnt == 3'd7) begin
                                rx_state <= RX_STOP;
                            end else begin
                                rx_bit_cnt <= rx_bit_cnt + 1;
                            end
                        end else begin
                            rx_baud_cnt <= rx_baud_cnt + 1;
                        end
                    end
                end
                RX_STOP: begin
                    if (baud16_tick) begin
                        if (rx_baud_cnt == 4'd15) begin
                            rx_baud_cnt <= 4'd0;
                            rx_state    <= RX_IDLE;
                            if (rx_sync2) begin   // valid stop bit
                                if (!rx_fifo_full) begin
                                    rx_fifo[rx_wr_ptr[FIFO_AW-1:0]] <= rx_shift;
                                    rx_wr_ptr <= rx_wr_ptr + 1;
                                end else begin
                                    rx_overrun <= 1'b1;
                                end
                            end else begin
                                frame_err <= 1'b1;
                            end
                        end else begin
                            rx_baud_cnt <= rx_baud_cnt + 1;
                        end
                    end
                end
                default: rx_state <= RX_IDLE;
            endcase
        end
    end

    // =========================================================================
    // TX FIFO Write (CPU writes TX_DATA register)
    // =========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx_wr_ptr <= '0;
        end else begin
            if (reg_req && reg_we && reg_addr == REG_TX_DATA && !tx_fifo_full) begin
                tx_fifo[tx_wr_ptr[FIFO_AW-1:0]] <= reg_wdata[7:0];
                tx_wr_ptr <= tx_wr_ptr + 1;
            end
        end
    end

    // =========================================================================
    // RX FIFO Read (CPU reads RX_DATA register)
    // =========================================================================
    logic [7:0] rx_rdata_byte;
    assign rx_rdata_byte = rx_fifo_empty ? 8'h00 : rx_fifo[rx_rd_ptr[FIFO_AW-1:0]];

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_rd_ptr <= '0;
        end else begin
            if (reg_req && !reg_we && reg_addr == REG_RX_DATA && !rx_fifo_empty) begin
                rx_rd_ptr <= rx_rd_ptr + 1;
            end
        end
    end

    // =========================================================================
    // Control register writes
    // =========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ctrl_tx_en  <= 1'b1;
            ctrl_rx_en  <= 1'b1;
            ctrl_irq_en <= 1'b0;
            baud_div    <= DEFAULT_DIV[15:0];
        end else begin
            if (reg_req && reg_we) begin
                case (reg_addr)
                    REG_CTRL:     begin
                                    ctrl_tx_en  <= reg_wdata[0];
                                    ctrl_rx_en  <= reg_wdata[1];
                                    ctrl_irq_en <= reg_wdata[2];
                                  end
                    REG_BAUD_DIV: baud_div <= reg_wdata[15:0];
                    default: ;
                endcase
            end
        end
    end

    // =========================================================================
    // IRQ
    // =========================================================================
    logic irq_sticky;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            irq_sticky <= 1'b0;
        end else begin
            if (reg_req && reg_we && reg_addr == REG_IRQ_CLR)
                irq_sticky <= 1'b0;
            else if (!rx_fifo_empty || rx_overrun || frame_err)
                irq_sticky <= 1'b1;
        end
    end
    assign uart_irq = irq_sticky & ctrl_irq_en;

    // =========================================================================
    // Register Read
    // =========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            reg_rdata <= 32'd0;
            reg_ack   <= 1'b0;
        end else begin
            reg_ack   <= reg_req;
            reg_rdata <= 32'd0;
            if (reg_req && !reg_we) begin
                case (reg_addr)
                    REG_RX_DATA:  reg_rdata <= {24'd0, rx_rdata_byte};
                    REG_STATUS:   reg_rdata <= {26'd0, frame_err, rx_overrun,
                                                rx_busy, tx_busy,
                                                ~rx_fifo_empty, ~tx_fifo_full};
                    REG_CTRL:     reg_rdata <= {29'd0, ctrl_irq_en, ctrl_rx_en, ctrl_tx_en};
                    REG_BAUD_DIV: reg_rdata <= {16'd0, baud_div};
                    REG_TX_LEVEL: reg_rdata <= {27'd0, tx_level[FIFO_AW:0]};
                    REG_RX_LEVEL: reg_rdata <= {27'd0, rx_level[FIFO_AW:0]};
                    default:      reg_rdata <= 32'h0;
                endcase
            end
        end
    end

endmodule
// =============================================================================
// END: uart_16550.sv
// =============================================================================
