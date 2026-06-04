// =============================================================================
// FILE: rtl/peripherals/spi_master.sv
// PROJECT: NeuroRV Edge SoC
// MODULE: spi_master
// DESCRIPTION: Full-duplex SPI Master, 4-wire, 8-bit transfers.
//              Supports CPOL/CPHA modes 0-3, configurable clock divider,
//              shift-register based, AXI-lite register interface.
// SYNTHESIZABLE: Yes (Yosys + Verilator compatible)
// =============================================================================

`timescale 1ns/1ps

module spi_master #(
    parameter int unsigned ADDR_BITS = 3    // 8 registers
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

    // SPI physical pins
    output logic        spi_sclk,
    output logic        spi_mosi,
    input  logic        spi_miso,
    output logic        spi_cs_n,   // active-low chip select

    // Interrupt
    output logic        spi_irq
);

    // =========================================================================
    // Register Map
    // 0x0 = TX_DATA  (WO): byte to transmit, writing triggers transfer
    // 0x1 = RX_DATA  (RO): last received byte
    // 0x2 = CTRL     (RW): [0]=cpol, [1]=cpha, [2]=cs_auto, [3]=irq_en,
    //                       [4]=lsb_first
    // 0x3 = STATUS   (RO): [0]=busy, [1]=done, [2]=rx_valid
    // 0x4 = CLK_DIV  (RW): SCLK = clk / (2 * (clk_div+1))
    // 0x5 = CS_CTRL  (WO): [0]=manual CS assert, [1]=manual CS deassert
    // 0x6 = IRQ_CLR  (WO): clear done/irq flag
    // 0x7 = DBG_CNT  (RO): transfer count
    // =========================================================================
    localparam logic [ADDR_BITS-1:0]
        REG_TX_DATA = 3'h0,
        REG_RX_DATA = 3'h1,
        REG_CTRL    = 3'h2,
        REG_STATUS  = 3'h3,
        REG_CLK_DIV = 3'h4,
        REG_CS_CTRL = 3'h5,
        REG_IRQ_CLR = 3'h6,
        REG_DBG_CNT = 3'h7;

    // =========================================================================
    // Config registers
    // =========================================================================
    logic        cfg_cpol;       // clock polarity
    logic        cfg_cpha;       // clock phase
    logic        cfg_cs_auto;    // auto-assert CS during transfer
    logic        cfg_irq_en;
    logic        cfg_lsb_first;
    logic [15:0] cfg_clk_div;    // clock divider

    // =========================================================================
    // SPI FSM
    // =========================================================================
    typedef enum logic [2:0] {
        SPI_IDLE    = 3'h0,
        SPI_CS_LEAD = 3'h1,   // CS setup time (1 half-period)
        SPI_SHIFT   = 3'h2,
        SPI_CS_HOLD = 3'h3,   // CS hold time (1 half-period)
        SPI_DONE    = 3'h4
    } spi_state_t;

    spi_state_t  spi_state;
    logic [7:0]  tx_shift;
    logic [7:0]  rx_shift;
    logic [7:0]  rx_data_reg;
    logic [2:0]  bit_cnt;      // 0..7
    logic [15:0] clk_cnt;      // clock divider counter
    logic        sclk_r;       // registered SCLK
    logic        busy;
    logic        done;
    logic        rx_valid;
    logic        irq_sticky;
    logic [31:0] dbg_xfer_cnt;

    // Half-period clock tick
    logic half_tick;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            clk_cnt   <= 16'd0;
            half_tick <= 1'b0;
        end else begin
            half_tick <= 1'b0;
            if (clk_cnt == 16'd0) begin
                clk_cnt   <= cfg_clk_div;
                half_tick <= 1'b1;
            end else begin
                clk_cnt <= clk_cnt - 1;
            end
        end
    end

    // =========================================================================
    // SPI FSM + Shift logic
    // =========================================================================
    // CPOL=0: SCLK idle low;  CPOL=1: SCLK idle high
    // CPHA=0: sample on leading edge, shift on trailing
    // CPHA=1: shift on leading edge, sample on trailing

    logic leading_edge;   // asserted when SCLK transitions to active
    logic trailing_edge;  // asserted when SCLK transitions to idle

    // Track SCLK edges during shift
    logic sclk_toggle;
    assign sclk_toggle   = half_tick && (spi_state == SPI_SHIFT);
    assign leading_edge  = sclk_toggle && (sclk_r == cfg_cpol);   // about to become active
    assign trailing_edge = sclk_toggle && (sclk_r != cfg_cpol);

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            spi_state   <= SPI_IDLE;
            tx_shift    <= 8'h00;
            rx_shift    <= 8'h00;
            rx_data_reg <= 8'h00;
            bit_cnt     <= 3'd0;
            sclk_r      <= 1'b0;
            spi_cs_n    <= 1'b1;
            spi_mosi    <= 1'b0;
            busy        <= 1'b0;
            done        <= 1'b0;
            rx_valid    <= 1'b0;
            irq_sticky  <= 1'b0;
            dbg_xfer_cnt<= 32'd0;
        end else begin
            done     <= 1'b0;

            case (spi_state)
                SPI_IDLE: begin
                    sclk_r <= cfg_cpol;   // idle clock level
                    busy   <= 1'b0;
                    // Transfer triggered by CPU writing TX_DATA (see reg write)
                end

                SPI_CS_LEAD: begin
                    // Assert CS, wait one half-period before shifting
                    if (cfg_cs_auto) spi_cs_n <= 1'b0;
                    if (half_tick) begin
                        spi_state <= SPI_SHIFT;
                        bit_cnt   <= 3'd0;
                        // For CPHA=1, output first bit now (before leading edge)
                        if (cfg_cpha) begin
                            spi_mosi <= cfg_lsb_first ? tx_shift[0] : tx_shift[7];
                            if (!cfg_lsb_first) tx_shift <= {tx_shift[6:0], 1'b0};
                            else                tx_shift <= {1'b0, tx_shift[7:1]};
                        end
                    end
                end

                SPI_SHIFT: begin
                    if (half_tick) begin
                        sclk_r <= ~sclk_r;   // toggle clock

                        if (leading_edge) begin
                            // CPHA=0: sample on leading edge
                            if (!cfg_cpha) begin
                                rx_shift <= cfg_lsb_first ?
                                            {spi_miso, rx_shift[7:1]} :
                                            {rx_shift[6:0], spi_miso};
                            end
                            // CPHA=1: shift out next bit on leading edge
                            if (cfg_cpha && bit_cnt != 3'd7) begin
                                spi_mosi <= cfg_lsb_first ? tx_shift[0] : tx_shift[7];
                                if (!cfg_lsb_first) tx_shift <= {tx_shift[6:0], 1'b0};
                                else                tx_shift <= {1'b0, tx_shift[7:1]};
                            end
                        end

                        if (trailing_edge) begin
                            // CPHA=0: shift out next bit on trailing edge
                            if (!cfg_cpha) begin
                                if (bit_cnt != 3'd7) begin
                                    spi_mosi <= cfg_lsb_first ? tx_shift[0] : tx_shift[7];
                                    if (!cfg_lsb_first) tx_shift <= {tx_shift[6:0], 1'b0};
                                    else                tx_shift <= {1'b0, tx_shift[7:1]};
                                end
                            end
                            // CPHA=1: sample on trailing edge
                            if (cfg_cpha) begin
                                rx_shift <= cfg_lsb_first ?
                                            {spi_miso, rx_shift[7:1]} :
                                            {rx_shift[6:0], spi_miso};
                            end

                            // Advance bit counter on every trailing edge
                            if (bit_cnt == 3'd7) begin
                                spi_state   <= SPI_CS_HOLD;
                                rx_data_reg <= cfg_lsb_first ?
                                              {spi_miso, rx_shift[7:1]} :
                                              {rx_shift[6:0], spi_miso};
                                rx_valid    <= 1'b1;
                            end else begin
                                bit_cnt <= bit_cnt + 1;
                            end
                        end
                    end
                end

                SPI_CS_HOLD: begin
                    if (half_tick) begin
                        if (cfg_cs_auto) spi_cs_n <= 1'b1;
                        spi_state    <= SPI_DONE;
                        dbg_xfer_cnt <= dbg_xfer_cnt + 1;
                    end
                end

                SPI_DONE: begin
                    done       <= 1'b1;
                    busy       <= 1'b0;
                    irq_sticky <= cfg_irq_en;
                    spi_state  <= SPI_IDLE;
                    sclk_r     <= cfg_cpol;
                end

                default: spi_state <= SPI_IDLE;
            endcase

            // Clear IRQ
            if (reg_req && reg_we && reg_addr == REG_IRQ_CLR) begin
                irq_sticky <= 1'b0;
                rx_valid   <= 1'b0;
            end
        end
    end

    // Output SCLK
    assign spi_sclk = sclk_r;

    // =========================================================================
    // Register Write
    // =========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cfg_cpol      <= 1'b0;
            cfg_cpha      <= 1'b0;
            cfg_cs_auto   <= 1'b1;
            cfg_irq_en    <= 1'b0;
            cfg_lsb_first <= 1'b0;
            cfg_clk_div   <= 16'd4;
        end else begin
            if (reg_req && reg_we) begin
                case (reg_addr)
                    REG_TX_DATA: begin
                        if (!busy && spi_state == SPI_IDLE) begin
                            // Pre-load MOSI with MSB (or LSB if lsb_first)
                            tx_shift  <= reg_wdata[7:0];
                            spi_mosi  <= cfg_lsb_first ? reg_wdata[0] : reg_wdata[7];
                            busy      <= 1'b1;
                            spi_state <= SPI_CS_LEAD;
                        end
                    end
                    REG_CTRL: begin
                        cfg_cpol      <= reg_wdata[0];
                        cfg_cpha      <= reg_wdata[1];
                        cfg_cs_auto   <= reg_wdata[2];
                        cfg_irq_en    <= reg_wdata[3];
                        cfg_lsb_first <= reg_wdata[4];
                    end
                    REG_CLK_DIV: cfg_clk_div <= reg_wdata[15:0];
                    REG_CS_CTRL: begin
                        if (reg_wdata[0]) spi_cs_n <= 1'b0;
                        if (reg_wdata[1]) spi_cs_n <= 1'b1;
                    end
                    default: ;
                endcase
            end
        end
    end

    assign spi_irq = irq_sticky;

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
                    REG_RX_DATA: reg_rdata <= {24'd0, rx_data_reg};
                    REG_CTRL:    reg_rdata <= {27'd0, cfg_lsb_first, cfg_irq_en,
                                               cfg_cs_auto, cfg_cpha, cfg_cpol};
                    REG_STATUS:  reg_rdata <= {29'd0, rx_valid, done, busy};
                    REG_CLK_DIV: reg_rdata <= {16'd0, cfg_clk_div};
                    REG_DBG_CNT: reg_rdata <= dbg_xfer_cnt;
                    default:     reg_rdata <= 32'd0;
                endcase
            end
        end
    end

endmodule
// =============================================================================
// END: spi_master.sv
// =============================================================================
