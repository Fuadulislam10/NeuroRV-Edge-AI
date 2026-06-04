// =============================================================================
// FILE: rtl/peripherals/spi_master.sv
// PROJECT: NeuroRV Edge — Phase 5 Peripheral Subsystem
// MODULE: spi_master
// DESCRIPTION: Full-duplex SPI master with configurable CPOL/CPHA modes,
//              8-bit transfer width, and AXI-lite register interface.
//
// Register Map (32-bit aligned, byte offset):
//   0x00  DATA_REG   [7:0]   Write = TX data, Read = last RX data
//   0x04  CTRL_REG   [7:0]   [0]=START, [1]=CPOL, [2]=CPHA, [3]=CS_AUTO
//   0x08  STATUS_REG [1:0]   [0]=BUSY, [1]=DONE
//   0x0C  CLK_DIV    [15:0]  SPI clock = sys_clk / (2 * (CLK_DIV + 1))
//
// SPI Modes:
//   CPOL=0, CPHA=0 → Mode 0   CPOL=0, CPHA=1 → Mode 1
//   CPOL=1, CPHA=0 → Mode 2   CPOL=1, CPHA=1 → Mode 3
//
// Synchronous reset, single clock domain.
// =============================================================================

`timescale 1ns/1ps

module spi_master #(
    parameter int CLK_DIV_DEFAULT = 4  // SPI clk = sys_clk / (2*(4+1)) = sys_clk/10
)(
    // Clock and reset
    input  logic        clk,
    input  logic        rst_n,

    // AXI-lite subordinate interface
    input  logic [3:0]  s_axil_awaddr,
    input  logic        s_axil_awvalid,
    output logic        s_axil_awready,
    input  logic [31:0] s_axil_wdata,
    input  logic [3:0]  s_axil_wstrb,
    input  logic        s_axil_wvalid,
    output logic        s_axil_wready,
    output logic [1:0]  s_axil_bresp,
    output logic        s_axil_bvalid,
    input  logic        s_axil_bready,
    input  logic [3:0]  s_axil_araddr,
    input  logic        s_axil_arvalid,
    output logic        s_axil_arready,
    output logic [31:0] s_axil_rdata,
    output logic [1:0]  s_axil_rresp,
    output logic        s_axil_rvalid,
    input  logic        s_axil_rready,

    // SPI physical interface
    output logic        spi_sclk,
    output logic        spi_mosi,
    input  logic        spi_miso,
    output logic        spi_cs_n,

    // Interrupt
    output logic        irq_done
);

    // -------------------------------------------------------------------------
    // Register addresses (use bits [3:2])
    // -------------------------------------------------------------------------
    localparam logic [1:0] ADDR_DATA   = 2'h0;
    localparam logic [1:0] ADDR_CTRL   = 2'h1;
    localparam logic [1:0] ADDR_STATUS = 2'h2;
    localparam logic [1:0] ADDR_CLKDIV = 2'h3;

    // -------------------------------------------------------------------------
    // Internal registers
    // -------------------------------------------------------------------------
    logic [7:0]  tx_data_reg;
    logic [7:0]  rx_data_reg;
    logic [7:0]  ctrl_reg;      // [0]=START, [1]=CPOL, [2]=CPHA, [3]=CS_AUTO
    logic [15:0] clk_div_reg;
    logic        busy_flag;
    logic        done_flag;

    wire cpol     = ctrl_reg[1];
    wire cpha     = ctrl_reg[2];
    wire cs_auto  = ctrl_reg[3];

    // -------------------------------------------------------------------------
    // Clock divider for SPI SCLK
    // -------------------------------------------------------------------------
    logic [15:0] clk_div_cnt;
    logic        sclk_toggle;   // pulses when SCLK edge should occur
    logic        sclk_phase;    // current SCLK state (before CPOL inversion)

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            clk_div_cnt  <= '0;
            sclk_toggle  <= 1'b0;
        end else begin
            sclk_toggle <= 1'b0;
            if (busy_flag) begin
                if (clk_div_cnt == clk_div_reg) begin
                    clk_div_cnt <= '0;
                    sclk_toggle <= 1'b1;
                end else begin
                    clk_div_cnt <= clk_div_cnt + 1'b1;
                end
            end else begin
                clk_div_cnt <= '0;
            end
        end
    end

    // -------------------------------------------------------------------------
    // SPI Transfer FSM
    // -------------------------------------------------------------------------
    typedef enum logic [1:0] {
        SPI_IDLE   = 2'b00,
        SPI_SETUP  = 2'b01,   // half-cycle setup before first edge (CPHA=1)
        SPI_XFER   = 2'b10,
        SPI_DONE   = 2'b11
    } spi_state_e;

    spi_state_e  spi_state;
    logic [7:0]  tx_shift;
    logic [7:0]  rx_shift;
    logic [3:0]  bit_cnt;    // counts half-periods; 8 bits × 2 edges = 16
    logic        sclk_r;     // registered SCLK

    // SCLK idle level = CPOL
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            spi_state  <= SPI_IDLE;
            tx_shift   <= '0;
            rx_shift   <= '0;
            rx_data_reg<= '0;
            bit_cnt    <= '0;
            sclk_r     <= 1'b0;
            sclk_phase <= 1'b0;
            busy_flag  <= 1'b0;
            done_flag  <= 1'b0;
            spi_cs_n   <= 1'b1;
        end else begin
            done_flag <= 1'b0;  // single-cycle pulse

            case (spi_state)
                SPI_IDLE: begin
                    sclk_phase <= cpol;       // SCLK idles at CPOL level
                    spi_cs_n   <= 1'b1;
                    busy_flag  <= 1'b0;

                    if (ctrl_reg[0]) begin    // START bit
                        tx_shift  <= tx_data_reg;
                        bit_cnt   <= '0;
                        spi_cs_n  <= cs_auto ? 1'b0 : 1'b1;
                        busy_flag <= 1'b1;
                        spi_state <= cpha ? SPI_SETUP : SPI_XFER;
                        sclk_phase<= cpol;
                    end
                end

                SPI_SETUP: begin
                    // One half-clock delay for CPHA=1 before first edge
                    if (sclk_toggle) begin
                        sclk_phase <= ~sclk_phase;
                        spi_state  <= SPI_XFER;
                    end
                end

                SPI_XFER: begin
                    if (sclk_toggle) begin
                        sclk_phase <= ~sclk_phase;

                        // Determine leading/trailing edge based on CPOL/CPHA
                        // Leading edge: sclk_phase transitions to active
                        // For Mode 0 (CPOL=0,CPHA=0): sample on rising (phase→1)
                        // For Mode 1 (CPOL=0,CPHA=1): sample on falling (phase→0)
                        // For Mode 2 (CPOL=1,CPHA=0): sample on falling (phase→0)
                        // For Mode 3 (CPOL=1,CPHA=1): sample on rising (phase→1)
                        //
                        // Sample when: (~cpol & ~cpha & rising) |
                        //              (~cpol &  cpha & falling)|
                        //              ( cpol & ~cpha & falling)|
                        //              ( cpol &  cpha & rising)
                        // Simplified: sample when sclk_phase becomes (cpol ^ cpha)
                        // Drive MOSI on opposite edge.

                        if (sclk_phase == (cpol ^ cpha)) begin
                            // Sample edge: capture MISO
                            rx_shift <= {rx_shift[6:0], spi_miso};
                        end else begin
                            // Drive edge: shift out MOSI
                            tx_shift <= {tx_shift[6:0], 1'b0};
                            bit_cnt  <= bit_cnt + 1'b1;
                        end

                        if (bit_cnt == 4'd8 && sclk_phase != (cpol ^ cpha)) begin
                            spi_state   <= SPI_DONE;
                            rx_data_reg <= rx_shift;
                            done_flag   <= 1'b1;
                        end
                    end
                end

                SPI_DONE: begin
                    busy_flag  <= 1'b0;
                    sclk_phase <= cpol;
                    if (cs_auto)
                        spi_cs_n <= 1'b1;
                    spi_state  <= SPI_IDLE;
                    // Clear START bit
                    ctrl_reg[0]<= 1'b0;
                end

                default: spi_state <= SPI_IDLE;
            endcase
        end
    end

    // Output assignments
    assign spi_sclk = sclk_phase;
    assign spi_mosi = tx_shift[7];
    assign irq_done = done_flag;

    // -------------------------------------------------------------------------
    // AXI-lite Write Channel
    // -------------------------------------------------------------------------
    assign s_axil_awready = 1'b1;
    assign s_axil_wready  = 1'b1;
    assign s_axil_bresp   = 2'b00;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx_data_reg  <= '0;
            ctrl_reg     <= 8'h08;  // CS_AUTO on by default
            clk_div_reg  <= CLK_DIV_DEFAULT[15:0];
            s_axil_bvalid<= 1'b0;
        end else begin
            if (s_axil_bvalid && s_axil_bready)
                s_axil_bvalid <= 1'b0;

            if (s_axil_awvalid && s_axil_wvalid && !busy_flag) begin
                s_axil_bvalid <= 1'b1;
                case (s_axil_awaddr[3:2])
                    ADDR_DATA:   tx_data_reg <= s_axil_wdata[7:0];
                    ADDR_CTRL:   ctrl_reg    <= s_axil_wdata[7:0];
                    ADDR_CLKDIV: clk_div_reg <= s_axil_wdata[15:0];
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

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s_axil_rvalid <= 1'b0;
            s_axil_rdata  <= '0;
        end else begin
            if (s_axil_rvalid && s_axil_rready)
                s_axil_rvalid <= 1'b0;

            if (s_axil_arvalid && s_axil_arready) begin
                s_axil_rvalid <= 1'b1;
                case (s_axil_araddr[3:2])
                    ADDR_DATA:   s_axil_rdata <= {24'h0, rx_data_reg};
                    ADDR_CTRL:   s_axil_rdata <= {24'h0, ctrl_reg};
                    ADDR_STATUS: s_axil_rdata <= {30'h0, done_flag, busy_flag};
                    ADDR_CLKDIV: s_axil_rdata <= {16'h0, clk_div_reg};
                    default:     s_axil_rdata <= 32'hDEADBEEF;
                endcase
            end
        end
    end

endmodule
// =============================================================================
// END OF FILE: spi_master.sv
// =============================================================================
