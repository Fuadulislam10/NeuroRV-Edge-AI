// =============================================================================
// FILE: rtl/peripherals/i2c_master.sv
// PROJECT: NeuroRV Edge — Phase 5 Peripheral Subsystem
// MODULE: i2c_master
// DESCRIPTION: I2C master controller with START/STOP generation, ACK/NACK
//              handling, 7-bit addressing, and AXI-lite register interface.
//
// Register Map (32-bit aligned, byte offset):
//   0x00  ADDR_REG  [7:0]  [7:1]=7-bit slave addr, [0]=R/W# (0=write, 1=read)
//   0x04  DATA_REG  [7:0]  Write = TX byte, Read = last received byte
//   0x08  CTRL_REG  [2:0]  [0]=START, [1]=STOP, [2]=ACK_EN (send ACK on read)
//   0x0C  STATUS_REG[3:0]  [0]=BUSY, [1]=ACK_RX, [2]=ARB_LOST, [3]=DONE
//   (SCL prescaler is derived from CLK_DIV parameter)
//
// I2C transaction sequence (firmware driven):
//   1. Write ADDR_REG with {slave_addr, rw}
//   2. Write DATA_REG with payload byte (if write transaction)
//   3. Write CTRL_REG[0]=1 to initiate transfer
//   4. Poll STATUS_REG[3] (DONE) or wait for IRQ
//   5. Read DATA_REG for received byte (read transactions)
//
// Synchronous reset, open-drain SCL/SDA modeled with tri-state outputs.
// =============================================================================

`timescale 1ns/1ps

module i2c_master #(
    parameter int CLK_FREQ_HZ  = 50_000_000,
    parameter int I2C_FREQ_HZ  = 100_000,       // Standard mode 100 kHz
    parameter int SCL_DIV      = CLK_FREQ_HZ / (I2C_FREQ_HZ * 4)  // quarter-period
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

    // I2C open-drain interface (1 = released/high, 0 = driven low)
    output logic        scl_oe,   // 1 = drive SCL low
    input  logic        scl_in,
    output logic        sda_oe,   // 1 = drive SDA low
    input  logic        sda_in,

    // Interrupt
    output logic        irq_done,
    output logic        irq_arb_lost
);

    // -------------------------------------------------------------------------
    // Register addresses
    // -------------------------------------------------------------------------
    localparam logic [1:0] ADDR_SLAVE  = 2'h0;
    localparam logic [1:0] ADDR_DATA   = 2'h1;
    localparam logic [1:0] ADDR_CTRL   = 2'h2;
    localparam logic [1:0] ADDR_STATUS = 2'h3;

    // -------------------------------------------------------------------------
    // Internal registers
    // -------------------------------------------------------------------------
    logic [7:0] slave_addr_reg;   // [7:1]=addr, [0]=R/W#
    logic [7:0] tx_data_reg;
    logic [7:0] rx_data_reg;
    logic [2:0] ctrl_reg;

    logic busy_flag;
    logic ack_rx_flag;
    logic arb_lost_flag;
    logic done_flag;

    // -------------------------------------------------------------------------
    // SCL clock generator (quarter-period ticks for phase control)
    // -------------------------------------------------------------------------
    // Quarter-period states: 0=SCL_LO_FIRST, 1=SCL_RISE, 2=SCL_HI, 3=SCL_FALL
    logic [$clog2(SCL_DIV)-1:0] clk_cnt;
    logic [1:0]                 scl_phase;   // 0..3
    logic                       scl_tick;    // one pulse per quarter period

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            clk_cnt   <= '0;
            scl_phase <= 2'd0;
            scl_tick  <= 1'b0;
        end else begin
            scl_tick <= 1'b0;
            if (busy_flag) begin
                if (clk_cnt == SCL_DIV[$clog2(SCL_DIV)-1:0] - 1) begin
                    clk_cnt   <= '0;
                    scl_phase <= scl_phase + 1'b1;
                    scl_tick  <= 1'b1;
                end else begin
                    clk_cnt <= clk_cnt + 1'b1;
                end
            end else begin
                clk_cnt   <= '0;
                scl_phase <= 2'd0;
            end
        end
    end

    // -------------------------------------------------------------------------
    // I2C Master FSM
    // -------------------------------------------------------------------------
    typedef enum logic [3:0] {
        I2C_IDLE      = 4'h0,
        I2C_START     = 4'h1,   // SDA falls while SCL high
        I2C_ADDR      = 4'h2,   // Transmit 7-bit address + R/W
        I2C_ADDR_ACK  = 4'h3,   // Receive ACK after address
        I2C_WRITE     = 4'h4,   // Transmit data byte
        I2C_WRITE_ACK = 4'h5,   // Receive ACK after data write
        I2C_READ      = 4'h6,   // Receive data byte
        I2C_READ_ACK  = 4'h7,   // Send ACK/NACK after data read
        I2C_STOP      = 4'h8,   // SDA rises while SCL high
        I2C_DONE      = 4'h9
    } i2c_state_e;

    i2c_state_e  i2c_state;
    logic [7:0]  shift_reg;     // TX/RX shift register
    logic [2:0]  bit_idx;       // current bit being transferred (7 downto 0)
    logic        scl_out;       // controlled SCL level
    logic        sda_out;       // controlled SDA level
    logic        rw_flag;       // latched R/W bit

    // Map phase to SCL level: phase 0,1 = low; 2,3 = ... but transitions:
    // phase 0 → SCL low  (hold)
    // phase 1 → SCL rises (tick triggers rise)
    // phase 2 → SCL high (hold, sample SDA here)
    // phase 3 → SCL falls (tick triggers fall)

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            i2c_state   <= I2C_IDLE;
            shift_reg   <= '0;
            bit_idx     <= 3'd7;
            scl_out     <= 1'b1;  // released (high)
            sda_out     <= 1'b1;  // released (high)
            busy_flag   <= 1'b0;
            ack_rx_flag <= 1'b0;
            arb_lost_flag<= 1'b0;
            done_flag   <= 1'b0;
            rx_data_reg <= '0;
            rw_flag     <= 1'b0;
        end else begin
            done_flag    <= 1'b0;  // default: single-cycle

            case (i2c_state)
                I2C_IDLE: begin
                    scl_out   <= 1'b1;
                    sda_out   <= 1'b1;
                    busy_flag <= 1'b0;

                    if (ctrl_reg[0]) begin  // START request
                        rw_flag   <= slave_addr_reg[0];
                        shift_reg <= slave_addr_reg;
                        bit_idx   <= 3'd7;
                        busy_flag <= 1'b1;
                        i2c_state <= I2C_START;
                        ctrl_reg[0] <= 1'b0;  // clear START
                    end
                end

                // --- START condition: SDA low while SCL high ---
                I2C_START: begin
                    if (scl_tick) begin
                        case (scl_phase)
                            2'd0: sda_out <= 1'b1;   // ensure SDA high before
                            2'd1: begin               // SCL high
                                    scl_out <= 1'b1;
                                    sda_out <= 1'b1;
                                  end
                            2'd2: sda_out <= 1'b0;   // SDA falls → START
                            2'd3: begin               // SCL low → data phase
                                    scl_out   <= 1'b0;
                                    i2c_state <= I2C_ADDR;
                                    bit_idx   <= 3'd7;
                                    shift_reg <= slave_addr_reg;
                                  end
                            default: ;
                        endcase
                    end
                end

                // --- Address + R/W# bits ---
                I2C_ADDR: begin
                    if (scl_tick) begin
                        case (scl_phase)
                            2'd0: sda_out <= shift_reg[bit_idx]; // setup SDA
                            2'd1: scl_out <= 1'b1;               // SCL rise
                            2'd2: begin                           // SCL high hold
                                    // Check arbitration
                                    if (!sda_in && shift_reg[bit_idx]) begin
                                        arb_lost_flag <= 1'b1;
                                        i2c_state     <= I2C_IDLE;
                                    end
                                  end
                            2'd3: begin                           // SCL fall
                                    scl_out <= 1'b0;
                                    if (bit_idx == 3'd0) begin
                                        i2c_state <= I2C_ADDR_ACK;
                                    end else begin
                                        bit_idx <= bit_idx - 1'b1;
                                    end
                                  end
                            default: ;
                        endcase
                    end
                end

                // --- ACK after address ---
                I2C_ADDR_ACK: begin
                    if (scl_tick) begin
                        case (scl_phase)
                            2'd0: sda_out <= 1'b1;   // release SDA for slave ACK
                            2'd1: scl_out <= 1'b1;
                            2'd2: ack_rx_flag <= !sda_in;  // sample ACK (low=ACK)
                            2'd3: begin
                                    scl_out <= 1'b0;
                                    bit_idx <= 3'd7;
                                    if (!ack_rx_flag) begin
                                        // NACK received → go to STOP
                                        i2c_state <= I2C_STOP;
                                    end else begin
                                        i2c_state <= rw_flag ? I2C_READ : I2C_WRITE;
                                        shift_reg <= tx_data_reg;
                                    end
                                  end
                            default: ;
                        endcase
                    end
                end

                // --- Write data byte ---
                I2C_WRITE: begin
                    if (scl_tick) begin
                        case (scl_phase)
                            2'd0: sda_out <= shift_reg[bit_idx];
                            2'd1: scl_out <= 1'b1;
                            2'd2: ; // hold
                            2'd3: begin
                                    scl_out <= 1'b0;
                                    if (bit_idx == 3'd0) begin
                                        i2c_state <= I2C_WRITE_ACK;
                                    end else begin
                                        bit_idx <= bit_idx - 1'b1;
                                    end
                                  end
                            default: ;
                        endcase
                    end
                end

                // --- ACK after write ---
                I2C_WRITE_ACK: begin
                    if (scl_tick) begin
                        case (scl_phase)
                            2'd0: sda_out <= 1'b1;
                            2'd1: scl_out <= 1'b1;
                            2'd2: ack_rx_flag <= !sda_in;
                            2'd3: begin
                                    scl_out   <= 1'b0;
                                    i2c_state <= (ctrl_reg[1]) ? I2C_STOP : I2C_DONE;
                                  end
                            default: ;
                        endcase
                    end
                end

                // --- Read data byte ---
                I2C_READ: begin
                    if (scl_tick) begin
                        case (scl_phase)
                            2'd0: sda_out <= 1'b1;   // release SDA
                            2'd1: scl_out <= 1'b1;
                            2'd2: shift_reg[bit_idx] <= sda_in;  // sample
                            2'd3: begin
                                    scl_out <= 1'b0;
                                    if (bit_idx == 3'd0) begin
                                        rx_data_reg <= shift_reg;
                                        i2c_state   <= I2C_READ_ACK;
                                    end else begin
                                        bit_idx <= bit_idx - 1'b1;
                                    end
                                  end
                            default: ;
                        endcase
                    end
                end

                // --- Send ACK or NACK after read ---
                I2C_READ_ACK: begin
                    if (scl_tick) begin
                        case (scl_phase)
                            // ctrl_reg[2]=ACK_EN: 0=NACK (release SDA), 1=ACK (pull low)
                            2'd0: sda_out <= !ctrl_reg[2];
                            2'd1: scl_out <= 1'b1;
                            2'd2: ; // hold
                            2'd3: begin
                                    scl_out   <= 1'b0;
                                    i2c_state <= (ctrl_reg[1]) ? I2C_STOP : I2C_DONE;
                                  end
                            default: ;
                        endcase
                    end
                end

                // --- STOP condition: SDA rises while SCL high ---
                I2C_STOP: begin
                    if (scl_tick) begin
                        case (scl_phase)
                            2'd0: sda_out <= 1'b0;   // ensure SDA low
                            2'd1: scl_out <= 1'b1;   // SCL rises
                            2'd2: sda_out <= 1'b1;   // SDA rises → STOP
                            2'd3: i2c_state <= I2C_DONE;
                            default: ;
                        endcase
                    end
                end

                I2C_DONE: begin
                    done_flag    <= 1'b1;
                    busy_flag    <= 1'b0;
                    ctrl_reg[1]  <= 1'b0;  // clear STOP
                    i2c_state    <= I2C_IDLE;
                end

                default: i2c_state <= I2C_IDLE;
            endcase
        end
    end

    // Open-drain drive: oe=1 means pull line low
    assign scl_oe = !scl_out;
    assign sda_oe = !sda_out;

    assign irq_done     = done_flag;
    assign irq_arb_lost = arb_lost_flag;

    // -------------------------------------------------------------------------
    // AXI-lite Write Channel
    // -------------------------------------------------------------------------
    assign s_axil_awready = 1'b1;
    assign s_axil_wready  = 1'b1;
    assign s_axil_bresp   = 2'b00;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            slave_addr_reg <= '0;
            tx_data_reg    <= '0;
            ctrl_reg       <= '0;
            s_axil_bvalid  <= 1'b0;
        end else begin
            if (s_axil_bvalid && s_axil_bready)
                s_axil_bvalid <= 1'b0;

            if (s_axil_awvalid && s_axil_wvalid && !busy_flag) begin
                s_axil_bvalid <= 1'b1;
                case (s_axil_awaddr[3:2])
                    ADDR_SLAVE:  slave_addr_reg <= s_axil_wdata[7:0];
                    ADDR_DATA:   tx_data_reg    <= s_axil_wdata[7:0];
                    ADDR_CTRL:   ctrl_reg       <= s_axil_wdata[2:0];
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
                    ADDR_SLAVE:  s_axil_rdata <= {24'h0, slave_addr_reg};
                    ADDR_DATA:   s_axil_rdata <= {24'h0, rx_data_reg};
                    ADDR_CTRL:   s_axil_rdata <= {29'h0, ctrl_reg};
                    ADDR_STATUS: s_axil_rdata <= {28'h0, done_flag, arb_lost_flag,
                                                          ack_rx_flag, busy_flag};
                    default:     s_axil_rdata <= 32'hDEADBEEF;
                endcase
            end
        end
    end

endmodule
// =============================================================================
// END OF FILE: i2c_master.sv
// =============================================================================
