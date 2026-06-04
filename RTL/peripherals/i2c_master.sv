// =============================================================================
// FILE: rtl/peripherals/i2c_master.sv
// PROJECT: NeuroRV Edge SoC
// MODULE: i2c_master
// DESCRIPTION: I2C Master controller. Supports START/STOP generation,
//              ACK/NACK handling, 7-bit slave addressing, configurable SCL
//              clock. Uses open-drain modelling (drives low or tri-states).
//              AXI-lite register interface for CPU programming.
// SYNTHESIZABLE: Yes (Yosys + Verilator compatible)
// =============================================================================

`timescale 1ns/1ps

module i2c_master #(
    parameter int unsigned ADDR_BITS = 3,
    parameter int unsigned CLK_FREQ_HZ  = 50_000_000,
    parameter int unsigned DEFAULT_SCL_HZ = 100_000    // 100kHz standard mode
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

    // I2C open-drain pins (active-low drive)
    output logic        i2c_scl_oe,    // 1 = drive SCL low
    input  logic        i2c_scl_in,    // SCL line read-back
    output logic        i2c_sda_oe,    // 1 = drive SDA low
    input  logic        i2c_sda_in,    // SDA line read-back

    // Interrupt
    output logic        i2c_irq
);

    // =========================================================================
    // Register Map
    // 0x0 = CTRL     (RW): [0]=start, [1]=stop, [2]=read_mode, [3]=irq_en,
    //                       [4]=ack_en (send ACK after receive)
    // 0x1 = STATUS   (RO): [0]=busy, [1]=done, [2]=ack_rxd (0=ACK,1=NACK),
    //                       [3]=arb_lost, [4]=rx_valid
    // 0x2 = ADDR_REG (RW): [7:1]=slave 7-bit addr, [0]=rw (1=read)
    // 0x3 = TX_DATA  (WO): byte to transmit (triggers write byte operation)
    // 0x4 = RX_DATA  (RO): received byte
    // 0x5 = CLK_DIV  (RW): SCL period = 4 * (clk_div+1) clocks
    // 0x6 = IRQ_CLR  (WO): write any to clear irq
    // 0x7 = DBG_CNT  (RO): byte transfer count
    // =========================================================================
    localparam logic [ADDR_BITS-1:0]
        REG_CTRL     = 3'h0,
        REG_STATUS   = 3'h1,
        REG_ADDR_REG = 3'h2,
        REG_TX_DATA  = 3'h3,
        REG_RX_DATA  = 3'h4,
        REG_CLK_DIV  = 3'h5,
        REG_IRQ_CLR  = 3'h6,
        REG_DBG_CNT  = 3'h7;

    localparam int unsigned DEFAULT_DIV = (CLK_FREQ_HZ / (DEFAULT_SCL_HZ * 4)) - 1;

    // =========================================================================
    // Config Registers
    // =========================================================================
    logic        cfg_irq_en;
    logic        cfg_ack_en;      // send ACK after byte receive
    logic [15:0] cfg_clk_div;
    logic [7:0]  cfg_slave_addr;  // [7:1]=addr, [0]=R/W

    // =========================================================================
    // I2C FSM
    // =========================================================================
    // SCL timing: 4 phases per bit: PHASE_LOW, PHASE_RISE, PHASE_HIGH, PHASE_FALL
    typedef enum logic [4:0] {
        I2C_IDLE        = 5'h00,
        I2C_START_1     = 5'h01,   // SDA goes low while SCL high
        I2C_START_2     = 5'h02,   // SCL goes low
        I2C_ADDR_LOAD   = 5'h03,
        I2C_SHIFT_BIT_0 = 5'h04,   // SCL low, set SDA
        I2C_SHIFT_BIT_1 = 5'h05,   // SCL rising
        I2C_SHIFT_BIT_2 = 5'h06,   // SCL high (sample on read)
        I2C_SHIFT_BIT_3 = 5'h07,   // SCL falling
        I2C_ACK_0       = 5'h08,   // ACK bit: SCL low, release SDA
        I2C_ACK_1       = 5'h09,
        I2C_ACK_2       = 5'h0A,   // Sample ACK from slave
        I2C_ACK_3       = 5'h0B,
        I2C_RDATA_LOAD  = 5'h0C,
        I2C_STOP_0      = 5'h0D,   // SCL goes high
        I2C_STOP_1      = 5'h0E,   // SDA goes high (STOP condition)
        I2C_DONE        = 5'h0F,
        I2C_ARB_LOST    = 5'h10
    } i2c_state_t;

    i2c_state_t  i2c_state;
    logic [7:0]  shift_reg;     // current byte being sent/received
    logic [2:0]  bit_cnt;       // current bit (0..7)
    logic [15:0] phase_cnt;     // counts within one SCL phase
    logic        phase_tick;    // end of current phase
    logic        busy;
    logic        done_pulse;
    logic        ack_rxd;       // 0=ACK received, 1=NACK
    logic        arb_lost;
    logic        read_mode;     // 1 = reading from slave
    logic        send_addr;     // 1 = currently sending address byte
    logic [7:0]  rx_data_reg;
    logic        rx_valid;
    logic        irq_sticky;
    logic [31:0] dbg_byte_cnt;

    // Phase counter
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            phase_cnt  <= 16'd0;
            phase_tick <= 1'b0;
        end else begin
            phase_tick <= 1'b0;
            if (phase_cnt == 16'd0) begin
                phase_cnt  <= cfg_clk_div;
                phase_tick <= 1'b1;
            end else if (i2c_state != I2C_IDLE) begin
                phase_cnt <= phase_cnt - 1;
            end
        end
    end

    // =========================================================================
    // Main I2C FSM
    // =========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            i2c_state   <= I2C_IDLE;
            shift_reg   <= 8'h00;
            bit_cnt     <= 3'd0;
            i2c_scl_oe  <= 1'b0;
            i2c_sda_oe  <= 1'b0;
            busy        <= 1'b0;
            done_pulse  <= 1'b0;
            ack_rxd     <= 1'b0;
            arb_lost    <= 1'b0;
            read_mode   <= 1'b0;
            send_addr   <= 1'b0;
            rx_data_reg <= 8'h00;
            rx_valid    <= 1'b0;
            irq_sticky  <= 1'b0;
            dbg_byte_cnt<= 32'd0;
        end else begin
            done_pulse <= 1'b0;

            case (i2c_state)
                //--------------------------------------------------------------
                I2C_IDLE: begin
                    i2c_scl_oe <= 1'b0;   // release SCL
                    i2c_sda_oe <= 1'b0;   // release SDA
                    busy       <= 1'b0;
                    arb_lost   <= 1'b0;
                end

                //--------------------------------------------------------------
                // START condition: SDA falls while SCL is high
                I2C_START_1: begin
                    busy       <= 1'b1;
                    i2c_sda_oe <= 1'b1;   // pull SDA low
                    if (phase_tick) i2c_state <= I2C_START_2;
                end

                I2C_START_2: begin
                    i2c_scl_oe <= 1'b1;   // pull SCL low
                    if (phase_tick) begin
                        i2c_state <= I2C_ADDR_LOAD;
                    end
                end

                //--------------------------------------------------------------
                I2C_ADDR_LOAD: begin
                    shift_reg  <= cfg_slave_addr;
                    bit_cnt    <= 3'd0;
                    send_addr  <= 1'b1;
                    read_mode  <= cfg_slave_addr[0];
                    i2c_state  <= I2C_SHIFT_BIT_0;
                end

                //--------------------------------------------------------------
                // Bit transmit: 4-phase per bit
                I2C_SHIFT_BIT_0: begin
                    // SCL low, set SDA
                    i2c_scl_oe <= 1'b1;
                    i2c_sda_oe <= ~shift_reg[7];   // 0 = drive low, release for 1
                    if (phase_tick) i2c_state <= I2C_SHIFT_BIT_1;
                end

                I2C_SHIFT_BIT_1: begin
                    // SCL rising
                    i2c_scl_oe <= 1'b0;   // release SCL
                    if (phase_tick) begin
                        // Arbitration check: if SDA should be high but is low
                        if (!i2c_sda_oe && !i2c_sda_in) begin
                            i2c_state <= I2C_ARB_LOST;
                        end else begin
                            i2c_state <= I2C_SHIFT_BIT_2;
                        end
                    end
                end

                I2C_SHIFT_BIT_2: begin
                    // SCL high — for read mode, sample SDA here
                    if (read_mode && !send_addr) begin
                        shift_reg <= {shift_reg[6:0], i2c_sda_in};
                    end
                    if (phase_tick) i2c_state <= I2C_SHIFT_BIT_3;
                end

                I2C_SHIFT_BIT_3: begin
                    // SCL falling
                    i2c_scl_oe <= 1'b1;
                    if (phase_tick) begin
                        if (bit_cnt == 3'd7) begin
                            i2c_state <= I2C_ACK_0;
                            shift_reg <= {shift_reg[6:0], 1'b0}; // shift for next
                        end else begin
                            bit_cnt   <= bit_cnt + 1;
                            shift_reg <= {shift_reg[6:0], 1'b0};
                            i2c_state <= I2C_SHIFT_BIT_0;
                        end
                    end
                end

                //--------------------------------------------------------------
                // ACK / NACK phase
                I2C_ACK_0: begin
                    // Release SDA (slave drives ACK=low, or stays high for NACK)
                    i2c_scl_oe <= 1'b1;
                    if (read_mode && !send_addr) begin
                        // We're master-reading: we drive ACK or NACK
                        i2c_sda_oe <= ~cfg_ack_en;  // 0=ACK (pull low), 1=NACK (release)
                    end else begin
                        i2c_sda_oe <= 1'b0;   // release: slave drives
                    end
                    if (phase_tick) i2c_state <= I2C_ACK_1;
                end

                I2C_ACK_1: begin
                    i2c_scl_oe <= 1'b0;   // SCL rising
                    if (phase_tick) i2c_state <= I2C_ACK_2;
                end

                I2C_ACK_2: begin
                    // Sample ACK from slave (for write or address phase)
                    if (!read_mode || send_addr) begin
                        ack_rxd <= i2c_sda_in;   // 0=ACK, 1=NACK
                    end
                    if (phase_tick) i2c_state <= I2C_ACK_3;
                end

                I2C_ACK_3: begin
                    i2c_scl_oe <= 1'b1;   // SCL falling
                    if (phase_tick) begin
                        dbg_byte_cnt <= dbg_byte_cnt + 1;
                        if (send_addr) begin
                            send_addr  <= 1'b0;
                            bit_cnt    <= 3'd0;
                            if (read_mode) begin
                                // Prepare to receive data byte
                                shift_reg <= 8'h00;
                                i2c_state <= I2C_RDATA_LOAD;
                            end else begin
                                i2c_state <= I2C_DONE;   // addr sent, wait for TX_DATA
                            end
                        end else begin
                            if (read_mode) begin
                                // Store received byte
                                rx_data_reg <= shift_reg;
                                rx_valid    <= 1'b1;
                            end
                            i2c_state <= I2C_DONE;
                        end
                    end
                end

                //--------------------------------------------------------------
                I2C_RDATA_LOAD: begin
                    shift_reg <= 8'hFF;   // all ones to release SDA during read
                    bit_cnt   <= 3'd0;
                    i2c_state <= I2C_SHIFT_BIT_0;
                end

                //--------------------------------------------------------------
                // STOP condition: SDA rises while SCL is high
                I2C_STOP_0: begin
                    i2c_scl_oe <= 1'b0;   // SCL high
                    i2c_sda_oe <= 1'b1;   // SDA still low
                    if (phase_tick) i2c_state <= I2C_STOP_1;
                end

                I2C_STOP_1: begin
                    i2c_sda_oe <= 1'b0;   // release SDA: SDA rises → STOP
                    if (phase_tick) begin
                        i2c_state  <= I2C_DONE;
                    end
                end

                //--------------------------------------------------------------
                I2C_DONE: begin
                    done_pulse <= 1'b1;
                    busy       <= 1'b0;
                    irq_sticky <= cfg_irq_en;
                    i2c_state  <= I2C_IDLE;
                end

                I2C_ARB_LOST: begin
                    arb_lost   <= 1'b1;
                    busy       <= 1'b0;
                    i2c_scl_oe <= 1'b0;
                    i2c_sda_oe <= 1'b0;
                    i2c_state  <= I2C_IDLE;
                end

                default: i2c_state <= I2C_IDLE;
            endcase

            // IRQ clear
            if (reg_req && reg_we && reg_addr == REG_IRQ_CLR) begin
                irq_sticky <= 1'b0;
                rx_valid   <= 1'b0;
            end
        end
    end

    assign i2c_irq = irq_sticky;

    // =========================================================================
    // Register Write
    // =========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cfg_irq_en    <= 1'b0;
            cfg_ack_en    <= 1'b1;
            cfg_clk_div   <= DEFAULT_DIV[15:0];
            cfg_slave_addr<= 8'h00;
        end else begin
            if (reg_req && reg_we) begin
                case (reg_addr)
                    REG_CTRL: begin
                        // [0]=start, [1]=stop handled below as pulses
                        cfg_irq_en <= reg_wdata[3];
                        cfg_ack_en <= reg_wdata[4];
                        if (reg_wdata[0] && !busy) begin
                            i2c_state <= I2C_START_1;
                        end
                        if (reg_wdata[1] && busy) begin
                            i2c_state <= I2C_STOP_0;
                        end
                    end
                    REG_ADDR_REG: cfg_slave_addr <= reg_wdata[7:0];
                    REG_TX_DATA: begin
                        if (!busy) begin
                            shift_reg <= reg_wdata[7:0];
                            bit_cnt   <= 3'd0;
                            send_addr <= 1'b0;
                            busy      <= 1'b1;
                            i2c_state <= I2C_SHIFT_BIT_0;
                        end
                    end
                    REG_CLK_DIV: cfg_clk_div <= reg_wdata[15:0];
                    default: ;
                endcase
            end
        end
    end

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
                    REG_CTRL:     reg_rdata <= {27'd0, cfg_ack_en, cfg_irq_en,
                                                1'b0, 1'b0, busy};
                    REG_STATUS:   reg_rdata <= {27'd0, rx_valid, arb_lost,
                                                ack_rxd, done_pulse, busy};
                    REG_ADDR_REG: reg_rdata <= {24'd0, cfg_slave_addr};
                    REG_RX_DATA:  reg_rdata <= {24'd0, rx_data_reg};
                    REG_CLK_DIV:  reg_rdata <= {16'd0, cfg_clk_div};
                    REG_DBG_CNT:  reg_rdata <= dbg_byte_cnt;
                    default:      reg_rdata <= 32'd0;
                endcase
            end
        end
    end

endmodule
// =============================================================================
// END: i2c_master.sv
// =============================================================================
