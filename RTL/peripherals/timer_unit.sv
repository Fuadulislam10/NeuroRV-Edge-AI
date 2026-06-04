// =============================================================================
// Project:     NeuroRV Edge
// Module:      timer_unit.sv
// Description: Synthesizable 32-bit Free-Running Timer / PWM Peripheral Unit
//              Compatible with Yosys, Verilator, FPGA, and ASIC target boards.
// =============================================================================

`default_nettype none

module timer_unit (
    // Global Elements
    input  wire        clk,
    input  wire        rst_n,

    // AXI4-Lite Address Register Decoders
    input  wire [11:0] s_axi_awaddr,
    input  wire        s_axi_awvalid,
    output reg         s_axi_awready,
    input  wire [31:0] s_axi_wdata,
    input  wire [3:0]  s_axi_wstrb,
    input  wire        s_axi_wvalid,
    output reg         s_axi_wready,
    output reg  [1:0]  s_axi_bresp,
    output reg         s_axi_bvalid,
    input  wire        s_axi_bready,
    input  wire [11:0] s_axi_araddr,
    input  wire        s_axi_arvalid,
    output reg         s_axi_arready,
    output reg  [31:0] s_axi_rdata,
    output reg  [1:0]  s_axi_rresp,
    output reg         s_axi_rvalid,
    input  wire        s_axi_rready,

    // Physical Peripheral Outputs
    output reg         pwm_out,

    // Interrupt Notification Line
    output reg         timer_irq
);

    // -------------------------------------------------------------------------
    // Peripheral Memory Map Configuration
    // -------------------------------------------------------------------------
    localparam bit [11:0] REG_CTRL      = 12'h000; // Counter Settings
    localparam bit [11:0] REG_COUNTER   = 12'h004; // Hardware Counter Register
    localparam bit [11:0] REG_COMPARE   = 12'h008; // Compare / PWM Duty Register
    localparam bit [11:0] REG_PRESCALER = 12'h00C; // Clock Division Metric

    // Register Blocks
    reg [31:0] ctrl_reg;
    reg [31:0] counter_reg;
    reg [31:0] compare_reg;
    reg [31:0] prescaler_reg;
    reg        irq_latch;

    // Bit Disassembly Unpacking Assignments
    wire timer_en = ctrl_reg[0];
    wire pwm_en   = ctrl_reg[1];
    wire irq_en   = ctrl_reg[2];

    // Prescaler Inner Tick Logic Counter
    reg [31:0] prescaler_counter;
    wire       prescaler_tick = (prescaler_counter == prescaler_reg);

    // -------------------------------------------------------------------------
    // Main Counter Sequential Processing Operations
    // -------------------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            counter_reg       <= 32'd0;
            prescaler_counter <= 32'd0;
            irq_latch         <= 1'b0;
            pwm_out           <= 1'b0;
        end else begin
            if (timer_en) begin
                // Prescaler Engine Execution
                if (prescaler_tick) begin
                    prescaler_counter <= 32'd0;
                    counter_reg       <= counter_reg + 32'd1;

                    // Overflow / Reset Comparison Sequence Boundary Checking
                    if (counter_reg == 32'hFFFF_FFFF) begin
                        irq_latch <= 1'b1;
                    end
                end else begin
                    prescaler_counter <= prescaler_counter + 32'd1;
                end

                // PWM Generation Strategy Block Execution
                if (pwm_en) begin
                    pwm_out <= (counter_reg < compare_reg);
                end else begin
                    pwm_out <= 1'b0;
                end

                // Basic Direct Mode Compare Value Match Check Assertion
                if (counter_reg == compare_reg && !pwm_en) begin
                    irq_latch <= 1'b1;
                end

            end else begin
                prescaler_counter <= 32'd0;
                pwm_out           <= 1'b0;
            end

            // Manual Status Overwrite Clearing Pattern
            if (s_axi_wvalid && s_axi_wready && waddr_valid && (waddr_latch[11:0] == REG_CTRL) && s_axi_wdata[3]) begin
                irq_latch <= 1'b0;
            end
        end
    end

    // Direct Asynchronous Routing For Interconnect Alert Signals
    always_ff @(posedge clk) begin
        if (!rst_n) timer_irq <= 1'b0;
        else        timer_irq <= irq_en && irq_latch;
    end

    // -------------------------------------------------------------------------
    // AXI-Lite Bus Standard Structural Handshake Realization
    // -------------------------------------------------------------------------
    reg [11:0] waddr_latch;
    reg        waddr_valid;

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            s_axi_awready <= 1'b0;
            s_axi_wready  <= 1'b0;
            s_axi_bvalid  <= 1'b0;
            s_axi_bresp   <= 2'b00;
            waddr_latch   <= 12'd0;
            waddr_valid   <= 1'b0;
            ctrl_reg      <= 32'd0;
            compare_reg   <= 32'd0;
            prescaler_reg <= 32'd0;
        end else begin
            if (s_axi_awvalid && !s_axi_awready && !waddr_valid) begin
                s_axi_awready <= 1'b1;
                waddr_latch   <= s_axi_awaddr;
                waddr_valid   <= 1'b1;
            end else begin
                s_axi_awready <= 1'b0;
            end

            if (s_axi_wvalid && !s_axi_wready && waddr_valid) begin
                s_axi_wready <= 1'b1;
                case (waddr_latch[11:0])
                    REG_CTRL: begin
                        if (s_axi_wstrb[0]) ctrl_reg[2:0]   <= s_axi_wdata[2:0]; // Filter writeable bits
                        if (s_axi_wstrb[1]) ctrl_reg[15:8]  <= s_axi_wdata[15:8];
                        if (s_axi_wstrb[2]) ctrl_reg[23:16] <= s_axi_wdata[23:16];
                        if (s_axi_wstrb[3]) ctrl_reg[31:24] <= s_axi_wdata[31:24];
                    end
                    REG_COMPARE: begin
                        if (s_axi_wstrb[0]) compare_reg[7:0]   <= s_axi_wdata[7:0];
                        if (s_axi_wstrb[1]) compare_reg[15:8]  <= s_axi_wdata[15:8];
                        if (s_axi_wstrb[2]) compare_reg[23:16] <= s_axi_wdata[23:16];
                        if (s_axi_wstrb[3]) compare_reg[31:24] <= s_axi_wdata[31:24];
                    end
                    REG_PRESCALER: begin
                        if (s_axi_wstrb[0]) prescaler_reg[7:0]   <= s_axi_wdata[7:0];
                        if (s_axi_wstrb[1]) prescaler_reg[15:8]  <= s_axi_wdata[15:8];
                        if (s_axi_wstrb[2]) prescaler_reg[23:16] <= s_axi_wdata[23:16];
                        if (s_axi_wstrb[3]) prescaler_reg[31:24] <= s_axi_wdata[31:24];
                    end
                    default: ; // Ignore writes to real-time execution registers
                endcase
            end else begin
                s_axi_wready <= 1'b0;
            end

            if (s_axi_wvalid && s_axi_wready && waddr_valid && !s_axi_bvalid) begin
                s_axi_bvalid <= 1'b1;
            end else if (s_axi_bready && s_axi_bvalid) begin
                s_axi_bvalid <= 1'b0;
                waddr_valid  <= 1'b0;
            end
        end
    end

    // AXI Target Module Memory Map Read Multiplexers
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            s_axi_arready <= 1'b0;
            s_axi_rvalid  <= 1'b0;
            s_axi_rdata   <= 32'd0;
            s_axi_rresp   <= 2'b00;
        end else begin
            if (s_axi_arvalid && !s_axi_arready && !s_axi_rvalid) begin
                s_axi_arready <= 1'b1;
                s_axi_rvalid  <= 1'b1;
                case (s_axi_araddr[11:0])
                    REG_CTRL:      s_axi_rdata <= {28'd0, irq_latch, ctrl_reg[2:0]};
                    REG_COUNTER:   s_axi_rdata <= counter_reg;
                    REG_COMPARE:   s_axi_rdata <= compare_reg;
                    REG_PRESCALER: s_axi_rdata <= prescaler_reg;
                    default:       s_axi_rdata <= 32'hDEADBEEF;
                endcase
            end else begin
                s_axi_arready <= 1'b0;
                if (s_axi_rready && s_axi_rvalid) begin
                    s_axi_rvalid <= 1'b0;
                end
            end
        end
    end

endmodule
`default_nettype wire
