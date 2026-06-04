// =============================================================================
// Project:     NeuroRV Edge
// Module:      gpio_ctrl.sv
// Description: Synthesizable 32-bit GPIO Subsystem Controller with Interconnect
//              Compatible with Yosys, Verilator, FPGA, and ASIC environments.
// =============================================================================

`default_nettype none

module gpio_ctrl (
    // Core Reference Clocks
    input  wire        clk,
    input  wire        rst_n,

    // AXI4-Lite Address Register Control Ports
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

    // Hardware Pin Configuration Arrays
    input  wire [31:0] gpio_pins_in,
    output reg  [31:0] gpio_pins_out,
    output reg  [31:0] gpio_pins_oe, // High = Output Mode, Low = Input High-Z

    // Subsystem Level Combined Interrupt Out
    output reg         gpio_irq
);

    // -------------------------------------------------------------------------
    // Register Offset Vector Structure
    // -------------------------------------------------------------------------
    localparam bit [11:0] REG_DATA_IN  = 12'h000; // Read Only Physical In
    localparam bit [11:0] REG_DATA_OUT = 12'h004; // Read/Write Safe Drivers
    localparam bit [11:0] REG_DIR      = 12'h008; // 1=Output, 0=Input Configuration
    localparam bit [11:0] REG_SET      = 12'h00C; // Bitwise Atomic Assert Mask
    localparam bit [11:0] REG_CLEAR    = 12'h010; // Bitwise Atomic De-assert Mask
    localparam bit [11:0] REG_INT_EN   = 12'h014; // Per Pin Interrupt Arming
    localparam bit [11:0] REG_INT_STAT = 12'h018; // Captured Edge Interrupt Status

    // Internal Arrays
    reg [31:0] data_out_reg;
    reg [31:0] direction_reg;
    reg [31:0] int_en_reg;
    reg [31:0] int_stat_reg;

    // Buffers for input tracking edge detection
    reg [31:0] raw_input_sync_0;
    reg [31:0] raw_input_sync_1;
    reg [31:0] previous_input_state;

    // Parallel Port Connections Linkage
    assign gpio_pins_out = data_out_reg;
    assign gpio_pins_oe  = direction_reg;

    // -------------------------------------------------------------------------
    // Meta-Stability & Edge Analysis Synchronization Processing
    // -------------------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            raw_input_sync_0     <= 32'd0;
            raw_input_sync_1     <= 32'd0;
            previous_input_state <= 32'd0;
        end else begin
            raw_input_sync_0     <= gpio_pins_in;
            raw_input_sync_1     <= raw_input_sync_0;
            previous_input_state <= raw_input_sync_1;
        end
    end

    // Interrupt Matrix Logic Calculations (Asynchronous edge assertion tracking)
    wire [31:0] pin_edges = (raw_input_sync_1 ^ previous_input_state) & raw_input_sync_1; // Rising Edge Check

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            int_stat_reg <= 32'd0;
            gpio_irq     <= 1'b0;
        end else begin
            // Track edges on pins enabled for interrupts
            int_stat_reg <= (int_stat_reg | (pin_edges & int_en_reg));

            // Combined interrupt output generation
            gpio_irq <= |(int_stat_reg & int_en_reg);
        end
    end

    // -------------------------------------------------------------------------
    // AXI Bus Interconnect Interface Subroutines
    // -------------------------------------------------------------------------
    reg [11:0] active_waddr;
    reg        waddr_asserted;

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            s_axi_awready   <= 1'b0;
            s_axi_wready    <= 1'b0;
            s_axi_bvalid    <= 1'b0;
            s_axi_bresp     <= 2'b00;
            active_waddr    <= 12'd0;
            waddr_asserted  <= 1'b0;
            data_out_reg    <= 32'd0;
            direction_reg   <= 32'd0;
            int_en_reg      <= 32'd0;
        end else begin
            if (s_axi_awvalid && !s_axi_awready && !waddr_asserted) begin
                s_axi_awready  <= 1'b1;
                active_waddr   <= s_axi_awaddr;
                waddr_asserted <= 1'b1;
            end else begin
                s_axi_awready <= 1'b0;
            end

            if (s_axi_wvalid && !s_axi_wready && waddr_asserted) begin
                s_axi_wready <= 1'b1;
                case (active_waddr[11:0])
                    REG_DATA_OUT: begin
                        if (s_axi_wstrb[0]) data_out_reg[7:0]   <= s_axi_wdata[7:0];
                        if (s_axi_wstrb[1]) data_out_reg[15:8]  <= s_axi_wdata[15:8];
                        if (s_axi_wstrb[2]) data_out_reg[23:16] <= s_axi_wdata[23:16];
                        if (s_axi_wstrb[3]) data_out_reg[31:24] <= s_axi_wdata[31:24];
                    end
                    REG_DIR: begin
                        if (s_axi_wstrb[0]) direction_reg[7:0]   <= s_axi_wdata[7:0];
                        if (s_axi_wstrb[1]) direction_reg[15:8]  <= s_axi_wdata[15:8];
                        if (s_axi_wstrb[2]) direction_reg[23:16] <= s_axi_wdata[23:16];
                        if (s_axi_wstrb[3]) direction_reg[31:24] <= s_axi_wdata[31:24];
                    end
                    REG_SET: begin
                        if (s_axi_wstrb[0]) data_out_reg[7:0]   <= data_out_reg[7:0]   | s_axi_wdata[7:0];
                        if (s_axi_wstrb[1]) data_out_reg[15:8]  <= data_out_reg[15:8]  | s_axi_wdata[15:8];
                        if (s_axi_wstrb[2]) data_out_reg[23:16] <= data_out_reg[23:16] | s_axi_wdata[23:16];
                        if (s_axi_wstrb[3]) data_out_reg[31:24] <= data_out_reg[31:24] | s_axi_wdata[31:24];
                    end
                    REG_CLEAR: begin
                        if (s_axi_wstrb[0]) data_out_reg[7:0]   <= data_out_reg[7:0]   & ~s_axi_wdata[7:0];
                        if (s_axi_wstrb[1]) data_out_reg[15:8]  <= data_out_reg[15:8]  & ~s_axi_wdata[15:8];
                        if (s_axi_wstrb[2]) data_out_reg[23:16] <= data_out_reg[23:16] & ~s_axi_wdata[23:16];
                        if (s_axi_wstrb[3]) data_out_reg[31:24] <= data_out_reg[31:24] & ~s_axi_wdata[31:24];
                    end
                    REG_INT_EN: begin
                        if (s_axi_wstrb[0]) int_en_reg[7:0]   <= s_axi_wdata[7:0];
                        if (s_axi_wstrb[1]) int_en_reg[15:8]  <= s_axi_wdata[15:8];
                        if (s_axi_wstrb[2]) int_en_reg[23:16] <= s_axi_wdata[23:16];
                        if (s_axi_wstrb[3]) int_en_reg[31:24] <= s_axi_wdata[31:24];
                    end
                    REG_INT_STAT: begin
                        // Write 1 to clear individual edge status bits
                        if (s_axi_wstrb[0]) int_stat_reg[7:0]   <= int_stat_reg[7:0]   & ~s_axi_wdata[7:0];
                        if (s_axi_wstrb[1]) int_stat_reg[15:8]  <= int_stat_reg[15:8]  & ~s_axi_wdata[15:8];
                        if (s_axi_wstrb[2]) int_stat_reg[23:16] <= int_stat_reg[23:16] & ~s_axi_wdata[23:16];
                        if (s_axi_wstrb[3]) int_stat_reg[31:24] <= int_stat_reg[31:24] & ~s_axi_wdata[31:24];
                    end
                    default: ;
                endcase
            end else begin
                s_axi_wready <= 1'b0;
            end

            if (s_axi_wvalid && s_axi_wready && waddr_asserted && !s_axi_bvalid) begin
                s_axi_bvalid <= 1'b1;
            end else if (s_axi_bready && s_axi_bvalid) begin
                s_axi_bvalid   <= 1'b0;
                waddr_asserted <= 1'b0;
            end
        end
    end

    // AXI Synchronous Read Routing Logic Configuration
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
                    REG_DATA_IN:  s_axi_rdata <= raw_input_sync_1;
                    REG_DATA_OUT: s_axi_rdata <= data_out_reg;
                    REG_DIR:      s_axi_rdata <= direction_reg;
                    REG_INT_EN:   s_axi_rdata <= int_en_reg;
                    REG_INT_STAT: s_axi_rdata <= int_stat_reg;
                    default:      s_axi_rdata <= 32'hDEADBEEF;
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
