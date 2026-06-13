// =============================================================================
// Project:     NeuroRV Edge
// Module:      fpga_top
// Description: Top-level wrapper for the Hybrid RISC-V AI Accelerator SoC.
//              Handles clock management, reset synchronization, and physical 
//              I/O mapping for FPGA deployment (Xilinx Artix-7/Nexys A7 baseline).
// =============================================================================

`timescale 1ns / 1ps

module fpga_top (
    // Clock and Reset
    input  logic        clk_100mhz,     // Physical 100MHz oscillator input
    input  logic        btn_reset_n,    // Active-low CPU reset button
    
    // UART Interface
    input  logic        uart_rx,        // FPGA RX line
    output logic        uart_tx,        // FPGA TX line
    
    // General Purpose I/O
    input  logic [7:0]  switches,       // Hardware slide switches
    output logic [7:0]  leds,           // Hardware LEDs
    
    // Debug & Status Lines
    output logic        clk_locked_led, // High when MMCM/PLL is locked
    output logic        sys_reset_led   // High when system is actively in reset
);

    // -------------------------------------------------------------------------
    // Internal Signals
    // -------------------------------------------------------------------------
    logic sys_clk;
    logic mmcm_locked;
    logic raw_reset_n;
    
    // Synchronized reset registers (Active-high for internal SoC operations)
    logic sys_reset_reg_p1;
    logic sys_reset;

    // -------------------------------------------------------------------------
    // Clocking Wizard Instance (MMCM / PLL)
    // -------------------------------------------------------------------------
    // Generates a stable 50MHz internal system clock from 100MHz input.
    // Lowers power consumption and eases timing closure for the fabric.
    clk_wiz_0 clk_gen (
        .clk_in1   (clk_100mhz),
        .clk_out1  (sys_clk),
        .resetn    (btn_reset_n),
        .locked    (mmcm_locked)
    );

    assign clk_locked_led = mmcm_locked;

    // -------------------------------------------------------------------------
    // Reset Synchronizer
    // -------------------------------------------------------------------------
    // Prevents metastability by launching reset synchronously to sys_clk.
    // Deasserts synchronously, asserts asynchronously.
    assign raw_reset_n = btn_reset_n & mmcm_locked;

    always_ff @(posedge sys_clk or negedge raw_reset_n) begin
        if (!raw_reset_n) begin
            sys_reset_reg_p1 <= 1'b1;
            sys_reset        <= 1'b1;
        end else begin
            sys_reset_reg_p1 <= 1'b0;
            sys_reset        <= sys_reset_reg_p1;
        end
    end

    assign sys_reset_led = sys_reset;

    // -------------------------------------------------------------------------
    // NeuroRV Edge SoC Core Instance
    // -------------------------------------------------------------------------
    neurorv_soc #(
        .CLK_FREQ_HZ (50_000_000), // 50 MHz
        .MEM_SIZE_BYTES(65536)     // 64 KB Internal BRAM Boot RAM
    ) u_neurorv_soc (
        .clk         (sys_clk),
        .rst         (sys_reset),
        
        // Peripherals
        .uart_rxd    (uart_rx),
        .uart_txd    (uart_tx),
        
        .gpio_in     (switches),
        .gpio_out    (leds)
    );

endmodule
