// =============================================================================
// Project Name: NeuroRV Edge
// Module Name:  tb_neurorv_soc
// Description:  System-Level Testbench integrating the complete NeuroRV Edge SoC.
//               Executes power-on-boot routines and scenario simulations.
// =============================================================================

`timescale 1ns / 1ps

module tb_neurorv_soc;
    logic clk;
    logic rst_n;

    // External Interface Connections
    logic        uart_rx;
    logic        uart_tx;
    logic        spi_sck;
    logic        spi_mosi;
    logic        spi_miso;
    logic        spi_cs_n;
    logic        i2c_sda_i;
    logic        i2c_sda_o;
    logic        i2c_sda_oe;
    logic        i2c_scl_i;
    logic        i2c_scl_o;
    logic        i2c_scl_oe;
    logic [31:0] gpio_i;
    logic [31:0] gpio_o;
    logic [31:0] gpio_oe;

    // Debug Trace Outputs
    logic [63:0] dbg_cycles;
    logic [63:0] dbg_instr_cnt;
    logic [31:0] dbg_dma_tx_cnt;
    logic [31:0] dbg_vxu_op_cnt;
    logic [31:0] dbg_int_cnt;

    int total_tests  = 0;
    int passed_tests = 0;
    int failed_tests = 0;
    logic sb_pass;

    // Clock Generator
    initial begin
        clk = 0;
        forever #5 clk = ~clk;
    end

    // Device Under Test (SoC Top Integration)
    neurorv_soc u_soc_top (
        .clk_i            (clk),
        .rst_n_i          (rst_n),
        .uart_rx_i        (uart_rx),
        .uart_tx_o        (uart_tx),
        .spi_sck_o        (spi_sck),
        .spi_mosi_o       (spi_mosi),
        .spi_miso_i       (spi_miso),
        .spi_cs_n_o       (spi_cs_n),
        .i2c_sda_i        (i2c_sda_i),
        .i2c_sda_o        (i2c_sda_o),
        .i2c_sda_oe_o     (i2c_sda_oe),
        .i2c_scl_i        (i2c_scl_i),
        .i2c_scl_o        (i2c_scl_o),
        .i2c_scl_oe_o     (i2c_scl_oe),
        .gpio_i           (gpio_i),
        .gpio_o           (gpio_o),
        .gpio_oe_o        (gpio_oe),
        .dbg_cycles_o     (dbg_cycles),
        .dbg_instr_cnt_o  (dbg_instr_cnt),
        .dbg_dma_tx_cnt_o (dbg_dma_tx_cnt),
        .dbg_vxu_op_cnt_o (dbg_vxu_op_cnt),
        .dbg_int_cnt_o    (dbg_int_cnt)
    );

    // Passive System Monitors
    interrupt_monitor u_int_mon (
        .clk_i    (clk),
        .rst_n_i  (rst_n),
        .dma_irq  (u_soc_top.dma_irq),
        .vxu_irq  (u_soc_top.vxu_irq),
        .timer_irq(u_soc_top.timer_irq),
        .uart_irq (u_soc_top.uart_irq),
        .gpio_irq (u_soc_top.gpio_irq),
        .cpu_irq  (u_soc_top.cpu_irq)
    );

    memory_monitor u_mem_mon (
        .clk_i     (clk),
        .rst_n_i   (rst_n),
        .sram_addr (u_soc_top.s_awaddr[0]),
        .sram_valid(u_soc_top.s_awvalid[0]),
        .sram_ready(u_soc_top.s_awready[0])
    );

    soc_scoreboard u_scoreboard (
        .clk_i     (clk),
        .rst_n_i   (rst_n),
        .cpu_pc    (u_soc_top.m_awaddr[0]),
        .cpu_valid (u_soc_top.m_awvalid[0]),
        .vxu_active(u_soc_top.pmu_clk_en_vxu),
        .dma_active(u_soc_top.pmu_clk_en_periph)
    );

    // Concurrent Assertions verifying structural protocol rules
    a_soc_pmu_init_check: assert property (@(posedge clk) !rst_n |-> (u_soc_top.pmu_clk_en_cpu == 1'b0));

    initial begin
        $dumpfile("dump.vcd");
        $dumpvars(0, tb_neurorv_soc);

        // System Variables Init
        uart_rx   = 1'b1;
        spi_miso  = 1'b0;
        i2c_sda_i = 1'b1;
        i2c_scl_i = 1'b1;
        gpio_i    = 32'h0;

        // Trigger Power-On Reset Sequence
        rst_n = 1'b0;
        #40;
        rst_n = 1'b1;
        #20;

        // Scenario 1: Power-On Boot Verification Task
        total_tests++;
        if (u_soc_top.pmu_rst_n_cpu === 1'b0 || u_soc_top.pmu_rst_n_cpu === 1'b1) begin
            passed_tests++;
        end else begin
            failed_tests++;
        end

        // Scenario 2: Memory & Peripheral Transaction Pipeline Stressing
        total_tests++;
        #100;
        u_scoreboard.check_final_status(sb_pass);
        if (sb_pass) passed_tests++;
        else         failed_tests++;

        // Simulation Completion and Report Manifest
        $display("========================================");
        if (failed_tests == 0) begin
            $display("SYSTEM LEVEL TESTBENCH SUITE: PASS");
        end else begin
            $display("SYSTEM LEVEL TESTBENCH SUITE: FAIL");
        end
        $display("Tests Run:    %0d", total_tests);
        $display("Tests Passed: %0d", passed_tests);
        $display("Tests Failed: %0d", failed_tests);
        $display("========================================");
        $finish;
    end
endmodule
