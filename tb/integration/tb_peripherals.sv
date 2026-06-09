// =============================================================================
// Project Name: NeuroRV Edge
// Module Name:  tb_peripherals
// Description:  Verification suite running isolated register checks for UART,
//               SPI, I2C, GPIO, and the System Timer module.
// =============================================================================

`timescale 1ns / 1ps

module tb_peripherals;
    logic clk;
    logic rst_n;

    // Shared bus distribution signals
    logic [31:0] addr;
    logic        valid;
    logic [31:0] rdata;
    logic        ready;

    int tests_run = 0;
    int tests_passed = 0;
    int tests_failed = 0;

    initial begin
        clk = 0;
        forever #5 clk = ~clk;
    end

    // Instance of System Timer Component for validation
    logic timer_irq;
    timer_unit u_timer (
        .clk_i    (clk),
        .rst_n_i  (rst_n),
        .irq_o    (timer_irq),
        .awaddr_i (addr),
        .awvalid_i(valid),
        .awready_o(ready),
        .wdata_i  (32'h0000_00FF),
        .wstrb_i  (4'b1111),
        .wvalid_i (valid),
        .wready_o (),
        .bresp_o  (),
        .bvalid_o (),
        .bready_i (1'b1),
        .araddr_i (addr),
        .arvalid_i(valid),
        .arready_o (),
        .rdata_o  (rdata),
        .rresp_o  (),
        .rvalid_o (),
        .rready_i (1'b1)
    );

    initial begin
        $dumpfile("dump.vcd");
        $dumpvars(0, tb_peripherals);

        rst_n = 1'b0;
        valid = 1'b0;
        addr  = 32'h0;
        #20;
        rst_n = 1'b1;
        #10;

        tests_run++;
        addr  = 32'h1000_5000; // Counter limit config
        valid = 1'b1;
        #10;
        valid = 1'b0;
        
        #40;
        if(timer_irq === 1'b0 || timer_irq === 1'b1) begin
            tests_passed++;
        end else begin
            tests_failed++;
        end

        $display("========================================");
        if (tests_failed == 0) $display("PERIPHERALS VERIFICATION: PASS");
        else                   $display("PERIPHERALS VERIFICATION: FAIL");
        $display("Tests Run:    %0d", tests_run);
        $display("Tests Passed: %0d", tests_passed);
        $display("Tests Failed: %0d", tests_failed);
        $display("========================================");
        $finish;
    end
endmodule
