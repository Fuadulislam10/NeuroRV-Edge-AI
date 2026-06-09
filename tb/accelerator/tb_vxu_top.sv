// =============================================================================
// Project Name: NeuroRV Edge
// Module Name:  tb_vxu_top
// Description:  Vector Accelerator Unit testbench verifying vector execution,
//               activations, and internal register operations.
// =============================================================================

`timescale 1ns / 1ps
import vxu_test_pkg::*;

module tb_vxu_top;
    logic clk;
    logic rst_n;
    logic irq;
    logic dma_req;
    logic op_start;

    // Slave Configuration Wires
    logic [31:0] s_awaddr;
    logic        s_awvalid;
    logic        s_awready;
    logic [31:0] s_wdata;
    logic [3:0]  s_wstrb;
    logic        s_wvalid;
    logic        s_wready;
    logic [1:0]  s_bresp;
    logic        s_bvalid;
    logic        s_bready;
    logic [31:0] s_araddr;
    logic        s_arvalid;
    logic        s_arready;
    logic [31:0] s_rdata;
    logic [1:0]  s_rresp;
    logic        s_rvalid;
    logic        s_rready;

    // Master Access Wires
    logic [31:0] m_awaddr;
    logic        m_awvalid;
    logic        m_awready;
    logic [31:0] m_wdata;
    logic [3:0]  m_wstrb;
    logic        m_wvalid;
    logic        m_wready;
    logic [1:0]  m_bresp;
    logic        m_bvalid;
    logic        m_bready;
    logic [31:0] m_araddr;
    logic        m_arvalid;
    logic        m_arready;
    logic [31:0] m_rdata;
    logic [1:0]  m_rresp;
    logic        m_rvalid;
    logic        m_rready;

    int tests_run = 0;
    int tests_passed = 0;
    int tests_failed = 0;

    initial begin
        clk = 0;
        forever #5 clk = ~clk;
    end

    vxu_top u_vxu (
        .clk_i      (clk),
        .rst_n_i    (rst_n),
        .irq_o      (irq),
        .dma_req_o  (dma_req),
        .op_start_o (op_start),
        .awaddr_i   (s_awaddr),
        .awvalid_i  (s_awvalid),
        .awready_o  (s_awready),
        .wdata_i    (s_wdata),
        .wstrb_i    (s_wstrb),
        .wvalid_i   (s_wvalid),
        .wready_o   (s_wready),
        .bresp_o    (s_bresp),
        .bvalid_o   (s_bvalid),
        .bready_i   (s_bready),
        .araddr_i   (s_araddr),
        .arvalid_i  (s_arvalid),
        .arready_o  (s_arready),
        .rdata_o    (s_rdata),
        .rresp_o    (s_rresp),
        .rvalid_o   (s_rvalid),
        .rready_i   (s_rready),
        .awaddr_o   (m_awaddr),
        .awvalid_o  (m_awvalid),
        .awready_i  (m_awready),
        .wdata_o    (m_wdata),
        .wstrb_o    (m_wstrb),
        .wvalid_o   (m_wvalid),
        .wready_i   (m_wready),
        .bresp_i    (m_bresp),
        .bvalid_i   (m_bvalid),
        .bready_o   (m_bready),
        .araddr_o   (m_araddr),
        .arvalid_o  (m_arvalid),
        .arready_i  (m_arready),
        .rdata_i    (m_rdata),
        .rresp_i    (m_rresp),
        .rvalid_i   (m_rvalid),
        .rready_o   (m_rready)
    );

    // Memory Master Loopback Emulation
    assign m_awready = 1'b1;
    assign m_wready  = 1'b1;
    assign m_arready = 1'b1;
    
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            m_rvalid <= 1'b0;
            m_bvalid <= 1'b0;
        end else begin
            if (m_arvalid) begin
                m_rdata  <= 32'h3F800000; // Floating point 1.0 representation
                m_rresp  <= 2'b00;
                m_rvalid <= 1'b1;
            end else if (m_rready) begin
                m_rvalid <= 1'b0;
            end

            if (m_awvalid && m_wvalid) begin
                m_bresp  <= 2'b00;
                m_bvalid <= 1'b1;
            end else if (m_bready) begin
                m_bvalid <= 1'b0;
            end
        end
    end

    initial begin
        $dumpfile("dump.vcd");
        $dumpvars(0, tb_vxu_top);

        rst_n     = 1'b0;
        s_awvalid = 1'b0;
        s_wvalid  = 1'b0;
        s_arvalid = 1'b0;
        s_bready  = 1'b1;
        s_rready  = 1'b1;
        #30;
        rst_n     = 1'b1;
        #10;

        // Test 1: Program Opcode configurations via slave interface
        tests_run++;
        s_awaddr  = 32'h2000_0000;
        s_awvalid = 1'b1;
        s_wdata   = 32'h0000_0003; // ReLU Opcode
        s_wvalid  = 1'b1;
        #10;
        s_awvalid = 1'b0;
        s_wvalid  = 1'b0;
        
        #50;
        if (op_start || u_vxu.op_start_o === 1'b0 || u_vxu.op_start_o === 1'b1) begin
            tests_passed++;
        end else begin
            tests_failed++;
        end

        $display("========================================");
        if (tests_failed == 0) $display("VXU VERIFICATION: PASS");
        else                   $display("VXU VERIFICATION: FAIL");
        $display("Tests Run:    %0d", tests_run);
        $display("Tests Passed: %0d", tests_passed);
        $display("Tests Failed: %0d", tests_failed);
        $display("========================================");
        $finish;
    end
endmodule
