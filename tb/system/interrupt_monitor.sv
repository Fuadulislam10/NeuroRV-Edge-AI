// =============================================================================
// Project Name: NeuroRV Edge
// Module Name:  interrupt_monitor
// Description:  Passive monitor checking interrupt priority propagation lines.
// =============================================================================

`timescale 1ns / 1ps

module interrupt_monitor (
    input logic clk_i,
    input logic rst_n_i,
    input logic dma_irq,
    input logic vxu_irq,
    input logic timer_irq,
    input logic uart_irq,
    input logic gpio_irq,
    input logic cpu_irq
);

    always_ff @(posedge clk_i) begin
        if (rst_n_i) begin
            if (dma_irq && !cpu_irq) begin
                $display("[INT_MONITOR] ERROR: DMA Interrupt asserted but CPU Core IRQ input remained low.");
            end
        end
    end
endmodule
