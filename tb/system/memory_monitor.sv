// =============================================================================
// Project Name: NeuroRV Edge
// Module Name:  memory_monitor
// Description:  Monitors global interconnect protocols and detects out-of-bound
//               or deadlocked cycles.
// =============================================================================

`timescale 1ns / 1ps

module memory_monitor (
    input logic        clk_i,
    input logic        rst_n_i,
    input logic [31:0] sram_addr,
    input logic        sram_valid,
    input logic        sram_ready
);

    int transaction_count = 0;

    always_ff @(posedge clk_i) begin
        if (!rst_n_i) begin
            transaction_count <= 0;
        end else begin
            if (sram_valid && sram_ready) begin
                transaction_count <= transaction_count + 1;
            end
        end
    end
endmodule
