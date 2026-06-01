// =============================================================================
// NeuroRV Edge — Instruction Fetch (IF) Stage
// File   : rtl/core/if_stage.sv
// =============================================================================

`timescale 1ns/1ps
`default_nettype none

module if_stage #(
    parameter logic [31:0] RESET_VECTOR = 32'h0000_0000
) (
    input  logic        clk_i,
    input  logic        rst_ni,
    input  logic        stall_i,
    input  logic        flush_i,
    input  logic        branch_taken_i,
    input  logic [31:0] branch_target_i,
    input  logic        trap_taken_i,
    input  logic [31:0] trap_vector_i,
    input  logic        mret_i,
    input  logic [31:0] mepc_i,
    // Instruction memory
    output logic [31:0] imem_addr_o,
    output logic        imem_req_o,
    input  logic [31:0] imem_rdata_i,
    input  logic        imem_gnt_i,
    input  logic        imem_rvalid_i,
    input  logic        imem_err_i,
    // Current PC
    output logic [31:0] pc_o,
    // IF/ID pipeline register outputs
    output logic [31:0] if_id_pc_o,
    output logic [31:0] if_id_instr_o,
    output logic        if_id_valid_o
);

    logic [31:0] pc_reg;
    logic [31:0] pc_next;
    logic        instr_valid;
    logic [31:0] instr_reg;
    logic        fetch_pending;

    // -------------------------------------------------------------------------
    // PC register
    // -------------------------------------------------------------------------
    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            pc_reg <= RESET_VECTOR;
        end else if (!stall_i) begin
            pc_reg <= pc_next;
        end
    end

    // -------------------------------------------------------------------------
    // PC next-state logic (priority: trap > mret > branch > +4)
    // -------------------------------------------------------------------------
    always_comb begin
        if (trap_taken_i)
            pc_next = trap_vector_i & ~32'h1; // clear LSB (vectored or direct)
        else if (mret_i)
            pc_next = mepc_i;
        else if (branch_taken_i)
            pc_next = branch_target_i;
        else
            pc_next = pc_reg + 32'd4;
    end

    // -------------------------------------------------------------------------
    // Memory request
    // -------------------------------------------------------------------------
    assign imem_addr_o = pc_reg;
    assign imem_req_o  = !stall_i;

    // -------------------------------------------------------------------------
    // Fetch tracking — assume single-cycle SRAM (rvalid same or next cycle)
    // For real memory with variable latency, a proper fetch buffer is needed
    // -------------------------------------------------------------------------
    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            if_id_pc_o    <= '0;
            if_id_instr_o <= 32'h0000_0013; // NOP (addi x0,x0,0)
            if_id_valid_o <= 1'b0;
        end else if (!stall_i) begin
            if (flush_i || branch_taken_i || trap_taken_i || mret_i) begin
                if_id_pc_o    <= '0;
                if_id_instr_o <= 32'h0000_0013; // NOP bubble
                if_id_valid_o <= 1'b0;
            end else if (imem_rvalid_i) begin
                if_id_pc_o    <= pc_reg;
                if_id_instr_o <= imem_err_i ? 32'h0 : imem_rdata_i;
                if_id_valid_o <= !imem_err_i;
            end else begin
                if_id_pc_o    <= pc_reg;
                if_id_instr_o <= 32'h0000_0013;
                if_id_valid_o <= 1'b0;
            end
        end
    end

    assign pc_o = pc_reg;

endmodule

`default_nettype wire
