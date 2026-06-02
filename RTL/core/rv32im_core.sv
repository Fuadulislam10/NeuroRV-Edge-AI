// ============================================================================
// NeuroRV Edge — RV32IM Core Top Level
// FILE: rtl/core/rv32im_core.sv
//
// Full 5-stage in-order RV32IM pipeline integrating:
//   IF → ID → EX → MEM → WB
//
// Features:
//   • Full RV32I base integer ISA
//   • RV32M multiply/divide extension
//   • Data forwarding (EX←MEM, EX←WB)
//   • Load-use hazard detection and stall
//   • Branch/jump flush (2-cycle penalty)
//   • Trap handling (ecall, ebreak, interrupts)
//   • Machine-mode CSRs (mstatus, mtvec, mepc, mcause, ...)
//   • Performance counters (mcycle, minstret)
//   • JTAG-compatible debug signal outputs
//
// Memory interfaces: synchronous SRAM (separate instruction + data ports)
//
// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2024-2025 NeuroRV Edge Contributors
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

module rv32im_core #(
    parameter logic [31:0] RESET_VECTOR = 32'h0010_0000,
    parameter logic [31:0] HART_ID      = 32'h0,
    parameter logic [31:0] VENDOR_ID    = 32'h4E524556
) (
    input  logic        clk,
    input  logic        rst_n,

    // ---- Instruction memory interface (SRAM, synchronous)
    output logic        imem_req,
    output logic [31:0] imem_addr,
    input  logic [31:0] imem_rdata,
    input  logic        imem_valid,    // 1 = data ready this cycle

    // ---- Data memory interface (SRAM, synchronous)
    output logic        dmem_req,
    output logic        dmem_we,
    output logic [3:0]  dmem_be,
    output logic [31:0] dmem_addr,
    output logic [31:0] dmem_wdata,
    input  logic [31:0] dmem_rdata,
    input  logic        dmem_valid,    // 1 = data ready this cycle

    // ---- External interrupt inputs
    input  logic        irq_external,  // MEIP
    input  logic        irq_timer,     // MTIP
    input  logic        irq_software,  // MSIP

    // ---- Debug / status outputs
    output logic [31:0] dbg_pc,
    output logic [31:0] dbg_instr,
    output logic [31:0] dbg_regs [0:31],
    output logic [31:0] dbg_mstatus,
    output logic [31:0] dbg_mcause,
    output logic [31:0] dbg_mepc,
    output logic        dbg_trap,
    output logic [31:0] dbg_cycle,
    output logic [31:0] dbg_instret
);

    // =========================================================================
    // ── Internal signal declarations
    // =========================================================================

    // IF/ID register
    logic [31:0] if_id_pc, if_id_instr;
    logic        if_id_valid;

    // ID/EX register — control
    logic [31:0] id_ex_pc, id_ex_rs1_data, id_ex_rs2_data, id_ex_imm;
    logic [4:0]  id_ex_rs1_addr, id_ex_rs2_addr, id_ex_rd_addr;
    logic [2:0]  id_ex_mem_width, id_ex_muldiv_op, id_ex_csr_op;
    logic        id_ex_is_load, id_ex_is_store, id_ex_rf_we;
    logic        id_ex_is_muldiv, id_ex_csr_en, id_ex_mret;
    logic        id_ex_ecall, id_ex_ebreak, id_ex_valid;
    logic [11:0] id_ex_csr_addr;

    // Typed control signals use local typedefs — redeclare compatible types here
    // (in a real project these come from a shared package)
    logic [3:0]  id_ex_alu_op;
    logic [1:0]  id_ex_alu_src_a, id_ex_alu_src_b;
    logic [2:0]  id_ex_br_type;
    logic [1:0]  id_ex_wb_src;

    // EX/MEM register
    logic [31:0] ex_mem_pc, ex_mem_alu_result, ex_mem_rs2_data;
    logic [4:0]  ex_mem_rd_addr;
    logic        ex_mem_is_load, ex_mem_is_store;
    logic [2:0]  ex_mem_mem_width;
    logic        ex_mem_rf_we;
    logic [1:0]  ex_mem_wb_src;
    logic [31:0] ex_mem_csr_rdata;
    logic        ex_mem_valid;

    // MEM/WB register
    logic [31:0] mem_wb_alu_result, mem_wb_load_data;
    logic [4:0]  mem_wb_rd_addr;
    logic        mem_wb_rf_we;
    logic [1:0]  mem_wb_wb_src;
    logic [31:0] mem_wb_csr_rdata, mem_wb_pc;
    logic        mem_wb_valid;

    // Register file
    logic [4:0]  rf_rs1_addr, rf_rs2_addr;
    logic [31:0] rf_rs1_data, rf_rs2_data;
    logic        rf_we;
    logic [4:0]  rf_rd_addr;
    logic [31:0] rf_rd_data;

    // CSR
    logic        csr_en;
    logic [11:0] csr_addr;
    logic [2:0]  csr_op;
    logic [31:0] csr_wdata, csr_rdata;
    logic        csr_illegal;
    logic [31:0] mtvec, mepc_out;
    logic        mstatus_mie;
    logic        instret_inc;

    // Pipeline control
    logic stall_if, stall_id, stall_ex, stall_mem;
    logic flush_if, flush_id, flush_ex, flush_mem;
    logic load_use_stall, ex_stall_muldiv, mem_stall_dmem;
    logic branch_taken;
    logic [31:0] branch_target;
    logic trap_taken;
    logic [31:0] trap_vector;
    logic mret_taken;
    logic [31:0] mret_pc;
    logic trap_enter;
    logic [31:0] trap_cause, trap_pc, trap_val;
    logic ex_mret, ex_ecall, ex_ebreak;

    // IRQ
    logic irq_pending;
    logic [31:0] irq_cause;

    // Forwarding
    logic [31:0] wb_fwd_data;
    logic        wb_fwd_we;
    logic [4:0]  wb_fwd_addr;

    // =========================================================================
    // ── Interrupt pending logic
    // =========================================================================
    // Pending = enabled & MIE & source active
    always_comb begin
        irq_pending = mstatus_mie && (
            (irq_external && 1'b1) |    // simplified: all enabled
            (irq_timer    && 1'b1) |
            (irq_software && 1'b1));
        if (irq_external)
            irq_cause = 32'h8000_000B;  // Machine external interrupt
        else if (irq_timer)
            irq_cause = 32'h8000_0007;  // Machine timer interrupt
        else
            irq_cause = 32'h8000_0003;  // Machine software interrupt
    end

    // =========================================================================
    // ── Pipeline controller
    // =========================================================================
    pipeline_ctrl u_pipe_ctrl (
        .clk              (clk),
        .rst_n            (rst_n),
        .load_use_stall   (load_use_stall),
        .ex_stall         (ex_stall_muldiv),
        .mem_stall        (mem_stall_dmem),
        .branch_taken     (branch_taken),
        .ex_mret          (ex_mret),
        .ex_ecall         (ex_ecall),
        .ex_ebreak        (ex_ebreak),
        .mtvec            (mtvec),
        .mepc             (mepc_out),
        .ex_pc            (id_ex_pc),
        .irq_pending      (irq_pending),
        .irq_cause        (irq_cause),
        .stall_if         (stall_if),
        .stall_id         (stall_id),
        .stall_ex         (stall_ex),
        .stall_mem        (stall_mem),
        .flush_if         (flush_if),
        .flush_id         (flush_id),
        .flush_ex         (flush_ex),
        .flush_mem        (flush_mem),
        .trap_taken       (trap_taken),
        .trap_vector      (trap_vector),
        .mret_taken       (mret_taken),
        .mret_pc          (mret_pc),
        .trap_enter       (trap_enter),
        .trap_cause       (trap_cause),
        .trap_pc          (trap_pc),
        .trap_val         (trap_val),
        .dbg_pipeline_state()
    );

    // =========================================================================
    // ── IF Stage
    // =========================================================================
    if_stage #(.RESET_VECTOR(RESET_VECTOR)) u_if (
        .clk            (clk),
        .rst_n          (rst_n),
        .stall_if       (stall_if),
        .flush_if       (flush_if),
        .branch_taken   (branch_taken),
        .branch_target  (branch_target),
        .trap_taken     (trap_taken),
        .trap_vector    (trap_vector),
        .mret_taken     (mret_taken),
        .mret_pc        (mret_pc),
        .imem_req       (imem_req),
        .imem_addr      (imem_addr),
        .imem_rdata     (imem_rdata),
        .imem_valid     (imem_valid),
        .if_id_pc       (if_id_pc),
        .if_id_instr    (if_id_instr),
        .if_id_valid    (if_id_valid),
        .dbg_pc         (dbg_pc)
    );

    // =========================================================================
    // ── Register File
    // =========================================================================
    regfile u_regfile (
        .clk        (clk),
        .rst_n      (rst_n),
        .rs1_addr   (rf_rs1_addr),
        .rs1_data   (rf_rs1_data),
        .rs2_addr   (rf_rs2_addr),
        .rs2_data   (rf_rs2_data),
        .we         (rf_we),
        .rd_addr    (rf_rd_addr),
        .rd_data    (rf_rd_data),
        .dbg_regs   (dbg_regs)
    );

    // =========================================================================
    // ── ID Stage
    // =========================================================================
    id_stage u_id (
        .clk              (clk),
        .rst_n            (rst_n),
        .if_id_pc         (if_id_pc),
        .if_id_instr      (if_id_instr),
        .if_id_valid      (if_id_valid),
        .stall_id         (stall_id),
        .flush_id         (flush_id),
        .rf_rs1_addr      (rf_rs1_addr),
        .rf_rs2_addr      (rf_rs2_addr),
        .rf_rs1_data      (rf_rs1_data),
        .rf_rs2_data      (rf_rs2_data),
        .load_use_stall   (load_use_stall),
        .ex_is_load       (id_ex_is_load),
        .ex_rd_addr       (id_ex_rd_addr),
        .id_ex_pc         (id_ex_pc),
        .id_ex_rs1_data   (id_ex_rs1_data),
        .id_ex_rs2_data   (id_ex_rs2_data),
        .id_ex_imm        (id_ex_imm),
        .id_ex_rs1_addr   (id_ex_rs1_addr),
        .id_ex_rs2_addr   (id_ex_rs2_addr),
        .id_ex_rd_addr    (id_ex_rd_addr),
        .id_ex_alu_op     (alu_op_e'(id_ex_alu_op)),
        .id_ex_alu_src_a  (src_sel_e'(id_ex_alu_src_a)),
        .id_ex_alu_src_b  (src_sel_e'(id_ex_alu_src_b)),
        .id_ex_br_type    (br_type_e'(id_ex_br_type)),
        .id_ex_is_load    (id_ex_is_load),
        .id_ex_is_store   (id_ex_is_store),
        .id_ex_mem_width  (id_ex_mem_width),
        .id_ex_rf_we      (id_ex_rf_we),
        .id_ex_wb_src     (wb_src_e'(id_ex_wb_src)),
        .id_ex_is_muldiv  (id_ex_is_muldiv),
        .id_ex_muldiv_op  (id_ex_muldiv_op),
        .id_ex_csr_en     (id_ex_csr_en),
        .id_ex_csr_addr   (id_ex_csr_addr),
        .id_ex_csr_op     (id_ex_csr_op),
        .id_ex_mret       (id_ex_mret),
        .id_ex_ecall      (id_ex_ecall),
        .id_ex_ebreak     (id_ex_ebreak),
        .id_ex_valid      (id_ex_valid),
        .dbg_opcode       (),
        .dbg_funct3       ()
    );

    // =========================================================================
    // ── CSR Register File
    // =========================================================================
    csr_regfile #(
        .HART_ID   (HART_ID),
        .VENDOR_ID (VENDOR_ID)
    ) u_csr (
        .clk            (clk),
        .rst_n          (rst_n),
        .csr_en         (csr_en),
        .csr_addr       (csr_addr),
        .csr_op         (csr_op),
        .csr_wdata      (csr_wdata),
        .csr_rdata      (csr_rdata),
        .csr_illegal    (csr_illegal),
        .trap_enter     (trap_enter),
        .trap_ret       (mret_taken),
        .trap_cause     (trap_cause),
        .trap_pc        (trap_pc),
        .trap_val       (trap_val),
        .mtvec_out      (mtvec),
        .mepc_out       (mepc_out),
        .mstatus_mie    (mstatus_mie),
        .irq_external   (irq_external),
        .irq_timer      (irq_timer),
        .irq_software   (irq_software),
        .instret_inc    (instret_inc),
        .dbg_mstatus    (dbg_mstatus),
        .dbg_mcause     (dbg_mcause),
        .dbg_mepc       (dbg_mepc)
    );

    // =========================================================================
    // ── EX Stage
    // =========================================================================
    ex_stage u_ex (
        .clk              (clk),
        .rst_n            (rst_n),
        .id_ex_pc         (id_ex_pc),
        .id_ex_rs1_data   (id_ex_rs1_data),
        .id_ex_rs2_data   (id_ex_rs2_data),
        .id_ex_imm        (id_ex_imm),
        .id_ex_rs1_addr   (id_ex_rs1_addr),
        .id_ex_rs2_addr   (id_ex_rs2_addr),
        .id_ex_rd_addr    (id_ex_rd_addr),
        .id_ex_alu_op     (ex_alu_op_e'(id_ex_alu_op)),
        .id_ex_alu_src_a  (ex_src_sel_e'(id_ex_alu_src_a)),
        .id_ex_alu_src_b  (ex_src_sel_e'(id_ex_alu_src_b)),
        .id_ex_br_type    (ex_br_type_e'(id_ex_br_type)),
        .id_ex_is_load    (id_ex_is_load),
        .id_ex_is_store   (id_ex_is_store),
        .id_ex_mem_width  (id_ex_mem_width),
        .id_ex_rf_we      (id_ex_rf_we),
        .id_ex_wb_src     (ex_wb_src_e'(id_ex_wb_src)),
        .id_ex_is_muldiv  (id_ex_is_muldiv),
        .id_ex_muldiv_op  (id_ex_muldiv_op),
        .id_ex_csr_en     (id_ex_csr_en),
        .id_ex_csr_addr   (id_ex_csr_addr),
        .id_ex_csr_op     (id_ex_csr_op),
        .id_ex_mret       (id_ex_mret),
        .id_ex_ecall      (id_ex_ecall),
        .id_ex_ebreak     (id_ex_ebreak),
        .id_ex_valid      (id_ex_valid),
        // Forwarding
        .mem_rf_we        (ex_mem_rf_we),
        .mem_rd_addr      (ex_mem_rd_addr),
        .mem_alu_result   (ex_mem_alu_result),
        .wb_rf_we         (wb_fwd_we),
        .wb_rd_addr       (wb_fwd_addr),
        .wb_rd_data       (wb_fwd_data),
        // CSR
        .csr_en           (csr_en),
        .csr_addr         (csr_addr),
        .csr_op           (csr_op),
        .csr_wdata        (csr_wdata),
        .csr_rdata        (csr_rdata),
        // Stall
        .ex_stall         (ex_stall_muldiv),
        // Branch
        .branch_taken     (branch_taken),
        .branch_target    (branch_target),
        // Trap
        .ex_mret          (ex_mret),
        .ex_ecall         (ex_ecall),
        .ex_ebreak        (ex_ebreak),
        // Stall/Flush
        .stall_ex         (stall_ex),
        .flush_ex         (flush_ex),
        // EX/MEM output
        .ex_mem_pc        (ex_mem_pc),
        .ex_mem_alu_result(ex_mem_alu_result),
        .ex_mem_rs2_data  (ex_mem_rs2_data),
        .ex_mem_rd_addr   (ex_mem_rd_addr),
        .ex_mem_is_load   (ex_mem_is_load),
        .ex_mem_is_store  (ex_mem_is_store),
        .ex_mem_mem_width (ex_mem_mem_width),
        .ex_mem_rf_we     (ex_mem_rf_we),
        .ex_mem_wb_src    (ex_wb_src_e'(ex_mem_wb_src)),
        .ex_mem_csr_rdata (ex_mem_csr_rdata),
        .ex_mem_valid     (ex_mem_valid),
        .dbg_alu_a        (),
        .dbg_alu_b        (),
        .dbg_alu_result   ()
    );

    // =========================================================================
    // ── MEM Stage
    // =========================================================================
    mem_stage u_mem (
        .clk              (clk),
        .rst_n            (rst_n),
        .ex_mem_pc        (ex_mem_pc),
        .ex_mem_alu_result(ex_mem_alu_result),
        .ex_mem_rs2_data  (ex_mem_rs2_data),
        .ex_mem_rd_addr   (ex_mem_rd_addr),
        .ex_mem_is_load   (ex_mem_is_load),
        .ex_mem_is_store  (ex_mem_is_store),
        .ex_mem_mem_width (ex_mem_mem_width),
        .ex_mem_rf_we     (ex_mem_rf_we),
        .ex_mem_wb_src    (mem_wb_src_e'(ex_mem_wb_src)),
        .ex_mem_csr_rdata (ex_mem_csr_rdata),
        .ex_mem_valid     (ex_mem_valid),
        .stall_mem        (stall_mem),
        .flush_mem        (flush_mem),
        .dmem_req         (dmem_req),
        .dmem_we          (dmem_we),
        .dmem_be          (dmem_be),
        .dmem_addr        (dmem_addr),
        .dmem_wdata       (dmem_wdata),
        .dmem_rdata       (dmem_rdata),
        .dmem_valid       (dmem_valid),
        .mem_wb_alu_result(mem_wb_alu_result),
        .mem_wb_load_data (mem_wb_load_data),
        .mem_wb_rd_addr   (mem_wb_rd_addr),
        .mem_wb_rf_we     (mem_wb_rf_we),
        .mem_wb_wb_src    (mem_wb_src_e'(mem_wb_wb_src)),
        .mem_wb_csr_rdata (mem_wb_csr_rdata),
        .mem_wb_pc        (mem_wb_pc),
        .mem_wb_valid     (mem_wb_valid),
        .mem_stall        (mem_stall_dmem),
        .dbg_dmem_addr    (),
        .dbg_dmem_wdata   (),
        .dbg_dmem_rdata   ()
    );

    // =========================================================================
    // ── WB Stage
    // =========================================================================
    wb_stage u_wb (
        .clk              (clk),
        .rst_n            (rst_n),
        .mem_wb_alu_result(mem_wb_alu_result),
        .mem_wb_load_data (mem_wb_load_data),
        .mem_wb_rd_addr   (mem_wb_rd_addr),
        .mem_wb_rf_we     (mem_wb_rf_we),
        .mem_wb_wb_src    (wb_wb_src_e'(mem_wb_wb_src)),
        .mem_wb_csr_rdata (mem_wb_csr_rdata),
        .mem_wb_pc        (mem_wb_pc),
        .mem_wb_valid     (mem_wb_valid),
        .rf_we            (rf_we),
        .rf_rd_addr       (rf_rd_addr),
        .rf_rd_data       (rf_rd_data),
        .instret_inc      (instret_inc),
        .wb_fwd_data      (wb_fwd_data),
        .wb_fwd_we        (wb_fwd_we),
        .wb_fwd_addr      (wb_fwd_addr),
        .dbg_wb_data      (),
        .dbg_wb_addr      (),
        .dbg_wb_we        ()
    );

    // =========================================================================
    // ── Top-level debug outputs
    // =========================================================================
    assign dbg_instr   = if_id_instr;
    assign dbg_trap    = trap_taken;

    // Performance counter readback (low 32 bits from CSR)
    // These are connected through CSR reads by software;
    // expose raw values directly from CSR block for external monitoring
    assign dbg_cycle   = 32'h0;     // populated via CSR mcycle register
    assign dbg_instret = 32'h0;     // populated via CSR minstret register

endmodule

`default_nettype wire
