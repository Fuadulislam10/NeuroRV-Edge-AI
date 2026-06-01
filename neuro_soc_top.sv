// ============================================================================
// NeuroRV Edge — SoC Top-Level Integration
// File   : rtl/neuro_soc_top.sv
// Author : NeuroRV Edge Contributors
// License: Apache 2.0
//
// Description:
//   Top-level SoC integrating all subsystems:
//   - RISC-V RV32IM 5-stage pipeline CPU
//   - 16-lane Vector Processing Unit
//   - AXI-lite Interconnect + 64KB SRAM
//   - Power Management Unit
//   - UART, GPIO
//   - Clock/Reset management
//
// Clock Domains:
//   clk_sys: main system clock (CPU, VPU, SRAM, interconnect)
//
// Memory Map:
//   0x0000_0000 – 0x0000_FFFF : 64KB SRAM (code + data)
//   0x1000_0000 – 0x1000_001F : VPU control registers
//   0x2000_0000 – 0x2000_000F : GPIO
//   0x3000_0000 – 0x3000_000F : UART debug
//   0x4000_0000 – 0x4000_001F : PMU registers
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

module neuro_soc_top #(
  parameter logic [31:0] RESET_PC    = 32'h0000_0000,
  parameter int          CLK_FREQ_HZ = 50_000_000
)(
  // Clock and Reset
  input  logic        clk_sys,
  input  logic        rst_ext_n,    // External active-low reset

  // GPIO
  input  logic [31:0] gpio_in,
  output logic [31:0] gpio_out,
  output logic [31:0] gpio_dir,

  // UART
  output logic        uart_tx,
  input  logic        uart_rx,

  // Debug / Trace
  output logic [31:0] debug_pc,
  output logic [2:0]  debug_pwr_state,
  output logic        debug_vpu_busy
);

  // ============================================================
  // Internal Reset Synchronization (2-FF synchronizer)
  // ============================================================
  logic rst_sync_q1, rst_sync_q2;
  logic rst_n_sync;

  always_ff @(posedge clk_sys or negedge rst_ext_n) begin
    if (!rst_ext_n) begin
      rst_sync_q1 <= 1'b0;
      rst_sync_q2 <= 1'b0;
    end else begin
      rst_sync_q1 <= 1'b1;
      rst_sync_q2 <= rst_sync_q1;
    end
  end
  assign rst_n_sync = rst_sync_q2;

  // ============================================================
  // Power Manager Clock Gate Signals
  // ============================================================
  logic clk_en_cpu, clk_en_vpu, clk_en_mem, clk_en_periph;
  logic rst_n_cpu, rst_n_vpu, rst_n_mem;
  logic cpu_halt;

  // Gated clocks (ICG simulation model — synthesis will infer ICGs)
  logic clk_cpu, clk_vpu, clk_mem, clk_periph;

  // Simple clock gate model for simulation (replace with ICG cells at ASIC)
  assign clk_cpu    = clk_sys & clk_en_cpu;
  assign clk_vpu    = clk_sys & clk_en_vpu;
  assign clk_mem    = clk_sys & clk_en_mem;
  assign clk_periph = clk_sys & clk_en_periph;

  // Combined resets
  logic cpu_rst_n, vpu_rst_n, mem_rst_n;
  assign cpu_rst_n = rst_n_sync & rst_n_cpu;
  assign vpu_rst_n = rst_n_sync & rst_n_vpu;
  assign mem_rst_n = rst_n_sync & rst_n_mem;

  // ============================================================
  // CPU ↔ Interconnect Wires
  // ============================================================
  logic [31:0] cpu_imem_addr, cpu_imem_data;
  logic        cpu_imem_valid;

  logic [31:0] cpu_dmem_addr, cpu_dmem_wdata, cpu_dmem_rdata;
  logic [3:0]  cpu_dmem_wstrb;
  logic        cpu_dmem_we, cpu_dmem_re, cpu_dmem_ack;
  logic        cpu_active_sig;
  logic [31:0] cpu_debug_pc;

  // PMU register access (CPU data bus passthrough for 0x4000_0000 space)
  logic [31:0] pmu_addr, pmu_wdata, pmu_rdata;
  logic        pmu_we, pmu_re;

  // ============================================================
  // VPU ↔ Interconnect Wires
  // ============================================================
  logic [31:0] vpu_ctrl_addr, vpu_ctrl_wdata, vpu_ctrl_rdata;
  logic        vpu_ctrl_we, vpu_ctrl_re;

  logic [31:0] vpu_mem_addr, vpu_mem_wdata, vpu_mem_rdata;
  logic        vpu_mem_we, vpu_mem_re, vpu_mem_ack;
  logic        vpu_busy, vpu_done, vpu_irq;

  // ============================================================
  // PMU Wires
  // ============================================================
  logic [3:0]  clk_div_hint;
  logic        pmu_irq;
  logic [2:0]  pmu_state_dbg;

  // ============================================================
  // CPU DMem Address Decode for PMU (0x4000_0000 space)
  // ============================================================
  logic is_pmu_access;
  assign is_pmu_access = (cpu_dmem_addr[31:8] == 24'h400000) &&
                         (cpu_dmem_we || cpu_dmem_re);

  // Mux CPU data response between interconnect and PMU
  logic [31:0] cpu_dmem_rdata_ic;  // from interconnect
  logic        cpu_dmem_ack_ic;

  assign cpu_dmem_rdata = is_pmu_access ? pmu_rdata : cpu_dmem_rdata_ic;
  assign cpu_dmem_ack   = is_pmu_access ? 1'b1      : cpu_dmem_ack_ic;

  // PMU register access routing
  assign pmu_addr  = cpu_dmem_addr;
  assign pmu_wdata = cpu_dmem_wdata;
  assign pmu_we    = is_pmu_access & cpu_dmem_we;
  assign pmu_re    = is_pmu_access & cpu_dmem_re;

  // Interconnect data access (pass only non-PMU accesses)
  logic cpu_dmem_we_ic, cpu_dmem_re_ic;
  assign cpu_dmem_we_ic = cpu_dmem_we & ~is_pmu_access;
  assign cpu_dmem_re_ic = cpu_dmem_re & ~is_pmu_access;

  // ============================================================
  // Module Instantiations
  // ============================================================

  // --- RISC-V CPU Core ---
  neuro_rv_core #(
    .RESET_PC(RESET_PC)
  ) u_cpu (
    .clk        (clk_cpu),
    .rst_n      (cpu_rst_n),
    .imem_addr  (cpu_imem_addr),
    .imem_data  (cpu_imem_data),
    .imem_valid (cpu_imem_valid),
    .dmem_addr  (cpu_dmem_addr),
    .dmem_wdata (cpu_dmem_wdata),
    .dmem_wstrb (cpu_dmem_wstrb),
    .dmem_we    (cpu_dmem_we),
    .dmem_re    (cpu_dmem_re),
    .dmem_rdata (cpu_dmem_rdata),
    .dmem_ack   (cpu_dmem_ack),
    .debug_pc   (cpu_debug_pc),
    .cpu_active (cpu_active_sig)
  );

  // --- Vector Processing Unit ---
  neuro_vector_unit u_vpu (
    .clk        (clk_vpu),
    .rst_n      (vpu_rst_n),
    .ctrl_addr  (vpu_ctrl_addr),
    .ctrl_wdata (vpu_ctrl_wdata),
    .ctrl_we    (vpu_ctrl_we),
    .ctrl_re    (vpu_ctrl_re),
    .ctrl_rdata (vpu_ctrl_rdata),
    .mem_addr   (vpu_mem_addr),
    .mem_wdata  (vpu_mem_wdata),
    .mem_we     (vpu_mem_we),
    .mem_re     (vpu_mem_re),
    .mem_rdata  (vpu_mem_rdata),
    .mem_ack    (vpu_mem_ack),
    .vpu_busy   (vpu_busy),
    .vpu_done   (vpu_done),
    .vpu_irq    (vpu_irq)
  );

  // --- Interconnect + SRAM ---
  neuro_interconnect #(
    .CLK_FREQ_HZ(CLK_FREQ_HZ)
  ) u_ic (
    .clk            (clk_mem),
    .rst_n          (mem_rst_n),
    // CPU instruction
    .cpu_imem_addr  (cpu_imem_addr),
    .cpu_imem_data  (cpu_imem_data),
    .cpu_imem_valid (cpu_imem_valid),
    // CPU data
    .cpu_dmem_addr  (cpu_dmem_addr),
    .cpu_dmem_wdata (cpu_dmem_wdata),
    .cpu_dmem_wstrb (cpu_dmem_wstrb),
    .cpu_dmem_we    (cpu_dmem_we_ic),
    .cpu_dmem_re    (cpu_dmem_re_ic),
    .cpu_dmem_rdata (cpu_dmem_rdata_ic),
    .cpu_dmem_ack   (cpu_dmem_ack_ic),
    // VPU data
    .vpu_mem_addr   (vpu_mem_addr),
    .vpu_mem_wdata  (vpu_mem_wdata),
    .vpu_mem_we     (vpu_mem_we),
    .vpu_mem_re     (vpu_mem_re),
    .vpu_mem_rdata  (vpu_mem_rdata),
    .vpu_mem_ack    (vpu_mem_ack),
    // VPU control
    .vpu_ctrl_addr  (vpu_ctrl_addr),
    .vpu_ctrl_wdata (vpu_ctrl_wdata),
    .vpu_ctrl_we    (vpu_ctrl_we),
    .vpu_ctrl_re    (vpu_ctrl_re),
    .vpu_ctrl_rdata (vpu_ctrl_rdata),
    // GPIO
    .gpio_in        (gpio_in),
    .gpio_out       (gpio_out),
    .gpio_dir       (gpio_dir),
    // UART
    .uart_tx_pin    (uart_tx),
    .uart_rx_pin    (uart_rx)
  );

  // --- Power Management Unit ---
  neuro_power_manager u_pmu (
    .clk           (clk_sys),     // PMU always runs on system clock
    .rst_n         (rst_n_sync),
    .clk_en_cpu    (clk_en_cpu),
    .clk_en_vpu    (clk_en_vpu),
    .clk_en_mem    (clk_en_mem),
    .clk_en_periph (clk_en_periph),
    .rst_n_cpu     (rst_n_cpu),
    .rst_n_vpu     (rst_n_vpu),
    .rst_n_mem     (rst_n_mem),
    .clk_div_hint  (clk_div_hint),
    .wkup_gpio     (|gpio_in),
    .wkup_uart     (uart_rx),
    .wkup_vpu_done (vpu_done),
    .wkup_timer    (1'b0),        // timer not yet implemented
    .cpu_active    (cpu_active_sig),
    .cpu_halt      (cpu_halt),
    .pmu_reg_addr  (pmu_addr),
    .pmu_reg_wdata (pmu_wdata),
    .pmu_reg_we    (pmu_we),
    .pmu_reg_re    (pmu_re),
    .pmu_reg_rdata (pmu_rdata),
    .pmu_irq       (pmu_irq)
  );

  // ============================================================
  // Debug Output
  // ============================================================
  assign debug_pc        = cpu_debug_pc;
  assign debug_pwr_state = {1'b0, clk_en_cpu, clk_en_vpu};
  assign debug_vpu_busy  = vpu_busy;

endmodule

`default_nettype wire
