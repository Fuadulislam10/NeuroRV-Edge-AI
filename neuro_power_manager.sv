// ============================================================================
// NeuroRV Edge — Power Management Unit (PMU)
// File   : rtl/neuro_power_manager.sv
// Author : NeuroRV Edge Contributors
// License: Apache 2.0
//
// Description:
//   Manages SoC power states with a 4-level power hierarchy:
//   - ACTIVE:     Full speed, all clocks enabled
//   - IDLE:       CPU halted, VPU & memory clocked (inference running)
//   - SLEEP:      CPU + VPU halted, memory retained, slow clock
//   - DEEP_SLEEP: All functional clocks gated, only RTC & wake logic active
//
//   Features:
//   - Clock gating enable per domain
//   - DVFS hints (clock divider output)
//   - Wake-up interrupt sources: GPIO, UART, VPU done, timer
//   - Configurable idle timeout → auto sleep transition
//   - Memory-mapped PMU control registers at 0x4000_0000
//
// Register Map:
//   0x00: PMU_CTRL   [RW] - [1:0] target_state, [2] force_transition, [3] wakeup_en
//   0x04: PMU_STATUS [RO] - [1:0] current_state, [2] transitioning
//   0x08: PMU_WAKEUP [RW] - [3:0] wakeup source enable mask
//   0x0C: PMU_TIMEOUT[RW] - idle→sleep countdown (cycles)
//   0x10: PMU_CLK_DIV[RW] - [3:0] clock divider ratio hint
//   0x14: PMU_IRQ    [RO] - [3:0] pending wakeup sources (write to clear)
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

module neuro_power_manager (
  input  logic        clk,         // System clock (always on)
  input  logic        rst_n,

  // --- Clock gate outputs (active-high enable to clock buffers) ---
  output logic        clk_en_cpu,
  output logic        clk_en_vpu,
  output logic        clk_en_mem,
  output logic        clk_en_periph,

  // --- Reset outputs ---
  output logic        rst_n_cpu,
  output logic        rst_n_vpu,
  output logic        rst_n_mem,

  // --- DVFS hint ---
  output logic [3:0]  clk_div_hint,  // 0=full, 1=div2, 2=div4, 3=div8

  // --- Wakeup sources ---
  input  logic        wkup_gpio,
  input  logic        wkup_uart,
  input  logic        wkup_vpu_done,
  input  logic        wkup_timer,

  // --- CPU interface (halt request) ---
  input  logic        cpu_active,   // CPU is executing
  output logic        cpu_halt,     // request CPU to halt (WFI)

  // --- PMU register interface ---
  input  logic [31:0] pmu_reg_addr,
  input  logic [31:0] pmu_reg_wdata,
  input  logic        pmu_reg_we,
  input  logic        pmu_reg_re,
  output logic [31:0] pmu_reg_rdata,

  // --- Interrupt output ---
  output logic        pmu_irq
);

  // ---- Power State Encoding ----
  typedef enum logic [1:0] {
    PWR_ACTIVE     = 2'b00,
    PWR_IDLE       = 2'b01,
    PWR_SLEEP      = 2'b10,
    PWR_DEEP_SLEEP = 2'b11
  } pwr_state_t;

  pwr_state_t  current_state;
  pwr_state_t  target_state;
  pwr_state_t  next_state;
  logic        transitioning;
  logic        force_transition;
  logic        wakeup_en;

  // ---- PMU Configuration Registers ----
  logic [3:0]  wakeup_mask;        // which sources can wake
  logic [31:0] idle_timeout;       // cycles before auto-sleep
  logic [3:0]  clk_div_cfg;        // DVFS divider setting
  logic [3:0]  wakeup_pending;     // pending wakeup flags

  // ---- Idle / Timeout Counter ----
  logic [31:0] idle_cnt;
  logic        timeout_reached;

  // ---- Wakeup event detection (edge detect) ----
  logic        wkup_gpio_r,  wkup_uart_r,  wkup_vpu_r,  wkup_timer_r;
  logic        wkup_gpio_edge, wkup_uart_edge, wkup_vpu_edge, wkup_timer_edge;

  // ---- Transition FSM States ----
  typedef enum logic [2:0] {
    TRANS_IDLE    = 3'h0,
    TRANS_GATE_CPU= 3'h1,
    TRANS_GATE_VPU= 3'h2,
    TRANS_GATE_ALL= 3'h3,
    TRANS_WAKE    = 3'h4,
    TRANS_UNGATE  = 3'h5,
    TRANS_DONE    = 3'h6
  } trans_state_t;

  trans_state_t trans_state;
  logic [7:0]  trans_cnt;   // transition settle counter

  // ==========================================================================
  // Wakeup Edge Detection
  // ==========================================================================
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      wkup_gpio_r  <= 1'b0;
      wkup_uart_r  <= 1'b0;
      wkup_vpu_r   <= 1'b0;
      wkup_timer_r <= 1'b0;
    end else begin
      wkup_gpio_r  <= wkup_gpio;
      wkup_uart_r  <= wkup_uart;
      wkup_vpu_r   <= wkup_vpu_done;
      wkup_timer_r <= wkup_timer;
    end
  end

  assign wkup_gpio_edge  = wkup_gpio  & ~wkup_gpio_r;
  assign wkup_uart_edge  = wkup_uart  & ~wkup_uart_r;
  assign wkup_vpu_edge   = wkup_vpu_done & ~wkup_vpu_r;
  assign wkup_timer_edge = wkup_timer & ~wkup_timer_r;

  // Combined wakeup trigger
  logic wakeup_trigger;
  assign wakeup_trigger = wakeup_en &&
    ((wkup_gpio_edge  & wakeup_mask[0]) |
     (wkup_uart_edge  & wakeup_mask[1]) |
     (wkup_vpu_edge   & wakeup_mask[2]) |
     (wkup_timer_edge & wakeup_mask[3]));

  // ==========================================================================
  // PMU Register Interface
  // ==========================================================================
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      target_state     <= PWR_ACTIVE;
      force_transition <= 1'b0;
      wakeup_en        <= 1'b1;
      wakeup_mask      <= 4'hF;
      idle_timeout     <= 32'd1000;
      clk_div_cfg      <= 4'h0;
      wakeup_pending   <= 4'h0;
    end else begin
      force_transition <= 1'b0; // auto-clear

      // Latch wakeup events
      if (wkup_gpio_edge)  wakeup_pending[0] <= 1'b1;
      if (wkup_uart_edge)  wakeup_pending[1] <= 1'b1;
      if (wkup_vpu_edge)   wakeup_pending[2] <= 1'b1;
      if (wkup_timer_edge) wakeup_pending[3] <= 1'b1;

      if (pmu_reg_we) begin
        case (pmu_reg_addr[4:0])
          5'h00: begin
            target_state     <= pwr_state_t'(pmu_reg_wdata[1:0]);
            force_transition <= pmu_reg_wdata[2];
            wakeup_en        <= pmu_reg_wdata[3];
          end
          5'h08: wakeup_mask  <= pmu_reg_wdata[3:0];
          5'h0C: idle_timeout <= pmu_reg_wdata;
          5'h10: clk_div_cfg  <= pmu_reg_wdata[3:0];
          5'h14: wakeup_pending <= wakeup_pending & ~pmu_reg_wdata[3:0]; // W1C
          default: ;
        endcase
      end
    end
  end

  always_comb begin
    pmu_reg_rdata = 32'h0;
    if (pmu_reg_re) begin
      case (pmu_reg_addr[4:0])
        5'h00: pmu_reg_rdata = {28'h0, wakeup_en, force_transition, target_state};
        5'h04: pmu_reg_rdata = {29'h0, transitioning, current_state};
        5'h08: pmu_reg_rdata = {28'h0, wakeup_mask};
        5'h0C: pmu_reg_rdata = idle_timeout;
        5'h10: pmu_reg_rdata = {28'h0, clk_div_cfg};
        5'h14: pmu_reg_rdata = {28'h0, wakeup_pending};
        default: pmu_reg_rdata = 32'h0;
      endcase
    end
  end

  // ==========================================================================
  // Idle Timeout Counter
  // ==========================================================================
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      idle_cnt <= 32'h0;
    end else begin
      if (cpu_active || current_state != PWR_ACTIVE)
        idle_cnt <= 32'h0;
      else
        idle_cnt <= idle_cnt + 32'h1;
    end
  end

  assign timeout_reached = (idle_cnt >= idle_timeout) && (idle_timeout != 32'h0);

  // ==========================================================================
  // Power State Machine
  // ==========================================================================
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      current_state  <= PWR_ACTIVE;
      trans_state    <= TRANS_IDLE;
      transitioning  <= 1'b0;
      trans_cnt      <= 8'h0;
      clk_en_cpu     <= 1'b1;
      clk_en_vpu     <= 1'b1;
      clk_en_mem     <= 1'b1;
      clk_en_periph  <= 1'b1;
      rst_n_cpu      <= 1'b1;
      rst_n_vpu      <= 1'b1;
      rst_n_mem      <= 1'b1;
      cpu_halt       <= 1'b0;
      clk_div_hint   <= 4'h0;
      pmu_irq        <= 1'b0;
    end else begin
      pmu_irq <= 1'b0;

      case (trans_state)
        // ---- Steady State ----
        TRANS_IDLE: begin
          transitioning <= 1'b0;
          trans_cnt     <= 8'h0;

          // Determine if we need to transition
          if (wakeup_trigger && current_state != PWR_ACTIVE) begin
            trans_state <= TRANS_WAKE;
            transitioning <= 1'b1;
          end else if ((force_transition && target_state != current_state) ||
                       (timeout_reached && current_state == PWR_ACTIVE)) begin
            transitioning <= 1'b1;
            case (current_state)
              PWR_ACTIVE: begin
                // Going to lower power
                cpu_halt    <= 1'b1;
                trans_state <= TRANS_GATE_CPU;
              end
              PWR_IDLE: begin
                trans_state <= TRANS_GATE_VPU;
              end
              default: trans_state <= TRANS_IDLE;
            endcase
          end
        end

        // ---- Gate CPU ----
        TRANS_GATE_CPU: begin
          if (trans_cnt == 8'd4) begin
            clk_en_cpu  <= 1'b0;
            rst_n_cpu   <= 1'b0;
            current_state <= PWR_IDLE;
            clk_div_hint <= 4'h1;
            if (target_state == PWR_SLEEP || target_state == PWR_DEEP_SLEEP)
              trans_state <= TRANS_GATE_VPU;
            else begin
              trans_state <= TRANS_DONE;
            end
            trans_cnt   <= 8'h0;
          end else trans_cnt <= trans_cnt + 8'h1;
        end

        // ---- Gate VPU ----
        TRANS_GATE_VPU: begin
          if (trans_cnt == 8'd4) begin
            clk_en_vpu  <= 1'b0;
            rst_n_vpu   <= 1'b0;
            current_state <= PWR_SLEEP;
            clk_div_hint <= 4'h2;
            if (target_state == PWR_DEEP_SLEEP)
              trans_state <= TRANS_GATE_ALL;
            else begin
              trans_state <= TRANS_DONE;
            end
            trans_cnt <= 8'h0;
          end else trans_cnt <= trans_cnt + 8'h1;
        end

        // ---- Gate All (Deep Sleep) ----
        TRANS_GATE_ALL: begin
          if (trans_cnt == 8'd8) begin
            clk_en_mem    <= 1'b0;
            clk_en_periph <= 1'b0;
            current_state  <= PWR_DEEP_SLEEP;
            clk_div_hint   <= 4'h3;
            trans_state    <= TRANS_DONE;
            trans_cnt      <= 8'h0;
          end else trans_cnt <= trans_cnt + 8'h1;
        end

        // ---- Wakeup ----
        TRANS_WAKE: begin
          // Re-enable clocks in reverse order
          clk_en_mem    <= 1'b1;
          clk_en_periph <= 1'b1;
          if (trans_cnt == 8'd4) begin
            clk_en_vpu  <= 1'b1;
            rst_n_vpu   <= 1'b1;
          end
          if (trans_cnt == 8'd8) begin
            clk_en_cpu  <= 1'b1;
            rst_n_cpu   <= 1'b1;
            cpu_halt    <= 1'b0;
            clk_div_hint <= 4'h0;
          end
          if (trans_cnt == 8'd12) begin
            current_state <= PWR_ACTIVE;
            trans_state   <= TRANS_DONE;
            trans_cnt     <= 8'h0;
          end else trans_cnt <= trans_cnt + 8'h1;
        end

        // ---- Transition Complete ----
        TRANS_DONE: begin
          transitioning <= 1'b0;
          pmu_irq       <= 1'b1;  // signal transition complete
          trans_state   <= TRANS_IDLE;
        end

        default: trans_state <= TRANS_IDLE;
      endcase
    end
  end

endmodule

`default_nettype wire
