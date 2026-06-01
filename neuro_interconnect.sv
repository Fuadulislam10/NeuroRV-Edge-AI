// ============================================================================
// NeuroRV Edge — AXI-lite Interconnect + SRAM Subsystem
// File   : rtl/neuro_interconnect.sv
// Author : NeuroRV Edge Contributors
// License: Apache 2.0
//
// Description:
//   A simple but synthesizable AXI-lite-style bus interconnect that:
//   - Accepts transactions from CPU and VPU (two masters)
//   - Routes to: SRAM (64KB), GPIO/MMIO, VPU control registers
//   - Deterministic priority arbitration: CPU > VPU
//   - Address Map:
//       0x0000_0000 – 0x0000_FFFF : 64KB SRAM (instruction + data)
//       0x1000_0000 – 0x1000_001F : VPU control registers
//       0x2000_0000 – 0x2000_00FF : GPIO / MMIO
//       0x3000_0000 – 0x3000_000F : UART debug
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

// ============================================================================
// 64KB Single-Port SRAM (synthesizable)
// ============================================================================
module sram_64k (
  input  logic        clk,
  input  logic        rst_n,
  // Port A — CPU instruction fetch
  input  logic [13:0] porta_addr,    // word address
  input  logic [31:0] porta_wdata,
  input  logic [3:0]  porta_wstrb,
  input  logic        porta_we,
  input  logic        porta_re,
  output logic [31:0] porta_rdata,
  output logic        porta_ack,
  // Port B — CPU data / VPU access
  input  logic [13:0] portb_addr,
  input  logic [31:0] portb_wdata,
  input  logic [3:0]  portb_wstrb,
  input  logic        portb_we,
  input  logic        portb_re,
  output logic [31:0] portb_rdata,
  output logic        portb_ack
);
  // 16K x 32-bit = 64KB
  (* ram_style = "block" *)
  logic [31:0] mem [0:16383];

  // Initialize to zero (for simulation / FPGA)
  initial begin
    for (int i = 0; i < 16384; i++) mem[i] = 32'h0;
  end

  // Port A: instruction fetch (read-only in typical use)
  always_ff @(posedge clk) begin
    if (porta_re)
      porta_rdata <= mem[porta_addr];
    if (porta_we) begin
      if (porta_wstrb[0]) mem[porta_addr][ 7: 0] <= porta_wdata[ 7: 0];
      if (porta_wstrb[1]) mem[porta_addr][15: 8] <= porta_wdata[15: 8];
      if (porta_wstrb[2]) mem[porta_addr][23:16] <= porta_wdata[23:16];
      if (porta_wstrb[3]) mem[porta_addr][31:24] <= porta_wdata[31:24];
    end
  end
  assign porta_ack = 1'b1; // single-cycle latency

  // Port B: data port
  always_ff @(posedge clk) begin
    if (portb_re)
      portb_rdata <= mem[portb_addr];
    if (portb_we) begin
      if (portb_wstrb[0]) mem[portb_addr][ 7: 0] <= portb_wdata[ 7: 0];
      if (portb_wstrb[1]) mem[portb_addr][15: 8] <= portb_wdata[15: 8];
      if (portb_wstrb[2]) mem[portb_addr][23:16] <= portb_wdata[23:16];
      if (portb_wstrb[3]) mem[portb_addr][31:24] <= portb_wdata[31:24];
    end
  end
  assign portb_ack = 1'b1;
endmodule

// ============================================================================
// UART Transmitter (simple 8N1)
// ============================================================================
module uart_tx #(
  parameter int CLK_FREQ_HZ = 50_000_000,
  parameter int BAUD_RATE   = 115200
)(
  input  logic       clk,
  input  logic       rst_n,
  input  logic [7:0] tx_data,
  input  logic       tx_valid,
  output logic       tx_ready,
  output logic       uart_tx_pin
);
  localparam int CLKS_PER_BIT = CLK_FREQ_HZ / BAUD_RATE;

  typedef enum logic [1:0] {
    UART_IDLE  = 2'b00,
    UART_START = 2'b01,
    UART_DATA  = 2'b10,
    UART_STOP  = 2'b11
  } uart_state_t;

  uart_state_t state;
  logic [$clog2(CLKS_PER_BIT)-1:0] clk_cnt;
  logic [7:0] shift_reg;
  logic [2:0] bit_cnt;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      state       <= UART_IDLE;
      clk_cnt     <= '0;
      bit_cnt     <= 3'h0;
      uart_tx_pin <= 1'b1;
      tx_ready    <= 1'b1;
      shift_reg   <= 8'h0;
    end else begin
      case (state)
        UART_IDLE: begin
          uart_tx_pin <= 1'b1;
          tx_ready    <= 1'b1;
          if (tx_valid) begin
            shift_reg <= tx_data;
            state     <= UART_START;
            tx_ready  <= 1'b0;
            clk_cnt   <= '0;
          end
        end
        UART_START: begin
          uart_tx_pin <= 1'b0; // start bit
          if (clk_cnt == CLKS_PER_BIT - 1) begin
            clk_cnt <= '0;
            bit_cnt <= 3'h0;
            state   <= UART_DATA;
          end else clk_cnt <= clk_cnt + 1;
        end
        UART_DATA: begin
          uart_tx_pin <= shift_reg[0];
          if (clk_cnt == CLKS_PER_BIT - 1) begin
            clk_cnt   <= '0;
            shift_reg <= {1'b0, shift_reg[7:1]};
            if (bit_cnt == 3'h7) state <= UART_STOP;
            else bit_cnt <= bit_cnt + 3'h1;
          end else clk_cnt <= clk_cnt + 1;
        end
        UART_STOP: begin
          uart_tx_pin <= 1'b1; // stop bit
          if (clk_cnt == CLKS_PER_BIT - 1) begin
            clk_cnt <= '0;
            state   <= UART_IDLE;
          end else clk_cnt <= clk_cnt + 1;
        end
        default: state <= UART_IDLE;
      endcase
    end
  end
endmodule

// ============================================================================
// AXI-lite Interconnect Top
// ============================================================================
module neuro_interconnect #(
  parameter int CLK_FREQ_HZ = 50_000_000
)(
  input  logic        clk,
  input  logic        rst_n,

  // === Master 0: CPU Instruction Fetch ===
  input  logic [31:0] cpu_imem_addr,
  output logic [31:0] cpu_imem_data,
  output logic        cpu_imem_valid,

  // === Master 1: CPU Data ===
  input  logic [31:0] cpu_dmem_addr,
  input  logic [31:0] cpu_dmem_wdata,
  input  logic [3:0]  cpu_dmem_wstrb,
  input  logic        cpu_dmem_we,
  input  logic        cpu_dmem_re,
  output logic [31:0] cpu_dmem_rdata,
  output logic        cpu_dmem_ack,

  // === Master 2: VPU Memory ===
  input  logic [31:0] vpu_mem_addr,
  input  logic [31:0] vpu_mem_wdata,
  input  logic        vpu_mem_we,
  input  logic        vpu_mem_re,
  output logic [31:0] vpu_mem_rdata,
  output logic        vpu_mem_ack,

  // === Slave: VPU Control Registers ===
  // (VPU ctrl regs connect back to VPU module directly)
  output logic [31:0] vpu_ctrl_addr,
  output logic [31:0] vpu_ctrl_wdata,
  output logic        vpu_ctrl_we,
  output logic        vpu_ctrl_re,
  input  logic [31:0] vpu_ctrl_rdata,

  // === GPIO ===
  input  logic [31:0] gpio_in,
  output logic [31:0] gpio_out,
  output logic [31:0] gpio_dir,

  // === UART ===
  output logic        uart_tx_pin,
  input  logic        uart_rx_pin
);

  // ---- SRAM port B arbitration: CPU data vs VPU (CPU wins) ----
  logic [13:0] sram_b_addr;
  logic [31:0] sram_b_wdata;
  logic [3:0]  sram_b_wstrb;
  logic        sram_b_we, sram_b_re;
  logic [31:0] sram_b_rdata;
  logic        sram_b_ack;

  // ---- Address decode helpers ----
  // SRAM: 0x0000_0000 – 0x0000_FFFF
  // VPU:  0x1000_0000 – 0x1000_001F
  // GPIO: 0x2000_0000 – 0x2000_00FF
  // UART: 0x3000_0000 – 0x3000_000F
  function automatic logic is_sram(input logic [31:0] addr);
    return (addr[31:16] == 16'h0000);
  endfunction
  function automatic logic is_vpu(input logic [31:0] addr);
    return (addr[31:8] == 24'h100000);
  endfunction
  function automatic logic is_gpio(input logic [31:0] addr);
    return (addr[31:8] == 24'h200000);
  endfunction
  function automatic logic is_uart(input logic [31:0] addr);
    return (addr[31:8] == 24'h300000);
  endfunction

  // ---- SRAM Instantiation ----
  sram_64k u_sram (
    .clk         (clk),
    .rst_n       (rst_n),
    // Port A: instruction fetch
    .porta_addr  (cpu_imem_addr[15:2]),
    .porta_wdata (32'h0),
    .porta_wstrb (4'h0),
    .porta_we    (1'b0),
    .porta_re    (1'b1),
    .porta_rdata (cpu_imem_data),
    .porta_ack   (cpu_imem_valid),
    // Port B: data
    .portb_addr  (sram_b_addr),
    .portb_wdata (sram_b_wdata),
    .portb_wstrb (sram_b_wstrb),
    .portb_we    (sram_b_we),
    .portb_re    (sram_b_re),
    .portb_rdata (sram_b_rdata),
    .portb_ack   (sram_b_ack)
  );

  // ---- UART TX ----
  logic [7:0] uart_tx_data;
  logic       uart_tx_valid;
  logic       uart_tx_ready;

  uart_tx #(
    .CLK_FREQ_HZ(CLK_FREQ_HZ),
    .BAUD_RATE  (115200)
  ) u_uart (
    .clk        (clk),
    .rst_n      (rst_n),
    .tx_data    (uart_tx_data),
    .tx_valid   (uart_tx_valid),
    .tx_ready   (uart_tx_ready),
    .uart_tx_pin(uart_tx_pin)
  );

  // ---- GPIO registers ----
  logic [31:0] gpio_out_r;
  logic [31:0] gpio_dir_r;
  assign gpio_out = gpio_out_r;
  assign gpio_dir = gpio_dir_r;

  // ========== ARBITRATION: CPU DATA vs VPU ==========
  // CPU has priority. VPU gets access when CPU is idle on data bus.
  logic cpu_data_sram = is_sram(cpu_dmem_addr) && (cpu_dmem_we || cpu_dmem_re);
  logic vpu_wants_sram = is_sram(vpu_mem_addr) && (vpu_mem_we || vpu_mem_re);

  // Grant logic
  logic grant_cpu_data;
  assign grant_cpu_data = cpu_data_sram;

  always_comb begin
    // Defaults
    sram_b_addr  = 14'h0;
    sram_b_wdata = 32'h0;
    sram_b_wstrb = 4'h0;
    sram_b_we    = 1'b0;
    sram_b_re    = 1'b0;

    cpu_dmem_rdata = 32'h0;
    cpu_dmem_ack   = 1'b0;
    vpu_mem_rdata  = 32'h0;
    vpu_mem_ack    = 1'b0;

    // VPU control register passthrough
    vpu_ctrl_addr  = cpu_dmem_addr;
    vpu_ctrl_wdata = cpu_dmem_wdata;
    vpu_ctrl_we    = 1'b0;
    vpu_ctrl_re    = 1'b0;

    uart_tx_data  = 8'h0;
    uart_tx_valid = 1'b0;

    if (cpu_dmem_we || cpu_dmem_re) begin
      // CPU data access
      if (is_sram(cpu_dmem_addr)) begin
        sram_b_addr  = cpu_dmem_addr[15:2];
        sram_b_wdata = cpu_dmem_wdata;
        sram_b_wstrb = cpu_dmem_wstrb;
        sram_b_we    = cpu_dmem_we;
        sram_b_re    = cpu_dmem_re;
        cpu_dmem_rdata = sram_b_rdata;
        cpu_dmem_ack   = sram_b_ack;
      end else if (is_vpu(cpu_dmem_addr)) begin
        vpu_ctrl_we    = cpu_dmem_we;
        vpu_ctrl_re    = cpu_dmem_re;
        cpu_dmem_rdata = vpu_ctrl_rdata;
        cpu_dmem_ack   = 1'b1;
      end else if (is_gpio(cpu_dmem_addr)) begin
        cpu_dmem_rdata = (cpu_dmem_addr[3:0] == 4'h0) ? gpio_in :
                         (cpu_dmem_addr[3:0] == 4'h4) ? gpio_out_r :
                          gpio_dir_r;
        cpu_dmem_ack   = 1'b1;
      end else if (is_uart(cpu_dmem_addr)) begin
        uart_tx_data  = cpu_dmem_wdata[7:0];
        uart_tx_valid = cpu_dmem_we;
        cpu_dmem_rdata = {31'h0, uart_tx_ready};
        cpu_dmem_ack   = 1'b1;
      end else begin
        cpu_dmem_rdata = 32'hDEAD_BEEF; // unmapped
        cpu_dmem_ack   = 1'b1;
      end
    end else if (vpu_wants_sram && !cpu_data_sram) begin
      // VPU data access (only when CPU is not using SRAM data port)
      sram_b_addr  = vpu_mem_addr[15:2];
      sram_b_wdata = vpu_mem_wdata;
      sram_b_wstrb = 4'hF;
      sram_b_we    = vpu_mem_we;
      sram_b_re    = vpu_mem_re;
      vpu_mem_rdata = sram_b_rdata;
      vpu_mem_ack   = sram_b_ack;
    end
  end

  // ---- GPIO register writes ----
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      gpio_out_r <= 32'h0;
      gpio_dir_r <= 32'h0;
    end else if (cpu_dmem_we && is_gpio(cpu_dmem_addr)) begin
      case (cpu_dmem_addr[3:0])
        4'h4: gpio_out_r <= cpu_dmem_wdata;
        4'h8: gpio_dir_r <= cpu_dmem_wdata;
        default: ;
      endcase
    end
  end

endmodule

`default_nettype wire
