// ============================================================================
// FILE: rtl/top/neurorv_soc.sv
// PROJECT: NeuroRV Edge — Phase 7 SoC Top Integration
// TAGLINE: Hybrid RISC-V AI Accelerator SoC for Ultra-Low-Power Edge Intelligence
//
// DESCRIPTION:
//   Top-level integration module for the NeuroRV Edge SoC.
//   Instantiates and connects all subsystems:
//     - rv32im_core        : RISC-V RV32IM CPU
//     - vxu_top            : Vector Execution Unit (AI accelerator)
//     - unified_sram       : 512KB shared SRAM (dual-port)
//     - axi_interconnect   : AXI-lite style 3-master/1-slave bus fabric
//     - dma_controller     : Memory-mapped burst DMA engine
//     - uart_16550         : UART peripheral
//     - spi_master         : SPI peripheral
//     - i2c_master         : I2C peripheral
//     - gpio_ctrl          : GPIO controller
//     - timer_unit         : System timer
//     - pmu_top            : Power management unit
//
// ============================================================================
// MEMORY MAP
//   0x0000_0000 – 0x0007_FFFF  Unified SRAM (512 KB)
//   0x1000_1000 – 0x1000_1FFF  UART 16550
//   0x1000_2000 – 0x1000_2FFF  SPI Master
//   0x1000_3000 – 0x1000_3FFF  I2C Master
//   0x1000_4000 – 0x1000_4FFF  GPIO Controller
//   0x1000_5000 – 0x1000_5FFF  Timer Unit
//   0x2000_0000 – 0x2000_00FF  VXU Control Registers
//   0x3000_0000 – 0x3000_00FF  DMA Controller Registers
//   0x4000_0000 – 0x4000_00FF  PMU Registers
// ============================================================================
// INTERRUPT PRIORITY (highest → lowest)
//   1. DMA completion
//   2. VXU completion
//   3. Timer
//   4. UART RX
//   5. GPIO
// ============================================================================
// COMPATIBLE: Verilator | Yosys | FPGA | OpenROAD ASIC flow
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

module neurorv_soc #(
    // -------------------------------------------------------------------------
    // Global bus parameters
    // -------------------------------------------------------------------------
    parameter int ADDR_W        = 32,
    parameter int DATA_W        = 32,

    // -------------------------------------------------------------------------
    // SRAM parameters
    // -------------------------------------------------------------------------
    parameter int SRAM_SIZE_BYTES = 512 * 1024,    // 512 KB
    parameter int PARITY_EN       = 1,

    // -------------------------------------------------------------------------
    // VXU parameters
    // -------------------------------------------------------------------------
    parameter int VXU_VEC_LEN  = 256,
    parameter int VXU_DATA_W   = 16,
    parameter int VXU_ACCUM_W  = 40,
    parameter int VXU_DMA_BURST= 16,

    // -------------------------------------------------------------------------
    // GPIO width
    // -------------------------------------------------------------------------
    parameter int GPIO_W       = 32,

    // -------------------------------------------------------------------------
    // Reset vector for CPU
    // -------------------------------------------------------------------------
    parameter logic [31:0] RESET_VECTOR = 32'h0000_0000
)(
    // -------------------------------------------------------------------------
    // Primary I/O
    // -------------------------------------------------------------------------
    input  logic        clk_i,
    input  logic        rst_n_i,

    // UART
    input  logic        uart_rx_i,
    output logic        uart_tx_o,

    // SPI
    output logic        spi_sclk_o,
    output logic        spi_mosi_o,
    input  logic        spi_miso_i,
    output logic        spi_cs_n_o,

    // I2C
    inout  wire         i2c_sda_io,
    inout  wire         i2c_scl_io,

    // GPIO
    inout  wire [GPIO_W-1:0] gpio_io,

    // Debug / Observability outputs
    output logic [31:0] dbg_cycle_count_o,
    output logic [31:0] dbg_instr_count_o,
    output logic [31:0] dbg_dma_txn_count_o,
    output logic [31:0] dbg_vxu_op_count_o,
    output logic [31:0] dbg_irq_count_o
);

    // =========================================================================
    // SECTION 1: CLOCK AND RESET DISTRIBUTION
    // All clocks derived from clk_i. PMU gates clocks per domain.
    // =========================================================================

    // PMU-gated clock enables (registered to avoid glitches)
    logic pmu_clk_en_cpu;
    logic pmu_clk_en_peri;
    logic pmu_clk_en_vxu;
    logic pmu_clk_en_dma;

    // Gated clocks implemented as clock enable (ASIC/FPGA safe)
    // All FFs use clk_i + CE internally. Exposed here for structural clarity.
    logic clk_cpu, clk_peri, clk_vxu, clk_dma;
    assign clk_cpu  = clk_i;   // CPU always on; gating via PMU CE in CPU core
    assign clk_peri = clk_i;
    assign clk_vxu  = clk_i;
    assign clk_dma  = clk_i;

    // =========================================================================
    // SECTION 2: RESET SYNCHRONIZER + SUBSYSTEM RESETS
    // Two-stage synchronizer per domain to prevent metastability.
    // PMU controls release order: PMU → SRAM → DMA → VXU → PERI → CPU
    // =========================================================================

    // PMU reset outputs
    logic pmu_rst_n_cpu;
    logic pmu_rst_n_vxu;
    logic pmu_rst_n_peri;
    logic pmu_rst_n_dma;
    logic pmu_rst_n_sram;
    logic pmu_rst_n_axi;

    // Two-stage synchronizer macro (inline for portability)
    // sync_rst_n = synchronized, active-low reset for each domain
    logic [1:0] sync_cpu_r,  sync_vxu_r,  sync_peri_r;
    logic [1:0] sync_dma_r,  sync_sram_r, sync_axi_r;

    always_ff @(posedge clk_i or negedge pmu_rst_n_cpu)
        if (!pmu_rst_n_cpu) sync_cpu_r  <= 2'b00; else sync_cpu_r  <= {sync_cpu_r[0],  1'b1};
    always_ff @(posedge clk_i or negedge pmu_rst_n_vxu)
        if (!pmu_rst_n_vxu) sync_vxu_r  <= 2'b00; else sync_vxu_r  <= {sync_vxu_r[0],  1'b1};
    always_ff @(posedge clk_i or negedge pmu_rst_n_peri)
        if (!pmu_rst_n_peri) sync_peri_r <= 2'b00; else sync_peri_r <= {sync_peri_r[0], 1'b1};
    always_ff @(posedge clk_i or negedge pmu_rst_n_dma)
        if (!pmu_rst_n_dma) sync_dma_r  <= 2'b00; else sync_dma_r  <= {sync_dma_r[0],  1'b1};
    always_ff @(posedge clk_i or negedge pmu_rst_n_sram)
        if (!pmu_rst_n_sram) sync_sram_r <= 2'b00; else sync_sram_r <= {sync_sram_r[0], 1'b1};
    always_ff @(posedge clk_i or negedge pmu_rst_n_axi)
        if (!pmu_rst_n_axi) sync_axi_r  <= 2'b00; else sync_axi_r  <= {sync_axi_r[0],  1'b1};

    logic rst_n_cpu,  rst_n_vxu,  rst_n_peri;
    logic rst_n_dma,  rst_n_sram, rst_n_axi;
    assign rst_n_cpu  = sync_cpu_r[1];
    assign rst_n_vxu  = sync_vxu_r[1];
    assign rst_n_peri = sync_peri_r[1];
    assign rst_n_dma  = sync_dma_r[1];
    assign rst_n_sram = sync_sram_r[1];
    assign rst_n_axi  = sync_axi_r[1];

    // =========================================================================
    // SECTION 3: INTERRUPT AGGREGATION
    // Priority: DMA(4) > VXU(3) > TIMER(2) > UART(1) > GPIO(0)
    // =========================================================================

    logic irq_dma, irq_vxu, irq_timer, irq_uart, irq_gpio;
    logic cpu_irq;

    // Priority-encoded interrupt to CPU (level, CPU must clear)
    assign cpu_irq = irq_dma | irq_vxu | irq_timer | irq_uart | irq_gpio;

    // Interrupt cause vector (for CPU interrupt controller / CLINT/PLIC stub)
    logic [4:0] irq_vec;
    assign irq_vec = {irq_dma, irq_vxu, irq_timer, irq_uart, irq_gpio};

    // IRQ counter (debug)
    logic [31:0] irq_count_r;
    always_ff @(posedge clk_i or negedge rst_n_i)
        if (!rst_n_i) irq_count_r <= '0;
        else if (cpu_irq) irq_count_r <= irq_count_r + 1;
    assign dbg_irq_count_o = irq_count_r;

    // =========================================================================
    // SECTION 4: PERIPHERAL BUS (APB-style, simplified)
    // Address-decoded from CPU load/store address.
    // Peripheral range: 0x1000_0000 – 0x1000_FFFF
    // =========================================================================

    // CPU bus interface signals (generated by rv32im_core)
    logic        cpu_mem_req;
    logic        cpu_mem_wr;
    logic [31:0] cpu_mem_addr;
    logic [31:0] cpu_mem_wdata;
    logic [3:0]  cpu_mem_be;
    logic [31:0] cpu_mem_rdata;
    logic        cpu_mem_ack;

    // Peripheral select signals (one-hot based on address decode)
    logic sel_sram, sel_uart, sel_spi, sel_i2c, sel_gpio;
    logic sel_timer, sel_vxu_cfg, sel_dma_cfg, sel_pmu_cfg;

    always_comb begin
        sel_sram    = (cpu_mem_addr[31:20] == 12'h000);             // 0x0000_0000–0x000F_FFFF (covers 512K)
        sel_uart    = (cpu_mem_addr[31:12] == 20'h10001);           // 0x1000_1xxx
        sel_spi     = (cpu_mem_addr[31:12] == 20'h10002);           // 0x1000_2xxx
        sel_i2c     = (cpu_mem_addr[31:12] == 20'h10003);           // 0x1000_3xxx
        sel_gpio    = (cpu_mem_addr[31:12] == 20'h10004);           // 0x1000_4xxx
        sel_timer   = (cpu_mem_addr[31:12] == 20'h10005);           // 0x1000_5xxx
        sel_vxu_cfg = (cpu_mem_addr[31:8]  == 24'h200000);          // 0x2000_00xx
        sel_dma_cfg = (cpu_mem_addr[31:8]  == 24'h300000);          // 0x3000_00xx
        sel_pmu_cfg = (cpu_mem_addr[31:8]  == 24'h400000);          // 0x4000_00xx
    end

    // Peripheral read-data mux back to CPU
    logic [31:0] uart_rdata, spi_rdata, i2c_rdata, gpio_rdata;
    logic [31:0] timer_rdata, pmu_rdata;
    logic [31:0] vxu_cfg_rdata_32;
    logic [31:0] dma_rdata;

    // cpu_mem_ack mux: peripherals ack in 1 cycle
    logic uart_ack, spi_ack, i2c_ack, gpio_ack, timer_ack, pmu_ack;
    logic vxu_cfg_ack, dma_cfg_ack;
    logic axi_cpu_ack;

    always_comb begin
        cpu_mem_rdata = 32'h0000_0000;
        cpu_mem_ack   = 1'b0;
        if (sel_sram)    begin cpu_mem_rdata = '0;           cpu_mem_ack = axi_cpu_ack;   end
        if (sel_uart)    begin cpu_mem_rdata = uart_rdata;   cpu_mem_ack = uart_ack;       end
        if (sel_spi)     begin cpu_mem_rdata = spi_rdata;    cpu_mem_ack = spi_ack;        end
        if (sel_i2c)     begin cpu_mem_rdata = i2c_rdata;    cpu_mem_ack = i2c_ack;        end
        if (sel_gpio)    begin cpu_mem_rdata = gpio_rdata;   cpu_mem_ack = gpio_ack;       end
        if (sel_timer)   begin cpu_mem_rdata = timer_rdata;  cpu_mem_ack = timer_ack;      end
        if (sel_vxu_cfg) begin cpu_mem_rdata = vxu_cfg_rdata_32; cpu_mem_ack = vxu_cfg_ack; end
        if (sel_dma_cfg) begin cpu_mem_rdata = dma_rdata;    cpu_mem_ack = dma_cfg_ack;   end
        if (sel_pmu_cfg) begin cpu_mem_rdata = pmu_rdata;    cpu_mem_ack = pmu_ack;        end
    end

    // =========================================================================
    // SECTION 5: AXI INTERCONNECT SIGNALS
    // M0=CPU, M1=VXU, M2=DMA  →  S0=SRAM
    // =========================================================================

    // --- CPU AXI master (M0) ---
    logic        m0_aw_valid, m0_aw_ready;
    logic [31:0] m0_aw_addr;
    logic [2:0]  m0_aw_prot;
    logic        m0_w_valid,  m0_w_ready;
    logic [31:0] m0_w_data;
    logic [3:0]  m0_w_strb;
    logic        m0_b_valid,  m0_b_ready;
    logic [1:0]  m0_b_resp;
    logic        m0_ar_valid, m0_ar_ready;
    logic [31:0] m0_ar_addr;
    logic [2:0]  m0_ar_prot;
    logic        m0_r_valid,  m0_r_ready;
    logic [31:0] m0_r_data;
    logic [1:0]  m0_r_resp;

    // --- VXU AXI master (M1) ---
    logic        m1_aw_valid, m1_aw_ready;
    logic [31:0] m1_aw_addr;
    logic [2:0]  m1_aw_prot;
    logic        m1_w_valid,  m1_w_ready;
    logic [31:0] m1_w_data;
    logic [3:0]  m1_w_strb;
    logic        m1_b_valid,  m1_b_ready;
    logic [1:0]  m1_b_resp;
    logic        m1_ar_valid, m1_ar_ready;
    logic [31:0] m1_ar_addr;
    logic [2:0]  m1_ar_prot;
    logic        m1_r_valid,  m1_r_ready;
    logic [31:0] m1_r_data;
    logic [1:0]  m1_r_resp;

    // --- DMA AXI master (M2) ---
    logic        m2_aw_valid, m2_aw_ready;
    logic [31:0] m2_aw_addr;
    logic [2:0]  m2_aw_prot;
    logic        m2_w_valid,  m2_w_ready;
    logic [31:0] m2_w_data;
    logic [3:0]  m2_w_strb;
    logic        m2_b_valid,  m2_b_ready;
    logic [1:0]  m2_b_resp;
    logic        m2_ar_valid, m2_ar_ready;
    logic [31:0] m2_ar_addr;
    logic [2:0]  m2_ar_prot;
    logic        m2_r_valid,  m2_r_ready;
    logic [31:0] m2_r_data;
    logic [1:0]  m2_r_resp;

    // --- SRAM AXI slave (S0) ---
    logic        s0_aw_valid, s0_aw_ready;
    logic [31:0] s0_aw_addr;
    logic [2:0]  s0_aw_prot;
    logic        s0_w_valid,  s0_w_ready;
    logic [31:0] s0_w_data;
    logic [3:0]  s0_w_strb;
    logic        s0_b_valid,  s0_b_ready;
    logic [1:0]  s0_b_resp;
    logic        s0_ar_valid, s0_ar_ready;
    logic [31:0] s0_ar_addr;
    logic [2:0]  s0_ar_prot;
    logic        s0_r_valid,  s0_r_ready;
    logic [31:0] s0_r_data;
    logic [1:0]  s0_r_resp;

    // AXI interconnect debug
    logic [31:0] axi_dbg_grant_count [0:2];
    logic [31:0] axi_dbg_stall_count [0:2];
    logic [31:0] axi_dbg_rd_txn, axi_dbg_wr_txn, axi_dbg_unmapped;

    // =========================================================================
    // SECTION 6: SRAM BRIDGE SIGNALS
    // AXI slave → SRAM dual-port adapter (combinational)
    // Port A = primary (from AXI S0), Port B = unused (tied off)
    // =========================================================================

    localparam int SRAM_WORDS   = SRAM_SIZE_BYTES / 4;
    localparam int SRAM_ADDR_W  = $clog2(SRAM_WORDS);

    logic                  sram_pa_req,  sram_pa_wr;
    logic [SRAM_ADDR_W-1:0] sram_pa_addr;
    logic [31:0]           sram_pa_wdata;
    logic [3:0]            sram_pa_be;
    logic [31:0]           sram_pa_rdata;
    logic                  sram_pa_ack,  sram_pa_stall;

    logic [3:0]  sram_pa_parity_err, sram_pb_parity_err;
    logic [31:0] sram_dbg_pa_txn, sram_dbg_pb_txn;
    logic [31:0] sram_dbg_collision, sram_dbg_pb_stall;

    // AXI-to-SRAM bridge (simplified: AXI-lite → single-cycle SRAM)
    // Write path: capture AW+W, drive SRAM, return B
    // Read  path: capture AR, drive SRAM, return R next cycle
    logic        axi_sram_wr_active;
    logic [31:0] axi_sram_wr_addr_r;
    logic [31:0] axi_sram_wr_data_r;
    logic [3:0]  axi_sram_wr_strb_r;

    // Write channel state
    always_ff @(posedge clk_i or negedge rst_n_sram) begin
        if (!rst_n_sram) begin
            axi_sram_wr_active  <= 1'b0;
            axi_sram_wr_addr_r  <= '0;
            axi_sram_wr_data_r  <= '0;
            axi_sram_wr_strb_r  <= '0;
            s0_aw_ready         <= 1'b0;
            s0_w_ready          <= 1'b0;
            s0_b_valid          <= 1'b0;
            s0_b_resp           <= 2'b00;
        end else begin
            s0_aw_ready <= 1'b0;
            s0_w_ready  <= 1'b0;
            s0_b_valid  <= 1'b0;

            if (!axi_sram_wr_active) begin
                if (s0_aw_valid && s0_w_valid) begin
                    // Both address and data available: single-beat capture
                    axi_sram_wr_addr_r  <= s0_aw_addr;
                    axi_sram_wr_data_r  <= s0_w_data;
                    axi_sram_wr_strb_r  <= s0_w_strb;
                    axi_sram_wr_active  <= 1'b1;
                    s0_aw_ready         <= 1'b1;
                    s0_w_ready          <= 1'b1;
                end
            end else begin
                // SRAM write issued last cycle; return B response
                s0_b_valid         <= 1'b1;
                s0_b_resp          <= 2'b00;  // OKAY
                axi_sram_wr_active <= 1'b0;
            end
        end
    end

    // Read channel state
    logic        axi_sram_rd_pending;
    logic [31:0] axi_sram_rd_addr_r;

    always_ff @(posedge clk_i or negedge rst_n_sram) begin
        if (!rst_n_sram) begin
            axi_sram_rd_pending <= 1'b0;
            axi_sram_rd_addr_r  <= '0;
            s0_ar_ready         <= 1'b0;
            s0_r_valid          <= 1'b0;
            s0_r_data           <= '0;
            s0_r_resp           <= 2'b00;
        end else begin
            s0_ar_ready <= 1'b0;
            s0_r_valid  <= 1'b0;

            if (!axi_sram_rd_pending) begin
                if (s0_ar_valid) begin
                    axi_sram_rd_addr_r  <= s0_ar_addr;
                    axi_sram_rd_pending <= 1'b1;
                    s0_ar_ready         <= 1'b1;
                end
            end else begin
                // SRAM read result available (1 cycle latency)
                s0_r_valid          <= 1'b1;
                s0_r_data           <= sram_pa_rdata;
                s0_r_resp           <= 2'b00;
                axi_sram_rd_pending <= 1'b0;
            end
        end
    end

    // SRAM port-A drive: write takes priority over read
    assign sram_pa_req   = axi_sram_wr_active | (s0_ar_valid & !axi_sram_wr_active);
    assign sram_pa_wr    = axi_sram_wr_active;
    assign sram_pa_addr  = axi_sram_wr_active ?
                           axi_sram_wr_addr_r[SRAM_ADDR_W+1:2] :
                           s0_ar_addr[SRAM_ADDR_W+1:2];
    assign sram_pa_wdata = axi_sram_wr_data_r;
    assign sram_pa_be    = axi_sram_wr_strb_r;

    // CPU AXI ack for peripheral mux
    assign axi_cpu_ack   = m0_r_valid | m0_b_valid;

    // =========================================================================
    // SECTION 7: CPU → AXI BRIDGE
    // Converts rv32im_core load/store bus → AXI-lite M0
    // =========================================================================

    // CPU→AXI write bridge state
    logic cpu_axi_wr_busy;
    always_ff @(posedge clk_i or negedge rst_n_cpu) begin
        if (!rst_n_cpu) begin
            m0_aw_valid  <= 1'b0;
            m0_aw_addr   <= '0;
            m0_aw_prot   <= 3'b000;
            m0_w_valid   <= 1'b0;
            m0_w_data    <= '0;
            m0_w_strb    <= '0;
            m0_b_ready   <= 1'b1;
            m0_ar_valid  <= 1'b0;
            m0_ar_addr   <= '0;
            m0_ar_prot   <= 3'b000;
            m0_r_ready   <= 1'b1;
            cpu_axi_wr_busy <= 1'b0;
        end else begin
            // Clear handshakes after acceptance
            if (m0_aw_ready) m0_aw_valid <= 1'b0;
            if (m0_w_ready)  m0_w_valid  <= 1'b0;
            if (m0_ar_ready) m0_ar_valid <= 1'b0;
            if (m0_b_valid && m0_b_ready) cpu_axi_wr_busy <= 1'b0;

            // Issue new transactions when CPU requests to SRAM region
            if (cpu_mem_req && sel_sram && !cpu_axi_wr_busy) begin
                if (cpu_mem_wr) begin
                    m0_aw_valid     <= 1'b1;
                    m0_aw_addr      <= cpu_mem_addr;
                    m0_w_valid      <= 1'b1;
                    m0_w_data       <= cpu_mem_wdata;
                    m0_w_strb       <= cpu_mem_be;
                    cpu_axi_wr_busy <= 1'b1;
                end else begin
                    m0_ar_valid <= 1'b1;
                    m0_ar_addr  <= cpu_mem_addr;
                end
            end
        end
    end

    // CPU read data from AXI: feed back when r_valid
    // (Only needed for SRAM reads; peripheral reads handled by sel_* mux)

    // =========================================================================
    // SECTION 8: VXU → AXI BRIDGE  (M1 — VXU DMA requests)
    // The VXU uses a simple req/ack bus internally. Bridge to AXI-lite M1.
    // =========================================================================

    // VXU DMA signals (from vxu_top)
    logic        vxu_dma_req, vxu_dma_wr;
    logic [31:0] vxu_dma_addr;
    logic [VXU_DATA_W-1:0] vxu_dma_wdata [0:VXU_VEC_LEN-1];
    logic [VXU_DATA_W-1:0] vxu_dma_rdata [0:VXU_VEC_LEN-1];
    logic        vxu_dma_ack, vxu_dma_valid;

    // VXU→AXI bridge: serialize first lane for AXI (simplified 1-lane bridge)
    // Full burst arbitration handled inside vxu_dma_ctrl; AXI carries 32-bit words.
    logic vxu_axi_wr_busy;
    always_ff @(posedge clk_i or negedge rst_n_vxu) begin
        if (!rst_n_vxu) begin
            m1_aw_valid  <= 1'b0;
            m1_aw_addr   <= '0;
            m1_aw_prot   <= 3'b000;
            m1_w_valid   <= 1'b0;
            m1_w_data    <= '0;
            m1_w_strb    <= 4'hF;
            m1_b_ready   <= 1'b1;
            m1_ar_valid  <= 1'b0;
            m1_ar_addr   <= '0;
            m1_ar_prot   <= 3'b000;
            m1_r_ready   <= 1'b1;
            vxu_axi_wr_busy <= 1'b0;
            vxu_dma_ack  <= 1'b0;
            vxu_dma_valid<= 1'b0;
        end else begin
            if (m1_aw_ready) m1_aw_valid <= 1'b0;
            if (m1_w_ready)  m1_w_valid  <= 1'b0;
            if (m1_ar_ready) m1_ar_valid <= 1'b0;
            if (m1_b_valid && m1_b_ready) begin
                vxu_axi_wr_busy <= 1'b0;
                vxu_dma_ack     <= 1'b1;
            end else begin
                vxu_dma_ack <= 1'b0;
            end

            vxu_dma_valid <= m1_r_valid;
            if (m1_r_valid) begin
                // Feed first lane; remaining lanes driven by vxu_dma_ctrl's burst counter
                vxu_dma_rdata[0] <= m1_r_data[VXU_DATA_W-1:0];
            end

            if (vxu_dma_req && !vxu_axi_wr_busy) begin
                if (vxu_dma_wr) begin
                    m1_aw_valid     <= 1'b1;
                    m1_aw_addr      <= {vxu_dma_addr[31:2], 2'b00};
                    m1_w_valid      <= 1'b1;
                    m1_w_data       <= {{(32-VXU_DATA_W){1'b0}}, vxu_dma_wdata[0]};
                    m1_w_strb       <= 4'hF;
                    vxu_axi_wr_busy <= 1'b1;
                end else begin
                    m1_ar_valid <= 1'b1;
                    m1_ar_addr  <= {vxu_dma_addr[31:2], 2'b00};
                end
            end
        end
    end

    // Remaining VXU lanes tied low (burst extension handled by vxu_dma_ctrl)
    generate
        for (genvar i = 1; i < VXU_VEC_LEN; i++) begin : gen_vxu_rdata_tie
            assign vxu_dma_rdata[i] = '0;
        end
    endgenerate

    // =========================================================================
    // SECTION 9: DMA → AXI BRIDGE (M2)
    // dma_controller presents an AXI-lite master; wire directly.
    // =========================================================================

    // DMA AXI master signals (from dma_controller)
    logic        dma_m_aw_valid, dma_m_aw_ready;
    logic [31:0] dma_m_aw_addr;
    logic        dma_m_w_valid,  dma_m_w_ready;
    logic [31:0] dma_m_w_data;
    logic [3:0]  dma_m_w_strb;
    logic        dma_m_b_valid,  dma_m_b_ready;
    logic [1:0]  dma_m_b_resp;
    logic        dma_m_ar_valid, dma_m_ar_ready;
    logic [31:0] dma_m_ar_addr;
    logic        dma_m_r_valid,  dma_m_r_ready;
    logic [31:0] dma_m_r_data;
    logic [1:0]  dma_m_r_resp;

    // Connect DMA AXI master to M2 on interconnect
    assign m2_aw_valid = dma_m_aw_valid;
    assign dma_m_aw_ready = m2_aw_ready;
    assign m2_aw_addr  = dma_m_aw_addr;
    assign m2_aw_prot  = 3'b000;
    assign m2_w_valid  = dma_m_w_valid;
    assign dma_m_w_ready = m2_w_ready;
    assign m2_w_data   = dma_m_w_data;
    assign m2_w_strb   = dma_m_w_strb;
    assign dma_m_b_valid = m2_b_valid;
    assign m2_b_ready  = dma_m_b_ready;
    assign dma_m_b_resp  = m2_b_resp;
    assign m2_ar_valid = dma_m_ar_valid;
    assign dma_m_ar_ready = m2_ar_ready;
    assign m2_ar_addr  = dma_m_ar_addr;
    assign m2_ar_prot  = 3'b000;
    assign dma_m_r_valid = m2_r_valid;
    assign m2_r_ready  = dma_m_r_ready;
    assign dma_m_r_data  = m2_r_data;
    assign dma_m_r_resp  = m2_r_resp;

    // DMA config interface (CPU-programmable registers)
    logic [31:0] dma_cfg_addr;
    logic [31:0] dma_cfg_wdata;
    logic [31:0] dma_cfg_rdata_int;
    logic        dma_cfg_wr_en, dma_cfg_rd_en;

    assign dma_cfg_addr   = cpu_mem_addr;
    assign dma_cfg_wdata  = cpu_mem_wdata;
    assign dma_cfg_wr_en  = cpu_mem_req & cpu_mem_wr  & sel_dma_cfg;
    assign dma_cfg_rd_en  = cpu_mem_req & !cpu_mem_wr & sel_dma_cfg;
    assign dma_rdata       = dma_cfg_rdata_int;

    always_ff @(posedge clk_i or negedge rst_n_dma)
        if (!rst_n_dma) dma_cfg_ack <= 1'b0;
        else            dma_cfg_ack <= dma_cfg_wr_en | dma_cfg_rd_en;

    // =========================================================================
    // SECTION 10: VXU CONFIGURATION INTERFACE
    // CPU accesses VXU control registers at 0x2000_00xx via sel_vxu_cfg
    // VXU config bus is 4-bit address, 32-bit data
    // =========================================================================

    logic        vxu_cfg_wr_en, vxu_cfg_rd_en;
    logic [3:0]  vxu_cfg_addr;
    logic [31:0] vxu_cfg_wdata;
    logic [31:0] vxu_cfg_rdata_w;

    assign vxu_cfg_wr_en = cpu_mem_req & cpu_mem_wr  & sel_vxu_cfg;
    assign vxu_cfg_rd_en = cpu_mem_req & !cpu_mem_wr & sel_vxu_cfg;
    assign vxu_cfg_addr  = cpu_mem_addr[5:2];  // Word-aligned register select
    assign vxu_cfg_wdata = cpu_mem_wdata;
    assign vxu_cfg_rdata_32 = vxu_cfg_rdata_w;

    always_ff @(posedge clk_i or negedge rst_n_vxu)
        if (!rst_n_vxu) vxu_cfg_ack <= 1'b0;
        else            vxu_cfg_ack <= vxu_cfg_wr_en | vxu_cfg_rd_en;

    // =========================================================================
    // SECTION 11: PERIPHERAL REGISTER INTERFACES
    // All peripherals: 32-bit word-addressed, 1-cycle ACK
    // =========================================================================

    logic [7:0]  peri_byte_addr;
    assign peri_byte_addr = cpu_mem_addr[7:0];

    // Peripheral write/read enables
    logic uart_wr, uart_rd, spi_wr, spi_rd, i2c_wr, i2c_rd;
    logic gpio_wr, gpio_rd, timer_wr, timer_rd;
    assign uart_wr  = cpu_mem_req & cpu_mem_wr  & sel_uart;
    assign uart_rd  = cpu_mem_req & !cpu_mem_wr & sel_uart;
    assign spi_wr   = cpu_mem_req & cpu_mem_wr  & sel_spi;
    assign spi_rd   = cpu_mem_req & !cpu_mem_wr & sel_spi;
    assign i2c_wr   = cpu_mem_req & cpu_mem_wr  & sel_i2c;
    assign i2c_rd   = cpu_mem_req & !cpu_mem_wr & sel_i2c;
    assign gpio_wr  = cpu_mem_req & cpu_mem_wr  & sel_gpio;
    assign gpio_rd  = cpu_mem_req & !cpu_mem_wr & sel_gpio;
    assign timer_wr = cpu_mem_req & cpu_mem_wr  & sel_timer;
    assign timer_rd = cpu_mem_req & !cpu_mem_wr & sel_timer;

    // ACK generation (1-cycle registered)
    always_ff @(posedge clk_i or negedge rst_n_peri) begin
        if (!rst_n_peri) begin
            uart_ack  <= 1'b0; spi_ack  <= 1'b0;
            i2c_ack   <= 1'b0; gpio_ack <= 1'b0; timer_ack <= 1'b0;
        end else begin
            uart_ack  <= uart_wr  | uart_rd;
            spi_ack   <= spi_wr   | spi_rd;
            i2c_ack   <= i2c_wr   | i2c_rd;
            gpio_ack  <= gpio_wr  | gpio_rd;
            timer_ack <= timer_wr | timer_rd;
        end
    end

    // GPIO tri-state: GPIO controller drives OE per pin
    logic [GPIO_W-1:0] gpio_out, gpio_oe, gpio_in;
    genvar gp;
    generate
        for (gp = 0; gp < GPIO_W; gp++) begin : gen_gpio_bidir
            assign gpio_io[gp] = gpio_oe[gp] ? gpio_out[gp] : 1'bz;
            assign gpio_in[gp] = gpio_io[gp];
        end
    endgenerate

    // I2C tri-state
    logic i2c_sda_out, i2c_sda_oe, i2c_scl_out, i2c_scl_oe;
    assign i2c_sda_io = i2c_sda_oe ? i2c_sda_out : 1'bz;
    assign i2c_scl_io = i2c_scl_oe ? i2c_scl_out : 1'bz;

    // =========================================================================
    // SECTION 12: PMU CONFIGURATION INTERFACE
    // =========================================================================

    logic pmu_cfg_wr, pmu_cfg_rd;
    assign pmu_cfg_wr = cpu_mem_req & cpu_mem_wr  & sel_pmu_cfg;
    assign pmu_cfg_rd = cpu_mem_req & !cpu_mem_wr & sel_pmu_cfg;

    always_ff @(posedge clk_i or negedge rst_n_i)
        if (!rst_n_i) pmu_ack <= 1'b0;
        else          pmu_ack <= pmu_cfg_wr | pmu_cfg_rd;

    // =========================================================================
    // SECTION 13: DEBUG COUNTERS
    // =========================================================================

    logic [31:0] cycle_count_r, instr_count_r, dma_txn_count_r, vxu_op_count_r;

    // CPU instruction retired (from rv32im_core)
    logic cpu_instr_ret;

    // DMA transaction counter
    logic dma_done_pulse;
    always_ff @(posedge clk_i or negedge rst_n_i) begin
        if (!rst_n_i) begin
            cycle_count_r   <= '0;
            instr_count_r   <= '0;
            dma_txn_count_r <= '0;
            vxu_op_count_r  <= '0;
        end else begin
            cycle_count_r <= cycle_count_r + 1;
            if (cpu_instr_ret)  instr_count_r   <= instr_count_r   + 1;
            if (dma_done_pulse) dma_txn_count_r <= dma_txn_count_r + 1;
            if (irq_vxu)        vxu_op_count_r  <= vxu_op_count_r  + 1;
        end
    end

    assign dbg_cycle_count_o   = cycle_count_r;
    assign dbg_instr_count_o   = instr_count_r;
    assign dbg_dma_txn_count_o = dma_txn_count_r;
    assign dbg_vxu_op_count_o  = vxu_op_count_r;

    // VXU debug wires
    logic [31:0]          vxu_dbg_cycle;
    logic [VXU_VEC_LEN-1:0] vxu_dbg_lane_active;
    logic [3:0]           vxu_dbg_op_mode;

    // =========================================================================
    // SECTION 14: MODULE INSTANTIATIONS
    // =========================================================================

    // -------------------------------------------------------------------------
    // 14.1  rv32im_core — RISC-V CPU
    // -------------------------------------------------------------------------
    rv32im_core #(
        .RESET_VECTOR (RESET_VECTOR)
    ) u_cpu (
        .clk            (clk_cpu),
        .rst_n          (rst_n_cpu),
        // Memory interface
        .mem_req        (cpu_mem_req),
        .mem_wr         (cpu_mem_wr),
        .mem_addr       (cpu_mem_addr),
        .mem_wdata      (cpu_mem_wdata),
        .mem_be         (cpu_mem_be),
        .mem_rdata      (cpu_mem_rdata),
        .mem_ack        (cpu_mem_ack),
        // Interrupt
        .irq            (cpu_irq),
        .irq_vec        (irq_vec),
        // Debug
        .instr_ret      (cpu_instr_ret)
    );

    // -------------------------------------------------------------------------
    // 14.2  vxu_top — Vector Execution Unit
    // -------------------------------------------------------------------------
    vxu_top #(
        .VEC_LEN   (VXU_VEC_LEN),
        .DATA_W    (VXU_DATA_W),
        .ACCUM_W   (VXU_ACCUM_W),
        .ADDR_W    (ADDR_W),
        .REG_W     (DATA_W),
        .DMA_BURST (VXU_DMA_BURST)
    ) u_vxu (
        .clk             (clk_vxu),
        .rst_n           (rst_n_vxu),
        // CPU config interface
        .cfg_wr_en       (vxu_cfg_wr_en),
        .cfg_rd_en       (vxu_cfg_rd_en),
        .cfg_addr        (vxu_cfg_addr),
        .cfg_wdata       (vxu_cfg_wdata),
        .cfg_rdata       (vxu_cfg_rdata_w),
        .cfg_ack         (/* monitored via vxu_cfg_ack */),
        // DMA memory bus
        .dma_req         (vxu_dma_req),
        .dma_wr          (vxu_dma_wr),
        .dma_addr        (vxu_dma_addr),
        .dma_wdata       (vxu_dma_wdata),
        .dma_rdata       (vxu_dma_rdata),
        .dma_ack         (vxu_dma_ack),
        .dma_valid       (vxu_dma_valid),
        // Status / IRQ
        .vxu_irq         (irq_vxu),
        .vxu_busy        (/* tied to PMU later */),
        .vxu_done        (/* tied to PMU later */),
        // Debug
        .dbg_cycle_count (vxu_dbg_cycle),
        .dbg_lane_active (vxu_dbg_lane_active),
        .dbg_op_mode     (vxu_dbg_op_mode)
    );

    // -------------------------------------------------------------------------
    // 14.3  axi_interconnect — Bus Fabric (3 masters, 1 slave = SRAM)
    // -------------------------------------------------------------------------
    axi_interconnect #(
        .ADDR_W       (ADDR_W),
        .DATA_W       (DATA_W),
        .SRAM_BASE    (32'h0000_0000),
        .SRAM_SIZE    (32'(SRAM_SIZE_BYTES))
    ) u_axi_ic (
        .clk               (clk_i),
        .rst_n             (rst_n_axi),
        // M0: CPU
        .m0_aw_valid       (m0_aw_valid),  .m0_aw_ready (m0_aw_ready),
        .m0_aw_addr        (m0_aw_addr),   .m0_aw_prot  (m0_aw_prot),
        .m0_w_valid        (m0_w_valid),   .m0_w_ready  (m0_w_ready),
        .m0_w_data         (m0_w_data),    .m0_w_strb   (m0_w_strb),
        .m0_b_valid        (m0_b_valid),   .m0_b_ready  (m0_b_ready),
        .m0_b_resp         (m0_b_resp),
        .m0_ar_valid       (m0_ar_valid),  .m0_ar_ready (m0_ar_ready),
        .m0_ar_addr        (m0_ar_addr),   .m0_ar_prot  (m0_ar_prot),
        .m0_r_valid        (m0_r_valid),   .m0_r_ready  (m0_r_ready),
        .m0_r_data         (m0_r_data),    .m0_r_resp   (m0_r_resp),
        // M1: VXU
        .m1_aw_valid       (m1_aw_valid),  .m1_aw_ready (m1_aw_ready),
        .m1_aw_addr        (m1_aw_addr),   .m1_aw_prot  (m1_aw_prot),
        .m1_w_valid        (m1_w_valid),   .m1_w_ready  (m1_w_ready),
        .m1_w_data         (m1_w_data),    .m1_w_strb   (m1_w_strb),
        .m1_b_valid        (m1_b_valid),   .m1_b_ready  (m1_b_ready),
        .m1_b_resp         (m1_b_resp),
        .m1_ar_valid       (m1_ar_valid),  .m1_ar_ready (m1_ar_ready),
        .m1_ar_addr        (m1_ar_addr),   .m1_ar_prot  (m1_ar_prot),
        .m1_r_valid        (m1_r_valid),   .m1_r_ready  (m1_r_ready),
        .m1_r_data         (m1_r_data),    .m1_r_resp   (m1_r_resp),
        // M2: DMA
        .m2_aw_valid       (m2_aw_valid),  .m2_aw_ready (m2_aw_ready),
        .m2_aw_addr        (m2_aw_addr),   .m2_aw_prot  (m2_aw_prot),
        .m2_w_valid        (m2_w_valid),   .m2_w_ready  (m2_w_ready),
        .m2_w_data         (m2_w_data),    .m2_w_strb   (m2_w_strb),
        .m2_b_valid        (m2_b_valid),   .m2_b_ready  (m2_b_ready),
        .m2_b_resp         (m2_b_resp),
        .m2_ar_valid       (m2_ar_valid),  .m2_ar_ready (m2_ar_ready),
        .m2_ar_addr        (m2_ar_addr),   .m2_ar_prot  (m2_ar_prot),
        .m2_r_valid        (m2_r_valid),   .m2_r_ready  (m2_r_ready),
        .m2_r_data         (m2_r_data),    .m2_r_resp   (m2_r_resp),
        // S0: SRAM
        .s0_aw_valid       (s0_aw_valid),  .s0_aw_ready (s0_aw_ready),
        .s0_aw_addr        (s0_aw_addr),   .s0_aw_prot  (s0_aw_prot),
        .s0_w_valid        (s0_w_valid),   .s0_w_ready  (s0_w_ready),
        .s0_w_data         (s0_w_data),    .s0_w_strb   (s0_w_strb),
        .s0_b_valid        (s0_b_valid),   .s0_b_ready  (s0_b_ready),
        .s0_b_resp         (s0_b_resp),
        .s0_ar_valid       (s0_ar_valid),  .s0_ar_ready (s0_ar_ready),
        .s0_ar_addr        (s0_ar_addr),   .s0_ar_prot  (s0_ar_prot),
        .s0_r_valid        (s0_r_valid),   .s0_r_ready  (s0_r_ready),
        .s0_r_data         (s0_r_data),    .s0_r_resp   (s0_r_resp),
        // Debug
        .dbg_arb_grant_count (axi_dbg_grant_count),
        .dbg_stall_count     (axi_dbg_stall_count),
        .dbg_rd_txn_count    (axi_dbg_rd_txn),
        .dbg_wr_txn_count    (axi_dbg_wr_txn),
        .dbg_unmapped_count  (axi_dbg_unmapped)
    );

    // -------------------------------------------------------------------------
    // 14.4  unified_sram — 512KB Shared SRAM
    // -------------------------------------------------------------------------
    unified_sram #(
        .SRAM_SIZE_BYTES (SRAM_SIZE_BYTES),
        .DATA_W          (DATA_W),
        .PARITY_EN       (PARITY_EN)
    ) u_sram (
        .clk              (clk_i),
        .rst_n            (rst_n_sram),
        // Port A: AXI bridge
        .pa_req           (sram_pa_req),
        .pa_wr            (sram_pa_wr),
        .pa_addr          (sram_pa_addr),
        .pa_wdata         (sram_pa_wdata),
        .pa_be            (sram_pa_be),
        .pa_rdata         (sram_pa_rdata),
        .pa_ack           (sram_pa_ack),
        .pa_stall         (sram_pa_stall),
        // Port B: unused, tied off
        .pb_req           (1'b0),
        .pb_wr            (1'b0),
        .pb_addr          ('0),
        .pb_wdata         ('0),
        .pb_be            ('0),
        .pb_rdata         (/* unused */),
        .pb_ack           (/* unused */),
        .pb_stall         (/* unused */),
        // Parity / debug
        .pa_parity_err    (sram_pa_parity_err),
        .pb_parity_err    (sram_pb_parity_err),
        .dbg_pa_txn_count (sram_dbg_pa_txn),
        .dbg_pb_txn_count (sram_dbg_pb_txn),
        .dbg_collision_count (sram_dbg_collision),
        .dbg_pb_stall_count  (sram_dbg_pb_stall)
    );

    // -------------------------------------------------------------------------
    // 14.5  dma_controller — Burst DMA engine
    // -------------------------------------------------------------------------
    dma_controller #(
        .ADDR_W    (ADDR_W),
        .DATA_W    (DATA_W)
    ) u_dma (
        .clk            (clk_dma),
        .rst_n          (rst_n_dma),
        // CPU config registers
        .cfg_addr       (dma_cfg_addr[7:0]),
        .cfg_wdata      (dma_cfg_wdata),
        .cfg_rdata      (dma_cfg_rdata_int),
        .cfg_wr         (dma_cfg_wr_en),
        .cfg_rd         (dma_cfg_rd_en),
        // AXI master
        .m_aw_valid     (dma_m_aw_valid),  .m_aw_ready (dma_m_aw_ready),
        .m_aw_addr      (dma_m_aw_addr),
        .m_w_valid      (dma_m_w_valid),   .m_w_ready  (dma_m_w_ready),
        .m_w_data       (dma_m_w_data),    .m_w_strb   (dma_m_w_strb),
        .m_b_valid      (dma_m_b_valid),   .m_b_ready  (dma_m_b_ready),
        .m_b_resp       (dma_m_b_resp),
        .m_ar_valid     (dma_m_ar_valid),  .m_ar_ready (dma_m_ar_ready),
        .m_ar_addr      (dma_m_ar_addr),
        .m_r_valid      (dma_m_r_valid),   .m_r_ready  (dma_m_r_ready),
        .m_r_data       (dma_m_r_data),    .m_r_resp   (dma_m_r_resp),
        // Interrupt
        .dma_done       (irq_dma),
        .dma_busy       (/* available for PMU */),
        .dma_done_pulse (dma_done_pulse)
    );

    // -------------------------------------------------------------------------
    // 14.6  uart_16550 — UART Peripheral
    // -------------------------------------------------------------------------
    uart_16550 #(
        .CLK_FREQ_HZ (100_000_000),
        .DEFAULT_BAUD(115200)
    ) u_uart (
        .clk        (clk_peri),
        .rst_n      (rst_n_peri),
        .addr       (cpu_mem_addr[5:2]),
        .wdata      (cpu_mem_wdata),
        .rdata      (uart_rdata),
        .wr_en      (uart_wr),
        .rd_en      (uart_rd),
        .rx         (uart_rx_i),
        .tx         (uart_tx_o),
        .irq        (irq_uart)
    );

    // -------------------------------------------------------------------------
    // 14.7  spi_master — SPI Peripheral
    // -------------------------------------------------------------------------
    spi_master #(
        .CLK_DIV  (4)
    ) u_spi (
        .clk        (clk_peri),
        .rst_n      (rst_n_peri),
        .addr       (cpu_mem_addr[4:2]),
        .wdata      (cpu_mem_wdata),
        .rdata      (spi_rdata),
        .wr_en      (spi_wr),
        .rd_en      (spi_rd),
        .sclk       (spi_sclk_o),
        .mosi       (spi_mosi_o),
        .miso       (spi_miso_i),
        .cs_n       (spi_cs_n_o)
    );

    // -------------------------------------------------------------------------
    // 14.8  i2c_master — I2C Peripheral
    // -------------------------------------------------------------------------
    i2c_master #(
        .CLK_FREQ_HZ  (100_000_000),
        .I2C_FREQ_HZ  (400_000)
    ) u_i2c (
        .clk        (clk_peri),
        .rst_n      (rst_n_peri),
        .addr       (cpu_mem_addr[4:2]),
        .wdata      (cpu_mem_wdata),
        .rdata      (i2c_rdata),
        .wr_en      (i2c_wr),
        .rd_en      (i2c_rd),
        .sda_out    (i2c_sda_out),
        .sda_oe     (i2c_sda_oe),
        .sda_in     (i2c_sda_io),
        .scl_out    (i2c_scl_out),
        .scl_oe     (i2c_scl_oe),
        .scl_in     (i2c_scl_io)
    );

    // -------------------------------------------------------------------------
    // 14.9  gpio_ctrl — GPIO Controller
    // -------------------------------------------------------------------------
    gpio_ctrl #(
        .GPIO_W (GPIO_W)
    ) u_gpio (
        .clk        (clk_peri),
        .rst_n      (rst_n_peri),
        .addr       (cpu_mem_addr[5:2]),
        .wdata      (cpu_mem_wdata),
        .rdata      (gpio_rdata),
        .wr_en      (gpio_wr),
        .rd_en      (gpio_rd),
        .gpio_in    (gpio_in),
        .gpio_out   (gpio_out),
        .gpio_oe    (gpio_oe),
        .irq        (irq_gpio)
    );

    // -------------------------------------------------------------------------
    // 14.10 timer_unit — System Timer
    // -------------------------------------------------------------------------
    timer_unit #(
        .CLK_FREQ_HZ (100_000_000)
    ) u_timer (
        .clk        (clk_peri),
        .rst_n      (rst_n_peri),
        .addr       (cpu_mem_addr[4:2]),
        .wdata      (cpu_mem_wdata),
        .rdata      (timer_rdata),
        .wr_en      (timer_wr),
        .rd_en      (timer_rd),
        .irq        (irq_timer)
    );

    // -------------------------------------------------------------------------
    // 14.11 pmu_top — Power Management Unit
    // -------------------------------------------------------------------------
    pmu_top u_pmu (
        .clk             (clk_i),
        .rst_n           (rst_n_i),
        // CPU config
        .cfg_addr        (cpu_mem_addr[7:2]),
        .cfg_wdata       (cpu_mem_wdata),
        .cfg_rdata       (pmu_rdata),
        .cfg_wr          (pmu_cfg_wr),
        .cfg_rd          (pmu_cfg_rd),
        // Reset outputs (controlled release)
        .rst_n_cpu       (pmu_rst_n_cpu),
        .rst_n_vxu       (pmu_rst_n_vxu),
        .rst_n_peri      (pmu_rst_n_peri),
        .rst_n_dma       (pmu_rst_n_dma),
        .rst_n_sram      (pmu_rst_n_sram),
        .rst_n_axi       (pmu_rst_n_axi),
        // Clock gate enables
        .clk_en_cpu      (pmu_clk_en_cpu),
        .clk_en_peri     (pmu_clk_en_peri),
        .clk_en_vxu      (pmu_clk_en_vxu),
        .clk_en_dma      (pmu_clk_en_dma),
        // Status inputs for power decisions
        .vxu_busy        (irq_vxu),
        .dma_busy        (dma_done_pulse)
    );

    // =========================================================================
    // SECTION 15: BOOT FLOW SEQUENCER
    // Ensures deterministic startup: PMU → SRAM → DMA → VXU → PERI → CPU
    // PMU controls the rst_n_* outputs, which are the sequencing mechanism.
    // This section documents the expected PMU boot FSM behavior:
    //
    //   Cycle 0:    rst_n_i deasserted → PMU enters ACTIVE state
    //   Cycle 1-4:  PMU releases rst_n_sram, rst_n_axi
    //   Cycle 5-8:  PMU releases rst_n_dma
    //   Cycle 9-12: PMU releases rst_n_vxu
    //   Cycle 13-16:PMU releases rst_n_peri
    //   Cycle 17-20:PMU releases rst_n_cpu → CPU begins fetch from RESET_VECTOR
    //
    // The synchronizer FFs above add 2 additional cycles of latency per domain.
    // =========================================================================

    // =========================================================================
    // SECTION 16: SIMULATION ASSERTIONS
    // =========================================================================
    // synthesis translate_off
    // synthesis translate_off decode check converted to wired expression
    wire [8:0] sel_all_w = {sel_sram, sel_uart, sel_spi, sel_i2c,
                             sel_gpio, sel_timer, sel_vxu_cfg, sel_dma_cfg, sel_pmu_cfg};
    // At most one select should be active at any time
    always @(posedge clk_i) begin
        if (cpu_mem_req) begin
            assert ($onehot0(sel_all_w))
                else $warning("[neurorv_soc] Multiple decode hits sel=0b%09b addr=0x%08X",
                              sel_all_w, cpu_mem_addr);
        end
    end

    initial begin
        assert (ADDR_W == 32)
            else $fatal(1, "neurorv_soc: ADDR_W must be 32");
        assert (DATA_W == 32)
            else $fatal(1, "neurorv_soc: DATA_W must be 32");
        assert (VXU_VEC_LEN >= 16 && (VXU_VEC_LEN % 16 == 0))
            else $fatal(1, "neurorv_soc: VXU_VEC_LEN must be >= 16 and divisible by 16");
        $display("[neurorv_soc] NeuroRV Edge SoC initialized.");
        $display("[neurorv_soc]   SRAM: %0d KB", SRAM_SIZE_BYTES / 1024);
        $display("[neurorv_soc]   VXU lanes: %0d", VXU_VEC_LEN);
        $display("[neurorv_soc]   Reset vector: 0x%08X", RESET_VECTOR);
    end
    // synthesis translate_on

endmodule

`default_nettype wire
