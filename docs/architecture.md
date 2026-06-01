# NeuroRV Edge — Architecture Specification
**Document ID:** NRV-ARCH-001  
**Revision:** 1.0.0  
**Status:** Released

---

## 1. System Overview

### 1.1 Introduction

NeuroRV Edge is a Single-Chip System-on-Chip (SoC) integrating a scalar RISC-V RV32IM CPU with a 256-lane SIMD Vector Execution Unit (VXU) optimized for neural network inference. The design uses a unified AXI4 interconnect fabric allowing the CPU, VXU, and DMA to share a flat 32-bit physical address space backed by on-chip SRAM.

### 1.2 Design Goals

- **Inference Latency**: Sub-100ms for MobileNetV2-class models at edge
- **Power Envelope**: <100mW active, <10μW deep-sleep
- **Area Target**: <2mm² in SKY130B 130nm process
- **FPGA Resource**: <60% LUT utilization on Artix-7 XC7A100T
- **Correctness**: RISC-V ISA compliance verified by riscv-tests suite
- **Openness**: Zero proprietary IP dependencies

---

## 2. Address Map

### 2.1 Physical Memory Map (32-bit)

```
0x0000_0000 – 0x0003_FFFF   256KB  Instruction SRAM (ISRAM)
0x0004_0000 – 0x0007_FFFF   256KB  Data SRAM (DSRAM)
0x0008_0000 – 0x000F_FFFF   512KB  VXU Weight Buffer (WSRAM)
0x1000_0000 – 0x1000_FFFF    64KB  VXU Control Registers
0x2000_0000 – 0x2000_0FFF     4KB  DMA Controller
0x3000_0000 – 0x3000_0FFF     4KB  UART0
0x3000_1000 – 0x3000_1FFF     4KB  UART1
0x3000_2000 – 0x3000_2FFF     4KB  SPI0
0x3000_3000 – 0x3000_3FFF     4KB  I2C0
0x3000_4000 – 0x3000_4FFF     4KB  GPIO
0x3000_5000 – 0x3000_5FFF     4KB  Timer0/1
0x4000_0000 – 0x4000_0FFF     4KB  PMU / Clock Control
0xE000_0000 – 0xE000_0FFF     4KB  CLINT (timer/sw interrupts)
0xE000_1000 – 0xE000_2FFF     8KB  PLIC (ext interrupts)
0xF000_0000 – 0xF000_0FFF     4KB  Debug Module (DM)
```

### 2.2 VXU Register Map (offset from 0x1000_0000)

```
0x000   VXU_CTRL        Control (start/stop/mode)
0x004   VXU_STATUS      Status (busy/done/error)
0x008   VXU_OP          Operation code (GEMM/CONV/POOL/NORM/ACT)
0x00C   VXU_SRC_ADDR    Source tensor base address
0x010   VXU_WGT_ADDR    Weight tensor base address
0x014   VXU_DST_ADDR    Destination tensor base address
0x018   VXU_SHAPE0      Tensor dimensions [N:W]
0x01C   VXU_SHAPE1      Tensor dimensions [H:C]
0x020   VXU_STRIDE      Convolution stride [Y:X]
0x024   VXU_PAD         Padding [bottom:top:right:left]
0x028   VXU_QUANT       Quantization scale/zero-point
0x02C   VXU_ACT_CFG     Activation config
0x030   VXU_IRQ_EN      Interrupt enable
0x034   VXU_IRQ_STATUS  Interrupt status (W1C)
0x038   VXU_PERF0       Cycle counter (low)
0x03C   VXU_PERF1       Cycle counter (high)
0x040   VXU_PERF2       MAC operation counter
0x100   VXU_CMD_FIFO    Command FIFO (64 entries, push-only)
```

---

## 3. CPU Subsystem (RV32IM)

### 3.1 Pipeline Architecture

```
   ┌───────────┐    ┌───────────┐    ┌───────────┐    ┌───────────┐    ┌───────────┐
   │  IF Stage │───►│  ID Stage │───►│  EX Stage │───►│ MEM Stage │───►│  WB Stage │
   │           │    │           │    │           │    │           │    │           │
   │ PC Logic  │    │ Decode    │    │ ALU       │    │ DMEM I/F  │    │ Writeback │
   │ ISRAM I/F │    │ Regfile   │    │ MulDiv    │    │ Load Align│    │ Mux       │
   │ Branch Pr.│    │ Imm-Gen   │    │ Branch    │    │ Store Mask│    │           │
   └───────────┘    │ Hazard Det│    │ CSR       │    └───────────┘    └───────────┘
                    └───────────┘    └───────────┘
                           │                │
                           └────────────────┘
                             Data Forwarding
                             (EX/MEM → ID/EX)
```

### 3.2 Pipeline Features

| Feature | Implementation |
|---|---|
| Branch prediction | Static predict-not-taken (1-cycle penalty on taken) |
| Branch resolution | End of EX stage |
| Load-use hazard | 1-cycle stall inserted by hazard unit |
| RAW hazard | Full forwarding: MEM→EX, WB→EX, WB→MEM |
| WAW/WAR hazards | Resolved by in-order retirement |
| Exception handling | Precise exceptions, trap to mtvec |
| Interrupt latency | ≤3 cycles from assertion to handler |

### 3.3 Multiply/Divide Unit

- **Multiplier**: Radix-4 Booth encoding, 4-cycle latency (MUL/MULH/MULHSU/MULHU)
- **Divider**: Non-restoring algorithm, 32-cycle worst-case (DIV/DIVU/REM/REMU)
- **Stall**: Pipeline stalls during multi-cycle operations

### 3.4 CSR Implementation

Mandatory CSRs implemented: `misa`, `mvendorid`, `marchid`, `mimpid`, `mhartid`, `mstatus`, `mtvec`, `medeleg` (stub), `mideleg` (stub), `mip`, `mie`, `mcycle`, `minstret`, `mcycleh`, `minstreth`, `mscratch`, `mepc`, `mcause`, `mtval`.

---

## 4. Vector AI Accelerator (VXU)

### 4.1 VXU Architecture

```
                    ┌─────────────────────────────────────┐
  AXI4 Master ─────►│          VXU DMA Controller          │
                    └─────────────┬───────────────────────┘
                                  │ Internal 256-bit bus
                    ┌─────────────▼───────────────────────┐
                    │         Dispatcher / Sequencer        │
                    │   (decodes VXU_OP, generates uops)   │
                    └──┬──────────┬────────────────────────┘
                       │          │
          ┌────────────▼──┐   ┌───▼──────────────────────┐
          │  Weight Buffer│   │   256-Lane MAC Array      │
          │  64KB SRAM    │   │   32 × INT8 MAC / cycle   │
          │  Ping-Pong    │   │   or 16 × FP16 MAC/cycle  │
          └────────────┬──┘   └───┬──────────────────────┘
                       │          │  Accumulation (32-bit)
                       │   ┌──────▼──────────────────────┐
                       │   │   Activation Unit            │
                       │   │   ReLU / ReLU6 / GELU /      │
                       │   │   Sigmoid / HardSwish        │
                       │   └──────┬──────────────────────┘
                       │   ┌──────▼──────────────────────┐
                       │   │   Pooling Unit               │
                       │   │   MaxPool / AvgPool / Global │
                       │   └──────┬──────────────────────┘
                       │   ┌──────▼──────────────────────┐
                       └──►│   Normalization Unit         │
                           │   BatchNorm / LayerNorm      │
                           └──────┬──────────────────────┘
                                  │
                           Write-back to DSRAM via DMA
```

### 4.2 Supported Operations

| Opcode | Operation | Data Types |
|---|---|---|
| 0x01 | GEMM | INT8, FP16 |
| 0x02 | CONV2D | INT8, FP16 |
| 0x03 | DEPTHWISE_CONV | INT8 |
| 0x04 | MAXPOOL | INT8, FP16 |
| 0x05 | AVGPOOL | INT8, FP16 |
| 0x06 | GLOBALAVGPOOL | INT8, FP16 |
| 0x07 | RELU | INT8, FP16 |
| 0x08 | RELU6 | INT8 |
| 0x09 | SIGMOID | FP16 |
| 0x0A | GELU | FP16 |
| 0x0B | BATCHNORM | FP16 |
| 0x0C | LAYERNORM | FP16 |
| 0x0D | ELTWISE_ADD | INT8, FP16 |
| 0x0E | ELTWISE_MUL | FP16 |
| 0x0F | SOFTMAX | FP16 |
| 0x10 | REQUANTIZE | INT8→INT8 |

### 4.3 MAC Array Details

- **Width**: 256 bits (32 INT8 MACs or 16 FP16 MACs)
- **Depth**: 16 rows (systolic tiling)
- **Accumulator**: 32-bit per lane (INT32 accumulation for INT8)
- **Weight stationary**: Weights loaded once per tile, activations streamed
- **Peak throughput (INT8)**: 256 MACs/cycle × 2 (mul+add) = 512 OPS/cycle

### 4.4 Quantization Support

INT8 symmetric and asymmetric quantization:
- Per-tensor and per-channel scale/zero-point
- Requantization unit for chained INT8 operations
- Saturation on overflow (no wrapping)

---

## 5. Memory Subsystem

### 5.1 Unified SRAM Architecture

```
         CPU I-Fetch    CPU D-Access    VXU DMA     DMA Controller
              │               │             │              │
         ┌────▼───────────────▼─────────────▼──────────────▼────┐
         │              AXI4 Crossbar (4×4)                      │
         │         Round-robin + priority arbitration             │
         └─────┬──────────────┬─────────────┬────────────────────┘
               │              │             │
        ┌──────▼──────┐ ┌─────▼──────┐ ┌───▼──────────┐
        │  ISRAM Bank │ │  DSRAM Bank│ │  WSRAM Banks │
        │  256KB      │ │  256KB     │ │  64KB (VXU)  │
        │  (4×64KB)   │ │  (4×64KB) │ │  (ping-pong) │
        └─────────────┘ └────────────┘ └──────────────┘
```

### 5.2 DMA Controller

- **Channels**: 4 independent channels
- **Transfer types**: Memory-to-memory, peripheral-to-memory, memory-to-peripheral
- **Descriptor format**: Scatter-gather linked list (64-bit descriptors)
- **Maximum transfer**: 64KB per descriptor
- **Burst size**: Up to 16 AXI beats
- **Interrupt**: Per-channel completion + error interrupt

---

## 6. Power Management

### 6.1 Power Domains

| Domain | Contents | Shutdown |
|---|---|---|
| PD_ALWAYS_ON | PMU, RTC, retention RAM | Never |
| PD_CPU | CPU pipeline, regfile | Deep-sleep |
| PD_MEM | SRAM banks | Configurable |
| PD_VXU | AI accelerator | Software |
| PD_PERIPH | UART/SPI/I2C/GPIO | Software |

### 6.2 Power Modes

| Mode | Active Domains | Wake Sources | Power (est.) |
|---|---|---|---|
| Active | All | — | ~100mW |
| Idle | All, CPU WFI | Any interrupt | ~40mW |
| Sleep | CPU off, VXU off | Timer, GPIO, UART | ~5mW |
| Deep-Sleep | PD_ALWAYS_ON only | GPIO, RTC | <10μW |
| Retention | PD_ALWAYS_ON + retention RAM | Same | <20μW |

### 6.3 Clock Architecture

```
External Crystal (12/24/48MHz)
          │
     ┌────▼────┐
     │  PLL    │  (FPGA: MMCM/PLL, ASIC: analog PLL)
     └────┬────┘
          │
    ┌─────▼──────┐
    │  Clock Mux │ ← PMU control
    └─────┬──────┘
          │
    ┌─────▼───────────────────────────────────────────┐
    │  Clock Distribution Tree                         │
    │  clk_sys (100MHz) → CPU, AXI, SRAM              │
    │  clk_vxu (100MHz) → VXU (can be gated)          │
    │  clk_peri (50MHz) → Peripherals                 │
    │  clk_slow (32kHz) → PMU, RTC                    │
    └──────────────────────────────────────────────────┘
```

---

## 7. Interrupts

### 7.1 Interrupt Architecture

NeuroRV Edge uses a Platform-Level Interrupt Controller (PLIC) compatible with the RISC-V PLIC specification.

| IRQ # | Source | Priority Default |
|---|---|---|
| 1 | UART0 RX/TX | 1 |
| 2 | UART1 RX/TX | 1 |
| 3 | SPI0 | 1 |
| 4 | I2C0 | 1 |
| 5 | GPIO (combined) | 2 |
| 6 | Timer0 | 3 |
| 7 | Timer1 | 3 |
| 8 | DMA Ch0 | 2 |
| 9 | DMA Ch1 | 2 |
| 10 | DMA Ch2 | 2 |
| 11 | DMA Ch3 | 2 |
| 12 | VXU Done | 4 |
| 13 | VXU Error | 4 |
| 14 | PMU Alert | 5 |

### 7.2 CLINT

Machine-mode software interrupt and timer interrupt via CLINT at 0xE000_0000.  
`mtime` / `mtimecmp` registers use `clk_slow` (32kHz tick).

---

## 8. Debug Subsystem

### 8.1 RISC-V Debug (JTAG)

Implements RISC-V External Debug Specification v0.13:
- **DTM** (Debug Transport Module): JTAG TAP
- **DMI** (Debug Module Interface): Abstract commands
- **DM** (Debug Module): Halt/resume, memory access, register access
- **JTAG signals**: TCK, TMS, TDI, TDO, TRST_N

### 8.2 Trace (Optional)

Instruction trace buffer (512 entries) controlled via debug registers.

---

## 9. Interfaces

### 9.1 UART
- 16550-compatible
- Baud rate: up to 3Mbps at 100MHz
- FIFOs: 16-byte TX, 16-byte RX
- Interrupts: RX available, TX empty, line status

### 9.2 SPI
- Master-only in v1.0
- Modes: 0, 1, 2, 3
- Max clock: clk_peri/2 = 25MHz
- 8/16/32-bit transfers
- CS: up to 4 chip selects

### 9.3 I²C
- Master-only in v1.0  
- Standard (100kbps), Fast (400kbps), Fast-Plus (1Mbps)
- 7-bit and 10-bit addressing

### 9.4 GPIO
- 32 bidirectional pins
- Configurable pull-up/pull-down (ASIC) or external (FPGA)
- Edge/level interrupt per pin
- Output drive strength configurable

---

## 10. Verification Strategy

### 10.1 Verification Hierarchy

```
Level 4: System (SoC)     ← Full regression, boot test, inference demo
Level 3: Integration       ← CPU+MEM, VXU+DMA, CPU+VXU IPC
Level 2: Sub-system        ← AXI interconnect, DMA standalone
Level 1: Unit              ← Each RTL module (CPU stages, VXU ops, peripherals)
Level 0: Formal            ← Assertions on critical properties
```

### 10.2 Toolchain

| Tool | Purpose |
|---|---|
| Verilator 5.x | Fast simulation, lint |
| Icarus Verilog | Reference simulation |
| cocotb | Python-based testbenches |
| riscv-tests | ISA compliance |
| RISC-V Torture | Random instruction stress test |
| SymbiYosys | Formal verification |
| GTKWave | Waveform viewing |

---

## 11. Synthesis Targets

### 11.1 FPGA (Xilinx Artix-7)

| Resource | Estimated | Budget |
|---|---|---|
| LUTs | ~28,000 | 63,400 (44%) |
| FFs | ~16,000 | 126,800 (13%) |
| BRAM (36Kb) | 36 | 135 (27%) |
| DSP48 | 24 | 240 (10%) |
| Fmax | ~85MHz | 100MHz target |

### 11.2 ASIC (SKY130B)

| Metric | Estimated |
|---|---|
| Area | ~1.8mm² |
| Cell count | ~250K gates |
| Fmax | ~200MHz |
| Power (active) | ~80mW @ 200MHz, 1.8V |
| Power (sleep) | <5μW |

---

*End of Architecture Specification NRV-ARCH-001 Rev 1.0.0*
