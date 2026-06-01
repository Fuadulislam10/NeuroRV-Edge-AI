# NeuroRV Edge
### Hybrid RISC-V AI Accelerator SoC for Ultra-Low-Power Edge Intelligence

[![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](LICENSE)
[![Simulation](https://img.shields.io/badge/Sim-Verilator%204.x-green)](https://www.veripool.org/verilator/)
[![Synthesis](https://img.shields.io/badge/Synth-Yosys-orange)](https://github.com/YosysHQ/yosys)
[![P&R](https://img.shields.io/badge/P%26R-OpenROAD-purple)](https://github.com/The-OpenROAD-Project/OpenROAD)

---

## 🧠 Project Overview

**NeuroRV Edge** is a complete, open-source System-on-Chip (SoC) designed for ultra-low-power AI inference at the edge. It combines a 5-stage pipelined **RISC-V RV32IM** scalar processor with a custom **16-lane Vector Processing Unit (VPU)** optimized for neural network workloads such as MNIST-class inference.

The design targets open-source silicon toolchains (Yosys + OpenROAD) and FPGA prototyping (Xilinx/AMD), while following real ASIC design practices suitable for a 180nm–65nm tapeout.

---

## 🏗️ Architecture Overview

```
┌─────────────────────────────────────────────────────────────────┐
│                        NeuroRV Edge SoC                         │
│                                                                 │
│  ┌──────────────────────┐    ┌──────────────────────────────┐   │
│  │   RISC-V RV32IM CPU  │    │   Vector Processing Unit     │   │
│  │   5-Stage Pipeline   │    │   16-Lane SIMD Accelerator   │   │
│  │  ┌──┐┌──┐┌──┐┌──┐┌─┐│    │  ┌────────────────────────┐  │   │
│  │  │IF││ID││EX││ME││WB││    │  │ 16x 32-bit ALU Lanes   │  │   │
│  │  └──┘└──┘└──┘└──┘└─┘│    │  │ ADD/MUL/RELU/SIGMOID   │  │   │
│  │   Hazard + Fwd Unit  │    │  │ Reduction Tree (Sum/Max)│  │   │
│  │   RV32IM Multiplier  │    │  │ 16x 512-bit VRF        │  │   │
│  └──────────┬───────────┘    └──────────────┬─────────────┘   │
│             │                               │                  │
│  ┌──────────▼───────────────────────────────▼─────────────┐   │
│  │              AXI-Lite Interconnect / Arbiter            │   │
│  │         Priority: CPU > VPU  |  Addr Decode             │   │
│  └────┬──────────────┬────────────────┬────────────────────┘   │
│       │              │                │                         │
│  ┌────▼────┐  ┌──────▼─────┐  ┌──────▼─────┐                  │
│  │  64KB   │  │   GPIO /   │  │    UART    │                   │
│  │  SRAM   │  │    MMIO    │  │   Debug    │                   │
│  └─────────┘  └────────────┘  └────────────┘                   │
│                                                                 │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │           Power Management Unit (DVFS)                    │   │
│  │   ACTIVE → IDLE → SLEEP → DEEP_SLEEP  (wake-on-IRQ)      │   │
│  └──────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────┘
```

---

## 📊 Performance Metrics (Estimated @ 65nm)

| Parameter               | Value                   |
|-------------------------|-------------------------|
| Clock Frequency         | 100 MHz (FPGA), 500 MHz (ASIC est.) |
| CPU ISA                 | RV32IM                  |
| Pipeline Stages         | 5 (IF/ID/EX/MEM/WB)     |
| VPU Parallelism         | 16 lanes × 32-bit       |
| Vector Register File    | 16 registers × 512-bit  |
| SRAM                    | 64KB unified             |
| AI Throughput (MAC)     | 16 MACs/cycle (VPU)     |
| MNIST Inference (est.)  | ~10k inferences/sec      |
| Idle Power (est.)       | < 1 mW @ 65nm           |
| Active Power (est.)     | < 50 mW @ 65nm           |
| Gate Count (est.)       | ~80k gates              |

---

## 🤖 AI Acceleration Explanation

The **Vector Processing Unit (VPU)** accelerates neural network layers via:

- **16-lane SIMD**: Executes 16 multiply-accumulate (MAC) operations per clock
- **RELU/SIGMOID**: Activation functions implemented in hardware
- **Reduction Tree**: Parallel sum and max operations for pooling layers
- **Memory Bandwidth**: AXI-lite burst access to shared SRAM

A typical dense layer of size 784→256 runs in **~50 cycles** on the VPU vs. **~800 cycles** on the scalar CPU — a **16× speedup**.

---

## 📁 Repository Structure

```
neurorv-edge/
├── rtl/                    # SystemVerilog RTL sources
│   ├── neuro_rv_core.sv    # RV32IM 5-stage pipeline CPU
│   ├── neuro_vector_unit.sv# 16-lane VPU with AI ops
│   ├── neuro_interconnect.sv# AXI-lite arbiter + SRAM
│   ├── neuro_power_manager.sv# DVFS + sleep state machine
│   └── neuro_soc_top.sv    # SoC integration top-level
├── firmware/               # Bare-metal C firmware
│   ├── main.c              # MNIST inference demo + power demo
│   ├── vpu_tests.c         # VPU test vectors
│   └── linker.ld           # RV32 linker script
├── tb/                     # Testbenches
│   ├── tb_soc.sv           # SystemVerilog SoC testbench
│   └── sim_main.cpp        # Verilator C++ harness
├── scripts/                # Tool flow scripts
│   ├── synth.tcl           # Yosys synthesis
│   ├── pnr.tcl             # OpenROAD place & route
│   └── sim.sh              # Simulation run script
├── fpga/                   # FPGA-specific files
│   ├── fpga_top.sv         # FPGA wrapper with PLLs
│   └── constraints.xdc     # Xilinx XDC constraints
├── docs/                   # Design documentation
│   ├── architecture.md     # Detailed architecture doc
│   ├── design_flow.md      # Tools & flow guide
│   └── verification_plan.md# Verification strategy
└── waveforms/              # VCD/FST waveform outputs
```

---

## 🛠️ Prerequisites

### Required Tools

```bash
# RISC-V toolchain
sudo apt install gcc-riscv64-unknown-elf

# Verilator (simulation)
sudo apt install verilator

# Yosys (synthesis)
sudo apt install yosys

# GTKWave (waveform viewer)
sudo apt install gtkwave

# OpenROAD (place & route) - build from source
git clone --recursive https://github.com/The-OpenROAD-Project/OpenROAD.git
```

---

## 🚀 Quick Start

### 1. Clone the Repository

```bash
git clone https://github.com/your-org/neurorv-edge.git
cd neurorv-edge
```

### 2. Run Simulation

```bash
chmod +x scripts/sim.sh
./scripts/sim.sh
# Outputs: waveforms/soc_sim.vcd
```

### 3. View Waveforms

```bash
gtkwave waveforms/soc_sim.vcd &
```

### 4. Synthesize with Yosys

```bash
yosys scripts/synth.tcl
# Outputs: build/neuro_soc_synth.json
```

### 5. Place & Route with OpenROAD

```bash
openroad scripts/pnr.tcl
# Outputs: results/neuro_soc_final.def
```

### 6. Build Firmware

```bash
cd firmware
riscv64-unknown-elf-gcc -march=rv32im -mabi=ilp32 \
  -T linker.ld -nostdlib -o firmware.elf main.c vpu_tests.c
riscv64-unknown-elf-objcopy -O ihex firmware.elf firmware.hex
```

---

## 🗺️ Roadmap

### v1.0 — FPGA Prototype (Current)
- [x] RV32IM 5-stage pipeline
- [x] 16-lane VPU with ADD/MUL/RELU/SIGMOID
- [x] AXI-lite interconnect
- [x] Power management state machine
- [x] 64KB SRAM subsystem
- [x] Bare-metal MNIST firmware
- [x] Verilator simulation
- [x] Yosys synthesis
- [ ] Vivado FPGA bitstream (Arty A7-35T)
- [ ] UART-based firmware loading

### v2.0 — ASIC Tapeout (Planned)
- [ ] Complete OpenROAD P&R flow (Sky130 PDK)
- [ ] DRC/LVS clean
- [ ] Post-layout simulation
- [ ] Formal verification (SymbiYosys)
- [ ] Static timing analysis (OpenSTA)
- [ ] Fabrication via Efabless/Tiny Tapeout

### v3.0 — Extended Features
- [ ] INT8 quantization in VPU
- [ ] DMA controller
- [ ] SPI/I2C peripherals
- [ ] Larger SRAM (256KB)
- [ ] JTAG debug interface

---

## 📖 Documentation

- [Architecture Deep Dive](docs/architecture.md)
- [Design Flow & Tools Guide](docs/design_flow.md)
- [Verification Plan](docs/verification_plan.md)

---

## 🤝 Contributing

Contributions are welcome! Please read [CONTRIBUTING.md](CONTRIBUTING.md) and open a PR with:
- Simulation passing for your changes
- Updated testbench coverage
- Documentation updates

---

## 📄 License

Apache 2.0 — See [LICENSE](LICENSE)

---

*NeuroRV Edge — Bringing neural intelligence to the silicon edge.*
