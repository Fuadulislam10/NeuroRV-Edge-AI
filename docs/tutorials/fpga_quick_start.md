# Quick Start Architecture Integration Guide: NeuroRV Edge

This tutorial covers compiling the hardware platform, building the system bitstream, flashing the target board, and running a basic validation firmware binary.

## Hardware Prerequisites
* **Target Development Board:** Nexys A7-100T (or Basys 3 alternative).
* **Host Development OS:** Linux Workstation (Ubuntu 22.04 LTS baseline recommended).
* **EDA Tool Suite:** Xilinx Vivado Suite v2022.2 or higher.
* **Cross Compiler Setup:** `riscv32-unknown-elf-gcc` toolchain available in shell execution profiles.

---

## Step-by-Step Bring-Up Pipeline

### 1. Synthesize and Construct Fabric Bitstreams
Navigate into the structural automation script workspace and trigger the deployment pipeline:
```bash
cd scripts/fpga
chmod +x *.sh
./build.sh
