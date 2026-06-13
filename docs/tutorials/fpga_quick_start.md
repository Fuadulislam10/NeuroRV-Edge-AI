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

---

This processes the underlying modules, maps system components, paths input nets, constructs internal hardware block RAMs, and structures the final binary image package at `synthesis/fpga/build_output/neurorv_edge.bit.`

### 2. Connect and Prepare Physical Infrastructure
Connect micro-USB lines directly to the board's JTAG/UART port connection.

Ensure slide switches (switches[7:0]) are set to their baseline positions.

Check that the board's power jumper configuration matches raw USB power source feeds.

Flip the master hardware power slide switch to the ON position.

3. Flash Hardware Design Configuration
Execute the automated JTAG programming tool wrapper:
./program.sh
