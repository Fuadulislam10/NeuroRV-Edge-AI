# Board Configuration Reference: NeuroRV Edge

This document outlines the standard configuration settings, target boards, and hardware resource deployment mappings for testing the `NeuroRV Edge` SoC.

## Target FPGA Parameters

| Parameter | Xilinx Reference Target | Intel Reference Target | Lattice Reference Target |
| :--- | :--- | :--- | :--- |
| **Board Name** | Nexys A7-100T / Basys 3 | Cyclone V SE / DE10-Nano | ECP5-5G Versa Board |
| **FPGA Device** | `xc7a100tcsg324-1` | `5CSEBA6U23I7` | `LFE5UM5G-45F-8BG381C` |
| **Input Clock** | 100 MHz | 50 MHz | 100 MHz |
| **System Clock**| 50 MHz (Internal) | 50 MHz (Direct) | 50 MHz (Internal) |
| **Logic Cells** | 101,440 LUTs | 110,000 LEs | 44,000 LUTs |
| **BRAM / Memory**| 4,860 Kbits | 5,570 Kbits | 1,008 Kbits |

## I/O Register Maps (Memory-Mapped Hardware Space)

* **Internal Boot RAM Base:** `0x0000_0000` (Size: 64 KB, pre-initialized with `memory_init.mem`)
* **GPIO Driver Register:** `0x8000_0000` (8-bits mappings to Physical LEDs and input Switches)
* **UART Serial Controller:** `0x8000_1000`
    * `0x8000_1000` -> Tx Data Buffer / Rx Data Buffer
    * `0x8000_1004` -> Baud Control / Line Status Field (Default: 115200 Baud, 8N1)

## Architecture Board Modifications

### For Basys 3 Deployments
Change device configurations within `build_fpga.tcl` to `xc7a35tcg236-1`. Also edit the package footprint labels within `constraints.xdc` to match the localized pin constraints for your UART Tx/Rx channels (`A18`/`B18`).

### For Intel Cyclone V (Quartus) Deployments
Skip the `.xdc` constraint ingestion. Utilize the assignments manager tool or write a `.qsf` layout mapping `clk_100mhz` to Pin `V11` on DE10 configurations.
