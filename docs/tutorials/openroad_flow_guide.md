# Technology Setup and Architecture Configuration: NeuroRV Edge
# Target Technologies: SkyWater SKY130 (Primary), GF180MCU (Secondary)

## 1. Process Design Kit (PDK) Configuration
The NeuroRV Edge ASIC implementation flow is built around the open-source **OpenLane / OpenROAD** ecosystem using the **SkyWater SKY130NM** and **GlobalFoundries 180MCU** CMOS processes. The structural directories assume environmental variables point to the standard cell abstractions (`LEF`, `LIB`, `GDS`, `SPICE`).

### SKY130 Default Target
* **Standard Cell Variant:** `sky130_fd_sc_hd` (High Density)
* **Metal Stack:** `5M1LI` (1 Local Interconnect layer + 5 metal layers)
* **Core Voltage:** 1.8V
* **I/O Voltage:** 3.3V

### GF180MCU Default Target
* **Standard Cell Variant:** `gf180mcu_fd_sc_mcu7t5v0` (7-track, 5V CMOS)
* **Metal Stack:** `3M1TL` or `5M1TL` (Depending on configuration, default 5-metal stack `5LM`)
* **Core Voltage:** 5.0V / 3.3V

---

## 2. Environmental Variables Reference Matrix
Before executing any Tcl script, the execution environment must register the physical and timing libraries. The automation wrappers resolve these keys dynamically:

| Variable Name | SKY130 Target Value | GF180MCU Target Value |
| :--- | :--- | :--- |
| `PDK_ROOT` | `/usr/local/share/pdk` | `/usr/local/share/pdk` |
| `PDK` | `sky130A` | `gf180mcuC` |
| `STD_CELL_LIBRARY` | `sky130_fd_sc_hd` | `gf180mcu_fd_sc_mcu7t5v0` |
| `TECH_LEF` | `sky130_fd_sc_hd__nom.tech.lef` | `gf180mcu_fd_sc_mcu7t5v0.tech.lef` |
| `CELL_LEF` | `sky130_fd_sc_hd.lef` | `gf180mcu_fd_sc_mcu7t5v0.lef` |
| `LIB_TYPICAL` | `sky130_fd_sc_hd__tt_025C_1v80.lib` | `gf180mcu_fd_sc_mcu7t5v0__tt_25C_5v00.lib` |
| `LIB_SLOW` | `sky130_fd_sc_hd__ss_100C_1v60.lib` | `gf180mcu_fd_sc_mcu7t5v0__ss_125C_4v50.lib` |
| `LIB_FAST` | `sky130_fd_sc_hd__ff_n40C_1v95.lib` | `gf180mcu_fd_sc_mcu7t5v0__ff_n40C_5v50.lib` |

---

## 3. Floorplan & Power Ring Planning Schemes

### SkyWater 130nm Parameters
* **Core Utilization:** 35% (conservative allocation to account for heavy AI accelerator routing matrix)
* **Die Dimensions:** 3.5 mm × 3.5 mm
* **Power Grid Rails:**
    * `met5` / `met4` structural trunk matrix.
    * Power Strap Width: $1.6\,\mu\text{m}$, Pitch: $16.0\,\mu\text{m}$.

### GlobalFoundries 180nm Parameters
* **Core Utilization:** 45%
* **Die Dimensions:** 5.0 mm × 5.0 mm
* **Power Grid Rails:**
    * `Metal5` / `Metal4` global distribution.
    * Power Strap Width: $2.2\,\mu\text{m}$, Pitch: $24.0\,\mu\text{m}$.
