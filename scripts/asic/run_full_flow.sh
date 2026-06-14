#!/usr/bin/env bash
# ==============================================================================
# FILE: run_full_flow.sh
# DESCRIPTION: Master ASIC Flow Automation Orchestrator for NeuroRV Edge
# ==============================================================================

set -euo pipefail

# Standard System Configuration Base Environment Overrides
export PDK_ROOT=${PDK_ROOT:-/usr/local/share/pdk}
export PDK=${PDK:-sky130A}
export STD_CELL_LIBRARY=${STD_CELL_LIBRARY:-sky130_fd_sc_hd}
export TECH_LEF=${TECH_LEF:-sky130_fd_sc_hd__nom.tech.lef}
export CELL_LEF=${CELL_LEF:-sky130_fd_sc_hd.lef}

echo "#######################################################################"
echo "        NEURORV EDGE AUTOMATED ASICS MASTER FLOW GENERATOR"
echo "#######################################################################"
echo "Starting physical design execution using target environment configurations."

# Create execution sandboxes
mkdir -p outputs reports logs

# 1. RTL Synthesis Flow Pass
./run_synthesis.sh 2>&1 | tee logs/1_synthesis.log

# 2. Design Floorplanning Pass
./run_floorplan.sh 2>&1 | tee logs/2_floorplan.log

# 3. Component Placement Optimization Pass
./run_place.sh 2>&1 | tee logs/3_placement.log

# 4. Clock Tree Synthesis Pass
./run_cts.sh 2>&1 | tee logs/4_cts.log

# 5. Global & Detailed Interconnect Routing Pass
./run_route.sh 2>&1 | tee logs/5_routing.log

# 6. Static Timing Signoff Pass
./run_sta.sh 2>&1 | tee logs/6_sta.log

# 7. Energy Consumption & Power Audit Pass
./run_power.sh 2>&1 | tee logs/7_power.log

# 8. Stream Out Final Design GDSII Binary
echo "======================================================================="
echo "[STAGE START] Generating Final Layout Mask Formats (GDSII Stream out)"
echo "======================================================================="
openroad -exit ../../synthesis/asic/export_gds.tcl 2>&1 | tee logs/8_export_gds.log

if [ -f "outputs/neurorv_soc.gds" ]; then
    echo "#######################################################################"
    echo " [SUCCESS] FULL ASIC FLOW COMPLETE: outputs/neurorv_soc.gds IS READY"
    echo "#######################################################################"
else
    echo "#######################################################################"
    echo " [CRITICAL FAILURE] Tapeout compilation terminated prior to GDS creation."
    echo "#######################################################################"
    exit 1
fi
