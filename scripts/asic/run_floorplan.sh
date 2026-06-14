#!/usr/bin/env bash
# ==============================================================================
# FILE: run_floorplan.sh
# DESCRIPTION: Automation script wrapper for OpenROAD Floorplanning
# ==============================================================================

set -euo pipefail

export PDK_ROOT=${PDK_ROOT:-/usr/local/share/pdk}
export PDK=${PDK:-sky130A}
export STD_CELL_LIBRARY=${STD_CELL_LIBRARY:-sky130_fd_sc_hd}
export TECH_LEF=${TECH_LEF:-sky130_fd_sc_hd__nom.tech.lef}
export CELL_LEF=${CELL_LEF:-sky130_fd_sc_hd.lef}

echo "======================================================================="
echo "[STAGE START] Initializing Floorplan Boundaries & PDN Structures"
echo "======================================================================="

openroad -exit ../../synthesis/asic/floorplan.tcl

if [ -f "current_design_floorplan.odb" ]; then
    echo "======================================================================="
    echo "[STAGE SUCCESS] Floorplanning completed. Database file initialized."
    echo "======================================================================="
else
    echo "[STAGE ERROR] Floorplanning execution failed."
    exit 1
fi
