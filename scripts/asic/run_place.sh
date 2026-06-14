#!/usr/bin/env bash
# ==============================================================================
# FILE: run_place.sh
# DESCRIPTION: Automation script wrapper for OpenROAD Cell Placement
# ==============================================================================

set -euo pipefail

export PDK_ROOT=${PDK_ROOT:-/usr/local/share/pdk}
export PDK=${PDK:-sky130A}
export STD_CELL_LIBRARY=${STD_CELL_LIBRARY:-sky130_fd_sc_hd}
export LIB_TYPICAL=${LIB_TYPICAL:-sky130_fd_sc_hd__tt_025C_1v80.lib}

echo "======================================================================="
echo "[STAGE START] Global and Detailed Standard Cell Placement"
echo "======================================================================="

openroad -exit ../../synthesis/asic/place.tcl

if [ -f "current_design_placed.odb" ]; then
    echo "======================================================================="
    echo "[STAGE SUCCESS] Cell Placement finalized and legalized."
    echo "======================================================================="
else
    echo "[STAGE ERROR] Placement optimization engine terminated unexpectedly."
    exit 1
fi
