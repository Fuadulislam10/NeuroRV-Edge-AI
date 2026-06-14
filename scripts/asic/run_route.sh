#!/usr/bin/env bash
# ==============================================================================
# FILE: run_route.sh
# DESCRIPTION: Automation script wrapper for OpenROAD Routing Layers
# ==============================================================================

set -euo pipefail

export PDK_ROOT=${PDK_ROOT:-/usr/local/share/pdk}
export PDK=${PDK:-sky130A}
export STD_CELL_LIBRARY=${STD_CELL_LIBRARY:-sky130_fd_sc_hd}
export LIB_TYPICAL=${LIB_TYPICAL:-sky130_fd_sc_hd__tt_025C_1v80.lib}

echo "======================================================================="
echo "[STAGE START] Global and Detailed Interconnect Routing"
echo "======================================================================="

openroad -exit ../../synthesis/asic/route.tcl

if [ -f "current_design_routed.odb" ]; then
    echo "======================================================================="
    echo "[STAGE SUCCESS] Interconnect Routing complete. Design clean."
    echo "======================================================================="
else
    echo "[STAGE ERROR] Detailed routing engine failed to clear track connections."
    exit 1
fi
