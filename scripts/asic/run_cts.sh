#!/usr/bin/env bash
# ==============================================================================
# FILE: run_cts.sh
# DESCRIPTION: Automation script wrapper for OpenROAD TritonCTS
# ==============================================================================

set -euo pipefail

export PDK_ROOT=${PDK_ROOT:-/usr/local/share/pdk}
export PDK=${PDK:-sky130A}
export STD_CELL_LIBRARY=${STD_CELL_LIBRARY:-sky130_fd_sc_hd}
export LIB_TYPICAL=${LIB_TYPICAL:-sky130_fd_sc_hd__tt_025C_1v80.lib}

echo "======================================================================="
echo "[STAGE START] Initiating Clock Tree Synthesis (TritonCTS)"
echo "======================================================================="

openroad -exit ../../synthesis/asic/cts.tcl

if [ -f "current_design_cts.odb" ]; then
    echo "======================================================================="
    echo "[STAGE SUCCESS] Clock Tree Synthesis executed successfully."
    echo "======================================================================="
else
    echo "[STAGE ERROR] CTS compilation failed."
    exit 1
fi
