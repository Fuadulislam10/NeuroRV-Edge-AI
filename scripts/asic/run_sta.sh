#!/usr/bin/env bash
# ==============================================================================
# FILE: run_sta.sh
# DESCRIPTION: Automation script wrapper for OpenSTA Static Timing Analysis
# ==============================================================================

set -euo pipefail

export PDK_ROOT=${PDK_ROOT:-/usr/local/share/pdk}
export PDK=${PDK:-sky130A}
export STD_CELL_LIBRARY=${STD_CELL_LIBRARY:-sky130_fd_sc_hd}
export TECH_LEF=${TECH_LEF:-sky130_fd_sc_hd__nom.tech.lef}
export CELL_LEF=${CELL_LEF:-sky130_fd_sc_hd.lef}
export LIB_FAST=${LIB_FAST:-sky130_fd_sc_hd__ff_n40C_1v95.lib}
export LIB_SLOW=${LIB_SLOW:-sky130_fd_sc_hd__ss_100C_1v60.lib}

echo "======================================================================="
echo "[STAGE START] Signoff Static Timing Analysis Execution Pass"
echo "======================================================================="

openroad -exit ../../synthesis/asic/timing.tcl

if [ -f "reports/timing_setup_signoff.rpt" ] && [ -f "reports/timing_hold_signoff.rpt" ]; then
    echo "======================================================================="
    echo "[STAGE SUCCESS] Static Timing Analysis signoff data archived."
    echo "======================================================================="
else
    echo "[STAGE ERROR] Signoff timing execution failed."
    exit 1
fi
