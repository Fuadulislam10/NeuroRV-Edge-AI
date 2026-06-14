#!/usr/bin/env bash
# ==============================================================================
# FILE: run_power.sh
# DESCRIPTION: Automation script wrapper for OpenROAD Power Analysis
# ==============================================================================

set -euo pipefail

export PDK_ROOT=${PDK_ROOT:-/usr/local/share/pdk}
export PDK=${PDK:-sky130A}
export STD_CELL_LIBRARY=${STD_CELL_LIBRARY:-sky130_fd_sc_hd}
export TECH_LEF=${TECH_LEF:-sky130_fd_sc_hd__nom.tech.lef}
export CELL_LEF=${CELL_LEF:-sky130_fd_sc_hd.lef}
export LIB_TYPICAL=${LIB_TYPICAL:-sky130_fd_sc_hd__tt_025C_1v80.lib}

echo "======================================================================="
echo "[STAGE START] Executing Multi-Corner Power Dissipation Profile"
echo "======================================================================="

openroad -exit ../../synthesis/asic/power_analysis.tcl

if [ -f "reports/power_signoff_summary.rpt" ]; then
    echo "======================================================================="
    echo "[STAGE SUCCESS] Power profile summary completed and exported."
    echo "======================================================================="
else
    echo "[STAGE ERROR] Power calculation phase failed."
    exit 1
fi
