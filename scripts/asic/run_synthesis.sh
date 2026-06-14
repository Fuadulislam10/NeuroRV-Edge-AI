#!/usr/bin/env bash
# ==============================================================================
# FILE: run_synthesis.sh
# DESCRIPTION: Automation script wrapper for Yosys RTL Synthesis
# ==============================================================================

set -euo pipefail

export PDK_ROOT=${PDK_ROOT:-/usr/local/share/pdk}
export PDK=${PDK:-sky130A}
export STD_CELL_LIBRARY=${STD_CELL_LIBRARY:-sky130_fd_sc_hd}

echo "======================================================================="
echo "[STAGE START] RTL Synthesis Setup - Top Module: neurorv_soc"
echo "======================================================================="

mkdir -p outputs reports

yosys -c ../../synthesis/asic/synth.tcl

if [ -f "outputs/neurorv_soc.v" ] && [ -s "outputs/neurorv_soc.v" ]; then
    echo "======================================================================="
    echo "[STAGE SUCCESS] RTL Synthesis completed. Netlist ready."
    echo "======================================================================="
else
    echo "[STAGE ERROR] RTL Synthesis netlist generation failed or output is empty."
    exit 1
fi
