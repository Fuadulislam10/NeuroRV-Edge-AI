#!/usr/bin/env bash

# =============================================================================
# Project:     NeuroRV Edge
# Script:      program.sh
# Description: Connects to local hardware configurations via Vivado Hardware Manager
#              and flashes the target bitstream to the connected FPGA target.
# =============================================================================

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"
BITSTREAM="${WORKSPACE_DIR}/synthesis/fpga/build_output/neurorv_edge.bit"
TCL_PROGRAM_SCRIPT="${WORKSPACE_DIR}/synthesis/fpga/build_output/hw_program.tcl"

echo "========================================================================="
echo "   NeuroRV Edge JTAG Programming Script"
echo "========================================================================="

# Validate bitstream availability before firing Hardware Manager instances
if [ ! -f "${BITSTREAM}" ]; then
    echo "ERROR: Target configuration bitstream absent. Run './build.sh' first."
    exit 1
fi

if ! command -v vivado &> /dev/null; then
    echo "ERROR: Vivado executables are unavailable in the current terminal scope."
    exit 1
fi

echo "Creating targeted programming directives..."
cat << EOF > "${TCL_PROGRAM_SCRIPT}"
open_hw_manager
connect_hw_server -url localhost:3121 -allow_non_jtag
open_hw_target
set target_device [lindex [get_hw_devices] 0]
set_property PROGRAM.FILE "${BITSTREAM}" \$target_device
current_hw_device \$target_device
program_hw_devices \$target_device
refresh_hw_device -update_hw_probes false \$target_device
close_hw_target
disconnect_hw_server
close_hw_manager
EOF

echo "Executing JTAG flashing procedures via hardware daemon..."
if vivado -mode batch -source "${TCL_PROGRAM_SCRIPT}"; then
    echo "------------------------------------------------------------------------"
    echo " SUCCESS: Hardware Target programmed and initialized successfully!"
    echo "------------------------------------------------------------------------"
    rm -f "${TCL_PROGRAM_SCRIPT}"
else
    echo "------------------------------------------------------------------------"
    echo " CRITICAL FAILURE: Hardware connection or configuration pipeline abort."
    echo "------------------------------------------------------------------------"
    rm -f "${TCL_PROGRAM_SCRIPT}"
    exit 2
fi
