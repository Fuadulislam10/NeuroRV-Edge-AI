#!/usr/bin/env bash

# =============================================================================
# Project:     NeuroRV Edge
# Script:      verify_fpga.sh
# Description: Hardware automated validation test orchestrator checks execution
#              logs, tracks timing slack reports, and reviews device connections.
# =============================================================================

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"
REPORT_PATH="${WORKSPACE_DIR}/synthesis/fpga/build_output/post_route_timing_summary.rpt"

echo "========================================================================="
echo "   NeuroRV Edge Automated Post-Synthesis Validation Framework"
echo "========================================================================="

# Test Item 1: Timing Closure Validation
echo -n "Checking Hardware Timing Closures... "
if [ ! -f "${REPORT_PATH}" ]; then
    echo "FAILED"
    echo "ERROR: Route timing report generation missing or corrupted. Run compilation."
    exit 1
fi

# Look for structural keywords indicating negative slack breaches
if grep -q "VIOLATED" "${REPORT_PATH}" || grep -q "Slack (VIOLATED)" "${REPORT_PATH}"; then
    echo "FAILED"
    echo "CRITICAL: The current architecture layout reports structural timing failures."
    exit 2
else
    echo "PASSED"
    # Print the positive slack value for structural awareness
    grep -m 1 "Slack" "${REPORT_PATH}" || true
fi

# Test Item 2: Connected Device Probing
echo -n "Checking JTAG Connection Bus Diagnostics... "
if command -v vivado &> /dev/null; then
    # Generate a lightweight tcl command structure to probe for JTAG target frames
    PROBE_TCL="${WORKSPACE_DIR}/synthesis/fpga/build_output/probe_jtag.tcl"
    cat << EOF > "${PROBE_TCL}"
open_hw_manager
if {[catch {connect_hw_server -url localhost:3121 -allow_non_jtag} msg]} { exit 10 }
if {[catch {open_hw_target} msg]} { exit 20 }
set devices [get_hw_devices]
puts "FOUND_DEVICES:\$devices"
close_hw_target
disconnect_hw_server
close_hw_manager
EOF
    
    PROBE_LOG=$(vivado -mode batch -source "${PROBE_TCL}" 2>/dev/null || echo "PROBE_ABORT")
    rm -f "${PROBE_TCL}"
    
    if [[ "${PROBE_LOG}" == *"FOUND_DEVICES"* ]]; then
        echo "PASSED"
        echo "Found available on-board JTAG logic cells."
    else
        echo "WARNING"
        echo "Vivado could not bind to a physical programming target. Check cable inputs."
    fi
else
    echo "SKIPPED (Missing Vivado Toolchain)"
fi

echo "------------------------------------------------------------------------"
echo "Automated validation complete."
echo "========================================================================="
