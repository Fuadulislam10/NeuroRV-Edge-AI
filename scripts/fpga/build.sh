#!/usr/bin/env bash

# =============================================================================
# Project:     NeuroRV Edge
# Script:      build.sh
# Description: Compiles the FPGA top-level wrapper, hooks components, runs 
#              synthesis, implementation, and produces production bitstreams.
# =============================================================================

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"
BUILD_DIR="${WORKSPACE_DIR}/synthesis/fpga/build_output"

echo "========================================================================="
echo "   NeuroRV Edge FPGA Synthesis & Build Orchestration Automated Tool"
echo "========================================================================="
echo "Workspace Root Detected: ${WORKSPACE_DIR}"

# Step 1: Verify presence of critical EDA tooling execution engine
if ! command -v vivado &> /dev/null; then
    echo "ERROR: Xilinx Vivado engine executable could not be resolved in PATH."
    echo "Please source your structural design environments (e.g., /tools/Xilinx/Vivado/X.X/settings64.sh)."
    exit 1
fi

# Step 2: Clean existing builds safely to avoid caching inconsistencies
if [ -d "${BUILD_DIR}" ]; then
    echo "Clearing out outdated artifact storage structures at ${BUILD_DIR}..."
    rm -rf "${BUILD_DIR}"
fi

# Step 3: Transition working environment directory to avoid relative path generation leakage
cd "${WORKSPACE_DIR}/synthesis/fpga"

echo "Launching Vivado Batch Synthesis & Implementation flow..."
vivado -mode batch -source build_fpga.tcl

# Step 4: Validate successful delivery pipeline compilation
if [ -f "${BUILD_DIR}/neurorv_edge.bit" ]; then
    echo "========================================================================="
    echo " SUCCESS: Target Bitstream generated cleanly!"
    echo " Artifact Path: ${BUILD_DIR}/neurorv_edge.bit"
    echo "========================================================================="
else
    echo "========================================================================="
    echo " FAILURE: Bitstream was not detected. Inspect logs within execution directory."
    echo "========================================================================="
    exit 2
fi
