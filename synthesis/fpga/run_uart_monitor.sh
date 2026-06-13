#!/usr/bin/env bash

# =============================================================================
# Project:     NeuroRV Edge
# Script:      run_uart_monitor.sh
# Description: Diagnostic serial transmission intercept connection tool.
#              Attaches directly to FPGA serial lines for firmware telemetry.
# =============================================================================

set -euo pipefail

DEFAULT_PORT="/dev/ttyUSB1"
DEFAULT_BAUD="115200"

PORT="${1:-$DEFAULT_PORT}"
BAUD="${2:-$DEFAULT_BAUD}"

echo "========================================================================="
echo "   NeuroRV Edge Diagnostic UART Telemetry Console"
echo "========================================================================="
echo "Target Interface Hardware Layer: ${PORT}"
echo "Configured Baud Rate Profile   : ${BAUD} bps (8N1 Data Framing)"
echo "------------------------------------------------------------------------"

# Verify access privileges to raw system terminal infrastructure nodes
if [ ! -e "${PORT}" ]; then
    echo "ERROR: Specified interface node [${PORT}] does not exist."
    echo "Please plug in the board or check 'dmesg | grep tty'."
    exit 1
fi

if [ ! -r "${PORT}" ] || [ ! -w "${PORT}" ]; then
    echo "WARNING: Insufficient operational permissions for device ${PORT}."
    echo "Attempting execution with root privilege framework escalation..."
    SUDO_CMD="sudo"
else
    SUDO_CMD=""
fi

# Detect operational console engine system wrappers present
if command -v picocom &> /dev/null; then
    echo "Launching telemetry tracking via picocom. Exit terminal using Ctrl+A followed by Ctrl+X."
    $SUDO_CMD picocom -b "${BAUD}" "${PORT}"
elif command -v minicom &> /dev/null; then
    echo "Launching telemetry tracking via minicom. Exit terminal using Ctrl+A followed by Z."
    $SUDO_CMD minicom -b "${BAUD}" -D "${PORT}"
elif command -v screen &> /dev/null; then
    echo "Launching telemetry tracking via screen. Exit terminal using Ctrl+A followed by Ctrl+K."
    $SUDO_CMD screen "${PORT}" "${BAUD}"
else
    echo "ERROR: Suitable serial communication engine (picocom, minicom, screen) missing."
    echo "Install via host manager tools: 'sudo apt install picocom'"
    exit 2
fi
