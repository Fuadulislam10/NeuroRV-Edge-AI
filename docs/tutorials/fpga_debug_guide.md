## FILE: docs/tutorials/fpga_debug_guide.md
```markdown
# Triage Debugging Architecture Guide: NeuroRV Edge

This document details common failure profiles encountered during physical hardware bring-up and provides remediation workflows.

## Diagnostic Scenarios and Resolution Runbooks

### 1. Hardware Timing Constraint Failures
* **Symptoms:** System crashes under heavy compute operations, vector executions produce inconsistent outputs, or the `verify_fpga.sh` script throws timing slack errors.
* **Root Cause:** The logic depth of the multi-element Vector Processing Unit (VXU) exceeds the single-cycle threshold of the target clock frequency.
* **Remediation:** 1. Open `synthesis/fpga/clocking_wizard_config.tcl` and lower the system clock request from `50.000` to `40.000` MHz.
    2. Re-run `./build.sh` to update placement routines and verify that worst-case negative slack drops into positive values.

### 2. Quiet Serial Console (No UART Output)
* **Symptoms:** LEDs shift patterns normally, but the serial terminal displays no characters.
* **Root Cause:** Inverted physical Tx/Rx pin bindings or host-side device node node indexing mismatch.
* **Remediation:**
    1. Run `dmesg | grep tty` on your host Linux machine to verify if the board is mounted at `/dev/ttyUSB0`, `/dev/ttyUSB1`, or `/dev/ttyACM0`. Pass the correct interface directly to the monitoring script: `./run_uart_monitor.sh /dev/ttyUSB0`.
    2. Review `synthesis/fpga/constraints.xdc` to ensure physical pins match your target device footprint (e.g., Nexys A7 uses `C4` for RX and `D4` for TX).

### 3. System Fails to De-assert Reset
* **Symptoms:** System remains unresponsive, and `sys_reset_led` stays lit.
* **Root Cause:** The internal clock generation wizard cannot achieve locked status because the input reference clock is unstable, or the active-low reset button polarity is inverted.
* **Remediation:**
    * Verify the polarity of your hardware reset button. The Nexys A7 board features an active-low CPU reset button (pin `C12`), whereas common alternatives utilize active-high inputs. If deploying to an active-high board, update the `fpga_top.sv` logic wrapper accordingly:
    ```systemverilog
    assign raw_reset_n = btn_reset && mmcm_locked; // Active-high adaptation
    ```

### 4. Memory Initialization Mismatch
* **Symptoms:** Core processor starts executing invalid instructions right out of reset.
* **Root Cause:** Vivado failed to locate or bind the `memory_init.mem` configuration matrix to the internal block RAM primitives.
* **Remediation:** Ensure the synthesis tool identifies the memory format pattern correctly. Check the synthesis log files (`runme.log`) for warnings like: `WARNING: [Synth 8-2898] data file memory_init.mem ignored`. Verify that the `.mem` file uses valid hexadecimal values without any address syntax violations.
