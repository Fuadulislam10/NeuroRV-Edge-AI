# Post-Synthesis Automated Hardware Validation Protocols

This test plan defines the validation procedures for confirming structural integrity, peripheral functionality, and execution accuracy on the `NeuroRV Edge` SoC platform.

---

## Hardware Validation Matrix

| Test ID | Targeted Component Block | Validation Firmware | Success Criteria |
| :--- | :--- | :--- | :--- |
| **VAL-001**| Boot ROM & System UART | `hello_world.c` | Initialization banner prints cleanly via serial console at 115200 baud. |
| **VAL-002**| System Crossbar & GPIO | `gpio_blink.c` | LEDs cycle through alternating binary patterns; toggle speed adapts when switches change. |
| **VAL-003**| Full Duplex UART Bus | `uart_loopback.c` | Host keyboard strokes are echoed back back to the host terminal without distortion. |
| **VAL-004**| RISC-V Core Timer Blocks | `timer_demo.c` | The core successfully processes machine timer interrupts, printing 'T' tokens to the console. |
| **VAL-005**| Vector Processing Unit | `vxu_demo.c` | Parallel execution array completes cleanly; LEDs display the completion token `0xAA`. |

---

## Execution Directives

### Executing VAL-002 (GPIO & Interconnect Interfacing)
1. Compile the target testing infrastructure using the cross-compiler toolset:
   ```bash
   riscv32-unknown-elf-gcc -O2 -march=rv32im -mabi=ilp32 -nostartfiles -T link.ld ../../firmware/examples/gpio_blink.c -o gpio_blink.elf
