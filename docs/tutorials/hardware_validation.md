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

---

1. Convert the compiled elf layout binary into a standard memory layout execution array image file.

2. Flash the target layout update to the development platform board.

3. Verify that the on-board user LEDs match the target output sequence pattern (0x55 toggling with 0xAA). Adjust the hardware slide switches to confirm that the internal delay calculation dynamically updates.

Executing VAL-005 (Co-Processor Execution Testing)
This step exercises the vector pipeline arrays, vector register file structures, and parallel mathematical elements.

1. Deploy vxu_demo.c to the execution space.

2. Monitor the physical hardware LEDs.

3. If the vector math processing checks execute successfully, the core drives the validation status token 0xAA (binary 10101010) directly across the user LEDs. If a mathematical or processing fault occurs, the status LEDs will display 0xFF, indicating a failure.
