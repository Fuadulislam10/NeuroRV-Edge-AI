// =============================================================================
// Project:     NeuroRV Edge
// File:        hello_world.c
// Description: Core execution code targeting firmware initialization systems.
//              Outputs primary boot notifications across the system UART lines.
// =============================================================================

#include <stdint.h>

// Structural Memory-Mapped Base Target Register Mappings
#define UART_BASE_ADDR   0x80001000
#define UART_TX_REG      ((volatile uint32_t*)(UART_BASE_ADDR + 0x00))
#define UART_STATUS_REG  ((volatile uint32_t*)(UART_BASE_ADDR + 0x04))

// Status register masks
#define UART_TX_FULL_MASK 0x01

// Low-level hardware block wrapper to block-write characters
void uart_putchar(char c) {
    // Spin while TX FIFO buffer is completely full
    while ((*UART_STATUS_REG) & UART_TX_FULL_MASK) {
        __asm__ volatile("nop");
    }
    // Ship byte execution unit across the register interface
    *UART_TX_REG = (uint32_t)c;
}

void uart_print(const char *str) {
    while (*str) {
        uart_putchar(*str++);
    }
}

int main(void) {
    // Small baseline loop latency configuration delay to account for FPGA PLL adjustments
    for (volatile int i = 0; i < 50000; i++) {
        __asm__ volatile("nop");
    }

    uart_print("\r\n==================================================\r\n");
    uart_print("  NeuroRV Edge Core Boot Pipeline Initializing...\r\n");
    uart_print("  Target Core Architecture: RISC-V RV32IMVX\r\n");
    uart_print("  Status: Core System Running on FPGA Target Fabric\r\n");
    uart_print("==================================================\r\n");

    while (1) {
        // Core execution remains active and holding
        __asm__ volatile("wfi"); 
    }
    return 0;
}
