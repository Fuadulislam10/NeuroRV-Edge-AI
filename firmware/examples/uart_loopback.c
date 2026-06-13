// =============================================================================
// Project:     NeuroRV Edge
// File:        uart_loopback.c
// Description: Implements a hardware diagnostics serial interaction layer,
//              echoing processing terminal input directly back to receiver frames.
// =============================================================================

#include <stdint.h>

#define UART_BASE_ADDR   0x80001000
#define UART_DATA_REG    ((volatile uint32_t*)(UART_BASE_ADDR + 0x00))
#define UART_STATUS_REG  ((volatile uint32_t*)(UART_BASE_ADDR + 0x04))

#define UART_RX_EMPTY_MASK 0x02
#define UART_TX_FULL_MASK  0x01

char uart_getchar(void) {
    // Spin until structural RX empty status flag clears (data arrived)
    while ((*UART_STATUS_REG) & UART_RX_EMPTY_MASK) {
        __asm__ volatile("nop");
    }
    return (char)(*UART_DATA_REG & 0xFF);
}

void uart_putchar(char c) {
    while ((*UART_STATUS_REG) & UART_TX_FULL_MASK) {
        __asm__ volatile("nop");
    }
    *UART_DATA_REG = (uint32_t)c;
}

void uart_print(const char *str) {
    while (*str) {
        uart_putchar(*str++);
    }
}

int main(void) {
    uart_print("Entering Echo Interactive loop... Type characters into terminal.\r\n");

    while (1) {
        char rx_byte = uart_getchar();
        
        // Formatting transformation intercept: translate carriage returns appropriately
        if (rx_byte == '\r') {
            uart_putchar('\r');
            uart_putchar('\n');
        } else {
            // Standard execution mode: mirror inputs back to outputs
            uart_putchar(rx_byte);
        }
    }
    return 0;
}
