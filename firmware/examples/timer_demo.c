// =============================================================================
// Project:     NeuroRV Edge
// File:        timer_demo.c
// Description: Registers, tracks, and handles standard core hardware interrupt
//              timers via the Machine Timer register architecture space.
// =============================================================================

#include <stdint.h>

// Core RISC-V standard machine timer system registers
#define MTIMER_BASE_ADDR  0x80002000
#define MTIME_LOW_REG     ((volatile uint32_t*)(MTIMER_BASE_ADDR + 0x00))
#define MTIME_HIGH_REG    ((volatile uint32_t*)(MTIMER_BASE_ADDR + 0x04))
#define MTIMECMP_LOW_REG  ((volatile uint32_t*)(MTIMER_BASE_ADDR + 0x08))
#define MTIMECMP_HIGH_REG ((volatile uint32_t*)(MTIMER_BASE_ADDR + 0x0C))

#define UART_BASE_ADDR    0x80001000
#define UART_TX_REG       ((volatile uint32_t*)(UART_BASE_ADDR + 0x00))

#define TIMER_INTERVAL    5000000 // Assumes a 50MHz baseline clock (100ms ticks)

volatile uint32_t interrupt_counter = 0;

// Embedded raw vector handler routing function decoration
void handle_trap(void) __attribute__((interrupt("machine")));

void handle_trap(void) {
    // Read the current absolute time profile block
    uint32_t current_time_low = *MTIME_LOW_REG;
    
    // Clear and schedule the next interrupt milestone match boundaries
    uint32_t next_trigger = current_time_low + TIMER_INTERVAL;
    *MTIMECMP_LOW_REG = next_trigger;
    
    interrupt_counter++;
    
    // Diagnostic output token inside the execution trap context
    *UART_TX_REG = 'T'; 
}

void init_timer(void) {
    // Disable comparison registers temporarily to avoid triggering a race condition
    *MTIMECMP_LOW_REG  = 0xFFFFFFFF;
    *MTIMECMP_HIGH_REG = 0xFFFFFFFF;
    
    // Clear the core operational time counters
    *MTIME_LOW_REG  = 0;
    *MTIME_HIGH_REG = 0;
    
    // Prime values for initial interrupt sequence match
    *MTIMECMP_LOW_REG  = TIMER_INTERVAL;
    *MTIMECMP_HIGH_REG = 0;
    
    // Write setup flags to Machine Status and Machine Interrupt Enable registers
    // MIE Bit 7 points directly to Machine Timer Interrupt Enablers (MTIE)
    __asm__ volatile("csrw mtvec, %0" :: "r"(handle_trap));
    __asm__ volatile("li t0, 0x80\n\t"
                     "csrs mie, t0");
    __asm__ volatile("li t0, 0x08\n\t"
                     "csrs mstatus, t0"); // Enable Global Machine Interrupts (MIE)
}

int main(void) {
    init_timer();
    
    while (1) {
        // Spin idling, waiting for the hardware timer interrupts to fire
        if (interrupt_counter >= 50) {
            // Once criteria thresholds execute, break configuration patterns
            __asm__ volatile("csrc mstatus, 0x08"); // Turn off global system interrupts
            break;
        }
    }
    
    return 0;
}
