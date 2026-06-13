// =============================================================================
// Project:     NeuroRV Edge
// File:        gpio_blink.c
// Description: Exercises the General Purpose Output structures, executing
//              alternating binary shift updates to board level user LEDs.
// =============================================================================

#include <stdint.h>

#define GPIO_BASE_ADDR   0x80000000
#define GPIO_LED_DATA    ((volatile uint32_t*)(GPIO_BASE_ADDR + 0x00))
#define GPIO_SW_DATA     ((volatile uint32_t*)(GPIO_BASE_ADDR + 0x04))

// Software execution delaying spin cycle loop
void execution_delay(volatile uint32_t counts) {
    while (counts > 0) {
        counts--;
        __asm__ volatile("nop");
    }
}

int main(void) {
    uint32_t pattern = 0x55; // Binary alternating pattern: 01010101

    while (1) {
        // Sample hardware input configuration switches to modulate speed dynamically
        uint32_t switch_inputs = *GPIO_SW_DATA;
        uint32_t variable_delay = 200000 + (switch_inputs * 20000);

        // Update hardware output data register target
        *GPIO_LED_DATA = pattern;

        // Bitwise logic flip to generate a smooth blinking cadence
        pattern = ~pattern & 0xFF;

        execution_delay(variable_delay);
    }

    return 0;
}
