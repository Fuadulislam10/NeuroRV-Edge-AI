// =============================================================================
// Project:     NeuroRV Edge
// File:        vxu_demo.c
// Description: Software validation tracking for Vector Acceleration coprocessor
//              units. Validates linear math functions across parallel elements.
// =============================================================================

#include <stdint.h>

// Vector Processing Unit Address Control Interfaces
#define VXU_BASE_ADDR     0x80005000
#define VXU_CTRL_REG      ((volatile uint32_t*)(VXU_BASE_ADDR + 0x00))
#define VXU_LEN_REG       ((volatile uint32_t*)(VXU_BASE_ADDR + 0x04))
#define VXU_VREG_A_PTR    ((volatile uint32_t*)(VXU_BASE_ADDR + 0x08))
#define VXU_VREG_B_PTR    ((volatile uint32_t*)(VXU_BASE_ADDR + 0x0C))
#define VXU_VREG_OUT_PTR  ((volatile uint32_t*)(VXU_BASE_ADDR + 0x10))

#define VXU_START_CMD     0x01
#define VXU_READY_MASK    0x02

#define VECTOR_SIZE       8

// Explicit arrays used as vector elements for processing validation
int32_t source_vector_a[VECTOR_SIZE] = {10,  20,  30,  40,  50,  60,  70,  80};
int32_t source_vector_b[VECTOR_SIZE] = {1,   2,   3,   4,   5,   6,   7,   8};
int32_t target_vector_o[VECTOR_SIZE] = {0};

int main(void) {
    // 1. Configure the mathematical parameters inside the VXU register spaces
    *VXU_LEN_REG      = VECTOR_SIZE;
    *VXU_VREG_A_PTR   = (uint32_t)source_vector_a;
    *VXU_VREG_B_PTR   = (uint32_t)source_vector_b;
    *VXU_VREG_OUT_PTR = (uint32_t)target_vector_o;

    // 2. Dispatch operational triggers to execute vector parallel addition
    // Command 0x01 flags parallel vector array arithmetic calculations
    *VXU_CTRL_REG = VXU_START_CMD;

    // 3. Loop until hardware status register signals operation complete
    while (!((*VXU_CTRL_REG) & VXU_READY_MASK)) {
        __asm__ volatile("nop");
    }

    // 4. Verification Check
    // If hardware works properly, index 0 result must reflect 10 + 1 = 11
    if (target_vector_o[0] == 11 && target_vector_o[7] == 88) {
        // Output confirmation write back to GPIO status mapping indicators
        *((volatile uint32_t*)0x80000000) = 0xAA; // 10101010 implies valid execution
    } else {
        *((volatile uint32_t*)0x80000000) = 0xFF; // 11111111 flags processing error
    }

    while (1) {
        __asm__ volatile("wfi");
    }
    return 0;
}
