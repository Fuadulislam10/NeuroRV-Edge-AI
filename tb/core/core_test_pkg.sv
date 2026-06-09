// =============================================================================
// Project Name: NeuroRV Edge
// Module Name:  core_test_pkg
// Description:  Package containing types, tasks, and configurations for
//               CPU Core Verification.
// =============================================================================

package core_test_pkg;
    typedef struct packed {
        logic [31:0] inst;
        logic [31:0] pc;
        logic        valid;
    } trace_t;

    typedef enum logic [3:0] {
        TEST_RESET     = 4'b0000,
        TEST_ALU       = 4'b0001,
        TEST_BRANCH    = 4'b0010,
        TEST_JUMP      = 4'b0011,
        TEST_LOAD_STORE= 4'b0100,
        TEST_MULDIV    = 4'b0101,
        TEST_HAZARDS   = 4'b0110,
        TEST_STALLS    = 4'b0111
    } test_case_e;
endpackage : core_test_pkg
