// =============================================================================
// Project Name: NeuroRV Edge
// Module Name:  vxu_test_pkg
// Description:  Package containing configuration structures and definitions
//               for Vector Accelerator Unit verification.
// =============================================================================

package vxu_test_pkg;
    typedef enum logic [2:0] {
        VADD = 3'b000,
        VMUL = 3'b001,
        VMAC = 3'b010,
        VRELU= 3'b011,
        VSIG = 3'b100
    } v_op_e;

    typedef struct packed {
        v_op_e        op_type;
        logic [31:0]  src_addr_a;
        logic [31:0]  src_addr_b;
        logic [31:0]  dest_addr;
        logic [15:0]  vector_len;
    } vxu_cmd_t;
endpackage : vxu_test_pkg
