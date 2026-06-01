// =============================================================================
// NeuroRV Edge — Memory Access (MEM) Stage
// File   : rtl/core/mem_stage.sv
// =============================================================================
`timescale 1ns/1ps
`default_nettype none

module mem_stage (
    input  logic        clk_i,
    input  logic        rst_ni,
    // EX/MEM inputs
    input  logic [31:0] ex_mem_pc_i,
    input  logic [31:0] ex_mem_alu_result_i,  // memory address or result
    input  logic [31:0] ex_mem_rs2_data_i,    // store data
    input  logic [4:0]  ex_mem_rd_addr_i,
    input  logic [2:0]  ex_mem_funct3_i,
    input  logic        ex_mem_reg_write_i,
    input  logic        ex_mem_mem_read_i,
    input  logic        ex_mem_mem_write_i,
    input  logic        ex_mem_valid_i,
    // Data memory interface
    output logic [31:0] dmem_addr_o,
    output logic [31:0] dmem_wdata_o,
    output logic [3:0]  dmem_be_o,
    output logic        dmem_we_o,
    output logic        dmem_req_o,
    input  logic [31:0] dmem_rdata_i,
    input  logic        dmem_gnt_i,
    input  logic        dmem_rvalid_i,
    input  logic        dmem_err_i,
    // MEM/WB outputs
    output logic [31:0] mem_wb_pc_o,
    output logic [31:0] mem_wb_alu_result_o,
    output logic [31:0] mem_wb_mem_rdata_o,
    output logic [4:0]  mem_wb_rd_addr_o,
    output logic        mem_wb_reg_write_o,
    output logic        mem_wb_mem_to_reg_o,
    output logic        mem_wb_valid_o
);

    logic [31:0] addr;
    logic [1:0]  byte_offset;
    logic [31:0] aligned_rdata;
    logic [31:0] wdata_aligned;

    assign addr        = ex_mem_alu_result_i;
    assign byte_offset = addr[1:0];

    // Memory request
    assign dmem_addr_o = {addr[31:2], 2'b00}; // word-aligned
    assign dmem_req_o  = (ex_mem_mem_read_i || ex_mem_mem_write_i) && ex_mem_valid_i;
    assign dmem_we_o   = ex_mem_mem_write_i && ex_mem_valid_i;

    // Store byte enable and data alignment
    always_comb begin
        dmem_be_o    = 4'b0000;
        wdata_aligned = '0;
        unique case (ex_mem_funct3_i)
            3'b000: begin // SB
                dmem_be_o = 4'b0001 << byte_offset;
                wdata_aligned = {24'h0, ex_mem_rs2_data_i[7:0]} << (byte_offset * 8);
            end
            3'b001: begin // SH
                dmem_be_o = 4'b0011 << (byte_offset & 2'b10);
                wdata_aligned = {16'h0, ex_mem_rs2_data_i[15:0]} << (byte_offset[1] ? 16 : 0);
            end
            3'b010: begin // SW
                dmem_be_o = 4'b1111;
                wdata_aligned = ex_mem_rs2_data_i;
            end
            default: begin
                dmem_be_o = 4'b1111;
                wdata_aligned = ex_mem_rs2_data_i;
            end
        endcase
    end
    assign dmem_wdata_o = wdata_aligned;

    // Load data sign/zero extension
    always_comb begin
        aligned_rdata = '0;
        unique case (ex_mem_funct3_i)
            3'b000: // LB — sign extend byte
                aligned_rdata = {{24{dmem_rdata_i[7 + byte_offset*8]}},
                                  dmem_rdata_i[7 + byte_offset*8 -: 8]};
            3'b001: // LH — sign extend half
                aligned_rdata = {{16{dmem_rdata_i[15 + byte_offset[1]*16]}},
                                  dmem_rdata_i[15 + byte_offset[1]*16 -: 16]};
            3'b010: // LW
                aligned_rdata = dmem_rdata_i;
            3'b100: // LBU — zero extend
                aligned_rdata = {24'h0, dmem_rdata_i[7 + byte_offset*8 -: 8]};
            3'b101: // LHU — zero extend
                aligned_rdata = {16'h0, dmem_rdata_i[15 + byte_offset[1]*16 -: 16]};
            default:
                aligned_rdata = dmem_rdata_i;
        endcase
    end

    // MEM/WB pipeline register
    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            mem_wb_pc_o         <= '0;
            mem_wb_alu_result_o <= '0;
            mem_wb_mem_rdata_o  <= '0;
            mem_wb_rd_addr_o    <= '0;
            mem_wb_reg_write_o  <= '0;
            mem_wb_mem_to_reg_o <= '0;
            mem_wb_valid_o      <= '0;
        end else begin
            mem_wb_pc_o         <= ex_mem_pc_i;
            mem_wb_alu_result_o <= ex_mem_alu_result_i;
            mem_wb_mem_rdata_o  <= dmem_rvalid_i ? aligned_rdata : '0;
            mem_wb_rd_addr_o    <= ex_mem_rd_addr_i;
            mem_wb_reg_write_o  <= ex_mem_reg_write_i;
            mem_wb_mem_to_reg_o <= ex_mem_mem_read_i;
            mem_wb_valid_o      <= ex_mem_valid_i;
        end
    end

endmodule

`default_nettype wire


// =============================================================================
// NeuroRV Edge — Write-Back (WB) Stage
// File   : rtl/core/wb_stage.sv
// =============================================================================
`timescale 1ns/1ps
`default_nettype none

module wb_stage (
    input  logic [31:0] mem_wb_alu_result_i,
    input  logic [31:0] mem_wb_mem_rdata_i,
    input  logic [4:0]  mem_wb_rd_addr_i,
    input  logic        mem_wb_reg_write_i,
    input  logic        mem_wb_mem_to_reg_i,
    input  logic        mem_wb_valid_i,
    output logic [4:0]  rf_rd_o,
    output logic [31:0] rf_wdata_o,
    output logic        rf_we_o
);
    assign rf_rd_o    = mem_wb_rd_addr_i;
    assign rf_wdata_o = mem_wb_mem_to_reg_i ? mem_wb_mem_rdata_i : mem_wb_alu_result_i;
    assign rf_we_o    = mem_wb_reg_write_i && mem_wb_valid_i && (mem_wb_rd_addr_i != 5'h0);

endmodule

`default_nettype wire
