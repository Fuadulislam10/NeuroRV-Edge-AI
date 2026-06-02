// ============================================================================
// FILE: rtl/accelerator/vxu_top.sv
// PROJECT: NeuroRV Edge - Phase 3 Vector AI Accelerator
// MODULE: vxu_top
// DESCRIPTION: Top-level Vector Execution Unit (VXU) integrating all
//              sub-units: MAC Array, Activation, Pooling, Norm, DMA
// COMPATIBLE: Yosys + Verilator | FPGA + ASIC
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

module vxu_top #(
    parameter int VEC_LEN       = 256,   // Vector lane count
    parameter int DATA_W        = 16,    // Data width (8 or 16)
    parameter int ACCUM_W       = 40,    // Accumulator width
    parameter int ADDR_W        = 32,    // Address width
    parameter int REG_W         = 32,    // Config register width
    parameter int NUM_REGS      = 16,    // Number of config registers
    parameter int DMA_BURST     = 16,    // DMA burst length
    parameter int POOL_MAX_SIZE = 4      // Max pooling window (4x4)
)(
    // Global
    input  logic                clk,
    input  logic                rst_n,

    // CPU Control Register Interface (AXI-Lite style, simplified)
    input  logic                cfg_wr_en,
    input  logic                cfg_rd_en,
    input  logic [3:0]          cfg_addr,
    input  logic [REG_W-1:0]    cfg_wdata,
    output logic [REG_W-1:0]    cfg_rdata,
    output logic                cfg_ack,

    // DMA / Memory Interface
    output logic                dma_req,
    output logic                dma_wr,
    output logic [ADDR_W-1:0]   dma_addr,
    output logic [DATA_W-1:0]   dma_wdata [0:VEC_LEN-1],
    input  logic [DATA_W-1:0]   dma_rdata [0:VEC_LEN-1],
    input  logic                dma_ack,
    input  logic                dma_valid,

    // Interrupt / Status
    output logic                vxu_irq,
    output logic                vxu_busy,
    output logic                vxu_done,

    // Debug
    output logic [31:0]         dbg_cycle_count,
    output logic [VEC_LEN-1:0]  dbg_lane_active,
    output logic [3:0]          dbg_op_mode
);

    // =========================================================================
    // Internal Register Map
    // REG[0]  = CTRL  : [0]=start [1]=mode_sel[1:0] [3:2]=act_sel [5:4]=pool_sel
    // REG[1]  = STATUS: [0]=busy [1]=done [2]=error
    // REG[2]  = SRC_ADDR_A  (operand A base address)
    // REG[3]  = SRC_ADDR_B  (operand B / weight base address)
    // REG[4]  = DST_ADDR    (result base address)
    // REG[5]  = VEC_COUNT   (number of vectors to process)
    // REG[6]  = NORM_MEAN   (Q8.8 fixed point mean)
    // REG[7]  = NORM_VAR    (Q8.8 fixed point variance scale)
    // REG[8]  = LEAKY_ALPHA (Q0.8 fixed point alpha for Leaky ReLU)
    // REG[9]  = POOL_CFG    [1:0]=window size (0=2x2, 1=4x4)
    // REG[10] = CYCLE_LIMIT (watchdog limit)
    // REG[11-15] = reserved
    // =========================================================================

    logic [REG_W-1:0] regs [0:NUM_REGS-1];

    // Decoded control fields
    logic        ctrl_start;
    logic [1:0]  ctrl_mode;   // 00=MAC, 01=POOL, 10=NORM, 11=PASSTHRU
    logic [1:0]  ctrl_act;    // 00=None, 01=ReLU, 10=LeakyReLU, 11=Sigmoid
    logic [1:0]  ctrl_pool;   // 00=Max2x2, 01=Avg2x2, 10=Max4x4, 11=Avg4x4
    logic [ADDR_W-1:0] src_addr_a, src_addr_b, dst_addr;
    logic [REG_W-1:0]  vec_count;
    logic [15:0] norm_mean, norm_var;
    logic [7:0]  leaky_alpha;
    logic [1:0]  pool_cfg;

    assign ctrl_start  = regs[0][0];
    assign ctrl_mode   = regs[0][2:1];
    assign ctrl_act    = regs[0][4:3];
    assign ctrl_pool   = regs[0][6:5];
    assign src_addr_a  = regs[2];
    assign src_addr_b  = regs[3];
    assign dst_addr    = regs[4];
    assign vec_count   = regs[5];
    assign norm_mean   = regs[6][15:0];
    assign norm_var    = regs[7][15:0];
    assign leaky_alpha = regs[8][7:0];
    assign pool_cfg    = regs[9][1:0];

    // =========================================================================
    // Register File Read/Write
    // =========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int i = 0; i < NUM_REGS; i++) regs[i] <= '0;
            cfg_ack   <= 1'b0;
            cfg_rdata <= '0;
        end else begin
            cfg_ack <= 1'b0;
            if (cfg_wr_en) begin
                regs[cfg_addr] <= cfg_wdata;
                cfg_ack <= 1'b1;
            end else if (cfg_rd_en) begin
                cfg_rdata <= regs[cfg_addr];
                cfg_ack   <= 1'b1;
            end
            // Auto-clear start bit after one cycle
            if (ctrl_start) regs[0][0] <= 1'b0;
            // Status writeback
            regs[1][0] <= vxu_busy;
            regs[1][1] <= vxu_done;
        end
    end

    // =========================================================================
    // Internal Datapath Signals
    // =========================================================================

    // MAC Array
    logic                     mac_start, mac_done, mac_valid_in, mac_valid_out;
    logic [DATA_W-1:0]        mac_a     [0:VEC_LEN-1];
    logic [DATA_W-1:0]        mac_b     [0:VEC_LEN-1];
    logic [ACCUM_W-1:0]       mac_result[0:VEC_LEN-1];
    logic [1:0]               mac_dtype; // 0=INT8, 1=INT16

    // Activation Unit
    logic                     act_valid_in, act_valid_out;
    logic [ACCUM_W-1:0]       act_data_in [0:VEC_LEN-1];
    logic [DATA_W-1:0]        act_data_out[0:VEC_LEN-1];

    // Pooling Unit
    logic                     pool_valid_in, pool_valid_out;
    logic [DATA_W-1:0]        pool_data_in [0:VEC_LEN-1];
    logic [DATA_W-1:0]        pool_data_out[0:VEC_LEN/4-1];

    // Norm Unit
    logic                     norm_valid_in, norm_valid_out;
    logic [DATA_W-1:0]        norm_data_in [0:VEC_LEN-1];
    logic [DATA_W-1:0]        norm_data_out[0:VEC_LEN-1];

    // DMA Controller
    logic                     dma_load_req, dma_store_req;
    logic                     dma_load_done, dma_store_done;
    logic [ADDR_W-1:0]        dma_load_addr, dma_store_addr;
    logic [DATA_W-1:0]        dma_load_data [0:VEC_LEN-1];
    logic [DATA_W-1:0]        dma_store_data[0:VEC_LEN-1];

    // =========================================================================
    // FSM: VXU Master Sequencer
    // =========================================================================
    typedef enum logic [3:0] {
        IDLE        = 4'h0,
        FETCH_A     = 4'h1,
        FETCH_B     = 4'h2,
        EXECUTE_MAC = 4'h3,
        EXECUTE_POOL= 4'h4,
        EXECUTE_NORM= 4'h5,
        ACTIVATE    = 4'h6,
        WRITEBACK   = 4'h7,
        COMPLETE    = 4'h8
    } vxu_state_t;

    vxu_state_t state, next_state;
    logic [31:0] cycle_cnt;
    logic [31:0] vec_processed;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state         <= IDLE;
            cycle_cnt     <= '0;
            vec_processed <= '0;
        end else begin
            state <= next_state;
            if (state != IDLE) cycle_cnt <= cycle_cnt + 1;
            else               cycle_cnt <= '0;
        end
    end

    always_comb begin
        next_state     = state;
        mac_start      = 1'b0;
        dma_load_req   = 1'b0;
        dma_store_req  = 1'b0;
        mac_valid_in   = 1'b0;
        act_valid_in   = 1'b0;
        pool_valid_in  = 1'b0;
        norm_valid_in  = 1'b0;

        case (state)
            IDLE: begin
                if (ctrl_start) next_state = FETCH_A;
            end
            FETCH_A: begin
                dma_load_req = 1'b1;
                if (dma_load_done) begin
                    if      (ctrl_mode == 2'b00) next_state = FETCH_B;
                    else if (ctrl_mode == 2'b01) next_state = EXECUTE_POOL;
                    else if (ctrl_mode == 2'b10) next_state = EXECUTE_NORM;
                    else                          next_state = ACTIVATE;
                end
            end
            FETCH_B: begin
                dma_load_req = 1'b1;
                if (dma_load_done) next_state = EXECUTE_MAC;
            end
            EXECUTE_MAC: begin
                mac_start    = 1'b1;
                mac_valid_in = 1'b1;
                if (mac_done) begin
                    if (ctrl_act != 2'b00) next_state = ACTIVATE;
                    else                   next_state = WRITEBACK;
                end
            end
            EXECUTE_POOL: begin
                pool_valid_in = 1'b1;
                if (pool_valid_out) next_state = WRITEBACK;
            end
            EXECUTE_NORM: begin
                norm_valid_in = 1'b1;
                if (norm_valid_out) begin
                    if (ctrl_act != 2'b00) next_state = ACTIVATE;
                    else                   next_state = WRITEBACK;
                end
            end
            ACTIVATE: begin
                act_valid_in = 1'b1;
                if (act_valid_out) next_state = WRITEBACK;
            end
            WRITEBACK: begin
                dma_store_req = 1'b1;
                if (dma_store_done) next_state = COMPLETE;
            end
            COMPLETE: begin
                next_state = IDLE;
            end
            default: next_state = IDLE;
        endcase
    end

    // Status outputs
    assign vxu_busy = (state != IDLE && state != COMPLETE);
    assign vxu_done = (state == COMPLETE);
    assign vxu_irq  = (state == COMPLETE);

    // Debug outputs
    assign dbg_cycle_count = cycle_cnt;
    assign dbg_op_mode     = {ctrl_act, ctrl_mode};
    assign dbg_lane_active = mac_valid_in ? {VEC_LEN{1'b1}} : '0;

    // DMA address mux
    assign dma_load_addr  = (state == FETCH_A) ? src_addr_a : src_addr_b;
    assign dma_store_addr = dst_addr;

    // Activation input mux: MAC result or norm output
    always_comb begin
        for (int i = 0; i < VEC_LEN; i++) begin
            act_data_in[i] = (ctrl_mode == 2'b10) ?
                             {{(ACCUM_W-DATA_W){norm_data_out[i][DATA_W-1]}}, norm_data_out[i]} :
                             mac_result[i];
        end
    end

    // Writeback data mux
    always_comb begin
        if (ctrl_act != 2'b00) begin
            for (int i = 0; i < VEC_LEN; i++) dma_store_data[i] = act_data_out[i];
        end else if (ctrl_mode == 2'b01) begin
            for (int i = 0; i < VEC_LEN/4; i++) dma_store_data[i] = pool_data_out[i];
            for (int i = VEC_LEN/4; i < VEC_LEN; i++) dma_store_data[i] = '0;
        end else if (ctrl_mode == 2'b10) begin
            for (int i = 0; i < VEC_LEN; i++) dma_store_data[i] = norm_data_out[i];
        end else begin
            for (int i = 0; i < VEC_LEN; i++)
                dma_store_data[i] = mac_result[i][DATA_W-1:0];
        end
    end

    // MAC inputs from DMA loaded data
    always_comb begin
        for (int i = 0; i < VEC_LEN; i++) begin
            mac_a[i] = dma_load_data[i];
            mac_b[i] = dma_load_data[i]; // second fetch overwrites
        end
    end

    assign pool_data_in = dma_load_data;
    assign norm_data_in = dma_load_data;

    assign mac_dtype = (DATA_W == 8) ? 2'b00 : 2'b01;

    // =========================================================================
    // Sub-module Instantiations
    // =========================================================================

    mac_array #(
        .VEC_LEN (VEC_LEN),
        .DATA_W  (DATA_W),
        .ACCUM_W (ACCUM_W)
    ) u_mac_array (
        .clk       (clk),
        .rst_n     (rst_n),
        .start     (mac_start),
        .dtype     (mac_dtype),
        .a         (mac_a),
        .b         (mac_b),
        .valid_in  (mac_valid_in),
        .result    (mac_result),
        .valid_out (mac_valid_out),
        .done      (mac_done)
    );

    activation_unit #(
        .VEC_LEN (VEC_LEN),
        .DATA_IN_W (ACCUM_W),
        .DATA_OUT_W(DATA_W)
    ) u_act (
        .clk       (clk),
        .rst_n     (rst_n),
        .act_sel   (ctrl_act),
        .alpha     (leaky_alpha),
        .valid_in  (act_valid_in),
        .data_in   (act_data_in),
        .valid_out (act_valid_out),
        .data_out  (act_data_out)
    );

    pooling_unit #(
        .VEC_LEN (VEC_LEN),
        .DATA_W  (DATA_W)
    ) u_pool (
        .clk       (clk),
        .rst_n     (rst_n),
        .pool_mode (ctrl_pool),
        .valid_in  (pool_valid_in),
        .data_in   (pool_data_in),
        .valid_out (pool_valid_out),
        .data_out  (pool_data_out)
    );

    norm_unit #(
        .VEC_LEN (VEC_LEN),
        .DATA_W  (DATA_W)
    ) u_norm (
        .clk       (clk),
        .rst_n     (rst_n),
        .mean      (norm_mean),
        .inv_std   (norm_var),
        .valid_in  (norm_valid_in),
        .data_in   (norm_data_in),
        .valid_out (norm_valid_out),
        .data_out  (norm_data_out)
    );

    vxu_dma_ctrl #(
        .VEC_LEN    (VEC_LEN),
        .DATA_W     (DATA_W),
        .ADDR_W     (ADDR_W),
        .BURST_LEN  (DMA_BURST)
    ) u_dma (
        .clk         (clk),
        .rst_n       (rst_n),
        .load_req    (dma_load_req),
        .store_req   (dma_store_req),
        .load_addr   (dma_load_addr),
        .store_addr  (dma_store_addr),
        .store_data  (dma_store_data),
        .load_data   (dma_load_data),
        .load_done   (dma_load_done),
        .store_done  (dma_store_done),
        // Memory bus
        .mem_req     (dma_req),
        .mem_wr      (dma_wr),
        .mem_addr    (dma_addr),
        .mem_wdata   (dma_wdata),
        .mem_rdata   (dma_rdata),
        .mem_ack     (dma_ack),
        .mem_valid   (dma_valid)
    );

endmodule

`default_nettype wire
