// ============================================================================
// FILE: rtl/accelerator/vxu_dma_ctrl.sv
// PROJECT: NeuroRV Edge - Phase 3 Vector AI Accelerator
// MODULE: vxu_dma_ctrl
// DESCRIPTION: DMA Controller for VXU vector memory transfers.
//              AXI-Lite style burst interface.
//              Supports:
//                - Vector burst read  (memory → VXU lane registers)
//                - Vector burst write (VXU lane registers → memory)
//              Burst length: BURST_LEN beats per transaction
//              Transfer granularity: DATA_W bits per beat
//
// STATE MACHINE:
//   IDLE → ADDR_PHASE → (READ_BURST | WRITE_BURST) → DONE → IDLE
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

module vxu_dma_ctrl #(
    parameter int VEC_LEN   = 256,
    parameter int DATA_W    = 16,
    parameter int ADDR_W    = 32,
    parameter int BURST_LEN = 16   // Number of DATA_W beats per burst
)(
    input  logic                  clk,
    input  logic                  rst_n,

    // VXU Internal Interface
    input  logic                  load_req,       // Request a vector load
    input  logic                  store_req,      // Request a vector store
    input  logic [ADDR_W-1:0]     load_addr,      // Source address for load
    input  logic [ADDR_W-1:0]     store_addr,     // Destination address for store
    input  logic [DATA_W-1:0]     store_data [0:VEC_LEN-1],  // Data to store
    output logic [DATA_W-1:0]     load_data  [0:VEC_LEN-1],  // Data loaded from memory
    output logic                  load_done,      // Load complete strobe
    output logic                  store_done,     // Store complete strobe

    // Memory / AXI-Lite Bus Interface
    output logic                  mem_req,        // Memory request
    output logic                  mem_wr,         // 1=write, 0=read
    output logic [ADDR_W-1:0]     mem_addr,       // Target address
    output logic [DATA_W-1:0]     mem_wdata [0:VEC_LEN-1],  // Write data
    input  logic [DATA_W-1:0]     mem_rdata [0:VEC_LEN-1],  // Read data
    input  logic                  mem_ack,        // Address phase acknowledged
    input  logic                  mem_valid       // Read data valid
);

    // =========================================================================
    // DMA FSM State Encoding
    // =========================================================================
    typedef enum logic [2:0] {
        DMA_IDLE       = 3'h0,
        DMA_ADDR       = 3'h1,
        DMA_READ_BURST = 3'h2,
        DMA_READ_DONE  = 3'h3,
        DMA_WRITE_PREP = 3'h4,
        DMA_WRITE_BURST= 3'h5,
        DMA_WRITE_DONE = 3'h6
    } dma_state_t;

    dma_state_t state, next_state;

    // =========================================================================
    // Internal Registers
    // =========================================================================
    logic [ADDR_W-1:0]   cur_addr;
    logic [$clog2(VEC_LEN)-1:0] beat_cnt;     // Beats completed in burst
    logic [$clog2(VEC_LEN)-1:0] total_beats;  // Total beats needed (VEC_LEN)
    logic                is_write;

    // Local data buffer for loaded data
    logic [DATA_W-1:0] rx_buffer [0:VEC_LEN-1];

    // =========================================================================
    // Total beats: VEC_LEN lanes, one beat per lane
    // =========================================================================
    assign total_beats = VEC_LEN - 1;

    // =========================================================================
    // FSM: Sequential
    // =========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state      <= DMA_IDLE;
            cur_addr   <= '0;
            beat_cnt   <= '0;
            is_write   <= 1'b0;
            load_done  <= 1'b0;
            store_done <= 1'b0;
        end else begin
            state      <= next_state;
            load_done  <= 1'b0;
            store_done <= 1'b0;

            case (state)
                DMA_IDLE: begin
                    beat_cnt <= '0;
                    if (load_req) begin
                        cur_addr <= load_addr;
                        is_write <= 1'b0;
                    end else if (store_req) begin
                        cur_addr <= store_addr;
                        is_write <= 1'b1;
                    end
                end

                DMA_ADDR: begin
                    if (mem_ack) begin
                        // Address phase done, begin data phase
                        beat_cnt <= '0;
                    end
                end

                DMA_READ_BURST: begin
                    if (mem_valid) begin
                        rx_buffer[beat_cnt] <= mem_rdata[beat_cnt];
                        if (beat_cnt != total_beats[$clog2(VEC_LEN+1)-1:0]) begin
                            beat_cnt <= beat_cnt + 1'b1;
                            cur_addr <= cur_addr + (DATA_W / 8);
                        end
                    end
                end

                DMA_READ_DONE: begin
                    for (int i = 0; i < VEC_LEN; i++) load_data[i] <= rx_buffer[i];
                    load_done <= 1'b1;
                end

                DMA_WRITE_PREP: begin
                    beat_cnt <= '0;
                    // Copy store_data to wdata (registered)
                end

                DMA_WRITE_BURST: begin
                    if (mem_ack) begin
                        if (beat_cnt != total_beats[$clog2(VEC_LEN+1)-1:0]) begin
                            beat_cnt <= beat_cnt + 1'b1;
                            cur_addr <= cur_addr + (DATA_W / 8);
                        end
                    end
                end

                DMA_WRITE_DONE: begin
                    store_done <= 1'b1;
                end

                default: ;
            endcase
        end
    end

    // =========================================================================
    // FSM: Combinational Next-State Logic
    // =========================================================================
    always_comb begin
        next_state = state;
        case (state)
            DMA_IDLE: begin
                if      (load_req)  next_state = DMA_ADDR;
                else if (store_req) next_state = DMA_ADDR;
            end
            DMA_ADDR: begin
                if (mem_ack) begin
                    if (is_write) next_state = DMA_WRITE_PREP;
                    else          next_state = DMA_READ_BURST;
                end
            end
            DMA_READ_BURST: begin
                if (mem_valid && (beat_cnt == total_beats[$clog2(VEC_LEN+1)-1:0]))
                    next_state = DMA_READ_DONE;
            end
            DMA_READ_DONE:  next_state = DMA_IDLE;
            DMA_WRITE_PREP: next_state = DMA_WRITE_BURST;
            DMA_WRITE_BURST: begin
                if (mem_ack && beat_cnt == $bits(beat_cnt)'(total_beats))
                    next_state = DMA_WRITE_DONE;
            end
            DMA_WRITE_DONE: next_state = DMA_IDLE;
            default:        next_state = DMA_IDLE;
        endcase
    end

    // =========================================================================
    // Memory Bus Output Assignments
    // =========================================================================
    always_comb begin
        mem_req   = 1'b0;
        mem_wr    = 1'b0;
        mem_addr  = cur_addr;
        for (int i = 0; i < VEC_LEN; i++) mem_wdata[i] = store_data[i];

        case (state)
            DMA_ADDR: begin
                mem_req = 1'b1;
                mem_wr  = is_write;
            end
            DMA_READ_BURST: begin
                mem_req = 1'b1;
                mem_wr  = 1'b0;
            end
            DMA_WRITE_BURST: begin
                mem_req = 1'b1;
                mem_wr  = 1'b1;
                for (int i = 0; i < VEC_LEN; i++)
                    mem_wdata[i] = store_data[i]; // All lanes written in burst
            end
            default: begin
                mem_req = 1'b0;
                mem_wr  = 1'b0;
            end
        endcase
    end

    // =========================================================================
    // Simulation checks
    // =========================================================================
    // synthesis translate_off
    always @(posedge clk) begin
        if (load_req && store_req)
            $warning("vxu_dma_ctrl: load_req and store_req asserted simultaneously — load takes priority");
    end

    initial begin
        assert (VEC_LEN > 0 && (VEC_LEN % BURST_LEN == 0))
            else $fatal(1, "vxu_dma_ctrl: VEC_LEN must be divisible by BURST_LEN");
        assert (DATA_W == 8 || DATA_W == 16 || DATA_W == 32)
            else $fatal(1, "vxu_dma_ctrl: DATA_W must be 8, 16, or 32");
    end
    // synthesis translate_on

endmodule

`default_nettype wire
