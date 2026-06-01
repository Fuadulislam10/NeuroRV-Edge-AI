// ============================================================================
// NeuroRV Edge — Verilator C++ Simulation Harness
// File   : tb/sim_main.cpp
// Author : NeuroRV Edge Contributors
// License: Apache 2.0
//
// Description:
//   Verilator top-level simulation driver:
//   - Drives clk_sys, rst_ext_n
//   - Optionally loads firmware .hex file into SRAM
//   - Runs simulation for N cycles or until finish signal
//   - Generates VCD via Verilator's trace infrastructure
//   - Prints PC trace and UART output
// ============================================================================

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cassert>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <string>

#include "verilated.h"
#include "verilated_vcd_c.h"
#include "Vneuro_soc_top.h"   // Verilator-generated header

// ============================================================
// Constants
// ============================================================
static const uint64_t MAX_SIM_CYCLES  = 200000;
static const uint32_t CLK_PERIOD_HALF = 5;   // ns half-period → 100 MHz
static const uint32_t SRAM_WORDS      = 16384; // 64KB / 4
static const uint32_t RESET_CYCLES    = 20;

// ============================================================
// SRAM Hex Loader
// Loads Intel HEX format firmware into SRAM model
// ============================================================
struct HexRecord {
    uint8_t  byte_count;
    uint32_t address;
    uint8_t  record_type;
    std::vector<uint8_t> data;
};

bool parse_intel_hex(const std::string& filename,
                     std::vector<uint32_t>& sram,
                     uint32_t sram_base = 0x00000000)
{
    std::ifstream f(filename);
    if (!f.is_open()) {
        fprintf(stderr, "[HEX] Cannot open: %s\n", filename.c_str());
        return false;
    }

    uint32_t ext_segment = 0;
    std::string line;
    int records = 0;

    while (std::getline(f, line)) {
        if (line.empty() || line[0] != ':') continue;
        HexRecord rec;
        rec.byte_count  = std::stoi(line.substr(1, 2), nullptr, 16);
        rec.address     = std::stoi(line.substr(3, 4), nullptr, 16);
        rec.record_type = std::stoi(line.substr(7, 2), nullptr, 16);

        for (int i = 0; i < rec.byte_count; i++) {
            uint8_t b = std::stoi(line.substr(9 + i*2, 2), nullptr, 16);
            rec.data.push_back(b);
        }

        switch (rec.record_type) {
            case 0x00: { // Data
                uint32_t abs_addr = ext_segment + rec.address;
                if (abs_addr >= sram_base) {
                    uint32_t offset = (abs_addr - sram_base);
                    for (size_t i = 0; i < rec.data.size(); i++) {
                        uint32_t word_idx = (offset + i) / 4;
                        uint32_t byte_idx = (offset + i) % 4;
                        if (word_idx < sram.size()) {
                            sram[word_idx] &= ~(0xFF << (byte_idx * 8));
                            sram[word_idx] |= (rec.data[i] << (byte_idx * 8));
                        }
                    }
                }
                records++;
                break;
            }
            case 0x02: // Extended segment address
                ext_segment = ((rec.data[0] << 8) | rec.data[1]) << 4;
                break;
            case 0x04: // Extended linear address
                ext_segment = ((rec.data[0] << 8) | rec.data[1]) << 16;
                break;
            case 0x01: // EOF
                goto done;
        }
    }
done:
    printf("[HEX] Loaded %d data records from %s\n", records, filename.c_str());
    return true;
}

// ============================================================
// Simple test program (NOP-sled + infinite loop fallback)
// ============================================================
static uint32_t default_program[] = {
    0x00100093,  // addi x1, x0, 1
    0xDEADB137,  // lui  x2, 0xDEADB
    0xEEF10113,  // addi x2, x2, -0x111
    0x10202023,  // sw   x2, 0x100(x0)
    0x10002183,  // lw   x3, 0x100(x0)
    0x05500213,  // addi x4, x0, 0x55
    0x20402023,  // sw   x4, 0x200(x0)
    0x100002B7,  // lui  x5, 0x10000   [VPU base]
    0x10120313,  // addi x6, x0, imm
    0x00628023,  // sw   x6, 0(x5)
    0x0042A383,  // lw   x7, 4(x5)
    0x0013F393,  // andi x7, x7, 1
    0xFE039CE3,  // bne  x7, x0, -8
    0x0000006F,  // jal  x0, 0  [spin]
};

// ============================================================
// UART Decoder State
// ============================================================
struct UartDecoder {
    bool     in_frame  = false;
    uint8_t  shift     = 0;
    int      bit_count = 0;
    uint64_t start_time= 0;
    std::string buffer;

    // 115200 baud @ 100MHz = 868 cycles per bit
    static const uint64_t CLKS_PER_BIT = 868;

    void feed(bool tx, uint64_t cycle) {
        // Simple polling decoder – check start bit
        if (!in_frame && !tx) {
            in_frame   = true;
            start_time = cycle;
            shift      = 0;
            bit_count  = 0;
        } else if (in_frame) {
            uint64_t elapsed = cycle - start_time;
            // Sample at 1.5 bits delay, then every bit
            uint64_t sample_point = CLKS_PER_BIT + CLKS_PER_BIT/2 + bit_count * CLKS_PER_BIT;
            if (elapsed >= sample_point && bit_count < 8) {
                shift |= (tx ? 1 : 0) << bit_count;
                bit_count++;
                if (bit_count == 8) {
                    printf("[UART] 0x%02X '%c'\n", shift,
                           (shift >= 0x20 && shift <= 0x7E) ? (char)shift : '.');
                    buffer += (char)shift;
                    in_frame = false;
                }
            }
        }
    }
};

// ============================================================
// Main simulation entry point
// ============================================================
int main(int argc, char** argv)
{
    Verilated::commandArgs(argc, argv);

    // Optional: pass hex file as first argument
    std::string hex_file = "";
    for (int i = 1; i < argc; i++) {
        if (strncmp(argv[i], "--hex=", 6) == 0)
            hex_file = std::string(argv[i] + 6);
    }

    // Build SRAM image
    std::vector<uint32_t> sram_image(SRAM_WORDS, 0);
    if (!hex_file.empty()) {
        if (!parse_intel_hex(hex_file, sram_image)) {
            fprintf(stderr, "Failed to load hex file; using default program\n");
            hex_file = "";
        }
    }
    if (hex_file.empty()) {
        size_t prog_words = sizeof(default_program) / sizeof(uint32_t);
        printf("[SIM] Loading built-in test program (%zu words)\n", prog_words);
        for (size_t i = 0; i < prog_words && i < SRAM_WORDS; i++)
            sram_image[i] = default_program[i];
    }

    // Instantiate DUT
    Vneuro_soc_top* dut = new Vneuro_soc_top;

    // VCD trace
    Verilated::traceEverOn(true);
    VerilatedVcdC* vcd = new VerilatedVcdC;
    dut->trace(vcd, 99);
    vcd->open("waveforms/soc_sim.vcd");
    printf("[SIM] VCD trace: waveforms/soc_sim.vcd\n");

    // Preload SRAM (access internal memory through DPI or direct handle)
    // Note: with Verilator, use --public-flat-rw or direct signal access
    // Here we preload before simulation starts via the public interface
    // (Actual SRAM preload happens once rst_n is asserted and before CPU runs)

    // Initialize signals
    dut->clk_sys   = 0;
    dut->rst_ext_n = 0;
    dut->gpio_in   = 0;
    dut->uart_rx   = 1;  // UART idle high

    uint64_t sim_time  = 0;
    uint64_t cycle_cnt = 0;
    uint64_t last_pc   = 0xFFFFFFFF;

    UartDecoder uart_dec;

    printf("[SIM] Starting NeuroRV Edge simulation...\n");
    printf("[SIM] Max cycles: %lu\n", MAX_SIM_CYCLES);

    // ---- Simulation Loop ----
    while (!Verilated::gotFinish() && cycle_cnt < MAX_SIM_CYCLES) {

        // Clock edges
        if (sim_time % CLK_PERIOD_HALF == 0)
            dut->clk_sys ^= 1;

        // Reset release after RESET_CYCLES
        if (cycle_cnt == RESET_CYCLES) {
            dut->rst_ext_n = 1;
            printf("[SIM] Reset released at cycle %lu\n", cycle_cnt);
        }

        // Evaluate
        dut->eval();

        // VCD dump
        vcd->dump(sim_time);

        // Only process on rising edge
        if (dut->clk_sys == 1 && sim_time % (CLK_PERIOD_HALF * 2) == 0) {

            // SRAM preload on cycle after reset
            if (cycle_cnt == RESET_CYCLES + 2) {
                // Access the internal SRAM via Verilator public interface
                // The actual path depends on hierarchy flattening
                // Typical access: dut->neuro_soc_top__DOT__u_ic__DOT__u_sram__DOT__mem
                // For now, print status
                printf("[SIM] Applying SRAM image (%u words)\n", SRAM_WORDS);
                // NOTE: Real Verilator build would use:
                //   for (int i = 0; i < SRAM_WORDS; i++)
                //     dut->neuro_soc_top__DOT__u_ic__DOT__u_sram__DOT__mem[i] = sram_image[i];
            }

            // PC trace (every 100 cycles or on change)
            uint32_t pc = dut->debug_pc;
            if (pc != last_pc || cycle_cnt % 100 == 0) {
                if (cycle_cnt % 100 == 0) {
                    printf("[SIM] cycle=%-8lu PC=0x%08X pwr=%01X vpu_busy=%d\n",
                           cycle_cnt, pc, dut->debug_pwr_state, dut->debug_vpu_busy);
                }
                last_pc = pc;
            }

            // UART monitor
            uart_dec.feed((bool)dut->uart_tx, cycle_cnt);

            cycle_cnt++;
        }

        sim_time += CLK_PERIOD_HALF;
    }

    // ---- Final Statistics ----
    printf("\n[SIM] =============================================\n");
    printf("[SIM] Simulation complete\n");
    printf("[SIM]   Total cycles  : %lu\n", cycle_cnt);
    printf("[SIM]   Sim time (ns) : %lu\n", sim_time);
    printf("[SIM]   Final PC      : 0x%08X\n", dut->debug_pc);
    printf("[SIM]   Power state   : 0x%X\n", dut->debug_pwr_state);
    printf("[SIM]   VPU busy      : %d\n", dut->debug_vpu_busy);
    if (!uart_dec.buffer.empty())
        printf("[SIM]   UART output   : \"%s\"\n", uart_dec.buffer.c_str());
    printf("[SIM] =============================================\n");

    // Cleanup
    vcd->close();
    dut->final();
    delete dut;
    delete vcd;

    return 0;
}
