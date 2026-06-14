# ==============================================================================
# FILE: cts.tcl
# DESCRIPTION: Clock Tree Synthesis (CTS) Optimization Script
# EDA TOOLS: OpenROAD App (TritonCTS)
# ==============================================================================

# 1. Initialize Placed Design Database State
read_db current_design_placed.odb
read_liberty $::env(PDK_ROOT)/$::env(PDK)/libs.ref/$::env(STD_CELL_LIBRARY)/lib/$::env(LIB_TYPICAL)
read_sdc constraints.sdc

# 2. Configure TritonCTS Drivers and Target Inverter Root Injections
# Specify available buffers and inverters to build balanced clock distribution trees.
set_cts_sink_clustering_size 25
set cts_buffer_cells "sky130_fd_sc_hd__clkbuf_1 sky130_fd_sc_hd__clkbuf_2 sky130_fd_sc_hd__clkbuf_4"

# 3. Synthesize the Synchronous Distribution Clock Tree Matrix
puts "\[INFO\] Commencing Clock Tree Synthesis Execution..."
clock_tree_synthesis \
    -buf_list $cts_buffer_cells \
    -root_buf sky130_fd_sc_hd__clkbuf_4 \
    -clk_nets sys_clk

# 4. Perform Legalization Post CTS Buffer Insertion
puts "\[INFO\] Re-legalizing layout boundaries after CTS buffer modifications..."
detailed_placement

# 5. Extract Skew, Structural Jitter, and Insertion Delay Reports
# Re-read timing assertions to account for newly synthesized clock nets.
read_sdc constraints.sdc
set_propagated_clock [all_clocks]

estimate_parasitics -placement
report_clock_skew -digits 4 > reports/cts_skew.rpt
report_clock_tree_latency > reports/cts_latency.rpt

# 6. Save CTS-Optimized Database Output State
write_db current_design_cts.odb
puts "\[INFO\] Clock Tree Synthesis completed. Timing profiles reported."
exit
