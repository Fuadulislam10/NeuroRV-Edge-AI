# ==============================================================================
# FILE: synth.tcl
# DESCRIPTION: Yosys RTL Synthesis Script for NeuroRV Edge SoC
# TARGET EDA: Yosys Open SYnthesis Suite
# ==============================================================================

yosys -log synth.log

# 1. Environment Variable Extraction & Parameter Checks
if {[info exists ::env(PDK_ROOT)] == 0} { set ::env(PDK_ROOT) "/usr/local/share/pdk" }
if {[info exists ::env(PDK)] == 0} { set ::env(PDK) "sky130A" }
if {[info exists ::env(STD_CELL_LIBRARY)] == 0} { set ::env(STD_CELL_LIBRARY) "sky130_fd_sc_hd" }

set pdk_dir "$::env(PDK_ROOT)/$::env(PDK)"
set target_lib "$pdk_dir/libs.ref/$::env(STD_CELL_LIBRARY)/lib/$::env(STD_CELL_LIBRARY)__tt_025C_1v80.lib"

# 2. Define Include Directories and Read SystemVerilog RTL Sources
# Note: In production, include directories resolve sub-modules.
set rtl_top "neurorv_soc"

puts "\[INFO\] Reading NeuroRV Edge RTL Sources..."
read_verilog -sv -I../../rtl/include ../../rtl/core/neurorv_core.sv
read_verilog -sv -I../../rtl/include ../../rtl/core/neurorv_rf.sv
read_verilog -sv -I../../rtl/include ../../rtl/core/neurorv_alu.sv
read_verilog -sv -I../../rtl/include ../../rtl/core/neurorv_control.sv
read_verilog -sv -I../../rtl/include ../../rtl/memory/neurorv_sram_interface.sv
read_verilog -sv -I../../rtl/include ../../rtl/accelerator/neurorv_systolic_array.sv
read_verilog -sv -I../../rtl/include ../../rtl/accelerator/neurorv_weight_buffer.sv
read_verilog -sv -I../../rtl/include ../../rtl/soc/neurorv_soc.sv

# 3. Check Design Hierarchy and Elaborate Top Module
puts "\[INFO\] Elaborating Design Top Module: $rtl_top"
hierarchy -check -top $rtl_top

# 4. High-Level Optimizations, Coarse Synthesis, and Design Flattening
puts "\[INFO\] Executing High-Level Optimization Flow..."
procs; opt; fsm; opt; memory; opt

# Flatten the hierarchy to unlock optimal cross-boundary logic optimizations across the AI core
flatten
opt

# 5. Technology Mapping to Target PDK Cell Library
puts "\[INFO\] Mapping Technology Gates via ABC using: $target_lib"
techmap

# Invoke ABC engine with specific timing-driven delay/area structural trade-offs
abc -liberty $target_lib -constr constraints.sdc

# Clean up redundant dangling wires and unmapped gates post-ABC mapping
opt_clean -purge

# 6. Explicit Technology Mapping for Registers / Sequential Latches
puts "\[INFO\] Mapping Flip-Flops to Standard Cell Library primitives..."
dfflegalize -cell $_DFF_P_ 0
dffinit

# Final optimization pass post technology mapping
opt

# 7. Write Structural Gate-Level Netlist Outputs
puts "\[INFO\] Generating Structural Gate-Level Netlist output..."
write_verilog -noattr -noexpr -nohex -nodedec outputs/neurorv_soc.v

# 8. Extract Area, Quality-of-Results (QoR), and Gate-Count Metrics
puts "\[INFO\] Compiling Synthesis Quality of Results (QoR) and Cell Allocation Reports..."
report_area -hierarchy > reports/synth_area.rpt
report_cell_usage > reports/synth_cells.rpt

puts "\[INFO\] RTL Synthesis completed successfully for design top: $rtl_top."
exit
