# =============================================================================
# Project:     NeuroRV Edge
# File:        build_fpga.tcl
# Description: Automated Vivado Build Script implementing the compilation flow
#              from HDL sources up to final Bitstream configuration file.
# Usage:       vivado -mode batch -source build_fpga.tcl
# =============================================================================

set outputDir ./build_output
file mkdir $outputDir

# 1. Setup Project & Define Target Device
create_project -force neurorv_edge_synth $outputDir/project -part xc7a100tcsg324-1

# 2. Ingest Hardware System HDL Sources
add_files {
    ../../synthesis/fpga/fpga_top.sv
}

# Ingest underlying SoC module files (assumed location alongside wrapper)
# If your design uses nested system directories, uncomment below or adapt pathways:
# add_files [glob ../../hdl/rtl/*.sv]

# 3. Import Physical Constraint Files
add_files -fileset constrs_1 {
    ../../synthesis/fpga/constraints.xdc
}

# 4. Generate Core Clocking Infrastructure IP
source ../../synthesis/fpga/clocking_wizard_config.tcl

# 5. Link memory initialization files to the execution path
set_property top fpga_top [current_fileset]
update_compile_order -fileset sources_1

# 6. Run Synthesis Stage
synth_design -top fpga_top -part xc7a100tcsg324-1
write_checkpoint -force $outputDir/post_synth.dcp
report_timing_summary -file $outputDir/post_synth_timing_summary.rpt
report_utilization -file $outputDir/post_synth_utilization.rpt

# 7. Run Implementation (Placement & Optimization)
opt_design
place_design
phys_opt_design
write_checkpoint -force $outputDir/post_place.dcp

# 8. Run Implementation (Routing)
route_design
write_checkpoint -force $outputDir/post_route.dcp
report_route_status -file $outputDir/post_route_status.rpt
report_timing_summary -file $outputDir/post_route_timing_summary.rpt
report_power -file $outputDir/post_route_power.rpt

# 9. Verify Timing Constraints Met
set q_status [get_property SLACK [get_timing_paths -max_paths 1 -setup]]
if {$q_status < 0} {
    puts "CRITICAL WARNING: Timing constraints violated! Slack: $q_status ns"
} else {
    puts "SUCCESS: Timing constraints met. Slack: $q_status ns"
}

# 10. Generate Output Production Bitstream
write_bitstream -force $outputDir/neurorv_edge.bit
puts "INFO: FPGA Build pipeline complete. Bitstream location: $outputDir/neurorv_edge.bit"
