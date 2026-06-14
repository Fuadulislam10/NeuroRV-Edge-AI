# ==============================================================================
# FILE: timing.tcl
# DESCRIPTION: OpenSTA Static Timing Analysis Signoff Script
# EDA TOOLS: OpenSTA (Integrated inside OpenROAD or standalone)
# ==============================================================================

# 1. Import Multi-Corner Physical Databases and Extracted Parasitics
read_lef $::env(PDK_ROOT)/$::env(PDK)/libs.ref/$::env(STD_CELL_LIBRARY)/techlef/$::env(TECH_LEF)
read_lef $::env(PDK_ROOT)/$::env(PDK)/libs.ref/$::env(STD_CELL_LIBRARY)/lef/$::env(CELL_LEF)

# Load Worst-Case Timing Corner Models for Setup Slacks Analysis
read_liberty -min $::env(PDK_ROOT)/$::env(PDK)/libs.ref/$::env(STD_CELL_LIBRARY)/lib/$::env(LIB_FAST)
read_liberty -max $::env(PDK_ROOT)/$::env(PDK)/libs.ref/$::env(STD_CELL_LIBRARY)/lib/$::env(LIB_SLOW)

read_verilog outputs/neurorv_soc.v
link_design neurorv_soc

# Load Parasitic Capacitance Estimates from Routing Layout
read_sdc constraints.sdc
set_propagated_clock [all_clocks]

# 2. Check Worst-Case Setup (Max-Corner) Violations
puts "\[INFO\] Compiling Setup (Max Corner) Analysis..."
report_checks -path_delay max -fields {slack input launch value} -format full_clock_expanded -digits 4 > reports/timing_setup_signoff.rpt

# 3. Check Best-Case Hold (Min-Corner) Violations
puts "\[INFO\] Compiling Hold (Min Corner) Analysis..."
report_checks -path_delay min -fields {slack input launch value} -format full_clock_expanded -digits 4 > reports/timing_hold_signoff.rpt

# 4. Log General Metrics (Unconstrained Paths, Violations Overview)
report_worst_slack -max > reports/worst_setup_slack.log
report_worst_slack -min > reports/worst_hold_slack.log
report_checks -unconstrained > reports/unconstrained_paths.rpt

puts "\[INFO\] Static Timing Signoff Analysis completed. Reports archived."
exit
