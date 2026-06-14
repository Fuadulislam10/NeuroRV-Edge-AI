# ==============================================================================
# FILE: power_analysis.tcl
# DESCRIPTION: OpenROAD/OpenSTA Vectorless & Vector-Driven Power Signoff
# EDA TOOLS: OpenROAD App / OpenSTA
# ==============================================================================

# 1. Read Physical Netlists and Timing Signoff Libraries
read_lef $::env(PDK_ROOT)/$::env(PDK)/libs.ref/$::env(STD_CELL_LIBRARY)/techlef/$::env(TECH_LEF)
read_lef $::env(PDK_ROOT)/$::env(PDK)/libs.ref/$::env(STD_CELL_LIBRARY)/lef/$::env(CELL_LEF)
read_liberty $::env(PDK_ROOT)/$::env(PDK)/libs.ref/$::env(STD_CELL_LIBRARY)/lib/$::env(LIB_TYPICAL)

read_verilog outputs/neurorv_soc.v
link_design neurorv_soc
read_sdc constraints.sdc

# 2. Establish Switching Activities Model
# Fallback vectorless activity assumptions if a functional VCD/SAIF trace profile is absent.
set_switching_activity -input_port_activity 0.15 -default_toggle_rate 0.10

# In production execution, parse standard functional simulation Value Change Dump (VCD) traces:
# read_vcd -activities "../../sim/vcd/neuro_workload.vcd"

# 3. Execute Internal Power Signoff Computations
puts "\[INFO\] Calculating Internal, Leakage, Dynamic, and Total Power dissipation metrics..."

# 4. Generate Core-Level Hierarchical Breakdown Summaries
report_power -digits 6 > reports/power_signoff_summary.rpt
report_power -hierarchy -depth 3 > reports/power_hierarchy_breakdown.rpt

puts "\[INFO\] Power dissipation analysis finalized."
exit
