# ==============================================================================
# FILE: place.tcl
# DESCRIPTION: OpenROAD Cell Placement Optimization Flow
# EDA TOOLS: OpenROAD App (RePlAce, Resizer)
# ==============================================================================

# 1. Initialize State by Loading the Pre-Floorplanned Design Database
read_db current_design_floorplan.odb

# Load Operational Timing Standard Cell Models for Placement Optimization
read_liberty $::env(PDK_ROOT)/$::env(PDK)/libs.ref/$::env(STD_CELL_LIBRARY)/lib/$::env(LIB_TYPICAL)
read_sdc constraints.sdc

# 2. Global Cell Placement Analysis and Macro-Driven Distribution Optimization
# Target density must be closely aligned with floorplan core utilization metrics.
puts "\[INFO\] Initiating Global Placement Execution Phase..."
global_placement \
    -density 0.40 \
    -overflow 0.10 \
    -pad_left 2 \
    -pad_right 2

# 3. High-Performance Timing-Driven Optimization Pass
# Slices buffering structures onto critical paths to resolve slack before final legalization.
estimate_parasitics -placement
repair_design
repair_tie_fanout -lib_cell sky130_fd_sc_hd__buf_1

# 4. Legalization of Standard Cells onto Target Track Slices
puts "\[INFO\] Legalizing cell assignments onto technology track slices..."
detailed_placement

# 5. Check Physical Density Violations and Cellular Congestion Check
check_placement -verbose

# 6. Save Complete Placed Database Output State
write_db current_design_placed.odb
puts "\[INFO\] Placement and cell distribution optimizations completed successfully."
exit
