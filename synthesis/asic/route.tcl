# ==============================================================================
# FILE: route.tcl
# DESCRIPTION: OpenROAD Global & Detailed Interconnect Routing Script
# EDA TOOLS: OpenROAD App (FastRoute, TritonRoute)
# ==============================================================================

# 1. Load Pre-Routed CTS Database File and Extraction Libraries
read_db current_design_cts.odb
read_liberty $::env(PDK_ROOT)/$::env(PDK)/libs.ref/$::env(STD_CELL_LIBRARY)/lib/$::env(LIB_TYPICAL)
read_sdc constraints.sdc

# 2. Global Routing Phase (Coarse Interconnect Path Allocation)
puts "\[INFO\] Launching Global Routing Analysis Phase..."
global_route \
    -guide_file outputs/route.guide \
    -layers met1-met5 \
    -congestion_iterations 50

# 3. Detailed Routing Implementation (Track Assignments and Via Extractions)
puts "\[INFO\] Launching Detailed Routing Engine. Processing via layer matrix..."
detailed_route \
    -output_drc reports/route_drc.rpt \
    -guide outputs/route.guide \
    -verbose 1

# 4. Check Interconnect Infrastructure Integrity and Extract Structural Violations
puts "\[INFO\] Verifying physical routing integrity..."
check_routes

# 5. Write Fully Routed Database Output State
write_db current_design_routed.odb
puts "\[INFO\] Interconnect routing phases executed and validated."
exit
