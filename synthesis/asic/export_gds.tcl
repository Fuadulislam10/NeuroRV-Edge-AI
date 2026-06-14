# ==============================================================================
# FILE: export_gds.tcl
# DESCRIPTION: Stream-Out Export Layout Flow to GDSII Format
# EDA TOOLS: OpenROAD App / Magic Layout Editor Stream Engine
# ==============================================================================

# 1. Load Fully Routed Realized Target Database File
read_db current_design_routed.odb

# 2. Configure Tech GDS Mapping Lookups
set tech_gds_map "$::env(PDK_ROOT)/$::env(PDK)/libs.tech/openroad/gds_map.txt"
set standard_cell_gds "$::env(PDK_ROOT)/$::env(PDK)/libs.ref/$::env(STD_CELL_LIBRARY)/gds/$::env(STD_CELL_LIBRARY).gds"

# 3. Write Out Def Layout Intermediary Def File
write_def outputs/neurorv_soc.def

# 4. Invoke Magic Layout System Stream-Out Processing
# Stream out the layout by combining standard cell GDS structures with custom routing layers.
puts "\[INFO\] Initializing Stream-out engine via Magic lookup mappings..."

# Execute standard system command pipeline to construct the final GDSII stream binary
set magic_cmd "magic -dnull -nocodes \
    -tech $::env(PDK_ROOT)/$::env(PDK)/libs.tech/magic/$::env(PDK).tech \
    << EOF
    def read outputs/neurorv_soc.def
    gds read $standard_cell_gds
    gds write outputs/neurorv_soc.gds
    exit
EOF"

if {[catch {exec sh -c $magic_cmd} msg]} {
    puts "\[ERROR\] Magic stream-out process failed: $msg"
    exit 1
}

puts "\[INFO\] Stream-out generation completed. Target Output File: outputs/neurorv_soc.gds"
exit
