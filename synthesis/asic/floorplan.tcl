# ==============================================================================
# FILE: floorplan.tcl
# DESCRIPTION: OpenROAD Floorplanning and Power Grid Generation Script
# EDA TOOLS: OpenROAD App
# ==============================================================================

# 1. Read Technology LEF and Structural Netlist Files
read_lef $::env(PDK_ROOT)/$::env(PDK)/libs.ref/$::env(STD_CELL_LIBRARY)/techlef/$::env(TECH_LEF)
read_lef $::env(PDK_ROOT)/$::env(PDK)/libs.ref/$::env(STD_CELL_LIBRARY)/lef/$::env(CELL_LEF)
read_verilog outputs/neurorv_soc.v
link_design neurorv_soc

# 2. Define Core Area Parameters and Physical Die Dimension Definition
# Utilization target: 35%. Aspect ratio: 1.0 (Square die)
# Core margin buffer allocation ensures structural density uniformity.
initialize_floorplan \
    -utilization 35 \
    -aspect_ratio 1.0 \
    -core_space_bottom 20.0 \
    -core_space_top 20.0 \
    -core_space_left 20.0 \
    -core_space_right 20.0 \
    -site unithd

# 3. Apply Track Grid Orientations
source $::env(PDK_ROOT)/$::env(PDK)/libs.tech/openroad/config.tcl
make_tracks

# 4. Create Power Distribution Network (PDN) Primitives
# Define structural power domains (VDD: Core Logic Power, VSS: Ground Rail)
create_voltage_domain CORE -power VDD -ground VSS

# Define Metal Layer Straps (met4 Horizontal, met5 Vertical for Sky130)
define_pdn_grid \
    -name global_pdn \
    -voltage_domains CORE

add_pdn_stripe \
    -grid global_pdn \
    -layer met4 \
    -width 1.6 \
    -pitch 16.0 \
    -offset 4.0 \
    -starts_with POWER

add_pdn_stripe \
    -grid global_pdn \
    -layer met5 \
    -width 1.6 \
    -pitch 16.0 \
    -offset 4.0 \
    -starts_with POWER

# 5. Build Global PDN Rails
pdngen

# 6. I/O Pin Placement Policy Configuration
# Randomize or group pins around boundaries to maximize peripheral routing access
place_pins \
    -hor_layers met3 \
    -ver_layers met2 \
    -random \
    -random_seed 42

# 7. Write Floorplanned Database Output State
write_db current_design_floorplan.odb
puts "\[INFO\] Floorplan design phase completed successfully."
exit
