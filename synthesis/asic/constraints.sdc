# ==============================================================================
# FILE: constraints.sdc
# DESCRIPTION: Synopsys Design Constraints (SDC) for NeuroRV Edge SoC
# TARGET SPEED: 100 MHz (Primary Target), 150 MHz (Stress Target Override)
# ==============================================================================

current_design neurorv_soc

# 1. Define Primary Clock Constraints
# Period = 10.00 ns (100 MHz target). For 150 MHz stress target, override to 6.666 ns.
set clk_period 10.000
set clk_port [get_ports clk]

create_clock -name sys_clk -period $clk_period $clk_port
set_clock_uncertainty 0.250 [get_clocks sys_clk]
set_clock_transition 0.150 [get_clocks sys_clk]

# 2. Input/Output Delays & Peripheral Modeling
# Restrict operational windows relative to launch/capture edges (Assume 20% of period for setups)
set input_delay_val [expr $clk_period * 0.20]
set output_delay_val [expr $clk_period * 0.20]

# Extract input and output port groups, filtering out clock boundaries
set all_inputs_ex_clk [remove_from_collection [all_inputs] $clk_port]
set all_outputs [all_outputs]

set_input_delay $input_delay_val -clock sys_clk $all_inputs_ex_clk
set_output_delay $output_delay_val -clock sys_clk $all_outputs

# 3. Electrical Environment Modeling
# Drive and capacitive loads based on standard drive buffers (e.g., sky130_fd_sc_hd__buf_4)
set_driving_cell -lib_cell sky130_fd_sc_hd__buf_4 -pin X $all_inputs_ex_clk
set_load 0.035 $all_outputs

# 4. Asynchronous and False Path Constraints
# Reset and configuration pins do not impact critical paths during operation
set_false_path -from [get_ports rst_n]
set_false_path -from [get_ports config_mode*]

# 5. Multicycle Paths Setup
# Matrix Multiplier Accumulator (MAC) pipelines in the Systolic Array require 2 clock cycles for completion
set_multicycle_path -setup -end 2 -to [get_pins -hierarchical -filter {name =~ *u_systolic_array/*MAC_ACCUM*/D}]
set_multicycle_path -hold -end 1 -to [get_pins -hierarchical -filter {name =~ *u_systolic_array/*MAC_ACCUM*/D}]

# 6. Maximum Fanout Limits for Synthesis/Placement
set_max_fanout 16 [current_design]
