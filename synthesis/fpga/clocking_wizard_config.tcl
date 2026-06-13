# =============================================================================
# Project:     NeuroRV Edge
# File:        clocking_wizard_config.tcl
# Description: Tcl configuration commands to generate the IP Clocking Wizard
#              via Vivado non-project mode or project mode scripting.
# =============================================================================

create_ip -name clk_wiz -vendor xilinx.com -library ip -version 6.0 -module_name clk_wiz_0

set_property -dict [list \
  CONFIG.CLKIN1_JITTER_PS {100.0} \
  CONFIG.CLKOUT1_DRIVES {BUFG} \
  CONFIG.CLKOUT1_JITTER {151.790} \
  CONFIG.CLKOUT1_PHASE_ERROR {98.575} \
  CONFIG.CLKOUT1_REQUESTED_OUT_FREQ {50.000} \
  CONFIG.MMCM_CLKFBOUT_MULT_F {10.000} \
  CONFIG.MMCM_CLKIN1_PERIOD {10.000} \
  CONFIG.MMCM_CLKOUT0_DIVIDE_F {20.000} \
  CONFIG.PRIM_IN_FREQ {100.000} \
  CONFIG.RESET_PORT {resetn} \
  CONFIG.RESET_TYPE {ACTIVE_LOW} \
] [get_ips clk_wiz_0]

generate_target {instantiation_template synthesis simulation} [get_ips clk_wiz_0]
