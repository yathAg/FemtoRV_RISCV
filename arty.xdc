# Clock pin
set_property PACKAGE_PIN E3 [get_ports {CLK}]
set_property IOSTANDARD LVCMOS33 [get_ports {CLK}]

# LEDSs
set_property PACKAGE_PIN H5  [get_ports {LEDS[0]}]
set_property PACKAGE_PIN J5  [get_ports {LEDS[1]}]
set_property PACKAGE_PIN T9  [get_ports {LEDS[2]}]
set_property PACKAGE_PIN T10 [get_ports {LEDS[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {LEDS[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {LEDS[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {LEDS[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {LEDS[3]}]

set_property -dict { PACKAGE_PIN D9    IOSTANDARD LVCMOS33 } [get_ports {RESET}]; #IO_L6N_T0_VREF_16 Sch=btn[0]
set_property -dict { PACKAGE_PIN G13   IOSTANDARD LVCMOS33 } [get_ports { TXD }]; #IO_0_15 Sch=ja[1]
set_property -dict { PACKAGE_PIN B11   IOSTANDARD LVCMOS33 } [get_ports { RXD }]; #IO_L4P_T0_15 Sch=ja[2]

# Clock constraints
create_clock -period 10.0 [get_ports {CLK}]
