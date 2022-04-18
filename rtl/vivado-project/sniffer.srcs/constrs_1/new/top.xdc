########################################################################################################################
# Pinout and IOSTANDARD constraints

set_property PACKAGE_PIN AB13 [get_ports {la0_p[7]}]
set_property PACKAGE_PIN AA11 [get_ports {la0_p[6]}]
set_property PACKAGE_PIN AA10 [get_ports {la0_p[5]}]
set_property IOSTANDARD LVDS [get_ports {la0_p[6]}]
set_property IOSTANDARD LVDS [get_ports {la0_p[5]}]
set_property IOSTANDARD LVDS [get_ports {la0_p[4]}]
set_property IOSTANDARD LVDS [get_ports {la0_p[3]}]
set_property IOSTANDARD LVDS [get_ports {la0_p[2]}]
set_property IOSTANDARD LVDS [get_ports {la0_p[1]}]
set_property IOSTANDARD LVDS [get_ports {la0_p[0]}]
set_property IOSTANDARD LVDS [get_ports {la0_p[7]}]
set_property PACKAGE_PIN AA9 [get_ports {la0_p[4]}]
set_property PACKAGE_PIN AB8 [get_ports {la0_p[3]}]
set_property PACKAGE_PIN W6 [get_ports {la0_p[2]}]
set_property PACKAGE_PIN AA6 [get_ports {la0_p[1]}]
set_property PACKAGE_PIN AA5 [get_ports {la0_p[0]}]
set_property PACKAGE_PIN V4 [get_ports {la1_p[7]}]
set_property PACKAGE_PIN T5 [get_ports {la1_p[6]}]
set_property PACKAGE_PIN T4 [get_ports {la1_p[5]}]
set_property PACKAGE_PIN P4 [get_ports {la1_p[4]}]
set_property PACKAGE_PIN W5 [get_ports {la1_p[3]}]
set_property PACKAGE_PIN AA4 [get_ports {la1_p[2]}]
set_property PACKAGE_PIN AA3 [get_ports {la1_p[1]}]
set_property PACKAGE_PIN AA1 [get_ports {la1_p[0]}]

set_property PACKAGE_PIN P2 [get_ports {lvds_gpio_p[15]}]
set_property PACKAGE_PIN N5 [get_ports {lvds_gpio_p[14]}]
set_property PACKAGE_PIN N3 [get_ports {lvds_gpio_p[13]}]
set_property PACKAGE_PIN M5 [get_ports {lvds_gpio_p[12]}]
set_property PACKAGE_PIN K3 [get_ports {lvds_gpio_p[11]}]
set_property PACKAGE_PIN L5 [get_ports {lvds_gpio_p[10]}]
set_property PACKAGE_PIN Y3 [get_ports {lvds_gpio_p[9]}]
set_property PACKAGE_PIN W1 [get_ports {lvds_gpio_p[8]}]
set_property PACKAGE_PIN V3 [get_ports {lvds_gpio_p[7]}]
set_property PACKAGE_PIN U2 [get_ports {lvds_gpio_p[6]}]
set_property PACKAGE_PIN T1 [get_ports {lvds_gpio_p[5]}]
set_property PACKAGE_PIN R3 [get_ports {lvds_gpio_p[4]}]
set_property PACKAGE_PIN P1 [get_ports {lvds_gpio_p[3]}]
set_property PACKAGE_PIN M2 [get_ports {lvds_gpio_p[2]}]
set_property PACKAGE_PIN L3 [get_ports {lvds_gpio_p[1]}]
set_property PACKAGE_PIN K1 [get_ports {lvds_gpio_p[0]}]
set_property PACKAGE_PIN U10 [get_ports clk_125mhz_p]

set_property IOSTANDARD LVCMOS18 [get_ports {eth_led_n_1v8[1]}]
set_property IOSTANDARD LVCMOS18 [get_ports {eth_led_n_1v8[0]}]
set_property PACKAGE_PIN U13 [get_ports {eth_led_n_1v8[1]}]
set_property PACKAGE_PIN U12 [get_ports {eth_led_n_1v8[0]}]
set_property IOSTANDARD LVCMOS18 [get_ports {rgmii_rxd[3]}]
set_property IOSTANDARD LVCMOS18 [get_ports {rgmii_rxd[2]}]
set_property IOSTANDARD LVCMOS18 [get_ports {rgmii_rxd[1]}]
set_property IOSTANDARD LVCMOS18 [get_ports {rgmii_rxd[0]}]
set_property IOSTANDARD LVCMOS18 [get_ports {rgmii_txd[3]}]
set_property IOSTANDARD LVCMOS18 [get_ports {rgmii_txd[2]}]
set_property IOSTANDARD LVCMOS18 [get_ports {rgmii_txd[1]}]
set_property IOSTANDARD LVCMOS18 [get_ports {rgmii_txd[0]}]
set_property PACKAGE_PIN U11 [get_ports {rgmii_rxd[3]}]
set_property PACKAGE_PIN Y11 [get_ports {rgmii_rxd[2]}]
set_property PACKAGE_PIN W10 [get_ports {rgmii_rxd[1]}]
set_property PACKAGE_PIN W11 [get_ports {rgmii_rxd[0]}]
set_property PACKAGE_PIN Y12 [get_ports {rgmii_txd[3]}]
set_property PACKAGE_PIN V13 [get_ports {rgmii_txd[2]}]
set_property PACKAGE_PIN Y13 [get_ports {rgmii_txd[1]}]
set_property PACKAGE_PIN AA13 [get_ports {rgmii_txd[0]}]
set_property IOSTANDARD LVCMOS18 [get_ports eth_mdio]
set_property IOSTANDARD LVCMOS18 [get_ports eth_mdc]
set_property IOSTANDARD LVCMOS18 [get_ports eth_rst_n]
set_property IOSTANDARD LVCMOS18 [get_ports rgmii_rx_dv]
set_property IOSTANDARD LVCMOS18 [get_ports rgmii_rxc]
set_property IOSTANDARD LVCMOS18 [get_ports rgmii_tx_en]
set_property IOSTANDARD LVCMOS18 [get_ports rgmii_txc]
set_property PACKAGE_PIN V8 [get_ports eth_mdc]
set_property PACKAGE_PIN Y8 [get_ports eth_mdio]
set_property PACKAGE_PIN V7 [get_ports eth_rst_n]
set_property PACKAGE_PIN Y9 [get_ports rgmii_rx_dv]
set_property PACKAGE_PIN W9 [get_ports rgmii_rxc]
set_property PACKAGE_PIN V12 [get_ports rgmii_tx_en]
set_property PACKAGE_PIN W12 [get_ports rgmii_txc]

set_property IOSTANDARD LVCMOS33 [get_ports ram_sda]
set_property PACKAGE_PIN F14 [get_ports ram_sda]
set_property PACKAGE_PIN G12 [get_ports {pmod_dq[7]}]
set_property PACKAGE_PIN B11 [get_ports {pmod_dq[6]}]
set_property PACKAGE_PIN D10 [get_ports {pmod_dq[5]}]
set_property PACKAGE_PIN G10 [get_ports {pmod_dq[4]}]
set_property PACKAGE_PIN A11 [get_ports {pmod_dq[3]}]
set_property PACKAGE_PIN G11 [get_ports {pmod_dq[2]}]
set_property PACKAGE_PIN F10 [get_ports {pmod_dq[1]}]
set_property PACKAGE_PIN H10 [get_ports {pmod_dq[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {pmod_dq[7]}]
set_property IOSTANDARD LVCMOS33 [get_ports {pmod_dq[6]}]
set_property IOSTANDARD LVCMOS33 [get_ports {pmod_dq[5]}]
set_property IOSTANDARD LVCMOS33 [get_ports {pmod_dq[4]}]
set_property IOSTANDARD LVCMOS33 [get_ports {pmod_dq[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {pmod_dq[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {pmod_dq[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {pmod_dq[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {gpio_led[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {gpio_led[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {gpio_led[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {gpio_led[0]}]
set_property PACKAGE_PIN A10 [get_ports {gpio_led[3]}]
set_property PACKAGE_PIN B10 [get_ports {gpio_led[2]}]
set_property PACKAGE_PIN A9 [get_ports {gpio_led[1]}]
set_property PACKAGE_PIN A8 [get_ports {gpio_led[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {sfp_rs[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {sfp_rs[0]}]
set_property PACKAGE_PIN D11 [get_ports {sfp_rs[1]}]
set_property PACKAGE_PIN F11 [get_ports {sfp_rs[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports la0_12v_en]
set_property IOSTANDARD LVCMOS33 [get_ports la0_12v_fault_n]
set_property IOSTANDARD LVCMOS33 [get_ports la0_present_n]
set_property IOSTANDARD LVCMOS33 [get_ports la0_uart_rx]
set_property IOSTANDARD LVCMOS33 [get_ports la1_12v_en]
set_property IOSTANDARD LVCMOS33 [get_ports la0_uart_tx]
set_property IOSTANDARD LVCMOS33 [get_ports la1_12v_fault_n]
set_property IOSTANDARD LVCMOS33 [get_ports la1_present_n]
set_property IOSTANDARD LVCMOS33 [get_ports la1_uart_rx]
set_property IOSTANDARD LVCMOS33 [get_ports la1_uart_tx]
set_property IOSTANDARD LVCMOS33 [get_ports ram_scl]
set_property PACKAGE_PIN F8 [get_ports la0_12v_en]
set_property PACKAGE_PIN G8 [get_ports la0_12v_fault_n]
set_property PACKAGE_PIN F9 [get_ports la0_present_n]
set_property PACKAGE_PIN E9 [get_ports la0_uart_rx]
set_property PACKAGE_PIN E8 [get_ports la0_uart_tx]
set_property PACKAGE_PIN C9 [get_ports la1_12v_en]
set_property PACKAGE_PIN D9 [get_ports la1_12v_fault_n]
set_property PACKAGE_PIN C8 [get_ports la1_present_n]
set_property PACKAGE_PIN B8 [get_ports la1_uart_rx]
set_property PACKAGE_PIN C10 [get_ports la1_uart_tx]
set_property PACKAGE_PIN H14 [get_ports ram_scl]
set_property IOSTANDARD LVCMOS33 [get_ports sfp_mod_abs]
set_property IOSTANDARD LVCMOS33 [get_ports sfp_rx_los]
set_property IOSTANDARD LVCMOS33 [get_ports sfp_scl]
set_property IOSTANDARD LVCMOS33 [get_ports sfp_sda]
set_property IOSTANDARD LVCMOS33 [get_ports sfp_tx_disable]
set_property IOSTANDARD LVCMOS33 [get_ports sfp_tx_fault]
set_property PACKAGE_PIN E12 [get_ports sfp_mod_abs]
set_property PACKAGE_PIN E11 [get_ports sfp_rx_los]
set_property PACKAGE_PIN H13 [get_ports sfp_scl]
set_property PACKAGE_PIN G13 [get_ports sfp_sda]
set_property PACKAGE_PIN E13 [get_ports sfp_tx_disable]
set_property PACKAGE_PIN F13 [get_ports sfp_tx_fault]

set_property PACKAGE_PIN L16 [get_ports flash_cs_n]
set_property PACKAGE_PIN H18 [get_ports flash_si]
set_property PACKAGE_PIN H19 [get_ports flash_so]
set_property IOSTANDARD SSTL135 [get_ports flash_cs_n]
set_property IOSTANDARD SSTL135 [get_ports flash_si]
set_property IOSTANDARD SSTL135 [get_ports flash_so]

set_property PACKAGE_PIN L19 [get_ports clk_200mhz_p]
set_property IOSTANDARD LVDS_25 [get_ports clk_200mhz_p]
set_property IOSTANDARD LVDS_25 [get_ports clk_200mhz_n]

set_property PACKAGE_PIN H8 [get_ports {eth_led_p_3v3[1]}]
set_property PACKAGE_PIN H9 [get_ports {eth_led_p_3v3[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {eth_led_p_3v3[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {eth_led_p_3v3[0]}]

set_property PACKAGE_PIN D6 [get_ports gtx_refclk_156_p]
set_property PACKAGE_PIN F6 [get_ports gtx_refclk_200_p]
set_property PACKAGE_PIN A4 [get_ports sma_tx_p]
set_property PACKAGE_PIN C4 [get_ports sfp_rx_p]

########################################################################################################################
# Pullup/down and slew constraints

set_property PULLUP true [get_ports la1_12v_fault_n]
set_property PULLUP true [get_ports la0_12v_fault_n]

set_property SLEW FAST [get_ports flash_cs_n]
set_property SLEW FAST [get_ports flash_si]

########################################################################################################################
# Input clocks

create_clock -period 8.000 -name clk_125mhz_p -waveform {0.000 4.000} [get_ports clk_125mhz_p]
create_clock -period 5.000 -name clk_200mhz_p -waveform {0.000 2.500} [get_ports clk_200mhz_p]
create_clock -period 8.000 -name rgmii_rxc -waveform {0.000 4.000} [get_ports rgmii_rxc]

create_clock -period 6.400 -name gtx_refclk_156_p -waveform {0.000 3.200} [get_ports gtx_refclk_156_p]
create_clock -period 5.000 -name gtx_refclk_200_p -waveform {0.000 2.500} [get_ports gtx_refclk_200_p]

########################################################################################################################
# Rename PLL outputs for convenience

create_generated_clock -name clk_125mhz -source [get_pins clockgen/pll_125/mmcm/CLKIN1] -master_clock [get_clocks clk_125mhz_p] [get_pins clockgen/pll_125/mmcm/CLKOUT0]
create_generated_clock -name clk_250mhz -source [get_pins clockgen/pll_125/mmcm/CLKIN1] -master_clock [get_clocks clk_125mhz_p] [get_pins clockgen/pll_125/mmcm/CLKOUT1]

create_generated_clock -name la0_clk_625mhz_io_0 -source [get_pins la0_clocks/pll/mmcm/CLKIN1] -master_clock [get_clocks clk_125mhz] [get_pins la0_clocks/pll/mmcm/CLKOUT0]
create_generated_clock -name la0_clk_625mhz_io_90 -source [get_pins la0_clocks/pll/mmcm/CLKIN1] -master_clock [get_clocks clk_125mhz] [get_pins la0_clocks/pll/mmcm/CLKOUT1]
create_generated_clock -name la0_clk_625mhz_fabric -source [get_pins la0_clocks/pll/mmcm/CLKIN1] -master_clock [get_clocks clk_125mhz] [get_pins la0_clocks/pll/mmcm/CLKOUT2]
create_generated_clock -name la0_clk_312p5mhz -source [get_pins la0_clocks/pll/mmcm/CLKIN1] -master_clock [get_clocks clk_125mhz] [get_pins la0_clocks/pll/mmcm/CLKOUT3]

create_generated_clock -name clk_ipstack -source [get_pins clockgen/pll_200/CLKIN1] -master_clock [get_clocks clk_200mhz_p] [get_pins clockgen/pll_200/CLKOUT1]

########################################################################################################################
# Logic analyzer timing and floorplanning

# Location for the IOLOGIC blocks in the phase alignment system
set_property LOC ILOGIC_X1Y0 [get_cells la0_clocks/phase_ctl/iserdes]
set_property LOC OLOGIC_X1Y0 [get_cells la0_clocks/phase_ctl/oserdes]

# Location for rest of the logic analyzer
create_pblock pblock_la0
add_cells_to_pblock [get_pblocks pblock_la0] [get_cells -quiet [list \
          ethernet/rgmii/gig_mac_wrapper \
          la0_clocks \
          la0_path/GND \
          la0_path/VCC \
          la0_path/VCC_1 \
          la0_path/cal \
          {la0_path/genblk1[0].ibuf} \
          {la0_path/genblk1[1].ibuf} \
          {la0_path/genblk1[2].ibuf} \
          {la0_path/genblk1[3].ibuf} \
          {la0_path/genblk1[4].ibuf} \
          {la0_path/genblk1[5].ibuf} \
          {la0_path/genblk1[6].ibuf} \
          {la0_path/genblk1[7].ibuf} \
          {la0_path/genblk2[0].iserdes_n} \
          {la0_path/genblk2[0].iserdes_p} \
          {la0_path/genblk2[1].iserdes_n} \
          {la0_path/genblk2[1].iserdes_p} \
          {la0_path/genblk2[2].iserdes_n} \
          {la0_path/genblk2[2].iserdes_p} \
          {la0_path/genblk2[3].iserdes_n} \
          {la0_path/genblk2[3].iserdes_p} \
          {la0_path/genblk2[4].iserdes_n} \
          {la0_path/genblk2[4].iserdes_p} \
          {la0_path/genblk2[5].iserdes_n} \
          {la0_path/genblk2[5].iserdes_p} \
          {la0_path/genblk2[6].iserdes_n} \
          {la0_path/genblk2[6].iserdes_p} \
          {la0_path/genblk2[7].iserdes_n} \
          {la0_path/genblk2[7].iserdes_p} \
          {la0_path/genblk3[0].sampler} \
          {la0_path/genblk3[1].sampler} \
          {la0_path/genblk3[2].sampler} \
          {la0_path/genblk3[3].sampler} \
          {la0_path/genblk3[4].sampler} \
          {la0_path/genblk3[5].sampler} \
          {la0_path/genblk3[6].sampler} \
          {la0_path/genblk3[7].sampler} \
          {la0_path/genblk4[0].compressor} \
          {la0_path/genblk4[1].compressor} \
          {la0_path/genblk4[2].compressor} \
          {la0_path/genblk4[3].compressor} \
          {la0_path/genblk4[4].compressor} \
          {la0_path/genblk4[5].compressor} \
          {la0_path/genblk4[6].compressor} \
          {la0_path/genblk4[7].compressor} \
          la0_path/idelay_n \
          la0_path/idelay_p \
          {la0_path/samples_reg[0][bits][0]} \
          {la0_path/samples_reg[0][bits][10]} \
          {la0_path/samples_reg[0][bits][11]} \
          {la0_path/samples_reg[0][bits][12]} \
          {la0_path/samples_reg[0][bits][13]} \
          {la0_path/samples_reg[0][bits][14]} \
          {la0_path/samples_reg[0][bits][15]} \
          {la0_path/samples_reg[0][bits][1]} \
          {la0_path/samples_reg[0][bits][2]} \
          {la0_path/samples_reg[0][bits][3]} \
          {la0_path/samples_reg[0][bits][4]} \
          {la0_path/samples_reg[0][bits][5]} \
          {la0_path/samples_reg[0][bits][6]} \
          {la0_path/samples_reg[0][bits][7]} \
          {la0_path/samples_reg[0][bits][8]} \
          {la0_path/samples_reg[0][bits][9]} \
          {la0_path/samples_reg[1][bits][0]} \
          {la0_path/samples_reg[1][bits][10]} \
          {la0_path/samples_reg[1][bits][11]} \
          {la0_path/samples_reg[1][bits][12]} \
          {la0_path/samples_reg[1][bits][13]} \
          {la0_path/samples_reg[1][bits][14]} \
          {la0_path/samples_reg[1][bits][15]} \
          {la0_path/samples_reg[1][bits][1]} \
          {la0_path/samples_reg[1][bits][2]} \
          {la0_path/samples_reg[1][bits][3]} \
          {la0_path/samples_reg[1][bits][4]} \
          {la0_path/samples_reg[1][bits][5]} \
          {la0_path/samples_reg[1][bits][6]} \
          {la0_path/samples_reg[1][bits][7]} \
          {la0_path/samples_reg[1][bits][8]} \
          {la0_path/samples_reg[1][bits][9]} \
          {la0_path/samples_reg[2][bits][0]} \
          {la0_path/samples_reg[2][bits][10]} \
          {la0_path/samples_reg[2][bits][11]} \
          {la0_path/samples_reg[2][bits][12]} \
          {la0_path/samples_reg[2][bits][13]} \
          {la0_path/samples_reg[2][bits][14]} \
          {la0_path/samples_reg[2][bits][15]} \
          {la0_path/samples_reg[2][bits][1]} \
          {la0_path/samples_reg[2][bits][2]} \
          {la0_path/samples_reg[2][bits][3]} \
          {la0_path/samples_reg[2][bits][4]} \
          {la0_path/samples_reg[2][bits][5]} \
          {la0_path/samples_reg[2][bits][6]} \
          {la0_path/samples_reg[2][bits][7]} \
          {la0_path/samples_reg[2][bits][8]} \
          {la0_path/samples_reg[2][bits][9]} \
          {la0_path/samples_reg[3][bits][0]} \
          {la0_path/samples_reg[3][bits][10]} \
          {la0_path/samples_reg[3][bits][11]} \
          {la0_path/samples_reg[3][bits][12]} \
          {la0_path/samples_reg[3][bits][13]} \
          {la0_path/samples_reg[3][bits][14]} \
          {la0_path/samples_reg[3][bits][15]} \
          {la0_path/samples_reg[3][bits][1]} \
          {la0_path/samples_reg[3][bits][2]} \
          {la0_path/samples_reg[3][bits][3]} \
          {la0_path/samples_reg[3][bits][4]} \
          {la0_path/samples_reg[3][bits][5]} \
          {la0_path/samples_reg[3][bits][6]} \
          {la0_path/samples_reg[3][bits][7]} \
          {la0_path/samples_reg[3][bits][8]} \
          {la0_path/samples_reg[3][bits][9]} \
          {la0_path/samples_reg[4][bits][0]} \
          {la0_path/samples_reg[4][bits][10]} \
          {la0_path/samples_reg[4][bits][11]} \
          {la0_path/samples_reg[4][bits][12]} \
          {la0_path/samples_reg[4][bits][13]} \
          {la0_path/samples_reg[4][bits][14]} \
          {la0_path/samples_reg[4][bits][15]} \
          {la0_path/samples_reg[4][bits][1]} \
          {la0_path/samples_reg[4][bits][2]} \
          {la0_path/samples_reg[4][bits][3]} \
          {la0_path/samples_reg[4][bits][4]} \
          {la0_path/samples_reg[4][bits][5]} \
          {la0_path/samples_reg[4][bits][6]} \
          {la0_path/samples_reg[4][bits][7]} \
          {la0_path/samples_reg[4][bits][8]} \
          {la0_path/samples_reg[4][bits][9]} \
          {la0_path/samples_reg[5][bits][0]} \
          {la0_path/samples_reg[5][bits][10]} \
          {la0_path/samples_reg[5][bits][11]} \
          {la0_path/samples_reg[5][bits][12]} \
          {la0_path/samples_reg[5][bits][13]} \
          {la0_path/samples_reg[5][bits][14]} \
          {la0_path/samples_reg[5][bits][15]} \
          {la0_path/samples_reg[5][bits][1]} \
          {la0_path/samples_reg[5][bits][2]} \
          {la0_path/samples_reg[5][bits][3]} \
          {la0_path/samples_reg[5][bits][4]} \
          {la0_path/samples_reg[5][bits][5]} \
          {la0_path/samples_reg[5][bits][6]} \
          {la0_path/samples_reg[5][bits][7]} \
          {la0_path/samples_reg[5][bits][8]} \
          {la0_path/samples_reg[5][bits][9]} \
          {la0_path/samples_reg[6][bits][0]} \
          {la0_path/samples_reg[6][bits][10]} \
          {la0_path/samples_reg[6][bits][11]} \
          {la0_path/samples_reg[6][bits][12]} \
          {la0_path/samples_reg[6][bits][13]} \
          {la0_path/samples_reg[6][bits][14]} \
          {la0_path/samples_reg[6][bits][15]} \
          {la0_path/samples_reg[6][bits][1]} \
          {la0_path/samples_reg[6][bits][2]} \
          {la0_path/samples_reg[6][bits][3]} \
          {la0_path/samples_reg[6][bits][4]} \
          {la0_path/samples_reg[6][bits][5]} \
          {la0_path/samples_reg[6][bits][6]} \
          {la0_path/samples_reg[6][bits][7]} \
          {la0_path/samples_reg[6][bits][8]} \
          {la0_path/samples_reg[6][bits][9]} \
          {la0_path/samples_reg[7][bits][0]} \
          {la0_path/samples_reg[7][bits][10]} \
          {la0_path/samples_reg[7][bits][11]} \
          {la0_path/samples_reg[7][bits][12]} \
          {la0_path/samples_reg[7][bits][13]} \
          {la0_path/samples_reg[7][bits][14]} \
          {la0_path/samples_reg[7][bits][15]} \
          {la0_path/samples_reg[7][bits][1]} \
          {la0_path/samples_reg[7][bits][2]} \
          {la0_path/samples_reg[7][bits][3]} \
          {la0_path/samples_reg[7][bits][4]} \
          {la0_path/samples_reg[7][bits][5]} \
          {la0_path/samples_reg[7][bits][6]} \
          {la0_path/samples_reg[7][bits][7]} \
          {la0_path/samples_reg[7][bits][8]} \
          {la0_path/samples_reg[7][bits][9]}]]
resize_pblock [get_pblocks pblock_la0] -add {CLOCKREGION_X1Y0:CLOCKREGION_X1Y0}
set_property IS_SOFT TRUE [get_pblocks pblock_la0]

create_pblock pblock_la1
resize_pblock [get_pblocks pblock_la1] -add {CLOCKREGION_X1Y1:CLOCKREGION_X1Y1}
set_property IS_SOFT TRUE [get_pblocks pblock_la1]

# IOSERDES loopback is a false path
set_false_path -from [get_pins la0_clocks/phase_ctl/oserdes/CLK] -to [get_pins la0_clocks/phase_ctl/iserdes/OFB]

# Tight timing path from BUFIO to BUFG clock
#set_max_delay -from [get_pins -hierarchical -filter { NAME =~  "*iserdes*" && NAME =~  "*Q*" && NAME =~  "*la*_path*" }] -to [get_cells -hierarchical *deser_*_ff*] 0.600
set_max_delay -from [get_pins -hierarchical -filter { NAME =~  "*la*_path*" && NAME =~  "*iserdes*" && NAME =~  "*Q*" }] -to [get_cells -hierarchical -filter { NAME =~  "*la*_path*" && NAME =~  "*fast_data_ff*" }] 0.600

# Path through LUTRAM FIFO can take a little while as it's multicycle

########################################################################################################################
# Other timing constraints

# CDC from RGMII to internal 125 MHz clock domain
set_clock_groups -asynchronous -group [get_clocks rgmii_rxc] -group [get_clocks clk_125mhz]
set_clock_groups -asynchronous -group [get_clocks clk_125mhz] -group [get_clocks rgmii_rxc]

# Don't need dedicated routing for 125 MHz refclk input
# as phase from the oscillator to anything else doesn't matter

########################################################################################################################
# Miscellaneous

create_pblock pblock_ddr
add_cells_to_pblock [get_pblocks pblock_ddr] [get_cells -quiet [list ram]]
resize_pblock [get_pblocks pblock_ddr] -add {SLICE_X0Y0:SLICE_X23Y149}
resize_pblock [get_pblocks pblock_ddr] -add {DSP48_X0Y0:DSP48_X1Y59}
resize_pblock [get_pblocks pblock_ddr] -add {RAMB18_X0Y0:RAMB18_X1Y59}
resize_pblock [get_pblocks pblock_ddr] -add {RAMB36_X0Y0:RAMB36_X1Y29}
set_property IS_SOFT TRUE [get_pblocks pblock_ddr]
create_pblock pblock_sfp
add_cells_to_pblock [get_pblocks pblock_sfp] [get_cells -quiet [list ethernet/baser_rx_cdc ethernet/sfp]]
resize_pblock [get_pblocks pblock_sfp] -add {CLOCKREGION_X1Y2:CLOCKREGION_X1Y2}
set_property IS_SOFT TRUE [get_pblocks pblock_sfp]

create_pblock pblock_sata
resize_pblock [get_pblocks pblock_sata] -add {CLOCKREGION_X0Y3:CLOCKREGION_X1Y3}
set_property IS_SOFT TRUE [get_pblocks pblock_sata]

set_clock_groups -asynchronous -group [get_clocks ethernet/sfp/xg_transceiver/inst/gtwizard_10gbe_i/gt0_gtwizard_10gbe_i/gtxe2_i/RXOUTCLK] -group [get_clocks clk_ipstack]
set_clock_groups -asynchronous -group [get_clocks rgmii_rxc] -group [get_clocks clk_ipstack]
set_clock_groups -asynchronous -group [get_clocks clk_ipstack] -group [get_clocks ethernet/sfp/xg_transceiver/inst/gtwizard_10gbe_i/gt0_gtwizard_10gbe_i/gtxe2_i/RXOUTCLK]
set_clock_groups -asynchronous -group [get_clocks clk_ipstack] -group [get_clocks rgmii_rxc]

set_clock_groups -asynchronous -group [get_clocks clk_ipstack] -group [get_clocks clk_125mhz]
set_clock_groups -asynchronous -group [get_clocks clk_125mhz] -group [get_clocks clk_ipstack]
set_clock_groups -asynchronous -group [get_clocks ethernet/sfp/xg_transceiver/inst/gtwizard_10gbe_i/gt0_gtwizard_10gbe_i/gtxe2_i/TXOUTCLK] -group [get_clocks clk_ipstack]
set_clock_groups -asynchronous -group [get_clocks clk_ipstack] -group [get_clocks ethernet/sfp/xg_transceiver/inst/gtwizard_10gbe_i/gt0_gtwizard_10gbe_i/gtxe2_i/TXOUTCLK]


set_max_delay -from [get_clocks *625mhz_fabric*] -through [get_cells -hierarchical *fifomem*] -to [get_clocks *312p5mhz*] 3.200
set_property C_CLK_INPUT_FREQ_HZ 300000000 [get_debug_cores dbg_hub]
set_property C_ENABLE_CLK_DIVIDER false [get_debug_cores dbg_hub]
set_property C_USER_SCAN_CHAIN 1 [get_debug_cores dbg_hub]
connect_debug_port dbg_hub/clk [get_nets clk_125mhz]
