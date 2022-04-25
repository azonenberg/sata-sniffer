set_max_delay -from [get_clocks [list la0_clk_625mhz_fabric la1_clk_625mhz_fabric]] -through [get_cells [list {la/la0_path/genblk3[0].cdc_fifo/fifomem} \
          {la/la0_path/genblk3[0].sampler/fifo_n_hi/fifomem} \
          {la/la0_path/genblk3[0].sampler/fifo_n_lo/fifomem} \
          {la/la0_path/genblk3[0].sampler/fifo_p_hi/fifomem} \
          {la/la0_path/genblk3[0].sampler/fifo_p_lo/fifomem} \
          {la/la0_path/genblk3[1].cdc_fifo/fifomem} \
          {la/la0_path/genblk3[1].sampler/fifo_n_hi/fifomem} \
          {la/la0_path/genblk3[1].sampler/fifo_n_lo/fifomem} \
          {la/la0_path/genblk3[1].sampler/fifo_p_hi/fifomem} \
          {la/la0_path/genblk3[1].sampler/fifo_p_lo/fifomem} \
          {la/la0_path/genblk3[2].cdc_fifo/fifomem} \
          {la/la0_path/genblk3[2].sampler/fifo_n_hi/fifomem} \
          {la/la0_path/genblk3[2].sampler/fifo_n_lo/fifomem} \
          {la/la0_path/genblk3[2].sampler/fifo_p_hi/fifomem} \
          {la/la0_path/genblk3[2].sampler/fifo_p_lo/fifomem} \
          {la/la0_path/genblk3[3].cdc_fifo/fifomem} \
          {la/la0_path/genblk3[3].sampler/fifo_n_hi/fifomem} \
          {la/la0_path/genblk3[3].sampler/fifo_n_lo/fifomem} \
          {la/la0_path/genblk3[3].sampler/fifo_p_hi/fifomem} \
          {la/la0_path/genblk3[3].sampler/fifo_p_lo/fifomem} \
          {la/la0_path/genblk3[4].cdc_fifo/fifomem} \
          {la/la0_path/genblk3[4].sampler/fifo_n_hi/fifomem} \
          {la/la0_path/genblk3[4].sampler/fifo_n_lo/fifomem} \
          {la/la0_path/genblk3[4].sampler/fifo_p_hi/fifomem} \
          {la/la0_path/genblk3[4].sampler/fifo_p_lo/fifomem} \
          {la/la0_path/genblk3[5].cdc_fifo/fifomem} \
          {la/la0_path/genblk3[5].sampler/fifo_n_hi/fifomem} \
          {la/la0_path/genblk3[5].sampler/fifo_n_lo/fifomem} \
          {la/la0_path/genblk3[5].sampler/fifo_p_hi/fifomem} \
          {la/la0_path/genblk3[5].sampler/fifo_p_lo/fifomem} \
          {la/la0_path/genblk3[6].cdc_fifo/fifomem} \
          {la/la0_path/genblk3[6].sampler/fifo_n_hi/fifomem} \
          {la/la0_path/genblk3[6].sampler/fifo_n_lo/fifomem} \
          {la/la0_path/genblk3[6].sampler/fifo_p_hi/fifomem} \
          {la/la0_path/genblk3[6].sampler/fifo_p_lo/fifomem} \
          {la/la0_path/genblk3[7].cdc_fifo/fifomem} \
          {la/la0_path/genblk3[7].sampler/fifo_n_hi/fifomem} \
          {la/la0_path/genblk3[7].sampler/fifo_n_lo/fifomem} \
          {la/la0_path/genblk3[7].sampler/fifo_p_hi/fifomem} \
          {la/la0_path/genblk3[7].sampler/fifo_p_lo/fifomem} \
          {la/la1_path/genblk3[0].cdc_fifo/fifomem} \
          {la/la1_path/genblk3[0].sampler/fifo_n_hi/fifomem} \
          {la/la1_path/genblk3[0].sampler/fifo_n_lo/fifomem} \
          {la/la1_path/genblk3[0].sampler/fifo_p_hi/fifomem} \
          {la/la1_path/genblk3[0].sampler/fifo_p_lo/fifomem} \
          {la/la1_path/genblk3[1].cdc_fifo/fifomem} \
          {la/la1_path/genblk3[1].sampler/fifo_n_hi/fifomem} \
          {la/la1_path/genblk3[1].sampler/fifo_n_lo/fifomem} \
          {la/la1_path/genblk3[1].sampler/fifo_p_hi/fifomem} \
          {la/la1_path/genblk3[1].sampler/fifo_p_lo/fifomem} \
          {la/la1_path/genblk3[2].cdc_fifo/fifomem} \
          {la/la1_path/genblk3[2].sampler/fifo_n_hi/fifomem} \
          {la/la1_path/genblk3[2].sampler/fifo_n_lo/fifomem} \
          {la/la1_path/genblk3[2].sampler/fifo_p_hi/fifomem} \
          {la/la1_path/genblk3[2].sampler/fifo_p_lo/fifomem} \
          {la/la1_path/genblk3[3].cdc_fifo/fifomem} \
          {la/la1_path/genblk3[3].sampler/fifo_n_hi/fifomem} \
          {la/la1_path/genblk3[3].sampler/fifo_n_lo/fifomem} \
          {la/la1_path/genblk3[3].sampler/fifo_p_hi/fifomem} \
          {la/la1_path/genblk3[3].sampler/fifo_p_lo/fifomem} \
          {la/la1_path/genblk3[4].cdc_fifo/fifomem} \
          {la/la1_path/genblk3[4].sampler/fifo_n_hi/fifomem} \
          {la/la1_path/genblk3[4].sampler/fifo_n_lo/fifomem} \
          {la/la1_path/genblk3[4].sampler/fifo_p_hi/fifomem} \
          {la/la1_path/genblk3[4].sampler/fifo_p_lo/fifomem} \
          {la/la1_path/genblk3[5].cdc_fifo/fifomem} \
          {la/la1_path/genblk3[5].sampler/fifo_n_hi/fifomem} \
          {la/la1_path/genblk3[5].sampler/fifo_n_lo/fifomem} \
          {la/la1_path/genblk3[5].sampler/fifo_p_hi/fifomem} \
          {la/la1_path/genblk3[5].sampler/fifo_p_lo/fifomem} \
          {la/la1_path/genblk3[6].cdc_fifo/fifomem} \
          {la/la1_path/genblk3[6].sampler/fifo_n_hi/fifomem} \
          {la/la1_path/genblk3[6].sampler/fifo_n_lo/fifomem} \
          {la/la1_path/genblk3[6].sampler/fifo_p_hi/fifomem} \
          {la/la1_path/genblk3[6].sampler/fifo_p_lo/fifomem} \
          {la/la1_path/genblk3[7].cdc_fifo/fifomem} \
          {la/la1_path/genblk3[7].sampler/fifo_n_hi/fifomem} \
          {la/la1_path/genblk3[7].sampler/fifo_n_lo/fifomem} \
          {la/la1_path/genblk3[7].sampler/fifo_p_hi/fifomem} \
          {la/la1_path/genblk3[7].sampler/fifo_p_lo/fifomem} \
          mem/arbiter/wr_data_fifo/fifomem]] -to [get_clocks [list la0_clk_312p5mhz la1_clk_312p5mhz]] 3.200
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

create_generated_clock -name la0_clk_625mhz_io_0 -source [get_pins la/la0_clocks/pll/mmcm/CLKIN1] -master_clock [get_clocks clk_125mhz] [get_pins la/la0_clocks/pll/mmcm/CLKOUT0]
create_generated_clock -name la0_clk_625mhz_io_90 -source [get_pins la/la0_clocks/pll/mmcm/CLKIN1] -master_clock [get_clocks clk_125mhz] [get_pins la/la0_clocks/pll/mmcm/CLKOUT1]
create_generated_clock -name la0_clk_625mhz_fabric -source [get_pins la/la0_clocks/pll/mmcm/CLKIN1] -master_clock [get_clocks clk_125mhz] [get_pins la/la0_clocks/pll/mmcm/CLKOUT2]
create_generated_clock -name la0_clk_312p5mhz -source [get_pins la/la0_clocks/pll/mmcm/CLKIN1] -master_clock [get_clocks clk_125mhz] [get_pins la/la0_clocks/pll/mmcm/CLKOUT3]

create_generated_clock -name la1_clk_625mhz_io_0 -source [get_pins la/la1_clocks/pll/mmcm/CLKIN1] -master_clock [get_clocks clk_125mhz] [get_pins la/la1_clocks/pll/mmcm/CLKOUT0]
create_generated_clock -name la1_clk_625mhz_io_90 -source [get_pins la/la1_clocks/pll/mmcm/CLKIN1] -master_clock [get_clocks clk_125mhz] [get_pins la/la1_clocks/pll/mmcm/CLKOUT1]
create_generated_clock -name la1_clk_625mhz_fabric -source [get_pins la/la1_clocks/pll/mmcm/CLKIN1] -master_clock [get_clocks clk_125mhz] [get_pins la/la1_clocks/pll/mmcm/CLKOUT2]
create_generated_clock -name la1_clk_312p5mhz -source [get_pins la/la1_clocks/pll/mmcm/CLKIN1] -master_clock [get_clocks clk_125mhz] [get_pins la/la1_clocks/pll/mmcm/CLKOUT3]

create_generated_clock -name clk_ipstack -source [get_pins clockgen/pll_200/CLKIN1] -master_clock [get_clocks clk_200mhz_p] [get_pins clockgen/pll_200/CLKOUT1]

create_generated_clock -name clk_ram -source [get_pins mem/ram/u_ddr3_mig/u_ddr3_infrastructure/gen_mmcm.mmcm_i/CLKIN1] -master_clock [get_clocks pll_clk3_out] [get_pins mem/ram/u_ddr3_mig/u_ddr3_infrastructure/gen_mmcm.mmcm_i/CLKFBOUT]

########################################################################################################################
# Logic analyzer timing and floorplanning

# Clock domain crossing from LA clocks to DDR clock domain
set_clock_groups -asynchronous -group [get_clocks la0_clk_312p5mhz] -group [get_clocks clk_ram]
set_clock_groups -asynchronous -group [get_clocks la1_clk_312p5mhz] -group [get_clocks clk_ram]
set_clock_groups -asynchronous -group [get_clocks clk_ram] -group [get_clocks la0_clk_312p5mhz]
set_clock_groups -asynchronous -group [get_clocks clk_ram] -group [get_clocks la1_clk_312p5mhz]

# Location for the IOLOGIC blocks in the phase alignment system
set_property LOC ILOGIC_X1Y0 [get_cells la/la0_clocks/phase_ctl/iserdes]
set_property LOC OLOGIC_X1Y0 [get_cells la/la0_clocks/phase_ctl/oserdes]

set_property LOC ILOGIC_X1Y50 [get_cells la/la1_clocks/phase_ctl/iserdes]
set_property LOC OLOGIC_X1Y50 [get_cells la/la1_clocks/phase_ctl/oserdes]

# Location for the IDELAYCTRL blocks for input phasing
set_property LOC IDELAYCTRL_X1Y0 [get_cells la/la0_path/cal/delay_control_block]
set_property LOC IDELAYCTRL_X1Y1 [get_cells la/la1_path/cal/delay_control_block]

# Location for rest of the logic analyzer
create_pblock pblock_la0
add_cells_to_pblock [get_pblocks pblock_la0] [get_cells -quiet [list \
          ethernet/rgmii/gig_mac_wrapper \
          la/la0_clocks \
          la/la0_path/GND \
          la/la0_path/VCC \
          la/la0_path/VCC_1 \
          la/la0_path/cal \
          {la/la0_path/genblk1[0].ibuf} \
          {la/la0_path/genblk1[1].ibuf} \
          {la/la0_path/genblk1[2].ibuf} \
          {la/la0_path/genblk1[3].ibuf} \
          {la/la0_path/genblk1[4].ibuf} \
          {la/la0_path/genblk1[5].ibuf} \
          {la/la0_path/genblk1[6].ibuf} \
          {la/la0_path/genblk1[7].ibuf} \
          {la/la0_path/genblk2[0].iserdes_n} \
          {la/la0_path/genblk2[0].iserdes_p} \
          {la/la0_path/genblk2[1].iserdes_n} \
          {la/la0_path/genblk2[1].iserdes_p} \
          {la/la0_path/genblk2[2].iserdes_n} \
          {la/la0_path/genblk2[2].iserdes_p} \
          {la/la0_path/genblk2[3].iserdes_n} \
          {la/la0_path/genblk2[3].iserdes_p} \
          {la/la0_path/genblk2[4].iserdes_n} \
          {la/la0_path/genblk2[4].iserdes_p} \
          {la/la0_path/genblk2[5].iserdes_n} \
          {la/la0_path/genblk2[5].iserdes_p} \
          {la/la0_path/genblk2[6].iserdes_n} \
          {la/la0_path/genblk2[6].iserdes_p} \
          {la/la0_path/genblk2[7].iserdes_n} \
          {la/la0_path/genblk2[7].iserdes_p} \
          {la/la0_path/genblk3[0].cdc_fifo} \
          {la/la0_path/genblk3[0].compressor} \
          {la/la0_path/genblk3[0].deserialization} \
          {la/la0_path/genblk3[0].sampler} \
          {la/la0_path/genblk3[1].cdc_fifo} \
          {la/la0_path/genblk3[1].compressor} \
          {la/la0_path/genblk3[1].deserialization} \
          {la/la0_path/genblk3[1].sampler} \
          {la/la0_path/genblk3[2].cdc_fifo} \
          {la/la0_path/genblk3[2].compressor} \
          {la/la0_path/genblk3[2].deserialization} \
          {la/la0_path/genblk3[2].sampler} \
          {la/la0_path/genblk3[3].cdc_fifo} \
          {la/la0_path/genblk3[3].compressor} \
          {la/la0_path/genblk3[3].deserialization} \
          {la/la0_path/genblk3[3].sampler} \
          {la/la0_path/genblk3[4].cdc_fifo} \
          {la/la0_path/genblk3[4].compressor} \
          {la/la0_path/genblk3[4].deserialization} \
          {la/la0_path/genblk3[4].sampler} \
          {la/la0_path/genblk3[5].cdc_fifo} \
          {la/la0_path/genblk3[5].compressor} \
          {la/la0_path/genblk3[5].deserialization} \
          {la/la0_path/genblk3[5].sampler} \
          {la/la0_path/genblk3[6].cdc_fifo} \
          {la/la0_path/genblk3[6].compressor} \
          {la/la0_path/genblk3[6].deserialization} \
          {la/la0_path/genblk3[6].sampler} \
          {la/la0_path/genblk3[7].cdc_fifo} \
          {la/la0_path/genblk3[7].compressor} \
          {la/la0_path/genblk3[7].deserialization} \
          {la/la0_path/genblk3[7].sampler} \
          la/la0_path/idelay_n \
          la/la0_path/idelay_p]]
resize_pblock [get_pblocks pblock_la0] -add {CLOCKREGION_X1Y0:CLOCKREGION_X1Y0}
set_property IS_SOFT TRUE [get_pblocks pblock_la0]

create_pblock pblock_la1
add_cells_to_pblock [get_pblocks pblock_la1] [get_cells -quiet [list \
          la/la1_clocks \
          la/la1_path/cal \
          {la/la1_path/genblk3[0].cdc_fifo} \
          {la/la1_path/genblk3[0].compressor} \
          {la/la1_path/genblk3[0].deserialization} \
          {la/la1_path/genblk3[0].sampler} \
          {la/la1_path/genblk3[1].cdc_fifo} \
          {la/la1_path/genblk3[1].compressor} \
          {la/la1_path/genblk3[1].deserialization} \
          {la/la1_path/genblk3[1].sampler} \
          {la/la1_path/genblk3[2].cdc_fifo} \
          {la/la1_path/genblk3[2].compressor} \
          {la/la1_path/genblk3[2].deserialization} \
          {la/la1_path/genblk3[2].sampler} \
          {la/la1_path/genblk3[3].cdc_fifo} \
          {la/la1_path/genblk3[3].compressor} \
          {la/la1_path/genblk3[3].deserialization} \
          {la/la1_path/genblk3[3].sampler} \
          {la/la1_path/genblk3[4].cdc_fifo} \
          {la/la1_path/genblk3[4].compressor} \
          {la/la1_path/genblk3[4].deserialization} \
          {la/la1_path/genblk3[4].sampler} \
          {la/la1_path/genblk3[5].cdc_fifo} \
          {la/la1_path/genblk3[5].compressor} \
          {la/la1_path/genblk3[5].deserialization} \
          {la/la1_path/genblk3[5].sampler} \
          {la/la1_path/genblk3[6].cdc_fifo} \
          {la/la1_path/genblk3[6].compressor} \
          {la/la1_path/genblk3[6].deserialization} \
          {la/la1_path/genblk3[6].sampler} \
          {la/la1_path/genblk3[7].cdc_fifo} \
          {la/la1_path/genblk3[7].compressor} \
          {la/la1_path/genblk3[7].deserialization} \
          {la/la1_path/genblk3[7].sampler} \
          la/la1_path/idelay_n \
          la/la1_path/idelay_p]]
resize_pblock [get_pblocks pblock_la1] -add {CLOCKREGION_X1Y1:CLOCKREGION_X1Y1}
set_property IS_SOFT TRUE [get_pblocks pblock_la1]

# IOSERDES loopback is a false path for timing since the clocks are asynchronous
set_false_path -from [get_pins la/la0_clocks/phase_ctl/oserdes/CLK] -to [get_pins la/la0_clocks/phase_ctl/iserdes/OFB]
set_false_path -from [get_pins la/la1_clocks/phase_ctl/oserdes/CLK] -to [get_pins la/la1_clocks/phase_ctl/iserdes/OFB]

# Tight timing path from BUFIO to BUFG clock
set_max_delay -from [get_pins -hierarchical -filter { NAME =~  "*la*" && NAME =~  "*la*_path*" && NAME =~  "*iserdes*" && NAME =~  "*Q*" }] -to [get_cells -hierarchical -filter { NAME =~  "*la*_path*" && NAME =~  "*fast_data_ff*" }] 0.600

# Path through LUTRAM FIFO can take a little while as it's multicycle
set_max_delay -from [get_clocks *625mhz_fabric*] -through [get_cells -hierarchical *fifomem*] -to [get_clocks *312p5mhz*] 3.200

# Shove slow UART stuff off in the top left
create_pblock pblock_slow_control
add_cells_to_pblock [get_pblocks pblock_slow_control] [get_cells -quiet [list la/la0_ctl la/la1_ctl]]
resize_pblock [get_pblocks pblock_slow_control] -add {SLICE_X0Y150:SLICE_X3Y199}
set_property IS_SOFT FALSE [get_pblocks pblock_slow_control]

########################################################################################################################
# Other timing constraints

# CDC from RGMII to internal 125 MHz clock domain
set_clock_groups -asynchronous -group [get_clocks rgmii_rxc] -group [get_clocks clk_125mhz]
set_clock_groups -asynchronous -group [get_clocks clk_125mhz] -group [get_clocks rgmii_rxc]

# IP stack clock domain crossing
set_clock_groups -asynchronous -group [get_clocks ethernet/sfp/xg_transceiver/inst/gtwizard_10gbe_i/gt0_gtwizard_10gbe_i/gtxe2_i/RXOUTCLK] -group [get_clocks clk_ipstack]
set_clock_groups -asynchronous -group [get_clocks rgmii_rxc] -group [get_clocks clk_ipstack]
set_clock_groups -asynchronous -group [get_clocks clk_ipstack] -group [get_clocks ethernet/sfp/xg_transceiver/inst/gtwizard_10gbe_i/gt0_gtwizard_10gbe_i/gtxe2_i/RXOUTCLK]
set_clock_groups -asynchronous -group [get_clocks clk_ipstack] -group [get_clocks rgmii_rxc]
set_clock_groups -asynchronous -group [get_clocks clk_ipstack] -group [get_clocks clk_125mhz]
set_clock_groups -asynchronous -group [get_clocks clk_125mhz] -group [get_clocks clk_ipstack]
set_clock_groups -asynchronous -group [get_clocks ethernet/sfp/xg_transceiver/inst/gtwizard_10gbe_i/gt0_gtwizard_10gbe_i/gtxe2_i/TXOUTCLK] -group [get_clocks clk_ipstack]
set_clock_groups -asynchronous -group [get_clocks clk_ipstack] -group [get_clocks ethernet/sfp/xg_transceiver/inst/gtwizard_10gbe_i/gt0_gtwizard_10gbe_i/gtxe2_i/TXOUTCLK]

########################################################################################################################
# Floorplanning for RAM

create_pblock pblock_ddr
add_cells_to_pblock [get_pblocks pblock_ddr] [get_cells -quiet [list mem/ram]]
resize_pblock [get_pblocks pblock_ddr] -add {SLICE_X0Y0:SLICE_X23Y149}
resize_pblock [get_pblocks pblock_ddr] -add {DSP48_X0Y0:DSP48_X1Y59}
resize_pblock [get_pblocks pblock_ddr] -add {RAMB18_X0Y0:RAMB18_X1Y59}
resize_pblock [get_pblocks pblock_ddr] -add {RAMB36_X0Y0:RAMB36_X1Y29}
set_property IS_SOFT TRUE [get_pblocks pblock_ddr]

########################################################################################################################
# Floorplanning for Ethernet stuff

create_pblock pblock_sfp
add_cells_to_pblock [get_pblocks pblock_sfp] [get_cells -quiet [list ethernet/sfp expansion_txvr]]
resize_pblock [get_pblocks pblock_sfp] -add {CLOCKREGION_X1Y2:CLOCKREGION_X1Y2}
set_property IS_SOFT FALSE [get_pblocks pblock_sfp]

create_pblock pblock_protocols
add_cells_to_pblock [get_pblocks pblock_protocols] [get_cells -quiet [list ethernet/arp ethernet/arp_mgr ethernet/icmp_ipv4 ethernet/ip_arbiter ethernet/ipv4 ethernet/rx_eth_decoder ethernet/tx_arbiter ethernet/tx_baser_buf ethernet/udp_ipv4]]
resize_pblock [get_pblocks pblock_protocols] -add {CLOCKREGION_X0Y2:CLOCKREGION_X0Y2}

create_pblock pblock_sfp_right
add_cells_to_pblock [get_pblocks pblock_sfp_right] [get_cells -quiet [list ethernet/sfp/mac ethernet/sfp/pcs]]
resize_pblock [get_pblocks pblock_sfp_right] -add {SLICE_X44Y100:SLICE_X53Y149}
resize_pblock [get_pblocks pblock_sfp_right] -add {DSP48_X2Y40:DSP48_X2Y59}

########################################################################################################################
# Floorplanning for SATA stuff

create_pblock pblock_sata
resize_pblock [get_pblocks pblock_sata] -add {CLOCKREGION_X0Y3:CLOCKREGION_X1Y3}
set_property IS_SOFT TRUE [get_pblocks pblock_sata]

########################################################################################################################
# Debugging

create_pblock pblock_xg_cdc
add_cells_to_pblock [get_pblocks pblock_xg_cdc] [get_cells -quiet [list ethernet/baser_rx_cdc]]
resize_pblock [get_pblocks pblock_xg_cdc] -add {SLICE_X36Y125:SLICE_X43Y149}
resize_pblock [get_pblocks pblock_xg_cdc] -add {RAMB18_X2Y50:RAMB18_X2Y59}
resize_pblock [get_pblocks pblock_xg_cdc] -add {RAMB36_X2Y25:RAMB36_X2Y29}
set_property IS_SOFT FALSE [get_pblocks pblock_xg_cdc]


set_clock_groups -asynchronous -group [get_clocks la0_clk_312p5mhz] -group [get_clocks clk_ram_2x_raw]
set_clock_groups -asynchronous -group [get_clocks la1_clk_312p5mhz] -group [get_clocks clk_ram_2x_raw]
set_clock_groups -asynchronous -group [get_clocks clk_ram_2x_raw] -group [get_clocks la0_clk_312p5mhz]
set_clock_groups -asynchronous -group [get_clocks clk_ram_2x_raw] -group [get_clocks la1_clk_312p5mhz]


set_max_delay -datapath_only -from [get_cells -hierarchical -filter { NAME =~  "*wr_data_fifo*" && NAME =~  "*sync*" && NAME =~  "*reg_a_ff*" }] -to [get_cells -hierarchical -filter { NAME =~  "*wr_data_fifo*" && NAME =~  "*sync*" && NAME =~  "*reg_b*" }] 3.200

set_max_delay -datapath_only -from [get_cells -hierarchical -filter { NAME =~  "*wr_data_fifo*" && NAME =~  "*sync*" && NAME =~  "*dout0*" }] -to [get_cells -hierarchical -filter { NAME =~  "*wr_data_fifo*" && NAME =~  "*sync*" && NAME =~  "*dout1*" }] 3.200
set_property C_CLK_INPUT_FREQ_HZ 300000000 [get_debug_cores dbg_hub]
set_property C_ENABLE_CLK_DIVIDER false [get_debug_cores dbg_hub]
set_property C_USER_SCAN_CHAIN 1 [get_debug_cores dbg_hub]
connect_debug_port dbg_hub/clk [get_nets clk_125mhz]
