`timescale 1ns / 1ps
`default_nettype none

/***********************************************************************************************************************
*                                                                                                                      *
* sata-sniffer v0.1                                                                                                    *
*                                                                                                                      *
* Copyright (c) 2021-2022 Andrew D. Zonenberg and contributors                                                         *
* All rights reserved.                                                                                                 *
*                                                                                                                      *
* Redistribution and use in source and binary forms, with or without modification, are permitted provided that the     *
* following conditions are met:                                                                                        *
*                                                                                                                      *
*    * Redistributions of source code must retain the above copyright notice, this list of conditions, and the         *
*      following disclaimer.                                                                                           *
*                                                                                                                      *
*    * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the       *
*      following disclaimer in the documentation and/or other materials provided with the distribution.                *
*                                                                                                                      *
*    * Neither the name of the author nor the names of any contributors may be used to endorse or promote products     *
*      derived from this software without specific prior written permission.                                           *
*                                                                                                                      *
* THIS SOFTWARE IS PROVIDED BY THE AUTHORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED   *
* TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL *
* THE AUTHORS BE HELD LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES        *
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR       *
* BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT *
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE       *
* POSSIBILITY OF SUCH DAMAGE.                                                                                          *
*                                                                                                                      *
***********************************************************************************************************************/

module SnifferTop(

	//Clock inputs
	input wire			clk_125mhz_p,
	input wire			clk_125mhz_n,

	input wire			clk_200mhz_p,
	input wire			clk_200mhz_n,

	/*
	input wire			gtx_refclk_156_p,
	input wire			gtx_refclk_156_n,
	input wire			gtx_refclk_200_p,
	input wire			gtx_refclk_200_n,

	//SATA device
	input wire			sata_device_rx_p,
	input wire			sata_device_rx_n,
	output wire			sata_device_tx_p,
	output wire			sata_device_tx_n,

	//SATA host
	input wire			sata_host_rx_p,
	input wire			sata_host_rx_n,
	output wire			sata_host_tx_p,
	output wire			sata_host_tx_n,

	*/
	//Extra GTX

	//10Gbase-R
	output wire			sfp_scl,
	inout wire			sfp_sda,
	input wire			sfp_tx_fault,
	output wire			sfp_tx_disable,
	input wire			sfp_mod_abs,
	input wire			sfp_rx_los,
	output wire[1:0]	sfp_rs,
	/*
	input wire			sfp_rx_p,
	input wire			sfp_rx_n,
	output wire			sfp_tx_p,
	output wire			sfp_tx_n,
	*/

	//RGMII
	output wire			eth_mdc,
	inout wire			eth_mdio,
	output logic		eth_rst_n	= 0,
	input wire[1:0]		eth_led_n_1v8,
	output wire[1:0]	eth_led_p_3v3,
	input wire			rgmii_rxc,
	input wire			rgmii_rx_dv,
	input wire[3:0]		rgmii_rxd,
	output wire			rgmii_txc,
	output wire			rgmii_tx_en,
	output wire[3:0]	rgmii_txd,

	//PMOD
	inout wire[7:0]		pmod_dq,

	//Logic analyzer pods
	input wire[7:0]		la0_p,
	input wire[7:0]		la0_n,
	input wire			la0_present_n,
	input wire			la0_12v_fault_n,
	output wire			la0_12v_en,
	input wire			la0_uart_rx,
	output wire			la0_uart_tx,

	input wire[7:0]		la1_p,
	input wire[7:0]		la1_n,
	input wire			la1_present_n,
	input wire			la1_12v_fault_n,
	output wire			la1_12v_en,
	input wire			la1_uart_rx,
	output wire			la1_uart_tx,

	//GPIO LEDs
	output wire[3:0]	gpio_led,

	//LVDS GPIO
	input wire[15:0] 	lvds_gpio_p,
	input wire[15:0]	lvds_gpio_n,

	//DDR3 (current config is for single rank SODIMM)
	output wire			ram_scl,
	inout wire			ram_sda,
	inout wire[63:0]	ddr3_dq,
	inout wire[7:0]		ddr3_dqs_n,
	inout wire[7:0]		ddr3_dqs_p,
	output wire[14:0]	ddr3_addr,
	output wire[2:0]	ddr3_ba,
	output wire			ddr3_ras_n,
	output wire			ddr3_cas_n,
	output wire			ddr3_we_n,
	output wire			ddr3_reset_n,
	output wire[0:0]	ddr3_ck_p,
	output wire[0:0]	ddr3_ck_n,
	output wire[0:0]	ddr3_cke,
	output wire[0:0]	ddr3_cs_n,
	output wire[7:0]	ddr3_dm,
	output wire[0:0]	ddr3_odt,

	//SPI flash
	output wire			flash_si,
	input wire			flash_so,
	output wire			flash_cs_n
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// GTX clock inputs
	/*
	wire	gtx_refclk_156;
	wire	gtx_refclk_200;

	IBUFDS_GTE2 clk_buf_refclk_156(
		.I(gtx_refclk_156_p),
		.IB(gtx_refclk_156_n),
		.CEB(1'b0),
		.O(gtx_refclk_156),
		.ODIV2()
	);

	IBUFDS_GTE2 clk_buf_refclk_200(
		.I(gtx_refclk_200_p),
		.IB(gtx_refclk_200_n),
		.CEB(1'b0),
		.O(gtx_refclk_200),
		.ODIV2()
	);
	*/

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// LVDS GPIOs

	//We don't do anything with them for now.
	//But instantiate the buffers as a placeholder for later.
	//TODO: channels 2 and 9 are inverted for routability, add inverters somewhere!
	wire[15:0]	lvds_gpio_in;
	DifferentialInputBuffer #(
		.WIDTH(16)
	) gpio_ibuf (
		.pad_in_p(lvds_gpio_p),
		.pad_in_n(lvds_gpio_n),
		.fabric_out(lvds_gpio_in)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Clock synthesis

	wire	clk_125mhz;
	wire	clk_200mhz;
	wire	clk_250mhz;
	wire	clk_400mhz;

	ClockGeneration clockgen(
		.clk_125mhz_p(clk_125mhz_p),
		.clk_125mhz_n(clk_125mhz_n),
		.clk_200mhz_p(clk_200mhz_p),
		.clk_200mhz_n(clk_200mhz_n),

		.clk_125mhz(clk_125mhz),
		.clk_200mhz(clk_200mhz),
		.clk_250mhz(clk_250mhz),
		.clk_400mhz(clk_400mhz),

		.pll_lock()
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Logic analyzer pods

	wire	la0_clk_312p5mhz;
	wire	la0_clk_625mhz_fabric;
	wire	la0_clk_625mhz_io_0;
	wire	la0_clk_625mhz_io_90;

	wire	la0_align_done;

	LogicPodClocking la0_clocks(
		.clk_125mhz(clk_125mhz),

		.clk_312p5mhz(la0_clk_312p5mhz),
		.clk_625mhz_io_0(la0_clk_625mhz_io_0),
		.clk_625mhz_io_90(la0_clk_625mhz_io_90),
		.clk_625mhz_fabric(la0_clk_625mhz_fabric),

		.pll_lock(),
		.align_done(la0_align_done)
	);

	LogicPodDatapath #(.LANE_INVERT(8'b10011100)) la0_path (
		.clk_312p5mhz(la0_clk_312p5mhz),
		.clk_400mhz(clk_400mhz),
		.clk_625mhz_io_0(la0_clk_625mhz_io_0),
		.clk_625mhz_io_90(la0_clk_625mhz_io_90),
		.clk_625mhz_fabric(la0_clk_625mhz_fabric),
		.pod_data_p(la0_p),
		.pod_data_n(la0_n));

	/*
	LogicPodDatapath #(.LANE_INVERT(8'b00000110)) la1_path (
		.clk_312p5mhz(clk_312p5mhz),
		.clk_400mhz(clk_400mhz),
		.clk_625mhz_io_0(clk_625mhz_io_0),
		.clk_625mhz_io_90(clk_625mhz_io_90),
		.clk_625mhz_fabric(clk_625mhz_fabric),
		.pod_data_p(la1_p),
		.pod_data_n(la1_n));*/

	LogicPodControl la0_ctl(
		.clk_125mhz(clk_125mhz),
		.pod_present_n(la0_present_n),
		.pod_power_fault_n(la0_12v_fault_n),
		.pod_power_en(la0_12v_en),
		.uart_rx(la0_uart_rx),
		.uart_tx(la0_uart_tx)
		);

	LogicPodControl la1_ctl(
		.clk_125mhz(clk_125mhz),
		.pod_present_n(la1_present_n),
		.pod_power_fault_n(la1_12v_fault_n),
		.pod_power_en(la1_12v_en),
		.uart_rx(la1_uart_rx),
		.uart_tx(la1_uart_tx)
		);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// DRAM controller

	wire		ddr_cal_complete;
	wire		ddr3_user_clk;

	wire[28:0]	ddr3_app_addr;
	wire[2:0]	ddr3_app_cmd;
	wire		ddr3_app_en;

	wire[511:0]	ddr3_app_wdf_data;
	wire		ddr3_app_wdf_end;
	wire[63:0]	ddr3_app_wdf_mask;

	wire		ddr3_app_wdf_wren;

	wire		ddr3_app_ref_req;
	wire		ddr3_app_sr_req;
	wire		ddr3_app_zq_req;

	ddr3 ram(

		//Top level pins
		.ddr3_dq(ddr3_dq),
		.ddr3_dqs_p(ddr3_dqs_p),
		.ddr3_dqs_n(ddr3_dqs_n),
		.ddr3_addr(ddr3_addr),
		.ddr3_ba(ddr3_ba),
		.ddr3_ras_n(ddr3_ras_n),
		.ddr3_cas_n(ddr3_cas_n),
		.ddr3_we_n(ddr3_we_n),
		.ddr3_reset_n(ddr3_reset_n),
		.ddr3_ck_p(ddr3_ck_p),
		.ddr3_ck_n(ddr3_ck_n),
		.ddr3_cke(ddr3_cke),
		.ddr3_cs_n(ddr3_cs_n),
		.ddr3_dm(ddr3_dm),
		.ddr3_odt(ddr3_odt),

		//Internal ports
		.sys_clk_i(clk_200mhz),	//Main controller clock
		.clk_ref_i(clk_200mhz),	//Reference clock for IDELAYCTRLs
		.app_addr(ddr3_app_addr),
		.app_cmd(ddr3_app_cmd),
		.app_en(ddr3_app_en),
		.app_wdf_data(ddr3_app_wdf_data),
		.app_wdf_end(ddr3_app_wdf_end),
		.app_wdf_mask(ddr3_app_wdf_mask),
		.app_wdf_wren(ddr3_app_wdf_wren),
		.app_rd_data(),
		.app_rd_data_end(),
		.app_rd_data_valid(),
		.app_rdy(),
		.app_wdf_rdy(),
		.app_sr_req(ddr3_app_sr_req),
		.app_ref_req(ddr3_app_ref_req),
		.app_zq_req(ddr3_app_zq_req),
		.app_sr_active(),
		.app_ref_ack(),
		.app_zq_ack(),
		.ui_clk(ddr3_user_clk),
		.ui_clk_sync_rst(),
		.init_calib_complete(ddr_cal_complete),
		.device_temp(),
		.sys_rst(1'b0)
	);

	//Tie off unused pins
	assign ddr3_app_addr = 0;
	assign ddr3_app_cmd = 0;
	assign ddr3_app_en = 0;
	assign ddr3_app_wdf_data = 0;
	assign ddr3_app_wdf_end = 0;
	assign ddr3_app_wdf_mask = 0;
	assign ddr3_app_wdf_wren = 0;
	assign ddr3_app_ref_req = 0;
	assign ddr3_app_zq_req = 0;
	assign ddr3_app_sr_req = 0;

	/*
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// SERDES quad PLL

	wire	qpll_clk;

	GTXE2_COMMON #(
		.SIM_QPLL_REFCLK_SEL(3'b010),	//must match QPLLREFCLKSEL
		.SIM_VERSION(4.0),

		//FIXME
		.QPLL_CFG(),
		.QPLL_CLKOUT_CFG(),
		.QPLL_COARSE_FREQ_OVRD(),
		.QPLL_COARSE_FREQ_OVRD_EN(1'b0),
		.QPLL_CP(),
		.QPLL_CP_MONITOR_EN(1'b0),
		.QPLL_DMONITOR_SEL().
		.QPLL_FBDIV(66),
		.QPLL_FBDIV_MONITOR_EN(1'b),
		.QPLL_FBDIV_RATIO(1'b0),
		.QPLL_INIT_CFG(),
		.QPLL_LOCK_CFG(),
		.QPLL_LPF(),
		.QPLL_REFCLK_DIV(),
		.RXOUT_DIV()
		.TXOUT_DIV(),
		.COMMON_CFG()

	) serdes_common(
		.QPLLDMONITOR(),
		.QPLLFBCLKLOST(),
		.QPLLLOCK(),
		.QPLLLOCKDETCLK(),
		.QPLLLOCKEN(1'b1),
		.QPLLOUTCLK(qpll_clk),
		.QPLLOUTRESET(1'b0),
		.QPLLPD(1'b0),
		.QPLLREFCLKLOST(),
		.QPLLREFCLKSEL(3'b010),			//use REFCLK1
		.QPLLRESET(1'b0),
		.QPLLRSVD1(16'h0),
		.QPLLRSVD2(5'h0),
		.BGBYPASB(1'b1),
		.BGMONITORENB(1'b1),
		.BGPDB(1'b1),
		.BGRCALOVRD(5'b11111),
		.BGRCALOVRDENB(1'b1),
		.RCALENB(1'b1),
		.PMARSVD(1'b0)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// SATA device

	gtwizard_sata_device device_transceiver(
		.sysclk_in(),
		.soft_reset_tx_in(1'b0),
		.soft_reset_rx_in(1'b0),
		.dont_reset_on_data_error_in(),
		.gt0_tx_fsm_reset_done_out(),
		.gt0_rx_fsm_reset_done_out(),
		.gt0_data_valid_in(),
		.gt0_tx_mmcm_lock_in(),
		.gt0_tx_mmcm_reset_out(),
		.gt0_cpllfbclklost_out(),
		.gt0_cplllock_out(),
		.gt0_cplllockdetclk_in(),
		.gt0_cpllreset_in(),
		.gt0_gtrefclk0_in(gtx_refclk_156),
		.gt0_gtrefclk1_in(gtx_refclk_200),
		.gt0_drpaddr_in(),
		.gt0_drpclk_in(),
		.gt0_drpdi_in(),
		.gt0_drpdo_out(),
		.gt0_drpen_in(),
		.gt0_drprdy_out(),
		.gt0_drpwe_in(),
		.gt0_dmonitorout_out(),
		.gt0_rxrate_in(),
		.gt0_eyescanreset_in(),
		.gt0_rxuserrdy_in(),
		.gt0_eyescandataerror_out(),
		.gt0_eyescantrigger_in(),
		.gt0_rxusrclk_in(),
		.gt0_rxusrclk2_in(),
		.gt0_rxdata_out(),
		.gt0_rxdisperr_out(),
		.gt0_rxnotintable_out(),
		.gt0_gtxrxp_in(sata_device_rx_p),
		.gt0_gtxrxn_in(sata_device_rx_n),
		.gt0_rxphmonitor_out(),
		.gt0_rxphslipmonitor_out(),
		.gt0_rxbyteisaligned_out(),
		.gt0_rxbyterealign_out(),
		.gt0_rxcommadet_out(),
		.gt0_rxdfelpmreset_in(),
		.gt0_rxmonitorout_out(),
		.gt0_rxmonitorsel_in(),
		.gt0_rxratedone_out(),
		.gt0_rxoutclk_out(),
		.gt0_rxoutclkfabric_out(),
		.gt0_gtrxreset_in(),
		.gt0_rxpmareset_in(),
		.gt0_rxcomsasdet_out(),
		.gt0_rxcomwakedet_out(),
		.gt0_rxcominitdet_out(),
		.gt0_rxelecidle_out(),
		.gt0_rxslide_in(),
		.gt0_rxchariscomma_out(),
		.gt0_rxcharisk_out(),
		.gt0_rxresetdone_out(),
		.gt0_gttxreset_in(),
		.gt0_txuserrdy_in(),
		.gt0_txusrclk_in(),
		.gt0_txusrclk2_in(),
		.gt0_txelecidle_in(),
		.gt0_txrate_in(),
		.gt0_txdata_in(),
		.gt0_gtxtxn_out(sata_device_tx_n),
		.gt0_gtxtxp_out(sata_device_tx_p),
		.gt0_txoutclk_out(),
		.gt0_txoutclkfabric_out(),
		.gt0_txoutclkpcs_out(),
		.gt0_txratedone_out(),
		.gt0_txcharisk_in(),
		.gt0_txresetdone_out(),
		.gt0_txcomfinish_out(),
		.gt0_txcominit_in(),
		.gt0_txcomsas_in(),
		.gt0_txcomwake_in(),
		.gt0_qplloutclk_in(),
		.gt0_qplloutrefclk_in()
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// SATA host

	gtwizard_sata_host host_transceiver(
		.sysclk_in(),
		.soft_reset_tx_in(1'b0),
		.soft_reset_rx_in(1'b0),
		.dont_reset_on_data_error_in(),
		.gt0_tx_fsm_reset_done_out(),
		.gt0_rx_fsm_reset_done_out(),
		.gt0_data_valid_in(),
		.gt0_tx_mmcm_lock_in(),
		.gt0_tx_mmcm_reset_out(),
		.gt0_cpllfbclklost_out(),
		.gt0_cplllock_out(),
		.gt0_cplllockdetclk_in(),
		.gt0_cpllreset_in(),
		.gt0_gtrefclk0_in(gtx_refclk_156),
		.gt0_gtrefclk1_in(gtx_refclk_200),
		.gt0_drpaddr_in(),
		.gt0_drpclk_in(),
		.gt0_drpdi_in(),
		.gt0_drpdo_out(),
		.gt0_drpen_in(),
		.gt0_drprdy_out(),
		.gt0_drpwe_in(),
		.gt0_dmonitorout_out(),
		.gt0_rxrate_in(),
		.gt0_eyescanreset_in(),
		.gt0_rxuserrdy_in(),
		.gt0_eyescandataerror_out(),
		.gt0_eyescantrigger_in(),
		.gt0_rxusrclk_in(),
		.gt0_rxusrclk2_in(),
		.gt0_rxdata_out(),
		.gt0_rxdisperr_out(),
		.gt0_rxnotintable_out(),
		.gt0_gtxrxp_in(sata_host_rx_p),
		.gt0_gtxrxn_in(sata_host_rx_n),
		.gt0_rxphmonitor_out(),
		.gt0_rxphslipmonitor_out(),
		.gt0_rxbyteisaligned_out(),
		.gt0_rxbyterealign_out(),
		.gt0_rxcommadet_out(),
		.gt0_rxdfelpmreset_in(),
		.gt0_rxmonitorout_out(),
		.gt0_rxmonitorsel_in(),
		.gt0_rxratedone_out(),
		.gt0_rxoutclk_out(),
		.gt0_rxoutclkfabric_out(),
		.gt0_gtrxreset_in(),
		.gt0_rxpmareset_in(),
		.gt0_rxcomsasdet_out(),
		.gt0_rxcomwakedet_out(),
		.gt0_rxcominitdet_out(),
		.gt0_rxelecidle_out(),
		.gt0_rxslide_in(),
		.gt0_rxchariscomma_out(),
		.gt0_rxcharisk_out(),
		.gt0_rxresetdone_out(),
		.gt0_gttxreset_in(),
		.gt0_txuserrdy_in(),
		.gt0_txusrclk_in(),
		.gt0_txusrclk2_in(),
		.gt0_txelecidle_in(),
		.gt0_txrate_in(),
		.gt0_txdata_in(),
		.gt0_gtxtxn_out(sata_host_tx_n),
		.gt0_gtxtxp_out(sata_host_tx_p),
		.gt0_txoutclk_out(),
		.gt0_txoutclkfabric_out(),
		.gt0_txoutclkpcs_out(),
		.gt0_txratedone_out(),
		.gt0_txcharisk_in(),
		.gt0_txresetdone_out(),
		.gt0_txcomfinish_out(),
		.gt0_txcominit_in(),
		.gt0_txcomsas_in(),
		.gt0_txcomwake_in(),
		.gt0_qplloutclk_in(),
		.gt0_qplloutrefclk_in()
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Extra GTX for PRBS generation

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// 10G Ethernet

	gtwizard_10gbe xg_transceiver(
		.sysclk_in(),
		.soft_reset_tx_in(1'b0),
		.soft_reset_rx_in(1'b0),
		.dont_reset_on_data_error_in(),
		.gt0_tx_fsm_reset_done_out(),
		.gt0_rx_fsm_reset_done_out(),
		.gt0_data_valid_in(),
		.gt0_tx_mmcm_lock_in(),
		.gt0_tx_mmcm_reset_out(),
		.gt0_drpaddr_in(),
		.gt0_drpclk_in(),
		.gt0_drpdi_in(),
		.gt0_drpdo_out(),
		.gt0_drpen_in(),
		.gt0_drprdy_out(),
		.gt0_drpwe_in(),
		.gt0_dmonitorout_out(),
		.gt0_eyescanreset_in(),
		.gt0_rxuserrdy_in(),
		.gt0_eyescandataerror_out(),
		.gt0_eyescantrigger_in(),
		.gt0_rxusrclk_in(),
		.gt0_rxusrclk2_in(),
		.gt0_rxdata_out(),
		.gt0_gtxrxp_in(sfp_rx_p),
		.gt0_gtxrxn_in(sfp_rx_n),
		.gt0_rxphmonitor_out(),
		.gt0_rxphslipmonitor_out(),
		.gt0_rxdfelpmreset_in(),
		.gt0_rxmonitorout_out(),
		.gt0_rxmonitorsel_in(),
		.gt0_rxoutclk_out(),
		.gt0_rxoutclkfabric_out(),
		.gt0_rxdatavalid_out(),
		.gt0_rxheader_out(),
		.gt0_rxheadervalid_out(),
		.gt0_rxgearboxslip_in(),
		.gt0_gtrxreset_in(),
		.gt0_rxpcsreset_in(),
		.gt0_rxpmareset_in(),
		.gt0_rxresetdone_out(),
		.gt0_txpostcursor_in(),
		.gt0_txprecursor_in(),
		.gt0_gttxreset_in(),
		.gt0_txuserrdy_in(),
		.gt0_txusrclk_in(),
		.gt0_txusrclk2_in(),
		.gt0_txbufstatus_out(),
		.gt0_txmaincursor_in(),
		.gt0_txdata_in(),
		.gt0_gtxtxn_out(sfp_tx_n),
		.gt0_gtxtxp_out(sfp_tx_p),
		.gt0_txoutclk_out(),
		.gt0_txoutclkfabric_out(),
		.gt0_txoutclkpcs_out(),
		.gt0_txheader_in(),
		.gt0_txsequence_in(),
		.gt0_txpcsreset_in(),
		.gt0_txpmareset_in(),
		.gt0_txresetdone_out(),
		.gt0_qplllock_in(),
		.gt0_qpllrefclklost_in(),
		.gt0_qpllreset_out(),
		.gt0_qplloutclk_in(),
		.gt0_qplloutrefclk_in()
	);
	*/
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Ethernet LED level shiters

	assign eth_led_p_3v3 = ~eth_led_n_1v8;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// 1G Ethernet

	//Bring up the PHY after a little while
	logic[15:0] eth_rst_count = 1;
	always_ff @(posedge clk_125mhz) begin
		if(eth_rst_count == 0)
			eth_rst_n		<= 1;
		else
			eth_rst_count	<= eth_rst_count + 1'h1;
	end

	wire		baset_link_up;
	lspeed_t	baset_link_speed;

	wire			mac_rx_clk;
	EthernetRxBus	mac_rx_bus;

	EthernetTxBus	mac_tx_bus;
	wire			mac_tx_ready;

	RGMIIMACWrapper gig_mac_wrapper(
		.clk_125mhz(clk_125mhz),
		.clk_250mhz(clk_250mhz),

		.rgmii_rxc(rgmii_rxc),
		.rgmii_rxd(rgmii_rxd),
		.rgmii_rx_ctl(rgmii_rx_dv),

		.rgmii_txc(rgmii_txc),
		.rgmii_txd(rgmii_txd),
		.rgmii_tx_ctl(rgmii_tx_en),

		.mac_rx_clk(mac_rx_clk),
		.mac_rx_bus(mac_rx_bus),
		.mac_tx_bus(mac_tx_bus),
		.mac_tx_ready(mac_tx_ready),
		.link_up(baset_link_up),
		.link_speed(baset_link_speed)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Debug LEDs

	assign gpio_led[0] = 1'b0;
	assign gpio_led[1] = 1'b0;
	assign gpio_led[2] = ddr_cal_complete;
	assign gpio_led[3] = la0_align_done;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Debug IP

	/*
	wire	ddr_cal_complete_sync;
	ThreeStageSynchronizer sync_1(
		.clk_in(ddr3_user_clk),
		.din(ddr_cal_complete),
		.clk_out(clk_125mhz),
		.dout(ddr_cal_complete_sync));

	vio_0 vio(
		.clk(clk_125mhz),

		.probe_in0(baset_link_up),
		.probe_in1(ddr_cal_complete_sync),
		.probe_in2(la0_present_n),
		.probe_in3(sfp_tx_fault),
		.probe_in4(la0_12v_fault_n),
		.probe_in5(eth_led_n_1v8),
		.probe_in6(baset_link_speed),
		.probe_in7(sfp_mod_abs)
	);
	*/

endmodule
