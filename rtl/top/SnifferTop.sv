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

/*
	MEMORY BUS FORMAT

	The DRAM (Kingston KVR16LS11S6/2) is 2^28 locations x 64 bits = 2GB (0x8000_0000) bytes in size.
	A burst is 4 clocks so if we only do aligned accesses, we have 2^26 256-bit blocks.

	RAM subsystem is 162.5 MHz on a 256 bit bus.
	Hope to upgrade to 200 MHz on a 256 bit bus (51.2 Gbps peak throughput)

	BITS (assuming 32-bit linear address space)

	TODO: this implies we should have one more address bit than we actually do... what gives?
	31				Unmapped, always 0
	30:16			Row address (15)
	15:13			Bank address (3)
	12:3			Column address (10)
	2:0				Unimplemented, always 0 (64 bit alignment)

	Rationale for choosing row-bank-col over bank-row-col: if we partition half the ram for LA and half for
	SATA with bank as the MSB then we have 4 banks statically allocated to the entire LA subsystem and 4 banks to the
	SATA subsystem. Each set of four adjacent channels will share a single bank, which is great if the expected write
	load per channel is similar. But it's not.

	If we use row-bank-col, OTOH, large write bursts within a single LA channel can stripe across multiple banks. We
	expect some channels to be much more compressible than others, so this allows the poorly compressible channels to
	get better performance by using multiple banks for bulk writes.

	MEMORY MAP

	Byte addressed for readability. Physical address bus is missing low 3 bits because 64 bit wide SODIMM

	START			END
	0000_0000		3FFF_FFFF			Reserved for use by SATA subsystem

	LOGIC ANALYZER BUFFER MEMORY
	START			END
	4000_0000		43FF_FFFF			LA0.0
	4400_0000		47FF_FFFF			LA0.1
	4800_0000		4BFF_FFFF			LA0.2
	4C00_0000		4FFF_FFFF			LA0.3
	5000_0000		53FF_FFFF			LA0.4
	5400_0000		57FF_FFFF			LA0.5
	5800_0000		5BFF_FFFF			LA0.6
	5C00_0000		5FFF_FFFF			LA0.7
	6000_0000		63FF_FFFF			LA1.0
	6400_0000		67FF_FFFF			LA1.1
	6800_0000		6BFF_FFFF			LA1.2
	6C00_0000		6FFF_FFFF			LA1.3
	7000_0000		73FF_FFFF			LA1.4
	7400_0000		77FF_FFFF			LA1.7
	7800_0000		7BFF_FFFF			LA1.6
	7C00_0000		7FFF_FFFF			LA1.7

	Each LA channel has 0x4000000 bytes or 64 MByte / 512 Mbit / 2 Mrows of sample buffer.


	Each row can store 15 compression blocks, or 240 to 3810 bits, depending on compression ratio.
	This gives a guaranteed memory depth of 480M samples, possibly up to 7620M with optimal compression.
 */
module SnifferTop(

	//Clock inputs
	input wire			clk_125mhz_p,
	input wire			clk_125mhz_n,

	input wire			clk_200mhz_p,
	input wire			clk_200mhz_n,

	input wire			gtx_refclk_156_p,
	input wire			gtx_refclk_156_n,
	input wire			gtx_refclk_200_p,
	input wire			gtx_refclk_200_n,

	/*
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
	//input wire		sma_rx_p,
	//input wire		sma_rx_n,
	output wire			sma_tx_p,
	output wire			sma_tx_n,

	//10Gbase-R
	output wire			sfp_scl,
	inout wire			sfp_sda,
	input wire			sfp_tx_fault,
	output wire			sfp_tx_disable,
	input wire			sfp_mod_abs,
	input wire			sfp_rx_los,
	output wire[1:0]	sfp_rs,

	input wire			sfp_rx_p,
	input wire			sfp_rx_n,
	output wire			sfp_tx_p,
	output wire			sfp_tx_n,

	//RGMII
	output wire			eth_mdc,
	inout wire			eth_mdio,
	output wire			eth_rst_n,
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
	// Top level clock generation

	wire	clk_125mhz;
	wire	clk_200mhz;
	wire	clk_250mhz;
	wire	clk_ipstack;
	wire	clk_400mhz;

	wire[1:0]	sys_pll_lock;

	ClockGeneration clockgen(
		.clk_125mhz_p(clk_125mhz_p),
		.clk_125mhz_n(clk_125mhz_n),
		.clk_200mhz_p(clk_200mhz_p),
		.clk_200mhz_n(clk_200mhz_n),

		.clk_125mhz(clk_125mhz),
		.clk_200mhz(clk_200mhz),
		.clk_250mhz(clk_250mhz),
		.clk_ipstack(clk_ipstack),
		.clk_400mhz(clk_400mhz),

		.pll_lock(sys_pll_lock)
	);

	wire qpll_clk;
	wire qpll_refclk;
	wire qpll_lock;
	wire qpll_refclk_lost;

	QuadPLL qpll(
		.gtx_refclk_156_p(gtx_refclk_156_p),
		.gtx_refclk_156_n(gtx_refclk_156_n),
		.gtx_refclk_200_p(gtx_refclk_200_p),
		.gtx_refclk_200_n(gtx_refclk_200_n),

		.clk_125mhz(clk_125mhz),

		.qpll_clk(qpll_clk),
		.qpll_refclk(qpll_refclk),
		.qpll_lock(qpll_lock),
		.qpll_refclk_lost(qpll_refclk_lost)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Ethernet

	EthernetSubsystem ethernet(
		.clk_125mhz(clk_125mhz),
		.clk_250mhz(clk_250mhz),
		.clk_ipstack(clk_ipstack),

		.qpll_clk(qpll_clk),
		.qpll_refclk(qpll_refclk),
		.qpll_refclk_lost(qpll_refclk_lost),
		.qpll_lock(qpll_lock),

		.sfp_scl(sfp_scl),
		.sfp_sda(sfp_sda),
		.sfp_tx_fault(sfp_tx_fault),
		.sfp_tx_disable(sfp_tx_disable),
		.sfp_mod_abs(sfp_mod_abs),
		.sfp_rx_los(sfp_rx_los),
		.sfp_rs(sfp_rs),

		.sfp_rx_p(sfp_rx_p),
		.sfp_rx_n(sfp_rx_n),
		.sfp_tx_p(sfp_tx_p),
		.sfp_tx_n(sfp_tx_n),

		.eth_mdc(eth_mdc),
		.eth_mdio(eth_mdio),
		.eth_rst_n(eth_rst_n),
		.eth_led_n_1v8(eth_led_n_1v8),
		.eth_led_p_3v3(eth_led_p_3v3),
		.rgmii_rxc(rgmii_rxc),
		.rgmii_rx_dv(rgmii_rx_dv),
		.rgmii_rxd(rgmii_rxd),
		.rgmii_txc(rgmii_txc),
		.rgmii_tx_en(rgmii_tx_en),
		.rgmii_txd(rgmii_txd)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Logic analyzer pods

	wire	ddr_cal_complete;
	wire	ddr3_user_clk;

	wire	la0_align_done;
	wire	la1_align_done;
	LogicAnalyzerSubsystem la(
		.clk_125mhz(clk_125mhz),
		.clk_400mhz(clk_400mhz),

		.la0_p(la0_p),
		.la0_n(la0_n),
		.la0_present_n(la0_present_n),
		.la0_12v_fault_n(la0_12v_fault_n),
		.la0_12v_en(la0_12v_en),
		.la0_uart_rx(la0_uart_rx),
		.la0_uart_tx(la0_uart_tx),
		.la1_p(la1_p),
		.la1_n(la1_n),
		.la1_present_n(la1_present_n),
		.la1_12v_fault_n(la1_12v_fault_n),
		.la1_12v_en(la1_12v_en),
		.la1_uart_rx(la1_uart_rx),
		.la1_uart_tx(la1_uart_tx),

		.ram_ready(ddr_cal_complete),
		.ram_clk(ddr3_user_clk),

		.la0_align_done(la0_align_done),
		.la1_align_done(la1_align_done)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// DRAM controller

	wire[28:0]	ddr3_app_addr;
	wire[2:0]	ddr3_app_cmd;
	wire		ddr3_app_en;

	wire[255:0]	ddr3_app_wdf_data;
	wire		ddr3_app_wdf_end;
	wire[31:0]	ddr3_app_wdf_mask;
	wire		ddr3_app_wdf_wren;
	wire		ddr3_app_wdf_rdy;

	wire		ddr3_app_ref_req;
	wire		ddr3_app_sr_req;
	wire		ddr3_app_zq_req;

	wire[255:0]	ddr3_app_rd_data;
	wire		ddr3_app_rd_data_end;
	wire		ddr3_app_rd_data_valid;

	wire		ddr3_app_rdy;

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
		.app_rd_data(ddr3_app_rd_data),
		.app_rd_data_end(ddr3_app_rd_data_end),
		.app_rd_data_valid(ddr3_app_rd_data_valid),
		.app_rdy(ddr3_app_rdy),
		.app_wdf_rdy(ddr3_app_wdf_rdy),
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
	*/
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Extra GTX for PRBS generation

	ExpansionTransceiver expansion_txvr(
		.clk_125mhz(clk_125mhz),

		.sma_tx_p(sma_tx_p),
		.sma_tx_n(sma_tx_n),

		.qpll_clk(qpll_clk),
		.qpll_refclk(qpll_refclk),
		.qpll_refclk_lost(qpll_refclk_lost),
		.qpll_lock(qpll_lock),

		.sys_pll_lock(sys_pll_lock)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Debug LEDs

	assign gpio_led[0] = ddr_cal_complete;
	assign gpio_led[1] = 1'b0;
	assign gpio_led[2] = la0_align_done;
	assign gpio_led[3] = la1_align_done;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Debug IP
	/*
	vio_0 vio(
		.clk(prbs_tx_clk),

		.probe_in0(qpll_lock),
		.probe_in1(qpll_refclk_lost),
		.probe_out0(prbs_sel),
		.probe_out1(tx_data)
	);
	*/

endmodule
