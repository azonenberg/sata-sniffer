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

/**
	@brief SFP+ control, 10G Ethernet MAC/PCS, etc
 */
module SFPInterface(

	//Global clocks
	input wire				clk_125mhz,

	//Inputs from quad PLL
	input wire				qpll_clk,
	input wire				qpll_refclk,
	input wire				qpll_refclk_lost,
	input wire				qpll_lock,

	//SFP+ signals
	output wire				sfp_scl,
	inout wire				sfp_sda,
	input wire				sfp_tx_fault,
	output wire				sfp_tx_disable,
	input wire				sfp_mod_abs,
	input wire				sfp_rx_los,
	output wire[1:0]		sfp_rs,

	input wire				sfp_rx_p,
	input wire				sfp_rx_n,
	output wire				sfp_tx_p,
	output wire				sfp_tx_n,

	//Status signals
	output wire				link_up,

	//Main MAC bus signals
	output wire				xgmii_rx_clk,
	output EthernetRxBus	mac_rx_bus,
	output wire				xgmii_tx_clk,
	input EthernetTxBus		mac_tx_bus
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Gate transmit when optic is removed or laser fault is reported

	assign sfp_tx_disable = (sfp_tx_fault || sfp_mod_abs);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// The transceiver instance

	wire[31:0]	sfp_rx_data;
	wire		sfp_rx_data_valid;
	wire[1:0]	sfp_rx_header;
	wire		sfp_rx_header_valid;
	wire		sfp_rx_bitslip;

	wire[31:0]	sfp_tx_data;

	wire		sfp_rx_clk_raw;
	wire		sfp_rx_clk;

	ClockBuffer #(
		.TYPE("GLOBAL"),
		.CE("NO")
	) clk_buf_sfp_rx_clk (
		.clkin(sfp_rx_clk_raw),
		.ce(1'b1),
		.clkout(sfp_rx_clk)
		);

	wire		sfp_tx_clk_raw;
	wire		sfp_tx_clk;
	wire[1:0]	sfp_tx_header;
	wire[5:0]	sfp_tx_sequence;

	ClockBuffer #(
		.TYPE("GLOBAL"),
		.CE("NO")
	) clk_buf_sfp_tx_clk (
		.clkin(sfp_tx_clk_raw),
		.ce(1'b1),
		.clkout(sfp_tx_clk)
		);

	gtwizard_10gbe xg_transceiver(

		.sysclk_in(clk_125mhz),
		.soft_reset_tx_in(1'b0),
		.soft_reset_rx_in(1'b0),
		.dont_reset_on_data_error_in(1'b1),
		.gt0_tx_fsm_reset_done_out(),
		.gt0_rx_fsm_reset_done_out(),
		.gt0_data_valid_in(1'b1),

		//QPLL clocks
		.gt0_qplllock_in(qpll_lock),
		.gt0_qpllrefclklost_in(qpll_refclk_lost),
		.gt0_qpllreset_out(),
		.gt0_qplloutclk_in(qpll_clk),
		.gt0_qplloutrefclk_in(qpll_refclk),

		//DRP - not used, tie off
		.gt0_drpaddr_in(9'b0),
		.gt0_drpclk_in(clk_125mhz),
		.gt0_drpdi_in(16'b0),
		.gt0_drpdo_out(),
		.gt0_drpen_in(1'b0),
		.gt0_drprdy_out(),
		.gt0_drpwe_in(1'b0),

		//Tie off other miscellaneous ports for unused interfaces
		.gt0_dmonitorout_out(),
		.gt0_eyescanreset_in(1'b0),
		.gt0_eyescandataerror_out(),
		.gt0_eyescantrigger_in(1'b0),
		.gt0_rxdfelpmreset_in(1'b0),
		.gt0_rxmonitorout_out(),
		.gt0_rxmonitorsel_in(2'b0),

		//Fabric RX interface
		.gt0_rxusrclk_in(sfp_rx_clk),
		.gt0_rxusrclk2_in(sfp_rx_clk),
		.gt0_rxdata_out(sfp_rx_data),
		.gt0_rxoutclk_out(sfp_rx_clk_raw),
		.gt0_rxoutclkfabric_out(),
		.gt0_rxdatavalid_out(sfp_rx_data_valid),
		.gt0_rxheader_out(sfp_rx_header),
		.gt0_rxheadervalid_out(sfp_rx_header_valid),
		.gt0_rxgearboxslip_in(sfp_rx_bitslip),

		//Reset controls
		.gt0_gtrxreset_in(1'b0),
		.gt0_rxpmareset_in(1'b0),
		.gt0_rxresetdone_out(),
		.gt0_gttxreset_in(1'b0),
		.gt0_rxuserrdy_in(1'b1),
		.gt0_txuserrdy_in(1'b1),
		.gt0_txresetdone_out(),

		//TX driver control
		.gt0_txpostcursor_in(5'b0),
		.gt0_txprecursor_in(5'b0),
		.gt0_txmaincursor_in(6'b0),
		.gt0_txdiffctrl_in(4'b0100),	//543 mV ppd

		//Fabric TX interface
		.gt0_txusrclk_in(sfp_tx_clk),
		.gt0_txusrclk2_in(sfp_tx_clk),
		.gt0_txdata_in(sfp_tx_data),
		.gt0_txoutclk_out(sfp_tx_clk_raw),
		.gt0_txoutclkfabric_out(),
		.gt0_txoutclkpcs_out(),
		.gt0_txheader_in(sfp_tx_header),
		.gt0_txsequence_in({1'b0, sfp_tx_sequence}),

		//Top level I/Os
		.gt0_gtxrxp_in(sfp_rx_p),
		.gt0_gtxrxn_in(sfp_rx_n),
		.gt0_gtxtxp_out(sfp_tx_p),
		.gt0_gtxtxn_out(sfp_tx_n)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// 10Gbase-R PCS

	XgmiiBus	xgmii_rx_bus;
	XgmiiBus	xgmii_tx_bus;

	wire		block_sync_good;
	wire		remote_fault;

	XGEthernetPCS pcs(
		.rx_clk(sfp_rx_clk),
		.tx_clk(sfp_tx_clk),

		.rx_header_valid(sfp_rx_header_valid),
		.rx_header(sfp_rx_header),
		.rx_data_valid(sfp_rx_data_valid),
		.rx_data(sfp_rx_data),
		.rx_bitslip(sfp_rx_bitslip),

		.tx_sequence(sfp_tx_sequence),
		.tx_header(sfp_tx_header),
		.tx_data(sfp_tx_data),

		.xgmii_rx_clk(xgmii_rx_clk),
		.xgmii_rx_bus(xgmii_rx_bus),

		.xgmii_tx_clk(xgmii_tx_clk),
		.xgmii_tx_bus(xgmii_tx_bus),

		.block_sync_good(block_sync_good),
		.link_up(link_up),
		.remote_fault(remote_fault)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// 10G Ethernet MAC

	XGEthernetMAC mac(
		.xgmii_rx_clk(xgmii_rx_clk),
		.xgmii_rx_bus(xgmii_rx_bus),

		.xgmii_tx_clk(xgmii_tx_clk),
		.xgmii_tx_bus(xgmii_tx_bus),

		.link_up(link_up),

		.rx_bus(mac_rx_bus),
		.tx_bus(mac_tx_bus)
	);

endmodule

