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
	@brief GTX on SMA expansion port
 */
module ExpansionTransceiver(
	input wire	clk_125mhz,

	output wire		sma_tx_p,
	output wire		sma_tx_n,

	input wire		qpll_clk,
	input wire		qpll_refclk,
	input wire		qpll_refclk_lost,
	input wire		qpll_lock,

	input wire[1:0]	sys_pll_lock
);

	wire	prbs_tx_clk_raw;
	wire	prbs_tx_clk;

	ClockBuffer #(
		.TYPE("LOCAL"),
		.CE("NO")
	) clk_buf_prbs_tx_clk (
		.clkin(prbs_tx_clk_raw),
		.ce(1'b1),
		.clkout(prbs_tx_clk)
		);

	//TODO: investigate all these other siganls and try to figure out what's not working...
	wire		tx_reset_done;

	gtwizard_sma prbs_transceiver(
		.sysclk_in(clk_125mhz),

		//TODO: what are these for?
		.soft_reset_tx_in(1'b0),
		.dont_reset_on_data_error_in(1'b0),
		.gt0_tx_fsm_reset_done_out(),
		.gt0_rx_fsm_reset_done_out(),

		//Tie off unused ports
		.gt0_drpaddr_in(9'b0),
		.gt0_drpclk_in(clk_125mhz),
		.gt0_drpdi_in(16'b0),
		.gt0_drpdo_out(),
		.gt0_drpen_in(1'b0),
		.gt0_drprdy_out(),
		.gt0_drpwe_in(1'b0),
		.gt0_dmonitorout_out(),
		.gt0_eyescanreset_in(1'b0),
		.gt0_eyescandataerror_out(),
		.gt0_eyescantrigger_in(1'b0),
		.gt0_rxphmonitor_out(),
		.gt0_rxphslipmonitor_out(),
		.gt0_rxmonitorout_out(),
		.gt0_rxmonitorsel_in(2'b0),
		.gt0_gtrxreset_in(1'b0),
		.gt0_gttxreset_in(1'b0),

		//Transmit interface
		.gt0_txuserrdy_in(sys_pll_lock[0]),
		.gt0_txusrclk_in(prbs_tx_clk),
		.gt0_txusrclk2_in(prbs_tx_clk),
		.gt0_data_valid_in(1'b1),
		.gt0_txdata_in(32'h00000000),
		.gt0_txoutclk_out(prbs_tx_clk_raw),
		.gt0_txoutclkfabric_out(),
		.gt0_txoutclkpcs_out(),
		.gt0_txresetdone_out(),

		//Output pattern selection
		.gt0_txprbssel_in(3'b010),	//PRBS-15

		//Top level diff pairs
		.gt0_gtxtxn_out(sma_tx_n),
		.gt0_gtxtxp_out(sma_tx_p),

		//Output swing control
		.gt0_txdiffctrl_in(4'b0100),	//543 mV p-p differential

		//Clock from QPLL
		.gt0_qplllock_in(qpll_lock),
		.gt0_qpllrefclklost_in(qpll_refclk_lost),
		.gt0_qpllreset_out(),
		.gt0_qplloutclk_in(qpll_clk),
		.gt0_qplloutrefclk_in(qpll_refclk)
		);

endmodule
