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

`include "GmiiBus.svh"
`include "EthernetBus.svh"

/**
	@brief RGMII MAC
 */
module RGMIIInterface(

	//Clocks
	input wire					clk_125mhz,
	input wire					clk_250mhz,

	//RGMII signals
	output wire					eth_mdc,
	inout wire					eth_mdio,
	output logic				eth_rst_n	= 0,
	input wire[1:0]				eth_led_n_1v8,
	output wire[1:0]			eth_led_p_3v3,
	input wire					rgmii_rxc,
	input wire					rgmii_rx_dv,
	input wire[3:0]				rgmii_rxd,
	output wire					rgmii_txc,
	output wire					rgmii_tx_en,
	output wire[3:0]			rgmii_txd,

	//Layer 2 bus and status signals to Ethernet subsystem
	output wire					link_up,
	output lspeed_t				link_speed,

	output wire					mac_rx_clk,
	output EthernetRxBus		mac_rx_bus,

	input wire EthernetTxBus	mac_tx_bus,
	output wire 				mac_tx_ready
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// The MAC

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
		.link_up(link_up),
		.link_speed(link_speed)
	);


	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Reset and support glue

	assign eth_led_p_3v3 = ~eth_led_n_1v8;

	//Bring up the PHY after a little while
	logic[15:0] eth_rst_count = 1;
	always_ff @(posedge clk_125mhz) begin
		if(eth_rst_count == 0)
			eth_rst_n		<= 1;
		else
			eth_rst_count	<= eth_rst_count + 1'h1;
	end

endmodule
