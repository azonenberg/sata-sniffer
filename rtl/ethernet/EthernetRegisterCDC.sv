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

`include "MicrocontrollerInterface.svh"

/**
	@brief Clock domain crossing for Ethernet SFRs from management domain to the respective destination domains
 */
module EthernetRegisterCDC(
	input wire				clk_ipstack,
	input wire				clk_250mhz,
	input wire				baser_mac_tx_clk,
	input wire				baset_mac_tx_clk,

	input wire cfgregs_t	cfgregs,

	output wire[47:0]		our_mac_addr_baser_txclk,
	output wire[47:0]		our_mac_addr_baset_txclk,
	output wire[47:0]		our_mac_addr_clk_ipstack,

	output IPv4Config		ip_config_clk_ipstack
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// MAC address

	RegisterSynchronizer #(
		.WIDTH(48)
	) sync_our_mac_addr_baser_txclk(
		.clk_a(clk_250mhz),
		.en_a(cfgregs.mac_address_updated),
		.ack_a(),
		.reg_a(cfgregs.mac_address),

		.clk_b(baser_mac_tx_clk),
		.updated_b(),
		.reset_b(1'b0),
		.reg_b(our_mac_addr_baser_txclk)
	);

	RegisterSynchronizer #(
		.WIDTH(48)
	) sync_our_mac_addr_baset_txclk(
		.clk_a(clk_250mhz),
		.en_a(cfgregs.mac_address_updated),
		.ack_a(),
		.reg_a(cfgregs.mac_address),

		.clk_b(baset_mac_tx_clk),
		.updated_b(),
		.reset_b(1'b0),
		.reg_b(our_mac_addr_baset_txclk)
	);

	RegisterSynchronizer #(
		.WIDTH(48)
	) sync_our_mac_addr_clk_ipstack(
		.clk_a(clk_250mhz),
		.en_a(cfgregs.mac_address_updated),
		.ack_a(),
		.reg_a(cfgregs.mac_address),

		.clk_b(clk_ipstack),
		.updated_b(),
		.reset_b(1'b0),
		.reg_b(our_mac_addr_clk_ipstack)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// IP address

	RegisterSynchronizer #(
		.WIDTH($bits(IPv4Config))
	) sync_ip_config_clk_ipstack(
		.clk_a(clk_250mhz),
		.en_a(cfgregs.ip_config_updated),
		.ack_a(),
		.reg_a(cfgregs.ip_config),

		.clk_b(clk_ipstack),
		.updated_b(),
		.reset_b(1'b0),
		.reg_b(ip_config_clk_ipstack)
	);

endmodule
