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

`include "EthernetBus.svh"
`include "GmiiBus.svh"
`include "IPv4Bus.svh"

/**
	@brief All of the Ethernet stuff
 */
module EthernetSubsystem(

	//Global clocks
	input wire			clk_125mhz,
	input wire			clk_250mhz,
	input wire			clk_ipstack,

	//Inputs from quad PLL
	input wire			qpll_clk,
	input wire			qpll_refclk,
	input wire			qpll_refclk_lost,
	input wire			qpll_lock,

	//SFP+ signals
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

	//RGMII signals
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
	output wire[3:0]	rgmii_txd
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// MAC address and IP configuration (TODO: make configurable)

	logic[47:0]		our_mac_addr				= 48'h02_ff_deadbeef;
	wire[47:0]		our_mac_addr_baser_txclk	= our_mac_addr;
	wire[47:0]		our_mac_addr_baset_txclk	= our_mac_addr;

	IPv4Config		ip_config;
	assign ip_config.address	= {  8'd10,   8'd2,   8'd6,  8'd11 };
	assign ip_config.mask		= { 8'd255, 8'd255, 8'd255,   8'd0 };
	assign ip_config.broadcast	= {  8'd10,   8'd2,   8'd6, 8'd255 };
	assign ip_config.gateway	= {  8'd10,   8'd2,   8'd6, 8'd252 };

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// 1G RGMII interface

	wire			baset_link_up;
	lspeed_t		baset_link_speed;

	wire			baset_mac_rx_clk;
	EthernetRxBus	baset_mac_rx_bus;

	wire			baset_mac_tx_clk;
	assign			baset_mac_tx_clk = clk_125mhz;
	EthernetTxBus	baset_mac_tx_bus;
	wire			baset_mac_tx_ready;

	RGMIIInterface rgmii(
		.clk_125mhz(clk_125mhz),
		.clk_250mhz(clk_250mhz),

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
		.rgmii_txd(rgmii_txd),

		.link_up(baset_link_up),
		.link_speed(baset_link_speed),

		.mac_rx_clk(baset_mac_rx_clk),
		.mac_rx_bus(baset_mac_rx_bus),

		.mac_tx_bus(baset_mac_tx_bus),
		.mac_tx_ready(baset_mac_tx_ready)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// 10G SFP+ interface

	wire			baser_link_up;
	wire			baser_mac_rx_clk;
	EthernetRxBus	baser_mac_rx_bus;
	wire			baser_mac_tx_clk;
	EthernetTxBus	baser_mac_tx_bus;

	SFPInterface sfp(
		.clk_125mhz(clk_125mhz),

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

		.link_up(baser_link_up),
		.xgmii_rx_clk(baser_mac_rx_clk),
		.mac_rx_bus(baser_mac_rx_bus),
		.xgmii_tx_clk(baser_mac_tx_clk),
		.mac_tx_bus(baser_mac_tx_bus)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Clock domain crossing from the two protocol blocks into the core TCP/IP stack clock domain

	EthernetRxBus	baset_cdc_rx_bus;
	EthernetRxBus	baser_cdc_rx_bus;

	EthernetRxClockCrossing baset_rx_cdc(
		.gmii_rxc(baset_mac_rx_clk),
		.mac_rx_bus(baset_mac_rx_bus),

		.sys_clk(clk_ipstack),
		.cdc_rx_bus(baset_cdc_rx_bus),

		.perf_rx_cdc_frames()
	);

	EthernetRxClockCrossing baser_rx_cdc(
		.gmii_rxc(baser_mac_rx_clk),
		.mac_rx_bus(baser_mac_rx_bus),

		.sys_clk(clk_ipstack),
		.cdc_rx_bus(baser_cdc_rx_bus),

		.perf_rx_cdc_frames()
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Mux to select which interface to use

	//Synchronize link-up flags into the IP stack clock domain
	wire	baset_link_up_ipstack;
	wire	baser_link_up_ipstack;
	wire	link_up_ipstack;
	ThreeStageSynchronizer sync_baset_link_up(
		.clk_in(baset_mac_rx_clk),
		.din(baset_link_up),
		.clk_out(clk_ipstack),
		.dout(baset_link_up_ipstack));
	ThreeStageSynchronizer sync_baser_link_up(
		.clk_in(baser_mac_rx_clk),
		.din(baser_link_up),
		.clk_out(clk_ipstack),
		.dout(baser_link_up_ipstack));

	//Check if either link is up
	assign link_up_ipstack = (baset_link_up_ipstack || baser_link_up_ipstack);

	//For now, we do not support multihoming. If the SFP+ link is up, it will be used exclusively and traffic
	//on the baseT link is ignored. If the SFP+ link is down, the baseT link is used exclusively.
	EthernetRxBus	muxed_rx_bus	= 0;
	always_ff @(posedge clk_ipstack) begin

		if(baser_link_up_ipstack)
			muxed_rx_bus	<= baser_cdc_rx_bus;
		else if(baset_link_up_ipstack)
			muxed_rx_bus	<= baset_cdc_rx_bus;
		else
			muxed_rx_bus	<= 0;

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Ethernet protocol decoder

	EthernetRxL2Bus	rx_l2_bus;

	Ethernet2TypeDecoder rx_eth_decoder(
		.rx_clk(clk_ipstack),

		.our_mac_address(our_mac_addr),
		.promisc_mode(1'b0),

		.mac_rx_bus(muxed_rx_bus),
		.rx_l2_bus(rx_l2_bus),

		.perf()
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Transmit CDC for both stacks

	EthernetTxL2Bus	tx_l2_bus;
	EthernetTxL2Bus	tx_baser_l2_bus;
	EthernetTxL2Bus	tx_baset_l2_bus;

	EthernetTransmitElasticBuffer #(
		.LINK_SPEED_IS_10G(1'b1),
		.HEADER_DEPTH(16),
		.PACKET_DEPTH(2048)
	) tx_baser_buf (
		.our_mac_address(our_mac_addr_baser_txclk),

		.tx_l2_clk(clk_ipstack),
		.tx_l2_bus(tx_baser_l2_bus),

		.mac_tx_clk(baser_mac_tx_clk),
		.mac_tx_ready(1'b1),
		.mac_tx_bus(baser_mac_tx_bus)
	);

	EthernetTransmitElasticBuffer #(
		.LINK_SPEED_IS_10G(1'b0),
		.HEADER_DEPTH(32),
		.PACKET_DEPTH(4096)
	) tx_baset_buf (
		.our_mac_address(our_mac_addr_baset_txclk),

		.tx_l2_clk(clk_ipstack),
		.tx_l2_bus(tx_baset_l2_bus),

		.mac_tx_clk(baset_mac_tx_clk),
		.mac_tx_ready(baset_mac_tx_ready),
		.mac_tx_bus(baset_mac_tx_bus)
	);

	//Muxing between the two TX paths: always write to both buffers but gate push flags
	always_comb begin

		tx_baser_l2_bus = tx_l2_bus;
		tx_baset_l2_bus = tx_l2_bus;

		if(!baser_link_up_ipstack) begin
			tx_baser_l2_bus.start		= 0;
			tx_baser_l2_bus.data_valid	= 0;
			tx_baser_l2_bus.commit		= 0;
		end

		if(!baset_link_up_ipstack) begin
			tx_baset_l2_bus.start		= 0;
			tx_baset_l2_bus.data_valid	= 0;
			tx_baset_l2_bus.commit		= 0;
		end

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Transmit bus arbitration

	EthernetTxL2Bus	arp_tx_l2_bus;
	EthernetTxL2Bus	ipv4_tx_arp_bus;

	EthernetTransmitArbiter tx_arbiter(
		.clk(clk_ipstack),

		.ipv4_tx_l2_bus(ipv4_tx_arp_bus),
		.arp_tx_l2_bus(arp_tx_l2_bus),

		.tx_l2_bus(tx_l2_bus),

		.perf()
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// ARP

	wire		arp_learn_valid;
	wire[31:0]	arp_learn_ip;
	wire[47:0]	arp_learn_mac;

	wire		arp_query_en;
	wire[31:0]	arp_query_ip;

	ARPProtocol arp(
		.clk(clk_ipstack),

		.our_mac_address(our_mac_addr),
		.our_ip_address(ip_config.address),

		.rx_l2_bus(rx_l2_bus),
		.tx_l2_bus(arp_tx_l2_bus),

		.learn_valid(arp_learn_valid),
		.learn_ip(arp_learn_ip),
		.learn_mac(arp_learn_mac),

		.query_en(arp_query_en),
		.query_ip(arp_query_ip)
	);

	EthernetTxArpBus	ipv4_tx_l2_bus;

	ARPManager #(
		.AGE_INTERVAL(300000000),
		.NUM_WAYS(2),
		.LINES_PER_WAY(128),
		.MAX_AGE(3600)
	) arp_mgr (
		.clk(clk_ipstack),

		.link_up(link_up_ipstack),
		.config_update(1'b0),
		.ip_config(ip_config),

		.ipv4_tx_l2_bus(ipv4_tx_l2_bus),
		.ipv4_tx_arp_bus(ipv4_tx_arp_bus),

		.learn_en(arp_learn_valid),
		.learn_ip(arp_learn_ip),
		.learn_mac(arp_learn_mac),

		.query_en(arp_query_en),
		.query_ip(arp_query_ip)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Debug ILA

	ila_1 ila(
		.clk(clk_ipstack),
		.probe0(rx_l2_bus),
		.probe1(arp_learn_valid),
		.probe2(arp_learn_ip),
		.probe3(arp_learn_mac)
	);

endmodule
