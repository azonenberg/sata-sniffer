/***********************************************************************************************************************
*                                                                                                                      *
* sata-sniffer v0.1                                                                                                    *
*                                                                                                                      *
* Copyright (c) 2021-2021 Andrew D. Zonenberg and contributors                                                         *
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

	//SATA

	//Extra GTX

	//10Gbase-R
	output wire			sfp_scl,
	inout wire			sfp_sda,
	input wire			sfp_tx_fault,
	output wire			sfp_tx_disable,
	input wire			sfp_mod_abs,
	input wire			sfp_rx_los,
	output wire[1:0]	sfp_rs,

	//RGMII
	output wire			eth_mdc,
	inout wire			eth_mdio,
	output wire			eth_rst_n,
	input wire			rgmii_rxc,
	input wire			rgmii_rx_dv,
	input wire[3:0]		rgmii_rxd,
	input wire[1:0]		eth_led_n_1v8,
	output wire			rgmii_txc,
	output wire			rgmii_tx_en,
	output wire[3:0]	rgmii_txd,

	//PMOD
	inout wire[7:0]		pmod_dq,

	//Logic analyzer pods
	input wire[7:0]		la0_p,
	input wire[7:0]		la0_n,
	input wire			la0_present,
	input wire			la0_12v_fault,
	output wire			la0_12v_en,
	input wire			la0_uart_rx,
	output wire			la0_uart_tx,

	input wire[7:0]		la1_p,
	input wire[7:0]		la1_n,
	input wire			la1_present,
	input wire			la1_12v_fault,
	output wire			la1_12v_en,
	input wire			la1_uart_rx,
	output wire			la1_uart_tx,

	//GPIO LEDs
	output wire[3:0]	gpio_led,

	//LVDS GPIO
	input wire[15:0] 	lvds_gpio_p,
	input wire[15:0]	lvds_gpio_n,

	//DDR3
	output wire			ram_scl,
	inout wire			ram_sda
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
	// Clock synthesis

	wire	clk_125mhz;
	wire	clk_250mhz;
	wire	clk_625mhz_0;
	wire	clk_625mhz_90;

	ClockGeneration clockgen(
		.clk_125mhz_p(clk_125mhz_p),
		.clk_125mhz_n(clk_125mhz_n),

		.clk_125mhz(clk_125mhz),
		.clk_250mhz(clk_250mhz),
		.clk_625mhz_0(clk_625mhz_0),
		.clk_625mhz_90(clk_625mhz_90)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Logic analyzer pods

	`include "LogicPod.svh"
	la_sample_t[7:0]	la0_samples;
	la_sample_t[7:0]	la1_samples;

	LogicPodDatapath #(.LANE_INVERT(8'b10011100)) la0_path (
		.clk_250mhz(clk_250mhz),
		.clk_625mhz_0(clk_625mhz_0),
		.clk_625mhz_90(clk_625mhz_90),
		.pod_data_p(la0_p),
		.pod_data_n(la0_n),
		.samples(la0_samples));

	LogicPodDatapath #(.LANE_INVERT(8'b00000110)) la1_path (
		.clk_250mhz(clk_250mhz),
		.clk_625mhz_0(clk_625mhz_0),
		.clk_625mhz_90(clk_625mhz_90),
		.pod_data_p(la1_p),
		.pod_data_n(la1_n),
		.samples(la1_samples));

endmodule
