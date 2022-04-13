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
	@brief Generate clocks for use by the design

	All clocks are synthesized from the 125 MHz input except for the 200 MHz, which goes straight to the RAM controller.
 */
module ClockGeneration(

	//Top level clock input pins
	input wire			clk_125mhz_p,
	input wire			clk_125mhz_n,

	input wire			clk_200mhz_p,
	input wire			clk_200mhz_n,

	//Global clock outputs
	output wire			clk_125mhz,
	output wire			clk_200mhz,
	output wire			clk_250mhz,
	output wire 		clk_400mhz,
	output wire[1:0]	pll_lock
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Input buffers

	wire	clk_125mhz_in;
	wire	clk_250mhz_in;

	DifferentialInputBuffer #(
		.WIDTH(1),
		.ODT(1)
	) clk_125_ibuf (
		.pad_in_p(clk_125mhz_p),
		.pad_in_n(clk_125mhz_n),
		.fabric_out(clk_125mhz_in)
	);

	wire clk_200mhz_raw;
	DifferentialInputBuffer #(
		.WIDTH(1),
		.ODT(0)			//In a 1.35V VCCO bank, cannot use DIFF_TERM
	) clk_200_ibuf (
		.pad_in_p(clk_200mhz_p),
		.pad_in_n(clk_200mhz_n),
		.fabric_out(clk_200mhz_raw)
	);

	ClockBuffer #(
		.TYPE("GLOBAL"),
		.CE("NO")
	) clk_buf_200(
		.clkin(clk_200mhz_raw),
		.ce(1'b1),
		.clkout(clk_200mhz)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Main system clock synthesis PLL (125 MHz input)

	wire[3:0]	clk_unused1;

	ReconfigurablePLL #(
		.IN0_PERIOD(8),			//125 MHz input
		.IN1_PERIOD(8),

		.OUTPUT_BUF_GLOBAL(	6'b000011),
		.OUTPUT_BUF_IO(		6'b000000),
		.OUTPUT_GATE(		6'b000011),
		.FINE_PHASE_SHIFT(6'b000000),

		.OUT0_MIN_PERIOD(8),	//125 MHz output
		.OUT1_MIN_PERIOD(4),	//250 MHz output
		.OUT2_MIN_PERIOD(8),	//125 MHz output (unused)
		.OUT3_MIN_PERIOD(8),	//125 MHz output (unused)
		.OUT4_MIN_PERIOD(8),	//125 MHz output (unused)
		.OUT5_MIN_PERIOD(8),	//125 MHz output (unused)

		.OUT0_DEFAULT_PHASE(0),
		.OUT1_DEFAULT_PHASE(0),
		.OUT2_DEFAULT_PHASE(0),
		.OUT3_DEFAULT_PHASE(0),
		.OUT4_DEFAULT_PHASE(0),
		.OUT5_DEFAULT_PHASE(0),

		.ACTIVE_ON_START(1)		//Start PLL on power up
	) pll_125 (
		.clkin({clk_125mhz_in, clk_125mhz_in}),
		.clksel(1'b0),

		.clkout({clk_unused1, clk_250mhz, clk_125mhz}),

		.reset(1'b0),
		.locked(pll_lock[0]),

		.busy(),
		.reconfig_clk(clk_125mhz_in),
		.reconfig_start(1'b0),
		.reconfig_finish(1'b0),
		.reconfig_cmd_done(),

		.reconfig_vco_en(1'b0),
		.reconfig_vco_mult(7'b0),
		.reconfig_vco_indiv(7'b0),
		.reconfig_vco_bandwidth(1'b0),

		.reconfig_output_en(1'b0),
		.reconfig_output_idx(3'b0),
		.reconfig_output_div(8'b0),
		.reconfig_output_phase(9'b0),

		.phase_shift_clk(1'b0),
		.phase_shift_en(1'b0),
		.phase_shift_inc(1'b0),
		.phase_shift_done()
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Secondary PLL (200 MHz input)

	wire[4:0] clk_unused2;

	ReconfigurablePLL #(
		.IN0_PERIOD(5),			//200 MHz input
		.IN1_PERIOD(5),

		.OUT0_MIN_PERIOD(2.5),	//400 MHz output
		.OUT1_MIN_PERIOD(5),	//200 MHz output (unused)
		.OUT2_MIN_PERIOD(5),	//200 MHz output (unused)
		.OUT3_MIN_PERIOD(5),	//200 MHz output (unused)
		.OUT4_MIN_PERIOD(5),	//200 MHz output (unused)
		.OUT5_MIN_PERIOD(5),	//200 MHz output (unused)

		.OUT0_DEFAULT_PHASE(0),
		.OUT1_DEFAULT_PHASE(0),
		.OUT2_DEFAULT_PHASE(0),
		.OUT3_DEFAULT_PHASE(0),
		.OUT4_DEFAULT_PHASE(0),
		.OUT5_DEFAULT_PHASE(0),

		.ACTIVE_ON_START(1)		//Start PLL on power up
	) pll_200 (
		.clkin({clk_200mhz, clk_200mhz}),
		.clksel(1'b0),

		.clkout({clk_unused2, clk_400mhz}),

		.reset(1'b0),
		.locked(pll_lock[1]),

		.busy(),
		.reconfig_clk(clk_125mhz_in),
		.reconfig_start(1'b0),
		.reconfig_finish(1'b0),
		.reconfig_cmd_done(),

		.reconfig_vco_en(1'b0),
		.reconfig_vco_mult(7'b0),
		.reconfig_vco_indiv(7'b0),
		.reconfig_vco_bandwidth(1'b0),

		.reconfig_output_en(1'b0),
		.reconfig_output_idx(3'b0),
		.reconfig_output_div(8'b0),
		.reconfig_output_phase(9'b0),

		.phase_shift_clk(1'b0),
		.phase_shift_en(1'b0),
		.phase_shift_inc(1'b0),
		.phase_shift_done()
	);

endmodule
