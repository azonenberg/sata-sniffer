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

`include "LogicPod.svh"

/**
	@brief Logic analyzer pod datapath

	This module takes in the combinatorial LVDS output of a single 8-bit logic analyzer pod (MEAD, CONWAY, etc) and
	samples it at 5 Gsps.

	All clocks must come from the same PLL and be aligned.
 */
module LogicPodDatapath #(
	parameter LANE_INVERT = 8'b00000000
) (

	//Main clocks
	input wire				clk_250mhz,

	//Oversampling clocks
	input wire				clk_625mhz_0,
	input wire				clk_625mhz_90,

	//LVDS input
	input wire[7:0]			pod_data_p,
	input wire[7:0]			pod_data_n,

	//Parallel digitized output
	output la_sample_t[7:0]	samples
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// LVDS input buffers

	wire[7:0]	data_p;
	wire[7:0]	data_n;

	for(genvar g=0; g<8; g=g+1) begin

		IBUFDS_DIFF_OUT #(
			.DIFF_TERM("TRUE"),
			.IBUF_LOW_PWR("FALSE"),
			.IOSTANDARD("LVDS")
		) ibuf (
			.I(pod_data_p[g]),
			.IB(pod_data_n[g]),
			.O(data_p[g]),
			.OB(data_n[g])
		);

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Input deserializers

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Output registers and inversion

endmodule
