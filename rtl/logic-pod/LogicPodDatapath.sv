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

	//Main clock
	input wire				clk_312p5mhz,

	//Reference clock for IDELAYs
	input wire				clk_400mhz,

	//Oversampling clocks
	input wire				clk_625mhz_fabric,
	input wire				clk_625mhz_io_0,
	input wire				clk_625mhz_io_90,

	//LVDS input
	input wire[7:0]			pod_data_p,
	input wire[7:0]			pod_data_n
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Remap pod data lines to match front panel ordering

	wire[7:0]	pod_data_p_remapped;
	wire[7:0]	pod_data_n_remapped;

	assign pod_data_p_remapped[0] = pod_data_p[3];
	assign pod_data_n_remapped[0] = pod_data_n[3];

	assign pod_data_p_remapped[1] = pod_data_p[2];
	assign pod_data_n_remapped[1] = pod_data_n[2];

	assign pod_data_p_remapped[2] = pod_data_p[7];
	assign pod_data_n_remapped[2] = pod_data_n[7];

	assign pod_data_p_remapped[3] = pod_data_p[6];
	assign pod_data_n_remapped[3] = pod_data_n[6];

	assign pod_data_p_remapped[4] = pod_data_p[5];
	assign pod_data_n_remapped[4] = pod_data_n[5];

	assign pod_data_p_remapped[5] = pod_data_p[4];
	assign pod_data_n_remapped[5] = pod_data_n[4];

	assign pod_data_p_remapped[6] = pod_data_p[1];
	assign pod_data_n_remapped[6] = pod_data_n[1];

	assign pod_data_p_remapped[7] = pod_data_p[0];
	assign pod_data_n_remapped[7] = pod_data_n[0];

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
			.I(pod_data_p_remapped[g]),
			.IB(pod_data_n_remapped[g]),
			.O(data_p[g]),
			.OB(data_n[g])
		);

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Delay line calibration

	IODelayCalibration cal(.refclk(clk_400mhz));

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Input delay lines

	wire[7:0] data_p_delayed;
	wire[7:0] data_n_delayed;

	IODelayBlock #(
		.WIDTH(8),
		.CAL_FREQ(400),
		.INPUT_DELAY(00),
		.DIRECTION("IN"),
		.IS_CLOCK(0)
	) idelay_p (
		.i_pad(data_p),
		.i_fabric(data_p_delayed),

		.o_pad(),
		.o_fabric(),

		.input_en(1'b1)
	);

	IODelayBlock #(
		.WIDTH(8),
		.CAL_FREQ(400),
		.INPUT_DELAY(200),
		.DIRECTION("IN"),
		.IS_CLOCK(0)
	) idelay_n (
		.i_pad(data_n),
		.i_fabric(data_n_delayed),

		.o_pad(),
		.o_fabric(),

		.input_en(1'b1)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Initial oversampling: 4 phases of 625 MHz = 2.5 Gsps, times P and N = 5 Gsps

	wire[3:0] deser_p[7:0];
	wire[3:0] deser_n[7:0];

	logic	iserdes_rst = 0;


	//Sampling order is Q1 Q3 Q2 Q4 in oversampling mode
	//then we interleave with negative before positive
	for(genvar g=0; g<8; g=g+1) begin
		ISERDESE2 #(
			.DATA_RATE("DDR"),
			.DATA_WIDTH("4"),
			.DYN_CLKDIV_INV_EN("FALSE"),
			.DYN_CLK_INV_EN("FALSE"),
			.INTERFACE_TYPE("OVERSAMPLE"),
			.NUM_CE(1),
			.OFB_USED("FALSE"),
			.SERDES_MODE("MASTER"),
			.IOBDELAY("BOTH")
		) iserdes_p (
			.Q1(deser_p[g][0]),
			.Q2(deser_p[g][2]),
			.Q3(deser_p[g][1]),
			.Q4(deser_p[g][3]),
			.Q5(),
			.Q6(),
			.Q7(),
			.Q8(),
			.O(),
			.SHIFTOUT1(),
			.SHIFTOUT2(),
			.D(),
			.DDLY(data_p_delayed[g]),
			.CLK(clk_625mhz_io_0),
			.CLKB(!clk_625mhz_io_0),
			.CE1(1'b1),
			.CE2(1'b1),
			.RST(iserdes_rst),
			.CLKDIV(),
			.CLKDIVP(1'b0),
			.OCLK(clk_625mhz_io_90),
			.OCLKB(!clk_625mhz_io_90),
			.BITSLIP(1'b0),
			.SHIFTIN1(),
			.SHIFTIN2(),
			.OFB(),
			.DYNCLKDIVSEL(),
			.DYNCLKSEL()
		);

		ISERDESE2 #(
			.DATA_RATE("DDR"),
			.DATA_WIDTH("4"),
			.DYN_CLKDIV_INV_EN("FALSE"),
			.DYN_CLK_INV_EN("FALSE"),
			.INTERFACE_TYPE("OVERSAMPLE"),
			.NUM_CE(1),
			.OFB_USED("FALSE"),
			.SERDES_MODE("MASTER"),
			.IOBDELAY("BOTH")
		) iserdes_n (
			.Q1(deser_n[g][0]),
			.Q2(deser_n[g][2]),
			.Q3(deser_n[g][1]),
			.Q4(deser_n[g][3]),
			.Q5(),
			.Q6(),
			.Q7(),
			.Q8(),
			.O(),
			.SHIFTOUT1(),
			.SHIFTOUT2(),
			.D(),
			.DDLY(data_n_delayed[g]),
			.CLK(clk_625mhz_io_0),
			.CLKB(!clk_625mhz_io_0),
			.CE1(1'b1),
			.CE2(1'b1),
			.RST(iserdes_rst),
			.CLKDIV(),
			.CLKDIVP(1'b0),
			.OCLK(clk_625mhz_io_90),
			.OCLKB(!clk_625mhz_io_90),
			.BITSLIP(1'b0),
			.SHIFTIN1(),
			.SHIFTIN2(),
			.OFB(),
			.DYNCLKDIVSEL(),
			.DYNCLKSEL()
		);

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Capture into the slow clock domain

	wire[7:0]	p_merged[7:0];
	wire[7:0]	n_merged[7:0];

	for(genvar g=0; g<8; g=g+1) begin
		LogicPodSampling sampler (
			.clk_625mhz_fabric(clk_625mhz_fabric),
			.clk_312p5mhz(clk_312p5mhz),
			.deser_p(deser_p[g]),
			.deser_n(deser_n[g]),
			.p_out(p_merged[g]),
			.n_out(n_merged[g])
		);
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Merge P/N into a single 16-bit stream, inverting as necessary

	//Parallel digitized output
	la_sample_t[7:0]	samples;

	//N has a larger delay than P, so it's logically earlier in the stream
	always_ff @(posedge clk_312p5mhz) begin

		for(integer i=0; i<8; i=i+1) begin

			for(integer j=0; j<8; j=j+1) begin
				samples[i].bits[j*2 + 1]	<= !n_merged[i][j] ^ LANE_INVERT[i];
				samples[i].bits[j*2]		<= p_merged[i][j] ^ LANE_INVERT[i];
			end

		end

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Compression (per lane)

	wire[7:0] out_valid;
	wire[7:0] out_format;
	wire[15:0] out_data[7:0];

	for(genvar g=0; g<8; g=g+1) begin

		LogicPodCompression compressor(
			.clk(clk_312p5mhz),
			.din(samples[g].bits),

			.out_valid(out_valid[g]),
			.out_format(out_format[g]),
			.out_data(out_data[g])
		);

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// CDC FIFOs

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Arbitration and deserialization in DRAM clock domain

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Debug ILA

	ila_0 ila(
		.clk(clk_312p5mhz),
		.probe0(samples[0].bits),
		.probe1(samples[1].bits),
		.probe2(out_valid),
		.probe3(out_format),
		.probe4(out_data[0]),
		.probe5(out_data[1]),
		.probe6(out_data[2]),
		.probe7(out_data[3]),
		.probe8(out_data[4]),
		.probe9(out_data[5]),
		.probe10(out_data[6]),
		.probe11(out_data[7])
	);

endmodule
