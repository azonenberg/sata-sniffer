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

module LogicPodSamplingFifo(
	input wire			fast_clk,
	input wire			slow_clk,

	input wire[3:0]		fast_data,
	output wire[3:0]	fast_data_ff,	//output so we can use for shift reg too
	output wire[3:0]	slow_data
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Register inputs in the fast clock domain

	//Inputs and write pointer live one slice left of the RAM / address slice
	(* RLOC = "X1Y0" *)
	(* DONT_TOUCH *)
	logic[3:0]	fast_data_ff_int	= 0;

	assign fast_data_ff = fast_data_ff_int;

	//Toggle has to go off to the right, cannot fit in same slice as memory
	(* RLOC = "X3Y0" *)
	(* DONT_TOUCH *)
	logic		toggle = 0;

	always_ff @(posedge fast_clk) begin
		toggle		<= !toggle;

		if(toggle)
			fast_data_ff_int	<= fast_data;
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Write registered values to a tiny FIFO made from LUT RAM

	//Do not synchronize pointers!
	//We just read garbage for the first few clocks after reset, but that's OK in this application.

	//Replicate write pointers so there's one copy in each slice right next to the RAM.
	//Write pointer and FIFO memory live in the same slice

	(* RLOC = "X1Y0" *)
	(* DONT_TOUCH *)
	logic[3:0] wr_ptr = 4'h0;

	//Half the speed of the write side, less placement critical
	//Cannot live in any of the other slices due to control set conflicts
	//but can be packed in with the read side data registers
	(* RLOC = "X0Y0" *)
	logic[3:0] rd_ptr 		= 5'h08;

	//Need to use primitive instantiation here
	//because RLOC doesn't seem to work on inferred memory
	wire[3:0]	rd_data;
	(* RLOC = "X2Y0" *)
	RAM32M fifomem(

		//Write side
		.DIA(fast_data_ff[3:2]),
		.DIB(fast_data_ff[1:0]),
		.DIC(2'b0),
		.DID(2'b0),
		.ADDRD({1'b0, wr_ptr}),
		.WE(toggle),
		.WCLK(fast_clk),

		//Read side
		.ADDRA({1'b0, rd_ptr}),
		.ADDRB({1'b0, rd_ptr}),
		.DOA(rd_data[3:2]),
		.DOB(rd_data[1:0]),

		//other half of bus not used
		.ADDRC(5'h0),
		.DOC(),
		.DOD()
	);

	always_ff @(posedge fast_clk) begin
		if(toggle)
			wr_ptr				<= wr_ptr + 1;
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Read size (8 clocks out of phase)

	(* RLOC = "X0Y0" *)
	logic[3:0] slow_data_ff	= 0;

	always_ff @(posedge slow_clk) begin

		for(integer i=0; i<8; i=i+1) begin
			slow_data_ff	<= rd_data;
			rd_ptr			<= rd_ptr + 1;
		end

	end

	assign slow_data = slow_data_ff;

endmodule

/**
	@brief RPM for getting data from 625 MHz clock domain into 312.5 MHz domain
 */
module LogicPodSampling(
	input wire			clk_625mhz_fabric,
	input wire			clk_312p5mhz,

	input wire[3:0]		deser_p,
	input wire[3:0]		deser_n,

	output wire[7:0]	p_out,
	output wire[7:0]	n_out
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// FIFO instances

	wire[3:0]	deser_p_ff;
	LogicPodSamplingFifo fifo_p_lo(
		.fast_clk(clk_625mhz_fabric),
		.slow_clk(clk_312p5mhz),
		.fast_data(deser_p_ff),
		.fast_data_ff(),
		.slow_data(p_out[3:0]));

	LogicPodSamplingFifo fifo_p_hi(
		.fast_clk(clk_625mhz_fabric),
		.slow_clk(clk_312p5mhz),
		.fast_data(deser_p),
		.fast_data_ff(deser_p_ff),
		.slow_data(p_out[7:4]));

	wire[3:0]	deser_n_ff;
	wire[7:0]	n_sampled;
	LogicPodSamplingFifo fifo_n_lo(
		.fast_clk(clk_625mhz_fabric),
		.slow_clk(clk_312p5mhz),
		.fast_data(deser_n_ff),
		.fast_data_ff(),
		.slow_data(n_out[3:0]));

	LogicPodSamplingFifo fifo_n_hi(
		.fast_clk(clk_625mhz_fabric),
		.slow_clk(clk_312p5mhz),
		.fast_data(deser_n),
		.fast_data_ff(deser_n_ff),
		.slow_data(n_out[7:4]));

endmodule
