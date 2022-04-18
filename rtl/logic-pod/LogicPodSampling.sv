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
	// Register inputs in the fast clock domain

	(* RLOC = "X1Y0" *)
	logic[3:0]	deser_p_ff;

	(* RLOC = "X1Y1" *)
	logic[3:0]	deser_p_ff2;

	(* RLOC = "X1Y2" *)
	logic[3:0]	deser_n_ff;

	(* RLOC = "X1Y3" *)
	logic[3:0]	deser_n_ff2;

	logic		toggle = 0;

	always_ff @(posedge clk_625mhz_fabric) begin
		deser_p_ff <= deser_p;
		deser_n_ff <= deser_n;

		deser_p_ff2	<= deser_p_ff;
		deser_n_ff2	<= deser_n_ff;

		toggle		<= !toggle;
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Write registered values to a tiny FIFO made from block RAM

	//Do not synchronize pointers!
	//We just read garbage for the first few clocks after reset, but that's OK in this application.

	//Replicate write pointers so there's one copy in each slice right next to the RAM.

	(* RLOC = "X0Y0" *)
	(* DONT_TOUCH *)
	logic[3:0] wr_ptr_p_lo = 5'h0;

	(* RLOC = "X0Y1" *)
	(* DONT_TOUCH *)
	logic[3:0] wr_ptr_p_hi = 5'h0;

	(* RLOC = "X0Y2" *)
	(* DONT_TOUCH *)
	logic[3:0] wr_ptr_n_lo = 5'h0;

	(* RLOC = "X0Y3" *)
	(* DONT_TOUCH *)
	logic[3:0] wr_ptr_n_hi = 5'h0;

	(* RAM_STYLE = "distributed" *)
	(* RLOC = "X0Y0" *)
	logic[3:0] fifo_mem_p_lo[15:0];

	(* RAM_STYLE = "distributed" *)
	(* RLOC = "X0Y1" *)
	logic[3:0] fifo_mem_p_hi[15:0];

	(* RAM_STYLE = "distributed" *)
	(* RLOC = "X0Y2" *)
	logic[3:0] fifo_mem_n_lo[15:0];

	(* RAM_STYLE = "distributed" *)
	(* RLOC = "X0Y3" *)
	logic[3:0] fifo_mem_n_hi[15:0];

	always_ff @(posedge clk_625mhz_fabric) begin
		if(toggle) begin
			wr_ptr_p_lo				<= wr_ptr_p_lo + 1;
			wr_ptr_n_lo				<= wr_ptr_n_lo + 1;
			wr_ptr_p_hi				<= wr_ptr_p_hi + 1;
			wr_ptr_n_hi				<= wr_ptr_n_hi + 1;

			fifo_mem_p_lo[wr_ptr_p_lo]	<= deser_p_ff;
			fifo_mem_p_hi[wr_ptr_p_hi]	<= deser_p_ff2;

			fifo_mem_n_lo[wr_ptr_n_lo]	<= deser_n_ff;
			fifo_mem_n_hi[wr_ptr_n_hi]	<= deser_n_ff2;

		end
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Read FIFO in the slow clock domain

	logic[3:0] rd_ptr 		= 5'h08;

	logic[7:0] p_sampled	= 0;
	logic[7:0] n_sampled	= 0;

	always_ff @(posedge clk_312p5mhz) begin

		for(integer i=0; i<8; i=i+1) begin
			p_sampled	<= { fifo_mem_p_hi[rd_ptr], fifo_mem_p_lo[rd_ptr] };
			n_sampled	<= { fifo_mem_n_hi[rd_ptr], fifo_mem_n_lo[rd_ptr] };
			rd_ptr		<= rd_ptr + 1;
		end

	end

	assign p_out = p_sampled;
	assign n_out = n_sampled;

endmodule
