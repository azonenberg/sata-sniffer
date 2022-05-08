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
	@brief Deserialize 17-bit data from the compressor into 128-bit data for the CDC FIFO

	Easy but inefficient: 7 blocks * 17 bits = 119 bits used per 128b word, 9 wasted (7% overhead)
	We can double this with some effort: 15 blocks per 2 words = 1 bit waste per 256 (0.39% overhead)
 */
module LogicPodDeserialization(
	input wire			clk,

	input wire			compress_out_valid,
	input wire			compress_out_format,
	input wire[15:0]	compress_out_data,

	input wire			flush,
	output logic		flush_done	= 0,

	output logic		fifo_wr		= 0,
	output logic[127:0]	fifo_wdata	= 0
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Deserialization

	logic[2:0]		deser_words	= 0;
	logic			flush_ff	= 0;

	//Pop the CDC FIFO into the shift register
	logic[2:0]	deser_words_fwd;
	always_comb begin
		deser_words_fwd		= deser_words + compress_out_valid;
		fifo_wr				= 0;

		//Done with block?
		if((deser_words_fwd == 7) || flush_ff) begin
			fifo_wr			= 1;
			deser_words_fwd	= 0;
		end

	end

	always_ff @(posedge clk) begin
		deser_words		<= deser_words_fwd;
		flush_ff		<= flush;
		flush_done		<= flush_ff;

		if(compress_out_valid)
			fifo_wdata	<= {8'd7, 1'b0, fifo_wdata[101:0], compress_out_format, compress_out_data};

		else if(flush)
			fifo_wdata[127:120]	<= {5'b0, deser_words};

	end

endmodule
