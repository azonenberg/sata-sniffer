`timescale 1ns/1ps
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

module LogicAnalyzerReadback(

	//Clocks
	input wire			clk_ram_2x,

	//Top level control inputs
	input wire			rst,
	input wire			flush_done,

	//Pointer access to pods
	output logic		ptr_rd_en		= 0,
	output logic[3:0]	ptr_rd_addr		= 0,
	input wire[28:0]	ptr_rd_data,

	//Bus to memory arbiter
	input wire			ram_addr_rd_en,
	output wire[28:0]	ram_addr_rd_data,
	output wire[7:0]	ram_addr_rd_size,
	input wire			ram_rd_data_start,
	input wire			ram_rd_data_valid,
	input wire[127:0]	ram_rd_data
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// FIFO of read requests going out to DRAM

	logic		dram_rd_en		= 0;
	logic[28:0]	dram_rd_addr	= 0;

	SingleClockFifo #(
		.WIDTH(29),
		.DEPTH(128),
		.USE_BLOCK(1),
		.OUT_REG(2)
	) addr_fifo (

		.clk(clk_ram_2x),
		.full(),
		.overflow(),
		.reset(rst),
		.empty(),
		.underflow(),

		.wr(dram_rd_en),
		.din(dram_rd_addr),
		.wsize(),

		.rd(ram_addr_rd_en),
		.dout(ram_addr_rd_data),
		.rsize(ram_addr_rd_size)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Main readback state machine

	enum logic[3:0]
	{
		STATE_IDLE,
		STATE_READ_PTR,
		STATE_DRAM_WAIT
	} state = STATE_IDLE;

	//High 5 bits of the address: constant for the given channel
	logic[4:0] addr_high	= 0;

	//Low 22 bits of the address (except low 2): row address within channel
	logic[21:0] addr_low	= 0;

	always_ff @(posedge clk_ram_2x) begin

		ptr_rd_en	<= 0;
		dram_rd_en	<= 0;

		case(state)

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// IDLE - wait for the acquisition to complete

			STATE_IDLE: begin

				if(flush_done) begin
					ptr_rd_en	<= 1;
					ptr_rd_addr	<= 0;
					state		<= STATE_READ_PTR;
				end

			end	//end STATE_IDLE

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// READ_PTR - wait for arbiter to give us the pointer

			STATE_READ_PTR: begin
				if(!ptr_rd_en) begin

					addr_high		<= ptr_rd_data[28:24];
					addr_low		<= ptr_rd_data[23:2];
					dram_rd_en		<= 1;
					dram_rd_addr	<= ptr_rd_data;
					state			<= STATE_DRAM_WAIT;

				end
			end	//end STATE_READ_PTR

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// DRAM_WAIT - wait for read to complete

			STATE_DRAM_WAIT: begin
			end	//end STATE_DRAM_WAIT

		endcase

		//Go back to idle from any state when we reset the trigger
		if(rst)
			state	<= STATE_IDLE;

	end

endmodule
