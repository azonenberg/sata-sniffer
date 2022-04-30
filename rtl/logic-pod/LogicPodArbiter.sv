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

module LogicPodArbiter #(
	parameter POD_NUMBER = 0
)(
	input wire			clk_ram_2x,

	output logic[7:0]	fifo_rd_en			= 0,
	input wire[127:0]	fifo_rd_data[7:0],
	input wire[9:0]		fifo_rd_size[7:0],
	input wire[7:0]		fifo_half_full,
	input wire[7:0]		fifo_burst_ready,

	output logic		data_fifo_wr_en		= 0,
	output logic[127:0]	data_fifo_wr_data	= 0,

	output logic		addr_fifo_wr_en		= 0,
	output logic[28:0]	addr_fifo_wr_data	= 0,

	input wire[9:0]		data_fifo_wr_size,
	input wire[7:0]		addr_fifo_wr_size
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// First level of arbitration: 8 lanes down to 1

	/*
		RAM clock domain is 162.5 MHz, 256 bits native bus. Need to deliver bursts on two consecutive clocks.
		So final FIFO to RAM should be 256b wide

		Save mux area by doing 4-cycle 128-bit bursts at 325 MHz for this phase.
	 */

	//0		idle, nothing in flight
	//1-4 	writing data
	//5		waiting for memory
	logic[2:0]	write_phase		= 0;

	logic		write_start		= 0;
	logic[2:0]	write_channel	= 0;

	logic		can_start_write;

	always_comb begin

		fifo_rd_en	= 0;

		//Read first word as soon as we have something to send
		if(write_start)
			fifo_rd_en[write_channel]	= 1;
		if( (write_phase > 0) && (write_phase < 4) )
			fifo_rd_en[write_channel]	= 1;

		//We can start a write if we're idle, or if on the last cycle of the previous one
		//(and not starting a new write)
		can_start_write = ((write_phase == 0) || (write_phase >= 3)) && !write_start;

	end

	logic	hit = 0;

	always_ff @(posedge clk_ram_2x) begin

		hit = 0;

		//Think about starting a write if we're in the right phase, and have space
		//(need 4 free slots for this burst, plus one more if a write is currently active)
		if( can_start_write && (data_fifo_wr_size > 4) && (addr_fifo_wr_size > 1) ) begin

			//Pass 1: read from highest numbered channel that's more than half full in the big fifo
			for(integer i=0; i<8; i=i+1) begin
				if(fifo_half_full[i]) begin
					hit	= 1;
					write_channel = i;
				end
			end

			//Pass 2: read from highest numbered channel with any data (note that a full burst is required)
			if(!hit) begin
				for(integer i=0; i<8; i=i+1) begin
					if(fifo_burst_ready[i]) begin
						hit	= 1;
						write_channel = i;
					end
				end
			end

		end

		write_start	<= hit;

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Pop the channel FIFOs into the output FIFO

	//Each LA channel has 0x200000 (2^22) 256-bit storage locations
	(* RAM_STYLE = "distributed" *)
	logic[21:0] dram_wr_ptr[7:0];
	initial begin
		for(integer i=0; i<8; i=i+1)
			dram_wr_ptr[i] <= 0;
	end

	logic			fifo_rd_valid		= 0;
	logic[2:0]		write_channel_ff	= 0;

	logic			data_fifo_wr_en_adv	= 0;

	always_ff @(posedge clk_ram_2x) begin

		//Pipeline write data (fifo read output has extra pipeline stage to improve timing)
		fifo_rd_valid			<= fifo_rd_en[write_channel];
		data_fifo_wr_en_adv		<= fifo_rd_valid;
		write_channel_ff		<= write_channel;

		//Mux write output (second stage)
		data_fifo_wr_en			<= data_fifo_wr_en_adv;
		data_fifo_wr_data		<= fifo_rd_data[write_channel_ff];

		//Start a new write cycle
		addr_fifo_wr_en			<= 0;
		if(write_start) begin
			write_phase			<= 1;

			//address fifo todo
			addr_fifo_wr_en		<= 1;
			addr_fifo_wr_data	<=
			{
				1'b1,
				POD_NUMBER[0],
				write_channel,
				dram_wr_ptr[write_channel],
				2'b0
			};

			//Bump pointer
			dram_wr_ptr[write_channel]	<= dram_wr_ptr[write_channel] + 1;

		end

		//Continue existing write cycle
		else begin
			if(write_phase == 4)
				write_phase <= 0;
			else if(write_phase > 0)
				write_phase	<= write_phase + 1;
		end

	end

endmodule
