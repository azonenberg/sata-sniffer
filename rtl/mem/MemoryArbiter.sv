`timescale 1ns / 1ps
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

module MemoryArbiter(

	//Top level clock
	input wire			clk_ram,
	input wire			clk_ram_2x,

	//Bus to memory controller
	output logic[28:0]	app_addr		= 0,
	output logic[2:0]	app_cmd			= 0,
	output logic		app_en			= 0,

	output logic[255:0]	app_wdf_data	= 0,
	output logic		app_wdf_end		= 0,
	output logic[31:0]	app_wdf_mask	= 0,	//default to not masking anything
	output logic		app_wdf_wren	= 0,
	input wire			app_wdf_rdy,

	output logic		app_ref_req		= 0,
	output logic		app_sr_req		= 0,
	output logic		app_zq_req		= 0,

	input wire[255:0]	app_rd_data,
	input wire			app_rd_data_end,
	input wire			app_rd_data_valid,

	input wire			app_rdy,

	//Buses from client domains to arbiter
	//128 bits at 2x controller clock
	//TODO: we could simplify things a lot by running the entire controller at this rate??
	input wire			la0_wr_en,
	input wire			la0_wr_valid,
	input wire[28:0]	la0_wr_addr,
	input wire[127:0]	la0_wr_data,
	output logic		la0_wr_ack,

	input wire			la1_wr_en,
	input wire			la1_wr_valid,
	input wire[28:0]	la1_wr_addr,
	input wire[127:0]	la1_wr_data,
	output logic		la1_wr_ack
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Remap inputs to numbered vectors for easier handling

	localparam NUM_PORTS = 2;

	wire		wr_en[NUM_PORTS-1:0];
	wire		wr_valid[NUM_PORTS-1:0];
	wire[28:0]	wr_addr[NUM_PORTS-1:0];
	wire[127:0]	wr_data[NUM_PORTS-1:0];

	assign wr_en[0]		= la0_wr_en;
	assign wr_valid[0]	= la0_wr_valid;
	assign wr_addr[0]	= la0_wr_addr;
	assign wr_data[0]	= la0_wr_data;

	assign wr_en[1]		= la1_wr_en;
	assign wr_valid[1]	= la1_wr_valid;
	assign wr_addr[1]	= la1_wr_addr;
	assign wr_data[1]	= la1_wr_data;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Remap outputs

	logic[NUM_PORTS-1:0]	wr_ack	= 0;
	assign la0_wr_ack = wr_ack[0];
	assign la1_wr_ack = wr_ack[1];

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Arbiter

	//Round robin, then pick first available port if selected port is unavailable
	logic			rr_source		= 0;

	logic			current_source;

	logic[2:0]		burst_count		= 0;
	wire[9:0]		fifo_wr_size;
	logic			fifo_almost_full;

	logic			fifo_wr_en;
	logic[127:0]	fifo_wr_data;

	//Writes have top priority as those are hard realtime
	always_comb begin

		fifo_almost_full		= (fifo_wr_size < 4);
		fifo_wr_en				= 0;

		//Output mux to fifo
		fifo_wr_data			= wr_data[current_source];

		//If starting a new burst, write to the FIFO
		if(wr_ack)
			fifo_wr_en			= 1;

		//If continuing a burst, write to the FIFO
		if( (burst_count > 0) && wr_valid[current_source] )
			fifo_wr_en			= 1;

	end

	always_ff @(posedge clk_ram_2x) begin

		wr_ack 					= 0;

		//Burst already in progress, or data FIFO too full for a new burst? Do nothing
		if( (burst_count != 0) || fifo_almost_full) begin
		end

		//Controller can accept a command. Is the RR winner interested in writing?
		else if(wr_en[rr_source]) begin
			wr_ack[rr_source]	= 1;
			current_source		= rr_source;
			burst_count			<= 1;
		end

		//Does anyone else want to send?
		else begin
			for(integer i=0; i<NUM_PORTS; i=i+1) begin
				if(!wr_ack) begin
					if(wr_en[i]) begin
						wr_ack[i]		= 1;
						current_source	= i;
						burst_count			<= 1;
					end
				end
			end
		end

		//else nobody wants to send, do nothing

		//Bump round robin counter
		if(wr_ack) begin

			if( (rr_source + 1) >= NUM_PORTS)
				rr_source	<= 0;
			else
				rr_source	<= rr_source + 1;

		end

		if(fifo_wr_en) begin
			burst_count	<= burst_count + 1;
			if(burst_count == 4)
				burst_count	<= 0;
		end

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// CDC FIFO from fast to slow clock domain
	// TODO: we don't actually need synchronizers here as the domains are related

	(* DONT_TOUCH *)
	wire[127:0]		fifo_rd_data;

	CrossClockFifo #(
		.WIDTH(128),
		.DEPTH(512),
		.USE_BLOCK(1),
		.OUT_REG(1)
	) wr_data_fifo (
		.wr_clk(clk_ram_2x),
		.wr_en(fifo_wr_en),
		.wr_data(fifo_wr_data),
		.wr_size(fifo_wr_size),
		.wr_full(),
		.wr_overflow(),
		.wr_reset(1'b0),

		.rd_clk(clk_ram),
		.rd_en(1'b1),
		.rd_data(fifo_rd_data),
		.rd_size(),
		.rd_empty(),
		.rd_underflow(),
		.rd_reset(1'b0)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Debug logic analyzer

	ila_1 ila(
		.clk(clk_ram_2x),

		.probe0(la0_wr_en),
		.probe1(la0_wr_addr),
		.probe2(la0_wr_ack),
		.probe3(la1_wr_en),
		.probe4(la1_wr_addr),
		.probe5(la1_wr_ack),
		.probe6(current_source),
		.probe7(rr_source),
		.probe8(la1_wr_valid),
		.probe9(la0_wr_valid),
		.probe10(fifo_wr_size),
		.probe11(fifo_almost_full),
		.probe12(fifo_wr_en),
		.probe13(fifo_wr_data),
		.probe14(burst_count)
	);

endmodule
