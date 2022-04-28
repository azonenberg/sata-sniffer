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

	output wire			la0_ram_data_rd_en,
	input wire[127:0]	la0_ram_data_rd_data,
	input wire[9:0]		la0_ram_data_rd_size,
	output wire			la0_ram_addr_rd_en,
	input wire[28:0]	la0_ram_addr_rd_data,
	input wire[7:0]		la0_ram_addr_rd_size,

	output wire			la1_ram_data_rd_en,
	input wire[127:0]	la1_ram_data_rd_data,
	input wire[9:0]		la1_ram_data_rd_size,
	output wire			la1_ram_addr_rd_en,
	input wire[28:0]	la1_ram_addr_rd_data,
	input wire[7:0]		la1_ram_addr_rd_size
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Remap I/Os to numbered vectors for easier handling

	localparam NUM_PORTS = 2;

	logic[NUM_PORTS-1:0]	idata_rd_en	= 0;
	assign la0_ram_data_rd_en = idata_rd_en[0];
	assign la1_ram_data_rd_en = idata_rd_en[1];

	logic[127:0]			idata_rd_data[NUM_PORTS-1:0];
	assign idata_rd_data[0] = la0_ram_data_rd_data;
	assign idata_rd_data[1] = la1_ram_data_rd_data;

	logic[9:0]				idata_rd_size[NUM_PORTS-1:0];
	assign idata_rd_size[0] = la0_ram_data_rd_size;
	assign idata_rd_size[1] = la1_ram_data_rd_size;

	logic[NUM_PORTS-1:0]	iaddr_rd_en	= 0;
	assign la0_ram_addr_rd_en = iaddr_rd_en[0];
	assign la1_ram_addr_rd_en = iaddr_rd_en[1];

	logic[28:0]				iaddr_rd_data[NUM_PORTS-1:0];
	assign iaddr_rd_data[0] = la0_ram_addr_rd_data;
	assign iaddr_rd_data[1] = la1_ram_addr_rd_data;

	logic[7:0]				iaddr_rd_size[NUM_PORTS-1:0];
	assign iaddr_rd_size[0] = la0_ram_addr_rd_size;
	assign iaddr_rd_size[1] = la1_ram_addr_rd_size;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Arbitration and deserialization to a single 256 bit stream in the 2x clock domain

	//Round robin, then pick first available port if selected port is unavailable
	logic					rr_source		= 0;
	logic					current_source;

	logic[2:0]				burst_count		= 0;
	logic					rd_phase		= 0;

	wire[9:0]				data_fifo_wr_size;
	wire[8:0]				addr_fifo_wr_size;
	logic					output_almost_full;

	logic[NUM_PORTS-1:0]	input_data_ready	= 0;

	logic					iaddr_rd_valid_adv	= 0;
	logic					iaddr_rd_valid		= 0;
	logic					idata_rd_valid_adv	= 0;
	logic					idata_rd_valid		= 0;

	logic					data_fifo_wr_en		= 0;
	logic					addr_fifo_wr_en		= 0;

	logic[127:0]			idata_rd_muxed;
	logic[28:0]				iaddr_rd_muxed;

	logic[29:0]				addr_fifo_wr_data;
	logic[255:0]			data_fifo_wr_data;

	always_comb begin

		//Make sure we have room in the output
		output_almost_full		= (data_fifo_wr_size < 5) || (addr_fifo_wr_size < 2);

		//Main read muxes
		idata_rd_muxed			= idata_rd_data[current_source];
		iaddr_rd_muxed			= iaddr_rd_data[current_source];

	end

	enum logic
	{
		MIG_CMD_READ	= 1'b1,
		MIG_CMD_WRITE	= 1'b0
	} mig_cmd_t;

	logic	can_start_burst;

	always_comb begin

		//Can't do anything if no space in output
		if(output_almost_full)
			 can_start_burst = 0;

		//Can start a new burst if idle, or in the last cycle of an existing one
		else if( (burst_count == 0) || (burst_count == 4) )
			can_start_burst = 1;

		//Nope, busy with an existing burst
		else
			can_start_burst = 0;

	end

	logic	hit = 0;
	always_ff @(posedge clk_ram_2x) begin

		//Check if there's something to read on the input
		for(integer i=0; i<NUM_PORTS; i=i+1)
			input_data_ready[i]	<= (idata_rd_size[i] >= 4) && (iaddr_rd_size[i] >= 1);

		//Default flags to off
		idata_rd_en				<= 0;
		iaddr_rd_en				<= 0;
		data_fifo_wr_en			<= 0;
		addr_fifo_wr_en			<= 0;

		//Pipeline valid flags
		iaddr_rd_valid_adv		<= (iaddr_rd_en != 0);
		iaddr_rd_valid			<= iaddr_rd_valid_adv;
		idata_rd_valid_adv		<= (idata_rd_en != 0);
		idata_rd_valid			<= idata_rd_valid_adv;

		//Bump round robin counter if starting a burst
		if(burst_count == 1) begin
			rr_source		<= rr_source + 1;
			if( (rr_source + 1) >= NUM_PORTS)
				rr_source	<= 0;
		end

		//Keep track of position in a burst
		if(burst_count > 0) begin
			burst_count	<= burst_count + 1;
			if(burst_count == 4)
				burst_count	<= 0;
			else
				idata_rd_en[current_source]	<= 1;
		end

		//Ready to start a new burst?
		if(can_start_burst) begin

			hit = 0;

			//Check if the round robin winner wants to send
			if(input_data_ready[rr_source]) begin
				hit 					= 1;
				current_source			= rr_source;
			end

			//Check if anyone else wants to send
			else begin
				for(integer i=0; i<NUM_PORTS; i=i+1) begin
					if(!hit && input_data_ready[i]) begin
						hit 			= 1;
						current_source	= i;
					end

				end
			end

			//If somebody wants to send, make it happen
			if(hit) begin
				idata_rd_en[current_source]	<= 1;
				iaddr_rd_en[current_source]	<= 1;
				burst_count					<= 1;
			end

		end

		//Push address/command to the FIFO
		if(iaddr_rd_valid) begin
			addr_fifo_wr_en		<= 1;
			addr_fifo_wr_data	<= { MIG_CMD_WRITE, iaddr_rd_muxed };
		end

		//Push data to the FIFO
		if(idata_rd_valid) begin
			rd_phase			<= !rd_phase;
			data_fifo_wr_data	<= { data_fifo_wr_data[127:0], idata_rd_muxed };
			if(rd_phase)
				data_fifo_wr_en	<= 1;
		end

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// CDC FIFOs from fast to slow clock domain

	wire[255:0]		fifo_rd_data;
	wire[9:0]		data_fifo_rd_size;
	logic			data_fifo_rd_en	= 1;

	CrossClockFifo #(
		.WIDTH(256),
		.DEPTH(512),
		.USE_BLOCK(1),
		.OUT_REG(1)
	) wr_data_fifo (
		.wr_clk(clk_ram_2x),
		.wr_en(data_fifo_wr_en),
		.wr_data(data_fifo_wr_data),
		.wr_size(data_fifo_wr_size),
		.wr_full(),
		.wr_overflow(),
		.wr_reset(1'b0),

		.rd_clk(clk_ram),
		.rd_en(data_fifo_rd_en),
		.rd_data(fifo_rd_data),
		.rd_size(data_fifo_rd_size),
		.rd_empty(),
		.rd_underflow(),
		.rd_reset(1'b0)
	);

	wire			fifo_rd_cmd;
	wire[28:0]		fifo_rd_addr;

	wire[8:0]		cmd_addr_fifo_rd_size;
	logic			cmd_addr_fifo_rd_en	= 1;
	CrossClockFifo #(
		.WIDTH(30),
		.DEPTH(256),
		.USE_BLOCK(1),
		.OUT_REG(1)
	) cmd_addr_fifo (
		.wr_clk(clk_ram_2x),
		.wr_en(addr_fifo_wr_en),
		.wr_data(addr_fifo_wr_data),
		.wr_size(addr_fifo_wr_size),
		.wr_full(),
		.wr_overflow(),
		.wr_reset(1'b0),

		.rd_clk(clk_ram),
		.rd_en(cmd_addr_fifo_rd_en),
		.rd_data({fifo_rd_cmd, fifo_rd_addr}),
		.rd_size(cmd_addr_fifo_rd_size),
		.rd_empty(),
		.rd_underflow(),
		.rd_reset(1'b0)
	);

	/*
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Pop the FIFOs into the MIG core

	logic	out_phase	 = 0;

	logic	fifos_ready_to_start;

	always_comb begin

		cmd_addr_fifo_rd_en	= 0;
		data_fifo_rd_en		= 0;

		//Check if FIFOs are ready for us to begin (one command and two data words available)
		fifos_ready_to_start	= (!cmd_addr_fifo_empty) && (data_fifo_rd_size >= 2);

		//Idle? Pop data+command FIFO if they're ready
		if(!out_phase && fifos_ready_to_start && app_rdy && app_wdf_rdy) begin
			cmd_addr_fifo_rd_en		= 1;
			data_fifo_rd_en			= 1;
		end

		//Read second half of a word
		if(out_phase)
			data_fifo_rd_en			= 1;

	end

	logic	cmd_addr_fifo_rd_valid	= 0;
	logic	data_fifo_rd_valid		= 0;
	always_ff @(posedge clk_ram) begin

		//Clear single cycle flags
		app_en			<= 0;
		app_wdf_wren	<= 0;
		app_wdf_end		<= 0;

		//Now in second half of a word
		if(cmd_addr_fifo_rd_en)
			out_phase	<= 1;

		//Done
		if(out_phase)
			out_phase	<= 0;

		//Forward outputs to controller
		//TODO: combinatorial if we can afford this?
		cmd_addr_fifo_rd_valid	<= cmd_addr_fifo_rd_en;
		data_fifo_rd_valid		<= data_fifo_rd_en;

		if(cmd_addr_fifo_rd_valid) begin
			app_cmd 	<= {2'b0, fifo_rd_cmd };
			app_addr	<= fifo_rd_addr;
			app_en		<= 1;
		end

		if(data_fifo_rd_valid) begin
			app_wdf_wren	<= 1;
			app_wdf_mask	<= 0;
			app_wdf_data	<= data_fifo_rd_valid;
			app_wdf_end		<= !cmd_addr_fifo_rd_valid;	//second half of burst?
		end

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Debug logic analyzer
	*/
	ila_1 ila(
		.clk(clk_ram_2x),

		.probe0(la0_ram_data_rd_en),
		.probe1(la0_ram_data_rd_size),
		.probe2(la0_ram_addr_rd_en),
		.probe3(la0_ram_addr_rd_size),
		.probe4(la1_ram_data_rd_en),
		.probe5(la1_ram_data_rd_size),
		.probe6(la1_ram_addr_rd_en),
		.probe7(la1_ram_addr_rd_size),
		.probe8(data_fifo_wr_en),
		.probe9(data_fifo_wr_size),
		.probe10(addr_fifo_wr_en),
		.probe11(addr_fifo_wr_size),
		.probe12(output_almost_full),
		.probe13(input_data_ready),
		.probe14(iaddr_rd_valid),
		.probe15(idata_rd_valid),
		.probe16(data_fifo_wr_data),
		.probe17(addr_fifo_wr_data),
		.probe18(rr_source),
		.probe19(current_source),
		.probe20(rd_phase),
		.probe21(burst_count),
		.probe22(can_start_burst)
	);

	ila_0 ila0(
		.clk(clk_ram),
		.probe0(data_fifo_rd_size),
		.probe1(cmd_addr_fifo_rd_size),
		.probe2(app_rdy),
		.probe3(fifo_rd_cmd),
		.probe4(fifo_rd_addr),
		.probe5(fifo_rd_data),
		.probe6(app_wdf_end),
		.probe7(app_wdf_wren),
		.probe8(app_wdf_rdy),
		.probe9(app_addr),
		.probe10(app_cmd),
		.probe11(app_en),
		.probe12(data_fifo_rd_en),
		.probe13(cmd_addr_fifo_rd_en)
	);

endmodule
