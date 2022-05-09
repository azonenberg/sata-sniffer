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

module LogicTriggering(

	//Clocks
	input wire		clk_ram,
	input wire		clk_ram_2x,
	input wire		la0_clk_312p5mhz,
	input wire		la1_clk_312p5mhz,

	//Arm request from upper level (clk_ram_2x domain)
	input wire		arm_req,

	//Output controls to port logic (in port clock domain)
	output wire		trig_rst_la0,				//clk_312p5mhz
	output wire		capture_en_la0,
	output wire		capture_flush_la0,

	output wire		trig_rst_la1,				//clk_312p5mhz
	output wire		capture_en_la1,
	output wire		capture_flush_la1,

	output wire		trig_rst_arbiter_2x,		//clk_ram_2x
	output wire		capture_flush_arbiter_2x,

	output wire		trig_rst_arbiter,			//clk_ram
	output wire		capture_flush_arbiter
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Generate reset and trigger arm signals for each LA

	//Main trigger state machine
	enum logic[2:0]
	{
		TRIG_STATE_IDLE			= 0,	//trigger not armed, nothing to do
		TRIG_STATE_RESET		= 1,	//resetting and flushing buffers
		TRIG_STATE_PRE_TRIG		= 2,	//filling pretrigger buffer, not looking for events yet
		TRIG_STATE_ARMED		= 3,	//armed and ready to go
		TRIG_STATE_POST_TRIG	= 4,	//trigger event happened, filling rest of buffer
		TRIG_STATE_DONE			= 5,	//done, waiting for waveform download to finish
		TRIG_STATE_CLEAR		= 6		//done, waiting for arm request to go low (just for debug)
	} trig_state = TRIG_STATE_IDLE;

	logic	trig_rst_internal		= 0;
	logic	capture_en_internal		= 0;
	logic	capture_flush_internal	= 0;

	logic[31:0]	count				= 0;

	logic[31:0] pre_trig_buf_size	= 256;
	logic[31:0] post_trig_buf_size	= 256;

	always_ff @(posedge clk_ram_2x) begin

		capture_flush_internal	<= 0;

		case(trig_state)

			//IDLE - wait for arm request, then reset everything
			TRIG_STATE_IDLE: begin

				if(arm_req) begin
					trig_rst_internal	<= 1;
					count				<= 0;
					trig_state			<= TRIG_STATE_RESET;
				end

			end	//end TRIG_STATE_IDLE

			//RESET - wait for resets to propagate through all of the respective clock domains,
			//then arm the trigger
			TRIG_STATE_RESET: begin
				count	<= count + 1;
				if(count == 31)
					trig_rst_internal	<= 0;
				else if(count == 63) begin
					capture_en_internal	<= 1;
					count				<= 0;
					trig_state			<= TRIG_STATE_PRE_TRIG;
				end
			end	//end TRIG_STATE_RESET

			//PRE TRIGGER - acquire samples into the buffer until the pre-trigger window elapses, but do not
			//arm the trigger yet.
			TRIG_STATE_PRE_TRIG: begin
				count	<= count + 1;
				if(count == pre_trig_buf_size)
					trig_state			<= TRIG_STATE_ARMED;
			end	//end TRIG_STATE_PRE_TRIG

			//ARMED - wait for trigger event.
			//For now, trigger instantly.
			TRIG_STATE_ARMED: begin
				trig_state				<= TRIG_STATE_POST_TRIG;
				count					<= 0;
			end	//end TRIG_STATE_ARMED

			//POST TRIGGER - acquire samples to desired memory depth
			TRIG_STATE_POST_TRIG: begin
				count	<= count + 1;
				if(count == post_trig_buf_size) begin
					capture_en_internal		<= 0;
					capture_flush_internal	<= 1;
					trig_state				<= TRIG_STATE_DONE;
				end
			end	//end TRIG_STATE_POST_TRIG

			//DONE - wait for waveform download
			TRIG_STATE_DONE: begin
				//for now, assume instant
				trig_state				<= TRIG_STATE_CLEAR;
			end	//end TRIG_STATE_DONE

			//CLEAR - wait for arm signal to go low
			TRIG_STATE_CLEAR: begin
				if(!arm_req)
					trig_state			<= TRIG_STATE_IDLE;
			end	//end TRIG_STATE_CLEAR

		endcase

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Synchronize stuff into the output domain

	ThreeStageSynchronizer #(
		.INIT(0),
		.IN_REG(0)
	) sync_rst_la0 (
		.clk_in(clk_ram_2x),
		.din(trig_rst_internal),
		.clk_out(la0_clk_312p5mhz),
		.dout(trig_rst_la0)
	);

	ThreeStageSynchronizer #(
		.INIT(0),
		.IN_REG(0)
	) sync_rst_la1 (
		.clk_in(clk_ram_2x),
		.din(trig_rst_internal),
		.clk_out(la1_clk_312p5mhz),
		.dout(trig_rst_la1)
	);

	//no sync needed
	assign trig_rst_arbiter_2x = trig_rst_internal;

	ThreeStageSynchronizer #(
		.INIT(0),
		.IN_REG(0)
	) sync_rst_arbiter (
		.clk_in(clk_ram_2x),
		.din(trig_rst_internal),
		.clk_out(clk_ram),
		.dout(trig_rst_arbiter)
	);

	ThreeStageSynchronizer #(
		.INIT(0),
		.IN_REG(0)
	) sync_en_la0 (
		.clk_in(clk_ram_2x),
		.din(capture_en_internal),
		.clk_out(la0_clk_312p5mhz),
		.dout(capture_en_la0)
	);

	ThreeStageSynchronizer #(
		.INIT(0),
		.IN_REG(0)
	) sync_en_la1 (
		.clk_in(clk_ram_2x),
		.din(capture_en_internal),
		.clk_out(la1_clk_312p5mhz),
		.dout(capture_en_la1)
	);

	PulseSynchronizer sync_flush_la0(
		.clk_a(clk_ram_2x),
		.pulse_a(capture_flush_internal),
		.clk_b(la0_clk_312p5mhz),
		.pulse_b(capture_flush_la0));

	PulseSynchronizer sync_flush_la1(
		.clk_a(clk_ram_2x),
		.pulse_a(capture_flush_internal),
		.clk_b(la1_clk_312p5mhz),
		.pulse_b(capture_flush_la1));

	//no sync needed
	assign capture_flush_arbiter_2x = capture_flush_internal;

	PulseSynchronizer sync_flush_arbiter(
		.clk_a(clk_ram_2x),
		.pulse_a(capture_flush_internal),
		.clk_b(clk_ram),
		.pulse_b(capture_flush_arbiter));

endmodule
