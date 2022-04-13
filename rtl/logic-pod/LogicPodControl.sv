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

/**
	@brief Control plane interface for logic analyzer pods
 */
module LogicPodControl(
	input wire		clk_125mhz,

	input wire		pod_present_n,
	input wire		pod_power_fault_n,
	output wire		pod_power_en,

	input wire		uart_rx,
	output wire		uart_tx
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Hotswap power control

	PowerControl hotswap_power(
		.clk_125mhz(clk_125mhz),
		.pod_present_n(pod_present_n),
		.pod_power_en(pod_power_en),
		.pod_power_fault_n(pod_power_fault_n)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// UART for talking to pod control logic

	wire	uart_tx_internal;

	BidirectionalBuffer obuf(
		.fabric_in(),
		.fabric_out(uart_tx_internal),
		.pad(uart_tx),
		.oe(pod_power_en)
	);

	wire		uart_rx_en;
	wire[7:0]	uart_rx_data;

	logic		uart_tx_en		= 0;
	logic[7:0]	uart_tx_data	= 0;
	wire		uart_tx_done;

	UART uart(
		.clk(clk_125mhz),
		.clkdiv(16'd1085),

		.rx(uart_rx),
		.rxactive(),
		.rx_data(uart_rx_data),
		.rx_en(uart_rx_en),

		.tx(uart_tx_internal),
		.tx_data(uart_tx_data),
		.tx_en(uart_tx_en),
		.txactive(),
		.tx_done(uart_tx_done)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// DEBUG: ROM of canned config commands

	logic[279:0] commands =
	{
		"n0PRBS\na01\nv00\n",		//120
		"n1square\na11\nv10.25\n"	//160
	};

	enum logic[3:0]
	{
		STATE_OFF			= 0,
		STATE_POWERUP_WAIT	= 1,
		STATE_SEND			= 2,
		STATE_SEND_WAIT		= 3,
		STATE_ON			= 4

	} state = STATE_OFF;

	logic[5:0] count = 0;
	logic[25:0] delay = 0;
	always_ff @(posedge clk_125mhz) begin

		uart_tx_en	<= 0;

		case(state)

			//Wait for the pod to be plugged in
			STATE_OFF: begin
				if(pod_power_en) begin
					delay	<= 1;
					state	<= STATE_POWERUP_WAIT;
				end
			end	//end STATE_OFF

			//Give it a little while to power up
			STATE_POWERUP_WAIT: begin
				delay <= delay + 1;
				if(delay == 0) begin
					count	<= 34;
					state	<= STATE_SEND;
				end
			end	//end STATE_POWERUP_WAIT

			//Send UART data
			STATE_SEND: begin
				uart_tx_en		<= 1;
				uart_tx_data	<= commands[count*8 +: 8];
				state			<= STATE_SEND_WAIT;
			end	//end STATE_SEND

			STATE_SEND_WAIT: begin
				if(uart_tx_done) begin
					if(count == 0)
						state	<= STATE_ON;
					else begin
						state	<= STATE_SEND;
						count	<= count - 1;
					end
				end
			end	//end STATE_SEND_WAIT

			STATE_ON: begin
				//wait for power to be removed
			end	//end STATE_ON

		endcase

		if(!pod_power_en)
			state	<= STATE_OFF;

	end

endmodule
