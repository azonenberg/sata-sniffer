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

module PowerControl(
	input wire		clk_125mhz,

	input wire		pod_present_n,
	input wire		pod_power_fault_n,
	output logic	pod_power_en		= 0
);

	//Synchronize presence-detect signal
	wire	present_n;

	ThreeStageSynchronizer #(
		.INIT(0),
		.IN_REG(0)
	) sync_present(
		.clk_in(clk_125mhz),
		.din(pod_present_n),
		.clk_out(clk_125mhz),
		.dout(present_n)
	);

	//Wait about 530ms after mate before applying power
	logic[25:0]	count = 0;

	enum logic[2:0]
	{
		STATE_OFF,
		STATE_TURNING_ON,
		STATE_ON,
		STATE_POWER_FAULT,
		STATE_FAULT_CLEARING
	} state = STATE_OFF;

	//Main power control state machine
	always_ff @(posedge clk_125mhz) begin

		case(state)

			STATE_OFF: begin

				if(!present_n) begin
					count	<= 1;
					state	<= STATE_TURNING_ON;
				end

			end	//end STATE_OFF

			STATE_TURNING_ON: begin
				count	<= count + 1;

				if(count == 0) begin
					pod_power_en	<= 1;
					state			<= STATE_ON;
				end

			end	//end STATE_TURNING_ON

			STATE_ON: begin
				if(present_n) begin
					pod_power_en	<= 0;
					state			<= STATE_OFF;
				end

				if(!pod_power_fault_n) begin
					state			<= STATE_POWER_FAULT;
					pod_power_en	<= 0;
				end

			end	//end STATE_ON

			STATE_POWER_FAULT: begin

				//Wait for pod to be unplugged
				if(present_n) begin
					count	<= 1;
					state	<= STATE_FAULT_CLEARING;
				end

			end	//end STATE_POWER_FAULT

			STATE_FAULT_CLEARING: begin
				count	<= count + 1;

				if(count == 0)
					state			<= STATE_OFF;

			end	//end STATE_FAULT_CLEARING

		endcase

	end

endmodule
