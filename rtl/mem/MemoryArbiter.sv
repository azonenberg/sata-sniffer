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
	output logic[31:0]	app_wdf_mask	= 0,
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
	input wire[28:0]	la0_wr_addr,
	input wire[127:0]	la0_wr_data,
	output wire			la0_wr_ack,

	input wire			la1_wr_en,
	input wire[28:0]	la1_wr_addr,
	input wire[127:0]	la1_wr_data,
	output wire			la1_wr_ack
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//

	assign la0_wr_ack = la0_wr_en;
	assign la1_wr_ack = la1_wr_en;

endmodule
