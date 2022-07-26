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

module MicrocontrollerInterface(
	input wire		clk_50mhz,
	input wire		clk_250mhz,

	output wire		mcu_refclk,

	input wire		qspi_sck,
	input wire		qspi_cs_n,
	inout wire[3:0]	qspi_dq,
	output logic	irq = 0
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// 50 MHz reference clock output

	DDROutputBuffer #(
		.WIDTH(1)
	) ddr_clkout (
		.clk_p(clk_50mhz),
		.clk_n(!clk_50mhz),
		.dout(mcu_refclk),
		.din0(1'b0),
		.din1(1'b1)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// QSPI interface

	wire		start;
	wire		insn_valid;
	wire[7:0]	insn;
	wire		wr_valid;
	wire[7:0]	wr_data;
	logic		rd_mode		= 0;
	wire		rd_ready;
	logic		rd_valid	= 0;
	logic[7:0]	rd_data;

	QSPIDeviceInterface #(
		.INSN_BYTES(1)
	) qspi (
		.clk(clk_250mhz),
		.sck(qspi_sck),
		.cs_n(qspi_cs_n),
		.dq(qspi_dq),

		.start(start),
		.insn_valid(insn_valid),
		.insn(insn),
		.wr_valid(wr_valid),
		.wr_data(wr_data),
		.rd_mode(rd_mode),
		.rd_ready(rd_ready),
		.rd_valid(rd_valid),
		.rd_data(rd_data)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Main QSPI state machine

	logic[1:0] count = 0;

	always_ff @(posedge clk_250mhz) begin

		rd_valid	<= 0;

		if(start)
			count		<= 0;

		//0xaa is an output (read) instruction
		//anything else is input (write)
		if(insn_valid) begin
			if(insn == 8'haa)
				rd_mode	<= 1;
			else
				rd_mode	<= 0;
		end

		if(rd_ready) begin
			count		<= count + 1;

			rd_valid	<= 1;

			case(count)
				0:	rd_data <= 8'hfe;
				1:	rd_data <= 8'hed;
				2:	rd_data <= 8'hfa;
				3:	rd_data <= 8'hce;
			endcase
		end

	end

endmodule
