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

`include "MicrocontrollerInterface.svh"

module MicrocontrollerInterface(

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// System clocks

	input wire			clk_50mhz,
	input wire			clk_250mhz,

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Bus to MCU

	output wire			mcu_refclk,

	input wire			qspi_sck,
	input wire			qspi_cs_n,
	inout wire[3:0]		qspi_dq,
	output logic		irq = 0,

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Interface to internal FPGA blocks

	output cfgregs_t	cfgregs	= 0
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
	wire[15:0]	insn;
	wire		wr_valid;
	wire[7:0]	wr_data;
	logic		rd_mode		= 0;
	wire		rd_ready;
	logic		rd_valid	= 0;
	logic[7:0]	rd_data;

	QSPIDeviceInterface #(
		.INSN_BYTES(2)
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
	// Device info block

	wire[63:0] die_serial;
	wire[31:0] idcode;

	DeviceInfo_7series devinfo(
		.clk(clk_50mhz),
		.die_serial(die_serial),
		.die_serial_valid(),
		.idcode(idcode),
		.idcode_valid()
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// List of registers

	typedef enum logic[15:0]
	{
		REG_FPGA_IDCODE =	16'h0000,	//R: 4 byte JTAG IDCODE
		REG_FPGA_SERIAL =	16'h0001,	//R: 8 byte die serial number
		REG_MAC_ADDRESS	=	16'h0002,	//W: 6 byte MAC address

		REG_IP_ADDRESS	=	16'h0003,	//W: 4 byte IPv4 address
		REG_SUBNET_MASK =	16'h0004,	//W: 4 byte IPv4 subnet mask
		REG_BROADCAST	=	16'h0005,	//W: 4 byte IPV4 broadcast address
		REG_GATEWAY		=	16'h0006	//W: 4 byte IPv4 default gateway
	} opcode_t;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Main QSPI state machine

	logic[15:0] count = 0;

	always_ff @(posedge clk_250mhz) begin

		rd_valid					<= 0;
		cfgregs.mac_address_updated	<= 0;
		cfgregs.ip_config_updated	<= 0;

		//Output read data
		if(rd_ready) begin
			count		<= count + 1;
			rd_valid	<= 1;

			case(insn)

				REG_FPGA_IDCODE:	rd_data <= idcode[(3 - count[1:0])*8 +: 8];
				REG_FPGA_SERIAL:	rd_data <= die_serial[(7 - count[2:0])*8 +: 8];

				//default to no output
				default:
					rd_data	<= 8'h0;

			endcase

		end

		//Tristate inputs by default, except for instructions that read values from us
		if(insn_valid) begin
			case(insn)

				REG_FPGA_IDCODE:	rd_mode	<= 1;
				REG_FPGA_SERIAL:	rd_mode	<= 1;

				//Reset count during read operations
				default: begin
					rd_mode	<= 0;
					count	<= 0;
				end

			endcase
		end

		//Process incoming data
		if(wr_valid) begin
			count		<= count + 1;

			case(insn)

				//Ugly because non power of two size
				REG_MAC_ADDRESS: begin

					case(count)
						0:	cfgregs.mac_address[40 +: 8] <= wr_data;
						1:	cfgregs.mac_address[32 +: 8] <= wr_data;
						2:	cfgregs.mac_address[24 +: 8] <= wr_data;
						3:	cfgregs.mac_address[16 +: 8] <= wr_data;
						4:	cfgregs.mac_address[8 +: 8] <= wr_data;
						5: begin
							cfgregs.mac_address[0 +: 8] <= wr_data;
							cfgregs.mac_address_updated	<= 1;
						end
					endcase

				end	//end REG_MAC_ADDRESS

				REG_IP_ADDRESS: begin
					cfgregs.ip_config.address[(3-count[1:0])*8 +: 8] <= wr_data;
					if(count == 3)
						cfgregs.ip_config_updated	<= 1;
				end	//end REG_IP_ADDRESS

				REG_SUBNET_MASK: begin
					cfgregs.ip_config.mask[(3-count[1:0])*8 +: 8] <= wr_data;
					if(count == 3)
						cfgregs.ip_config_updated	<= 1;
				end	//end REG_SUBNET_MASK

				REG_BROADCAST: begin
					cfgregs.ip_config.broadcast[(3-count[1:0])*8 +: 8] <= wr_data;
					if(count == 3)
						cfgregs.ip_config_updated	<= 1;
				end	//end REG_BROADCAST

				REG_GATEWAY: begin
					cfgregs.ip_config.gateway[(3-count[1:0])*8 +: 8] <= wr_data;
					if(count == 3)
						cfgregs.ip_config_updated	<= 1;
				end	//end REG_GATEWAY

			endcase

		end

		//Reset on CS# falling edge
		if(start) begin
			count		<= 0;
		end

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Debug logic analyzer

	ila_2 ila(
		.clk(clk_250mhz),
		.probe0(start),
		.probe1(insn_valid),
		.probe2(insn),
		.probe3(wr_valid),
		.probe4(wr_data),
		.probe5(rd_ready),
		.probe6(rd_valid),
		.probe7(rd_data),
		.probe8(rd_mode),
		.probe9(count)
	);

endmodule
