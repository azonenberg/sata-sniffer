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

/*
	MEMORY BUS FORMAT

	The DRAM (Kingston KVR16LS11S6/2) is 2^28 locations x 64 bits = 2GB (0x8000_0000) bytes in size.
	A burst is 4 clocks so if we only do aligned accesses, we have 2^26 256-bit blocks.

	RAM subsystem is 162.5 MHz on a 256 bit bus.
	Hope to upgrade to 200 MHz on a 256 bit bus (51.2 Gbps peak throughput)

	BITS (assuming 32-bit linear address space)

	TODO: this implies we should have one more address bit than we actually do... what gives?
	31				Unmapped, always 0
	30:16			Row address (15)
	15:13			Bank address (3)
	12:3			Column address (10)
	2:0				Unimplemented, always 0 (64 bit alignment)

	Rationale for choosing row-bank-col over bank-row-col: if we partition half the ram for LA and half for
	SATA with bank as the MSB then we have 4 banks statically allocated to the entire LA subsystem and 4 banks to the
	SATA subsystem. Each set of four adjacent channels will share a single bank, which is great if the expected write
	load per channel is similar. But it's not.

	If we use row-bank-col, OTOH, large write bursts within a single LA channel can stripe across multiple banks. We
	expect some channels to be much more compressible than others, so this allows the poorly compressible channels to
	get better performance by using multiple banks for bulk writes.

	MEMORY MAP

	Byte addressed for readability. Physical address bus is missing low 3 bits because 64 bit wide SODIMM

	START			END
	0000_0000		3FFF_FFFF			Reserved for use by SATA subsystem

	LOGIC ANALYZER BUFFER MEMORY
	START			END
	4000_0000		43FF_FFFF			LA0.0
	4400_0000		47FF_FFFF			LA0.1
	4800_0000		4BFF_FFFF			LA0.2
	4C00_0000		4FFF_FFFF			LA0.3
	5000_0000		53FF_FFFF			LA0.4
	5400_0000		57FF_FFFF			LA0.5
	5800_0000		5BFF_FFFF			LA0.6
	5C00_0000		5FFF_FFFF			LA0.7
	6000_0000		63FF_FFFF			LA1.0
	6400_0000		67FF_FFFF			LA1.1
	6800_0000		6BFF_FFFF			LA1.2
	6C00_0000		6FFF_FFFF			LA1.3
	7000_0000		73FF_FFFF			LA1.4
	7400_0000		77FF_FFFF			LA1.7
	7800_0000		7BFF_FFFF			LA1.6
	7C00_0000		7FFF_FFFF			LA1.7

	Each LA channel has 0x4000000 bytes or 64 MByte / 512 Mbit / 2 Mrows of sample buffer.


	Each row can store 15 compression blocks, or 240 to 3810 bits, depending on compression ratio.
	This gives a guaranteed memory depth of 480M samples, possibly up to 7620M with optimal compression.
 */
module MemorySubsystem(

	//Bus from DDR controller to top level
	inout wire[63:0]	ddr3_dq,
	inout wire[7:0]		ddr3_dqs_n,
	inout wire[7:0]		ddr3_dqs_p,
	output wire[14:0]	ddr3_addr,
	output wire[2:0]	ddr3_ba,
	output wire			ddr3_ras_n,
	output wire			ddr3_cas_n,
	output wire			ddr3_we_n,
	output wire			ddr3_reset_n,
	output wire[0:0]	ddr3_ck_p,
	output wire[0:0]	ddr3_ck_n,
	output wire[0:0]	ddr3_cke,
	output wire[0:0]	ddr3_cs_n,
	output wire[7:0]	ddr3_dm,
	output wire[0:0]	ddr3_odt,

	//Reference clock from top level
	input wire			clk_200mhz,

	//Status outputs
	output wire			ddr_cal_complete,

	//Clock to client domains
	output wire			clk_ram,
	output wire			clk_ram_2x,

	//Buses from client domains to arbiter
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
	input wire[7:0]		la1_ram_addr_rd_size,

	output wire			la_readback_ram_addr_rd_en,
	input wire[28:0]	la_readback_ram_addr_rd_data,
	input wire[7:0]		la_readback_ram_addr_rd_size,

	//Flush signals from logic analyzer subsystem
	input wire			trig_rst_arbiter,			//clk_ram
	input wire			capture_flush_arbiter,

	input wire			trig_rst_arbiter_2x,		//clk_ram_2x
	input wire			flush_arbiter_2x,
	input wire			la0_flush_complete,
	input wire			la1_flush_complete,
	output wire			flush_done
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// PLL for 2x memory interface clock

	wire	clk_ram_2x_raw;

	wire	pll_lock;
	ClockBuffer #(
		.TYPE("GLOBAL"),
		.CE("YES")
	) buf_clk_ram_2x(
		.clkin(clk_ram_2x_raw),
		.ce(pll_lock),
		.clkout(clk_ram_2x)
	);

	wire	pll_ram_fb;
	PLLE2_BASE #(
		.BANDWIDTH("OPTIMIZED"),
		.STARTUP_WAIT("TRUE"),

		.CLKFBOUT_MULT(8),			//162.5 MHz -> 1.3 GHz VCO
		.DIVCLK_DIVIDE(1),
		.CLKFBOUT_PHASE(0),

		.CLKIN1_PERIOD(6.1538),

		.CLKOUT0_DIVIDE(4),			//325 MHz output to LA muxing datapath
		.CLKOUT1_DIVIDE(16),
		.CLKOUT2_DIVIDE(16),
		.CLKOUT3_DIVIDE(16),
		.CLKOUT4_DIVIDE(16),
		.CLKOUT5_DIVIDE(16),

		.CLKOUT0_PHASE(0.0),
		.CLKOUT1_PHASE(0.0),
		.CLKOUT2_PHASE(0.0),
		.CLKOUT3_PHASE(0.0),
		.CLKOUT4_PHASE(0.0),
		.CLKOUT5_PHASE(0.0),

		.CLKOUT0_DUTY_CYCLE(0.5),
		.CLKOUT1_DUTY_CYCLE(0.5),
		.CLKOUT2_DUTY_CYCLE(0.5),
		.CLKOUT3_DUTY_CYCLE(0.5),
		.CLKOUT4_DUTY_CYCLE(0.5),
		.CLKOUT5_DUTY_CYCLE(0.5)
	) pll_ram (
		.RST(1'b0),
		.PWRDWN(1'b0),
		.LOCKED(pll_lock),

		.CLKIN1(clk_ram),
		.CLKFBIN(pll_ram_fb),
		.CLKFBOUT(pll_ram_fb),

		.CLKOUT0(clk_ram_2x_raw),
		.CLKOUT1(),
		.CLKOUT2(),
		.CLKOUT3(),
		.CLKOUT4(),
		.CLKOUT5()
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Memory controller

	wire[28:0]	ddr3_app_addr;
	wire[2:0]	ddr3_app_cmd;
	wire		ddr3_app_en;

	wire[255:0]	ddr3_app_wdf_data;
	wire		ddr3_app_wdf_end;
	wire[31:0]	ddr3_app_wdf_mask;
	wire		ddr3_app_wdf_wren;
	wire		ddr3_app_wdf_rdy;

	wire		ddr3_app_ref_req;
	wire		ddr3_app_sr_req;
	wire		ddr3_app_zq_req;

	wire[255:0]	ddr3_app_rd_data;
	wire		ddr3_app_rd_data_end;
	wire		ddr3_app_rd_data_valid;

	wire		ddr3_app_rdy;

	ddr3 ram(

		//Top level pins
		.ddr3_dq(ddr3_dq),
		.ddr3_dqs_p(ddr3_dqs_p),
		.ddr3_dqs_n(ddr3_dqs_n),
		.ddr3_addr(ddr3_addr),
		.ddr3_ba(ddr3_ba),
		.ddr3_ras_n(ddr3_ras_n),
		.ddr3_cas_n(ddr3_cas_n),
		.ddr3_we_n(ddr3_we_n),
		.ddr3_reset_n(ddr3_reset_n),
		.ddr3_ck_p(ddr3_ck_p),
		.ddr3_ck_n(ddr3_ck_n),
		.ddr3_cke(ddr3_cke),
		.ddr3_cs_n(ddr3_cs_n),
		.ddr3_dm(ddr3_dm),
		.ddr3_odt(ddr3_odt),

		//Internal ports
		.sys_clk_i(clk_200mhz),	//Main controller clock
		.clk_ref_i(clk_200mhz),	//Reference clock for IDELAYCTRLs
		.app_addr(ddr3_app_addr),
		.app_cmd(ddr3_app_cmd),
		.app_en(ddr3_app_en),
		.app_wdf_data(ddr3_app_wdf_data),
		.app_wdf_end(ddr3_app_wdf_end),
		.app_wdf_mask(ddr3_app_wdf_mask),
		.app_wdf_wren(ddr3_app_wdf_wren),
		.app_rd_data(ddr3_app_rd_data),
		.app_rd_data_end(ddr3_app_rd_data_end),
		.app_rd_data_valid(ddr3_app_rd_data_valid),
		.app_rdy(ddr3_app_rdy),
		.app_wdf_rdy(ddr3_app_wdf_rdy),
		.app_sr_req(ddr3_app_sr_req),
		.app_ref_req(ddr3_app_ref_req),
		.app_zq_req(ddr3_app_zq_req),
		.app_sr_active(),
		.app_ref_ack(),
		.app_zq_ack(),
		.ui_clk(clk_ram),
		.ui_clk_sync_rst(),
		.init_calib_complete(ddr_cal_complete),
		.device_temp(),
		.sys_rst(1'b0)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Arbiter

	MemoryArbiter arbiter(
		.clk_ram(clk_ram),
		.clk_ram_2x(clk_ram_2x),

		//Memory
		.app_addr(ddr3_app_addr),
		.app_cmd(ddr3_app_cmd),
		.app_en(ddr3_app_en),
		.app_wdf_data(ddr3_app_wdf_data),
		.app_wdf_end(ddr3_app_wdf_end),
		.app_wdf_mask(ddr3_app_wdf_mask),
		.app_wdf_wren(ddr3_app_wdf_wren),
		.app_wdf_rdy(ddr3_app_wdf_rdy),
		.app_ref_req(ddr3_app_ref_req),
		.app_sr_req(ddr3_app_sr_req),
		.app_zq_req(ddr3_app_zq_req),
		.app_rd_data(ddr3_app_rd_data),
		.app_rd_data_end(ddr3_app_rd_data_end),
		.app_rd_data_valid(ddr3_app_rd_data_valid),
		.app_rdy(ddr3_app_rdy),

		//Client domains
		.la0_ram_data_rd_en(la0_ram_data_rd_en),
		.la0_ram_data_rd_data(la0_ram_data_rd_data),
		.la0_ram_data_rd_size(la0_ram_data_rd_size),
		.la0_ram_addr_rd_en(la0_ram_addr_rd_en),
		.la0_ram_addr_rd_data(la0_ram_addr_rd_data),
		.la0_ram_addr_rd_size(la0_ram_addr_rd_size),

		.la1_ram_data_rd_en(la1_ram_data_rd_en),
		.la1_ram_data_rd_data(la1_ram_data_rd_data),
		.la1_ram_data_rd_size(la1_ram_data_rd_size),
		.la1_ram_addr_rd_en(la1_ram_addr_rd_en),
		.la1_ram_addr_rd_data(la1_ram_addr_rd_data),
		.la1_ram_addr_rd_size(la1_ram_addr_rd_size),

		.la_readback_ram_addr_rd_en(la_readback_ram_addr_rd_en),
		.la_readback_ram_addr_rd_data(la_readback_ram_addr_rd_data),
		.la_readback_ram_addr_rd_size(la_readback_ram_addr_rd_size),

		//Flush signals
		.trig_rst_arbiter(trig_rst_arbiter),
		.capture_flush_arbiter(capture_flush_arbiter),
		.trig_rst_arbiter_2x(trig_rst_arbiter_2x),
		.flush_arbiter_2x(flush_arbiter_2x),
		.la0_flush_complete(la0_flush_complete),
		.la1_flush_complete(la1_flush_complete),
		.flush_done(flush_done)
	);

endmodule
