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

`include "LogicPod.svh"

/**
	@brief Logic analyzer pod datapath

	This module takes in the combinatorial LVDS output of a single 8-bit logic analyzer pod (MEAD, CONWAY, etc) and
	samples it at 5 Gsps.

	All clocks must come from the same PLL and be aligned.
 */
module LogicPodDatapath #(
	parameter LANE_INVERT 	= 8'b00000000,
	parameter POD_NUMBER 	= 0
) (

	//Main clock
	input wire			clk_312p5mhz,

	//Reference clock for IDELAYs
	input wire			clk_400mhz,

	//Oversampling clocks
	input wire			clk_625mhz_fabric,
	input wire			clk_625mhz_io_0,
	input wire			clk_625mhz_io_90,

	//LVDS input
	input wire[7:0]		pod_data_p,
	input wire[7:0]		pod_data_n,

	//DDR interface
	input wire			clk_ram_2x,
	input wire			ram_ready,

	input wire			ram_data_rd_en,
	output wire[127:0]	ram_data_rd_data,
	output wire[9:0]	ram_data_rd_size,
	input wire			ram_addr_rd_en,
	output wire[28:0]	ram_addr_rd_data,
	output wire[7:0]	ram_addr_rd_size,

	//Trigger control (312.5 MHz domain)
	input wire			trig_rst,
	input wire			capture_en,
	input wire			capture_flush,
	input wire			trig_rst_arbiter_2x,

	//Trigger status (clk_ram_2x domain)
	//Goes high when all of the per-port FIFOs have been flushed to the output FIFO
	output wire			flush_complete,

	//Read pointer access (clk_ram_2x domain)
	input wire			ptr_rd_en,
	input wire[2:0]		ptr_rd_addr,
	output wire[28:0]	ptr_rd_data
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Remap pod data lines to match front panel ordering

	wire[7:0]	pod_data_p_remapped;
	wire[7:0]	pod_data_n_remapped;

	assign pod_data_p_remapped[0] = pod_data_p[3];
	assign pod_data_n_remapped[0] = pod_data_n[3];

	assign pod_data_p_remapped[1] = pod_data_p[2];
	assign pod_data_n_remapped[1] = pod_data_n[2];

	assign pod_data_p_remapped[2] = pod_data_p[7];
	assign pod_data_n_remapped[2] = pod_data_n[7];

	assign pod_data_p_remapped[3] = pod_data_p[6];
	assign pod_data_n_remapped[3] = pod_data_n[6];

	assign pod_data_p_remapped[4] = pod_data_p[5];
	assign pod_data_n_remapped[4] = pod_data_n[5];

	assign pod_data_p_remapped[5] = pod_data_p[4];
	assign pod_data_n_remapped[5] = pod_data_n[4];

	assign pod_data_p_remapped[6] = pod_data_p[1];
	assign pod_data_n_remapped[6] = pod_data_n[1];

	assign pod_data_p_remapped[7] = pod_data_p[0];
	assign pod_data_n_remapped[7] = pod_data_n[0];

	localparam LANE_INVERT_REMAPPED =
	{
		LANE_INVERT[0],
		LANE_INVERT[1],
		LANE_INVERT[4],
		LANE_INVERT[5],
		LANE_INVERT[6],
		LANE_INVERT[7],
		LANE_INVERT[2],
		LANE_INVERT[3]
	};

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// LVDS input buffers

	wire[7:0]	data_p;
	wire[7:0]	data_n;

	for(genvar g=0; g<8; g=g+1) begin

		IBUFDS_DIFF_OUT #(
			.DIFF_TERM("TRUE"),
			.IBUF_LOW_PWR("FALSE"),
			.IOSTANDARD("LVDS")
		) ibuf (
			.I(pod_data_p_remapped[g]),
			.IB(pod_data_n_remapped[g]),
			.O(data_p[g]),
			.OB(data_n[g])
		);

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Delay line calibration

	IODelayCalibration cal(.refclk(clk_400mhz));

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Synchronize armed flag into fast clock domain

	//For now, just use RAM ready flag
	wire	armed_sync;
	wire	armed;

	ThreeStageSynchronizer #(
		.INIT(0),
		.IN_REG(0)
	) sync_armed(
		.clk_in(clk_312p5mhz),
		.din(armed),
		.clk_out(clk_312p5mhz),
		.dout(armed_sync)
	);

	assign armed = ram_ready;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Input delay lines

	wire[7:0] data_p_delayed;
	wire[7:0] data_n_delayed;

	IODelayBlock #(
		.WIDTH(8),
		.CAL_FREQ(400),
		.INPUT_DELAY(00),
		.DIRECTION("IN"),
		.IS_CLOCK(0)
	) idelay_p (
		.i_pad(data_p),
		.i_fabric(data_p_delayed),

		.o_pad(),
		.o_fabric(),

		.input_en(1'b1)
	);

	IODelayBlock #(
		.WIDTH(8),
		.CAL_FREQ(400),
		.INPUT_DELAY(200),
		.DIRECTION("IN"),
		.IS_CLOCK(0)
	) idelay_n (
		.i_pad(data_n),
		.i_fabric(data_n_delayed),

		.o_pad(),
		.o_fabric(),

		.input_en(1'b1)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Initial oversampling: 4 phases of 625 MHz = 2.5 Gsps, times P and N = 5 Gsps

	wire[3:0] deser_p[7:0];
	wire[3:0] deser_n[7:0];

	logic	iserdes_rst = 0;

	//Sampling order is Q1 Q3 Q2 Q4 in oversampling mode
	//then we interleave with negative before positive
	for(genvar g=0; g<8; g=g+1) begin
		ISERDESE2 #(
			.DATA_RATE("DDR"),
			.DATA_WIDTH("4"),
			.DYN_CLKDIV_INV_EN("FALSE"),
			.DYN_CLK_INV_EN("FALSE"),
			.INTERFACE_TYPE("OVERSAMPLE"),
			.NUM_CE(1),
			.OFB_USED("FALSE"),
			.SERDES_MODE("MASTER"),
			.IOBDELAY("BOTH")
		) iserdes_p (
			.Q1(deser_p[g][0]),
			.Q2(deser_p[g][2]),
			.Q3(deser_p[g][1]),
			.Q4(deser_p[g][3]),
			.Q5(),
			.Q6(),
			.Q7(),
			.Q8(),
			.O(),
			.SHIFTOUT1(),
			.SHIFTOUT2(),
			.D(),
			.DDLY(data_p_delayed[g]),
			.CLK(clk_625mhz_io_0),
			.CLKB(!clk_625mhz_io_0),
			.CE1(1'b1),
			.CE2(1'b1),
			.RST(iserdes_rst),
			.CLKDIV(),
			.CLKDIVP(1'b0),
			.OCLK(clk_625mhz_io_90),
			.OCLKB(!clk_625mhz_io_90),
			.BITSLIP(1'b0),
			.SHIFTIN1(),
			.SHIFTIN2(),
			.OFB(),
			.DYNCLKDIVSEL(),
			.DYNCLKSEL()
		);

		ISERDESE2 #(
			.DATA_RATE("DDR"),
			.DATA_WIDTH("4"),
			.DYN_CLKDIV_INV_EN("FALSE"),
			.DYN_CLK_INV_EN("FALSE"),
			.INTERFACE_TYPE("OVERSAMPLE"),
			.NUM_CE(1),
			.OFB_USED("FALSE"),
			.SERDES_MODE("MASTER"),
			.IOBDELAY("BOTH")
		) iserdes_n (
			.Q1(deser_n[g][0]),
			.Q2(deser_n[g][2]),
			.Q3(deser_n[g][1]),
			.Q4(deser_n[g][3]),
			.Q5(),
			.Q6(),
			.Q7(),
			.Q8(),
			.O(),
			.SHIFTOUT1(),
			.SHIFTOUT2(),
			.D(),
			.DDLY(data_n_delayed[g]),
			.CLK(clk_625mhz_io_0),
			.CLKB(!clk_625mhz_io_0),
			.CE1(1'b1),
			.CE2(1'b1),
			.RST(iserdes_rst),
			.CLKDIV(),
			.CLKDIVP(1'b0),
			.OCLK(clk_625mhz_io_90),
			.OCLKB(!clk_625mhz_io_90),
			.BITSLIP(1'b0),
			.SHIFTIN1(),
			.SHIFTIN2(),
			.OFB(),
			.DYNCLKDIVSEL(),
			.DYNCLKSEL()
		);

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Capturing and processing

	wire[7:0]		fifo_rd_en;
	wire[7:0]		fifo_rd_underflow;
	wire[127:0]		fifo_rd_data[7:0];
	wire[9:0]		fifo_rd_size[7:0];
	logic[7:0]		fifo_half_full		= 0;
	logic[7:0]		fifo_burst_ready	= 0;

	logic[31:0] 	count_1hz	= 0;
	logic			pps			= 0;

	logic[31:0]		overflows[7:0];

	logic[7:0]		channel_flushed	= 0;

	for(genvar g=0; g<8; g=g+1) begin

		////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		// Capture into the slow clock domain

		wire[7:0]	p_merged;
		wire[7:0]	n_merged;

		LogicPodSampling sampler (
			.clk_625mhz_fabric(clk_625mhz_fabric),
			.clk_312p5mhz(clk_312p5mhz),
			.deser_p(deser_p[g]),
			.deser_n(deser_n[g]),
			.p_out(p_merged),
			.n_out(n_merged)
		);

		////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		// Merge P/N into a single 16-bit stream, inverting as necessary

		//Parallel digitized output
		la_sample_t		samples;

		//N has a larger delay than P, so it's logically earlier in the stream
		always_ff @(posedge clk_312p5mhz) begin
			for(integer i=0; i<8; i=i+1) begin
				samples.bits[i*2 + 1]	<= !n_merged[i] ^ LANE_INVERT_REMAPPED[g];
				samples.bits[i*2]		<= p_merged[i] ^ LANE_INVERT_REMAPPED[g];
			end
		end

		////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		// RLE compression

		wire		compress_out_valid;
		wire		compress_out_format;
		wire[15:0]	compress_out_data;
		wire		deser_flush;

		LogicPodCompression compressor(
			.clk(clk_312p5mhz),
			.din(samples.bits),

			.en(capture_en),
			.rst(trig_rst),
			.flush(capture_flush),
			.flush_done(deser_flush),

			.out_valid(compress_out_valid),
			.out_format(compress_out_format),
			.out_data(compress_out_data)
		);

		////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		// CDC FIFO for deserialized compressor output

		wire		fifo_wr;
		wire[127:0]	fifo_wdata;

		wire		deser_flush_done;

		LogicPodDeserialization deserialization(
			.clk(clk_312p5mhz),
			.compress_out_valid(compress_out_valid),
			.compress_out_format(compress_out_format),
			.compress_out_data(compress_out_data),
			.flush(deser_flush),
			.flush_done(deser_flush_done),
			.fifo_wr(fifo_wr),
			.fifo_wdata(fifo_wdata)
		);

		wire		deser_flush_done_sync;
		PulseSynchronizer sync_deser_flush_done(
			.clk_a(clk_312p5mhz),
			.pulse_a(deser_flush_done),
			.clk_b(clk_ram_2x),
			.pulse_b(deser_flush_done_sync)
		);

		always_ff @(posedge clk_ram_2x) begin
			if(trig_rst_arbiter_2x)
				channel_flushed[g]		<= 0;
			if(deser_flush_done_sync)
				channel_flushed[g]		<= 1;
		end

		wire	overflow;
		CrossClockFifo #(
			.WIDTH(128),
			.DEPTH(512),
			.USE_BLOCK(1),
			.OUT_REG(2)
		) cdc_fifo (

			//Write side: push in at full rate.
			//No flow control really possible here as the data can't be slowed down.
			//The FIFO will just start dropping samples if you push too fast.
			//TODO: add error flag to detect when this happens
			.wr_clk(clk_312p5mhz),
			.wr_en(fifo_wr),
			.wr_data(fifo_wdata),
			.wr_size(),
			.wr_full(),
			.wr_overflow(overflow),
			.wr_reset(trig_rst),

			//Read side
			.rd_clk(clk_ram_2x),
			.rd_en(fifo_rd_en[g]),
			.rd_data(fifo_rd_data[g]),
			.rd_size(fifo_rd_size[g]),
			.rd_empty(),
			.rd_underflow(fifo_rd_underflow[g]),
			.rd_reset(trig_rst_arbiter_2x)
		);

		always_ff @(posedge clk_ram_2x) begin
			fifo_half_full[g] 	<= (fifo_rd_size[g] > 256);
			fifo_burst_ready[g] <= (fifo_rd_size[g] >= 4);
		end

		//Performance counters
		logic[31:0] running_overflows = 0;
		always_ff @(posedge clk_312p5mhz) begin
			if(overflow)
				running_overflows	<= running_overflows + 1;

			if(pps) begin
				overflows[g]		<= running_overflows;
				running_overflows	<= 0;
			end

			if(trig_rst)
				running_overflows	<= 0;
		end

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Performance counter stuff

	localparam CLK_HZ = 312500000;

	always_ff @(posedge clk_312p5mhz) begin
		count_1hz		<= count_1hz + 1;
		pps				<= 0;

		if(count_1hz == (CLK_HZ - 1) ) begin
			pps			<= 1;
			count_1hz	<= 0;
		end

	end

	vio_1 vio(
		.clk(clk_312p5mhz),
		.probe_in0(overflows[0]),
		.probe_in1(overflows[1]),
		.probe_in2(overflows[2]),
		.probe_in3(overflows[3]),
		.probe_in4(overflows[4]),
		.probe_in5(overflows[5]),
		.probe_in6(overflows[6]),
		.probe_in7(overflows[7])
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Arbitration from all 8 channels to the output buffer

	wire		data_fifo_wr_en;
	wire[127:0]	data_fifo_wr_data;

	wire		addr_fifo_wr_en;
	wire[28:0]	addr_fifo_wr_data;

	wire[9:0]	data_fifo_wr_size;
	wire[7:0]	addr_fifo_wr_size;

	logic		channel_flushed_all = 0;
	always_ff @(posedge clk_ram_2x) begin
		channel_flushed_all <= (channel_flushed == 8'hff);
	end

	LogicPodArbiter #(
		.POD_NUMBER(POD_NUMBER)
	) arbiter(
		.clk_ram_2x(clk_ram_2x),
		.rst(trig_rst_arbiter_2x),
		.flush(channel_flushed_all),
		.flush_complete(flush_complete),
		.fifo_rd_en(fifo_rd_en),
		.fifo_rd_underflow(fifo_rd_underflow),
		.fifo_rd_data(fifo_rd_data),
		.fifo_rd_size(fifo_rd_size),
		.fifo_half_full(fifo_half_full),
		.fifo_burst_ready(fifo_burst_ready),
		.data_fifo_wr_en(data_fifo_wr_en),
		.data_fifo_wr_data(data_fifo_wr_data),
		.addr_fifo_wr_en(addr_fifo_wr_en),
		.addr_fifo_wr_data(addr_fifo_wr_data),
		.data_fifo_wr_size(data_fifo_wr_size),
		.addr_fifo_wr_size(addr_fifo_wr_size),

		.ptr_rd_en(ptr_rd_en),
		.ptr_rd_addr(ptr_rd_addr),
		.ptr_rd_data(ptr_rd_data)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// FIFOs on output of mux path

	SingleClockFifo #(
		.WIDTH(128),
		.DEPTH(512),
		.USE_BLOCK(1),
		.OUT_REG(2)
	) data_fifo (

		.clk(clk_ram_2x),
		.full(),
		.overflow(),
		.reset(trig_rst_arbiter_2x),
		.empty(),
		.underflow(),

		.wr(data_fifo_wr_en),
		.din(data_fifo_wr_data),
		.wsize(data_fifo_wr_size),

		.rd(ram_data_rd_en),
		.dout(ram_data_rd_data),
		.rsize(ram_data_rd_size)
	);

	SingleClockFifo #(
		.WIDTH(29),
		.DEPTH(128),
		.USE_BLOCK(1),
		.OUT_REG(2)
	) addr_fifo (

		.clk(clk_ram_2x),
		.full(),
		.overflow(),
		.reset(trig_rst_arbiter_2x),
		.empty(),
		.underflow(),

		.wr(addr_fifo_wr_en),
		.din(addr_fifo_wr_data),
		.wsize(addr_fifo_wr_size),

		.rd(ram_addr_rd_en),
		.dout(ram_addr_rd_data),
		.rsize(ram_addr_rd_size)
	);

endmodule
