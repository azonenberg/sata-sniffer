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

module LogicAnalyzerSubsystem(

	//Global clocks
	input wire			clk_125mhz,
	input wire			clk_400mhz,
	input wire			clk_ram,
	input wire			clk_ram_2x,

	//Top level pins
	input wire[7:0]		la0_p,
	input wire[7:0]		la0_n,
	input wire			la0_present_n,
	input wire			la0_12v_fault_n,
	output wire			la0_12v_en,
	input wire			la0_uart_rx,
	output wire			la0_uart_tx,

	input wire[7:0]		la1_p,
	input wire[7:0]		la1_n,
	input wire			la1_present_n,
	input wire			la1_12v_fault_n,
	output wire			la1_12v_en,
	input wire			la1_uart_rx,
	output wire			la1_uart_tx,

	//DRAM status lines
	input wire			ram_ready,

	//Status outputs
	output wire			la0_align_done,
	output wire			la1_align_done,

	//Ports to top level DDR arbiter
	input wire			la0_ram_data_rd_en,
	output wire[127:0]	la0_ram_data_rd_data,
	output wire[9:0]	la0_ram_data_rd_size,
	input wire			la0_ram_addr_rd_en,
	output wire[28:0]	la0_ram_addr_rd_data,
	output wire[7:0]	la0_ram_addr_rd_size,

	input wire			la1_ram_data_rd_en,
	output wire[127:0]	la1_ram_data_rd_data,
	output wire[9:0]	la1_ram_data_rd_size,
	input wire			la1_ram_addr_rd_en,
	output wire[28:0]	la1_ram_addr_rd_data,
	output wire[7:0]	la1_ram_addr_rd_size,

	input wire			readback_ram_addr_rd_en,
	output wire[28:0]	readback_ram_addr_rd_data,
	output wire[7:0]	readback_ram_addr_rd_size,

	//Trigger controls (clk_ram domain)
	input wire			trigger_arm,

	//Status/control lines to top level memory arbiter
	output wire			trig_rst_arbiter,			//clk_ram
	output wire			capture_flush_arbiter,

	output wire			trig_rst_arbiter_2x,		//clk_ram_2x
	output wire			flush_arbiter_2x,
	output wire			la0_flush_complete,
	output wire			la1_flush_complete,
	input wire			flush_done
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Hotswap power control / UART

	LogicPodControl la0_ctl(
		.clk_125mhz(clk_125mhz),
		.pod_present_n(la0_present_n),
		.pod_power_fault_n(la0_12v_fault_n),
		.pod_power_en(la0_12v_en),
		.uart_rx(la0_uart_rx),
		.uart_tx(la0_uart_tx)
		);

	LogicPodControl la1_ctl(
		.clk_125mhz(clk_125mhz),
		.pod_present_n(la1_present_n),
		.pod_power_fault_n(la1_12v_fault_n),
		.pod_power_en(la1_12v_en),
		.uart_rx(la1_uart_rx),
		.uart_tx(la1_uart_tx)
		);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Clock generation

	wire	la0_clk_312p5mhz;
	wire	la0_clk_625mhz_fabric;
	wire	la0_clk_625mhz_io_0;
	wire	la0_clk_625mhz_io_90;

	wire	la1_clk_312p5mhz;
	wire	la1_clk_625mhz_fabric;
	wire	la1_clk_625mhz_io_0;
	wire	la1_clk_625mhz_io_90;

	LogicPodClocking la0_clocks(
		.clk_125mhz(clk_125mhz),

		.clk_312p5mhz(la0_clk_312p5mhz),
		.clk_625mhz_io_0(la0_clk_625mhz_io_0),
		.clk_625mhz_io_90(la0_clk_625mhz_io_90),
		.clk_625mhz_fabric(la0_clk_625mhz_fabric),

		.pll_lock(),
		.align_done(la0_align_done)
	);

	LogicPodClocking la1_clocks(
		.clk_125mhz(clk_125mhz),

		.clk_312p5mhz(la1_clk_312p5mhz),
		.clk_625mhz_io_0(la1_clk_625mhz_io_0),
		.clk_625mhz_io_90(la1_clk_625mhz_io_90),
		.clk_625mhz_fabric(la1_clk_625mhz_fabric),

		.pll_lock(),
		.align_done(la1_align_done)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Trigger stuff

	wire	arm_req;

	wire	trig_rst_la0;
	wire	capture_en_la0;
	wire	capture_flush_la0;

	wire	trig_rst_la1;
	wire	capture_en_la1;
	wire	capture_flush_la1;

	LogicTriggering trig(
		.clk_ram(clk_ram),
		.clk_ram_2x(clk_ram_2x),
		.la0_clk_312p5mhz(la0_clk_312p5mhz),
		.la1_clk_312p5mhz(la1_clk_312p5mhz),
		.arm_req(arm_req),

		.trig_rst_la0(trig_rst_la0),
		.capture_en_la0(capture_en_la0),
		.capture_flush_la0(capture_flush_la0),

		.trig_rst_la1(trig_rst_la1),
		.capture_en_la1(capture_en_la1),
		.capture_flush_la1(capture_flush_la1),

		.trig_rst_arbiter_2x(trig_rst_arbiter_2x),
		.capture_flush_arbiter_2x(flush_arbiter_2x),

		.trig_rst_arbiter(trig_rst_arbiter),
		.capture_flush_arbiter(capture_flush_arbiter)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// High speed capture/compression datapaths

	wire		ptr_rd_en;
	wire[3:0]	ptr_rd_addr;
	wire[28:0]	ptr_rd_data[1:0];

	LogicPodDatapath #(
		.LANE_INVERT(8'b10011100),
		.POD_NUMBER(0)
	) la0_path (
		.clk_312p5mhz(la0_clk_312p5mhz),
		.clk_400mhz(clk_400mhz),
		.clk_625mhz_io_0(la0_clk_625mhz_io_0),
		.clk_625mhz_io_90(la0_clk_625mhz_io_90),
		.clk_625mhz_fabric(la0_clk_625mhz_fabric),
		.pod_data_p(la0_p),
		.pod_data_n(la0_n),

		.clk_ram_2x(clk_ram_2x),
		.ram_ready(ram_ready),
		.ram_data_rd_en(la0_ram_data_rd_en),
		.ram_data_rd_data(la0_ram_data_rd_data),
		.ram_data_rd_size(la0_ram_data_rd_size),
		.ram_addr_rd_en(la0_ram_addr_rd_en),
		.ram_addr_rd_data(la0_ram_addr_rd_data),
		.ram_addr_rd_size(la0_ram_addr_rd_size),

		.trig_rst(trig_rst_la0),
		.trig_rst_arbiter_2x(trig_rst_arbiter_2x),
		.capture_en(capture_en_la0),
		.capture_flush(capture_flush_la0),

		.flush_complete(la0_flush_complete),

		.ptr_rd_en(ptr_rd_en & !ptr_rd_addr[3]),
		.ptr_rd_addr(ptr_rd_addr[2:0]),
		.ptr_rd_data(ptr_rd_data[0])
		);

	LogicPodDatapath #(
		.LANE_INVERT(8'b00000110),
		.POD_NUMBER(1)
	) la1_path (
		.clk_312p5mhz(la1_clk_312p5mhz),
		.clk_400mhz(clk_400mhz),
		.clk_625mhz_io_0(la1_clk_625mhz_io_0),
		.clk_625mhz_io_90(la1_clk_625mhz_io_90),
		.clk_625mhz_fabric(la1_clk_625mhz_fabric),
		.pod_data_p(la1_p),
		.pod_data_n(la1_n),

		.clk_ram_2x(clk_ram_2x),
		.ram_ready(ram_ready),
		.ram_data_rd_en(la1_ram_data_rd_en),
		.ram_data_rd_data(la1_ram_data_rd_data),
		.ram_data_rd_size(la1_ram_data_rd_size),
		.ram_addr_rd_en(la1_ram_addr_rd_en),
		.ram_addr_rd_data(la1_ram_addr_rd_data),
		.ram_addr_rd_size(la1_ram_addr_rd_size),

		.trig_rst(trig_rst_la1),
		.trig_rst_arbiter_2x(trig_rst_arbiter_2x),
		.capture_en(capture_en_la1),
		.capture_flush(capture_flush_la1),

		.flush_complete(la1_flush_complete),

		.ptr_rd_en(ptr_rd_en & ptr_rd_addr[3]),
		.ptr_rd_addr(ptr_rd_addr[2:0]),
		.ptr_rd_data(ptr_rd_data[1])
		);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Readback stuff

	wire[28:0] ptr_rd_data_muxed;
	assign ptr_rd_data_muxed = ptr_rd_data[ptr_rd_addr[3]];

	LogicAnalyzerReadback readback(
		.clk_ram_2x(clk_ram_2x),

		.rst(trig_rst_arbiter_2x),
		.flush_done(flush_done),

		.ptr_rd_en(ptr_rd_en),
		.ptr_rd_addr(ptr_rd_addr),
		.ptr_rd_data(ptr_rd_data_muxed),

		.ram_addr_rd_en(readback_ram_addr_rd_en),
		.ram_addr_rd_data(readback_ram_addr_rd_data),
		.ram_addr_rd_size(readback_ram_addr_rd_size),
		.ram_rd_data_start(1'b0),
		.ram_rd_data_valid(1'b0),
		.ram_rd_data(128'b0)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Debug VIO and ILAs

	vio_2 vio(
		.clk(clk_ram_2x),
		.probe_out0(arm_req)
	);

	/*
	ila_0 ila(
		.clk(clk_ram_2x),
		.probe0(trig_rst_arbiter_2x),
		.probe1(flush_arbiter_2x),
		.probe2(la0_flush_complete),
		.probe3(la1_flush_complete),
		.probe4(flush_done),

		//new here
		.probe5(ptr_rd_en),
		.probe6(ptr_rd_addr),
		.probe7(ptr_rd_data_muxed),
		.probe8(readback.state),
		.probe9(readback_ram_addr_rd_en),
		.probe10(readback_ram_addr_rd_data),
		.probe11(readback_ram_addr_rd_size),
		.probe12(readback.dram_rd_en),
		.probe13(readback.dram_rd_addr)
	);
	*/

endmodule
