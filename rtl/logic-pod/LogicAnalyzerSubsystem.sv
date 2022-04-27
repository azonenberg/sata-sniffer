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
	output wire[7:0]	la1_ram_addr_rd_size
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
	// High speed capture/compression datapaths

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
		.ram_addr_rd_size(la0_ram_addr_rd_size)
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
		.ram_addr_rd_size(la1_ram_addr_rd_size)
		);

endmodule
