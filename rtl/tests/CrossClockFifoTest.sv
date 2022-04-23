`default_nettype none
`timescale 1ns/1ps
module CrossClockFifoTest(
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Clock generation

	logic clk_fast = 0;
	always begin
		#1;
		clk_fast = 1;
		#1;
		clk_fast = 0;
	end

	logic clk_slow = 0;
	always begin
		#1.5;
		clk_slow = 1;
		#1.5;
		clk_slow = 0;
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// The DUT

	CrossClockFifo fifo(
		.wr_clk(clk_fast),
		.wr_en(1'b1),
		.wr_data(16'hdead),
		.wr_full(),
		.wr_size(),
		.wr_overflow(),
		.wr_reset(1'b0),
		.rd_clk(clk_slow),
		.rd_en(1'b1),
		.rd_data(),
		.rd_size(),
		.rd_empty(),
		.rd_underflow(),
		.rd_reset(1'b0)
	);

endmodule
