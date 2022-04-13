module CrossClockSampling();

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Clock synthesis

	logic clk_fast = 0;
	logic clk_slow = 0;

	always begin
		#0.8;
		clk_fast = 1;
		#0.8;
		clk_fast = 0;
	end

	always begin
		#1.6;
		clk_slow = 1;
		#1.6;
		clk_slow = 0;

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Toggle FF in fast clock domain, then sample in slow

	logic toggle = 0;
	logic toggle2 = 0;

	always_ff @(posedge clk_fast) begin
		toggle <= !toggle;
		if(toggle)
			toggle2	<= !toggle2;
	end

	logic toggle_slow		= 0;
	logic toggle_slow_ff	= 0;
	always_ff @(posedge clk_slow) begin
		toggle_slow		<= toggle;
		toggle_slow_ff	<= toggle_slow;
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Write data to two register sets in fast clock domain

	logic[3:0] set0 	= 0;
	logic[3:0] set0_ff	= 0;

	logic[3:0] set1 	= 0;
	logic[3:0] set1_ff	= 0;

	always_ff @(posedge clk_fast) begin

		if(toggle2) begin

		end

	end

endmodule
