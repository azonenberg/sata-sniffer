module LogicPodCompressionTest();

	logic clk = 0;
	always begin
		#1.6;
		clk = 1;
		#1.6;
		clk = 0;
	end

	logic[15:0] din = 0;

	LogicPodCompression dut(
		.clk(clk),
		.din(din)
		);

	logic[7:0] state = 0;

	always_ff @(posedge clk) begin
		state <= state + 1;

		case(state)
			0: din <= 16'h0000;
			1: din <= 16'h0000;
			2: din <= 16'hffff;
			3: din <= 16'h0000;
			4: din <= 16'h00ff;
			5: din <= 16'hf0f0;
			6: din <= 16'h00ff;
			7: din <= 16'hffff;
			8: din <= 16'hffff;
			9: din <= 16'hff00;
			10: din <= 16'h0055;
			11: din <= 16'h0;
			12: $finish;
		endcase
	end

endmodule
