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
	@brief Compression engine for a single logic analyzer data lane

	Input: 16 bits per clock.

	Output: a sequence of 17-bit chunks.

	Chunk format:
		1 bit	Format

		Format 0 - incompressible
			16 bits of verbatim sample data

		Format 1 - compressible
			1 bit value A
			7 bit repetition count A
			1 bit value B
			7 bit repetition count B

	Each input block maps to at most one output chunk.

	Worst case: 1: 1.0625 overhead (16 -> 17 bits)
	Best case: 14.94 : 1 compression (254 -> 17 bits)
 */
module LogicPodCompression(
	input wire			clk,

	//Input data (valid every clock)
	input wire[15:0]	din,

	//Output data bus
	output logic		out_valid	= 0,
	output logic		out_format	= 0,
	output logic[15:0]	out_data	= 0
);

	typedef enum logic
	{
		INCOMPRESSIBLE	= 1'b0,
		COMPRESSIBLE 	= 1'b1
	} mode_t;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Pipeline stage 1: Register input and search for stage2_edges

	logic[15:0] stage2_din		= 0;
	logic[14:0]	stage2_edges	= 0;

	always_ff @(posedge clk) begin

		//Register input
		stage2_din	<= din;

		//Find stage2_edges
		stage2_edges	<= { din[15:1] ^ din[14:0] };

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Pipeline stage 2: search for smaller runs

	logic[15:0] stage3_din = 0;

	logic		stage3_incompressible	= 0;

	logic		stage3_run_valid_1;
	logic[1:0]	stage3_run_value;
	logic[4:0]	stage3_run_count;	//length of the first run (second is implicitly 16-this if we're compressible)

	//There can be at most two runs in an input block in order to compress better than just copying it.
	//One run: input is constant 0 or 1
	//Two runs: input starts with 0 then transitions to 1, or starts with 1 and transitions to 0.
	//Rather than doing fancy loop logic, just enumerate all states that we can compress!
	always_ff @(posedge clk) begin

		//Register input
		stage3_din	<= stage2_din;

		//Default state: assume we have two runs
		stage3_incompressible	<= 0;
		stage3_run_valid_1		<= 1;

		//First run value is always the MSB of the input
		//If we have two runs, it will be the LSB of the input
		stage3_run_value[0] 	<= stage2_din[15];
		stage3_run_value[1] 	<= stage2_din[0];

		case(stage2_edges)

			//If no toggles at all, easy mode: single run
			15'h0000: begin
				stage3_run_count			<= 16;
				stage3_run_valid_1			<= 0;
			end

			//If there is exactly one toggle, report the location.
			15'h0001:	stage3_run_count	<= 15;
			15'h0002:	stage3_run_count	<= 14;
			15'h0004:	stage3_run_count	<= 13;
			15'h0008:	stage3_run_count	<= 12;
			15'h0010:	stage3_run_count	<= 11;
			15'h0020:	stage3_run_count	<= 10;
			15'h0040:	stage3_run_count	<= 9;
			15'h0080:	stage3_run_count	<= 8;
			15'h0100:	stage3_run_count	<= 7;
			15'h0200:	stage3_run_count	<= 6;
			15'h0400:	stage3_run_count	<= 5;
			15'h0800:	stage3_run_count	<= 4;
			15'h1000:	stage3_run_count	<= 3;
			15'h2000:	stage3_run_count	<= 2;
			15'h4000:	stage3_run_count	<= 1;

			//If more than one, incompressible.
			default: begin
				stage3_incompressible 		<= 1;
			end

		endcase

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Pipeline stage 3: compression logic

	logic		active_run1_valid	= 0;
	logic		active_run1_value	= 0;
	logic[6:0]	active_run1_count	= 0;

	logic		active_run2_valid	= 0;
	logic		active_run2_value	= 0;
	logic[6:0]	active_run2_count	= 0;

	logic		active_block_valid	= 0;
	logic[15:0]	active_block_data	= 0;

	always_ff @(posedge clk) begin

		out_valid	<= 0;
		out_data	<= 0;

		//If we have a pending incompressible block, output that.
		if(active_block_valid) begin
			active_block_valid	<= 0;

			out_valid			<= 1;
			out_format			<= INCOMPRESSIBLE;
			out_data			<= active_block_data;

			//We cannot have both incompressible and compressible data pending
			//Therefore, no need to check for pending compressible data.

			//Save new incompressible block
			if(stage3_incompressible) begin
				active_block_valid	<= 1;
				active_block_data	<= stage3_din;
			end

			//Save new compressible block
			else begin
				active_run1_valid	<= 1;
				active_run1_value	<= stage3_run_value[0];
				active_run1_count	<= stage3_run_count;

				active_run2_value	<= stage3_run_valid_1;
				active_run2_value	<= stage3_run_value[1];
				active_run2_count	<= 16 - stage3_run_count;
			end

		end

		//New incompressible block?
		else if(stage3_incompressible) begin

			//If we have any pending runs, output them.
			if(active_run1_valid) begin

				//Output the run(s)
				out_valid			<= 1;
				out_format			<= COMPRESSIBLE;
				out_data			<= { active_run1_value, active_run1_count, active_run2_value, active_run2_count };

				//Clear pending
				active_run1_valid	<= 0;
				active_run1_value	<= 0;
				active_run1_count	<= 0;

				active_run2_valid	<= 0;
				active_run2_value	<= 0;
				active_run2_count	<= 0;

				//Save the incompressible block for next cycle
				active_block_valid	<= 1;
				active_block_data	<= stage3_din;

			end

			//No pending data. Output the block immediately.
			else begin
				out_valid			<= 1;
				out_format			<= INCOMPRESSIBLE;
				out_data			<= stage3_din;
			end

		end

		//New compressible block
		else begin

			//We have one existing run
			if(active_run1_valid && !active_run2_valid) begin

				//One new run. We will not be outputting anything this cycle regardless.
				if(!stage3_run_valid_1) begin

 					//Same value as existing run? Extend it.
					if(active_run1_value == stage3_run_value[0]) begin

						//Handle overflow
						if(active_run1_count > 111) begin
							active_run1_count 	<= 127;
							active_run2_valid	<= 1;
							active_run2_value	<= active_run1_value;
							active_run2_count	<= (active_run1_count + stage3_run_count) - 127;
						end

						//No overflow, just extend the run
						else
							active_run1_count	<= active_run1_count + stage3_run_count;

					end

					//Nope, create a new run
					else begin
						active_run2_count	<= 16;
						active_run2_valid	<= 1;
						active_run2_value	<= stage3_run_value[0];
					end

				end	//end one new run

				//Two new runs. We might output something.
				else begin

					//First new run same value as existing first run? Extend it.
					if(active_run1_value == stage3_run_value[0]) begin

						//Handle overflow
						if(active_run1_count > 111) begin

							//We have three runs total: existing first run which filled up,
							//second run consisting of the overflow plus our new first run,
							//and a third run consisting of our new second run.

							//Output the first and second
							out_valid			<= 1;
							out_format			<= COMPRESSIBLE;
							out_data[15]		<= active_run1_value;
							out_data[14:8]		<= 127;
							out_data[7]			<= active_run1_value;
							out_data[6:0]		<= (active_run1_count + stage3_run_count) - 127;

							//and save the third
							active_run1_count	<= 16 - stage3_run_count;
							active_run1_valid	<= 1;
							active_run1_value	<= stage3_run_value[1];

							//No fourth run to save
							active_run2_valid	<= 0;
							active_run2_value	<= 0;
							active_run2_count	<= 0;

						end

						//No overflow, just extend the first run and create a second
						else begin

							active_run1_count	<= active_run1_count + stage3_run_count;

							active_run2_count	<= 16 - stage3_run_count;
							active_run2_valid	<= 1;
							active_run2_value	<= stage3_run_value[1];

						end

					end

				end	//end two new runs

			end	//end one existing run

			//We have two existing runs.
			else begin

				//One new run. Output if we're overflowing or mismatch
				if(!stage3_run_valid_1) begin

					//Same value as existing run? Extend it.
					if(active_run2_value == stage3_run_value[0]) begin

						//Handle overflow
						if(active_run2_count > 111) begin

							//Output the existing run plus the extender and save the new one
							out_valid			<= 1;
							out_format			<= COMPRESSIBLE;
							out_data[15]		<= active_run1_value;
							out_data[14:8]		<= active_run1_count;
							out_data[7]			<= active_run2_value;
							out_data[6:0]		<= (active_run2_count + stage3_run_count) - 127;

							active_run1_valid	<= 1;
							active_run1_count 	<= (active_run1_count + stage3_run_count) - 127;
							active_run1_value	<= stage3_run_value[0];

							active_run2_valid	<= 0;
							active_run2_value	<= 0;
							active_run2_count	<= 0;
						end

						//No overflow, just extend the run
						else
							active_run2_count	<= active_run2_count + stage3_run_count;

					end

					//Nope. Output the existing runs and start a new one.
					else begin
						out_valid			<= 1;
						out_format			<= COMPRESSIBLE;
						out_data			<= { active_run1_value, active_run1_count, active_run2_value, active_run2_count };

						active_run1_count	<= 16;
						active_run1_valid	<= 1;
						active_run1_value	<= stage3_run_value[0];

						active_run2_valid	<= 0;
						active_run2_value	<= 0;
						active_run2_count	<= 0;
					end

				end	//end one new run

				//Two new runs. Output regardless.
				//We could try to merge, but for now sacrifice compression ratio slightly to improve timing.
				else begin

					//Output the run(s)
					out_valid			<= 1;
					out_format			<= COMPRESSIBLE;
					out_data			<= { active_run1_value, active_run1_count, active_run2_value, active_run2_count };

					//Save new blocks
					active_run1_valid	<= 1;
					active_run1_value	<= stage3_run_value[0];
					active_run1_count	<= stage3_run_count;

					active_run2_value	<= stage3_run_valid_1;
					active_run2_value	<= stage3_run_value[1];
					active_run2_count	<= 16 - stage3_run_count;

				end	//end two new runs

			end	//end two existing runs

		end

	end

endmodule
