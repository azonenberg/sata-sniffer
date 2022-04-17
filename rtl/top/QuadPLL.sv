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

/**
	@brief SERDES quad PLL
 */
module QuadPLL(
	input wire		gtx_refclk_156_p,
	input wire		gtx_refclk_156_n,
	input wire		gtx_refclk_200_p,
	input wire		gtx_refclk_200_n,

	input wire		clk_125mhz,

	output wire		qpll_clk,
	output wire		qpll_refclk,
	output wire		qpll_lock,
	output wire		qpll_refclk_lost
);

	wire	gtx_refclk_156;
	wire	gtx_refclk_200;

	IBUFDS_GTE2 clk_buf_refclk_156(
		.I(gtx_refclk_156_p),
		.IB(gtx_refclk_156_n),
		.CEB(1'b0),
		.O(gtx_refclk_156),
		.ODIV2()
	);

	IBUFDS_GTE2 clk_buf_refclk_200(
		.I(gtx_refclk_200_p),
		.IB(gtx_refclk_200_n),
		.CEB(1'b0),
		.O(gtx_refclk_200),
		.ODIV2()
	);

	GTXE2_COMMON #(
		//Magic numbers from transceivers wizard
		.BIAS_CFG                               (64'h0000040000001000),
		.COMMON_CFG                             (32'h00000000),
		.QPLL_CFG                               (27'h0680181),
		.QPLL_CLKOUT_CFG                        (4'b0000),
		.QPLL_COARSE_FREQ_OVRD                  (6'b010000),
		.QPLL_COARSE_FREQ_OVRD_EN               (1'b0),
		.QPLL_CP                                (10'b0000011111),
		.QPLL_CP_MONITOR_EN                     (1'b0),
		.QPLL_DMONITOR_SEL                      (1'b0),
		.QPLL_FBDIV                             (10'b0101000000),
		.QPLL_FBDIV_MONITOR_EN                  (1'b0),
		.QPLL_FBDIV_RATIO                       (1'b0),
		.QPLL_INIT_CFG                          (24'h000006),
		.QPLL_LOCK_CFG                          (16'h21E8),
		.QPLL_LPF                               (4'b1111),
		.QPLL_REFCLK_DIV                        (1)
	) serdes_common(
		.DRPADDR(8'b0),
		.DRPCLK(clk_125mhz),
		.DRPDI(16'b0),
		.DRPEN(1'b0),
		.DRPWE(1'b0),
		.DRPRDY(),
		.DRPDO(),
		.REFCLKOUTMONITOR(),
		.GTGREFCLK(1'b0),
		.GTNORTHREFCLK0(1'b0),
		.GTNORTHREFCLK1(1'b0),
		.GTSOUTHREFCLK0(1'b0),
		.GTSOUTHREFCLK1(1'b0),
		.GTREFCLK0(gtx_refclk_156),
		.GTREFCLK1(gtx_refclk_200),
		.QPLLDMONITOR(),
		.QPLLFBCLKLOST(),
		.QPLLLOCK(qpll_lock),
		.QPLLLOCKDETCLK(clk_125mhz),
		.QPLLLOCKEN(1'b1),
		.QPLLOUTCLK(qpll_clk),
		.QPLLOUTREFCLK(qpll_refclk),
		.QPLLOUTRESET(1'b0),
		.QPLLPD(1'b0),
		.QPLLREFCLKLOST(qpll_refclk_lost),
		.QPLLREFCLKSEL(3'b001),			//use REFCLK0 (156.25 MHz)
		.QPLLRESET(1'b0),
		.QPLLRSVD1(16'h0),
		.QPLLRSVD2(5'b11111),
		.BGBYPASSB(1'b1),
		.BGMONITORENB(1'b1),
		.BGPDB(1'b1),
		.BGRCALOVRD(5'b11111),
		.RCALENB(1'b1),
		.PMARSVD(1'b0)
	);

endmodule
