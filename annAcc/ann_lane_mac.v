// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
/////////////////////////////////////////////////////////////////////
//
// ann_lane_mac -- one MAC lane of the output-stationary ANN tile ().
//
// Authors      : Simon Davidson & Claude
// Created      : 2026-11-21
// Last modified: 2026-11-21
//
// acc_o = acc_i + (w_i * a_i)[31:0]   (signed, 32-bit wrap-around).
//
// w_i is the weight already sign-extended to WBITS_MAX bits (<= 16 for the
// 8/16-bit weight sizes the tile supports). a_i is the activation operand
// already extended to ACT_BITS+1 signed bits (zero- or sign-extended per
// act_signed). The low 32 bits of an exact signed product do not depend on the
// operand width as long as the operands are exact, so this is bit-identical to
// the legacy 32b x 33b multiplier for every supported weight/activation width.
// ACT_BITS (= MAC_ACT_BITS in the tile) is the area knob: 8 gives an 16x9
// multiplier per lane instead of 16x33.
/////////////////////////////////////////////////////////////////////
module ann_lane_mac #(
    parameter WBITS_MAX = 16,
    parameter ACT_BITS  = 32
)(
    input  wire signed [WBITS_MAX-1:0] w_i,
    input  wire signed [ACT_BITS:0]    a_i,
    input  wire        [31:0]          acc_i,
    output wire        [31:0]          acc_o
);
    localparam PW = WBITS_MAX + ACT_BITS + 1;      // exact product width
    wire signed [PW-1:0] prod = w_i * a_i;
    // sign-extend (PW < 32) or truncate (PW >= 32) the exact product to 32 bits
    wire [31:0] prod32;
    generate if (PW >= 32) begin : g_trunc
        assign prod32 = prod[31:0];
    end else begin : g_ext
        assign prod32 = {{(32-PW){prod[PW-1]}}, prod};
    end endgenerate
    assign acc_o = acc_i + prod32;
endmodule
