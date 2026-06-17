// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
// =============================================================================
// tb_update_state_for_neuron  (annAcc: ann_update_state_for_neuron)
//
// Authors      : Simon Davidson & Claude
// Created      : 2026-05-08
// Last modified: 2026-06-17
//
// Threshold (RELU/LUT/ABS) + decay, checked against the np_ann_threshold /
// np_ann_decay golden over directed corner cases + a constrained-random loop.
// (Output requantisation lives in ann neuron_processing, not this module.)
//
// §5.2 coverage: out_signed_i sign-extends a signed (tanh) LUT entry from its
// slice MSB instead of zero-extending. Proven on the 8-bit DUT (extend from
// bit 7) AND a second 16-bit DUT (extend from bit 15 — the 16-bit deploy width).
// out_signed_i defaults 0 for the whole random loop, so the legacy zero-extend
// path stays bit-identical.
// =============================================================================
`timescale 1ns/1ps

module tb_update_state_for_neuron;

    localparam integer NVEC = 3000;

    reg                clk, reset;
    reg        [1:0]   thresh_op;
    reg                neuron_valid;
    reg  signed [31:0] potential;
    reg        [7:0]   lut_result;
    reg        [31:0]  pot_dcy;
    reg                result_taken;
    reg                out_signed;       // §5.2: 0 = zero-extend (default), 1 = sign-extend

    wire               neuron_taken, result_valid;
    wire       [31:0]  act_out_o, decayed_act_o;

    integer verif_errors, verif_checks, verif_to;
    `include "../verif/checks.vh"
    `include "../verif/np_ref.vh"
    `include "../verif/vt_driver.vh"

    ann_update_state_for_neuron #(
        .POT_SLICE_BITS(32), .THRESH_SLICE_BITS(8), .POT_DECAY_BITS(32))
    dut (
        .clk(clk), .reset(reset),
        .thresh_op_i(thresh_op),
        .out_signed_i(out_signed),
        .neuron_valid_i(neuron_valid),
        .potential_i(potential), .lut_result_i(lut_result),
        .potential_decay_mult_i(pot_dcy),
        .neuron_taken_o(neuron_taken),
        .result_valid_o(result_valid),
        .act_out_o(act_out_o), .decayed_act_o(decayed_act_o),
        .result_taken_i(result_taken));

    // §5.2 deploy-width DUT: 16-bit LUT entries (sign-extend from bit 15)
    reg                neuron_valid16, result_taken16;
    reg        [15:0]  lut_result16;
    wire               neuron_taken16, result_valid16;
    wire       [31:0]  act_out_o16, decayed_act_o16;

    ann_update_state_for_neuron #(
        .POT_SLICE_BITS(32), .THRESH_SLICE_BITS(16), .POT_DECAY_BITS(32))
    dut16 (
        .clk(clk), .reset(reset),
        .thresh_op_i(2'b01),                 // LUT mode
        .out_signed_i(1'b1),                 // signed (tanh) output
        .neuron_valid_i(neuron_valid16),
        .potential_i(32'sd0), .lut_result_i(lut_result16),
        .potential_decay_mult_i(pot_dcy),
        .neuron_taken_o(neuron_taken16),
        .result_valid_o(result_valid16),
        .act_out_o(act_out_o16), .decayed_act_o(decayed_act_o16),
        .result_taken_i(result_taken16));

    initial clk = 0; always #5 clk = ~clk;

    reg [31:0] d_act, d_dec, g_act, g_dec;
    reg        [31:0] e_act;     // expected act_out (unsigned 32-bit view)
    reg signed [31:0] s_act;     // signed view, for the decay multiply
    reg signed [63:0] s_prod;

    task drive_and_check;
        input signed [31:0] i_pot;
        input        [7:0]  i_lut;
        input        [1:0]  i_op;
        input        [31:0] i_dcy;
        input        [255:0] tag;
        begin
            potential=i_pot; lut_result=i_lut; thresh_op=i_op; pot_dcy=i_dcy;
            g_act = np_ann_threshold(i_pot, i_lut, i_op);
            g_dec = np_ann_decay(g_act, i_dcy);
            `VT_PULSE(neuron_valid)
            while (!result_valid) @(posedge clk); #1;
            d_act = act_out_o; d_dec = decayed_act_o;
            `VT_CONSUME(result_valid, result_taken)
            check_eq_u(d_act, g_act, {tag, " act_out"});
            check_eq_u(d_dec, g_dec, {tag, " decayed"});
        end
    endtask

    // §5.2 on the 8-bit DUT: out_signed=1 must sign-extend the int8 entry from
    // bit 7 (act_out) and feed the signed value to the decay multiply.
    task drive_check_signed8;
        input        [7:0]   i_lut;       // signed int8 entry
        input        [31:0]  i_dcy;
        input        [255:0] tag;
        begin
            potential = 32'sd0; lut_result = i_lut; thresh_op = 2'b01; pot_dcy = i_dcy;
            out_signed = 1'b1;
            e_act  = {{24{i_lut[7]}}, i_lut};            // expected sign-extended act_out
            s_act  = e_act;                              // signed view for the mul
            s_prod = s_act * $signed({1'b0, i_dcy});     // mirrors the RTL decay mul
            `VT_PULSE(neuron_valid)
            while (!result_valid) @(posedge clk); #1;
            d_act = act_out_o; d_dec = decayed_act_o;
            `VT_CONSUME(result_valid, result_taken)
            check_eq_u(d_act, e_act,         {tag, " s8 act_out"});
            check_eq_u(d_dec, s_prod[63:32], {tag, " s8 decayed"});
            out_signed = 1'b0;
        end
    endtask

    // §5.2 on the 16-bit deploy-width DUT (dut16: LUT+signed hard-wired):
    // sign-extend the int16 entry from bit 15.
    task drive_check_signed16;
        input        [15:0]  i_lut;       // signed int16 entry
        input        [31:0]  i_dcy;
        input        [255:0] tag;
        begin
            lut_result16 = i_lut; pot_dcy = i_dcy;
            e_act  = {{16{i_lut[15]}}, i_lut};
            s_act  = e_act;
            s_prod = s_act * $signed({1'b0, i_dcy});
            `VT_PULSE(neuron_valid16)
            while (!result_valid16) @(posedge clk); #1;
            d_act = act_out_o16; d_dec = decayed_act_o16;
            `VT_CONSUME(result_valid16, result_taken16)
            check_eq_u(d_act, e_act,         {tag, " s16 act_out"});
            check_eq_u(d_dec, s_prod[63:32], {tag, " s16 decayed"});
        end
    endtask

    integer k;
    initial begin
        verif_errors=0; verif_checks=0;
        neuron_valid=0; result_taken=0; thresh_op=0; potential=0; lut_result=0; pot_dcy=0;
        out_signed=0;
        neuron_valid16=0; result_taken16=0; lut_result16=0;
        reset=1; @(posedge clk); #1; reset=0; @(posedge clk); #1;
        $display("=== tb_update_state_for_neuron (annAcc) ===");

        // directed: RELU pos/neg, ABS pos/neg/min-neg, LUT
        drive_and_check( 32'sd100, 8'd0,  2'b00, 32'h8000_0000, "D1 RELU pos");
        drive_and_check(-32'sd100, 8'd0,  2'b00, 32'h8000_0000, "D2 RELU neg->0");
        drive_and_check(-32'sd100, 8'd0,  2'b10, 32'hFFFF_FFFF, "D3 ABS neg");
        drive_and_check( 32'sd100, 8'd0,  2'b10, 32'h8000_0000, "D4 ABS pos");
        drive_and_check( 32'h8000_0000, 8'd0, 2'b10, 32'h8000_0000, "D5 ABS min-neg sat");
        drive_and_check( 32'sd7,   8'd200,2'b01, 32'h8000_0000, "D6 LUT");

        // §5.2 default-off (out_signed=0): LUT entry with bit 7 set zero-extends.
        // 0xF8 -> act_out 0x000000F8 (= +248, NOT -8) via the golden (zero-extend).
        drive_and_check( 32'sd0, 8'hF8, 2'b01, 32'h8000_0000, "D7 LUT unsigned 0xF8->248");

        // §5.2 ON (8-bit): same entry sign-extends to -8.
        drive_check_signed8(8'hF8, 32'h8000_0000, "D8 int8 -8 @0.5");   // act -8, dec -4
        drive_check_signed8(8'h80, 32'hFFFF_FFFF, "D9 int8 -128 @~1");  // act -128
        drive_check_signed8(8'h7F, 32'h8000_0000, "D10 int8 +127 @0.5");// sign bit 0 -> +127

        // §5.2 ON (16-bit deploy width): sign-extend from bit 15.
        drive_check_signed16(16'hC000, 32'h8000_0000, "D11 int16 -16384 @0.5"); // act -16384
        drive_check_signed16(16'h8000, 32'h8000_0000, "D12 int16 -32768 @0.5");
        drive_check_signed16(16'h7FFF, 32'h8000_0000, "D13 int16 +32767 @0.5"); // positive

        void'($urandom(32'hA00A_5501));
        for (k=0; k<NVEC; k=k+1)
            drive_and_check($urandom(), $urandom(), $urandom()%3, $urandom(), "rand");

        `VERIF_EPILOGUE("tb_update_state_for_neuron")
    end

    `VERIF_WATCHDOG(2000000)

endmodule
