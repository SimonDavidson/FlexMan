// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
// =============================================================================
// tb_update_state_for_neuron  (annAcc: ann_update_state_for_neuron)
//
// Authors      : Simon Davidson & Claude
// Created      : 2026-05-08
// Last modified: 2026-06-07
//
// Threshold (RELU/LUT/ABS) + decay, checked against the np_ann_threshold /
// np_ann_decay golden over directed corner cases + a constrained-random loop.
// (Output requantisation lives in ann neuron_processing, not this module.)
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
        .neuron_valid_i(neuron_valid),
        .potential_i(potential), .lut_result_i(lut_result),
        .potential_decay_mult_i(pot_dcy),
        .neuron_taken_o(neuron_taken),
        .result_valid_o(result_valid),
        .act_out_o(act_out_o), .decayed_act_o(decayed_act_o),
        .result_taken_i(result_taken));

    initial clk = 0; always #5 clk = ~clk;

    reg [31:0] d_act, d_dec, g_act, g_dec;

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

    integer k;
    initial begin
        verif_errors=0; verif_checks=0;
        neuron_valid=0; result_taken=0; thresh_op=0; potential=0; lut_result=0; pot_dcy=0;
        reset=1; @(posedge clk); #1; reset=0; @(posedge clk); #1;
        $display("=== tb_update_state_for_neuron (annAcc) ===");

        // directed: RELU pos/neg, ABS pos/neg/min-neg, LUT
        drive_and_check( 32'sd100, 8'd0,  2'b00, 32'h8000_0000, "D1 RELU pos");
        drive_and_check(-32'sd100, 8'd0,  2'b00, 32'h8000_0000, "D2 RELU neg->0");
        drive_and_check(-32'sd100, 8'd0,  2'b10, 32'hFFFF_FFFF, "D3 ABS neg");
        drive_and_check( 32'sd100, 8'd0,  2'b10, 32'h8000_0000, "D4 ABS pos");
        drive_and_check( 32'h8000_0000, 8'd0, 2'b10, 32'h8000_0000, "D5 ABS min-neg sat");
        drive_and_check( 32'sd7,   8'd200,2'b01, 32'h8000_0000, "D6 LUT");

        void'($urandom(32'hA00A_5501));
        for (k=0; k<NVEC; k=k+1)
            drive_and_check($urandom(), $urandom(), $urandom()%3, $urandom(), "rand");

        `VERIF_EPILOGUE("tb_update_state_for_neuron")
    end

    `VERIF_WATCHDOG(2000000)

endmodule
