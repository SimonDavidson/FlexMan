// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
// =============================================================================
// tb_update_state_for_neuron  (snnAcc, LIF)
//
// Authors      : Simon Davidson & Claude
// Created      : 2026-05-07
// Last modified: 2026-06-07
//
// Aggressive rewrite: directed corner cases (now checked against the np_ref_lif
// golden model) PLUS a constrained-random loop comparing the DUT bit-for-bit to
// the golden over thousands of vectors, PLUS the documented saturation quirk and
// the F1 negative-decay spec-intent probe (see verif/FINDINGS.md).
//
// DUT is the 3-cycle pipeline (ST_IDLE -> ST_C1 -> ST_C2).  The driver polls
// result_valid_o, so it is agnostic to pipeline depth.
// =============================================================================
`timescale 1ns/1ps

module tb_update_state_for_neuron;

    localparam integer NVEC = 3000;

    reg                clk, reset;
    reg                neuron_valid;
    reg  signed [31:0] syn, pot, bias, thresh;
    reg        [31:0]  syn_dcy, pot_dcy;
    reg                sub_on_fire;
    reg                result_taken;

    wire               neuron_taken, result_valid, spike_o;
    wire signed [31:0] potential_o, syn_curr_o;

    integer verif_errors, verif_checks, verif_to;
    `include "../verif/checks.vh"
    `include "../verif/np_ref.vh"
    `include "../verif/vt_driver.vh"

    update_state_for_neuron #(
        .SYN_CURR_SLICE_BITS (32), .POT_SLICE_BITS(32),
        .BIAS_CURR_SLICE_BITS(32), .THRESH_SLICE_BITS(32))
    dut (
        .clk(clk), .reset(reset),
        .neuron_valid_i(neuron_valid),
        .syn_curr_i(syn), .potential_i(pot), .bias_curr_i(bias), .threshold_i(thresh),
        .syn_curr_decay_mult_i(syn_dcy), .potential_decay_mult_i(pot_dcy),
        .sub_on_fire_i(sub_on_fire),
        .neuron_taken_o(neuron_taken),
        .result_valid_o(result_valid),
        .potential_o(potential_o), .syn_curr_o(syn_curr_o), .spike_o(spike_o),
        .result_taken_i(result_taken));

    initial clk = 0; always #5 clk = ~clk;

    reg signed [31:0] d_pot, d_syn, g_pot, g_syn;
    reg               d_spk, g_spk;

    task run_neuron;
        input signed [31:0] i_syn, i_pot, i_bias, i_thresh;
        input        [31:0] i_sdcy, i_pdcy;
        input               i_sof;
        begin
            syn=i_syn; pot=i_pot; bias=i_bias; thresh=i_thresh;
            syn_dcy=i_sdcy; pot_dcy=i_pdcy; sub_on_fire=i_sof;
            `VT_PULSE(neuron_valid)
            while (!result_valid) @(posedge clk); #1;
            d_pot=potential_o; d_syn=syn_curr_o; d_spk=spike_o;
            `VT_CONSUME(result_valid, result_taken)
        end
    endtask

    // Drive DUT + golden with the same stimulus, compare all three outputs.
    task drive_and_check;
        input signed [31:0] i_syn, i_pot, i_bias, i_thresh;
        input        [31:0] i_sdcy, i_pdcy;
        input               i_sof;
        input        [255:0] tag;
        begin
            np_ref_lif(i_syn, i_pot, i_bias, i_thresh, i_sdcy, i_pdcy, i_sof,
                       g_spk, g_pot, g_syn);
            run_neuron(i_syn, i_pot, i_bias, i_thresh, i_sdcy, i_pdcy, i_sof);
            check_bit(d_spk, g_spk, tag);
            check_eq (d_pot, g_pot, tag);
            check_eq (d_syn, g_syn, tag);
        end
    endtask

    integer k;
    reg signed [31:0] ideal_syn;

    initial begin
        verif_errors=0; verif_checks=0;
        neuron_valid=0; result_taken=0; sub_on_fire=0;
        syn=0; pot=0; bias=0; thresh=0; syn_dcy=0; pot_dcy=0;
        reset=1; @(posedge clk); #1; reset=0; @(posedge clk); #1;
        $display("=== tb_update_state_for_neuron (snnAcc LIF) ===");

        // ---- directed corner cases (expected via golden) ----
        drive_and_check(32'd10, 32'd20, 32'd5,  32'd100, 32'h80000000, 32'h80000000, 1'b0, "D1 no-spike");
        drive_and_check(32'd0,  32'd90, 32'd10, 32'd100, 32'h80000000, 32'h80000000, 1'b0, "D2 spike sum==thresh");
        drive_and_check(32'd50, 32'd60, 32'd20, 32'd100, 32'h80000000, 32'h80000000, 1'b0, "D3 spike sum>thresh");
        drive_and_check(32'd42, 32'd55, 32'd3,  32'd1000,32'h00000000, 32'h00000000, 1'b0, "D4 zero decay");
        drive_and_check(32'd0,  32'd100,32'd0,  32'd80,  32'h80000000, 32'h80000000, 1'b1, "D5 sub_on_fire spike");
        drive_and_check(32'd0,  32'd40, 32'd0,  32'd80,  32'h80000000, 32'h80000000, 1'b1, "D6 sub_on_fire no-spike");

        // ---- directed: documented saturation quirk (sum 0x80000000) ----
        // sum = 0x7FFFFFFE + 1 + 1 = 0x80000000 -> clamps to 0x80000000, decays to 0x40000000
        drive_and_check(32'h00000001, 32'h7FFFFFFE, 32'h00000001, 32'h7FFFFFFF,
                        32'h80000000, 32'h80000000, 1'b0, "D7 sat quirk");
        check_eq(g_pot, 32'sh4000_0000, "D7 quirk decayed clamp == 0x40000000");

        // ---- constrained-random vs golden (mirror must match exactly) ----
        void'($urandom(32'hD5DA_7E01));
        for (k=0; k<NVEC; k=k+1)
            drive_and_check($urandom(), $urandom(), $urandom(), $urandom(),
                            $urandom(), $urandom(), $urandom()&1'b1, "rand");

        // ---- F1 spec-intent probe: negative syn decay (signed vs unsigned) ----
        // Non-fatal NOTE until the RTL decision is made (see verif/FINDINGS.md F1).
        run_neuron(-32'sd1000, 32'd0, 32'd0, 32'h7FFFFFFF,
                   32'h80000000, 32'h80000000, 1'b0);
        ideal_syn = np_decay_signed(-32'sd1000, 32'h80000000);   // = -500
        if (d_syn !== ideal_syn)
            $display("NOTE F1 negative-syn decay: DUT=%0d (0x%08h) ideal_signed=%0d (0x%08h) -- signed*unsigned divergence",
                     d_syn, d_syn, ideal_syn, ideal_syn);

        `VERIF_EPILOGUE("tb_update_state_for_neuron")
    end

    `VERIF_WATCHDOG(2000000)

endmodule
