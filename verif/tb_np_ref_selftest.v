// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
// =============================================================================
// verif/tb_np_ref_selftest.v
//
// Authors      : Simon Davidson & Claude
// Created      : 2026-06-07
// Last modified: 2026-06-07
//
// Phase-0 self-test for the shared verif/ library.  Validates, in one run:
//   (1) checks.vh / np_ref.vh / vt_driver.vh compile and link under xrun -sv
//       with the includes placed INSIDE the module body;
//   (2) the np_ref_lif golden matches the REAL snnAcc update_state_for_neuron
//       DUT bit-for-bit over many random vectors (mirror correctness);
//   (3) the documented saturation quirk (sum 0x80000000 -> 0x80000000).
//
// Run:  bash verif/selftest.bsh   (or via run_regression.sh)
// =============================================================================
`timescale 1ns/1ps

module tb_np_ref_selftest;

    localparam integer NVEC = 2000;     // random vectors

    reg                clk, reset;
    reg                neuron_valid;
    reg  signed [31:0] syn, pot, bias, thresh;
    reg        [31:0]  syn_dcy, pot_dcy;
    reg                sub_on_fire;
    reg                result_taken;

    wire               neuron_taken;
    wire               result_valid;
    wire signed [31:0] potential_o;
    wire signed [31:0] syn_curr_o;
    wire               spike_o;

    // ---- self-checking + golden model (inside module scope) ----------------
    integer verif_errors;
    integer verif_checks;
    integer verif_to;
    `include "../verif/checks.vh"
    `include "../verif/np_ref.vh"
    `include "../verif/vt_driver.vh"

    // ---- DUT: the real snnAcc update_state_for_neuron ----------------------
    update_state_for_neuron #(
        .SYN_CURR_SLICE_BITS (32),
        .POT_SLICE_BITS      (32),
        .BIAS_CURR_SLICE_BITS(32),
        .THRESH_SLICE_BITS   (32))
    dut (
        .clk                   (clk),
        .reset                 (reset),
        .neuron_valid_i        (neuron_valid),
        .syn_curr_i            (syn),
        .potential_i           (pot),
        .bias_curr_i           (bias),
        .threshold_i           (thresh),
        .syn_curr_decay_mult_i (syn_dcy),
        .potential_decay_mult_i(pot_dcy),
        .sub_on_fire_i         (sub_on_fire),
        .neuron_taken_o        (neuron_taken),
        .result_valid_o        (result_valid),
        .potential_o           (potential_o),
        .syn_curr_o            (syn_curr_o),
        .spike_o               (spike_o),
        .result_taken_i        (result_taken));

    initial clk = 0;
    always #5 clk = ~clk;

    // ---- drive one neuron through the pipeline, sample outputs -------------
    task run_neuron;
        input signed [31:0] i_syn, i_pot, i_bias, i_thresh;
        input        [31:0] i_sdcy, i_pdcy;
        input               i_sof;
        output signed [31:0] o_pot, o_syn;
        output               o_spk;
        begin
            syn = i_syn; pot = i_pot; bias = i_bias; thresh = i_thresh;
            syn_dcy = i_sdcy; pot_dcy = i_pdcy; sub_on_fire = i_sof;
            `VT_PULSE(neuron_valid)
            while (!result_valid) @(posedge clk); #1;
            o_pot = potential_o; o_syn = syn_curr_o; o_spk = spike_o;
            `VT_CONSUME(result_valid, result_taken)
        end
    endtask

    integer k;
    reg signed [31:0] g_pot, g_syn, d_pot, d_syn;
    reg               g_spk, d_spk;
    reg signed [31:0] ideal_syn;

    initial begin
        verif_errors = 0; verif_checks = 0;
        neuron_valid = 0; result_taken = 0; sub_on_fire = 0;
        syn = 0; pot = 0; bias = 0; thresh = 0; syn_dcy = 0; pot_dcy = 0;
        reset = 1;
        @(posedge clk); #1;
        reset = 0;
        @(posedge clk); #1;

        $display("=== tb_np_ref_selftest: %0d random LIF vectors ===", NVEC);
        void'($urandom(32'h5117_0001));   // deterministic seed

        // -------- random DUT-vs-golden (mirror must match exactly) ----------
        for (k = 0; k < NVEC; k = k + 1) begin
            syn     = $urandom();
            pot     = $urandom();
            bias    = $urandom();
            thresh  = $urandom();
            syn_dcy = $urandom();
            pot_dcy = $urandom();
            sub_on_fire = $urandom() & 1'b1;

            np_ref_lif(syn, pot, bias, thresh, syn_dcy, pot_dcy, sub_on_fire,
                       g_spk, g_pot, g_syn);
            run_neuron(syn, pot, bias, thresh, syn_dcy, pot_dcy, sub_on_fire,
                       d_pot, d_syn, d_spk);

            check_bit(d_spk, g_spk, "rand spike");
            check_eq (d_pot, g_pot, "rand potential_o");
            check_eq (d_syn, g_syn, "rand syn_curr_o");
        end

        // -------- directed: saturation quirk (sum == 0x80000000) ------------
        // syn=1, pot=0x7FFFFFFE, bias=1 -> sum=0x80000000 -> clamps to 0x80000000.
        np_ref_lif(32'h0000_0001, 32'h7FFF_FFFE, 32'h0000_0001, 32'h7FFF_FFFF,
                   32'h8000_0000, 32'h8000_0000, 1'b0, g_spk, g_pot, g_syn);
        run_neuron(32'h0000_0001, 32'h7FFF_FFFE, 32'h0000_0001, 32'h7FFF_FFFF,
                   32'h8000_0000, 32'h8000_0000, 1'b0, d_pot, d_syn, d_spk);
        check_eq(d_pot, g_pot, "quirk: DUT == golden");
        // psat clamps to 0x80000000 (= -2^31); F1-fixed signed decay by 0.5
        // gives -2^30 = 0xC0000000. Assert the concrete value:
        check_eq(g_pot, 32'shC000_0000, "quirk: decayed clamp value");

        // -------- F1 (FIXED): negative syn decay must be signed-correct ------
        run_neuron(-32'sd1000, 32'd0, 32'd0, 32'h7FFF_FFFF,
                   32'h8000_0000 /*0.5*/, 32'h8000_0000, 1'b0,
                   d_pot, d_syn, d_spk);
        ideal_syn = np_decay_signed(-32'sd1000, 32'h8000_0000);  // = -500
        check_eq(d_syn, ideal_syn, "F1 fixed: negative-syn decay signed-correct (-500)");

        `VERIF_EPILOGUE("tb_np_ref_selftest")
    end

    `VERIF_WATCHDOG(2000000)

endmodule
