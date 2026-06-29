// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
// =============================================================================
// tb_update_state_for_neuron  (fmiSnnAcc: per-neuron decay + adaptive)
//
// Authors      : Simon Davidson & Claude
// Created      : 2026-05-15
// Last modified: 2026-06-29
//
// FMI membrane update checked against np_ref_fmi over directed corner cases +
// a constrained-random loop, exercising both the plain (has_ada=0, 2-cycle) and
// adaptive (has_ada=1, 3-cycle) paths. The driver polls result_valid_o so it is
// depth-agnostic.
// =============================================================================
`timescale 1ns/1ps

module tb_update_state_for_neuron;

    localparam integer NVEC = 3000;

    reg                clk, reset;
    reg                neuron_valid;
    reg  signed [31:0] syn, pot, thresh;
    reg        [31:0]  syn_dcy, mem_dcy;
    reg                has_ada;
    reg        [31:0]  ada;
    reg  signed [31:0] b_eff;
    reg        [31:0]  dcy_ada, scl_ada;
    reg                result_taken;

    wire               neuron_taken, result_valid, spike_o;
    wire signed [31:0] potential_o, syn_curr_o;
    wire       [31:0]  ada_o;

    integer verif_errors, verif_checks, verif_to;
    `include "../verif/checks.vh"
    `include "../verif/np_ref.vh"
    `include "../verif/vt_driver.vh"

    update_state_for_neuron #(
        .SYN_CURR_SLICE_BITS(32), .POT_SLICE_BITS(32), .THRESH_SLICE_BITS(32))
    dut (
        .clk(clk), .reset(reset),
        .neuron_valid_i(neuron_valid),
        .syn_curr_i(syn), .potential_i(pot), .threshold_i(thresh),
        .syn_dcy_i(syn_dcy), .mem_dcy_i(mem_dcy),
        .has_ada_i(has_ada), .ada_i(ada), .b_eff_i(b_eff),
        .dcy_ada_i(dcy_ada), .scl_ada_i(scl_ada),
        .readout_mode_i(1'b0),   // default-off: plain/adaptive LIF (readout = con6 only)
        .neuron_taken_o(neuron_taken),
        .result_valid_o(result_valid),
        .potential_o(potential_o), .syn_curr_o(syn_curr_o), .spike_o(spike_o),
        .ada_o(ada_o), .result_taken_i(result_taken));

    initial clk = 0; always #5 clk = ~clk;

    reg signed [31:0] d_pot, d_syn, g_pot, g_syn;
    reg        [31:0] d_ada, g_ada;
    reg               d_spk, g_spk;

    task drive_and_check;
        input signed [31:0] i_syn, i_pot, i_thresh;
        input        [31:0] i_sdcy, i_mdcy;
        input               i_ada_en;
        input        [31:0] i_ada, i_dcyada, i_sclada;
        input  signed [31:0] i_beff;
        input        [255:0] tag;
        begin
            syn=i_syn; pot=i_pot; thresh=i_thresh; syn_dcy=i_sdcy; mem_dcy=i_mdcy;
            has_ada=i_ada_en; ada=i_ada; dcy_ada=i_dcyada; scl_ada=i_sclada; b_eff=i_beff;
            np_ref_fmi(i_syn, i_pot, i_thresh, i_sdcy, i_mdcy, i_ada_en,
                       i_ada, i_dcyada, i_sclada, i_beff, g_spk, g_pot, g_syn, g_ada);
            `VT_PULSE(neuron_valid)
            while (!result_valid) @(posedge clk); #1;
            d_pot=potential_o; d_syn=syn_curr_o; d_spk=spike_o; d_ada=ada_o;
            `VT_CONSUME(result_valid, result_taken)
            check_bit (d_spk, g_spk, {tag, " spike"});
            check_eq  (d_pot, g_pot, {tag, " pot"});
            check_eq  (d_syn, g_syn, {tag, " syn"});
            check_eq_u(d_ada, g_ada, {tag, " ada"});
        end
    endtask

    integer k;
    initial begin
        verif_errors=0; verif_checks=0;
        neuron_valid=0; result_taken=0; has_ada=0;
        syn=0; pot=0; thresh=0; syn_dcy=0; mem_dcy=0; ada=0; b_eff=0; dcy_ada=0; scl_ada=0;
        reset=1; @(posedge clk); #1; reset=0; @(posedge clk); #1;
        $display("=== tb_update_state_for_neuron (fmiSnnAcc) ===");

        // directed: plain LIF no-spike / spike; adaptive no-spike / spike
        drive_and_check(32'sd10, 32'sd20, 32'sd1000, 32'h8000_0000, 32'h8000_0000,
                        1'b0, 32'd0, 32'd0, 32'd0, 32'sd0, "D1 plain no-spike");
        drive_and_check(32'sd50, 32'sd900, 32'sd100, 32'h8000_0000, 32'hC000_0000,
                        1'b0, 32'd0, 32'd0, 32'd0, 32'sd0, "D2 plain spike");
        drive_and_check(32'sd10, 32'sd20, 32'sd1000, 32'h8000_0000, 32'h8000_0000,
                        1'b1, 32'd100, 32'h4000_0000, 32'h4000_0000, 32'sh2000_0000, "D3 ada no-spike");
        drive_and_check(32'sd50, 32'sd900, 32'sd100, 32'h8000_0000, 32'hC000_0000,
                        1'b1, 32'd200, 32'h4000_0000, 32'h4000_0000, 32'sh2000_0000, "D4 ada spike");

        void'($urandom(32'hF301_AdA5));
        for (k=0; k<NVEC; k=k+1)
            drive_and_check($urandom(), $urandom(), $urandom(), $urandom(), $urandom(),
                            $urandom()&1'b1, $urandom(), $urandom(), $urandom(), $urandom(), "rand");

        `VERIF_EPILOGUE("tb_update_state_for_neuron")
    end

    `VERIF_WATCHDOG(2500000)

endmodule
