// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
// =============================================================================
// tb_bram_rdgate  --  RD_GATE=1 read gating on bram_sdp / bram_sp / bram_sdp_uram
//
// Authors      : Simon Davidson & Claude
// Created      : 2026-09-16
// Last modified: 2026-09-16
//
// RD_GATE=1 makes the read conditional on `re`, which infers the BRAM enable
// pin. Motivation is measured, not theoretical: with the Bosch design idle
// (clock running, every *_mem_rd_o low) the xczu7ev build still burned 24 mW in
// BRAM + 5 mW in URAM, because these arrays read on EVERY edge forever, and idle
// dominates the deployed duty cycle.
//
// Two properties, and they pull in opposite directions -- which is what makes
// this test able to fail:
//
//   A. EQUIVALENCE. With re held high, RD_GATE=1 must be bit-identical to
//      RD_GATE=0. If gating changed behaviour when enabled, this catches it.
//
//   B. HOLD. With re low, dout must HOLD while raddr moves underneath it.
//      If `re` were ignored (gating silently doing nothing -- the exact
//      regression worth fearing, because it would still pass every functional
//      test while saving no power) this fails.
//
// NEGATIVE CONTROL: property B is applied to the RD_GATE=0 instance too, where
// it MUST fail -- that instance tracks raddr by design. Those expected failures
// are counted separately and asserted to be non-zero. A "hold" check that
// cannot fail would prove nothing about the gated instance.
// =============================================================================
`timescale 10ps/1ps

module tb_bram_rdgate;

    localparam integer DEPTH  = 64;
    localparam integer DATA_W = 32;
    localparam integer AW     = 6;
    localparam integer NVEC   = 2000;

    reg               clk, we, re;
    reg  [AW-1:0]     waddr, raddr;
    reg  [DATA_W-1:0] din;

    integer verif_errors, verif_checks;
    `include "../verif/checks.vh"

    // Same stimulus into every instance; only RD_GATE differs.
    wire [DATA_W-1:0] d_sdp_g, d_sdp_f, d_uram_g, d_sp_g, d_sp_f;

    bram_sdp #(.DEPTH(DEPTH), .DATA_W(DATA_W), .RD_GATE(1)) u_sdp_gated (
        .clk(clk), .we(we), .waddr(waddr), .din(din),
        .re(re), .raddr(raddr), .dout(d_sdp_g));

    bram_sdp #(.DEPTH(DEPTH), .DATA_W(DATA_W), .RD_GATE(0)) u_sdp_free (
        .clk(clk), .we(we), .waddr(waddr), .din(din),
        .re(1'b0), .raddr(raddr), .dout(d_sdp_f));   // re deliberately ignored

    bram_sdp_uram #(.DEPTH(DEPTH), .DATA_W(DATA_W), .RD_GATE(1)) u_uram_gated (
        .clk(clk), .we(we), .waddr(waddr), .din(din),
        .re(re), .raddr(raddr), .dout(d_uram_g));

    bram_sp #(.DEPTH(DEPTH), .DATA_W(DATA_W), .RD_GATE(1)) u_sp_gated (
        .clk(clk), .we(we), .re(re), .addr(raddr), .din(din), .dout(d_sp_g));

    bram_sp #(.DEPTH(DEPTH), .DATA_W(DATA_W), .RD_GATE(0)) u_sp_free (
        .clk(clk), .we(we), .re(1'b0), .addr(raddr), .din(din), .dout(d_sp_f));

    initial clk = 0; always #50 clk = ~clk;

    integer k;
    integer neg_control_failures;      // property B applied to RD_GATE=0: must be > 0
    // One frozen reference PER instance: bram_sp writes to `addr` while
    // bram_sdp writes to `waddr`, so their contents legitimately diverge
    // during phase A. Comparing one against the other's frozen value was a
    // testbench bug, not an RTL bug.
    reg [DATA_W-1:0] held_sdp_g, held_uram_g, held_sp_g, held_sdp_f;

    // A plain inequality check that does NOT touch verif_errors -- used only to
    // count the negative control, where a mismatch is the expected outcome.
    task expect_tracks;                // free-running instance must NOT hold
        input [DATA_W-1:0] observed;
        input [DATA_W-1:0] frozen;
        begin
            if (observed !== frozen) neg_control_failures = neg_control_failures + 1;
        end
    endtask

    initial begin
        verif_errors = 0; verif_checks = 0; neg_control_failures = 0;
        we = 0; re = 1; waddr = 0; raddr = 0; din = 0;
        @(posedge clk);

        // Seed the array so reads return something distinguishable.
        for (k = 0; k < DEPTH; k = k + 1) begin
            we = 1; waddr = k[AW-1:0]; din = 32'hA5A5_0000 | k;
            @(posedge clk); #1;
        end
        we = 0;

        // ── Property A: re high => gated is bit-identical to free-running ────
        re = 1;
        for (k = 0; k < NVEC; k = k + 1) begin
            we    = $urandom & 1'b1;
            waddr = $urandom_range(DEPTH-1);
            raddr = $urandom_range(DEPTH-1);
            din   = $urandom;
            @(posedge clk); #1;
            check_eq_u(d_sdp_g,  d_sdp_f, "sdp  gated==free when re=1");
            check_eq_u(d_uram_g, d_sdp_f, "uram gated==free when re=1");
            check_eq_u(d_sp_g,   d_sp_f,  "sp   gated==free when re=1");
        end

        // ── Property B: re low => dout HOLDS while raddr moves ───────────────
        we = 0; re = 1; raddr = 0;
        @(posedge clk); #1;
        held_sdp_g  = d_sdp_g;            // freeze each instance's own last read
        held_uram_g = d_uram_g;
        held_sp_g   = d_sp_g;
        held_sdp_f  = d_sdp_f;
        re = 0;
        for (k = 0; k < DEPTH; k = k + 1) begin
            raddr = k[AW-1:0];            // sweep the address underneath
            @(posedge clk); #1;
            check_eq_u(d_sdp_g,  held_sdp_g,  "sdp  holds while re=0");
            check_eq_u(d_uram_g, held_uram_g, "uram holds while re=0");
            check_eq_u(d_sp_g,   held_sp_g,   "sp   holds while re=0");
            // NEGATIVE CONTROL: the RD_GATE=0 instance must NOT hold.
            expect_tracks(d_sdp_f, held_sdp_f);
        end

        // The negative control must have fired, or property B proves nothing.
        check_true(neg_control_failures > 0,
                   "NEGATIVE CONTROL: RD_GATE=0 instance tracked raddr as expected");
        $display("[rdgate] negative control: %0d/%0d cycles where the ungated RAM moved",
                 neg_control_failures, DEPTH);

        `VERIF_EPILOGUE("tb_bram_rdgate")
    end

    `VERIF_WATCHDOG(2000000)

endmodule
