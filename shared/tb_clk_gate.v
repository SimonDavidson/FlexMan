// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
// =============================================================================
// tb_clk_gate  --  clk_gate (gated-clock cell) + clk_gate_ctrl (enable logic)
//
// Authors      : Simon Davidson & Claude
// Created      : 2026-09-23
// Last modified: 2026-09-23
//
// Four properties. The first two are what the design needs; the last two are
// the ones that catch the two ways this kind of logic fails SILENTLY -- still
// functionally correct, still passing every other test, saving no power or
// corrupting the domain it gates.
//
//   A. EQUIVALENCE. Enable held high => the gated clock is edge-for-edge the
//      free clock. A counter on each must agree.
//   B. IT ACTUALLY STOPS. Enable low => the gated clock emits NO edges and a
//      counter on it freezes. This is the property that fails if the gate is
//      inert -- the regression worth fearing, because an inert gate is
//      invisible to functional tests and buys nothing.
//   C. WAKE. Re-enabling resumes counting from the frozen value: the domain
//      was stopped, not reset.
//   D. NO RUNT PULSES. `clk_o` may only fall while `clk_i` is already low.
//      This is what separates a real gate from `assign clk_o = clk_i & en`,
//      which chops the clock mid-high-phase and would destroy the domain.
//
// NEGATIVE CONTROLS -- each guarded property is also applied where it MUST
// fail, and the failure count is asserted non-zero:
//   * B is applied to a GATE_STYLE=0 (pass-through) instance, which by
//     definition keeps toggling.
//   * D is applied to a deliberately naive `clk & enable` gate built here,
//     which by construction produces runts.
// A check that cannot fail proves nothing about the instance it is guarding.
// =============================================================================
`timescale 10ps/1ps

module tb_clk_gate;

    localparam integer TAIL = 4;

    reg clk, reset, enable;

    integer verif_errors, verif_checks;
    `include "../verif/checks.vh"

    integer free_edges, gated_edges, pass_edges;
    integer neg_stop_failures, neg_runt_failures, runt_failures;
    integer held;

    wire clk_gated, clk_pass;
    wire clk_naive = clk & enable;          // NEGATIVE CONTROL: the classic bug

    clk_gate #(.GATE_STYLE(3)) u_gate  (.clk_i(clk), .enable_i(enable), .clk_o(clk_gated));
    clk_gate #(.GATE_STYLE(0)) u_pass  (.clk_i(clk), .enable_i(enable), .clk_o(clk_pass));

    // ── counters, each in its own clock domain ──────────────────────────────
    always @(posedge clk)       free_edges  = free_edges  + 1;
    always @(posedge clk_gated) gated_edges = gated_edges + 1;
    always @(posedge clk_pass)  pass_edges  = pass_edges  + 1;

    // ── Property D, continuously: a gated clock may only fall with clk low ──
    always @(negedge clk_gated) if (clk !== 1'b0) runt_failures = runt_failures + 1;
    always @(negedge clk_naive) if (clk !== 1'b0) neg_runt_failures = neg_runt_failures + 1;

    initial clk = 0;
    always #50 clk = ~clk;

    integer k;
    initial begin
        verif_errors = 0; verif_checks = 0;
        free_edges = 0; gated_edges = 0; pass_edges = 0;
        neg_stop_failures = 0; neg_runt_failures = 0; runt_failures = 0;
        reset = 1; enable = 1;
        repeat (4) @(posedge clk);
        reset = 0;

        // ── A. equivalence while enabled ────────────────────────────────────
        @(negedge clk);                     // zero away from the counting edge
        free_edges = 0; gated_edges = 0;
        repeat (20) @(posedge clk);
        #1 check_eq(gated_edges, free_edges, "A: gated clock matches free clock while enabled");

        // ── B. it actually stops ────────────────────────────────────────────
        // Drop the enable just after a RISING edge, which is when a registered
        // enable really moves. Changing it on the falling edge instead would
        // make the gate look correct whatever it was built from -- and would
        // leave property D unable to fail (see the runt negative control).
        @(posedge clk); #1 enable = 0;
        held = gated_edges; pass_edges = 0; free_edges = 0;
        for (k = 0; k < 20; k = k + 1) begin
            @(posedge clk); #1;
            check_eq(gated_edges, held, "B: gated clock emits no edges while disabled");
            // NEGATIVE CONTROL: the pass-through instance cannot stop.
            if (pass_edges != 0) neg_stop_failures = neg_stop_failures + 1;
        end

        // ── C. wake: resume from the frozen value, not from zero ────────────
        @(posedge clk); #1 enable = 1;
        repeat (10) @(posedge clk);
        #1 check_eq(gated_edges, held + 10, "C: resumes from the value it froze at");
        check_eq(pass_edges, free_edges, "C: GATE_STYLE=0 is edge-for-edge the free clock");

        // ── D. no runt pulses on the real gate ──────────────────────────────
        check_eq(runt_failures, 0, "D: gated clock never falls while clk_i is high");

        // ── the negative controls must have fired ───────────────────────────
        check_true(neg_stop_failures > 0,
                   "NEG CONTROL: pass-through instance kept toggling, as it must");
        check_true(neg_runt_failures > 0,
                   "NEG CONTROL: naive (clk & en) gate produced runt pulses, as it must");
        $display("[clk_gate] negative controls fired: stop %0d, runt %0d",
                 neg_stop_failures, neg_runt_failures);

        ctrl_tests;
        `VERIF_EPILOGUE("tb_clk_gate")
    end

    // ── clk_gate_ctrl ───────────────────────────────────────────────────────
    reg  [3:0] busy;
    reg        wake;
    wire       ctrl_en;
    clk_gate_ctrl #(.NUM_BUSY(4), .TAIL_CYCLES(TAIL))
        u_ctrl (.clk(clk), .reset(reset), .busy_i(busy), .wake_i(wake), .enable_o(ctrl_en));

    task ctrl_tests;
        integer n;
        begin
            busy = 0; wake = 0;
            repeat (TAIL + 4) @(posedge clk);
            #1 check_bit(ctrl_en, 1'b0, "ctrl: idle with no busy and no wake");

            wake = 1; @(posedge clk); #1;
            check_bit(ctrl_en, 1'b1, "ctrl: wake raises the enable");
            wake = 0; busy = 4'b0010; @(posedge clk); #1;
            check_bit(ctrl_en, 1'b1, "ctrl: any busy bit holds the enable");

            // tail: the clock must survive exactly TAIL cycles past the last busy
            busy = 0;
            n = 0;
            while (ctrl_en === 1'b1) begin
                @(posedge clk); #1;
                n = n + 1;
                if (n > TAIL + 4) begin
                    fail_now("ctrl: tail never expired");
                    disable ctrl_tests;
                end
            end
            check_eq(n, TAIL + 1, "ctrl: enable drops TAIL cycles after the last busy");

            // reset must force the clock on, or the domain never sees its reset
            reset = 1; @(posedge clk); #1;
            check_bit(ctrl_en, 1'b1, "ctrl: reset forces the enable high");
            reset = 0;
        end
    endtask

    `VERIF_WATCHDOG(2000000)

endmodule
