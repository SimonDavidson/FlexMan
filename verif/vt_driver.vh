// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
// =============================================================================
// verif/vt_driver.vh  --  valid/taken handshake driver+monitor helpers
//
// Authors      : Simon Davidson & Claude
// Created      : 2026-06-07
// Last modified: 2026-06-07
//
// The universal FlexMan flow-control protocol: a producer asserts *_valid_o and
// holds data stable until the consumer asserts *_taken_i.  These macros promote
// the canonical, pipeline-depth-agnostic drive pattern (poll the valid, then
// pulse the taken) into reusable form.
//
// Requires `clk` in scope.  *_WAIT_FINISH requires `integer verif_to;` and the
// `verif_errors` counter (from checks.vh) declared in the TB.
//
// Usage:
//   `VT_PULSE(start_new_block)             // one-cycle strobe
//   `VT_CONSUME(result_valid, result_taken)// wait for valid, then take 1 cycle
//   `VT_WAIT_FINISH(acc_finished, 500)     // bounded wait; FAILS (not hangs) on timeout
// =============================================================================

// One-cycle high pulse on SIG (e.g. start_new_block_i, neuron_valid_i).
`define VT_PULSE(SIG)                                                          \
    begin SIG = 1'b1; @(posedge clk); #1; SIG = 1'b0; end

// Wait until VALID is asserted, then pulse TAKEN for one cycle. Depth-agnostic.
`define VT_CONSUME(VALID, TAKEN)                                              \
    begin                                                                     \
        TAKEN = 1'b0;                                                          \
        while (!VALID) @(posedge clk);                                         \
        #1;                                                                    \
        TAKEN = 1'b1; @(posedge clk); #1;                                      \
        TAKEN = 1'b0;                                                          \
    end

// Wait up to MAXC cycles for FIN to assert. If it never does, record a failure
// (does not hang). Uses `verif_to` (integer, declared in TB) as the counter.
`define VT_WAIT_FINISH(FIN, MAXC)                                            \
    begin                                                                     \
        verif_to = (MAXC);                                                    \
        while (!(FIN) && verif_to > 0) begin                                  \
            @(posedge clk); #1; verif_to = verif_to - 1;                      \
        end                                                                   \
        if (verif_to == 0) begin                                             \
            $display("FAIL: timeout waiting for %m / FIN after %0d cycles", (MAXC)); \
            verif_errors = verif_errors + 1;                                  \
        end                                                                   \
    end
