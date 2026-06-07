// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
// =============================================================================
// verif/checks.vh  --  shared self-checking primitives for FlexMan testbenches
//
// Authors      : Simon Davidson & Claude
// Created      : 2026-06-07
// Last modified: 2026-06-07
//
// Usage
// -----
//   Include this file INSIDE the testbench module body (not at $unit scope):
//
//       module tb_foo;
//           integer verif_errors;
//           integer verif_checks;
//           `include "../verif/checks.vh"
//           ...
//           initial begin
//               verif_errors = 0; verif_checks = 0;
//               ...
//               check_eq(got, exp, "label");
//               ...
//               `VERIF_EPILOGUE("tb_foo")
//           end
//           `VERIF_WATCHDOG(200000)   // module-scope, ns
//       endmodule
//
//   The including module MUST declare `integer verif_errors;` and
//   `integer verif_checks;` before the include.  Placing the include inside
//   the module makes these tasks module-local, which is immune to Xcelium
//   cross-file compilation-unit ($unit) visibility ordering.
//
// Exit / pass-fail contract
// -------------------------
//   * Every check uses !== so an X/Z on a DUT output is a FAILURE, never a
//     silent pass.
//   * `VERIF_EPILOGUE prints the literal token "PASS" or "FAIL" on its own
//     line (grepped by run_regression.sh) and, on failure, calls $fatal so the
//     simulator process returns a non-zero exit status.  On success it calls
//     plain $finish (exit status 0).
// =============================================================================

// ---- 64-bit signed compare, decimal + hex display -------------------------
task check_eq;
    input signed [63:0] got;
    input signed [63:0] exp;
    input        [255:0] label;
    begin
        verif_checks = verif_checks + 1;
        if (got !== exp) begin
            $display("FAIL %0s: got %0d (0x%016h)  exp %0d (0x%016h)",
                     label, got, got, exp, exp);
            verif_errors = verif_errors + 1;
        end
    end
endtask

// ---- 64-bit unsigned compare, hex display ---------------------------------
task check_eq_u;
    input [63:0] got;
    input [63:0] exp;
    input [255:0] label;
    begin
        verif_checks = verif_checks + 1;
        if (got !== exp) begin
            $display("FAIL %0s: got 0x%016h (%0d)  exp 0x%016h (%0d)",
                     label, got, got, exp, exp);
            verif_errors = verif_errors + 1;
        end
    end
endtask

// ---- single-bit compare ----------------------------------------------------
task check_bit;
    input got;
    input exp;
    input [255:0] label;
    begin
        verif_checks = verif_checks + 1;
        if (got !== exp) begin
            $display("FAIL %0s: got %b  exp %b", label, got, exp);
            verif_errors = verif_errors + 1;
        end
    end
endtask

// ---- explicit boolean assertion (cond must hold) ---------------------------
task check_true;
    input cond;
    input [255:0] label;
    begin
        verif_checks = verif_checks + 1;
        if (cond !== 1'b1) begin
            $display("FAIL %0s: condition not true (got %b)", label, cond);
            verif_errors = verif_errors + 1;
        end
    end
endtask

// ---- unconditional failure (use in unreachable/illegal branches) ----------
task fail_now;
    input [255:0] label;
    begin
        $display("FAIL %0s", label);
        verif_errors = verif_errors + 1;
    end
endtask

// ---- End-of-test epilogue: prints PASS/FAIL token, sets process exit code --
`define VERIF_EPILOGUE(NAME)                                                   \
    $display("=== %0s: %0d check(s), %0d failure(s) ===",                      \
             NAME, verif_checks, verif_errors);                               \
    if (verif_errors == 0) begin                                              \
        $display("PASS");                                                     \
        $finish;                                                              \
    end else begin                                                            \
        $display("FAIL");                                                     \
        $fatal(1, "%0s: %0d verif failure(s)", NAME, verif_errors);           \
    end

// ---- Global watchdog: fail (not hang) if the test runs too long -----------
// Place at module scope.  NS is the timeout in nanoseconds.
`define VERIF_WATCHDOG(NS)                                                     \
    initial begin                                                             \
        #(NS);                                                                \
        $display("FAIL: global simulation timeout after %0d ns", NS);         \
        $fatal(1, "global timeout");                                          \
    end
