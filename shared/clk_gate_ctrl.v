// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
// Authors: Simon Davidson & Claude | Created 2026-09-23 | Last modified 2026-09-23
`timescale 10ps/1ps

// Enable generator for clk_gate: decides when an accelerator clock domain may
// stop. Lives in the FREE-RUNNING domain -- it must keep running in order to
// restart the domain it gates.
//
//   enable_o = |busy_i | wake_i | (tail counter still running)
//
// `busy_i`  one bit per gated unit, high while that unit has work in flight.
//           In FlexMan these are the accelerators' own `acc_busy_o` outputs,
//           the same bits the scheduler consumes as `acc_busy_i`. Include the
//           fill unit's busy if it writes into memories inside the gated
//           domain, or its fills are dropped while the clock is stopped.
// `wake_i`  asserted by the ungated domain BEFORE the gated domain is needed.
//
// DEADLOCK-FREEDOM is by construction, not by argument: `wake_i` is driven from
// logic this module does not gate, so a stopped domain can always be restarted.
// A gated unit can never be responsible for waking itself.
//
// TIMING -- the reason for TAIL_CYCLES. The gate captures its enable while the
// clock is low, so the enable must LEAD the first required rising edge. Two
// mechanisms give that lead:
//   * `enable_o` is registered, so it rises one cycle after the request. Drive
//     `wake_i` from a signal that leads the real start -- in FlexMan, the config
//     manager going busy leads its config-finished pulse by tens of cycles.
//   * TAIL_CYCLES keeps the clock alive after the last busy bit drops, so a
//     trailing pipeline stage cannot be stranded mid-drain. The cost is
//     TAIL_CYCLES of clock per burst, which against a 1.59% duty cycle is noise;
//     the risk it removes -- a too-tight enable failing setup on the gate's CE
//     pin -- is not.
//
// RESET: `enable_o` is forced high while `reset` is asserted and for
// TAIL_CYCLES after its release, so reset always propagates into the gated
// domain. Gating a domain that has not seen its own reset is the other way this
// kind of logic fails silently.
module clk_gate_ctrl #(
    parameter NUM_BUSY    = 4,
    parameter TAIL_CYCLES = 8
)(
    input  wire                clk,      // free-running
    input  wire                reset,
    input  wire [NUM_BUSY-1:0] busy_i,
    input  wire                wake_i,
    output reg                 enable_o
);

    localparam integer CW = (TAIL_CYCLES < 2) ? 1 : $clog2(TAIL_CYCLES + 1);

    wire req = (|busy_i) | wake_i;
    reg [CW-1:0] tail;

    always @(posedge clk) begin
        if (reset) begin
            tail     <= TAIL_CYCLES[CW-1:0];
            enable_o <= 1'b1;
        end else begin
            enable_o <= req | (tail != 0);
            if (req)            tail <= TAIL_CYCLES[CW-1:0];
            else if (tail != 0) tail <= tail - 1'b1;
        end
    end

endmodule
