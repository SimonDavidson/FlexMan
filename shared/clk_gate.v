// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
// Authors: Simon Davidson & Claude | Created 2026-09-23 | Last modified 2026-09-23
`timescale 10ps/1ps

// Portable gated-clock cell.
//
// GATE_STYLE = 0 (DEFAULT) : pass-through. `clk_o` IS `clk_i`, `enable_i` is
//                            unread, and the synthesised result is provably
//                            identical to a build that never instantiated this
//                            module. Every existing deployment keeps this.
//              1 : Xilinx BUFGCE.
//              2 : Intel/Altera clock-control buffer.
//              3 : generic latch-based ICG, for an ASIC flow or for a family
//                  with no dedicated cell.
//
// Styles 1-3 all implement the SAME behaviour: `enable_i` is captured into a
// transparent-low latch and ANDed with the clock, so the gate opens and closes
// only while the clock is low and `clk_o` can never emit a runt pulse. That is
// exactly what BUFGCE does internally, which is why the behavioural fallback
// below is a faithful model of it rather than an approximation.
//
// The enable is therefore sampled on the FALLING edge of `clk_i`. A consumer
// must raise `enable_i` at least one full cycle before the first rising edge it
// needs -- see clk_gate_ctrl, which registers the enable and provides a lead-in
// for exactly this reason.
//
// Measured motivation: with the Bosch design idle, the clock tree is the whole
// remaining idle-dynamic term -- 13 of 20 mW on xczu7ev, and ~100% of the ~15 mW
// on a right-sized 7-series part, where the memories already stop under RD_GATE
// and DSP idle is zero. Idle dominates the deployed duty cycle (1.59%).
//
// Simulation note: style 1 instantiates the real BUFGCE primitive only when
// XILINX_BUFGCE is defined (the synthesis flow passes it). Undefined -- as in
// every testbench -- it builds the behavioural equivalent above, so the unit
// tests need no vendor library.
module clk_gate #(
    parameter GATE_STYLE = 0
)(
    input  wire clk_i,
    input  wire enable_i,
    output wire clk_o
);

    generate
    if (GATE_STYLE == 0) begin : g_passthrough
        // `enable_i` deliberately unread: this must be inert by construction.
        assign clk_o = clk_i;

    end else if (GATE_STYLE == 1) begin : g_bufgce
`ifdef XILINX_BUFGCE
        // BUFGCE captures CE while I is low; feed it the raw enable so the
        // hardware does its own capture rather than double-latching.
        BUFGCE u_bufgce (.I(clk_i), .CE(enable_i), .O(clk_o));
`else
        reg en_latch;
        always @(*) if (!clk_i) en_latch = enable_i;
        assign clk_o = clk_i & en_latch;
`endif

    end else if (GATE_STYLE == 2) begin : g_altclkctrl
`ifdef ALTERA_CLKCTRL
        altclkctrl u_altclkctrl (.inclk(clk_i), .ena(enable_i), .outclk(clk_o));
`else
        reg en_latch;
        always @(*) if (!clk_i) en_latch = enable_i;
        assign clk_o = clk_i & en_latch;
`endif

    end else begin : g_icg
        // Generic integrated clock gate: transparent-low enable latch + AND.
        reg en_latch;
        always @(*) if (!clk_i) en_latch = enable_i;
        assign clk_o = clk_i & en_latch;
    end
    endgenerate

endmodule
