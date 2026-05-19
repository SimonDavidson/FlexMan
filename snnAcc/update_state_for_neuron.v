// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
// =============================================================================
// Module: update_state_for_neuron
//
// 3-cycle pipeline (was 2-cycle).  The extra stage registers the potential
// multiplier output and spike before the output mux, breaking the previously
// critical combinatorial path:
//
//   ST_IDLE : syn_curr × syn_dcy multiply runs combinatorially.
//             On neuron_valid_i: capture potential_r, threshold_r,
//             decayed_syn_curr_r; advance to ST_C1.
//
//   ST_C1   : potential_r × pot_dcy multiply runs combinatorially.
//             spike wire computed from potential_r >= threshold_r.
//             On clock edge: capture decayed_potential_r, spike_r;
//             advance to ST_C2.
//
//   ST_C2   : result_valid_o=1.  All outputs driven from registered
//             values — no multiplier or comparator on this path.
//             Stall until result_taken_i, then return to ST_IDLE.
//
// Critical paths after this change:
//   ST_IDLE  : syn_curr_i → 32x32 mul → register  (reg-to-reg)
//   ST_C1    : potential_r → 32x32 mul → register  (reg-to-reg)
//   ST_C2    : registers → mux → outputs            (<2 ns)
// =============================================================================

`timescale 1ns / 1ps

module update_state_for_neuron # (
    parameter SYN_CURR_SLICE_BITS  = 8,
    parameter POT_SLICE_BITS       = 8,
    parameter BIAS_CURR_SLICE_BITS = 8,
    parameter THRESH_SLICE_BITS    = 8
)(
    input  wire                                    clk,
    input  wire                                    reset,

    // Input interface (valid/taken handshake)
    input  wire                                    neuron_valid_i,
    input  wire signed [SYN_CURR_SLICE_BITS-1:0]   syn_curr_i,
    input  wire signed [POT_SLICE_BITS-1:0]        potential_i,
    input  wire signed [BIAS_CURR_SLICE_BITS-1:0]  bias_curr_i,
    input  wire signed [THRESH_SLICE_BITS-1:0]     threshold_i,
    input  wire        [SYN_CURR_SLICE_BITS-1:0]   syn_curr_decay_mult_i,   // Q0.32 unsigned
    input  wire        [POT_SLICE_BITS-1:0]        potential_decay_mult_i,  // Q0.32 unsigned
    input  wire                                    sub_on_fire_i,
    output wire                                    neuron_taken_o,

    // Output interface (valid/taken handshake)
    output wire                                    result_valid_o,
    output wire signed [POT_SLICE_BITS-1:0]        potential_o,
    output wire signed [SYN_CURR_SLICE_BITS-1:0]   syn_curr_o,
    output wire                                    spike_o,
    input  wire                                    result_taken_i
);

    // =========================================================================
    // State machine: ST_IDLE → ST_C1 → ST_C2 → (stall or ST_IDLE)
    // =========================================================================
    localparam [1:0] ST_IDLE = 2'd0, ST_C1 = 2'd1, ST_C2 = 2'd2;
    reg [1:0] state_r;

    always @(posedge clk) begin
        if (reset)
            state_r <= ST_IDLE;
        else case (state_r)
            ST_IDLE: if (neuron_valid_i) state_r <= ST_C1;
            ST_C1:                       state_r <= ST_C2;
            ST_C2:   if (result_taken_i) state_r <= ST_IDLE;
            default:                     state_r <= ST_IDLE;
        endcase
    end

    // =========================================================================
    // Potential sum + saturation (combinatorial, evaluated in ST_IDLE)
    // Registered into potential_r and threshold_r on the IDLE→C1 edge.
    // =========================================================================
    wire signed [POT_SLICE_BITS+1:0] new_potential;
    wire                             pot_overflow, pot_underflow;
    wire signed [POT_SLICE_BITS-1:0] saturated_potential;

    assign new_potential  = potential_i + bias_curr_i + syn_curr_i;
    assign pot_overflow   = (~new_potential[POT_SLICE_BITS-1] &&
                             (new_potential[POT_SLICE_BITS+1:POT_SLICE_BITS] != 2'b00));
    assign pot_underflow  = ( new_potential[POT_SLICE_BITS-1] &&
                             (new_potential[POT_SLICE_BITS+1:POT_SLICE_BITS] != 2'b11));
    assign saturated_potential = pot_overflow  ? 32'h7FFFFFFF :
                                 pot_underflow ? 32'h80000000 :
                                                 new_potential[POT_SLICE_BITS-1:0];

    reg signed [POT_SLICE_BITS-1:0]    potential_r;
    reg signed [THRESH_SLICE_BITS-1:0] threshold_r;

    always @(posedge clk) begin
        if (reset) begin
            potential_r <= {POT_SLICE_BITS{1'b0}};
            threshold_r <= {THRESH_SLICE_BITS{1'b0}};
        end else if (state_r == ST_IDLE && neuron_valid_i) begin
            potential_r <= saturated_potential;
            threshold_r <= threshold_i;
        end
    end

    // =========================================================================
    // Shared multiplier — mux selects operands based on pipeline stage:
    //   ST_IDLE / ST_C2 : syn_curr_i    × syn_curr_decay_mult_i
    //   ST_C1            : potential_r  × potential_decay_mult_i
    //
    // Signed × unsigned-Q0.32: result[63:32] is the Q0.32-scaled output.
    // (Signed/unsigned mixing on mul_b is intentional — same as original.)
    // =========================================================================
    wire signed [31:0] mul_a;
    wire        [31:0] mul_b;
    wire signed [63:0] mul_result;

    assign mul_a = (state_r == ST_C1) ? potential_r     : syn_curr_i;
    assign mul_b = (state_r == ST_C1) ? potential_decay_mult_i : syn_curr_decay_mult_i;
    assign mul_result = mul_a * mul_b;

    // Decayed syn_curr: registered on IDLE→C1 edge (multiply ran in ST_IDLE).
    reg signed [SYN_CURR_SLICE_BITS-1:0] decayed_syn_curr_r;

    always @(posedge clk) begin
        if (reset)
            decayed_syn_curr_r <= {SYN_CURR_SLICE_BITS{1'b0}};
        else if (state_r == ST_IDLE && neuron_valid_i)
            decayed_syn_curr_r <= mul_result[63:32];
    end

    // Decayed potential + spike: registered on C1→C2 edge (multiply ran in ST_C1).
    // This is the stage that breaks the old critical path.
    wire spike_wire = (potential_r >= threshold_r);

    reg signed [POT_SLICE_BITS-1:0] decayed_potential_r;
    reg                              spike_r;

    always @(posedge clk) begin
        if (reset) begin
            decayed_potential_r <= {POT_SLICE_BITS{1'b0}};
            spike_r             <= 1'b0;
        end else if (state_r == ST_C1) begin
            decayed_potential_r <= mul_result[63:32];
            spike_r             <= spike_wire;
        end
    end

    // =========================================================================
    // ST_C2 outputs: all from registered values — no multiplier on this path
    // =========================================================================
    assign result_valid_o = (state_r == ST_C2);
    assign spike_o        = spike_r;
    assign syn_curr_o     = decayed_syn_curr_r;
    assign potential_o    = spike_r
        ? (sub_on_fire_i
            ? decayed_potential_r - {{(POT_SLICE_BITS-THRESH_SLICE_BITS){threshold_r[THRESH_SLICE_BITS-1]}}, threshold_r}
            : {POT_SLICE_BITS{1'b0}})
        : decayed_potential_r;

    // Upstream handshake: accept new neuron only when downstream takes result.
    assign neuron_taken_o = result_taken_i;

endmodule
