// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
// =============================================================================
// Module: ann_update_state_for_neuron (annAcc variant)
//
// Applies a configurable threshold to the pre-activation potential and
// decays the result.  3-cycle pipeline (was 2-cycle).  The extra stage
// registers the multiplier output, breaking the previously critical path:
//
//   ST_IDLE : threshold applied combinatorially (RELU/LUT/ABS).
//             On neuron_valid_i: register act_out_r; advance to ST_C1.
//
//   ST_C1   : act_out_r x decay_mult multiply runs combinatorially.
//             On clock edge: register decayed_act_r; advance to ST_C2.
//
//   ST_C2   : result_valid_o=1.  Both outputs driven from registered
//             values — no multiplier on this path.
//             Stall until result_taken_i, then return to ST_IDLE.
//
// Critical paths after this change:
//   ST_IDLE  : threshold combinatorial logic only        (<2 ns)
//   ST_C1    : act_out_r -> 32x32 mul -> register        (reg-to-reg)
//   ST_C2    : registers -> outputs                       (<1 ns)
//
// Threshold modes (thresh_op_i):
//   2'b00 = RELU — negative pre-activations clamped to 0
//   2'b01 = LUT  — lut_result_i zero-extended to POT_SLICE_BITS
//   2'b10 = ABS  — |potential_i|; saturates to 0x7FFFFFFF at min-negative
// =============================================================================

`timescale 1ns / 1ps

module ann_update_state_for_neuron # (
    parameter POT_SLICE_BITS    = 32,
    parameter THRESH_SLICE_BITS = 8,
    parameter POT_DECAY_BITS    = 32    // D_pot (unsigned Q0.D_pot)
)(
    input  wire                                clk,
    input  wire                                reset,

    input  wire                    [1:0]       thresh_op_i,

    // Input handshake
    input  wire                                neuron_valid_i,
    input  wire signed [POT_SLICE_BITS-1:0]    potential_i,
    input  wire        [THRESH_SLICE_BITS-1:0] lut_result_i,
    input  wire        [POT_DECAY_BITS-1:0]    potential_decay_mult_i,
    output wire                                neuron_taken_o,

    // Output handshake
    output wire                                result_valid_o,
    output wire        [POT_SLICE_BITS-1:0]    act_out_o,
    output wire        [POT_SLICE_BITS-1:0]    decayed_act_o,
    input  wire                                result_taken_i
);

    // =========================================================================
    // State machine: ST_IDLE -> ST_C1 -> ST_C2 -> (stall or ST_IDLE)
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
    // ST_IDLE: threshold combinatorial, registered on IDLE->C1 edge
    // =========================================================================
    wire abs_overflow = (potential_i == {1'b1, {(POT_SLICE_BITS-1){1'b0}}});
    wire [POT_SLICE_BITS-1:0] abs_value =
        abs_overflow ? {1'b0, {(POT_SLICE_BITS-1){1'b1}}}
                     : (potential_i[POT_SLICE_BITS-1] ? -potential_i : potential_i);

    wire [POT_SLICE_BITS-1:0] act_out_comb =
        (thresh_op_i == 2'b01) ? {{(POT_SLICE_BITS-THRESH_SLICE_BITS){1'b0}}, lut_result_i}
      : (thresh_op_i == 2'b10) ? abs_value
      :                          (potential_i[POT_SLICE_BITS-1]
                                  ? {POT_SLICE_BITS{1'b0}}
                                  : potential_i[POT_SLICE_BITS-1:0]);

    reg [POT_SLICE_BITS-1:0] act_out_r;

    always @(posedge clk) begin
        if (reset)
            act_out_r <= {POT_SLICE_BITS{1'b0}};
        else if (state_r == ST_IDLE && neuron_valid_i)
            act_out_r <= act_out_comb;
    end

    // =========================================================================
    // ST_C1: decay multiply — act_out_r x decay_mult (Q0.D_pot)
    // Registered into decayed_act_r on C1->C2 edge.
    // (Signed/unsigned mixing on mul_b is intentional — same as original.)
    //
    // Full S+D product internally; extract the high POT_SLICE_BITS bits
    // above the POT_DECAY_BITS fractional bits. Synthesis trims any bits
    // we never consume.
    // =========================================================================
    localparam MUL_R_BITS = POT_SLICE_BITS + POT_DECAY_BITS;

    wire signed [POT_SLICE_BITS-1:0] mul_a;
    wire        [POT_DECAY_BITS-1:0] mul_b;
    wire signed [MUL_R_BITS-1:0]     mul_result;

    assign mul_a      = act_out_r;
    assign mul_b      = potential_decay_mult_i;
    assign mul_result = mul_a * mul_b;

    reg [POT_SLICE_BITS-1:0] decayed_act_r;

    always @(posedge clk) begin
        if (reset)
            decayed_act_r <= {POT_SLICE_BITS{1'b0}};
        else if (state_r == ST_C1)
            decayed_act_r <= mul_result[POT_DECAY_BITS + POT_SLICE_BITS - 1
                                         : POT_DECAY_BITS];
    end

    // =========================================================================
    // ST_C2 outputs: purely from registered values — no multiplier on path
    // =========================================================================
    assign result_valid_o = (state_r == ST_C2);
    assign act_out_o      = act_out_r;
    assign decayed_act_o  = decayed_act_r;

    assign neuron_taken_o = result_taken_i;

endmodule
