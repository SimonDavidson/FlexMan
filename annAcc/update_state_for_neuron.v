// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
// =============================================================================
// Module: update_state_for_neuron (annAcc variant)
//
// Applies a configurable threshold to the pre-activation potential and
// decays the result.  Two-cycle pipeline (same timing contract as snnAcc):
//   Cycle 1: potential_i → threshold → act_out_r
//   Cycle 2: act_out_r  × decay_mult → decayed_act_o; result_valid_o pulses
//
// Threshold modes (thresh_op_i):
//   2'b00 = RELU — negative pre-activations clamped to 0
//   2'b01 = LUT  — lut_result_i (pre-fetched by lut_cache in neuron_processing)
//                  zero-extended to POT_SLICE_BITS
//   2'b10 = ABS  — |potential_i|; saturates to 0x7FFFFFFF at potential == 0x80000000
// =============================================================================

`timescale 1ns / 1ps

module ann_update_state_for_neuron # (
    parameter POT_SLICE_BITS    = 32,   // pre-activation and output width
    parameter THRESH_SLICE_BITS = 8     // LUT output element width
)(
    input  wire                                clk,
    input  wire                                reset,

    input  wire                    [1:0]       thresh_op_i,            // 00=RELU 01=LUT 10=ABS

    // Input handshake
    input  wire                                neuron_valid_i,
    input  wire signed [POT_SLICE_BITS-1:0]    potential_i,            // pre-activation
    input  wire        [THRESH_SLICE_BITS-1:0] lut_result_i,           // LUT output (LUT mode)
    input  wire        [31:0]                  potential_decay_mult_i,  // Q0.32 unsigned
    output wire                                neuron_taken_o,

    // Output handshake
    output wire                                result_valid_o,
    output wire        [POT_SLICE_BITS-1:0]    act_out_o,              // post-threshold activation
    output wire        [POT_SLICE_BITS-1:0]    decayed_act_o,          // decayed → pot write-back
    input  wire                                result_taken_i
);

    reg                       state_cycle2_r;
    reg [POT_SLICE_BITS-1:0]  act_out_r;

    wire [POT_SLICE_BITS-1:0] act_out_comb;

    // Q0.32 decay multiplier (same pattern as snnAcc; signed/unsigned mixing is intentional)
    wire signed [31:0]        mul_a;
    wire        [31:0]        mul_b;
    wire signed [63:0]        mul_result;

    // ABS: |potential_i|, saturated at max-positive when potential is min-negative
    wire abs_overflow = (potential_i == {1'b1, {(POT_SLICE_BITS-1){1'b0}}});
    wire [POT_SLICE_BITS-1:0] abs_value =
        abs_overflow ? {1'b0, {(POT_SLICE_BITS-1){1'b1}}}
                     : (potential_i[POT_SLICE_BITS-1] ? -potential_i : potential_i);

    // Cycle 1: apply threshold
    assign act_out_comb = (thresh_op_i == 2'b01) ? {{(POT_SLICE_BITS-THRESH_SLICE_BITS){1'b0}}, lut_result_i}
                        : (thresh_op_i == 2'b10) ? abs_value
                        :                          (potential_i[POT_SLICE_BITS-1]
                                                    ? {POT_SLICE_BITS{1'b0}}
                                                    : potential_i[POT_SLICE_BITS-1:0]);

    always @ (posedge clk)
    begin
        if (reset)
            act_out_r <= {POT_SLICE_BITS{1'b0}};
        else if (neuron_valid_i & ~state_cycle2_r)
            act_out_r <= act_out_comb;
    end

    // Cycle-state register
    always @ (posedge clk)
    begin
        if (reset)
            state_cycle2_r <= 1'b0;
        else if (result_taken_i)
            state_cycle2_r <= 1'b0;
        else if (neuron_valid_i)
            state_cycle2_r <= 1'b1;
    end

    // Cycle 2: decay (Q0.32 multiply; upper 32 bits of 64-bit product = decayed value)
    assign mul_a       = act_out_r;
    assign mul_b       = potential_decay_mult_i;
    assign mul_result  = mul_a * mul_b;

    assign decayed_act_o  = mul_result[63:32];
    assign act_out_o      = act_out_r;
    assign result_valid_o = state_cycle2_r;
    assign neuron_taken_o = result_taken_i;

endmodule
