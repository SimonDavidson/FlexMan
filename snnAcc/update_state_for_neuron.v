// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
// =============================================================================
// Module: update_state_for_neuron
// Description: 
// =============================================================================

`timescale 1ns / 1ps

module update_state_for_neuron # (
	parameter SYN_CURR_SLICE_BITS  = 8,
	parameter POT_SLICE_BITS       = 8,
	parameter BIAS_CURR_SLICE_BITS = 8,
	parameter THRESH_SLICE_BITS    = 8
     )
    (
    // Clock and reset
    input  wire                        clk,
    input  wire                        reset,            // Active-high asynchronous reset

    // Input interface (valid/taken handshake)
    input  wire                                   neuron_valid_i,   // Input transaction valid
    input  wire signed [SYN_CURR_SLICE_BITS-1:0]  syn_curr_i,
    input  wire signed [POT_SLICE_BITS-1:0]       potential_i,
    input  wire signed [BIAS_CURR_SLICE_BITS-1:0] bias_curr_i,
    input  wire signed [THRESH_SLICE_BITS-1:0]    threshold_i,
    input  wire        [SYN_CURR_SLICE_BITS-1:0]  syn_curr_decay_mult_i,   // Q0.32 unsigned fractional
    input  wire        [POT_SLICE_BITS-1:0]       potential_decay_mult_i,  // Q0.32 unsigned fractional
    input  wire                                   sub_on_fire_i,
    output wire                                   neuron_taken_o,  // Acknowledge to upstream

    // Output interface (valid/taken handshake)
    output wire                                   result_valid_o,
    output wire signed       [POT_SLICE_BITS-1:0] potential_o,
    output wire signed  [SYN_CURR_SLICE_BITS-1:0] syn_curr_o,
    output wire                                   spike_o,
    input  wire                                   result_taken_i   // Acknowledge from downstream
);

    reg                                           state_cycle2_r;
    wire signed              [POT_SLICE_BITS+1:0] new_potential;
    wire signed              [POT_SLICE_BITS-1:0] saturated_potential;
    reg  signed              [POT_SLICE_BITS-1:0] potential_r;
    wire signed              [POT_SLICE_BITS-1:0] decayed_potential;
    reg  signed         [SYN_CURR_SLICE_BITS-1:0] decayed_syn_curr_r;
    wire                                          pot_overflow;
    wire                                          spike;

    wire                    signed [31:0] mul_a;            // signed operand
    wire                           [31:0] mul_b;            // unsigned Q0.32 decay
    wire                    signed [63:0] mul_result;

/////////////////////////////////////////////////////////////////////////////
// Update potential
//
// Add new synaptic current and bias current to the potential
//

////////////////////////////////////////
// Perform addition:
assign new_potential = potential_i + bias_curr_i + syn_curr_i;

// Check if potential has overflowed or underflowed:
assign pot_overflow  = (
         ~new_potential[POT_SLICE_BITS-1] && 
         (new_potential[POT_SLICE_BITS+1:POT_SLICE_BITS] != 2'b00))? 1'b1:1'b0;

assign pot_underflow = (
	  new_potential[POT_SLICE_BITS-1] && 
	 (new_potential[POT_SLICE_BITS+1:POT_SLICE_BITS] != 2'b11))? 1'b1:1'b0;

////////////////////////////////////////
// Perform saturation on new potential
//
assign saturated_potential = ( pot_overflow)? 32'h7FFFFFFF :
	                     (pot_underflow)? 32'h80000000 :
			                      new_potential[POT_SLICE_BITS-1:0];

//////////////////////////////////////////////////////////////////////////////
// Register New potential into next pipeline stage for thresholding and decay
//

always @ (posedge clk)
begin
   if (reset)
      potential_r <= 'b0;
   else if (neuron_valid_i & ~result_taken_i)
      potential_r <= saturated_potential;
end

//////////////////////////////////////////////////////////////////////////////
// state to remember which if the two cycles we are on (controls multiplier)
//

always @ (posedge clk)
begin
   if (reset)
      state_cycle2_r <= 1'b0;
   else if (result_taken_i)
      state_cycle2_r <= 1'b0;
   else if (neuron_valid_i)
      state_cycle2_r <= 1'b1;
end


//////////////////////////////////////////////////////////////////////////////
// Neuron thresholding - compare potential with given threshold
// Generates a spike signal, 1 = spike; 0 = no-spike

assign spike = (potential_r >=threshold_i) ? 1'b1 : 1'b0;


//////////////////////////////////////////////////////////////////////////////
// Multiplier For potential and synaptic current decay
// Select between sources for inputs a and b:
// Cycle 1) a = synaptic current; b = syn_curr decay factor
// Cycle 2) a = new potential; b = potential decay factor
//
assign mul_a = (state_cycle2_r) ? potential_r : syn_curr_i;
assign mul_b = (state_cycle2_r) ? potential_decay_mult_i : syn_curr_decay_mult_i;

assign mul_result = mul_a * mul_b;

// This is only valid on second cycle:
assign decayed_potential = mul_result[63:32];

// Register output of multiplier on cycle 1, as this is th decayed syn_current

always @ (posedge clk)
begin
   if (reset)
      decayed_syn_curr_r <= 'b0;
   else if (neuron_valid_i & ~state_cycle2_r)
      decayed_syn_curr_r <= mul_result[63:32];
end


//////////////////////////////////////////////////////////////////////////////
// Seelct output potential - either decayed or zeroed value, depending
// on whether or not we generated a spike

assign potential_o = spike ? (sub_on_fire_i
                             ? decayed_potential - {{(POT_SLICE_BITS-THRESH_SLICE_BITS){threshold_i[THRESH_SLICE_BITS-1]}}, threshold_i}
                             : 'b0)
                           : decayed_potential;

// Assign output signals:
assign spike_o = spike;

assign syn_curr_o = decayed_syn_curr_r;

assign result_valid_o = state_cycle2_r;

// Acknowledge back to upstream pipeline comes from acknowledge into this
// stage:
assign neuron_taken_o = result_taken_i;

endmodule
