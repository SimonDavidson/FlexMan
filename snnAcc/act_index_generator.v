// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
`include "../shared/constants.v"

module act_index_generator
                        # (parameter X_INPUT_SZ         = 8,
                           parameter Y_INPUT_SZ         = 8,
                           parameter X_KERNEL_SZ        = 3,
                           parameter Y_KERNEL_SZ        = 3,
                           parameter X_KERNEL_OFF_SZ    = 3,
                           parameter Y_KERNEL_OFF_SZ    = 3,
                           parameter X_STEP_SZ          = 3,
                           parameter Y_STEP_SZ          = 3,
                           parameter ELEMS_PER_ROW      = 4,
                           parameter ROWS_PER_NEURON    = 16,
                           parameter IN_DATA_BITS       = 32,
                           parameter ELEM_SZ            = 8,
		           parameter ACT_IDX_SZ         = `COL_BITS,
		           parameter MEM_ADDR_BITS      = `ADDR_SIZE)
   (
    input  wire                    clk,
    input  wire                    reset,

    // Interface to control signals:
    input  wire                       start_new_block_i,
    input  wire                       running_i,
    input  wire                       next_in_neuron_i, // XXX to remove use act_index_taken
    output wire                       finished_timestep_o,
    output wire                       act_index_gen_running_o,
    
    input  wire                 [1:0] weight_mode_i,     // 00 = full,01 = sparse,
                                                         // 10 = convolutional
    input  wire      [X_INPUT_SZ-1:0] in_x_len_i,        // Vector len in 'full'
    input  wire      [Y_INPUT_SZ-1:0] in_y_len_i,        // == '1' in 'full'
    input  wire     [X_KERNEL_SZ-1:0] x_kernel_len_i,    // Conv only - size of kernel
    input  wire     [Y_KERNEL_SZ-1:0] y_kernel_len_i,    // Conv only - size of kernel
    input  wire       [X_STEP_SZ-1:0] x_kernel_step_i,   // Conv only - step in x-dim
    input  wire       [Y_STEP_SZ-1:0] y_kernel_step_i,   // Conv only - step in y-dim
    input  wire   [ELEMS_PER_ROW-1:0] weights_per_word_i, // How many weight words 
                                                          // in 32-bits
    input  wire [ROWS_PER_NEURON-1:0] rows_per_neuron_i,  // Sparse W-matrix only

    // Addressing for activations (input spikes) memory:
    output wire                       act_index_valid_o,
    output wire      [X_INPUT_SZ-1:0] act_index_x_o,
    output wire      [Y_INPUT_SZ-1:0] act_index_y_o,
    output wire      [ACT_IDX_SZ-1:0] act_index_o,
    output wire                       act_index_last_o,
    input  wire                       act_index_taken_i
);

reg                [X_INPUT_SZ-1:0] in_x_index_r;
reg                [Y_INPUT_SZ-1:0] in_y_index_r;
wire               [X_INPUT_SZ-1:0] in_x_index_nxt;
wire               [Y_INPUT_SZ-1:0] in_y_index_nxt;
wire                [`PIN_BITS-1:0] in_elem_count_nxt;
reg                 [`PIN_BITS-1:0] in_elem_count_r;
reg                                 act_index_gen_running_r;

always @ (posedge clk)
if (reset)
    act_index_gen_running_r = 1'b0;
else if (finished_timestep_o)
    act_index_gen_running_r = 1'b0;
else if (start_new_block_i)
    act_index_gen_running_r = 1'b1;

assign act_index_gen_running_o = act_index_gen_running_r;

////////////////////////////////////////////////////////////////
// Weight mode
//
// Maybe we don't care in this block. Always do activations in order,
// sweeping x (inner loop) and y (outer loop):

assign is_fullConn    = (weight_mode_i == 2'b00)? 1'b1 : 1'b0;
assign is_sparseConn  = (weight_mode_i == 2'b01)? 1'b1 : 1'b0;
assign is_convolution = (weight_mode_i == 2'b10)? 1'b1 : 1'b0;


///////////////////////////////////////////////////////////
//
// Generate activation (spike) index

///////////////////////////////////////////////////////////
// Spike index for full-connectivity & convolutional modes
//

assign  in_x_end_of_row = ( in_x_index_r == ( in_x_len_i-1'b1))? 1'b1 : 1'b0;
assign  in_y_end_of_col = ( in_y_index_r == ( in_y_len_i-1'b1))? 1'b1 : 1'b0;

assign in_x_index_nxt = (act_index_taken_i &  in_x_end_of_row)? 'b0 :
                        (act_index_taken_i)? (in_x_index_r + 1'b1)  : in_x_index_r;

assign in_y_index_nxt = (act_index_taken_i & in_y_end_of_col & in_x_end_of_row)? 'b0 :
                        (act_index_taken_i & in_x_end_of_row)? in_y_index_r + 1'b1   :
                                                               in_y_index_r;

assign in_elem_count_nxt = //(act_index_taken_i)?                    1'b0 :
                           (act_index_taken_i)?  in_elem_count_r + 1'b1 :
                                                 in_elem_count_r;

// Count through the input image, in 2-D.
always @ (posedge clk)
   if (reset | ~act_index_gen_running_r)
      begin
         in_x_index_r    <= 1'b0;
         in_y_index_r    <= 1'b0;
         in_elem_count_r <=  'b0;
      end
   else
      begin
         in_x_index_r    <= in_x_index_nxt;
         in_y_index_r    <= in_y_index_nxt;
         in_elem_count_r <= in_elem_count_nxt;
      end

assign act_index_x_o     = in_x_index_r;
assign act_index_y_o     = in_y_index_r;
assign act_index_valid_o = act_index_gen_running_r;
assign act_index_o       = in_elem_count_r;
assign act_index_last_o  = act_index_gen_running_r & in_x_end_of_row & in_y_end_of_col;

assign finished_timestep_o  = act_index_valid_o & act_index_last_o & act_index_taken_i;

endmodule
