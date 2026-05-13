// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
/////////////////////////////////////////////////////////////////////
//
// spike_processing
//
// Input is a set of activations (spikes) and a set of synaptic 
// connection strengths (weights). Output is an updated set of 
// synaptic currents, to pass to the neuron models in the next stage.
//

`include "../shared/constants.v"

module ann_spike_processing # (parameter NUM_TIMESTEPS      = 32,
	                   parameter X_INPUT_SZ         = 8,
                           parameter Y_INPUT_SZ         = 8,
                           parameter X_OUTPUT_SZ        = 8,
                           parameter Y_OUTPUT_SZ        = 8,
                           parameter X_KERNEL_SZ        = 3,
                           parameter Y_KERNEL_SZ        = 3,
                           parameter X_KERNEL_OFF_SZ    = 3,
                           parameter Y_KERNEL_OFF_SZ    = 3,
                           parameter X_STEP_SZ          = 3,
                           parameter Y_STEP_SZ          = 3,
                           parameter ELEMS_PER_ROW      = 4,
                           parameter ROWS_PER_NEURON    = 16,
			   parameter TIMESTEP_SZ        = 10,
                           parameter IN_DATA_BITS       = 32,
                           parameter ELEM_SZ            = 8,
		           parameter ACT_IDX_SZ         = `COL_BITS,
		           parameter ACT_SLICE_SZ       = 5,
		           parameter ACT_DATA_IDX_SZ    = 5,
                           parameter WEIGHT_ENTRY_BITS  = 8,
                           parameter WEIGHT_IDX_SZ      = 5, // 2^5 = 32-bit
                           parameter WEIGHT_SLICE_SZ    = 5, // 2^5 = 32-bit
                           parameter WEIGHT_DATA_IDX_SZ = 5, // 2^5 = 32-bit
                           parameter SYN_CURR_IDX_SZ    = 10,
                           parameter SYN_CURR_DATA_IDX_SZ = 5,
                           parameter SYN_CURR_SLICE_SZ  = 3,
                           parameter SYN_CURR_SLICE_BITS= 10,
                           parameter BIAS_CURR_IDX_SZ   = 2,
                           parameter BIAS_CURR_DATA_IDX_SZ = 5,
			   parameter BIAS_CURR_SLICE_SZ = 3,
			   parameter BIAS_CURR_SLICE_BITS= 8,
			   parameter SPARSE_IDX_SZ      = 16,
		           parameter MEM_ADDR_BITS      = `ADDR_SIZE)(
    input  wire                    clk,
    input  wire                    reset,

    // Config registers (driven from acc_snn_processor)
    input wire   [MEM_ADDR_BITS-1:0] act_base_addr_i,
    input wire   [MEM_ADDR_BITS-1:0] weight_base_addr_i,
    input wire   [MEM_ADDR_BITS-1:0] syn_curr_base_addr_i,
    input wire [WEIGHT_SLICE_SZ-1:0] weight_sz_i,
    input wire                 [4:0] bin_point_syn_curr_i,
    input wire                 [1:0] weight_mode_i,
    input wire                 [2:0] act_slice_sz_i,   // runtime activation element width
    input wire    [X_INPUT_SZ-1:0] in_x_len_i,
    input wire    [Y_INPUT_SZ-1:0] in_y_len_i,
    input wire   [X_OUTPUT_SZ-1:0] out_x_len_i,
    input wire   [Y_OUTPUT_SZ-1:0] out_y_len_i,
    input wire     [X_KERNEL_SZ-1:0] x_kernel_len_i,
    input wire     [Y_KERNEL_SZ-1:0] y_kernel_len_i,
    input wire [X_KERNEL_OFF_SZ-1:0] x_kernel_offset_i,
    input wire [Y_KERNEL_OFF_SZ-1:0] y_kernel_offset_i,
    input wire       [X_STEP_SZ-1:0] x_kernel_step_i,
    input wire       [Y_STEP_SZ-1:0] y_kernel_step_i,
    input wire [WEIGHT_SLICE_SZ-1:0] index_sz_i,
    input wire [WEIGHT_SLICE_SZ-1:0] tuple_sz_i,
    input wire     [`PIN_BITS-1:0]   sparse_count_i,
    input wire  [ELEMS_PER_ROW-1:0] weights_per_word_i,
    input wire [ROWS_PER_NEURON-1:0] rows_per_neuron_i,
    input wire  [WEIGHT_IDX_SZ-1:0] weight_idx_sz_i,

    // Interface to Scheduler:
    input  wire                     start_new_block_i,
    input  wire   [`TGT_ACC_SZ-1:0] target_acc_i,
    input  wire [`SCH_ENTRY_SZ-1:0] buffer_info_i,
    output wire                     spike_proc_finished_o,
    output wire                     acc_busy_o,
    output wire                     acc_finished_o,

    // Interface to the buffer_addr block:
    // Sends base addresses for buffers required for
    // the new task:
    input wire      [`PIN_BITS-1:0] src1_buff_addr_i, // in-spikes
    input wire      [`PIN_BITS-1:0] src2_buff_addr_i, // weights
    input wire      [`PIN_BITS-1:0] src3_buff_addr_i, // not used
    input wire      [`PIN_BITS-1:0]  tgt_buff_addr_i, // syn_currents
    input wire      [`PIN_BITS-1:0] weight_row_len_i, // Bits per row
 
    /* Memory interfaces */
    output wire                  weight_mem_rd_o,
    input  wire                  weight_mem_wait_i,
    output wire [`ADDR_SIZE-1:0] weight_mem_addr_o,
    input  wire  [`WTD_BITS-1:0] weight_mem_data_i,

    output wire                  act_mem_req_o,
    input  wire                  act_mem_wait_i,
    output wire [`ADDR_SIZE-1:0] act_mem_addr_o,
    input  wire  [`ACT_BITS-1:0] act_mem_data_i,

    output wire                  syn_curr_mem_wr_o,
    output wire                  syn_curr_mem_rd_o,
    input  wire                  syn_curr_mem_wait_i,
    output wire [`ADDR_SIZE-1:0] syn_curr_mem_addr_o,
    output wire  [`POT_BITS-1:0] syn_curr_mem_data_wr_o,
    input  wire  [`POT_BITS-1:0] syn_curr_mem_data_rd_i


);

localparam ACT_IDX_BITS      = 2**ACT_IDX_SZ;
localparam ACT_BITS          = 2**ACT_SLICE_SZ;
localparam WEIGHT_IDX_BITS   = 2**WEIGHT_IDX_SZ;
localparam WEIGHT_BITS       = 2**WEIGHT_SLICE_SZ;
localparam SYN_CURR_IDX_BITS = 2**SYN_CURR_IDX_SZ;

reg                       running_r;
reg     [TIMESTEP_SZ-1:0] iteration_count_r;
wire                      syn_curr_update_running;
wire                      act_index_gen_running;
wire                      weight_gen_running;
wire                      finished_timestep_act_index_gen;
wire                      finished_one_pass_weight_gen;
wire                      finished_pass_weight_gen;
wire                      finished_pass_syn_curr_update;
wire                      done_last_iteration;
wire [WEIGHT_ENTRY_BITS-1:0] weight_entry_bits;
wire                      act_index_valid;
wire                      act_index_last;
wire     [ACT_IDX_SZ-1:0] act_index;
wire     [X_INPUT_SZ-1:0] act_index_x;
wire     [Y_INPUT_SZ-1:0] act_index_y;
wire                      act_data_valid;
wire                      act_data_gated_valid;
wire                      act_ignore_non_spike;
wire       [ACT_BITS-1:0] act_data_out;
wire     [X_INPUT_SZ-1:0] act_data_idx_x;
wire     [Y_INPUT_SZ-1:0] act_data_idx_y;
wire[ACT_DATA_IDX_SZ-1:0] act_data_idx;
wire                      act_data_last;
wire                      act_index_wait;
wire                      weight_index_valid;
wire  [WEIGHT_IDX_SZ-1:0] weight_index;
wire    [X_OUTPUT_SZ-1:0] weight_index_x;
wire    [Y_OUTPUT_SZ-1:0] weight_index_y;
wire                      weight_index_last;
wire     [X_INPUT_SZ-1:0] weight_data_index_x;
wire     [Y_INPUT_SZ-1:0] weight_data_index_y;
//wire  [WEIGHT_DATA_IDX_SZ-1:0] weight_data_idx;
wire                      weight_value_valid;
wire    [WEIGHT_BITS-1:0] weight_value;
wire                      weight_index_taken;
wire                      weight_value_taken;
wire                      act_data_taken;


////////////////////////////////////////////////////////
// Modules:
//
// (i)   Input activation address generation, for all modes
// (ii)  Data-cache for activations
// (iii) Weight address generator, for all modes
// (iv)  Data-cache for weights
// (v)   Data-cache for syn_currents (driven by weight addr-gen)
// (vi)  syn_current update and writeback

////////////////////////////////////////////////////////
// Global control state & signals
//
// Manages the running/sleep mode, 
// Maintains timestep count
// Manages colour mode
//
// 
///////////////////////////////////
// running_r
// Global signal saying that we are processing a block.
// Receives 'go' signal to start then waits for individual 
// 'done' signals from the various blocks and clears everything
// once all blocks have signaled 'done':
//
//always @ (posedge clk)
//if (reset)
//   running_r <= 1'b0;
//else if (start_new_block_i)
//   running_r <= 1'b1;
//else if (done_last_iteration)
//   running_r <= 1'b0;

////////////////////////////////////
// Iteration_count_r
//
// Counts number of interations and switches off running when we have
// reached the required number of steps. 

always @ (posedge clk)
if (reset)
begin
	running_r         <= 1'b0;
	iteration_count_r <=  'b0;
end
else if (start_new_block_i)
begin
	running_r         <= 1'b1;
        iteration_count_r <= iteration_count_r + 1;
end
else if (spike_proc_finished_o)
begin
	running_r         <= 1'b0;
	iteration_count_r <=  'b0;
end

assign spike_proc_finished_o = running_r               & 
	                      ~act_index_gen_running   & 
	                      ~weight_gen_running      & 
			      ~syn_curr_update_running;

assign acc_busy_o     = running_r;
assign acc_finished_o = spike_proc_finished_o;

//assign done_last_iteration = (finished_timestep_act_index_gen & 
//	                      finished_pass_weight_gen  & 
//	                     (iteration_count_r == total_timesteps_i))? 1'b1 : 
//			                                                1'b0;

assign done_last_iteration = (finished_timestep_act_index_gen & 
	                      finished_pass_weight_gen)?  1'b1 : 
			                                  1'b0;

//assign finished_timestep = 1'b0; // XXX Add module finished signals here

////////////////////////////////////////////////////////////////
// Act Index Generation
//
act_index_generator #(
   .X_INPUT_SZ(X_INPUT_SZ),
   .Y_INPUT_SZ(Y_INPUT_SZ),
   .X_KERNEL_SZ(X_KERNEL_SZ),
   .Y_KERNEL_SZ(Y_KERNEL_SZ),
   .X_KERNEL_OFF_SZ(X_KERNEL_OFF_SZ),
   .Y_KERNEL_OFF_SZ(Y_KERNEL_OFF_SZ),
   .X_STEP_SZ(X_STEP_SZ),
   .Y_STEP_SZ(Y_STEP_SZ),
   .ELEMS_PER_ROW(ELEMS_PER_ROW),
   .ROWS_PER_NEURON(ROWS_PER_NEURON),
   .IN_DATA_BITS(IN_DATA_BITS),
   .ELEM_SZ(ELEM_SZ),
   .ACT_IDX_SZ(ACT_IDX_SZ),
   .MEM_ADDR_BITS(MEM_ADDR_BITS))
   act_index_gen0
   (
    .clk(clk),
    .reset(reset),

    .start_new_block_i(start_new_block_i),
    .running_i(running_r),
    .next_in_neuron_i(next_in_neuron),
    .finished_timestep_o(finished_timestep_act_index_gen),
    .act_index_gen_running_o(act_index_gen_running),
    .weight_mode_i(weight_mode_i),
    .in_x_len_i(in_x_len_i),
    .in_y_len_i(in_y_len_i),
    .x_kernel_len_i(x_kernel_len_i),
    .y_kernel_len_i(y_kernel_len_i),
    .x_kernel_step_i(x_kernel_step_i),
    .y_kernel_step_i(y_kernel_step_i),
    .weights_per_word_i(weights_per_word_i),
    .rows_per_neuron_i(rows_per_neuron_i),
    .act_index_valid_o(act_index_valid),
    .act_index_x_o(act_index_x),
    .act_index_y_o(act_index_y),
    .act_index_o(act_index),
    .act_index_last_o(act_index_last),
    .act_index_taken_i(act_index_taken)
);


assign next_in_neuron = finished_pass_syn_curr_update;

/////////////////////////////////////////
// * Input spike fetcher
//
// Read from input spike buffer.
dataline_cache_with_xy #(
    .IN_DATA_BITS(IN_DATA_BITS),
    .X_INPUT_SZ(X_INPUT_SZ),
    .Y_INPUT_SZ(Y_INPUT_SZ),
    .IDX_ADDR_BITS(ACT_IDX_SZ),
    .SLICE_DATA_IDX_SZ(ACT_DATA_IDX_SZ),
    .SLICE_SIZE_SZ(ACT_SLICE_SZ),
    .OUT_DATA_BITS(ACT_BITS))

    act_cache (
    .clk(clk),
    .reset(reset),

    .colour_select_o(),
    .slice_sz_i({{(ACT_SLICE_SZ-3){1'b0}}, act_slice_sz_i}),
    .base_addr_i(act_base_addr_i),

    .sys_addr_i(act_index),
    //.sys_data_o(act_data_out),
    .sys_req_i(act_index_valid),
    .sys_index_x_i(act_index_x),
    .sys_index_y_i(act_index_y),
    .sys_colour_i(1'b0),
    .sys_wait_o(act_index_wait),
    .sys_last_i(act_index_last),

    .slice_data_valid_o(act_data_valid),
    .slice_data_idx_o(act_data_idx),
    .slice_data_index_x_o(act_data_idx_x),
    .slice_data_index_y_o(act_data_idx_y),
    .slice_data_o(act_data_out),
    .slice_data_last_o(act_data_last),
    .slice_data_taken_i(act_data_taken),

    .mem_addr_o(act_mem_addr_o),
    .mem_data_i(act_mem_data_i),
    .mem_req_o(act_mem_req_o),
    .mem_wait_i(act_mem_wait_i)
);

// Only pass on activation when it is high (spike not gap).
assign act_data_gated_valid = act_data_valid & (act_data_out != {ACT_BITS{1'b0}});

// Dump valid activations that are zero (non-spike) without passing on:
assign act_ignore_non_spike = act_data_valid & (act_data_out == {ACT_BITS{1'b0}});

// Get next activation if we've processed the connections from an input spike:
assign act_data_taken = finished_one_pass_weight_gen; // | 
                           //(act_data_valid & ~act_data_out[ACT_BITS-1]);

//assign act_index_taken = ~act_index_wait;
// Move onto next actovation either when the current spike has been processed
// or when the current activation is a non-spike, so we dump it and move on:
assign act_index_taken = act_data_taken | act_ignore_non_spike & ~act_index_wait;

////////////////////////////////////////////////////////////////
// Weight Index Generation
//
weight_generator #(
   .X_INPUT_SZ(X_INPUT_SZ),
   .Y_INPUT_SZ(Y_INPUT_SZ),
   .X_OUTPUT_SZ(X_OUTPUT_SZ),
   .Y_OUTPUT_SZ(Y_OUTPUT_SZ),
   .X_KERNEL_SZ(X_KERNEL_SZ),
   .Y_KERNEL_SZ(Y_KERNEL_SZ),
   .X_KERNEL_OFF_SZ(X_KERNEL_OFF_SZ),
   .Y_KERNEL_OFF_SZ(Y_KERNEL_OFF_SZ),
   .X_STEP_SZ(X_STEP_SZ),
   .Y_STEP_SZ(Y_STEP_SZ),
   .ELEMS_PER_ROW(ELEMS_PER_ROW),
   .ROWS_PER_NEURON(ROWS_PER_NEURON),
   .IN_DATA_BITS(IN_DATA_BITS),
   .ELEM_SZ(ELEM_SZ),
   .ACT_IDX_SZ(ACT_DATA_IDX_SZ),
   .ACT_DATA_SZ(ACT_BITS),
   .WEIGHT_ENTRY_BITS(WEIGHT_ENTRY_BITS),
   .WEIGHT_IDX_SZ(WEIGHT_IDX_SZ),
   .WEIGHT_SLICE_SZ(WEIGHT_SLICE_SZ),
   .WEIGHT_DATA_IDX_SZ(WEIGHT_DATA_IDX_SZ),
   .MEM_ADDR_BITS(MEM_ADDR_BITS))
   weight_gen0
   (
    .clk(clk),
    .reset(reset),

    .start_new_block_i(start_new_block_i),
    .running_i(running_r),
    .finished_one_pass_o(finished_one_pass_weight_gen),
    .finished_pass_o(finished_pass_weight_gen),
    .running_weight_pass_o(weight_gen_running),
    .weight_mode_i(weight_mode_i),
    .in_x_len_i(in_x_len_i),
    .in_y_len_i(in_y_len_i),
    .out_x_len_i(out_x_len_i),
    .out_y_len_i(out_y_len_i),
    .x_kernel_len_i(x_kernel_len_i),
    .y_kernel_len_i(y_kernel_len_i),
    .x_kernel_offset_i(x_kernel_offset_i),
    .y_kernel_offset_i(y_kernel_offset_i),
    .x_kernel_step_i(x_kernel_step_i),
    .y_kernel_step_i(y_kernel_step_i),
    .weight_base_addr_i(weight_base_addr_i),
    .weight_sz_i(weight_sz_i),
    .tuple_sz_i(tuple_sz_i),
    .sparse_count_i(sparse_count_i),
    .weight_entry_bits_i(weight_entry_bits),
    .weights_per_word_i(weights_per_word_i),
    .rows_per_neuron_i(rows_per_neuron_i),
    .act_data_valid_i(act_data_gated_valid), // Only process true spikes
    .act_data_x_i(act_data_idx_x),
    .act_data_y_i(act_data_idx_y),
    .act_data_idx_i(act_data_idx),
    .act_data_i(act_data_out),
    .act_data_last_i(act_data_last),
    .weight_mem_rd_o(weight_mem_rd_o),
    .weight_mem_wait_i(weight_mem_wait_i),
    .weight_mem_addr_o(weight_mem_addr_o),
    .weight_mem_data_i(weight_mem_data_i),
    .weight_index_valid_o(weight_index_valid),
    .weight_index_x_o(weight_index_x),
    .weight_index_y_o(weight_index_y),
    .weight_index_o(weight_index),
    .weight_index_last_o(weight_index_last),
    .weight_index_taken_i(weight_index_taken),
    .weight_value_valid_o(weight_value_valid),
    .weight_value_o(weight_value),
    .weight_value_taken_i(weight_value_taken)
);

//assign weight_index_taken = weight_value_valid; // XXXX
//assign weight_value_taken = weight_value_valid; // XXXX

////////////////////////////////////////////////////////////////
// Sparse-mode tuple unpacker
//
// In sparse mode (weight_mode_i == 2'b01), weight_value coming from
// weight_generator carries a TUPLE (because the cache is configured
// to fetch tuple-sized slices). The tuple is left-justified in the
// 32-bit slice port. Within the tuple:
//   [ index_sz bits | weight_sz bits | trailing zeros to tuple boundary ]
// index_sz and weight_sz are independent powers of two; tuple_sz is the
// smallest power of two >= index_sz + weight_sz.
//
// Unpack the tuple combinationally into:
//   - actual_index : the parsed output index (right-justified)
//   - actual_weight: the parsed weight value (right-justified)
//
// In full / conv mode, the weight_value pathway and weight_index_x/y
// are passed through unchanged.

reg [SPARSE_IDX_SZ-1:0] actual_index;
reg     [WEIGHT_BITS-1:0] actual_weight;

// Index always occupies the top index_sz bits of the 32-bit slice port
always @* begin
   case (index_sz_i)
      3'b000: actual_index = {{(SPARSE_IDX_SZ-1){1'b0}}, weight_value[WEIGHT_BITS-1]};
      3'b001: actual_index = {{(SPARSE_IDX_SZ-2){1'b0}}, weight_value[WEIGHT_BITS-1 -: 2]};
      3'b010: actual_index = {{(SPARSE_IDX_SZ-4){1'b0}}, weight_value[WEIGHT_BITS-1 -: 4]};
      3'b011: actual_index = {{(SPARSE_IDX_SZ-8){1'b0}}, weight_value[WEIGHT_BITS-1 -: 8]};
      3'b100: actual_index = weight_value[WEIGHT_BITS-1 -: 16];
      default: actual_index = {SPARSE_IDX_SZ{1'b0}};
   endcase
end

// Weight occupies the weight_sz bits immediately following the index_sz bits,
// left-justified in the tuple, which is itself left-justified in weight_value.
// The weight MSB position depends only on index_sz (not tuple_sz):
//   index=1b  (000) → weight MSB at WEIGHT_BITS-2  = bit 30
//   index=2b  (001) → weight MSB at WEIGHT_BITS-3  = bit 29
//   index=4b  (010) → weight MSB at WEIGHT_BITS-5  = bit 27
//   index=8b  (011) → weight MSB at WEIGHT_BITS-9  = bit 23
//   index=16b (100) → weight MSB at WEIGHT_BITS-17 = bit 15
always @* begin
   actual_weight = {WEIGHT_BITS{1'b0}};
   case (index_sz_i)
      3'b000: case (weight_sz_i)   // index=1b  → weight MSB at WEIGHT_BITS-2
         3'b000: actual_weight = {{(WEIGHT_BITS-1) {weight_value[WEIGHT_BITS-2]}},  weight_value[WEIGHT_BITS-2]};
         3'b001: actual_weight = {{(WEIGHT_BITS-2) {weight_value[WEIGHT_BITS-2]}},  weight_value[WEIGHT_BITS-2  -: 2]};
         3'b010: actual_weight = {{(WEIGHT_BITS-4) {weight_value[WEIGHT_BITS-2]}},  weight_value[WEIGHT_BITS-2  -: 4]};
         3'b011: actual_weight = {{(WEIGHT_BITS-8) {weight_value[WEIGHT_BITS-2]}},  weight_value[WEIGHT_BITS-2  -: 8]};
         3'b100: actual_weight = {{(WEIGHT_BITS-16){weight_value[WEIGHT_BITS-2]}},  weight_value[WEIGHT_BITS-2  -: 16]};
         default: actual_weight = {WEIGHT_BITS{1'b0}};
      endcase
      3'b001: case (weight_sz_i)   // index=2b  → weight MSB at WEIGHT_BITS-3
         3'b000: actual_weight = {{(WEIGHT_BITS-1) {weight_value[WEIGHT_BITS-3]}},  weight_value[WEIGHT_BITS-3]};
         3'b001: actual_weight = {{(WEIGHT_BITS-2) {weight_value[WEIGHT_BITS-3]}},  weight_value[WEIGHT_BITS-3  -: 2]};
         3'b010: actual_weight = {{(WEIGHT_BITS-4) {weight_value[WEIGHT_BITS-3]}},  weight_value[WEIGHT_BITS-3  -: 4]};
         3'b011: actual_weight = {{(WEIGHT_BITS-8) {weight_value[WEIGHT_BITS-3]}},  weight_value[WEIGHT_BITS-3  -: 8]};
         3'b100: actual_weight = {{(WEIGHT_BITS-16){weight_value[WEIGHT_BITS-3]}},  weight_value[WEIGHT_BITS-3  -: 16]};
         default: actual_weight = {WEIGHT_BITS{1'b0}};
      endcase
      3'b010: case (weight_sz_i)   // index=4b  → weight MSB at WEIGHT_BITS-5
         3'b000: actual_weight = {{(WEIGHT_BITS-1) {weight_value[WEIGHT_BITS-5]}},  weight_value[WEIGHT_BITS-5]};
         3'b001: actual_weight = {{(WEIGHT_BITS-2) {weight_value[WEIGHT_BITS-5]}},  weight_value[WEIGHT_BITS-5  -: 2]};
         3'b010: actual_weight = {{(WEIGHT_BITS-4) {weight_value[WEIGHT_BITS-5]}},  weight_value[WEIGHT_BITS-5  -: 4]};
         3'b011: actual_weight = {{(WEIGHT_BITS-8) {weight_value[WEIGHT_BITS-5]}},  weight_value[WEIGHT_BITS-5  -: 8]};
         3'b100: actual_weight = {{(WEIGHT_BITS-16){weight_value[WEIGHT_BITS-5]}},  weight_value[WEIGHT_BITS-5  -: 16]};
         default: actual_weight = {WEIGHT_BITS{1'b0}};
      endcase
      3'b011: case (weight_sz_i)   // index=8b  → weight MSB at WEIGHT_BITS-9
         3'b000: actual_weight = {{(WEIGHT_BITS-1) {weight_value[WEIGHT_BITS-9]}},  weight_value[WEIGHT_BITS-9]};
         3'b001: actual_weight = {{(WEIGHT_BITS-2) {weight_value[WEIGHT_BITS-9]}},  weight_value[WEIGHT_BITS-9  -: 2]};
         3'b010: actual_weight = {{(WEIGHT_BITS-4) {weight_value[WEIGHT_BITS-9]}},  weight_value[WEIGHT_BITS-9  -: 4]};
         3'b011: actual_weight = {{(WEIGHT_BITS-8) {weight_value[WEIGHT_BITS-9]}},  weight_value[WEIGHT_BITS-9  -: 8]};
         3'b100: actual_weight = {{(WEIGHT_BITS-16){weight_value[WEIGHT_BITS-9]}},  weight_value[WEIGHT_BITS-9  -: 16]};
         default: actual_weight = {WEIGHT_BITS{1'b0}};
      endcase
      3'b100: case (weight_sz_i)   // index=16b → weight MSB at WEIGHT_BITS-17
         3'b000: actual_weight = {{(WEIGHT_BITS-1) {weight_value[WEIGHT_BITS-17]}}, weight_value[WEIGHT_BITS-17]};
         3'b001: actual_weight = {{(WEIGHT_BITS-2) {weight_value[WEIGHT_BITS-17]}}, weight_value[WEIGHT_BITS-17 -: 2]};
         3'b010: actual_weight = {{(WEIGHT_BITS-4) {weight_value[WEIGHT_BITS-17]}}, weight_value[WEIGHT_BITS-17 -: 4]};
         3'b011: actual_weight = {{(WEIGHT_BITS-8) {weight_value[WEIGHT_BITS-17]}}, weight_value[WEIGHT_BITS-17 -: 8]};
         3'b100: actual_weight = {{(WEIGHT_BITS-16){weight_value[WEIGHT_BITS-17]}}, weight_value[WEIGHT_BITS-17 -: 16]};
         default: actual_weight = {WEIGHT_BITS{1'b0}};
      endcase
      default: actual_weight = {WEIGHT_BITS{1'b0}};
   endcase
end

// Non-sparse path: slice_and_align places the weight in the top weight_sz
// bits of the WEIGHT_BITS-wide port (left-justified when weight_sz < WEIGHT_BITS).
// Right-justify and sign-extend so the accumulator sees a properly-aligned
// signed value regardless of port-vs-slice width.
reg [WEIGHT_BITS-1:0] weight_rj_nonsparse;
always @* begin
   case (weight_sz_i)
      3'b000: weight_rj_nonsparse = {{(WEIGHT_BITS-1){weight_value[WEIGHT_BITS-1]}},
                                     weight_value[WEIGHT_BITS-1]};
      3'b001: weight_rj_nonsparse = {{(WEIGHT_BITS-2){weight_value[WEIGHT_BITS-1]}},
                                     weight_value[WEIGHT_BITS-1:WEIGHT_BITS-2]};
      3'b010: weight_rj_nonsparse = {{(WEIGHT_BITS-4){weight_value[WEIGHT_BITS-1]}},
                                     weight_value[WEIGHT_BITS-1:WEIGHT_BITS-4]};
      3'b011: weight_rj_nonsparse = {{(WEIGHT_BITS-8){weight_value[WEIGHT_BITS-1]}},
                                     weight_value[WEIGHT_BITS-1:WEIGHT_BITS-8]};
      3'b100: weight_rj_nonsparse = {{(WEIGHT_BITS-16){weight_value[WEIGHT_BITS-1]}},
                                     weight_value[WEIGHT_BITS-1:WEIGHT_BITS-16]};
      3'b101: weight_rj_nonsparse = weight_value;
      default: weight_rj_nonsparse = {WEIGHT_BITS{1'b0}};
   endcase
end

wire is_sparse = (weight_mode_i == 2'b01);
wire [WEIGHT_BITS-1:0] weight_value_to_update = is_sparse ? actual_weight
                                                          : weight_rj_nonsparse;

// Right-justify act_data_out (left-justified by slice_and_align) for MAC.
// act_slice_sz_i selects element width; top bits of act_data_out hold the value.
reg [ACT_BITS-1:0] act_value_rj;
always @* begin
    case (act_slice_sz_i)
        3'b000: act_value_rj = {{(ACT_BITS-1){1'b0}},  act_data_out[ACT_BITS-1]};
        3'b001: act_value_rj = {{(ACT_BITS-2){1'b0}},  act_data_out[ACT_BITS-1:ACT_BITS-2]};
        3'b010: act_value_rj = {{(ACT_BITS-4){1'b0}},  act_data_out[ACT_BITS-1:ACT_BITS-4]};
        3'b011: act_value_rj = {{(ACT_BITS-8){1'b0}},  act_data_out[ACT_BITS-1:ACT_BITS-8]};
        3'b100: act_value_rj = {{(ACT_BITS-16){1'b0}}, act_data_out[ACT_BITS-1:ACT_BITS-16]};
        3'b101: act_value_rj = act_data_out;
        default: act_value_rj = {ACT_BITS{1'b0}};
    endcase
end

`ifdef SPARSE_DEBUG
always @ (posedge clk)
   if (is_sparse & weight_value_valid & weight_value_taken)
      $display("sparse: act_idx=%0d slot=%0d wv=%h idx=%0d wgt=%0d sc_addr=%h",
               act_data_idx, weight_index, weight_value,
               actual_index, actual_weight, syn_curr_mem_addr_o);
`endif

ann_syn_curr_update # (
   .X_OUTPUT_SZ(X_OUTPUT_SZ),
   .Y_OUTPUT_SZ(Y_OUTPUT_SZ),
   .IN_DATA_BITS(IN_DATA_BITS),
   .WEIGHT_IDX_SZ(WEIGHT_IDX_SZ),
   .WEIGHT_SLICE_SZ(WEIGHT_SLICE_SZ),
   .WEIGHT_DATA_IDX_SZ(WEIGHT_DATA_IDX_SZ),
   .SPARSE_IDX_SZ(SPARSE_IDX_SZ),
   .MEM_ADDR_BITS(MEM_ADDR_BITS))
   syn_curr_update0
   (
   .clk(clk),
   .reset(reset),
   .start_new_block_i(start_new_block_i),
   .running_i(running_r),
   .finished_pass_weight_i(finished_pass_weight_gen),
   .finished_pass_o(finished_pass_syn_curr_update),
   .syn_curr_update_running_o(syn_curr_update_running),
   .weight_mode_i(weight_mode_i),
   .sparse_index_i(actual_index),
   .syn_curr_base_addr_i(syn_curr_base_addr_i),
   .out_x_len_i(out_x_len_i),
   .weight_index_valid_i(weight_index_valid),
   .weight_index_i(weight_index),
   .weight_index_x_i(weight_index_x),
   .weight_index_y_i(weight_index_y),
   .weight_index_last_i(weight_index_last),
   .weight_index_taken_o(weight_index_taken),
   .weight_value_valid_i(weight_value_valid),
   .weight_value_i(weight_value_to_update),
   .weight_value_taken_o(weight_value_taken),
   .syn_curr_mem_rd_o(syn_curr_mem_rd_o),
   .syn_curr_mem_wr_o(syn_curr_mem_wr_o),
   .syn_curr_mem_wait_i(syn_curr_mem_wait_i),
   .syn_curr_mem_addr_o(syn_curr_mem_addr_o),
   .syn_curr_mem_data_i(syn_curr_mem_data_rd_i),
   .syn_curr_mem_data_o(syn_curr_mem_data_wr_o),
   .act_value_i(act_value_rj)
);

endmodule

