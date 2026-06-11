// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
/////////////////////////////////////////////////////////////////////
//
// neuron_processor
//
// Given a set of synaptic current buffers, bias currents and
// potentials, update the potential for the current timestep and 
// the perform thresholding.
// output buffers then contain the updated synaptic currents, the
// potentials, and the output spikes.
//
// Steps:
// For all neurons:
// 1) Read synaptic current, bias current and potential
// 2) Add bias and synaptic current to potential
// 3) Decay synaptic current, then write back
// 4) Read neuron threshold
// 5) Perform threshold
// 6) Write output spike value (0 or 1) to output spike buffer
// 7) Decay potential, then write back

`include "../shared/constants.v"

module neuron_processing # (parameter TGT_ACC_ID            = 3'b000,
	                    parameter NUM_TIMESTEPS         = 32,
	 		    parameter TIMESTEP_SZ           = 10,
                            parameter IN_DATA_BITS          = 32,
                            parameter NEURON_IDX_SZ         = 10,
                            parameter SYN_CURR_IDX_SZ       = 10,
                            parameter SYN_CURR_DATA_IDX_SZ  = 5,
                            parameter SYN_CURR_SLICE_SZ     = 3,
                            parameter SYN_CURR_SLICE_BITS   = 10,
                            parameter BIAS_CURR_IDX_SZ      = 2,
                            parameter BIAS_CURR_DATA_IDX_SZ = 5,
	  		    parameter BIAS_CURR_SLICE_SZ    = 3,
	 		    parameter BIAS_CURR_SLICE_BITS  = 8,
                            parameter POT_IDX_SZ            = 2,
                            parameter POT_DATA_IDX_SZ       = 5,
	  		    parameter POT_SLICE_SZ          = 3,
	 		    parameter POT_SLICE_BITS        = 8,
                            parameter SPIKE_IDX_SZ          = 2,
                            parameter SPIKE_DATA_IDX_SZ     = 5,
	  		    parameter SPIKE_SLICE_SZ        = 3,
	 		    parameter SPIKE_SLICE_BITS      = 8,
                            parameter SYN_DECAY_BITS        = 32,
                            parameter POT_DECAY_BITS        = 32,
	 	            parameter MEM_ADDR_BITS         = `ADDR_SIZE)(

    input  wire                    clk,
    input  wire                    reset,

    // Config registers (driven from acc_snn_processor)
    input wire      [NEURON_IDX_SZ-1:0] last_neuron_idx_i,
    input wire      [MEM_ADDR_BITS-1:0] syn_curr_base_addr_i,
    input wire      [MEM_ADDR_BITS-1:0] bias_curr_base_addr_i,
    input wire      [MEM_ADDR_BITS-1:0] thresh_base_addr_i,
    input wire      [MEM_ADDR_BITS-1:0] pot_base_addr_i,
    input wire      [MEM_ADDR_BITS-1:0] spike_base_addr_i,
    input wire  [SYN_CURR_SLICE_SZ-1:0] syn_curr_sz_i,
    input wire [BIAS_CURR_SLICE_SZ-1:0] bias_curr_sz_i,
    input wire       [POT_SLICE_SZ-1:0] pot_sz_i,
    input wire                    [4:0] bin_point_syn_curr_i,
    input wire    [SYN_DECAY_BITS-1:0] syn_curr_decay_mult_i,
    input wire    [POT_DECAY_BITS-1:0] pot_decay_mult_i,
    input wire                          sub_on_fire_i,
    input wire                          clear_pot_i,

    // Interface to Scheduler:
    input  wire                     start_new_block_i,
    input  wire   [`TGT_ACC_SZ-1:0] target_acc_i,
    input  wire [`SCH_ENTRY_SZ-1:0] buffer_info_i,
    output wire                     neuron_proc_finished_o,
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
 
    // Memory interfaces:
    // Synaptic currents
    output wire                     syn_curr_mem_wr_o,
    output wire                     syn_curr_mem_rd_o,
    input  wire                     syn_curr_mem_wait_i,
    output wire    [`ADDR_SIZE-1:0] syn_curr_mem_addr_o,
    output wire     [`POT_BITS-1:0] syn_curr_mem_data_o,
    input  wire     [`POT_BITS-1:0] syn_curr_mem_data_i,
    // Bias currents
    output wire                     bias_curr_mem_rd_o,
    input  wire                     bias_curr_mem_wait_i,
    output wire    [`ADDR_SIZE-1:0] bias_curr_mem_addr_o,
    input  wire     [`WTD_BITS-1:0] bias_curr_mem_data_i,
    // Threshold:
    output wire                     thresh_mem_rd_o,
    input  wire                     thresh_mem_wait_i,
    output wire    [`ADDR_SIZE-1:0] thresh_mem_addr_o,
    input  wire     [`WTD_BITS-1:0] thresh_mem_data_i,
    // Potentials
    output wire                     pot_mem_wr_o,
    output wire                     pot_mem_rd_o,
    input  wire                     pot_mem_wait_i,
    output wire    [`ADDR_SIZE-1:0] pot_mem_addr_o,
    output wire     [`POT_BITS-1:0] pot_mem_data_o,
    input  wire     [`POT_BITS-1:0] pot_mem_data_i,
    // Output activations (spikes)
    output wire                     spike_mem_wr_o,
    input  wire                     spike_mem_wait_i,
    output wire    [`ADDR_SIZE-1:0] spike_mem_addr_o,
    output wire     [`ACT_BITS-1:0] spike_mem_data_o

);

// Config params that will arrive through the config interface:
localparam POT_BIN_POINT_BIT       = 8;
localparam BIAS_CURR_BIN_POINT_BIT = 8;
localparam SYN_CURR_BIN_POINT_BIT  = 8;

reg                              neuron_update_running_r;
reg            [TIMESTEP_SZ-1:0] total_timesteps_r;
reg          [NEURON_IDX_SZ-1:0] neuron_counter_r;
wire                             neuron_update_complete;
wire                             next_neuron;
wire                             neuron_valid;
wire                             result_valid;
wire                             result_taken;
wire                             spike;
wire                             packer_busy;
wire                             packer_finish;
wire                             packer_full;
// Signals for the synaptic current:
wire       [SYN_CURR_IDX_SZ-1:0] syn_curr_index;
wire                             syn_curr_index_valid;
wire                             syn_curr_wait;
wire                             syn_curr_index_last;
wire                             syn_curr_data_valid;
wire  [SYN_CURR_DATA_IDX_SZ-1:0] syn_curr_data_idx;
wire  [SYN_CURR_SLICE_BITS-1:0] syn_curr_data_out;
wire                             syn_curr_data_last;
// Signals for the bias current:
wire      [BIAS_CURR_IDX_SZ-1:0] bias_curr_index;
wire                             bias_curr_index_valid;
wire                             bias_curr_wait;
wire                             bias_curr_index_last;
wire                             bias_curr_data_valid;
wire [BIAS_CURR_DATA_IDX_SZ-1:0] bias_curr_data_idx;
wire  [BIAS_CURR_SLICE_BITS-1:0] bias_curr_data_out;
wire                             bias_curr_data_last;
//wire            [`ADDR_SIZE-1:0] bias_curr_mem_addr_o;
//wire             [`POT_BITS-1:0] bias_curr_mem_data_i;
//wire                             bias_curr_mem_req_o;
//wire                             bias_curr_mem_wait_i;
// Signal for the threshold:
wire [BIAS_CURR_DATA_IDX_SZ-1:0] thresh_data_idx;
wire  [BIAS_CURR_SLICE_BITS-1:0] thresh_data_out;
// Signals for the potential:
wire            [POT_IDX_SZ-1:0] pot_index;
wire                             pot_index_valid;
wire                             pot_wait;
wire                             pot_index_last;
wire                             pot_data_valid;
wire       [POT_DATA_IDX_SZ-1:0] pot_data_idx;
wire        [POT_SLICE_BITS-1:0] pot_data_out;
wire                             pot_data_last;
wire                             pot_mem_req_o;
wire            [`ADDR_SIZE-1:0] spike_base_addr_r;
wire            [`ADDR_SIZE-1:0] syn_curr_cache_mem_addr;
wire                             syn_curr_cache_mem_rd;
wire            [`ADDR_SIZE-1:0] pot_cache_mem_addr;
wire                             pot_cache_mem_rd;
wire                             syn_curr_wb_wr;
wire            [`ADDR_SIZE-1:0] syn_curr_wb_addr;
wire                      [31:0] syn_curr_wb_data_bus;
wire                             syn_curr_wb_full;
wire                             pot_wb_wr;
wire            [`ADDR_SIZE-1:0] pot_wb_addr;
wire                      [31:0] pot_wb_data_bus;
wire                             pot_wb_full;
reg                       [31:0] syn_curr_wb_lj;
reg                       [31:0] pot_wb_lj;
wire                             neuron_taken;
// Raw outputs from update_state_for_neuron at the parameterised widths.
// Sign-extended to 32 bits below so the (32-bit-wide) left-justify case
// blocks below see them as signed quantities.
wire signed [POT_SLICE_BITS-1:0]      updated_potential_raw;
wire signed [SYN_CURR_SLICE_BITS-1:0] updated_syn_curr_raw;
wire signed              [31:0]       updated_potential =
    {{(32-POT_SLICE_BITS){updated_potential_raw[POT_SLICE_BITS-1]}}, updated_potential_raw};
wire signed              [31:0]       updated_syn_curr =
    {{(32-SYN_CURR_SLICE_BITS){updated_syn_curr_raw[SYN_CURR_SLICE_BITS-1]}}, updated_syn_curr_raw};

assign spike_base_addr_r = spike_base_addr_i;

///////////////////////////////////////////////////////////////////////
// 0) Counters to cycle over all neurons in the layer in order
///////////////////////////////////////////////////////////////////////

// End of phase detection logic

assign next_neuron = neuron_taken;

assign neuron_update_complete = (neuron_counter_r == last_neuron_idx_i) ? neuron_taken : 1'b0;

always @ (posedge clk)
begin
if (reset)
   neuron_update_running_r <= 1'b0;
else if (start_new_block_i & target_acc_i == TGT_ACC_ID)
   neuron_update_running_r <= 1'b1;
else if (neuron_update_complete)
   neuron_update_running_r <= 1'b0;
end

// Assign output info, to inform scheduler of task completion:
assign acc_busy_o     = neuron_update_running_r; // Steady state 'busy' signal
assign acc_finished_o = neuron_update_running_r & neuron_update_complete; // 1-cycle finish strobe
assign neuron_proc_finished_o = acc_finished_o;

always @ (posedge clk)
begin
   if (reset | start_new_block_i)
      neuron_counter_r <= 'b0;
   else if (next_neuron)
      neuron_counter_r <= neuron_counter_r + 1'b1;
end

////////////////////////////////////////////////////////////////////////
// 1) Read synaptic current, bias current and potential
////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////
// Manage synaptic current fetching
//

assign syn_curr_index       = neuron_counter_r;
assign syn_curr_index_valid = neuron_update_running_r;
assign syn_curr_index_last  = (neuron_counter_r == last_neuron_idx_i) ? 1'b1 : 1'b0;

// Reading synaptic currents:
dataline_cache_with_xy #(
    .IN_DATA_BITS(IN_DATA_BITS),
    .X_INPUT_SZ(1),
    .Y_INPUT_SZ(1),
    .IDX_ADDR_BITS(SYN_CURR_IDX_SZ),
    .SLICE_DATA_IDX_SZ(SYN_CURR_DATA_IDX_SZ),
    .SLICE_SIZE_SZ(SYN_CURR_SLICE_SZ),
    .OUT_DATA_BITS(SYN_CURR_SLICE_BITS))

    syn_curr_cache (
    .clk(clk),
    .reset(reset),

    .colour_select_o(),
    .invalidate_i(start_new_block_i), // task dispatch — cache quiescent
    .slice_sz_i(syn_curr_sz_i),
    .base_addr_i(syn_curr_base_addr_i),

    .sys_addr_i(syn_curr_index),
    .sys_req_i(syn_curr_index_valid),
    .sys_index_x_i(1'b0),
    .sys_index_y_i(1'b0),
    .sys_colour_i(1'b0),
    .sys_wait_o(syn_curr_wait),
    .sys_last_i(syn_curr_index_last),

    .slice_data_valid_o(syn_curr_data_valid),
    .slice_data_idx_o(syn_curr_data_idx),
    .slice_data_index_x_o(),
    .slice_data_index_y_o(),
    .slice_data_o(syn_curr_data_out),
    .slice_data_last_o(syn_curr_data_last),
    .slice_data_taken_i(neuron_taken),

    .mem_addr_o(syn_curr_cache_mem_addr),
    .mem_data_i(syn_curr_mem_data_i),
    .mem_req_o (syn_curr_cache_mem_rd),
    .mem_wait_i(syn_curr_mem_wait_i | syn_curr_wb_wr)
);

//////////////////////////////////////////////////////////////
// Manage bias current fetching
//
assign bias_curr_index       = neuron_counter_r;
assign bias_curr_index_valid = neuron_update_running_r;
assign bias_curr_index_last  = (neuron_counter_r == last_neuron_idx_i) ? 1'b1 : 1'b0;

// Reading bias currents:
dataline_cache_with_xy #(
    .IN_DATA_BITS(IN_DATA_BITS),
    .X_INPUT_SZ(1),
    .Y_INPUT_SZ(1),
    .IDX_ADDR_BITS(BIAS_CURR_IDX_SZ),
    .SLICE_DATA_IDX_SZ(BIAS_CURR_DATA_IDX_SZ),
    .SLICE_SIZE_SZ(BIAS_CURR_SLICE_SZ),
    .OUT_DATA_BITS(BIAS_CURR_SLICE_BITS))

    bias_curr_cache (
    .clk(clk),
    .reset(reset),

    .colour_select_o(),
    .invalidate_i(start_new_block_i), // task dispatch — cache quiescent
    .slice_sz_i(bias_curr_sz_i),
    .base_addr_i(bias_curr_base_addr_i),

    .sys_addr_i(bias_curr_index),
    .sys_req_i(bias_curr_index_valid),
    .sys_index_x_i(1'b0),
    .sys_index_y_i(1'b0),
    .sys_colour_i(1'b0),
    .sys_wait_o(bias_curr_wait),
    .sys_last_i(bias_curr_index_last),

    .slice_data_valid_o(bias_curr_data_valid),
    .slice_data_idx_o(bias_curr_data_idx),
    .slice_data_index_x_o(),
    .slice_data_index_y_o(),
    .slice_data_o(bias_curr_data_out),
    .slice_data_last_o(bias_curr_data_last),
    .slice_data_taken_i(neuron_taken),

    .mem_addr_o(bias_curr_mem_addr_o),
    .mem_data_i(bias_curr_mem_data_i),
    .mem_req_o(bias_curr_mem_rd_o),
    .mem_wait_i(bias_curr_mem_wait_i)
);

// Reading thresholds:
dataline_cache_with_xy #(
    .IN_DATA_BITS(IN_DATA_BITS),
    .X_INPUT_SZ(1),
    .Y_INPUT_SZ(1),
    .IDX_ADDR_BITS(BIAS_CURR_IDX_SZ),
    .SLICE_DATA_IDX_SZ(BIAS_CURR_DATA_IDX_SZ),
    .SLICE_SIZE_SZ(BIAS_CURR_SLICE_SZ),
    .OUT_DATA_BITS(BIAS_CURR_SLICE_BITS))

    threshold_cache (
    .clk(clk),
    .reset(reset),

    .colour_select_o(),
    .invalidate_i(start_new_block_i), // task dispatch — cache quiescent
    .slice_sz_i(bias_curr_sz_i),
    .base_addr_i(thresh_base_addr_i),

    .sys_addr_i(bias_curr_index),
    .sys_req_i(bias_curr_index_valid),
    .sys_index_x_i(1'b0),
    .sys_index_y_i(1'b0),
    .sys_colour_i(1'b0),
    .sys_wait_o(thresh_wait),
    .sys_last_i(bias_curr_index_last),

    .slice_data_valid_o(thresh_data_valid),
    .slice_data_idx_o(thresh_data_idx),
    .slice_data_index_x_o(),
    .slice_data_index_y_o(),
    .slice_data_o(thresh_data_out),
    .slice_data_last_o(thresh_data_last),
    .slice_data_taken_i(neuron_taken),

    .mem_addr_o(thresh_mem_addr_o),
    .mem_data_i(thresh_mem_data_i),
    .mem_req_o(thresh_mem_rd_o),
    .mem_wait_i(thresh_mem_wait_i)
);

//////////////////////////////////////////////////////////////
// Manage neuron potential fetching
//
assign pot_index = neuron_counter_r;
assign pot_index_valid = neuron_update_running_r;
assign pot_index_last  = (neuron_counter_r == last_neuron_idx_i) ? 1'b1 : 1'b0;

// Reading neuron potentials:
dataline_cache_with_xy #(
    .IN_DATA_BITS(IN_DATA_BITS),
    .X_INPUT_SZ(1),
    .Y_INPUT_SZ(1),
    .IDX_ADDR_BITS(POT_IDX_SZ),
    .SLICE_DATA_IDX_SZ(POT_DATA_IDX_SZ),
    .SLICE_SIZE_SZ(POT_SLICE_SZ),
    .OUT_DATA_BITS(POT_SLICE_BITS))

    pot_cache (
    .clk(clk),
    .reset(reset),

    .colour_select_o(),
    .invalidate_i(start_new_block_i), // task dispatch — cache quiescent
    .slice_sz_i(pot_sz_i),
    .base_addr_i(pot_base_addr_i),

    .sys_addr_i(pot_index),
    .sys_req_i(pot_index_valid),
    .sys_index_x_i(1'b0),
    .sys_index_y_i(1'b0),
    .sys_colour_i(1'b0),
    .sys_wait_o(pot_wait),
    .sys_last_i(pot_index_last),

    .slice_data_valid_o(pot_data_valid),
    .slice_data_idx_o(pot_data_idx),
    .slice_data_index_x_o(),
    .slice_data_index_y_o(),
    .slice_data_o(pot_data_out),
    .slice_data_last_o(pot_data_last),
    .slice_data_taken_i(neuron_taken),

    .mem_addr_o(pot_cache_mem_addr),
    .mem_data_i(pot_mem_data_i),
    .mem_req_o (pot_cache_mem_rd),
    .mem_wait_i(pot_mem_wait_i | pot_wb_wr)
);

// The neuron is valid when all of its data fields are available:
assign neuron_valid = syn_curr_data_valid  & 
                      pot_data_valid       &
		      bias_curr_data_valid &
		      thresh_data_valid;

wire signed [POT_SLICE_BITS-1:0] pot_input;
assign pot_input = clear_pot_i ? {POT_SLICE_BITS{1'b0}} : pot_data_out;

// Instantiate state updte module:
update_state_for_neuron #(
    .SYN_CURR_SLICE_BITS(SYN_CURR_SLICE_BITS),
    .POT_SLICE_BITS(POT_SLICE_BITS),
    .BIAS_CURR_SLICE_BITS(BIAS_CURR_SLICE_BITS),
    .THRESH_SLICE_BITS(BIAS_CURR_SLICE_BITS),
    .SYN_DECAY_BITS(SYN_DECAY_BITS),
    .POT_DECAY_BITS(POT_DECAY_BITS)
      )
      neuron_update0 (
     .clk(clk),
     .reset(reset),
     .neuron_valid_i(neuron_valid),
     .syn_curr_i(syn_curr_data_out),
     .potential_i(pot_input),
     .bias_curr_i(bias_curr_data_out),
     .threshold_i(thresh_data_out),
     .sub_on_fire_i(sub_on_fire_i),
     .syn_curr_decay_mult_i(syn_curr_decay_mult_i),
     .potential_decay_mult_i(pot_decay_mult_i),
     .neuron_taken_o(neuron_taken),
     .result_valid_o(result_valid),
     .potential_o(updated_potential_raw),
     .syn_curr_o(updated_syn_curr_raw),
     .spike_o(spike),
     .result_taken_i(result_taken)
);

// Left-justify updated values for packer input (element in MSBs of 32-bit word):
always @(*) begin
    case (syn_curr_sz_i)
        3'b000: syn_curr_wb_lj = {updated_syn_curr[0],    31'b0};
        3'b001: syn_curr_wb_lj = {updated_syn_curr[1:0],  30'b0};
        3'b010: syn_curr_wb_lj = {updated_syn_curr[3:0],  28'b0};
        3'b011: syn_curr_wb_lj = {updated_syn_curr[7:0],  24'b0};
        3'b100: syn_curr_wb_lj = {updated_syn_curr[15:0], 16'b0};
        3'b101: syn_curr_wb_lj = updated_syn_curr[31:0];
        default: syn_curr_wb_lj = 32'b0;
    endcase
end

always @(*) begin
    case (pot_sz_i)
        3'b000: pot_wb_lj = {updated_potential[0],    31'b0};
        3'b001: pot_wb_lj = {updated_potential[1:0],  30'b0};
        3'b010: pot_wb_lj = {updated_potential[3:0],  28'b0};
        3'b011: pot_wb_lj = {updated_potential[7:0],  24'b0};
        3'b100: pot_wb_lj = {updated_potential[15:0], 16'b0};
        3'b101: pot_wb_lj = updated_potential[31:0];
        default: pot_wb_lj = 32'b0;
    endcase
end

assign syn_curr_mem_wr_o   = syn_curr_wb_wr;
assign syn_curr_mem_rd_o   = syn_curr_cache_mem_rd & ~syn_curr_wb_wr;
assign syn_curr_mem_addr_o = syn_curr_wb_wr ? syn_curr_wb_addr : syn_curr_cache_mem_addr;
assign syn_curr_mem_data_o = syn_curr_wb_data_bus;

assign pot_mem_wr_o   = pot_wb_wr;
assign pot_mem_rd_o   = pot_cache_mem_rd & ~pot_wb_wr;
assign pot_mem_addr_o = pot_wb_wr ? pot_wb_addr : pot_cache_mem_addr;
assign pot_mem_data_o = pot_wb_data_bus;

// Stall update_state_for_neuron while any packer cannot accept data:
assign result_taken = result_valid & ~packer_full & ~syn_curr_wb_full & ~pot_wb_full;

// Pack spike bits into 32-bit words and write to spike memory:
packer spike_packer0 (
    .clk                 (clk),
    .reset               (reset),
    .busy_o              (packer_busy),
    .finish_o            (packer_finish),
    .pak_write_i         (result_valid),
    .pak_full_o          (packer_full),
    .pak_colour_i        (1'b0),
    .pak_last_i          (neuron_counter_r == last_neuron_idx_i),
    .pak_index_i         ({{(`PIN_BITS-NEURON_IDX_SZ){1'b0}}, neuron_counter_r}),
    .pak_acc_data_i      ({spike, {(`POT_BITS-1){1'b0}}}),
    .pot_wr_o            (spike_mem_wr_o),
    .pot_wait_i          (spike_mem_wait_i),
    .pot_addr_o          (spike_mem_addr_o),
    .pot_data_o          (spike_mem_data_o),
    .pak_colour_sel_o    (),
    .pak_out_sz_i        ({`POT_OUT_SZ_SZ{1'b0}}),
    .pak_colour_bs_o     (),
    .pak_out_base_addr_i (spike_base_addr_r)
);

// Accumulate decayed syn_curr elements into 32-bit words and write back:
packer syn_curr_wb_packer (
    .clk                 (clk),
    .reset               (reset),
    .busy_o              (),
    .finish_o            (),
    .pak_write_i         (result_valid),
    .pak_full_o          (syn_curr_wb_full),
    .pak_colour_i        (1'b0),
    .pak_last_i          (neuron_counter_r == last_neuron_idx_i),
    .pak_index_i         ({{(`PIN_BITS-NEURON_IDX_SZ){1'b0}}, neuron_counter_r}),
    .pak_acc_data_i      (syn_curr_wb_lj),
    .pot_wr_o            (syn_curr_wb_wr),
    .pot_wait_i          (syn_curr_mem_wait_i),
    .pot_addr_o          (syn_curr_wb_addr),
    .pot_data_o          (syn_curr_wb_data_bus),
    .pak_colour_sel_o    (),
    .pak_out_sz_i        (syn_curr_sz_i),
    .pak_colour_bs_o     (),
    .pak_out_base_addr_i (syn_curr_base_addr_i)
);

// Accumulate decayed potential elements into 32-bit words and write back:
packer pot_wb_packer (
    .clk                 (clk),
    .reset               (reset),
    .busy_o              (),
    .finish_o            (),
    .pak_write_i         (result_valid),
    .pak_full_o          (pot_wb_full),
    .pak_colour_i        (1'b0),
    .pak_last_i          (neuron_counter_r == last_neuron_idx_i),
    .pak_index_i         ({{(`PIN_BITS-NEURON_IDX_SZ){1'b0}}, neuron_counter_r}),
    .pak_acc_data_i      (pot_wb_lj),
    .pot_wr_o            (pot_wb_wr),
    .pot_wait_i          (pot_mem_wait_i),
    .pot_addr_o          (pot_wb_addr),
    .pot_data_o          (pot_wb_data_bus),
    .pak_colour_sel_o    (),
    .pak_out_sz_i        (pot_sz_i),
    .pak_colour_bs_o     (),
    .pak_out_base_addr_i (pot_base_addr_i)
);

endmodule

