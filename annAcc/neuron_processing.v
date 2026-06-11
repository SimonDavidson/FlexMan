// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
// annAcc neuron_processing:
// Reads accumulated potential from syn_curr_mem (written by spike_processing),
// applies RELU or LUT threshold, decays the post-threshold activation, and
// writes decayed activation to pot_mem and packed output activations to spike_mem.
// No bias_curr read, no syn_curr write-back.

`include "../shared/constants.v"

module ann_neuron_processing # (
    parameter TGT_ACC_ID            = 3'b000,
    parameter NUM_TIMESTEPS         = 32,
    parameter TIMESTEP_SZ           = 10,
    parameter IN_DATA_BITS          = 32,
    parameter NEURON_IDX_SZ         = 10,
    parameter SYN_CURR_IDX_SZ       = 10,
    parameter SYN_CURR_DATA_IDX_SZ  = 5,
    parameter SYN_CURR_SLICE_SZ     = 3,
    parameter SYN_CURR_SLICE_BITS   = 32,
    parameter LUT_IDX_SZ            = 8,
    parameter LUT_DATA_IDX_SZ       = 5,
    parameter LUT_SLICE_SZ          = 3,
    parameter LUT_SLICE_BITS        = 8,
    parameter POT_SLICE_SZ          = 3,
    parameter POT_SLICE_BITS        = 32,
    parameter POT_DECAY_BITS        = 32,
    parameter MEM_ADDR_BITS         = `ADDR_SIZE
)(
    input  wire                    clk,
    input  wire                    reset,

    // Config registers
    input wire      [NEURON_IDX_SZ-1:0] last_neuron_idx_i,
    input wire      [MEM_ADDR_BITS-1:0] syn_curr_base_addr_i,
    input wire      [MEM_ADDR_BITS-1:0] thresh_base_addr_i,   // lut_base_addr
    input wire      [MEM_ADDR_BITS-1:0] pot_base_addr_i,
    input wire      [MEM_ADDR_BITS-1:0] spike_base_addr_i,
    input wire  [SYN_CURR_SLICE_SZ-1:0] syn_curr_sz_i,
    input wire       [POT_SLICE_SZ-1:0] pot_sz_i,
    input wire                    [2:0] lut_out_sz_i,          // LUT element width
    input wire                    [2:0] act_out_sz_i,          // output activation width
    input wire                    [1:0] thresh_op_i,           // 00=RELU 01=LUT 10=ABS
    input wire                    [4:0] bin_point_syn_curr_i,  // requant: input (accumulator) bin point
    input wire                    [4:0] np_out_bin_point_i,    // requant: output activation bin point (0=disabled)
    input wire    [POT_DECAY_BITS-1:0] pot_decay_mult_i,

    // Scheduler interface
    input  wire                     start_new_block_i,
    input  wire   [`TGT_ACC_SZ-1:0] target_acc_i,
    input  wire [`SCH_ENTRY_SZ-1:0] buffer_info_i,
    output wire                     neuron_proc_finished_o,
    output wire                     acc_busy_o,
    output wire                     acc_finished_o,

    // Buffer addr interface
    input wire      [`PIN_BITS-1:0] src1_buff_addr_i,
    input wire      [`PIN_BITS-1:0] src2_buff_addr_i,
    input wire      [`PIN_BITS-1:0] src3_buff_addr_i,
    input wire      [`PIN_BITS-1:0]  tgt_buff_addr_i,
    input wire      [`PIN_BITS-1:0] weight_row_len_i,

    // Synaptic currents (read-only from NP)
    output wire                     syn_curr_mem_wr_o,
    output wire                     syn_curr_mem_rd_o,
    input  wire                     syn_curr_mem_wait_i,
    output wire    [`ADDR_SIZE-1:0] syn_curr_mem_addr_o,
    output wire     [`POT_BITS-1:0] syn_curr_mem_data_o,
    input  wire     [`POT_BITS-1:0] syn_curr_mem_data_i,
    // LUT memory (thresh_mem ports repurposed)
    output wire                     thresh_mem_rd_o,
    input  wire                     thresh_mem_wait_i,
    output wire    [`ADDR_SIZE-1:0] thresh_mem_addr_o,
    input  wire     [`WTD_BITS-1:0] thresh_mem_data_i,
    // Potentials (write-only from NP)
    output wire                     pot_mem_wr_o,
    output wire                     pot_mem_rd_o,
    input  wire                     pot_mem_wait_i,
    output wire    [`ADDR_SIZE-1:0] pot_mem_addr_o,
    output wire     [`POT_BITS-1:0] pot_mem_data_o,
    input  wire     [`POT_BITS-1:0] pot_mem_data_i,
    // Output activations
    output wire                     spike_mem_wr_o,
    input  wire                     spike_mem_wait_i,
    output wire    [`ADDR_SIZE-1:0] spike_mem_addr_o,
    output wire     [`ACT_BITS-1:0] spike_mem_data_o
);

reg                              neuron_update_running_r;
reg          [NEURON_IDX_SZ-1:0] neuron_counter_r;
wire                             neuron_update_complete;
wire                             next_neuron;
wire                             neuron_valid;
wire                             result_valid;
wire                             result_taken;
wire                             packer_busy;
wire                             packer_finish;
wire                             packer_full;
// syn_curr cache
wire      [SYN_CURR_IDX_SZ-1:0] syn_curr_index;
wire                             syn_curr_index_valid;
wire                             syn_curr_wait;
wire                             syn_curr_index_last;
wire                             syn_curr_data_valid;
wire [SYN_CURR_DATA_IDX_SZ-1:0] syn_curr_data_idx;
wire [SYN_CURR_SLICE_BITS-1:0]  syn_curr_data_out;
wire                             syn_curr_data_last;
wire            [`ADDR_SIZE-1:0] syn_curr_cache_mem_addr;
wire                             syn_curr_cache_mem_rd;
// LUT cache
wire           [LUT_IDX_SZ-1:0] lut_index;
wire                             lut_data_valid;
wire      [LUT_DATA_IDX_SZ-1:0] lut_data_idx;
wire      [LUT_SLICE_BITS-1:0]  lut_data_out;
wire                             lut_data_last;
wire            [`ADDR_SIZE-1:0] lut_cache_mem_addr;
wire                             lut_cache_mem_rd;
// pot write-back packer
wire                             pot_wb_wr;
wire            [`ADDR_SIZE-1:0] pot_wb_addr;
wire                      [31:0] pot_wb_data_bus;
wire                             pot_wb_full;
reg                       [31:0] act_out_lj;
reg                       [31:0] pot_wb_lj;
wire                             neuron_taken;
wire [SYN_CURR_SLICE_BITS-1:0]  act_out_w;
wire [SYN_CURR_SLICE_BITS-1:0]  decayed_act_w;
wire            [`ADDR_SIZE-1:0] spike_base_addr_r;

assign spike_base_addr_r = spike_base_addr_i;

///////////////////////////////////////////////////////////////////////
// 0) Counters
///////////////////////////////////////////////////////////////////////

assign next_neuron           = neuron_taken;
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

assign acc_busy_o             = neuron_update_running_r;
assign acc_finished_o         = neuron_update_running_r & neuron_update_complete;
assign neuron_proc_finished_o = acc_finished_o;

always @ (posedge clk)
begin
    if (reset | start_new_block_i)
        neuron_counter_r <= 'b0;
    else if (next_neuron)
        neuron_counter_r <= neuron_counter_r + 1'b1;
end

///////////////////////////////////////////////////////////////////////
// 1) Read accumulated potential from syn_curr_mem
///////////////////////////////////////////////////////////////////////

assign syn_curr_index       = neuron_counter_r;
assign syn_curr_index_valid = neuron_update_running_r;
assign syn_curr_index_last  = (neuron_counter_r == last_neuron_idx_i) ? 1'b1 : 1'b0;

dataline_cache_with_xy #(
    .IN_DATA_BITS      (IN_DATA_BITS),
    .X_INPUT_SZ        (1),
    .Y_INPUT_SZ        (1),
    .IDX_ADDR_BITS     (SYN_CURR_IDX_SZ),
    .SLICE_DATA_IDX_SZ (SYN_CURR_DATA_IDX_SZ),
    .SLICE_SIZE_SZ     (SYN_CURR_SLICE_SZ),
    .OUT_DATA_BITS     (SYN_CURR_SLICE_BITS))
syn_curr_cache (
    .clk               (clk),
    .reset             (reset),
    .colour_select_o   (),
    .invalidate_i(start_new_block_i), // task dispatch — cache quiescent
    .slice_sz_i        (syn_curr_sz_i),
    .base_addr_i       (syn_curr_base_addr_i),
    .sys_addr_i        (syn_curr_index),
    .sys_req_i         (syn_curr_index_valid),
    .sys_index_x_i     (1'b0),
    .sys_index_y_i     (1'b0),
    .sys_colour_i      (1'b0),
    .sys_wait_o        (syn_curr_wait),
    .sys_last_i        (syn_curr_index_last),
    .slice_data_valid_o(syn_curr_data_valid),
    .slice_data_idx_o  (syn_curr_data_idx),
    .slice_data_index_x_o(),
    .slice_data_index_y_o(),
    .slice_data_o      (syn_curr_data_out),
    .slice_data_last_o (syn_curr_data_last),
    .slice_data_taken_i(neuron_taken),
    .mem_addr_o        (syn_curr_cache_mem_addr),
    .mem_data_i        (syn_curr_mem_data_i),
    .mem_req_o         (syn_curr_cache_mem_rd),
    .mem_wait_i        (syn_curr_mem_wait_i)
);

///////////////////////////////////////////////////////////////////////
// 2) LUT cache — triggered once potential is available (LUT mode only)
//    Uses thresh_mem_* ports; indexed by lower LUT_IDX_SZ bits of potential
///////////////////////////////////////////////////////////////////////

assign lut_index = syn_curr_data_out[LUT_IDX_SZ-1:0];

dataline_cache_with_xy #(
    .IN_DATA_BITS      (IN_DATA_BITS),
    .X_INPUT_SZ        (1),
    .Y_INPUT_SZ        (1),
    .IDX_ADDR_BITS     (LUT_IDX_SZ),
    .SLICE_DATA_IDX_SZ (LUT_DATA_IDX_SZ),
    .SLICE_SIZE_SZ     (LUT_SLICE_SZ),
    .OUT_DATA_BITS     (LUT_SLICE_BITS))
lut_cache (
    .clk               (clk),
    .reset             (reset),
    .colour_select_o   (),
    .invalidate_i(start_new_block_i), // task dispatch — cache quiescent
    .slice_sz_i        (lut_out_sz_i),
    .base_addr_i       (thresh_base_addr_i),
    .sys_addr_i        (lut_index),
    .sys_req_i         (syn_curr_data_valid & thresh_op_i[0]),
    .sys_index_x_i     (1'b0),
    .sys_index_y_i     (1'b0),
    .sys_colour_i      (1'b0),
    .sys_wait_o        (),
    .sys_last_i        (syn_curr_index_last),
    .slice_data_valid_o(lut_data_valid),
    .slice_data_idx_o  (lut_data_idx),
    .slice_data_index_x_o(),
    .slice_data_index_y_o(),
    .slice_data_o      (lut_data_out),
    .slice_data_last_o (lut_data_last),
    .slice_data_taken_i(neuron_taken),
    .mem_addr_o        (lut_cache_mem_addr),
    .mem_data_i        (thresh_mem_data_i),
    .mem_req_o         (lut_cache_mem_rd),
    .mem_wait_i        (thresh_mem_wait_i)
);

///////////////////////////////////////////////////////////////////////
// 3) Neuron valid: RELU needs potential only; LUT needs both
///////////////////////////////////////////////////////////////////////

assign neuron_valid = syn_curr_data_valid & (~thresh_op_i[0] | lut_data_valid);

///////////////////////////////////////////////////////////////////////
// 4) Apply threshold and decay
///////////////////////////////////////////////////////////////////////

ann_update_state_for_neuron #(
    .POT_SLICE_BITS   (SYN_CURR_SLICE_BITS),
    .THRESH_SLICE_BITS(LUT_SLICE_BITS),
    .POT_DECAY_BITS   (POT_DECAY_BITS))
neuron_update0 (
    .clk                    (clk),
    .reset                  (reset),
    .thresh_op_i            (thresh_op_i),
    .neuron_valid_i         (neuron_valid),
    .potential_i            (syn_curr_data_out),
    .lut_result_i           (lut_data_out),
    .potential_decay_mult_i (pot_decay_mult_i),
    .neuron_taken_o         (neuron_taken),
    .result_valid_o         (result_valid),
    .act_out_o              (act_out_w),
    .decayed_act_o          (decayed_act_w),
    .result_taken_i         (result_taken)
);

///////////////////////////////////////////////////////////////////////
// 4b) Output requantisation: align the accumulator bin point to the output
//     activation bin point, round-half-up, then unsigned-saturate to the
//     act_out_sz field width.  act_out_w is the non-negative threshold result
//     (ABS / RELU / LUT), so it is treated as unsigned magnitude.
//       np_out_bin_point_i == 0  -> requant DISABLED: act_out_rq = act_out_w
//          (legacy behaviour — the left-justify below takes the low N bits).
//       otherwise  shift = bin_point_syn_curr_i - np_out_bin_point_i.
//     Reuses the previously-unused bin_point_syn_curr_i hook.
///////////////////////////////////////////////////////////////////////
wire [4:0] rq_shift = bin_point_syn_curr_i - np_out_bin_point_i;
wire [SYN_CURR_SLICE_BITS:0] rq_round =                              // +1 guard bit
        {1'b0, act_out_w}
      + ((rq_shift == 5'd0) ? {(SYN_CURR_SLICE_BITS+1){1'b0}}
                            : ({{SYN_CURR_SLICE_BITS{1'b0}}, 1'b1} << (rq_shift - 5'd1)));
wire [SYN_CURR_SLICE_BITS:0] rq_shifted = rq_round >> rq_shift;

reg [SYN_CURR_SLICE_BITS-1:0] rq_max;     // unsigned max for the output field width
always @(*) begin
    case (act_out_sz_i)
        3'b000: rq_max = 32'h0000_0001;   //  1 bit
        3'b001: rq_max = 32'h0000_0003;   //  2 bits
        3'b010: rq_max = 32'h0000_000F;   //  4 bits
        3'b011: rq_max = 32'h0000_00FF;   //  8 bits
        3'b100: rq_max = 32'h0000_FFFF;   // 16 bits
        3'b101: rq_max = 32'hFFFF_FFFF;   // 32 bits (no clamp)
        default: rq_max = 32'hFFFF_FFFF;
    endcase
end

reg [SYN_CURR_SLICE_BITS-1:0] act_out_rq;
always @(*) begin
    if (np_out_bin_point_i == 5'd0)
        act_out_rq = act_out_w;                                  // requant disabled
    else if (rq_shifted > {1'b0, rq_max})
        act_out_rq = rq_max;                                     // unsigned saturate
    else
        act_out_rq = rq_shifted[SYN_CURR_SLICE_BITS-1:0];
end

///////////////////////////////////////////////////////////////////////
// 5) Left-justify for packers
///////////////////////////////////////////////////////////////////////

always @(*) begin
    case (act_out_sz_i)
        3'b000: act_out_lj = {act_out_rq[0],    31'b0};
        3'b001: act_out_lj = {act_out_rq[1:0],  30'b0};
        3'b010: act_out_lj = {act_out_rq[3:0],  28'b0};
        3'b011: act_out_lj = {act_out_rq[7:0],  24'b0};
        3'b100: act_out_lj = {act_out_rq[15:0], 16'b0};
        3'b101: act_out_lj = act_out_rq[31:0];
        default: act_out_lj = 32'b0;
    endcase
end

always @(*) begin
    case (pot_sz_i)
        3'b000: pot_wb_lj = {decayed_act_w[0],    31'b0};
        3'b001: pot_wb_lj = {decayed_act_w[1:0],  30'b0};
        3'b010: pot_wb_lj = {decayed_act_w[3:0],  28'b0};
        3'b011: pot_wb_lj = {decayed_act_w[7:0],  24'b0};
        3'b100: pot_wb_lj = {decayed_act_w[15:0], 16'b0};
        3'b101: pot_wb_lj = decayed_act_w[31:0];
        default: pot_wb_lj = 32'b0;
    endcase
end

///////////////////////////////////////////////////////////////////////
// 6) Memory connections
///////////////////////////////////////////////////////////////////////

assign syn_curr_mem_wr_o   = 1'b0;
assign syn_curr_mem_rd_o   = syn_curr_cache_mem_rd;
assign syn_curr_mem_addr_o = syn_curr_cache_mem_addr;
assign syn_curr_mem_data_o = {`POT_BITS{1'b0}};

assign thresh_mem_rd_o   = lut_cache_mem_rd;
assign thresh_mem_addr_o = lut_cache_mem_addr;

assign pot_mem_wr_o   = pot_wb_wr;
assign pot_mem_rd_o   = 1'b0;
assign pot_mem_addr_o = pot_wb_addr;
assign pot_mem_data_o = pot_wb_data_bus;

///////////////////////////////////////////////////////////////////////
// 7) Stall update_state while any packer cannot accept
///////////////////////////////////////////////////////////////////////

assign result_taken = result_valid & ~packer_full & ~pot_wb_full;

///////////////////////////////////////////////////////////////////////
// 8) Pack output activations → spike_mem
///////////////////////////////////////////////////////////////////////

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
    .pak_acc_data_i      (act_out_lj),
    .pot_wr_o            (spike_mem_wr_o),
    .pot_wait_i          (spike_mem_wait_i),
    .pot_addr_o          (spike_mem_addr_o),
    .pot_data_o          (spike_mem_data_o),
    .pak_colour_sel_o    (),
    .pak_out_sz_i        (act_out_sz_i),
    .pak_colour_bs_o     (),
    .pak_out_base_addr_i (spike_base_addr_r)
);

///////////////////////////////////////////////////////////////////////
// 9) Pack decayed activations → pot_mem
///////////////////////////////////////////////////////////////////////

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
