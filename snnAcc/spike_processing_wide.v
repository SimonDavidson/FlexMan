// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
// =============================================================================
// Module: spike_processing_wide
// Authors: Simon Davidson & Claude
// Created: 2026-06-07
// Last modified: 2026-11-21
//
// Lane-parallel ("wide") spike_processing for FULL connectivity (weight_mode 00).
// The input sweep is unchanged (scalar) — act_index_generator + act_cache pick
// one spiking input at a time.  For each spiking input the output neurons are
// swept in groups of NUM_LANES; for full connectivity both the weight row and
// the syn_curr accumulators are contiguous in output index, so one fetch feeds
// L lanes (single-fetch / parallel-deserialise, as in neuron_processing_wide).
//
//   for each spiking input i:
//     weight_row_base = i * rows_per_neuron + weight_base
//     for g = 0, L, 2L, ... < num_out:
//        weights[g..g+L-1]   <- np_fetch_sel(weight mem)         (cacheable)
//        syn_curr word RMW: syn_curr[o] += weight[o]             (direct, uncached)
//
// Element widths fixed at elaboration: SYN_CURR_SLICE_BITS (syn_curr) and
// WEIGHT_ELEM_BITS (weight).  Supports NUM_LANES <= 32/SYN_CURR_SLICE_BITS
// (single syn_curr word per group).  Port list matches spike_processing.v.
//
// NOTE: this version implements FULL mode only.  Convolutional mode (straddle +
// OOB) is a separate extension; sparse mode stays scalar.
// =============================================================================

`include "../shared/constants.v"

module spike_processing_wide # (
    parameter NUM_LANES          = 2,
    parameter PREFETCH           = 0,
    parameter WEIGHT_ELEM_BITS   = 8,
    parameter NUM_TIMESTEPS      = 32,
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
    parameter ACT_SLICE_SZ       = 2,
    parameter ACT_DATA_IDX_SZ    = 5,
    parameter WEIGHT_ENTRY_BITS  = 8,
    parameter WEIGHT_IDX_SZ      = 5,
    parameter WEIGHT_SLICE_SZ    = 5,
    parameter WEIGHT_DATA_IDX_SZ = 5,
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

    // Config registers
    input wire   [MEM_ADDR_BITS-1:0] act_base_addr_i,
    input wire   [MEM_ADDR_BITS-1:0] weight_base_addr_i,
    input wire   [MEM_ADDR_BITS-1:0] syn_curr_base_addr_i,
    input wire [WEIGHT_SLICE_SZ-1:0] weight_sz_i,
    input wire                 [4:0] bin_point_syn_curr_i,
    input wire                 [1:0] weight_mode_i,
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

    // Scheduler
    input  wire                     start_new_block_i,
    input  wire   [`TGT_ACC_SZ-1:0] target_acc_i,
    input  wire [`SCH_ENTRY_SZ-1:0] buffer_info_i,
    output wire                     spike_proc_finished_o,
    output wire                     acc_busy_o,
    output wire                     acc_finished_o,

    // Buffer-address interface (kept for port compatibility)
    input wire      [`PIN_BITS-1:0] src1_buff_addr_i,
    input wire      [`PIN_BITS-1:0] src2_buff_addr_i,
    input wire      [`PIN_BITS-1:0] src3_buff_addr_i,
    input wire      [`PIN_BITS-1:0]  tgt_buff_addr_i,
    input wire      [`PIN_BITS-1:0] weight_row_len_i,

    // Memory interfaces
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

    localparam ACT_BITS = 2**ACT_SLICE_SZ;
    localparam SC_BITS  = SYN_CURR_SLICE_BITS;
    localparam WB       = WEIGHT_ELEM_BITS;
    localparam IDX      = SYN_CURR_IDX_SZ;

    // ---------------- global run control (mirrors spike_processing.v) ----------
    reg                   running_r;
    reg [TIMESTEP_SZ-1:0] iteration_count_r;
    wire act_index_gen_running;
    wire finished_timestep_act_index_gen;

    always @(posedge clk)
        if (reset) begin running_r <= 1'b0; iteration_count_r <= 'b0; end
        else if (start_new_block_i) begin running_r <= 1'b1; iteration_count_r <= iteration_count_r + 1; end
        else if (spike_proc_finished_o) begin running_r <= 1'b0; iteration_count_r <= 'b0; end

    wire wide_busy;
    // Full-mode completion, OR force-finish immediately in an unsupported
    // (non-full) mode so the task can never hang on a wide build.
    assign spike_proc_finished_o = (running_r & ~act_index_gen_running & ~wide_busy)
                                 | (running_r & (weight_mode_i != 2'b00));
    assign acc_busy_o     = running_r;
    assign acc_finished_o = spike_proc_finished_o;

    // ---------------- input sweep (scalar, reused) -----------------------------
    wire                  act_index_valid, act_index_last, act_index_wait;
    wire [ACT_IDX_SZ-1:0] act_index;
    wire [X_INPUT_SZ-1:0] act_index_x;
    wire [Y_INPUT_SZ-1:0] act_index_y;
    wire                  act_data_valid, act_data_last;
    wire   [ACT_BITS-1:0] act_data_out;
    wire [ACT_DATA_IDX_SZ-1:0] act_data_idx;
    wire [X_INPUT_SZ-1:0] act_data_idx_x;
    wire [Y_INPUT_SZ-1:0] act_data_idx_y;
    wire                  act_data_taken, act_index_taken;
    wire                  next_in_neuron;

    act_index_generator #(
        .X_INPUT_SZ(X_INPUT_SZ), .Y_INPUT_SZ(Y_INPUT_SZ),
        .X_KERNEL_SZ(X_KERNEL_SZ), .Y_KERNEL_SZ(Y_KERNEL_SZ),
        .X_KERNEL_OFF_SZ(X_KERNEL_OFF_SZ), .Y_KERNEL_OFF_SZ(Y_KERNEL_OFF_SZ),
        .X_STEP_SZ(X_STEP_SZ), .Y_STEP_SZ(Y_STEP_SZ),
        .ELEMS_PER_ROW(ELEMS_PER_ROW), .ROWS_PER_NEURON(ROWS_PER_NEURON),
        .IN_DATA_BITS(IN_DATA_BITS), .ELEM_SZ(ELEM_SZ),
        .ACT_IDX_SZ(ACT_IDX_SZ), .MEM_ADDR_BITS(MEM_ADDR_BITS))
    act_index_gen0 (
        .clk(clk), .reset(reset),
        .start_new_block_i(start_new_block_i), .running_i(running_r),
        .next_in_neuron_i(next_in_neuron),
        .finished_timestep_o(finished_timestep_act_index_gen),
        .act_index_gen_running_o(act_index_gen_running),
        .weight_mode_i(weight_mode_i),
        .in_x_len_i(in_x_len_i), .in_y_len_i(in_y_len_i),
        .x_kernel_len_i(x_kernel_len_i), .y_kernel_len_i(y_kernel_len_i),
        .x_kernel_step_i(x_kernel_step_i), .y_kernel_step_i(y_kernel_step_i),
        .weights_per_word_i(weights_per_word_i), .rows_per_neuron_i(rows_per_neuron_i),
        .act_index_valid_o(act_index_valid),
        .act_index_x_o(act_index_x), .act_index_y_o(act_index_y),
        .act_index_o(act_index), .act_index_last_o(act_index_last),
        .act_index_taken_i(act_index_taken));

    dataline_cache_with_xy #(
        .IN_DATA_BITS(IN_DATA_BITS), .X_INPUT_SZ(X_INPUT_SZ), .Y_INPUT_SZ(Y_INPUT_SZ),
        .IDX_ADDR_BITS(ACT_IDX_SZ), .SLICE_DATA_IDX_SZ(ACT_DATA_IDX_SZ),
        .SLICE_SIZE_SZ(ACT_SLICE_SZ), .OUT_DATA_BITS(ACT_BITS))
    act_cache (
        .clk(clk), .reset(reset),
        .colour_select_o(), .invalidate_i(start_new_block_i), .slice_sz_i({ACT_SLICE_SZ{1'b0}}), .base_addr_i(act_base_addr_i),
        .sys_addr_i(act_index), .sys_req_i(act_index_valid),
        .sys_index_x_i(act_index_x), .sys_index_y_i(act_index_y), .sys_colour_i(1'b0),
        .sys_wait_o(act_index_wait), .sys_last_i(act_index_last),
        .slice_data_valid_o(act_data_valid), .slice_data_idx_o(act_data_idx),
        .slice_data_index_x_o(act_data_idx_x), .slice_data_index_y_o(act_data_idx_y),
        .slice_data_o(act_data_out), .slice_data_last_o(act_data_last),
        .slice_data_taken_i(act_data_taken),
        .mem_addr_o(act_mem_addr_o), .mem_data_i(act_mem_data_i),
        .mem_req_o(act_mem_req_o), .mem_wait_i(act_mem_wait_i));

    wire act_data_gated_valid = act_data_valid &  act_data_out[ACT_BITS-1];
    wire act_ignore_non_spike = act_data_valid & ~act_data_out[ACT_BITS-1];

    // Mode guard: this wide datapath implements FULL connectivity only
    // (weight_mode 00), which covers fully-connected (Linear / recurrent) layers.
    // Sparse and convolutional connectivity are NOT supported.  For any other mode
    // we must NOT silently run full-mode addressing (that would corrupt
    // syn_curr): instead we skip the output sweep (no syn_curr writes) and let
    // the task drain, plus flag it loudly in simulation.
    wire is_full          = (weight_mode_i == 2'b00);
    wire mode_unsupported = ~is_full;

    // ---------------- wide output sweep (full connectivity) --------------------
    wire [IDX-1:0] num_out = ({{(IDX-X_OUTPUT_SZ){1'b0}}, out_x_len_i})
                           * ({{(IDX-Y_OUTPUT_SZ){1'b0}}, out_y_len_i});

    wire [`ADDR_SIZE-1:0] weight_row_base =
        ({{(`ADDR_SIZE-ACT_DATA_IDX_SZ){1'b0}}, act_data_idx}
         * {{(`ADDR_SIZE-ROWS_PER_NEURON){1'b0}}, rows_per_neuron_i})
        + weight_base_addr_i;

    reg            out_sweep_active_r;
    reg  [IDX-1:0] g_r;

    wire [IDX:0] g_ext   = {1'b0, g_r};
    wire [IDX:0] nout_ext = {1'b0, num_out};
    wire last_group = (g_ext + NUM_LANES) >= nout_ext;

    wire [NUM_LANES-1:0] lane_active;
    genvar a;
    generate for (a = 0; a < NUM_LANES; a = a + 1) begin : g_act
        assign lane_active[a] = out_sweep_active_r & ((g_ext + a) < nout_ext);
    end endgenerate

    // weight fetch (L weights/group)
    wire                       wgt_valid;
    wire [NUM_LANES*WB-1:0]     wgt_slices;
    np_fetch_sel #(.PREFETCH(PREFETCH), .ELEM_BITS(WB), .NUM_LANES(NUM_LANES),
                   .IDX_BITS(IDX), .ADDR_BITS(`ADDR_SIZE))
    weight_rd (
        .clk(clk), .reset(reset),
        .base_addr_i(weight_row_base), .start_index_i(g_r),
        .req_i(out_sweep_active_r),
        .data_valid_o(wgt_valid), .slices_o(wgt_slices),
        .mem_rd_o(weight_mem_rd_o), .mem_addr_o(weight_mem_addr_o),
        .mem_data_i(weight_mem_data_i), .mem_wait_i(weight_mem_wait_i));

    // sign-extend each weight to the syn_curr element width
    wire [NUM_LANES*SC_BITS-1:0] wgt_sext;
    genvar j;
    generate for (j = 0; j < NUM_LANES; j = j + 1) begin : g_sext
        wire [WB-1:0] wj = wgt_slices[j*WB +: WB];
        assign wgt_sext[j*SC_BITS +: SC_BITS] =
            (WB >= SC_BITS) ? wj[SC_BITS-1:0]
                            : {{(SC_BITS-WB){wj[WB-1]}}, wj};
    end endgenerate

    wire acc_done;
    syn_curr_accum_wide #(.ELEM_BITS(SC_BITS), .NUM_LANES(NUM_LANES),
                          .IDX_BITS(IDX), .ADDR_BITS(`ADDR_SIZE))
    accum (
        .clk(clk), .reset(reset),
        .base_addr_i(syn_curr_base_addr_i), .out_index_i(g_r),
        .weights_i(wgt_sext), .active_i(lane_active),
        .go_i(wgt_valid & out_sweep_active_r),
        .busy_o(), .done_o(acc_done),
        .syn_curr_mem_rd_o(syn_curr_mem_rd_o), .syn_curr_mem_wr_o(syn_curr_mem_wr_o),
        .syn_curr_mem_wait_i(syn_curr_mem_wait_i),
        .syn_curr_mem_addr_o(syn_curr_mem_addr_o),
        .syn_curr_mem_data_i(syn_curr_mem_data_rd_i),
        .syn_curr_mem_data_o(syn_curr_mem_data_wr_o));

    wire out_sweep_done = acc_done & last_group;

    always @(posedge clk) begin
        if (reset) begin
            out_sweep_active_r <= 1'b0;
            g_r <= {IDX{1'b0}};
        end else if (~out_sweep_active_r) begin
            if (act_data_gated_valid & is_full) begin    // full mode only
                out_sweep_active_r <= 1'b1;
                g_r <= {IDX{1'b0}};
            end
        end else if (acc_done) begin
            if (last_group) out_sweep_active_r <= 1'b0;
            else            g_r <= g_r + NUM_LANES[IDX-1:0];
        end
    end

    assign wide_busy      = out_sweep_active_r;
    assign act_data_taken = out_sweep_done;
    assign next_in_neuron = out_sweep_done;
    assign act_index_taken = out_sweep_done | (act_ignore_non_spike & ~act_index_wait);

    // synthesis translate_off
    always @(posedge clk)
        if (start_new_block_i & mode_unsupported)
            $display("WARNING [%0t] spike_processing_wide: weight_mode=%0d unsupported (FULL connectivity only on the wide build); task finishes WITHOUT updating syn_curr",
                     $time, weight_mode_i);
    // synthesis translate_on

endmodule
