// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
// =============================================================================
// Module: neuron_processing  (FMI variant)
//
// Extends the standard neuron_processing with:
//   - Per-neuron synaptic decay  (dcy_syn_mem)
//   - Per-neuron membrane decay  (dcy_mem_mem)
//   - Optional adaptive threshold state (ada_mem, b_eff_mem, dcy_ada_mem, scl_ada_mem)
//     controlled by has_ada_i config.
//
// All new per-neuron parameter memories are always 32-bit wide (slice_sz = 3'b101).
// The ada_mem is read AND written back each timestep (same pattern as pot_mem).
//
// Removed vs original:
//   - syn_curr_decay_mult_i and pot_decay_mult_i global config inputs
//   - bias_curr memory bank (FMI model has no per-neuron membrane bias)
//
// New memory ports (all exclusive to this module):
//   dcy_syn_mem  — per-neuron Q0.32 synaptic decay factor (read-only)
//   dcy_mem_mem  — per-neuron Q0.32 membrane decay factor (read-only)
//   ada_mem      — per-neuron ada state (read + write)
//   b_eff_mem    — per-neuron scl_mem*adapt_b (read-only)
//   dcy_ada_mem  — per-neuron Q0.32 ada decay (read-only)
//   scl_ada_mem  — per-neuron Q0.32 1-dcy_ada (read-only)
// =============================================================================

`include "../shared/constants.v"

module neuron_processing # (
    parameter TGT_ACC_ID            = 3'b000,
    parameter NUM_TIMESTEPS         = 32,
    parameter TIMESTEP_SZ           = 10,
    parameter IN_DATA_BITS          = 32,
    parameter NEURON_IDX_SZ         = 10,
    parameter SYN_CURR_IDX_SZ       = 10,
    parameter SYN_CURR_DATA_IDX_SZ  = 5,
    parameter SYN_CURR_SLICE_SZ     = 3,
    parameter SYN_CURR_SLICE_BITS   = 32,
    parameter POT_IDX_SZ            = 2,
    parameter POT_DATA_IDX_SZ       = 5,
    parameter POT_SLICE_SZ          = 3,
    parameter POT_SLICE_BITS        = 32,
    parameter SPIKE_IDX_SZ          = 2,
    parameter SPIKE_DATA_IDX_SZ     = 5,
    parameter SPIKE_SLICE_SZ        = 3,
    parameter SPIKE_SLICE_BITS      = 8,
    parameter MEM_ADDR_BITS         = `ADDR_SIZE
)(
    input  wire                    clk,
    input  wire                    reset,

    // Config registers
    input wire  [NEURON_IDX_SZ-1:0]  last_neuron_idx_i,
    input wire  [MEM_ADDR_BITS-1:0]  syn_curr_base_addr_i,
    input wire  [MEM_ADDR_BITS-1:0]  pot_base_addr_i,
    input wire  [MEM_ADDR_BITS-1:0]  spike_base_addr_i,
    input wire  [MEM_ADDR_BITS-1:0]  thresh_base_addr_i,
    input wire  [SYN_CURR_SLICE_SZ-1:0] syn_curr_sz_i,
    input wire  [POT_SLICE_SZ-1:0]   pot_sz_i,
    input wire  [4:0]                bin_point_syn_curr_i,
    input wire                       sub_on_fire_i,
    input wire                       clear_pot_i,
    // Per-neuron decay memory base addresses
    input wire  [MEM_ADDR_BITS-1:0]  dcy_syn_base_addr_i,
    input wire  [MEM_ADDR_BITS-1:0]  dcy_mem_base_addr_i,
    // Adaptive threshold config
    input wire                       has_ada_i,
    input wire  [MEM_ADDR_BITS-1:0]  ada_base_addr_i,
    input wire  [MEM_ADDR_BITS-1:0]  b_eff_base_addr_i,
    input wire  [MEM_ADDR_BITS-1:0]  dcy_ada_base_addr_i,
    input wire  [MEM_ADDR_BITS-1:0]  scl_ada_base_addr_i,

    // Scheduler interface
    input  wire                      start_new_block_i,
    input  wire  [`TGT_ACC_SZ-1:0]   target_acc_i,
    input  wire  [`SCH_ENTRY_SZ-1:0] buffer_info_i,
    output wire                      neuron_proc_finished_o,
    output wire                      acc_busy_o,
    output wire                      acc_finished_o,

    // Buffer address interface (unused ports kept for scheduler compatibility)
    input wire  [`PIN_BITS-1:0]  src1_buff_addr_i,
    input wire  [`PIN_BITS-1:0]  src2_buff_addr_i,
    input wire  [`PIN_BITS-1:0]  src3_buff_addr_i,
    input wire  [`PIN_BITS-1:0]   tgt_buff_addr_i,
    input wire  [`PIN_BITS-1:0]  weight_row_len_i,

    // Memory: synaptic currents (shared R/W)
    output wire                      syn_curr_mem_wr_o,
    output wire                      syn_curr_mem_rd_o,
    input  wire                      syn_curr_mem_wait_i,
    output wire   [`ADDR_SIZE-1:0]   syn_curr_mem_addr_o,
    output wire    [`POT_BITS-1:0]   syn_curr_mem_data_o,
    input  wire    [`POT_BITS-1:0]   syn_curr_mem_data_i,

    // Memory: threshold (read-only)
    output wire                      thresh_mem_rd_o,
    input  wire                      thresh_mem_wait_i,
    output wire   [`ADDR_SIZE-1:0]   thresh_mem_addr_o,
    input  wire    [`WTD_BITS-1:0]   thresh_mem_data_i,

    // Memory: potentials (R/W)
    output wire                      pot_mem_wr_o,
    output wire                      pot_mem_rd_o,
    input  wire                      pot_mem_wait_i,
    output wire   [`ADDR_SIZE-1:0]   pot_mem_addr_o,
    output wire    [`POT_BITS-1:0]   pot_mem_data_o,
    input  wire    [`POT_BITS-1:0]   pot_mem_data_i,

    // Memory: output spikes
    output wire                      spike_mem_wr_o,
    input  wire                      spike_mem_wait_i,
    output wire   [`ADDR_SIZE-1:0]   spike_mem_addr_o,
    output wire    [`ACT_BITS-1:0]   spike_mem_data_o,

    // Memory: per-neuron syn decay (read-only, always 32-bit)
    output wire                      dcy_syn_mem_rd_o,
    input  wire                      dcy_syn_mem_wait_i,
    output wire   [`ADDR_SIZE-1:0]   dcy_syn_mem_addr_o,
    input  wire            [31:0]    dcy_syn_mem_data_i,

    // Memory: per-neuron mem decay (read-only, always 32-bit)
    output wire                      dcy_mem_mem_rd_o,
    input  wire                      dcy_mem_mem_wait_i,
    output wire   [`ADDR_SIZE-1:0]   dcy_mem_mem_addr_o,
    input  wire            [31:0]    dcy_mem_mem_data_i,

    // Memory: ada state (R/W, 32-bit)
    output wire                      ada_mem_wr_o,
    output wire                      ada_mem_rd_o,
    input  wire                      ada_mem_wait_i,
    output wire   [`ADDR_SIZE-1:0]   ada_mem_addr_o,
    output wire            [31:0]    ada_mem_data_o,
    input  wire            [31:0]    ada_mem_data_i,

    // Memory: b_eff (read-only, 32-bit signed)
    output wire                      b_eff_mem_rd_o,
    input  wire                      b_eff_mem_wait_i,
    output wire   [`ADDR_SIZE-1:0]   b_eff_mem_addr_o,
    input  wire            [31:0]    b_eff_mem_data_i,

    // Memory: dcy_ada (read-only, 32-bit Q0.32)
    output wire                      dcy_ada_mem_rd_o,
    input  wire                      dcy_ada_mem_wait_i,
    output wire   [`ADDR_SIZE-1:0]   dcy_ada_mem_addr_o,
    input  wire            [31:0]    dcy_ada_mem_data_i,

    // Memory: scl_ada (read-only, 32-bit Q0.32)
    output wire                      scl_ada_mem_rd_o,
    input  wire                      scl_ada_mem_wait_i,
    output wire   [`ADDR_SIZE-1:0]   scl_ada_mem_addr_o,
    input  wire            [31:0]    scl_ada_mem_data_i
);

    // =========================================================================
    // Forward declarations (used before instantiation)
    // =========================================================================
    wire                      neuron_taken;
    wire                      result_valid;
    wire                      result_taken;
    wire                      spike;
    wire [POT_SLICE_BITS-1:0] updated_potential;
    wire [SYN_CURR_SLICE_BITS-1:0] updated_syn_curr;
    wire [31:0]               updated_ada;
    wire                      packer_busy, packer_finish, packer_full;

    // =========================================================================
    // Neuron counter
    // =========================================================================
    reg                              neuron_update_running_r;
    reg        [NEURON_IDX_SZ-1:0]  neuron_counter_r;

    wire neuron_update_complete = (neuron_counter_r == last_neuron_idx_i) & neuron_taken;
    wire next_neuron = neuron_taken;

    always @(posedge clk) begin
        if (reset)
            neuron_update_running_r <= 1'b0;
        else if (start_new_block_i & (target_acc_i == TGT_ACC_ID))
            neuron_update_running_r <= 1'b1;
        else if (neuron_update_complete)
            neuron_update_running_r <= 1'b0;
    end

    assign acc_busy_o             = neuron_update_running_r;
    assign acc_finished_o         = neuron_update_running_r & neuron_update_complete;
    assign neuron_proc_finished_o = acc_finished_o;

    always @(posedge clk) begin
        if (reset | start_new_block_i)
            neuron_counter_r <= 'b0;
        else if (next_neuron)
            neuron_counter_r <= neuron_counter_r + 1'b1;
    end

    wire last_neuron = (neuron_counter_r == last_neuron_idx_i);

    // =========================================================================
    // Cache instances — shared parameter block
    // =========================================================================
    // All per-neuron index generation uses neuron_counter_r directly.
    // Slice size for 32-bit params is always 3'b101 (32-bit elements).
    localparam PARAM_IDX_SZ      = NEURON_IDX_SZ;
    localparam PARAM_DATA_IDX_SZ = 5;
    localparam PARAM_SLICE_SZ    = 3;
    localparam PARAM_SLICE_BITS  = 32;

    // =========================================================================
    // Synaptic current cache (existing pattern)
    // =========================================================================
    wire                      syn_curr_data_valid;
    wire [SYN_CURR_SLICE_BITS-1:0] syn_curr_data_out;
    wire                      syn_curr_wait;
    wire [`ADDR_SIZE-1:0]     syn_curr_cache_mem_addr;
    wire                      syn_curr_cache_mem_rd;
    wire                      syn_curr_wb_wr;
    wire [`ADDR_SIZE-1:0]     syn_curr_wb_addr;
    wire [31:0]               syn_curr_wb_data_bus;
    wire                      syn_curr_wb_full;

    dataline_cache_with_xy #(
        .IN_DATA_BITS       (IN_DATA_BITS),
        .X_INPUT_SZ         (1), .Y_INPUT_SZ(1),
        .IDX_ADDR_BITS      (SYN_CURR_IDX_SZ),
        .SLICE_DATA_IDX_SZ  (SYN_CURR_DATA_IDX_SZ),
        .SLICE_SIZE_SZ      (SYN_CURR_SLICE_SZ),
        .OUT_DATA_BITS      (SYN_CURR_SLICE_BITS))
    syn_curr_cache (
        .clk(clk), .reset(reset),
        .colour_select_o(), .slice_sz_i(syn_curr_sz_i),
        .base_addr_i(syn_curr_base_addr_i),
        .sys_addr_i(neuron_counter_r), .sys_req_i(neuron_update_running_r),
        .sys_index_x_i(1'b0), .sys_index_y_i(1'b0), .sys_colour_i(1'b0),
        .sys_wait_o(syn_curr_wait), .sys_last_i(last_neuron),
        .slice_data_valid_o(syn_curr_data_valid),
        .slice_data_idx_o(), .slice_data_index_x_o(), .slice_data_index_y_o(),
        .slice_data_o(syn_curr_data_out), .slice_data_last_o(),
        .slice_data_taken_i(neuron_taken),
        .mem_addr_o(syn_curr_cache_mem_addr),
        .mem_data_i(syn_curr_mem_data_i),
        .mem_req_o(syn_curr_cache_mem_rd),
        .mem_wait_i(syn_curr_mem_wait_i | syn_curr_wb_wr)
    );

    // =========================================================================
    // Threshold cache
    // =========================================================================
    wire                      thresh_data_valid;
    wire [POT_SLICE_BITS-1:0] thresh_data_out;
    wire                      thresh_wait;

    dataline_cache_with_xy #(
        .IN_DATA_BITS       (IN_DATA_BITS),
        .X_INPUT_SZ         (1), .Y_INPUT_SZ(1),
        .IDX_ADDR_BITS      (PARAM_IDX_SZ),
        .SLICE_DATA_IDX_SZ  (PARAM_DATA_IDX_SZ),
        .SLICE_SIZE_SZ      (PARAM_SLICE_SZ),
        .OUT_DATA_BITS      (PARAM_SLICE_BITS))
    threshold_cache (
        .clk(clk), .reset(reset),
        .colour_select_o(), .slice_sz_i(3'b101),          // always 32-bit
        .base_addr_i(thresh_base_addr_i),
        .sys_addr_i(neuron_counter_r), .sys_req_i(neuron_update_running_r),
        .sys_index_x_i(1'b0), .sys_index_y_i(1'b0), .sys_colour_i(1'b0),
        .sys_wait_o(thresh_wait), .sys_last_i(last_neuron),
        .slice_data_valid_o(thresh_data_valid),
        .slice_data_idx_o(), .slice_data_index_x_o(), .slice_data_index_y_o(),
        .slice_data_o(thresh_data_out), .slice_data_last_o(),
        .slice_data_taken_i(neuron_taken),
        .mem_addr_o(thresh_mem_addr_o),
        .mem_data_i(thresh_mem_data_i),
        .mem_req_o(thresh_mem_rd_o),
        .mem_wait_i(thresh_mem_wait_i)
    );

    // =========================================================================
    // Potential cache
    // =========================================================================
    wire                      pot_data_valid;
    wire [POT_SLICE_BITS-1:0] pot_data_out;
    wire                      pot_wait;
    wire [`ADDR_SIZE-1:0]     pot_cache_mem_addr;
    wire                      pot_cache_mem_rd;
    wire                      pot_wb_wr;
    wire [`ADDR_SIZE-1:0]     pot_wb_addr;
    wire [31:0]               pot_wb_data_bus;
    wire                      pot_wb_full;

    dataline_cache_with_xy #(
        .IN_DATA_BITS       (IN_DATA_BITS),
        .X_INPUT_SZ         (1), .Y_INPUT_SZ(1),
        .IDX_ADDR_BITS      (POT_IDX_SZ),
        .SLICE_DATA_IDX_SZ  (POT_DATA_IDX_SZ),
        .SLICE_SIZE_SZ      (POT_SLICE_SZ),
        .OUT_DATA_BITS      (POT_SLICE_BITS))
    pot_cache (
        .clk(clk), .reset(reset),
        .colour_select_o(), .slice_sz_i(pot_sz_i),
        .base_addr_i(pot_base_addr_i),
        .sys_addr_i(neuron_counter_r), .sys_req_i(neuron_update_running_r),
        .sys_index_x_i(1'b0), .sys_index_y_i(1'b0), .sys_colour_i(1'b0),
        .sys_wait_o(pot_wait), .sys_last_i(last_neuron),
        .slice_data_valid_o(pot_data_valid),
        .slice_data_idx_o(), .slice_data_index_x_o(), .slice_data_index_y_o(),
        .slice_data_o(pot_data_out), .slice_data_last_o(),
        .slice_data_taken_i(neuron_taken),
        .mem_addr_o(pot_cache_mem_addr),
        .mem_data_i(pot_mem_data_i),
        .mem_req_o(pot_cache_mem_rd),
        .mem_wait_i(pot_mem_wait_i | pot_wb_wr)
    );

    // =========================================================================
    // Per-neuron synaptic decay cache (dcy_syn, 32-bit, read-only)
    // =========================================================================
    wire                dcy_syn_data_valid;
    wire [31:0]         dcy_syn_data_out;
    wire                dcy_syn_wait;

    dataline_cache_with_xy #(
        .IN_DATA_BITS       (IN_DATA_BITS),
        .X_INPUT_SZ         (1), .Y_INPUT_SZ(1),
        .IDX_ADDR_BITS      (PARAM_IDX_SZ),
        .SLICE_DATA_IDX_SZ  (PARAM_DATA_IDX_SZ),
        .SLICE_SIZE_SZ      (PARAM_SLICE_SZ),
        .OUT_DATA_BITS      (PARAM_SLICE_BITS))
    dcy_syn_cache (
        .clk(clk), .reset(reset),
        .colour_select_o(), .slice_sz_i(3'b101),
        .base_addr_i(dcy_syn_base_addr_i),
        .sys_addr_i(neuron_counter_r), .sys_req_i(neuron_update_running_r),
        .sys_index_x_i(1'b0), .sys_index_y_i(1'b0), .sys_colour_i(1'b0),
        .sys_wait_o(dcy_syn_wait), .sys_last_i(last_neuron),
        .slice_data_valid_o(dcy_syn_data_valid),
        .slice_data_idx_o(), .slice_data_index_x_o(), .slice_data_index_y_o(),
        .slice_data_o(dcy_syn_data_out), .slice_data_last_o(),
        .slice_data_taken_i(neuron_taken),
        .mem_addr_o(dcy_syn_mem_addr_o),
        .mem_data_i(dcy_syn_mem_data_i),
        .mem_req_o(dcy_syn_mem_rd_o),
        .mem_wait_i(dcy_syn_mem_wait_i)
    );

    // =========================================================================
    // Per-neuron membrane decay cache (dcy_mem, 32-bit, read-only)
    // =========================================================================
    wire                dcy_mem_data_valid;
    wire [31:0]         dcy_mem_data_out;
    wire                dcy_mem_wait;

    dataline_cache_with_xy #(
        .IN_DATA_BITS       (IN_DATA_BITS),
        .X_INPUT_SZ         (1), .Y_INPUT_SZ(1),
        .IDX_ADDR_BITS      (PARAM_IDX_SZ),
        .SLICE_DATA_IDX_SZ  (PARAM_DATA_IDX_SZ),
        .SLICE_SIZE_SZ      (PARAM_SLICE_SZ),
        .OUT_DATA_BITS      (PARAM_SLICE_BITS))
    dcy_mem_cache (
        .clk(clk), .reset(reset),
        .colour_select_o(), .slice_sz_i(3'b101),
        .base_addr_i(dcy_mem_base_addr_i),
        .sys_addr_i(neuron_counter_r), .sys_req_i(neuron_update_running_r),
        .sys_index_x_i(1'b0), .sys_index_y_i(1'b0), .sys_colour_i(1'b0),
        .sys_wait_o(dcy_mem_wait), .sys_last_i(last_neuron),
        .slice_data_valid_o(dcy_mem_data_valid),
        .slice_data_idx_o(), .slice_data_index_x_o(), .slice_data_index_y_o(),
        .slice_data_o(dcy_mem_data_out), .slice_data_last_o(),
        .slice_data_taken_i(neuron_taken),
        .mem_addr_o(dcy_mem_mem_addr_o),
        .mem_data_i(dcy_mem_mem_data_i),
        .mem_req_o(dcy_mem_mem_rd_o),
        .mem_wait_i(dcy_mem_mem_wait_i)
    );

    // =========================================================================
    // Ada-specific caches (all gated by has_ada_i)
    // =========================================================================

    // ada state cache (R/W)
    wire                ada_data_valid;
    wire [31:0]         ada_data_out;
    wire                ada_wait;
    wire [`ADDR_SIZE-1:0] ada_cache_mem_addr;
    wire                ada_cache_mem_rd;
    wire                ada_wb_wr;
    wire [`ADDR_SIZE-1:0] ada_wb_addr;
    wire [31:0]         ada_wb_data_bus;
    wire                ada_wb_full;

    dataline_cache_with_xy #(
        .IN_DATA_BITS       (IN_DATA_BITS),
        .X_INPUT_SZ         (1), .Y_INPUT_SZ(1),
        .IDX_ADDR_BITS      (PARAM_IDX_SZ),
        .SLICE_DATA_IDX_SZ  (PARAM_DATA_IDX_SZ),
        .SLICE_SIZE_SZ      (PARAM_SLICE_SZ),
        .OUT_DATA_BITS      (PARAM_SLICE_BITS))
    ada_cache (
        .clk(clk), .reset(reset),
        .colour_select_o(), .slice_sz_i(3'b101),
        .base_addr_i(ada_base_addr_i),
        .sys_addr_i(neuron_counter_r), .sys_req_i(neuron_update_running_r & has_ada_i),
        .sys_index_x_i(1'b0), .sys_index_y_i(1'b0), .sys_colour_i(1'b0),
        .sys_wait_o(ada_wait), .sys_last_i(last_neuron),
        .slice_data_valid_o(ada_data_valid),
        .slice_data_idx_o(), .slice_data_index_x_o(), .slice_data_index_y_o(),
        .slice_data_o(ada_data_out), .slice_data_last_o(),
        .slice_data_taken_i(neuron_taken),
        .mem_addr_o(ada_cache_mem_addr),
        .mem_data_i(ada_mem_data_i),
        .mem_req_o(ada_cache_mem_rd),
        .mem_wait_i(ada_mem_wait_i | ada_wb_wr)
    );

    // b_eff cache (read-only)
    wire                b_eff_data_valid;
    wire [31:0]         b_eff_data_out;
    wire                b_eff_wait;

    dataline_cache_with_xy #(
        .IN_DATA_BITS       (IN_DATA_BITS),
        .X_INPUT_SZ         (1), .Y_INPUT_SZ(1),
        .IDX_ADDR_BITS      (PARAM_IDX_SZ),
        .SLICE_DATA_IDX_SZ  (PARAM_DATA_IDX_SZ),
        .SLICE_SIZE_SZ      (PARAM_SLICE_SZ),
        .OUT_DATA_BITS      (PARAM_SLICE_BITS))
    b_eff_cache (
        .clk(clk), .reset(reset),
        .colour_select_o(), .slice_sz_i(3'b101),
        .base_addr_i(b_eff_base_addr_i),
        .sys_addr_i(neuron_counter_r), .sys_req_i(neuron_update_running_r & has_ada_i),
        .sys_index_x_i(1'b0), .sys_index_y_i(1'b0), .sys_colour_i(1'b0),
        .sys_wait_o(b_eff_wait), .sys_last_i(last_neuron),
        .slice_data_valid_o(b_eff_data_valid),
        .slice_data_idx_o(), .slice_data_index_x_o(), .slice_data_index_y_o(),
        .slice_data_o(b_eff_data_out), .slice_data_last_o(),
        .slice_data_taken_i(neuron_taken),
        .mem_addr_o(b_eff_mem_addr_o),
        .mem_data_i(b_eff_mem_data_i),
        .mem_req_o(b_eff_mem_rd_o),
        .mem_wait_i(b_eff_mem_wait_i)
    );

    // dcy_ada cache (read-only)
    wire                dcy_ada_data_valid;
    wire [31:0]         dcy_ada_data_out;
    wire                dcy_ada_wait;

    dataline_cache_with_xy #(
        .IN_DATA_BITS       (IN_DATA_BITS),
        .X_INPUT_SZ         (1), .Y_INPUT_SZ(1),
        .IDX_ADDR_BITS      (PARAM_IDX_SZ),
        .SLICE_DATA_IDX_SZ  (PARAM_DATA_IDX_SZ),
        .SLICE_SIZE_SZ      (PARAM_SLICE_SZ),
        .OUT_DATA_BITS      (PARAM_SLICE_BITS))
    dcy_ada_cache (
        .clk(clk), .reset(reset),
        .colour_select_o(), .slice_sz_i(3'b101),
        .base_addr_i(dcy_ada_base_addr_i),
        .sys_addr_i(neuron_counter_r), .sys_req_i(neuron_update_running_r & has_ada_i),
        .sys_index_x_i(1'b0), .sys_index_y_i(1'b0), .sys_colour_i(1'b0),
        .sys_wait_o(dcy_ada_wait), .sys_last_i(last_neuron),
        .slice_data_valid_o(dcy_ada_data_valid),
        .slice_data_idx_o(), .slice_data_index_x_o(), .slice_data_index_y_o(),
        .slice_data_o(dcy_ada_data_out), .slice_data_last_o(),
        .slice_data_taken_i(neuron_taken),
        .mem_addr_o(dcy_ada_mem_addr_o),
        .mem_data_i(dcy_ada_mem_data_i),
        .mem_req_o(dcy_ada_mem_rd_o),
        .mem_wait_i(dcy_ada_mem_wait_i)
    );

    // scl_ada cache (read-only)
    wire                scl_ada_data_valid;
    wire [31:0]         scl_ada_data_out;
    wire                scl_ada_wait;

    dataline_cache_with_xy #(
        .IN_DATA_BITS       (IN_DATA_BITS),
        .X_INPUT_SZ         (1), .Y_INPUT_SZ(1),
        .IDX_ADDR_BITS      (PARAM_IDX_SZ),
        .SLICE_DATA_IDX_SZ  (PARAM_DATA_IDX_SZ),
        .SLICE_SIZE_SZ      (PARAM_SLICE_SZ),
        .OUT_DATA_BITS      (PARAM_SLICE_BITS))
    scl_ada_cache (
        .clk(clk), .reset(reset),
        .colour_select_o(), .slice_sz_i(3'b101),
        .base_addr_i(scl_ada_base_addr_i),
        .sys_addr_i(neuron_counter_r), .sys_req_i(neuron_update_running_r & has_ada_i),
        .sys_index_x_i(1'b0), .sys_index_y_i(1'b0), .sys_colour_i(1'b0),
        .sys_wait_o(scl_ada_wait), .sys_last_i(last_neuron),
        .slice_data_valid_o(scl_ada_data_valid),
        .slice_data_idx_o(), .slice_data_index_x_o(), .slice_data_index_y_o(),
        .slice_data_o(scl_ada_data_out), .slice_data_last_o(),
        .slice_data_taken_i(neuron_taken),
        .mem_addr_o(scl_ada_mem_addr_o),
        .mem_data_i(scl_ada_mem_data_i),
        .mem_req_o(scl_ada_mem_rd_o),
        .mem_wait_i(scl_ada_mem_wait_i)
    );

    // =========================================================================
    // neuron_valid: all required data must be present
    // Ada caches contribute only when has_ada_i is set
    // =========================================================================
    wire ada_inputs_valid = ~has_ada_i | (ada_data_valid & b_eff_data_valid &
                                          dcy_ada_data_valid & scl_ada_data_valid);

    wire neuron_valid = syn_curr_data_valid & pot_data_valid & thresh_data_valid &
                        dcy_syn_data_valid   & dcy_mem_data_valid & ada_inputs_valid;

    // Clear pot when config requests it
    wire signed [POT_SLICE_BITS-1:0] pot_input;
    assign pot_input = clear_pot_i ? {POT_SLICE_BITS{1'b0}} : $signed(pot_data_out);

    // =========================================================================
    // update_state_for_neuron instantiation
    // =========================================================================
    update_state_for_neuron #(
        .SYN_CURR_SLICE_BITS (SYN_CURR_SLICE_BITS),
        .POT_SLICE_BITS      (POT_SLICE_BITS),
        .THRESH_SLICE_BITS   (POT_SLICE_BITS)
    ) neuron_update0 (
        .clk              (clk),
        .reset            (reset),
        .neuron_valid_i   (neuron_valid),
        .syn_curr_i       ($signed(syn_curr_data_out)),
        .potential_i      (pot_input),
        .threshold_i      ($signed(thresh_data_out)),
        .syn_dcy_i        (dcy_syn_data_out),
        .mem_dcy_i        (dcy_mem_data_out),
        .has_ada_i        (has_ada_i),
        .ada_i            (ada_data_out),
        .b_eff_i          ($signed(b_eff_data_out)),
        .dcy_ada_i        (dcy_ada_data_out),
        .scl_ada_i        (scl_ada_data_out),
        .neuron_taken_o   (neuron_taken),
        .result_valid_o   (result_valid),
        .potential_o      (updated_potential),
        .syn_curr_o       (updated_syn_curr),
        .spike_o          (spike),
        .ada_o            (updated_ada),
        .result_taken_i   (result_taken)
    );

    // =========================================================================
    // Write-back packers
    // =========================================================================

    // Left-justify helpers for variable-width writeback
    reg [31:0] syn_curr_wb_lj;
    reg [31:0] pot_wb_lj;

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

    // Stall update until ALL packers can accept
    assign result_taken = result_valid & ~packer_full & ~syn_curr_wb_full &
                          ~pot_wb_full & ~ada_wb_full;

    // Spike packer
    packer spike_packer0 (
        .clk(clk), .reset(reset),
        .busy_o(packer_busy), .finish_o(packer_finish),
        .pak_write_i(result_valid),   .pak_full_o(packer_full),
        .pak_colour_i(1'b0),          .pak_last_i(last_neuron),
        .pak_index_i({{(`PIN_BITS-NEURON_IDX_SZ){1'b0}}, neuron_counter_r}),
        .pak_acc_data_i({spike, {(`POT_BITS-1){1'b0}}}),
        .pot_wr_o(spike_mem_wr_o),    .pot_wait_i(spike_mem_wait_i),
        .pot_addr_o(spike_mem_addr_o),.pot_data_o(spike_mem_data_o),
        .pak_colour_sel_o(), .pak_out_sz_i({`POT_OUT_SZ_SZ{1'b0}}),
        .pak_colour_bs_o(), .pak_out_base_addr_i(spike_base_addr_i)
    );

    // Syn curr writeback packer
    packer syn_curr_wb_packer (
        .clk(clk), .reset(reset),
        .busy_o(), .finish_o(),
        .pak_write_i(result_valid),   .pak_full_o(syn_curr_wb_full),
        .pak_colour_i(1'b0),          .pak_last_i(last_neuron),
        .pak_index_i({{(`PIN_BITS-NEURON_IDX_SZ){1'b0}}, neuron_counter_r}),
        .pak_acc_data_i(syn_curr_wb_lj),
        .pot_wr_o(syn_curr_wb_wr),    .pot_wait_i(syn_curr_mem_wait_i),
        .pot_addr_o(syn_curr_wb_addr),.pot_data_o(syn_curr_wb_data_bus),
        .pak_colour_sel_o(), .pak_out_sz_i(syn_curr_sz_i),
        .pak_colour_bs_o(), .pak_out_base_addr_i(syn_curr_base_addr_i)
    );

    // Potential writeback packer
    packer pot_wb_packer (
        .clk(clk), .reset(reset),
        .busy_o(), .finish_o(),
        .pak_write_i(result_valid),   .pak_full_o(pot_wb_full),
        .pak_colour_i(1'b0),          .pak_last_i(last_neuron),
        .pak_index_i({{(`PIN_BITS-NEURON_IDX_SZ){1'b0}}, neuron_counter_r}),
        .pak_acc_data_i(pot_wb_lj),
        .pot_wr_o(pot_wb_wr),         .pot_wait_i(pot_mem_wait_i),
        .pot_addr_o(pot_wb_addr),     .pot_data_o(pot_wb_data_bus),
        .pak_colour_sel_o(), .pak_out_sz_i(pot_sz_i),
        .pak_colour_bs_o(), .pak_out_base_addr_i(pot_base_addr_i)
    );

    // Ada writeback packer (writes updated_ada back to ada_mem, gated by has_ada_i)
    // When has_ada_i=0, ada_wb_full is always 0 (packer never fills)
    wire ada_wb_full_raw;
    assign ada_wb_full = has_ada_i & ada_wb_full_raw;

    packer ada_wb_packer (
        .clk(clk), .reset(reset),
        .busy_o(), .finish_o(),
        .pak_write_i(result_valid & has_ada_i),  .pak_full_o(ada_wb_full_raw),
        .pak_colour_i(1'b0),                     .pak_last_i(last_neuron),
        .pak_index_i({{(`PIN_BITS-NEURON_IDX_SZ){1'b0}}, neuron_counter_r}),
        .pak_acc_data_i(updated_ada),
        .pot_wr_o(ada_wb_wr),         .pot_wait_i(ada_mem_wait_i),
        .pot_addr_o(ada_wb_addr),     .pot_data_o(ada_wb_data_bus),
        .pak_colour_sel_o(), .pak_out_sz_i(3'b101),         // always 32-bit
        .pak_colour_bs_o(), .pak_out_base_addr_i(ada_base_addr_i)
    );

    // =========================================================================
    // Memory bus assignments
    // =========================================================================
    assign syn_curr_mem_wr_o   = syn_curr_wb_wr;
    assign syn_curr_mem_rd_o   = syn_curr_cache_mem_rd & ~syn_curr_wb_wr;
    assign syn_curr_mem_addr_o = syn_curr_wb_wr ? syn_curr_wb_addr : syn_curr_cache_mem_addr;
    assign syn_curr_mem_data_o = syn_curr_wb_data_bus;

    assign pot_mem_wr_o        = pot_wb_wr;
    assign pot_mem_rd_o        = pot_cache_mem_rd & ~pot_wb_wr;
    assign pot_mem_addr_o      = pot_wb_wr ? pot_wb_addr : pot_cache_mem_addr;
    assign pot_mem_data_o      = pot_wb_data_bus;

    assign ada_mem_wr_o        = ada_wb_wr;
    assign ada_mem_rd_o        = ada_cache_mem_rd & ~ada_wb_wr;
    assign ada_mem_addr_o      = ada_wb_wr ? ada_wb_addr : ada_cache_mem_addr;
    assign ada_mem_data_o      = ada_wb_data_bus;

endmodule
