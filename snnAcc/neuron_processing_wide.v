// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
// =============================================================================
// Module: neuron_processing_wide
// Authors: Simon Davidson & Claude
// Created: 2026-06-04
// Last modified: 2026-06-04
//
// Lane-parallel ("wide") variant of neuron_processing.  Processes NUM_LANES
// neurons per group using the single-fetch / parallel-deserialise approach:
// one np_line_fetch per read stream fetches the 32-bit word(s) covering a group
// of NUM_LANES contiguous neurons and presents all NUM_LANES element slices at
// once; NUM_LANES update_state_for_neuron lanes run in lockstep (II=3 each, one
// 32x32 multiplier each); packer_wide repacks the lane results into words on
// writeback.  Single-port memory throughout — no banking.
//
// External port list is identical to neuron_processing.v so it drops into
// acc_snn_processor unchanged apart from the module name + NUM_LANES.
// NUM_LANES=1 reproduces the original control flow (regression anchor).
//
// Element widths are fixed at compile time by SYN_CURR_SLICE_BITS /
// POT_SLICE_BITS (8 or 16).  This version supports the WPG=1 cells, i.e.
// NUM_LANES <= 32/SLICE_BITS for syn_curr & pot (8b: L<=4, 16b: L<=2).  The
// runtime *_sz_i inputs are ignored (width is a parameter here).
// =============================================================================

`include "../shared/constants.v"

module neuron_processing_wide # (
    parameter TGT_ACC_ID            = 3'b000,
    parameter NUM_LANES             = 2,
    parameter PREFETCH              = 0,   // 1 = prefetch next word (hide fetch bubble)
    parameter NUM_TIMESTEPS         = 32,
    parameter TIMESTEP_SZ           = 10,
    parameter IN_DATA_BITS          = 32,
    parameter NEURON_IDX_SZ         = 10,
    parameter SYN_CURR_IDX_SZ       = 10,
    parameter SYN_CURR_DATA_IDX_SZ  = 5,
    parameter SYN_CURR_SLICE_SZ     = 3,
    parameter SYN_CURR_SLICE_BITS   = 8,
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

    // Config registers
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

    // Scheduler interface
    input  wire                     start_new_block_i,
    input  wire   [`TGT_ACC_SZ-1:0] target_acc_i,
    input  wire [`SCH_ENTRY_SZ-1:0] buffer_info_i,
    output wire                     neuron_proc_finished_o,
    output wire                     acc_busy_o,
    output wire                     acc_finished_o,

    // Buffer-address interface (unused here, kept for port compatibility)
    input wire      [`PIN_BITS-1:0] src1_buff_addr_i,
    input wire      [`PIN_BITS-1:0] src2_buff_addr_i,
    input wire      [`PIN_BITS-1:0] src3_buff_addr_i,
    input wire      [`PIN_BITS-1:0]  tgt_buff_addr_i,
    input wire      [`PIN_BITS-1:0] weight_row_len_i,

    // Synaptic currents (shared, arbitrated upstream)
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
    // Threshold
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

    localparam SC_BITS  = SYN_CURR_SLICE_BITS;
    localparam POTB     = POT_SLICE_BITS;
    localparam BIAS_BITS= BIAS_CURR_SLICE_BITS;   // threshold uses the same width
    localparam WIDX     = NEURON_IDX_SZ + 2;      // widened index domain for compares

    // ---------------------------------------------------------------------
    // Control: cycle over neurons in groups of NUM_LANES
    // ---------------------------------------------------------------------
    reg                      neuron_update_running_r;
    reg  [NEURON_IDX_SZ-1:0] neuron_counter_r;       // first neuron of current group

    wire group_valid;
    wire group_result_valid;
    wire group_taken;

    wire [WIDX-1:0] cnt_ext  = {{(WIDX-NEURON_IDX_SZ){1'b0}}, neuron_counter_r};
    wire [WIDX-1:0] last_ext = {{(WIDX-NEURON_IDX_SZ){1'b0}}, last_neuron_idx_i};
    wire            is_last_group = (cnt_ext + (NUM_LANES-1)) >= last_ext;
    wire            neuron_update_complete = is_last_group & group_taken;

    always @(posedge clk) begin
        if (reset)                                              neuron_update_running_r <= 1'b0;
        else if (start_new_block_i & (target_acc_i == TGT_ACC_ID)) neuron_update_running_r <= 1'b1;
        else if (neuron_update_complete)                        neuron_update_running_r <= 1'b0;
    end

    always @(posedge clk) begin
        if (reset | start_new_block_i) neuron_counter_r <= {NEURON_IDX_SZ{1'b0}};
        else if (group_taken)          neuron_counter_r <= neuron_counter_r + NUM_LANES[NEURON_IDX_SZ-1:0];
    end

    assign acc_busy_o             = neuron_update_running_r;
    assign acc_finished_o         = neuron_update_running_r & neuron_update_complete;
    assign neuron_proc_finished_o = acc_finished_o;

    // Per-lane active mask (final partial group masks the surplus lanes)
    wire [NUM_LANES-1:0] lane_active;
    genvar a;
    generate for (a = 0; a < NUM_LANES; a = a + 1) begin : g_active
        assign lane_active[a] = neuron_update_running_r &
            ((cnt_ext + a) <= last_ext);
    end endgenerate

    // ---------------------------------------------------------------------
    // Read front-ends (single-fetch, parallel-deserialise)
    // ---------------------------------------------------------------------
    wire req = neuron_update_running_r;

    wire [NUM_LANES*SC_BITS-1:0]    syn_slices;
    wire [NUM_LANES*POTB-1:0]       pot_slices;
    wire [NUM_LANES*BIAS_BITS-1:0]  bias_slices;
    wire [NUM_LANES*BIAS_BITS-1:0]  thresh_slices;
    wire syn_dv, pot_dv, bias_dv, thresh_dv;

    // syn_curr: read port muxed with writeback below
    wire                  sc_rd;
    wire [`ADDR_SIZE-1:0] sc_rd_addr;
    wire                  sc_wr;
    wire [`ADDR_SIZE-1:0] sc_wr_addr;
    wire [`POT_BITS-1:0]  sc_wr_data;

    np_fetch_sel #(.PREFETCH(PREFETCH), .ELEM_BITS(SC_BITS), .NUM_LANES(NUM_LANES),
                    .IDX_BITS(NEURON_IDX_SZ), .ADDR_BITS(`ADDR_SIZE))
    syn_rd (
        .clk(clk), .reset(reset),
        .base_addr_i(syn_curr_base_addr_i), .start_index_i(neuron_counter_r), .req_i(req),
        .data_valid_o(syn_dv), .slices_o(syn_slices),
        .mem_rd_o(sc_rd), .mem_addr_o(sc_rd_addr),
        .mem_data_i(syn_curr_mem_data_i), .mem_wait_i(syn_curr_mem_wait_i | sc_wr));

    // pot: read port muxed with writeback below
    wire                  pot_rd;
    wire [`ADDR_SIZE-1:0] pot_rd_addr;
    wire                  pot_wr;
    wire [`ADDR_SIZE-1:0] pot_wr_addr;
    wire [`POT_BITS-1:0]  pot_wr_data;

    np_fetch_sel #(.PREFETCH(PREFETCH), .ELEM_BITS(POTB), .NUM_LANES(NUM_LANES),
                    .IDX_BITS(NEURON_IDX_SZ), .ADDR_BITS(`ADDR_SIZE))
    pot_rd_u (
        .clk(clk), .reset(reset),
        .base_addr_i(pot_base_addr_i), .start_index_i(neuron_counter_r), .req_i(req),
        .data_valid_o(pot_dv), .slices_o(pot_slices),
        .mem_rd_o(pot_rd), .mem_addr_o(pot_rd_addr),
        .mem_data_i(pot_mem_data_i), .mem_wait_i(pot_mem_wait_i | pot_wr));

    // bias (read only)
    np_fetch_sel #(.PREFETCH(PREFETCH), .ELEM_BITS(BIAS_BITS), .NUM_LANES(NUM_LANES),
                    .IDX_BITS(NEURON_IDX_SZ), .ADDR_BITS(`ADDR_SIZE))
    bias_rd (
        .clk(clk), .reset(reset),
        .base_addr_i(bias_curr_base_addr_i), .start_index_i(neuron_counter_r), .req_i(req),
        .data_valid_o(bias_dv), .slices_o(bias_slices),
        .mem_rd_o(bias_curr_mem_rd_o), .mem_addr_o(bias_curr_mem_addr_o),
        .mem_data_i(bias_curr_mem_data_i), .mem_wait_i(bias_curr_mem_wait_i));

    // threshold (read only)
    np_fetch_sel #(.PREFETCH(PREFETCH), .ELEM_BITS(BIAS_BITS), .NUM_LANES(NUM_LANES),
                    .IDX_BITS(NEURON_IDX_SZ), .ADDR_BITS(`ADDR_SIZE))
    thresh_rd (
        .clk(clk), .reset(reset),
        .base_addr_i(thresh_base_addr_i), .start_index_i(neuron_counter_r), .req_i(req),
        .data_valid_o(thresh_dv), .slices_o(thresh_slices),
        .mem_rd_o(thresh_mem_rd_o), .mem_addr_o(thresh_mem_addr_o),
        .mem_data_i(thresh_mem_data_i), .mem_wait_i(thresh_mem_wait_i));

    assign group_valid = syn_dv & pot_dv & bias_dv & thresh_dv;

    // ---------------------------------------------------------------------
    // NUM_LANES update_state lanes (lockstep)
    // ---------------------------------------------------------------------
    wire [NUM_LANES*POTB-1:0]   lane_pot_flat;
    wire [NUM_LANES*SC_BITS-1:0] lane_syn_flat;
    wire [NUM_LANES-1:0]        lane_spk;
    wire [NUM_LANES-1:0]        lane_rvalid;

    genvar j;
    generate for (j = 0; j < NUM_LANES; j = j + 1) begin : g_lane
        wire [POTB-1:0] pot_in = clear_pot_i ? {POTB{1'b0}} : pot_slices[j*POTB +: POTB];
        update_state_for_neuron #(
            .SYN_CURR_SLICE_BITS(SC_BITS),
            .POT_SLICE_BITS(POTB),
            .BIAS_CURR_SLICE_BITS(BIAS_BITS),
            .THRESH_SLICE_BITS(BIAS_BITS),
            .SYN_DECAY_BITS(SYN_DECAY_BITS),
            .POT_DECAY_BITS(POT_DECAY_BITS))
        lane (
            .clk(clk), .reset(reset),
            .neuron_valid_i(group_valid),
            .syn_curr_i (syn_slices[j*SC_BITS +: SC_BITS]),
            .potential_i(pot_in),
            .bias_curr_i(bias_slices[j*BIAS_BITS +: BIAS_BITS]),
            .threshold_i(thresh_slices[j*BIAS_BITS +: BIAS_BITS]),
            .syn_curr_decay_mult_i(syn_curr_decay_mult_i),
            .potential_decay_mult_i(pot_decay_mult_i),
            .sub_on_fire_i(sub_on_fire_i),
            .neuron_taken_o(),
            .result_valid_o(lane_rvalid[j]),
            .potential_o(lane_pot_flat[j*POTB +: POTB]),
            .syn_curr_o (lane_syn_flat[j*SC_BITS +: SC_BITS]),
            .spike_o    (lane_spk[j]),
            .result_taken_i(group_taken));
    end endgenerate

    assign group_result_valid = &lane_rvalid;     // all lanes reach ST_C2 together
    assign group_taken        = group_result_valid;  // mem_wait tied low; no packer backpressure

    // ---------------------------------------------------------------------
    // Writeback packers (WPG=1: one word per group per stream)
    // ---------------------------------------------------------------------
    packer_wide #(.ELEM_BITS(SC_BITS), .NUM_LANES(NUM_LANES),
                  .IDX_BITS(NEURON_IDX_SZ), .ADDR_BITS(`ADDR_SIZE))
    syn_wb (
        .clk(clk), .reset(reset),
        .write_i(group_result_valid), .last_i(is_last_group),
        .base_index_i(neuron_counter_r), .active_i(lane_active),
        .lane_data_i(lane_syn_flat), .base_addr_i(syn_curr_base_addr_i),
        .mem_wr_o(sc_wr), .mem_addr_o(sc_wr_addr), .mem_data_o(sc_wr_data),
        .mem_wait_i(syn_curr_mem_wait_i), .full_o());

    packer_wide #(.ELEM_BITS(POTB), .NUM_LANES(NUM_LANES),
                  .IDX_BITS(NEURON_IDX_SZ), .ADDR_BITS(`ADDR_SIZE))
    pot_wb (
        .clk(clk), .reset(reset),
        .write_i(group_result_valid), .last_i(is_last_group),
        .base_index_i(neuron_counter_r), .active_i(lane_active),
        .lane_data_i(lane_pot_flat), .base_addr_i(pot_base_addr_i),
        .mem_wr_o(pot_wr), .mem_addr_o(pot_wr_addr), .mem_data_o(pot_wr_data),
        .mem_wait_i(pot_mem_wait_i), .full_o());

    packer_wide #(.ELEM_BITS(1), .NUM_LANES(NUM_LANES),
                  .IDX_BITS(NEURON_IDX_SZ), .ADDR_BITS(`ADDR_SIZE))
    spike_wb (
        .clk(clk), .reset(reset),
        .write_i(group_result_valid), .last_i(is_last_group),
        .base_index_i(neuron_counter_r), .active_i(lane_active),
        .lane_data_i(lane_spk), .base_addr_i(spike_base_addr_i),
        .mem_wr_o(spike_mem_wr_o), .mem_addr_o(spike_mem_addr_o),
        .mem_data_o(spike_mem_data_o), .mem_wait_i(spike_mem_wait_i), .full_o());

    // ---------------------------------------------------------------------
    // Mux read/write onto the shared syn_curr and pot ports
    // ---------------------------------------------------------------------
    assign syn_curr_mem_wr_o   = sc_wr;
    assign syn_curr_mem_rd_o   = sc_rd & ~sc_wr;
    assign syn_curr_mem_addr_o = sc_wr ? sc_wr_addr : sc_rd_addr;
    assign syn_curr_mem_data_o = sc_wr_data;

    assign pot_mem_wr_o   = pot_wr;
    assign pot_mem_rd_o   = pot_rd & ~pot_wr;
    assign pot_mem_addr_o = pot_wr ? pot_wr_addr : pot_rd_addr;
    assign pot_mem_data_o = pot_wr_data;

endmodule
