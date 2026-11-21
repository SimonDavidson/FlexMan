// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
// =============================================================================
// Module: packer_wide
// Authors: Simon Davidson & Claude
// Created: 2026-06-04
// Last modified: 2026-06-04
//
// Wide writeback packer for the lane-parallel neuron_processing experiment.
// Accepts NUM_LANES results per group and packs them, LSB-first, into a 32-bit
// word at their element columns (element k -> bits[k*ELEM_BITS +: ELEM_BITS],
// matching np_line_fetch / slice_and_align).  Accumulates across groups when a
// group only partially fills a word (e.g. 8-bit x2 lanes fills a word every two
// groups) and emits the word when the top column is written or on the last
// group.  One write per word, single-port memory (no banking).
//
// Handles ELEM_BITS in {1,8,16}: 1-bit is the spike stream (32 per word); 8/16
// are the syn_curr / potential writeback streams.  Requires NUM_LANES <= EPW
// (the WPG=1 case: one word per group).  The WPG>1 case (16-bit x4) is handled
// by instantiating one packer_wide per word with the right lane subset.
//
// Group base index g = base_index_i; the group's word index is g>>log2(EPW)
// and the within-word column of lane j is (g mod EPW)+j.  Inactive lanes
// (final partial group) are masked out via active_i.
//
// mem_wait_i is assumed low (as in the testbenches); writes are taken the cycle
// they are issued.
// =============================================================================
`timescale 1ns / 1ps

module packer_wide #(
    parameter ELEM_BITS    = 8,
    parameter NUM_LANES    = 2,
    parameter IDX_BITS     = 10,
    parameter ADDR_BITS    = 30,
    parameter IN_DATA_BITS = 32
)(
    input  wire                           clk,
    input  wire                           reset,

    input  wire                           write_i,       // group produced results
    input  wire                           last_i,        // group holds the last neuron
    input  wire          [IDX_BITS-1:0]   base_index_i,  // g
    input  wire        [NUM_LANES-1:0]    active_i,      // per-lane active mask
    input  wire [NUM_LANES*ELEM_BITS-1:0] lane_data_i,
    input  wire         [ADDR_BITS-1:0]   base_addr_i,

    output wire                           mem_wr_o,
    output wire         [ADDR_BITS-1:0]   mem_addr_o,
    output wire      [IN_DATA_BITS-1:0]   mem_data_o,
    input  wire                           mem_wait_i,
    output wire                           full_o
);

    localparam EPW     = IN_DATA_BITS / ELEM_BITS;
    localparam LOG_EPW = (EPW <= 1) ? 1 : $clog2(EPW);

    wire [LOG_EPW-1:0]  g_low = base_index_i[LOG_EPW-1:0];

    reg [IN_DATA_BITS-1:0] output_buffer_r;

    // Assemble this group's contribution: each active lane's element at its
    // within-word column (g_low + j).
    reg [IN_DATA_BITS-1:0] new_bits;
    integer j;
    always @(*) begin
        new_bits = {IN_DATA_BITS{1'b0}};
        for (j = 0; j < NUM_LANES; j = j + 1)
            if (active_i[j])
                new_bits = new_bits |
                    ({{(IN_DATA_BITS-ELEM_BITS){1'b0}}, lane_data_i[j*ELEM_BITS +: ELEM_BITS]}
                        << ((g_low + j) * ELEM_BITS));
    end

    // Highest active lane -> top column written this group.
    integer k;
    reg [LOG_EPW:0] hi_lane;
    always @(*) begin
        hi_lane = {(LOG_EPW+1){1'b0}};
        for (k = 0; k < NUM_LANES; k = k + 1)
            if (active_i[k]) hi_lane = k[LOG_EPW:0];
    end
    wire [LOG_EPW:0] col_top   = {1'b0, g_low} + hi_lane;
    wire             word_full = (col_top == (EPW-1)) | last_i;

    wire [IN_DATA_BITS-1:0] merged = output_buffer_r | new_bits;

    always @(posedge clk) begin
        if (reset)
            output_buffer_r <= {IN_DATA_BITS{1'b0}};
        else if (write_i) begin
            if (word_full) output_buffer_r <= {IN_DATA_BITS{1'b0}};
            else           output_buffer_r <= merged;
        end
    end

    assign mem_data_o = merged;
    assign mem_addr_o = base_addr_i
                      + {{(ADDR_BITS-(IDX_BITS-LOG_EPW)){1'b0}},
                         base_index_i[IDX_BITS-1:LOG_EPW]};   // word index = g >> log2(EPW)
    assign mem_wr_o   = write_i & word_full;
    assign full_o     = 1'b0;   // mem_wait tied low in testbenches; no backpressure

endmodule
