// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
// =============================================================================
// Module: syn_curr_accum_wide
// Authors: Simon Davidson & Claude
// Created: 2026-06-07
// Last modified: 2026-06-07
//
// Wide read-modify-write accumulator for the lane-parallel spike_processing.
// For one lane-group of NUM_LANES contiguous output neurons it:
//   1) reads the 32-bit syn_curr word holding those neurons (DIRECT, uncached —
//      the value must reflect every prior input spike's contribution),
//   2) adds each active lane's (already sign-extended) weight into that neuron's
//      packed element, leaving untouched elements unchanged,
//   3) writes the full word back.
//
// Single-word case (NUM_LANES <= EPW = 32/ELEM_BITS, group aligned so it stays
// within one word) — this is the full-connectivity path (outputs swept from 0,
// group index a multiple of NUM_LANES).  The convolutional path (kernel
// projections that can straddle a word boundary) is handled by the straddle
// variant; not in this module.
//
// 2 cycles/group: S_RD issues the read, S_WR computes the merged word
// combinationally from the just-returned read data and writes it.  done_o pulses
// when the write is accepted.  mem_wait_i honoured.
// =============================================================================
`timescale 1ns / 1ps

module syn_curr_accum_wide #(
    parameter ELEM_BITS    = 8,
    parameter NUM_LANES    = 2,
    parameter IDX_BITS     = 10,
    parameter ADDR_BITS    = 30,
    parameter IN_DATA_BITS = 32
)(
    input  wire                           clk,
    input  wire                           reset,

    // request: a group of NUM_LANES outputs starting at out_index_i
    input  wire         [ADDR_BITS-1:0]   base_addr_i,
    input  wire          [IDX_BITS-1:0]   out_index_i,   // g
    input  wire [NUM_LANES*ELEM_BITS-1:0] weights_i,     // sign-extended to ELEM_BITS
    input  wire        [NUM_LANES-1:0]    active_i,
    input  wire                           go_i,
    output wire                           busy_o,
    output wire                           done_o,        // 1-cycle: group committed

    // syn_curr memory (shared port, direct RMW)
    output wire                           syn_curr_mem_rd_o,
    output wire                           syn_curr_mem_wr_o,
    input  wire                           syn_curr_mem_wait_i,
    output wire         [ADDR_BITS-1:0]   syn_curr_mem_addr_o,
    input  wire      [IN_DATA_BITS-1:0]   syn_curr_mem_data_i,
    output wire      [IN_DATA_BITS-1:0]   syn_curr_mem_data_o
);
    localparam EPW     = IN_DATA_BITS / ELEM_BITS;
    localparam LOG_EPW = (EPW <= 1) ? 1 : $clog2(EPW);

    localparam S_RD = 1'b0, S_WR = 1'b1;
    reg                          state_r;
    reg          [IDX_BITS-1:0]  g_r;
    reg [NUM_LANES*ELEM_BITS-1:0] w_r;
    reg         [NUM_LANES-1:0]  act_r;

    always @(posedge clk) begin
        if (reset) begin
            state_r <= S_RD;
            g_r <= {IDX_BITS{1'b0}}; w_r <= {(NUM_LANES*ELEM_BITS){1'b0}};
            act_r <= {NUM_LANES{1'b0}};
        end else case (state_r)
            S_RD: if (go_i & ~syn_curr_mem_wait_i) begin
                      g_r   <= out_index_i;
                      w_r   <= weights_i;
                      act_r <= active_i;
                      state_r <= S_WR;
                  end
            S_WR: if (~syn_curr_mem_wait_i) state_r <= S_RD;
        endcase
    end

    wire [LOG_EPW-1:0] glow = g_r[LOG_EPW-1:0];

    // Merge: add active lanes' weights into the read word at their element columns
    reg [IN_DATA_BITS-1:0] new_word;
    integer lane;
    always @(*) begin
        new_word = syn_curr_mem_data_i;
        for (lane = 0; lane < NUM_LANES; lane = lane + 1)
            if (act_r[lane])
                new_word[(glow + lane)*ELEM_BITS +: ELEM_BITS] =
                    syn_curr_mem_data_i[(glow + lane)*ELEM_BITS +: ELEM_BITS]
                    + w_r[lane*ELEM_BITS +: ELEM_BITS];
    end

    assign syn_curr_mem_rd_o   = (state_r == S_RD) & go_i;
    assign syn_curr_mem_wr_o   = (state_r == S_WR);
    assign syn_curr_mem_addr_o = (state_r == S_WR)
        ? base_addr_i + {{(ADDR_BITS-(IDX_BITS-LOG_EPW)){1'b0}}, g_r[IDX_BITS-1:LOG_EPW]}
        : base_addr_i + {{(ADDR_BITS-(IDX_BITS-LOG_EPW)){1'b0}}, out_index_i[IDX_BITS-1:LOG_EPW]};
    assign syn_curr_mem_data_o = new_word;

    assign busy_o = (state_r == S_WR);
    assign done_o = (state_r == S_WR) & ~syn_curr_mem_wait_i;

endmodule
