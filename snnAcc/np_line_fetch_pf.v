// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
// =============================================================================
// Module: np_line_fetch_pf
// Authors: Simon Davidson & Claude
// Created: 2026-06-04
// Last modified: 2026-06-04
//
// Prefetching variant of np_line_fetch for the WPG=1 case (NUM_LANES*ELEM_BITS
// <= 32, i.e. one word per lane-group: 8b L<=4, 16b L<=2).
//
// Thesis under test: the single-lane design wastes memory bandwidth, so the
// per-group fetch can be hidden under the 3-cycle compute by prefetching the
// NEXT word while the current one is being processed.  This keeps data_valid_o
// HIGH across a word advance, so the lanes never stall on a fetch and the
// throughput approaches the ideal L x (vs the fetch-bubble-limited plain
// np_line_fetch).
//
// Implementation: a 2-slot fully-associative word cache.  Each slot holds
// {valid, word-index, base-addr, data}.  The consumer reads whichever slot
// matches the current word (w0); meanwhile the fetch engine keeps both the
// current word (w0) and the next word (w0+1) resident, fetching whichever is
// missing into an evictable slot.  When the consumer advances to w0+1 it is
// already resident -> no bubble; the old word's slot becomes evictable and is
// reused to prefetch w0+2.  Per-slot base tracking handles base-address
// changes (region switches) without a global flush.  One outstanding read at a
// time (1 word / 2 cycles) — sufficient to stay ahead of a 3-cycle group.
// =============================================================================

`timescale 1ns / 1ps

module np_line_fetch_pf #(
    parameter IN_DATA_BITS = 32,
    parameter ELEM_BITS    = 8,
    parameter NUM_LANES    = 2,
    parameter IDX_BITS     = 10,
    parameter ADDR_BITS    = 30
)(
    input  wire                           clk,
    input  wire                           reset,
    input  wire        [ADDR_BITS-1:0]    base_addr_i,
    input  wire         [IDX_BITS-1:0]    start_index_i,
    input  wire                           req_i,
    output wire                           data_valid_o,
    output wire [NUM_LANES*ELEM_BITS-1:0] slices_o,
    output wire                           mem_rd_o,
    output wire        [ADDR_BITS-1:0]    mem_addr_o,
    input  wire     [IN_DATA_BITS-1:0]    mem_data_i,
    input  wire                           mem_wait_i
);
    localparam EPW     = IN_DATA_BITS / ELEM_BITS;
    localparam LOG_EPW = (EPW <= 1) ? 1 : $clog2(EPW);

    wire  [IDX_BITS-1:0] w0    = start_index_i >> LOG_EPW;
    wire  [IDX_BITS-1:0] w1    = w0 + 1'b1;
    wire [LOG_EPW-1:0]   g_low = start_index_i[LOG_EPW-1:0];

    // ---- two associative slots ----
    reg                 vA_r, vB_r;
    reg  [IDX_BITS-1:0] wordA_r, wordB_r;
    reg [ADDR_BITS-1:0] baseA_r, baseB_r;
    reg [IN_DATA_BITS-1:0] dA_r, dB_r;

    reg                 pend_r;       // a read is outstanding (data next cycle)
    reg                 pend_dest_r;  // 0 -> slot A, 1 -> slot B
    reg  [IDX_BITS-1:0] pend_word_r;
    reg [ADDR_BITS-1:0] pend_base_r;

    wire hitA = vA_r & (wordA_r == w0) & (baseA_r == base_addr_i);
    wire hitB = vB_r & (wordB_r == w0) & (baseB_r == base_addr_i);

    // presence of the current (w0) and next (w1) words across both slots
    wire pN0_A = vA_r & (wordA_r == w0) & (baseA_r == base_addr_i);
    wire pN0_B = vB_r & (wordB_r == w0) & (baseB_r == base_addr_i);
    wire pN1_A = vA_r & (wordA_r == w1) & (baseA_r == base_addr_i);
    wire pN1_B = vB_r & (wordB_r == w1) & (baseB_r == base_addr_i);
    wire presentN0 = pN0_A | pN0_B;
    wire presentN1 = pN1_A | pN1_B;

    wire inflightN0 = pend_r & (pend_word_r == w0) & (pend_base_r == base_addr_i);
    wire inflightN1 = pend_r & (pend_word_r == w1) & (pend_base_r == base_addr_i);

    // choose the word to fetch: current word first, then prefetch the next
    wire need0 = ~presentN0 & ~inflightN0;
    wire need1 = ~presentN1 & ~inflightN1;
    wire fetch_req  = req_i & (need0 | need1);
    wire [IDX_BITS-1:0] fetch_word = need0 ? w0 : w1;

    // a slot is evictable if invalid, wrong base, or holding neither w0 nor w1
    wire evictA = ~vA_r | (baseA_r != base_addr_i) |
                  ((wordA_r != w0) & (wordA_r != w1));
    wire dest = evictA ? 1'b0 : 1'b1;   // prefer A when evictable, else B

    wire issue = fetch_req & ~pend_r & ~mem_wait_i;

    assign mem_rd_o   = issue;
    assign mem_addr_o = base_addr_i + {{(ADDR_BITS-IDX_BITS){1'b0}}, fetch_word};

    always @(posedge clk) begin
        if (reset) begin
            vA_r <= 1'b0; vB_r <= 1'b0; pend_r <= 1'b0;
            wordA_r <= {IDX_BITS{1'b0}}; wordB_r <= {IDX_BITS{1'b0}};
            baseA_r <= {ADDR_BITS{1'b0}}; baseB_r <= {ADDR_BITS{1'b0}};
            dA_r <= {IN_DATA_BITS{1'b0}}; dB_r <= {IN_DATA_BITS{1'b0}};
            pend_dest_r <= 1'b0; pend_word_r <= {IDX_BITS{1'b0}};
            pend_base_r <= {ADDR_BITS{1'b0}};
        end else begin
            // capture returning read (data valid this cycle)
            if (pend_r) begin
                if (pend_dest_r == 1'b0) begin
                    dA_r <= mem_data_i; wordA_r <= pend_word_r;
                    baseA_r <= pend_base_r; vA_r <= 1'b1;
                end else begin
                    dB_r <= mem_data_i; wordB_r <= pend_word_r;
                    baseB_r <= pend_base_r; vB_r <= 1'b1;
                end
                pend_r <= 1'b0;
            end
            // issue a new read (only when none outstanding)
            if (issue) begin
                pend_r      <= 1'b1;
                pend_dest_r <= dest;
                pend_word_r <= fetch_word;
                pend_base_r <= base_addr_i;
            end
        end
    end

    // ---- combinational L-way slice tap from the matching slot ----
    wire [IN_DATA_BITS-1:0] word_sel = hitA ? dA_r : dB_r;
    genvar j;
    generate for (j = 0; j < NUM_LANES; j = j + 1) begin : g_tap
        assign slices_o[j*ELEM_BITS +: ELEM_BITS] =
            word_sel[(g_low + j)*ELEM_BITS +: ELEM_BITS];
    end endgenerate

    assign data_valid_o = req_i & (hitA | hitB);

endmodule
