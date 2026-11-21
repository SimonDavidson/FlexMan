// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
// =============================================================================
// Module: np_line_fetch
// Authors: Simon Davidson & Claude
// Created: 2026-06-04
// Last modified: 2026-06-04
//
// Wide read front-end for the lane-parallel neuron_processing experiment.
//
// Core idea (the "single-fetch, parallel-deserialise" approach): the on-chip
// memory is 32-bit and narrow operands are bit-packed LSB-first (element k at
// bits[k*ELEM_BITS +: ELEM_BITS], 4x8-bit or 2x16-bit per word).  Instead of
// deserialising one element per cycle (as dataline_cache_with_xy does), this
// block fetches the WORDS covering a contiguous group of NUM_LANES elements
// ONCE, holds them in a small line buffer, and presents all NUM_LANES element
// slices combinationally so NUM_LANES compute lanes can run in parallel.
//
//   EPW = 32 / ELEM_BITS            elements per 32-bit word (4 @8b, 2 @16b)
//   WPG = ceil(NUM_LANES / EPW)     words per lane-group (1, or 2 for 16b x4)
//
// A lane-group starts at neuron index g = start_index_i (a multiple of
// NUM_LANES, supplied by the parent).  The group's elements g..g+L-1 live in
// the WPG contiguous words starting at word index w0 = g >> log2(EPW).  When
// the buffered window already covers w0 the data is presented immediately
// (a "hit" — this is the cross-group reuse that makes 8b/L2 read one word per
// two groups); otherwise the WPG words are fetched from single-port memory
// (still one port — no banking), 1-cycle registered read latency each.
//
// At WPG=1 / NUM_LANES=1 this is behaviourally a single-word cache, matching
// the existing dataline_cache_with_xy fetch contract.
//
// Memory contract: registered read — assert mem_rd_o with mem_addr_o, data
// arrives on mem_data_i the following cycle (matches sram_model / sram_bram).
// mem_wait_i (if ever asserted) stalls request issue.
// =============================================================================

`timescale 1ns / 1ps

module np_line_fetch #(
    parameter IN_DATA_BITS = 32,   // memory word width
    parameter ELEM_BITS    = 8,    // packed element width (8 or 16)
    parameter NUM_LANES    = 2,    // L
    parameter IDX_BITS     = 10,   // neuron-index width
    parameter ADDR_BITS    = 30
)(
    input  wire                          clk,
    input  wire                          reset,

    // Control / system side
    input  wire        [ADDR_BITS-1:0]   base_addr_i,    // word base address
    input  wire         [IDX_BITS-1:0]   start_index_i,  // g = first neuron of group
    input  wire                          req_i,          // group requested
    output wire                          data_valid_o,   // all L slices valid
    output wire [NUM_LANES*ELEM_BITS-1:0] slices_o,      // lane j @ [j*EB +: EB]

    // Memory side (single-port, 1-cycle registered read)
    output wire                          mem_rd_o,
    output wire        [ADDR_BITS-1:0]   mem_addr_o,
    input  wire     [IN_DATA_BITS-1:0]   mem_data_i,
    input  wire                          mem_wait_i
);

    // ---- derived sizing ----------------------------------------------------
    localparam EPW     = IN_DATA_BITS / ELEM_BITS;          // elems / word
    localparam LOG_EPW = (EPW <= 1) ? 1 : $clog2(EPW);
    localparam WPG     = (NUM_LANES + EPW - 1) / EPW;       // words / group
    localparam BUF_W   = WPG * IN_DATA_BITS;
    localparam CNT_W   = (WPG <= 1) ? 1 : ($clog2(WPG) + 1);

    // word index of the group's first word
    wire [IDX_BITS-1:0] w0 = start_index_i >> LOG_EPW;
    // element offset of the group's first element within its word
    wire [LOG_EPW-1:0]  g_low = start_index_i[LOG_EPW-1:0];

    // ---- line buffer + fill state ------------------------------------------
    reg [IN_DATA_BITS-1:0] buf_r [0:WPG-1];
    reg     [IDX_BITS-1:0] win_base_r;       // word index currently buffered
    reg    [ADDR_BITS-1:0] win_base_addr_r;  // base_addr at fill time (for invalidation)
    reg                    buf_valid_r;
    reg                    filling_r;
    reg        [CNT_W-1:0] issue_cnt_r;  // words requested so far
    reg        [CNT_W-1:0] recv_cnt_r;   // words captured so far
    reg                    recv_pending_r; // a read was accepted last cycle

    // Hit needs matching word index AND matching base address: base_addr_i
    // changes when the consumer moves to a new memory region, mirroring
    // dataline_cache_with_xy's cache_base_addr_r invalidation.
    wire hit = buf_valid_r & (win_base_r == w0) & (win_base_addr_r == base_addr_i);

    integer ri;
    always @(posedge clk) begin
        if (reset) begin
            buf_valid_r    <= 1'b0;
            filling_r      <= 1'b0;
            issue_cnt_r    <= {CNT_W{1'b0}};
            recv_cnt_r     <= {CNT_W{1'b0}};
            recv_pending_r <= 1'b0;
            win_base_r     <= {IDX_BITS{1'b1}}; // unlikely to match index 0 at reset
            win_base_addr_r<= {ADDR_BITS{1'b0}};
            for (ri = 0; ri < WPG; ri = ri + 1) buf_r[ri] <= {IN_DATA_BITS{1'b0}};
        end else if (!filling_r) begin
            // Idle / holding: start a fill on a miss.
            if (req_i & ~hit) begin
                filling_r      <= 1'b1;
                win_base_r     <= w0;
                win_base_addr_r<= base_addr_i;
                buf_valid_r    <= 1'b0;
                issue_cnt_r    <= {CNT_W{1'b0}};
                recv_cnt_r     <= {CNT_W{1'b0}};
                recv_pending_r <= 1'b0;
            end
        end else begin
            // Filling: capture the word fetched last cycle, then issue the next.
            if (recv_pending_r) begin
                buf_r[recv_cnt_r] <= mem_data_i;
                recv_cnt_r        <= recv_cnt_r + 1'b1;
                if (recv_cnt_r + 1'b1 == WPG) begin
                    buf_valid_r <= 1'b1;
                    filling_r   <= 1'b0;
                end
            end
            if ((issue_cnt_r < WPG) & ~mem_wait_i) begin
                issue_cnt_r    <= issue_cnt_r + 1'b1;
                recv_pending_r <= 1'b1;
            end else begin
                recv_pending_r <= 1'b0;
            end
        end
    end

    assign mem_rd_o   = filling_r & (issue_cnt_r < WPG);
    assign mem_addr_o = base_addr_i
                      + {{(ADDR_BITS-IDX_BITS){1'b0}}, win_base_r}
                      + {{(ADDR_BITS-CNT_W){1'b0}}, issue_cnt_r};

    // ---- combinational L-way slice tap from the line buffer ----------------
    wire [BUF_W-1:0] buf_flat;
    genvar w;
    generate
        for (w = 0; w < WPG; w = w + 1) begin : g_flat
            assign buf_flat[w*IN_DATA_BITS +: IN_DATA_BITS] = buf_r[w];
        end
    endgenerate

    genvar j;
    generate
        for (j = 0; j < NUM_LANES; j = j + 1) begin : g_tap
            // local element index within the window = g_low + j (< WPG*EPW)
            assign slices_o[j*ELEM_BITS +: ELEM_BITS] =
                buf_flat[(g_low + j)*ELEM_BITS +: ELEM_BITS];
        end
    endgenerate

    assign data_valid_o = req_i & hit;

endmodule
