// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
// =============================================================================
// tb_np_line_fetch — aggressive unit test for the wide read front-end.
// Authors: Simon Davidson & Claude
// Created: 2026-06-04
// Last modified: 2026-06-07
//
// Exercises np_fetch_sel (plain np_line_fetch when PF=0, prefetching
// np_line_fetch_pf when PF=1).  Both present element j of a contiguous
// NUM_LANES group at slices_o[j*EB +: EB], where the group's elements live
// LSB-first packed in 32-bit words (EPW = 32/EB per word, WPG = ceil(L/EPW)).
//
// SW slice model (the heart of the check): for a group whose first element is
// start_index g, element (g+j) lives in word (g+j)>>log2EPW at column
// ((g+j) mod EPW)*EB.  We hold a SW memory image identical to the DUT's, so the
// expected slice for lane j is mem_image[ (g+j)>>log2EPW ][ ((g+j)%EPW)*EB +: EB ].
//
// Methodology: drive a request, poll data_valid_o, then on the valid cycle check
// every lane slice against the SW model.  This is timing-agnostic (works for
// WPG=1 single-word, WPG=2 two-word groups, and the prefetch cache) — what
// matters is that whenever the fetcher SAYS the data is valid, the presented
// slices are correct for the requested group and base region.
//
// Coverage:
//   * random word streams (random memory image, random group indices)
//   * hit / miss: re-request the same word index -> immediate hit (no stall)
//   * cross-group reuse: two groups sharing one word (8b/L2) -> 2nd is a hit
//   * two-word groups (16b/L4, WPG=2)
//   * base-address (region) switch -> forces a refetch (no false hit)
//   * mem_wait back-pressure on the fetch path (random stalls)
//   * EB in {8,16}; LANES in {1,2,4}; PF in {0,1}
// =============================================================================
`timescale 1ns/1ps

`ifndef LANES
 `define LANES 2
`endif
`ifndef EB
 `define EB 8
`endif
`ifndef PF
 `define PF 0
`endif

module tb_np_line_fetch;

    localparam integer LANES   = `LANES;
    localparam integer EB      = `EB;
    localparam integer PF      = `PF;
    localparam integer ADDR_BITS = 30;
    localparam integer IDX_BITS  = 10;
    localparam integer IN_DATA_BITS = 32;
    localparam integer EPW     = IN_DATA_BITS / EB;
    localparam integer LOG_EPW = (EB==8) ? 2 : (EB==16) ? 1 : 0;  // log2(EPW)
    localparam integer NWORDS  = 64;     // image words used (well under 256)
    localparam integer NREQ    = 4000;

    reg                       clk, reset;
    reg  [ADDR_BITS-1:0]      base_addr_i;
    reg  [IDX_BITS-1:0]       start_index_i;
    reg                       req_i;
    wire                      data_valid_o;
    wire [LANES*EB-1:0]       slices_o;
    wire                      mem_rd_o;
    wire [ADDR_BITS-1:0]      mem_addr_o;
    reg  [IN_DATA_BITS-1:0]   mem_data_i;
    reg                       mem_wait_i;

    integer verif_errors, verif_checks;
    `include "../verif/checks.vh"

    np_fetch_sel #(.PREFETCH(PF), .IN_DATA_BITS(IN_DATA_BITS), .ELEM_BITS(EB),
                   .NUM_LANES(LANES), .IDX_BITS(IDX_BITS), .ADDR_BITS(ADDR_BITS))
    dut (
        .clk(clk), .reset(reset),
        .base_addr_i(base_addr_i), .start_index_i(start_index_i), .req_i(req_i),
        .data_valid_o(data_valid_o), .slices_o(slices_o),
        .mem_rd_o(mem_rd_o), .mem_addr_o(mem_addr_o),
        .mem_data_i(mem_data_i), .mem_wait_i(mem_wait_i));

    initial clk = 0; always #5 clk = ~clk;

    // -------- memory model: 1-cycle registered read, gated by ~mem_wait --------
    // Contract (per np_line_fetch comment): mem_wait stalls request ISSUE; an
    // accepted read returns its data the following cycle.
    reg [31:0] mem [0:255];
    always @(posedge clk)
        if (mem_rd_o & ~mem_wait_i) mem_data_i <= mem[mem_addr_o[7:0]];

    // -------- race-free back-pressure: mem_wait driven from a clocked process
    // gated by bp_en (matches the FlexMan template pattern; avoids the delta
    // race of blocking-assigning mem_wait inside the polling loop). --------
    reg     bp_en;
    integer bp_pct_r;
    always @(posedge clk)
        mem_wait_i <= bp_en ? ($urandom_range(99) < bp_pct_r) : 1'b0;

    integer i;
    task fill_mem;
        begin
            for (i = 0; i < 256; i = i + 1) mem[i] = {$random, $random} ^ (i*32'h9E3779B1);
        end
    endtask

    // -------- SW slice model: expected lane j of group g in region base --------
    function [EB-1:0] sw_slice;
        input [ADDR_BITS-1:0] baddr;
        input [IDX_BITS-1:0]  g;
        input integer         lane;
        reg [IDX_BITS:0]      elem;
        reg [7:0]             word;
        reg [31:0]            col;
        begin
            elem = g + lane;
            word = (baddr + (elem >> LOG_EPW)) & 8'hFF;
            col  = (elem % EPW) * EB;
            sw_slice = mem[word][col +: EB];
        end
    endfunction

    // check all lane slices at the current sample point against the SW model
    integer guard, lane;
    task check_slices;
        input [ADDR_BITS-1:0] baddr;
        input [IDX_BITS-1:0]  g;
        input [255:0]         tag;
        begin
            for (lane = 0; lane < LANES; lane = lane + 1)
                check_eq_u(slices_o[lane*EB +: EB], sw_slice(baddr, g, lane),
                           {tag, " lane"});
        end
    endtask

    // Issue a fresh request (req deasserted between calls), poll data_valid,
    // then check all lane slices.  bp_pct (>0) enables per-cycle mem_wait stalls
    // on the fetch path via the clocked bp driver; the fetcher must still
    // complete with the correct slices (latency-invariant under back-pressure).
    task fetch_and_check;
        input [ADDR_BITS-1:0] baddr;
        input [IDX_BITS-1:0]  g;
        input integer         bp_pct;
        input [255:0]         tag;
        begin
            // present a clean request after a fully-idle settle (req low, bp off)
            @(negedge clk); req_i = 1'b0; bp_en = 1'b0;
            @(negedge clk); @(negedge clk);     // let mem_wait settle low
            base_addr_i = baddr; start_index_i = g; req_i = 1'b1;
            bp_pct_r = bp_pct; bp_en = (bp_pct > 0);
            guard = 0;
            while (!data_valid_o && guard < 300) begin @(posedge clk); guard = guard + 1; end
            bp_en = 1'b0;
            #1;
            check_true(guard < 300, {tag, " valid within bound"});
            check_bit(data_valid_o, 1'b1, {tag, " data_valid"});
            check_slices(baddr, g, tag);
            @(negedge clk); req_i = 1'b0;
        end
    endtask

    // ASSERT the data is valid IMMEDIATELY (a hit) with req held high across an
    // index change — proves cross-group / same-group reuse with no refetch
    // bubble.  Caller guarantees the requested word is already resident.
    task expect_hit;
        input [ADDR_BITS-1:0] baddr;
        input [IDX_BITS-1:0]  g;
        input [255:0]         tag;
        begin
            @(negedge clk);
            base_addr_i = baddr; start_index_i = g; req_i = 1'b1;
            #1;
            check_bit(data_valid_o, 1'b1, {tag, " immediate hit"});
            check_slices(baddr, g, tag);
            @(posedge clk);
        end
    endtask

    integer rq, ridx, rbase;
    reg [IDX_BITS-1:0] g_align;

    task reset_dut;
        begin
            reset = 1; req_i = 0; base_addr_i = 0; start_index_i = 0; bp_en = 0;
            @(posedge clk); #1; @(posedge clk); #1; @(posedge clk); #1;
            reset = 0; @(posedge clk); #1;
        end
    endtask

    initial begin
        verif_errors = 0; verif_checks = 0; bp_en = 0; bp_pct_r = 0;
        fill_mem;
        reset_dut;
        $display("=== tb_np_line_fetch  EB=%0d LANES=%0d PF=%0d EPW=%0d WPG=%0d ===",
                 EB, LANES, PF, EPW, (LANES+EPW-1)/EPW);
        void'($urandom(32'hF0E0_0000 ^ (EB<<8) ^ (LANES<<4) ^ PF));

        // ---- directed: first group of region 0 (miss -> fill -> valid) ----
        // fetch g=0 and HOLD req high (leave word 0 resident) for the hit probes.
        @(negedge clk); base_addr_i = 30'd0; start_index_i = {IDX_BITS{1'b0}}; req_i = 1'b1;
        guard = 0;
        while (!data_valid_o && guard < 300) begin @(posedge clk); guard = guard + 1; end
        #1;
        check_true(guard < 300, "D0 first valid within bound");
        check_bit(data_valid_o, 1'b1, "D0 first data_valid");
        check_slices(30'd0, {IDX_BITS{1'b0}}, "D0 first");

        // ---- directed: cross-group reuse within one word (a second aligned
        //      group still living in the same word -> immediate hit) ----
        if (EPW >= 2*LANES)
            expect_hit(30'd0, LANES[IDX_BITS-1:0], "D1 cross-group hit");

        // ---- directed: re-request the exact same group -> immediate hit ----
        expect_hit(30'd0, {IDX_BITS{1'b0}}, "D2 same-group hit");

        // ---- directed: advance to the next word (miss -> refetch) ----
        fetch_and_check(30'd0, EPW[IDX_BITS-1:0], 0, "D3 next word");

        // ---- directed: base-address (region) switch must NOT false-hit ----
        // move to a different region; index 0 again — must refetch from new base
        fetch_and_check(30'd32, {IDX_BITS{1'b0}}, 0, "D4 region switch");

        // ---- random stream (no back-pressure) ----
        for (rq = 0; rq < NREQ; rq = rq + 1) begin
            rbase = $urandom_range(0,3) * 32;              // region base (word)
            ridx  = $urandom_range(0, (NWORDS-2)*EPW);      // any element start
            // align group start to a multiple of LANES (the parent always does)
            g_align = (ridx / LANES) * LANES;
            fetch_and_check(rbase[ADDR_BITS-1:0], g_align, 0, "rand");
        end

        // ---- random stream WITH mem_wait back-pressure on the fetch path ----
        for (rq = 0; rq < NREQ/2; rq = rq + 1) begin
            rbase = $urandom_range(0,3) * 32;
            ridx  = $urandom_range(0, (NWORDS-2)*EPW);
            g_align = (ridx / LANES) * LANES;
            fetch_and_check(rbase[ADDR_BITS-1:0], g_align, 35, "rand-bp");
        end

        req_i = 1'b0;
        `VERIF_EPILOGUE("tb_np_line_fetch")
    end

    `VERIF_WATCHDOG(60000000)

endmodule
