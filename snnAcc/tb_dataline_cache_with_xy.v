// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
// =============================================================================
// tb_dataline_cache_with_xy  (snnAcc)
//
// Authors      : Simon Davidson & Claude
// Created      : 2026-05-07
// Last modified: 2026-06-07
//
// Aggressive rewrite using the shared verif library.  dataline_cache_with_xy is
// a single-entry cache in front of a synchronous SRAM: on a miss it issues a
// memory fetch (1-cycle latency), asserts sys_wait_o while the fetch is in
// flight, and once the word is resident it extracts the addressed sub-word via
// slice_and_align.  base_addr_i changing invalidates the entry (new memory row).
//
// Address decomposition (LACTB = clog2(IN_DATA_BITS) = 5):
//   word_index   = sys_addr_i >> (5 - slice_sz_i)
//   mem_addr_o   = word_index + base_addr_i
//   slice_offset = sys_addr_i & (0x1F >> slice_sz_i)
//   slice_data_o = slice_and_align( sram[mem_addr], slice_sz, slice_offset )
//
// Software golden: read the addressed SRAM word and run the SAME slice-extract
// as slice_and_align (left-justified element).  Every requested beat is checked,
// across hit/miss, base-change invalidation, ALL six slice sizes, and random
// mem_wait back-pressure injected via `SRAM_RD_WAIT.
// =============================================================================
`timescale 1ns/1ps
`include "../shared/constants.v"

module tb_dataline_cache_with_xy;

    localparam IN_DATA_BITS      = 32;
    localparam IDX_ADDR_BITS     = 8;   // 256 addressable element slots
    localparam SLICE_DATA_IDX_SZ = 5;
    localparam SLICE_SIZE_SZ     = 3;
    localparam OUT_DATA_BITS     = 32;
    localparam X_INPUT_SZ        = 4;
    localparam Y_INPUT_SZ        = 4;

    integer verif_errors, verif_checks, verif_to, sram_i;
    `include "../verif/checks.vh"
    `include "../verif/sram_bfm.vh"

    reg                          clk, reset;
    reg  [SLICE_SIZE_SZ-1:0]     slice_sz;
    reg  [`ADDR_SIZE-1:0]        base_addr;
    reg  [IDX_ADDR_BITS-1:0]     sys_addr;
    reg                          sys_req;
    reg  [X_INPUT_SZ-1:0]        sys_index_x;
    reg  [Y_INPUT_SZ-1:0]        sys_index_y;
    reg                          sys_colour;
    reg                          sys_last;
    reg                          slice_data_taken;
    reg                          mem_wait;

    wire                         colour_select;
    wire                         sys_wait;
    wire                         slice_data_valid;
    wire [SLICE_DATA_IDX_SZ-1:0] slice_data_idx;
    wire [X_INPUT_SZ-1:0]        slice_data_index_x;
    wire [Y_INPUT_SZ-1:0]        slice_data_index_y;
    wire [OUT_DATA_BITS-1:0]     slice_data;
    wire                         slice_data_last;
    wire [`ADDR_SIZE-1:0]        mem_addr;
    wire                         mem_req;
    reg  [IN_DATA_BITS-1:0]      mem_data;

    dataline_cache_with_xy #(
        .IN_DATA_BITS(IN_DATA_BITS),
        .X_INPUT_SZ(X_INPUT_SZ),
        .Y_INPUT_SZ(Y_INPUT_SZ),
        .IDX_ADDR_BITS(IDX_ADDR_BITS),
        .SLICE_DATA_IDX_SZ(SLICE_DATA_IDX_SZ),
        .SLICE_SIZE_SZ(SLICE_SIZE_SZ),
        .OUT_DATA_BITS(OUT_DATA_BITS))
    dut (
        .clk(clk), .reset(reset),
        .colour_select_o(colour_select),
        .invalidate_i(1'b0),   // sub-module TB: no task boundaries
        .slice_sz_i(slice_sz),
        .base_addr_i(base_addr),
        .sys_addr_i(sys_addr),
        .sys_req_i(sys_req),
        .sys_index_x_i(sys_index_x),
        .sys_index_y_i(sys_index_y),
        .sys_colour_i(sys_colour),
        .sys_wait_o(sys_wait),
        .sys_last_i(sys_last),
        .slice_data_valid_o(slice_data_valid),
        .slice_data_idx_o(slice_data_idx),
        .slice_data_index_x_o(slice_data_index_x),
        .slice_data_index_y_o(slice_data_index_y),
        .slice_data_o(slice_data),
        .slice_data_last_o(slice_data_last),
        .slice_data_taken_i(slice_data_taken),
        .mem_addr_o(mem_addr),
        .mem_data_i(mem_data),
        .mem_req_o(mem_req),
        .mem_wait_i(mem_wait));

    // Synchronous SRAM with mem_wait back-pressure (read honoured only when
    // ~mem_wait), 1-cycle latency.  256 x 32-bit, address truncated to [7:0].
    `SRAM_RD_WAIT(sram, mem_req, mem_wait, mem_addr, mem_data)

    initial clk = 0; always #5 clk = ~clk;

    // ---- Software golden: slice extract of the addressed SRAM word --------
    function [31:0] g_slice;
        input [2:0]  sz;
        input [4:0]  idx;
        input [31:0] word;
        integer       w;
        reg    [63:0] wide;
        begin
            w       = 1 << sz;
            wide    = {32'h0, word} >> (idx * w);
            g_slice = wide[31:0] << (32 - w);
        end
    endfunction

    // Decompose sys_addr into word index and in-word slice offset.
    function [`ADDR_SIZE-1:0] g_word_idx;
        input [IDX_ADDR_BITS-1:0] addr;
        input [2:0]               sz;
        begin
            g_word_idx = addr >> (5 - sz);
        end
    endfunction
    function [4:0] g_slice_off;
        input [IDX_ADDR_BITS-1:0] addr;
        input [2:0]               sz;
        begin
            g_slice_off = addr & (8'h1F >> sz);
        end
    endfunction

    // ---- Issue a request and check the produced slice against the golden --
    // Holds sys_req until slice_data_valid, then pulses slice_data_taken.
    // mem_wait is driven externally (random); the cache must back-pressure the
    // SRAM transparently.  Depth/latency-agnostic.
    task request_and_check;
        input [IDX_ADDR_BITS-1:0] addr;
        input [2:0]               sz;
        input [`ADDR_SIZE-1:0]    base;
        input integer             miss_wait; // mem_wait cycles to inject on a miss
        input [255:0]             tag;
        reg   [31:0]              exp_word;
        reg   [4:0]               off;
        reg   [`ADDR_SIZE-1:0]    widx;
        integer                   mw;
        begin
            slice_sz  = sz;
            base_addr = base;
            sys_addr  = addr;
            widx      = g_word_idx(addr, sz);
            off       = g_slice_off(addr, sz);
            exp_word  = sram[(base + widx) & 8'hFF];

            // Inject memory back-pressure for the requested number of cycles
            // while the fetch is outstanding (mem_req asserted), then release.
            mem_wait         = (miss_wait > 0);
            sys_req          = 1'b1;
            slice_data_taken = 1'b0;
            mw = miss_wait;
            verif_to = 80;
            while (!slice_data_valid && verif_to > 0) begin
                @(posedge clk); #1; verif_to = verif_to - 1;
                if (mw > 0) begin
                    mw = mw - 1;
                    mem_wait = (mw > 0);
                end
            end
            mem_wait = 1'b0;
            if (verif_to == 0) begin
                $display("FAIL %0s: timeout waiting for slice_data_valid", tag);
                verif_errors = verif_errors + 1;
            end else begin
                check_eq_u(slice_data,     g_slice(sz, off, exp_word), tag);
                // slice_data_idx_o returns the WHOLE element index (= sys_addr_i,
                // truncated to SLICE_DATA_IDX_SZ) — the consumer needs the
                // input-neuron number, not the within-word offset.
                check_eq_u(slice_data_idx, addr[SLICE_DATA_IDX_SZ-1:0], "cache slice_data_idx_o");
            end
            // consume
            slice_data_taken = 1'b1;
            @(posedge clk); #1;
            slice_data_taken = 1'b0;
            sys_req          = 1'b0;
            @(posedge clk); #1;
        end
    endtask

    integer t, sz_l, a;
    reg [`ADDR_SIZE-1:0] base_l;

    initial begin
        verif_errors = 0; verif_checks = 0;
        reset = 1; sys_req = 0; slice_data_taken = 1; mem_wait = 0; mem_data = 0;
        slice_sz = 3'b101; base_addr = 0; sys_addr = 0;
        sys_index_x = 0; sys_index_y = 0; sys_colour = 0; sys_last = 0;

        // Init SRAM with distinct words.
        `SRAM_CLEAR(sram)
        for (sram_i = 0; sram_i < 256; sram_i = sram_i + 1)
            sram[sram_i] = {sram_i[7:0], ~sram_i[7:0], sram_i[7:0], 8'hA5} ^ 32'h1357_9BDF;

        @(posedge clk); #1; reset = 0; @(posedge clk); #1;
        $display("=== tb_dataline_cache_with_xy (snnAcc) ===");

        // ---- Test A: cold miss then hit (no back-pressure) ----------------
        mem_wait = 0;
        $display("A: cold miss + hit, 32-bit slices");
        request_and_check(8'd2, 3'b101, 30'd0, 0, "A miss addr2");
        request_and_check(8'd2, 3'b101, 30'd0, 0, "A hit  addr2");

        // ---- Test B: explicit mem_req on cold miss, no re-fetch on hit ----
        $display("B: mem_req on miss, none on hit");
        slice_sz = 3'b101; base_addr = 0; sys_addr = 8'd5;
        sys_req = 1; slice_data_taken = 0;
        #1;
        check_bit(mem_req, 1'b1, "B mem_req on cold miss");
        @(posedge clk); #1;
        verif_to = 20;
        while (!slice_data_valid && verif_to > 0) begin @(posedge clk); #1; verif_to = verif_to - 1; end
        // hold the request (cache valid) and re-sample: must be a hit, no mem_req
        @(posedge clk); #1;
        check_bit(mem_req, 1'b0, "B no mem_req on hit");
        check_bit(slice_data_valid, 1'b1, "B valid on hit");
        sys_req = 0; slice_data_taken = 1; @(posedge clk); #1;

        // ---- Test C: base_addr change invalidates -------------------------
        $display("C: base_addr change forces re-fetch");
        request_and_check(8'd3, 3'b101, 30'd0, 0, "C base0 addr3");
        // same element addr, different base -> different word -> re-fetch
        sys_addr = 8'd3; slice_sz = 3'b101; base_addr = 30'd16;
        sys_req = 1; slice_data_taken = 0; #1;
        check_bit(mem_req, 1'b1, "C mem_req on base change");
        sys_req = 0; slice_data_taken = 1; @(posedge clk); #1;
        request_and_check(8'd3, 3'b101, 30'd16, 0, "C base16 addr3");

        // ---- Test D: back-pressure (taken=0 on hit asserts sys_wait) ------
        $display("D: sys_wait on stalled hit");
        request_and_check(8'd7, 3'b101, 30'd0, 0, "D warm addr7");
        sys_addr = 8'd7; slice_sz = 3'b101; base_addr = 30'd0;
        sys_req = 1; slice_data_taken = 0;
        @(posedge clk); #1;
        check_bit(sys_wait, 1'b1, "D sys_wait when taken=0 on hit");
        slice_data_taken = 1; sys_req = 0; @(posedge clk); #1;

        // ---- Test E: every slice size, sweep indices, no wait -------------
        $display("E: all slice sizes, swept indices");
        mem_wait = 0;
        for (sz_l = 0; sz_l <= 5; sz_l = sz_l + 1) begin
            // sweep a range of element addresses spanning several words
            for (a = 0; a < 40; a = a + 1)
                request_and_check(a[IDX_ADDR_BITS-1:0], sz_l[2:0], 30'd32, 0,
                                  "E sweep slice");
        end

        // ---- Test F: random addr/size/base with random mem_wait ----------
        // The single-entry cache compares word indices using the CURRENT
        // slice_sz; changing slice_sz between two requests at the same base can
        // alias onto a stale entry (an out-of-spec sequence — in real use the
        // slice size is fixed for the whole task).  To keep the random stream
        // faithful to legal usage, the DUT is invalidated (reset pulse) before
        // each random transaction so every beat is a clean cold miss whose data
        // path (mem bus -> slice) is what we are stressing here; hit behaviour
        // is covered exhaustively by tests A-E above.
        $display("F: constrained-random with random mem_wait");
        void'($urandom(32'hCAC4_E001));
        for (t = 0; t < 1500; t = t + 1) begin
            reset = 1; @(posedge clk); #1; reset = 0; @(posedge clk); #1;
            sz_l   = $urandom_range(5);
            a      = $urandom_range(200);
            base_l = $urandom_range(0, 40);
            // 0..4 cycles of memory back-pressure on the fetch
            request_and_check(a[IDX_ADDR_BITS-1:0], sz_l[2:0], base_l,
                              ($urandom_range(99) < 50) ? $urandom_range(1,4) : 0,
                              "F rand");
        end

        // ---- Test G: random stream at a FIXED slice size (exercises hits) -
        // With slice_sz held constant, consecutive requests legitimately hit or
        // miss; the golden tracks both.  Sweeps each size in turn.
        $display("G: fixed-size random stream (hit + miss)");
        for (sz_l = 0; sz_l <= 5; sz_l = sz_l + 1) begin
            reset = 1; @(posedge clk); #1; reset = 0; @(posedge clk); #1;
            base_l = $urandom_range(0, 40);
            for (t = 0; t < 200; t = t + 1) begin
                a = $urandom_range(120);
                request_and_check(a[IDX_ADDR_BITS-1:0], sz_l[2:0], base_l,
                                  ($urandom_range(99) < 40) ? $urandom_range(1,3) : 0,
                                  "G fixed-size rand");
            end
        end

        `VERIF_EPILOGUE("tb_dataline_cache_with_xy")
    end

    `VERIF_WATCHDOG(8000000)

endmodule
