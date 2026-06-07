// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
// =============================================================================
// tb_packer  (snnAcc)
//
// Authors      : Simon Davidson & Claude
// Created      : 2026-06-07
// Last modified: 2026-06-07
//
// Aggressive unit test for packer.v.  The packer accumulates variable-width
// left-justified elements (pak_acc_data_i: element in the MSBs of a 32-bit
// word) into a 32-bit output word, emitting a RAM write when a whole word has
// filled (the "top" element of the word) OR when pak_last_i flushes a partial
// word.
//
// Software golden model (mirrors the RTL bit-for-bit):
//   shift_mask = 0x1F >> out_sz                          (low bits of index used)
//   data_mask  = 32'hFFFFFFFF << (32 - (1<<out_sz))      (top 2^out_sz bits)
//   masked     = index[4:0] & shift_mask
//   plus       = masked + 1
//   data_shift = (out_sz==5) ? 32 : (plus << out_sz)
//   new_data   = (({32'h0, (value & data_mask)} << data_shift) >> 32)[31:0]
//   buffer    |= new_data                                (OR-accumulate)
//   top_bit    = (masked == shift_mask)                  (word full)
//   offset     = latched_index >> (5 - out_sz)           (output word offset)
//   addr       = base + offset
// A write is emitted (pot_wr_o=1) on the cycle AFTER an input that sets
// buffer_full (= top_bit | pak_last); pot_data_o on that cycle is the
// accumulated buffer, pot_addr_o = base + (latched index >> (5-out_sz)) where
// the latched index is the index of the element that triggered the fill.
//
// The DUT is driven through its valid/full handshake: present pak_write_i with
// the element, then (once buffer_full) hold until the write is honoured
// (pot_wr_o & ~pot_wait_i).  The model is run in lockstep and every emitted
// (pot_data_o, pot_addr_o, pot_wr_o) beat is checked, across ALL out_sz codes,
// random value/index/last streams, partial-word flush via pak_last, and
// pot_wait_i back-pressure.
// =============================================================================
`timescale 1ns/1ps
`include "../shared/constants.v"

module tb_packer;

    integer verif_errors, verif_checks;
    `include "../verif/checks.vh"

    reg                       clk, reset;
    reg                       pak_write;
    reg                       pak_colour;
    reg                       pak_last;
    reg  [`PIN_BITS-1:0]      pak_index;
    reg  [`POT_BITS-1:0]      pak_acc_data;
    reg  [`POT_OUT_SZ_SZ-1:0] pak_out_sz;
    reg  [`ADDR_SIZE-1:0]     pak_out_base_addr;
    reg                       pot_wait;

    wire                  busy;
    wire                  finish;
    wire                  pak_full;
    wire                  pot_wr;
    wire [`ADDR_SIZE-1:0] pot_addr;
    wire [31:0]           pot_data;
    wire                  pak_colour_sel;
    wire                  pak_colour_bs;

    packer dut (
        .clk(clk), .reset(reset),
        .busy_o(busy), .finish_o(finish),
        .pak_write_i(pak_write), .pak_full_o(pak_full),
        .pak_colour_i(pak_colour), .pak_last_i(pak_last),
        .pak_index_i(pak_index), .pak_acc_data_i(pak_acc_data),
        .pot_wr_o(pot_wr), .pot_wait_i(pot_wait),
        .pot_addr_o(pot_addr), .pot_data_o(pot_data),
        .pak_colour_sel_o(pak_colour_sel),
        .pak_out_sz_i(pak_out_sz),
        .pak_colour_bs_o(pak_colour_bs),
        .pak_out_base_addr_i(pak_out_base_addr));

    initial clk = 0; always #5 clk = ~clk;

    // -------------------------------------------------------------------------
    // Software golden model of one element's contribution to the output buffer.
    // Returns the 32-bit new_data word that gets OR'd into the accumulator.
    // -------------------------------------------------------------------------
    function [31:0] g_new_data;
        input [4:0]  idx;
        input [31:0] value;     // left-justified element (MSBs)
        input [2:0]  out_sz;
        reg   [4:0]  shift_mask;
        reg   [31:0] data_mask;
        reg   [5:0]  masked;
        reg   [6:0]  plus;
        reg   [6:0]  data_shift;
        reg   [63:0] temp;
        integer      w;
        begin
            w          = 1 << out_sz;                   // element width in bits
            shift_mask = 5'h1F >> out_sz;
            data_mask  = (out_sz == 3'd5) ? 32'hFFFF_FFFF
                                          : (32'hFFFF_FFFF << (32 - w));
            masked     = {1'b0, idx} & {1'b0, shift_mask};
            plus       = masked + 7'h01;
            data_shift = (out_sz == 3'd5) ? 7'h20 : (plus << out_sz);
            temp       = ({32'h0, (value & data_mask)} << data_shift);
            g_new_data = temp[63:32];
        end
    endfunction

    function g_top_bit;
        input [4:0] idx;
        input [2:0] out_sz;
        reg   [4:0] shift_mask;
        begin
            shift_mask = 5'h1F >> out_sz;
            g_top_bit  = ((idx & shift_mask) == shift_mask);
        end
    endfunction

    function [`ADDR_SIZE-1:0] g_offset;
        input [`PIN_BITS-1:0] idx;
        input [2:0]           out_sz;
        begin
            g_offset = idx >> (5 - out_sz);
        end
    endfunction

    // -------------------------------------------------------------------------
    // Golden accumulator state, advanced in lockstep with the DUT.
    // -------------------------------------------------------------------------
    reg [31:0]            g_buffer;
    reg [`PIN_BITS-1:0]   g_latched_idx;
    reg                   g_last_latched;

    integer verif_to;

    // Present one element and, if it fills/flushes a word, wait for the write
    // to be honoured and check the emitted (data, addr, wr) against the model.
    //
    // Acts as a well-behaved producer: it must NOT assert pak_write while the
    // packer is full (pak_full_o), otherwise the RTL OR-accumulates a new field
    // into a buffer that is already pending writeback (corruption).  When a
    // fill is expected, `stall_cycles` injects pot_wait back-pressure for that
    // many cycles before honouring the write — exercising the held-write path.
    task feed_element;
        input [`PIN_BITS-1:0] idx;
        input [31:0]          value;
        input                 last_in;
        input [2:0]           out_sz;
        input [`ADDR_SIZE-1:0] base;
        input integer         stall_cycles;
        reg   [31:0]          exp_data;
        reg [`ADDR_SIZE-1:0]  exp_addr;
        reg                   fills;
        integer               sc;
        begin
            // Respect back-pressure: wait for the packer to drain any pending
            // word before presenting a new element.
            pot_wait = 1'b0;
            verif_to = 60;
            while (pak_full && verif_to > 0) begin
                @(posedge clk); #1; verif_to = verif_to - 1;
            end

            pak_out_sz        = out_sz;
            pak_out_base_addr = base;
            pak_index         = idx;
            pak_acc_data      = value;
            pak_last          = last_in;

            // Predict the post-write buffer and whether this element fills.
            fills    = g_top_bit(idx[4:0], out_sz) | last_in;
            exp_data = g_buffer | g_new_data(idx[4:0], value, out_sz);
            exp_addr = base + g_offset(idx, out_sz);

            // Drive the write strobe for exactly one accepted input cycle.
            pak_write = 1'b1;
            @(posedge clk); #1;
            pak_write = 1'b0;

            // The element is now latched; output_buffer/buffer_full updated.
            if (fills) begin
                // buffer_full asserts; pot_wr_o is high now (combinational on
                // buffer_full).  Inject `stall_cycles` of pot_wait first: the
                // write must be HELD and the data stable.
                pot_wait = (stall_cycles > 0);
                check_bit(pot_wr, 1'b1, "pack pot_wr_o asserted on fill");
                check_eq_u(pot_data, exp_data, "pack pot_data_o");
                check_eq_u(pot_addr, exp_addr, "pack pot_addr_o");
                for (sc = 0; sc < stall_cycles; sc = sc + 1) begin
                    @(posedge clk); #1;
                    check_bit (pot_wr,   1'b1,     "pack pot_wr_o held during wait");
                    check_eq_u(pot_data, exp_data, "pack pot_data_o stable during wait");
                end
                // Honour the write: drop wait, advance one edge -> buffer clears.
                pot_wait = 1'b0;
                @(posedge clk); #1;
                g_buffer = 32'h0;
            end else begin
                // No fill — buffer accumulates, no write this beat.
                g_buffer = exp_data;
                check_bit(pot_wr, 1'b0, "pack pot_wr_o low (no fill)");
            end
        end
    endtask

    // Pack a full ascending run of `count` elements of a given out_sz, with the
    // final one flagged pak_last, checking every emitted write.
    integer e, run_len, sz_loop;
    reg [31:0] rnd_val;
    reg [`ADDR_SIZE-1:0] base_addr;

    initial begin
        verif_errors = 0; verif_checks = 0;
        clk = 0; reset = 1;
        pak_write = 0; pak_colour = 0; pak_last = 0;
        pak_index = 0; pak_acc_data = 0; pak_out_sz = 0;
        pak_out_base_addr = 0; pot_wait = 0;
        g_buffer = 32'h0; g_latched_idx = 0; g_last_latched = 0;

        @(posedge clk); #1; reset = 0; @(posedge clk); #1;
        $display("=== tb_packer (snnAcc) ===");

        void'($urandom(32'h0AC0_C0DE));

        // ---- Directed: one full word per out_sz, ascending indices ----------
        // elements per word = 32 >> out_sz  (1b:32, 2b:16, 4b:8, 8b:4, 16b:2, 32b:1)
        for (sz_loop = 0; sz_loop <= 5; sz_loop = sz_loop + 1) begin
            run_len   = 32 >> sz_loop;
            base_addr = 30'd64;
            g_buffer  = 32'h0;
            for (e = 0; e < run_len; e = e + 1) begin
                rnd_val = $urandom();
                feed_element(e[`PIN_BITS-1:0], rnd_val,
                             (e == run_len-1) ? 1'b1 : 1'b0,
                             sz_loop[2:0], base_addr, 0);
            end
            // reset DUT between runs for a clean accumulator
            reset = 1; @(posedge clk); #1; reset = 0; @(posedge clk); #1;
            g_buffer = 32'h0;
        end

        // ---- Directed: partial-word flush via pak_last (no top_bit) ---------
        // 4-bit elements (8/word) but flush after only 3 elements.
        reset = 1; @(posedge clk); #1; reset = 0; @(posedge clk); #1;
        g_buffer = 32'h0;
        feed_element(10'd0, 32'hA000_0000, 1'b0, 3'd2, 30'd128, 0);
        feed_element(10'd1, 32'hB000_0000, 1'b0, 3'd2, 30'd128, 0);
        feed_element(10'd2, 32'hC000_0000, 1'b1, 3'd2, 30'd128, 0); // pak_last flush

        // ---- Directed: multi-word run that spans several output addresses ---
        // 8-bit elements (4/word), 10 elements -> 3 writes (full,full,partial).
        // Inject a 2-cycle stall on every honoured write to exercise the held
        // write path against the multi-address offset arithmetic.
        reset = 1; @(posedge clk); #1; reset = 0; @(posedge clk); #1;
        g_buffer = 32'h0;
        for (e = 0; e < 10; e = e + 1)
            feed_element(e[`PIN_BITS-1:0], $urandom(),
                         (e == 9) ? 1'b1 : 1'b0, 3'd3, 30'd0, 2);

        // ---- Back-pressure: random pot_wait during a run -------------------
        // Hold pot_wait asserted on a fill and verify the write is held and the
        // accumulator is not cleared until the wait drops.
        reset = 1; @(posedge clk); #1; reset = 0; @(posedge clk); #1;
        g_buffer = 32'h0;
        begin : bp_block
            reg [31:0] v0, v1;
            reg [31:0] exp0;
            v0 = 32'h9000_0000; v1 = 32'h6000_0000;
            // out_sz=4 (16-bit, 2/word): element 0 (no fill), element 1 (fill).
            pak_out_sz = 3'd4; pak_out_base_addr = 30'd0;
            // element 0
            pak_index = 10'd0; pak_acc_data = v0; pak_last = 0;
            pak_write = 1; @(posedge clk); #1; pak_write = 0;
            g_buffer = g_buffer | g_new_data(5'd0, v0, 3'd4);
            // element 1 fills the word; assert pot_wait to stall the write
            exp0 = g_buffer | g_new_data(5'd1, v1, 3'd4);
            pot_wait = 1;
            pak_index = 10'd1; pak_acc_data = v1; pak_last = 0;
            pak_write = 1; @(posedge clk); #1; pak_write = 0;
            // pot_wr asserts but is stalled: data must hold, no accumulator clear
            @(posedge clk); #1;
            check_bit (pot_wr,   1'b1, "BP pot_wr held during wait");
            check_eq_u(pot_data, exp0, "BP pot_data stable during wait");
            @(posedge clk); #1;
            check_eq_u(pot_data, exp0, "BP pot_data still stable (wait 2)");
            // release the wait; the write is honoured this cycle
            pot_wait = 0;
            @(posedge clk); #1;   // honoured -> buffer clears next edge
            check_bit(pot_wr, 1'b0, "BP pot_wr drops after honoured write");
            g_buffer = 32'h0;
        end

        // ---- pak_full_o: asserted when a word is full awaiting write --------
        reset = 1; @(posedge clk); #1; reset = 0; @(posedge clk); #1;
        g_buffer = 32'h0;
        begin : full_block
            // 32-bit element (1/word): a single write always fills immediately.
            pot_wait = 1;            // stall so pak_full can be observed
            pak_out_sz = 3'd5; pak_out_base_addr = 30'd0;
            pak_index = 10'd0; pak_acc_data = 32'h1234_5678; pak_last = 0;
            pak_write = 1; @(posedge clk); #1; pak_write = 0;
            @(posedge clk); #1;
            check_bit(pak_full, 1'b1, "FULL pak_full_o while stalled-full");
            pot_wait = 0;
            @(posedge clk); #1;
            check_bit(pak_full, 1'b0, "FULL pak_full_o clears after write");
            g_buffer = 32'h0;
        end

        // ---- Constrained-random: many runs, random out_sz/base/last/wait ----
        begin : rand_block
            integer t, n, len;
            reg [2:0] sz;
            reg [`ADDR_SIZE-1:0] b;
            for (t = 0; t < 400; t = t + 1) begin
                reset = 1; @(posedge clk); #1; reset = 0; @(posedge clk); #1;
                g_buffer = 32'h0;
                sz  = $urandom_range(5);
                b   = $urandom_range(192);
                len = $urandom_range(1, 2*(32 >> sz) + 3); // up to ~2 words
                for (n = 0; n < len; n = n + 1) begin
                    // sprinkle back-pressure: 0..3 stall cycles per honoured write
                    feed_element(n[`PIN_BITS-1:0], $urandom(),
                                 (n == len-1) ? 1'b1 : 1'b0, sz, b,
                                 ($urandom_range(99) < 35) ? $urandom_range(1,3) : 0);
                end
                pot_wait = 0;
            end
        end

        `VERIF_EPILOGUE("tb_packer")
    end

    `VERIF_WATCHDOG(5000000)

endmodule
