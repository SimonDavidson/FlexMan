// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
// =============================================================================
// tb_slice_and_align  (snnAcc)
//
// Authors      : Simon Davidson & Claude
// Created      : 2026-05-07
// Last modified: 2026-06-07
//
// Aggressive rewrite using the shared verif library.  slice_and_align is a pure
// combinational sub-word extractor: it pulls the element of width 2^slice_size
// at slice_idx out of a 32-bit word and LEFT-justifies it into the top
// OUT_DATA_BITS bits of out_word_o.  last_slice_o flags the final element of
// the 32-bit word for the given slice size.
//
// Software golden (OUT_DATA_BITS = 32):
//   w        = 1 << slice_size
//   elem     = (in_word >> (idx*w)) & ((1<<w)-1)
//   out_word = elem << (32 - w)
//   last     = (idx == (32/w) - 1)
//
// Strategy: sweep ALL 6 slice sizes at EVERY valid index across a directed set
// of corner words (mixed/all-ones/all-zeros/walking patterns) PLUS a large
// constrained-random word loop, checking out_word_o and last_slice_o each time.
// =============================================================================
`timescale 1ns/1ps
`include "../shared/constants.v"

module tb_slice_and_align;

    integer verif_errors, verif_checks;
    `include "../verif/checks.vh"

    reg  [2:0]  slice_size;
    reg  [4:0]  slice_idx;
    reg  [31:0] in_word;
    wire [31:0] out_word;
    wire        last_slice;

    slice_and_align #(
        .IN_DATA_BITS(32), .SLICE_IDX_BITS(5),
        .SLICE_SIZE_BITS(3), .OUT_DATA_SZ(3), .OUT_DATA_BITS(32))
    dut (
        .slice_size_i(slice_size),
        .slice_idx_i (slice_idx),
        .in_word_i   (in_word),
        .out_word_o  (out_word),
        .last_slice_o(last_slice));

    // ---- Software golden model -------------------------------------------
    function [31:0] g_out;
        input [2:0]  sz;
        input [4:0]  idx;
        input [31:0] word;
        integer       w;
        reg    [63:0] wide;
        begin
            w     = 1 << sz;
            wide  = {32'h0, word} >> (idx * w);   // element to bits [w-1:0]
            g_out = wide[31:0] << (32 - w);       // left-justify in 32-bit out
        end
    endfunction

    function g_last;
        input [2:0] sz;
        input [4:0] idx;
        integer max_idx;
        begin
            max_idx = (32 >> sz) - 1;
            g_last  = (idx == max_idx[4:0]);
        end
    endfunction

    // ---- Drive one (size, idx, word) vector and check both outputs --------
    task check_vec;
        input [2:0]  sz;
        input [4:0]  idx;
        input [31:0] word;
        begin
            slice_size = sz;
            slice_idx  = idx;
            in_word    = word;
            #1;   // combinational settle
            check_eq_u(out_word,   g_out (sz, idx, word), "slice out_word_o");
            check_bit (last_slice, g_last(sz, idx),       "slice last_slice_o");
        end
    endtask

    // Sweep every valid index for a given size against one word.
    task sweep_word;
        input [31:0] word;
        integer sz, idx, w, maxidx;
        begin
            for (sz = 0; sz <= 5; sz = sz + 1) begin
                w      = 1 << sz;
                maxidx = (32 / w) - 1;
                for (idx = 0; idx <= maxidx; idx = idx + 1)
                    check_vec(sz[2:0], idx[4:0], word);
            end
        end
    endtask

    integer t;

    initial begin
        verif_errors = 0; verif_checks = 0;
        slice_size = 0; slice_idx = 0; in_word = 0;
        $display("=== tb_slice_and_align (snnAcc) ===");

        // ---- Directed corner words ----------------------------------------
        sweep_word(32'hA5C3_8F1E);   // mixed
        sweep_word(32'hFFFF_FFFF);   // all ones
        sweep_word(32'h0000_0000);   // all zeros
        sweep_word(32'h8000_0001);   // endpoints set
        sweep_word(32'h0123_4567);   // ascending nibbles
        sweep_word(32'hFEDC_BA98);   // descending nibbles
        sweep_word(32'hAAAA_AAAA);   // alternating
        sweep_word(32'h5555_5555);

        // ---- Constrained-random word sweep --------------------------------
        void'($urandom(32'h5117_A11E));
        for (t = 0; t < 500; t = t + 1)
            sweep_word($urandom());

        `VERIF_EPILOGUE("tb_slice_and_align")
    end

    `VERIF_WATCHDOG(2000000)

endmodule
