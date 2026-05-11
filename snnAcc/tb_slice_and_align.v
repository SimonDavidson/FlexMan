// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
`timescale 1ns/1ps
`include "../shared/constants.v"

// ================================================================
// tb_slice_and_align
//
// Exercises all (slice_size, slice_idx) combinations across three
// input words (mixed, all-ones, all-zeros).
//
// Reference model: element idx occupies bits [(idx+1)*bits-1 : idx*bits]
// of in_word_i (0-indexed from the LSB).  The DUT places that element
// left-justified (MSB-aligned) in out_word_o.  Equivalently:
//   element  = (in_word >> (idx * element_bits)) & elem_mask
//   expected = element << (32 - element_bits)
//
// Also checks last_slice_o, which fires on the final element
// within each 32-bit word for the given slice size.
// ================================================================
module tb_slice_and_align;

reg  [2:0] slice_size;
reg  [4:0] slice_idx;
reg [31:0] in_word;
wire [31:0] out_word;
wire        last_slice;

slice_and_align dut (
    .slice_size_i(slice_size),
    .slice_idx_i (slice_idx),
    .in_word_i   (in_word),
    .out_word_o  (out_word),
    .last_slice_o(last_slice)
);

// ----------------------------------------------------------------
// Reference model
// ----------------------------------------------------------------
function [31:0] ref_out;
    input [2:0]  sz;
    input [4:0]  idx;
    input [31:0] word;
    integer       bits;
    reg    [63:0] wide;
    begin
        bits    = 1 << sz;
        // Shift element N to bits [bits-1:0], zero-extending into 64-bit.
        wide    = {32'h0, word} >> (idx * bits);
        // Left-justify the element in the 32-bit output.
        ref_out = wide[31:0] << (32 - bits);
    end
endfunction

function ref_last;
    input [2:0] sz;
    input [4:0] idx;
    integer max_idx;
    begin
        max_idx = (32 >> sz) - 1;   // number of elements per word minus one
        ref_last = (idx == max_idx[4:0]) ? 1'b1 : 1'b0;
    end
endfunction

// ----------------------------------------------------------------
// Stimulus and checking
// ----------------------------------------------------------------
integer sz, idx_i, w, errors, tests, bits_per_elem, max_idx_i;
reg [31:0] test_words [0:2];

initial begin
    errors = 0;
    tests  = 0;

    test_words[0] = 32'hA5C3_8F1E;   // mixed bit pattern
    test_words[1] = 32'hFFFF_FFFF;   // all ones
    test_words[2] = 32'h0000_0000;   // all zeros

    $display("=== tb_slice_and_align ===");

    for (w = 0; w < 3; w = w + 1) begin
        in_word = test_words[w];
        for (sz = 0; sz <= 5; sz = sz + 1) begin
            bits_per_elem = 1 << sz;
            max_idx_i     = (32 / bits_per_elem) - 1;
            for (idx_i = 0; idx_i <= max_idx_i; idx_i = idx_i + 1) begin
                slice_size = sz[2:0];
                slice_idx  = idx_i[4:0];
                #1;  // combinational settle

                tests = tests + 1;
                if (out_word !== ref_out(sz[2:0], idx_i[4:0], in_word)) begin
                    $display("FAIL out   word=%08h sz=%0d idx=%0d : got %08h  exp %08h",
                             in_word, sz, idx_i, out_word,
                             ref_out(sz[2:0], idx_i[4:0], in_word));
                    errors = errors + 1;
                end

                tests = tests + 1;
                if (last_slice !== ref_last(sz[2:0], idx_i[4:0])) begin
                    $display("FAIL last  word=%08h sz=%0d idx=%0d : got %b  exp %b",
                             in_word, sz, idx_i, last_slice,
                             ref_last(sz[2:0], idx_i[4:0]));
                    errors = errors + 1;
                end
            end
        end
    end

    $display("=== %0d checks, %0d failures ===", tests, errors);
    if (errors == 0) $display("PASS"); else $display("FAIL");
    $finish;
end

endmodule
