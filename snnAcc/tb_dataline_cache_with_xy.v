// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
`timescale 1ns/1ps
`include "../shared/constants.v"

// ================================================================
// tb_dataline_cache_with_xy
//
// Uses 32-bit slice elements (slice_sz = 3'b101) so each element
// address maps 1-to-1 with a 32-bit memory word, keeping the
// address arithmetic simple.
//
// SRAM model: 32 words × 32 bits, 1-cycle read latency, no wait.
//
// Tests
// -----
//   1. Cold miss: first request fetches from memory (mem_req_o=1).
//   2. Back-to-back hit: repeat request does NOT re-issue mem_req_o.
//   3. Back-pressure: slice_data_taken_i=0 asserts sys_wait_o
//      on a cache hit.
//   4. base_addr change: same element address but different base
//      causes a fresh fetch (cache invalidation on base mismatch).
//
// Timing note
// -----------
// mem_req_o is combinational: it fires while sys_req_i is asserted and
// the cache misses, but drops to 0 the instant after the posedge clocks
// mem_new_data_valid_r to 1.  Therefore mem_req_o must be sampled BEFORE
// the posedge (with a short #1 settle), not after.
//
// On a cold miss data is valid after ONE clock cycle:
//   Cycle T  : request → miss → mem_req_o=1 (combinational, sampled here)
//   Cycle T+1: mem_new_data_valid_r=1 → slice_data_valid=1, data on output.
//              sys_wait_o=1 tells the UPSTREAM not to change the address yet
//              (the cache needs one more cycle to latch the data into its
//              register), but the DOWNSTREAM consumer can accept at T+1.
//   Cycle T+2: cache_valid_r=1, cache hit, sys_wait_o=0 → upstream can advance.
// wait_for_data blocks until slice_data_valid=1, matching actual consumer
// behaviour (downstream does not gate on sys_wait).
// ================================================================
module tb_dataline_cache_with_xy;

localparam IN_DATA_BITS      = 32;
localparam IDX_ADDR_BITS     = 5;   // 32 addressable words
localparam SLICE_DATA_IDX_SZ = 5;
localparam SLICE_SIZE_SZ     = 3;
localparam OUT_DATA_BITS     = 32;
localparam X_INPUT_SZ        = 4;
localparam Y_INPUT_SZ        = 4;
localparam SRAM_DEPTH        = 32;

reg                          clk, reset;
reg  [SLICE_SIZE_SZ-1:0]    slice_sz;
reg  [`ADDR_SIZE-1:0]       base_addr;
reg  [IDX_ADDR_BITS-1:0]   sys_addr;
reg                         sys_req;
reg  [X_INPUT_SZ-1:0]      sys_index_x;
reg  [Y_INPUT_SZ-1:0]      sys_index_y;
reg                         sys_colour;
reg                         sys_last;
reg                         slice_data_taken;
reg                         mem_wait;

wire                        colour_select;
wire                        sys_wait;
wire                        slice_data_valid;
wire [SLICE_DATA_IDX_SZ-1:0] slice_data_idx;
wire [X_INPUT_SZ-1:0]      slice_data_index_x;
wire [Y_INPUT_SZ-1:0]      slice_data_index_y;
wire [OUT_DATA_BITS-1:0]   slice_data;
wire                        slice_data_last;
wire [`ADDR_SIZE-1:0]      mem_addr;
wire                        mem_req;
reg  [IN_DATA_BITS-1:0]    mem_data;

dataline_cache_with_xy #(
    .IN_DATA_BITS(IN_DATA_BITS),
    .X_INPUT_SZ(X_INPUT_SZ),
    .Y_INPUT_SZ(Y_INPUT_SZ),
    .IDX_ADDR_BITS(IDX_ADDR_BITS),
    .SLICE_DATA_IDX_SZ(SLICE_DATA_IDX_SZ),
    .SLICE_SIZE_SZ(SLICE_SIZE_SZ),
    .OUT_DATA_BITS(OUT_DATA_BITS))
dut (
    .clk(clk),              .reset(reset),
    .colour_select_o(colour_select),
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
    .mem_wait_i(mem_wait)
);

// ----------------------------------------------------------------
// Synchronous SRAM model: 1-cycle read latency, no wait states
// ----------------------------------------------------------------
reg [31:0] sram [0:SRAM_DEPTH-1];
integer mi;
initial begin
    for (mi = 0; mi < SRAM_DEPTH; mi = mi + 1)
        sram[mi] = 32'hC0DE_0000 | mi[7:0];
end

always @(posedge clk)
    if (mem_req & ~mem_wait)
        mem_data <= sram[mem_addr[4:0]];

// ----------------------------------------------------------------
// Clock
// ----------------------------------------------------------------
initial clk = 0;
always  #5 clk = ~clk;

// ----------------------------------------------------------------
// Helpers
// ----------------------------------------------------------------
integer errors;
integer timeout;

task wait_for_data;
    begin
        timeout = 20;
        @(posedge clk); #1;
        while (!slice_data_valid && timeout > 0) begin
            timeout = timeout - 1;
            @(posedge clk); #1;
        end
        if (timeout == 0) begin
            $display("FAIL: timed out waiting for slice_data_valid");
            errors = errors + 1;
        end
    end
endtask

task check_eq;
    input [31:0] got;
    input [31:0] exp;
    input [255:0] label;
    begin
        if (got !== exp) begin
            $display("FAIL %s: got %08h  exp %08h", label, got, exp);
            errors = errors + 1;
        end
    end
endtask

task check_bit;
    input got;
    input exp;
    input [255:0] label;
    begin
        if (got !== exp) begin
            $display("FAIL %s: got %b  exp %b", label, got, exp);
            errors = errors + 1;
        end
    end
endtask

// ----------------------------------------------------------------
// Test sequence
// ----------------------------------------------------------------
initial begin
    errors          = 0;
    reset           = 1;
    sys_req         = 0;
    slice_data_taken = 1;
    mem_wait        = 0;
    mem_data        = 0;
    slice_sz        = 3'b101;   // 32-bit elements — 1 element per word
    base_addr       = 0;
    sys_addr        = 0;
    sys_index_x     = 0;
    sys_index_y     = 0;
    sys_colour      = 0;
    sys_last        = 0;

    @(posedge clk); #1;
    reset = 0;
    @(posedge clk); #1;   // allow reset to propagate

    // ----------------------------------------------------------
    // Test 1: Cold miss at address 2, base=0
    // mem_req_o is combinational: sample it before the posedge
    // that registers mem_new_data_valid_r.
    // ----------------------------------------------------------
    $display("Test 1: cold miss");
    sys_req  = 1;
    sys_addr = 5'd2;
    #1;   // combinational settle — do NOT clock yet
    check_bit(mem_req, 1'b1, "T1 mem_req_o on cold miss");
    @(posedge clk); #1;   // clock the request; SRAM and mem_new_data_valid_r update

    wait_for_data;
    check_eq(slice_data, sram[2], "T1 slice_data after miss");

    // ----------------------------------------------------------
    // Test 2: Repeat same request — should hit (no mem_req_o)
    // Keep sys_req=1 to hold the cache valid.
    // ----------------------------------------------------------
    $display("Test 2: cache hit (same addr+base)");
    sys_req  = 1;
    sys_addr = 5'd2;
    @(posedge clk); #1;
    check_bit(mem_req, 1'b0, "T2 mem_req_o must be 0 on hit");
    check_bit(slice_data_valid, 1'b1, "T2 slice_data_valid on hit");
    check_eq(slice_data, sram[2], "T2 hit data value");
    sys_req = 0;
    @(posedge clk); #1;

    // ----------------------------------------------------------
    // Test 3: Back-pressure — hold taken=0 during a hit and
    // verify sys_wait_o is asserted.
    // ----------------------------------------------------------
    $display("Test 3: back-pressure (taken=0 on hit)");
    // Re-request addr=2 to warm cache
    sys_req  = 1;
    sys_addr = 5'd2;
    wait_for_data;   // warm the cache

    slice_data_taken = 0;
    // One more cycle with the same hit — upstream should stall
    @(posedge clk); #1;
    check_bit(sys_wait, 1'b1, "T3 sys_wait_o when taken=0 on hit");
    slice_data_taken = 1;
    sys_req = 0;
    @(posedge clk); #1;

    // ----------------------------------------------------------
    // Test 4: base_addr change invalidates cache
    // Same element index, different base → must re-fetch.
    // mem_req_o is combinational: sample before the posedge.
    // ----------------------------------------------------------
    $display("Test 4: base_addr change → cache invalidation");
    sys_req   = 1;
    sys_addr  = 5'd2;
    base_addr = 30'd8;   // different base address
    #1;   // combinational settle
    check_bit(mem_req, 1'b1, "T4 mem_req_o must fire on base_addr change");
    @(posedge clk); #1;   // clock the request
    wait_for_data;
    // Data read from address (2 + 8) = 10, wrapped to sram[10]
    check_eq(slice_data, sram[10], "T4 data after re-fetch with new base");
    sys_req   = 0;
    base_addr = 0;
    @(posedge clk); #1;

    $display("=== tb_dataline_cache_with_xy: %0d failure(s) ===", errors);
    if (errors == 0) $display("PASS"); else $display("FAIL");
    $finish;
end

endmodule
