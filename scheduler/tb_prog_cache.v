// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
`include "../shared/constants.v"

`timescale 1ns/1ps

// Testbench for prog_cache.v
//
// prog_cache is a direct-mapped cache for the scheduler's program memory.
// The cache line granularity is set by slice_sz_i [2:0]:
//   slice_sz_i=5 → 1-word cache lines (tag covers all index bits, no field)
//   slice_sz_i=0 → 32-word cache lines (5-bit field, upper bits are tag)
//
// Memory protocol: mem_req_o goes high on a miss; mem_data_i is sampled
//   the cycle after fill is acknowledged (mem_wait_i=0).
//
// Timing relative to a miss (slice_sz_i=5):
//   Cycle 0: sys_req_i=1, miss → mem_req_o=1, fill=1
//   Cycle 1: read_L=1; sys_data_o = forwarded mem_data_i; valid←1
//   Cycle 2: hit on same address; sys_data_o = cache_data
//
// Note: prog_cache is not currently instantiated inside scheduler.v.
//       It is provided as a standalone component for future use.

localparam PROG_BITS_LP  = `PROG_BITS;   // 10
localparam COL_BITS_LP   = `COL_BITS;    // 10
localparam ADDR_SIZE_LP  = `ADDR_SIZE;   // 30

module top;

parameter IN_DATA_SZ    = 32;
parameter SLICE_IDX_SZ  = 5;
parameter SLICE_SIZE_SZ = 3;

reg clk, reset;
initial clk = 1'b0;
always  #5 clk = ~clk;

initial begin
    $dumpfile("tb_prog_cache.vcd");
    $dumpvars(0, top);
end

// DUT ports
reg  [SLICE_SIZE_SZ-1:0]  slice_sz_i;
reg  [PROG_BITS_LP-1:0]   read_base_i;
reg  [COL_BITS_LP-1:0]    sys_index_i;
reg                        sys_req_i;
reg                        last_i;
reg  [IN_DATA_SZ-1:0]     mem_data_i;
reg                        mem_wait_i;

wire [IN_DATA_SZ-1:0]     sys_data_o;
wire [ADDR_SIZE_LP-1:0]   mem_addr_o;
wire                       mem_req_o;

// Simple combinatorial memory model (32 words)
reg [31:0] mem [0:31];

// Provide memory data: deliver in the cycle following the request
// (fill cycle).  No wait states by default.
always @(*) mem_data_i = mem[mem_addr_o[4:0]];

prog_cache #(
    .IN_DATA_SZ(IN_DATA_SZ),
    .SLICE_IDX_SZ(SLICE_IDX_SZ),
    .SLICE_SIZE_SZ(SLICE_SIZE_SZ)
) dut (
    .clk(clk),
    .reset(reset),
    .slice_sz_i(slice_sz_i),
    .read_base_i(read_base_i),
    .sys_index_i(sys_index_i),
    .sys_data_o(sys_data_o),
    .sys_req_i(sys_req_i),
    .last_i(last_i),
    .mem_addr_o(mem_addr_o),
    .mem_data_i(mem_data_i),
    .mem_req_o(mem_req_o),
    .mem_wait_i(mem_wait_i)
);

integer errors;
initial errors = 0;

task chk;
    input        got;
    input        exp;
    input [255:0] label;
    begin
        if (got !== exp) begin
            $display("FAIL [%0t] %0s: got=%0b exp=%0b", $time, label, got, exp);
            errors = errors + 1;
        end
    end
endtask

task chk32;
    input [31:0] got;
    input [31:0] exp;
    input [255:0] label;
    begin
        if (got !== exp) begin
            $display("FAIL [%0t] %0s: got=%0h exp=%0h", $time, label, got, exp);
            errors = errors + 1;
        end
    end
endtask

integer i;
initial begin
    // Initialise memory with easily recognisable values
    for (i = 0; i < 32; i = i + 1)
        mem[i] = 32'hA000_0000 | i;

    reset       = 1'b1;
    slice_sz_i  = 3'd5;   // 1-word cache lines
    read_base_i = 10'd0;
    sys_index_i = 10'd0;
    sys_req_i   = 1'b0;
    last_i      = 1'b0;
    mem_wait_i  = 1'b0;

    repeat(3) @(posedge clk); #1;
    reset = 1'b0;

    // ==================================================================
    // Group 1: single-word cache lines (slice_sz_i=5)
    //   tag = all 10 index bits; field = none
    //   mem_addr = sys_index + read_base
    // ==================================================================

    // ------------------------------------------------------------------
    // Test 1: miss on first request – mem_req_o should assert
    // ------------------------------------------------------------------
    @(posedge clk); #1;
    sys_req_i   = 1'b1;
    sys_index_i = 10'd7;
    @(negedge clk);
    chk(mem_req_o, 1'b1, "T1 miss: mem_req_o asserted");
    chk32(mem_addr_o, 30'd7, "T1 miss: mem_addr=7");

    // ------------------------------------------------------------------
    // Test 2: same index next cycle – hit (valid set, tag matches)
    //   sys_data_o forwards mem_data_i via read_L (no wait state)
    // ------------------------------------------------------------------
    @(posedge clk);
    // mem_wait_i=0 so fill happened in previous cycle; valid now set
    @(negedge clk);
    chk(mem_req_o,  1'b0,           "T2 hit: no mem request");
    chk32(sys_data_o, mem[7],       "T2 hit: data correct (forwarded)");

    // ------------------------------------------------------------------
    // Test 3: different index – miss again
    // ------------------------------------------------------------------
    @(posedge clk); #1;
    sys_index_i = 10'd12;
    @(negedge clk);
    chk(mem_req_o, 1'b1, "T3 new addr: miss");
    chk32(mem_addr_o, 30'd12, "T3 new addr: mem_addr=12");

    @(posedge clk);
    @(negedge clk);
    chk(mem_req_o,  1'b0,     "T3 next: hit");
    chk32(sys_data_o, mem[12], "T3 next: data correct");

    // ------------------------------------------------------------------
    // Test 4: same index again – still cached
    // ------------------------------------------------------------------
    @(posedge clk);
    @(negedge clk);
    chk(mem_req_o, 1'b0, "T4 same: still hit");

    // ------------------------------------------------------------------
    // Test 5: last_i=1 invalidates the cache line
    // ------------------------------------------------------------------
    @(posedge clk); #1;
    sys_index_i = 10'd3;
    @(posedge clk); #1;
    // After a miss (fill), re-request with last_i=1
    last_i = 1'b1;
    @(posedge clk); #1;
    last_i      = 1'b0;
    sys_index_i = 10'd3;   // same index
    @(negedge clk);
    // last_i cleared valid so next request on same address should miss
    chk(mem_req_o, 1'b1, "T5 after last_i: miss on previously valid line");

    sys_req_i = 1'b0;
    @(posedge clk);
    @(negedge clk);

    // ==================================================================
    // Group 2: 32-word cache lines (slice_sz_i=0)
    //   tag   = sys_index[9:5]   (upper 5 bits)
    //   field = sys_index[4:0]   (lower 5 bits)
    //   index (→ mem_addr) = sys_index >> 5 + read_base
    // ==================================================================
    slice_sz_i = 3'd0;

    // ------------------------------------------------------------------
    // Test 6: miss on index 0 (line tag = 0)
    // ------------------------------------------------------------------
    @(posedge clk); #1;
    sys_req_i   = 1'b1;
    sys_index_i = 10'd0;
    @(negedge clk);
    chk(mem_req_o, 1'b1,    "T6 line0: miss");
    chk32(mem_addr_o, 30'd0, "T6 line0: mem_addr=0");

    // ------------------------------------------------------------------
    // Test 7: nearby index within same cache line → hit
    //   index 5 → same line (tag = 5>>5 = 0), no memory request
    // ------------------------------------------------------------------
    @(posedge clk); #1;
    sys_index_i = 10'd5;
    @(posedge clk);   // fill from T6 has landed, valid=1
    @(negedge clk);
    chk(mem_req_o, 1'b0, "T7 same line: hit (no mem_req)");

    // ------------------------------------------------------------------
    // Test 8: different line (tag changes) → miss
    //   index 32 → tag = 32>>5 = 1
    // ------------------------------------------------------------------
    @(posedge clk); #1;
    sys_index_i = 10'd32;
    @(negedge clk);
    chk(mem_req_o,  1'b1,    "T8 new line: miss");
    chk32(mem_addr_o, 30'd1, "T8 new line: mem_addr=1 (index>>5)");

    sys_req_i = 1'b0;
    @(posedge clk);

    // ==================================================================
    // Group 3: memory wait state (mem_wait_i=1 stalls fill)
    // ==================================================================
    slice_sz_i = 3'd5;
    @(posedge clk); #1;
    sys_req_i   = 1'b1;
    sys_index_i = 10'd20;
    mem_wait_i  = 1'b1;
    @(negedge clk);
    chk(mem_req_o, 1'b1, "T9 wait: mem_req stays asserted");

    @(posedge clk);
    @(negedge clk);
    chk(mem_req_o, 1'b1, "T9 wait cycle 2: still waiting");

    // Release wait state
    @(posedge clk); #1;
    mem_wait_i = 1'b0;
    @(posedge clk);   // fill completes
    @(negedge clk);
    chk(mem_req_o, 1'b0, "T9 wait released: hit");

    sys_req_i = 1'b0;

    // ------------------------------------------------------------------
    @(posedge clk);
    if (errors == 0)
        $display("PASS – prog_cache: all tests passed.");
    else
        $display("FAIL – prog_cache: %0d error(s).", errors);
    $finish;
end

initial begin
    #20000;
    $display("TIMEOUT – prog_cache");
    $finish;
end

endmodule
