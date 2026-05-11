// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
`timescale 1ns/1ps

// Testbench for buffer_state_entry.v
// Covers: reset, new-target, buff-now-full, content-consumed (multi), mark-as-full
// TGT_COUNT_SZ is declared at file scope in buffer_state_entry.v and is visible
// to this file when both are compiled in the same Xcelium compilation unit.

module top;

reg        clk, reset;
reg        mark_as_full_i;
reg  [2:0] mark_buff_usage_i;
reg        buff_new_tgt_i;
reg  [2:0] buff_new_usage_count_i;
reg        buff_new_colour_i;
reg        buff_now_full_i;
reg        buff_content_consumed_i;

wire [2:0] buff_usage_count_o;
wire       buff_colour_o;
wire       buff_free_o;
wire       buff_full_o;

initial clk = 1'b0;
always  #5 clk = ~clk;

initial begin
    $dumpfile("tb_buffer_state_entry.vcd");
    $dumpvars(0, top);
end

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

task chk_count;
    input [2:0] got;
    input [2:0] exp;
    input [255:0] label;
    begin
        if (got !== exp) begin
            $display("FAIL [%0t] %0s: got=%0d exp=%0d", $time, label, got, exp);
            errors = errors + 1;
        end
    end
endtask

buffer_state_entry dut (
    .clk(clk),
    .reset(reset),
    .mark_as_full_i(mark_as_full_i),
    .mark_buff_usage_i(mark_buff_usage_i),
    .buff_new_tgt_i(buff_new_tgt_i),
    .buff_new_usage_count_i(buff_new_usage_count_i),
    .buff_new_colour_i(buff_new_colour_i),
    .buff_now_full_i(buff_now_full_i),
    .buff_content_consumed_i(buff_content_consumed_i),
    .buff_usage_count_o(buff_usage_count_o),
    .buff_colour_o(buff_colour_o),
    .buff_free_o(buff_free_o),
    .buff_full_o(buff_full_o)
);

initial begin
    reset                   = 1'b1;
    mark_as_full_i          = 1'b0;
    mark_buff_usage_i       = 3'd0;
    buff_new_tgt_i          = 1'b0;
    buff_new_usage_count_i  = 3'd0;
    buff_new_colour_i       = 1'b0;
    buff_now_full_i         = 1'b0;
    buff_content_consumed_i = 1'b0;

    repeat(3) @(posedge clk); #1;
    reset = 1'b0;
    @(negedge clk);

    // ------------------------------------------------------------------
    // Test 1: post-reset state
    // ------------------------------------------------------------------
    chk(buff_free_o,   1'b1, "reset: free=1");
    chk(buff_full_o,   1'b0, "reset: full=0");
    chk(buff_colour_o, 1'b0, "reset: colour=0");

    // ------------------------------------------------------------------
    // Test 2: new target allocation (task dispatched, this is tgt buffer)
    // ------------------------------------------------------------------
    @(posedge clk); #1;
    buff_new_tgt_i          = 1'b1;
    buff_new_usage_count_i  = 3'd2;
    buff_new_colour_i       = 1'b1;
    @(posedge clk); #1;
    buff_new_tgt_i          = 1'b0;
    @(negedge clk);
    chk(buff_free_o,          1'b0, "new_tgt: free→0");
    chk(buff_full_o,          1'b0, "new_tgt: full stays 0");
    chk(buff_colour_o,        1'b1, "new_tgt: colour latched");
    chk_count(buff_usage_count_o, 3'd2, "new_tgt: usage_count=2");

    // ------------------------------------------------------------------
    // Test 3: producing task completes → buffer becomes full
    // ------------------------------------------------------------------
    @(posedge clk); #1;
    buff_now_full_i = 1'b1;
    @(posedge clk); #1;
    buff_now_full_i = 1'b0;
    @(negedge clk);
    chk(buff_free_o, 1'b0, "now_full: free stays 0");
    chk(buff_full_o, 1'b1, "now_full: full→1");

    // ------------------------------------------------------------------
    // Test 4: first consumer takes from buffer (usage_count 2→1)
    // ------------------------------------------------------------------
    @(posedge clk); #1;
    buff_content_consumed_i = 1'b1;
    @(posedge clk); #1;
    buff_content_consumed_i = 1'b0;
    @(negedge clk);
    chk_count(buff_usage_count_o, 3'd1, "consume1: count→1");
    chk(buff_free_o, 1'b0, "consume1: still not free");
    chk(buff_full_o, 1'b1, "consume1: still full");

    // ------------------------------------------------------------------
    // Test 5: last consumer → buffer freed
    // ------------------------------------------------------------------
    @(posedge clk); #1;
    buff_content_consumed_i = 1'b1;
    @(posedge clk); #1;
    buff_content_consumed_i = 1'b0;
    @(negedge clk);
    chk(buff_free_o, 1'b1, "consume2: free→1");
    chk(buff_full_o, 1'b0, "consume2: full→0");

    // ------------------------------------------------------------------
    // Test 6: mark_as_full (external data pre-load, 3 consumers)
    // ------------------------------------------------------------------
    @(posedge clk); #1;
    mark_as_full_i    = 1'b1;
    mark_buff_usage_i = 3'd3;
    @(posedge clk); #1;
    mark_as_full_i = 1'b0;
    @(negedge clk);
    chk(buff_free_o,         1'b0, "mark_full: free→0");
    chk(buff_full_o,         1'b1, "mark_full: full→1");
    chk_count(buff_usage_count_o, 3'd3, "mark_full: count=3");

    // ------------------------------------------------------------------
    // Test 7: three consecutive consumers free the externally-loaded buffer
    // ------------------------------------------------------------------
    repeat(3) begin
        @(posedge clk); #1;
        buff_content_consumed_i = 1'b1;
        @(posedge clk); #1;
        buff_content_consumed_i = 1'b0;
    end
    @(negedge clk);
    chk(buff_free_o, 1'b1, "3x consume: free→1");
    chk(buff_full_o, 1'b0, "3x consume: full→0");

    // ------------------------------------------------------------------
    @(posedge clk);
    if (errors == 0)
        $display("PASS – buffer_state_entry: all tests passed.");
    else
        $display("FAIL – buffer_state_entry: %0d error(s).", errors);
    $finish;
end

initial begin
    #5000;
    $display("TIMEOUT – buffer_state_entry");
    $finish;
end

endmodule
