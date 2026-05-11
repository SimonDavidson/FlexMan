// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
`timescale 1ns/1ps

// Testbench for acc_hw_buffer_tracker.v
// Covers: reset, new-task (busy + buffer capture), task-finished, back-to-back tasks
// BUFF_IDX_SZ is declared at file scope in acc_hw_buffer_tracker.v and is visible
// to this file when both are compiled in the same Xcelium compilation unit.

module top;

reg        clk, reset;
reg        new_task_i;
reg  [3:0] tgt_buff_i, src1_buff_i, src2_buff_i, src3_buff_i;
reg        task_finished_i;

wire       acc_free_o;
wire [3:0] tgt_buff_o, src1_buff_o, src2_buff_o, src3_buff_o;

initial clk = 1'b0;
always  #5 clk = ~clk;

initial begin
    $dumpfile("tb_acc_hw_buffer_tracker.vcd");
    $dumpvars(0, top);
end

integer errors;
initial errors = 0;

task chk_bit;
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

task chk_idx;
    input [3:0] got;
    input [3:0] exp;
    input [255:0] label;
    begin
        if (got !== exp) begin
            $display("FAIL [%0t] %0s: got=%0d exp=%0d", $time, label, got, exp);
            errors = errors + 1;
        end
    end
endtask

acc_hw_buffer_tracker dut (
    .clk(clk),
    .reset(reset),
    .new_task_i(new_task_i),
    .tgt_buff_i(tgt_buff_i),
    .src1_buff_i(src1_buff_i),
    .src2_buff_i(src2_buff_i),
    .src3_buff_i(src3_buff_i),
    .task_finished_i(task_finished_i),
    .acc_free_o(acc_free_o),
    .tgt_buff_o(tgt_buff_o),
    .src1_buff_o(src1_buff_o),
    .src2_buff_o(src2_buff_o),
    .src3_buff_o(src3_buff_o)
);

initial begin
    reset           = 1'b1;
    new_task_i      = 1'b0;
    task_finished_i = 1'b0;
    tgt_buff_i      = 4'd0;
    src1_buff_i     = 4'd0;
    src2_buff_i     = 4'd0;
    src3_buff_i     = 4'd0;

    repeat(3) @(posedge clk); #1;
    reset = 1'b0;
    @(negedge clk);

    // ------------------------------------------------------------------
    // Test 1: after reset accelerator is free
    // ------------------------------------------------------------------
    chk_bit(acc_free_o, 1'b1, "reset: acc_free");

    // ------------------------------------------------------------------
    // Test 2: new_task clears acc_free and captures buffer IDs
    // ------------------------------------------------------------------
    @(posedge clk); #1;
    new_task_i  = 1'b1;
    tgt_buff_i  = 4'd7;
    src1_buff_i = 4'd3;
    src2_buff_i = 4'd5;
    src3_buff_i = 4'd0;
    @(posedge clk); #1;
    new_task_i  = 1'b0;
    @(negedge clk);
    chk_bit(acc_free_o,  1'b0, "new_task: acc_free→0");
    chk_idx(tgt_buff_o,  4'd7, "new_task: tgt captured");
    chk_idx(src1_buff_o, 4'd3, "new_task: src1 captured");
    chk_idx(src2_buff_o, 4'd5, "new_task: src2 captured");
    chk_idx(src3_buff_o, 4'd0, "new_task: src3 captured");

    // ------------------------------------------------------------------
    // Test 3: acc stays busy while task is running
    // ------------------------------------------------------------------
    repeat(4) @(posedge clk);
    @(negedge clk);
    chk_bit(acc_free_o, 1'b0, "mid-task: still busy");

    // ------------------------------------------------------------------
    // Test 4: task_finished restores acc_free
    // ------------------------------------------------------------------
    @(posedge clk); #1;
    task_finished_i = 1'b1;
    @(posedge clk); #1;
    task_finished_i = 1'b0;
    @(negedge clk);
    chk_bit(acc_free_o, 1'b1, "task_finished: acc_free→1");

    // ------------------------------------------------------------------
    // Test 5: second task immediately after finish
    // ------------------------------------------------------------------
    @(posedge clk); #1;
    new_task_i  = 1'b1;
    tgt_buff_i  = 4'd12;
    src1_buff_i = 4'd1;
    src2_buff_i = 4'd2;
    src3_buff_i = 4'd4;
    @(posedge clk); #1;
    new_task_i  = 1'b0;
    @(negedge clk);
    chk_bit(acc_free_o,  1'b0,  "task2: busy");
    chk_idx(tgt_buff_o,  4'd12, "task2: tgt");
    chk_idx(src1_buff_o, 4'd1,  "task2: src1");
    chk_idx(src2_buff_o, 4'd2,  "task2: src2");
    chk_idx(src3_buff_o, 4'd4,  "task2: src3");

    @(posedge clk); #1;
    task_finished_i = 1'b1;
    @(posedge clk); #1;
    task_finished_i = 1'b0;
    @(negedge clk);
    chk_bit(acc_free_o, 1'b1, "task2 done: free");

    // ------------------------------------------------------------------
    @(posedge clk);
    if (errors == 0)
        $display("PASS – acc_hw_buffer_tracker: all tests passed.");
    else
        $display("FAIL – acc_hw_buffer_tracker: %0d error(s).", errors);
    $finish;
end

initial begin
    #5000;
    $display("TIMEOUT – acc_hw_buffer_tracker");
    $finish;
end

endmodule
