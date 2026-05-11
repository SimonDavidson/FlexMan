// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
`timescale 1ns/1ps

// Testbench for sch_buffer_state.v
//
// Exercises the complete dispatch-and-complete lifecycle through the combined
// buffer-state and accelerator-tracker hierarchy.
//
// Buffer state change timing (cycles after asserting acc_finished_i):
//   +1 posedge: acc_process_pending_r latches the finished flag
//   +2 posedge: cleanup combinatorial signals drive buffer_state_entry regs
//               → buffers_full_o / buffers_free_o / acc_available_o update

module top;

localparam NUM_BUFFERS         = 16;
localparam NUM_HW_ACCELERATORS = 2;
localparam BUFF_INDX_SZ        = 4;   // $clog2(16)
localparam TGT_ACC_SZ          = 1;   // $clog2(2)
localparam TGT_COUNT_SZ        = 3;

reg clk, reset;
initial clk = 1'b0;
always  #5 clk = ~clk;

initial begin
    $dumpfile("tb_sch_buffer_state.vcd");
    $dumpvars(0, top);
end

// DUT ports
reg [NUM_HW_ACCELERATORS-1:0] acc_busy_i;
reg [NUM_HW_ACCELERATORS-1:0] acc_finished_i;
reg [NUM_HW_ACCELERATORS-1:0] acc_result_i;

reg                    mark_buff_as_full_i;
reg [BUFF_INDX_SZ-1:0] full_buff_id_i;
reg              [2:0] full_buff_usage_i;
reg                    start_new_task_i;
reg  [TGT_ACC_SZ-1:0] tgt_acc_id_i;
reg [BUFF_INDX_SZ-1:0] tgt_buff_idx_i;
reg [TGT_COUNT_SZ-1:0] tgt_usage_count_i;
reg                    tgt_colour_i;
reg [BUFF_INDX_SZ-1:0] src1_buff_idx_i;
reg [BUFF_INDX_SZ-1:0] src2_buff_idx_i;
reg [BUFF_INDX_SZ-1:0] src3_buff_idx_i;

wire [NUM_HW_ACCELERATORS-1:0] acc_available_o;
wire [NUM_BUFFERS-1:0]         buffers_full_o;
wire [NUM_BUFFERS-1:0]         buffers_free_o;
wire [NUM_BUFFERS-1:0]         buffers_colour_o;
wire [NUM_BUFFERS-1:0]         target_status_o;

sch_buffer_state #(
    .NUM_BUFFERS(NUM_BUFFERS),
    .NUM_HW_ACCELERATORS(NUM_HW_ACCELERATORS),
    .TGT_ACC_SZ(TGT_ACC_SZ),
    .TGT_COUNT_SZ(TGT_COUNT_SZ)
) dut (
    .clk(clk),
    .reset(reset),
    .acc_busy_i(acc_busy_i),
    .acc_finished_i(acc_finished_i),
    .acc_result_i(acc_result_i),
    .mark_buff_as_full_i(mark_buff_as_full_i),
    .full_buff_id_i(full_buff_id_i),
    .full_buff_usage_i(full_buff_usage_i),
    .start_new_task_i(start_new_task_i),
    .tgt_acc_id_i(tgt_acc_id_i),
    .tgt_buff_idx_i(tgt_buff_idx_i),
    .tgt_usage_count_i(tgt_usage_count_i),
    .tgt_colour_i(tgt_colour_i),
    .src1_buff_idx_i(src1_buff_idx_i),
    .src2_buff_idx_i(src2_buff_idx_i),
    .src3_buff_idx_i(src3_buff_idx_i),
    .acc_available_o(acc_available_o),
    .buffers_full_o(buffers_full_o),
    .buffers_free_o(buffers_free_o),
    .buffers_colour_o(buffers_colour_o),
    .target_status_o(target_status_o)
);

// helpers
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

task chk2;
    input [1:0] got;
    input [1:0] exp;
    input [255:0] label;
    begin
        if (got !== exp) begin
            $display("FAIL [%0t] %0s: got=%0b exp=%0b", $time, label, got, exp);
            errors = errors + 1;
        end
    end
endtask

// Hold all task-dispatch inputs low for one cycle
task deassert_task;
    begin
        @(posedge clk); #1;
        start_new_task_i  = 1'b0;
        tgt_acc_id_i      = 1'b0;
        tgt_buff_idx_i    = 4'd0;
        tgt_usage_count_i = 3'd0;
        tgt_colour_i      = 1'b0;
        src1_buff_idx_i   = 4'd0;
        src2_buff_idx_i   = 4'd0;
        src3_buff_idx_i   = 4'd0;
    end
endtask

// Assert acc_finished for one cycle; wait for state to propagate (2 extra posedges)
task signal_done;
    input [1:0] which_acc;
    input [1:0] result;
    begin
        @(posedge clk); #1;
        acc_finished_i = which_acc;
        acc_result_i   = result;
        @(posedge clk); #1;        // pending_r latches
        acc_finished_i = 2'b00;
        acc_result_i   = 2'b00;
        @(posedge clk);            // buffer_state_entry regs update
        @(negedge clk);
    end
endtask

initial begin
    reset               = 1'b1;
    acc_busy_i          = 2'b00;
    acc_finished_i      = 2'b00;
    acc_result_i        = 2'b00;
    mark_buff_as_full_i = 1'b0;
    full_buff_id_i      = 4'd0;
    full_buff_usage_i   = 3'd0;
    start_new_task_i    = 1'b0;
    tgt_acc_id_i        = 1'b0;
    tgt_buff_idx_i      = 4'd0;
    tgt_usage_count_i   = 3'd0;
    tgt_colour_i        = 1'b0;
    src1_buff_idx_i     = 4'd0;
    src2_buff_idx_i     = 4'd0;
    src3_buff_idx_i     = 4'd0;

    repeat(3) @(posedge clk); #1;
    reset = 1'b0;
    @(negedge clk);

    // ------------------------------------------------------------------
    // Test 1: reset state
    // ------------------------------------------------------------------
    chk2(acc_available_o,  2'b11,     "reset: both accs available");
    if (buffers_free_o !== 16'hFFFF) begin
        $display("FAIL reset: buffers_free_o=%0h, expected FFFF", buffers_free_o);
        errors = errors + 1;
    end
    if (buffers_full_o !== 16'h0000) begin
        $display("FAIL reset: buffers_full_o=%0h, expected 0000", buffers_full_o);
        errors = errors + 1;
    end

    // ------------------------------------------------------------------
    // Test 2: mark_buff_as_full – external data arrives into buff 3
    //         (2 future consumers)
    // ------------------------------------------------------------------
    @(posedge clk); #1;
    mark_buff_as_full_i = 1'b1;
    full_buff_id_i      = 4'd3;
    full_buff_usage_i   = 3'd2;
    @(posedge clk); #1;
    mark_buff_as_full_i = 1'b0;
    @(negedge clk);
    chk(buffers_full_o[3], 1'b1, "mark_full buff3: full");
    chk(buffers_free_o[3], 1'b0, "mark_full buff3: not free");

    // ------------------------------------------------------------------
    // Test 3: dispatch task on acc0
    //   tgt=buff5 (2 consumers), src1=buff3, src2/3=buff0
    // ------------------------------------------------------------------
    @(posedge clk); #1;
    start_new_task_i  = 1'b1;
    tgt_acc_id_i      = 1'b0;
    tgt_buff_idx_i    = 4'd5;
    tgt_usage_count_i = 3'd2;
    tgt_colour_i      = 1'b0;
    src1_buff_idx_i   = 4'd3;
    src2_buff_idx_i   = 4'd0;
    src3_buff_idx_i   = 4'd0;
    deassert_task;
    @(negedge clk);
    chk(buffers_free_o[5],  1'b0, "dispatched: buff5 not free");
    chk(buffers_full_o[5],  1'b0, "dispatched: buff5 not yet full");
    chk(acc_available_o[0], 1'b0, "dispatched: acc0 busy");

    // ------------------------------------------------------------------
    // Test 4: acc0 completes
    //   buff5 → full; buff3 usage 2→1 (still full); acc0 free
    // ------------------------------------------------------------------
    signal_done(2'b01, 2'b00);   // acc0 done, result=success
    chk(buffers_full_o[5],  1'b1, "acc0 done: buff5 full");
    chk(acc_available_o[0], 1'b1, "acc0 done: acc0 free");
    chk(buffers_full_o[3],  1'b1, "acc0 done: buff3 still full (1 consumer left)");
    chk(buffers_free_o[3],  1'b0, "acc0 done: buff3 still not free");

    // ------------------------------------------------------------------
    // Test 5: dispatch second task on acc1
    //   tgt=buff6 (1 consumer), src1=buff3 (last consume), src2=buff5
    // ------------------------------------------------------------------
    @(posedge clk); #1;
    start_new_task_i  = 1'b1;
    tgt_acc_id_i      = 1'b1;
    tgt_buff_idx_i    = 4'd6;
    tgt_usage_count_i = 3'd1;
    tgt_colour_i      = 1'b0;
    src1_buff_idx_i   = 4'd3;
    src2_buff_idx_i   = 4'd5;
    src3_buff_idx_i   = 4'd0;
    deassert_task;
    @(negedge clk);
    chk(acc_available_o[1], 1'b0, "task2 dispatched: acc1 busy");

    // ------------------------------------------------------------------
    // Test 6: acc1 completes
    //   buff6 → full; buff3 usage 1→0 (freed); buff5 usage 2→1 (still full)
    // ------------------------------------------------------------------
    signal_done(2'b10, 2'b00);   // acc1 done
    chk(buffers_full_o[6],  1'b1, "acc1 done: buff6 full");
    chk(acc_available_o[1], 1'b1, "acc1 done: acc1 free");
    chk(buffers_free_o[3],  1'b1, "acc1 done: buff3 freed (all consumers done)");
    chk(buffers_full_o[3],  1'b0, "acc1 done: buff3 no longer full");
    chk(buffers_full_o[5],  1'b1, "acc1 done: buff5 still full (1 consumer left)");

    // ------------------------------------------------------------------
    // Test 7: both accs idle after test sequence
    // ------------------------------------------------------------------
    chk2(acc_available_o, 2'b11, "end: both accs free");

    // ------------------------------------------------------------------
    @(posedge clk);
    if (errors == 0)
        $display("PASS – sch_buffer_state: all tests passed.");
    else
        $display("FAIL – sch_buffer_state: %0d error(s).", errors);
    $finish;
end

initial begin
    #20000;
    $display("TIMEOUT – sch_buffer_state");
    $finish;
end

endmodule
