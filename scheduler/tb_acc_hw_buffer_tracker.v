// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
`timescale 1ns/1ps

// Testbench for acc_hw_buffer_tracker.v
// Covers: reset, new-task (busy + slot capture), task-finished, back-to-back tasks

module top;

localparam NUM_SLOTS    = 6;
localparam BUFF_INDX_SZ = 4;
localparam TGT_COUNT_SZ = 3;
localparam MODE_SZ      = 2;

localparam MODE_UNUSED = 2'b00;
localparam MODE_SRC    = 2'b01;
localparam MODE_RW     = 2'b10;
localparam MODE_TGT    = 2'b11;

reg clk, reset;
initial clk = 1'b0;
always  #5 clk = ~clk;

initial begin
    $dumpfile("tb_acc_hw_buffer_tracker.vcd");
    $dumpvars(0, top);
end

integer errors;
initial errors = 0;

reg                                  new_task_i;
reg  [NUM_SLOTS*BUFF_INDX_SZ-1:0]   slot_buff_i;
reg  [NUM_SLOTS*MODE_SZ-1:0]         slot_mode_i;
reg  [NUM_SLOTS*TGT_COUNT_SZ-1:0]    slot_ntgt_i;
reg                                  task_finished_i;

wire                                 acc_free_o;
wire [NUM_SLOTS*BUFF_INDX_SZ-1:0]   slot_buff_o;
wire [NUM_SLOTS*MODE_SZ-1:0]         slot_mode_o;
wire [NUM_SLOTS*TGT_COUNT_SZ-1:0]    slot_ntgt_o;

acc_hw_buffer_tracker #(
    .NUM_SLOTS(NUM_SLOTS),
    .BUFF_INDX_SZ(BUFF_INDX_SZ),
    .TGT_COUNT_SZ(TGT_COUNT_SZ),
    .MODE_SZ(MODE_SZ)
) dut (
    .clk(clk),
    .reset(reset),
    .new_task_i(new_task_i),
    .slot_buff_i(slot_buff_i),
    .slot_mode_i(slot_mode_i),
    .slot_ntgt_i(slot_ntgt_i),
    .task_finished_i(task_finished_i),
    .acc_free_o(acc_free_o),
    .slot_buff_o(slot_buff_o),
    .slot_mode_o(slot_mode_o),
    .slot_ntgt_o(slot_ntgt_o)
);

// Helpers
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
    input [3:0]  got;
    input [3:0]  exp;
    input [255:0] label;
    begin
        if (got !== exp) begin
            $display("FAIL [%0t] %0s: got=%0d exp=%0d", $time, label, got, exp);
            errors = errors + 1;
        end
    end
endtask

task chk_mode;
    input [1:0]  got;
    input [1:0]  exp;
    input [255:0] label;
    begin
        if (got !== exp) begin
            $display("FAIL [%0t] %0s: got=%0b exp=%0b", $time, label, got, exp);
            errors = errors + 1;
        end
    end
endtask

task set_slot;
    input integer          s;
    input [MODE_SZ-1:0]    md;
    input [BUFF_INDX_SZ-1:0] bid;
    input [TGT_COUNT_SZ-1:0] ntgt;
    begin
        slot_mode_i[s*MODE_SZ      +: MODE_SZ]      = md;
        slot_buff_i[s*BUFF_INDX_SZ +: BUFF_INDX_SZ] = bid;
        slot_ntgt_i[s*TGT_COUNT_SZ +: TGT_COUNT_SZ] = ntgt;
    end
endtask

task clear_slots;
    integer s;
    begin
        for (s = 0; s < NUM_SLOTS; s = s + 1)
            set_slot(s, MODE_UNUSED, 4'd0, 3'd0);
    end
endtask

// Shorthand slot reads from output:
function [BUFF_INDX_SZ-1:0] out_buff;
    input integer s;
    out_buff = slot_buff_o[s*BUFF_INDX_SZ +: BUFF_INDX_SZ];
endfunction

function [MODE_SZ-1:0] out_mode;
    input integer s;
    out_mode = slot_mode_o[s*MODE_SZ +: MODE_SZ];
endfunction

function [TGT_COUNT_SZ-1:0] out_ntgt;
    input integer s;
    out_ntgt = slot_ntgt_o[s*TGT_COUNT_SZ +: TGT_COUNT_SZ];
endfunction

initial begin
    reset           = 1'b1;
    new_task_i      = 1'b0;
    task_finished_i = 1'b0;
    clear_slots;

    repeat(3) @(posedge clk); #1;
    reset = 1'b0;
    @(negedge clk);

    // ------------------------------------------------------------------
    // Test 1: after reset accelerator is free
    // ------------------------------------------------------------------
    chk_bit(acc_free_o, 1'b1, "reset: acc_free");

    // ------------------------------------------------------------------
    // Test 2: new_task clears acc_free and captures all slot data
    //   Slot 0: source buff3
    //   Slot 1: source buff5
    //   Slot 3: target buff7 (ntgt=1)
    // ------------------------------------------------------------------
    @(posedge clk); #1;
    new_task_i = 1'b1;
    set_slot(0, MODE_SRC, 4'd3, 3'd0);
    set_slot(1, MODE_SRC, 4'd5, 3'd0);
    set_slot(3, MODE_TGT, 4'd7, 3'd1);
    @(posedge clk); #1;
    new_task_i = 1'b0;
    @(negedge clk);
    chk_bit(acc_free_o,         1'b0,       "new_task: acc_free→0");
    chk_idx(out_buff(0),        4'd3,        "new_task: slot0 buff");
    chk_mode(out_mode(0),       MODE_SRC,    "new_task: slot0 mode=src");
    chk_idx(out_buff(1),        4'd5,        "new_task: slot1 buff");
    chk_mode(out_mode(1),       MODE_SRC,    "new_task: slot1 mode=src");
    chk_idx(out_buff(3),        4'd7,        "new_task: slot3 buff");
    chk_mode(out_mode(3),       MODE_TGT,    "new_task: slot3 mode=tgt");
    chk_idx(out_ntgt(3),        3'd1,        "new_task: slot3 ntgt=1");

    // ------------------------------------------------------------------
    // Test 3: acc stays busy while task is running
    // ------------------------------------------------------------------
    repeat(4) @(posedge clk);
    @(negedge clk);
    chk_bit(acc_free_o, 1'b0, "mid-task: still busy");

    // ------------------------------------------------------------------
    // Test 4: task_finished restores acc_free (slot data retained)
    // ------------------------------------------------------------------
    @(posedge clk); #1;
    task_finished_i = 1'b1;
    @(posedge clk); #1;
    task_finished_i = 1'b0;
    @(negedge clk);
    chk_bit(acc_free_o,   1'b1, "task_finished: acc_free→1");
    chk_idx(out_buff(3),  4'd7, "task_finished: slot data retained");

    // ------------------------------------------------------------------
    // Test 5: second task with RW slot immediately after finish
    //   Slot 4: RW buff12 (ntgt=2); Slot 5: target buff9 (ntgt=1)
    // ------------------------------------------------------------------
    @(posedge clk); #1;
    clear_slots;
    new_task_i = 1'b1;
    set_slot(4, MODE_RW,  4'd12, 3'd2);
    set_slot(5, MODE_TGT, 4'd9,  3'd1);
    @(posedge clk); #1;
    new_task_i = 1'b0;
    @(negedge clk);
    chk_bit(acc_free_o,     1'b0,      "task2: busy");
    chk_mode(out_mode(4),   MODE_RW,   "task2: slot4 mode=rw");
    chk_idx(out_buff(4),    4'd12,     "task2: slot4 buff");
    chk_idx(out_ntgt(4),    3'd2,      "task2: slot4 ntgt=2");
    chk_mode(out_mode(5),   MODE_TGT,  "task2: slot5 mode=tgt");
    chk_idx(out_buff(5),    4'd9,      "task2: slot5 buff");
    chk_idx(out_ntgt(5),    3'd1,      "task2: slot5 ntgt=1");
    // unused slots should be zero:
    chk_mode(out_mode(0),   MODE_UNUSED, "task2: slot0 unused");

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
