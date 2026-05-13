// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
`timescale 1ns/1ps

// Testbench for sch_buffer_state.v
//
// Buffer state change timing (cycles after asserting acc_finished_i):
//   +1 posedge: acc_process_pending_r latches the finished flag
//   +2 posedge: cleanup combinatorial signals drive buffer_state_entry regs
//               → buffers_full_o / buffers_free_o / acc_available_o update

module top;

localparam NUM_BUFFERS         = 16;
localparam NUM_HW_ACCELERATORS = 2;
localparam BUFF_INDX_SZ        = 4;
localparam TGT_ACC_SZ          = 2;
localparam TGT_COUNT_SZ        = 3;
localparam NUM_SLOTS           = 6;
localparam MODE_SZ             = 2;

// Slot modes:
localparam MODE_UNUSED = 2'b00;
localparam MODE_SRC    = 2'b01;
localparam MODE_RW     = 2'b10;
localparam MODE_TGT    = 2'b11;

reg clk, reset;
initial clk = 1'b0;
always  #5 clk = ~clk;

initial begin
    $dumpfile("tb_sch_buffer_state.vcd");
    $dumpvars(0, top);
end

// DUT ports
reg [NUM_HW_ACCELERATORS-1:0]      acc_busy_i;
reg [NUM_HW_ACCELERATORS-1:0]      acc_finished_i;
reg [NUM_HW_ACCELERATORS-1:0]      acc_result_i;

reg                                mark_buff_as_full_i;
reg [BUFF_INDX_SZ-1:0]             full_buff_id_i;
reg [TGT_COUNT_SZ-1:0]             full_buff_usage_i;

reg                                start_new_task_i;
reg [TGT_ACC_SZ-1:0]               tgt_acc_id_i;
reg [NUM_SLOTS*BUFF_INDX_SZ-1:0]   slot_buff_i;
reg [NUM_SLOTS*MODE_SZ-1:0]         slot_mode_i;
reg [NUM_SLOTS*TGT_COUNT_SZ-1:0]    slot_ntgt_i;
reg                                tgt_colour_i;

wire [NUM_HW_ACCELERATORS-1:0]     acc_available_o;
wire [NUM_BUFFERS-1:0]             buffers_full_o;
wire [NUM_BUFFERS-1:0]             buffers_free_o;
wire [NUM_BUFFERS-1:0]             buffers_colour_o;
wire [NUM_BUFFERS-1:0]             target_status_o;

sch_buffer_state #(
    .NUM_BUFFERS(NUM_BUFFERS),
    .NUM_HW_ACCELERATORS(NUM_HW_ACCELERATORS),
    .TGT_ACC_SZ(TGT_ACC_SZ),
    .TGT_COUNT_SZ(TGT_COUNT_SZ),
    .NUM_SLOTS(NUM_SLOTS),
    .MODE_SZ(MODE_SZ)
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
    .slot_buff_i(slot_buff_i),
    .slot_mode_i(slot_mode_i),
    .slot_ntgt_i(slot_ntgt_i),
    .tgt_colour_i(tgt_colour_i),
    .acc_available_o(acc_available_o),
    .buffers_full_o(buffers_full_o),
    .buffers_free_o(buffers_free_o),
    .buffers_colour_o(buffers_colour_o),
    .target_status_o(target_status_o)
);

// ---------------------------------------------------------------
// Helpers

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

// Pack a single slot into the flat vectors at position s:
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

task deassert_task;
    begin
        @(posedge clk); #1;
        start_new_task_i = 1'b0;
        tgt_acc_id_i     = 2'd0;
        tgt_colour_i     = 1'b0;
        clear_slots;
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

// ---------------------------------------------------------------
// Stimulus

initial begin
    reset               = 1'b1;
    acc_busy_i          = 2'b00;
    acc_finished_i      = 2'b00;
    acc_result_i        = 2'b00;
    mark_buff_as_full_i = 1'b0;
    full_buff_id_i      = 4'd0;
    full_buff_usage_i   = 3'd0;
    start_new_task_i    = 1'b0;
    tgt_acc_id_i        = 2'd0;
    tgt_colour_i        = 1'b0;
    clear_slots;

    repeat(3) @(posedge clk); #1;
    reset = 1'b0;
    @(negedge clk);

    // ------------------------------------------------------------------
    // Test 1: reset state
    // ------------------------------------------------------------------
    chk2(acc_available_o, 2'b11, "reset: both accs available");
    if (buffers_free_o !== 16'hFFFF) begin
        $display("FAIL reset: buffers_free_o=%0h, expected FFFF", buffers_free_o);
        errors = errors + 1;
    end
    if (buffers_full_o !== 16'h0000) begin
        $display("FAIL reset: buffers_full_o=%0h, expected 0000", buffers_full_o);
        errors = errors + 1;
    end

    // ------------------------------------------------------------------
    // Test 2: mark_buff_as_full – external data arrives into buff 3 (2 consumers)
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
    //   Slot 0 = source buff3; Slot 3 = target buff5 (ntgt=2)
    // ------------------------------------------------------------------
    @(posedge clk); #1;
    start_new_task_i = 1'b1;
    tgt_acc_id_i     = 2'd0;
    tgt_colour_i     = 1'b0;
    set_slot(0, MODE_SRC, 4'd3, 3'd0);   // source: buff3
    set_slot(3, MODE_TGT, 4'd5, 3'd2);   // target: buff5, 2 consumers
    deassert_task;
    @(negedge clk);
    chk(buffers_free_o[5],  1'b0, "dispatched: buff5 not free");
    chk(buffers_full_o[5],  1'b0, "dispatched: buff5 not yet full");
    chk(acc_available_o[0], 1'b0, "dispatched: acc0 busy");

    // ------------------------------------------------------------------
    // Test 4: acc0 completes
    //   buff5 → full; buff3 usage 2→1 (still full); acc0 free
    // ------------------------------------------------------------------
    signal_done(2'b01, 2'b00);
    chk(buffers_full_o[5],  1'b1, "acc0 done: buff5 full");
    chk(acc_available_o[0], 1'b1, "acc0 done: acc0 free");
    chk(buffers_full_o[3],  1'b1, "acc0 done: buff3 still full (1 consumer left)");
    chk(buffers_free_o[3],  1'b0, "acc0 done: buff3 still not free");

    // ------------------------------------------------------------------
    // Test 5: dispatch second task on acc1
    //   Slot 0 = source buff3 (last consume); Slot 1 = source buff5
    //   Slot 3 = target buff6 (ntgt=1)
    // ------------------------------------------------------------------
    @(posedge clk); #1;
    start_new_task_i = 1'b1;
    tgt_acc_id_i     = 2'd1;
    tgt_colour_i     = 1'b0;
    set_slot(0, MODE_SRC, 4'd3, 3'd0);   // source: buff3 (last consumer)
    set_slot(1, MODE_SRC, 4'd5, 3'd0);   // source: buff5
    set_slot(3, MODE_TGT, 4'd6, 3'd1);   // target: buff6, 1 consumer
    deassert_task;
    @(negedge clk);
    chk(acc_available_o[1], 1'b0, "task2 dispatched: acc1 busy");

    // ------------------------------------------------------------------
    // Test 6: acc1 completes
    //   buff6 → full; buff3 usage 1→0 (freed); buff5 usage 2→1 (still full)
    // ------------------------------------------------------------------
    signal_done(2'b10, 2'b00);
    chk(buffers_full_o[6],  1'b1, "acc1 done: buff6 full");
    chk(acc_available_o[1], 1'b1, "acc1 done: acc1 free");
    chk(buffers_free_o[3],  1'b1, "acc1 done: buff3 freed");
    chk(buffers_full_o[3],  1'b0, "acc1 done: buff3 no longer full");
    chk(buffers_full_o[5],  1'b1, "acc1 done: buff5 still full (1 consumer left)");
    chk2(acc_available_o,   2'b11, "end of tests 1-6: both accs free");

    // ------------------------------------------------------------------
    // Test 7: RW buffer (mode 10) — buff7 is full (ntgt=1);
    //   Dispatch task on acc0 with slot4=RW buff7 (ntgt=2 after completion)
    //   buff7 must go busy (free=0, full=0) on dispatch, full again on completion
    // ------------------------------------------------------------------
    @(posedge clk); #1;
    mark_buff_as_full_i = 1'b1;
    full_buff_id_i      = 4'd7;
    full_buff_usage_i   = 3'd1;
    @(posedge clk); #1;
    mark_buff_as_full_i = 1'b0;
    @(negedge clk);
    chk(buffers_full_o[7], 1'b1, "RW setup: buff7 full");
    chk(buffers_free_o[7], 1'b0, "RW setup: buff7 not free");

    @(posedge clk); #1;
    start_new_task_i = 1'b1;
    tgt_acc_id_i     = 2'd0;
    tgt_colour_i     = 1'b1;
    set_slot(4, MODE_RW,  4'd7, 3'd2);   // RW: buff7, 2 consumers after completion
    set_slot(5, MODE_TGT, 4'd8, 3'd1);   // target: buff8, 1 consumer
    deassert_task;
    @(negedge clk);
    chk(buffers_full_o[7], 1'b0, "RW dispatch: buff7 no longer full (busy)");
    chk(buffers_free_o[7], 1'b0, "RW dispatch: buff7 not free (claimed)");
    chk(buffers_full_o[8], 1'b0, "RW dispatch: buff8 not yet full");
    chk(buffers_free_o[8], 1'b0, "RW dispatch: buff8 reserved");
    chk(acc_available_o[0], 1'b0, "RW dispatch: acc0 busy");

    signal_done(2'b01, 2'b00);
    chk(buffers_full_o[7],  1'b1, "RW done: buff7 full again");
    chk(buffers_free_o[7],  1'b0, "RW done: buff7 not free (2 consumers pending)");
    chk(buffers_full_o[8],  1'b1, "RW done: buff8 full");
    chk(acc_available_o[0], 1'b1, "RW done: acc0 free");

    // ------------------------------------------------------------------
    // Test 8: external mark_full with colour; verify colour output
    // ------------------------------------------------------------------
    @(posedge clk); #1;
    mark_buff_as_full_i = 1'b1;
    full_buff_id_i      = 4'd9;
    full_buff_usage_i   = 3'd1;
    @(posedge clk); #1;
    mark_buff_as_full_i = 1'b0;
    @(negedge clk);
    chk(buffers_full_o[9],    1'b1, "colour: buff9 full after mark");
    // colour is set by buff_new_tgt_i / buff_rw_claim_i at dispatch; mark_as_full
    // does not set colour — colour defaults to 0 from reset:
    chk(buffers_colour_o[9],  1'b0, "colour: buff9 colour defaults to 0");

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
