// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
// =============================================================================
// tb_acc_tracker_d2  (scheduler — acc_hw_buffer_tracker DEPTH=2)
//
// Authors      : Simon Davidson & Claude
// Created      : 2026-08-22
// Last modified: 2026-08-22
//
// DEPTH=2 lets an accelerator hold two tasks in flight. The property that
// matters is which slot set is PUBLISHED: sch_buffer_state applies it on the
// next completion, so it must always be the OLDEST in-flight task's. Publishing
// the younger one frees and fills the wrong buffers — the failure the single
// slot register caused before DEPTH existed.
//
// T4 is the default-off guarantee: with acc_ready_next_i tied low, DEPTH=2 must
// be indistinguishable from DEPTH=1.
// =============================================================================
`timescale 1ns/1ps

module tb_acc_tracker_d2;

    localparam NUM_SLOTS = 6, BUFF_INDX_SZ = 4, TGT_COUNT_SZ = 4, MODE_SZ = 2;
    localparam BW = NUM_SLOTS*BUFF_INDX_SZ, MW = NUM_SLOTS*MODE_SZ,
               NW = NUM_SLOTS*TGT_COUNT_SZ;

    reg clk = 1'b0, reset = 1'b1;
    reg           new_task_i = 1'b0, task_finished_i = 1'b0, acc_ready_next_i = 1'b0;
    reg  [BW-1:0] slot_buff_i = 0;
    reg  [MW-1:0] slot_mode_i = 0;
    reg  [NW-1:0] slot_ntgt_i = 0;

    wire          d2_free,  d1_free;
    wire [BW-1:0] d2_buff,  d1_buff;
    wire [MW-1:0] d2_mode,  d1_mode;
    wire [NW-1:0] d2_ntgt,  d1_ntgt;

    integer verif_errors, verif_checks;
    `include "../verif/checks.vh"

    acc_hw_buffer_tracker #(.NUM_SLOTS(NUM_SLOTS), .BUFF_INDX_SZ(BUFF_INDX_SZ),
                            .TGT_COUNT_SZ(TGT_COUNT_SZ), .MODE_SZ(MODE_SZ),
                            .DEPTH(2)) d2 (
        .clk(clk), .reset(reset), .new_task_i(new_task_i),
        .slot_buff_i(slot_buff_i), .slot_mode_i(slot_mode_i), .slot_ntgt_i(slot_ntgt_i),
        .task_finished_i(task_finished_i), .acc_ready_next_i(acc_ready_next_i),
        .acc_free_o(d2_free), .slot_buff_o(d2_buff),
        .slot_mode_o(d2_mode), .slot_ntgt_o(d2_ntgt));

    acc_hw_buffer_tracker #(.NUM_SLOTS(NUM_SLOTS), .BUFF_INDX_SZ(BUFF_INDX_SZ),
                            .TGT_COUNT_SZ(TGT_COUNT_SZ), .MODE_SZ(MODE_SZ),
                            .DEPTH(1)) d1 (
        .clk(clk), .reset(reset), .new_task_i(new_task_i),
        .slot_buff_i(slot_buff_i), .slot_mode_i(slot_mode_i), .slot_ntgt_i(slot_ntgt_i),
        .task_finished_i(task_finished_i), .acc_ready_next_i(1'b0),
        .acc_free_o(d1_free), .slot_buff_o(d1_buff),
        .slot_mode_o(d1_mode), .slot_ntgt_o(d1_ntgt));

    always #5 clk = ~clk;

    task tick;
        begin
            @(posedge clk); #1;
            new_task_i = 1'b0; task_finished_i = 1'b0;
        end
    endtask

    task push;                       // distinct payload per task id
        input [7:0] id;
        begin
            slot_buff_i = {NUM_SLOTS{id[BUFF_INDX_SZ-1:0]}};
            slot_mode_i = {NUM_SLOTS{id[MODE_SZ-1:0]}};
            slot_ntgt_i = {NUM_SLOTS{id[TGT_COUNT_SZ-1:0]}};
            new_task_i  = 1'b1;
        end
    endtask

    function [BW-1:0] buff_of; input [7:0] id; buff_of = {NUM_SLOTS{id[BUFF_INDX_SZ-1:0]}}; endfunction
    function [NW-1:0] ntgt_of; input [7:0] id; ntgt_of = {NUM_SLOTS{id[TGT_COUNT_SZ-1:0]}}; endfunction

    integer i, outstanding;
    reg [3:0] pick;

    initial begin
        verif_errors = 0; verif_checks = 0;
        $display("=== tb_acc_tracker_d2 (scheduler) ===");
        tick; tick; reset = 1'b0; tick;
        check_eq(d2_free, 1'b1, "T0 empty tracker is free");

        // ---- 1. ready_next low: a second task is refused -------------------
        acc_ready_next_i = 1'b0;
        push(8'h5); tick;
        check_eq(d2_free, 1'b0, "T1 one task, not ready -> busy");
        tick;
        check_eq(d2_free, 1'b0, "T1 stays busy while not ready");
        task_finished_i = 1'b1; tick; tick;
        check_eq(d2_free, 1'b1, "T1 free again after completion");

        // ---- 2. the ordering property --------------------------------------
        acc_ready_next_i = 1'b1;
        push(8'h3); tick;                                   // task A
        check_eq(d2_buff, buff_of(8'h3), "T2 head is A");
        check_eq(d2_free, 1'b1, "T2 ready_next -> can take a second");
        push(8'h9); tick;                                   // task B
        check_eq(d2_buff, buff_of(8'h3), "T2 head is STILL A with two in flight");
        check_eq(d2_ntgt, ntgt_of(8'h3), "T2 A's ntgt still published");
        check_eq(d2_free, 1'b0, "T2 two in flight -> full");
        task_finished_i = 1'b1; tick;                       // A retires
        check_eq(d2_buff, buff_of(8'h9), "T2 head becomes B after A retires");
        check_eq(d2_ntgt, ntgt_of(8'h9), "T2 B's ntgt now published");
        task_finished_i = 1'b1; tick; tick;
        check_eq(d2_free, 1'b1, "T2 empty again");

        // ---- 3. push and pop on the SAME cycle at occupancy 2 --------------
        push(8'h1); tick;
        push(8'h2); tick;                                   // A=1, B=2
        push(8'h4); task_finished_i = 1'b1; tick;           // C arrives as A ends
        check_eq(d2_buff, buff_of(8'h2), "T3 head is B after simultaneous push+pop");
        task_finished_i = 1'b1; tick;
        check_eq(d2_buff, buff_of(8'h4), "T3 then C");
        task_finished_i = 1'b1; tick; tick;
        check_eq(d2_free, 1'b1, "T3 drains empty");

        // ---- 4. equivalence with DEPTH=1 while never overlapping -----------
        reset = 1'b1; tick; reset = 1'b0; acc_ready_next_i = 1'b0; outstanding = 0;
        for (i = 0; i < 3000; i = i + 1) begin
            pick = $urandom_range(0, 3);
            if (pick < 2 && outstanding == 0) begin
                push($urandom_range(1, 255)); outstanding = 1;
            end else if (pick == 2 && outstanding == 1) begin
                task_finished_i = 1'b1; outstanding = 0;
            end
            tick;
            check_eq(d2_free, d1_free, "T4 acc_free matches DEPTH=1");
            check_eq(d2_buff, d1_buff, "T4 slot_buff matches DEPTH=1");
            check_eq(d2_mode, d1_mode, "T4 slot_mode matches DEPTH=1");
            check_eq(d2_ntgt, d1_ntgt, "T4 slot_ntgt matches DEPTH=1");
        end

        `VERIF_EPILOGUE("tb_acc_tracker_d2")
    end

    `VERIF_WATCHDOG(2000000)

endmodule
