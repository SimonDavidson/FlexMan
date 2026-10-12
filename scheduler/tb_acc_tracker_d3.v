// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
// =============================================================================
// tb_acc_tracker_d3  (scheduler — acc_hw_buffer_tracker DEPTH=3, gen_depthn)
//
// Authors      : Simon Davidson & Claude
// Created      : 2026-10-12
// Last modified: 2026-10-12
//
// DEPTH=3 lets an accelerator hold three tasks in flight. Checked against a
// behavioural in-order queue under random dispatch / completion / ready_next,
// driven the way sch_buffer_state drives it (a dispatch only while acc_free_o,
// a completion only while a task is in flight, push and pop in one cycle):
//   * the PUBLISHED slot set is always the oldest in-flight task's;
//   * acc_free_o is the registered rule  empty -> 1, 1..2 -> ready_next, 3 -> 0;
//   * occupancy 3 is reached, and push+pop is exercised at occupancy 1 and 2.
// Then the default-off guarantee: with acc_ready_next_i low, DEPTH=3 and DEPTH=2
// are cycle-for-cycle identical to DEPTH=1.
// =============================================================================
`timescale 1ns/1ps

module tb_acc_tracker_d3;

    localparam NUM_SLOTS = 6, BUFF_INDX_SZ = 4, TGT_COUNT_SZ = 4, MODE_SZ = 2;
    localparam BW = NUM_SLOTS*BUFF_INDX_SZ, MW = NUM_SLOTS*MODE_SZ,
               NW = NUM_SLOTS*TGT_COUNT_SZ, SW = BW + MW + NW;

    reg clk = 1'b0, reset = 1'b1;
    always #5 clk = ~clk;
    reg           push = 1'b0, pop = 1'b0, rdy = 1'b0;
    reg  [SW-1:0] set_i = 0;

    wire          f3, f2, f1;
    wire [BW-1:0] b3, b2, b1;
    wire [MW-1:0] m3, m2, m1;
    wire [NW-1:0] n3, n2, n1;

    acc_hw_buffer_tracker #(.NUM_SLOTS(NUM_SLOTS), .BUFF_INDX_SZ(BUFF_INDX_SZ),
        .TGT_COUNT_SZ(TGT_COUNT_SZ), .MODE_SZ(MODE_SZ), .DEPTH(3)) u3 (
        .clk(clk), .reset(reset), .new_task_i(push),
        .slot_buff_i(set_i[BW-1:0]), .slot_mode_i(set_i[BW +: MW]), .slot_ntgt_i(set_i[BW+MW +: NW]),
        .task_finished_i(pop), .acc_ready_next_i(rdy),
        .acc_free_o(f3), .slot_buff_o(b3), .slot_mode_o(m3), .slot_ntgt_o(n3));
    acc_hw_buffer_tracker #(.NUM_SLOTS(NUM_SLOTS), .BUFF_INDX_SZ(BUFF_INDX_SZ),
        .TGT_COUNT_SZ(TGT_COUNT_SZ), .MODE_SZ(MODE_SZ), .DEPTH(2)) u2 (
        .clk(clk), .reset(reset), .new_task_i(push),
        .slot_buff_i(set_i[BW-1:0]), .slot_mode_i(set_i[BW +: MW]), .slot_ntgt_i(set_i[BW+MW +: NW]),
        .task_finished_i(pop), .acc_ready_next_i(rdy),
        .acc_free_o(f2), .slot_buff_o(b2), .slot_mode_o(m2), .slot_ntgt_o(n2));
    acc_hw_buffer_tracker #(.NUM_SLOTS(NUM_SLOTS), .BUFF_INDX_SZ(BUFF_INDX_SZ),
        .TGT_COUNT_SZ(TGT_COUNT_SZ), .MODE_SZ(MODE_SZ), .DEPTH(1)) u1 (
        .clk(clk), .reset(reset), .new_task_i(push),
        .slot_buff_i(set_i[BW-1:0]), .slot_mode_i(set_i[BW +: MW]), .slot_ntgt_i(set_i[BW+MW +: NW]),
        .task_finished_i(pop), .acc_ready_next_i(1'b0),
        .acc_free_o(f1), .slot_buff_o(b1), .slot_mode_o(m1), .slot_ntgt_o(n1));

    // behavioural model of the DEPTH=3 tracker
    reg  [SW-1:0] q [0:3];
    integer       occ, nxt, k, errs, checks, cyc, hit3, pp_at [0:3];
    reg           free_m;
    wire [SW-1:0] head3 = {n3, m3, b3};

    task chk(input cond, input [8*48-1:0] what);
        begin
            checks = checks + 1;
            if (!cond) begin
                errs = errs + 1;
                if (errs <= 10) $display("[%0t] MISMATCH %0s (occ %0d)", $time, what, occ);
            end
        end
    endtask

    initial begin
        errs = 0; checks = 0; occ = 0; free_m = 1; hit3 = 0;
        for (k = 0; k < 4; k = k + 1) pp_at[k] = 0;
        repeat (3) @(negedge clk);
        reset = 0;
        // ---- random phase: DEPTH=3 against the model ----
        for (cyc = 0; cyc < 20000; cyc = cyc + 1) begin
            @(negedge clk);
            rdy   = ($urandom % 4) != 0;
            push  = f3 && (($urandom % 3) != 0);         // dispatch only while free
            pop   = (occ > 0) && (($urandom % 3) == 0);  // completion only in flight
            set_i = {$urandom, $urandom, $urandom};
            if (push && pop) pp_at[occ] = pp_at[occ] + 1;
            @(posedge clk); #1;
            // model update (same edge)
            if (pop) begin for (k = 0; k < 3; k = k + 1) q[k] = q[k + 1]; occ = occ - 1; end
            if (push) begin q[occ] = set_i; occ = occ + 1; end
            if (occ == 3) hit3 = hit3 + 1;
            free_m = (occ == 0) ? 1'b1 : (occ < 3) ? rdy : 1'b0;
            chk(f3 === free_m, "acc_free_o");
            if (occ > 0) chk(head3 === q[0], "published head is the oldest");
        end
        push = 0; pop = 0;
        // drain
        while (occ > 0) begin
            @(negedge clk); pop = 1; @(posedge clk); #1;
            for (k = 0; k < 3; k = k + 1) q[k] = q[k + 1]; occ = occ - 1;
            if (occ > 0) chk(head3 === q[0], "head while draining");
        end
        @(negedge clk); pop = 0;
        chk(hit3 > 0, "occupancy 3 reached");
        // (push+pop at occupancy 3 cannot happen: a full tracker is not free,
        //  so no dispatch arrives -- the free rule, checked every cycle above)
        chk(pp_at[1] > 0 && pp_at[2] > 0 && pp_at[3] == 0, "push+pop at occupancy 1 and 2 only");
        // ---- default-off: ready_next low -> DEPTH 3 == DEPTH 2 == DEPTH 1 ----
        rdy = 0;
        for (cyc = 0; cyc < 5000; cyc = cyc + 1) begin
            @(negedge clk);
            push  = f1 && (($urandom % 2) != 0);
            pop   = !f1 && !push && (($urandom % 3) == 0);
            set_i = {$urandom, $urandom, $urandom};
            @(posedge clk); #1;
            chk(f3 === f1 && f2 === f1, "free equals DEPTH=1 with ready low");
            chk({n3, m3, b3} === {n1, m1, b1} && {n2, m2, b2} === {n1, m1, b1},
                "head equals DEPTH=1 with ready low");
        end
        $display("tb_acc_tracker_d3: %0d checks, %0d mismatches; occ=3 on %0d cycles; push+pop at occ1/2/3: %0d/%0d/%0d",
                 checks, errs, hit3, pp_at[1], pp_at[2], pp_at[3]);
        if (errs == 0) $display("[TB] PASS"); else $display("[TB] FAIL");
        $finish;
    end
endmodule
