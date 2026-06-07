// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
// =============================================================================
// tb_acc_hw_buffer_tracker  (scheduler — per-accelerator slot latch + acc_free)
//
// Authors      : Simon Davidson & Claude
// Created      : 2026-05-12
// Last modified: 2026-06-07
//
// Aggressive rewrite with a cycle-accurate SW reference model of the tracker:
//   slot_*_r : reset->0 ; new_task->latch slot_*_i ; else hold
//   acc_free : reset->1 ; new_task->0 ; task_finished->1 ; else hold
//             (new_task has priority over task_finished in acc_free_nxt)
// Directed: latch-on-dispatch, replay/retain-on-finish, acc_free clear/set, and
// the new_task+task_finished same-cycle priority case. Plus a constrained-random
// loop driving new_task / task_finished with random slot payloads, checking
// acc_free and all six slot triples every cycle vs the model.
// =============================================================================
`timescale 1ns/1ps

module tb_acc_hw_buffer_tracker;

    localparam integer NUM_SLOTS    = 6;
    localparam integer BUFF_INDX_SZ = 4;
    localparam integer TGT_COUNT_SZ = 3;
    localparam integer MODE_SZ      = 2;
    localparam integer NRAND        = 6000;

    reg                                clk, reset;
    reg                                new_task_i;
    reg  [NUM_SLOTS*BUFF_INDX_SZ-1:0]  slot_buff_i;
    reg  [NUM_SLOTS*MODE_SZ-1:0]       slot_mode_i;
    reg  [NUM_SLOTS*TGT_COUNT_SZ-1:0]  slot_ntgt_i;
    reg                                task_finished_i;

    wire                               acc_free_o;
    wire [NUM_SLOTS*BUFF_INDX_SZ-1:0]  slot_buff_o;
    wire [NUM_SLOTS*MODE_SZ-1:0]       slot_mode_o;
    wire [NUM_SLOTS*TGT_COUNT_SZ-1:0]  slot_ntgt_o;

    integer verif_errors, verif_checks, verif_to;
    `include "../verif/checks.vh"

    acc_hw_buffer_tracker #(
        .NUM_SLOTS(NUM_SLOTS), .BUFF_INDX_SZ(BUFF_INDX_SZ),
        .TGT_COUNT_SZ(TGT_COUNT_SZ), .MODE_SZ(MODE_SZ)) dut (
        .clk(clk), .reset(reset),
        .new_task_i(new_task_i),
        .slot_buff_i(slot_buff_i), .slot_mode_i(slot_mode_i), .slot_ntgt_i(slot_ntgt_i),
        .task_finished_i(task_finished_i),
        .acc_free_o(acc_free_o),
        .slot_buff_o(slot_buff_o), .slot_mode_o(slot_mode_o), .slot_ntgt_o(slot_ntgt_o));

    initial clk = 1'b0; always #5 clk = ~clk;

    // SW reference model
    reg                                m_free;
    reg [NUM_SLOTS*BUFF_INDX_SZ-1:0]   m_buff;
    reg [NUM_SLOTS*MODE_SZ-1:0]        m_mode;
    reg [NUM_SLOTS*TGT_COUNT_SZ-1:0]   m_ntgt;

    task model_step;
        reg nf;
        begin
            // acc_free
            if      (reset)           nf = 1'b1;
            else if (new_task_i)      nf = 1'b0;   // priority over finished
            else if (task_finished_i) nf = 1'b1;
            else                      nf = m_free;
            m_free = nf;
            // slot registers
            if (reset) begin
                m_buff = 'b0; m_mode = 'b0; m_ntgt = 'b0;
            end else if (new_task_i) begin
                m_buff = slot_buff_i; m_mode = slot_mode_i; m_ntgt = slot_ntgt_i;
            end
        end
    endtask

    task step_and_check;
        input [255:0] tag;
        begin
            model_step;
            @(posedge clk); #1;
            check_bit (acc_free_o,             m_free, {tag, " acc_free"});
            check_eq_u(slot_buff_o,            m_buff, {tag, " slot_buff"});
            check_eq_u(slot_mode_o,            m_mode, {tag, " slot_mode"});
            check_eq_u(slot_ntgt_o,            m_ntgt, {tag, " slot_ntgt"});
        end
    endtask

    task clear_inputs;
        begin
            new_task_i=0; task_finished_i=0;
            slot_buff_i=0; slot_mode_i=0; slot_ntgt_i=0;
        end
    endtask

    task set_slot;
        input integer s;
        input [MODE_SZ-1:0]      md;
        input [BUFF_INDX_SZ-1:0] bid;
        input [TGT_COUNT_SZ-1:0] nt;
        begin
            slot_mode_i[s*MODE_SZ      +: MODE_SZ]      = md;
            slot_buff_i[s*BUFF_INDX_SZ +: BUFF_INDX_SZ] = bid;
            slot_ntgt_i[s*TGT_COUNT_SZ +: TGT_COUNT_SZ] = nt;
        end
    endtask

    integer it, s;

    initial begin
        verif_errors=0; verif_checks=0;
        clear_inputs; reset=1'b1;
        repeat(3) begin model_step; @(posedge clk); #1; end
        reset=1'b0;
        $display("=== tb_acc_hw_buffer_tracker (scheduler) ===");
        m_free=1; m_buff=0; m_mode=0; m_ntgt=0;
        check_bit(acc_free_o, 1'b1, "post-reset acc_free");

        // ---- directed: dispatch latches slots, clears free ----
        clear_inputs;
        new_task_i=1;
        set_slot(0, 2'b01, 4'd3, 3'd0);
        set_slot(1, 2'b01, 4'd5, 3'd0);
        set_slot(3, 2'b11, 4'd7, 3'd1);
        step_and_check("D-newtask latch");
        // busy held while idle, slots retained
        clear_inputs;
        repeat(4) step_and_check("D-mid-task hold");
        // finish: free set, slots retained (replay-on-finish)
        clear_inputs; task_finished_i=1;
        step_and_check("D-finish free+retain");
        clear_inputs;
        step_and_check("D-post-finish idle");

        // ---- directed: second task overwrites latch ----
        clear_inputs;
        new_task_i=1;
        set_slot(4, 2'b10, 4'd12, 3'd2);
        set_slot(5, 2'b11, 4'd9,  3'd1);
        step_and_check("D-task2 latch");
        clear_inputs; task_finished_i=1; step_and_check("D-task2 finish");

        // ---- directed: same-cycle new_task + task_finished (new_task wins) ----
        clear_inputs;
        new_task_i=1; task_finished_i=1;
        set_slot(2, 2'b01, 4'd6, 3'd0);
        step_and_check("D-newtask+finish (busy wins)");
        clear_inputs; step_and_check("D-after collide");

        // ---- directed: finish with no outstanding task (free stays/returns 1) ----
        clear_inputs; task_finished_i=1; step_and_check("D-finish while free");

        // ---- constrained-random ----
        clear_inputs; reset=1'b1; model_step; @(posedge clk); #1; reset=1'b0;
        m_free=1; m_buff=0; m_mode=0; m_ntgt=0;

        void'($urandom(32'hACC0_7BAF));
        for (it=0; it<NRAND; it=it+1) begin
            clear_inputs;
            new_task_i      = ($urandom_range(0,99) < 25);
            task_finished_i = ($urandom_range(0,99) < 25);
            if (new_task_i)
                for (s=0; s<NUM_SLOTS; s=s+1)
                    set_slot(s, $urandom_range(0,3), $urandom_range(0,15),
                                $urandom_range(0,7));
            if ($urandom_range(0,199)==0) reset=1'b1;   // rare reset
            step_and_check("rand");
            reset=1'b0;
        end

        `VERIF_EPILOGUE("tb_acc_hw_buffer_tracker")
    end

    `VERIF_WATCHDOG(2000000)

endmodule
