// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
// =============================================================================
// tb_sch_buffer_state  (scheduler — buffer + accelerator state aggregator)
//
// Authors      : Simon Davidson & Claude
// Created      : 2026-05-12
// Last modified: 2026-06-07
//
// Aggressive rewrite with a full structural SW reference model of
// sch_buffer_state.v: the per-accelerator trackers, the per-buffer
// {free,full,colour,count} entries, the pending-completion queue, and the
// lowest-index-finisher priority encoder.  The model reproduces the EXACT
// timing: completion is queued on acc_finished, then on the NEXT edge the
// selected finisher drives the buffer cleanup strobes (consume-on-completion,
// refill RW/TGT slots full), so visible buffer state changes land +2 edges
// after acc_finished is sampled.
//
// Every cycle the model is advanced in lockstep with the DUT and ALL outputs
// (buffers_full/free/colour, acc_available, target_status) are compared.
// Directed: mark_as_full, dispatch (TGT/RW), single + multi-consumer expiry,
// RW claim->busy->refill, simultaneous finishes (priority), back-to-back tasks.
// Then a long constrained-random loop.
// =============================================================================
`timescale 1ns/1ps

module tb_sch_buffer_state;

    localparam integer NUM_BUFFERS         = 16;
    localparam integer NUM_HW_ACCELERATORS = 2;
    localparam integer BUFF_INDX_SZ        = 4;
    localparam integer TGT_ACC_SZ          = 2;
    localparam integer TGT_COUNT_SZ        = 3;
    localparam integer NUM_SLOTS           = 6;
    localparam integer MODE_SZ             = 2;
    localparam integer NRAND               = 5000;

    localparam [1:0] MODE_UNUSED = 2'b00;
    localparam [1:0] MODE_SRC    = 2'b01;
    localparam [1:0] MODE_RW     = 2'b10;
    localparam [1:0] MODE_TGT    = 2'b11;

    reg                                clk, reset;
    reg [NUM_HW_ACCELERATORS-1:0]      acc_busy_i;
    reg [NUM_HW_ACCELERATORS-1:0]      acc_finished_i;
    reg [NUM_HW_ACCELERATORS-1:0]      acc_result_i;
    reg                                mark_buff_as_full_i;
    reg [BUFF_INDX_SZ-1:0]             full_buff_id_i;
    reg [TGT_COUNT_SZ-1:0]             full_buff_usage_i;
    reg                                start_new_task_i;
    reg [TGT_ACC_SZ-1:0]               tgt_acc_id_i;
    reg [NUM_SLOTS*BUFF_INDX_SZ-1:0]   slot_buff_i;
    reg [NUM_SLOTS*MODE_SZ-1:0]        slot_mode_i;
    reg [NUM_SLOTS*TGT_COUNT_SZ-1:0]   slot_ntgt_i;
    reg                                tgt_colour_i;

    wire [NUM_HW_ACCELERATORS-1:0]     acc_available_o;
    wire [NUM_BUFFERS-1:0]             buffers_full_o;
    wire [NUM_BUFFERS-1:0]             buffers_free_o;
    wire [NUM_BUFFERS-1:0]             buffers_colour_o;
    wire [NUM_BUFFERS-1:0]             target_status_o;

    integer verif_errors, verif_checks, verif_to;
    `include "../verif/checks.vh"

    sch_buffer_state #(
        .NUM_BUFFERS(NUM_BUFFERS), .NUM_HW_ACCELERATORS(NUM_HW_ACCELERATORS),
        .TGT_ACC_SZ(TGT_ACC_SZ), .TGT_COUNT_SZ(TGT_COUNT_SZ),
        .NUM_SLOTS(NUM_SLOTS), .MODE_SZ(MODE_SZ)) dut (
        .clk(clk), .reset(reset),
        .acc_busy_i(acc_busy_i), .acc_finished_i(acc_finished_i), .acc_result_i(acc_result_i),
        .acc_ready_next_i({NUM_HW_ACCELERATORS{1'b0}}),
        .mark_buff_as_full_i(mark_buff_as_full_i), .full_buff_id_i(full_buff_id_i),
        .full_buff_usage_i(full_buff_usage_i),
        .start_new_task_i(start_new_task_i), .tgt_acc_id_i(tgt_acc_id_i),
        .slot_buff_i(slot_buff_i), .slot_mode_i(slot_mode_i), .slot_ntgt_i(slot_ntgt_i),
        .tgt_colour_i(tgt_colour_i),
        .acc_available_o(acc_available_o),
        .buffers_full_o(buffers_full_o), .buffers_free_o(buffers_free_o),
        .buffers_colour_o(buffers_colour_o), .target_status_o(target_status_o));

    initial clk = 1'b0; always #5 clk = ~clk;

    // ===================== SW reference model =====================
    // per-buffer entry state
    reg                     bf_free   [0:NUM_BUFFERS-1];
    reg                     bf_full   [0:NUM_BUFFERS-1];
    reg                     bf_colour [0:NUM_BUFFERS-1];
    reg [TGT_COUNT_SZ-1:0]  bf_count  [0:NUM_BUFFERS-1];
    // per-acc tracker state
    reg                     tr_free   [0:NUM_HW_ACCELERATORS-1];
    reg [NUM_SLOTS*BUFF_INDX_SZ-1:0] tr_buff [0:NUM_HW_ACCELERATORS-1];
    reg [NUM_SLOTS*MODE_SZ-1:0]      tr_mode [0:NUM_HW_ACCELERATORS-1];
    reg [NUM_SLOTS*TGT_COUNT_SZ-1:0] tr_ntgt [0:NUM_HW_ACCELERATORS-1];
    // pending-completion queue + target_status
    reg [NUM_HW_ACCELERATORS-1:0] m_pending;
    reg [NUM_BUFFERS-1:0]         m_tstatus;

    integer b, a, s2;

    // Advance the SW model one cycle.  Order matches RTL combinational eval
    // (uses CURRENT registered state) followed by the registered update.
    task model_step;
        // combinational intermediates
        reg [NUM_HW_ACCELERATORS-1:0] sel_fin;
        reg [TGT_ACC_SZ-1:0]          sel_idx;
        reg                           found, cleanup;
        reg [NUM_BUFFERS-1:0]         s_rwclaim, s_newtgt, s_markfull, s_nowfull, s_consumed;
        reg [TGT_COUNT_SZ-1:0]        s_nowntgt [0:NUM_BUFFERS-1];
        reg [NUM_HW_ACCELERATORS-1:0] new_task_acc;
        reg [BUFF_INDX_SZ-1:0]        bid; reg [MODE_SZ-1:0] md;
        reg [TGT_COUNT_SZ-1:0]        nt;
        // per-buffer next state
        reg nfree, nfull, ncol; reg [TGT_COUNT_SZ-1:0] ncnt; reg nln;
        // tracker next
        reg ntr_free;
        begin
            // ---- priority-encode lowest-index pending finisher ----
            sel_fin='b0; sel_idx='b0; found=1'b0;
            for (a=0; a<NUM_HW_ACCELERATORS; a=a+1)
                if (!found && m_pending[a]) begin
                    sel_fin[a]=1'b1; sel_idx=a[TGT_ACC_SZ-1:0]; found=1'b1;
                end
            cleanup = |sel_fin;

            // ---- external mark-as-full one-hot ----
            s_markfull='b0;
            if (mark_buff_as_full_i) s_markfull[full_buff_id_i]=1'b1;

            // ---- dispatch decode ----
            s_rwclaim='b0; s_newtgt='b0; new_task_acc='b0;
            if (start_new_task_i) begin
                new_task_acc[tgt_acc_id_i]=1'b1;
                for (s2=0; s2<NUM_SLOTS; s2=s2+1) begin
                    bid = slot_buff_i[s2*BUFF_INDX_SZ +: BUFF_INDX_SZ];
                    md  = slot_mode_i[s2*MODE_SZ      +: MODE_SZ];
                    if (md==MODE_RW)  s_rwclaim[bid]=1'b1;
                    if (md==MODE_TGT) s_newtgt[bid] =1'b1;
                end
            end

            // ---- completion decode (from selected finisher's tracker) ----
            s_nowfull='b0; s_consumed='b0;
            for (b=0; b<NUM_BUFFERS; b=b+1) s_nowntgt[b]='b0;
            if (cleanup) begin
                for (s2=0; s2<NUM_SLOTS; s2=s2+1) begin
                    bid = tr_buff[sel_idx][s2*BUFF_INDX_SZ +: BUFF_INDX_SZ];
                    md  = tr_mode[sel_idx][s2*MODE_SZ      +: MODE_SZ];
                    nt  = tr_ntgt[sel_idx][s2*TGT_COUNT_SZ +: TGT_COUNT_SZ];
                    if (md==MODE_RW || md==MODE_TGT) begin
                        s_nowfull[bid]=1'b1; s_nowntgt[bid]=nt;
                    end else if (md==MODE_SRC)
                        s_consumed[bid]=1'b1;
                end
            end

            // ---- per-buffer entry next-state (exact priority chains) ----
            for (b=0; b<NUM_BUFFERS; b=b+1) begin
                nln = (bf_count[b]=='b1) & s_consumed[b];
                // free
                if      (reset)            nfree=1'b1;
                else if (s_rwclaim[b])     nfree=1'b0;
                else if (s_markfull[b])    nfree=1'b0;
                else if (nln)              nfree=1'b1;
                else if (s_newtgt[b])      nfree=1'b0;
                else                       nfree=bf_free[b];
                // full
                if      (reset)                      nfull=1'b0;
                else if (s_rwclaim[b])               nfull=1'b0;
                else if (s_nowfull[b]|s_markfull[b]) nfull=1'b1;
                else if (nln)                        nfull=1'b0;
                else                                 nfull=bf_full[b];
                // colour
                if      (reset)        ncol=1'b0;
                else if (s_rwclaim[b]) ncol=tgt_colour_i;
                else if (s_newtgt[b])  ncol=tgt_colour_i;
                else                   ncol=bf_colour[b];
                // count  (new_usage_count is hardwired 0 in the instantiation)
                if      (reset)         ncnt='b0;
                else if (s_markfull[b]) ncnt=full_buff_usage_i;
                else if (s_nowfull[b])  ncnt=s_nowntgt[b];
                else if (s_newtgt[b])   ncnt={TGT_COUNT_SZ{1'b0}};
                else if (s_consumed[b]) ncnt=bf_count[b]-'b1;
                else                    ncnt=bf_count[b];
                bf_free[b]=nfree; bf_full[b]=nfull; bf_colour[b]=ncol; bf_count[b]=ncnt;
            end

            // ---- tracker next-state ----
            for (a=0; a<NUM_HW_ACCELERATORS; a=a+1) begin
                if      (reset)            ntr_free=1'b1;
                else if (new_task_acc[a])  ntr_free=1'b0;
                else if (sel_fin[a])       ntr_free=1'b1;
                else                       ntr_free=tr_free[a];
                if (reset) begin
                    tr_buff[a]='b0; tr_mode[a]='b0; tr_ntgt[a]='b0;
                end else if (new_task_acc[a]) begin
                    tr_buff[a]=slot_buff_i; tr_mode[a]=slot_mode_i; tr_ntgt[a]=slot_ntgt_i;
                end
                tr_free[a]=ntr_free;
            end

            // ---- target_status_r next-state ----
            if (reset) m_tstatus='b0;
            else if (cleanup) begin
                for (s2=0; s2<NUM_SLOTS; s2=s2+1) begin
                    bid = tr_buff[sel_idx][s2*BUFF_INDX_SZ +: BUFF_INDX_SZ];
                    md  = tr_mode[sel_idx][s2*MODE_SZ      +: MODE_SZ];
                    if (md==MODE_TGT || md==MODE_RW)
                        m_tstatus[bid] = acc_result_i[sel_idx];
                end
            end

            // ---- pending queue next-state ----
            if (reset) m_pending='b0;
            else       m_pending=(m_pending | acc_finished_i) & ~sel_fin;
        end
    endtask

    // pack model buffer/acc state into bit vectors for comparison
    function [NUM_BUFFERS-1:0] m_full_vec;
        integer b2; begin m_full_vec=0;
            for (b2=0;b2<NUM_BUFFERS;b2=b2+1) m_full_vec[b2]=bf_full[b2]; end
    endfunction
    function [NUM_BUFFERS-1:0] m_free_vec;
        integer b2; begin m_free_vec=0;
            for (b2=0;b2<NUM_BUFFERS;b2=b2+1) m_free_vec[b2]=bf_free[b2]; end
    endfunction
    function [NUM_BUFFERS-1:0] m_col_vec;
        integer b2; begin m_col_vec=0;
            for (b2=0;b2<NUM_BUFFERS;b2=b2+1) m_col_vec[b2]=bf_colour[b2]; end
    endfunction
    function [NUM_HW_ACCELERATORS-1:0] m_acc_vec;
        integer a2; begin m_acc_vec=0;
            for (a2=0;a2<NUM_HW_ACCELERATORS;a2=a2+1) m_acc_vec[a2]=tr_free[a2]; end
    endfunction

    task step_and_check;
        input [255:0] tag;
        begin
            model_step;
            @(posedge clk); #1;
            check_eq_u(buffers_full_o,   m_full_vec(), {tag, " full"});
            check_eq_u(buffers_free_o,   m_free_vec(), {tag, " free"});
            check_eq_u(buffers_colour_o, m_col_vec(),  {tag, " colour"});
            check_eq_u(acc_available_o,  m_acc_vec(),  {tag, " acc_avail"});
            check_eq_u(target_status_o,  m_tstatus,    {tag, " tstatus"});
        end
    endtask

    task clear_inputs;
        begin
            acc_finished_i=0; acc_result_i=0;
            mark_buff_as_full_i=0; full_buff_id_i=0; full_buff_usage_i=0;
            start_new_task_i=0; tgt_acc_id_i=0; tgt_colour_i=0;
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
        clear_inputs; acc_busy_i=0; reset=1'b1;
        repeat(3) begin model_step; @(posedge clk); #1; end
        reset=1'b0;
        // sync model to reset state
        for (b=0;b<NUM_BUFFERS;b=b+1) begin
            bf_free[b]=1; bf_full[b]=0; bf_colour[b]=0; bf_count[b]=0;
        end
        for (a=0;a<NUM_HW_ACCELERATORS;a=a+1) begin
            tr_free[a]=1; tr_buff[a]=0; tr_mode[a]=0; tr_ntgt[a]=0;
        end
        m_pending=0; m_tstatus=0;
        $display("=== tb_sch_buffer_state (scheduler) ===");

        // reset checks
        clear_inputs; step_and_check("D-reset state");

        // ---- mark_as_full buff3, 2 consumers ----
        clear_inputs; mark_buff_as_full_i=1; full_buff_id_i=4'd3; full_buff_usage_i=3'd2;
        step_and_check("D-markfull b3");
        clear_inputs; step_and_check("D-after markfull");

        // ---- dispatch on acc0: SRC b3, TGT b5 (ntgt=2) ----
        clear_inputs; start_new_task_i=1; tgt_acc_id_i=2'd0; tgt_colour_i=1'b0;
        set_slot(0, MODE_SRC, 4'd3, 3'd0);
        set_slot(3, MODE_TGT, 4'd5, 3'd2);
        step_and_check("D-dispatch acc0");
        clear_inputs; step_and_check("D-after dispatch");

        // ---- acc0 finishes -> +1 queue, +1 cleanup ----
        clear_inputs; acc_finished_i=2'b01; acc_result_i=2'b00;
        step_and_check("D-acc0 finish (queue)");
        clear_inputs; step_and_check("D-acc0 cleanup");     // buff5 full, b3 count 2->1
        clear_inputs; step_and_check("D-acc0 settled");

        // ---- dispatch on acc1: SRC b3 (last consumer), SRC b5, TGT b6 (ntgt=1) ----
        clear_inputs; start_new_task_i=1; tgt_acc_id_i=2'd1; tgt_colour_i=1'b0;
        set_slot(0, MODE_SRC, 4'd3, 3'd0);
        set_slot(1, MODE_SRC, 4'd5, 3'd0);
        set_slot(3, MODE_TGT, 4'd6, 3'd1);
        step_and_check("D-dispatch acc1");
        clear_inputs; step_and_check("D-after dispatch2");
        clear_inputs; acc_finished_i=2'b10;
        step_and_check("D-acc1 finish (queue)");
        clear_inputs; step_and_check("D-acc1 cleanup");     // b3 freed, b5 2->1, b6 full
        clear_inputs; step_and_check("D-acc1 settled");

        // ---- RW: mark b7 full(1), dispatch RW b7(->2)+TGT b8, finish ----
        clear_inputs; mark_buff_as_full_i=1; full_buff_id_i=4'd7; full_buff_usage_i=3'd1;
        step_and_check("D-RW markfull b7");
        clear_inputs; start_new_task_i=1; tgt_acc_id_i=2'd0; tgt_colour_i=1'b1;
        set_slot(4, MODE_RW,  4'd7, 3'd2);
        set_slot(5, MODE_TGT, 4'd8, 3'd1);
        step_and_check("D-RW dispatch (b7 busy)");
        clear_inputs; step_and_check("D-RW dispatched");
        clear_inputs; acc_finished_i=2'b01;
        step_and_check("D-RW finish queue");
        clear_inputs; step_and_check("D-RW cleanup (b7 full again, b8 full)");
        clear_inputs; step_and_check("D-RW settled");

        // ---- simultaneous finishes: queue both, priority encoder drains
        //      lowest index first across successive cycles ----
        // set up two independent target buffers via two dispatches
        clear_inputs; start_new_task_i=1; tgt_acc_id_i=2'd0; tgt_colour_i=1'b0;
        set_slot(3, MODE_TGT, 4'd10, 3'd1);
        step_and_check("D-simul setup acc0");
        clear_inputs; start_new_task_i=1; tgt_acc_id_i=2'd1; tgt_colour_i=1'b1;
        set_slot(3, MODE_TGT, 4'd11, 3'd1);
        step_and_check("D-simul setup acc1");
        clear_inputs; acc_finished_i=2'b11;             // both finish same cycle
        step_and_check("D-both finish (queue both)");
        clear_inputs; step_and_check("D-drain acc0 first");
        clear_inputs; step_and_check("D-drain acc1 next");
        clear_inputs; step_and_check("D-simul settled");

        // ---- result propagation to target_status (acc_result=1) ----
        clear_inputs; start_new_task_i=1; tgt_acc_id_i=2'd0; tgt_colour_i=1'b0;
        set_slot(3, MODE_TGT, 4'd12, 3'd1);
        step_and_check("D-tstatus dispatch");
        clear_inputs; acc_finished_i=2'b01; acc_result_i=2'b01;
        step_and_check("D-tstatus finish");
        clear_inputs; step_and_check("D-tstatus cleanup (b12 result=1)");

        // ---- constrained-random ----
        clear_inputs; reset=1'b1; model_step; @(posedge clk); #1; reset=1'b0;
        for (b=0;b<NUM_BUFFERS;b=b+1) begin bf_free[b]=1;bf_full[b]=0;bf_colour[b]=0;bf_count[b]=0; end
        for (a=0;a<NUM_HW_ACCELERATORS;a=a+1) begin tr_free[a]=1;tr_buff[a]=0;tr_mode[a]=0;tr_ntgt[a]=0; end
        m_pending=0; m_tstatus=0;

        void'($urandom(32'h5B5F_5701));
        for (it=0; it<NRAND; it=it+1) begin
            clear_inputs;
            // dispatch only on a tracker-free acc (mirrors scheduler usage; keeps
            // tracker latch meaningful). Pick a random free acc if any.
            if (($urandom_range(0,99) < 35)) begin
                a = $urandom_range(0,NUM_HW_ACCELERATORS-1);
                if (tr_free[a]) begin
                    start_new_task_i=1; tgt_acc_id_i=a[TGT_ACC_SZ-1:0];
                    tgt_colour_i=$urandom&1;
                    for (s=0;s<NUM_SLOTS;s=s+1)
                        set_slot(s, $urandom_range(0,3), $urandom_range(0,15),
                                    $urandom_range(0,7));
                end
            end
            // finish only accs that are busy in the model (realistic)
            for (a=0;a<NUM_HW_ACCELERATORS;a=a+1)
                if (!tr_free[a] && ($urandom_range(0,99) < 30)) begin
                    acc_finished_i[a]=1'b1;
                    acc_result_i[a]=$urandom&1;
                end
            // occasional external mark-as-full
            if ($urandom_range(0,99) < 10) begin
                mark_buff_as_full_i=1;
                full_buff_id_i=$urandom_range(0,15);
                full_buff_usage_i=$urandom_range(1,7);
            end
            step_and_check("rand");
        end

        `VERIF_EPILOGUE("tb_sch_buffer_state")
    end

    `VERIF_WATCHDOG(4000000)

endmodule
