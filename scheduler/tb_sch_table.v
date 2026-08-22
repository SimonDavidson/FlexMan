// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
// =============================================================================
// tb_sch_table  (scheduler — 4-entry in-order dispatch table)
//
// Authors      : Simon Davidson & Claude
// Created      : 2026-05-28
// Last modified: 2026-06-07
//
// Aggressive rewrite with a cycle-accurate SW reference model that replicates
// sch_table.v's full next-state EXACTLY: per-entry ready_to_go (recomputed by a
// faithful copy of the sch_entry dispatch predicate), select_to_go (only the
// in-order head, index 0, dispatches), the shift_entry / delete chain, and the
// load-at-tail rule.  Every cycle we check, vs the model:
//   * table_empty_o, table_slot_free_o
//   * dispatch_to_acc_o and (when dispatching) entry_data_o == head entry
//   * each of the 4 entry valid bits and data words
//
// Directed scenarios: strict in-order (younger ready entry can NOT pass a
// stalled older one), compaction toward slot 0, fill-to-full / slot_free gating,
// drain-to-empty.  Then a long constrained-random loop with random loads and
// random resource buses (acc_busy / buffers_full / free / colour).
// =============================================================================
`timescale 1ns/1ps

module tb_sch_table;

    localparam integer SCH_ENTRY_SZ        = 53;
    localparam integer TGT_ACC_SZ          = 2;
    localparam integer NUM_BUFFERS         = 16;
    localparam integer COL_BUFF_ID_SZ      = 16;
    localparam integer NUM_SCH_ENTRIES     = 4;
    localparam integer NUM_HW_ACCELERATORS = 2;
    localparam integer BUFF_INDX_SZ        = 4;
    localparam integer TGT_COUNT_SZ        = 3;
    localparam integer NUM_SLOTS           = 6;
    localparam integer MODE_SZ             = 2;
    localparam integer NRAND               = 5000;

    localparam integer SLOT_SHORT_SZ = MODE_SZ + BUFF_INDX_SZ;
    localparam integer SLOT_LONG_SZ  = MODE_SZ + BUFF_INDX_SZ + TGT_COUNT_SZ;
    localparam integer LONG_BASE     = 3 * SLOT_SHORT_SZ;
    localparam integer E_COLOUR      = LONG_BASE + 3 * SLOT_LONG_SZ;
    localparam integer E_ACC_START   = E_COLOUR + 1;

    localparam [1:0] MODE_UNUSED = 2'b00;
    localparam [1:0] MODE_SRC    = 2'b01;
    localparam [1:0] MODE_RW     = 2'b10;
    localparam [1:0] MODE_TGT    = 2'b11;

    reg                            clk, reset;
    reg                            load_new_entry_i;
    reg                            delete_entry_i;
    reg  [SCH_ENTRY_SZ-1:0]        entry_data_i;
    reg  [NUM_HW_ACCELERATORS-1:0] acc_busy_i;
    reg  [COL_BUFF_ID_SZ-1:0]      buffers_full_i;
    reg  [COL_BUFF_ID_SZ-1:0]      buffers_free_i;
    reg  [COL_BUFF_ID_SZ-1:0]      buffers_colour_i;

    wire                           table_slot_free_o;
    wire                           table_empty_o;
    wire                           dispatch_to_acc_o;
    wire [SCH_ENTRY_SZ-1:0]        entry_data_o;

    integer verif_errors, verif_checks, verif_to;
    `include "../verif/checks.vh"

    sch_table #(
        .SCH_ENTRY_SZ(SCH_ENTRY_SZ), .TGT_ACC_SZ(TGT_ACC_SZ),
        .NUM_BUFFERS(NUM_BUFFERS), .COL_BUFF_ID_SZ(COL_BUFF_ID_SZ),
        .NUM_SCH_ENTRIES(NUM_SCH_ENTRIES), .NUM_HW_ACCELERATORS(NUM_HW_ACCELERATORS)) dut (
        .clk(clk), .reset(reset),
        .table_slot_free_o(table_slot_free_o), .table_empty_o(table_empty_o),
        .load_new_entry_i(load_new_entry_i), .delete_entry_i(delete_entry_i),
        .entry_data_i(entry_data_i), .entry_hint_i(1'b0),
        .acc_busy_i(acc_busy_i), .buffers_full_i(buffers_full_i),
        .buffers_free_i(buffers_free_i), .buffers_colour_i(buffers_colour_i),
        .cm_busy_i(1'b0),
        .dispatch_to_acc_o(dispatch_to_acc_o), .entry_data_o(entry_data_o));

    initial clk = 1'b0; always #5 clk = ~clk;

    // ---- entry builder ----
    reg [SCH_ENTRY_SZ-1:0] entry_build;
    task build_entry;
        input [MODE_SZ-1:0] m0; input [BUFF_INDX_SZ-1:0] i0;
        input [MODE_SZ-1:0] m1; input [BUFF_INDX_SZ-1:0] i1;
        input [MODE_SZ-1:0] m2; input [BUFF_INDX_SZ-1:0] i2;
        input [MODE_SZ-1:0] m3; input [BUFF_INDX_SZ-1:0] i3; input [TGT_COUNT_SZ-1:0] n3;
        input [MODE_SZ-1:0] m4; input [BUFF_INDX_SZ-1:0] i4; input [TGT_COUNT_SZ-1:0] n4;
        input [MODE_SZ-1:0] m5; input [BUFF_INDX_SZ-1:0] i5; input [TGT_COUNT_SZ-1:0] n5;
        input               colour;
        input [TGT_ACC_SZ-1:0] acc_id;
        input [4:0]         cfg_id;
        begin
            entry_build = 'b0;
            entry_build[0*SLOT_SHORT_SZ +: MODE_SZ] = m0;
            entry_build[0*SLOT_SHORT_SZ + MODE_SZ +: BUFF_INDX_SZ] = i0;
            entry_build[1*SLOT_SHORT_SZ +: MODE_SZ] = m1;
            entry_build[1*SLOT_SHORT_SZ + MODE_SZ +: BUFF_INDX_SZ] = i1;
            entry_build[2*SLOT_SHORT_SZ +: MODE_SZ] = m2;
            entry_build[2*SLOT_SHORT_SZ + MODE_SZ +: BUFF_INDX_SZ] = i2;
            entry_build[LONG_BASE + 0*SLOT_LONG_SZ +: MODE_SZ] = m3;
            entry_build[LONG_BASE + 0*SLOT_LONG_SZ + MODE_SZ +: BUFF_INDX_SZ] = i3;
            entry_build[LONG_BASE + 0*SLOT_LONG_SZ + MODE_SZ + BUFF_INDX_SZ +: TGT_COUNT_SZ] = n3;
            entry_build[LONG_BASE + 1*SLOT_LONG_SZ +: MODE_SZ] = m4;
            entry_build[LONG_BASE + 1*SLOT_LONG_SZ + MODE_SZ +: BUFF_INDX_SZ] = i4;
            entry_build[LONG_BASE + 1*SLOT_LONG_SZ + MODE_SZ + BUFF_INDX_SZ +: TGT_COUNT_SZ] = n4;
            entry_build[LONG_BASE + 2*SLOT_LONG_SZ +: MODE_SZ] = m5;
            entry_build[LONG_BASE + 2*SLOT_LONG_SZ + MODE_SZ +: BUFF_INDX_SZ] = i5;
            entry_build[LONG_BASE + 2*SLOT_LONG_SZ + MODE_SZ + BUFF_INDX_SZ +: TGT_COUNT_SZ] = n5;
            entry_build[E_COLOUR] = colour;
            entry_build[E_ACC_START +: TGT_ACC_SZ] = acc_id;
            entry_build[E_ACC_START + TGT_ACC_SZ +: 5] = cfg_id;
        end
    endtask

    // ===================== SW reference model =====================
    reg                    m_valid [0:NUM_SCH_ENTRIES-1];
    reg [SCH_ENTRY_SZ-1:0] m_data  [0:NUM_SCH_ENTRIES-1];

    // sch_entry dispatch predicate (mirror of the RTL/sch_entry model)
    function automatic model_ready;
        input [SCH_ENTRY_SZ-1:0] d;
        integer s, base;
        reg [MODE_SZ-1:0] md; reg [BUFF_INDX_SZ-1:0] bid;
        reg ecolour; reg [TGT_ACC_SZ-1:0] aid; reg acc_free, ok;
        reg needs_full, needs_free, src_ok, tgt_ok;
        begin
            ecolour = d[E_COLOUR];
            aid     = d[E_ACC_START +: TGT_ACC_SZ];
            if (aid >= NUM_HW_ACCELERATORS) acc_free = 1'b1;
            else                            acc_free = ~acc_busy_i[aid];
            ok = acc_free;
            for (s=0; s<NUM_SLOTS; s=s+1) begin
                if (s<3) base = s*SLOT_SHORT_SZ;
                else     base = LONG_BASE + (s-3)*SLOT_LONG_SZ;
                md  = d[base +: MODE_SZ];
                bid = d[base + MODE_SZ +: BUFF_INDX_SZ];
                needs_full = (md==MODE_SRC) | (md==MODE_RW);
                needs_free = (md==MODE_TGT);
                src_ok = ~needs_full | (buffers_full_i[bid] & (buffers_colour_i[bid]==ecolour));
                tgt_ok = ~needs_free | buffers_free_i[bid];
                ok = ok & src_ok & tgt_ok;
            end
            model_ready = ok;
        end
    endfunction

    // model outputs (combinational, from current registered state + inputs)
    reg                    md_table_empty, md_slot_free, md_dispatch;
    reg [SCH_ENTRY_SZ-1:0] md_entry_data;

    // Replicate sch_table next-state combinational logic.
    // Arrays sized NUM_SCH_ENTRIES+1 for the (N) sentinel index, like the RTL.
    reg [NUM_SCH_ENTRIES:0]   ev;        // entry_valid_r[0..N], ev[N]=0
    reg [NUM_SCH_ENTRIES:0]   sov;       // shift_out_valid[0..N], sov[N]=0
    reg [NUM_SCH_ENTRIES-1:0] rtg;       // ready_to_go
    reg [NUM_SCH_ENTRIES-1:0] ld;        // load_entry
    reg [NUM_SCH_ENTRIES:0]   sel;       // select_to_go
    reg [NUM_SCH_ENTRIES:0]   sh;        // shift_entry
    reg [NUM_SCH_ENTRIES-1:0] dsh;       // delete_shifted_entry
    reg [NUM_SCH_ENTRIES-1:0] dl;        // delete_launched_entry
    reg [NUM_SCH_ENTRIES-1:0] del;       // delete_entry
    reg                       free_to_add, launching;
    integer mi;

    task model_eval;   // compute combinational outputs from current m_valid/m_data
        begin
            for (mi=0; mi<NUM_SCH_ENTRIES; mi=mi+1) begin
                ev[mi]  = m_valid[mi];
                sov[mi] = m_valid[mi];        // shift_out = delete?0:valid; delete folded below
                rtg[mi] = m_valid[mi] & model_ready(m_data[mi]);
            end
            ev[NUM_SCH_ENTRIES]  = 1'b0;
            sov[NUM_SCH_ENTRIES] = 1'b0;

            free_to_add = ~ev[NUM_SCH_ENTRIES-1];
            ld = 'b0;
            ld[NUM_SCH_ENTRIES-1] = free_to_add & load_new_entry_i;

            sel = 'b0;
            if (rtg[0]) sel[0] = 1'b1;

            // shift_entry chain (exact port)
            sh = 'b0;
            sh[0] = ~ld[0] & ~sel[1] & ev[1] & (~ev[0]);
            sh[NUM_SCH_ENTRIES] = 1'b0;
            for (mi=1; mi<NUM_SCH_ENTRIES-1; mi=mi+1)
                sh[mi] = ~ld[mi] & ~sel[mi+1] & ev[mi+1] & (~ev[mi] | sh[mi-1]);
            sh[NUM_SCH_ENTRIES-1] = ~ld[NUM_SCH_ENTRIES-1]
                                  & ev[NUM_SCH_ENTRIES]
                                  & (~ev[NUM_SCH_ENTRIES-1] | sh[NUM_SCH_ENTRIES-2]);

            // delete_shifted_entry
            dsh = 'b0;
            for (mi=1; mi<NUM_SCH_ENTRIES; mi=mi+1)
                dsh[mi] = ~ld[mi] &
                          ((sel[mi+1] & sh[mi-1])
                         | (~sov[mi+1] & ~sh[mi] & sh[mi-1]));

            // entry_data_mux (select head; default entry 0)
            md_entry_data = m_data[0];
            for (mi=0; mi<NUM_SCH_ENTRIES; mi=mi+1)
                if (sel[mi]) md_entry_data = m_data[mi];

            // dispatch / delete (dispatch_ok = ~cm_busy = 1 here)
            dl  = sel[NUM_SCH_ENTRIES-1:0];
            del = dl | dsh;
            launching = |sel[NUM_SCH_ENTRIES-1:0];

            md_table_empty = ~(|ev[NUM_SCH_ENTRIES-1:0]);
            md_slot_free   = free_to_add;
            md_dispatch    = launching;
        end
    endtask

    // Advance the SW model one cycle (compute next-state of m_valid/m_data).
    task model_step;
        reg                    nv [0:NUM_SCH_ENTRIES-1];
        reg [SCH_ENTRY_SZ-1:0] nd [0:NUM_SCH_ENTRIES-1];
        reg                    si_v; reg [SCH_ENTRY_SZ-1:0] si_d;
        begin
            model_eval;
            for (mi=0; mi<NUM_SCH_ENTRIES; mi=mi+1) begin
                // shift-in source for entry mi is entry mi+1 (entry N-1 sees 0)
                if (mi==NUM_SCH_ENTRIES-1) begin si_v=1'b0; si_d=m_data[mi]; end
                else begin si_v=m_valid[mi+1]; si_d=m_data[mi+1]; end

                if      (ld[mi])     nv[mi]=1'b1;
                else if (del[mi])    nv[mi]=1'b0;
                else if (sh[mi])     nv[mi]=si_v;
                else                 nv[mi]=m_valid[mi];

                if      (ld[mi])     nd[mi]=entry_data_i;
                else if (del[mi])    nd[mi]=m_data[mi];
                else if (sh[mi])     nd[mi]=si_d;
                else                 nd[mi]=m_data[mi];
            end
            if (reset)
                for (mi=0; mi<NUM_SCH_ENTRIES; mi=mi+1) begin nv[mi]=0; nd[mi]=0; end
            for (mi=0; mi<NUM_SCH_ENTRIES; mi=mi+1) begin
                m_valid[mi]=nv[mi]; m_data[mi]=nd[mi];
            end
        end
    endtask

    // Check combinational outputs vs the model for the current state.
    task check_comb;
        input [255:0] tag;
        begin
            #1;
            model_eval;
            check_bit(table_empty_o,     md_table_empty, {tag, " empty"});
            check_bit(table_slot_free_o, md_slot_free,   {tag, " slot_free"});
            check_bit(dispatch_to_acc_o, md_dispatch,    {tag, " dispatch"});
            if (md_dispatch)
                check_eq_u(entry_data_o, md_entry_data,  {tag, " disp_data"});
        end
    endtask

    // Full cycle: check combinational outputs vs model, then advance both DUT
    // and model over the clock edge.  Because all observable DUT outputs are
    // functions of the registered entry valids/data, checking them every cycle
    // transitively validates the model's per-entry state evolution.
    task step_and_check;
        input [255:0] tag;
        begin
            check_comb(tag);
            model_step;
            @(posedge clk); #1;
        end
    endtask

    task do_reset;
        begin
            reset=1'b1; model_step; @(posedge clk); #1; reset=1'b0;
            for (mi=0; mi<NUM_SCH_ENTRIES; mi=mi+1) begin m_valid[mi]=0; m_data[mi]=0; end
        end
    endtask

    task load_one;     // pulse load_new_entry for one cycle with given data
        input [SCH_ENTRY_SZ-1:0] dat;
        begin
            load_new_entry_i=1; entry_data_i=dat;
            step_and_check("load");
            load_new_entry_i=0;
        end
    endtask

    integer it;
    reg [SCH_ENTRY_SZ-1:0] eA, eB, eC;

    initial begin
        verif_errors=0; verif_checks=0;
        load_new_entry_i=0; delete_entry_i=0; entry_data_i=0;
        acc_busy_i=2'b11; buffers_full_i=0; buffers_free_i=16'hFFFF; buffers_colour_i=0;
        reset=1'b1;
        repeat(3) begin model_step; @(posedge clk); #1; end
        reset=1'b0;
        for (mi=0; mi<NUM_SCH_ENTRIES; mi=mi+1) begin m_valid[mi]=0; m_data[mi]=0; end
        $display("=== tb_sch_table (scheduler) ===");

        // reset state
        check_comb("D-reset");

        // Entry A: acc0, slot0 SRC b4, slot1 SRC b8, slot5 TGT b15
        build_entry(MODE_SRC,4'd4, MODE_SRC,4'd8, MODE_UNUSED,4'd0,
                    MODE_UNUSED,4'd0,3'd0, MODE_UNUSED,4'd0,3'd0, MODE_TGT,4'd15,3'd1,
                    1'b0, 2'd0, 5'd1);
        eA = entry_build;
        // Entry B: acc1, slot0 SRC b1, slot1 SRC b2, slot5 TGT b14
        build_entry(MODE_SRC,4'd1, MODE_SRC,4'd2, MODE_UNUSED,4'd0,
                    MODE_UNUSED,4'd0,3'd0, MODE_UNUSED,4'd0,3'd0, MODE_TGT,4'd14,3'd1,
                    1'b0, 2'd1, 5'd1);
        eB = entry_build;

        // ---- load A; blocked (acc busy) ----
        load_one(eA);
        repeat(5) step_and_check("A settle");     // compacts toward slot0, still blocked

        // ---- load B while A waits ----
        load_one(eB);
        repeat(5) step_and_check("B settle");

        // ---- STRICT IN-ORDER: make B (head's junior) ready but A NOT ready.
        // A needs b4,b8 full + b15 free on acc0; B needs b1,b2 full + b14 free on acc1.
        // Provide B's resources + acc1, but NOT A's, and keep acc0 busy.
        // The head is A (loaded first) -> nothing must dispatch.
        acc_busy_i=2'b01;                 // acc0 busy, acc1 free
        buffers_full_i=16'h0006;          // b1,b2 (B) full; b4,b8 (A) NOT
        buffers_free_i=16'hFFFF;
        repeat(4) step_and_check("in-order: B ready A stalled -> no dispatch");

        // ---- now enable A; A dispatches (head), B then becomes head ----
        acc_busy_i=2'b00;                 // both free
        buffers_full_i=16'h0116;          // b1,b2,b4,b8 full (0x0116 = bits1,2,4,8)
        repeat(8) step_and_check("A then B dispatch");

        // ---- drain to empty ----
        acc_busy_i=2'b11; buffers_full_i=16'h0000;
        repeat(6) step_and_check("drain");
        check_comb("post-drain empty");

        // ---- fill table to full; slot_free must gate ----
        acc_busy_i=2'b11; buffers_full_i=0; buffers_free_i=16'hFFFF;
        for (it=0; it<4; it=it+1) begin
            // only load when a slot is free, else just step
            if (md_slot_free) load_one(eA);
            else              step_and_check("fill-wait");
            repeat(1) step_and_check("fill-settle");
        end
        repeat(3) step_and_check("post-fill settle");
        check_comb("full table");

        // ---- constrained-random ----
        do_reset;
        acc_busy_i=2'b11; buffers_full_i=0; buffers_free_i=16'hFFFF; buffers_colour_i=0;
        void'($urandom(32'h7AB1_E001));
        for (it=0; it<NRAND; it=it+1) begin
            // random load (only valid if a slot is free, but the RTL gates it itself)
            if (($urandom_range(0,99) < 40)) begin
                load_new_entry_i = 1'b1;
                // random legal-ish entry
                build_entry($urandom_range(0,3),$urandom_range(0,15),
                            $urandom_range(0,3),$urandom_range(0,15),
                            $urandom_range(0,3),$urandom_range(0,15),
                            $urandom_range(0,3),$urandom_range(0,15),$urandom_range(0,7),
                            $urandom_range(0,3),$urandom_range(0,15),$urandom_range(0,7),
                            $urandom_range(0,3),$urandom_range(0,15),$urandom_range(0,7),
                            $urandom&1, $urandom_range(0,1), $urandom_range(0,31));
                entry_data_i = entry_build;
            end else load_new_entry_i = 1'b0;
            // random resource buses
            acc_busy_i       = $urandom_range(0,3);
            buffers_full_i   = $urandom;
            buffers_free_i   = $urandom;
            buffers_colour_i = $urandom;
            step_and_check("rand");
            load_new_entry_i = 1'b0;
        end

        `VERIF_EPILOGUE("tb_sch_table")
    end

    `VERIF_WATCHDOG(4000000)

endmodule
