// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
// =============================================================================
// tb_sch_entry  (scheduler — one table slot: hold/shift/delete + dispatch-ready)
//
// Authors      : Simon Davidson & Claude
// Created      : 2026-05-12
// Last modified: 2026-06-07
//
// Aggressive rewrite with a SW reference model of:
//   (a) the entry storage FSM:
//         valid_nxt = load ? 1 : delete ? 0 : shift ? shift_in_valid : valid
//         data_nxt  = load ? new_data : delete ? data : shift ? shift_in_data : data
//         shift_out_valid = delete ? 0 : valid
//   (b) the combinational dispatch-ready predicate, recomputed in SW from the
//       stored entry + driven resource buses, per the exact RTL priority:
//         SOURCE/RW slot: buffer full AND colour==entry_colour
//         TARGET slot:    buffer free
//         plus acc_free (the required accelerator bit not busy)
//
// Directed coverage of every readiness term (acc busy, src not full, colour
// mismatch on SRC and on RW, tgt busy, RW-needs-no-free invariant) PLUS a
// constrained-random loop: random entries loaded/shifted/deleted and random
// resource buses, checking entry_valid / entry_data / shift_out_valid every
// cycle and ready_to_execute combinationally against the model.
// =============================================================================
`timescale 1ns/1ps

module tb_sch_entry;

    localparam integer SCH_ENTRY_SZ        = 53;
    localparam integer NUM_HW_ACCELERATORS = 2;
    localparam integer TGT_ACC_SZ          = 2;
    localparam integer NUM_BUFFERS         = 16;
    localparam integer BUFF_INDX_SZ        = 4;
    localparam integer TGT_COUNT_SZ        = 3;
    localparam integer NUM_SLOTS           = 6;
    localparam integer MODE_SZ             = 2;
    localparam integer NRAND               = 6000;

    localparam integer SLOT_SHORT_SZ = MODE_SZ + BUFF_INDX_SZ;                 // 6
    localparam integer SLOT_LONG_SZ  = MODE_SZ + BUFF_INDX_SZ + TGT_COUNT_SZ;  // 9
    localparam integer LONG_BASE     = 3 * SLOT_SHORT_SZ;                      // 18
    localparam integer E_COLOUR      = LONG_BASE + 3 * SLOT_LONG_SZ;           // 45
    localparam integer E_ACC_START   = E_COLOUR + 1;                           // 46

    localparam [1:0] MODE_UNUSED = 2'b00;
    localparam [1:0] MODE_SRC    = 2'b01;
    localparam [1:0] MODE_RW     = 2'b10;
    localparam [1:0] MODE_TGT    = 2'b11;

    reg                            clk, reset;
    reg                            load_new_entry_i;
    reg                            shift_entry_i;
    reg                            delete_entry_i;
    reg                            new_entry_valid_i;
    reg  [SCH_ENTRY_SZ-1:0]        new_entry_data_i;
    reg                            shift_in_entry_valid_i;
    reg  [SCH_ENTRY_SZ-1:0]        shift_in_entry_data_i;
    reg  [NUM_HW_ACCELERATORS-1:0] acc_busy_i;
    reg  [NUM_BUFFERS-1:0]         buffers_full_i;
    reg  [NUM_BUFFERS-1:0]         buffers_free_i;
    reg  [NUM_BUFFERS-1:0]         buffers_colour_i;

    wire                           shift_out_entry_valid_o;
    wire                           entry_valid_o;
    wire [SCH_ENTRY_SZ-1:0]        entry_data_o;
    wire                           ready_to_execute_o;

    integer verif_errors, verif_checks, verif_to;
    `include "../verif/checks.vh"

    sch_entry #(
        .SCH_ENTRY_SZ(SCH_ENTRY_SZ), .NUM_HW_ACCELERATORS(NUM_HW_ACCELERATORS),
        .TGT_ACC_SZ(TGT_ACC_SZ), .NUM_BUFFERS(NUM_BUFFERS),
        .BUFF_INDX_SZ(BUFF_INDX_SZ), .TGT_COUNT_SZ(TGT_COUNT_SZ),
        .NUM_SLOTS(NUM_SLOTS), .MODE_SZ(MODE_SZ)) dut (
        .clk(clk), .reset(reset),
        .load_new_entry_i(load_new_entry_i), .shift_entry_i(shift_entry_i),
        .delete_entry_i(delete_entry_i), .new_entry_valid_i(new_entry_valid_i),
        .new_entry_data_i(new_entry_data_i), .new_entry_hint_i(1'b0),
        .shift_in_entry_valid_i(shift_in_entry_valid_i),
        .shift_in_entry_data_i(shift_in_entry_data_i), .shift_in_entry_hint_i(1'b0),
        .shift_out_entry_valid_o(shift_out_entry_valid_o),
        .entry_valid_o(entry_valid_o), .entry_data_o(entry_data_o), .entry_hint_o(),
        .acc_busy_i(acc_busy_i), .buffers_full_i(buffers_full_i),
        .buffers_free_i(buffers_free_i), .buffers_colour_i(buffers_colour_i),
        .ready_to_execute_o(ready_to_execute_o));

    initial clk = 1'b0; always #5 clk = ~clk;

    // ---- entry builder (matches RTL slot bit layout) ----
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

    // ---- SW model: stored entry state ----
    reg                     m_valid;
    reg [SCH_ENTRY_SZ-1:0]  m_data;

    task model_step;
        reg nv;
        reg [SCH_ENTRY_SZ-1:0] nd;
        begin
            if      (load_new_entry_i) nv = 1'b1;
            else if (delete_entry_i)   nv = 1'b0;
            else if (shift_entry_i)    nv = shift_in_entry_valid_i;
            else                       nv = m_valid;

            if      (load_new_entry_i) nd = new_entry_data_i;
            else if (delete_entry_i)   nd = m_data;
            else if (shift_entry_i)    nd = shift_in_entry_data_i;
            else                       nd = m_data;

            if (reset) begin nv = 1'b0; nd = 'b0; end
            m_valid = nv; m_data = nd;
        end
    endtask

    // ---- SW model: combinational dispatch-ready predicate from stored entry ----
    function automatic model_ready;
        input                          v;
        input [SCH_ENTRY_SZ-1:0]       d;
        input [NUM_HW_ACCELERATORS-1:0] busy;
        input [NUM_BUFFERS-1:0]        full;
        input [NUM_BUFFERS-1:0]        free;
        input [NUM_BUFFERS-1:0]        colour;
        integer s, base;
        reg [MODE_SZ-1:0]      md;
        reg [BUFF_INDX_SZ-1:0] bid;
        reg                    ecolour;
        reg [TGT_ACC_SZ-1:0]   aid;
        reg                    acc_free, ok;
        reg                    needs_full, needs_free, src_ok, tgt_ok;
        begin
            ecolour = d[E_COLOUR];
            aid     = d[E_ACC_START +: TGT_ACC_SZ];
            // RTL: required_acc = (1<<aid) truncated to NUM_HW_ACCELERATORS bits;
            // acc_free = &(~required_acc | ~busy). If aid >= NUM_HW_ACCELERATORS the
            // shifted 1 falls off the top -> required_acc==0 -> acc_free is always 1.
            if (aid >= NUM_HW_ACCELERATORS) acc_free = 1'b1;
            else                            acc_free = ~busy[aid];
            ok = v & acc_free;
            for (s = 0; s < NUM_SLOTS; s = s + 1) begin
                if (s < 3) base = s*SLOT_SHORT_SZ;
                else       base = LONG_BASE + (s-3)*SLOT_LONG_SZ;
                md  = d[base +: MODE_SZ];
                bid = d[base + MODE_SZ +: BUFF_INDX_SZ];
                needs_full = (md == MODE_SRC) | (md == MODE_RW);
                needs_free = (md == MODE_TGT);
                src_ok = ~needs_full | (full[bid] & (colour[bid] == ecolour));
                tgt_ok = ~needs_free | free[bid];
                ok = ok & src_ok & tgt_ok;
            end
            model_ready = ok;
        end
    endfunction

    // Check combinational ready against the model for the CURRENT stored state.
    task check_ready_now;
        input [255:0] tag;
        reg exp;
        begin
            #1;   // let combinational ready settle after any blocking input change
            exp = model_ready(m_valid, m_data, acc_busy_i,
                              buffers_full_i, buffers_free_i, buffers_colour_i);
            check_bit(ready_to_execute_o, exp, {tag, " ready"});
        end
    endtask

    // Advance one clock and check the stored-state outputs + ready.
    task step_and_check;
        input [255:0] tag;
        begin
            model_step;
            @(posedge clk); #1;
            check_bit (entry_valid_o,            m_valid, {tag, " valid"});
            check_eq_u(entry_data_o,             m_data,  {tag, " data"});
            check_bit (shift_out_entry_valid_o,
                       delete_entry_i ? 1'b0 : m_valid,   {tag, " shift_out"});
            check_ready_now(tag);
        end
    endtask

    task clear_inputs;
        begin
            load_new_entry_i=0; shift_entry_i=0; delete_entry_i=0;
            new_entry_valid_i=0; new_entry_data_i=0;
            shift_in_entry_valid_i=0; shift_in_entry_data_i=0;
        end
    endtask

    integer it;

    initial begin
        verif_errors=0; verif_checks=0;
        clear_inputs;
        acc_busy_i=2'b11; buffers_full_i=0; buffers_free_i=16'hFFFF; buffers_colour_i=0;
        reset=1'b1;
        repeat(3) begin model_step; @(posedge clk); #1; end
        reset=1'b0; m_valid=0; m_data=0;
        $display("=== tb_sch_entry (scheduler) ===");

        // Reference entry: slot0=SRC b4, slot1=SRC b5, slot4=RW b7, slot5=TGT b9
        build_entry(MODE_SRC,4'd4, MODE_SRC,4'd5, MODE_UNUSED,4'd0,
                    MODE_UNUSED,4'd0,3'd0, MODE_RW,4'd7,3'd1, MODE_TGT,4'd9,3'd1,
                    1'b0, 2'd0, 5'd1);

        // ---- empty entry: not ready ----
        check_ready_now("D-empty");

        // ---- load entry; acc busy + src empty -> not ready ----
        clear_inputs; load_new_entry_i=1; new_entry_valid_i=1; new_entry_data_i=entry_build;
        step_and_check("D-load");
        clear_inputs; check_ready_now("D-loaded acc busy");

        // ---- src buffers full, acc still busy -> blocked ----
        buffers_full_i=16'h00B0; buffers_colour_i=16'h0000;  // b4,b5,b7
        check_ready_now("D-src full acc busy");

        // ---- free acc0 -> ready ----
        acc_busy_i=2'b00; check_ready_now("D-all met");

        // ---- target busy ----
        buffers_free_i=16'hFDFF; check_ready_now("D-tgt b9 busy");
        buffers_free_i=16'hFFFF; check_ready_now("D-tgt b9 free");

        // ---- RW buffer busy (full=0) ----
        buffers_full_i=16'h0030; check_ready_now("D-RW b7 busy");
        buffers_full_i=16'h00B0; check_ready_now("D-RW b7 full again");

        // ---- RW needs no free check: free=0 with full=1 still ready ----
        buffers_free_i=16'hFF7F; check_ready_now("D-RW b7 free=0 still ready");
        buffers_free_i=16'hFFFF;

        // ---- a source not full ----
        buffers_full_i=16'h0090; check_ready_now("D-src b5 not full");
        buffers_full_i=16'h00B0;

        // ---- colour mismatch on SRC b4 ----
        buffers_colour_i=16'h0010; check_ready_now("D-colour mismatch SRC");
        buffers_colour_i=16'h0000; check_ready_now("D-colour match restored");

        // ---- colour mismatch on RW b7 ----
        buffers_colour_i=16'h0080; check_ready_now("D-colour mismatch RW");
        buffers_colour_i=16'h0000;

        // ---- delete entry ----
        clear_inputs; delete_entry_i=1; step_and_check("D-delete");
        clear_inputs; check_ready_now("D-after delete");

        // ---- shift-in valid entry ----
        clear_inputs; shift_entry_i=1; shift_in_entry_valid_i=1; shift_in_entry_data_i=entry_build;
        step_and_check("D-shift-in valid");
        // ---- shift-in invalid clears ----
        clear_inputs; shift_entry_i=1; shift_in_entry_valid_i=1'b0;
        step_and_check("D-shift-in invalid");

        // ---- constrained-random ----
        clear_inputs; reset=1'b1; model_step; @(posedge clk); #1; reset=1'b0;
        m_valid=0; m_data=0;
        void'($urandom(32'h5C4E_7011));
        for (it=0; it<NRAND; it=it+1) begin
            clear_inputs;
            // random storage op (mutually exclusive priority handled by model)
            case ($urandom_range(0,4))
                0: ; // hold
                1: begin load_new_entry_i=1; new_entry_valid_i=1;
                         new_entry_data_i=$urandom ^ ($urandom<<16) ^ ($urandom<<32); end
                2: delete_entry_i=1;
                3: begin shift_entry_i=1; shift_in_entry_valid_i=$urandom&1;
                         shift_in_entry_data_i=$urandom ^ ($urandom<<16) ^ ($urandom<<32); end
                4: begin // load a structured legal entry (better ready coverage)
                         build_entry($urandom_range(0,3),$urandom_range(0,15),
                                     $urandom_range(0,3),$urandom_range(0,15),
                                     $urandom_range(0,3),$urandom_range(0,15),
                                     $urandom_range(0,3),$urandom_range(0,15),$urandom_range(0,7),
                                     $urandom_range(0,3),$urandom_range(0,15),$urandom_range(0,7),
                                     $urandom_range(0,3),$urandom_range(0,15),$urandom_range(0,7),
                                     $urandom&1, $urandom_range(0,1), $urandom_range(0,31));
                         load_new_entry_i=1; new_entry_valid_i=1; new_entry_data_i=entry_build; end
            endcase
            // random resource buses every cycle
            acc_busy_i       = $urandom_range(0,3);
            buffers_full_i   = $urandom;
            buffers_free_i   = $urandom;
            buffers_colour_i = $urandom;
            step_and_check("rand");
        end

        `VERIF_EPILOGUE("tb_sch_entry")
    end

    `VERIF_WATCHDOG(2000000)

endmodule
