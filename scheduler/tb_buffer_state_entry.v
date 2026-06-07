// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
// =============================================================================
// tb_buffer_state_entry  (scheduler — per-buffer {free,full,colour,counter})
//
// Authors      : Simon Davidson & Claude
// Created      : 2026-05-12
// Last modified: 2026-06-07
//
// Aggressive rewrite. Maintains a cycle-accurate SOFTWARE reference model that
// replicates buffer_state_entry.v's four priority chains EXACTLY, then:
//   * directed transitions for every strobe class and the free-on-last-consumer
//     edge (count 1->0), plus the full counter range 0..7,
//   * a long constrained-random loop driving one strobe-class per cycle and
//     checking free/full/colour/count against the SW model EVERY cycle.
//
// RTL priority chains modelled (highest first), all on posedge clk:
//   free_r:   reset->1 ; rw_claim->0 ; mark_as_full->0 ; (cnt==1 & consumed)->1 ; new_tgt->0
//   full_r:   reset->0 ; rw_claim->0 ; (now_full|mark_as_full)->1 ; (cnt==1 & consumed)->0
//   colour_r: reset->0 ; rw_claim->rw_colour ; new_tgt->new_colour
//   count_r:  reset->0 ; mark_as_full->mark_usage ; now_full->now_usage ;
//             new_tgt->new_usage ; consumed->count-1
// =============================================================================
`timescale 1ns/1ps

module tb_buffer_state_entry;

    localparam integer TGT_COUNT_SZ = 3;
    localparam integer NRAND        = 8000;

    reg                     clk, reset;
    reg                     mark_as_full_i;
    reg  [TGT_COUNT_SZ-1:0] mark_buff_usage_i;
    reg                     buff_new_tgt_i;
    reg  [TGT_COUNT_SZ-1:0] buff_new_usage_count_i;
    reg                     buff_new_colour_i;
    reg                     buff_rw_claim_i;
    reg                     buff_rw_colour_i;
    reg                     buff_now_full_i;
    reg  [TGT_COUNT_SZ-1:0] buff_now_usage_count_i;
    reg                     buff_content_consumed_i;

    wire [TGT_COUNT_SZ-1:0] buff_usage_count_o;
    wire                    buff_colour_o;
    wire                    buff_free_o;
    wire                    buff_full_o;

    integer verif_errors, verif_checks, verif_to;
    `include "../verif/checks.vh"

    buffer_state_entry #(.TGT_COUNT_SZ(TGT_COUNT_SZ)) dut (
        .clk(clk), .reset(reset),
        .mark_as_full_i(mark_as_full_i), .mark_buff_usage_i(mark_buff_usage_i),
        .buff_new_tgt_i(buff_new_tgt_i),
        .buff_new_usage_count_i(buff_new_usage_count_i),
        .buff_new_colour_i(buff_new_colour_i),
        .buff_rw_claim_i(buff_rw_claim_i), .buff_rw_colour_i(buff_rw_colour_i),
        .buff_now_full_i(buff_now_full_i),
        .buff_now_usage_count_i(buff_now_usage_count_i),
        .buff_content_consumed_i(buff_content_consumed_i),
        .buff_usage_count_o(buff_usage_count_o),
        .buff_colour_o(buff_colour_o),
        .buff_free_o(buff_free_o),
        .buff_full_o(buff_full_o));

    initial clk = 1'b0; always #5 clk = ~clk;

    // ----------------------------------------------------------------------
    // Software reference model: holds the EXPECTED registered state.
    // ----------------------------------------------------------------------
    reg                     m_free, m_full, m_colour;
    reg [TGT_COUNT_SZ-1:0]  m_count;

    // Compute next-state from the SW model given the currently-driven inputs.
    // Mirrors the RTL priority order exactly.  `no_longer_needed` uses the
    // CURRENT (pre-edge) count, exactly like the RTL combinational wire.
    task model_step;
        reg no_longer_needed;
        reg nf, nfu, nc;
        reg [TGT_COUNT_SZ-1:0] ncnt;
        begin
            no_longer_needed = (m_count == 'b1) & buff_content_consumed_i;

            // free_r
            if      (reset)            nf = 1'b1;
            else if (buff_rw_claim_i)  nf = 1'b0;
            else if (mark_as_full_i)   nf = 1'b0;
            else if (no_longer_needed) nf = 1'b1;
            else if (buff_new_tgt_i)   nf = 1'b0;
            else                       nf = m_free;

            // full_r
            if      (reset)                              nfu = 1'b0;
            else if (buff_rw_claim_i)                    nfu = 1'b0;
            else if (buff_now_full_i | mark_as_full_i)   nfu = 1'b1;
            else if (no_longer_needed)                   nfu = 1'b0;
            else                                         nfu = m_full;

            // colour_r
            if      (reset)           nc = 1'b0;
            else if (buff_rw_claim_i) nc = buff_rw_colour_i;
            else if (buff_new_tgt_i)  nc = buff_new_colour_i;
            else                      nc = m_colour;

            // count_r
            if      (reset)                   ncnt = 'b0;
            else if (mark_as_full_i)          ncnt = mark_buff_usage_i;
            else if (buff_now_full_i)         ncnt = buff_now_usage_count_i;
            else if (buff_new_tgt_i)          ncnt = buff_new_usage_count_i;
            else if (buff_content_consumed_i) ncnt = m_count - 'b1;
            else                              ncnt = m_count;

            m_free   = nf;
            m_full   = nfu;
            m_colour = nc;
            m_count  = ncnt;
        end
    endtask

    // Drive inputs for one cycle, advance both DUT and model over the edge, then
    // compare all four outputs.  Inputs must already be set before calling.
    task step_and_check;
        input [255:0] tag;
        begin
            model_step;            // compute expected post-edge state
            @(posedge clk); #1;    // DUT registers update
            check_bit(buff_free_o,                  m_free,  {tag, " free"});
            check_bit(buff_full_o,                  m_full,  {tag, " full"});
            check_bit(buff_colour_o,                m_colour,{tag, " colour"});
            check_eq ($unsigned(buff_usage_count_o),
                      $unsigned(m_count),                    {tag, " count"});
        end
    endtask

    task clear_inputs;
        begin
            mark_as_full_i=0; mark_buff_usage_i=0;
            buff_new_tgt_i=0; buff_new_usage_count_i=0; buff_new_colour_i=0;
            buff_rw_claim_i=0; buff_rw_colour_i=0;
            buff_now_full_i=0; buff_now_usage_count_i=0;
            buff_content_consumed_i=0;
        end
    endtask

    integer it, klass;

    initial begin
        verif_errors=0; verif_checks=0;
        clear_inputs; reset=1'b1;
        // hold reset a few cycles, model tracks it
        repeat(3) begin model_step; @(posedge clk); #1; end
        reset=1'b0;
        $display("=== tb_buffer_state_entry (scheduler) ===");
        check_bit(buff_free_o,   1'b1, "post-reset free");
        check_bit(buff_full_o,   1'b0, "post-reset full");
        check_bit(buff_colour_o, 1'b0, "post-reset colour");
        // sync model to confirmed post-reset state
        m_free=1; m_full=0; m_colour=0; m_count=0;

        // ---- directed: producer target -> full -> drain to free ----
        clear_inputs;
        buff_new_tgt_i=1; buff_new_usage_count_i=3'd2; buff_new_colour_i=1'b1;
        step_and_check("D-newtgt");
        clear_inputs;
        buff_now_full_i=1; buff_now_usage_count_i=3'd2;
        step_and_check("D-nowfull");
        clear_inputs;
        buff_content_consumed_i=1; step_and_check("D-consume 2->1");
        buff_content_consumed_i=1; step_and_check("D-consume 1->0 free edge");
        clear_inputs; step_and_check("D-idle after free");

        // ---- directed: mark_as_full then full drain over all counts ----
        clear_inputs;
        buff_new_tgt_i=1; buff_new_usage_count_i=3'd7; buff_new_colour_i=1'b0;
        step_and_check("D-newtgt cnt7");
        clear_inputs;
        buff_now_full_i=1; buff_now_usage_count_i=3'd7;
        step_and_check("D-nowfull cnt7");
        for (it=0; it<7; it=it+1) begin
            clear_inputs; buff_content_consumed_i=1;
            step_and_check("D-drain7");
        end
        clear_inputs; step_and_check("D-drained7 idle");

        // ---- directed: RW claim (full->busy), refill ----
        clear_inputs;
        mark_as_full_i=1; mark_buff_usage_i=3'd3;
        step_and_check("D-markfull3");
        clear_inputs;
        buff_rw_claim_i=1; buff_rw_colour_i=1'b1;
        step_and_check("D-rwclaim busy");
        clear_inputs;
        buff_now_full_i=1; buff_now_usage_count_i=3'd1;
        step_and_check("D-rw refill cnt1");
        clear_inputs;
        buff_content_consumed_i=1;
        step_and_check("D-rw final consume free");

        // ---- directed: simultaneous now_full + consumed on count==1 ----
        // RTL: now_full has priority over no_longer_needed for full_r; count is
        // reloaded by now_full (consume branch lower priority). free unaffected.
        clear_inputs;
        buff_new_tgt_i=1; buff_new_usage_count_i=3'd1;
        step_and_check("D-precond newtgt1");
        clear_inputs;
        buff_now_full_i=1; buff_now_usage_count_i=3'd1;
        step_and_check("D-precond nowfull1");
        clear_inputs;
        buff_now_full_i=1; buff_now_usage_count_i=3'd4; buff_content_consumed_i=1;
        step_and_check("D-nowfull+consume @cnt1 (now_full wins)");

        // ---- directed: consume when count==0 (underflow wraps; matches RTL) ----
        clear_inputs;
        buff_new_tgt_i=1; buff_new_usage_count_i=3'd0;
        step_and_check("D-newtgt cnt0");
        clear_inputs;
        buff_now_full_i=1; buff_now_usage_count_i=3'd0;
        step_and_check("D-nowfull cnt0");
        clear_inputs; buff_content_consumed_i=1;
        step_and_check("D-consume @cnt0 (no_longer_needed=0, wraps)");

        // ---- constrained-random: one strobe-class per cycle ----
        // Re-establish a clean known state via a reset pulse first.
        clear_inputs; reset=1'b1; model_step; @(posedge clk); #1; reset=1'b0;
        m_free=1; m_full=0; m_colour=0; m_count=0;

        void'($urandom(32'hB0FF_5747));
        for (it=0; it<NRAND; it=it+1) begin
            clear_inputs;
            klass = $urandom_range(0,6);
            case (klass)
                0: ; // idle
                1: begin
                       buff_new_tgt_i=1;
                       buff_new_usage_count_i=$urandom_range(0,7);
                       buff_new_colour_i=$urandom & 1'b1;
                   end
                2: begin
                       buff_now_full_i=1;
                       buff_now_usage_count_i=$urandom_range(0,7);
                   end
                3: begin
                       buff_rw_claim_i=1;
                       buff_rw_colour_i=$urandom & 1'b1;
                   end
                4: begin
                       mark_as_full_i=1;
                       mark_buff_usage_i=$urandom_range(0,7);
                   end
                5: buff_content_consumed_i=1;
                6: begin
                       // occasional rare reset to exercise the reset branch mid-stream
                       if ($urandom_range(0,9)==0) reset=1'b1;
                       else                          buff_content_consumed_i=1;
                   end
            endcase
            step_and_check("rand");
            reset=1'b0;
        end

        `VERIF_EPILOGUE("tb_buffer_state_entry")
    end

    `VERIF_WATCHDOG(2000000)

endmodule
