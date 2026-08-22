// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
// =============================================================================
// tb_buffer_state_multi  (scheduler — buffer_state_entry MULTI_WRITER=1)
//
// Authors      : Simon Davidson & Claude
// Created      : 2026-08-22
// Last modified: 2026-08-22
//
// MULTI_WRITER=1 lets a buffer have more than one task writing disjoint
// sub-ranges of it at once (see sch_entry's DISJOINT_HINT). The property that
// matters is NEGATIVE and easy to get wrong:
//
//     while ANY writer is still outstanding, the buffer must NOT read as full.
//
// Without it the sequence is: successor claims (full->0), predecessor completes
// (full->1), successor still writing — and a downstream consumer, which needs
// only `full`, dispatches and reads a PARTIALLY WRITTEN buffer. Test 2 is that
// exact sequence.
//
// Test 5 is the default-off guarantee: with at most one writer outstanding, the
// MULTI_WRITER=1 entry must agree with the MULTI_WRITER=0 entry cycle by cycle.
// =============================================================================
`timescale 1ns/1ps

module tb_buffer_state_multi;

    localparam TGT_COUNT_SZ = 4;

    reg clk = 1'b0, reset = 1'b1;
    reg                     mark_as_full_i          = 1'b0;
    reg  [TGT_COUNT_SZ-1:0] mark_buff_usage_i       = 0;
    reg                     buff_new_tgt_i          = 1'b0;
    reg  [TGT_COUNT_SZ-1:0] buff_new_usage_count_i  = 0;
    reg                     buff_new_colour_i       = 1'b0;
    reg                     buff_rw_claim_i         = 1'b0;
    reg                     buff_rw_colour_i        = 1'b0;
    reg                     buff_now_full_i         = 1'b0;
    reg  [TGT_COUNT_SZ-1:0] buff_now_usage_count_i  = 0;
    reg                     buff_content_consumed_i = 1'b0;

    wire [TGT_COUNT_SZ-1:0] m_usage,  s_usage;
    wire                    m_colour, s_colour;
    wire                    m_free,   s_free;
    wire                    m_full,   s_full;

    integer verif_errors, verif_checks;
    `include "../verif/checks.vh"

    // DUT under test: many writers allowed.
    buffer_state_entry #(.TGT_COUNT_SZ(TGT_COUNT_SZ), .MULTI_WRITER(1)) dut_multi (
        .clk(clk), .reset(reset),
        .mark_as_full_i(mark_as_full_i), .mark_buff_usage_i(mark_buff_usage_i),
        .buff_new_tgt_i(buff_new_tgt_i),
        .buff_new_usage_count_i(buff_new_usage_count_i),
        .buff_new_colour_i(buff_new_colour_i),
        .buff_rw_claim_i(buff_rw_claim_i), .buff_rw_colour_i(buff_rw_colour_i),
        .buff_now_full_i(buff_now_full_i),
        .buff_now_usage_count_i(buff_now_usage_count_i),
        .buff_content_consumed_i(buff_content_consumed_i),
        .buff_usage_count_o(m_usage), .buff_colour_o(m_colour),
        .buff_free_o(m_free), .buff_full_o(m_full));

    // Reference: the original single-writer entry, same stimulus.
    buffer_state_entry #(.TGT_COUNT_SZ(TGT_COUNT_SZ), .MULTI_WRITER(0)) dut_single (
        .clk(clk), .reset(reset),
        .mark_as_full_i(mark_as_full_i), .mark_buff_usage_i(mark_buff_usage_i),
        .buff_new_tgt_i(buff_new_tgt_i),
        .buff_new_usage_count_i(buff_new_usage_count_i),
        .buff_new_colour_i(buff_new_colour_i),
        .buff_rw_claim_i(buff_rw_claim_i), .buff_rw_colour_i(buff_rw_colour_i),
        .buff_now_full_i(buff_now_full_i),
        .buff_now_usage_count_i(buff_now_usage_count_i),
        .buff_content_consumed_i(buff_content_consumed_i),
        .buff_usage_count_o(s_usage), .buff_colour_o(s_colour),
        .buff_free_o(s_free), .buff_full_o(s_full));

    always #5 clk = ~clk;

    task tick;
        begin
            @(posedge clk); #1;
            mark_as_full_i = 1'b0; buff_new_tgt_i = 1'b0; buff_rw_claim_i = 1'b0;
            buff_now_full_i = 1'b0; buff_content_consumed_i = 1'b0;
        end
    endtask

    // Put the buffer in the "full, one reader pending" state both DUTs agree on.
    task prime;
        begin
            reset = 1'b1; tick; reset = 1'b0;
            mark_as_full_i = 1'b1; mark_buff_usage_i = 4'd1; tick;
        end
    endtask

    integer i, outstanding;
    reg [3:0] pick;

    initial begin
        verif_errors = 0; verif_checks = 0;
        $display("=== tb_buffer_state_multi (scheduler) ===");
        reset = 1'b1; tick; tick; reset = 1'b0; tick;

        // ---- 1. one writer behaves exactly as before -----------------------
        prime;
        buff_rw_claim_i = 1'b1; buff_rw_colour_i = 1'b1; tick;
        check_eq(m_full, 1'b0, "T1 claim clears full");
        buff_now_full_i = 1'b1; buff_now_usage_count_i = 4'd5; tick;
        check_eq(m_full,  1'b1, "T1 completion refills");
        check_eq(m_usage, 4'd5, "T1 completion loads usage");

        // ---- 2. THE HAZARD: two writers, first one completes ---------------
        prime;
        buff_rw_claim_i = 1'b1; tick;                        // writer A claims
        buff_rw_claim_i = 1'b1; tick;                        // writer B claims
        check_eq(m_full, 1'b0, "T2 two claims -> not full");
        buff_now_full_i = 1'b1; buff_now_usage_count_i = 4'd1; tick;   // A done
        check_eq(m_full, 1'b0,
                    "T2 FULL MUST STAY 0 WHILE B IS STILL WRITING");
        check_eq(s_full, 1'b1,
                    "T2 the single-writer entry gets this WRONG (documents why)");
        buff_now_full_i = 1'b1; buff_now_usage_count_i = 4'd6; tick;   // B done
        check_eq(m_full,  1'b1, "T2 last writer refills");
        check_eq(m_usage, 4'd6, "T2 LAST writer's usage count wins");

        // ---- 3. claim and completion on the SAME cycle ---------------------
        prime;
        buff_rw_claim_i = 1'b1; tick;                        // A claims
        buff_rw_claim_i = 1'b1; buff_now_full_i = 1'b1;      // B claims as A ends
        buff_now_usage_count_i = 4'd2; tick;
        check_eq(m_full, 1'b0, "T3 simultaneous start+finish -> still a writer");
        buff_now_full_i = 1'b1; buff_now_usage_count_i = 4'd3; tick;
        check_eq(m_full,  1'b1, "T3 refills once B completes");
        check_eq(m_usage, 4'd3, "T3 B's usage count lands");

        // ---- 4. a TGT counts as a writer too -------------------------------
        prime;
        buff_new_tgt_i = 1'b1; buff_new_usage_count_i = 4'd1; tick;
        buff_rw_claim_i = 1'b1; tick;
        buff_now_full_i = 1'b1; buff_now_usage_count_i = 4'd1; tick;
        check_eq(m_full, 1'b0, "T4 TGT+RW outstanding -> not full");
        buff_now_full_i = 1'b1; buff_now_usage_count_i = 4'd4; tick;
        check_eq(m_full, 1'b1, "T4 refills when both retire");

        // ---- 5. equivalence while only ONE writer is ever outstanding ------
        // This is the default-off guarantee: nothing existing can tell the two
        // implementations apart, because nothing existing overlaps writers.
        reset = 1'b1; tick; reset = 1'b0; outstanding = 0;
        for (i = 0; i < 4000; i = i + 1) begin
            pick = $urandom_range(0, 5);
            case (pick)
                0, 1: if (outstanding == 0) begin
                          buff_rw_claim_i = 1'b1;
                          buff_rw_colour_i = $urandom & 1'b1;
                          outstanding = 1;
                      end
                2:    if (outstanding == 0) begin
                          buff_new_tgt_i = 1'b1;
                          buff_new_usage_count_i = $urandom_range(1, 7);
                          buff_new_colour_i = $urandom & 1'b1;
                          outstanding = 1;
                      end
                3:    if (outstanding == 1) begin
                          buff_now_full_i = 1'b1;
                          buff_now_usage_count_i = $urandom_range(1, 7);
                          outstanding = 0;
                      end
                4:    if (outstanding == 0) begin
                          mark_as_full_i = 1'b1;
                          mark_buff_usage_i = $urandom_range(1, 7);
                      end
                5:    buff_content_consumed_i = 1'b1;
            endcase
            tick;
            check_eq(m_full,   s_full,   "T5 full matches single-writer entry");
            check_eq(m_free,   s_free,   "T5 free matches single-writer entry");
            check_eq(m_usage,  s_usage,  "T5 usage matches single-writer entry");
            check_eq(m_colour, s_colour, "T5 colour matches single-writer entry");
        end

        `VERIF_EPILOGUE("tb_buffer_state_multi")
    end

    `VERIF_WATCHDOG(2000000)

endmodule
