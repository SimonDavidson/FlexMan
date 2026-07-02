// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
// Author: Simon Davidson & Claude
// Created: 2026-07-02
// Last modified: 2026-07-02
//
// ─────────────────────────────────────────────────────────────────────────────
// tb_shared_pool_equiv — ARB_FAST equivalence testbench
//
// Instantiates shared_pool twice with IDENTICAL parameters except ARB_FAST
// (0 = sequential scan reference, 1 = log-depth implementation) and drives
// both with the same stimulus, comparing EVERY output every cycle. Because
// the only internal state (rr_ptr, rdsel_r) evolves purely from the grant
// decisions, any functional divergence shows up immediately at the outputs
// and then compounds — a strict lockstep check.
//
// Covered per configuration pair:
//   - directed corner patterns (none / all / single requester, all-same-bank,
//     walking-one, all-banks-waiting)
//   - long constrained-random runs (random act/rd/wr/addr/wdata/bank_wait)
// Config pairs:  deployment-shape 16req×4banks ARB_RR=1, same with ARB_RR=0,
//                small 5req×2banks ARB_RR=1 (dense state coverage).
// ─────────────────────────────────────────────────────────────────────────────

`timescale 1ns/1ps

module pool_pair #(
    parameter NUM_BANKS = 4,
    parameter NUM_REQ   = 16,
    parameter ADDR_W    = 30,
    parameter DATA_W    = 32,
    parameter ARB_RR    = 1
)(
    input  wire                        clk,
    input  wire [NUM_REQ-1:0]          req_act_i,
    input  wire [NUM_REQ-1:0]          req_rd_i,
    input  wire [NUM_REQ-1:0]          req_wr_i,
    input  wire [NUM_REQ*ADDR_W-1:0]   req_addr_i,
    input  wire [NUM_REQ*DATA_W-1:0]   req_wdata_i,
    input  wire [NUM_BANKS-1:0]        bank_wait_i,
    input  wire [NUM_BANKS*DATA_W-1:0] bank_rdata_i,
    output wire                        mismatch_o
);

    wire [NUM_REQ-1:0]          ref_wait,  fast_wait;
    wire [NUM_REQ*DATA_W-1:0]   ref_rdata, fast_rdata;
    wire [NUM_BANKS-1:0]        ref_brd,   fast_brd;
    wire [NUM_BANKS-1:0]        ref_bwr,   fast_bwr;
    wire [NUM_BANKS*ADDR_W-1:0] ref_baddr, fast_baddr;
    wire [NUM_BANKS*DATA_W-1:0] ref_bwdata,fast_bwdata;

    shared_pool #(.NUM_BANKS(NUM_BANKS), .NUM_REQ(NUM_REQ), .ADDR_W(ADDR_W),
                  .DATA_W(DATA_W), .ARB_RR(ARB_RR), .ARB_FAST(0)) u_ref (
        .clk(clk),
        .req_act_i(req_act_i), .req_rd_i(req_rd_i), .req_wr_i(req_wr_i),
        .req_addr_i(req_addr_i), .req_wdata_i(req_wdata_i),
        .req_wait_o(ref_wait), .req_rdata_o(ref_rdata),
        .bank_rd_o(ref_brd), .bank_wr_o(ref_bwr),
        .bank_addr_o(ref_baddr), .bank_wdata_o(ref_bwdata),
        .bank_wait_i(bank_wait_i), .bank_rdata_i(bank_rdata_i));

    shared_pool #(.NUM_BANKS(NUM_BANKS), .NUM_REQ(NUM_REQ), .ADDR_W(ADDR_W),
                  .DATA_W(DATA_W), .ARB_RR(ARB_RR), .ARB_FAST(1)) u_fast (
        .clk(clk),
        .req_act_i(req_act_i), .req_rd_i(req_rd_i), .req_wr_i(req_wr_i),
        .req_addr_i(req_addr_i), .req_wdata_i(req_wdata_i),
        .req_wait_o(fast_wait), .req_rdata_o(fast_rdata),
        .bank_rd_o(fast_brd), .bank_wr_o(fast_bwr),
        .bank_addr_o(fast_baddr), .bank_wdata_o(fast_bwdata),
        .bank_wait_i(bank_wait_i), .bank_rdata_i(bank_rdata_i));

    assign mismatch_o = (ref_wait   !== fast_wait  ) |
                        (ref_rdata  !== fast_rdata ) |
                        (ref_brd    !== fast_brd   ) |
                        (ref_bwr    !== fast_bwr   ) |
                        (ref_baddr  !== fast_baddr ) |
                        (ref_bwdata !== fast_bwdata);

endmodule


module tb_shared_pool_equiv;

    localparam ADDR_W = 30;
    localparam DATA_W = 32;

    // pair A: deployment shape, round-robin (the shipping configuration)
    localparam A_REQ = 16, A_BANK = 4;
    // pair B: deployment shape, strict priority
    // pair C: small shape, round-robin (dense pointer/eligibility coverage)
    localparam C_REQ = 5,  C_BANK = 2;

    reg clk = 1'b0;
    always #5 clk = ~clk;

    // ── stimulus (widest shape; pair C uses the low slices) ──────────────────
    reg [A_REQ-1:0]          act;
    reg [A_REQ-1:0]          rd;
    reg [A_REQ-1:0]          wr;
    reg [A_REQ*ADDR_W-1:0]   addr;
    reg [A_REQ*DATA_W-1:0]   wdata;
    reg [A_BANK-1:0]         bwait;
    reg [A_BANK*DATA_W-1:0]  brdata;

    wire mm_a, mm_b, mm_c;

    pool_pair #(.NUM_BANKS(A_BANK), .NUM_REQ(A_REQ), .ADDR_W(ADDR_W),
                .DATA_W(DATA_W), .ARB_RR(1)) pair_a (
        .clk(clk), .req_act_i(act), .req_rd_i(rd), .req_wr_i(wr),
        .req_addr_i(addr), .req_wdata_i(wdata),
        .bank_wait_i(bwait), .bank_rdata_i(brdata), .mismatch_o(mm_a));

    pool_pair #(.NUM_BANKS(A_BANK), .NUM_REQ(A_REQ), .ADDR_W(ADDR_W),
                .DATA_W(DATA_W), .ARB_RR(0)) pair_b (
        .clk(clk), .req_act_i(act), .req_rd_i(rd), .req_wr_i(wr),
        .req_addr_i(addr), .req_wdata_i(wdata),
        .bank_wait_i(bwait), .bank_rdata_i(brdata), .mismatch_o(mm_b));

    pool_pair #(.NUM_BANKS(C_BANK), .NUM_REQ(C_REQ), .ADDR_W(ADDR_W),
                .DATA_W(DATA_W), .ARB_RR(1)) pair_c (
        .clk(clk), .req_act_i(act[C_REQ-1:0]), .req_rd_i(rd[C_REQ-1:0]),
        .req_wr_i(wr[C_REQ-1:0]),
        .req_addr_i(addr[C_REQ*ADDR_W-1:0]), .req_wdata_i(wdata[C_REQ*DATA_W-1:0]),
        .bank_wait_i(bwait[C_BANK-1:0]), .bank_rdata_i(brdata[C_BANK*DATA_W-1:0]),
        .mismatch_o(mm_c));

    integer errors = 0;
    integer checks = 0;

    // Compare on the falling edge so combinational outputs and the registered
    // read-return (rdsel_r) have both settled after the rising edge.
    always @(negedge clk) begin
        checks = checks + 1;
        if (mm_a === 1'b1) begin
            errors = errors + 1;
            $display("FAIL pair_a (16x4 RR) mismatch at %0t", $time);
        end
        if (mm_b === 1'b1) begin
            errors = errors + 1;
            $display("FAIL pair_b (16x4 strict) mismatch at %0t", $time);
        end
        if (mm_c === 1'b1) begin
            errors = errors + 1;
            $display("FAIL pair_c (5x2 RR) mismatch at %0t", $time);
        end
        if (errors > 20) begin
            $display("TB: too many errors, aborting");
            $finish;
        end
    end

    integer i;

    task set_idle;
        begin
            act = 'b0; rd = 'b0; wr = 'b0;
            addr = 'b0; wdata = 'b0; bwait = 'b0;
        end
    endtask

    // random request vectors: ~75% of requesters active, rd/wr split,
    // random bank targets, random bank_wait
    task randomize_cycle;
        reg [31:0] r;
        integer j;                       // task-local (NOT the initial-block i)
        begin
            for (j = 0; j < A_REQ; j = j + 1) begin
                r = $random;
                act[j] = (r[1:0] != 2'b00);              // 75% active
                rd[j]  = act[j] &  r[2];
                wr[j]  = act[j] & ~r[2];
                addr [j*ADDR_W +: ADDR_W] = {$random} % (1<<16);
                wdata[j*DATA_W +: DATA_W] = $random;
            end
            r = $random;
            bwait = r[A_BANK-1:0] & r[2*A_BANK-1:A_BANK]; // ~25% per bank
            for (j = 0; j < A_BANK; j = j + 1)
                brdata[j*DATA_W +: DATA_W] = $random;
        end
    endtask

    initial begin
        set_idle;
        brdata = {A_BANK{32'hDEAD_BEEF}};
        repeat (4) @(negedge clk);

        // ── directed corners ────────────────────────────────────────────────
        // walking single requester, each bank
        for (i = 0; i < A_REQ*4; i = i + 1) begin
            set_idle;
            act[i % A_REQ] = 1'b1;
            rd [i % A_REQ] = 1'b1;
            addr[(i % A_REQ)*ADDR_W +: ADDR_W] = i / A_REQ; // bank = i/A_REQ
            @(negedge clk);
        end

        // all requesters active, all targeting the SAME bank (max contention)
        for (i = 0; i < 64; i = i + 1) begin
            act = {A_REQ{1'b1}};
            rd  = {A_REQ{1'b1}};
            wr  = 'b0;
            begin : same_bank
                integer k;
                for (k = 0; k < A_REQ; k = k + 1)
                    addr[k*ADDR_W +: ADDR_W] = (i % 4) + 4*k; // same bank, diff words
            end
            bwait = (i[5:4] == 2'b10) ? {A_BANK{1'b1}} : 'b0;
            @(negedge clk);
        end

        // none active with stale addresses
        set_idle;
        repeat (8) @(negedge clk);

        // ── constrained random ──────────────────────────────────────────────
        for (i = 0; i < 100000; i = i + 1) begin
            randomize_cycle;
            @(negedge clk);
        end

        if (errors == 0)
            $display("TB PASSED: shared_pool ARB_FAST equivalent over %0d checks (3 config pairs)", checks);
        else
            $display("TB FAILED: %0d mismatches over %0d checks", errors, checks);
        $finish;
    end

endmodule
