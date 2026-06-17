// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
`timescale 10ps/1ps

// ───────────────────────────────────────────────────────────────────────────
// shared_pool — parameterized interleaved shared-memory pool arbiter
//
// NUM_BANKS single-port banks serve NUM_REQ requesters via low-order address
// interleaving:
//     bank      = logical_addr[BANK_SEL_BITS-1:0]
//     bank word = logical_addr >> BANK_SEL_BITS         (emitted on bank_addr_o)
//
// Per bank, one active requester wins each cycle. Requesters that lose
// arbitration on their target bank are back-pressured via req_wait_o and are
// expected to hold and retry. Single-port: one grant per bank per cycle (a read
// OR a write).
//
// Arbitration policy is selectable (ARB_RR):
//   ARB_RR=0  STRICT priority — lowest requester index wins (index 0 = highest).
//             Bit-identical to the original arbiter; the default, so any existing
//             instantiator is unaffected.
//   ARB_RR=1  ROUND-ROBIN — a per-bank pointer rotates priority past the last
//             winner each cycle, guaranteeing no requester is starved. Needed
//             when one accelerator issues two reads to the same bank and stalls
//             holding one (e.g. the annAcc's act + syn_curr reads colliding on a
//             bank: strict priority starves the lower-index port and deadlocks).
//   The two modes share one scan: STRICT is exactly ROUND-ROBIN with the pointer
//   frozen at NUM_REQ-1 (scan always starts at index 0).
//
// Reads are 1-cycle synchronous: the serving bank is registered per requester
// and the bank read-data is muxed back to that requester one cycle later (so the
// requester's cache/consumer sees its data aligned with its accepted request,
// exactly as dataline_cache_with_xy expects). Correct under concurrency — each
// reader samples its own serving bank one cycle after its own grant, independent
// of arbitration policy.
//
// Banks are EXTERNAL (instantiated by the top/testbench); this module is the
// arbiter + read-data return only. All ports are flattened Verilog-2001 vectors:
// requester k occupies bit k / word [k*W +: W]; bank b occupies bit b / [b*W +: W].
// ───────────────────────────────────────────────────────────────────────────

module shared_pool #(
    parameter NUM_BANKS = 4,
    parameter NUM_REQ   = 14,
    parameter ADDR_W    = 30,
    parameter DATA_W    = 32,
    parameter ARB_RR    = 0       // 0 = strict priority (default), 1 = round-robin
)(
    input  wire                        clk,

    // ── Requester side ──────────────────────────────────────────────────────
    input  wire [NUM_REQ-1:0]          req_act_i,    // requesting this cycle
    input  wire [NUM_REQ-1:0]          req_rd_i,     // request is a read
    input  wire [NUM_REQ-1:0]          req_wr_i,     // request is a write
    input  wire [NUM_REQ*ADDR_W-1:0]   req_addr_i,   // logical address per req
    input  wire [NUM_REQ*DATA_W-1:0]   req_wdata_i,  // write data per req
    output wire [NUM_REQ-1:0]          req_wait_o,   // stall (lost arb or bank wait)
    output wire [NUM_REQ*DATA_W-1:0]   req_rdata_o,  // read data (1-cycle later)

    // ── Bank side ───────────────────────────────────────────────────────────
    output wire [NUM_BANKS-1:0]        bank_rd_o,
    output wire [NUM_BANKS-1:0]        bank_wr_o,
    output wire [NUM_BANKS*ADDR_W-1:0] bank_addr_o,  // bank-local word (addr>>sel)
    output wire [NUM_BANKS*DATA_W-1:0] bank_wdata_o,
    input  wire [NUM_BANKS-1:0]        bank_wait_i,
    input  wire [NUM_BANKS*DATA_W-1:0] bank_rdata_i
);

    localparam BANK_SEL_BITS = (NUM_BANKS <= 1) ? 1 : $clog2(NUM_BANKS);
    localparam REQ_IDX_W     = (NUM_REQ   <= 1) ? 1 : $clog2(NUM_REQ);

    // ── Per-bank rotating priority pointer ──────────────────────────────────
    // Initialised to NUM_REQ-1 so the first scan starts at index 0 (== strict).
    // Only advanced when ARB_RR; in strict mode it stays frozen → lowest-index
    // requester always wins → bit-identical to the original arbiter.
    reg [REQ_IDX_W-1:0] rr_ptr [0:NUM_BANKS-1];
    integer ip;
    initial for (ip = 0; ip < NUM_BANKS; ip = ip + 1) rr_ptr[ip] = NUM_REQ-1;

    // ── Combinational grant + per-bank bus selection ────────────────────────
    integer b, off, ci, cand;
    reg [BANK_SEL_BITS-1:0]     cand_bank;
    reg                         found;
    reg [NUM_REQ-1:0]           grant;
    reg [NUM_BANKS-1:0]         bank_grant;
    reg [NUM_BANKS-1:0]         bank_rd_r, bank_wr_r;
    reg [NUM_BANKS*ADDR_W-1:0]  bank_addr_r;
    reg [NUM_BANKS*DATA_W-1:0]  bank_wdata_r;
    reg [NUM_REQ-1:0]           wait_r;
    reg [REQ_IDX_W-1:0]         win_idx [0:NUM_BANKS-1];

    always @(*) begin
        grant        = {NUM_REQ{1'b0}};
        bank_grant   = {NUM_BANKS{1'b0}};
        bank_rd_r    = {NUM_BANKS{1'b0}};
        bank_wr_r    = {NUM_BANKS{1'b0}};
        bank_addr_r  = {(NUM_BANKS*ADDR_W){1'b0}};
        bank_wdata_r = {(NUM_BANKS*DATA_W){1'b0}};
        wait_r       = {NUM_REQ{1'b0}};
        for (b = 0; b < NUM_BANKS; b = b + 1) win_idx[b] = {REQ_IDX_W{1'b0}};

        // Per bank, pick the first active requester targeting it, scanning in
        // rotated order starting just past the last winner (rr_ptr[b]+1).
        for (b = 0; b < NUM_BANKS; b = b + 1) begin
            found = 1'b0;
            for (off = 1; off <= NUM_REQ; off = off + 1) begin
                cand = rr_ptr[b] + off;
                if (cand >= NUM_REQ) cand = cand - NUM_REQ;
                cand_bank = req_addr_i[cand*ADDR_W +: BANK_SEL_BITS];
                if (!found && req_act_i[cand] && (cand_bank == b)) begin
                    found                 = 1'b1;
                    win_idx[b]            = cand;
                    grant[cand]           = 1'b1;
                    bank_grant[b]         = 1'b1;
                    bank_rd_r[b]          = req_rd_i[cand];
                    bank_wr_r[b]          = req_wr_i[cand];
                    bank_addr_r [b*ADDR_W +: ADDR_W] =
                        req_addr_i[cand*ADDR_W +: ADDR_W] >> BANK_SEL_BITS;
                    bank_wdata_r[b*DATA_W +: DATA_W] =
                        req_wdata_i[cand*DATA_W +: DATA_W];
                end
            end
        end

        // A requester waits if it is active and either lost arbitration or its
        // granted bank is itself stalled.
        for (ci = 0; ci < NUM_REQ; ci = ci + 1) begin
            if (req_act_i[ci]) begin
                if (grant[ci])
                    wait_r[ci] = bank_wait_i[req_addr_i[ci*ADDR_W +: BANK_SEL_BITS]];
                else
                    wait_r[ci] = 1'b1;
            end
        end
    end

    assign bank_rd_o    = bank_rd_r;
    assign bank_wr_o    = bank_wr_r;
    assign bank_addr_o  = bank_addr_r;
    assign bank_wdata_o = bank_wdata_r;
    assign req_wait_o   = wait_r;

    // ── Advance the rotating pointer past each bank's winner (RR mode only) ──
    integer bb;
    always @(posedge clk) begin
        if (ARB_RR) begin
            for (bb = 0; bb < NUM_BANKS; bb = bb + 1)
                if (bank_grant[bb])
                    rr_ptr[bb] <= win_idx[bb];
        end
    end

    // ── Read-data return: register serving bank per reader, mux next cycle ───
    reg [BANK_SEL_BITS-1:0] rdsel_r [0:NUM_REQ-1];
    integer j;
    initial for (j = 0; j < NUM_REQ; j = j + 1) rdsel_r[j] = {BANK_SEL_BITS{1'b0}};

    integer k;
    always @(posedge clk) begin
        for (k = 0; k < NUM_REQ; k = k + 1)
            if (grant[k] & req_rd_i[k])
                rdsel_r[k] <= req_addr_i[k*ADDR_W +: BANK_SEL_BITS];
    end

    genvar gk;
    generate for (gk = 0; gk < NUM_REQ; gk = gk + 1) begin : g_rdata
        assign req_rdata_o[gk*DATA_W +: DATA_W] =
            bank_rdata_i[rdsel_r[gk]*DATA_W +: DATA_W];
    end endgenerate

endmodule
