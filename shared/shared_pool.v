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
// Per bank, the lowest-index active requester wins (STRICT priority; requester
// index 0 = highest). Requesters that lose arbitration on their target bank are
// back-pressured via req_wait_o and are expected to hold and retry. Single-port:
// one grant per bank per cycle (a read OR a write).
//
// Reads are 1-cycle synchronous: the serving bank is registered per requester
// and the bank read-data is muxed back to that requester one cycle later (so the
// requester's cache/consumer sees its data aligned with its accepted request,
// exactly as dataline_cache_with_xy expects). Correct under concurrency — each
// reader samples its own serving bank one cycle after its own grant.
//
// Banks are EXTERNAL (instantiated by the top/testbench); this module is the
// arbiter + read-data return only. All ports are flattened Verilog-2001 vectors:
// requester k occupies bit k / word [k*W +: W]; bank b occupies bit b / [b*W +: W].
//
// NOTE: strict priority can starve the lowest-priority requester under sustained
// concurrent load. Acceptable for current sequential/bursty workloads; replace
// the per-bank pick with round-robin if real starvation appears.
// ───────────────────────────────────────────────────────────────────────────

module shared_pool #(
    parameter NUM_BANKS = 4,
    parameter NUM_REQ   = 14,
    parameter ADDR_W    = 30,
    parameter DATA_W    = 32
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

    integer k;
    reg [ADDR_W-1:0]         rq_addr;
    reg [BANK_SEL_BITS-1:0]  rq_bank;

    // ── Combinational priority grant + per-bank bus selection ───────────────
    reg [NUM_REQ-1:0]            grant;
    reg [NUM_BANKS-1:0]          bank_taken;
    reg [NUM_BANKS-1:0]          bank_rd_r, bank_wr_r;
    reg [NUM_BANKS*ADDR_W-1:0]   bank_addr_r;
    reg [NUM_BANKS*DATA_W-1:0]   bank_wdata_r;
    reg [NUM_REQ-1:0]            wait_r;

    always @(*) begin
        grant        = {NUM_REQ{1'b0}};
        bank_taken   = {NUM_BANKS{1'b0}};
        bank_rd_r    = {NUM_BANKS{1'b0}};
        bank_wr_r    = {NUM_BANKS{1'b0}};
        bank_addr_r  = {(NUM_BANKS*ADDR_W){1'b0}};
        bank_wdata_r = {(NUM_BANKS*DATA_W){1'b0}};
        wait_r       = {NUM_REQ{1'b0}};
        for (k = 0; k < NUM_REQ; k = k + 1) begin
            rq_addr = req_addr_i[k*ADDR_W +: ADDR_W];
            rq_bank = rq_addr[BANK_SEL_BITS-1:0];
            if (req_act_i[k]) begin
                if (!bank_taken[rq_bank]) begin
                    grant[k]            = 1'b1;
                    bank_taken[rq_bank] = 1'b1;
                    bank_rd_r[rq_bank]  = req_rd_i[k];
                    bank_wr_r[rq_bank]  = req_wr_i[k];
                    bank_addr_r [rq_bank*ADDR_W +: ADDR_W] = rq_addr >> BANK_SEL_BITS;
                    bank_wdata_r[rq_bank*DATA_W +: DATA_W] = req_wdata_i[k*DATA_W +: DATA_W];
                    wait_r[k] = bank_wait_i[rq_bank];   // granted: pass bank wait
                end else begin
                    wait_r[k] = 1'b1;                   // lost arbitration: stall
                end
            end
        end
    end

    assign bank_rd_o    = bank_rd_r;
    assign bank_wr_o    = bank_wr_r;
    assign bank_addr_o  = bank_addr_r;
    assign bank_wdata_o = bank_wdata_r;
    assign req_wait_o   = wait_r;

    // ── Read-data return: register serving bank per reader, mux next cycle ───
    reg [BANK_SEL_BITS-1:0] rdsel_r [0:NUM_REQ-1];
    integer j;
    initial for (j = 0; j < NUM_REQ; j = j + 1) rdsel_r[j] = {BANK_SEL_BITS{1'b0}};

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
