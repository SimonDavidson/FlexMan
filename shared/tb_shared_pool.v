// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
// =============================================================================
// tb_shared_pool  --  multi-requester interleaved shared-memory pool arbiter
//
// Authors      : Simon Davidson & Claude
// Created      : 2026-06-07
// Last modified: 2026-06-07
//
// shared_pool is a pure arbiter + 1-cycle read-data return (banks are external).
// Interleave:  bank = addr[BANK_SEL_BITS-1:0],  bank_word = addr >> BANK_SEL_BITS.
// Per bank the LOWEST-index active requester wins (strict priority). Losers and
// requesters whose granted bank asserts bank_wait_i are stalled via req_wait_o.
// Reads return 1 cycle later, muxed from the bank the requester was granted.
//
// SW golden (per cycle, combinational arbiter):
//   * replay the exact priority loop -> grant[], bank_taken[], bank rd/wr/addr/
//     wdata, req_wait[]  and CHECK every DUT output against it.
//   * model registered rdsel per reader and CHECK req_rdata_o one cycle later
//     against the bank read-data the reader was granted.
// Bank read-data is produced by a TB bank-memory model with 1-cycle latency
// driven from the DUT's own bank_rd_o/bank_addr_o (so reads return real data).
// =============================================================================
`timescale 10ps/1ps

module tb_shared_pool;

    localparam integer NUM_BANKS = 4;
    localparam integer NUM_REQ   = 6;
    localparam integer ADDR_W    = 12;          // small for fast bank arrays
    localparam integer DATA_W    = 32;
    localparam integer SEL       = 2;           // $clog2(4)
    localparam integer BANK_WORDS = 64;         // words per bank model
    localparam integer NCYC      = 6000;

    reg                          clk;
    reg  [NUM_REQ-1:0]           req_act, req_rd, req_wr;
    reg  [NUM_REQ*ADDR_W-1:0]    req_addr;
    reg  [NUM_REQ*DATA_W-1:0]    req_wdata;
    wire [NUM_REQ-1:0]           req_wait;
    wire [NUM_REQ*DATA_W-1:0]    req_rdata;

    wire [NUM_BANKS-1:0]         bank_rd, bank_wr;
    wire [NUM_BANKS*ADDR_W-1:0]  bank_addr;
    wire [NUM_BANKS*DATA_W-1:0]  bank_wdata;
    reg  [NUM_BANKS-1:0]         bank_wait;
    reg  [NUM_BANKS*DATA_W-1:0]  bank_rdata;

    integer verif_errors, verif_checks;
    `include "../verif/checks.vh"

    shared_pool #(
        .NUM_BANKS(NUM_BANKS), .NUM_REQ(NUM_REQ),
        .ADDR_W(ADDR_W), .DATA_W(DATA_W)
    ) dut (
        .clk(clk),
        .req_act_i(req_act), .req_rd_i(req_rd), .req_wr_i(req_wr),
        .req_addr_i(req_addr), .req_wdata_i(req_wdata),
        .req_wait_o(req_wait), .req_rdata_o(req_rdata),
        .bank_rd_o(bank_rd), .bank_wr_o(bank_wr),
        .bank_addr_o(bank_addr), .bank_wdata_o(bank_wdata),
        .bank_wait_i(bank_wait), .bank_rdata_i(bank_rdata));

    initial clk = 0; always #50 clk = ~clk;

    // ---- TB bank memory model (NUM_BANKS independent single-port arrays) ----
    // 1-cycle synchronous read: registers bank_rdata from the DUT's grant.
    reg [DATA_W-1:0] bankmem [0:NUM_BANKS-1][0:BANK_WORDS-1];
    integer bb, ww, ii;

    always @(posedge clk) begin
        for (bb = 0; bb < NUM_BANKS; bb = bb + 1) begin
            // honour bank_wait: a waited bank performs no access this cycle
            if (bank_rd[bb] && !bank_wait[bb])
                bank_rdata[bb*DATA_W +: DATA_W] <=
                    bankmem[bb][bank_addr[bb*ADDR_W +: ADDR_W] % BANK_WORDS];
            if (bank_wr[bb] && !bank_wait[bb])
                bankmem[bb][bank_addr[bb*ADDR_W +: ADDR_W] % BANK_WORDS] <=
                    bank_wdata[bb*DATA_W +: DATA_W];
        end
    end

    // =====================================================================
    // SW golden arbiter (mirrors shared_pool combinational block exactly)
    // =====================================================================
    reg [NUM_REQ-1:0]           g_grant;
    reg [NUM_BANKS-1:0]         g_btaken, g_brd, g_bwr;
    reg [NUM_BANKS*ADDR_W-1:0]  g_baddr;
    reg [NUM_BANKS*DATA_W-1:0]  g_bwdata;
    reg [NUM_REQ-1:0]           g_wait;
    integer kk;
    reg [ADDR_W-1:0]            ga;
    reg [SEL-1:0]               gbk;

    task golden_arb;
        begin
            g_grant=0; g_btaken=0; g_brd=0; g_bwr=0; g_wait=0;
            g_baddr={(NUM_BANKS*ADDR_W){1'b0}};
            g_bwdata={(NUM_BANKS*DATA_W){1'b0}};
            for (kk = 0; kk < NUM_REQ; kk = kk + 1) begin
                ga  = req_addr[kk*ADDR_W +: ADDR_W];
                gbk = ga[SEL-1:0];
                if (req_act[kk]) begin
                    if (!g_btaken[gbk]) begin
                        g_grant[kk]   = 1'b1;
                        g_btaken[gbk] = 1'b1;
                        g_brd[gbk]    = req_rd[kk];
                        g_bwr[gbk]    = req_wr[kk];
                        g_baddr [gbk*ADDR_W +: ADDR_W] = ga >> SEL;
                        g_bwdata[gbk*DATA_W +: DATA_W] = req_wdata[kk*DATA_W +: DATA_W];
                        g_wait[kk] = bank_wait[gbk];
                    end else
                        g_wait[kk] = 1'b1;
                end
            end
        end
    endtask

    // Registered read-select model (1-cycle read-data alignment).
    reg [SEL-1:0] g_rdsel [0:NUM_REQ-1];
    reg           g_rdsel_v [0:NUM_REQ-1];   // has this reader ever been granted a read
    integer jj;

    // ---- per-cycle drive + check ------------------------------------------
    task check_cycle;
        input [255:0] tag;
        reg [DATA_W-1:0] exp_rd;
        begin
            // Inputs already driven. Compute golden arbiter (combinational).
            golden_arb;
            #1; // settle DUT combinational outputs at the same time-point
            // --- check combinational arbiter outputs ---
            check_eq_u(req_wait, g_wait,  {tag, " req_wait"});
            check_eq_u(bank_rd,  g_brd,   {tag, " bank_rd"});
            check_eq_u(bank_wr,  g_bwr,   {tag, " bank_wr"});
            check_eq_u(bank_addr,  g_baddr,  {tag, " bank_addr"});
            check_eq_u(bank_wdata, g_bwdata, {tag, " bank_wdata"});

            // --- check 1-cycle-delayed read return against PREVIOUS rdsel ---
            for (jj = 0; jj < NUM_REQ; jj = jj + 1) begin
                if (g_rdsel_v[jj]) begin
                    exp_rd = bank_rdata[g_rdsel[jj]*DATA_W +: DATA_W];
                    check_eq_u(req_rdata[jj*DATA_W +: DATA_W], exp_rd,
                               {tag, " req_rdata"});
                end
            end

            // advance clock
            @(posedge clk);
            // update registered rdsel exactly as the RTL does (on granted reads)
            for (jj = 0; jj < NUM_REQ; jj = jj + 1)
                if (g_grant[jj] & req_rd[jj]) begin
                    g_rdsel[jj]   = req_addr[jj*ADDR_W +: SEL];
                    g_rdsel_v[jj] = 1'b1;
                end
            #1;
        end
    endtask

    integer c, r;

    initial begin
        verif_errors = 0; verif_checks = 0;
        req_act=0; req_rd=0; req_wr=0; req_addr=0; req_wdata=0; bank_wait=0;
        bank_rdata={(NUM_BANKS*DATA_W){1'b0}};
        for (bb = 0; bb < NUM_BANKS; bb = bb + 1)
            for (ww = 0; ww < BANK_WORDS; ww = ww + 1)
                bankmem[bb][ww] = {bb[7:0], 16'hBA5E, ww[7:0]};
        for (jj = 0; jj < NUM_REQ; jj = jj + 1) begin g_rdsel[jj]=0; g_rdsel_v[jj]=0; end
        @(posedge clk); #1;
        $display("=== tb_shared_pool ===");

        // ---------------------------------------------------------------
        // Directed D1: two requesters target the SAME bank (addr%4 equal).
        // Lower index (req0) wins; req1 stalls. Both read.
        // ---------------------------------------------------------------
        req_act = 6'b000011;
        req_rd  = 6'b000011;
        req_wr  = 6'b000000;
        req_addr[0*ADDR_W +: ADDR_W] = 12'd8;   // bank 0, word 2
        req_addr[1*ADDR_W +: ADDR_W] = 12'd12;  // bank 0, word 3  (collides)
        check_cycle("D1 same-bank prio");
        // Expect: req1 waited last cycle, so it holds + retries; here just clear.
        req_act = 0; req_rd = 0;
        check_cycle("D1 idle");
        check_cycle("D1 idle2");   // let req0's read return land + be checked

        // ---------------------------------------------------------------
        // Directed D2: four requesters hit four DIFFERENT banks -> all grant.
        // ---------------------------------------------------------------
        req_act = 6'b001111;
        req_rd  = 6'b001111;
        req_wr  = 6'b000000;
        req_addr[0*ADDR_W +: ADDR_W] = 12'd0;   // bank0 word0
        req_addr[1*ADDR_W +: ADDR_W] = 12'd1;   // bank1 word0
        req_addr[2*ADDR_W +: ADDR_W] = 12'd2;   // bank2 word0
        req_addr[3*ADDR_W +: ADDR_W] = 12'd3;   // bank3 word0
        check_cycle("D2 four banks");
        req_act = 0; req_rd = 0;
        check_cycle("D2 idle");
        check_cycle("D2 idle2");

        // ---------------------------------------------------------------
        // Directed D3: bank_wait on a granted bank passes through to req_wait.
        // ---------------------------------------------------------------
        bank_wait = 4'b0001;       // bank 0 stalled
        req_act = 6'b000001;
        req_rd  = 6'b000000;
        req_wr  = 6'b000001;
        req_addr[0*ADDR_W +: ADDR_W] = 12'd4;   // bank0 word1
        req_wdata[0*DATA_W +: DATA_W] = 32'hF00D_0001;
        check_cycle("D3 bank_wait passthru");
        bank_wait = 0;
        req_act = 0; req_wr = 0;
        check_cycle("D3 idle");

        // ---------------------------------------------------------------
        // Directed D4: a write then a read-back of the same bank word.
        // ---------------------------------------------------------------
        req_act = 6'b000001; req_rd = 0; req_wr = 6'b000001;
        req_addr [0*ADDR_W +: ADDR_W] = 12'd20;  // bank0 word5
        req_wdata[0*DATA_W +: DATA_W] = 32'hCAFE_1234;
        check_cycle("D4 write");
        req_act = 6'b000001; req_rd = 6'b000001; req_wr = 0;
        req_addr[0*ADDR_W +: ADDR_W] = 12'd20;   // read same word
        check_cycle("D4 read issue");
        req_act = 0; req_rd = 0;
        check_cycle("D4 read return");           // req_rdata should be CAFE_1234
        // explicit value check of the returned read
        check_eq_u(req_rdata[0*DATA_W +: DATA_W], 32'hCAFE_1234, "D4 readback value");

        // ---------------------------------------------------------------
        // Constrained-random storm
        // ---------------------------------------------------------------
        void'($urandom(32'h9A11_0001));
        for (c = 0; c < NCYC; c = c + 1) begin
            for (r = 0; r < NUM_REQ; r = r + 1) begin
                req_act[r] = ($urandom_range(99) < 60);
                // each active req is either a read or a write (not both)
                if ($urandom & 1'b1) begin req_rd[r]=1'b1; req_wr[r]=1'b0; end
                else                 begin req_rd[r]=1'b0; req_wr[r]=1'b1; end
                req_addr [r*ADDR_W +: ADDR_W] = $urandom_range(BANK_WORDS*NUM_BANKS-1);
                req_wdata[r*DATA_W +: DATA_W] = $urandom;
            end
            // ~25% of cycles inject some bank back-pressure, else none.
            if ($urandom_range(99) < 25)
                bank_wait = $urandom_range((1<<NUM_BANKS)-1);
            else
                bank_wait = {NUM_BANKS{1'b0}};
            check_cycle("rand");
        end

        `VERIF_EPILOGUE("tb_shared_pool")
    end

    `VERIF_WATCHDOG(5000000)

endmodule
