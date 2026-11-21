// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
// =============================================================================
// tb_syn_curr_accum_wide — aggressive unit test for the wide RMW accumulator.
// Authors: Simon Davidson & Claude
// Created: 2026-06-07
// Last modified: 2026-06-07
//
// syn_curr_accum_wide does a 2-cycle read-modify-write into ONE 32-bit syn_curr
// word for a group of NUM_LANES contiguous output neurons:
//   S_RD: issue read of word (g >> log2EPW)
//   S_WR: new_word[col] = read_word[col] + weight[lane]  (per active lane)
//         write new_word back; done_o pulses when accepted.
//
// SW golden: a flat syn_curr[] element array.  For each committed group, every
// active lane j gets  sc[g+j] = sc[g+j] + weight[j]  (ELEM_BITS-wide two's-
// complement add, wrapping — exactly what the RTL "+" does on the packed slice).
// We drive the weights ALREADY sign-extended to ELEM_BITS (as the parent does),
// and check the whole syn_curr word after each group, plus the byte-exact
// memory image at the end.
//
// Covers: random groups across the address space, random active masks (partial
// final groups), sign boundaries (large +/- weights that wrap the element),
// and mem_wait back-pressure injected on BOTH the read and write phases.
//
// Configs swept via -define: EB in {8,16}; LANES in {1,2,4} (LANES<=EPW).
// =============================================================================
`timescale 1ns/1ps

`ifndef LANES
 `define LANES 2
`endif
`ifndef EB
 `define EB 8
`endif

module tb_syn_curr_accum_wide;

    localparam integer LANES   = `LANES;
    localparam integer EB      = `EB;
    localparam integer ADDR_BITS = 30;
    localparam integer IDX_BITS  = 10;
    localparam integer IN_DATA_BITS = 32;
    localparam integer EPW     = IN_DATA_BITS / EB;
    localparam integer NWORDS  = 64;            // syn_curr image words
    localparam integer NELEM   = NWORDS * EPW;
    localparam integer NGROUP  = 3000;

    reg                       clk, reset;
    reg  [ADDR_BITS-1:0]      base_addr_i;
    reg  [IDX_BITS-1:0]       out_index_i;
    reg  [LANES*EB-1:0]       weights_i;
    reg  [LANES-1:0]          active_i;
    reg                       go_i;
    wire                      busy_o, done_o;

    wire                      sc_rd, sc_wr;
    reg                       sc_wait;
    wire [ADDR_BITS-1:0]      sc_addr;
    reg  [IN_DATA_BITS-1:0]   sc_rdata;
    wire [IN_DATA_BITS-1:0]   sc_wdata;

    integer verif_errors, verif_checks, sram_i;
    `include "../verif/checks.vh"

    syn_curr_accum_wide #(.ELEM_BITS(EB), .NUM_LANES(LANES),
                          .IDX_BITS(IDX_BITS), .ADDR_BITS(ADDR_BITS),
                          .IN_DATA_BITS(IN_DATA_BITS))
    dut (
        .clk(clk), .reset(reset),
        .base_addr_i(base_addr_i), .out_index_i(out_index_i),
        .weights_i(weights_i), .active_i(active_i),
        .go_i(go_i), .busy_o(busy_o), .done_o(done_o),
        .syn_curr_mem_rd_o(sc_rd), .syn_curr_mem_wr_o(sc_wr),
        .syn_curr_mem_wait_i(sc_wait),
        .syn_curr_mem_addr_o(sc_addr),
        .syn_curr_mem_data_i(sc_rdata),
        .syn_curr_mem_data_o(sc_wdata));

    initial clk = 0; always #5 clk = ~clk;

    // -------- syn_curr SRAM (256x32, [7:0] addr trunc, gated by ~wait) --------
    // Read honoured only when ~wait; write honoured only when ~wait (mirrors a
    // memory that stalls under back-pressure).
    reg [31:0] sc_mem [0:255];
    always @(posedge clk) begin
        if (sc_rd & ~sc_wait) sc_rdata <= sc_mem[sc_addr[7:0]];
        if (sc_wr & ~sc_wait) sc_mem[sc_addr[7:0]] <= sc_wdata;
    end

    // -------- SW golden image (mirror of sc_mem at the element level) --------
    reg [31:0] sw_mem [0:255];

    integer i;
    task init_mem;
        begin
            for (i = 0; i < 256; i = i + 1) begin
                sc_mem[i] = $random;
                sw_mem[i] = sc_mem[i];
            end
        end
    endtask

    // Apply one group to the SW image (g must be EPW-aligned within its word for
    // the single-word RMW contract; the parent always issues g as a multiple of
    // NUM_LANES and NUM_LANES<=EPW with EPW-aligned bases).
    reg [IDX_BITS-1:0] base_word;
    reg [31:0]         glow;
    task sw_apply;
        input [ADDR_BITS-1:0] baddr;
        input [IDX_BITS-1:0]  g;
        input [LANES*EB-1:0]  w;
        input [LANES-1:0]     act;
        reg [7:0] waddr;
        reg [EB-1:0] cur, add, nw;
        integer lane;
        begin
            glow  = g % EPW;
            waddr = (baddr + (g >> ((EPW<=1)?0:$clog2(EPW)))) & 8'hFF;
            for (lane = 0; lane < LANES; lane = lane + 1)
                if (act[lane]) begin
                    cur = sw_mem[waddr][(glow+lane)*EB +: EB];
                    add = w[lane*EB +: EB];
                    nw  = cur + add;                 // EB-wide wrap add
                    sw_mem[waddr][(glow+lane)*EB +: EB] = nw;
                end
        end
    endtask

    // Drive one group through the 2-cycle FSM (with optional back-pressure) and
    // verify the written word equals the SW image.
    integer bp;
    task do_group;
        input [ADDR_BITS-1:0] baddr;
        input [IDX_BITS-1:0]  g;
        input [LANES*EB-1:0]  w;
        input [LANES-1:0]     act;
        input                 use_bp;
        input [255:0]         tag;
        reg [7:0] waddr;
        integer guard;
        begin
            sw_apply(baddr, g, w, act);
            waddr = (baddr + (g >> ((EPW<=1)?0:$clog2(EPW)))) & 8'hFF;

            // present request
            @(negedge clk);
            base_addr_i = baddr; out_index_i = g; weights_i = w; active_i = act;
            go_i = 1'b1;
            if (use_bp) bp = 1; else bp = 0;

            // wait for done_o (the write-accept pulse), bounded
            guard = 0;
            while (!done_o && guard < 200) begin @(posedge clk); guard = guard + 1; end
            check_true(guard < 200, {tag, " done within bound"});
            // done_o is high now; deassert go on the next negedge
            @(negedge clk); go_i = 1'b0; bp = 0;
            @(posedge clk); #1;                  // let write commit into sc_mem
            // verify committed word
            check_eq_u(sc_mem[waddr], sw_mem[waddr], {tag, " word"});
        end
    endtask

    // random back-pressure on the memory during a group
    always @(posedge clk) sc_wait <= (bp && ($urandom_range(99) < 40));

    integer gi, ng, k, base_w, full_grp;
    reg [IDX_BITS-1:0]  gidx;
    reg [LANES-1:0]     amask;
    reg [LANES*EB-1:0]  wvec;
    integer             lane;
    reg signed [EB-1:0] wsigned;

    task reset_dut;
        begin
            reset = 1; go_i = 0; base_addr_i = 0; out_index_i = 0;
            weights_i = 0; active_i = 0; bp = 0; sc_wait = 0;
            @(posedge clk); #1; @(posedge clk); #1; reset = 0; @(posedge clk); #1;
        end
    endtask

    initial begin
        verif_errors = 0; verif_checks = 0; bp = 0; sc_wait = 0;
        reset_dut;
        init_mem;
        $display("=== tb_syn_curr_accum_wide  EB=%0d LANES=%0d EPW=%0d ===", EB, LANES, EPW);
        void'($urandom(32'h5CA0_0000 ^ (EB<<8) ^ LANES));

        // ---- directed: a full layer of NELEM outputs swept in groups of LANES,
        //      uniform weight, init from random image (single contiguous sweep) ----
        for (full_grp = 0; full_grp + LANES <= EPW*4; full_grp = full_grp + LANES) begin
            wvec = {LANES{ {(EB){1'b0}} | 8'd1 }};         // +1 each lane (EB-wide)
            do_group(30'd0, full_grp[IDX_BITS-1:0], wvec, {LANES{1'b1}}, 1'b0, "D-sweep");
        end

        // ---- directed: sign boundaries (max positive, max negative weights) ----
        // +max: 0x7F.. at EB; should wrap the element on overflow exactly as +.
        wvec = {LANES{ {1'b0, {(EB-1){1'b1}}} }};          // +(2^(EB-1)-1)
        do_group(30'd0, {IDX_BITS{1'b0}}, wvec, {LANES{1'b1}}, 1'b0, "D-posmax");
        wvec = {LANES{ {1'b1, {(EB-1){1'b0}}} }};          // -(2^(EB-1))
        do_group(30'd0, {IDX_BITS{1'b0}}, wvec, {LANES{1'b1}}, 1'b0, "D-negmax");

        // ---- directed: back-pressure on a single group ----
        wvec = {LANES{ {(EB){1'b0}} | 8'd3 }};
        do_group(30'd0, {IDX_BITS{1'b0}}, wvec, {LANES{1'b1}}, 1'b1, "D-bp");

        // ---- random groups across the address space, partial masks, +/- weights, bp ----
        for (ng = 0; ng < NGROUP; ng = ng + 1) begin
            base_w = $urandom_range(0,3) * 16;                 // word region base
            // a layer of k outputs -> groups of LANES
            k = $urandom_range(1, EPW*3);
            for (gi = 0; gi*LANES < k; gi = gi + 1) begin
                gidx = (gi*LANES) % (EPW*3);                   // EPW-aligned group base
                // build per-lane signed weight vector
                wvec = {(LANES*EB){1'b0}};
                for (lane = 0; lane < LANES; lane = lane + 1) begin
                    wsigned = $urandom_range(0,(1<<EB)-1) - (1<<(EB-1));
                    wvec[lane*EB +: EB] = wsigned;
                end
                // active mask (final partial group)
                if ((gi+1)*LANES > k) begin
                    amask = {LANES{1'b0}};
                    for (lane = 0; lane < LANES; lane = lane + 1)
                        if (gi*LANES + lane < k) amask[lane] = 1'b1;
                end else
                    amask = {LANES{1'b1}};
                do_group(base_w[ADDR_BITS-1:0], gidx, wvec, amask,
                         ($urandom_range(0,2)==0), "rand");
            end
        end

        // ---- final byte-exact image compare ----
        for (i = 0; i < 256; i = i + 1)
            check_eq_u(sc_mem[i], sw_mem[i], "final image");

        `VERIF_EPILOGUE("tb_syn_curr_accum_wide")
    end

    `VERIF_WATCHDOG(40000000)

endmodule
