// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
// =============================================================================
// tb_packer_wide — aggressive unit test for the wide writeback packer.
// Authors: Simon Davidson & Claude
// Created: 2026-06-07
// Last modified: 2026-06-07
//
// packer_wide accepts NUM_LANES results per group and packs them LSB-first into
// a 32-bit word (element k -> bits[k*EB +: EB]).  It accumulates across groups
// that only partially fill a word and emits ONE write per word, asserting
// mem_wr_o on the cycle the top column is written (col_top == EPW-1) or on
// last_i.  This TB drives random groups and checks mem_wr_o / mem_addr_o /
// mem_data_o against an independent SW packer model.
//
// SW model (mirrors the RTL combinationally):
//   g_low   = base_index mod EPW
//   col_top = g_low + (highest active lane index)
//   merged  = sw_buffer | OR_j( active[j] ? lane[j] << ((g_low+j)*EB) : 0 )
//   word_full = (col_top == EPW-1) | last
//   mem_wr  = write & word_full ; mem_data = merged ; word index = g >> log2EPW
//   next sw_buffer = write ? (word_full ? 0 : merged) : sw_buffer
//
// Configs swept via -define: EB in {1,8,16}; LANES in {1,2,4} (LANES<=EPW).
// full_o is tied 0 in the RTL (mem_wait low); a directed check covers it.
// =============================================================================
`timescale 1ns/1ps

`ifndef LANES
 `define LANES 2
`endif
`ifndef EB
 `define EB 8
`endif

module tb_packer_wide;

    localparam integer LANES   = `LANES;
    localparam integer EB      = `EB;
    localparam integer ADDR_BITS = 30;
    localparam integer IDX_BITS  = 10;
    localparam integer IN_DATA_BITS = 32;
    localparam integer EPW     = IN_DATA_BITS / EB;
    localparam integer NGROUP  = 4000;

    reg                       clk, reset;
    reg                       write_i, last_i;
    reg  [IDX_BITS-1:0]       base_index_i;
    reg  [LANES-1:0]          active_i;
    reg  [LANES*EB-1:0]       lane_data_i;
    reg  [ADDR_BITS-1:0]      base_addr_i;

    wire                      mem_wr_o, full_o;
    wire [ADDR_BITS-1:0]      mem_addr_o;
    wire [IN_DATA_BITS-1:0]   mem_data_o;

    integer verif_errors, verif_checks;
    `include "../verif/checks.vh"

    packer_wide #(.ELEM_BITS(EB), .NUM_LANES(LANES),
                  .IDX_BITS(IDX_BITS), .ADDR_BITS(ADDR_BITS),
                  .IN_DATA_BITS(IN_DATA_BITS))
    dut (
        .clk(clk), .reset(reset),
        .write_i(write_i), .last_i(last_i),
        .base_index_i(base_index_i), .active_i(active_i),
        .lane_data_i(lane_data_i), .base_addr_i(base_addr_i),
        .mem_wr_o(mem_wr_o), .mem_addr_o(mem_addr_o), .mem_data_o(mem_data_o),
        .mem_wait_i(1'b0), .full_o(full_o));

    initial clk = 0; always #5 clk = ~clk;

    // -------- SW packer model (combinational mirror) --------
    reg [IN_DATA_BITS-1:0] sw_buf;       // accumulated partial word

    // compute SW expectation for a given drive, returning expected wr/addr/data
    // and the NEXT buffer value.
    reg [IN_DATA_BITS-1:0] sw_new_bits, sw_merged;
    reg [31:0]             sw_glow, sw_col_top, sw_hi_lane;
    reg                    sw_word_full, sw_mem_wr;
    reg [ADDR_BITS-1:0]    sw_addr;
    integer                jj;

    task sw_eval;
        input        [IDX_BITS-1:0] g;
        input        [LANES-1:0]    act;
        input        [LANES*EB-1:0] ld;
        input        [ADDR_BITS-1:0] baddr;
        input                       wr;
        input                       last;
        begin
            sw_glow = g % EPW;
            // assemble new contribution
            sw_new_bits = {IN_DATA_BITS{1'b0}};
            for (jj = 0; jj < LANES; jj = jj + 1)
                if (act[jj])
                    sw_new_bits = sw_new_bits |
                        ((ld[jj*EB +: EB] & ((1<<EB)-1)) << ((sw_glow + jj) * EB));
            // highest active lane (defaults 0 if none active, matching RTL)
            sw_hi_lane = 0;
            for (jj = 0; jj < LANES; jj = jj + 1)
                if (act[jj]) sw_hi_lane = jj;
            sw_col_top   = sw_glow + sw_hi_lane;
            sw_word_full = (sw_col_top == (EPW-1)) | last;
            sw_merged    = sw_buf | sw_new_bits;
            sw_mem_wr    = wr & sw_word_full;
            sw_addr      = baddr + (g >> ((EPW<=1)?0:$clog2(EPW)));
        end
    endtask

    // Drive one group and check the combinational outputs at the posedge sample
    // point, then advance the model buffer in lockstep with the RTL register.
    task drive_group;
        input        [IDX_BITS-1:0] g;
        input        [LANES-1:0]    act;
        input        [LANES*EB-1:0] ld;
        input        [ADDR_BITS-1:0] baddr;
        input                       last;
        input        [255:0]        tag;
        begin
            sw_eval(g, act, ld, baddr, 1'b1, last);
            // drive
            @(negedge clk);
            write_i = 1'b1; last_i = last; base_index_i = g;
            active_i = act; lane_data_i = ld; base_addr_i = baddr;
            #1;  // settle combinational outputs
            // combinational outputs valid now (write_i high)
            check_bit(mem_wr_o, sw_mem_wr, {tag, " mem_wr"});
            if (sw_mem_wr) begin
                check_eq_u(mem_data_o, sw_merged, {tag, " mem_data"});
                check_eq_u(mem_addr_o, sw_addr,   {tag, " mem_addr"});
            end
            // commit at posedge (RTL registers output_buffer_r)
            @(posedge clk); #1;
            sw_buf = sw_word_full ? {IN_DATA_BITS{1'b0}} : sw_merged;
            write_i = 1'b0;
        end
    endtask

    // idle (write_i=0) cycle: buffer must hold
    task idle_cycle;
        begin
            @(negedge clk); write_i = 1'b0; #1;
            check_bit(mem_wr_o, 1'b0, "idle: no write");
            @(posedge clk); #1;     // sw_buf unchanged
        end
    endtask

    integer gi, base, ng, k, full_grp;
    reg [IDX_BITS-1:0] gidx;
    reg [LANES-1:0]    amask;
    reg [LANES*EB-1:0] ldata;
    reg                lastf;

    task reset_dut;
        begin
            reset = 1; write_i = 0; last_i = 0; active_i = 0;
            lane_data_i = 0; base_index_i = 0; base_addr_i = 0;
            @(posedge clk); #1; @(posedge clk); #1; reset = 0; @(posedge clk); #1;
            sw_buf = {IN_DATA_BITS{1'b0}};
        end
    endtask

    initial begin
        verif_errors = 0; verif_checks = 0;
        reset_dut;
        $display("=== tb_packer_wide  EB=%0d LANES=%0d EPW=%0d ===", EB, LANES, EPW);
        void'($urandom(32'hAC03_0000 ^ (EB<<8) ^ LANES));

        // full_o is always 0 (no back-pressure in this design)
        check_bit(full_o, 1'b0, "full_o tied 0");

        // ---- directed: a full word built from consecutive aligned groups ----
        // sweep group bases 0, L, 2L, ... that exactly tile EPW columns; the
        // final group of the word must trigger the write.
        for (full_grp = 0; full_grp < EPW; full_grp = full_grp + LANES) begin
            gidx  = full_grp[IDX_BITS-1:0];
            amask = {LANES{1'b1}};
            ldata = $random;
            lastf = 1'b0;       // rely on col_top == EPW-1 to flush
            drive_group(gidx, amask, ldata, 30'd0, lastf, "D-tile");
        end

        // ---- directed: partial final group flushed by last_i ----
        // a single lane at column 0 of a fresh word, flushed via last_i
        reset_dut;
        drive_group({IDX_BITS{1'b0}}, {{(LANES-1){1'b0}},1'b1},
                    {(LANES*EB){1'b1}} & {LANES{ {(EB){1'b1}} }},
                    30'd5, 1'b1, "D-last-partial");

        // ---- directed: spike-style EB=1 32-wide accumulation ----
        // only meaningful when EB==1: 32 columns, fill across many groups, one
        // write at col 31 / last.  (covered generically by random below.)

        // ---- random groups: many full layers across word boundaries ----
        for (ng = 0; ng < NGROUP; ng = ng + 1) begin
            reset_dut;
            // a random "layer" of N neurons -> groups of LANES, base advancing
            base = $urandom_range(0,3) * 64;           // word-aligned region base
            // number of neurons in the layer (1..40)
            k    = $urandom_range(1, 40);
            for (gi = 0; gi*LANES < k; gi = gi + 1) begin
                gidx  = (gi*LANES) % (EPW * 8);        // keep within a few words
                // active mask: full unless this is the final partial group
                if ((gi+1)*LANES > k) begin
                    amask = {LANES{1'b0}};
                    for (jj = 0; jj < LANES; jj = jj + 1)
                        if (gi*LANES + jj < k) amask[jj] = 1'b1;
                end else
                    amask = {LANES{1'b1}};
                ldata = {$random, $random};
                lastf = ((gi+1)*LANES >= k);
                drive_group(gidx, amask, ldata, base[ADDR_BITS-1:0], lastf, "rand");
                // occasionally interleave an idle cycle (buffer must hold)
                if (($urandom_range(0,3) == 0) && !lastf) idle_cycle;
            end
        end

        `VERIF_EPILOGUE("tb_packer_wide")
    end

    `VERIF_WATCHDOG(20000000)

endmodule
