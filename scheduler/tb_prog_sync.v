// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
// =============================================================================
// tb_prog_sync -- PROG_MEM_SYNC: the scheduler fetching from a 1-cycle SRAM
//
// Authors      : Simon Davidson & Claude
// Created      : 2026-09-29
// Last modified: 2026-09-29
//
// Three scheduler lanes run in LOCKSTEP on one stimulus (host bus, stall and
// cm_busy streams). Each lane has its own program store and accelerator stubs,
// and the stubs are deterministic, so two lanes can only stay in step if their
// outputs agree every cycle:
//
//   ref  PROG_MEM_SYNC=0, combinational store  -- today's design (the oracle)
//   dut  PROG_MEM_SYNC=1, registered 1RW store that HOLDS its output between
//        reads (the monarch_sram32_prog contract)
//   neg  PROG_MEM_SYNC=0 on the registered store -- the Bosch 7-Aug failure.
//        MUST diverge, or this bench cannot see the bug it exists to catch.
//
//   P1  dut == ref every cycle: dispatches, packed entries, FILL operands, NXT
//       pulses, PC and consume strobes (i.e. ZERO cycle cost, not just the
//       same results).
//   P2  neg != ref (negative control).
//   P3  both runs actually finish (STOP reached, expected dispatch count).
//
// Program mixes every instruction form -- narrow TASK, 3-word wide TASK, FILL,
// JUMP over poison words, CHECK, LOOP/LOOPEND, NXT -- with random stalls
// and cm_busy, then a SOFT_RESET, a program rewrite, LOAD_PC and a restart,
// with a PAUSE/UNPAUSE mid-run.
// =============================================================================
`timescale 1ns/1ps

// ---- deterministic accelerator stub ------------------------------------------
// Latency is a function of this accelerator's own task count, so lanes that
// dispatch identically see identical completions.
module ps_acc #(parameter ACC_ID = 0, parameter TGT_ACC_SZ = 3) (
    input  wire                  clk, reset,
    input  wire                  start_i,
    input  wire [TGT_ACC_SZ-1:0] target_acc_i,
    output reg                   busy_o,
    output reg                   finished_o
);
    reg [5:0] cnt;
    reg [7:0] ntask;
    always @(posedge clk) begin
        if (reset) begin
            busy_o <= 0; finished_o <= 0; cnt <= 0; ntask <= 0;
        end else begin
            finished_o <= 0;
            if (start_i && target_acc_i == ACC_ID && !busy_o) begin
                busy_o <= 1;
                cnt    <= 2 + ((ntask * 7 + ACC_ID * 3) % 13);
                ntask  <= ntask + 1;
            end else if (busy_o) begin
                if (cnt == 1) begin busy_o <= 0; finished_o <= 1; end
                cnt <= cnt - 1;
            end
        end
    end
endmodule

// ---- one scheduler + its program store + its accelerators --------------------
module ps_lane #(parameter PROG_MEM_SYNC = 0, parameter STORE_SYNC = 0) (
    input  wire        clk, reset,
    input  wire        sys_req, input wire [31:0] sys_addr, input wire [31:0] sys_data,
    input  wire        stall, input wire cm_busy,
    output wire        start_new_block,
    output wire [2:0]  target_acc,
    output wire [72:0] buffer_info,
    output wire        nxt_in, nxt_out,
    output wire [31:0] fill_value,
    output wire [19:0] fill_block_size,
    output reg  [31:0] nreads
);
    localparam PAB = 10, NACC = 5;
    wire [31:0]    pm_addr;
    wire           pm_rd, pm_wr;
    wire [PAB-1:0] pm_waddr;
    wire [31:0]    pm_wdata;
    wire [31:0]    pm_data;

    reg [31:0] mem [0:(1<<PAB)-1];
    integer i; initial for (i = 0; i < (1<<PAB); i = i + 1) mem[i] = 32'h0000_0002; // STOP

    generate if (STORE_SYNC) begin : g_sync
        // 1RW, write-priority; output HELD when not reading; X after a write
        // cycle so that consuming a lost read is visible.
        reg [31:0] q;
        always @(posedge clk) begin
            if (pm_wr) begin mem[pm_waddr] <= pm_wdata; q <= 32'hxxxx_xxxx; end
            else if (pm_rd) q <= mem[pm_addr[PAB-1:0]];
        end
        assign pm_data = q;
    end else begin : g_comb
        always @(posedge clk) if (pm_wr) mem[pm_waddr] <= pm_wdata;
        assign pm_data = mem[pm_addr[PAB-1:0]];
    end endgenerate

    always @(posedge clk) if (reset) nreads <= 0; else if (pm_rd) nreads <= nreads + 1;

    wire [NACC-1:0] busy, fin;
    genvar k;
    generate for (k = 0; k < NACC; k = k + 1) begin : g_acc
        ps_acc #(.ACC_ID(k)) u (.clk(clk), .reset(reset), .start_i(start_new_block),
                                .target_acc_i(target_acc), .busy_o(busy[k]), .finished_o(fin[k]));
    end endgenerate

    scheduler #(
        .TGT_ACC_SZ(3), .TGT_COUNT_SZ(7), .WIDE_NTGT(1), .CFG_ID_SZ(12),
        .NUM_BUFFERS(16), .COL_BUFF_ID_SZ(16), .NUM_SCH_ENTRIES(4),
        .NUM_HW_ACCELERATORS(NACC), .PROG_ADDR_BITS(PAB), .PROG_DATA_BITS(32),
        .BUFF_INDX_SZ(4), .PROG_MEM_SYNC(PROG_MEM_SYNC)
    ) sch (
        .clk(clk), .reset(reset), .test_stall_pipe(stall),
        .sys_req_i(sys_req), .sys_ack_o(), .sys_addr_i(sys_addr), .sys_data_i(sys_data),
        .sys_data_o(),
        .prog_mem_addr_o(pm_addr), .prog_mem_data_i(pm_data),
        .prog_mem_req_o(), .prog_mem_wait_i(1'b0), .prog_mem_rd_o(pm_rd),
        .prog_mem_wr_o(pm_wr), .prog_mem_wr_addr_o(pm_waddr), .prog_mem_wr_data_o(pm_wdata),
        .prog_mem_wr_wait_i(1'b0),
        .acc_busy_i(busy), .acc_finished_i(fin), .acc_result_i({NACC{1'b0}}),
        .acc_ready_next_i({NACC{1'b0}}),
        .start_new_block_o(start_new_block), .target_acc_o(target_acc),
        .buffer_info_o(buffer_info),
        .nxt_input_pulse_o(nxt_in), .nxt_output_pulse_o(nxt_out),
        .fill_value_o(fill_value), .fill_block_size_o(fill_block_size),
        .cm_busy_i(cm_busy),
        .dbg_frontend_o(), .dbg_inst_word_o()
    );
endmodule

module tb_prog_sync;

localparam MODE_UNUSED = 2'b00, MODE_SRC = 2'b01, MODE_RW = 2'b10, MODE_TGT = 2'b11;
localparam CFG_ID_SZ = 12, SLOT_LONG_SZ = 13;

// ---- encoders (mirror tb_sch_wide / tools/flexman_backend/isa.py) ------------
function [31:0] tw1; input [1:0] acc; input [6:0] cfg; input colour;
    input [1:0] m0; input [3:0] id0; input [1:0] m1; input [3:0] id1; input [1:0] m2; input [3:0] id2;
    tw1 = {1'b0, id2, m2, id1, m1, id0, m0, colour, cfg, acc, 3'b000}; endfunction
function [31:0] tw2; input [1:0] m3; input [3:0] id3; input [3:0] n3;
    input [1:0] m4; input [3:0] id4; input [3:0] n4; input [1:0] m5; input [3:0] id5; input [3:0] n5;
    tw2 = {n5, id5, m5, n4, id4, m4, n3, id3, m3, 2'b00}; endfunction
function [31:0] tw1w; input [1:0] acc; input colour;
    input [1:0] m0; input [3:0] id0;
    tw1w = {1'b1, 4'd0, MODE_UNUSED, 4'd0, MODE_UNUSED, id0, m0, colour, 7'd0, acc, 3'b000}; endfunction
function [31:0] tw2w; input [1:0] m3; input [3:0] id3; input [6:0] n3;
    tw2w = {4'b0000, {7'd0,4'd0,MODE_UNUSED}, {n3,id3,m3}, 2'b00}; endfunction
function [31:0] tw3w; input [CFG_ID_SZ-1:0] cfg; input hint;
    tw3w = {hint, {(31-SLOT_LONG_SZ-CFG_ID_SZ){1'b0}}, cfg, {7'd0,4'd0,MODE_UNUSED}}; endfunction
// FILL, WIDE_NTGT=1 build: [31:16] block, [15:9] ntgt, [8] colour, [6:3] buf
function [31:0] fw1; input [3:0] buf_id; input [6:0] ntgt; input [15:0] blk;
    fw1 = {blk, ntgt, 1'b0, 1'b0, buf_id, 3'b101}; endfunction
function [31:0] jmp;  input [9:0] tgt; jmp  = {19'd0, tgt, 3'b001}; endfunction
function [31:0] chk;  input [3:0] id; input mode; input [9:0] skip;
    chk = {10'd0, skip, 4'd0, id, mode, 3'b011}; endfunction
function [31:0] loop_i; input [2:0] id; input [25:0] n; loop_i = {n, id, 3'b110}; endfunction
function [31:0] lend; input [2:0] id; lend = {26'd0, id, 3'b111}; endfunction
function [31:0] nxt;  input in_p, out_p; nxt = {26'd0, out_p, in_p, 1'b0, 3'b100}; endfunction
localparam [31:0] STOP = 32'h0000_0002;
localparam [31:0] POISON = 32'h1111_1111;   // a JUMP to 546 if ever decoded

// ---- clock, reset, shared stimulus -------------------------------------------
reg clk = 0; always #5 clk = ~clk;
reg reset = 1;
reg sys_req = 0; reg [31:0] sys_addr = 0, sys_data = 0;
reg [15:0] lfsr = 16'hACE1;
reg stall_en = 0;
wire stall   = stall_en & (lfsr[3:0] == 4'd0);   // ~6%
wire cm_busy = stall_en & (lfsr[9:7] == 3'd0);   // ~12%
always @(posedge clk) lfsr <= {lfsr[14:0], lfsr[15]^lfsr[13]^lfsr[12]^lfsr[10]};

`define LANE_PORTS(p) .clk(clk), .reset(reset), .sys_req(sys_req), .sys_addr(sys_addr), \
    .sys_data(sys_data), .stall(stall), .cm_busy(cm_busy), \
    .start_new_block(p``_snb), .target_acc(p``_acc), .buffer_info(p``_bi), \
    .nxt_in(p``_ni), .nxt_out(p``_no), .fill_value(p``_fv), .fill_block_size(p``_fb), .nreads(p``_nr)

wire r_snb, d_snb, n_snb, r_ni, d_ni, n_ni, r_no, d_no, n_no;
wire [2:0] r_acc, d_acc, n_acc; wire [72:0] r_bi, d_bi, n_bi;
wire [31:0] r_fv, d_fv, n_fv, r_nr, d_nr, n_nr; wire [19:0] r_fb, d_fb, n_fb;

ps_lane #(.PROG_MEM_SYNC(0), .STORE_SYNC(0)) ref_l (`LANE_PORTS(r));
ps_lane #(.PROG_MEM_SYNC(1), .STORE_SYNC(1)) dut_l (`LANE_PORTS(d));
ps_lane #(.PROG_MEM_SYNC(0), .STORE_SYNC(1)) neg_l (`LANE_PORTS(n));

// ---- lockstep comparison -----------------------------------------------------
function [191:0] sig; input snb; input [2:0] acc; input [72:0] bi; input ni, no;
    input [31:0] fv; input [19:0] fb; input [9:0] pc; input cons;
    sig = {snb, acc, snb ? bi : 73'd0, ni, no, snb ? fv : 32'd0, snb ? fb : 20'd0, pc, cons};
endfunction
wire [191:0] r_sig = sig(r_snb, r_acc, r_bi, r_ni, r_no, r_fv, r_fb,
                         ref_l.sch.prog_counter_r, ref_l.sch.inst_consumed);
wire [191:0] d_sig = sig(d_snb, d_acc, d_bi, d_ni, d_no, d_fv, d_fb,
                         dut_l.sch.prog_counter_r, dut_l.sch.inst_consumed);
wire [191:0] n_sig = sig(n_snb, n_acc, n_bi, n_ni, n_no, n_fv, n_fb,
                         neg_l.sch.prog_counter_r, neg_l.sch.inst_consumed);

integer cyc = 0, d_mis = 0, n_mis = 0, n_first = -1, r_disp = 0, d_disp = 0, xs = 0;
always @(posedge clk) if (!reset) begin
    cyc = cyc + 1;
    if (r_snb) r_disp = r_disp + 1;
    if (d_snb) d_disp = d_disp + 1;
    if (^d_sig === 1'bx) xs = xs + 1;
    if (d_sig !== r_sig) begin
        if (d_mis < 5)
            $display("[%0t] P1 MISMATCH pc ref=%0d dut=%0d snb %b/%b acc %0d/%0d",
                     $time, ref_l.sch.prog_counter_r, dut_l.sch.prog_counter_r,
                     r_snb, d_snb, r_acc, d_acc);
        d_mis = d_mis + 1;
    end
    if (n_sig !== r_sig) begin
        if (n_first < 0) n_first = cyc;
        n_mis = n_mis + 1;
    end
end

task axi_write(input [31:0] a, input [31:0] d);
    begin @(posedge clk); #1; sys_addr = a; sys_data = d; sys_req = 1;
          @(posedge clk); #1; sys_req = 0; end
endtask
task pw(input [9:0] a, input [31:0] d); axi_write(32'hD000_0000 | {a, 2'b00}, d); endtask
task ctrl(input [4:0] op, input [31:0] d); axi_write(32'hE000_0000 | {op, 20'd0}, d); endtask

integer a;
integer checks = 0, errors = 0;
task check(input cond, input [8*64-1:0] tag);
    begin checks = checks + 1;
          if (!cond) begin errors = errors + 1; $display("FAIL %0s", tag); end
          else $display("ok   %0s", tag); end
endtask

initial begin
    repeat (4) @(posedge clk); #1 reset = 0;
    repeat (2) @(posedge clk);

    // ---- program 1, loaded through the host bus (exercises the 1RW write path)
    a = 0;
    pw(a, fw1(4'd1, 7'd1, 16'd40));  a=a+1;  pw(a, 32'hDEAD_BEEF);   a=a+1;   // FILL b1
    pw(a, tw1(2'd0, 7'd3, 0, MODE_SRC,4'd1, MODE_UNUSED,0, MODE_UNUSED,0)); a=a+1; // consume b1
    pw(a, tw2(MODE_TGT,4'd2,4'd1, MODE_UNUSED,0,0, MODE_UNUSED,0,0));       a=a+1; //  -> b2
    pw(a, chk(4'd2, 1'b0, 10'd8));   a=a+1;                                  // 4: CHECK b2
    pw(a, tw1(2'd1, 7'd5, 0, MODE_UNUSED,0, MODE_UNUSED,0, MODE_UNUSED,0)); a=a+1;
    pw(a, tw2(MODE_UNUSED,0,0, MODE_UNUSED,0,0, MODE_UNUSED,0,0));          a=a+1;
    pw(a, nxt(1'b1, 1'b0));          a=a+1;                                  // 7: fall-through only
    pw(a, tw1(2'd2, 7'd0, 0, MODE_SRC,4'd2, MODE_UNUSED,0, MODE_UNUSED,0)); a=a+1; // 8: consume b2
    pw(a, tw2(MODE_UNUSED,0,0, MODE_UNUSED,0,0, MODE_UNUSED,0,0));          a=a+1;
    pw(a, loop_i(3'd0, 26'd5));      a=a+1;                                  // 10: LOOP x6
    //   body: wide TASK, narrow TASK, JUMP over poison, NXT
    pw(a, tw1w(2'd3, 0, MODE_UNUSED, 0));       a=a+1;
    pw(a, tw2w(MODE_UNUSED, 0, 7'd0));          a=a+1;
    pw(a, tw3w(12'd964, 1'b1));                 a=a+1;
    pw(a, tw1(2'd0, 7'd9, 0, MODE_UNUSED,0, MODE_UNUSED,0, MODE_UNUSED,0)); a=a+1;
    pw(a, tw2(MODE_UNUSED,0,0, MODE_UNUSED,0,0, MODE_UNUSED,0,0));          a=a+1;
    pw(a, jmp(10'd19));              a=a+1;                                  // 16
    pw(a, POISON);                   a=a+1;
    pw(a, POISON);                   a=a+1;
    pw(a, nxt(1'b0, 1'b1));          a=a+1;                                  // 19
    pw(a, tw1(2'd1, 7'd1, 0, MODE_UNUSED,0, MODE_UNUSED,0, MODE_UNUSED,0)); a=a+1;
    pw(a, tw2(MODE_UNUSED,0,0, MODE_UNUSED,0,0, MODE_UNUSED,0,0));          a=a+1;
    pw(a, lend(3'd0));               a=a+1;                                  // 22
    pw(a, STOP);                     a=a+1;

    // START on the cycle after the last program write (tightest case)
    stall_en = 1;
    ctrl(5'd0, 32'd0);    // LOAD_PC 0
    ctrl(5'd1, 32'd0);    // START
    repeat (150) @(posedge clk);
    ctrl(5'd3, 32'd0);    // PAUSE
    repeat (37) @(posedge clk);
    ctrl(5'd4, 32'd0);    // UNPAUSE
    repeat (3000) @(posedge clk);
    check(ref_l.sch.inst_is_stop && ref_l.sch.prog_counter_r == 23,
          "P3a program 1 reached its STOP (pc 23)");

    // ---- SOFT_RESET, rewrite a region, LOAD_PC elsewhere, restart ------------
    ctrl(5'd6, 32'd0);    // SOFT_RESET
    a = 100;
    pw(a, loop_i(3'd1, 26'd9)); a=a+1;
    pw(a, fw1(4'd5, 7'd1, 16'd3)); a=a+1; pw(a, 32'h0000_0105);             a=a+1;
    pw(a, tw1(2'd2, 7'd4, 0, MODE_SRC,4'd5, MODE_UNUSED,0, MODE_UNUSED,0)); a=a+1; // consume b5
    pw(a, tw2(MODE_UNUSED,0,0, MODE_UNUSED,0,0, MODE_UNUSED,0,0));          a=a+1;
    pw(a, lend(3'd1));          a=a+1;
    pw(a, nxt(1'b1, 1'b1));     a=a+1;
    pw(a, STOP);                a=a+1;
    ctrl(5'd0, 32'd100);  // LOAD_PC 100
    repeat (3) @(posedge clk);
    ctrl(5'd1, 32'd0);    // START
    repeat (3000) @(posedge clk);
    check(ref_l.sch.inst_is_stop && ref_l.sch.prog_counter_r == 107,
          "P3d program 2 reached its STOP (pc 107)");

    check(d_mis == 0,  "P1  dut == ref every cycle (zero-cycle-cost equivalence)");
    check(xs == 0,     "P1b dut outputs never X");
    check(n_mis > 0,   "P2  negative control (no look-ahead) DIVERGES");
    check(r_disp > 20, "P3b ref dispatched a non-trivial workload");
    check(d_disp == r_disp, "P3c dut dispatch count == ref");
    $display("cycles %0d, dispatches ref %0d dut %0d, neg first divergence at cycle %0d (%0d cycles differ)",
             cyc, r_disp, d_disp, n_first, n_mis);
    $display("program-store reads: ref %0d (comb, per request)  dut %0d (SRAM, on PC change)",
             r_nr, d_nr);
    if (errors == 0) $display("=== tb_prog_sync: %0d check(s), 0 failure(s) ===\nPASS", checks);
    else             $display("=== tb_prog_sync: %0d check(s), %0d FAILURE(S) ===\nFAIL", checks, errors);
    $finish;
end

initial begin #2000000; $display("FAIL: timeout"); $finish; end

endmodule
