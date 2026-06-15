// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
// =============================================================================
// tb_hadamard_unit  --  Hadamard accelerator TOP-LEVEL integration testbench
//
// Authors      : Simon Davidson & Claude
// Created      : 2026-05 (original 3-case smoke test)
// Last modified: 2026-06-15
//
// Exercises the full datapath end-to-end:
//   config regs -> stream_generator (x4) -> hu_compute -> packer -> R memory.
//
// Per element:  R = clamp( round_half_up( Z*(A-B) + B + mode*R_prev ) )
// to the signed elem_sz_r range, left-aligned for the packer.
//
// The SOFTWARE GOLDEN below is ported verbatim from tb_hu_compute.v's golden()
// task (the corrected F4/F5 fixed-point reference): it aligns A,B,R_prev to the
// internal binary point HALF=24, does an EXACT chunked z_int*amb multiply at
// (bp_z+HALF), adds B/R_prev<<bp_z, shifts to bp_r with round-half-up, clamps
// to the signed element range, and re-left-aligns.  Because the chunked
// multiply is exact, this golden equals the fixed RTL bit-for-bit.
//
// Coverage:
//   * Tests 1-3 : original smoke cases (Z=0 / Z=0+Rprev / A=B) -- KEPT.
//   * D4..D11   : DIRECTED real-multiply vectors that ASSERT the TRUE value
//                 (R=17, +127 clamp, -128 clamp, mode1 Rprev=22, bp!=0, 16-bit),
//                 with PER-ELEMENT-DISTINCT A/B/Z across the stream so each is a
//                 genuine multiply (element k gets A=base+k, etc.), all checked.
//   * DW1..DW3  : MULTI-WORD directed cases (16-bit 3-word, 32-bit 3-word,
//                 8-bit 2-word) that assert values landing in NON-ZERO output
//                 words -- the F7 multi-word + 32-bit addressing proof.
//   * RND       : constrained-random loop: random MULTI-WORD stream_len, random
//                 elem_sz (incl. 32-bit), bin_pts, mode and per-element
//                 A/B/Z/R_prev; every output element word checked vs golden.
//
// PASS/FAIL via verif/checks.vh epilogue (prints literal "PASS"/"FAIL").
//
// F7 (FIXED 2026-06-15): hadamard_unit previously fed the packer the cache's
// PER-WORD slice index instead of the GLOBAL element index, so (1) multi-word
// streams wrote every output word to base+0 and (2) 32-bit elements emitted an
// X address (dataline_cache_with_xy.slice_idx had no 'b101 case). Fix: the
// stream generator exposes its global `index` (data_global_idx_o) and
// hadamard_unit drives the packer pak_index_i from it; the cache gains the
// 'b101 slice_idx case. Multi-word + 32-bit are now scored here (DW1-DW3, RND).
// See verif/FINDINGS.md F7.
//
// Note: in update mode the DUT reads R_prev from and writes R back to the SAME
// R memory, so mem_r is clobbered before scoring -> golden R_prev is taken from
// a pre-task snapshot (mem_rprev). Consecutive tasks reset the DUT between runs
// because the 1-entry stream caches are only invalidated by reset (cache_hit
// ignores base_addr; direct mem writes do not invalidate).
// =============================================================================
`timescale 1ns/1ps
`include "../shared/constants.v"

module tb_hadamard_unit;

localparam TGT_ACC_ID   = 0;
localparam TGT_CFG_BASE = 16'h1D1D;
localparam DATA_BITS    = 32;
localparam ADDR_SZ      = `ADDR_SIZE;
localparam PIN_BITS     = `PIN_BITS;
localparam TGT_ACC_SZ   = `TGT_ACC_SZ;
localparam SCH_ENTRY_SZ = `SCH_ENTRY_SZ;
localparam NUM_ELEM_SZ  = 16;
localparam ACT_SZ       = `POT_OUT_SZ_SZ;   // 3
localparam BINPT_SZ     = 5;

// Golden-arithmetic widths (must match tb_hu_compute.v / hu_compute.v)
localparam integer WIDE     = DATA_BITS + 16;       // 48
localparam integer HALF     = WIDE/2;               // 24
localparam integer ACC_BITS = WIDE + DATA_BITS + 4; // 84

reg clk, reset;

reg            hu_sys_req_i;
wire           hu_sys_ack_o;
reg  [31:0]    hu_sys_addr_i;
reg  [31:0]    hu_sys_data_i;

reg  [TGT_ACC_SZ-1:0]   hu_target_acc_i;
reg  [SCH_ENTRY_SZ-1:0] hu_buffer_info_i;
reg                      hu_start_new_block_i;
wire                     hu_acc_busy_o;
wire                     hu_acc_finished_o;

wire [ADDR_SZ-1:0]   src_a_mem_addr_o;
wire                 src_a_mem_rd_o;
reg  [DATA_BITS-1:0] src_a_mem_data_i;
reg                  src_a_mem_wait_i;

wire [ADDR_SZ-1:0]   src_b_mem_addr_o;
wire                 src_b_mem_rd_o;
reg  [DATA_BITS-1:0] src_b_mem_data_i;
reg                  src_b_mem_wait_i;

wire [ADDR_SZ-1:0]   src_z_mem_addr_o;
wire                 src_z_mem_rd_o;
reg  [DATA_BITS-1:0] src_z_mem_data_i;
reg                  src_z_mem_wait_i;

wire [ADDR_SZ-1:0]   src_r_mem_addr_o;
wire                 src_r_mem_rd_o;
reg  [DATA_BITS-1:0] src_r_mem_data_i;
reg                  src_r_mem_wait_i;

wire                 src_r_mem_wr_o;
wire [ADDR_SZ-1:0]   src_r_mem_wr_addr_o;
reg                  src_r_mem_wr_wait_i;
wire [DATA_BITS-1:0] src_r_mem_data_o;

reg [DATA_BITS-1:0] mem_a [0:255];
reg [DATA_BITS-1:0] mem_b [0:255];
reg [DATA_BITS-1:0] mem_z [0:255];
reg [DATA_BITS-1:0] mem_r [0:255];
// Snapshot of mem_r taken just before start_task: in update mode the DUT both
// READS R_prev from and WRITES the result back to the SAME R memory, so the
// live mem_r is clobbered before we score. The golden's R_prev MUST come from
// this pre-task snapshot.
reg [DATA_BITS-1:0] mem_rprev [0:255];

integer i;

// verif/checks.vh self-checking primitives + PASS/FAIL epilogue
integer verif_errors, verif_checks;
`include "../verif/checks.vh"

hadamard_unit #(
    .TGT_ACC_ID           (TGT_ACC_ID),
    .TGT_CONFIG_BASE_ADDR (TGT_CFG_BASE),
    .MAX_STREAM_LEN       (1024),
    .ADDR_SZ              (ADDR_SZ),
    .ACT_SZ              (`POT_OUT_SZ_SZ),
    .TGT_ACC_SZ           (TGT_ACC_SZ),
    .SCH_ENTRY_SZ         (SCH_ENTRY_SZ),
    .PIN_BITS             (PIN_BITS),
    .NUM_ELEM_SZ          (NUM_ELEM_SZ)
) dut (
    .clk                  (clk),
    .reset                (reset),
    .hu_sys_req_i         (hu_sys_req_i),
    .hu_sys_ack_o         (hu_sys_ack_o),
    .hu_sys_addr_i        (hu_sys_addr_i),
    .hu_sys_data_i        (hu_sys_data_i),
    .hu_start_new_block_i (hu_start_new_block_i),
    .hu_target_acc_i      (hu_target_acc_i),
    .hu_buffer_info_i     (hu_buffer_info_i),
    .hu_acc_busy_o        (hu_acc_busy_o),
    .hu_acc_finished_o    (hu_acc_finished_o),
    .src_a_mem_rd_o       (src_a_mem_rd_o),
    .src_a_mem_wait_i     (src_a_mem_wait_i),
    .src_a_mem_addr_o     (src_a_mem_addr_o),
    .src_a_mem_data_i     (src_a_mem_data_i),
    .src_b_mem_rd_o       (src_b_mem_rd_o),
    .src_b_mem_wait_i     (src_b_mem_wait_i),
    .src_b_mem_addr_o     (src_b_mem_addr_o),
    .src_b_mem_data_i     (src_b_mem_data_i),
    .src_z_mem_rd_o       (src_z_mem_rd_o),
    .src_z_mem_wait_i     (src_z_mem_wait_i),
    .src_z_mem_addr_o     (src_z_mem_addr_o),
    .src_z_mem_data_i     (src_z_mem_data_i),
    .src_r_mem_rd_o       (src_r_mem_rd_o),
    .src_r_mem_wait_i     (src_r_mem_wait_i),
    .src_r_mem_addr_o     (src_r_mem_addr_o),
    .src_r_mem_data_i     (src_r_mem_data_i),
    .src_r_mem_wr_o       (src_r_mem_wr_o),
    .src_r_mem_wr_addr_o  (src_r_mem_wr_addr_o),
    .src_r_mem_wr_wait_i  (src_r_mem_wr_wait_i),
    .src_r_mem_data_o     (src_r_mem_data_o)
);

always #5 clk = ~clk;


// 1-cycle read latency memory models
always @(posedge clk) begin
    if (src_a_mem_rd_o && !src_a_mem_wait_i) src_a_mem_data_i <= mem_a[src_a_mem_addr_o[7:0]];
    if (src_b_mem_rd_o && !src_b_mem_wait_i) src_b_mem_data_i <= mem_b[src_b_mem_addr_o[7:0]];
    if (src_z_mem_rd_o && !src_z_mem_wait_i) src_z_mem_data_i <= mem_z[src_z_mem_addr_o[7:0]];
    if (src_r_mem_rd_o && !src_r_mem_wait_i) src_r_mem_data_i <= mem_r[src_r_mem_addr_o[7:0]];
end

// Capture R write-back
always @(posedge clk)
    if (src_r_mem_wr_o && !src_r_mem_wr_wait_i)
        mem_r[src_r_mem_wr_addr_o[7:0]] <= src_r_mem_data_o;

// =====================================================================
//  Software golden helpers  (ported from tb_hu_compute.v)
// =====================================================================
function [5:0] rsh;                 // right_shift_for_sz (left-align amount)
    input [ACT_SZ-1:0] sz;
    begin
        case (sz)
            3'd0: rsh = 6'd31; 3'd1: rsh = 6'd30; 3'd2: rsh = 6'd28;
            3'd3: rsh = 6'd24; 3'd4: rsh = 6'd16; 3'd5: rsh = 6'd0;
            default: rsh = 6'd0;
        endcase
    end
endfunction

// element width in bits for an elem_sz code
function integer ewidth;
    input [ACT_SZ-1:0] sz;
    begin
        case (sz)
            3'd0: ewidth = 1;  3'd1: ewidth = 2;  3'd2: ewidth = 4;
            3'd3: ewidth = 8;  3'd4: ewidth = 16; 3'd5: ewidth = 32;
            default: ewidth = 32;
        endcase
    end
endfunction

function [2:0] mul_total_of;
    input [ACT_SZ-1:0] sz;
    begin
        case (sz)
            3'd4: mul_total_of = 3'd2;
            3'd5: mul_total_of = 3'd4;
            default: mul_total_of = 3'd1;
        endcase
    end
endfunction

// left-align a value into the top of a 32-bit slice word (single element)
function [DATA_BITS-1:0] la;
    input signed [31:0] v;
    input [ACT_SZ-1:0]  esz;
    begin
        la = v << rsh(esz);
    end
endfunction

// sign-extend a DATA_BITS (left-aligned) value to WIDE, arith-shift right by
// rsh, then left-shift to put the binary point at bit HALF.
function signed [WIDE-1:0] align_wide;
    input [DATA_BITS-1:0] v;
    input [ACT_SZ-1:0]    esz;
    input [BINPT_SZ-1:0]  bp;
    reg signed [WIDE-1:0] sx;
    begin
        sx = $signed({{(WIDE-DATA_BITS){v[DATA_BITS-1]}}, v}) >>> rsh(esz);
        align_wide = sx << (HALF - bp);
    end
endfunction

// IDEAL golden -- TRUE math R = Z*(A-B) + B + mode*R_prev, round-half-up,
// clamp, left-align.  a_la/b_la/z_la/rprev_la are LEFT-ALIGNED 32-bit element
// values (top of the slice word), exactly as the stream generator presents
// them to hu_compute.  Returns res left-aligned (top of slice), plus over/under.
task golden;
    input  [DATA_BITS-1:0] a_la, b_la, z_la, rprev_la;
    input  [ACT_SZ-1:0]    esz_ab, esz_z, esz_r;
    input  [BINPT_SZ-1:0]  bp_a, bp_b, bp_z, bp_r;
    input                  mode;
    output [DATA_BITS-1:0] res;
    output                 ov, un;
    reg signed [WIDE-1:0]  a_al, b_al, amb, rprev_al;
    reg signed [ACC_BITS-1:0] acc, amb_ext, chunk_ext;
    reg signed [DATA_BITS-1:0] z_int;
    reg [2:0]              mt, mc;
    reg [7:0]              zchunk;
    reg signed [ACC_BITS-1:0] shifted, round_bias;
    reg signed [DATA_BITS-1:0] clamped, maxv, minv, low32;
    integer                out_shift;
    begin
        a_al     = align_wide(a_la, esz_ab, bp_a);
        b_al     = align_wide(b_la, esz_ab, bp_b);
        amb      = a_al - b_al;
        rprev_al = align_wide(rprev_la, esz_r, bp_r);
        amb_ext  = $signed({{(ACC_BITS-WIDE){amb[WIDE-1]}}, amb});

        // right-aligned integer Z (value in LSBs, at bp_z); chunk LSB-first,
        // top chunk signed (two's-complement) -> exact z_int*amb.
        z_int = $signed(z_la) >>> rsh(esz_z);
        mt    = mul_total_of(esz_z);
        acc   = {ACC_BITS{1'b0}};
        for (mc = 0; mc < mt; mc = mc + 1) begin
            zchunk    = z_int >> (mc*8);
            chunk_ext = (mc == mt-1)
                ? $signed({{(ACC_BITS-8){zchunk[7]}}, zchunk})
                : $signed({{(ACC_BITS-8){1'b0}},      zchunk});
            acc = acc + ((chunk_ext * amb_ext) << (mc*8));
        end

        // ST_ACCUM: add B (and R_prev in update mode), each << bp_z
        acc = acc
            + ($signed({{(ACC_BITS-WIDE){b_al[WIDE-1]}}, b_al}) << bp_z)
            + (mode ? ($signed({{(ACC_BITS-WIDE){rprev_al[WIDE-1]}}, rprev_al}) << bp_z)
                    : {ACC_BITS{1'b0}});

        // ST_TRUNCATE: shift to bp_r (= HALF + bp_z - bp_r), round-half-up
        out_shift = HALF + bp_z - bp_r;
        if (out_shift > 0) begin
            round_bias = {{(ACC_BITS-1){1'b0}}, 1'b1} <<< (out_shift - 1);
            shifted    = (acc + round_bias) >>> out_shift;
        end else begin
            shifted    = acc <<< (-out_shift);
        end
        low32     = shifted[DATA_BITS-1:0];

        case (esz_r)
            3'd0: begin maxv = 32'sh00000000; minv = 32'shffffffff; end
            3'd1: begin maxv = 32'sh00000001; minv = 32'shfffffffe; end
            3'd2: begin maxv = 32'sh00000007; minv = 32'shfffffff8; end
            3'd3: begin maxv = 32'sh0000007f; minv = 32'shffffff80; end
            3'd4: begin maxv = 32'sh00007fff; minv = 32'shffff8000; end
            default: begin maxv = 32'sh7fffffff; minv = 32'sh80000000; end
        endcase

        ov = 1'b0; un = 1'b0;
        if (low32 > maxv)      begin clamped = maxv; ov = 1'b1; end
        else if (low32 < minv) begin clamped = minv; un = 1'b1; end
        else                        clamped = low32;

        res = clamped << rsh(esz_r);    // re-left-align (top of slice)
    end
endtask

// =====================================================================
//  Per-element memory packing helpers
//
//  Element k of size esz occupies bits [(k%epw)*w +: w] of word (k/epw),
//  where w=ewidth(esz), epw=32/w.  This matches slice_and_align (idx 0 ->
//  low field) and the packer write-back.  Values are stored as raw w-bit
//  two's-complement fields (NOT left-aligned within the word).
// =====================================================================

// raw w-bit field value (mask + position) for element k
function [DATA_BITS-1:0] field_of;
    input signed [31:0] v;
    input [ACT_SZ-1:0]  esz;
    input integer       k;
    integer w, epw, pos;
    reg [DATA_BITS-1:0] mask, fld;
    begin
        w   = ewidth(esz);
        epw = 32 / w;
        pos = (k % epw) * w;
        mask = (w == 32) ? 32'hFFFF_FFFF : ((32'h1 << w) - 1);
        fld  = v & mask;
        field_of = fld << pos;
    end
endfunction

// write element k value into a source memory array selected by `which`
// (0=A, 1=B, 2=Z, 3=R). Array-by-reference task args are avoided for
// portability; we OR the field directly into the chosen global array.
localparam SEL_A = 0, SEL_B = 1, SEL_Z = 2, SEL_R = 3;
task put_elem;
    input integer         which;
    input signed [31:0]   v;
    input [ACT_SZ-1:0]    esz;
    input integer         k;
    integer w, epw, word;
    reg [DATA_BITS-1:0] fld;
    begin
        w    = ewidth(esz);
        epw  = 32 / w;
        word = k / epw;
        fld  = field_of(v, esz, k);
        case (which)
            SEL_A: mem_a[word] = mem_a[word] | fld;
            SEL_B: mem_b[word] = mem_b[word] | fld;
            SEL_Z: mem_z[word] = mem_z[word] | fld;
            SEL_R: mem_r[word] = mem_r[word] | fld;
        endcase
    end
endtask

// extract the LEFT-ALIGNED 32-bit slice value (as hu_compute sees it) for
// element k from a source word -- mirror of slice_and_align: take the raw
// field and shift it to the TOP of the 32-bit word.
function [DATA_BITS-1:0] slice_of;
    input [DATA_BITS-1:0] word;
    input [ACT_SZ-1:0]    esz;
    input integer         k;
    integer w, epw, pos;
    reg [DATA_BITS-1:0] mask, fld;
    begin
        w    = ewidth(esz);
        epw  = 32 / w;
        pos  = (k % epw) * w;
        mask = (w == 32) ? 32'hFFFF_FFFF : ((32'h1 << w) - 1);
        fld  = (word >> pos) & mask;
        slice_of = fld << rsh(esz);   // left-align to top of slice
    end
endfunction

// read back element k from the output R memory and compare against golden.
task check_elem;
    input [ACT_SZ-1:0]    esz_ab, esz_z, esz_r;
    input [BINPT_SZ-1:0]  bp_a, bp_b, bp_z, bp_r;
    input                 mode;
    input integer         k;
    input [255:0]         tag;
    reg [DATA_BITS-1:0] a_la, b_la, z_la, rprev_la, exp_la, got_la;
    reg                 g_ov, g_un;
    integer wA, wB, wZ, wR, posR;
    reg [DATA_BITS-1:0] maskR;
    begin
        // left-aligned operand slices as the streamers present them
        wA = ewidth(esz_ab); wB = ewidth(esz_ab);
        wZ = ewidth(esz_z);  wR = ewidth(esz_r);
        a_la     = slice_of(mem_a[k / (32/wA)], esz_ab, k);
        b_la     = slice_of(mem_b[k / (32/wB)], esz_ab, k);
        z_la     = slice_of(mem_z[k / (32/wZ)], esz_z,  k);
        // R_prev from the PRE-TASK snapshot (live mem_r is clobbered by write-back)
        rprev_la = mode ? slice_of(mem_rprev[k / (32/wR)], esz_r, k)
                        : {DATA_BITS{1'b0}};

        golden(a_la, b_la, z_la, rprev_la,
               esz_ab, esz_z, esz_r, bp_a, bp_b, bp_z, bp_r, mode,
               exp_la, g_ov, g_un);

        // read back element k's field from R memory, re-left-align to compare
        posR  = (k % (32/wR)) * wR;
        maskR = (wR == 32) ? 32'hFFFF_FFFF : ((32'h1 << wR) - 1);
        got_la = (((mem_r[k / (32/wR)]) >> posR) & maskR) << rsh(esz_r);

        check_eq_u(got_la, exp_la, tag);
    end
endtask

// =====================================================================
//  Bus / control tasks
// =====================================================================
task cfg_write;
    input [31:0] addr;
    input [31:0] data;
    begin
        @(posedge clk); #1;
        hu_sys_req_i  = 1'b1;
        hu_sys_addr_i = addr;
        hu_sys_data_i = data;
        @(negedge clk);
        check_bit(hu_sys_ack_o, 1'b1, "cfg ACK combinational");
        @(posedge clk); #1;
        hu_sys_req_i = 1'b0;
    end
endtask

task start_task;
    begin
        @(posedge clk); #1;
        hu_start_new_block_i = 1'b1;
        @(posedge clk); #1;
        hu_start_new_block_i = 1'b0;
    end
endtask

// Pulse reset to invalidate the per-stream 1-entry caches.
//
// NOTE: dataline_cache_with_xy's cache_hit compares ONLY (shifted_index ==
// reg_addr) && cache_valid_r -- it does NOT include base_addr, and a direct
// testbench write to mem_* does not invalidate the cache. So consecutive
// start_task calls at the same base would get stale cache HITS from the prior
// task. Resetting between independent tasks forces a clean fetch. (Config regs
// are reprogrammed after each reset.)
task reset_dut;
    begin
        @(posedge clk); #1;
        reset = 1'b1;
        repeat (2) @(posedge clk); #1;
        reset = 1'b0;
        repeat (2) @(posedge clk); #1;
    end
endtask

task wait_done;
    begin
        fork
            begin : wdog
                repeat (4000) @(posedge clk);
                $display("FAIL: TIMEOUT waiting for hu_acc_finished_o");
                verif_errors = verif_errors + 1;
                disable done;
            end
            begin : done
                @(posedge hu_acc_finished_o);
                disable wdog;
            end
        join
        repeat (2) @(posedge clk);
    end
endtask

// legacy single-word check (used by the 3 original smoke tests)
task check_r;
    input [7:0]  waddr;
    input [31:0] expected;
    input integer tnum;
    reg [255:0] tag;
    begin
        $sformat(tag, "test %0d mem_r[0x%02X]", tnum, waddr);
        check_eq_u(mem_r[waddr], expected, tag);
    end
endtask

// configure all four element-size / bin-point register sets at once
task cfg_common;
    input [15:0] slen;
    input [ACT_SZ-1:0] esz_a, esz_b, esz_z, esz_r;
    input [BINPT_SZ-1:0] bp_a, bp_b, bp_z, bp_r;
    begin
        cfg_write(32'h1D1D_0004, {16'd0, slen});  // stream_len  (w1)
        cfg_write(32'h1D1D_0008, 32'd0);          // A base      (w2)
        cfg_write(32'h1D1D_000C, {29'd0, esz_a}); // A elem_sz   (w3)
        cfg_write(32'h1D1D_0010, {27'd0, bp_a});  // A bin_pt    (w4)
        cfg_write(32'h1D1D_0014, 32'd0);          // B base      (w5)
        cfg_write(32'h1D1D_0018, {29'd0, esz_b}); // B elem_sz   (w6)
        cfg_write(32'h1D1D_001C, {27'd0, bp_b});  // B bin_pt    (w7)
        cfg_write(32'h1D1D_0020, 32'd0);          // Z base      (w8)
        cfg_write(32'h1D1D_0024, {29'd0, esz_z}); // Z elem_sz   (w9)
        cfg_write(32'h1D1D_0028, {27'd0, bp_z});  // Z bin_pt    (w10)
        cfg_write(32'h1D1D_002C, 32'd0);          // R base      (w11)
        cfg_write(32'h1D1D_0030, {29'd0, esz_r}); // R elem_sz   (w12)
        cfg_write(32'h1D1D_0034, {27'd0, bp_r});  // R bin_pt    (w13)
    end
endtask

// clear all source/result memories
task clear_mem;
    integer j;
    begin
        for (j = 0; j < 256; j = j + 1) begin
            mem_a[j] = 0; mem_b[j] = 0; mem_z[j] = 0;
            mem_r[j] = 0; mem_rprev[j] = 0;
        end
    end
endtask

// snapshot mem_r -> mem_rprev (call immediately before start_task)
task snapshot_rprev;
    integer j;
    begin
        for (j = 0; j < 256; j = j + 1) mem_rprev[j] = mem_r[j];
    end
endtask

// =====================================================================
//  Directed real-multiply driver: per-element-distinct A/B/Z/R_prev so the
//  multiply is genuine (not a broadcast).  Runs the task and checks every
//  output element word against the golden.
// =====================================================================
task run_directed;
    input [255:0]        tag;
    input integer        slen;
    input [ACT_SZ-1:0]   esz;        // common A/B/Z/R element size
    input [BINPT_SZ-1:0] bp;         // common bin point
    input                mode;
    input signed [31:0]  a0, ad;     // element k:  A = a0 + k*ad
    input signed [31:0]  b0, bd;     //             B = b0 + k*bd
    input signed [31:0]  z0, zd;     //             Z = z0 + k*zd
    input signed [31:0]  rp0, rpd;   //             R_prev = rp0 + k*rpd
    integer kk;
    reg [255:0] etag;
    begin
        clear_mem;
        // preload distinct per-element source operands
        for (kk = 0; kk < slen; kk = kk + 1) begin
            put_elem(SEL_A, a0 + kk*ad,  esz, kk);
            put_elem(SEL_B, b0 + kk*bd,  esz, kk);
            put_elem(SEL_Z, z0 + kk*zd,  esz, kk);
            if (mode) put_elem(SEL_R, rp0 + kk*rpd, esz, kk);
        end
        reset_dut;                                 // invalidate stream caches
        cfg_write(32'h1D1D_0000, {31'd0, mode});   // mode
        cfg_common(slen[15:0], esz, esz, esz, esz, bp, bp, bp, bp);
        snapshot_rprev;                            // capture R_prev before clobber
        start_task; wait_done;
        for (kk = 0; kk < slen; kk = kk + 1) begin
            $sformat(etag, "%0s elem %0d", tag, kk);
            check_elem(esz, esz, esz, bp, bp, bp, bp, mode, kk, etag);
        end
    end
endtask

integer k;

// constrained-random support
reg [ACT_SZ-1:0]   r_esz;
reg [BINPT_SZ-1:0] r_bpa, r_bpb, r_bpz, r_bpr;
reg                r_mode;
integer            r_len, ridx, epw;
reg signed [31:0]  rmax, rmin, zmax, zmin;
reg signed [31:0]  vA, vB, vZ, vRP;

initial begin
    clk                  = 0;
    reset                = 1;
    hu_sys_req_i         = 0;
    hu_sys_addr_i        = 0;
    hu_sys_data_i        = 0;
    hu_start_new_block_i = 0;
    hu_target_acc_i      = TGT_ACC_ID[TGT_ACC_SZ-1:0];
    hu_buffer_info_i     = 0;
    src_a_mem_wait_i     = 0;
    src_b_mem_wait_i     = 0;
    src_z_mem_wait_i     = 0;
    src_r_mem_wait_i     = 0;
    src_r_mem_wr_wait_i  = 0;
    src_a_mem_data_i     = 0;
    src_b_mem_data_i     = 0;
    src_z_mem_data_i     = 0;
    src_r_mem_data_i     = 0;
    verif_errors         = 0;
    verif_checks         = 0;

    clear_mem;

    $dumpfile("tb_hadamard_unit.vcd");
    $dumpvars(0, tb_hadamard_unit);

    repeat (2) @(posedge clk);
    #1 reset = 0;
    repeat (2) @(posedge clk);

    $display("=== tb_hadamard_unit ===");

    // ===================================================================
    //  ORIGINAL smoke tests 1-3 (KEPT). 8-bit, bp=0, stream_len=4, base 0.
    //  Broadcast operands (same value in every element). Each task resets the
    //  DUT first to invalidate the per-stream 1-entry caches (see reset_dut).
    // ===================================================================

    // Test 1: mode=0, Z=0, A=8, B=4 -> R = 0*(8-4)+4 = 4 per element
    clear_mem;
    mem_a[0] = 32'h08080808; mem_b[0] = 32'h04040404; mem_z[0] = 32'h00000000;
    reset_dut;
    cfg_common(16'd4, 3'd3, 3'd3, 3'd3, 3'd3, 5'd0, 5'd0, 5'd0, 5'd0);
    cfg_write(32'h1D1D_0000, 32'd0);
    start_task; wait_done;
    check_r(8'h00, 32'h04040404, 1);

    // Test 2: mode=1, Z=0, A=8, B=4, R_prev=2 -> R = 0+4+2 = 6 per element
    clear_mem;
    mem_a[0] = 32'h08080808; mem_b[0] = 32'h04040404; mem_z[0] = 32'h00000000;
    mem_r[0] = 32'h02020202;
    reset_dut;
    cfg_common(16'd4, 3'd3, 3'd3, 3'd3, 3'd3, 5'd0, 5'd0, 5'd0, 5'd0);
    cfg_write(32'h1D1D_0000, 32'd1);
    start_task; wait_done;
    check_r(8'h00, 32'h06060606, 2);

    // Test 3: mode=0, A=B, Z!=0 -> R = Z*0+B = B = 4 per element
    clear_mem;
    mem_a[0] = 32'h04040404; mem_b[0] = 32'h04040404; mem_z[0] = 32'h20202020;
    reset_dut;
    cfg_common(16'd4, 3'd3, 3'd3, 3'd3, 3'd3, 5'd0, 5'd0, 5'd0, 5'd0);
    cfg_write(32'h1D1D_0000, 32'd0);
    start_task; wait_done;
    check_r(8'h00, 32'h04040404, 3);

    // ===================================================================
    //  DIRECTED real-multiply vectors -- ASSERT concrete TRUE values.
    //  Per-element-distinct operands (element k: A=base+k etc.) so each
    //  element is a genuine multiply; every element checked vs golden.
    //
    //  NOTE (F7 FIXED 2026-06-15): hadamard_unit now feeds the packer the
    //  GLOBAL stream-element index (stream_generator.data_global_idx_o) instead
    //  of the cache PER-WORD slice index, so multi-word output streams address
    //  ascending output words correctly (and 32-bit no longer emits an X
    //  address). The single-word D-cases (D4-D11) below are retained unchanged;
    //  multi-word + 32-bit are exercised end-to-end in DW1-DW3 and the random
    //  loop. See verif/FINDINGS.md F7.
    // ===================================================================

    // D4: Z=2,A=10,B=3,mode0,8-bit,bp0 -> R = 2*(10-3)+3 = 17 per stream.
    //     element k uses A=10+k, B=3+k, Z=2 (constant):
    //     2*((10+k)-(3+k)) + (3+k) = 2*7 + 3 + k = 17 + k.
    run_directed("D4 Z*(A-B)+B=17+k", 4, 3'd3, 5'd0, 1'b0,
                 32'sd10, 32'sd1,   // A = 10 + k
                 32'sd3,  32'sd1,   // B =  3 + k
                 32'sd2,  32'sd0,   // Z =  2
                 32'sd0,  32'sd0);  // R_prev unused
    // explicit TRUE-value assert for element 0:  R = 2*(10-3)+3 = 17
    check_eq_u(((mem_r[0]) & 32'h0000_00FF) << rsh(3'd3), la(17, 3'd3),
               "D4 TRUE elem0 = 17");
    // and element 3 (still in word 0, high byte):  17 + 3 = 20
    check_eq_u(((mem_r[0] >> 24) & 32'h0000_00FF) << rsh(3'd3), la(20, 3'd3),
               "D4 TRUE elem3 = 20");

    // D5: positive overflow clamp to 8-bit signed max +127.
    //     Z=100, A=5+k, B=0 -> 100*(5+k) clamps to 127 for all k.
    run_directed("D5 +127 clamp", 4, 3'd3, 5'd0, 1'b0,
                 32'sd5,  32'sd1,
                 32'sd0,  32'sd0,
                 32'sd100,32'sd0,
                 32'sd0,  32'sd0);
    check_eq_u(((mem_r[0]) & 32'h0000_00FF) << rsh(3'd3), la(127, 3'd3),
               "D5 TRUE elem0 = +127 (clamp)");

    // D6: negative underflow clamp to 8-bit signed min -128.
    //     Z=100, A=-(5+k), B=0 -> -100*(5+k) clamps to -128 for all k.
    run_directed("D6 -128 clamp", 4, 3'd3, 5'd0, 1'b0,
                 -32'sd5, -32'sd1,
                 32'sd0,  32'sd0,
                 32'sd100,32'sd0,
                 32'sd0,  32'sd0);
    check_eq_u(((mem_r[0]) & 32'h0000_00FF) << rsh(3'd3), la(-128, 3'd3),
               "D6 TRUE elem0 = -128 (clamp)");

    // D7: mode=1 with nonzero R_prev. Z=2,A=10,B=3,R_prev=5 -> 2*7+3+5 = 22.
    //     element k: A=10+k, B=3+k, Z=2, R_prev=5 -> 17 + k + 5 = 22 + k.
    run_directed("D7 mode1 Rprev=22+k", 4, 3'd3, 5'd0, 1'b1,
                 32'sd10, 32'sd1,
                 32'sd3,  32'sd1,
                 32'sd2,  32'sd0,
                 32'sd5,  32'sd1);
    check_eq_u(((mem_r[0]) & 32'h0000_00FF) << rsh(3'd3), la(22, 3'd3),
               "D7 TRUE elem0 = 22");
    // element 3: A=13,B=6,Z=2,R_prev=8 -> 2*(13-6)+6+8 = 28 (R_prev also += k)
    check_eq_u(((mem_r[0] >> 24) & 32'h0000_00FF) << rsh(3'd3), la(28, 3'd3),
               "D7 TRUE elem3 = 28");

    // D8: nonzero binary point (fractional scaling). 8-bit, bp=2.
    //     Operands stored at bp=2: stored value v represents v/4.
    //     A=8(->2.0), B=4(->1.0), Z=2(->0.5) -> R = 0.5*(2.0-1.0)+1.0 = 1.5
    //     which at bp=2 is stored as 6.  element k: A=8+4k, B=4, Z=2.
    run_directed("D8 binpoint=2", 4, 3'd3, 5'd2, 1'b0,
                 32'sd8,  32'sd4,  // A = 8 + 4k  (2.0, 3.0, 4.0, 5.0)
                 32'sd4,  32'sd0,  // B = 4       (1.0)
                 32'sd2,  32'sd0,  // Z = 2       (0.5)
                 32'sd0,  32'sd0);
    // element 0:  0.5*(2.0-1.0)+1.0 = 1.5 -> stored 6 at bp2
    check_eq_u(((mem_r[0]) & 32'h0000_00FF) << rsh(3'd3), la(6, 3'd3),
               "D8 TRUE elem0 = 6 (1.5 at bp2)");

    // D9: 16-bit element case (multi-cycle multiply path), real multiply.
    //     2 elems / 32-bit word -> stream_len=2 keeps it in one output word.
    //     Z=3,A=100,B=20 -> 3*(100-20)+20 = 260.  element k: A=100+10k.
    //     R = 3*((100+10k)-20)+20 = 3*(80+10k)+20 = 260 + 30k.
    run_directed("D9 16-bit 260+30k", 2, 3'd4, 5'd0, 1'b0,
                 32'sd100, 32'sd10,
                 32'sd20,  32'sd0,
                 32'sd3,   32'sd0,
                 32'sd0,   32'sd0);
    // element 0 (low 16 bits of word 0):  260
    check_eq_u(((mem_r[0]) & 32'h0000_FFFF) << rsh(3'd4), la(260, 3'd4),
               "D9 TRUE elem0 = 260 (16-bit)");
    // element 1 (high 16 bits of word 0):  290
    check_eq_u(((mem_r[0] >> 16) & 32'h0000_FFFF) << rsh(3'd4), la(290, 3'd4),
               "D9 TRUE elem1 = 290 (16-bit)");

    // D10: 16-bit, real multiply, mode=1 (multi-cycle multiply + R_prev).
    //      2 elems / word -> stream_len=2. A=200,B=50,Z=2,R_prev=10 ->
    //      2*(200-50)+50+10 = 360.  element k: A=200+5k, R_prev=10+k.
    run_directed("D10 16-bit mode1 360+11k", 2, 3'd4, 5'd0, 1'b1,
                 32'sd200, 32'sd5,
                 32'sd50,  32'sd0,
                 32'sd2,   32'sd0,
                 32'sd10,  32'sd1);
    // element 0: 2*(200-50)+50+10 = 360
    check_eq_u(((mem_r[0]) & 32'h0000_FFFF) << rsh(3'd4), la(360, 3'd4),
               "D10 TRUE elem0 = 360 (16-bit mode1)");
    // element 1: A=205,R_prev=11 -> 2*(205-50)+50+11 = 371
    check_eq_u(((mem_r[0] >> 16) & 32'h0000_FFFF) << rsh(3'd4), la(371, 3'd4),
               "D10 TRUE elem1 = 371 (16-bit mode1)");

    // D11: 4-bit elements, real multiply (8 elems/word). Z=1,A=k+1,B=0:
    //      R = 1*((k+1)-0)+0 = k+1, clamped to signed-4 range [-8,7].
    run_directed("D11 4-bit k+1", 8, 3'd2, 5'd0, 1'b0,
                 32'sd1, 32'sd1,    // A = 1 + k
                 32'sd0, 32'sd0,
                 32'sd1, 32'sd0,    // Z = 1
                 32'sd0, 32'sd0);
    // element 0 (low nibble): 1 ; element 6: 7 (max in range)
    check_eq_u(((mem_r[0]) & 32'h0000_000F) << rsh(3'd2), la(1, 3'd2),
               "D11 TRUE elem0 = 1 (4-bit)");
    // element 6 (k=6): A=7 -> 1*7 = 7 (signed-4 max)
    check_eq_u(((mem_r[0] >> 24) & 32'h0000_000F) << rsh(3'd2), la(7, 3'd2),
               "D11 TRUE elem6 = 7 (4-bit max)");

    // ===================================================================
    //  MULTI-WORD + 32-bit directed cases (F7 fix proof).  Each spans >1
    //  output word; run_directed checks EVERY element via check_elem, which
    //  reads mem_r[k/(32/wR)] -- the correct output word.  These FAIL on the
    //  old RTL (all words collapse to mem_r[0]; 32-bit address goes X) and pass
    //  only with the global-index packer addressing.  The explicit asserts hit
    //  a NON-ZERO output word to make the ascending-address proof loud.
    // ===================================================================

    // DW1: 16-bit, 6 elems = 3 output words (2 elems/word). Z=3,A=100+10k,B=20
    //      -> R = 3*((100+10k)-20)+20 = 260 + 30k.
    run_directed("DW1 16-bit 6elem 3word", 6, 3'd4, 5'd0, 1'b0,
                 32'sd100, 32'sd10,   // A = 100 + 10k
                 32'sd20,  32'sd0,    // B = 20
                 32'sd3,   32'sd0,    // Z = 3
                 32'sd0,   32'sd0);
    // element 5 lives in WORD 2 (5/2=2), high half (5%2=1): R = 260+30*5 = 410
    check_eq_u(((mem_r[2] >> 16) & 32'h0000_FFFF) << rsh(3'd4), la(410, 3'd4),
               "DW1 TRUE elem5 = 410 (word2 hi)");

    // DW2: 32-bit, 3 elems = 3 output words (1 elem/word). Proves the 'b101
    //      slice_idx fix end-to-end. Z=2,A=1000+100k,B=50 ->
    //      R = 2*((1000+100k)-50)+50 = 1950 + 200k.
    run_directed("DW2 32-bit 3elem 3word", 3, 3'd5, 5'd0, 1'b0,
                 32'sd1000, 32'sd100, // A = 1000 + 100k
                 32'sd50,   32'sd0,   // B = 50
                 32'sd2,    32'sd0,   // Z = 2
                 32'sd0,    32'sd0);
    // element 2 is the WHOLE of WORD 2: R = 1950 + 200*2 = 2350 (rsh=0 for 32b)
    check_eq_u(mem_r[2], la(2350, 3'd5), "DW2 TRUE elem2 = 2350 (32-bit word2)");

    // DW3: 8-bit, 6 elems = 2 output words (4 elems/word). Z=2,A=10+k,B=3+k
    //      -> R = 2*7 + (3+k) ... = 17 + k.
    run_directed("DW3 8-bit 6elem 2word", 6, 3'd3, 5'd0, 1'b0,
                 32'sd10, 32'sd1,    // A = 10 + k
                 32'sd3,  32'sd1,    // B =  3 + k
                 32'sd2,  32'sd0,    // Z =  2
                 32'sd0,  32'sd0);
    // element 5 lives in WORD 1 (5/4=1), byte 1 (5%4=1): R = 17 + 5 = 22
    check_eq_u(((mem_r[1] >> 8) & 32'h0000_00FF) << rsh(3'd3), la(22, 3'd3),
               "DW3 TRUE elem5 = 22 (word1 byte1)");

    // ===================================================================
    //  CONSTRAINED-RANDOM loop. Random elem_sz (1/2/4/8/16/32-bit), random
    //  MULTI-WORD stream_len (1 .. 3 output words for that size), random bin
    //  points (0..4), random mode, per-element A/B/Z/R_prev mostly avoiding
    //  clamping but occasionally hitting it. Every element checked vs golden.
    //  F7 FIXED: multi-word + 32-bit now addressed correctly (global index).
    //  Stream length is bounded to 3 words so the largest case (1-bit -> 96
    //  elems) stays well under the 256-word mem_* arrays and the wait_done
    //  watchdog.
    // ===================================================================
    void'($urandom(32'hDADA_3717));
    for (ridx = 0; ridx < 60; ridx = ridx + 1) begin
        r_esz  = $urandom_range(5);                   // 1/2/4/8/16/32 bit
        epw    = 32 / ewidth(r_esz);                  // elems per 32-bit word
        r_len  = 1 + $urandom_range(3*epw - 1);       // 1 .. 3 output words
        r_bpa  = $urandom_range(4);
        r_bpb  = r_bpa;                                // A and B share esz/bp domain in this DUT path
        r_bpz  = $urandom_range(4);
        r_bpr  = $urandom_range(4);
        r_mode = $urandom & 1;

        // range that mostly avoids clamping; every ~5th vector pushes hard.
        if ((ridx % 5) == 4) begin
            rmax = 32'sd6000;                          // can clamp small sizes
            rmin = -rmax;
        end else begin
            // keep within the signed range of the element size to avoid clamp
            case (r_esz)
                3'd0: rmax = 32'sd0;   3'd1: rmax = 32'sd1;
                3'd2: rmax = 32'sd3;   3'd3: rmax = 32'sd10;
                3'd4: rmax = 32'sd40;  default: rmax = 32'sd200;
            endcase
            rmin = -rmax;
        end

        // Z range bounded to the element's signed representable range (so the
        // stored Z field is not itself truncated), and small for non-clamp.
        case (r_esz)
            3'd0: begin zmax = 32'sd0;   zmin = -32'sd1;  end   // 1-bit: {-1,0}
            3'd1: begin zmax = 32'sd1;   zmin = -32'sd2;  end   // 2-bit
            3'd2: begin zmax = 32'sd2;   zmin = -32'sd2;  end   // 4-bit (small)
            default: begin zmax = 32'sd2; zmin = -32'sd2; end   // small multiplier
        endcase
        if ((ridx % 5) == 4) begin                              // clamp-pushing
            case (r_esz)
                3'd0: begin zmax = 32'sd0;   zmin = -32'sd1;   end
                3'd1: begin zmax = 32'sd1;   zmin = -32'sd2;   end
                3'd2: begin zmax = 32'sd7;   zmin = -32'sd8;   end
                3'd3: begin zmax = 32'sd127; zmin = -32'sd128; end
                default: begin zmax = 32'sd200; zmin = -32'sd200; end
            endcase
        end

        clear_mem;
        for (k = 0; k < r_len; k = k + 1) begin
            vA  = rmin + $urandom_range(rmax - rmin);
            vB  = rmin + $urandom_range(rmax - rmin);
            vZ  = zmin + $urandom_range(zmax - zmin);
            vRP = rmin + $urandom_range(rmax - rmin);
            put_elem(SEL_A, vA, r_esz, k);
            put_elem(SEL_B, vB, r_esz, k);
            put_elem(SEL_Z, vZ, r_esz, k);
            if (r_mode) put_elem(SEL_R, vRP, r_esz, k);
        end

        reset_dut;                                 // invalidate stream caches
        cfg_write(32'h1D1D_0000, {31'd0, r_mode});
        cfg_common(r_len[15:0], r_esz, r_esz, r_esz, r_esz,
                   r_bpa, r_bpb, r_bpz, r_bpr);
        snapshot_rprev;                            // capture R_prev before clobber
        start_task; wait_done;

        for (k = 0; k < r_len; k = k + 1) begin
            reg [255:0] rtag;
            $sformat(rtag, "RND[%0d] esz=%0d mode=%0d elem %0d",
                     ridx, r_esz, r_mode, k);
            check_elem(r_esz, r_esz, r_esz, r_bpa, r_bpb, r_bpz, r_bpr,
                       r_mode, k, rtag);
        end
    end

    `VERIF_EPILOGUE("tb_hadamard_unit")
end

`VERIF_WATCHDOG(5000000)

endmodule
