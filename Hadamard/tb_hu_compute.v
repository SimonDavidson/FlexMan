// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
// =============================================================================
// tb_hu_compute  --  Hadamard compute datapath
//
// Authors      : Simon Davidson & Claude
// Created      : 2026-06-07
// Last modified: 2026-06-07
//
// DUT: R = clamp( Z*(A-B) + B + mode*R_prev ), left-aligned for the packer.
//
// The SOFTWARE GOLDEN below is the IDEAL fixed-point reference (the TRUE math),
// updated after the F4/F5 fix. Because the corrected chunked multiply is exact,
// it also equals the fixed RTL bit-for-bit:
//   * left-alignment strip: value = sext(in) >>> rsh(elem_sz), rsh per the RTL
//     table {0:31,1:30,2:28,3:24,4:16,5:0}.
//   * A,B aligned to internal binary point HALF=24: << (24 - bin_point).
//   * amb = a_aligned - b_aligned.
//   * MULTIPLY: chunk the RIGHT-aligned integer z_int LSB-first, top chunk
//     signed (two's complement), lower chunks unsigned; partial product shifted
//     into place: acc += (chunk*amb) << (count*8).  -> acc = z_int*amb at
//     binary point (bp_z + HALF).
//   * ACCUM: acc += (b_aligned << bp_z) + (mode ? (r_prev_aligned << bp_z) : 0)
//     so B/R_prev sit at the SAME binary point as the multiply term.
//   * TRUNCATE: out_shift = HALF + bp_z - bp_r; round-half-up; clamp
//     shifted[31:0] (signed) to the signed element range; result =
//     clamped << rsh(elem_sz_r); set sticky over/under.
//
// Directed cases assert the TRUE value (D4=9, D5=127+over, D6=-128+under,
// D7=260, D8=1993, D9=6), so correctness is verified, not just DUT==golden.
// Checks: pak_data_o, pak_index_o, pak_last_o, sticky over_r_o/under_r_o,
// across directed corners + a constrained-random loop.
// =============================================================================
`timescale 1ns/1ps
`include "../shared/constants.v"

module tb_hu_compute;

    localparam integer DATA_BITS = 32;
    localparam integer ACT_SZ    = `POT_OUT_SZ_SZ;   // 3
    localparam integer BINPT_SZ  = 5;
    localparam integer PIN_BITS  = `PIN_BITS;        // 10
    localparam integer WIDE      = DATA_BITS + 16;   // 48
    localparam integer HALF      = WIDE/2;           // 24
    localparam integer ACC_BITS  = WIDE + DATA_BITS + 4; // 84
    localparam integer NVEC      = 4000;

    reg clk, reset;
    reg                  valid_i;
    wire                 ready_o;
    reg  [DATA_BITS-1:0] a_i, b_i, z_i, r_prev_i;
    reg  [PIN_BITS-1:0]  index_i;
    reg                  last_i;
    reg  [BINPT_SZ-1:0]  bp_a_i, bp_b_i, bp_z_i, bp_r_i;
    reg  [ACT_SZ-1:0]    esz_ab_i, esz_z_i, esz_r_i;
    reg                  mode_i;

    wire                 pak_write_o;
    reg                  pak_full_i;
    wire [DATA_BITS-1:0] pak_data_o;
    wire [PIN_BITS-1:0]  pak_index_o;
    wire                 pak_last_o;
    wire                 over_r_o, under_r_o;

    integer verif_errors, verif_checks, verif_to;
    `include "../verif/checks.vh"
    `include "../verif/vt_driver.vh"

    hu_compute #(
        .DATA_BITS(DATA_BITS), .ACT_SZ(ACT_SZ), .BINPT_SZ(BINPT_SZ), .PIN_BITS(PIN_BITS)
    ) dut (
        .clk(clk), .reset(reset),
        .valid_i(valid_i), .ready_o(ready_o),
        .a_i(a_i), .b_i(b_i), .z_i(z_i), .r_prev_i(r_prev_i),
        .index_i(index_i), .last_i(last_i),
        .bin_point_a_i(bp_a_i), .bin_point_b_i(bp_b_i),
        .bin_point_z_i(bp_z_i), .bin_point_r_i(bp_r_i),
        .elem_sz_ab_i(esz_ab_i), .elem_sz_z_i(esz_z_i), .elem_sz_r_i(esz_r_i),
        .mode_i(mode_i),
        .pak_write_o(pak_write_o), .pak_full_i(pak_full_i),
        .pak_data_o(pak_data_o), .pak_index_o(pak_index_o), .pak_last_o(pak_last_o),
        .over_r_o(over_r_o), .under_r_o(under_r_o));

    initial clk = 0; always #5 clk = ~clk;

    // ---------------------------------------------------------------------
    // Software golden helpers
    // ---------------------------------------------------------------------
    function [5:0] rsh;                 // right_shift_for_sz
        input [ACT_SZ-1:0] sz;
        begin
            case (sz)
                3'd0: rsh = 6'd31; 3'd1: rsh = 6'd30; 3'd2: rsh = 6'd28;
                3'd3: rsh = 6'd24; 3'd4: rsh = 6'd16; 3'd5: rsh = 6'd0;
                default: rsh = 6'd0;
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

    // sign-extend a DATA_BITS value to WIDE, arith-shift right by rsh, then
    // left-shift to put the binary point at bit HALF.
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

    // IDEAL golden: computes the TRUE math R = Z*(A-B) + B + mode*R_prev with
    // round-half-up output, clamp, and left-align. Because the chunked multiply
    // is exact, this also equals the FIXED RTL bit-for-bit. (Was a
    // characterization model that mirrored the broken behaviour.)
    task golden;
        output [DATA_BITS-1:0] res;
        output                 ov, un;
        reg signed [WIDE-1:0]  a_al, b_al, amb, rprev_al;
        reg signed [ACC_BITS-1:0] acc, amb_ext, chunk_ext, round_bias;
        reg signed [DATA_BITS-1:0] z_int;
        reg [2:0]              mt, mc;
        reg [7:0]              zchunk;
        reg signed [ACC_BITS-1:0] shifted;
        reg signed [DATA_BITS-1:0] clamped, maxv, minv, low32;
        integer                out_shift;
        begin
            a_al     = align_wide(a_i, esz_ab_i, bp_a_i);
            b_al     = align_wide(b_i, esz_ab_i, bp_b_i);
            amb      = a_al - b_al;
            rprev_al = align_wide(r_prev_i, esz_r_i, bp_r_i);
            amb_ext  = $signed({{(ACC_BITS-WIDE){amb[WIDE-1]}}, amb});

            // right-aligned integer Z (value in LSBs, at bp_z); chunk LSB-first,
            // top chunk signed (two's-complement) -> exact z_int*amb.
            z_int = $signed(z_i) >>> rsh(esz_z_i);
            mt    = mul_total_of(esz_z_i);
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
                + ($signed({{(ACC_BITS-WIDE){b_al[WIDE-1]}}, b_al}) << bp_z_i)
                + (mode_i ? ($signed({{(ACC_BITS-WIDE){rprev_al[WIDE-1]}}, rprev_al}) << bp_z_i)
                          : {ACC_BITS{1'b0}});

            // ST_TRUNCATE: shift to bp_r (= HALF + bp_z - bp_r), round-half-up
            out_shift = HALF + bp_z_i - bp_r_i;
            if (out_shift > 0) begin
                round_bias = {{(ACC_BITS-1){1'b0}}, 1'b1} <<< (out_shift - 1);
                shifted    = (acc + round_bias) >>> out_shift;
            end else begin
                shifted    = acc <<< (-out_shift);
            end
            low32     = shifted[DATA_BITS-1:0];

            case (esz_r_i)
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

            res = clamped << rsh(esz_r_i);    // re-left-align
        end
    endtask

    // ---------------------------------------------------------------------
    // Drive one element through the DUT, polling ready/pak (depth-agnostic).
    // ---------------------------------------------------------------------
    reg [DATA_BITS-1:0] d_data, g_data;
    reg [PIN_BITS-1:0]  d_index;
    reg                 d_last, g_ov, g_un;

    task drive;
        begin
            // wait until DUT can accept
            while (!ready_o) @(posedge clk);
            valid_i = 1'b1; @(posedge clk); #1; valid_i = 1'b0;
            // wait for the packer write strobe
            verif_to = 200;
            while (!pak_write_o && verif_to > 0) begin @(posedge clk); #1; verif_to=verif_to-1; end
            if (verif_to == 0) begin
                $display("FAIL: timeout waiting for pak_write_o"); verif_errors=verif_errors+1;
            end
            d_data  = pak_data_o;
            d_index = pak_index_o;
            d_last  = pak_last_o;
            @(posedge clk); #1;   // let FSM return to IDLE
        end
    endtask

    // Reset accumulated sticky golden flags by toggling reset on the DUT.
    task do_reset;
        begin
            reset = 1'b1; valid_i=1'b0; @(posedge clk); @(posedge clk); #1;
            reset = 1'b0; @(posedge clk); #1;
        end
    endtask

    integer k;
    reg exp_ov_sticky, exp_un_sticky;

    task drive_and_check;
        input [255:0] tag;
        begin
            golden(g_data, g_ov, g_un);
            if (g_ov) exp_ov_sticky = 1'b1;
            if (g_un) exp_un_sticky = 1'b1;
            drive;
            check_eq_u(d_data, g_data, {tag, " data"});
            check_eq_u(d_index, index_i, {tag, " index"});
            check_bit(d_last, last_i, {tag, " last"});
            check_bit(over_r_o,  exp_ov_sticky, {tag, " over_sticky"});
            check_bit(under_r_o, exp_un_sticky, {tag, " under_sticky"});
        end
    endtask

    // Convenience: pack a small signed value into a left-aligned element field.
    function [DATA_BITS-1:0] la;        // left-align value for elem size
        input signed [31:0] v;
        input [ACT_SZ-1:0]  esz;
        begin
            la = v << rsh(esz);
        end
    endfunction

    initial begin
        verif_errors=0; verif_checks=0;
        valid_i=0; pak_full_i=0; last_i=0; index_i=0; mode_i=0;
        a_i=0; b_i=0; z_i=0; r_prev_i=0;
        bp_a_i=0; bp_b_i=0; bp_z_i=0; bp_r_i=0;
        esz_ab_i=3'd3; esz_z_i=3'd3; esz_r_i=3'd3;
        exp_ov_sticky=0; exp_un_sticky=0;
        do_reset;
        $display("=== tb_hu_compute ===");

        // ---- directed (8-bit elems, bp=0). Each asserts the TRUE math value
        //      (g_data), not just DUT==golden. ----
        // D1: mode0, Z=0, A=8, B=4 -> R = B = 4
        a_i=la(8,3); b_i=la(4,3); z_i=la(0,3); r_prev_i=0; index_i=10'd1; last_i=0; mode_i=0;
        drive_and_check("D1 Z=0 -> B");
        check_eq_u(g_data, la(4,3),   "D1 true=4");
        // D2: mode1, Z=0, A=8, B=4, Rprev=2 -> R = B + Rprev = 6
        a_i=la(8,3); b_i=la(4,3); z_i=la(0,3); r_prev_i=la(2,3); index_i=10'd2; last_i=1; mode_i=1;
        drive_and_check("D2 Z=0 mode1 -> B+Rprev");
        check_eq_u(g_data, la(6,3),   "D2 true=6");
        // D3: mode0, A=B, Z!=0 -> R = B (since A-B=0)
        a_i=la(4,3); b_i=la(4,3); z_i=la(5,3); r_prev_i=0; index_i=10'd3; last_i=0; mode_i=0;
        drive_and_check("D3 A=B -> B");
        check_eq_u(g_data, la(4,3),   "D3 true=4");
        // D4: real multiply: Z=2, A=5, B=1 -> 2*(5-1)+1 = 9
        a_i=la(5,3); b_i=la(1,3); z_i=la(2,3); r_prev_i=0; index_i=10'd4; last_i=0; mode_i=0;
        drive_and_check("D4 Z*(A-B)+B");
        check_eq_u(g_data, la(9,3),   "D4 true=9");
        // D5: positive overflow clamp (8-bit max 127): Z=100, A=5, B=0 -> 500 -> clamp 127
        a_i=la(5,3); b_i=la(0,3); z_i=la(100,3); r_prev_i=0; index_i=10'd5; last_i=0; mode_i=0;
        drive_and_check("D5 over clamp");
        check_eq_u(g_data, la(127,3), "D5 true=127");
        check_bit(g_ov, 1'b1,         "D5 over flagged");
        // D6: negative underflow clamp (8-bit min -128): Z=100, A=-5, B=0 -> -500 -> clamp -128
        a_i=la(-5,3); b_i=la(0,3); z_i=la(100,3); r_prev_i=0; index_i=10'd6; last_i=1; mode_i=0;
        drive_and_check("D6 under clamp");
        check_eq_u(g_data, la(-128,3),"D6 true=-128");
        check_bit(g_un, 1'b1,         "D6 under flagged");

        // ---- 16-bit and 32-bit element sizes (multi-cycle multiply path) ----
        esz_ab_i=3'd4; esz_z_i=3'd4; esz_r_i=3'd4;
        a_i=la(100,4); b_i=la(20,4); z_i=la(3,4); r_prev_i=0; index_i=10'd7; last_i=0; mode_i=0;
        drive_and_check("D7 16-bit");
        check_eq_u(g_data, la(260,4),  "D7 true=260 (3*(100-20)+20)");
        esz_ab_i=3'd5; esz_z_i=3'd5; esz_r_i=3'd5;
        a_i=la(1000,5); b_i=la(7,5); z_i=la(2,5); r_prev_i=0; index_i=10'd8; last_i=0; mode_i=0;
        drive_and_check("D8 32-bit");
        check_eq_u(g_data, la(1993,5), "D8 true=1993 (2*(1000-7)+7)");

        // ---- nonzero binary points (fractional scaling) ----
        esz_ab_i=3'd3; esz_z_i=3'd3; esz_r_i=3'd3;
        bp_a_i=2; bp_b_i=2; bp_z_i=2; bp_r_i=2;
        a_i=la(8,3); b_i=la(4,3); z_i=la(2,3); r_prev_i=0; index_i=10'd9; last_i=0; mode_i=0;
        drive_and_check("D9 binpoint=2");
        check_eq_u(g_data, la(6,3),    "D9 true=6 (0.5*(2-1)+1=1.5 at bp2)");
        bp_a_i=0; bp_b_i=0; bp_z_i=0; bp_r_i=0;

        // ---- constrained-random loop ----
        void'($urandom(32'hC0DE_AB01));
        for (k = 0; k < NVEC; k = k + 1) begin
            esz_ab_i = $urandom_range(5);
            esz_z_i  = $urandom_range(5);
            esz_r_i  = $urandom_range(5);
            bp_a_i   = $urandom_range(8);
            bp_b_i   = $urandom_range(8);
            bp_z_i   = $urandom_range(8);
            bp_r_i   = $urandom_range(8);
            a_i      = $urandom;
            b_i      = $urandom;
            z_i      = $urandom;
            r_prev_i = $urandom;
            index_i  = $urandom_range((1<<PIN_BITS)-1);
            last_i   = $urandom & 1'b1;
            mode_i   = $urandom & 1'b1;
            drive_and_check("rand");
        end

        `VERIF_EPILOGUE("tb_hu_compute")
    end

    `VERIF_WATCHDOG(5000000)

endmodule
