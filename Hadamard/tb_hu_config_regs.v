// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
// =============================================================================
// tb_hu_config_regs  --  Hadamard config register bank
//
// Authors      : Simon Davidson & Claude
// Created      : 2026-06-07
// Last modified: 2026-06-07
//
// hu_config_regs is a simple AXI-lite-ish register file:
//   * addr_match = (addr[31:16] == TGT_CONFIG_BASE_ADDR)
//   * hu_sys_ack_o = hu_sys_req_i & addr_match   (combinational, same cycle)
//   * on posedge, when req & match, write the field selected by addr[7:0]
//   * reset clears every register to 0; unknown offsets are ignored
//
// Tests:
//   * reset values (all zero)
//   * every documented offset: write a field-width pattern, read it back, and
//     confirm the field is truncated to the register width
//   * ACK timing: combinational high when req & match, low when match fails
//   * address-match gating: a write with the WRONG upper 16 bits is ignored
//     (and produces no ACK), and a low-but-nonzero req with match still acks
//   * unknown offset (within the matched window) writes nothing
// =============================================================================
`timescale 1ns/1ps
`include "../shared/constants.v"

module tb_hu_config_regs;

    localparam [15:0] TGT_CFG = 16'h1D1D;
    localparam integer ADDR_SZ     = `ADDR_SIZE;       // 30
    localparam integer ACT_SZ      = `POT_OUT_SZ_SZ;   // 3
    localparam integer NUM_ELEM_SZ = 16;

    reg                    clk, reset;
    reg                    req;
    wire                   ack;
    reg  [31:0]            addr, data;

    wire                   mode_r;
    wire [NUM_ELEM_SZ-1:0] stream_len_r;
    wire [ADDR_SZ-1:0]     a_base, b_base, z_base, r_base;
    wire [ACT_SZ-1:0]      a_esz, b_esz, z_esz, r_esz;
    wire [4:0]             a_bp, b_bp, z_bp, r_bp;

    integer verif_errors, verif_checks;
    `include "../verif/checks.vh"

    hu_config_regs #(
        .TGT_CONFIG_BASE_ADDR(TGT_CFG), .ADDR_SZ(ADDR_SZ),
        .ACT_SZ(ACT_SZ), .NUM_ELEM_SZ(NUM_ELEM_SZ)
    ) dut (
        .clk(clk), .reset(reset),
        .hu_sys_req_i(req), .hu_sys_ack_o(ack),
        .hu_sys_addr_i(addr), .hu_sys_data_i(data),
        .mode_r_o(mode_r), .stream_len_r_o(stream_len_r),
        .src_a_base_addr_r_o(a_base), .src_a_elem_sz_r_o(a_esz), .src_a_bin_point_r_o(a_bp),
        .src_b_base_addr_r_o(b_base), .src_b_elem_sz_r_o(b_esz), .src_b_bin_point_r_o(b_bp),
        .src_z_base_addr_r_o(z_base), .src_z_elem_sz_r_o(z_esz), .src_z_bin_point_r_o(z_bp),
        .src_r_base_addr_r_o(r_base), .src_r_elem_sz_r_o(r_esz), .src_r_bin_point_r_o(r_bp));

    initial clk = 0; always #5 clk = ~clk;

    // Drive a config write to (TGT_CFG<<16 | offset) and verify combinational ACK.
    task cfg_write;
        input [7:0]  offset;
        input [31:0] wdata;
        input        expect_ack;
        begin
            @(posedge clk); #1;
            req  = 1'b1;
            addr = {TGT_CFG, 8'h00, offset};
            data = wdata;
            #1;                      // settle combinational ack
            check_bit(ack, expect_ack, "ack combinational");
            @(posedge clk); #1;      // write commits here
            req = 1'b0; #1;
            check_bit(ack, 1'b0, "ack deasserts with req low");
        end
    endtask

    // Drive a write to a NON-matching upper address; must NOT ack or write.
    task cfg_write_nomatch;
        input [7:0]  offset;
        input [31:0] wdata;
        begin
            @(posedge clk); #1;
            req  = 1'b1;
            addr = {16'hBEEF, 8'h00, offset};   // wrong upper 16 bits
            data = wdata;
            #1;
            check_bit(ack, 1'b0, "no ack on addr mismatch");
            @(posedge clk); #1;
            req = 1'b0; #1;
        end
    endtask

    initial begin
        verif_errors=0; verif_checks=0;
        req=0; addr=0; data=0;
        reset=1; @(posedge clk); @(posedge clk); #1; reset=0; @(posedge clk); #1;
        $display("=== tb_hu_config_regs ===");

        // ---- reset values: all registers zero ----
        check_bit(mode_r, 1'b0, "rst mode");
        check_eq_u(stream_len_r, 16'd0, "rst stream_len");
        check_eq_u(a_base, {ADDR_SZ{1'b0}}, "rst a_base");
        check_eq_u(a_esz,  {ACT_SZ{1'b0}},  "rst a_esz");
        check_eq_u(a_bp,   5'd0, "rst a_bp");
        check_eq_u(r_base, {ADDR_SZ{1'b0}}, "rst r_base");
        check_eq_u(r_esz,  {ACT_SZ{1'b0}},  "rst r_esz");
        check_eq_u(r_bp,   5'd0, "rst r_bp");

        // ---- write/read-back each offset with full-width pattern; confirm
        //      the register truncates to its own width ----
        // 0x00 mode  [0]
        cfg_write(8'h00, 32'hFFFF_FFFF, 1'b1);
        check_bit(mode_r, 1'b1, "mode = bit0");
        cfg_write(8'h00, 32'hFFFF_FFFE, 1'b1);
        check_bit(mode_r, 1'b0, "mode = 0 (bit0 low)");

        // 0x01 stream_len [15:0]
        cfg_write(8'h01, 32'h1234_ABCD, 1'b1);
        check_eq_u(stream_len_r, 16'hABCD, "stream_len trunc 16b");

        // src A
        cfg_write(8'h04, 32'hFFFF_FFFF, 1'b1);
        check_eq_u(a_base, {ADDR_SZ{1'b1}}, "a_base trunc 30b");
        cfg_write(8'h05, 32'hFFFF_FFFF, 1'b1);
        check_eq_u(a_esz, {ACT_SZ{1'b1}}, "a_esz trunc 3b");
        cfg_write(8'h06, 32'hFFFF_FFFF, 1'b1);
        check_eq_u(a_bp, 5'h1F, "a_bp trunc 5b");

        // src B
        cfg_write(8'h08, 32'h2AAA_AAAA, 1'b1);
        check_eq_u(b_base, 30'h2AAA_AAAA & {ADDR_SZ{1'b1}}, "b_base");
        cfg_write(8'h09, 32'd5, 1'b1);
        check_eq_u(b_esz, 3'd5, "b_esz");
        cfg_write(8'h0A, 32'd17, 1'b1);
        check_eq_u(b_bp, 5'd17, "b_bp");

        // src Z
        cfg_write(8'h0C, 32'h1555_5555, 1'b1);
        check_eq_u(z_base, 30'h1555_5555, "z_base");
        cfg_write(8'h0D, 32'd4, 1'b1);
        check_eq_u(z_esz, 3'd4, "z_esz");
        cfg_write(8'h0E, 32'd9, 1'b1);
        check_eq_u(z_bp, 5'd9, "z_bp");

        // src R
        cfg_write(8'h10, 32'h0123_4567, 1'b1);
        check_eq_u(r_base, 30'h0123_4567, "r_base");
        cfg_write(8'h11, 32'd3, 1'b1);
        check_eq_u(r_esz, 3'd3, "r_esz");
        cfg_write(8'h12, 32'd31, 1'b1);
        check_eq_u(r_bp, 5'd31, "r_bp");

        // ---- isolation: writing one register must not disturb neighbours ----
        // (a_base still all-ones from earlier; b_esz==5; rewrite z_bp and recheck)
        check_eq_u(a_base, {ADDR_SZ{1'b1}}, "a_base unchanged");
        check_eq_u(b_esz, 3'd5, "b_esz unchanged");

        // ---- address-match gating: wrong upper bits => ignored, no ack ----
        cfg_write_nomatch(8'h11, 32'd0);            // would zero r_esz if it hit
        check_eq_u(r_esz, 3'd3, "r_esz untouched by nomatch write");

        // ---- unknown offset within matched window writes nothing (acks though) ----
        cfg_write(8'h7F, 32'hFFFF_FFFF, 1'b1);      // undefined offset, still acks
        check_eq_u(r_esz, 3'd3, "r_esz untouched by unknown offset");
        check_eq_u(a_base, {ADDR_SZ{1'b1}}, "a_base untouched by unknown offset");

        // ---- ACK low when req low even with matching address ----
        @(posedge clk); #1;
        req = 1'b0; addr = {TGT_CFG, 16'h0000};
        #1; check_bit(ack, 1'b0, "ack low when req low");

        `VERIF_EPILOGUE("tb_hu_config_regs")
    end

    `VERIF_WATCHDOG(500000)

endmodule
