// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
// =============================================================================
// verif/sram_bfm.vh  --  reusable synchronous SRAM models for FlexMan TBs
//
// Authors      : Simon Davidson & Claude
// Created      : 2026-06-07
// Last modified: 2026-06-07
//
// Drop-in macros that reproduce the FlexMan testbench SRAM convention exactly:
//   * 256 x 32-bit array, address truncated to [7:0] (deliberate aliasing —
//     base addresses 0/64/128/192 are chosen to stay inside one 256-word window)
//   * 1-cycle synchronous read latency, immediate (next-edge) write
//
// Provided because each memory interface in the DUTs uses a different port-name
// prefix and a different rd/wr/wait combination; a macro keeps the existing
// flat-array pattern while letting random tests inject mem_wait back-pressure.
//
// Requires `clk` in scope.  Declare the read-data reg and (for *_WAIT) the
// wait reg in the TB; the macro declares the array.
//
// Usage:
//   `SRAM_RW (syn_sram, syn_rd, syn_wr, syn_addr, syn_wdata, syn_rdata)
//   `SRAM_RD (bias_sram, bias_rd, bias_addr, bias_rdata)
//   `SRAM_RD_WAIT(act_sram, act_rd, act_wait, act_addr, act_rdata)
//   `SRAM_WAIT_RANDOM(act_wait, 25)   // 25% chance of mem_wait each cycle
// =============================================================================

// Read/write array sharing one port (read-modify-write style: syn_curr, pot).
// Read and write never collide in the DUTs (cache blocks read during writeback).
`define SRAM_RW(ARR, RD, WR, ADDR, WDATA, RDATA)                              \
    reg [31:0] ARR [0:255];                                                   \
    always @(posedge clk) begin                                              \
        if (RD) RDATA <= ARR[ADDR[7:0]];                                     \
        if (WR) ARR[ADDR[7:0]] <= WDATA;                                     \
    end

// Read-only array, unconditional read on RD strobe (bias, thresh).
`define SRAM_RD(ARR, RD, ADDR, RDATA)                                         \
    reg [31:0] ARR [0:255];                                                   \
    always @(posedge clk)                                                     \
        if (RD) RDATA <= ARR[ADDR[7:0]];

// Read-only array, read gated by ~WAIT (act, weight — request honoured only
// when not back-pressured).
`define SRAM_RD_WAIT(ARR, RD, WAIT, ADDR, RDATA)                             \
    reg [31:0] ARR [0:255];                                                   \
    always @(posedge clk)                                                     \
        if (RD & ~WAIT) RDATA <= ARR[ADDR[7:0]];

// Write-only array (spike output).
`define SRAM_WR(ARR, WR, ADDR, WDATA)                                         \
    reg [31:0] ARR [0:255];                                                   \
    always @(posedge clk)                                                     \
        if (WR) ARR[ADDR[7:0]] <= WDATA;

// Random back-pressure generator: drive a mem_wait reg high with PROB_PCT %
// probability each cycle.  WAITREG must be a reg declared in the TB and driven
// by nothing else.  (SystemVerilog $urandom_range; requires -sv.)
`define SRAM_WAIT_RANDOM(WAITREG, PROB_PCT)                                   \
    always @(posedge clk) WAITREG <= ($urandom_range(99) < (PROB_PCT));

// Zero-initialise a 256-word array (call inside an initial block).
`define SRAM_CLEAR(ARR)                                                       \
    for (sram_i = 0; sram_i < 256; sram_i = sram_i + 1) ARR[sram_i] = 32'd0;
