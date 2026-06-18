// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
// Authors: Simon Davidson & Claude | Created 2026-06-16 | Last modified 2026-06-16
`timescale 10ps/1ps

// Distributed-RAM simple-dual-port: synchronous write, COMBINATIONAL read.
// Same port list as bram_sdp, but dout is asynchronous (0-cycle) — for memories
// whose reader assumes combinational data under wait=0 (e.g. the scheduler's
// program-memory instruction fetch, which mis-sequences with a 1-cycle BRAM).
// Maps to LUTRAM on Xilinx; intended for SMALL memories (e.g. 64x32 program).
module bram_dist #(
    parameter DEPTH  = 64,
    parameter DATA_W = 32
)(
    input  wire                     clk,
    input  wire                     we,
    input  wire [$clog2(DEPTH)-1:0] waddr,
    input  wire [DATA_W-1:0]        din,
    input  wire [$clog2(DEPTH)-1:0] raddr,
    output wire [DATA_W-1:0]        dout
);
    (* ram_style = "distributed" *) reg [DATA_W-1:0] mem [0:DEPTH-1];

    always @(posedge clk)
        if (we) mem[waddr] <= din;

    assign dout = mem[raddr];   // combinational read
endmodule
