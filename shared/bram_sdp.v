// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
`timescale 10ps/1ps

// Simple dual-port synchronous BRAM.  Fully independent read and write
// address buses, single clock.  Infers RAMB36/RAMB18 in SDP mode on
// Xilinx 7-series; dout updates every cycle from raddr.
module bram_sdp #(
    parameter DEPTH  = 1024,
    parameter DATA_W = 32
)(
    input  wire                     clk,
    input  wire                     we,
    input  wire [$clog2(DEPTH)-1:0] waddr,
    input  wire [DATA_W-1:0]        din,
    input  wire [$clog2(DEPTH)-1:0] raddr,
    output reg  [DATA_W-1:0]        dout
);
    (* ram_style = "block" *) reg [DATA_W-1:0] mem [0:DEPTH-1];

    always @(posedge clk) begin
        if (we) mem[waddr] <= din;
        dout <= mem[raddr];
    end
endmodule
