// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
// Authors: Simon Davidson & Claude | Last modified: 2026-08-14
`timescale 10ps/1ps

// Single-port synchronous BRAM.  Read-first mode: output reflects the old
// value when the same address is written and read in the same cycle.
// Adding (* ram_style = "block" *) ensures Vivado maps this to RAMB36/RAMB18
// rather than LUTRAM even at small depths.
module bram_sp #(
    parameter DEPTH  = 1024,
    parameter DATA_W = 32
)(
    input  wire                     clk,
    input  wire                     we,
    input  wire [$clog2(DEPTH)-1:0] addr,
    input  wire [DATA_W-1:0]        din,
    output reg  [DATA_W-1:0]        dout
);
`ifdef ALTERA
    (* ramstyle = "M10K" *) reg [DATA_W-1:0] mem [0:DEPTH-1];
`else
    (* ram_style = "block" *) reg [DATA_W-1:0] mem [0:DEPTH-1];
`endif

    always @(posedge clk) begin
        dout <= mem[addr];          // read-first: capture before write
        if (we) mem[addr] <= din;
    end
endmodule
