// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
// Authors: Simon Davidson & Claude | Created 2026-06-16 | Last modified 2026-08-14
`timescale 10ps/1ps

// Simple dual-port synchronous BRAM — UltraRAM-targeted variant of bram_sdp.
// Behaviourally identical to bram_sdp (1-cycle registered read, independent
// read/write address buses, single clock), but forces UltraRAM inference via
// (* ram_style = "ultra" *).  Intended for deep (>= 4096) 32-bit memories such
// as the Bosch annAcc feature-extraction weight store (16384 x 32) on
// UltraScale+ parts (xczu7ev), where block-RAM would otherwise consume ~16
// RAMB36.  Drop-in for bram_sdp at deep instances; keep bram_sdp for the rest.
module bram_sdp_uram #(
    parameter DEPTH  = 16384,
    parameter DATA_W = 32
)(
    input  wire                     clk,
    input  wire                     we,
    input  wire [$clog2(DEPTH)-1:0] waddr,
    input  wire [DATA_W-1:0]        din,
    input  wire [$clog2(DEPTH)-1:0] raddr,
    output reg  [DATA_W-1:0]        dout
);
`ifdef ALTERA
    // Cyclone V has no UltraRAM equivalent -- the deep store falls back to M10K.
    // At the Bosch 16384x32 instance that is ~64 M10K (x32 mode = 256 words per
    // block), which is the dominant memory cost on that family. Behaviour is
    // unchanged: still a 1-cycle registered read.
    (* ramstyle = "M10K" *) reg [DATA_W-1:0] mem [0:DEPTH-1];
`else
    (* ram_style = "ultra" *) reg [DATA_W-1:0] mem [0:DEPTH-1];
`endif

    always @(posedge clk) begin
        if (we) mem[waddr] <= din;
        dout <= mem[raddr];
    end
endmodule
