// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
// Authors: Simon Davidson & Claude | Last modified: 2026-08-14
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
`ifdef ALTERA
    // Quartus reads `ramstyle`; Vivado reads `ram_style`. Each tool ignores the
    // other's attribute, so both could be stated unconditionally -- but this is
    // guarded so the Xilinx flow (which produces the Bosch area/power figures)
    // is provably byte-for-byte unchanged. ALTERA is set only in the .qsf.
    (* ramstyle = "M10K" *) reg [DATA_W-1:0] mem [0:DEPTH-1];
`else
    (* ram_style = "block" *) reg [DATA_W-1:0] mem [0:DEPTH-1];
`endif

    always @(posedge clk) begin
        if (we) mem[waddr] <= din;
        dout <= mem[raddr];
    end
endmodule
