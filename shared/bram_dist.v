// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
// Authors: Simon Davidson & Claude | Created 2026-06-16 | Last modified 2026-08-14
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
`ifdef ALTERA
    // NOT "MLAB". Cyclone V MLABs cannot provide the asynchronous read this
    // module exists to give -- Quartus would either register the output or fall
    // back silently, and a 1-cycle program memory makes the scheduler double-fire
    // NXT (see FPGA_BUILD.md; reproduced by tb_sync_progonly.v). "logic" forces
    // an explicit flop array + read mux, which is correct by construction.
    // At the 64x32 program instance that is 2048 registers -- affordable, and
    // the same structure regfile_2r1w already uses for the bba memory.
    (* ramstyle = "logic" *) reg [DATA_W-1:0] mem [0:DEPTH-1];
`else
    (* ram_style = "distributed" *) reg [DATA_W-1:0] mem [0:DEPTH-1];
`endif

    always @(posedge clk)
        if (we) mem[waddr] <= din;

    assign dout = mem[raddr];   // combinational read
endmodule
