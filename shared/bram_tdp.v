// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
// Authors: Simon Davidson & Claude | Last modified: 2026-08-14
`timescale 10ps/1ps

// True dual-port synchronous BRAM.  Ports A and B are fully independent
// (separate address, data, enable, and write-enable).  Single clock.
// Read-first collision mode on each port.  Infers RAMB36E1 (TDP) on
// Xilinx 7-series.
module bram_tdp #(
    parameter DEPTH  = 64,
    parameter DATA_W = 32
)(
    input  wire                     clk,
    // Port A
    input  wire                     ena,
    input  wire                     wea,
    input  wire [$clog2(DEPTH)-1:0] addra,
    input  wire [DATA_W-1:0]        dina,
    output reg  [DATA_W-1:0]        douta,
    // Port B
    input  wire                     enb,
    input  wire                     web,
    input  wire [$clog2(DEPTH)-1:0] addrb,
    input  wire [DATA_W-1:0]        dinb,
    output reg  [DATA_W-1:0]        doutb
);
`ifdef ALTERA
    (* ramstyle = "M10K" *) reg [DATA_W-1:0] mem [0:DEPTH-1];
`else
    (* ram_style = "block" *) reg [DATA_W-1:0] mem [0:DEPTH-1];
`endif

    always @(posedge clk) begin
        if (ena) begin
            douta <= mem[addra];
            if (wea) mem[addra] <= dina;
        end
    end

    always @(posedge clk) begin
        if (enb) begin
            doutb <= mem[addrb];
            if (web) mem[addrb] <= dinb;
        end
    end
endmodule
