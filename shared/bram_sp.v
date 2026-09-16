// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
// Authors: Simon Davidson & Claude | Last modified: 2026-09-16
`timescale 10ps/1ps

// Single-port synchronous BRAM.  Read-first mode: output reflects the old
// value when the same address is written and read in the same cycle.
// Adding (* ram_style = "block" *) ensures Vivado maps this to RAMB36/RAMB18
// rather than LUTRAM even at small depths.
//
// READ GATING (RD_GATE, default 0 = OFF)
//   RD_GATE=0 : legacy free-running read -- `dout <= mem[raddr]` every cycle,
//               bit-identical to every build before 2026-09-16. `re` is unread.
//   RD_GATE=1 : `if (re) dout <= mem[raddr]`, which infers the BRAM enable pin
//               and stops the array being read when nothing wants the data.
//               NOTE the behavioural change: dout HOLDS while re is low instead
//               of tracking raddr. Consumers that latch on their own accept
//               strobe are fine -- a downstream deployment's sram_model has always gated
//               reads this way and the 299-frame e2e passes against it.
//   Measured motivation: with the design idle (clock running, every *_mem_rd_o
//   low) the Bosch xczu7ev build still burned 24 mW in BRAM + 5 mW in URAM,
//   because these arrays read on every edge forever. Idle dominates the deployed
//   duty cycle, so that power is burned doing nothing.
module bram_sp #(
    parameter DEPTH  = 1024,
    parameter DATA_W = 32,
    // 0 = legacy free-running read (default). 1 = gate the read on `re`.
    parameter RD_GATE = 0
)(
    input  wire                     clk,
    input  wire                     we,
    input  wire                     re,     // read enable; ignored when RD_GATE=0
    input  wire [$clog2(DEPTH)-1:0] addr,
    input  wire [DATA_W-1:0]        din,
    output reg  [DATA_W-1:0]        dout
);
`ifdef ALTERA
    (* ramstyle = "M10K" *) reg [DATA_W-1:0] mem [0:DEPTH-1];
`else
    (* ram_style = "block" *) reg [DATA_W-1:0] mem [0:DEPTH-1];
`endif

    generate
    if (RD_GATE == 0) begin : g_rd_free
        always @(posedge clk) begin
            dout <= mem[addr];          // read-first: capture before write
            if (we) mem[addr] <= din;
        end
    end else begin : g_rd_gated
        always @(posedge clk) begin
            if (re) dout <= mem[addr]; // read-first: capture before write
            if (we) mem[addr] <= din;
        end
    end
    endgenerate
endmodule
