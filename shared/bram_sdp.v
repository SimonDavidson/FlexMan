// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
// Authors: Simon Davidson & Claude | Last modified: 2026-09-16
`timescale 10ps/1ps

// Simple dual-port synchronous BRAM.  Fully independent read and write
// address buses, single clock.  Infers RAMB36/RAMB18 in SDP mode on
// Xilinx 7-series; dout updates every cycle from raddr.
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
module bram_sdp #(
    parameter DEPTH  = 1024,
    parameter DATA_W = 32,
    // 0 = legacy free-running read (default). 1 = gate the read on `re`.
    parameter RD_GATE = 0
)(
    input  wire                     clk,
    input  wire                     we,
    input  wire [$clog2(DEPTH)-1:0] waddr,
    input  wire [DATA_W-1:0]        din,
    input  wire                     re,     // read enable; ignored when RD_GATE=0
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

    generate
    if (RD_GATE == 0) begin : g_rd_free
        always @(posedge clk) begin
            if (we) mem[waddr] <= din;
            dout <= mem[raddr];
        end
    end else begin : g_rd_gated
        always @(posedge clk) begin
            if (we) mem[waddr] <= din;
            if (re) dout <= mem[raddr];
        end
    end
    endgenerate
endmodule
