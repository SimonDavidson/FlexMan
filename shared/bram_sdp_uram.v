// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
// Authors: Simon Davidson & Claude | Created 2026-06-16 | Last modified 2026-09-17
`timescale 10ps/1ps

// Simple dual-port synchronous BRAM — UltraRAM-targeted variant of bram_sdp.
// Behaviourally identical to bram_sdp (1-cycle registered read, independent
// read/write address buses, single clock), but forces UltraRAM inference via
// (* ram_style = "ultra" *).  Intended for deep (>= 4096) 32-bit memories such
// as the Bosch annAcc feature-extraction weight store (16384 x 32) on
// UltraScale+ parts (xczu7ev), where block-RAM would otherwise consume ~16
// RAMB36.  Drop-in for bram_sdp at deep instances; keep bram_sdp for the rest.
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
module bram_sdp_uram #(
    parameter DEPTH  = 16384,
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
    // Cyclone V has no UltraRAM equivalent -- the deep store falls back to M10K.
    // At the Bosch 16384x32 instance that is ~64 M10K (x32 mode = 256 words per
    // block), which is the dominant memory cost on that family. Behaviour is
    // unchanged: still a 1-cycle registered read.
    (* ramstyle = "M10K" *) reg [DATA_W-1:0] mem [0:DEPTH-1];
`elsif NO_URAM
    // NO_URAM (2026-09-17): Xilinx families with no UltraRAM sites -- 7-series,
    // Spartan-7, Artix-7. Without this branch the `else` below applies
    // ram_style="ultra" on a device that has no such primitive, which had never
    // been synthesised and whose outcome (silent fallback / warning / error) was
    // unknown. Force block RAM explicitly instead of relying on the tool.
    //
    // This costs block RAM: a 16384x32 store is 512 Kbit ~= 15 RAMB36 tiles, so
    // on a small part it is the memory footprint that decides whether the design
    // fits, not the logic. Behaviour is unchanged -- still a 1-cycle registered
    // read -- so this is a mapping directive only, and it is DEFAULT-OFF: a build
    // that does not define NO_URAM is bit-identical to every build before it.
    (* ram_style = "block" *) reg [DATA_W-1:0] mem [0:DEPTH-1];
`else
    (* ram_style = "ultra" *) reg [DATA_W-1:0] mem [0:DEPTH-1];
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
