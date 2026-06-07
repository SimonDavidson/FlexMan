// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
// =============================================================================
// tb_bram_sdp  --  simple dual-port synchronous read-first BRAM
//
// Authors      : Simon Davidson & Claude
// Created      : 2026-06-07
// Last modified: 2026-06-07
//
// Independent read (raddr) and write (waddr) buses, single clock.  Each cycle:
//   if (we) mem[waddr] <= din;   dout <= mem[raddr];
// Both are NBAs, so on a same-address collision (raddr==waddr & we) dout returns
// the OLD value (read-first).  The SW golden mirrors this exactly: it samples
// model[raddr] BEFORE committing the write into model[waddr], so a collision
// returns OLD.  After a #1 settle the registered dout reflects THIS cycle's
// raddr, so the check is in-phase (no pipeline skew).  Covers:
//   * independent read/write to different addresses
//   * read-after-write to the same address (next-cycle sees new data)
//   * same-cycle raddr==waddr collision (dout == OLD value)
//   * a long constrained-random stream
// =============================================================================
`timescale 10ps/1ps

module tb_bram_sdp;

    localparam integer DEPTH  = 64;
    localparam integer DATA_W = 32;
    localparam integer AW     = 6;
    localparam integer NVEC   = 4000;

    reg                 clk;
    reg                 we;
    reg  [AW-1:0]       waddr, raddr;
    reg  [DATA_W-1:0]   din;
    wire [DATA_W-1:0]   dout;

    integer verif_errors, verif_checks;
    `include "../verif/checks.vh"

    bram_sdp #(.DEPTH(DEPTH), .DATA_W(DATA_W)) dut (
        .clk(clk), .we(we), .waddr(waddr), .din(din), .raddr(raddr), .dout(dout));

    initial clk = 0; always #50 clk = ~clk;

    reg [DATA_W-1:0] model [0:DEPTH-1];
    integer i, k;

    // One driven cycle.  Sample OLD value at raddr (read-first) BEFORE the write
    // commits, advance the clock, settle, check dout, then commit the write.
    task step;
        input [255:0] tag;
        reg [DATA_W-1:0] pred;
        begin
            pred = model[raddr];
            @(posedge clk); #1;
            check_eq_u(dout, pred, tag);
            if (we) model[waddr] = din;
        end
    endtask

    task warmup;
        begin
            for (i = 0; i < DEPTH; i = i + 1) begin
                we = 1; waddr = i[AW-1:0]; raddr = 0; din = 32'hB00C_0000 | i;
                @(posedge clk); #1;
                model[i] = 32'hB00C_0000 | i;
            end
            we = 0; waddr = 0; raddr = 0; din = 0; @(posedge clk); #1;
        end
    endtask

    initial begin
        verif_errors = 0; verif_checks = 0;
        we = 0; waddr = 0; raddr = 0; din = 0;
        for (i = 0; i < DEPTH; i = i + 1) model[i] = 0;
        @(posedge clk); #1;
        $display("=== tb_bram_sdp ===");
        warmup;

        // ---- directed: independent ports (write a8 while reading a3) ----
        we = 1; waddr = 6'd8; raddr = 6'd3;  din = 32'h1111_2222; step("W a8 | R a3");
        // ---- read-after-write: read a8 next cycle, expect new value ----
        we = 0; waddr = 0;    raddr = 6'd8;  din = 0;             step("R a8 new");

        // ---- collision: write+read same address -> dout = OLD value ----
        // a8 currently holds 1111_2222; overwrite while reading it.
        we = 1; waddr = 6'd8; raddr = 6'd8;  din = 32'h3333_4444; step("RW a8 collide");
        we = 0; waddr = 0;    raddr = 6'd8;  din = 0;             step("R a8 after coll");
        check_eq_u(model[8], 32'h3333_4444, "model a8 updated by collide");

        // ---- constrained-random stream ----
        void'($urandom(32'h5DD9_0001));
        for (k = 0; k < NVEC; k = k + 1) begin
            we    = $urandom & 1'b1;
            waddr = $urandom_range(DEPTH-1);
            raddr = $urandom_range(DEPTH-1);
            din   = $urandom;
            step("rand");
        end

        `VERIF_EPILOGUE("tb_bram_sdp")
    end

    `VERIF_WATCHDOG(2000000)

endmodule
