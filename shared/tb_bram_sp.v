// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
// =============================================================================
// tb_bram_sp  --  single-port synchronous read-first BRAM
//
// Authors      : Simon Davidson & Claude
// Created      : 2026-06-07
// Last modified: 2026-06-07
//
// Read-first model: every cycle dout <= mem[addr] (OLD value), then if(we)
// mem[addr] <= din.  The SW golden maintains an identical `model` array updated
// with the SAME read-first ordering and a 1-cycle-delayed expected-dout pipe
// register, so the DUT and golden stay phase-aligned.  Covers:
//   * write-then-read-back
//   * same-cycle R/W collision on one address (dout must be the OLD value)
//   * a long constrained-random write/read stream with frequent collisions
// =============================================================================
`timescale 10ps/1ps

module tb_bram_sp;

    localparam integer DEPTH  = 64;
    localparam integer DATA_W = 32;
    localparam integer AW     = 6;            // $clog2(64)
    localparam integer NVEC   = 4000;

    reg                 clk;
    reg                 we;
    reg  [AW-1:0]       addr;
    reg  [DATA_W-1:0]   din;
    wire [DATA_W-1:0]   dout;

    integer verif_errors, verif_checks;
    `include "../verif/checks.vh"

    bram_sp #(.DEPTH(DEPTH), .DATA_W(DATA_W)) dut (
        .clk(clk), .we(we), .addr(addr), .din(din), .dout(dout));

    initial clk = 0; always #50 clk = ~clk;   // 10ps units -> 1ns period

    // ---- software golden (same read-first timing) -------------------------
    reg [DATA_W-1:0] model [0:DEPTH-1];

    integer i, k;

    // Apply one driven cycle.  Inputs (we/addr/din) are set up by the caller
    // BEFORE this is called.  At the posedge the DUT registers dout<=mem[addr]
    // (OLD value, read-first) and mem[addr]<=din.  After a #1 settle, the
    // registered dout reflects the OLD value of THIS op's address, which is
    // exactly `pred`.  The golden then commits the write into `model`.
    task step;
        input [255:0] tag;
        reg [DATA_W-1:0] pred;
        begin
            pred = model[addr];        // read-first OLD value for THIS op
            @(posedge clk); #1;        // edge: dout<=mem[addr]; settle NBAs
            check_eq_u(dout, pred, tag);
            if (we) model[addr] = din; // commit write active during this cycle
        end
    endtask

    // Warm up DUT + golden so neither holds X: write every address once.
    task warmup;
        begin
            for (i = 0; i < DEPTH; i = i + 1) begin
                we = 1; addr = i[AW-1:0]; din = 32'hC0DE_0000 | i;
                @(posedge clk); #1;
                model[i] = 32'hC0DE_0000 | i;
            end
            we = 0; addr = 0; din = 0; @(posedge clk); #1;
        end
    endtask

    initial begin
        verif_errors = 0; verif_checks = 0;
        we = 0; addr = 0; din = 0;
        for (i = 0; i < DEPTH; i = i + 1) model[i] = 0;
        @(posedge clk); #1;
        $display("=== tb_bram_sp ===");
        warmup;

        // ---- directed: write then read back ----
        // (checks are pipelined: each step verifies dout for the PREVIOUS op.)
        we = 1; addr = 6'd5;  din = 32'hDEAD_BEEF; step("W a5=DEADBEEF");
        we = 0; addr = 6'd5;  din = 0;             step("R a5 back");
        we = 1; addr = 6'd10; din = 32'h1234_5678; step("W a10");
        we = 0; addr = 6'd10; din = 0;             step("R a10 back");

        // ---- directed: same-cycle R/W collision -> dout must be OLD value ----
        // a5 holds DEADBEEF.  Write AAAA5555 while reading a5: dout=OLD DEADBEEF,
        // memory updates to AAAA5555 (verified by the following read).
        we = 1; addr = 6'd5; din = 32'hAAAA_5555; step("RW a5 collide");
        we = 0; addr = 6'd5; din = 0;             step("R a5 after coll");
        we = 0; addr = 6'd0; din = 0;             step("idle flush");
        check_eq_u(model[5], 32'hAAAA_5555, "model a5 updated by collide");

        // ---- constrained-random stream (frequent collisions via small DEPTH) --
        void'($urandom(32'hB7A4_0001));
        for (k = 0; k < NVEC; k = k + 1) begin
            we   = $urandom & 1'b1;
            addr = $urandom_range(DEPTH-1);
            din  = $urandom;
            step("rand");
        end

        // ---- final settle check ----
        we = 0; addr = 0; din = 0; step("settle");

        `VERIF_EPILOGUE("tb_bram_sp")
    end

    `VERIF_WATCHDOG(2000000)

endmodule
