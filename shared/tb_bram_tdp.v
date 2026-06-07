// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
// =============================================================================
// tb_bram_tdp  --  true dual-port synchronous read-first BRAM
//
// Authors      : Simon Davidson & Claude
// Created      : 2026-06-07
// Last modified: 2026-06-07
//
// Two fully independent ports A/B, each gated by ena/enb, each read-first.  Per
// port, when enabled: dout<=mem[addr] (OLD) then if(we) mem[addr]<=din.  Both
// always-blocks fire on the same posedge: every dout RHS is sampled before any
// write commits, so a cross-port collision (A writes X while B reads X) returns
// the OLD value of X on port B (read-first cross-port).
//
// The SW golden samples BOTH ports' OLD read values BEFORE committing EITHER
// write, then commits both writes — reproducing the NBA read-first semantics.
//
// NOTE: the testbench never has both ports write the SAME address in one cycle
// (that is a hardware race with undefined winner); the random generator forces
// the B write-address away from A's whenever both write.  Covers:
//   * two ports, independent addresses, read/write mix
//   * cross-port collision: A writes X, B reads X -> B sees OLD X
//   * en gating: a disabled port holds its dout and does not write
//   * a long constrained-random two-port stream
// =============================================================================
`timescale 10ps/1ps

module tb_bram_tdp;

    localparam integer DEPTH  = 64;
    localparam integer DATA_W = 32;
    localparam integer AW     = 6;
    localparam integer NVEC   = 4000;

    reg                 clk;
    reg                 ena, wea, enb, web;
    reg  [AW-1:0]       addra, addrb;
    reg  [DATA_W-1:0]   dina, dinb;
    wire [DATA_W-1:0]   douta, doutb;

    integer verif_errors, verif_checks;
    `include "../verif/checks.vh"

    bram_tdp #(.DEPTH(DEPTH), .DATA_W(DATA_W)) dut (
        .clk(clk),
        .ena(ena), .wea(wea), .addra(addra), .dina(dina), .douta(douta),
        .enb(enb), .web(web), .addrb(addrb), .dinb(dinb), .doutb(doutb));

    initial clk = 0; always #50 clk = ~clk;

    reg [DATA_W-1:0] model [0:DEPTH-1];
    // Predicted dout values: only updated when the port is enabled; otherwise the
    // DUT's registered dout HOLDS its previous value, so the golden holds too.
    reg [DATA_W-1:0] exp_a, exp_b;
    reg              va, vb;          // prediction validity (after first enabled op)
    integer i, k, ra;

    // One driven cycle for both ports.  Capture OLD reads first, advance clock,
    // settle, check, then commit writes.  dout HOLDS across a disabled cycle.
    task step;
        input [255:0] tag;
        reg [DATA_W-1:0] pred_a, pred_b;
        reg              chka, chkb;
        begin
            // Sample read-first OLD values BEFORE any write commits this cycle.
            pred_a = model[addra];
            pred_b = model[addrb];
            chka = ena; chkb = enb;
            @(posedge clk); #1;
            // Check current dout: if the port was enabled it took the OLD value;
            // if disabled it held the previous prediction.
            if (chka) begin exp_a = pred_a; va = 1'b1; end
            if (chkb) begin exp_b = pred_b; vb = 1'b1; end
            if (va) check_eq_u(douta, exp_a, {tag, " A"});
            if (vb) check_eq_u(doutb, exp_b, {tag, " B"});
            // Commit writes (both, AFTER both reads sampled).
            if (ena && wea) model[addra] = dina;
            if (enb && web) model[addrb] = dinb;
        end
    endtask

    task warmup;
        begin
            for (i = 0; i < DEPTH; i = i + 1) begin
                ena = 1; wea = 1; addra = i[AW-1:0]; dina = 32'hDAD0_0000 | i;
                enb = 0; web = 0; addrb = 0; dinb = 0;
                @(posedge clk); #1;
                model[i] = 32'hDAD0_0000 | i;
            end
            ena=0; wea=0; enb=0; web=0; addra=0; addrb=0; dina=0; dinb=0;
            va=0; vb=0;
            @(posedge clk); #1;
        end
    endtask

    initial begin
        verif_errors = 0; verif_checks = 0;
        ena=0; wea=0; enb=0; web=0; addra=0; addrb=0; dina=0; dinb=0;
        va=0; vb=0;
        for (i = 0; i < DEPTH; i = i + 1) model[i] = 0;
        @(posedge clk); #1;
        $display("=== tb_bram_tdp ===");
        warmup;

        // ---- directed: independent addresses, A writes / B reads ----
        ena=1; wea=1; addra=6'd12; dina=32'hAAAA_0001;
        enb=1; web=0; addrb=6'd20;                       step("A W a12 | B R a20");

        // ---- directed: cross-port collision A writes X, B reads X -> B OLD ----
        // a12 now holds AAAA_0001; A overwrites it while B reads it.
        ena=1; wea=1; addra=6'd12; dina=32'hBBBB_0002;
        enb=1; web=0; addrb=6'd12;                       step("A W a12 | B R a12 OLD");
        // next cycle B reads a12 again -> sees the new value BBBB_0002
        ena=0; wea=0; addra=0; dina=0;
        enb=1; web=0; addrb=6'd12;                       step("B R a12 new");
        check_eq_u(model[12], 32'hBBBB_0002, "model a12 after cross-port write");

        // ---- directed: en gating, B disabled holds its dout ----
        // B is disabled this cycle: doutb must HOLD (== previous BBBB_0002).
        ena=1; wea=1; addra=6'd30; dina=32'hCCCC_0003;
        enb=0; web=0; addrb=6'd5;                        step("A W a30 | B disabled hold");

        // ---- constrained-random two-port stream ----
        void'($urandom(32'h7DDA_0001));
        for (k = 0; k < NVEC; k = k + 1) begin
            ena   = $urandom & 1'b1;  wea = $urandom & 1'b1;
            enb   = $urandom & 1'b1;  web = $urandom & 1'b1;
            addra = $urandom_range(DEPTH-1);
            ra    = $urandom_range(DEPTH-1);
            // Avoid both ports writing the SAME address (hardware race).
            if (ena && wea && enb && web && (ra == addra))
                ra = (ra + 1) % DEPTH;
            addrb = ra[AW-1:0];
            dina  = $urandom; dinb = $urandom;
            step("rand");
        end

        `VERIF_EPILOGUE("tb_bram_tdp")
    end

    `VERIF_WATCHDOG(2000000)

endmodule
