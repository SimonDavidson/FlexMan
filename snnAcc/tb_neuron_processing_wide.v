// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
// =============================================================================
// tb_neuron_processing_wide — AGGRESSIVE unit test for the lane-parallel
// neuron datapath.  Authors: Simon Davidson & Claude.
// Created: 2026-06-04 ; Last modified: 2026-06-07
//
// Drives whole LAYERS of random neurons through neuron_processing_wide and
// checks EVERY memory write-back (decayed syn_curr, decayed potential, packed
// spike word) against the per-neuron np_ref_lif golden (verif/np_ref.vh).  The
// per-neuron math is identical to standard snnAcc, so np_ref_lif applies per
// lane exactly; the goldens are lane-count- and prefetch-independent, so
// passing identically across LANES in {1,2,4} and PF in {0,1} is itself a
// strong equivalence check.
//
// Element widths: syn_curr & potential are EB-wide (8 or 16, set by -define EB),
// bias & threshold are 8-bit (BIAS_CURR_SLICE_BITS, packed 4/word).  The golden
// sign-extends each operand from its native width, runs np_ref_lif, then
// truncates syn_o/pot_o back to EB for the writeback compare (the RTL stores the
// low EB bits).
//
// Coverage: random per-neuron syn/bias/thresh/pot in small signed ranges (good
// spike mix), mixed spike / no-spike, the 32-neuron spike-word boundary,
// clear_pot, sub_on_fire, decay-rate sweeps, and read-path back-pressure on
// bias & thresh (result must be latency-invariant).
//
// -define knobs: LANES (1/2/4 for 8-bit, 1/2 for 16-bit), EB (8 or 16),
//                PF (0/1), NN (neurons, default 40).
// =============================================================================
`timescale 1ns/1ps
`include "../shared/constants.v"

`ifndef LANES
 `define LANES 2
`endif
`ifndef EB
 `define EB 8
`endif
`ifndef PF
 `define PF 0
`endif
`ifndef NN
 `define NN 40
`endif

module tb_neuron_processing_wide;
    localparam CLK   = 10;
    localparam LANES = `LANES;
    localparam EB    = `EB;             // syn_curr / pot element width
    localparam BB    = 8;              // bias / threshold element width
    localparam NNMAX = 128;
    localparam SLICE_SZ = (EB==8) ? 3 : (EB==16) ? 4 : 5;
    localparam NEURON_IDX_SZ_LP = 10;
    localparam EPW   = 32/EB;
    localparam BPW   = 32/BB;           // bias/thresh elements per word

    localparam SYN_BASE=0, POT_BASE=64, BIAS_BASE=128, THR_BASE=160, SPK_BASE=192;
    localparam MEM_DEPTH = 256;

    reg clk = 0; always #(CLK/2) clk = ~clk;
    reg reset;

    integer verif_errors, verif_checks, verif_to;
    `include "../verif/checks.vh"
    `include "../verif/np_ref.vh"

    // config
    reg [NEURON_IDX_SZ_LP-1:0] last_idx;
    reg [31:0] syn_dcy, pot_dcy;
    reg        sub_on_fire, clear_pot;

    // DUT memory buses
    wire                  sc_wr, sc_rd; wire [`ADDR_SIZE-1:0] sc_addr;
    wire [`POT_BITS-1:0]  sc_wdata; wire [`POT_BITS-1:0] sc_rdata;
    wire                  b_rd;  wire [`ADDR_SIZE-1:0] b_addr;  wire [`WTD_BITS-1:0] b_rdata;
    wire                  th_rd; wire [`ADDR_SIZE-1:0] th_addr; wire [`WTD_BITS-1:0] th_rdata;
    wire                  p_wr, p_rd; wire [`ADDR_SIZE-1:0] p_addr;
    wire [`POT_BITS-1:0]  p_wdata; wire [`POT_BITS-1:0] p_rdata;
    wire                  spk_wr; wire [`ADDR_SIZE-1:0] spk_addr; wire [`ACT_BITS-1:0] spk_wdata;

    reg start; wire busy, fin, npfin;
    reg b_wait, th_wait;

    neuron_processing_wide #(
        .TGT_ACC_ID(3'b000), .NUM_LANES(LANES), .PREFETCH(`PF),
        .NEURON_IDX_SZ(NEURON_IDX_SZ_LP),
        .SYN_CURR_SLICE_BITS(EB), .POT_SLICE_BITS(EB),
        .BIAS_CURR_SLICE_BITS(BB),
        .SYN_DECAY_BITS(32), .POT_DECAY_BITS(32)
    ) dut (
        .clk(clk), .reset(reset),
        .last_neuron_idx_i(last_idx),
        .syn_curr_base_addr_i(SYN_BASE), .bias_curr_base_addr_i(BIAS_BASE),
        .thresh_base_addr_i(THR_BASE), .pot_base_addr_i(POT_BASE),
        .spike_base_addr_i(SPK_BASE),
        .syn_curr_sz_i(SLICE_SZ[2:0]), .bias_curr_sz_i(3'd3), .pot_sz_i(SLICE_SZ[2:0]),
        .bin_point_syn_curr_i(5'd0),
        .syn_curr_decay_mult_i(syn_dcy), .pot_decay_mult_i(pot_dcy),
        .sub_on_fire_i(sub_on_fire), .clear_pot_i(clear_pot),
        .start_new_block_i(start), .target_acc_i(3'b000), .buffer_info_i(32'b0),
        .neuron_proc_finished_o(npfin), .acc_busy_o(busy), .acc_finished_o(fin),
        .src1_buff_addr_i(10'b0), .src2_buff_addr_i(10'b0), .src3_buff_addr_i(10'b0),
        .tgt_buff_addr_i(10'b0), .weight_row_len_i(10'b0),
        .syn_curr_mem_wr_o(sc_wr), .syn_curr_mem_rd_o(sc_rd), .syn_curr_mem_wait_i(1'b0),
        .syn_curr_mem_addr_o(sc_addr), .syn_curr_mem_data_o(sc_wdata), .syn_curr_mem_data_i(sc_rdata),
        .bias_curr_mem_rd_o(b_rd), .bias_curr_mem_wait_i(b_wait),
        .bias_curr_mem_addr_o(b_addr), .bias_curr_mem_data_i(b_rdata),
        .thresh_mem_rd_o(th_rd), .thresh_mem_wait_i(th_wait),
        .thresh_mem_addr_o(th_addr), .thresh_mem_data_i(th_rdata),
        .pot_mem_wr_o(p_wr), .pot_mem_rd_o(p_rd), .pot_mem_wait_i(1'b0),
        .pot_mem_addr_o(p_addr), .pot_mem_data_o(p_wdata), .pot_mem_data_i(p_rdata),
        .spike_mem_wr_o(spk_wr), .spike_mem_wait_i(1'b0),
        .spike_mem_addr_o(spk_addr), .spike_mem_data_o(spk_wdata)
    );

    // syn & pot are read-modify-write (gated by ~mem_wait, but tied 0 here);
    // bias & thresh honour their mem_wait.
    sram_model #(.DATA_W(32),.DEPTH(MEM_DEPTH)) u_syn (.clk(clk),.we(sc_wr),.re(sc_rd),.rwait(1'b0),
        .addr(sc_addr[7:0]),.wdata(sc_wdata),.rdata(sc_rdata));
    sram_model #(.DATA_W(32),.DEPTH(MEM_DEPTH)) u_bias(.clk(clk),.we(1'b0),.re(b_rd),.rwait(b_wait),
        .addr(b_addr[7:0]),.wdata(32'b0),.rdata(b_rdata));
    sram_model #(.DATA_W(32),.DEPTH(MEM_DEPTH)) u_thr (.clk(clk),.we(1'b0),.re(th_rd),.rwait(th_wait),
        .addr(th_addr[7:0]),.wdata(32'b0),.rdata(th_rdata));
    sram_model #(.DATA_W(32),.DEPTH(MEM_DEPTH)) u_pot (.clk(clk),.we(p_wr),.re(p_rd),.rwait(1'b0),
        .addr(p_addr[7:0]),.wdata(p_wdata),.rdata(p_rdata));
    sram_model #(.DATA_W(32),.DEPTH(MEM_DEPTH)) u_spk (.clk(clk),.we(spk_wr),.re(1'b0),.rwait(1'b0),
        .addr(spk_addr[7:0]),.wdata(spk_wdata),.rdata());

    // ---- race-free back-pressure on bias & thresh reads ----
    reg bp_en;
    always @(posedge clk) begin
        b_wait  <= bp_en ? ($urandom_range(99) < 30) : 1'b0;
        th_wait <= bp_en ? ($urandom_range(99) < 30) : 1'b0;
    end

    // ---- golden arrays ----
    reg signed [31:0] in_syn[0:NNMAX-1], in_bias[0:NNMAX-1], in_thr[0:NNMAX-1], in_pot[0:NNMAX-1];
    reg signed [31:0] g_syn [0:NNMAX-1], g_pot [0:NNMAX-1];
    reg               g_spk [0:NNMAX-1];

    // ---- EB-width LIF golden (mirrors update_state_for_neuron at slice width) ----
    // The wide datapath computes the membrane sum, saturation, threshold compare
    // and decay at SLICE width (EB), NOT 32-bit.  bias & threshold are BB-bit,
    // sign-extended into the EB domain.  Decay reuses np_decay_signed (signed
    // Q0.32) then truncates to EB (magnitude fits since |operand|<2^(EB-1)).
    //   pot_o (no spike)        = decay_EB(psat, pot_dcy)
    //   pot_o (spike, !sof)     = 0
    //   pot_o (spike, sof)      = decay_EB(psat, pot_dcy) - thresh
    //   syn_o                   = decay_EB(syn, syn_dcy)
    task np_ref_lif_w;
        input signed [EB-1:0] syn;     // EB-bit signed
        input signed [EB-1:0] pot;     // EB-bit signed (0 if clear_pot)
        input signed [BB-1:0] bias;    // BB-bit signed
        input signed [BB-1:0] thr;     // BB-bit signed
        input        [31:0]   sdcy, pdcy;
        input                 sof;
        output                spk_o;
        output signed [EB-1:0] po, so;
        reg signed [EB+1:0] sum;       // EB+2-bit sum (matches new_potential width)
        reg signed [EB-1:0] psat, dpot;
        reg ov, un;
        reg signed [EB-1:0] thr_eb;
        begin
            thr_eb = thr;                              // BB->EB sign-extend
            sum = $signed({{2{pot[EB-1]}},  pot})
                + $signed({{(EB+2-BB){bias[BB-1]}}, bias})
                + $signed({{2{syn[EB-1]}},  syn});
            ov = ~sum[EB-1] & (sum[EB+1:EB] != 2'b00);
            un =  sum[EB-1] & (sum[EB+1:EB] != 2'b11);
            psat = ov ? {1'b0, {(EB-1){1'b1}}} :
                   un ? {1'b1, {(EB-1){1'b0}}} : sum[EB-1:0];
            spk_o = (psat >= thr_eb);
            dpot  = np_decay_signed($signed(psat), pdcy);   // low EB bits
            po    = spk_o ? (sof ? (dpot - thr_eb) : {EB{1'b0}}) : dpot;
            so    = np_decay_signed($signed(syn), sdcy);
        end
    endtask

    reg              gspk;
    reg signed [EB-1:0] gpo, gso;

    integer n, w, off, t, wi;
    reg [31:0] ew;
    reg signed [EB-1:0] s_syn, s_pot;
    reg signed [BB-1:0] s_bias, s_thr;

    task clear_mem;
        begin
            for (t=0;t<MEM_DEPTH;t=t+1) begin
                u_syn.mem[t]=0; u_bias.mem[t]=0; u_thr.mem[t]=0;
                u_pot.mem[t]=0; u_spk.mem[t]=0;
            end
        end
    endtask

    // Run one layer of N random neurons; small signed ranges -> good spike mix.
    task run_layer;
        input integer N;
        input         sof;
        input         clrpot;
        input         bp;
        input [255:0] tag;
        begin
            @(negedge clk); reset = 1; bp_en = 0;
            clear_mem;
            for (n=0;n<N;n=n+1) begin
                in_syn[n]  = $urandom_range(0, (1<<EB)-1) - (1<<(EB-1)); // full EB signed
                in_pot[n]  = $urandom_range(0, (1<<EB)-1) - (1<<(EB-1));
                in_bias[n] = $urandom_range(0, 40) - 20;                 // 8-bit signed small
                in_thr[n]  = $urandom_range(0, (1<<(EB-1))-1);           // non-neg threshold
                // keep thr within 8-bit signed positive range
                if (in_thr[n] > 120) in_thr[n] = 120;
                // pack into memory at native widths
                w = SYN_BASE + (n/EPW); off=(n%EPW)*EB;
                u_syn.mem[w][off +: EB] = in_syn[n][EB-1:0];
                w = POT_BASE + (n/EPW);
                u_pot.mem[w][off +: EB] = in_pot[n][EB-1:0];
                w = BIAS_BASE + (n/BPW); off=(n%BPW)*BB;
                u_bias.mem[w][off +: BB] = in_bias[n][BB-1:0];
                w = THR_BASE + (n/BPW);
                u_thr.mem[w][off +: BB] = in_thr[n][BB-1:0];
                // EB-width golden (clear_pot zeros the pot operand)
                np_ref_lif_w(in_syn[n][EB-1:0], clrpot ? {EB{1'b0}} : in_pot[n][EB-1:0],
                             in_bias[n][BB-1:0], in_thr[n][BB-1:0],
                             syn_dcy, pot_dcy, sof, gspk, gpo, gso);
                g_spk[n] = gspk; g_pot[n] = gpo; g_syn[n] = gso;
            end

            last_idx = N-1; sub_on_fire = sof; clear_pot = clrpot;
            repeat (2) @(posedge clk); @(negedge clk); reset = 0; repeat (2) @(posedge clk);
            bp_en = bp;
            @(negedge clk); start = 1; @(negedge clk); start = 0;

            verif_to = 8000;
            while (!fin && verif_to > 0) begin @(posedge clk); verif_to = verif_to - 1; end
            check_true(verif_to > 0, {tag, " finished"});
            repeat (8) @(posedge clk); #1;          // let packers drain
            bp_en = 0;

            // check syn_out / pot_out (EB-truncated golden) and packed spikes
            for (n=0;n<N;n=n+1) begin
                w=(n/EPW); off=(n%EPW)*EB;
                check_eq_u(u_syn.mem[SYN_BASE+w][off +: EB], g_syn[n][EB-1:0], {tag, " syn_wb"});
                check_eq_u(u_pot.mem[POT_BASE+w][off +: EB], g_pot[n][EB-1:0], {tag, " pot_wb"});
            end
            for (wi=0; wi*32 < N; wi=wi+1) begin
                ew = 32'b0;
                for (n=0; n<32 && wi*32+n < N; n=n+1) ew[n] = g_spk[wi*32+n];
                check_eq_u(u_spk.mem[SPK_BASE+wi], ew, {tag, " spike_word"});
            end
        end
    endtask

    initial begin
        verif_errors=0; verif_checks=0;
        start=0; reset=1; bp_en=0; b_wait=0; th_wait=0;
        sub_on_fire=0; clear_pot=0;
        syn_dcy=32'h8000_0000; pot_dcy=32'h8000_0000;  // 0.5
        last_idx=0;
        repeat (4) @(posedge clk); @(negedge clk); reset=0; repeat (2) @(posedge clk);

        $display("=== tb_neuron_processing_wide  LANES=%0d EB=%0d PF=%0d ===", LANES, EB, `PF);
        void'($urandom(32'h4E50_0000 ^ (EB<<8) ^ (LANES<<4) ^ `PF));

        run_layer(`NN, 1'b0, 1'b0, 1'b0, "B1 random no-flags");
        run_layer(`NN, 1'b0, 1'b1, 1'b0, "B2 clear_pot");
        run_layer(`NN, 1'b1, 1'b0, 1'b0, "B3 sub_on_fire");
        run_layer(33,   1'b0, 1'b0, 1'b0, "B4 two spike words");
        run_layer(20,   1'b0, 1'b0, 1'b1, "B5 backpressure");
        // decay sweeps
        syn_dcy=32'hC000_0000; pot_dcy=32'h4000_0000;  // 0.75 / 0.25
        run_layer(`NN, 1'b1, 1'b0, 1'b1, "B6 decay .75/.25 + sof + bp");
        syn_dcy=32'hFFFF_FFFF; pot_dcy=32'h0000_0000;  // ~1.0 / 0.0
        run_layer(24,   1'b0, 1'b0, 1'b0, "B7 decay extremes");
        syn_dcy=32'h0000_0000; pot_dcy=32'hFFFF_FFFF;  // 0.0 / ~1.0
        run_layer(`NN, 1'b1, 1'b1, 1'b0, "B8 decay 0/~1 + sof + clrpot");

        `VERIF_EPILOGUE("tb_neuron_processing_wide")
    end

    `VERIF_WATCHDOG(20000000)
endmodule

// ----- synchronous SRAM model (1-cycle read latency, registered write) -----
// read gated by ~rwait (back-pressure); write unconditional on we.
module sram_model #(parameter DATA_W=32, parameter DEPTH=256)(
    input wire clk, input wire we, input wire re, input wire rwait,
    input wire [7:0] addr, input wire [DATA_W-1:0] wdata, output reg [DATA_W-1:0] rdata);
    reg [DATA_W-1:0] mem [0:DEPTH-1];
    integer i; initial begin for(i=0;i<DEPTH;i=i+1) mem[i]={DATA_W{1'b0}}; rdata={DATA_W{1'b0}}; end
    always @(posedge clk) begin
        if (we) mem[addr] <= wdata;
        if (re & ~rwait) rdata <= mem[addr];
    end
endmodule
