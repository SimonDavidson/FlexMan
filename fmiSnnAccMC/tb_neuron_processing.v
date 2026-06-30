// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
// =============================================================================
// tb_neuron_processing  (fmiSnnAcc, FMI per-neuron-decay + adaptive-threshold)
//
// Authors      : Simon Davidson & Claude
// Created      : 2026-06-08
// Last modified: 2026-06-30
//
// FIRST aggressive unit test for fmiSnnAcc/neuron_processing.  Drives whole
// layers of neurons through the neuron loop and checks every memory write-back
// (decayed syn_curr, decayed/new potential, packed spike word, and — when
// has_ada=1 — the updated ada state) against the np_ref_fmi golden.
//
// Covers:
//   * many neurons, random data, mixed spike / no-spike
//   * the 32-neuron spike-word boundary (N>32 -> two packed spike words)
//   * BOTH the plain (has_ada=0) and adaptive (has_ada=1) datapaths
//       - 2-cycle FSM latency (plain) and 3-cycle FSM latency (adaptive)
//   * per-neuron dcy_syn / dcy_mem (Q0.32) over the full range incl small/large
//   * random ada / b_eff / dcy_ada / scl_ada in the adaptive blocks;
//     ada writeback checked ONLY when has_ada=1 (golden ada_o is 0 otherwise)
//   * read-path back-pressure on the read-only memories (random mem_wait):
//     result must be latency-invariant
//   * clear_pot directed corner (golden fed pot=0 to mirror the DUT)
//
// 32-bit slices throughout (one element per word).  Sub-32-bit packed
// write-back is NOT exercised here: with several neurons per word the spike
// packer (32/word) and the syn_curr/pot packers (e.g. 2/word for 16-bit)
// fill at different cadences, which the dedicated packer test must cover —
// and which surfaced a real RTL defect during bring-up (see report).
//
// SEPARATE SRAM arrays per memory (distinct bases) so there is no aliasing.
// Memory bases follow the integration tb convention:
//   syn_curr=20 thresh=40 pot=50 spike=60
//   dcy_syn=70 dcy_mem=80 ada=90 b_eff=100 dcy_ada=110 scl_ada=120
// =============================================================================
`timescale 1ns/1ps
`include "../shared/constants.v"

module tb_neuron_processing;

    localparam NEURON_IDX_SZ = 7;          // up to 127 neurons
    localparam SLICE_BITS    = 32;
    localparam SZ32          = 3'b101;      // 32-bit slice code

    reg clk, reset;

    // Config
    reg [NEURON_IDX_SZ-1:0] last_neuron_idx;
    reg [`ADDR_SIZE-1:0]    syn_curr_base, thresh_base, pot_base, spike_base;
    reg [`ADDR_SIZE-1:0]    dcy_syn_base, dcy_mem_base, ada_base, b_eff_base, dcy_ada_base, scl_ada_base;
    reg [2:0]               syn_curr_sz, pot_sz;
    reg [4:0]               bin_point_syn_curr;
    reg                     sub_on_fire, clear_pot, has_ada;

    // Scheduler iface
    reg                     start_new_block;
    reg [`TGT_ACC_SZ-1:0]   target_acc;
    reg [`SCH_ENTRY_SZ-1:0] buffer_info;
    reg [`PIN_BITS-1:0]     src1, src2, src3, tgt, wrl;

    wire neuron_proc_finished, acc_busy, acc_finished;

    // syn_curr (rd+wr)
    wire                  syn_curr_mem_wr, syn_curr_mem_rd;
    reg                   syn_curr_mem_wait;
    wire [`ADDR_SIZE-1:0] syn_curr_mem_addr;
    wire [`POT_BITS-1:0]  syn_curr_mem_data_wr;
    reg  [`POT_BITS-1:0]  syn_curr_mem_data_rd;
    // thresh (rd)
    wire thresh_mem_rd; reg thresh_mem_wait;
    wire [`ADDR_SIZE-1:0] thresh_mem_addr; reg [`WTD_BITS-1:0] thresh_mem_data;
    // pot (rd+wr)
    wire pot_mem_wr, pot_mem_rd; reg pot_mem_wait;
    wire [`ADDR_SIZE-1:0] pot_mem_addr; wire [`POT_BITS-1:0] pot_mem_data_wr; reg [`POT_BITS-1:0] pot_mem_data_rd;
    // spike (wr)
    wire spike_mem_wr; reg spike_mem_wait;
    wire [`ADDR_SIZE-1:0] spike_mem_addr; wire [`ACT_BITS-1:0] spike_mem_data;
    // dcy_syn (rd)
    wire dcy_syn_mem_rd; reg dcy_syn_mem_wait;
    wire [`ADDR_SIZE-1:0] dcy_syn_mem_addr; reg [31:0] dcy_syn_mem_data;
    // dcy_mem (rd)
    wire dcy_mem_mem_rd; reg dcy_mem_mem_wait;
    wire [`ADDR_SIZE-1:0] dcy_mem_mem_addr; reg [31:0] dcy_mem_mem_data;
    // ada (rd+wr)
    wire ada_mem_wr, ada_mem_rd; reg ada_mem_wait;
    wire [`ADDR_SIZE-1:0] ada_mem_addr; wire [31:0] ada_mem_data_wr; reg [31:0] ada_mem_data_rd;
    // b_eff (rd)
    wire b_eff_mem_rd; reg b_eff_mem_wait;
    wire [`ADDR_SIZE-1:0] b_eff_mem_addr; reg [31:0] b_eff_mem_data;
    // dcy_ada (rd)
    wire dcy_ada_mem_rd; reg dcy_ada_mem_wait;
    wire [`ADDR_SIZE-1:0] dcy_ada_mem_addr; reg [31:0] dcy_ada_mem_data;
    // scl_ada (rd)
    wire scl_ada_mem_rd; reg scl_ada_mem_wait;
    wire [`ADDR_SIZE-1:0] scl_ada_mem_addr; reg [31:0] scl_ada_mem_data;

    integer verif_errors, verif_checks, verif_to, sram_i;
    `include "../verif/checks.vh"
    `include "../verif/np_ref.vh"
    `include "../verif/vt_driver.vh"
    `include "../verif/sram_bfm.vh"

    neuron_processing #(
        .NEURON_IDX_SZ(NEURON_IDX_SZ),
        .SYN_CURR_IDX_SZ(10), .SYN_CURR_DATA_IDX_SZ(5),
        .SYN_CURR_SLICE_SZ(3), .SYN_CURR_SLICE_BITS(SLICE_BITS),
        .POT_IDX_SZ(NEURON_IDX_SZ), .POT_DATA_IDX_SZ(5), .POT_SLICE_SZ(3), .POT_SLICE_BITS(SLICE_BITS),
        .SPIKE_IDX_SZ(NEURON_IDX_SZ), .SPIKE_DATA_IDX_SZ(5), .SPIKE_SLICE_SZ(3), .SPIKE_SLICE_BITS(8))
    dut (
        .clk(clk), .reset(reset),
        .last_neuron_idx_i(last_neuron_idx),
        .syn_curr_base_addr_i(syn_curr_base),
        .pot_base_addr_i(pot_base), .spike_base_addr_i(spike_base),
        .thresh_base_addr_i(thresh_base),
        .syn_curr_sz_i(syn_curr_sz), .pot_sz_i(pot_sz),
        .bin_point_syn_curr_i(bin_point_syn_curr),
        .sub_on_fire_i(sub_on_fire), .clear_pot_i(clear_pot),
        .dcy_syn_base_addr_i(dcy_syn_base), .dcy_mem_base_addr_i(dcy_mem_base),
        .has_ada_i(has_ada),
        .readout_mode_i(1'b0),   // default-off: plain/adaptive LIF (readout = con6 only)
        .ada_base_addr_i(ada_base), .b_eff_base_addr_i(b_eff_base),
        .dcy_ada_base_addr_i(dcy_ada_base), .scl_ada_base_addr_i(scl_ada_base),
        .start_new_block_i(start_new_block), .target_acc_i(target_acc), .buffer_info_i(buffer_info),
        .neuron_proc_finished_o(neuron_proc_finished), .acc_busy_o(acc_busy), .acc_finished_o(acc_finished),
        .src1_buff_addr_i(src1), .src2_buff_addr_i(src2), .src3_buff_addr_i(src3),
        .tgt_buff_addr_i(tgt), .weight_row_len_i(wrl),
        // syn_curr
        .syn_curr_mem_wr_o(syn_curr_mem_wr), .syn_curr_mem_rd_o(syn_curr_mem_rd),
        .syn_curr_mem_wait_i(syn_curr_mem_wait), .syn_curr_mem_addr_o(syn_curr_mem_addr),
        .syn_curr_mem_data_o(syn_curr_mem_data_wr), .syn_curr_mem_data_i(syn_curr_mem_data_rd),
        // thresh
        .thresh_mem_rd_o(thresh_mem_rd), .thresh_mem_wait_i(thresh_mem_wait),
        .thresh_mem_addr_o(thresh_mem_addr), .thresh_mem_data_i(thresh_mem_data),
        // pot
        .pot_mem_wr_o(pot_mem_wr), .pot_mem_rd_o(pot_mem_rd), .pot_mem_wait_i(pot_mem_wait),
        .pot_mem_addr_o(pot_mem_addr), .pot_mem_data_o(pot_mem_data_wr), .pot_mem_data_i(pot_mem_data_rd),
        // spike
        .spike_mem_wr_o(spike_mem_wr), .spike_mem_wait_i(spike_mem_wait),
        .spike_mem_addr_o(spike_mem_addr), .spike_mem_data_o(spike_mem_data),
        // dcy_syn
        .dcy_syn_mem_rd_o(dcy_syn_mem_rd), .dcy_syn_mem_wait_i(dcy_syn_mem_wait),
        .dcy_syn_mem_addr_o(dcy_syn_mem_addr), .dcy_syn_mem_data_i(dcy_syn_mem_data),
        // dcy_mem
        .dcy_mem_mem_rd_o(dcy_mem_mem_rd), .dcy_mem_mem_wait_i(dcy_mem_mem_wait),
        .dcy_mem_mem_addr_o(dcy_mem_mem_addr), .dcy_mem_mem_data_i(dcy_mem_mem_data),
        // ada
        .ada_mem_wr_o(ada_mem_wr), .ada_mem_rd_o(ada_mem_rd), .ada_mem_wait_i(ada_mem_wait),
        .ada_mem_addr_o(ada_mem_addr), .ada_mem_data_o(ada_mem_data_wr), .ada_mem_data_i(ada_mem_data_rd),
        // b_eff
        .b_eff_mem_rd_o(b_eff_mem_rd), .b_eff_mem_wait_i(b_eff_mem_wait),
        .b_eff_mem_addr_o(b_eff_mem_addr), .b_eff_mem_data_i(b_eff_mem_data),
        // dcy_ada
        .dcy_ada_mem_rd_o(dcy_ada_mem_rd), .dcy_ada_mem_wait_i(dcy_ada_mem_wait),
        .dcy_ada_mem_addr_o(dcy_ada_mem_addr), .dcy_ada_mem_data_i(dcy_ada_mem_data),
        // scl_ada
        .scl_ada_mem_rd_o(scl_ada_mem_rd), .scl_ada_mem_wait_i(scl_ada_mem_wait),
        .scl_ada_mem_addr_o(scl_ada_mem_addr), .scl_ada_mem_data_i(scl_ada_mem_data));

    // ---- SRAM models. SEPARATE arrays per memory (no aliasing). -------------
    // R/W (read-modify-write style): syn_curr, pot, ada.  Each neuron owns a
    // distinct 32-bit word (one element per word), so read and write-back never
    // collide.
    // Wait-honouring so write back-pressure (bp_en) stalls the packer writeback —
    // the F8 path. The shared WAIT pin gates both read and write (mirrors the DUT
    // mem_wait_i = wait | wb_wr).
    `SRAM_RW_WAIT (syn_sram, syn_curr_mem_rd, syn_curr_mem_wr, syn_curr_mem_wait, syn_curr_mem_addr, syn_curr_mem_data_wr, syn_curr_mem_data_rd)
    `SRAM_RW_WAIT (pot_sram, pot_mem_rd, pot_mem_wr, pot_mem_wait, pot_mem_addr, pot_mem_data_wr, pot_mem_data_rd)
    `SRAM_RW_WAIT (ada_sram, ada_mem_rd, ada_mem_wr, ada_mem_wait, ada_mem_addr, ada_mem_data_wr, ada_mem_data_rd)
    // Read-only, honour mem_wait (back-pressure injection on all read-only mems).
    `SRAM_RD_WAIT(thr_sram,     thresh_mem_rd,  thresh_mem_wait,  thresh_mem_addr,  thresh_mem_data)
    `SRAM_RD_WAIT(dcy_syn_sram, dcy_syn_mem_rd, dcy_syn_mem_wait, dcy_syn_mem_addr, dcy_syn_mem_data)
    `SRAM_RD_WAIT(dcy_mem_sram, dcy_mem_mem_rd, dcy_mem_mem_wait, dcy_mem_mem_addr, dcy_mem_mem_data)
    `SRAM_RD_WAIT(b_eff_sram,   b_eff_mem_rd,   b_eff_mem_wait,   b_eff_mem_addr,   b_eff_mem_data)
    `SRAM_RD_WAIT(dcy_ada_sram, dcy_ada_mem_rd, dcy_ada_mem_wait, dcy_ada_mem_addr, dcy_ada_mem_data)
    `SRAM_RD_WAIT(scl_ada_sram, scl_ada_mem_rd, scl_ada_mem_wait, scl_ada_mem_addr, scl_ada_mem_data)
    // Write-only: spike (wait-honouring — the 1-bit packer's held flush is the
    // clearest F8 trigger).
    `SRAM_WR_WAIT (spk_sram, spike_mem_wr, spike_mem_wait, spike_mem_addr, spike_mem_data)

    initial clk = 0; always #5 clk = ~clk;

    // golden model arrays
    reg signed [31:0] g_syn [0:127];
    reg signed [31:0] g_pot [0:127];
    reg               g_spk [0:127];
    reg        [31:0] g_ada [0:127];
    reg signed [31:0] in_syn[0:127], in_thr[0:127], in_pot[0:127], in_beff[0:127];
    reg        [31:0] in_dsyn[0:127], in_dmem[0:127], in_ada[0:127], in_dada[0:127], in_sada[0:127];

    // back-pressure driver — when bp_en, drives BOTH the read-only mems' wait
    // (latency-invariance) AND the writeback mems' wait (the F8 packer-cadence
    // path — robustness coverage; the reliable F8 discriminator is
    // tb_packer_cadence). Spike held heavier to stress the 1-bit packer.
    reg bp_en;
    always @(posedge clk) begin
        thresh_mem_wait   <= bp_en ? ($urandom_range(99) < 30) : 1'b0;
        dcy_syn_mem_wait  <= bp_en ? ($urandom_range(99) < 30) : 1'b0;
        dcy_mem_mem_wait  <= bp_en ? ($urandom_range(99) < 30) : 1'b0;
        b_eff_mem_wait    <= bp_en ? ($urandom_range(99) < 30) : 1'b0;
        dcy_ada_mem_wait  <= bp_en ? ($urandom_range(99) < 30) : 1'b0;
        scl_ada_mem_wait  <= bp_en ? ($urandom_range(99) < 30) : 1'b0;
        syn_curr_mem_wait <= bp_en ? ($urandom_range(99) < 40) : 1'b0;
        pot_mem_wait      <= bp_en ? ($urandom_range(99) < 40) : 1'b0;
        ada_mem_wait      <= bp_en ? ($urandom_range(99) < 40) : 1'b0;
        spike_mem_wait    <= bp_en ? ($urandom_range(99) < 55) : 1'b0;
    end

    integer n;

    // Random Q0.32 decay value, biased to hit small/large/extreme corners.
    function [31:0] rand_q032;
        integer k;
        begin
            k = $urandom_range(0, 9);
            case (k)
                0: rand_q032 = 32'h0000_0000;   // 0.0
                1: rand_q032 = 32'hFFFF_FFFF;   // ~1.0
                2: rand_q032 = 32'h8000_0000;   // 0.5
                3: rand_q032 = 32'h0000_0001;   // tiny
                4: rand_q032 = 32'hFFFF_0000;   // near 1.0
                default: rand_q032 = {$urandom_range(0,16'hFFFF), $urandom_range(0,16'hFFFF)};
            endcase
        end
    endfunction

    // Run one layer of N neurons (all caches 32-bit: one element per word).
    //   ha     = has_ada
    //   clrpot = clear_pot       (golden is fed pot=0 to mirror DUT)
    //   bp     = inject read back-pressure on the read-only memories
    //   binpt  = bin_point_syn_curr value (0..30). syn_sram is loaded with
    //           in_syn[n] << binpt (memory scale); the golden is called with
    //           in_syn[n] (neuron scale); the writeback compares against
    //           g_syn[n] << binpt. Other inputs (pot, thresh, b_eff, ada) live
    //           at neuron scale on both sides — they are not shifted in the RTL.
    task run_block;
        input integer N;
        input         ha;
        input         clrpot;
        input         bp;
        input integer binpt;
        input [255:0] tag;
        integer wi;
        reg [31:0] ew;
        reg signed [31:0] gp_in_s;
        reg signed [31:0] expected_syn_wb;
        begin
            // init memories + inputs + golden
            `SRAM_CLEAR(syn_sram) `SRAM_CLEAR(thr_sram) `SRAM_CLEAR(pot_sram)
            `SRAM_CLEAR(spk_sram) `SRAM_CLEAR(dcy_syn_sram) `SRAM_CLEAR(dcy_mem_sram)
            `SRAM_CLEAR(ada_sram) `SRAM_CLEAR(b_eff_sram) `SRAM_CLEAR(dcy_ada_sram)
            `SRAM_CLEAR(scl_ada_sram)

            for (n = 0; n < N; n = n + 1) begin
                in_syn[n]  = $urandom_range(0,200) - 100;
                in_pot[n]  = $urandom_range(0,200) - 100;
                in_thr[n]  = $urandom_range(0,60) - 30;   // mix of + / - => spike mix
                in_dsyn[n] = rand_q032();
                in_dmem[n] = rand_q032();
                // ada inputs (used only when ha=1, but always randomised)
                in_ada[n]  = {$urandom_range(0,16'hFFFF), $urandom_range(0,16'hFFFF)};
                in_beff[n] = rand_q032();                  // b_eff signed Q0.32, >=0 in practice
                in_dada[n] = rand_q032();
                in_sada[n] = rand_q032();

                // populate SRAMs (golden is signed; SRAM stores raw 32-bit).
                // Every cache is 32-bit, so one neuron per word — read and
                // write-back never collide in the shared syn_curr/pot arrays.
                //
                // syn_curr lives in SRAM at memory scale (in_syn << binpt) so
                // that the RTL right-shift recovers the neuron-scale value the
                // golden was called with. Other inputs are unshifted.
                syn_sram[syn_curr_base + n]    = in_syn[n] <<< binpt;
                pot_sram[pot_base + n]         = in_pot[n];
                thr_sram[thresh_base + n]      = in_thr[n];
                dcy_syn_sram[dcy_syn_base + n] = in_dsyn[n];
                dcy_mem_sram[dcy_mem_base + n] = in_dmem[n];
                ada_sram[ada_base + n]         = in_ada[n];
                b_eff_sram[b_eff_base + n]     = in_beff[n];
                dcy_ada_sram[dcy_ada_base + n] = in_dada[n];
                scl_ada_sram[scl_ada_base + n] = in_sada[n];

                // golden: clear_pot mirrored by feeding pot=0.
                gp_in_s = clrpot ? 32'sd0 : in_pot[n];
                np_ref_fmi(in_syn[n], gp_in_s, in_thr[n],
                           in_dsyn[n], in_dmem[n], ha,
                           in_ada[n], in_dada[n], in_sada[n], in_beff[n],
                           g_spk[n], g_pot[n], g_syn[n], g_ada[n]);
            end

            last_neuron_idx = N - 1;
            sub_on_fire = 1'b0; clear_pot = clrpot; has_ada = ha;
            syn_curr_sz = SZ32; pot_sz = SZ32; bp_en = bp;
            bin_point_syn_curr = binpt[4:0];

            `VT_PULSE(start_new_block)
            `VT_WAIT_FINISH(neuron_proc_finished, 6000)
            // Drop back-pressure FIRST, then drain wait-free: neuron_proc_finished
            // leads the final packer writeback, which under write back-pressure can
            // still be stalled. Any corruption already happened during the run.
            bp_en = 0;
            repeat (60) @(posedge clk); #1;       // let packers drain (wait-free)

            // check syn_curr + pot write-backs (full 32-bit per neuron)
            // syn writeback is at memory scale (g_syn << binpt); pot stays at neuron scale.
            for (n = 0; n < N; n = n + 1) begin
                expected_syn_wb = g_syn[n] <<< binpt;
                check_eq($signed(syn_sram[syn_curr_base + n]), expected_syn_wb, {tag, " syn_wb"});
                check_eq($signed(pot_sram[pot_base + n]),      g_pot[n],         {tag, " pot_wb"});
            end
            // check packed spike words (1-bit, 32 neurons/word)
            for (wi = 0; wi*32 < N; wi = wi + 1) begin
                ew = 32'b0;
                for (n = 0; n < 32 && wi*32 + n < N; n = n + 1)
                    ew[n] = g_spk[wi*32 + n];
                check_eq_u(spk_sram[spike_base + wi], ew, {tag, " spike_word"});
            end
            // check ada write-back ONLY when has_ada=1; else ada mem untouched.
            for (n = 0; n < N; n = n + 1) begin
                if (ha)
                    check_eq_u(ada_sram[ada_base + n], g_ada[n], {tag, " ada_wb"});
                else
                    // has_ada=0 -> ada packer never writes; mem stays at init value.
                    check_eq_u(ada_sram[ada_base + n], in_ada[n], {tag, " ada_untouched"});
            end
        end
    endtask

    initial begin
        verif_errors=0; verif_checks=0;
        start_new_block=0; target_acc=0; buffer_info=0;
        src1=0; src2=0; src3=0; tgt=0; wrl=0;
        syn_curr_base=20; thresh_base=40; pot_base=50; spike_base=60;
        dcy_syn_base=70; dcy_mem_base=80; ada_base=90; b_eff_base=100;
        dcy_ada_base=110; scl_ada_base=120;
        syn_curr_sz=SZ32; pot_sz=SZ32;
        bin_point_syn_curr=0; sub_on_fire=0; clear_pot=0; has_ada=0; bp_en=0;
        syn_curr_mem_wait=0; pot_mem_wait=0; spike_mem_wait=0; ada_mem_wait=0;
        thresh_mem_wait=0; dcy_syn_mem_wait=0; dcy_mem_mem_wait=0;
        b_eff_mem_wait=0; dcy_ada_mem_wait=0; scl_ada_mem_wait=0;
        reset=1; repeat(2) @(posedge clk); #1; reset=0; @(posedge clk); #1;

        $display("=== tb_neuron_processing (fmiSnnAcc FMI) ===");
        void'($urandom(32'h4E20_F1A1));

        // ---- has_ada = 0 (plain LIF, 2-cycle FSM) --------------------------
        run_block(16, 1'b0, 1'b0, 1'b0, 0, "B1 N16 plain");
        run_block(33, 1'b0, 1'b0, 1'b0, 0, "B2 N33 plain two-spike-words");
        run_block(16, 1'b0, 1'b1, 1'b0, 0, "B3 plain clear_pot");
        run_block(20, 1'b0, 1'b0, 1'b1, 0, "B4 plain backpressure");
        run_block(32, 1'b0, 1'b0, 1'b1, 0, "B5 plain N32 boundary + bp");

        // ---- has_ada = 1 (adaptive, 3-cycle FSM) ---------------------------
        run_block(16, 1'b1, 1'b0, 1'b0, 0, "B6 N16 ada");
        run_block(33, 1'b1, 1'b0, 1'b0, 0, "B7 N33 ada two-spike-words");
        run_block(20, 1'b1, 1'b0, 1'b1, 0, "B8 ada backpressure");
        run_block(16, 1'b1, 1'b1, 1'b0, 0, "B9 ada clear_pot");
        run_block(32, 1'b1, 1'b0, 1'b1, 0, "B10 ada N32 boundary + bp");

        // ---- bin_point_syn_curr non-zero: verify the syn_curr scale shift --
        // Plain: small/medium/large bin_point with both signs of in_syn.
        run_block(16, 1'b0, 1'b0, 1'b0,  1, "BP1 plain binpt=1");
        run_block(20, 1'b0, 1'b0, 1'b0,  4, "BP2 plain binpt=4");
        run_block(33, 1'b0, 1'b0, 1'b0,  7, "BP3 plain binpt=7 (representative bp>0 shift)");
        run_block(20, 1'b0, 1'b0, 1'b1,  7, "BP4 plain binpt=7 + bp");
        run_block(16, 1'b0, 1'b1, 1'b0,  7, "BP5 plain binpt=7 clear_pot");
        // Adaptive with non-zero shift: ada_corr stays at neuron scale (no shift).
        run_block(16, 1'b1, 1'b0, 1'b0,  7, "BP6 ada binpt=7");
        run_block(33, 1'b1, 1'b0, 1'b1,  7, "BP7 ada binpt=7 + bp");

        // ---- constrained-random loop: alternate has_ada + bp each pass -----
        for (n = 0; n < 16; n = n + 1) begin : rnd_loop
            reg ha_r, bp_r;
            integer nn, binpt_r;
            ha_r    = $urandom_range(0,1);
            bp_r    = $urandom_range(0,1);
            binpt_r = $urandom_range(0,8);  // random shift up to 8 bits
            nn      = $urandom_range(16,33);   // cross the 32-neuron boundary
            run_block(nn, ha_r, 1'b0, bp_r, binpt_r, "BR rand");
        end

        `VERIF_EPILOGUE("tb_neuron_processing")
    end

    `VERIF_WATCHDOG(20000000)

endmodule
