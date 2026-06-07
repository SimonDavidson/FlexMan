// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
// =============================================================================
// tb_neuron_processing  (ipSnnAcc, LIF)
//
// Authors      : Simon Davidson & Claude
// Created      : 2026-05-08
// Last modified: 2026-06-07
//
// Aggressive rewrite. Drives whole layers of neurons through the neuron loop and
// checks every memory write-back (decayed syn_curr, decayed potential, packed
// spike word) against the np_ref_lif golden. Covers:
//   * many neurons, random data, mixed spike / no-spike
//   * the 32-neuron spike-word boundary (N>32 -> two packed spike words)
//   * clear_pot and sub_on_fire modes
//   * read-path back-pressure (random bias/thresh mem_wait): result must be
//     latency-invariant
//
// 32-bit slices throughout (1 element per word); exhaustive slice-size packing
// is covered separately by the dedicated packer test (Phase 2B).
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
    reg [`ADDR_SIZE-1:0]    syn_curr_base, bias_base, thresh_base, pot_base, spike_base;
    reg [2:0]               syn_curr_sz, bias_curr_sz, pot_sz;
    reg [4:0]               bin_point_syn_curr;
    reg [31:0]              syn_curr_decay_mult, pot_decay_mult;
    reg                     sub_on_fire, clear_pot;

    // Scheduler iface
    reg                     start_new_block;
    reg [`TGT_ACC_SZ-1:0]   target_acc;
    reg [`SCH_ENTRY_SZ-1:0] buffer_info;
    reg [`PIN_BITS-1:0]     src1, src2, src3, tgt, wrl;

    wire neuron_proc_finished, acc_busy, acc_finished;

    // syn_curr (rd+wr)
    wire                 syn_curr_mem_wr, syn_curr_mem_rd;
    reg                  syn_curr_mem_wait;
    wire [`ADDR_SIZE-1:0] syn_curr_mem_addr;
    wire [`POT_BITS-1:0]  syn_curr_mem_data_wr;
    reg  [`POT_BITS-1:0]  syn_curr_mem_data_rd;
    // bias (rd)
    wire bias_curr_mem_rd; reg bias_curr_mem_wait;
    wire [`ADDR_SIZE-1:0] bias_curr_mem_addr; reg [`WTD_BITS-1:0] bias_curr_mem_data;
    // thresh (rd)
    wire thresh_mem_rd; reg thresh_mem_wait;
    wire [`ADDR_SIZE-1:0] thresh_mem_addr; reg [`WTD_BITS-1:0] thresh_mem_data;
    // pot (rd+wr)
    wire pot_mem_wr, pot_mem_rd; reg pot_mem_wait;
    wire [`ADDR_SIZE-1:0] pot_mem_addr; wire [`POT_BITS-1:0] pot_mem_data_wr; reg [`POT_BITS-1:0] pot_mem_data_rd;
    // spike (wr)
    wire spike_mem_wr; reg spike_mem_wait;
    wire [`ADDR_SIZE-1:0] spike_mem_addr; wire [`ACT_BITS-1:0] spike_mem_data;

    integer verif_errors, verif_checks, verif_to, sram_i;
    `include "../verif/checks.vh"
    `include "../verif/np_ref.vh"
    `include "../verif/vt_driver.vh"
    `include "../verif/sram_bfm.vh"

    neuron_processing #(
        .NEURON_IDX_SZ(NEURON_IDX_SZ), .SYN_CURR_IDX_SZ(10), .SYN_CURR_DATA_IDX_SZ(5),
        .SYN_CURR_SLICE_SZ(3), .SYN_CURR_SLICE_BITS(SLICE_BITS),
        .BIAS_CURR_IDX_SZ(7), .BIAS_CURR_DATA_IDX_SZ(5), .BIAS_CURR_SLICE_SZ(3), .BIAS_CURR_SLICE_BITS(SLICE_BITS),
        .POT_IDX_SZ(7), .POT_DATA_IDX_SZ(5), .POT_SLICE_SZ(3), .POT_SLICE_BITS(SLICE_BITS),
        .SPIKE_IDX_SZ(7), .SPIKE_DATA_IDX_SZ(5), .SPIKE_SLICE_SZ(3), .SPIKE_SLICE_BITS(SLICE_BITS))
    dut (
        .clk(clk), .reset(reset),
        .last_neuron_idx_i(last_neuron_idx),
        .syn_curr_base_addr_i(syn_curr_base), .bias_curr_base_addr_i(bias_base),
        .thresh_base_addr_i(thresh_base), .pot_base_addr_i(pot_base), .spike_base_addr_i(spike_base),
        .syn_curr_sz_i(syn_curr_sz), .bias_curr_sz_i(bias_curr_sz), .pot_sz_i(pot_sz),
        .bin_point_syn_curr_i(bin_point_syn_curr),
        .syn_curr_decay_mult_i(syn_curr_decay_mult), .pot_decay_mult_i(pot_decay_mult),
        .sub_on_fire_i(sub_on_fire), .clear_pot_i(clear_pot),
        .start_new_block_i(start_new_block), .target_acc_i(target_acc), .buffer_info_i(buffer_info),
        .neuron_proc_finished_o(neuron_proc_finished), .acc_busy_o(acc_busy), .acc_finished_o(acc_finished),
        .src1_buff_addr_i(src1), .src2_buff_addr_i(src2), .src3_buff_addr_i(src3),
        .tgt_buff_addr_i(tgt), .weight_row_len_i(wrl),
        .syn_curr_mem_wr_o(syn_curr_mem_wr), .syn_curr_mem_rd_o(syn_curr_mem_rd),
        .syn_curr_mem_wait_i(syn_curr_mem_wait), .syn_curr_mem_addr_o(syn_curr_mem_addr),
        .syn_curr_mem_data_o(syn_curr_mem_data_wr), .syn_curr_mem_data_i(syn_curr_mem_data_rd),
        .bias_curr_mem_rd_o(bias_curr_mem_rd), .bias_curr_mem_wait_i(bias_curr_mem_wait),
        .bias_curr_mem_addr_o(bias_curr_mem_addr), .bias_curr_mem_data_i(bias_curr_mem_data),
        .thresh_mem_rd_o(thresh_mem_rd), .thresh_mem_wait_i(thresh_mem_wait),
        .thresh_mem_addr_o(thresh_mem_addr), .thresh_mem_data_i(thresh_mem_data),
        .pot_mem_wr_o(pot_mem_wr), .pot_mem_rd_o(pot_mem_rd), .pot_mem_wait_i(pot_mem_wait),
        .pot_mem_addr_o(pot_mem_addr), .pot_mem_data_o(pot_mem_data_wr), .pot_mem_data_i(pot_mem_data_rd),
        .spike_mem_wr_o(spike_mem_wr), .spike_mem_wait_i(spike_mem_wait),
        .spike_mem_addr_o(spike_mem_addr), .spike_mem_data_o(spike_mem_data));

    // SRAM models (256x32, [7:0] truncation). bias/thresh honour mem_wait.
    `SRAM_RW (syn_sram, syn_curr_mem_rd, syn_curr_mem_wr, syn_curr_mem_addr, syn_curr_mem_data_wr, syn_curr_mem_data_rd)
    `SRAM_RD_WAIT(bias_sram, bias_curr_mem_rd, bias_curr_mem_wait, bias_curr_mem_addr, bias_curr_mem_data)
    `SRAM_RD_WAIT(thr_sram,  thresh_mem_rd,    thresh_mem_wait,    thresh_mem_addr,    thresh_mem_data)
    `SRAM_RW (pot_sram, pot_mem_rd, pot_mem_wr, pot_mem_addr, pot_mem_data_wr, pot_mem_data_rd)
    `SRAM_WR (spk_sram, spike_mem_wr, spike_mem_addr, spike_mem_data)

    initial clk = 0; always #5 clk = ~clk;

    // golden model arrays
    reg signed [31:0] g_syn [0:127];
    reg signed [31:0] g_pot [0:127];
    reg               g_spk [0:127];
    reg signed [31:0] in_syn[0:127], in_bias[0:127], in_thr[0:127], in_pot[0:127];

    // back-pressure driver
    reg bp_en;
    always @(posedge clk) begin
        bias_curr_mem_wait <= bp_en ? ($urandom_range(99) < 30) : 1'b0;
        thresh_mem_wait    <= bp_en ? ($urandom_range(99) < 30) : 1'b0;
    end

    integer n;
    reg [31:0] exp_spk_word;

    // Run one layer of N neurons; small signed inputs => good spike mix.
    task run_block;
        input integer N;
        input         sof;
        input         clrpot;
        input         bp;
        input [255:0] tag;
        integer wi, base_n;
        reg [31:0] ew;
        begin
            // init memories + inputs + golden
            `SRAM_CLEAR(syn_sram) `SRAM_CLEAR(bias_sram) `SRAM_CLEAR(thr_sram)
            `SRAM_CLEAR(pot_sram) `SRAM_CLEAR(spk_sram)
            for (n = 0; n < N; n = n + 1) begin
                in_syn[n]  = $urandom_range(0,120) - 60;
                in_bias[n] = $urandom_range(0,40)  - 20;
                in_thr[n]  = $urandom_range(0,100);
                in_pot[n]  = $urandom_range(0,120) - 60;
                syn_sram[n]  = in_syn[n];
                bias_sram[n] = in_bias[n];
                thr_sram[n]  = in_thr[n];
                pot_sram[n]  = in_pot[n];
                np_ref_lif(in_syn[n], clrpot ? 32'sd0 : in_pot[n], in_bias[n], in_thr[n],
                           syn_curr_decay_mult, pot_decay_mult, sof,
                           g_spk[n], g_pot[n], g_syn[n]);
            end

            last_neuron_idx = N - 1;
            sub_on_fire = sof; clear_pot = clrpot; bp_en = bp;

            `VT_PULSE(start_new_block)
            `VT_WAIT_FINISH(neuron_proc_finished, 4000)
            repeat (30) @(posedge clk); #1;       // let packers drain
            bp_en = 0;

            // check writebacks
            for (n = 0; n < N; n = n + 1) begin
                check_eq($signed(syn_sram[n]), g_syn[n], {tag, " syn_wb"});
                check_eq($signed(pot_sram[n]), g_pot[n], {tag, " pot_wb"});
            end
            // check packed spike words
            for (wi = 0; wi*32 < N; wi = wi + 1) begin
                ew = 32'b0;
                for (n = 0; n < 32 && wi*32 + n < N; n = n + 1)
                    ew[n] = g_spk[wi*32 + n];
                check_eq_u(spk_sram[wi], ew, {tag, " spike_word"});
            end
        end
    endtask

    initial begin
        verif_errors=0; verif_checks=0;
        start_new_block=0; target_acc=0; buffer_info=0;
        src1=0; src2=0; src3=0; tgt=0; wrl=0;
        syn_curr_base=0; bias_base=0; thresh_base=0; pot_base=0; spike_base=0;
        syn_curr_sz=SZ32; bias_curr_sz=SZ32; pot_sz=SZ32;
        bin_point_syn_curr=0; sub_on_fire=0; clear_pot=0; bp_en=0;
        syn_curr_mem_wait=0; pot_mem_wait=0; spike_mem_wait=0;
        bias_curr_mem_wait=0; thresh_mem_wait=0;
        syn_curr_decay_mult=32'h8000_0000; pot_decay_mult=32'h8000_0000;  // 0.5
        reset=1; repeat(2) @(posedge clk); #1; reset=0; @(posedge clk); #1;

        $display("=== tb_neuron_processing (ipSnnAcc LIF) ===");
        void'($urandom(32'h4E20_0007));

        run_block(16, 1'b0, 1'b0, 1'b0, "B1 N16");
        run_block(33, 1'b0, 1'b0, 1'b0, "B2 N33 two-spike-words");
        run_block(16, 1'b0, 1'b1, 1'b0, "B3 clear_pot");
        run_block(16, 1'b1, 1'b0, 1'b0, "B4 sub_on_fire");
        run_block(20, 1'b0, 1'b0, 1'b1, "B5 backpressure");
        // vary decay rates
        syn_curr_decay_mult = 32'hC000_0000; pot_decay_mult = 32'h4000_0000;
        run_block(24, 1'b0, 1'b0, 1'b0, "B6 decay 0.75/0.25");
        syn_curr_decay_mult = 32'hFFFF_FFFF; pot_decay_mult = 32'h0000_0000;
        run_block(12, 1'b1, 1'b0, 1'b1, "B7 decay extremes + sof + bp");

        `VERIF_EPILOGUE("tb_neuron_processing")
    end

    `VERIF_WATCHDOG(5000000)

endmodule
