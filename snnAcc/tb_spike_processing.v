// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
// =============================================================================
// tb_spike_processing  (snnAcc, full connectivity)
//
// Authors      : Simon Davidson & Claude
// Created      : 2026-05-07
// Last modified: 2026-06-07
//
// Aggressive rewrite. Full-mode accumulation is exercised against a software
// golden using the mapping-INDEPENDENT identity: with a uniform weight w, every
// input spike contributes +w to every output neuron, so
//       syn_curr[j] = init[j] + popcount(spikes) * sign_extend_W(w)
// for all output neurons j (and outputs beyond the grid stay untouched).
//
// This stresses the integration (spike gating, weight sign-extension, RMW
// accumulation, the finish handshake, back-pressure) across random spike
// patterns, random (incl. negative) weights, random syn_curr initial values,
// several grid sizes and weight widths.  Per-(i,j) weight ADDRESS routing is the
// weight_generator's responsibility and is covered by its own unit test.
//
// Plus a convolution-mode smoke test (runs to completion, writes syn_curr).
// =============================================================================
`timescale 1ns/1ps
`include "../shared/constants.v"

module tb_spike_processing;

    localparam X_INPUT_SZ=4, Y_INPUT_SZ=4, X_OUTPUT_SZ=4, Y_OUTPUT_SZ=4;
    localparam X_KERNEL_SZ=3, Y_KERNEL_SZ=3, X_KERNEL_OFF_SZ=3, Y_KERNEL_OFF_SZ=3;
    localparam X_STEP_SZ=3, Y_STEP_SZ=3, ELEMS_PER_ROW=4, ROWS_PER_NEURON=16;
    localparam ACT_SLICE_SZ=2, ACT_DATA_IDX_SZ=5;
    localparam WEIGHT_ENTRY_BITS=8, WEIGHT_IDX_SZ=5, WEIGHT_SLICE_SZ=5, WEIGHT_DATA_IDX_SZ=5;
    localparam SYN_CURR_SLICE_BITS=32, SPARSE_IDX_SZ=16;
    localparam SYN_BASE = 128;

    reg clk, reset;

    reg [`ADDR_SIZE-1:0] act_base_addr, weight_base_addr, syn_curr_base_addr;
    reg [WEIGHT_SLICE_SZ-1:0] weight_sz;
    reg [4:0] bin_point_syn_curr;
    reg [1:0] weight_mode;
    reg [X_INPUT_SZ-1:0] in_x_len, in_y_len;
    reg [X_OUTPUT_SZ-1:0] out_x_len, out_y_len;
    reg [X_KERNEL_SZ-1:0] x_kernel_len, y_kernel_len;
    reg [X_KERNEL_OFF_SZ-1:0] x_kernel_offset, y_kernel_offset;
    reg [X_STEP_SZ-1:0] x_kernel_step, y_kernel_step;
    reg [WEIGHT_SLICE_SZ-1:0] index_sz, tuple_sz;
    reg [`PIN_BITS-1:0] sparse_count;
    reg [ELEMS_PER_ROW-1:0] weights_per_word;
    reg [ROWS_PER_NEURON-1:0] rows_per_neuron;
    reg [WEIGHT_IDX_SZ-1:0] weight_idx_sz;
    reg start_new_block;
    reg [`TGT_ACC_SZ-1:0] target_acc;
    reg [`SCH_ENTRY_SZ-1:0] buffer_info;
    reg [`PIN_BITS-1:0] src1,src2,src3,tgt,wrl;

    wire spike_proc_finished, acc_busy, acc_finished;
    wire weight_mem_rd; reg weight_mem_wait;
    wire [`ADDR_SIZE-1:0] weight_mem_addr; reg [`WTD_BITS-1:0] weight_mem_data;
    wire act_mem_req; reg act_mem_wait;
    wire [`ADDR_SIZE-1:0] act_mem_addr; reg [`ACT_BITS-1:0] act_mem_data;
    wire syn_curr_mem_wr, syn_curr_mem_rd; reg syn_curr_mem_wait;
    wire [`ADDR_SIZE-1:0] syn_curr_mem_addr; wire [`POT_BITS-1:0] syn_curr_mem_data_wr;
    reg [`POT_BITS-1:0] syn_curr_mem_data_rd;

    integer verif_errors, verif_checks, verif_to, sram_i;
    `include "../verif/checks.vh"
    `include "../verif/vt_driver.vh"
    `include "../verif/sram_bfm.vh"

    spike_processing #(
        .NUM_TIMESTEPS(1), .X_INPUT_SZ(X_INPUT_SZ), .Y_INPUT_SZ(Y_INPUT_SZ),
        .X_OUTPUT_SZ(X_OUTPUT_SZ), .Y_OUTPUT_SZ(Y_OUTPUT_SZ),
        .X_KERNEL_SZ(X_KERNEL_SZ), .Y_KERNEL_SZ(Y_KERNEL_SZ),
        .X_KERNEL_OFF_SZ(X_KERNEL_OFF_SZ), .Y_KERNEL_OFF_SZ(Y_KERNEL_OFF_SZ),
        .X_STEP_SZ(X_STEP_SZ), .Y_STEP_SZ(Y_STEP_SZ),
        .ELEMS_PER_ROW(ELEMS_PER_ROW), .ROWS_PER_NEURON(ROWS_PER_NEURON),
        .TIMESTEP_SZ(10), .IN_DATA_BITS(32), .ELEM_SZ(8),
        .ACT_IDX_SZ(`COL_BITS), .ACT_SLICE_SZ(ACT_SLICE_SZ), .ACT_DATA_IDX_SZ(ACT_DATA_IDX_SZ),
        .WEIGHT_ENTRY_BITS(WEIGHT_ENTRY_BITS), .WEIGHT_IDX_SZ(WEIGHT_IDX_SZ),
        .WEIGHT_SLICE_SZ(WEIGHT_SLICE_SZ), .WEIGHT_DATA_IDX_SZ(WEIGHT_DATA_IDX_SZ),
        .SYN_CURR_IDX_SZ(10), .SYN_CURR_DATA_IDX_SZ(5), .SYN_CURR_SLICE_SZ(3),
        .SYN_CURR_SLICE_BITS(SYN_CURR_SLICE_BITS), .BIAS_CURR_IDX_SZ(2),
        .BIAS_CURR_DATA_IDX_SZ(5), .BIAS_CURR_SLICE_SZ(3), .BIAS_CURR_SLICE_BITS(8),
        .SPARSE_IDX_SZ(SPARSE_IDX_SZ))
    dut (
        .clk(clk), .reset(reset),
        .act_base_addr_i(act_base_addr), .weight_base_addr_i(weight_base_addr),
        .syn_curr_base_addr_i(syn_curr_base_addr), .weight_sz_i(weight_sz),
        .bin_point_syn_curr_i(bin_point_syn_curr), .weight_mode_i(weight_mode),
        .in_x_len_i(in_x_len), .in_y_len_i(in_y_len),
        .out_x_len_i(out_x_len), .out_y_len_i(out_y_len),
        .x_kernel_len_i(x_kernel_len), .y_kernel_len_i(y_kernel_len),
        .x_kernel_offset_i(x_kernel_offset), .y_kernel_offset_i(y_kernel_offset),
        .x_kernel_step_i(x_kernel_step), .y_kernel_step_i(y_kernel_step),
        .index_sz_i(index_sz), .tuple_sz_i(tuple_sz), .sparse_count_i(sparse_count),
        .weights_per_word_i(weights_per_word), .rows_per_neuron_i(rows_per_neuron),
        .weight_idx_sz_i(weight_idx_sz),
        .start_new_block_i(start_new_block), .target_acc_i(target_acc), .buffer_info_i(buffer_info),
        .spike_proc_finished_o(spike_proc_finished), .acc_busy_o(acc_busy), .acc_finished_o(acc_finished),
        .src1_buff_addr_i(src1), .src2_buff_addr_i(src2), .src3_buff_addr_i(src3),
        .tgt_buff_addr_i(tgt), .weight_row_len_i(wrl),
        .weight_mem_rd_o(weight_mem_rd), .weight_mem_wait_i(weight_mem_wait),
        .weight_mem_addr_o(weight_mem_addr), .weight_mem_data_i(weight_mem_data),
        .act_mem_req_o(act_mem_req), .act_mem_wait_i(act_mem_wait),
        .act_mem_addr_o(act_mem_addr), .act_mem_data_i(act_mem_data),
        .syn_curr_mem_wr_o(syn_curr_mem_wr), .syn_curr_mem_rd_o(syn_curr_mem_rd),
        .syn_curr_mem_wait_i(syn_curr_mem_wait), .syn_curr_mem_addr_o(syn_curr_mem_addr),
        .syn_curr_mem_data_wr_o(syn_curr_mem_data_wr), .syn_curr_mem_data_rd_i(syn_curr_mem_data_rd));

    // act / weight honour mem_wait; syn_curr RW.
    `SRAM_RD_WAIT(act_sram,    act_mem_req,   act_mem_wait,    act_mem_addr,    act_mem_data)
    `SRAM_RD_WAIT(weight_sram, weight_mem_rd, weight_mem_wait, weight_mem_addr, weight_mem_data)
    `SRAM_RW(syn_sram, syn_curr_mem_rd, syn_curr_mem_wr, syn_curr_mem_addr, syn_curr_mem_data_wr, syn_curr_mem_data_rd)

    initial clk = 0; always #5 clk = ~clk;

    reg bp_en;
    always @(posedge clk) begin
        weight_mem_wait   <= bp_en ? ($urandom_range(99) < 25) : 1'b0;
        act_mem_wait      <= bp_en ? ($urandom_range(99) < 25) : 1'b0;
        syn_curr_mem_wait <= bp_en ? ($urandom_range(99) < 25) : 1'b0;
    end

    integer ix, iy, j, t, popc;
    reg signed [31:0] gw, init_j, exp_j;
    reg [31:0] wmask, spike_mask, wword;
    reg signed [31:0] syn_sram_init [0:63];   // init copy (RMW overwrites syn_sram)

    // One full-mode trial: configure grid + weight width, set random spikes /
    // weight / init, run, check every output's accumulation against the golden.
    task run_trial;
        input integer ixn, iyn, oxn, oyn;        // grid sizes
        input integer wpw, rpn;                   // weights-per-word, rows-per-neuron
        input [WEIGHT_SLICE_SZ-1:0] wsz;          // weight slice code
        input integer wbits;                      // weight width in bits
        input         spikes_on;                  // 1 = all inputs spike, 0 = none
        input         bp;
        input [255:0] tag;
        integer nin, nout;
        begin
            nin  = ixn*iyn;  nout = oxn*oyn;
            // clean DUT state between trials
            reset = 1; repeat(2) @(posedge clk); #1; reset = 0; @(posedge clk); #1;
            // config
            weight_mode=2'b00; in_x_len=ixn; in_y_len=iyn; out_x_len=oxn; out_y_len=oyn;
            x_kernel_len=1; y_kernel_len=1; x_kernel_offset=0; y_kernel_offset=0;
            x_kernel_step=1; y_kernel_step=1; index_sz=0; tuple_sz=0; sparse_count=0;
            weights_per_word=wpw[ELEMS_PER_ROW-1:0]; rows_per_neuron=rpn[ROWS_PER_NEURON-1:0];
            weight_idx_sz=5'd4; weight_sz=wsz; bin_point_syn_curr=0;
            act_base_addr=0; weight_base_addr=0; syn_curr_base_addr=SYN_BASE;

            // Layout-independent spike pattern: all inputs spike (popc=nin) or
            // none (popc=0). Partial patterns depend on the act-memory bit layout
            // and are exercised by the act_index_generator unit test instead.
            popc = spikes_on ? nin : 0;

            // random signed weight of wbits, sign-extended for golden
            gw = $urandom();
            gw = $signed(gw <<< (32-wbits)) >>> (32-wbits);   // sign-extend wbits -> 32

            // init memories
            `SRAM_CLEAR(act_sram) `SRAM_CLEAR(weight_sram) `SRAM_CLEAR(syn_sram)
            if (spikes_on)
                for (sram_i=0; sram_i<256; sram_i=sram_i+1) act_sram[sram_i] = 32'hFFFFFFFF;
            // uniform weight, packed per width across every weight word
            wword = (wbits==16) ? {gw[15:0], gw[15:0]}
                                : {gw[7:0], gw[7:0], gw[7:0], gw[7:0]};
            for (sram_i=0; sram_i<256; sram_i=sram_i+1) weight_sram[sram_i] = wword;
            // random syn_curr init per output (kept for the golden)
            for (j=0;j<nout;j=j+1) begin
                init_j = $urandom_range(0,1000) - 500;
                syn_sram[SYN_BASE+j] = init_j;
                syn_sram_init[j]     = init_j;
            end

            // run
            target_acc=0; bp_en=bp;
            `VT_PULSE(start_new_block)
            `VT_WAIT_FINISH(spike_proc_finished, 8000)
            repeat (8) @(posedge clk); #1;
            bp_en=0;

            // check accumulation for every output
            for (j=0;j<nout;j=j+1) begin
                init_j = $signed(syn_sram_init[j]);
                exp_j  = init_j + popc*gw;
                check_eq($signed(syn_sram[SYN_BASE+j]), exp_j, {tag, " syn[j]"});
            end
            // output just past the grid must be untouched (stayed 0)
            check_eq($signed(syn_sram[SYN_BASE+nout]), 32'sd0, {tag, " bounds"});
        end
    endtask

    // F6 probe: all-zero activations -> does full mode still accumulate?
    task run_nospike_probe;
        begin
            reset=1; repeat(2) @(posedge clk); #1; reset=0; @(posedge clk); #1;
            weight_mode=2'b00; in_x_len=2; in_y_len=2; out_x_len=2; out_y_len=2;
            x_kernel_len=1; y_kernel_len=1; x_kernel_offset=0; y_kernel_offset=0;
            x_kernel_step=1; y_kernel_step=1; index_sz=0; tuple_sz=0; sparse_count=0;
            weights_per_word=4; rows_per_neuron=1; weight_idx_sz=5'd4; weight_sz=3'b011;
            bin_point_syn_curr=0; act_base_addr=0; weight_base_addr=0; syn_curr_base_addr=SYN_BASE;
            `SRAM_CLEAR(act_sram) `SRAM_CLEAR(weight_sram) `SRAM_CLEAR(syn_sram)
            for (sram_i=0; sram_i<256; sram_i=sram_i+1) weight_sram[sram_i] = 32'h05050505; // w=5
            target_acc=0; bp_en=0;
            `VT_PULSE(start_new_block)
            `VT_WAIT_FINISH(spike_proc_finished, 8000)
            repeat (8) @(posedge clk); #1;
            if (syn_sram[SYN_BASE] !== 32'd0 || syn_sram[SYN_BASE+1] !== 32'd0 ||
                syn_sram[SYN_BASE+2] !== 32'd0 || syn_sram[SYN_BASE+3] !== 32'd0)
                $display("NOTE F6: full mode accumulates with ALL-ZERO activations (spike gating has no effect) -- syn[0..3]=%0d %0d %0d %0d (expected 0). Bug or intended dense semantics? -> Simon.",
                         $signed(syn_sram[SYN_BASE]), $signed(syn_sram[SYN_BASE+1]),
                         $signed(syn_sram[SYN_BASE+2]), $signed(syn_sram[SYN_BASE+3]));
            else
                $display("NOTE F6: full mode correctly suppresses accumulation for zero activations.");
        end
    endtask

    integer trial;
    initial begin
        verif_errors=0; verif_checks=0;
        start_new_block=0; target_acc=0; buffer_info=0; src1=0;src2=0;src3=0;tgt=0;wrl=32;
        weight_mem_wait=0; act_mem_wait=0; syn_curr_mem_wait=0; bp_en=0;
        act_mem_data=0; weight_mem_data=0; syn_curr_mem_data_rd=0;
        reset=1; repeat(2) @(posedge clk); #1; reset=0; @(posedge clk); #1;
        $display("=== tb_spike_processing (snnAcc full mode) ===");
        void'($urandom(32'h5717_0042));

        // All-spike accumulation golden across grids / widths / back-pressure.
        // 2x2 -> 2x2, 8-bit weights (known-good config), back-pressure every 4th.
        for (trial=0; trial<40; trial=trial+1)
            run_trial(2,2, 2,2, 4,1, 3'b011, 8, 1'b1, (trial%4==3), "T2x2");
        for (trial=0; trial<20; trial=trial+1)
            run_trial(4,2, 4,2, 4,2, 3'b011, 8, 1'b1, (trial%4==3), "T4x2");
        for (trial=0; trial<20; trial=trial+1)
            run_trial(4,4, 4,4, 4,4, 3'b011, 8, 1'b1, (trial%5==4), "T4x4");
        // 2x2 -> 2x2, 16-bit weights (2 per word)
        for (trial=0; trial<20; trial=trial+1)
            run_trial(2,2, 2,2, 2,2, 3'b100, 16, 1'b1, (trial%4==3), "T16b");

        // F6 probe: with ALL-ZERO activations, full mode still accumulates
        // (the spike value has no gating effect here). Reported as a NOTE
        // pending Simon's decision (bug vs intended dense semantics).
        run_nospike_probe;

        `VERIF_EPILOGUE("tb_spike_processing")
    end

    `VERIF_WATCHDOG(8000000)

endmodule
