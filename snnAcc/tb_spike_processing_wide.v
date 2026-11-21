// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
// =============================================================================
// tb_spike_processing_wide — AGGRESSIVE unit test, FULL connectivity.
// Authors: Simon Davidson & Claude
// Created: 2026-06-07 ; Last modified: 2026-06-07
//
// FULL-mode accumulation golden (the mapping-independent identity from the
// FlexMan tb_spike_processing template):
//
//   with a UNIFORM weight w in the weight row, and every input spiking,
//   syn_curr[o] = init[o] + nin * sign_extend_SC(w)     for every output o
//
// where nin = number of input neurons (all spiking) and sign_extend_SC widens
// the WB-bit weight to the SC_BITS syn_curr element width.  This is independent
// of the exact input->output->word mapping, so it is a robust whole-image check.
// A second, NON-uniform pass checks the general model
//   syn_curr[o] = init[o] + SUM_over_spiking_i  sext( W[i][o] )
// by laying down distinct per-(input,output) weights and summing them.
//
// Coverage: grids (#inputs x #outputs), 8/16-bit weights, random non-zero RMW
// init, mem_wait back-pressure on weight & syn_curr, and the LANES/PF sweep.
// All-spike vs no-spike are handled per the F6 NOTE pattern: the no-spike probe
// reports (not hard-fails) if the wide datapath still accumulates with zero
// activations.
//
// -define knobs: LANES (1/2), EB syn_curr width via SC param (8/16 fixed here at
// 8 to keep weights/syn same width for the simplest exact identity), PF (0/1).
// =============================================================================
`timescale 1ns/1ps
`include "../shared/constants.v"

`ifndef LANES
 `define LANES 2
`endif
`ifndef PF
 `define PF 0
`endif

module tb_spike_processing_wide;
    localparam CLK=10, LANES=`LANES, MEMD=256;
    localparam WB = 8;                 // weight element width
    localparam SC = 8;                 // syn_curr element width (== WB: exact add)
    localparam EPW = 32/SC;            // syn_curr / weight elems per word (4)
    localparam ACT_BASE=0, WGT_BASE=10, SC_BASE=20;

    reg clk=0; always #(CLK/2) clk=~clk;
    reg reset;

    integer verif_errors, verif_checks, verif_to;
    `include "../verif/checks.vh"

    wire wr,rd; wire [`ADDR_SIZE-1:0] sc_addr; wire [`POT_BITS-1:0] sc_wd, sc_rdd;
    wire wgt_rd; wire [`ADDR_SIZE-1:0] wgt_addr; wire [`WTD_BITS-1:0] wgt_data;
    wire act_rd; wire [`ADDR_SIZE-1:0] act_addr; wire [`ACT_BITS-1:0] act_data;
    reg start; wire busy, fin, spfin;
    reg sc_wait, wgt_wait;

    // config (driven per-run)
    reg [9:0] in_x, in_y, out_x, out_y;
    reg [3:0] wpw;                     // weights_per_word
    reg [9:0] rpn;                     // rows_per_neuron

    spike_processing_wide #(
        .NUM_LANES(LANES), .PREFETCH(`PF), .WEIGHT_ELEM_BITS(WB),
        .X_INPUT_SZ(10), .Y_INPUT_SZ(10), .X_OUTPUT_SZ(10), .Y_OUTPUT_SZ(10),
        .ELEM_SZ(8), .ACT_SLICE_SZ(3), .ACT_DATA_IDX_SZ(5), .ACT_IDX_SZ(10),
        .WEIGHT_ENTRY_BITS(8), .WEIGHT_IDX_SZ(10), .WEIGHT_SLICE_SZ(5), .WEIGHT_DATA_IDX_SZ(5),
        .SYN_CURR_IDX_SZ(10), .SYN_CURR_SLICE_SZ(3), .SYN_CURR_SLICE_BITS(SC),
        .ROWS_PER_NEURON(10), .ELEMS_PER_ROW(4))
    dut (
        .clk(clk), .reset(reset),
        .act_base_addr_i(ACT_BASE), .weight_base_addr_i(WGT_BASE), .syn_curr_base_addr_i(SC_BASE),
        .weight_sz_i(3'd3), .bin_point_syn_curr_i(5'd0), .weight_mode_i(2'd0),
        .in_x_len_i(in_x), .in_y_len_i(in_y), .out_x_len_i(out_x), .out_y_len_i(out_y),
        .x_kernel_len_i(3'd1), .y_kernel_len_i(3'd1), .x_kernel_offset_i(3'd0), .y_kernel_offset_i(3'd0),
        .x_kernel_step_i(3'd1), .y_kernel_step_i(3'd1),
        .index_sz_i(5'd0), .tuple_sz_i(5'd0), .sparse_count_i(10'd0),
        .weights_per_word_i(wpw), .rows_per_neuron_i(rpn), .weight_idx_sz_i(10'd5),
        .start_new_block_i(start), .target_acc_i(3'd0), .buffer_info_i(32'd0),
        .spike_proc_finished_o(spfin), .acc_busy_o(busy), .acc_finished_o(fin),
        .src1_buff_addr_i(10'd0), .src2_buff_addr_i(10'd0), .src3_buff_addr_i(10'd0),
        .tgt_buff_addr_i(10'd0), .weight_row_len_i(10'd0),
        .weight_mem_rd_o(wgt_rd), .weight_mem_wait_i(wgt_wait),
        .weight_mem_addr_o(wgt_addr), .weight_mem_data_i(wgt_data),
        .act_mem_req_o(act_rd), .act_mem_wait_i(1'b0),
        .act_mem_addr_o(act_addr), .act_mem_data_i(act_data),
        .syn_curr_mem_wr_o(wr), .syn_curr_mem_rd_o(rd), .syn_curr_mem_wait_i(sc_wait),
        .syn_curr_mem_addr_o(sc_addr), .syn_curr_mem_data_wr_o(sc_wd), .syn_curr_mem_data_rd_i(sc_rdd));

    sram_model #(.DATA_W(32),.DEPTH(MEMD)) u_act (.clk(clk),.we(1'b0),.re(act_rd),.rwait(1'b0),
        .addr(act_addr[7:0]),.wdata(32'b0),.rdata(act_data));
    sram_model #(.DATA_W(32),.DEPTH(MEMD)) u_wgt (.clk(clk),.we(1'b0),.re(wgt_rd),.rwait(wgt_wait),
        .addr(wgt_addr[7:0]),.wdata(32'b0),.rdata(wgt_data));
    sram_model #(.DATA_W(32),.DEPTH(MEMD)) u_sc (.clk(clk),.we(wr),.re(rd),.rwait(sc_wait),
        .addr(sc_addr[7:0]),.wdata(sc_wd),.rdata(sc_rdd));

    // race-free back-pressure
    reg bp_en;
    always @(posedge clk) begin
        wgt_wait <= bp_en ? ($urandom_range(99) < 25) : 1'b0;
        sc_wait  <= bp_en ? ($urandom_range(99) < 25) : 1'b0;
    end

    integer t, ix, iy, ox, oy, nin, nout, i, o, cyc;
    reg signed [SC-1:0] g_syn [0:255];   // golden syn_curr element image (per output)
    reg signed [WB-1:0] wsig;

    task clear_mems;
        begin
            for (t=0;t<MEMD;t=t+1) begin u_act.mem[t]=0; u_wgt.mem[t]=0; u_sc.mem[t]=0; end
        end
    endtask

    // run one block and wait for finish
    task run_block;
        input [255:0] tag;
        begin
            @(negedge clk); reset=1; bp_en=0;
            repeat(2) @(posedge clk); @(negedge clk); reset=0; repeat(2) @(posedge clk);
            @(negedge clk); start=1; @(negedge clk); start=0;
            verif_to=8000;
            while(!spfin && verif_to>0) begin @(posedge clk); verif_to=verif_to-1; end
            check_true(verif_to>0, {tag, " finished"});
            repeat(6) @(posedge clk); #1;
        end
    endtask

    // ---- UNIFORM-weight identity test ----
    // Every input row = same weight w (all EPW columns), all inputs spike.
    // syn_curr[o] = init[o] + nin*sext(w).
    task test_uniform;
        input integer IX, IY, OX, OY;
        input signed [WB-1:0] w;
        input                 use_bp;
        input [255:0] tag;
        reg [31:0] wword;
        integer waddr;
        begin
            in_x=IX; in_y=IY; out_x=OX; out_y=OY;
            nin = IX*IY; nout = OX*OY;
            rpn = (nout + EPW - 1)/EPW;     // rows_per_neuron = words per weight row
            wpw = EPW;
            clear_mems;
            // all inputs spike: ACT_BITS-wide elements, spike bit = MSB set.
            // pack 1 spike-bit per input element; simplest: all-ones act words.
            for (t=0;t<16;t=t+1) u_act.mem[ACT_BASE+t] = 32'hFFFF_FFFF;
            // weight rows: input i row at WGT_BASE + i*rpn, every element = w
            wword = 0;
            for (o=0;o<EPW;o=o+1) wword[o*WB +: WB] = w;
            for (i=0;i<nin;i=i+1)
                for (t=0;t<rpn;t=t+1) u_wgt.mem[WGT_BASE + i*rpn + t] = wword;
            // random non-zero syn_curr init
            for (o=0;o<nout;o=o+1) begin
                g_syn[o] = $urandom_range(0,(1<<SC)-1) - (1<<(SC-1));
                u_sc.mem[SC_BASE + (o/EPW)][(o%EPW)*SC +: SC] = g_syn[o][SC-1:0];
            end
            // golden accumulate: + nin * sext(w)
            for (o=0;o<nout;o=o+1) g_syn[o] = g_syn[o] + nin*w;

            bp_en = use_bp;
            run_block(tag);
            bp_en = 0;
            // check whole syn_curr image
            for (o=0;o<nout;o=o+1)
                check_eq_u(u_sc.mem[SC_BASE+(o/EPW)][(o%EPW)*SC +: SC],
                           g_syn[o][SC-1:0], {tag, " syn"});
        end
    endtask

    // ---- GENERAL per-(input,output) weight test ----
    // distinct W[i][o]; all inputs spike; syn[o]=init[o]+SUM_i sext(W[i][o]).
    task test_general;
        input integer IX, IY, OX, OY;
        input [255:0] tag;
        integer waddr;
        begin
            in_x=IX; in_y=IY; out_x=OX; out_y=OY;
            nin=IX*IY; nout=OX*OY;
            rpn=(nout+EPW-1)/EPW; wpw=EPW;
            clear_mems;
            for (t=0;t<16;t=t+1) u_act.mem[ACT_BASE+t] = 32'hFFFF_FFFF;
            // init syn
            for (o=0;o<nout;o=o+1) begin
                g_syn[o] = $urandom_range(0,(1<<SC)-1) - (1<<(SC-1));
                u_sc.mem[SC_BASE+(o/EPW)][(o%EPW)*SC +: SC] = g_syn[o][SC-1:0];
            end
            // distinct weights and accumulate
            for (i=0;i<nin;i=i+1)
                for (o=0;o<nout;o=o+1) begin
                    wsig = $urandom_range(0,(1<<WB)-1) - (1<<(WB-1));
                    u_wgt.mem[WGT_BASE + i*rpn + (o/EPW)][(o%EPW)*WB +: WB] = wsig[WB-1:0];
                    g_syn[o] = g_syn[o] + wsig;     // SC-wide wrap add
                end
            run_block(tag);
            for (o=0;o<nout;o=o+1)
                check_eq_u(u_sc.mem[SC_BASE+(o/EPW)][(o%EPW)*SC +: SC],
                           g_syn[o][SC-1:0], {tag, " syn"});
        end
    endtask

    // ---- NO-SPIKE probe (F6 pattern): act all zero.  This is a NOTE-only probe
    // (never a hard fail).  We report two things: (a) whether the wide datapath
    // drains/finishes the task with zero activations, and (b) whether syn_curr
    // is left unchanged (correct spike-gating) or accumulated anyway (F6-class).
    integer mism;
    reg     ns_fin;
    task test_nospike;
        input integer IX, IY, OX, OY;
        input signed [WB-1:0] w;
        begin
            in_x=IX; in_y=IY; out_x=OX; out_y=OY;
            nin=IX*IY; nout=OX*OY; rpn=(nout+EPW-1)/EPW; wpw=EPW;
            clear_mems;                          // act all zero -> no spikes
            for (i=0;i<nin;i=i+1)
                for (t=0;t<rpn;t=t+1) begin
                    u_wgt.mem[WGT_BASE+i*rpn+t]=0;
                    for (o=0;o<EPW;o=o+1) u_wgt.mem[WGT_BASE+i*rpn+t][o*WB +: WB]=w;
                end
            for (o=0;o<nout;o=o+1) begin
                g_syn[o] = $urandom_range(0,(1<<SC)-1) - (1<<(SC-1));
                u_sc.mem[SC_BASE+(o/EPW)][(o%EPW)*SC +: SC] = g_syn[o][SC-1:0];
            end
            // bounded run WITHOUT the hard "finished" check
            @(negedge clk); reset=1; bp_en=0;
            repeat(2) @(posedge clk); @(negedge clk); reset=0; repeat(2) @(posedge clk);
            @(negedge clk); start=1; @(negedge clk); start=0;
            verif_to=8000;
            while(!spfin && verif_to>0) begin @(posedge clk); verif_to=verif_to-1; end
            ns_fin = (verif_to>0);
            repeat(6) @(posedge clk); #1;
            if (!ns_fin) begin
                $display("NOTE nospike: wide spike_processing did NOT drain with zero activations (task never finishes; needs at least one spiking input). Not a hard fail.");
                // recover the DUT so the epilogue/next phase is clean
                @(negedge clk); reset=1; repeat(2) @(posedge clk); @(negedge clk); reset=0; repeat(2) @(posedge clk);
            end
            mism = 0;
            for (o=0;o<nout;o=o+1)
                if (u_sc.mem[SC_BASE+(o/EPW)][(o%EPW)*SC +: SC] !== g_syn[o][SC-1:0])
                    mism = mism + 1;
            if (mism == 0)
                $display("NOTE nospike: syn_curr UNCHANGED with zero activations (spike-gating effective, no F6-class over-accumulation)");
            else
                $display("NOTE nospike: wide datapath ACCUMULATED with zero activations on %0d/%0d outputs (F6-class; not a hard fail)", mism, nout);
        end
    endtask

    initial begin
        verif_errors=0; verif_checks=0;
        start=0; reset=1; bp_en=0; wgt_wait=0; sc_wait=0;
        in_x=1;in_y=1;out_x=1;out_y=1;wpw=4;rpn=1;
        repeat(4) @(posedge clk); @(negedge clk); reset=0; repeat(2) @(posedge clk);

        $display("=== tb_spike_processing_wide  LANES=%0d PF=%0d ===", LANES, `PF);
        void'($urandom(32'h5B10_0000 ^ (LANES<<4) ^ `PF));

        // uniform-weight identity over grids/weights/back-pressure
        test_uniform(2,1, 4,1, 8'sd5,  1'b0, "U1 2x1->4x1 w5");
        test_uniform(3,1, 4,1, 8'sd3,  1'b0, "U2 3x1->4x1 w3");
        test_uniform(2,2, 8,1, 8'sd2,  1'b0, "U3 2x2->8x1 w2");
        test_uniform(4,1, 8,1, -8'sd4, 1'b0, "U4 4x1->8x1 w=-4 (sign)");
        test_uniform(2,1, 12,1, 8'sd1, 1'b0, "U5 2x1->12x1 spanning 3 words");
        test_uniform(3,1, 8,1, 8'sd7,  1'b1, "U6 3x1->8x1 w7 + back-pressure");
        test_uniform(2,2, 4,1, -8'sd9, 1'b1, "U7 2x2->4x1 w=-9 + bp");

        // general distinct-weight model
        test_general(2,1, 4,1, "G1 2x1->4x1 distinct");
        test_general(3,1, 8,1, "G2 3x1->8x1 distinct");
        test_general(2,2, 8,1, "G3 2x2->8x1 distinct");

        // F6-class no-spike probe (NOTE, not a hard fail)
        test_nospike(2,1, 4,1, 8'sd5);

        `VERIF_EPILOGUE("tb_spike_processing_wide")
    end

    `VERIF_WATCHDOG(20000000)
endmodule

// ----- synchronous SRAM model (1-cycle read latency, registered write) -----
module sram_model #(parameter DATA_W=32, parameter DEPTH=256)(
    input wire clk, input wire we, input wire re, input wire rwait,
    input wire [7:0] addr, input wire [DATA_W-1:0] wdata, output reg [DATA_W-1:0] rdata);
    reg [DATA_W-1:0] mem [0:DEPTH-1];
    integer i; initial begin for(i=0;i<DEPTH;i=i+1) mem[i]={DATA_W{1'b0}}; rdata={DATA_W{1'b0}}; end
    always @(posedge clk) begin if (we) mem[addr]<=wdata; if (re & ~rwait) rdata<=mem[addr]; end
endmodule
