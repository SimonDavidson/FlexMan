// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
// =============================================================================
// tb_acc_snn_wide — end-to-end wide snnAcc (SP -> NP), FULL connectivity,
// narrow (8/16-bit) syn_curr & potential.  Authors: Simon Davidson & Claude.
// Created 2026-06-07.  Last modified 2026-11-21 (packed per-task config).
//
// Layer: 2 inputs (both spike) x 4 outputs, weights W[0]={1,2,3,4} W[1]={5,6,7,8}
//   SP:  syn_curr[o] = W0[o]+W1[o] = {6,8,10,12}
//   NP (thresh=50 > pot, no spike, decay 0.5):
//        pot_out      = {6,8,10,12} * 0.5 = {3,4,5,6}
//        syn_curr_wb  = {6,8,10,12} * 0.5 = {3,4,5,6}
//        spikes       = 0
//   Packed (EB-wide, LSB-first): for EB=8 -> word 0x06050403.
// Golden is lane- and width-independent (values <127); PASS at LANES=1/2/4
// (8-bit) and 1/2 (16-bit) proves end-to-end equivalence + the new narrow-
// syn_curr pipeline.
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
 `define PF 1
`endif

module tb_acc_snn_wide;
    localparam CLK=10, LANES=`LANES, EB=`EB, MEMD=256;
    localparam SS = (EB==8)?3:(EB==16)?4:5;          // slice-size code
    reg clk=0; always #(CLK/2) clk=~clk;
    reg reset; integer errors=0, t;

    // AXI cfg
    reg sys_req=0; reg [31:0] sys_addr=0, sys_data=0; wire sys_ack;
    reg start=0; reg [`TGT_ACC_SZ-1:0] tacc=0; reg [`SCH_ENTRY_SZ-1:0] binfo=0;
    wire spfin, busy, fin;

    // memory buses
    wire wgt_rd; wire [`ADDR_SIZE-1:0] wgt_addr; wire [`WTD_BITS-1:0] wgt_data;
    wire act_rd; wire [`ADDR_SIZE-1:0] act_addr; wire [`ACT_BITS-1:0] act_data;
    wire sc_wr, sc_rd; wire [`ADDR_SIZE-1:0] sc_addr; wire [`POT_BITS-1:0] sc_wd, sc_rdd;
    wire b_rd; wire [`ADDR_SIZE-1:0] b_addr; wire [`WTD_BITS-1:0] b_data;
    wire th_rd; wire [`ADDR_SIZE-1:0] th_addr; wire [`WTD_BITS-1:0] th_data;
    wire p_wr, p_rd; wire [`ADDR_SIZE-1:0] p_addr; wire [`POT_BITS-1:0] p_wd, p_rdd;
    wire spk_wr; wire [`ADDR_SIZE-1:0] spk_addr; wire [`ACT_BITS-1:0] spk_wd;

    acc_snn_processor #(
        .TGT_ACC_ID(0), .TGT_CONFIG_BASE_ADDR(32'hFFFFFFFF),
        .NUM_LANES(LANES), .PREFETCH(`PF),
        .SP_ELEM_SZ(8), .SP_ACT_SLICE_SZ(3), .SP_WEIGHT_ENTRY_BITS(8),
        .SP_WEIGHT_SLICE_SZ(5), .SP_SYN_CURR_SLICE_SZ(SS), .SP_SYN_CURR_SLICE_BITS(EB),
        .NP_SYN_CURR_SLICE_SZ(SS), .NP_SYN_CURR_SLICE_BITS(EB),
        .NP_POT_SLICE_SZ(SS), .NP_POT_SLICE_BITS(EB),
        .NP_BIAS_CURR_SLICE_BITS(8))
    dut (
        .clk(clk), .reset(reset),
        .sys_req_i(sys_req), .sys_ack_o(sys_ack), .sys_addr_i(sys_addr), .sys_data_i(sys_data),
        .start_new_block_i(start), .target_acc_i(tacc), .buffer_info_i(binfo),
        .spike_proc_finished_o(spfin), .acc_busy_o(busy), .acc_finished_o(fin),
        .sp_src1_buff_addr_i(0), .sp_src2_buff_addr_i(0), .sp_src3_buff_addr_i(0),
        .sp_tgt_buff_addr_i(0), .sp_weight_row_len_i(0),
        .np_src1_buff_addr_i(0), .np_src2_buff_addr_i(0), .np_src3_buff_addr_i(0),
        .np_tgt_buff_addr_i(0), .np_weight_row_len_i(0),
        .weight_mem_rd_o(wgt_rd), .weight_mem_wait_i(1'b0), .weight_mem_addr_o(wgt_addr), .weight_mem_data_i(wgt_data),
        .act_mem_req_o(act_rd), .act_mem_wait_i(1'b0), .act_mem_addr_o(act_addr), .act_mem_data_i(act_data),
        .syn_curr_mem_wr_o(sc_wr), .syn_curr_mem_rd_o(sc_rd), .syn_curr_mem_wait_i(1'b0),
        .syn_curr_mem_addr_o(sc_addr), .syn_curr_mem_data_o(sc_wd), .syn_curr_mem_data_i(sc_rdd),
        .bias_curr_mem_rd_o(b_rd), .bias_curr_mem_wait_i(1'b0), .bias_curr_mem_addr_o(b_addr), .bias_curr_mem_data_i(b_data),
        .thresh_mem_rd_o(th_rd), .thresh_mem_wait_i(1'b0), .thresh_mem_addr_o(th_addr), .thresh_mem_data_i(th_data),
        .pot_mem_wr_o(p_wr), .pot_mem_rd_o(p_rd), .pot_mem_wait_i(1'b0),
        .pot_mem_addr_o(p_addr), .pot_mem_data_o(p_wd), .pot_mem_data_i(p_rdd),
        .spike_mem_wr_o(spk_wr), .spike_mem_wait_i(1'b0), .spike_mem_addr_o(spk_addr), .spike_mem_data_o(spk_wd));

    sram_model #(.DATA_W(32),.DEPTH(MEMD)) u_wgt(.clk(clk),.we(1'b0),.re(wgt_rd),.addr(wgt_addr[7:0]),.wdata(32'b0),.rdata(wgt_data));
    sram_model #(.DATA_W(32),.DEPTH(MEMD)) u_act(.clk(clk),.we(1'b0),.re(act_rd),.addr(act_addr[7:0]),.wdata(32'b0),.rdata(act_data));
    sram_model #(.DATA_W(32),.DEPTH(MEMD)) u_sc (.clk(clk),.we(sc_wr),.re(sc_rd),.addr(sc_addr[7:0]),.wdata(sc_wd),.rdata(sc_rdd));
    sram_model #(.DATA_W(32),.DEPTH(MEMD)) u_b  (.clk(clk),.we(1'b0),.re(b_rd),.addr(b_addr[7:0]),.wdata(32'b0),.rdata(b_data));
    sram_model #(.DATA_W(32),.DEPTH(MEMD)) u_th (.clk(clk),.we(1'b0),.re(th_rd),.addr(th_addr[7:0]),.wdata(32'b0),.rdata(th_data));
    sram_model #(.DATA_W(32),.DEPTH(MEMD)) u_p  (.clk(clk),.we(p_wr),.re(p_rd),.addr(p_addr[7:0]),.wdata(p_wd),.rdata(p_rdd));
    sram_model #(.DATA_W(32),.DEPTH(MEMD)) u_spk(.clk(clk),.we(spk_wr),.re(1'b0),.addr(spk_addr[7:0]),.wdata(spk_wd),.rdata());

    task cfg; input [31:0] a,d; begin
        @(negedge clk); sys_req=1; sys_addr=a; sys_data=d; #1;
        if(!sys_ack) begin $display("FAIL no ack @%08h",a); errors=errors+1; end
        @(negedge clk); sys_req=0; end endtask

    // expected packed word for {3,4,5,6} at EB
    function [31:0] packed4; input [31:0] e0,e1,e2,e3; begin
        packed4 = (e0 & ((1<<EB)-1)) | (e1 << EB) | (e2 << (2*EB)) | (e3 << (3*EB));
    end endfunction

    reg [31:0] expw;
    integer to;
    initial begin
        reset=1;
        for (t=0;t<MEMD;t=t+1) begin u_wgt.mem[t]=0;u_act.mem[t]=0;u_sc.mem[t]=0;u_b.mem[t]=0;u_th.mem[t]=0;u_p.mem[t]=0;u_spk.mem[t]=0; end
        u_act.mem[0]   = 32'hFFFF_FFFF;     // both inputs spike
        u_wgt.mem[10]  = 32'h04030201;      // W[0]={1,2,3,4}
        u_wgt.mem[11]  = 32'h08070605;      // W[1]={5,6,7,8}
        // thresh=50 packed 8-bit x4 (no spike); at base 40
        u_th.mem[40]   = 32'h32323232;
        repeat(5) @(posedge clk); @(negedge clk); reset=0; repeat(2) @(posedge clk);

        // Packed per-task layout (word i at offset i*4), as acc_snn_processor decodes.
        cfg(32'hFFFF_0000, 0);             // act_base
        cfg(32'hFFFF_0004, 10);            // weight_base
        cfg(32'hFFFF_0008, 20);            // syn_curr_base
        cfg(32'hFFFF_000C, 30);            // bias_base
        cfg(32'hFFFF_0010, 40);            // thresh_base
        cfg(32'hFFFF_0014, 50);            // pot_base
        cfg(32'hFFFF_0018, 60);            // spike_base
        cfg(32'hFFFF_001C, 32'h8000_0000); // syn decay 0.5
        cfg(32'hFFFF_0020, 32'h8000_0000); // pot decay 0.5
        cfg(32'hFFFF_0024, 32'h0001_0002); // S0 in_y=1 | in_x=2
        cfg(32'hFFFF_0028, 32'h0001_0004); // S1 out_y=1 | out_x=4
        cfg(32'hFFFF_002C, 32'h0003_0001); // S2 last_neuron_idx=3 | rows_per_neuron=1
        cfg(32'hFFFF_0030, 32'h0000_0001); // S3 total_timesteps=1
        // M0: weight_sz=3 (8b) | syn_curr_sz=SS | bias_sz=3 (8b) | pot_sz=SS | weight_mode=0 (full)
        cfg(32'hFFFF_0034, 32'd3 | (SS<<4) | (32'd3<<8) | (SS<<12));
        // M1: skip=0 np_mode=0 weights_per_word=4 bin_point=0
        cfg(32'hFFFF_0038, 32'h0000_0100);
        // boot-only conv/sparse params
        cfg(32'hFFFF_005C, 5);             // weight_idx_sz
        cfg(32'hFFFF_0074, 1); cfg(32'hFFFF_0078, 1);
        cfg(32'hFFFF_007C, 1); cfg(32'hFFFF_0080, 1);
        cfg(32'hFFFF_0084, 0); cfg(32'hFFFF_0088, 0);

        @(negedge clk); start=1; @(negedge clk); start=0;
        to=0; while(!fin && to<8000) begin @(posedge clk); to=to+1; end
        @(posedge clk); #1;
        if (to>=8000) begin $display("FAIL timeout"); errors=errors+1; end

        $display("== acc_snn_wide LANES=%0d EB=%0d : finished in %0d cycles ==", LANES, EB, to);
        expw = packed4(3,4,5,6);
        // 4 neurons fit one word at EB=8; at EB=16 they span 2 words (3,4 | 5,6)
        if (LANES==1) begin
            // Original (scalar) path: one element per 32-bit word, NOT narrow-packed.
            // Same arithmetic as the wide path; only the memory layout differs.
            for (t=0;t<4;t=t+1) begin
                if (u_p.mem[50+t]  !== (3+t)) begin $display("FAIL L1 pot[%0d]=0x%08h exp %0d",50+t,u_p.mem[50+t],3+t); errors=errors+1; end
                if (u_sc.mem[20+t] !== (3+t)) begin $display("FAIL L1 syn[%0d]=0x%08h exp %0d",20+t,u_sc.mem[20+t],3+t); errors=errors+1; end
            end
        end else if (EB==8) begin
            if (u_p.mem[50]  !== expw) begin $display("FAIL pot[50]=0x%08h exp 0x%08h",u_p.mem[50],expw); errors=errors+1; end
            if (u_sc.mem[20] !== expw) begin $display("FAIL syn[20]=0x%08h exp 0x%08h",u_sc.mem[20],expw); errors=errors+1; end
        end else begin
            if (u_p.mem[50]  !== ((4<<16)|3)) begin $display("FAIL pot[50]=0x%08h exp 0x%08h",u_p.mem[50],(4<<16)|3); errors=errors+1; end
            if (u_p.mem[51]  !== ((6<<16)|5)) begin $display("FAIL pot[51]=0x%08h exp 0x%08h",u_p.mem[51],(6<<16)|5); errors=errors+1; end
            if (u_sc.mem[20] !== ((4<<16)|3)) begin $display("FAIL syn[20]=0x%08h",u_sc.mem[20]); errors=errors+1; end
            if (u_sc.mem[21] !== ((6<<16)|5)) begin $display("FAIL syn[21]=0x%08h",u_sc.mem[21]); errors=errors+1; end
        end
        if (u_spk.mem[60] !== 32'd0) begin $display("FAIL spike[60]=0x%08h exp 0",u_spk.mem[60]); errors=errors+1; end

        if (errors==0) $display("PASS (LANES=%0d EB=%0d): cycles=%0d", LANES, EB, to);
        else           $display("FAIL: %0d error(s)", errors);
        $finish;
    end
    initial begin #2000000; $display("FAIL global timeout"); $finish; end
endmodule

module sram_model #(parameter DATA_W=32, parameter DEPTH=256)(
    input wire clk, input wire we, input wire re,
    input wire [7:0] addr, input wire [DATA_W-1:0] wdata, output reg [DATA_W-1:0] rdata);
    reg [DATA_W-1:0] mem [0:DEPTH-1];
    integer i; initial begin for(i=0;i<DEPTH;i=i+1) mem[i]={DATA_W{1'b0}}; rdata={DATA_W{1'b0}}; end
    always @(posedge clk) begin if (we) mem[addr]<=wdata; if (re) rdata<=mem[addr]; end
endmodule
