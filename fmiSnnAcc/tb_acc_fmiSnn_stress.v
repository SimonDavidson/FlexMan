// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
// =============================================================================
// tb_acc_fmiSnn_stress  (fmiSnnAcc)  --  focused F9 gapped-activation confirmation
//
// Authors      : Simon Davidson & Claude
// Created      : 2026-06-30
// Last modified: 2026-06-30
//
// Port of snnAcc/tb_acc_snn_stress.v narrowed to the F9 surface for fmiSnnAcc
// (top module `acc_fmiSnn_processor`): confirm the full-mode FC weight-cache phase
// fix (the `& act_data_valid_i` gate on weight_index_valid_full) holds end-to-end.
//
// Runs the WHOLE acc_fmiSnn_processor with sp_skip_neuron=1 (neuron stage off, no
// FMI neuron-model golden needed), so it only exercises spike_processing's
// accumulation. fmiSnnAcc is a 1-bit SPIKE-gate front-end (active = act MSB set,
// like snnAcc) and is 1-D (Y hardwired to 1). We drive act in {0,1} (1 = spike ->
// contributes the weight; 0 = GAP), so the connectivity golden is
// syn[j] = sum over spiking i of weight[i][j]. The 9 NP-side memory ports
// (thresh/pot/spike + per-neuron decay/ada) are tied off (NP never runs).
//
// Oracles: connectivity golden + stall-invariance on syn_curr; directed gap_probe.
// =============================================================================
`timescale 1ns/1ps

`include "../shared/constants.v"

`define TGT_ACC_ID 'h0
`define TGT_CONFIG_BASE_ADDR 32'hFFFFFFFF
`define NUM_TIMESTEPS 32
`define X_INPUT_SZ 5
`define X_OUTPUT_SZ 5
`define X_KERNEL_SZ 3
`define X_STEP_SZ   3
`define ELEMS_PER_ROW   4
`define ROWS_PER_NEURON 4
`define ELEM_SZ 8
`define WEIGHT_SLICE_SZ 5
`define WEIGHT_IDX_SZ 10
`define WEIGHT_DATA_IDX_SZ 5
`define ACT_SLICE_SZ 3
`define ACT_DATA_IDX_SZ 5
`define SYN_CURR_IDX_SZ 10
`define SYN_CURR_DATA_IDX_SZ 5
`define SYN_CURR_SLICE_SZ 3
`define SYN_CURR_SLICE_BITS 32
`define BIAS_CURR_IDX_SZ 10
`define BIAS_CURR_DATA_IDX_SZ 5
`define BIAS_CURR_SLICE_SZ 3
`define BIAS_CURR_SLICE_BITS 8
`define POT_IDX_SZ 10
`define POT_DATA_IDX_SZ 5
`define POT_SLICE_SZ 3
`define POT_SLICE_BITS 32
`define SPIKE_IDX_SZ 10
`define SPIKE_DATA_IDX_SZ 5
`define SPIKE_SLICE_SZ 3
`define SPIKE_SLICE_BITS 8

module tb_acc_fmiSnn_stress;

    localparam CLK_PERIOD = 10;
    localparam integer ACT_BASE    = 0;
    localparam integer WEIGHT_BASE = 8;
    localparam integer SYN_BASE    = 40;
    localparam [31:0]  CFG         = 32'hFFFF_0000;
    localparam         IS_MAC      = 1'b0;     // fmiSnnAcc: 1-bit spike gate

    reg clk, reset;
    initial clk = 1'b0;
    always #(CLK_PERIOD/2) clk = ~clk;

    integer verif_errors, verif_checks, verif_to, sram_i;
    `include "../verif/checks.vh"
    `include "../verif/vt_driver.vh"
    `include "../verif/sram_bfm.vh"

    reg                      sys_req_i = 0;
    reg             [31:0]   sys_addr_i = 0, sys_data_i = 0;
    reg                      start_new_block_i = 0;
    reg   [`TGT_ACC_SZ-1:0]  target_acc_i  = 0;
    reg [`SCH_ENTRY_SZ-1:0]  buffer_info_i = 0;
    reg [`PIN_BITS-1:0] sp_src1_buff_addr_i=0, sp_src2_buff_addr_i=0, sp_src3_buff_addr_i=0,
                        sp_tgt_buff_addr_i=0, sp_weight_row_len_i=0;
    reg [`PIN_BITS-1:0] np_src1_buff_addr_i=0, np_src2_buff_addr_i=0, np_src3_buff_addr_i=0,
                        np_tgt_buff_addr_i=0, np_weight_row_len_i=0;

    wire sys_ack_o, spike_proc_finished_o, acc_busy_o, acc_finished_o;
    wire weight_mem_rd_o;  wire [`ADDR_SIZE-1:0] weight_mem_addr_o;
    wire act_mem_req_o;    wire [`ADDR_SIZE-1:0] act_mem_addr_o;
    wire syn_curr_mem_wr_o, syn_curr_mem_rd_o;
    wire [`ADDR_SIZE-1:0] syn_curr_mem_addr_o;  wire [`POT_BITS-1:0] syn_curr_mem_data_o;
    wire thresh_mem_rd_o;     wire [`ADDR_SIZE-1:0] thresh_mem_addr_o;
    wire pot_mem_wr_o, pot_mem_rd_o;  wire [`ADDR_SIZE-1:0] pot_mem_addr_o;  wire [`POT_BITS-1:0] pot_mem_data_o;
    wire spike_mem_wr_o;  wire [`ADDR_SIZE-1:0] spike_mem_addr_o;  wire [`ACT_BITS-1:0] spike_mem_data_o;
    wire dcy_syn_mem_rd_o; wire [`ADDR_SIZE-1:0] dcy_syn_mem_addr_o;
    wire dcy_mem_mem_rd_o; wire [`ADDR_SIZE-1:0] dcy_mem_mem_addr_o;
    wire ada_mem_wr_o, ada_mem_rd_o; wire [`ADDR_SIZE-1:0] ada_mem_addr_o; wire [`POT_BITS-1:0] ada_mem_data_o;
    wire b_eff_mem_rd_o;  wire [`ADDR_SIZE-1:0] b_eff_mem_addr_o;
    wire dcy_ada_mem_rd_o; wire [`ADDR_SIZE-1:0] dcy_ada_mem_addr_o;
    wire scl_ada_mem_rd_o; wire [`ADDR_SIZE-1:0] scl_ada_mem_addr_o;

    // active SP-side buses (BFMs)
    reg [`WTD_BITS-1:0] weight_mem_data_i;
    reg [`ACT_BITS-1:0] act_mem_data_i;
    reg [`POT_BITS-1:0] syn_curr_mem_data_i;
    reg weight_wait, act_wait, syn_wait;

    // tied-off NP-side inputs (NP never runs under skip_neuron)
    wire [`WTD_BITS-1:0] thresh_mem_data_i = 0, b_eff_mem_data_i = 0;
    wire [`POT_BITS-1:0] pot_mem_data_i = 0, ada_mem_data_i = 0;
    wire [`POT_BITS-1:0] dcy_syn_mem_data_i = 0, dcy_mem_mem_data_i = 0,
                         dcy_ada_mem_data_i = 0, scl_ada_mem_data_i = 0;

    acc_fmiSnn_processor # (
        .TGT_ACC_ID(`TGT_ACC_ID), .TGT_CONFIG_BASE_ADDR(`TGT_CONFIG_BASE_ADDR),
        .SP_NUM_TIMESTEPS(`NUM_TIMESTEPS),
        .SP_X_INPUT_SZ(`X_INPUT_SZ), .SP_X_OUTPUT_SZ(`X_OUTPUT_SZ),
        .SP_X_KERNEL_SZ(`X_KERNEL_SZ), .SP_X_KERNEL_OFF_SZ(`X_STEP_SZ), .SP_X_STEP_SZ(`X_STEP_SZ),
        .SP_ELEMS_PER_ROW(`ELEMS_PER_ROW), .SP_ROWS_PER_NEURON(`ROWS_PER_NEURON),
        .SP_TIMESTEP_SZ(10), .SP_IN_DATA_BITS(32), .SP_ELEM_SZ(`ELEM_SZ),
        .SP_ACT_SLICE_SZ(`ACT_SLICE_SZ), .SP_ACT_DATA_IDX_SZ(`ACT_DATA_IDX_SZ),
        .SP_WEIGHT_ENTRY_BITS(8), .SP_WEIGHT_IDX_SZ(`WEIGHT_IDX_SZ),
        .SP_WEIGHT_SLICE_SZ(`WEIGHT_SLICE_SZ), .SP_WEIGHT_DATA_IDX_SZ(`WEIGHT_DATA_IDX_SZ),
        .SP_SYN_CURR_IDX_SZ(`SYN_CURR_IDX_SZ), .SP_SYN_CURR_DATA_IDX_SZ(`SYN_CURR_DATA_IDX_SZ),
        .SP_SYN_CURR_SLICE_SZ(`SYN_CURR_SLICE_SZ), .SP_SYN_CURR_SLICE_BITS(`SYN_CURR_SLICE_BITS),
        .SP_BIAS_CURR_IDX_SZ(`BIAS_CURR_IDX_SZ), .SP_BIAS_CURR_DATA_IDX_SZ(`BIAS_CURR_DATA_IDX_SZ),
        .SP_BIAS_CURR_SLICE_SZ(`BIAS_CURR_SLICE_SZ), .SP_BIAS_CURR_SLICE_BITS(`BIAS_CURR_SLICE_BITS),
        .NP_NUM_TIMESTEPS(`NUM_TIMESTEPS), .NP_TIMESTEP_SZ(10), .NP_IN_DATA_BITS(32),
        .NP_NEURON_IDX_SZ(10),
        .NP_SYN_CURR_IDX_SZ(`SYN_CURR_IDX_SZ), .NP_SYN_CURR_DATA_IDX_SZ(`SYN_CURR_DATA_IDX_SZ),
        .NP_SYN_CURR_SLICE_SZ(`SYN_CURR_SLICE_SZ), .NP_SYN_CURR_SLICE_BITS(`SYN_CURR_SLICE_BITS),
        .NP_POT_IDX_SZ(`POT_IDX_SZ), .NP_POT_DATA_IDX_SZ(`POT_DATA_IDX_SZ),
        .NP_POT_SLICE_SZ(`POT_SLICE_SZ), .NP_POT_SLICE_BITS(`POT_SLICE_BITS),
        .NP_SPIKE_IDX_SZ(`SPIKE_IDX_SZ), .NP_SPIKE_DATA_IDX_SZ(`SPIKE_DATA_IDX_SZ),
        .NP_SPIKE_SLICE_SZ(`SPIKE_SLICE_SZ), .NP_SPIKE_SLICE_BITS(`SPIKE_SLICE_BITS),
        .MEM_ADDR_BITS(`ADDR_SIZE)
    ) u_dut (
        .clk(clk), .reset(reset),
        .sys_req_i(sys_req_i), .sys_ack_o(sys_ack_o),
        .sys_addr_i(sys_addr_i), .sys_data_i(sys_data_i),
        .start_new_block_i(start_new_block_i), .target_acc_i(target_acc_i),
        .buffer_info_i(buffer_info_i),
        .spike_proc_finished_o(spike_proc_finished_o),
        .acc_busy_o(acc_busy_o), .acc_finished_o(acc_finished_o),
        .sp_src1_buff_addr_i(sp_src1_buff_addr_i), .sp_src2_buff_addr_i(sp_src2_buff_addr_i),
        .sp_src3_buff_addr_i(sp_src3_buff_addr_i), .sp_tgt_buff_addr_i(sp_tgt_buff_addr_i),
        .sp_weight_row_len_i(sp_weight_row_len_i),
        .np_src1_buff_addr_i(np_src1_buff_addr_i), .np_src2_buff_addr_i(np_src2_buff_addr_i),
        .np_src3_buff_addr_i(np_src3_buff_addr_i), .np_tgt_buff_addr_i(np_tgt_buff_addr_i),
        .np_weight_row_len_i(np_weight_row_len_i),
        .weight_mem_rd_o(weight_mem_rd_o), .weight_mem_wait_i(weight_wait),
        .weight_mem_addr_o(weight_mem_addr_o), .weight_mem_data_i(weight_mem_data_i),
        .act_mem_req_o(act_mem_req_o), .act_mem_wait_i(act_wait),
        .act_mem_addr_o(act_mem_addr_o), .act_mem_data_i(act_mem_data_i),
        .syn_curr_mem_wr_o(syn_curr_mem_wr_o), .syn_curr_mem_rd_o(syn_curr_mem_rd_o),
        .syn_curr_mem_wait_i(syn_wait), .syn_curr_mem_addr_o(syn_curr_mem_addr_o),
        .syn_curr_mem_data_o(syn_curr_mem_data_o), .syn_curr_mem_data_i(syn_curr_mem_data_i),
        .thresh_mem_rd_o(thresh_mem_rd_o), .thresh_mem_wait_i(1'b0),
        .thresh_mem_addr_o(thresh_mem_addr_o), .thresh_mem_data_i(thresh_mem_data_i),
        .pot_mem_wr_o(pot_mem_wr_o), .pot_mem_rd_o(pot_mem_rd_o),
        .pot_mem_wait_i(1'b0), .pot_mem_addr_o(pot_mem_addr_o),
        .pot_mem_data_o(pot_mem_data_o), .pot_mem_data_i(pot_mem_data_i),
        .spike_mem_wr_o(spike_mem_wr_o), .spike_mem_wait_i(1'b0),
        .spike_mem_addr_o(spike_mem_addr_o), .spike_mem_data_o(spike_mem_data_o),
        .dcy_syn_mem_rd_o(dcy_syn_mem_rd_o), .dcy_syn_mem_wait_i(1'b0),
        .dcy_syn_mem_addr_o(dcy_syn_mem_addr_o), .dcy_syn_mem_data_i(dcy_syn_mem_data_i),
        .dcy_mem_mem_rd_o(dcy_mem_mem_rd_o), .dcy_mem_mem_wait_i(1'b0),
        .dcy_mem_mem_addr_o(dcy_mem_mem_addr_o), .dcy_mem_mem_data_i(dcy_mem_mem_data_i),
        .ada_mem_wr_o(ada_mem_wr_o), .ada_mem_rd_o(ada_mem_rd_o), .ada_mem_wait_i(1'b0),
        .ada_mem_addr_o(ada_mem_addr_o), .ada_mem_data_o(ada_mem_data_o), .ada_mem_data_i(ada_mem_data_i),
        .b_eff_mem_rd_o(b_eff_mem_rd_o), .b_eff_mem_wait_i(1'b0),
        .b_eff_mem_addr_o(b_eff_mem_addr_o), .b_eff_mem_data_i(b_eff_mem_data_i),
        .dcy_ada_mem_rd_o(dcy_ada_mem_rd_o), .dcy_ada_mem_wait_i(1'b0),
        .dcy_ada_mem_addr_o(dcy_ada_mem_addr_o), .dcy_ada_mem_data_i(dcy_ada_mem_data_i),
        .scl_ada_mem_rd_o(scl_ada_mem_rd_o), .scl_ada_mem_wait_i(1'b0),
        .scl_ada_mem_addr_o(scl_ada_mem_addr_o), .scl_ada_mem_data_i(scl_ada_mem_data_i)
    );

    `SRAM_RD_WAIT(act_sram,    act_mem_req_o,   act_wait,    act_mem_addr_o,    act_mem_data_i)
    `SRAM_RD_WAIT(weight_sram, weight_mem_rd_o, weight_wait, weight_mem_addr_o, weight_mem_data_i)
    `SRAM_RW_WAIT(syn_sram, syn_curr_mem_rd_o, syn_curr_mem_wr_o, syn_wait,
                  syn_curr_mem_addr_o, syn_curr_mem_data_o, syn_curr_mem_data_i)

    reg     stall_en;
    integer stall_pct;
    always @(posedge clk) begin
        if (!stall_en) begin weight_wait<=0; act_wait<=0; syn_wait<=0; end
        else begin
            weight_wait <= ($urandom_range(99) < stall_pct);
            act_wait    <= ($urandom_range(99) < stall_pct);
            syn_wait    <= ($urandom_range(99) < (stall_pct + 20));
        end
    end

    // ================================================================
    // Helpers
    // ================================================================
    task cfg_write;
        input [31:0] addr, data;
        begin
            @(negedge clk); sys_req_i=1; sys_addr_i=addr; sys_data_i=data; #1;
            if (!sys_ack_o) begin
                $display("FAIL cfg_write: no ACK for addr=0x%08h", addr);
                verif_errors = verif_errors + 1;
            end
            @(negedge clk); sys_req_i=0;
        end
    endtask

    function signed [31:0] sgnext;
        input [31:0] v; input integer b;
        begin sgnext = $signed(v << (32-b)) >>> (32-b); end
    endfunction

    task pack_stream;
        input integer base, k, wb; input [31:0] val;
        integer p, wd, bt;
        begin p=wb*k; wd=p/32; bt=p%32;
              weight_sram[base+wd] = weight_sram[base+wd] | ((val & ((1<<wb)-1)) << bt); end
    endtask

    // ================================================================
    // Per-config state + connectivity golden (1-D: Y=1)
    // ================================================================
    integer cmode, cwszc, cwszb;
    integer cinx, coutx, crpn;
    integer ckxl, ckxs, ckxo;
    integer ciszc, ctszc, ciszb, cwszb_sp, ctszb, cscnt;
    integer nin, nout;

    integer           act      [0:31];
    reg signed [31:0] wfull    [0:511];
    reg signed [31:0] kern     [0:63];
    integer           sidx     [0:255];
    reg signed [31:0] swgt     [0:255];
    reg signed [31:0] syn_acc  [0:63];
    reg [31:0]        ref_syn  [0:63];

    integer gi, gj, gt, gox, ox;

    task compute_golden;
        begin
            for (gj=0; gj<nout; gj=gj+1) syn_acc[gj] = 32'sd0;
            if (cmode == 0) begin
                for (gi=0; gi<nin; gi=gi+1)
                    if (act[gi] != 0)
                        for (gj=0; gj<nout; gj=gj+1)
                            syn_acc[gj] = syn_acc[gj] + act[gi]*wfull[gi*32+gj];
            end else if (cmode == 1) begin
                for (gi=0; gi<nin; gi=gi+1)
                    if (act[gi] != 0)
                        for (gt=0; gt<cscnt; gt=gt+1)
                            syn_acc[sidx[gi*16+gt]] = syn_acc[sidx[gi*16+gt]] + act[gi]*swgt[gi*16+gt];
            end else begin                              // CONV (1-D)
                for (gi=0; gi<nin; gi=gi+1)
                    if (act[gi] != 0)
                        for (gox=0; gox<ckxl; gox=gox+1) begin
                            ox = gi*ckxs + gox - ckxo;
                            if (ox>=0 && ox<coutx)
                                syn_acc[ox] = syn_acc[ox] + act[gi]*kern[gox];
                        end
            end
        end
    endtask

    task setup_mem;
        integer i, t, w0;
        begin
            for (sram_i=0; sram_i<256; sram_i=sram_i+1) begin
                act_sram[sram_i]=0; weight_sram[sram_i]=0; syn_sram[sram_i]=0;
            end
            for (i=0; i<nin; i=i+1)
                if (IS_MAC) act_sram[ACT_BASE + i/4]  = act_sram[ACT_BASE + i/4]  | ((act[i] & 8'hFF) << ((i%4)*8));
                else        act_sram[ACT_BASE + i/32] = act_sram[ACT_BASE + i/32] | ((act[i] & 1)      << (i%32));

            if (cmode == 0) begin
                for (i=0; i<nin; i=i+1)
                    for (t=0; t<nout; t=t+1)
                        pack_stream(WEIGHT_BASE + i*crpn, t, cwszb, wfull[i*32+t]);
            end else if (cmode == 1) begin
                for (i=0; i<nin; i=i+1) begin
                    w0 = 0;
                    for (t=0; t<cscnt; t=t+1)
                        w0 = w0 | ( ( (sidx[i*16+t] << (ctszb - ciszb)) |
                                      ((swgt[i*16+t] & ((1<<cwszb_sp)-1)) << (ctszb - ciszb - cwszb_sp)) )
                                    << (ctszb*t) );
                    weight_sram[WEIGHT_BASE + i] = w0;
                end
            end else begin
                for (i=0; i<ckxl; i=i+1)
                    pack_stream(WEIGHT_BASE, i, cwszb, kern[i]);
            end

            for (i=0; i<nout; i=i+1) syn_sram[SYN_BASE + i] = 0;
        end
    endtask

    task configure;
        begin
            cfg_write(CFG+32'h00, ACT_BASE);     cfg_write(CFG+32'h04, WEIGHT_BASE);
            cfg_write(CFG+32'h08, SYN_BASE);     cfg_write(CFG+32'h0C, 100);   // thresh
            cfg_write(CFG+32'h10, 110);          cfg_write(CFG+32'h14, 120);   // pot, spike
            cfg_write(CFG+32'h18, 130);          cfg_write(CFG+32'h1C, 140);   // dcy_syn, dcy_mem
            cfg_write(CFG+32'h20, 150);          cfg_write(CFG+32'h24, 160);   // ada, b_eff
            cfg_write(CFG+32'h28, 170);          cfg_write(CFG+32'h2C, 180);   // dcy_ada, scl_ada
            cfg_write(CFG+32'h30, (coutx<<16) | cinx);        // S0 out_x | in_x
            cfg_write(CFG+32'h34, ((nout-1)<<16) | crpn);     // S1 last_neuron | rpn
            cfg_write(CFG+32'h38, 32'd1);                     // S2 total_timesteps
            // M0: skip[1:0]=1 | wpw[9:6]=4 | weight_sz[19:16] | syn_sz[23:20]=5 |
            //     pot_sz[27:24]=5 | weight_mode[29:28] | has_ada[30]=0
            cfg_write(CFG+32'h3C, 32'd1 | (4<<6) | (cwszc<<16) | (5<<20) | (5<<24) | (cmode<<28));
            cfg_write(CFG+32'h5C, 32'd5);
            cfg_write(CFG+32'h74, ckxl); cfg_write(CFG+32'h7C, ckxs); cfg_write(CFG+32'h84, ckxo);
            cfg_write(CFG+32'h8C, ciszc); cfg_write(CFG+32'h90, ctszc); cfg_write(CFG+32'h94, cscnt);
        end
    endtask

    task launch;
        begin
            @(negedge clk); start_new_block_i=1;
            @(negedge clk); start_new_block_i=0;
            `VT_WAIT_FINISH(acc_finished_o, 4000)
            stall_en=0; repeat (40) @(posedge clk); #1;
        end
    endtask

    integer ckj;
    task run_pair;
        input [255:0] tag;
        begin
            compute_golden; configure;
            setup_mem; stall_en=0; launch;
            for (ckj=0; ckj<nout; ckj=ckj+1) begin
                check_eq($signed(syn_sram[SYN_BASE+ckj]), syn_acc[ckj], {tag, " syn"});
                ref_syn[ckj] = syn_sram[SYN_BASE+ckj];
            end
            setup_mem; stall_pct=35; stall_en=1; launch;
            for (ckj=0; ckj<nout; ckj=ckj+1)
                check_eq_u(syn_sram[SYN_BASE+ckj], ref_syn[ckj], {tag, " syn(stall)"});
        end
    endtask

    integer i, t;
    task rand_acts;
        begin for (i=0;i<nin;i=i+1) act[i] = ($urandom_range(0,3) != 0) ? 1 : 0; end
    endtask

    task scn_full;
        input integer wszb; input [2:0] wszc; input [255:0] tag;
        begin
            cmode=0; cwszb=wszb; cwszc=wszc;
            cinx=4; coutx=4; nin=4; nout=4;
            crpn=(nout*wszb+31)/32; if (crpn<1) crpn=1;
            ckxl=1;ckxs=1;ckxo=0; ciszc=0;ctszc=0;cscnt=0;
            rand_acts;
            for (i=0;i<nin;i=i+1) for (t=0;t<nout;t=t+1)
                wfull[i*32+t] = sgnext($urandom_range(0,(1<<wszb)-1), wszb);
            run_pair(tag);
        end
    endtask

    task scn_sparse;
        input integer iszb, wszb, tszb; input [2:0] iszc, wszc, tszc;
        input integer scnt; input [255:0] tag;
        begin
            cmode=1; cwszb=wszb; cwszc=wszc;
            cinx=4; coutx=4; nin=4; nout=4;
            crpn=1; ciszb=iszb; cwszb_sp=wszb; ctszb=tszb;
            ciszc=iszc; ctszc=tszc; cscnt=scnt; ckxl=1;ckxs=1;ckxo=0;
            rand_acts;
            for (i=0;i<nin;i=i+1) for (t=0;t<scnt;t=t+1) begin
                sidx[i*16+t] = $urandom_range(0, nout-1);
                swgt[i*16+t] = sgnext($urandom_range(0,(1<<wszb)-1), wszb);
            end
            run_pair(tag);
        end
    endtask

    task scn_conv;
        input integer wszb; input [2:0] wszc;
        input integer kxl,kxs,kxo, inx,outx; input [255:0] tag;
        begin
            cmode=2; cwszb=wszb; cwszc=wszc;
            cinx=inx; coutx=outx; nin=inx; nout=outx;
            crpn=1; ckxl=kxl;ckxs=kxs;ckxo=kxo; ciszc=0;ctszc=0;cscnt=0;
            rand_acts;
            for (i=0;i<kxl;i=i+1) kern[i] = sgnext($urandom_range(0,(1<<wszb)-1), wszb);
            run_pair(tag);
        end
    endtask

    integer pj;
    task gap_probe;
        begin
            cmode=0; cwszb=8; cwszc=3'b011;
            cinx=4; coutx=4; nin=4; nout=4; crpn=1;
            ckxl=1;ckxs=1;ckxo=0; ciszc=0;ctszc=0;cscnt=0;
            act[0]=1; act[1]=1; act[2]=0; act[3]=1;       // input 2 GAP
            for (pj=0;pj<4;pj=pj+1) begin
                wfull[0*32+pj]=10+pj; wfull[1*32+pj]=20+pj;
                wfull[2*32+pj]=30+pj; wfull[3*32+pj]=40+pj;
            end
            compute_golden; configure; setup_mem; stall_en=0; launch;
            for (pj=0;pj<nout;pj=pj+1)
                check_eq($signed(syn_sram[SYN_BASE+pj]), syn_acc[pj], "F9 gap_probe syn[grid-index]");
        end
    endtask

    integer rep;
    initial begin
        verif_errors=0; verif_checks=0; stall_en=0; stall_pct=0;
        weight_wait=0; act_wait=0; syn_wait=0;
        weight_mem_data_i=0; act_mem_data_i=0; syn_curr_mem_data_i=0;
        reset=1; sys_req_i=0; start_new_block_i=0;
        repeat (5) @(posedge clk); @(negedge clk); reset=0; repeat(2) @(posedge clk);

        $display("=== tb_acc_fmiSnn_stress (fmiSnnAcc full-chip, skip_neuron, 1-D spike) ===");
        void'($urandom(32'hF1_5417));

        for (rep=0; rep<3; rep=rep+1) begin
            scn_full(1,  3'b000, "F_1b");   scn_full(2,  3'b001, "F_2b");
            scn_full(4,  3'b010, "F_4b");   scn_full(8,  3'b011, "F_8b");
            scn_full(16, 3'b100, "F_16b");  scn_full(32, 3'b101, "F_32b");
            scn_sparse(4,4,8,  3'b010,3'b010,3'b011, 4, "S_i4w4");
            scn_sparse(8,8,16, 3'b011,3'b011,3'b100, 2, "S_i8w8");
            scn_sparse(2,2,4,  3'b001,3'b001,3'b010, 8, "S_i2w2");
            scn_conv(8, 3'b011, 1,1,0, 4,4, "C_1x1");
            scn_conv(8, 3'b011, 3,1,1, 4,4, "C_1x3");
            scn_conv(8, 3'b011, 3,2,1, 4,2, "C_strd2");
        end
        gap_probe;

        `VERIF_EPILOGUE("tb_acc_fmiSnn_stress")
    end

    `VERIF_WATCHDOG(80000000)

endmodule
