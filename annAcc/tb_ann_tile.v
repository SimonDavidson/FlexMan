// SPDX-License-Identifier: MIT
// =============================================================================
// tb_ann_tile -- bit-exact + cycle test of the NLANE output-stationary dense
// MAC engine (PROTOTYPE), full ann_processor chip, SP only (sp_skip_neuron=1).
//
// Authors      : Simon Davidson & Claude
// Created      : 2026-11-21
// Last modified: 2026-11-21
//
// Defines: NLANE (1,2,4), MACB (MAC_ACT_BITS), RR (REG_RETURN), NSCEN (random
// scenarios), SEED, STALLPCT.
// Oracle: TB golden  syn_final[j] = syn_init[j] + sum_i act[i]*W[i][j]  (32-bit
// wrap), for random 8/16-bit weights, 8/16-bit signed/unsigned activations with
// gaps, random non-zero initial syn_curr (= bias seed / earlier-task partial
// sum), a SECOND accumulating task into the same buffer, the real bias-preload
// engine (np_bias_en), and a stalled re-run. Per-scenario CRC lines let two
// builds (NLANE=1 vs NLANE=N) be diffed for bit-identity.
// =============================================================================
`timescale 1ns/1ps
`include "../shared/constants.v"
`ifndef NLANE
 `define NLANE 4
`endif
`ifndef MACB
 `define MACB 32
`endif
`ifndef RR
 `define RR 0
`endif
`ifndef NSCEN
 `define NSCEN 60
`endif
`ifndef SEED
 `define SEED 32'h0A5417
`endif
`ifndef STALLPCT
 `define STALLPCT 35
`endif
`ifdef REF
 `define NLPARAMS
`else
 `define NLPARAMS , .NLANE(`NLANE), .MAC_ACT_BITS(`MACB)
`endif
`define TGT_ACC_ID 'h0
`define TGT_CONFIG_BASE_ADDR 32'hFFFFFFFF
`define NUM_TIMESTEPS 32
`define X_INPUT_SZ 8
`define Y_INPUT_SZ 8
`define X_OUTPUT_SZ 8
`define Y_OUTPUT_SZ 8
`define X_KERNEL_SZ 3
`define Y_KERNEL_SZ 3
`define X_STEP_SZ   3
`define Y_STEP_SZ   3
`define ELEMS_PER_ROW   4
`define ROWS_PER_NEURON 8
`define ELEM_SZ 8
`define WEIGHT_SLICE_SZ 5
`define WEIGHT_IDX_SZ 10
`define WEIGHT_DATA_IDX_SZ 5
`define ACT_SLICE_SZ 5
`define ACT_DATA_IDX_SZ 10
`define SYN_CURR_IDX_SZ 10
`define SYN_CURR_DATA_IDX_SZ 5
`define SYN_CURR_SLICE_SZ 3
`define SYN_CURR_SLICE_BITS 32
`define BIAS_CURR_IDX_SZ 10
`define BIAS_CURR_DATA_IDX_SZ 5
`define BIAS_CURR_SLICE_SZ 3
`define BIAS_CURR_SLICE_BITS 8
`define NP_LUT_IDX_SZ 8
`define NP_LUT_DATA_IDX_SZ 5
`define NP_LUT_SLICE_SZ 3
`define NP_LUT_SLICE_BITS 8
`define POT_SLICE_SZ 3
`define POT_SLICE_BITS 32

module tb_ann_tile;
    localparam CLK_PERIOD = 10;
    localparam integer ACT_BASE    = 0;
    localparam integer BIAS_BASE   = 8;
    localparam integer WEIGHT_BASE = 16;
    localparam integer SYN_BASE    = 40;
    localparam [31:0]  CFG         = 32'hFFFF_0000;

    reg clk, reset;
    initial clk = 1'b0;
    always #(CLK_PERIOD/2) clk = ~clk;

    integer verif_errors, verif_checks, verif_to;
    `include "../verif/checks.vh"

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
    wire bias_curr_mem_rd_o;  wire [`ADDR_SIZE-1:0] bias_curr_mem_addr_o;
    wire thresh_mem_rd_o;     wire [`ADDR_SIZE-1:0] thresh_mem_addr_o;
    wire pot_mem_wr_o, pot_mem_rd_o;  wire [`ADDR_SIZE-1:0] pot_mem_addr_o;
    wire [`POT_BITS-1:0] pot_mem_data_o;
    wire spike_mem_wr_o;  wire [`ADDR_SIZE-1:0] spike_mem_addr_o;  wire [`ACT_BITS-1:0] spike_mem_data_o;
    reg [`WTD_BITS-1:0] weight_mem_data_i;
    reg [`ACT_BITS-1:0] act_mem_data_i;
    reg [`POT_BITS-1:0] syn_curr_mem_data_i;
    reg [`WTD_BITS-1:0] bias_mem_data_r;
    reg weight_wait, act_wait, syn_wait;
    wire [`WTD_BITS-1:0] thresh_mem_data_i = 0;
    wire [`POT_BITS-1:0] pot_mem_data_i = 0;
    wire [`WTD_BITS-1:0] bias_curr_mem_data_i = bias_mem_data_r;

    ann_processor # (
    .TGT_ACC_ID(`TGT_ACC_ID), .TGT_CONFIG_BASE_ADDR(`TGT_CONFIG_BASE_ADDR),
    .SP_NUM_TIMESTEPS(`NUM_TIMESTEPS),
    .SP_X_INPUT_SZ(`X_INPUT_SZ), .SP_Y_INPUT_SZ(`Y_INPUT_SZ),
    .SP_X_OUTPUT_SZ(`X_OUTPUT_SZ), .SP_Y_OUTPUT_SZ(`Y_OUTPUT_SZ),
    .SP_X_KERNEL_SZ(`X_KERNEL_SZ), .SP_Y_KERNEL_SZ(`Y_KERNEL_SZ),
    .SP_X_KERNEL_OFF_SZ(`X_STEP_SZ), .SP_Y_KERNEL_OFF_SZ(`Y_STEP_SZ),
    .SP_X_STEP_SZ(`X_STEP_SZ), .SP_Y_STEP_SZ(`Y_STEP_SZ),
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
    .NP_LUT_IDX_SZ(`NP_LUT_IDX_SZ), .NP_LUT_DATA_IDX_SZ(`NP_LUT_DATA_IDX_SZ),
    .NP_LUT_SLICE_SZ(`NP_LUT_SLICE_SZ), .NP_LUT_SLICE_BITS(`NP_LUT_SLICE_BITS),
    .NP_POT_SLICE_SZ(`POT_SLICE_SZ), .NP_POT_SLICE_BITS(`POT_SLICE_BITS),
    .MEM_ADDR_BITS(`ADDR_SIZE),
    .REG_RETURN(`RR) `NLPARAMS
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
        .bias_curr_mem_rd_o(bias_curr_mem_rd_o), .bias_curr_mem_wait_i(1'b0),
        .bias_curr_mem_addr_o(bias_curr_mem_addr_o), .bias_curr_mem_data_i(bias_curr_mem_data_i),
        .thresh_mem_rd_o(thresh_mem_rd_o), .thresh_mem_wait_i(1'b0),
        .thresh_mem_addr_o(thresh_mem_addr_o), .thresh_mem_data_i(thresh_mem_data_i),
        .pot_mem_wr_o(pot_mem_wr_o), .pot_mem_rd_o(pot_mem_rd_o),
        .pot_mem_wait_i(1'b0), .pot_mem_addr_o(pot_mem_addr_o),
        .pot_mem_data_o(pot_mem_data_o), .pot_mem_data_i(pot_mem_data_i),
        .spike_mem_wr_o(spike_mem_wr_o), .spike_mem_wait_i(1'b0),
        .spike_mem_addr_o(spike_mem_addr_o), .spike_mem_data_o(spike_mem_data_o)
    );

    // ---- memories (4096 deep; request honoured only when ~wait) ----
    reg [31:0] act_sram    [0:4095];
    reg [31:0] weight_sram [0:4095];
    reg [31:0] syn_sram    [0:4095];
    reg [31:0] bias_sram   [0:4095];
    always @(posedge clk) if (act_mem_req_o & ~act_wait)       act_mem_data_i    <= act_sram[act_mem_addr_o[11:0]];
    always @(posedge clk) if (weight_mem_rd_o & ~weight_wait)  weight_mem_data_i <= weight_sram[weight_mem_addr_o[11:0]];
    always @(posedge clk) begin
        if (syn_curr_mem_rd_o & ~syn_wait) syn_curr_mem_data_i <= syn_sram[syn_curr_mem_addr_o[11:0]];
        if (syn_curr_mem_wr_o & ~syn_wait) syn_sram[syn_curr_mem_addr_o[11:0]] <= syn_curr_mem_data_o;
    end
    always @(posedge clk) if (bias_curr_mem_rd_o) bias_mem_data_r <= bias_sram[bias_curr_mem_addr_o[11:0]];

    reg stall_en; integer stall_pct;
    always @(posedge clk) begin
        if (!stall_en) begin weight_wait<=0; act_wait<=0; syn_wait<=0; end
        else begin
            weight_wait <= ($urandom_range(99) < stall_pct);
            act_wait    <= ($urandom_range(99) < stall_pct);
            syn_wait    <= ($urandom_range(99) < (stall_pct + 20));
        end
    end

    task cfg_write; input [31:0] addr, data; begin
        @(negedge clk); sys_req_i=1; sys_addr_i=addr; sys_data_i=data; #1;
        if (!sys_ack_o) begin $display("FAIL cfg_write: no ACK for 0x%08h", addr); verif_errors=verif_errors+1; end
        @(negedge clk); sys_req_i=0; end endtask

    // ------------------------------------------------ scenario state / golden
    integer nin, nout, wb, ab, asgn, bias_en, dens, rows;
    integer act_v   [0:255];
    integer w_v     [0:255*64-1];
    reg signed [63:0] syn_init [0:255];
    reg signed [63:0] bias_v   [0:255];
    reg [31:0] gold [0:255];
    integer gi, gj;
    reg signed [63:0] ga;

    function signed [31:0] sgnext; input [31:0] v; input integer b;
        begin sgnext = $signed(v << (32-b)) >>> (32-b); end endfunction

    task pack_w; input integer i, j; input integer val; integer p, wd, bt; begin
        p = wb*j; wd = p/32; bt = p%32;
        weight_sram[WEIGHT_BASE + i*rows + wd] = weight_sram[WEIGHT_BASE + i*rows + wd] | ((val & ((1<<wb)-1)) << bt);
    end endtask

    integer cycles_last;
    task gen_data; integer i, j; begin
        for (i=0;i<nin;i=i+1) begin
            if ($urandom_range(99) < dens) act_v[i] = 0;
            else if (asgn) act_v[i] = sgnext($urandom_range(0,(1<<ab)-1), ab);
            else           act_v[i] = $urandom_range(1,(1<<ab)-1);
            for (j=0;j<nout;j=j+1) w_v[i*64+j] = sgnext($urandom, wb);
        end
    end endtask
    task load_data; integer i, j; begin
        for (i=0;i<4096;i=i+1) begin act_sram[i]=0; weight_sram[i]=0; end
        for (i=0;i<nin;i=i+1) begin
            act_sram[ACT_BASE + (i*ab)/32] = act_sram[ACT_BASE + (i*ab)/32] | ((act_v[i] & ((1<<ab)-1)) << ((i*ab)%32));
            for (j=0;j<nout;j=j+1) pack_w(i, j, w_v[i*64+j]);
        end
    end endtask
    task add_golden; begin
        for (gj=0;gj<nout;gj=gj+1) begin
            ga = syn_init[gj];
            for (gi=0;gi<nin;gi=gi+1) ga = ga + (act_v[gi] * w_v[gi*64+gj]);
            syn_init[gj] = ga;           // low 32 bits are what the hardware keeps
        end
    end endtask

    task configure;
        integer m0; begin
        cfg_write(CFG+32'h00, ACT_BASE);     cfg_write(CFG+32'h04, WEIGHT_BASE);
        cfg_write(CFG+32'h08, SYN_BASE);     cfg_write(CFG+32'h0C, BIAS_BASE);
        cfg_write(CFG+32'h10, 40);           cfg_write(CFG+32'h14, 50);
        cfg_write(CFG+32'h18, 60);           cfg_write(CFG+32'h1C, 32'h8000_0000);
        cfg_write(CFG+32'h20, 32'h8000_0000);
        cfg_write(CFG+32'h24, (1<<16) | nin);
        cfg_write(CFG+32'h28, (1<<16) | nout);
        cfg_write(CFG+32'h2C, ((nout-1)<<16) | rows);
        cfg_write(CFG+32'h30, 32'd1);
        // M0: weight_sz | syn_curr_sz=5 | lut_out_sz=3 | pot_sz=5 | act_sz | thresh_op=0 |
        //     weight_mode=0 | [28] act_signed | [31] np_bias_en
        m0 = ((wb==8)?3:4) | (5<<4) | (3<<8) | (5<<12) | (((ab==8)?3:4)<<16)
             | (asgn<<28) | (bias_en<<31);
        cfg_write(CFG+32'h34, m0);
        cfg_write(CFG+32'h38, 32'd1 | (((wb==8)?4:2)<<6));   // skip_neuron=1, bin_point 0
        cfg_write(CFG+32'h5C, 32'd5);
        cfg_write(CFG+32'h74, 1); cfg_write(CFG+32'h78, 1); cfg_write(CFG+32'h7C, 1);
        cfg_write(CFG+32'h80, 1); cfg_write(CFG+32'h84, 0); cfg_write(CFG+32'h88, 0);
    end endtask

    integer t0, to;
    task launch; begin
        @(negedge clk); start_new_block_i=1; t0=$time;
        @(negedge clk); start_new_block_i=0;
        to = 200000;
        while (!acc_finished_o && to>0) begin @(posedge clk); #1; to=to-1; end
        if (to==0) begin $display("FAIL timeout waiting for acc_finished (NLANE=%0d nin=%0d nout=%0d)", `NLANE, nin, nout); verif_errors=verif_errors+1; end
        cycles_last = ($time - t0)/CLK_PERIOD;
        stall_en=0; repeat (20) @(posedge clk); #1;
    end endtask

    reg [63:0] crc;
    integer ck;
    task check_syn; input [255:0] tag; begin
        for (ck=0;ck<nout;ck=ck+1) begin
            check_eq_u(syn_sram[SYN_BASE+ck], syn_init[ck] & 64'hFFFFFFFF, tag);
            crc = {crc[62:0],crc[63]} ^ {32'b0, syn_sram[SYN_BASE+ck]};
        end
    end endtask

    integer sc, ii;
    task run_scenario; input integer wb_i, ab_i, asgn_i, bias_i, nin_i, nout_i, dens_i; begin
        wb=wb_i; ab=ab_i; asgn=asgn_i; bias_en=bias_i; nin=nin_i; nout=nout_i; dens=dens_i;
        rows = (nout*wb+31)/32; if (rows<1) rows=1;
        // -------- task 1 (no stall), non-zero initial syn_curr
        gen_data; load_data;
        for (ii=0;ii<nout;ii=ii+1) begin
            syn_init[ii] = $signed($urandom);
            syn_sram[SYN_BASE+ii] = syn_init[ii][31:0];
            bias_v[ii] = bias_en ? $signed($urandom_range(0,255)) - 100 : 0;
            bias_sram[BIAS_BASE+ii] = bias_v[ii][31:0];
        end
        if (bias_en) for (ii=0;ii<nout;ii=ii+1) syn_init[ii] = syn_init[ii] + bias_v[ii];
        add_golden; configure; stall_en=0; launch; check_syn("T1 syn");
        $display("CYC nl=%0d nin=%0d nout=%0d wb=%0d ab=%0d macs=%0d cycles=%0d", `NLANE, nin, nout, wb, ab, nin*nout, cycles_last);
        // -------- task 2 accumulates into the same buffer (no reset of syn)
        gen_data; load_data; bias_en=0; add_golden; configure; launch; check_syn("T2 acc");
        // -------- stalled re-run of task 2's data on a fresh seed buffer
        for (ii=0;ii<nout;ii=ii+1) begin
            syn_init[ii] = $signed($urandom); syn_sram[SYN_BASE+ii] = syn_init[ii][31:0]; end
        add_golden; stall_pct=`STALLPCT; stall_en=1; launch; check_syn("T3 stall");
    end endtask

    integer rnin, rnout, rw, ra;
    initial begin
        verif_errors=0; verif_checks=0; stall_en=0; stall_pct=0; crc=64'h1;
        weight_wait=0; act_wait=0; syn_wait=0; weight_mem_data_i=0; act_mem_data_i=0; syn_curr_mem_data_i=0; bias_mem_data_r=0;
        reset=1; sys_req_i=0; start_new_block_i=0;
        repeat (5) @(posedge clk); @(negedge clk); reset=0; repeat(2) @(posedge clk);
        $display("=== tb_ann_tile NLANE=%0d MACB=%0d RR=%0d ===", `NLANE, `MACB, `RR);
        void'($urandom(`SEED));
        // ---- directed edge cases: tail tiles, tiny fan-in/out, single output
        run_scenario(8, 8, 0, 0,  1,  1, 0);
        run_scenario(8, 8, 0, 0,  1,  4, 0);
        run_scenario(8, 8, 1, 0,  2,  3, 0);
        run_scenario(8, 8, 0, 0,  3,  5, 50);
        run_scenario(8, 8, 1, 0,  5,  7, 0);
        run_scenario(8, 8, 0, 0, 16, 32, 25);
        run_scenario(8, 8, 1, 0, 17, 33, 25);
        run_scenario(16, 8, 1, 0, 16, 32, 0);
        run_scenario(16, 16, 1, 0, 9, 13, 0);
        run_scenario(8, 8, 1, 1, 12, 20, 10);          // real bias-preload engine
        // ---- random sweep
        for (sc=0; sc<`NSCEN; sc=sc+1) begin
            rnin  = $urandom_range(1, 48);
            rnout = $urandom_range(1, 48);
            rw    = ($urandom_range(0,2)==0) ? 16 : 8;
            ra    = ($urandom_range(0,3)==0) ? 16 : 8;
            run_scenario(rw, ra, $urandom_range(0,1), ($urandom_range(0,4)==0), rnin, rnout, $urandom_range(0,60));
        end
        $display("CRC %h", crc);
        `VERIF_EPILOGUE("tb_ann_tile")
    end
    `VERIF_WATCHDOG(400000000)
endmodule
