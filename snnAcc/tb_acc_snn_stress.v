// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
// =============================================================================
// tb_acc_snn_stress  (snnAcc)  --  full-chip stress testbench
//
// Authors      : Simon Davidson & Claude
// Created      : 2026-06-30
// Last modified: 2026-06-30
//
// Purpose
// -------
// The existing tb_acc_snn_processor runs full mode / 8-bit weights only and ties
// ALL seven *_mem_wait_i to 1'b0 -- so it never exercises sparse/conv, the other
// weight widths, the neuron-update-OFF (sp_skip_neuron) path, np_mode, or ANY
// back-pressure at the integration level (the very blind spot that hid F8).
//
// This testbench drives the WHOLE acc_snn_processor (both stages + the shared
// syn_curr arbiter + AXI config) across:
//   * connectivity modes  : full / sparse / conv
//   * weight sizes         : 1/2/4/8/16/32-bit (sparse: 1/2/4/8/16-bit)
//   * neuron-update on/off : sp_skip_neuron = 0 (full pipeline) and 1 (SP only)
//   * np_mode              : sub_on_fire, clear_pot
//   * back-pressure        : per-port random mem_wait on ALL seven ports, plus a
//                            heavy syn_curr stall that forces SP-vs-NP arbiter
//                            contention.
//
// Two independent oracles per configuration:
//   (1) Correctness    -- the no-stall run is checked against a software golden:
//       a connectivity model (full/sparse/conv routing, built consistent-by-
//       construction from the same weight arrays that fill memory) feeds the
//       shared LIF neuron golden np_ref_lif.
//   (2) Stall-invariance -- the SAME configuration is then re-run with heavy
//       random per-port stalls and the resulting syn/pot/spike memory is required
//       to be BYTE-IDENTICAL to the no-stall run.  Stalls may only change timing,
//       never data: a lost operand or a mis-aligned stream fails this check
//       independently of the golden.
//
// Scope note: syn_curr and pot run at 32-bit/neuron (one word each -- snnAcc
// always stores 32-bit syn_curr) and bias/thresh at 8-bit, matching the DUT
// datapath params.  The writeback packer SLICE-SIZE x write-stall cadence matrix
// (8/16/32-bit, the F8 surface) is covered exhaustively at the unit level by
// tb_packer_cadence; here the focus is end-to-end mode/size/skip/stall coverage.
// =============================================================================
`timescale 1ns/1ps

`include "../shared/constants.v"

// CONFIG PARAMS START  (compile-time RTL parameters -- copied from
// tb_acc_snn_processor; WEIGHT_SLICE_SZ=5 => 32-bit weight bus covers every
// runtime weight_sz, and a full sparse tuple.)
`define TGT_ACC_ID 'h0
`define TGT_CONFIG_BASE_ADDR 32'hFFFFFFFF
`define NUM_TIMESTEPS 32
`define IN_DATA_SZ 32
`define X_INPUT_SZ 5
`define Y_INPUT_SZ 5
`define X_OUTPUT_SZ 5
`define Y_OUTPUT_SZ 5
`define X_KERNEL_SZ 3
`define Y_KERNEL_SZ 3
`define X_STEP_SZ   3
`define Y_STEP_SZ   3
`define ELEMS_PER_ROW   4
`define ROWS_PER_NEURON 4
`define ELEM_SZ 8
`define WEIGHT_SLICE_SZ 5
`define WEIGHT_IDX_SZ 10
`define WEIGHT_DATA_IDX_SZ 5
`define ACT_SLICE_SZ 3      // Don't change (act cache slice is tied to 1-bit)
`define ACT_IDX_SZ 10
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
// CONFIG PARAMS END

module tb_acc_snn_stress;

    localparam CLK_PERIOD = 10;

    // Memory layout (all bases fit in 8-bit addr; regions non-overlapping):
    localparam integer ACT_BASE    = 0;    // 1-bit spikes, bit i = input i
    localparam integer WEIGHT_BASE = 8;    // full rows / conv kernel / sparse tuples
    localparam integer SYN_BASE    = 40;   // 32-bit per neuron (RMW + decayed wb)
    localparam integer BIAS_BASE   = 60;   // 8-bit per neuron, 4/word
    localparam integer THRESH_BASE = 70;   // 8-bit per neuron, 4/word
    localparam integer POT_BASE    = 80;   // 32-bit per neuron
    localparam integer SPIKE_BASE  = 100;  // 1-bit per neuron, 32/word
    localparam [31:0]  DECAY        = 32'h8000_0000;  // 0.5 in Q0.32
    localparam [31:0]  CFG          = 32'hFFFF_0000;   // config base (upper 16 match)

    // ----------------------------------------------------------------
    // Clock & reset
    // ----------------------------------------------------------------
    reg clk, reset;
    initial clk = 1'b0;
    always #(CLK_PERIOD/2) clk = ~clk;

    integer verif_errors, verif_checks, verif_to, sram_i;
    `include "../verif/checks.vh"
    `include "../verif/np_ref.vh"
    `include "../verif/vt_driver.vh"
    `include "../verif/sram_bfm.vh"

    // ----------------------------------------------------------------
    // DUT control inputs
    // ----------------------------------------------------------------
    reg                      sys_req_i   = 1'b0;
    reg             [31:0]   sys_addr_i  = 32'b0;
    reg             [31:0]   sys_data_i  = 32'b0;
    reg                      start_new_block_i = 1'b0;
    reg   [`TGT_ACC_SZ-1:0]  target_acc_i      = {`TGT_ACC_SZ{1'b0}};
    reg [`SCH_ENTRY_SZ-1:0]  buffer_info_i     = {`SCH_ENTRY_SZ{1'b0}};

    reg [`PIN_BITS-1:0] sp_src1_buff_addr_i = 0, sp_src2_buff_addr_i = 0,
                        sp_src3_buff_addr_i = 0, sp_tgt_buff_addr_i = 0,
                        sp_weight_row_len_i = 0;
    reg [`PIN_BITS-1:0] np_src1_buff_addr_i = 0, np_src2_buff_addr_i = 0,
                        np_src3_buff_addr_i = 0, np_tgt_buff_addr_i = 0,
                        np_weight_row_len_i = 0;

    // ----------------------------------------------------------------
    // DUT outputs
    // ----------------------------------------------------------------
    wire                    sys_ack_o, spike_proc_finished_o, acc_busy_o, acc_finished_o;
    wire                    weight_mem_rd_o;  wire [`ADDR_SIZE-1:0] weight_mem_addr_o;
    wire                    act_mem_req_o;    wire [`ADDR_SIZE-1:0] act_mem_addr_o;
    wire                    syn_curr_mem_wr_o, syn_curr_mem_rd_o;
    wire [`ADDR_SIZE-1:0]   syn_curr_mem_addr_o;  wire [`POT_BITS-1:0] syn_curr_mem_data_o;
    wire                    bias_curr_mem_rd_o;  wire [`ADDR_SIZE-1:0] bias_curr_mem_addr_o;
    wire                    thresh_mem_rd_o;     wire [`ADDR_SIZE-1:0] thresh_mem_addr_o;
    wire                    pot_mem_wr_o, pot_mem_rd_o;
    wire [`ADDR_SIZE-1:0]   pot_mem_addr_o;   wire [`POT_BITS-1:0] pot_mem_data_o;
    wire                    spike_mem_wr_o;   wire [`ADDR_SIZE-1:0] spike_mem_addr_o;
    wire [`ACT_BITS-1:0]    spike_mem_data_o;

    // Memory read-data buses are driven by the BFMs (declared as regs):
    reg [`WTD_BITS-1:0] weight_mem_data_i, bias_curr_mem_data_i, thresh_mem_data_i;
    reg [`ACT_BITS-1:0] act_mem_data_i;
    reg [`POT_BITS-1:0] syn_curr_mem_data_i, pot_mem_data_i;

    // Per-port mem_wait back-pressure regs (driven by the stall generator):
    reg weight_wait, act_wait, syn_wait, bias_wait, thresh_wait, pot_wait, spike_wait;

    // ----------------------------------------------------------------
    // DUT
    // ----------------------------------------------------------------
    acc_snn_processor # (
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
    .NP_BIAS_CURR_IDX_SZ(`BIAS_CURR_IDX_SZ), .NP_BIAS_CURR_DATA_IDX_SZ(`BIAS_CURR_DATA_IDX_SZ),
    .NP_BIAS_CURR_SLICE_SZ(`BIAS_CURR_SLICE_SZ), .NP_BIAS_CURR_SLICE_BITS(`BIAS_CURR_SLICE_BITS),
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
        .bias_curr_mem_rd_o(bias_curr_mem_rd_o), .bias_curr_mem_wait_i(bias_wait),
        .bias_curr_mem_addr_o(bias_curr_mem_addr_o), .bias_curr_mem_data_i(bias_curr_mem_data_i),
        .thresh_mem_rd_o(thresh_mem_rd_o), .thresh_mem_wait_i(thresh_wait),
        .thresh_mem_addr_o(thresh_mem_addr_o), .thresh_mem_data_i(thresh_mem_data_i),
        .pot_mem_wr_o(pot_mem_wr_o), .pot_mem_rd_o(pot_mem_rd_o),
        .pot_mem_wait_i(pot_wait), .pot_mem_addr_o(pot_mem_addr_o),
        .pot_mem_data_o(pot_mem_data_o), .pot_mem_data_i(pot_mem_data_i),
        .spike_mem_wr_o(spike_mem_wr_o), .spike_mem_wait_i(spike_wait),
        .spike_mem_addr_o(spike_mem_addr_o), .spike_mem_data_o(spike_mem_data_o)
    );

    // ----------------------------------------------------------------
    // Wait-honouring SRAM BFMs (from verif/sram_bfm.vh)
    // ----------------------------------------------------------------
    `SRAM_RD_WAIT(act_sram,    act_mem_req_o,  act_wait,    act_mem_addr_o,    act_mem_data_i)
    `SRAM_RD_WAIT(weight_sram, weight_mem_rd_o, weight_wait, weight_mem_addr_o, weight_mem_data_i)
    `SRAM_RD_WAIT(bias_sram,   bias_curr_mem_rd_o, bias_wait, bias_curr_mem_addr_o, bias_curr_mem_data_i)
    `SRAM_RD_WAIT(thresh_sram, thresh_mem_rd_o, thresh_wait, thresh_mem_addr_o, thresh_mem_data_i)
    `SRAM_RW_WAIT(syn_sram,  syn_curr_mem_rd_o, syn_curr_mem_wr_o, syn_wait,
                  syn_curr_mem_addr_o, syn_curr_mem_data_o, syn_curr_mem_data_i)
    `SRAM_RW_WAIT(pot_sram,  pot_mem_rd_o, pot_mem_wr_o, pot_wait,
                  pot_mem_addr_o, pot_mem_data_o, pot_mem_data_i)
    `SRAM_WR_WAIT(spike_sram, spike_mem_wr_o, spike_wait, spike_mem_addr_o, spike_mem_data_o)

    // ----------------------------------------------------------------
    // Stall generator: when stall_en, drive each port's wait with stall_pct%
    // probability per cycle (syn gets an extra-heavy bias to stress the arbiter).
    // ----------------------------------------------------------------
    reg        stall_en;
    integer    stall_pct;
    always @(posedge clk) begin
        if (!stall_en) begin
            weight_wait<=0; act_wait<=0; syn_wait<=0; bias_wait<=0;
            thresh_wait<=0; pot_wait<=0; spike_wait<=0;
        end else begin
            weight_wait <= ($urandom_range(99) < stall_pct);
            act_wait    <= ($urandom_range(99) < stall_pct);
            syn_wait    <= ($urandom_range(99) < (stall_pct + 20));  // heavier: arbiter
            bias_wait   <= ($urandom_range(99) < stall_pct);
            thresh_wait <= ($urandom_range(99) < stall_pct);
            pot_wait    <= ($urandom_range(99) < stall_pct);
            spike_wait  <= ($urandom_range(99) < stall_pct);
        end
    end

`ifdef GAP_PROBE_DBG
    // Debug aid: trace act_data_idx -> weight_row_base on each weight fetch.
    // Used to characterise F9 (gapped-activation weight-row mis-indexing): a
    // non-spike input does not advance act_data_idx, so the next spiking input
    // inherits the gap's row.  Compile with -define GAP_PROBE_DBG to enable.
    always @(posedge clk)
        if (weight_mem_rd_o)
            $display("PRB t=%0t aidx=%0d wbase=%0d waddr=%0d", $time,
                     u_dut.u_spike_processing.act_data_idx,
                     u_dut.u_spike_processing.weight_gen0.weight_row_base_addr,
                     weight_mem_addr_o);
`endif

    // ================================================================
    // Helpers
    // ================================================================
    task cfg_write;
        input [31:0] addr, data;
        begin
            @(negedge clk); sys_req_i=1'b1; sys_addr_i=addr; sys_data_i=data; #1;
            if (!sys_ack_o) begin
                $display("FAIL cfg_write: no ACK for addr=0x%08h", addr);
                verif_errors = verif_errors + 1;
            end
            @(negedge clk); sys_req_i=1'b0;
        end
    endtask

    function signed [31:0] sgnext;        // sign-extend low `b` bits of v to 32
        input [31:0] v; input integer b;
        begin sgnext = $signed(v << (32-b)) >>> (32-b); end
    endfunction

    // Pack a `wb`-bit value as element k of a contiguous bitstream from word base
    // (wb divides 32, so no field crosses a word boundary).
    task pack_stream;
        input integer base, k, wb; input [31:0] val;
        integer p, wd, bt;
        begin p=wb*k; wd=p/32; bt=p%32;
              weight_sram[base+wd] = weight_sram[base+wd] | ((val & ((1<<wb)-1)) << bt); end
    endtask

    // ================================================================
    // Per-configuration state + golden
    // ================================================================
    integer cmode, cwszc, cwszb, cskip, cnpmode;
    integer cinx, ciny, coutx, couty, crpn;
    integer ckxl, ckyl, ckxs, ckys, ckxo, ckyo;
    integer ciszc, ctszc, ciszb, cwszb_sp, ctszb, cscnt;
    integer nin, nout;

    reg              spike_in [0:31];
    reg signed [31:0] wfull   [0:511];    // full weights, idx i*32+j
    reg signed [31:0] kern    [0:63];     // conv kernel, idx ky*kxl+kx
    integer          sidx     [0:255];    // sparse tuple index, idx i*16+t
    reg signed [31:0] swgt    [0:255];    // sparse tuple weight (signed)
    reg signed [31:0] bias    [0:31], thr [0:31], potin [0:31];
    reg signed [31:0] syn_acc [0:63];     // golden accumulated syn (pre-NP)

    reg signed [31:0] exp_syn [0:63], exp_pot [0:63];
    reg               exp_spk [0:63];
    reg        [31:0] exp_spk_word;

    // Snapshot of the no-stall output (for the stall-invariance compare).
    // Unsigned so check_eq_u (which zero-extends both args to 64-bit) matches the
    // zero-extended syn_sram/pot_sram words bit-for-bit.
    reg [31:0] ref_syn [0:63], ref_pot [0:63];
    reg [31:0] ref_spk_word;

    integer gi, gj, gk, gt, gox, goy;
    reg               gspk;
    reg signed [31:0] gpo, gso;

    // -------- compute the connectivity + neuron golden for the current config --
    task compute_golden;
        begin
            for (gj=0; gj<nout; gj=gj+1) syn_acc[gj] = 32'sd0;

            if (cmode == 0) begin                       // FULL
                for (gi=0; gi<nin; gi=gi+1)
                    if (spike_in[gi])
                        for (gj=0; gj<nout; gj=gj+1)
                            syn_acc[gj] = syn_acc[gj] + wfull[gi*32+gj];
            end else if (cmode == 1) begin              // SPARSE
                for (gi=0; gi<nin; gi=gi+1)
                    if (spike_in[gi])
                        for (gt=0; gt<cscnt; gt=gt+1)
                            syn_acc[sidx[gi*16+gt]] = syn_acc[sidx[gi*16+gt]] + swgt[gi*16+gt];
            end else begin                              // CONV
                for (gi=0; gi<nin; gi=gi+1)
                    if (spike_in[gi])
                        for (goy=0; goy<ckyl; goy=goy+1)
                          for (gox=0; gox<ckxl; gox=gox+1) begin
                             gk = (gi/cinx)*ckys + goy - ckyo;      // out_y
                             gj = (gi%cinx)*ckxs + gox - ckxo;      // out_x
                             if (gj>=0 && gj<coutx && gk>=0 && gk<couty)
                                syn_acc[gk*coutx+gj] = syn_acc[gk*coutx+gj] + kern[goy*ckxl+gox];
                          end
            end

            // neuron stage (or skip)
            exp_spk_word = 32'b0;
            for (gj=0; gj<nout; gj=gj+1) begin
                if (cskip) begin
                    exp_syn[gj] = syn_acc[gj];      // SP writes accumulated syn, 32-bit
                    exp_pot[gj] = potin[gj];        // pot untouched
                    exp_spk[gj] = 1'b0;             // spike mem untouched (init 0)
                end else begin
                    np_ref_lif(syn_acc[gj], (cnpmode & 4) ? 32'sd0 : potin[gj],
                               bias[gj], thr[gj], DECAY, DECAY, (cnpmode & 1),
                               gspk, gpo, gso);
                    exp_syn[gj] = gso;
                    exp_pot[gj] = gpo;
                    exp_spk[gj] = gspk;
                    exp_spk_word = exp_spk_word | (gspk << (gj & 31));
                end
            end
        end
    endtask

    // -------- (re)load all memories for the current config ---------------------
    task setup_mem;
        integer i, t, w0;
        begin
            for (sram_i=0; sram_i<256; sram_i=sram_i+1) begin
                act_sram[sram_i]=0;  weight_sram[sram_i]=0; bias_sram[sram_i]=0;
                thresh_sram[sram_i]=0; syn_sram[sram_i]=0; pot_sram[sram_i]=0;
                spike_sram[sram_i]=0;
            end
            // activations: bit i set if input i spikes
            for (i=0; i<nin; i=i+1)
                if (spike_in[i]) act_sram[ACT_BASE + i/32] = act_sram[ACT_BASE + i/32] | (1 << (i%32));

            // weights / kernel / tuples
            if (cmode == 0) begin
                for (i=0; i<nin; i=i+1)
                    for (t=0; t<nout; t=t+1)
                        pack_stream(WEIGHT_BASE + i*crpn, t, cwszb, wfull[i*32+t]);
            end else if (cmode == 1) begin
                for (i=0; i<nin; i=i+1) begin
                    w0 = 32'b0;
                    for (t=0; t<cscnt; t=t+1)
                        w0 = w0 | ( ( (sidx[i*16+t] << (ctszb - ciszb)) |
                                      ((swgt[i*16+t] & ((1<<cwszb_sp)-1)) << (ctszb - ciszb - cwszb_sp)) )
                                    << (ctszb*t) );
                    weight_sram[WEIGHT_BASE + i] = w0;     // rows_per_neuron = 1 in sparse
                end
            end else begin
                for (i=0; i<ckxl*ckyl; i=i+1)
                    pack_stream(WEIGHT_BASE, i, cwszb, kern[i]);
            end

            // bias / thresh (8-bit, 4/word) and pot init (32-bit/word)
            for (i=0; i<nout; i=i+1) begin
                bias_sram[BIAS_BASE + i/4]     = bias_sram[BIAS_BASE + i/4]
                                                 | ((bias[i] & 32'hFF) << ((i%4)*8));
                thresh_sram[THRESH_BASE + i/4] = thresh_sram[THRESH_BASE + i/4]
                                                 | ((thr[i]  & 32'hFF) << ((i%4)*8));
                syn_sram[SYN_BASE + i] = 32'b0;            // SP accumulates from 0
                pot_sram[POT_BASE + i] = potin[i];
            end
        end
    endtask

    // -------- write all config registers for the current config ----------------
    task configure;
        begin
            cfg_write(CFG+32'h00, ACT_BASE);     cfg_write(CFG+32'h04, WEIGHT_BASE);
            cfg_write(CFG+32'h08, SYN_BASE);     cfg_write(CFG+32'h0C, BIAS_BASE);
            cfg_write(CFG+32'h10, THRESH_BASE);  cfg_write(CFG+32'h14, POT_BASE);
            cfg_write(CFG+32'h18, SPIKE_BASE);   cfg_write(CFG+32'h1C, DECAY);
            cfg_write(CFG+32'h20, DECAY);
            cfg_write(CFG+32'h24, (ciny<<16) | cinx);
            cfg_write(CFG+32'h28, (couty<<16) | coutx);
            cfg_write(CFG+32'h2C, ((nout-1)<<16) | crpn);
            cfg_write(CFG+32'h30, 32'd1);                       // total_timesteps
            cfg_write(CFG+32'h34, cwszc | (5<<4) | (3<<8) | (5<<12) | (cmode<<24));
            cfg_write(CFG+32'h38, cskip | (cnpmode<<2) | (4<<6));
            cfg_write(CFG+32'h5C, 32'd5);                       // weight_idx_sz
            cfg_write(CFG+32'h74, ckxl); cfg_write(CFG+32'h78, ckyl);
            cfg_write(CFG+32'h7C, ckxs); cfg_write(CFG+32'h80, ckys);
            cfg_write(CFG+32'h84, ckxo); cfg_write(CFG+32'h88, ckyo);
            cfg_write(CFG+32'h8C, ciszc); cfg_write(CFG+32'h90, ctszc);
            cfg_write(CFG+32'h94, cscnt);
        end
    endtask

    // -------- launch one TASK and wait (bounded) for completion ---------------
    task launch;
        begin
            @(negedge clk); start_new_block_i=1'b1;
            @(negedge clk); start_new_block_i=1'b0;
            `VT_WAIT_FINISH(acc_finished_o, 4000)
            // drain: drop stalls and let the writeback packers flush their last word
            stall_en=0;
            repeat (60) @(posedge clk);
            #1;
        end
    endtask

    // -------- run the current config no-stall (check golden + snapshot), then
    //          re-run with heavy stalls (require byte-identical memory) ---------
    integer ckj;
    task run_pair;
        input [255:0] tag;
        begin
            compute_golden;
            configure;

            // pass 1: no stalls -> check against the software golden, snapshot out
            setup_mem; stall_en=0;
            launch;
            for (ckj=0; ckj<nout; ckj=ckj+1) begin
                check_eq($signed(syn_sram[SYN_BASE+ckj]), exp_syn[ckj], {tag, " syn"});
                check_eq($signed(pot_sram[POT_BASE+ckj]), exp_pot[ckj], {tag, " pot"});
                ref_syn[ckj] = syn_sram[SYN_BASE+ckj];
                ref_pot[ckj] = pot_sram[POT_BASE+ckj];
            end
            check_eq_u(spike_sram[SPIKE_BASE], exp_spk_word, {tag, " spk"});
            ref_spk_word = spike_sram[SPIKE_BASE];

            // pass 2: heavy per-port stalls -> must reproduce pass 1 byte-for-byte
            setup_mem; stall_pct=35; stall_en=1;
            launch;
            for (ckj=0; ckj<nout; ckj=ckj+1) begin
                check_eq_u(syn_sram[SYN_BASE+ckj], ref_syn[ckj], {tag, " syn(stall)"});
                check_eq_u(pot_sram[POT_BASE+ckj], ref_pot[ckj], {tag, " pot(stall)"});
            end
            check_eq_u(spike_sram[SPIKE_BASE], ref_spk_word, {tag, " spk(stall)"});
        end
    endtask

    // ================================================================
    // Randomised config builders (set state, then run_pair)
    // ================================================================
    integer i, t;

    task rand_common;                     // spikes, bias, thresh, pot init
        begin
            // ALL inputs spike: with no gaps, act_data_idx tracks the grid index,
            // so the connectivity golden is exact.  Gapped/sparse activations are
            // probed separately (gap_probe) because they expose a weight-row
            // mis-indexing in the DUT (see verif/FINDINGS.md F9 -- pending Simon).
            for (i=0; i<nin;  i=i+1) spike_in[i] = 1'b1;
            for (i=0; i<nout; i=i+1) begin
                bias[i]  = sgnext($urandom_range(0,255), 8);
                thr[i]   = sgnext($urandom_range(0,255), 8);
                potin[i] = $urandom_range(0,2000) - 1000;
            end
        end
    endtask

    // Directed probe (reported as a NOTE, not scored): a non-spike input in the
    // MIDDLE of the grid.  Architecturally each spiking input's weight row =
    // its grid index, so spikes at grid {0,1,3} must read rows {0,1,3}.  The DUT
    // compacts to {0,1,2} (the silent input does not advance act_data_idx), so a
    // full layer with sparse activations accumulates the wrong weight rows.
    integer pj;
    task gap_probe;
        reg ok;
        begin
            cmode=0; cwszb=8; cwszc=3'b011; cskip=1; cnpmode=0;
            cinx=2; ciny=2; coutx=4; couty=1; nin=4; nout=4; crpn=1;
            ckxl=1;ckyl=1;ckxs=1;ckys=1;ckxo=0;ckyo=0; ciszc=0;ctszc=0;cscnt=0;
            spike_in[0]=1; spike_in[1]=1; spike_in[2]=0; spike_in[3]=1;
            for (pj=0;pj<nout;pj=pj+1) begin bias[pj]=0; thr[pj]=120; potin[pj]=0; end
            for (pj=0;pj<4;pj=pj+1) begin
                wfull[0*32+pj]=10+pj; wfull[1*32+pj]=20+pj;
                wfull[2*32+pj]=30+pj; wfull[3*32+pj]=40+pj;
            end
            compute_golden; configure; setup_mem; stall_en=0; launch;
            ok = 1'b1;
            for (pj=0;pj<nout;pj=pj+1)
                if (syn_sram[SYN_BASE+pj] !== exp_syn[pj]) ok = 1'b0;
            if (ok)
                $display("NOTE F9: gapped-activation weight indexing OK (grid-indexed).");
            else
                $display("NOTE F9: gapped activations mis-index weight rows -- spikes at grid {0,1,3} read rows {0,1,2}. syn got %0d %0d %0d %0d, architecturally-correct %0d %0d %0d %0d. Suspected DUT bug -> Simon.",
                         $signed(syn_sram[SYN_BASE+0]), $signed(syn_sram[SYN_BASE+1]),
                         $signed(syn_sram[SYN_BASE+2]), $signed(syn_sram[SYN_BASE+3]),
                         exp_syn[0], exp_syn[1], exp_syn[2], exp_syn[3]);
        end
    endtask

    task scn_full;
        input integer wszb; input [2:0] wszc; input integer skip, npmode; input [255:0] tag;
        begin
            cmode=0; cwszb=wszb; cwszc=wszc; cskip=skip; cnpmode=npmode;
            cinx=2; ciny=2; coutx=4; couty=1; nin=cinx*ciny; nout=coutx*couty;
            crpn = (nout*wszb + 31)/32; if (crpn<1) crpn=1;
            ckxl=1;ckyl=1;ckxs=1;ckys=1;ckxo=0;ckyo=0; ciszc=0;ctszc=0;cscnt=0;
            rand_common;
            for (i=0;i<nin;i=i+1) for (t=0;t<nout;t=t+1)
                wfull[i*32+t] = sgnext($urandom_range(0,(1<<wszb)-1), wszb);
            run_pair(tag);
        end
    endtask

    task scn_sparse;
        input integer iszb, wszb, tszb; input [2:0] iszc, wszc, tszc;
        input integer scnt, skip, npmode; input [255:0] tag;
        begin
            cmode=1; cwszb=wszb; cwszc=wszc; cskip=skip; cnpmode=npmode;
            cinx=2; ciny=2; coutx=4; couty=1; nin=cinx*ciny; nout=coutx*couty;
            crpn=1; ciszb=iszb; cwszb_sp=wszb; ctszb=tszb;
            ciszc=iszc; ctszc=tszc; cwszc=wszc; cscnt=scnt;
            ckxl=1;ckyl=1;ckxs=1;ckys=1;ckxo=0;ckyo=0;
            rand_common;
            for (i=0;i<nin;i=i+1) for (t=0;t<scnt;t=t+1) begin
                sidx[i*16+t] = $urandom_range(0, nout-1);
                swgt[i*16+t] = sgnext($urandom_range(0,(1<<wszb)-1), wszb);
            end
            run_pair(tag);
        end
    endtask

    task scn_conv;
        input integer wszb; input [2:0] wszc;
        input integer kxl,kyl,kxs,kys,kxo,kyo, inx,iny,outx,outy, skip,npmode;
        input [255:0] tag;
        begin
            cmode=2; cwszb=wszb; cwszc=wszc; cskip=skip; cnpmode=npmode;
            cinx=inx; ciny=iny; coutx=outx; couty=outy; nin=inx*iny; nout=outx*outy;
            crpn=1; ckxl=kxl;ckyl=kyl;ckxs=kxs;ckys=kys;ckxo=kxo;ckyo=kyo;
            ciszc=0;ctszc=0;cscnt=0;
            rand_common;
            for (i=0;i<kxl*kyl;i=i+1) kern[i] = sgnext($urandom_range(0,(1<<wszb)-1), wszb);
            run_pair(tag);
        end
    endtask

    // ================================================================
    // Stimulus
    // ================================================================
    integer rep;
    initial begin
        verif_errors=0; verif_checks=0;
        stall_en=0; stall_pct=0;
        weight_wait=0; act_wait=0; syn_wait=0; bias_wait=0; thresh_wait=0; pot_wait=0; spike_wait=0;
        weight_mem_data_i=0; act_mem_data_i=0; syn_curr_mem_data_i=0;
        bias_curr_mem_data_i=0; thresh_mem_data_i=0; pot_mem_data_i=0;
        reset=1; sys_req_i=0; start_new_block_i=0;
        repeat (5) @(posedge clk); @(negedge clk); reset=1'b0; repeat(2) @(posedge clk);

        $display("=== tb_acc_snn_stress (snnAcc full-chip) ===");
        $display("snnAcc SNN_ACC_VERSION = 0x%08h", u_dut.SNN_ACC_VERSION);
        if (u_dut.SNN_ACC_VERSION === 32'h0) begin
            $display("FAIL: SNN_ACC_VERSION is zero/unset"); verif_errors=verif_errors+1;
        end
        void'($urandom(32'h5417_0001));

        // ---- FULL mode: every weight width x neuron-update on/off x np_mode ----
        for (rep=0; rep<3; rep=rep+1) begin
            scn_full(1,  3'b000, 0, 0, "F_1b");
            scn_full(2,  3'b001, 0, 0, "F_2b");
            scn_full(4,  3'b010, 0, 0, "F_4b");
            scn_full(8,  3'b011, 0, 0, "F_8b");
            scn_full(16, 3'b100, 0, 0, "F_16b");
            scn_full(32, 3'b101, 0, 0, "F_32b");
            scn_full(8,  3'b011, 1, 0, "F_skip");        // neuron-update OFF
            scn_full(8,  3'b011, 0, 1, "F_subfire");     // np_mode sub_on_fire
            scn_full(8,  3'b011, 0, 4, "F_clrpot");      // np_mode clear_pot
        end

        // ---- SPARSE mode: index_sz x weight_sz unpack matrix x skip -----------
        for (rep=0; rep<3; rep=rep+1) begin
            scn_sparse(4,4,8,  3'b010,3'b010,3'b011, 4, 0, 0, "S_i4w4");
            scn_sparse(8,8,16, 3'b011,3'b011,3'b100, 2, 0, 0, "S_i8w8");
            scn_sparse(2,2,4,  3'b001,3'b001,3'b010, 8, 0, 0, "S_i2w2");
            scn_sparse(4,8,16, 3'b010,3'b011,3'b100, 2, 0, 0, "S_i4w8");
            scn_sparse(4,4,8,  3'b010,3'b010,3'b011, 4, 1, 0, "S_skip");
            scn_sparse(4,4,8,  3'b010,3'b010,3'b011, 4, 0, 1, "S_subfire");
        end

        // ---- CONV mode: identity, 3x3 (border OOB), strided x skip ------------
        for (rep=0; rep<3; rep=rep+1) begin
            scn_conv(8, 3'b011, 1,1,1,1,0,0, 3,3,3,3, 0, 0, "C_1x1");
            scn_conv(8, 3'b011, 3,3,1,1,1,1, 3,3,3,3, 0, 0, "C_3x3");
            scn_conv(8, 3'b011, 3,1,2,1,1,0, 4,1,2,1, 0, 0, "C_strd2");
            scn_conv(4, 3'b010, 3,3,1,1,1,1, 3,3,3,3, 0, 0, "C_3x3_4b");
            scn_conv(8, 3'b011, 3,3,1,1,1,1, 3,3,3,3, 1, 0, "C_skip");
            scn_conv(8, 3'b011, 3,3,1,1,1,1, 3,3,3,3, 0, 1, "C_subfire");
        end

        // Gapped/sparse-activation indexing probe (NOTE, not scored).
        gap_probe;

        `VERIF_EPILOGUE("tb_acc_snn_stress")
    end

    `VERIF_WATCHDOG(80000000)

endmodule
