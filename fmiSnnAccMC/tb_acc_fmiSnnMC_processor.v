// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
`timescale 1ns/1ps

`include "../shared/constants.v"

// ====================================================================
//  tb_acc_fmiSnnMC_processor  (FMI variant) — automatic-check testbench
//
//  Configuration
//  -------------
//  weight_mode    = full connectivity (2'b00)
//  Input layer    : 2 neurons (in_x_len=2, in_y_len=1)
//  Output layer   : 2 neurons (out_x_len=2, out_y_len=1, last_neuron_idx=1)
//  Weights        : 8-bit value=10; weight_sram filled with 0x0A0A_0A0A
//  Activations    : all spiking; act_sram filled with 0xFFFF_FFFF
//  dcy_syn = dcy_mem = 0x8000_0000 (0.5 in Q0.32), per neuron
//
//  FMI neuron update (no ada):
//    eff_syn = syn_curr = 20  (2 spikes × weight 10)
//    diff    = pot - eff_syn  = 0 - 20 = -20
//    decayed_diff = floor(-20 × 0.5) = -10
//    new_mem = decayed_diff + eff_syn = -10 + 20 = 10
//    syn_o   = floor(20 × 0.5) = 10
//
//  Test 1 — no spike  (threshold=50)
//    new_mem=10 < 50  → no spike
//    spike_sram[60]       = 0x00000000
//    syn_curr_sram[20/21] = 32'd10
//    pot_sram[50/51]      = 32'd10
//
//  Test 2 — spike  (threshold=5)
//    new_mem=10 >= 5  → both neurons spike
//    spike_sram[60]       = 0x00000003
//    syn_curr_sram[20/21] = 32'd10
//    pot_sram[50/51]      = 32'd0
//
//  Test 3 — adaptive neuron  (has_ada=1, ada=0 initially)
//    b_eff=0, ada=0  → ada_corr=0, same result as non-adaptive
//    After spike: ada_o = scl_ada = 0x8000_0000  (written back to ada_sram)
//
//  Test 4 — non-uniform column-major weights, no spike
//
//  Test 5 — LAST input is a non-spike (weight-pass hang regression,
//    2026-06-10): the gated act stream never shows act_data_last_i to
//    the weight generator, so the dump of the last non-spike must
//    terminate the pass (act_last_dumped_i) or the pipeline hangs.
//
//  Config is delivered in the PACKED per-task layout
//  (regmap.PACKED_FMI_*: cfg_mem word i -> offset i*4; 16 words).
// ====================================================================

`define TGT_ACC_ID            'h0
`define TGT_CONFIG_BASE_ADDR  32'hFFFFFFFF
`define NUM_TIMESTEPS         32
`define IN_DATA_SZ            32
`define X_INPUT_SZ            5
`define X_OUTPUT_SZ           5
`define X_KERNEL_SZ           3
`define X_STEP_SZ             3
`define ELEMS_PER_ROW         4
`define ROWS_PER_NEURON       4
`define ELEM_SZ               8
`define WEIGHT_SLICE_SZ       5
`define WEIGHT_IDX_SZ         10
`define WEIGHT_DATA_IDX_SZ    5
`define ACT_SLICE_SZ          3
`define ACT_IDX_SZ            10
`define ACT_DATA_IDX_SZ       5
`define SYN_CURR_IDX_SZ       10
`define SYN_CURR_DATA_IDX_SZ  5
`define SYN_CURR_SLICE_SZ     3
`define SYN_CURR_SLICE_BITS   32
`define POT_IDX_SZ            10
`define POT_DATA_IDX_SZ       5
`define POT_SLICE_SZ          3
`define POT_SLICE_BITS        32
`define SPIKE_IDX_SZ          10
`define SPIKE_DATA_IDX_SZ     5
`define SPIKE_SLICE_SZ        3
`define SPIKE_SLICE_BITS      8
// SP_BIAS_CURR params used only by spike_processing (unchanged sub-module)
`define BIAS_CURR_IDX_SZ      10
`define BIAS_CURR_DATA_IDX_SZ 5
`define BIAS_CURR_SLICE_SZ    3
`define BIAS_CURR_SLICE_BITS  8

module tb_acc_fmiSnnMC_processor;

    localparam CLK_PERIOD = 10;
    localparam MEM_DEPTH  = 4096;   // bumped 256 -> 4096 for T6_LAYER (group4 full layer)

    reg clk;
    reg reset;

    initial clk = 1'b0;
    always #(CLK_PERIOD/2) clk = ~clk;

    // ----------------------------------------------------------------
    // DUT inputs
    // ----------------------------------------------------------------
    reg                      sys_req_i   = 1'b0;
    reg             [31:0]   sys_addr_i  = 32'b0;
    reg             [31:0]   sys_data_i  = 32'b0;

    reg                      start_new_block_i = 1'b0;
    reg   [`TGT_ACC_SZ-1:0]  target_acc_i      = {`TGT_ACC_SZ{1'b0}};
    reg [`SCH_ENTRY_SZ-1:0]  buffer_info_i     = {`SCH_ENTRY_SZ{1'b0}};

    reg [`PIN_BITS-1:0] sp_src1_buff_addr_i = {`PIN_BITS{1'b0}};
    reg [`PIN_BITS-1:0] sp_src2_buff_addr_i = {`PIN_BITS{1'b0}};
    reg [`PIN_BITS-1:0] sp_src3_buff_addr_i = {`PIN_BITS{1'b0}};
    reg [`PIN_BITS-1:0] sp_tgt_buff_addr_i  = {`PIN_BITS{1'b0}};
    reg [`PIN_BITS-1:0] sp_weight_row_len_i = {`PIN_BITS{1'b0}};

    reg [`PIN_BITS-1:0] np_src1_buff_addr_i = {`PIN_BITS{1'b0}};
    reg [`PIN_BITS-1:0] np_src2_buff_addr_i = {`PIN_BITS{1'b0}};
    reg [`PIN_BITS-1:0] np_src3_buff_addr_i = {`PIN_BITS{1'b0}};
    reg [`PIN_BITS-1:0] np_tgt_buff_addr_i  = {`PIN_BITS{1'b0}};
    reg [`PIN_BITS-1:0] np_weight_row_len_i = {`PIN_BITS{1'b0}};

    // MC: multi-channel conv sizing (POC: testbench-driven, no AXI reg).
    // Default 1 collapses to the legacy single-channel conv behaviour, so
    // T1-T7 keep working unchanged.
    reg [6:0]  tb_sp_cin_len   = 7'd1;
    reg [6:0]  tb_sp_cout_len  = 7'd1;

    // ----------------------------------------------------------------
    // DUT outputs
    // ----------------------------------------------------------------
    wire                    sys_ack_o;
    wire                    spike_proc_finished_o;
    wire                    acc_busy_o;
    wire                    acc_finished_o;

    wire                    weight_mem_rd_o;
    wire [`ADDR_SIZE-1:0]   weight_mem_addr_o;
    wire                    act_mem_req_o;
    wire [`ADDR_SIZE-1:0]   act_mem_addr_o;
    wire                    syn_curr_mem_wr_o;
    wire                    syn_curr_mem_rd_o;
    wire [`ADDR_SIZE-1:0]   syn_curr_mem_addr_o;
    wire  [`POT_BITS-1:0]   syn_curr_mem_data_o;
    wire                    thresh_mem_rd_o;
    wire [`ADDR_SIZE-1:0]   thresh_mem_addr_o;
    wire                    pot_mem_wr_o;
    wire                    pot_mem_rd_o;
    wire [`ADDR_SIZE-1:0]   pot_mem_addr_o;
    wire  [`POT_BITS-1:0]   pot_mem_data_o;
    wire                    spike_mem_wr_o;
    wire [`ADDR_SIZE-1:0]   spike_mem_addr_o;
    wire  [`ACT_BITS-1:0]   spike_mem_data_o;

    wire                    dcy_syn_mem_rd_o;
    wire [`ADDR_SIZE-1:0]   dcy_syn_mem_addr_o;
    wire                    dcy_mem_mem_rd_o;
    wire [`ADDR_SIZE-1:0]   dcy_mem_mem_addr_o;
    wire                    ada_mem_wr_o;
    wire                    ada_mem_rd_o;
    wire [`ADDR_SIZE-1:0]   ada_mem_addr_o;
    wire  [31:0]            ada_mem_data_o;
    wire                    b_eff_mem_rd_o;
    wire [`ADDR_SIZE-1:0]   b_eff_mem_addr_o;
    wire                    dcy_ada_mem_rd_o;
    wire [`ADDR_SIZE-1:0]   dcy_ada_mem_addr_o;
    wire                    scl_ada_mem_rd_o;
    wire [`ADDR_SIZE-1:0]   scl_ada_mem_addr_o;

    // ----------------------------------------------------------------
    // SRAM read-data buses
    // ----------------------------------------------------------------
    wire  [`WTD_BITS-1:0]   weight_mem_data_i;
    wire  [`ACT_BITS-1:0]   act_mem_data_i;
    wire  [`POT_BITS-1:0]   syn_curr_mem_data_i;
    wire  [`WTD_BITS-1:0]   thresh_mem_data_i;
    wire  [`POT_BITS-1:0]   pot_mem_data_i;
    wire  [31:0]            dcy_syn_mem_data_i;
    wire  [31:0]            dcy_mem_mem_data_i;
    wire  [31:0]            ada_mem_data_i;
    wire  [31:0]            b_eff_mem_data_i;
    wire  [31:0]            dcy_ada_mem_data_i;
    wire  [31:0]            scl_ada_mem_data_i;

    // All wait inputs tied low (zero-latency SRAM models)
    wire weight_mem_wait_i   = 1'b0;
    wire act_mem_wait_i      = 1'b0;
    wire syn_curr_mem_wait_i = 1'b0;
    wire thresh_mem_wait_i   = 1'b0;
    wire pot_mem_wait_i      = 1'b0;
    wire spike_mem_wait_i    = 1'b0;
    wire dcy_syn_mem_wait_i  = 1'b0;
    wire dcy_mem_mem_wait_i  = 1'b0;
    wire ada_mem_wait_i      = 1'b0;
    wire b_eff_mem_wait_i    = 1'b0;
    wire dcy_ada_mem_wait_i  = 1'b0;
    wire scl_ada_mem_wait_i  = 1'b0;

    // ----------------------------------------------------------------
    // DUT instantiation
    // ----------------------------------------------------------------
    acc_fmiSnnMC_processor # (
        .TGT_ACC_ID               (`TGT_ACC_ID),
        .TGT_CONFIG_BASE_ADDR     (`TGT_CONFIG_BASE_ADDR),
        .SP_NUM_TIMESTEPS         (`NUM_TIMESTEPS),
        .SP_X_INPUT_SZ            (`X_INPUT_SZ),
        .SP_X_OUTPUT_SZ           (`X_OUTPUT_SZ),
        .SP_X_KERNEL_SZ           (`X_KERNEL_SZ),
        .SP_X_KERNEL_OFF_SZ       (`X_STEP_SZ),
        .SP_X_STEP_SZ             (`X_STEP_SZ),
        .SP_ELEMS_PER_ROW         (`ELEMS_PER_ROW),
        .SP_ROWS_PER_NEURON       (`ROWS_PER_NEURON),
        .SP_TIMESTEP_SZ           (10),
        .SP_IN_DATA_BITS          (32),
        .SP_ELEM_SZ               (`ELEM_SZ),
        .SP_ACT_SLICE_SZ          (`ACT_SLICE_SZ),
        .SP_ACT_DATA_IDX_SZ       (`ACT_DATA_IDX_SZ),
        .SP_WEIGHT_ENTRY_BITS     (8),
        .SP_WEIGHT_IDX_SZ         (`WEIGHT_IDX_SZ),
        .SP_WEIGHT_SLICE_SZ       (`WEIGHT_SLICE_SZ),
        .SP_WEIGHT_DATA_IDX_SZ    (`WEIGHT_DATA_IDX_SZ),
        .SP_SYN_CURR_IDX_SZ       (`SYN_CURR_IDX_SZ),
        .SP_SYN_CURR_DATA_IDX_SZ  (`SYN_CURR_DATA_IDX_SZ),
        .SP_SYN_CURR_SLICE_SZ     (`SYN_CURR_SLICE_SZ),
        .SP_SYN_CURR_SLICE_BITS   (`SYN_CURR_SLICE_BITS),
        .SP_BIAS_CURR_IDX_SZ      (`BIAS_CURR_IDX_SZ),
        .SP_BIAS_CURR_DATA_IDX_SZ (`BIAS_CURR_DATA_IDX_SZ),
        .SP_BIAS_CURR_SLICE_SZ    (`BIAS_CURR_SLICE_SZ),
        .SP_BIAS_CURR_SLICE_BITS  (`BIAS_CURR_SLICE_BITS),
        .NP_NUM_TIMESTEPS         (`NUM_TIMESTEPS),
        .NP_TIMESTEP_SZ           (10),
        .NP_IN_DATA_BITS          (32),
        .NP_NEURON_IDX_SZ         (10),
        .NP_SYN_CURR_IDX_SZ       (`SYN_CURR_IDX_SZ),
        .NP_SYN_CURR_DATA_IDX_SZ  (`SYN_CURR_DATA_IDX_SZ),
        .NP_SYN_CURR_SLICE_SZ     (`SYN_CURR_SLICE_SZ),
        .NP_SYN_CURR_SLICE_BITS   (`SYN_CURR_SLICE_BITS),
        .NP_POT_IDX_SZ            (`POT_IDX_SZ),
        .NP_POT_DATA_IDX_SZ       (`POT_DATA_IDX_SZ),
        .NP_POT_SLICE_SZ          (`POT_SLICE_SZ),
        .NP_POT_SLICE_BITS        (`POT_SLICE_BITS),
        .NP_SPIKE_IDX_SZ          (`SPIKE_IDX_SZ),
        .NP_SPIKE_DATA_IDX_SZ     (`SPIKE_DATA_IDX_SZ),
        .NP_SPIKE_SLICE_SZ        (`SPIKE_SLICE_SZ),
        .NP_SPIKE_SLICE_BITS      (`SPIKE_SLICE_BITS),
        .MEM_ADDR_BITS            (`ADDR_SIZE)
    ) u_dut (
        .clk                      (clk),
        .reset                    (reset),
        .sys_req_i                (sys_req_i),
        .sys_ack_o                (sys_ack_o),
        .sys_addr_i               (sys_addr_i),
        .sys_data_i               (sys_data_i),
        .start_new_block_i        (start_new_block_i),
        .target_acc_i             (target_acc_i),
        .buffer_info_i            (buffer_info_i),
        .spike_proc_finished_o    (spike_proc_finished_o),
        .acc_busy_o               (acc_busy_o),
        .acc_finished_o           (acc_finished_o),
        .sp_src1_buff_addr_i      (sp_src1_buff_addr_i),
        .sp_src2_buff_addr_i      (sp_src2_buff_addr_i),
        .sp_src3_buff_addr_i      (sp_src3_buff_addr_i),
        .sp_tgt_buff_addr_i       (sp_tgt_buff_addr_i),
        .sp_weight_row_len_i      (sp_weight_row_len_i),
        .np_src1_buff_addr_i      (np_src1_buff_addr_i),
        .np_src2_buff_addr_i      (np_src2_buff_addr_i),
        .np_src3_buff_addr_i      (np_src3_buff_addr_i),
        .np_tgt_buff_addr_i       (np_tgt_buff_addr_i),
        .np_weight_row_len_i      (np_weight_row_len_i),
        .weight_mem_rd_o          (weight_mem_rd_o),
        .weight_mem_wait_i        (weight_mem_wait_i),
        .weight_mem_addr_o        (weight_mem_addr_o),
        .weight_mem_data_i        (weight_mem_data_i),
        .act_mem_req_o            (act_mem_req_o),
        .act_mem_wait_i           (act_mem_wait_i),
        .act_mem_addr_o           (act_mem_addr_o),
        .act_mem_data_i           (act_mem_data_i),
        .syn_curr_mem_wr_o        (syn_curr_mem_wr_o),
        .syn_curr_mem_rd_o        (syn_curr_mem_rd_o),
        .syn_curr_mem_wait_i      (syn_curr_mem_wait_i),
        .syn_curr_mem_addr_o      (syn_curr_mem_addr_o),
        .syn_curr_mem_data_o      (syn_curr_mem_data_o),
        .syn_curr_mem_data_i      (syn_curr_mem_data_i),
        .thresh_mem_rd_o          (thresh_mem_rd_o),
        .thresh_mem_wait_i        (thresh_mem_wait_i),
        .thresh_mem_addr_o        (thresh_mem_addr_o),
        .thresh_mem_data_i        (thresh_mem_data_i),
        .pot_mem_wr_o             (pot_mem_wr_o),
        .pot_mem_rd_o             (pot_mem_rd_o),
        .pot_mem_wait_i           (pot_mem_wait_i),
        .pot_mem_addr_o           (pot_mem_addr_o),
        .pot_mem_data_o           (pot_mem_data_o),
        .pot_mem_data_i           (pot_mem_data_i),
        .spike_mem_wr_o           (spike_mem_wr_o),
        .spike_mem_wait_i         (spike_mem_wait_i),
        .spike_mem_addr_o         (spike_mem_addr_o),
        .spike_mem_data_o         (spike_mem_data_o),
        .dcy_syn_mem_rd_o         (dcy_syn_mem_rd_o),
        .dcy_syn_mem_wait_i       (dcy_syn_mem_wait_i),
        .dcy_syn_mem_addr_o       (dcy_syn_mem_addr_o),
        .dcy_syn_mem_data_i       (dcy_syn_mem_data_i),
        .dcy_mem_mem_rd_o         (dcy_mem_mem_rd_o),
        .dcy_mem_mem_wait_i       (dcy_mem_mem_wait_i),
        .dcy_mem_mem_addr_o       (dcy_mem_mem_addr_o),
        .dcy_mem_mem_data_i       (dcy_mem_mem_data_i),
        .ada_mem_wr_o             (ada_mem_wr_o),
        .ada_mem_rd_o             (ada_mem_rd_o),
        .ada_mem_wait_i           (ada_mem_wait_i),
        .ada_mem_addr_o           (ada_mem_addr_o),
        .ada_mem_data_o           (ada_mem_data_o),
        .ada_mem_data_i           (ada_mem_data_i),
        .b_eff_mem_rd_o           (b_eff_mem_rd_o),
        .b_eff_mem_wait_i         (b_eff_mem_wait_i),
        .b_eff_mem_addr_o         (b_eff_mem_addr_o),
        .b_eff_mem_data_i         (b_eff_mem_data_i),
        .dcy_ada_mem_rd_o         (dcy_ada_mem_rd_o),
        .dcy_ada_mem_wait_i       (dcy_ada_mem_wait_i),
        .dcy_ada_mem_addr_o       (dcy_ada_mem_addr_o),
        .dcy_ada_mem_data_i       (dcy_ada_mem_data_i),
        .scl_ada_mem_rd_o         (scl_ada_mem_rd_o),
        .scl_ada_mem_wait_i       (scl_ada_mem_wait_i),
        .scl_ada_mem_addr_o       (scl_ada_mem_addr_o),
        .scl_ada_mem_data_i       (scl_ada_mem_data_i),
        // MC: testbench-driven multi-channel conv sizing (POC: no AXI reg).
        // Tied to 1 for T1-T7 (collapse to legacy behaviour); overridden per
        // test for T8 (multi-channel conv).
        .sp_cin_len_i             (tb_sp_cin_len),
        .sp_cout_len_i            (tb_sp_cout_len)
    );

    // ----------------------------------------------------------------
    // SRAM models  (synchronous, 1-cycle read latency)
    // ----------------------------------------------------------------
    sram_model #(.DATA_W(`WTD_BITS), .DEPTH(MEM_DEPTH)) u_weight_mem (
        .clk(clk), .we(1'b0), .re(weight_mem_rd_o),
        .addr(weight_mem_addr_o[15:0]), .wdata({`WTD_BITS{1'b0}}),
        .rdata(weight_mem_data_i));

    sram_model #(.DATA_W(`ACT_BITS), .DEPTH(MEM_DEPTH)) u_act_mem (
        .clk(clk), .we(1'b0), .re(act_mem_req_o),
        .addr(act_mem_addr_o[15:0]), .wdata({`ACT_BITS{1'b0}}),
        .rdata(act_mem_data_i));

    sram_model #(.DATA_W(`POT_BITS), .DEPTH(MEM_DEPTH)) u_syn_curr_mem (
        .clk(clk), .we(syn_curr_mem_wr_o), .re(syn_curr_mem_rd_o),
        .addr(syn_curr_mem_addr_o[15:0]), .wdata(syn_curr_mem_data_o),
        .rdata(syn_curr_mem_data_i));

    sram_model #(.DATA_W(`WTD_BITS), .DEPTH(MEM_DEPTH)) u_thresh_mem (
        .clk(clk), .we(1'b0), .re(thresh_mem_rd_o),
        .addr(thresh_mem_addr_o[15:0]), .wdata({`WTD_BITS{1'b0}}),
        .rdata(thresh_mem_data_i));

    sram_model #(.DATA_W(`POT_BITS), .DEPTH(MEM_DEPTH)) u_pot_mem (
        .clk(clk), .we(pot_mem_wr_o), .re(pot_mem_rd_o),
        .addr(pot_mem_addr_o[15:0]), .wdata(pot_mem_data_o),
        .rdata(pot_mem_data_i));

    wire [`ACT_BITS-1:0] spike_rdata_nc;
    sram_model #(.DATA_W(`ACT_BITS), .DEPTH(MEM_DEPTH)) u_spike_mem (
        .clk(clk), .we(spike_mem_wr_o), .re(1'b0),
        .addr(spike_mem_addr_o[15:0]), .wdata(spike_mem_data_o),
        .rdata(spike_rdata_nc));

    // Per-neuron decay memories (read-only from DUT perspective)
    sram_model #(.DATA_W(32), .DEPTH(MEM_DEPTH)) u_dcy_syn_mem (
        .clk(clk), .we(1'b0), .re(dcy_syn_mem_rd_o),
        .addr(dcy_syn_mem_addr_o[15:0]), .wdata(32'b0),
        .rdata(dcy_syn_mem_data_i));

    sram_model #(.DATA_W(32), .DEPTH(MEM_DEPTH)) u_dcy_mem_mem (
        .clk(clk), .we(1'b0), .re(dcy_mem_mem_rd_o),
        .addr(dcy_mem_mem_addr_o[15:0]), .wdata(32'b0),
        .rdata(dcy_mem_mem_data_i));

    // Ada state memory (read/write)
    sram_model #(.DATA_W(32), .DEPTH(MEM_DEPTH)) u_ada_mem (
        .clk(clk), .we(ada_mem_wr_o), .re(ada_mem_rd_o),
        .addr(ada_mem_addr_o[15:0]), .wdata(ada_mem_data_o),
        .rdata(ada_mem_data_i));

    sram_model #(.DATA_W(32), .DEPTH(MEM_DEPTH)) u_b_eff_mem (
        .clk(clk), .we(1'b0), .re(b_eff_mem_rd_o),
        .addr(b_eff_mem_addr_o[15:0]), .wdata(32'b0),
        .rdata(b_eff_mem_data_i));

    sram_model #(.DATA_W(32), .DEPTH(MEM_DEPTH)) u_dcy_ada_mem (
        .clk(clk), .we(1'b0), .re(dcy_ada_mem_rd_o),
        .addr(dcy_ada_mem_addr_o[15:0]), .wdata(32'b0),
        .rdata(dcy_ada_mem_data_i));

    sram_model #(.DATA_W(32), .DEPTH(MEM_DEPTH)) u_scl_ada_mem (
        .clk(clk), .we(1'b0), .re(scl_ada_mem_rd_o),
        .addr(scl_ada_mem_addr_o[15:0]), .wdata(32'b0),
        .rdata(scl_ada_mem_data_i));

    // ----------------------------------------------------------------
    // SRAM initialisation
    //
    //  Memory layout:
    //   act_base     =  0   all spiking (0xFFFF_FFFF)
    //   weight_base  = 10   8-bit weight=10 (0x0A0A_0A0A)
    //   syn_curr     = 20   zero initially; SP accumulates here
    //   thresh_base  = 40   set per test
    //   pot_base     = 50   zero initially
    //   spike_base   = 60   written by NP packer
    //   dcy_syn_base = 70   0x8000_0000 (0.5) per neuron
    //   dcy_mem_base = 80   0x8000_0000 (0.5) per neuron
    //   ada_base     = 90   zero initially
    //   b_eff_base   = 100  zero (no adaptive correction by default)
    //   dcy_ada_base = 110  0x8000_0000 (used only when has_ada=1)
    //   scl_ada_base = 120  0x8000_0000 (used only when has_ada=1)
    // ----------------------------------------------------------------
    integer i_init;
    initial begin
        #1;
        for (i_init = 0; i_init < MEM_DEPTH; i_init = i_init + 1) begin
            u_act_mem.mem[i_init]      = 32'hFFFF_FFFF;
            u_weight_mem.mem[i_init]   = 32'h0A0A_0A0A;
            u_syn_curr_mem.mem[i_init] = 32'd0;
            u_thresh_mem.mem[i_init]   = 32'd0;
            u_pot_mem.mem[i_init]      = 32'd0;
            u_spike_mem.mem[i_init]    = 32'd0;
            u_dcy_syn_mem.mem[i_init]  = 32'h8000_0000;
            u_dcy_mem_mem.mem[i_init]  = 32'h8000_0000;
            u_ada_mem.mem[i_init]      = 32'd0;
            u_b_eff_mem.mem[i_init]    = 32'd0;
            u_dcy_ada_mem.mem[i_init]  = 32'h8000_0000;
            u_scl_ada_mem.mem[i_init]  = 32'h8000_0000;
        end
    end

    // ----------------------------------------------------------------
    // Helpers
    // ----------------------------------------------------------------
    integer errors;
    integer timeout;

    task cfg_write;
        input [31:0] addr;
        input [31:0] data;
        begin
            @(negedge clk);
            sys_req_i  = 1'b1;
            sys_addr_i = addr;
            sys_data_i = data;
            #1;
            if (!sys_ack_o) begin
                $display("FAIL cfg_write: no ACK for addr=0x%08h", addr);
                errors = errors + 1;
            end
            @(negedge clk);
            sys_req_i = 1'b0;
        end
    endtask

    task check_eq;
        input [31:0] got;
        input [31:0] exp;
        input [255:0] label;
        begin
            if (got !== exp) begin
                $display("FAIL %s: got 0x%08h  exp 0x%08h", label, got, exp);
                errors = errors + 1;
            end else begin
                $display("  OK  %s = 0x%08h", label, got);
            end
        end
    endtask

    task wait_pipeline;
        output reg timed_out;
        begin
            timed_out = 0;

            timeout = 500;
            @(posedge clk);
            while (!spike_proc_finished_o && timeout > 0) begin
                timeout = timeout - 1;
                @(posedge clk);
            end
            #1;
            if (timeout == 0) begin
                $display("FAIL: spike_proc_finished_o timeout");
                errors    = errors + 1;
                timed_out = 1;
            end else
                $display("  spike_proc_finished after %0d cycles", 500 - timeout);

            timeout = 500;
            @(posedge clk);
            while (!acc_finished_o && timeout > 0) begin
                timeout = timeout - 1;
                @(posedge clk);
            end
            #1;
            if (timeout == 0) begin
                $display("FAIL: acc_finished_o timeout");
                errors    = errors + 1;
                timed_out = 1;
            end else
                $display("  acc_finished after %0d more cycles", 500 - timeout);

            @(posedge clk); #1;
            if (acc_busy_o) begin
                $display("FAIL: acc_busy_o still high after acc_finished");
                errors = errors + 1;
            end
        end
    endtask

    // ----------------------------------------------------------------
    // Stimulus
    // ----------------------------------------------------------------
    reg timed_out;

    initial begin
        errors            = 0;
        reset             = 1'b1;
        sys_req_i         = 1'b0;
        start_new_block_i = 1'b0;
        target_acc_i      = {`TGT_ACC_SZ{1'b0}};
        buffer_info_i     = {`SCH_ENTRY_SZ{1'b0}};

        repeat (5) @(posedge clk);
        @(negedge clk); reset = 1'b0;
        repeat (2) @(posedge clk);

        // ============================================================
        // Write configuration registers (PACKED per-task layout,
        // regmap.PACKED_FMI_*: word i at offset i*4; 16 words)
        // ============================================================
        // W0..W11: base addresses (one per word)
        cfg_write(32'hFFFF_0000, 32'd0);    // act_base_addr      = 0
        cfg_write(32'hFFFF_0004, 32'd10);   // weight_base_addr   = 10
        cfg_write(32'hFFFF_0008, 32'd20);   // syn_curr_base_addr = 20
        cfg_write(32'hFFFF_000C, 32'd40);   // thresh_base        = 40
        cfg_write(32'hFFFF_0010, 32'd50);   // pot_base           = 50
        cfg_write(32'hFFFF_0014, 32'd60);   // spike_base         = 60
        cfg_write(32'hFFFF_0018, 32'd70);   // dcy_syn_base       = 70
        cfg_write(32'hFFFF_001C, 32'd80);   // dcy_mem_base       = 80
        cfg_write(32'hFFFF_0020, 32'd90);   // ada_base           = 90
        cfg_write(32'hFFFF_0024, 32'd100);  // b_eff_base         = 100
        cfg_write(32'hFFFF_0028, 32'd110);  // dcy_ada_base       = 110
        cfg_write(32'hFFFF_002C, 32'd120);  // scl_ada_base       = 120
        // S0..S2: two 16-bit size lanes each
        cfg_write(32'hFFFF_0030, 32'h0002_0002);    // S0 out_x=2 | in_x=2
        cfg_write(32'hFFFF_0034, 32'h0001_0001);    // S1 last_neuron_idx=1 | rows_per_neuron=1
        cfg_write(32'hFFFF_0038, 32'h0000_0001);    // S2 total_timesteps=1
        // M0: [1:0] skip=0  [5:2] np_mode=0  [9:6] weights_per_word=4
        //     [15:10] bin_point=0  [19:16] weight_sz=3(8b)
        //     [23:20] syn_curr_sz=5(32b)  [27:24] pot_sz=5(32b)
        //     [29:28] weight_mode=0(full)  [30] has_ada=0
        cfg_write(32'hFFFF_003C, 32'h0553_0100);
        // boot-only conv/sparse params (out-of-window offsets unchanged)
        cfg_write(32'hFFFF_005C, 32'd5);    // weight_idx_sz      = 5
        cfg_write(32'hFFFF_0074, 32'd1);    // x_kernel_len       = 1
        cfg_write(32'hFFFF_007C, 32'd1);    // x_kernel_step      = 1
        cfg_write(32'hFFFF_0084, 32'd0);    // x_kernel_offset    = 0

        $display("=== tb_acc_fmiSnnMC_processor (FMI variant) ===");

        // ============================================================
        // Test 1: no spike  (threshold=50 > new_mem=10)
        //
        // FMI update (dcy_mem=dcy_syn=0.5, no ada):
        //   syn_curr_in = 20 (after SP)
        //   eff_syn     = 20
        //   diff        = 0 - 20 = -20
        //   decayed_diff = floor(-20 × 0.5) = -10
        //   new_mem      = -10 + 20 = 10  <  50  → no spike
        //   syn_o        = floor(20 × 0.5) = 10
        //   pot_o        = 10
        //
        // thresh memory: thresh cache is 32-bit slice (sz=5), one word per neuron.
        // neurons 0 and 1 each get their own word.
        // thresh_sram[40] = 32'd50  (neuron 0)
        // thresh_sram[41] = 32'd50  (neuron 1)
        // ============================================================
        $display("Test 1: full pipeline, no spike (thresh=50 > new_mem=10)");
        u_thresh_mem.mem[40] = 32'd50;
        u_thresh_mem.mem[41] = 32'd50;

        @(negedge clk); start_new_block_i = 1'b1;
        @(negedge clk); start_new_block_i = 1'b0;

        wait_pipeline(timed_out);
        if (!timed_out) begin
            check_eq(u_spike_mem.mem[60],    32'h0000_0000, "T1 spike_sram[60]");
            check_eq(u_syn_curr_mem.mem[20], 32'd10,        "T1 syn_curr_sram[20]");
            check_eq(u_syn_curr_mem.mem[21], 32'd10,        "T1 syn_curr_sram[21]");
            check_eq(u_pot_mem.mem[50],      32'd10,        "T1 pot_sram[50]");
            check_eq(u_pot_mem.mem[51],      32'd10,        "T1 pot_sram[51]");
        end

        // ============================================================
        // Test 2: spike  (threshold=5 ≤ new_mem=10)
        //
        // same accumulation (syn_curr starts at zero again),
        // new_mem = 10 >= 5 → both neurons spike
        //
        // Note: pot_sram[50/51] now holds 10 from Test 1. On the second
        // run the update sees pot=10:
        //   diff = 10 - 20 = -10, decayed_diff = -5
        //   new_mem = -5 + 20 = 15 >= 5 → spike, pot_o = 0
        // ============================================================
        $display("Test 2: full pipeline, both neurons spike (thresh=5, new_mem=15)");

        for (i_init = 0; i_init < MEM_DEPTH; i_init = i_init + 1) begin
            u_syn_curr_mem.mem[i_init] = 32'd0;
            u_spike_mem.mem[i_init]    = 32'd0;
        end
        // Force fresh thresh cache fetch by pointing to a new address
        u_thresh_mem.mem[44] = 32'd5;
        u_thresh_mem.mem[45] = 32'd5;
        cfg_write(32'hFFFF_000C, 32'd44);   // thresh_base = 44

        @(negedge clk); start_new_block_i = 1'b1;
        @(negedge clk); start_new_block_i = 1'b0;

        wait_pipeline(timed_out);
        if (!timed_out) begin
            check_eq(u_spike_mem.mem[60],    32'h0000_0003, "T2 spike_sram[60]");
            check_eq(u_syn_curr_mem.mem[20], 32'd10,        "T2 syn_curr_sram[20]");
            check_eq(u_syn_curr_mem.mem[21], 32'd10,        "T2 syn_curr_sram[21]");
            check_eq(u_pot_mem.mem[50],      32'd0,         "T2 pot_sram[50] (spike reset)");
            check_eq(u_pot_mem.mem[51],      32'd0,         "T2 pot_sram[51] (spike reset)");
        end

        // ============================================================
        // Test 3: adaptive layer  (has_ada=1, ada=0, b_eff=0)
        // With ada=0 and b_eff=0 the result is identical to non-adaptive.
        // After spike: ada_sram gets updated with scl_ada (0x80000000).
        //
        // dcy_ada=0.5, scl_ada=0.5 (0x80000000 each).
        // new_ada = floor(0*0.5) + scl_ada_word = 0 + 0x80000000 = 0x80000000
        // ============================================================
        $display("Test 3: adaptive layer, spike fires, ada_sram updated");

        for (i_init = 0; i_init < MEM_DEPTH; i_init = i_init + 1) begin
            u_syn_curr_mem.mem[i_init] = 32'd0;
            u_pot_mem.mem[i_init]      = 32'd0;
            u_spike_mem.mem[i_init]    = 32'd0;
            u_ada_mem.mem[i_init]      = 32'd0;
        end
        cfg_write(32'hFFFF_000C, 32'd44);        // keep thresh_base=44 (thresh=5)
        cfg_write(32'hFFFF_003C, 32'h4553_0100); // M0 with has_ada=1 (bit 30)

        @(negedge clk); start_new_block_i = 1'b1;
        @(negedge clk); start_new_block_i = 1'b0;

        wait_pipeline(timed_out);
        if (!timed_out) begin
            check_eq(u_spike_mem.mem[60],    32'h0000_0003,  "T3 spike_sram[60]");
            check_eq(u_pot_mem.mem[50],      32'd0,          "T3 pot_sram[50] (spike reset)");
            check_eq(u_pot_mem.mem[51],      32'd0,          "T3 pot_sram[51] (spike reset)");
            // ada was 0, spike fired, so new_ada = 0 + scl_ada = 0x80000000
            check_eq(u_ada_mem.mem[90],      32'h8000_0000,  "T3 ada_sram[90] neuron0");
            check_eq(u_ada_mem.mem[91],      32'h8000_0000,  "T3 ada_sram[91] neuron1");
        end

        // ============================================================
        // Test 4: non-uniform column-major weights, no spike
        //   Distinct weights per output neuron — uniform-weight tests
        //   T1-T3 pass even with the pre-2026-05-29 out_elem_count_r
        //   bug; this one fails without the fix.
        //   W[0][0]=W[0][1]=+10, W[1][0]=W[1][1]=+5, x=1 each:
        //     syn_curr[0] = 10 + 10 = 20
        //     syn_curr[1] =  5 +  5 = 10
        //   FMI update (dcy_mem=dcy_syn=0.5, ada turned off again):
        //     For n=0: eff_syn=20, diff=-20, decayed_diff=-10, new_mem=10
        //              decayed_syn=10. 10 < thresh=50 -> no spike.
        //     For n=1: eff_syn=10, diff=-10, decayed_diff=-5,  new_mem=5
        //              decayed_syn=5.  5 < thresh=50 -> no spike.
        // Expected:
        //   spike_sram[60]    = 0x00000000
        //   syn_curr_sram[20] = 32'd10
        //   syn_curr_sram[21] = 32'd5
        //   pot_sram[50]      = 32'd10
        //   pot_sram[51]      = 32'd5
        // ============================================================
        $display("Test 4: non-uniform column-major weights, no spike");

        for (i_init = 0; i_init < MEM_DEPTH; i_init = i_init + 1) begin
            u_syn_curr_mem.mem[i_init] = 32'd0;
            u_pot_mem.mem[i_init]      = 32'd0;
            u_spike_mem.mem[i_init]    = 32'd0;
        end
        cfg_write(32'hFFFF_003C, 32'h0553_0100); // M0 with has_ada=0 (adaptive off again)

        // Column-major weights at a NEW base (12) to force a weight-cache miss
        //   weight_sram[12] = column 0 = {W[0][0]=10, W[1][0]=5, slice2=0, slice3=0}
        //   weight_sram[13] = column 1 = {W[0][1]=10, W[1][1]=5, slice2=0, slice3=0}
        u_weight_mem.mem[12] = 32'h0000_050A;
        u_weight_mem.mem[13] = 32'h0000_050A;
        cfg_write(32'hFFFF_0004, 32'd12);   // weight_base_addr = 12

        // High threshold at a fresh address (48) → no spike → thresh-cache miss
        u_thresh_mem.mem[48] = 32'd50;
        u_thresh_mem.mem[49] = 32'd50;
        cfg_write(32'hFFFF_000C, 32'd48);   // thresh_base = 48

        @(negedge clk); start_new_block_i = 1'b1;
        @(negedge clk); start_new_block_i = 1'b0;

        wait_pipeline(timed_out);
        if (!timed_out) begin
            check_eq(u_spike_mem.mem[60],    32'h0000_0000,  "T4 spike_sram[60]");
            check_eq(u_syn_curr_mem.mem[20], 32'd10,         "T4 syn_curr_sram[20]");
            check_eq(u_syn_curr_mem.mem[21], 32'd5,          "T4 syn_curr_sram[21]");
            check_eq(u_pot_mem.mem[50],      32'd10,         "T4 pot_sram[50]");
            check_eq(u_pot_mem.mem[51],      32'd5,          "T4 pot_sram[51]");
        end

        // ============================================================
        // Test 5: LAST input is a NON-spike — regression for the
        //   weight-pass termination hang (2026-06-10). Non-spikes are
        //   gated out of the weight generator's act stream, so before
        //   the act_last_dumped_i fix it never saw act_data_last_i on
        //   a valid token and the pipeline hung here.
        //   Weights stay at base 12 (T4 layout): col 0 = {10, 5}.
        //   Only input 0 contributes: syn_curr = [10, 5].
        //   FMI update (pot zeroed, dcy=0.5):
        //     n0: diff=-10, decayed=-5,           new_mem=5; syn_o=5
        //     n1: diff=-5,  decayed=floor(-2.5)=-3, new_mem=2; syn_o=2
        //   Threshold 50 (still at base 48 from T4) → no spike.
        // ============================================================
        $display("Test 5: last input non-spiking (weight-pass hang regression)");

        for (i_init = 0; i_init < MEM_DEPTH; i_init = i_init + 1) begin
            u_syn_curr_mem.mem[i_init] = 32'd0;
            u_pot_mem.mem[i_init]      = 32'd0;
            u_spike_mem.mem[i_init]    = 32'd0;
        end

        // Acts at a NEW base (2) to force an act-cache miss:
        //   bit 0 = input 0 spikes, bit 1 = input 1 (the LAST) silent.
        u_act_mem.mem[2] = 32'h0000_0001;
        cfg_write(32'hFFFF_0000, 32'd2);         // act_base_addr = 2

        @(negedge clk); start_new_block_i = 1'b1;
        @(negedge clk); start_new_block_i = 1'b0;

        wait_pipeline(timed_out);
        if (!timed_out) begin
            check_eq(u_spike_mem.mem[60],    32'h0000_0000, "T5 spike_sram[60]");
            check_eq(u_syn_curr_mem.mem[20], 32'd5,         "T5 syn_curr_sram[20]");
            check_eq(u_syn_curr_mem.mem[21], 32'd2,         "T5 syn_curr_sram[21]");
            check_eq(u_pot_mem.mem[50],      32'd5,         "T5 pot_sram[50]");
            check_eq(u_pot_mem.mem[51],      32'd2,         "T5 pot_sram[51]");
        end

        // ============================================================
        // Test 6: $readmemh of FMI hex files into per-neuron memories.
        //
        //   Validates the loading path (convert_model.py output -> RTL via
        //   $readmemh) and the per-neuron decay path with REAL group4
        //   parameters from the recurrent_snn model.
        //
        //   Reuses T1's tiny full-connectivity setup (2 in, 2 out, all
        //   spiking, weight=10) so the inputs to the LIF are deterministic.
        //   What differs: dcy_syn, dcy_mem and thresh come from
        //   ../../fmi/mem_files/recurrent/group4_*.hex (neurons 0 and 1).
        //
        //   Pre-computed expected values from simulate_int_recurrent.py-
        //   equivalent integer math (recompute on the Python side and
        //   paste here if model files change):
        //     group4_dcy_syn[0..1] from hex:  filled by readmemh
        //     group4_dcy_mem[0..1] from hex:  filled by readmemh
        //     group4_thresh[0..1] = 0x4000 (1.0 at K_neuron=14)
        //
        //   With 2 spikes * weight 10 = 20 at K_mem=14 (bin_point=0,
        //   so neuron-scale syn = 20 too):
        //     diff = 0 - 20 = -20
        //     decayed_diff = (-20 * dcy_mem) >> 32  (signed)
        //     new_mem = decayed_diff + 20
        //     spike  = new_mem >= 16384 (=1.0 at K=14)   — won't fire for
        //              small new_mem; will be verified after first run.
        //
        //   First-run mode: $display the actual outputs so we can compute
        //   the goldens by hand from the loaded hex values, then convert
        //   to check_eq in a follow-up commit.
        // ============================================================
        $display("Test 6: $readmemh of group4 hex into dcy/thresh memories");

        for (i_init = 0; i_init < MEM_DEPTH; i_init = i_init + 1) begin
            u_syn_curr_mem.mem[i_init] = 32'd0;
            u_pot_mem.mem[i_init]      = 32'd0;
            u_spike_mem.mem[i_init]    = 32'd0;
        end

        // Load group4 per-neuron decay and threshold hex.
        // Paths are relative to wherever the simulator is invoked; this
        // testbench is normally launched from fmiSnnAcc/ via tbAccFmiSNN.bsh.
        $readmemh("../../fmi/mem_files/recurrent/group4_dcy_syn.hex",
                  u_dcy_syn_mem.mem);
        $readmemh("../../fmi/mem_files/recurrent/group4_dcy_mem.hex",
                  u_dcy_mem_mem.mem);
        $readmemh("../../fmi/mem_files/recurrent/group4_thresh.hex",
                  u_thresh_mem.mem);

        // Restore the simple T1 act/weight: 2 inputs all spiking, weight=10.
        u_act_mem.mem[0]    = 32'hFFFF_FFFF;
        u_weight_mem.mem[0] = 32'h0A0A_0A0A;

        // Repoint all the base addresses to 0 (the hex files load starting at
        // index 0 in each SRAM).
        cfg_write(32'hFFFF_0000, 32'd0);    // act_base_addr      = 0
        cfg_write(32'hFFFF_0004, 32'd0);    // weight_base_addr   = 0
        cfg_write(32'hFFFF_0008, 32'd0);    // syn_curr_base_addr = 0
        cfg_write(32'hFFFF_000C, 32'd0);    // thresh_base        = 0
        cfg_write(32'hFFFF_0010, 32'd0);    // pot_base           = 0
        cfg_write(32'hFFFF_0014, 32'd0);    // spike_base         = 0
        cfg_write(32'hFFFF_0018, 32'd0);    // dcy_syn_base       = 0
        cfg_write(32'hFFFF_001C, 32'd0);    // dcy_mem_base       = 0

        // S1 still has last_neuron_idx=1 (2 neurons); has_ada still 0.
        // M0 = 0x0553_0100 unchanged from T1 (no ada, full conn, 8-bit weights).
        cfg_write(32'hFFFF_003C, 32'h0553_0100);

        @(negedge clk); start_new_block_i = 1'b1;
        @(negedge clk); start_new_block_i = 1'b0;

        wait_pipeline(timed_out);
        if (!timed_out) begin
            $display("  T6 neuron 0:  syn_curr=0x%08h  pot=0x%08h  spike_word=0x%08h",
                     u_syn_curr_mem.mem[0], u_pot_mem.mem[0], u_spike_mem.mem[0]);
            $display("  T6 neuron 1:  syn_curr=0x%08h  pot=0x%08h",
                     u_syn_curr_mem.mem[1], u_pot_mem.mem[1]);
            $display("  T6 loaded dcy_syn[0]=0x%08h  dcy_mem[0]=0x%08h  thresh[0]=0x%08h",
                     u_dcy_syn_mem.mem[0], u_dcy_mem_mem.mem[0], u_thresh_mem.mem[0]);
            $display("  T6 loaded dcy_syn[1]=0x%08h  dcy_mem[1]=0x%08h  thresh[1]=0x%08h",
                     u_dcy_syn_mem.mem[1], u_dcy_mem_mem.mem[1], u_thresh_mem.mem[1]);
            // Threshold from hex is 0x4000 (=1.0 at K=14). With weight=10 (very small
            // at K=14), new_mem will be far below threshold -> no spike expected.
            //
            // Expected outputs (computed by hand, agreeing with simulate_int_recurrent.py):
            //   MAC: 2 spikes * weight 10 = 20 at K_mem=14 (bin_point=0 -> neuron-scale 20)
            //   For each neuron i:
            //     syn_writeback = (20 * dcy_syn[i]) >> 32   (signed * unsigned Q0.32)
            //     new_mem       = ((-20) * dcy_mem[i]) >> 32 + 20
            //     spike         = new_mem >= 0x4000  -> 0 (both)
            //   Neuron 0: dcy_syn=0xfae9da00 -> 20*4209758720>>32 = 19 -> 0x13
            //             dcy_mem=0xfd07d300 -> -20*4245242624>>32 = -20, new_mem=0
            //   Neuron 1: dcy_syn=0xfc493400 -> 20*4232029184>>32 = 19 -> 0x13
            //             dcy_mem=0xfb6b5800 -> -20*4218747392>>32 = -20, new_mem=0
            check_eq(u_syn_curr_mem.mem[0], 32'h0000_0013, "T6 syn_curr[0] (decay of 20)");
            check_eq(u_syn_curr_mem.mem[1], 32'h0000_0013, "T6 syn_curr[1] (decay of 20)");
            check_eq(u_pot_mem.mem[0],      32'd0,         "T6 pot[0] (new_mem=0)");
            check_eq(u_pot_mem.mem[1],      32'd0,         "T6 pot[1] (new_mem=0)");
            check_eq(u_spike_mem.mem[0], 32'h0000_0000, "T6 spike_sram[0] (no spike)");
            check_eq(u_thresh_mem.mem[0], 32'h0000_4000, "T6 thresh[0] = 1.0 at K=14");
            check_eq(u_thresh_mem.mem[1], 32'h0000_4000, "T6 thresh[1] = 1.0 at K=14");
        end

        // ============================================================
        // Test 7: $readmemh of group3 adaptive-LIF hex files.
        //
        //   Counterpart to T6 but for the adaptive datapath. Loads all
        //   six per-neuron memories for group3 (dcy_syn, dcy_mem, thresh,
        //   b_eff, dcy_ada, scl_ada) from convert_model.py output, and
        //   pre-loads ada to a known non-zero value so the b_eff*ada
        //   multiplier produces an observable ada_corr.
        //
        //   T1 inputs: 2 spikes * weight 10 = 20 at K_mem=14, bp=0.
        //   With ada_init = 0x4000_0000 (= 0.25 in Q0.32):
        //     ada_corr[i] = (b_eff[i] * 0x4000_0000) >> 32   (signed * unsigned)
        //                 = b_eff[i] / 4 (arithmetic, signed)
        //     eff_syn[i]  = 20 - ada_corr[i]
        //     diff[i]     = 0 - eff_syn[i]
        //     decayed_diff[i] = (diff[i] * dcy_mem[i]) >> 32
        //     new_mem[i]  = decayed_diff[i] + eff_syn[i]
        //     spike[i]    = new_mem[i] >= 0x4000          -> 0 (small new_mem)
        //     syn_wb[i]   = (20 * dcy_syn[i]) >> 32
        //     ada_wb[i]   = ((0x4000_0000 * dcy_ada[i]) >> 32) + spike[i]*scl_ada[i]
        //
        //   Same first-run mode: $display loaded hex + outputs to compute
        //   expected values, then convert to check_eq.
        // ============================================================
        $display("Test 7: $readmemh of group3 hex into adaptive-LIF memories");

        for (i_init = 0; i_init < MEM_DEPTH; i_init = i_init + 1) begin
            u_syn_curr_mem.mem[i_init] = 32'd0;
            u_pot_mem.mem[i_init]      = 32'd0;
            u_spike_mem.mem[i_init]    = 32'd0;
            u_ada_mem.mem[i_init]      = 32'd0;
        end

        $readmemh("../../fmi/mem_files/recurrent/group3_dcy_syn.hex",
                  u_dcy_syn_mem.mem);
        $readmemh("../../fmi/mem_files/recurrent/group3_dcy_mem.hex",
                  u_dcy_mem_mem.mem);
        $readmemh("../../fmi/mem_files/recurrent/group3_thresh.hex",
                  u_thresh_mem.mem);
        $readmemh("../../fmi/mem_files/recurrent/group3_b_eff.hex",
                  u_b_eff_mem.mem);
        $readmemh("../../fmi/mem_files/recurrent/group3_dcy_ada.hex",
                  u_dcy_ada_mem.mem);
        $readmemh("../../fmi/mem_files/recurrent/group3_scl_ada.hex",
                  u_scl_ada_mem.mem);

        // Pre-load ada to 0.25 in Q0.32 to make ada_corr observable.
        u_ada_mem.mem[0] = 32'h4000_0000;
        u_ada_mem.mem[1] = 32'h4000_0000;

        // Same T1 act/weight setup.
        u_act_mem.mem[0]    = 32'hFFFF_FFFF;
        u_weight_mem.mem[0] = 32'h0A0A_0A0A;

        // Repoint the ada-related base addresses to 0 (still pointing at T1
        // defaults 90/100/110/120 otherwise). Must come BEFORE the trigger.
        cfg_write(32'hFFFF_0020, 32'd0);    // ada_base_addr     = 0
        cfg_write(32'hFFFF_0024, 32'd0);    // b_eff_base_addr   = 0
        cfg_write(32'hFFFF_0028, 32'd0);    // dcy_ada_base_addr = 0
        cfg_write(32'hFFFF_002C, 32'd0);    // scl_ada_base_addr = 0

        // M0 with has_ada=1 (bit 30 set), other fields same as T1.
        cfg_write(32'hFFFF_003C, 32'h4553_0100);

        @(negedge clk); start_new_block_i = 1'b1;
        @(negedge clk); start_new_block_i = 1'b0;

        wait_pipeline(timed_out);
        if (!timed_out) begin
            $display("  T7 loaded dcy_syn[0]=0x%08h  dcy_mem[0]=0x%08h",
                     u_dcy_syn_mem.mem[0], u_dcy_mem_mem.mem[0]);
            $display("  T7 loaded thresh[0]=0x%08h   b_eff[0]=0x%08h",
                     u_thresh_mem.mem[0],  u_b_eff_mem.mem[0]);
            $display("  T7 loaded dcy_ada[0]=0x%08h  scl_ada[0]=0x%08h",
                     u_dcy_ada_mem.mem[0], u_scl_ada_mem.mem[0]);
            $display("  T7 loaded dcy_syn[1]=0x%08h  dcy_mem[1]=0x%08h",
                     u_dcy_syn_mem.mem[1], u_dcy_mem_mem.mem[1]);
            $display("  T7 loaded thresh[1]=0x%08h   b_eff[1]=0x%08h",
                     u_thresh_mem.mem[1],  u_b_eff_mem.mem[1]);
            $display("  T7 loaded dcy_ada[1]=0x%08h  scl_ada[1]=0x%08h",
                     u_dcy_ada_mem.mem[1], u_scl_ada_mem.mem[1]);
            $display("  T7 neuron 0:  syn=0x%08h  pot=0x%08h  ada=0x%08h",
                     u_syn_curr_mem.mem[0], u_pot_mem.mem[0], u_ada_mem.mem[0]);
            $display("  T7 neuron 1:  syn=0x%08h  pot=0x%08h  ada=0x%08h",
                     u_syn_curr_mem.mem[1], u_pot_mem.mem[1], u_ada_mem.mem[1]);
            $display("  T7 spike_word=0x%08h", u_spike_mem.mem[0]);

            // Expected outputs (verified against simulate_int_recurrent.py
            // primitives, all multiplications use the upper 32 bits of
            // signed * unsigned products):
            //   Neuron 0 -- b_eff=0xae(=+174), dcy_mem=0xfb3f6100, dcy_ada=0xfdf6ef00
            //     ada_corr      = (174 * 0x40000000) >> 32 = 43
            //     eff_syn       = 20 - 43 = -23
            //     diff          = 0 - (-23) = 23
            //     decayed_diff  = (23 * 0xfb3f6100) >> 32 = 22
            //     new_mem       = 22 + (-23) = -1 = 0xFFFFFFFF
            //     spike         = -1 >= 0x4000 -> 0
            //     syn_writeback = (20 * 0xfc2afd00) >> 32 = 19 = 0x13
            //     ada_writeback = (0x40000000 * 0xfdf6ef00) >> 32 = 0x3F7DBBC0
            //   Neuron 1 -- b_eff=0x03(=+3), dcy_mem=0xfd327700, dcy_ada=0xf8470a00
            //     ada_corr      = (3 * 0x40000000) >> 32 = 0
            //     eff_syn       = 20
            //     decayed_diff  = (-20 * 0xfd327700) >> 32 = -20
            //     new_mem       = -20 + 20 = 0
            //     syn_writeback = 0x13 (same dcy_syn rounding)
            //     ada_writeback = (0x40000000 * 0xf8470a00) >> 32 = 0x3E11C280
            check_eq(u_syn_curr_mem.mem[0], 32'h0000_0013, "T7 syn_curr[0]");
            check_eq(u_syn_curr_mem.mem[1], 32'h0000_0013, "T7 syn_curr[1]");
            check_eq(u_pot_mem.mem[0],      32'hFFFF_FFFF, "T7 pot[0] (-1, no spike)");
            check_eq(u_pot_mem.mem[1],      32'h0000_0000, "T7 pot[1] (0)");
            check_eq(u_ada_mem.mem[0],      32'h3F7D_BBC0, "T7 ada[0] (decayed 0.25)");
            check_eq(u_ada_mem.mem[1],      32'h3E11_C280, "T7 ada[1] (decayed 0.25)");
            check_eq(u_spike_mem.mem[0],    32'h0000_0000, "T7 spike_sram[0] (no spike)");
            check_eq(u_thresh_mem.mem[0],   32'h0000_4000, "T7 thresh[0] = 1.0 at K=14");
            check_eq(u_thresh_mem.mem[1],   32'h0000_4000, "T7 thresh[1] = 1.0 at K=14");
        end

        // ============================================================
        // Test 8: native multi-channel conv (MC POC)
        //
        //   Cin=2, Cout=2, Kx=3, Ky=1, in_x=4, stride=1, pad=1, out_x=4.
        //   12 hand-computable 32-bit weights, PyTorch row-major
        //   (Cout, Cin, Kx) flatten:
        //     w_idx 0..2  = w[0][0][0..2] = 1, 2, 3
        //     w_idx 3..5  = w[0][1][0..2] = 4, 5, 6
        //     w_idx 6..8  = w[1][0][0..2] = 7, 8, 9
        //     w_idx 9..11 = w[1][1][0..2] = 10, 11, 12
        //
        //   Single input spike at (cin=0, in_x=2). With stride=1, pad=1:
        //     for kx in 0..2: out_x = act_x + kx - offset = 2 + kx - 1 in {1,2,3}
        //   Hardware projects this input to outputs (cout, out_x):
        //     kx=0 -> out_x=1, weight = w[cout][0][0]
        //     kx=1 -> out_x=2, weight = w[cout][0][1]
        //     kx=2 -> out_x=3, weight = w[cout][0][2]
        //
        //   Channel-major syn_curr layout (cout * out_x_len + out_x):
        //     syn_curr_mem[0] = (cout=0, x=0) = 0      (no kernel hits)
        //     syn_curr_mem[1] = (cout=0, x=1) = 1
        //     syn_curr_mem[2] = (cout=0, x=2) = 2
        //     syn_curr_mem[3] = (cout=0, x=3) = 3
        //     syn_curr_mem[4] = (cout=1, x=0) = 0
        //     syn_curr_mem[5] = (cout=1, x=1) = 7
        //     syn_curr_mem[6] = (cout=1, x=2) = 8
        //     syn_curr_mem[7] = (cout=1, x=3) = 9
        //
        //   sp_skip_neuron=1: dispatch SP only, no NP (no LIF decay applied
        //   to syn_curr, so the raw MAC accumulator is observable).
        //   weight_mode=2'b10 (conv), weight_sz=5 (32-bit), weights_per_word=1.
        // ============================================================
        $display("Test 8: native multi-channel conv (Cin=2, Cout=2, K=3)");

        // Zero everything, and explicitly drive the per-neuron LIF memories
        // to deterministic values so we can predict the NP writeback exactly:
        //   dcy_syn = 0xFFFFFFFF (almost 1.0): syn_wb = (syn*0xFFFFFFFF)>>32
        //                                          = syn - 1 for positive syn
        //                                          = 0     for syn == 0
        //   dcy_mem = 0 (no membrane carry across timesteps; no spike since
        //              new_mem = 0*(pot-syn)+syn = syn = small)
        //   thresh = max int32 (so new_mem never reaches threshold; no spike;
        //                       pot writeback = new_mem ignored by this test)
        for (i_init = 0; i_init < MEM_DEPTH; i_init = i_init + 1) begin
            u_syn_curr_mem.mem[i_init] = 32'd0;
            u_pot_mem.mem[i_init]      = 32'd0;
            u_spike_mem.mem[i_init]    = 32'd0;
            u_weight_mem.mem[i_init]   = 32'd0;
            u_act_mem.mem[i_init]      = 32'd0;
            u_dcy_syn_mem.mem[i_init]  = 32'hFFFFFFFF;
            u_dcy_mem_mem.mem[i_init]  = 32'h00000000;
            u_thresh_mem.mem[i_init]   = 32'h7FFFFFFF;
        end

        // 12 32-bit weights in PyTorch row-major (Cout, Cin, Kx) order.
        u_weight_mem.mem[ 0] = 32'd1;
        u_weight_mem.mem[ 1] = 32'd2;
        u_weight_mem.mem[ 2] = 32'd3;
        u_weight_mem.mem[ 3] = 32'd4;
        u_weight_mem.mem[ 4] = 32'd5;
        u_weight_mem.mem[ 5] = 32'd6;
        u_weight_mem.mem[ 6] = 32'd7;
        u_weight_mem.mem[ 7] = 32'd8;
        u_weight_mem.mem[ 8] = 32'd9;
        u_weight_mem.mem[ 9] = 32'd10;
        u_weight_mem.mem[10] = 32'd11;
        u_weight_mem.mem[11] = 32'd12;

        // Single spike at (cin=0, in_x=2): flat index = 0*4 + 2 = 2 -> bit 2.
        u_act_mem.mem[0] = 32'h0000_0004;

        // Drive MC sizing wires for T8.
        tb_sp_cin_len  = 7'd2;
        tb_sp_cout_len = 7'd2;

        // PACKED_FMI config for T8.
        // Base addresses all 0:
        cfg_write(32'hFFFF_0000, 32'd0);    // act_base_addr
        cfg_write(32'hFFFF_0004, 32'd0);    // weight_base_addr
        cfg_write(32'hFFFF_0008, 32'd0);    // syn_curr_base_addr
        cfg_write(32'hFFFF_000C, 32'd0);    // thresh_base (unused with skip)
        cfg_write(32'hFFFF_0010, 32'd0);    // pot_base
        cfg_write(32'hFFFF_0014, 32'd0);    // spike_base
        cfg_write(32'hFFFF_0018, 32'd0);    // dcy_syn_base
        cfg_write(32'hFFFF_001C, 32'd0);    // dcy_mem_base
        cfg_write(32'hFFFF_0020, 32'd0);    // ada_base
        cfg_write(32'hFFFF_0024, 32'd0);    // b_eff_base
        cfg_write(32'hFFFF_0028, 32'd0);    // dcy_ada_base
        cfg_write(32'hFFFF_002C, 32'd0);    // scl_ada_base

        // S0: in_x_len=4 | out_x_len=4
        cfg_write(32'hFFFF_0030, 32'h0004_0004);
        // S1: last_neuron_idx=7 | rows_per_neuron=12
        cfg_write(32'hFFFF_0034, 32'h0007_000C);
        // S2: total_timesteps=1
        cfg_write(32'hFFFF_0038, 32'h0000_0001);
        // M0: skip_neuron=0, np_mode=0, weights_per_word=1, bin_point=0,
        //     weight_sz=5(32b), syn_curr_sz=5, pot_sz=5, weight_mode=10(conv), has_ada=0
        //     (Debugging: NP runs, but the LIF dynamics simply decay syn_curr
        //      slightly — we check the EXACT raw syn_curr via $display.)
        cfg_write(32'hFFFF_003C, 32'h2555_0040);
        // Boot regs (conv params):
        cfg_write(32'hFFFF_005C, 32'd4);   // weight_idx_sz = 4 (Cout*Cin*K=12 fits in 4)
        cfg_write(32'hFFFF_0074, 32'd3);   // x_kernel_len = 3
        cfg_write(32'hFFFF_007C, 32'd1);   // x_kernel_step = 1
        cfg_write(32'hFFFF_0084, 32'd1);   // x_kernel_offset = 1 (pad=1)

        @(negedge clk); start_new_block_i = 1'b1;
        @(negedge clk); start_new_block_i = 1'b0;

        wait_pipeline(timed_out);
        if (!timed_out) begin
            $display("  T8 syn_curr_mem[0..7] = %0d %0d %0d %0d  %0d %0d %0d %0d",
                     u_syn_curr_mem.mem[0], u_syn_curr_mem.mem[1],
                     u_syn_curr_mem.mem[2], u_syn_curr_mem.mem[3],
                     u_syn_curr_mem.mem[4], u_syn_curr_mem.mem[5],
                     u_syn_curr_mem.mem[6], u_syn_curr_mem.mem[7]);
            // Expected = MAC_raw - 1 for syn_writeback (because dcy_syn = 0xFFFFFFFF
            // gives (syn * 0xFFFFFFFF) >> 32 = syn - 1 for positive syn). For
            // zero MAC contributions the writeback is 0 (0 - 1 isn't applied
            // because 0 * 0xFFFFFFFF = 0).
            // The shape of the syn_curr image — which cells got contributions
            // from which weights — is what this test validates. MAC values:
            //   (0,0)=0  (0,1)=1  (0,2)=2  (0,3)=3
            //   (1,0)=0  (1,1)=7  (1,2)=8  (1,3)=9
            check_eq(u_syn_curr_mem.mem[0], 32'd0, "T8 syn(0,0) (no kernel hit)");
            check_eq(u_syn_curr_mem.mem[1], 32'd0, "T8 syn(0,1) MAC=1 -> wb=0");
            check_eq(u_syn_curr_mem.mem[2], 32'd1, "T8 syn(0,2) MAC=2 -> wb=1 (= w[0][0][1]-1)");
            check_eq(u_syn_curr_mem.mem[3], 32'd2, "T8 syn(0,3) MAC=3 -> wb=2 (= w[0][0][2]-1)");
            check_eq(u_syn_curr_mem.mem[4], 32'd0, "T8 syn(1,0) (no kernel hit)");
            check_eq(u_syn_curr_mem.mem[5], 32'd6, "T8 syn(1,1) MAC=7 -> wb=6 (= w[1][0][0]-1)");
            check_eq(u_syn_curr_mem.mem[6], 32'd7, "T8 syn(1,2) MAC=8 -> wb=7 (= w[1][0][1]-1)");
            check_eq(u_syn_curr_mem.mem[7], 32'd8, "T8 syn(1,3) MAC=9 -> wb=8 (= w[1][0][2]-1)");
        end

        // Reset MC sizing back to 1 in case future tests are added.
        tb_sp_cin_len  = 7'd1;
        tb_sp_cout_len = 7'd1;

        $display("=== tb_acc_fmiSnnMC_processor: %0d failure(s) ===", errors);
        if (errors == 0) $display("PASS"); else $display("FAIL");
        $finish;
    end

    initial begin
        #500000;
        $display("FAIL: global simulation timeout");
        $finish;
    end

endmodule


// ====================================================================
//  sram_model  (shared with tb_update_state_for_neuron if needed)
// ====================================================================
module sram_model #(
    parameter DATA_W = 32,
    parameter DEPTH  = 256
)(
    input  wire              clk,
    input  wire              we,
    input  wire              re,
    input  wire       [15:0] addr,   // widened 8 -> 16 bits for T6_LAYER (group4 has 1920 entries)
    input  wire [DATA_W-1:0] wdata,
    output reg  [DATA_W-1:0] rdata
);
    reg [DATA_W-1:0] mem [0:DEPTH-1];
    integer i;
    initial begin
        for (i = 0; i < DEPTH; i = i + 1)
            mem[i] = {DATA_W{1'b0}};
        rdata = {DATA_W{1'b0}};
    end
    always @(posedge clk) begin
        if (we) mem[addr] <= wdata;
        if (re) rdata     <= mem[addr];
    end
endmodule
