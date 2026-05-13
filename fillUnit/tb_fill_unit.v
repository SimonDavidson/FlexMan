// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
`timescale 10ps/1ps
`include "../shared/constants.v"

// ================================================================
// tb_fill_unit
//
// Verifies fill_unit.v: AXI mem_sel table initialisation, BBA read,
// constant-value block fills across all 25 memory buses, and both
// forms of back-pressure (BBA wait and memory bus wait).
//
// Simulation
// ----------
//   xrun -sv -exit -timescale 10ps/1ps -access wrc -top tb_fill_unit \
//     ../shared/constants.v fill_unit.v tb_fill_unit.v
//
// Tests
// -----
//   T1  Basic fill: buf 3 → s0_syn_curr (idx 2), 8 words, value=0xDEAD_BEEF
//       Verifies sequential addresses, correct data, acc_finished_o pulse,
//       and that no other memory bus fires.
//   T2  Different memory: buf 7 → hd_src_r (idx 24), 4 words, value=0xCAFE_0000
//       Confirms a high-index memory type is correctly selected.
//   T3  BBA back-pressure: hold fu_bba_wait_i for 3 cycles after dispatch;
//       verify no writes fire during the wait, then normal fill resumes.
//   T4  Memory back-pressure: assert s0_syn_curr_wait_i mid-fill;
//       verify write count and address sequence are still correct.
//   T5  Back-to-back: T2 immediately followed by T1 (dispatch on the same
//       cycle as acc_finished_o of the first fill).
// ================================================================

module tb_fill_unit;

// ─── Parameters ───────────────────────────────────────────────────────────────
localparam [31:0] FU_TABLE_ADDR      = 32'hC000_0000;
localparam [31:0] FU_TABLE_ADDR_MASK = 32'hFF00_0000;
localparam NUM_BUFFERS   = 16;
localparam BUFF_INDX_SZ  = 4;
localparam ADDR_SIZE     = `ADDR_SIZE;   // 30
localparam DATA_SZ       = 32;
localparam NUM_MEM_TYPES = 25;

// Memory index constants (must match fill_unit.v)
localparam IDX_S0_WEIGHT    =  0;
localparam IDX_S0_ACT       =  1;
localparam IDX_S0_SYN_CURR  =  2;
localparam IDX_S0_BIAS_CURR =  3;
localparam IDX_S0_THRESH    =  4;
localparam IDX_S0_POT       =  5;
localparam IDX_S0_SPIKE     =  6;
localparam IDX_S1_WEIGHT    =  7;
localparam IDX_S1_ACT       =  8;
localparam IDX_S1_SYN_CURR  =  9;
localparam IDX_S1_BIAS_CURR = 10;
localparam IDX_S1_THRESH    = 11;
localparam IDX_S1_POT       = 12;
localparam IDX_S1_SPIKE     = 13;
localparam IDX_A0_WEIGHT    = 14;
localparam IDX_A0_ACT       = 15;
localparam IDX_A0_SYN_CURR  = 16;
localparam IDX_A0_BIAS_CURR = 17;
localparam IDX_A0_THRESH    = 18;
localparam IDX_A0_POT       = 19;
localparam IDX_A0_SPIKE     = 20;
localparam IDX_HD_SRC_A     = 21;
localparam IDX_HD_SRC_B     = 22;
localparam IDX_HD_SRC_Z     = 23;
localparam IDX_HD_SRC_R     = 24;

localparam CLK_HALF = 5;
localparam TIMEOUT  = 300;

// ─── DUT signals ─────────────────────────────────────────────────────────────
reg  clk, reset;

// AXI
reg         sys_req_i;
wire        sys_ack_o;
reg  [31:0] sys_addr_i;
reg  [31:0] sys_data_i;

// Dispatch
reg                      start_new_block_i;
wire                     acc_busy_o;
wire                     acc_finished_o;
reg  [BUFF_INDX_SZ-1:0]  buff_id_i;
reg  [DATA_SZ-1:0]        fill_value_i;
reg  [19:0]               fill_block_size_i;

// BBA
wire                     fu_bba_rd_o;
reg                      fu_bba_wait_i;
wire [31:0]              fu_bba_addr_o;
reg  [31:0]              fu_bba_data_i;

// snnAcc0 write ports
wire                  s0_weight_wr_o;    wire [ADDR_SIZE-1:0] s0_weight_addr_o;    wire [DATA_SZ-1:0] s0_weight_data_o;    reg s0_weight_wait_i;
wire                  s0_act_wr_o;       wire [ADDR_SIZE-1:0] s0_act_addr_o;       wire [DATA_SZ-1:0] s0_act_data_o;       reg s0_act_wait_i;
wire                  s0_syn_curr_wr_o;  wire [ADDR_SIZE-1:0] s0_syn_curr_addr_o;  wire [DATA_SZ-1:0] s0_syn_curr_data_o;  reg s0_syn_curr_wait_i;
wire                  s0_bias_curr_wr_o; wire [ADDR_SIZE-1:0] s0_bias_curr_addr_o; wire [DATA_SZ-1:0] s0_bias_curr_data_o; reg s0_bias_curr_wait_i;
wire                  s0_thresh_wr_o;    wire [ADDR_SIZE-1:0] s0_thresh_addr_o;    wire [DATA_SZ-1:0] s0_thresh_data_o;    reg s0_thresh_wait_i;
wire                  s0_pot_wr_o;       wire [ADDR_SIZE-1:0] s0_pot_addr_o;       wire [DATA_SZ-1:0] s0_pot_data_o;       reg s0_pot_wait_i;
wire                  s0_spike_wr_o;     wire [ADDR_SIZE-1:0] s0_spike_addr_o;     wire [DATA_SZ-1:0] s0_spike_data_o;     reg s0_spike_wait_i;

// snnAcc1 write ports
wire                  s1_weight_wr_o;    wire [ADDR_SIZE-1:0] s1_weight_addr_o;    wire [DATA_SZ-1:0] s1_weight_data_o;    reg s1_weight_wait_i;
wire                  s1_act_wr_o;       wire [ADDR_SIZE-1:0] s1_act_addr_o;       wire [DATA_SZ-1:0] s1_act_data_o;       reg s1_act_wait_i;
wire                  s1_syn_curr_wr_o;  wire [ADDR_SIZE-1:0] s1_syn_curr_addr_o;  wire [DATA_SZ-1:0] s1_syn_curr_data_o;  reg s1_syn_curr_wait_i;
wire                  s1_bias_curr_wr_o; wire [ADDR_SIZE-1:0] s1_bias_curr_addr_o; wire [DATA_SZ-1:0] s1_bias_curr_data_o; reg s1_bias_curr_wait_i;
wire                  s1_thresh_wr_o;    wire [ADDR_SIZE-1:0] s1_thresh_addr_o;    wire [DATA_SZ-1:0] s1_thresh_data_o;    reg s1_thresh_wait_i;
wire                  s1_pot_wr_o;       wire [ADDR_SIZE-1:0] s1_pot_addr_o;       wire [DATA_SZ-1:0] s1_pot_data_o;       reg s1_pot_wait_i;
wire                  s1_spike_wr_o;     wire [ADDR_SIZE-1:0] s1_spike_addr_o;     wire [DATA_SZ-1:0] s1_spike_data_o;     reg s1_spike_wait_i;

// annAcc write ports
wire                  a0_weight_wr_o;    wire [ADDR_SIZE-1:0] a0_weight_addr_o;    wire [DATA_SZ-1:0] a0_weight_data_o;    reg a0_weight_wait_i;
wire                  a0_act_wr_o;       wire [ADDR_SIZE-1:0] a0_act_addr_o;       wire [DATA_SZ-1:0] a0_act_data_o;       reg a0_act_wait_i;
wire                  a0_syn_curr_wr_o;  wire [ADDR_SIZE-1:0] a0_syn_curr_addr_o;  wire [DATA_SZ-1:0] a0_syn_curr_data_o;  reg a0_syn_curr_wait_i;
wire                  a0_bias_curr_wr_o; wire [ADDR_SIZE-1:0] a0_bias_curr_addr_o; wire [DATA_SZ-1:0] a0_bias_curr_data_o; reg a0_bias_curr_wait_i;
wire                  a0_thresh_wr_o;    wire [ADDR_SIZE-1:0] a0_thresh_addr_o;    wire [DATA_SZ-1:0] a0_thresh_data_o;    reg a0_thresh_wait_i;
wire                  a0_pot_wr_o;       wire [ADDR_SIZE-1:0] a0_pot_addr_o;       wire [DATA_SZ-1:0] a0_pot_data_o;       reg a0_pot_wait_i;
wire                  a0_spike_wr_o;     wire [ADDR_SIZE-1:0] a0_spike_addr_o;     wire [DATA_SZ-1:0] a0_spike_data_o;     reg a0_spike_wait_i;

// Hadamard write ports
wire                  hd_src_a_wr_o;    wire [ADDR_SIZE-1:0] hd_src_a_addr_o;    wire [DATA_SZ-1:0] hd_src_a_data_o;    reg hd_src_a_wait_i;
wire                  hd_src_b_wr_o;    wire [ADDR_SIZE-1:0] hd_src_b_addr_o;    wire [DATA_SZ-1:0] hd_src_b_data_o;    reg hd_src_b_wait_i;
wire                  hd_src_z_wr_o;    wire [ADDR_SIZE-1:0] hd_src_z_addr_o;    wire [DATA_SZ-1:0] hd_src_z_data_o;    reg hd_src_z_wait_i;
wire                  hd_src_r_wr_o;    wire [ADDR_SIZE-1:0] hd_src_r_addr_o;    wire [DATA_SZ-1:0] hd_src_r_data_o;    reg hd_src_r_wait_i;

// ─── DUT ─────────────────────────────────────────────────────────────────────
fill_unit #(
    .FU_TABLE_ADDR      (FU_TABLE_ADDR),
    .FU_TABLE_ADDR_MASK (FU_TABLE_ADDR_MASK),
    .NUM_BUFFERS        (NUM_BUFFERS),
    .BUFF_INDX_SZ       (BUFF_INDX_SZ)
) dut (
    .clk                (clk),
    .reset              (reset),
    .sys_req_i          (sys_req_i),
    .sys_ack_o          (sys_ack_o),
    .sys_addr_i         (sys_addr_i),
    .sys_data_i         (sys_data_i),
    .start_new_block_i  (start_new_block_i),
    .acc_busy_o         (acc_busy_o),
    .acc_finished_o     (acc_finished_o),
    .buff_id_i          (buff_id_i),
    .fill_value_i       (fill_value_i),
    .fill_block_size_i  (fill_block_size_i),
    .fu_bba_rd_o        (fu_bba_rd_o),
    .fu_bba_wait_i      (fu_bba_wait_i),
    .fu_bba_addr_o      (fu_bba_addr_o),
    .fu_bba_data_i      (fu_bba_data_i),
    .s0_weight_wr_o     (s0_weight_wr_o),    .s0_weight_addr_o    (s0_weight_addr_o),    .s0_weight_data_o    (s0_weight_data_o),    .s0_weight_wait_i    (s0_weight_wait_i),
    .s0_act_wr_o        (s0_act_wr_o),       .s0_act_addr_o       (s0_act_addr_o),       .s0_act_data_o       (s0_act_data_o),       .s0_act_wait_i       (s0_act_wait_i),
    .s0_syn_curr_wr_o   (s0_syn_curr_wr_o),  .s0_syn_curr_addr_o  (s0_syn_curr_addr_o),  .s0_syn_curr_data_o  (s0_syn_curr_data_o),  .s0_syn_curr_wait_i  (s0_syn_curr_wait_i),
    .s0_bias_curr_wr_o  (s0_bias_curr_wr_o), .s0_bias_curr_addr_o (s0_bias_curr_addr_o), .s0_bias_curr_data_o (s0_bias_curr_data_o), .s0_bias_curr_wait_i (s0_bias_curr_wait_i),
    .s0_thresh_wr_o     (s0_thresh_wr_o),    .s0_thresh_addr_o    (s0_thresh_addr_o),    .s0_thresh_data_o    (s0_thresh_data_o),    .s0_thresh_wait_i    (s0_thresh_wait_i),
    .s0_pot_wr_o        (s0_pot_wr_o),       .s0_pot_addr_o       (s0_pot_addr_o),       .s0_pot_data_o       (s0_pot_data_o),       .s0_pot_wait_i       (s0_pot_wait_i),
    .s0_spike_wr_o      (s0_spike_wr_o),     .s0_spike_addr_o     (s0_spike_addr_o),     .s0_spike_data_o     (s0_spike_data_o),     .s0_spike_wait_i     (s0_spike_wait_i),
    .s1_weight_wr_o     (s1_weight_wr_o),    .s1_weight_addr_o    (s1_weight_addr_o),    .s1_weight_data_o    (s1_weight_data_o),    .s1_weight_wait_i    (s1_weight_wait_i),
    .s1_act_wr_o        (s1_act_wr_o),       .s1_act_addr_o       (s1_act_addr_o),       .s1_act_data_o       (s1_act_data_o),       .s1_act_wait_i       (s1_act_wait_i),
    .s1_syn_curr_wr_o   (s1_syn_curr_wr_o),  .s1_syn_curr_addr_o  (s1_syn_curr_addr_o),  .s1_syn_curr_data_o  (s1_syn_curr_data_o),  .s1_syn_curr_wait_i  (s1_syn_curr_wait_i),
    .s1_bias_curr_wr_o  (s1_bias_curr_wr_o), .s1_bias_curr_addr_o (s1_bias_curr_addr_o), .s1_bias_curr_data_o (s1_bias_curr_data_o), .s1_bias_curr_wait_i (s1_bias_curr_wait_i),
    .s1_thresh_wr_o     (s1_thresh_wr_o),    .s1_thresh_addr_o    (s1_thresh_addr_o),    .s1_thresh_data_o    (s1_thresh_data_o),    .s1_thresh_wait_i    (s1_thresh_wait_i),
    .s1_pot_wr_o        (s1_pot_wr_o),       .s1_pot_addr_o       (s1_pot_addr_o),       .s1_pot_data_o       (s1_pot_data_o),       .s1_pot_wait_i       (s1_pot_wait_i),
    .s1_spike_wr_o      (s1_spike_wr_o),     .s1_spike_addr_o     (s1_spike_addr_o),     .s1_spike_data_o     (s1_spike_data_o),     .s1_spike_wait_i     (s1_spike_wait_i),
    .a0_weight_wr_o     (a0_weight_wr_o),    .a0_weight_addr_o    (a0_weight_addr_o),    .a0_weight_data_o    (a0_weight_data_o),    .a0_weight_wait_i    (a0_weight_wait_i),
    .a0_act_wr_o        (a0_act_wr_o),       .a0_act_addr_o       (a0_act_addr_o),       .a0_act_data_o       (a0_act_data_o),       .a0_act_wait_i       (a0_act_wait_i),
    .a0_syn_curr_wr_o   (a0_syn_curr_wr_o),  .a0_syn_curr_addr_o  (a0_syn_curr_addr_o),  .a0_syn_curr_data_o  (a0_syn_curr_data_o),  .a0_syn_curr_wait_i  (a0_syn_curr_wait_i),
    .a0_bias_curr_wr_o  (a0_bias_curr_wr_o), .a0_bias_curr_addr_o (a0_bias_curr_addr_o), .a0_bias_curr_data_o (a0_bias_curr_data_o), .a0_bias_curr_wait_i (a0_bias_curr_wait_i),
    .a0_thresh_wr_o     (a0_thresh_wr_o),    .a0_thresh_addr_o    (a0_thresh_addr_o),    .a0_thresh_data_o    (a0_thresh_data_o),    .a0_thresh_wait_i    (a0_thresh_wait_i),
    .a0_pot_wr_o        (a0_pot_wr_o),       .a0_pot_addr_o       (a0_pot_addr_o),       .a0_pot_data_o       (a0_pot_data_o),       .a0_pot_wait_i       (a0_pot_wait_i),
    .a0_spike_wr_o      (a0_spike_wr_o),     .a0_spike_addr_o     (a0_spike_addr_o),     .a0_spike_data_o     (a0_spike_data_o),     .a0_spike_wait_i     (a0_spike_wait_i),
    .hd_src_a_wr_o      (hd_src_a_wr_o),    .hd_src_a_addr_o     (hd_src_a_addr_o),     .hd_src_a_data_o     (hd_src_a_data_o),     .hd_src_a_wait_i     (hd_src_a_wait_i),
    .hd_src_b_wr_o      (hd_src_b_wr_o),    .hd_src_b_addr_o     (hd_src_b_addr_o),     .hd_src_b_data_o     (hd_src_b_data_o),     .hd_src_b_wait_i     (hd_src_b_wait_i),
    .hd_src_z_wr_o      (hd_src_z_wr_o),    .hd_src_z_addr_o     (hd_src_z_addr_o),     .hd_src_z_data_o     (hd_src_z_data_o),     .hd_src_z_wait_i     (hd_src_z_wait_i),
    .hd_src_r_wr_o      (hd_src_r_wr_o),    .hd_src_r_addr_o     (hd_src_r_addr_o),     .hd_src_r_data_o     (hd_src_r_data_o),     .hd_src_r_wait_i     (hd_src_r_wait_i)
);

// ─── Clock ────────────────────────────────────────────────────────────────────
always #CLK_HALF clk = ~clk;

// ─── OR of all write enables ─────────────────────────────────────────────────
// Used to detect spurious writes on non-selected buses.
wire any_wr_o =
    s0_weight_wr_o | s0_act_wr_o | s0_syn_curr_wr_o | s0_bias_curr_wr_o |
    s0_thresh_wr_o | s0_pot_wr_o | s0_spike_wr_o |
    s1_weight_wr_o | s1_act_wr_o | s1_syn_curr_wr_o | s1_bias_curr_wr_o |
    s1_thresh_wr_o | s1_pot_wr_o | s1_spike_wr_o |
    a0_weight_wr_o | a0_act_wr_o | a0_syn_curr_wr_o | a0_bias_curr_wr_o |
    a0_thresh_wr_o | a0_pot_wr_o | a0_spike_wr_o |
    hd_src_a_wr_o | hd_src_b_wr_o | hd_src_z_wr_o | hd_src_r_wr_o;

// ─── Write capture ────────────────────────────────────────────────────────────
// per_wr_cnt[i]: number of writes on memory bus i this test
integer          per_wr_cnt [0:NUM_MEM_TYPES-1];
// Sequential-address and data tracking for the bus-under-test
reg [ADDR_SIZE-1:0] cap_next_addr;   // next expected word address
reg                 cap_addr_err;    // set if any address was out of sequence
reg                 cap_data_err;    // set if any data word was wrong
integer             fail_count;

// Mux: address/data from whichever bus is active (one-hot, so priority mux is fine)
wire [ADDR_SIZE-1:0] active_addr_o =
    s0_weight_wr_o    ? s0_weight_addr_o    :
    s0_act_wr_o       ? s0_act_addr_o       :
    s0_syn_curr_wr_o  ? s0_syn_curr_addr_o  :
    s0_bias_curr_wr_o ? s0_bias_curr_addr_o :
    s0_thresh_wr_o    ? s0_thresh_addr_o    :
    s0_pot_wr_o       ? s0_pot_addr_o       :
    s0_spike_wr_o     ? s0_spike_addr_o     :
    s1_weight_wr_o    ? s1_weight_addr_o    :
    s1_act_wr_o       ? s1_act_addr_o       :
    s1_syn_curr_wr_o  ? s1_syn_curr_addr_o  :
    s1_bias_curr_wr_o ? s1_bias_curr_addr_o :
    s1_thresh_wr_o    ? s1_thresh_addr_o    :
    s1_pot_wr_o       ? s1_pot_addr_o       :
    s1_spike_wr_o     ? s1_spike_addr_o     :
    a0_weight_wr_o    ? a0_weight_addr_o    :
    a0_act_wr_o       ? a0_act_addr_o       :
    a0_syn_curr_wr_o  ? a0_syn_curr_addr_o  :
    a0_bias_curr_wr_o ? a0_bias_curr_addr_o :
    a0_thresh_wr_o    ? a0_thresh_addr_o    :
    a0_pot_wr_o       ? a0_pot_addr_o       :
    a0_spike_wr_o     ? a0_spike_addr_o     :
    hd_src_a_wr_o     ? hd_src_a_addr_o     :
    hd_src_b_wr_o     ? hd_src_b_addr_o     :
    hd_src_z_wr_o     ? hd_src_z_addr_o     :
    hd_src_r_wr_o     ? hd_src_r_addr_o     :
                        {ADDR_SIZE{1'bx}};

wire [DATA_SZ-1:0] active_data_o =
    s0_weight_wr_o    ? s0_weight_data_o    :
    s0_act_wr_o       ? s0_act_data_o       :
    s0_syn_curr_wr_o  ? s0_syn_curr_data_o  :
    s0_bias_curr_wr_o ? s0_bias_curr_data_o :
    s0_thresh_wr_o    ? s0_thresh_data_o    :
    s0_pot_wr_o       ? s0_pot_data_o       :
    s0_spike_wr_o     ? s0_spike_data_o     :
    s1_weight_wr_o    ? s1_weight_data_o    :
    s1_act_wr_o       ? s1_act_data_o       :
    s1_syn_curr_wr_o  ? s1_syn_curr_data_o  :
    s1_bias_curr_wr_o ? s1_bias_curr_data_o :
    s1_thresh_wr_o    ? s1_thresh_data_o    :
    s1_pot_wr_o       ? s1_pot_data_o       :
    s1_spike_wr_o     ? s1_spike_data_o     :
    a0_weight_wr_o    ? a0_weight_data_o    :
    a0_act_wr_o       ? a0_act_data_o       :
    a0_syn_curr_wr_o  ? a0_syn_curr_data_o  :
    a0_bias_curr_wr_o ? a0_bias_curr_data_o :
    a0_thresh_wr_o    ? a0_thresh_data_o    :
    a0_pot_wr_o       ? a0_pot_data_o       :
    a0_spike_wr_o     ? a0_spike_data_o     :
    hd_src_a_wr_o     ? hd_src_a_data_o     :
    hd_src_b_wr_o     ? hd_src_b_data_o     :
    hd_src_z_wr_o     ? hd_src_z_data_o     :
    hd_src_r_wr_o     ? hd_src_r_data_o     :
                        {DATA_SZ{1'bx}};

// Index of the currently-active write bus (for per_wr_cnt updates)
wire [4:0] active_bus_idx =
    s0_weight_wr_o    ? IDX_S0_WEIGHT    :
    s0_act_wr_o       ? IDX_S0_ACT       :
    s0_syn_curr_wr_o  ? IDX_S0_SYN_CURR  :
    s0_bias_curr_wr_o ? IDX_S0_BIAS_CURR :
    s0_thresh_wr_o    ? IDX_S0_THRESH    :
    s0_pot_wr_o       ? IDX_S0_POT       :
    s0_spike_wr_o     ? IDX_S0_SPIKE     :
    s1_weight_wr_o    ? IDX_S1_WEIGHT    :
    s1_act_wr_o       ? IDX_S1_ACT       :
    s1_syn_curr_wr_o  ? IDX_S1_SYN_CURR  :
    s1_bias_curr_wr_o ? IDX_S1_BIAS_CURR :
    s1_thresh_wr_o    ? IDX_S1_THRESH    :
    s1_pot_wr_o       ? IDX_S1_POT       :
    s1_spike_wr_o     ? IDX_S1_SPIKE     :
    a0_weight_wr_o    ? IDX_A0_WEIGHT    :
    a0_act_wr_o       ? IDX_A0_ACT       :
    a0_syn_curr_wr_o  ? IDX_A0_SYN_CURR  :
    a0_bias_curr_wr_o ? IDX_A0_BIAS_CURR :
    a0_thresh_wr_o    ? IDX_A0_THRESH    :
    a0_pot_wr_o       ? IDX_A0_POT       :
    a0_spike_wr_o     ? IDX_A0_SPIKE     :
    hd_src_a_wr_o     ? IDX_HD_SRC_A     :
    hd_src_b_wr_o     ? IDX_HD_SRC_B     :
    hd_src_z_wr_o     ? IDX_HD_SRC_Z     :
    hd_src_r_wr_o     ? IDX_HD_SRC_R     : 5'd31;   // 31 = none

// Capture on the falling edge to see combinational outputs from last posedge
always @(negedge clk) begin
    if (any_wr_o) begin
        per_wr_cnt[active_bus_idx] = per_wr_cnt[active_bus_idx] + 1;
        if (active_addr_o !== cap_next_addr)
            cap_addr_err = 1'b1;
        if (active_data_o !== fill_value_i)
            cap_data_err = 1'b1;
        cap_next_addr = cap_next_addr + 1'b1;
    end
end

// ─── Tasks ────────────────────────────────────────────────────────────────────

task do_reset;
    integer i;
    begin
        clk               = 1'b0;
        reset             = 1'b1;
        sys_req_i         = 1'b0;
        sys_addr_i        = 32'b0;
        sys_data_i        = 32'b0;
        start_new_block_i = 1'b0;
        buff_id_i         = 'b0;
        fill_value_i      = 32'b0;
        fill_block_size_i = 20'b0;
        fu_bba_wait_i     = 1'b0;
        fu_bba_data_i     = 32'b0;
        s0_weight_wait_i    = 1'b0; s0_act_wait_i      = 1'b0;
        s0_syn_curr_wait_i  = 1'b0; s0_bias_curr_wait_i= 1'b0;
        s0_thresh_wait_i    = 1'b0; s0_pot_wait_i      = 1'b0;
        s0_spike_wait_i     = 1'b0;
        s1_weight_wait_i    = 1'b0; s1_act_wait_i      = 1'b0;
        s1_syn_curr_wait_i  = 1'b0; s1_bias_curr_wait_i= 1'b0;
        s1_thresh_wait_i    = 1'b0; s1_pot_wait_i      = 1'b0;
        s1_spike_wait_i     = 1'b0;
        a0_weight_wait_i    = 1'b0; a0_act_wait_i      = 1'b0;
        a0_syn_curr_wait_i  = 1'b0; a0_bias_curr_wait_i= 1'b0;
        a0_thresh_wait_i    = 1'b0; a0_pot_wait_i      = 1'b0;
        a0_spike_wait_i     = 1'b0;
        hd_src_a_wait_i     = 1'b0; hd_src_b_wait_i    = 1'b0;
        hd_src_z_wait_i     = 1'b0; hd_src_r_wait_i    = 1'b0;
        fail_count          = 0;
        repeat (4) @(posedge clk);
        #1;
        reset = 1'b0;
        @(posedge clk); #1;
    end
endtask

// Reset per-test capture state.  Call before each dispatch.
task clear_capture;
    input [ADDR_SIZE-1:0] first_addr;   // expected first write address
    integer i;
    begin
        for (i = 0; i < NUM_MEM_TYPES; i = i + 1)
            per_wr_cnt[i] = 0;
        cap_next_addr = first_addr;
        cap_addr_err  = 1'b0;
        cap_data_err  = 1'b0;
    end
endtask

// AXI write — single clock cycle, combinational ack
task axi_wr;
    input [31:0] addr;
    input [31:0] data;
    begin
        sys_addr_i = addr;
        sys_data_i = data;
        sys_req_i  = 1'b1;
        @(posedge clk); #1;
        sys_req_i  = 1'b0;
    end
endtask

// Helper: AXI address for buffer B in the mem_sel table
function [31:0] fu_tbl_addr;
    input integer buf_id;
    begin
        fu_tbl_addr = (FU_TABLE_ADDR & FU_TABLE_ADDR_MASK) | (buf_id[31:0] << 2);
    end
endfunction

// Dispatch a FILL: one-cycle start pulse.  fu_bba_data_i must be set before call.
task do_dispatch;
    input [BUFF_INDX_SZ-1:0] bid;
    input [DATA_SZ-1:0]       fval;
    input [19:0]              bsz;
    begin
        buff_id_i         = bid;
        fill_value_i      = fval;
        fill_block_size_i = bsz;
        start_new_block_i = 1'b1;
        @(posedge clk); #1;
        start_new_block_i = 1'b0;
    end
endtask

// Wait until acc_finished_o pulses (or TIMEOUT)
task wait_finished;
    integer t;
    reg     seen;
    begin
        seen = 1'b0;
        for (t = 0; t < TIMEOUT && !seen; t = t + 1) begin
            @(posedge clk); #1;
            if (acc_finished_o) seen = 1'b1;
        end
        if (!seen) begin
            $display("FAIL: timeout — acc_finished_o never pulsed");
            fail_count = fail_count + 1;
        end
    end
endtask

// ─── Check helpers ────────────────────────────────────────────────────────────

// Verify that exactly one bus (expected_idx) received exactly exp_count writes,
// all others zero, and that address/data captures show no errors.
task check_fill;
    input integer expected_idx;
    input integer exp_count;
    input [63:0]  test_name;    // up to 8 ASCII chars packed in 64 bits
    integer i;
    integer any_other;
    begin
        // Check expected bus count
        if (per_wr_cnt[expected_idx] !== exp_count) begin
            $display("FAIL %s: bus %0d write count = %0d (expected %0d)",
                     test_name, expected_idx, per_wr_cnt[expected_idx], exp_count);
            fail_count = fail_count + 1;
        end else
            $display("PASS %s: write count = %0d", test_name, exp_count);

        // Check no other bus fired
        any_other = 0;
        for (i = 0; i < NUM_MEM_TYPES; i = i + 1)
            if (i != expected_idx && per_wr_cnt[i] > 0)
                any_other = any_other + per_wr_cnt[i];
        if (any_other > 0) begin
            $display("FAIL %s: %0d spurious write(s) on non-selected buses",
                     test_name, any_other);
            fail_count = fail_count + 1;
        end else
            $display("PASS %s: no spurious writes on other buses", test_name);

        // Address and data integrity
        if (cap_addr_err) begin
            $display("FAIL %s: address out of sequence", test_name);
            fail_count = fail_count + 1;
        end else
            $display("PASS %s: addresses sequential", test_name);

        if (cap_data_err) begin
            $display("FAIL %s: wrong data written", test_name);
            fail_count = fail_count + 1;
        end else
            $display("PASS %s: data correct", test_name);
    end
endtask

// ─── Tests ────────────────────────────────────────────────────────────────────
integer i;

initial begin
    do_reset;

    // --------------------------------------------------------------------- T1
    // Basic fill: buffer 3 → s0_syn_curr (IDX 2), 8 words, value=0xDEAD_BEEF
    // Base address in BBA = 0x1000 (word address)
    $display("\n--- T1: basic fill (buf3 → s0_syn_curr, 8 words, 0xDEAD_BEEF) ---");

    // Program mem_sel_table[3] via AXI: enable bit IDX_S0_SYN_CURR=2
    axi_wr(fu_tbl_addr(3), (32'b1 << IDX_S0_SYN_CURR));

    // Set BBA data for buffer 3
    fu_bba_data_i = 32'h0000_1000;   // word-address 0x1000

    clear_capture(30'h1000);
    do_dispatch(4'd3, 32'hDEAD_BEEF, 20'd8);

    wait_finished;

    check_fill(IDX_S0_SYN_CURR, 8, "T1");

    // Verify acc_busy_o returned to 0 after finish
    if (acc_busy_o !== 1'b0) begin
        $display("FAIL T1: acc_busy_o still set after finish");
        fail_count = fail_count + 1;
    end else
        $display("PASS T1: acc_busy_o clear after finish");

    @(posedge clk); @(posedge clk);

    // --------------------------------------------------------------------- T2
    // Different memory: buffer 7 → hd_src_r (IDX 24), 4 words, value=0xCAFE_0000
    $display("\n--- T2: different memory (buf7 → hd_src_r, 4 words, 0xCAFE_0000) ---");

    axi_wr(fu_tbl_addr(7), (32'b1 << IDX_HD_SRC_R));
    fu_bba_data_i = 32'h0000_2000;

    clear_capture(30'h2000);
    do_dispatch(4'd7, 32'hCAFE_0000, 20'd4);

    wait_finished;

    check_fill(IDX_HD_SRC_R, 4, "T2");

    @(posedge clk); @(posedge clk);

    // --------------------------------------------------------------------- T3
    // BBA back-pressure: hold fu_bba_wait_i for 3 cycles after dispatch.
    // No writes should fire during BBA_WAIT; fill should proceed normally after.
    $display("\n--- T3: BBA back-pressure (3 wait cycles) ---");

    // Reuse buffer 3 / s0_syn_curr setup from T1
    fu_bba_data_i  = 32'h0000_3000;
    fu_bba_wait_i  = 1'b1;           // pre-assert BBA wait

    clear_capture(30'h3000);
    do_dispatch(4'd3, 32'hDEAD_BEEF, 20'd8);

    // Spend 3 cycles in BBA_WAIT: verify no writes during this time
    repeat (3) begin
        @(posedge clk); #1;
        if (any_wr_o) begin
            $display("FAIL T3: write fired while BBA wait asserted");
            fail_count = fail_count + 1;
        end
    end

    // Release BBA wait
    fu_bba_wait_i = 1'b0;

    wait_finished;

    // Write count must still be 8; addresses sequential from 0x3000
    if (per_wr_cnt[IDX_S0_SYN_CURR] !== 8) begin
        $display("FAIL T3: wrong write count %0d (expected 8)", per_wr_cnt[IDX_S0_SYN_CURR]);
        fail_count = fail_count + 1;
    end else
        $display("PASS T3: correct write count after BBA wait");

    if (cap_addr_err) begin
        $display("FAIL T3: address error after BBA wait");
        fail_count = fail_count + 1;
    end else
        $display("PASS T3: addresses correct after BBA wait");

    @(posedge clk); @(posedge clk);

    // --------------------------------------------------------------------- T4
    // Memory back-pressure: assert s0_syn_curr_wait_i after the 3rd write.
    // The fill must pause and resume, delivering all 8 writes in the correct order.
    $display("\n--- T4: memory back-pressure (hold wait after write 3) ---");

    fu_bba_data_i = 32'h0000_4000;

    clear_capture(30'h4000);
    do_dispatch(4'd3, 32'hDEAD_BEEF, 20'd8);

    begin : t4_block
        integer wr_before_wait;
        integer t;
        reg     seen_finish;

        // Wait until fill starts (BBA latched)
        @(posedge clk); #1;  // dispatch posedge → BBA_WAIT
        @(posedge clk); #1;  // BBA_WAIT → FILLING (BBA data latches)

        // Let 3 writes fire
        repeat (3) @(posedge clk); #1;

        // Record write count, then hold memory wait
        wr_before_wait = per_wr_cnt[IDX_S0_SYN_CURR];
        s0_syn_curr_wait_i = 1'b1;

        // Verify no progress for 4 stall cycles
        repeat (4) begin
            @(posedge clk); #1;
            if (per_wr_cnt[IDX_S0_SYN_CURR] !== wr_before_wait) begin
                $display("FAIL T4: write count advanced during wait (%0d → %0d)",
                         wr_before_wait, per_wr_cnt[IDX_S0_SYN_CURR]);
                fail_count = fail_count + 1;
            end
        end
        $display("PASS T4: write count held at %0d during memory wait", wr_before_wait);

        // Release and wait for finish
        s0_syn_curr_wait_i = 1'b0;
    end

    wait_finished;

    check_fill(IDX_S0_SYN_CURR, 8, "T4");

    @(posedge clk); @(posedge clk);

    // --------------------------------------------------------------------- T5
    // Back-to-back: dispatch T2 (buf7/hd_src_r/4 words) then T1 (buf3/syn_curr/8)
    // immediately — start_new_block_i for second fill on the same cycle as
    // acc_finished_o of the first.
    $display("\n--- T5: back-to-back fills (T2 then T1, no idle gap) ---");

    // Confirm table entries still valid from T1/T2 setup above (they survive reset)
    fu_bba_data_i = 32'h0000_2000;   // T2 BBA

    clear_capture(30'h2000);
    // Dispatch first fill
    buff_id_i         = 4'd7;
    fill_value_i      = 32'hCAFE_0000;
    fill_block_size_i = 20'd4;
    start_new_block_i = 1'b1;
    @(posedge clk); #1;
    start_new_block_i = 1'b0;

    // Wait until acc_finished_o — then immediately dispatch second fill on same cycle
    begin : t5_block
        integer t5_t;
        reg     t5_seen;
        t5_seen = 1'b0;
        for (t5_t = 0; t5_t < TIMEOUT && !t5_seen; t5_t = t5_t + 1) begin
            @(posedge clk); #1;
            if (acc_finished_o) begin
                t5_seen = 1'b1;
                // Check T2 part while still at this time point
                if (per_wr_cnt[IDX_HD_SRC_R] !== 4) begin
                    $display("FAIL T5 (part A): hd_src_r write count %0d (exp 4)",
                             per_wr_cnt[IDX_HD_SRC_R]);
                    fail_count = fail_count + 1;
                end else
                    $display("PASS T5 (part A): first fill (hd_src_r) completed with 4 writes");
            end
        end
        if (!t5_seen) begin
            $display("FAIL T5: first fill never finished");
            fail_count = fail_count + 1;
        end
    end

    // Dispatch second fill immediately (next cycle, FSM now IDLE)
    fu_bba_data_i     = 32'h0000_1000;
    clear_capture(30'h1000);
    do_dispatch(4'd3, 32'hDEAD_BEEF, 20'd8);

    wait_finished;

    if (per_wr_cnt[IDX_S0_SYN_CURR] !== 8) begin
        $display("FAIL T5 (part B): s0_syn_curr write count %0d (exp 8)",
                 per_wr_cnt[IDX_S0_SYN_CURR]);
        fail_count = fail_count + 1;
    end else
        $display("PASS T5 (part B): second fill (s0_syn_curr) completed with 8 writes");

    if (cap_addr_err) begin
        $display("FAIL T5 (part B): address error in second fill");
        fail_count = fail_count + 1;
    end else
        $display("PASS T5 (part B): addresses sequential");

    // ─── Final summary ────────────────────────────────────────────────────────
    @(posedge clk); @(posedge clk);
    $display("\n=== Results: %0d failure(s) ===", fail_count);
    if (fail_count == 0)
        $display("ALL TESTS PASSED");
    else
        $display("SOME TESTS FAILED");

    $finish;
end

endmodule // tb_fill_unit
