// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
`timescale 10ps/1ps
`include "../shared/constants.v"

// ============================================================================
// tb_flexman — top-level integration testbench for flexman
//
// All external memory interfaces are 256-word behavioral RAMs.
// All wait/backpressure signals are tied to zero (no stalls).
//
// Tests:
//   T1: NXT(in+out) + STOP  — NXT on empty table fires immediately
//   T2: FILL buf=0 (4 words, val=0xDEADBEEF → s0_act) + STOP
//       Checks s0_act_mem[0..3] == 0xDEADBEEF after fill completes
// ============================================================================
module tb_flexman;

// ─── Parameters ───────────────────────────────────────────────────────────────
localparam NUM_BUFFERS         = 16;
localparam NUM_HW_ACCELERATORS = 5;
localparam WORDS_PER_CONFIG    = 4;
localparam CFG_ID_SZ           = 5;
localparam BUFF_INDX_SZ        = 4;
localparam TGT_ACC_SZ          = 3;
localparam TGT_COUNT_SZ        = 3;
localparam PROG_ADDR_BITS      = 10;
localparam PROG_DATA_BITS      = 32;
localparam NUM_SCH_ENTRIES     = 4;
localparam COL_BUFF_ID_SZ      = 16;

localparam MEM_DEPTH     = 256;
localparam MEM_ADDR_BITS = 8;
localparam PROG_DEPTH    = 64;
localparam PROG_IDX_BITS = 6;   // $clog2(PROG_DEPTH)

// Default AXI base addresses (match flexman defaults)
localparam [31:0] FU_TABLE_ADDR      = 32'hC000_0000;
localparam [31:0] FU_TABLE_ADDR_MASK = 32'hFF00_0000;

// ─── Instruction encoding ─────────────────────────────────────────────────────
localparam STOP_INST = 32'h0000_0002;   // opcode 3'b010

function [31:0] nxt_inst;
    input nxt_in, nxt_out;
    // [2:0]=opcode(100), [3]=rsv, [4]=nxt_in, [5]=nxt_out
    nxt_inst = {26'b0, nxt_out, nxt_in, 1'b0, 3'b100};
endfunction

// FILL word 1: [2:0]=101, [6:3]=buf_id (4-bit), [7]=0, [8]=colour,
//              [11:9]=#targets, [31:12]=block_size
function [31:0] fill_w1;
    input [3:0]  buf_id;
    input        col;
    input [2:0]  ntgt;
    input [19:0] sz;
    fill_w1 = {sz, ntgt, col, 1'b0, buf_id, 3'b101};
endfunction

// ─── Clock/reset ──────────────────────────────────────────────────────────────
reg clk, reset;
initial clk = 1'b0;
always #5 clk = ~clk;

// ─── Behavioral memories ──────────────────────────────────────────────────────
reg [31:0]          prog_mem       [0:PROG_DEPTH-1];
reg [31:0]          cfg_mem        [0:MEM_DEPTH-1];
reg [31:0]          bba_mem        [0:MEM_DEPTH-1];

// snnAcc0
reg [`WTD_BITS-1:0] s0_weight_mem  [0:MEM_DEPTH-1];
reg [`ACT_BITS-1:0] s0_act_mem     [0:MEM_DEPTH-1];
reg [`POT_BITS-1:0] s0_syn_curr_mem[0:MEM_DEPTH-1];
reg [`WTD_BITS-1:0] s0_bias_curr_mem[0:MEM_DEPTH-1];
reg [`WTD_BITS-1:0] s0_thresh_mem  [0:MEM_DEPTH-1];
reg [`POT_BITS-1:0] s0_pot_mem     [0:MEM_DEPTH-1];
reg [`ACT_BITS-1:0] s0_spike_mem   [0:MEM_DEPTH-1];

// snnAcc1
reg [`WTD_BITS-1:0] s1_weight_mem  [0:MEM_DEPTH-1];
reg [`ACT_BITS-1:0] s1_act_mem     [0:MEM_DEPTH-1];
reg [`POT_BITS-1:0] s1_syn_curr_mem[0:MEM_DEPTH-1];
reg [`WTD_BITS-1:0] s1_bias_curr_mem[0:MEM_DEPTH-1];
reg [`WTD_BITS-1:0] s1_thresh_mem  [0:MEM_DEPTH-1];
reg [`POT_BITS-1:0] s1_pot_mem     [0:MEM_DEPTH-1];
reg [`ACT_BITS-1:0] s1_spike_mem   [0:MEM_DEPTH-1];

// annAcc
reg [`WTD_BITS-1:0] a0_weight_mem  [0:MEM_DEPTH-1];
reg [`ACT_BITS-1:0] a0_act_mem     [0:MEM_DEPTH-1];
reg [`POT_BITS-1:0] a0_syn_curr_mem[0:MEM_DEPTH-1];
reg [`WTD_BITS-1:0] a0_bias_curr_mem[0:MEM_DEPTH-1];
reg [`WTD_BITS-1:0] a0_thresh_mem  [0:MEM_DEPTH-1];
reg [`POT_BITS-1:0] a0_pot_mem     [0:MEM_DEPTH-1];
reg [`ACT_BITS-1:0] a0_spike_mem   [0:MEM_DEPTH-1];

// Hadamard
reg [31:0]          hd_src_a_mem   [0:MEM_DEPTH-1];
reg [31:0]          hd_src_b_mem   [0:MEM_DEPTH-1];
reg [31:0]          hd_src_z_mem   [0:MEM_DEPTH-1];
reg [31:0]          hd_src_r_mem   [0:MEM_DEPTH-1];

// ─── DUT input regs (driven procedurally) ────────────────────────────────────
reg         sys_req_i;
reg  [31:0] sys_addr_i;
reg  [31:0] sys_data_i;

reg                         start_program_i;
reg  [PROG_ADDR_BITS-1:0]   program_addr_i;

reg  mark_buff_as_full_i;
reg  [BUFF_INDX_SZ-1:0]   full_buff_id_i;
reg  [TGT_COUNT_SZ-1:0]   full_buff_usage_i;

// ─── DUT outputs (wires) ──────────────────────────────────────────────────────
wire         sys_ack_o;
wire [31:0]  sys_data_o;

wire [`ADDR_SIZE-1:0]      prog_mem_addr_o;
wire                        prog_mem_req_o;

wire                        cfg_mem_rd_o;
wire [31:0]                 cfg_mem_addr_o;
wire                        cfg_mem_wr_o;
wire [31:0]                 cfg_mem_wr_addr_o;
wire [31:0]                 cfg_mem_wr_data_o;

wire                        bba_mem_rd_o;
wire [31:0]                 bba_mem_addr_o;
wire                        bba_mem_wr_o;
wire [31:0]                 bba_mem_wr_addr_o;
wire [31:0]                 bba_mem_wr_data_o;

wire                        fu_bba_mem_rd_o;
wire [31:0]                 fu_bba_mem_addr_o;

wire                        nxt_input_pulse_o;
wire                        nxt_output_pulse_o;
wire                        cm_config_finished_o;

// snnAcc0
wire                        s0_weight_mem_rd_o;
wire [`ADDR_SIZE-1:0]       s0_weight_mem_addr_o;
wire                        s0_weight_mem_wr_o;
wire [`ADDR_SIZE-1:0]       s0_weight_mem_wr_addr_o;
wire [31:0]                 s0_weight_mem_wr_data_o;

wire                        s0_act_mem_req_o;
wire [`ADDR_SIZE-1:0]       s0_act_mem_addr_o;
wire                        s0_act_mem_wr_o;
wire [`ADDR_SIZE-1:0]       s0_act_mem_wr_addr_o;
wire [31:0]                 s0_act_mem_wr_data_o;

wire                        s0_syn_curr_mem_wr_o;
wire                        s0_syn_curr_mem_rd_o;
wire [`ADDR_SIZE-1:0]       s0_syn_curr_mem_addr_o;
wire [`POT_BITS-1:0]        s0_syn_curr_mem_data_o;

wire                        s0_bias_curr_mem_rd_o;
wire [`ADDR_SIZE-1:0]       s0_bias_curr_mem_addr_o;
wire                        s0_bias_curr_mem_wr_o;
wire [`ADDR_SIZE-1:0]       s0_bias_curr_mem_wr_addr_o;
wire [31:0]                 s0_bias_curr_mem_wr_data_o;

wire                        s0_thresh_mem_rd_o;
wire [`ADDR_SIZE-1:0]       s0_thresh_mem_addr_o;
wire                        s0_thresh_mem_wr_o;
wire [`ADDR_SIZE-1:0]       s0_thresh_mem_wr_addr_o;
wire [31:0]                 s0_thresh_mem_wr_data_o;

wire                        s0_pot_mem_wr_o;
wire                        s0_pot_mem_rd_o;
wire [`ADDR_SIZE-1:0]       s0_pot_mem_addr_o;
wire [`POT_BITS-1:0]        s0_pot_mem_data_o;

wire                        s0_spike_mem_wr_o;
wire [`ADDR_SIZE-1:0]       s0_spike_mem_addr_o;
wire [`ACT_BITS-1:0]        s0_spike_mem_data_o;

// snnAcc1
wire                        s1_weight_mem_rd_o;
wire [`ADDR_SIZE-1:0]       s1_weight_mem_addr_o;
wire                        s1_weight_mem_wr_o;
wire [`ADDR_SIZE-1:0]       s1_weight_mem_wr_addr_o;
wire [31:0]                 s1_weight_mem_wr_data_o;

wire                        s1_act_mem_req_o;
wire [`ADDR_SIZE-1:0]       s1_act_mem_addr_o;
wire                        s1_act_mem_wr_o;
wire [`ADDR_SIZE-1:0]       s1_act_mem_wr_addr_o;
wire [31:0]                 s1_act_mem_wr_data_o;

wire                        s1_syn_curr_mem_wr_o;
wire                        s1_syn_curr_mem_rd_o;
wire [`ADDR_SIZE-1:0]       s1_syn_curr_mem_addr_o;
wire [`POT_BITS-1:0]        s1_syn_curr_mem_data_o;

wire                        s1_bias_curr_mem_rd_o;
wire [`ADDR_SIZE-1:0]       s1_bias_curr_mem_addr_o;
wire                        s1_bias_curr_mem_wr_o;
wire [`ADDR_SIZE-1:0]       s1_bias_curr_mem_wr_addr_o;
wire [31:0]                 s1_bias_curr_mem_wr_data_o;

wire                        s1_thresh_mem_rd_o;
wire [`ADDR_SIZE-1:0]       s1_thresh_mem_addr_o;
wire                        s1_thresh_mem_wr_o;
wire [`ADDR_SIZE-1:0]       s1_thresh_mem_wr_addr_o;
wire [31:0]                 s1_thresh_mem_wr_data_o;

wire                        s1_pot_mem_wr_o;
wire                        s1_pot_mem_rd_o;
wire [`ADDR_SIZE-1:0]       s1_pot_mem_addr_o;
wire [`POT_BITS-1:0]        s1_pot_mem_data_o;

wire                        s1_spike_mem_wr_o;
wire [`ADDR_SIZE-1:0]       s1_spike_mem_addr_o;
wire [`ACT_BITS-1:0]        s1_spike_mem_data_o;

// annAcc
wire                        a0_weight_mem_rd_o;
wire [`ADDR_SIZE-1:0]       a0_weight_mem_addr_o;
wire                        a0_weight_mem_wr_o;
wire [`ADDR_SIZE-1:0]       a0_weight_mem_wr_addr_o;
wire [31:0]                 a0_weight_mem_wr_data_o;

wire                        a0_act_mem_req_o;
wire [`ADDR_SIZE-1:0]       a0_act_mem_addr_o;
wire                        a0_act_mem_wr_o;
wire [`ADDR_SIZE-1:0]       a0_act_mem_wr_addr_o;
wire [31:0]                 a0_act_mem_wr_data_o;

wire                        a0_syn_curr_mem_wr_o;
wire                        a0_syn_curr_mem_rd_o;
wire [`ADDR_SIZE-1:0]       a0_syn_curr_mem_addr_o;
wire [`POT_BITS-1:0]        a0_syn_curr_mem_data_o;

wire                        a0_bias_curr_mem_rd_o;
wire [`ADDR_SIZE-1:0]       a0_bias_curr_mem_addr_o;
wire                        a0_bias_curr_mem_wr_o;
wire [`ADDR_SIZE-1:0]       a0_bias_curr_mem_wr_addr_o;
wire [31:0]                 a0_bias_curr_mem_wr_data_o;

wire                        a0_thresh_mem_rd_o;
wire [`ADDR_SIZE-1:0]       a0_thresh_mem_addr_o;
wire                        a0_thresh_mem_wr_o;
wire [`ADDR_SIZE-1:0]       a0_thresh_mem_wr_addr_o;
wire [31:0]                 a0_thresh_mem_wr_data_o;

wire                        a0_pot_mem_wr_o;
wire                        a0_pot_mem_rd_o;
wire [`ADDR_SIZE-1:0]       a0_pot_mem_addr_o;
wire [`POT_BITS-1:0]        a0_pot_mem_data_o;

wire                        a0_spike_mem_wr_o;
wire [`ADDR_SIZE-1:0]       a0_spike_mem_addr_o;
wire [`ACT_BITS-1:0]        a0_spike_mem_data_o;

// Hadamard
wire                        hd_src_a_mem_rd_o;
wire [`ADDR_SIZE-1:0]       hd_src_a_mem_addr_o;
wire                        hd_src_a_mem_wr_o;
wire [`ADDR_SIZE-1:0]       hd_src_a_mem_wr_addr_o;
wire [31:0]                 hd_src_a_mem_wr_data_o;

wire                        hd_src_b_mem_rd_o;
wire [`ADDR_SIZE-1:0]       hd_src_b_mem_addr_o;
wire                        hd_src_b_mem_wr_o;
wire [`ADDR_SIZE-1:0]       hd_src_b_mem_wr_addr_o;
wire [31:0]                 hd_src_b_mem_wr_data_o;

wire                        hd_src_z_mem_rd_o;
wire [`ADDR_SIZE-1:0]       hd_src_z_mem_addr_o;
wire                        hd_src_z_mem_wr_o;
wire [`ADDR_SIZE-1:0]       hd_src_z_mem_wr_addr_o;
wire [31:0]                 hd_src_z_mem_wr_data_o;

wire                        hd_src_r_mem_rd_o;
wire [`ADDR_SIZE-1:0]       hd_src_r_mem_addr_o;
wire                        hd_src_r_mem_wr_o;
wire [`ADDR_SIZE-1:0]       hd_src_r_mem_wr_addr_o;
wire [31:0]                 hd_src_r_mem_data_o;

// ─── Constant inputs (tied to 0) ──────────────────────────────────────────────
wire test_stall_pipe      = 1'b0;
wire prog_mem_wait_i      = 1'b0;
wire cfg_mem_wait_i       = 1'b0;
wire bba_mem_wait_i       = 1'b0;
wire fu_bba_mem_wait_i    = 1'b0;

wire s0_weight_mem_wait_i    = 1'b0;
wire s0_weight_mem_wr_wait_i = 1'b0;
wire s0_act_mem_wait_i       = 1'b0;
wire s0_act_mem_wr_wait_i    = 1'b0;
wire s0_syn_curr_mem_wait_i  = 1'b0;
wire s0_bias_curr_mem_wait_i = 1'b0;
wire s0_bias_curr_mem_wr_wait_i = 1'b0;
wire s0_thresh_mem_wait_i    = 1'b0;
wire s0_thresh_mem_wr_wait_i = 1'b0;
wire s0_pot_mem_wait_i       = 1'b0;
wire s0_spike_mem_wait_i     = 1'b0;

wire s1_weight_mem_wait_i    = 1'b0;
wire s1_weight_mem_wr_wait_i = 1'b0;
wire s1_act_mem_wait_i       = 1'b0;
wire s1_act_mem_wr_wait_i    = 1'b0;
wire s1_syn_curr_mem_wait_i  = 1'b0;
wire s1_bias_curr_mem_wait_i = 1'b0;
wire s1_bias_curr_mem_wr_wait_i = 1'b0;
wire s1_thresh_mem_wait_i    = 1'b0;
wire s1_thresh_mem_wr_wait_i = 1'b0;
wire s1_pot_mem_wait_i       = 1'b0;
wire s1_spike_mem_wait_i     = 1'b0;

wire a0_weight_mem_wait_i    = 1'b0;
wire a0_weight_mem_wr_wait_i = 1'b0;
wire a0_act_mem_wait_i       = 1'b0;
wire a0_act_mem_wr_wait_i    = 1'b0;
wire a0_syn_curr_mem_wait_i  = 1'b0;
wire a0_bias_curr_mem_wait_i = 1'b0;
wire a0_bias_curr_mem_wr_wait_i = 1'b0;
wire a0_thresh_mem_wait_i    = 1'b0;
wire a0_thresh_mem_wr_wait_i = 1'b0;
wire a0_pot_mem_wait_i       = 1'b0;
wire a0_spike_mem_wait_i     = 1'b0;

wire hd_src_a_mem_wait_i     = 1'b0;
wire hd_src_a_mem_wr_wait_i  = 1'b0;
wire hd_src_b_mem_wait_i     = 1'b0;
wire hd_src_b_mem_wr_wait_i  = 1'b0;
wire hd_src_z_mem_wait_i     = 1'b0;
wire hd_src_z_mem_wr_wait_i  = 1'b0;
wire hd_src_r_mem_wait_i     = 1'b0;
wire hd_src_r_mem_wr_wait_i  = 1'b0;

// ─── Memory read data (combinatorial from arrays) ─────────────────────────────
wire [PROG_DATA_BITS-1:0] prog_mem_data_i =
    prog_mem[prog_mem_addr_o[PROG_IDX_BITS-1:0]];

wire [31:0] cfg_mem_data_i =
    cfg_mem[cfg_mem_addr_o[MEM_ADDR_BITS-1:0]];

wire [31:0] bba_mem_data_i =
    bba_mem[bba_mem_addr_o[MEM_ADDR_BITS-1:0]];

wire [31:0] fu_bba_mem_data_i =
    bba_mem[fu_bba_mem_addr_o[MEM_ADDR_BITS-1:0]];  // same physical RAM, second port

// snnAcc0 read data
wire [`WTD_BITS-1:0] s0_weight_mem_data_i =
    s0_weight_mem[s0_weight_mem_addr_o[MEM_ADDR_BITS-1:0]];
wire [`ACT_BITS-1:0] s0_act_mem_data_i =
    s0_act_mem[s0_act_mem_addr_o[MEM_ADDR_BITS-1:0]];
wire [`POT_BITS-1:0] s0_syn_curr_mem_data_i =
    s0_syn_curr_mem[s0_syn_curr_mem_addr_o[MEM_ADDR_BITS-1:0]];
wire [`WTD_BITS-1:0] s0_bias_curr_mem_data_i =
    s0_bias_curr_mem[s0_bias_curr_mem_addr_o[MEM_ADDR_BITS-1:0]];
wire [`WTD_BITS-1:0] s0_thresh_mem_data_i =
    s0_thresh_mem[s0_thresh_mem_addr_o[MEM_ADDR_BITS-1:0]];
wire [`POT_BITS-1:0] s0_pot_mem_data_i =
    s0_pot_mem[s0_pot_mem_addr_o[MEM_ADDR_BITS-1:0]];

// snnAcc1 read data
wire [`WTD_BITS-1:0] s1_weight_mem_data_i =
    s1_weight_mem[s1_weight_mem_addr_o[MEM_ADDR_BITS-1:0]];
wire [`ACT_BITS-1:0] s1_act_mem_data_i =
    s1_act_mem[s1_act_mem_addr_o[MEM_ADDR_BITS-1:0]];
wire [`POT_BITS-1:0] s1_syn_curr_mem_data_i =
    s1_syn_curr_mem[s1_syn_curr_mem_addr_o[MEM_ADDR_BITS-1:0]];
wire [`WTD_BITS-1:0] s1_bias_curr_mem_data_i =
    s1_bias_curr_mem[s1_bias_curr_mem_addr_o[MEM_ADDR_BITS-1:0]];
wire [`WTD_BITS-1:0] s1_thresh_mem_data_i =
    s1_thresh_mem[s1_thresh_mem_addr_o[MEM_ADDR_BITS-1:0]];
wire [`POT_BITS-1:0] s1_pot_mem_data_i =
    s1_pot_mem[s1_pot_mem_addr_o[MEM_ADDR_BITS-1:0]];

// annAcc read data
wire [`WTD_BITS-1:0] a0_weight_mem_data_i =
    a0_weight_mem[a0_weight_mem_addr_o[MEM_ADDR_BITS-1:0]];
wire [`ACT_BITS-1:0] a0_act_mem_data_i =
    a0_act_mem[a0_act_mem_addr_o[MEM_ADDR_BITS-1:0]];
wire [`POT_BITS-1:0] a0_syn_curr_mem_data_i =
    a0_syn_curr_mem[a0_syn_curr_mem_addr_o[MEM_ADDR_BITS-1:0]];
wire [`WTD_BITS-1:0] a0_bias_curr_mem_data_i =
    a0_bias_curr_mem[a0_bias_curr_mem_addr_o[MEM_ADDR_BITS-1:0]];
wire [`WTD_BITS-1:0] a0_thresh_mem_data_i =
    a0_thresh_mem[a0_thresh_mem_addr_o[MEM_ADDR_BITS-1:0]];
wire [`POT_BITS-1:0] a0_pot_mem_data_i =
    a0_pot_mem[a0_pot_mem_addr_o[MEM_ADDR_BITS-1:0]];

// Hadamard read data
wire [31:0] hd_src_a_mem_data_i =
    hd_src_a_mem[hd_src_a_mem_addr_o[MEM_ADDR_BITS-1:0]];
wire [31:0] hd_src_b_mem_data_i =
    hd_src_b_mem[hd_src_b_mem_addr_o[MEM_ADDR_BITS-1:0]];
wire [31:0] hd_src_z_mem_data_i =
    hd_src_z_mem[hd_src_z_mem_addr_o[MEM_ADDR_BITS-1:0]];
wire [31:0] hd_src_r_mem_data_i =
    hd_src_r_mem[hd_src_r_mem_addr_o[MEM_ADDR_BITS-1:0]];

// ─── Sequential write-backs to behavioral memories ────────────────────────────
always @(posedge clk) begin
    if (cfg_mem_wr_o)
        cfg_mem[cfg_mem_wr_addr_o[MEM_ADDR_BITS-1:0]] <= cfg_mem_wr_data_o;
    if (bba_mem_wr_o)
        bba_mem[bba_mem_wr_addr_o[MEM_ADDR_BITS-1:0]] <= bba_mem_wr_data_o;
end

// snnAcc0 write-backs
always @(posedge clk) begin
    if (s0_weight_mem_wr_o)
        s0_weight_mem[s0_weight_mem_wr_addr_o[MEM_ADDR_BITS-1:0]]
            <= s0_weight_mem_wr_data_o;
    if (s0_act_mem_wr_o)
        s0_act_mem[s0_act_mem_wr_addr_o[MEM_ADDR_BITS-1:0]]
            <= s0_act_mem_wr_data_o;
    if (s0_syn_curr_mem_wr_o)
        s0_syn_curr_mem[s0_syn_curr_mem_addr_o[MEM_ADDR_BITS-1:0]]
            <= s0_syn_curr_mem_data_o;
    if (s0_bias_curr_mem_wr_o)
        s0_bias_curr_mem[s0_bias_curr_mem_wr_addr_o[MEM_ADDR_BITS-1:0]]
            <= s0_bias_curr_mem_wr_data_o;
    if (s0_thresh_mem_wr_o)
        s0_thresh_mem[s0_thresh_mem_wr_addr_o[MEM_ADDR_BITS-1:0]]
            <= s0_thresh_mem_wr_data_o;
    if (s0_pot_mem_wr_o)
        s0_pot_mem[s0_pot_mem_addr_o[MEM_ADDR_BITS-1:0]]
            <= s0_pot_mem_data_o;
    if (s0_spike_mem_wr_o)
        s0_spike_mem[s0_spike_mem_addr_o[MEM_ADDR_BITS-1:0]]
            <= s0_spike_mem_data_o;
end

// snnAcc1 write-backs
always @(posedge clk) begin
    if (s1_weight_mem_wr_o)
        s1_weight_mem[s1_weight_mem_wr_addr_o[MEM_ADDR_BITS-1:0]]
            <= s1_weight_mem_wr_data_o;
    if (s1_act_mem_wr_o)
        s1_act_mem[s1_act_mem_wr_addr_o[MEM_ADDR_BITS-1:0]]
            <= s1_act_mem_wr_data_o;
    if (s1_syn_curr_mem_wr_o)
        s1_syn_curr_mem[s1_syn_curr_mem_addr_o[MEM_ADDR_BITS-1:0]]
            <= s1_syn_curr_mem_data_o;
    if (s1_bias_curr_mem_wr_o)
        s1_bias_curr_mem[s1_bias_curr_mem_wr_addr_o[MEM_ADDR_BITS-1:0]]
            <= s1_bias_curr_mem_wr_data_o;
    if (s1_thresh_mem_wr_o)
        s1_thresh_mem[s1_thresh_mem_wr_addr_o[MEM_ADDR_BITS-1:0]]
            <= s1_thresh_mem_wr_data_o;
    if (s1_pot_mem_wr_o)
        s1_pot_mem[s1_pot_mem_addr_o[MEM_ADDR_BITS-1:0]]
            <= s1_pot_mem_data_o;
    if (s1_spike_mem_wr_o)
        s1_spike_mem[s1_spike_mem_addr_o[MEM_ADDR_BITS-1:0]]
            <= s1_spike_mem_data_o;
end

// annAcc write-backs
always @(posedge clk) begin
    if (a0_weight_mem_wr_o)
        a0_weight_mem[a0_weight_mem_wr_addr_o[MEM_ADDR_BITS-1:0]]
            <= a0_weight_mem_wr_data_o;
    if (a0_act_mem_wr_o)
        a0_act_mem[a0_act_mem_wr_addr_o[MEM_ADDR_BITS-1:0]]
            <= a0_act_mem_wr_data_o;
    if (a0_syn_curr_mem_wr_o)
        a0_syn_curr_mem[a0_syn_curr_mem_addr_o[MEM_ADDR_BITS-1:0]]
            <= a0_syn_curr_mem_data_o;
    if (a0_bias_curr_mem_wr_o)
        a0_bias_curr_mem[a0_bias_curr_mem_wr_addr_o[MEM_ADDR_BITS-1:0]]
            <= a0_bias_curr_mem_wr_data_o;
    if (a0_thresh_mem_wr_o)
        a0_thresh_mem[a0_thresh_mem_wr_addr_o[MEM_ADDR_BITS-1:0]]
            <= a0_thresh_mem_wr_data_o;
    if (a0_pot_mem_wr_o)
        a0_pot_mem[a0_pot_mem_addr_o[MEM_ADDR_BITS-1:0]]
            <= a0_pot_mem_data_o;
    if (a0_spike_mem_wr_o)
        a0_spike_mem[a0_spike_mem_addr_o[MEM_ADDR_BITS-1:0]]
            <= a0_spike_mem_data_o;
end

// Hadamard write-backs
always @(posedge clk) begin
    if (hd_src_a_mem_wr_o)
        hd_src_a_mem[hd_src_a_mem_wr_addr_o[MEM_ADDR_BITS-1:0]]
            <= hd_src_a_mem_wr_data_o;
    if (hd_src_b_mem_wr_o)
        hd_src_b_mem[hd_src_b_mem_wr_addr_o[MEM_ADDR_BITS-1:0]]
            <= hd_src_b_mem_wr_data_o;
    if (hd_src_z_mem_wr_o)
        hd_src_z_mem[hd_src_z_mem_wr_addr_o[MEM_ADDR_BITS-1:0]]
            <= hd_src_z_mem_wr_data_o;
    if (hd_src_r_mem_wr_o)
        hd_src_r_mem[hd_src_r_mem_wr_addr_o[MEM_ADDR_BITS-1:0]]
            <= hd_src_r_mem_data_o;
end

// ─── DUT ──────────────────────────────────────────────────────────────────────
flexman #(
    .NUM_BUFFERS         (NUM_BUFFERS),
    .NUM_HW_ACCELERATORS (NUM_HW_ACCELERATORS),
    .WORDS_PER_CONFIG    (WORDS_PER_CONFIG),
    .CFG_ID_SZ           (CFG_ID_SZ),
    .BUFF_INDX_SZ        (BUFF_INDX_SZ),
    .TGT_ACC_SZ          (TGT_ACC_SZ),
    .TGT_COUNT_SZ        (TGT_COUNT_SZ),
    .PROG_ADDR_BITS      (PROG_ADDR_BITS),
    .PROG_DATA_BITS      (PROG_DATA_BITS),
    .NUM_SCH_ENTRIES     (NUM_SCH_ENTRIES),
    .COL_BUFF_ID_SZ      (COL_BUFF_ID_SZ)
) dut (
    .clk                  (clk),
    .reset                (reset),
    .test_stall_pipe      (test_stall_pipe),

    .sys_req_i            (sys_req_i),
    .sys_ack_o            (sys_ack_o),
    .sys_addr_i           (sys_addr_i),
    .sys_data_i           (sys_data_i),
    .sys_data_o           (sys_data_o),

    .start_program_i      (start_program_i),
    .program_addr_i       (program_addr_i),
    .prog_mem_addr_o      (prog_mem_addr_o),
    .prog_mem_data_i      (prog_mem_data_i),
    .prog_mem_req_o       (prog_mem_req_o),
    .prog_mem_wait_i      (prog_mem_wait_i),

    .cfg_mem_rd_o         (cfg_mem_rd_o),
    .cfg_mem_wait_i       (cfg_mem_wait_i),
    .cfg_mem_addr_o       (cfg_mem_addr_o),
    .cfg_mem_data_i       (cfg_mem_data_i),
    .cfg_mem_wr_o         (cfg_mem_wr_o),
    .cfg_mem_wr_addr_o    (cfg_mem_wr_addr_o),
    .cfg_mem_wr_data_o    (cfg_mem_wr_data_o),

    .bba_mem_rd_o         (bba_mem_rd_o),
    .bba_mem_wait_i       (bba_mem_wait_i),
    .bba_mem_addr_o       (bba_mem_addr_o),
    .bba_mem_data_i       (bba_mem_data_i),
    .bba_mem_wr_o         (bba_mem_wr_o),
    .bba_mem_wr_addr_o    (bba_mem_wr_addr_o),
    .bba_mem_wr_data_o    (bba_mem_wr_data_o),

    .fu_bba_mem_rd_o      (fu_bba_mem_rd_o),
    .fu_bba_mem_wait_i    (fu_bba_mem_wait_i),
    .fu_bba_mem_addr_o    (fu_bba_mem_addr_o),
    .fu_bba_mem_data_i    (fu_bba_mem_data_i),

    .mark_buff_as_full_i  (mark_buff_as_full_i),
    .full_buff_id_i       (full_buff_id_i),
    .full_buff_usage_i    (full_buff_usage_i),
    .nxt_input_pulse_o    (nxt_input_pulse_o),
    .nxt_output_pulse_o   (nxt_output_pulse_o),

    .cm_config_finished_o (cm_config_finished_o),

    // snnAcc0
    .s0_weight_mem_rd_o      (s0_weight_mem_rd_o),
    .s0_weight_mem_wait_i    (s0_weight_mem_wait_i),
    .s0_weight_mem_addr_o    (s0_weight_mem_addr_o),
    .s0_weight_mem_data_i    (s0_weight_mem_data_i),
    .s0_weight_mem_wr_o      (s0_weight_mem_wr_o),
    .s0_weight_mem_wr_wait_i (s0_weight_mem_wr_wait_i),
    .s0_weight_mem_wr_addr_o (s0_weight_mem_wr_addr_o),
    .s0_weight_mem_wr_data_o (s0_weight_mem_wr_data_o),

    .s0_act_mem_req_o        (s0_act_mem_req_o),
    .s0_act_mem_wait_i       (s0_act_mem_wait_i),
    .s0_act_mem_addr_o       (s0_act_mem_addr_o),
    .s0_act_mem_data_i       (s0_act_mem_data_i),
    .s0_act_mem_wr_o         (s0_act_mem_wr_o),
    .s0_act_mem_wr_wait_i    (s0_act_mem_wr_wait_i),
    .s0_act_mem_wr_addr_o    (s0_act_mem_wr_addr_o),
    .s0_act_mem_wr_data_o    (s0_act_mem_wr_data_o),

    .s0_syn_curr_mem_wr_o    (s0_syn_curr_mem_wr_o),
    .s0_syn_curr_mem_rd_o    (s0_syn_curr_mem_rd_o),
    .s0_syn_curr_mem_wait_i  (s0_syn_curr_mem_wait_i),
    .s0_syn_curr_mem_addr_o  (s0_syn_curr_mem_addr_o),
    .s0_syn_curr_mem_data_o  (s0_syn_curr_mem_data_o),
    .s0_syn_curr_mem_data_i  (s0_syn_curr_mem_data_i),

    .s0_bias_curr_mem_rd_o      (s0_bias_curr_mem_rd_o),
    .s0_bias_curr_mem_wait_i    (s0_bias_curr_mem_wait_i),
    .s0_bias_curr_mem_addr_o    (s0_bias_curr_mem_addr_o),
    .s0_bias_curr_mem_data_i    (s0_bias_curr_mem_data_i),
    .s0_bias_curr_mem_wr_o      (s0_bias_curr_mem_wr_o),
    .s0_bias_curr_mem_wr_wait_i (s0_bias_curr_mem_wr_wait_i),
    .s0_bias_curr_mem_wr_addr_o (s0_bias_curr_mem_wr_addr_o),
    .s0_bias_curr_mem_wr_data_o (s0_bias_curr_mem_wr_data_o),

    .s0_thresh_mem_rd_o      (s0_thresh_mem_rd_o),
    .s0_thresh_mem_wait_i    (s0_thresh_mem_wait_i),
    .s0_thresh_mem_addr_o    (s0_thresh_mem_addr_o),
    .s0_thresh_mem_data_i    (s0_thresh_mem_data_i),
    .s0_thresh_mem_wr_o      (s0_thresh_mem_wr_o),
    .s0_thresh_mem_wr_wait_i (s0_thresh_mem_wr_wait_i),
    .s0_thresh_mem_wr_addr_o (s0_thresh_mem_wr_addr_o),
    .s0_thresh_mem_wr_data_o (s0_thresh_mem_wr_data_o),

    .s0_pot_mem_wr_o         (s0_pot_mem_wr_o),
    .s0_pot_mem_rd_o         (s0_pot_mem_rd_o),
    .s0_pot_mem_wait_i       (s0_pot_mem_wait_i),
    .s0_pot_mem_addr_o       (s0_pot_mem_addr_o),
    .s0_pot_mem_data_o       (s0_pot_mem_data_o),
    .s0_pot_mem_data_i       (s0_pot_mem_data_i),

    .s0_spike_mem_wr_o       (s0_spike_mem_wr_o),
    .s0_spike_mem_wait_i     (s0_spike_mem_wait_i),
    .s0_spike_mem_addr_o     (s0_spike_mem_addr_o),
    .s0_spike_mem_data_o     (s0_spike_mem_data_o),

    // snnAcc1
    .s1_weight_mem_rd_o      (s1_weight_mem_rd_o),
    .s1_weight_mem_wait_i    (s1_weight_mem_wait_i),
    .s1_weight_mem_addr_o    (s1_weight_mem_addr_o),
    .s1_weight_mem_data_i    (s1_weight_mem_data_i),
    .s1_weight_mem_wr_o      (s1_weight_mem_wr_o),
    .s1_weight_mem_wr_wait_i (s1_weight_mem_wr_wait_i),
    .s1_weight_mem_wr_addr_o (s1_weight_mem_wr_addr_o),
    .s1_weight_mem_wr_data_o (s1_weight_mem_wr_data_o),

    .s1_act_mem_req_o        (s1_act_mem_req_o),
    .s1_act_mem_wait_i       (s1_act_mem_wait_i),
    .s1_act_mem_addr_o       (s1_act_mem_addr_o),
    .s1_act_mem_data_i       (s1_act_mem_data_i),
    .s1_act_mem_wr_o         (s1_act_mem_wr_o),
    .s1_act_mem_wr_wait_i    (s1_act_mem_wr_wait_i),
    .s1_act_mem_wr_addr_o    (s1_act_mem_wr_addr_o),
    .s1_act_mem_wr_data_o    (s1_act_mem_wr_data_o),

    .s1_syn_curr_mem_wr_o    (s1_syn_curr_mem_wr_o),
    .s1_syn_curr_mem_rd_o    (s1_syn_curr_mem_rd_o),
    .s1_syn_curr_mem_wait_i  (s1_syn_curr_mem_wait_i),
    .s1_syn_curr_mem_addr_o  (s1_syn_curr_mem_addr_o),
    .s1_syn_curr_mem_data_o  (s1_syn_curr_mem_data_o),
    .s1_syn_curr_mem_data_i  (s1_syn_curr_mem_data_i),

    .s1_bias_curr_mem_rd_o      (s1_bias_curr_mem_rd_o),
    .s1_bias_curr_mem_wait_i    (s1_bias_curr_mem_wait_i),
    .s1_bias_curr_mem_addr_o    (s1_bias_curr_mem_addr_o),
    .s1_bias_curr_mem_data_i    (s1_bias_curr_mem_data_i),
    .s1_bias_curr_mem_wr_o      (s1_bias_curr_mem_wr_o),
    .s1_bias_curr_mem_wr_wait_i (s1_bias_curr_mem_wr_wait_i),
    .s1_bias_curr_mem_wr_addr_o (s1_bias_curr_mem_wr_addr_o),
    .s1_bias_curr_mem_wr_data_o (s1_bias_curr_mem_wr_data_o),

    .s1_thresh_mem_rd_o      (s1_thresh_mem_rd_o),
    .s1_thresh_mem_wait_i    (s1_thresh_mem_wait_i),
    .s1_thresh_mem_addr_o    (s1_thresh_mem_addr_o),
    .s1_thresh_mem_data_i    (s1_thresh_mem_data_i),
    .s1_thresh_mem_wr_o      (s1_thresh_mem_wr_o),
    .s1_thresh_mem_wr_wait_i (s1_thresh_mem_wr_wait_i),
    .s1_thresh_mem_wr_addr_o (s1_thresh_mem_wr_addr_o),
    .s1_thresh_mem_wr_data_o (s1_thresh_mem_wr_data_o),

    .s1_pot_mem_wr_o         (s1_pot_mem_wr_o),
    .s1_pot_mem_rd_o         (s1_pot_mem_rd_o),
    .s1_pot_mem_wait_i       (s1_pot_mem_wait_i),
    .s1_pot_mem_addr_o       (s1_pot_mem_addr_o),
    .s1_pot_mem_data_o       (s1_pot_mem_data_o),
    .s1_pot_mem_data_i       (s1_pot_mem_data_i),

    .s1_spike_mem_wr_o       (s1_spike_mem_wr_o),
    .s1_spike_mem_wait_i     (s1_spike_mem_wait_i),
    .s1_spike_mem_addr_o     (s1_spike_mem_addr_o),
    .s1_spike_mem_data_o     (s1_spike_mem_data_o),

    // annAcc
    .a0_weight_mem_rd_o      (a0_weight_mem_rd_o),
    .a0_weight_mem_wait_i    (a0_weight_mem_wait_i),
    .a0_weight_mem_addr_o    (a0_weight_mem_addr_o),
    .a0_weight_mem_data_i    (a0_weight_mem_data_i),
    .a0_weight_mem_wr_o      (a0_weight_mem_wr_o),
    .a0_weight_mem_wr_wait_i (a0_weight_mem_wr_wait_i),
    .a0_weight_mem_wr_addr_o (a0_weight_mem_wr_addr_o),
    .a0_weight_mem_wr_data_o (a0_weight_mem_wr_data_o),

    .a0_act_mem_req_o        (a0_act_mem_req_o),
    .a0_act_mem_wait_i       (a0_act_mem_wait_i),
    .a0_act_mem_addr_o       (a0_act_mem_addr_o),
    .a0_act_mem_data_i       (a0_act_mem_data_i),
    .a0_act_mem_wr_o         (a0_act_mem_wr_o),
    .a0_act_mem_wr_wait_i    (a0_act_mem_wr_wait_i),
    .a0_act_mem_wr_addr_o    (a0_act_mem_wr_addr_o),
    .a0_act_mem_wr_data_o    (a0_act_mem_wr_data_o),

    .a0_syn_curr_mem_wr_o    (a0_syn_curr_mem_wr_o),
    .a0_syn_curr_mem_rd_o    (a0_syn_curr_mem_rd_o),
    .a0_syn_curr_mem_wait_i  (a0_syn_curr_mem_wait_i),
    .a0_syn_curr_mem_addr_o  (a0_syn_curr_mem_addr_o),
    .a0_syn_curr_mem_data_o  (a0_syn_curr_mem_data_o),
    .a0_syn_curr_mem_data_i  (a0_syn_curr_mem_data_i),

    .a0_bias_curr_mem_rd_o      (a0_bias_curr_mem_rd_o),
    .a0_bias_curr_mem_wait_i    (a0_bias_curr_mem_wait_i),
    .a0_bias_curr_mem_addr_o    (a0_bias_curr_mem_addr_o),
    .a0_bias_curr_mem_data_i    (a0_bias_curr_mem_data_i),
    .a0_bias_curr_mem_wr_o      (a0_bias_curr_mem_wr_o),
    .a0_bias_curr_mem_wr_wait_i (a0_bias_curr_mem_wr_wait_i),
    .a0_bias_curr_mem_wr_addr_o (a0_bias_curr_mem_wr_addr_o),
    .a0_bias_curr_mem_wr_data_o (a0_bias_curr_mem_wr_data_o),

    .a0_thresh_mem_rd_o      (a0_thresh_mem_rd_o),
    .a0_thresh_mem_wait_i    (a0_thresh_mem_wait_i),
    .a0_thresh_mem_addr_o    (a0_thresh_mem_addr_o),
    .a0_thresh_mem_data_i    (a0_thresh_mem_data_i),
    .a0_thresh_mem_wr_o      (a0_thresh_mem_wr_o),
    .a0_thresh_mem_wr_wait_i (a0_thresh_mem_wr_wait_i),
    .a0_thresh_mem_wr_addr_o (a0_thresh_mem_wr_addr_o),
    .a0_thresh_mem_wr_data_o (a0_thresh_mem_wr_data_o),

    .a0_pot_mem_wr_o         (a0_pot_mem_wr_o),
    .a0_pot_mem_rd_o         (a0_pot_mem_rd_o),
    .a0_pot_mem_wait_i       (a0_pot_mem_wait_i),
    .a0_pot_mem_addr_o       (a0_pot_mem_addr_o),
    .a0_pot_mem_data_o       (a0_pot_mem_data_o),
    .a0_pot_mem_data_i       (a0_pot_mem_data_i),

    .a0_spike_mem_wr_o       (a0_spike_mem_wr_o),
    .a0_spike_mem_wait_i     (a0_spike_mem_wait_i),
    .a0_spike_mem_addr_o     (a0_spike_mem_addr_o),
    .a0_spike_mem_data_o     (a0_spike_mem_data_o),

    // Hadamard
    .hd_src_a_mem_rd_o       (hd_src_a_mem_rd_o),
    .hd_src_a_mem_wait_i     (hd_src_a_mem_wait_i),
    .hd_src_a_mem_addr_o     (hd_src_a_mem_addr_o),
    .hd_src_a_mem_data_i     (hd_src_a_mem_data_i),
    .hd_src_a_mem_wr_o       (hd_src_a_mem_wr_o),
    .hd_src_a_mem_wr_wait_i  (hd_src_a_mem_wr_wait_i),
    .hd_src_a_mem_wr_addr_o  (hd_src_a_mem_wr_addr_o),
    .hd_src_a_mem_wr_data_o  (hd_src_a_mem_wr_data_o),

    .hd_src_b_mem_rd_o       (hd_src_b_mem_rd_o),
    .hd_src_b_mem_wait_i     (hd_src_b_mem_wait_i),
    .hd_src_b_mem_addr_o     (hd_src_b_mem_addr_o),
    .hd_src_b_mem_data_i     (hd_src_b_mem_data_i),
    .hd_src_b_mem_wr_o       (hd_src_b_mem_wr_o),
    .hd_src_b_mem_wr_wait_i  (hd_src_b_mem_wr_wait_i),
    .hd_src_b_mem_wr_addr_o  (hd_src_b_mem_wr_addr_o),
    .hd_src_b_mem_wr_data_o  (hd_src_b_mem_wr_data_o),

    .hd_src_z_mem_rd_o       (hd_src_z_mem_rd_o),
    .hd_src_z_mem_wait_i     (hd_src_z_mem_wait_i),
    .hd_src_z_mem_addr_o     (hd_src_z_mem_addr_o),
    .hd_src_z_mem_data_i     (hd_src_z_mem_data_i),
    .hd_src_z_mem_wr_o       (hd_src_z_mem_wr_o),
    .hd_src_z_mem_wr_wait_i  (hd_src_z_mem_wr_wait_i),
    .hd_src_z_mem_wr_addr_o  (hd_src_z_mem_wr_addr_o),
    .hd_src_z_mem_wr_data_o  (hd_src_z_mem_wr_data_o),

    .hd_src_r_mem_rd_o       (hd_src_r_mem_rd_o),
    .hd_src_r_mem_wait_i     (hd_src_r_mem_wait_i),
    .hd_src_r_mem_addr_o     (hd_src_r_mem_addr_o),
    .hd_src_r_mem_data_i     (hd_src_r_mem_data_i),
    .hd_src_r_mem_wr_o       (hd_src_r_mem_wr_o),
    .hd_src_r_mem_wr_addr_o  (hd_src_r_mem_wr_addr_o),
    .hd_src_r_mem_wr_wait_i  (hd_src_r_mem_wr_wait_i),
    .hd_src_r_mem_data_o     (hd_src_r_mem_data_o)
);

// ─── Monitors ────────────────────────────────────────────────────────────────
integer nxt_in_count;
integer nxt_out_count;
integer fill_wr_count;
integer test_num;

initial begin
    nxt_in_count  = 0;
    nxt_out_count = 0;
    fill_wr_count = 0;
    test_num      = 0;
end

always @(posedge clk) begin
    if (!reset) begin
        if (nxt_input_pulse_o) begin
            nxt_in_count = nxt_in_count + 1;
            $display("[%0t] T%0d: nxt_input_pulse  #%0d", $time, test_num, nxt_in_count);
        end
        if (nxt_output_pulse_o) begin
            nxt_out_count = nxt_out_count + 1;
            $display("[%0t] T%0d: nxt_output_pulse #%0d", $time, test_num, nxt_out_count);
        end
        if (s0_act_mem_wr_o) begin
            fill_wr_count = fill_wr_count + 1;
            $display("[%0t] T%0d: s0_act FILL wr [%0d] = %08h",
                     $time, test_num,
                     s0_act_mem_wr_addr_o, s0_act_mem_wr_data_o);
        end
    end
end

// ─── Waveform dump ───────────────────────────────────────────────────────────
initial begin
    $dumpfile("tb_flexman.vcd");
    $dumpvars(0, tb_flexman);
end

// ─── Timeout ─────────────────────────────────────────────────────────────────
initial begin
    #500000;
    $display("[TIMEOUT] simulation exceeded 500000 time units (test %0d).", test_num);
    $finish;
end

// ─── AXI helper ──────────────────────────────────────────────────────────────
// Single-cycle write: drives req for one posedge, then deasserts.
// For fill_unit's combinatorial ack the write is registered on that posedge.
task axi_write;
    input [31:0] addr;
    input [31:0] data;
    begin
        @(posedge clk); #1;
        sys_req_i  = 1'b1;
        sys_addr_i = addr;
        sys_data_i = data;
        @(posedge clk); #1;
        sys_req_i  = 1'b0;
        sys_addr_i = 32'h0;
        sys_data_i = 32'h0;
    end
endtask

// ─── Shared init helper ───────────────────────────────────────────────────────
integer mi;
task clear_all_mems;
    begin
        for (mi = 0; mi < PROG_DEPTH; mi = mi + 1)
            prog_mem[mi] = 32'h0;
        for (mi = 0; mi < MEM_DEPTH; mi = mi + 1) begin
            cfg_mem[mi]          = 32'h0;
            bba_mem[mi]          = 32'h0;
            s0_weight_mem[mi]    = 32'h0;
            s0_act_mem[mi]       = 32'h0;
            s0_syn_curr_mem[mi]  = 32'h0;
            s0_bias_curr_mem[mi] = 32'h0;
            s0_thresh_mem[mi]    = 32'h0;
            s0_pot_mem[mi]       = 32'h0;
            s0_spike_mem[mi]     = 32'h0;
            s1_weight_mem[mi]    = 32'h0;
            s1_act_mem[mi]       = 32'h0;
            s1_syn_curr_mem[mi]  = 32'h0;
            s1_bias_curr_mem[mi] = 32'h0;
            s1_thresh_mem[mi]    = 32'h0;
            s1_pot_mem[mi]       = 32'h0;
            s1_spike_mem[mi]     = 32'h0;
            a0_weight_mem[mi]    = 32'h0;
            a0_act_mem[mi]       = 32'h0;
            a0_syn_curr_mem[mi]  = 32'h0;
            a0_bias_curr_mem[mi] = 32'h0;
            a0_thresh_mem[mi]    = 32'h0;
            a0_pot_mem[mi]       = 32'h0;
            a0_spike_mem[mi]     = 32'h0;
            hd_src_a_mem[mi]     = 32'h0;
            hd_src_b_mem[mi]     = 32'h0;
            hd_src_z_mem[mi]     = 32'h0;
            hd_src_r_mem[mi]     = 32'h0;
        end
    end
endtask

// ─── Stimulus ────────────────────────────────────────────────────────────────
initial begin
    // ── Initial state ──────────────────────────────────────────────────────
    reset               = 1'b1;
    sys_req_i           = 1'b0;
    sys_addr_i          = 32'h0;
    sys_data_i          = 32'h0;
    start_program_i     = 1'b0;
    program_addr_i      = {PROG_ADDR_BITS{1'b0}};
    mark_buff_as_full_i = 1'b0;
    full_buff_id_i      = {BUFF_INDX_SZ{1'b0}};
    full_buff_usage_i   = {TGT_COUNT_SZ{1'b0}};
    clear_all_mems;

    // ══════════════════════════════════════════════════════════════════════
    // T1: NXT(in+out) + STOP
    //   No tasks; NXT on an empty table fires immediately.
    //   Expected: nxt_in_count=1, nxt_out_count=1
    // ══════════════════════════════════════════════════════════════════════
    test_num = 1;
    prog_mem[0] = nxt_inst(1'b1, 1'b1);   // NXT nxt_in=1 nxt_out=1
    prog_mem[1] = STOP_INST;

    repeat(4) @(posedge clk); #1;
    reset = 1'b0;

    @(posedge clk); #1;
    start_program_i = 1'b1;
    program_addr_i  = {PROG_ADDR_BITS{1'b0}};
    @(posedge clk); #1;
    start_program_i = 1'b0;

    repeat(60) @(posedge clk);

    if (nxt_in_count == 1 && nxt_out_count == 1) begin
        $display("[T1] PASS  NXT in×%0d out×%0d", nxt_in_count, nxt_out_count);
    end else begin
        $display("[T1] FAIL  NXT in×%0d (exp 1)  out×%0d (exp 1)",
                 nxt_in_count, nxt_out_count);
        #20 $finish;
    end

    // ══════════════════════════════════════════════════════════════════════
    // T2: FILL buf=0 (4 words, 0xDEADBEEF) → s0_act_mem + STOP
    //   fill_unit writes 4 words starting at word-address 0 of s0_act.
    //   Expected: s0_act_mem[0..3] == 32'hDEADBEEF
    // ══════════════════════════════════════════════════════════════════════
    reset = 1'b1;
    nxt_in_count  = 0;
    nxt_out_count = 0;
    fill_wr_count = 0;
    clear_all_mems;
    repeat(4) @(posedge clk); #1;
    reset = 1'b0;

    test_num = 2;

    // Program: FILL(buf=0, col=0, #tgt=1, size=4, val=0xDEADBEEF) + STOP
    // fill_w1: [2:0]=101, [6:3]=buf_id, [7]=0, [8]=col, [11:9]=#tgt, [31:12]=sz
    prog_mem[0] = fill_w1(4'd0, 1'b0, 3'd1, 20'd4);  // 32'h0000_4205
    prog_mem[1] = 32'hDEAD_BEEF;                       // fill value (word 2)
    prog_mem[2] = STOP_INST;

    // BBA memory: buffer 0 base word-address = 0  (fill writes to s0_act[0..3])
    bba_mem[0] = 32'h0000_0000;

    // fill_unit mem_sel table: buffer 0 → s0_act (index 1, one-hot bit 1)
    // AXI addr: FU_TABLE_ADDR + (buff_id << 2) = 0xC000_0000
    axi_write(32'hC000_0000, 32'h0000_0002);

    @(posedge clk); #1;
    start_program_i = 1'b1;
    program_addr_i  = {PROG_ADDR_BITS{1'b0}};
    @(posedge clk); #1;
    start_program_i = 1'b0;

    // BBA read takes 1 cycle; then 4 write cycles; then done pulse.
    // Allow generous margin for scheduler fetch + dispatch latency.
    repeat(100) @(posedge clk);

    if (fill_wr_count == 4           &&
        s0_act_mem[0] == 32'hDEADBEEF &&
        s0_act_mem[1] == 32'hDEADBEEF &&
        s0_act_mem[2] == 32'hDEADBEEF &&
        s0_act_mem[3] == 32'hDEADBEEF) begin
        $display("[T2] PASS  FILL wrote %0d words; s0_act[0..3] = 0xDEADBEEF",
                 fill_wr_count);
    end else begin
        $display("[T2] FAIL  fill_wr_count=%0d (exp 4)  s0_act[0..3]=%08h %08h %08h %08h",
                 fill_wr_count,
                 s0_act_mem[0], s0_act_mem[1], s0_act_mem[2], s0_act_mem[3]);
        #20 $finish;
    end

    // ══════════════════════════════════════════════════════════════════════
    $display("[ALL] PASS");
    #20 $finish;
end

endmodule // tb_flexman
