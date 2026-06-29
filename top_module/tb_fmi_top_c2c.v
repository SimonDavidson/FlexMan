// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
`timescale 10ps/1ps
`include "../shared/constants.v"

// =============================================================================
// tb_fmi_top_c2c.v — FULL recurrent network g3->g4->g5->g6 on fmi_top
// Authors: Simon Davidson & Claude   Created: 2026-06-29   Last modified: 2026-06-29
//
// The capstone integration: EVERY layer of the FMI recurrent SNN runs on the
// fabric, driven only by the real log-mel input each timestep. SIX dispatches per
// timestep through the scheduler hardware loop, with the intermediate spike trains
// handed off BETWEEN dispatches via the scheduler buffer system (the new coverage
// vs C2b, where the tb re-loaded spike3 each step):
//
//   1. con1 (cfg0, g3) cin=2 real-MAC + adaptive-LIF  : act(x,one) -> spike3
//   2. con2 (cfg1, g4) conv-only accumulate (skip=1)  : spike3     -> syn4
//   3. con3 (cfg2, g4) LIF, read-before-write spike4  : spike4[t-1]-> spike4[t]
//   4. con4 (cfg3, g5) conv-only accumulate (skip=1)  : spike4[t]  -> syn5
//   5. con5 (cfg4, g5) LIF, read-before-write spike5  : spike5[t-1]-> spike5[t]
//   6. con6 (cfg5, g6) conv-FC + readout neuron       : spike5[t]  -> pot6
//
// Buffer choreography (ISA_REFERENCE.md modes; verified vs buffer_state_entry.v):
//   buf0 act     SOURCE  : tb-fed + re-marked full each timestep (cnt=1)
//   buf1 spike3  con1 TARGET #t=1 -> con2 SOURCE  (fresh feed-forward generation)
//   buf2 spike4  con3 RW #t=2     -> con4 SOURCE   (recurrent: RW keeps it full for
//                                                   con3[t+1] while con4 consumes to 1)
//   buf3 spike5  con5 RW #t=2     -> con6 SOURCE   (same recurrent + feed-forward role)
//   buf4 dummy   con6 RW #t=1     -> (no consumer) (con6 never spikes; self-cycles)
//
// === AXI-readback build: NO -access wrc (like C2b 32780d4) ===
// ZERO hierarchical (.mem[]) backdoor references, so it compiles WITHOUT Xcelium's
// -access wrc (which disables optimisation and dominates the long T=249 run). Each
// access avoids the backdoor:
//   - spike3/4/5 compare : AXI reads through the host POOL_RD_BASE window (real DUT
//                          read port; the path C1 validated against the backdoor).
//   - input load         : a tb-side write port MUX'd over each pool bank's DUT
//                          write port (port connectivity, not a hierarchical ref);
//                          driven only in the per-timestep gate window (DUT idle).
//   - pot6 compare       : pot6 lives in the dedicated pot_mem (NOT in the pool, so
//                          POOL_RD cannot reach it) -> a tb-side READ port MUX'd over
//                          pot_mem's re/addr inputs, sampled in the idle gate window.
//   - dedicated mems     : $readmemh into the tb sram_model instances (no wrc).
//   - state zeroing       : relies on sram_model's init-zero (no clear backdoor).
//
// Memory layout (all hex already on disk; NO golden regeneration, NO RTL change):
//   pool   : ACT 0 | SPIKE3 256 | SPIKE4 320 | SPIKE5 384 | DUMMY 448
//            SYN3 512 | SYN4 2560 | SYN5 4608 | SYN6 6656
//   weights: packed con1@0 con2@32 con3@2592 con4@7712 con5@17952 con6@38432
//   params : per-group slice g3@0 g4@2048 g5@4096 g6@6144 (each <=1920/group)
//
// Golden: fmi/gen_c2c_golden.py (sample 0). Compared EVERY timestep. Committed at
// T=16 (fast regression); the full T=249 pass is VERIFIED (0 failures, ~7.5 min) —
// bump T_C2C to 249 to reproduce (goldens already hold all 249).
// =============================================================================
module tb_fmi_top_c2c;

// ─── C2c test size (16 = fast regression; 249 = full network pass, verified) ──
localparam integer T_C2C   = 16;
localparam integer SETTLE  = 16;   // cycles to let the writeback pipeline drain

// ─── Fabric params (match fmi_top defaults) ──────────────────────────────────
localparam PROG_DATA_BITS = 32;

// ─── TB memory sizing ────────────────────────────────────────────────────────
localparam CFG_DEPTH     = 256;
localparam CFG_ADDR_BITS = 8;
localparam PROG_DEPTH    = 64;
localparam PROG_IDX_BITS = 6;
localparam DATA_DEPTH    = 8192;    // pool banks + per-group param/pot/ada mems
localparam WEIGHT_DEPTH  = 49152;   // 6 weight sets packed (ends at 40352)

// ─── Address map ─────────────────────────────────────────────────────────────
localparam [31:0] FMI_CFG_BASE = 32'h1000_0000;
localparam [31:0] POOL_RD_BASE = 32'h1010_0000;
localparam [31:0] SCH_LOAD_PC  = 32'hE000_0000;
localparam [31:0] SCH_START    = 32'hE010_0000;
localparam [31:0] SCH_MARKFULL = 32'hE050_0000;
localparam STOP_INST = 32'h0000_0002;

// ─── Pool buffer layout ──────────────────────────────────────────────────────
localparam ACT_BASE    = 0;      // con1 cin=2 real act (x[120] then one[120]) [0..239]
localparam SPIKE3_BASE = 256;    // g3 spike3 (con1 out / con2 in)             [256..315]
localparam SPIKE4_BASE = 320;    // g4 spike4 (con3 rw / con4 in)              [320..379]
localparam SPIKE5_BASE = 384;    // g5 spike5 (con5 rw / con6 in)              [384..443]
localparam DUMMY_BASE  = 448;    // con6 spike output (=0, never fires)        [448..507]
localparam SYN3_BASE   = 512;    // g3 syn (carried)                          [512..2431]
localparam SYN4_BASE   = 2560;   // g4 syn (carried)                          [2560..4479]
localparam SYN5_BASE   = 4608;   // g5 syn (carried)                          [4608..6527]
localparam SYN6_BASE   = 6656;   // g6 syn (inj6, fresh; 1 neuron)            [6656]

// ─── Dedicated-mem weight bases (additive: row_base = weight_base) ───────────
localparam W_CON1 = 0;       // 16*2*1   = 32   words
localparam W_CON2 = 32;      // 32*16*5  = 2560
localparam W_CON3 = 2592;    // 32*32*5  = 5120
localparam W_CON4 = 7712;    // 64*32*5  = 10240
localparam W_CON5 = 17952;   // 64*64*5  = 20480
localparam W_CON6 = 38432;   // 1*64*30  = 1920  (ends 40352)

// ─── Dedicated-mem per-group param/pot/ada bases (each group <=1920) ─────────
localparam G3 = 0;
localparam G4 = 2048;
localparam G5 = 4096;
localparam G6 = 6144;

integer errors = 0;

// ─── Clock / reset ───────────────────────────────────────────────────────────
reg clk = 1'b0;
reg reset = 1'b1;
reg test_stall_pipe = 1'b0;
always #5 clk = ~clk;

// ─── Host AXI ────────────────────────────────────────────────────────────────
reg         sys_req_i  = 1'b0;
reg  [31:0] sys_addr_i = 32'h0;
reg  [31:0] sys_data_i = 32'h0;
wire        sys_ack_o;
wire [31:0] sys_data_o;

// ─── DUT memory-side nets ────────────────────────────────────────────────────
wire [`ADDR_SIZE-1:0]     prog_mem_addr_o;
wire                      prog_mem_req_o;
wire                      prog_mem_wr_o;
wire [9:0]                prog_mem_wr_addr_o;
wire [PROG_DATA_BITS-1:0] prog_mem_wr_data_o;

wire         cfg_mem_rd_o,  cfg_mem_wr_o;
wire [31:0]  cfg_mem_addr_o, cfg_mem_wr_addr_o, cfg_mem_wr_data_o;
wire         bba_mem_rd_o,  bba_mem_wr_o;
wire [31:0]  bba_mem_addr_o, bba_mem_wr_addr_o, bba_mem_wr_data_o;

wire nxt_input_pulse_o, nxt_output_pulse_o, cm_config_finished_o;

// dedicated acc memory nets
wire                  weight_mem_rd_o;   wire [`ADDR_SIZE-1:0] weight_mem_addr_o;
wire  [`WTD_BITS-1:0] weight_mem_data_i;
wire                  thresh_mem_rd_o;   wire [`ADDR_SIZE-1:0] thresh_mem_addr_o;
wire  [`WTD_BITS-1:0] thresh_mem_data_i;
wire                  dcy_syn_mem_rd_o;  wire [`ADDR_SIZE-1:0] dcy_syn_mem_addr_o;
wire           [31:0] dcy_syn_mem_data_i;
wire                  dcy_mem_mem_rd_o;  wire [`ADDR_SIZE-1:0] dcy_mem_mem_addr_o;
wire           [31:0] dcy_mem_mem_data_i;
wire                  b_eff_mem_rd_o;    wire [`ADDR_SIZE-1:0] b_eff_mem_addr_o;
wire           [31:0] b_eff_mem_data_i;
wire                  dcy_ada_mem_rd_o;  wire [`ADDR_SIZE-1:0] dcy_ada_mem_addr_o;
wire           [31:0] dcy_ada_mem_data_i;
wire                  scl_ada_mem_rd_o;  wire [`ADDR_SIZE-1:0] scl_ada_mem_addr_o;
wire           [31:0] scl_ada_mem_data_i;
wire                  pot_mem_wr_o, pot_mem_rd_o;  wire [`ADDR_SIZE-1:0] pot_mem_addr_o;
wire  [`POT_BITS-1:0] pot_mem_data_o, pot_mem_data_i;
wire                  ada_mem_wr_o, ada_mem_rd_o;  wire [`ADDR_SIZE-1:0] ada_mem_addr_o;
wire           [31:0] ada_mem_data_o, ada_mem_data_i;

// pool bank nets (DUT side)
wire                  m0_rd, m0_wr;  wire [`ADDR_SIZE-1:0] m0_addr;  wire [31:0] m0_wdata, m0_rdata;
wire                  m1_rd, m1_wr;  wire [`ADDR_SIZE-1:0] m1_addr;  wire [31:0] m1_wdata, m1_rdata;
wire                  m2_rd, m2_wr;  wire [`ADDR_SIZE-1:0] m2_addr;  wire [31:0] m2_wdata, m2_rdata;
wire                  m3_rd, m3_wr;  wire [`ADDR_SIZE-1:0] m3_addr;  wire [31:0] m3_wdata, m3_rdata;

// ─── Behavioural config tables (combinational read) ──────────────────────────
reg [PROG_DATA_BITS-1:0] prog_mem [0:PROG_DEPTH-1];
reg [31:0]               cfg_mem  [0:CFG_DEPTH-1];
reg [31:0]               bba_mem  [0:CFG_DEPTH-1];

wire [PROG_DATA_BITS-1:0] prog_mem_data_i = prog_mem[prog_mem_addr_o[PROG_IDX_BITS-1:0]];
wire [31:0]               cfg_mem_data_i  = cfg_mem [cfg_mem_addr_o [CFG_ADDR_BITS-1:0]];
wire [31:0]               bba_mem_data_i  = bba_mem [bba_mem_addr_o [CFG_ADDR_BITS-1:0]];

always @(posedge clk) begin
    if (cfg_mem_wr_o) cfg_mem[cfg_mem_wr_addr_o[CFG_ADDR_BITS-1:0]] <= cfg_mem_wr_data_o;
    if (bba_mem_wr_o) bba_mem[bba_mem_wr_addr_o[CFG_ADDR_BITS-1:0]] <= bba_mem_wr_data_o;
end

// ─── TB-side pot_mem READ port, mux'd over pot_mem's re/addr (no backdoor) ────
// pot6 is in the dedicated pot_mem, not the pool, so the AXI POOL_RD window cannot
// reach it. Drive a read into pot_mem @ G6 in the idle gate window (DUT not using
// pot_mem) and capture the registered rdata -- port connectivity, no -access wrc.
reg        tb_pot_rd = 1'b0;
wire                   pot_mem_re_x   = tb_pot_rd ? 1'b1      : pot_mem_rd_o;
wire [15:0]            pot_mem_addr_x = tb_pot_rd ? G6[15:0]  : pot_mem_addr_o[15:0];

// ─── Dedicated acc memories (sram_model, 1-cycle; $readmemh-loaded, no wrc) ──
sram_model #(.DATA_W(`WTD_BITS), .DEPTH(WEIGHT_DEPTH)) u_weight_mem (
    .clk(clk), .we(1'b0), .re(weight_mem_rd_o),
    .addr(weight_mem_addr_o[15:0]), .wdata({`WTD_BITS{1'b0}}), .rdata(weight_mem_data_i));
sram_model #(.DATA_W(`WTD_BITS), .DEPTH(DATA_DEPTH)) u_thresh_mem (
    .clk(clk), .we(1'b0), .re(thresh_mem_rd_o),
    .addr(thresh_mem_addr_o[15:0]), .wdata({`WTD_BITS{1'b0}}), .rdata(thresh_mem_data_i));
sram_model #(.DATA_W(32), .DEPTH(DATA_DEPTH)) u_dcy_syn_mem (
    .clk(clk), .we(1'b0), .re(dcy_syn_mem_rd_o),
    .addr(dcy_syn_mem_addr_o[15:0]), .wdata(32'b0), .rdata(dcy_syn_mem_data_i));
sram_model #(.DATA_W(32), .DEPTH(DATA_DEPTH)) u_dcy_mem_mem (
    .clk(clk), .we(1'b0), .re(dcy_mem_mem_rd_o),
    .addr(dcy_mem_mem_addr_o[15:0]), .wdata(32'b0), .rdata(dcy_mem_mem_data_i));
sram_model #(.DATA_W(32), .DEPTH(DATA_DEPTH)) u_b_eff_mem (
    .clk(clk), .we(1'b0), .re(b_eff_mem_rd_o),
    .addr(b_eff_mem_addr_o[15:0]), .wdata(32'b0), .rdata(b_eff_mem_data_i));
sram_model #(.DATA_W(32), .DEPTH(DATA_DEPTH)) u_dcy_ada_mem (
    .clk(clk), .we(1'b0), .re(dcy_ada_mem_rd_o),
    .addr(dcy_ada_mem_addr_o[15:0]), .wdata(32'b0), .rdata(dcy_ada_mem_data_i));
sram_model #(.DATA_W(32), .DEPTH(DATA_DEPTH)) u_scl_ada_mem (
    .clk(clk), .we(1'b0), .re(scl_ada_mem_rd_o),
    .addr(scl_ada_mem_addr_o[15:0]), .wdata(32'b0), .rdata(scl_ada_mem_data_i));
sram_model #(.DATA_W(`POT_BITS), .DEPTH(DATA_DEPTH)) u_pot_mem (
    .clk(clk), .we(pot_mem_wr_o), .re(pot_mem_re_x),
    .addr(pot_mem_addr_x), .wdata(pot_mem_data_o), .rdata(pot_mem_data_i));
sram_model #(.DATA_W(32), .DEPTH(DATA_DEPTH)) u_ada_mem (
    .clk(clk), .we(ada_mem_wr_o), .re(ada_mem_rd_o),
    .addr(ada_mem_addr_o[15:0]), .wdata(ada_mem_data_o), .rdata(ada_mem_data_i));

// ─── TB-side pool write port, mux'd over each bank's DUT write port ──────────
// Loads inputs into the pool via PORT connectivity (no hierarchical .mem[] backdoor
// -> no -access wrc). Driven only in the per-timestep gate window while the DUT is
// idle, so muxing the shared single write/addr port is safe.
reg        tb_drive = 1'b0;
reg [15:0] tb_m0_addr=0, tb_m1_addr=0, tb_m2_addr=0, tb_m3_addr=0;
reg [31:0] tb_m0_wd=0,   tb_m1_wd=0,   tb_m2_wd=0,   tb_m3_wd=0;
reg        tb_m0_we=0,   tb_m1_we=0,   tb_m2_we=0,   tb_m3_we=0;
wire        m0_we_x = tb_drive ? tb_m0_we : m0_wr;
wire        m1_we_x = tb_drive ? tb_m1_we : m1_wr;
wire        m2_we_x = tb_drive ? tb_m2_we : m2_wr;
wire        m3_we_x = tb_drive ? tb_m3_we : m3_wr;
wire [15:0] m0_addr_x = tb_drive ? tb_m0_addr : m0_addr[15:0];
wire [15:0] m1_addr_x = tb_drive ? tb_m1_addr : m1_addr[15:0];
wire [15:0] m2_addr_x = tb_drive ? tb_m2_addr : m2_addr[15:0];
wire [15:0] m3_addr_x = tb_drive ? tb_m3_addr : m3_addr[15:0];
wire [31:0] m0_wd_x = tb_drive ? tb_m0_wd : m0_wdata;
wire [31:0] m1_wd_x = tb_drive ? tb_m1_wd : m1_wdata;
wire [31:0] m2_wd_x = tb_drive ? tb_m2_wd : m2_wdata;
wire [31:0] m3_wd_x = tb_drive ? tb_m3_wd : m3_wdata;

sram_model #(.DATA_W(32), .DEPTH(DATA_DEPTH)) u_m0 (
    .clk(clk), .we(m0_we_x), .re(m0_rd), .addr(m0_addr_x), .wdata(m0_wd_x), .rdata(m0_rdata));
sram_model #(.DATA_W(32), .DEPTH(DATA_DEPTH)) u_m1 (
    .clk(clk), .we(m1_we_x), .re(m1_rd), .addr(m1_addr_x), .wdata(m1_wd_x), .rdata(m1_rdata));
sram_model #(.DATA_W(32), .DEPTH(DATA_DEPTH)) u_m2 (
    .clk(clk), .we(m2_we_x), .re(m2_rd), .addr(m2_addr_x), .wdata(m2_wd_x), .rdata(m2_rdata));
sram_model #(.DATA_W(32), .DEPTH(DATA_DEPTH)) u_m3 (
    .clk(clk), .we(m3_we_x), .re(m3_rd), .addr(m3_addr_x), .wdata(m3_wd_x), .rdata(m3_rdata));

// ─── DUT ─────────────────────────────────────────────────────────────────────
fmi_top u_dut (
    .clk(clk), .reset(reset), .test_stall_pipe(test_stall_pipe),
    .sys_req_i(sys_req_i), .sys_ack_o(sys_ack_o),
    .sys_addr_i(sys_addr_i), .sys_data_i(sys_data_i), .sys_data_o(sys_data_o),
    .prog_mem_addr_o(prog_mem_addr_o), .prog_mem_data_i(prog_mem_data_i),
    .prog_mem_req_o(prog_mem_req_o), .prog_mem_wait_i(1'b0),
    .prog_mem_wr_o(prog_mem_wr_o), .prog_mem_wr_addr_o(prog_mem_wr_addr_o),
    .prog_mem_wr_data_o(prog_mem_wr_data_o), .prog_mem_wr_wait_i(1'b0),
    .cfg_mem_rd_o(cfg_mem_rd_o), .cfg_mem_wait_i(1'b0), .cfg_mem_addr_o(cfg_mem_addr_o),
    .cfg_mem_data_i(cfg_mem_data_i), .cfg_mem_wr_o(cfg_mem_wr_o),
    .cfg_mem_wr_addr_o(cfg_mem_wr_addr_o), .cfg_mem_wr_data_o(cfg_mem_wr_data_o),
    .bba_mem_rd_o(bba_mem_rd_o), .bba_mem_wait_i(1'b0), .bba_mem_addr_o(bba_mem_addr_o),
    .bba_mem_data_i(bba_mem_data_i), .bba_mem_wr_o(bba_mem_wr_o),
    .bba_mem_wr_addr_o(bba_mem_wr_addr_o), .bba_mem_wr_data_o(bba_mem_wr_data_o),
    .nxt_input_pulse_o(nxt_input_pulse_o), .nxt_output_pulse_o(nxt_output_pulse_o),
    .cm_config_finished_o(cm_config_finished_o),
    .weight_mem_rd_o(weight_mem_rd_o), .weight_mem_wait_i(1'b0),
    .weight_mem_addr_o(weight_mem_addr_o), .weight_mem_data_i(weight_mem_data_i),
    .thresh_mem_rd_o(thresh_mem_rd_o), .thresh_mem_wait_i(1'b0),
    .thresh_mem_addr_o(thresh_mem_addr_o), .thresh_mem_data_i(thresh_mem_data_i),
    .dcy_syn_mem_rd_o(dcy_syn_mem_rd_o), .dcy_syn_mem_wait_i(1'b0),
    .dcy_syn_mem_addr_o(dcy_syn_mem_addr_o), .dcy_syn_mem_data_i(dcy_syn_mem_data_i),
    .dcy_mem_mem_rd_o(dcy_mem_mem_rd_o), .dcy_mem_mem_wait_i(1'b0),
    .dcy_mem_mem_addr_o(dcy_mem_mem_addr_o), .dcy_mem_mem_data_i(dcy_mem_mem_data_i),
    .b_eff_mem_rd_o(b_eff_mem_rd_o), .b_eff_mem_wait_i(1'b0),
    .b_eff_mem_addr_o(b_eff_mem_addr_o), .b_eff_mem_data_i(b_eff_mem_data_i),
    .dcy_ada_mem_rd_o(dcy_ada_mem_rd_o), .dcy_ada_mem_wait_i(1'b0),
    .dcy_ada_mem_addr_o(dcy_ada_mem_addr_o), .dcy_ada_mem_data_i(dcy_ada_mem_data_i),
    .scl_ada_mem_rd_o(scl_ada_mem_rd_o), .scl_ada_mem_wait_i(1'b0),
    .scl_ada_mem_addr_o(scl_ada_mem_addr_o), .scl_ada_mem_data_i(scl_ada_mem_data_i),
    .pot_mem_wr_o(pot_mem_wr_o), .pot_mem_rd_o(pot_mem_rd_o), .pot_mem_wait_i(1'b0),
    .pot_mem_addr_o(pot_mem_addr_o), .pot_mem_data_o(pot_mem_data_o), .pot_mem_data_i(pot_mem_data_i),
    .ada_mem_wr_o(ada_mem_wr_o), .ada_mem_rd_o(ada_mem_rd_o), .ada_mem_wait_i(1'b0),
    .ada_mem_addr_o(ada_mem_addr_o), .ada_mem_data_o(ada_mem_data_o), .ada_mem_data_i(ada_mem_data_i),
    .m0_data_mem_rd_o(m0_rd), .m0_data_mem_wr_o(m0_wr), .m0_data_mem_wait_i(1'b0),
    .m0_data_mem_addr_o(m0_addr), .m0_data_mem_wdata_o(m0_wdata), .m0_data_mem_rdata_i(m0_rdata),
    .m1_data_mem_rd_o(m1_rd), .m1_data_mem_wr_o(m1_wr), .m1_data_mem_wait_i(1'b0),
    .m1_data_mem_addr_o(m1_addr), .m1_data_mem_wdata_o(m1_wdata), .m1_data_mem_rdata_i(m1_rdata),
    .m2_data_mem_rd_o(m2_rd), .m2_data_mem_wr_o(m2_wr), .m2_data_mem_wait_i(1'b0),
    .m2_data_mem_addr_o(m2_addr), .m2_data_mem_wdata_o(m2_wdata), .m2_data_mem_rdata_i(m2_rdata),
    .m3_data_mem_rd_o(m3_rd), .m3_data_mem_wr_o(m3_wr), .m3_data_mem_wait_i(1'b0),
    .m3_data_mem_addr_o(m3_addr), .m3_data_mem_wdata_o(m3_wdata), .m3_data_mem_rdata_i(m3_rdata)
);

// ─── AXI host write / read ───────────────────────────────────────────────────
task axi_write;
    input [31:0] addr;
    input [31:0] data;
    begin
        @(posedge clk); #1;
        sys_req_i = 1'b1; sys_addr_i = addr; sys_data_i = data;
        @(posedge clk); #1;
        sys_req_i = 1'b0; sys_addr_i = 32'h0; sys_data_i = 32'h0;
    end
endtask
task axi_read;
    input  [31:0] addr;
    output [31:0] data;
    integer guard;
    begin
        @(posedge clk); #1;
        sys_req_i = 1'b1; sys_addr_i = addr; sys_data_i = 32'h0;
        guard = 0;
        @(posedge clk); #1;
        while (sys_ack_o !== 1'b1 && guard < 1000) begin @(posedge clk); #1; guard = guard + 1; end
        data = sys_data_o;
        sys_req_i = 1'b0; sys_addr_i = 32'h0;
    end
endtask

// Read pot6 from the dedicated pot_mem @ G6 via the tb-side read-port mux.
task read_pot6;
    output [31:0] data;
    begin
        tb_pot_rd = 1'b1;
        @(posedge clk); #1;        // pot_mem registers rdata <= mem[G6]
        data = pot_mem_data_i;
        tb_pot_rd = 1'b0;
    end
endtask

// ─── NXT output-pulse counter (one pulse per timestep, after all 6 dispatches) ─
reg [31:0] nxt_out_count = 32'd0;
always @(posedge clk) if (nxt_output_pulse_o) nxt_out_count <= nxt_out_count + 1;

// ─── Golden streams (one block per timestep, concatenated) ───────────────────
reg [31:0] c2c_input_stream [0:T_C2C*240-1];   // cin=2 act: x(120) then one(120)
reg [31:0] c2c_spike3_golden[0:T_C2C*60-1];
reg [31:0] c2c_spike4_golden[0:T_C2C*60-1];
reg [31:0] c2c_spike5_golden[0:T_C2C*60-1];
reg [31:0] c2c_pot6_golden   [0:T_C2C-1];

integer i;

// ─── Static (read-only) model memories ───────────────────────────────────────
// Six weight sets packed contiguously (additive base) + each group's params at
// its slice. dcy_syn @ G6 is left at init-zero so con6's syn is fresh each step.
task load_static_mems;
    begin
        // weights (con1 cin=2 [w,b]; con2..con6 the per-layer hex)
        $readmemh("../../fmi/mem_files/recurrent/test_inputs/c1/c1_weight_cin2.hex", u_weight_mem.mem, W_CON1);
        $readmemh("../../fmi/mem_files/recurrent/con2_weight.hex", u_weight_mem.mem, W_CON2);
        $readmemh("../../fmi/mem_files/recurrent/con3_weight.hex", u_weight_mem.mem, W_CON3);
        $readmemh("../../fmi/mem_files/recurrent/con4_weight.hex", u_weight_mem.mem, W_CON4);
        $readmemh("../../fmi/mem_files/recurrent/con5_weight.hex", u_weight_mem.mem, W_CON5);
        $readmemh("../../fmi/mem_files/recurrent/con6_weight.hex", u_weight_mem.mem, W_CON6);
        // group3 adaptive-LIF full param set @ G3
        $readmemh("../../fmi/mem_files/recurrent/group3_dcy_syn.hex", u_dcy_syn_mem.mem, G3);
        $readmemh("../../fmi/mem_files/recurrent/group3_dcy_mem.hex", u_dcy_mem_mem.mem, G3);
        $readmemh("../../fmi/mem_files/recurrent/group3_dcy_ada.hex", u_dcy_ada_mem.mem, G3);
        $readmemh("../../fmi/mem_files/recurrent/group3_scl_ada.hex", u_scl_ada_mem.mem, G3);
        $readmemh("../../fmi/mem_files/recurrent/group3_b_eff.hex",   u_b_eff_mem.mem,   G3);
        $readmemh("../../fmi/mem_files/recurrent/group3_thresh.hex",  u_thresh_mem.mem,  G3);
        // group4 LIF params @ G4
        $readmemh("../../fmi/mem_files/recurrent/group4_dcy_syn.hex", u_dcy_syn_mem.mem, G4);
        $readmemh("../../fmi/mem_files/recurrent/group4_dcy_mem.hex", u_dcy_mem_mem.mem, G4);
        $readmemh("../../fmi/mem_files/recurrent/group4_thresh.hex",  u_thresh_mem.mem,  G4);
        // group5 LIF params @ G5
        $readmemh("../../fmi/mem_files/recurrent/group5_dcy_syn.hex", u_dcy_syn_mem.mem, G5);
        $readmemh("../../fmi/mem_files/recurrent/group5_dcy_mem.hex", u_dcy_mem_mem.mem, G5);
        $readmemh("../../fmi/mem_files/recurrent/group5_thresh.hex",  u_thresh_mem.mem,  G5);
        // group6 readout decay @ G6 (dcy_syn @ G6 stays zero -> syn fresh)
        $readmemh("../../fmi/mem_files/recurrent/group6_dcy_mem.hex", u_dcy_mem_mem.mem, G6);
    end
endtask

// ─── Six packed 16-word configs ──────────────────────────────────────────────
// Per-config: kernel_len/offset in S1[7:0]; stride in S2[12:10]; tt in S2[0].
task load_config;
    begin
        // ── cfg0 : con1 (g3) cin=2 real-MAC + adaptive-LIF -> spike3 ──────────
        cfg_mem[ 0] = ACT_BASE;       cfg_mem[ 1] = W_CON1;       cfg_mem[ 2] = SYN3_BASE;
        cfg_mem[ 3] = G3;             cfg_mem[ 4] = G3;           cfg_mem[ 5] = SPIKE3_BASE;
        cfg_mem[ 6] = G3;             cfg_mem[ 7] = G3;           cfg_mem[ 8] = G3;
        cfg_mem[ 9] = G3;             cfg_mem[10] = G3;           cfg_mem[11] = G3;
        cfg_mem[12] = {16'd120, 16'd120};  // S0: out_x=120 | in_x=120
        cfg_mem[13] = {16'd1919, 16'h0001};   // S1: last_neuron=1919 | kernel=1,off=0
        cfg_mem[14] = 32'h1002_0401;       // S2: cout=16 | cin=2  | stride=1 | tt=1
        cfg_mem[15] = 32'hE555_0040;       // M0: conv, adaptive-LIF, real_mac=1

        // ── cfg1 : con2 (g4) conv-only accumulate (skip_neuron=1) -> syn4 ─────
        cfg_mem[16] = SPIKE3_BASE;    cfg_mem[17] = W_CON2;       cfg_mem[18] = SYN4_BASE;
        cfg_mem[19] = G4;             cfg_mem[20] = G4;           cfg_mem[21] = DUMMY_BASE;
        cfg_mem[22] = G4;             cfg_mem[23] = G4;           cfg_mem[24] = G4;
        cfg_mem[25] = G4;             cfg_mem[26] = G4;           cfg_mem[27] = G4;
        cfg_mem[28] = {16'd60,  16'd120};  // S0: out_x=60  | in_x=120
        cfg_mem[29] = {16'd1919, 16'h0045};   // S1: last_neuron=1919 | kernel=5,off=2
        cfg_mem[30] = 32'h2010_0801;       // S2: cout=32 | cin=16 | stride=2 | tt=1
        cfg_mem[31] = 32'h2555_0041;       // M0: conv, LIF, skip_neuron=1

        // ── cfg2 : con3 (g4) recurrent LIF, read-before-write spike4 ─────────
        cfg_mem[32] = SPIKE4_BASE;    cfg_mem[33] = W_CON3;       cfg_mem[34] = SYN4_BASE;
        cfg_mem[35] = G4;             cfg_mem[36] = G4;           cfg_mem[37] = SPIKE4_BASE;
        cfg_mem[38] = G4;             cfg_mem[39] = G4;           cfg_mem[40] = G4;
        cfg_mem[41] = G4;             cfg_mem[42] = G4;           cfg_mem[43] = G4;
        cfg_mem[44] = {16'd60,  16'd60};   // S0: out_x=60  | in_x=60
        cfg_mem[45] = {16'd1919, 16'h0045};   // S1: last_neuron=1919 | kernel=5,off=2
        cfg_mem[46] = 32'h2020_0401;       // S2: cout=32 | cin=32 | stride=1 | tt=1
        cfg_mem[47] = 32'h2555_0040;       // M0: conv, LIF, skip_neuron=0

        // ── cfg3 : con4 (g5) conv-only accumulate (skip_neuron=1) -> syn5 ─────
        cfg_mem[48] = SPIKE4_BASE;    cfg_mem[49] = W_CON4;       cfg_mem[50] = SYN5_BASE;
        cfg_mem[51] = G5;             cfg_mem[52] = G5;           cfg_mem[53] = DUMMY_BASE;
        cfg_mem[54] = G5;             cfg_mem[55] = G5;           cfg_mem[56] = G5;
        cfg_mem[57] = G5;             cfg_mem[58] = G5;           cfg_mem[59] = G5;
        cfg_mem[60] = {16'd30,  16'd60};   // S0: out_x=30  | in_x=60
        cfg_mem[61] = {16'd1919, 16'h0045};   // S1: last_neuron=1919 | kernel=5,off=2
        cfg_mem[62] = 32'h4020_0801;       // S2: cout=64 | cin=32 | stride=2 | tt=1
        cfg_mem[63] = 32'h2555_0041;       // M0: conv, LIF, skip_neuron=1

        // ── cfg4 : con5 (g5) recurrent LIF, read-before-write spike5 ─────────
        cfg_mem[64] = SPIKE5_BASE;    cfg_mem[65] = W_CON5;       cfg_mem[66] = SYN5_BASE;
        cfg_mem[67] = G5;             cfg_mem[68] = G5;           cfg_mem[69] = SPIKE5_BASE;
        cfg_mem[70] = G5;             cfg_mem[71] = G5;           cfg_mem[72] = G5;
        cfg_mem[73] = G5;             cfg_mem[74] = G5;           cfg_mem[75] = G5;
        cfg_mem[76] = {16'd30,  16'd30};   // S0: out_x=30  | in_x=30
        cfg_mem[77] = {16'd1919, 16'h0045};   // S1: last_neuron=1919 | kernel=5,off=2
        cfg_mem[78] = 32'h4040_0401;       // S2: cout=64 | cin=64 | stride=1 | tt=1
        cfg_mem[79] = 32'h2555_0040;       // M0: conv, LIF, skip_neuron=0

        // ── cfg5 : con6 (g6) conv-FC + readout neuron -> pot6 ────────────────
        cfg_mem[80] = SPIKE5_BASE;    cfg_mem[81] = W_CON6;       cfg_mem[82] = SYN6_BASE;
        cfg_mem[83] = G6;             cfg_mem[84] = G6;           cfg_mem[85] = DUMMY_BASE;
        cfg_mem[86] = G6;             cfg_mem[87] = G6;           cfg_mem[88] = G6;
        cfg_mem[89] = G6;             cfg_mem[90] = G6;           cfg_mem[91] = G6;
        cfg_mem[92] = {16'd1,   16'd30};   // S0: out_x=1   | in_x=30
        cfg_mem[93] = {16'd0,   16'h001E};   // S1: last_neuron=0 | kernel=30,off=0
        cfg_mem[94] = 32'h0140_0401;       // S2: cout=1  | cin=64 | stride=1 | tt=1
        cfg_mem[95] = 32'h2555_0048;       // M0: conv, readout neuron (np_mode[1])

        for (i = 0; i < 8; i = i + 1) bba_mem[i] = 32'd0;
    end
endtask

// ─── Boot-only conv params (outside the 16-word window) via DIRECT host AXI ──
task write_boot_regs;
    begin
        axi_write(FMI_CFG_BASE | 32'h5C, 32'd15);  // weight_idx_sz (con5=64*64*5=20480 < 2^15)
        axi_write(FMI_CFG_BASE | 32'h98, 32'd14);  // mac_shift = K_MEM (con1 real-MAC product shift)
        // kernel_len/offset are per-config in S1[7:0]; stride per-config in S2[12:10].
    end
endtask

// ─── Drive this timestep's cin=2 activation (x then one) into the ACT region ──
// ACT_BASE is bank-aligned, so logical word (4g+b) lands in bank b at word g.
task load_input_frame;
    input integer t;
    integer g;
    begin
        tb_drive = 1'b1;
        for (g = 0; g < 60; g = g + 1) begin
            tb_m0_addr = (ACT_BASE>>2)+g; tb_m0_wd = c2c_input_stream[t*240 + 4*g + 0]; tb_m0_we = 1'b1;
            tb_m1_addr = (ACT_BASE>>2)+g; tb_m1_wd = c2c_input_stream[t*240 + 4*g + 1]; tb_m1_we = 1'b1;
            tb_m2_addr = (ACT_BASE>>2)+g; tb_m2_wd = c2c_input_stream[t*240 + 4*g + 2]; tb_m2_we = 1'b1;
            tb_m3_addr = (ACT_BASE>>2)+g; tb_m3_wd = c2c_input_stream[t*240 + 4*g + 3]; tb_m3_we = 1'b1;
            @(posedge clk); #1;
        end
        tb_m0_we = 1'b0; tb_m1_we = 1'b0; tb_m2_we = 1'b0; tb_m3_we = 1'b0;
        tb_drive = 1'b0;
    end
endtask

// ─── Compare this timestep's spike3 + spike4 + spike5 (AXI) + pot6 (mux) ─────
task check_timestep;
    input integer t;
    integer w, s3m, s4m, s5m;
    reg [31:0] got, pot;
    begin
        s3m = 0; s4m = 0; s5m = 0;
        for (w = 0; w < 60; w = w + 1) begin
            axi_read(POOL_RD_BASE + ((SPIKE3_BASE + w) << 2), got);
            if (got !== c2c_spike3_golden[t*60 + w]) begin
                if (s3m < 4) $display("    t=%0d spike3 MISMATCH w=%0d got %08h exp %08h",
                                       t, w, got, c2c_spike3_golden[t*60 + w]);
                s3m = s3m + 1;
            end
            axi_read(POOL_RD_BASE + ((SPIKE4_BASE + w) << 2), got);
            if (got !== c2c_spike4_golden[t*60 + w]) begin
                if (s4m < 4) $display("    t=%0d spike4 MISMATCH w=%0d got %08h exp %08h",
                                       t, w, got, c2c_spike4_golden[t*60 + w]);
                s4m = s4m + 1;
            end
            axi_read(POOL_RD_BASE + ((SPIKE5_BASE + w) << 2), got);
            if (got !== c2c_spike5_golden[t*60 + w]) begin
                if (s5m < 4) $display("    t=%0d spike5 MISMATCH w=%0d got %08h exp %08h",
                                       t, w, got, c2c_spike5_golden[t*60 + w]);
                s5m = s5m + 1;
            end
        end
        read_pot6(pot);
        if (s3m == 0 && s4m == 0 && s5m == 0 && pot === c2c_pot6_golden[t]) begin
            if (t % 25 == 0 || t == T_C2C-1)
                $display("  OK  t=%0d  spike3/4/5 60/60 + pot6 %08h match", t, pot);
        end else begin
            if (pot !== c2c_pot6_golden[t])
                $display("    t=%0d pot6 MISMATCH got %08h exp %08h", t, pot, c2c_pot6_golden[t]);
            $display("  FAIL t=%0d  s3 %0d / s4 %0d / s5 %0d mismatches%s",
                     t, s3m, s4m, s5m, (pot !== c2c_pot6_golden[t]) ? " + pot6" : "");
            errors = errors + 1;
        end
    end
endtask

// ─── Main ────────────────────────────────────────────────────────────────────
integer t, guard;
initial begin : run
    repeat(4) @(posedge clk);

    // fresh DUT state (carried state relies on sram_model init-zero, no backdoor clear)
    reset = 1'b1; repeat(4) @(posedge clk); #1;
    reset = 1'b0; @(posedge clk); #1;

    // memories + goldens
    load_static_mems;
    $readmemh("../../fmi/mem_files/recurrent/test_inputs/c2c/c2c_input_stream.hex", c2c_input_stream);
    $readmemh("../../fmi/mem_files/recurrent/test_inputs/c2c/c2c_spike3_golden.hex", c2c_spike3_golden);
    $readmemh("../../fmi/mem_files/recurrent/test_inputs/c2c/c2c_spike4_golden.hex", c2c_spike4_golden);
    $readmemh("../../fmi/mem_files/recurrent/test_inputs/c2c/c2c_spike5_golden.hex", c2c_spike5_golden);
    $readmemh("../../fmi/mem_files/recurrent/test_inputs/c2c/c2c_pot6_golden.hex",   c2c_pot6_golden);
    load_input_frame(0);                       // pre-load the cin=2 act for t=0 (tb write port)

    // config + boot regs
    load_config;
    write_boot_regs;

    // ── Buffer seeding (data = {usage[7:4], id[3:0]}) ────────────────────────
    //   buf0 act    : full cnt=1 (re-marked each timestep)
    //   buf1 spike3 : leave FREE (con1 TARGET needs free)
    //   buf2 spike4 : full cnt=2 (recurrent: survives con4's one consume)
    //   buf3 spike5 : full cnt=2 (same)
    //   buf4 dummy  : full cnt=1 (con6 RW self-cycle, no consumer)
    axi_write(SCH_MARKFULL, 32'h0000_0010);    // {cnt=1, id=0}
    axi_write(SCH_MARKFULL, 32'h0000_0022);    // {cnt=2, id=2}
    axi_write(SCH_MARKFULL, 32'h0000_0023);    // {cnt=2, id=3}
    axi_write(SCH_MARKFULL, 32'h0000_0014);    // {cnt=1, id=4}

    // ── Program: LOOP id0 count=T-1 ; six TASKs ; NXT in+out ; LOOPEND ; STOP ─
    prog_mem[ 0] = ((T_C2C-1) << 6) | 32'h6;   // LOOP id0, count=T-1
    prog_mem[ 1] = 32'h0000_2000;              // con1 w1: SOURCE buf0 (act), cfg0
    prog_mem[ 2] = 32'h11C0_0000;              // con1 w2: TARGET buf1 (spike3) #t=1
    prog_mem[ 3] = 32'h0000_A020;              // con2 w1: SOURCE buf1 (spike3), cfg1
    prog_mem[ 4] = 32'h0000_0000;              // con2 w2: conv-only (no output)
    prog_mem[ 5] = 32'h0000_0040;              // con3 w1: cfg2 (act from cfg act_base)
    prog_mem[ 6] = 32'h2280_0000;              // con3 w2: RW buf2 (spike4) #t=2
    prog_mem[ 7] = 32'h0001_2060;              // con4 w1: SOURCE buf2 (spike4), cfg3
    prog_mem[ 8] = 32'h0000_0000;              // con4 w2: conv-only (no output)
    prog_mem[ 9] = 32'h0000_0080;              // con5 w1: cfg4 (act from cfg act_base)
    prog_mem[10] = 32'h2380_0000;              // con5 w2: RW buf3 (spike5) #t=2
    prog_mem[11] = 32'h0001_A0A0;              // con6 w1: SOURCE buf3 (spike5), cfg5
    prog_mem[12] = 32'h1480_0000;              // con6 w2: RW buf4 (dummy spike) #t=1
    prog_mem[13] = 32'h0000_0034;              // NXT in+out
    prog_mem[14] = 32'h0000_0007;              // LOOPEND id0
    prog_mem[15] = STOP_INST;

    axi_write(SCH_LOAD_PC, 32'd0);
    axi_write(SCH_START,   32'd0);

    $display("[C2c] FULL g3->g6 on fmi_top (6 dispatches/timestep, AXI readback, no wrc), T=%0d", T_C2C);

    // Per-timestep lock-step: wait for this timestep's NXT, settle, check all four
    // streams, then re-feed the input + re-mark buf0 to release the next iteration.
    for (t = 0; t < T_C2C; t = t + 1) begin
        guard = 0;
        while (nxt_out_count != (t+1) && guard < 8_000_000) begin
            @(posedge clk); guard = guard + 1;
        end
        if (nxt_out_count != (t+1)) begin
            $display("  FAIL t=%0d: no NXT pulse (dispatch stalled / timeout)", t);
            errors = errors + 1;
            disable run;
        end
        repeat(SETTLE) @(posedge clk); #1;
        check_timestep(t);
        if (t + 1 < T_C2C) begin
            load_input_frame(t + 1);                 // cin=2 act for t+1 into ACT (tb write port)
            axi_write(SCH_MARKFULL, 32'h0000_0010);  // re-mark buf0 full {cnt=1,id=0}
        end
    end

    $display("=== tb_fmi_top_c2c: %0d failure(s) ===", errors);
    if (errors == 0) $display("PASS"); else $display("FAIL");
    #20 $finish;
end

// safety net — cycle-counted (a single #delay literal would overflow 32 bits).
initial begin
    repeat (T_C2C + 2) repeat (8_000_000) @(posedge clk);
    $display("FAIL: global timeout");
    $finish;
end

endmodule // tb_fmi_top_c2c

// ─────────────────────────────────────────────────────────────────────────────
// Simple synchronous 1-cycle-read SRAM model (matches the fmiSnnMC block tb).
module sram_model #(
    parameter DATA_W = 32,
    parameter DEPTH  = 256
)(
    input  wire              clk,
    input  wire              we,
    input  wire              re,
    input  wire       [15:0] addr,
    input  wire [DATA_W-1:0] wdata,
    output reg  [DATA_W-1:0] rdata
);
    reg [DATA_W-1:0] mem [0:DEPTH-1];
    integer k;
    initial begin
        for (k = 0; k < DEPTH; k = k + 1) mem[k] = {DATA_W{1'b0}};
        rdata = {DATA_W{1'b0}};
    end
    always @(posedge clk) begin
        if (we) mem[addr] <= wdata;
        if (re) rdata     <= mem[addr];
    end
endmodule
