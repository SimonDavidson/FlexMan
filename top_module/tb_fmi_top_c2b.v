// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
`timescale 10ps/1ps
`include "../shared/constants.v"

// =============================================================================
// tb_fmi_top_c2b.v — Stage C2b integration testbench for fmi_top.v
// Authors: Simon Davidson & Claude   Created: 2026-06-27   Last modified: 2026-06-29
//
// group4 con2 feed-forward + con3 recurrence, TWO dispatches/timestep through the
// scheduler hardware loop (see git history for the full datapath rationale):
//   A: con2 (cfg0, skip_neuron=1, conv-only accumulate, no decay)
//   B: con3 (cfg1, skip_neuron=0) reads spike4[t-1] then neuron writes spike4[t]
//      to the SAME RW buffer (read-before-write; acc_fmiSnnMC_processor.v:554).
// Per-config stride (S2[12:10]) lets con2 (stride 2) and con3 (stride 1) coexist.
//
// === AXI-readback build: NO -access wrc ===
// This tb uses ZERO hierarchical (.mem[]) backdoor references, so it compiles
// WITHOUT Xcelium's -access wrc (which disables optimisation and dominates long
// runs — T=249 under wrc took hours). How each access avoids the backdoor:
//   - result compare  : AXI reads through the host POOL_RD_BASE window (real DUT
//                        read port; the path C1 validated against the backdoor).
//   - input load       : a tb-side write port MUX'd over each pool bank's DUT
//                        write port (port connectivity, not a hierarchical ref);
//                        driven only in the per-timestep gate window (DUT idle).
//   - dedicated mems   : $readmemh into the tb sram_model instances (confirmed
//                        NOT to require -access wrc).
//   - state zeroing    : relies on sram_model's init-zero (no clear backdoor).
// =============================================================================
module tb_fmi_top_c2b;

// ─── C2b test size (bump to 249 for the full pass) ───────────────────────────
localparam integer T_C2B    = 16;
localparam integer SETTLE   = 16;
localparam integer WEIGHT_DEPTH = 8192;   // con2 @0 (2560) + con3 @2560 (5120) = 7680

// ─── Fabric params ───────────────────────────────────────────────────────────
localparam PROG_DATA_BITS = 32;
localparam CFG_DEPTH     = 256;
localparam CFG_ADDR_BITS = 8;
localparam PROG_DEPTH    = 64;
localparam PROG_IDX_BITS = 6;
localparam DATA_DEPTH    = 4096;

// ─── Address map ─────────────────────────────────────────────────────────────
localparam [31:0] FMI_CFG_BASE = 32'h1000_0000;
localparam [31:0] POOL_RD_BASE = 32'h1010_0000;
localparam [31:0] SCH_LOAD_PC  = 32'hE000_0000;
localparam [31:0] SCH_START    = 32'hE010_0000;
localparam [31:0] SCH_MARKFULL = 32'hE050_0000;
localparam STOP_INST = 32'h0000_0002;

// ─── Pool buffer layout (same as C1/C2a) ─────────────────────────────────────
localparam ACT_BASE      = 0;     // spike3 input  -> [0   .. 59]
localparam SPIKE_BASE    = 64;    // spike4 (con3 input t-1 AND output t) -> [64 .. 123]
localparam SYN_CURR_BASE = 128;   // syn4          -> [128 .. 2047]

// ─── con weight memory layout ────────────────────────────────────────────────
localparam CON3_WEIGHT_BASE = 2560;   // con2 = 32*16*5 = 2560 words at base 0

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
    .clk(clk), .we(pot_mem_wr_o), .re(pot_mem_rd_o),
    .addr(pot_mem_addr_o[15:0]), .wdata(pot_mem_data_o), .rdata(pot_mem_data_i));
sram_model #(.DATA_W(32), .DEPTH(DATA_DEPTH)) u_ada_mem (
    .clk(clk), .we(ada_mem_wr_o), .re(ada_mem_rd_o),
    .addr(ada_mem_addr_o[15:0]), .wdata(ada_mem_data_o), .rdata(ada_mem_data_i));

// ─── TB-side pool write port, mux'd over each bank's DUT write port ──────────
// Loads inputs into the pool via PORT connectivity (no hierarchical .mem[]
// backdoor → no -access wrc). Driven only in the per-timestep gate window while
// the DUT is idle, so muxing the shared single write/addr port is safe.
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

// ─── NXT output-pulse counter (one pulse per timestep, after dispatches A+B) ──
reg [31:0] nxt_out_count = 32'd0;
always @(posedge clk) if (nxt_output_pulse_o) nxt_out_count <= nxt_out_count + 1;

// ─── Golden streams (one block per timestep, concatenated) ───────────────────
reg [31:0] spike3_stream [0:T_C2B*60-1];
reg [31:0] spike4_golden [0:T_C2B*60-1];
reg [31:0] syn4_golden   [0:T_C2B*1920-1];

integer i;

// Static group4 model mems: con2 @0, con3 @CON3_WEIGHT_BASE, + group4 decay/thresh.
task load_static_mems;
    begin
        $readmemh("../../fmi/mem_files/recurrent/con2_weight.hex", u_weight_mem.mem);
        $readmemh("../../fmi/mem_files/recurrent/con3_weight.hex", u_weight_mem.mem, CON3_WEIGHT_BASE);
        $readmemh("../../fmi/mem_files/recurrent/group4_dcy_syn.hex", u_dcy_syn_mem.mem);
        $readmemh("../../fmi/mem_files/recurrent/group4_dcy_mem.hex", u_dcy_mem_mem.mem);
        $readmemh("../../fmi/mem_files/recurrent/group4_thresh.hex",  u_thresh_mem.mem);
    end
endtask

// Two PACKED configs: cfg0 = con2 (A, skip_neuron=1), cfg1 = con3 (B, neuron).
task load_config;
    begin
        // ---- cfg_id 0 : con2 feed-forward (dispatch A) ----
        cfg_mem[ 0] = ACT_BASE;        cfg_mem[ 1] = 32'd0;           cfg_mem[ 2] = SYN_CURR_BASE;
        cfg_mem[ 3] = 32'd0;           cfg_mem[ 4] = 32'd0;           cfg_mem[ 5] = SPIKE_BASE;
        cfg_mem[ 6] = 32'd0;           cfg_mem[ 7] = 32'd0;           cfg_mem[ 8] = 32'd0;
        cfg_mem[ 9] = 32'd0;           cfg_mem[10] = 32'd0;           cfg_mem[11] = 32'd0;
        cfg_mem[12] = {16'd60, 16'd120};   // S0: out_x=60 | in_x=120
        cfg_mem[13] = {16'd1919, 16'h0045};   // S1: last_neuron=1919 | kernel=5,offset=2 (0x45)
        cfg_mem[14] = 32'h2010_0801;       // S2: cout=32 | cin=16 | stride=2[12:10] | tt=1
        cfg_mem[15] = 32'h2555_0041;       // M0: conv, plain LIF, skip_neuron=1
        // ---- cfg_id 1 : con3 recurrent (dispatch B) ----
        cfg_mem[16] = SPIKE_BASE;          cfg_mem[17] = CON3_WEIGHT_BASE; cfg_mem[18] = SYN_CURR_BASE;
        cfg_mem[19] = 32'd0;           cfg_mem[20] = 32'd0;           cfg_mem[21] = SPIKE_BASE;
        cfg_mem[22] = 32'd0;           cfg_mem[23] = 32'd0;           cfg_mem[24] = 32'd0;
        cfg_mem[25] = 32'd0;           cfg_mem[26] = 32'd0;           cfg_mem[27] = 32'd0;
        cfg_mem[28] = {16'd60, 16'd60};    // S0: out_x=60 | in_x=60
        cfg_mem[29] = {16'd1919, 16'h0045};   // S1: last_neuron=1919 | kernel=5,offset=2 (0x45)
        cfg_mem[30] = 32'h2020_0401;       // S2: cout=32 | cin=32 | stride=1[12:10] | tt=1
        cfg_mem[31] = 32'h2555_0040;       // M0: conv, plain LIF, skip_neuron=0
        for (i = 0; i < 8; i = i + 1) bba_mem[i] = 32'd0;
    end
endtask

// Boot regs: weight_idx_sz=13 (kernel_len=5/offset=2 are per-config in S1[7:0]; stride in S2).
task write_boot_regs;
    begin
        axi_write(FMI_CFG_BASE | 32'h5C, 32'd13);  // weight_idx_sz
        // x_kernel_len=5 / offset=2 are now per-config in S1[7:0] (=0x45) for both
        // con2 (cfg_mem[13]) and con3 (cfg_mem[29]); x_kernel_step in S2[12:10]
    end
endtask

// Drive spike3[t] into the (fixed) ACT pool region via the tb-side write port.
// ACT_BASE is bank-aligned, so logical word (4g+b) lands in bank b at word g.
task load_input_frame;
    input integer t;
    integer g;
    begin
        tb_drive = 1'b1;
        for (g = 0; g < 15; g = g + 1) begin
            tb_m0_addr = (ACT_BASE>>2)+g; tb_m0_wd = spike3_stream[t*60 + 4*g + 0]; tb_m0_we = 1'b1;
            tb_m1_addr = (ACT_BASE>>2)+g; tb_m1_wd = spike3_stream[t*60 + 4*g + 1]; tb_m1_we = 1'b1;
            tb_m2_addr = (ACT_BASE>>2)+g; tb_m2_wd = spike3_stream[t*60 + 4*g + 2]; tb_m2_we = 1'b1;
            tb_m3_addr = (ACT_BASE>>2)+g; tb_m3_wd = spike3_stream[t*60 + 4*g + 3]; tb_m3_we = 1'b1;
            @(posedge clk); #1;
        end
        tb_m0_we = 1'b0; tb_m1_we = 1'b0; tb_m2_we = 1'b0; tb_m3_we = 1'b0;
        tb_drive = 1'b0;
    end
endtask

// Compare this timestep's spike4 (60) + syn4 (1920) to golden via AXI POOL_RD.
task check_timestep;
    input integer t;
    integer w, n, spk_mism, syn_mism;
    reg [31:0] got;
    begin
        spk_mism = 0; syn_mism = 0;
        for (w = 0; w < 60; w = w + 1) begin
            axi_read(POOL_RD_BASE + ((SPIKE_BASE + w) << 2), got);
            if (got !== spike4_golden[t*60 + w]) begin
                if (spk_mism < 4) $display("    t=%0d spike MISMATCH w=%0d got %08h exp %08h",
                                            t, w, got, spike4_golden[t*60 + w]);
                spk_mism = spk_mism + 1;
            end
        end
        for (n = 0; n < 1920; n = n + 1) begin
            axi_read(POOL_RD_BASE + ((SYN_CURR_BASE + n) << 2), got);
            if (got !== syn4_golden[t*1920 + n]) begin
                if (syn_mism < 4) $display("    t=%0d syn MISMATCH n=%0d got %08h exp %08h",
                                            t, n, got, syn4_golden[t*1920 + n]);
                syn_mism = syn_mism + 1;
            end
        end
        if (spk_mism == 0 && syn_mism == 0)
            $display("  OK  t=%0d  spike4 60/60 + syn4 1920/1920 match", t);
        else begin
            $display("  FAIL t=%0d  spike %0d / syn %0d mismatches", t, spk_mism, syn_mism);
            errors = errors + 1;
        end
    end
endtask

// ─── Main ────────────────────────────────────────────────────────────────────
integer t, guard;
initial begin : run
    repeat(4) @(posedge clk);
    reset = 1'b1; repeat(4) @(posedge clk); #1;
    reset = 1'b0; @(posedge clk); #1;

    // syn_curr / spike / pot / ada start at the sram_model init-zero (no backdoor clear).
    load_static_mems;
    $readmemh("../../fmi/mem_files/recurrent/test_inputs/c2b/c2b_spike3_stream.hex", spike3_stream);
    $readmemh("../../fmi/mem_files/recurrent/test_inputs/c2b/c2b_spike4_golden.hex", spike4_golden);
    $readmemh("../../fmi/mem_files/recurrent/test_inputs/c2b/c2b_syn4_golden.hex",   syn4_golden);
    load_input_frame(0);                       // pre-load spike3[0] via the tb write port

    load_config;
    write_boot_regs;

    axi_write(SCH_MARKFULL, 32'h0000_0010);    // buf0 (spike3 SRC) full {cnt=1,id=0}
    axi_write(SCH_MARKFULL, 32'h0000_0011);    // buf1 (spike4 RW)  full {cnt=1,id=1}

    prog_mem[0] = ((T_C2B-1) << 6) | 32'h6;    // LOOP id0, count=T-1
    prog_mem[1] = 32'h0000_2000;               // TASK_A w1 : acc0 cfg0 slot0=SRC buf0
    prog_mem[2] = 32'h0000_0000;               // TASK_A w2 : conv-only (no output)
    prog_mem[3] = 32'h0000_0020;               // TASK_B w1 : acc0 cfg1
    prog_mem[4] = 32'h1180_0000;               // TASK_B w2 : slot5=RW buf1 ntgt=1
    prog_mem[5] = 32'h0000_0034;               // NXT in+out
    prog_mem[6] = 32'h0000_0007;               // LOOPEND id0
    prog_mem[7] = STOP_INST;

    axi_write(SCH_LOAD_PC, 32'd0);
    axi_write(SCH_START,   32'd0);

    $display("[C2b] scheduler hardware-loop + con3 recurrence (AXI readback, no wrc), T=%0d", T_C2B);

    for (t = 0; t < T_C2B; t = t + 1) begin
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
        if (t + 1 < T_C2B) begin
            load_input_frame(t + 1);                 // spike3[t+1] into ACT (tb write port)
            axi_write(SCH_MARKFULL, 32'h0000_0010);  // re-mark buf0 full {cnt=1,id=0}
        end
    end

    $display("=== tb_fmi_top_c2b: %0d failure(s) ===", errors);
    if (errors == 0) $display("PASS"); else $display("FAIL");
    #20 $finish;
end

// safety net — cycle-counted (a single #delay literal would overflow 32 bits).
initial begin
    repeat (T_C2B + 2) repeat (8_000_000) @(posedge clk);
    $display("FAIL: global timeout");
    $finish;
end

endmodule // tb_fmi_top_c2b

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
