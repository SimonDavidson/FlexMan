// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
`timescale 10ps/1ps
`include "../shared/constants.v"

// =============================================================================
// tb_fmi_top_c6.v — con6 READOUT layer on fmi_top (scheduler-loop integration)
// Authors: Simon Davidson & Claude   Created: 2026-06-29   Last modified: 2026-06-29
//
// Runs the network's OUTPUT layer (con6: the dense 1920->1 leaky-integrator
// readout) through the full fmi_top fabric (scheduler / config_manager /
// shared_pool), one dispatch per timestep over T timesteps. This is the C2a
// scheduler-hardware-loop applied to the new fmiSnnAccMC READOUT-neuron mode:
//   - con6 runs as a CONV dispatch with a full-width kernel (cin=64, kernel=in_x
//     =30, out_x=1): the single output sums weight[cin,x]*spike5[cin,x] over all
//     1920 = inj6. (Native full mode is broken for wide-weight FC; the conv
//     datapath has a constant weight base -- see weight_generator.v.)
//   - the readout neuron (M0[3]=np_mode[1]) then integrates: pot6 = dcy_mem*pot6
//     + inj6, NO threshold/reset, output the wrapped pot. pot6 CARRIES; syn_curr
//     is fresh each step (group6 has no synaptic decay -> dcy_syn=0).
// New coverage vs the block T12: the SAME readout dispatch carried through the
// scheduler dispatch, config_manager 16-word push (incl. M0[3] readout), and the
// shared_pool routing of the spike5 input -- while the OUTPUT (pot6) lives in the
// dedicated pot_mem, HW-carried across timesteps. TB-only -- NO RTL changes
// (readout-neuron mode already landed, FlexMan 497c609).
//
// Buffer choreography (identical to C1/C2a, validated vs sch_entry.v:90):
//   - buf0  input act (spike5)     : SOURCE, re-fed + re-marked full each iter.
//   - buf1  output spike (=0)      : READ-WRITE, marked full once; self-cycles
//                                    (con6 never spikes; buf1 is the dummy RW
//                                    output the scheduler needs to gate dispatch).
//   - syn_curr (inj6)              : pool, config-addressed, HW-fresh (dcy_syn=0).
//   - pot6                         : dedicated pot_mem, config-addressed, CARRIED.
//
// Golden: fmi/gen_c6_golden.py. pot6 compared EVERY step (the dedicated pot_mem
// is a tb-side sram_model -> backdoor read; pot6 is NOT in the pool, so the AXI
// POOL_RD window cannot reach it -> -access wrc, like C1).
// =============================================================================
module tb_fmi_top_c6;

// ─── C6-layer test size (spike5 fires from ~t=13; bump for a longer window) ───
localparam integer T_C6    = 16;
localparam integer SETTLE  = 16;   // cycles to let the writeback pipeline drain

// ─── Fabric params (match fmi_top defaults) ──────────────────────────────────
localparam PROG_DATA_BITS = 32;

// ─── TB memory sizing ────────────────────────────────────────────────────────
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

// ─── Pool buffer layout (act = spike5 = 1920 bits = 60 words) ────────────────
localparam ACT_BASE      = 0;     // spike5 input        -> [0   .. 59]
localparam SPIKE_BASE    = 256;   // spike output (=0)   -> [256 .. 315]
localparam SYN_CURR_BASE = 512;   // syn_curr (inj6)     -> [512 .. 2431]

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

// pool bank nets
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

// ─── Dedicated acc memories (sram_model, 1-cycle) ────────────────────────────
sram_model #(.DATA_W(`WTD_BITS), .DEPTH(DATA_DEPTH)) u_weight_mem (
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

// ─── Shared pool banks (4 interleaved, sram_model 1-cycle) ───────────────────
sram_model #(.DATA_W(32), .DEPTH(DATA_DEPTH)) u_m0 (
    .clk(clk), .we(m0_wr), .re(m0_rd), .addr(m0_addr[15:0]), .wdata(m0_wdata), .rdata(m0_rdata));
sram_model #(.DATA_W(32), .DEPTH(DATA_DEPTH)) u_m1 (
    .clk(clk), .we(m1_wr), .re(m1_rd), .addr(m1_addr[15:0]), .wdata(m1_wdata), .rdata(m1_rdata));
sram_model #(.DATA_W(32), .DEPTH(DATA_DEPTH)) u_m2 (
    .clk(clk), .we(m2_wr), .re(m2_rd), .addr(m2_addr[15:0]), .wdata(m2_wdata), .rdata(m2_rdata));
sram_model #(.DATA_W(32), .DEPTH(DATA_DEPTH)) u_m3 (
    .clk(clk), .we(m3_wr), .re(m3_rd), .addr(m3_addr[15:0]), .wdata(m3_wdata), .rdata(m3_rdata));

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

// ─── Pool backdoor access: logical addr -> (bank=a[1:0], word=a>>2) ──────────
function [31:0] pool_rd;
    input integer a;
    case (a[1:0])
        2'd0: pool_rd = u_m0.mem[a >> 2];
        2'd1: pool_rd = u_m1.mem[a >> 2];
        2'd2: pool_rd = u_m2.mem[a >> 2];
        default: pool_rd = u_m3.mem[a >> 2];
    endcase
endfunction
task pool_wr;
    input integer a;
    input [31:0]  d;
    begin
        case (a[1:0])
            2'd0: u_m0.mem[a >> 2] = d;
            2'd1: u_m1.mem[a >> 2] = d;
            2'd2: u_m2.mem[a >> 2] = d;
            default: u_m3.mem[a >> 2] = d;
        endcase
    end
endtask

// ─── AXI host write ──────────────────────────────────────────────────────────
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

// ─── NXT output-pulse counter (robust per-timestep sync) ─────────────────────
reg [31:0] nxt_out_count = 32'd0;
always @(posedge clk) if (nxt_output_pulse_o) nxt_out_count <= nxt_out_count + 1;

// ─── Golden streams (one block per timestep, concatenated) ───────────────────
reg [31:0] c6_spike5_stream [0:T_C6*60-1];   // spike5 input (packed 1-bit, 60 words)
reg [31:0] c6_pot6_golden   [0:T_C6-1];      // pot6 readout, one scalar per timestep

integer i;

// Static (read-only) con6 model memories: the 1920 FC weights + g6_dmem.
task load_static_mems;
    begin
        $readmemh("../../fmi/mem_files/recurrent/con6_weight.hex",    u_weight_mem.mem);
        $readmemh("../../fmi/mem_files/recurrent/group6_dcy_mem.hex", u_dcy_mem_mem.mem); // g6_dmem at [0]
        // dcy_syn = 0 (group6 has no synaptic decay -> syn fresh); thresh / ada unused.
    end
endtask

// Program the 16-word PACKED config for cfg_id 0 (con6 conv-FC + readout neuron;
// total_timesteps=1 because the SCHEDULER provides T, one dispatch per timestep).
task load_config;
    begin
        cfg_mem[ 0] = ACT_BASE;        // 0x00 act_base   (pool, spike5)
        cfg_mem[ 1] = 32'd0;           // 0x04 weight_base(dedicated)
        cfg_mem[ 2] = SYN_CURR_BASE;   // 0x08 syn_curr_base (pool, inj6)
        cfg_mem[ 3] = 32'd0;           // 0x0C thresh_base(dedicated, unused)
        cfg_mem[ 4] = 32'd0;           // 0x10 pot_base   (dedicated, pot6 -> pot_mem[0])
        cfg_mem[ 5] = SPIKE_BASE;      // 0x14 spike_base (pool, =0)
        cfg_mem[ 6] = 32'd0;           // 0x18 dcy_syn_base (=0 -> syn fresh)
        cfg_mem[ 7] = 32'd0;           // 0x1C dcy_mem_base (g6_dmem)
        cfg_mem[ 8] = 32'd0;           // 0x20 ada_base
        cfg_mem[ 9] = 32'd0;           // 0x24 b_eff_base
        cfg_mem[10] = 32'd0;           // 0x28 dcy_ada_base
        cfg_mem[11] = 32'd0;           // 0x2C scl_ada_base
        cfg_mem[12] = {16'd1,  16'd30};    // 0x30 S0: out_x=1 | in_x=30
        cfg_mem[13] = {16'd0,  16'h001E};  // 0x34 S1: last_neuron=0 | kernel=30,offset=0 (0x1E)
        cfg_mem[14] = 32'h0140_0401;       // 0x38 S2: cout=1 | cin=64 | stride=1[12:10] | tt=1
        cfg_mem[15] = 32'h2555_0048;       // 0x3C M0: conv, readout neuron (np_mode[1]), skip=0
        for (i = 0; i < 8; i = i + 1) bba_mem[i] = 32'd0;
    end
endtask

// Boot-only conv params (outside the 16-word window) via DIRECT host AXI.
task write_boot_regs;
    begin
        axi_write(FMI_CFG_BASE | 32'h5C, 32'd11);  // weight_idx_sz (cout*cin*k=1*64*30=1920 < 2^11)
        // x_kernel_len=30 / offset=0 are now per-config in S1[7:0] (=0x1E) — see cfg_mem[13]
        // x_kernel_step (stride) is per-config in S2[12:10] = 1 (see cfg_mem[14])
    end
endtask

// Zero the carried state ONCE at t=0 (syn_curr, spike region, pot, ada).
task clear_state;
    begin
        for (i = 0; i < 1920; i = i + 1) pool_wr(SYN_CURR_BASE + i, 32'd0);
        for (i = 0; i < 60;   i = i + 1) pool_wr(SPIKE_BASE   + i, 32'd0);
        for (i = 0; i < DATA_DEPTH; i = i + 1) begin
            u_pot_mem.mem[i] = 32'd0;
            u_ada_mem.mem[i] = 32'd0;
        end
    end
endtask

// Drive this timestep's spike5 (60 packed words) into the (fixed) ACT region.
task load_input_frame;
    input integer t;
    integer w;
    begin
        for (w = 0; w < 60; w = w + 1) pool_wr(ACT_BASE + w, c6_spike5_stream[t*60 + w]);
    end
endtask

// Compare this timestep's pot6 to golden (the dedicated pot_mem is HW-carried).
task check_timestep;
    input integer t;
    reg [31:0] got;
    begin
        got = u_pot_mem.mem[0];
        if (got !== c6_pot6_golden[t]) begin
            $display("  FAIL t=%0d  pot6 got %08h exp %08h", t, got, c6_pot6_golden[t]);
            errors = errors + 1;
        end else
            $display("  OK  t=%0d  pot6 %08h match", t, got);
    end
endtask

// ─── Main ────────────────────────────────────────────────────────────────────
integer t, guard;
initial begin : run
    repeat(4) @(posedge clk);

    // fresh DUT state
    reset = 1'b1; repeat(4) @(posedge clk); #1;
    reset = 1'b0; @(posedge clk); #1;

    // memories + goldens
    load_static_mems;
    clear_state;
    $readmemh("../../fmi/mem_files/recurrent/test_inputs/c6/c6_spike5_stream.hex", c6_spike5_stream);
    $readmemh("../../fmi/mem_files/recurrent/test_inputs/c6/c6_pot6_golden.hex",   c6_pot6_golden);
    load_input_frame(0);                       // pre-load spike5 for t=0

    // config + boot regs
    load_config;
    write_boot_regs;

    // buffer seeding: buf0 (SRC input) full cnt=1; buf1 (RW output) full cnt=1.
    axi_write(SCH_MARKFULL, 32'h0000_0010);    // {cnt=1, id=0}
    axi_write(SCH_MARKFULL, 32'h0000_0011);    // {cnt=1, id=1}

    // Loop program (identical structure to C1/C2a; one dispatch per timestep):
    //   LOOP id0 count=T-1 ; TASK(con6 conv-FC + readout neuron) ; NXT in+out ; LOOPEND ; STOP
    //   TASK w1 0x0000_2000 : acc0 cfg0, slot0=SRC buf0 (spike5)
    //   TASK w2 0x1180_0000 : slot5=RW buf1 ntgt=1 (spike out self-cycles, no consumer)
    prog_mem[0] = ((T_C6-1) << 6) | 32'h6;     // LOOP id0, count=T-1
    prog_mem[1] = 32'h0000_2000;               // TASK word 1
    prog_mem[2] = 32'h1180_0000;               // TASK word 2 (RW output)
    prog_mem[3] = 32'h0000_0034;               // NXT nxt_in | nxt_out
    prog_mem[4] = 32'h0000_0007;               // LOOPEND id0
    prog_mem[5] = STOP_INST;

    axi_write(SCH_LOAD_PC, 32'd0);
    axi_write(SCH_START,   32'd0);

    $display("[C6] con6 readout layer on fmi_top (scheduler loop), T=%0d", T_C6);

    // Per-timestep lock-step: wait for this timestep's NXT, settle, check pot6, then
    // (and only then) re-feed spike5 + re-mark buf0 to release the next TASK.
    for (t = 0; t < T_C6; t = t + 1) begin
        guard = 0;
        while (nxt_out_count != (t+1) && guard < 4_000_000) begin
            @(posedge clk); guard = guard + 1;
        end
        if (nxt_out_count != (t+1)) begin
            $display("  FAIL t=%0d: no NXT pulse (dispatch stalled / timeout)", t);
            errors = errors + 1;
            disable run;
        end
        repeat(SETTLE) @(posedge clk); #1;
        check_timestep(t);
        if (t + 1 < T_C6) begin
            load_input_frame(t + 1);                 // spike5 for t+1 into ACT
            axi_write(SCH_MARKFULL, 32'h0000_0010);  // re-mark buf0 full {cnt=1,id=0}
        end
    end

    $display("=== tb_fmi_top_c6: %0d failure(s) ===", errors);
    if (errors == 0) $display("PASS"); else $display("FAIL");
    #20 $finish;
end

// safety net
initial begin
    #2_000_000_000;
    $display("FAIL: global timeout");
    $finish;
end

endmodule // tb_fmi_top_c6

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
