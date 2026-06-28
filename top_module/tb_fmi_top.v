// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
`timescale 10ps/1ps
`include "../shared/constants.v"

// =============================================================================
// tb_fmi_top.v — Stage C / C1 integration testbench for fmi_top.v
// Authors: Simon Davidson & Claude   Created: 2026-06-23   Last modified: 2026-06-23
//
// Drives the FMI multi-channel recurrent accelerator THROUGH THE FULL FABRIC
// (scheduler → config_manager → shared_pool) and reproduces the bit-exact T9
// golden (con2 feed-forward, group3→group4, ONE timestep) that the block-level
// tb_acc_fmiSnnMC_processor T9 already passes. New coverage vs T9 = the
// integration: scheduler dispatch, the 16-word config push, and pool arbitration
// of act/syn_curr/spike — not new golden arithmetic.
//
// Memory-backing split (deliberate):
//   - prog_mem / cfg_mem / bba_mem : behavioural arrays, COMBINATIONAL read —
//     matches the scheduler/config_manager expectation (cf. tb_flexman).
//   - 9 dedicated acc memories + 4 pool banks : sram_model, 1-CYCLE sync read —
//     matches the fmiSnnMC datapath timing that T9 validates (cf. T9 tb).
//
// Addressing (Step-0 finding): fmiSnnMC's sp_*/np_*_buff_addr ports are DEAD
// ("unused, kept for scheduler compatibility"); ALL addressing is via the config
// base words. So act/syn_curr/spike share the pool at distinct CONFIG bases
// (ACT_BASE / SYN_CURR_BASE / SPIKE_BASE); bba_mem content is don't-care.
//
// Readback: POOL_RD_BASE host AXI window (primary) + backdoor pool gather (cross-
// check), per the agreed plan. Runs both T9 variants (B sparse, A dense).
// =============================================================================
module tb_fmi_top;

// ─── Fabric params (match fmi_top defaults) ──────────────────────────────────
localparam PROG_DATA_BITS = 32;

// ─── TB memory sizing ────────────────────────────────────────────────────────
localparam CFG_DEPTH     = 256;   // prog/cfg/bba config tables (small)
localparam CFG_ADDR_BITS = 8;
localparam PROG_DEPTH    = 64;
localparam PROG_IDX_BITS = 6;
localparam DATA_DEPTH    = 4096;  // acc data memories + pool banks (1920 neurons)

// ─── Address map ─────────────────────────────────────────────────────────────
localparam [31:0] FMI_CFG_BASE = 32'h1000_0000;
localparam [31:0] POOL_RD_BASE = 32'h1010_0000;
localparam [31:0] SCH_LOAD_PC  = 32'hE000_0000;   // [24:20]=0
localparam [31:0] SCH_START    = 32'hE010_0000;   // [24:20]=1
localparam [31:0] SCH_STATUS   = 32'hE004_0000;   // read, [25:20]=1
localparam [31:0] SCH_MARKFULL = 32'hE050_0000;   // [24:20]=5
localparam STOP_INST = 32'h0000_0002;             // opcode 3'b010

// ─── Pool buffer layout (distinct, non-overlapping LOGICAL regions) ──────────
//   act  : 60 packed input words  -> [0   .. 59]
//   spike: 60 packed output words -> [64  .. 123]
//   syn_curr: 1920 per-neuron words -> [128 .. 2047]
localparam ACT_BASE      = 0;
localparam SPIKE_BASE    = 64;
localparam SYN_CURR_BASE = 128;

integer errors = 0;

// ─── Clock / reset ───────────────────────────────────────────────────────────
reg clk = 1'b0;
reg reset = 1'b1;
reg test_stall_pipe = 1'b0;
always #5 clk = ~clk;   // 10ps period units

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

// ─── AXI host tasks ──────────────────────────────────────────────────────────
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
        while (sys_ack_o !== 1'b1 && guard < 1000) begin
            @(posedge clk); #1; guard = guard + 1;
        end
        data = sys_data_o;
        sys_req_i = 1'b0; sys_addr_i = 32'h0;
    end
endtask

// Wait for the accelerator to dispatch then finish (poll DUT busy).
task wait_acc_done;
    integer guard;
    begin
        guard = 0;
        while (!u_dut.fmi_busy && guard < 200000) begin @(posedge clk); guard = guard + 1; end
        if (!u_dut.fmi_busy) begin
            $display("  FAIL: accelerator never dispatched"); errors = errors + 1;
        end else begin
            guard = 0;
            while (u_dut.fmi_busy && guard < 4000000) begin @(posedge clk); guard = guard + 1; end
            if (u_dut.fmi_busy) begin
                $display("  FAIL: accelerator did not finish (timeout)"); errors = errors + 1;
            end else
                $display("  acc finished after %0d cycles", guard);
        end
        // settle the writeback pipeline
        repeat(8) @(posedge clk);
    end
endtask

// ─── Golden + scratch arrays ─────────────────────────────────────────────────
reg [31:0] act_buf     [0:59];
reg [31:0] golden_syn  [0:1919];
reg [31:0] golden_spike[0:59];

integer i;

// Load the static (read-only, variant-independent) model memories once.
task load_static_mems;
    begin
        $readmemh("../../fmi/mem_files/recurrent/con2_weight.hex",    u_weight_mem.mem);
        $readmemh("../../fmi/mem_files/recurrent/group4_dcy_syn.hex", u_dcy_syn_mem.mem);
        $readmemh("../../fmi/mem_files/recurrent/group4_dcy_mem.hex", u_dcy_mem_mem.mem);
        $readmemh("../../fmi/mem_files/recurrent/group4_thresh.hex",  u_thresh_mem.mem);
    end
endtask

// Program the 16-word PACKED config for cfg_id 0 (T9 values; pool bases distinct).
task load_config;
    begin
        // base addresses (word index → cfg offset i*4)
        cfg_mem[ 0] = ACT_BASE;        // 0x00 act_base   (pool)
        cfg_mem[ 1] = 32'd0;           // 0x04 weight_base(dedicated)
        cfg_mem[ 2] = SYN_CURR_BASE;   // 0x08 syn_curr_base (pool)
        cfg_mem[ 3] = 32'd0;           // 0x0C thresh_base(dedicated)
        cfg_mem[ 4] = 32'd0;           // 0x10 pot_base   (dedicated)
        cfg_mem[ 5] = SPIKE_BASE;      // 0x14 spike_base (pool)
        cfg_mem[ 6] = 32'd0;           // 0x18 dcy_syn_base
        cfg_mem[ 7] = 32'd0;           // 0x1C dcy_mem_base
        cfg_mem[ 8] = 32'd0;           // 0x20 ada_base
        cfg_mem[ 9] = 32'd0;           // 0x24 b_eff_base
        cfg_mem[10] = 32'd0;           // 0x28 dcy_ada_base
        cfg_mem[11] = 32'd0;           // 0x2C scl_ada_base
        cfg_mem[12] = {16'd60, 16'd120};   // 0x30 S0: out_x=60 | in_x=120
        cfg_mem[13] = {16'd1919, 16'd0};   // 0x34 S1: last_neuron=1919 | rows_per_neuron=0
        cfg_mem[14] = 32'h2010_0801;       // 0x38 S2: cout=32 | cin=16 | stride=2[12:10] | total_timesteps=1
        cfg_mem[15] = 32'h2555_0040;       // 0x3C M0: conv, plain LIF, 32b
        // BBA table — don't-care (buff_addr ports are dead); keep zero.
        for (i = 0; i < 8; i = i + 1) bba_mem[i] = 32'd0;
    end
endtask

// Boot-only conv params (outside the 16-word window) via DIRECT host AXI.
task write_boot_regs;
    begin
        axi_write(FMI_CFG_BASE | 32'h5C, 32'd12);  // weight_idx_sz
        axi_write(FMI_CFG_BASE | 32'h74, 32'd5);   // x_kernel_len
        axi_write(FMI_CFG_BASE | 32'h84, 32'd2);   // x_kernel_offset (pad)
        // x_kernel_step (stride) is now per-config in S2[12:10] — see cfg_mem[14]
    end
endtask

// Zero the mutable state (pool syn_curr/spike regions, pot, ada) before a run.
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

// Compare pool results vs golden, via POOL_RD AXI window (primary) + backdoor.
task compare_results;
    input [255:0] tag;
    integer axi_mism, bd_mism;
    reg [31:0] got_axi, got_bd, logical;
    begin
        axi_mism = 0; bd_mism = 0;
        for (i = 0; i < 1920; i = i + 1) begin
            logical = SYN_CURR_BASE + i;
            axi_read(POOL_RD_BASE + (logical << 2), got_axi);
            got_bd = pool_rd(logical);
            if (got_axi !== golden_syn[i]) begin
                if (axi_mism < 8) $display("  %0s syn AXI MISMATCH n=%0d got %08h exp %08h",
                                            tag, i, got_axi, golden_syn[i]);
                axi_mism = axi_mism + 1;
            end
            if (got_bd !== golden_syn[i]) bd_mism = bd_mism + 1;
        end
        if (axi_mism == 0 && bd_mism == 0)
            $display("  OK  %0s syn_curr: 1920/1920 match (AXI + backdoor)", tag);
        else begin
            $display("  FAIL %0s syn_curr: AXI %0d, backdoor %0d mismatches", tag, axi_mism, bd_mism);
            errors = errors + 1;
        end

        axi_mism = 0; bd_mism = 0;
        for (i = 0; i < 60; i = i + 1) begin
            logical = SPIKE_BASE + i;
            axi_read(POOL_RD_BASE + (logical << 2), got_axi);
            got_bd = pool_rd(logical);
            if (got_axi !== golden_spike[i]) begin
                if (axi_mism < 8) $display("  %0s spike AXI MISMATCH w=%0d got %08h exp %08h",
                                            tag, i, got_axi, golden_spike[i]);
                axi_mism = axi_mism + 1;
            end
            if (got_bd !== golden_spike[i]) bd_mism = bd_mism + 1;
        end
        if (axi_mism == 0 && bd_mism == 0)
            $display("  OK  %0s spike: 60/60 match (AXI + backdoor)", tag);
        else begin
            $display("  FAIL %0s spike: AXI %0d, backdoor %0d mismatches", tag, axi_mism, bd_mism);
            errors = errors + 1;
        end
    end
endtask

// One full scheduler-driven dispatch + compare for a T9 variant.
task run_variant;
    input [255:0]  tag;
    input [1023:0] act_file;
    input [1023:0] syn_gold_file;
    input [1023:0] spk_gold_file;
    begin
        $display("[%0s] scheduler-driven T9 reproduction", tag);
        // fresh DUT state
        reset = 1'b1;
        repeat(4) @(posedge clk); #1;
        reset = 1'b0;
        @(posedge clk); #1;

        // memories
        load_static_mems;
        clear_state;
        $readmemh(act_file,      act_buf);
        $readmemh(syn_gold_file, golden_syn);
        $readmemh(spk_gold_file, golden_spike);
        for (i = 0; i < 60; i = i + 1) pool_wr(ACT_BASE + i, act_buf[i]);

        // config + boot regs + buffer readiness
        load_config;
        write_boot_regs;
        axi_write(SCH_MARKFULL, 32'h0000_0010);   // mark input buffer 0 full {cnt=1,id=0}

        // 1-TASK program: acc0 cfg0, slot0=SRC buf0 -> slot5=TGT buf1
        //   (7-bit cfg layout: slot0=SRC@[14:13], id0@[18:15] -> 0x2000)
        //   (4-bit ntgt layout: slot5=TGT@[23:22], id5@[27:24], ntgt@[31:28] -> 0x11C0_0000)
        prog_mem[0] = 32'h0000_2000;
        prog_mem[1] = 32'h11C0_0000;
        prog_mem[2] = STOP_INST;

        axi_write(SCH_LOAD_PC, 32'd0);
        axi_write(SCH_START,   32'd0);

        wait_acc_done;
        compare_results(tag);
    end
endtask

// ─── Main ────────────────────────────────────────────────────────────────────
initial begin
    $dumpfile("tb_fmi_top.vcd");
    $dumpvars(0, tb_fmi_top);

    repeat(4) @(posedge clk);

    run_variant("T9B",
                "../../fmi/mem_files/recurrent/test_inputs/act_input_B.hex",
                "../../fmi/mem_files/recurrent/test_inputs/syn_curr_golden_B.hex",
                "../../fmi/mem_files/recurrent/test_inputs/spike_golden_B.hex");

    run_variant("T9A",
                "../../fmi/mem_files/recurrent/test_inputs/act_input_A.hex",
                "../../fmi/mem_files/recurrent/test_inputs/syn_curr_golden_A.hex",
                "../../fmi/mem_files/recurrent/test_inputs/spike_golden_A.hex");

    $display("=== tb_fmi_top: %0d failure(s) ===", errors);
    if (errors == 0) $display("PASS"); else $display("FAIL");
    #20 $finish;
end

// safety net
initial begin
    #2_000_000_000;
    $display("FAIL: global timeout");
    $finish;
end

endmodule // tb_fmi_top

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
