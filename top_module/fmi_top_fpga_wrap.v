// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
// Authors: Simon Davidson & Claude   Created: 2026-06-30   Last modified: 2026-06-30
`timescale 10ps/1ps
`include "../shared/constants.v"

// FPGA synthesis wrapper for fmi_top (the FMI recurrent-SNN fabric, fmiSnnAccMC).
//
// Closes every external memory bus of fmi_top into on-chip BRAMs so this module is
// the FPGA top level -- only clk/reset/control + the sys_* host bus + a small loader
// stub are exposed.  Mirrors flexman_fpga_wrap.v (the generic-FlexMan equivalent)
// but instantiates fmi_top instead of flexman.
//
// All *_wait_i are tied low: synchronous BRAMs deliver data one cycle after the
// request with no stall (scheduler/config_manager already assume 1-cycle latency).
//
// Memory closures:
//   prog                  -> bram_dist (combinational read; scheduler fetch assumes
//                            0-cycle program memory under wait=0)
//   cfg, bba              -> bram_sdp  (config_manager R/W; loaded via the sys_* bus)
//   weight, thresh,       -> bram_sdp, READ-ONLY from the accelerator.  fmi_top has NO
//   dcy_syn, dcy_mem,        write path to these ("external loader writes via the
//   b_eff, dcy_ada,          SRAM's 2nd port"), so their write port is driven from a
//   scl_ada                  top-level LOADER STUB (ld_we_i/ld_addr_i/ld_data_i, shared
//                            across all seven).  Rationale: a tied-off write port would
//                            let synthesis fold these to constant-0 ROMs and prune the
//                            datapath, making utilisation meaningless; sourcing the
//                            write port from top-level inputs keeps them as real,
//                            loadable BRAMs.  The stub writes all seven in lockstep
//                            (same addr/data) -- fine for synth/util/timing; real
//                            per-memory AXI-mapped loading is a later change (cf. the
//                            pool-read window POOL_RD_BASE already in fmi_top for host
//                            result readback).
//   pot, ada              -> bram_sp  (accelerator R/W)
//   m0..m3 pool           -> bram_sp x4 (act rd / spike wr / syn_curr rw, 4 banks)
//
// ACC_MEM_DEPTH defaults to 4096: the FMI network needs it (con2 = 2560 weights,
// group4 = 1920 neurons).  Synthesis reports the real RAMB36 figure.

module fmi_top_fpga_wrap #(
    parameter ACC_MEM_DEPTH  = 4096,
    parameter PROG_MEM_DEPTH = 1024,
    parameter CFG_MEM_DEPTH  = 1024,
    parameter BBA_MEM_DEPTH  = 64,
    // fmi_top structural params (forwarded; defaults match fmi_top.v)
    parameter [31:0] FMI_CFG_BASE      = 32'h1000_0000,
    parameter [31:0] POOL_RD_BASE      = 32'h1010_0000,
    parameter [31:0] CM_CFG_MEM_ADDR   = 32'hA000_0000,
    parameter [31:0] CM_CFG_MEM_MASK   = 32'hFF00_0000,
    parameter [31:0] CM_BBA_MEM_ADDR   = 32'hB000_0000,
    parameter [31:0] CM_BBA_MEM_MASK   = 32'hFF00_0000,
    parameter [31:0] SCH_PROG_MEM_ADDR = 32'hD000_0000,
    parameter [31:0] SCH_PROG_MEM_MASK = 32'hFF00_0000,
    parameter NUM_BUFFERS         = 16,
    parameter NUM_HW_ACCELERATORS = 2,
    parameter WORDS_PER_CONFIG    = 16,
    parameter CFG_ID_SZ           = 7,
    parameter BUFF_INDX_SZ        = 4,
    parameter TGT_ACC_SZ          = 3,
    parameter TGT_COUNT_SZ        = 4,
    parameter PROG_ADDR_BITS      = 10,
    parameter PROG_DATA_BITS      = 32,
    parameter NUM_SCH_ENTRIES     = 4,
    parameter COL_BUFF_ID_SZ      = 16
)(
    input  wire clk,
    input  wire reset,
    input  wire test_stall_pipe,

    // ── Shared host bus ──────────────────────────────────────────────────────
    input  wire         sys_req_i,
    output wire         sys_ack_o,
    input  wire  [31:0] sys_addr_i,
    input  wire  [31:0] sys_data_i,
    output wire  [31:0] sys_data_o,

    // ── Status ───────────────────────────────────────────────────────────────
    output wire         nxt_input_pulse_o,
    output wire         nxt_output_pulse_o,
    output wire         cm_config_finished_o,

    // ── Loader stub for the read-only accelerator memories (see header) ───────
    input  wire                  ld_we_i,
    input  wire [`ADDR_SIZE-1:0] ld_addr_i,
    input  wire           [31:0] ld_data_i
);

// ─── Derived address widths ──────────────────────────────────────────────────
localparam ACC_ABITS  = $clog2(ACC_MEM_DEPTH);
localparam PROG_ABITS = $clog2(PROG_MEM_DEPTH);
localparam CFG_ABITS  = $clog2(CFG_MEM_DEPTH);
localparam BBA_ABITS  = $clog2(BBA_MEM_DEPTH);

// ─── Internal wires — fmi_top ↔ BRAMs ────────────────────────────────────────
// Program memory
wire [`ADDR_SIZE-1:0]      prog_mem_addr_o;
wire [PROG_DATA_BITS-1:0]  prog_mem_data_i;
wire                       prog_mem_req_o;
wire                       prog_mem_wr_o;
wire [PROG_ADDR_BITS-1:0]  prog_mem_wr_addr_o;
wire [PROG_DATA_BITS-1:0]  prog_mem_wr_data_o;

// Config memory
wire                  cfg_mem_rd_o, cfg_mem_wr_o;
wire [31:0]           cfg_mem_addr_o, cfg_mem_wr_addr_o, cfg_mem_wr_data_o;
wire [31:0]           cfg_mem_data_i;

// Buffer base-address memory
wire                  bba_mem_rd_o, bba_mem_wr_o;
wire [31:0]           bba_mem_addr_o, bba_mem_wr_addr_o, bba_mem_wr_data_o;
wire [31:0]           bba_mem_data_i;

// Read-only accelerator memories (loaded via the loader stub)
wire                  weight_mem_rd_o;   wire [`ADDR_SIZE-1:0] weight_mem_addr_o;   wire [`WTD_BITS-1:0] weight_mem_data_i;
wire                  thresh_mem_rd_o;   wire [`ADDR_SIZE-1:0] thresh_mem_addr_o;   wire [`WTD_BITS-1:0] thresh_mem_data_i;
wire                  dcy_syn_mem_rd_o;  wire [`ADDR_SIZE-1:0] dcy_syn_mem_addr_o;  wire [31:0] dcy_syn_mem_data_i;
wire                  dcy_mem_mem_rd_o;  wire [`ADDR_SIZE-1:0] dcy_mem_mem_addr_o;  wire [31:0] dcy_mem_mem_data_i;
wire                  b_eff_mem_rd_o;    wire [`ADDR_SIZE-1:0] b_eff_mem_addr_o;    wire [31:0] b_eff_mem_data_i;
wire                  dcy_ada_mem_rd_o;  wire [`ADDR_SIZE-1:0] dcy_ada_mem_addr_o;  wire [31:0] dcy_ada_mem_data_i;
wire                  scl_ada_mem_rd_o;  wire [`ADDR_SIZE-1:0] scl_ada_mem_addr_o;  wire [31:0] scl_ada_mem_data_i;

// Read/write accelerator memories
wire                  pot_mem_wr_o, pot_mem_rd_o;
wire [`ADDR_SIZE-1:0] pot_mem_addr_o;
wire [`POT_BITS-1:0]  pot_mem_data_o, pot_mem_data_i;

wire                  ada_mem_wr_o, ada_mem_rd_o;
wire [`ADDR_SIZE-1:0] ada_mem_addr_o;
wire [31:0]           ada_mem_data_o, ada_mem_data_i;

// Shared act/spike/syn_curr data pool (4 interleaved banks)
wire                  m0_data_mem_rd_o, m0_data_mem_wr_o; wire [`ADDR_SIZE-1:0] m0_data_mem_addr_o; wire [31:0] m0_data_mem_wdata_o, m0_data_mem_rdata_i;
wire                  m1_data_mem_rd_o, m1_data_mem_wr_o; wire [`ADDR_SIZE-1:0] m1_data_mem_addr_o; wire [31:0] m1_data_mem_wdata_o, m1_data_mem_rdata_i;
wire                  m2_data_mem_rd_o, m2_data_mem_wr_o; wire [`ADDR_SIZE-1:0] m2_data_mem_addr_o; wire [31:0] m2_data_mem_wdata_o, m2_data_mem_rdata_i;
wire                  m3_data_mem_rd_o, m3_data_mem_wr_o; wire [`ADDR_SIZE-1:0] m3_data_mem_addr_o; wire [31:0] m3_data_mem_wdata_o, m3_data_mem_rdata_i;

// IOB output-register intermediates
wire        sys_ack_int;
wire [31:0] sys_data_int;
wire        nxt_input_pulse_int, nxt_output_pulse_int, cm_config_finished_int;

// ─── fmi_top instantiation ───────────────────────────────────────────────────
fmi_top #(
    .FMI_CFG_BASE        (FMI_CFG_BASE),
    .POOL_RD_BASE        (POOL_RD_BASE),
    .CM_CFG_MEM_ADDR     (CM_CFG_MEM_ADDR),
    .CM_CFG_MEM_MASK     (CM_CFG_MEM_MASK),
    .CM_BBA_MEM_ADDR     (CM_BBA_MEM_ADDR),
    .CM_BBA_MEM_MASK     (CM_BBA_MEM_MASK),
    .SCH_PROG_MEM_ADDR   (SCH_PROG_MEM_ADDR),
    .SCH_PROG_MEM_MASK   (SCH_PROG_MEM_MASK),
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
) u_fmi_top (
    .clk                 (clk),
    .reset               (reset),
    .test_stall_pipe     (test_stall_pipe),

    .sys_req_i           (sys_req_i),
    .sys_ack_o           (sys_ack_int),
    .sys_addr_i          (sys_addr_i),
    .sys_data_i          (sys_data_i),
    .sys_data_o          (sys_data_int),

    // Program memory
    .prog_mem_addr_o     (prog_mem_addr_o),
    .prog_mem_data_i     (prog_mem_data_i),
    .prog_mem_req_o      (prog_mem_req_o),
    .prog_mem_wait_i     (1'b0),
    .prog_mem_wr_o       (prog_mem_wr_o),
    .prog_mem_wr_addr_o  (prog_mem_wr_addr_o),
    .prog_mem_wr_data_o  (prog_mem_wr_data_o),
    .prog_mem_wr_wait_i  (1'b0),

    // Config memory
    .cfg_mem_rd_o        (cfg_mem_rd_o),
    .cfg_mem_wait_i      (1'b0),
    .cfg_mem_addr_o      (cfg_mem_addr_o),
    .cfg_mem_data_i      (cfg_mem_data_i),
    .cfg_mem_wr_o        (cfg_mem_wr_o),
    .cfg_mem_wr_addr_o   (cfg_mem_wr_addr_o),
    .cfg_mem_wr_data_o   (cfg_mem_wr_data_o),

    // Buffer base-address memory
    .bba_mem_rd_o        (bba_mem_rd_o),
    .bba_mem_wait_i      (1'b0),
    .bba_mem_addr_o      (bba_mem_addr_o),
    .bba_mem_data_i      (bba_mem_data_i),
    .bba_mem_wr_o        (bba_mem_wr_o),
    .bba_mem_wr_addr_o   (bba_mem_wr_addr_o),
    .bba_mem_wr_data_o   (bba_mem_wr_data_o),

    // Scheduler NXT pulses + config status
    .nxt_input_pulse_o   (nxt_input_pulse_int),
    .nxt_output_pulse_o  (nxt_output_pulse_int),
    .cm_config_finished_o(cm_config_finished_int),

    // Read-only accelerator memories
    .weight_mem_rd_o     (weight_mem_rd_o),
    .weight_mem_wait_i   (1'b0),
    .weight_mem_addr_o   (weight_mem_addr_o),
    .weight_mem_data_i   (weight_mem_data_i),

    .thresh_mem_rd_o     (thresh_mem_rd_o),
    .thresh_mem_wait_i   (1'b0),
    .thresh_mem_addr_o   (thresh_mem_addr_o),
    .thresh_mem_data_i   (thresh_mem_data_i),

    .dcy_syn_mem_rd_o    (dcy_syn_mem_rd_o),
    .dcy_syn_mem_wait_i  (1'b0),
    .dcy_syn_mem_addr_o  (dcy_syn_mem_addr_o),
    .dcy_syn_mem_data_i  (dcy_syn_mem_data_i),

    .dcy_mem_mem_rd_o    (dcy_mem_mem_rd_o),
    .dcy_mem_mem_wait_i  (1'b0),
    .dcy_mem_mem_addr_o  (dcy_mem_mem_addr_o),
    .dcy_mem_mem_data_i  (dcy_mem_mem_data_i),

    .b_eff_mem_rd_o      (b_eff_mem_rd_o),
    .b_eff_mem_wait_i    (1'b0),
    .b_eff_mem_addr_o    (b_eff_mem_addr_o),
    .b_eff_mem_data_i    (b_eff_mem_data_i),

    .dcy_ada_mem_rd_o    (dcy_ada_mem_rd_o),
    .dcy_ada_mem_wait_i  (1'b0),
    .dcy_ada_mem_addr_o  (dcy_ada_mem_addr_o),
    .dcy_ada_mem_data_i  (dcy_ada_mem_data_i),

    .scl_ada_mem_rd_o    (scl_ada_mem_rd_o),
    .scl_ada_mem_wait_i  (1'b0),
    .scl_ada_mem_addr_o  (scl_ada_mem_addr_o),
    .scl_ada_mem_data_i  (scl_ada_mem_data_i),

    // Read/write accelerator memories
    .pot_mem_wr_o        (pot_mem_wr_o),
    .pot_mem_rd_o        (pot_mem_rd_o),
    .pot_mem_wait_i      (1'b0),
    .pot_mem_addr_o      (pot_mem_addr_o),
    .pot_mem_data_o      (pot_mem_data_o),
    .pot_mem_data_i      (pot_mem_data_i),

    .ada_mem_wr_o        (ada_mem_wr_o),
    .ada_mem_rd_o        (ada_mem_rd_o),
    .ada_mem_wait_i      (1'b0),
    .ada_mem_addr_o      (ada_mem_addr_o),
    .ada_mem_data_o      (ada_mem_data_o),
    .ada_mem_data_i      (ada_mem_data_i),

    // Shared act/spike/syn_curr data pool
    .m0_data_mem_rd_o    (m0_data_mem_rd_o),
    .m0_data_mem_wr_o    (m0_data_mem_wr_o),
    .m0_data_mem_wait_i  (1'b0),
    .m0_data_mem_addr_o  (m0_data_mem_addr_o),
    .m0_data_mem_wdata_o (m0_data_mem_wdata_o),
    .m0_data_mem_rdata_i (m0_data_mem_rdata_i),

    .m1_data_mem_rd_o    (m1_data_mem_rd_o),
    .m1_data_mem_wr_o    (m1_data_mem_wr_o),
    .m1_data_mem_wait_i  (1'b0),
    .m1_data_mem_addr_o  (m1_data_mem_addr_o),
    .m1_data_mem_wdata_o (m1_data_mem_wdata_o),
    .m1_data_mem_rdata_i (m1_data_mem_rdata_i),

    .m2_data_mem_rd_o    (m2_data_mem_rd_o),
    .m2_data_mem_wr_o    (m2_data_mem_wr_o),
    .m2_data_mem_wait_i  (1'b0),
    .m2_data_mem_addr_o  (m2_data_mem_addr_o),
    .m2_data_mem_wdata_o (m2_data_mem_wdata_o),
    .m2_data_mem_rdata_i (m2_data_mem_rdata_i),

    .m3_data_mem_rd_o    (m3_data_mem_rd_o),
    .m3_data_mem_wr_o    (m3_data_mem_wr_o),
    .m3_data_mem_wait_i  (1'b0),
    .m3_data_mem_addr_o  (m3_data_mem_addr_o),
    .m3_data_mem_wdata_o (m3_data_mem_wdata_o),
    .m3_data_mem_rdata_i (m3_data_mem_rdata_i)
);

// ─── System memories ─────────────────────────────────────────────────────────

// Program memory: AXI loads via prog_mem_wr_*, scheduler reads (combinational).
bram_dist #(.DEPTH(PROG_MEM_DEPTH), .DATA_W(PROG_DATA_BITS)) u_prog_mem (
    .clk(clk), .we(prog_mem_wr_o), .waddr(prog_mem_wr_addr_o[PROG_ABITS-1:0]),
    .din(prog_mem_wr_data_o), .raddr(prog_mem_addr_o[PROG_ABITS-1:0]), .dout(prog_mem_data_i)
);

// Config + buffer-base-address memories (config_manager R/W via the sys_* bus).
bram_sdp #(.DEPTH(CFG_MEM_DEPTH), .DATA_W(32)) u_cfg_mem (
    .clk(clk), .we(cfg_mem_wr_o), .waddr(cfg_mem_wr_addr_o[CFG_ABITS-1:0]),
    .din(cfg_mem_wr_data_o), .raddr(cfg_mem_addr_o[CFG_ABITS-1:0]), .dout(cfg_mem_data_i)
);
bram_sdp #(.DEPTH(BBA_MEM_DEPTH), .DATA_W(32)) u_bba_mem (
    .clk(clk), .we(bba_mem_wr_o), .waddr(bba_mem_wr_addr_o[BBA_ABITS-1:0]),
    .din(bba_mem_wr_data_o), .raddr(bba_mem_addr_o[BBA_ABITS-1:0]), .dout(bba_mem_data_i)
);

// ─── Read-only accelerator memories (write port = loader stub) ────────────────
bram_sdp #(.DEPTH(ACC_MEM_DEPTH), .DATA_W(`WTD_BITS)) u_weight_mem (
    .clk(clk), .we(ld_we_i), .waddr(ld_addr_i[ACC_ABITS-1:0]), .din(ld_data_i[`WTD_BITS-1:0]),
    .raddr(weight_mem_addr_o[ACC_ABITS-1:0]), .dout(weight_mem_data_i)
);
bram_sdp #(.DEPTH(ACC_MEM_DEPTH), .DATA_W(`WTD_BITS)) u_thresh_mem (
    .clk(clk), .we(ld_we_i), .waddr(ld_addr_i[ACC_ABITS-1:0]), .din(ld_data_i[`WTD_BITS-1:0]),
    .raddr(thresh_mem_addr_o[ACC_ABITS-1:0]), .dout(thresh_mem_data_i)
);
bram_sdp #(.DEPTH(ACC_MEM_DEPTH), .DATA_W(32)) u_dcy_syn_mem (
    .clk(clk), .we(ld_we_i), .waddr(ld_addr_i[ACC_ABITS-1:0]), .din(ld_data_i),
    .raddr(dcy_syn_mem_addr_o[ACC_ABITS-1:0]), .dout(dcy_syn_mem_data_i)
);
bram_sdp #(.DEPTH(ACC_MEM_DEPTH), .DATA_W(32)) u_dcy_mem_mem (
    .clk(clk), .we(ld_we_i), .waddr(ld_addr_i[ACC_ABITS-1:0]), .din(ld_data_i),
    .raddr(dcy_mem_mem_addr_o[ACC_ABITS-1:0]), .dout(dcy_mem_mem_data_i)
);
bram_sdp #(.DEPTH(ACC_MEM_DEPTH), .DATA_W(32)) u_b_eff_mem (
    .clk(clk), .we(ld_we_i), .waddr(ld_addr_i[ACC_ABITS-1:0]), .din(ld_data_i),
    .raddr(b_eff_mem_addr_o[ACC_ABITS-1:0]), .dout(b_eff_mem_data_i)
);
bram_sdp #(.DEPTH(ACC_MEM_DEPTH), .DATA_W(32)) u_dcy_ada_mem (
    .clk(clk), .we(ld_we_i), .waddr(ld_addr_i[ACC_ABITS-1:0]), .din(ld_data_i),
    .raddr(dcy_ada_mem_addr_o[ACC_ABITS-1:0]), .dout(dcy_ada_mem_data_i)
);
bram_sdp #(.DEPTH(ACC_MEM_DEPTH), .DATA_W(32)) u_scl_ada_mem (
    .clk(clk), .we(ld_we_i), .waddr(ld_addr_i[ACC_ABITS-1:0]), .din(ld_data_i),
    .raddr(scl_ada_mem_addr_o[ACC_ABITS-1:0]), .dout(scl_ada_mem_data_i)
);

// ─── Read/write accelerator memories ──────────────────────────────────────────
bram_sp #(.DEPTH(ACC_MEM_DEPTH), .DATA_W(`POT_BITS)) u_pot_mem (
    .clk(clk), .we(pot_mem_wr_o), .addr(pot_mem_addr_o[ACC_ABITS-1:0]),
    .din(pot_mem_data_o), .dout(pot_mem_data_i)
);
bram_sp #(.DEPTH(ACC_MEM_DEPTH), .DATA_W(32)) u_ada_mem (
    .clk(clk), .we(ada_mem_wr_o), .addr(ada_mem_addr_o[ACC_ABITS-1:0]),
    .din(ada_mem_data_o), .dout(ada_mem_data_i)
);

// ─── Shared data pool — 4 interleaved 32-bit banks ────────────────────────────
bram_sp #(.DEPTH(ACC_MEM_DEPTH), .DATA_W(32)) u_m0_data_mem (
    .clk(clk), .we(m0_data_mem_wr_o), .addr(m0_data_mem_addr_o[ACC_ABITS-1:0]),
    .din(m0_data_mem_wdata_o), .dout(m0_data_mem_rdata_i)
);
bram_sp #(.DEPTH(ACC_MEM_DEPTH), .DATA_W(32)) u_m1_data_mem (
    .clk(clk), .we(m1_data_mem_wr_o), .addr(m1_data_mem_addr_o[ACC_ABITS-1:0]),
    .din(m1_data_mem_wdata_o), .dout(m1_data_mem_rdata_i)
);
bram_sp #(.DEPTH(ACC_MEM_DEPTH), .DATA_W(32)) u_m2_data_mem (
    .clk(clk), .we(m2_data_mem_wr_o), .addr(m2_data_mem_addr_o[ACC_ABITS-1:0]),
    .din(m2_data_mem_wdata_o), .dout(m2_data_mem_rdata_i)
);
bram_sp #(.DEPTH(ACC_MEM_DEPTH), .DATA_W(32)) u_m3_data_mem (
    .clk(clk), .we(m3_data_mem_wr_o), .addr(m3_data_mem_addr_o[ACC_ABITS-1:0]),
    .din(m3_data_mem_wdata_o), .dout(m3_data_mem_rdata_i)
);

// ─── IOB output registers ─────────────────────────────────────────────────────
// One FF per output, placed in the I/O block.  All outputs gain +1 cycle latency
// at the pin (matches flexman_fpga_wrap); the AXI host must allow for it.
(* IOB = "TRUE" *) reg        sys_ack_r;
(* IOB = "TRUE" *) reg [31:0] sys_data_r;
(* IOB = "TRUE" *) reg        nxt_input_pulse_r;
(* IOB = "TRUE" *) reg        nxt_output_pulse_r;
(* IOB = "TRUE" *) reg        cm_config_finished_r;

always @(posedge clk) begin
    sys_ack_r            <= sys_ack_int;
    sys_data_r           <= sys_data_int;
    nxt_input_pulse_r    <= nxt_input_pulse_int;
    nxt_output_pulse_r   <= nxt_output_pulse_int;
    cm_config_finished_r <= cm_config_finished_int;
end

assign sys_ack_o            = sys_ack_r;
assign sys_data_o           = sys_data_r;
assign nxt_input_pulse_o    = nxt_input_pulse_r;
assign nxt_output_pulse_o   = nxt_output_pulse_r;
assign cm_config_finished_o = cm_config_finished_r;

endmodule
