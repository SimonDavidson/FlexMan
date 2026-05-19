// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
`timescale 10ps/1ps
`include "../shared/constants.v"

// FPGA synthesis wrapper for flexman.
//
// Closes all external memory buses from flexman.v into on-chip BRAMs so
// that this module is the FPGA top level — only control, AXI, and spike
// readback ports are exposed externally.
//
// All *_wait_i signals are tied low: synchronous BRAMs deliver data one
// cycle after the request with no stall.  The scheduler and config_manager
// are already designed for 1-cycle memory latency.
//
// Memory sizing (all depths in 32-bit words):
//   ACC_MEM_DEPTH  — per-accelerator data memories (weight, act, etc.)
//   PROG_MEM_DEPTH — program instruction memory
//   CFG_MEM_DEPTH  — config_manager configuration data memory
//   BBA_MEM_DEPTH  — buffer base-address memory (≥ NUM_BUFFERS × 4)
//
// With defaults (1024 words each, 64 for BBA): ~27 RAMB36 on Xilinx
// 7-series.  Increase ACC_MEM_DEPTH to 4096 for larger networks (~56 RAMB36).
//
// Program loading:
//   Write instructions via prog_wr_en_i / prog_wr_addr_i / prog_wr_data_i
//   before asserting start_program_i.
//
// Spike readback:
//   After computation, read spike outputs from s0/s1/a0_spike_rd_addr_i →
//   s0/s1/a0_spike_rd_data_o.  Data is registered; present address on
//   cycle N, data valid on cycle N+1.

module flexman_fpga_wrap #(
    parameter ACC_MEM_DEPTH       = 1024,
    parameter PROG_MEM_DEPTH      = 1024,
    parameter CFG_MEM_DEPTH       = 1024,
    parameter BBA_MEM_DEPTH       = 64,
    // flexman structural parameters — must match sub-module compilation
    parameter [31:0] SNN0_CFG_BASE           = 32'h1000_0000,
    parameter [31:0] SNN1_CFG_BASE           = 32'h1001_0000,
    parameter [31:0] ANN_CFG_BASE            = 32'h1002_0000,
    parameter [31:0] HAD_CFG_BASE            = 32'h1003_0000,
    parameter [31:0] CM_CFG_MEM_ADDR         = 32'hA000_0000,
    parameter [31:0] CM_CFG_MEM_MASK         = 32'hFF00_0000,
    parameter [31:0] CM_BBA_MEM_ADDR         = 32'hB000_0000,
    parameter [31:0] CM_BBA_MEM_MASK         = 32'hFF00_0000,
    parameter [31:0] FU_TABLE_ADDR           = 32'hC000_0000,
    parameter [31:0] FU_TABLE_ADDR_MASK      = 32'hFF00_0000,
    parameter [31:0] SCH_PROG_MEM_ADDR       = 32'hD000_0000,
    parameter [31:0] SCH_PROG_MEM_MASK       = 32'hFF00_0000,
    parameter NUM_BUFFERS         = 16,
    parameter NUM_HW_ACCELERATORS = 5,
    parameter WORDS_PER_CONFIG    = 4,
    parameter CFG_ID_SZ           = 5,
    parameter BUFF_INDX_SZ        = 4,
    parameter TGT_ACC_SZ          = 3,
    parameter TGT_COUNT_SZ        = 3,
    parameter PROG_ADDR_BITS      = 10,
    parameter PROG_DATA_BITS      = 32,
    parameter NUM_SCH_ENTRIES     = 4,
    parameter COL_BUFF_ID_SZ      = 16
)(
    input  wire clk,
    input  wire reset,
    input  wire test_stall_pipe,

    // ── Shared host AXI bus ──────────────────────────────────────────────────
    input  wire         sys_req_i,
    output wire         sys_ack_o,
    input  wire  [31:0] sys_addr_i,
    input  wire  [31:0] sys_data_i,
    output wire  [31:0] sys_data_o,

    // ── Buffer pre-fill / NXT pulses ─────────────────────────────────────────
    input  wire                    mark_buff_as_full_i,
    input  wire [BUFF_INDX_SZ-1:0] full_buff_id_i,
    input  wire [TGT_COUNT_SZ-1:0] full_buff_usage_i,
    output wire                    nxt_input_pulse_o,
    output wire                    nxt_output_pulse_o,

    // ── Config manager status ────────────────────────────────────────────────
    output wire cm_config_finished_o,

    // ── Spike memory readback (host reads spike train after computation) ─────
    input  wire [$clog2(ACC_MEM_DEPTH)-1:0] s0_spike_rd_addr_i,
    output wire             [`ACT_BITS-1:0] s0_spike_rd_data_o,
    input  wire [$clog2(ACC_MEM_DEPTH)-1:0] s1_spike_rd_addr_i,
    output wire             [`ACT_BITS-1:0] s1_spike_rd_data_o,
    input  wire [$clog2(ACC_MEM_DEPTH)-1:0] a0_spike_rd_addr_i,
    output wire             [`ACT_BITS-1:0] a0_spike_rd_data_o
);

// ─── Derived address widths ───────────────────────────────────────────────────
localparam ACC_ABITS  = $clog2(ACC_MEM_DEPTH);
localparam PROG_ABITS = $clog2(PROG_MEM_DEPTH);
localparam CFG_ABITS  = $clog2(CFG_MEM_DEPTH);
localparam BBA_ABITS  = $clog2(BBA_MEM_DEPTH);

// ─── Internal wires — flexman ↔ BRAMs ────────────────────────────────────────
// Naming mirrors flexman port names exactly; _o wires carry flexman outputs
// (BRAM inputs), _i wires carry BRAM outputs (flexman inputs).

// Program memory
wire [`ADDR_SIZE-1:0]      prog_mem_addr_o;
wire                       prog_mem_req_o;
wire [PROG_DATA_BITS-1:0]  prog_mem_data_i;
// Program memory write bus (scheduler ← AXI → BRAM)
wire                       prog_mem_wr_o;
wire [PROG_ABITS-1:0]      prog_mem_wr_addr;
wire [PROG_DATA_BITS-1:0]  prog_mem_wr_data;

// Config memory
wire                  cfg_mem_rd_o,  cfg_mem_wr_o;
wire [31:0]           cfg_mem_addr_o, cfg_mem_wr_addr_o, cfg_mem_wr_data_o;
wire [31:0]           cfg_mem_data_i;

// BBA memory — config_manager port
wire                  bba_mem_rd_o,  bba_mem_wr_o;
wire [31:0]           bba_mem_addr_o, bba_mem_wr_addr_o, bba_mem_wr_data_o;
wire [31:0]           bba_mem_data_i;

// BBA memory — fill_unit port
wire                  fu_bba_mem_rd_o;
wire [31:0]           fu_bba_mem_addr_o;
wire [31:0]           fu_bba_mem_data_i;

// BBA TDP port A mux: cfg_mgr read and write share one physical port
wire [BBA_ABITS-1:0] bba_port_a_addr =
    bba_mem_wr_o ? bba_mem_wr_addr_o[BBA_ABITS-1:0]
                 : bba_mem_addr_o[BBA_ABITS-1:0];

// ── snnAcc0 ──────────────────────────────────────────────────────────────────
wire                   s0_weight_mem_rd_o;
wire [`ADDR_SIZE-1:0]  s0_weight_mem_addr_o;
wire [`WTD_BITS-1:0]   s0_weight_mem_data_i;
wire                   s0_weight_mem_wr_o;
wire [`ADDR_SIZE-1:0]  s0_weight_mem_wr_addr_o;
wire [31:0]            s0_weight_mem_wr_data_o;

wire                   s0_act_mem_req_o;
wire [`ADDR_SIZE-1:0]  s0_act_mem_addr_o;
wire [`ACT_BITS-1:0]   s0_act_mem_data_i;
wire                   s0_act_mem_wr_o;
wire [`ADDR_SIZE-1:0]  s0_act_mem_wr_addr_o;
wire [31:0]            s0_act_mem_wr_data_o;

wire                   s0_syn_curr_mem_wr_o, s0_syn_curr_mem_rd_o;
wire [`ADDR_SIZE-1:0]  s0_syn_curr_mem_addr_o;
wire [`POT_BITS-1:0]   s0_syn_curr_mem_data_o;
wire [`POT_BITS-1:0]   s0_syn_curr_mem_data_i;

wire                   s0_bias_curr_mem_rd_o;
wire [`ADDR_SIZE-1:0]  s0_bias_curr_mem_addr_o;
wire [`WTD_BITS-1:0]   s0_bias_curr_mem_data_i;
wire                   s0_bias_curr_mem_wr_o;
wire [`ADDR_SIZE-1:0]  s0_bias_curr_mem_wr_addr_o;
wire [31:0]            s0_bias_curr_mem_wr_data_o;

wire                   s0_thresh_mem_rd_o;
wire [`ADDR_SIZE-1:0]  s0_thresh_mem_addr_o;
wire [`WTD_BITS-1:0]   s0_thresh_mem_data_i;
wire                   s0_thresh_mem_wr_o;
wire [`ADDR_SIZE-1:0]  s0_thresh_mem_wr_addr_o;
wire [31:0]            s0_thresh_mem_wr_data_o;

wire                   s0_pot_mem_wr_o, s0_pot_mem_rd_o;
wire [`ADDR_SIZE-1:0]  s0_pot_mem_addr_o;
wire [`POT_BITS-1:0]   s0_pot_mem_data_o;
wire [`POT_BITS-1:0]   s0_pot_mem_data_i;

wire                   s0_spike_mem_wr_o;
wire [`ADDR_SIZE-1:0]  s0_spike_mem_addr_o;
wire [`ACT_BITS-1:0]   s0_spike_mem_data_o;

// ── snnAcc1 ──────────────────────────────────────────────────────────────────
wire                   s1_weight_mem_rd_o;
wire [`ADDR_SIZE-1:0]  s1_weight_mem_addr_o;
wire [`WTD_BITS-1:0]   s1_weight_mem_data_i;
wire                   s1_weight_mem_wr_o;
wire [`ADDR_SIZE-1:0]  s1_weight_mem_wr_addr_o;
wire [31:0]            s1_weight_mem_wr_data_o;

wire                   s1_act_mem_req_o;
wire [`ADDR_SIZE-1:0]  s1_act_mem_addr_o;
wire [`ACT_BITS-1:0]   s1_act_mem_data_i;
wire                   s1_act_mem_wr_o;
wire [`ADDR_SIZE-1:0]  s1_act_mem_wr_addr_o;
wire [31:0]            s1_act_mem_wr_data_o;

wire                   s1_syn_curr_mem_wr_o, s1_syn_curr_mem_rd_o;
wire [`ADDR_SIZE-1:0]  s1_syn_curr_mem_addr_o;
wire [`POT_BITS-1:0]   s1_syn_curr_mem_data_o;
wire [`POT_BITS-1:0]   s1_syn_curr_mem_data_i;

wire                   s1_bias_curr_mem_rd_o;
wire [`ADDR_SIZE-1:0]  s1_bias_curr_mem_addr_o;
wire [`WTD_BITS-1:0]   s1_bias_curr_mem_data_i;
wire                   s1_bias_curr_mem_wr_o;
wire [`ADDR_SIZE-1:0]  s1_bias_curr_mem_wr_addr_o;
wire [31:0]            s1_bias_curr_mem_wr_data_o;

wire                   s1_thresh_mem_rd_o;
wire [`ADDR_SIZE-1:0]  s1_thresh_mem_addr_o;
wire [`WTD_BITS-1:0]   s1_thresh_mem_data_i;
wire                   s1_thresh_mem_wr_o;
wire [`ADDR_SIZE-1:0]  s1_thresh_mem_wr_addr_o;
wire [31:0]            s1_thresh_mem_wr_data_o;

wire                   s1_pot_mem_wr_o, s1_pot_mem_rd_o;
wire [`ADDR_SIZE-1:0]  s1_pot_mem_addr_o;
wire [`POT_BITS-1:0]   s1_pot_mem_data_o;
wire [`POT_BITS-1:0]   s1_pot_mem_data_i;

wire                   s1_spike_mem_wr_o;
wire [`ADDR_SIZE-1:0]  s1_spike_mem_addr_o;
wire [`ACT_BITS-1:0]   s1_spike_mem_data_o;

// ── annAcc ───────────────────────────────────────────────────────────────────
wire                   a0_weight_mem_rd_o;
wire [`ADDR_SIZE-1:0]  a0_weight_mem_addr_o;
wire [`WTD_BITS-1:0]   a0_weight_mem_data_i;
wire                   a0_weight_mem_wr_o;
wire [`ADDR_SIZE-1:0]  a0_weight_mem_wr_addr_o;
wire [31:0]            a0_weight_mem_wr_data_o;

wire                   a0_act_mem_req_o;
wire [`ADDR_SIZE-1:0]  a0_act_mem_addr_o;
wire [`ACT_BITS-1:0]   a0_act_mem_data_i;
wire                   a0_act_mem_wr_o;
wire [`ADDR_SIZE-1:0]  a0_act_mem_wr_addr_o;
wire [31:0]            a0_act_mem_wr_data_o;

wire                   a0_syn_curr_mem_wr_o, a0_syn_curr_mem_rd_o;
wire [`ADDR_SIZE-1:0]  a0_syn_curr_mem_addr_o;
wire [`POT_BITS-1:0]   a0_syn_curr_mem_data_o;
wire [`POT_BITS-1:0]   a0_syn_curr_mem_data_i;

wire                   a0_bias_curr_mem_rd_o;
wire [`ADDR_SIZE-1:0]  a0_bias_curr_mem_addr_o;
wire [`WTD_BITS-1:0]   a0_bias_curr_mem_data_i;
wire                   a0_bias_curr_mem_wr_o;
wire [`ADDR_SIZE-1:0]  a0_bias_curr_mem_wr_addr_o;
wire [31:0]            a0_bias_curr_mem_wr_data_o;

wire                   a0_thresh_mem_rd_o;
wire [`ADDR_SIZE-1:0]  a0_thresh_mem_addr_o;
wire [`WTD_BITS-1:0]   a0_thresh_mem_data_i;
wire                   a0_thresh_mem_wr_o;
wire [`ADDR_SIZE-1:0]  a0_thresh_mem_wr_addr_o;
wire [31:0]            a0_thresh_mem_wr_data_o;

wire                   a0_pot_mem_wr_o, a0_pot_mem_rd_o;
wire [`ADDR_SIZE-1:0]  a0_pot_mem_addr_o;
wire [`POT_BITS-1:0]   a0_pot_mem_data_o;
wire [`POT_BITS-1:0]   a0_pot_mem_data_i;

wire                   a0_spike_mem_wr_o;
wire [`ADDR_SIZE-1:0]  a0_spike_mem_addr_o;
wire [`ACT_BITS-1:0]   a0_spike_mem_data_o;

// ── Hadamard ─────────────────────────────────────────────────────────────────
wire                   hd_src_a_mem_rd_o;
wire [`ADDR_SIZE-1:0]  hd_src_a_mem_addr_o;
wire [31:0]            hd_src_a_mem_data_i;
wire                   hd_src_a_mem_wr_o;
wire [`ADDR_SIZE-1:0]  hd_src_a_mem_wr_addr_o;
wire [31:0]            hd_src_a_mem_wr_data_o;

wire                   hd_src_b_mem_rd_o;
wire [`ADDR_SIZE-1:0]  hd_src_b_mem_addr_o;
wire [31:0]            hd_src_b_mem_data_i;
wire                   hd_src_b_mem_wr_o;
wire [`ADDR_SIZE-1:0]  hd_src_b_mem_wr_addr_o;
wire [31:0]            hd_src_b_mem_wr_data_o;

wire                   hd_src_z_mem_rd_o;
wire [`ADDR_SIZE-1:0]  hd_src_z_mem_addr_o;
wire [31:0]            hd_src_z_mem_data_i;
wire                   hd_src_z_mem_wr_o;
wire [`ADDR_SIZE-1:0]  hd_src_z_mem_wr_addr_o;
wire [31:0]            hd_src_z_mem_wr_data_o;

wire                   hd_src_r_mem_rd_o;
wire [`ADDR_SIZE-1:0]  hd_src_r_mem_addr_o;
wire [31:0]            hd_src_r_mem_data_i;
wire                   hd_src_r_mem_wr_o;
wire [`ADDR_SIZE-1:0]  hd_src_r_mem_wr_addr_o;
wire [31:0]            hd_src_r_mem_data_o;

// ─── IOB output register intermediates ───────────────────────────────────────
// Internal wires carry flexman/BRAM outputs to the IOB register stage below.
wire        sys_ack_int;
wire [31:0] sys_data_int;
wire        nxt_input_pulse_int;
wire        nxt_output_pulse_int;
wire        cm_config_finished_int;
wire [`ACT_BITS-1:0] s0_spike_rd_data_int;
wire [`ACT_BITS-1:0] s1_spike_rd_data_int;
wire [`ACT_BITS-1:0] a0_spike_rd_data_int;

// ─── flexman instantiation ────────────────────────────────────────────────────
flexman #(
    .SNN0_CFG_BASE        (SNN0_CFG_BASE),
    .SNN1_CFG_BASE        (SNN1_CFG_BASE),
    .ANN_CFG_BASE         (ANN_CFG_BASE),
    .HAD_CFG_BASE         (HAD_CFG_BASE),
    .CM_CFG_MEM_ADDR      (CM_CFG_MEM_ADDR),
    .CM_CFG_MEM_MASK      (CM_CFG_MEM_MASK),
    .CM_BBA_MEM_ADDR      (CM_BBA_MEM_ADDR),
    .CM_BBA_MEM_MASK      (CM_BBA_MEM_MASK),
    .FU_TABLE_ADDR        (FU_TABLE_ADDR),
    .FU_TABLE_ADDR_MASK   (FU_TABLE_ADDR_MASK),
    .SCH_PROG_MEM_ADDR    (SCH_PROG_MEM_ADDR),
    .SCH_PROG_MEM_MASK    (SCH_PROG_MEM_MASK),
    .NUM_BUFFERS          (NUM_BUFFERS),
    .NUM_HW_ACCELERATORS  (NUM_HW_ACCELERATORS),
    .WORDS_PER_CONFIG     (WORDS_PER_CONFIG),
    .CFG_ID_SZ            (CFG_ID_SZ),
    .BUFF_INDX_SZ         (BUFF_INDX_SZ),
    .TGT_ACC_SZ           (TGT_ACC_SZ),
    .TGT_COUNT_SZ         (TGT_COUNT_SZ),
    .PROG_ADDR_BITS       (PROG_ADDR_BITS),
    .PROG_DATA_BITS       (PROG_DATA_BITS),
    .NUM_SCH_ENTRIES      (NUM_SCH_ENTRIES),
    .COL_BUFF_ID_SZ       (COL_BUFF_ID_SZ)
) u_flexman (
    .clk                        (clk),
    .reset                      (reset),
    .test_stall_pipe             (test_stall_pipe),

    // AXI
    .sys_req_i                  (sys_req_i),
    .sys_ack_o                  (sys_ack_int),
    .sys_addr_i                 (sys_addr_i),
    .sys_data_i                 (sys_data_i),
    .sys_data_o                 (sys_data_int),

    // Program memory
    .prog_mem_addr_o            (prog_mem_addr_o),
    .prog_mem_data_i            (prog_mem_data_i),
    .prog_mem_req_o             (prog_mem_req_o),
    .prog_mem_wait_i            (1'b0),
    .prog_mem_wr_o              (prog_mem_wr_o),
    .prog_mem_wr_addr_o         (prog_mem_wr_addr),
    .prog_mem_wr_data_o         (prog_mem_wr_data),
    .prog_mem_wr_wait_i         (1'b0),

    // Config memory
    .cfg_mem_rd_o               (cfg_mem_rd_o),
    .cfg_mem_wait_i             (1'b0),
    .cfg_mem_addr_o             (cfg_mem_addr_o),
    .cfg_mem_data_i             (cfg_mem_data_i),
    .cfg_mem_wr_o               (cfg_mem_wr_o),
    .cfg_mem_wr_addr_o          (cfg_mem_wr_addr_o),
    .cfg_mem_wr_data_o          (cfg_mem_wr_data_o),

    // BBA memory — config_manager port
    .bba_mem_rd_o               (bba_mem_rd_o),
    .bba_mem_wait_i             (1'b0),
    .bba_mem_addr_o             (bba_mem_addr_o),
    .bba_mem_data_i             (bba_mem_data_i),
    .bba_mem_wr_o               (bba_mem_wr_o),
    .bba_mem_wr_addr_o          (bba_mem_wr_addr_o),
    .bba_mem_wr_data_o          (bba_mem_wr_data_o),

    // BBA memory — fill_unit port
    .fu_bba_mem_rd_o            (fu_bba_mem_rd_o),
    .fu_bba_mem_wait_i          (1'b0),
    .fu_bba_mem_addr_o          (fu_bba_mem_addr_o),
    .fu_bba_mem_data_i          (fu_bba_mem_data_i),

    // Buffer / NXT
    .mark_buff_as_full_i        (mark_buff_as_full_i),
    .full_buff_id_i             (full_buff_id_i),
    .full_buff_usage_i          (full_buff_usage_i),
    .nxt_input_pulse_o          (nxt_input_pulse_int),
    .nxt_output_pulse_o         (nxt_output_pulse_int),
    .cm_config_finished_o       (cm_config_finished_int),

    // snnAcc0
    .s0_weight_mem_rd_o         (s0_weight_mem_rd_o),
    .s0_weight_mem_wait_i       (1'b0),
    .s0_weight_mem_addr_o       (s0_weight_mem_addr_o),
    .s0_weight_mem_data_i       (s0_weight_mem_data_i),
    .s0_weight_mem_wr_o         (s0_weight_mem_wr_o),
    .s0_weight_mem_wr_wait_i    (1'b0),
    .s0_weight_mem_wr_addr_o    (s0_weight_mem_wr_addr_o),
    .s0_weight_mem_wr_data_o    (s0_weight_mem_wr_data_o),

    .s0_act_mem_req_o           (s0_act_mem_req_o),
    .s0_act_mem_wait_i          (1'b0),
    .s0_act_mem_addr_o          (s0_act_mem_addr_o),
    .s0_act_mem_data_i          (s0_act_mem_data_i),
    .s0_act_mem_wr_o            (s0_act_mem_wr_o),
    .s0_act_mem_wr_wait_i       (1'b0),
    .s0_act_mem_wr_addr_o       (s0_act_mem_wr_addr_o),
    .s0_act_mem_wr_data_o       (s0_act_mem_wr_data_o),

    .s0_syn_curr_mem_wr_o       (s0_syn_curr_mem_wr_o),
    .s0_syn_curr_mem_rd_o       (s0_syn_curr_mem_rd_o),
    .s0_syn_curr_mem_wait_i     (1'b0),
    .s0_syn_curr_mem_addr_o     (s0_syn_curr_mem_addr_o),
    .s0_syn_curr_mem_data_o     (s0_syn_curr_mem_data_o),
    .s0_syn_curr_mem_data_i     (s0_syn_curr_mem_data_i),

    .s0_bias_curr_mem_rd_o      (s0_bias_curr_mem_rd_o),
    .s0_bias_curr_mem_wait_i    (1'b0),
    .s0_bias_curr_mem_addr_o    (s0_bias_curr_mem_addr_o),
    .s0_bias_curr_mem_data_i    (s0_bias_curr_mem_data_i),
    .s0_bias_curr_mem_wr_o      (s0_bias_curr_mem_wr_o),
    .s0_bias_curr_mem_wr_wait_i (1'b0),
    .s0_bias_curr_mem_wr_addr_o (s0_bias_curr_mem_wr_addr_o),
    .s0_bias_curr_mem_wr_data_o (s0_bias_curr_mem_wr_data_o),

    .s0_thresh_mem_rd_o         (s0_thresh_mem_rd_o),
    .s0_thresh_mem_wait_i       (1'b0),
    .s0_thresh_mem_addr_o       (s0_thresh_mem_addr_o),
    .s0_thresh_mem_data_i       (s0_thresh_mem_data_i),
    .s0_thresh_mem_wr_o         (s0_thresh_mem_wr_o),
    .s0_thresh_mem_wr_wait_i    (1'b0),
    .s0_thresh_mem_wr_addr_o    (s0_thresh_mem_wr_addr_o),
    .s0_thresh_mem_wr_data_o    (s0_thresh_mem_wr_data_o),

    .s0_pot_mem_wr_o            (s0_pot_mem_wr_o),
    .s0_pot_mem_rd_o            (s0_pot_mem_rd_o),
    .s0_pot_mem_wait_i          (1'b0),
    .s0_pot_mem_addr_o          (s0_pot_mem_addr_o),
    .s0_pot_mem_data_o          (s0_pot_mem_data_o),
    .s0_pot_mem_data_i          (s0_pot_mem_data_i),

    .s0_spike_mem_wr_o          (s0_spike_mem_wr_o),
    .s0_spike_mem_wait_i        (1'b0),
    .s0_spike_mem_addr_o        (s0_spike_mem_addr_o),
    .s0_spike_mem_data_o        (s0_spike_mem_data_o),

    // snnAcc1
    .s1_weight_mem_rd_o         (s1_weight_mem_rd_o),
    .s1_weight_mem_wait_i       (1'b0),
    .s1_weight_mem_addr_o       (s1_weight_mem_addr_o),
    .s1_weight_mem_data_i       (s1_weight_mem_data_i),
    .s1_weight_mem_wr_o         (s1_weight_mem_wr_o),
    .s1_weight_mem_wr_wait_i    (1'b0),
    .s1_weight_mem_wr_addr_o    (s1_weight_mem_wr_addr_o),
    .s1_weight_mem_wr_data_o    (s1_weight_mem_wr_data_o),

    .s1_act_mem_req_o           (s1_act_mem_req_o),
    .s1_act_mem_wait_i          (1'b0),
    .s1_act_mem_addr_o          (s1_act_mem_addr_o),
    .s1_act_mem_data_i          (s1_act_mem_data_i),
    .s1_act_mem_wr_o            (s1_act_mem_wr_o),
    .s1_act_mem_wr_wait_i       (1'b0),
    .s1_act_mem_wr_addr_o       (s1_act_mem_wr_addr_o),
    .s1_act_mem_wr_data_o       (s1_act_mem_wr_data_o),

    .s1_syn_curr_mem_wr_o       (s1_syn_curr_mem_wr_o),
    .s1_syn_curr_mem_rd_o       (s1_syn_curr_mem_rd_o),
    .s1_syn_curr_mem_wait_i     (1'b0),
    .s1_syn_curr_mem_addr_o     (s1_syn_curr_mem_addr_o),
    .s1_syn_curr_mem_data_o     (s1_syn_curr_mem_data_o),
    .s1_syn_curr_mem_data_i     (s1_syn_curr_mem_data_i),

    .s1_bias_curr_mem_rd_o      (s1_bias_curr_mem_rd_o),
    .s1_bias_curr_mem_wait_i    (1'b0),
    .s1_bias_curr_mem_addr_o    (s1_bias_curr_mem_addr_o),
    .s1_bias_curr_mem_data_i    (s1_bias_curr_mem_data_i),
    .s1_bias_curr_mem_wr_o      (s1_bias_curr_mem_wr_o),
    .s1_bias_curr_mem_wr_wait_i (1'b0),
    .s1_bias_curr_mem_wr_addr_o (s1_bias_curr_mem_wr_addr_o),
    .s1_bias_curr_mem_wr_data_o (s1_bias_curr_mem_wr_data_o),

    .s1_thresh_mem_rd_o         (s1_thresh_mem_rd_o),
    .s1_thresh_mem_wait_i       (1'b0),
    .s1_thresh_mem_addr_o       (s1_thresh_mem_addr_o),
    .s1_thresh_mem_data_i       (s1_thresh_mem_data_i),
    .s1_thresh_mem_wr_o         (s1_thresh_mem_wr_o),
    .s1_thresh_mem_wr_wait_i    (1'b0),
    .s1_thresh_mem_wr_addr_o    (s1_thresh_mem_wr_addr_o),
    .s1_thresh_mem_wr_data_o    (s1_thresh_mem_wr_data_o),

    .s1_pot_mem_wr_o            (s1_pot_mem_wr_o),
    .s1_pot_mem_rd_o            (s1_pot_mem_rd_o),
    .s1_pot_mem_wait_i          (1'b0),
    .s1_pot_mem_addr_o          (s1_pot_mem_addr_o),
    .s1_pot_mem_data_o          (s1_pot_mem_data_o),
    .s1_pot_mem_data_i          (s1_pot_mem_data_i),

    .s1_spike_mem_wr_o          (s1_spike_mem_wr_o),
    .s1_spike_mem_wait_i        (1'b0),
    .s1_spike_mem_addr_o        (s1_spike_mem_addr_o),
    .s1_spike_mem_data_o        (s1_spike_mem_data_o),

    // annAcc
    .a0_weight_mem_rd_o         (a0_weight_mem_rd_o),
    .a0_weight_mem_wait_i       (1'b0),
    .a0_weight_mem_addr_o       (a0_weight_mem_addr_o),
    .a0_weight_mem_data_i       (a0_weight_mem_data_i),
    .a0_weight_mem_wr_o         (a0_weight_mem_wr_o),
    .a0_weight_mem_wr_wait_i    (1'b0),
    .a0_weight_mem_wr_addr_o    (a0_weight_mem_wr_addr_o),
    .a0_weight_mem_wr_data_o    (a0_weight_mem_wr_data_o),

    .a0_act_mem_req_o           (a0_act_mem_req_o),
    .a0_act_mem_wait_i          (1'b0),
    .a0_act_mem_addr_o          (a0_act_mem_addr_o),
    .a0_act_mem_data_i          (a0_act_mem_data_i),
    .a0_act_mem_wr_o            (a0_act_mem_wr_o),
    .a0_act_mem_wr_wait_i       (1'b0),
    .a0_act_mem_wr_addr_o       (a0_act_mem_wr_addr_o),
    .a0_act_mem_wr_data_o       (a0_act_mem_wr_data_o),

    .a0_syn_curr_mem_wr_o       (a0_syn_curr_mem_wr_o),
    .a0_syn_curr_mem_rd_o       (a0_syn_curr_mem_rd_o),
    .a0_syn_curr_mem_wait_i     (1'b0),
    .a0_syn_curr_mem_addr_o     (a0_syn_curr_mem_addr_o),
    .a0_syn_curr_mem_data_o     (a0_syn_curr_mem_data_o),
    .a0_syn_curr_mem_data_i     (a0_syn_curr_mem_data_i),

    .a0_bias_curr_mem_rd_o      (a0_bias_curr_mem_rd_o),
    .a0_bias_curr_mem_wait_i    (1'b0),
    .a0_bias_curr_mem_addr_o    (a0_bias_curr_mem_addr_o),
    .a0_bias_curr_mem_data_i    (a0_bias_curr_mem_data_i),
    .a0_bias_curr_mem_wr_o      (a0_bias_curr_mem_wr_o),
    .a0_bias_curr_mem_wr_wait_i (1'b0),
    .a0_bias_curr_mem_wr_addr_o (a0_bias_curr_mem_wr_addr_o),
    .a0_bias_curr_mem_wr_data_o (a0_bias_curr_mem_wr_data_o),

    .a0_thresh_mem_rd_o         (a0_thresh_mem_rd_o),
    .a0_thresh_mem_wait_i       (1'b0),
    .a0_thresh_mem_addr_o       (a0_thresh_mem_addr_o),
    .a0_thresh_mem_data_i       (a0_thresh_mem_data_i),
    .a0_thresh_mem_wr_o         (a0_thresh_mem_wr_o),
    .a0_thresh_mem_wr_wait_i    (1'b0),
    .a0_thresh_mem_wr_addr_o    (a0_thresh_mem_wr_addr_o),
    .a0_thresh_mem_wr_data_o    (a0_thresh_mem_wr_data_o),

    .a0_pot_mem_wr_o            (a0_pot_mem_wr_o),
    .a0_pot_mem_rd_o            (a0_pot_mem_rd_o),
    .a0_pot_mem_wait_i          (1'b0),
    .a0_pot_mem_addr_o          (a0_pot_mem_addr_o),
    .a0_pot_mem_data_o          (a0_pot_mem_data_o),
    .a0_pot_mem_data_i          (a0_pot_mem_data_i),

    .a0_spike_mem_wr_o          (a0_spike_mem_wr_o),
    .a0_spike_mem_wait_i        (1'b0),
    .a0_spike_mem_addr_o        (a0_spike_mem_addr_o),
    .a0_spike_mem_data_o        (a0_spike_mem_data_o),

    // Hadamard
    .hd_src_a_mem_rd_o          (hd_src_a_mem_rd_o),
    .hd_src_a_mem_wait_i        (1'b0),
    .hd_src_a_mem_addr_o        (hd_src_a_mem_addr_o),
    .hd_src_a_mem_data_i        (hd_src_a_mem_data_i),
    .hd_src_a_mem_wr_o          (hd_src_a_mem_wr_o),
    .hd_src_a_mem_wr_wait_i     (1'b0),
    .hd_src_a_mem_wr_addr_o     (hd_src_a_mem_wr_addr_o),
    .hd_src_a_mem_wr_data_o     (hd_src_a_mem_wr_data_o),

    .hd_src_b_mem_rd_o          (hd_src_b_mem_rd_o),
    .hd_src_b_mem_wait_i        (1'b0),
    .hd_src_b_mem_addr_o        (hd_src_b_mem_addr_o),
    .hd_src_b_mem_data_i        (hd_src_b_mem_data_i),
    .hd_src_b_mem_wr_o          (hd_src_b_mem_wr_o),
    .hd_src_b_mem_wr_wait_i     (1'b0),
    .hd_src_b_mem_wr_addr_o     (hd_src_b_mem_wr_addr_o),
    .hd_src_b_mem_wr_data_o     (hd_src_b_mem_wr_data_o),

    .hd_src_z_mem_rd_o          (hd_src_z_mem_rd_o),
    .hd_src_z_mem_wait_i        (1'b0),
    .hd_src_z_mem_addr_o        (hd_src_z_mem_addr_o),
    .hd_src_z_mem_data_i        (hd_src_z_mem_data_i),
    .hd_src_z_mem_wr_o          (hd_src_z_mem_wr_o),
    .hd_src_z_mem_wr_wait_i     (1'b0),
    .hd_src_z_mem_wr_addr_o     (hd_src_z_mem_wr_addr_o),
    .hd_src_z_mem_wr_data_o     (hd_src_z_mem_wr_data_o),

    .hd_src_r_mem_rd_o          (hd_src_r_mem_rd_o),
    .hd_src_r_mem_wait_i        (1'b0),
    .hd_src_r_mem_addr_o        (hd_src_r_mem_addr_o),
    .hd_src_r_mem_data_i        (hd_src_r_mem_data_i),
    .hd_src_r_mem_wr_o          (hd_src_r_mem_wr_o),
    .hd_src_r_mem_wr_addr_o     (hd_src_r_mem_wr_addr_o),
    .hd_src_r_mem_wr_wait_i     (1'b0),
    .hd_src_r_mem_data_o        (hd_src_r_mem_data_o)
);

// ─── System memories ──────────────────────────────────────────────────────────

// Program memory: scheduler loads via AXI (prog_mem_wr_*), reads instructions.
bram_sdp #(.DEPTH(PROG_MEM_DEPTH), .DATA_W(PROG_DATA_BITS)) u_prog_mem (
    .clk   (clk),
    .we    (prog_mem_wr_o),
    .waddr (prog_mem_wr_addr),
    .din   (prog_mem_wr_data),
    .raddr (prog_mem_addr_o[PROG_ABITS-1:0]),
    .dout  (prog_mem_data_i)
);

// Config memory: config_manager reads and writes (separate rd/wr addr buses).
bram_sdp #(.DEPTH(CFG_MEM_DEPTH), .DATA_W(32)) u_cfg_mem (
    .clk   (clk),
    .we    (cfg_mem_wr_o),
    .waddr (cfg_mem_wr_addr_o[CFG_ABITS-1:0]),
    .din   (cfg_mem_wr_data_o),
    .raddr (cfg_mem_addr_o[CFG_ABITS-1:0]),
    .dout  (cfg_mem_data_i)
);

// BBA memory: config_manager R/W on port A, fill_unit read on port B.
bram_tdp #(.DEPTH(BBA_MEM_DEPTH), .DATA_W(32)) u_bba_mem (
    .clk   (clk),
    .ena   (bba_mem_rd_o | bba_mem_wr_o),
    .wea   (bba_mem_wr_o),
    .addra (bba_port_a_addr),
    .dina  (bba_mem_wr_data_o),
    .douta (bba_mem_data_i),
    .enb   (fu_bba_mem_rd_o),
    .web   (1'b0),
    .addrb (fu_bba_mem_addr_o[BBA_ABITS-1:0]),
    .dinb  (32'b0),
    .doutb (fu_bba_mem_data_i)
);

// ─── snnAcc0 memories ─────────────────────────────────────────────────────────

bram_sdp #(.DEPTH(ACC_MEM_DEPTH), .DATA_W(`WTD_BITS)) u_s0_weight_mem (
    .clk   (clk),
    .we    (s0_weight_mem_wr_o),
    .waddr (s0_weight_mem_wr_addr_o[ACC_ABITS-1:0]),
    .din   (s0_weight_mem_wr_data_o),
    .raddr (s0_weight_mem_addr_o[ACC_ABITS-1:0]),
    .dout  (s0_weight_mem_data_i)
);

bram_sdp #(.DEPTH(ACC_MEM_DEPTH), .DATA_W(`ACT_BITS)) u_s0_act_mem (
    .clk   (clk),
    .we    (s0_act_mem_wr_o),
    .waddr (s0_act_mem_wr_addr_o[ACC_ABITS-1:0]),
    .din   (s0_act_mem_wr_data_o),
    .raddr (s0_act_mem_addr_o[ACC_ABITS-1:0]),
    .dout  (s0_act_mem_data_i)
);

bram_sp #(.DEPTH(ACC_MEM_DEPTH), .DATA_W(`POT_BITS)) u_s0_syn_curr_mem (
    .clk  (clk),
    .we   (s0_syn_curr_mem_wr_o),
    .addr (s0_syn_curr_mem_addr_o[ACC_ABITS-1:0]),
    .din  (s0_syn_curr_mem_data_o),
    .dout (s0_syn_curr_mem_data_i)
);

bram_sdp #(.DEPTH(ACC_MEM_DEPTH), .DATA_W(`WTD_BITS)) u_s0_bias_curr_mem (
    .clk   (clk),
    .we    (s0_bias_curr_mem_wr_o),
    .waddr (s0_bias_curr_mem_wr_addr_o[ACC_ABITS-1:0]),
    .din   (s0_bias_curr_mem_wr_data_o),
    .raddr (s0_bias_curr_mem_addr_o[ACC_ABITS-1:0]),
    .dout  (s0_bias_curr_mem_data_i)
);

bram_sdp #(.DEPTH(ACC_MEM_DEPTH), .DATA_W(`WTD_BITS)) u_s0_thresh_mem (
    .clk   (clk),
    .we    (s0_thresh_mem_wr_o),
    .waddr (s0_thresh_mem_wr_addr_o[ACC_ABITS-1:0]),
    .din   (s0_thresh_mem_wr_data_o),
    .raddr (s0_thresh_mem_addr_o[ACC_ABITS-1:0]),
    .dout  (s0_thresh_mem_data_i)
);

bram_sp #(.DEPTH(ACC_MEM_DEPTH), .DATA_W(`POT_BITS)) u_s0_pot_mem (
    .clk  (clk),
    .we   (s0_pot_mem_wr_o),
    .addr (s0_pot_mem_addr_o[ACC_ABITS-1:0]),
    .din  (s0_pot_mem_data_o),
    .dout (s0_pot_mem_data_i)
);

// Spike memory: acc writes on waddr, host reads on raddr.
bram_sdp #(.DEPTH(ACC_MEM_DEPTH), .DATA_W(`ACT_BITS)) u_s0_spike_mem (
    .clk   (clk),
    .we    (s0_spike_mem_wr_o),
    .waddr (s0_spike_mem_addr_o[ACC_ABITS-1:0]),
    .din   (s0_spike_mem_data_o),
    .raddr (s0_spike_rd_addr_i),
    .dout  (s0_spike_rd_data_int)
);

// ─── snnAcc1 memories ─────────────────────────────────────────────────────────

bram_sdp #(.DEPTH(ACC_MEM_DEPTH), .DATA_W(`WTD_BITS)) u_s1_weight_mem (
    .clk   (clk),
    .we    (s1_weight_mem_wr_o),
    .waddr (s1_weight_mem_wr_addr_o[ACC_ABITS-1:0]),
    .din   (s1_weight_mem_wr_data_o),
    .raddr (s1_weight_mem_addr_o[ACC_ABITS-1:0]),
    .dout  (s1_weight_mem_data_i)
);

bram_sdp #(.DEPTH(ACC_MEM_DEPTH), .DATA_W(`ACT_BITS)) u_s1_act_mem (
    .clk   (clk),
    .we    (s1_act_mem_wr_o),
    .waddr (s1_act_mem_wr_addr_o[ACC_ABITS-1:0]),
    .din   (s1_act_mem_wr_data_o),
    .raddr (s1_act_mem_addr_o[ACC_ABITS-1:0]),
    .dout  (s1_act_mem_data_i)
);

bram_sp #(.DEPTH(ACC_MEM_DEPTH), .DATA_W(`POT_BITS)) u_s1_syn_curr_mem (
    .clk  (clk),
    .we   (s1_syn_curr_mem_wr_o),
    .addr (s1_syn_curr_mem_addr_o[ACC_ABITS-1:0]),
    .din  (s1_syn_curr_mem_data_o),
    .dout (s1_syn_curr_mem_data_i)
);

bram_sdp #(.DEPTH(ACC_MEM_DEPTH), .DATA_W(`WTD_BITS)) u_s1_bias_curr_mem (
    .clk   (clk),
    .we    (s1_bias_curr_mem_wr_o),
    .waddr (s1_bias_curr_mem_wr_addr_o[ACC_ABITS-1:0]),
    .din   (s1_bias_curr_mem_wr_data_o),
    .raddr (s1_bias_curr_mem_addr_o[ACC_ABITS-1:0]),
    .dout  (s1_bias_curr_mem_data_i)
);

bram_sdp #(.DEPTH(ACC_MEM_DEPTH), .DATA_W(`WTD_BITS)) u_s1_thresh_mem (
    .clk   (clk),
    .we    (s1_thresh_mem_wr_o),
    .waddr (s1_thresh_mem_wr_addr_o[ACC_ABITS-1:0]),
    .din   (s1_thresh_mem_wr_data_o),
    .raddr (s1_thresh_mem_addr_o[ACC_ABITS-1:0]),
    .dout  (s1_thresh_mem_data_i)
);

bram_sp #(.DEPTH(ACC_MEM_DEPTH), .DATA_W(`POT_BITS)) u_s1_pot_mem (
    .clk  (clk),
    .we   (s1_pot_mem_wr_o),
    .addr (s1_pot_mem_addr_o[ACC_ABITS-1:0]),
    .din  (s1_pot_mem_data_o),
    .dout (s1_pot_mem_data_i)
);

bram_sdp #(.DEPTH(ACC_MEM_DEPTH), .DATA_W(`ACT_BITS)) u_s1_spike_mem (
    .clk   (clk),
    .we    (s1_spike_mem_wr_o),
    .waddr (s1_spike_mem_addr_o[ACC_ABITS-1:0]),
    .din   (s1_spike_mem_data_o),
    .raddr (s1_spike_rd_addr_i),
    .dout  (s1_spike_rd_data_int)
);

// ─── annAcc memories ──────────────────────────────────────────────────────────

bram_sdp #(.DEPTH(ACC_MEM_DEPTH), .DATA_W(`WTD_BITS)) u_a0_weight_mem (
    .clk   (clk),
    .we    (a0_weight_mem_wr_o),
    .waddr (a0_weight_mem_wr_addr_o[ACC_ABITS-1:0]),
    .din   (a0_weight_mem_wr_data_o),
    .raddr (a0_weight_mem_addr_o[ACC_ABITS-1:0]),
    .dout  (a0_weight_mem_data_i)
);

bram_sdp #(.DEPTH(ACC_MEM_DEPTH), .DATA_W(`ACT_BITS)) u_a0_act_mem (
    .clk   (clk),
    .we    (a0_act_mem_wr_o),
    .waddr (a0_act_mem_wr_addr_o[ACC_ABITS-1:0]),
    .din   (a0_act_mem_wr_data_o),
    .raddr (a0_act_mem_addr_o[ACC_ABITS-1:0]),
    .dout  (a0_act_mem_data_i)
);

bram_sp #(.DEPTH(ACC_MEM_DEPTH), .DATA_W(`POT_BITS)) u_a0_syn_curr_mem (
    .clk  (clk),
    .we   (a0_syn_curr_mem_wr_o),
    .addr (a0_syn_curr_mem_addr_o[ACC_ABITS-1:0]),
    .din  (a0_syn_curr_mem_data_o),
    .dout (a0_syn_curr_mem_data_i)
);

bram_sdp #(.DEPTH(ACC_MEM_DEPTH), .DATA_W(`WTD_BITS)) u_a0_bias_curr_mem (
    .clk   (clk),
    .we    (a0_bias_curr_mem_wr_o),
    .waddr (a0_bias_curr_mem_wr_addr_o[ACC_ABITS-1:0]),
    .din   (a0_bias_curr_mem_wr_data_o),
    .raddr (a0_bias_curr_mem_addr_o[ACC_ABITS-1:0]),
    .dout  (a0_bias_curr_mem_data_i)
);

bram_sdp #(.DEPTH(ACC_MEM_DEPTH), .DATA_W(`WTD_BITS)) u_a0_thresh_mem (
    .clk   (clk),
    .we    (a0_thresh_mem_wr_o),
    .waddr (a0_thresh_mem_wr_addr_o[ACC_ABITS-1:0]),
    .din   (a0_thresh_mem_wr_data_o),
    .raddr (a0_thresh_mem_addr_o[ACC_ABITS-1:0]),
    .dout  (a0_thresh_mem_data_i)
);

bram_sp #(.DEPTH(ACC_MEM_DEPTH), .DATA_W(`POT_BITS)) u_a0_pot_mem (
    .clk  (clk),
    .we   (a0_pot_mem_wr_o),
    .addr (a0_pot_mem_addr_o[ACC_ABITS-1:0]),
    .din  (a0_pot_mem_data_o),
    .dout (a0_pot_mem_data_i)
);

bram_sdp #(.DEPTH(ACC_MEM_DEPTH), .DATA_W(`ACT_BITS)) u_a0_spike_mem (
    .clk   (clk),
    .we    (a0_spike_mem_wr_o),
    .waddr (a0_spike_mem_addr_o[ACC_ABITS-1:0]),
    .din   (a0_spike_mem_data_o),
    .raddr (a0_spike_rd_addr_i),
    .dout  (a0_spike_rd_data_int)
);

// ─── Hadamard memories ────────────────────────────────────────────────────────

// src_a, src_b, src_z: acc reads, fill_unit writes (separate port buses).
bram_sdp #(.DEPTH(ACC_MEM_DEPTH), .DATA_W(32)) u_hd_src_a_mem (
    .clk   (clk),
    .we    (hd_src_a_mem_wr_o),
    .waddr (hd_src_a_mem_wr_addr_o[ACC_ABITS-1:0]),
    .din   (hd_src_a_mem_wr_data_o),
    .raddr (hd_src_a_mem_addr_o[ACC_ABITS-1:0]),
    .dout  (hd_src_a_mem_data_i)
);

bram_sdp #(.DEPTH(ACC_MEM_DEPTH), .DATA_W(32)) u_hd_src_b_mem (
    .clk   (clk),
    .we    (hd_src_b_mem_wr_o),
    .waddr (hd_src_b_mem_wr_addr_o[ACC_ABITS-1:0]),
    .din   (hd_src_b_mem_wr_data_o),
    .raddr (hd_src_b_mem_addr_o[ACC_ABITS-1:0]),
    .dout  (hd_src_b_mem_data_i)
);

bram_sdp #(.DEPTH(ACC_MEM_DEPTH), .DATA_W(32)) u_hd_src_z_mem (
    .clk   (clk),
    .we    (hd_src_z_mem_wr_o),
    .waddr (hd_src_z_mem_wr_addr_o[ACC_ABITS-1:0]),
    .din   (hd_src_z_mem_wr_data_o),
    .raddr (hd_src_z_mem_addr_o[ACC_ABITS-1:0]),
    .dout  (hd_src_z_mem_data_i)
);

// src_r: arbitrated read+write bus (separate rd/wr addr buses post-arbitration).
bram_sdp #(.DEPTH(ACC_MEM_DEPTH), .DATA_W(32)) u_hd_src_r_mem (
    .clk   (clk),
    .we    (hd_src_r_mem_wr_o),
    .waddr (hd_src_r_mem_wr_addr_o[ACC_ABITS-1:0]),
    .din   (hd_src_r_mem_data_o),
    .raddr (hd_src_r_mem_addr_o[ACC_ABITS-1:0]),
    .dout  (hd_src_r_mem_data_i)
);

// ─── IOB output registers ─────────────────────────────────────────────────────
// (* IOB = "TRUE" *) places each FF in the physical I/O block so that
// set_output_delay constrains only the IOB-FF → output-buffer path (~1 ns).
// The fabric-to-IOB-FF path is analysed as a register-to-register path against
// the full clock period.  All outputs have +1 cycle latency at the pin.
// Spike readback data has 2 cycles total (BRAM registered read + this stage).
(* IOB = "TRUE" *) reg        sys_ack_r;
(* IOB = "TRUE" *) reg [31:0] sys_data_r;
(* IOB = "TRUE" *) reg        nxt_input_pulse_r;
(* IOB = "TRUE" *) reg        nxt_output_pulse_r;
(* IOB = "TRUE" *) reg        cm_config_finished_r;
(* IOB = "TRUE" *) reg [`ACT_BITS-1:0] s0_spike_rd_data_r;
(* IOB = "TRUE" *) reg [`ACT_BITS-1:0] s1_spike_rd_data_r;
(* IOB = "TRUE" *) reg [`ACT_BITS-1:0] a0_spike_rd_data_r;

always @(posedge clk) begin
    sys_ack_r            <= sys_ack_int;
    sys_data_r           <= sys_data_int;
    nxt_input_pulse_r    <= nxt_input_pulse_int;
    nxt_output_pulse_r   <= nxt_output_pulse_int;
    cm_config_finished_r <= cm_config_finished_int;
    s0_spike_rd_data_r   <= s0_spike_rd_data_int;
    s1_spike_rd_data_r   <= s1_spike_rd_data_int;
    a0_spike_rd_data_r   <= a0_spike_rd_data_int;
end

assign sys_ack_o            = sys_ack_r;
assign sys_data_o           = sys_data_r;
assign nxt_input_pulse_o    = nxt_input_pulse_r;
assign nxt_output_pulse_o   = nxt_output_pulse_r;
assign cm_config_finished_o = cm_config_finished_r;
assign s0_spike_rd_data_o   = s0_spike_rd_data_r;
assign s1_spike_rd_data_o   = s1_spike_rd_data_r;
assign a0_spike_rd_data_o   = a0_spike_rd_data_r;

endmodule
