// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
`timescale 10ps/1ps
`include "../shared/constants.v"

// Top-level integration module for FlexMan.
// Instantiates: 1× scheduler, 1× config_manager, 2× snnAcc, 1× annAcc,
// 1× hadamard_unit, 1× fill_unit.
//
// AXI bus sharing:
//   A single host AXI bus (sys_req_i/sys_addr_i/sys_data_i) is broadcast to
//   all sub-modules.  Each sub-module self-filters by address.  Acks are OR'd.
//   Read data (sys_data_o) comes from the scheduler (status registers) or,
//   for the POOL_RD_BASE window, from the shared-pool readback path below.
//
// Host pool readback (POOL_RD_BASE window, read-only):
//   A memory-mapped window lets the host read the shared pool back out (e.g.
//   output activations after a run).  An access in the window issues ONE pool
//   read requester at the LOWEST arbiter priority, so it never disturbs compute
//   (it just waits for a free bank cycle).  Reads are VARIABLE LATENCY: the host
//   must hold sys_req_i until sys_ack_o.  For prompt, race-free readback the host
//   should quiesce compute first:
//     PAUSE (sched ctrl reg 3) -> poll status reg 1 until acc_busy==0
//       -> read POOL_RD_BASE+(word<<2) ...  -> UNPAUSE (sched ctrl reg 4).
//
// Config write adapter (per accelerator):
//   config_manager outputs cm_config_wr_o[i] + cm_config_data_o with no address.
//   An internal word counter per accelerator generates the AXI write address
//   (BASE + word_cnt*4), with config_manager writes taking priority over the
//   external host for each accelerator's AXI port.
//
// BBA register bank (per accelerator):
//   config_manager sends 4 buffer base-addresses sequentially:
//   [0]=tgt, [1]=src1, [2]=src2, [3]=src3.
//   An internal counter captures each into a 32-bit register and drives the
//   accelerator's sp_*/np_*_buff_addr_i ports (lower PIN_BITS bits).
//
// fill_unit (acc_id = FILL_ACC_ID = NUM_HW_ACCELERATORS-1 = 4):
//   FILL dispatches are gated away from the computation accelerators and
//   config_manager.  fill_unit gets its own start_new_block pulse.
//   Memory write arbitration: accelerator wins over fill_unit on shared buses
//   (syn_curr, pot, spike, hd_src_r write port).  Read-only-from-acc memories
//   (weight, act, bias_curr, thresh, hd_src_a/b/z) expose dedicated write
//   ports at the top level for fill_unit (dual-port SRAM model).

module flexman #(
    // AXI config base addresses for each accelerator.
    // Address decode: sys_addr_i[31:16] == XYZW_CFG_BASE[31:16].
    parameter [31:0] SNN0_CFG_BASE           = 32'h1000_0000,
    parameter [31:0] SNN1_CFG_BASE           = 32'h1001_0000,
    parameter [31:0] ANN_CFG_BASE            = 32'h1002_0000,
    parameter [31:0] HAD_CFG_BASE            = 32'h1003_0000,
    // Host pool-readback window — read-only, memory-mapped view of the shared
    // pool.  Decoded on sys_addr_i[31:20] == POOL_RD_BASE[31:20] (1 MB / 256K
    // words).  Clear of the accelerator windows (0x100x) and CM/BBA/FU/SCH.
    parameter [31:0] POOL_RD_BASE            = 32'h1010_0000,
    // config_manager AXI slave address ranges (must not overlap with above).
    parameter [31:0] CM_CFG_MEM_ADDR         = 32'hA000_0000,
    parameter [31:0] CM_CFG_MEM_MASK         = 32'hFF00_0000,
    parameter [31:0] CM_BBA_MEM_ADDR         = 32'hB000_0000,
    parameter [31:0] CM_BBA_MEM_MASK         = 32'hFF00_0000,
    // fill_unit AXI slave address range for mem_sel table.
    parameter [31:0] FU_TABLE_ADDR           = 32'hC000_0000,
    parameter [31:0] FU_TABLE_ADDR_MASK      = 32'hFF00_0000,
    // scheduler AXI address range for program memory writes.
    parameter [31:0] SCH_PROG_MEM_ADDR       = 32'hD000_0000,
    parameter [31:0] SCH_PROG_MEM_MASK       = 32'hFF00_0000,
    // System sizing — must be consistent across all sub-modules.
    parameter NUM_BUFFERS         = 16,
    parameter NUM_HW_ACCELERATORS = 5,    // 4 computation + 1 fill_unit
    parameter WORDS_PER_CONFIG    = 16,   // must be a power of 2 >= 2; 16 fits
                                          // the snnAcc per-task config (~16 regs)
    parameter CFG_ID_SZ           = 7,
    parameter BUFF_INDX_SZ        = 4,    // = $clog2(NUM_BUFFERS)
    parameter TGT_ACC_SZ          = 3,    // extended to hold FILL_ACC_ID=4
    parameter TGT_COUNT_SZ        = 4,    // usage count up to 15 (4-bit ntgt field)
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

    // ── Program memory (scheduler read + AXI write) ─────────────────────────
    output wire [`ADDR_SIZE-1:0]     prog_mem_addr_o,
    input  wire [PROG_DATA_BITS-1:0] prog_mem_data_i,
    output wire                      prog_mem_req_o,
    input  wire                      prog_mem_wait_i,
    output wire                      prog_mem_wr_o,
    output wire [PROG_ADDR_BITS-1:0] prog_mem_wr_addr_o,
    output wire [PROG_DATA_BITS-1:0] prog_mem_wr_data_o,
    input  wire                      prog_mem_wr_wait_i,

    // ── Config memory (config_manager read/write) ────────────────────────────
    output wire         cfg_mem_rd_o,
    input  wire         cfg_mem_wait_i,
    output wire  [31:0] cfg_mem_addr_o,
    input  wire  [31:0] cfg_mem_data_i,
    output wire         cfg_mem_wr_o,
    output wire  [31:0] cfg_mem_wr_addr_o,
    output wire  [31:0] cfg_mem_wr_data_o,

    // ── Buffer base-address memory — config_manager port (read/write) ────────
    output wire         bba_mem_rd_o,
    input  wire         bba_mem_wait_i,
    output wire  [31:0] bba_mem_addr_o,
    input  wire  [31:0] bba_mem_data_i,
    output wire         bba_mem_wr_o,
    output wire  [31:0] bba_mem_wr_addr_o,
    output wire  [31:0] bba_mem_wr_data_o,

    // ── Buffer base-address memory — fill_unit second read port ─────────────
    output wire         fu_bba_mem_rd_o,
    input  wire         fu_bba_mem_wait_i,
    output wire  [31:0] fu_bba_mem_addr_o,
    input  wire  [31:0] fu_bba_mem_data_i,

    // ── Scheduler NXT pulses ─────────────────────────────────────────────────
    output wire                   nxt_input_pulse_o,
    output wire                   nxt_output_pulse_o,

    // ── Config manager status ────────────────────────────────────────────────
    output wire cm_config_finished_o,

    // ── snnAcc0 memory ports (prefix s0_) ────────────────────────────────────
    // Existing accelerator read/write ports (acc priority on shared buses)
    output wire                  s0_weight_mem_rd_o,
    input  wire                  s0_weight_mem_wait_i,
    output wire [`ADDR_SIZE-1:0] s0_weight_mem_addr_o,
    input  wire  [`WTD_BITS-1:0] s0_weight_mem_data_i,
    // fill_unit dedicated write port (dual-port; no arbitration needed)
    output wire                  s0_weight_mem_wr_o,
    input  wire                  s0_weight_mem_wr_wait_i,
    output wire [`ADDR_SIZE-1:0] s0_weight_mem_wr_addr_o,
    output wire           [31:0] s0_weight_mem_wr_data_o,

    // act + syn_curr moved to the shared m0..m3 data pool (ports at end).
    output wire                  s0_bias_curr_mem_rd_o,
    input  wire                  s0_bias_curr_mem_wait_i,
    output wire [`ADDR_SIZE-1:0] s0_bias_curr_mem_addr_o,
    input  wire  [`WTD_BITS-1:0] s0_bias_curr_mem_data_i,
    output wire                  s0_bias_curr_mem_wr_o,
    input  wire                  s0_bias_curr_mem_wr_wait_i,
    output wire [`ADDR_SIZE-1:0] s0_bias_curr_mem_wr_addr_o,
    output wire           [31:0] s0_bias_curr_mem_wr_data_o,

    output wire                  s0_thresh_mem_rd_o,
    input  wire                  s0_thresh_mem_wait_i,
    output wire [`ADDR_SIZE-1:0] s0_thresh_mem_addr_o,
    input  wire  [`WTD_BITS-1:0] s0_thresh_mem_data_i,
    output wire                  s0_thresh_mem_wr_o,
    input  wire                  s0_thresh_mem_wr_wait_i,
    output wire [`ADDR_SIZE-1:0] s0_thresh_mem_wr_addr_o,
    output wire           [31:0] s0_thresh_mem_wr_data_o,

    // Shared read/write bus (arbitrated: acc wins)
    output wire                  s0_pot_mem_wr_o,
    output wire                  s0_pot_mem_rd_o,
    input  wire                  s0_pot_mem_wait_i,
    output wire [`ADDR_SIZE-1:0] s0_pot_mem_addr_o,
    output wire  [`POT_BITS-1:0] s0_pot_mem_data_o,
    input  wire  [`POT_BITS-1:0] s0_pot_mem_data_i,

    // ── snnAcc1 memory ports (prefix s1_) ────────────────────────────────────
    output wire                  s1_weight_mem_rd_o,
    input  wire                  s1_weight_mem_wait_i,
    output wire [`ADDR_SIZE-1:0] s1_weight_mem_addr_o,
    input  wire  [`WTD_BITS-1:0] s1_weight_mem_data_i,
    output wire                  s1_weight_mem_wr_o,
    input  wire                  s1_weight_mem_wr_wait_i,
    output wire [`ADDR_SIZE-1:0] s1_weight_mem_wr_addr_o,
    output wire           [31:0] s1_weight_mem_wr_data_o,

    // act + syn_curr moved to the shared m0..m3 data pool.
    output wire                  s1_bias_curr_mem_rd_o,
    input  wire                  s1_bias_curr_mem_wait_i,
    output wire [`ADDR_SIZE-1:0] s1_bias_curr_mem_addr_o,
    input  wire  [`WTD_BITS-1:0] s1_bias_curr_mem_data_i,
    output wire                  s1_bias_curr_mem_wr_o,
    input  wire                  s1_bias_curr_mem_wr_wait_i,
    output wire [`ADDR_SIZE-1:0] s1_bias_curr_mem_wr_addr_o,
    output wire           [31:0] s1_bias_curr_mem_wr_data_o,

    output wire                  s1_thresh_mem_rd_o,
    input  wire                  s1_thresh_mem_wait_i,
    output wire [`ADDR_SIZE-1:0] s1_thresh_mem_addr_o,
    input  wire  [`WTD_BITS-1:0] s1_thresh_mem_data_i,
    output wire                  s1_thresh_mem_wr_o,
    input  wire                  s1_thresh_mem_wr_wait_i,
    output wire [`ADDR_SIZE-1:0] s1_thresh_mem_wr_addr_o,
    output wire           [31:0] s1_thresh_mem_wr_data_o,

    output wire                  s1_pot_mem_wr_o,
    output wire                  s1_pot_mem_rd_o,
    input  wire                  s1_pot_mem_wait_i,
    output wire [`ADDR_SIZE-1:0] s1_pot_mem_addr_o,
    output wire  [`POT_BITS-1:0] s1_pot_mem_data_o,
    input  wire  [`POT_BITS-1:0] s1_pot_mem_data_i,

    // ── annAcc memory ports (prefix a0_) ─────────────────────────────────────
    output wire                  a0_weight_mem_rd_o,
    input  wire                  a0_weight_mem_wait_i,
    output wire [`ADDR_SIZE-1:0] a0_weight_mem_addr_o,
    input  wire  [`WTD_BITS-1:0] a0_weight_mem_data_i,
    output wire                  a0_weight_mem_wr_o,
    input  wire                  a0_weight_mem_wr_wait_i,
    output wire [`ADDR_SIZE-1:0] a0_weight_mem_wr_addr_o,
    output wire           [31:0] a0_weight_mem_wr_data_o,

    // act + syn_curr moved to the shared m0..m3 data pool.
    output wire                  a0_bias_curr_mem_rd_o,
    input  wire                  a0_bias_curr_mem_wait_i,
    output wire [`ADDR_SIZE-1:0] a0_bias_curr_mem_addr_o,
    input  wire  [`WTD_BITS-1:0] a0_bias_curr_mem_data_i,
    output wire                  a0_bias_curr_mem_wr_o,
    input  wire                  a0_bias_curr_mem_wr_wait_i,
    output wire [`ADDR_SIZE-1:0] a0_bias_curr_mem_wr_addr_o,
    output wire           [31:0] a0_bias_curr_mem_wr_data_o,

    output wire                  a0_thresh_mem_rd_o,
    input  wire                  a0_thresh_mem_wait_i,
    output wire [`ADDR_SIZE-1:0] a0_thresh_mem_addr_o,
    input  wire  [`WTD_BITS-1:0] a0_thresh_mem_data_i,
    output wire                  a0_thresh_mem_wr_o,
    input  wire                  a0_thresh_mem_wr_wait_i,
    output wire [`ADDR_SIZE-1:0] a0_thresh_mem_wr_addr_o,
    output wire           [31:0] a0_thresh_mem_wr_data_o,

    output wire                  a0_pot_mem_wr_o,
    output wire                  a0_pot_mem_rd_o,
    input  wire                  a0_pot_mem_wait_i,
    output wire [`ADDR_SIZE-1:0] a0_pot_mem_addr_o,
    output wire  [`POT_BITS-1:0] a0_pot_mem_data_o,
    input  wire  [`POT_BITS-1:0] a0_pot_mem_data_i,

    // ── Shared act/spike/syn_curr + Hadamard data pool (4 interleaved banks) ──
    // act/spike/syn_curr for snn0/snn1/ann AND Hadamard's src_a/b/z (read) +
    // src_r (read/write) are striped across these four single-port banks by the
    // low 2 address bits:  logical addr[9:0] = { word[7:0], bank_sel[1:0] }.
    // The top emits the bank-local word (logical >> 2) on *_addr_o.  Single-port
    // banks, arbitrated by the shared_pool instance below.  pot / weight /
    // bias_curr / thresh stay dedicated.
    output wire                  m0_data_mem_rd_o,
    output wire                  m0_data_mem_wr_o,
    input  wire                  m0_data_mem_wait_i,
    output wire [`ADDR_SIZE-1:0] m0_data_mem_addr_o,
    output wire           [31:0] m0_data_mem_wdata_o,
    input  wire           [31:0] m0_data_mem_rdata_i,

    output wire                  m1_data_mem_rd_o,
    output wire                  m1_data_mem_wr_o,
    input  wire                  m1_data_mem_wait_i,
    output wire [`ADDR_SIZE-1:0] m1_data_mem_addr_o,
    output wire           [31:0] m1_data_mem_wdata_o,
    input  wire           [31:0] m1_data_mem_rdata_i,

    output wire                  m2_data_mem_rd_o,
    output wire                  m2_data_mem_wr_o,
    input  wire                  m2_data_mem_wait_i,
    output wire [`ADDR_SIZE-1:0] m2_data_mem_addr_o,
    output wire           [31:0] m2_data_mem_wdata_o,
    input  wire           [31:0] m2_data_mem_rdata_i,

    output wire                  m3_data_mem_rd_o,
    output wire                  m3_data_mem_wr_o,
    input  wire                  m3_data_mem_wait_i,
    output wire [`ADDR_SIZE-1:0] m3_data_mem_addr_o,
    output wire           [31:0] m3_data_mem_wdata_o,
    input  wire           [31:0] m3_data_mem_rdata_i
);

// ─── Derived constants ────────────────────────────────────────────────────────
localparam MODE_SZ       = 2;
localparam SLOT_SHORT_SZ = MODE_SZ + BUFF_INDX_SZ;                            // 6
localparam SLOT_LONG_SZ  = MODE_SZ + BUFF_INDX_SZ + TGT_COUNT_SZ;            // 10 (TGT_COUNT_SZ=4)
// Entry layout: 3 short + 3 long + colour + acc_id + cfg_id
localparam ENTRY_DATA_SZ = 3*SLOT_SHORT_SZ + 3*SLOT_LONG_SZ
                           + 1 + TGT_ACC_SZ + CFG_ID_SZ;                      // 59 (CFG_ID_SZ=7)
localparam LONG_BASE     = 3 * SLOT_SHORT_SZ;                                 // 18
// Field offsets within buffer_info_o (matches scheduler internals). Positions
// MOVE with TGT_COUNT_SZ (via SLOT_LONG_SZ) — the values below are for the
// default TGT_COUNT_SZ=4. NEVER hardcode a slice from this table (hardcoded
// TGT_COUNT_SZ=3 offsets hid in cm_buffer_info + the TB TRACE hooks, fixed
// 2026-07-10):
//   Short slot 0: mode=[1:0],   id=[5:2]
//   Long  slot 0: mode=[19:18], id=[23:20], ntgt=[27:24]   (BASE=18)
//   Long  slot 1: mode=[29:28], id=[33:30], ntgt=[37:34]   (BASE=28)
//   Long  slot 2: mode=[39:38], id=[43:40], ntgt=[47:44]   (BASE=38)
//   colour=[48], acc_id=[51:49], cfg_id=[58:52] (7-bit)
localparam E_COLOUR    = 3*SLOT_SHORT_SZ + 3*SLOT_LONG_SZ;                    // 48
localparam E_ACC_START = E_COLOUR + 1;                                         // 49
localparam E_CFG_START = E_ACC_START + TGT_ACC_SZ;                            // 52
// Long-slot buffer-id offsets (cm_buffer_info below) — derived, not hardcoded:
localparam LS0_ID_START = LONG_BASE                  + MODE_SZ;               // 20
localparam LS1_ID_START = LONG_BASE + 1*SLOT_LONG_SZ + MODE_SZ;               // 30
localparam LS2_ID_START = LONG_BASE + 2*SLOT_LONG_SZ + MODE_SZ;               // 40

// fill_unit is the last accelerator slot; TASK's 2-bit acc_id field can only
// reach 0-3, so FILL_ACC_ID=4 is unreachable by normal TASK instructions.
localparam [TGT_ACC_SZ-1:0] FILL_ACC_ID = NUM_HW_ACCELERATORS - 1;           // 4

localparam LOG2_WPC    = 4;   // $clog2(WORDS_PER_CONFIG=16)
localparam BBA_CNT_SZ  = 2;   // $clog2(NUM_BBA_SENDS=4)

// Number of computation accelerators served by config_manager (excludes fill_unit)
localparam NUM_COMP_ACC = NUM_HW_ACCELERATORS - 1;                            // 4

// ─── Scheduler outputs ───────────────────────────────────────────────────────
wire                      sch_ack;
wire               [31:0] sch_data_o;
wire [ENTRY_DATA_SZ-1:0]  sch_buffer_info;
wire [TGT_ACC_SZ-1:0]     sch_target_acc;
wire                      sch_start_new_block;
wire               [31:0] sch_fill_value;
wire               [19:0] sch_fill_block_size;

// FILL dispatch gating — separates fill_unit dispatch from computation acc dispatch
wire is_fill_dispatch    = sch_start_new_block & (sch_target_acc == FILL_ACC_ID);
// Compute accelerators start AFTER config_manager has finished pushing their
// per-task configs. cm_config_finished_o only fires for non-FILL dispatches
// so no extra FILL gate is needed. cm_tgt_acc carries the latched target_acc
// from config_manager (stable through the whole dispatch lifecycle).
wire                       acc_start_new_block;       // assigned after cm_tgt_acc declared
localparam CM_TGT_ACC_SZ = 2;                         // = $clog2(NUM_COMP_ACC=4); config_manager's port width
wire [CM_TGT_ACC_SZ-1:0]   cm_tgt_acc_narrow;         // raw output from config_manager
wire [TGT_ACC_SZ-1:0]      cm_tgt_acc =
    {{(TGT_ACC_SZ - CM_TGT_ACC_SZ){1'b0}}, cm_tgt_acc_narrow};   // zero-extend to TGT_ACC_SZ

// Target buffer ID for fill_unit (long slot 0 id = bits [LONG_BASE+MODE_SZ +: BUFF_INDX_SZ])
wire [BUFF_INDX_SZ-1:0] fu_buff_id = sch_buffer_info[LONG_BASE + MODE_SZ +: BUFF_INDX_SZ];

// ─── Config manager outputs ───────────────────────────────────────────────────
wire                         cm_ack;
wire [NUM_COMP_ACC-1:0]      cm_config_wr;
wire [NUM_COMP_ACC-1:0]      cm_buff_base_wr;
wire [31:0]                  cm_config_data;
wire [31:0]                  cm_buff_base_data;
// Back-pressure: high while config_manager is mid-push. Scheduler stalls
// dispatch on this so a second start_new_block_i pulse isn't lost.
wire                         cm_busy;

// config_manager start is suppressed for FILL dispatches
wire cm_start_new_block = sch_start_new_block & ~is_fill_dispatch;

// Compute-acc dispatch fires when config_manager finishes pushing the
// cfg + bba streams. cm_tgt_acc was latched at the scheduler's dispatch
// pulse; it remains stable through this run until the next dispatch.
assign acc_start_new_block = cm_config_finished_o;

// ─── Accelerator AXI (config write mux outputs) ───────────────────────────────
wire  snn0_sys_req, snn0_sys_ack;
wire [31:0] snn0_sys_addr, snn0_sys_data_in;
wire  snn1_sys_req, snn1_sys_ack;
wire [31:0] snn1_sys_addr, snn1_sys_data_in;
wire  ann_sys_req,  ann_sys_ack;
wire [31:0] ann_sys_addr,  ann_sys_data_in;
wire  had_sys_req,  had_sys_ack;
wire [31:0] had_sys_addr,  had_sys_data_in;

// ─── Accelerator and fill_unit status ────────────────────────────────────────
wire  snn0_busy, snn0_finished, snn0_result;
wire  snn1_busy, snn1_finished, snn1_result;
wire  ann_busy,  ann_finished,  ann_result;
wire  had_busy,  had_finished;
wire  fu_busy,   fu_finished,   fu_ack;

// ─── Buffer address wires (BBA register bank outputs) ────────────────────────
wire [`PIN_BITS-1:0] snn0_sp_tgt,  snn0_sp_src1, snn0_sp_src2, snn0_sp_src3;
wire [`PIN_BITS-1:0] snn0_np_tgt,  snn0_np_src1, snn0_np_src2, snn0_np_src3;
wire [`PIN_BITS-1:0] snn1_sp_tgt,  snn1_sp_src1, snn1_sp_src2, snn1_sp_src3;
wire [`PIN_BITS-1:0] snn1_np_tgt,  snn1_np_src1, snn1_np_src2, snn1_np_src3;
wire [`PIN_BITS-1:0] ann_sp_tgt,   ann_sp_src1,  ann_sp_src2,  ann_sp_src3;
wire [`PIN_BITS-1:0] ann_np_tgt,   ann_np_src1,  ann_np_src2,  ann_np_src3;

// ─── Config write counters (one 2-bit counter per computation accelerator) ───
// Reset at each new dispatch to that accelerator; increment per config word write.
reg [LOG2_WPC-1:0] cfg_cnt_0, cfg_cnt_1, cfg_cnt_2, cfg_cnt_3;

always @(posedge clk or posedge reset) begin
    if (reset || (sch_start_new_block && sch_target_acc == 3'd0))
        cfg_cnt_0 <= 2'b00;
    else if (cm_config_wr[0])
        cfg_cnt_0 <= cfg_cnt_0 + 1'b1;
end
always @(posedge clk or posedge reset) begin
    if (reset || (sch_start_new_block && sch_target_acc == 3'd1))
        cfg_cnt_1 <= 2'b00;
    else if (cm_config_wr[1])
        cfg_cnt_1 <= cfg_cnt_1 + 1'b1;
end
always @(posedge clk or posedge reset) begin
    if (reset || (sch_start_new_block && sch_target_acc == 3'd2))
        cfg_cnt_2 <= 2'b00;
    else if (cm_config_wr[2])
        cfg_cnt_2 <= cfg_cnt_2 + 1'b1;
end
always @(posedge clk or posedge reset) begin
    if (reset || (sch_start_new_block && sch_target_acc == 3'd3))
        cfg_cnt_3 <= 2'b00;
    else if (cm_config_wr[3])
        cfg_cnt_3 <= cfg_cnt_3 + 1'b1;
end

// ─── AXI mux: config_manager writes take priority over external host ──────────
wire snn0_ext_req = sys_req_i & (sys_addr_i[31:16] == SNN0_CFG_BASE[31:16]);
wire snn1_ext_req = sys_req_i & (sys_addr_i[31:16] == SNN1_CFG_BASE[31:16]);
wire ann_ext_req  = sys_req_i & (sys_addr_i[31:16] == ANN_CFG_BASE[31:16]);
wire had_ext_req  = sys_req_i & (sys_addr_i[31:16] == HAD_CFG_BASE[31:16]);

assign snn0_sys_req     = cm_config_wr[0] ? 1'b1 : snn0_ext_req;
assign snn0_sys_addr    = cm_config_wr[0]
                        ? (SNN0_CFG_BASE | {{28{1'b0}}, cfg_cnt_0, 2'b00})
                        : sys_addr_i;
assign snn0_sys_data_in = cm_config_wr[0] ? cm_config_data : sys_data_i;

assign snn1_sys_req     = cm_config_wr[1] ? 1'b1 : snn1_ext_req;
assign snn1_sys_addr    = cm_config_wr[1]
                        ? (SNN1_CFG_BASE | {{28{1'b0}}, cfg_cnt_1, 2'b00})
                        : sys_addr_i;
assign snn1_sys_data_in = cm_config_wr[1] ? cm_config_data : sys_data_i;

assign ann_sys_req      = cm_config_wr[2] ? 1'b1 : ann_ext_req;
assign ann_sys_addr     = cm_config_wr[2]
                        ? (ANN_CFG_BASE | {{28{1'b0}}, cfg_cnt_2, 2'b00})
                        : sys_addr_i;
assign ann_sys_data_in  = cm_config_wr[2] ? cm_config_data : sys_data_i;

assign had_sys_req      = cm_config_wr[3] ? 1'b1 : had_ext_req;
assign had_sys_addr     = cm_config_wr[3]
                        ? (HAD_CFG_BASE | {{28{1'b0}}, cfg_cnt_3, 2'b00})
                        : sys_addr_i;
assign had_sys_data_in  = cm_config_wr[3] ? cm_config_data : sys_data_i;

wire snn0_ext_ack = snn0_sys_ack & ~cm_config_wr[0];
wire snn1_ext_ack = snn1_sys_ack & ~cm_config_wr[1];
wire ann_ext_ack  = ann_sys_ack  & ~cm_config_wr[2];
wire had_ext_ack  = had_sys_ack  & ~cm_config_wr[3];

wire [NUM_COMP_ACC-1:0] cm_config_wait    = {NUM_COMP_ACC{1'b0}};
wire [NUM_COMP_ACC-1:0] cm_buff_base_wait = {NUM_COMP_ACC{1'b0}};

// sys_ack_o / sys_data_o are driven below, after the pool-readback FSM, so the
// mux can fold in pool_rd_ack / prd_data_r from the POOL_RD_BASE window.

// ─── BBA register banks ───────────────────────────────────────────────────────
reg [BBA_CNT_SZ-1:0] bba_cnt_0, bba_cnt_1, bba_cnt_2, bba_cnt_3;
reg [31:0] bba_r0 [0:3];
reg [31:0] bba_r1 [0:3];
reg [31:0] bba_r2 [0:3];
reg [31:0] bba_r3 [0:3];

always @(posedge clk or posedge reset) begin
    if (reset || (sch_start_new_block && sch_target_acc == 3'd0))
        bba_cnt_0 <= 2'b00;
    else if (cm_buff_base_wr[0])
        bba_cnt_0 <= bba_cnt_0 + 1'b1;
end
always @(posedge clk) begin
    if (cm_buff_base_wr[0])
        bba_r0[bba_cnt_0] <= cm_buff_base_data;
end

always @(posedge clk or posedge reset) begin
    if (reset || (sch_start_new_block && sch_target_acc == 3'd1))
        bba_cnt_1 <= 2'b00;
    else if (cm_buff_base_wr[1])
        bba_cnt_1 <= bba_cnt_1 + 1'b1;
end
always @(posedge clk) begin
    if (cm_buff_base_wr[1])
        bba_r1[bba_cnt_1] <= cm_buff_base_data;
end

always @(posedge clk or posedge reset) begin
    if (reset || (sch_start_new_block && sch_target_acc == 3'd2))
        bba_cnt_2 <= 2'b00;
    else if (cm_buff_base_wr[2])
        bba_cnt_2 <= bba_cnt_2 + 1'b1;
end
always @(posedge clk) begin
    if (cm_buff_base_wr[2])
        bba_r2[bba_cnt_2] <= cm_buff_base_data;
end

always @(posedge clk or posedge reset) begin
    if (reset || (sch_start_new_block && sch_target_acc == 3'd3))
        bba_cnt_3 <= 2'b00;
    else if (cm_buff_base_wr[3])
        bba_cnt_3 <= bba_cnt_3 + 1'b1;
end
always @(posedge clk) begin
    if (cm_buff_base_wr[3])
        bba_r3[bba_cnt_3] <= cm_buff_base_data;
end

assign snn0_sp_tgt  = bba_r0[0][`PIN_BITS-1:0];
assign snn0_sp_src1 = bba_r0[1][`PIN_BITS-1:0];
assign snn0_sp_src2 = bba_r0[2][`PIN_BITS-1:0];
assign snn0_sp_src3 = bba_r0[3][`PIN_BITS-1:0];
assign snn0_np_tgt  = bba_r0[0][`PIN_BITS-1:0];
assign snn0_np_src1 = bba_r0[1][`PIN_BITS-1:0];
assign snn0_np_src2 = bba_r0[2][`PIN_BITS-1:0];
assign snn0_np_src3 = bba_r0[3][`PIN_BITS-1:0];

assign snn1_sp_tgt  = bba_r1[0][`PIN_BITS-1:0];
assign snn1_sp_src1 = bba_r1[1][`PIN_BITS-1:0];
assign snn1_sp_src2 = bba_r1[2][`PIN_BITS-1:0];
assign snn1_sp_src3 = bba_r1[3][`PIN_BITS-1:0];
assign snn1_np_tgt  = bba_r1[0][`PIN_BITS-1:0];
assign snn1_np_src1 = bba_r1[1][`PIN_BITS-1:0];
assign snn1_np_src2 = bba_r1[2][`PIN_BITS-1:0];
assign snn1_np_src3 = bba_r1[3][`PIN_BITS-1:0];

assign ann_sp_tgt   = bba_r2[0][`PIN_BITS-1:0];
assign ann_sp_src1  = bba_r2[1][`PIN_BITS-1:0];
assign ann_sp_src2  = bba_r2[2][`PIN_BITS-1:0];
assign ann_sp_src3  = bba_r2[3][`PIN_BITS-1:0];
assign ann_np_tgt   = bba_r2[0][`PIN_BITS-1:0];
assign ann_np_src1  = bba_r2[1][`PIN_BITS-1:0];
assign ann_np_src2  = bba_r2[2][`PIN_BITS-1:0];
assign ann_np_src3  = bba_r2[3][`PIN_BITS-1:0];

// ─── Buffer info extraction for config_manager ────────────────────────────────
// config_manager (BUFF_INDEX_SZ=BUFF_INDX_SZ=4):
//   buffer_info_i[3:0]   = src1 id  (long  slot 0 id @ LS0_ID_START)
//   buffer_info_i[7:4]   = src2 id  (long  slot 1 id @ LS1_ID_START)
//   buffer_info_i[11:8]  = src3 id  (long  slot 2 id @ LS2_ID_START)
//   buffer_info_i[15:12] = tgt  id  (short slot 0 id @ MODE_SZ)
//   buffer_info_i[31:16] = 16'b0
wire [`SCH_ENTRY_SZ-1:0] cm_buffer_info;
assign cm_buffer_info = {16'b0,
                          sch_buffer_info[MODE_SZ +: BUFF_INDX_SZ],       // tgt:  short slot 0 id
                          sch_buffer_info[LS2_ID_START +: BUFF_INDX_SZ],  // src3: long  slot 2 id
                          sch_buffer_info[LS1_ID_START +: BUFF_INDX_SZ],  // src2: long  slot 1 id
                          sch_buffer_info[LS0_ID_START +: BUFF_INDX_SZ]}; // src1: long  slot 0 id

wire [CFG_ID_SZ-1:0] cm_config_id = sch_buffer_info[E_CFG_START +: CFG_ID_SZ];

// ─── Accelerator-side internal wires for write-shared memories ────────────────
// These decouple the accelerator outputs from the top-level ports so the
// arbitration mux can choose between acc and fill_unit.

// snnAcc0 — syn_curr (rd+wr), pot (rd+wr), spike (wr), act (rd)
wire                  acc_s0_syn_curr_wr,   acc_s0_syn_curr_rd;
wire [`ADDR_SIZE-1:0] acc_s0_syn_curr_addr;
wire  [`POT_BITS-1:0] acc_s0_syn_curr_data;
wire  [`POT_BITS-1:0] acc_s0_syn_curr_data_i;
wire                  acc_s0_syn_curr_wait;
wire                  acc_s0_pot_wr,        acc_s0_pot_rd;
wire [`ADDR_SIZE-1:0] acc_s0_pot_addr;
wire  [`POT_BITS-1:0] acc_s0_pot_data;
wire                  acc_s0_spike_wr;
wire [`ADDR_SIZE-1:0] acc_s0_spike_addr;
wire  [`ACT_BITS-1:0] acc_s0_spike_data;
wire                  acc_s0_spike_wait;
wire                  acc_s0_act_req;
wire [`ADDR_SIZE-1:0] acc_s0_act_addr;
wire  [`ACT_BITS-1:0] acc_s0_act_data;
wire                  acc_s0_act_wait;

// snnAcc1 — syn_curr, pot, spike, act
wire                  acc_s1_syn_curr_wr,   acc_s1_syn_curr_rd;
wire [`ADDR_SIZE-1:0] acc_s1_syn_curr_addr;
wire  [`POT_BITS-1:0] acc_s1_syn_curr_data;
wire  [`POT_BITS-1:0] acc_s1_syn_curr_data_i;
wire                  acc_s1_syn_curr_wait;
wire                  acc_s1_pot_wr,        acc_s1_pot_rd;
wire [`ADDR_SIZE-1:0] acc_s1_pot_addr;
wire  [`POT_BITS-1:0] acc_s1_pot_data;
wire                  acc_s1_spike_wr;
wire [`ADDR_SIZE-1:0] acc_s1_spike_addr;
wire  [`ACT_BITS-1:0] acc_s1_spike_data;
wire                  acc_s1_spike_wait;
wire                  acc_s1_act_req;
wire [`ADDR_SIZE-1:0] acc_s1_act_addr;
wire  [`ACT_BITS-1:0] acc_s1_act_data;
wire                  acc_s1_act_wait;

// annAcc — syn_curr, pot, spike, act
wire                  acc_a0_syn_curr_wr,   acc_a0_syn_curr_rd;
wire [`ADDR_SIZE-1:0] acc_a0_syn_curr_addr;
wire  [`POT_BITS-1:0] acc_a0_syn_curr_data;
wire  [`POT_BITS-1:0] acc_a0_syn_curr_data_i;
wire                  acc_a0_syn_curr_wait;
wire                  acc_a0_pot_wr,        acc_a0_pot_rd;
wire [`ADDR_SIZE-1:0] acc_a0_pot_addr;
wire  [`POT_BITS-1:0] acc_a0_pot_data;
wire                  acc_a0_spike_wr;
wire [`ADDR_SIZE-1:0] acc_a0_spike_addr;
wire  [`ACT_BITS-1:0] acc_a0_spike_data;
wire                  acc_a0_spike_wait;
wire                  acc_a0_act_req;
wire [`ADDR_SIZE-1:0] acc_a0_act_addr;
wire  [`ACT_BITS-1:0] acc_a0_act_data;
wire                  acc_a0_act_wait;

// Hadamard — src_a/b/z reads, src_r read + write (all pooled).
wire                  acc_hd_a_req;
wire [`ADDR_SIZE-1:0] acc_hd_a_addr;
wire           [31:0] acc_hd_a_data;
wire                  acc_hd_a_wait;
wire                  acc_hd_b_req;
wire [`ADDR_SIZE-1:0] acc_hd_b_addr;
wire           [31:0] acc_hd_b_data;
wire                  acc_hd_b_wait;
wire                  acc_hd_z_req;
wire [`ADDR_SIZE-1:0] acc_hd_z_addr;
wire           [31:0] acc_hd_z_data;
wire                  acc_hd_z_wait;
wire                  acc_hd_r_rd_req;
wire [`ADDR_SIZE-1:0] acc_hd_r_rd_addr;
wire           [31:0] acc_hd_r_rd_data;
wire                  acc_hd_r_rd_wait;
wire                  acc_hd_r_wr;
wire [`ADDR_SIZE-1:0] acc_hd_r_wr_addr;
wire           [31:0] acc_hd_r_wr_data;
wire                  acc_hd_r_wr_wait;

// ─── fill_unit internal wires for write-shared memory buses ──────────────────
// (fill-only memory buses connect directly in the u_fill instantiation below)

// pot stays dedicated per accelerator (arbitrated acc-vs-fill, as before).
wire                  fu_s0_pot_wr;
wire [`ADDR_SIZE-1:0] fu_s0_pot_addr;
wire           [31:0] fu_s0_pot_data;
wire                  fu_s0_pot_wait;

wire                  fu_s1_pot_wr;
wire [`ADDR_SIZE-1:0] fu_s1_pot_addr;
wire           [31:0] fu_s1_pot_data;
wire                  fu_s1_pot_wait;

wire                  fu_a0_pot_wr;
wire [`ADDR_SIZE-1:0] fu_a0_pot_addr;
wire           [31:0] fu_a0_pot_data;
wire                  fu_a0_pot_wait;

// fill_unit's single write port into the shared pool (act/spike/syn_curr + all
// Hadamard buffers). Routed through the shared_pool arbiter (addr[1:0] picks
// the bank). The per-acc act/spike/syn_curr + hd_src_a/b/z/r fill ports are
// left unconnected at u_fill.
wire                  fu_shared_wr;
wire [`ADDR_SIZE-1:0] fu_shared_addr;
wire           [31:0] fu_shared_data;
wire                  fu_shared_wait;

// ─── pot arbitration muxes (dedicated per-acc; acc wins over fill) ───────────
// snnAcc0 pot
wire acc_uses_s0_pot = acc_s0_pot_rd | acc_s0_pot_wr;
assign s0_pot_mem_wr_o   = acc_uses_s0_pot ? acc_s0_pot_wr   : fu_s0_pot_wr;
assign s0_pot_mem_rd_o   = acc_s0_pot_rd;
assign s0_pot_mem_addr_o = acc_uses_s0_pot ? acc_s0_pot_addr : fu_s0_pot_addr;
assign s0_pot_mem_data_o = acc_uses_s0_pot ? acc_s0_pot_data : fu_s0_pot_data[`POT_BITS-1:0];
assign fu_s0_pot_wait    = acc_uses_s0_pot | s0_pot_mem_wait_i;

// snnAcc1 pot
wire acc_uses_s1_pot = acc_s1_pot_rd | acc_s1_pot_wr;
assign s1_pot_mem_wr_o   = acc_uses_s1_pot ? acc_s1_pot_wr   : fu_s1_pot_wr;
assign s1_pot_mem_rd_o   = acc_s1_pot_rd;
assign s1_pot_mem_addr_o = acc_uses_s1_pot ? acc_s1_pot_addr : fu_s1_pot_addr;
assign s1_pot_mem_data_o = acc_uses_s1_pot ? acc_s1_pot_data : fu_s1_pot_data[`POT_BITS-1:0];
assign fu_s1_pot_wait    = acc_uses_s1_pot | s1_pot_mem_wait_i;

// annAcc pot
wire acc_uses_a0_pot = acc_a0_pot_rd | acc_a0_pot_wr;
assign a0_pot_mem_wr_o   = acc_uses_a0_pot ? acc_a0_pot_wr   : fu_a0_pot_wr;
assign a0_pot_mem_rd_o   = acc_a0_pot_rd;
assign a0_pot_mem_addr_o = acc_uses_a0_pot ? acc_a0_pot_addr : fu_a0_pot_addr;
assign a0_pot_mem_data_o = acc_uses_a0_pot ? acc_a0_pot_data : fu_a0_pot_data[`POT_BITS-1:0];
assign fu_a0_pot_wait    = acc_uses_a0_pot | a0_pot_mem_wait_i;

// ─── Shared act/spike/syn_curr + Hadamard pool — shared_pool (4 banks) ───────
// Requesters, strict priority (index 0 = highest, index SP_NREQ-1 = lowest):
//   0 FILL  1 s0_syn 2 s0_act 3 s0_spike  4 s1_syn 5 s1_act 6 s1_spike
//   7 a0_syn 8 a0_act 9 a0_spike  10 hd_a 11 hd_b 12 hd_z 13 hd_r_rd 14 hd_r_wr
//   15 pool_rd  (host AXI readback window — lowest priority, never disturbs compute)
localparam SP_NREQ  = 16;
localparam SP_NBANK = 4;

wire [SP_NREQ-1:0]            sp_req_act;
wire [SP_NREQ-1:0]            sp_req_rd;
wire [SP_NREQ-1:0]            sp_req_wr;
wire [SP_NREQ*`ADDR_SIZE-1:0] sp_req_addr;
wire [SP_NREQ*32-1:0]         sp_req_wdata;
wire [SP_NREQ-1:0]            sp_req_wait;
wire [SP_NREQ*32-1:0]         sp_req_rdata;

// ── Host pool-readback window (requester index SP_NREQ-1, lowest priority) ──
// Read-only, memory-mapped view of the shared pool on the sys_* (AXI) bus.
// Variable latency: the host holds sys_req_i until sys_ack_o.  Read FSM:
//   REQ : drive the pool read until granted (!sp_req_wait[SP_NREQ-1])
//   CAP : pool read data is valid for this one cycle -> latch it
//   ACK : assert sys_ack_o + present data until the host drops sys_req_i
wire pool_rd_sel = sys_req_i & (sys_addr_i[31:20] == POOL_RD_BASE[31:20]);

// window byte-offset >> 2 = pool logical word (bank-interleaved, addr[1:0]=bank)
wire [`ADDR_SIZE-1:0] pool_rd_addr =
    {{(`ADDR_SIZE-18){1'b0}}, sys_addr_i[19:2]};

localparam PRD_IDLE = 2'd0, PRD_REQ = 2'd1, PRD_CAP = 2'd2, PRD_ACK = 2'd3;
reg  [1:0]  prd_state;
reg  [31:0] prd_data_r;

always @(posedge clk or posedge reset) begin
    if (reset) begin
        prd_state  <= PRD_IDLE;
        prd_data_r <= 32'b0;
    end else begin
        case (prd_state)
            PRD_IDLE: if (pool_rd_sel)                  prd_state <= PRD_REQ;
            PRD_REQ : if (!pool_rd_sel)                 prd_state <= PRD_IDLE;
                      else if (!sp_req_wait[SP_NREQ-1]) prd_state <= PRD_CAP;
            PRD_CAP : begin
                          prd_data_r <= sp_req_rdata[(SP_NREQ-1)*32 +: 32];
                          prd_state  <= PRD_ACK;
                      end
            PRD_ACK : if (!pool_rd_sel)                 prd_state <= PRD_IDLE;
            default :                                   prd_state <= PRD_IDLE;
        endcase
    end
end

wire pool_rd_req_active = (prd_state == PRD_REQ) & pool_rd_sel;
wire pool_rd_ack        = (prd_state == PRD_ACK);

// Host AXI response: fold the pool-readback path into the OR'd ack / data mux.
assign sys_ack_o  = sch_ack | cm_ack | fu_ack
                  | snn0_ext_ack | snn1_ext_ack | ann_ext_ack | had_ext_ack
                  | pool_rd_ack;
assign sys_data_o = pool_rd_ack ? prd_data_r : sch_data_o;

assign sp_req_act = { pool_rd_req_active,
                      acc_hd_r_wr, acc_hd_r_rd_req, acc_hd_z_req, acc_hd_b_req,
                      acc_hd_a_req,
                      acc_a0_spike_wr, acc_a0_act_req,
                      (acc_a0_syn_curr_rd | acc_a0_syn_curr_wr),
                      acc_s1_spike_wr, acc_s1_act_req,
                      (acc_s1_syn_curr_rd | acc_s1_syn_curr_wr),
                      acc_s0_spike_wr, acc_s0_act_req,
                      (acc_s0_syn_curr_rd | acc_s0_syn_curr_wr),
                      fu_shared_wr };
assign sp_req_rd  = { pool_rd_req_active,
                      1'b0, acc_hd_r_rd_req, acc_hd_z_req, acc_hd_b_req,
                      acc_hd_a_req,
                      1'b0, acc_a0_act_req, acc_a0_syn_curr_rd,
                      1'b0, acc_s1_act_req, acc_s1_syn_curr_rd,
                      1'b0, acc_s0_act_req, acc_s0_syn_curr_rd,
                      1'b0 };
assign sp_req_wr  = { 1'b0,
                      acc_hd_r_wr, 1'b0, 1'b0, 1'b0, 1'b0,
                      acc_a0_spike_wr, 1'b0, acc_a0_syn_curr_wr,
                      acc_s1_spike_wr, 1'b0, acc_s1_syn_curr_wr,
                      acc_s0_spike_wr, 1'b0, acc_s0_syn_curr_wr,
                      fu_shared_wr };
assign sp_req_addr = { pool_rd_addr,
                       acc_hd_r_wr_addr, acc_hd_r_rd_addr, acc_hd_z_addr,
                       acc_hd_b_addr, acc_hd_a_addr,
                       acc_a0_spike_addr, acc_a0_act_addr, acc_a0_syn_curr_addr,
                       acc_s1_spike_addr, acc_s1_act_addr, acc_s1_syn_curr_addr,
                       acc_s0_spike_addr, acc_s0_act_addr, acc_s0_syn_curr_addr,
                       fu_shared_addr };
assign sp_req_wdata = { 32'b0,
                        acc_hd_r_wr_data, 32'b0, 32'b0, 32'b0, 32'b0,
                        acc_a0_spike_data, 32'b0, acc_a0_syn_curr_data,
                        acc_s1_spike_data, 32'b0, acc_s1_syn_curr_data,
                        acc_s0_spike_data, 32'b0, acc_s0_syn_curr_data,
                        fu_shared_data };

shared_pool #(
    .NUM_BANKS (SP_NBANK),
    .NUM_REQ   (SP_NREQ),
    .ADDR_W    (`ADDR_SIZE),
    .DATA_W    (32),
    // Round-robin per-bank arbitration: an accelerator can issue two reads to the
    // same bank and stall holding one (e.g. annAcc act + syn_curr both on bank 0);
    // strict priority would starve the lower-index port and deadlock. RR guarantees
    // forward progress. Bit-identical to strict for non-contending traffic.
    .ARB_RR    (1)
) u_pool (
    .clk          (clk),
    .req_act_i    (sp_req_act),
    .req_rd_i     (sp_req_rd),
    .req_wr_i     (sp_req_wr),
    .req_addr_i   (sp_req_addr),
    .req_wdata_i  (sp_req_wdata),
    .req_wait_o   (sp_req_wait),
    .req_rdata_o  (sp_req_rdata),
    .bank_rd_o    ({m3_data_mem_rd_o,    m2_data_mem_rd_o,    m1_data_mem_rd_o,    m0_data_mem_rd_o}),
    .bank_wr_o    ({m3_data_mem_wr_o,    m2_data_mem_wr_o,    m1_data_mem_wr_o,    m0_data_mem_wr_o}),
    .bank_addr_o  ({m3_data_mem_addr_o,  m2_data_mem_addr_o,  m1_data_mem_addr_o,  m0_data_mem_addr_o}),
    .bank_wdata_o ({m3_data_mem_wdata_o, m2_data_mem_wdata_o, m1_data_mem_wdata_o, m0_data_mem_wdata_o}),
    .bank_wait_i  ({m3_data_mem_wait_i,  m2_data_mem_wait_i,  m1_data_mem_wait_i,  m0_data_mem_wait_i}),
    .bank_rdata_i ({m3_data_mem_rdata_i, m2_data_mem_rdata_i, m1_data_mem_rdata_i, m0_data_mem_rdata_i})
);

// Unpack per-requester wait + read data.
assign fu_shared_wait       = sp_req_wait[0];
assign acc_s0_syn_curr_wait = sp_req_wait[1];
assign acc_s0_act_wait      = sp_req_wait[2];
assign acc_s0_spike_wait    = sp_req_wait[3];
assign acc_s1_syn_curr_wait = sp_req_wait[4];
assign acc_s1_act_wait      = sp_req_wait[5];
assign acc_s1_spike_wait    = sp_req_wait[6];
assign acc_a0_syn_curr_wait = sp_req_wait[7];
assign acc_a0_act_wait      = sp_req_wait[8];
assign acc_a0_spike_wait    = sp_req_wait[9];
assign acc_hd_a_wait        = sp_req_wait[10];
assign acc_hd_b_wait        = sp_req_wait[11];
assign acc_hd_z_wait        = sp_req_wait[12];
assign acc_hd_r_rd_wait     = sp_req_wait[13];
assign acc_hd_r_wr_wait     = sp_req_wait[14];

assign acc_s0_syn_curr_data_i = sp_req_rdata[1*32  +: 32];
assign acc_s0_act_data        = sp_req_rdata[2*32  +: 32];
assign acc_s1_syn_curr_data_i = sp_req_rdata[4*32  +: 32];
assign acc_s1_act_data        = sp_req_rdata[5*32  +: 32];
assign acc_a0_syn_curr_data_i = sp_req_rdata[7*32  +: 32];
assign acc_a0_act_data        = sp_req_rdata[8*32  +: 32];
assign acc_hd_a_data          = sp_req_rdata[10*32 +: 32];
assign acc_hd_b_data          = sp_req_rdata[11*32 +: 32];
assign acc_hd_z_data          = sp_req_rdata[12*32 +: 32];
assign acc_hd_r_rd_data       = sp_req_rdata[13*32 +: 32];

// ─── Scheduler ────────────────────────────────────────────────────────────────
scheduler #(
    .NUM_HW_ACCELERATORS (NUM_HW_ACCELERATORS),
    .TGT_ACC_SZ          (TGT_ACC_SZ),
    .NUM_BUFFERS         (NUM_BUFFERS),
    .COL_BUFF_ID_SZ      (COL_BUFF_ID_SZ),
    .NUM_SCH_ENTRIES     (NUM_SCH_ENTRIES),
    .CFG_ID_SZ           (CFG_ID_SZ),
    .TGT_COUNT_SZ        (TGT_COUNT_SZ),
    .PROG_ADDR_BITS      (PROG_ADDR_BITS),
    .PROG_DATA_BITS      (PROG_DATA_BITS),
    .SCH_PROG_MEM_ADDR   (SCH_PROG_MEM_ADDR),
    .SCH_PROG_MEM_MASK   (SCH_PROG_MEM_MASK)
) u_scheduler (
    .clk                 (clk),
    .reset               (reset),
    .test_stall_pipe     (test_stall_pipe),
    .sys_req_i           (sys_req_i),
    .sys_ack_o           (sch_ack),
    .sys_addr_i          (sys_addr_i),
    .sys_data_i          (sys_data_i),
    .sys_data_o          (sch_data_o),
    .prog_mem_addr_o     (prog_mem_addr_o),
    .prog_mem_data_i     (prog_mem_data_i),
    .prog_mem_req_o      (prog_mem_req_o),
    .prog_mem_wait_i     (prog_mem_wait_i),
    .prog_mem_wr_o       (prog_mem_wr_o),
    .prog_mem_wr_addr_o  (prog_mem_wr_addr_o),
    .prog_mem_wr_data_o  (prog_mem_wr_data_o),
    .prog_mem_wr_wait_i  (prog_mem_wr_wait_i),
    .acc_busy_i          ({fu_busy,   had_busy,     ann_busy,     snn1_busy,     snn0_busy}),
    .acc_finished_i      ({fu_finished, had_finished, ann_finished, snn1_finished, snn0_finished}),
    .acc_result_i        ({1'b0,      1'b0,         ann_result,   snn1_result,   snn0_result}),
    .start_new_block_o   (sch_start_new_block),
    .target_acc_o        (sch_target_acc),
    .buffer_info_o       (sch_buffer_info),
    .fill_value_o        (sch_fill_value),
    .fill_block_size_o   (sch_fill_block_size),
    .nxt_input_pulse_o   (nxt_input_pulse_o),
    .nxt_output_pulse_o  (nxt_output_pulse_o),
    .cm_busy_i           (cm_busy)
);

// ─── Config manager ───────────────────────────────────────────────────────────
// NUM_ACC=4: config_manager only serves computation accelerators 0-3.
// start_new_block_i is suppressed for FILL dispatches.
// target_acc_i is 2-bit ($clog2(4)); acc_id 0-3 always fit in 2 bits.
config_manager #(
    .NUM_ACC                 (NUM_COMP_ACC),
    .BUFF_INDEX_SZ           (BUFF_INDX_SZ),
    .NUM_BUFFERS             (NUM_BUFFERS),
    .CFG_ID_SZ               (CFG_ID_SZ),
    .WORDS_PER_CONFIG        (WORDS_PER_CONFIG),
    .TGT_CONFIG_MEM_ADDR     (CM_CFG_MEM_ADDR),
    .TGT_CONFIG_MEM_ADDR_MASK(CM_CFG_MEM_MASK),
    .TGT_BUFF_STORE_ADDR     (CM_BBA_MEM_ADDR),
    .TGT_BUFF_STORE_ADDR_MASK(CM_BBA_MEM_MASK)
) u_config_manager (
    .clk                  (clk),
    .reset                (reset),
    .cm_sys_req_i         (sys_req_i),
    .cm_sys_ack_o         (cm_ack),
    .cm_sys_addr_i        (sys_addr_i),
    .cm_sys_data_i        (sys_data_i),
    .start_new_block_i    (cm_start_new_block),
    .target_acc_i         (sch_target_acc[1:0]),   // always 0-3 for non-FILL
    .config_id_i          (cm_config_id),
    .buffer_info_i        (cm_buffer_info),
    .cm_config_finished_o (cm_config_finished_o),
    .cm_busy_o            (cm_busy),
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
    .cm_tgt_acc_o         (cm_tgt_acc_narrow),
    .cm_config_wr_o       (cm_config_wr),
    .cm_config_wait_i     (cm_config_wait),
    .cm_config_data_o     (cm_config_data),
    .cm_buff_base_wr_o    (cm_buff_base_wr),
    .cm_buff_base_wait_i  (cm_buff_base_wait),
    .cm_buff_base_data_o  (cm_buff_base_data)
);

// ─── snnAcc0 (TGT_ACC_ID = 0) ────────────────────────────────────────────────
acc_snn_processor #(
    .TGT_ACC_ID           (0),
    .TGT_CONFIG_BASE_ADDR (SNN0_CFG_BASE),
    // 32-bit syn_curr/pot/bias slices: use the full stored value rather than
    // the top bits of a right-justified small integer (matches the tb mems).
    .NP_SYN_CURR_SLICE_BITS (32),
    .NP_POT_SLICE_BITS      (32),
    .NP_BIAS_CURR_SLICE_BITS(32),
    .SP_SYN_CURR_SLICE_BITS (32),
    .SP_BIAS_CURR_SLICE_BITS(32)
) u_snn0 (
    .clk                   (clk),
    .reset                 (reset),
    .sys_req_i             (snn0_sys_req),
    .sys_ack_o             (snn0_sys_ack),
    .sys_addr_i            (snn0_sys_addr),
    .sys_data_i            (snn0_sys_data_in),
    .start_new_block_i     (acc_start_new_block),
    .target_acc_i          (cm_tgt_acc),
    .buffer_info_i         (sch_buffer_info[`SCH_ENTRY_SZ-1:0]),
    .spike_proc_finished_o (snn0_result),
    .acc_busy_o            (snn0_busy),
    .acc_finished_o        (snn0_finished),
    .sp_src1_buff_addr_i   (snn0_sp_src1),
    .sp_src2_buff_addr_i   (snn0_sp_src2),
    .sp_src3_buff_addr_i   (snn0_sp_src3),
    .sp_tgt_buff_addr_i    (snn0_sp_tgt),
    .sp_weight_row_len_i   ({`PIN_BITS{1'b0}}),
    .np_src1_buff_addr_i   (snn0_np_src1),
    .np_src2_buff_addr_i   (snn0_np_src2),
    .np_src3_buff_addr_i   (snn0_np_src3),
    .np_tgt_buff_addr_i    (snn0_np_tgt),
    .np_weight_row_len_i   ({`PIN_BITS{1'b0}}),
    .weight_mem_rd_o       (s0_weight_mem_rd_o),
    .weight_mem_wait_i     (s0_weight_mem_wait_i),
    .weight_mem_addr_o     (s0_weight_mem_addr_o),
    .weight_mem_data_i     (s0_weight_mem_data_i),
    .act_mem_req_o         (acc_s0_act_req),
    .act_mem_wait_i        (acc_s0_act_wait),
    .act_mem_addr_o        (acc_s0_act_addr),
    .act_mem_data_i        (acc_s0_act_data),
    .syn_curr_mem_wr_o     (acc_s0_syn_curr_wr),
    .syn_curr_mem_rd_o     (acc_s0_syn_curr_rd),
    .syn_curr_mem_wait_i   (acc_s0_syn_curr_wait),
    .syn_curr_mem_addr_o   (acc_s0_syn_curr_addr),
    .syn_curr_mem_data_o   (acc_s0_syn_curr_data),
    .syn_curr_mem_data_i   (acc_s0_syn_curr_data_i),
    .bias_curr_mem_rd_o    (s0_bias_curr_mem_rd_o),
    .bias_curr_mem_wait_i  (s0_bias_curr_mem_wait_i),
    .bias_curr_mem_addr_o  (s0_bias_curr_mem_addr_o),
    .bias_curr_mem_data_i  (s0_bias_curr_mem_data_i),
    .thresh_mem_rd_o       (s0_thresh_mem_rd_o),
    .thresh_mem_wait_i     (s0_thresh_mem_wait_i),
    .thresh_mem_addr_o     (s0_thresh_mem_addr_o),
    .thresh_mem_data_i     (s0_thresh_mem_data_i),
    .pot_mem_wr_o          (acc_s0_pot_wr),
    .pot_mem_rd_o          (acc_s0_pot_rd),
    .pot_mem_wait_i        (s0_pot_mem_wait_i),
    .pot_mem_addr_o        (acc_s0_pot_addr),
    .pot_mem_data_o        (acc_s0_pot_data),
    .pot_mem_data_i        (s0_pot_mem_data_i),
    .spike_mem_wr_o        (acc_s0_spike_wr),
    .spike_mem_wait_i      (acc_s0_spike_wait),
    .spike_mem_addr_o      (acc_s0_spike_addr),
    .spike_mem_data_o      (acc_s0_spike_data)
);

// ─── snnAcc1 (TGT_ACC_ID = 1) ────────────────────────────────────────────────
acc_snn_processor #(
    .TGT_ACC_ID           (1),
    .TGT_CONFIG_BASE_ADDR (SNN1_CFG_BASE),
    .NP_SYN_CURR_SLICE_BITS (32),
    .NP_POT_SLICE_BITS      (32),
    .NP_BIAS_CURR_SLICE_BITS(32),
    .SP_SYN_CURR_SLICE_BITS (32),
    .SP_BIAS_CURR_SLICE_BITS(32)
) u_snn1 (
    .clk                   (clk),
    .reset                 (reset),
    .sys_req_i             (snn1_sys_req),
    .sys_ack_o             (snn1_sys_ack),
    .sys_addr_i            (snn1_sys_addr),
    .sys_data_i            (snn1_sys_data_in),
    .start_new_block_i     (acc_start_new_block),
    .target_acc_i          (cm_tgt_acc),
    .buffer_info_i         (sch_buffer_info[`SCH_ENTRY_SZ-1:0]),
    .spike_proc_finished_o (snn1_result),
    .acc_busy_o            (snn1_busy),
    .acc_finished_o        (snn1_finished),
    .sp_src1_buff_addr_i   (snn1_sp_src1),
    .sp_src2_buff_addr_i   (snn1_sp_src2),
    .sp_src3_buff_addr_i   (snn1_sp_src3),
    .sp_tgt_buff_addr_i    (snn1_sp_tgt),
    .sp_weight_row_len_i   ({`PIN_BITS{1'b0}}),
    .np_src1_buff_addr_i   (snn1_np_src1),
    .np_src2_buff_addr_i   (snn1_np_src2),
    .np_src3_buff_addr_i   (snn1_np_src3),
    .np_tgt_buff_addr_i    (snn1_np_tgt),
    .np_weight_row_len_i   ({`PIN_BITS{1'b0}}),
    .weight_mem_rd_o       (s1_weight_mem_rd_o),
    .weight_mem_wait_i     (s1_weight_mem_wait_i),
    .weight_mem_addr_o     (s1_weight_mem_addr_o),
    .weight_mem_data_i     (s1_weight_mem_data_i),
    .act_mem_req_o         (acc_s1_act_req),
    .act_mem_wait_i        (acc_s1_act_wait),
    .act_mem_addr_o        (acc_s1_act_addr),
    .act_mem_data_i        (acc_s1_act_data),
    .syn_curr_mem_wr_o     (acc_s1_syn_curr_wr),
    .syn_curr_mem_rd_o     (acc_s1_syn_curr_rd),
    .syn_curr_mem_wait_i   (acc_s1_syn_curr_wait),
    .syn_curr_mem_addr_o   (acc_s1_syn_curr_addr),
    .syn_curr_mem_data_o   (acc_s1_syn_curr_data),
    .syn_curr_mem_data_i   (acc_s1_syn_curr_data_i),
    .bias_curr_mem_rd_o    (s1_bias_curr_mem_rd_o),
    .bias_curr_mem_wait_i  (s1_bias_curr_mem_wait_i),
    .bias_curr_mem_addr_o  (s1_bias_curr_mem_addr_o),
    .bias_curr_mem_data_i  (s1_bias_curr_mem_data_i),
    .thresh_mem_rd_o       (s1_thresh_mem_rd_o),
    .thresh_mem_wait_i     (s1_thresh_mem_wait_i),
    .thresh_mem_addr_o     (s1_thresh_mem_addr_o),
    .thresh_mem_data_i     (s1_thresh_mem_data_i),
    .pot_mem_wr_o          (acc_s1_pot_wr),
    .pot_mem_rd_o          (acc_s1_pot_rd),
    .pot_mem_wait_i        (s1_pot_mem_wait_i),
    .pot_mem_addr_o        (acc_s1_pot_addr),
    .pot_mem_data_o        (acc_s1_pot_data),
    .pot_mem_data_i        (s1_pot_mem_data_i),
    .spike_mem_wr_o        (acc_s1_spike_wr),
    .spike_mem_wait_i      (acc_s1_spike_wait),
    .spike_mem_addr_o      (acc_s1_spike_addr),
    .spike_mem_data_o      (acc_s1_spike_data)
);

// ─── annAcc (TGT_ACC_ID = 2) ─────────────────────────────────────────────────
ann_processor #(
    .TGT_ACC_ID           (2),
    .TGT_CONFIG_BASE_ADDR (ANN_CFG_BASE),
    // annAcc already defaults NP_SYN_CURR/NP_POT slices to 32; spike_processing's
    // syn_curr slice still defaults to 10 — bump it so a real task on annAcc reads
    // full-precision 32-bit syn_curr (matches the dense ANN configuration).
    .SP_SYN_CURR_SLICE_BITS (32),
    // §5.2: widen LUT entries from int8 to int16 so 16-bit gate tables fit
    // (sigmoid Q15, tanh Q14 both need 16 bits). Only the LUT path is affected;
    // no current net uses LUT mode (default is ABS), so this is bit-identical
    // for every existing task.
    .NP_LUT_SLICE_BITS      (16),
    // Wide layers: layers wider than an 8-bit input/output grid lane (>255)
    // need wider X grid-index fields to keep the proven 1-D dense
    // datapath (in_y=out_y=1) validated at in_x=512 — rather than rely
    // on the untested 2-D dense path. Wider counters only; bit-identical for any
    // task whose grid fits in 8 bits.
    .SP_X_INPUT_SZ          (16),
    .SP_X_OUTPUT_SZ         (16),
    // Wide layers: weight_generator's output-element index (out_elem_count, drives
    // the weight-cache word/slice address) is WEIGHT_IDX_SZ-wide and defaults to 5
    // bits = 2^5 = 32. With >32 output neurons it wrapped at o=32, re-reading the
    // first 8 weight words for every block of 32 (output bit-exact for o<32,
    // diverged for o>=32). Widen to match SP_X_OUTPUT_SZ. annAcc-only; the default
    // 5 is bit-identical for snnAcc/ipSnnAcc and any task with <=32 output neurons.
    .SP_WEIGHT_IDX_SZ       (16)
) u_ann (
    .clk                   (clk),
    .reset                 (reset),
    .sys_req_i             (ann_sys_req),
    .sys_ack_o             (ann_sys_ack),
    .sys_addr_i            (ann_sys_addr),
    .sys_data_i            (ann_sys_data_in),
    .start_new_block_i     (acc_start_new_block),
    .target_acc_i          (cm_tgt_acc),
    .buffer_info_i         (sch_buffer_info[`SCH_ENTRY_SZ-1:0]),
    .spike_proc_finished_o (ann_result),
    .acc_busy_o            (ann_busy),
    .acc_finished_o        (ann_finished),
    .sp_src1_buff_addr_i   (ann_sp_src1),
    .sp_src2_buff_addr_i   (ann_sp_src2),
    .sp_src3_buff_addr_i   (ann_sp_src3),
    .sp_tgt_buff_addr_i    (ann_sp_tgt),
    .sp_weight_row_len_i   ({`PIN_BITS{1'b0}}),
    .np_src1_buff_addr_i   (ann_np_src1),
    .np_src2_buff_addr_i   (ann_np_src2),
    .np_src3_buff_addr_i   (ann_np_src3),
    .np_tgt_buff_addr_i    (ann_np_tgt),
    .np_weight_row_len_i   ({`PIN_BITS{1'b0}}),
    .weight_mem_rd_o       (a0_weight_mem_rd_o),
    .weight_mem_wait_i     (a0_weight_mem_wait_i),
    .weight_mem_addr_o     (a0_weight_mem_addr_o),
    .weight_mem_data_i     (a0_weight_mem_data_i),
    .act_mem_req_o         (acc_a0_act_req),
    .act_mem_wait_i        (acc_a0_act_wait),
    .act_mem_addr_o        (acc_a0_act_addr),
    .act_mem_data_i        (acc_a0_act_data),
    .syn_curr_mem_wr_o     (acc_a0_syn_curr_wr),
    .syn_curr_mem_rd_o     (acc_a0_syn_curr_rd),
    .syn_curr_mem_wait_i   (acc_a0_syn_curr_wait),
    .syn_curr_mem_addr_o   (acc_a0_syn_curr_addr),
    .syn_curr_mem_data_o   (acc_a0_syn_curr_data),
    .syn_curr_mem_data_i   (acc_a0_syn_curr_data_i),
    .bias_curr_mem_rd_o    (a0_bias_curr_mem_rd_o),
    .bias_curr_mem_wait_i  (a0_bias_curr_mem_wait_i),
    .bias_curr_mem_addr_o  (a0_bias_curr_mem_addr_o),
    .bias_curr_mem_data_i  (a0_bias_curr_mem_data_i),
    .thresh_mem_rd_o       (a0_thresh_mem_rd_o),
    .thresh_mem_wait_i     (a0_thresh_mem_wait_i),
    .thresh_mem_addr_o     (a0_thresh_mem_addr_o),
    .thresh_mem_data_i     (a0_thresh_mem_data_i),
    .pot_mem_wr_o          (acc_a0_pot_wr),
    .pot_mem_rd_o          (acc_a0_pot_rd),
    .pot_mem_wait_i        (a0_pot_mem_wait_i),
    .pot_mem_addr_o        (acc_a0_pot_addr),
    .pot_mem_data_o        (acc_a0_pot_data),
    .pot_mem_data_i        (a0_pot_mem_data_i),
    .spike_mem_wr_o        (acc_a0_spike_wr),
    .spike_mem_wait_i      (acc_a0_spike_wait),
    .spike_mem_addr_o      (acc_a0_spike_addr),
    .spike_mem_data_o      (acc_a0_spike_data)
);

// ─── Hadamard (TGT_ACC_ID = 3) ───────────────────────────────────────────────
hadamard_unit #(
    .TGT_ACC_ID           (3),
    .TGT_CONFIG_BASE_ADDR (HAD_CFG_BASE[31:16]),
    .TGT_ACC_SZ           (TGT_ACC_SZ)
) u_hadamard (
    .clk                  (clk),
    .reset                (reset),
    .hu_sys_req_i         (had_sys_req),
    .hu_sys_ack_o         (had_sys_ack),
    .hu_sys_addr_i        (had_sys_addr),
    .hu_sys_data_i        (had_sys_data_in),
    .hu_start_new_block_i (acc_start_new_block),
    .hu_target_acc_i      (cm_tgt_acc),
    .hu_buffer_info_i     (sch_buffer_info[`SCH_ENTRY_SZ-1:0]),
    .hu_acc_busy_o        (had_busy),
    .hu_acc_finished_o    (had_finished),
    // src_a/b/z reads + src_r read/write all routed to the shared pool.
    .src_a_mem_rd_o       (acc_hd_a_req),
    .src_a_mem_wait_i     (acc_hd_a_wait),
    .src_a_mem_addr_o     (acc_hd_a_addr),
    .src_a_mem_data_i     (acc_hd_a_data),
    .src_b_mem_rd_o       (acc_hd_b_req),
    .src_b_mem_wait_i     (acc_hd_b_wait),
    .src_b_mem_addr_o     (acc_hd_b_addr),
    .src_b_mem_data_i     (acc_hd_b_data),
    .src_z_mem_rd_o       (acc_hd_z_req),
    .src_z_mem_wait_i     (acc_hd_z_wait),
    .src_z_mem_addr_o     (acc_hd_z_addr),
    .src_z_mem_data_i     (acc_hd_z_data),
    .src_r_mem_rd_o       (acc_hd_r_rd_req),
    .src_r_mem_wait_i     (acc_hd_r_rd_wait),
    .src_r_mem_addr_o     (acc_hd_r_rd_addr),
    .src_r_mem_data_i     (acc_hd_r_rd_data),
    .src_r_mem_wr_o       (acc_hd_r_wr),
    .src_r_mem_wr_addr_o  (acc_hd_r_wr_addr),
    .src_r_mem_wr_wait_i  (acc_hd_r_wr_wait),
    .src_r_mem_data_o     (acc_hd_r_wr_data)
);

// ─── fill_unit (TGT_ACC_ID = FILL_ACC_ID = 4) ────────────────────────────────
fill_unit #(
    .FU_TABLE_ADDR      (FU_TABLE_ADDR),
    .FU_TABLE_ADDR_MASK (FU_TABLE_ADDR_MASK),
    .NUM_BUFFERS        (NUM_BUFFERS),
    .BUFF_INDX_SZ       (BUFF_INDX_SZ)
) u_fill (
    .clk                (clk),
    .reset              (reset),
    .sys_req_i          (sys_req_i),
    .sys_ack_o          (fu_ack),
    .sys_addr_i         (sys_addr_i),
    .sys_data_i         (sys_data_i),
    .start_new_block_i  (is_fill_dispatch),
    .acc_busy_o         (fu_busy),
    .acc_finished_o     (fu_finished),
    .buff_id_i          (fu_buff_id),
    .fill_value_i       (sch_fill_value),
    .fill_block_size_i  (sch_fill_block_size),
    .fu_bba_rd_o        (fu_bba_mem_rd_o),
    .fu_bba_wait_i      (fu_bba_mem_wait_i),
    .fu_bba_addr_o      (fu_bba_mem_addr_o),
    .fu_bba_data_i      (fu_bba_mem_data_i),
    // snnAcc0 — fill-only (dual-port write) buses
    .s0_weight_wr_o     (s0_weight_mem_wr_o),
    .s0_weight_addr_o   (s0_weight_mem_wr_addr_o),
    .s0_weight_data_o   (s0_weight_mem_wr_data_o),
    .s0_weight_wait_i   (s0_weight_mem_wr_wait_i),
    // act/spike/syn_curr + Hadamard buffers fill via shared_data_* (below).
    .s0_bias_curr_wr_o  (s0_bias_curr_mem_wr_o),
    .s0_bias_curr_addr_o(s0_bias_curr_mem_wr_addr_o),
    .s0_bias_curr_data_o(s0_bias_curr_mem_wr_data_o),
    .s0_bias_curr_wait_i(s0_bias_curr_mem_wr_wait_i),
    .s0_thresh_wr_o     (s0_thresh_mem_wr_o),
    .s0_thresh_addr_o   (s0_thresh_mem_wr_addr_o),
    .s0_thresh_data_o   (s0_thresh_mem_wr_data_o),
    .s0_thresh_wait_i   (s0_thresh_mem_wr_wait_i),
    .s0_pot_wr_o        (fu_s0_pot_wr),
    .s0_pot_addr_o      (fu_s0_pot_addr),
    .s0_pot_data_o      (fu_s0_pot_data),
    .s0_pot_wait_i      (fu_s0_pot_wait),
    // snnAcc1 — fill-only buses
    .s1_weight_wr_o     (s1_weight_mem_wr_o),
    .s1_weight_addr_o   (s1_weight_mem_wr_addr_o),
    .s1_weight_data_o   (s1_weight_mem_wr_data_o),
    .s1_weight_wait_i   (s1_weight_mem_wr_wait_i),
    .s1_bias_curr_wr_o  (s1_bias_curr_mem_wr_o),
    .s1_bias_curr_addr_o(s1_bias_curr_mem_wr_addr_o),
    .s1_bias_curr_data_o(s1_bias_curr_mem_wr_data_o),
    .s1_bias_curr_wait_i(s1_bias_curr_mem_wr_wait_i),
    .s1_thresh_wr_o     (s1_thresh_mem_wr_o),
    .s1_thresh_addr_o   (s1_thresh_mem_wr_addr_o),
    .s1_thresh_data_o   (s1_thresh_mem_wr_data_o),
    .s1_thresh_wait_i   (s1_thresh_mem_wr_wait_i),
    .s1_pot_wr_o        (fu_s1_pot_wr),
    .s1_pot_addr_o      (fu_s1_pot_addr),
    .s1_pot_data_o      (fu_s1_pot_data),
    .s1_pot_wait_i      (fu_s1_pot_wait),
    // annAcc — fill-only buses
    .a0_weight_wr_o     (a0_weight_mem_wr_o),
    .a0_weight_addr_o   (a0_weight_mem_wr_addr_o),
    .a0_weight_data_o   (a0_weight_mem_wr_data_o),
    .a0_weight_wait_i   (a0_weight_mem_wr_wait_i),
    .a0_bias_curr_wr_o  (a0_bias_curr_mem_wr_o),
    .a0_bias_curr_addr_o(a0_bias_curr_mem_wr_addr_o),
    .a0_bias_curr_data_o(a0_bias_curr_mem_wr_data_o),
    .a0_bias_curr_wait_i(a0_bias_curr_mem_wr_wait_i),
    .a0_thresh_wr_o     (a0_thresh_mem_wr_o),
    .a0_thresh_addr_o   (a0_thresh_mem_wr_addr_o),
    .a0_thresh_data_o   (a0_thresh_mem_wr_data_o),
    .a0_thresh_wait_i   (a0_thresh_mem_wr_wait_i),
    .a0_pot_wr_o        (fu_a0_pot_wr),
    .a0_pot_addr_o      (fu_a0_pot_addr),
    .a0_pot_data_o      (fu_a0_pot_data),
    .a0_pot_wait_i      (fu_a0_pot_wait),
    // Shared act/spike/syn_curr + Hadamard pool — routed via shared_pool.
    .shared_data_wr_o   (fu_shared_wr),
    .shared_data_addr_o (fu_shared_addr),
    .shared_data_data_o (fu_shared_data),
    .shared_data_wait_i (fu_shared_wait)
);

endmodule // flexman
