// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
`timescale 10ps/1ps
`include "../shared/constants.v"

// =============================================================================
// fmi_top.v — dedicated top level for the FMI application.
// Authors: Simon Davidson & Claude   Created: 2026-06-23   Last modified: 2026-07-21
//
// Wires ONE multi-channel recurrent SNN accelerator (acc_fmiSnnMC_processor,
// 1920 neurons, <=64 channels, 120->60 conv downsampling) into the proven
// FlexMan fabric: scheduler + config_manager + shared_pool. Derived from
// example_app_top.v (itself flexman.v's fabric) but STRIPPED to a single
// compute accelerator — no snn1/ann/hadamard, and (this round) no fill_unit.
//
// Accelerator count (Stage B decision):
//   NUM_HW_ACCELERATORS = 2 = 1 compute (acc 0 = u_fmi) + 1 RESERVED fill slot
//   (acc 1 = FILL_ACC_ID). No fill_unit is instantiated this round — its write
//   ports are hardcoded to the snn/ann/hadamard memory set and extending it for
//   FMI's per-neuron memories is a Stage-C task. The fill slot is reserved so
//   the scheduler's FILL-dispatch gating stays intact; its busy/finished/result
//   inputs are tied to 0 (a fill dispatch would never complete, so the FMI
//   layer program simply must not issue one until Stage C).
//
// config_manager floor: config_manager requires NUM_ACC >= 2 (its
// TGT_ACC_ID_SZ = $clog2(NUM_ACC)). With a single compute accelerator we pass
// NUM_ACC = 2 (the floor) but wire only index [0]; index [1] is never targeted
// (only acc 0 is ever dispatched to config_manager) so its cfg/bba strobes stay
// dead.
//
// AXI / memory conventions are identical to example_app_top.v's u_snn0:
//   - One host AXI bus broadcast to scheduler + config_manager + the accelerator;
//     acks OR'd; reads from the scheduler status regs or the POOL_RD_BASE window.
//   - config_manager pushes WORDS_PER_CONFIG=16 sequential words; a per-acc word
//     counter generates the AXI address (FMI_CFG_BASE + word*4). Boot regs
//     (0x5C+: weight_idx_sz / kernel params) are written once by the host via
//     direct AXI (ext-req path), outside the 16-word config window.
//   - BBA register bank: config_manager sends 4 buffer base-addresses
//     ([0]=tgt,[1]=src1,[2]=src2,[3]=src3) captured into the sp_*/np_* addr ports.
//
// Memory set (12 interfaces): act/syn_curr/spike are POOL-shared (shared_pool,
// 4 banks); the other 9 (weight, thresh, pot, dcy_syn, dcy_mem, ada, b_eff,
// dcy_ada, scl_ada) are dedicated and exposed straight to top-level ports
// (loaded externally via the SRAMs' second port, outside this module). No
// bias_curr (FMI folds bias into the per-neuron state). With no fill_unit there
// is no acc-vs-fill arbitration: dedicated ports pass straight through.
// =============================================================================
module fmi_top #(
    // ── Address map ──────────────────────────────────────────────────────────
    parameter [31:0] FMI_CFG_BASE           = 32'h1000_0000,
    parameter [31:0] POOL_RD_BASE           = 32'h1010_0000,

    parameter [31:0] CM_CFG_MEM_ADDR        = 32'hA000_0000,
    parameter [31:0] CM_CFG_MEM_MASK        = 32'hFF00_0000,
    parameter [31:0] CM_BBA_MEM_ADDR        = 32'hB000_0000,
    parameter [31:0] CM_BBA_MEM_MASK        = 32'hFF00_0000,

    parameter [31:0] SCH_PROG_MEM_ADDR      = 32'hD000_0000,
    parameter [31:0] SCH_PROG_MEM_MASK      = 32'hFF00_0000,

    // ── Fabric sizing ────────────────────────────────────────────────────────
    parameter NUM_BUFFERS         = 16,
    parameter NUM_HW_ACCELERATORS = 2,    // 1 compute + 1 reserved fill slot
    parameter WORDS_PER_CONFIG    = 16,   // must be a power of 2 >= 2

    parameter CFG_ID_SZ           = 7,
    parameter BUFF_INDX_SZ        = 4,    // = $clog2(NUM_BUFFERS)
    parameter TGT_ACC_SZ          = 3,    // holds FILL_ACC_ID
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

    // ── Program memory (scheduler read + AXI write) ──────────────────────────
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

    // ── Buffer base-address memory (config_manager read/write) ───────────────
    output wire         bba_mem_rd_o,
    input  wire         bba_mem_wait_i,
    output wire  [31:0] bba_mem_addr_o,
    input  wire  [31:0] bba_mem_data_i,
    output wire         bba_mem_wr_o,
    output wire  [31:0] bba_mem_wr_addr_o,
    output wire  [31:0] bba_mem_wr_data_o,

    // ── Scheduler NXT pulses ─────────────────────────────────────────────────
    output wire                   nxt_input_pulse_o,
    output wire                   nxt_output_pulse_o,

    // ── Config manager status ────────────────────────────────────────────────
    output wire cm_config_finished_o,

    // ── FMI dedicated accelerator memory ports ───────────────────────────────
    // Read-only-from-acc (external loader writes via the SRAM's 2nd port).
    output wire                  weight_mem_rd_o,
    input  wire                  weight_mem_wait_i,
    output wire [`ADDR_SIZE-1:0] weight_mem_addr_o,
    input  wire  [`WTD_BITS-1:0] weight_mem_data_i,

    output wire                  thresh_mem_rd_o,
    input  wire                  thresh_mem_wait_i,
    output wire [`ADDR_SIZE-1:0] thresh_mem_addr_o,
    input  wire  [`WTD_BITS-1:0] thresh_mem_data_i,

    output wire                  dcy_syn_mem_rd_o,
    input  wire                  dcy_syn_mem_wait_i,
    output wire [`ADDR_SIZE-1:0] dcy_syn_mem_addr_o,
    input  wire           [31:0] dcy_syn_mem_data_i,

    output wire                  dcy_mem_mem_rd_o,
    input  wire                  dcy_mem_mem_wait_i,
    output wire [`ADDR_SIZE-1:0] dcy_mem_mem_addr_o,
    input  wire           [31:0] dcy_mem_mem_data_i,

    output wire                  b_eff_mem_rd_o,
    input  wire                  b_eff_mem_wait_i,
    output wire [`ADDR_SIZE-1:0] b_eff_mem_addr_o,
    input  wire           [31:0] b_eff_mem_data_i,

    output wire                  dcy_ada_mem_rd_o,
    input  wire                  dcy_ada_mem_wait_i,
    output wire [`ADDR_SIZE-1:0] dcy_ada_mem_addr_o,
    input  wire           [31:0] dcy_ada_mem_data_i,

    output wire                  scl_ada_mem_rd_o,
    input  wire                  scl_ada_mem_wait_i,
    output wire [`ADDR_SIZE-1:0] scl_ada_mem_addr_o,
    input  wire           [31:0] scl_ada_mem_data_i,

    // Read/write-from-acc (pot, ada): no fill ⇒ no arbitration, straight through.
    output wire                  pot_mem_wr_o,
    output wire                  pot_mem_rd_o,
    input  wire                  pot_mem_wait_i,
    output wire [`ADDR_SIZE-1:0] pot_mem_addr_o,
    output wire  [`POT_BITS-1:0] pot_mem_data_o,
    input  wire  [`POT_BITS-1:0] pot_mem_data_i,

    output wire                  ada_mem_wr_o,
    output wire                  ada_mem_rd_o,
    input  wire                  ada_mem_wait_i,
    output wire [`ADDR_SIZE-1:0] ada_mem_addr_o,
    output wire           [31:0] ada_mem_data_o,
    input  wire           [31:0] ada_mem_data_i,

    // ── Shared act/spike/syn_curr data pool (4 interleaved banks) ────────────
    // act (rd) / spike (wr) / syn_curr (rd+wr) striped across four single-port
    // banks by the low 2 address bits; arbitrated by the shared_pool instance.
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
localparam ENTRY_DATA_SZ = 3*SLOT_SHORT_SZ + 3*SLOT_LONG_SZ
                           + 1 + TGT_ACC_SZ + CFG_ID_SZ;                      // 59 (CFG_ID_SZ=7)
localparam LONG_BASE     = 3 * SLOT_SHORT_SZ;                                 // 18
// Entry-field offsets MOVE with TGT_COUNT_SZ (via SLOT_LONG_SZ) — derived,
// never hardcoded (TGT_COUNT_SZ=3 offsets hid in cm_buffer_info, fixed 2026-07-10).
localparam E_COLOUR    = 3*SLOT_SHORT_SZ + 3*SLOT_LONG_SZ;                    // 48
localparam E_ACC_START = E_COLOUR + 1;                                         // 49
localparam E_CFG_START = E_ACC_START + TGT_ACC_SZ;                            // 52
// Long-slot buffer-id offsets (cm_buffer_info below):
localparam LS0_ID_START = LONG_BASE                  + MODE_SZ;               // 20
localparam LS1_ID_START = LONG_BASE + 1*SLOT_LONG_SZ + MODE_SZ;               // 30
localparam LS2_ID_START = LONG_BASE + 2*SLOT_LONG_SZ + MODE_SZ;               // 40

// fill_unit occupies the last accelerator slot; a TASK's acc_id field can only
// reach the compute accelerators, so FILL_ACC_ID is unreachable by normal TASKs.
localparam [TGT_ACC_SZ-1:0] FILL_ACC_ID = NUM_HW_ACCELERATORS - 1;           // 1

localparam LOG2_WPC    = 4;   // $clog2(WORDS_PER_CONFIG=16)
localparam BBA_CNT_SZ  = 2;   // $clog2(NUM_BBA_SENDS=4)

// Computation accelerators (excludes the reserved fill slot)
localparam NUM_COMP_ACC = NUM_HW_ACCELERATORS - 1;                            // 1
// config_manager floor: NUM_ACC must be >= 2. Only index 0 is ever targeted.
localparam CONFIG_NUM_ACC = 2;
localparam CM_TGT_ACC_SZ  = 1;   // = $clog2(CONFIG_NUM_ACC=2)

// ─── Scheduler outputs ───────────────────────────────────────────────────────
wire                      sch_ack;
wire               [31:0] sch_data_o;
wire [ENTRY_DATA_SZ-1:0]  sch_buffer_info;
wire [TGT_ACC_SZ-1:0]     sch_target_acc;
wire                      sch_start_new_block;
wire               [31:0] sch_fill_value;
wire               [19:0] sch_fill_block_size;

// FILL dispatch gating — keeps a (reserved) fill dispatch off the compute path.
wire is_fill_dispatch = sch_start_new_block & (sch_target_acc == FILL_ACC_ID);
wire                       acc_start_new_block;       // assigned after cm_tgt_acc
wire [CM_TGT_ACC_SZ-1:0]   cm_tgt_acc_narrow;         // raw output from config_manager
wire [TGT_ACC_SZ-1:0]      cm_tgt_acc =
    {{(TGT_ACC_SZ - CM_TGT_ACC_SZ){1'b0}}, cm_tgt_acc_narrow};   // zero-extend

// ─── Config manager outputs ───────────────────────────────────────────────────
wire                         cm_ack;
wire [CONFIG_NUM_ACC-1:0]    cm_config_wr;
wire [CONFIG_NUM_ACC-1:0]    cm_buff_base_wr;
wire [31:0]                  cm_config_data;
wire [31:0]                  cm_buff_base_data;
wire                         cm_busy;

// config_manager start is suppressed for FILL dispatches.
wire cm_start_new_block = sch_start_new_block & ~is_fill_dispatch;

// Compute-acc dispatch fires when config_manager finishes pushing cfg + bba.
assign acc_start_new_block = cm_config_finished_o;

// ─── Accelerator AXI (config write mux output) ────────────────────────────────
wire  fmi_sys_req, fmi_sys_ack;
wire [31:0] fmi_sys_addr, fmi_sys_data_in;

// ─── Accelerator status ──────────────────────────────────────────────────────
wire  fmi_busy, fmi_finished, fmi_result;

// ─── Buffer address wires (BBA register bank outputs) ────────────────────────
wire [`PIN_BITS-1:0] fmi_sp_tgt,  fmi_sp_src1, fmi_sp_src2, fmi_sp_src3;
wire [`PIN_BITS-1:0] fmi_np_tgt,  fmi_np_src1, fmi_np_src2, fmi_np_src3;

// ─── Config write counter (one per computation accelerator = acc 0) ──────────
reg [LOG2_WPC-1:0] cfg_cnt_0;
always @(posedge clk or posedge reset) begin
    if (reset)
        cfg_cnt_0 <= {LOG2_WPC{1'b0}};
    else if (sch_start_new_block && sch_target_acc == 3'd0)
        cfg_cnt_0 <= {LOG2_WPC{1'b0}};
    else if (cm_config_wr[0])
        cfg_cnt_0 <= cfg_cnt_0 + 1'b1;
end

// ─── AXI mux: config_manager writes take priority over external host ──────────
wire fmi_ext_req = sys_req_i & (sys_addr_i[31:16] == FMI_CFG_BASE[31:16]);

assign fmi_sys_req     = cm_config_wr[0] ? 1'b1 : fmi_ext_req;
assign fmi_sys_addr    = cm_config_wr[0]
                       ? (FMI_CFG_BASE | {{26{1'b0}}, cfg_cnt_0, 2'b00})
                       : sys_addr_i;
assign fmi_sys_data_in = cm_config_wr[0] ? cm_config_data : sys_data_i;

wire fmi_ext_ack = fmi_sys_ack & ~cm_config_wr[0];

wire [CONFIG_NUM_ACC-1:0] cm_config_wait    = {CONFIG_NUM_ACC{1'b0}};
wire [CONFIG_NUM_ACC-1:0] cm_buff_base_wait = {CONFIG_NUM_ACC{1'b0}};

// ─── BBA register bank (acc 0) ────────────────────────────────────────────────
reg [BBA_CNT_SZ-1:0] bba_cnt_0;
reg [31:0] bba_r0 [0:3];

always @(posedge clk or posedge reset) begin
    if (reset)
        bba_cnt_0 <= 2'b00;
    else if (sch_start_new_block && sch_target_acc == 3'd0)
        bba_cnt_0 <= 2'b00;
    else if (cm_buff_base_wr[0])
        bba_cnt_0 <= bba_cnt_0 + 1'b1;
end
always @(posedge clk) begin
    if (cm_buff_base_wr[0])
        bba_r0[bba_cnt_0] <= cm_buff_base_data;
end

assign fmi_sp_tgt  = bba_r0[0][`PIN_BITS-1:0];
assign fmi_sp_src1 = bba_r0[1][`PIN_BITS-1:0];
assign fmi_sp_src2 = bba_r0[2][`PIN_BITS-1:0];
assign fmi_sp_src3 = bba_r0[3][`PIN_BITS-1:0];
assign fmi_np_tgt  = bba_r0[0][`PIN_BITS-1:0];
assign fmi_np_src1 = bba_r0[1][`PIN_BITS-1:0];
assign fmi_np_src2 = bba_r0[2][`PIN_BITS-1:0];
assign fmi_np_src3 = bba_r0[3][`PIN_BITS-1:0];

// ─── Buffer info extraction for config_manager ────────────────────────────────
//   buffer_info_i[3:0]   = src1 id  (long  slot 0 id @ LS0_ID_START)
//   buffer_info_i[7:4]   = src2 id  (long  slot 1 id @ LS1_ID_START)
//   buffer_info_i[11:8]  = src3 id  (long  slot 2 id @ LS2_ID_START)
//   buffer_info_i[15:12] = tgt  id  (short slot 0 id @ MODE_SZ)
wire [`SCH_ENTRY_SZ-1:0] cm_buffer_info;
assign cm_buffer_info = {16'b0,
                          sch_buffer_info[MODE_SZ +: BUFF_INDX_SZ],       // tgt:  short slot 0 id
                          sch_buffer_info[LS2_ID_START +: BUFF_INDX_SZ],  // src3: long  slot 2 id
                          sch_buffer_info[LS1_ID_START +: BUFF_INDX_SZ],  // src2: long  slot 1 id
                          sch_buffer_info[LS0_ID_START +: BUFF_INDX_SZ]}; // src1: long  slot 0 id

wire [CFG_ID_SZ-1:0] cm_config_id = sch_buffer_info[E_CFG_START +: CFG_ID_SZ];

// ─── Accelerator-side internal wires for the pool-shared memories ─────────────
// syn_curr (rd+wr), act (rd), spike (wr) route to shared_pool.
wire                  acc_syn_curr_wr,   acc_syn_curr_rd;
wire [`ADDR_SIZE-1:0] acc_syn_curr_addr;
wire  [`POT_BITS-1:0] acc_syn_curr_data;
wire  [`POT_BITS-1:0] acc_syn_curr_data_i;
wire                  acc_syn_curr_wait;
wire                  acc_spike_wr;
wire [`ADDR_SIZE-1:0] acc_spike_addr;
wire  [`ACT_BITS-1:0] acc_spike_data;
wire                  acc_spike_wait;
wire                  acc_act_req;
wire [`ADDR_SIZE-1:0] acc_act_addr;
wire  [`ACT_BITS-1:0] acc_act_data;
wire                  acc_act_wait;

// ─── Shared act/spike/syn_curr pool — shared_pool (4 banks) ──────────────────
// Requesters, strict index (0 = highest priority, NUM_REQ-1 = lowest):
//   0 syn_curr   1 act   2 spike   3 pool_rd (host AXI readback, lowest)
localparam SP_NREQ  = 4;
localparam SP_NBANK = 4;

wire [SP_NREQ-1:0]            sp_req_act;
wire [SP_NREQ-1:0]            sp_req_rd;
wire [SP_NREQ-1:0]            sp_req_wr;
wire [SP_NREQ*`ADDR_SIZE-1:0] sp_req_addr;
wire [SP_NREQ*32-1:0]         sp_req_wdata;
wire [SP_NREQ-1:0]            sp_req_wait;
wire [SP_NREQ*32-1:0]         sp_req_rdata;

// ── Host pool-readback window (requester index SP_NREQ-1, lowest priority) ──
wire pool_rd_sel = sys_req_i & (sys_addr_i[31:20] == POOL_RD_BASE[31:20]);
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

// Host AXI response: scheduler status regs OR the pool-readback path.
assign sys_ack_o  = sch_ack | cm_ack | fmi_ext_ack | pool_rd_ack;
assign sys_data_o = pool_rd_ack ? prd_data_r : sch_data_o;

assign sp_req_act = { pool_rd_req_active,
                      acc_spike_wr, acc_act_req,
                      (acc_syn_curr_rd | acc_syn_curr_wr) };
assign sp_req_rd  = { pool_rd_req_active,
                      1'b0, acc_act_req, acc_syn_curr_rd };
assign sp_req_wr  = { 1'b0,
                      acc_spike_wr, 1'b0, acc_syn_curr_wr };
assign sp_req_addr = { pool_rd_addr,
                       acc_spike_addr, acc_act_addr, acc_syn_curr_addr };
assign sp_req_wdata = { 32'b0,
                        acc_spike_data, 32'b0, acc_syn_curr_data };

shared_pool #(
    .NUM_BANKS (SP_NBANK),
    .NUM_REQ   (SP_NREQ),
    .ADDR_W    (`ADDR_SIZE),
    .DATA_W    (32),
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
assign acc_syn_curr_wait   = sp_req_wait[0];
assign acc_act_wait        = sp_req_wait[1];
assign acc_spike_wait      = sp_req_wait[2];
assign acc_syn_curr_data_i = sp_req_rdata[0*32 +: 32];
assign acc_act_data        = sp_req_rdata[1*32 +: 32];

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
    // Reserved fill slot (acc 1) tied to 0 — no fill_unit this round.
    .acc_busy_i          ({1'b0, fmi_busy}),
    .acc_finished_i      ({1'b0, fmi_finished}),
    .acc_result_i        ({1'b0, fmi_result}),
    .acc_ready_next_i    ({NUM_HW_ACCELERATORS{1'b0}}),   // no overlapped accelerators
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
// NUM_ACC=2 (config_manager floor); only index 0 (u_fmi) is ever targeted.
config_manager #(
    .NUM_ACC                 (CONFIG_NUM_ACC),
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
    .target_acc_i         (sch_target_acc[CM_TGT_ACC_SZ-1:0]),   // always 0
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

// =============================================================================
// PER-APPLICATION BUS SIZING — FMI recurrent SNN layer. The accelerator
// instance below is sized from these localparams; they mirror the T9 (FMI-scale:
// 1920 neurons, 16->32 channels, 120->60 conv) parameter set validated bit-exact
// by fmiSnnAccMC/tb_acc_fmiSnnMC_processor.v. Real per-layer values (kernel/tile
// counts) are finalized in Stage C (convert_model). cin/cout VALUES are runtime
// config (S2 word 0x38); the *_SZ params below are only the field widths.
// =============================================================================
localparam FMI_NUM_TIMESTEPS      = 32;
localparam FMI_X_INPUT_SZ         = 8;
localparam FMI_X_OUTPUT_SZ        = 8;
localparam FMI_X_KERNEL_SZ        = 5;   // up to 31: con6 FC-via-conv full-width kernel=30
localparam FMI_X_KERNEL_OFF_SZ    = 3;
localparam FMI_X_STEP_SZ          = 3;
localparam FMI_ELEMS_PER_ROW      = 4;
localparam FMI_ROWS_PER_NEURON    = 4;
localparam FMI_ELEM_SZ            = 8;
localparam FMI_ACT_SLICE_SZ       = 5;   // 32-bit act slices (real-MAC ready); legacy spikes use runtime slice code 0
localparam FMI_ACT_IDX_SZ         = 11;   // input-neuron flat index  (>1024)
localparam FMI_ACT_DATA_IDX_SZ    = 11;  // flat input-index width (full-mode FC base): 11 spans con6's 1920 inputs
localparam FMI_WEIGHT_SLICE_SZ    = 5;
localparam FMI_WEIGHT_IDX_SZ      = 16;
localparam FMI_WEIGHT_DATA_IDX_SZ = 5;
localparam FMI_SYN_CURR_IDX_SZ    = 12;
localparam FMI_SYN_CURR_DATA_IDX_SZ = 5;
localparam FMI_SYN_CURR_SLICE_SZ  = 3;
localparam FMI_SYN_CURR_SLICE_BITS = 32;
localparam FMI_BIAS_CURR_IDX_SZ   = 10;
localparam FMI_BIAS_CURR_DATA_IDX_SZ = 5;
localparam FMI_BIAS_CURR_SLICE_SZ = 3;
localparam FMI_BIAS_CURR_SLICE_BITS = 8;
localparam FMI_NEURON_IDX_SZ      = 11;   // output neurons: 1920 -> 11 bits
localparam FMI_POT_IDX_SZ         = 12;
localparam FMI_POT_DATA_IDX_SZ    = 5;
localparam FMI_POT_SLICE_SZ       = 3;
localparam FMI_POT_SLICE_BITS     = 32;
localparam FMI_SPIKE_IDX_SZ       = 12;
localparam FMI_SPIKE_DATA_IDX_SZ  = 5;
localparam FMI_SPIKE_SLICE_SZ     = 3;
localparam FMI_SPIKE_SLICE_BITS   = 8;
localparam FMI_CIN_SZ             = 7;    // <=64 input channels
localparam FMI_COUT_SZ            = 7;    // <=64 output channels

// ─── FMI multi-channel recurrent accelerator (TGT_ACC_ID = 0) ────────────────
acc_fmiSnnMC_processor #(
    .TGT_ACC_ID               (0),
    .TGT_CONFIG_BASE_ADDR     (FMI_CFG_BASE),
    .SP_NUM_TIMESTEPS         (FMI_NUM_TIMESTEPS),
    .SP_X_INPUT_SZ            (FMI_X_INPUT_SZ),
    .SP_X_OUTPUT_SZ          (FMI_X_OUTPUT_SZ),
    .SP_X_KERNEL_SZ          (FMI_X_KERNEL_SZ),
    .SP_X_KERNEL_OFF_SZ      (FMI_X_KERNEL_OFF_SZ),
    .SP_X_STEP_SZ            (FMI_X_STEP_SZ),
    .SP_ELEMS_PER_ROW        (FMI_ELEMS_PER_ROW),
    .SP_ROWS_PER_NEURON      (FMI_ROWS_PER_NEURON),
    .SP_TIMESTEP_SZ          (10),
    .SP_IN_DATA_BITS         (32),
    .SP_ELEM_SZ              (FMI_ELEM_SZ),
    .SP_ACT_SLICE_SZ         (FMI_ACT_SLICE_SZ),
    .SP_ACT_IDX_SZ           (FMI_ACT_IDX_SZ),
    .SP_ACT_DATA_IDX_SZ      (FMI_ACT_DATA_IDX_SZ),
    .SP_WEIGHT_ENTRY_BITS    (8),
    .SP_WEIGHT_IDX_SZ        (FMI_WEIGHT_IDX_SZ),
    .SP_WEIGHT_SLICE_SZ      (FMI_WEIGHT_SLICE_SZ),
    .SP_WEIGHT_DATA_IDX_SZ   (FMI_WEIGHT_DATA_IDX_SZ),
    .SP_SYN_CURR_IDX_SZ      (FMI_SYN_CURR_IDX_SZ),
    .SP_SYN_CURR_DATA_IDX_SZ (FMI_SYN_CURR_DATA_IDX_SZ),
    .SP_SYN_CURR_SLICE_SZ    (FMI_SYN_CURR_SLICE_SZ),
    .SP_SYN_CURR_SLICE_BITS  (FMI_SYN_CURR_SLICE_BITS),
    .SP_BIAS_CURR_IDX_SZ     (FMI_BIAS_CURR_IDX_SZ),
    .SP_BIAS_CURR_DATA_IDX_SZ(FMI_BIAS_CURR_DATA_IDX_SZ),
    .SP_BIAS_CURR_SLICE_SZ   (FMI_BIAS_CURR_SLICE_SZ),
    .SP_BIAS_CURR_SLICE_BITS (FMI_BIAS_CURR_SLICE_BITS),
    .NP_NUM_TIMESTEPS        (FMI_NUM_TIMESTEPS),
    .NP_TIMESTEP_SZ          (10),
    .NP_IN_DATA_BITS         (32),
    .NP_NEURON_IDX_SZ        (FMI_NEURON_IDX_SZ),
    .NP_SYN_CURR_IDX_SZ      (FMI_SYN_CURR_IDX_SZ),
    .NP_SYN_CURR_DATA_IDX_SZ (FMI_SYN_CURR_DATA_IDX_SZ),
    .NP_SYN_CURR_SLICE_SZ    (FMI_SYN_CURR_SLICE_SZ),
    .NP_SYN_CURR_SLICE_BITS  (FMI_SYN_CURR_SLICE_BITS),
    .NP_POT_IDX_SZ           (FMI_POT_IDX_SZ),
    .NP_POT_DATA_IDX_SZ      (FMI_POT_DATA_IDX_SZ),
    .NP_POT_SLICE_SZ         (FMI_POT_SLICE_SZ),
    .NP_POT_SLICE_BITS       (FMI_POT_SLICE_BITS),
    .NP_SPIKE_IDX_SZ         (FMI_SPIKE_IDX_SZ),
    .NP_SPIKE_DATA_IDX_SZ    (FMI_SPIKE_DATA_IDX_SZ),
    .NP_SPIKE_SLICE_SZ       (FMI_SPIKE_SLICE_SZ),
    .NP_SPIKE_SLICE_BITS     (FMI_SPIKE_SLICE_BITS),
    .SP_CIN_SZ               (FMI_CIN_SZ),
    .SP_COUT_SZ              (FMI_COUT_SZ),
    .MEM_ADDR_BITS           (`ADDR_SIZE)
) u_fmi (
    .clk                   (clk),
    .reset                 (reset),
    .sys_req_i             (fmi_sys_req),
    .sys_ack_o             (fmi_sys_ack),
    .sys_addr_i            (fmi_sys_addr),
    .sys_data_i            (fmi_sys_data_in),
    .start_new_block_i     (acc_start_new_block),
    .target_acc_i          (cm_tgt_acc),
    .buffer_info_i         (sch_buffer_info[`SCH_ENTRY_SZ-1:0]),
    .spike_proc_finished_o (fmi_result),
    .acc_busy_o            (fmi_busy),
    .acc_finished_o        (fmi_finished),
    .sp_src1_buff_addr_i   (fmi_sp_src1),
    .sp_src2_buff_addr_i   (fmi_sp_src2),
    .sp_src3_buff_addr_i   (fmi_sp_src3),
    .sp_tgt_buff_addr_i    (fmi_sp_tgt),
    .sp_weight_row_len_i   ({`PIN_BITS{1'b0}}),
    .np_src1_buff_addr_i   (fmi_np_src1),
    .np_src2_buff_addr_i   (fmi_np_src2),
    .np_src3_buff_addr_i   (fmi_np_src3),
    .np_tgt_buff_addr_i    (fmi_np_tgt),
    .np_weight_row_len_i   ({`PIN_BITS{1'b0}}),
    // weight (rd) — dedicated, straight to top port
    .weight_mem_rd_o       (weight_mem_rd_o),
    .weight_mem_wait_i     (weight_mem_wait_i),
    .weight_mem_addr_o     (weight_mem_addr_o),
    .weight_mem_data_i     (weight_mem_data_i),
    // act (rd) — pooled
    .act_mem_req_o         (acc_act_req),
    .act_mem_wait_i        (acc_act_wait),
    .act_mem_addr_o        (acc_act_addr),
    .act_mem_data_i        (acc_act_data),
    // syn_curr (rd+wr) — pooled
    .syn_curr_mem_wr_o     (acc_syn_curr_wr),
    .syn_curr_mem_rd_o     (acc_syn_curr_rd),
    .syn_curr_mem_wait_i   (acc_syn_curr_wait),
    .syn_curr_mem_addr_o   (acc_syn_curr_addr),
    .syn_curr_mem_data_o   (acc_syn_curr_data),
    .syn_curr_mem_data_i   (acc_syn_curr_data_i),
    // thresh (rd) — dedicated
    .thresh_mem_rd_o       (thresh_mem_rd_o),
    .thresh_mem_wait_i     (thresh_mem_wait_i),
    .thresh_mem_addr_o     (thresh_mem_addr_o),
    .thresh_mem_data_i     (thresh_mem_data_i),
    // pot (rd+wr) — dedicated, straight through (no fill arbitration)
    .pot_mem_wr_o          (pot_mem_wr_o),
    .pot_mem_rd_o          (pot_mem_rd_o),
    .pot_mem_wait_i        (pot_mem_wait_i),
    .pot_mem_addr_o        (pot_mem_addr_o),
    .pot_mem_data_o        (pot_mem_data_o),
    .pot_mem_data_i        (pot_mem_data_i),
    // spike (wr) — pooled
    .spike_mem_wr_o        (acc_spike_wr),
    .spike_mem_wait_i      (acc_spike_wait),
    .spike_mem_addr_o      (acc_spike_addr),
    .spike_mem_data_o      (acc_spike_data),
    // dcy_syn (rd) — dedicated
    .dcy_syn_mem_rd_o      (dcy_syn_mem_rd_o),
    .dcy_syn_mem_wait_i    (dcy_syn_mem_wait_i),
    .dcy_syn_mem_addr_o    (dcy_syn_mem_addr_o),
    .dcy_syn_mem_data_i    (dcy_syn_mem_data_i),
    // dcy_mem (rd) — dedicated
    .dcy_mem_mem_rd_o      (dcy_mem_mem_rd_o),
    .dcy_mem_mem_wait_i    (dcy_mem_mem_wait_i),
    .dcy_mem_mem_addr_o    (dcy_mem_mem_addr_o),
    .dcy_mem_mem_data_i    (dcy_mem_mem_data_i),
    // ada (rd+wr) — dedicated, straight through
    .ada_mem_wr_o          (ada_mem_wr_o),
    .ada_mem_rd_o          (ada_mem_rd_o),
    .ada_mem_wait_i        (ada_mem_wait_i),
    .ada_mem_addr_o        (ada_mem_addr_o),
    .ada_mem_data_o        (ada_mem_data_o),
    .ada_mem_data_i        (ada_mem_data_i),
    // b_eff (rd) — dedicated
    .b_eff_mem_rd_o        (b_eff_mem_rd_o),
    .b_eff_mem_wait_i      (b_eff_mem_wait_i),
    .b_eff_mem_addr_o      (b_eff_mem_addr_o),
    .b_eff_mem_data_i      (b_eff_mem_data_i),
    // dcy_ada (rd) — dedicated
    .dcy_ada_mem_rd_o      (dcy_ada_mem_rd_o),
    .dcy_ada_mem_wait_i    (dcy_ada_mem_wait_i),
    .dcy_ada_mem_addr_o    (dcy_ada_mem_addr_o),
    .dcy_ada_mem_data_i    (dcy_ada_mem_data_i),
    // scl_ada (rd) — dedicated
    .scl_ada_mem_rd_o      (scl_ada_mem_rd_o),
    .scl_ada_mem_wait_i    (scl_ada_mem_wait_i),
    .scl_ada_mem_addr_o    (scl_ada_mem_addr_o),
    .scl_ada_mem_data_i    (scl_ada_mem_data_i)
);

endmodule // fmi_top
