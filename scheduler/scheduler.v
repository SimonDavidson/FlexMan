// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
// Authors: Simon Davidson & Claude | Last modified: 2026-08-22
`include "../shared/constants.v"

`timescale 10ps/1ps

// Slot modes (shared with sch_entry / sch_buffer_state):
`define MODE_UNUSED 2'b00
`define MODE_SRC    2'b01
`define MODE_RW     2'b10
`define MODE_TGT    2'b11

module scheduler
   #(parameter TGT_COUNT_SZ        = 4,
     // WIDE_NTGT=1 enables the three-word "wide" TASK form, selected per
     // instruction by TASK word 1 bit 31 (always reserved and zero until now --
     // ISA_REFERENCE.md:117 -- so enabling this ADDS an instruction form and
     // leaves every existing encoding untouched). Needed because the narrow
     // form's 4-bit ntgt caps usage counts at 15, while Monarch at nblocks=40
     // needs 85.
     //
     // Default 0 = the original decode, bit-identical, and the wide logic
     // constant-folds away entirely.
     //
     // CAVEAT: FILL's widened ntgt (7-bit [15:9], block_size 16-bit [31:16]) is
     // FILL word 1 bit 7 IS free (buf_id occupies [6:3], colour sits at [8]) and could
     // carry a per-instruction selector like TASK's bit 31 — but it is deliberately NOT
     // spent. That bit is free only because BUFF_INDX_SZ=4; at 32 buffers buf_id becomes
     // [7:3] and consumes it. The ceiling is already binding: the N=2 multi-lane program
     // uses 15 of 16 buffer ids (5 singletons + 5 per-lane roles, so N=3 would need 20).
     // Keeping bit 7 reserved preserves the ability to widen buf_id; the price is that
     // FILL's widened ntgt is build-wide, which costs nothing real because only builds
     // that regenerate their programs set WIDE_NTGT=1.
     // A WIDE_NTGT=1 build therefore requires a program whose FILLs use the wide
     // encoder. This is the one part that is not backward compatible.
     //
     // CONTRACT: the toolchain's isa.NTGT_SZ_WIDE must equal TGT_COUNT_SZ.
     parameter WIDE_NTGT           = 0,
     parameter CFG_ID_SZ           = 7,
     // RELAX_LOOP_BARRIER=1 removes the all_tasks_drained gate from NXT and
     // LOOPEND (default 0 = the completion barriers, bit-identical behaviour).
     // With the gate removed, loop iterations overlap: the front-end fetches
     // and the table dispatches the next iteration's tasks (still strictly
     // in-order) while this iteration's tasks are in flight. Data ordering is
     // then enforced entirely by the buffer machinery (claim-at-dispatch,
     // consume-at-completion, colour); the HOST must sequence its windowed
     // I/O off buffer state — rearm inputs when the input buffer is FREE,
     // read outputs when the output buffer goes FULL — instead of treating
     // the (now early) nxt_*_pulse_o as completion barriers.
     parameter RELAX_LOOP_BARRIER  = 0,
     parameter NUM_BUFFERS         = 16,
     parameter COL_BUFF_ID_SZ      = 16,
     parameter NUM_SCH_ENTRIES     = 4,
     parameter NUM_HW_ACCELERATORS = 2,
     parameter PROG_ADDR_BITS      = 10,
     parameter PROG_DATA_BITS      = 32,
     parameter BUFF_INDX_SZ        = $clog2(NUM_BUFFERS),
     // AXI address range for program memory writes (host loads instructions here)
     parameter [31:0] SCH_PROG_MEM_ADDR = 32'hD000_0000,
     parameter [31:0] SCH_PROG_MEM_MASK = 32'hFF00_0000,
     // Six buffer slots: 3 short (mode+id) + 3 long (mode+id+ntgt)
     parameter NUM_SLOTS           = 6,
     parameter MODE_SZ             = 2,
     // One bit per accelerator; 1 = that accelerator may hold TWO tasks in
     // flight, accepting the next while the current one drains its output
     // stage.  Default 0 = original behaviour everywhere.  A masked
     // accelerator must drive acc_ready_next_i.  See acc_hw_buffer_tracker.
     parameter OVERLAP_ACC_MASK    = {NUM_HW_ACCELERATORS{1'b0}},
     // DISJOINT_HINT=1 honours the wide TASK's per-instruction "my buffer writes
     // do not overlap the previous task on my accelerator" bit, letting sch_entry
     // skip the RW needs-full check. Default 0 ignores it. The bit sits just
     // above cfg_id in wide word 3 and is zero in every image generated before
     // 2026-08-22, so honouring it is a no-op on existing programs — the
     // parameter exists to make that explicit rather than implicit.
     // Requires MULTI_WRITER=1 in the buffer entries, else a completing
     // predecessor marks the buffer full while its successor is still writing.
     parameter DISJOINT_HINT       = 0,
     // Must be 1 whenever DISJOINT_HINT is (see buffer_state_entry).
     parameter MULTI_WRITER        = 0,
     // ACC ID width in scheduler table (3 bits to accommodate fill_unit at id 4):
     parameter TGT_ACC_SZ          = 3,
     // Derived entry layout sizes:
     parameter SLOT_SHORT_SZ       = MODE_SZ + BUFF_INDX_SZ,          // 6
     parameter SLOT_LONG_SZ        = MODE_SZ + BUFF_INDX_SZ + TGT_COUNT_SZ, // 10 (default TGT_COUNT_SZ=4)
     parameter ENTRY_DATA_SZ       = 3*SLOT_SHORT_SZ + 3*SLOT_LONG_SZ
                                     + 1           // colour
                                     + TGT_ACC_SZ  // acc_id
                                     + CFG_ID_SZ   // cfg_id
    )
    (input  wire                       clk,
     input  wire                       reset,
     input  wire                       test_stall_pipe,

     // AXI bus interface:
     input  wire                       sys_req_i,
     output wire                       sys_ack_o,
     input  wire   [31:0]              sys_addr_i,
     input  wire   [31:0]              sys_data_i,
     output wire   [31:0]              sys_data_o,

     // Program memory read interface (scheduler fetches instructions):
     output wire [`ADDR_SIZE-1:0]      prog_mem_addr_o,
     input  wire [PROG_DATA_BITS-1:0]  prog_mem_data_i,
     output wire                       prog_mem_req_o,
     input  wire                       prog_mem_wait_i,

     // Program memory write interface (host loads program via AXI):
     output wire                       prog_mem_wr_o,
     output wire [PROG_ADDR_BITS-1:0]  prog_mem_wr_addr_o,
     output wire [PROG_DATA_BITS-1:0]  prog_mem_wr_data_o,
     input  wire                       prog_mem_wr_wait_i,

     // Accelerator interface:
     input wire [NUM_HW_ACCELERATORS-1:0] acc_busy_i,
     input wire [NUM_HW_ACCELERATORS-1:0] acc_finished_i,
     input wire [NUM_HW_ACCELERATORS-1:0] acc_result_i,
     // Only meaningful for accelerators set in OVERLAP_ACC_MASK; tie 0 else.
     input wire [NUM_HW_ACCELERATORS-1:0] acc_ready_next_i,
     output wire                          start_new_block_o,
     output wire [TGT_ACC_SZ-1:0]         target_acc_o,
     output wire [ENTRY_DATA_SZ-1:0]      buffer_info_o,
     output wire                          nxt_input_pulse_o,
     output wire                          nxt_output_pulse_o,
     // fill_unit dispatch parameters (valid cycle of start_new_block_o for FILL):
     output wire [31:0]                   fill_value_o,
     output wire [19:0]                   fill_block_size_o,

     // Back-pressure from config_manager — see sch_table.v. Tie to 1'b0
     // in tops that don't instantiate a config_manager.
     input  wire                          cm_busy_i,

     // ── Debug observability (KV260 first-silicon bring-up) ───────────────
     // Everything the host needs to say WHY instruction consumption stopped.
     // Purely additive and combinational off existing state: tops that don't
     // want it may leave these unconnected.
     output wire [31:0]                   dbg_frontend_o,
     output wire [31:0]                   dbg_inst_word_o
    );

// ------------------------------------------------------------
// AXI address decode

wire sch_ctrl_wr = sys_req_i & (sys_addr_i[31:29] == 3'b111)
                             & (sys_addr_i[28:25] == 4'b0000);
wire sch_stat_rd = sys_req_i & (sys_addr_i[31:30] == 2'b11)
                             & (sys_addr_i[29:26] == 4'b0001);
wire sch_prog_wr = sys_req_i
                 & ((sys_addr_i & SCH_PROG_MEM_MASK) ==
                    (SCH_PROG_MEM_ADDR & SCH_PROG_MEM_MASK))
                 & ~prog_mem_wr_wait_i;

wire do_start    = sch_ctrl_wr & (sys_addr_i[24:20] == 5'd1);
wire do_continue = sch_ctrl_wr & (sys_addr_i[24:20] == 5'd2);

// SOFT_RESET (ctrl reg 6) — clear all scheduler state WITHOUT touching memory.
//
// Why this exists: after a program STOPs there is no way to run it again. A
// fresh LOAD_PC+START does re-fetch and re-create the table entry, but the
// entry can never become ready, because its target buffers are still FULL from
// the previous run and a TARGET slot requires the buffer FREE. So the only
// recovery was a full logic reset plus reloading every image — including the
// 348k-word weight store — once per utterance.
//
// This clears the buffer state, the task table and the fetch/decode front end,
// leaving program/config/weight memories intact. Restart then becomes
// SOFT_RESET -> MARK_FULL -> LOAD_PC -> START, with the images loaded once per
// session.
//
// HOST RESPONSIBILITY: only issue this when no accelerator is busy (after STOP,
// or PAUSE until status reg 1 reads acc_busy==0). Clearing the trackers while a
// task is in flight would let its later completion update buffers using slot
// information that no longer exists. It is deliberately NOT gated on idle in
// hardware, because recovering a WEDGED machine is the main use and a wedged
// machine must still be clearable.
//
// Everything below is inert unless this register is written, so tops that never
// use it are bit-identical.
wire do_soft_reset = sch_ctrl_wr & (sys_addr_i[24:20] == 5'd6);
wire sch_clr       = reset | do_soft_reset;
// MARK_BUFF_FULL (reg 5): data[BUFF_INDX_SZ-1:0]=buf_id, data[BUFF_INDX_SZ+:TGT_COUNT_SZ]=usage
wire do_mark_full       = sch_ctrl_wr & (sys_addr_i[24:20] == 5'd5);
wire [BUFF_INDX_SZ-1:0] mark_full_id  = sys_data_i[BUFF_INDX_SZ-1:0];
wire [TGT_COUNT_SZ-1:0] mark_full_cnt = sys_data_i[BUFF_INDX_SZ +: TGT_COUNT_SZ];

// ------------------------------------------------------------
// Entry data field offsets (lsb first, matching sch_entry):
//   Slots 0-2: [mode(2), id(4)] × 3  =  3 × 6 = 18 bits
//   Slots 3-5: [mode(2), id(4), ntgt(TGT_COUNT_SZ)] × 3 = 3 × 10 = 30 bits (default TGT_COUNT_SZ=4)
//   colour(1), acc_id(TGT_ACC_SZ=3), cfg_id(CFG_ID_SZ=7)
localparam LONG_BASE   = 3 * SLOT_SHORT_SZ;           // 18
localparam E_COLOUR    = LONG_BASE + 3 * SLOT_LONG_SZ; // 48 (default TGT_COUNT_SZ=4)
localparam E_ACC_START = E_COLOUR + 1;                 // 49
localparam E_CFG_START = E_ACC_START + TGT_ACC_SZ;    // 52 (TGT_ACC_SZ=3)

// Instruction opcodes:
localparam INST_TASK    = 3'b000;
localparam INST_JUMP    = 3'b001;
localparam INST_STOP    = 3'b010;
localparam INST_CHECK   = 3'b011;
localparam INST_NXT     = 3'b100;
localparam INST_FILL    = 3'b101;
localparam INST_LOOP    = 3'b110;
localparam INST_LOOPEND = 3'b111;

// fill_unit is assigned the highest accelerator slot (NUM_HW_ACCELERATORS-1).
// TASK instructions only encode 2-bit acc_id [4:3] so they can never target
// this slot; fill_unit is exclusively reached via FILL instructions.
localparam [TGT_ACC_SZ-1:0] FILL_ACC_ID = NUM_HW_ACCELERATORS - 1;

localparam NUM_LOOPS   = 8;
localparam LOOP_ID_SZ  = 3;
localparam LOOP_CNT_SZ = 26;

// ------------------------------------------------------------
// Wires / regs

wire                            start_new_task;
reg  [ENTRY_DATA_SZ-1:0]        d;
wire [ENTRY_DATA_SZ-1:0]        new_entry_data;
wire                            new_entry_hint;
wire [ENTRY_DATA_SZ-1:0]        entry_data_to_be_launched;
wire [TGT_ACC_SZ-1:0]           to_launch_acc_hw_id;   // declared here for
                                                       // dispatch_in_flight_r below

wire [COL_BUFF_ID_SZ-1:0]       buff_full;
wire [COL_BUFF_ID_SZ-1:0]       buff_free;
wire [COL_BUFF_ID_SZ-1:0]       buff_colour;
wire [NUM_BUFFERS-1:0]          target_status;
wire [NUM_HW_ACCELERATORS-1:0]  acc_available;

wire load_new_entry;
wire table_slot_free;
wire table_empty;
wire [NUM_SCH_ENTRIES-1:0] dbg_entry_valid;
wire [NUM_SCH_ENTRIES-1:0] dbg_ready_to_go;

// Fully drained = nothing waiting to dispatch (table_empty) AND nothing still
// executing on any accelerator (~|acc_busy_i).  table_empty alone is only a
// *dispatch* barrier (entries are deleted on dispatch, not completion), so the
// LOOPEND and NXT completion barriers must also wait for in-flight tasks to
// finish — NXT must not advance the input/output window pointer until the tasks
// using the current window have completed.
//
// A freshly dispatched task is INVISIBLE to both terms while the
// config_manager streams its per-task config (~WORDS_PER_CONFIG cycles): the
// table entry is deleted at dispatch and the target accelerator only raises
// busy after the push + start pulse. Bridge that window — track from the
// dispatch pulse until the dispatched accelerator's busy is observed (covers
// the one-cycle cm_config_finished -> acc-busy gap too), and also hold while
// the config_manager itself is busy.
reg                  dispatch_in_flight_r;
reg [TGT_ACC_SZ-1:0] dispatch_tgt_r;
always @ (posedge clk)
   if (sch_clr) begin
      dispatch_in_flight_r <= 1'b0;
      dispatch_tgt_r       <= {TGT_ACC_SZ{1'b0}};
   end
   else if (start_new_task) begin
      dispatch_in_flight_r <= 1'b1;
      dispatch_tgt_r       <= to_launch_acc_hw_id;
   end
   else if (acc_busy_i[dispatch_tgt_r])
      dispatch_in_flight_r <= 1'b0;

wire all_tasks_drained = table_empty & ~|acc_busy_i
                       & ~cm_busy_i & ~dispatch_in_flight_r;

reg  prog_running_r;
wire prog_running_nxt;
wire keep_fetching;
reg  [PROG_ADDR_BITS-1:0] prog_counter_r;
reg  [PROG_ADDR_BITS-1:0] prog_start_addr_r;   // latched by LOAD_PC write
reg                        prog_paused_r;        // set/cleared by PAUSE/UNPAUSE
wire [PROG_ADDR_BITS-1:0] prog_counter_nxt;
reg  prog_stopping_r;
wire prog_stopping_nxt;
reg  word_ready_r;

wire [PROG_DATA_BITS-1:0] inst_word;
reg  [PROG_DATA_BITS-1:0] held_inst_word_r;
reg                        inst_word_valid_r;
wire                       inst_word_valid_nxt;
wire                       inst_valid;
wire                       inst_valid_for_decode;
wire                       inst_consumed;
wire                       inst_consumed_w2;

// Two-word fetch state (shared by TASK and FILL):
reg  [PROG_DATA_BITS-1:0] task_w1_r;
reg  [PROG_DATA_BITS-1:0] task_w2_r;
reg                        task_w2_pending_r;
wire                       task_w2_arrived;
reg                        pending_is_fill_r;    // 1 = pending word 2 is FILL
// Wide (three-word) TASK: word 3 fetch state. Idle when WIDE_NTGT=0.
reg                        task_w3_pending_r;
reg                        pending_is_wide_r;    // 1 = this TASK is the wide form
wire                       task_w3_arrived;
wire                       inst_consumed_w3;
// Registered fields from FILL word 1 (latched when FILL word 1 is consumed):
reg  [19:0]                fill_block_size_r;
reg  [BUFF_INDX_SZ-1:0]   fill_dst_buf_r;
reg  [TGT_COUNT_SZ-1:0]   fill_ntgt_r;
reg                        fill_colour_r;
// Registered fill constant from FILL word 2 (latched at inst_consumed_w2):
reg  [31:0]                fill_value_r;
// One FILL in flight at a time. block_size/value above are SINGLE registers that
// the next FILL word-1 overwrites, but buff_id travels with the table entry — so a
// launch delayed by a busy fill_unit could dispatch a FILL with a LATER FILL's
// block_size/value. Set when a FILL entry loads, cleared when it launches; gates
// the next FILL's word-1 decode.
reg                        fill_in_flight_r;

reg  inst_is_task;
reg  inst_is_jump;
reg  inst_is_stop;
reg  inst_is_check;
reg  inst_is_fill;
reg  inst_is_nxt;
reg  inst_is_loop;
reg  inst_is_loopend;
reg  inst_unknown;

wire [LOOP_ID_SZ-1:0]   loop_id_w;
wire [LOOP_CNT_SZ-1:0]  loop_max_val_w;
wire                     loopend_active;
reg  [LOOP_CNT_SZ-1:0]  loop_counter_r [0:NUM_LOOPS-1];
reg  [PROG_ADDR_BITS-1:0] loop_restart_r [0:NUM_LOOPS-1];

wire [BUFF_INDX_SZ-1:0]  check_buff_id;
wire                      check_mode;
wire [PROG_ADDR_BITS-1:0] check_skip_addr_w;
wire                      check_result_ready;
wire                      check_success;
wire [PROG_ADDR_BITS-1:0] jump_target;
wire                      do_jump;
wire                      goto_nxt;

// Unpack entry to be launched for sch_buffer_state:
wire [NUM_SLOTS*BUFF_INDX_SZ-1:0] to_launch_slot_buff;
wire [NUM_SLOTS*MODE_SZ-1:0]       to_launch_slot_mode;
wire [NUM_SLOTS*TGT_COUNT_SZ-1:0]  to_launch_slot_ntgt;
wire                               to_launch_colour;

// ------------------------------------------------------------
// Unpack launched entry fields

genvar us;
generate
   // Short slots 0-2:
   for (us = 0; us < 3; us = us + 1) begin : unpack_short
      localparam BASE = us * SLOT_SHORT_SZ;
      assign to_launch_slot_mode[us*MODE_SZ      +: MODE_SZ]      = entry_data_to_be_launched[BASE           +: MODE_SZ];
      assign to_launch_slot_buff[us*BUFF_INDX_SZ +: BUFF_INDX_SZ] = entry_data_to_be_launched[BASE + MODE_SZ +: BUFF_INDX_SZ];
      assign to_launch_slot_ntgt[us*TGT_COUNT_SZ +: TGT_COUNT_SZ] = 'b0; // short slots have no ntgt
   end
   // Long slots 3-5:
   for (us = 0; us < 3; us = us + 1) begin : unpack_long
      localparam BASE = LONG_BASE + us * SLOT_LONG_SZ;
      localparam SI   = 3 + us;
      assign to_launch_slot_mode[SI*MODE_SZ      +: MODE_SZ]      = entry_data_to_be_launched[BASE                          +: MODE_SZ];
      assign to_launch_slot_buff[SI*BUFF_INDX_SZ +: BUFF_INDX_SZ] = entry_data_to_be_launched[BASE + MODE_SZ                +: BUFF_INDX_SZ];
      assign to_launch_slot_ntgt[SI*TGT_COUNT_SZ +: TGT_COUNT_SZ] = entry_data_to_be_launched[BASE + MODE_SZ + BUFF_INDX_SZ +: TGT_COUNT_SZ];
   end
endgenerate

assign to_launch_acc_hw_id = entry_data_to_be_launched[E_ACC_START +: TGT_ACC_SZ];
assign to_launch_colour    = entry_data_to_be_launched[E_COLOUR];

assign target_acc_o  = to_launch_acc_hw_id;
assign buffer_info_o = entry_data_to_be_launched;

// ------------------------------------------------------------
// CHECK instruction decode

assign check_buff_id      = inst_word[3 + BUFF_INDX_SZ : 4];
assign check_mode         = inst_word[3];
assign check_skip_addr_w  = inst_word[PROG_ADDR_BITS+11:12];
assign check_result_ready = buff_full[check_buff_id];
assign check_success      = ~target_status[check_buff_id];

// LOOP field decode:
assign loop_id_w      = inst_word[5:3];
assign loop_max_val_w = inst_word[31:6];
assign loopend_active = inst_is_loopend & (loop_counter_r[loop_id_w] != 'b0);

// Shared jump mux (JUMP / CHECK / LOOPEND):
assign jump_target = inst_is_jump    ? inst_word[PROG_ADDR_BITS+2:3]
                   : inst_is_loopend ? loop_restart_r[loop_id_w]
                   :                   check_skip_addr_w;

assign do_jump  = (inst_is_jump    & inst_consumed)
                | (inst_is_check   & inst_consumed & check_success & ~check_mode)
                | (inst_is_loopend & inst_consumed & loopend_active);

assign goto_nxt = inst_consumed;

assign prog_counter_nxt = do_start    ? prog_start_addr_r
                        : do_continue ? prog_counter_r + 1'b1
                        : do_jump     ? jump_target
                        : goto_nxt    ? prog_counter_r + 1
                        :               prog_counter_r;

assign prog_running_nxt = (do_start | do_continue) ? 1'b1 : prog_running_r;

assign prog_stopping_nxt =
    (prog_running_r & inst_valid_for_decode & inst_is_stop) ? 1'b1 :
    (prog_running_r & inst_valid_for_decode & inst_is_check &
     check_result_ready & check_success & check_mode) ? 1'b1 : 1'b0;

// ------------------------------------------------------------
// Instruction word hold register

assign inst_word_valid_nxt = (do_start | do_continue) ? 1'b0
                           : (word_ready_r      & ~inst_consumed) ? 1'b1
                           : (inst_word_valid_r & ~inst_consumed) ? 1'b1
                           : (~word_ready_r     &  inst_consumed) ? 1'b0
                           : inst_word_valid_r;

always @ (posedge clk)
begin
   if (sch_clr)
   begin
      prog_running_r    <= 1'b0;
      prog_counter_r    <= 'b0;
      prog_paused_r     <= 1'b0;
      prog_stopping_r   <= 1'b0;
      inst_word_valid_r <= 1'b0;
      held_inst_word_r  <= 'b0;
   end
   else
   begin
      prog_running_r    <= prog_running_nxt;
      inst_word_valid_r <= inst_word_valid_nxt;
      prog_stopping_r   <= prog_stopping_nxt;
      if (prog_running_nxt)
         prog_counter_r <= prog_counter_nxt;
      if (word_ready_r & ~inst_consumed)
         held_inst_word_r <= prog_mem_data_i;
      // AXI control register writes
      if (sch_ctrl_wr && sys_addr_i[24:20] == 5'd3) prog_paused_r <= 1'b1;  // PAUSE
      if (sch_ctrl_wr && sys_addr_i[24:20] == 5'd4) prog_paused_r <= 1'b0;  // UNPAUSE
   end
   // LOAD_PC target survives SOFT_RESET (hard reset only) so the host may issue
   // LOAD_PC and SOFT_RESET in either order.
   if (reset)
      prog_start_addr_r <= 'b0;
   else if (sch_ctrl_wr && sys_addr_i[24:20] == 5'd0)
      prog_start_addr_r <= sys_data_i[PROG_ADDR_BITS-1:0];      // LOAD_PC
end

assign inst_word = inst_word_valid_r ? held_inst_word_r : prog_mem_data_i;

// ------------------------------------------------------------
// Two-word fetch state machine (TASK and FILL)
//
// When word 1 of a TASK or FILL is consumed, set task_w2_pending_r and save
// word 1 (and fill-specific fields for FILL). Word 2 is latched silently
// (inst_consumed_w2), the entry is packed, and load_new_entry fires.
// Normal decode is suppressed while pending.

assign task_w2_arrived   = task_w2_pending_r & inst_valid;
assign inst_consumed_w2  = task_w2_arrived & ~test_stall_pipe;
assign task_w3_arrived   = task_w3_pending_r & inst_valid;
assign inst_consumed_w3  = task_w3_arrived & ~test_stall_pipe;

always @ (posedge clk)
begin
   if (sch_clr)
   begin
      task_w2_pending_r <= 1'b0;
      pending_is_fill_r <= 1'b0;
      task_w3_pending_r <= 1'b0;
      pending_is_wide_r <= 1'b0;
      task_w1_r         <= 'b0;
      task_w2_r         <= 'b0;
      fill_block_size_r <= 'b0;
      fill_value_r      <= 'b0;
      fill_dst_buf_r    <= 'b0;
      fill_ntgt_r       <= 'b0;
      fill_colour_r     <= 1'b0;
   end
   else
   begin
      if (inst_valid_for_decode & (inst_is_task | inst_is_fill) & inst_consumed)
      begin
         task_w2_pending_r <= 1'b1;
         pending_is_fill_r <= inst_is_fill;
         // Only a TASK can be wide; FILL stays two-word.
         pending_is_wide_r <= (WIDE_NTGT != 0) & inst_is_task & inst_word[31];
         task_w1_r         <= inst_word;
         if (inst_is_fill) begin
            // Latch FILL word 1 fields. [6:3]=buf_id, [8]=colour, then either
            //   WIDE_NTGT=0: [12:9]  ntgt (4-bit),  [31:13] block_size (19-bit)
            //   WIDE_NTGT=1: [15:9]  ntgt (7-bit),  [31:16] block_size (16-bit)
            // FILL word 1 has no spare bit for a per-instruction selector, so
            // this is a BUILD-WIDE choice -- see the WIDE_NTGT parameter note.
            // 16 bits still covers 65,536 words; the largest fill in any current
            // schedule is 632.
            fill_dst_buf_r    <= inst_word[3 +: BUFF_INDX_SZ];
            fill_ntgt_r       <= inst_word[9 +: TGT_COUNT_SZ];
            fill_colour_r     <= inst_word[8];
            fill_block_size_r <= (WIDE_NTGT != 0) ? {3'b000, inst_word[31:16]}
                                                  : inst_word[31:13];
         end
      end
      if (inst_consumed_w2)
      begin
         task_w2_pending_r <= 1'b0;
         pending_is_fill_r <= 1'b0;
         task_w2_r         <= inst_word;
         if (pending_is_fill_r)
            fill_value_r <= inst_word;  // full 32-bit constant, no sentinel check
         // Wide TASK: word 2 does NOT complete the instruction; go fetch word 3.
         // task_w1_r/task_w2_r hold; load_new_entry is deferred to word 3.
         if (pending_is_wide_r)
            task_w3_pending_r <= 1'b1;
      end
      if (inst_consumed_w3)
      begin
         task_w3_pending_r <= 1'b0;
         pending_is_wide_r <= 1'b0;
      end
      // Flush any in-flight multi-word instruction on START or CONTINUE
      if (do_start | do_continue)
      begin
         task_w2_pending_r <= 1'b0;
         pending_is_fill_r <= 1'b0;
         task_w3_pending_r <= 1'b0;
         pending_is_wide_r <= 1'b0;
      end
   end
end

// One-FILL-in-flight gate (see fill_in_flight_r decl). Set when a FILL entry is
// loaded (its word 2 consumed), cleared when a FILL launches. The decode gate in
// inst_consumed prevents a new FILL's word 1 while one is in flight, so load and
// launch are mutually exclusive per cycle (no deadlock / no same-cycle race).
wire fill_launched = start_new_task & (to_launch_acc_hw_id == FILL_ACC_ID);
always @ (posedge clk)
begin
   if (sch_clr)
      fill_in_flight_r <= 1'b0;
   else if (load_new_entry & pending_is_fill_r)
      fill_in_flight_r <= 1'b1;
   else if (fill_launched)
      fill_in_flight_r <= 1'b0;
end

// Suppress normal decode while fetching word 2 (or word 3 of a wide TASK):
assign inst_valid_for_decode = inst_valid & ~task_w2_pending_r & ~task_w3_pending_r;

// ------------------------------------------------------------
// Loop counters

integer li;
always @(posedge clk)
begin
   if (sch_clr)
   begin
      for (li = 0; li < NUM_LOOPS; li = li + 1)
      begin
         loop_counter_r[li] <= 'b0;
         loop_restart_r[li] <= 'b0;
      end
   end
   else
   begin
      if (inst_is_loop & inst_consumed)
      begin
         loop_counter_r[loop_id_w] <= loop_max_val_w;
         loop_restart_r[loop_id_w] <= prog_counter_r + 1'b1;
      end
      if (inst_is_loopend & inst_consumed & loopend_active)
         loop_counter_r[loop_id_w] <= loop_counter_r[loop_id_w] - 1'b1;
   end
end

// ------------------------------------------------------------
// Fetch control

assign keep_fetching  = prog_running_r & ~prog_stopping_r & ~inst_is_stop
                      & ~prog_paused_r;
assign prog_mem_addr_o = prog_counter_r;
assign prog_mem_req_o  = keep_fetching & (~inst_valid | ~inst_word_valid_nxt);

always @ (posedge clk)
begin
   if (sch_clr)
      word_ready_r <= 1'b0;
   else if (prog_mem_req_o & ~prog_mem_wait_i)
      word_ready_r <= 1'b1;
   else
      word_ready_r <= 1'b0;
end

assign inst_valid = word_ready_r | inst_word_valid_r;

// ------------------------------------------------------------
// Instruction decode (on inst_word, gated by inst_valid_for_decode)

// CANDIDATE FIX v2 (under test) — qualify the decode AT THE SOURCE.
// These flags are a raw combinational decode of inst_word, which is ALSO the
// second word of a two-word instruction (a FILL's 32-bit constant) on the cycle
// that word is consumed. inst_valid_for_decode already knows this
// (& ~task_w2_pending_r), but several CONSUMERS bypassed it and keyed off
// inst_consumed instead -- and inst_consumed deliberately ORs in
// inst_consumed_w2 outside that gate so the PC can advance past word 2.
// Measured consequences on the unfixed RTL:
//   constant 0x11111111 (tail 001 JUMP) -> do_jump, pc 1 -> 546 = word[12:3]
//   constant 0x000002BE (tail 110 LOOP) -> loop_counter_r[7]=10, restart[7]=2
// Zeroing the flags unless the word is a real instruction fixes every consumer
// at once, including any added later, rather than patching each use site.
always @ (*)
begin
   inst_is_task    = 1'b0;
   inst_is_jump    = 1'b0;
   inst_is_stop    = 1'b0;
   inst_is_check   = 1'b0;
   inst_is_nxt     = 1'b0;
   inst_is_fill    = 1'b0;
   inst_is_loop    = 1'b0;
   inst_is_loopend = 1'b0;
   inst_unknown    = 1'b0;
   if (inst_valid_for_decode)
   case (inst_word[2:0])
      INST_TASK:    inst_is_task    = 1'b1;
      INST_JUMP:    inst_is_jump    = 1'b1;
      INST_STOP:    inst_is_stop    = 1'b1;
      INST_CHECK:   inst_is_check   = 1'b1;
      INST_NXT:     inst_is_nxt     = 1'b1;
      INST_FILL:    inst_is_fill    = 1'b1;
      INST_LOOP:    inst_is_loop    = 1'b1;
      INST_LOOPEND: inst_is_loopend = 1'b1;
      default:      inst_unknown    = 1'b1;
   endcase
end

// inst_consumed: each instruction type's readiness condition.
// TASK/FILL word 1 fires when a scheduler table slot is free; word 2 consumed
// by inst_consumed_w2 which also fires load_new_entry.
// NXT/LOOPEND barrier: the completion drain by default; RELAX_LOOP_BARRIER
// lifts it (see the parameter comment — host I/O then sequences off buffer
// state, and the in-order table + claim/consume protocol carries the data
// ordering across the loop boundary).
wire loop_barrier_ok = RELAX_LOOP_BARRIER ? 1'b1 : all_tasks_drained;

assign inst_consumed = inst_valid_for_decode & ~test_stall_pipe & (
                          ((inst_is_task | (inst_is_fill & ~fill_in_flight_r)) & table_slot_free)
                        |  inst_is_jump
                        | (inst_is_check  & check_result_ready)
                        |  inst_is_loop
                        | (inst_is_loopend & loop_barrier_ok)
                        | (inst_is_nxt    & loop_barrier_ok)
                       )
                     | inst_consumed_w2
                     | inst_consumed_w3;

// load_new_entry fires on the LAST word of the instruction: word 2 for the
// narrow form, word 3 for the wide one.
assign load_new_entry = (inst_consumed_w2 & ~pending_is_wide_r)
                      |  inst_consumed_w3;

assign nxt_input_pulse_o  = inst_valid_for_decode & inst_is_nxt & inst_word[4] & loop_barrier_ok & ~test_stall_pipe;
assign nxt_output_pulse_o = inst_valid_for_decode & inst_is_nxt & inst_word[5] & loop_barrier_ok & ~test_stall_pipe;

assign fill_value_o      = fill_value_r;
assign fill_block_size_o = fill_block_size_r;

// ------------------------------------------------------------
// Entry data packing: TASK or FILL depending on pending_is_fill_r.
//
// TASK word 1 fields (latched into task_w1_r):
//   [2:0]   opcode (discarded)
//   [4:3]   acc_id (2-bit; extended to TGT_ACC_SZ with zero MSBs)
//   [11:5]  cfg_id (7-bit; up to 128 configs)
//   [12]    colour
//   [14:13] slot 0 mode,  [18:15] slot 0 id
//   [20:19] slot 1 mode,  [24:21] slot 1 id
//   [26:25] slot 2 mode,  [30:27] slot 2 id
//   [31]    reserved
//
// TASK word 2 fields (latched into task_w2_r):
//   [1:0]   sentinel 2'b00
//   [3:2]   slot 3 mode,  [7:4]   slot 3 id,  [11:8]  slot 3 ntgt (4-bit)
//   [13:12] slot 4 mode,  [17:14] slot 4 id,  [21:18] slot 4 ntgt (4-bit)
//   [23:22] slot 5 mode,  [27:24] slot 5 id,  [31:28] slot 5 ntgt (4-bit)
//   (ntgt is 4-bit, up to 15; word-2 is now packed to bit 31)
//
// FILL entry: slot 3 = TARGET(fill_dst_buf_r, fill_ntgt_r), others UNUSED.
//   acc_id = FILL_ACC_ID, cfg_id = 0, colour = fill_colour_r.

always @*
begin
   d = 'b0;
   if (pending_is_fill_r) begin
      // Slot 3 = TARGET: fill_unit writes the target buffer
      d[LONG_BASE + 0*SLOT_LONG_SZ +: MODE_SZ]                          = `MODE_TGT;
      d[LONG_BASE + 0*SLOT_LONG_SZ + MODE_SZ +: BUFF_INDX_SZ]           = fill_dst_buf_r;
      d[LONG_BASE + 0*SLOT_LONG_SZ + MODE_SZ + BUFF_INDX_SZ +: TGT_COUNT_SZ] = fill_ntgt_r;
      d[E_COLOUR]                   = fill_colour_r;
      d[E_ACC_START +: TGT_ACC_SZ] = FILL_ACC_ID;
      // cfg_id = 0; slots 0-2, 4-5 = UNUSED (already 0 from d='b0)
   end else begin
      // Slot 0 (short: mode+id):
      d[0*SLOT_SHORT_SZ +: MODE_SZ]                        = task_w1_r[14:13];
      d[0*SLOT_SHORT_SZ + MODE_SZ +: BUFF_INDX_SZ]         = task_w1_r[18:15];
      // Slot 1:
      d[1*SLOT_SHORT_SZ +: MODE_SZ]                        = task_w1_r[20:19];
      d[1*SLOT_SHORT_SZ + MODE_SZ +: BUFF_INDX_SZ]         = task_w1_r[24:21];
      // Slot 2:
      d[2*SLOT_SHORT_SZ +: MODE_SZ]                        = task_w1_r[26:25];
      d[2*SLOT_SHORT_SZ + MODE_SZ +: BUFF_INDX_SZ]         = task_w1_r[30:27];
      // ---- long slots 3..5 ----
      if (pending_is_wide_r) begin
         // WIDE form. The instruction packs long slots at exactly the same
         // SLOT_LONG_SZ stride, and in the same {mode,id,ntgt} lsb-first order,
         // that the entry uses internally -- so decode is a straight slice copy
         // and rescales automatically with TGT_COUNT_SZ / BUFF_INDX_SZ.
         //   W2: [1:0] sentinel, slot3 at bit 2, slot4 at bit 2+SLOT_LONG_SZ
         //   W3: slot5 at bit 0
         // task_w2_r is settled here (latched a cycle earlier); inst_word is the
         // live W3, for the same reason the narrow path reads it live.
         d[LONG_BASE + 0*SLOT_LONG_SZ +: SLOT_LONG_SZ] =
                                     task_w2_r[2                 +: SLOT_LONG_SZ];
         d[LONG_BASE + 1*SLOT_LONG_SZ +: SLOT_LONG_SZ] =
                                     task_w2_r[2 + SLOT_LONG_SZ  +: SLOT_LONG_SZ];
         d[LONG_BASE + 2*SLOT_LONG_SZ +: SLOT_LONG_SZ] =
                                     inst_word[0                 +: SLOT_LONG_SZ];
      end else begin
      // Slot 3 (long: mode+id+ntgt):
      // Use inst_word (live W2) not task_w2_r: load_new_entry fires in the same cycle
      // as inst_consumed_w2, before task_w2_r latches the new value at posedge.
      d[LONG_BASE + 0*SLOT_LONG_SZ +: MODE_SZ]             = inst_word[3:2];
      d[LONG_BASE + 0*SLOT_LONG_SZ + MODE_SZ +: BUFF_INDX_SZ]         = inst_word[7:4];
      d[LONG_BASE + 0*SLOT_LONG_SZ + MODE_SZ + BUFF_INDX_SZ +: TGT_COUNT_SZ] = inst_word[11:8];
      // Slot 4:
      d[LONG_BASE + 1*SLOT_LONG_SZ +: MODE_SZ]             = inst_word[13:12];
      d[LONG_BASE + 1*SLOT_LONG_SZ + MODE_SZ +: BUFF_INDX_SZ]         = inst_word[17:14];
      d[LONG_BASE + 1*SLOT_LONG_SZ + MODE_SZ + BUFF_INDX_SZ +: TGT_COUNT_SZ] = inst_word[21:18];
      // Slot 5:
      d[LONG_BASE + 2*SLOT_LONG_SZ +: MODE_SZ]             = inst_word[23:22];
      d[LONG_BASE + 2*SLOT_LONG_SZ + MODE_SZ +: BUFF_INDX_SZ]         = inst_word[27:24];
      d[LONG_BASE + 2*SLOT_LONG_SZ + MODE_SZ + BUFF_INDX_SZ +: TGT_COUNT_SZ] = inst_word[31:28];
      end
      // Header (acc_id zero-extended from 2-bit TASK field to TGT_ACC_SZ bits):
      d[E_COLOUR]                   = task_w1_r[12];
      d[E_ACC_START +: TGT_ACC_SZ] = {{(TGT_ACC_SZ-2){1'b0}}, task_w1_r[4:3]};
      // cfg_id: word 1 [11:5] is only 7 bits (128 configs), but Monarch needs 172
      // at nblocks=20 and 329 at nblocks=40. The WIDE form therefore carries it
      // WHOLLY in word 3, above slot 5 -- contiguous, like every other field, so
      // it reads straight out of a hex dump. A wide TASK ALWAYS takes cfg_id from
      // word 3 (word 1's cfg field is zero there): one rule, no redundant copy.
      // inst_word is the live word 3 here, as for the slots above.
      if (pending_is_wide_r)
         d[E_CFG_START +: CFG_ID_SZ] = inst_word[SLOT_LONG_SZ +: CFG_ID_SZ];
      else
         d[E_CFG_START +: CFG_ID_SZ] = task_w1_r[11:5];   // zero-extends if wider
   end
end

assign new_entry_data = d;

// Disjoint hint: wide word 3, the bit immediately above cfg_id.
//
// It must be read at the ISA's WIDE widths, NOT at this instance's. The wide
// TASK ENCODING is fixed by the toolchain (isa.DISJOINT_BIT = SLOT_LONG_SZ_WIDE
// + CFG_ID_SZ_WIDE = 13 + 9 = 22) whatever CFG_ID_SZ this instance happens to
// carry, so deriving the position from CFG_ID_SZ tracks the wrong thing. This
// previously read "derived rather than hardcoded so it tracks the field widths",
// which is backwards, and it bit: a variant whose config count fits 7 bits was
// given CFG_ID_SZ=7, the hint was read at bit 20, EVERY hint was lost, and the
// SP/NP overlap collapsed -- 1.85% of frame time at nblocks=8, with the run
// still bit-exact so nothing failed (2026-09-01).
//
// Neutral where things already worked: at CFG_ID_SZ=9 / TGT_COUNT_SZ=7 the old
// expression is also 22. FILL and narrow TASK never carry the hint, and
// DISJOINT_HINT defaults to 0, so a narrow-only build is unaffected either way.
localparam NTGT_SZ_WIDE_ISA      = 7;    // == isa.NTGT_SZ_WIDE
localparam CFG_ID_SZ_WIDE_ISA    = 9;    // == isa.CFG_ID_SZ_WIDE
localparam SLOT_LONG_SZ_WIDE_ISA = MODE_SZ + BUFF_INDX_SZ + NTGT_SZ_WIDE_ISA;
localparam E_HINT_BIT = SLOT_LONG_SZ_WIDE_ISA + CFG_ID_SZ_WIDE_ISA;   // 22
assign new_entry_hint = (DISJOINT_HINT != 0) & ~pending_is_fill_r
                      & pending_is_wide_r & inst_word[E_HINT_BIT];

// ------------------------------------------------------------
// Scheduler table

sch_table #(
   .SCH_ENTRY_SZ(ENTRY_DATA_SZ),
   .NUM_HW_ACCELERATORS(NUM_HW_ACCELERATORS),
   .TGT_ACC_SZ(TGT_ACC_SZ),
   .NUM_BUFFERS(NUM_BUFFERS),
   .COL_BUFF_ID_SZ(COL_BUFF_ID_SZ),
   .NUM_SCH_ENTRIES(NUM_SCH_ENTRIES),
   .ACC_ID_BTM(E_ACC_START),
   .MODE_SZ(MODE_SZ),
   .BUFF_INDX_SZ(BUFF_INDX_SZ),
   .TGT_COUNT_SZ(TGT_COUNT_SZ),
   .DISJOINT_HINT(DISJOINT_HINT)
) sch_table0 (
   .clk(clk),
   .reset(sch_clr),
   .load_new_entry_i(load_new_entry),
   .delete_entry_i(1'b0),
   .entry_data_i(new_entry_data),
   .entry_hint_i(new_entry_hint),
   .dispatch_to_acc_o(start_new_task),
   .entry_data_o(entry_data_to_be_launched),
   .table_slot_free_o(table_slot_free),
   .table_empty_o(table_empty),
   .dbg_entry_valid_o(dbg_entry_valid),
   .dbg_ready_to_go_o(dbg_ready_to_go),
   .acc_busy_i(~acc_available),
   .buffers_full_i(buff_full),
   .buffers_free_i(buff_free),
   .buffers_colour_i(buff_colour),
   .cm_busy_i(cm_busy_i)
);

assign start_new_block_o = start_new_task;

// ------------------------------------------------------------
// Buffer and accelerator state tracker

sch_buffer_state #(
   .NUM_BUFFERS(NUM_BUFFERS),
   .BUFF_INDX_SZ(BUFF_INDX_SZ),
   .NUM_HW_ACCELERATORS(NUM_HW_ACCELERATORS),
   .TGT_ACC_SZ(TGT_ACC_SZ),
   .TGT_COUNT_SZ(TGT_COUNT_SZ),
   .NUM_SLOTS(NUM_SLOTS),
   .MODE_SZ(MODE_SZ),
   .OVERLAP_ACC_MASK(OVERLAP_ACC_MASK),
   .MULTI_WRITER(MULTI_WRITER)
) sch_buff_state0 (
   .clk(clk),
   .reset(sch_clr),
   .acc_busy_i(acc_busy_i),
   .acc_finished_i(acc_finished_i),
   .acc_result_i(acc_result_i),
   .acc_ready_next_i(acc_ready_next_i),
   .mark_buff_as_full_i(do_mark_full),
   .full_buff_id_i(mark_full_id),
   .full_buff_usage_i(mark_full_cnt),
   .start_new_task_i(start_new_task),
   .tgt_acc_id_i(to_launch_acc_hw_id),
   .slot_buff_i(to_launch_slot_buff),
   .slot_mode_i(to_launch_slot_mode),
   .slot_ntgt_i(to_launch_slot_ntgt),
   .tgt_colour_i(to_launch_colour),
   .acc_available_o(acc_available),
   .buffers_full_o(buff_full),
   .buffers_free_o(buff_free),
   .buffers_colour_o(buff_colour),
   .target_status_o(target_status)
);

// ------------------------------------------------------------
// AXI response

assign sys_ack_o = sch_ctrl_wr | sch_stat_rd | sch_prog_wr;

wire [5:0] stat_reg = sys_addr_i[25:20];
assign sys_data_o =
    ~sch_stat_rd                  ? 32'b0
    : (stat_reg == 6'd0)          ? {{(32-PROG_ADDR_BITS){1'b0}}, prog_counter_r}
    : (stat_reg == 6'd1)          ? {prog_paused_r, prog_stopping_r,
                                     {(30-NUM_HW_ACCELERATORS){1'b0}},
                                     acc_busy_i}
    : (stat_reg == 6'd2)          ? {{(32-NUM_BUFFERS){1'b0}}, buff_full}
    : (stat_reg == 6'd3)          ? {{(32-NUM_BUFFERS){1'b0}}, ~buff_free & ~buff_full}
    : (stat_reg == 6'd4)          ? {{(32-NUM_BUFFERS){1'b0}}, buff_colour}
    :                               32'b0;

// ------------------------------------------------------------
// Program memory write side (host loads instructions via AXI)

assign prog_mem_wr_o      = sch_prog_wr;
assign prog_mem_wr_addr_o = (sys_addr_i & ~SCH_PROG_MEM_MASK) >> 2;
assign prog_mem_wr_data_o = sys_data_i;

// ------------------------------------------------------------
// Debug bundle (KV260 bring-up).  Read-only taps on existing state — no
// functional effect.  Together with prog_counter_r (status reg 0) and the
// buffer state (status regs 2/3/4) these say exactly WHY the front-end
// stopped consuming instructions:
//   table_empty=1                    -> nothing queued: the FETCH side stalled
//                                       (check fill_in_flight / slot_free / stopping)
//   table_empty=0, ready_to_go[0]=0  -> head entry blocked: compare acc_avail
//                                       against the entry's acc, and the buffer
//                                       free/full regs against its slots
//   slot_free=0                      -> table full: dispatch is the blockage
// NUM_HW_ACCELERATORS is assumed <= 8 for the acc_avail field; the table
// fields are pinned to 4 bits so the concat is always exactly 32 wide
// regardless of NUM_SCH_ENTRIES.
wire [3:0] dbg_ev4 = dbg_entry_valid;
wire [3:0] dbg_rg4 = dbg_ready_to_go;

assign dbg_frontend_o = {
    {(8-NUM_HW_ACCELERATORS){1'b0}}, acc_available,   // [31:24]
    all_tasks_drained,                                // [23]
    start_new_task,                                   // [22]
    cm_busy_i,                                        // [21]
    dispatch_in_flight_r,                             // [20]
    prog_paused_r,                                    // [19]
    prog_stopping_r,                                  // [18]
    prog_running_r,                                   // [17]
    inst_consumed,                                    // [16]
    inst_valid,                                       // [15]
    word_ready_r,                                     // [14]
    inst_word_valid_r,                                // [13]
    pending_is_fill_r,                                // [12]
    task_w2_pending_r,                                // [11]
    fill_in_flight_r,                                 // [10]
    table_empty,                                      // [9]
    table_slot_free,                                  // [8]
    dbg_rg4,                                          // [7:4]
    dbg_ev4                                           // [3:0]
};

assign dbg_inst_word_o = inst_word;

endmodule
