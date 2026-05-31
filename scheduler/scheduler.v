// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
`include "../shared/constants.v"

`timescale 10ps/1ps

// Slot modes (shared with sch_entry / sch_buffer_state):
`define MODE_UNUSED 2'b00
`define MODE_SRC    2'b01
`define MODE_RW     2'b10
`define MODE_TGT    2'b11

module scheduler
   #(parameter TGT_COUNT_SZ        = 3,
     parameter CFG_ID_SZ           = 5,
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
     // ACC ID width in scheduler table (3 bits to accommodate fill_unit at id 4):
     parameter TGT_ACC_SZ          = 3,
     // Derived entry layout sizes:
     parameter SLOT_SHORT_SZ       = MODE_SZ + BUFF_INDX_SZ,          // 6
     parameter SLOT_LONG_SZ        = MODE_SZ + BUFF_INDX_SZ + TGT_COUNT_SZ, // 9
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
     input  wire                          cm_busy_i
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
// MARK_BUFF_FULL (reg 5): data[BUFF_INDX_SZ-1:0]=buf_id, data[BUFF_INDX_SZ+:TGT_COUNT_SZ]=usage
wire do_mark_full       = sch_ctrl_wr & (sys_addr_i[24:20] == 5'd5);
wire [BUFF_INDX_SZ-1:0] mark_full_id  = sys_data_i[BUFF_INDX_SZ-1:0];
wire [TGT_COUNT_SZ-1:0] mark_full_cnt = sys_data_i[BUFF_INDX_SZ +: TGT_COUNT_SZ];

// ------------------------------------------------------------
// Entry data field offsets (lsb first, matching sch_entry):
//   Slots 0-2: [mode(2), id(4)] × 3  =  3 × 6 = 18 bits
//   Slots 3-5: [mode(2), id(4), ntgt(3)] × 3 = 3 × 9 = 27 bits
//   colour(1), acc_id(2), cfg_id(5)
localparam LONG_BASE   = 3 * SLOT_SHORT_SZ;           // 18
localparam E_COLOUR    = LONG_BASE + 3 * SLOT_LONG_SZ; // 45
localparam E_ACC_START = E_COLOUR + 1;                 // 46
localparam E_CFG_START = E_ACC_START + TGT_ACC_SZ;    // 49 (TGT_ACC_SZ=3)

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
wire [ENTRY_DATA_SZ-1:0]        entry_data_to_be_launched;

wire [COL_BUFF_ID_SZ-1:0]       buff_full;
wire [COL_BUFF_ID_SZ-1:0]       buff_free;
wire [COL_BUFF_ID_SZ-1:0]       buff_colour;
wire [NUM_BUFFERS-1:0]          target_status;
wire [NUM_HW_ACCELERATORS-1:0]  acc_available;

wire load_new_entry;
wire table_slot_free;
wire table_empty;

// Fully drained = nothing waiting to dispatch (table_empty) AND nothing still
// executing on any accelerator (~|acc_busy_i).  table_empty alone is only a
// *dispatch* barrier (entries are deleted on dispatch, not completion), so the
// LOOPEND and NXT completion barriers must also wait for in-flight tasks to
// finish — NXT must not advance the input/output window pointer until the tasks
// using the current window have completed.
wire all_tasks_drained = table_empty & ~|acc_busy_i;

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
// Registered fields from FILL word 1 (latched when FILL word 1 is consumed):
reg  [19:0]                fill_block_size_r;
reg  [BUFF_INDX_SZ-1:0]   fill_dst_buf_r;
reg  [TGT_COUNT_SZ-1:0]   fill_ntgt_r;
reg                        fill_colour_r;
// Registered fill constant from FILL word 2 (latched at inst_consumed_w2):
reg  [31:0]                fill_value_r;

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
wire [TGT_ACC_SZ-1:0]              to_launch_acc_hw_id;
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
   if (reset)
   begin
      prog_running_r    <= 1'b0;
      prog_counter_r    <= 'b0;
      prog_start_addr_r <= 'b0;
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
      if (sch_ctrl_wr && sys_addr_i[24:20] == 5'd0)
         prog_start_addr_r <= sys_data_i[PROG_ADDR_BITS-1:0];   // LOAD_PC
      if (sch_ctrl_wr && sys_addr_i[24:20] == 5'd3) prog_paused_r <= 1'b1;  // PAUSE
      if (sch_ctrl_wr && sys_addr_i[24:20] == 5'd4) prog_paused_r <= 1'b0;  // UNPAUSE
   end
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

always @ (posedge clk)
begin
   if (reset)
   begin
      task_w2_pending_r <= 1'b0;
      pending_is_fill_r <= 1'b0;
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
         task_w1_r         <= inst_word;
         if (inst_is_fill) begin
            // Latch FILL word 1 fields: [6:3]=buf_id (4 bits), [8]=colour,
            // [11:9]=#targets, [31:12]=block_size (20 bits)
            fill_dst_buf_r    <= inst_word[3 +: BUFF_INDX_SZ];
            fill_ntgt_r       <= inst_word[11:9];
            fill_colour_r     <= inst_word[8];
            fill_block_size_r <= inst_word[31:12];
         end
      end
      if (inst_consumed_w2)
      begin
         task_w2_pending_r <= 1'b0;
         pending_is_fill_r <= 1'b0;
         task_w2_r         <= inst_word;
         if (pending_is_fill_r)
            fill_value_r <= inst_word;  // full 32-bit constant, no sentinel check
      end
      // Flush in-flight two-word instruction on START or CONTINUE
      if (do_start | do_continue)
      begin
         task_w2_pending_r <= 1'b0;
         pending_is_fill_r <= 1'b0;
      end
   end
end

// Suppress normal decode while fetching word 2:
assign inst_valid_for_decode = inst_valid & ~task_w2_pending_r;

// ------------------------------------------------------------
// Loop counters

integer li;
always @(posedge clk)
begin
   if (reset)
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
   if (reset)
      word_ready_r <= 1'b0;
   else if (prog_mem_req_o & ~prog_mem_wait_i)
      word_ready_r <= 1'b1;
   else
      word_ready_r <= 1'b0;
end

assign inst_valid = word_ready_r | inst_word_valid_r;

// ------------------------------------------------------------
// Instruction decode (on inst_word, gated by inst_valid_for_decode)

always @ (inst_word)
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
assign inst_consumed = inst_valid_for_decode & ~test_stall_pipe & (
                          ((inst_is_task | inst_is_fill) & table_slot_free)
                        |  inst_is_jump
                        | (inst_is_check  & check_result_ready)
                        |  inst_is_loop
                        | (inst_is_loopend & all_tasks_drained)
                        | (inst_is_nxt    & all_tasks_drained)
                       )
                     | inst_consumed_w2;

// load_new_entry fires when word 2 of a two-word instruction is latched:
assign load_new_entry = inst_consumed_w2;

assign nxt_input_pulse_o  = inst_valid_for_decode & inst_is_nxt & inst_word[4] & all_tasks_drained & ~test_stall_pipe;
assign nxt_output_pulse_o = inst_valid_for_decode & inst_is_nxt & inst_word[5] & all_tasks_drained & ~test_stall_pipe;

assign fill_value_o      = fill_value_r;
assign fill_block_size_o = fill_block_size_r;

// ------------------------------------------------------------
// Entry data packing: TASK or FILL depending on pending_is_fill_r.
//
// TASK word 1 fields (latched into task_w1_r):
//   [2:0]   opcode (discarded)
//   [4:3]   acc_id (2-bit; extended to TGT_ACC_SZ with zero MSBs)
//   [9:5]   cfg_id
//   [10]    colour
//   [12:11] slot 0 mode,  [16:13] slot 0 id
//   [18:17] slot 1 mode,  [22:19] slot 1 id
//   [24:23] slot 2 mode,  [28:25] slot 2 id
//   [31:29] reserved
//
// TASK word 2 fields (latched into task_w2_r):
//   [1:0]   sentinel 2'b00
//   [3:2]   slot 3 mode,  [7:4]   slot 3 id,  [10:8]  slot 3 ntgt
//   [12:11] slot 4 mode,  [16:13] slot 4 id,  [19:17] slot 4 ntgt
//   [21:20] slot 5 mode,  [25:22] slot 5 id,  [28:26] slot 5 ntgt
//   [31:29] reserved
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
      d[0*SLOT_SHORT_SZ +: MODE_SZ]                        = task_w1_r[12:11];
      d[0*SLOT_SHORT_SZ + MODE_SZ +: BUFF_INDX_SZ]         = task_w1_r[16:13];
      // Slot 1:
      d[1*SLOT_SHORT_SZ +: MODE_SZ]                        = task_w1_r[18:17];
      d[1*SLOT_SHORT_SZ + MODE_SZ +: BUFF_INDX_SZ]         = task_w1_r[22:19];
      // Slot 2:
      d[2*SLOT_SHORT_SZ +: MODE_SZ]                        = task_w1_r[24:23];
      d[2*SLOT_SHORT_SZ + MODE_SZ +: BUFF_INDX_SZ]         = task_w1_r[28:25];
      // Slot 3 (long: mode+id+ntgt):
      // Use inst_word (live W2) not task_w2_r: load_new_entry fires in the same cycle
      // as inst_consumed_w2, before task_w2_r latches the new value at posedge.
      d[LONG_BASE + 0*SLOT_LONG_SZ +: MODE_SZ]             = inst_word[3:2];
      d[LONG_BASE + 0*SLOT_LONG_SZ + MODE_SZ +: BUFF_INDX_SZ]         = inst_word[7:4];
      d[LONG_BASE + 0*SLOT_LONG_SZ + MODE_SZ + BUFF_INDX_SZ +: TGT_COUNT_SZ] = inst_word[10:8];
      // Slot 4:
      d[LONG_BASE + 1*SLOT_LONG_SZ +: MODE_SZ]             = inst_word[12:11];
      d[LONG_BASE + 1*SLOT_LONG_SZ + MODE_SZ +: BUFF_INDX_SZ]         = inst_word[16:13];
      d[LONG_BASE + 1*SLOT_LONG_SZ + MODE_SZ + BUFF_INDX_SZ +: TGT_COUNT_SZ] = inst_word[19:17];
      // Slot 5:
      d[LONG_BASE + 2*SLOT_LONG_SZ +: MODE_SZ]             = inst_word[21:20];
      d[LONG_BASE + 2*SLOT_LONG_SZ + MODE_SZ +: BUFF_INDX_SZ]         = inst_word[25:22];
      d[LONG_BASE + 2*SLOT_LONG_SZ + MODE_SZ + BUFF_INDX_SZ +: TGT_COUNT_SZ] = inst_word[28:26];
      // Header (acc_id zero-extended from 2-bit TASK field to TGT_ACC_SZ bits):
      d[E_COLOUR]                   = task_w1_r[10];
      d[E_ACC_START +: TGT_ACC_SZ] = {{(TGT_ACC_SZ-2){1'b0}}, task_w1_r[4:3]};
      d[E_CFG_START +: CFG_ID_SZ]  = task_w1_r[9:5];
   end
end

assign new_entry_data = d;

// ------------------------------------------------------------
// Scheduler table

sch_table #(
   .SCH_ENTRY_SZ(ENTRY_DATA_SZ),
   .NUM_HW_ACCELERATORS(NUM_HW_ACCELERATORS),
   .TGT_ACC_SZ(TGT_ACC_SZ),
   .NUM_BUFFERS(NUM_BUFFERS),
   .COL_BUFF_ID_SZ(COL_BUFF_ID_SZ),
   .NUM_SCH_ENTRIES(NUM_SCH_ENTRIES),
   .ACC_ID_BTM(E_ACC_START)
) sch_table0 (
   .clk(clk),
   .reset(reset),
   .load_new_entry_i(load_new_entry),
   .delete_entry_i(1'b0),
   .entry_data_i(new_entry_data),
   .dispatch_to_acc_o(start_new_task),
   .entry_data_o(entry_data_to_be_launched),
   .table_slot_free_o(table_slot_free),
   .table_empty_o(table_empty),
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
   .MODE_SZ(MODE_SZ)
) sch_buff_state0 (
   .clk(clk),
   .reset(reset),
   .acc_busy_i(acc_busy_i),
   .acc_finished_i(acc_finished_i),
   .acc_result_i(acc_result_i),
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

endmodule
