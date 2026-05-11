// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
`include "../shared/constants.v"

`define OUTPUTS 400

`timescale 10ps/1ps

module scheduler  #(parameter SCH_ENTRY_SZ        = 32,
                    parameter TGT_ACC_SZ          = 2,
                    parameter TGT_COUNT_SZ        = 3,
		    parameter CFG_ID_SZ           = 5,
                    parameter NUM_BUFFERS         = 16,
                    parameter COL_BUFF_ID_SZ      = 16,
                    parameter NUM_SCH_ENTRIES     = 4,
		    parameter NUM_HW_ACCELERATORS = 2,
		    parameter PROG_ADDR_BITS      = 10,
		    parameter PROG_DATA_BITS      = 32,
		    parameter BUFF_INDX_SZ        = $clog2(NUM_BUFFERS)
            )
             (input  wire                       clk,
              input  wire                       reset,
	      input  wire                       test_stall_pipe,

	      // Interface to AXI bus:
	      input  wire                       sys_req_i,
	      output wire                       sys_ack_o,
	      input  wire   [31:0]              sys_data_i, 
	      output wire   [31:0]              sys_data_o,

	      input wire                       start_program_i,
	      input wire [PROG_ADDR_BITS-1:0]  program_addr_i,
	      
	      // Interface to program memory:
	      output wire     [`ADDR_SIZE-1:0]  prog_mem_addr_o,
              input  wire [PROG_DATA_BITS-1:0]  prog_mem_data_i,
              output wire                       prog_mem_req_o,
              input  wire                       prog_mem_wait_i,
	      
	      // Interface to Accelerators:
	      input wire [NUM_HW_ACCELERATORS-1:0] acc_busy_i,
	      input wire [NUM_HW_ACCELERATORS-1:0] acc_finished_i,
	      input wire [NUM_HW_ACCELERATORS-1:0] acc_result_i,
	      output wire                          start_new_block_o,
	      output wire [$clog2(NUM_HW_ACCELERATORS)-1:0] target_acc_o,
	      output wire [SCH_ENTRY_SZ-1:0]              buffer_info_o,
	      output wire                          nxt_input_pulse_o,
	      output wire                          nxt_output_pulse_o,

	      // External buffer pre-fill interface (replaces internal test hack):
	      input wire                            mark_buff_as_full_i,
	      input wire [BUFF_INDX_SZ-1:0]        full_buff_id_i,
	      input wire [TGT_COUNT_SZ-1:0]        full_buff_usage_i
             );

//parameter SCH_ENTRY_SZ    = 32,
//                    parameter TGT_ACC_SZ      = 2,
//                    parameter NUM_BUFFERS     = 16,
//                    parameter COL_BUFF_ID_SZ  = 16,
//                    parameter NUM_SCH_ENTRIES = 4
//            )

localparam COLOUR_SZ            = 1;
localparam HW_ACC_SZ            = $clog2(NUM_HW_ACCELERATORS);
localparam NUM_TGTS_SZ          = 3;
localparam ENTRY_SBUFF1_START   = 0;
localparam ENTRY_SBUFF1_END     = ENTRY_SBUFF1_START + BUFF_INDX_SZ - 1;
localparam ENTRY_SBUFF2_START   = ENTRY_SBUFF1_END   + 1;
localparam ENTRY_SBUFF2_END     = ENTRY_SBUFF2_START + BUFF_INDX_SZ - 1;
localparam ENTRY_SBUFF3_START   = ENTRY_SBUFF2_END   + 1;
localparam ENTRY_SBUFF3_END     = ENTRY_SBUFF3_START + BUFF_INDX_SZ - 1;
localparam ENTRY_TBUFF_START    = ENTRY_SBUFF3_END   + 1;
localparam ENTRY_TBUFF_END      = ENTRY_TBUFF_START  + BUFF_INDX_SZ - 1;
localparam ENTRY_NUM_TGTS_START = ENTRY_TBUFF_END    + 1;
localparam ENTRY_NUM_TGTS_END   = ENTRY_NUM_TGTS_START  + NUM_TGTS_SZ - 1;
localparam ENTRY_COLOUR         = ENTRY_NUM_TGTS_END    + 1;
localparam ENTRY_ACC_ID_START   = ENTRY_COLOUR       + 1;
localparam ENTRY_ACC_ID_END     = ENTRY_ACC_ID_START + HW_ACC_SZ - 1;
localparam ENTRY_CFG_ID_START   = ENTRY_ACC_ID_END   + 1;
localparam ENTRY_CFG_ID_END     = ENTRY_CFG_ID_START + CFG_ID_SZ-1;
localparam ENTRY_DATA_SZ        = ENTRY_CFG_ID_END   + 1;
localparam INST_TASK            = 3'b000;
localparam INST_JUMP            = 3'b001;
localparam INST_STOP            = 3'b010;
localparam INST_CHECK           = 3'b011;
localparam INST_NXT             = 3'b100;
localparam INST_FILL            = 3'b101;
localparam INST_LOOP            = 3'b110;
localparam INST_LOOPEND         = 3'b111;
localparam NUM_LOOPS            = 8;
localparam LOOP_ID_SZ           = 3;
localparam LOOP_CNT_SZ          = 26;  // inst_word[31:6]

wire                           start_new_task;
wire [ENTRY_DATA_SZ-1:0]       new_entry_data;
wire [ENTRY_DATA_SZ-1:0]       d;
wire [ENTRY_DATA_SZ-1:0]       entry_data_to_be_launched;
//wire [SCH_ENTRY_SZ-1:0]        entry_data_o;
wire [COL_BUFF_ID_SZ-1:0]      buff_full;
wire [COL_BUFF_ID_SZ-1:0]      buff_free;
wire                           load_new_entry;
reg                            prog_running_r;
wire                           prog_running_nxt;
wire                           keep_fetching;
reg  [PROG_ADDR_BITS-1:0]      prog_counter_r;
wire [PROG_ADDR_BITS-1:0]      prog_counter_nxt;
reg                            prog_stopping_r;
reg                            word_ready_r;
wire                           do_jump;
wire                           goto_nxt;
//wire [NUM_HW_ACCELERATORS-1:0] acc_finished_i;
wire  [PROG_DATA_BITS-1:0]     inst_word;
reg   [PROG_DATA_BITS-1:0]     held_inst_word_r;
reg                            inst_word_valid_r;
wire                           inst_word_valid_nxt;
wire                           inst_valid;
wire                           inst_consumed;
reg                            inst_is_task;
reg                            inst_is_jump;
reg                            inst_is_stop;
reg                            inst_is_check;
reg                            inst_is_fill;
reg                            inst_is_nxt;
reg                            inst_is_loop;
reg                            inst_is_loopend;
reg                            inst_unknown;

wire [LOOP_ID_SZ-1:0]          loop_id_w;
wire [LOOP_CNT_SZ-1:0]         loop_max_val_w;
wire                           loopend_active;
reg  [LOOP_CNT_SZ-1:0]         loop_counter_r [0:NUM_LOOPS-1];
reg  [PROG_ADDR_BITS-1:0]      loop_restart_r [0:NUM_LOOPS-1];

// CHECK instruction decode
wire [BUFF_INDX_SZ-1:0]       check_buff_id;
wire                           check_mode;
wire [PROG_ADDR_BITS-1:0]      check_skip_addr_w;
wire                           check_result_ready;
wire                           check_success;
wire [PROG_ADDR_BITS-1:0]      jump_target;
wire [NUM_BUFFERS-1:0]         target_status;

wire [TGT_ACC_SZ-1:0]          acc_available;

wire                           fill_complete;
reg  [NUM_BUFFERS-1:0]         tgt_selected;
wire [NUM_BUFFERS-1:0]         buff_colour;

wire  [BUFF_INDX_SZ-1:0]       to_launch_tgt_buff_id;
wire  [BUFF_INDX_SZ-1:0]       to_launch_src1_buff_id;
wire  [BUFF_INDX_SZ-1:0]       to_launch_src2_buff_id;
wire  [BUFF_INDX_SZ-1:0]       to_launch_src3_buff_id;
wire     [HW_ACC_SZ-1:0]       to_launch_acc_hw_id;
wire  [TGT_COUNT_SZ-1:0]       to_launch_num_tgts;
wire     [CFG_ID_SZ-1:0]       to_launch_cfg_id;
wire                           to_launch_tgt_colour;
wire                           table_slot_free;
wire                           table_empty;

// Temporary drivers:
assign target_acc_o       = entry_data_to_be_launched[ENTRY_ACC_ID_END:ENTRY_ACC_ID_START];
assign buffer_info_o      = entry_data_to_be_launched;

// Unpack scheduler entry to deploy the fields:
assign to_launch_src1_buff_id = 
       entry_data_to_be_launched[ENTRY_SBUFF1_END:ENTRY_SBUFF1_START];
assign to_launch_src2_buff_id = 
       entry_data_to_be_launched[ENTRY_SBUFF2_END:ENTRY_SBUFF2_START];
assign to_launch_src3_buff_id = 
       entry_data_to_be_launched[ENTRY_SBUFF3_END:ENTRY_SBUFF3_START];
assign to_launch_tgt_buff_id  = 
       entry_data_to_be_launched[ENTRY_TBUFF_END:ENTRY_TBUFF_START];
assign to_launch_num_tgts     =
       entry_data_to_be_launched[ENTRY_NUM_TGTS_END:ENTRY_NUM_TGTS_START];
assign to_launch_acc_hw_id    = 
       entry_data_to_be_launched[ENTRY_ACC_ID_END:ENTRY_ACC_ID_START];
assign to_launch_tgt_colour   = 
       entry_data_to_be_launched[ENTRY_COLOUR];
assign to_launch_cfg_id       = 
       entry_data_to_be_launched[ENTRY_CFG_ID_END:ENTRY_CFG_ID_START];

////////////////////////////////////////////////////////
// Mini-cpu to run the NN program
// (will pull this out as a separate module later)

// CHECK instruction decode
assign check_buff_id     = inst_word[3+BUFF_INDX_SZ:4];  // lower BUFF_INDX_SZ bits of [10:4]
assign check_mode        = inst_word[3];                  // 1=finish-on-success, 0=skip-on-success
assign check_skip_addr_w = inst_word[PROG_ADDR_BITS+11:12];
assign check_result_ready = buff_full[check_buff_id];    // stall until producing TASK completes
assign check_success     = ~target_status[check_buff_id]; // result=0 means SUCCESS

// LOOP field decode
assign loop_id_w      = inst_word[5:3];
assign loop_max_val_w = inst_word[31:6];
assign loopend_active = inst_is_loopend & (loop_counter_r[loop_id_w] != 'b0);

// JUMP / CHECK / LOOPEND all share the jump mux
assign jump_target = inst_is_jump    ? inst_word[PROG_ADDR_BITS+2:3]  // JUMP: bits[31:3] word-addressed
                   : inst_is_loopend ? loop_restart_r[loop_id_w]
                   :                   check_skip_addr_w;

assign do_jump = (inst_is_jump    & inst_consumed)
               |(inst_is_check   & inst_consumed & check_success & ~check_mode)
               |(inst_is_loopend & inst_consumed & loopend_active);

assign goto_nxt         = inst_consumed;

assign prog_counter_nxt = (start_program_i) ? program_addr_i
                        : do_jump           ? jump_target
                        : goto_nxt          ? prog_counter_r + 1
                        :                     prog_counter_r;

assign prog_running_nxt = (start_program_i) ? 1'b1 : prog_running_r;

assign prog_stopping_nxt =
    (prog_running_r & inst_valid & inst_is_stop) ? 1'b1 :
    (prog_running_r & inst_valid & inst_is_check &
     check_result_ready & check_success & check_mode) ? 1'b1 :
    1'b0;

assign inst_word_valid_nxt = (word_ready_r      & ~inst_consumed) ? 1'b1
                           : (inst_word_valid_r & ~inst_consumed) ? 1'b1
                           : (~word_ready_r     &  inst_consumed) ? 1'b0
                           : inst_word_valid_r;

always @ (posedge clk)
begin
   if (reset)
   begin
      prog_running_r        <= 1'b0;
      prog_counter_r        <=  'b0;
      prog_stopping_r       <= 1'b0;
      inst_word_valid_r     <= 1'b0;
      held_inst_word_r      <=  'b0;
   end
   else
   begin
      prog_running_r        <= prog_running_nxt;
      inst_word_valid_r     <= inst_word_valid_nxt;
      prog_stopping_r       <= prog_stopping_nxt;
      if (prog_running_nxt)
         prog_counter_r     <= prog_counter_nxt;
      if (word_ready_r      & ~inst_consumed)
         held_inst_word_r   <= prog_mem_data_i;
   end
end

assign inst_word = (inst_word_valid_r) ? held_inst_word_r : prog_mem_data_i;

// Loop counter and restart address registers
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

// If we are running but not coming to a halt (after STOP inst) then
// keep fetching progrsm words:
assign keep_fetching = prog_running_r & ~prog_stopping_r & ~inst_is_stop;

assign prog_mem_addr_o = prog_counter_r;

assign prog_mem_req_o = keep_fetching & (~inst_valid | ~inst_word_valid_nxt);

always @ (posedge clk)
begin
   if (reset)
      word_ready_r <= 1'b0;
   else if (prog_mem_req_o & ~prog_mem_wait_i)
      word_ready_r <= 1'b1;
   else
      word_ready_r <= 1'b0;
end

// Decode instruction word:
//
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
   case(inst_word[2:0])
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

//assign inst_is_task  = (prog_mem_data_i[2:0] == INST_TASK)  ? 1'b1 : 1'b0;
//assign inst_is_stop  = (prog_mem_data_i[2:0] == INST_STOP)  ? 1'b1 : 1'b0;
//assign inst_is_check = (prog_mem_data_i[2:0] == INST_CHECK) ? 1'b1 : 1'b0;
//assign inst_is_fill  = (prog_mem_data_i[2:0] == INST_FILL)  ? 1'b1 : 1'b0;
end

// For TASK instructions, add fields to the table:
//
assign d[ENTRY_TBUFF_END:ENTRY_TBUFF_START]       = inst_word[7:3];
assign d[ENTRY_COLOUR]                            = inst_word[8];
assign d[ENTRY_NUM_TGTS_END:ENTRY_NUM_TGTS_START] = inst_word[11:9];
assign d[ENTRY_CFG_ID_END:ENTRY_CFG_ID_START]     = inst_word[16:12];
assign d[ENTRY_ACC_ID_END:ENTRY_ACC_ID_START]     = inst_word[19:17];
assign d[ENTRY_SBUFF1_END:ENTRY_SBUFF1_START]     = inst_word[24:20];
assign d[ENTRY_SBUFF2_END:ENTRY_SBUFF2_START]     = inst_word[29:25];
assign d[ENTRY_SBUFF3_END:ENTRY_SBUFF3_START]     = inst_word[29:25]; // Duplicate!

assign new_entry_data = d;

// Control flow: 
//
// Instruction can either be in the local buffer (inst_word_r) or coming
// directly from the memory (word_ready_r):
assign inst_valid = word_ready_r | inst_word_valid_r;

// Signal when we have executed an instruction and can fetch the next one:
// Note that external signal test_stall_pipe can block this when asserted,
// for test purposes.
assign inst_consumed = inst_valid & (
                         (inst_is_task  & table_slot_free)
                       |  inst_is_jump
                       | (inst_is_check & check_result_ready)
                       | (inst_is_fill  & fill_complete)
                       |  inst_is_loop
                       |  inst_is_loopend
                       | (inst_is_nxt   & table_empty)
                     ) & ~test_stall_pipe;

assign load_new_entry     = (inst_valid & inst_is_task & inst_consumed) ?
                            1'b1 : 1'b0;

assign nxt_input_pulse_o  = inst_valid & inst_is_nxt & inst_word[4] & table_empty & ~test_stall_pipe;
assign nxt_output_pulse_o = inst_valid & inst_is_nxt & inst_word[5] & table_empty & ~test_stall_pipe;

/////////////////////////////////////////
// Fill instruction state machine:
// Reads data from given memory location and writes it 
// to a buffer
//
// Check if the target buffer is free:
// convert binary tgt ID into one-hot:
always @ (d[ENTRY_TBUFF_END:ENTRY_TBUFF_START])
begin
   tgt_selected = 'b0;
   tgt_selected[d[ENTRY_TBUFF_END:ENTRY_TBUFF_START]] = 1'b1;
end

//assign tgt_free = ((tgt_selected & buff_free) != 'b0)? 1'b1 : 1'b0;
assign fill_complete = 1'b0;//TODO

// End of mini-cpu block

////////////////////////////////////////////////////////
// instantiate sub-components

// Scheduler table - list of tasks waiting for data and 
// assignment to an accelerator
//
sch_table  #(
	      .SCH_ENTRY_SZ(ENTRY_DATA_SZ),
	      .NUM_HW_ACCELERATORS(NUM_HW_ACCELERATORS),
              .TGT_ACC_SZ(HW_ACC_SZ),
              .NUM_BUFFERS(NUM_BUFFERS),
              .COL_BUFF_ID_SZ(COL_BUFF_ID_SZ),
              .NUM_SCH_ENTRIES(NUM_SCH_ENTRIES),
              .ACC_ID_BTM(ENTRY_ACC_ID_START)
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
             .buffers_free_i(buff_free)
             );

assign start_new_block_o = start_new_task;

// Instantiate tracking table for buffers and HW accelerators:
//
//wire  [BUFF_INDX_SZ-1:0] to_launch_tgt_buff_id;
//wire  [BUFF_INDX_SZ-1:0] to_launch_src1_buff_id;
//wire  [BUFF_INDX_SZ-1:0] to_launch_src2_buff_id;
//wire  [BUFF_INDX_SZ-1:0] to_launch_src3_buff_id;
//wire  [HW_ACC_SZ-1:0]    to_launch_acc_hw_id;
//wire                     to_launch_tgt_colour;

sch_buffer_state
       #(.NUM_BUFFERS(NUM_BUFFERS),
         .BUFF_INDX_SZ(BUFF_INDX_SZ),
         .TGT_ACC_SZ(HW_ACC_SZ),
         .TGT_COUNT_SZ(TGT_COUNT_SZ))
	 sch_buff_state0
       (.clk(clk),
        .reset(reset),
        .acc_busy_i(acc_busy_i),
        .acc_finished_i(acc_finished_i),
        .acc_result_i(acc_result_i),
	.mark_buff_as_full_i(mark_buff_as_full_i),
        .full_buff_id_i(full_buff_id_i),
        .full_buff_usage_i(full_buff_usage_i),
        .start_new_task_i(start_new_task),
        .tgt_acc_id_i(to_launch_acc_hw_id),
        .tgt_buff_idx_i(to_launch_tgt_buff_id),
        .tgt_usage_count_i(to_launch_num_tgts),
        .tgt_colour_i(to_launch_tgt_colour),
        .src1_buff_idx_i(to_launch_src1_buff_id),
        .src2_buff_idx_i(to_launch_src2_buff_id),
        .src3_buff_idx_i(to_launch_src3_buff_id),
        .acc_available_o(acc_available),
        .buffers_full_o(buff_full),
        .buffers_free_o(buff_free),
        .buffers_colour_o(buff_colour),
        .target_status_o(target_status)
        );

endmodule
