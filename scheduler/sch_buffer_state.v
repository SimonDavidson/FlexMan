// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
////////////////////////////////////////////////////////////////////////
//
// sch_buffer_state
//
// Author: Simon D.
// Version: 1.0
// Date 7/2/2025
//
// Tracks usage and status of buffers and accelerators 
// for the scheduler.
//

module sch_buffer_state
      #(parameter NUM_BUFFERS         = 16,
        parameter BUFF_INDX_SZ        = $clog2(NUM_BUFFERS),
	parameter NUM_HW_ACCELERATORS = 2,
	parameter TGT_ACC_SZ          = $clog2(NUM_HW_ACCELERATORS),
        parameter TGT_COUNT_SZ        = 3
       )
       (input  wire                    clk,
        input  wire                    reset,

	// Process info from hardware accelerators:
	input wire [NUM_HW_ACCELERATORS-1:0] acc_busy_i,
	input wire [NUM_HW_ACCELERATORS-1:0] acc_finished_i,
	input wire [NUM_HW_ACCELERATORS-1:0] acc_result_i,

	// Interface from scheduler control:
	input wire                     mark_buff_as_full_i,
	input wire  [BUFF_INDX_SZ-1:0] full_buff_id_i,
	input wire  [TGT_COUNT_SZ-1:0] full_buff_usage_i,
	input wire                     start_new_task_i,
	input wire  [TGT_ACC_SZ-1:0]   tgt_acc_id_i,
	input wire  [BUFF_INDX_SZ-1:0] tgt_buff_idx_i,
	input wire  [TGT_COUNT_SZ-1:0] tgt_usage_count_i,
	input wire                     tgt_colour_i,
	input wire  [BUFF_INDX_SZ-1:0] src1_buff_idx_i,
	input wire  [BUFF_INDX_SZ-1:0] src2_buff_idx_i,
	input wire  [BUFF_INDX_SZ-1:0] src3_buff_idx_i,

	// Broadcast current accelerator status:
	output wire [NUM_HW_ACCELERATORS-1:0] acc_available_o,

	// Broadcast current buffer status:
	output wire [NUM_BUFFERS-1:0]  buffers_full_o,
        output wire [NUM_BUFFERS-1:0]  buffers_free_o,
	output wire [NUM_BUFFERS-1:0]  buffers_colour_o,
	output wire [NUM_BUFFERS-1:0]  target_status_o
        );

//localparam BUFF_INDX_SZ = $clog2(NUM_BUFFERS);
localparam ACC_ENTRY_SZ = BUFF_INDX_SZ;

reg  [NUM_HW_ACCELERATORS-1:0] selected_finisher;
reg  [TGT_ACC_SZ-1:0]          selected_acc_idx;
reg  [NUM_HW_ACCELERATORS-1:0] acc_process_pending_r;
wire [NUM_HW_ACCELERATORS-1:0] acc_process_pending_nxt;
//reg  [TGT_ACC_SZ-1:0]          acc_process_pending_r;
//reg  [TGT_ACC_SZ-1:0]          acc_process_pending_nxt;
reg  [NUM_HW_ACCELERATORS-1:0] new_task;
wire [NUM_HW_ACCELERATORS-1:0] acc_free;
wire                           post_task_cleanup;
wire [BUFF_INDX_SZ-1:0]  tgt_buff_r [0:NUM_HW_ACCELERATORS-1];
wire [BUFF_INDX_SZ-1:0] src1_buff_r [0:NUM_HW_ACCELERATORS-1];
wire [BUFF_INDX_SZ-1:0] src2_buff_r [0:NUM_HW_ACCELERATORS-1];
wire [BUFF_INDX_SZ-1:0] src3_buff_r [0:NUM_HW_ACCELERATORS-1];
reg  [BUFF_INDX_SZ-1:0]        selected_tgt_buff;
reg  [BUFF_INDX_SZ-1:0]        selected_src1_buff;
reg  [BUFF_INDX_SZ-1:0]        selected_src2_buff;
reg  [BUFF_INDX_SZ-1:0]        selected_src3_buff;
reg  [NUM_BUFFERS-1:0]         target_status_r;
reg  [NUM_BUFFERS-1:0]         buff_new_tgt;
reg  [NUM_BUFFERS-1:0]         buff_now_full;
wire [NUM_BUFFERS-1:0]         buff_content_consumed;
reg  [NUM_BUFFERS-1:0]         src1_buff_consumed;
reg  [NUM_BUFFERS-1:0]         src2_buff_consumed;
reg  [NUM_BUFFERS-1:0]         src3_buff_consumed;
wire [TGT_COUNT_SZ-1:0]        buff_new_usage_count;
wire [TGT_COUNT_SZ-1:0] buff_usage_count [0:NUM_BUFFERS-1];
wire                           buff_new_colour;
reg  [NUM_BUFFERS-1:0]         mark_as_full;

////////////////////////////////////////////////////////////
// Mark a buffer as full.
// Two scenarios where this needs to happen:
//
// 1) Data is arriving from an external input source.
//    When that data block is ready to use, signal this to the
//    scheduler by marking the input buffer as full, i.e. ready.
//    Signalled using a 'mark_full' command.
//
// 2) When an explicit 'fill' command is executed by the scheduler
//    causing a data copy from one place to another. The destination
//    is a buffer.
//
// Convert buffer ID (index) into one hot encoding (enable):

always @ (full_buff_id_i, mark_buff_as_full_i)
   begin
      mark_as_full = 'b0;
      if (mark_buff_as_full_i)
         mark_as_full[full_buff_id_i] = 1'b1;
   end

////////////////////////////////////////////////////////////
// Add a new active task to the queues (and launch it!)
//
// The input signals for this come from the scheduler table,
// which choses a task that is ready to run.
//
// Actions:
// 1) For the target buffer, modify its entry in the buffers
//    list, so we know that it is no longer free, but not full
//    yet. Also, store how many future tasks will consume this
//    buffer once it is full.
//
// 2) Update the entry for the chosen hardware acc unit,
//    to track the buffer usage (target buffer and sources)
//

// Convert binary target buffer ID into one-hot encoding:
always @ (tgt_buff_idx_i, start_new_task_i)
   begin
      buff_new_tgt = 'b0;
      if (start_new_task_i)
         buff_new_tgt[tgt_buff_idx_i] = 1'b1;
   end

// Convert binary ACC ID into one-hot encoding:
always @ (tgt_acc_id_i, start_new_task_i)
   begin
      new_task = 'b0;
      if (start_new_task_i)
         new_task[tgt_acc_id_i] = 1'b1;
   end

////////////////////////////////////////////////////////////
// Track when tasks are complete
//
// Capture events from individual compute units. 
// We can only deal with one at a time, so treat 
// them like interrupts to process in turn.
//

// One-hot queue of finished tasks:
// OR in any newly-finished tasks and removed those already dealt with:
assign acc_process_pending_nxt = (acc_process_pending_r | acc_finished_i) 
                                & ~selected_finisher;

always @ (posedge clk)
begin
   if (reset)
      acc_process_pending_r <= 'b0;   
   else 
      acc_process_pending_r <= acc_process_pending_nxt;
end

assign post_task_cleanup = |selected_finisher;

////////////////////////////////////////////////////////////
// Select completed task to remove from the queues
//

// selected_finisher will have a '1' in the earliest bit position of
// a finished HW unit, and '0's everywhere else:

integer i;
reg     found;
always @*
begin
   selected_finisher  = 'b0;
   selected_acc_idx   = 'b0;
   selected_tgt_buff  = 'b0;
   selected_src1_buff = 'b0;
   selected_src2_buff = 'b0;
   selected_src3_buff = 'b0;
   found              = 1'b0;
   for(i=0; i<NUM_HW_ACCELERATORS; i=i+1) begin
      if (!found && acc_process_pending_r[i])
         begin
	    selected_finisher[i] = 1'b1;
	    selected_acc_idx      = i[TGT_ACC_SZ-1:0];
	    selected_tgt_buff     =  tgt_buff_r[i];
	    selected_src1_buff    = src1_buff_r[i];
	    selected_src2_buff    = src2_buff_r[i];
	    selected_src3_buff    = src3_buff_r[i];
	    found                 = 1'b1;
         end
   end
end

// Convert selected target and sources from binary to 1-hot:
// Target buffer is now full and can be consumed by other tasks:
always @ (selected_tgt_buff, post_task_cleanup)
   begin
      buff_now_full = 'b0;
      if (post_task_cleanup)
         buff_now_full[selected_tgt_buff] = 1'b1;
   end

always @ (selected_src1_buff, post_task_cleanup)
   begin
      src1_buff_consumed = 'b0;
      if (post_task_cleanup)
         src1_buff_consumed[selected_src1_buff] = 1'b1;
   end

always @ (selected_src2_buff, post_task_cleanup)
   begin
      src2_buff_consumed = 'b0;
      if (post_task_cleanup)
         src2_buff_consumed[selected_src2_buff] = 1'b1;
   end

always @ (selected_src3_buff, post_task_cleanup)
   begin
      src3_buff_consumed = 'b0;
      if (post_task_cleanup)
         src3_buff_consumed[selected_src3_buff] = 1'b1;
   end

assign buff_content_consumed = src1_buff_consumed
                             | src2_buff_consumed
			     | src3_buff_consumed;

////////////////////////////////////////////////////////////
// Remove entry at end of task
// 
// Given that a HW acc unit has signalled that it has finished
// we must:
//
// 1) From the list by accelerator, get the target and source buffer list.
//
// 2) For the target buffer, go to the buffer list and mark it as full.
//
// 3) For each source, mark them as having consumed one of their uses.
//    This will mark them as free if the usage counter reaches zero.
//

genvar ai;
generate
   for (ai = 0; ai < NUM_HW_ACCELERATORS; ai = ai + 1) begin : gen_acc
      acc_hw_buffer_tracker #(.BUFF_INDX_SZ(BUFF_INDX_SZ)) hw_n (
         .clk(clk),
         .reset(reset),
         .new_task_i(new_task[ai]),
         .tgt_buff_i(tgt_buff_idx_i),
         .src1_buff_i(src1_buff_idx_i),
         .src2_buff_i(src2_buff_idx_i),
         .src3_buff_i(src3_buff_idx_i),
         .task_finished_i(selected_finisher[ai]),
         .acc_free_o(acc_free[ai]),
         .tgt_buff_o(tgt_buff_r[ai]),
         .src1_buff_o(src1_buff_r[ai]),
         .src2_buff_o(src2_buff_r[ai]),
         .src3_buff_o(src3_buff_r[ai])
      );
   end
endgenerate

assign acc_available_o = acc_free;

////////////////////////////////////////////////////////////
// Instantiate per-buffer state, to track fullness, freeness
// and outstanding targets for each buffer.
//
assign buff_new_usage_count = tgt_usage_count_i;
assign buff_new_colour      = tgt_colour_i;

genvar bi;
generate
   for (bi = 0; bi < NUM_BUFFERS; bi = bi + 1) begin : gen_buf
      buffer_state_entry #(.TGT_COUNT_SZ(TGT_COUNT_SZ)) buffer_n (
         .clk(clk),
         .reset(reset),
         .mark_as_full_i(mark_as_full[bi]),
         .mark_buff_usage_i(full_buff_usage_i),
         .buff_new_tgt_i(buff_new_tgt[bi]),
         .buff_new_usage_count_i(buff_new_usage_count),
         .buff_new_colour_i(buff_new_colour),
         .buff_now_full_i(buff_now_full[bi]),
         .buff_content_consumed_i(buff_content_consumed[bi]),
         .buff_usage_count_o(buff_usage_count[bi]),
         .buff_colour_o(buffers_colour_o[bi]),
         .buff_free_o(buffers_free_o[bi]),
         .buff_full_o(buffers_full_o[bi])
      );
   end
endgenerate

////////////////////////////////////////////////////////////
// Per-buffer task result register file.
// Captures acc_result_i for the completing accelerator's
// target buffer when that accelerator signals finished.
// 1 = FAIL, 0 = SUCCESS.
//
always @ (posedge clk)
begin
   if (reset)
      target_status_r <= 'b0;
   else if (post_task_cleanup)
      target_status_r[selected_tgt_buff] <= acc_result_i[selected_acc_idx];
end

assign target_status_o = target_status_r;

endmodule	// sch_buffer_state

