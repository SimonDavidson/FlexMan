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
	input wire               [2:0] full_buff_usage_i,
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
wire [2:0]                     buff_new_usage_count;
wire [2:0] buff_usage_count [0:NUM_BUFFERS-1];
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
      mark_as_full <= 'b0;
      if (mark_buff_as_full_i)
         case (full_buff_id_i)
            4'b0000: mark_as_full[0]  <= 1'b1;
            4'b0001: mark_as_full[1]  <= 1'b1;
            4'b0010: mark_as_full[2]  <= 1'b1;
            4'b0011: mark_as_full[3]  <= 1'b1;
            4'b0100: mark_as_full[4]  <= 1'b1;
            4'b0101: mark_as_full[5]  <= 1'b1;
            4'b0110: mark_as_full[6]  <= 1'b1;
            4'b0111: mark_as_full[7]  <= 1'b1;
            4'b1000: mark_as_full[8]  <= 1'b1;
            4'b1001: mark_as_full[9]  <= 1'b1;
            4'b1010: mark_as_full[10] <= 1'b1;
            4'b1011: mark_as_full[11] <= 1'b1;
            4'b1100: mark_as_full[12] <= 1'b1;
            4'b1101: mark_as_full[13] <= 1'b1;
            4'b1110: mark_as_full[14] <= 1'b1;
            4'b1111: mark_as_full[15] <= 1'b1;
	    default: mark_as_full[0]  <= 1'b0;
         endcase
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
         case (tgt_buff_idx_i)
            4'b0000: buff_new_tgt[0]  = 1'b1;
            4'b0001: buff_new_tgt[1]  = 1'b1;
            4'b0010: buff_new_tgt[2]  = 1'b1;
            4'b0011: buff_new_tgt[3]  = 1'b1;
            4'b0100: buff_new_tgt[4]  = 1'b1;
            4'b0101: buff_new_tgt[5]  = 1'b1;
            4'b0110: buff_new_tgt[6]  = 1'b1;
            4'b0111: buff_new_tgt[7]  = 1'b1;
            4'b1000: buff_new_tgt[8]  = 1'b1;
            4'b1001: buff_new_tgt[9]  = 1'b1;
            4'b1010: buff_new_tgt[10] = 1'b1;
            4'b1011: buff_new_tgt[11] = 1'b1;
            4'b1100: buff_new_tgt[12] = 1'b1;
            4'b1101: buff_new_tgt[13] = 1'b1;
            4'b1110: buff_new_tgt[14] = 1'b1;
            4'b1111: buff_new_tgt[15] = 1'b1;
	    default: buff_new_tgt[0]  = 1'b0;
         endcase
   end

// Convert binary ACC ID into one-hot encoding:
always @ (tgt_acc_id_i, start_new_task_i)
   begin
      new_task <= 'b0;
      case (tgt_acc_id_i)
         1'b0:    new_task[0] <= start_new_task_i;
         1'b1:    new_task[1] <= start_new_task_i;
	 default: new_task[0] <= 1'b0;
      endcase 
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
      buff_now_full <= 'b0;
      if (post_task_cleanup)
         case (selected_tgt_buff)
            4'b0000: buff_now_full[0]  <= 1'b1;
            4'b0001: buff_now_full[1]  <= 1'b1;
            4'b0010: buff_now_full[2]  <= 1'b1;
            4'b0011: buff_now_full[3]  <= 1'b1;
            4'b0100: buff_now_full[4]  <= 1'b1;
            4'b0101: buff_now_full[5]  <= 1'b1;
            4'b0110: buff_now_full[6]  <= 1'b1;
            4'b0111: buff_now_full[7]  <= 1'b1;
            4'b1000: buff_now_full[8]  <= 1'b1;
            4'b1001: buff_now_full[9]  <= 1'b1;
            4'b1010: buff_now_full[10] <= 1'b1;
            4'b1011: buff_now_full[11] <= 1'b1;
            4'b1100: buff_now_full[12] <= 1'b1;
            4'b1101: buff_now_full[13] <= 1'b1;
            4'b1110: buff_now_full[14] <= 1'b1;
            4'b1111: buff_now_full[15] <= 1'b1;
            default: buff_now_full[0]  <= 1'b0;
         endcase
   end

always @ (selected_src1_buff, post_task_cleanup)
   begin
      src1_buff_consumed <= 'b0;
      if (post_task_cleanup)
         case (selected_src1_buff)
            4'b0000: src1_buff_consumed[0]  <= 1'b1;
            4'b0001: src1_buff_consumed[1]  <= 1'b1;
            4'b0010: src1_buff_consumed[2]  <= 1'b1;
            4'b0011: src1_buff_consumed[3]  <= 1'b1;
            4'b0100: src1_buff_consumed[4]  <= 1'b1;
            4'b0101: src1_buff_consumed[5]  <= 1'b1;
            4'b0110: src1_buff_consumed[6]  <= 1'b1;
            4'b0111: src1_buff_consumed[7]  <= 1'b1;
            4'b1000: src1_buff_consumed[8]  <= 1'b1;
            4'b1001: src1_buff_consumed[9]  <= 1'b1;
            4'b1010: src1_buff_consumed[10] <= 1'b1;
            4'b1011: src1_buff_consumed[11] <= 1'b1;
            4'b1100: src1_buff_consumed[12] <= 1'b1;
            4'b1101: src1_buff_consumed[13] <= 1'b1;
            4'b1110: src1_buff_consumed[14] <= 1'b1;
            4'b1111: src1_buff_consumed[15] <= 1'b1;
            default: src1_buff_consumed[0]  <= 1'b0;
         endcase
   end

always @ (selected_src2_buff, post_task_cleanup)
   begin
      src2_buff_consumed <= 'b0;
      if (post_task_cleanup)
         case (selected_src2_buff)
            4'b0000: src2_buff_consumed[0]  <= 1'b1;
            4'b0001: src2_buff_consumed[1]  <= 1'b1;
            4'b0010: src2_buff_consumed[2]  <= 1'b1;
            4'b0011: src2_buff_consumed[3]  <= 1'b1;
            4'b0100: src2_buff_consumed[4]  <= 1'b1;
            4'b0101: src2_buff_consumed[5]  <= 1'b1;
            4'b0110: src2_buff_consumed[6]  <= 1'b1;
            4'b0111: src2_buff_consumed[7]  <= 1'b1;
            4'b1000: src2_buff_consumed[8]  <= 1'b1;
            4'b1001: src2_buff_consumed[9]  <= 1'b1;
            4'b1010: src2_buff_consumed[10] <= 1'b1;
            4'b1011: src2_buff_consumed[11] <= 1'b1;
            4'b1100: src2_buff_consumed[12] <= 1'b1;
            4'b1101: src2_buff_consumed[13] <= 1'b1;
            4'b1110: src2_buff_consumed[14] <= 1'b1;
            4'b1111: src2_buff_consumed[15] <= 1'b1;
            default: src2_buff_consumed[0]  <= 1'b0;
         endcase
   end

always @ (selected_src3_buff, post_task_cleanup)
   begin
      src3_buff_consumed <= 'b0;
      if (post_task_cleanup)
         case (selected_src3_buff)
            4'b0000: src3_buff_consumed[0]  <= 1'b1;
            4'b0001: src3_buff_consumed[1]  <= 1'b1;
            4'b0010: src3_buff_consumed[2]  <= 1'b1;
            4'b0011: src3_buff_consumed[3]  <= 1'b1;
            4'b0100: src3_buff_consumed[4]  <= 1'b1;
            4'b0101: src3_buff_consumed[5]  <= 1'b1;
            4'b0110: src3_buff_consumed[6]  <= 1'b1;
            4'b0111: src3_buff_consumed[7]  <= 1'b1;
            4'b1000: src3_buff_consumed[8]  <= 1'b1;
            4'b1001: src3_buff_consumed[9]  <= 1'b1;
            4'b1010: src3_buff_consumed[10] <= 1'b1;
            4'b1011: src3_buff_consumed[11] <= 1'b1;
            4'b1100: src3_buff_consumed[12] <= 1'b1;
            4'b1101: src3_buff_consumed[13] <= 1'b1;
            4'b1110: src3_buff_consumed[14] <= 1'b1;
            4'b1111: src3_buff_consumed[15] <= 1'b1;
            default: src3_buff_consumed[0]  <= 1'b0;
         endcase
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

acc_hw_buffer_tracker hw0 
   (clk,
    reset,
    new_task[0],
    tgt_buff_idx_i,
    src1_buff_idx_i,
    src2_buff_idx_i,
    src3_buff_idx_i,
    selected_finisher[0],
    acc_free[0],
    tgt_buff_r[0],
    src1_buff_r[0],
    src2_buff_r[0],
    src3_buff_r[0]
   );

acc_hw_buffer_tracker hw1 
   (clk,
    reset,
    new_task[1],
    tgt_buff_idx_i,
    src1_buff_idx_i,
    src2_buff_idx_i,
    src3_buff_idx_i,
    selected_finisher[1],
    acc_free[1],
    tgt_buff_r[1],
    src1_buff_r[1],
    src2_buff_r[1],
    src3_buff_r[1]
   );

assign acc_available_o = acc_free;

////////////////////////////////////////////////////////////
// Instantiate per-buffer state, to track fullness, freeness
// and outstanding targets for each buffer.
//
assign buff_new_usage_count = tgt_usage_count_i;
assign buff_new_colour      = tgt_colour_i;

//assign 
buffer_state_entry buffer0
         (clk,
          reset,
	  mark_as_full[0],
	  full_buff_usage_i,
          buff_new_tgt[0],
          buff_new_usage_count,
          buff_new_colour,
          buff_now_full[0],
          buff_content_consumed[0],
          buff_usage_count[0],
          buffers_colour_o[0],
          buffers_free_o[0],
          buffers_full_o[0]
         );

buffer_state_entry buffer1
         (clk,
          reset,
	  mark_as_full[1],
	  full_buff_usage_i,
          buff_new_tgt[1],
          buff_new_usage_count,
          buff_new_colour,
          buff_now_full[1],
          buff_content_consumed[1],
          buff_usage_count[1],
          buffers_colour_o[1],
          buffers_free_o[1],
          buffers_full_o[1]
         );

buffer_state_entry buffer2
         (clk,
          reset,
	  mark_as_full[2],
	  full_buff_usage_i,
          buff_new_tgt[2],
          buff_new_usage_count,
          buff_new_colour,
          buff_now_full[2],
          buff_content_consumed[2],
          buff_usage_count[2],
          buffers_colour_o[2],
          buffers_free_o[2],
          buffers_full_o[2]
         );

buffer_state_entry buffer3
         (clk,
          reset,
	  mark_as_full[3],
	  full_buff_usage_i,
          buff_new_tgt[3],
          buff_new_usage_count,
          buff_new_colour,
          buff_now_full[3],
          buff_content_consumed[3],
          buff_usage_count[3],
          buffers_colour_o[3],
          buffers_free_o[3],
          buffers_full_o[3]
         );

buffer_state_entry buffer4
         (clk,
          reset,
	  mark_as_full[4],
	  full_buff_usage_i,
          buff_new_tgt[4],
          buff_new_usage_count,
          buff_new_colour,
          buff_now_full[4],
          buff_content_consumed[4],
          buff_usage_count[4],
          buffers_colour_o[4],
          buffers_free_o[4],
          buffers_full_o[4]
         );

buffer_state_entry buffer5
         (clk,
          reset,
	  mark_as_full[5],
	  full_buff_usage_i,
          buff_new_tgt[5],
          buff_new_usage_count,
          buff_new_colour,
          buff_now_full[5],
          buff_content_consumed[5],
          buff_usage_count[5],
          buffers_colour_o[5],
          buffers_free_o[5],
          buffers_full_o[5]
         );

buffer_state_entry buffer6
         (clk,
          reset,
	  mark_as_full[6],
	  full_buff_usage_i,
          buff_new_tgt[6],
          buff_new_usage_count,
          buff_new_colour,
          buff_now_full[6],
          buff_content_consumed[6],
          buff_usage_count[6],
          buffers_colour_o[6],
          buffers_free_o[6],
          buffers_full_o[6]
         );

buffer_state_entry buffer7
         (clk,
          reset,
	  mark_as_full[7],
	  full_buff_usage_i,
          buff_new_tgt[7],
          buff_new_usage_count,
          buff_new_colour,
          buff_now_full[7],
          buff_content_consumed[7],
          buff_usage_count[7],
          buffers_colour_o[7],
          buffers_free_o[7],
          buffers_full_o[7]
         );

buffer_state_entry buffer8
         (clk,
          reset,
	  mark_as_full[8],
	  full_buff_usage_i,
          buff_new_tgt[8],
          buff_new_usage_count,
          buff_new_colour,
          buff_now_full[8],
          buff_content_consumed[8],
          buff_usage_count[8],
          buffers_colour_o[8],
          buffers_free_o[8],
          buffers_full_o[8]
         );

buffer_state_entry buffer9
         (clk,
          reset,
	  mark_as_full[9],
	  full_buff_usage_i,
          buff_new_tgt[9],
          buff_new_usage_count,
          buff_new_colour,
          buff_now_full[9],
          buff_content_consumed[9],
          buff_usage_count[9],
          buffers_colour_o[9],
          buffers_free_o[9],
          buffers_full_o[9]
         );

buffer_state_entry buffer10
         (clk,
          reset,
	  mark_as_full[10],
	  full_buff_usage_i,
          buff_new_tgt[10],
          buff_new_usage_count,
          buff_new_colour,
          buff_now_full[10],
          buff_content_consumed[10],
          buff_usage_count[10],
          buffers_colour_o[10],
          buffers_free_o[10],
          buffers_full_o[10]
         );

buffer_state_entry buffer11
         (clk,
          reset,
	  mark_as_full[11],
	  full_buff_usage_i,
          buff_new_tgt[11],
          buff_new_usage_count,
          buff_new_colour,
          buff_now_full[11],
          buff_content_consumed[11],
          buff_usage_count[11],
          buffers_colour_o[11],
          buffers_free_o[11],
          buffers_full_o[11]
         );

buffer_state_entry buffer12
         (clk,
          reset,
	  mark_as_full[12],
	  full_buff_usage_i,
          buff_new_tgt[12],
          buff_new_usage_count,
          buff_new_colour,
          buff_now_full[12],
          buff_content_consumed[12],
          buff_usage_count[12],
          buffers_colour_o[12],
          buffers_free_o[12],
          buffers_full_o[12]
         );

buffer_state_entry buffer13
         (clk,
          reset,
	  mark_as_full[13],
	  full_buff_usage_i,
          buff_new_tgt[13],
          buff_new_usage_count,
          buff_new_colour,
          buff_now_full[13],
          buff_content_consumed[13],
          buff_usage_count[13],
          buffers_colour_o[13],
          buffers_free_o[13],
          buffers_full_o[13]
         );

buffer_state_entry buffer14
         (clk,
          reset,
	  mark_as_full[14],
	  full_buff_usage_i,
          buff_new_tgt[14],
          buff_new_usage_count,
          buff_new_colour,
          buff_now_full[14],
          buff_content_consumed[14],
          buff_usage_count[14],
          buffers_colour_o[14],
          buffers_free_o[14],
          buffers_full_o[14]
         );

buffer_state_entry buffer15
         (clk,
          reset,
	  mark_as_full[15],
	  full_buff_usage_i,
          buff_new_tgt[15],
          buff_new_usage_count,
          buff_new_colour,
          buff_now_full[15],
          buff_content_consumed[15],
          buff_usage_count[15],
          buffers_colour_o[15],
          buffers_free_o[15],
          buffers_full_o[15]
         );

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

