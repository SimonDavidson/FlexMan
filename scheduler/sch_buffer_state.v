// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
////////////////////////////////////////////////////////////////////////
//
// sch_buffer_state
//
// Author: Simon D.
// Version: 2.0
//
// Tracks buffer and accelerator state for the scheduler.
//
// Each TASK carries NUM_SLOTS buffer slots, each with a 2-bit mode:
//   2'b00 unused  — skip
//   2'b01 source  — consume (decrement usage count) on dispatch
//   2'b10 rw      — claim (free=0, full=0) on dispatch; refill on completion
//   2'b11 target  — reserve (free=0) on dispatch; fill on completion
//
// On completion, slots with mode 10 or 11 are marked full with the
// stored #targets count from the completing task.
//

`define MODE_UNUSED 2'b00
`define MODE_SRC    2'b01
`define MODE_RW     2'b10
`define MODE_TGT    2'b11

module sch_buffer_state
      #(parameter NUM_BUFFERS         = 16,
        parameter BUFF_INDX_SZ        = $clog2(NUM_BUFFERS),
        parameter NUM_HW_ACCELERATORS = 2,
        parameter TGT_ACC_SZ          = $clog2(NUM_HW_ACCELERATORS),
        parameter TGT_COUNT_SZ        = 3,
        parameter NUM_SLOTS           = 6,
        parameter MODE_SZ             = 2
       )
       (input  wire                    clk,
        input  wire                    reset,

        // Process info from hardware accelerators:
        input wire [NUM_HW_ACCELERATORS-1:0] acc_busy_i,
        input wire [NUM_HW_ACCELERATORS-1:0] acc_finished_i,
        input wire [NUM_HW_ACCELERATORS-1:0] acc_result_i,

        // External buffer pre-fill:
        input wire                     mark_buff_as_full_i,
        input wire  [BUFF_INDX_SZ-1:0] full_buff_id_i,
        input wire  [TGT_COUNT_SZ-1:0] full_buff_usage_i,

        // Dispatch interface from scheduler:
        input wire                                start_new_task_i,
        input wire  [TGT_ACC_SZ-1:0]              tgt_acc_id_i,
        input wire  [NUM_SLOTS*BUFF_INDX_SZ-1:0]  slot_buff_i,
        input wire  [NUM_SLOTS*MODE_SZ-1:0]        slot_mode_i,
        input wire  [NUM_SLOTS*TGT_COUNT_SZ-1:0]   slot_ntgt_i,
        input wire                                 tgt_colour_i,

        // Broadcast current accelerator status:
        output wire [NUM_HW_ACCELERATORS-1:0] acc_available_o,

        // Broadcast current buffer status:
        output wire [NUM_BUFFERS-1:0]  buffers_full_o,
        output wire [NUM_BUFFERS-1:0]  buffers_free_o,
        output wire [NUM_BUFFERS-1:0]  buffers_colour_o,
        output wire [NUM_BUFFERS-1:0]  target_status_o
        );

// Per-accelerator tracker outputs:
wire [NUM_HW_ACCELERATORS-1:0]             acc_free;
wire [NUM_SLOTS*BUFF_INDX_SZ-1:0] tr_slot_buff [0:NUM_HW_ACCELERATORS-1];
wire [NUM_SLOTS*MODE_SZ-1:0]       tr_slot_mode [0:NUM_HW_ACCELERATORS-1];
wire [NUM_SLOTS*TGT_COUNT_SZ-1:0]  tr_slot_ntgt [0:NUM_HW_ACCELERATORS-1];

// Pending-completion queue (one-hot, one bit per accelerator):
reg  [NUM_HW_ACCELERATORS-1:0] acc_process_pending_r;
wire [NUM_HW_ACCELERATORS-1:0] acc_process_pending_nxt;

// Selected finisher (combinatorial):
reg  [NUM_HW_ACCELERATORS-1:0] selected_finisher;
reg  [TGT_ACC_SZ-1:0]          selected_acc_idx;
wire                            post_task_cleanup;

// One-hot buffer action strobes (combinatorial):
reg  [NUM_BUFFERS-1:0] buff_src_consumed;
reg  [NUM_BUFFERS-1:0] buff_rw_claim;
reg  [NUM_BUFFERS-1:0] buff_new_tgt;
reg  [NUM_BUFFERS-1:0] buff_now_full;

// Per-buffer completion data (driven from selected finisher's tracker):
reg  [TGT_COUNT_SZ-1:0] buff_now_ntgt   [0:NUM_BUFFERS-1];

// External mark-as-full one-hot:
reg  [NUM_BUFFERS-1:0] mark_as_full;

// ACC ID one-hot for new task dispatch:
reg [NUM_HW_ACCELERATORS-1:0] new_task_acc;

integer s, c, bi_idx;
reg found;

//-----------------------------------------------------------------
// Pending-completion queue
//-----------------------------------------------------------------
assign acc_process_pending_nxt = (acc_process_pending_r | acc_finished_i)
                               & ~selected_finisher;

always @ (posedge clk)
begin
   if (reset)
      acc_process_pending_r <= 'b0;
   else
      acc_process_pending_r <= acc_process_pending_nxt;
end

//-----------------------------------------------------------------
// Priority-encode the lowest-index pending finisher
//-----------------------------------------------------------------
always @*
begin
   selected_finisher = 'b0;
   selected_acc_idx  = 'b0;
   found             = 1'b0;
   for (s = 0; s < NUM_HW_ACCELERATORS; s = s + 1)
      if (!found && acc_process_pending_r[s])
      begin
         selected_finisher[s] = 1'b1;
         selected_acc_idx     = s[TGT_ACC_SZ-1:0];
         found                = 1'b1;
      end
end

assign post_task_cleanup = |selected_finisher;

//-----------------------------------------------------------------
// External mark-as-full
//-----------------------------------------------------------------
always @ (full_buff_id_i, mark_buff_as_full_i)
begin
   mark_as_full = 'b0;
   if (mark_buff_as_full_i)
      mark_as_full[full_buff_id_i] = 1'b1;
end

//-----------------------------------------------------------------
// Dispatch: decode slot modes and assert buffer action strobes
//-----------------------------------------------------------------
always @*
begin
   buff_src_consumed = 'b0;
   buff_rw_claim     = 'b0;
   buff_new_tgt      = 'b0;
   new_task_acc      = 'b0;
   if (start_new_task_i)
   begin
      new_task_acc[tgt_acc_id_i] = 1'b1;
      for (s = 0; s < NUM_SLOTS; s = s + 1)
      begin : dispatch_slot
         reg [BUFF_INDX_SZ-1:0] bid;
         reg [MODE_SZ-1:0]      md;
         bid = slot_buff_i[s*BUFF_INDX_SZ +: BUFF_INDX_SZ];
         md  = slot_mode_i[s*MODE_SZ      +: MODE_SZ];
         case (md)
            `MODE_SRC: buff_src_consumed[bid] = 1'b1;
            `MODE_RW:  buff_rw_claim[bid]     = 1'b1;
            `MODE_TGT: buff_new_tgt[bid]      = 1'b1;
            default:   ;
         endcase
      end
   end
end

//-----------------------------------------------------------------
// Completion: mark output slots full with stored #targets
//-----------------------------------------------------------------
always @*
begin
   buff_now_full = 'b0;
   for (bi_idx = 0; bi_idx < NUM_BUFFERS; bi_idx = bi_idx + 1)
      buff_now_ntgt[bi_idx] = 'b0;

   if (post_task_cleanup)
   begin
      for (s = 0; s < NUM_SLOTS; s = s + 1)
      begin : completion_slot
         reg [BUFF_INDX_SZ-1:0] bid;
         reg [MODE_SZ-1:0]      md;
         bid = tr_slot_buff[selected_acc_idx][s*BUFF_INDX_SZ +: BUFF_INDX_SZ];
         md  = tr_slot_mode[selected_acc_idx][s*MODE_SZ      +: MODE_SZ];
         if (md == `MODE_RW || md == `MODE_TGT)
         begin
            buff_now_full[bid]   = 1'b1;
            buff_now_ntgt[bid]   =
               tr_slot_ntgt[selected_acc_idx][s*TGT_COUNT_SZ +: TGT_COUNT_SZ];
         end
      end
   end
end

//-----------------------------------------------------------------
// Instantiate per-accelerator buffer trackers
//-----------------------------------------------------------------
genvar ai;
generate
   for (ai = 0; ai < NUM_HW_ACCELERATORS; ai = ai + 1) begin : gen_acc
      acc_hw_buffer_tracker #(
         .NUM_SLOTS(NUM_SLOTS),
         .BUFF_INDX_SZ(BUFF_INDX_SZ),
         .TGT_COUNT_SZ(TGT_COUNT_SZ),
         .MODE_SZ(MODE_SZ)
      ) hw_n (
         .clk(clk),
         .reset(reset),
         .new_task_i(new_task_acc[ai]),
         .slot_buff_i(slot_buff_i),
         .slot_mode_i(slot_mode_i),
         .slot_ntgt_i(slot_ntgt_i),
         .task_finished_i(selected_finisher[ai]),
         .acc_free_o(acc_free[ai]),
         .slot_buff_o(tr_slot_buff[ai]),
         .slot_mode_o(tr_slot_mode[ai]),
         .slot_ntgt_o(tr_slot_ntgt[ai])
      );
   end
endgenerate

assign acc_available_o = acc_free;

//-----------------------------------------------------------------
// Instantiate per-buffer state entries
//-----------------------------------------------------------------
genvar bi;
generate
   for (bi = 0; bi < NUM_BUFFERS; bi = bi + 1) begin : gen_buf
      buffer_state_entry #(.TGT_COUNT_SZ(TGT_COUNT_SZ)) buffer_n (
         .clk(clk),
         .reset(reset),
         .mark_as_full_i(mark_as_full[bi]),
         .mark_buff_usage_i(full_buff_usage_i),
         .buff_new_tgt_i(buff_new_tgt[bi]),
         .buff_new_usage_count_i({TGT_COUNT_SZ{1'b0}}),
         .buff_new_colour_i(tgt_colour_i),
         .buff_rw_claim_i(buff_rw_claim[bi]),
         .buff_rw_colour_i(tgt_colour_i),
         .buff_now_full_i(buff_now_full[bi]),
         .buff_now_usage_count_i(buff_now_ntgt[bi]),
         .buff_content_consumed_i(buff_src_consumed[bi]),
         .buff_usage_count_o(),
         .buff_colour_o(buffers_colour_o[bi]),
         .buff_free_o(buffers_free_o[bi]),
         .buff_full_o(buffers_full_o[bi])
      );
   end
endgenerate

//-----------------------------------------------------------------
// Per-buffer task result register file (used by CHECK instruction)
//-----------------------------------------------------------------
reg [NUM_BUFFERS-1:0] target_status_r;

always @ (posedge clk)
begin
   if (reset)
      target_status_r <= 'b0;
   else if (post_task_cleanup)
   begin
      for (s = 0; s < NUM_SLOTS; s = s + 1)
      begin : result_slot
         reg [BUFF_INDX_SZ-1:0] bid;
         reg [MODE_SZ-1:0]      md;
         bid = tr_slot_buff[selected_acc_idx][s*BUFF_INDX_SZ +: BUFF_INDX_SZ];
         md  = tr_slot_mode[selected_acc_idx][s*MODE_SZ      +: MODE_SZ];
         if (md == `MODE_TGT || md == `MODE_RW)
            target_status_r[bid] <= acc_result_i[selected_acc_idx];
      end
   end
end

assign target_status_o = target_status_r;

endmodule // sch_buffer_state
