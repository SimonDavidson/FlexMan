// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
////////////////////////////////////////////////////////////////////////
//
// buffer_state_entry
//
// Author: Simon D.
// Version: 2.1
// Date 10/2/2025
// Authors: Simon Davidson & Claude   Last modified: 2026-08-22
//
// State for one scheduler buffer entry.
// Supports three dispatch modes driven by sch_buffer_state:
//   buff_new_tgt_i   — target-only: clear free, full stays 0
//   buff_rw_claim_i  — read-write:  clear both free and full (full→busy)
//   buff_now_full_i  — completion:  set full with new usage count
//
// MULTI_WRITER (2026-08-22, Simon Davidson & Claude)
// --------------------------------------------------
// Default 0 = exactly the behaviour above; `set_full` is literally
// buff_now_full_i, so every existing build is bit-identical.
//
// MULTI_WRITER=1 counts OUTSTANDING WRITERS and only marks the buffer full on
// the completion that takes the count to zero. Needed once an accelerator may
// hold two tasks in flight (see acc_hw_buffer_tracker's DEPTH and sch_entry's
// disjoint hint): the successor's claim and the predecessor's refill land on
// DIFFERENT cycles, so without the count —
//   1. successor dispatches -> its RW claim clears full
//   2. predecessor completes -> full goes back to 1
//   3. successor is still writing, but the buffer already reads as full, so a
//      downstream consumer dispatches and reads a PARTIALLY WRITTEN buffer.
// The count also fixes which usage count lands: the LAST writer's, which is the
// one the schedule means (Monarch's fc1.b3 carries ntgt=6 while b0..b2 carry 1).
//
// The count can only exceed 1 via a hint-bypassed early dispatch, so enabling it
// changes nothing on its own.
//

module buffer_state_entry
    #(parameter TGT_COUNT_SZ = 3,
      // 1 = allow >1 outstanding writer; full asserts only when the last retires.
      parameter MULTI_WRITER = 0)
    (input wire                     clk,
     input wire                     reset,

     // Mark this buffer as already full (external pre-fill):
     input wire                     mark_as_full_i,
     input wire [TGT_COUNT_SZ-1:0]  mark_buff_usage_i,

     // New target-only buffer dispatched: clear free, full stays 0:
     input wire                     buff_new_tgt_i,
     input wire [TGT_COUNT_SZ-1:0]  buff_new_usage_count_i,
     input wire                     buff_new_colour_i,

     // RW buffer claimed on dispatch: clear both free and full:
     input wire                     buff_rw_claim_i,
     input wire                     buff_rw_colour_i,

     // Task writing this buffer has completed: set full, load new count:
     input wire                     buff_now_full_i,
     input wire [TGT_COUNT_SZ-1:0]  buff_now_usage_count_i,

     // One consumer of this buffer has completed:
     input wire                     buff_content_consumed_i,

     // Current status of this buffer:
     output wire [TGT_COUNT_SZ-1:0] buff_usage_count_o, // debug
     output wire                    buff_colour_o,
     output wire                    buff_free_o,
     output wire                    buff_full_o
    );

reg                     buff_free_r;
reg                     buff_full_r;
reg                     buff_colour_r;
reg  [TGT_COUNT_SZ-1:0] buff_usage_count_r;

wire buff_no_longer_needed;
wire set_full;

assign buff_no_longer_needed = (buff_usage_count_r == 'b1) & buff_content_consumed_i;

//-----------------------------------------------------------------
// Completion -> "this buffer is now full"
//-----------------------------------------------------------------
generate
if (MULTI_WRITER == 0) begin : gen_single_writer
   // Original: one writer at a time, so every completion fills the buffer.
   assign set_full = buff_now_full_i;
end else begin : gen_multi_writer
   // Two tasks may be writing disjoint sub-ranges at once.  A simultaneous
   // start+finish leaves the count unchanged — a writer arrived as one left, so
   // the buffer is still being written and must NOT go full.
   reg  [1:0] writers_r;
   wire       wr_start     = buff_rw_claim_i | buff_new_tgt_i;
   wire [1:0] writers_nxt  = writers_r + {1'b0, wr_start}
                                       - {1'b0, buff_now_full_i};

   always @ (posedge clk)
      if (reset) writers_r <= 2'd0;
      else       writers_r <= writers_nxt;

   assign set_full = buff_now_full_i & (writers_nxt == 2'd0);
end
endgenerate

always @ (posedge clk)
begin
   if (reset)
      buff_free_r <= 1'b1;
   else if (buff_rw_claim_i)
      buff_free_r <= 1'b0;
   else if (mark_as_full_i)
      buff_free_r <= 1'b0;
   else if (buff_no_longer_needed)
      buff_free_r <= 1'b1;
   else if (buff_new_tgt_i)
      buff_free_r <= 1'b0;
end

always @ (posedge clk)
begin
   if (reset)
      buff_full_r <= 1'b0;
   else if (buff_rw_claim_i)
      // RW claim: buffer goes from full → busy
      buff_full_r <= 1'b0;
   else if (set_full | mark_as_full_i)
      buff_full_r <= 1'b1;
   else if (buff_no_longer_needed)
      buff_full_r <= 1'b0;
end

always @ (posedge clk)
begin
   if (reset)
      buff_colour_r <= 1'b0;
   else if (buff_rw_claim_i)
      buff_colour_r <= buff_rw_colour_i;
   else if (buff_new_tgt_i)
      buff_colour_r <= buff_new_colour_i;
end

always @ (posedge clk)
begin
   if (reset)
      buff_usage_count_r <= 'b0;
   else if (mark_as_full_i)
      buff_usage_count_r <= mark_buff_usage_i;
   else if (set_full)
      buff_usage_count_r <= buff_now_usage_count_i;
   else if (buff_new_tgt_i)
      buff_usage_count_r <= buff_new_usage_count_i;
   else if (buff_content_consumed_i)
      buff_usage_count_r <= buff_usage_count_r - 'b1;
end

assign buff_colour_o      = buff_colour_r;
assign buff_free_o        = buff_free_r;
assign buff_full_o        = buff_full_r;
assign buff_usage_count_o = buff_usage_count_r;

endmodule
