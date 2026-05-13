// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
////////////////////////////////////////////////////////////////////////
//
// buffer_state_entry
//
// Author: Simon D.
// Version: 2.0
// Date 10/2/2025
//
// State for one scheduler buffer entry.
// Supports three dispatch modes driven by sch_buffer_state:
//   buff_new_tgt_i   — target-only: clear free, full stays 0
//   buff_rw_claim_i  — read-write:  clear both free and full (full→busy)
//   buff_now_full_i  — completion:  set full with new usage count
//

module buffer_state_entry
    #(parameter TGT_COUNT_SZ = 3)
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

     // One consumer of this buffer has been dispatched:
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

assign buff_no_longer_needed = (buff_usage_count_r == 'b1) & buff_content_consumed_i;

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
   else if (buff_now_full_i | mark_as_full_i)
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
   else if (buff_now_full_i)
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
