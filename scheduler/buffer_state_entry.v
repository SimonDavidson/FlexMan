// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
////////////////////////////////////////////////////////////////////////
//
// buffer_state_entry
//
// Author: Simon D.
// Version: 1.0
// Date 10/2/2025
//
// State for one scheduler buffer entry
//

module buffer_state_entry
    #(parameter TGT_COUNT_SZ = 3)
    (input wire                     clk,
     input wire                     reset,

     // Mark this buffer as already full:
     input wire                     mark_as_full_i,
     input wire [TGT_COUNT_SZ-1:0]  mark_buff_usage_i,

     // Initialise buffer entry (new target buffer):
     input wire                     buff_new_tgt_i,
     input wire [TGT_COUNT_SZ-1:0]  buff_new_usage_count_i,
     input wire                     buff_new_colour_i,

     // Signal that the task filling this buffer is complete
     // so buffer can now be consumed by other tasks:
     input wire                     buff_now_full_i,

     // Signal that one of that consumers of this buffer 
     // has finished with it:
     input wire                     buff_content_consumed_i,

     // Current status of this buffer:
     output wire [TGT_COUNT_SZ-1:0] buff_usage_count_o, // Debug use only
     output wire                    buff_colour_o,
     output wire                    buff_free_o,
     output wire                    buff_full_o
    );

// The state being held for each buffer:
reg                          buff_free_r;
reg                          buff_full_r;
reg                          buff_colour_r;
reg  [TGT_COUNT_SZ-1:0]      buff_usage_count_r;
wire [TGT_COUNT_SZ-1:0]      buff_usage_count_nxt;

wire buff_no_longer_needed;

assign buff_no_longer_needed = (buff_usage_count_r == 'b1) 
                              ? buff_content_consumed_i 
             	              : 0;

assign buff_usage_count_nxt = buff_usage_count_r - 'b1;

always @ (posedge clk)
begin
   if (reset)
      buff_free_r <= 1'b1;
   else if (mark_as_full_i)
      buff_free_r <= 1'b0;
   else if (buff_no_longer_needed)
      // Last consumer has finished. Buffer can be freed:
      buff_free_r <= 1'b1;
   else if (buff_new_tgt_i)
      // Buff is a target and will be filled:
      buff_free_r <= 1'b0;
end

always @ (posedge clk)
begin
   if (reset)
      buff_full_r <= 1'b0;
   else if (buff_now_full_i | mark_as_full_i)
      buff_full_r <= 1'b1;
   else if (buff_no_longer_needed)
      // Buffer was in use but the last consumer has finished so invalidate it:
      buff_full_r <= 1'b0;
end

always @ (posedge clk)
begin
   if (reset)
      buff_colour_r   <= 1'b0;
   else if (buff_new_tgt_i)
	   buff_colour_r <= buff_new_colour_i;
end

always @ (posedge clk)
begin
   if (reset)
      buff_usage_count_r <= 1'b0;
   else if (mark_as_full_i)
      buff_usage_count_r <= mark_buff_usage_i;
   else if (buff_new_tgt_i)
      buff_usage_count_r <= buff_new_usage_count_i;
   else if (buff_content_consumed_i)
      buff_usage_count_r <= buff_usage_count_nxt;
end

assign buff_colour_o      = buff_colour_r;
assign buff_free_o        = buff_free_r;
assign buff_full_o        = buff_full_r;
assign buff_usage_count_o = buff_usage_count_r;

endmodule

