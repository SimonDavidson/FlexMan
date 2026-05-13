// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
////////////////////////////////////////////////////////////////////////
//
// acc_hw_buffer_tracker
//
// Author: Simon D.
// Version: 2.0
// Date 10/2/2025
//
// Tracks which buffers are in use by one hardware accelerator.
// Stores six slots (mode + buffer ID + #targets) so sch_buffer_state
// can correctly update buffer state on task completion.
//
// Slot modes:
//   2'b00 unused  — ignored on completion
//   2'b01 source  — consumed on dispatch, no action on completion
//   2'b10 rw      — claimed on dispatch, refilled on completion
//   2'b11 target  — reserved on dispatch, filled on completion
//

module acc_hw_buffer_tracker
    #(parameter NUM_SLOTS    = 6,
      parameter BUFF_INDX_SZ = 4,
      parameter TGT_COUNT_SZ = 3,
      parameter MODE_SZ      = 2)
    (input wire                                   clk,
     input wire                                   reset,

     input wire                                   new_task_i,
     input wire [NUM_SLOTS*BUFF_INDX_SZ-1:0]     slot_buff_i,
     input wire [NUM_SLOTS*MODE_SZ-1:0]           slot_mode_i,
     input wire [NUM_SLOTS*TGT_COUNT_SZ-1:0]      slot_ntgt_i,

     input wire                                   task_finished_i,

     output wire                                  acc_free_o,
     output wire [NUM_SLOTS*BUFF_INDX_SZ-1:0]    slot_buff_o,
     output wire [NUM_SLOTS*MODE_SZ-1:0]          slot_mode_o,
     output wire [NUM_SLOTS*TGT_COUNT_SZ-1:0]     slot_ntgt_o
    );

reg [NUM_SLOTS*BUFF_INDX_SZ-1:0]  slot_buff_r;
reg [NUM_SLOTS*MODE_SZ-1:0]        slot_mode_r;
reg [NUM_SLOTS*TGT_COUNT_SZ-1:0]   slot_ntgt_r;
reg                                 acc_free_r;

always @ (posedge clk)
begin
   if (reset)
   begin
      slot_buff_r <= 'b0;
      slot_mode_r <= 'b0;
      slot_ntgt_r <= 'b0;
   end
   else if (new_task_i)
   begin
      slot_buff_r <= slot_buff_i;
      slot_mode_r <= slot_mode_i;
      slot_ntgt_r <= slot_ntgt_i;
   end
end

assign acc_free_o   = acc_free_r;
assign slot_buff_o  = slot_buff_r;
assign slot_mode_o  = slot_mode_r;
assign slot_ntgt_o  = slot_ntgt_r;

wire acc_free_nxt = new_task_i      ? 1'b0
                  : task_finished_i ? 1'b1
                  :                   acc_free_r;

always @ (posedge clk)
begin
   if (reset)
      acc_free_r <= 1'b1;
   else
      acc_free_r <= acc_free_nxt;
end

endmodule
