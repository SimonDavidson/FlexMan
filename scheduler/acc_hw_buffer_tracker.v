// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
////////////////////////////////////////////////////////////////////////
//
// acc_hw_buffer_tracker
//
// Author: Simon D.
// Version: 1.0
// Date 10/2/2025
//
// State for one hardware compute unit, to track what buffers it is using
//

module acc_hw_buffer_tracker
    #(parameter BUFF_INDX_SZ = 4)
    (input wire                      clk,
     input wire                      reset,

     input wire                     new_task_i,
     input wire [BUFF_INDX_SZ-1:0]  tgt_buff_i,
     input wire [BUFF_INDX_SZ-1:0]  src1_buff_i,
     input wire [BUFF_INDX_SZ-1:0]  src2_buff_i,
     input wire [BUFF_INDX_SZ-1:0]  src3_buff_i,

     input wire                     task_finished_i,

     output wire                    acc_free_o,
     output wire [BUFF_INDX_SZ-1:0] tgt_buff_o,
     output wire [BUFF_INDX_SZ-1:0] src1_buff_o,
     output wire [BUFF_INDX_SZ-1:0] src2_buff_o,
     output wire [BUFF_INDX_SZ-1:0] src3_buff_o
    );

reg [BUFF_INDX_SZ-1:0]  tgt_buff_r;
reg [BUFF_INDX_SZ-1:0] src1_buff_r;
reg [BUFF_INDX_SZ-1:0] src2_buff_r;
reg [BUFF_INDX_SZ-1:0] src3_buff_r;
reg                   acc_free_r;
wire                  acc_free_nxt;

// The buffers implicated in this HW accelerator's task:
always @ (posedge clk)
begin
   if (reset)
      begin
          tgt_buff_r <= 'b0;
         src1_buff_r <= 'b0;
         src2_buff_r <= 'b0;
         src3_buff_r <= 'b0;
      end
   else if (new_task_i)
      begin
          tgt_buff_r <= tgt_buff_i;
         src1_buff_r <= src1_buff_i;
         src2_buff_r <= src2_buff_i;
         src3_buff_r <= src3_buff_i;
      end

end

// Busy signal for the accelerator itself:

assign acc_free_nxt =   (new_task_i)      ? 1'b0
                      : (task_finished_i) ? 1'b1
                      :                     acc_free_r;

always @ (posedge clk)
begin
   if (reset)
      acc_free_r = 1'b1;
   else
      acc_free_r = acc_free_nxt;
end

assign tgt_buff_o  =  tgt_buff_r;
assign src1_buff_o = src1_buff_r;
assign src2_buff_o = src2_buff_r;
assign src3_buff_o = src3_buff_r;
assign acc_free_o  =  acc_free_r;

endmodule

