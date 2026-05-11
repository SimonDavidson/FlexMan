// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
module sch_entry  #(parameter SCH_ENTRY_SZ        = 32,
	            parameter NUM_HW_ACCELERATORS = 2,
	            parameter TGT_ACC_SZ          = 2,
		    parameter NUM_BUFFERS         = 16,
                    parameter COL_BUFF_ID_SZ      = 16,
                    parameter ACC_ID_BTM          = 0
	    )
             (input  wire                      clk,
              input  wire                      reset,
              input  wire                      load_new_entry_i,
              input  wire                      shift_entry_i,
              input  wire                      delete_entry_i,
	      input  wire                      new_entry_valid_i,
              input  wire [SCH_ENTRY_SZ-1:0]   new_entry_data_i,
	      input  wire                      shift_in_entry_valid_i,
              input  wire [SCH_ENTRY_SZ-1:0]   shift_in_entry_data_i,
	      output wire                      shift_out_entry_valid_o,
	      output reg                       entry_valid_o,
              output reg  [SCH_ENTRY_SZ-1:0]   entry_data_o,

              input  wire [NUM_HW_ACCELERATORS-1:0] acc_busy_i,
	      input  wire [COL_BUFF_ID_SZ-1:0] buffers_full_i,
	      input  wire [COL_BUFF_ID_SZ-1:0] buffers_free_i,
	      output wire                      ready_to_execute_o
              );

localparam TGT_ACC_BTM = 0;
localparam TGT_ACC_TOP = TGT_ACC_SZ   - 1;
localparam INBUFFS_BTM = TGT_ACC_TOP  + 1;
localparam INBUFFS_TOP = INBUFFS_BTM  + NUM_BUFFERS - 1;
localparam OUTBUFF_BTM = INBUFFS_TOP  + 1;
localparam OUTBUFF_TOP = OUTBUFF_BTM  + NUM_BUFFERS - 1;
localparam BUFF_INDX_SZ         = $clog2(NUM_BUFFERS);
localparam COLOUR_SZ            = 1;
localparam NUM_TGTS_SZ          = 3;
localparam ENTRY_SBUFF1_START   = 0;
localparam ENTRY_SBUFF1_END     = ENTRY_SBUFF1_START + BUFF_INDX_SZ - 1;
localparam ENTRY_SBUFF2_START   = ENTRY_SBUFF1_END   + 1;
localparam ENTRY_SBUFF2_END     = ENTRY_SBUFF2_START + BUFF_INDX_SZ - 1;
localparam ENTRY_SBUFF3_START   = ENTRY_SBUFF2_END   + 1;
localparam ENTRY_SBUFF3_END     = ENTRY_SBUFF3_START + BUFF_INDX_SZ - 1;
localparam ENTRY_TBUFF_START    = ENTRY_SBUFF3_END   + 1;
localparam ENTRY_TBUFF_END      = ENTRY_TBUFF_START  + BUFF_INDX_SZ - 1;


wire entry_valid_nxt;
//reg  entry_valid_o;
//reg  [SCH_ENTRY_SZ-1:0] entry_data_o;
wire [SCH_ENTRY_SZ-1:0] updated_entry_data;
wire [SCH_ENTRY_SZ-1:0] entry_data_nxt;
wire [SCH_ENTRY_SZ-1:0] inbuffers_needed;
wire [SCH_ENTRY_SZ-1:0] waiting_for_inbuffer;
wire [NUM_BUFFERS-1:0]  outbuffers_needed;
wire [SCH_ENTRY_SZ-1:0] waiting_for_outbuffer;
wire                    got_all_inbuffers;
wire                    got_all_outbuffers;
wire                    acc_free;
wire [NUM_HW_ACCELERATORS-1:0] required_acc;
wire [NUM_BUFFERS-1:0]  src1_buff_1hot;
wire [NUM_BUFFERS-1:0]  src2_buff_1hot;
wire [NUM_BUFFERS-1:0]   tgt_buff_1hot;

assign src1_buff_1hot = 1 << entry_data_o[ENTRY_SBUFF1_END:ENTRY_SBUFF1_START];
assign src2_buff_1hot = 1 << entry_data_o[ENTRY_SBUFF2_END:ENTRY_SBUFF2_START];
assign  tgt_buff_1hot = 1 << entry_data_o[ENTRY_TBUFF_END:ENTRY_TBUFF_START];

// The valid bit for this entry on the assumption that it is
// being shifted forward. This is identical to the entry's valid bit,
// except when this entry is being deleted:
assign shift_out_entry_valid_o = delete_entry_i ? 1'b0 : entry_valid_o;

assign entry_valid_nxt = load_new_entry_i ? 1'b1                   : 
	                 delete_entry_i   ? 1'b0                   :
	                 shift_entry_i    ? shift_in_entry_valid_i :
		                            entry_valid_o;

assign entry_data_nxt = load_new_entry_i  ? new_entry_data_i       :
	                 delete_entry_i   ? entry_data_o           :
                         shift_entry_i    ? shift_in_entry_data_i  :
                                            entry_data_o;

assign updated_entry_data = entry_data_o;

always @ (posedge clk)
if (reset)
  begin
     entry_valid_o <= 1'b0;
     entry_data_o  <= 'b0;
  end
else
  begin
     entry_valid_o <= entry_valid_nxt;
     entry_data_o  <= entry_data_nxt;
  end

//////////////////////////////////////////////
// Check if input buffers are all available:

assign inbuffers_needed      = src1_buff_1hot | src2_buff_1hot;
assign waiting_for_inbuffer  = inbuffers_needed & ~buffers_full_i;
assign got_all_inbuffers     = ~|waiting_for_inbuffer;

//////////////////////////////////////////////
// Check if output buffers are all available:

assign outbuffers_needed     = tgt_buff_1hot;
assign waiting_for_outbuffer = outbuffers_needed & ~buffers_free_i;
assign got_all_outbuffers    = ~|waiting_for_outbuffer;

//////////////////////////////////////////////
// Check if target accelerator is available:
//
assign required_acc = 1'b1<< entry_data_o[ACC_ID_BTM+TGT_ACC_SZ-1:ACC_ID_BTM];
assign acc_free     = &(~required_acc | ~acc_busy_i);

//////////////////////////////////////////////
// Flag if this entry is ready to execute:
//
assign ready_to_execute_o = entry_valid_o & got_all_inbuffers & 
	                    acc_free      & got_all_outbuffers;
//
//assign ready_to_execute_o = 1'b0; //entry_valid_o;

endmodule	// sch_entry

