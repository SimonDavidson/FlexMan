// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
module sch_entry  #(parameter SCH_ENTRY_SZ        = 52,
                    parameter NUM_HW_ACCELERATORS = 2,
                    parameter TGT_ACC_SZ          = 1,   // $clog2(NUM_HW_ACCELERATORS)
                    parameter NUM_BUFFERS         = 16,
                    parameter BUFF_INDX_SZ        = 4,   // $clog2(NUM_BUFFERS)
                    parameter NUM_SLOTS           = 6,
                    parameter MODE_SZ             = 2,
                    parameter TGT_COUNT_SZ        = 3,
                    parameter ACC_ID_BTM          = 0    // kept for sch_table compat
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
              input  wire [NUM_BUFFERS-1:0]    buffers_full_i,
              input  wire [NUM_BUFFERS-1:0]    buffers_free_i,
              input  wire [NUM_BUFFERS-1:0]    buffers_colour_i,
              output wire                      ready_to_execute_o
              );

// Slot layout within SCH_ENTRY_SZ (lsb first):
//   Slots 0-2 (short): [mode(2), id(BUFF_INDX_SZ)]
//   Slots 3-5 (long):  [mode(2), id(BUFF_INDX_SZ), ntgt(TGT_COUNT_SZ)]
//   colour(1), acc_id(TGT_ACC_SZ), cfg_id(5)
localparam SLOT_SHORT_SZ = MODE_SZ + BUFF_INDX_SZ;
localparam SLOT_LONG_SZ  = MODE_SZ + BUFF_INDX_SZ + TGT_COUNT_SZ;
localparam SLOTS_SHORT   = 3;
localparam SLOTS_LONG    = 3;
localparam COLOUR_START  = SLOTS_SHORT*SLOT_SHORT_SZ + SLOTS_LONG*SLOT_LONG_SZ;
localparam ACC_ID_START  = COLOUR_START + 1;

`define MODE_UNUSED 2'b00
`define MODE_SRC    2'b01
`define MODE_RW     2'b10
`define MODE_TGT    2'b11

wire entry_valid_nxt;
wire [SCH_ENTRY_SZ-1:0] entry_data_nxt;

assign shift_out_entry_valid_o = delete_entry_i ? 1'b0 : entry_valid_o;

assign entry_valid_nxt = load_new_entry_i ? 1'b1
                       : delete_entry_i   ? 1'b0
                       : shift_entry_i    ? shift_in_entry_valid_i
                       :                    entry_valid_o;

assign entry_data_nxt  = load_new_entry_i ? new_entry_data_i
                       : delete_entry_i   ? entry_data_o
                       : shift_entry_i    ? shift_in_entry_data_i
                       :                    entry_data_o;

always @ (posedge clk)
   if (reset) begin
      entry_valid_o <= 1'b0;
      entry_data_o  <= 'b0;
   end else begin
      entry_valid_o <= entry_valid_nxt;
      entry_data_o  <= entry_data_nxt;
   end

// Task colour and accelerator ID from stored entry:
wire                       entry_colour  = entry_data_o[COLOUR_START];
wire [TGT_ACC_SZ-1:0]      entry_acc_id  = entry_data_o[ACC_ID_START +: TGT_ACC_SZ];
wire [NUM_HW_ACCELERATORS-1:0] required_acc = {{(NUM_HW_ACCELERATORS-1){1'b0}}, 1'b1} << entry_acc_id;
wire acc_free = &(~required_acc | ~acc_busy_i);

// Per-slot readiness signals (one bit each for source and target checks):
wire [NUM_SLOTS-1:0] slot_src_ok;
wire [NUM_SLOTS-1:0] slot_tgt_ok;

// Short slots 0-2 (mode + id, no ntgt):
genvar gs;
generate
   for (gs = 0; gs < SLOTS_SHORT; gs = gs + 1) begin : slot_short
      localparam BASE = gs * SLOT_SHORT_SZ;
      wire [MODE_SZ-1:0]      md  = entry_data_o[BASE           +: MODE_SZ];
      wire [BUFF_INDX_SZ-1:0] bid = entry_data_o[BASE + MODE_SZ +: BUFF_INDX_SZ];
      wire needs_full = (md == `MODE_SRC) | (md == `MODE_RW);
      // RW needs no free check: full=1 already implies the buffer is not busy.
      // Busy state (full=0, free=0) is caught by the full check failing.
      wire needs_free = (md == `MODE_TGT);
      assign slot_src_ok[gs] = ~needs_full
                             | (buffers_full_i[bid] & (buffers_colour_i[bid] == entry_colour));
      assign slot_tgt_ok[gs] = ~needs_free | buffers_free_i[bid];
   end

   // Long slots 3-5 (mode + id + ntgt):
   for (gs = 0; gs < SLOTS_LONG; gs = gs + 1) begin : slot_long
      localparam BASE = SLOTS_SHORT*SLOT_SHORT_SZ + gs * SLOT_LONG_SZ;
      wire [MODE_SZ-1:0]      md  = entry_data_o[BASE           +: MODE_SZ];
      wire [BUFF_INDX_SZ-1:0] bid = entry_data_o[BASE + MODE_SZ +: BUFF_INDX_SZ];
      wire needs_full = (md == `MODE_SRC) | (md == `MODE_RW);
      wire needs_free = (md == `MODE_TGT);
      assign slot_src_ok[SLOTS_SHORT + gs] = ~needs_full
                             | (buffers_full_i[bid] & (buffers_colour_i[bid] == entry_colour));
      assign slot_tgt_ok[SLOTS_SHORT + gs] = ~needs_free | buffers_free_i[bid];
   end
endgenerate

assign ready_to_execute_o = entry_valid_o & acc_free & (&slot_src_ok) & (&slot_tgt_ok);

endmodule // sch_entry
