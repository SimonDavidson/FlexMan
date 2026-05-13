// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
module sch_table  #(parameter SCH_ENTRY_SZ        = 52,
                    parameter NUM_HW_ACCELERATORS = 2,
                    parameter TGT_ACC_SZ          = 1,
                    parameter NUM_BUFFERS         = 16,
                    parameter COL_BUFF_ID_SZ      = 16,
                    parameter NUM_SCH_ENTRIES     = 4,
                    parameter ACC_ID_BTM          = 0
    )
             (input  wire                      clk,
              input  wire                      reset,

              // Load new entries:
              output wire                      table_slot_free_o,
              output wire                      table_empty_o,
              input  wire                      load_new_entry_i,
              input  wire                      delete_entry_i,
              input  wire [SCH_ENTRY_SZ-1:0]   entry_data_i,

              // Resource state:
              input  wire [NUM_HW_ACCELERATORS-1:0] acc_busy_i,
              input  wire [COL_BUFF_ID_SZ-1:0] buffers_full_i,
              input  wire [COL_BUFF_ID_SZ-1:0] buffers_free_i,
              input  wire [COL_BUFF_ID_SZ-1:0] buffers_colour_i,

              // Dispatch:
              output wire                      dispatch_to_acc_o,
              output wire [SCH_ENTRY_SZ-1:0]   entry_data_o
             );

wire [SCH_ENTRY_SZ-1:0] entry_data_r [0:NUM_SCH_ENTRIES-1];
wire  [NUM_SCH_ENTRIES:0]  entry_valid_r;
wire  [NUM_SCH_ENTRIES:0]  shift_out_valid;
reg  [NUM_SCH_ENTRIES:0]   shift_entry;
wire [NUM_SCH_ENTRIES-1:0] new_entry_valid_r;
wire [NUM_SCH_ENTRIES-1:0] ready_to_go;
wire [NUM_SCH_ENTRIES-1:0] load_entry;
wire [NUM_SCH_ENTRIES-1:0] delete_entry;
wire [NUM_SCH_ENTRIES-1:0] delete_launched_entry;
reg  [NUM_SCH_ENTRIES-1:0] delete_shifted_entry;
reg  [NUM_SCH_ENTRIES:0]   select_to_go;
wire                       launching;
wire                       free_to_add_entry;
wire                       table_empty;

assign table_slot_free_o = free_to_add_entry;
assign free_to_add_entry = ~entry_valid_r[NUM_SCH_ENTRIES-1];

assign new_entry_valid_r[NUM_SCH_ENTRIES-1]   = load_new_entry_i;
assign new_entry_valid_r[NUM_SCH_ENTRIES-2:0] = {(NUM_SCH_ENTRIES-1){1'b0}};

assign table_empty   = ~|entry_valid_r;
assign table_empty_o = table_empty;

assign load_entry[NUM_SCH_ENTRIES-1] = free_to_add_entry & load_new_entry_i;
assign load_entry[NUM_SCH_ENTRIES-2:0] = 'b0;

// Select the earliest (lowest-index) ready slot:
integer j;
always @(*) begin
   select_to_go = {(NUM_SCH_ENTRIES+1){1'b0}};
   for (j = NUM_SCH_ENTRIES-1; j >= 0; j = j - 1)
      if (ready_to_go[j]) select_to_go = ({{NUM_SCH_ENTRIES{1'b0}}, 1'b1} << j);
end

integer i;
always @ (*)
begin
   shift_entry[0] = ~load_entry[0] & ~select_to_go[1]
                     &  entry_valid_r[1] & (~entry_valid_r[0]);
   shift_entry[NUM_SCH_ENTRIES] = 1'b0;
   for (i = 1; i < NUM_SCH_ENTRIES-1; i = i+1)
      shift_entry[i] = ~load_entry[i] & ~select_to_go[i+1]
                     &  entry_valid_r[i+1]
                     & (~entry_valid_r[i] | shift_entry[i-1]);
   shift_entry[NUM_SCH_ENTRIES-1] = ~load_entry[NUM_SCH_ENTRIES-1]
                                  &  entry_valid_r[NUM_SCH_ENTRIES]
                                  & (~entry_valid_r[NUM_SCH_ENTRIES-1]
                                    | shift_entry[NUM_SCH_ENTRIES-2]);
end

always @ (load_entry, select_to_go, shift_entry, shift_out_valid)
begin
   delete_shifted_entry[0] = 1'b0;
   for (i = 1; i < NUM_SCH_ENTRIES; i = i+1)
      delete_shifted_entry[i] = ~load_entry[i] &
                               ((select_to_go[i+1]   &  shift_entry[i-1])
                              | (~shift_out_valid[i+1] & ~shift_entry[i] & shift_entry[i-1]));
end

reg [SCH_ENTRY_SZ-1:0] entry_data_mux;
integer k;
always @(*) begin
   entry_data_mux = entry_data_r[0];
   for (k = 0; k < NUM_SCH_ENTRIES; k = k + 1)
      if (select_to_go[k]) entry_data_mux = entry_data_r[k];
end
assign entry_data_o = entry_data_mux;

assign delete_launched_entry = select_to_go[NUM_SCH_ENTRIES-1:0];
assign delete_entry          = delete_launched_entry | delete_shifted_entry;
assign launching             = |ready_to_go;
assign dispatch_to_acc_o     = launching;

assign entry_valid_r[NUM_SCH_ENTRIES]   = 1'b0;
assign shift_out_valid[NUM_SCH_ENTRIES] = 1'b0;

// Instantiate scheduler entries:
sch_entry #(.SCH_ENTRY_SZ(SCH_ENTRY_SZ), .NUM_HW_ACCELERATORS(NUM_HW_ACCELERATORS),
            .TGT_ACC_SZ(TGT_ACC_SZ), .NUM_BUFFERS(NUM_BUFFERS), .ACC_ID_BTM(ACC_ID_BTM)
           ) sch_entry0 (
   .clk(clk), .reset(reset),
   .load_new_entry_i(load_entry[0]), .shift_entry_i(shift_entry[0]),
   .delete_entry_i(delete_entry[0]), .new_entry_valid_i(new_entry_valid_r[0]),
   .new_entry_data_i(entry_data_i),
   .shift_in_entry_valid_i(entry_valid_r[1]), .shift_in_entry_data_i(entry_data_r[1]),
   .shift_out_entry_valid_o(shift_out_valid[0]),
   .entry_valid_o(entry_valid_r[0]), .entry_data_o(entry_data_r[0]),
   .acc_busy_i(acc_busy_i), .buffers_full_i(buffers_full_i),
   .buffers_free_i(buffers_free_i), .buffers_colour_i(buffers_colour_i),
   .ready_to_execute_o(ready_to_go[0]));

sch_entry #(.SCH_ENTRY_SZ(SCH_ENTRY_SZ), .NUM_HW_ACCELERATORS(NUM_HW_ACCELERATORS),
            .TGT_ACC_SZ(TGT_ACC_SZ), .NUM_BUFFERS(NUM_BUFFERS), .ACC_ID_BTM(ACC_ID_BTM)
           ) sch_entry1 (
   .clk(clk), .reset(reset),
   .load_new_entry_i(load_entry[1]), .shift_entry_i(shift_entry[1]),
   .delete_entry_i(delete_entry[1]), .new_entry_valid_i(new_entry_valid_r[1]),
   .new_entry_data_i(entry_data_i),
   .shift_in_entry_valid_i(entry_valid_r[2]), .shift_in_entry_data_i(entry_data_r[2]),
   .shift_out_entry_valid_o(shift_out_valid[1]),
   .entry_valid_o(entry_valid_r[1]), .entry_data_o(entry_data_r[1]),
   .acc_busy_i(acc_busy_i), .buffers_full_i(buffers_full_i),
   .buffers_free_i(buffers_free_i), .buffers_colour_i(buffers_colour_i),
   .ready_to_execute_o(ready_to_go[1]));

sch_entry #(.SCH_ENTRY_SZ(SCH_ENTRY_SZ), .NUM_HW_ACCELERATORS(NUM_HW_ACCELERATORS),
            .TGT_ACC_SZ(TGT_ACC_SZ), .NUM_BUFFERS(NUM_BUFFERS), .ACC_ID_BTM(ACC_ID_BTM)
           ) sch_entry2 (
   .clk(clk), .reset(reset),
   .load_new_entry_i(load_entry[2]), .shift_entry_i(shift_entry[2]),
   .delete_entry_i(delete_entry[2]), .new_entry_valid_i(new_entry_valid_r[2]),
   .new_entry_data_i(entry_data_i),
   .shift_in_entry_valid_i(entry_valid_r[3]), .shift_in_entry_data_i(entry_data_r[3]),
   .shift_out_entry_valid_o(shift_out_valid[2]),
   .entry_valid_o(entry_valid_r[2]), .entry_data_o(entry_data_r[2]),
   .acc_busy_i(acc_busy_i), .buffers_full_i(buffers_full_i),
   .buffers_free_i(buffers_free_i), .buffers_colour_i(buffers_colour_i),
   .ready_to_execute_o(ready_to_go[2]));

sch_entry #(.SCH_ENTRY_SZ(SCH_ENTRY_SZ), .NUM_HW_ACCELERATORS(NUM_HW_ACCELERATORS),
            .TGT_ACC_SZ(TGT_ACC_SZ), .NUM_BUFFERS(NUM_BUFFERS), .ACC_ID_BTM(ACC_ID_BTM)
           ) sch_entry3 (
   .clk(clk), .reset(reset),
   .load_new_entry_i(load_entry[3]), .shift_entry_i(shift_entry[3]),
   .delete_entry_i(delete_entry[3]), .new_entry_valid_i(new_entry_valid_r[3]),
   .new_entry_data_i(entry_data_i),
   .shift_in_entry_valid_i(1'b0), .shift_in_entry_data_i(entry_data_r[3]),
   .shift_out_entry_valid_o(shift_out_valid[3]),
   .entry_valid_o(entry_valid_r[3]), .entry_data_o(entry_data_r[3]),
   .acc_busy_i(acc_busy_i), .buffers_full_i(buffers_full_i),
   .buffers_free_i(buffers_free_i), .buffers_colour_i(buffers_colour_i),
   .ready_to_execute_o(ready_to_go[3]));

endmodule // sch_table
