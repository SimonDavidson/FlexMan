module sch_table  #(parameter SCH_ENTRY_SZ    = 32,
                    parameter NUM_HW_ACCELERATORS = 2,
	            parameter TGT_ACC_SZ      = 2,
		    parameter NUM_BUFFERS     = 16,
                    parameter COL_BUFF_ID_SZ  = 16,
		    parameter NUM_SCH_ENTRIES = 4
	    )
             (input  wire                      clk,
              input  wire                      reset,

	      // Load new entries
	      output wire                      table_slot_free_o,
	      output wire                      table_empty_o,
              input  wire                      load_new_entry_i,
              input  wire                      delete_entry_i,
              input  wire [SCH_ENTRY_SZ-1:0]   entry_data_i,

	      // Resource state:
              input  wire [NUM_HW_ACCELERATORS-1:0] acc_busy_i,
	      input  wire [COL_BUFF_ID_SZ-1:0] buffers_full_i,
	      input  wire [COL_BUFF_ID_SZ-1:0] buffers_free_i,

	      // Send new process to be executed:
	      output wire                      dispatch_to_acc_o,
              output wire [SCH_ENTRY_SZ-1:0]   entry_data_o
             );

localparam NUM_SCHEDULABLE = 4;
localparam CMD_SZ          = 3;

localparam TGT_ACC_BTM = 0;
localparam TGT_ACC_TOP = TGT_ACC_SZ  - 1;
localparam INBUFFS_BTM = TGT_ACC_TOP + 1;
localparam INBUFFS_TOP = INBUFFS_BTM + NUM_BUFFERS - 1;
localparam OUTBUFF_BTM = INBUFFS_TOP + 1;
localparam OUTBUFF_TOP = OUTBUFF_BTM + NUM_BUFFERS - 1;
localparam CMD_BTM     = OUTBUFF_TOP + 1;
localparam CMD_TOP     = CMD_BTM + CMD_SZ;

wire [SCH_ENTRY_SZ-1:0] entry_data_r [0:NUM_SCH_ENTRIES-1];
wire [NUM_SCH_ENTRIES:0]   entry_valid_r;
wire [NUM_SCH_ENTRIES:0]   shift_out_valid;
reg  [NUM_SCH_ENTRIES:0]   shift_entry;
wire [NUM_SCH_ENTRIES-1:0] new_entry_valid_r;
wire [NUM_SCH_ENTRIES-1:0] ready_to_go;
wire [NUM_SCH_ENTRIES-1:0] load_entry;
wire [NUM_SCH_ENTRIES-1:0] delete_entry;
wire [NUM_SCH_ENTRIES-1:0] delete_launched_entry;
reg  [NUM_SCH_ENTRIES-1:0] delete_shifted_entry;
wire [NUM_SCHEDULABLE:0]   select_to_go;
wire                       launching;
wire                       free_to_add_entry;
wire                       table_empty;

//////////////////////////////////////////////////
//
//Indicate if we have any free table entry slots:

assign table_slot_free_o = ~&entry_valid_r;

// Top level will not try to load an entry if there
// are no slots free.

//////////////////////////////////////////////////
//
// Select one entry to execute and generate control
// signals for shifting, adding and deleting entries
//
// Select the earliest slot that is ready to go:
assign select_to_go =  {1'b0, ready_to_go[NUM_SCHEDULABLE-1:0]}         &  // TODO!
                      ~{1'b0, ready_to_go[NUM_SCHEDULABLE-2:0], 1'b0}   &
                      ~{1'b0, ready_to_go[NUM_SCHEDULABLE-3:0], 2'b00}  &
                      ~{1'b0, ready_to_go[NUM_SCHEDULABLE-4:0], 3'b000};

//assign free_to_add_entry = ~&entry_valid_r;
//assign free_to_add_entry = ~entry_valid_r[NUM_SCH_ENTRIES-1] | 
//	                    shift_entry[NUM_SCH_ENTRIES-1];
assign free_to_add_entry = ~entry_valid_r[NUM_SCH_ENTRIES-1];

assign new_entry_valid_r[3]   = load_new_entry_i;
assign new_entry_valid_r[2:0] = 3'b000;

assign table_empty = ~|entry_valid_r;

assign table_empty_o = table_empty;

// Load signals. Either a shuffle down (when housekeeping) or loading an
// external entry.
assign load_entry[NUM_SCH_ENTRIES-1] = (free_to_add_entry && load_new_entry_i)?
                                                               1'b1 : 1'b0;
assign load_entry[NUM_SCH_ENTRIES-2:0] = 'b0;

// Shift_entry is high to mean that the entry should take its next 
// values from the entry after it in the list, but this is superceded
// by a fresh load from outside:
integer i;
always @ (*)
begin
   shift_entry[0]    <= ~load_entry[0] & ~select_to_go[1]
                        &   entry_valid_r[1]
                        & (~entry_valid_r[0]);

   shift_entry[4]    <= 1'b0;

   for(i=1; i<NUM_SCH_ENTRIES-1; i=i+1)
   begin
      shift_entry[i] <=    ~load_entry[i] & ~select_to_go[i+1] 
                        &   entry_valid_r[i+1]
                        & (~entry_valid_r[i] | shift_entry[i-1]);
   end
   shift_entry[NUM_SCH_ENTRIES-1] <=    ~load_entry[NUM_SCH_ENTRIES-1] 
                                     &   entry_valid_r[NUM_SCH_ENTRIES]
                                     & (~entry_valid_r[NUM_SCH_ENTRIES-1] 
				       | shift_entry[NUM_SCH_ENTRIES-2]);
end

always @ (load_entry, select_to_go, shift_entry, shift_out_valid)
begin
   delete_shifted_entry[0] <= 1'b0; //select_to_go[1] & ~load_entry[0];
   for(i=1; i<NUM_SCH_ENTRIES; i=i+1)
   begin
      delete_shifted_entry[i] <=  ~load_entry[i]        &
                                (( select_to_go[i+1]    &  shift_entry[i-1])
			      |	 (~shift_out_valid[i+1] & ~shift_entry[i] & shift_entry[i-1]));
   end
end

assign entry_data_o = select_to_go[0] ? entry_data_r[0] :
                      select_to_go[1] ? entry_data_r[1] :
                      select_to_go[2] ? entry_data_r[2] :
                                        entry_data_r[3] ;

assign delete_launched_entry = select_to_go[0] ? 4'b0001 :
                               select_to_go[1] ? 4'b0010 :
                               select_to_go[2] ? 4'b0100 :
                               select_to_go[3] ? 4'b1000 :
                                                 4'b0000 ;

assign delete_entry = delete_launched_entry | delete_shifted_entry;

assign launching         = |ready_to_go;
assign dispatch_to_acc_o = launching;

//////////////////////////////////////////////////
//
// Instantiate list of scheduler entries

assign entry_valid_r[NUM_SCH_ENTRIES]   = 1'b0; // Used in shift process
assign shift_out_valid[NUM_SCH_ENTRIES] = 1'b0; // Used in shift process

sch_entry   #(
	     .SCH_ENTRY_SZ(SCH_ENTRY_SZ),
	     .NUM_HW_ACCELERATORS(NUM_HW_ACCELERATORS),
             .TGT_ACC_SZ(TGT_ACC_SZ),
             .NUM_BUFFERS(NUM_BUFFERS),
             .COL_BUFF_ID_SZ(COL_BUFF_ID_SZ)
            ) sch_entry0
             (.clk(clk),
              .reset(reset),
              .load_new_entry_i(load_entry[0]),
              .shift_entry_i(shift_entry[0]),
              .delete_entry_i(delete_entry[0]),
              .new_entry_valid_i(new_entry_valid_r[0]),
              .new_entry_data_i(entry_data_i),
              .shift_in_entry_valid_i(entry_valid_r[1]),
              .shift_in_entry_data_i(entry_data_r[1]),
              .shift_out_entry_valid_o(shift_out_valid[0]),
              .entry_valid_o(entry_valid_r[0]),
              .entry_data_o(entry_data_r[0]),
              .acc_busy_i(acc_busy_i),
              .buffers_full_i(buffers_full_i),
              .buffers_free_i(buffers_free_i),
              .ready_to_execute_o(ready_to_go[0])
              );

sch_entry  #(
	     .SCH_ENTRY_SZ(SCH_ENTRY_SZ),
	     .NUM_HW_ACCELERATORS(NUM_HW_ACCELERATORS),
             .TGT_ACC_SZ(TGT_ACC_SZ),
             .NUM_BUFFERS(NUM_BUFFERS),
             .COL_BUFF_ID_SZ(COL_BUFF_ID_SZ)
            ) sch_entry1
             (.clk(clk),
              .reset(reset),
              .load_new_entry_i(load_entry[1]),
              .shift_entry_i(shift_entry[1]),
              .delete_entry_i(delete_entry[1]),
              .new_entry_valid_i(new_entry_valid_r[1]),
              .new_entry_data_i(entry_data_i),
              .shift_in_entry_valid_i(entry_valid_r[2]),
              .shift_in_entry_data_i(entry_data_r[2]),
              .shift_out_entry_valid_o(shift_out_valid[1]),
              .entry_valid_o(entry_valid_r[1]),
              .entry_data_o(entry_data_r[1]),

              .acc_busy_i(acc_busy_i),
              .buffers_full_i(buffers_full_i),
              .buffers_free_i(buffers_free_i),
              .ready_to_execute_o(ready_to_go[1])
              );

sch_entry   #(
	     .SCH_ENTRY_SZ(SCH_ENTRY_SZ),
	     .NUM_HW_ACCELERATORS(NUM_HW_ACCELERATORS),
             .TGT_ACC_SZ(TGT_ACC_SZ),
             .NUM_BUFFERS(NUM_BUFFERS),
             .COL_BUFF_ID_SZ(COL_BUFF_ID_SZ)
            ) sch_entry2
             (.clk(clk),
              .reset(reset),
              .load_new_entry_i(load_entry[2]),
              .shift_entry_i(shift_entry[2]),
              .delete_entry_i(delete_entry[2]),
              .new_entry_valid_i(new_entry_valid_r[2]),
              .new_entry_data_i(entry_data_i),
              .shift_in_entry_valid_i(entry_valid_r[3]),
              .shift_in_entry_data_i(entry_data_r[3]),
              .shift_out_entry_valid_o(shift_out_valid[2]),
              .entry_valid_o(entry_valid_r[2]),
              .entry_data_o(entry_data_r[2]),

              .acc_busy_i(acc_busy_i),
              .buffers_full_i(buffers_full_i),
              .buffers_free_i(buffers_free_i),
              .ready_to_execute_o(ready_to_go[2])
              );

sch_entry   #(
	     .SCH_ENTRY_SZ(SCH_ENTRY_SZ),
	     .NUM_HW_ACCELERATORS(NUM_HW_ACCELERATORS),
             .TGT_ACC_SZ(TGT_ACC_SZ),
             .NUM_BUFFERS(NUM_BUFFERS),
             .COL_BUFF_ID_SZ(COL_BUFF_ID_SZ)
            ) sch_entry3
             (.clk(clk),
              .reset(reset),
              .load_new_entry_i(load_entry[3]),
              .shift_entry_i(shift_entry[3]),
              .delete_entry_i(delete_entry[3]),
              .new_entry_valid_i(new_entry_valid_r[3]),
              .new_entry_data_i(entry_data_i),
              .shift_in_entry_valid_i(1'b0),
              .shift_in_entry_data_i(entry_data_r[3]), // Temp as only 4
              .shift_out_entry_valid_o(shift_out_valid[3]),
              .entry_valid_o(entry_valid_r[3]),
              .entry_data_o(entry_data_r[3]),

              .acc_busy_i(acc_busy_i),
              .buffers_full_i(buffers_full_i),
              .buffers_free_i(buffers_free_i),
              .ready_to_execute_o(ready_to_go[3])
              );

endmodule	// sch_table

