/////////////////////////////////////////////////////////////////////////
//
// sch_ctrl
//
// Reads the program from memory, decodes and executes instructions
// and places task in the scheduler table. Executes jumps and early exit 
// checks, manages stop and start at the program level.
//
// Author: Simon D.
// Date: 18/2/2025
// Version: 1.0
//
//////////////////////////////////////////////////////////////////////////

module sch_ctrl  #(parameter SCH_ENTRY_SZ         = 32,
                    parameter NUM_HW_ACCELERATORS = 2,
	            parameter TGT_ACC_SZ          = 2,
		    parameter NUM_BUFFERS         = 16,
                    parameter COL_BUFF_ID_SZ      = 16,
		    parameter NUM_SCH_ENTRIES     = 4
	    )
             (input  wire                      clk,
              input  wire                      reset,

              output wire [SCH_ENTRY_SZ-1:0]   entry_data_o
             );

);

endmodule	// sch_ctrl

