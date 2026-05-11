`include "constants.v"

`define OUTPUTS 400

`timescale 10ps/1ps

localparam PROG_ADDR_BITS  = 10;
localparam PROG_DATA_BITS  = 32;
localparam NUM_HW_ACCELERATORS = 2;
localparam TGT_ACC_SZ      = $clog2(NUM_HW_ACCELERATORS);
localparam CFG_ID_SZ       = 5;
localparam SCH_ENTRY_SZ    = 32;
localparam NUM_BUFFERS     = 16;
localparam COL_BUFF_ID_SZ  = 16;
localparam NUM_SCH_ENTRIES = 4;

localparam BUFF_INDX_SZ = $clog2(NUM_BUFFERS);
localparam HW_ACC_SZ            = $clog2(NUM_HW_ACCELERATORS);

module top ();

reg                       clk;
reg                       reset;
reg [NUM_HW_ACCELERATORS-1:0] acc_busy;
reg [NUM_HW_ACCELERATORS-1:0] acc_finished;
reg [NUM_HW_ACCELERATORS-1:0] acc_available;
reg [NUM_BUFFERS-1:0]         buff_free;
reg [NUM_BUFFERS-1:0]         buff_full;
reg [NUM_BUFFERS-1:0]         buffers_colour;

reg                     mark_buff_as_full;
reg  [BUFF_INDX_SZ-1:0] full_buff_id;
reg               [2:0] full_buff_usage;
reg                     start_new_task;
reg    [TGT_ACC_SZ-1:0] to_launch_acc_hw_id;
reg  [BUFF_INDX_SZ-1:0] to_launch_tgt_buff_id;
reg  [TGT_COUNT_SZ-1:0] to_launch_num_tgts;
reg                     to_launch_tgt_colour;
reg  [BUFF_INDX_SZ-1:0] to_launch_src1_buff_id;
reg  [BUFF_INDX_SZ-1:0] to_launch_src2_buff_id;
reg  [BUFF_INDX_SZ-1:0] to_launch_src3_buff_id;

initial
begin
    $dumpfile("top.vcd");
    $dumpvars(0, top);
end

initial clk = 1'b1;

always #5 clk <= !clk;

initial
begin
    reset <= 1'b1;
    repeat (2) @ (posedge clk);
    reset <= 1'b0;
end

initial
begin
    repeat (4000) @ (posedge clk);
    $finish;
end

initial
begin
	acc_busy               <= 2'b00;
	acc_finished           <=  'b0;
	mark_buff_as_full      <= 1'b0;
	full_buff_id           <=  'b0;
	full_buff_usage        <=  'b0;
	start_new_task         <= 1'b0;
	to_launch_acc_hw_id    <=  'b0;
	to_launch_tgt_buff_id  <=  'b0;
	to_launch_num_tgts     <=  'b0;
	to_launch_tgt_colour   <= 1'b0;
	to_launch_src1_buff_id <=  'b0;
	to_launch_src2_buff_id <=  'b0;
	to_launch_src3_buff_id <=  'b0;
	repeat (3) @ (posedge clk);
	acc_busy               <= 2'b00;
	acc_finished           <=  'b0;
	mark_buff_as_full      <= 1'b0;
	full_buff_id           <=  'b0;
	full_buff_usage        <=  'b0;
	start_new_task         <= 1'b1;
	to_launch_acc_hw_id    <=  'b0;
	to_launch_tgt_buff_id  <=  'b101; // Tgt Buff 5
	to_launch_num_tgts     <=  'b10;// Result will be used in 2 other tasks
	to_launch_tgt_colour   <= 1'b0;
	to_launch_src1_buff_id <=  'b01;
	to_launch_src2_buff_id <=  'b10;
	to_launch_src3_buff_id <=  'b0;
	repeat (1) @ (posedge clk);
	acc_busy               <= 2'b00;
	acc_finished           <=  'b0;
	mark_buff_as_full      <= 1'b0;
	full_buff_id           <=  'b0;
	full_buff_usage        <=  'b0;
	start_new_task         <= 1'b0;
	to_launch_acc_hw_id    <=  'b0;
	to_launch_tgt_buff_id  <=  'b0;
	to_launch_num_tgts     <=  'b0;
	to_launch_tgt_colour   <= 1'b0;
	to_launch_src1_buff_id <=  'b0;
	to_launch_src2_buff_id <=  'b0;
	to_launch_src3_buff_id <=  'b0;
	repeat (2) @ (posedge clk);
	mark_buff_as_full      <= 1'b1;
	full_buff_id           <=  'b01;
	full_buff_usage        <=  'b01;
	repeat (1) @ (posedge clk);
	mark_buff_as_full      <= 1'b1;
	full_buff_id           <=  'b10;
	full_buff_usage        <=  'b10;
	repeat (1) @ (posedge clk);
	acc_busy               <= 2'b01;
	mark_buff_as_full      <= 1'b0;
	full_buff_id           <=  'b0;
	full_buff_usage        <=  'b0;
	repeat (4) @ (posedge clk);
	acc_busy               <= 2'b00;
	acc_finished           <=  'b1;
	repeat (1) @ (posedge clk);
	acc_finished           <=  'b0;
end

sch_buffer_state
       #(.NUM_BUFFERS(NUM_BUFFERS),
         .BUFF_INDX_SZ(BUFF_INDX_SZ),
         .TGT_ACC_SZ(HW_ACC_SZ),
         .TGT_COUNT_SZ(TGT_COUNT_SZ))
         sch_buff_state0
       (.clk(clk),
        .reset(reset),
        .acc_busy_i(acc_busy),
        .acc_finished_i(acc_finished),
	.mark_buff_as_full_i(mark_buff_as_full),
        .full_buff_id_i(full_buff_id),
	.full_buff_usage_i(full_buff_usage),
        .start_new_task_i(start_new_task),
        .tgt_acc_id_i(to_launch_acc_hw_id),
        .tgt_buff_idx_i(to_launch_tgt_buff_id),
        .tgt_usage_count_i(to_launch_num_tgts),
        .tgt_colour_i(to_launch_tgt_colour),
        .src1_buff_idx_i(to_launch_src1_buff_id),
        .src2_buff_idx_i(to_launch_src2_buff_id),
        .src3_buff_idx_i(to_launch_src3_buff_id),
        .acc_available_o(acc_available),
        .buffers_full_o(buff_full),
        .buffers_free_o(buff_free),
        .buffers_colour_o(buffers_colour)
        );

endmodule

