// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
`include "../../shared/constants.v"

`define OUTPUTS 400

`timescale 10ps/1ps

module top ();

reg                   clk;
reg                   reset;
reg                   load_new_entry;
reg                   delete_entry;
reg  [15:0]           buff_full;
reg  [15:0]           buff_free;
reg  [31:0]           new_entry_data;
wire                  start_new_block;
reg  [1:0]            acc_busy;

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
    repeat (40) @ (posedge clk);
    $finish;
end

initial
begin
	load_new_entry <= 1'b0;
	delete_entry   <= 1'b0;
	buff_full      <= 16'h0000;
	buff_free      <= 16'hFFFF;
	new_entry_data <= 32'h00000000;
	acc_busy       <= 2'b11;
	repeat (3) @ (posedge clk);
	buff_full      <= 16'h0000;
	buff_free      <= 16'hFFFF;
	load_new_entry <= 1'b1;
	new_entry_data <= 32'hABCDEF21;
	acc_busy       <= 2'b11;
	repeat (1) @ (posedge clk);
	buff_full      <= 16'hFFFF;
	buff_free      <= 16'hFFFF;
	load_new_entry <= 1'b0;
	new_entry_data <= 32'h00000000;
	repeat (2) @ (posedge clk);
	load_new_entry <= 1'b1;
	new_entry_data <= 32'h1234561A;
	repeat (1) @ (posedge clk);
	buff_full      <= 16'hFF2F;
	acc_busy       <= 2'b01;
	load_new_entry <= 1'b0;
	new_entry_data <= 32'h00000000;
	repeat (1) @ (posedge clk);
	acc_busy       <= 2'b11;
	repeat (2) @ (posedge clk);
	acc_busy       <= 2'b10;
	repeat (1) @ (posedge clk);
	buff_full      <= 16'hFFFF;
	acc_busy       <= 2'b00;
end

/*
weight_mem weight_mem(
    .clk(clk),
    .reset(reset),
    .weight_rd_i(weight_mem_rd),
    .weight_wait_o(weight_mem_wait),
    .weight_addr_i(weight_mem_addr),
    .weight_data_o(weight_mem_data)
);
*/

sch_table  #(
	      .SCH_ENTRY_SZ(32),
              .TGT_ACC_SZ(2),
              .NUM_BUFFERS(16),
              .COL_BUFF_ID_SZ(16),
              .NUM_SCH_ENTRIES(4)
            ) sch_table0 (
             .clk(clk),
             .reset(reset),
	     .table_slot_free_o(table_slot_free),
             .load_new_entry_i(load_new_entry),
	     .delete_entry_i(delete_entry),
             .entry_data_i(new_entry_data),
             .acc_busy_i(acc_busy),
             .buffers_full_i(buff_full),
             .buffers_free_i(buff_free),
             .dispatch_to_acc_o(start_new_block),
             .entry_data_o()
             );

endmodule
