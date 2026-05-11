// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
`timescale 1ns/1ps

// Testbench for sch_table.v
//
// Uses default parameters (TGT_ACC_SZ=2) matching old testbench convention.
// With TGT_ACC_SZ=2 sch_entry reads acc_id from entry_data[1:0]; those two
// bits are also the low bits of src1_buff_id – entries below are crafted to
// avoid conflict:
//   Entry A: src1=buff4(1000), acc bits[1:0]=00 → acc0 check
//   Entry B: src1=buff1(0001), acc bits[1:0]=01 → acc1 check
//
// Settling note: entries load into slot 3 and shift toward slot 0.
//   One entry takes ~3 cycles to reach slot 0 after loading.

module top;

localparam SCH_ENTRY_SZ    = 32;
localparam TGT_ACC_SZ      = 2;
localparam NUM_BUFFERS     = 16;
localparam COL_BUFF_ID_SZ  = 16;
localparam NUM_SCH_ENTRIES = 4;
localparam NUM_HW_ACCELERATORS = 2;

// Entry format (TGT_ACC_SZ=2):
//  bits[3:0]  = src1_buff  (acc at [1:0])
//  bits[7:4]  = src2_buff
//  bits[11:8] = src3_buff
//  bits[15:12]= tgt_buff
//
// Entry A: src1=4(1000)→acc0, src2=8(1000), src3=0, tgt=15(1111)
localparam ENTRY_A = 32'h0000_F084;  // tgt=F,src3=0,src2=8,src1=4
// Entry B: src1=1(0001)→acc1, src2=2(0010), src3=0, tgt=14(1110)
localparam ENTRY_B = 32'h0000_E021;  // tgt=E,src3=0,src2=2,src1=1

reg clk, reset;
initial clk = 1'b0;
always  #5 clk = ~clk;

initial begin
    $dumpfile("tb_sch_table.vcd");
    $dumpvars(0, top);
end

integer errors;
initial errors = 0;

task chk;
    input        got;
    input        exp;
    input [255:0] label;
    begin
        if (got !== exp) begin
            $display("FAIL [%0t] %0s: got=%0b exp=%0b", $time, label, got, exp);
            errors = errors + 1;
        end
    end
endtask

reg                    load_new_entry_i;
reg                    delete_entry_i;
reg  [SCH_ENTRY_SZ-1:0] entry_data_i;
reg  [NUM_HW_ACCELERATORS-1:0] acc_busy_i;
reg  [COL_BUFF_ID_SZ-1:0]      buffers_full_i;
reg  [COL_BUFF_ID_SZ-1:0]      buffers_free_i;

wire                   table_slot_free_o;
wire                   table_empty_o;
wire                   dispatch_to_acc_o;
wire [SCH_ENTRY_SZ-1:0] entry_data_o;

sch_table #(
    .SCH_ENTRY_SZ(SCH_ENTRY_SZ),
    .TGT_ACC_SZ(TGT_ACC_SZ),
    .NUM_BUFFERS(NUM_BUFFERS),
    .COL_BUFF_ID_SZ(COL_BUFF_ID_SZ),
    .NUM_SCH_ENTRIES(NUM_SCH_ENTRIES),
    .NUM_HW_ACCELERATORS(NUM_HW_ACCELERATORS)
) dut (
    .clk(clk),
    .reset(reset),
    .table_slot_free_o(table_slot_free_o),
    .table_empty_o(table_empty_o),
    .load_new_entry_i(load_new_entry_i),
    .delete_entry_i(delete_entry_i),
    .entry_data_i(entry_data_i),
    .acc_busy_i(acc_busy_i),
    .buffers_full_i(buffers_full_i),
    .buffers_free_i(buffers_free_i),
    .dispatch_to_acc_o(dispatch_to_acc_o),
    .entry_data_o(entry_data_o)
);

// Monitor dispatches; latch so testbench can check after the 1-cycle pulse
reg dispatch_seen;
initial dispatch_seen = 1'b0;
always @(posedge clk) begin
    if (dispatch_to_acc_o) begin
        $display("[%0t ns] DISPATCH  entry_data=%0h", $time, entry_data_o);
        dispatch_seen <= 1'b1;
    end
end

initial begin
    reset            = 1'b1;
    load_new_entry_i = 1'b0;
    delete_entry_i   = 1'b0;
    entry_data_i     = 32'h0;
    acc_busy_i       = 2'b11;    // all accs busy – nothing should dispatch yet
    buffers_full_i   = 16'h0000;
    buffers_free_i   = 16'hFFFF;

    repeat(3) @(posedge clk); #1;
    reset = 1'b0;
    @(negedge clk);

    // ------------------------------------------------------------------
    // Test 1: reset state – table empty, slot free
    // ------------------------------------------------------------------
    chk(table_empty_o,    1'b1, "reset: table empty");
    chk(table_slot_free_o, 1'b1, "reset: slot free");
    chk(dispatch_to_acc_o, 1'b0, "reset: no dispatch");

    // ------------------------------------------------------------------
    // Test 2: load Entry A (conditions blocked → no dispatch)
    // ------------------------------------------------------------------
    @(posedge clk); #1;
    load_new_entry_i = 1'b1;
    entry_data_i     = ENTRY_A;
    @(posedge clk); #1;
    load_new_entry_i = 1'b0;
    @(negedge clk);
    chk(table_empty_o,    1'b0, "A loaded: not empty");
    chk(dispatch_to_acc_o, 1'b0, "A loaded: blocked (acc busy)");

    // Wait for entry to settle toward slot 0
    repeat(4) @(posedge clk);
    @(negedge clk);
    chk(dispatch_to_acc_o, 1'b0, "A settling: still blocked");

    // ------------------------------------------------------------------
    // Test 3: load Entry B while A is settling
    // ------------------------------------------------------------------
    @(posedge clk); #1;
    load_new_entry_i = 1'b1;
    entry_data_i     = ENTRY_B;
    @(posedge clk); #1;
    load_new_entry_i = 1'b0;
    @(negedge clk);
    chk(dispatch_to_acc_o, 1'b0, "B loaded: still blocked");

    repeat(4) @(posedge clk);
    @(negedge clk);

    // ------------------------------------------------------------------
    // Test 4: enable conditions for Entry A only
    //   acc0 free, src1=buff4 full, src2=buff8 full, tgt=buff15 free
    // ------------------------------------------------------------------
    dispatch_seen  = 1'b0;           // clear latch before this test
    acc_busy_i     = 2'b10;          // acc0 free, acc1 busy
    buffers_full_i = 16'h0110;       // bits 4 and 8 set (src1,src2 for A)
    buffers_free_i = 16'hFFFF;

    // Entry B (acc1, src1=1, src2=2) stays blocked since acc1 busy and buff1,2 not full
    repeat(4) @(posedge clk);
    @(negedge clk);
    chk(dispatch_seen, 1'b1, "A ready: dispatch asserted");

    // ------------------------------------------------------------------
    // Test 5: after A dispatches, enable conditions for B
    // ------------------------------------------------------------------
    dispatch_seen  = 1'b0;           // clear latch before this test
    // A will be removed by the table on dispatch; wait for it to clear
    @(posedge clk);
    // Now free acc1 and provide B's src buffers
    acc_busy_i     = 2'b01;          // acc0 busy, acc1 free (invert)
    buffers_full_i = 16'h0006;       // bits 1 and 2 set (src1,src2 for B)
    @(negedge clk);

    // Wait for B to settle and dispatch
    repeat(8) @(posedge clk);
    @(negedge clk);
    chk(dispatch_seen, 1'b1, "B ready: dispatch asserted");

    // ------------------------------------------------------------------
    // Test 6: after both entries dispatched table should be empty
    // ------------------------------------------------------------------
    @(posedge clk);
    acc_busy_i     = 2'b11;
    buffers_full_i = 16'h0000;
    repeat(6) @(posedge clk);
    @(negedge clk);
    chk(table_empty_o,    1'b1, "all dispatched: table empty");
    chk(dispatch_to_acc_o, 1'b0, "empty table: no dispatch");

    // ------------------------------------------------------------------
    // Test 7: table_slot_free_o gates when full
    //   Load 4 entries to fill the table.
    // ------------------------------------------------------------------
    acc_busy_i     = 2'b11;   // block all dispatch during fill
    buffers_full_i = 16'h0000;
    buffers_free_i = 16'hFFFF;
    begin : fill_loop
        integer k;
        for (k = 0; k < 4; k = k + 1) begin
            // Wait until a slot is free before loading
            @(negedge clk);
            while (!table_slot_free_o) @(negedge clk);
            @(posedge clk); #1;
            load_new_entry_i = 1'b1;
            entry_data_i     = ENTRY_A;
            @(posedge clk); #1;
            load_new_entry_i = 1'b0;
        end
    end
    // After 4 entries and enough settling, slot 3 should be occupied
    repeat(2) @(posedge clk);
    @(negedge clk);
    chk(table_slot_free_o, 1'b0, "full table: no slot free");
    chk(table_empty_o,     1'b0, "full table: not empty");

    // ------------------------------------------------------------------
    @(posedge clk);
    if (errors == 0)
        $display("PASS – sch_table: all tests passed.");
    else
        $display("FAIL – sch_table: %0d error(s).", errors);
    $finish;
end

initial begin
    #30000;
    $display("TIMEOUT – sch_table");
    $finish;
end

endmodule
