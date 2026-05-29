// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
`timescale 1ns/1ps

// Testbench for sch_table.v
//
// Entry format (lsb first):
//   Slots 0-2 (short): [mode(2), id(4)] × 3 = 18 bits
//   Slots 3-5 (long):  [mode(2), id(4), ntgt(3)] × 3 = 27 bits
//   colour(1), acc_id(2), cfg_id(5) = 8 bits
//   Total = 53 bits
//
// Entry A: acc0, slot0=SRC/buff4, slot1=SRC/buff8, slot5=TGT/buff15/ntgt=1, colour=0
// Entry B: acc1, slot0=SRC/buff1, slot1=SRC/buff2, slot5=TGT/buff14/ntgt=1, colour=0
//
// Settling note: entries load into slot 3 and shift toward slot 0.
//   One entry takes ~3 cycles to reach slot 0 after loading.

module top;

localparam SCH_ENTRY_SZ        = 53;
localparam TGT_ACC_SZ          = 2;
localparam NUM_BUFFERS         = 16;
localparam COL_BUFF_ID_SZ      = 16;
localparam NUM_SCH_ENTRIES     = 4;
localparam NUM_HW_ACCELERATORS = 2;
localparam BUFF_INDX_SZ        = 4;
localparam TGT_COUNT_SZ        = 3;
localparam NUM_SLOTS           = 6;
localparam MODE_SZ             = 2;

localparam SLOT_SHORT_SZ = MODE_SZ + BUFF_INDX_SZ;            // 6
localparam SLOT_LONG_SZ  = MODE_SZ + BUFF_INDX_SZ + TGT_COUNT_SZ; // 9
localparam LONG_BASE     = 3 * SLOT_SHORT_SZ;                 // 18
localparam E_COLOUR      = LONG_BASE + 3 * SLOT_LONG_SZ;      // 45
localparam E_ACC_START   = E_COLOUR + 1;                      // 46

localparam MODE_UNUSED = 2'b00;
localparam MODE_SRC    = 2'b01;
localparam MODE_RW     = 2'b10;
localparam MODE_TGT    = 2'b11;

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

// Build a 53-bit entry from per-slot fields.
// Short slots 0-2: mode and id only.
// Long  slots 3-5: mode, id, and ntgt.
reg [SCH_ENTRY_SZ-1:0] entry_build;

task build_entry;
    input [MODE_SZ-1:0]    m0; input [BUFF_INDX_SZ-1:0]  i0;
    input [MODE_SZ-1:0]    m1; input [BUFF_INDX_SZ-1:0]  i1;
    input [MODE_SZ-1:0]    m2; input [BUFF_INDX_SZ-1:0]  i2;
    input [MODE_SZ-1:0]    m3; input [BUFF_INDX_SZ-1:0]  i3; input [TGT_COUNT_SZ-1:0] n3;
    input [MODE_SZ-1:0]    m4; input [BUFF_INDX_SZ-1:0]  i4; input [TGT_COUNT_SZ-1:0] n4;
    input [MODE_SZ-1:0]    m5; input [BUFF_INDX_SZ-1:0]  i5; input [TGT_COUNT_SZ-1:0] n5;
    input                  colour;
    input [TGT_ACC_SZ-1:0] acc_id;
    input [4:0]            cfg_id;
    begin
        entry_build = 'b0;
        entry_build[0*SLOT_SHORT_SZ +: MODE_SZ]         = m0;
        entry_build[0*SLOT_SHORT_SZ + MODE_SZ +: BUFF_INDX_SZ] = i0;
        entry_build[1*SLOT_SHORT_SZ +: MODE_SZ]         = m1;
        entry_build[1*SLOT_SHORT_SZ + MODE_SZ +: BUFF_INDX_SZ] = i1;
        entry_build[2*SLOT_SHORT_SZ +: MODE_SZ]         = m2;
        entry_build[2*SLOT_SHORT_SZ + MODE_SZ +: BUFF_INDX_SZ] = i2;
        entry_build[LONG_BASE + 0*SLOT_LONG_SZ +: MODE_SZ]          = m3;
        entry_build[LONG_BASE + 0*SLOT_LONG_SZ + MODE_SZ +: BUFF_INDX_SZ]     = i3;
        entry_build[LONG_BASE + 0*SLOT_LONG_SZ + MODE_SZ + BUFF_INDX_SZ +: TGT_COUNT_SZ] = n3;
        entry_build[LONG_BASE + 1*SLOT_LONG_SZ +: MODE_SZ]          = m4;
        entry_build[LONG_BASE + 1*SLOT_LONG_SZ + MODE_SZ +: BUFF_INDX_SZ]     = i4;
        entry_build[LONG_BASE + 1*SLOT_LONG_SZ + MODE_SZ + BUFF_INDX_SZ +: TGT_COUNT_SZ] = n4;
        entry_build[LONG_BASE + 2*SLOT_LONG_SZ +: MODE_SZ]          = m5;
        entry_build[LONG_BASE + 2*SLOT_LONG_SZ + MODE_SZ +: BUFF_INDX_SZ]     = i5;
        entry_build[LONG_BASE + 2*SLOT_LONG_SZ + MODE_SZ + BUFF_INDX_SZ +: TGT_COUNT_SZ] = n5;
        entry_build[E_COLOUR]                  = colour;
        entry_build[E_ACC_START +: TGT_ACC_SZ] = acc_id;
        entry_build[E_ACC_START + TGT_ACC_SZ +: 5] = cfg_id;
    end
endtask

reg  [SCH_ENTRY_SZ-1:0]    load_entry_data;     // built by build_entry
reg                         load_new_entry_i;
reg                         delete_entry_i;
reg  [SCH_ENTRY_SZ-1:0]    entry_data_i;
reg  [NUM_HW_ACCELERATORS-1:0] acc_busy_i;
reg  [COL_BUFF_ID_SZ-1:0]  buffers_full_i;
reg  [COL_BUFF_ID_SZ-1:0]  buffers_free_i;
reg  [COL_BUFF_ID_SZ-1:0]  buffers_colour_i;

wire                        table_slot_free_o;
wire                        table_empty_o;
wire                        dispatch_to_acc_o;
wire [SCH_ENTRY_SZ-1:0]    entry_data_o;

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
    .buffers_colour_i(buffers_colour_i),
    .cm_busy_i(1'b0),     // standalone tb: no config_manager — never back-pressure
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
    entry_data_i     = 'b0;
    acc_busy_i       = 2'b11;    // all accs busy – nothing should dispatch yet
    buffers_full_i   = 16'h0000;
    buffers_free_i   = 16'hFFFF;
    buffers_colour_i = 16'h0000; // all colour=0

    repeat(3) @(posedge clk); #1;
    reset = 1'b0;
    @(negedge clk);

    // ------------------------------------------------------------------
    // Test 1: reset state – table empty, slot free
    // ------------------------------------------------------------------
    chk(table_empty_o,    1'b1, "reset: table empty");
    chk(table_slot_free_o, 1'b1, "reset: slot free");
    chk(dispatch_to_acc_o, 1'b0, "reset: no dispatch");

    // Build Entry A: acc0, slot0=SRC/buff4, slot1=SRC/buff8, slot5=TGT/buff15
    build_entry(
        MODE_SRC, 4'd4, MODE_SRC, 4'd8, MODE_UNUSED, 4'd0,
        MODE_UNUSED, 4'd0, 3'd0, MODE_UNUSED, 4'd0, 3'd0, MODE_TGT, 4'd15, 3'd1,
        1'b0, 2'd0, 5'd1);
    load_entry_data = entry_build;

    // Build Entry B: acc1, slot0=SRC/buff1, slot1=SRC/buff2, slot5=TGT/buff14
    // (built later before use)

    // ------------------------------------------------------------------
    // Test 2: load Entry A (conditions blocked → no dispatch)
    // ------------------------------------------------------------------
    @(posedge clk); #1;
    load_new_entry_i = 1'b1;
    entry_data_i     = load_entry_data;
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
    build_entry(
        MODE_SRC, 4'd1, MODE_SRC, 4'd2, MODE_UNUSED, 4'd0,
        MODE_UNUSED, 4'd0, 3'd0, MODE_UNUSED, 4'd0, 3'd0, MODE_TGT, 4'd14, 3'd1,
        1'b0, 2'd1, 5'd1);
    @(posedge clk); #1;
    load_new_entry_i = 1'b1;
    entry_data_i     = entry_build;
    @(posedge clk); #1;
    load_new_entry_i = 1'b0;
    @(negedge clk);
    chk(dispatch_to_acc_o, 1'b0, "B loaded: still blocked");

    repeat(4) @(posedge clk);
    @(negedge clk);

    // ------------------------------------------------------------------
    // Test 4: enable conditions for Entry A only
    //   acc0 free, slot0/buff4 full, slot1/buff8 full, slot5/buff15 free
    // ------------------------------------------------------------------
    dispatch_seen    = 1'b0;
    acc_busy_i       = 2'b10;       // acc0 free, acc1 busy
    buffers_full_i   = 16'h0110;    // bits 4 and 8 set (slot0, slot1 for A)
    buffers_free_i   = 16'hFFFF;
    buffers_colour_i = 16'h0000;    // all colour=0, matches entry colour=0

    // Entry B (acc1, buff1, buff2) stays blocked since acc1 busy and buff1,2 not full
    repeat(4) @(posedge clk);
    @(negedge clk);
    chk(dispatch_seen, 1'b1, "A ready: dispatch asserted");

    // ------------------------------------------------------------------
    // Test 5: after A dispatches, enable conditions for B
    // ------------------------------------------------------------------
    dispatch_seen  = 1'b0;
    @(posedge clk);
    acc_busy_i     = 2'b01;         // acc0 busy, acc1 free
    buffers_full_i = 16'h0006;      // bits 1 and 2 set (slot0, slot1 for B)
    @(negedge clk);

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
            @(negedge clk);
            while (!table_slot_free_o) @(negedge clk);
            @(posedge clk); #1;
            load_new_entry_i = 1'b1;
            entry_data_i     = load_entry_data;   // Entry A
            @(posedge clk); #1;
            load_new_entry_i = 1'b0;
        end
    end
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
