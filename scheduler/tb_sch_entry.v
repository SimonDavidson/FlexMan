// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
`timescale 1ns/1ps

// Testbench for sch_entry.v
//
// Entry format (lsb first):
//   Slots 0-2 (short): [mode(2), id(4)] × 3 = 18 bits
//   Slots 3-5 (long):  [mode(2), id(4), ntgt(3)] × 3 = 27 bits
//   colour(1), acc_id(2), cfg_id(5)  = 8 bits
//   Total = 53 bits
//
// Slot modes: 00=unused 01=source 10=rw 11=target
//
// Readiness: entry_valid & acc_free &
//            (all source/RW slots: buffer full AND colour match) &
//            (all RW/target slots: buffer free)

module top;

localparam SCH_ENTRY_SZ        = 53;
localparam NUM_HW_ACCELERATORS = 2;
localparam TGT_ACC_SZ          = 2;
localparam NUM_BUFFERS         = 16;
localparam BUFF_INDX_SZ        = 4;
localparam TGT_COUNT_SZ        = 3;
localparam NUM_SLOTS           = 6;
localparam MODE_SZ             = 2;

localparam SLOT_SHORT_SZ = MODE_SZ + BUFF_INDX_SZ;           // 6
localparam SLOT_LONG_SZ  = MODE_SZ + BUFF_INDX_SZ + TGT_COUNT_SZ; // 9
localparam LONG_BASE     = 3 * SLOT_SHORT_SZ;                // 18
localparam E_COLOUR      = LONG_BASE + 3 * SLOT_LONG_SZ;     // 45
localparam E_ACC_START   = E_COLOUR + 1;                     // 46

localparam MODE_UNUSED = 2'b00;
localparam MODE_SRC    = 2'b01;
localparam MODE_RW     = 2'b10;
localparam MODE_TGT    = 2'b11;

reg clk, reset;
initial clk = 1'b0;
always  #5 clk = ~clk;

initial begin
    $dumpfile("tb_sch_entry.vcd");
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
// Short slots 0-2: provide mode and id only.
// Long  slots 3-5: provide mode, id, and ntgt.
// Also provide colour, acc_id, cfg_id.
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
        // Short slots:
        entry_build[0*SLOT_SHORT_SZ +: MODE_SZ]         = m0;
        entry_build[0*SLOT_SHORT_SZ + MODE_SZ +: BUFF_INDX_SZ] = i0;
        entry_build[1*SLOT_SHORT_SZ +: MODE_SZ]         = m1;
        entry_build[1*SLOT_SHORT_SZ + MODE_SZ +: BUFF_INDX_SZ] = i1;
        entry_build[2*SLOT_SHORT_SZ +: MODE_SZ]         = m2;
        entry_build[2*SLOT_SHORT_SZ + MODE_SZ +: BUFF_INDX_SZ] = i2;
        // Long slots:
        entry_build[LONG_BASE + 0*SLOT_LONG_SZ +: MODE_SZ]          = m3;
        entry_build[LONG_BASE + 0*SLOT_LONG_SZ + MODE_SZ +: BUFF_INDX_SZ]     = i3;
        entry_build[LONG_BASE + 0*SLOT_LONG_SZ + MODE_SZ + BUFF_INDX_SZ +: TGT_COUNT_SZ] = n3;
        entry_build[LONG_BASE + 1*SLOT_LONG_SZ +: MODE_SZ]          = m4;
        entry_build[LONG_BASE + 1*SLOT_LONG_SZ + MODE_SZ +: BUFF_INDX_SZ]     = i4;
        entry_build[LONG_BASE + 1*SLOT_LONG_SZ + MODE_SZ + BUFF_INDX_SZ +: TGT_COUNT_SZ] = n4;
        entry_build[LONG_BASE + 2*SLOT_LONG_SZ +: MODE_SZ]          = m5;
        entry_build[LONG_BASE + 2*SLOT_LONG_SZ + MODE_SZ +: BUFF_INDX_SZ]     = i5;
        entry_build[LONG_BASE + 2*SLOT_LONG_SZ + MODE_SZ + BUFF_INDX_SZ +: TGT_COUNT_SZ] = n5;
        // Header:
        entry_build[E_COLOUR]                  = colour;
        entry_build[E_ACC_START +: TGT_ACC_SZ] = acc_id;
        entry_build[E_ACC_START + TGT_ACC_SZ +: 5] = cfg_id;
    end
endtask

reg  [SCH_ENTRY_SZ-1:0] new_entry_data_i;
reg  [SCH_ENTRY_SZ-1:0] shift_in_entry_data_i;
reg  load_new_entry_i;
reg  shift_entry_i;
reg  delete_entry_i;
reg  new_entry_valid_i;
reg  shift_in_entry_valid_i;
reg  [NUM_HW_ACCELERATORS-1:0] acc_busy_i;
reg  [NUM_BUFFERS-1:0]         buffers_full_i;
reg  [NUM_BUFFERS-1:0]         buffers_free_i;
reg  [NUM_BUFFERS-1:0]         buffers_colour_i;

wire shift_out_entry_valid_o;
wire entry_valid_o;
wire [SCH_ENTRY_SZ-1:0] entry_data_o;
wire ready_to_execute_o;

sch_entry #(
    .SCH_ENTRY_SZ(SCH_ENTRY_SZ),
    .NUM_HW_ACCELERATORS(NUM_HW_ACCELERATORS),
    .TGT_ACC_SZ(TGT_ACC_SZ),
    .NUM_BUFFERS(NUM_BUFFERS),
    .BUFF_INDX_SZ(BUFF_INDX_SZ),
    .TGT_COUNT_SZ(TGT_COUNT_SZ),
    .NUM_SLOTS(NUM_SLOTS),
    .MODE_SZ(MODE_SZ)
) dut (
    .clk(clk),
    .reset(reset),
    .load_new_entry_i(load_new_entry_i),
    .shift_entry_i(shift_entry_i),
    .delete_entry_i(delete_entry_i),
    .new_entry_valid_i(new_entry_valid_i),
    .new_entry_data_i(new_entry_data_i),
    .shift_in_entry_valid_i(shift_in_entry_valid_i),
    .shift_in_entry_data_i(shift_in_entry_data_i),
    .shift_out_entry_valid_o(shift_out_entry_valid_o),
    .entry_valid_o(entry_valid_o),
    .entry_data_o(entry_data_o),
    .acc_busy_i(acc_busy_i),
    .buffers_full_i(buffers_full_i),
    .buffers_free_i(buffers_free_i),
    .buffers_colour_i(buffers_colour_i),
    .ready_to_execute_o(ready_to_execute_o)
);

// Entry used in most tests:
//   Slot 0: SOURCE  buff 4  (colour 0 required)
//   Slot 1: SOURCE  buff 5  (colour 0 required)
//   Slot 2: UNUSED
//   Slot 3: UNUSED  (long)
//   Slot 4: RW      buff 7  ntgt=1 (colour 0, must be full+colour; free check NOT required)
//   Slot 5: TARGET  buff 9  ntgt=1
//   colour=0, acc_id=0, cfg_id=1
// Ready when: acc0 free, buff4/5/7 full+colour0, buff9 free.

initial begin
    reset                  = 1'b1;
    load_new_entry_i       = 1'b0;
    shift_entry_i          = 1'b0;
    delete_entry_i         = 1'b0;
    new_entry_valid_i      = 1'b1;
    new_entry_data_i       = 'b0;
    shift_in_entry_valid_i = 1'b0;
    shift_in_entry_data_i  = 'b0;
    acc_busy_i             = 2'b11;
    buffers_full_i         = 16'h0000;
    buffers_free_i         = 16'hFFFF;
    buffers_colour_i       = 16'h0000;   // all colour=0

    repeat(3) @(posedge clk); #1;
    reset = 1'b0;
    @(negedge clk);

    // ------------------------------------------------------------------
    // Test 1: empty entry is not valid or ready
    // ------------------------------------------------------------------
    chk(entry_valid_o,      1'b0, "empty: valid=0");
    chk(ready_to_execute_o, 1'b0, "empty: not ready");

    // ------------------------------------------------------------------
    // Test 2: load entry – acc busy and source buffers empty → not ready
    // ------------------------------------------------------------------
    build_entry(
        MODE_SRC, 4'd4, MODE_SRC, 4'd5, MODE_UNUSED, 4'd0,
        MODE_UNUSED, 4'd0, 3'd0, MODE_RW, 4'd7, 3'd1, MODE_TGT, 4'd9, 3'd1,
        1'b0, 2'd0, 5'd1);
    @(posedge clk); #1;
    load_new_entry_i  = 1'b1;
    new_entry_data_i  = entry_build;
    @(posedge clk); #1;
    load_new_entry_i  = 1'b0;
    @(negedge clk);
    chk(entry_valid_o,      1'b1, "loaded: valid");
    chk(ready_to_execute_o, 1'b0, "loaded: not ready (acc busy, src empty)");

    // ------------------------------------------------------------------
    // Test 3: source buffers full but acc still busy → blocked
    // ------------------------------------------------------------------
    // buff4, buff5, buff7 full; colour=0 for all (matches entry colour=0)
    buffers_full_i   = 16'h00B0;    // bits 4,5,7 set
    buffers_colour_i = 16'h0000;    // all colour 0
    @(negedge clk);
    chk(ready_to_execute_o, 1'b0, "src full but acc busy: not ready");

    // ------------------------------------------------------------------
    // Test 4: free acc0 → all conditions met (buff9 free via FFFF)
    // ------------------------------------------------------------------
    acc_busy_i = 2'b00;
    @(negedge clk);
    chk(ready_to_execute_o, 1'b1, "all conditions met: ready");

    // ------------------------------------------------------------------
    // Test 5: target buffer busy → blocked
    // ------------------------------------------------------------------
    buffers_free_i = 16'hFDFF;   // bit 9 clear → buff9 not free
    @(negedge clk);
    chk(ready_to_execute_o, 1'b0, "tgt buff9 busy: not ready");
    buffers_free_i = 16'hFFFF;
    @(negedge clk);
    chk(ready_to_execute_o, 1'b1, "tgt buff9 free: ready");

    // ------------------------------------------------------------------
    // Test 6: RW slot – buff7 busy (full=0, free=0) → stalls entry.
    // RW only needs full+colour, not free.  Clearing full simulates the
    // buffer being claimed by another in-flight RW task.
    // ------------------------------------------------------------------
    buffers_full_i = 16'h0030;   // buff7 full cleared → RW buffer busy
    @(negedge clk);
    chk(ready_to_execute_o, 1'b0, "RW buff7 busy (full=0): not ready");
    buffers_full_i = 16'h00B0;   // restore buff7 full
    @(negedge clk);
    chk(ready_to_execute_o, 1'b1, "RW buff7 full again: ready");

    // RW must not require free: verify entry stays ready when buff7 free=0
    // (this is the normal in-use state: full=1, free=0).
    buffers_free_i = 16'hFF7F;   // buff7 free=0 (normal full-buffer state)
    @(negedge clk);
    chk(ready_to_execute_o, 1'b1, "RW buff7 free=0 (full still 1): still ready");

    // ------------------------------------------------------------------
    // Test 7: one source buffer not full → blocked
    // ------------------------------------------------------------------
    buffers_full_i = 16'h0090;   // buff4 and buff7 full; buff5 missing
    @(negedge clk);
    chk(ready_to_execute_o, 1'b0, "src buff5 not full: not ready");
    buffers_full_i = 16'h00B0;
    @(negedge clk);
    chk(ready_to_execute_o, 1'b1, "restored: ready");

    // ------------------------------------------------------------------
    // Test 8: colour mismatch on source buffer → blocked
    // Entry colour=0; set buff4's colour to 1 → mismatch
    // ------------------------------------------------------------------
    buffers_colour_i = 16'h0010;   // buff4 colour=1, rest=0
    @(negedge clk);
    chk(ready_to_execute_o, 1'b0, "colour mismatch buff4: not ready");

    buffers_colour_i = 16'h0000;   // restore
    @(negedge clk);
    chk(ready_to_execute_o, 1'b1, "colour match restored: ready");

    // ------------------------------------------------------------------
    // Test 9: colour mismatch on RW buffer → blocked
    // ------------------------------------------------------------------
    buffers_colour_i = 16'h0080;   // buff7 colour=1 (RW slot, needs full+colour)
    @(negedge clk);
    chk(ready_to_execute_o, 1'b0, "colour mismatch RW buff7: not ready");
    buffers_colour_i = 16'h0000;
    @(negedge clk);
    chk(ready_to_execute_o, 1'b1, "colour match RW buff7 restored: ready");

    // ------------------------------------------------------------------
    // Test 10: delete entry
    // ------------------------------------------------------------------
    @(posedge clk); #1;
    delete_entry_i = 1'b1;
    @(posedge clk); #1;
    delete_entry_i = 1'b0;
    @(negedge clk);
    chk(entry_valid_o,      1'b0, "deleted: valid=0");
    chk(ready_to_execute_o, 1'b0, "deleted: not ready");

    // ------------------------------------------------------------------
    // Test 11: shift entry in, verify data stored
    // ------------------------------------------------------------------
    @(posedge clk); #1;
    shift_entry_i          = 1'b1;
    shift_in_entry_valid_i = 1'b1;
    shift_in_entry_data_i  = entry_build;  // reuse last built entry
    @(posedge clk); #1;
    shift_entry_i          = 1'b0;
    @(negedge clk);
    chk(entry_valid_o,            1'b1,       "shifted: valid");
    chk(entry_data_o === entry_build, 1'b1,   "shifted: data matches");

    // ------------------------------------------------------------------
    // Test 12: shift in invalid entry clears valid
    // ------------------------------------------------------------------
    @(posedge clk); #1;
    shift_entry_i          = 1'b1;
    shift_in_entry_valid_i = 1'b0;
    @(posedge clk); #1;
    shift_entry_i          = 1'b0;
    @(negedge clk);
    chk(entry_valid_o, 1'b0, "shift-invalid: valid→0");

    // ------------------------------------------------------------------
    @(posedge clk);
    if (errors == 0)
        $display("PASS – sch_entry: all tests passed.");
    else
        $display("FAIL – sch_entry: %0d error(s).", errors);
    $finish;
end

initial begin
    #10000;
    $display("TIMEOUT – sch_entry");
    $finish;
end

endmodule
