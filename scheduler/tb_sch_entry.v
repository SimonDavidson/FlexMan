// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
`timescale 1ns/1ps

// Testbench for sch_entry.v
//
// Uses the default parameter TGT_ACC_SZ=2.  With this setting sch_entry
// decodes the accelerator ID from entry_data_o[1:0], which overlaps with
// the ENTRY_SBUFF1 field [3:0].  The entry encoding used below is chosen
// so that src1_buff_id[1:0] also encodes the desired accelerator ID:
//
//   To target acc0: src1 must have bits[1:0]==2'b00  (e.g. buff 4, 8, 12)
//   To target acc1: src1 must have bits[1:0]==2'b01  (e.g. buff 1, 5, 9)
//
// Entry A used in tests:
//   src1 = buff 4 (0100) → acc0 (bits[1:0]=00)
//   src2 = buff 5 (0101)
//   src3 = buff 0 (0000)
//   tgt  = buff 7 (0111)
//   entry_data = 32'h0000_7054

module top;

localparam SCH_ENTRY_SZ        = 32;
localparam NUM_HW_ACCELERATORS = 2;
localparam TGT_ACC_SZ          = 2;   // default – acc check uses bits[1:0]
localparam NUM_BUFFERS         = 16;
localparam COL_BUFF_ID_SZ      = 16;

localparam ENTRY_A = 32'h0000_7054;   // src1=4,src2=5,src3=0,tgt=7 → acc0

reg        clk, reset;
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

task chk32;
    input [31:0] got;
    input [31:0] exp;
    input [255:0] label;
    begin
        if (got !== exp) begin
            $display("FAIL [%0t] %0s: got=%0h exp=%0h", $time, label, got, exp);
            errors = errors + 1;
        end
    end
endtask

reg        load_new_entry_i;
reg        shift_entry_i;
reg        delete_entry_i;
reg        new_entry_valid_i;
reg [31:0] new_entry_data_i;
reg        shift_in_entry_valid_i;
reg [31:0] shift_in_entry_data_i;
reg  [NUM_HW_ACCELERATORS-1:0] acc_busy_i;
reg  [COL_BUFF_ID_SZ-1:0]      buffers_full_i;
reg  [COL_BUFF_ID_SZ-1:0]      buffers_free_i;

wire       shift_out_entry_valid_o;
wire       entry_valid_o;
wire [31:0] entry_data_o;
wire        ready_to_execute_o;

sch_entry #(
    .SCH_ENTRY_SZ(SCH_ENTRY_SZ),
    .NUM_HW_ACCELERATORS(NUM_HW_ACCELERATORS),
    .TGT_ACC_SZ(TGT_ACC_SZ),
    .NUM_BUFFERS(NUM_BUFFERS),
    .COL_BUFF_ID_SZ(COL_BUFF_ID_SZ)
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
    .ready_to_execute_o(ready_to_execute_o)
);

initial begin
    reset                  = 1'b1;
    load_new_entry_i       = 1'b0;
    shift_entry_i          = 1'b0;
    delete_entry_i         = 1'b0;
    new_entry_valid_i      = 1'b1;
    new_entry_data_i       = 32'h0;
    shift_in_entry_valid_i = 1'b0;
    shift_in_entry_data_i  = 32'h0;
    acc_busy_i             = 2'b11;   // all accs busy
    buffers_full_i         = 16'h0000;
    buffers_free_i         = 16'hFFFF;

    repeat(3) @(posedge clk); #1;
    reset = 1'b0;
    @(negedge clk);

    // ------------------------------------------------------------------
    // Test 1: empty entry – never ready
    // ------------------------------------------------------------------
    chk(entry_valid_o,      1'b0, "empty: valid=0");
    chk(ready_to_execute_o, 1'b0, "empty: not ready");

    // ------------------------------------------------------------------
    // Test 2: load entry – conditions not met (acc busy, src buffs empty)
    // ------------------------------------------------------------------
    @(posedge clk); #1;
    load_new_entry_i  = 1'b1;
    new_entry_data_i  = ENTRY_A;
    new_entry_valid_i = 1'b1;
    @(posedge clk); #1;
    load_new_entry_i  = 1'b0;
    @(negedge clk);
    chk(entry_valid_o,      1'b1, "loaded: valid=1");
    chk(ready_to_execute_o, 1'b0, "loaded: not ready (acc busy, no src)");

    // ------------------------------------------------------------------
    // Test 3: provide src buffers full but acc still busy → still blocked
    // ------------------------------------------------------------------
    buffers_full_i = 16'h0030;   // bits 4 and 5 set (src1=buff4, src2=buff5)
    @(negedge clk);
    chk(ready_to_execute_o, 1'b0, "src full but acc busy: not ready");

    // ------------------------------------------------------------------
    // Test 4: free acc0 → all conditions met (tgt buff7 free via FFFF)
    // ------------------------------------------------------------------
    buffers_free_i = 16'hFFFF;
    acc_busy_i     = 2'b00;
    @(negedge clk);
    chk(ready_to_execute_o, 1'b1, "all conditions met: ready");

    // ------------------------------------------------------------------
    // Test 5: target buffer busy → blocked
    // ------------------------------------------------------------------
    buffers_free_i = 16'hFF7F;   // bit 7 cleared → tgt buff7 not free
    @(negedge clk);
    chk(ready_to_execute_o, 1'b0, "tgt busy: not ready");
    buffers_free_i = 16'hFFFF;   // restore
    @(negedge clk);
    chk(ready_to_execute_o, 1'b1, "tgt free again: ready");

    // ------------------------------------------------------------------
    // Test 6: one src buffer disappears → blocked
    // ------------------------------------------------------------------
    buffers_full_i = 16'h0010;   // only buff4 full; buff5 empty
    @(negedge clk);
    chk(ready_to_execute_o, 1'b0, "src2 not full: not ready");
    buffers_full_i = 16'h0030;   // restore
    @(negedge clk);
    chk(ready_to_execute_o, 1'b1, "src2 restored: ready");

    // ------------------------------------------------------------------
    // Test 7: delete entry
    // ------------------------------------------------------------------
    @(posedge clk); #1;
    delete_entry_i = 1'b1;
    @(posedge clk); #1;
    delete_entry_i = 1'b0;
    @(negedge clk);
    chk(entry_valid_o,      1'b0, "deleted: valid=0");
    chk(ready_to_execute_o, 1'b0, "deleted: not ready");

    // ------------------------------------------------------------------
    // Test 8: shift entry in
    // ------------------------------------------------------------------
    @(posedge clk); #1;
    shift_entry_i          = 1'b1;
    shift_in_entry_valid_i = 1'b1;
    shift_in_entry_data_i  = 32'hDEAD_BEEF;
    @(posedge clk); #1;
    shift_entry_i          = 1'b0;
    @(negedge clk);
    chk(entry_valid_o, 1'b1, "shifted: valid=1");
    chk32(entry_data_o, 32'hDEAD_BEEF, "shifted: data correct");

    // ------------------------------------------------------------------
    // Test 9: shift in invalid entry clears valid
    // ------------------------------------------------------------------
    @(posedge clk); #1;
    shift_entry_i          = 1'b1;
    shift_in_entry_valid_i = 1'b0;
    shift_in_entry_data_i  = 32'h0;
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
