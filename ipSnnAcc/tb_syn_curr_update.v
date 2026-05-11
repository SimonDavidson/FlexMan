// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
`timescale 1ns/1ps
`include "../shared/constants.v"

// ================================================================
// tb_syn_curr_update
//
// Tests the read-modify-write pipeline for synaptic current
// accumulation.  One weight landing on output neuron (x,y) causes:
//   cycle T  : weight_index_valid=1 & weight_value_valid=1
//              → rd=1, address latched into req_pending
//   cycle T+1: req_pending=1 → wr=1
//              data_o = sram[addr] + sign_extended(weight)
//              written back to sram[addr]
//
// SRAM model: 256 words × 32 bits, 1-cycle read latency.
// Weight values are sign-extended from WEIGHT_BITS to 32 bits.
//
// Tests
// -----
//   1. Full mode (weight_mode=00): positive weight accumulates
//      correctly into the target neuron's syn_curr word.
//   2. Full mode: negative weight (sign-extended) subtracts correctly.
//   3. Sparse mode (weight_mode=01): sparse_index_i drives the
//      syn_curr address directly, ignoring (x,y) projection.
//   4. Memory stall (syn_curr_mem_wait_i=1): req_pending held,
//      wr not committed until wait de-asserts.
// ================================================================
module tb_syn_curr_update;

localparam X_OUTPUT_SZ        = 4;
localparam Y_OUTPUT_SZ        = 4;
localparam IN_DATA_BITS       = 32;
localparam WEIGHT_IDX_SZ      = 5;
localparam WEIGHT_SLICE_SZ    = 5;   // 2^5 = 32-bit weights
localparam WEIGHT_DATA_IDX_SZ = 5;
localparam SPARSE_IDX_SZ      = 8;
localparam WEIGHT_BITS        = 32;  // 2**WEIGHT_SLICE_SZ
localparam SRAM_DEPTH         = 256;

reg                          clk, reset;
reg                          start_new_block;
reg                          running;
reg                          finished_pass_weight;
reg [1:0]                    weight_mode;
reg [SPARSE_IDX_SZ-1:0]     sparse_index;
reg [`ADDR_SIZE-1:0]         syn_curr_base;
reg [X_OUTPUT_SZ-1:0]        out_x_len;
reg                          weight_index_valid;
reg [WEIGHT_IDX_SZ-1:0]      weight_index;
reg [X_OUTPUT_SZ-1:0]        weight_index_x;
reg [Y_OUTPUT_SZ-1:0]        weight_index_y;
reg                          weight_index_last;
reg                          weight_value_valid;
reg [WEIGHT_BITS-1:0]        weight_value;
reg                          syn_curr_mem_wait;

wire                         finished_pass;
wire                         syn_curr_update_running;
wire                         syn_curr_mem_rd;
wire                         syn_curr_mem_wr;
wire [`ADDR_SIZE-1:0]        syn_curr_mem_addr;
wire [`WTD_BITS-1:0]         syn_curr_mem_data_o;
reg  [`WTD_BITS-1:0]         syn_curr_mem_data_i;
wire                         weight_index_taken;
wire                         weight_value_taken;
reg  [7:0]                   act_value;

syn_curr_update #(
    .X_OUTPUT_SZ       (X_OUTPUT_SZ),
    .Y_OUTPUT_SZ       (Y_OUTPUT_SZ),
    .IN_DATA_BITS      (IN_DATA_BITS),
    .WEIGHT_IDX_SZ     (WEIGHT_IDX_SZ),
    .WEIGHT_SLICE_SZ   (WEIGHT_SLICE_SZ),
    .WEIGHT_DATA_IDX_SZ(WEIGHT_DATA_IDX_SZ),
    .SPARSE_IDX_SZ     (SPARSE_IDX_SZ))
dut (
    .clk                     (clk),
    .reset                   (reset),
    .start_new_block_i       (start_new_block),
    .running_i               (running),
    .finished_pass_weight_i  (finished_pass_weight),
    .finished_pass_o         (finished_pass),
    .syn_curr_update_running_o(syn_curr_update_running),
    .weight_mode_i           (weight_mode),
    .sparse_index_i          (sparse_index),
    .syn_curr_base_addr_i    (syn_curr_base),
    .out_x_len_i             (out_x_len),
    .weight_index_valid_i    (weight_index_valid),
    .weight_index_i          (weight_index),
    .weight_index_x_i        (weight_index_x),
    .weight_index_y_i        (weight_index_y),
    .weight_index_last_i     (weight_index_last),
    .weight_index_taken_o    (weight_index_taken),
    .weight_value_valid_i    (weight_value_valid),
    .weight_value_i          (weight_value),
    .weight_value_taken_o    (weight_value_taken),
    .syn_curr_mem_rd_o       (syn_curr_mem_rd),
    .syn_curr_mem_wr_o       (syn_curr_mem_wr),
    .syn_curr_mem_wait_i     (syn_curr_mem_wait),
    .syn_curr_mem_addr_o     (syn_curr_mem_addr),
    .syn_curr_mem_data_i     (syn_curr_mem_data_i),
    .syn_curr_mem_data_o     (syn_curr_mem_data_o),
    .act_value_i             (act_value)
);

// ----------------------------------------------------------------
// Synchronous SRAM model (1-cycle read latency, immediate write)
// ----------------------------------------------------------------
reg [31:0] sram [0:SRAM_DEPTH-1];
integer mi;

always @(posedge clk) begin
    if (syn_curr_mem_rd & ~syn_curr_mem_wait)
        syn_curr_mem_data_i <= sram[syn_curr_mem_addr[7:0]];
    if (syn_curr_mem_wr & ~syn_curr_mem_wait)
        sram[syn_curr_mem_addr[7:0]] <= syn_curr_mem_data_o;
end

initial clk = 0;
always  #5 clk = ~clk;

integer errors;
integer timeout;

task wait_wr_done;
    // Sample weight_index_taken AT the posedge (before NBA clears req_pending_r),
    // not at #1 after it.  The NBA simultaneously commits the write and drops
    // pending to 0; sampling after #1 always sees taken=0 on the commit cycle.
    begin
        timeout = 20;
        @(posedge clk);
        while (!weight_index_taken && timeout > 0) begin
            timeout = timeout - 1;
            @(posedge clk);
        end
        if (timeout == 0) begin
            $display("FAIL: timed out waiting for weight_index_taken");
            errors = errors + 1;
        end
        #1;
    end
endtask

task check_eq;
    input [31:0] got, exp;
    input [255:0] label;
    begin
        if (got !== exp) begin
            $display("FAIL %s: got %0d (0x%08h)  exp %0d (0x%08h)",
                     label, got, got, exp, exp);
            errors = errors + 1;
        end
    end
endtask

initial begin
    errors              = 0;
    reset               = 1;
    start_new_block     = 0;
    running             = 0;
    finished_pass_weight = 0;
    weight_mode         = 2'b00;
    sparse_index        = 0;
    syn_curr_base       = 0;
    out_x_len           = X_OUTPUT_SZ;
    weight_index_valid  = 0;
    weight_index        = 0;
    weight_index_x      = 0;
    weight_index_y      = 0;
    weight_index_last   = 0;
    weight_value_valid  = 0;
    weight_value        = 0;
    act_value           = 8'd1;   // act=1 preserves original test expectations
    syn_curr_mem_wait   = 0;
    syn_curr_mem_data_i = 0;

    // Initialise SRAM with known values
    for (mi = 0; mi < SRAM_DEPTH; mi = mi + 1)
        sram[mi] = mi * 4;   // sram[k] = 4k

    @(posedge clk); #1;
    reset   = 0;
    running = 1;
    @(posedge clk); #1;   // syn_curr_update_running_r becomes 1

    $display("=== tb_syn_curr_update ===");

    // ----------------------------------------------------------
    // Test 1: Positive weight, full mode, neuron at (x=1, y=0)
    // syn_curr address = base + y*out_x_len + x = 0 + 0*4 + 1 = 1
    // sram[1] initial = 4, weight = 10
    // expected result = 14
    // ----------------------------------------------------------
    $display("Test 1: positive weight, full mode");
    weight_mode        = 2'b00;
    weight_index_valid = 1;
    weight_value_valid = 1;
    weight_index_x     = 4'd1;
    weight_index_y     = 4'd0;
    weight_index       = 5'd1;
    weight_index_last  = 1;
    weight_value       = 32'd10;   // positive weight

    wait_wr_done;
    weight_index_valid = 0;
    weight_value_valid = 0;
    @(posedge clk); #1;
    check_eq(sram[1], 32'd14, "T1 sram[1] after positive weight");

    // ----------------------------------------------------------
    // Test 2: Negative weight (sign-extended), full mode
    // neuron at (x=2, y=0) → address = 2
    // sram[2] initial = 8, weight = -3 (32-bit signed)
    // expected result = 5
    // ----------------------------------------------------------
    $display("Test 2: negative weight, full mode");
    running = 0;
    @(posedge clk); #1;
    running = 1;
    @(posedge clk); #1;

    weight_mode        = 2'b00;
    weight_index_valid = 1;
    weight_value_valid = 1;
    weight_index_x     = 4'd2;
    weight_index_y     = 4'd0;
    weight_index       = 5'd2;
    weight_index_last  = 1;
    weight_value       = 32'hFFFF_FFFD;   // -3 in signed 32-bit

    wait_wr_done;
    weight_index_valid = 0;
    weight_value_valid = 0;
    @(posedge clk); #1;
    check_eq(sram[2], 32'd5, "T2 sram[2] after negative weight (-3)");

    // ----------------------------------------------------------
    // Test 3: Sparse mode — sparse_index drives the address
    // sparse_index = 7 → sram[7] = 28 initially, weight = 5
    // expected result = 33
    // ----------------------------------------------------------
    $display("Test 3: sparse mode");
    running = 0;
    @(posedge clk); #1;
    running = 1;
    @(posedge clk); #1;

    weight_mode        = 2'b01;
    sparse_index       = 8'd7;
    weight_index_valid = 1;
    weight_value_valid = 1;
    weight_index_x     = 0;
    weight_index_y     = 0;
    weight_index       = 5'd0;
    weight_index_last  = 1;
    weight_value       = 32'd5;

    wait_wr_done;
    weight_index_valid = 0;
    weight_value_valid = 0;
    @(posedge clk); #1;
    check_eq(sram[7], 32'd33, "T3 sram[7] sparse mode");

    // ----------------------------------------------------------
    // Test 4: Memory stall — assert mem_wait for 2 cycles
    // neuron at (x=3, y=0) → address = 3, sram[3]=12, weight=1
    // expected result = 13
    // ----------------------------------------------------------
    $display("Test 4: memory stall");
    running = 0;
    @(posedge clk); #1;
    running = 1;
    @(posedge clk); #1;

    weight_mode        = 2'b00;
    weight_index_valid = 1;
    weight_value_valid = 1;
    weight_index_x     = 4'd3;
    weight_index_y     = 4'd0;
    weight_index       = 5'd3;
    weight_index_last  = 1;
    weight_value       = 32'd1;
    // Stall memory for 2 cycles once the write is about to happen
    @(posedge clk); #1;   // rd cycle passes
    syn_curr_mem_wait = 1;
    @(posedge clk); #1;
    @(posedge clk); #1;
    syn_curr_mem_wait = 0;

    wait_wr_done;
    weight_index_valid = 0;
    weight_value_valid = 0;
    @(posedge clk); #1;
    check_eq(sram[3], 32'd13, "T4 sram[3] after stalled write");

    // ----------------------------------------------------------
    // Test 5: MAC — act=2, weight=5, full mode
    // neuron (x=0,y=1) → address=4, sram[4]=16 initially
    // expected = 16 + 2*5 = 26
    // ----------------------------------------------------------
    $display("Test 5: MAC act=2 weight=5");
    running = 0;
    @(posedge clk); #1;
    running = 1;
    act_value = 8'd2;
    @(posedge clk); #1;

    weight_mode        = 2'b00;
    weight_index_valid = 1;
    weight_value_valid = 1;
    weight_index_x     = 4'd0;
    weight_index_y     = 4'd1;
    weight_index       = 5'd4;
    weight_index_last  = 1;
    weight_value       = 32'd5;

    wait_wr_done;
    weight_index_valid = 0;
    weight_value_valid = 0;
    @(posedge clk); #1;
    check_eq(sram[4], 32'd26, "T5 sram[4] MAC act=2 weight=5");

    // ----------------------------------------------------------
    // Test 6: MAC — act=4, weight=-3, full mode
    // neuron (x=1,y=1) → address=5, sram[5]=20 initially
    // expected = 20 + 4*(-3) = 8
    // ----------------------------------------------------------
    $display("Test 6: MAC act=4 weight=-3");
    running = 0;
    @(posedge clk); #1;
    running = 1;
    act_value = 8'd4;
    @(posedge clk); #1;

    weight_mode        = 2'b00;
    weight_index_valid = 1;
    weight_value_valid = 1;
    weight_index_x     = 4'd1;
    weight_index_y     = 4'd1;
    weight_index       = 5'd5;
    weight_index_last  = 1;
    weight_value       = 32'hFFFF_FFFD;   // -3 signed

    wait_wr_done;
    weight_index_valid = 0;
    weight_value_valid = 0;
    @(posedge clk); #1;
    check_eq(sram[5], 32'd8, "T6 sram[5] MAC act=4 weight=-3");

    $display("=== tb_syn_curr_update: %0d failure(s) ===", errors);
    if (errors == 0) $display("PASS"); else $display("FAIL");
    $finish;
end

endmodule
