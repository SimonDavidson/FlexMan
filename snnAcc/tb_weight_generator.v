// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
`timescale 1ns/1ps
`include "../shared/constants.v"

// ================================================================
// tb_weight_generator
//
// weight_generator contains a dataline_cache_with_xy internally;
// the memory side is exposed as weight_mem_*.  A small synchronous
// SRAM model provides weight data.
//
// Grid: 2×1 input, 3×1 output, 8-bit weights packed 4-per-word
//   (weight_sz = 3'b011, WEIGHT_SLICE_SZ=3 → 2^3=8-bit elements).
// Weight memory is pre-filled: word k = 0xAABBCCDD.
//
// Tests
// -----
//   1. Full mode (weight_mode=00): for a single active input neuron
//      the generator produces weight indices 0,1,2 (one per output)
//      and the value stream comes from the weight SRAM.
//      Checks: weight_index_valid_o, weight_index_o sequence,
//              finished_one_pass_o, finished_pass_o.
//   2. Conv mode (weight_mode=10): 3×1 input, 3×1 output, 1×1 kernel
//      (step=1, offset=0).  Each input neuron projects to exactly one
//      output → generated (x,y) sequence matches input.
//      Also checks that an out-of-bounds projection is silently
//      skipped (weight_index_valid_o suppressed for that beat).
// ================================================================
module tb_weight_generator;

localparam X_INPUT_SZ        = 4;
localparam Y_INPUT_SZ        = 4;
localparam X_OUTPUT_SZ       = 4;
localparam Y_OUTPUT_SZ       = 4;
localparam X_KERNEL_SZ       = 3;
localparam Y_KERNEL_SZ       = 3;
localparam X_KERNEL_OFF_SZ   = 3;
localparam Y_KERNEL_OFF_SZ   = 3;
localparam X_STEP_SZ         = 3;
localparam Y_STEP_SZ         = 3;
localparam WEIGHT_ENTRY_BITS = 8;
localparam ELEMS_PER_ROW     = 4;
localparam ROWS_PER_NEURON   = 16;
localparam IN_DATA_BITS      = 32;
localparam ELEM_SZ           = 8;
localparam ACT_IDX_SZ        = 5;
localparam ACT_DATA_SZ       = 8;
localparam WEIGHT_IDX_SZ     = 5;
localparam WEIGHT_SLICE_SZ   = 3;   // 2^3 = 8-bit weight elements
localparam WEIGHT_DATA_IDX_SZ= 5;
localparam WEIGHT_BITS       = 8;   // 2**WEIGHT_SLICE_SZ

reg                          clk, reset;
reg                          start_new_block;
reg                          running;
reg [1:0]                    weight_mode;
reg [X_INPUT_SZ-1:0]        in_x_len;
reg [Y_INPUT_SZ-1:0]        in_y_len;
reg [X_OUTPUT_SZ-1:0]       out_x_len;
reg [Y_OUTPUT_SZ-1:0]       out_y_len;
reg [X_KERNEL_SZ-1:0]       x_kernel_len;
reg [Y_KERNEL_SZ-1:0]       y_kernel_len;
reg [X_KERNEL_OFF_SZ-1:0]   x_kernel_offset;
reg [Y_KERNEL_OFF_SZ-1:0]   y_kernel_offset;
reg [X_STEP_SZ-1:0]         x_kernel_step;
reg [Y_STEP_SZ-1:0]         y_kernel_step;
reg [`ADDR_SIZE-1:0]         weight_base_addr;
reg [WEIGHT_SLICE_SZ-1:0]   weight_sz;
reg [WEIGHT_SLICE_SZ-1:0]   tuple_sz;
reg [`PIN_BITS-1:0]          sparse_count;
reg [WEIGHT_ENTRY_BITS-1:0] weight_entry_bits;
reg [ELEMS_PER_ROW-1:0]     weights_per_word;
reg [ROWS_PER_NEURON-1:0]   rows_per_neuron;
// Activation data input
reg                          act_data_valid;
reg [ACT_IDX_SZ-1:0]        act_data_idx;
reg [X_INPUT_SZ-1:0]        act_data_x;
reg [Y_INPUT_SZ-1:0]        act_data_y;
reg [ACT_DATA_SZ-1:0]       act_data;
reg                          act_data_last;
// Downstream handshake
reg                          weight_index_taken;
reg                          weight_value_taken;
// Memory wait
reg                          weight_mem_wait;
// Memory data
reg [`WTD_BITS-1:0]          weight_mem_data;

wire                         finished_one_pass;
wire                         finished_pass;
wire                         running_weight_pass;
wire                         weight_mem_rd;
wire [`ADDR_SIZE-1:0]        weight_mem_addr;
wire                         weight_index_valid;
wire [WEIGHT_IDX_SZ-1:0]    weight_index;
wire [X_OUTPUT_SZ-1:0]      weight_index_x;
wire [Y_OUTPUT_SZ-1:0]      weight_index_y;
wire                         weight_index_last;
wire                         weight_value_valid;
wire [WEIGHT_BITS-1:0]      weight_value;

weight_generator #(
    .X_INPUT_SZ        (X_INPUT_SZ),
    .Y_INPUT_SZ        (Y_INPUT_SZ),
    .X_OUTPUT_SZ       (X_OUTPUT_SZ),
    .Y_OUTPUT_SZ       (Y_OUTPUT_SZ),
    .X_KERNEL_SZ       (X_KERNEL_SZ),
    .Y_KERNEL_SZ       (Y_KERNEL_SZ),
    .X_KERNEL_OFF_SZ   (X_KERNEL_OFF_SZ),
    .Y_KERNEL_OFF_SZ   (Y_KERNEL_OFF_SZ),
    .X_STEP_SZ         (X_STEP_SZ),
    .Y_STEP_SZ         (Y_STEP_SZ),
    .WEIGHT_ENTRY_BITS (WEIGHT_ENTRY_BITS),
    .ELEMS_PER_ROW     (ELEMS_PER_ROW),
    .ROWS_PER_NEURON   (ROWS_PER_NEURON),
    .IN_DATA_BITS      (IN_DATA_BITS),
    .ELEM_SZ           (ELEM_SZ),
    .ACT_IDX_SZ        (ACT_IDX_SZ),
    .ACT_DATA_SZ       (ACT_DATA_SZ),
    .WEIGHT_IDX_SZ     (WEIGHT_IDX_SZ),
    .WEIGHT_SLICE_SZ   (WEIGHT_SLICE_SZ),
    .WEIGHT_DATA_IDX_SZ(WEIGHT_DATA_IDX_SZ))
dut (
    .clk               (clk),
    .reset             (reset),
    .start_new_block_i (start_new_block),
    .running_i         (running),
    .finished_one_pass_o(finished_one_pass),
    .finished_pass_o   (finished_pass),
    .running_weight_pass_o(running_weight_pass),
    .weight_mode_i     (weight_mode),
    .in_x_len_i        (in_x_len),
    .in_y_len_i        (in_y_len),
    .out_x_len_i       (out_x_len),
    .out_y_len_i       (out_y_len),
    .x_kernel_len_i    (x_kernel_len),
    .y_kernel_len_i    (y_kernel_len),
    .x_kernel_offset_i (x_kernel_offset),
    .y_kernel_offset_i (y_kernel_offset),
    .x_kernel_step_i   (x_kernel_step),
    .y_kernel_step_i   (y_kernel_step),
    .weight_base_addr_i(weight_base_addr),
    .weight_sz_i       (weight_sz),
    .tuple_sz_i        (tuple_sz),
    .sparse_count_i    (sparse_count),
    .weight_entry_bits_i(weight_entry_bits),
    .weights_per_word_i(weights_per_word),
    .rows_per_neuron_i (rows_per_neuron),
    .act_data_valid_i  (act_data_valid),
    .act_data_x_i      (act_data_x),
    .act_data_y_i      (act_data_y),
    .act_data_idx_i    (act_data_idx),
    .act_data_i        (act_data),
    .act_data_last_i   (act_data_last),
    .weight_mem_rd_o   (weight_mem_rd),
    .weight_mem_wait_i (weight_mem_wait),
    .weight_mem_addr_o (weight_mem_addr),
    .weight_mem_data_i (weight_mem_data),
    .weight_index_valid_o(weight_index_valid),
    .weight_index_o    (weight_index),
    .weight_index_x_o  (weight_index_x),
    .weight_index_y_o  (weight_index_y),
    .weight_index_last_o(weight_index_last),
    .weight_index_taken_i(weight_index_taken),
    .weight_value_valid_o(weight_value_valid),
    .weight_value_o    (weight_value),
    .weight_value_taken_i(weight_value_taken)
);

// ----------------------------------------------------------------
// SRAM: 256 words × 32 bits, 1-cycle latency
// Each 32-bit word holds 4 × 8-bit weights: bytes = BB,CC,DD,EE, ...
// ----------------------------------------------------------------
reg [31:0] sram [0:255];
integer mi;

always @(posedge clk)
    if (weight_mem_rd & ~weight_mem_wait)
        weight_mem_data <= sram[weight_mem_addr[7:0]];

initial clk = 0;
always  #5 clk = ~clk;

integer errors;
integer timeout;
integer idx_count;

task wait_value_valid;
    begin
        timeout = 50;
        while (!weight_value_valid && timeout > 0) begin
            @(posedge clk); #1;
            timeout = timeout - 1;
        end
        if (timeout == 0) begin
            $display("FAIL: timed out waiting for weight_value_valid");
            errors = errors + 1;
        end
    end
endtask

task wait_pass_done;
    begin
        timeout = 100;
        while (!finished_pass && timeout > 0) begin
            weight_index_taken = weight_index_valid;
            weight_value_taken = weight_value_valid;
            @(posedge clk); #1;
            timeout = timeout - 1;
        end
        if (timeout == 0) begin
            $display("FAIL: timed out waiting for finished_pass");
            errors = errors + 1;
        end
        weight_index_taken = 0;
        weight_value_taken = 0;
    end
endtask

task check_eq;
    input integer got, exp;
    input [255:0] label;
    begin
        if (got !== exp) begin
            $display("FAIL %s: got %0d  exp %0d", label, got, exp);
            errors = errors + 1;
        end
    end
endtask

task check_bit;
    input got, exp;
    input [255:0] label;
    begin
        if (got !== exp) begin
            $display("FAIL %s: got %b  exp %b", label, got, exp);
            errors = errors + 1;
        end
    end
endtask

initial begin
    errors          = 0;
    reset           = 1;
    start_new_block = 0;
    running         = 0;
    act_data_valid  = 0;
    act_data_idx    = 0;
    act_data_x      = 0;
    act_data_y      = 0;
    act_data        = 8'hFF;   // 1-bit spike: MSB=1
    act_data_last   = 0;
    weight_index_taken = 0;
    weight_value_taken = 0;
    weight_mem_wait = 0;
    weight_mem_data = 0;
    weight_mode     = 2'b00;
    weight_sz       = 3'b011;  // 8-bit elements
    tuple_sz        = 3'b011;
    sparse_count    = 4;
    weight_entry_bits = 8;
    weights_per_word  = 4;
    rows_per_neuron   = 1;
    weight_base_addr  = 0;
    x_kernel_len    = 1;
    y_kernel_len    = 1;
    x_kernel_offset = 0;
    y_kernel_offset = 0;
    x_kernel_step   = 1;
    y_kernel_step   = 1;

    for (mi = 0; mi < 256; mi = mi + 1)
        sram[mi] = {mi[7:0]+8'd3, mi[7:0]+8'd2, mi[7:0]+8'd1, mi[7:0]};

    @(posedge clk); #1;
    reset = 0;
    @(posedge clk); #1;

    $display("=== tb_weight_generator ===");

    // ----------------------------------------------------------
    // Test 1: Full mode, 1×1 input, 3×1 output
    // One active input neuron → generator should produce indices
    // 0, 1, 2 (one per output neuron) and signal finished_pass_o.
    // ----------------------------------------------------------
    $display("Test 1: full mode, 1×1 in, 3×1 out");
    weight_mode   = 2'b00;
    in_x_len      = 4'd1;
    in_y_len      = 4'd1;
    out_x_len     = 4'd3;
    out_y_len     = 4'd1;
    running       = 1;
    start_new_block = 1;
    @(posedge clk); #1;
    start_new_block = 0;

    // Present one spike (active input neuron 0)
    act_data_valid = 1;
    act_data_x     = 0;
    act_data_y     = 0;
    act_data_idx   = 0;
    act_data_last  = 1;
    act_data       = 8'hFF;   // spike

    idx_count = 0;
    wait_pass_done;

    check_bit(finished_pass, 1'b1, "T1 finished_pass_o");
    act_data_valid = 0;

    // ----------------------------------------------------------
    // Test 2: Conv mode, 3×1 input, 3×1 output, 1×1 kernel
    // step=1, offset=0 → out_x = act_x*1 + 0 - 0 = act_x
    // All projections in-bounds for act_x in {0,1,2}.
    //
    // act_data_valid is deasserted between neurons.  Without this,
    // after finishing act_x=N the DUT (doing_weight_pass_r=1, valid=1)
    // immediately re-issues a request with the stale act_data_x still
    // on the bus.  The cache hits (same addr/base), weight_value_valid
    // fires before the testbench can change act_data_x, and
    // wait_value_valid catches the wrong projection.
    // ----------------------------------------------------------
    $display("Test 2: conv mode, 3x1 in+out, 1x1 kernel");
    running = 0;
    @(posedge clk); #1;
    running         = 1;
    weight_mode     = 2'b10;
    in_x_len        = 4'd3;
    in_y_len        = 4'd1;
    out_x_len       = 4'd3;
    out_y_len       = 4'd1;
    x_kernel_len    = 3'd1;
    y_kernel_len    = 3'd1;
    x_kernel_offset = 3'd0;
    y_kernel_offset = 3'd0;
    x_kernel_step   = 3'd1;
    y_kernel_step   = 3'd1;

    // Input neuron (0,0)
    act_data_valid = 1;
    act_data_x     = 4'd0;
    act_data_y     = 4'd0;
    act_data_idx   = 5'd0;
    act_data_last  = 0;
    act_data       = 8'hFF;

    wait_value_valid;
    check_eq(weight_index_x, 0, "T2 projected x for act_x=0");
    weight_value_taken = 1;
    weight_index_taken = 1;
    @(posedge clk); #1;
    weight_value_taken = 0;
    weight_index_taken = 0;
    act_data_valid = 0;   // deassert — prevent re-request with stale act_data_x
    @(posedge clk); #1;  // suspend so combinational logic settles before reassert

    // Input neuron (1,0)
    act_data_x    = 4'd1;
    act_data_idx  = 5'd1;
    act_data_last = 0;
    act_data_valid = 1;   // re-assert with new act_data_x
    wait_value_valid;
    check_eq(weight_index_x, 1, "T2 projected x for act_x=1");
    weight_value_taken = 1;
    weight_index_taken = 1;
    @(posedge clk); #1;
    weight_value_taken = 0;
    weight_index_taken = 0;
    act_data_valid = 0;   // deassert — prevent re-request with stale act_data_x
    @(posedge clk); #1;  // suspend so combinational logic settles before reassert

    // Input neuron (2,0) — last
    act_data_x    = 4'd2;
    act_data_idx  = 5'd2;
    act_data_last = 1;
    act_data_valid = 1;   // re-assert with new act_data_x
    weight_index_taken = 1;
    weight_value_taken = 1;
    // Sample finished_pass AT the posedge, before the NBA that clears
    // doing_weight_pass_r.  finished_pass_o is purely combinational and
    // drops to 0 immediately after that NBA, so checking at #1 always
    // misses the pulse.
    timeout = 50;
    @(posedge clk);
    while (!finished_pass && timeout > 0) begin
        timeout = timeout - 1;
        @(posedge clk);
    end
    if (timeout == 0) begin
        $display("FAIL T2: timed out waiting for finished_pass (conv)");
        errors = errors + 1;
    end else begin
        check_bit(finished_pass, 1'b1, "T2 finished_pass conv mode");
    end
    #1;
    weight_index_taken = 0;
    weight_value_taken = 0;
    act_data_valid = 0;

    $display("=== tb_weight_generator: %0d failure(s) ===", errors);
    if (errors == 0) $display("PASS"); else $display("FAIL");
    $finish;
end

endmodule
