// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
`timescale 1ns/1ps
`include "../shared/constants.v"

// ================================================================
// tb_act_index_generator
//
// Tests the activation index generator in full-connectivity mode,
// which sweeps a 2-D input grid in row-major order (x inner loop,
// y outer loop).
//
// Parameters: 3×2 grid (in_x_len=3, in_y_len=2) → 6 indices.
// Expected (x,y,flat) sequence:
//   (0,0,0) (1,0,1) (2,0,2) (0,1,3) (1,1,4) (2,1,5)
//
// Tests
// -----
//   1. Full-mode sequential scan: verify (x,y) and flat index at
//      each step when act_index_taken_i=1 every cycle.
//   2. act_index_last_o fires only on the final element (2,1).
//   3. finished_timestep_o pulses for one cycle when the last index
//      is taken, then act_index_gen_running_o goes low.
//   4. Back-pressure: hold act_index_taken_i=0 for two cycles →
//      generator stays on the same index.
// ================================================================
module tb_act_index_generator;

localparam X_INPUT_SZ    = 4;
localparam Y_INPUT_SZ    = 4;
localparam X_KERNEL_SZ   = 3;
localparam Y_KERNEL_SZ   = 3;
localparam X_KERNEL_OFF  = 3;
localparam Y_KERNEL_OFF  = 3;
localparam X_STEP_SZ     = 3;
localparam Y_STEP_SZ     = 3;
localparam ELEMS_PER_ROW = 4;
localparam ROWS_PER_NEURON = 16;
localparam IN_DATA_BITS  = 32;
localparam ELEM_SZ       = 8;
localparam ACT_IDX_SZ    = `COL_BITS;

reg                       clk, reset;
reg                       start_new_block;
reg                       running;
reg                       next_in_neuron;
reg [1:0]                 weight_mode;
reg [X_INPUT_SZ-1:0]     in_x_len;
reg [Y_INPUT_SZ-1:0]     in_y_len;
reg [X_KERNEL_SZ-1:0]    x_kernel_len;
reg [Y_KERNEL_SZ-1:0]    y_kernel_len;
reg [X_STEP_SZ-1:0]      x_kernel_step;
reg [Y_STEP_SZ-1:0]      y_kernel_step;
reg [ELEMS_PER_ROW-1:0]  weights_per_word;
reg [ROWS_PER_NEURON-1:0] rows_per_neuron;
reg                       act_index_taken;

wire                      finished_timestep;
wire                      act_index_gen_running;
wire                      act_index_valid;
wire [X_INPUT_SZ-1:0]    act_index_x;
wire [Y_INPUT_SZ-1:0]    act_index_y;
wire [ACT_IDX_SZ-1:0]    act_index;
wire                      act_index_last;

act_index_generator #(
    .X_INPUT_SZ    (X_INPUT_SZ),
    .Y_INPUT_SZ    (Y_INPUT_SZ),
    .X_KERNEL_SZ   (X_KERNEL_SZ),
    .Y_KERNEL_SZ   (Y_KERNEL_SZ),
    .X_KERNEL_OFF_SZ(X_KERNEL_OFF),
    .Y_KERNEL_OFF_SZ(Y_KERNEL_OFF),
    .X_STEP_SZ     (X_STEP_SZ),
    .Y_STEP_SZ     (Y_STEP_SZ),
    .ELEMS_PER_ROW (ELEMS_PER_ROW),
    .ROWS_PER_NEURON(ROWS_PER_NEURON),
    .IN_DATA_BITS  (IN_DATA_BITS),
    .ELEM_SZ       (ELEM_SZ),
    .ACT_IDX_SZ    (ACT_IDX_SZ))
dut (
    .clk                  (clk),
    .reset                (reset),
    .start_new_block_i    (start_new_block),
    .running_i            (running),
    .next_in_neuron_i     (next_in_neuron),
    .finished_timestep_o  (finished_timestep),
    .act_index_gen_running_o(act_index_gen_running),
    .weight_mode_i        (weight_mode),
    .in_x_len_i           (in_x_len),
    .in_y_len_i           (in_y_len),
    .x_kernel_len_i       (x_kernel_len),
    .y_kernel_len_i       (y_kernel_len),
    .x_kernel_step_i      (x_kernel_step),
    .y_kernel_step_i      (y_kernel_step),
    .weights_per_word_i   (weights_per_word),
    .rows_per_neuron_i    (rows_per_neuron),
    .act_index_valid_o    (act_index_valid),
    .act_index_x_o        (act_index_x),
    .act_index_y_o        (act_index_y),
    .act_index_o          (act_index),
    .act_index_last_o     (act_index_last),
    .act_index_taken_i    (act_index_taken)
);

initial clk = 0;
always  #5 clk = ~clk;

integer errors, i;

// Expected full-mode (x,y) sequence for a 3×2 grid
integer exp_x [0:5];
integer exp_y [0:5];
initial begin
    exp_x[0]=0; exp_y[0]=0;
    exp_x[1]=1; exp_y[1]=0;
    exp_x[2]=2; exp_y[2]=0;
    exp_x[3]=0; exp_y[3]=1;
    exp_x[4]=1; exp_y[4]=1;
    exp_x[5]=2; exp_y[5]=1;
end

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
    next_in_neuron  = 0;
    weight_mode     = 2'b00;   // full connectivity
    in_x_len        = 4'd3;
    in_y_len        = 4'd2;
    x_kernel_len    = 1;
    y_kernel_len    = 1;
    x_kernel_step   = 1;
    y_kernel_step   = 1;
    weights_per_word = 1;
    rows_per_neuron  = 1;
    act_index_taken  = 0;

    @(posedge clk); #1;
    reset   = 0;
    running = 1;
    @(posedge clk); #1;

    $display("=== tb_act_index_generator ===");

    // ----------------------------------------------------------
    // Test 1 + 2: Sequential scan (act_index_taken=1 every cycle)
    // ----------------------------------------------------------
    $display("Test 1-2: sequential scan, 3×2 grid");
    start_new_block = 1;
    @(posedge clk); #1;
    start_new_block = 0;
    act_index_taken = 1;

    for (i = 0; i < 6; i = i + 1) begin
        // Wait until valid
        if (!act_index_valid) begin
            @(posedge clk); #1;
        end
        check_eq(act_index_x, exp_x[i], "T1 act_index_x");
        check_eq(act_index_y, exp_y[i], "T1 act_index_y");
        check_eq(act_index,   i,        "T1 act_index (flat)");
        // last should be high only on the final element
        check_bit(act_index_last, (i == 5) ? 1'b1 : 1'b0, "T2 act_index_last");
        @(posedge clk); #1;
    end

    // ----------------------------------------------------------
    // Test 3: finished_timestep_o and running going low
    // ----------------------------------------------------------
    $display("Test 3: finished_timestep_o and generator halt");
    // finished_timestep fires on the same cycle as the last taken index.
    // After the loop above the generator should have finished.
    check_bit(act_index_gen_running, 1'b0, "T3 generator not running after completion");

    // ----------------------------------------------------------
    // Test 4: Back-pressure — hold taken=0
    // ----------------------------------------------------------
    $display("Test 4: back-pressure");
    act_index_taken = 0;
    running         = 1;
    start_new_block = 1;
    @(posedge clk); #1;
    start_new_block = 0;
    @(posedge clk); #1;  // generator starts, index=(0,0)

    // Hold taken=0 for two more cycles — index must not advance
    @(posedge clk); #1;
    check_eq(act_index_x, 0, "T4 x stays 0 with taken=0");
    check_eq(act_index_y, 0, "T4 y stays 0 with taken=0");
    @(posedge clk); #1;
    check_eq(act_index_x, 0, "T4 x still 0 after stall");

    // Release taken — generator advances
    act_index_taken = 1;
    @(posedge clk); #1;
    check_eq(act_index_x, 1, "T4 x advances to 1 after release");

    $display("=== tb_act_index_generator: %0d failure(s) ===", errors);
    if (errors == 0) $display("PASS"); else $display("FAIL");
    $finish;
end

endmodule
