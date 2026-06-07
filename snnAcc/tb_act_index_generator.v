// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
// =============================================================================
// tb_act_index_generator  (snnAcc)
//
// Authors      : Simon Davidson & Claude
// Created      : 2026-05-07
// Last modified: 2026-06-07
//
// Aggressive rewrite using the shared verif library.  act_index_generator
// sweeps a 2-D input grid in row-major order (x inner, y outer), emitting a flat
// element index alongside (x,y).  The handshake is valid/taken; act_index_last_o
// flags the final element; finished_timestep_o pulses when that final index is
// taken and act_index_gen_running_o then drops.
//
// Software golden: for an in_x_len * in_y_len grid the k-th emitted element is
//     x = k % in_x_len,  y = k / in_x_len,  flat = k
// for k = 0 .. in_x_len*in_y_len - 1, with last on the final k.
//
// Strategy: drive several grid sizes, consume each index with a random
// taken-pattern (back-pressure), and check (x, y, flat, last) at every beat,
// plus finished_timestep_o / running-drop at the end.
// =============================================================================
`timescale 1ns/1ps
`include "../shared/constants.v"

module tb_act_index_generator;

    localparam X_INPUT_SZ    = 5;
    localparam Y_INPUT_SZ    = 5;
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

    integer verif_errors, verif_checks, verif_to;
    `include "../verif/checks.vh"
    `include "../verif/vt_driver.vh"

    reg                       clk, reset;
    reg                       start_new_block;
    reg                       running;
    reg                       next_in_neuron;
    reg [1:0]                 weight_mode;
    reg [X_INPUT_SZ-1:0]      in_x_len;
    reg [Y_INPUT_SZ-1:0]      in_y_len;
    reg [X_KERNEL_SZ-1:0]     x_kernel_len;
    reg [Y_KERNEL_SZ-1:0]     y_kernel_len;
    reg [X_STEP_SZ-1:0]       x_kernel_step;
    reg [Y_STEP_SZ-1:0]       y_kernel_step;
    reg [ELEMS_PER_ROW-1:0]   weights_per_word;
    reg [ROWS_PER_NEURON-1:0] rows_per_neuron;
    reg                       act_index_taken;

    wire                      finished_timestep;
    wire                      act_index_gen_running;
    wire                      act_index_valid;
    wire [X_INPUT_SZ-1:0]     act_index_x;
    wire [Y_INPUT_SZ-1:0]     act_index_y;
    wire [ACT_IDX_SZ-1:0]     act_index;
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
        .clk(clk), .reset(reset),
        .start_new_block_i(start_new_block),
        .running_i(running),
        .next_in_neuron_i(next_in_neuron),
        .finished_timestep_o(finished_timestep),
        .act_index_gen_running_o(act_index_gen_running),
        .weight_mode_i(weight_mode),
        .in_x_len_i(in_x_len),
        .in_y_len_i(in_y_len),
        .x_kernel_len_i(x_kernel_len),
        .y_kernel_len_i(y_kernel_len),
        .x_kernel_step_i(x_kernel_step),
        .y_kernel_step_i(y_kernel_step),
        .weights_per_word_i(weights_per_word),
        .rows_per_neuron_i(rows_per_neuron),
        .act_index_valid_o(act_index_valid),
        .act_index_x_o(act_index_x),
        .act_index_y_o(act_index_y),
        .act_index_o(act_index),
        .act_index_last_o(act_index_last),
        .act_index_taken_i(act_index_taken));

    initial clk = 0; always #5 clk = ~clk;

    // ---- Run one full sweep of an (xlen,ylen) grid with a random taken
    //      pattern, checking every emitted index against the row-major golden.
    task run_grid;
        input [X_INPUT_SZ-1:0] xlen;
        input [Y_INPUT_SZ-1:0] ylen;
        input integer          bp_pct;   // % chance of a back-pressure cycle
        integer total, k, gx, gy;
        begin
            in_x_len = xlen;
            in_y_len = ylen;
            total    = xlen * ylen;

            // restart the generator
            running = 0; act_index_taken = 0;
            @(posedge clk); #1;
            running = 1;
            `VT_PULSE(start_new_block)

            for (k = 0; k < total; k = k + 1) begin
                gx = k % xlen;
                gy = k / xlen;
                // wait for a valid index (sweep is steady-state once running)
                verif_to = 50;
                while (!act_index_valid && verif_to > 0) begin
                    @(posedge clk); #1; verif_to = verif_to - 1;
                end
                // optionally hold taken=0 to back-pressure; index must not move
                act_index_taken = 0;
                while ($urandom_range(99) < bp_pct) begin
                    @(posedge clk); #1;
                    check_eq_u(act_index_x, gx, "act bp x held");
                    check_eq_u(act_index_y, gy, "act bp y held");
                end
                // check the beat
                check_eq_u(act_index_x,   gx,                    "act_index_x");
                check_eq_u(act_index_y,   gy,                    "act_index_y");
                check_eq_u(act_index,     k,                     "act_index flat");
                check_bit (act_index_last, (k == total-1),       "act_index_last");
                check_bit (act_index_valid, 1'b1,                "act_index_valid");
                // consume one beat
                act_index_taken = 1;
                @(posedge clk); #1;
                act_index_taken = 0;
            end

            // After the final index is taken the generator must have stopped.
            check_bit(act_index_gen_running, 1'b0, "act gen stopped after sweep");
        end
    endtask

    integer trial;
    reg [X_INPUT_SZ-1:0] rxl;
    reg [Y_INPUT_SZ-1:0] ryl;

    initial begin
        verif_errors = 0; verif_checks = 0;
        reset = 1; start_new_block = 0; running = 0; next_in_neuron = 0;
        weight_mode = 2'b00;
        in_x_len = 4'd3; in_y_len = 4'd2;
        x_kernel_len = 1; y_kernel_len = 1;
        x_kernel_step = 1; y_kernel_step = 1;
        weights_per_word = 1; rows_per_neuron = 1;
        act_index_taken = 0;

        @(posedge clk); #1; reset = 0; @(posedge clk); #1;
        $display("=== tb_act_index_generator (snnAcc) ===");

        // ---- Directed grids, no back-pressure -----------------------------
        run_grid(5'd3, 5'd2, 0);     // 3x2
        run_grid(5'd1, 5'd1, 0);     // single element
        run_grid(5'd4, 5'd1, 0);     // single row
        run_grid(5'd1, 5'd4, 0);     // single column
        run_grid(5'd5, 5'd5, 0);     // full 5x5

        // ---- finished_timestep pulse + running drop (directed observation) -
        $display("checking finished_timestep pulse on last taken");
        in_x_len = 5'd2; in_y_len = 5'd1;
        running = 0; act_index_taken = 0; @(posedge clk); #1;
        running = 1; `VT_PULSE(start_new_block)
        act_index_taken = 1;
        // element 0
        check_bit(finished_timestep, 1'b0, "ft low on first");
        @(posedge clk); #1;
        // element 1 (last) being taken this cycle -> finished_timestep pulses
        check_bit(act_index_last, 1'b1, "last high on element 1");
        check_bit(finished_timestep, 1'b1, "finished_timestep pulse on last taken");
        @(posedge clk); #1;
        act_index_taken = 0;
        check_bit(act_index_gen_running, 1'b0, "gen halted after timestep");

        // ---- Constrained-random grids with random back-pressure -----------
        void'($urandom(32'hAC1D_5EED));
        for (trial = 0; trial < 40; trial = trial + 1) begin
            rxl = $urandom_range(1, 5);
            ryl = $urandom_range(1, 5);
            run_grid(rxl, ryl, $urandom_range(0, 60));
        end

        `VERIF_EPILOGUE("tb_act_index_generator")
    end

    `VERIF_WATCHDOG(8000000)

endmodule
