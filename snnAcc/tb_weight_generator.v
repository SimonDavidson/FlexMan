// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
// =============================================================================
// tb_weight_generator  (snnAcc)
//
// Authors      : Simon Davidson & Claude
// Created      : 2026-05-07
// Last modified: 2026-06-07
//
// Aggressive rewrite using the shared verif library.  weight_generator emits a
// stream of weight-memory address beats (weight_index_o, weight_index_x/y_o,
// weight_index_last_o) and the corresponding weight values fetched through an
// internal dataline_cache_with_xy, for full / sparse / convolutional modes.
//
// Methodology: a monitor accepts every (weight_index_valid & taken) beat and
// records (index, x, y, last).  A software model of each mode's address/index
// sequence is generated up front; the captured stream is checked against it
// beat-for-beat.
//
//   * FULL   golden: for each active input neuron the output sweep is row-major
//                    over out_x_len x out_y_len; index = running element count,
//                    last on the final (x,y).
//   * SPARSE golden: sparse_count tuples per input neuron; x=y=0; index counts.
//   * CONV   smoke : project (act_x*step+kx-off, act_y*step+ky-off) over the
//                    kernel; OOB projections are skipped (no valid) but the
//                    kernel pointer still advances.  We check the emitted
//                    sequence length equals the in-bounds count and that every
//                    emitted (x,y) is the in-bounds projection in kernel order.
// =============================================================================
`timescale 1ns/1ps
`include "../shared/constants.v"

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
    localparam WEIGHT_SLICE_SZ   = 3;   // 8-bit weight elements
    localparam WEIGHT_DATA_IDX_SZ= 5;
    localparam WEIGHT_BITS       = 8;

    integer verif_errors, verif_checks, verif_to;
    `include "../verif/checks.vh"

    reg                          clk, reset;
    reg                          start_new_block;
    reg                          running;
    reg [1:0]                    weight_mode;
    reg [X_INPUT_SZ-1:0]         in_x_len;
    reg [Y_INPUT_SZ-1:0]         in_y_len;
    reg [X_OUTPUT_SZ-1:0]        out_x_len;
    reg [Y_OUTPUT_SZ-1:0]        out_y_len;
    reg [X_KERNEL_SZ-1:0]        x_kernel_len;
    reg [Y_KERNEL_SZ-1:0]        y_kernel_len;
    reg [X_KERNEL_OFF_SZ-1:0]    x_kernel_offset;
    reg [Y_KERNEL_OFF_SZ-1:0]    y_kernel_offset;
    reg [X_STEP_SZ-1:0]          x_kernel_step;
    reg [Y_STEP_SZ-1:0]          y_kernel_step;
    reg [`ADDR_SIZE-1:0]         weight_base_addr;
    reg [WEIGHT_SLICE_SZ-1:0]    weight_sz;
    reg [WEIGHT_SLICE_SZ-1:0]    tuple_sz;
    reg [`PIN_BITS-1:0]          sparse_count;
    reg [WEIGHT_ENTRY_BITS-1:0]  weight_entry_bits;
    reg [ELEMS_PER_ROW-1:0]      weights_per_word;
    reg [ROWS_PER_NEURON-1:0]    rows_per_neuron;
    reg                          act_data_valid;
    reg [ACT_IDX_SZ-1:0]         act_data_idx;
    reg [X_INPUT_SZ-1:0]         act_data_x;
    reg [Y_INPUT_SZ-1:0]         act_data_y;
    reg [ACT_DATA_SZ-1:0]        act_data;
    reg                          act_data_last;
    reg                          weight_index_taken;
    reg                          weight_value_taken;
    reg                          weight_mem_wait;
    reg [`WTD_BITS-1:0]          weight_mem_data;

    wire                         finished_one_pass;
    wire                         finished_pass;
    wire                         running_weight_pass;
    wire                         weight_mem_rd;
    wire [`ADDR_SIZE-1:0]        weight_mem_addr;
    wire                         weight_index_valid;
    wire [WEIGHT_IDX_SZ-1:0]     weight_index;
    wire [X_OUTPUT_SZ-1:0]       weight_index_x;
    wire [Y_OUTPUT_SZ-1:0]       weight_index_y;
    wire                         weight_index_last;
    wire                         weight_value_valid;
    wire [WEIGHT_BITS-1:0]       weight_value;

    weight_generator #(
        .X_INPUT_SZ(X_INPUT_SZ), .Y_INPUT_SZ(Y_INPUT_SZ),
        .X_OUTPUT_SZ(X_OUTPUT_SZ), .Y_OUTPUT_SZ(Y_OUTPUT_SZ),
        .X_KERNEL_SZ(X_KERNEL_SZ), .Y_KERNEL_SZ(Y_KERNEL_SZ),
        .X_KERNEL_OFF_SZ(X_KERNEL_OFF_SZ), .Y_KERNEL_OFF_SZ(Y_KERNEL_OFF_SZ),
        .X_STEP_SZ(X_STEP_SZ), .Y_STEP_SZ(Y_STEP_SZ),
        .WEIGHT_ENTRY_BITS(WEIGHT_ENTRY_BITS),
        .ELEMS_PER_ROW(ELEMS_PER_ROW), .ROWS_PER_NEURON(ROWS_PER_NEURON),
        .IN_DATA_BITS(IN_DATA_BITS), .ELEM_SZ(ELEM_SZ),
        .ACT_IDX_SZ(ACT_IDX_SZ), .ACT_DATA_SZ(ACT_DATA_SZ),
        .WEIGHT_IDX_SZ(WEIGHT_IDX_SZ), .WEIGHT_SLICE_SZ(WEIGHT_SLICE_SZ),
        .WEIGHT_DATA_IDX_SZ(WEIGHT_DATA_IDX_SZ))
    dut (
        .clk(clk), .reset(reset),
        .start_new_block_i(start_new_block), .running_i(running),
        .finished_one_pass_o(finished_one_pass),
        .finished_pass_o(finished_pass),
        .running_weight_pass_o(running_weight_pass),
        .weight_mode_i(weight_mode),
        .in_x_len_i(in_x_len), .in_y_len_i(in_y_len),
        .out_x_len_i(out_x_len), .out_y_len_i(out_y_len),
        .x_kernel_len_i(x_kernel_len), .y_kernel_len_i(y_kernel_len),
        .x_kernel_offset_i(x_kernel_offset), .y_kernel_offset_i(y_kernel_offset),
        .x_kernel_step_i(x_kernel_step), .y_kernel_step_i(y_kernel_step),
        .weight_base_addr_i(weight_base_addr),
        .weight_sz_i(weight_sz), .tuple_sz_i(tuple_sz),
        .sparse_count_i(sparse_count),
        .weight_entry_bits_i(weight_entry_bits),
        .weights_per_word_i(weights_per_word),
        .rows_per_neuron_i(rows_per_neuron),
        .act_data_valid_i(act_data_valid),
        .act_data_x_i(act_data_x), .act_data_y_i(act_data_y),
        .act_data_idx_i(act_data_idx), .act_data_i(act_data),
        .act_data_last_i(act_data_last),
        .act_last_dumped_i(1'b0),   // sub-module TB: no gated act stream
        .weight_mem_rd_o(weight_mem_rd), .weight_mem_wait_i(weight_mem_wait),
        .weight_mem_addr_o(weight_mem_addr), .weight_mem_data_i(weight_mem_data),
        .weight_index_valid_o(weight_index_valid),
        .weight_index_o(weight_index),
        .weight_index_x_o(weight_index_x), .weight_index_y_o(weight_index_y),
        .weight_index_last_o(weight_index_last),
        .weight_index_taken_i(weight_index_taken),
        .weight_value_valid_o(weight_value_valid),
        .weight_value_o(weight_value),
        .weight_value_taken_i(weight_value_taken));

    // Weight SRAM: word k = ascending bytes so each fetched slice is unique.
    reg [31:0] sram [0:255];
    integer mi;
    always @(posedge clk)
        if (weight_mem_rd & ~weight_mem_wait)
            weight_mem_data <= sram[weight_mem_addr[7:0]];

    initial clk = 0; always #5 clk = ~clk;

    // ---- Captured stream of accepted weight-index beats ------------------
    integer cap_n;
    integer cap_idx [0:255];
    integer cap_x   [0:255];
    integer cap_y   [0:255];
    integer cap_last[0:255];

    // Drive a single active input neuron's weight pass to completion, capturing
    // every accepted (weight_index_valid & weight_index_taken) beat.  Both
    // index and value handshakes are taken together (they are tied in the RTL).
    task run_neuron_capture;
        input [X_INPUT_SZ-1:0] ax;
        input [Y_INPUT_SZ-1:0] ay;
        input [ACT_IDX_SZ-1:0] aidx;
        input                  alast;
        input integer          max_cyc;
        integer c;
        reg     done;
        begin
            act_data_valid = 1; act_data_x = ax; act_data_y = ay;
            act_data_idx = aidx; act_data_last = alast; act_data = 8'hFF;
            c = 0; done = 0;
            // Capture every accepted beat.  finished_pass is a COMBINATIONAL
            // pulse that asserts on the same cycle the final beat is taken and
            // drops again the instant the kernel/element counters advance past
            // the edge — so it must be sampled AT the posedge (with the taken
            // strobe still applied), never at #1 after it.
            while (!done && c < max_cyc) begin
                weight_index_taken = weight_index_valid;
                weight_value_taken = weight_value_valid;
                if (weight_index_valid & weight_value_valid) begin
                    cap_idx [cap_n] = weight_index;
                    cap_x   [cap_n] = weight_index_x;
                    cap_y   [cap_n] = weight_index_y;
                    cap_last[cap_n] = weight_index_last;
                    cap_n = cap_n + 1;
                end
                @(posedge clk);
                done = finished_pass;   // sample the pulse AT the edge
                #1;
                c = c + 1;
            end
            if (c >= max_cyc) begin
                $display("FAIL: run_neuron_capture timed out (no finished_pass)");
                verif_errors = verif_errors + 1;
            end
            weight_index_taken = 0; weight_value_taken = 0;
            act_data_valid = 0;
        end
    endtask

    task restart;
        begin
            running = 0; act_data_valid = 0;
            weight_index_taken = 0; weight_value_taken = 0;
            @(posedge clk); #1;
            running = 1;
            start_new_block = 1; @(posedge clk); #1; start_new_block = 0;
            cap_n = 0;
        end
    endtask

    integer k, ex, ey, ei, total;

    initial begin
        verif_errors = 0; verif_checks = 0;
        reset = 1; start_new_block = 0; running = 0;
        act_data_valid = 0; act_data_idx = 0; act_data_x = 0; act_data_y = 0;
        act_data = 8'hFF; act_data_last = 0;
        weight_index_taken = 0; weight_value_taken = 0;
        weight_mem_wait = 0; weight_mem_data = 0;
        weight_mode = 2'b00; weight_sz = 3'b011; tuple_sz = 3'b011;
        sparse_count = 4; weight_entry_bits = 8; weights_per_word = 4;
        rows_per_neuron = 1; weight_base_addr = 0;
        x_kernel_len = 1; y_kernel_len = 1;
        x_kernel_offset = 0; y_kernel_offset = 0;
        x_kernel_step = 1; y_kernel_step = 1;
        in_x_len = 1; in_y_len = 1; out_x_len = 1; out_y_len = 1;

        for (mi = 0; mi < 256; mi = mi + 1)
            sram[mi] = {mi[7:0]+8'd3, mi[7:0]+8'd2, mi[7:0]+8'd1, mi[7:0]};

        @(posedge clk); #1; reset = 0; @(posedge clk); #1;
        $display("=== tb_weight_generator (snnAcc) ===");

        // ===================================================================
        // FULL mode: 1 active input neuron, out grid 3x1 -> indices 0,1,2
        // ===================================================================
        $display("FULL: out 3x1");
        weight_mode = 2'b00; in_x_len = 1; in_y_len = 1;
        out_x_len = 3; out_y_len = 1;
        restart;
        run_neuron_capture(0, 0, 0, 1, 200);
        total = out_x_len * out_y_len;
        check_eq_u(cap_n, total, "FULL 3x1 beat count");
        for (k = 0; k < total && k < cap_n; k = k + 1) begin
            ex = k % out_x_len; ey = k / out_x_len;
            check_eq_u(cap_idx[k],  k,                     "FULL idx");
            check_eq_u(cap_x[k],    ex,                    "FULL x");
            check_eq_u(cap_y[k],    ey,                    "FULL y");
            check_bit (cap_last[k], (k == total-1),        "FULL last");
        end

        // ---- FULL mode: out grid 3x2 -> row-major 6 beats -----------------
        $display("FULL: out 3x2");
        weight_mode = 2'b00; out_x_len = 3; out_y_len = 2;
        restart;
        run_neuron_capture(0, 0, 0, 1, 200);
        total = out_x_len * out_y_len;
        check_eq_u(cap_n, total, "FULL 3x2 beat count");
        for (k = 0; k < total && k < cap_n; k = k + 1) begin
            ex = k % out_x_len; ey = k / out_x_len;
            check_eq_u(cap_idx[k], k,  "FULL 3x2 idx");
            check_eq_u(cap_x[k],   ex, "FULL 3x2 x");
            check_eq_u(cap_y[k],   ey, "FULL 3x2 y");
        end

        // ===================================================================
        // SPARSE mode: sparse_count tuples for one input neuron, x=y=0
        // ===================================================================
        $display("SPARSE: 4 tuples");
        weight_mode = 2'b01; sparse_count = 4; tuple_sz = 3'b011;
        out_x_len = 4; out_y_len = 4;
        restart;
        run_neuron_capture(0, 0, 0, 1, 200);
        check_eq_u(cap_n, 4, "SPARSE beat count == sparse_count");
        for (k = 0; k < 4 && k < cap_n; k = k + 1) begin
            check_eq_u(cap_idx[k],  k,                "SPARSE idx");
            check_eq_u(cap_x[k],    0,                "SPARSE x==0");
            check_eq_u(cap_y[k],    0,                "SPARSE y==0");
            check_bit (cap_last[k], (k == 3),         "SPARSE last");
        end

        // ---- SPARSE: different count -------------------------------------
        $display("SPARSE: 7 tuples");
        weight_mode = 2'b01; sparse_count = 7;
        restart;
        run_neuron_capture(0, 0, 0, 1, 200);
        check_eq_u(cap_n, 7, "SPARSE 7 beat count");
        for (k = 0; k < 7 && k < cap_n; k = k + 1)
            check_eq_u(cap_idx[k], k, "SPARSE 7 idx");

        // ===================================================================
        // CONV smoke: 3x3 kernel, step=1, offset=1, out 3x3.  For input
        // neuron (1,1): projections out=(0..2,0..2) all in-bounds -> 9 beats.
        // For input neuron (0,0): out=(-1..1,-1..1) -> only (0,0),(0,1),(1,0),
        // (1,1) in-bounds = 4 beats (kernel pointer still advances on skips).
        // We verify the in-bounds count and the projected (x,y) order.
        // ===================================================================
        $display("CONV: 3x3 kernel, offset 1, in (1,1) fully in-bounds");
        weight_mode = 2'b10;
        in_x_len = 3; in_y_len = 3; out_x_len = 3; out_y_len = 3;
        x_kernel_len = 3; y_kernel_len = 3;
        x_kernel_offset = 1; y_kernel_offset = 1;
        x_kernel_step = 1; y_kernel_step = 1;
        restart;
        run_neuron_capture(1, 1, 0, 1, 300);
        check_eq_u(cap_n, 9, "CONV in(1,1) 9 in-bounds beats");
        // expected projections in kernel row-major order
        for (k = 0; k < 9 && k < cap_n; k = k + 1) begin
            ex = (1*1) + (k % 3) - 1;   // act_x*step + kx - off
            ey = (1*1) + (k / 3) - 1;   // act_y*step + ky - off
            check_eq_u(cap_x[k], ex, "CONV in(1,1) x");
            check_eq_u(cap_y[k], ey, "CONV in(1,1) y");
        end

        // ---- CONV smoke: corner input (0,0): 4 in-bounds beats ------------
        $display("CONV: in (0,0) corner -> 4 in-bounds beats (skips advance ptr)");
        restart;
        run_neuron_capture(0, 0, 0, 1, 300);
        check_eq_u(cap_n, 4, "CONV in(0,0) 4 in-bounds beats");
        // in-bounds (kx,ky): (1,1)->(0,0); (2,1)->(1,0); (1,2)->(0,1); (2,2)->(1,1)
        begin : conv_corner_check
            integer kx, ky, px, py, b;
            b = 0;
            for (ky = 0; ky < 3; ky = ky + 1)
                for (kx = 0; kx < 3; kx = kx + 1) begin
                    px = 0 + kx - 1; py = 0 + ky - 1;
                    if (px >= 0 && px < 3 && py >= 0 && py < 3) begin
                        if (b < cap_n) begin
                            check_eq_u(cap_x[b], px, "CONV corner x");
                            check_eq_u(cap_y[b], py, "CONV corner y");
                        end
                        b = b + 1;
                    end
                end
            check_eq_u(b, cap_n, "CONV corner in-bounds count matches capture");
        end

        // ---- CONV smoke: 1x1 kernel identity over 3x1 in/out --------------
        $display("CONV: 1x1 kernel, step1 offset0 over 3x1");
        in_x_len = 3; in_y_len = 1; out_x_len = 3; out_y_len = 1;
        x_kernel_len = 1; y_kernel_len = 1;
        x_kernel_offset = 0; y_kernel_offset = 0;
        restart;
        // single input neuron (act_data_last=1), projects to a single output
        run_neuron_capture(0, 0, 0, 1, 100);
        check_eq_u(cap_n, 1, "CONV 1x1 in0 -> 1 beat");
        check_eq_u(cap_x[0], 0, "CONV 1x1 in0 x");

        `VERIF_EPILOGUE("tb_weight_generator")
    end

    `VERIF_WATCHDOG(10000000)

endmodule
