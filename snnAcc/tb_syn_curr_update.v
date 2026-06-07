// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
// =============================================================================
// tb_syn_curr_update  (snnAcc)
//
// Authors      : Simon Davidson & Claude
// Created      : 2026-05-07
// Last modified: 2026-06-07
//
// Aggressive rewrite using the shared verif library.  syn_curr_update is the
// read-modify-write accumulator: each accepted (weight_index, weight_value)
// beat does
//     syn_curr[base + (sparse ? sparse_index : y*out_x_len + x)] +=
//         sign_extend_WEIGHT_BITS_to_32(weight_value)
// with one cycle to latch the address (rd) and one to write back (wr), gated by
// syn_curr_mem_wait_i.
//
// To exercise the sign-extension path (which a 32-bit-weight instance never
// hits), this TB instantiates the DUT with 8-bit weights (WEIGHT_SLICE_SZ=3).
//
// Golden: a parallel software syn_curr array `g_sram` accumulated with the SAME
// 8->32 sign extension; after each accepted write the touched word is checked
// against the model.  Covers full mode (x/y addressing), sparse mode
// (sparse_index addressing), positive/negative weights at the 8-bit boundary,
// repeated accumulation onto the same neuron, and memory back-pressure.
// =============================================================================
`timescale 1ns/1ps
`include "../shared/constants.v"

module tb_syn_curr_update;

    localparam X_OUTPUT_SZ        = 4;
    localparam Y_OUTPUT_SZ        = 4;
    localparam IN_DATA_BITS       = 32;
    localparam WEIGHT_IDX_SZ      = 5;
    localparam WEIGHT_SLICE_SZ    = 3;   // 2^3 = 8-bit weights (sign-extended)
    localparam WEIGHT_DATA_IDX_SZ = 5;
    localparam SPARSE_IDX_SZ      = 8;
    localparam WEIGHT_BITS        = 8;   // 2**WEIGHT_SLICE_SZ
    localparam SRAM_DEPTH         = 256;

    integer verif_errors, verif_checks, verif_to, sram_i;
    `include "../verif/checks.vh"
    `include "../verif/sram_bfm.vh"

    reg                          clk, reset;
    reg                          running;
    reg                          finished_pass_weight;
    reg [1:0]                    weight_mode;
    reg [SPARSE_IDX_SZ-1:0]      sparse_index;
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

    syn_curr_update #(
        .X_OUTPUT_SZ       (X_OUTPUT_SZ),
        .Y_OUTPUT_SZ       (Y_OUTPUT_SZ),
        .IN_DATA_BITS      (IN_DATA_BITS),
        .WEIGHT_IDX_SZ     (WEIGHT_IDX_SZ),
        .WEIGHT_SLICE_SZ   (WEIGHT_SLICE_SZ),
        .WEIGHT_DATA_IDX_SZ(WEIGHT_DATA_IDX_SZ),
        .SPARSE_IDX_SZ     (SPARSE_IDX_SZ))
    dut (
        .clk(clk), .reset(reset),
        .running_i(running),
        .finished_pass_weight_i(finished_pass_weight),
        .finished_pass_o(finished_pass),
        .syn_curr_update_running_o(syn_curr_update_running),
        .weight_mode_i(weight_mode),
        .sparse_index_i(sparse_index),
        .syn_curr_base_addr_i(syn_curr_base),
        .out_x_len_i(out_x_len),
        .weight_index_valid_i(weight_index_valid),
        .weight_index_i(weight_index),
        .weight_index_x_i(weight_index_x),
        .weight_index_y_i(weight_index_y),
        .weight_index_last_i(weight_index_last),
        .weight_index_taken_o(weight_index_taken),
        .weight_value_valid_i(weight_value_valid),
        .weight_value_i(weight_value),
        .weight_value_taken_o(weight_value_taken),
        .syn_curr_mem_rd_o(syn_curr_mem_rd),
        .syn_curr_mem_wr_o(syn_curr_mem_wr),
        .syn_curr_mem_wait_i(syn_curr_mem_wait),
        .syn_curr_mem_addr_o(syn_curr_mem_addr),
        .syn_curr_mem_data_i(syn_curr_mem_data_i),
        .syn_curr_mem_data_o(syn_curr_mem_data_o));

    // Synchronous SRAM (read-modify-write port): 1-cycle read, gated write.
    `SRAM_RW(sram, syn_curr_mem_rd, (syn_curr_mem_wr & ~syn_curr_mem_wait),
             syn_curr_mem_addr, syn_curr_mem_data_o, syn_curr_mem_data_i)

    initial clk = 0; always #5 clk = ~clk;

    // ---- Software golden mirror of the syn_curr array --------------------
    reg signed [31:0] g_sram [0:SRAM_DEPTH-1];

    // 8-bit -> 32-bit sign extension (mirrors aligned_weight_value).
    function signed [31:0] g_sext;
        input [WEIGHT_BITS-1:0] w;
        begin
            g_sext = {{(32-WEIGHT_BITS){w[WEIGHT_BITS-1]}}, w};
        end
    endfunction

    function [`ADDR_SIZE-1:0] g_addr;
        input                    sparse;
        input [SPARSE_IDX_SZ-1:0] sidx;
        input [X_OUTPUT_SZ-1:0]  x;
        input [Y_OUTPUT_SZ-1:0]  y;
        input [X_OUTPUT_SZ-1:0]  xlen;
        input [`ADDR_SIZE-1:0]   base;
        begin
            g_addr = base + (sparse ? sidx : (y * xlen + x));
        end
    endfunction

    // Pulse running to (re)arm syn_curr_update_running_r for a new beat.
    task arm_running;
        begin
            running = 0; @(posedge clk); #1;
            running = 1; @(posedge clk); #1;
        end
    endtask

    // Wait for the write to be honoured (weight taken).  Sampled AT the posedge
    // because the NBA that commits the write also drops req_pending the same
    // edge — sampling after #1 would always miss it.
    task wait_wr_done;
        begin
            verif_to = 40;
            @(posedge clk);
            while (!weight_index_taken && verif_to > 0) begin
                verif_to = verif_to - 1;
                @(posedge clk);
            end
            if (verif_to == 0) begin
                $display("FAIL: timeout waiting for weight_index_taken");
                verif_errors = verif_errors + 1;
            end
            #1;
        end
    endtask

    // Drive one accumulation beat (with optional pre-write mem stall) and check
    // the touched word against the golden after the write commits.
    task accumulate;
        input                    sparse;
        input [SPARSE_IDX_SZ-1:0] sidx;
        input [X_OUTPUT_SZ-1:0]  x;
        input [Y_OUTPUT_SZ-1:0]  y;
        input [WEIGHT_BITS-1:0]  w;
        input integer            stall;
        input                    last_in;
        reg   [`ADDR_SIZE-1:0]   addr;
        integer                  s;
        begin
            arm_running;
            addr = g_addr(sparse, sidx, x, y, out_x_len, syn_curr_base) & 8'hFF;

            weight_mode        = sparse ? 2'b01 : 2'b00;
            sparse_index       = sidx;
            weight_index_valid = 1;
            weight_value_valid = 1;
            weight_index_x     = x;
            weight_index_y     = y;
            weight_index       = x;            // arbitrary, addressing uses x/y or sparse
            weight_index_last  = last_in;
            weight_value       = w;

            // optional memory back-pressure before the write commits
            if (stall > 0) begin
                @(posedge clk); #1;     // rd cycle
                syn_curr_mem_wait = 1;
                for (s = 0; s < stall; s = s + 1) begin @(posedge clk); #1; end
                syn_curr_mem_wait = 0;
            end

            wait_wr_done;
            weight_index_valid = 0;
            weight_value_valid = 0;
            @(posedge clk); #1;

            // advance golden
            g_sram[addr] = g_sram[addr] + g_sext(w);
            check_eq($signed(sram[addr]), g_sram[addr], "syn_curr accumulate");
        end
    endtask

    integer t, i;
    reg [WEIGHT_BITS-1:0] rw;
    reg [X_OUTPUT_SZ-1:0] rx;
    reg [Y_OUTPUT_SZ-1:0] ry;
    reg [SPARSE_IDX_SZ-1:0] rs;

    initial begin
        verif_errors = 0; verif_checks = 0;
        reset = 1; running = 0; finished_pass_weight = 0;
        weight_mode = 2'b00; sparse_index = 0; syn_curr_base = 0;
        out_x_len = X_OUTPUT_SZ;
        weight_index_valid = 0; weight_index = 0;
        weight_index_x = 0; weight_index_y = 0; weight_index_last = 0;
        weight_value_valid = 0; weight_value = 0;
        syn_curr_mem_wait = 0; syn_curr_mem_data_i = 0;

        // init both memories identically
        `SRAM_CLEAR(sram)
        for (sram_i = 0; sram_i < SRAM_DEPTH; sram_i = sram_i + 1) begin
            sram[sram_i]   = sram_i * 4;
            g_sram[sram_i] = sram_i * 4;
        end

        @(posedge clk); #1; reset = 0; @(posedge clk); #1;
        $display("=== tb_syn_curr_update (snnAcc) ===");

        // ---- Directed: positive weight, full mode (x=1,y=0) ---------------
        accumulate(1'b0, 8'd0, 4'd1, 4'd0, 8'd10, 0, 1'b1);
        // ---- Directed: negative weight, full mode (x=2,y=0): -3 = 0xFD ----
        accumulate(1'b0, 8'd0, 4'd2, 4'd0, 8'hFD, 0, 1'b1);
        // ---- Directed: sparse mode, index 7, +5 ---------------------------
        accumulate(1'b1, 8'd7, 4'd0, 4'd0, 8'd5, 0, 1'b1);
        // ---- Directed: memory stall (x=3,y=0), +1, 2-cycle stall ----------
        accumulate(1'b0, 8'd0, 4'd3, 4'd0, 8'd1, 2, 1'b1);
        // ---- Directed: sign boundary: most negative 8-bit weight 0x80=-128
        accumulate(1'b0, 8'd0, 4'd0, 4'd1, 8'h80, 0, 1'b1);
        // ---- Directed: max positive 8-bit weight 0x7F=+127 ----------------
        accumulate(1'b0, 8'd0, 4'd1, 4'd1, 8'h7F, 0, 1'b1);
        // ---- Directed: repeated accumulation onto same sparse neuron ------
        accumulate(1'b1, 8'd20, 4'd0, 4'd0, 8'd9,  0, 1'b0);
        accumulate(1'b1, 8'd20, 4'd0, 4'd0, 8'hFB, 0, 1'b1);  // +9 then -5

        // ---- Constrained-random: full mode -------------------------------
        void'($urandom(32'h5C00_FACE));
        for (t = 0; t < 600; t = t + 1) begin
            rw = $urandom();
            rx = $urandom_range(X_OUTPUT_SZ-1);
            ry = $urandom_range(Y_OUTPUT_SZ-1);
            accumulate(1'b0, 8'd0, rx, ry, rw,
                       ($urandom_range(99) < 30) ? $urandom_range(1,3) : 0,
                       ($urandom_range(1)));
        end

        // ---- Constrained-random: sparse mode -----------------------------
        for (t = 0; t < 600; t = t + 1) begin
            rw = $urandom();
            rs = $urandom_range(SRAM_DEPTH-1);
            accumulate(1'b1, rs, 4'd0, 4'd0, rw,
                       ($urandom_range(99) < 30) ? $urandom_range(1,3) : 0,
                       ($urandom_range(1)));
        end

        // ---- Random out_x_len with full-mode (x,y) addressing ------------
        out_x_len = 4'd3;
        for (t = 0; t < 200; t = t + 1) begin
            rw = $urandom();
            rx = $urandom_range(2);
            ry = $urandom_range(3);
            accumulate(1'b0, 8'd0, rx, ry, rw, 0, 1'b1);
        end

        `VERIF_EPILOGUE("tb_syn_curr_update")
    end

    `VERIF_WATCHDOG(8000000)

endmodule
