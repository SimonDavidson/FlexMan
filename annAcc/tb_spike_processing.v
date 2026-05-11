// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
`timescale 1ns/1ps
`include "../shared/constants.v"

// ================================================================
// tb_spike_processing
//
// Stage-level test for spike_processing.
//
// Configuration
// -------------
//   weight_mode = full connectivity (2'b00)
//   Input layer:  2×2 neurons (in_x_len=2, in_y_len=2)
//   Output layer: 2×2 neurons (out_x_len=2, out_y_len=2)
//   Weight width: 8-bit (weight_sz = 3'b011)
//   All 4 input activation bits pre-set to 1 (all spiking)
//   Weight SRAM pre-filled with a known pattern
//
// The test checks that:
//   1. spike_proc_finished_o asserts within a timeout after
//      start_new_block_i is pulsed.
//   2. syn_curr_mem_wr_o fires at least once, confirming that
//      weight accumulation actually wrote back synaptic currents.
//   3. acc_busy_o de-asserts when finished.
//
// Memory models: synchronous SRAMs (1-cycle read, 1-cycle write).
// ================================================================
module tb_spike_processing;

localparam NUM_TIMESTEPS      = 1;
localparam X_INPUT_SZ         = 4;
localparam Y_INPUT_SZ         = 4;
localparam X_OUTPUT_SZ        = 4;
localparam Y_OUTPUT_SZ        = 4;
localparam X_KERNEL_SZ        = 3;
localparam Y_KERNEL_SZ        = 3;
localparam X_KERNEL_OFF_SZ    = 3;
localparam Y_KERNEL_OFF_SZ    = 3;
localparam X_STEP_SZ          = 3;
localparam Y_STEP_SZ          = 3;
localparam ELEMS_PER_ROW      = 4;
localparam ROWS_PER_NEURON    = 16;
localparam TIMESTEP_SZ        = 10;
localparam IN_DATA_BITS       = 32;
localparam ELEM_SZ            = 8;
localparam ACT_IDX_SZ         = `COL_BITS;
localparam ACT_SLICE_SZ       = 5;     // 2^5 = 32-bit act bus; runtime sz set via act_slice_sz_i
localparam ACT_DATA_IDX_SZ    = 5;
localparam WEIGHT_ENTRY_BITS  = 8;
localparam WEIGHT_IDX_SZ      = 5;
localparam WEIGHT_SLICE_SZ    = 5;     // 2^5 = 32-bit weight bus
localparam WEIGHT_DATA_IDX_SZ = 5;
localparam SYN_CURR_IDX_SZ    = 10;
localparam SYN_CURR_DATA_IDX_SZ = 5;
localparam SYN_CURR_SLICE_SZ  = 3;
localparam SYN_CURR_SLICE_BITS= 32;
localparam BIAS_CURR_IDX_SZ   = 2;
localparam BIAS_CURR_DATA_IDX_SZ = 5;
localparam BIAS_CURR_SLICE_SZ = 3;
localparam BIAS_CURR_SLICE_BITS = 8;
localparam SPARSE_IDX_SZ      = 16;

reg  clk, reset;
reg [2:0]                   act_slice_sz;

// Config registers
reg [`ADDR_SIZE-1:0]        act_base_addr;
reg [`ADDR_SIZE-1:0]        weight_base_addr;
reg [`ADDR_SIZE-1:0]        syn_curr_base_addr;
reg [WEIGHT_SLICE_SZ-1:0]  weight_sz;
reg [4:0]                   bin_point_syn_curr;
reg [1:0]                   weight_mode;
reg [X_INPUT_SZ-1:0]       in_x_len;
reg [Y_INPUT_SZ-1:0]       in_y_len;
reg [X_OUTPUT_SZ-1:0]      out_x_len;
reg [Y_OUTPUT_SZ-1:0]      out_y_len;
reg [X_KERNEL_SZ-1:0]      x_kernel_len;
reg [Y_KERNEL_SZ-1:0]      y_kernel_len;
reg [X_KERNEL_OFF_SZ-1:0]  x_kernel_offset;
reg [Y_KERNEL_OFF_SZ-1:0]  y_kernel_offset;
reg [X_STEP_SZ-1:0]        x_kernel_step;
reg [Y_STEP_SZ-1:0]        y_kernel_step;
reg [WEIGHT_SLICE_SZ-1:0]  index_sz;
reg [WEIGHT_SLICE_SZ-1:0]  tuple_sz;
reg [`PIN_BITS-1:0]         sparse_count;
reg [ELEMS_PER_ROW-1:0]    weights_per_word;
reg [ROWS_PER_NEURON-1:0]  rows_per_neuron;
reg [WEIGHT_IDX_SZ-1:0]    weight_idx_sz;
// Scheduler interface
reg                         start_new_block;
reg [`TGT_ACC_SZ-1:0]      target_acc;
reg [`SCH_ENTRY_SZ-1:0]    buffer_info;
// Buffer addresses
reg [`PIN_BITS-1:0]         src1_buff_addr;
reg [`PIN_BITS-1:0]         src2_buff_addr;
reg [`PIN_BITS-1:0]         src3_buff_addr;
reg [`PIN_BITS-1:0]         tgt_buff_addr;
reg [`PIN_BITS-1:0]         weight_row_len;

wire                        spike_proc_finished;
wire                        acc_busy;
wire                        acc_finished;

// Memory interfaces
wire                        weight_mem_rd;
reg                         weight_mem_wait;
wire [`ADDR_SIZE-1:0]       weight_mem_addr;
reg  [`WTD_BITS-1:0]        weight_mem_data;

wire                        act_mem_req;
reg                         act_mem_wait;
wire [`ADDR_SIZE-1:0]       act_mem_addr;
reg  [`ACT_BITS-1:0]        act_mem_data;

wire                        syn_curr_mem_wr;
wire                        syn_curr_mem_rd;
reg                         syn_curr_mem_wait;
wire [`ADDR_SIZE-1:0]       syn_curr_mem_addr;
wire [`POT_BITS-1:0]        syn_curr_mem_data_wr;
reg  [`POT_BITS-1:0]        syn_curr_mem_data_rd;

spike_processing #(
    .NUM_TIMESTEPS     (NUM_TIMESTEPS),
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
    .ELEMS_PER_ROW     (ELEMS_PER_ROW),
    .ROWS_PER_NEURON   (ROWS_PER_NEURON),
    .TIMESTEP_SZ       (TIMESTEP_SZ),
    .IN_DATA_BITS      (IN_DATA_BITS),
    .ELEM_SZ           (ELEM_SZ),
    .ACT_IDX_SZ        (ACT_IDX_SZ),
    .ACT_SLICE_SZ      (ACT_SLICE_SZ),
    .ACT_DATA_IDX_SZ   (ACT_DATA_IDX_SZ),
    .WEIGHT_ENTRY_BITS (WEIGHT_ENTRY_BITS),
    .WEIGHT_IDX_SZ     (WEIGHT_IDX_SZ),
    .WEIGHT_SLICE_SZ   (WEIGHT_SLICE_SZ),
    .WEIGHT_DATA_IDX_SZ(WEIGHT_DATA_IDX_SZ),
    .SYN_CURR_IDX_SZ   (SYN_CURR_IDX_SZ),
    .SYN_CURR_DATA_IDX_SZ(SYN_CURR_DATA_IDX_SZ),
    .SYN_CURR_SLICE_SZ (SYN_CURR_SLICE_SZ),
    .SYN_CURR_SLICE_BITS(SYN_CURR_SLICE_BITS),
    .BIAS_CURR_IDX_SZ  (BIAS_CURR_IDX_SZ),
    .BIAS_CURR_DATA_IDX_SZ(BIAS_CURR_DATA_IDX_SZ),
    .BIAS_CURR_SLICE_SZ(BIAS_CURR_SLICE_SZ),
    .BIAS_CURR_SLICE_BITS(BIAS_CURR_SLICE_BITS),
    .SPARSE_IDX_SZ     (SPARSE_IDX_SZ))
dut (
    .clk                    (clk),
    .reset                  (reset),
    .act_base_addr_i        (act_base_addr),
    .weight_base_addr_i     (weight_base_addr),
    .syn_curr_base_addr_i   (syn_curr_base_addr),
    .weight_sz_i            (weight_sz),
    .bin_point_syn_curr_i   (bin_point_syn_curr),
    .weight_mode_i          (weight_mode),
    .in_x_len_i             (in_x_len),
    .in_y_len_i             (in_y_len),
    .out_x_len_i            (out_x_len),
    .out_y_len_i            (out_y_len),
    .x_kernel_len_i         (x_kernel_len),
    .y_kernel_len_i         (y_kernel_len),
    .x_kernel_offset_i      (x_kernel_offset),
    .y_kernel_offset_i      (y_kernel_offset),
    .x_kernel_step_i        (x_kernel_step),
    .y_kernel_step_i        (y_kernel_step),
    .index_sz_i             (index_sz),
    .tuple_sz_i             (tuple_sz),
    .sparse_count_i         (sparse_count),
    .weights_per_word_i     (weights_per_word),
    .rows_per_neuron_i      (rows_per_neuron),
    .weight_idx_sz_i        (weight_idx_sz),
    .start_new_block_i      (start_new_block),
    .target_acc_i           (target_acc),
    .buffer_info_i          (buffer_info),
    .spike_proc_finished_o  (spike_proc_finished),
    .acc_busy_o             (acc_busy),
    .acc_finished_o         (acc_finished),
    .src1_buff_addr_i       (src1_buff_addr),
    .src2_buff_addr_i       (src2_buff_addr),
    .src3_buff_addr_i       (src3_buff_addr),
    .tgt_buff_addr_i        (tgt_buff_addr),
    .weight_row_len_i       (weight_row_len),
    .weight_mem_rd_o        (weight_mem_rd),
    .weight_mem_wait_i      (weight_mem_wait),
    .weight_mem_addr_o      (weight_mem_addr),
    .weight_mem_data_i      (weight_mem_data),
    .act_mem_req_o          (act_mem_req),
    .act_mem_wait_i         (act_mem_wait),
    .act_mem_addr_o         (act_mem_addr),
    .act_mem_data_i         (act_mem_data),
    .act_slice_sz_i         (act_slice_sz),
    .syn_curr_mem_wr_o      (syn_curr_mem_wr),
    .syn_curr_mem_rd_o      (syn_curr_mem_rd),
    .syn_curr_mem_wait_i    (syn_curr_mem_wait),
    .syn_curr_mem_addr_o    (syn_curr_mem_addr),
    .syn_curr_mem_data_wr_o (syn_curr_mem_data_wr),
    .syn_curr_mem_data_rd_i (syn_curr_mem_data_rd)
);

// ----------------------------------------------------------------
// SRAM models (1-cycle read latency, no wait)
// ----------------------------------------------------------------
reg [31:0] act_sram    [0:255];
reg [31:0] weight_sram [0:255];
reg [31:0] syn_curr_sram [0:255];

integer mi;

always @(posedge clk) begin
    if (act_mem_req    & ~act_mem_wait)
        act_mem_data       <= act_sram[act_mem_addr[7:0]];
    if (weight_mem_rd  & ~weight_mem_wait)
        weight_mem_data    <= weight_sram[weight_mem_addr[7:0]];
    if (syn_curr_mem_rd)
        syn_curr_mem_data_rd <= syn_curr_sram[syn_curr_mem_addr[7:0]];
    if (syn_curr_mem_wr)
        syn_curr_sram[syn_curr_mem_addr[7:0]] <= syn_curr_mem_data_wr;
end

initial clk = 0;
always  #5 clk = ~clk;

integer errors, timeout;

initial begin
    errors = 0;

    // Init SRAMs
    for (mi = 0; mi < 256; mi = mi + 1) begin
        // 8-bit elements (act_slice_sz_i=3), 4 per 32-bit word; all = 1 (non-zero).
        act_sram[mi]      = 32'h0101_0101;
        // 8-bit weights packed 4-per-word; use value 1 for easy checking
        weight_sram[mi]   = 32'h0101_0101;
        syn_curr_sram[mi] = 32'h0;
    end

    reset            = 1;
    act_slice_sz     = 3'b011;   // 8-bit elements at runtime
    start_new_block  = 0;
    act_mem_wait     = 0;
    weight_mem_wait  = 0;
    syn_curr_mem_wait = 0;
    act_mem_data     = 0;
    weight_mem_data  = 0;
    syn_curr_mem_data_rd = 0;
    target_acc       = 0;
    buffer_info      = 0;
    src1_buff_addr   = 0;
    src2_buff_addr   = 0;
    src3_buff_addr   = 0;
    tgt_buff_addr    = 0;
    weight_row_len   = 32;
    weight_sz        = 3'b011;   // 8-bit weights
    bin_point_syn_curr = 5'd0;
    weight_mode      = 2'b00;    // full connectivity
    in_x_len         = 4'd2;
    in_y_len         = 4'd2;
    out_x_len        = 4'd2;
    out_y_len        = 4'd2;
    x_kernel_len     = 1;
    y_kernel_len     = 1;
    x_kernel_offset  = 0;
    y_kernel_offset  = 0;
    x_kernel_step    = 1;
    y_kernel_step    = 1;
    index_sz         = 0;
    tuple_sz         = 0;
    sparse_count     = 0;
    weights_per_word = 4;
    rows_per_neuron  = 1;
    weight_idx_sz    = 5'd4;
    act_base_addr    = 0;
    weight_base_addr = 0;
    syn_curr_base_addr = 30'd128;   // offset to avoid collision

    @(posedge clk); #1;
    reset = 0;
    @(posedge clk); #1;

    $display("=== tb_spike_processing ===");
    $display("Test 1: full-mode, 2×2 in/out, all spikes");

    // Pulse start
    start_new_block = 1;
    @(posedge clk); #1;
    start_new_block = 0;

    // Wait for completion with timeout
    timeout = 500;
    while (!spike_proc_finished && timeout > 0) begin
        @(posedge clk); #1;
        timeout = timeout - 1;
    end

    if (timeout == 0) begin
        $display("FAIL: spike_proc_finished never asserted (timeout)");
        errors = errors + 1;
    end else begin
        $display("  spike_proc_finished after %0d cycles", 500 - timeout);
        // spike_proc_finished_o is combinational (running_r & ~submodules).
        // acc_busy_o = running_r, which is cleared by the NBA at the posedge
        // where spike_proc_finished fires.  Check acc_busy one cycle later,
        // after running_r has been clocked to 0.
        @(posedge clk); #1;
        if (acc_busy) begin
            $display("FAIL: acc_busy_o still high after finished");
            errors = errors + 1;
        end
        // Check that at least one syn_curr write happened
        // (syn_curr_sram entries beyond base should have changed)
        if (syn_curr_sram[128] === 32'h0) begin
            $display("FAIL: syn_curr_sram[128] still 0 (no write-back detected)");
            errors = errors + 1;
        end else begin
            $display("  syn_curr_sram[128] = 0x%08h (non-zero as expected)", syn_curr_sram[128]);
        end
    end

    $display("=== tb_spike_processing: %0d failure(s) ===", errors);
    if (errors == 0) $display("PASS"); else $display("FAIL");
    $finish;
end

// Safety watchdog
initial begin
    #100000;
    $display("FAIL: global simulation timeout");
    $finish;
end

endmodule
