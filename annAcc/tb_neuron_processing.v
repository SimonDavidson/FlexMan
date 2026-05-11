// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
`timescale 1ns/1ps
`include "../shared/constants.v"

// ================================================================
// tb_neuron_processing (annAcc variant)
//
// Tests the annAcc NP pipeline:
//   - Reads accumulated potential from syn_curr_mem (written by SP)
//   - Applies RELU or LUT threshold
//   - Decays post-threshold activation (Q0.32 multiply)
//   - Writes decayed value to pot_mem and packed activations to spike_mem
//   - No bias_curr read; no syn_curr write-back
//
// Configuration: 4 neurons (last_neuron_idx=3), all widths 32-bit,
// pot_decay = 0.5 (0x80000000 in Q0.32).
//
// SRAM layout
//   syn_curr  base=  0: words 0..3  (read-only from NP)
//   lut       base= 64: words 64..67 (thresh_mem, read-only; used in LUT mode)
//   pot       base=128: words 128..131 (write-only from NP)
//   spike     base=192: words 192..195 (write-only from NP)
//
// Tests
// -----
//   1. RELU mode: positive and negative potentials
//      syn_curr = [10, -5, 30, -1]
//      Positive → act_out = potential, decayed = floor(potential/2)
//      Negative → act_out = 0, decayed = 0
//   2. LUT mode: 32-bit LUT entries at thresh_base
//      syn_curr = [0, 1, 2, 3] (used as LUT indices)
//      LUT[0..3] = [50, 75, 100, 125]
//      act_out = LUT[syn_curr], decayed = floor(LUT[syn_curr]/2)
//   3. ABS mode: act_out = |syn_curr|, decayed = floor(|syn_curr|/2)
//      syn_curr = [-20, 40, -10, 0]
// ================================================================
module tb_neuron_processing;

localparam NEURON_IDX_SZ        = 4;
localparam SYN_CURR_IDX_SZ      = 10;
localparam SYN_CURR_DATA_IDX_SZ = 5;
localparam SYN_CURR_SLICE_SZ    = 3;
localparam SYN_CURR_SLICE_BITS  = 32;
localparam LUT_IDX_SZ           = 8;
localparam LUT_DATA_IDX_SZ      = 5;
localparam LUT_SLICE_SZ         = 3;
localparam LUT_SLICE_BITS       = 32;  // 32-bit LUT entries for clean addressing
localparam POT_SLICE_SZ         = 3;
localparam POT_SLICE_BITS       = 32;
localparam SRAM_DEPTH           = 256;

reg clk, reset;

// Config
reg [NEURON_IDX_SZ-1:0]      last_neuron_idx;
reg [`ADDR_SIZE-1:0]          syn_curr_base_addr;
reg [`ADDR_SIZE-1:0]          thresh_base_addr;
reg [`ADDR_SIZE-1:0]          pot_base_addr;
reg [`ADDR_SIZE-1:0]          spike_base_addr;
reg [SYN_CURR_SLICE_SZ-1:0]  syn_curr_sz;
reg [POT_SLICE_SZ-1:0]       pot_sz;
reg [2:0]                     lut_out_sz;
reg [2:0]                     act_out_sz;
reg                     [1:0] thresh_op;
reg [31:0]                    pot_decay_mult;

// Scheduler
reg                           start_new_block;
reg [`TGT_ACC_SZ-1:0]        target_acc;
reg [`SCH_ENTRY_SZ-1:0]      buffer_info;
reg [`PIN_BITS-1:0]           src1_buff_addr, src2_buff_addr,
                               src3_buff_addr, tgt_buff_addr,
                               weight_row_len;

wire                          neuron_proc_finished;
wire                          acc_busy;
wire                          acc_finished;

// syn_curr memory (read-only from NP)
wire                          syn_curr_mem_wr;
wire                          syn_curr_mem_rd;
reg                           syn_curr_mem_wait;
wire [`ADDR_SIZE-1:0]         syn_curr_mem_addr;
wire [`POT_BITS-1:0]          syn_curr_mem_data_wr;
reg  [`POT_BITS-1:0]          syn_curr_mem_data_rd;

// LUT memory via thresh_mem ports (read-only from NP)
wire                          thresh_mem_rd;
reg                           thresh_mem_wait;
wire [`ADDR_SIZE-1:0]         thresh_mem_addr;
reg  [`WTD_BITS-1:0]          thresh_mem_data;

// pot memory (write-only from NP)
wire                          pot_mem_wr;
wire                          pot_mem_rd;
reg                           pot_mem_wait;
wire [`ADDR_SIZE-1:0]         pot_mem_addr;
wire [`POT_BITS-1:0]          pot_mem_data_wr;
reg  [`POT_BITS-1:0]          pot_mem_data_rd;

// spike/act_out memory (write-only from NP)
wire                          spike_mem_wr;
reg                           spike_mem_wait;
wire [`ADDR_SIZE-1:0]         spike_mem_addr;
wire [`ACT_BITS-1:0]          spike_mem_data;

neuron_processing #(
    .NEURON_IDX_SZ        (NEURON_IDX_SZ),
    .SYN_CURR_IDX_SZ      (SYN_CURR_IDX_SZ),
    .SYN_CURR_DATA_IDX_SZ (SYN_CURR_DATA_IDX_SZ),
    .SYN_CURR_SLICE_SZ    (SYN_CURR_SLICE_SZ),
    .SYN_CURR_SLICE_BITS  (SYN_CURR_SLICE_BITS),
    .LUT_IDX_SZ           (LUT_IDX_SZ),
    .LUT_DATA_IDX_SZ      (LUT_DATA_IDX_SZ),
    .LUT_SLICE_SZ         (LUT_SLICE_SZ),
    .LUT_SLICE_BITS       (LUT_SLICE_BITS),
    .POT_SLICE_SZ         (POT_SLICE_SZ),
    .POT_SLICE_BITS       (POT_SLICE_BITS))
dut (
    .clk                    (clk),
    .reset                  (reset),
    .last_neuron_idx_i      (last_neuron_idx),
    .syn_curr_base_addr_i   (syn_curr_base_addr),
    .thresh_base_addr_i     (thresh_base_addr),
    .pot_base_addr_i        (pot_base_addr),
    .spike_base_addr_i      (spike_base_addr),
    .syn_curr_sz_i          (syn_curr_sz),
    .pot_sz_i               (pot_sz),
    .lut_out_sz_i           (lut_out_sz),
    .act_out_sz_i           (act_out_sz),
    .thresh_op_i            (thresh_op),
    .pot_decay_mult_i       (pot_decay_mult),
    .start_new_block_i      (start_new_block),
    .target_acc_i           (target_acc),
    .buffer_info_i          (buffer_info),
    .neuron_proc_finished_o (neuron_proc_finished),
    .acc_busy_o             (acc_busy),
    .acc_finished_o         (acc_finished),
    .src1_buff_addr_i       (src1_buff_addr),
    .src2_buff_addr_i       (src2_buff_addr),
    .src3_buff_addr_i       (src3_buff_addr),
    .tgt_buff_addr_i        (tgt_buff_addr),
    .weight_row_len_i       (weight_row_len),
    .syn_curr_mem_wr_o      (syn_curr_mem_wr),
    .syn_curr_mem_rd_o      (syn_curr_mem_rd),
    .syn_curr_mem_wait_i    (syn_curr_mem_wait),
    .syn_curr_mem_addr_o    (syn_curr_mem_addr),
    .syn_curr_mem_data_o    (syn_curr_mem_data_wr),
    .syn_curr_mem_data_i    (syn_curr_mem_data_rd),
    .thresh_mem_rd_o        (thresh_mem_rd),
    .thresh_mem_wait_i      (thresh_mem_wait),
    .thresh_mem_addr_o      (thresh_mem_addr),
    .thresh_mem_data_i      (thresh_mem_data),
    .pot_mem_wr_o           (pot_mem_wr),
    .pot_mem_rd_o           (pot_mem_rd),
    .pot_mem_wait_i         (pot_mem_wait),
    .pot_mem_addr_o         (pot_mem_addr),
    .pot_mem_data_o         (pot_mem_data_wr),
    .pot_mem_data_i         (pot_mem_data_rd),
    .spike_mem_wr_o         (spike_mem_wr),
    .spike_mem_wait_i       (spike_mem_wait),
    .spike_mem_addr_o       (spike_mem_addr),
    .spike_mem_data_o       (spike_mem_data)
);

// ----------------------------------------------------------------
// SRAM models — 1-cycle read latency, immediate write
// ----------------------------------------------------------------
reg [31:0] syn_curr_sram [0:SRAM_DEPTH-1];
reg [31:0] lut_sram      [0:SRAM_DEPTH-1];
reg [31:0] pot_sram      [0:SRAM_DEPTH-1];
reg [31:0] spike_sram    [0:SRAM_DEPTH-1];

always @(posedge clk)
    if (syn_curr_mem_rd & ~syn_curr_mem_wait)
        syn_curr_mem_data_rd <= syn_curr_sram[syn_curr_mem_addr[7:0]];

always @(posedge clk)
    if (thresh_mem_rd & ~thresh_mem_wait)
        thresh_mem_data <= lut_sram[thresh_mem_addr[7:0]];

always @(posedge clk)
    if (pot_mem_wr & ~pot_mem_wait)
        pot_sram[pot_mem_addr[7:0]] <= pot_mem_data_wr;

always @(posedge clk)
    if (spike_mem_wr & ~spike_mem_wait)
        spike_sram[spike_mem_addr[7:0]] <= spike_mem_data;

initial clk = 0;
always  #5 clk = ~clk;

integer errors, timeout, mi;

// Q0.32 decay reference: floor(val * mult / 2^32)
function [31:0] decayed;
    input signed [31:0] val;
    input        [31:0] mult;
    reg signed [63:0] tmp;
    begin
        tmp    = $signed({{32{val[31]}}, val}) * {32'b0, mult};
        decayed = tmp[63:32];
    end
endfunction

task check_eq;
    input [31:0] got, exp;
    input [255:0] label;
    begin
        if (got !== exp) begin
            $display("FAIL %s: got %0d (0x%08h)  exp %0d (0x%08h)",
                     label, got, got, exp, exp);
            errors = errors + 1;
        end else
            $display("  %s = %0d (OK)", label, got);
    end
endtask

task wait_finished;
    begin
        timeout = 500;
        while (!neuron_proc_finished && timeout > 0) begin
            @(posedge clk); #1;
            timeout = timeout - 1;
        end
        if (timeout == 0) begin
            $display("FAIL: neuron_proc_finished never asserted (timeout)");
            errors = errors + 1;
        end else begin
            $display("  finished after %0d cycles", 500 - timeout);
            @(posedge clk); #1;   // let acc_busy clear
        end
        // Allow packer write-backs to drain
        repeat (20) @(posedge clk); #1;
    end
endtask

initial begin
    errors = 0;

    // ---- Initialise SRAMs ----
    for (mi = 0; mi < SRAM_DEPTH; mi = mi + 1) begin
        syn_curr_sram[mi] = 32'd0;
        lut_sram[mi]      = 32'd0;
        pot_sram[mi]      = 32'd0;
        spike_sram[mi]    = 32'd0;
    end

    // ---- Driver defaults ----
    reset              = 1;
    start_new_block    = 0;
    target_acc         = 1'b0;
    buffer_info        = 0;
    src1_buff_addr     = 0;  src2_buff_addr = 0;
    src3_buff_addr     = 0;  tgt_buff_addr  = 0;
    weight_row_len     = 0;
    last_neuron_idx    = 4'd3;       // neurons 0..3
    syn_curr_base_addr = 30'd0;
    thresh_base_addr   = 30'd64;
    pot_base_addr      = 30'd128;
    spike_base_addr    = 30'd192;
    syn_curr_sz        = 3'b101;    // 32-bit potentials
    pot_sz             = 3'b101;    // 32-bit pot write-back
    lut_out_sz         = 3'b101;    // 32-bit LUT entries
    act_out_sz         = 3'b101;    // 32-bit output activations
    thresh_op          = 2'b00;     // RELU
    pot_decay_mult     = 32'h80000000;  // 0.5 in Q0.32
    syn_curr_mem_wait  = 0;
    thresh_mem_wait    = 0;
    pot_mem_wait       = 0;
    spike_mem_wait     = 0;
    syn_curr_mem_data_rd = 0;
    thresh_mem_data    = 0;
    pot_mem_data_rd    = 0;

    @(posedge clk); #1;
    reset = 0;
    @(posedge clk); #1;

    $display("=== tb_neuron_processing (annAcc) ===");

    // ----------------------------------------------------------
    // Test 1: RELU mode — positive and negative potentials
    //
    // syn_curr = [10, -5, 30, -1]
    //   neuron 0: 10 > 0 → act=10, decayed=5
    //   neuron 1: -5 < 0 → act=0,  decayed=0
    //   neuron 2: 30 > 0 → act=30, decayed=15
    //   neuron 3: -1 < 0 → act=0,  decayed=0
    // ----------------------------------------------------------
    $display("Test 1: RELU mode");

    syn_curr_sram[0] = 32'd10;
    syn_curr_sram[1] = -32'd5;     // 0xFFFFFFFB
    syn_curr_sram[2] = 32'd30;
    syn_curr_sram[3] = -32'd1;     // 0xFFFFFFFF
    thresh_op = 2'b00;

    start_new_block = 1;
    @(posedge clk); #1;
    start_new_block = 0;

    wait_finished;

    if (acc_busy) begin
        $display("FAIL T1: acc_busy_o still high after finished");
        errors = errors + 1;
    end

    check_eq(spike_sram[192], 32'd10, "T1 spike[0] act_out neuron 0");
    check_eq(spike_sram[193], 32'd0,  "T1 spike[1] act_out neuron 1 (neg→0)");
    check_eq(spike_sram[194], 32'd30, "T1 spike[2] act_out neuron 2");
    check_eq(spike_sram[195], 32'd0,  "T1 spike[3] act_out neuron 3 (neg→0)");

    check_eq(pot_sram[128], decayed(32'd10, 32'h80000000), "T1 pot[0] decayed");
    check_eq(pot_sram[129], 32'd0,                         "T1 pot[1] decayed (0)");
    check_eq(pot_sram[130], decayed(32'd30, 32'h80000000), "T1 pot[2] decayed");
    check_eq(pot_sram[131], 32'd0,                         "T1 pot[3] decayed (0)");

    // ----------------------------------------------------------
    // Test 2: LUT mode — 32-bit LUT entries
    //
    // syn_curr = [0, 1, 2, 3] → used as LUT word addresses
    // LUT: lut_sram[64]=50, [65]=75, [66]=100, [67]=125
    //   neuron 0: lut_index=0 → act=50,  decayed=25
    //   neuron 1: lut_index=1 → act=75,  decayed=37
    //   neuron 2: lut_index=2 → act=100, decayed=50
    //   neuron 3: lut_index=3 → act=125, decayed=62
    // ----------------------------------------------------------
    $display("Test 2: LUT mode");

    for (mi = 0; mi < SRAM_DEPTH; mi = mi + 1) begin
        syn_curr_sram[mi] = 32'd0;
        pot_sram[mi]      = 32'd0;
        spike_sram[mi]    = 32'd0;
    end

    syn_curr_sram[0] = 32'd0;
    syn_curr_sram[1] = 32'd1;
    syn_curr_sram[2] = 32'd2;
    syn_curr_sram[3] = 32'd3;

    lut_sram[64] = 32'd50;
    lut_sram[65] = 32'd75;
    lut_sram[66] = 32'd100;
    lut_sram[67] = 32'd125;

    thresh_op = 2'b01;

    start_new_block = 1;
    @(posedge clk); #1;
    start_new_block = 0;

    wait_finished;

    check_eq(spike_sram[192], 32'd50,  "T2 spike[0] lut→50");
    check_eq(spike_sram[193], 32'd75,  "T2 spike[1] lut→75");
    check_eq(spike_sram[194], 32'd100, "T2 spike[2] lut→100");
    check_eq(spike_sram[195], 32'd125, "T2 spike[3] lut→125");

    check_eq(pot_sram[128], decayed(32'd50,  32'h80000000), "T2 pot[0] decayed");
    check_eq(pot_sram[129], decayed(32'd75,  32'h80000000), "T2 pot[1] decayed");
    check_eq(pot_sram[130], decayed(32'd100, 32'h80000000), "T2 pot[2] decayed");
    check_eq(pot_sram[131], decayed(32'd125, 32'h80000000), "T2 pot[3] decayed");

    // ----------------------------------------------------------
    // Test 3: ABS mode — act_out = |syn_curr|
    //
    // syn_curr = [-20, 40, -10, 0]
    //   neuron 0: |-20| = 20, decayed = 10
    //   neuron 1: |40|  = 40, decayed = 20
    //   neuron 2: |-10| = 10, decayed = 5
    //   neuron 3: |0|   = 0,  decayed = 0
    // ----------------------------------------------------------
    $display("Test 3: ABS mode");

    for (mi = 0; mi < SRAM_DEPTH; mi = mi + 1) begin
        syn_curr_sram[mi] = 32'd0;
        pot_sram[mi]      = 32'd0;
        spike_sram[mi]    = 32'd0;
    end

    syn_curr_sram[0] = -32'd20;
    syn_curr_sram[1] =  32'd40;
    syn_curr_sram[2] = -32'd10;
    syn_curr_sram[3] =  32'd0;

    thresh_op = 2'b10;

    start_new_block = 1;
    @(posedge clk); #1;
    start_new_block = 0;

    wait_finished;

    check_eq(spike_sram[192], 32'd20, "T3 spike[0] |−20|=20");
    check_eq(spike_sram[193], 32'd40, "T3 spike[1] |40|=40");
    check_eq(spike_sram[194], 32'd10, "T3 spike[2] |−10|=10");
    check_eq(spike_sram[195], 32'd0,  "T3 spike[3] |0|=0");

    check_eq(pot_sram[128], decayed(32'd20, 32'h80000000), "T3 pot[0] decayed");
    check_eq(pot_sram[129], decayed(32'd40, 32'h80000000), "T3 pot[1] decayed");
    check_eq(pot_sram[130], decayed(32'd10, 32'h80000000), "T3 pot[2] decayed");
    check_eq(pot_sram[131], 32'd0,                         "T3 pot[3] decayed (0)");

    $display("=== tb_neuron_processing: %0d failure(s) ===", errors);
    if (errors == 0) $display("PASS"); else $display("FAIL");
    $finish;
end

initial begin
    #200000;
    $display("FAIL: global simulation timeout");
    $finish;
end

endmodule
