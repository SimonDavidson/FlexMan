// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
`timescale 1ns/1ps
`include "../shared/constants.v"

// ================================================================
// tb_neuron_processing
//
// Tests the neuron update pipeline: synaptic current + bias +
// potential → update_state_for_neuron → decayed write-backs and
// spike output via packers.
//
// Configuration
// -------------
//   4 neurons (last_neuron_idx=3), all data widths = 32-bit
//   syn_curr_decay = pot_decay = 0.5  (0x80000000 in Q0.32)
//
// SRAM layout (separate arrays per memory interface)
//   syn_curr  base=  0: words 0..3   (rd+wr)
//   bias      base=  0: words 0..3   (rd only, separate SRAM)
//   thresh    base= 64: words 64..67 (rd only)
//   pot       base=128: words 128..131 (rd+wr)
//   spike     base=192: word 192     (wr only, 4 neurons in 1 word)
//
// Tests
// -----
//   1. No-spike: syn=[10,20,30,40], bias=5, thresh=100, pot=0
//      → neuron_proc_finished_o asserts within 500 cycles
//      → acc_busy_o de-asserts when finished
//      → syn_curr write-back: sram[0]=5   (floor(10*0.5))
//      → pot write-back:      sram[128]=7 (floor((0+10+5)*0.5))
//   2. Spike: syn_curr[0]=90, bias=10, thresh=100 → sum=100≥100
//      → pot_sram[128]=0 (reset-to-zero on spike)
//      → spike_sram[192][0]=1 (neuron 0 spike bit set)
// ================================================================
module tb_neuron_processing;

localparam NEURON_IDX_SZ         = 4;
localparam SYN_CURR_IDX_SZ       = 10;
localparam SYN_CURR_DATA_IDX_SZ  = 5;
localparam SYN_CURR_SLICE_SZ     = 3;
localparam SYN_CURR_SLICE_BITS   = 32;
localparam BIAS_CURR_IDX_SZ      = 2;
localparam BIAS_CURR_DATA_IDX_SZ = 5;
localparam BIAS_CURR_SLICE_SZ    = 3;
localparam BIAS_CURR_SLICE_BITS  = 32;
localparam POT_IDX_SZ            = 2;
localparam POT_DATA_IDX_SZ       = 5;
localparam POT_SLICE_SZ          = 3;
localparam POT_SLICE_BITS        = 32;
localparam SPIKE_IDX_SZ          = 2;
localparam SPIKE_DATA_IDX_SZ     = 5;
localparam SPIKE_SLICE_SZ        = 3;
localparam SPIKE_SLICE_BITS      = 32;

reg clk, reset;

// Config registers
reg [NEURON_IDX_SZ-1:0]        last_neuron_idx;
reg [`ADDR_SIZE-1:0]            syn_curr_base_addr;
reg [`ADDR_SIZE-1:0]            bias_curr_base_addr;
reg [`ADDR_SIZE-1:0]            thresh_base_addr;
reg [`ADDR_SIZE-1:0]            pot_base_addr;
reg [`ADDR_SIZE-1:0]            spike_base_addr;
reg [SYN_CURR_SLICE_SZ-1:0]    syn_curr_sz;
reg [BIAS_CURR_SLICE_SZ-1:0]   bias_curr_sz;
reg [POT_SLICE_SZ-1:0]         pot_sz;
reg [4:0]                       bin_point_syn_curr;
reg [31:0]                      syn_curr_decay_mult;
reg [31:0]                      pot_decay_mult;
reg                             sub_on_fire;
reg                             clear_pot;

// Scheduler interface
reg                             start_new_block;
reg [`TGT_ACC_SZ-1:0]          target_acc;
reg [`SCH_ENTRY_SZ-1:0]        buffer_info;
reg [`PIN_BITS-1:0]             src1_buff_addr;
reg [`PIN_BITS-1:0]             src2_buff_addr;
reg [`PIN_BITS-1:0]             src3_buff_addr;
reg [`PIN_BITS-1:0]             tgt_buff_addr;
reg [`PIN_BITS-1:0]             weight_row_len;

wire                            neuron_proc_finished;
wire                            acc_busy;
wire                            acc_finished;

// Memory ports — syn_curr (rd + wr)
wire                            syn_curr_mem_wr;
wire                            syn_curr_mem_rd;
reg                             syn_curr_mem_wait;
wire [`ADDR_SIZE-1:0]           syn_curr_mem_addr;
wire [`POT_BITS-1:0]            syn_curr_mem_data_wr;
reg  [`POT_BITS-1:0]            syn_curr_mem_data_rd;

// Memory ports — bias (rd only)
wire                            bias_curr_mem_rd;
reg                             bias_curr_mem_wait;
wire [`ADDR_SIZE-1:0]           bias_curr_mem_addr;
reg  [`WTD_BITS-1:0]            bias_curr_mem_data;

// Memory ports — threshold (rd only)
wire                            thresh_mem_rd;
reg                             thresh_mem_wait;
wire [`ADDR_SIZE-1:0]           thresh_mem_addr;
reg  [`WTD_BITS-1:0]            thresh_mem_data;

// Memory ports — potential (rd + wr)
wire                            pot_mem_wr;
wire                            pot_mem_rd;
reg                             pot_mem_wait;
wire [`ADDR_SIZE-1:0]           pot_mem_addr;
wire [`POT_BITS-1:0]            pot_mem_data_wr;
reg  [`POT_BITS-1:0]            pot_mem_data_rd;

// Memory ports — spike (wr only)
wire                            spike_mem_wr;
reg                             spike_mem_wait;
wire [`ADDR_SIZE-1:0]           spike_mem_addr;
wire [`ACT_BITS-1:0]            spike_mem_data;

neuron_processing #(
    .NEURON_IDX_SZ         (NEURON_IDX_SZ),
    .SYN_CURR_IDX_SZ       (SYN_CURR_IDX_SZ),
    .SYN_CURR_DATA_IDX_SZ  (SYN_CURR_DATA_IDX_SZ),
    .SYN_CURR_SLICE_SZ     (SYN_CURR_SLICE_SZ),
    .SYN_CURR_SLICE_BITS   (SYN_CURR_SLICE_BITS),
    .BIAS_CURR_IDX_SZ      (BIAS_CURR_IDX_SZ),
    .BIAS_CURR_DATA_IDX_SZ (BIAS_CURR_DATA_IDX_SZ),
    .BIAS_CURR_SLICE_SZ    (BIAS_CURR_SLICE_SZ),
    .BIAS_CURR_SLICE_BITS  (BIAS_CURR_SLICE_BITS),
    .POT_IDX_SZ            (POT_IDX_SZ),
    .POT_DATA_IDX_SZ       (POT_DATA_IDX_SZ),
    .POT_SLICE_SZ          (POT_SLICE_SZ),
    .POT_SLICE_BITS        (POT_SLICE_BITS),
    .SPIKE_IDX_SZ          (SPIKE_IDX_SZ),
    .SPIKE_DATA_IDX_SZ     (SPIKE_DATA_IDX_SZ),
    .SPIKE_SLICE_SZ        (SPIKE_SLICE_SZ),
    .SPIKE_SLICE_BITS      (SPIKE_SLICE_BITS))
dut (
    .clk                    (clk),
    .reset                  (reset),
    .last_neuron_idx_i      (last_neuron_idx),
    .syn_curr_base_addr_i   (syn_curr_base_addr),
    .bias_curr_base_addr_i  (bias_curr_base_addr),
    .thresh_base_addr_i     (thresh_base_addr),
    .pot_base_addr_i        (pot_base_addr),
    .spike_base_addr_i      (spike_base_addr),
    .syn_curr_sz_i          (syn_curr_sz),
    .bias_curr_sz_i         (bias_curr_sz),
    .pot_sz_i               (pot_sz),
    .bin_point_syn_curr_i   (bin_point_syn_curr),
    .syn_curr_decay_mult_i  (syn_curr_decay_mult),
    .pot_decay_mult_i       (pot_decay_mult),
    .sub_on_fire_i          (sub_on_fire),
    .clear_pot_i            (clear_pot),
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
    .bias_curr_mem_rd_o     (bias_curr_mem_rd),
    .bias_curr_mem_wait_i   (bias_curr_mem_wait),
    .bias_curr_mem_addr_o   (bias_curr_mem_addr),
    .bias_curr_mem_data_i   (bias_curr_mem_data),
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
// SRAM models — synchronous, 1-cycle read latency, immediate write
// ----------------------------------------------------------------
reg [31:0] syn_curr_sram [0:255];
reg [31:0] bias_sram     [0:255];
reg [31:0] thresh_sram   [0:255];
reg [31:0] pot_sram      [0:255];
reg [31:0] spike_sram    [0:255];

// syn_curr: read and write-back share the same array.
// The DUT blocks the cache read while syn_curr_wb_wr is active,
// so rd and wr are never simultaneous on this interface.
always @(posedge clk) begin
    if (syn_curr_mem_rd)
        syn_curr_mem_data_rd <= syn_curr_sram[syn_curr_mem_addr[7:0]];
    if (syn_curr_mem_wr)
        syn_curr_sram[syn_curr_mem_addr[7:0]] <= syn_curr_mem_data_wr;
end

always @(posedge clk)
    if (bias_curr_mem_rd)
        bias_curr_mem_data <= bias_sram[bias_curr_mem_addr[7:0]];

always @(posedge clk)
    if (thresh_mem_rd)
        thresh_mem_data <= thresh_sram[thresh_mem_addr[7:0]];

// pot: read and write-back share the same array.
always @(posedge clk) begin
    if (pot_mem_rd)
        pot_mem_data_rd <= pot_sram[pot_mem_addr[7:0]];
    if (pot_mem_wr)
        pot_sram[pot_mem_addr[7:0]] <= pot_mem_data_wr;
end

always @(posedge clk)
    if (spike_mem_wr)
        spike_sram[spike_mem_addr[7:0]] <= spike_mem_data;

initial clk = 0;
always  #5 clk = ~clk;

integer errors, timeout, mi;

initial begin
    errors = 0;

    // ---- Initialise SRAMs ----
    for (mi = 0; mi < 256; mi = mi + 1) begin
        syn_curr_sram[mi] = 32'd0;
        bias_sram[mi]     = 32'd0;
        thresh_sram[mi]   = 32'd0;
        pot_sram[mi]      = 32'd0;
        spike_sram[mi]    = 32'd0;
    end
    // syn_curr at base=0, 32-bit slices → word k holds neuron k
    syn_curr_sram[0] = 32'd10;
    syn_curr_sram[1] = 32'd20;
    syn_curr_sram[2] = 32'd30;
    syn_curr_sram[3] = 32'd40;
    // bias at base=0 (separate SRAM)
    bias_sram[0] = 32'd5;
    bias_sram[1] = 32'd5;
    bias_sram[2] = 32'd5;
    bias_sram[3] = 32'd5;
    // thresh at base=64
    thresh_sram[64] = 32'd100;
    thresh_sram[65] = 32'd100;
    thresh_sram[66] = 32'd100;
    thresh_sram[67] = 32'd100;
    // pot at base=128: all zero initially
    // spike at base=192: write-only, starts zero

    // ---- Driver defaults ----
    reset               = 1;
    start_new_block     = 0;
    target_acc          = 0;       // matches TGT_ACC_ID default 3'b000 → LSB = 0
    buffer_info         = 0;
    src1_buff_addr      = 0;
    src2_buff_addr      = 0;
    src3_buff_addr      = 0;
    tgt_buff_addr       = 0;
    weight_row_len      = 0;
    last_neuron_idx     = 4'd3;    // 4 neurons: indices 0..3
    syn_curr_base_addr  = 30'd0;
    bias_curr_base_addr = 30'd0;
    thresh_base_addr    = 30'd64;
    pot_base_addr       = 30'd128;
    spike_base_addr     = 30'd192;
    syn_curr_sz         = 3'b101;  // 32-bit elements (1 per word)
    bias_curr_sz        = 3'b101;
    pot_sz              = 3'b101;
    bin_point_syn_curr  = 5'd0;
    syn_curr_decay_mult = 32'h80000000;  // 0.5 in Q0.32
    pot_decay_mult      = 32'h80000000;
    sub_on_fire         = 0;
    clear_pot           = 0;
    syn_curr_mem_wait   = 0;
    bias_curr_mem_wait  = 0;
    thresh_mem_wait     = 0;
    pot_mem_wait        = 0;
    spike_mem_wait      = 0;
    syn_curr_mem_data_rd = 0;
    bias_curr_mem_data   = 0;
    thresh_mem_data      = 0;
    pot_mem_data_rd      = 0;

    @(posedge clk); #1;
    reset = 0;
    @(posedge clk); #1;

    $display("=== tb_neuron_processing ===");

    // ----------------------------------------------------------
    // Test 1: No-spike pass
    // syn=[10,20,30,40], bias=5, thresh=100, pot=0, decay=0.5
    //
    // For neuron 0:
    //   new_pot = 0 + 10 + 5 = 15  < 100  → no spike
    //   syn_curr_wb = floor(10 * 0.5) = 5
    //   pot_wb      = floor(15 * 0.5) = 7
    // ----------------------------------------------------------
    $display("Test 1: no-spike, 4 neurons, 32-bit slices");

    start_new_block = 1;
    @(posedge clk); #1;
    start_new_block = 0;

    timeout = 500;
    while (!neuron_proc_finished && timeout > 0) begin
        @(posedge clk); #1;
        timeout = timeout - 1;
    end

    if (timeout == 0) begin
        $display("FAIL T1: neuron_proc_finished never asserted (timeout)");
        errors = errors + 1;
    end else begin
        $display("  neuron_proc_finished after %0d cycles", 500 - timeout);
        // neuron_proc_finished_o fires combinationally one cycle before
        // neuron_update_running_r is cleared by its NBA. Clock one more
        // cycle so acc_busy_o (= running_r) has been cleared.
        @(posedge clk); #1;
        if (acc_busy) begin
            $display("FAIL T1: acc_busy_o still high after finished");
            errors = errors + 1;
        end
    end

    // Allow packer write-backs to drain (2+ cycles needed; use 20)
    repeat (20) @(posedge clk);
    #1;

    // Check syn_curr write-back: neuron 0 → floor(10/2) = 5
    if (syn_curr_sram[0] !== 32'd5) begin
        $display("FAIL T1: syn_curr_sram[0] expected 5, got %0d", syn_curr_sram[0]);
        errors = errors + 1;
    end else
        $display("  syn_curr_sram[0] = %0d (OK)", syn_curr_sram[0]);

    // Check pot write-back: neuron 0 → floor((0+10+5)/2) = floor(15/2) = 7
    if (pot_sram[128] !== 32'd7) begin
        $display("FAIL T1: pot_sram[128] expected 7, got %0d", pot_sram[128]);
        errors = errors + 1;
    end else
        $display("  pot_sram[128] = %0d (OK)", pot_sram[128]);

    // ----------------------------------------------------------
    // Test 2: Spike — neuron 0 fires
    // syn_curr[0]=90, bias=10, thresh=100 → sum=100 ≥ 100 → spike
    //   pot_wb[0]  = 0   (reset-to-zero on spike)
    //   spike word = 0x00000001 (bit 0 set: packer places neuron n at bit n)
    // ----------------------------------------------------------
    $display("Test 2: spike for neuron 0");

    // Reset mutable SRAMs; keep thresh unchanged (still 100 at addr 64..67)
    for (mi = 0; mi < 256; mi = mi + 1) begin
        syn_curr_sram[mi] = 32'd0;
        pot_sram[mi]      = 32'd0;
        spike_sram[mi]    = 32'd0;
    end
    syn_curr_sram[0] = 32'd90;   // neuron 0: large current → will spike
    // bias already at 10 from test 1 final state — reset to 10 explicitly
    bias_sram[0] = 32'd10;
    bias_sram[1] = 32'd10;
    bias_sram[2] = 32'd10;
    bias_sram[3] = 32'd10;

    start_new_block = 1;
    @(posedge clk); #1;
    start_new_block = 0;

    timeout = 500;
    while (!neuron_proc_finished && timeout > 0) begin
        @(posedge clk); #1;
        timeout = timeout - 1;
    end

    if (timeout == 0) begin
        $display("FAIL T2: neuron_proc_finished timeout");
        errors = errors + 1;
    end else begin
        $display("  neuron_proc_finished after %0d cycles", 500 - timeout);

        repeat (20) @(posedge clk);
        #1;

        // Neuron 0 spiked → potential reset to 0, then decayed: 0*0.5 = 0
        if (pot_sram[128] !== 32'd0) begin
            $display("FAIL T2: pot_sram[128] expected 0 (spike resets potential), got %0d",
                     pot_sram[128]);
            errors = errors + 1;
        end else
            $display("  pot_sram[128] = 0 (OK — spike reset)");

        // spike_sram[192]: packer places neuron n at bit n of the word.
        // Neuron 0 spiked → bit 0 = 1 → expected 0x00000001.
        if (spike_sram[192] !== 32'h00000001) begin
            $display("FAIL T2: spike_sram[192] = 0x%08h, expected 0x00000001",
                     spike_sram[192]);
            errors = errors + 1;
        end else
            $display("  spike_sram[192] = 0x%08h (OK — neuron 0 spike at bit 0)",
                     spike_sram[192]);
    end

    // ----------------------------------------------------------
    // Test 3: clear_pot=1 — potential memory ignored, all neurons
    //         start this task from zero potential.
    // syn_curr[0]=10, bias=5, thresh=100, pot_sram[128]=1000
    // With clear_pot=1, potential_i fed to update_state = 0 (not 1000)
    //   new_pot = 0 + 10 + 5 = 15 < 100 → no spike
    //   pot_wb  = floor(15 * 0.5) = 7   (NOT floor((1000+10+5)*0.5))
    // ----------------------------------------------------------
    $display("Test 3: clear_pot=1 — potential starts at zero regardless of SRAM");

    for (mi = 0; mi < 256; mi = mi + 1) begin
        syn_curr_sram[mi] = 32'd0;
        pot_sram[mi]      = 32'd0;
        spike_sram[mi]    = 32'd0;
    end
    syn_curr_sram[0] = 32'd10;
    syn_curr_sram[1] = 32'd20;
    syn_curr_sram[2] = 32'd30;
    syn_curr_sram[3] = 32'd40;
    bias_sram[0] = 32'd5;
    bias_sram[1] = 32'd5;
    bias_sram[2] = 32'd5;
    bias_sram[3] = 32'd5;
    // thresh_sram[64..67] already = 100 from Test 1 init

    // Set pot_sram to a large value that would change the result if not cleared
    pot_sram[128] = 32'd1000;
    pot_sram[129] = 32'd1000;
    pot_sram[130] = 32'd1000;
    pot_sram[131] = 32'd1000;

    clear_pot = 1;
    start_new_block = 1;
    @(posedge clk); #1;
    start_new_block = 0;

    timeout = 500;
    while (!neuron_proc_finished && timeout > 0) begin
        @(posedge clk); #1;
        timeout = timeout - 1;
    end

    if (timeout == 0) begin
        $display("FAIL T3: neuron_proc_finished timeout");
        errors = errors + 1;
    end else begin
        $display("  neuron_proc_finished after %0d cycles", 500 - timeout);
        repeat (20) @(posedge clk);
        #1;

        // Neuron 0: new_pot = 0+10+5=15, pot_wb = floor(15*0.5) = 7
        if (pot_sram[128] !== 32'd7) begin
            $display("FAIL T3: pot_sram[128] expected 7 (clear_pot zeroed input), got %0d",
                     pot_sram[128]);
            errors = errors + 1;
        end else
            $display("  pot_sram[128] = %0d (OK — clear_pot zeroed initial potential)", pot_sram[128]);

        // No spike → spike_sram[192] should be 0
        if (spike_sram[192] !== 32'd0) begin
            $display("FAIL T3: spike_sram[192] expected 0 (no spike), got 0x%08h",
                     spike_sram[192]);
            errors = errors + 1;
        end else
            $display("  spike_sram[192] = 0 (OK — no spike)");
    end
    clear_pot = 0;

    $display("=== tb_neuron_processing: %0d failure(s) ===", errors);
    if (errors == 0) $display("PASS"); else $display("FAIL");
    $finish;
end

// Safety watchdog
initial begin
    #200000;
    $display("FAIL: global simulation timeout");
    $finish;
end

endmodule
