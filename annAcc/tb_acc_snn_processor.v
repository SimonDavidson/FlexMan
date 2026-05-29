// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
`timescale 1ns/1ps

`include "../shared/constants.v"

// ====================================================================
//  tb_acc_snn_processor  (annAcc variant)
//
//  Configuration
//  -------------
//  weight_mode    = full connectivity (2'b00)
//  Input layer    : 2 neurons (in_x_len=2, in_y_len=1)
//  Output layer   : 2 neurons (out_x_len=2, out_y_len=1, last_neuron_idx=1)
//  Weights        : 8-bit value=10; weight_sram filled with 0x0A0A_0A0A
//  Activations    : 8-bit act=1; act_sram filled with 0x0101_0101
//  sp_act_sz      : 3 (8-bit elements at runtime, configured via AXI 0x98)
//  syn_curr_sz    : 5 (32-bit elements, one per neuron word)
//  act_out_sz     : 5 (32-bit output activation)
//  lut_out_sz     : 3 (8-bit LUT entries)
//  pot_decay_mult : 0x8000_0000 (0.5 in Q0.32)
//
//  SP: syn_curr = sum(act × weight) = 2 × (1 × 10) = 20 per output neuron
//
//  Test 1 — RELU mode (thresh_op=0):
//    RELU(20) = 20 (positive pass-through)
//    decayed_act = floor(20 × 0.5) = 10
//    spike_sram[60/61] = 32'd20
//    pot_sram[50/51]   = 32'd10
//
//  Test 2 — LUT mode (thresh_op=1):
//    Reset memories; lut_base=40, lut[20]=0xAA=170
//    syn_curr=20 → lut_index=20 → act_out=170
//    decayed_act = floor(170 × 0.5) = 85
//    spike_sram[60/61] = 32'h0000_00AA
//    pot_sram[50/51]   = 32'd85
//
//  Test 3 — ABS mode (thresh_op=2):
//    weight_sram -> 0xF6F6_F6F6 (8-bit signed -10 per slot); act=1
//    SP: syn_curr = 2 × (1 × -10) = -20 per output neuron
//    NP: ABS(-20) = 20, decayed = floor(20 × 0.5) = 10
//    spike_sram[60/61] = 32'd20
//    pot_sram[50/51]   = 32'd10
// ====================================================================

// CONFIG PARAMS START
`define TGT_ACC_ID 'h0
`define TGT_CONFIG_BASE_ADDR 32'hFFFFFFFF
`define NUM_TIMESTEPS 32
`define IN_DATA_SZ 32
`define X_INPUT_SZ 5
`define Y_INPUT_SZ 5
`define X_OUTPUT_SZ 5
`define Y_OUTPUT_SZ 5
`define X_KERNEL_SZ 3
`define Y_KERNEL_SZ 3
`define X_STEP_SZ   3
`define Y_STEP_SZ   3
`define ELEMS_PER_ROW   4
`define ROWS_PER_NEURON 4
`define ELEM_SZ 8
`define WEIGHT_SLICE_SZ 5
`define WEIGHT_IDX_SZ 10
`define WEIGHT_DATA_IDX_SZ 5
`define ACT_SLICE_SZ 5      // annAcc: max 32-bit act bus; runtime sz via AXI 0x98
`define ACT_IDX_SZ 10
`define ACT_DATA_IDX_SZ 5
`define SYN_CURR_IDX_SZ 10
`define SYN_CURR_DATA_IDX_SZ 5
`define SYN_CURR_SLICE_SZ 3
`define SYN_CURR_SLICE_BITS 32
`define BIAS_CURR_IDX_SZ 10
`define BIAS_CURR_DATA_IDX_SZ 5
`define BIAS_CURR_SLICE_SZ 3
`define BIAS_CURR_SLICE_BITS 8
`define NP_LUT_IDX_SZ 8
`define NP_LUT_DATA_IDX_SZ 5
`define NP_LUT_SLICE_SZ 3
`define NP_LUT_SLICE_BITS 8
`define POT_SLICE_SZ 3
`define POT_SLICE_BITS 32
// CONFIG PARAMS END

module tb_acc_snn_processor;

    localparam CLK_PERIOD = 10;
    localparam MEM_DEPTH  = 256;

    // ----------------------------------------------------------------
    // Clock & reset
    // ----------------------------------------------------------------
    reg clk;
    reg reset;

    initial clk = 1'b0;
    always #(CLK_PERIOD/2) clk = ~clk;

    // ----------------------------------------------------------------
    // DUT inputs
    // ----------------------------------------------------------------
    reg                      sys_req_i   = 1'b0;
    reg             [31:0]   sys_addr_i  = 32'b0;
    reg             [31:0]   sys_data_i  = 32'b0;

    reg                      start_new_block_i = 1'b0;
    reg   [`TGT_ACC_SZ-1:0]  target_acc_i      = {`TGT_ACC_SZ{1'b0}};
    reg [`SCH_ENTRY_SZ-1:0]  buffer_info_i     = {`SCH_ENTRY_SZ{1'b0}};

    reg [`PIN_BITS-1:0] sp_src1_buff_addr_i  = {`PIN_BITS{1'b0}};
    reg [`PIN_BITS-1:0] sp_src2_buff_addr_i  = {`PIN_BITS{1'b0}};
    reg [`PIN_BITS-1:0] sp_src3_buff_addr_i  = {`PIN_BITS{1'b0}};
    reg [`PIN_BITS-1:0] sp_tgt_buff_addr_i   = {`PIN_BITS{1'b0}};
    reg [`PIN_BITS-1:0] sp_weight_row_len_i  = {`PIN_BITS{1'b0}};

    reg [`PIN_BITS-1:0] np_src1_buff_addr_i  = {`PIN_BITS{1'b0}};
    reg [`PIN_BITS-1:0] np_src2_buff_addr_i  = {`PIN_BITS{1'b0}};
    reg [`PIN_BITS-1:0] np_src3_buff_addr_i  = {`PIN_BITS{1'b0}};
    reg [`PIN_BITS-1:0] np_tgt_buff_addr_i   = {`PIN_BITS{1'b0}};
    reg [`PIN_BITS-1:0] np_weight_row_len_i  = {`PIN_BITS{1'b0}};

    // ----------------------------------------------------------------
    // DUT outputs
    // ----------------------------------------------------------------
    wire                    sys_ack_o;
    wire                    spike_proc_finished_o;
    wire                    acc_busy_o;
    wire                    acc_finished_o;

    wire                    weight_mem_rd_o;
    wire [`ADDR_SIZE-1:0]   weight_mem_addr_o;
    wire                    act_mem_req_o;
    wire [`ADDR_SIZE-1:0]   act_mem_addr_o;
    wire                    syn_curr_mem_wr_o;
    wire                    syn_curr_mem_rd_o;
    wire [`ADDR_SIZE-1:0]   syn_curr_mem_addr_o;
    wire  [`POT_BITS-1:0]   syn_curr_mem_data_o;
    wire                    bias_curr_mem_rd_o;
    wire [`ADDR_SIZE-1:0]   bias_curr_mem_addr_o;
    wire                    thresh_mem_rd_o;
    wire [`ADDR_SIZE-1:0]   thresh_mem_addr_o;
    wire                    pot_mem_wr_o;
    wire                    pot_mem_rd_o;
    wire [`ADDR_SIZE-1:0]   pot_mem_addr_o;
    wire  [`POT_BITS-1:0]   pot_mem_data_o;
    wire                    spike_mem_wr_o;
    wire [`ADDR_SIZE-1:0]   spike_mem_addr_o;
    wire  [`ACT_BITS-1:0]   spike_mem_data_o;

    wire  [`WTD_BITS-1:0]   weight_mem_data_i;
    wire  [`ACT_BITS-1:0]   act_mem_data_i;
    wire  [`POT_BITS-1:0]   syn_curr_mem_data_i;
    wire  [`WTD_BITS-1:0]   bias_curr_mem_data_i;
    wire  [`WTD_BITS-1:0]   thresh_mem_data_i;
    wire  [`POT_BITS-1:0]   pot_mem_data_i;

    wire weight_mem_wait_i    = 1'b0;
    wire act_mem_wait_i       = 1'b0;
    wire syn_curr_mem_wait_i  = 1'b0;
    wire bias_curr_mem_wait_i = 1'b0;
    wire thresh_mem_wait_i    = 1'b0;
    wire pot_mem_wait_i       = 1'b0;
    wire spike_mem_wait_i     = 1'b0;

    // ----------------------------------------------------------------
    // DUT instantiation
    // ----------------------------------------------------------------
    ann_processor # (
    .TGT_ACC_ID(`TGT_ACC_ID),
    .TGT_CONFIG_BASE_ADDR(`TGT_CONFIG_BASE_ADDR),
    .SP_NUM_TIMESTEPS(`NUM_TIMESTEPS),
    .SP_X_INPUT_SZ(`X_INPUT_SZ),
    .SP_Y_INPUT_SZ(`Y_INPUT_SZ),
    .SP_X_OUTPUT_SZ(`X_OUTPUT_SZ),
    .SP_Y_OUTPUT_SZ(`Y_OUTPUT_SZ),
    .SP_X_KERNEL_SZ(`X_KERNEL_SZ),
    .SP_Y_KERNEL_SZ(`Y_KERNEL_SZ),
    .SP_X_KERNEL_OFF_SZ(`X_STEP_SZ),
    .SP_Y_KERNEL_OFF_SZ(`Y_STEP_SZ),
    .SP_X_STEP_SZ(`X_STEP_SZ),
    .SP_Y_STEP_SZ(`Y_STEP_SZ),
    .SP_ELEMS_PER_ROW(`ELEMS_PER_ROW),
    .SP_ROWS_PER_NEURON(`ROWS_PER_NEURON),
    .SP_TIMESTEP_SZ(10),
    .SP_IN_DATA_BITS(32),
    .SP_ELEM_SZ(`ELEM_SZ),
    .SP_ACT_SLICE_SZ(`ACT_SLICE_SZ),
    .SP_ACT_DATA_IDX_SZ(`ACT_DATA_IDX_SZ),
    .SP_WEIGHT_ENTRY_BITS(8),
    .SP_WEIGHT_IDX_SZ(`WEIGHT_IDX_SZ),
    .SP_WEIGHT_SLICE_SZ(`WEIGHT_SLICE_SZ),
    .SP_WEIGHT_DATA_IDX_SZ(`WEIGHT_DATA_IDX_SZ),
    .SP_SYN_CURR_IDX_SZ(`SYN_CURR_IDX_SZ),
    .SP_SYN_CURR_DATA_IDX_SZ(`SYN_CURR_DATA_IDX_SZ),
    .SP_SYN_CURR_SLICE_SZ(`SYN_CURR_SLICE_SZ),
    .SP_SYN_CURR_SLICE_BITS(`SYN_CURR_SLICE_BITS),
    .SP_BIAS_CURR_IDX_SZ(`BIAS_CURR_IDX_SZ),
    .SP_BIAS_CURR_DATA_IDX_SZ(`BIAS_CURR_DATA_IDX_SZ),
    .SP_BIAS_CURR_SLICE_SZ(`BIAS_CURR_SLICE_SZ),
    .SP_BIAS_CURR_SLICE_BITS(`BIAS_CURR_SLICE_BITS),
    .NP_NUM_TIMESTEPS(`NUM_TIMESTEPS),
    .NP_TIMESTEP_SZ(10),
    .NP_IN_DATA_BITS(32),
    .NP_NEURON_IDX_SZ(10),
    .NP_SYN_CURR_IDX_SZ(`SYN_CURR_IDX_SZ),
    .NP_SYN_CURR_DATA_IDX_SZ(`SYN_CURR_DATA_IDX_SZ),
    .NP_SYN_CURR_SLICE_SZ(`SYN_CURR_SLICE_SZ),
    .NP_SYN_CURR_SLICE_BITS(`SYN_CURR_SLICE_BITS),
    .NP_LUT_IDX_SZ(`NP_LUT_IDX_SZ),
    .NP_LUT_DATA_IDX_SZ(`NP_LUT_DATA_IDX_SZ),
    .NP_LUT_SLICE_SZ(`NP_LUT_SLICE_SZ),
    .NP_LUT_SLICE_BITS(`NP_LUT_SLICE_BITS),
    .NP_POT_SLICE_SZ(`POT_SLICE_SZ),
    .NP_POT_SLICE_BITS(`POT_SLICE_BITS),
    .MEM_ADDR_BITS(`ADDR_SIZE)
    )
    u_dut (
        .clk                        (clk),
        .reset                      (reset),

        .sys_req_i                  (sys_req_i),
        .sys_ack_o                  (sys_ack_o),
        .sys_addr_i                 (sys_addr_i),
        .sys_data_i                 (sys_data_i),

        .start_new_block_i          (start_new_block_i),
        .target_acc_i               (target_acc_i),
        .buffer_info_i              (buffer_info_i),
        .spike_proc_finished_o      (spike_proc_finished_o),
        .acc_busy_o                 (acc_busy_o),
        .acc_finished_o             (acc_finished_o),

        .sp_src1_buff_addr_i        (sp_src1_buff_addr_i),
        .sp_src2_buff_addr_i        (sp_src2_buff_addr_i),
        .sp_src3_buff_addr_i        (sp_src3_buff_addr_i),
        .sp_tgt_buff_addr_i         (sp_tgt_buff_addr_i),
        .sp_weight_row_len_i        (sp_weight_row_len_i),

        .np_src1_buff_addr_i        (np_src1_buff_addr_i),
        .np_src2_buff_addr_i        (np_src2_buff_addr_i),
        .np_src3_buff_addr_i        (np_src3_buff_addr_i),
        .np_tgt_buff_addr_i         (np_tgt_buff_addr_i),
        .np_weight_row_len_i        (np_weight_row_len_i),

        .weight_mem_rd_o            (weight_mem_rd_o),
        .weight_mem_wait_i          (weight_mem_wait_i),
        .weight_mem_addr_o          (weight_mem_addr_o),
        .weight_mem_data_i          (weight_mem_data_i),

        .act_mem_req_o              (act_mem_req_o),
        .act_mem_wait_i             (act_mem_wait_i),
        .act_mem_addr_o             (act_mem_addr_o),
        .act_mem_data_i             (act_mem_data_i),

        .syn_curr_mem_wr_o          (syn_curr_mem_wr_o),
        .syn_curr_mem_rd_o          (syn_curr_mem_rd_o),
        .syn_curr_mem_wait_i        (syn_curr_mem_wait_i),
        .syn_curr_mem_addr_o        (syn_curr_mem_addr_o),
        .syn_curr_mem_data_o        (syn_curr_mem_data_o),
        .syn_curr_mem_data_i        (syn_curr_mem_data_i),

        .bias_curr_mem_rd_o         (bias_curr_mem_rd_o),
        .bias_curr_mem_wait_i       (bias_curr_mem_wait_i),
        .bias_curr_mem_addr_o       (bias_curr_mem_addr_o),
        .bias_curr_mem_data_i       (bias_curr_mem_data_i),

        .thresh_mem_rd_o            (thresh_mem_rd_o),
        .thresh_mem_wait_i          (thresh_mem_wait_i),
        .thresh_mem_addr_o          (thresh_mem_addr_o),
        .thresh_mem_data_i          (thresh_mem_data_i),

        .pot_mem_wr_o               (pot_mem_wr_o),
        .pot_mem_rd_o               (pot_mem_rd_o),
        .pot_mem_wait_i             (pot_mem_wait_i),
        .pot_mem_addr_o             (pot_mem_addr_o),
        .pot_mem_data_o             (pot_mem_data_o),
        .pot_mem_data_i             (pot_mem_data_i),

        .spike_mem_wr_o             (spike_mem_wr_o),
        .spike_mem_wait_i           (spike_mem_wait_i),
        .spike_mem_addr_o           (spike_mem_addr_o),
        .spike_mem_data_o           (spike_mem_data_o)
    );

    // ----------------------------------------------------------------
    // SRAM models  (synchronous, 1-cycle read latency)
    // ----------------------------------------------------------------
    sram_model #(.DATA_W(`WTD_BITS), .DEPTH(MEM_DEPTH)) u_weight_mem (
        .clk(clk), .we(1'b0), .re(weight_mem_rd_o),
        .addr(weight_mem_addr_o[7:0]), .wdata({`WTD_BITS{1'b0}}),
        .rdata(weight_mem_data_i));

    sram_model #(.DATA_W(`ACT_BITS), .DEPTH(MEM_DEPTH)) u_act_mem (
        .clk(clk), .we(1'b0), .re(act_mem_req_o),
        .addr(act_mem_addr_o[7:0]), .wdata({`ACT_BITS{1'b0}}),
        .rdata(act_mem_data_i));

    sram_model #(.DATA_W(`POT_BITS), .DEPTH(MEM_DEPTH)) u_syn_curr_mem (
        .clk(clk), .we(syn_curr_mem_wr_o), .re(syn_curr_mem_rd_o),
        .addr(syn_curr_mem_addr_o[7:0]), .wdata(syn_curr_mem_data_o),
        .rdata(syn_curr_mem_data_i));

    sram_model #(.DATA_W(`WTD_BITS), .DEPTH(MEM_DEPTH)) u_bias_curr_mem (
        .clk(clk), .we(1'b0), .re(bias_curr_mem_rd_o),
        .addr(bias_curr_mem_addr_o[7:0]), .wdata({`WTD_BITS{1'b0}}),
        .rdata(bias_curr_mem_data_i));

    // thresh_mem is repurposed as lut_mem in annAcc NP
    sram_model #(.DATA_W(`WTD_BITS), .DEPTH(MEM_DEPTH)) u_thresh_mem (
        .clk(clk), .we(1'b0), .re(thresh_mem_rd_o),
        .addr(thresh_mem_addr_o[7:0]), .wdata({`WTD_BITS{1'b0}}),
        .rdata(thresh_mem_data_i));

    sram_model #(.DATA_W(`POT_BITS), .DEPTH(MEM_DEPTH)) u_pot_mem (
        .clk(clk), .we(pot_mem_wr_o), .re(pot_mem_rd_o),
        .addr(pot_mem_addr_o[7:0]), .wdata(pot_mem_data_o),
        .rdata(pot_mem_data_i));

    wire [`ACT_BITS-1:0] spike_rdata_nc;
    sram_model #(.DATA_W(`ACT_BITS), .DEPTH(MEM_DEPTH)) u_spike_mem (
        .clk(clk), .we(spike_mem_wr_o), .re(1'b0),
        .addr(spike_mem_addr_o[7:0]), .wdata(spike_mem_data_o),
        .rdata(spike_rdata_nc));

    // ----------------------------------------------------------------
    // SRAM initialisation
    //
    // Memory layout:
    //   act_base    =  0   act_sram[0]    = 0x0101_0101 (8-bit act=1, 4 per word)
    //   weight_base = 10   weight_sram[*] = 0x0A0A_0A0A (8-bit weight=10)
    //   syn_curr    = 20   zero initially; SP accumulates here
    //   lut_base    = 40   set per test (thresh_mem repurposed as lut_mem)
    //   pot_base    = 50   written by NP pot_wb_packer (decayed activation)
    //   spike_base  = 60   written by NP spike_packer (activation output)
    // ----------------------------------------------------------------
    integer i_init;
    initial begin
        #1;
        for (i_init = 0; i_init < MEM_DEPTH; i_init = i_init + 1) begin
            u_act_mem.mem[i_init]      = 32'h0101_0101;  // 8-bit acts = 1
            u_weight_mem.mem[i_init]   = 32'h0A0A_0A0A;
            u_syn_curr_mem.mem[i_init] = 32'd0;
            u_bias_curr_mem.mem[i_init]= 32'd0;
            u_thresh_mem.mem[i_init]   = 32'd0;
            u_pot_mem.mem[i_init]      = 32'd0;
            u_spike_mem.mem[i_init]    = 32'd0;
        end
    end

    // ----------------------------------------------------------------
    // Helpers
    // ----------------------------------------------------------------
    integer errors;
    integer timeout;

    task cfg_write;
        input [31:0] addr;
        input [31:0] data;
        begin
            @(negedge clk);
            sys_req_i  = 1'b1;
            sys_addr_i = addr;
            sys_data_i = data;
            #1;
            if (!sys_ack_o) begin
                $display("FAIL cfg_write: no ACK for addr=0x%08h", addr);
                errors = errors + 1;
            end
            @(negedge clk);
            sys_req_i = 1'b0;
        end
    endtask

    task check_eq;
        input [31:0] got;
        input [31:0] exp;
        input [255:0] label;
        begin
            if (got !== exp) begin
                $display("FAIL %s: got 0x%08h  exp 0x%08h", label, got, exp);
                errors = errors + 1;
            end else begin
                $display("  OK  %s = 0x%08h", label, got);
            end
        end
    endtask

    task wait_pipeline;
        output reg timed_out;
        begin
            timed_out = 0;

            timeout = 500;
            @(posedge clk);
            while (!spike_proc_finished_o && timeout > 0) begin
                timeout = timeout - 1;
                @(posedge clk);
            end
            #1;
            if (timeout == 0) begin
                $display("FAIL: spike_proc_finished_o timeout");
                errors    = errors + 1;
                timed_out = 1;
            end else
                $display("  spike_proc_finished after %0d cycles", 500 - timeout);

            timeout = 500;
            @(posedge clk);
            while (!acc_finished_o && timeout > 0) begin
                timeout = timeout - 1;
                @(posedge clk);
            end
            #1;
            if (timeout == 0) begin
                $display("FAIL: acc_finished_o timeout");
                errors    = errors + 1;
                timed_out = 1;
            end else
                $display("  acc_finished after %0d more cycles", 500 - timeout);

            @(posedge clk); #1;
            if (acc_busy_o) begin
                $display("FAIL: acc_busy_o still high after acc_finished");
                errors = errors + 1;
            end
        end
    endtask

    // ----------------------------------------------------------------
    // Stimulus
    // ----------------------------------------------------------------
    reg timed_out;

    initial begin
        errors            = 0;
        reset             = 1'b1;
        sys_req_i         = 1'b0;
        start_new_block_i = 1'b0;
        target_acc_i      = {`TGT_ACC_SZ{1'b0}};
        buffer_info_i     = {`SCH_ENTRY_SZ{1'b0}};

        repeat (5) @(posedge clk);
        @(negedge clk); reset = 1'b0;
        repeat (2) @(posedge clk);

        // ============================================================
        // Write configuration registers
        // ============================================================
        // spike_processing
        cfg_write(32'hFFFF_0000, 32'd0);    // act_base_addr      = 0
        cfg_write(32'hFFFF_0004, 32'd10);   // weight_base_addr   = 10
        cfg_write(32'hFFFF_0008, 32'd20);   // syn_curr_base_addr = 20
        cfg_write(32'hFFFF_000C, 32'd3);    // weight_sz          = 3 (8-bit)
        cfg_write(32'hFFFF_0014, 32'd1);    // total_timesteps    = 1
        cfg_write(32'hFFFF_0040, 32'd0);    // bin_point_syn_curr = 0
        cfg_write(32'hFFFF_0044, 32'd2);    // in_x_len           = 2
        cfg_write(32'hFFFF_0048, 32'd1);    // in_y_len           = 1
        cfg_write(32'hFFFF_004C, 32'd2);    // out_x_len          = 2
        cfg_write(32'hFFFF_0050, 32'd1);    // out_y_len          = 1
        cfg_write(32'hFFFF_0054, 32'd4);    // weights_per_word   = 4
        cfg_write(32'hFFFF_0058, 32'd1);    // rows_per_neuron    = 1
        cfg_write(32'hFFFF_005C, 32'd5);    // weight_idx_sz      = 5
        cfg_write(32'hFFFF_0070, 32'd0);    // weight_mode        = 0 (full)
        cfg_write(32'hFFFF_0074, 32'd1);    // x_kernel_len       = 1
        cfg_write(32'hFFFF_0078, 32'd1);    // y_kernel_len       = 1
        cfg_write(32'hFFFF_007C, 32'd1);    // x_kernel_step      = 1
        cfg_write(32'hFFFF_0080, 32'd1);    // y_kernel_step      = 1
        cfg_write(32'hFFFF_0084, 32'd0);    // x_kernel_offset    = 0
        cfg_write(32'hFFFF_0088, 32'd0);    // y_kernel_offset    = 0
        cfg_write(32'hFFFF_0098, 32'd3);    // sp_act_sz          = 3 (8-bit runtime)

        // neuron_processing
        cfg_write(32'hFFFF_0020, 32'd1);            // last_neuron_idx   = 1 (2 neurons)
        cfg_write(32'hFFFF_002C, 32'd40);           // lut_base_addr     = 40
        cfg_write(32'hFFFF_0030, 32'd50);           // pot_base          = 50
        cfg_write(32'hFFFF_0064, 32'd60);           // spike_base        = 60
        cfg_write(32'hFFFF_0034, 32'd5);            // syn_curr_sz       = 5 (32-bit)
        cfg_write(32'hFFFF_0038, 32'd3);            // lut_out_sz        = 3 (8-bit LUT entries)
        cfg_write(32'hFFFF_003C, 32'd5);            // act_out_sz        = 5 (32-bit output)
        cfg_write(32'hFFFF_006C, 32'h8000_0000);    // pot_decay_mult    = 0.5 (Q0.32)
        cfg_write(32'hFFFF_00A0, 32'd0);            // thresh_op         = 0 (RELU)

        $display("=== tb_acc_snn_processor (annAcc) ===");

        // ============================================================
        // Test 1: RELU mode
        // SP: syn_curr = 2 × (1 × 10) = 20 per output neuron
        // NP: RELU(20) = 20, decayed = floor(20 × 0.5) = 10
        //   spike_sram[60/61] = 32'd20
        //   pot_sram[50/51]   = 32'd10
        // ============================================================
        $display("Test 1: RELU mode, syn_curr=20 -> act_out=20, decayed=10");

        @(negedge clk); start_new_block_i = 1'b1;
        @(negedge clk); start_new_block_i = 1'b0;

        wait_pipeline(timed_out);
        if (!timed_out) begin
            check_eq(u_spike_mem.mem[60], 32'd20, "T1 act_out[0] RELU(20)=20");
            check_eq(u_spike_mem.mem[61], 32'd20, "T1 act_out[1] RELU(20)=20");
            check_eq(u_pot_mem.mem[50],   32'd10, "T1 pot[0] decayed=10");
            check_eq(u_pot_mem.mem[51],   32'd10, "T1 pot[1] decayed=10");
        end

        // ============================================================
        // Test 2: LUT mode
        // Reset memories; set thresh_op=1; lut[20]=0xAA=170
        // SP: syn_curr=20 per neuron
        // NP: lut_index=20 -> lut_sram[40+5=45] bits[31:24]=0xAA=170
        //   act_out = 32'h0000_00AA = 170
        //   decayed = floor(170 × 0.5) = 85
        //   spike_sram[60/61] = 32'h0000_00AA
        //   pot_sram[50/51]   = 32'd85
        // ============================================================
        $display("Test 2: LUT mode, lut[20]=0xAA=170, decayed=85");

        for (i_init = 0; i_init < MEM_DEPTH; i_init = i_init + 1) begin
            u_syn_curr_mem.mem[i_init] = 32'd0;
            u_pot_mem.mem[i_init]      = 32'd0;
            u_spike_mem.mem[i_init]    = 32'd0;
        end
        // lut_base=40; lut[20]: word = 40 + 20/4 = 45, element = 20%4 = 0 -> bits[7:0] (little-endian)
        u_thresh_mem.mem[45] = 32'h000000AA;
        cfg_write(32'hFFFF_00A0, 32'd1);    // thresh_op = 1 (LUT)

        @(negedge clk); start_new_block_i = 1'b1;
        @(negedge clk); start_new_block_i = 1'b0;

        wait_pipeline(timed_out);
        if (!timed_out) begin
            check_eq(u_spike_mem.mem[60], 32'h0000_00AA, "T2 act_out[0] lut[20]=0xAA");
            check_eq(u_spike_mem.mem[61], 32'h0000_00AA, "T2 act_out[1] lut[20]=0xAA");
            check_eq(u_pot_mem.mem[50],   32'd85,        "T2 pot[0] decayed=85");
            check_eq(u_pot_mem.mem[51],   32'd85,        "T2 pot[1] decayed=85");
        end

        // ============================================================
        // Test 3: ABS mode
        // weight_sram -> 0xF6F6_F6F6 (8-bit signed -10 per slot); act=1
        // SP: syn_curr = 2 × (1 × -10) = -20 per output neuron
        // NP: ABS(-20) = 20, decayed = floor(20 × 0.5) = 10
        //   spike_sram[60/61] = 32'd20
        //   pot_sram[50/51]   = 32'd10
        // ============================================================
        $display("Test 3: ABS mode, syn_curr=-20 -> ABS=20, decayed=10");

        for (i_init = 0; i_init < MEM_DEPTH; i_init = i_init + 1) begin
            u_weight_mem.mem[i_init]   = 32'hF6F6_F6F6;  // 8-bit signed -10 per slot
            u_syn_curr_mem.mem[i_init] = 32'd0;
            u_pot_mem.mem[i_init]      = 32'd0;
            u_spike_mem.mem[i_init]    = 32'd0;
        end
        cfg_write(32'hFFFF_00A0, 32'd2);    // thresh_op = 2 (ABS)

        @(negedge clk); start_new_block_i = 1'b1;
        @(negedge clk); start_new_block_i = 1'b0;

        wait_pipeline(timed_out);
        if (!timed_out) begin
            check_eq(u_spike_mem.mem[60], 32'd20, "T3 act_out[0] ABS(-20)=20");
            check_eq(u_spike_mem.mem[61], 32'd20, "T3 act_out[1] ABS(-20)=20");
            check_eq(u_pot_mem.mem[50],   32'd10, "T3 pot[0] decayed=10");
            check_eq(u_pot_mem.mem[51],   32'd10, "T3 pot[1] decayed=10");
        end

        // ============================================================
        // Test 4: non-uniform column-major weights, RELU
        //   Distinct weights per output neuron — uniform-weight tests
        //   T1-T3 pass even with the pre-2026-05-29 out_elem_count_r
        //   bug; this one fails without the fix.
        //   W[0][0]=W[0][1]=+10, W[1][0]=W[1][1]=+5, x=1 each:
        //     syn_curr[0] = 10 + 10 = 20
        //     syn_curr[1] =  5 +  5 = 10
        //   RELU pass-through: act_out[0] = 20, act_out[1] = 10
        //   Decay × 0.5:        pot[0] = 10,    pot[1] = 5
        // Expected:
        //   spike_sram[60] = 32'd20, spike_sram[61] = 32'd10
        //   pot_sram[50]   = 32'd10, pot_sram[51]   = 32'd5
        // ============================================================
        $display("Test 4: non-uniform column-major weights, RELU");

        for (i_init = 0; i_init < MEM_DEPTH; i_init = i_init + 1) begin
            u_weight_mem.mem[i_init]   = 32'h0A0A_0A0A;  // restore default
            u_syn_curr_mem.mem[i_init] = 32'd0;
            u_pot_mem.mem[i_init]      = 32'd0;
            u_spike_mem.mem[i_init]    = 32'd0;
        end

        // Column-major weights at a NEW base (14) to force a weight-cache miss
        //   weight_sram[14] = column 0 = {W[0][0]=10, W[1][0]=5, slice2=0, slice3=0}
        //   weight_sram[15] = column 1 = {W[0][1]=10, W[1][1]=5, slice2=0, slice3=0}
        u_weight_mem.mem[14] = 32'h0000_050A;
        u_weight_mem.mem[15] = 32'h0000_050A;
        cfg_write(32'hFFFF_0004, 32'd14);   // weight_base_addr = 14

        cfg_write(32'hFFFF_00A0, 32'd0);    // thresh_op = 0 (RELU)

        @(negedge clk); start_new_block_i = 1'b1;
        @(negedge clk); start_new_block_i = 1'b0;

        wait_pipeline(timed_out);
        if (!timed_out) begin
            check_eq(u_spike_mem.mem[60], 32'd20, "T4 act_out[0] RELU(20)=20");
            check_eq(u_spike_mem.mem[61], 32'd10, "T4 act_out[1] RELU(10)=10");
            check_eq(u_pot_mem.mem[50],   32'd10, "T4 pot[0] decayed=10");
            check_eq(u_pot_mem.mem[51],   32'd5,  "T4 pot[1] decayed=5");
        end

        $display("=== tb_acc_snn_processor: %0d failure(s) ===", errors);
        if (errors == 0) $display("PASS"); else $display("FAIL");
        $finish;
    end

    // ----------------------------------------------------------------
    // Safety watchdog
    // ----------------------------------------------------------------
    initial begin
        #200000;
        $display("FAIL: global simulation timeout");
        $finish;
    end

endmodule


// ====================================================================
//  sram_model
// ====================================================================
module sram_model #(
    parameter DATA_W = 32,
    parameter DEPTH  = 256
)(
    input  wire              clk,
    input  wire              we,
    input  wire              re,
    input  wire        [7:0] addr,
    input  wire [DATA_W-1:0] wdata,
    output reg  [DATA_W-1:0] rdata
);
    reg [DATA_W-1:0] mem [0:DEPTH-1];
    integer i;
    initial begin
        for (i = 0; i < DEPTH; i = i + 1)
            mem[i] = {DATA_W{1'b0}};
        rdata = {DATA_W{1'b0}};
    end
    always @(posedge clk) begin
        if (we) mem[addr] <= wdata;
        if (re) rdata     <= mem[addr];
    end
endmodule
