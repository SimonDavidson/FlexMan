// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
`include "../../shared/constants.v"
`timescale 1ns/1ps

// ============================================================================
// dummy_acc
//   Simulates one hardware accelerator.
//   Accepts a task when start_i is high and target_acc_i[0] == ACC_ID.
//   Completes after a pseudo-random 20-30 cycle delay.
//   result_o is high ~70% of the time.
// ============================================================================
module dummy_acc #(parameter ACC_ID = 0) (
    input  wire       clk,
    input  wire       reset,
    input  wire       start_i,        // start_new_block_o from scheduler
    input  wire [1:0] target_acc_i,   // target_acc_o from scheduler
    output reg        busy_o,
    output reg        finished_o,
    output reg        result_o
);

reg [5:0] countdown;

always @(posedge clk) begin
    if (reset) begin
        busy_o     <= 1'b0;
        finished_o <= 1'b0;
        result_o   <= 1'b0;
        countdown  <= 6'd0;
    end else begin
        finished_o <= 1'b0;
        if (start_i && (target_acc_i[0] == ACC_ID[0]) && !busy_o) begin
            busy_o    <= 1'b1;
            countdown <= 20 + ($urandom % 11);   // 20 to 30 cycles
        end else if (busy_o) begin
            if (countdown == 6'd1) begin
                busy_o     <= 1'b0;
                finished_o <= 1'b1;
                result_o   <= ($urandom % 10) < 7; // high ~70% of the time
                countdown  <= 6'd0;
            end else begin
                countdown <= countdown - 6'd1;
            end
        end
    end
end

endmodule


// ============================================================================
// top  –  testbench for scheduler
//
// Program: 30 TASK instructions + STOP.
//
// Dependency graph: three-buffer ring recycled over 10 layers.
// Pre-filled seed buffers: 0, 1, 2 (usage = 2 each).
//
//  Layer 1  (T00-T02): {0,1} → 3,  {1,2} → 4,  {0,2} → 5    (#tgts=2)
//  Layer 2  (T03-T05): {3,4} → 6,  {4,5} → 7,  {3,5} → 8    (#tgts=2)
//  Layer 3  (T06-T08): {6,7} → 0,  {7,8} → 1,  {6,8} → 2    (#tgts=2)
//  Layer 4  (T09-T11): {0,1} → 3,  {1,2} → 4,  {0,2} → 5    (#tgts=2)
//  Layer 5  (T12-T14): {3,4} → 6,  {4,5} → 7,  {3,5} → 8    (#tgts=2)
//  Layer 6  (T15-T17): {6,7} → 0,  {7,8} → 1,  {6,8} → 2    (#tgts=2)
//  Layer 7  (T18-T20): {0,1} → 3,  {1,2} → 4,  {0,2} → 5    (#tgts=2)
//  Layer 8  (T21-T23): {3,4} → 6,  {4,5} → 7,  {3,5} → 8    (#tgts=2)
//  Layer 9  (T24-T26): {6,7} → 9,  {7,8} → 10, {6,8} → 11   (#tgts=2)
//  Layer 10 (T27-T29): {9,10} → 12, {10,11} → 13, {9,11} → 14 (#tgts=1)
//
// Acc assignment alternates 0/1/0 within each layer triplet.
// All tasks use colour = 0.
// ============================================================================
module top;

// ---- Parameters (must match DUT) ------------------------------------------
localparam SCH_ENTRY_SZ        = 32;
localparam TGT_ACC_SZ          = 2;
localparam TGT_COUNT_SZ        = 3;
localparam CFG_ID_SZ           = 5;
localparam NUM_BUFFERS         = 16;
localparam COL_BUFF_ID_SZ      = 16;
localparam NUM_SCH_ENTRIES     = 4;
localparam NUM_HW_ACCELERATORS = 2;
localparam PROG_ADDR_BITS      = 10;
localparam PROG_DATA_BITS      = 32;
localparam BUFF_INDX_SZ        = $clog2(NUM_BUFFERS);  // 4

// ---- Entry field bit positions (mirrors scheduler.v localparams) ----------
// With BUFF_INDX_SZ=4, HW_ACC_SZ=1, CFG_ID_SZ=5, NUM_TGTS_SZ=3:
localparam E_SRC1_LO  =  0;   localparam E_SRC1_HI  =  3;
localparam E_SRC2_LO  =  4;   localparam E_SRC2_HI  =  7;
localparam E_SRC3_LO  =  8;   localparam E_SRC3_HI  = 11;
localparam E_TGT_LO   = 12;   localparam E_TGT_HI   = 15;
localparam E_NTGTS_LO = 16;   localparam E_NTGTS_HI = 18;
localparam E_COLOUR   = 19;
localparam E_ACC_LO   = 20;   localparam E_ACC_HI   = 20;
localparam E_CFG_LO   = 21;   localparam E_CFG_HI   = 25;

// ---- Clock and reset -------------------------------------------------------
reg clk, reset;
initial clk = 1'b0;
always #5 clk = ~clk;

// ---- Scheduler interface wires --------------------------------------------
wire [`ADDR_SIZE-1:0]          prog_mem_addr;
wire                           prog_mem_req;
reg                            prog_mem_wait;
wire [PROG_DATA_BITS-1:0]      prog_mem_data;

wire [NUM_HW_ACCELERATORS-1:0] acc_busy_w;
wire [NUM_HW_ACCELERATORS-1:0] acc_finished_w;
wire [NUM_HW_ACCELERATORS-1:0] acc_result_w;
wire                           start_new_block;
wire [TGT_ACC_SZ-1:0]          target_acc;
wire [SCH_ENTRY_SZ-1:0]        buffer_info;

reg                            start_program;
reg  [PROG_ADDR_BITS-1:0]      program_addr;
reg                            mark_buff_as_full;
reg  [BUFF_INDX_SZ-1:0]        full_buff_id;
reg  [2:0]                     full_buff_usage;

// ---- Program memory (64 x 32-bit, combinatorial read) ---------------------
reg [31:0] prog_mem [0:63];
assign prog_mem_data = prog_mem[prog_mem_addr[5:0]];

// ---- Instruction encoding -------------------------------------------------
// TASK: {00, src2[4:0], src1[4:0], 00, acc, cfg[4:0], ntgts[2:0], colour=0, tgt[4:0], 3'b000}
// Bit positions: [31:30]=0, [29:25]=src2, [24:20]=src1, [19:18]=0,
//               [17]=acc, [16:12]=cfg, [11:9]=ntgts, [8]=colour, [7:3]=tgt, [2:0]=000
function [31:0] task_inst;
    input [4:0] src2;
    input [4:0] src1;
    input       acc;
    input [4:0] cfg;
    input [2:0] ntgts;
    input [4:0] tgt;
    task_inst = {2'b00, src2, src1, 2'b00, acc, cfg, ntgts, 1'b0, tgt, 3'b000};
endfunction

localparam STOP_INST = 32'h00000002; // opcode 3'b010

// ---- Dummy accelerators ---------------------------------------------------
wire busy_0,     busy_1;
wire finished_0, finished_1;
wire result_0,   result_1;

assign acc_busy_w     = {busy_1,     busy_0};
assign acc_finished_w = {finished_1, finished_0};
assign acc_result_w   = {result_1,   result_0};

dummy_acc #(.ACC_ID(0)) acc0 (
    .clk(clk), .reset(reset),
    .start_i(start_new_block), .target_acc_i(target_acc),
    .busy_o(busy_0), .finished_o(finished_0), .result_o(result_0)
);

dummy_acc #(.ACC_ID(1)) acc1 (
    .clk(clk), .reset(reset),
    .start_i(start_new_block), .target_acc_i(target_acc),
    .busy_o(busy_1), .finished_o(finished_1), .result_o(result_1)
);

// ---- DUT ------------------------------------------------------------------
scheduler #(
    .SCH_ENTRY_SZ(SCH_ENTRY_SZ),
    .TGT_ACC_SZ(TGT_ACC_SZ),
    .TGT_COUNT_SZ(TGT_COUNT_SZ),
    .CFG_ID_SZ(CFG_ID_SZ),
    .NUM_BUFFERS(NUM_BUFFERS),
    .COL_BUFF_ID_SZ(COL_BUFF_ID_SZ),
    .NUM_SCH_ENTRIES(NUM_SCH_ENTRIES),
    .NUM_HW_ACCELERATORS(NUM_HW_ACCELERATORS),
    .PROG_ADDR_BITS(PROG_ADDR_BITS),
    .PROG_DATA_BITS(PROG_DATA_BITS),
    .BUFF_INDX_SZ(BUFF_INDX_SZ)
) dut (
    .clk(clk),
    .reset(reset),
    .test_stall_pipe(1'b0),
    .sys_req_i(1'b0),
    .sys_ack_o(),
    .sys_data_i(32'b0),
    .sys_data_o(),
    .start_program_i(start_program),
    .program_addr_i(program_addr),
    .prog_mem_addr_o(prog_mem_addr),
    .prog_mem_data_i(prog_mem_data),
    .prog_mem_req_o(prog_mem_req),
    .prog_mem_wait_i(prog_mem_wait),
    .acc_busy_i(acc_busy_w),
    .acc_finished_i(acc_finished_w),
    .acc_result_i(acc_result_w),
    .acc_ready_next_i({NUM_HW_ACCELERATORS{1'b0}}),
    .start_new_block_o(start_new_block),
    .target_acc_o(target_acc),
    .buffer_info_o(buffer_info),
    .mark_buff_as_full_i(mark_buff_as_full),
    .full_buff_id_i(full_buff_id),
    .full_buff_usage_i(full_buff_usage)
);

// ---- Event monitors -------------------------------------------------------
integer dispatch_count, finish_count;

always @(posedge clk) begin
    if (reset) begin
        dispatch_count <= 0;
        finish_count   <= 0;
    end else begin
        if (start_new_block) begin
            dispatch_count <= dispatch_count + 1;
            $display("[%6t ns] DISPATCH #%0d  acc=%0d  cfg=%02d  tgt=%02d  src1=%02d  src2=%02d",
                $time,
                dispatch_count + 1,
                buffer_info[E_ACC_HI:E_ACC_LO],
                buffer_info[E_CFG_HI:E_CFG_LO],
                buffer_info[E_TGT_HI:E_TGT_LO],
                buffer_info[E_SRC1_HI:E_SRC1_LO],
                buffer_info[E_SRC2_HI:E_SRC2_LO]);
        end
        if (acc_finished_w[0])
            $display("[%6t ns] ACC0 DONE  result=%0d", $time, acc_result_w[0]);
        if (acc_finished_w[1])
            $display("[%6t ns] ACC1 DONE  result=%0d", $time, acc_result_w[1]);
        finish_count <= finish_count + acc_finished_w[0] + acc_finished_w[1];
        if (finish_count + acc_finished_w[0] + acc_finished_w[1] >= 30) begin
            $display("[%6t ns] All 30 tasks completed successfully.", $time);
            #20 $finish;
        end
    end
end

// ---- Timeout --------------------------------------------------------------
initial begin
    #50000;
    $display("[%6t ns] TIMEOUT after 50000 ns (%0d dispatched, %0d finished).",
             $time, dispatch_count, finish_count);
    $finish;
end

// ---- Waveform dump --------------------------------------------------------
initial begin
    $dumpfile("tb_scheduler.vcd");
    $dumpvars(0, top);
end

// ---- Main stimulus --------------------------------------------------------
integer k;
initial begin
    reset         = 1'b1;
    prog_mem_wait = 1'b0;
    start_program = 1'b0;
    program_addr  = 'b0;
    mark_buff_as_full = 1'b0;
    full_buff_id      = 4'd0;
    full_buff_usage   = 3'd0;

    // ---- Load program into memory -----------------------------------------
    //
    // TASK(src2, src1, acc, cfg, ntgts, tgt)
    //
    // Layer 1: seed {0,1,2} → intermediates {3,4,5}
    prog_mem[ 0] = task_inst(5'd1,  5'd0,  1'b0, 5'd0,  3'd2, 5'd3); // T00
    prog_mem[ 1] = task_inst(5'd2,  5'd1,  1'b1, 5'd1,  3'd2, 5'd4); // T01
    prog_mem[ 2] = task_inst(5'd2,  5'd0,  1'b0, 5'd2,  3'd2, 5'd5); // T02
    // Layer 2
    prog_mem[ 3] = task_inst(5'd4,  5'd3,  1'b1, 5'd3,  3'd2, 5'd6); // T03
    prog_mem[ 4] = task_inst(5'd5,  5'd4,  1'b0, 5'd4,  3'd2, 5'd7); // T04
    prog_mem[ 5] = task_inst(5'd5,  5'd3,  1'b1, 5'd5,  3'd2, 5'd8); // T05
    // Layer 3 (writes back into {0,1,2} freed after layer 1 completes)
    prog_mem[ 6] = task_inst(5'd7,  5'd6,  1'b0, 5'd6,  3'd2, 5'd0); // T06
    prog_mem[ 7] = task_inst(5'd8,  5'd7,  1'b1, 5'd7,  3'd2, 5'd1); // T07
    prog_mem[ 8] = task_inst(5'd8,  5'd6,  1'b0, 5'd8,  3'd2, 5'd2); // T08
    // Layer 4
    prog_mem[ 9] = task_inst(5'd1,  5'd0,  1'b1, 5'd9,  3'd2, 5'd3); // T09
    prog_mem[10] = task_inst(5'd2,  5'd1,  1'b0, 5'd10, 3'd2, 5'd4); // T10
    prog_mem[11] = task_inst(5'd2,  5'd0,  1'b1, 5'd11, 3'd2, 5'd5); // T11
    // Layer 5
    prog_mem[12] = task_inst(5'd4,  5'd3,  1'b0, 5'd12, 3'd2, 5'd6); // T12
    prog_mem[13] = task_inst(5'd5,  5'd4,  1'b1, 5'd13, 3'd2, 5'd7); // T13
    prog_mem[14] = task_inst(5'd5,  5'd3,  1'b0, 5'd14, 3'd2, 5'd8); // T14
    // Layer 6
    prog_mem[15] = task_inst(5'd7,  5'd6,  1'b1, 5'd15, 3'd2, 5'd0); // T15
    prog_mem[16] = task_inst(5'd8,  5'd7,  1'b0, 5'd16, 3'd2, 5'd1); // T16
    prog_mem[17] = task_inst(5'd8,  5'd6,  1'b1, 5'd17, 3'd2, 5'd2); // T17
    // Layer 7
    prog_mem[18] = task_inst(5'd1,  5'd0,  1'b0, 5'd18, 3'd2, 5'd3); // T18
    prog_mem[19] = task_inst(5'd2,  5'd1,  1'b1, 5'd19, 3'd2, 5'd4); // T19
    prog_mem[20] = task_inst(5'd2,  5'd0,  1'b0, 5'd20, 3'd2, 5'd5); // T20
    // Layer 8
    prog_mem[21] = task_inst(5'd4,  5'd3,  1'b1, 5'd21, 3'd2, 5'd6); // T21
    prog_mem[22] = task_inst(5'd5,  5'd4,  1'b0, 5'd22, 3'd2, 5'd7); // T22
    prog_mem[23] = task_inst(5'd5,  5'd3,  1'b1, 5'd23, 3'd2, 5'd8); // T23
    // Layer 9 (drains into fresh buffers 9, 10, 11)
    prog_mem[24] = task_inst(5'd7,  5'd6,  1'b0, 5'd24, 3'd2, 5'd9);  // T24
    prog_mem[25] = task_inst(5'd8,  5'd7,  1'b1, 5'd25, 3'd2, 5'd10); // T25
    prog_mem[26] = task_inst(5'd8,  5'd6,  1'b0, 5'd26, 3'd2, 5'd11); // T26
    // Layer 10 (final outputs into buffers 12, 13, 14; each produced once)
    prog_mem[27] = task_inst(5'd10, 5'd9,  1'b1, 5'd27, 3'd1, 5'd12); // T27
    prog_mem[28] = task_inst(5'd11, 5'd10, 1'b0, 5'd28, 3'd1, 5'd13); // T28
    prog_mem[29] = task_inst(5'd11, 5'd9,  1'b1, 5'd29, 3'd1, 5'd14); // T29
    // STOP
    prog_mem[30] = STOP_INST;
    for (k = 31; k < 64; k = k + 1)
        prog_mem[k] = 32'h0;

    // ---- Release reset ----------------------------------------------------
    repeat(4) @(posedge clk);
    @(negedge clk);
    reset = 1'b0;

    // ---- Pre-fill seed buffers 0, 1, 2 (2 consumers each) ----------------
    // Each presented for one full clock cycle in turn.
    @(posedge clk);
    mark_buff_as_full = 1'b1; full_buff_id = 4'd0; full_buff_usage = 3'd2;
    @(posedge clk);
    full_buff_id = 4'd1;
    @(posedge clk);
    full_buff_id = 4'd2;
    @(posedge clk);
    mark_buff_as_full = 1'b0;

    // ---- Start program ----------------------------------------------------
    @(posedge clk);
    start_program = 1'b1;
    program_addr  = {PROG_ADDR_BITS{1'b0}};
    @(posedge clk);
    start_program = 1'b0;
end

endmodule
