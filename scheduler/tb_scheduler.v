// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
`include "../shared/constants.v"
`timescale 1ns/1ps

// ============================================================================
// dummy_acc  –  simple accelerator stub
//
// Accepts a task when start_i is asserted and target_acc_i[0] matches ACC_ID.
// Completes after a pseudo-random 20–30 cycle delay.  result_o is HIGH (~70 %)
// indicating success.
// ============================================================================
module dummy_acc #(parameter ACC_ID = 0) (
    input  wire       clk,
    input  wire       reset,
    input  wire       start_i,
    input  wire [0:0] target_acc_i,
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
            countdown <= 20 + ($urandom % 11);   // 20–30 cycles
        end else if (busy_o) begin
            if (countdown == 6'd1) begin
                busy_o     <= 1'b0;
                finished_o <= 1'b1;
                result_o   <= ($urandom % 10) < 7;
                countdown  <= 6'd0;
            end else begin
                countdown <= countdown - 6'd1;
            end
        end
    end
end

endmodule


// ============================================================================
// top  –  scheduler integration testbench
//
// Program: 30 TASK instructions + 3 NXT instructions + 1 post-NXT TASK + STOP
//
// Dependency graph – three-buffer ring recycled across 10 layers:
//
//  Layer  1  (T00–T02): {0,1}→3,  {1,2}→4,  {0,2}→5    #tgts=2
//  Layer  2  (T03–T05): {3,4}→6,  {4,5}→7,  {3,5}→8    #tgts=2
//  Layer  3  (T06–T08): {6,7}→0,  {7,8}→1,  {6,8}→2    #tgts=2
//  Layer  4  (T09–T11): {0,1}→3,  {1,2}→4,  {0,2}→5    #tgts=2
//  Layer  5  (T12–T14): {3,4}→6,  {4,5}→7,  {3,5}→8    #tgts=2
//  Layer  6  (T15–T17): {6,7}→0,  {7,8}→1,  {6,8}→2    #tgts=2
//  Layer  7  (T18–T20): {0,1}→3,  {1,2}→4,  {0,2}→5    #tgts=2
//  Layer  8  (T21–T23): {3,4}→6,  {4,5}→7,  {3,5}→8    #tgts=2
//  Layer  9  (T24–T26): {6,7}→9,  {7,8}→10, {6,8}→11   #tgts=2
//  Layer 10  (T27–T29): {9,10}→12,{10,11}→13,{9,11}→14  #tgts=1
//
// addr 30: NXT(in+out)  – barrier fires once T27–T29 dispatched; pulses both outputs
// addr 31: NXT(in only) – immediate (table already empty); pulses input output only
// addr 32: NXT(out only)– immediate; pulses output only
// addr 33: T30 {12,13}→15 – post-NXT task; stalls until T27/T28 complete, then runs
// addr 34: STOP
//
// NXT checks:
//   - nxt_input_pulse_o  must not fire before all 30 pre-NXT tasks dispatched
//   - nxt_output_pulse_o must not fire before all 30 pre-NXT tasks dispatched
//   - nxt_in_count  == 2 (NXT_BOTH + NXT_IN)
//   - nxt_out_count == 2 (NXT_BOTH + NXT_OUT)
//   - T30 dispatches and completes after the NXT block (finish_count reaches 31)
//
// Seed buffers 0, 1, 2 are pre-filled (2 consumers each) before START.
// Acc assignment alternates 0 / 1 across tasks.  Colour is always 0.
// ============================================================================
module top;

// ---- Parameters matching DUT -----------------------------------------------
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
localparam BUFF_INDX_SZ        = 4;   // $clog2(16)

// ---- Entry field positions in entry_data / buffer_info_o -------------------
localparam E_SRC1_LO  =  0; localparam E_SRC1_HI  =  3;
localparam E_SRC2_LO  =  4; localparam E_SRC2_HI  =  7;
localparam E_SRC3_LO  =  8; localparam E_SRC3_HI  = 11;
localparam E_TGT_LO   = 12; localparam E_TGT_HI   = 15;
localparam E_NTGTS_LO = 16; localparam E_NTGTS_HI = 18;
localparam E_COLOUR   = 19;
localparam E_ACC_LO   = 20; localparam E_ACC_HI   = 20;
localparam E_CFG_LO   = 21; localparam E_CFG_HI   = 25;

// ---- Instruction encoding --------------------------------------------------
//  TASK: {2'b00, src2[4:0], src1[4:0], 2'b00, acc[0], cfg[4:0], ntgts[2:0], colour=0, tgt[4:0], 3'b000}
function [31:0] task_inst;
    input [4:0] src2;
    input [4:0] src1;
    input       acc;
    input [4:0] cfg;
    input [2:0] ntgts;
    input [4:0] tgt;
    task_inst = {2'b00, src2, src1, 2'b00, acc, cfg, ntgts, 1'b0, tgt, 3'b000};
endfunction

localparam STOP_INST = 32'h00000002;   // opcode = 3'b010

function [31:0] nxt_inst;
    input nxt_in;
    input nxt_out;
    nxt_inst = {26'b0, nxt_out, nxt_in, 1'b0, 3'b100};  // [5]=nxt_out, [4]=nxt_in, [2:0]=3'b100
endfunction

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
wire [$clog2(NUM_HW_ACCELERATORS)-1:0] target_acc;
wire [SCH_ENTRY_SZ-1:0]        buffer_info;
wire                           nxt_in_pulse;
wire                           nxt_out_pulse;

reg                            start_program;
reg  [PROG_ADDR_BITS-1:0]      program_addr;
reg                            mark_buff_as_full;
reg  [BUFF_INDX_SZ-1:0]        full_buff_id;
reg  [2:0]                     full_buff_usage;

// ---- Program memory (64-word ROM, combinatorial read) ---------------------
reg [31:0] prog_mem [0:63];
assign prog_mem_data = prog_mem[prog_mem_addr[5:0]];

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
    .start_new_block_o(start_new_block),
    .target_acc_o(target_acc),
    .buffer_info_o(buffer_info),
    .nxt_input_pulse_o(nxt_in_pulse),
    .nxt_output_pulse_o(nxt_out_pulse),
    .mark_buff_as_full_i(mark_buff_as_full),
    .full_buff_id_i(full_buff_id),
    .full_buff_usage_i(full_buff_usage)
);

// ---- Event monitors -------------------------------------------------------
integer dispatch_count;
integer finish_count;
integer nxt_in_count;
integer nxt_out_count;

initial begin
    dispatch_count = 0;
    finish_count   = 0;
    nxt_in_count   = 0;
    nxt_out_count  = 0;
end

always @(posedge clk) begin
    if (!reset) begin
        if (start_new_block) begin
            dispatch_count = dispatch_count + 1;
            $display("[%0t ns] DISPATCH #%0d  acc=%0b  cfg=%02d  tgt=%02d  src1=%02d  src2=%02d",
                $time,
                dispatch_count,
                buffer_info[E_ACC_HI:E_ACC_LO],
                buffer_info[E_CFG_HI:E_CFG_LO],
                buffer_info[E_TGT_HI:E_TGT_LO],
                buffer_info[E_SRC1_HI:E_SRC1_LO],
                buffer_info[E_SRC2_HI:E_SRC2_LO]);
        end
        if (acc_finished_w[0])
            $display("[%0t ns] ACC0 done", $time);
        if (acc_finished_w[1])
            $display("[%0t ns] ACC1 done", $time);

        finish_count = finish_count + acc_finished_w[0] + acc_finished_w[1];

        // NXT pulse monitors
        if (nxt_in_pulse || nxt_out_pulse) begin
            if (dispatch_count < 30) begin
                $display("[%0t ns] FAIL – NXT fired before all 30 tasks dispatched (dispatched=%0d).",
                         $time, dispatch_count);
                $finish;
            end
            if (nxt_in_pulse) begin
                nxt_in_count = nxt_in_count + 1;
                $display("[%0t ns] NXT in_pulse  #%0d", $time, nxt_in_count);
            end
            if (nxt_out_pulse) begin
                nxt_out_count = nxt_out_count + 1;
                $display("[%0t ns] NXT out_pulse #%0d", $time, nxt_out_count);
            end
        end

        if (finish_count >= 31) begin
            // All 31 tasks (30 pre-NXT + T30 post-NXT) done.
            // NXT(in+out) fires in×1 out×1; NXT(in) fires in×1; NXT(out) fires out×1 → total in×2 out×2.
            if (nxt_in_count == 2 && nxt_out_count == 2) begin
                $display("[%0t ns] PASS – 31 tasks completed; NXT in×%0d out×%0d; T30 ran after NXT.",
                         $time, nxt_in_count, nxt_out_count);
            end else begin
                $display("[%0t ns] FAIL – NXT pulse counts wrong: in=%0d (exp 2) out=%0d (exp 2).",
                         $time, nxt_in_count, nxt_out_count);
            end
            #20 $finish;
        end
    end
end

// ---- Timeout ---------------------------------------------------------------
initial begin
    #200000;
    $display("[%0t ns] FAIL – scheduler: TIMEOUT (%0d dispatched, %0d/31 finished, NXT in×%0d out×%0d).",
             $time, dispatch_count, finish_count, nxt_in_count, nxt_out_count);
    $finish;
end

// ---- Waveform dump ---------------------------------------------------------
initial begin
    $dumpfile("tb_scheduler.vcd");
    $dumpvars(0, top);
end

// ---- Main stimulus ---------------------------------------------------------
integer k;
initial begin
    reset             = 1'b1;
    prog_mem_wait     = 1'b0;
    start_program     = 1'b0;
    program_addr      = {PROG_ADDR_BITS{1'b0}};
    mark_buff_as_full = 1'b0;
    full_buff_id      = 4'd0;
    full_buff_usage   = 3'd0;

    // ---- Populate program memory ------------------------------------------
    //  task_inst(src2, src1, acc, cfg, ntgts, tgt)
    // Layer 1: seeds {0,1,2} → intermediates {3,4,5}
    prog_mem[ 0] = task_inst(5'd1,  5'd0,  1'b0, 5'd0,  3'd2, 5'd3);  // T00
    prog_mem[ 1] = task_inst(5'd2,  5'd1,  1'b1, 5'd1,  3'd2, 5'd4);  // T01
    prog_mem[ 2] = task_inst(5'd2,  5'd0,  1'b0, 5'd2,  3'd2, 5'd5);  // T02
    // Layer 2
    prog_mem[ 3] = task_inst(5'd4,  5'd3,  1'b1, 5'd3,  3'd2, 5'd6);  // T03
    prog_mem[ 4] = task_inst(5'd5,  5'd4,  1'b0, 5'd4,  3'd2, 5'd7);  // T04
    prog_mem[ 5] = task_inst(5'd5,  5'd3,  1'b1, 5'd5,  3'd2, 5'd8);  // T05
    // Layer 3 (recycles buffers 0,1,2)
    prog_mem[ 6] = task_inst(5'd7,  5'd6,  1'b0, 5'd6,  3'd2, 5'd0);  // T06
    prog_mem[ 7] = task_inst(5'd8,  5'd7,  1'b1, 5'd7,  3'd2, 5'd1);  // T07
    prog_mem[ 8] = task_inst(5'd8,  5'd6,  1'b0, 5'd8,  3'd2, 5'd2);  // T08
    // Layer 4
    prog_mem[ 9] = task_inst(5'd1,  5'd0,  1'b1, 5'd9,  3'd2, 5'd3);  // T09
    prog_mem[10] = task_inst(5'd2,  5'd1,  1'b0, 5'd10, 3'd2, 5'd4);  // T10
    prog_mem[11] = task_inst(5'd2,  5'd0,  1'b1, 5'd11, 3'd2, 5'd5);  // T11
    // Layer 5
    prog_mem[12] = task_inst(5'd4,  5'd3,  1'b0, 5'd12, 3'd2, 5'd6);  // T12
    prog_mem[13] = task_inst(5'd5,  5'd4,  1'b1, 5'd13, 3'd2, 5'd7);  // T13
    prog_mem[14] = task_inst(5'd5,  5'd3,  1'b0, 5'd14, 3'd2, 5'd8);  // T14
    // Layer 6
    prog_mem[15] = task_inst(5'd7,  5'd6,  1'b1, 5'd15, 3'd2, 5'd0);  // T15
    prog_mem[16] = task_inst(5'd8,  5'd7,  1'b0, 5'd16, 3'd2, 5'd1);  // T16
    prog_mem[17] = task_inst(5'd8,  5'd6,  1'b1, 5'd17, 3'd2, 5'd2);  // T17
    // Layer 7
    prog_mem[18] = task_inst(5'd1,  5'd0,  1'b0, 5'd18, 3'd2, 5'd3);  // T18
    prog_mem[19] = task_inst(5'd2,  5'd1,  1'b1, 5'd19, 3'd2, 5'd4);  // T19
    prog_mem[20] = task_inst(5'd2,  5'd0,  1'b0, 5'd20, 3'd2, 5'd5);  // T20
    // Layer 8
    prog_mem[21] = task_inst(5'd4,  5'd3,  1'b1, 5'd21, 3'd2, 5'd6);  // T21
    prog_mem[22] = task_inst(5'd5,  5'd4,  1'b0, 5'd22, 3'd2, 5'd7);  // T22
    prog_mem[23] = task_inst(5'd5,  5'd3,  1'b1, 5'd23, 3'd2, 5'd8);  // T23
    // Layer 9 (drains into fresh buffers 9,10,11)
    prog_mem[24] = task_inst(5'd7,  5'd6,  1'b0, 5'd24, 3'd2, 5'd9);  // T24
    prog_mem[25] = task_inst(5'd8,  5'd7,  1'b1, 5'd25, 3'd2, 5'd10); // T25
    prog_mem[26] = task_inst(5'd8,  5'd6,  1'b0, 5'd26, 3'd2, 5'd11); // T26
    // Layer 10 (final outputs into buffers 12,13,14)
    prog_mem[27] = task_inst(5'd10, 5'd9,  1'b1, 5'd27, 3'd1, 5'd12); // T27
    prog_mem[28] = task_inst(5'd11, 5'd10, 1'b0, 5'd28, 3'd1, 5'd13); // T28
    prog_mem[29] = task_inst(5'd11, 5'd9,  1'b1, 5'd29, 3'd1, 5'd14); // T29
    prog_mem[30] = nxt_inst(1'b1, 1'b1);  // NXT: advance both input and output
    prog_mem[31] = nxt_inst(1'b1, 1'b0);  // NXT: advance input only
    prog_mem[32] = nxt_inst(1'b0, 1'b1);  // NXT: advance output only
    // T30: post-NXT task – consumes {12,13} (written by T27/T28), writes buffer 15
    // Verifies execution resumes after NXT and the table still works correctly.
    prog_mem[33] = task_inst(5'd13, 5'd12, 1'b0, 5'd30, 3'd1, 5'd15); // T30
    prog_mem[34] = STOP_INST;
    for (k = 35; k < 64; k = k + 1) prog_mem[k] = 32'h0;

    // ---- Release reset ---------------------------------------------------
    repeat(4) @(posedge clk); #1;
    reset = 1'b0;

    // ---- Pre-fill seed buffers 0, 1, 2 (2 consumers each) ---------------
    @(posedge clk); #1;
    mark_buff_as_full = 1'b1; full_buff_id = 4'd0; full_buff_usage = 3'd2;
    @(posedge clk); #1;
    full_buff_id = 4'd1;
    @(posedge clk); #1;
    full_buff_id = 4'd2;
    @(posedge clk); #1;
    mark_buff_as_full = 1'b0;

    // ---- Start program ---------------------------------------------------
    @(posedge clk); #1;
    start_program = 1'b1;
    program_addr  = {PROG_ADDR_BITS{1'b0}};
    @(posedge clk); #1;
    start_program = 1'b0;
end

endmodule
