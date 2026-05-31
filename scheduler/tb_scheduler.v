// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
`include "../shared/constants.v"
`timescale 1ns/1ps

// ============================================================================
// dummy_acc  –  simple accelerator stub
//
// Accepts a task when start_i is asserted and target_acc_i matches ACC_ID.
// Completes after a pseudo-random 20–30 cycle delay.
// ============================================================================
module dummy_acc #(parameter ACC_ID = 0, parameter TGT_ACC_SZ = 2) (
    input  wire                  clk,
    input  wire                  reset,
    input  wire                  start_i,
    input  wire [TGT_ACC_SZ-1:0] target_acc_i,
    output reg                   busy_o,
    output reg                   finished_o,
    output reg                   result_o
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
        if (start_i && (target_acc_i == ACC_ID) && !busy_o) begin
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
// Program: 9 two-word TASK instructions + 3 NXT + 1 two-word TASK + STOP
//
// Dependency graph:
//
//  Layer 1 (T00–T02): {0,1}→3,  {1,2}→4,  {0,2}→5    #tgts=2
//  Layer 2 (T03–T05): {3,4}→6,  {4,5}→7,  {3,5}→8    #tgts=2
//  Layer 3 (T06–T08): {6,7}→9,  {7,8}→10, {6,8}→11   #tgts=1
//  addr 18: NXT(in+out)  – barrier: fires once table empty after T06–T08 dispatched
//  addr 19: NXT(in only) – immediate (table already empty)
//  addr 20: NXT(out only)– immediate
//  addr 21–22: T09 {9,10}→12 #tgts=1  (post-NXT task)
//  addr 23: STOP
//
// Each TASK is two consecutive 32-bit words; see scheduler CLAUDE.md for encoding.
// Source buffers use slot 0 and slot 1 (short, mode=SRC); target uses slot 5 (long).
//
// Checks:
//   - NXT must not fire before all 9 pre-NXT tasks dispatched
//   - nxt_in_count == 2, nxt_out_count == 2
//   - T09 dispatches after NXT and completes (finish_count reaches 10)
//
// Seed buffers 0, 1, 2 are pre-filled (2 consumers each) before START.
// Acc assignment alternates 0/1 across tasks.  Colour is always 0.
// ============================================================================
module top;

// ---- Parameters matching DUT -----------------------------------------------
localparam TGT_ACC_SZ          = 3;
localparam TGT_COUNT_SZ        = 3;
localparam CFG_ID_SZ           = 5;
localparam NUM_BUFFERS         = 16;
localparam COL_BUFF_ID_SZ      = 16;
localparam NUM_SCH_ENTRIES     = 4;
localparam NUM_HW_ACCELERATORS = 5;
localparam PROG_ADDR_BITS      = 10;
localparam PROG_DATA_BITS      = 32;
localparam BUFF_INDX_SZ        = 4;
localparam NUM_SLOTS           = 6;
localparam MODE_SZ             = 2;

// Derived entry data layout (must match scheduler.v internals):
localparam SLOT_SHORT_SZ = MODE_SZ + BUFF_INDX_SZ;                    // 6
localparam SLOT_LONG_SZ  = MODE_SZ + BUFF_INDX_SZ + TGT_COUNT_SZ;    // 9
localparam ENTRY_DATA_SZ = 3*SLOT_SHORT_SZ + 3*SLOT_LONG_SZ
                           + 1 + TGT_ACC_SZ + CFG_ID_SZ;              // 54

// Entry field offsets within buffer_info_o:
localparam LONG_BASE   = 3 * SLOT_SHORT_SZ;              // 18
localparam E_COLOUR    = LONG_BASE + 3 * SLOT_LONG_SZ;   // 45
localparam E_ACC_START = E_COLOUR + 1;                   // 46
localparam E_CFG_START = E_ACC_START + TGT_ACC_SZ;       // 49

localparam MODE_UNUSED = 2'b00;
localparam MODE_SRC    = 2'b01;
localparam MODE_RW     = 2'b10;
localparam MODE_TGT    = 2'b11;

// ---- Instruction encoding --------------------------------------------------
//
// TASK Word 1:
//   [2:0]=3'b000(opcode), [4:3]=acc_id, [9:5]=cfg_id, [10]=colour,
//   [12:11]=slot0_mode, [16:13]=slot0_id,
//   [18:17]=slot1_mode, [22:19]=slot1_id,
//   [24:23]=slot2_mode, [28:25]=slot2_id, [31:29]=reserved
//
// TASK Word 2:
//   [1:0]=2'b00(sentinel),
//   [3:2]=slot3_mode, [7:4]=slot3_id, [10:8]=slot3_ntgt,
//   [12:11]=slot4_mode, [16:13]=slot4_id, [19:17]=slot4_ntgt,
//   [21:20]=slot5_mode, [25:22]=slot5_id, [28:26]=slot5_ntgt, [31:29]=reserved

function [31:0] tw1;
    input [1:0] acc;  input [4:0] cfg;  input colour;
    input [1:0] m0;   input [3:0] id0;
    input [1:0] m1;   input [3:0] id1;
    input [1:0] m2;   input [3:0] id2;
    tw1 = {3'b000, id2, m2, id1, m1, id0, m0, colour, cfg, acc, 3'b000};
endfunction

function [31:0] tw2;
    input [1:0] m3; input [3:0] id3; input [2:0] n3;
    input [1:0] m4; input [3:0] id4; input [2:0] n4;
    input [1:0] m5; input [3:0] id5; input [2:0] n5;
    tw2 = {3'b000, n5, id5, m5, n4, id4, m4, n3, id3, m3, 2'b00};
endfunction

// Convenience: slot0=SRC/src0, slot1=SRC/src1, slots 2–4 unused, slot5=TGT/tgt
function [31:0] simple_w1;
    input [1:0] acc;  input [4:0] cfg;
    input [3:0] src0; input [3:0] src1;
    simple_w1 = tw1(acc, cfg, 1'b0,
                    MODE_SRC, src0, MODE_SRC, src1, MODE_UNUSED, 4'd0);
endfunction

function [31:0] simple_w2;
    input [3:0] tgt; input [2:0] ntgt;
    simple_w2 = tw2(MODE_UNUSED, 4'd0, 3'd0,
                    MODE_UNUSED, 4'd0, 3'd0,
                    MODE_TGT, tgt, ntgt);
endfunction

localparam STOP_INST = 32'h00000002;   // opcode = 3'b010

function [31:0] nxt_inst;
    input nxt_in;
    input nxt_out;
    nxt_inst = {26'b0, nxt_out, nxt_in, 1'b0, 3'b100};
endfunction

localparam INST_LOOP    = 3'b110;
localparam INST_LOOPEND = 3'b111;

// LOOP: [31:6]=count, [5:3]=loop id, [2:0]=opcode.  Body runs count+1 times.
function [31:0] loop_inst;
    input [2:0]  id;
    input [25:0] count;
    loop_inst = {count, id, INST_LOOP};
endfunction

// LOOPEND: [5:3]=loop id, [2:0]=opcode.  Now a drain barrier (waits for table_empty).
function [31:0] loopend_inst;
    input [2:0] id;
    loopend_inst = {26'b0, id, INST_LOOPEND};
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
wire [TGT_ACC_SZ-1:0]          target_acc;
wire [ENTRY_DATA_SZ-1:0]       buffer_info;
wire                           nxt_in_pulse;
wire                           nxt_out_pulse;

reg  [31:0]                    sys_addr_tb;
reg                            sys_req_tb;
reg  [31:0]                    sys_data_tb;
wire                           sys_ack_tb;

// ---- Program memory (64-word ROM, combinatorial read) ---------------------
reg [31:0] prog_mem [0:63];
assign prog_mem_data = prog_mem[prog_mem_addr[5:0]];

// ---- Dummy accelerators ---------------------------------------------------
wire busy_0,     busy_1;
wire finished_0, finished_1;
wire result_0,   result_1;

assign acc_busy_w     = {3'b0, busy_1,     busy_0};
assign acc_finished_w = {3'b0, finished_1, finished_0};
assign acc_result_w   = {3'b0, result_1,   result_0};

dummy_acc #(.ACC_ID(0), .TGT_ACC_SZ(TGT_ACC_SZ)) acc0 (
    .clk(clk), .reset(reset),
    .start_i(start_new_block), .target_acc_i(target_acc),
    .busy_o(busy_0), .finished_o(finished_0), .result_o(result_0)
);

dummy_acc #(.ACC_ID(1), .TGT_ACC_SZ(TGT_ACC_SZ)) acc1 (
    .clk(clk), .reset(reset),
    .start_i(start_new_block), .target_acc_i(target_acc),
    .busy_o(busy_1), .finished_o(finished_1), .result_o(result_1)
);

// ---- DUT ------------------------------------------------------------------
scheduler #(
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
    .sys_req_i(sys_req_tb),
    .sys_ack_o(sys_ack_tb),
    .sys_addr_i(sys_addr_tb),
    .sys_data_i(sys_data_tb),
    .sys_data_o(),
    .prog_mem_addr_o(prog_mem_addr),
    .prog_mem_data_i(prog_mem_data),
    .prog_mem_req_o(prog_mem_req),
    .prog_mem_wait_i(prog_mem_wait),
    .prog_mem_wr_o(),
    .prog_mem_wr_addr_o(),
    .prog_mem_wr_data_o(),
    .prog_mem_wr_wait_i(1'b0),
    .acc_busy_i(acc_busy_w),
    .acc_finished_i(acc_finished_w),
    .acc_result_i(acc_result_w),
    .start_new_block_o(start_new_block),
    .target_acc_o(target_acc),
    .buffer_info_o(buffer_info),
    .nxt_input_pulse_o(nxt_in_pulse),
    .nxt_output_pulse_o(nxt_out_pulse),
    .fill_value_o(),
    .fill_block_size_o(),
    .cm_busy_i(1'b0)    // no config_manager in this tb — never back-pressure
);

// ---- Event monitors -------------------------------------------------------
integer dispatch_count;
integer finish_count;
integer nxt_in_count;
integer nxt_out_count;

// Phase counters: 0 = program 1, 1 = program 2 (RW contention), 2 = program 3 (LOOPEND barrier)
integer phase;
integer ph2_dispatch_count;
integer ph2_finish_count;
reg     ph2_t31_premature; // set if 2nd dispatch fires before 1st completion

// Phase 3 (LOOPEND drain barrier) counters:
integer ph3_dispatch_count;
integer ph3_finish_count;
reg     ph3_overlap;       // set if a new iteration's first task dispatches before the table drains

// Phase 4 (free-on-completion WAR) counters:
integer ph4_finish_count;
reg     ph4_r_done;        // reader (R, acc0) has completed
reg     ph4_war;           // set if writer (W) dispatches before reader R completes

initial begin
    dispatch_count    = 0;
    finish_count      = 0;
    nxt_in_count      = 0;
    nxt_out_count     = 0;
    phase             = 0;
    ph2_dispatch_count = 0;
    ph2_finish_count  = 0;
    ph2_t31_premature = 1'b0;
    ph3_dispatch_count = 0;
    ph3_finish_count  = 0;
    ph3_overlap       = 1'b0;
    ph4_finish_count  = 0;
    ph4_r_done        = 1'b0;
    ph4_war           = 1'b0;
end

always @(posedge clk) begin
    if (!reset) begin
        if (phase == 0) begin
            // ---- Program 1: dependency-chain + NXT test ----------------------
            if (start_new_block) begin
                dispatch_count = dispatch_count + 1;
                $display("[%0t ns] P1 DISPATCH #%0d  acc=%0d  cfg=%02d",
                    $time, dispatch_count,
                    buffer_info[E_ACC_START +: TGT_ACC_SZ],
                    buffer_info[E_CFG_START +: CFG_ID_SZ]);
            end
            if (acc_finished_w[0]) $display("[%0t ns] P1 ACC0 done", $time);
            if (acc_finished_w[1]) $display("[%0t ns] P1 ACC1 done", $time);
            finish_count = finish_count + acc_finished_w[0] + acc_finished_w[1];

            if (nxt_in_pulse || nxt_out_pulse) begin
                if (dispatch_count < 9) begin
                    $display("[%0t ns] P1 FAIL – NXT fired before all 9 tasks (dispatched=%0d).",
                             $time, dispatch_count);
                    $finish;
                end
                if (nxt_in_pulse) begin
                    nxt_in_count = nxt_in_count + 1;
                    $display("[%0t ns] NXT in_pulse #%0d", $time, nxt_in_count);
                end
                if (nxt_out_pulse) begin
                    nxt_out_count = nxt_out_count + 1;
                    $display("[%0t ns] NXT out_pulse #%0d", $time, nxt_out_count);
                end
            end

            if (finish_count >= 10) begin
                if (nxt_in_count == 2 && nxt_out_count == 2)
                    $display("[%0t ns] P1 PASS – 10 tasks; NXT in×%0d out×%0d; T09 after NXT.",
                             $time, nxt_in_count, nxt_out_count);
                else begin
                    $display("[%0t ns] P1 FAIL – NXT counts wrong: in=%0d (exp 2) out=%0d (exp 2).",
                             $time, nxt_in_count, nxt_out_count);
                    $finish;
                end
                phase = 1; // signal initial block to launch program 2
            end
        end else if (phase == 1) begin
            // ---- Program 2: RW contention test --------------------------------
            // Count completions first so a same-cycle dispatch+complete is not a false fail.
            if (acc_finished_w[0]) $display("[%0t ns] P2 ACC0 done", $time);
            if (acc_finished_w[1]) $display("[%0t ns] P2 ACC1 done", $time);
            ph2_finish_count = ph2_finish_count + acc_finished_w[0] + acc_finished_w[1];
            if (start_new_block) begin
                ph2_dispatch_count = ph2_dispatch_count + 1;
                $display("[%0t ns] P2 DISPATCH #%0d  acc=%0d",
                         $time, ph2_dispatch_count,
                         buffer_info[E_ACC_START +: TGT_ACC_SZ]);
                // 2nd dispatch must not fire while buf2 is still busy (0 completions yet).
                if (ph2_dispatch_count == 2 && ph2_finish_count == 0)
                    ph2_t31_premature = 1'b1;
            end

            if (ph2_finish_count >= 2) begin
                if (!ph2_t31_premature)
                    $display("[%0t ns] P2 PASS – RW contention: T31 waited for T30 to complete.",
                             $time);
                else
                    $display("[%0t ns] P2 FAIL – T31 dispatched before T30 completed.", $time);
                phase = 2; // launch program 3 (LOOPEND barrier test)
            end
        end else if (phase == 2) begin
            // ---- Program 3: LOOPEND drain barrier (cross-timestep WAR) ---------
            // Count completions first so a same-cycle dispatch+complete is not a false fail.
            if (acc_finished_w[0]) $display("[%0t ns] P3 ACC0 done", $time);
            if (acc_finished_w[1]) $display("[%0t ns] P3 ACC1 done", $time);
            ph3_finish_count = ph3_finish_count + acc_finished_w[0] + acc_finished_w[1];
            if (start_new_block) begin
                ph3_dispatch_count = ph3_dispatch_count + 1;
                $display("[%0t ns] P3 DISPATCH #%0d  acc=%0d  cfg=%0d",
                         $time, ph3_dispatch_count,
                         buffer_info[E_ACC_START +: TGT_ACC_SZ],
                         buffer_info[E_CFG_START +: CFG_ID_SZ]);
                // Each timestep's first task (cfg=20) must see a fully drained table:
                // every previously dispatched task has completed.  If not, the LOOPEND
                // barrier failed and timestep t+1 overlapped timestep t.
                if (buffer_info[E_CFG_START +: CFG_ID_SZ] == 20 &&
                    ph3_finish_count != ph3_dispatch_count - 1)
                    ph3_overlap = 1'b1;
            end
            if (ph3_finish_count >= 6) begin
                if (!ph3_overlap)
                    $display("[%0t ns] P3 PASS - LOOPEND barrier: 3 timesteps, no cross-iteration overlap.",
                             $time);
                else
                    $display("[%0t ns] P3 FAIL - cross-iteration overlap: timestep t+1 dispatched before timestep t drained.",
                             $time);
                phase = 3; // launch program 4 (free-on-completion WAR test)
            end
        end else begin
            // ---- Program 4: free source buffer on COMPLETION, not dispatch -----
            // R (acc0) reads X(buf5); W (acc1) overwrites X. W is gated only by X
            // becoming free.  With free-on-completion, W must wait for R to finish.
            // Count completions first so a same-cycle complete+dispatch is not missed.
            if (acc_finished_w[0]) begin
                $display("[%0t ns] P4 ACC0 done (reader R)", $time);
                ph4_r_done = 1'b1;
            end
            if (acc_finished_w[1]) $display("[%0t ns] P4 ACC1 done (writer W)", $time);
            ph4_finish_count = ph4_finish_count + acc_finished_w[0] + acc_finished_w[1];
            if (start_new_block) begin
                $display("[%0t ns] P4 DISPATCH  acc=%0d  cfg=%0d", $time,
                         buffer_info[E_ACC_START +: TGT_ACC_SZ],
                         buffer_info[E_CFG_START +: CFG_ID_SZ]);
                // Writer W (cfg=31) must not dispatch until reader R has completed.
                if (buffer_info[E_CFG_START +: CFG_ID_SZ] == 31 && !ph4_r_done)
                    ph4_war = 1'b1;
            end
            if (ph4_finish_count >= 2) begin
                if (!ph4_war)
                    $display("[%0t ns] P4 PASS - WAR: writer waited for reader to complete before overwriting buffer.",
                             $time);
                else
                    $display("[%0t ns] P4 FAIL - WAR: writer overwrote buffer while reader still reading.",
                             $time);
                #20 $finish;
            end
        end
    end
end

// ---- Timeout ---------------------------------------------------------------
initial begin
    #200000;
    $display("[%0t ns] FAIL – scheduler: TIMEOUT (%0d dispatched, %0d/10 finished, NXT in×%0d out×%0d).",
             $time, dispatch_count, finish_count, nxt_in_count, nxt_out_count);
    $finish;
end

// ---- Waveform dump ---------------------------------------------------------
initial begin
    $dumpfile("tb_scheduler.vcd");
    $dumpvars(0, top);
end

// ---- AXI write task --------------------------------------------------------
// ack is combinatorial, so req need only be held for one clock cycle.
task axi_write;
    input [31:0] addr;
    input [31:0] data;
    begin
        @(posedge clk); #1;
        sys_addr_tb = addr;
        sys_data_tb = data;
        sys_req_tb  = 1'b1;
        @(posedge clk); #1;
        sys_req_tb  = 1'b0;
    end
endtask

// ---- Main stimulus ---------------------------------------------------------
integer k;
initial begin
    reset             = 1'b1;
    prog_mem_wait     = 1'b0;
    sys_req_tb        = 1'b0;
    sys_addr_tb       = 32'b0;
    sys_data_tb       = 32'b0;

    // ---- Populate program memory ------------------------------------------
    // simple_w1(acc, cfg, src0, src1) + simple_w2(tgt, ntgt)
    //
    // Layer 1: seeds {0,1,2} → intermediates {3,4,5}
    prog_mem[ 0] = simple_w1(2'd0, 5'd0, 4'd0, 4'd1); prog_mem[ 1] = simple_w2(4'd3,  3'd2); // T00
    prog_mem[ 2] = simple_w1(2'd1, 5'd1, 4'd1, 4'd2); prog_mem[ 3] = simple_w2(4'd4,  3'd2); // T01
    prog_mem[ 4] = simple_w1(2'd0, 5'd2, 4'd0, 4'd2); prog_mem[ 5] = simple_w2(4'd5,  3'd2); // T02
    // Layer 2: {3,4,5} → {6,7,8}
    prog_mem[ 6] = simple_w1(2'd1, 5'd3, 4'd3, 4'd4); prog_mem[ 7] = simple_w2(4'd6,  3'd2); // T03
    prog_mem[ 8] = simple_w1(2'd0, 5'd4, 4'd4, 4'd5); prog_mem[ 9] = simple_w2(4'd7,  3'd2); // T04
    prog_mem[10] = simple_w1(2'd1, 5'd5, 4'd3, 4'd5); prog_mem[11] = simple_w2(4'd8,  3'd2); // T05
    // Layer 3: {6,7,8} → {9,10,11}
    prog_mem[12] = simple_w1(2'd0, 5'd6, 4'd6, 4'd7); prog_mem[13] = simple_w2(4'd9,  3'd1); // T06
    prog_mem[14] = simple_w1(2'd1, 5'd7, 4'd7, 4'd8); prog_mem[15] = simple_w2(4'd10, 3'd1); // T07
    prog_mem[16] = simple_w1(2'd0, 5'd8, 4'd6, 4'd8); prog_mem[17] = simple_w2(4'd11, 3'd1); // T08
    // NXT block (single words):
    prog_mem[18] = nxt_inst(1'b1, 1'b1);  // NXT: advance both input and output
    prog_mem[19] = nxt_inst(1'b1, 1'b0);  // NXT: advance input only
    prog_mem[20] = nxt_inst(1'b0, 1'b1);  // NXT: advance output only
    // T09: post-NXT task – consumes {9,10} (written by T06/T07), writes buffer 12
    prog_mem[21] = simple_w1(2'd1, 5'd9, 4'd9, 4'd10); prog_mem[22] = simple_w2(4'd12, 3'd1); // T09
    prog_mem[23] = STOP_INST;

    // ---- Program 2: RW contention test (words 32–36) ---------------------
    // T30: SRC=buf0, RW=buf2/ntgt=2, TGT=buf3/ntgt=1  (acc 0, cfg 10)
    // T31: SRC=buf1, RW=buf2/ntgt=1, TGT=buf4/ntgt=1  (acc 1, cfg 11)
    // Both tasks need buf2 as RW. T30 enters table first (lower index) and
    // dispatches first; buf2 goes busy. T31 stalls until T30 completes and
    // buf2 becomes full again.
    prog_mem[32] = tw1(2'd0, 5'd10, 1'b0,
                       MODE_SRC, 4'd0, MODE_UNUSED, 4'd0, MODE_UNUSED, 4'd0);
    prog_mem[33] = tw2(MODE_RW, 4'd2, 3'd2,
                       MODE_UNUSED, 4'd0, 3'd0,
                       MODE_TGT, 4'd3, 3'd1);
    prog_mem[34] = tw1(2'd1, 5'd11, 1'b0,
                       MODE_SRC, 4'd1, MODE_UNUSED, 4'd0, MODE_UNUSED, 4'd0);
    prog_mem[35] = tw2(MODE_RW, 4'd2, 3'd1,
                       MODE_UNUSED, 4'd0, 3'd0,
                       MODE_TGT, 4'd4, 3'd1);
    prog_mem[36] = STOP_INST;

    for (k = 24; k < 32; k = k + 1) prog_mem[k] = 32'h0;
    for (k = 37; k < 64; k = k + 1) prog_mem[k] = 32'h0;

    // ---- Program 3: LOOPEND drain barrier test (words 40–46) -------------
    // 3 timesteps (LOOP count=2).  Body = 2 tasks:
    //   task1 (acc0,cfg20): SRC inA(buf0), RW P(buf8), TGT mid(buf9 ntgt=1)
    //   task2 (acc1,cfg21): SRC mid(buf9), RW Q(buf11)
    // P,Q are persistent recurrent RW state; mid is the per-timestep inter-layer
    // output, produced then consumed within each iteration.  task1 on acc0 and
    // task2 on acc1 so any cross-iteration overlap would be gated purely by
    // buffer state, not accelerator-busy.  With the LOOPEND barrier each timestep
    // must fully drain before the next begins.  (Assignments come after the
    // zero-fill loops above so they are not overwritten.)
    prog_mem[40] = loop_inst(3'd0, 26'd2);                  // LOOP id0, 3 iterations
    prog_mem[41] = tw1(2'd0, 5'd20, 1'b0,
                       MODE_SRC, 4'd0, MODE_UNUSED, 4'd0, MODE_UNUSED, 4'd0);
    prog_mem[42] = tw2(MODE_RW, 4'd8, 3'd0,                 // slot3: RW P(buf8)
                       MODE_UNUSED, 4'd0, 3'd0,
                       MODE_TGT, 4'd9, 3'd1);               // slot5: TGT mid(buf9)
    prog_mem[43] = tw1(2'd1, 5'd21, 1'b0,
                       MODE_SRC, 4'd9, MODE_UNUSED, 4'd0, MODE_UNUSED, 4'd0);
    prog_mem[44] = tw2(MODE_RW, 4'd11, 3'd0,                // slot3: RW Q(buf11)
                       MODE_UNUSED, 4'd0, 3'd0,
                       MODE_UNUSED, 4'd0, 3'd0);
    prog_mem[45] = loopend_inst(3'd0);                      // LOOPEND id0 (drain barrier)
    prog_mem[46] = STOP_INST;

    // ---- Program 4: free-on-completion WAR test (words 50–54) ------------
    //   R (acc0,cfg30): SRC X(buf5)  -> TGT buf6   (reader of X)
    //   W (acc1,cfg31): SRC buf7     -> TGT X(buf5) (overwrites X)
    // W is gated only by X(buf5) becoming free.  With the SOURCE consume moved
    // to completion, X frees only when R *completes*, so W must wait for R.
    prog_mem[50] = tw1(2'd0, 5'd30, 1'b0,
                       MODE_SRC, 4'd5, MODE_UNUSED, 4'd0, MODE_UNUSED, 4'd0);
    prog_mem[51] = tw2(MODE_UNUSED, 4'd0, 3'd0,
                       MODE_UNUSED, 4'd0, 3'd0,
                       MODE_TGT, 4'd6, 3'd1);               // R writes buf6
    prog_mem[52] = tw1(2'd1, 5'd31, 1'b0,
                       MODE_SRC, 4'd7, MODE_UNUSED, 4'd0, MODE_UNUSED, 4'd0);
    prog_mem[53] = tw2(MODE_UNUSED, 4'd0, 3'd0,
                       MODE_UNUSED, 4'd0, 3'd0,
                       MODE_TGT, 4'd5, 3'd1);               // W overwrites X(buf5)
    prog_mem[54] = STOP_INST;

    // ---- Release reset ---------------------------------------------------
    repeat(4) @(posedge clk); #1;
    reset = 1'b0;

    // ---- Pre-fill seed buffers 0, 1, 2 (2 consumers each) ---------------
    // MARK_BUFF_FULL: addr=0xE050_0000, data[3:0]=buf_id, data[6:4]=usage
    axi_write(32'hE050_0000, {25'b0, 3'd2, 4'd0});  // buf 0, ntgt=2
    axi_write(32'hE050_0000, {25'b0, 3'd2, 4'd1});  // buf 1, ntgt=2
    axi_write(32'hE050_0000, {25'b0, 3'd2, 4'd2});  // buf 2, ntgt=2

    // ---- Start program 1 ------------------------------------------------
    axi_write(32'hE000_0000, 32'd0);            // LOAD_PC: start at word 0
    axi_write(32'hE010_0000, 32'b0);            // START

    // ---- Wait for program 1 to finish (monitor sets phase=1) -------------
    wait(phase == 1);

    // ---- Reset for program 2 --------------------------------------------
    reset = 1'b1;
    repeat(4) @(posedge clk); #1;
    reset = 1'b0;

    // ---- Pre-fill program 2 seed buffers (colour=0 by reset default) ----
    // buf0 = source for T30 (ntgt=1); buf1 = source for T31 (ntgt=1)
    // buf2 = shared RW buffer (ntgt value overwritten on first RW completion)
    axi_write(32'hE050_0000, {25'b0, 3'd1, 4'd0});  // buf 0, ntgt=1
    axi_write(32'hE050_0000, {25'b0, 3'd1, 4'd1});  // buf 1, ntgt=1
    axi_write(32'hE050_0000, {25'b0, 3'd1, 4'd2});  // buf 2, ntgt=1

    // ---- Start program 2 ------------------------------------------------
    axi_write(32'hE000_0000, 32'd32);           // LOAD_PC: start at word 32
    axi_write(32'hE010_0000, 32'b0);            // START

    // ---- Wait for program 2 to finish (monitor sets phase=2) -------------
    wait(phase == 2);

    // ---- Reset for program 3 --------------------------------------------
    reset = 1'b1;
    repeat(4) @(posedge clk); #1;
    reset = 1'b0;

    // ---- Pre-fill program 3 seed buffers --------------------------------
    // inA(buf0): input, ntgt=3 so it stays full across all 3 timestep reads.
    // P(buf8), Q(buf11): recurrent RW state, seeded full so iter-1 RW slots dispatch.
    axi_write(32'hE050_0000, {25'b0, 3'd3, 4'd0});   // buf 0  (inA), ntgt=3
    axi_write(32'hE050_0000, {25'b0, 3'd1, 4'd8});   // buf 8  (P),   full
    axi_write(32'hE050_0000, {25'b0, 3'd1, 4'd11});  // buf 11 (Q),   full

    // ---- Start program 3 ------------------------------------------------
    axi_write(32'hE000_0000, 32'd40);           // LOAD_PC: start at word 40
    axi_write(32'hE010_0000, 32'b0);            // START

    // ---- Wait for program 3 to finish (monitor sets phase=3) -------------
    wait(phase == 3);

    // ---- Reset for program 4 --------------------------------------------
    reset = 1'b1;
    repeat(4) @(posedge clk); #1;
    reset = 1'b0;

    // ---- Pre-fill program 4 seed buffers --------------------------------
    // X(buf5): read by R (ntgt=1); buf7: source for W (ntgt=1).
    axi_write(32'hE050_0000, {25'b0, 3'd1, 4'd5});   // buf 5 (X),   ntgt=1
    axi_write(32'hE050_0000, {25'b0, 3'd1, 4'd7});   // buf 7,       ntgt=1

    // ---- Start program 4 ------------------------------------------------
    axi_write(32'hE000_0000, 32'd50);           // LOAD_PC: start at word 50
    axi_write(32'hE010_0000, 32'b0);            // START
end

endmodule
