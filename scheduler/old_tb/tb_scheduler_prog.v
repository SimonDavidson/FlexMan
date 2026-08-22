// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
// Scheduler testbench: 30-instruction program with ~24 TASK instructions.
// Tests table-full stalls, source-buffer-not-ready stalls, and accelerator-busy
// stalls across three rounds of computation reusing the same buffer pool.
//
// Network topology (all colour=0, no colour-match checking in current RTL):
//
//   Round 1: pre-filled B0-B7 → compute → B8-B15
//   Round 2: B8-B15           → compute → B0-B7   (freed by round 1)
//   Round 3: B0-B7            → compute → B8-B15  (freed by round 2)
//   [24] JUMP → [27]          (skips two dead tasks at [25],[26])
//   [27] STOP

`include "../../shared/constants.v"

`timescale 10ps/1ps

localparam PROG_ADDR_BITS      = 10;
localparam PROG_DATA_BITS      = 32;
localparam NUM_HW_ACCELERATORS = 2;
localparam TGT_ACC_SZ          = 2;
localparam CFG_ID_SZ           = 5;
localparam SCH_ENTRY_SZ        = 32;
localparam NUM_BUFFERS         = 16;
localparam COL_BUFF_ID_SZ      = 16;
localparam NUM_SCH_ENTRIES     = 4;

// Cycles each simulated accelerator task takes to complete.
localparam TASK_LATENCY = 12;

module top();

// -----------------------------------------------------------------------
// DUT ports
// -----------------------------------------------------------------------
reg  clk, reset, test_stall_pipe;
reg  start_program;
reg  [PROG_ADDR_BITS-1:0]      program_addr;
reg  [NUM_HW_ACCELERATORS-1:0] acc_busy;
reg  [NUM_HW_ACCELERATORS-1:0] acc_finished;
reg  [NUM_HW_ACCELERATORS-1:0] acc_result;
reg  mark_buff_as_full;
reg  [3:0] full_buff_id;
reg  [2:0] full_buff_usage;
wire start_new_block;
wire [TGT_ACC_SZ-1:0]    target_acc;
wire [SCH_ENTRY_SZ-1:0]  buffer_info;
wire [`ADDR_SIZE-1:0]    prog_mem_addr;
wire                     prog_mem_req;
reg  [PROG_DATA_BITS-1:0] prog_mem_data;
reg  prog_mem_wait;

// -----------------------------------------------------------------------
// Program memory: 30 instructions in a 32-entry ROM
// -----------------------------------------------------------------------
reg [31:0] prog_mem [0:31];

// Combinatorial address decode — 1-cycle latency, no wait states.
always @ (prog_mem_addr)
    prog_mem_data = prog_mem[prog_mem_addr[4:0]];

// Build a TASK instruction word from its fields.
// Encoding: {2'b00, src2[4:0], src1[4:0], acc[2:0], cfg[4:0],
//             ntgt[2:0], col, tgt[4:0], 3'b000}
function [31:0] task_inst;
    input [4:0] src2, src1;
    input [2:0] acc;
    input [4:0] cfg;
    input [2:0] ntgt;
    input       col;
    input [4:0] tgt;
    begin
        task_inst = {2'b00, src2, src1, acc, cfg, ntgt, col, tgt, 3'b000};
    end
endfunction

// Build a JUMP instruction.  PC loads inst_word[PROG_ADDR_BITS+2:3].
function [31:0] jump_inst;
    input [PROG_ADDR_BITS-1:0] addr;
    begin
        jump_inst = {{(29-PROG_ADDR_BITS){1'b0}}, addr, 3'b001};
    end
endfunction

localparam STOP_INST = 32'h00000002;

// -----------------------------------------------------------------------
// Program: three rounds of 8 TASK instructions, a JUMP, and a STOP.
//
// Buffer usage-count accounting (#tgt = number of consumer tasks):
//
//   Pre-fill: B0-B7, #tgt=2 each (two round-1 tasks consume each buffer).
//
//   Round 1 outputs (acc's target #tgt = number of round-2 consumers):
//     B8  (#tgt=2): consumed by T8(src1) and T9(src1)
//     B9  (#tgt=2): consumed by T8(src2) and T10(src1)
//     B10 (#tgt=2): consumed by T9(src2) and T11(src1)
//     B11 (#tgt=2): consumed by T10(src2) and T11(src2)
//     B12 (#tgt=2): consumed by T12(src1) and T13(src1)
//     B13 (#tgt=2): consumed by T12(src2) and T14(src1)
//     B14 (#tgt=2): consumed by T13(src2) and T15(src1)
//     B15 (#tgt=2): consumed by T14(src2) and T15(src2)
//
//   Round 2 outputs (#tgt=2 each): B0-B7 refilled for round 3.
//   Round 3 outputs (#tgt=1 each): B8-B15 as final results.
// -----------------------------------------------------------------------
initial begin
    // Round 1: sources B0-B7 (pre-filled), write to B8-B15
    prog_mem[ 0] = task_inst(5'd1, 5'd0, 3'd0, 5'd1, 3'd2, 1'b0, 5'd8);  // T0:  B0+B1→B8,  acc0
    prog_mem[ 1] = task_inst(5'd3, 5'd2, 3'd1, 5'd1, 3'd2, 1'b0, 5'd9);  // T1:  B2+B3→B9,  acc1
    prog_mem[ 2] = task_inst(5'd2, 5'd0, 3'd0, 5'd1, 3'd2, 1'b0, 5'd10); // T2:  B0+B2→B10, acc0
    prog_mem[ 3] = task_inst(5'd3, 5'd1, 3'd1, 5'd1, 3'd2, 1'b0, 5'd11); // T3:  B1+B3→B11, acc1
    prog_mem[ 4] = task_inst(5'd5, 5'd4, 3'd0, 5'd2, 3'd2, 1'b0, 5'd12); // T4:  B4+B5→B12, acc0
    prog_mem[ 5] = task_inst(5'd6, 5'd4, 3'd1, 5'd2, 3'd2, 1'b0, 5'd13); // T5:  B4+B6→B13, acc1
    prog_mem[ 6] = task_inst(5'd7, 5'd5, 3'd0, 5'd2, 3'd2, 1'b0, 5'd14); // T6:  B5+B7→B14, acc0
    prog_mem[ 7] = task_inst(5'd7, 5'd6, 3'd1, 5'd2, 3'd2, 1'b0, 5'd15); // T7:  B6+B7→B15, acc1

    // Round 2: sources B8-B15 (filled by round 1), write back to B0-B7
    prog_mem[ 8] = task_inst(5'd9,  5'd8,  3'd0, 5'd3, 3'd2, 1'b0, 5'd0); // T8:  B8+B9→B0,   acc0
    prog_mem[ 9] = task_inst(5'd10, 5'd8,  3'd1, 5'd3, 3'd2, 1'b0, 5'd1); // T9:  B8+B10→B1,  acc1
    prog_mem[10] = task_inst(5'd11, 5'd9,  3'd0, 5'd3, 3'd2, 1'b0, 5'd2); // T10: B9+B11→B2,  acc0
    prog_mem[11] = task_inst(5'd11, 5'd10, 3'd1, 5'd3, 3'd2, 1'b0, 5'd3); // T11: B10+B11→B3, acc1
    prog_mem[12] = task_inst(5'd13, 5'd12, 3'd0, 5'd4, 3'd2, 1'b0, 5'd4); // T12: B12+B13→B4, acc0
    prog_mem[13] = task_inst(5'd14, 5'd12, 3'd1, 5'd4, 3'd2, 1'b0, 5'd5); // T13: B12+B14→B5, acc1
    prog_mem[14] = task_inst(5'd15, 5'd13, 3'd0, 5'd4, 3'd2, 1'b0, 5'd6); // T14: B13+B15→B6, acc0
    prog_mem[15] = task_inst(5'd15, 5'd14, 3'd1, 5'd4, 3'd2, 1'b0, 5'd7); // T15: B14+B15→B7, acc1

    // Round 3: sources B0-B7 (refilled by round 2), write to B8-B15
    prog_mem[16] = task_inst(5'd1, 5'd0, 3'd0, 5'd5, 3'd1, 1'b0, 5'd8);  // T16: B0+B1→B8,  acc0
    prog_mem[17] = task_inst(5'd3, 5'd2, 3'd1, 5'd5, 3'd1, 1'b0, 5'd9);  // T17: B2+B3→B9,  acc1
    prog_mem[18] = task_inst(5'd2, 5'd0, 3'd0, 5'd5, 3'd1, 1'b0, 5'd10); // T18: B0+B2→B10, acc0
    prog_mem[19] = task_inst(5'd3, 5'd1, 3'd1, 5'd5, 3'd1, 1'b0, 5'd11); // T19: B1+B3→B11, acc1
    prog_mem[20] = task_inst(5'd5, 5'd4, 3'd0, 5'd5, 3'd1, 1'b0, 5'd12); // T20: B4+B5→B12, acc0
    prog_mem[21] = task_inst(5'd6, 5'd4, 3'd1, 5'd5, 3'd1, 1'b0, 5'd13); // T21: B4+B6→B13, acc1
    prog_mem[22] = task_inst(5'd7, 5'd5, 3'd0, 5'd5, 3'd1, 1'b0, 5'd14); // T22: B5+B7→B14, acc0
    prog_mem[23] = task_inst(5'd7, 5'd6, 3'd1, 5'd5, 3'd1, 1'b0, 5'd15); // T23: B6+B7→B15, acc1

    // Control flow
    prog_mem[24] = jump_inst(10'd27);  // JUMP → address 27 (skip dead tasks)
    prog_mem[25] = task_inst(5'd14, 5'd12, 3'd0, 5'd6, 3'd1, 1'b0, 5'd0); // dead (skipped)
    prog_mem[26] = task_inst(5'd15, 5'd13, 3'd1, 5'd6, 3'd1, 1'b0, 5'd1); // dead (skipped)
    prog_mem[27] = STOP_INST;
    prog_mem[28] = task_inst(5'd0, 5'd1, 3'd0, 5'd7, 3'd1, 1'b0, 5'd2);   // after STOP – unreachable
    prog_mem[29] = task_inst(5'd0, 5'd1, 3'd1, 5'd7, 3'd1, 1'b0, 5'd3);   // after STOP – unreachable
    prog_mem[30] = 32'hDEADBEEF;
    prog_mem[31] = 32'hDEADBEEF;
end

// -----------------------------------------------------------------------
// Clock and reset
// -----------------------------------------------------------------------
initial clk = 1'b1;
always  #5 clk = ~clk;

initial begin
    reset          = 1'b1;
    repeat (3) @ (posedge clk);
    #1 reset       = 1'b0;
end

// -----------------------------------------------------------------------
// Waveform dump
// -----------------------------------------------------------------------
initial begin
    $dumpfile("top.vcd");
    $dumpvars(0, top);
end

// -----------------------------------------------------------------------
// Simulation timeout
// -----------------------------------------------------------------------
initial begin
    repeat (4000) @ (posedge clk);
    $display("TIMEOUT: simulation did not finish in time");
    $finish;
end

// -----------------------------------------------------------------------
// Stimulus: initialise signals, pre-fill B0-B7, then start the program
// -----------------------------------------------------------------------
initial begin
    #1;
    acc_busy          = 2'b00;
    acc_finished      = 2'b00;
    acc_result        = 2'b00;
    prog_mem_wait     = 1'b0;
    test_stall_pipe   = 1'b0;
    mark_buff_as_full = 1'b0;
    full_buff_id      = 4'b0;
    full_buff_usage   = 3'd2;
    start_program     = 1'b0;
    program_addr      = 10'b0;

    // Wait for reset to clear
    repeat (4) @ (posedge clk);

    // Pre-fill B0-B7, each with usage count 2 (two round-1 tasks consume each).
    // One buffer per cycle via the external marking interface.
    #1 mark_buff_as_full = 1'b1; full_buff_id = 4'd0; full_buff_usage = 3'd2;
    @ (posedge clk);
    #1 mark_buff_as_full = 1'b1; full_buff_id = 4'd1; full_buff_usage = 3'd2;
    @ (posedge clk);
    #1 mark_buff_as_full = 1'b1; full_buff_id = 4'd2; full_buff_usage = 3'd2;
    @ (posedge clk);
    #1 mark_buff_as_full = 1'b1; full_buff_id = 4'd3; full_buff_usage = 3'd2;
    @ (posedge clk);
    #1 mark_buff_as_full = 1'b1; full_buff_id = 4'd4; full_buff_usage = 3'd2;
    @ (posedge clk);
    #1 mark_buff_as_full = 1'b1; full_buff_id = 4'd5; full_buff_usage = 3'd2;
    @ (posedge clk);
    #1 mark_buff_as_full = 1'b1; full_buff_id = 4'd6; full_buff_usage = 3'd2;
    @ (posedge clk);
    #1 mark_buff_as_full = 1'b1; full_buff_id = 4'd7; full_buff_usage = 3'd2;
    @ (posedge clk);
    #1 mark_buff_as_full = 1'b0;

    // Start the program from address 0
    @ (posedge clk);
    #1 start_program = 1'b1; program_addr = 10'b0;
    @ (posedge clk);
    #1 start_program = 1'b0;
end

// -----------------------------------------------------------------------
// Accelerator model
//
// When start_new_block fires, a '1' enters the shift-register pipeline for
// the nominated accelerator.  After TASK_LATENCY cycles the '1' emerges
// and acc_finished is pulsed for one cycle.  acc_result is always 0
// (SUCCESS) so target_status entries will always read PASS.
// -----------------------------------------------------------------------
reg [TASK_LATENCY-1:0] acc0_pipe, acc1_pipe;

always @ (posedge clk) begin
    if (reset) begin
        acc0_pipe    <= {TASK_LATENCY{1'b0}};
        acc1_pipe    <= {TASK_LATENCY{1'b0}};
        acc_finished <= 2'b00;
        acc_result   <= 2'b00;
    end else begin
        // Shift pipelines: new task enters at bit 0, exits at MSB.
        acc0_pipe <= {acc0_pipe[TASK_LATENCY-2:0],
                      (start_new_block & (target_acc[0] == 1'b0))};
        acc1_pipe <= {acc1_pipe[TASK_LATENCY-2:0],
                      (start_new_block & (target_acc[0] == 1'b1))};

        // Fire finished when a task exits the pipeline.
        acc_finished[0] <= acc0_pipe[TASK_LATENCY-1];
        acc_finished[1] <= acc1_pipe[TASK_LATENCY-1];

        // Always report SUCCESS.
        acc_result <= 2'b00;
    end
end

// -----------------------------------------------------------------------
// DUT instantiation
// -----------------------------------------------------------------------
scheduler #(
    .SCH_ENTRY_SZ(SCH_ENTRY_SZ),
    .TGT_ACC_SZ(TGT_ACC_SZ),
    .CFG_ID_SZ(CFG_ID_SZ),
    .NUM_BUFFERS(NUM_BUFFERS),
    .COL_BUFF_ID_SZ(COL_BUFF_ID_SZ),
    .NUM_SCH_ENTRIES(NUM_SCH_ENTRIES),
    .NUM_HW_ACCELERATORS(NUM_HW_ACCELERATORS),
    .PROG_ADDR_BITS(PROG_ADDR_BITS),
    .PROG_DATA_BITS(PROG_DATA_BITS)
) scheduler0 (
    .clk(clk),
    .reset(reset),
    .test_stall_pipe(test_stall_pipe),

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

    .acc_busy_i(acc_busy),
    .acc_finished_i(acc_finished),
    .acc_result_i(acc_result),
    .acc_ready_next_i({NUM_HW_ACCELERATORS{1'b0}}),
    .start_new_block_o(start_new_block),
    .target_acc_o(target_acc),
    .buffer_info_o(buffer_info),

    .mark_buff_as_full_i(mark_buff_as_full),
    .full_buff_id_i(full_buff_id),
    .full_buff_usage_i(full_buff_usage)
);

endmodule
