// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
// =============================================================================
// tb_sch_wide -- wide (three-word) TASK decode
//
// Authors      : Simon Davidson & Claude
// Created      : 2026-08-20
// Last modified: 2026-08-20
//
// The narrow TASK packs three long slots x [mode(2)+id(4)+ntgt(4)] into word 2's
// 30 usable bits, capping usage counts at 15. Monarch at nblocks=40 needs 85.
// A wide TASK is selected by TASK word 1 bit 31 -- previously reserved and always
// zero -- and spends three words, keeping all three long slots at 7-bit ntgt.
//
// The central property, and what this bench exists to prove:
//
//   W1  DECODE EQUIVALENCE. A WIDE_NTGT=1 build still decodes narrow TASKs, so
//       both forms can appear in ONE instruction stream. The same logical task,
//       encoded each way, must produce a byte-identical packed scheduler entry.
//       This is the unit-level twin of the nblocks=4 full-chip equivalence run
//       (nb=4 needs max ntgt 13, so it fits both forms).
//
//   W2  WIDE RANGE. ntgt values the narrow form cannot express (16..127) must
//       survive decode intact.
//
//   W3  PC SEQUENCING. A wide TASK consumes exactly three program words: the
//       instruction after it must be fetched from the right address, and its
//       word 2 / word 3 must never be decoded as instructions in their own
//       right. (That last hazard is real -- see the "qualify the decode AT THE
//       SOURCE" note in scheduler.v; a word-2 constant whose low bits looked
//       like an opcode used to trigger spurious jumps.)
// =============================================================================
`timescale 1ns/1ps

module tb_sch_wide;

localparam TGT_ACC_SZ         = 3;
localparam TGT_COUNT_SZ       = 7;    // wide build
localparam CFG_ID_SZ          = 9;   // wide build: 512 configs (nb=40 needs 329)
localparam NUM_BUFFERS        = 16;
localparam BUFF_INDX_SZ       = 4;
localparam COL_BUFF_ID_SZ     = 16;
localparam NUM_SCH_ENTRIES    = 4;
localparam NUM_HW_ACCELERATORS= 5;
localparam PROG_ADDR_BITS     = 10;
localparam PROG_DATA_BITS     = 32;
localparam MODE_SZ            = 2;
localparam SLOT_LONG_SZ       = MODE_SZ + BUFF_INDX_SZ + TGT_COUNT_SZ;  // 13

localparam MODE_UNUSED = 2'b00;
localparam MODE_SRC    = 2'b01;
localparam MODE_RW     = 2'b10;
localparam MODE_TGT    = 2'b11;

integer errors = 0;
integer checks = 0;

task check_eq;
    input [63:0] got, exp;
    input [8*40-1:0] tag;
    begin
        checks = checks + 1;
        if (got !== exp) begin
            $display("FAIL %0s: got %h expected %h", tag, got, exp);
            errors = errors + 1;
        end
    end
endtask

// ---- instruction encoders (mirror tools/flexman_backend/isa.py) ------------
function [31:0] tw1;                       // narrow TASK word 1
    input [1:0] acc; input [6:0] cfg; input colour;
    input [1:0] m0; input [3:0] id0;
    input [1:0] m1; input [3:0] id1;
    input [1:0] m2; input [3:0] id2;
    begin tw1 = {1'b0, id2, m2, id1, m1, id0, m0, colour, cfg, acc, 3'b000}; end
endfunction

// Wide word 1: selector set, and cfg_id field ZERO — a wide TASK carries cfg_id
// in word 3 instead (word 1 is full, and contiguous beats split).
function [31:0] tw1_wide;
    input [1:0] acc; input [6:0] cfg; input colour;
    input [1:0] m0; input [3:0] id0;
    input [1:0] m1; input [3:0] id1;
    input [1:0] m2; input [3:0] id2;
    begin tw1_wide = {1'b1, id2, m2, id1, m1, id0, m0, colour, 7'd0, acc, 3'b000}; end
endfunction

function [31:0] tw2;                       // narrow word 2: 4-bit ntgt
    input [1:0] m3; input [3:0] id3; input [3:0] n3;
    input [1:0] m4; input [3:0] id4; input [3:0] n4;
    input [1:0] m5; input [3:0] id5; input [3:0] n5;
    begin tw2 = {n5, id5, m5, n4, id4, m4, n3, id3, m3, 2'b00}; end
endfunction

function [SLOT_LONG_SZ-1:0] slot_wide;     // {ntgt, id, mode} lsb-first
    input [1:0] m; input [3:0] id; input [6:0] n;
    begin slot_wide = {n, id, m}; end
endfunction

function [31:0] tw2_wide;                  // wide word 2: sentinel + slots 3,4
    input [1:0] m3; input [3:0] id3; input [6:0] n3;
    input [1:0] m4; input [3:0] id4; input [6:0] n4;
    begin tw2_wide = {4'b0000, slot_wide(m4,id4,n4), slot_wide(m3,id3,n3), 2'b00}; end
endfunction

function [31:0] tw3_wide;   // wide word 3: slot 5, then cfg_id, rest spare
    input [1:0] m5; input [3:0] id5; input [6:0] n5; input [CFG_ID_SZ-1:0] cfg;
    begin tw3_wide = {{(32-SLOT_LONG_SZ-CFG_ID_SZ){1'b0}}, cfg,
                      slot_wide(m5,id5,n5)}; end
endfunction

localparam [31:0] STOP_INST = 32'h00000002;

// ---- clock / reset --------------------------------------------------------
reg clk = 0; always #5 clk = ~clk;
reg reset;

// ---- program memory -------------------------------------------------------
reg [31:0] prog_mem [0:1023];
wire [PROG_ADDR_BITS-1:0] prog_mem_addr;
wire                      prog_mem_req;
// Combinational, matching tb_scheduler.v:219 — the scheduler's fetch expects the
// word to be present in the same cycle it drives the address.
wire [31:0]               prog_mem_data = prog_mem[prog_mem_addr];

// ---- DUT ------------------------------------------------------------------
reg        sys_req  = 0;
reg [31:0] sys_addr = 0;
reg [31:0] sys_data = 0;

wire                      start_new_block;
wire [TGT_ACC_SZ-1:0]     target_acc;
wire [63:0]               buffer_info;

scheduler #(
    .TGT_ACC_SZ(TGT_ACC_SZ), .TGT_COUNT_SZ(TGT_COUNT_SZ),
    .WIDE_NTGT(1),
    .CFG_ID_SZ(CFG_ID_SZ), .NUM_BUFFERS(NUM_BUFFERS),
    .COL_BUFF_ID_SZ(COL_BUFF_ID_SZ), .NUM_SCH_ENTRIES(NUM_SCH_ENTRIES),
    .NUM_HW_ACCELERATORS(NUM_HW_ACCELERATORS),
    .PROG_ADDR_BITS(PROG_ADDR_BITS), .PROG_DATA_BITS(PROG_DATA_BITS),
    .BUFF_INDX_SZ(BUFF_INDX_SZ)
) dut (
    .clk(clk), .reset(reset), .test_stall_pipe(1'b0),
    .sys_req_i(sys_req), .sys_ack_o(), .sys_addr_i(sys_addr), .sys_data_i(sys_data),
    .sys_data_o(),
    .prog_mem_addr_o(prog_mem_addr), .prog_mem_data_i(prog_mem_data),
    .prog_mem_req_o(prog_mem_req), .prog_mem_wait_i(1'b0),
    .prog_mem_wr_o(), .prog_mem_wr_addr_o(), .prog_mem_wr_data_o(),
    .prog_mem_wr_wait_i(1'b0),
    .acc_busy_i({NUM_HW_ACCELERATORS{1'b0}}),
    .acc_finished_i({NUM_HW_ACCELERATORS{1'b0}}),
    .acc_result_i(1'b0),
    .acc_ready_next_i(1'b0),
    .start_new_block_o(start_new_block), .target_acc_o(target_acc),
    .buffer_info_o(buffer_info),
    .nxt_input_pulse_o(), .nxt_output_pulse_o(),
    .fill_value_o(), .fill_block_size_o(), .cm_busy_i(1'b0)
);

task axi_write;
    input [31:0] addr, data;
    begin
        @(posedge clk); #1;
        sys_addr = addr; sys_data = data; sys_req = 1'b1;
        @(posedge clk); #1;
        sys_req = 1'b0;
    end
endtask

// ---- capture every packed entry as it is loaded ---------------------------
// dut.new_entry_data is the decoder's whole output: comparing it between the two
// encodings is a direct test of the decode paths, independent of downstream
// scheduling behaviour.
localparam ENTRY_SZ    = 3*(MODE_SZ+BUFF_INDX_SZ) + 3*SLOT_LONG_SZ + 1 + TGT_ACC_SZ + CFG_ID_SZ;
localparam E_CFG_START = 3*(MODE_SZ+BUFF_INDX_SZ) + 3*SLOT_LONG_SZ + 1 + TGT_ACC_SZ;
reg [ENTRY_SZ-1:0] entry_cap [0:7];
integer            entry_n = 0;
always @(posedge clk) if (!reset && dut.load_new_entry) begin
    entry_cap[entry_n] = dut.new_entry_data;
    entry_n = entry_n + 1;
end

// ---- debug trace ----------------------------------------------------------
`ifdef WDBG
always @(posedge clk) if (!reset)
    if (dut.inst_valid | dut.task_w2_pending_r | dut.task_w3_pending_r)
        $display("[%0t] pc=%0d word=%h valid=%b vfd=%b task=%b w2p=%b w3p=%b wide=%b cons=%b c2=%b c3=%b load=%b slotfree=%b",
                 $time, dut.prog_counter_r, dut.inst_word, dut.inst_valid,
                 dut.inst_valid_for_decode, dut.inst_is_task,
                 dut.task_w2_pending_r, dut.task_w3_pending_r, dut.pending_is_wide_r,
                 dut.inst_consumed, dut.inst_consumed_w2, dut.inst_consumed_w3,
                 dut.load_new_entry, dut.table_slot_free);
`endif

// ---- self-check on the encoders (mirrors tb_scheduler's style) ------------
initial begin
    if (tw2_wide(MODE_TGT,4'd5,7'd85, MODE_RW,4'd6,7'd40) >> 2 !== {slot_wide(MODE_RW,4'd6,7'd40), slot_wide(MODE_TGT,4'd5,7'd85)})
        $display("[WIDE-SELFCHECK] FAIL tw2_wide packing");
    else
        $display("[WIDE-SELFCHECK] PASS (slots contiguous at the SLOT_LONG_SZ stride)");
end

integer i;
initial begin
    for (i = 0; i < 1024; i = i + 1) prog_mem[i] = STOP_INST;

    // --- the SAME logical task, encoded both ways ---------------------------
    // acc 0, cfg 10, colour 0; shorts: SRC b1, SRC b2, UNUSED
    // longs: TGT b3 ntgt 9, RW b4 ntgt 13, SRC b5 ntgt 2   (all fit 4 bits)
    prog_mem[0] = tw1(2'd0, 7'd10, 1'b0, MODE_SRC,4'd1, MODE_SRC,4'd2, MODE_UNUSED,4'd0);
    prog_mem[1] = tw2(MODE_TGT,4'd3,4'd9, MODE_RW,4'd4,4'd13, MODE_SRC,4'd5,4'd2);

    prog_mem[2] = tw1_wide(2'd0, 7'd10, 1'b0, MODE_SRC,4'd1, MODE_SRC,4'd2, MODE_UNUSED,4'd0);
    prog_mem[3] = tw2_wide(MODE_TGT,4'd3,7'd9, MODE_RW,4'd4,7'd13);
    prog_mem[4] = tw3_wide(MODE_SRC,4'd5,7'd2, 7'd10);   // same cfg as the narrow twin

    // --- a task only the wide form can express ------------------------------
    prog_mem[5] = tw1_wide(2'd1, 7'd11, 1'b1, MODE_SRC,4'd6, MODE_UNUSED,4'd0, MODE_UNUSED,4'd0);
    prog_mem[6] = tw2_wide(MODE_TGT,4'd7,7'd85, MODE_RW,4'd8,7'd127);
    prog_mem[7] = tw3_wide(MODE_SRC,4'd9,7'd64, 7'd11);

    // --- a cfg_id only the wide form can express (329 configs at nblocks=40) ---
    prog_mem[8]  = tw1_wide(2'd0, 7'd0, 1'b0, MODE_SRC,4'd1, MODE_UNUSED,4'd0, MODE_UNUSED,4'd0);
    prog_mem[9]  = tw2_wide(MODE_TGT,4'd2,7'd3, MODE_UNUSED,4'd0,7'd0);
    prog_mem[10] = tw3_wide(MODE_UNUSED,4'd0,7'd0, 9'd329);

    prog_mem[11] = STOP_INST;

    reset = 1; sys_req = 0;
    repeat (4) @(posedge clk); #1;
    reset = 0;
    @(posedge clk); #1;

    axi_write(32'hE000_0000, 32'd0);   // LOAD_PC: start at word 0
    axi_write(32'hE010_0000, 32'b0);   // START

    repeat (200) @(posedge clk);

    // ---- W1: decode equivalence ------------------------------------------
    if (entry_n < 2) begin
        $display("FAIL W1: only %0d entries loaded (expected >= 3)", entry_n);
        errors = errors + 1;
    end else begin
        check_eq(entry_cap[0], entry_cap[1],
                 "W1 narrow vs wide encode identically");
        $display("[W1] narrow entry = %h", entry_cap[0]);
        $display("[W1] wide   entry = %h", entry_cap[1]);
    end

    // ---- W2: values the narrow form cannot hold ---------------------------
    if (entry_n < 3) begin
        $display("FAIL W2: wide-range task never loaded"); errors = errors + 1;
    end else begin
        check_eq((entry_cap[2] >> (3*(MODE_SZ+BUFF_INDX_SZ) + 0*SLOT_LONG_SZ
                                   + MODE_SZ + BUFF_INDX_SZ)) & 7'h7F, 7'd85,
                 "W2 slot3 ntgt=85 survives");
        check_eq((entry_cap[2] >> (3*(MODE_SZ+BUFF_INDX_SZ) + 1*SLOT_LONG_SZ
                                   + MODE_SZ + BUFF_INDX_SZ)) & 7'h7F, 7'd127,
                 "W2 slot4 ntgt=127 survives");
        check_eq((entry_cap[2] >> (3*(MODE_SZ+BUFF_INDX_SZ) + 2*SLOT_LONG_SZ
                                   + MODE_SZ + BUFF_INDX_SZ)) & 7'h7F, 7'd64,
                 "W2 slot5 ntgt=64 survives (from word 3)");
    end

    // ---- W3: cfg_id beyond the narrow 7-bit field -------------------------
    if (entry_n < 4) begin
        $display("FAIL W3: wide-cfg task never loaded"); errors = errors + 1;
    end else begin
        check_eq((entry_cap[3] >> E_CFG_START) & ((1<<CFG_ID_SZ)-1), 329,
                 "W3 cfg_id=329 survives (word 3)");
    end
    // cfg_id of the equivalence pair must also match (10, from opposite words)
    check_eq((entry_cap[0] >> E_CFG_START) & ((1<<CFG_ID_SZ)-1), 10, "W1 narrow cfg_id");
    check_eq((entry_cap[1] >> E_CFG_START) & ((1<<CFG_ID_SZ)-1), 10, "W1 wide cfg_id");

    // ---- W4: exactly four entries, no phantom instructions ----------------
    check_eq(entry_n, 4, "W4 exactly 4 tasks decoded (no phantom/lost words)");

    if (errors == 0) $display("=== tb_sch_wide: %0d check(s), 0 failure(s) ===\nPASS", checks);
    else             $display("=== tb_sch_wide: %0d check(s), %0d FAILURE(S) ===\nFAIL", checks, errors);
    $finish;
end

initial begin #500000; $display("FAIL: timeout"); $finish; end

endmodule
