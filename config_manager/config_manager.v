// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
// Authors: Simon Davidson & Claude | Created: 2026-05-08 | Last modified: 2026-08-19
`timescale 10ps/1ps
`include "../shared/constants.v"

// Reads config words and buffer base addresses from external RAMs and pushes
// them to the target accelerator's register bank, then signals completion.
// Both engines run in parallel: one streams config words, the other streams BBA
// entries (target first, then sources).  cm_config_finished_o is a single-
// cycle pulse when both are done.
//
// NOTES:
//   - WORDS_PER_CONFIG must be a power of 2 and >= 2 (LOG2_WPC used as width).
//   - NUM_ACC must be >= 2 (TGT_ACC_ID_SZ = $clog2(NUM_ACC) defaults to 1).
//   - ADDR_SZ must equal 32 for the AXI write-address arithmetic to be correct.
//   - $clog2 in parameter defaults is supported by Vivado/Quartus in Verilog mode.
//
// ─── Config-push latency (2026-08-19) ─────────────────────────────────────────
// The whole push is SERIAL in front of every compute task: the scheduler only
// dispatches when the target accelerator is free (sch_table.v gates on
// acc_busy_i and ~cm_busy_i), and the accelerator only starts on
// cm_config_finished_o (flexman.v: acc_start_new_block = cm_config_finished_o).
// So these cycles are paid once per annAcc/Hadamard task, not hidden under it.
//
// The original config engine spent 3 cycles per word (read / wait / write, no
// overlap) = 49 cycles from start_new_block_i to cm_config_finished_o at
// WORDS_PER_CONFIG=16.  Negligible against nblocks=4 deployment tasks (~28k cyc
// each) but ~1/3 of an nblocks=40 task, where it would dominate.  The config
// engine below overlaps the read of word w+1 with the accelerator write of
// word w:
//
//   CFG_MEM_SYNC=0 (default)  2 cyc/word  -> 34 cycles @ WPC=16
//   CFG_MEM_SYNC=1            1 cyc/word  -> 19 cycles @ WPC=16
//
// CFG_MEM_SYNC selects what the config-memory READ PORT is allowed to be, and
// that is the only reason both modes exist.  The original 3-state engine held
// cfg_mem_addr_o stable across two cycles, so it read correctly from EITHER a
// 0-cycle combinational memory OR a 1-cycle registered one.  That tolerance is
// relied on across the project (see a downstream deployment TB's SYNCMEM_FAB
// note, and the ~15 testbenches that model cfg_mem combinationally), so the
// DEFAULT mode preserves it exactly: the address is still held for two cycles
// and the capture still happens on the second, so any memory that worked before
// still works, unchanged.  Two cycles per word is the floor for a reader that
// does not know its memory's latency.
//
// CFG_MEM_SYNC=1 drops that tolerance in exchange for the last 15 cycles: it
// issues a new address every cycle and captures one cycle later, so it REQUIRES
// a 1-cycle synchronous read port.  Every real build already has one
// (shared/bram_sdp.v, a deployment SRAM model), but a combinational model will
// silently return the wrong word, so opt in per-top and make sure the
// testbench's cfg_mem model is registered.
//
// The BBA engine is deliberately left on the original 3-state structure.  Its
// 4 sends finish long before the 16 config words either way, so pipelining it
// buys nothing — and on silicon the BBA store is a combinational flop regfile
// (a deployment ASIC top), read 0-cycle by fill_unit, so its latency tolerance is
// load-bearing in a way the config memory's is not.

module config_manager #(
    // AXI write-slave address decode — fixed at build time, not run-time
    parameter [31:0] TGT_CONFIG_MEM_ADDR      = 32'hffff_ffff,
    parameter [31:0] TGT_CONFIG_MEM_ADDR_MASK  = 32'hffff_0000,
    parameter [31:0] TGT_BUFF_STORE_ADDR       = 32'hffff_ffff,
    parameter [31:0] TGT_BUFF_STORE_ADDR_MASK  = 32'hffff_0000,
    // Memory dimensions
    parameter        CONFIG_MEM_SZ             = 32,
    parameter        WORDS_PER_CONFIG          = 4,
    parameter        NUM_BUFFERS               = 32,
    parameter        BUFF_INDEX_SZ             = 5,
    // Accelerator interface sizing
    parameter        NUM_ACC                   = 2,
    parameter        TGT_ACC_ID_SZ             = $clog2(NUM_ACC),
    parameter        CFG_ID_SZ                 = 5,
    // Data / address widths
    parameter        DATAWORD_SZ               = `DATAWORD_SZ,
    parameter        ADDR_SZ                   = 32,
    parameter        SCH_ENTRY_SZ              = 32,
    // Number of source buffers; target is always sent first (index 0)
    parameter        NUM_SOURCES               = 3,
    // Config-memory read-port timing — see the header note.
    //   0 = latency-agnostic (address held 2 cycles): works with a 0-cycle
    //       combinational OR a 1-cycle registered cfg memory.  2 cyc/word.
    //   1 = assume a 1-cycle SYNCHRONOUS cfg memory.  1 cyc/word.
    parameter        CFG_MEM_SYNC              = 0
) (
    input  wire                        clk,
    input  wire                        reset,

    // System bus — AXI write slave (populates config and BBA tables)
    input  wire                        cm_sys_req_i,
    output wire                        cm_sys_ack_o,
    input  wire [ADDR_SZ-1:0]          cm_sys_addr_i,
    input  wire [DATAWORD_SZ-1:0]      cm_sys_data_i,

    // Flow control from scheduler
    input  wire                        start_new_block_i,
    input  wire [TGT_ACC_ID_SZ-1:0]    target_acc_i,
    input  wire [CFG_ID_SZ-1:0]        config_id_i,
    input  wire [SCH_ENTRY_SZ-1:0]     buffer_info_i,

    // One-cycle pulse when both FSMs have finished for this block
    output wire                        cm_config_finished_o,

    // High while either FSM is mid-push. Scheduler back-pressures dispatch
    // against this so a second start_new_block_i pulse can't be silently
    // dropped during an in-flight push.
    output wire                        cm_busy_o,

    // Config memory — read port
    output wire                        cfg_mem_rd_o,
    input  wire                        cfg_mem_wait_i,
    output wire [ADDR_SZ-1:0]          cfg_mem_addr_o,
    input  wire [DATAWORD_SZ-1:0]      cfg_mem_data_i,

    // Config memory — write port (AXI slave path)
    output wire                        cfg_mem_wr_o,
    output wire [ADDR_SZ-1:0]          cfg_mem_wr_addr_o,
    output wire [DATAWORD_SZ-1:0]      cfg_mem_wr_data_o,

    // Buffer base-address memory — read port
    output wire                        bba_mem_rd_o,
    input  wire                        bba_mem_wait_i,
    output wire [ADDR_SZ-1:0]          bba_mem_addr_o,
    input  wire [DATAWORD_SZ-1:0]      bba_mem_data_i,

    // Buffer base-address memory — write port (AXI slave path)
    output wire                        bba_mem_wr_o,
    output wire [ADDR_SZ-1:0]          bba_mem_wr_addr_o,
    output wire [DATAWORD_SZ-1:0]      bba_mem_wr_data_o,

    // Accelerator config-word interface
    output wire [TGT_ACC_ID_SZ-1:0]    cm_tgt_acc_o,
    output wire [NUM_ACC-1:0]          cm_config_wr_o,
    input  wire [NUM_ACC-1:0]          cm_config_wait_i,
    output wire [DATAWORD_SZ-1:0]      cm_config_data_o,

    // Accelerator buffer-base-address interface
    output wire [NUM_ACC-1:0]          cm_buff_base_wr_o,
    input  wire [NUM_ACC-1:0]          cm_buff_base_wait_i,
    output wire [DATAWORD_SZ-1:0]      cm_buff_base_data_o
);

// ─── Derived constants ────────────────────────────────────────────────────────
localparam LOG2_WPC      = $clog2(WORDS_PER_CONFIG);
localparam NUM_BBA_SENDS = NUM_SOURCES + 1;       // target + sources
localparam BBA_CNT_SZ    = $clog2(NUM_BBA_SENDS);

// Bit positions of buffer IDs packed into buffer_info_i (mirrors scheduler)
localparam SBUFF1_MSB = BUFF_INDEX_SZ - 1;
localparam SBUFF2_LSB = BUFF_INDEX_SZ;
localparam SBUFF2_MSB = 2*BUFF_INDEX_SZ - 1;
localparam SBUFF3_LSB = 2*BUFF_INDEX_SZ;
localparam SBUFF3_MSB = 3*BUFF_INDEX_SZ - 1;
localparam TBUFF_LSB  = 3*BUFF_INDEX_SZ;
localparam TBUFF_MSB  = 4*BUFF_INDEX_SZ - 1;

// FSM state encodings — BBA engine only (the config engine is now pipelined
// and counter-driven; see the header note)
localparam FSM_IDLE      = 2'd0;
localparam FSM_MEM_RD    = 2'd1;
localparam FSM_WAIT_DATA = 2'd2;
localparam FSM_ACC_WR    = 2'd3;

// ─── Latch scheduler inputs on start_new_block_i ──────────────────────────────
reg [TGT_ACC_ID_SZ-1:0]  tgt_acc_r;
reg [CFG_ID_SZ-1:0]      config_id_r;
reg [BUFF_INDEX_SZ-1:0]  src1_buff_r;
reg [BUFF_INDEX_SZ-1:0]  src2_buff_r;
reg [BUFF_INDEX_SZ-1:0]  src3_buff_r;
reg [BUFF_INDEX_SZ-1:0]  tgt_buff_r;

always @(posedge clk or posedge reset) begin
    if (reset) begin
        tgt_acc_r   <= {TGT_ACC_ID_SZ{1'b0}};
        config_id_r <= {CFG_ID_SZ{1'b0}};
        src1_buff_r <= {BUFF_INDEX_SZ{1'b0}};
        src2_buff_r <= {BUFF_INDEX_SZ{1'b0}};
        src3_buff_r <= {BUFF_INDEX_SZ{1'b0}};
        tgt_buff_r  <= {BUFF_INDEX_SZ{1'b0}};
    end else if (start_new_block_i) begin
        tgt_acc_r   <= target_acc_i;
        config_id_r <= config_id_i;
        src1_buff_r <= buffer_info_i[SBUFF1_MSB:0];
        src2_buff_r <= buffer_info_i[SBUFF2_MSB:SBUFF2_LSB];
        src3_buff_r <= buffer_info_i[SBUFF3_MSB:SBUFF3_LSB];
        tgt_buff_r  <= buffer_info_i[TBUFF_MSB:TBUFF_LSB];
    end
end

// ─── System bus address decode and write routing ──────────────────────────────
wire cfg_addr_match;
wire bba_addr_match;

assign cfg_addr_match = ((cm_sys_addr_i & TGT_CONFIG_MEM_ADDR_MASK) ==
                          (TGT_CONFIG_MEM_ADDR & TGT_CONFIG_MEM_ADDR_MASK));
assign bba_addr_match = ((cm_sys_addr_i & TGT_BUFF_STORE_ADDR_MASK) ==
                          (TGT_BUFF_STORE_ADDR & TGT_BUFF_STORE_ADDR_MASK));

assign cm_sys_ack_o      = cm_sys_req_i & (cfg_addr_match | bba_addr_match);

assign cfg_mem_wr_o      = cm_sys_req_i & cfg_addr_match;
assign cfg_mem_wr_addr_o = (cm_sys_addr_i & ~TGT_CONFIG_MEM_ADDR_MASK) >> 2;
assign cfg_mem_wr_data_o = cm_sys_data_i;

assign bba_mem_wr_o      = cm_sys_req_i & bba_addr_match;
assign bba_mem_wr_addr_o = (cm_sys_addr_i & ~TGT_BUFF_STORE_ADDR_MASK) >> 2;
assign bba_mem_wr_data_o = cm_sys_data_i;

// ─── Config parameters engine ─────────────────────────────────────────────────
// Streams WORDS_PER_CONFIG words from config_mem starting at address
// (config_id * WORDS_PER_CONFIG) to the target accelerator's config registers.
//
// Decoupled read side and write side, joined by a 2-entry holding buffer, so
// the read of word w+1 overlaps the accelerator write of word w.  The holding
// buffer is what lets cm_config_wait_i still be honoured properly: when the
// accelerator stalls, the word already in flight from the memory has somewhere
// to land instead of being dropped, and the read side stops issuing.  (Every
// real build ties that wait to 1'b0, but the protocol is still a protocol.)

reg                   cfg_run_r;        // streaming in progress
reg [LOG2_WPC:0]      cfg_rd_cnt_r;     // next word to READ  (0..WORDS_PER_CONFIG)
reg [LOG2_WPC:0]      cfg_wr_cnt_r;     // next word to WRITE (0..WORDS_PER_CONFIG)
reg                   cfg_hold_r;       // SYNC=0: 1 = second (address-hold) cycle
reg                   cfg_inflight_r;   // SYNC=1: a read was accepted last cycle
reg                   cfg_done_r;

// 2-entry holding buffer.  d0 is the word presented to the accelerator.
reg                   cfg_d0_v_r, cfg_d1_v_r;
reg [DATAWORD_SZ-1:0] cfg_d0_r,   cfg_d1_r;

// A read is outstanding when the memory has been addressed but the word has
// not yet reached the holding buffer.  SYNC=0 counts the address-hold cycle;
// SYNC=1 counts the one-cycle memory latency.
wire cfg_outstanding = CFG_MEM_SYNC ? cfg_inflight_r : cfg_hold_r;

// Write side: present d0 whenever we have one; the accelerator's wait decides
// whether it is taken this cycle (matching the original ACC_WR behaviour,
// which also held the strobe high across a stall).
wire cfg_wr_w  = cfg_d0_v_r;
wire cfg_deq   = cfg_d0_v_r & ~cm_config_wait_i[tgt_acc_r];

// Read side: issue only while the holding buffer provably has room for the
// word that would come back.  Occupancy counts both buffered words and the
// outstanding read; subtracting cfg_deq is what keeps the SYNC=1 pipeline at a
// full word per cycle in the (universal) no-stall case.
wire [1:0] cfg_occ = {1'b0, cfg_d0_v_r} + {1'b0, cfg_d1_v_r} + {1'b0, cfg_outstanding};
wire       cfg_room = (cfg_occ - {1'b0, cfg_deq}) < 2'd2;

wire cfg_issue = cfg_run_r & (cfg_rd_cnt_r != WORDS_PER_CONFIG) & cfg_room
                 & (CFG_MEM_SYNC ? 1'b1 : ~cfg_hold_r);
wire cfg_accept = cfg_issue & ~cfg_mem_wait_i;

// Word arrives in the holding buffer: one cycle after acceptance (SYNC=1), or
// on the address-hold cycle (SYNC=0, where the address is still being driven
// so a combinational memory reads correctly too).
wire cfg_enq = CFG_MEM_SYNC ? cfg_inflight_r : cfg_hold_r;

always @(posedge clk or posedge reset) begin
    if (reset) begin
        cfg_run_r      <= 1'b0;
        cfg_rd_cnt_r   <= {(LOG2_WPC+1){1'b0}};
        cfg_wr_cnt_r   <= {(LOG2_WPC+1){1'b0}};
        cfg_hold_r     <= 1'b0;
        cfg_inflight_r <= 1'b0;
        cfg_done_r     <= 1'b0;
        cfg_d0_v_r     <= 1'b0;
        cfg_d1_v_r     <= 1'b0;
        cfg_d0_r       <= {DATAWORD_SZ{1'b0}};
        cfg_d1_r       <= {DATAWORD_SZ{1'b0}};
    end else if (start_new_block_i && !cfg_run_r) begin
        cfg_run_r      <= 1'b1;
        cfg_rd_cnt_r   <= {(LOG2_WPC+1){1'b0}};
        cfg_wr_cnt_r   <= {(LOG2_WPC+1){1'b0}};
        cfg_hold_r     <= 1'b0;
        cfg_inflight_r <= 1'b0;
        cfg_done_r     <= 1'b0;
        cfg_d0_v_r     <= 1'b0;
        cfg_d1_v_r     <= 1'b0;
    end else begin
        // ── read side ────────────────────────────────────────────────────────
        cfg_inflight_r <= CFG_MEM_SYNC ? cfg_accept : 1'b0;   // SYNC=1 only

        if (!CFG_MEM_SYNC) begin
            if (cfg_hold_r) begin              // address-hold cycle completes
                cfg_hold_r   <= 1'b0;
                cfg_rd_cnt_r <= cfg_rd_cnt_r + 1'b1;
            end else if (cfg_accept) begin
                cfg_hold_r   <= 1'b1;
            end
        end else if (cfg_accept) begin
            cfg_rd_cnt_r <= cfg_rd_cnt_r + 1'b1;
        end

        // ── holding buffer ───────────────────────────────────────────────────
        case ({cfg_enq, cfg_deq})
            2'b01: begin                       // taken, nothing arriving
                cfg_d0_v_r <= cfg_d1_v_r;
                cfg_d0_r   <= cfg_d1_r;
                cfg_d1_v_r <= 1'b0;
            end
            2'b10: begin                       // arriving, nothing taken
                if (!cfg_d0_v_r) begin
                    cfg_d0_v_r <= 1'b1;
                    cfg_d0_r   <= cfg_mem_data_i;
                end else begin
                    cfg_d1_v_r <= 1'b1;
                    cfg_d1_r   <= cfg_mem_data_i;
                end
            end
            2'b11: begin                       // both: shift through
                if (cfg_d1_v_r) begin
                    cfg_d0_r <= cfg_d1_r;
                    cfg_d1_r <= cfg_mem_data_i;
                end else begin
                    cfg_d0_r <= cfg_mem_data_i;
                end
            end
            default: ;                         // 2'b00: idle
        endcase

        // ── write side ───────────────────────────────────────────────────────
        if (cfg_deq) begin
            cfg_wr_cnt_r <= cfg_wr_cnt_r + 1'b1;
            if (cfg_wr_cnt_r == WORDS_PER_CONFIG - 1) begin
                cfg_done_r <= 1'b1;
                cfg_run_r  <= 1'b0;
            end
        end
    end
end

// Read address = config_id concatenated with word counter (= config_id*WPC + cnt).
// cfg_rd_cnt_r only reaches WORDS_PER_CONFIG once issuing has stopped, so the
// truncation to LOG2_WPC bits never affects a driven address.
assign cfg_mem_rd_o   = cfg_issue;
assign cfg_mem_addr_o = {{(ADDR_SZ-CFG_ID_SZ-LOG2_WPC){1'b0}}, config_id_r,
                         cfg_rd_cnt_r[LOG2_WPC-1:0]};

// ─── BBA FSM ──────────────────────────────────────────────────────────────────
// Streams NUM_BBA_SENDS base addresses from bba_mem to the target accelerator:
// index 0 = target buffer, 1..NUM_SOURCES = source buffers.

reg [1:0]             bba_state_r;
reg [BBA_CNT_SZ-1:0]  bba_send_cnt_r;
reg [DATAWORD_SZ-1:0] bba_data_r;
reg                   bba_done_r;

// Mux buffer ID from send counter — drives bba_mem read address
reg [BUFF_INDEX_SZ-1:0] bba_buff_id_w;
always @(*) begin
    case (bba_send_cnt_r)
        2'd0:    bba_buff_id_w = tgt_buff_r;
        2'd1:    bba_buff_id_w = src1_buff_r;
        2'd2:    bba_buff_id_w = src2_buff_r;
        default: bba_buff_id_w = src3_buff_r;
    endcase
end

always @(posedge clk or posedge reset) begin
    if (reset) begin
        bba_state_r    <= FSM_IDLE;
        bba_send_cnt_r <= {BBA_CNT_SZ{1'b0}};
        bba_data_r     <= {DATAWORD_SZ{1'b0}};
        bba_done_r     <= 1'b0;
    end else begin
        case (bba_state_r)
            FSM_IDLE: begin
                if (start_new_block_i) begin
                    bba_send_cnt_r <= {BBA_CNT_SZ{1'b0}};
                    bba_done_r     <= 1'b0;
                    bba_state_r    <= FSM_MEM_RD;
                end
            end
            FSM_MEM_RD: begin
                if (!bba_mem_wait_i)
                    bba_state_r <= FSM_WAIT_DATA;
            end
            FSM_WAIT_DATA: begin
                bba_data_r  <= bba_mem_data_i;
                bba_state_r <= FSM_ACC_WR;
            end
            FSM_ACC_WR: begin
                if (!cm_buff_base_wait_i[tgt_acc_r]) begin
                    if (bba_send_cnt_r == NUM_BBA_SENDS - 1) begin
                        bba_done_r  <= 1'b1;
                        bba_state_r <= FSM_IDLE;
                    end else begin
                        bba_send_cnt_r <= bba_send_cnt_r + 1'b1;
                        bba_state_r    <= FSM_MEM_RD;
                    end
                end
            end
            default: bba_state_r <= FSM_IDLE;
        endcase
    end
end

// Read address = buffer ID used as direct index into bba_mem
assign bba_mem_rd_o  = (bba_state_r == FSM_MEM_RD);
assign bba_mem_addr_o = {{(ADDR_SZ-BUFF_INDEX_SZ){1'b0}}, bba_buff_id_w};

// ─── Completion pulse — one cycle on rising edge of (cfg_done & bba_done) ─────
wire both_done;
reg  both_done_prev_r;

assign both_done = cfg_done_r & bba_done_r;

always @(posedge clk or posedge reset) begin
    if (reset)
        both_done_prev_r <= 1'b0;
    else
        both_done_prev_r <= both_done;
end

assign cm_config_finished_o = both_done & ~both_done_prev_r;

// Busy while either engine is mid-push. Used by the scheduler as
// back-pressure on `dispatch_to_acc_o` — a second start_new_block_i
// pulse must not arrive while we're still pushing the previous one.
// cfg_run_r clears on the last accelerator write, so busy falls on the same
// cycle cm_config_finished_o pulses, exactly as the original FSM did.
assign cm_busy_o = cfg_run_r | (bba_state_r != FSM_IDLE);

// ─── Static output assignments ────────────────────────────────────────────────
assign cm_tgt_acc_o        = tgt_acc_r;
assign cm_config_data_o    = cfg_d0_r;
assign cm_buff_base_data_o = bba_data_r;

// One-hot write strobes: bit [tgt_acc_r] asserted while a word is presented
assign cm_config_wr_o    = cfg_wr_w
                           ? ({{(NUM_ACC-1){1'b0}}, 1'b1} << tgt_acc_r)
                           : {NUM_ACC{1'b0}};

assign cm_buff_base_wr_o = (bba_state_r == FSM_ACC_WR)
                           ? ({{(NUM_ACC-1){1'b0}}, 1'b1} << tgt_acc_r)
                           : {NUM_ACC{1'b0}};

endmodule
