// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
// Authors: Simon Davidson & Claude | Created: 2026-05-11 | Last modified: 2026-08-19
`timescale 10ps/1ps
`include "../shared/constants.v"

// ================================================================
// tb_config_manager
//
// Verifies config_manager.v: AXI-slave memory population, parallel
// config-params and BBA FSMs, one-hot accelerator strobes, and the
// cm_config_finished_o completion pulse.
//
// Tests
// -----
//   T1  Basic no-wait: config_id=0 → acc 0, bufs tgt=5 s1=3 s2=7 s3=1
//   T2  Different config entry: config_id=1 → acc 0, different bufs
//   T3  Target accelerator 1: same data as T1 but directed to acc 1,
//       acc 0 must receive nothing
//   T4  Back-to-back blocks: two consecutive blocks without gap
//   T5  Memory back-pressure: wait_i pre-asserted on first read
//   T6  Accelerator back-pressure: cm_config_wait_i held on first write
//
// Build modes (all three are standing regressions — see the .bsh files):
//   (none)                     WPC=4,  CFG_MEM_SYNC=0, COMBINATIONAL cfg mem
//   -define WPC16              WPC=16, CFG_MEM_SYNC=0, COMBINATIONAL cfg mem
//   -define CFG_SYNC -define WPC16
//                              WPC=16, CFG_MEM_SYNC=1, REGISTERED cfg mem
// T1 additionally asserts the config-push latency (start_new_block_i ->
// cm_config_finished_o), which is the dead time in front of every compute
// task. At WPC=16 that is 34 cycles in the default mode and 19 with
// CFG_MEM_SYNC=1, versus 49 before the config engine was pipelined. At the
// default WPC=4 the untouched 4-send BBA engine sets the latency (13) in
// every mode, which is why WPC16 exists.
//
// Test data encoding
//   cfg word n of config C  →  32'hC0nn_0000 | (C<<8) | n
//   BBA entry for buffer B  →  32'hB000_0000 | B
// ================================================================

module tb_config_manager;

// ─── TB constants ─────────────────────────────────────────────────────────────
localparam CONFIG_MEM_SZ    = 32;
// -define WPC16 builds at the real deployed width (flexman.v / a deployment top
// both instantiate WORDS_PER_CONFIG=16); the default 4 keeps the original
// vectors. The config-push latency check below only bites at 16, where the
// config engine — not the untouched 4-send BBA engine — sets the latency.
`ifdef WPC16
localparam WORDS_PER_CONFIG = 16;
`else
localparam WORDS_PER_CONFIG = 4;
`endif
localparam NUM_BUFFERS      = 32;
localparam BUFF_INDEX_SZ    = 5;
localparam NUM_ACC          = 2;
localparam TGT_ACC_ID_SZ    = 1;   // $clog2(2)
localparam CFG_ID_SZ        = 5;
localparam DATAWORD_SZ      = `DATAWORD_SZ;
localparam ADDR_SZ          = 32;
localparam SCH_ENTRY_SZ     = 32;
localparam NUM_SOURCES      = 3;
localparam NUM_BBA_SENDS    = 4;   // NUM_SOURCES + 1

localparam [31:0] CFG_BASE = 32'h1000_0000;
localparam [31:0] CFG_MASK = 32'hFF00_0000;
localparam [31:0] BBA_BASE = 32'h2000_0000;
localparam [31:0] BBA_MASK = 32'hFF00_0000;

localparam CLK_HALF = 5;    // 5 × 10 ps = 50 ps half-period → 100 ps clock
localparam TIMEOUT  = 500;  // cycles before declaring a hang

// ─── DUT signal declarations ─────────────────────────────────────────────────
reg  clk, reset;

reg                       cm_sys_req_i;
wire                      cm_sys_ack_o;
reg  [ADDR_SZ-1:0]        cm_sys_addr_i;
reg  [DATAWORD_SZ-1:0]    cm_sys_data_i;

reg                       start_new_block_i;
reg  [TGT_ACC_ID_SZ-1:0]  target_acc_i;
reg  [CFG_ID_SZ-1:0]      config_id_i;
reg  [SCH_ENTRY_SZ-1:0]   buffer_info_i;

wire                      cm_config_finished_o;

wire                      cfg_mem_rd_o;
reg                       cfg_mem_wait_i;
wire [ADDR_SZ-1:0]        cfg_mem_addr_o;
reg  [DATAWORD_SZ-1:0]    cfg_mem_data_i;
wire                      cfg_mem_wr_o;
wire [ADDR_SZ-1:0]        cfg_mem_wr_addr_o;
wire [DATAWORD_SZ-1:0]    cfg_mem_wr_data_o;

wire                      bba_mem_rd_o;
reg                       bba_mem_wait_i;
wire [ADDR_SZ-1:0]        bba_mem_addr_o;
reg  [DATAWORD_SZ-1:0]    bba_mem_data_i;
wire                      bba_mem_wr_o;
wire [ADDR_SZ-1:0]        bba_mem_wr_addr_o;
wire [DATAWORD_SZ-1:0]    bba_mem_wr_data_o;

wire [TGT_ACC_ID_SZ-1:0]  cm_tgt_acc_o;
wire [NUM_ACC-1:0]        cm_config_wr_o;
reg  [NUM_ACC-1:0]        cm_config_wait_i;
wire [DATAWORD_SZ-1:0]    cm_config_data_o;
wire [NUM_ACC-1:0]        cm_buff_base_wr_o;
reg  [NUM_ACC-1:0]        cm_buff_base_wait_i;
wire [DATAWORD_SZ-1:0]    cm_buff_base_data_o;

// ─── DUT ─────────────────────────────────────────────────────────────────────
config_manager #(
    .TGT_CONFIG_MEM_ADDR      (CFG_BASE),
    .TGT_CONFIG_MEM_ADDR_MASK (CFG_MASK),
    .TGT_BUFF_STORE_ADDR      (BBA_BASE),
    .TGT_BUFF_STORE_ADDR_MASK (BBA_MASK),
    .CONFIG_MEM_SZ            (CONFIG_MEM_SZ),
    .WORDS_PER_CONFIG         (WORDS_PER_CONFIG),
    .NUM_BUFFERS              (NUM_BUFFERS),
    .BUFF_INDEX_SZ            (BUFF_INDEX_SZ),
    .NUM_ACC                  (NUM_ACC),
    .TGT_ACC_ID_SZ            (TGT_ACC_ID_SZ),
    .CFG_ID_SZ                (CFG_ID_SZ),
    .DATAWORD_SZ              (DATAWORD_SZ),
    .ADDR_SZ                  (ADDR_SZ),
    .SCH_ENTRY_SZ             (SCH_ENTRY_SZ),
    .NUM_SOURCES              (NUM_SOURCES),
`ifdef CFG_SYNC
    .CFG_MEM_SYNC             (1)
`else
    .CFG_MEM_SYNC             (0)
`endif
) dut (
    .clk                  (clk),
    .reset                (reset),
    .cm_sys_req_i         (cm_sys_req_i),
    .cm_sys_ack_o         (cm_sys_ack_o),
    .cm_sys_addr_i        (cm_sys_addr_i),
    .cm_sys_data_i        (cm_sys_data_i),
    .start_new_block_i    (start_new_block_i),
    .target_acc_i         (target_acc_i),
    .config_id_i          (config_id_i),
    .buffer_info_i        (buffer_info_i),
    .cm_config_finished_o (cm_config_finished_o),
    .cfg_mem_rd_o         (cfg_mem_rd_o),
    .cfg_mem_wait_i       (cfg_mem_wait_i),
    .cfg_mem_addr_o       (cfg_mem_addr_o),
    .cfg_mem_data_i       (cfg_mem_data_i),
    .cfg_mem_wr_o         (cfg_mem_wr_o),
    .cfg_mem_wr_addr_o    (cfg_mem_wr_addr_o),
    .cfg_mem_wr_data_o    (cfg_mem_wr_data_o),
    .bba_mem_rd_o         (bba_mem_rd_o),
    .bba_mem_wait_i       (bba_mem_wait_i),
    .bba_mem_addr_o       (bba_mem_addr_o),
    .bba_mem_data_i       (bba_mem_data_i),
    .bba_mem_wr_o         (bba_mem_wr_o),
    .bba_mem_wr_addr_o    (bba_mem_wr_addr_o),
    .bba_mem_wr_data_o    (bba_mem_wr_data_o),
    .cm_tgt_acc_o         (cm_tgt_acc_o),
    .cm_config_wr_o       (cm_config_wr_o),
    .cm_config_wait_i     (cm_config_wait_i),
    .cm_config_data_o     (cm_config_data_o),
    .cm_buff_base_wr_o    (cm_buff_base_wr_o),
    .cm_buff_base_wait_i  (cm_buff_base_wait_i),
    .cm_buff_base_data_o  (cm_buff_base_data_o)
);

// ─── Clock ────────────────────────────────────────────────────────────────────
initial clk = 1'b0;
always  #CLK_HALF clk = ~clk;

// ─── Memory arrays (backing store behind DUT's AXI write ports) ───────────────
reg [DATAWORD_SZ-1:0] cfg_mem_arr [0:CONFIG_MEM_SZ-1];
reg [DATAWORD_SZ-1:0] bba_mem_arr [0:NUM_BUFFERS-1];

always @(posedge clk) begin
    if (cfg_mem_wr_o)
        cfg_mem_arr[cfg_mem_wr_addr_o[4:0]] <= cfg_mem_wr_data_o;
    if (bba_mem_wr_o)
        bba_mem_arr[bba_mem_wr_addr_o[4:0]] <= bba_mem_wr_data_o;
end

// Memory read models.
//
// Default: COMBINATIONAL — data is valid the same cycle the address is driven.
// This is the harder case for the config engine, and the one the ~15 top-level
// testbenches across FlexMan/jabra/Bosch use, so CFG_MEM_SYNC=0 must handle it.
//
// -define CFG_SYNC: a 1-cycle REGISTERED read, matching what every real build
// actually instantiates (shared/bram_sdp.v, a deployment SRAM model), paired with
// CFG_MEM_SYNC=1 above.  Gated on cfg_mem_rd_o, like the SYNCMEM_FAB model in
// a downstream deployment TB, so it also proves the engine never samples a
// cycle where it failed to issue a read.
//
// bba is combinational in BOTH modes: the BBA engine is untouched, and on
// silicon the BBA store is a flop regfile read 0-cycle by fill_unit.
`ifdef CFG_SYNC
always @(posedge clk) if (cfg_mem_rd_o) cfg_mem_data_i <= cfg_mem_arr[cfg_mem_addr_o[4:0]];
`else
always @(*) cfg_mem_data_i = cfg_mem_arr[cfg_mem_addr_o[4:0]];
`endif
always @(*) bba_mem_data_i = bba_mem_arr[bba_mem_addr_o[4:0]];

// ─── Accelerator receive capture ──────────────────────────────────────────────
// clr_capture: one-cycle pulse from initial block resets counters
reg clr_capture;

reg [5:0] acc0_cfg_cnt, acc1_cfg_cnt;
reg [5:0] acc0_bba_cnt, acc1_bba_cnt;

reg [DATAWORD_SZ-1:0] acc0_cfg_rx [0:WORDS_PER_CONFIG-1];
reg [DATAWORD_SZ-1:0] acc0_bba_rx [0:NUM_BBA_SENDS-1];
reg [DATAWORD_SZ-1:0] acc1_cfg_rx [0:WORDS_PER_CONFIG-1];
reg [DATAWORD_SZ-1:0] acc1_bba_rx [0:NUM_BBA_SENDS-1];

always @(posedge clk or posedge reset) begin
    if (reset || clr_capture) begin
        acc0_cfg_cnt <= 6'd0;
        acc1_cfg_cnt <= 6'd0;
        acc0_bba_cnt <= 6'd0;
        acc1_bba_cnt <= 6'd0;
    end else begin
        if (cm_config_wr_o[0] && !cm_config_wait_i[0]) begin
            if (acc0_cfg_cnt < WORDS_PER_CONFIG)
                acc0_cfg_rx[acc0_cfg_cnt] <= cm_config_data_o;
            acc0_cfg_cnt <= acc0_cfg_cnt + 1'b1;
        end
        if (cm_config_wr_o[1] && !cm_config_wait_i[1]) begin
            if (acc1_cfg_cnt < WORDS_PER_CONFIG)
                acc1_cfg_rx[acc1_cfg_cnt] <= cm_config_data_o;
            acc1_cfg_cnt <= acc1_cfg_cnt + 1'b1;
        end
        if (cm_buff_base_wr_o[0] && !cm_buff_base_wait_i[0]) begin
            if (acc0_bba_cnt < NUM_BBA_SENDS)
                acc0_bba_rx[acc0_bba_cnt] <= cm_buff_base_data_o;
            acc0_bba_cnt <= acc0_bba_cnt + 1'b1;
        end
        if (cm_buff_base_wr_o[1] && !cm_buff_base_wait_i[1]) begin
            if (acc1_bba_cnt < NUM_BBA_SENDS)
                acc1_bba_rx[acc1_bba_cnt] <= cm_buff_base_data_o;
            acc1_bba_cnt <= acc1_bba_cnt + 1'b1;
        end
    end
end

// ─── Transaction monitor ──────────────────────────────────────────────────────
always @(posedge clk) begin
    if (cm_config_wr_o != 0 && cm_config_wait_i == 0)
        $display("[%0t] CFG_WR  acc=%0d  data=%08h",
                 $time, cm_tgt_acc_o, cm_config_data_o);
    if (cm_buff_base_wr_o != 0 && cm_buff_base_wait_i == 0)
        $display("[%0t] BBA_WR  acc=%0d  data=%08h",
                 $time, cm_tgt_acc_o, cm_buff_base_data_o);
    if (cm_config_finished_o)
        $display("[%0t] FINISHED", $time);
end

// ─── Helper functions ─────────────────────────────────────────────────────────
// Byte address of config word `word` within config entry `cfg_id`
function [ADDR_SZ-1:0] cfg_addr;
    input integer cfg_id;
    input integer word;
    begin
        cfg_addr = CFG_BASE + (cfg_id * WORDS_PER_CONFIG + word) * 4;
    end
endfunction

// Byte address of BBA entry for buffer `buf_id`
function [ADDR_SZ-1:0] bba_addr;
    input integer buf_id;
    begin
        bba_addr = BBA_BASE + buf_id * 4;
    end
endfunction

// Pack four buffer IDs into a scheduler-format buffer_info word
// Packing: [4:0]=src1, [9:5]=src2, [14:10]=src3, [19:15]=tgt
function [SCH_ENTRY_SZ-1:0] pack_bufs;
    input [BUFF_INDEX_SZ-1:0] tgt, s1, s2, s3;
    begin
        pack_bufs = {12'b0, tgt, s3, s2, s1};
    end
endfunction

// ─── Tasks ────────────────────────────────────────────────────────────────────
integer fail_count;

task do_reset;
    begin
        reset               = 1'b1;
        clr_capture         = 1'b0;
        cm_sys_req_i        = 1'b0;
        cm_sys_addr_i       = {ADDR_SZ{1'b0}};
        cm_sys_data_i       = {DATAWORD_SZ{1'b0}};
        start_new_block_i   = 1'b0;
        target_acc_i        = {TGT_ACC_ID_SZ{1'b0}};
        config_id_i         = {CFG_ID_SZ{1'b0}};
        buffer_info_i       = {SCH_ENTRY_SZ{1'b0}};
        cfg_mem_wait_i      = 1'b0;
        bba_mem_wait_i      = 1'b0;
        cm_config_wait_i    = {NUM_ACC{1'b0}};
        cm_buff_base_wait_i = {NUM_ACC{1'b0}};
        repeat(4) @(posedge clk);
        #1;
        reset = 1'b0;
        @(posedge clk); #1;  // one settle cycle out of reset
    end
endtask

// Reset accelerator capture counters (does NOT clear arrays — old values
// beyond the counter are never checked)
task clear_cap;
    begin
        clr_capture = 1'b1;
        @(posedge clk); #1;
        clr_capture = 1'b0;
    end
endtask

// Single AXI write: hold req for one clock cycle
task axi_wr;
    input [ADDR_SZ-1:0]    addr;
    input [DATAWORD_SZ-1:0] data;
    begin
        cm_sys_addr_i = addr;
        cm_sys_data_i = data;
        cm_sys_req_i  = 1'b1;
        @(posedge clk); #1;   // memory write captured at this posedge
        cm_sys_req_i  = 1'b0;
    end
endtask

// Pre-load both tables with test data via DUT's AXI slave port
task load_tables;
    integer i;
    begin
        // Config 0, words 0-3: data = 32'hC000_0000 | word_index
        for (i = 0; i < WORDS_PER_CONFIG; i = i + 1)
            axi_wr(cfg_addr(0, i), 32'hC000_0000 | i[DATAWORD_SZ-1:0]);
        // Config 1, words 0-3: data = 32'hC100_0000 | word_index
        for (i = 0; i < WORDS_PER_CONFIG; i = i + 1)
            axi_wr(cfg_addr(1, i), 32'hC100_0000 | i[DATAWORD_SZ-1:0]);
        // BBA entries for all buffer IDs used across all tests
        // (bufs 1,3,5,7,9,10,11,12): data = 32'hB000_0000 | buf_id
        axi_wr(bba_addr(1),  32'hB000_0001);
        axi_wr(bba_addr(3),  32'hB000_0003);
        axi_wr(bba_addr(5),  32'hB000_0005);
        axi_wr(bba_addr(7),  32'hB000_0007);
        axi_wr(bba_addr(9),  32'hB000_0009);
        axi_wr(bba_addr(10), 32'hB000_000A);
        axi_wr(bba_addr(11), 32'hB000_000B);
        axi_wr(bba_addr(12), 32'hB000_000C);
    end
endtask

// ─── Config-push latency measurement ──────────────────────────────────────────
// Cycles from the start_new_block_i pulse to cm_config_finished_o — which is
// exactly the dead time in front of every compute task, since the accelerator
// only starts on that pulse (flexman.v: acc_start_new_block = cm_config_finished_o).
// One always-block so `cyc` is never read across a race.
integer cyc, trig_cyc, meas_lat;
initial begin cyc = 0; trig_cyc = 0; meas_lat = -1; end
always @(posedge clk) if (!reset) begin
    cyc = cyc + 1;
    if (start_new_block_i)    trig_cyc = cyc;
    if (cm_config_finished_o) meas_lat = cyc - trig_cyc;
end

// Expected latency: whichever engine finishes last.  The config engine writes
// word w at T+3+STRIDE*w and raises done the next cycle; the (untouched) BBA
// engine writes send w at T+3+3w.  STRIDE is the whole point of CFG_MEM_SYNC:
// 3 before this was pipelined, 2 latency-agnostic, 1 with a synchronous memory.
`ifdef CFG_SYNC
localparam CFG_STRIDE = 1;
`else
localparam CFG_STRIDE = 2;
`endif
localparam EXP_LAT_CFG = 4 + CFG_STRIDE * (WORDS_PER_CONFIG - 1);
localparam EXP_LAT_BBA = 4 + 3 * (NUM_BBA_SENDS - 1);
localparam EXP_LAT     = (EXP_LAT_CFG > EXP_LAT_BBA) ? EXP_LAT_CFG : EXP_LAT_BBA;

task check_latency;
    input [8*8-1:0] tag;
    begin
        // wait_done returns on the same edge that PRODUCES the pulse; the
        // counter block samples it on the NEXT edge. Line them up.
        @(posedge clk); #1;
        if (meas_lat !== EXP_LAT) begin
            $display("FAIL %0s: config-push latency %0d cyc (exp %0d) — WPC=%0d STRIDE=%0d",
                     tag, meas_lat, EXP_LAT, WORDS_PER_CONFIG, CFG_STRIDE);
            fail_count = fail_count + 1;
        end else
            $display("PASS %0s: config-push latency %0d cyc (WPC=%0d, %0d cyc/word)",
                     tag, meas_lat, WORDS_PER_CONFIG, CFG_STRIDE);
    end
endtask

// One-cycle start pulse to the DUT
task trigger;
    input [TGT_ACC_ID_SZ-1:0]  acc;
    input [CFG_ID_SZ-1:0]      cfg_id;
    input [BUFF_INDEX_SZ-1:0]  tgt, s1, s2, s3;
    begin
        target_acc_i      = acc;
        config_id_i       = cfg_id;
        buffer_info_i     = pack_bufs(tgt, s1, s2, s3);
        start_new_block_i = 1'b1;
        @(posedge clk); #1;
        start_new_block_i = 1'b0;
    end
endtask

// Spin until cm_config_finished_o pulses or TIMEOUT cycles elapse
task wait_done;
    reg   seen;
    integer t;
    begin
        seen = 1'b0;
        for (t = 0; t < TIMEOUT && !seen; t = t + 1) begin
            @(posedge clk); #1;
            if (cm_config_finished_o) seen = 1'b1;
        end
        if (!seen) begin
            $display("FAIL: timeout — cm_config_finished_o never pulsed");
            fail_count = fail_count + 1;
        end
    end
endtask

// ─── Main stimulus ────────────────────────────────────────────────────────────
initial begin
    fail_count = 0;

    do_reset;
    load_tables;

    // ── T1: basic no-wait, target acc 0 ──────────────────────────────────────
    $display("\n--- T1: basic no-wait (acc 0, config 0, tgt=5 s1=3 s2=7 s3=1) ---");
    clear_cap;
    trigger(1'b0, 5'd0, 5'd5, 5'd3, 5'd7, 5'd1);
    wait_done;
    check_latency("T1");

    // cfg: words of config 0 in order
    if (acc0_cfg_cnt !== WORDS_PER_CONFIG)
        begin $display("FAIL T1: acc0 cfg_cnt=%0d (exp %0d)", acc0_cfg_cnt, WORDS_PER_CONFIG); fail_count=fail_count+1; end
    else if (acc0_cfg_rx[0]!==32'hC000_0000 || acc0_cfg_rx[1]!==32'hC000_0001 ||
             acc0_cfg_rx[2]!==32'hC000_0002 || acc0_cfg_rx[3]!==32'hC000_0003)
        begin
            $display("FAIL T1: acc0 cfg data wrong");
            $display("  got  %h %h %h %h",
                     acc0_cfg_rx[0],acc0_cfg_rx[1],acc0_cfg_rx[2],acc0_cfg_rx[3]);
            $display("  exp  c0000000 c0000001 c0000002 c0000003");
            fail_count = fail_count + 1;
        end
    else $display("PASS T1: acc0 cfg data");

    // bba: target(5) src1(3) src2(7) src3(1)
    if (acc0_bba_cnt !== NUM_BBA_SENDS)
        begin $display("FAIL T1: acc0 bba_cnt=%0d (exp %0d)", acc0_bba_cnt, NUM_BBA_SENDS); fail_count=fail_count+1; end
    else if (acc0_bba_rx[0]!==32'hB000_0005 || acc0_bba_rx[1]!==32'hB000_0003 ||
             acc0_bba_rx[2]!==32'hB000_0007 || acc0_bba_rx[3]!==32'hB000_0001)
        begin
            $display("FAIL T1: acc0 bba data wrong");
            $display("  got  %h %h %h %h",
                     acc0_bba_rx[0],acc0_bba_rx[1],acc0_bba_rx[2],acc0_bba_rx[3]);
            $display("  exp  b0000005 b0000003 b0000007 b0000001");
            fail_count = fail_count + 1;
        end
    else $display("PASS T1: acc0 bba data");

    if (acc1_cfg_cnt !== 0)
        begin $display("FAIL T1: acc1 spuriously received %0d cfg words", acc1_cfg_cnt); fail_count=fail_count+1; end
    else $display("PASS T1: acc1 received nothing (cfg)");

    if (acc1_bba_cnt !== 0)
        begin $display("FAIL T1: acc1 spuriously received %0d bba entries", acc1_bba_cnt); fail_count=fail_count+1; end
    else $display("PASS T1: acc1 received nothing (bba)");

    // ── T2: different config_id and different buffers ─────────────────────────
    $display("\n--- T2: config_id=1, acc 0, tgt=10 s1=11 s2=12 s3=9 ---");
    clear_cap;
    trigger(1'b0, 5'd1, 5'd10, 5'd11, 5'd12, 5'd9);
    wait_done;

    if (acc0_cfg_cnt !== WORDS_PER_CONFIG)
        begin $display("FAIL T2: acc0 cfg_cnt=%0d", acc0_cfg_cnt); fail_count=fail_count+1; end
    else if (acc0_cfg_rx[0]!==32'hC100_0000 || acc0_cfg_rx[1]!==32'hC100_0001 ||
             acc0_cfg_rx[2]!==32'hC100_0002 || acc0_cfg_rx[3]!==32'hC100_0003)
        begin
            $display("FAIL T2: acc0 cfg data wrong");
            $display("  got  %h %h %h %h",
                     acc0_cfg_rx[0],acc0_cfg_rx[1],acc0_cfg_rx[2],acc0_cfg_rx[3]);
            fail_count = fail_count + 1;
        end
    else $display("PASS T2: acc0 cfg data (config_id=1)");

    // bba: tgt(10) src1(11) src2(12) src3(9)
    if (acc0_bba_cnt !== NUM_BBA_SENDS)
        begin $display("FAIL T2: acc0 bba_cnt=%0d", acc0_bba_cnt); fail_count=fail_count+1; end
    else if (acc0_bba_rx[0]!==32'hB000_000A || acc0_bba_rx[1]!==32'hB000_000B ||
             acc0_bba_rx[2]!==32'hB000_000C || acc0_bba_rx[3]!==32'hB000_0009)
        begin
            $display("FAIL T2: acc0 bba data wrong");
            $display("  got  %h %h %h %h",
                     acc0_bba_rx[0],acc0_bba_rx[1],acc0_bba_rx[2],acc0_bba_rx[3]);
            fail_count = fail_count + 1;
        end
    else $display("PASS T2: acc0 bba data");

    // ── T3: target accelerator 1, acc 0 must receive nothing ─────────────────
    $display("\n--- T3: target acc 1 (config 0, same bufs as T1) ---");
    clear_cap;
    trigger(1'b1, 5'd0, 5'd5, 5'd3, 5'd7, 5'd1);
    wait_done;

    if (acc1_cfg_cnt !== WORDS_PER_CONFIG)
        begin $display("FAIL T3: acc1 cfg_cnt=%0d (exp %0d)", acc1_cfg_cnt, WORDS_PER_CONFIG); fail_count=fail_count+1; end
    else if (acc1_cfg_rx[0]!==32'hC000_0000 || acc1_cfg_rx[1]!==32'hC000_0001 ||
             acc1_cfg_rx[2]!==32'hC000_0002 || acc1_cfg_rx[3]!==32'hC000_0003)
        begin
            $display("FAIL T3: acc1 cfg data wrong");
            $display("  got  %h %h %h %h",
                     acc1_cfg_rx[0],acc1_cfg_rx[1],acc1_cfg_rx[2],acc1_cfg_rx[3]);
            fail_count = fail_count + 1;
        end
    else $display("PASS T3: acc1 cfg data");

    if (acc1_bba_cnt !== NUM_BBA_SENDS)
        begin $display("FAIL T3: acc1 bba_cnt=%0d (exp %0d)", acc1_bba_cnt, NUM_BBA_SENDS); fail_count=fail_count+1; end
    else if (acc1_bba_rx[0]!==32'hB000_0005 || acc1_bba_rx[1]!==32'hB000_0003 ||
             acc1_bba_rx[2]!==32'hB000_0007 || acc1_bba_rx[3]!==32'hB000_0001)
        begin
            $display("FAIL T3: acc1 bba data wrong");
            $display("  got  %h %h %h %h",
                     acc1_bba_rx[0],acc1_bba_rx[1],acc1_bba_rx[2],acc1_bba_rx[3]);
            fail_count = fail_count + 1;
        end
    else $display("PASS T3: acc1 bba data");

    if (acc0_cfg_cnt !== 0)
        begin $display("FAIL T3: acc0 spuriously received %0d cfg words", acc0_cfg_cnt); fail_count=fail_count+1; end
    else $display("PASS T3: acc0 received nothing (one-hot strobe correct)");

    // ── T4: back-to-back blocks without gap ───────────────────────────────────
    $display("\n--- T4: back-to-back blocks ---");
    clear_cap;
    trigger(1'b0, 5'd0, 5'd5, 5'd3, 5'd7, 5'd1);
    wait_done;
    // immediately trigger second block (captures are cumulative — check cnt=4 each time)
    clear_cap;
    trigger(1'b0, 5'd0, 5'd5, 5'd3, 5'd7, 5'd1);
    wait_done;

    if (acc0_cfg_cnt !== WORDS_PER_CONFIG || acc0_bba_cnt !== NUM_BBA_SENDS)
        begin
            $display("FAIL T4: second block incomplete: cfg_cnt=%0d bba_cnt=%0d",
                     acc0_cfg_cnt, acc0_bba_cnt);
            fail_count = fail_count + 1;
        end
    else $display("PASS T4: second block completed cleanly");

    // ── T5: memory back-pressure on first read ────────────────────────────────
    $display("\n--- T5: memory wait states (pre-assert wait before trigger) ---");
    clear_cap;
    cfg_mem_wait_i = 1'b1;   // assert before DUT even sees the read
    bba_mem_wait_i = 1'b1;
    trigger(1'b0, 5'd0, 5'd5, 5'd3, 5'd7, 5'd1);
    // DUT is now in MEM_RD with wait=1 on both channels; hold for 2 extra cycles
    @(posedge clk); @(posedge clk);
    #1;
    cfg_mem_wait_i = 1'b0;
    bba_mem_wait_i = 1'b0;
    wait_done;

    if (acc0_cfg_cnt !== WORDS_PER_CONFIG)
        begin $display("FAIL T5: acc0 cfg_cnt=%0d", acc0_cfg_cnt); fail_count=fail_count+1; end
    else if (acc0_cfg_rx[0]!==32'hC000_0000 || acc0_cfg_rx[1]!==32'hC000_0001 ||
             acc0_cfg_rx[2]!==32'hC000_0002 || acc0_cfg_rx[3]!==32'hC000_0003)
        begin $display("FAIL T5: cfg data wrong after memory wait"); fail_count=fail_count+1; end
    else $display("PASS T5: cfg data correct despite memory back-pressure");

    if (acc0_bba_cnt !== NUM_BBA_SENDS)
        begin $display("FAIL T5: acc0 bba_cnt=%0d", acc0_bba_cnt); fail_count=fail_count+1; end
    else if (acc0_bba_rx[0]!==32'hB000_0005 || acc0_bba_rx[1]!==32'hB000_0003 ||
             acc0_bba_rx[2]!==32'hB000_0007 || acc0_bba_rx[3]!==32'hB000_0001)
        begin $display("FAIL T5: bba data wrong after memory wait"); fail_count=fail_count+1; end
    else $display("PASS T5: bba data correct despite memory back-pressure");

    // ── T6: accelerator back-pressure ─────────────────────────────────────────
    $display("\n--- T6: accelerator wait states (acc 0 busy for first write) ---");
    clear_cap;
    // Pre-assert accelerator busy so DUT stalls in ACC_WR on arrival
    cm_config_wait_i    = 2'b01;
    cm_buff_base_wait_i = 2'b01;
    trigger(1'b0, 5'd0, 5'd5, 5'd3, 5'd7, 5'd1);
    // Allow FSMs to reach ACC_WR: 3 cycles (IDLE→MEM_RD, MEM_RD→WAIT_DATA, WAIT_DATA→ACC_WR)
    // then one cycle stalling in ACC_WR; release after total of 5 cycles
    @(posedge clk); @(posedge clk); @(posedge clk);
    @(posedge clk); @(posedge clk);
    #1;
    cm_config_wait_i    = 2'b00;
    cm_buff_base_wait_i = 2'b00;
    wait_done;

    if (acc0_cfg_cnt !== WORDS_PER_CONFIG)
        begin $display("FAIL T6: acc0 cfg_cnt=%0d", acc0_cfg_cnt); fail_count=fail_count+1; end
    else if (acc0_cfg_rx[0]!==32'hC000_0000 || acc0_cfg_rx[1]!==32'hC000_0001 ||
             acc0_cfg_rx[2]!==32'hC000_0002 || acc0_cfg_rx[3]!==32'hC000_0003)
        begin $display("FAIL T6: cfg data wrong after accelerator wait"); fail_count=fail_count+1; end
    else $display("PASS T6: cfg data correct despite accelerator back-pressure");

    if (acc0_bba_cnt !== NUM_BBA_SENDS)
        begin $display("FAIL T6: acc0 bba_cnt=%0d", acc0_bba_cnt); fail_count=fail_count+1; end
    else if (acc0_bba_rx[0]!==32'hB000_0005 || acc0_bba_rx[1]!==32'hB000_0003 ||
             acc0_bba_rx[2]!==32'hB000_0007 || acc0_bba_rx[3]!==32'hB000_0001)
        begin $display("FAIL T6: bba data wrong after accelerator wait"); fail_count=fail_count+1; end
    else $display("PASS T6: bba data correct despite accelerator back-pressure");

    // ── Summary ───────────────────────────────────────────────────────────────
    $display("");
    if (fail_count == 0)
        $display("ALL TESTS PASSED");
    else
        $display("%0d CHECK(S) FAILED", fail_count);

    $finish;
end

endmodule
