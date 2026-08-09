// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
// Authors: Simon Davidson & Claude | Created 2026-08-08 | Last modified 2026-08-08
`timescale 10ps/1ps
`include "../shared/constants.v"

// =============================================================================
// tb_flexman_wrap — first simulation of flexman_fpga_wrap (the SYNTHESIS top)
//
// WHY THIS EXISTS
// tb_flexman.v drives the `flexman` CORE with 256-word behavioural arrays whose
// reads are COMBINATIONAL and whose wait inputs are tied to zero. Vivado
// synthesises flexman_fpga_wrap, where every one of those 19 memories is a real
// shared/bram_*.v primitive with a REGISTERED read. Nothing had ever simulated
// that difference: the bba bug (fill_unit sampling a 1-cycle BRAM
// combinationally) survived ~3 months of green core regressions and was only
// caught when a downstream deployment wrapper TB became the first TB to drive a wrapper.
//
// The fix — hold fu_bba_mem_wait_i for the first cycle of each read — was
// propagated into this wrapper on 2026-08-07 backed ONLY by elaboration and
// code equivalence. T5 below is the direct simulation evidence, and it is
// verified to have teeth: it FAILS under +define+NEG_CTRL_BBA (which defeats
// the fix) and PASSES without it.
//
// It is a near-clone of tb_flexman.v — same four tests (T1..T4), same vectors,
// same expected values — plus T5, which tb_flexman has no equivalent of. The
// other differences are all forced by driving the wrapper rather than the core:
//   1. 11 ports instead of ~180 — every memory is now INSIDE the DUT.
//   2. Memories are preloaded by backdoor into dut.u_<name>.mem. Every
//      shared/bram_*.v names its array `mem`, so this is uniform.
//   3. Those bram_*.v modules have NO `initial` block (tb_flexman's sram_model
//      does), so every array AND every dout register must be zeroed explicitly
//      or X propagates into the scheduler and wedges the design before it
//      executes an instruction.
//   4. cfg/bba/weight reads are now 1-cycle REGISTERED where tb_flexman modelled
//      them as combinational.
//
// ⚠️ TWO WRAPPER-DEFAULT DISCREPANCIES FOUND WHILE WRITING THIS (2026-08-08)
//   a) flexman_fpga_wrap declares WORDS_PER_CONFIG = 4. flexman.v itself
//      defaults to 16, and jabra_top_fpga_wrap, another deployment wrapper,
//      the deployment board top and the deployment wrapper ALL use 16 — this
//      wrapper is the only one at 4. It changes the config_manager's cfg_mem
//      stride (4 words per cfg_id instead of 16), so a program built for the
//      16-word layout reads the wrong words. This TB instantiates with 16 to
//      match tb_flexman's vectors and its siblings. RAISED FOR DECISION — if 4
//      is not deliberate, the wrapper default should change.
//   b) The wrapper does NOT forward POOL_RD_BASE to flexman, so it cannot be
//      overridden from the synthesis top. It falls back to flexman's default
//      32'h1010_0000, which is the value T4 expects, so T4 passes — but the
//      parameter is unreachable for any build that needs a different window.
//
// Usage: bash sim_flexman_wrap.bsh
// =============================================================================
module tb_flexman_wrap;

// ─── Parameters (mirror tb_flexman.v) ────────────────────────────────────────
localparam NUM_BUFFERS         = 16;
localparam NUM_HW_ACCELERATORS = 5;
localparam WORDS_PER_CONFIG    = 16;   // see discrepancy (a) above
localparam CFG_ID_SZ           = 7;
localparam BUFF_INDX_SZ        = 4;
localparam TGT_ACC_SZ          = 3;
localparam TGT_COUNT_SZ        = 4;
localparam PROG_ADDR_BITS      = 10;
localparam PROG_DATA_BITS      = 32;
localparam NUM_SCH_ENTRIES     = 4;
localparam COL_BUFF_ID_SZ      = 16;

// Wrapper region depths — left at flexman_fpga_wrap's DEFAULTS so this TB
// exercises exactly the configuration Vivado builds. Only the zeroing loops
// need them; the DUT is instantiated without depth overrides.
localparam ACC_MEM_DEPTH  = 1024;
localparam PROG_MEM_DEPTH = 1024;
localparam CFG_MEM_DEPTH  = 1024;
localparam BBA_MEM_DEPTH  = 64;

localparam [31:0] POOL_RD_BASE = 32'h1010_0000;  // flexman default; see (b)

// ─── Instruction encoding (verbatim from tb_flexman.v) ───────────────────────
localparam STOP_INST = 32'h0000_0002;   // opcode 3'b010

function [31:0] nxt_inst;
    input nxt_in, nxt_out;
    // [2:0]=opcode(100), [3]=rsv, [4]=nxt_in, [5]=nxt_out
    nxt_inst = {26'b0, nxt_out, nxt_in, 1'b0, 3'b100};
endfunction

// FILL word 1: [2:0]=101, [6:3]=buf_id (4-bit), [7]=0, [8]=colour,
//              [12:9]=#targets (4-bit), [31:13]=block_size (19-bit)
function [31:0] fill_w1;
    input [3:0]  buf_id;
    input        col;
    input [3:0]  ntgt;
    input [18:0] sz;
    fill_w1 = {sz, ntgt, col, 1'b0, buf_id, 3'b101};
endfunction

// ─── Clock / reset ───────────────────────────────────────────────────────────
reg clk, reset;
initial clk = 1'b0;
always #5 clk = ~clk;

// ─── DUT interface (the wrapper's entire port list: 11 signals) ──────────────
reg         sys_req_i;
reg  [31:0] sys_addr_i;
reg  [31:0] sys_data_i;
wire        test_stall_pipe = 1'b0;
wire        sys_ack_o;
wire [31:0] sys_data_o;
wire        nxt_input_pulse_o, nxt_output_pulse_o, cm_config_finished_o;

// ─── DUT: the SYNTHESIS top ──────────────────────────────────────────────────
flexman_fpga_wrap #(
    .NUM_BUFFERS         (NUM_BUFFERS),
    .NUM_HW_ACCELERATORS (NUM_HW_ACCELERATORS),
    .WORDS_PER_CONFIG    (WORDS_PER_CONFIG),
    .CFG_ID_SZ           (CFG_ID_SZ),
    .BUFF_INDX_SZ        (BUFF_INDX_SZ),
    .TGT_ACC_SZ          (TGT_ACC_SZ),
    .TGT_COUNT_SZ        (TGT_COUNT_SZ),
    .PROG_ADDR_BITS      (PROG_ADDR_BITS),
    .PROG_DATA_BITS      (PROG_DATA_BITS),
    .NUM_SCH_ENTRIES     (NUM_SCH_ENTRIES),
    .COL_BUFF_ID_SZ      (COL_BUFF_ID_SZ)
) dut (
    .clk(clk), .reset(reset), .test_stall_pipe(test_stall_pipe),
    .sys_req_i(sys_req_i), .sys_ack_o(sys_ack_o), .sys_addr_i(sys_addr_i),
    .sys_data_i(sys_data_i), .sys_data_o(sys_data_o),
    .nxt_input_pulse_o(nxt_input_pulse_o), .nxt_output_pulse_o(nxt_output_pulse_o),
    .cm_config_finished_o(cm_config_finished_o)
);

// ─── Pool access: logical addr -> (bank = a[1:0], word = a>>2) ───────────────
// Same mapping as tb_flexman; only the target changes (DUT arrays, not the TB's).
function [31:0] pool_rd;
    input integer a;
    case (a[1:0])
        2'd0: pool_rd = dut.u_m0_data_mem.mem[a >> 2];
        2'd1: pool_rd = dut.u_m1_data_mem.mem[a >> 2];
        2'd2: pool_rd = dut.u_m2_data_mem.mem[a >> 2];
        default: pool_rd = dut.u_m3_data_mem.mem[a >> 2];
    endcase
endfunction
task pool_wr;
    input integer a;
    input [31:0]  d;
    begin
        case (a[1:0])
            2'd0: dut.u_m0_data_mem.mem[a >> 2] = d;
            2'd1: dut.u_m1_data_mem.mem[a >> 2] = d;
            2'd2: dut.u_m2_data_mem.mem[a >> 2] = d;
            default: dut.u_m3_data_mem.mem[a >> 2] = d;
        endcase
    end
endtask

// ─── Zero every array AND every output register in the DUT ───────────────────
// MANDATORY: none of the bram_*.v primitives self-initialise. Anything left
// unwritten stays X, and that X propagates straight into the scheduler /
// config_manager / fill_unit and wedges the design before it executes a single
// instruction. tb_flexman never hits this because its sram_model HAS an initial
// block zeroing both mem and rdata.
//
// Per-primitive rule:
//   bram_dist (u_prog_mem)  — `dout` is a continuous assign, NOT a reg. Zero the
//                             array only; assigning .dout is illegal.
//   bram_sdp / bram_sp      — `output reg dout`  -> zero .dout
//   bram_tdp (u_bba_mem)    — `output reg douta/doutb` -> zero both
integer mi;
task clear_all_mems;
    begin
        for (mi = 0; mi < PROG_MEM_DEPTH; mi = mi + 1) dut.u_prog_mem.mem[mi] = 32'h0;
        for (mi = 0; mi < CFG_MEM_DEPTH;  mi = mi + 1) dut.u_cfg_mem.mem[mi]  = 32'h0;
        for (mi = 0; mi < BBA_MEM_DEPTH;  mi = mi + 1) dut.u_bba_mem.mem[mi]  = 32'h0;
        for (mi = 0; mi < ACC_MEM_DEPTH;  mi = mi + 1) begin
            dut.u_s0_weight_mem.mem[mi]    = 0;
            dut.u_s0_bias_curr_mem.mem[mi] = 0;
            dut.u_s0_thresh_mem.mem[mi]    = 0;
            dut.u_s0_pot_mem.mem[mi]       = 0;
            dut.u_s1_weight_mem.mem[mi]    = 0;
            dut.u_s1_bias_curr_mem.mem[mi] = 0;
            dut.u_s1_thresh_mem.mem[mi]    = 0;
            dut.u_s1_pot_mem.mem[mi]       = 0;
            dut.u_a0_weight_mem.mem[mi]    = 0;
            dut.u_a0_bias_curr_mem.mem[mi] = 0;
            dut.u_a0_thresh_mem.mem[mi]    = 0;
            dut.u_a0_pot_mem.mem[mi]       = 0;
            dut.u_m0_data_mem.mem[mi]      = 0;
            dut.u_m1_data_mem.mem[mi]      = 0;
            dut.u_m2_data_mem.mem[mi]      = 0;
            dut.u_m3_data_mem.mem[mi]      = 0;
        end
        // ── The OUTPUT REGISTERS, not just the arrays ────────────────────────
        dut.u_cfg_mem.dout          = 0;
        dut.u_bba_mem.douta         = 0;
        dut.u_bba_mem.doutb         = 0;
        dut.u_s0_weight_mem.dout    = 0;
        dut.u_s0_bias_curr_mem.dout = 0;
        dut.u_s0_thresh_mem.dout    = 0;
        dut.u_s0_pot_mem.dout       = 0;
        dut.u_s1_weight_mem.dout    = 0;
        dut.u_s1_bias_curr_mem.dout = 0;
        dut.u_s1_thresh_mem.dout    = 0;
        dut.u_s1_pot_mem.dout       = 0;
        dut.u_a0_weight_mem.dout    = 0;
        dut.u_a0_bias_curr_mem.dout = 0;
        dut.u_a0_thresh_mem.dout    = 0;
        dut.u_a0_pot_mem.dout       = 0;
        dut.u_m0_data_mem.dout      = 0;
        dut.u_m1_data_mem.dout      = 0;
        dut.u_m2_data_mem.dout      = 0;
        dut.u_m3_data_mem.dout      = 0;
    end
endtask

// ─── Monitors (verbatim from tb_flexman, re-pointed into the wrapper) ────────
integer nxt_in_count;
integer nxt_out_count;
integer fill_wr_count;
integer cm_cfg_done_count;
integer test_num;
integer t3_snn0_fin, t3_snn1_fin, t3_disp0, t3_disp1;
integer errors;

initial begin
    nxt_in_count  = 0;
    nxt_out_count = 0;
    fill_wr_count = 0;
    cm_cfg_done_count = 0;
    test_num      = 0;
    errors        = 0;
    t3_snn0_fin = 0; t3_snn1_fin = 0; t3_disp0 = 0; t3_disp1 = 0;
end

always @(posedge clk) begin
    if (!reset) begin
        if (nxt_input_pulse_o) begin
            nxt_in_count = nxt_in_count + 1;
            $display("[%0t] T%0d: nxt_input_pulse  #%0d", $time, test_num, nxt_in_count);
        end
        if (nxt_output_pulse_o) begin
            nxt_out_count = nxt_out_count + 1;
            $display("[%0t] T%0d: nxt_output_pulse #%0d", $time, test_num, nxt_out_count);
        end
        // Shared-pool writes (FILL during T2; only FILL writes the pool then).
        // These are now INTERNAL wrapper wires rather than TB-level signals.
        if (dut.m0_data_mem_wr_o) fill_wr_count = fill_wr_count + 1;
        if (dut.m1_data_mem_wr_o) fill_wr_count = fill_wr_count + 1;
        if (dut.m2_data_mem_wr_o) fill_wr_count = fill_wr_count + 1;
        if (dut.m3_data_mem_wr_o) fill_wr_count = fill_wr_count + 1;
        if (cm_config_finished_o) cm_cfg_done_count = cm_cfg_done_count + 1;
        if (dut.u_flexman.u_snn0.acc_finished_o) t3_snn0_fin = t3_snn0_fin + 1;
        if (dut.u_flexman.u_snn1.acc_finished_o) t3_snn1_fin = t3_snn1_fin + 1;
        if (dut.u_flexman.sch_start_new_block && dut.u_flexman.sch_target_acc == 3'd0)
            t3_disp0 = t3_disp0 + 1;
        if (dut.u_flexman.sch_start_new_block && dut.u_flexman.sch_target_acc == 3'd1)
            t3_disp1 = t3_disp1 + 1;
    end
end

// ─── Bring-up debug (`+define+WRAP_DBG`) ─────────────────────────────────────
// This is the first simulation of flexman_fpga_wrap, so when it stalls the
// question is always "how far did it get": did the program start, is the
// scheduler still fetching, is the fill_unit parked waiting on bba?
`ifdef WRAP_DBG
integer dbgc; initial dbgc = 0;
always @(posedge clk) begin
    dbgc = dbgc + 1;
    if (dbgc > 30 && dbgc < 160)
        $display("[FU] c=%0d fu_state=%0d buff=%0d rd=%b wait=%b bba_addr=%0d bba_data=%08x | sch_fill=%b",
                 dbgc, dut.u_flexman.u_fill.state_r, dut.u_flexman.u_fill.buff_id_r,
                 dut.fu_bba_mem_rd_o, dut.fu_bba_wait,
                 dut.fu_bba_mem_addr_o, dut.fu_bba_mem_data_i,
                 dut.u_flexman.u_scheduler.inst_is_fill);
    if (dbgc < 40 || dbgc % 5000 == 0)
        $display("[DBG] c=%0d run=%b pc=%0d inst=%08x | task=%b nxt=%b fill=%b stop=%b | req=%b cfg_fin=%b",
                 dbgc, dut.u_flexman.u_scheduler.prog_running_r,
                 dut.u_flexman.u_scheduler.prog_counter_r,
                 dut.u_flexman.u_scheduler.held_inst_word_r,
                 dut.u_flexman.u_scheduler.inst_is_task,
                 dut.u_flexman.u_scheduler.inst_is_nxt,
                 dut.u_flexman.u_scheduler.inst_is_fill,
                 dut.u_flexman.u_scheduler.inst_is_stop,
                 dut.prog_mem_req_o, cm_config_finished_o);
end
`endif

// ─── Pool-write tracing (`+define+POOL_DBG`) ─────────────────────────────────
`ifdef POOL_DBG
always @(posedge clk) if (!reset) begin
    if (dut.m0_data_mem_wr_o) $display("[POOLW] @%0t T%0d bank0 word=%0d data=%08h",
        $time, test_num, dut.m0_data_mem_addr_o, dut.m0_data_mem_wdata_o);
    if (dut.m1_data_mem_wr_o) $display("[POOLW] @%0t T%0d bank1 word=%0d data=%08h",
        $time, test_num, dut.m1_data_mem_addr_o, dut.m1_data_mem_wdata_o);
    if (dut.m2_data_mem_wr_o) $display("[POOLW] @%0t T%0d bank2 word=%0d data=%08h",
        $time, test_num, dut.m2_data_mem_addr_o, dut.m2_data_mem_wdata_o);
    if (dut.m3_data_mem_wr_o) $display("[POOLW] @%0t T%0d bank3 word=%0d data=%08h",
        $time, test_num, dut.m3_data_mem_addr_o, dut.m3_data_mem_wdata_o);
    if (dut.fu_bba_mem_rd_o) $display("[BBARD] @%0t T%0d addr=%0d wait=%b doutb=%08h",
        $time, test_num, dut.fu_bba_mem_addr_o, dut.fu_bba_wait, dut.fu_bba_mem_data_i);
end
`endif

// ─── NEGATIVE CONTROL (`+define+NEG_CTRL_BBA`) ───────────────────────────────
// A regression test is only evidence if it FAILS when the bug is present. This
// force re-creates the pre-fix wrapper by defeating the bba wait state, so
// fill_unit samples u_bba_mem a cycle early — exactly the ~3-month-old bug.
//
// EXPECTED RESULT WITH THIS DEFINE: [T5] FAIL, with the fill landing at the
// poisoned base (pool[0..3] = 0x22222222) instead of the real one
// (pool[64..67]). If T5 still PASSES here, T5 is not exercising the bba path
// and this TB's headline claim is void — do not trust it.
//
// Note T2 passes either way, by design; see the T5 comment for why.
// Sim-only; no RTL change.
`ifdef NEG_CTRL_BBA
initial begin
    @(negedge reset);
    force dut.fu_bba_wait = 1'b0;
    $display("[NEG_CTRL] fu_bba_wait forced to 0 — T5 is EXPECTED to FAIL.");
end
`endif

// ─── Timeout ─────────────────────────────────────────────────────────────────
initial begin
    #500000;
    $display("[VERDICT] TIMEOUT — simulation exceeded 500000 time units (test %0d).", test_num);
    $finish;
end

// ─── AXI helpers (verbatim from tb_flexman.v) ────────────────────────────────
task axi_write;
    input [31:0] addr;
    input [31:0] data;
    begin
        @(posedge clk); #1;
        sys_req_i  = 1'b1;
        sys_addr_i = addr;
        sys_data_i = data;
        @(posedge clk); #1;
        sys_req_i  = 1'b0;
        sys_addr_i = 32'h0;
        sys_data_i = 32'h0;
    end
endtask

task axi_read;
    input  [31:0] addr;
    output [31:0] data;
    integer guard;
    begin
        @(posedge clk); #1;
        sys_req_i  = 1'b1;
        sys_addr_i = addr;
        sys_data_i = 32'h0;
        guard      = 0;
        @(posedge clk); #1;
        while (sys_ack_o !== 1'b1 && guard < 1000) begin
            @(posedge clk); #1;
            guard = guard + 1;
        end
        data       = sys_data_o;
        sys_req_i  = 1'b0;
        sys_addr_i = 32'h0;
    end
endtask

// ─── snnAcc static-config helper (verbatim from tb_flexman.v) ────────────────
task cfg_snn_static;
    input [31:0] base;
    input [31:0] spike_base;
    begin
        axi_write(base | 32'h40, 32'd0);          // bin_point_syn_curr
        axi_write(base | 32'h44, 32'd2);          // sp_in_x_len
        axi_write(base | 32'h48, 32'd1);          // sp_in_y_len
        axi_write(base | 32'h4C, 32'd2);          // sp_out_x_len
        axi_write(base | 32'h50, 32'd1);          // sp_out_y_len
        axi_write(base | 32'h54, 32'd4);          // sp_weights_per_word
        axi_write(base | 32'h58, 32'd1);          // sp_rows_per_neuron
        axi_write(base | 32'h5C, 32'd5);          // sp_weight_idx_sz
        axi_write(base | 32'h64, spike_base);     // np_spike_base_addr
        axi_write(base | 32'h68, 32'hFFFF_FFFF);  // syn_curr_decay (static)
        axi_write(base | 32'h6C, 32'hFFFF_FFFF);  // pot_decay (static)
        axi_write(base | 32'h70, 32'd0);          // sp_weight_mode = full
        axi_write(base | 32'h74, 32'd1);          // x_kernel_len
        axi_write(base | 32'h78, 32'd1);          // y_kernel_len
        axi_write(base | 32'h7C, 32'd1);          // x_kernel_step
        axi_write(base | 32'h80, 32'd1);          // y_kernel_step
        axi_write(base | 32'h84, 32'd0);          // x_kernel_offset
        axi_write(base | 32'h88, 32'd0);          // y_kernel_offset
    end
endtask

// ─── Stimulus ────────────────────────────────────────────────────────────────
initial begin
    reset      = 1'b1;
    sys_req_i  = 1'b0;
    sys_addr_i = 32'h0;
    sys_data_i = 32'h0;
    clear_all_mems;

    // ══════════════════════════════════════════════════════════════════════
    // T1: NXT(in+out) + STOP — NXT on an empty table fires immediately.
    // ══════════════════════════════════════════════════════════════════════
    test_num = 1;
    dut.u_prog_mem.mem[0] = nxt_inst(1'b1, 1'b1);
    dut.u_prog_mem.mem[1] = STOP_INST;

    repeat(4) @(posedge clk); #1;
    reset = 1'b0;

    axi_write(32'hE000_0000, 32'd0);   // LOAD_PC = 0
    axi_write(32'hE010_0000, 32'd0);   // START

    repeat(60) @(posedge clk);

    if (nxt_in_count == 1 && nxt_out_count == 1) begin
        $display("[T1] PASS  NXT in x%0d out x%0d", nxt_in_count, nxt_out_count);
    end else begin
        $display("[T1] FAIL  NXT in x%0d (exp 1)  out x%0d (exp 1)",
                 nxt_in_count, nxt_out_count);
        errors = errors + 1;
        $display("[VERDICT] FAIL"); #20 $finish;
    end

    // ══════════════════════════════════════════════════════════════════════
    // T2: FILL buf=0 (4 words, 0xDEADBEEF) -> shared pool + STOP
    //
    // Basic FILL dispatch through the wrapper's registered memories. NOTE: this
    // does NOT detect the bba bug — a single fill with an all-zero bba reads the
    // same zero whether or not the wait state is present. See T5 for the test
    // that actually catches it.
    // ══════════════════════════════════════════════════════════════════════
    reset = 1'b1;
    nxt_in_count  = 0;
    nxt_out_count = 0;
    fill_wr_count = 0;
    clear_all_mems;
    repeat(4) @(posedge clk); #1;
    reset = 1'b0;

    test_num = 2;

    dut.u_prog_mem.mem[0] = fill_w1(4'd0, 1'b0, 4'd1, 19'd4);  // 32'h0000_8205
    dut.u_prog_mem.mem[1] = 32'hDEAD_BEEF;                     // fill value
    dut.u_prog_mem.mem[2] = STOP_INST;

    // BBA memory: buffer 0 base word-address = 0 (fill writes pool[0..3])
    dut.u_bba_mem.mem[0] = 32'h0000_0000;

    // fill_unit mem_sel table: buffer 0 -> shared pool (IDX_SHARED_DATA = bit 25).
    axi_write(32'hC000_0000, 32'h0200_0000);

    axi_write(32'hE000_0000, 32'd0);   // LOAD_PC = 0
    axi_write(32'hE010_0000, 32'd0);   // START

    // BBA read now costs an EXTRA cycle vs tb_flexman (the wait-state fix), then
    // 4 write cycles, then the done pulse. The 100-cycle margin absorbs it.
    repeat(100) @(posedge clk);

    if (fill_wr_count == 4         &&
        pool_rd(0) == 32'hDEADBEEF &&
        pool_rd(1) == 32'hDEADBEEF &&
        pool_rd(2) == 32'hDEADBEEF &&
        pool_rd(3) == 32'hDEADBEEF) begin
        $display("[T2] PASS  FILL wrote %0d words; pool[0..3] = 0xDEADBEEF (bba path OK)",
                 fill_wr_count);
    end else begin
        $display("[T2] FAIL  fill_wr_count=%0d (exp 4)  pool[0..3]=%08h %08h %08h %08h",
                 fill_wr_count, pool_rd(0), pool_rd(1), pool_rd(2), pool_rd(3));
        errors = errors + 1;
        $display("[VERDICT] FAIL"); #20 $finish;
    end

    // ══════════════════════════════════════════════════════════════════════
    // T5: ★ THE bba REGRESSION — ONE FILL, WITH doutb PRE-POISONED
    //
    //   T2 above canNOT detect the bba bug, verified by experiment: under
    //   +define+NEG_CTRL_BBA (which defeats the fix) T2 still PASSES. T2 does a
    //   single fill with an all-zero bba, so sampling u_bba_mem one cycle early
    //   reads the same zero and lands in the right place regardless. That is
    //   exactly why the bug read as "intermittent" for 3 months.
    //
    //   The question this test must answer is narrow: does fill_unit sample
    //   u_bba_mem.doutb BEFORE or AFTER the BRAM has presented the addressed
    //   word? Driving that with two real fills was tried and abandoned — both
    //   back-to-back fills (T6) and two separate program runs hit UNRELATED
    //   pre-existing scheduler/fill_unit defects that confound the result.
    //
    //   So poison doutb directly instead. bram_tdp gates doutb on enb
    //   (= fu_bba_mem_rd_o) and HOLDS it otherwise, so a value written here
    //   survives untouched until fill_unit's own read — nothing else drives
    //   port B. One fill, no restarts, no hand-off, single variable:
    //
    //     bba[1] = 64, doutb poisoned to 0, then FILL buffer 1 with 0x22222222
    //     FIXED  : waits a cycle, reads the real 64  -> pool[64..67]
    //     BROKEN : samples the poisoned 0            -> pool[0..3]
    //
    //   Confirmed by negative control: FAILS under +define+NEG_CTRL_BBA, PASSES
    //   without it. That is what makes it evidence rather than decoration.
    // ══════════════════════════════════════════════════════════════════════
    reset = 1'b1;
    fill_wr_count = 0;
    clear_all_mems;
    repeat(4) @(posedge clk); #1;
    reset = 1'b0;

    test_num = 5;

    dut.u_bba_mem.mem[1] = 32'd64;             // the TRUE base for buffer 1
    axi_write(32'hC000_0004, 32'h0200_0000);   // mem_sel: buffer 1 -> shared pool

    dut.u_prog_mem.mem[0] = fill_w1(4'd1, 1'b0, 4'd1, 19'd4);
    dut.u_prog_mem.mem[1] = 32'h2222_2222;
    dut.u_prog_mem.mem[2] = STOP_INST;

    // ★ Poison the BRAM's port-B output register with a WRONG base (0). Held
    //   until fill_unit reads, because enb is fill_unit's own read strobe.
    dut.u_bba_mem.doutb = 32'd0;

    axi_write(32'hE000_0000, 32'd0);   // LOAD_PC = 0
    axi_write(32'hE010_0000, 32'd0);   // START
    repeat(100) @(posedge clk);

    if (fill_wr_count == 4            &&
        pool_rd(64) == 32'h22222222 && pool_rd(65) == 32'h22222222 &&
        pool_rd(66) == 32'h22222222 && pool_rd(67) == 32'h22222222 &&
        pool_rd( 0) == 32'd0        && pool_rd( 3) == 32'd0) begin
        $display("[T5] PASS  FILL used the REAL bba base (64), not the poisoned doutb (bba wait state OK)");
    end else begin
        $display("[T5] FAIL  fill_unit sampled u_bba_mem a cycle early (stale base).");
        $display("           writes=%0d (exp 4)", fill_wr_count);
        $display("           pool[64..67] = %08h %08h %08h %08h  (exp 22222222 x4)",
                 pool_rd(64), pool_rd(65), pool_rd(66), pool_rd(67));
        $display("           pool[0..3]   = %08h %08h %08h %08h  (exp 0 — nonzero means the poison won)",
                 pool_rd(0), pool_rd(1), pool_rd(2), pool_rd(3));
        errors = errors + 1;
        $display("[VERDICT] FAIL"); #20 $finish;
    end

    // ══════════════════════════════════════════════════════════════════════
    // T4: HOST POOL READBACK via the POOL_RD_BASE AXI window
    //   Backdoor-load distinct values into pool words 64..75 (all 4 banks),
    //   read them back through the memory-mapped window and compare. Runs
    //   before T3 and on a disjoint word range so it cannot perturb it.
    // ══════════════════════════════════════════════════════════════════════
    reset = 1'b1; @(posedge clk); #1; @(posedge clk); #1; reset = 1'b0;
    @(posedge clk); #1;
    begin : t4
        integer    a;
        reg [31:0] got;
        reg        t4_ok;
        t4_ok = 1'b1;
        for (a = 64; a < 76; a = a + 1)
            pool_wr(a, 32'hA5A5_0000 | a);
        for (a = 64; a < 76; a = a + 1) begin
            axi_read(POOL_RD_BASE + (a << 2), got);
            if (got !== (32'hA5A5_0000 | a)) begin
                t4_ok = 1'b0;
                $display("[T4] FAIL  word %0d (bank %0d): got %08h exp %08h",
                         a, a % 4, got, 32'hA5A5_0000 | a);
            end
        end
        if (t4_ok)
            $display("[T4] PASS  host read 12 pool words back through POOL_RD_BASE (all 4 banks)");
        else begin
            $display("[T4] FAIL  pool readback mismatch via POOL_RD_BASE");
            errors = errors + 1;
            $display("[VERDICT] FAIL"); #20 $finish;
        end
    end

    // ══════════════════════════════════════════════════════════════════════
    // T3: INTER-ACCELERATOR DATAFLOW THROUGH THE SHARED POOL
    //   snn0 (TASK 1): reads spikes [1,1] @ pool[0] -> syn_curr [20,10], fires
    //        (thresh=1) -> writes spike [1,1] to pool[8] (MID).
    //   snn1 (TASK 2): reads pool[8] as act input -> syn_curr [20,10] @ pool[24].
    //   A non-zero, snn0-dependent snn1 result proves the spike crossed through
    //   the shared pool. This is also the second bba exercise: the scheduler's
    //   producer/consumer dependency is resolved through buffer base addresses.
    // ══════════════════════════════════════════════════════════════════════
    reset = 1'b1;
    nxt_in_count  = 0;
    nxt_out_count = 0;
    fill_wr_count = 0;
    clear_all_mems;
    repeat(4) @(posedge clk); #1;
    reset = 1'b0;
    test_num = 3;

    // --- per-accelerator static (shape) config ---
    cfg_snn_static(32'h1000_0000, 32'd8);    // snn0: spike out -> pool[8] (MID)
    cfg_snn_static(32'h1001_0000, 32'd32);   // snn1: spike out -> pool[32] (OUT)

    // --- per-task cfg_mem (16 words each -> packed offsets 0x00..0x3C) ---
    // cfg_id 0 (snn0): act_base=0(IN), weight=12, syn_curr=20, spike=8(MID)
    dut.u_cfg_mem.mem[ 0]=32'd0;         dut.u_cfg_mem.mem[ 1]=32'd12;
    dut.u_cfg_mem.mem[ 2]=32'd20;        dut.u_cfg_mem.mem[ 3]=32'd30;
    dut.u_cfg_mem.mem[ 4]=32'd40;        dut.u_cfg_mem.mem[ 5]=32'd50;
    dut.u_cfg_mem.mem[ 6]=32'd8;         dut.u_cfg_mem.mem[ 7]=32'hFFFF_FFFF;
    dut.u_cfg_mem.mem[ 8]=32'hFFFF_FFFF; dut.u_cfg_mem.mem[ 9]=32'h0001_0002;
    dut.u_cfg_mem.mem[10]=32'h0001_0002; dut.u_cfg_mem.mem[11]=32'h0001_0001;
    dut.u_cfg_mem.mem[12]=32'd1;         dut.u_cfg_mem.mem[13]=32'h0000_5553;
    dut.u_cfg_mem.mem[14]=32'h0000_0100; dut.u_cfg_mem.mem[15]=32'd0;
    // cfg_id 1 (snn1): act_base=8(MID), weight=12, syn_curr=24, spike=32(OUT)
    dut.u_cfg_mem.mem[16]=32'd8;         dut.u_cfg_mem.mem[17]=32'd12;
    dut.u_cfg_mem.mem[18]=32'd24;        dut.u_cfg_mem.mem[19]=32'd30;
    dut.u_cfg_mem.mem[20]=32'd40;        dut.u_cfg_mem.mem[21]=32'd50;
    dut.u_cfg_mem.mem[22]=32'd32;        dut.u_cfg_mem.mem[23]=32'hFFFF_FFFF;
    dut.u_cfg_mem.mem[24]=32'hFFFF_FFFF; dut.u_cfg_mem.mem[25]=32'h0001_0002;
    dut.u_cfg_mem.mem[26]=32'h0001_0002; dut.u_cfg_mem.mem[27]=32'h0001_0001;
    dut.u_cfg_mem.mem[28]=32'd1;         dut.u_cfg_mem.mem[29]=32'h0000_5553;
    dut.u_cfg_mem.mem[30]=32'h0000_0100; dut.u_cfg_mem.mem[31]=32'd0;

    // --- buffer base-address table (all 0; cfg bases are absolute) ---
    dut.u_bba_mem.mem[0] = 32'd0;
    dut.u_bba_mem.mem[1] = 32'd0;
    dut.u_bba_mem.mem[2] = 32'd0;

    // --- weights: W = [10,5] col-major (8-bit) -> syn_curr [20,10] from [1,1] ---
    dut.u_s0_weight_mem.mem[12] = 32'h0000_050A;
    dut.u_s0_weight_mem.mem[13] = 32'h0000_050A;
    dut.u_s1_weight_mem.mem[12] = 32'h0000_050A;
    dut.u_s1_weight_mem.mem[13] = 32'h0000_050A;
    // --- bias = 0; thresholds (32-bit per neuron, NP_*_SLICE_BITS=32) ---
    dut.u_s0_bias_curr_mem.mem[30] = 32'd0;  dut.u_s0_bias_curr_mem.mem[31] = 32'd0;
    dut.u_s1_bias_curr_mem.mem[30] = 32'd0;  dut.u_s1_bias_curr_mem.mem[31] = 32'd0;
    dut.u_s0_thresh_mem.mem[40]    = 32'd1;  dut.u_s0_thresh_mem.mem[41]    = 32'd1;
    dut.u_s1_thresh_mem.mem[40]    = 32'd50; dut.u_s1_thresh_mem.mem[41]    = 32'd50;

    // --- input spikes for snn0: x = [1,1] at pool[0] ---
    pool_wr(0, 32'h0000_0003);

    // --- mark the input buffer (id 0) FULL so snn0 can dispatch ---
    axi_write(32'hE050_0000, 32'h0000_0010);   // {cnt=1, id=0}

    // --- program: TASK(snn0: IN(0)->MID(1)); TASK(snn1: MID(1)->OUT(2)); STOP ---
    dut.u_prog_mem.mem[0] = 32'h0000_2000;   // snn0: acc=0 cfg=0, slot0=SRC id=0
    dut.u_prog_mem.mem[1] = 32'h11C0_0000;   // slot5=TGT id=1 (MID) ntgt=1
    dut.u_prog_mem.mem[2] = 32'h0000_A028;   // snn1: acc=1 cfg=1, slot0=SRC id=1
    dut.u_prog_mem.mem[3] = 32'h12C0_0000;   // slot5=TGT id=2 (OUT) ntgt=1
    dut.u_prog_mem.mem[4] = STOP_INST;

    axi_write(32'hE000_0000, 32'd0);   // LOAD_PC = 0
    axi_write(32'hE010_0000, 32'd0);   // START

    repeat(4000) @(posedge clk);

    $display("[T3] dispatched snn0/snn1 = %0d/%0d, finished = %0d/%0d",
             t3_disp0, t3_disp1, t3_snn0_fin, t3_snn1_fin);
    $display("       snn0 spike output  pool[8]     = %08h   (handoff buffer MID)",
             pool_rd(8));
    $display("       snn1 syn_curr      pool[24/25] = %08h %08h",
             pool_rd(24), pool_rd(25));
    if (pool_rd(8)  != 32'd0    &&
        pool_rd(24) == 32'h0000_0013 &&
        pool_rd(25) == 32'h0000_0009) begin
        $display("[T3] PASS  inter-accelerator dataflow: snn0 spike -> pool[8] -> snn1 act -> snn1 syn_curr");
    end else begin
        $display("[T3] FAIL  snn1 did not receive snn0's spike output through the pool");
        errors = errors + 1;
        $display("[VERDICT] FAIL"); #20 $finish;
    end

    // ══════════════════════════════════════════════════════════════════════
    // T6 / T7: A FILL CONSTANT MUST NOT BE EXECUTED AS AN INSTRUCTION
    //
    //   A FILL is two words: the descriptor, then a free 32-bit constant. Before
    //   the fix that constant was decoded as an instruction on the cycle it was
    //   consumed as word 2, and acted upon — because the inst_is_* flags are a
    //   raw combinational decode of inst_word, and several consumers keyed off
    //   inst_consumed (which deliberately ORs in inst_consumed_w2 outside the
    //   inst_valid_for_decode gate, so the PC can advance past word 2).
    //
    //   Fixed by gating the decode at source (scheduler.v, commit fb8ec9c).
    //   These two tests are the regression: both FAIL on the pre-fix RTL.
    //
    //   T6  constant 0x11111111, low bits 001 = JUMP
    //       was: do_jump asserted, prog_counter <= inst_word[12:3], pc 1 -> 546;
    //       a single 4-word FILL issued 23 pool writes and the program re-ran.
    //   T7  constant 0x000002BE, low bits 110 = LOOP, id [5:3]=7, max [31:6]=10
    //       was: loop_counter_r[7] <= 10 and loop_restart_r[7] <= 2, silently —
    //       fill DATA corrupting loop state, with no visible symptom until a
    //       later LOOPEND used the counter.
    //
    //   Not wrapper-specific: both reproduce on the `flexman` core too. Nothing
    //   ever hit them in practice because every deployed program fills with
    //   0x00000000. Back-to-back FILLs were never the trigger.
    // ══════════════════════════════════════════════════════════════════════
    reset = 1'b1;
    fill_wr_count = 0;
    clear_all_mems;
    repeat(4) @(posedge clk); #1;
    reset = 1'b0;
    test_num = 6;

    dut.u_bba_mem.mem[0] = 32'd0;
    axi_write(32'hC000_0000, 32'h0200_0000);

    // ONE fill. The only thing unusual about it is the constant's low 3 bits.
    dut.u_prog_mem.mem[0] = fill_w1(4'd0, 1'b0, 4'd1, 19'd4);
    dut.u_prog_mem.mem[1] = 32'h1111_1111;   // low bits 001 == INST_JUMP
    dut.u_prog_mem.mem[2] = STOP_INST;

    axi_write(32'hE000_0000, 32'd0);
    axi_write(32'hE010_0000, 32'd0);
    repeat(400) @(posedge clk);

    if (fill_wr_count == 4 &&
        pool_rd(0) == 32'h11111111 && pool_rd(3) == 32'h11111111) begin
        $display("[T6] PASS  JUMP-tailed fill constant not executed (%0d writes)", fill_wr_count);
    end else begin
        $display("[T6] FAIL  fill constant 0x11111111 executed as JUMP");
        $display("           writes=%0d (exp 4)  pool[0..3]=%08h %08h %08h %08h",
                 fill_wr_count, pool_rd(0), pool_rd(1), pool_rd(2), pool_rd(3));
        errors = errors + 1;
        $display("[VERDICT] FAIL"); #20 $finish;
    end

    // ── T7: LOOP-tailed constant must not touch the loop state ──────────────
    reset = 1'b1;
    fill_wr_count = 0;
    clear_all_mems;
    repeat(4) @(posedge clk); #1;
    reset = 1'b0;
    test_num = 7;

    dut.u_bba_mem.mem[0] = 32'd0;
    axi_write(32'hC000_0000, 32'h0200_0000);

    dut.u_prog_mem.mem[0] = fill_w1(4'd0, 1'b0, 4'd1, 19'd4);
    dut.u_prog_mem.mem[1] = 32'h0000_02BE;   // low bits 110 == INST_LOOP, id=7, max=10
    dut.u_prog_mem.mem[2] = STOP_INST;

    axi_write(32'hE000_0000, 32'd0);
    axi_write(32'hE010_0000, 32'd0);
    repeat(400) @(posedge clk);

    begin : t7
        integer li; integer dirty;
        dirty = 0;
        for (li = 0; li < 8; li = li + 1)
            if (dut.u_flexman.u_scheduler.loop_counter_r[li] !== 0 ||
                dut.u_flexman.u_scheduler.loop_restart_r[li] !== 0) dirty = dirty + 1;
        if (fill_wr_count == 4 && dirty == 0 &&
            pool_rd(0) == 32'h000002BE && pool_rd(3) == 32'h000002BE) begin
            $display("[T7] PASS  LOOP-tailed fill constant left loop state untouched");
        end else begin
            $display("[T7] FAIL  fill constant 0x000002BE corrupted loop state");
            $display("           writes=%0d (exp 4)  dirty loop regs=%0d  counter[7]=%0d restart[7]=%0d",
                     fill_wr_count, dirty,
                     dut.u_flexman.u_scheduler.loop_counter_r[7],
                     dut.u_flexman.u_scheduler.loop_restart_r[7]);
            errors = errors + 1;
            $display("[VERDICT] FAIL"); #20 $finish;
        end
    end

    // ══════════════════════════════════════════════════════════════════════
    if (errors == 0) begin
        $display("[VERDICT] PASS  flexman_fpga_wrap: T1 T2 T3 T4 T5 T6 T7 all passed");
    end else begin
        $display("[VERDICT] FAIL  %0d test(s) failed", errors);
    end
    #20 $finish;
end

endmodule // tb_flexman_wrap
