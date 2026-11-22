// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
// Authors      : Simon Davidson & Claude
// Created      : 2026-11-22
// Last modified: 2026-11-22
`timescale 10ps/1ps

// =============================================================================
// tb_flexman_ann_layer -- run ONE dense 8-bit ANN layer end to end on the FlexMan
// core (scheduler + config manager + annAcc) and check it bit-exact.
//
// DUT: fm_tb_wrap (generated from top_module/flexman_fpga_wrap.v by mkwrap.py).
//   The memories are the wrapper's own shared/bram_*.v primitives -- registered
//   reads, wait inputs tied low, exactly what Vivado builds. For gate-level runs
//   the `flexman` core inside is replaced by the routed netlist; the memories stay
//   RTL (they are the "macros" at the core boundary). The testbench only touches
//   the wrapper's memories by hierarchical path; the `flexman` core is probed
//   only under `ifndef GATE_SIM.
//
// Inputs (all $readmemh, produced by gen_ann_layer.py into $IMG):
//   prog.hex cfg.hex weights.hex pool_in.hex golden.hex  +  params.vh
// Defines:
//   ANN_NLANE ANN_MACB   annAcc lanes / MAC activation width   (RTL runs)
//   CLKHALF              half clock period in 10 ps units (default 333.5 = 6.67 ns)
//   GATE_SIM             DUT core is the routed netlist (no hierarchical probes)
//   GATE_SAIF            $stop at the window edges so a Tcl script can dump SAIF
//   WIN_START WIN_END    window, in clock cycles after reset release (GATE_SIM)
//   IMG="dir/"           image directory (with trailing slash); also pass -incdir dir
//   WINFILE GINITFILE    window / gate-init include names (default window.vh gate_init.vh)
//
// Window = ANN task dispatch (scheduler start_new_block to acc 2) .. annAcc
// acc_finished (spike stage + neuron stage). RTL runs measure it and write
// window.vh; gate runs replay the same cycle numbers (the design is synchronous
// and the gate model zero-delay, so the cycle count is identical -- verified by
// the output check at the same absolute cycle).
// =============================================================================
`ifndef ANN_NLANE
 `define ANN_NLANE 1
`endif
`ifndef ANN_MACB
 `define ANN_MACB 32
`endif
`ifndef CLKHALF
 `define CLKHALF 333.5
`endif
`ifndef IMG
 `define IMG ""
`endif
`ifndef WINFILE
 `define WINFILE "window.vh"
`endif
`ifndef GINITFILE
 `define GINITFILE "gate_init.vh"
`endif
`include "params.vh"

module tb_flexman_ann_layer;

localparam MEM_DEPTH = 1024;
localparam NPOOL     = 4096;

reg clk, reset;
initial clk = 1'b0;
always #(`CLKHALF) clk = ~clk;

reg         sys_req_i;
reg  [31:0] sys_addr_i;
reg  [31:0] sys_data_i;
wire        sys_ack_o;
wire [31:0] sys_data_o;
wire        nxt_input_pulse_o, nxt_output_pulse_o, cm_config_finished_o;

fm_tb_wrap dut (
    .clk(clk), .reset(reset), .test_stall_pipe(1'b0),
    .sys_req_i(sys_req_i), .sys_ack_o(sys_ack_o), .sys_addr_i(sys_addr_i),
    .sys_data_i(sys_data_i), .sys_data_o(sys_data_o),
    .nxt_input_pulse_o(nxt_input_pulse_o), .nxt_output_pulse_o(nxt_output_pulse_o),
    .cm_config_finished_o(cm_config_finished_o)
);

// ---- pool access: logical word address -> (bank = a[1:0], word = a>>2) ------
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
    case (a[1:0])
        2'd0: dut.u_m0_data_mem.mem[a >> 2] = d;
        2'd1: dut.u_m1_data_mem.mem[a >> 2] = d;
        2'd2: dut.u_m2_data_mem.mem[a >> 2] = d;
        default: dut.u_m3_data_mem.mem[a >> 2] = d;
    endcase
endtask

// ---- zero every array and every output register (bram_* have no initial) ----
integer mi;
task clear_all_mems;
    begin
        for (mi = 0; mi < 1024; mi = mi + 1) dut.u_prog_mem.mem[mi] = 32'h0;
        for (mi = 0; mi < 1024; mi = mi + 1) dut.u_cfg_mem.mem[mi]  = 32'h0;
        for (mi = 0; mi < 64;   mi = mi + 1) dut.u_bba_mem.mem[mi]  = 32'h0;
        for (mi = 0; mi < MEM_DEPTH; mi = mi + 1) begin
            dut.u_s0_weight_mem.mem[mi] = 0; dut.u_s0_bias_curr_mem.mem[mi] = 0;
            dut.u_s0_thresh_mem.mem[mi] = 0; dut.u_s0_pot_mem.mem[mi] = 0;
            dut.u_s1_weight_mem.mem[mi] = 0; dut.u_s1_bias_curr_mem.mem[mi] = 0;
            dut.u_s1_thresh_mem.mem[mi] = 0; dut.u_s1_pot_mem.mem[mi] = 0;
            dut.u_a0_weight_mem.mem[mi] = 0; dut.u_a0_bias_curr_mem.mem[mi] = 0;
            dut.u_a0_thresh_mem.mem[mi] = 0; dut.u_a0_pot_mem.mem[mi] = 0;
            dut.u_m0_data_mem.mem[mi] = 0; dut.u_m1_data_mem.mem[mi] = 0;
            dut.u_m2_data_mem.mem[mi] = 0; dut.u_m3_data_mem.mem[mi] = 0;
        end
        dut.u_cfg_mem.dout = 0; dut.u_bba_mem.douta = 0; dut.u_bba_mem.doutb = 0;
        dut.u_s0_weight_mem.dout = 0; dut.u_s0_bias_curr_mem.dout = 0;
        dut.u_s0_thresh_mem.dout = 0; dut.u_s0_pot_mem.dout = 0;
        dut.u_s1_weight_mem.dout = 0; dut.u_s1_bias_curr_mem.dout = 0;
        dut.u_s1_thresh_mem.dout = 0; dut.u_s1_pot_mem.dout = 0;
        dut.u_a0_weight_mem.dout = 0; dut.u_a0_bias_curr_mem.dout = 0;
        dut.u_a0_thresh_mem.dout = 0; dut.u_a0_pot_mem.dout = 0;
        dut.u_m0_data_mem.dout = 0; dut.u_m1_data_mem.dout = 0;
        dut.u_m2_data_mem.dout = 0; dut.u_m3_data_mem.dout = 0;
    end
endtask

// ---- AXI helper (same single-cycle write as tb_flexman) ---------------------
task axi_write;
    input [31:0] addr;
    input [31:0] data;
    begin
        @(posedge clk); #1;
        sys_req_i = 1'b1; sys_addr_i = addr; sys_data_i = data;
        @(posedge clk); #1;
        sys_req_i = 1'b0; sys_addr_i = 32'h0; sys_data_i = 32'h0;
    end
endtask

// ---- images -------------------------------------------------------------------
reg [31:0] pool_img [0:NPOOL-1];
reg [31:0] gold_img [0:`OUT_WORDS-1];
integer k, errors, shown;

// ---- cycle counter / window ---------------------------------------------------
integer cyc;                 // cycles since reset release
integer wstart, wend;
initial begin cyc = 0; wstart = -1; wend = -1; end
always @(posedge clk) if (!reset) cyc <= cyc + 1;

`ifndef GATE_SIM
always @(posedge clk) if (!reset) begin
    if (wstart < 0 && dut.u_flexman.sch_start_new_block &&
        dut.u_flexman.sch_target_acc == 3'd`ANN_ACC_ID) wstart = cyc;
    if (wstart >= 0 && wend < 0 && dut.u_flexman.u_ann.acc_finished_o) wend = cyc;
end
`else
 `include `WINFILE
`endif

// Port-level activity counters over the window (identical in RTL and gate runs)
integer w_wt_rd, w_pool_wr, w_pool_rd;
initial begin w_wt_rd = 0; w_pool_wr = 0; w_pool_rd = 0; end
`ifdef GATE_SIM
wire in_win = (cyc > `WIN_START) && (cyc <= `WIN_END);
`else
wire in_win = (wstart >= 0) && (wend < 0);
`endif
always @(posedge clk) if (!reset && in_win) begin
    if (dut.a0_weight_mem_rd_o) w_wt_rd = w_wt_rd + 1;
    if (dut.m0_data_mem_wr_o | dut.m1_data_mem_wr_o | dut.m2_data_mem_wr_o | dut.m3_data_mem_wr_o)
        w_pool_wr = w_pool_wr + 1;
    if (dut.m0_data_mem_rd_o | dut.m1_data_mem_rd_o | dut.m2_data_mem_rd_o | dut.m3_data_mem_rd_o)
        w_pool_rd = w_pool_rd + 1;
end

`ifdef GATE_SAIF
// $stop at the window edges: the Tcl driver dumps SAIF between the two stops.
always @(posedge clk) if (!reset) begin
    if (cyc == `WIN_START) begin #1; $display("[SAIF] window open  cyc=%0d t=%0t", cyc, $time); $stop; end
    if (cyc == `WIN_END)   begin #1; $display("[SAIF] window close cyc=%0d t=%0t", cyc, $time); $stop; end
end
`endif


`ifdef GATE_DBG
always @(posedge clk) if (!reset && (cyc < 80 || cyc % 2000 == 0))
    $display("[DBG] cyc=%0d ack=%b req=%b prog_addr=%h prog_data=%h cm_fin=%b nxt_in=%b a0_wt_rd=%b cfg_rd=%b cfg_addr=%h bba_rd=%b",
        cyc, sys_ack_o, sys_req_i, dut.prog_mem_addr_o, dut.prog_mem_data_i, cm_config_finished_o,
        nxt_input_pulse_o, dut.a0_weight_mem_rd_o, dut.cfg_mem_rd_o, dut.cfg_mem_addr_o, dut.bba_mem_rd_o);
`endif

// ---- stimulus -------------------------------------------------------------------
integer guard;
initial begin
    reset = 1'b1; sys_req_i = 1'b0; sys_addr_i = 32'h0; sys_data_i = 32'h0;
    clear_all_mems;
`ifdef GATE_SIM
    #1;   // after the models' own time-0 initial blocks
    `include `GINITFILE
`endif
    $readmemh({`IMG, "prog.hex"},    dut.u_prog_mem.mem);
    $readmemh({`IMG, "cfg.hex"},     dut.u_cfg_mem.mem);
    $readmemh({`IMG, "weights.hex"}, dut.u_a0_weight_mem.mem);
    $readmemh({`IMG, "pool_in.hex"}, pool_img);
    $readmemh({`IMG, "golden.hex"},  gold_img);
    for (k = 0; k < NPOOL; k = k + 1) pool_wr(k, pool_img[k]);

    repeat (4) @(posedge clk); #1;
    reset = 1'b0;

    // boot-only annAcc registers, the input buffer FULL, then LOAD_PC + START
    `ANN_BOOT_WRITES
    axi_write(32'hE050_0000, {24'd0, 4'd1, 4'(`BUF_IN)});   // {count=1, id=BUF_IN}
    axi_write(32'hE000_0000, 32'd0);                         // LOAD_PC = 0
    axi_write(32'hE010_0000, 32'd0);                         // START

    // wait for completion (RTL: window end; gate: the preset window end) + margin
    guard = 0;
`ifdef GATE_SIM
    while (cyc < `WIN_END + 200 && guard < 400000) begin @(posedge clk); guard = guard + 1; end
`else
    while ((wend < 0) && guard < 400000) begin @(posedge clk); guard = guard + 1; end
    repeat (200) @(posedge clk);
`endif

    // ---- check ------------------------------------------------------------------
    errors = 0; shown = 0;
    for (k = 0; k < `OUT_WORDS; k = k + 1)
        if (pool_rd(`OUT_BASE + k) !== gold_img[k]) begin
            errors = errors + 1;
            if (shown < 8) begin
                shown = shown + 1;
                $display("[MISMATCH] out word %0d: got %08h exp %08h", k, pool_rd(`OUT_BASE + k), gold_img[k]);
            end
        end
`ifndef GATE_SIM
    $display("[WINDOW] start=%0d end=%0d cycles=%0d", wstart, wend, wend - wstart);
    begin : wf
        integer fd;
        fd = $fopen({`IMG, `WINFILE}, "w");
        $fwrite(fd, "`define WIN_START %0d\n`define WIN_END %0d\n", wstart, wend);
        $fclose(fd);
    end
`else
    $display("[WINDOW] replayed start=%0d end=%0d", `WIN_START, `WIN_END);
`endif
    $display("[ACTIVITY] weight_rd=%0d pool_rd=%0d pool_wr=%0d (window)", w_wt_rd, w_pool_rd, w_pool_wr);
    if (errors == 0 && guard < 400000)
        $display("[CHECK] PASS  %0dx%0d layer, %0d output words match golden  (NLANE=%0d)",
                 `NIN, `NOUT, `OUT_WORDS, `ANN_NLANE);
    else
        $display("[CHECK] FAIL  %0d mismatching words (guard=%0d)", errors, guard);
    #20 $finish;
end

initial begin
    #200000000;   // 2 ms @ 10 ps units guard
    $display("[TIMEOUT]");
    $finish;
end

endmodule
