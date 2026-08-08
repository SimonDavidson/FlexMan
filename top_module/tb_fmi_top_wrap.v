// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
// Authors: Simon Davidson & Claude | Created 2026-08-08 | Last modified 2026-08-08
`timescale 10ps/1ps
`include "../shared/constants.v"

// =============================================================================
// tb_fmi_top_wrap — first simulation of fmi_top_fpga_wrap (the SYNTHESIS top)
//
// WHY THIS EXISTS
// tb_fmi_top.v drives the `fmi_top` CORE and deliberately splits its memory
// backing: prog/cfg/bba are behavioural arrays with COMBINATIONAL reads, while
// the 9 acc memories and 4 pool banks are 1-cycle sram_models. Vivado
// synthesises fmi_top_fpga_wrap, where cfg and bba are REGISTERED bram_sdp.
// Nothing had ever simulated fmi_top's config_manager against registered config
// reads — that is the main new coverage here, and the most likely place for a
// wrapper-only bug.
//
// Reproduces tb_fmi_top's T9B and T9A golden runs verbatim (same vectors, same
// 1920 syn_curr + 60 spike comparisons via both the POOL_RD_BASE AXI window and
// the backdoor). Differences forced by driving the wrapper:
//   1. 14 ports instead of ~120 — every memory is now INSIDE the DUT.
//   2. Preload by backdoor into dut.u_<name>.mem (every bram_*.v names it `mem`).
//   3. bram_*.v has NO `initial` block (sram_model does), so arrays AND dout
//      registers must be zeroed explicitly or X wedges the design.
//   4. cfg/bba reads are 1-cycle registered, not combinational.
//
// NOTE: fmi_top has NO fill_unit (NUM_HW_ACCELERATORS=2), so there is no
// fu_bba path and the bba wait-state bug that affects the other wrappers does
// not apply here. u_bba_mem is a plain bram_sdp and its contents are
// don't-care for these tests (fmiSnnMC's buff_addr ports are dead; all
// addressing comes from the config base words).
//
// ⚠️ FINDING — THE ld_* LOADER STUB IS NOT USABLE AS A LOADER (2026-08-08)
// ld_we_i / ld_addr_i / ld_data_i fan out to ALL SEVEN read-only accelerator
// memories with the SAME we, the SAME address and the SAME data (see
// fmi_top_fpga_wrap.v:307-333). There is no per-memory select, so a host
// CANNOT load distinct images into weight / thresh / dcy_syn / dcy_mem / b_eff
// / dcy_ada / scl_ada — every write goes to all seven at once. T_LD below
// documents this behaviour rather than asserting it is correct. RAISED FOR
// DECISION: the port needs a memory-select field before it can load a real
// network, and this TB preloads by backdoor in the meantime.
//
// Usage: bash sim_fmi_top_wrap.bsh     (run from top_module/ — the golden
//        vectors are read via ../../fmi/mem_files/... relative paths)
// =============================================================================
module tb_fmi_top_wrap;

localparam PROG_DATA_BITS = 32;

// ─── Wrapper region depths — fmi_top_fpga_wrap DEFAULTS, not overridden, so
//     this TB exercises exactly the configuration Vivado builds. ACC_MEM_DEPTH
//     is 4096, which matches tb_fmi_top's DATA_DEPTH exactly.
localparam ACC_MEM_DEPTH  = 4096;
localparam PROG_MEM_DEPTH = 1024;
localparam CFG_MEM_DEPTH  = 1024;
localparam BBA_MEM_DEPTH  = 64;

// ─── Address map (verbatim from tb_fmi_top.v) ────────────────────────────────
localparam [31:0] FMI_CFG_BASE = 32'h1000_0000;
localparam [31:0] POOL_RD_BASE = 32'h1010_0000;
localparam [31:0] SCH_LOAD_PC  = 32'hE000_0000;
localparam [31:0] SCH_START    = 32'hE010_0000;
localparam [31:0] SCH_MARKFULL = 32'hE050_0000;
localparam STOP_INST = 32'h0000_0002;

// ─── Pool buffer layout (verbatim) ───────────────────────────────────────────
localparam ACT_BASE      = 0;
localparam SPIKE_BASE    = 64;
localparam SYN_CURR_BASE = 128;

integer errors = 0;

// ─── Clock / reset ───────────────────────────────────────────────────────────
reg clk = 1'b0;
reg reset = 1'b1;
reg test_stall_pipe = 1'b0;
always #5 clk = ~clk;

// ─── DUT interface (the wrapper's entire port list: 14 signals) ──────────────
reg         sys_req_i  = 1'b0;
reg  [31:0] sys_addr_i = 32'h0;
reg  [31:0] sys_data_i = 32'h0;
wire        sys_ack_o;
wire [31:0] sys_data_o;
wire        nxt_input_pulse_o, nxt_output_pulse_o, cm_config_finished_o;

// Loader stub — held inactive except during T_LD.
reg                  ld_we_i   = 1'b0;
reg [`ADDR_SIZE-1:0] ld_addr_i = 0;
reg           [31:0] ld_data_i = 32'h0;

// ─── DUT: the SYNTHESIS top ──────────────────────────────────────────────────
fmi_top_fpga_wrap dut (
    .clk(clk), .reset(reset), .test_stall_pipe(test_stall_pipe),
    .sys_req_i(sys_req_i), .sys_ack_o(sys_ack_o), .sys_addr_i(sys_addr_i),
    .sys_data_i(sys_data_i), .sys_data_o(sys_data_o),
    .nxt_input_pulse_o(nxt_input_pulse_o), .nxt_output_pulse_o(nxt_output_pulse_o),
    .cm_config_finished_o(cm_config_finished_o),
    .ld_we_i(ld_we_i), .ld_addr_i(ld_addr_i), .ld_data_i(ld_data_i)
);

// ─── Pool backdoor: logical addr -> (bank=a[1:0], word=a>>2) ─────────────────
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

// ─── Zero every array AND every dout register ────────────────────────────────
// MANDATORY: bram_*.v does not self-initialise; tb_fmi_top's sram_model does.
// Per-primitive rule:
//   bram_dist (u_prog_mem) — dout is a continuous assign; zero the array only.
//   bram_sdp / bram_sp     — `output reg dout` -> zero .dout as well.
integer mi;
task clear_all_mems;
    begin
        for (mi = 0; mi < PROG_MEM_DEPTH; mi = mi + 1) dut.u_prog_mem.mem[mi] = 32'h0;
        for (mi = 0; mi < CFG_MEM_DEPTH;  mi = mi + 1) dut.u_cfg_mem.mem[mi]  = 32'h0;
        for (mi = 0; mi < BBA_MEM_DEPTH;  mi = mi + 1) dut.u_bba_mem.mem[mi]  = 32'h0;
        for (mi = 0; mi < ACC_MEM_DEPTH;  mi = mi + 1) begin
            dut.u_weight_mem.mem[mi]   = 0;
            dut.u_thresh_mem.mem[mi]   = 0;
            dut.u_dcy_syn_mem.mem[mi]  = 0;
            dut.u_dcy_mem_mem.mem[mi]  = 0;
            dut.u_b_eff_mem.mem[mi]    = 0;
            dut.u_dcy_ada_mem.mem[mi]  = 0;
            dut.u_scl_ada_mem.mem[mi]  = 0;
            dut.u_pot_mem.mem[mi]      = 0;
            dut.u_ada_mem.mem[mi]      = 0;
            dut.u_m0_data_mem.mem[mi]  = 0;
            dut.u_m1_data_mem.mem[mi]  = 0;
            dut.u_m2_data_mem.mem[mi]  = 0;
            dut.u_m3_data_mem.mem[mi]  = 0;
        end
        dut.u_cfg_mem.dout      = 0;
        dut.u_bba_mem.dout      = 0;
        dut.u_weight_mem.dout   = 0;
        dut.u_thresh_mem.dout   = 0;
        dut.u_dcy_syn_mem.dout  = 0;
        dut.u_dcy_mem_mem.dout  = 0;
        dut.u_b_eff_mem.dout    = 0;
        dut.u_dcy_ada_mem.dout  = 0;
        dut.u_scl_ada_mem.dout  = 0;
        dut.u_pot_mem.dout      = 0;
        dut.u_ada_mem.dout      = 0;
        dut.u_m0_data_mem.dout  = 0;
        dut.u_m1_data_mem.dout  = 0;
        dut.u_m2_data_mem.dout  = 0;
        dut.u_m3_data_mem.dout  = 0;
    end
endtask

// ─── AXI host tasks (verbatim from tb_fmi_top.v) ─────────────────────────────
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
task axi_read;
    input  [31:0] addr;
    output [31:0] data;
    integer guard;
    begin
        @(posedge clk); #1;
        sys_req_i = 1'b1; sys_addr_i = addr; sys_data_i = 32'h0;
        guard = 0;
        @(posedge clk); #1;
        while (sys_ack_o !== 1'b1 && guard < 1000) begin
            @(posedge clk); #1; guard = guard + 1;
        end
        data = sys_data_o;
        sys_req_i = 1'b0; sys_addr_i = 32'h0;
    end
endtask

// Wait for the accelerator to dispatch then finish (poll DUT busy, now one
// level deeper: dut.u_fmi_top rather than u_dut).
task wait_acc_done;
    integer guard;
    begin
        guard = 0;
        while (!dut.u_fmi_top.fmi_busy && guard < 200000) begin @(posedge clk); guard = guard + 1; end
        if (!dut.u_fmi_top.fmi_busy) begin
            $display("  FAIL: accelerator never dispatched"); errors = errors + 1;
        end else begin
            guard = 0;
            while (dut.u_fmi_top.fmi_busy && guard < 4000000) begin @(posedge clk); guard = guard + 1; end
            if (dut.u_fmi_top.fmi_busy) begin
                $display("  FAIL: accelerator did not finish (timeout)"); errors = errors + 1;
            end else
                $display("  acc finished after %0d cycles", guard);
        end
        repeat(8) @(posedge clk);
    end
endtask

// ─── Golden + scratch arrays ─────────────────────────────────────────────────
reg [31:0] act_buf     [0:59];
reg [31:0] golden_syn  [0:1919];
reg [31:0] golden_spike[0:59];

integer i;

task load_static_mems;
    begin
        $readmemh("../../fmi/mem_files/recurrent/con2_weight.hex",    dut.u_weight_mem.mem);
        $readmemh("../../fmi/mem_files/recurrent/group4_dcy_syn.hex", dut.u_dcy_syn_mem.mem);
        $readmemh("../../fmi/mem_files/recurrent/group4_dcy_mem.hex", dut.u_dcy_mem_mem.mem);
        $readmemh("../../fmi/mem_files/recurrent/group4_thresh.hex",  dut.u_thresh_mem.mem);
    end
endtask

// 16-word PACKED config for cfg_id 0 (T9 values), backdoored into the cfg BRAM.
task load_config;
    begin
        dut.u_cfg_mem.mem[ 0] = ACT_BASE;
        dut.u_cfg_mem.mem[ 1] = 32'd0;
        dut.u_cfg_mem.mem[ 2] = SYN_CURR_BASE;
        dut.u_cfg_mem.mem[ 3] = 32'd0;
        dut.u_cfg_mem.mem[ 4] = 32'd0;
        dut.u_cfg_mem.mem[ 5] = SPIKE_BASE;
        dut.u_cfg_mem.mem[ 6] = 32'd0;
        dut.u_cfg_mem.mem[ 7] = 32'd0;
        dut.u_cfg_mem.mem[ 8] = 32'd0;
        dut.u_cfg_mem.mem[ 9] = 32'd0;
        dut.u_cfg_mem.mem[10] = 32'd0;
        dut.u_cfg_mem.mem[11] = 32'd0;
        dut.u_cfg_mem.mem[12] = {16'd60, 16'd120};
        dut.u_cfg_mem.mem[13] = {16'd1919, 16'h0045};
        dut.u_cfg_mem.mem[14] = 32'h2010_0801;
        dut.u_cfg_mem.mem[15] = 32'h2555_0040;
        for (i = 0; i < 8; i = i + 1) dut.u_bba_mem.mem[i] = 32'd0;
    end
endtask

task write_boot_regs;
    begin
        axi_write(FMI_CFG_BASE | 32'h5C, 32'd12);  // weight_idx_sz
    end
endtask

task clear_state;
    begin
        for (i = 0; i < 1920; i = i + 1) pool_wr(SYN_CURR_BASE + i, 32'd0);
        for (i = 0; i < 60;   i = i + 1) pool_wr(SPIKE_BASE   + i, 32'd0);
        for (i = 0; i < ACC_MEM_DEPTH; i = i + 1) begin
            dut.u_pot_mem.mem[i] = 32'd0;
            dut.u_ada_mem.mem[i] = 32'd0;
        end
    end
endtask

task compare_results;
    input [255:0] tag;
    integer axi_mism, bd_mism;
    reg [31:0] got_axi, got_bd, logical;
    begin
        axi_mism = 0; bd_mism = 0;
        for (i = 0; i < 1920; i = i + 1) begin
            logical = SYN_CURR_BASE + i;
            axi_read(POOL_RD_BASE + (logical << 2), got_axi);
            got_bd = pool_rd(logical);
            if (got_axi !== golden_syn[i]) begin
                if (axi_mism < 8) $display("  %0s syn AXI MISMATCH n=%0d got %08h exp %08h",
                                            tag, i, got_axi, golden_syn[i]);
                axi_mism = axi_mism + 1;
            end
            if (got_bd !== golden_syn[i]) bd_mism = bd_mism + 1;
        end
        if (axi_mism == 0 && bd_mism == 0)
            $display("  OK  %0s syn_curr: 1920/1920 match (AXI + backdoor)", tag);
        else begin
            $display("  FAIL %0s syn_curr: AXI %0d, backdoor %0d mismatches", tag, axi_mism, bd_mism);
            errors = errors + 1;
        end

        axi_mism = 0; bd_mism = 0;
        for (i = 0; i < 60; i = i + 1) begin
            logical = SPIKE_BASE + i;
            axi_read(POOL_RD_BASE + (logical << 2), got_axi);
            got_bd = pool_rd(logical);
            if (got_axi !== golden_spike[i]) begin
                if (axi_mism < 8) $display("  %0s spike AXI MISMATCH w=%0d got %08h exp %08h",
                                            tag, i, got_axi, golden_spike[i]);
                axi_mism = axi_mism + 1;
            end
            if (got_bd !== golden_spike[i]) bd_mism = bd_mism + 1;
        end
        if (axi_mism == 0 && bd_mism == 0)
            $display("  OK  %0s spike: 60/60 match (AXI + backdoor)", tag);
        else begin
            $display("  FAIL %0s spike: AXI %0d, backdoor %0d mismatches", tag, axi_mism, bd_mism);
            errors = errors + 1;
        end
    end
endtask

task run_variant;
    input [255:0]  tag;
    input [1023:0] act_file;
    input [1023:0] syn_gold_file;
    input [1023:0] spk_gold_file;
    begin
        $display("[%0s] scheduler-driven T9 reproduction (WRAPPER)", tag);
        reset = 1'b1;
        repeat(4) @(posedge clk); #1;
        // Zero everything while held in reset — the wrapper's BRAMs have no
        // initial block, so this must happen before the first read.
        clear_all_mems;
        reset = 1'b0;
        @(posedge clk); #1;

        load_static_mems;
        clear_state;
        $readmemh(act_file,      act_buf);
        $readmemh(syn_gold_file, golden_syn);
        $readmemh(spk_gold_file, golden_spike);
        for (i = 0; i < 60; i = i + 1) pool_wr(ACT_BASE + i, act_buf[i]);

        load_config;
        write_boot_regs;
        axi_write(SCH_MARKFULL, 32'h0000_0010);

        dut.u_prog_mem.mem[0] = 32'h0000_2000;
        dut.u_prog_mem.mem[1] = 32'h11C0_0000;
        dut.u_prog_mem.mem[2] = STOP_INST;

        axi_write(SCH_LOAD_PC, 32'd0);
        axi_write(SCH_START,   32'd0);

        wait_acc_done;
        compare_results(tag);
    end
endtask

// ─── T_LD: the ld_* loader stub — NEVER SIMULATED BEFORE ─────────────────────
// This does not assert that the loader is correct; it CHARACTERISES it. The
// three ld_* signals fan out to all seven read-only memories with a shared we,
// address and data, so one write lands in all seven. The test confirms that is
// what actually happens, which is the evidence for the finding in the header.
task test_ld_port;
    reg [31:0] probe;
    integer hits;
    begin
        $display("[T_LD] ld_* loader stub characterisation");
        probe = 32'hABCD_1234;
        hits  = 0;

        reset = 1'b1;
        repeat(2) @(posedge clk); #1;
        clear_all_mems;
        reset = 1'b0;
        @(posedge clk); #1;

        // One loader write to address 100.
        @(posedge clk); #1;
        ld_we_i = 1'b1; ld_addr_i = 100; ld_data_i = probe;
        @(posedge clk); #1;
        ld_we_i = 1'b0; ld_addr_i = 0;   ld_data_i = 32'h0;
        @(posedge clk); #1;

        if (dut.u_weight_mem.mem[100]  === probe[`WTD_BITS-1:0]) hits = hits + 1;
        if (dut.u_thresh_mem.mem[100]  === probe[`WTD_BITS-1:0]) hits = hits + 1;
        if (dut.u_dcy_syn_mem.mem[100] === probe) hits = hits + 1;
        if (dut.u_dcy_mem_mem.mem[100] === probe) hits = hits + 1;
        if (dut.u_b_eff_mem.mem[100]   === probe) hits = hits + 1;
        if (dut.u_dcy_ada_mem.mem[100] === probe) hits = hits + 1;
        if (dut.u_scl_ada_mem.mem[100] === probe) hits = hits + 1;

        if (hits == 7)
            $display("  OK  ld_* write reached all 7 memories (BROADCAST — see header finding: no per-memory select, so distinct images cannot be loaded)");
        else if (hits == 0) begin
            $display("  FAIL ld_* write reached NO memory — the loader port is dead");
            errors = errors + 1;
        end else begin
            $display("  FAIL ld_* write reached %0d of 7 memories (expected all 7 given the shared we/addr/din)", hits);
            errors = errors + 1;
        end
    end
endtask

// ─── Main ────────────────────────────────────────────────────────────────────
initial begin
    repeat(4) @(posedge clk);

    test_ld_port;

    run_variant("T9B",
                "../../fmi/mem_files/recurrent/test_inputs/act_input_B.hex",
                "../../fmi/mem_files/recurrent/test_inputs/syn_curr_golden_B.hex",
                "../../fmi/mem_files/recurrent/test_inputs/spike_golden_B.hex");

    run_variant("T9A",
                "../../fmi/mem_files/recurrent/test_inputs/act_input_A.hex",
                "../../fmi/mem_files/recurrent/test_inputs/syn_curr_golden_A.hex",
                "../../fmi/mem_files/recurrent/test_inputs/spike_golden_A.hex");

    $display("=== tb_fmi_top_wrap: %0d failure(s) ===", errors);
    if (errors == 0) $display("[VERDICT] PASS  fmi_top_fpga_wrap: T9A + T9B golden match, ld_* characterised");
    else             $display("[VERDICT] FAIL  %0d failure(s)", errors);
    #20 $finish;
end

// safety net
initial begin
    #2_000_000_000;
    $display("[VERDICT] TIMEOUT");
    $finish;
end

endmodule // tb_fmi_top_wrap
