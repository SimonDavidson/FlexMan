// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
// =============================================================================
// tb_packer_cadence  (fmiSnnAccMC)  --  dedicated multi-packer cadence test (F8)
//
// Authors      : Simon Davidson & Claude
// Created      : 2026-06-21
// Last modified: 2026-06-21
//
// Purpose
// -------
// Confirm / characterise F8 (verif/FINDINGS.md): "sub-32-bit slice writeback
// corrupts fields" in neuron_processing's multi-packer writeback path.
//
// This test reproduces neuron_processing's writeback EXACTLY but in isolation:
//   * the REAL update_state_for_neuron as the producer (true result_valid /
//     result_taken / neuron_taken handshake and FSM cadence),
//   * a neuron counter that advances on result_taken (= neuron_taken),
//   * the REAL packer x4 wired identically to neuron_processing
//       - spike  : 1-bit fields, 32 neurons/word   (pak_out_sz = 0)
//       - syn_wb : <run-time> fields/word          (pak_out_sz = syn_sz)
//       - pot_wb : <run-time> fields/word          (pak_out_sz = pot_sz)
//       - ada_wb : 32-bit fields, 1 neuron/word    (pak_out_sz = 5)
//   * wait-HONOURING writeback SRAMs (write commits only when ~mem_wait), so we
//     can inject back-pressure on the WRITE side (which the existing
//     tb_neuron_processing never does — it holds all write waits at 0).
//
// The caches are intentionally omitted: read-side latency only ADDS inter-neuron
// gap (more forgiving), whereas F8 is a WRITE-side cadence effect.  neuron_valid
// is held ready so the FSM runs back-to-back (tightest cadence).
//
// Golden: per packer, OR each neuron's left-justified field contribution into
// its destination word (the proven tb_packer field model).  This is timing- and
// cadence-INDEPENDENT: if the RTL packs each neuron into the correct word/field,
// memory matches regardless of how write waits perturb the schedule.
//
// Status: F8 FIXED 2026-06-21.  This test now exercises the FIXED wiring
// (use_taken=1 -> pak_write_i = result_taken, the one-cycle accept that all
// five neuron_processing variants adopted) across the full slice-size matrix
// under heavy write-back back-pressure.  The original as-built (result_valid)
// negative control gave 70 failures here; it is documented in FINDINGS.md and
// not run (check_eq_u prints a bare FAIL token that run_regression.sh greps).
// The use_taken mux is retained so the as-built path can be re-checked by hand.
// =============================================================================
`timescale 1ns/1ps
`include "../shared/constants.v"

module tb_packer_cadence;

    localparam NEURON_IDX_SZ = 7;           // up to 127 neurons
    localparam SZ8  = 3'b011, SZ16 = 3'b100, SZ32 = 3'b101;

    integer verif_errors, verif_checks, verif_to, sram_i;
    `include "../verif/checks.vh"
    `include "../verif/np_ref.vh"

    reg clk, reset;
    initial clk = 0; always #5 clk = ~clk;

    // -------------------------------------------------------------------------
    // Config / control
    // -------------------------------------------------------------------------
    reg                      start;
    reg                      running_r;
    reg [NEURON_IDX_SZ-1:0]  neuron_counter_r;
    reg [NEURON_IDX_SZ-1:0]  last_neuron_idx;
    reg [2:0]                syn_sz, pot_sz;
    reg                      has_ada;
    reg [`ADDR_SIZE-1:0]     syn_base, pot_base, spike_base, ada_base;
    reg [4:0]                binpt;        // bin_point_syn_curr (kept 0 here)
    reg [1:0]                wmode;        // 0=no wait, 1=30% rand, 2=60% rand
    reg                      use_taken;    // 0 = as-built (result_valid strobe)
                                           // 1 = proposed fix (result_taken strobe)

    // per-neuron inputs (drive update_state)
    reg signed [31:0] in_syn[0:127], in_pot[0:127], in_thr[0:127], in_beff[0:127];
    reg        [31:0] in_dsyn[0:127], in_dmem[0:127], in_ada[0:127], in_dada[0:127], in_sada[0:127];

    // golden per-neuron results
    reg               g_spk[0:127];
    reg signed [31:0] g_pot[0:127], g_syn[0:127];
    reg        [31:0] g_ada[0:127];

    wire last_neuron = (neuron_counter_r == last_neuron_idx);

    // -------------------------------------------------------------------------
    // Producer: REAL update_state_for_neuron  (32-bit datapath)
    // -------------------------------------------------------------------------
    wire               result_valid, neuron_taken_unused, spike;
    wire signed [31:0] upd_pot, upd_syn;
    wire        [31:0] upd_ada;
    wire               packer_full, syn_wb_full, pot_wb_full, ada_wb_full;

    wire neuron_valid = running_r & (neuron_counter_r <= last_neuron_idx);

    // Advance only when EVERY packer can accept (mirrors neuron_processing).
    wire result_taken = result_valid & ~packer_full & ~syn_wb_full &
                        ~pot_wb_full & ~ada_wb_full;

    update_state_for_neuron #(
        .SYN_CURR_SLICE_BITS(32), .POT_SLICE_BITS(32), .THRESH_SLICE_BITS(32)
    ) prod (
        .clk(clk), .reset(reset),
        .neuron_valid_i(neuron_valid),
        .syn_curr_i (in_syn [neuron_counter_r]),
        .potential_i(in_pot [neuron_counter_r]),
        .threshold_i(in_thr [neuron_counter_r]),
        .syn_dcy_i  (in_dsyn[neuron_counter_r]),
        .mem_dcy_i  (in_dmem[neuron_counter_r]),
        .has_ada_i  (has_ada),
        .ada_i      (in_ada [neuron_counter_r]),
        .b_eff_i    (in_beff[neuron_counter_r]),
        .dcy_ada_i  (in_dada[neuron_counter_r]),
        .scl_ada_i  (in_sada[neuron_counter_r]),
        .neuron_taken_o(neuron_taken_unused),
        .result_valid_o(result_valid),
        .potential_o(upd_pot),
        .syn_curr_o (upd_syn),
        .spike_o    (spike),
        .ada_o      (upd_ada),
        .result_taken_i(result_taken)
    );

    // neuron counter + running flag
    always @(posedge clk) begin
        if (reset | start)           neuron_counter_r <= 'b0;
        else if (result_taken)       neuron_counter_r <= neuron_counter_r + 1'b1;
    end
    always @(posedge clk) begin
        if (reset)                                   running_r <= 1'b0;
        else if (start)                              running_r <= 1'b1;
        else if (result_taken & last_neuron)         running_r <= 1'b0;
    end

    // Monitor: count accepted neurons (result_taken pulses) per run. The whole
    // fix rests on "exactly one accumulate per accepted neuron", so this must
    // equal N — a held strobe that re-accumulated would over-count here too.
    reg [31:0] accepts;
    always @(posedge clk) begin
        if (reset | start)     accepts <= 32'b0;
        else if (result_taken) accepts <= accepts + 1'b1;
    end

    // -------------------------------------------------------------------------
    // Left-justify helper (matches neuron_processing's syn_curr_wb_lj/pot_wb_lj)
    // -------------------------------------------------------------------------
    function [31:0] leftjust;
        input [31:0] v; input [2:0] sz;
        begin
            case (sz)
                3'b000: leftjust = {v[0],    31'b0};
                3'b001: leftjust = {v[1:0],  30'b0};
                3'b010: leftjust = {v[3:0],  28'b0};
                3'b011: leftjust = {v[7:0],  24'b0};
                3'b100: leftjust = {v[15:0], 16'b0};
                3'b101: leftjust = v;
                default: leftjust = 32'b0;
            endcase
        end
    endfunction

    wire signed [31:0] updated_syn_wb = $signed(upd_syn) <<< binpt;
    wire        [31:0] syn_curr_wb_lj = leftjust(updated_syn_wb, syn_sz);
    wire        [31:0] pot_wb_lj      = leftjust(upd_pot,        pot_sz);

    // -------------------------------------------------------------------------
    // Writeback memory buses + REAL packers (wired as in neuron_processing)
    // -------------------------------------------------------------------------
    wire                  spike_wr;  wire [`ADDR_SIZE-1:0] spike_addr;  wire [`ACT_BITS-1:0] spike_data;
    wire                  syn_wr;    wire [`ADDR_SIZE-1:0] syn_addr;    wire [31:0]          syn_data;
    wire                  pot_wr;    wire [`ADDR_SIZE-1:0] pot_addr;    wire [31:0]          pot_data;
    wire                  ada_wr;    wire [`ADDR_SIZE-1:0] ada_addr;    wire [31:0]          ada_data;
    reg                   spike_wait, syn_wait, pot_wait, ada_wait;

    wire [`PIN_BITS-1:0]  pak_index = {{(`PIN_BITS-NEURON_IDX_SZ){1'b0}}, neuron_counter_r};

    // Strobe under test: as-built drives the packers from the HELD result_valid
    // level; the proposed fix drives them from the one-cycle result_taken accept.
    wire pak_write     = use_taken ? result_taken            : result_valid;
    wire pak_write_ada = use_taken ? (result_taken & has_ada): (result_valid & has_ada);

    packer spike_packer0 (
        .clk(clk), .reset(reset), .busy_o(), .finish_o(),
        .pak_write_i(pak_write), .pak_full_o(packer_full),
        .pak_colour_i(1'b0), .pak_last_i(last_neuron),
        .pak_index_i(pak_index), .pak_acc_data_i({spike, {(`POT_BITS-1){1'b0}}}),
        .pot_wr_o(spike_wr), .pot_wait_i(spike_wait),
        .pot_addr_o(spike_addr), .pot_data_o(spike_data),
        .pak_colour_sel_o(), .pak_out_sz_i({`POT_OUT_SZ_SZ{1'b0}}),
        .pak_colour_bs_o(), .pak_out_base_addr_i(spike_base));

    packer syn_curr_wb_packer (
        .clk(clk), .reset(reset), .busy_o(), .finish_o(),
        .pak_write_i(pak_write), .pak_full_o(syn_wb_full),
        .pak_colour_i(1'b0), .pak_last_i(last_neuron),
        .pak_index_i(pak_index), .pak_acc_data_i(syn_curr_wb_lj),
        .pot_wr_o(syn_wr), .pot_wait_i(syn_wait),
        .pot_addr_o(syn_addr), .pot_data_o(syn_data),
        .pak_colour_sel_o(), .pak_out_sz_i(syn_sz),
        .pak_colour_bs_o(), .pak_out_base_addr_i(syn_base));

    packer pot_wb_packer (
        .clk(clk), .reset(reset), .busy_o(), .finish_o(),
        .pak_write_i(pak_write), .pak_full_o(pot_wb_full),
        .pak_colour_i(1'b0), .pak_last_i(last_neuron),
        .pak_index_i(pak_index), .pak_acc_data_i(pot_wb_lj),
        .pot_wr_o(pot_wr), .pot_wait_i(pot_wait),
        .pot_addr_o(pot_addr), .pot_data_o(pot_data),
        .pak_colour_sel_o(), .pak_out_sz_i(pot_sz),
        .pak_colour_bs_o(), .pak_out_base_addr_i(pot_base));

    wire ada_wb_full_raw;
    assign ada_wb_full = has_ada & ada_wb_full_raw;
    packer ada_wb_packer (
        .clk(clk), .reset(reset), .busy_o(), .finish_o(),
        .pak_write_i(pak_write_ada), .pak_full_o(ada_wb_full_raw),
        .pak_colour_i(1'b0), .pak_last_i(last_neuron),
        .pak_index_i(pak_index), .pak_acc_data_i(upd_ada),
        .pot_wr_o(ada_wr), .pot_wait_i(ada_wait),
        .pot_addr_o(ada_addr), .pot_data_o(ada_data),
        .pak_colour_sel_o(), .pak_out_sz_i(3'b101),
        .pak_colour_bs_o(), .pak_out_base_addr_i(ada_base));

    // -------------------------------------------------------------------------
    // Wait-HONOURING writeback SRAMs (commit only on wr & ~wait)
    // -------------------------------------------------------------------------
    reg [31:0] spk_sram[0:255], syn_sram[0:255], pot_sram[0:255], ada_sram[0:255];
    always @(posedge clk) begin
        if (spike_wr & ~spike_wait) spk_sram[spike_addr[7:0]] <= spike_data;
        if (syn_wr   & ~syn_wait  ) syn_sram[syn_addr[7:0]]   <= syn_data;
        if (pot_wr   & ~pot_wait  ) pot_sram[pot_addr[7:0]]   <= pot_data;
        if (ada_wr   & ~ada_wait  ) ada_sram[ada_addr[7:0]]   <= ada_data;
    end

    // write-side back-pressure generator
    always @(posedge clk) begin
        case (wmode)
            2'd1: begin
                spike_wait <= ($urandom_range(99) < 30);
                syn_wait   <= ($urandom_range(99) < 30);
                pot_wait   <= ($urandom_range(99) < 30);
                ada_wait   <= ($urandom_range(99) < 30);
            end
            2'd2: begin
                spike_wait <= ($urandom_range(99) < 60);
                syn_wait   <= ($urandom_range(99) < 60);
                pot_wait   <= ($urandom_range(99) < 60);
                ada_wait   <= ($urandom_range(99) < 60);
            end
            default: begin
                spike_wait <= 1'b0; syn_wait <= 1'b0; pot_wait <= 1'b0; ada_wait <= 1'b0;
            end
        endcase
    end

    // -------------------------------------------------------------------------
    // tb_packer field model (proven golden for one packer)
    // -------------------------------------------------------------------------
    function [31:0] g_new_data;
        input [4:0]  idx;
        input [31:0] value;     // left-justified element (MSBs)
        input [2:0]  out_sz;
        reg   [4:0]  shift_mask;
        reg   [31:0] data_mask;
        reg   [5:0]  masked;
        reg   [6:0]  plus, data_shift;
        reg   [63:0] temp;
        integer      w;
        begin
            w          = 1 << out_sz;
            shift_mask = 5'h1F >> out_sz;
            data_mask  = (out_sz == 3'd5) ? 32'hFFFF_FFFF : (32'hFFFF_FFFF << (32 - w));
            masked     = {1'b0, idx} & {1'b0, shift_mask};
            plus       = masked + 7'h01;
            data_shift = (out_sz == 3'd5) ? 7'h20 : (plus << out_sz);
            temp       = ({32'h0, (value & data_mask)} << data_shift);
            g_new_data = temp[63:32];
        end
    endfunction

    function [`ADDR_SIZE-1:0] g_offset;
        input [`PIN_BITS-1:0] idx;
        input [2:0]           out_sz;
        begin g_offset = idx >> (5 - out_sz); end
    endfunction

    // expected memory images
    reg [31:0] e_spk[0:255], e_syn[0:255], e_pot[0:255], e_ada[0:255];

    integer n, k, wcnt;

    function [31:0] rand_q032;
        integer kk;
        begin
            kk = $urandom_range(0, 9);
            case (kk)
                0: rand_q032 = 32'h0000_0000;
                1: rand_q032 = 32'hFFFF_FFFF;
                2: rand_q032 = 32'h8000_0000;
                3: rand_q032 = 32'h0000_0001;
                4: rand_q032 = 32'hFFFF_0000;
                default: rand_q032 = {$urandom_range(0,16'hFFFF), $urandom_range(0,16'hFFFF)};
            endcase
        end
    endfunction

    // Run one layer of N neurons with the given slice sizes and write-wait mode.
    task run_block;
        input integer N;
        input         ha;
        input [2:0]   ssz;
        input [2:0]   psz;
        input [1:0]   wm;
        input [255:0] tag;
        begin
            // reset DUT + packers
            reset = 1'b1; @(posedge clk); #1; @(posedge clk); #1; reset = 1'b0; @(posedge clk); #1;

            for (sram_i=0; sram_i<256; sram_i=sram_i+1) begin
                spk_sram[sram_i]=0; syn_sram[sram_i]=0; pot_sram[sram_i]=0; ada_sram[sram_i]=0;
                e_spk[sram_i]=0;    e_syn[sram_i]=0;    e_pot[sram_i]=0;    e_ada[sram_i]=0;
            end

            for (n = 0; n < N; n = n + 1) begin
                in_syn[n]  = $urandom_range(0,200) - 100;
                in_pot[n]  = $urandom_range(0,200) - 100;
                in_thr[n]  = $urandom_range(0,60)  - 30;
                in_dsyn[n] = rand_q032();
                in_dmem[n] = rand_q032();
                in_ada[n]  = {$urandom_range(0,16'hFFFF), $urandom_range(0,16'hFFFF)};
                in_beff[n] = rand_q032();
                in_dada[n] = rand_q032();
                in_sada[n] = rand_q032();

                np_ref_fmi(in_syn[n], in_pot[n], in_thr[n], in_dsyn[n], in_dmem[n], ha,
                           in_ada[n], in_dada[n], in_sada[n], in_beff[n],
                           g_spk[n], g_pot[n], g_syn[n], g_ada[n]);

                // golden field contributions (OR into destination words)
                e_syn[syn_base  + g_offset(n[`PIN_BITS-1:0], ssz)]   |= g_new_data(n[4:0], leftjust(g_syn[n], ssz), ssz);
                e_pot[pot_base  + g_offset(n[`PIN_BITS-1:0], psz)]   |= g_new_data(n[4:0], leftjust(g_pot[n], psz), psz);
                e_spk[spike_base+ g_offset(n[`PIN_BITS-1:0], 3'b000)]|= g_new_data(n[4:0], {g_spk[n], 31'b0}, 3'b000);
                if (ha)
                    e_ada[ada_base + g_offset(n[`PIN_BITS-1:0], 3'b101)] |= g_new_data(n[4:0], g_ada[n], 3'b101);
            end

            last_neuron_idx = N - 1;
            syn_sz = ssz; pot_sz = psz; has_ada = ha; wmode = wm; binpt = 5'd0;

            // launch and run
            start = 1'b1; @(posedge clk); #1; start = 1'b0;
            verif_to = 20000;
            while (running_r && verif_to > 0) begin @(posedge clk); #1; verif_to = verif_to - 1; end
            if (verif_to == 0) begin
                $display("FAIL %0s: run did not finish", tag);
                verif_errors = verif_errors + 1;
            end
            // Corruption (if any) already occurred DURING the run while a packer
            // was full+stalled and the next neuron's result_valid re-OR'd in.
            // Drop write-back wait now and drain generously so every buffer
            // flushes its FINAL (possibly corrupted) word for inspection.
            wmode = 2'd0;
            repeat (200) begin @(posedge clk); #1; end

            // ---- exactly one accumulate per accepted neuron ----
            check_eq_u(accepts, N, {tag, " accept_count"});

            // ---- check every written word ----
            for (wcnt = 0; (wcnt*32) < N; wcnt = wcnt + 1)
                check_eq_u(spk_sram[spike_base + wcnt], e_spk[spike_base + wcnt], {tag, " spike_word"});

            for (k = 0; k < N; k = k + 1) begin
                check_eq_u(syn_sram[syn_base + g_offset(k[`PIN_BITS-1:0], ssz)],
                           e_syn[syn_base + g_offset(k[`PIN_BITS-1:0], ssz)], {tag, " syn_word"});
                check_eq_u(pot_sram[pot_base + g_offset(k[`PIN_BITS-1:0], psz)],
                           e_pot[pot_base + g_offset(k[`PIN_BITS-1:0], psz)], {tag, " pot_word"});
                if (ha)
                    check_eq_u(ada_sram[ada_base + k], e_ada[ada_base + k], {tag, " ada_word"});
            end
        end
    endtask

    initial begin
        verif_errors=0; verif_checks=0;
        start=0; running_r=0; neuron_counter_r=0; last_neuron_idx=0;
        syn_sz=SZ32; pot_sz=SZ32; has_ada=0; binpt=0; wmode=0;
        spike_wait=0; syn_wait=0; pot_wait=0; ada_wait=0;
        syn_base=20; pot_base=50; spike_base=80; ada_base=110;
        reset=1; repeat(2) @(posedge clk); #1; reset=0; @(posedge clk); #1;

        $display("=== tb_packer_cadence (fmiSnnAccMC) ===");
        void'($urandom(32'hCADE_0F8));

        // Exercises the FIXED wiring (pak_write_i = result_taken, the one-cycle
        // accept) that all five neuron_processing variants now use, under heavy
        // write-back back-pressure across the full slice-size matrix. The
        // as-built (result_valid) negative control that originally demonstrated
        // F8 (70 failures) is documented in verif/FINDINGS.md, not run here:
        // check_eq_u hard-prints a bare FAIL token that run_regression.sh greps,
        // so a scored negative-control pass cannot coexist with a green run.
        use_taken = 1'b1;

        // ---- 32-bit (1 neuron/word): spike packer still re-flushes if broken -
        run_block(20, 1'b0, SZ32, SZ32, 2'd0, "C1 32b noWait");
        run_block(20, 1'b0, SZ32, SZ32, 2'd2, "C2 32b heavyWait");
        run_block(33, 1'b1, SZ32, SZ32, 2'd2, "C3 32b ada heavyWait");
        run_block(70, 1'b0, SZ32, SZ32, 2'd2, "C4 32b N70 heavyWait");  // 2 spike-word flushes under stall

        // ---- 16-bit syn/pot (2/word): the F8 cadence-mismatch case ----------
        run_block(20, 1'b0, SZ16, SZ16, 2'd0, "S1 16b noWait");
        run_block(20, 1'b0, SZ16, SZ16, 2'd1, "S2 16b wait30");
        run_block(20, 1'b0, SZ16, SZ16, 2'd2, "S3 16b wait60");
        run_block(33, 1'b1, SZ16, SZ16, 2'd2, "S4 16b ada wait60");
        run_block(40, 1'b0, SZ16, SZ16, 2'd2, "S5 16b N40 wait60");

        // ---- 8-bit syn/pot (4/word) -----------------------------------------
        run_block(24, 1'b0, SZ8,  SZ8,  2'd0, "E1 8b noWait");
        run_block(24, 1'b0, SZ8,  SZ8,  2'd2, "E2 8b wait60");

        // ---- mixed (syn 16-bit, pot 32-bit) ---------------------------------
        run_block(20, 1'b0, SZ16, SZ32, 2'd2, "M1 syn16 pot32 wait60");

        `VERIF_EPILOGUE("tb_packer_cadence")
    end

    `VERIF_WATCHDOG(40000000)

endmodule
