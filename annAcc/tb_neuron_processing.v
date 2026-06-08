// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
// =============================================================================
// tb_neuron_processing  (annAcc, ANN feature-extractor neuron stage + requant)
//
// Authors      : Simon Davidson & Claude
// Created      : 2026-05-08
// Last modified: 2026-06-08
//
// Aggressive golden-checked rewrite. Drives WHOLE LAYERS of neurons through
// ann_neuron_processing and checks every memory write-back against a per-neuron
// golden (verif/np_ref.vh), over a constrained-random loop plus directed
// corners. Replaces the previous shallow 5-case directed test.
//
// Per neuron the DUT:
//   * reads the accumulated potential from syn_curr_mem (signed 32b/neuron),
//   * applies a threshold op (00=RELU, 01=LUT, 10=ABS) -> act_out,
//       LUT mode: lut_result = low 8 bits of lut_sram[thresh_base + (pot & 0xFF)],
//   * REQUANTISES act_out (np_ann_requant: bin-point align + round-half-up +
//     unsigned-saturate to the act_out_sz field width) -> packed into spike_mem,
//   * DECAYS act_out (np_ann_decay, signed Q0.32) -> packed into pot_mem.
//
// Goldens (all validated against ann_update_state_for_neuron / §4b requant):
//   np_ann_threshold, np_ann_requant, np_ann_decay  (verif/np_ref.vh).
//
// Coverage
// --------
//   * constrained-random potentials (mix of signed ranges, incl. negative for
//     RELU/ABS), 16-32 neurons per layer
//   * thresh_op swept over {RELU, LUT, ABS}; LUT mode pre-loads a random LUT
//   * act_out_sz / pot_sz packing widths swept: 32-bit (1/word) AND 8-bit
//     (4/word) -> verifies the packed write-back words bit-for-bit
//   * np_out_bin_point swept: 0 (requant disabled) AND active shifts
//     (bin_point_syn_curr=16, np_out_bin_point=4 -> round/shift/saturate),
//     including a forced-saturation layer at the field max
//   * read-path back-pressure (random syn_curr / thresh mem_wait): result must
//     be latency-invariant
//
// SRAM layout (256x32 each, addr truncated to [7:0]):
//   syn_curr base=  0  (read-only; the accumulated potentials)
//   thresh/LUT base= 64 (read-only; LUT table, 32-bit entries -> low 8b used)
//   pot      base=128  (write-only; decayed activations, packed per pot_sz)
//   spike    base=192  (write-only; requantised activations, packed per act_sz)
// =============================================================================
`timescale 1ns/1ps
`include "../shared/constants.v"

module tb_neuron_processing;

    localparam NEURON_IDX_SZ        = 6;       // up to 63 neurons
    localparam SYN_CURR_IDX_SZ      = 10;
    localparam SYN_CURR_DATA_IDX_SZ = 5;
    localparam SYN_CURR_SLICE_SZ    = 3;
    localparam SYN_CURR_SLICE_BITS  = 32;
    localparam LUT_IDX_SZ           = 8;
    localparam LUT_DATA_IDX_SZ      = 5;
    localparam LUT_SLICE_SZ         = 3;
    localparam LUT_SLICE_BITS       = 32;      // 32-bit LUT entries (low 8b = result)
    localparam POT_SLICE_SZ         = 3;
    localparam POT_SLICE_BITS       = 32;

    localparam SZ8  = 3'b011;                  // 8-bit  field (4/word)
    localparam SZ32 = 3'b101;                  // 32-bit field (1/word)

    localparam ROP_RELU = 2'b00;
    localparam ROP_LUT  = 2'b01;
    localparam ROP_ABS  = 2'b10;

    reg clk, reset;

    // Config
    reg [NEURON_IDX_SZ-1:0]      last_neuron_idx;
    reg [`ADDR_SIZE-1:0]         syn_curr_base, thresh_base, pot_base, spike_base;
    reg [SYN_CURR_SLICE_SZ-1:0]  syn_curr_sz;
    reg [POT_SLICE_SZ-1:0]       pot_sz;
    reg [2:0]                    lut_out_sz, act_out_sz;
    reg [4:0]                    bin_point_syn_curr, np_out_bin_point;
    reg [1:0]                    thresh_op;
    reg [31:0]                   pot_decay_mult;

    // Scheduler iface
    reg                     start_new_block;
    reg [`TGT_ACC_SZ-1:0]   target_acc;
    reg [`SCH_ENTRY_SZ-1:0] buffer_info;
    reg [`PIN_BITS-1:0]     src1, src2, src3, tgt, wrl;

    wire neuron_proc_finished, acc_busy, acc_finished;

    // syn_curr (rd only)
    wire                  syn_curr_mem_wr, syn_curr_mem_rd;
    reg                   syn_curr_mem_wait;
    wire [`ADDR_SIZE-1:0] syn_curr_mem_addr;
    wire [`POT_BITS-1:0]  syn_curr_mem_data_wr;
    reg  [`POT_BITS-1:0]  syn_curr_mem_data_rd;
    // thresh / LUT (rd only)
    wire                  thresh_mem_rd;
    reg                   thresh_mem_wait;
    wire [`ADDR_SIZE-1:0] thresh_mem_addr;
    reg  [`WTD_BITS-1:0]  thresh_mem_data;
    // pot (wr only)
    wire                  pot_mem_wr, pot_mem_rd;
    reg                   pot_mem_wait;
    wire [`ADDR_SIZE-1:0] pot_mem_addr;
    wire [`POT_BITS-1:0]  pot_mem_data_wr;
    reg  [`POT_BITS-1:0]  pot_mem_data_rd;
    // spike (wr only)
    wire                  spike_mem_wr;
    reg                   spike_mem_wait;
    wire [`ADDR_SIZE-1:0] spike_mem_addr;
    wire [`ACT_BITS-1:0]  spike_mem_data;

    integer verif_errors, verif_checks, verif_to, sram_i;
    `include "../verif/checks.vh"
    `include "../verif/np_ref.vh"
    `include "../verif/vt_driver.vh"
    `include "../verif/sram_bfm.vh"

    ann_neuron_processing #(
        .NEURON_IDX_SZ        (NEURON_IDX_SZ),
        .SYN_CURR_IDX_SZ      (SYN_CURR_IDX_SZ),
        .SYN_CURR_DATA_IDX_SZ (SYN_CURR_DATA_IDX_SZ),
        .SYN_CURR_SLICE_SZ    (SYN_CURR_SLICE_SZ),
        .SYN_CURR_SLICE_BITS  (SYN_CURR_SLICE_BITS),
        .LUT_IDX_SZ           (LUT_IDX_SZ),
        .LUT_DATA_IDX_SZ      (LUT_DATA_IDX_SZ),
        .LUT_SLICE_SZ         (LUT_SLICE_SZ),
        .LUT_SLICE_BITS       (LUT_SLICE_BITS),
        .POT_SLICE_SZ         (POT_SLICE_SZ),
        .POT_SLICE_BITS       (POT_SLICE_BITS))
    dut (
        .clk                    (clk),
        .reset                  (reset),
        .last_neuron_idx_i      (last_neuron_idx),
        .syn_curr_base_addr_i   (syn_curr_base),
        .thresh_base_addr_i     (thresh_base),
        .pot_base_addr_i        (pot_base),
        .spike_base_addr_i      (spike_base),
        .syn_curr_sz_i          (syn_curr_sz),
        .pot_sz_i               (pot_sz),
        .lut_out_sz_i           (lut_out_sz),
        .act_out_sz_i           (act_out_sz),
        .thresh_op_i            (thresh_op),
        .bin_point_syn_curr_i   (bin_point_syn_curr),
        .np_out_bin_point_i     (np_out_bin_point),
        .pot_decay_mult_i       (pot_decay_mult),
        .start_new_block_i      (start_new_block),
        .target_acc_i           (target_acc),
        .buffer_info_i          (buffer_info),
        .neuron_proc_finished_o (neuron_proc_finished),
        .acc_busy_o             (acc_busy),
        .acc_finished_o         (acc_finished),
        .src1_buff_addr_i       (src1),
        .src2_buff_addr_i       (src2),
        .src3_buff_addr_i       (src3),
        .tgt_buff_addr_i        (tgt),
        .weight_row_len_i       (wrl),
        .syn_curr_mem_wr_o      (syn_curr_mem_wr),
        .syn_curr_mem_rd_o      (syn_curr_mem_rd),
        .syn_curr_mem_wait_i    (syn_curr_mem_wait),
        .syn_curr_mem_addr_o    (syn_curr_mem_addr),
        .syn_curr_mem_data_o    (syn_curr_mem_data_wr),
        .syn_curr_mem_data_i    (syn_curr_mem_data_rd),
        .thresh_mem_rd_o        (thresh_mem_rd),
        .thresh_mem_wait_i      (thresh_mem_wait),
        .thresh_mem_addr_o      (thresh_mem_addr),
        .thresh_mem_data_i      (thresh_mem_data),
        .pot_mem_wr_o           (pot_mem_wr),
        .pot_mem_rd_o           (pot_mem_rd),
        .pot_mem_wait_i         (pot_mem_wait),
        .pot_mem_addr_o         (pot_mem_addr),
        .pot_mem_data_o         (pot_mem_data_wr),
        .pot_mem_data_i         (pot_mem_data_rd),
        .spike_mem_wr_o         (spike_mem_wr),
        .spike_mem_wait_i       (spike_mem_wait),
        .spike_mem_addr_o       (spike_mem_addr),
        .spike_mem_data_o       (spike_mem_data));

    // SRAM models (256x32, [7:0] truncation). syn_curr / thresh honour mem_wait.
    `SRAM_RD_WAIT(syn_sram, syn_curr_mem_rd, syn_curr_mem_wait, syn_curr_mem_addr, syn_curr_mem_data_rd)
    `SRAM_RD_WAIT(lut_sram, thresh_mem_rd,   thresh_mem_wait,   thresh_mem_addr,   thresh_mem_data)
    `SRAM_WR    (pot_sram,  pot_mem_wr,   pot_mem_addr,   pot_mem_data_wr)
    `SRAM_WR    (spk_sram,  spike_mem_wr, spike_mem_addr, spike_mem_data)

    initial clk = 0; always #5 clk = ~clk;

    // golden arrays
    reg signed [31:0] in_pot  [0:63];          // accumulated potential per neuron
    reg signed [31:0] g_act   [0:63];          // threshold result (pre-requant)
    reg        [31:0] g_spkval[0:63];          // requantised activation (spike_mem)
    reg signed [31:0] g_potval[0:63];          // decayed activation   (pot_mem)

    // read-path back-pressure driver (latency-invariance check)
    reg bp_en;
    always @(posedge clk) begin
        syn_curr_mem_wait <= bp_en ? ($urandom_range(99) < 30) : 1'b0;
        thresh_mem_wait   <= bp_en ? ($urandom_range(99) < 30) : 1'b0;
    end

    integer n, wi;

    // ---- packer-faithful field builder ------------------------------------
    // Replicates packer.v: left-justify the value's top w bits, then shift by
    // data_shift = (((n & shift_mask)+1) << out_sz), take bits [63:32]. ORing
    // these per word reproduces the packed output word bit-for-bit.
    function [31:0] g_new_data;
        input [4:0]  idx;
        input [31:0] value;        // already left-justified to top w bits
        input [2:0]  out_sz;
        reg   [4:0]  shift_mask;
        reg   [5:0]  plus;
        reg   [6:0]  data_shift;
        reg   [31:0] data_mask;
        reg   [63:0] temp;
        begin
            shift_mask = 5'h1F >> out_sz;
            plus       = {1'b0, (idx & shift_mask)} + 6'h01;
            data_mask  = (out_sz == 3'd5) ? 32'hFFFF_FFFF
                                          : (32'hFFFF_FFFF << (32 - (1 << out_sz)));
            data_shift = (out_sz == 3'd5) ? 7'h20 : (plus << out_sz);
            temp       = ({32'h0, (value & data_mask)} << data_shift);
            g_new_data = temp[63:32];
        end
    endfunction

    // Left-justify a requant/pot result's low w bits into the top of a word,
    // mirroring the act_out_lj / pot_wb_lj case in neuron_processing.v.
    function [31:0] left_justify;
        input [31:0] value;
        input [2:0]  sz;
        begin
            case (sz)
                3'b000: left_justify = {value[0],     31'b0};
                3'b001: left_justify = {value[1:0],   30'b0};
                3'b010: left_justify = {value[3:0],   28'b0};
                3'b011: left_justify = {value[7:0],   24'b0};
                3'b100: left_justify = {value[15:0],  16'b0};
                3'b101: left_justify = value[31:0];
                default:left_justify = 32'b0;
            endcase
        end
    endfunction

    // ---- run one layer of N neurons, predict + check all write-backs -------
    task run_block;
        input integer N;
        input [1:0]   rop;
        input [2:0]   actsz;
        input [2:0]   potsz;
        input [4:0]   bp_in;
        input [4:0]   bp_out;
        input [31:0]  decay;
        input         bp;            // inject read mem_wait back-pressure
        input         lut_rand;      // preload a random LUT
        input [255:0] tag;
        reg   [7:0]   lut_idx;
        reg   [7:0]   lut_res;
        reg   [31:0]  exp_word;
        begin
            `SRAM_CLEAR(syn_sram) `SRAM_CLEAR(lut_sram)
            `SRAM_CLEAR(pot_sram) `SRAM_CLEAR(spk_sram)

            // random LUT. NOTE: LUT_SLICE_BITS=32 so the cache returns the whole
            // word and ann_update_state zero-extends all of it. The golden uses
            // only the low 8 bits, so mask entries to 8 bits to keep them equal.
            if (lut_rand)
                for (n = 0; n < 256; n = n + 1)
                    lut_sram[n] = $urandom & 32'h0000_00FF;

            // per-neuron stimulus + golden
            for (n = 0; n < N; n = n + 1) begin
                // mix of signed ranges incl. negatives; widen for requant tests
                if (bp_out != 5'd0)
                    in_pot[n] = $urandom_range(0, 32'h0030_0000) - 32'sh0008_0000;
                else
                    in_pot[n] = $urandom_range(0, 200) - 80;
                syn_sram[n] = in_pot[n];

                // LUT result = low 8 bits of lut_sram[(thresh_base + (pot & 0xFF)) & 0xFF].
                // Cache addr = base + index; SRAM truncates to [7:0] (matches DUT).
                lut_idx  = in_pot[n][7:0];
                lut_res  = lut_sram[(thresh_base + {22'b0, lut_idx}) & 8'hFF][7:0];

                g_act[n]    = np_ann_threshold(in_pot[n], lut_res, rop);
                g_spkval[n] = np_ann_requant(g_act[n], bp_in, bp_out, actsz);
                g_potval[n] = np_ann_decay(g_act[n], decay);
            end

            // drive config
            last_neuron_idx    = N - 1;
            thresh_op          = rop;
            act_out_sz         = actsz;
            pot_sz             = potsz;
            bin_point_syn_curr = bp_in;
            np_out_bin_point   = bp_out;
            pot_decay_mult     = decay;
            bp_en              = bp;

            `VT_PULSE(start_new_block)
            `VT_WAIT_FINISH(neuron_proc_finished, 6000)
            // acc_busy / running_r clears the edge AFTER the finished pulse.
            @(posedge clk); #1;
            check_bit(acc_busy, 1'b0, {tag, " acc_busy low after finish"});
            repeat (40) @(posedge clk); #1;     // let both packers drain
            bp_en = 0;

            // ---- check packed spike_mem (requantised activations) ----------
            // Packer writes word wi to (spike_base + wi); SRAM truncates [7:0].
            for (wi = 0; wi * (32 >> actsz) < N; wi = wi + 1) begin
                exp_word = 32'b0;
                for (n = wi * (32 >> actsz);
                     n < N && (n >> (5 - actsz)) == wi;
                     n = n + 1)
                    exp_word = exp_word
                             | g_new_data(n[4:0], left_justify(g_spkval[n], actsz), actsz);
                check_eq_u(spk_sram[(spike_base + wi) & 8'hFF], exp_word, {tag, " spike_word"});
            end

            // ---- check packed pot_mem (decayed activations) ----------------
            for (wi = 0; wi * (32 >> potsz) < N; wi = wi + 1) begin
                exp_word = 32'b0;
                for (n = wi * (32 >> potsz);
                     n < N && (n >> (5 - potsz)) == wi;
                     n = n + 1)
                    exp_word = exp_word
                             | g_new_data(n[4:0], left_justify(g_potval[n], potsz), potsz);
                check_eq_u(pot_sram[(pot_base + wi) & 8'hFF], exp_word, {tag, " pot_word"});
            end
        end
    endtask

    initial begin
        verif_errors = 0; verif_checks = 0;
        start_new_block = 0; target_acc = 0; buffer_info = 0;
        src1 = 0; src2 = 0; src3 = 0; tgt = 0; wrl = 0;
        syn_curr_base = 30'd0;  thresh_base = 30'd64;
        pot_base      = 30'd128; spike_base = 30'd192;
        syn_curr_sz   = SZ32;   lut_out_sz  = SZ32;
        act_out_sz    = SZ32;   pot_sz      = SZ32;
        bin_point_syn_curr = 5'd0; np_out_bin_point = 5'd0;
        thresh_op     = ROP_RELU;
        pot_decay_mult = 32'h8000_0000;       // 0.5
        syn_curr_mem_wait = 0; thresh_mem_wait = 0;
        pot_mem_wait = 0; spike_mem_wait = 0;
        syn_curr_mem_data_rd = 0; thresh_mem_data = 0; pot_mem_data_rd = 0;
        bp_en = 0;

        reset = 1; repeat (2) @(posedge clk); #1; reset = 0; @(posedge clk); #1;

        $display("=== tb_neuron_processing (annAcc, golden) ===");
        void'($urandom(32'h0A11_AC22));

        //                 N   rop       actsz potsz bp_in bp_out  decay        bp lut  tag
        // ---- requant DISABLED (np_out_bin_point=0): threshold + decay path --
        run_block(16, ROP_RELU, SZ32, SZ32, 5'd0,  5'd0, 32'h8000_0000, 0, 0, "B1 RELU 32b");
        run_block(24, ROP_ABS,  SZ32, SZ32, 5'd0,  5'd0, 32'hC000_0000, 0, 0, "B2 ABS 32b 0.75decay");
        run_block(20, ROP_LUT,  SZ32, SZ32, 5'd0,  5'd0, 32'h8000_0000, 0, 1, "B3 LUT 32b randLUT");

        // ---- sub-32 packing (8-bit, 4/word) with requant disabled ----------
        run_block(32, ROP_RELU, SZ8,  SZ8,  5'd0,  5'd0, 32'h8000_0000, 0, 0, "B4 RELU 8b pack");
        run_block(18, ROP_LUT,  SZ8,  SZ32, 5'd0,  5'd0, 32'h4000_0000, 0, 1, "B5 LUT 8b/32b mixed");

        // ---- requant ENABLED: bp 16->4 (shift 12), round-half-up, saturate --
        run_block(16, ROP_ABS,  SZ32, SZ32, 5'd16, 5'd4, 32'h8000_0000, 0, 0, "B6 ABS requant 32b");
        run_block(32, ROP_RELU, SZ8,  SZ8,  5'd16, 5'd4, 32'h8000_0000, 0, 0, "B7 RELU requant 8b sat");
        run_block(24, ROP_ABS,  SZ8,  SZ32, 5'd16, 5'd8, 32'h8000_0000, 0, 1, "B8 ABS requant 8b sh8");

        // ---- read-path back-pressure: result must be latency-invariant -----
        run_block(20, ROP_RELU, SZ32, SZ32, 5'd0,  5'd0, 32'h8000_0000, 1, 0, "B9 RELU bp");
        run_block(20, ROP_LUT,  SZ8,  SZ8,  5'd16, 5'd4, 32'h8000_0000, 1, 1, "B10 LUT requant 8b bp");
        run_block(28, ROP_ABS,  SZ32, SZ8,  5'd16, 5'd4, 32'hC000_0000, 1, 0, "B11 ABS requant bp");

        // ---- directed saturation corner: all neurons clamp to field max ----
        // pot = 20.0 @ bp16 -> /2^12 = 320 -> sat 255 (8-bit) on every neuron.
        `SRAM_CLEAR(syn_sram) `SRAM_CLEAR(lut_sram)
        `SRAM_CLEAR(pot_sram) `SRAM_CLEAR(spk_sram)
        for (n = 0; n < 16; n = n + 1) begin
            in_pot[n]   = 32'h0014_0000;
            syn_sram[n] = in_pot[n];
            g_act[n]    = np_ann_threshold(in_pot[n], 8'd0, ROP_ABS);
            g_spkval[n] = np_ann_requant(g_act[n], 5'd16, 5'd4, SZ8);   // -> 255
            g_potval[n] = np_ann_decay(g_act[n], 32'h8000_0000);
        end
        last_neuron_idx = 16 - 1; thresh_op = ROP_ABS;
        act_out_sz = SZ8; pot_sz = SZ32;
        bin_point_syn_curr = 5'd16; np_out_bin_point = 5'd4;
        pot_decay_mult = 32'h8000_0000; bp_en = 0;
        `VT_PULSE(start_new_block)
        `VT_WAIT_FINISH(neuron_proc_finished, 6000)
        repeat (40) @(posedge clk); #1;
        // every 8-bit field = 255 -> each packed word = 0xFFFFFFFF (4 neurons).
        // spike_base=192 -> SRAM index (192 + word_offset) & 0xFF; word 0 -> 192.
        check_eq_u(spk_sram[(192 + 0) & 8'hFF], 32'hFFFF_FFFF, "B12 sat word0 0xFFFFFFFF");
        check_eq_u(spk_sram[(192 + 3) & 8'hFF], 32'hFFFF_FFFF, "B12 sat word3 0xFFFFFFFF");
        check_eq_u(g_spkval[0], 32'h0000_00FF, "B12 golden saturates to 255");

        `VERIF_EPILOGUE("tb_neuron_processing")
    end

    `VERIF_WATCHDOG(8000000)

endmodule
