// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
// =============================================================================
// verif/np_ref.vh  --  golden neuron reference model for FlexMan testbenches
//
// Authors      : Simon Davidson & Claude
// Created      : 2026-06-07
// Last modified: 2026-06-07
//
// ONE golden model covering all four accelerator variants:
//   LIF  : snnAcc / ipSnnAcc   -> np_ref_lif
//   ANN  : annAcc              -> np_ann_threshold / np_ann_decay / np_ann_requant
//   FMI  : fmiSnnAcc           -> np_ref_fmi
//
// Include INSIDE the testbench module body, AFTER checks.vh:
//       module tb_foo;
//           ...
//           `include "../verif/checks.vh"
//           `include "../verif/np_ref.vh"
//           ...
//
// All helpers are `automatic` so the file may be included in several modules
// within one xrun invocation without name clashes, and are defined before the
// tasks that call them (Xcelium 1800-2009 strict mode forbids forward refs).
//
// MIRROR vs IDEAL  (important)
// ----------------------------
// The decay helpers come in two flavours:
//   * MIRROR  (np_decay_unsigned / np_decay_signed) reproduce the EXACT Verilog
//     operand-signedness form of the corresponding RTL multiply, so a random
//     stimulus loop comparing DUT-vs-golden tracks the datapath/pipeline/memory
//     integration WITHOUT raising false arithmetic failures.  Use these for the
//     constrained-random loops.
//   * IDEAL   (np_decay_signed, again) is the mathematically-correct signed
//     Q0.32 decay.  Use it in DIRECTED "spec-intent" checks (e.g. decaying a
//     NEGATIVE synaptic current).  snnAcc/annAcc decay a negative operand with
//     a signed*UNSIGNED multiply (np_decay_unsigned) which DIFFERS from the
//     ideal signed result; a directed check that asserts the ideal value will
//     therefore FLAG that behaviour.  If such a check fails it is a FINDING to
//     raise with Simon, not something to silently paper over.
//
// RTL sources mirrored (verified 2026-06-07):
//   snnAcc/update_state_for_neuron.v        (LIF, 3-operand membrane sat)
//   annAcc/update_state_for_neuron.v        (ANN threshold + decay)
//   annAcc/neuron_processing.v  §4b         (ANN requant)
//   fmiSnnAcc/update_state_for_neuron.v     (FMI per-neuron decay + adaptive)
//   neuron_model.md                         (equations; bias added at MEMBRANE)
// =============================================================================

// -----------------------------------------------------------------------------
// Decay primitives
// -----------------------------------------------------------------------------

// MIRROR of snnAcc / annAcc decay: `mul_a (signed) * mul_b (unsigned)` with the
// 64-bit product's high 32 bits taken.  Replicates the RTL operand types
// EXACTLY (no $signed cast), so it matches the DUT bit-for-bit, including the
// signed*unsigned behaviour for negative x.
function automatic signed [31:0] np_decay_unsigned;
    input signed [31:0] x;
    input        [31:0] mult;
    reg signed [31:0] a;
    reg        [31:0] b;
    reg signed [63:0] p;
    begin
        a = x;
        b = mult;
        p = a * b;                 // signed x unsigned, 64-bit context (mirrors RTL)
        np_decay_unsigned = p[63:32];
    end
endfunction

// MIRROR of FMI decay AND the IDEAL signed Q0.32 decay: both operands cast to
// signed (decay zero-extended to 33 bits, always positive).  This is the
// mathematically-correct floor(x * mult / 2^32) for signed x.
function automatic signed [31:0] np_decay_signed;
    input signed [31:0] x;
    input        [31:0] mult;
    reg signed [63:0] p;
    begin
        p = $signed(x) * $signed({1'b0, mult});
        np_decay_signed = p[63:32];
    end
endfunction

// -----------------------------------------------------------------------------
// Saturation primitives
// -----------------------------------------------------------------------------

// LIF membrane saturation (snnAcc/update_state_for_neuron.v:86-93).
// 3-operand sum carried in 34 bits; NOTE the documented quirk: a sum of exactly
// 0x80000000 takes the UNDERFLOW branch and clamps to 0x80000000.
function automatic signed [31:0] np_sat34_lif;
    input signed [33:0] s;          // potential_i + bias_i + syn_i, sign-extended to 34b
    reg ov, un;
    begin
        ov = ~s[31] & (s[33:32] != 2'b00);
        un =  s[31] & (s[33:32] != 2'b11);
        np_sat34_lif = ov ? 32'sh7FFFFFFF :
                       un ? 32'sh80000000 : s[31:0];
    end
endfunction

// FMI membrane saturation (fmiSnnAcc/update_state_for_neuron.v:210-214).
// 2-operand sum carried in 34 bits; overflow when [33:32]==01, underflow ==10.
function automatic signed [31:0] np_sat34_fmi;
    input signed [33:0] s;          // decayed_diff + eff_syn, sign-extended to 34b
    begin
        if      (s[33:32] == 2'b01) np_sat34_fmi = 32'sh7FFFFFFF;
        else if (s[33:32] == 2'b10) np_sat34_fmi = 32'sh80000000;
        else                        np_sat34_fmi = s[31:0];
    end
endfunction

// -----------------------------------------------------------------------------
// ANN threshold / decay / requant (annAcc)
// -----------------------------------------------------------------------------

// Threshold op (annAcc/update_state_for_neuron.v:77-87):
//   2'b01 = LUT  (zero-extend 8-bit lut_result), 2'b10 = ABS, else RELU.
//   ABS of 0x80000000 saturates to 0x7FFFFFFF.
function automatic [31:0] np_ann_threshold;
    input signed [31:0] potential;
    input        [7:0]  lut_result;
    input        [1:0]  thresh_op;
    reg [31:0] absv;
    begin
        absv = (potential == 32'h80000000) ? 32'h7FFFFFFF
             : (potential[31] ? -potential : potential);
        if      (thresh_op == 2'b01) np_ann_threshold = {24'b0, lut_result};
        else if (thresh_op == 2'b10) np_ann_threshold = absv;
        else                         np_ann_threshold = potential[31] ? 32'b0 : potential;
    end
endfunction

// Decay of the (non-negative) threshold result -> pot_mem writeback value.
// Mirrors ann_update_state_for_neuron mul_a(signed)*mul_b(unsigned), [63:32].
function automatic [31:0] np_ann_decay;
    input [31:0] act_out;
    input [31:0] decay;
    reg signed [31:0] a;
    reg        [31:0] b;
    reg signed [63:0] p;
    begin
        a = act_out;
        b = decay;
        p = a * b;
        np_ann_decay = p[63:32];
    end
endfunction

// Output requant (annAcc/neuron_processing.v:283-311): align bin point,
// round-half-up, unsigned-saturate to act_sz field width.
// bp_out==0 disables requant (returns act_out unchanged).
function automatic [31:0] np_ann_requant;
    input [31:0] act_out;
    input [4:0]  bp_in;
    input [4:0]  bp_out;
    input [2:0]  act_sz;
    reg [4:0]  sh;
    reg [32:0] rnd, shifted;
    reg [31:0] rqmax;
    begin
        sh  = bp_in - bp_out;                       // 5-bit, wraps (mirrors RTL)
        rnd = {1'b0, act_out} +
              ((sh == 5'd0) ? 33'd0 : (33'd1 << (sh - 5'd1)));
        shifted = rnd >> sh;
        case (act_sz)
            3'b000:  rqmax = 32'h0000_0001;
            3'b001:  rqmax = 32'h0000_0003;
            3'b010:  rqmax = 32'h0000_000F;
            3'b011:  rqmax = 32'h0000_00FF;
            3'b100:  rqmax = 32'h0000_FFFF;
            default: rqmax = 32'hFFFF_FFFF;
        endcase
        if      (bp_out == 5'd0)            np_ann_requant = act_out;
        else if (shifted > {1'b0, rqmax})  np_ann_requant = rqmax;
        else                               np_ann_requant = shifted[31:0];
    end
endfunction

// -----------------------------------------------------------------------------
// Full neuron golden: LIF (snnAcc / ipSnnAcc)
// -----------------------------------------------------------------------------
// pot = sat(potential + bias + syn); spike = pot >= thresh;
// pot_o = spike ? (sub_on_fire ? decay(pot)-thresh : 0) : decay(pot)
// syn_o = decay(syn)                       (decays RAW syn, signed*unsigned mirror)
task automatic np_ref_lif;
    input  signed [31:0] syn;
    input  signed [31:0] pot;
    input  signed [31:0] bias;
    input  signed [31:0] thresh;
    input         [31:0] syn_dcy;
    input         [31:0] pot_dcy;
    input                sub_on_fire;
    output               spike_o;
    output signed [31:0] pot_o;
    output signed [31:0] syn_o;
    reg signed [33:0] sum34;
    reg signed [31:0] psat, dec_pot;
    begin
        sum34   = $signed({{2{pot[31]}},  pot})
                + $signed({{2{bias[31]}}, bias})
                + $signed({{2{syn[31]}},  syn});
        psat    = np_sat34_lif(sum34);
        spike_o = (psat >= thresh);
        dec_pot = np_decay_unsigned(psat, pot_dcy);
        pot_o   = spike_o ? (sub_on_fire ? (dec_pot - thresh) : 32'sd0)
                          : dec_pot;
        syn_o   = np_decay_unsigned(syn, syn_dcy);
    end
endtask

// -----------------------------------------------------------------------------
// Full neuron golden: FMI (fmiSnnAcc)
// -----------------------------------------------------------------------------
// syn_o   = decay_signed(syn, syn_dcy)
// ada_corr= (b_eff * ada)[63:32]   (signed * unsigned-ada)        [has_ada only]
// eff_syn = has_ada ? syn - ada_corr : syn
// new_mem = sat( decay_signed(pot - eff_syn, mem_dcy) + eff_syn )
// spike   = new_mem >= thresh ; pot_o = spike ? 0 : new_mem
// ada_o   = has_ada ? (ada*dcy_ada)[63:32] + (spike?scl_ada:0) : 0
task automatic np_ref_fmi;
    input  signed [31:0] syn;
    input  signed [31:0] pot;
    input  signed [31:0] thresh;
    input         [31:0] syn_dcy;
    input         [31:0] mem_dcy;
    input                has_ada;
    input         [31:0] ada;
    input         [31:0] dcy_ada;
    input         [31:0] scl_ada;
    input  signed [31:0] b_eff;
    output               spike_o;
    output signed [31:0] pot_o;
    output signed [31:0] syn_o;
    output        [31:0] ada_o;
    reg signed [31:0] ada_corr, eff_syn, diff, dec_diff, nm_sat;
    reg signed [33:0] nm34;
    reg signed [63:0] p;
    begin
        syn_o = np_decay_signed(syn, syn_dcy);
        p        = $signed(b_eff) * $signed({1'b0, ada});
        ada_corr = p[63:32];
        eff_syn  = has_ada ? (syn - ada_corr) : syn;
        diff     = pot - eff_syn;                        // 32-bit wrap (mirrors RTL)
        dec_diff = np_decay_signed(diff, mem_dcy);
        nm34     = $signed({{2{dec_diff[31]}}, dec_diff})
                 + $signed({{2{eff_syn[31]}},  eff_syn});
        nm_sat   = np_sat34_fmi(nm34);
        spike_o  = (nm_sat >= thresh);
        pot_o    = spike_o ? 32'sd0 : nm_sat;
        if (has_ada) begin
            p     = $signed({1'b0, ada}) * $signed({1'b0, dcy_ada});
            ada_o = p[63:32] + (spike_o ? scl_ada : 32'd0);
        end else
            ada_o = 32'd0;
    end
endtask
