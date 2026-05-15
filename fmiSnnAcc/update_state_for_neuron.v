// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
// =============================================================================
// Module: update_state_for_neuron  (FMI variant)
//
// Implements the per-neuron membrane update for the FMI spiking-SQP model:
//
//   (a) Plain heterogeneous-tau LIF  (groups 4+):
//         writeback_syn = dcy_syn * syn_curr_i
//         new_mem       = dcy_mem*(pot - syn) + syn        [= dcy_mem*pot + scl_mem*syn]
//         spike         = new_mem >= threshold
//         pot_out       = spike ? 0 : new_mem
//
//   (b) Adaptive LIF  (group3, has_ada_i=1):
//         ada_corr      = b_eff * ada            [b_eff = scl_mem*adapt_b, pre-computed]
//         eff_syn       = syn_curr_i - ada_corr
//         new_mem       = dcy_mem*(pot - eff_syn) + eff_syn
//         new_ada       = dcy_ada*ada + (spike ? scl_ada : 0)
//
// Pipeline (single shared multiplier per path; 2 cycles without ada, 3 with ada):
//
//   C1  mul1 = dcy_syn * syn_curr_i  →  decayed_syn_r  (writeback)
//       mul2 = b_eff   * ada_i       →  ada_corr_r     (has_ada only)
//
//   C2  eff_syn_r = syn_curr_r - ada_corr_r  (or syn_curr_r if !has_ada)
//       diff_r    = potential_r - eff_syn_r
//       mul1 = dcy_mem * diff_r      →  decayed_diff  (combinatorial, used same cycle)
//       mul2 = dcy_ada * ada_r       →  dcy_ada_x_ada_r  (registered for C3)
//       new_mem_r = decayed_diff + eff_syn_r
//       spike_r   = new_mem_r >= threshold_r
//       result_valid_o HIGH for !has_ada (outputs from combinatorial net)
//
//   C3  (has_ada only)
//       new_ada = dcy_ada_x_ada_r + (spike_r ? scl_ada_r : 0)
//       result_valid_o HIGH  (outputs from registered C2 values + combinatorial ada)
//
// Multiplier conventions (same as original snnAcc):
//   signed × unsigned-Q0.32 : 64-bit signed product, result = product[63:32]
//   signed × signed (b_eff*ada): 64-bit signed product, result = product[63:32]
// =============================================================================

`timescale 1ns / 1ps

module update_state_for_neuron # (
    parameter SYN_CURR_SLICE_BITS  = 32,
    parameter POT_SLICE_BITS       = 32,
    parameter THRESH_SLICE_BITS    = 32
)(
    input  wire                                    clk,
    input  wire                                    reset,

    // Input interface (valid/taken handshake)
    input  wire                                    neuron_valid_i,
    input  wire signed [SYN_CURR_SLICE_BITS-1:0]   syn_curr_i,
    input  wire signed [POT_SLICE_BITS-1:0]        potential_i,
    input  wire signed [THRESH_SLICE_BITS-1:0]     threshold_i,
    input  wire        [31:0]                      syn_dcy_i,     // Q0.32 unsigned
    input  wire        [31:0]                      mem_dcy_i,     // Q0.32 unsigned
    input  wire                                    has_ada_i,
    input  wire        [POT_SLICE_BITS-1:0]        ada_i,         // unsigned ada state
    input  wire signed [POT_SLICE_BITS-1:0]        b_eff_i,       // scl_mem*adapt_b, signed
    input  wire        [31:0]                      dcy_ada_i,     // Q0.32 unsigned
    input  wire        [31:0]                      scl_ada_i,     // Q0.32 unsigned (1-dcy_ada)
    output wire                                    neuron_taken_o,

    // Output interface (valid/taken handshake)
    output wire                                    result_valid_o,
    output wire signed [POT_SLICE_BITS-1:0]        potential_o,
    output wire signed [SYN_CURR_SLICE_BITS-1:0]   syn_curr_o,
    output wire                                    spike_o,
    output wire        [POT_SLICE_BITS-1:0]        ada_o,
    input  wire                                    result_taken_i
);

    // =========================================================================
    // State machine: IDLE(0) → C1(1) → C2(2) → [C3(3) if has_ada]
    // =========================================================================
    localparam [1:0] ST_IDLE = 2'd0, ST_C1 = 2'd1, ST_C2 = 2'd2, ST_C3 = 2'd3;
    reg [1:0] state_r;
    reg       has_ada_r;

    always @(posedge clk) begin
        if (reset)
            state_r <= ST_IDLE;
        else case (state_r)
            ST_IDLE: if (neuron_valid_i)              state_r <= ST_C1;
            ST_C1:                                    state_r <= ST_C2;
            ST_C2:   if (has_ada_r)                   state_r <= ST_C3;
                     else if (result_taken_i)         state_r <= ST_IDLE;
            ST_C3:   if (result_taken_i)              state_r <= ST_IDLE;
            default:                                  state_r <= ST_IDLE;
        endcase
    end

    // =========================================================================
    // Input capture registers (loaded on IDLE→C1 edge)
    // =========================================================================
    reg signed [SYN_CURR_SLICE_BITS-1:0] syn_curr_r;
    reg signed [POT_SLICE_BITS-1:0]      potential_r;
    reg signed [THRESH_SLICE_BITS-1:0]   threshold_r;
    reg        [31:0]                    syn_dcy_r;
    reg        [31:0]                    mem_dcy_r;
    reg        [POT_SLICE_BITS-1:0]      ada_r;
    reg signed [POT_SLICE_BITS-1:0]      b_eff_r;
    reg        [31:0]                    dcy_ada_r;
    reg        [31:0]                    scl_ada_r;

    always @(posedge clk) begin
        if (reset) begin
            syn_curr_r  <= {SYN_CURR_SLICE_BITS{1'b0}};
            potential_r <= {POT_SLICE_BITS{1'b0}};
            threshold_r <= {THRESH_SLICE_BITS{1'b0}};
            syn_dcy_r   <= 32'b0;
            mem_dcy_r   <= 32'b0;
            ada_r       <= {POT_SLICE_BITS{1'b0}};
            b_eff_r     <= {POT_SLICE_BITS{1'b0}};
            dcy_ada_r   <= 32'b0;
            scl_ada_r   <= 32'b0;
            has_ada_r   <= 1'b0;
        end else if (state_r == ST_IDLE && neuron_valid_i) begin
            syn_curr_r  <= syn_curr_i;
            potential_r <= potential_i;
            threshold_r <= threshold_i;
            syn_dcy_r   <= syn_dcy_i;
            mem_dcy_r   <= mem_dcy_i;
            ada_r       <= ada_i;
            b_eff_r     <= b_eff_i;
            dcy_ada_r   <= dcy_ada_i;
            scl_ada_r   <= scl_ada_i;
            has_ada_r   <= has_ada_i;
        end
    end

    // =========================================================================
    // Multiplier 1: syn decay (C1) then mem decay (C2)
    //   mul1_a: signed  [SYN_CURR_SLICE_BITS or POT_SLICE_BITS]
    //   mul1_b: unsigned Q0.32
    //   result[63:32] gives the Q0.32-scaled output in the same format as mul1_a
    // =========================================================================
    wire signed [SYN_CURR_SLICE_BITS-1:0] mul1_a;
    wire        [31:0]                    mul1_b;
    wire signed [63:0]                    mul1_result;

    // C1: syn_curr × syn_dcy;  C2: diff × mem_dcy
    wire signed [POT_SLICE_BITS-1:0]  eff_syn_wire;   // computed combinatorially in C2
    wire signed [POT_SLICE_BITS-1:0]  diff_wire;      // potential - eff_syn

    assign mul1_a = (state_r == ST_C1) ? syn_curr_r :
                    (state_r == ST_C2) ? diff_wire   :
                                         {SYN_CURR_SLICE_BITS{1'b0}};
    assign mul1_b = (state_r == ST_C1) ? syn_dcy_r  :
                    (state_r == ST_C2) ? mem_dcy_r   : 32'b0;

    assign mul1_result = $signed(mul1_a) * $signed({1'b0, mul1_b});

    // Registered result from C1 (syn writeback)
    reg signed [SYN_CURR_SLICE_BITS-1:0] decayed_syn_r;
    always @(posedge clk) begin
        if (reset)              decayed_syn_r <= {SYN_CURR_SLICE_BITS{1'b0}};
        else if (state_r == ST_C1) decayed_syn_r <= mul1_result[SYN_CURR_SLICE_BITS+31:32];
    end

    // =========================================================================
    // Multiplier 2: ada correction (C1) then ada decay (C2)
    // C1: b_eff (signed) × ada (unsigned) — both POT_SLICE_BITS
    // C2: ada (unsigned) × dcy_ada (Q0.32 unsigned)
    // Result extraction: [2*POT_SLICE_BITS-1 : POT_SLICE_BITS] (upper half)
    // =========================================================================
    wire signed [2*POT_SLICE_BITS-1:0] mul2_beff_ada;
    wire        [63:0]                 mul2_dcy_ada;

    assign mul2_beff_ada = b_eff_r * $signed({1'b0, ada_r});           // C1
    assign mul2_dcy_ada  = $signed({1'b0, ada_r}) * $signed({1'b0, dcy_ada_r}); // C2

    // Register C1 ada correction result
    reg signed [POT_SLICE_BITS-1:0] ada_corr_r;
    always @(posedge clk) begin
        if (reset)              ada_corr_r <= {POT_SLICE_BITS{1'b0}};
        else if (state_r == ST_C1)
            ada_corr_r <= mul2_beff_ada[2*POT_SLICE_BITS-1:POT_SLICE_BITS];
    end

    // Register C2 ada decay result
    reg [POT_SLICE_BITS-1:0] dcy_ada_x_ada_r;
    always @(posedge clk) begin
        if (reset)              dcy_ada_x_ada_r <= {POT_SLICE_BITS{1'b0}};
        else if (state_r == ST_C2)
            dcy_ada_x_ada_r <= mul2_dcy_ada[63:32];
    end

    // =========================================================================
    // Cycle 2 combinatorial: eff_syn, diff, new_mem, spike
    // These are valid throughout state C2 (and C3 for !has_ada they're outputs directly)
    // =========================================================================

    // Effective synaptic input (subtract ada correction for adaptive neurons)
    assign eff_syn_wire = has_ada_r ? (syn_curr_r - ada_corr_r) : syn_curr_r;
    assign diff_wire    = potential_r - eff_syn_wire;

    // new_mem via reformulation:  dcy_mem*(pot-eff_syn) + eff_syn
    wire signed [POT_SLICE_BITS+1:0] new_mem_wide;
    wire signed [POT_SLICE_BITS-1:0] decayed_diff;
    wire                             new_mem_overflow, new_mem_underflow;
    wire signed [POT_SLICE_BITS-1:0] new_mem_sat;

    assign decayed_diff    = mul1_result[POT_SLICE_BITS+31:32];
    assign new_mem_wide    = $signed({decayed_diff[POT_SLICE_BITS-1], decayed_diff}) +
                             $signed({eff_syn_wire[POT_SLICE_BITS-1], eff_syn_wire});

    assign new_mem_overflow  = ~new_mem_wide[POT_SLICE_BITS+1] && new_mem_wide[POT_SLICE_BITS];
    assign new_mem_underflow =  new_mem_wide[POT_SLICE_BITS+1] && ~new_mem_wide[POT_SLICE_BITS];
    assign new_mem_sat = new_mem_overflow  ? {1'b0, {(POT_SLICE_BITS-1){1'b1}}} :
                         new_mem_underflow ? {1'b1, {(POT_SLICE_BITS-1){1'b0}}} :
                                             new_mem_wide[POT_SLICE_BITS-1:0];

    wire spike_comb;
    assign spike_comb = (new_mem_sat >= $signed(threshold_r));

    wire signed [POT_SLICE_BITS-1:0] pot_comb;
    assign pot_comb = spike_comb ? {POT_SLICE_BITS{1'b0}} : new_mem_sat;

    // =========================================================================
    // C2→C3 registers (has_ada path: hold C2 results for C3 output)
    // =========================================================================
    reg signed [POT_SLICE_BITS-1:0] new_mem_r;
    reg                             spike_r;
    reg        [31:0]               scl_ada_r2;   // copy for use in C3

    always @(posedge clk) begin
        if (reset) begin
            new_mem_r  <= {POT_SLICE_BITS{1'b0}};
            spike_r    <= 1'b0;
            scl_ada_r2 <= 32'b0;
        end else if (state_r == ST_C2 && has_ada_r) begin
            new_mem_r  <= pot_comb;
            spike_r    <= spike_comb;
            scl_ada_r2 <= scl_ada_r;
        end
    end

    // =========================================================================
    // Cycle 3 combinatorial: new_ada = dcy_ada_x_ada + (spike ? scl_ada : 0)
    // =========================================================================
    wire [POT_SLICE_BITS-1:0] new_ada_comb;
    assign new_ada_comb = dcy_ada_x_ada_r + (spike_r ? scl_ada_r2 : {32{1'b0}});

    // =========================================================================
    // Output mux: select C2 (non-adaptive) or C3 (adaptive) results
    // =========================================================================
    assign result_valid_o = (state_r == ST_C2 && !has_ada_r) || (state_r == ST_C3);

    assign potential_o = (state_r == ST_C3) ? new_mem_r  : pot_comb;
    assign spike_o     = (state_r == ST_C3) ? spike_r    : spike_comb;
    assign ada_o       = (state_r == ST_C3) ? new_ada_comb : {POT_SLICE_BITS{1'b0}};
    assign syn_curr_o  = decayed_syn_r;

    // Upstream handshake: accept a new neuron only when downstream accepted the result
    assign neuron_taken_o = result_taken_i;

endmodule
