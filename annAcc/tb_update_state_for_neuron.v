// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
`timescale 1ns/1ps

// ================================================================
// tb_update_state_for_neuron (annAcc variant)
//
// 3-cycle pipeline:
//   ST_IDLE (neuron_valid_i=1):
//     thresh_op=00 (RELU): act_out = potential_i < 0 ? 0 : potential_i
//     thresh_op=01 (LUT):  act_out = zero_extend(lut_result_i)
//     thresh_op=10 (ABS):  act_out = |potential_i|, sat at 0x7FFFFFFF
//     act_out_r registered on IDLE->C1 edge.
//   ST_C1:
//     act_out_r * pot_decay_mult >> 32 -> decayed_act_r  (registered on C1->C2)
//   ST_C2 (result_valid_o=1):
//     decayed_act_o = decayed_act_r
//     act_out_o = act_out_r
//
// Decay factor 0x80000000 = 0.5 in Q0.32 → floor(value / 2).
//
// Tests
// -----
//   1. RELU: positive potential passes through, decays correctly.
//   2. RELU: negative potential clamped to 0; decayed = 0.
//   3. RELU: zero potential → 0 output.
//   4. LUT:  lut_result zero-extended; decayed correctly.
//   5. LUT:  lut_result = 0 → both outputs = 0.
//   6. Zero decay (decay_mult=0) → decayed_act = 0.
//   7. Full decay (decay_mult=0xFFFFFFFF ≈ 1.0) → decayed ≈ act.
//   8. ABS: positive potential unchanged.
//   9. ABS: negative potential negated.
//  10. ABS: 0x80000000 saturates to 0x7FFFFFFF.
// ================================================================

module tb_update_state_for_neuron;

localparam POT_W   = 32;
localparam THRESH_W = 8;

reg                     clk, reset;
reg               [1:0] thresh_op;
reg                     neuron_valid;
reg signed [POT_W-1:0]  potential;
reg       [THRESH_W-1:0] lut_result;
reg       [31:0]         pot_decay;
reg                      result_taken;

wire                     neuron_taken;
wire                     result_valid;
wire [POT_W-1:0]         act_out;
wire [POT_W-1:0]         decayed_act;

ann_update_state_for_neuron #(
    .POT_SLICE_BITS   (POT_W),
    .THRESH_SLICE_BITS(THRESH_W))
dut (
    .clk                    (clk),
    .reset                  (reset),
    .thresh_op_i            (thresh_op),
    .neuron_valid_i         (neuron_valid),
    .potential_i            (potential),
    .lut_result_i           (lut_result),
    .potential_decay_mult_i (pot_decay),
    .neuron_taken_o         (neuron_taken),
    .result_valid_o         (result_valid),
    .act_out_o              (act_out),
    .decayed_act_o          (decayed_act),
    .result_taken_i         (result_taken)
);

initial clk = 0;
always  #5 clk = ~clk;

integer errors;

// Run one neuron: present inputs for one cycle, poll result_valid_o so
// the task works regardless of pipeline depth.
task run_neuron;
    input signed [POT_W-1:0]   pot_in;
    input        [THRESH_W-1:0] lut_in;
    input        [31:0]         decay;
    input        [1:0]          thr_op;
    output [POT_W-1:0] act_out_r, decayed_r;
    begin
        potential    = pot_in;
        lut_result   = lut_in;
        pot_decay    = decay;
        thresh_op    = thr_op;
        neuron_valid = 1;
        result_taken = 0;
        @(posedge clk); #1;   // IDLE->C1: act_out_r latched

        neuron_valid = 0;
        // Wait for result (handles any pipeline depth)
        while (!result_valid) @(posedge clk); #1;

        // result_valid_o=1: sample outputs
        act_out_r = act_out;
        decayed_r = decayed_act;

        result_taken = 1;
        @(posedge clk); #1;   // consume result, C2->IDLE
        result_taken = 0;
        @(posedge clk); #1;   // idle
    end
endtask

task check_eq;
    input [POT_W-1:0] got, exp;
    input [255:0] label;
    begin
        if (got !== exp) begin
            $display("FAIL %s: got %0d (0x%08h)  exp %0d (0x%08h)",
                     label, got, got, exp, exp);
            errors = errors + 1;
        end
    end
endtask

// Expected Q0.32 decay: floor(value * decay / 2^32)
function [POT_W-1:0] decayed;
    input [POT_W-1:0] val;
    input [31:0]      mult;
    reg [63:0] tmp;
    begin
        tmp    = $signed({{32{val[POT_W-1]}}, val}) * {32'b0, mult};
        decayed = tmp[63:32];
    end
endfunction

reg [POT_W-1:0] act_r, dec_r;

initial begin
    errors       = 0;
    neuron_valid = 0;
    result_taken = 0;
    thresh_op    = 0;
    potential    = 0;
    lut_result   = 0;
    pot_decay    = 32'h80000000;
    reset        = 1;
    @(posedge clk); #1;
    reset = 0;
    @(posedge clk); #1;

    $display("=== tb_update_state_for_neuron (annAcc) ===");

    // --------------------------------------------------------
    // Test 1: RELU — positive potential
    // potential = 40, decay = 0.5 → act_out = 40, decayed = 20
    // --------------------------------------------------------
    $display("Test 1: RELU positive potential");
    run_neuron(32'd40, 8'h00, 32'h80000000, 1'b0, act_r, dec_r);
    check_eq(act_r, 32'd40, "T1 act_out");
    check_eq(dec_r, decayed(32'd40, 32'h80000000), "T1 decayed_act");

    // --------------------------------------------------------
    // Test 2: RELU — negative potential clamped to 0
    // potential = -10 → act_out = 0, decayed = 0
    // --------------------------------------------------------
    $display("Test 2: RELU negative potential → clamped to 0");
    run_neuron(-32'd10, 8'h00, 32'h80000000, 1'b0, act_r, dec_r);
    check_eq(act_r, 32'd0, "T2 act_out clamped");
    check_eq(dec_r, 32'd0, "T2 decayed_act = 0");

    // --------------------------------------------------------
    // Test 3: RELU — zero potential
    // --------------------------------------------------------
    $display("Test 3: RELU zero potential");
    run_neuron(32'd0, 8'h00, 32'h80000000, 1'b0, act_r, dec_r);
    check_eq(act_r, 32'd0, "T3 act_out = 0");
    check_eq(dec_r, 32'd0, "T3 decayed_act = 0");

    // --------------------------------------------------------
    // Test 4: LUT — lut_result zero-extended, decay 0.5
    // lut_result = 8'hAB = 171, act_out = 32'h000000AB = 171
    // decayed = floor(171 / 2) = 85
    // --------------------------------------------------------
    $display("Test 4: LUT threshold");
    run_neuron(32'd0, 8'hAB, 32'h80000000, 2'b01, act_r, dec_r);
    check_eq(act_r, 32'h000000AB,               "T4 act_out = lut zero-extended");
    check_eq(dec_r, decayed(32'h0000_00AB, 32'h80000000), "T4 decayed_act");

    // --------------------------------------------------------
    // Test 5: LUT — lut_result = 0 → both outputs = 0
    // --------------------------------------------------------
    $display("Test 5: LUT result = 0");
    run_neuron(32'd999, 8'h00, 32'h80000000, 2'b01, act_r, dec_r);
    check_eq(act_r, 32'd0, "T5 act_out = 0");
    check_eq(dec_r, 32'd0, "T5 decayed_act = 0");

    // --------------------------------------------------------
    // Test 6: Zero decay (decay_mult=0) → decayed = 0
    // potential = 100, RELU
    // --------------------------------------------------------
    $display("Test 6: zero decay");
    run_neuron(32'd100, 8'h00, 32'h00000000, 1'b0, act_r, dec_r);
    check_eq(act_r, 32'd100, "T6 act_out = potential");
    check_eq(dec_r, 32'd0,   "T6 decayed = 0 with zero mult");

    // --------------------------------------------------------
    // Test 7: Near-full decay (decay_mult=0xFFFFFFFF ≈ 1.0)
    // potential = 32'd100
    // decayed = floor(100 * 0xFFFFFFFF / 2^32) = floor(100 * (1 - 2^-32)) = 99
    // --------------------------------------------------------
    $display("Test 7: near-full decay");
    run_neuron(32'd100, 8'h00, 32'hFFFFFFFF, 1'b0, act_r, dec_r);
    check_eq(act_r, 32'd100, "T7 act_out = potential");
    check_eq(dec_r, decayed(32'd100, 32'hFFFFFFFF), "T7 decayed near-full");

    // --------------------------------------------------------
    // Test 8: ABS — positive potential unchanged
    // potential = 50, decay = 0.5 → act_out = 50, decayed = 25
    // --------------------------------------------------------
    $display("Test 8: ABS positive potential");
    run_neuron(32'd50, 8'h00, 32'h80000000, 2'b10, act_r, dec_r);
    check_eq(act_r, 32'd50,                              "T8 act_out");
    check_eq(dec_r, decayed(32'd50, 32'h80000000),       "T8 decayed_act");

    // --------------------------------------------------------
    // Test 9: ABS — negative potential negated
    // potential = -30, decay = 0.5 → act_out = 30, decayed = 15
    // --------------------------------------------------------
    $display("Test 9: ABS negative potential → |potential|");
    run_neuron(-32'd30, 8'h00, 32'h80000000, 2'b10, act_r, dec_r);
    check_eq(act_r, 32'd30,                              "T9 act_out = 30");
    check_eq(dec_r, decayed(32'd30, 32'h80000000),       "T9 decayed_act");

    // --------------------------------------------------------
    // Test 10: ABS — overflow saturates to 0x7FFFFFFF
    // potential = 0x80000000 (most-negative), decay = 0.5
    // act_out = 0x7FFFFFFF; decayed = floor(0x7FFFFFFF * 0.5) = 0x3FFFFFFF
    // --------------------------------------------------------
    $display("Test 10: ABS overflow saturation");
    run_neuron(32'h80000000, 8'h00, 32'h80000000, 2'b10, act_r, dec_r);
    check_eq(act_r, 32'h7FFFFFFF,                        "T10 act_out saturated");
    check_eq(dec_r, decayed(32'h7FFFFFFF, 32'h80000000), "T10 decayed_act");

    $display("=== tb_update_state_for_neuron: %0d failure(s) ===", errors);
    if (errors == 0) $display("PASS"); else $display("FAIL");
    $finish;
end

endmodule
