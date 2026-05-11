// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
/* Hadamard compute: R[i] = Z[i]*(A[i]-B[i]) + B[i] + mode*R_prev[i]
 *
 * One shared iterative multiplier. Number of multiply cycles:
 *   elem_sz 0..3 (1..8-bit)  -> 1 cycle
 *   elem_sz 4   (16-bit)     -> 2 cycles
 *   elem_sz 5   (32-bit)     -> 4 cycles
 *
 * All data arrives left-aligned (value in MSBs), as produced by
 * slice_and_align and expected by packer.
 */
`include "../shared/constants.v"

module hu_compute #(
    parameter DATA_BITS   = 32,
    parameter ACT_SZ      = `POT_OUT_SZ_SZ,   /* 3 */
    parameter BINPT_SZ    = 5,
    parameter PIN_BITS    = `PIN_BITS          /* 10 - element index width */
)(
    input  wire                  clk,
    input  wire                  reset,

    /* Handshake with synchroniser */
    input  wire                  valid_i,      /* quad of elements ready    */
    output reg                   ready_o,      /* we can accept this cycle  */

    /* Element data (left-aligned, MSBs hold the value) */
    input  wire [DATA_BITS-1:0]  a_i,
    input  wire [DATA_BITS-1:0]  b_i,
    input  wire [DATA_BITS-1:0]  z_i,
    input  wire [DATA_BITS-1:0]  r_prev_i,
    input  wire [PIN_BITS-1:0]   index_i,
    input  wire                  last_i,

    /* Binary point configuration */
    input  wire [BINPT_SZ-1:0]   bin_point_a_i,
    input  wire [BINPT_SZ-1:0]   bin_point_b_i,
    input  wire [BINPT_SZ-1:0]   bin_point_z_i,
    input  wire [BINPT_SZ-1:0]   bin_point_r_i,

    /* Element size configuration */
    input  wire [ACT_SZ-1:0]     elem_sz_ab_i, /* A and B share a size     */
    input  wire [ACT_SZ-1:0]     elem_sz_z_i,
    input  wire [ACT_SZ-1:0]     elem_sz_r_i,

    /* Mode */
    input  wire                  mode_i,       /* 1 = update, 0 = init      */

    /* Output to packer (left-aligned) */
    output reg                   pak_write_o,
    input  wire                  pak_full_i,
    output reg  [DATA_BITS-1:0]  pak_data_o,
    output reg  [PIN_BITS-1:0]   pak_index_o,
    output reg                   pak_last_o,

    /* Sticky overflow / underflow bits */
    output reg                   over_r_o,
    output reg                   under_r_o
);

/* -------------------------------------------------------------------------
 * State encoding
 * ---------------------------------------------------------------------- */
localparam ST_IDLE       = 3'd0;
localparam ST_MUL        = 3'd1;  /* iterative multiply cycles             */
localparam ST_ACCUM      = 3'd2;  /* add B and R_prev                      */
localparam ST_TRUNCATE   = 3'd3;  /* align to BinPoint_R, clamp            */
localparam ST_WAIT_PAK   = 3'd4;  /* hold result until packer ready        */

reg [2:0] state;

/* -------------------------------------------------------------------------
 * Latched inputs
 * ---------------------------------------------------------------------- */
reg [DATA_BITS-1:0]  a_r, b_r, z_r, r_prev_r;
reg [PIN_BITS-1:0]   index_r;
reg                  last_r;
reg [BINPT_SZ-1:0]   bp_a_r, bp_b_r, bp_z_r, bp_r_r;
reg [ACT_SZ-1:0]     esz_ab_r, esz_z_r, esz_r_r;
reg                  mode_r;

/* -------------------------------------------------------------------------
 * Intermediate signals
 * ---------------------------------------------------------------------- */

/* Number of multiply cycles needed (1, 2, or 4), derived from Z elem size */
reg [2:0] mul_total;   /* set at latch time                                */
reg [2:0] mul_count;   /* counts up during ST_MUL                         */

always @(*) begin
    case (elem_sz_z_i)
        3'd4:    mul_total = 3'd2;   /* 16-bit: 2 chunks                   */
        3'd5:    mul_total = 3'd4;   /* 32-bit: 4 chunks                   */
        default: mul_total = 3'd1;   /* 1..8-bit: 1 chunk                  */
    endcase
end

/* Right-shift to strip left-alignment before arithmetic.
 * Data is stored left-aligned (value in [31: 32-elem_bits]).
 * Shift right by (32 - elem_bits) to get value in LSBs for arithmetic. */
function [5:0] right_shift_for_sz;
    input [ACT_SZ-1:0] sz;
    begin
        case (sz)
            3'd0: right_shift_for_sz = 6'd31; /*  1-bit */
            3'd1: right_shift_for_sz = 6'd30; /*  2-bit */
            3'd2: right_shift_for_sz = 6'd28; /*  4-bit */
            3'd3: right_shift_for_sz = 6'd24; /*  8-bit */
            3'd4: right_shift_for_sz = 6'd16; /* 16-bit */
            3'd5: right_shift_for_sz = 6'd0;  /* 32-bit */
            default: right_shift_for_sz = 6'd0;
        endcase
    end
endfunction

/* Sign-extended right-shifted values (arithmetic, value in LSBs) */
wire signed [DATA_BITS-1:0] a_val, b_val, z_val, r_prev_val;

assign a_val      = $signed(a_r)      >>> right_shift_for_sz(esz_ab_r);
assign b_val      = $signed(b_r)      >>> right_shift_for_sz(esz_ab_r);
assign z_val      = $signed(z_r)      >>> right_shift_for_sz(esz_z_r);
assign r_prev_val = $signed(r_prev_r) >>> right_shift_for_sz(esz_r_r);

/* A-B difference: computed combinatorially when entering ST_MUL.
 * Aligned so both operands are at the same binary point.
 * If bin_point_a != bin_point_b we shift the one with fewer integer bits.
 * For simplicity we work at a common internal precision of DATA_BITS+16
 * to avoid losing fractional bits.                                        */
localparam WIDE = DATA_BITS + 16;   /* 48-bit internal width               */

reg signed [WIDE-1:0] a_aligned;
reg signed [WIDE-1:0] b_aligned;
reg signed [WIDE-1:0] amb;          /* A - B at common binary point        */
reg signed [WIDE-1:0] b_latch;      /* B kept at internal precision        */
reg signed [WIDE-1:0] r_prev_latch; /* R_prev at internal precision        */

/* Multiply accumulator: 48 + 32 + 4 bits of headroom */
localparam ACC_BITS = WIDE + DATA_BITS + 4;  /* 84 bits                    */
reg signed [ACC_BITS-1:0] accumulator;

/* Current 8-bit chunk of Z being processed */
wire [7:0] z_chunk;
assign z_chunk = z_r[DATA_BITS-1 -: 8] >> ({mul_count, 3'b000});
/* Shift z_r right by (mul_count * 8) to get the current low byte.        */
/* Since z_r is left-aligned we rotate through MSB-first.                 */

/* -------------------------------------------------------------------------
 * Result holding register (ST_WAIT_PAK)
 * ---------------------------------------------------------------------- */
reg [DATA_BITS-1:0] result_r;
reg [PIN_BITS-1:0]  result_index_r;
reg                 result_last_r;

/* -------------------------------------------------------------------------
 * FSM
 * ---------------------------------------------------------------------- */
always @(posedge clk) begin
    if (reset) begin
        state       <= ST_IDLE;
        ready_o     <= 1'b1;
        pak_write_o <= 1'b0;
        over_r_o    <= 1'b0;
        under_r_o   <= 1'b0;
        mul_count   <= 3'd0;
    end else begin
        pak_write_o <= 1'b0;  /* default: no write strobe                  */

        case (state)

        /* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */
        ST_IDLE: begin
            ready_o <= 1'b1;
            if (valid_i) begin
                /* Latch all inputs */
                a_r        <= a_i;
                b_r        <= b_i;
                z_r        <= z_i;
                r_prev_r   <= r_prev_i;
                index_r    <= index_i;
                last_r     <= last_i;
                bp_a_r     <= bin_point_a_i;
                bp_b_r     <= bin_point_b_i;
                bp_z_r     <= bin_point_z_i;
                bp_r_r     <= bin_point_r_i;
                esz_ab_r   <= elem_sz_ab_i;
                esz_z_r    <= elem_sz_z_i;
                esz_r_r    <= elem_sz_r_i;
                mode_r     <= mode_i;
                ready_o    <= 1'b0;
                mul_count  <= 3'd0;
                accumulator <= {ACC_BITS{1'b0}};

                /* Pre-compute A and B at common internal binary point.
                 * We extend both to WIDE bits then shift so the binary
                 * point is at bit (WIDE/2). bp_* registers hold the
                 * position of the binary point from the MSB of the
                 * element field.  We convert: shift left by
                 * (WIDE/2 - bp) to place the binary point at WIDE/2.  */
                a_aligned <= ($signed({{(WIDE-DATA_BITS){a_i[DATA_BITS-1]}}, a_i})
                              >>> right_shift_for_sz(elem_sz_ab_i))
                             << (WIDE/2 - bin_point_a_i);
                b_aligned <= ($signed({{(WIDE-DATA_BITS){b_i[DATA_BITS-1]}}, b_i})
                              >>> right_shift_for_sz(elem_sz_ab_i))
                             << (WIDE/2 - bin_point_b_i);
                /* Pre-compute amb here so it's settled for ST_MUL cycle 0.
                 * Reading a_aligned/b_aligned in ST_MUL cycle 0 would read
                 * the old (potentially X) values because the NBAs above have
                 * not yet committed when the accumulator multiply runs. */
                amb <= (($signed({{(WIDE-DATA_BITS){a_i[DATA_BITS-1]}}, a_i})
                          >>> right_shift_for_sz(elem_sz_ab_i))
                         << (WIDE/2 - bin_point_a_i))
                      - (($signed({{(WIDE-DATA_BITS){b_i[DATA_BITS-1]}}, b_i})
                           >>> right_shift_for_sz(elem_sz_ab_i))
                          << (WIDE/2 - bin_point_b_i));

                state <= ST_MUL;
            end
        end

        /* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */
        ST_MUL: begin
            /* amb was pre-computed in ST_IDLE; no update needed on cycle 0 */

            /* Partial product: current 8-bit chunk of Z * amb, shifted   */
            /* by (mul_count * 8) to account for chunk position.          */
            /* z_chunk gives bits [(mul_count+1)*8-1 : mul_count*8] of Z. */
            accumulator <= accumulator +
                ($signed({{(ACC_BITS-8){z_chunk[7]}}, z_chunk}) *
                 $signed({{(ACC_BITS-WIDE){amb[WIDE-1]}}, amb}))
                << ({mul_count, 3'b000});

            if (mul_count == mul_total - 1'b1) begin
                /* All chunks done: prepare B and R_prev for ACCUM */
                b_latch     <= b_aligned;
                r_prev_latch <= ($signed({{(WIDE-DATA_BITS){r_prev_r[DATA_BITS-1]}},
                                          r_prev_r})
                                 >>> right_shift_for_sz(esz_r_r))
                                << (WIDE/2 - bp_r_r);
                mul_count   <= 3'd0;
                state       <= ST_ACCUM;
            end else begin
                mul_count <= mul_count + 1'b1;
            end
        end

        /* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */
        ST_ACCUM: begin
            /* accumulator holds Z*(A-B) at double the internal precision.
             * Add B and optionally R_prev, both at internal precision.   */
            accumulator <= accumulator
                         + ($signed({{(ACC_BITS-WIDE){b_latch[WIDE-1]}}, b_latch})
                            << (WIDE/2))
                         + (mode_r ? ($signed({{(ACC_BITS-WIDE){r_prev_latch[WIDE-1]}},
                                               r_prev_latch}) << (WIDE/2))
                                   : {ACC_BITS{1'b0}});
            state <= ST_TRUNCATE;
        end

        /* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */
        ST_TRUNCATE: begin
            /* Shift accumulator to align output to BinPoint_R and extract
             * the element-sized field. Clamp to element range.           */
            begin : trunc_block
                reg signed [ACC_BITS-1:0] shifted;
                reg signed [DATA_BITS-1:0] clamped;
                reg signed [DATA_BITS-1:0] max_val;
                reg signed [DATA_BITS-1:0] min_val;
                reg [5:0] out_shift;

                /* Shift accumulator right to bring result to DATA_BITS    */
                out_shift = WIDE/2 + (WIDE/2 - bp_r_r);
                shifted   = accumulator >>> out_shift;

                /* Element range limits (signed, element bits wide)        */
                case (esz_r_r)
                    3'd0: begin max_val = 32'sh00000000; min_val = 32'shffffffff; end /* 1-bit signed: 0/-1 */
                    3'd1: begin max_val = 32'sh00000001; min_val = 32'shfffffffe; end /* 2-bit: 1/-2 */
                    3'd2: begin max_val = 32'sh00000007; min_val = 32'shfffffff8; end /* 4-bit: 7/-8 */
                    3'd3: begin max_val = 32'sh0000007f; min_val = 32'hffffff80; end  /* 8-bit */
                    3'd4: begin max_val = 32'sh00007fff; min_val = 32'hffff8000; end  /* 16-bit */
                    default: begin max_val = 32'sh7fffffff; min_val = 32'h80000000; end /* 32-bit */
                endcase

                if ($signed(shifted[DATA_BITS-1:0]) > max_val) begin
                    clamped  = max_val;
                    over_r_o <= 1'b1;
                end else if ($signed(shifted[DATA_BITS-1:0]) < min_val) begin
                    clamped  = min_val;
                    under_r_o <= 1'b1;
                end else begin
                    clamped = shifted[DATA_BITS-1:0];
                end

                /* Left-align result for packer: shift value to MSBs */
                result_r       <= clamped << right_shift_for_sz(esz_r_r);
                result_index_r <= index_r;
                result_last_r  <= last_r;
            end
            state <= ST_WAIT_PAK;
        end

        /* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */
        ST_WAIT_PAK: begin
            if (!pak_full_i) begin
                pak_write_o <= 1'b1;
                pak_data_o  <= result_r;
                pak_index_o <= result_index_r;
                pak_last_o  <= result_last_r;
                state       <= ST_IDLE;
                ready_o     <= 1'b1;
            end
        end

        default: state <= ST_IDLE;
        endcase
    end
end

endmodule
/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
