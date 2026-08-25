// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
// Authors: Simon Davidson & Claude | Last modified: 2026-08-20
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
    parameter PIN_BITS    = `PIN_BITS,         /* 10 - element index width */
    /* FMAX_PIPE=1 splits each heavy single-cycle state in two, registering
     * the intermediate (partial product / shifted addends / rounded shift):
     * the ST_MUL chunk-multiply-shift-accumulate cone is the FPGA's binding
     * path once the pool/cache families are cut (~26-30 LUT levels). Costs
     * 1 extra cycle per split state per element on a unit that is <0.5%
     * busy. Default 0 = original single-cycle states, bit-identical. */
    parameter FMAX_PIPE   = 0,
    /* HU_II=1 replaces the sequential FSM with a 5-stage pipeline: the
     * initiation interval drops from (mul_total + 4) to mul_total cycles per
     * element -- 6 -> 2 for the 16-bit Z that Monarch uses. The multiply is
     * the only multi-cycle SHARED resource (8 bits of Z per cycle into one
     * accumulator), so mul_total is the floor; the other four cycles are
     * pipeline overhead and overlap away. The Hadamard costs ~9,622 cycles of
     * EVERY Monarch frame regardless of nblocks (4 calls over the 400-element
     * GRU hidden state), so this is 4.1% of an nblocks=40 frame.
     * Default 0 = original FSM, bit- and cycle-identical.
     *
     * NOT compatible with FMAX_PIPE=1: those splits exist to shorten the
     * ST_MUL cone, and splitting the multiply stage of a pipeline would
     * double II. FPGA measurement (960ff28) already showed the splits buy
     * nothing at 85 MHz after the DSP-width fix; monarch_top_asic keeps
     * FMAX_PIPE=1 for its ~302 MHz target and stays on the sequential path. */
    parameter HU_II       = 0
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

/* ---------------------------------------------------------------------------
 * Shared by both implementations (HU_II selects between them below)
 * ------------------------------------------------------------------------ */
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

localparam WIDE = DATA_BITS + 16;   /* 48-bit internal width               */
localparam ACC_BITS = WIDE + DATA_BITS + 4;  /* 84 bits                    */


/* Number of 8-bit multiply chunks for a given Z element size (1, 2 or 4).
 * Same table as the sequential branch's mul_total; a function so the pipeline
 * can evaluate it for whichever stage drives the multiplier this cycle. */
function [2:0] mul_total_for;
    input [ACT_SZ-1:0] sz;
    begin
        case (sz)
            3'd4:    mul_total_for = 3'd2;   /* 16-bit: 2 chunks  */
            3'd5:    mul_total_for = 3'd4;   /* 32-bit: 4 chunks  */
            default: mul_total_for = 3'd1;   /* 1..8-bit: 1 chunk */
        endcase
    end
endfunction

/* HU_II=1 and FMAX_PIPE=1 are mutually exclusive -- see the HU_II comment. */
// synthesis translate_off
initial if (HU_II != 0 && FMAX_PIPE != 0) begin
    $display("FATAL hu_compute: HU_II=1 is incompatible with FMAX_PIPE=1");
    $finish;
end
// synthesis translate_on

generate
if (HU_II == 0) begin : gen_hu_seq
    /* -------------------------------------------------------------------------
     * State encoding
     * ---------------------------------------------------------------------- */
    localparam ST_IDLE       = 3'd0;
    localparam ST_MUL        = 3'd1;  /* iterative multiply cycles             */
    localparam ST_ACCUM      = 3'd2;  /* add B and R_prev                      */
    localparam ST_TRUNCATE   = 3'd3;  /* align to BinPoint_R, clamp            */
    localparam ST_WAIT_PAK   = 3'd4;  /* hold result until packer ready        */
    /* FMAX_PIPE second halves of the split states (unused when FMAX_PIPE=0)   */
    localparam ST_MUL_ADD    = 3'd5;  /* accumulate the registered partial     */
    localparam ST_ACCUM_ADD  = 3'd6;  /* accumulate the registered addends     */
    localparam ST_TRUNC_CLAMP= 3'd7;  /* clamp the registered rounded shift    */

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

    /* F4/F5 fix (2026-06-07): derive from the LATCHED size esz_z_r so mul_total
     * is stable through ST_MUL (was elem_sz_z_i, the live input).             */
    always @(*) begin
        case (esz_z_r)
            3'd4:    mul_total = 3'd2;   /* 16-bit: 2 chunks                   */
            3'd5:    mul_total = 3'd4;   /* 32-bit: 4 chunks                   */
            default: mul_total = 3'd1;   /* 1..8-bit: 1 chunk                  */
        endcase
    end


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

    reg signed [WIDE-1:0] a_aligned;
    reg signed [WIDE-1:0] b_aligned;
    reg signed [WIDE-1:0] amb;          /* A - B at common binary point        */
    reg signed [WIDE-1:0] b_latch;      /* B kept at internal precision        */
    reg signed [WIDE-1:0] r_prev_latch; /* R_prev at internal precision        */

    /* Multiply accumulator: 48 + 32 + 4 bits of headroom */
    reg signed [ACC_BITS-1:0] accumulator;

    /* FMAX_PIPE intermediate registers (idle when FMAX_PIPE=0) */
    reg signed [ACC_BITS-1:0] partial_r;        /* ST_MUL      -> ST_MUL_ADD    */
    reg signed [ACC_BITS-1:0] b_sh_r, rp_sh_r;  /* ST_ACCUM    -> ST_ACCUM_ADD  */
    reg signed [ACC_BITS-1:0] shifted_r;        /* ST_TRUNCATE -> ST_TRUNC_CLAMP*/

    /* Current 8-bit chunk of Z being processed.
     * F4/F5 fix (2026-06-07): take LSB-first bytes of the RIGHT-aligned integer
     * z_val (was the top byte of left-aligned z_r, which zeroed all higher
     * chunks).  The top chunk carries the sign (two's complement); lower chunks
     * are unsigned.  The partial product (chunk*amb) is shifted into place; the
     * accumulator then adds it (previously the WHOLE running sum was shifted
     * because Verilog '+' binds tighter than '<<').                            */
    wire [7:0] z_chunk;
    assign z_chunk = z_val >> ({mul_count, 3'b000});      /* byte mul_count of z_val */
    wire is_top_chunk = (mul_count == mul_total - 1'b1);
    /* 2026-08-20: multiply operands are declared at their REAL widths, not at the
     * accumulator width.  Previously both were sign-extended to ACC_BITS first, so
     * this read as an 84 x 84 signed multiply; Vivado did not prune it back to the
     * 8 significant bits of the chunk and spent 12 DSP48E2 on it -- 57% of every
     * DSP in the whole design, for a multiply that needs about 2.  Narrowing to
     * 9 x 49 -> 58 bits is the SAME arithmetic (the product cannot exceed 58 bits,
     * so the old truncation to ACC_BITS never removed anything) and measures
     * 12 -> 3 DSP48E2, 45 -> 41 CARRY8, 2202 -> 2126 LUT, FF unchanged, in
     * out-of-context synthesis on xck26.  Verified bit-identical by tb_hu_compute's
     * full suite (20,059 checks against the software golden, in both FMAX_PIPE
     * modes) and by full-chip bit-exactness.
     * Sign convention is unchanged: the top chunk is signed, lower chunks unsigned. */
    wire signed [8:0]          chunk_n = is_top_chunk
        ? $signed({z_chunk[7], z_chunk})                   /* sign-extend top byte   */
        : $signed({1'b0,       z_chunk});                  /* zero-extend low bytes  */
    wire signed [WIDE:0]       amb_n   = $signed({amb[WIDE-1], amb});
    wire signed [WIDE+9:0]     prod_n  = chunk_n * amb_n;  /* 9 x 49 -> 58 bits      */
    wire signed [ACC_BITS-1:0] partial =
        $signed({{(ACC_BITS-WIDE-10){prod_n[WIDE+9]}}, prod_n}) << ({mul_count, 3'b000});

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

                if (FMAX_PIPE) begin
                    /* Split: register the shifted partial product this cycle,
                     * accumulate it in ST_MUL_ADD. mul_count stays put so the
                     * registered value corresponds to this chunk.              */
                    partial_r <= partial;
                    state     <= ST_MUL_ADD;
                end else begin
                    /* Add the (correctly shifted) partial product. After all
                     * chunks, accumulator = z_val * amb at binary point (bp_z+HALF). */
                    accumulator <= accumulator + partial;

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
            end

            /* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */
            ST_MUL_ADD: begin  /* FMAX_PIPE only: accumulate the registered chunk */
                accumulator <= accumulator + partial_r;

                if (mul_count == mul_total - 1'b1) begin
                    b_latch     <= b_aligned;
                    r_prev_latch <= ($signed({{(WIDE-DATA_BITS){r_prev_r[DATA_BITS-1]}},
                                              r_prev_r})
                                     >>> right_shift_for_sz(esz_r_r))
                                    << (WIDE/2 - bp_r_r);
                    mul_count   <= 3'd0;
                    state       <= ST_ACCUM;
                end else begin
                    mul_count <= mul_count + 1'b1;
                    state     <= ST_MUL;
                end
            end

            /* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */
            ST_ACCUM: begin
                /* accumulator holds Z*(A-B) at binary point (bp_z + HALF).
                 * F4/F5 fix (2026-06-07): add B (and R_prev) at the SAME binary
                 * point. b_latch/r_prev_latch are at HALF, so shift by bp_z_r
                 * (was << HALF, which left the multiply term 24 bits too low and
                 * truncated it away).                                          */
                if (FMAX_PIPE) begin
                    /* Split: register the shifted addends, add in ST_ACCUM_ADD */
                    b_sh_r  <= $signed({{(ACC_BITS-WIDE){b_latch[WIDE-1]}}, b_latch})
                               << bp_z_r;
                    rp_sh_r <= mode_r ? ($signed({{(ACC_BITS-WIDE){r_prev_latch[WIDE-1]}},
                                                  r_prev_latch}) << bp_z_r)
                                      : {ACC_BITS{1'b0}};
                    state   <= ST_ACCUM_ADD;
                end else begin
                    accumulator <= accumulator
                                 + ($signed({{(ACC_BITS-WIDE){b_latch[WIDE-1]}}, b_latch})
                                    << bp_z_r)
                                 + (mode_r ? ($signed({{(ACC_BITS-WIDE){r_prev_latch[WIDE-1]}},
                                                       r_prev_latch}) << bp_z_r)
                                           : {ACC_BITS{1'b0}});
                    state <= ST_TRUNCATE;
                end
            end

            /* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */
            ST_ACCUM_ADD: begin  /* FMAX_PIPE only */
                accumulator <= accumulator + b_sh_r + rp_sh_r;
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
                    integer                    out_shift;   /* signed */
                    reg signed [ACC_BITS-1:0]  round_bias;

                    /* F4/F5 fix (2026-06-07): accumulator is at binary point
                     * (bp_z + HALF). Shift right by (bp_z + HALF - bp_r) to land
                     * at bp_r, with round-half-up. (Was WIDE/2 + (WIDE/2 - bp_r),
                     * i.e. 48 - bp_r, which assumed the multiply term was at 48.) */
                    out_shift = (WIDE/2) + bp_z_r - bp_r_r;
                    if (out_shift > 0) begin
                        round_bias = {{(ACC_BITS-1){1'b0}}, 1'b1} <<< (out_shift - 1);
                        shifted    = (accumulator + round_bias) >>> out_shift;
                    end else begin
                        /* bp_r > bp_z+HALF (atypical): scale up, no rounding */
                        shifted    = accumulator <<< (-out_shift);
                    end

                    if (FMAX_PIPE) begin
                        /* Split: register the rounded shift, clamp next cycle */
                        shifted_r <= shifted;
                        state     <= ST_TRUNC_CLAMP;
                    end else begin
                        /* Element range limits (signed, element bits wide)    */
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
                        state          <= ST_WAIT_PAK;
                    end
                end
            end

            /* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */
            ST_TRUNC_CLAMP: begin  /* FMAX_PIPE only: clamp the registered shift */
                begin : clamp_block
                    reg signed [DATA_BITS-1:0] clamped;
                    reg signed [DATA_BITS-1:0] max_val;
                    reg signed [DATA_BITS-1:0] min_val;

                    case (esz_r_r)
                        3'd0: begin max_val = 32'sh00000000; min_val = 32'shffffffff; end /* 1-bit signed: 0/-1 */
                        3'd1: begin max_val = 32'sh00000001; min_val = 32'shfffffffe; end /* 2-bit: 1/-2 */
                        3'd2: begin max_val = 32'sh00000007; min_val = 32'shfffffff8; end /* 4-bit: 7/-8 */
                        3'd3: begin max_val = 32'sh0000007f; min_val = 32'hffffff80; end  /* 8-bit */
                        3'd4: begin max_val = 32'sh00007fff; min_val = 32'hffff8000; end  /* 16-bit */
                        default: begin max_val = 32'sh7fffffff; min_val = 32'h80000000; end /* 32-bit */
                    endcase

                    if ($signed(shifted_r[DATA_BITS-1:0]) > max_val) begin
                        clamped  = max_val;
                        over_r_o <= 1'b1;
                    end else if ($signed(shifted_r[DATA_BITS-1:0]) < min_val) begin
                        clamped  = min_val;
                        under_r_o <= 1'b1;
                    end else begin
                        clamped = shifted_r[DATA_BITS-1:0];
                    end

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
    end else begin : gen_hu_pipe
    /* =====================================================================
     * HU_II=1: 5-stage pipeline, II = mul_total (2 for the 16-bit Z Monarch
     * uses) instead of mul_total + 4.
     *
     *   A  align     1 cyc   latch inputs; form amb / b_aligned / rp_aligned
     *   B  multiply  mul_total cycles -- THE ONLY multi-cycle shared resource
     *   C  accum     1 cyc   += (B << bp_z) + (mode ? R_prev << bp_z : 0)
     *   D  truncate  1 cyc   round-half-up shift, clamp, re-left-align
     *   E  write     1 cyc   strobe the packer
     *
     * Latency is unchanged at mul_total + 4; only the INITIATION INTERVAL
     * changes, because A/C/D/E each hold an element for one cycle while B
     * holds one for mul_total.
     *
     * EVERY ARITHMETIC EXPRESSION IS LIFTED VERBATIM from the sequential
     * branch above -- the left-alignment strip, the << (WIDE/2 - bp)
     * alignment, the F4/F5 binary-point convention (accumulator sits at
     * bp_z + HALF; addends shift by bp_z; out_shift = WIDE/2 + bp_z - bp_r),
     * the signed-top / unsigned-low chunk convention, the round-half-up bias
     * and the clamp table. tb_hu_compute's independent golden is the gate.
     * ================================================================== */

    /* ---- stage A: aligned operands, one element ---------------------- */
    reg                          a_vld;
    reg signed [WIDE-1:0]        a_amb, a_b, a_rp;
    reg signed [DATA_BITS-1:0]   a_zval;
    reg [BINPT_SZ-1:0]           a_bpz, a_bpr;
    reg [ACT_SZ-1:0]             a_eszz, a_eszr;
    reg                          a_mode;
    reg [PIN_BITS-1:0]           a_idx;
    reg                          a_lst;

    /* ---- stage B: the shared multiplier ------------------------------ */
    reg                          b_vld;      /* chunks still outstanding   */
    reg [2:0]                    b_cnt;      /* chunks ISSUED so far       */
    reg signed [ACC_BITS-1:0]    b_acc;
    reg signed [WIDE-1:0]        b_amb, b_b, b_rp;
    reg signed [DATA_BITS-1:0]   b_zval;
    reg [BINPT_SZ-1:0]           b_bpz, b_bpr;
    reg [ACT_SZ-1:0]             b_eszz, b_eszr;
    reg                          b_mode;
    reg [PIN_BITS-1:0]           b_idx;
    reg                          b_lst;

    /* ---- stage C: Z*(A-B), awaiting the B/R_prev addition ------------ */
    reg                          c_vld;
    reg signed [ACC_BITS-1:0]    c_acc;
    reg signed [WIDE-1:0]        c_b, c_rp;
    reg [BINPT_SZ-1:0]           c_bpz, c_bpr;
    reg [ACT_SZ-1:0]             c_eszr;
    reg                          c_mode;
    reg [PIN_BITS-1:0]           c_idx;
    reg                          c_lst;

    /* ---- stage D: full sum, awaiting truncate/clamp ------------------ */
    reg                          d_vld;
    reg signed [ACC_BITS-1:0]    d_acc;
    reg [BINPT_SZ-1:0]           d_bpz, d_bpr;
    reg [ACT_SZ-1:0]             d_eszr;
    reg [PIN_BITS-1:0]           d_idx;
    reg                          d_lst;

    /* ---- stage E: packed result, awaiting the packer ----------------- */
    reg                          e_vld;
    reg [DATA_BITS-1:0]          e_res;
    reg [PIN_BITS-1:0]           e_idx;
    reg                          e_lst;

    /* ---- flow control ------------------------------------------------
     * The packer is the only thing that can back-pressure, and the
     * HU_PROFILE run measured it never doing so in the Monarch workload, so
     * a whole-pipeline freeze is both correct and much simpler than
     * per-stage skid buffers.  tbHuComputeBp.bsh exercises it deliberately. */
    wire p_stall  = e_vld & pak_full_i;
    wire a_accept = ready_o & valid_i & ~p_stall;
    wire b_take   = a_vld & ~b_vld & ~p_stall;   /* A -> B this cycle      */
    wire a_vld_nxt = b_take ? a_accept : (a_vld | a_accept);

    /* ---- the shared multiplier --------------------------------------
     * On the A->B transfer cycle the operands come straight from stage A's
     * registers, so chunk 0 issues in the SAME cycle as the transfer and the
     * multiplier is busy exactly mul_total cycles per element. Registering
     * into B first and starting the next cycle would make it mul_total + 1
     * and the whole II argument would collapse. */
    wire signed [WIDE-1:0]      mul_amb  = b_take ? a_amb  : b_amb;
    wire signed [DATA_BITS-1:0] mul_zval = b_take ? a_zval : b_zval;
    wire [2:0]                  mul_cnt  = b_take ? 3'd0   : b_cnt;
    wire [2:0]                  mul_tot  = mul_total_for(b_take ? a_eszz : b_eszz);

    wire [7:0] mul_chunk   = mul_zval >> ({mul_cnt, 3'b000});
    wire       mul_is_top  = (mul_cnt == mul_tot - 1'b1);
    wire signed [8:0] mul_chunk_n = mul_is_top
        ? $signed({mul_chunk[7], mul_chunk})     /* sign-extend top byte    */
        : $signed({1'b0,         mul_chunk});    /* zero-extend low bytes   */
    wire signed [WIDE:0]   mul_amb_n = $signed({mul_amb[WIDE-1], mul_amb});
    wire signed [WIDE+9:0] mul_prod  = mul_chunk_n * mul_amb_n;  /* 9 x 49  */
    wire signed [ACC_BITS-1:0] mul_partial =
        $signed({{(ACC_BITS-WIDE-10){mul_prod[WIDE+9]}}, mul_prod})
        << ({mul_cnt, 3'b000});

    wire signed [ACC_BITS-1:0] mul_acc_nxt =
        (b_take ? {ACC_BITS{1'b0}} : b_acc) + mul_partial;
    wire mul_active = (b_take | b_vld) & ~p_stall;
    wire mul_done   = mul_active & (mul_cnt == mul_tot - 1'b1);

    /* ---- stage C combinational: add B and R_prev at bp_z -------------- */
    wire signed [ACC_BITS-1:0] c_sum = c_acc
        + ($signed({{(ACC_BITS-WIDE){c_b[WIDE-1]}}, c_b}) << c_bpz)
        + (c_mode ? ($signed({{(ACC_BITS-WIDE){c_rp[WIDE-1]}}, c_rp}) << c_bpz)
                  : {ACC_BITS{1'b0}});

    always @(posedge clk) begin
        if (reset) begin
            ready_o     <= 1'b1;
            pak_write_o <= 1'b0;
            over_r_o    <= 1'b0;
            under_r_o   <= 1'b0;
            a_vld <= 1'b0; b_vld <= 1'b0; c_vld <= 1'b0;
            d_vld <= 1'b0; e_vld <= 1'b0;
            b_cnt <= 3'd0; b_acc <= {ACC_BITS{1'b0}};
        end else begin
            pak_write_o <= 1'b0;
            ready_o     <= ~a_vld_nxt & ~p_stall;

            /* ---------------- stage A ---------------------------------- */
            if (a_accept) begin
                /* Same expressions as ST_IDLE, from the live inputs. */
                a_amb  <= (($signed({{(WIDE-DATA_BITS){a_i[DATA_BITS-1]}}, a_i})
                             >>> right_shift_for_sz(elem_sz_ab_i))
                            << (WIDE/2 - bin_point_a_i))
                        - (($signed({{(WIDE-DATA_BITS){b_i[DATA_BITS-1]}}, b_i})
                             >>> right_shift_for_sz(elem_sz_ab_i))
                            << (WIDE/2 - bin_point_b_i));
                a_b    <= ($signed({{(WIDE-DATA_BITS){b_i[DATA_BITS-1]}}, b_i})
                            >>> right_shift_for_sz(elem_sz_ab_i))
                           << (WIDE/2 - bin_point_b_i);
                /* r_prev_latch, computed in ST_MUL's last cycle there. */
                a_rp   <= ($signed({{(WIDE-DATA_BITS){r_prev_i[DATA_BITS-1]}}, r_prev_i})
                            >>> right_shift_for_sz(elem_sz_r_i))
                           << (WIDE/2 - bin_point_r_i);
                a_zval <= $signed(z_i) >>> right_shift_for_sz(elem_sz_z_i);
                a_bpz  <= bin_point_z_i;  a_bpr  <= bin_point_r_i;
                a_eszz <= elem_sz_z_i;    a_eszr <= elem_sz_r_i;
                a_mode <= mode_i;         a_idx  <= index_i;  a_lst <= last_i;
            end
            if (!p_stall) a_vld <= a_vld_nxt;

            /* ---------------- stage B: multiply ------------------------ */
            if (mul_active) begin
                if (mul_done) begin
                    b_vld <= 1'b0;              /* element leaves B        */
                end else begin
                    b_vld <= 1'b1;
                    b_cnt <= mul_cnt + 1'b1;
                    b_acc <= mul_acc_nxt;
                    if (b_take) begin           /* carry A's fields into B */
                        b_amb  <= a_amb;  b_zval <= a_zval;
                        b_b    <= a_b;    b_rp   <= a_rp;
                        b_bpz  <= a_bpz;  b_bpr  <= a_bpr;
                        b_eszz <= a_eszz; b_eszr <= a_eszr;
                        b_mode <= a_mode; b_idx  <= a_idx;  b_lst <= a_lst;
                    end
                end
            end

            /* ---------------- B -> C ----------------------------------- */
            if (!p_stall) c_vld <= mul_done;
            if (mul_done) begin
                c_acc  <= mul_acc_nxt;
                c_b    <= b_take ? a_b    : b_b;
                c_rp   <= b_take ? a_rp   : b_rp;
                c_bpz  <= b_take ? a_bpz  : b_bpz;
                c_bpr  <= b_take ? a_bpr  : b_bpr;
                c_eszr <= b_take ? a_eszr : b_eszr;
                c_mode <= b_take ? a_mode : b_mode;
                c_idx  <= b_take ? a_idx  : b_idx;
                c_lst  <= b_take ? a_lst  : b_lst;
            end

            /* ---------------- stage C -> D: accumulate ----------------- */
            if (!p_stall) begin
                d_vld <= c_vld;
                if (c_vld) begin
                    d_acc  <= c_sum;
                    d_bpz  <= c_bpz;  d_bpr <= c_bpr;  d_eszr <= c_eszr;
                    d_idx  <= c_idx;  d_lst <= c_lst;
                end
            end

            /* ---------------- stage D -> E: truncate and clamp --------- */
            if (!p_stall) begin
                e_vld <= d_vld;
                if (d_vld) begin : pipe_trunc
                    reg signed [ACC_BITS-1:0] shifted;
                    reg signed [ACC_BITS-1:0] round_bias;
                    reg signed [DATA_BITS-1:0] clamped, max_val, min_val;
                    integer                    out_shift;

                    out_shift = (WIDE/2) + d_bpz - d_bpr;
                    if (out_shift > 0) begin
                        round_bias = {{(ACC_BITS-1){1'b0}}, 1'b1} <<< (out_shift - 1);
                        shifted    = (d_acc + round_bias) >>> out_shift;
                    end else begin
                        shifted    = d_acc <<< (-out_shift);
                    end

                    case (d_eszr)
                        3'd0: begin max_val = 32'sh00000000; min_val = 32'shffffffff; end
                        3'd1: begin max_val = 32'sh00000001; min_val = 32'shfffffffe; end
                        3'd2: begin max_val = 32'sh00000007; min_val = 32'shfffffff8; end
                        3'd3: begin max_val = 32'sh0000007f; min_val = 32'hffffff80; end
                        3'd4: begin max_val = 32'sh00007fff; min_val = 32'hffff8000; end
                        default: begin max_val = 32'sh7fffffff; min_val = 32'h80000000; end
                    endcase

                    if ($signed(shifted[DATA_BITS-1:0]) > max_val) begin
                        clamped  = max_val;
                        over_r_o <= 1'b1;       /* sticky, per stream      */
                    end else if ($signed(shifted[DATA_BITS-1:0]) < min_val) begin
                        clamped   = min_val;
                        under_r_o <= 1'b1;
                    end else begin
                        clamped = shifted[DATA_BITS-1:0];
                    end

                    e_res <= clamped << right_shift_for_sz(d_eszr);
                    e_idx <= d_idx;
                    e_lst <= d_lst;
                end
            end

            /* ---------------- stage E: packer -------------------------- */
            if (e_vld && !pak_full_i) begin
                pak_write_o <= 1'b1;
                pak_data_o  <= e_res;
                pak_index_o <= e_idx;
                pak_last_o  <= e_lst;
            end
        end
    end
    end
    endgenerate

endmodule
/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
