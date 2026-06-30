// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
`include "../shared/constants.v"

module weight_generator
                        # (
			   parameter X_INPUT_SZ         = 8,
                           parameter Y_INPUT_SZ         = 8,
			   parameter X_OUTPUT_SZ        = 8,
                           parameter Y_OUTPUT_SZ        = 8,
                           parameter X_KERNEL_SZ        = 3,
                           parameter Y_KERNEL_SZ        = 3,
                           parameter X_KERNEL_OFF_SZ    = 3,
                           parameter Y_KERNEL_OFF_SZ    = 3,
                           parameter X_STEP_SZ          = 3,
                           parameter Y_STEP_SZ          = 3,
			   parameter WEIGHT_ENTRY_BITS  = 8,
                           parameter ELEMS_PER_ROW      = 4,
                           parameter ROWS_PER_NEURON    = 16,
                           parameter IN_DATA_BITS       = 32,
                           parameter ELEM_SZ            = 8,
			   parameter ACT_IDX_SZ         = 8, // 2^8 = 256-bit
			   parameter ACT_DATA_SZ        = 8, // 2^8 = 256-bit
			   parameter WEIGHT_IDX_SZ      = 16,// MC: widened 5 -> 16 to span cout*Cin*K
                           parameter WEIGHT_SLICE_SZ    = 3, // 2^3 =   8-bit
                           parameter WEIGHT_DATA_IDX_SZ = 5, // 2^5 =  32-bit
                           parameter CIN_SZ             = 7, // MC: input-channel width
                           parameter COUT_SZ            = 7, // MC: output-channel width
		           parameter MEM_ADDR_BITS      = `ADDR_SIZE)
   (
    input  wire                    clk,
    input  wire                    reset,

    // Interface to control signals and status for start/stop:
    input  wire                       start_new_block_i,     // Pulse  in
    input  wire                       running_i,             // Steady in
    output wire                       finished_one_pass_o,   // Pulse  out for 
                                                             // one ip neuron
    output wire                       finished_pass_o,       // Pulse  out
    output wire                       running_weight_pass_o, // Steady out
    
    input  wire                 [1:0] weight_mode_i,     // 00 = full,01 = sparse,
                                                         // 10 = convolutional
    input  wire      [X_INPUT_SZ-1:0] in_x_len_i,        // Vector len in 'full'
    input  wire      [Y_INPUT_SZ-1:0] in_y_len_i,        // == '1' in 'full'
    input  wire     [X_OUTPUT_SZ-1:0] out_x_len_i,       // Vector len in 'full'
    input  wire     [Y_OUTPUT_SZ-1:0] out_y_len_i,       // == '1' in 'full'
    input  wire     [X_KERNEL_SZ-1:0] x_kernel_len_i,    // Conv only - size of kernel
    input  wire     [Y_KERNEL_SZ-1:0] y_kernel_len_i,    // Conv only - size of kernel
    input  wire [X_KERNEL_OFF_SZ-1:0] x_kernel_offset_i, // Conv only - kernel x origin offset
    input  wire [Y_KERNEL_OFF_SZ-1:0] y_kernel_offset_i, // Conv only - kernel y origin offset
    input  wire       [X_STEP_SZ-1:0] x_kernel_step_i,   // Conv only - step in x-dim
    input  wire       [Y_STEP_SZ-1:0] y_kernel_step_i,   // Conv only - step in y-dim
    input  wire      [`ADDR_SIZE-1:0] weight_base_addr_i,
    input  wire [WEIGHT_SLICE_SZ-1:0] weight_sz_i,
    input  wire [WEIGHT_SLICE_SZ-1:0] tuple_sz_i,        // Sparse only - tuple slice size
    input  wire        [`PIN_BITS-1:0] sparse_count_i,   // Sparse only - tuples per input neuron

    input  wire    [WEIGHT_ENTRY_BITS-1:0] weight_entry_bits_i,// Bits per entry (weight+index, if any)
    input  wire      [ELEMS_PER_ROW-1:0] weights_per_word_i,   // How many weight words 
                                                            // in 32-bits
    input  wire    [ROWS_PER_NEURON-1:0] rows_per_neuron_i, // Used to calculate
	                                                 // start address for rows
    input  wire             [CIN_SZ-1:0] cin_len_i,         // MC: input  channels (1 = legacy)
    input  wire            [COUT_SZ-1:0] cout_len_i,        // MC: output channels (1 = legacy)
    input wire                           act_data_valid_i,
    input wire          [ACT_IDX_SZ-1:0] act_data_idx_i,
    input wire          [X_INPUT_SZ-1:0] act_data_x_i,
    input wire          [Y_INPUT_SZ-1:0] act_data_y_i,
    input wire              [CIN_SZ-1:0] act_data_cin_i,    // MC: current input channel
    input wire         [ACT_DATA_SZ-1:0] act_data_i,
    input wire                           act_data_last_i,
    input wire                           act_last_dumped_i, // Pulse: LAST act was
                                                            // a gated-out non-spike

    // Addressing for synaptic row memory:
    output wire                          weight_mem_rd_o,
    input  wire                          weight_mem_wait_i,
    output wire         [`ADDR_SIZE-1:0] weight_mem_addr_o,
    input  wire          [`WTD_BITS-1:0] weight_mem_data_i,

    // Weight value, plus index info for synaptic current fetch:
    output wire                          weight_index_valid_o,
    output wire      [WEIGHT_IDX_SZ-1:0] weight_index_o,
    output wire        [X_OUTPUT_SZ-1:0] weight_index_x_o,
    output wire        [Y_OUTPUT_SZ-1:0] weight_index_y_o,
    output wire             [COUT_SZ-1:0] weight_index_cout_o, // MC: cout for syn_curr addr
    output wire                          weight_index_last_o,
    input  wire                          weight_index_taken_i,
    output wire                          weight_value_valid_o,
    output wire [2**WEIGHT_SLICE_SZ-1:0] weight_value_o,
    input  wire                          weight_value_taken_i

);

localparam WEIGHT_BITS = 2**WEIGHT_SLICE_SZ;

// Signed projection width: covers act_x*step + kernel - offset across full range,
// with one extra bit of headroom plus sign bit.
localparam OUT_X_PROJ_SZ = X_INPUT_SZ + X_STEP_SZ + 2;
localparam OUT_Y_PROJ_SZ = Y_INPUT_SZ + Y_STEP_SZ + 2;

// Signals for fuly-connected weight matrix tracking:
wire                                next_out_neuron;
wire                                next_step;
wire               [`ADDR_SIZE-1:0] weight_row_base_addr;
wire                [`PIN_BITS-1:0] out_elem_count_nxt;
reg                 [`PIN_BITS-1:0] out_elem_count_r;
wire              [X_OUTPUT_SZ-1:0] out_x_index_nxt;
wire              [Y_OUTPUT_SZ-1:0] out_y_index_nxt;
reg               [X_OUTPUT_SZ-1:0] out_x_index_r;
reg               [Y_OUTPUT_SZ-1:0] out_y_index_r;
wire              [X_OUTPUT_SZ-1:0] weight_data_index_x;
wire              [Y_OUTPUT_SZ-1:0] weight_data_index_y;
//wire      [$clog2(WEIGHT_BITS)-1:0] weight_data_idx;
wire       [WEIGHT_DATA_IDX_SZ-1:0] weight_data_idx;
wire              [WEIGHT_BITS-1:0] weight_data_out;

// Conv-mode signals:
reg               [X_KERNEL_SZ-1:0] kernel_x_count_r;
reg               [Y_KERNEL_SZ-1:0] kernel_y_count_r;
reg                    [COUT_SZ-1:0] cout_count_r;          // MC: output-channel counter (outer)
wire              [X_KERNEL_SZ-1:0] kernel_x_count_nxt;
wire              [Y_KERNEL_SZ-1:0] kernel_y_count_nxt;
wire                   [COUT_SZ-1:0] cout_count_nxt;        // MC
wire                                kernel_x_at_end;
wire                                kernel_y_at_end;
wire                                cout_at_end;            // MC: cout_count == cout_len-1
wire                                next_kernel_pos;
// MC weight-index formula intermediate: cout * Cin * Ky * Kx + cin * Ky * Kx + ky * Kx + kx.
// Widened products fit WEIGHT_IDX_SZ (= 16) for con5 worst case (64*64*5 = 20480).
wire              [WEIGHT_IDX_SZ-1:0] weight_index_mc_conv;
wire                                oob_skip;
wire signed     [OUT_X_PROJ_SZ-1:0] out_x_proj_s;
wire signed     [OUT_Y_PROJ_SZ-1:0] out_y_proj_s;
wire signed     [OUT_X_PROJ_SZ-1:0] out_x_num_s;   // MC stride fix: act_x + offset - kx
wire signed     [OUT_Y_PROJ_SZ-1:0] out_y_num_s;   // MC stride fix: act_y + offset - ky
reg                           [2:0] x_step_log2;   // MC stride fix: log2(step), pow-2 strides
reg                           [2:0] y_step_log2;
wire                                x_not_multiple;// MC stride fix: (num) not a multiple of step
wire                                y_not_multiple;
wire                                out_x_oob;
wire                                out_y_oob;
wire                                is_fullConn;
wire                                is_sparseConn;
wire                                is_convolution;
wire                                weight_index_valid_full;
wire                                weight_index_valid_conv;
wire                                weight_index_valid_sparse;
wire                                weight_index_last_full;
wire                                weight_index_last_conv;
wire                                weight_index_last_sparse;
wire                                weight_value_valid_int;
wire                                sparse_last_tuple;
wire                                finished_for_this_ip_neuron_sparse;
wire           [WEIGHT_SLICE_SZ-1:0] cache_slice_sz;
wire                                finished_for_this_ip_neuron_full;
wire                                finished_for_this_ip_neuron_conv;
wire              [X_OUTPUT_SZ-1:0] weight_index_x_full;
wire              [Y_OUTPUT_SZ-1:0] weight_index_y_full;
wire              [X_OUTPUT_SZ-1:0] weight_index_x_conv;
wire              [Y_OUTPUT_SZ-1:0] weight_index_y_conv;

// Generic flow control signals:
reg                                 doing_weight_pass_r;
reg                                 weight_pass_done_r;
wire                                finished_for_this_ip_neuron;
wire                                finished_weight_pass;

////////////////////////////////////////////////////////////////
// Weight mode
//
// Support three modes: Fully, sparsely connected or convolutional
//
assign is_fullConn    = (weight_mode_i == 2'b00)? 1'b1 : 1'b0;
assign is_sparseConn  = (weight_mode_i == 2'b01)? 1'b1 : 1'b0;
assign is_convolution = (weight_mode_i == 2'b10)? 1'b1 : 1'b0;

// Calculate base address for synaptic row. In conv mode the kernel weights are
// shared across all inputs (base independent of the input index). In full/sparse
// mode the base is the input's flat index act_data_idx_i, width SP_ACT_DATA_IDX_SZ
// (sized per app; default 5 spans only 32 inputs).
// NOTE (was BROKEN, FIXED 2026-06-30 in this MC copy): for 1-weight-per-word FC the
// per-input weight base changes every input, and the shared cache fetches in a 2-cycle
// request->data cadence. Full mode used to free-run the cache REQUEST ungated, so the
// index advanced past an in-flight read and the cache served the PREVIOUS input's weight
// on the wrong request/data phase (off-by-one-low; phase flipped at each act-word
// boundary). Fix: gate weight_index_valid_full on act_data_valid_i (below), exactly like
// sparse/conv, so the only cache reads issued are for a HELD spiking input -> always a
// matched fetch. con6's 1920->1 FC readout still uses CONV mode (full-width kernel); this
// re-enables native full mode. Other SNN variants keep the latent bug (MC-only prototype).
assign weight_row_base_addr = is_convolution ? weight_base_addr_i
                                             : (act_data_idx_i * rows_per_neuron_i
                                                              + weight_base_addr_i);

// Cache slice size: in sparse mode fetch tuple-sized slices,
// in full/conv mode fetch weight-sized slices.
assign cache_slice_sz = is_sparseConn ? tuple_sz_i : weight_sz_i;

///////////////////////////////////////////////////////////
//
// Index Generation by Connectivity Mode

///////////////////////////////////////////////////////////
//
// Spike index for full-connectivity
//
// Activations (spikes) may still be sparse. So wait for the activation
// and go to fetch the weight row corresponding to that input spike.
// From that list of weights, calculate the weight address and
// the index and address of the target output neuron, to allow
// the target synaptic current to be fetched:
//
// Generate a stream of indices given by the dimensions of the output
// layer. Weights are in order.

assign next_out_neuron = weight_value_taken_i;

assign  out_x_end_of_row = ( out_x_index_r == ( out_x_len_i-1'b1))? 1'b1 : 1'b0;
assign  out_y_end_of_col = ( out_y_index_r == ( out_y_len_i-1'b1))? 1'b1 : 1'b0;

assign out_x_index_nxt = (next_out_neuron & out_x_end_of_row)? 'b0 :
                         (next_out_neuron)? (out_x_index_r + 1'b1) : out_x_index_r;

assign out_y_index_nxt = (next_out_neuron & out_y_end_of_col & out_x_end_of_row)? 'b0 :
                         (next_out_neuron & out_x_end_of_row)? out_y_index_r + 1'b1   :
                                                               out_y_index_r;

///////////////////////////////////////////////////////////
//
// Convolutional mode kernel projection
//
// For each input neuron at (act_data_x_i, act_data_y_i), iterate kernel
// position (kx, ky) in row-major order and project to output coordinates:
//   out_x = act_x * x_step + kx - x_offset
//   out_y = act_y * y_step + ky - y_offset
// Out-of-bounds projections (out_x < 0, >= out_x_len_i, etc) are skipped:
// weight_index_valid_o is suppressed but the kernel-position counter still
// advances so the kernel-weight slice pointer stays in lockstep.

assign kernel_x_at_end = (kernel_x_count_r == (x_kernel_len_i - 1'b1));
assign kernel_y_at_end = (kernel_y_count_r == (y_kernel_len_i - 1'b1));

// MC stride fix (2026-06-21): conv must DOWNSAMPLE.  Output index is
//   out = (act + offset - k) / step
// i.e. a PyTorch-equivalent strided cross-correlation.  The previous form
//   out = act*step + k - offset
// was an input-UPSAMPLING (transposed-conv) scatter: correct only at step=1,
// and kernel-flipped vs PyTorch.  FMI inter-group convs are stride-2
// downsampling (group3 120 -> group4 60 -> group5 30).
//
// Division by step is an arithmetic right shift (power-of-2 strides only;
// FMI uses 2).  A kernel tap contributes only when (act+offset-k) is an exact
// multiple of step; non-multiples are skipped just like an OOB projection, so
// the kernel-position counter and weight-slice pointer stay in lockstep.
assign out_x_num_s = $signed({1'b0, act_data_x_i})
                   + $signed({1'b0, x_kernel_offset_i})
                   - $signed({1'b0, kernel_x_count_r});
assign out_y_num_s = $signed({1'b0, act_data_y_i})
                   + $signed({1'b0, y_kernel_offset_i})
                   - $signed({1'b0, kernel_y_count_r});

// log2(step) for the divide-by-step shift (power-of-2 strides).
always @(*)
   case (x_kernel_step_i)
      'd1:     x_step_log2 = 3'd0;
      'd2:     x_step_log2 = 3'd1;
      'd4:     x_step_log2 = 3'd2;
      default: x_step_log2 = 3'd0;   // non-power-of-2 stride unsupported
   endcase
always @(*)
   case (y_kernel_step_i)
      'd1:     y_step_log2 = 3'd0;
      'd2:     y_step_log2 = 3'd1;
      'd4:     y_step_log2 = 3'd2;
      default: y_step_log2 = 3'd0;
   endcase

// Divisibility: low log2(step) bits of the numerator must be zero.
// (step-1) is the power-of-2 modulo mask.  Negative non-multiples are caught
// here; negative exact multiples fall through to the (<0) OOB test below.
assign x_not_multiple = |(out_x_num_s[X_STEP_SZ-1:0] & (x_kernel_step_i - 1'b1));
assign y_not_multiple = |(out_y_num_s[Y_STEP_SZ-1:0] & (y_kernel_step_i - 1'b1));

assign out_x_proj_s = out_x_num_s >>> x_step_log2;
assign out_y_proj_s = out_y_num_s >>> y_step_log2;

assign out_x_oob = (out_x_proj_s < 0)
                 | (out_x_proj_s >= $signed({1'b0, out_x_len_i}));
assign out_y_oob = (out_y_proj_s < 0)
                 | (out_y_proj_s >= $signed({1'b0, out_y_len_i}));

assign oob_skip  = is_convolution & doing_weight_pass_r & act_data_valid_i
                 & (out_x_oob | out_y_oob | x_not_multiple | y_not_multiple);

assign next_kernel_pos = is_convolution & doing_weight_pass_r & ~weight_pass_done_r
                       & act_data_valid_i
                       & ((weight_value_valid_o & weight_value_taken_i) | oob_skip);

// Per-kernel-position increment for kx (innermost) and ky (middle).
// MC: cout (outermost) advances when both kx and ky wrap, and itself wraps
// at cout_len_i-1. When cin_len = cout_len = 1, cout_at_end is always true
// and cout_count_r stays at 0 (collapses to single-channel conv behaviour).
assign kernel_x_count_nxt = (next_kernel_pos & kernel_x_at_end) ? 'b0 :
                            (next_kernel_pos) ? (kernel_x_count_r + 1'b1) :
                                                 kernel_x_count_r;

assign kernel_y_count_nxt = (next_kernel_pos & kernel_x_at_end & kernel_y_at_end) ? 'b0 :
                            (next_kernel_pos & kernel_x_at_end) ? (kernel_y_count_r + 1'b1) :
                                                                   kernel_y_count_r;

assign cout_at_end     = (cout_count_r == (cout_len_i - 1'b1));
assign cout_count_nxt  = (next_kernel_pos & kernel_x_at_end & kernel_y_at_end & cout_at_end)
                                                            ? {COUT_SZ{1'b0}} :
                         (next_kernel_pos & kernel_x_at_end & kernel_y_at_end)
                                                            ? (cout_count_r + 1'b1) :
                                                              cout_count_r;

assign weight_index_x_conv = out_x_proj_s[X_OUTPUT_SZ-1:0];
assign weight_index_y_conv = out_y_proj_s[Y_OUTPUT_SZ-1:0];

// MC weight-index: PyTorch-row-major (Cout, Cin, Ky, Kx) flat layout.
// cin_len_i = cout_len_i = 1 collapses to ky*Kx + kx (legacy conv addressing).
assign weight_index_mc_conv =
        cout_count_r * (cin_len_i * y_kernel_len_i * x_kernel_len_i)
      + act_data_cin_i * (y_kernel_len_i * x_kernel_len_i)
      + kernel_y_count_r * x_kernel_len_i
      + kernel_x_count_r;

///////////////////////////////////////////////////////////
//
// Mode-conditional next-step signal for the output element
// counter (drives the weight memory slice index):
//   - full mode: advance on weight_value taken (one slot per output)
//   - conv mode: advance on every kernel position (taken or skipped)

assign next_step = is_convolution ? next_kernel_pos : next_out_neuron;

// Sparse end-of-tuple-list detector (per input neuron)
assign sparse_last_tuple = is_sparseConn & doing_weight_pass_r &
                           (out_elem_count_r == (sparse_count_i - 1'b1));

// Reset the flat slice counter per input neuron in ALL connectivity modes.
// (Previously full mode reset only on finished_pass_o = end of the entire
//  pass.  That caused the weight cache to keep producing slices from the
//  same word across input-neuron boundaries when
//  weights_per_word * rows_per_neuron > out_x_len * out_y_len, leaking
//  unused slices into outputs 0..out_x_len-1 via the out_x_index wrap.)
assign out_elem_count_nxt = (finished_for_this_ip_neuron)? 1'b0 :
                            (next_step)? out_elem_count_r + 1'b1 :
                                         out_elem_count_r;

// Count through the input image, in 2-D, plus kernel position in conv mode.
// Reset out_elem_count_r per-input-neuron in conv and sparse; reset per-pass in full mode.
// MC: also reset cout_count_r per input neuron.
always @ (posedge clk)
   if (reset | ~running_i)
      begin
         out_x_index_r    <= 1'b0;
         out_y_index_r    <= 1'b0;
         out_elem_count_r <=  'b0;
         kernel_x_count_r <=  'b0;
         kernel_y_count_r <=  'b0;
         cout_count_r     <= {COUT_SZ{1'b0}};
      end
   else if (is_convolution & finished_for_this_ip_neuron)
      begin
         out_x_index_r    <= out_x_index_nxt;
         out_y_index_r    <= out_y_index_nxt;
         out_elem_count_r <= out_elem_count_nxt;
         kernel_x_count_r <=  'b0;
         kernel_y_count_r <=  'b0;
         cout_count_r     <= {COUT_SZ{1'b0}};
      end
   else
      begin
         out_x_index_r    <= out_x_index_nxt;
         out_y_index_r    <= out_y_index_nxt;
         out_elem_count_r <= out_elem_count_nxt;
         kernel_x_count_r <= kernel_x_count_nxt;
         kernel_y_count_r <= kernel_y_count_nxt;
         cout_count_r     <= cout_count_nxt;
      end

assign finished_for_this_ip_neuron_full   = is_fullConn
                                          & weight_index_last_o & weight_value_valid_o
                                          & weight_value_taken_i;
// MC: per-input-neuron is done when ALL (cout, ky, kx) have been visited.
assign finished_for_this_ip_neuron_conv   = is_convolution
                                          & kernel_x_at_end & kernel_y_at_end
                                          & cout_at_end
                                          & next_kernel_pos;
assign finished_for_this_ip_neuron_sparse = is_sparseConn
                                          & sparse_last_tuple
                                          & weight_value_valid_o & weight_value_taken_i;
assign finished_for_this_ip_neuron        = finished_for_this_ip_neuron_full
                                          | finished_for_this_ip_neuron_conv
                                          | finished_for_this_ip_neuron_sparse;

// The pass also finishes when the LAST activation was a non-spike: act gating
// drops it before this module ever sees act_data_last_i on a valid token, so
// the parent pulses act_last_dumped_i instead. The dump can only occur once
// the previous input's pass has completed (a non-spike cannot reach the act
// cache head before then), so nothing is in flight when this fires.
assign finished_weight_pass = (finished_for_this_ip_neuron & act_data_last_i)
                            | act_last_dumped_i;

always @ (posedge clk)
   if (reset | ~running_i)
      weight_pass_done_r  <= 1'b0;
   else if (finished_weight_pass)
      weight_pass_done_r <= 1'b1;
//   else if (weight_pass_done_r)
//      weight_pass_done_r  <= 1'b0;

always @ (posedge clk)
   if (reset | ~running_i)
      doing_weight_pass_r <= 1'b0;
   else if (finished_weight_pass)
      doing_weight_pass_r <= 1'b0;
   else if (~weight_pass_done_r &
	   ((is_fullConn) |                       // Full connectivity
	    (is_sparseConn  & act_data_valid_i) | // Sparse connectivity
	    (is_convolution & act_data_valid_i))) // Convolutional connectivity
      doing_weight_pass_r <= 1'b1;

//   else if (weight_pass_done_r)
//      doing_weight_pass_r <= 1'b0;

assign running_weight_pass_o = doing_weight_pass_r;

// act_data_valid_i gate (2026-06-30): only request a weight while a real spiking input
// is held at the head. Without it, full mode free-ran the cache on non-spiking inputs and
// the per-input base desynced from the 2-cycle cache fetch -> off-by-one weight. See the
// weight_row_base_addr note above and tb T13.
assign weight_index_valid_full   = is_fullConn   & running_i & doing_weight_pass_r
                                 & ~weight_pass_done_r & act_data_valid_i;
assign weight_index_valid_conv   = is_convolution & running_i & doing_weight_pass_r
                                 & ~weight_pass_done_r & act_data_valid_i & ~oob_skip;
assign weight_index_valid_sparse = is_sparseConn & running_i & doing_weight_pass_r
                                 & ~weight_pass_done_r & act_data_valid_i;

assign weight_index_x_full     = out_x_index_r;
assign weight_index_y_full     = out_y_index_r;

assign weight_index_last_full   = running_i & out_x_end_of_row & out_y_end_of_col;
// MC: 'last' for conv only when all (cout, ky, kx) are at end.
assign weight_index_last_conv   = running_i & doing_weight_pass_r & act_data_valid_i
                                & kernel_x_at_end & kernel_y_at_end & cout_at_end;
assign weight_index_last_sparse = running_i & doing_weight_pass_r & act_data_valid_i
                                & sparse_last_tuple;

assign weight_index_valid_o    = is_convolution ? weight_index_valid_conv :
                                 is_sparseConn  ? weight_index_valid_sparse :
                                                  weight_index_valid_full;
// MC: conv mode uses the explicit (cout, cin, ky, kx) formula. Full/sparse keep
// the flat per-pass counter as before.
assign weight_index_o          = is_convolution ? weight_index_mc_conv :
                                                  out_elem_count_r;
assign weight_index_cout_o     = cout_count_r;
assign weight_index_x_o        = is_convolution ? weight_index_x_conv :
                                 is_sparseConn  ? {X_OUTPUT_SZ{1'b0}} :
                                                  weight_index_x_full;
assign weight_index_y_o        = is_convolution ? weight_index_y_conv :
                                 is_sparseConn  ? {Y_OUTPUT_SZ{1'b0}} :
                                                  weight_index_y_full;
assign weight_index_last_o     = is_convolution ? weight_index_last_conv :
                                 is_sparseConn  ? weight_index_last_sparse :
                                                  weight_index_last_full;

assign finished_pass_o       = finished_weight_pass;
assign finished_one_pass_o   = finished_for_this_ip_neuron;

`ifdef CONV_DEBUG
always @ (posedge clk)
   if (is_convolution & doing_weight_pass_r & act_data_valid_i)
      $display("[%0d] conv: act=(%0d,%0d) k=(%0d,%0d) proj=(%0d,%0d) outlen=(%0d,%0d) ox_oob=%b oy_oob=%b skip=%b validI=%b takenI=%b done=%b",
               $time, act_data_x_i, act_data_y_i,
               kernel_x_count_r, kernel_y_count_r,
               out_x_proj_s, out_y_proj_s,
               out_x_len_i, out_y_len_i,
               out_x_oob, out_y_oob, oob_skip,
               weight_value_valid_int, weight_value_taken_i,
               finished_for_this_ip_neuron);
`endif

// Suppress weight_value_valid_o externally for OOB conv kernel positions.
// (The cache won't be requested in OOB cycles; this guards against any
// residual valid from a held-but-not-yet-taken previous fetch.)
// Also gate on act_data_valid_i: the weight value must not be consumed while
// the activation operand is invalid. When the activation fetch is stalled by
// the memory arbiter, the act cache transiently presents stale data
// (old_word[new_lane]); without this gate the MAC accumulated that stale
// activation for the first output(s) of an input neuron.
assign weight_value_valid_o = weight_value_valid_int & ~oob_skip & act_data_valid_i;

///////////////////////////////////////////////////////////////
// Fetch weight, either from local cache or from external memory


dataline_cache_with_xy #(
    .IN_DATA_BITS(IN_DATA_BITS),
    .X_INPUT_SZ(X_OUTPUT_SZ),
    .Y_INPUT_SZ(Y_OUTPUT_SZ),
    .IDX_ADDR_BITS(WEIGHT_IDX_SZ),
    .SLICE_DATA_IDX_SZ(WEIGHT_DATA_IDX_SZ),
    .SLICE_SIZE_SZ(WEIGHT_SLICE_SZ),
    .OUT_DATA_BITS(WEIGHT_BITS))

    weight_cache (
    .clk(clk),
    .reset(reset),

    .colour_select_o(),
    .invalidate_i(start_new_block_i), // task dispatch — cache quiescent
    .slice_sz_i(cache_slice_sz),
    .base_addr_i(weight_row_base_addr),

    .sys_addr_i(weight_index_o),
    .sys_req_i(weight_index_valid_o),
    .sys_index_x_i(weight_index_x_o),
    .sys_index_y_i(weight_index_y_o),
    .sys_colour_i(1'b0),
    .sys_wait_o(weight_wait),
    .sys_last_i(weight_index_last_o),

    .slice_data_valid_o(weight_value_valid_int),
    .slice_data_idx_o(weight_data_idx),
    .slice_data_index_x_o(weight_data_index_x),
    .slice_data_index_y_o(weight_data_index_y),
    .slice_data_o(weight_value_o),
    .slice_data_last_o(weight_data_last),
    .slice_data_taken_i(weight_value_taken_i),

    .mem_addr_o(weight_mem_addr_o),
    .mem_data_i(weight_mem_data_i),
    .mem_req_o(weight_mem_rd_o),
    .mem_wait_i(weight_mem_wait_i)
);

endmodule
