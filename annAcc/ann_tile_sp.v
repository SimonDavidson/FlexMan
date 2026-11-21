// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
/////////////////////////////////////////////////////////////////////
//
// ann_tile_sp -- output-stationary NLANE-wide dense MAC engine ().
//
// Authors      : Simon Davidson & Claude
// Created      : 2026-11-21
// Last modified: 2026-11-21
//
// Replaces ann_spike_processing for dense ("full", weight_mode 00) layers with
// 8- or 16-bit weights. Same memory-port contract as the legacy path (a request
// is accepted when its *_wait_i is low; read data returns the NEXT cycle), so
// it drops in behind the same arbiter / shared pool.
//
// Dataflow (legacy full mode is input-stationary: one syn_curr read-modify-
// write per MAC, 2 port accesses/MAC, hard floor 2 cycles/MAC). Here, for each
// tile of NLANE consecutive outputs [j0, j0+NLANE):
//   for every input i (all of them, including zero activations):
//       read ONE 32-bit weight word  W[i][j0..]  (addr = wbase + i*rows + j0/EPW)
//       read the activation word when i crosses a word boundary
//       NLANE multiplies by the same activation, added to NLANE register accs
//   then ONE read-modify-write per output into syn_curr (bit-exact: signed
//   32-bit wrap-around addition is associative and commutative).
// The accumulator bank is double-buffered ("pending"), so the 2*NLANE-cycle
// syn_curr drain of tile t overlaps the compute of tile t+1.
//
// The weight word is read straight from the weight port (pipelined, 1/cycle),
// NOT through dataline_cache_with_xy, which can deliver at most 0.5 word/cycle
// (measured in tb_wfetch_rate: 2.0 cycles/word even for ideal contiguous reads).
//
// Eligibility (checked by ann_spike_processing wrapper): weight_mode==full,
// weight_sz in {8b,16b} with NLANE <= weights/word, act width <= MAC_ACT_BITS.
// Zero activations are NOT skipped (legacy skips them, which saves cycles on
// sparse activations; the result is identical because 0*w = 0).
/////////////////////////////////////////////////////////////////////
`include "../shared/constants.v"

module ann_tile_sp #(
    parameter NLANE         = 4,            // power of two, 2 or 4
    parameter MAC_ACT_BITS  = 32,           // max supported activation width
    parameter X_INPUT_SZ    = 8,
    parameter Y_INPUT_SZ    = 8,
    parameter X_OUTPUT_SZ   = 8,
    parameter Y_OUTPUT_SZ   = 8,
    parameter ROWS_PER_NEURON = 16,
    parameter WEIGHT_SLICE_SZ = 5,
    parameter MEM_ADDR_BITS = `ADDR_SIZE
)(
    input  wire                      clk,
    input  wire                      reset,
    input  wire                      start_i,           // task dispatch pulse
    // config (static during a task)
    input  wire [MEM_ADDR_BITS-1:0]  act_base_addr_i,
    input  wire [MEM_ADDR_BITS-1:0]  weight_base_addr_i,
    input  wire [MEM_ADDR_BITS-1:0]  syn_curr_base_addr_i,
    input  wire [WEIGHT_SLICE_SZ-1:0] weight_sz_i,      // 3 = 8-bit, 4 = 16-bit
    input  wire                [2:0] act_slice_sz_i,    // 0..5 = 1..32-bit elements
    input  wire                      act_signed_i,
    input  wire [X_INPUT_SZ-1:0]     in_x_len_i,
    input  wire [Y_INPUT_SZ-1:0]     in_y_len_i,
    input  wire [X_OUTPUT_SZ-1:0]    out_x_len_i,
    input  wire [Y_OUTPUT_SZ-1:0]    out_y_len_i,
    input  wire [ROWS_PER_NEURON-1:0] rows_per_neuron_i,
    // status
    output wire                      busy_o,
    output wire                      finished_o,        // 1 cycle, like legacy
    // weight port (read only)
    output wire                      weight_mem_rd_o,
    input  wire                      weight_mem_wait_i,
    output wire [`ADDR_SIZE-1:0]     weight_mem_addr_o,
    input  wire [`WTD_BITS-1:0]      weight_mem_data_i,
    // activation port (read only)
    output wire                      act_mem_req_o,
    input  wire                      act_mem_wait_i,
    output wire [`ADDR_SIZE-1:0]     act_mem_addr_o,
    input  wire [`ACT_BITS-1:0]      act_mem_data_i,
    // syn_curr port (read-modify-write, one request at a time)
    output wire                      syn_curr_mem_wr_o,
    output wire                      syn_curr_mem_rd_o,
    input  wire                      syn_curr_mem_wait_i,
    output wire [`ADDR_SIZE-1:0]     syn_curr_mem_addr_o,
    output wire [`POT_BITS-1:0]      syn_curr_mem_data_wr_o,
    input  wire [`POT_BITS-1:0]      syn_curr_mem_data_rd_i
);

localparam NIN_W   = X_INPUT_SZ  + Y_INPUT_SZ;
localparam NOUT_W  = X_OUTPUT_SZ + Y_OUTPUT_SZ;
localparam LW      = (NLANE <= 2) ? 1 : 2;       // log2(NLANE) for 2,4

// ---------------------------------------------------------------- run control
reg  running_r;
reg  [1:0] state_r;
localparam S_IDLE = 2'd0, S_RUN = 2'd1, S_FLUSH = 2'd2;

reg  [NIN_W-1:0]  nin_r;
reg  [NOUT_W-1:0] nout_r;
wire [NIN_W-1:0]  nin_w  = in_x_len_i  * in_y_len_i;
wire [NOUT_W-1:0] nout_w = out_x_len_i * out_y_len_i;

// weight geometry
wire [4:0] wsz   = weight_sz_i[4:0];
wire [4:0] lg_epw = 5'd5 - wsz;                  // log2(elements per 32-bit word): 2 (8b) or 1 (16b)
wire [5:0] wbits = 6'd1 << wsz;                  // 8 or 16
wire [4:0] lg_apw = 5'd5 - {2'b0, act_slice_sz_i};  // log2(activations per word)
wire [5:0] abits  = 6'd1 << act_slice_sz_i;

// issue-side counters
reg  [NIN_W-1:0]        i_r;                     // input index within the tile
reg  [NOUT_W-1:0]       j0_r;                    // first output of the tile
reg  [`ADDR_SIZE-1:0]   wrow_r;                  // i * rows_per_neuron (incremental)
wire [`ADDR_SIZE-1:0]   woff   = j0_r >> lg_epw;
wire [3:0]              sub    = j0_r & ((4'd1 << lg_epw) - 4'd1);   // lane0 element within word
wire is_last_in   = (i_r == nin_r - 1'b1);
wire is_last_tile = ((j0_r + NLANE) >= nout_r);
wire [NOUT_W-1:0] remain = nout_r - j0_r;
wire [3:0] nvalid = (remain >= NLANE) ? NLANE[3:0] : remain[3:0];
wire need_act     = ((i_r & ((1 << lg_apw) - 1)) == 0);

// pipeline status / hazard
reg  s1_v, s1_new_act, s1_first, s1_last;
reg  [4:0] s1_ielem;   reg [3:0] s1_sub, s1_nv;   reg [NOUT_W-1:0] s1_j0;
reg  s2_v, s2_first, s2_last;
reg  [4:0] s2_ielem;   reg [3:0] s2_sub, s2_nv;   reg [NOUT_W-1:0] s2_j0;
reg  [1:0] dstate_r;
localparam D_IDLE = 2'd0, D_RD = 2'd1, D_CAPT = 2'd2, D_WR = 2'd3;
wire pend_busy = (dstate_r != D_IDLE);

wire hold_last = is_last_in & (pend_busy | (s1_v & s1_last) | (s2_v & s2_last));
wire issuing   = (state_r == S_RUN) & ~hold_last;

assign weight_mem_rd_o   = issuing;
assign weight_mem_addr_o = weight_base_addr_i + wrow_r + woff;
assign act_mem_req_o     = issuing & need_act;
assign act_mem_addr_o    = act_base_addr_i + (i_r >> lg_apw);

wire fire = issuing & ~weight_mem_wait_i & (~need_act | ~act_mem_wait_i);

always @(posedge clk) begin
    if (reset) begin
        running_r <= 1'b0; state_r <= S_IDLE;
        i_r <= 0; j0_r <= 0; wrow_r <= 0; nin_r <= 0; nout_r <= 0;
        s1_v <= 1'b0;
    end else begin
        s1_v <= fire;
        if (start_i) begin
            running_r <= 1'b1; state_r <= S_RUN;
            nin_r <= nin_w; nout_r <= nout_w;
            i_r <= 0; j0_r <= 0; wrow_r <= 0;
        end else if (finished_o) begin
            running_r <= 1'b0; state_r <= S_IDLE;
        end else if (fire) begin
            if (is_last_in) begin
                i_r <= 0; wrow_r <= 0;
                if (is_last_tile) state_r <= S_FLUSH;
                else              j0_r <= j0_r + NLANE;
            end else begin
                i_r <= i_r + 1'b1; wrow_r <= wrow_r + rows_per_neuron_i;
            end
        end
    end
end

// Stage-1 tags travel with the issued request (data returns next cycle).
always @(posedge clk) begin
    s1_new_act <= need_act;
    s1_first   <= (i_r == 0);
    s1_last    <= is_last_in;
    s1_ielem   <= i_r & ((1 << lg_apw) - 1);
    s1_sub     <= sub;
    s1_nv      <= nvalid;
    s1_j0      <= j0_r;
end

// -------------------------------------------------- stage 1: capture the data
reg [31:0] w_word_r, act_word_r;
always @(posedge clk) begin
    if (reset) s2_v <= 1'b0;
    else       s2_v <= s1_v;
    if (s1_v) begin
        w_word_r <= weight_mem_data_i;
        if (s1_new_act) act_word_r <= act_mem_data_i;
        s2_ielem <= s1_ielem; s2_first <= s1_first; s2_last <= s1_last;
        s2_sub <= s1_sub; s2_nv <= s1_nv; s2_j0 <= s1_j0;
    end
end

// ------------------------------------------- stage 2: NLANE MACs + accumulate
// activation operand
wire [4:0]  ashift = s2_ielem * abits[4:0];
wire [31:0] araw   = (act_word_r >> ashift);
wire [31:0] amask  = (abits == 6'd32) ? 32'hFFFFFFFF : ((32'd1 << abits) - 1);
wire [31:0] aelem  = araw & amask;
wire        asign  = act_signed_i & aelem[abits-1];
wire [31:0] aext   = asign ? (aelem | ~amask) : aelem;
wire signed [MAC_ACT_BITS:0] act_op = {asign, aext[MAC_ACT_BITS-1:0]};

reg [31:0] acc_r  [0:NLANE-1];
reg [31:0] pend_r [0:NLANE-1];
wire [31:0] acc_nxt [0:NLANE-1];
reg [NOUT_W-1:0] pend_j0_r;  reg [3:0] pend_nv_r;

genvar k;
generate for (k = 0; k < NLANE; k = k + 1) begin : g_lane
    wire [4:0]  widx  = s2_sub + k;
    wire [4:0]  wsh   = widx * wbits[4:0];
    wire [31:0] wr    = w_word_r >> wsh;
    wire [15:0] wraw  = (wbits == 6'd8) ? {8'h00, wr[7:0]} : wr[15:0];
    wire signed [15:0] wext = (wbits == 6'd8) ? {{8{wr[7]}}, wr[7:0]} : wr[15:0];
    ann_lane_mac #(.WBITS_MAX(16), .ACT_BITS(MAC_ACT_BITS)) u_mac (
        .w_i(wext), .a_i(act_op), .acc_i(s2_first ? 32'd0 : acc_r[k]), .acc_o(acc_nxt[k]));
    always @(posedge clk) begin
        if (s2_v & ~s2_last)         acc_r[k]  <= acc_nxt[k];
        if (s2_v &  s2_last)         pend_r[k] <= acc_nxt[k];
    end
end endgenerate

wire load_pend = s2_v & s2_last;
always @(posedge clk) if (load_pend) begin pend_j0_r <= s2_j0; pend_nv_r <= s2_nv; end

// --------------------------------------------- drain: pend -> syn_curr (RMW)
reg [3:0]  r_r;                       // lane being read / written
reg [31:0] rdat_r [0:NLANE-1];
reg        cap_v;  reg [3:0] cap_lane;
wire rd_accept = (dstate_r == D_RD) & ~syn_curr_mem_wait_i;
wire wr_accept = (dstate_r == D_WR) & ~syn_curr_mem_wait_i;

always @(posedge clk) begin
    if (reset) begin dstate_r <= D_IDLE; r_r <= 0; cap_v <= 1'b0; cap_lane <= 0; end
    else begin
        cap_v    <= rd_accept;
        cap_lane <= r_r;
        if (cap_v) rdat_r[cap_lane] <= syn_curr_mem_data_rd_i;
        case (dstate_r)
            D_IDLE: if (load_pend) begin dstate_r <= D_RD; r_r <= 0; end
            D_RD:   if (rd_accept) begin
                        if (r_r == pend_nv_r - 1'b1) dstate_r <= D_CAPT;
                        else r_r <= r_r + 1'b1;
                    end
            D_CAPT: begin dstate_r <= D_WR; r_r <= 0; end
            D_WR:   if (wr_accept) begin
                        if (r_r == pend_nv_r - 1'b1) dstate_r <= D_IDLE;
                        else r_r <= r_r + 1'b1;
                    end
        endcase
    end
end

assign syn_curr_mem_rd_o      = (dstate_r == D_RD);
assign syn_curr_mem_wr_o      = (dstate_r == D_WR);
assign syn_curr_mem_addr_o    = syn_curr_base_addr_i + pend_j0_r + r_r;
assign syn_curr_mem_data_wr_o = rdat_r[r_r] + pend_r[r_r];

// ------------------------------------------------------------------ finished
wire flushed = (state_r == S_FLUSH) & ~s1_v & ~s2_v & (dstate_r == D_IDLE);
assign finished_o = running_r & flushed;
assign busy_o     = running_r;

endmodule
