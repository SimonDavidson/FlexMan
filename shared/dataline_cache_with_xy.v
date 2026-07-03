// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
/*
 * dataline_read_only_cache.v
 *
 * @author: Samuel López
 *          Simon D.
 */


`include "../shared/constants.v"

module dataline_cache_with_xy #(
    parameter IN_DATA_BITS      = 32,
    parameter X_INPUT_SZ        = 8,
    parameter Y_INPUT_SZ        = 8,
    parameter IDX_ADDR_BITS     = 10,
    parameter SLICE_DATA_IDX_SZ = 5,
    parameter SLICE_SIZE_SZ     = 3,
    parameter OUT_DATA_BITS     = 32,
    // Read-return handling (FPGA-Fmax lever; 0 = original behaviour):
    //   REG_RETURN=0  combinational forward — the memory return is served
    //                 through slice_and_align in its arrival cycle (default,
    //                 bit-identical to the original cache).
    //   REG_RETURN=1  registered serve — the return lands in the line register
    //                 first and is served the following cycle. Breaks the
    //                 mem-DOUT -> consumer -> next-request critical path;
    //                 costs +1 cycle per word fetch.
    //   REG_RETURN=2  registered serve + NEXT-LINE PREFETCH — two line
    //                 registers; while word w is being served the cache
    //                 fetches word w+1 (same base) into the other line, so a
    //                 sequential stream sees hits instead of refill stalls.
    //                 Same timing cut as mode 1 without the per-word cycle
    //                 cost. One wasted overrun fetch per stream/row end
    //                 (discarded; reads are side-effect free). Only useful
    //                 for sequential consumers — keep random-access users
    //                 (e.g. the LUT cache) at mode 1.
    parameter REG_RETURN        = 0)
(
    input wire                          clk,
    input wire                          reset,

    /* Control params */
    output wire                         colour_select_o,
    // Drop the cached line (next read refetches). Asserted on task dispatch,
    // when the memory contents under a cached address may have changed (e.g.
    // z_rec rewritten by NP between two SP reads of the same word). Must only
    // assert while the cache is quiescent (no request / slice in flight).
    input  wire                         invalidate_i,
    input  wire     [SLICE_SIZE_SZ-1:0] slice_sz_i,
    input  wire        [`ADDR_SIZE-1:0] base_addr_i,

    /* System side - in */
    input wire      [IDX_ADDR_BITS-1:0] sys_addr_i,
    input wire                          sys_req_i,
    input wire         [X_INPUT_SZ-1:0] sys_index_x_i,
    input wire         [Y_INPUT_SZ-1:0] sys_index_y_i,

    input  wire                         sys_colour_i,
    output wire                         sys_wait_o,
    input  wire                         sys_last_i,

    /* System side - out */
    output wire                         slice_data_valid_o,
    output wire [SLICE_DATA_IDX_SZ-1:0] slice_data_idx_o,
    output wire        [X_INPUT_SZ-1:0] slice_data_index_x_o,
    output wire        [Y_INPUT_SZ-1:0] slice_data_index_y_o,
    output wire     [OUT_DATA_BITS-1:0] slice_data_o,
    output wire                         slice_data_last_o,
    input  wire                         slice_data_taken_i,

    /* Memory side */
    output wire      [`ADDR_SIZE-1:0]   mem_addr_o,
    input  wire      [IN_DATA_BITS-1:0] mem_data_i,
    output wire                         mem_req_o,
    input  wire                         mem_wait_i
);

localparam ACTW  = 32;     // Activation bit width
                           // TODO Implement as input to choose
                           // between activation bit widths

localparam LACTW = SLICE_SIZE_SZ; //$clog2(ACTW); // Log2 of data element bitwidth
localparam LACTB = $clog2(IN_DATA_BITS);   // Log2 of memory bit width

localparam cache = 1'b0, memory = 1'b1; // FSM states

// Signals shared by both engines (single-line / prefetch):
wire [IN_DATA_BITS-1:0] slice_in;
wire                    word_last_slice;
reg [IDX_ADDR_BITS-1:0] shifted_index;
reg        [LACTB-1:0]  slice_idx;

////////////////////////////////////////////
// Generate address slices, for cache address and slice_and_align block
//

// Drop the bottom (LACTB - slice_sz_i) bits — those are the in-word slice
// offset; what remains is the word-level memory index. Done as a barrel
// shift so the bounds stay legal for any IDX_ADDR_BITS (the per-arm
// part-selects in the previous case statement went negative when
// IDX_ADDR_BITS < 6, which Vivado rejects even on unreachable arms).
always @* begin
    shifted_index = sys_addr_i  >> (LACTB - slice_sz_i);
end

// In-word slice offset = the low (LACTB - slice_sz_i) bits of sys_addr_i.
// Mask form avoids fixed-width part-selects that would over-read sys_addr_i
// when IDX_ADDR_BITS < LACTB (e.g. the bias / pot caches with IDX_ADDR_BITS=2).
always @* begin
   slice_idx = sys_addr_i & ({LACTB{1'b1}} >> slice_sz_i);
end

generate if (REG_RETURN != 2) begin : g_line

////////////////////////////////////////////////////////////////////////////
// Single-line engine (REG_RETURN = 0 combinational-forward / 1 registered)
////////////////////////////////////////////////////////////////////////////

/* Auxiliary signals */
reg            [IDX_ADDR_BITS-1:0] reg_addr;
wire                               cache_hit;


/* Slice-and-align signals */
wire                     cache_valid_nxt;
reg                      cache_valid_r;
reg  [IDX_ADDR_BITS-1:0] cache_addr_r;
wire                     mem_data_next_cycle;
reg                      mem_new_data_valid_r;
wire                     stall_downstream;
//reg   [IN_DATA_BITS-1:0] cache_data_nxt;
reg   [IN_DATA_BITS-1:0] cache_data_r;

always @ (posedge clk)
   if (reset)
   begin
      cache_valid_r    <= 1'b0;
      mem_new_data_valid_r   <= 1'b0;
   end
   else
   begin
      cache_valid_r    <= cache_valid_nxt;
      mem_new_data_valid_r <= mem_data_next_cycle;
   end

// Track base_addr at cache fill time so we can invalidate when it changes.
reg [`ADDR_SIZE-1:0] cache_base_addr_r;

always @ (posedge clk)
   if (reset)
   begin
      cache_addr_r       <= 'b10101010;
      cache_data_r       <= 'b0;
      cache_base_addr_r  <= 'b0;
   end
   else
   begin
      if (mem_data_next_cycle)
         begin
            cache_addr_r    <= sys_addr_i;
         end
      if (mem_new_data_valid_r)
      begin
         cache_data_r       <= mem_data_i;
         cache_base_addr_r  <= base_addr_i;
      end
   end

assign stall_downstream = slice_data_valid_o & ~slice_data_taken_i;

// Do we need to send a requst to memory for the required data?
// Yes, if there is a request, it's not in the cache and we aren't
// stalling downstream, so that we can continue:
assign mem_req_o = sys_req_i & ~cache_hit & ~mem_new_data_valid_r;

// Remember that we sent a request and that it was accepted:
assign mem_data_next_cycle = mem_req_o & ~mem_wait_i;

// Cache_valid either if: (i) no request and its already valid OR
// (ii) there's a request, it's in cache
// and the memory is responding with data on the clock edge:
// TODO add back pressure from later stages:
//
assign cache_valid_nxt = (invalidate_i) ? 1'b0 :   // task dispatch: line is stale
	                 (mem_new_data_valid_r) ? 1'b1 :
	                 (cache_valid_r & (~slice_data_taken_i | ~word_last_slice)) ? 1'b1 :
			 (slice_data_taken_i & ~sys_req_i) ? 1'b0 :
			 cache_valid_r;


//assign cache_data_nxt  = (mem_new_data_valid_r & ~cache_valid_r)? mem_data_i : cache_data_r;

////////////////////////////////////////////
// Generate output control signals
//

// Back pressure on the index generator if we can't move the current
// request forward this cycle:
//assign sys_wait_o = (sys_req_i & ~(cache_hit | (mem_req_o & ~mem_wait_i))) |
//                    (req_valid_r & ~slice_data_taken_i);

assign sys_wait_o = (sys_req_i & ~cache_hit &  mem_new_data_valid_r) |
                    (sys_req_i &  cache_hit & ~slice_data_taken_i);

always @* begin
    reg_addr      = cache_addr_r >> (LACTB - slice_sz_i);
end

// Cache hit - matching slice address, matching base address, and valid data.
// (base_addr_i changes when the upstream consumer moves to a new memory row,
//  e.g. a new input neuron in sparse / full-connectivity weight rows.)
assign cache_hit = (shifted_index == reg_addr)
                 && (base_addr_i  == cache_base_addr_r)
                 && cache_valid_r;

// Take the data directly from the memory bus if a request was accepted last
// cycle. If not, take the cached data. Under REG_RETURN the memory return is
// never forwarded combinationally: it lands in cache_data_r at the end of its
// arrival cycle and the slice is served from the register a cycle later (the
// arrival cycle produces no slice_data_valid_o, see below).
if (REG_RETURN == 1) begin : g_slice_in_reg
    assign slice_in = cache_data_r;
end else begin : g_slice_in_fwd
    assign slice_in = (mem_new_data_valid_r)? mem_data_i :
	                            cache_data_r;
end

assign mem_addr_o           = shifted_index + base_addr_i;

// REG_RETURN: only a registered-line hit serves data (the return cycle itself
// loads the register; the slice is served the following cycle).
//
// The ~mem_new_data_valid_r term is LOAD-BEARING: on a fetch-accept the cache
// updates cache_addr_r that cycle but cache_valid_r stays set from the
// PREVIOUS word, so during the return cycle cache_hit is spuriously true with
// cache_data_r still holding the old word. The combinational-forward path
// masks that window by muxing mem_data_i over the stale line; with the
// forward removed the window must be suppressed instead, or every consumer is
// served the previous word's data one element early (surfaced as a clean
// one-element shift in every NP output stream, 2026-07-02).
if (REG_RETURN == 1) begin : g_valid_reg
    assign slice_data_valid_o = sys_req_i & cache_hit & ~mem_new_data_valid_r;
end else begin : g_valid_fwd
    assign slice_data_valid_o = sys_req_i & (cache_hit | mem_new_data_valid_r);
end

end else begin : g_prefetch

////////////////////////////////////////////////////////////////////////////
// Prefetch engine (REG_RETURN = 2): two line registers, registered serve.
//
// While word w is being served from one line, word w+1 (same base) is
// fetched into the other, so a sequential stream sees back-to-back hits.
// A line becomes hit-eligible only when its data has LANDED (per-line valid
// set at the load edge, tags recorded at issue) — the mode-1 stale-hit
// window cannot occur by construction. Single fetch outstanding (the
// memory/pool contract is 1-cycle read, data valid for exactly the return
// cycle). Demand misses (row/base change) behave like mode 1: fetch, land,
// serve — once per stream start.
//
// Prefetch safety: reads are side-effect free; an overrun fetch past the
// stream end is discarded (tag never matches a real request). In-task
// writers of a prefetched buffer always TRAIL the reader element index
// (in-place Hadamard h / R_prev updates), and cross-task reuse is covered
// by invalidate_i at dispatch, which clears both lines and squashes any
// in-flight landing.
////////////////////////////////////////////////////////////////////////////

reg [IN_DATA_BITS-1:0]  pl_data0, pl_data1;
reg [IDX_ADDR_BITS-1:0] pl_word0, pl_word1;
reg [`ADDR_SIZE-1:0]    pl_base0, pl_base1;
reg [1:0]               pl_valid;
reg [1:0]               pl_pend;      // fetch issued for this line, not landed
reg                     pl_fill_r;    // line the in-flight fetch targets
reg                     pl_ret_r;     // memory data lands THIS cycle

wire [IDX_ADDR_BITS-1:0] cur_word  = shifted_index;
wire [IDX_ADDR_BITS-1:0] next_word = shifted_index + 1'b1;

wire hit0 = pl_valid[0] & (pl_word0 == cur_word) & (pl_base0 == base_addr_i);
wire hit1 = pl_valid[1] & (pl_word1 == cur_word) & (pl_base1 == base_addr_i);
wire any_hit = hit0 | hit1;

assign slice_in           = hit1 ? pl_data1 : pl_data0;
assign slice_data_valid_o = sys_req_i & any_hit;

// Is the NEXT word already held or being fetched (either line)?
wire next_in0 = (pl_valid[0] | pl_pend[0]) & (pl_word0 == next_word)
              & (pl_base0 == base_addr_i);
wire next_in1 = (pl_valid[1] | pl_pend[1]) & (pl_word1 == next_word)
              & (pl_base1 == base_addr_i);
wire next_held = next_in0 | next_in1;

// Is the CURRENT word already in flight (its prefetch landing)?  Without this
// a demand re-fetch of an in-flight word would duplicate it across both lines.
wire cur_pend = (pl_pend[0] & (pl_word0 == cur_word) & (pl_base0 == base_addr_i))
              | (pl_pend[1] & (pl_word1 == cur_word) & (pl_base1 == base_addr_i));

wire issue_demand = sys_req_i & ~any_hit & ~cur_pend;
wire want_pref    = sys_req_i &  any_hit & ~next_held;

// One outstanding fetch; nothing issues in the return cycle (the return bus
// carries this cycle's landing data). Demand has priority over prefetch.
assign mem_req_o  = (issue_demand | want_pref) & ~pl_ret_r;

// The accepted address must be HELD through the return cycle: the memories
// behind this interface may read combinationally (data = mem[addr] presented
// the cycle after the accept), which the single-line engine satisfied by
// construction because its requester holds sys_addr until served. With
// prefetch the natural next address appears one cycle early, so the landing
// capture would read the WRONG word (demand for w landed w+1's data — FC1
// word-0 corruption, 2026-07-03). Register the address at accept and present
// it while the data lands.
wire [`ADDR_SIZE-1:0] mem_addr_now = (issue_demand ? cur_word : next_word)
                                   + base_addr_i;
reg  [`ADDR_SIZE-1:0] pl_maddr_r;
assign mem_addr_o = pl_ret_r ? pl_maddr_r : mem_addr_now;

wire accept = mem_req_o & ~mem_wait_i;

// Fill target: prefetch fills the non-serving line; a demand miss fills
// whichever line does NOT hold the (possibly useful) next word.
wire fill_line = issue_demand ? (next_in0 ? 1'b1 : 1'b0) : ~hit1;

always @ (posedge clk)
   if (reset)
   begin
      pl_valid  <= 2'b00;
      pl_pend   <= 2'b00;
      pl_ret_r  <= 1'b0;
      pl_fill_r <= 1'b0;
      pl_word0  <= {IDX_ADDR_BITS{1'b0}};
      pl_word1  <= {IDX_ADDR_BITS{1'b0}};
      pl_base0  <= {`ADDR_SIZE{1'b0}};
      pl_base1  <= {`ADDR_SIZE{1'b0}};
      pl_data0  <= {IN_DATA_BITS{1'b0}};
      pl_data1  <= {IN_DATA_BITS{1'b0}};
   end
   else
   begin
      // invalidate squashes an accept landing next cycle as well
      pl_ret_r <= accept & ~invalidate_i;
      if (accept)
      begin
         pl_fill_r  <= fill_line;
         pl_maddr_r <= mem_addr_now;
         if (fill_line) begin
            pl_word1 <= issue_demand ? cur_word : next_word;
            pl_base1 <= base_addr_i;
         end else begin
            pl_word0 <= issue_demand ? cur_word : next_word;
            pl_base0 <= base_addr_i;
         end
      end
      if (invalidate_i)
      begin
         // task dispatch: both lines stale; drop any in-flight fetch
         pl_valid <= 2'b00;
         pl_pend  <= 2'b00;
      end
      else
      begin
         if (accept)
         begin
            // The line's tag now describes the IN-FLIGHT word: it must not be
            // hit-eligible until the data lands, or the old data serves for
            // one cycle under the new tag (the mode-1 stale-hit window again,
            // resurrected through the valid bit — surfaced as sparse weight
            // corruption at row changes, 2026-07-03).
            pl_valid[fill_line] <= 1'b0;
            pl_pend [fill_line] <= 1'b1;
         end
         if (pl_ret_r)
         begin
            if (pl_fill_r) pl_data1 <= mem_data_i;
            else           pl_data0 <= mem_data_i;
            pl_valid[pl_fill_r] <= 1'b1;
            pl_pend [pl_fill_r] <= 1'b0;
         end
      end
   end

// Same contract as mode 1: hold the requester while its demand is landing,
// and while a served slice has not been taken.
assign sys_wait_o = (sys_req_i & ~any_hit & pl_ret_r) |
                    (sys_req_i &  any_hit & ~slice_data_taken_i);

`ifdef PF_DEBUG
integer dbg_n = 0;
always @ (posedge clk)
   if (dbg_n < 100000 && (accept | pl_ret_r | (slice_data_valid_o & slice_data_taken_i))) begin
      dbg_n = dbg_n + 1;
      if (accept)
         $display("[PF %0t] FETCH  maddr=%0h (w=%0d base=%0h dem=%b fill=%b)",
                  $time, mem_addr_o, issue_demand ? cur_word : next_word,
                  base_addr_i, issue_demand, fill_line);
      if (pl_ret_r)
         $display("[PF %0t] LAND   line=%b data=%h", $time, pl_fill_r, mem_data_i);
      if (slice_data_valid_o & slice_data_taken_i)
         $display("[PF %0t] TAKE   addr=%0d w=%0d base=%0h line=%b slice=%h",
                  $time, sys_addr_i, cur_word, base_addr_i, hit1, slice_data_o);
   end
`endif

end endgenerate

assign colour_select_o = sys_colour_i;

/* Output multiplexer (slice-and-align) */
slice_and_align #(
    .IN_DATA_BITS(IN_DATA_BITS),
    .SLICE_IDX_BITS(LACTB),
    .SLICE_SIZE_BITS(SLICE_SIZE_SZ),
    .OUT_DATA_SZ(SLICE_SIZE_SZ),
    .OUT_DATA_BITS(OUT_DATA_BITS)
) slice_and_align (
    .slice_size_i(slice_sz_i),
    .slice_idx_i(slice_idx),
    .in_word_i(slice_in),
    .out_word_o(slice_data_o),
    .last_slice_o(word_last_slice)
);

// slice_data_idx_o is the WHOLE index of the returned element (the consumer
// needs the input-neuron number to address the weight row). slice_idx (the
// within-word part) is used only internally by slice_and_align. (Was = slice_idx,
// which truncated to within-word bits → weight_row_base aliased per dataword.)
// This is the shared cache for annAcc/snnAcc/ipSnnAcc in the closed-loop build.
assign slice_data_idx_o     = sys_addr_i;
assign slice_data_index_x_o = sys_index_x_i;
assign slice_data_index_y_o = sys_index_y_i;
assign slice_data_last_o    = sys_last_i;

endmodule
