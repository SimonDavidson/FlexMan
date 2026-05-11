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
    parameter OUT_DATA_BITS     = 32)
(
    input wire                         clk,
    input wire                         reset,

    /* Control params */
    output wire                        colour_select_o,
    input  wire    [SLICE_SIZE_SZ-1:0] slice_sz_i,
    input  wire       [`ADDR_SIZE-1:0] base_addr_i, 

    /* System side - in */
    input wire     [IDX_ADDR_BITS-1:0] sys_addr_i,
    input wire                         sys_req_i,
    input wire        [X_INPUT_SZ-1:0] sys_index_x_i,
    input wire        [Y_INPUT_SZ-1:0] sys_index_y_i,

    input  wire                        sys_colour_i,
    output reg                         sys_wait_o,
    input  wire                        sys_last_i,
   
    /* System side - out */
    output reg                         slice_data_valid_o,
    output reg [SLICE_DATA_IDX_SZ-1:0] slice_data_idx_o,
    output wire       [X_INPUT_SZ-1:0] slice_data_index_x_o,
    output wire       [Y_INPUT_SZ-1:0] slice_data_index_y_o,
    output reg     [OUT_DATA_BITS-1:0] slice_data_o,
    output reg                         slice_data_last_o,
    input  reg                         slice_data_taken_i,

    /* Memory side */
    output reg      [`ADDR_SIZE-1:0]   mem_addr_o,
    input  wire     [IN_DATA_BITS-1:0] mem_data_i,
    output reg                         mem_req_o,
    input  wire                        mem_wait_i
);

localparam ACTW  = 32;     // Activation bit width
                           // TODO Implement as input to choose
                           // between activation bit widths

localparam LACTW = SLICE_SIZE_SZ; //$clog2(ACTW); // Log2 of data element bitwidth
localparam LACTB = $clog2(IN_DATA_BITS);   // Log2 of memory bit width

localparam cache = 1'b0, memory = 1'b1; // FSM states

/* FSM */
//reg                 sn, sr;     // FSM state
//reg [`COL_BITS-1:0] an, ar;     // Cached address
//reg [`ACT_BITS-1:0] dn, dr;     // Cached data
//reg                 vn, vr;     // Valid bit

/* Auxiliary signals */
reg [`COL_BITS-(LACTB-LACTW)-1:0] sys_addr;
reg [`COL_BITS-(LACTB-LACTW)-1:0] reg_addr;
reg                               cache_hit;
reg           [IDX_ADDR_BITS-1:0] shifted_index;


/* Slice-and-align signals */
reg          [LACTB-1:0] slice_idx;
reg   [IN_DATA_BITS-1:0] slice_in;
wire [OUT_DATA_BITS-1:0] slice_out;
wire                     cache_valid_nxt;
reg                      cache_valid_r;
reg  [IDX_ADDR_BITS-1:0] cache_addr_r;
wire                     mem_data_next_cycle;
reg                      mem_new_data_valid_r;
wire                     word_last_slice;
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

always @ (posedge clk)
   if (reset)
   begin
      cache_addr_r    <= 'b10101010;
      cache_data_r    <= 'b0;
   end
   else
   begin
      if (sys_req_i & ~sys_wait_o)
         begin
            cache_addr_r    <= sys_addr_i;
         end
      if (mem_new_data_valid_r)
         cache_data_r <= mem_data_i;
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
assign cache_valid_nxt = (mem_new_data_valid_r) ? 1'b1 :
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

assign colour_select_o = sys_colour_i;

////////////////////////////////////////////
// Generate address slices, for cache address and slice_and_align block
//

always @ (slice_sz_i, sys_addr_i, cache_addr_r)
begin
    case(slice_sz_i)
       3'b000: //  1-bit elements
       begin
          shifted_index = {1'b0,    sys_addr_i[IDX_ADDR_BITS-1:5]};
          reg_addr      = {1'b0, cache_addr_r[IDX_ADDR_BITS-1:5]};
       end
       3'b001: //  2-bit elements
       begin
          shifted_index = {1'b0,    sys_addr_i[IDX_ADDR_BITS-1:4]};
          reg_addr      = {1'b0, cache_addr_r[IDX_ADDR_BITS-1:4]};
       end
       3'b010: //  4-bit elements
       begin
          shifted_index = {1'b0,    sys_addr_i[IDX_ADDR_BITS-1:3]};
          reg_addr      = {1'b0, cache_addr_r[IDX_ADDR_BITS-1:3]};
       end
       3'b011: //  8-bit elements
       begin
          shifted_index = {1'b0,    sys_addr_i[IDX_ADDR_BITS-1:2]};
          reg_addr      = {1'b0, cache_addr_r[IDX_ADDR_BITS-1:2]};
       end
       3'b100: // 16-bit elements
       begin
          shifted_index = {1'b0,    sys_addr_i[IDX_ADDR_BITS-1:1]};
          reg_addr      = {1'b0, cache_addr_r[IDX_ADDR_BITS-1:1]};
       end
       3'b101: // 32-bit elements
       begin
          shifted_index = {1'b0,    sys_addr_i[IDX_ADDR_BITS-1:0]};
          reg_addr      = {1'b0, cache_addr_r[IDX_ADDR_BITS-1:0]};
       end
    endcase
end

// Cache hit - matching addresses and valid data
assign cache_hit = ((shifted_index == reg_addr) && cache_valid_r);

assign sys_addr = sys_addr_i[IDX_ADDR_BITS-1:LACTB-LACTW];

always @ (sys_addr_i, slice_sz_i)
begin
   slice_idx = 'b0;
   case (slice_sz_i)
      'b000:
         slice_idx = sys_addr_i[4:0];
      'b001:
         slice_idx = {1'b0, sys_addr_i[3:0]};
      'b010:
         slice_idx = {2'b0, sys_addr_i[2:0]};
      'b011:
         slice_idx = {3'b0, sys_addr_i[1:0]};
      'b100:
         slice_idx = {4'b0, sys_addr_i[0]};
      default: 
         slice_idx = 'hx;
   endcase
end

// Take the data dirctly from the memory bus if a request was accepted last
// cycle. If not, take the cached data:
assign slice_in = (mem_new_data_valid_r)? mem_data_i :
	                            cache_data_r;

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

assign mem_addr_o           = shifted_index + base_addr_i;

assign slice_data_valid_o   = sys_req_i & (cache_hit | mem_new_data_valid_r);
assign slice_data_idx_o     = slice_idx;
assign slice_data_index_x_o = sys_index_x_i;
assign slice_data_index_y_o = sys_index_y_i;
assign slice_data_last_o    = sys_last_i;

endmodule

