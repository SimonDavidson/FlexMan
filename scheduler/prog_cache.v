/*
 * prog_cache.v
 *
 * @author: SD, derived from act_cache by Samuel López + JDG
 */

`include "constants.v"

module prog_cache #(
    parameter IN_DATA_SZ    = 32,
    parameter SLICE_IDX_SZ  = 5,
    parameter SLICE_SIZE_SZ = 3)
(
    input wire clk,
    input wire reset,

    /* Control params */
    input  wire [SLICE_SIZE_SZ-1:0] slice_sz_i,
    input  wire [`PROG_BITS-1:0] read_base_i, 

    /* System side */
    input  wire [`COL_BITS-1:0]   sys_index_i,
    output wire [IN_DATA_SZ-1:0] sys_data_o,
    //output reg  sys_wait_o,
    input  wire sys_req_i,
    input  wire last_i,

    /* Memory side */
    output wire  [`ADDR_SIZE-1:0] mem_addr_o,                 /* Word address */
    input  wire [IN_DATA_SZ-1:0] mem_data_i,
    output wire mem_req_o,                             /* Memory read request */
    input  wire mem_wait_i
);

/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/

reg  [`COL_BITS-1:0] tag;                           /* Cached line high index */
wire [`COL_BITS-1:0] tag_mask;       /* Mask separating index into addr/field */
reg  [`ACT_BITS-1:0] cache_data;                               /* Cached word */
wire [`ACT_BITS-1:0] dout;      /* Currently valid data - sometimes forwarded */
reg                  valid;                                 /* Cache validity */
reg            [4:0] field;              /* Indexed field for data extraction */
reg            [2:0] slice_sz_L;            /* Held over for data shift phase */
/* Input signal state (from FIFO) not guaranteed when not actively requested. */
wire                 fill;                 /* Memory read request - fulfilled */
reg                  read_L;             /* Fill delayed: enable data capture */
wire                 hit;                                        /* Cache hit */

wire [`COL_BITS-1:0] index;                          /* Word index for memory */

assign tag_mask  = (1 << (5 - slice_sz_i)) - 1;
assign hit       = valid && !(|((sys_index_i ^ tag) & ~tag_mask));/* (== ish) */
assign mem_req_o = sys_req_i && !hit;
assign fill      = mem_req_o && !mem_wait_i; /* Read promises data next cycle */

always @ (posedge clk)
if (reset) valid <= 1'b0;
else
  begin
  if (fill) tag <= sys_index_i & ~tag_mask;                     /* Update tag */
  if (read_L)      /* Lags by a clock: latch data (forwarded in return cycle) */
    cache_data <= mem_data_i;
  read_L <= fill;                           /* Data will arrive a cycle later */
  if (sys_req_i && (hit || !mem_wait_i))
    begin
    valid <= !last_i;
    field <= sys_index_i[4:0] & tag_mask[4:0];     /* (Predicate superfluous) */
    slice_sz_L <= slice_sz_i;
    end
  end

assign dout = read_L ? mem_data_i : cache_data;  /* Choose memory/cached data */

assign index      = sys_index_i >> (5 - slice_sz_i);        /* Shift to index */
assign mem_addr_o = index + read_base_i;                /* Add base of vector */

assign sys_data_o = dout;

endmodule

/*============================================================================*/
