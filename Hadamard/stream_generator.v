// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
`include "../shared/constants.v"

module stream_generator #(
    parameter MAX_STREAM_LEN    = 1024,
    parameter IN_DATA_BITS      = 32,
    parameter SLICE_SIZE_SZ     = `POT_OUT_SZ_SZ,   /* 3 */
    parameter SLICE_DATA_IDX_SZ = 5,
    parameter OUT_DATA_BITS     = 32
)(
    input  wire                              clk,
    input  wire                              reset,

    /* Control */
    input  wire                              start_task_i,
    input  wire  [$clog2(MAX_STREAM_LEN):0]  stream_len_i, /* 1..MAX_STREAM_LEN */
    input  wire  [SLICE_SIZE_SZ-1:0]         slice_sz_i,
    input  wire  [`ADDR_SIZE-1:0]            base_addr_i,

    /* Status */
    output wire                              busy_o,

    /* Memory interface */
    output wire  [`ADDR_SIZE-1:0]            mem_addr_o,
    input  wire  [IN_DATA_BITS-1:0]          mem_data_i,
    output wire                              mem_req_o,
    input  wire                              mem_wait_i,

    /* Output element interface */
    output wire                              data_valid_o,
    output wire  [OUT_DATA_BITS-1:0]         data_o,
    output wire  [SLICE_DATA_IDX_SZ-1:0]     data_idx_o,
    output wire  [$clog2(MAX_STREAM_LEN)-1:0] data_global_idx_o, /* global element pos (F7 fix) */
    output wire                              data_last_o,
    input  wire                              data_taken_i  /* All downstream stall conditions */
);

localparam IDX_BITS = $clog2(MAX_STREAM_LEN);

reg                  active;
reg  [IDX_BITS-1:0]  index;

wire                 cache_wait;
wire                 last;
wire                 advance;

assign last    = ({1'b0, index} == stream_len_i - 1'b1);
assign advance = active & data_valid_o & ~cache_wait;

always @(posedge clk)
    if (reset) begin
        active <= 1'b0;
        index  <= {IDX_BITS{1'b0}};
    end else if (start_task_i) begin
        active <= 1'b1;
        index  <= {IDX_BITS{1'b0}};
    end else if (advance) begin
        if (last)
            active <= 1'b0;
        else
            index  <= index + 1'b1;
    end

assign busy_o = active;

/* Global element position (0..stream_len-1), combinationally aligned with
 * data_o/data_idx_o (all derive from the same `index` register).  Feeds the
 * output packer so multi-word streams address ascending output words (F7 fix
 * 2026-06-15).  data_idx_o is only the WITHIN-word slice index. */
assign data_global_idx_o = index;

dataline_cache_with_xy #(
    .IN_DATA_BITS      (IN_DATA_BITS),
    .X_INPUT_SZ        (1),
    .Y_INPUT_SZ        (1),
    .IDX_ADDR_BITS     (IDX_BITS),
    .SLICE_DATA_IDX_SZ (SLICE_DATA_IDX_SZ),
    .SLICE_SIZE_SZ     (SLICE_SIZE_SZ),
    .OUT_DATA_BITS     (OUT_DATA_BITS)
) u_cache (
    .clk                 (clk),
    .reset               (reset),
    // Invalidate the cached line at task start (matches every other cache user,
    // which ties invalidate_i to start_new_block_i). Leaving it unconnected floats
    // it to X, which poisons cache_valid → mem_req_o/data_valid/sys_wait go X and
    // the stream never advances (first surfaced when the Hadamard ran through the
    // shared pool for the GRU).
    .invalidate_i        (start_task_i),
    .colour_select_o     (),
    .slice_sz_i          (slice_sz_i),
    .base_addr_i         (base_addr_i),
    .sys_addr_i          (index),
    .sys_req_i           (active),
    .sys_index_x_i       (1'b0),
    .sys_index_y_i       (1'b0),
    .sys_colour_i        (1'b0),
    .sys_wait_o          (cache_wait),
    .sys_last_i          (active & last),
    .slice_data_valid_o  (data_valid_o),
    .slice_data_idx_o    (data_idx_o),
    .slice_data_index_x_o(),
    .slice_data_index_y_o(),
    .slice_data_o        (data_o),
    .slice_data_last_o   (data_last_o),
    .slice_data_taken_i  (data_taken_i),
    .mem_addr_o          (mem_addr_o),
    .mem_data_i          (mem_data_i),
    .mem_req_o           (mem_req_o),
    .mem_wait_i          (mem_wait_i)
);

endmodule

/*----------------------------------------------------------------------------*/
