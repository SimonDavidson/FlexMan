// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
/////////////////////////////////////////////////////////////////////
//
// ann_tile_path -- optional NLANE output-stationary dense path for annAcc.
//
// Authors      : Simon Davidson & Claude
// Created      : 2026-11-21
// Last modified: 2026-11-21
//
// Instantiated by ann_processor ONLY when NLANE > 1 (a generate); with the
// default NLANE = 1 this module does not exist in the elaborated design and
// ann_spike_processing is wired straight to the ports, as before.
//
// At each dispatch the path is chosen once:
//   tile  : weight_mode == full, 8/16-bit weights with NLANE <= weights/word,
//           activation width <= MAC_ACT_BITS, non-zero dims  -> ann_tile_sp
//   legacy: everything else (sparse, conv, other widths)      -> ann_spike_processing
// The legacy block is held idle (legacy_start_o = 0) for tile tasks. The two
// paths share the weight / act / syn_curr memory ports through a mux.
/////////////////////////////////////////////////////////////////////
`include "../shared/constants.v"

module ann_tile_path #(
    parameter NLANE           = 4,
    parameter MAC_ACT_BITS    = 32,
    parameter X_INPUT_SZ      = 8,
    parameter Y_INPUT_SZ      = 8,
    parameter X_OUTPUT_SZ     = 8,
    parameter Y_OUTPUT_SZ     = 8,
    parameter ROWS_PER_NEURON = 16,
    parameter WEIGHT_SLICE_SZ = 5,
    parameter MEM_ADDR_BITS   = `ADDR_SIZE
)(
    input  wire clk,
    input  wire reset,
    input  wire start_i,
    // task configuration (static during a task)
    input  wire [MEM_ADDR_BITS-1:0]   act_base_addr_i,
    input  wire [MEM_ADDR_BITS-1:0]   weight_base_addr_i,
    input  wire [MEM_ADDR_BITS-1:0]   syn_curr_base_addr_i,
    input  wire [WEIGHT_SLICE_SZ-1:0] weight_sz_i,
    input  wire [1:0]                 weight_mode_i,
    input  wire [2:0]                 act_slice_sz_i,
    input  wire                       act_signed_i,
    input  wire [X_INPUT_SZ-1:0]      in_x_len_i,
    input  wire [Y_INPUT_SZ-1:0]      in_y_len_i,
    input  wire [X_OUTPUT_SZ-1:0]     out_x_len_i,
    input  wire [Y_OUTPUT_SZ-1:0]     out_y_len_i,
    input  wire [ROWS_PER_NEURON-1:0] rows_per_neuron_i,
    // to the legacy spike_processing
    output wire                       legacy_start_o,
    // legacy-path outputs
    input  wire                       l_fin_i, l_busy_i, l_afin_i,
    input  wire                       l_w_rd_i, l_a_req_i, l_s_wr_i, l_s_rd_i,
    input  wire [`ADDR_SIZE-1:0]      l_w_addr_i, l_a_addr_i, l_s_addr_i,
    input  wire [`POT_BITS-1:0]       l_s_dwr_i,
    // merged outputs (to the ports)
    output wire                       spike_proc_finished_o,
    output wire                       acc_busy_o,
    output wire                       acc_finished_o,
    output wire                       weight_mem_rd_o,
    output wire [`ADDR_SIZE-1:0]      weight_mem_addr_o,
    output wire                       act_mem_req_o,
    output wire [`ADDR_SIZE-1:0]      act_mem_addr_o,
    output wire                       syn_curr_mem_wr_o,
    output wire                       syn_curr_mem_rd_o,
    output wire [`ADDR_SIZE-1:0]      syn_curr_mem_addr_o,
    output wire [`POT_BITS-1:0]       syn_curr_mem_data_wr_o,
    // memory returns / wait (tile path reads them directly)
    input  wire                       weight_mem_wait_i,
    input  wire [`WTD_BITS-1:0]       weight_mem_data_i,
    input  wire                       act_mem_wait_i,
    input  wire [`ACT_BITS-1:0]       act_mem_data_i,
    input  wire                       syn_curr_mem_wait_i,
    input  wire [`POT_BITS-1:0]       syn_curr_mem_data_rd_i
);
    wire is_full = (weight_mode_i == 2'b00);
    wire w_ok    = ((weight_sz_i == 3) || (weight_sz_i == 4)) &&
                   (NLANE <= (32 >> weight_sz_i));
    wire a_ok    = (act_slice_sz_i <= 3'd5) && ((32'd1 << act_slice_sz_i) <= MAC_ACT_BITS);
    wire dims_ok = ((in_x_len_i * in_y_len_i) != 0) && ((out_x_len_i * out_y_len_i) != 0);
    wire tile_ok = is_full & w_ok & a_ok & dims_ok;

    reg use_tile_r;
    always @(posedge clk)
        if (reset) use_tile_r <= 1'b0;
        else if (start_i) use_tile_r <= tile_ok;

    assign legacy_start_o = start_i & ~tile_ok;

    wire                  t_fin, t_busy;
    wire                  t_w_rd, t_a_req, t_s_wr, t_s_rd;
    wire [`ADDR_SIZE-1:0] t_w_addr, t_a_addr, t_s_addr;
    wire  [`POT_BITS-1:0] t_s_dwr;

    ann_tile_sp # (.NLANE(NLANE), .MAC_ACT_BITS(MAC_ACT_BITS),
        .X_INPUT_SZ(X_INPUT_SZ), .Y_INPUT_SZ(Y_INPUT_SZ),
        .X_OUTPUT_SZ(X_OUTPUT_SZ), .Y_OUTPUT_SZ(Y_OUTPUT_SZ),
        .ROWS_PER_NEURON(ROWS_PER_NEURON), .WEIGHT_SLICE_SZ(WEIGHT_SLICE_SZ),
        .MEM_ADDR_BITS(MEM_ADDR_BITS)) u_tile (
        .clk(clk), .reset(reset),
        .start_i(start_i & tile_ok),
        .act_base_addr_i(act_base_addr_i), .weight_base_addr_i(weight_base_addr_i),
        .syn_curr_base_addr_i(syn_curr_base_addr_i),
        .weight_sz_i(weight_sz_i), .act_slice_sz_i(act_slice_sz_i), .act_signed_i(act_signed_i),
        .in_x_len_i(in_x_len_i), .in_y_len_i(in_y_len_i),
        .out_x_len_i(out_x_len_i), .out_y_len_i(out_y_len_i),
        .rows_per_neuron_i(rows_per_neuron_i),
        .busy_o(t_busy), .finished_o(t_fin),
        .weight_mem_rd_o(t_w_rd), .weight_mem_wait_i(weight_mem_wait_i),
        .weight_mem_addr_o(t_w_addr), .weight_mem_data_i(weight_mem_data_i),
        .act_mem_req_o(t_a_req), .act_mem_wait_i(act_mem_wait_i),
        .act_mem_addr_o(t_a_addr), .act_mem_data_i(act_mem_data_i),
        .syn_curr_mem_wr_o(t_s_wr), .syn_curr_mem_rd_o(t_s_rd),
        .syn_curr_mem_wait_i(syn_curr_mem_wait_i), .syn_curr_mem_addr_o(t_s_addr),
        .syn_curr_mem_data_wr_o(t_s_dwr), .syn_curr_mem_data_rd_i(syn_curr_mem_data_rd_i));

    assign spike_proc_finished_o = l_fin_i | t_fin;
    assign acc_finished_o        = l_afin_i | t_fin;
    assign acc_busy_o            = l_busy_i | t_busy;
    assign weight_mem_rd_o       = use_tile_r ? t_w_rd   : l_w_rd_i;
    assign weight_mem_addr_o     = use_tile_r ? t_w_addr : l_w_addr_i;
    assign act_mem_req_o         = use_tile_r ? t_a_req  : l_a_req_i;
    assign act_mem_addr_o        = use_tile_r ? t_a_addr : l_a_addr_i;
    assign syn_curr_mem_wr_o     = use_tile_r ? t_s_wr   : l_s_wr_i;
    assign syn_curr_mem_rd_o     = use_tile_r ? t_s_rd   : l_s_rd_i;
    assign syn_curr_mem_addr_o   = use_tile_r ? t_s_addr : l_s_addr_i;
    assign syn_curr_mem_data_wr_o= use_tile_r ? t_s_dwr  : l_s_dwr_i;
endmodule
