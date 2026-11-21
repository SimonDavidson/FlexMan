// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
// =============================================================================
// Module: np_fetch_sel
// Authors: Simon Davidson & Claude
// Created: 2026-06-04
// Last modified: 2026-06-04
//
// Thin selector: instantiates the prefetching read front-end (np_line_fetch_pf)
// when PREFETCH=1, otherwise the plain one (np_line_fetch).  Identical port
// list to both, so neuron_processing_wide can switch by a single parameter.
// =============================================================================
`timescale 1ns / 1ps

module np_fetch_sel #(
    parameter PREFETCH     = 0,
    parameter IN_DATA_BITS = 32,
    parameter ELEM_BITS    = 8,
    parameter NUM_LANES    = 2,
    parameter IDX_BITS     = 10,
    parameter ADDR_BITS    = 30
)(
    input  wire                           clk,
    input  wire                           reset,
    input  wire        [ADDR_BITS-1:0]    base_addr_i,
    input  wire         [IDX_BITS-1:0]    start_index_i,
    input  wire                           req_i,
    output wire                           data_valid_o,
    output wire [NUM_LANES*ELEM_BITS-1:0] slices_o,
    output wire                           mem_rd_o,
    output wire        [ADDR_BITS-1:0]    mem_addr_o,
    input  wire     [IN_DATA_BITS-1:0]    mem_data_i,
    input  wire                           mem_wait_i
);
    generate
        if (PREFETCH) begin : g_pf
            np_line_fetch_pf #(.IN_DATA_BITS(IN_DATA_BITS), .ELEM_BITS(ELEM_BITS),
                               .NUM_LANES(NUM_LANES), .IDX_BITS(IDX_BITS), .ADDR_BITS(ADDR_BITS))
            u (.clk(clk), .reset(reset), .base_addr_i(base_addr_i),
               .start_index_i(start_index_i), .req_i(req_i), .data_valid_o(data_valid_o),
               .slices_o(slices_o), .mem_rd_o(mem_rd_o), .mem_addr_o(mem_addr_o),
               .mem_data_i(mem_data_i), .mem_wait_i(mem_wait_i));
        end else begin : g_plain
            np_line_fetch #(.IN_DATA_BITS(IN_DATA_BITS), .ELEM_BITS(ELEM_BITS),
                            .NUM_LANES(NUM_LANES), .IDX_BITS(IDX_BITS), .ADDR_BITS(ADDR_BITS))
            u (.clk(clk), .reset(reset), .base_addr_i(base_addr_i),
               .start_index_i(start_index_i), .req_i(req_i), .data_valid_o(data_valid_o),
               .slices_o(slices_o), .mem_rd_o(mem_rd_o), .mem_addr_o(mem_addr_o),
               .mem_data_i(mem_data_i), .mem_wait_i(mem_wait_i));
        end
    endgenerate
endmodule
