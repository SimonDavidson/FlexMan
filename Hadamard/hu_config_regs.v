// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
`include "../shared/constants.v"

module hu_config_regs #(
    parameter TGT_CONFIG_BASE_ADDR = 16'hffff,
    parameter ADDR_SZ              = `ADDR_SIZE,
    parameter ACT_SZ               = `POT_OUT_SZ_SZ,
    parameter NUM_ELEM_SZ          = 16
)(
    input  wire                   clk,
    input  wire                   reset,

    /* Configuration write interface */
    input  wire                   hu_sys_req_i,
    output wire                   hu_sys_ack_o,
    input  wire [31:0]            hu_sys_addr_i,
    input  wire [31:0]            hu_sys_data_i,

    /* Register outputs */
    output reg                    mode_r_o,
    output reg  [NUM_ELEM_SZ-1:0] stream_len_r_o,

    output reg  [ADDR_SZ-1:0]     src_a_base_addr_r_o,
    output reg  [ACT_SZ-1:0]      src_a_elem_sz_r_o,
    output reg  [4:0]             src_a_bin_point_r_o,

    output reg  [ADDR_SZ-1:0]     src_b_base_addr_r_o,
    output reg  [ACT_SZ-1:0]      src_b_elem_sz_r_o,
    output reg  [4:0]             src_b_bin_point_r_o,

    output reg  [ADDR_SZ-1:0]     src_z_base_addr_r_o,
    output reg  [ACT_SZ-1:0]      src_z_elem_sz_r_o,
    output reg  [4:0]             src_z_bin_point_r_o,

    output reg  [ADDR_SZ-1:0]     src_r_base_addr_r_o,
    output reg  [ACT_SZ-1:0]      src_r_elem_sz_r_o,
    output reg  [4:0]             src_r_bin_point_r_o
);

wire        addr_match;
wire [7:0]  reg_sel;

assign addr_match  = (hu_sys_addr_i[31:16] == TGT_CONFIG_BASE_ADDR);
assign reg_sel     = hu_sys_addr_i[7:0];
assign hu_sys_ack_o = hu_sys_req_i & addr_match;

always @(posedge clk) begin
    if (reset) begin
        mode_r_o             <= 1'b0;
        stream_len_r_o       <= {NUM_ELEM_SZ{1'b0}};
        src_a_base_addr_r_o  <= {ADDR_SZ{1'b0}};
        src_a_elem_sz_r_o    <= {ACT_SZ{1'b0}};
        src_a_bin_point_r_o  <= 5'b0;
        src_b_base_addr_r_o  <= {ADDR_SZ{1'b0}};
        src_b_elem_sz_r_o    <= {ACT_SZ{1'b0}};
        src_b_bin_point_r_o  <= 5'b0;
        src_z_base_addr_r_o  <= {ADDR_SZ{1'b0}};
        src_z_elem_sz_r_o    <= {ACT_SZ{1'b0}};
        src_z_bin_point_r_o  <= 5'b0;
        src_r_base_addr_r_o  <= {ADDR_SZ{1'b0}};
        src_r_elem_sz_r_o    <= {ACT_SZ{1'b0}};
        src_r_bin_point_r_o  <= 5'b0;
    end else if (hu_sys_req_i & addr_match) begin
        case (reg_sel)
            8'h00: mode_r_o            <= hu_sys_data_i[0];
            8'h01: stream_len_r_o      <= hu_sys_data_i[NUM_ELEM_SZ-1:0];
            8'h04: src_a_base_addr_r_o <= hu_sys_data_i[ADDR_SZ-1:0];
            8'h05: src_a_elem_sz_r_o   <= hu_sys_data_i[ACT_SZ-1:0];
            8'h06: src_a_bin_point_r_o <= hu_sys_data_i[4:0];
            8'h08: src_b_base_addr_r_o <= hu_sys_data_i[ADDR_SZ-1:0];
            8'h09: src_b_elem_sz_r_o   <= hu_sys_data_i[ACT_SZ-1:0];
            8'h0A: src_b_bin_point_r_o <= hu_sys_data_i[4:0];
            8'h0C: src_z_base_addr_r_o <= hu_sys_data_i[ADDR_SZ-1:0];
            8'h0D: src_z_elem_sz_r_o   <= hu_sys_data_i[ACT_SZ-1:0];
            8'h0E: src_z_bin_point_r_o <= hu_sys_data_i[4:0];
            8'h10: src_r_base_addr_r_o <= hu_sys_data_i[ADDR_SZ-1:0];
            8'h11: src_r_elem_sz_r_o   <= hu_sys_data_i[ACT_SZ-1:0];
            8'h12: src_r_bin_point_r_o <= hu_sys_data_i[4:0];
            default: ; /* ignore unknown addresses */
        endcase
    end
end

endmodule
/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
