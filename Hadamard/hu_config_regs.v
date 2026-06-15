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

/* Register map: ONE 32-bit register per word (byte offsets 0x00,0x04,...,0x34),
 * so the config_manager's per-task stream (word i -> byte i*4) delivers all 14
 * fields in order, and a 32-bit bus write lands one whole register. (Was a
 * unit-stride index 0x00,0x01,0x04,... that only the behavioural TB could drive;
 * the config_manager's word*4 grid could never hit stream_len/elem_sz/bin_point.
 * 2026-06-15.) Field i (below) sits at byte offset i*4 == regmap.HU_REG_OFFSETS. */

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
            8'h00: mode_r_o            <= hu_sys_data_i[0];                  /* w0  */
            8'h04: stream_len_r_o      <= hu_sys_data_i[NUM_ELEM_SZ-1:0];    /* w1  */
            8'h08: src_a_base_addr_r_o <= hu_sys_data_i[ADDR_SZ-1:0];        /* w2  */
            8'h0C: src_a_elem_sz_r_o   <= hu_sys_data_i[ACT_SZ-1:0];         /* w3  */
            8'h10: src_a_bin_point_r_o <= hu_sys_data_i[4:0];               /* w4  */
            8'h14: src_b_base_addr_r_o <= hu_sys_data_i[ADDR_SZ-1:0];        /* w5  */
            8'h18: src_b_elem_sz_r_o   <= hu_sys_data_i[ACT_SZ-1:0];         /* w6  */
            8'h1C: src_b_bin_point_r_o <= hu_sys_data_i[4:0];               /* w7  */
            8'h20: src_z_base_addr_r_o <= hu_sys_data_i[ADDR_SZ-1:0];        /* w8  */
            8'h24: src_z_elem_sz_r_o   <= hu_sys_data_i[ACT_SZ-1:0];         /* w9  */
            8'h28: src_z_bin_point_r_o <= hu_sys_data_i[4:0];               /* w10 */
            8'h2C: src_r_base_addr_r_o <= hu_sys_data_i[ADDR_SZ-1:0];        /* w11 */
            8'h30: src_r_elem_sz_r_o   <= hu_sys_data_i[ACT_SZ-1:0];         /* w12 */
            8'h34: src_r_bin_point_r_o <= hu_sys_data_i[4:0];               /* w13 */
            default: ; /* ignore unknown addresses */
        endcase
    end
end

endmodule
/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
