/*
 * slice_and_align.v
 *
 * @author: Simon Davidson
 * @author: Samuel López
 */

module slice_and_align #(
    parameter IN_DATA_BITS    = 32,
    parameter SLICE_IDX_BITS  = 5,
    parameter SLICE_SIZE_BITS = 3,
    parameter OUT_DATA_SZ     = 3,
    parameter OUT_DATA_BITS   = 32)
(
input  wire   [SLICE_SIZE_BITS-1:0] slice_size_i,
input  wire    [SLICE_IDX_BITS-1:0] slice_idx_i,
input  wire      [IN_DATA_BITS-1:0] in_word_i,
output wire     [OUT_DATA_BITS-1:0] out_word_o,
output wire                         last_slice_o
);

wire [IN_DATA_BITS-1:0] assembled_data;
reg  [IN_DATA_BITS-1:0] data_mask;
wire [IN_DATA_BITS-1:0] masked_assembled_data;

wire [15:0] bits15_0;
wire [7:0]  bits23_16;
wire [3:0]  bits27_24;
wire [1:0]  bits29_28;
wire        bit30;
wire        bit31;

// Slice sizes: (3-bit)
// 1-bit:  3'b000
// 2-bit:  3'b001
// 4-bit:  3'b010
// 8-bit:  3'b011
// 16-bit: 3'b100
// 32-bit: 3'b101

// Slice indices: (5-bit value)
// 0 -> 31

// Is this the last slice in the current data word?

assign last_slice_o = (
      (slice_size_i == 3'b000 & slice_idx_i == 5'b11111) || // 1-bit elements
      (slice_size_i == 3'b001 & slice_idx_i == 5'b01111) || // 2-bit elements
      (slice_size_i == 3'b010 & slice_idx_i == 5'b00111) || // 4-bit elements
      (slice_size_i == 3'b011 & slice_idx_i == 5'b00011) || // 8-bit elements
      (slice_size_i == 3'b100 & slice_idx_i == 5'b00001) || //16-bit elements
      (slice_size_i == 3'b101                          ));  //32-bit elements


/*            32-bit   / X
 * -----------------------
 * o[15:0]  = i[15:0]  / X
 */
assign bits15_0 = in_word_i[15:0];

/*            32-bit   / 16-bit       / X
 * --------------------------------------
 * o[23:16] = i[23:16] / i[7:0]   (0) / X
 *                     / i[23:16] (1) /
 */
assign bits23_16 = ((slice_size_i == 3'b101) || 
	                (slice_size_i == 3'b100 && slice_idx_i == 5'b00001)     )? in_word_i[23:16] :
                                                                           in_word_i[7:0];

/*            32-bit   / 16-bit       / 8-bit        / X
 * -----------------------------------------------------
 * o[27:24] = i[27:24] / i[11:8]  (0) / i[3:0]   (0) / X
 *                     / i[27:24] (1) / i[11:8]  (1) /
 *                                    / i[19:16] (2) /
 *                                    / i[27:24] (3) /
 */
assign bits27_24 = ((slice_size_i == 3'b101) || 
	                (slice_size_i == 3'b100 && slice_idx_i == 5'b00001) ||
                    (slice_size_i == 3'b011 && slice_idx_i == 5'b00011)     )? in_word_i[27:24] : 

	               ((slice_size_i == 3'b100 && slice_idx_i == 5'b00000) ||
	                (slice_size_i == 3'b011 && slice_idx_i == 5'b00001)     )? in_word_i[11:8] :

	                (slice_idx_i == 5'b00010)? in_word_i[19:16] :
	                                         in_word_i[3:0];

/*            32-bit   / 16-bit       / 8-bit        / 4-bit        / X
 * --------------------------------------------------------------------
 * o[29:28] = i[29:28] / i[13:12] (0) / i[5:4]   (0) / i[1:0]   (0) / X
 *                     / i[29:28] (1) / i[13:12] (1) / i[5:4]   (1) /
 *                                    / i[21:20] (2) / i[9:8]   (2) /
 *                                    / i[29:28] (3) / i[13:12] (3) /
 *                                                   / i[17:16] (4) /
 *                                                   / i[21:20] (5) /
 *                                                   / i[25:24] (6) /
 *                                                   / i[29:28] (7) /
 */
assign bits29_28 = ((slice_size_i == 3'b101) || 
                    (slice_size_i == 3'b100 && slice_idx_i == 5'b00001) ||
	                (slice_size_i == 3'b011 && slice_idx_i == 5'b00011) ||
	                (slice_size_i == 3'b010 && slice_idx_i == 5'b00111)     )? in_word_i[29:28] :

		           ((slice_size_i == 3'b011 && slice_idx_i == 5'b00010) ||
	                (slice_size_i == 3'b010 && slice_idx_i == 5'b00101)     )? in_word_i[21:20] :

	               ((slice_size_i == 3'b100 && slice_idx_i == 5'b00000) ||
	                (slice_size_i == 3'b011 && slice_idx_i == 5'b00001) ||
		            (slice_size_i == 3'b010 && slice_idx_i == 5'b00011)     )? in_word_i[13:12] :

		           ((slice_size_i == 3'b011 && slice_idx_i == 5'b00000) ||
	                (slice_size_i == 3'b010 && slice_idx_i == 5'b00001)     )? in_word_i[5:4] :

	                (slice_idx_i == 5'b00110)? in_word_i[25:24] :
	                (slice_idx_i == 5'b00100)? in_word_i[17:16] :
	                (slice_idx_i == 5'b00010)? in_word_i[9:8] :
                                             in_word_i[1:0];

/*         32-bit  / 16-bit    / 8-bit     / 4-bit     / 2-bit      / X
 * --------------------------------------------------------------------
 * o[30] = i[30]   / i[14] (0) / i[6]  (0) / i[2]  (0) / i[0]   (0) / X
 *                 / i[30] (1) / i[14] (1) / i[6]  (1) / i[2]   (1) /
 *                             / i[22] (2) / i[10] (2) / i[4]   (2) /
 *                             / i[30] (3) / i[14] (3) / i[6]   (3) /
 *                                         / i[18] (4) / i[8]   (4) /
 *                                         / i[22] (5) / i[10]  (5) /
 *                                         / i[26] (6) / i[12]  (6) /
 *                                         / i[30] (7) / i[14]  (7) /
 *                                                     / i[16]  (8) /
 *                                                     / i[18]  (9) /
 *                                                     / i[20] (10) /
 *                                                     / i[22] (11) /
 *                                                     / i[24] (12) /
 *                                                     / i[26] (13) /
 *                                                     / i[28] (14) /
 *                                                     / i[30] (15) /
 */
assign bit30     = ((slice_size_i == 3'b101) ||
                    (slice_size_i == 3'b100 && slice_idx_i == 5'b00001) ||
	                (slice_size_i == 3'b011 && slice_idx_i == 5'b00011) ||
	                (slice_size_i == 3'b010 && slice_idx_i == 5'b00111) ||
	                (slice_size_i == 3'b001 && slice_idx_i == 5'b01111)     )? in_word_i[30] :

		           ((slice_size_i == 3'b010 && slice_idx_i == 5'b00110) ||
	                (slice_size_i == 3'b001 && slice_idx_i == 5'b01101)     )? in_word_i[26] :

		           ((slice_size_i == 3'b011 && slice_idx_i == 5'b00010) ||
	                (slice_size_i == 3'b010 && slice_idx_i == 5'b00101) ||
	                (slice_size_i == 3'b001 && slice_idx_i == 5'b01011)     )? in_word_i[22] :

		           ((slice_size_i == 3'b010 && slice_idx_i == 5'b00100) ||
	                (slice_size_i == 3'b001 && slice_idx_i == 5'b01001)     )? in_word_i[18] :

                   ((slice_size_i == 3'b100 && slice_idx_i == 5'b00000) ||
	                (slice_size_i == 3'b011 && slice_idx_i == 5'b00001) ||
	                (slice_size_i == 3'b010 && slice_idx_i == 5'b00011) ||
	                (slice_size_i == 3'b001 && slice_idx_i == 5'b00111)     )? in_word_i[14] :

		           ((slice_size_i == 3'b010 && slice_idx_i == 5'b00010) ||
	                (slice_size_i == 3'b001 && slice_idx_i == 5'b00101)     )? in_word_i[10] :

		           ((slice_size_i == 3'b011 && slice_idx_i == 5'b00000) ||
	                (slice_size_i == 3'b010 && slice_idx_i == 5'b00001) ||
	                (slice_size_i == 3'b001 && slice_idx_i == 5'b00011)     )? in_word_i[6] :

		           ((slice_size_i == 3'b010 && slice_idx_i == 5'b00000) ||
	                (slice_size_i == 3'b001 && slice_idx_i == 5'b00001)     )? in_word_i[2] :

	                (slice_idx_i == 5'b01110)? in_word_i[28] :
	                (slice_idx_i == 5'b01100)? in_word_i[24] :
	                (slice_idx_i == 5'b01010)? in_word_i[20] :
	                (slice_idx_i == 5'b01000)? in_word_i[16] :
	                (slice_idx_i == 5'b00110)? in_word_i[12] :
	                (slice_idx_i == 5'b00100)? in_word_i[8] :
	                (slice_idx_i == 5'b00010)? in_word_i[4] :
                                             in_word_i[0];

/*         32-bit  / 16-bit    / 8-bit     / 4-bit     / 2-bit      / 1-bit
 * -------------------------------------------------------------------------------
 * o[31] = i[31]   / i[15] (0) / i[7]  (0) / i[3]  (0) / i[1]   (0) / i[0]   (0) /
 *                 / i[31] (1) / i[15] (1) / i[7]  (1) / i[3]   (1) / i[1]   (1) /
 *                             / i[23] (2) / i[11] (2) / i[5]   (2) / i[2]   (2) /
 *                             / i[31] (3) / i[15] (3) / i[7]   (3) / i[3]   (3) /
 *                                         / i[19] (4) / i[9]   (4) / i[4]   (4) /
 *                                         / i[23] (5) / i[11]  (5) / i[5]   (5) /
 *                                         / i[27] (6) / i[13]  (6) / i[6]   (6) /
 *                                         / i[31] (7) / i[15]  (7) / i[7]   (7) /
 *                                                     / i[17]  (8) / i[8]   (8) /
 *                                                     / i[19]  (9) / i[9]   (9) /
 *                                                     / i[21] (10) / i[10] (10) /
 *                                                     / i[23] (11) / i[11] (11) /
 *                                                     / i[25] (12) / i[12] (12) /
 *                                                     / i[27] (13) / i[13] (13) /
 *                                                     / i[29] (14) / i[14] (14) /
 *                                                     / i[31] (15) / i[15] (15) /
 *                                                                  / i[16] (16) /
 *                                                                  / i[17] (17) /
 *                                                                  / i[18] (18) /
 *                                                                  / i[19] (19) /
 *                                                                  / i[20] (20) /
 *                                                                  / i[21] (21) /
 *                                                                  / i[22] (22) /
 *                                                                  / i[23] (23) /
 *                                                                  / i[24] (24) /
 *                                                                  / i[25] (25) /
 *                                                                  / i[26] (26) /
 *                                                                  / i[27] (27) /
 *                                                                  / i[28] (28) /
 *                                                                  / i[29] (29) /
 *                                                                  / i[30] (30) /
 *                                                                  / i[31] (31) /
 */
assign bit31     = ((slice_size_i == 3'b101)||
                    (slice_size_i == 3'b100 && slice_idx_i == 5'b00001) ||
	                (slice_size_i == 3'b011 && slice_idx_i == 5'b00011) ||
	                (slice_size_i == 3'b010 && slice_idx_i == 5'b00111) ||
	                (slice_size_i == 3'b001 && slice_idx_i == 5'b01111) ||
	                (slice_size_i == 3'b000 && slice_idx_i == 5'b11111)     )? in_word_i[31] :

		           ((slice_size_i == 3'b001 && slice_idx_i == 5'b01110) ||
	                (slice_size_i == 3'b000 && slice_idx_i == 5'b11101)     )? in_word_i[29] :

		           ((slice_size_i == 3'b010 && slice_idx_i == 5'b00110) ||
	                (slice_size_i == 3'b001 && slice_idx_i == 5'b01101) ||
	                (slice_size_i == 3'b000 && slice_idx_i == 5'b11011)     )? in_word_i[27] :

		           ((slice_size_i == 3'b001 && slice_idx_i == 5'b01100) ||
	                (slice_size_i == 3'b000 && slice_idx_i == 5'b11001)     )? in_word_i[25] :

		           ((slice_size_i == 3'b011 && slice_idx_i == 5'b00010) ||
	                (slice_size_i == 3'b010 && slice_idx_i == 5'b00101) ||
	                (slice_size_i == 3'b001 && slice_idx_i == 5'b01011) ||
	                (slice_size_i == 3'b000 && slice_idx_i == 5'b10111)     )? in_word_i[23] :

		           ((slice_size_i == 3'b001 && slice_idx_i == 5'b01010) ||
	                (slice_size_i == 3'b000 && slice_idx_i == 5'b10101)     )? in_word_i[21] :

		           ((slice_size_i == 3'b010 && slice_idx_i == 5'b00100) ||
	                (slice_size_i == 3'b001 && slice_idx_i == 5'b01001) ||
	                (slice_size_i == 3'b000 && slice_idx_i == 5'b10011)     )? in_word_i[19] :

		           ((slice_size_i == 3'b001 && slice_idx_i == 5'b01000) ||
	                (slice_size_i == 3'b000 && slice_idx_i == 5'b10001)     )? in_word_i[17] :

	               ((slice_size_i == 3'b100 && slice_idx_i == 5'b00000) ||
	                (slice_size_i == 3'b011 && slice_idx_i == 5'b00001) ||
	                (slice_size_i == 3'b010 && slice_idx_i == 5'b00011) ||
	                (slice_size_i == 3'b001 && slice_idx_i == 5'b00111) ||
	                (slice_size_i == 3'b000 && slice_idx_i == 5'b01111)     )? in_word_i[15] :

		           ((slice_size_i == 3'b001 && slice_idx_i == 5'b00110) ||
	                (slice_size_i == 3'b000 && slice_idx_i == 5'b01101)     )? in_word_i[13] :

		           ((slice_size_i == 3'b010 && slice_idx_i == 5'b00010) ||
	                (slice_size_i == 3'b001 && slice_idx_i == 5'b00101) ||
	                (slice_size_i == 3'b000 && slice_idx_i == 5'b01011)     )? in_word_i[11] : 

		           ((slice_size_i == 3'b001 && slice_idx_i == 5'b00100) ||
	                (slice_size_i == 3'b000 && slice_idx_i == 5'b01001)     )? in_word_i[9] :

		           ((slice_size_i == 3'b011 && slice_idx_i == 5'b00000) ||
	                (slice_size_i == 3'b010 && slice_idx_i == 5'b00001) ||
	                (slice_size_i == 3'b001 && slice_idx_i == 5'b00011) ||
	                (slice_size_i == 3'b000 && slice_idx_i == 5'b00111)     )? in_word_i[7] :

		           ((slice_size_i == 3'b001 && slice_idx_i == 5'b00010) ||
	                (slice_size_i == 3'b000 && slice_idx_i == 5'b00101)     )? in_word_i[5] :

		           ((slice_size_i == 3'b010 && slice_idx_i == 5'b00000) ||
	                (slice_size_i == 3'b001 && slice_idx_i == 5'b00001) ||
	                (slice_size_i == 3'b000 && slice_idx_i == 5'b00011)     )? in_word_i[3] :

		           ((slice_size_i == 3'b001 && slice_idx_i == 5'b00000) ||
	                (slice_size_i == 3'b000 && slice_idx_i == 5'b00001)     )? in_word_i[1] :

		            (slice_idx_i == 5'b11110)? in_word_i[30] :
		            (slice_idx_i == 5'b11100)? in_word_i[28] :
		            (slice_idx_i == 5'b11010)? in_word_i[26] :
		            (slice_idx_i == 5'b11000)? in_word_i[24] :
		            (slice_idx_i == 5'b10110)? in_word_i[22] :
		            (slice_idx_i == 5'b10100)? in_word_i[20] :
		            (slice_idx_i == 5'b10010)? in_word_i[18] :
		            (slice_idx_i == 5'b10000)? in_word_i[16] :
		            (slice_idx_i == 5'b01110)? in_word_i[14] :
		            (slice_idx_i == 5'b01100)? in_word_i[12] :
		            (slice_idx_i == 5'b01010)? in_word_i[10] :
		            (slice_idx_i == 5'b01000)? in_word_i[8] :
		            (slice_idx_i == 5'b00110)? in_word_i[6] :
		            (slice_idx_i == 5'b00100)? in_word_i[4] :
		            (slice_idx_i == 5'b00010)? in_word_i[2] :
		                                       in_word_i[0];

assign assembled_data = {bit31, bit30, bits29_28, bits27_24, bits23_16, bits15_0};

// Create mask for output word:
always @ (slice_size_i)
begin
    case (slice_size_i)
        3'b000:  data_mask = 32'H80000000; //  1-bit
        3'b001:  data_mask = 32'HC0000000; //  2-bit
        3'b010:  data_mask = 32'HF0000000; //  4-bit
        3'b011:  data_mask = 32'HFF000000; //  8-bit
        3'b100:  data_mask = 32'HFFFF0000; // 16-bit
        3'b101:  data_mask = 32'HFFFFFFFF; // 32-bit
        default: data_mask = 32'H0F0F0F0F; // Error!
    endcase
end

// Mask off unwanted bits:
assign masked_assembled_data = assembled_data & data_mask; 
assign out_word_o = masked_assembled_data[IN_DATA_BITS-1:IN_DATA_BITS-OUT_DATA_BITS];

endmodule
