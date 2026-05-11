// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
//`include "../shared/constants.v"

module packer(input  wire                  clk,
              input  wire                  reset,

              output wire                  busy_o,               /* Not empty */
              output wire                  finish_o,          /* Op. complete */

              input  wire                  pak_write_i,       /* Input strobe */
              output wire                  pak_full_o,         /* Don't input */
              input  wire                  pak_colour_i,
              input  wire                  pak_last_i,
              input  wire  [`PIN_BITS-1:0] pak_index_i,
              input  wire  [`POT_BITS-1:0] pak_acc_data_i,       /* New value */

              output wire                  pot_wr_o,          /* RAM write en */
              input  wire                  pot_wait_i,       /* RAM not ready */
              output wire [`ADDR_SIZE-1:0] pot_addr_o,         /* RAM address */
              output wire           [31:0] pot_data_o,         /* RAM wr data */

              output wire                      pak_colour_sel_o,
              input  wire [`POT_OUT_SZ_SZ-1:0] pak_out_sz_i,
              output reg                       pak_colour_bs_o,
              input  wire [`ADDR_SIZE-1:0]     pak_out_base_addr_i);

reg  [`POT_BITS-1:0] output_buffer;                   /* Accumulator register */
reg  [`PIN_BITS-1:0] pak_index_latched;    /* index that goes with the buffer */
reg                  buffer_valid;                       /* Holding something */
reg                  buffer_full;                        /* Wanting to output */
reg                  last;                     /* End of matrix mul. sequence */

wire           [4:0] pak_index_masked;                /* Isolated field index */
wire           [5:0] pak_index_plus;              /* Isolated field index + 1 */
wire                 writing;                  /* Write to memory going ahead */
wire                 top_bit;    /* Input field includes MSB: triggers output */

wire [`POT_OUT_SZ_SZ-1:0] index_shift;       /* Right shifts: index => offset */
wire      [`PIN_BITS-1:0] offset;                           /* Address offset */
reg                [31:0] data_mask; /* Data size masked before justification */
reg                 [4:0] shift_mask;
reg                 [6:0] data_shift;                /* To justify data field */
wire               [63:0] temp_data;  /* Post shift bus - TOO MANY BITS?! @@@ */
wire               [31:0] new_data;       /* As above, clipped to output word */


always @ (*)                          /* Right-justified mask for input field */
case (pak_out_sz_i)    /* Isolates part of index used for field pos. in word. */
  0: shift_mask = 5'h1F;                                     /*  1 bit fields */
  1: shift_mask = 5'h0F;                                     /*  2 bit fields */
  2: shift_mask = 5'h07;                                     /*  4 bit fields */
  3: shift_mask = 5'h03;                                     /*  8 bit fields */
  4: shift_mask = 5'h01;                                     /* 16 bit fields */
  5: shift_mask = 5'h00;                                     /* 32 bit fields */
  default: shift_mask = 5'h00;
endcase

assign pak_index_masked = pak_index_i[4:0] & shift_mask;
assign pak_index_plus   = {1'b0, pak_index_masked} + 6'h01;
/* This sets up for a shift which is 'one more' than the index value which    */
/* allows the left-justified input to emerge right-justified (if further up). */
/* Can't legally overflow. */

assign pot_wr_o = buffer_full;                    /* Once full request output */
assign writing  = pot_wr_o && !pot_wait_i;     /* Output request honoured now */

//always @ (*)        /* Indices always issued in ascending order => last entry */
assign top_bit = (pak_index_masked == shift_mask);
                                                /* Relevant field is all '1's */
                           /* Detect that a whole output word has accumulated */

always @ (*)                           /* Left-justified mask for input field */
case (pak_out_sz_i)
  0: data_mask = 32'h8000_0000;
  1: data_mask = 32'hC000_0000;
  2: data_mask = 32'hF000_0000;
  3: data_mask = 32'hFF00_0000;
  4: data_mask = 32'hFFFF_0000;
  5: data_mask = 32'hFFFF_FFFF;
  default: data_mask = 32'hFFFF_FFFF;
endcase

always @ (*)            /* Field shift amount to justify data field in buffer */
case (pak_out_sz_i)
  0: data_shift = {pak_index_plus[5:0]};    /* 1 bit field: shift 1-32 places */
  1: data_shift = {pak_index_plus[4:0], 1'h0};
  2: data_shift = {pak_index_plus[3:0], 2'h0};
  3: data_shift = {pak_index_plus[2:0], 3'h0};
  4: data_shift = {pak_index_plus[1:0], 4'h0};
  5: data_shift = 6'h20;
  default: data_shift = 6'h00;                            /* Not a legal code */
endcase

assign temp_data = (pak_acc_data_i[31:0] & data_mask) << data_shift;

assign new_data   = temp_data[63:32];
assign pot_data_o = output_buffer;

always @ (posedge clk)
if (reset || writing) output_buffer <= 32'h0000_0000;
else                         /* OR in data - easier than separate enables (?) */
if (pak_write_i) output_buffer     <= output_buffer | new_data;

assign index_shift  = (5 - pak_out_sz_i);   /* Complementary shift for offset */

assign pak_colour_sel_o = pak_colour_i;

always @ (posedge clk)
if (reset)
  begin
     pak_index_latched <= {`PIN_BITS{1'b0}};
     pak_colour_bs_o   <= 1'b0;
     last              <= 1'b0;
  end
else if (pak_write_i)
  begin
     pak_index_latched <= pak_index_i;
     pak_colour_bs_o   <= pak_colour_i;		// FIX timing skews *** @@@ ***
     last              <= pak_last_i;
  end

assign offset = pak_index_latched >> index_shift;      /* Held for output time */
assign pot_addr_o = pak_out_base_addr_i + {{(`ADDR_SIZE - `PIN_BITS){1'b0}}, offset};

always @ (posedge clk)
if (reset || writing)                                      /* Init. or output */
  begin
  buffer_valid <= 1'b0;
  buffer_full  <= 1'b0;
  end
else
  if (pak_write_i)                                                   /* Input */
    begin
    buffer_valid <= 1'b1;
    buffer_full  <= top_bit || pak_last_i;
    end

assign pak_full_o = buffer_full;
assign busy_o     = buffer_valid;
assign finish_o   = writing && last;        /* Global output status indicator */

endmodule

/*----------------------------------------------------------------------------*/
