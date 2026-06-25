// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
`include "../shared/constants.v"

module ann_syn_curr_update
                        # (
                           parameter X_OUTPUT_SZ        = 8,
                           parameter Y_OUTPUT_SZ        = 8,
                           parameter IN_DATA_BITS       = 32,
                           parameter ACT_VAL_BITS       = IN_DATA_BITS, // activation operand width into the MAC
                           parameter WEIGHT_IDX_SZ      = 5, // 2^5 =  32-bit
			   parameter WEIGHT_SLICE_SZ    = 5,
                           parameter WEIGHT_DATA_IDX_SZ = 5, // 2^5 =  32-bit
                           parameter SPARSE_IDX_SZ      = 16,
                           parameter MEM_ADDR_BITS      = `ADDR_SIZE)
   (
    input  wire                          clk,
    input  wire                          reset,

    // Interface to control signals:
    input  wire                          start_new_block_i,
    input  wire                          running_i,
    input  wire                          finished_pass_weight_i,
    output wire                          finished_pass_o,
    output wire                          syn_curr_update_running_o,

    input  wire                    [1:0] weight_mode_i,
    input  wire   [SPARSE_IDX_SZ-1:0]    sparse_index_i,

    input  wire         [`ADDR_SIZE-1:0] syn_curr_base_addr_i,
    input  wire     [X_OUTPUT_SZ-1:0]    out_x_len_i,

    // Input weight information:
    input  wire                          weight_index_valid_i,
    input  wire   [WEIGHT_IDX_SZ-1:0]    weight_index_i,
    input  wire     [X_OUTPUT_SZ-1:0]    weight_index_x_i,
    input  wire     [Y_OUTPUT_SZ-1:0]    weight_index_y_i,
    input  wire                          weight_index_last_i,
    output wire                          weight_index_taken_o,
    input  wire                          weight_value_valid_i,
    input  wire [2**WEIGHT_SLICE_SZ-1:0] weight_value_i,
    output wire                          weight_value_taken_o,

    // Addressing for synaptic current memory:
    output wire                          syn_curr_mem_rd_o,
    output wire                          syn_curr_mem_wr_o,
    input  wire                          syn_curr_mem_wait_i,
    output wire         [`ADDR_SIZE-1:0] syn_curr_mem_addr_o,
    input  wire          [`WTD_BITS-1:0] syn_curr_mem_data_i,
    output wire          [`WTD_BITS-1:0] syn_curr_mem_data_o,

    // Right-justified input activation. Default: unsigned (zero-extended) feature
    // activation. When act_signed_i=1 it is a sign-extended signed value, so the
    // signed GRU state h / candidate n (tanh in [-1,1]) can feed the MAC (§5.4).
    input  wire          [IN_DATA_BITS-1:0] act_value_i,
    input  wire                             act_signed_i
);

localparam WEIGHT_BITS = 2**WEIGHT_SLICE_SZ;

reg               [WEIGHT_IDX_SZ-1:0]    weight_index_r;
wire              [WEIGHT_IDX_SZ-1:0]    weight_index;
reg                                      req_pending_r;
reg                     [`ADDR_SIZE-1:0] syn_curr_addr_r;
wire                  [IN_DATA_BITS-1:0] aligned_weight_value;
reg                                      syn_curr_update_running_r;
reg              [WEIGHT_BITS-1:0] weight_value_r;
reg             [IN_DATA_BITS-1:0] act_value_r;
wire                                     syn_curr_update_running_nxt;

// Accumulator read-return latch (multi-annAcc pool-contention fix).
// The shared pool presents a requester's read data for EXACTLY ONE cycle after
// the read is granted; a write does not refresh it. The accumulator write-back
// below is combinational off syn_curr_mem_data_i, so if the write-back stalls on
// pool back-pressure (syn_curr_mem_wait_i) and another requester reads the same
// bank in the gap, the live bus is clobbered and a wrong accumulator value would
// be committed. Latch the read value the cycle after the read grant (== the first
// write-back cycle) and use the latched copy on any stalled retry.
reg              [`WTD_BITS-1:0]   acc_read_r;
reg                                acc_read_valid_r;

always @ (posedge clk)
if (reset)
    syn_curr_update_running_r <= 1'b0;
else 
    syn_curr_update_running_r <= syn_curr_update_running_nxt;

// Stop conditions take PRIORITY over the restart term: a finished_pass_weight_i
// pulse can arrive while running_r is low (the weight pass was terminated by a
// gated-out last activation — act_last_dumped_i — one cycle after this block
// stopped on the previous input's last weight index). With the restart term
// first, that pulse was swallowed, this block restarted, and no stop condition
// ever fired again (snn0 e2e hang, 2026-06-10).
assign syn_curr_update_running_nxt = (((weight_index_valid_i &
				        weight_index_last_i  &
				        weight_index_taken_o) |
				       finished_pass_weight_i)    ? 1'b0 :
	                             (running_i &
	                              ~syn_curr_update_running_r) ? 1'b1 :
	                              syn_curr_update_running_r);
                  
assign syn_curr_update_running_o = syn_curr_update_running_r;

assign finished_pass_o = syn_curr_update_running_r & ~syn_curr_update_running_nxt;

//////////////////////////////////////////////////////////////////////////////
// Fetch logic for syn_current value

// Compute flat output-neuron index for syn_curr addressing.
//   - sparse mode (2'b01): use sparse_index_i directly (the unpacked tuple index)
//   - full / conv mode:    base + y * out_x_len + x
wire is_sparse_mode = (weight_mode_i == 2'b01);
wire [`ADDR_SIZE-1:0] syn_curr_flat_index_xy;
wire [`ADDR_SIZE-1:0] syn_curr_flat_index;
assign syn_curr_flat_index_xy = ({{(`ADDR_SIZE-Y_OUTPUT_SZ){1'b0}}, weight_index_y_i})
                              * ({{(`ADDR_SIZE-X_OUTPUT_SZ){1'b0}}, out_x_len_i})
                              + ({{(`ADDR_SIZE-X_OUTPUT_SZ){1'b0}}, weight_index_x_i});
assign syn_curr_flat_index    = is_sparse_mode
                              ? {{(`ADDR_SIZE-SPARSE_IDX_SZ){1'b0}}, sparse_index_i}
                              : syn_curr_flat_index_xy;

reg [`ADDR_SIZE-1:0] syn_curr_flat_index_r;

always @ (posedge clk)
begin
   if (reset)
   begin
      weight_index_r  <=  'hDEAD;
      syn_curr_addr_r <=  'hABABAB;
      syn_curr_flat_index_r <= 'h0;
      req_pending_r   <= 1'b0;
      weight_value_r  <= 'h0;
      act_value_r     <= 'h0;
   end
   else if (weight_index_valid_i & weight_value_valid_i & ~req_pending_r & ~syn_curr_mem_wait_i)
   begin
	   weight_index_r  <= weight_index_i;
	   req_pending_r   <= 1'b1;
           syn_curr_addr_r <= weight_index_i;
           syn_curr_flat_index_r <= syn_curr_flat_index;
           weight_value_r  <= weight_value_i;
           act_value_r     <= act_value_i;
   end
   else if (req_pending_r & ~syn_curr_mem_wait_i)
	   req_pending_r  <= 1'b0;
end

// Latch the read-return one cycle after the read is granted. req_pending_r goes
// high the cycle the read is accepted, so the cycle it is FIRST high is exactly
// when the pool presents valid read data; sample it then. acc_read_valid_r lags
// req_pending_r by one cycle, so it is low on that first write-back cycle (use
// the live bus, bit-identical to before) and high on any stalled retry (use the
// latched copy, immune to a same-bank clobber).
always @ (posedge clk)
begin
   if (reset)
   begin
      acc_read_r       <= {`WTD_BITS{1'b0}};
      acc_read_valid_r <= 1'b0;
   end
   else
   begin
      acc_read_valid_r <= req_pending_r;
      if (req_pending_r & ~acc_read_valid_r)
         acc_read_r <= syn_curr_mem_data_i;
   end
end

wire [`WTD_BITS-1:0] acc_read_value = acc_read_valid_r ? acc_read_r
                                                       : syn_curr_mem_data_i;

// Select cached index if we have one, else the incoming index.
// The address is computed from the projected output (x,y) so that
// each output neuron's syn_curr accumulates contributions from every
// connected input neuron:
assign weight_index = (req_pending_r)? weight_index_r : weight_index_i;

assign syn_curr_mem_addr_o = syn_curr_base_addr_i +
                             (req_pending_r ? syn_curr_flat_index_r
                                            : syn_curr_flat_index);

assign syn_curr_mem_rd_o = (syn_curr_mem_wr_o)    ? 1'b0 :
	                   (weight_index_valid_i) ? 1'b1 :
			                            1'b0 ;

//////////////////////////////////////////////////////////////////////////////
// Update syn_curr value using given weight
//

assign aligned_weight_value = {{(IN_DATA_BITS-WEIGHT_BITS){weight_value_r[WEIGHT_BITS-1]}}, weight_value_r[WEIGHT_BITS-1:0]};

// MAC: accumulate (signed weight) * activation into syn_curr.
//   act_signed_i=0 : activation is unsigned magnitude (zero-extended) — legacy,
//                    bit-identical to the original $signed({1'b0, act_value_r}) form.
//   act_signed_i=1 : activation is signed (sign-extended from its slice MSB in
//                    spike_processing), so the signed GRU state can feed the
//                    W_i.h / W_h.h MACs. Symmetric with the signed-weight path. §5.4
wire signed [IN_DATA_BITS+ACT_VAL_BITS:0] mac_product;
wire signed [ACT_VAL_BITS:0]              act_operand =
        act_signed_i ? {act_value_r[ACT_VAL_BITS-1], act_value_r}   // sign-extend
                     : {1'b0,                         act_value_r}; // zero-extend
assign mac_product = $signed(aligned_weight_value) * act_operand;
assign syn_curr_mem_data_o = acc_read_value + mac_product[IN_DATA_BITS-1:0];

//////////////////////////////////////////////////////////////////////////////
// Writeback syn_curr value to memory
//

assign syn_curr_mem_wr_o = (req_pending_r) ? 1'b1 : 1'b0;

//////////////////////////////////////////////////////////////////////////////
// Acknowedge both weight index and weight value
// Must be able to write the syn_curr value back first
//

assign weight_index_taken_o = syn_curr_mem_wr_o & ~syn_curr_mem_wait_i;
assign weight_value_taken_o = syn_curr_mem_wr_o & ~syn_curr_mem_wait_i;

endmodule

