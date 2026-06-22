// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
`include "../shared/constants.v"

module syn_curr_update
                        # (
                           parameter X_OUTPUT_SZ        = 8,
                           parameter Y_OUTPUT_SZ        = 8,
                           parameter IN_DATA_BITS       = 32,
                           parameter WEIGHT_IDX_SZ      = 16, // MC: widened 5 -> 16
			   parameter WEIGHT_SLICE_SZ    = 5,
                           parameter WEIGHT_DATA_IDX_SZ = 5, // 2^5 =  32-bit
                           parameter SPARSE_IDX_SZ      = 16,
                           parameter COUT_SZ            = 7, // MC: output-channel width
                           parameter MEM_ADDR_BITS      = `ADDR_SIZE)
   (
    input  wire                          clk,
    input  wire                          reset,

    // Interface to control signals:
    input  wire                          running_i,
    input  wire                          finished_pass_weight_i,
    output wire                          finished_pass_o,
    output wire                          syn_curr_update_running_o,

    input  wire                    [1:0] weight_mode_i,
    input  wire   [SPARSE_IDX_SZ-1:0]    sparse_index_i,

    input  wire         [`ADDR_SIZE-1:0] syn_curr_base_addr_i,
    input  wire     [X_OUTPUT_SZ-1:0]    out_x_len_i,
    input  wire     [Y_OUTPUT_SZ-1:0]    out_y_len_i,         // MC: needed for cout stride

    // Input weight information:
    input  wire                          weight_index_valid_i,
    input  wire   [WEIGHT_IDX_SZ-1:0]    weight_index_i,
    input  wire     [X_OUTPUT_SZ-1:0]    weight_index_x_i,
    input  wire     [Y_OUTPUT_SZ-1:0]    weight_index_y_i,
    input  wire          [COUT_SZ-1:0]   weight_index_cout_i, // MC: output channel
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
    output wire          [`WTD_BITS-1:0] syn_curr_mem_data_o
);

localparam WEIGHT_BITS     = 2**WEIGHT_SLICE_SZ;

reg               [WEIGHT_IDX_SZ-1:0]    weight_index_r;
wire              [WEIGHT_IDX_SZ-1:0]    weight_index;
reg                                      req_pending_r;
reg                     [`ADDR_SIZE-1:0] syn_curr_addr_r;
wire                  [IN_DATA_BITS-1:0] aligned_weight_value;
reg                                      syn_curr_update_running_r;
wire                                     syn_curr_update_running_nxt;
reg                     [`ADDR_SIZE-1:0] syn_curr_flat_index_r;

always @ (posedge clk)
if (reset)
    syn_curr_update_running_r <= 1'b0;
else 
    syn_curr_update_running_r <= syn_curr_update_running_nxt;

// Stop conditions take PRIORITY over the restart term: a finished_pass_weight_i
// pulse can arrive while running_r is low (the weight pass was terminated by a
// gated-out last activation — act_last_dumped — one cycle after this block
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
//   - full / conv mode:    base + cout*out_x*out_y + y*out_x + x   (channel-major)
// Backwards-compatible: cout_len = 1 means weight_index_cout_i is 0, so the
// channel term vanishes and the formula collapses to the legacy base + y*out_x + x.
wire is_sparse_mode = (weight_mode_i == 2'b01);
wire [`ADDR_SIZE-1:0] syn_curr_flat_index_xy;
wire [`ADDR_SIZE-1:0] syn_curr_flat_index;
assign syn_curr_flat_index_xy = ({{(`ADDR_SIZE-COUT_SZ){1'b0}}, weight_index_cout_i})
                              * ({{(`ADDR_SIZE-Y_OUTPUT_SZ){1'b0}}, out_y_len_i})
                              * ({{(`ADDR_SIZE-X_OUTPUT_SZ){1'b0}}, out_x_len_i})
                              + ({{(`ADDR_SIZE-Y_OUTPUT_SZ){1'b0}}, weight_index_y_i})
                              * ({{(`ADDR_SIZE-X_OUTPUT_SZ){1'b0}}, out_x_len_i})
                              + ({{(`ADDR_SIZE-X_OUTPUT_SZ){1'b0}}, weight_index_x_i});
assign syn_curr_flat_index    = is_sparse_mode
                              ? {{(`ADDR_SIZE-SPARSE_IDX_SZ){1'b0}}, sparse_index_i}
                              : syn_curr_flat_index_xy;

always @ (posedge clk)
begin
   if (reset)
   begin
      weight_index_r  <=  'hDEAD;
      syn_curr_addr_r <=  'hABABAB;
      syn_curr_flat_index_r <= 'h0;
      req_pending_r   <= 1'b0;
   end
   else if (weight_index_valid_i & weight_value_valid_i & ~req_pending_r & ~syn_curr_mem_wait_i)
   begin
	   weight_index_r  <= weight_index_i;
	   req_pending_r   <= 1'b1;
           syn_curr_addr_r <= weight_index_i;
           syn_curr_flat_index_r <= syn_curr_flat_index;
`ifdef CONV_DEBUG
           $display("[%0d] syn_wr: cout=%0d x=%0d y=%0d val=%0d -> flat=%0d",
                    $time, weight_index_cout_i, weight_index_x_i, weight_index_y_i,
                    $signed(weight_value_i), syn_curr_flat_index);
`endif
   end
   else if (req_pending_r & ~syn_curr_mem_wait_i)
	   req_pending_r  <= 1'b0;
end

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

assign aligned_weight_value = {{(IN_DATA_BITS-WEIGHT_BITS){weight_value_i[WEIGHT_BITS-1]}}, weight_value_i[WEIGHT_BITS-1:0]};

// syn_curr always accumulates from memory.  To start a buffer from zero, issue
// a FILL(value=0) task on the syn_curr buffer before use (the old in-accelerator
// clear_syn_curr first-write tracking has been removed to save fabric area).
wire [`WTD_BITS-1:0] base_syn_curr = syn_curr_mem_data_i;

assign syn_curr_mem_data_o = base_syn_curr + aligned_weight_value;

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

