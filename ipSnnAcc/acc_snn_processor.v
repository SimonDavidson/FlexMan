// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
/////////////////////////////////////////////////////////////////////
//
// acc_snn_processor
//
// Top-level wrapper that instantiates spike_processing and
// neuron_processing as a pipeline.
//
// The synaptic current memory port is shared between the two
// sub-modules and is arbitrated here.  neuron_processing has fixed
// higher priority.  The grant is cycle-by-cycle (no burst lock).
//
// All other memory interfaces are exposed directly as ports.
// The AXI config bus and scheduler interfaces are exposed as
// separate port groups, one per sub-module.
//
/////////////////////////////////////////////////////////////////////

`include "../shared/constants.v"

module acc_snn_processor # (

    parameter TGT_ACC_ID              = 'h0,
    parameter TGT_CONFIG_BASE_ADDR    = 32'hFFFFFFFF,
    //----------------------------------------------------------------
    // Parameters forwarded to spike_processing
    //----------------------------------------------------------------
    parameter SP_NUM_TIMESTEPS        = 32,
    parameter SP_X_INPUT_SZ           = 8,
    parameter SP_Y_INPUT_SZ           = 8,
    parameter SP_X_OUTPUT_SZ          = 8,
    parameter SP_Y_OUTPUT_SZ          = 8,
    parameter SP_X_KERNEL_SZ          = 3,
    parameter SP_Y_KERNEL_SZ          = 3,
    parameter SP_X_KERNEL_OFF_SZ      = 3,
    parameter SP_Y_KERNEL_OFF_SZ      = 3,
    parameter SP_X_STEP_SZ            = 3,
    parameter SP_Y_STEP_SZ            = 3,
    parameter SP_ELEMS_PER_ROW        = 4,
    parameter SP_ROWS_PER_NEURON      = 16,
    parameter SP_TIMESTEP_SZ          = 10,
    parameter SP_IN_DATA_BITS         = 32,
    parameter SP_ELEM_SZ              = 8,
    parameter SP_ACT_SLICE_SZ         = 3,
    parameter SP_ACT_DATA_IDX_SZ      = 5,
    parameter SP_WEIGHT_ENTRY_BITS    = 8,
    parameter SP_WEIGHT_IDX_SZ        = 5,
    parameter SP_WEIGHT_SLICE_SZ      = 5, // 32-bit slice port (must hold a full sparse tuple)
    parameter SP_WEIGHT_DATA_IDX_SZ   = 2, // 2-bit slice index (32-bit words)
    parameter SP_SYN_CURR_IDX_SZ      = 10,
    parameter SP_SYN_CURR_DATA_IDX_SZ = 5,
    parameter SP_SYN_CURR_SLICE_SZ    = 3, // 8-bit data
    parameter SP_SYN_CURR_SLICE_BITS  = 10,
    parameter SP_BIAS_CURR_IDX_SZ     = 2, // 2-bit slice index (32-bit words)
    parameter SP_BIAS_CURR_DATA_IDX_SZ= 5,
    parameter SP_BIAS_CURR_SLICE_SZ   = 3,
    parameter SP_BIAS_CURR_SLICE_BITS = 8,

    //----------------------------------------------------------------
    // Parameters forwarded to neuron_processing
    //----------------------------------------------------------------
    parameter NP_NUM_TIMESTEPS        = 32,
    parameter NP_TIMESTEP_SZ          = 10,
    parameter NP_IN_DATA_BITS         = 32,
    parameter NP_NEURON_IDX_SZ        = 10,
    parameter NP_SYN_CURR_IDX_SZ      = 10,
    parameter NP_SYN_CURR_DATA_IDX_SZ = 5,
    parameter NP_SYN_CURR_SLICE_SZ    = 3,
    parameter NP_SYN_CURR_SLICE_BITS  = 10,
    parameter NP_BIAS_CURR_IDX_SZ     = 2,
    parameter NP_BIAS_CURR_DATA_IDX_SZ= 5,
    parameter NP_BIAS_CURR_SLICE_SZ   = 3,
    parameter NP_BIAS_CURR_SLICE_BITS = 8,
    parameter NP_POT_IDX_SZ           = 2,
    parameter NP_POT_DATA_IDX_SZ      = 5,
    parameter NP_POT_SLICE_SZ         = 3,
    parameter NP_POT_SLICE_BITS       = 8,
    parameter NP_SPIKE_IDX_SZ         = 2,
    parameter NP_SPIKE_DATA_IDX_SZ    = 5,
    parameter NP_SPIKE_SLICE_SZ       = 3,
    parameter NP_SPIKE_SLICE_BITS     = 8,
    parameter NP_SYN_DECAY_BITS       = 32,
    parameter NP_POT_DECAY_BITS       = 32,
    parameter MEM_ADDR_BITS           = `ADDR_SIZE
)(
    input  wire clk,
    input  wire reset,

    //================================================================
    // AXI config interface
    //================================================================
    input  wire                    sys_req_i,
    output wire                    sys_ack_o,
    input  wire             [31:0] sys_addr_i,
    input  wire             [31:0] sys_data_i,

    //================================================================
    // Scheduler interface 
    //================================================================
    input  wire                      start_new_block_i,
    input  wire   [`TGT_ACC_SZ-1:0]  target_acc_i,
    input  wire [`SCH_ENTRY_SZ-1:0]  buffer_info_i,
    output wire                      spike_proc_finished_o,
    output wire                      acc_busy_o,
    output wire                      acc_finished_o,

    //================================================================
    // Buffer-address interface – spike_processing
    //================================================================
    input  wire [`PIN_BITS-1:0] sp_src1_buff_addr_i,
    input  wire [`PIN_BITS-1:0] sp_src2_buff_addr_i,
    input  wire [`PIN_BITS-1:0] sp_src3_buff_addr_i,
    input  wire [`PIN_BITS-1:0] sp_tgt_buff_addr_i,
    input  wire [`PIN_BITS-1:0] sp_weight_row_len_i,

    //================================================================
    // Buffer-address interface – neuron_processing
    //================================================================
    input  wire [`PIN_BITS-1:0] np_src1_buff_addr_i,
    input  wire [`PIN_BITS-1:0] np_src2_buff_addr_i,
    input  wire [`PIN_BITS-1:0] np_src3_buff_addr_i,
    input  wire [`PIN_BITS-1:0] np_tgt_buff_addr_i,
    input  wire [`PIN_BITS-1:0] np_weight_row_len_i,

    //================================================================
    // External memory – weight (spike_processing only)
    //================================================================
    output wire                  weight_mem_rd_o,
    input  wire                  weight_mem_wait_i,
    output wire [`ADDR_SIZE-1:0] weight_mem_addr_o,
    input  wire  [`WTD_BITS-1:0] weight_mem_data_i,

    //================================================================
    // External memory – input activations (spike_processing only)
    //================================================================
    output wire                  act_mem_req_o,
    input  wire                  act_mem_wait_i,
    output wire [`ADDR_SIZE-1:0] act_mem_addr_o,
    input  wire  [`ACT_BITS-1:0] act_mem_data_i,

    //================================================================
    // External memory – synaptic currents (SHARED, arbitrated)
    //================================================================
    output wire                  syn_curr_mem_wr_o,
    output wire                  syn_curr_mem_rd_o,
    input  wire                  syn_curr_mem_wait_i,
    output wire [`ADDR_SIZE-1:0] syn_curr_mem_addr_o,
    output wire  [`POT_BITS-1:0] syn_curr_mem_data_o,   // write data
    input  wire  [`POT_BITS-1:0] syn_curr_mem_data_i,   // read data (shared)

    //================================================================
    // External memory – bias currents (neuron_processing only)
    //================================================================
    output wire                  bias_curr_mem_rd_o,
    input  wire                  bias_curr_mem_wait_i,
    output wire [`ADDR_SIZE-1:0] bias_curr_mem_addr_o,
    input  wire  [`WTD_BITS-1:0] bias_curr_mem_data_i,

    //================================================================
    // External memory – threshold (neuron_processing only)
    //================================================================
    output wire                  thresh_mem_rd_o,
    input  wire                  thresh_mem_wait_i,
    output wire [`ADDR_SIZE-1:0] thresh_mem_addr_o,
    input  wire  [`WTD_BITS-1:0] thresh_mem_data_i,

    //================================================================
    // External memory – neuron potentials (neuron_processing only)
    //================================================================
    output wire                  pot_mem_wr_o,
    output wire                  pot_mem_rd_o,
    input  wire                  pot_mem_wait_i,
    output wire [`ADDR_SIZE-1:0] pot_mem_addr_o,
    output wire  [`POT_BITS-1:0] pot_mem_data_o,
    input  wire  [`POT_BITS-1:0] pot_mem_data_i,

    //================================================================
    // External memory – output spikes (neuron_processing only)
    //================================================================
    output wire                  spike_mem_wr_o,
    input  wire                  spike_mem_wait_i,
    output wire [`ADDR_SIZE-1:0] spike_mem_addr_o,
    output wire  [`ACT_BITS-1:0] spike_mem_data_o
);

    //----------------------------------------------------------------
    // spike_processing config registers
    //----------------------------------------------------------------
    reg      [MEM_ADDR_BITS-1:0] sp_act_base_addr_r;
    reg      [MEM_ADDR_BITS-1:0] sp_weight_base_addr_r;
    reg      [MEM_ADDR_BITS-1:0] syn_curr_base_addr_r;
    reg  [SP_WEIGHT_SLICE_SZ-1:0] sp_weight_sz_r;
    reg     [SP_TIMESTEP_SZ-1:0] sp_total_timesteps_r;
    reg   [SP_X_INPUT_SZ-1:0] sp_in_x_len_r;
    reg   [SP_Y_INPUT_SZ-1:0] sp_in_y_len_r;
    reg  [SP_X_OUTPUT_SZ-1:0] sp_out_x_len_r;
    reg  [SP_Y_OUTPUT_SZ-1:0] sp_out_y_len_r;
    reg  [SP_ELEMS_PER_ROW-1:0] sp_weights_per_word_r;
    reg [SP_ROWS_PER_NEURON-1:0] sp_rows_per_neuron_r;
    reg  [SP_WEIGHT_IDX_SZ-1:0] sp_weight_idx_sz_r;
    reg                    [1:0] sp_weight_mode_r;
    reg     [SP_X_KERNEL_SZ-1:0] sp_x_kernel_len_r;
    reg     [SP_Y_KERNEL_SZ-1:0] sp_y_kernel_len_r;
    reg [SP_X_KERNEL_OFF_SZ-1:0] sp_x_kernel_offset_r;
    reg [SP_Y_KERNEL_OFF_SZ-1:0] sp_y_kernel_offset_r;
    reg       [SP_X_STEP_SZ-1:0] sp_x_kernel_step_r;
    reg       [SP_Y_STEP_SZ-1:0] sp_y_kernel_step_r;
    reg  [SP_WEIGHT_SLICE_SZ-1:0] sp_index_sz_r;
    reg  [SP_WEIGHT_SLICE_SZ-1:0] sp_tuple_sz_r;
    reg          [`PIN_BITS-1:0]  sp_sparse_count_r;

    //----------------------------------------------------------------
    // neuron_processing config registers
    //----------------------------------------------------------------
    reg   [NP_NEURON_IDX_SZ-1:0] np_last_neuron_idx_r;
    reg      [MEM_ADDR_BITS-1:0] np_bias_curr_base_addr_r;
    reg      [MEM_ADDR_BITS-1:0] np_thresh_base_addr_r;
    reg      [MEM_ADDR_BITS-1:0] np_pot_base_addr_r;
    reg      [MEM_ADDR_BITS-1:0] np_spike_base_addr_r;
    reg [NP_SYN_CURR_SLICE_SZ-1:0] np_syn_curr_sz_r;
    reg [NP_BIAS_CURR_SLICE_SZ-1:0] np_bias_curr_sz_r;
    reg    [NP_POT_SLICE_SZ-1:0] np_pot_sz_r;
    reg                    [2:0] np_mode_r;             // [0]=sub_on_fire  [1]=clear_syn_curr  [2]=clear_pot
    reg                          sp_skip_neuron_r;      // 1 = skip neuron_processing after spike_processing

    //----------------------------------------------------------------
    // Shared config registers
    //----------------------------------------------------------------
    reg                    [4:0] bin_point_syn_curr_r;
    reg                   [31:0] np_syn_curr_decay_mult_r;
    reg                   [31:0] np_pot_decay_mult_r;

    //================================================================
    // AXI config register decode
    //
    // Address match: sys_addr_i[31:16] must equal TGT_CONFIG_BASE_ADDR[31:16].
    // Register select: sys_addr_i[7:0] (word-aligned offsets below).
    // sys_ack_o is asserted combinationally on the same cycle as sys_req_i.
    //
    // spike_processing registers:
    //   8'h00  sp_act_base_addr_r
    //   8'h04  sp_weight_base_addr_r
    //   8'h08  syn_curr_base_addr_r        (shared with neuron_processing)
    //   8'h0C  sp_weight_sz_r
    //   8'h14  sp_total_timesteps_r
    //   8'h44  sp_in_x_len_r
    //   8'h48  sp_in_y_len_r
    //   8'h4C  sp_out_x_len_r
    //   8'h50  sp_out_y_len_r
    //   8'h54  sp_weights_per_word_r
    //   8'h58  sp_rows_per_neuron_r
    //   8'h5C  sp_weight_idx_sz_r
    //   8'h70  sp_weight_mode_r
    //   8'h74  sp_x_kernel_len_r
    //   8'h78  sp_y_kernel_len_r
    //   8'h7C  sp_x_kernel_step_r
    //   8'h80  sp_y_kernel_step_r
    //   8'h84  sp_x_kernel_offset_r
    //   8'h88  sp_y_kernel_offset_r
    //   8'h8C  sp_index_sz_r
    //   8'h90  sp_tuple_sz_r
    //   8'h94  sp_sparse_count_r
    //
    // neuron_processing registers:
    //   8'h20  np_last_neuron_idx_r
    //   8'h28  np_bias_curr_base_addr_r
    //   8'h2C  np_thresh_base_addr_r
    //   8'h30  np_pot_base_addr_r
    //   8'h34  np_syn_curr_sz_r
    //   8'h38  np_bias_curr_sz_r
    //   8'h3C  np_pot_sz_r
    //   8'h64  np_spike_base_addr_r
    //
    // shared registers:
    //   8'h40  bin_point_syn_curr_r
    //   8'h68  np_syn_curr_decay_mult_r
    //   8'h6C  np_pot_decay_mult_r
    //   8'h98  np_mode_r              [2:0]  sub_on_fire / clear_syn_curr / clear_pot
    //   8'h9C  sp_skip_neuron_r       [0]    1 = skip neuron_processing after spike_processing
    //
    // per-task control word (lives inside the cfg_mem push window):
    //   8'h10  task_ctrl              [0]    sp_skip_neuron
    //                                 [3:1]  np_mode
    //                                 [6:4]  reserved (sp_act_sz, future)
    //                                 [8:7]  reserved (np_thresh_op on annAcc, future)
    //          Writes here drive the same flops as 0x98/0x9C — last write
    //          wins. config_manager pushes cfg_mem word 4 here on every
    //          TASK dispatch, so per-cfg_id rotation works automatically.
    //================================================================
    wire addr_match = (sys_addr_i[31:16] == TGT_CONFIG_BASE_ADDR[31:16]);

    assign sys_ack_o = sys_req_i & addr_match;

    always @ (posedge clk) begin
        if (reset) begin
            sp_act_base_addr_r      <= {MEM_ADDR_BITS{1'b0}};
            sp_weight_base_addr_r   <= {MEM_ADDR_BITS{1'b0}};
            syn_curr_base_addr_r    <= {MEM_ADDR_BITS{1'b0}};
            sp_weight_sz_r          <= {SP_WEIGHT_SLICE_SZ{1'b0}};
            sp_total_timesteps_r    <= {SP_TIMESTEP_SZ{1'b0}};
            sp_in_x_len_r           <= {SP_X_INPUT_SZ{1'b0}};
            sp_in_y_len_r           <= {SP_Y_INPUT_SZ{1'b0}};
            sp_out_x_len_r          <= {SP_X_OUTPUT_SZ{1'b0}};
            sp_out_y_len_r          <= {SP_Y_OUTPUT_SZ{1'b0}};
            sp_weights_per_word_r   <= {SP_ELEMS_PER_ROW{1'b0}};
            sp_rows_per_neuron_r    <= {SP_ROWS_PER_NEURON{1'b0}};
            sp_weight_idx_sz_r      <= {SP_WEIGHT_IDX_SZ{1'b0}};
            sp_weight_mode_r        <= 2'b0;
            sp_x_kernel_len_r       <= {SP_X_KERNEL_SZ{1'b0}};
            sp_y_kernel_len_r       <= {SP_Y_KERNEL_SZ{1'b0}};
            sp_x_kernel_offset_r    <= {SP_X_KERNEL_OFF_SZ{1'b0}};
            sp_y_kernel_offset_r    <= {SP_Y_KERNEL_OFF_SZ{1'b0}};
            sp_x_kernel_step_r      <= {SP_X_STEP_SZ{1'b0}};
            sp_y_kernel_step_r      <= {SP_Y_STEP_SZ{1'b0}};
            sp_index_sz_r           <= {SP_WEIGHT_SLICE_SZ{1'b0}};
            sp_tuple_sz_r           <= {SP_WEIGHT_SLICE_SZ{1'b0}};
            sp_sparse_count_r       <= {`PIN_BITS{1'b0}};
            np_last_neuron_idx_r        <= {NP_NEURON_IDX_SZ{1'b0}};
            np_bias_curr_base_addr_r<= {MEM_ADDR_BITS{1'b0}};
            np_thresh_base_addr_r   <= {MEM_ADDR_BITS{1'b0}};
            np_pot_base_addr_r      <= {MEM_ADDR_BITS{1'b0}};
            np_spike_base_addr_r    <= {MEM_ADDR_BITS{1'b0}};
            np_syn_curr_sz_r        <= {NP_SYN_CURR_SLICE_SZ{1'b0}};
            np_bias_curr_sz_r       <= {NP_BIAS_CURR_SLICE_SZ{1'b0}};
            np_pot_sz_r             <= {NP_POT_SLICE_SZ{1'b0}};
            np_mode_r               <= 3'b0;
            sp_skip_neuron_r        <= 1'b0;
            bin_point_syn_curr_r    <= 5'b0;
            np_syn_curr_decay_mult_r <= 32'b0;
            np_pot_decay_mult_r      <= 32'b0;
        end else if (sys_req_i & addr_match) begin
            case (sys_addr_i[7:0])
                8'h00: sp_act_base_addr_r      <= sys_data_i[MEM_ADDR_BITS-1:0];
                8'h04: sp_weight_base_addr_r   <= sys_data_i[MEM_ADDR_BITS-1:0];
                8'h08: syn_curr_base_addr_r    <= sys_data_i[MEM_ADDR_BITS-1:0];
                8'h0C: sp_weight_sz_r          <= sys_data_i[SP_WEIGHT_SLICE_SZ-1:0];
                8'h14: sp_total_timesteps_r    <= sys_data_i[SP_TIMESTEP_SZ-1:0];
                8'h44: sp_in_x_len_r           <= sys_data_i[SP_X_INPUT_SZ-1:0];
                8'h48: sp_in_y_len_r           <= sys_data_i[SP_Y_INPUT_SZ-1:0];
                8'h4C: sp_out_x_len_r          <= sys_data_i[SP_X_OUTPUT_SZ-1:0];
                8'h50: sp_out_y_len_r          <= sys_data_i[SP_Y_OUTPUT_SZ-1:0];
                8'h54: sp_weights_per_word_r   <= sys_data_i[SP_ELEMS_PER_ROW-1:0];
                8'h58: sp_rows_per_neuron_r    <= sys_data_i[SP_ROWS_PER_NEURON-1:0];
                8'h5C: sp_weight_idx_sz_r      <= sys_data_i[SP_WEIGHT_IDX_SZ-1:0];
                8'h70: sp_weight_mode_r        <= sys_data_i[1:0];
                8'h74: sp_x_kernel_len_r       <= sys_data_i[SP_X_KERNEL_SZ-1:0];
                8'h78: sp_y_kernel_len_r       <= sys_data_i[SP_Y_KERNEL_SZ-1:0];
                8'h7C: sp_x_kernel_step_r      <= sys_data_i[SP_X_STEP_SZ-1:0];
                8'h80: sp_y_kernel_step_r      <= sys_data_i[SP_Y_STEP_SZ-1:0];
                8'h84: sp_x_kernel_offset_r    <= sys_data_i[SP_X_KERNEL_OFF_SZ-1:0];
                8'h88: sp_y_kernel_offset_r    <= sys_data_i[SP_Y_KERNEL_OFF_SZ-1:0];
                8'h8C: sp_index_sz_r           <= sys_data_i[SP_WEIGHT_SLICE_SZ-1:0];
                8'h90: sp_tuple_sz_r           <= sys_data_i[SP_WEIGHT_SLICE_SZ-1:0];
                8'h94: sp_sparse_count_r       <= sys_data_i[`PIN_BITS-1:0];
                8'h20: np_last_neuron_idx_r        <= sys_data_i[NP_NEURON_IDX_SZ-1:0];
                8'h28: np_bias_curr_base_addr_r<= sys_data_i[MEM_ADDR_BITS-1:0];
                8'h2C: np_thresh_base_addr_r   <= sys_data_i[MEM_ADDR_BITS-1:0];
                8'h30: np_pot_base_addr_r      <= sys_data_i[MEM_ADDR_BITS-1:0];
                8'h64: np_spike_base_addr_r    <= sys_data_i[MEM_ADDR_BITS-1:0];
                8'h34: np_syn_curr_sz_r        <= sys_data_i[NP_SYN_CURR_SLICE_SZ-1:0];
                8'h38: np_bias_curr_sz_r       <= sys_data_i[NP_BIAS_CURR_SLICE_SZ-1:0];
                8'h3C: np_pot_sz_r             <= sys_data_i[NP_POT_SLICE_SZ-1:0];
                8'h40: bin_point_syn_curr_r      <= sys_data_i[4:0];
                8'h68: np_syn_curr_decay_mult_r  <= sys_data_i[31:0];
                8'h6C: np_pot_decay_mult_r        <= sys_data_i[31:0];
                8'h98: np_mode_r                 <= sys_data_i[2:0];
                8'h9C: sp_skip_neuron_r          <= sys_data_i[0];
                8'h10: begin
                    sp_skip_neuron_r             <= sys_data_i[0];
                    np_mode_r                    <= sys_data_i[3:1];
                end
                default: ; // ignore unrecognised addresses
            endcase
        end
    end

    //----------------------------------------------------------------
    // Internal syn_curr wires – spike_processing side
    //----------------------------------------------------------------
    wire                  sp_syn_curr_mem_wr;
    wire                  sp_syn_curr_mem_rd;
    wire [`ADDR_SIZE-1:0] sp_syn_curr_mem_addr;
    wire  [`POT_BITS-1:0] sp_syn_curr_mem_data_wr;
    wire                  sp_syn_curr_mem_wait;   // fed back from arbiter

    //----------------------------------------------------------------
    // Internal syn_curr wires – neuron_processing side
    //----------------------------------------------------------------
    wire                  np_syn_curr_mem_wr;
    wire                  np_syn_curr_mem_rd;
    wire [`ADDR_SIZE-1:0] np_syn_curr_mem_addr;
    wire  [`POT_BITS-1:0] np_syn_curr_mem_data_wr;
    wire                  np_syn_curr_mem_wait;   // fed back from arbiter

    //================================================================
    // Synaptic-current memory arbiter
    //
    // neuron_processing has fixed higher priority.
    // Grant is cycle-by-cycle: re-evaluated every clock edge.
    // The "losing" module sees mem_wait asserted for as long as the
    // higher-priority module holds the bus.
    //
    // grant_np = 1 when neuron_processing is requesting (rd or wr).
    // grant_sp = 1 when spike_processing is requesting AND
    //            neuron_processing is not.
    //================================================================
    wire np_req = np_syn_curr_mem_rd | np_syn_curr_mem_wr;
    wire sp_req = sp_syn_curr_mem_rd | sp_syn_curr_mem_wr;

    wire grant_np = np_req;
    wire grant_sp = sp_req & ~np_req;

    // Shared bus driven by whichever module holds the grant.
    assign syn_curr_mem_wr_o   = grant_np ? np_syn_curr_mem_wr
                                          : (grant_sp ? sp_syn_curr_mem_wr  : 1'b0);
    assign syn_curr_mem_rd_o   = grant_np ? np_syn_curr_mem_rd
                                          : (grant_sp ? sp_syn_curr_mem_rd  : 1'b0);
    assign syn_curr_mem_addr_o = grant_np ? np_syn_curr_mem_addr
                                          : sp_syn_curr_mem_addr;
    assign syn_curr_mem_data_o = grant_np ? np_syn_curr_mem_data_wr
                                          : sp_syn_curr_mem_data_wr;

    // Wait feedback: a module must wait if it is not granted the bus,
    // OR if it is granted and the memory itself is asserting wait.
    assign np_syn_curr_mem_wait = (~grant_np & np_req) | (grant_np & syn_curr_mem_wait_i);
    assign sp_syn_curr_mem_wait = (~grant_sp & sp_req) | (grant_sp & syn_curr_mem_wait_i);

    //----------------------------------------------------------------
    // Internal scheduler wires
    //----------------------------------------------------------------
    wire sp_acc_busy;
    wire sp_acc_finished;     // triggers neuron_processing
    wire np_neuron_proc_finished;
    wire np_acc_busy;
    wire np_acc_finished;

    assign acc_busy_o     = sp_acc_busy | np_acc_busy;
    assign acc_finished_o = sp_skip_neuron_r ? sp_acc_finished : np_acc_finished;

    //----------------------------------------------------------------
    // Latched dispatch target_acc
    //----------------------------------------------------------------
    reg [`TGT_ACC_SZ-1:0] dispatched_target_acc_r;
    always @(posedge clk) begin
        if (reset)
            dispatched_target_acc_r <= TGT_ACC_ID;
        else if (start_new_block_i && (target_acc_i == TGT_ACC_ID))
            dispatched_target_acc_r <= target_acc_i;
    end
    wire my_dispatch = start_new_block_i & (target_acc_i == TGT_ACC_ID);

    //================================================================
    // spike_processing instantiation
    //================================================================
    spike_processing # (
        .NUM_TIMESTEPS        (SP_NUM_TIMESTEPS),
        .X_INPUT_SZ           (SP_X_INPUT_SZ),
        .Y_INPUT_SZ           (SP_Y_INPUT_SZ),
        .X_OUTPUT_SZ          (SP_X_OUTPUT_SZ),
        .Y_OUTPUT_SZ          (SP_Y_OUTPUT_SZ),
        .X_KERNEL_SZ          (SP_X_KERNEL_SZ),
        .Y_KERNEL_SZ          (SP_Y_KERNEL_SZ),
        .X_KERNEL_OFF_SZ      (SP_X_KERNEL_OFF_SZ),
        .Y_KERNEL_OFF_SZ      (SP_Y_KERNEL_OFF_SZ),
        .X_STEP_SZ            (SP_X_STEP_SZ),
        .Y_STEP_SZ            (SP_Y_STEP_SZ),
        .ELEMS_PER_ROW        (SP_ELEMS_PER_ROW),
        .ROWS_PER_NEURON      (SP_ROWS_PER_NEURON),
        .TIMESTEP_SZ          (SP_TIMESTEP_SZ),
        .IN_DATA_BITS         (SP_IN_DATA_BITS),
        .ELEM_SZ              (SP_ELEM_SZ),
        .ACT_SLICE_SZ         (SP_ACT_SLICE_SZ),
        .ACT_DATA_IDX_SZ      (SP_ACT_DATA_IDX_SZ),
        .WEIGHT_ENTRY_BITS    (SP_WEIGHT_ENTRY_BITS),
        .WEIGHT_IDX_SZ        (SP_WEIGHT_IDX_SZ),
        .WEIGHT_SLICE_SZ      (SP_WEIGHT_SLICE_SZ),
        .WEIGHT_DATA_IDX_SZ   (SP_WEIGHT_DATA_IDX_SZ),
        .SYN_CURR_IDX_SZ      (SP_SYN_CURR_IDX_SZ),
        .SYN_CURR_DATA_IDX_SZ (SP_SYN_CURR_DATA_IDX_SZ),
        .SYN_CURR_SLICE_SZ    (SP_SYN_CURR_SLICE_SZ),
        .SYN_CURR_SLICE_BITS  (SP_SYN_CURR_SLICE_BITS),
        .BIAS_CURR_IDX_SZ     (SP_BIAS_CURR_IDX_SZ),
        .BIAS_CURR_DATA_IDX_SZ(SP_BIAS_CURR_DATA_IDX_SZ),
        .BIAS_CURR_SLICE_SZ   (SP_BIAS_CURR_SLICE_SZ),
        .BIAS_CURR_SLICE_BITS (SP_BIAS_CURR_SLICE_BITS),
        .MEM_ADDR_BITS        (MEM_ADDR_BITS)
    ) u_spike_processing (
        .clk                    (clk),
        .reset                  (reset),

        // Config registers
        .act_base_addr_i        (sp_act_base_addr_r),
        .weight_base_addr_i     (sp_weight_base_addr_r),
        .syn_curr_base_addr_i   (syn_curr_base_addr_r),
        .weight_sz_i            (sp_weight_sz_r),
        .bin_point_syn_curr_i   (bin_point_syn_curr_r),
        .in_x_len_i             (sp_in_x_len_r),
        .in_y_len_i             (sp_in_y_len_r),
        .out_x_len_i            (sp_out_x_len_r),
        .out_y_len_i            (sp_out_y_len_r),
        .weights_per_word_i     (sp_weights_per_word_r),
        .rows_per_neuron_i      (sp_rows_per_neuron_r),
        .weight_idx_sz_i        (sp_weight_idx_sz_r),
        .weight_mode_i          (sp_weight_mode_r),
        .clear_syn_curr_i       (np_mode_r[1]),
        .x_kernel_len_i         (sp_x_kernel_len_r),
        .y_kernel_len_i         (sp_y_kernel_len_r),
        .x_kernel_offset_i      (sp_x_kernel_offset_r),
        .y_kernel_offset_i      (sp_y_kernel_offset_r),
        .x_kernel_step_i        (sp_x_kernel_step_r),
        .y_kernel_step_i        (sp_y_kernel_step_r),
        .index_sz_i             (sp_index_sz_r),
        .tuple_sz_i             (sp_tuple_sz_r),
        .sparse_count_i         (sp_sparse_count_r),

        // Scheduler — gated dispatch
        .start_new_block_i      (my_dispatch),
        .target_acc_i           (dispatched_target_acc_r),
        .buffer_info_i          (buffer_info_i),
        .spike_proc_finished_o  (spike_proc_finished_o),
        .acc_busy_o             (sp_acc_busy),
        .acc_finished_o         (sp_acc_finished),

        // Buffer addresses
        .src1_buff_addr_i       (sp_src1_buff_addr_i),
        .src2_buff_addr_i       (sp_src2_buff_addr_i),
        .src3_buff_addr_i       (sp_src3_buff_addr_i),
        .tgt_buff_addr_i        (sp_tgt_buff_addr_i),
        .weight_row_len_i       (sp_weight_row_len_i),

        // Weight memory (exclusive to spike_processing)
        .weight_mem_rd_o        (weight_mem_rd_o),
        .weight_mem_wait_i      (weight_mem_wait_i),
        .weight_mem_addr_o      (weight_mem_addr_o),
        .weight_mem_data_i      (weight_mem_data_i),

        // Activation memory (exclusive to spike_processing)
        .act_mem_req_o          (act_mem_req_o),
        .act_mem_wait_i         (act_mem_wait_i),
        .act_mem_addr_o         (act_mem_addr_o),
        .act_mem_data_i         (act_mem_data_i),

        // Synaptic current memory (via arbiter)
        .syn_curr_mem_wr_o      (sp_syn_curr_mem_wr),
        .syn_curr_mem_rd_o      (sp_syn_curr_mem_rd),
        .syn_curr_mem_wait_i    (sp_syn_curr_mem_wait),
        .syn_curr_mem_addr_o    (sp_syn_curr_mem_addr),
        .syn_curr_mem_data_wr_o (sp_syn_curr_mem_data_wr),
        .syn_curr_mem_data_rd_i (syn_curr_mem_data_i)
    );

    //================================================================
    // neuron_processing instantiation
    //================================================================
    neuron_processing # (
        .TGT_ACC_ID           (TGT_ACC_ID),
        .NUM_TIMESTEPS        (NP_NUM_TIMESTEPS),
        .TIMESTEP_SZ          (NP_TIMESTEP_SZ),
        .IN_DATA_BITS         (NP_IN_DATA_BITS),
        .NEURON_IDX_SZ        (NP_NEURON_IDX_SZ),
        .SYN_CURR_IDX_SZ      (NP_SYN_CURR_IDX_SZ),
        .SYN_CURR_DATA_IDX_SZ (NP_SYN_CURR_DATA_IDX_SZ),
        .SYN_CURR_SLICE_SZ    (NP_SYN_CURR_SLICE_SZ),
        .SYN_CURR_SLICE_BITS  (NP_SYN_CURR_SLICE_BITS),
        .BIAS_CURR_IDX_SZ     (NP_BIAS_CURR_IDX_SZ),
        .BIAS_CURR_DATA_IDX_SZ(NP_BIAS_CURR_DATA_IDX_SZ),
        .BIAS_CURR_SLICE_SZ   (NP_BIAS_CURR_SLICE_SZ),
        .BIAS_CURR_SLICE_BITS (NP_BIAS_CURR_SLICE_BITS),
        .POT_IDX_SZ           (NP_POT_IDX_SZ),
        .POT_DATA_IDX_SZ      (NP_POT_DATA_IDX_SZ),
        .POT_SLICE_SZ         (NP_POT_SLICE_SZ),
        .POT_SLICE_BITS       (NP_POT_SLICE_BITS),
        .SPIKE_IDX_SZ         (NP_SPIKE_IDX_SZ),
        .SPIKE_DATA_IDX_SZ    (NP_SPIKE_DATA_IDX_SZ),
        .SPIKE_SLICE_SZ       (NP_SPIKE_SLICE_SZ),
        .SPIKE_SLICE_BITS     (NP_SPIKE_SLICE_BITS),
        .SYN_DECAY_BITS       (NP_SYN_DECAY_BITS),
        .POT_DECAY_BITS       (NP_POT_DECAY_BITS),
        .MEM_ADDR_BITS        (MEM_ADDR_BITS)
    ) u_neuron_processing (
        .clk                    (clk),
        .reset                  (reset),

        // Config registers
        .last_neuron_idx_i      (np_last_neuron_idx_r),
        .syn_curr_base_addr_i   (syn_curr_base_addr_r),
        .bias_curr_base_addr_i  (np_bias_curr_base_addr_r),
        .thresh_base_addr_i     (np_thresh_base_addr_r),
        .pot_base_addr_i        (np_pot_base_addr_r),
        .spike_base_addr_i      (np_spike_base_addr_r),
        .syn_curr_sz_i          (np_syn_curr_sz_r),
        .bias_curr_sz_i         (np_bias_curr_sz_r),
        .pot_sz_i               (np_pot_sz_r),
        .bin_point_syn_curr_i   (bin_point_syn_curr_r),
        .syn_curr_decay_mult_i  (np_syn_curr_decay_mult_r[NP_SYN_DECAY_BITS-1:0]),
        .pot_decay_mult_i       (np_pot_decay_mult_r[NP_POT_DECAY_BITS-1:0]),
        .sub_on_fire_i          (np_mode_r[0]),
        .clear_pot_i            (np_mode_r[2]),

        // Scheduler – triggered by spike_processing completion (gated by sp_skip_neuron_r)
        .start_new_block_i      (sp_acc_finished & ~sp_skip_neuron_r),
        .target_acc_i           (dispatched_target_acc_r),
        .buffer_info_i          (buffer_info_i),
        .neuron_proc_finished_o (np_neuron_proc_finished),
        .acc_busy_o             (np_acc_busy),
        .acc_finished_o         (np_acc_finished),

        // Buffer addresses
        .src1_buff_addr_i       (np_src1_buff_addr_i),
        .src2_buff_addr_i       (np_src2_buff_addr_i),
        .src3_buff_addr_i       (np_src3_buff_addr_i),
        .tgt_buff_addr_i        (np_tgt_buff_addr_i),
        .weight_row_len_i       (np_weight_row_len_i),

        // Synaptic current memory (via arbiter, high priority)
        .syn_curr_mem_wr_o      (np_syn_curr_mem_wr),
        .syn_curr_mem_rd_o      (np_syn_curr_mem_rd),
        .syn_curr_mem_wait_i    (np_syn_curr_mem_wait),
        .syn_curr_mem_addr_o    (np_syn_curr_mem_addr),
        .syn_curr_mem_data_o    (np_syn_curr_mem_data_wr),
        .syn_curr_mem_data_i    (syn_curr_mem_data_i),

        // Bias current memory (exclusive to neuron_processing)
        .bias_curr_mem_rd_o     (bias_curr_mem_rd_o),
        .bias_curr_mem_wait_i   (bias_curr_mem_wait_i),
        .bias_curr_mem_addr_o   (bias_curr_mem_addr_o),
        .bias_curr_mem_data_i   (bias_curr_mem_data_i),

        // Threshold memory (exclusive to neuron_processing)
        .thresh_mem_rd_o        (thresh_mem_rd_o),
        .thresh_mem_wait_i      (thresh_mem_wait_i),
        .thresh_mem_addr_o      (thresh_mem_addr_o),
        .thresh_mem_data_i      (thresh_mem_data_i),

        // Potential memory (exclusive to neuron_processing)
        .pot_mem_wr_o           (pot_mem_wr_o),
        .pot_mem_rd_o           (pot_mem_rd_o),
        .pot_mem_wait_i         (pot_mem_wait_i),
        .pot_mem_addr_o         (pot_mem_addr_o),
        .pot_mem_data_o         (pot_mem_data_o),
        .pot_mem_data_i         (pot_mem_data_i),

        // Output spike memory (exclusive to neuron_processing)
        .spike_mem_wr_o         (spike_mem_wr_o),
        .spike_mem_wait_i       (spike_mem_wait_i),
        .spike_mem_addr_o       (spike_mem_addr_o),
        .spike_mem_data_o       (spike_mem_data_o)
    );

endmodule
