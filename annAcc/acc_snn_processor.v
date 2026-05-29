// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
// annAcc top-level: instantiates spike_processing (MAC accumulation into
// syn_curr_mem) and neuron_processing (threshold + decay, writes to pot_mem
// and spike_mem/act_out_mem).  syn_curr_mem is shared and arbitrated here.

`include "../shared/constants.v"

module ann_processor # (

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
    parameter SP_ACT_SLICE_SZ         = 5,   // max 32-bit acts in annAcc
    parameter SP_ACT_DATA_IDX_SZ      = 5,
    parameter SP_WEIGHT_ENTRY_BITS    = 8,
    parameter SP_WEIGHT_IDX_SZ        = 5,
    parameter SP_WEIGHT_SLICE_SZ      = 5,
    parameter SP_WEIGHT_DATA_IDX_SZ   = 2,
    parameter SP_SYN_CURR_IDX_SZ      = 10,
    parameter SP_SYN_CURR_DATA_IDX_SZ = 5,
    parameter SP_SYN_CURR_SLICE_SZ    = 3,
    parameter SP_SYN_CURR_SLICE_BITS  = 10,
    parameter SP_BIAS_CURR_IDX_SZ     = 2,
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
    parameter NP_SYN_CURR_SLICE_BITS  = 32,  // full 32-bit potential
    parameter NP_LUT_IDX_SZ           = 8,   // bits of potential used as LUT address
    parameter NP_LUT_DATA_IDX_SZ      = 5,
    parameter NP_LUT_SLICE_SZ         = 3,
    parameter NP_LUT_SLICE_BITS       = 8,
    parameter NP_POT_SLICE_SZ         = 3,
    parameter NP_POT_SLICE_BITS       = 32,
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
    output wire  [`POT_BITS-1:0] syn_curr_mem_data_o,
    input  wire  [`POT_BITS-1:0] syn_curr_mem_data_i,

    //================================================================
    // External memory – bias currents (kept for port compatibility; tied off)
    //================================================================
    output wire                  bias_curr_mem_rd_o,
    input  wire                  bias_curr_mem_wait_i,
    output wire [`ADDR_SIZE-1:0] bias_curr_mem_addr_o,
    input  wire  [`WTD_BITS-1:0] bias_curr_mem_data_i,

    //================================================================
    // External memory – LUT (reuses thresh_mem ports, NP only)
    //================================================================
    output wire                  thresh_mem_rd_o,
    input  wire                  thresh_mem_wait_i,
    output wire [`ADDR_SIZE-1:0] thresh_mem_addr_o,
    input  wire  [`WTD_BITS-1:0] thresh_mem_data_i,

    //================================================================
    // External memory – neuron potentials / decayed activations (NP only)
    //================================================================
    output wire                  pot_mem_wr_o,
    output wire                  pot_mem_rd_o,
    input  wire                  pot_mem_wait_i,
    output wire [`ADDR_SIZE-1:0] pot_mem_addr_o,
    output wire  [`POT_BITS-1:0] pot_mem_data_o,
    input  wire  [`POT_BITS-1:0] pot_mem_data_i,

    //================================================================
    // External memory – output activations (NP only)
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
    reg    [SP_X_INPUT_SZ-1:0]   sp_in_x_len_r;
    reg    [SP_Y_INPUT_SZ-1:0]   sp_in_y_len_r;
    reg   [SP_X_OUTPUT_SZ-1:0]   sp_out_x_len_r;
    reg   [SP_Y_OUTPUT_SZ-1:0]   sp_out_y_len_r;
    reg   [SP_ELEMS_PER_ROW-1:0] sp_weights_per_word_r;
    reg [SP_ROWS_PER_NEURON-1:0] sp_rows_per_neuron_r;
    reg   [SP_WEIGHT_IDX_SZ-1:0] sp_weight_idx_sz_r;
    (* MAX_FANOUT = "20" *) reg [1:0] sp_weight_mode_r;
    reg    [SP_X_KERNEL_SZ-1:0]  sp_x_kernel_len_r;
    reg    [SP_Y_KERNEL_SZ-1:0]  sp_y_kernel_len_r;
    reg [SP_X_KERNEL_OFF_SZ-1:0] sp_x_kernel_offset_r;
    reg [SP_Y_KERNEL_OFF_SZ-1:0] sp_y_kernel_offset_r;
    reg      [SP_X_STEP_SZ-1:0]  sp_x_kernel_step_r;
    reg      [SP_Y_STEP_SZ-1:0]  sp_y_kernel_step_r;
    reg  [SP_WEIGHT_SLICE_SZ-1:0] sp_index_sz_r;
    reg  [SP_WEIGHT_SLICE_SZ-1:0] sp_tuple_sz_r;
    reg          [`PIN_BITS-1:0]  sp_sparse_count_r;
    reg                    [2:0]  sp_act_sz_r;        // runtime activation element width

    //----------------------------------------------------------------
    // neuron_processing config registers
    //----------------------------------------------------------------
    reg   [NP_NEURON_IDX_SZ-1:0] np_last_neuron_idx_r;
    reg      [MEM_ADDR_BITS-1:0] np_bias_curr_base_addr_r;  // kept for compat
    reg      [MEM_ADDR_BITS-1:0] np_thresh_base_addr_r;     // lut_base_addr
    reg      [MEM_ADDR_BITS-1:0] np_pot_base_addr_r;
    reg      [MEM_ADDR_BITS-1:0] np_spike_base_addr_r;
    reg [NP_SYN_CURR_SLICE_SZ-1:0] np_syn_curr_sz_r;
    reg                    [2:0]  np_bias_curr_sz_r;         // reused as lut_out_sz
    reg    [NP_POT_SLICE_SZ-1:0]  np_pot_sz_r;              // reused as act_out_sz
    reg                     [1:0] np_thresh_op_r;            // 00=RELU 01=LUT 10=ABS

    //----------------------------------------------------------------
    // Shared config registers
    //----------------------------------------------------------------
    reg                    [4:0] bin_point_syn_curr_r;
    reg                   [31:0] np_pot_decay_mult_r;
    reg                          sp_skip_neuron_r;     // 1 = skip neuron_processing after spike_processing

    //================================================================
    // AXI config register decode
    //
    //   spike_processing:
    //   8'h00  sp_act_base_addr_r
    //   8'h04  sp_weight_base_addr_r
    //   8'h08  syn_curr_base_addr_r
    //   8'h0C  sp_weight_sz_r
    //   8'h14  sp_total_timesteps_r
    //   8'h44–8'h5C  grid/kernel size registers
    //   8'h70  sp_weight_mode_r
    //   8'h74–8'h88  conv kernel registers
    //   8'h8C–8'h94  sparse registers
    //   8'h98  sp_act_sz_r                 (annAcc new)
    //
    //   neuron_processing:
    //   8'h20  np_last_neuron_idx_r
    //   8'h28  np_bias_curr_base_addr_r    (unused; kept for compat)
    //   8'h2C  np_thresh_base_addr_r       (= lut_base_addr)
    //   8'h30  np_pot_base_addr_r
    //   8'h34  np_syn_curr_sz_r
    //   8'h38  np_bias_curr_sz_r           (reused as lut_out_sz)
    //   8'h3C  np_pot_sz_r                 (reused as act_out_sz)
    //   8'h64  np_spike_base_addr_r
    //   8'hA0  np_thresh_op_r              (annAcc new)
    //
    //   shared:
    //   8'h40  bin_point_syn_curr_r
    //   8'h6C  np_pot_decay_mult_r
    //   8'h9C  sp_skip_neuron_r       [0]    1 = skip neuron_processing after spike_processing
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
            sp_act_sz_r             <= 3'b0;
            np_last_neuron_idx_r    <= {NP_NEURON_IDX_SZ{1'b0}};
            np_bias_curr_base_addr_r<= {MEM_ADDR_BITS{1'b0}};
            np_thresh_base_addr_r   <= {MEM_ADDR_BITS{1'b0}};
            np_pot_base_addr_r      <= {MEM_ADDR_BITS{1'b0}};
            np_spike_base_addr_r    <= {MEM_ADDR_BITS{1'b0}};
            np_syn_curr_sz_r        <= {NP_SYN_CURR_SLICE_SZ{1'b0}};
            np_bias_curr_sz_r       <= 3'b0;
            np_pot_sz_r             <= {NP_POT_SLICE_SZ{1'b0}};
            np_thresh_op_r          <= 2'b0;
            sp_skip_neuron_r        <= 1'b0;
            bin_point_syn_curr_r    <= 5'b0;
            np_pot_decay_mult_r     <= 32'b0;
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
                8'h98: sp_act_sz_r             <= sys_data_i[2:0];
                8'h20: np_last_neuron_idx_r    <= sys_data_i[NP_NEURON_IDX_SZ-1:0];
                8'h28: np_bias_curr_base_addr_r<= sys_data_i[MEM_ADDR_BITS-1:0];
                8'h2C: np_thresh_base_addr_r   <= sys_data_i[MEM_ADDR_BITS-1:0];
                8'h30: np_pot_base_addr_r      <= sys_data_i[MEM_ADDR_BITS-1:0];
                8'h64: np_spike_base_addr_r    <= sys_data_i[MEM_ADDR_BITS-1:0];
                8'h34: np_syn_curr_sz_r        <= sys_data_i[NP_SYN_CURR_SLICE_SZ-1:0];
                8'h38: np_bias_curr_sz_r       <= sys_data_i[2:0];
                8'h3C: np_pot_sz_r             <= sys_data_i[NP_POT_SLICE_SZ-1:0];
                8'h40: bin_point_syn_curr_r    <= sys_data_i[4:0];
                8'h6C: np_pot_decay_mult_r     <= sys_data_i[31:0];
                8'hA0: np_thresh_op_r          <= sys_data_i[1:0];
                8'h9C: sp_skip_neuron_r        <= sys_data_i[0];
                default: ;
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
    wire                  sp_syn_curr_mem_wait;

    //----------------------------------------------------------------
    // Internal syn_curr wires – neuron_processing side
    //----------------------------------------------------------------
    wire                  np_syn_curr_mem_wr;
    wire                  np_syn_curr_mem_rd;
    wire [`ADDR_SIZE-1:0] np_syn_curr_mem_addr;
    wire  [`POT_BITS-1:0] np_syn_curr_mem_data_wr;
    wire                  np_syn_curr_mem_wait;

    //================================================================
    // Synaptic-current memory arbiter (NP has fixed higher priority)
    //================================================================
    wire np_req = np_syn_curr_mem_rd | np_syn_curr_mem_wr;
    wire sp_req = sp_syn_curr_mem_rd | sp_syn_curr_mem_wr;

    wire grant_np = np_req;
    wire grant_sp = sp_req & ~np_req;

    assign syn_curr_mem_wr_o   = grant_np ? np_syn_curr_mem_wr
                                          : (grant_sp ? sp_syn_curr_mem_wr  : 1'b0);
    assign syn_curr_mem_rd_o   = grant_np ? np_syn_curr_mem_rd
                                          : (grant_sp ? sp_syn_curr_mem_rd  : 1'b0);
    assign syn_curr_mem_addr_o = grant_np ? np_syn_curr_mem_addr
                                          : sp_syn_curr_mem_addr;
    assign syn_curr_mem_data_o = grant_np ? np_syn_curr_mem_data_wr
                                          : sp_syn_curr_mem_data_wr;

    assign np_syn_curr_mem_wait = (~grant_np & np_req) | (grant_np & syn_curr_mem_wait_i);
    assign sp_syn_curr_mem_wait = (~grant_sp & sp_req) | (grant_sp & syn_curr_mem_wait_i);

    // bias_curr ports are unused in annAcc — tie outputs off
    assign bias_curr_mem_rd_o   = 1'b0;
    assign bias_curr_mem_addr_o = {`ADDR_SIZE{1'b0}};

    //----------------------------------------------------------------
    // Internal scheduler wires
    //----------------------------------------------------------------
    wire sp_acc_busy;
    wire sp_acc_finished;
    wire np_neuron_proc_finished;
    wire np_acc_busy;
    wire np_acc_finished;

    assign acc_busy_o     = sp_acc_busy | np_acc_busy;
    assign acc_finished_o = sp_skip_neuron_r ? sp_acc_finished : np_acc_finished;

    //================================================================
    // spike_processing instantiation
    //================================================================
    ann_spike_processing # (
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
        .x_kernel_len_i         (sp_x_kernel_len_r),
        .y_kernel_len_i         (sp_y_kernel_len_r),
        .x_kernel_offset_i      (sp_x_kernel_offset_r),
        .y_kernel_offset_i      (sp_y_kernel_offset_r),
        .x_kernel_step_i        (sp_x_kernel_step_r),
        .y_kernel_step_i        (sp_y_kernel_step_r),
        .index_sz_i             (sp_index_sz_r),
        .tuple_sz_i             (sp_tuple_sz_r),
        .sparse_count_i         (sp_sparse_count_r),
        .act_slice_sz_i         (sp_act_sz_r),

        // Scheduler
        .start_new_block_i      (start_new_block_i),
        .target_acc_i           (target_acc_i),
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

        // Weight memory
        .weight_mem_rd_o        (weight_mem_rd_o),
        .weight_mem_wait_i      (weight_mem_wait_i),
        .weight_mem_addr_o      (weight_mem_addr_o),
        .weight_mem_data_i      (weight_mem_data_i),

        // Activation memory
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
    ann_neuron_processing # (
        .TGT_ACC_ID           (TGT_ACC_ID),
        .NUM_TIMESTEPS        (NP_NUM_TIMESTEPS),
        .TIMESTEP_SZ          (NP_TIMESTEP_SZ),
        .IN_DATA_BITS         (NP_IN_DATA_BITS),
        .NEURON_IDX_SZ        (NP_NEURON_IDX_SZ),
        .SYN_CURR_IDX_SZ      (NP_SYN_CURR_IDX_SZ),
        .SYN_CURR_DATA_IDX_SZ (NP_SYN_CURR_DATA_IDX_SZ),
        .SYN_CURR_SLICE_SZ    (NP_SYN_CURR_SLICE_SZ),
        .SYN_CURR_SLICE_BITS  (NP_SYN_CURR_SLICE_BITS),
        .LUT_IDX_SZ           (NP_LUT_IDX_SZ),
        .LUT_DATA_IDX_SZ      (NP_LUT_DATA_IDX_SZ),
        .LUT_SLICE_SZ         (NP_LUT_SLICE_SZ),
        .LUT_SLICE_BITS       (NP_LUT_SLICE_BITS),
        .POT_SLICE_SZ         (NP_POT_SLICE_SZ),
        .POT_SLICE_BITS       (NP_POT_SLICE_BITS),
        .POT_DECAY_BITS       (NP_POT_DECAY_BITS),
        .MEM_ADDR_BITS        (MEM_ADDR_BITS)
    ) u_neuron_processing (
        .clk                    (clk),
        .reset                  (reset),

        // Config registers
        .last_neuron_idx_i      (np_last_neuron_idx_r),
        .syn_curr_base_addr_i   (syn_curr_base_addr_r),
        .thresh_base_addr_i     (np_thresh_base_addr_r),
        .pot_base_addr_i        (np_pot_base_addr_r),
        .spike_base_addr_i      (np_spike_base_addr_r),
        .syn_curr_sz_i          (np_syn_curr_sz_r),
        .pot_sz_i               (np_pot_sz_r),
        .lut_out_sz_i           (np_bias_curr_sz_r),  // np_bias_curr_sz_r reused
        .act_out_sz_i           (np_pot_sz_r),        // same config reg for act width
        .thresh_op_i            (np_thresh_op_r),
        .pot_decay_mult_i       (np_pot_decay_mult_r[NP_POT_DECAY_BITS-1:0]),

        // Scheduler – triggered by spike_processing completion
        .start_new_block_i      (sp_acc_finished & ~sp_skip_neuron_r),
        .target_acc_i           (target_acc_i),
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

        // LUT memory (thresh_mem ports)
        .thresh_mem_rd_o        (thresh_mem_rd_o),
        .thresh_mem_wait_i      (thresh_mem_wait_i),
        .thresh_mem_addr_o      (thresh_mem_addr_o),
        .thresh_mem_data_i      (thresh_mem_data_i),

        // Decayed activation write-back → pot_mem
        .pot_mem_wr_o           (pot_mem_wr_o),
        .pot_mem_rd_o           (pot_mem_rd_o),
        .pot_mem_wait_i         (pot_mem_wait_i),
        .pot_mem_addr_o         (pot_mem_addr_o),
        .pot_mem_data_o         (pot_mem_data_o),
        .pot_mem_data_i         (pot_mem_data_i),

        // Output activations → spike_mem
        .spike_mem_wr_o         (spike_mem_wr_o),
        .spike_mem_wait_i       (spike_mem_wait_i),
        .spike_mem_addr_o       (spike_mem_addr_o),
        .spike_mem_data_o       (spike_mem_data_o)
    );

endmodule
