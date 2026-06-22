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
    parameter SP_ACT_IDX_SZ           = `PIN_BITS,  // input-neuron flat-index width (override per app)
    // Width of the activation element index that selects the weight row.
    // Must be >= COL_BITS (the act_index width) so the WHOLE index survives to
    // weight_row_base; at 5 it truncated to the within-word bits (alias bug).
    parameter SP_ACT_DATA_IDX_SZ      = `COL_BITS,
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
    reg                           act_signed_r;       // 1 = signed-activation MAC (§5.4); default 0

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
    reg                           np_lut_window_r;           // §5.1: 1 = windowed/saturated LUT index; default 0
    reg                           np_out_signed_r;           // §5.2: 1 = sign-extend signed LUT (tanh) output; default 0
    reg                           np_bias_en_r;              // §5.5: 1 = seed syn_curr with the per-neuron bias before the MAC; default 0
    reg                    [4:0]  bias_bin_point_r;          // §5.5: binary point of the stored bias (aligned to bin_point_syn_curr)

    //----------------------------------------------------------------
    // Shared config registers
    //----------------------------------------------------------------
    reg                    [4:0] bin_point_syn_curr_r;
    reg                    [4:0] np_out_bin_point_r;   // output-activation binary point;
                                                       // requant shift = bin_point_syn_curr - this.
                                                       // 0 = requant disabled (legacy low bits).
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
    //   8'hA4  np_out_bin_point_r          (annAcc new) output-activation bin point;
    //                                      requant shift = bin_point_syn_curr - this;
    //                                      0 = requant disabled (legacy low bits)
    //
    //   shared:
    //   8'h40  bin_point_syn_curr_r
    //   8'h6C  np_pot_decay_mult_r
    //   8'h9C  sp_skip_neuron_r       [0]    1 = skip neuron_processing after spike_processing
    //
    //   per-task control word (lives inside the cfg_mem push window):
    //   8'h10  task_ctrl              [0]    sp_skip_neuron
    //                                 [3:1]  reserved (np_mode on LIF accelerators)
    //                                 [6:4]  reserved (sp_act_sz, future)
    //                                 [8:7]  reserved (np_thresh_op, future)
    //          Writes here drive the same flop as 0x9C — last write wins.
    //          config_manager pushes cfg_mem word 4 here on every TASK
    //          dispatch, so per-cfg_id rotation works automatically.
    //
    //   per-task decay multiplier (also inside the cfg_mem push window):
    //   8'h1C  np_pot_decay_mult_r       (32-bit Q0.32) — alias of 0x6C
    //          (annAcc has no syn_curr decay path; 0x18 unused.)
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
            act_signed_r            <= 1'b0;
            np_last_neuron_idx_r    <= {NP_NEURON_IDX_SZ{1'b0}};
            np_bias_curr_base_addr_r<= {MEM_ADDR_BITS{1'b0}};
            np_thresh_base_addr_r   <= {MEM_ADDR_BITS{1'b0}};
            np_pot_base_addr_r      <= {MEM_ADDR_BITS{1'b0}};
            np_spike_base_addr_r    <= {MEM_ADDR_BITS{1'b0}};
            np_syn_curr_sz_r        <= {NP_SYN_CURR_SLICE_SZ{1'b0}};
            np_bias_curr_sz_r       <= 3'b0;
            np_pot_sz_r             <= {NP_POT_SLICE_SZ{1'b0}};
            np_thresh_op_r          <= 2'b0;
            np_lut_window_r         <= 1'b0;
            np_out_signed_r         <= 1'b0;
            np_bias_en_r            <= 1'b0;
            bias_bin_point_r        <= 5'b0;
            sp_skip_neuron_r        <= 1'b0;
            bin_point_syn_curr_r    <= 5'b0;
            np_out_bin_point_r      <= 5'b0;
            np_pot_decay_mult_r     <= 32'b0;
        end else if (sys_req_i & addr_match) begin
            case (sys_addr_i[7:0])
                // ---- packed per-task config (cfg_mem word i -> offset i*4) ----
                // W0..W8: full-width base addresses + decay multipliers
                // (annAcc has no syn-curr decay, so W7/0x1C is unused.)
                8'h00: sp_act_base_addr_r       <= sys_data_i[MEM_ADDR_BITS-1:0];
                8'h04: sp_weight_base_addr_r    <= sys_data_i[MEM_ADDR_BITS-1:0];
                8'h08: syn_curr_base_addr_r     <= sys_data_i[MEM_ADDR_BITS-1:0];
                8'h0C: np_bias_curr_base_addr_r <= sys_data_i[MEM_ADDR_BITS-1:0];
                8'h10: np_thresh_base_addr_r    <= sys_data_i[MEM_ADDR_BITS-1:0]; // lut base
                8'h14: np_pot_base_addr_r       <= sys_data_i[MEM_ADDR_BITS-1:0];
                8'h18: np_spike_base_addr_r     <= sys_data_i[MEM_ADDR_BITS-1:0];
                8'h20: np_pot_decay_mult_r      <= sys_data_i[31:0];
                // S0..S3: two 16-bit size lanes each (low [15:0], high [31:16])
                8'h24: begin                                       // S0
                    sp_in_x_len_r  <= sys_data_i[SP_X_INPUT_SZ-1:0];
                    sp_in_y_len_r  <= sys_data_i[16 +: SP_Y_INPUT_SZ];
                end
                8'h28: begin                                       // S1
                    sp_out_x_len_r <= sys_data_i[SP_X_OUTPUT_SZ-1:0];
                    sp_out_y_len_r <= sys_data_i[16 +: SP_Y_OUTPUT_SZ];
                end
                8'h2C: begin                                       // S2
                    sp_rows_per_neuron_r <= sys_data_i[SP_ROWS_PER_NEURON-1:0];
                    np_last_neuron_idx_r <= sys_data_i[16 +: NP_NEURON_IDX_SZ];
                end
                8'h30: sp_total_timesteps_r <= sys_data_i[SP_TIMESTEP_SZ-1:0]; // S3
                // M0,M1: bit-packed mode / slice-size fields (4-bit lanes + headroom)
                8'h34: begin                                       // M0
                    sp_weight_sz_r    <= sys_data_i[ 3:0];
                    np_syn_curr_sz_r  <= sys_data_i[ 7:4];
                    np_bias_curr_sz_r <= sys_data_i[11:8];   // = lut_out_sz on annAcc
                    np_pot_sz_r       <= sys_data_i[15:12];  // = act_out_sz on annAcc
                    sp_act_sz_r       <= sys_data_i[19:16];
                    np_thresh_op_r    <= sys_data_i[23:20];
                    sp_weight_mode_r  <= sys_data_i[27:24];
                    act_signed_r      <= sys_data_i[28];   // §5.4 signed-activation MAC (default 0)
                    np_lut_window_r   <= sys_data_i[29];   // §5.1 windowed/saturated LUT index (default 0)
                    np_out_signed_r   <= sys_data_i[30];   // §5.2 signed LUT (tanh) output (default 0)
                    np_bias_en_r      <= sys_data_i[31];   // §5.5 per-neuron bias accumulator-seed (default 0)
                end
                8'h38: begin                                       // M1
                    sp_skip_neuron_r      <= sys_data_i[0];
                    // [5:2] np_mode -> not present on annAcc
                    sp_weights_per_word_r <= sys_data_i[ 9:6];
                    bin_point_syn_curr_r  <= sys_data_i[15:10];
                    np_out_bin_point_r    <= sys_data_i[21:16];
                    bias_bin_point_r      <= sys_data_i[27:22]; // §5.5 bias binary point
                end
                // ---- boot-only conv/sparse params (out-of-packed-window, >=0x5C) ----
                8'h5C: sp_weight_idx_sz_r      <= sys_data_i[SP_WEIGHT_IDX_SZ-1:0];
                8'h74: sp_x_kernel_len_r       <= sys_data_i[SP_X_KERNEL_SZ-1:0];
                8'h78: sp_y_kernel_len_r       <= sys_data_i[SP_Y_KERNEL_SZ-1:0];
                8'h7C: sp_x_kernel_step_r      <= sys_data_i[SP_X_STEP_SZ-1:0];
                8'h80: sp_y_kernel_step_r      <= sys_data_i[SP_Y_STEP_SZ-1:0];
                8'h84: sp_x_kernel_offset_r    <= sys_data_i[SP_X_KERNEL_OFF_SZ-1:0];
                8'h88: sp_y_kernel_offset_r    <= sys_data_i[SP_Y_KERNEL_OFF_SZ-1:0];
                8'h8C: sp_index_sz_r           <= sys_data_i[SP_WEIGHT_SLICE_SZ-1:0];
                8'h90: sp_tuple_sz_r           <= sys_data_i[SP_WEIGHT_SLICE_SZ-1:0];
                8'h94: sp_sparse_count_r       <= sys_data_i[`PIN_BITS-1:0];
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
    // §5.5 bias-preload requester (drives syn_curr while seeding the per-neuron
    // bias before the MAC). Declared below; highest priority — it runs only
    // before spike_processing starts, so NP/SP are idle during the preload.
    wire                  pl_syn_curr_rd;
    wire                  pl_syn_curr_wr;
    wire [`ADDR_SIZE-1:0] pl_syn_curr_addr;
    wire  [`POT_BITS-1:0] pl_syn_curr_data;
    wire                  pl_bias_rd;
    wire [`ADDR_SIZE-1:0] pl_bias_addr;
    wire                  pl_busy;

    wire pl_req = pl_syn_curr_rd | pl_syn_curr_wr;
    wire np_req = np_syn_curr_mem_rd | np_syn_curr_mem_wr;
    wire sp_req = sp_syn_curr_mem_rd | sp_syn_curr_mem_wr;

    wire grant_pl = pl_req;
    wire grant_np = np_req & ~pl_req;
    wire grant_sp = sp_req & ~np_req & ~pl_req;

    assign syn_curr_mem_wr_o   = grant_pl ? pl_syn_curr_wr
                               : grant_np ? np_syn_curr_mem_wr
                                          : (grant_sp ? sp_syn_curr_mem_wr  : 1'b0);
    assign syn_curr_mem_rd_o   = grant_pl ? pl_syn_curr_rd
                               : grant_np ? np_syn_curr_mem_rd
                                          : (grant_sp ? sp_syn_curr_mem_rd  : 1'b0);
    assign syn_curr_mem_addr_o = grant_pl ? pl_syn_curr_addr
                               : grant_np ? np_syn_curr_mem_addr
                                          : sp_syn_curr_mem_addr;
    assign syn_curr_mem_data_o = grant_pl ? pl_syn_curr_data
                               : grant_np ? np_syn_curr_mem_data_wr
                                          : sp_syn_curr_mem_data_wr;

    assign np_syn_curr_mem_wait = (~grant_np & np_req) | (grant_np & syn_curr_mem_wait_i);
    assign sp_syn_curr_mem_wait = (~grant_sp & sp_req) | (grant_sp & syn_curr_mem_wait_i);

    // §5.5 bias_curr read — driven by the preload FSM when np_bias_en, else off
    // (bit-identical to the old tie-off: pl_bias_rd is 0 when the preload idles).
    assign bias_curr_mem_rd_o   = pl_bias_rd;
    assign bias_curr_mem_addr_o = pl_bias_addr;

    //----------------------------------------------------------------
    // Internal scheduler wires
    //----------------------------------------------------------------
    wire sp_acc_busy;
    wire sp_acc_finished;
    wire np_neuron_proc_finished;
    wire np_acc_busy;
    wire np_acc_finished;

    assign acc_busy_o     = sp_acc_busy | np_acc_busy | pl_busy;   // pl_busy: §5.5 preload
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
    // §5.5 Bias-channel accumulator-seed preload
    //
    // When np_bias_en_r is set, this small FSM runs ONCE on dispatch — before
    // spike_processing's MAC starts — and adds the per-neuron bias from
    // bias_curr_mem into syn_curr_mem, aligning the stored bias from its own
    // binary point (bias_bin_point_r) to the accumulator binary point
    // (bin_point_syn_curr_r). It is an ADD (read-modify-write), so it composes
    // with both a FILL-zeroed accumulator (gates r/z, candidate c_n -> seed=bias)
    // and a Hadamard-seeded accumulator (candidate n: seed = r·c_n + bias_in).
    // Seeding BEFORE the MAC keeps the summation order bias-first, matching the
    // emulator (mac_signed(bias, ...)). Default-off: when np_bias_en_r=0 the FSM
    // stays IDLE, drives nothing, and spike_processing starts directly on
    // dispatch — bit-identical to the pre-§5.5 behaviour.
    //
    // sram_bram is a 1-cycle registered read: PL_RD issues the syn_curr + bias
    // reads, PL_LAT latches them when valid, PL_WB writes back the sum. The
    // shared-pool wait is honoured (stall while syn_curr_mem_wait_i).
    //================================================================
    localparam PL_IDLE = 2'd0, PL_RD = 2'd1, PL_LAT = 2'd2, PL_WB = 2'd3;
    reg [1:0]                    pl_state_r;
    reg   [NP_NEURON_IDX_SZ-1:0] pl_neuron_r;
    reg          [`POT_BITS-1:0] pl_syn_lat_r;
    reg          [`WTD_BITS-1:0] pl_bias_lat_r;

    assign pl_busy = (pl_state_r != PL_IDLE);
    wire pl_last = (pl_neuron_r == np_last_neuron_idx_r);

    assign pl_syn_curr_rd = (pl_state_r == PL_RD);
    assign pl_bias_rd     = (pl_state_r == PL_RD);
    assign pl_syn_curr_wr = (pl_state_r == PL_WB);
    assign pl_syn_curr_addr = syn_curr_base_addr_r    + {{(`ADDR_SIZE-NP_NEURON_IDX_SZ){1'b0}}, pl_neuron_r};
    assign pl_bias_addr     = np_bias_curr_base_addr_r + {{(`ADDR_SIZE-NP_NEURON_IDX_SZ){1'b0}}, pl_neuron_r};

    // Align the stored bias to the accumulator (syn_curr) binary point.
    wire signed [6:0] pl_shift = $signed({2'b00, bin_point_syn_curr_r})
                               - $signed({2'b00, bias_bin_point_r});
    wire signed [`POT_BITS-1:0] pl_bias_s = $signed(pl_bias_lat_r);
    wire signed [`POT_BITS-1:0] pl_bias_aligned =
            (pl_shift >= 0) ? (pl_bias_s <<<  pl_shift)
                            : (pl_bias_s >>> (-pl_shift));
    assign pl_syn_curr_data = pl_syn_lat_r + pl_bias_aligned;

    // Pulse: the final neuron's bias write has been accepted -> release the MAC.
    wire pl_done_pulse = (pl_state_r == PL_WB) & ~syn_curr_mem_wait_i & pl_last;

    always @(posedge clk) begin
        if (reset) begin
            pl_state_r    <= PL_IDLE;
            pl_neuron_r   <= {NP_NEURON_IDX_SZ{1'b0}};
            pl_syn_lat_r  <= {`POT_BITS{1'b0}};
            pl_bias_lat_r <= {`WTD_BITS{1'b0}};
        end else begin
            case (pl_state_r)
                PL_IDLE:
                    if (my_dispatch & np_bias_en_r) begin
                        pl_neuron_r <= {NP_NEURON_IDX_SZ{1'b0}};
                        pl_state_r  <= PL_RD;
                    end
                PL_RD:                                   // issue reads; advance when accepted
                    if (~syn_curr_mem_wait_i)
                        pl_state_r <= PL_LAT;
                PL_LAT: begin                            // read data now valid — latch both
                    pl_syn_lat_r  <= syn_curr_mem_data_i;
                    pl_bias_lat_r <= bias_curr_mem_data_i;
                    pl_state_r    <= PL_WB;
                end
                PL_WB:                                   // write syn_curr += aligned bias
                    if (~syn_curr_mem_wait_i) begin
                        if (pl_last)
                            pl_state_r <= PL_IDLE;
                        else begin
                            pl_neuron_r <= pl_neuron_r + 1'b1;
                            pl_state_r  <= PL_RD;
                        end
                    end
            endcase
        end
    end

    // spike_processing starts on dispatch, OR (when bias-seeding) once the
    // preload has finished writing every neuron's seed.
    wire sp_go = np_bias_en_r ? pl_done_pulse : my_dispatch;

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
        .ACT_IDX_SZ           (SP_ACT_IDX_SZ),
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
        .act_signed_i           (act_signed_r),

        // Scheduler — gated dispatch (deferred past the §5.5 bias preload)
        .start_new_block_i      (sp_go),
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
        .bin_point_syn_curr_i   (bin_point_syn_curr_r),  // requant: input bin point
        .np_out_bin_point_i     (np_out_bin_point_r),    // requant: output bin point (0=off)
        .lut_window_i           (np_lut_window_r),       // §5.1 windowed/saturated LUT index
        .out_signed_i           (np_out_signed_r),       // §5.2 signed LUT (tanh) output
        .pot_decay_mult_i       (np_pot_decay_mult_r[NP_POT_DECAY_BITS-1:0]),

        // Scheduler – triggered by spike_processing completion
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
