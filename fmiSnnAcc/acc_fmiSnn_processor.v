// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
// =============================================================================
// Module: acc_fmiSnn_processor
//
// Top-level wrapper — same two-stage pipeline as the original (spike_processing
// followed by neuron_processing) with the following changes:
//
//   Removed config registers (vs the snn variant):
//     np_syn_curr_decay_mult_r  (replaced by per-neuron dcy_syn_mem)
//     np_pot_decay_mult_r       (replaced by per-neuron dcy_mem_mem)
//
//   New config registers (NP only), delivered in the PACKED per-task window
//   (W6..W11, offsets 0x18-0x2C; has_ada in M0 bit 30 — see the decode below):
//     np_dcy_syn_base_addr_r    base address of per-neuron syn decay mem
//     np_dcy_mem_base_addr_r    base address of per-neuron mem decay mem
//     np_has_ada_r              enable adaptive threshold for this layer
//     np_ada_base_addr_r        ada state memory base
//     np_b_eff_base_addr_r      b_eff = scl_mem*adapt_b memory base
//     np_dcy_ada_base_addr_r    ada decay factor memory base
//     np_scl_ada_base_addr_r    ada scale factor (1-dcy_ada) memory base
//
//   New memory ports exposed for the six new per-neuron memories.
//   The bias_curr memory is removed (FMI model has no per-neuron membrane bias).
// =============================================================================

`include "../shared/constants.v"

module acc_fmiSnn_processor # (
    parameter TGT_ACC_ID              = 'h0,
    parameter TGT_CONFIG_BASE_ADDR    = 32'hFFFFFFFF,
    //----------------------------------------------------------------
    // Parameters forwarded to spike_processing
    //----------------------------------------------------------------
    parameter SP_NUM_TIMESTEPS        = 32,
    parameter SP_X_INPUT_SZ           = 8,
    parameter SP_X_OUTPUT_SZ          = 8,
    parameter SP_X_KERNEL_SZ          = 3,
    parameter SP_X_KERNEL_OFF_SZ      = 3,
    parameter SP_X_STEP_SZ            = 3,
    parameter SP_ELEMS_PER_ROW        = 4,
    parameter SP_ROWS_PER_NEURON      = 16,
    parameter SP_TIMESTEP_SZ          = 10,
    parameter SP_IN_DATA_BITS         = 32,
    parameter SP_ELEM_SZ              = 8,
    parameter SP_ACT_SLICE_SZ         = 3,
    parameter SP_ACT_IDX_SZ           = `PIN_BITS,  // input-neuron flat-index width (override per app)
    parameter SP_ACT_DATA_IDX_SZ      = 5,
    parameter SP_WEIGHT_ENTRY_BITS    = 8,
    parameter SP_WEIGHT_IDX_SZ        = 5,
    parameter SP_WEIGHT_SLICE_SZ      = 5,
    parameter SP_WEIGHT_DATA_IDX_SZ   = 2,
    parameter SP_SYN_CURR_IDX_SZ      = 10,
    parameter SP_SYN_CURR_DATA_IDX_SZ = 5,
    parameter SP_SYN_CURR_SLICE_SZ    = 3,
    parameter SP_SYN_CURR_SLICE_BITS  = 32,
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
    parameter NP_SYN_CURR_SLICE_BITS  = 32,
    parameter NP_POT_IDX_SZ           = 2,
    parameter NP_POT_DATA_IDX_SZ      = 5,
    parameter NP_POT_SLICE_SZ         = 3,
    parameter NP_POT_SLICE_BITS       = 32,
    parameter NP_SPIKE_IDX_SZ         = 2,
    parameter NP_SPIKE_DATA_IDX_SZ    = 5,
    parameter NP_SPIKE_SLICE_SZ       = 3,
    parameter NP_SPIKE_SLICE_BITS     = 8,
    parameter MEM_ADDR_BITS           = `ADDR_SIZE
)(
    input  wire clk,
    input  wire reset,

    // AXI config interface
    input  wire                    sys_req_i,
    output wire                    sys_ack_o,
    input  wire             [31:0] sys_addr_i,
    input  wire             [31:0] sys_data_i,

    // Scheduler interface
    input  wire                      start_new_block_i,
    input  wire   [`TGT_ACC_SZ-1:0]  target_acc_i,
    input  wire [`SCH_ENTRY_SZ-1:0]  buffer_info_i,
    output wire                      spike_proc_finished_o,
    output wire                      acc_busy_o,
    output wire                      acc_finished_o,

    // Buffer addresses — spike_processing
    input  wire [`PIN_BITS-1:0] sp_src1_buff_addr_i,
    input  wire [`PIN_BITS-1:0] sp_src2_buff_addr_i,
    input  wire [`PIN_BITS-1:0] sp_src3_buff_addr_i,
    input  wire [`PIN_BITS-1:0] sp_tgt_buff_addr_i,
    input  wire [`PIN_BITS-1:0] sp_weight_row_len_i,

    // Buffer addresses — neuron_processing
    input  wire [`PIN_BITS-1:0] np_src1_buff_addr_i,
    input  wire [`PIN_BITS-1:0] np_src2_buff_addr_i,
    input  wire [`PIN_BITS-1:0] np_src3_buff_addr_i,
    input  wire [`PIN_BITS-1:0] np_tgt_buff_addr_i,
    input  wire [`PIN_BITS-1:0] np_weight_row_len_i,

    // External memory — weights (spike_processing only)
    output wire                  weight_mem_rd_o,
    input  wire                  weight_mem_wait_i,
    output wire [`ADDR_SIZE-1:0] weight_mem_addr_o,
    input  wire  [`WTD_BITS-1:0] weight_mem_data_i,

    // External memory — input activations (spike_processing only)
    output wire                  act_mem_req_o,
    input  wire                  act_mem_wait_i,
    output wire [`ADDR_SIZE-1:0] act_mem_addr_o,
    input  wire  [`ACT_BITS-1:0] act_mem_data_i,

    // External memory — synaptic currents (SHARED, arbitrated)
    output wire                  syn_curr_mem_wr_o,
    output wire                  syn_curr_mem_rd_o,
    input  wire                  syn_curr_mem_wait_i,
    output wire [`ADDR_SIZE-1:0] syn_curr_mem_addr_o,
    output wire  [`POT_BITS-1:0] syn_curr_mem_data_o,
    input  wire  [`POT_BITS-1:0] syn_curr_mem_data_i,

    // External memory — threshold (neuron_processing only)
    output wire                  thresh_mem_rd_o,
    input  wire                  thresh_mem_wait_i,
    output wire [`ADDR_SIZE-1:0] thresh_mem_addr_o,
    input  wire  [`WTD_BITS-1:0] thresh_mem_data_i,

    // External memory — potentials (neuron_processing only)
    output wire                  pot_mem_wr_o,
    output wire                  pot_mem_rd_o,
    input  wire                  pot_mem_wait_i,
    output wire [`ADDR_SIZE-1:0] pot_mem_addr_o,
    output wire  [`POT_BITS-1:0] pot_mem_data_o,
    input  wire  [`POT_BITS-1:0] pot_mem_data_i,

    // External memory — output spikes (neuron_processing only)
    output wire                  spike_mem_wr_o,
    input  wire                  spike_mem_wait_i,
    output wire [`ADDR_SIZE-1:0] spike_mem_addr_o,
    output wire  [`ACT_BITS-1:0] spike_mem_data_o,

    // External memory — per-neuron synaptic decay (neuron_processing only)
    output wire                  dcy_syn_mem_rd_o,
    input  wire                  dcy_syn_mem_wait_i,
    output wire [`ADDR_SIZE-1:0] dcy_syn_mem_addr_o,
    input  wire          [31:0]  dcy_syn_mem_data_i,

    // External memory — per-neuron membrane decay (neuron_processing only)
    output wire                  dcy_mem_mem_rd_o,
    input  wire                  dcy_mem_mem_wait_i,
    output wire [`ADDR_SIZE-1:0] dcy_mem_mem_addr_o,
    input  wire          [31:0]  dcy_mem_mem_data_i,

    // External memory — ada state (neuron_processing only, R/W)
    output wire                  ada_mem_wr_o,
    output wire                  ada_mem_rd_o,
    input  wire                  ada_mem_wait_i,
    output wire [`ADDR_SIZE-1:0] ada_mem_addr_o,
    output wire          [31:0]  ada_mem_data_o,
    input  wire          [31:0]  ada_mem_data_i,

    // External memory — b_eff (neuron_processing only)
    output wire                  b_eff_mem_rd_o,
    input  wire                  b_eff_mem_wait_i,
    output wire [`ADDR_SIZE-1:0] b_eff_mem_addr_o,
    input  wire          [31:0]  b_eff_mem_data_i,

    // External memory — dcy_ada (neuron_processing only)
    output wire                  dcy_ada_mem_rd_o,
    input  wire                  dcy_ada_mem_wait_i,
    output wire [`ADDR_SIZE-1:0] dcy_ada_mem_addr_o,
    input  wire          [31:0]  dcy_ada_mem_data_i,

    // External memory — scl_ada (neuron_processing only)
    output wire                  scl_ada_mem_rd_o,
    input  wire                  scl_ada_mem_wait_i,
    output wire [`ADDR_SIZE-1:0] scl_ada_mem_addr_o,
    input  wire          [31:0]  scl_ada_mem_data_i
);

    // =========================================================================
    // spike_processing config registers
    // =========================================================================
    reg      [MEM_ADDR_BITS-1:0] sp_act_base_addr_r;
    reg      [MEM_ADDR_BITS-1:0] sp_weight_base_addr_r;
    reg      [MEM_ADDR_BITS-1:0] syn_curr_base_addr_r;
    reg  [SP_WEIGHT_SLICE_SZ-1:0] sp_weight_sz_r;
    reg     [SP_TIMESTEP_SZ-1:0] sp_total_timesteps_r;
    reg    [SP_X_INPUT_SZ-1:0]   sp_in_x_len_r;
    reg   [SP_X_OUTPUT_SZ-1:0]   sp_out_x_len_r;
    reg   [SP_ELEMS_PER_ROW-1:0] sp_weights_per_word_r;
    reg [SP_ROWS_PER_NEURON-1:0] sp_rows_per_neuron_r;
    reg   [SP_WEIGHT_IDX_SZ-1:0] sp_weight_idx_sz_r;
    reg                    [1:0] sp_weight_mode_r;
    reg    [SP_X_KERNEL_SZ-1:0]  sp_x_kernel_len_r;
    reg [SP_X_KERNEL_OFF_SZ-1:0] sp_x_kernel_offset_r;
    reg    [SP_X_STEP_SZ-1:0]    sp_x_kernel_step_r;
    reg  [SP_WEIGHT_SLICE_SZ-1:0] sp_index_sz_r;
    reg  [SP_WEIGHT_SLICE_SZ-1:0] sp_tuple_sz_r;
    reg         [`PIN_BITS-1:0]   sp_sparse_count_r;

    // =========================================================================
    // neuron_processing config registers
    // =========================================================================
    reg  [NP_NEURON_IDX_SZ-1:0]  np_last_neuron_idx_r;
    reg     [MEM_ADDR_BITS-1:0]  np_thresh_base_addr_r;
    reg     [MEM_ADDR_BITS-1:0]  np_pot_base_addr_r;
    reg     [MEM_ADDR_BITS-1:0]  np_spike_base_addr_r;
    reg [NP_SYN_CURR_SLICE_SZ-1:0] np_syn_curr_sz_r;
    reg   [NP_POT_SLICE_SZ-1:0]  np_pot_sz_r;
    // Per-neuron decay addresses
    reg     [MEM_ADDR_BITS-1:0]  np_dcy_syn_base_addr_r;
    reg     [MEM_ADDR_BITS-1:0]  np_dcy_mem_base_addr_r;
    // Adaptive threshold
    reg                          np_has_ada_r;
    reg     [MEM_ADDR_BITS-1:0]  np_ada_base_addr_r;
    reg     [MEM_ADDR_BITS-1:0]  np_b_eff_base_addr_r;
    reg     [MEM_ADDR_BITS-1:0]  np_dcy_ada_base_addr_r;
    reg     [MEM_ADDR_BITS-1:0]  np_scl_ada_base_addr_r;

    // Shared config registers
    reg                    [4:0] bin_point_syn_curr_r;
    reg                    [2:0] np_mode_r;   // [0]=sub_on_fire [1]=reserved (was clear_syn_curr; clear via FILL) [2]=clear_pot
    reg                          sp_skip_neuron_r;  // 1 = skip neuron_processing after spike_processing
    // Real-MAC config fields: decoded for FMI config-layout parity with the MC
    // variant (regmap PACKED_FMI_* / BOOT_REG_OFFSETS_FMI). The real-valued MAC
    // datapath is MC-only (con1 needs multi-channel), so these are reserved here.
    reg                          sp_real_mac_r;     // M0[31] (reserved)
    reg                    [5:0] sp_mac_shift_r;    // 0x98   (reserved)

    // =========================================================================
    // AXI config register decode
    //
    // PACKED per-task config layout (cfg_mem word i -> offset i*4; 16 words,
    // the full config_manager stride — no spare word). Mirrored by the tools
    // regmap.PACKED_FMI_* tables; cross-checked by test_regmap_vs_rtl.py.
    //
    //   W0..W11 0x00-0x2C : full-width base addresses, one per word:
    //                       act, weight, syn_curr (shared), thresh, pot, spike,
    //                       dcy_syn, dcy_mem, ada, b_eff, dcy_ada, scl_ada
    //                       (the six adaptive-LIF per-neuron memory bases
    //                       replace the scalar decay multipliers and the
    //                       bias_curr memory of the snn variant)
    //   S0..S2  0x30-0x38 : two 16-bit size lanes each (low [15:0], high [31:16]):
    //                       S0 in_x/out_x (fmi is 1-D — no y lengths),
    //                       S1 rows_per_neuron/last_neuron_idx,
    //                       S2 total_timesteps/spare
    //   M0      0x3C      : bit-packed mode/slice-size fields:
    //                       [1:0] skip_neuron, [5:2] np_mode, [9:6] weights_per_word,
    //                       [15:10] bin_point_syn_curr, [19:16] weight_sz,
    //                       [23:20] syn_curr_sz, [27:24] pot_sz,
    //                       [29:28] weight_mode, [30] has_ada
    //
    // Boot-only conv/sparse registers stay out of the packed window (>=0x5C),
    // written once via direct AXI: 0x5C weight_idx_sz, 0x74/0x7C/0x84 x-kernel
    // len/step/offset, 0x8C index_sz, 0x90 tuple_sz, 0x94 sparse_count.
    // =========================================================================
    wire addr_match = (sys_addr_i[31:16] == TGT_CONFIG_BASE_ADDR[31:16]);
    assign sys_ack_o = sys_req_i & addr_match;

    always @(posedge clk) begin
        if (reset) begin
            sp_act_base_addr_r       <= {MEM_ADDR_BITS{1'b0}};
            sp_weight_base_addr_r    <= {MEM_ADDR_BITS{1'b0}};
            syn_curr_base_addr_r     <= {MEM_ADDR_BITS{1'b0}};
            sp_weight_sz_r           <= {SP_WEIGHT_SLICE_SZ{1'b0}};
            sp_total_timesteps_r     <= {SP_TIMESTEP_SZ{1'b0}};
            sp_in_x_len_r            <= {SP_X_INPUT_SZ{1'b0}};
            sp_out_x_len_r           <= {SP_X_OUTPUT_SZ{1'b0}};
            sp_weights_per_word_r    <= {SP_ELEMS_PER_ROW{1'b0}};
            sp_rows_per_neuron_r     <= {SP_ROWS_PER_NEURON{1'b0}};
            sp_weight_idx_sz_r       <= {SP_WEIGHT_IDX_SZ{1'b0}};
            sp_weight_mode_r         <= 2'b0;
            sp_x_kernel_len_r        <= {SP_X_KERNEL_SZ{1'b0}};
            sp_x_kernel_offset_r     <= {SP_X_KERNEL_OFF_SZ{1'b0}};
            sp_x_kernel_step_r       <= {SP_X_STEP_SZ{1'b0}};
            sp_index_sz_r            <= {SP_WEIGHT_SLICE_SZ{1'b0}};
            sp_tuple_sz_r            <= {SP_WEIGHT_SLICE_SZ{1'b0}};
            sp_sparse_count_r        <= {`PIN_BITS{1'b0}};
            np_last_neuron_idx_r     <= {NP_NEURON_IDX_SZ{1'b0}};
            np_thresh_base_addr_r    <= {MEM_ADDR_BITS{1'b0}};
            np_pot_base_addr_r       <= {MEM_ADDR_BITS{1'b0}};
            np_spike_base_addr_r     <= {MEM_ADDR_BITS{1'b0}};
            np_syn_curr_sz_r         <= {NP_SYN_CURR_SLICE_SZ{1'b0}};
            np_pot_sz_r              <= {NP_POT_SLICE_SZ{1'b0}};
            bin_point_syn_curr_r     <= 5'b0;
            np_mode_r                <= 3'b0;
            sp_skip_neuron_r         <= 1'b0;
            sp_real_mac_r            <= 1'b0;
            sp_mac_shift_r           <= 6'd0;
            np_dcy_syn_base_addr_r   <= {MEM_ADDR_BITS{1'b0}};
            np_dcy_mem_base_addr_r   <= {MEM_ADDR_BITS{1'b0}};
            np_has_ada_r             <= 1'b0;
            np_ada_base_addr_r       <= {MEM_ADDR_BITS{1'b0}};
            np_b_eff_base_addr_r     <= {MEM_ADDR_BITS{1'b0}};
            np_dcy_ada_base_addr_r   <= {MEM_ADDR_BITS{1'b0}};
            np_scl_ada_base_addr_r   <= {MEM_ADDR_BITS{1'b0}};
        end else if (sys_req_i & addr_match) begin
            case (sys_addr_i[7:0])
                // ---- PACKED per-task config (cfg_mem word i -> offset i*4) ----
                // Layout mirrored by tools regmap.PACKED_FMI_*: 12 address words,
                // 3 size words, 1 mode word = the full 16-word cfg_mem stride.
                // W0..W11: full-width base addresses (one per word)
                8'h00: sp_act_base_addr_r      <= sys_data_i[MEM_ADDR_BITS-1:0];
                8'h04: sp_weight_base_addr_r   <= sys_data_i[MEM_ADDR_BITS-1:0];
                8'h08: syn_curr_base_addr_r    <= sys_data_i[MEM_ADDR_BITS-1:0];
                8'h0C: np_thresh_base_addr_r   <= sys_data_i[MEM_ADDR_BITS-1:0];
                8'h10: np_pot_base_addr_r      <= sys_data_i[MEM_ADDR_BITS-1:0];
                8'h14: np_spike_base_addr_r    <= sys_data_i[MEM_ADDR_BITS-1:0];
                8'h18: np_dcy_syn_base_addr_r  <= sys_data_i[MEM_ADDR_BITS-1:0];
                8'h1C: np_dcy_mem_base_addr_r  <= sys_data_i[MEM_ADDR_BITS-1:0];
                8'h20: np_ada_base_addr_r      <= sys_data_i[MEM_ADDR_BITS-1:0];
                8'h24: np_b_eff_base_addr_r    <= sys_data_i[MEM_ADDR_BITS-1:0];
                8'h28: np_dcy_ada_base_addr_r  <= sys_data_i[MEM_ADDR_BITS-1:0];
                8'h2C: np_scl_ada_base_addr_r  <= sys_data_i[MEM_ADDR_BITS-1:0];
                // S0..S2: two 16-bit size lanes each (low [15:0], high [31:16])
                8'h30: begin                                       // S0
                    sp_in_x_len_r  <= sys_data_i[SP_X_INPUT_SZ-1:0];
                    sp_out_x_len_r <= sys_data_i[16 +: SP_X_OUTPUT_SZ];
                end
                8'h34: begin                                       // S1
                    sp_rows_per_neuron_r <= sys_data_i[SP_ROWS_PER_NEURON-1:0];
                    np_last_neuron_idx_r <= sys_data_i[16 +: NP_NEURON_IDX_SZ];
                end
                8'h38: sp_total_timesteps_r <= sys_data_i[SP_TIMESTEP_SZ-1:0]; // S2
                // M0: bit-packed mode / slice-size fields
                8'h3C: begin                                       // M0
                    sp_skip_neuron_r      <= sys_data_i[0];
                    np_mode_r             <= sys_data_i[4:2];
                    sp_weights_per_word_r <= sys_data_i[9:6];
                    bin_point_syn_curr_r  <= sys_data_i[14:10];
                    sp_weight_sz_r        <= sys_data_i[19:16];
                    np_syn_curr_sz_r      <= sys_data_i[23:20];
                    np_pot_sz_r           <= sys_data_i[27:24];
                    sp_weight_mode_r      <= sys_data_i[29:28];
                    np_has_ada_r          <= sys_data_i[30];
                    sp_real_mac_r         <= sys_data_i[31];   // reserved (MC-only datapath)
                end
                // ---- boot-only conv/sparse params (out-of-packed-window, >=0x5C) ----
                8'h5C: sp_weight_idx_sz_r      <= sys_data_i[SP_WEIGHT_IDX_SZ-1:0];
                8'h74: sp_x_kernel_len_r       <= sys_data_i[SP_X_KERNEL_SZ-1:0];
                8'h7C: sp_x_kernel_step_r      <= sys_data_i[SP_X_STEP_SZ-1:0];
                8'h84: sp_x_kernel_offset_r    <= sys_data_i[SP_X_KERNEL_OFF_SZ-1:0];
                8'h8C: sp_index_sz_r           <= sys_data_i[SP_WEIGHT_SLICE_SZ-1:0];
                8'h90: sp_tuple_sz_r           <= sys_data_i[SP_WEIGHT_SLICE_SZ-1:0];
                8'h94: sp_sparse_count_r       <= sys_data_i[`PIN_BITS-1:0];
                8'h98: sp_mac_shift_r          <= sys_data_i[5:0];   // reserved (MC-only datapath)
                default: ;
            endcase
        end
    end

    // =========================================================================
    // Synaptic-current memory arbiter (unchanged logic)
    // =========================================================================
    wire sp_syn_curr_mem_wr, sp_syn_curr_mem_rd;
    wire [`ADDR_SIZE-1:0] sp_syn_curr_mem_addr;
    wire [`POT_BITS-1:0]  sp_syn_curr_mem_data_wr;
    wire sp_syn_curr_mem_wait;

    wire np_syn_curr_mem_wr, np_syn_curr_mem_rd;
    wire [`ADDR_SIZE-1:0] np_syn_curr_mem_addr;
    wire [`POT_BITS-1:0]  np_syn_curr_mem_data_wr;
    wire np_syn_curr_mem_wait;

    wire np_req = np_syn_curr_mem_rd | np_syn_curr_mem_wr;
    wire sp_req = sp_syn_curr_mem_rd | sp_syn_curr_mem_wr;
    wire grant_np = np_req;
    wire grant_sp = sp_req & ~np_req;

    assign syn_curr_mem_wr_o   = grant_np ? np_syn_curr_mem_wr
                                          : (grant_sp ? sp_syn_curr_mem_wr  : 1'b0);
    assign syn_curr_mem_rd_o   = grant_np ? np_syn_curr_mem_rd
                                          : (grant_sp ? sp_syn_curr_mem_rd  : 1'b0);
    assign syn_curr_mem_addr_o = grant_np ? np_syn_curr_mem_addr : sp_syn_curr_mem_addr;
    assign syn_curr_mem_data_o = grant_np ? np_syn_curr_mem_data_wr : sp_syn_curr_mem_data_wr;

    assign np_syn_curr_mem_wait = (~grant_np & np_req) | (grant_np & syn_curr_mem_wait_i);
    assign sp_syn_curr_mem_wait = (~grant_sp & sp_req) | (grant_sp & syn_curr_mem_wait_i);

    // =========================================================================
    // Internal scheduler wires
    // =========================================================================
    wire sp_acc_busy, sp_acc_finished;
    wire np_neuron_proc_finished, np_acc_busy;
    wire np_acc_finished;

    assign acc_busy_o     = sp_acc_busy | np_acc_busy;
    assign acc_finished_o = sp_skip_neuron_r ? sp_acc_finished : np_acc_finished;

    //----------------------------------------------------------------
    // Latched dispatch target_acc
    // (See acc_snn_processor.v for rationale; same shape applied here
    // so fmiSnnAcc is ready for multi-acc top integration.)
    //----------------------------------------------------------------
    reg [`TGT_ACC_SZ-1:0] dispatched_target_acc_r;
    always @(posedge clk) begin
        if (reset)
            dispatched_target_acc_r <= TGT_ACC_ID;
        else if (start_new_block_i && (target_acc_i == TGT_ACC_ID))
            dispatched_target_acc_r <= target_acc_i;
    end
    wire my_dispatch = start_new_block_i & (target_acc_i == TGT_ACC_ID);

    // =========================================================================
    // spike_processing instantiation
    // Y-dimension parameters are hardwired to 1 (FMI model uses 1D only).
    // =========================================================================
    spike_processing # (
        .NUM_TIMESTEPS        (SP_NUM_TIMESTEPS),
        .X_INPUT_SZ           (SP_X_INPUT_SZ),
        .Y_INPUT_SZ           (1),
        .X_OUTPUT_SZ          (SP_X_OUTPUT_SZ),
        .Y_OUTPUT_SZ          (1),
        .X_KERNEL_SZ          (SP_X_KERNEL_SZ),
        .Y_KERNEL_SZ          (1),
        .X_KERNEL_OFF_SZ      (SP_X_KERNEL_OFF_SZ),
        .Y_KERNEL_OFF_SZ      (1),
        .X_STEP_SZ            (SP_X_STEP_SZ),
        .Y_STEP_SZ            (1),
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
        .act_base_addr_i        (sp_act_base_addr_r),
        .weight_base_addr_i     (sp_weight_base_addr_r),
        .syn_curr_base_addr_i   (syn_curr_base_addr_r),
        .weight_sz_i            (sp_weight_sz_r),
        .bin_point_syn_curr_i   (bin_point_syn_curr_r),
        .in_x_len_i             (sp_in_x_len_r),
        .in_y_len_i             (1'b1),
        .out_x_len_i            (sp_out_x_len_r),
        .out_y_len_i            (1'b1),
        .weights_per_word_i     (sp_weights_per_word_r),
        .rows_per_neuron_i      (sp_rows_per_neuron_r),
        .weight_idx_sz_i        (sp_weight_idx_sz_r),
        .weight_mode_i          (sp_weight_mode_r),
        .x_kernel_len_i         (sp_x_kernel_len_r),
        .y_kernel_len_i         (1'b1),
        .x_kernel_offset_i      (sp_x_kernel_offset_r),
        .y_kernel_offset_i      (1'b0),
        .x_kernel_step_i        (sp_x_kernel_step_r),
        .y_kernel_step_i        (1'b1),
        .index_sz_i             (sp_index_sz_r),
        .tuple_sz_i             (sp_tuple_sz_r),
        .sparse_count_i         (sp_sparse_count_r),
        .start_new_block_i      (my_dispatch),
        .target_acc_i           (dispatched_target_acc_r),
        .buffer_info_i          (buffer_info_i),
        .spike_proc_finished_o  (spike_proc_finished_o),
        .acc_busy_o             (sp_acc_busy),
        .acc_finished_o         (sp_acc_finished),
        .src1_buff_addr_i       (sp_src1_buff_addr_i),
        .src2_buff_addr_i       (sp_src2_buff_addr_i),
        .src3_buff_addr_i       (sp_src3_buff_addr_i),
        .tgt_buff_addr_i        (sp_tgt_buff_addr_i),
        .weight_row_len_i       (sp_weight_row_len_i),
        .weight_mem_rd_o        (weight_mem_rd_o),
        .weight_mem_wait_i      (weight_mem_wait_i),
        .weight_mem_addr_o      (weight_mem_addr_o),
        .weight_mem_data_i      (weight_mem_data_i),
        .act_mem_req_o          (act_mem_req_o),
        .act_mem_wait_i         (act_mem_wait_i),
        .act_mem_addr_o         (act_mem_addr_o),
        .act_mem_data_i         (act_mem_data_i),
        .syn_curr_mem_wr_o      (sp_syn_curr_mem_wr),
        .syn_curr_mem_rd_o      (sp_syn_curr_mem_rd),
        .syn_curr_mem_wait_i    (sp_syn_curr_mem_wait),
        .syn_curr_mem_addr_o    (sp_syn_curr_mem_addr),
        .syn_curr_mem_data_wr_o (sp_syn_curr_mem_data_wr),
        .syn_curr_mem_data_rd_i (syn_curr_mem_data_i)
    );

    // =========================================================================
    // neuron_processing instantiation (FMI variant)
    // =========================================================================
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
        .POT_IDX_SZ           (NP_POT_IDX_SZ),
        .POT_DATA_IDX_SZ      (NP_POT_DATA_IDX_SZ),
        .POT_SLICE_SZ         (NP_POT_SLICE_SZ),
        .POT_SLICE_BITS       (NP_POT_SLICE_BITS),
        .SPIKE_IDX_SZ         (NP_SPIKE_IDX_SZ),
        .SPIKE_DATA_IDX_SZ    (NP_SPIKE_DATA_IDX_SZ),
        .SPIKE_SLICE_SZ       (NP_SPIKE_SLICE_SZ),
        .SPIKE_SLICE_BITS     (NP_SPIKE_SLICE_BITS),
        .MEM_ADDR_BITS        (MEM_ADDR_BITS)
    ) u_neuron_processing (
        .clk                    (clk),
        .reset                  (reset),
        .last_neuron_idx_i      (np_last_neuron_idx_r),
        .syn_curr_base_addr_i   (syn_curr_base_addr_r),
        .thresh_base_addr_i     (np_thresh_base_addr_r),
        .pot_base_addr_i        (np_pot_base_addr_r),
        .spike_base_addr_i      (np_spike_base_addr_r),
        .syn_curr_sz_i          (np_syn_curr_sz_r),
        .pot_sz_i               (np_pot_sz_r),
        .bin_point_syn_curr_i   (bin_point_syn_curr_r),
        .sub_on_fire_i          (np_mode_r[0]),
        .clear_pot_i            (np_mode_r[2]),
        .dcy_syn_base_addr_i    (np_dcy_syn_base_addr_r),
        .dcy_mem_base_addr_i    (np_dcy_mem_base_addr_r),
        .has_ada_i              (np_has_ada_r),
        .ada_base_addr_i        (np_ada_base_addr_r),
        .b_eff_base_addr_i      (np_b_eff_base_addr_r),
        .dcy_ada_base_addr_i    (np_dcy_ada_base_addr_r),
        .scl_ada_base_addr_i    (np_scl_ada_base_addr_r),
        .start_new_block_i      (sp_acc_finished & ~sp_skip_neuron_r),
        .target_acc_i           (dispatched_target_acc_r),
        .buffer_info_i          (buffer_info_i),
        .neuron_proc_finished_o (np_neuron_proc_finished),
        .acc_busy_o             (np_acc_busy),
        .acc_finished_o         (np_acc_finished),
        .src1_buff_addr_i       (np_src1_buff_addr_i),
        .src2_buff_addr_i       (np_src2_buff_addr_i),
        .src3_buff_addr_i       (np_src3_buff_addr_i),
        .tgt_buff_addr_i        (np_tgt_buff_addr_i),
        .weight_row_len_i       (np_weight_row_len_i),
        .syn_curr_mem_wr_o      (np_syn_curr_mem_wr),
        .syn_curr_mem_rd_o      (np_syn_curr_mem_rd),
        .syn_curr_mem_wait_i    (np_syn_curr_mem_wait),
        .syn_curr_mem_addr_o    (np_syn_curr_mem_addr),
        .syn_curr_mem_data_o    (np_syn_curr_mem_data_wr),
        .syn_curr_mem_data_i    (syn_curr_mem_data_i),
        .thresh_mem_rd_o        (thresh_mem_rd_o),
        .thresh_mem_wait_i      (thresh_mem_wait_i),
        .thresh_mem_addr_o      (thresh_mem_addr_o),
        .thresh_mem_data_i      (thresh_mem_data_i),
        .pot_mem_wr_o           (pot_mem_wr_o),
        .pot_mem_rd_o           (pot_mem_rd_o),
        .pot_mem_wait_i         (pot_mem_wait_i),
        .pot_mem_addr_o         (pot_mem_addr_o),
        .pot_mem_data_o         (pot_mem_data_o),
        .pot_mem_data_i         (pot_mem_data_i),
        .spike_mem_wr_o         (spike_mem_wr_o),
        .spike_mem_wait_i       (spike_mem_wait_i),
        .spike_mem_addr_o       (spike_mem_addr_o),
        .spike_mem_data_o       (spike_mem_data_o),
        .dcy_syn_mem_rd_o       (dcy_syn_mem_rd_o),
        .dcy_syn_mem_wait_i     (dcy_syn_mem_wait_i),
        .dcy_syn_mem_addr_o     (dcy_syn_mem_addr_o),
        .dcy_syn_mem_data_i     (dcy_syn_mem_data_i),
        .dcy_mem_mem_rd_o       (dcy_mem_mem_rd_o),
        .dcy_mem_mem_wait_i     (dcy_mem_mem_wait_i),
        .dcy_mem_mem_addr_o     (dcy_mem_mem_addr_o),
        .dcy_mem_mem_data_i     (dcy_mem_mem_data_i),
        .ada_mem_wr_o           (ada_mem_wr_o),
        .ada_mem_rd_o           (ada_mem_rd_o),
        .ada_mem_wait_i         (ada_mem_wait_i),
        .ada_mem_addr_o         (ada_mem_addr_o),
        .ada_mem_data_o         (ada_mem_data_o),
        .ada_mem_data_i         (ada_mem_data_i),
        .b_eff_mem_rd_o         (b_eff_mem_rd_o),
        .b_eff_mem_wait_i       (b_eff_mem_wait_i),
        .b_eff_mem_addr_o       (b_eff_mem_addr_o),
        .b_eff_mem_data_i       (b_eff_mem_data_i),
        .dcy_ada_mem_rd_o       (dcy_ada_mem_rd_o),
        .dcy_ada_mem_wait_i     (dcy_ada_mem_wait_i),
        .dcy_ada_mem_addr_o     (dcy_ada_mem_addr_o),
        .dcy_ada_mem_data_i     (dcy_ada_mem_data_i),
        .scl_ada_mem_rd_o       (scl_ada_mem_rd_o),
        .scl_ada_mem_wait_i     (scl_ada_mem_wait_i),
        .scl_ada_mem_addr_o     (scl_ada_mem_addr_o),
        .scl_ada_mem_data_i     (scl_ada_mem_data_i)
    );

endmodule
