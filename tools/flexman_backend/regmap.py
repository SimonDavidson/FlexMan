# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Simon Davidson, University of Manchester
# =============================================================================
# flexman_backend.regmap — FlexMan AXI register maps & address helpers
#
# Author: Simon Davidson & Claude
# Created: 2026-06-08
# Last modified: 2026-06-17
#
# Pure-Python (no torch). These maps mirror the per-accelerator AXI config-reg
# decoders in the RTL and the scheduler's fixed control/memory address decode.
# They are HARDWARE-DEFINED — kept here, versioned with the RTL, and cross-checked
# by tools/tests/test_regmap_vs_rtl.py.
#
# Sources:
#   PACKED_* / BOOT_REG_OFFSETS : snnAcc / annAcc / ipSnnAcc acc_snn_processor.v
#   PACKED_FMI_*                : fmiSnnAcc/acc_fmiSnn_processor.v
#   HU_REG_OFFSETS              : Hadamard/hu_config_regs.v
#   scheduler control/mem addrs : scheduler/ISA_REFERENCE.md
#
# NOTE: per-accelerator config BASE addresses (which instance sits at which AXI
# base) are TOP-MODULE / deployment specific, so they live in each front-end's
# memory_map, NOT here.
# =============================================================================
from __future__ import annotations

# ---------------------------------------------------------------------------
# PACKED per-task config layout (single source of truth, mirrors the RTL decode).
#
# The config_manager streams PACKED_WORDS_PER_CONFIG words to a task on dispatch;
# word i lands at accelerator AXI offset i*4. The full per-task config is packed
# into 15 words so a multi-layer accelerator (e.g. snnAcc running recurrent
# pass1/pass2/readout) gets its own in_x/out_x/rows_per_neuron/modes per task.
#
# Convention: SIZE fields use 16-bit lanes (two per word -> up to 65536); MODE /
# slice-size fields are bit-packed with 1-2 spare bits each for future modes.
# A field absent from a given accelerator's cfg dict simply packs as 0.
#
#   words 0..8  : full-32-bit addresses + decay multipliers (one per word)
#   words 9..12 : S0..S3  -> two 16-bit size lanes  (low=[15:0], high=[31:16])
#   words 13,14 : M0,M1   -> packed mode/slice-size fields (field, lsb, width)
# ---------------------------------------------------------------------------
PACKED_ADDR_WORDS = [          # offsets 0x00..0x20
    "sp_act_base_addr",        # 0x00
    "sp_weight_base_addr",     # 0x04
    "syn_curr_base_addr",      # 0x08
    "np_bias_curr_base_addr",  # 0x0C
    "np_thresh_base_addr",     # 0x10  (= lut base on annAcc)
    "np_pot_base_addr",        # 0x14
    "np_spike_base_addr",      # 0x18
    "np_syn_curr_decay_mult",  # 0x1C  (0 on annAcc)
    "np_pot_decay_mult",       # 0x20
]
PACKED_SIZE_WORDS = [          # offsets 0x24..0x30 ; (low_field, high_field)
    ("sp_in_x_len",        "sp_in_y_len"),         # S0 0x24
    ("sp_out_x_len",       "sp_out_y_len"),        # S1 0x28
    ("sp_rows_per_neuron", "np_last_neuron_idx"),  # S2 0x2C
    ("sp_total_timesteps", None),                  # S3 0x30 (high lane spare)
]
PACKED_MODE_WORDS = [          # offsets 0x34..0x38 ; (field, lsb, width)
    [   # M0 0x34
        ("sp_weight_sz",    0, 4),
        ("np_syn_curr_sz",  4, 4),
        ("np_bias_curr_sz", 8, 4),
        ("np_pot_sz",      12, 4),
        ("sp_act_sz",      16, 4),
        ("np_thresh_op",   20, 4),
        ("sp_weight_mode", 24, 4),
        ("act_signed",     28, 1),   # §5.4 signed-activation MAC (annAcc only; default 0)
        ("np_lut_window",  29, 1),   # §5.1 windowed/saturated LUT index (annAcc only; default 0)
        ("np_out_signed",  30, 1),   # §5.2 signed LUT (tanh) output (annAcc only; default 0)
        ("np_bias_en",     31, 1),   # §5.5 per-neuron bias accumulator-seed (annAcc only; default 0)
    ],
    [   # M1 0x38
        ("sp_skip_neuron",      0, 2),
        ("np_mode",             2, 4),
        ("sp_weights_per_word", 6, 4),
        ("bin_point_syn_curr", 10, 6),
        ("np_out_bin_point",   16, 6),
        ("bias_bin_point",     22, 6),   # §5.5 bias binary point (annAcc only; default 0)
    ],
]
PACKED_WORDS_PER_CONFIG = (len(PACKED_ADDR_WORDS)
                           + len(PACKED_SIZE_WORDS)
                           + len(PACKED_MODE_WORDS))   # = 15

# ---------------------------------------------------------------------------
# PACKED per-task config layout for fmiSnnAcc (mirrors fmiSnnAcc/acc_fmiSnn_processor.v).
#
# fmiSnnAcc replaces the scalar decay multipliers and bias_curr memory with six
# per-neuron adaptive-LIF memories, so its packed layout differs from the snn/ann
# one: 12 full-width base addresses, 3 size words, 1 mode word = 16 words, exactly
# the cfg_mem stride (no spare word). fmi is 1-D (Y hardwired to 1): no y lengths.
#
#   words 0..11  : offsets 0x00..0x2C, full-32-bit base addresses (one per word)
#   words 12..14 : S0..S2 -> two 16-bit size lanes (low=[15:0], high=[31:16])
#   word  15     : M0     -> packed mode/slice-size fields (field, lsb, width)
# ---------------------------------------------------------------------------
PACKED_FMI_ADDR_WORDS = [      # offsets 0x00..0x2C
    "sp_act_base_addr",        # 0x00
    "sp_weight_base_addr",     # 0x04
    "syn_curr_base_addr",      # 0x08
    "np_thresh_base_addr",     # 0x0C
    "np_pot_base_addr",        # 0x10
    "np_spike_base_addr",      # 0x14
    "np_dcy_syn_base_addr",    # 0x18  per-neuron syn-current decay memory
    "np_dcy_mem_base_addr",    # 0x1C  per-neuron membrane decay memory
    "np_ada_base_addr",        # 0x20  adaptive-threshold state memory
    "np_b_eff_base_addr",      # 0x24  effective-threshold memory
    "np_dcy_ada_base_addr",    # 0x28  adaptation decay memory
    "np_scl_ada_base_addr",    # 0x2C  adaptation scale memory
]
PACKED_FMI_SIZE_WORDS = [      # offsets 0x30..0x38 ; (low_field, high_field)
    ("sp_in_x_len",        "sp_out_x_len"),        # S0 0x30
    ("sp_rows_per_neuron", "np_last_neuron_idx"),  # S1 0x34
    ("sp_total_timesteps", None),                  # S2 0x38: low [9:0] tt; +stride[12:10], +cin[22:16]/cout[30:24] (MC)
]
# NOTE (fmiSnnAccMC only): the MULTI-CHANNEL variant additionally decodes
# sp_x_kernel_step (stride) at S2 bits [12:10] PER-CONFIG — so con2/con3/con4/con5
# can alternate stride 2/1 within one program; the MC RTL drops the 0x7C boot-reg
# decode that the single-channel fmiSnnAcc keeps. It also decodes
# sp_cin_len at S2 bit 16 (+:SP_CIN_SZ) and sp_cout_len at S2 bit 24
# (+:SP_COUT_SZ) — i.e. the "spare" lanes of word S2 (0x38) carry
# {cout[14:8], cin[6:0]} when packed as a 16-bit high field. The single-channel
# fmiSnnAcc (which this PACKED_FMI_* layout is cross-checked against in
# tools/tests/test_regmap_vs_rtl.py) leaves it spare, so it is NOT added to the
# (low,high) tuple above. Encoding it requires either a dedicated MC size-word
# list or a packer that supports two sub-fields per lane — deferred to the MC
# config generator (FMI bring-up Stage C / convert_model).
PACKED_FMI_MODE_WORDS = [      # offset 0x3C ; (field, lsb, width)
    [   # M0 0x3C
        ("sp_skip_neuron",      0, 2),
        ("np_mode",             2, 4),
        ("sp_weights_per_word", 6, 4),
        ("bin_point_syn_curr", 10, 6),
        ("sp_weight_sz",       16, 4),
        ("np_syn_curr_sz",     20, 4),
        ("np_pot_sz",          24, 4),
        ("sp_weight_mode",     28, 2),
        ("np_has_ada",         30, 1),
        ("sp_real_mac",        31, 1),   # real-valued act MAC (MC con1 input layer)
    ],
]
PACKED_FMI_WORDS_PER_CONFIG = (len(PACKED_FMI_ADDR_WORDS)
                               + len(PACKED_FMI_SIZE_WORDS)
                               + len(PACKED_FMI_MODE_WORDS))   # = 16

# Boot-only (out-of-packed-window) registers, set once via direct AXI for the
# rare conv/sparse layers. Not per-task; dense applications never write these.
BOOT_REG_OFFSETS = {
    "sp_weight_idx_sz":   0x5C,
    "sp_x_kernel_len":    0x74,
    "sp_y_kernel_len":    0x78,
    "sp_x_kernel_step":   0x7C,
    "sp_y_kernel_step":   0x80,
    "sp_x_kernel_offset": 0x84,
    "sp_y_kernel_offset": 0x88,
    "sp_index_sz":        0x8C,
    "sp_tuple_sz":        0x90,
    "sp_sparse_count":    0x94,
}

# fmiSnnAcc boot-only registers: same offsets as BOOT_REG_OFFSETS minus the
# y-kernel params (fmi is 1-D).
BOOT_REG_OFFSETS_FMI = {
    "sp_weight_idx_sz":   0x5C,
    "sp_x_kernel_len":    0x74,
    "sp_x_kernel_step":   0x7C,   # single-channel fmiSnnAcc only; the MC variant
                                  # decodes stride PER-CONFIG in S2[12:10] instead
    "sp_x_kernel_offset": 0x84,
    "sp_index_sz":        0x8C,
    "sp_tuple_sz":        0x90,
    "sp_sparse_count":    0x94,
    "sp_mac_shift":       0x98,   # real-MAC product shift (= K_MEM); MC con1 input layer
}

# Hadamard config-register offsets (Hadamard/hu_config_regs.v reg_sel = addr[7:0]).
# Computes R = Z*(A-B) + B + mode*R_prev over a stream of `stream_len` elements.
# ONE 32-bit register per word (byte offset = word_index*4), so the config_manager
# per-task stream (word i -> byte i*4) delivers all 14 fields in order — i.e. the
# packed cfg_mem block is just word[i] = field[i] (see cfgmem.pack_hu_cfg_words).
HU_REG_OFFSETS = {
    "mode":             0x00,   # w0  [0]: 1 = update (+R_prev), 0 = init
    "stream_len":       0x04,   # w1  number of vector elements
    "src_a_base_addr":  0x08,   # w2
    "src_a_elem_sz":    0x0C,   # w3
    "src_a_bin_point":  0x10,   # w4
    "src_b_base_addr":  0x14,   # w5
    "src_b_elem_sz":    0x18,   # w6
    "src_b_bin_point":  0x1C,   # w7
    "src_z_base_addr":  0x20,   # w8
    "src_z_elem_sz":    0x24,   # w9
    "src_z_bin_point":  0x28,   # w10
    "src_r_base_addr":  0x2C,   # w11
    "src_r_elem_sz":    0x30,   # w12
    "src_r_bin_point":  0x34,   # w13
}

# ---------------------------------------------------------------------------
# np_thresh_op encodings (annAcc activation function select)
# ---------------------------------------------------------------------------
THRESH_OP_RELU = 0b00
THRESH_OP_LUT  = 0b01
THRESH_OP_ABS  = 0b10

# ---------------------------------------------------------------------------
# Slice-size encoding (slice_sz / elem_sz fields), per dataline_cache_with_xy
# ---------------------------------------------------------------------------
SLICE_SZ_1BIT  = 0b000
SLICE_SZ_2BIT  = 0b001
SLICE_SZ_4BIT  = 0b010
SLICE_SZ_8BIT  = 0b011
SLICE_SZ_16BIT = 0b100
SLICE_SZ_32BIT = 0b101

# Map element bit-width -> slice_sz encoding (for the packers / configs).
SLICE_SZ_FOR_BITS = {1: 0b000, 2: 0b001, 4: 0b010, 8: 0b011, 16: 0b100, 32: 0b101}

# ---------------------------------------------------------------------------
# fill_unit mem_sel one-hot bit positions (mirror fillUnit/fill_unit.v).
# act / spike / syn_curr route through IDX_SHARED_DATA; dedicated kinds keep
# their own bit.
# ---------------------------------------------------------------------------
IDX_S0_WEIGHT    =  0
IDX_S0_BIAS_CURR =  3
IDX_S0_THRESH    =  4
IDX_S0_POT       =  5
IDX_S1_WEIGHT    =  7
IDX_S1_BIAS_CURR = 10
IDX_S1_THRESH    = 11
IDX_S1_POT       = 12
IDX_A0_WEIGHT    = 14
IDX_A0_BIAS_CURR = 17
IDX_A0_THRESH    = 18
IDX_A0_POT       = 19
IDX_SHARED_DATA  = 25

# ---------------------------------------------------------------------------
# Scheduler control / memory AXI base addresses (fixed by the scheduler RTL).
# ---------------------------------------------------------------------------
LOAD_PC_ADDR        = 0xE0000000
START_ADDR          = 0xE0100000
CONTINUE_ADDR       = 0xE0200000
PAUSE_ADDR          = 0xE0300000
UNPAUSE_ADDR        = 0xE0400000
MARK_BUFF_FULL_ADDR = 0xE0500000
CFG_MEM_BASE        = 0xA0000000   # config_manager cfg_mem
BBA_MEM_BASE        = 0xB0000000   # config_manager bba_mem
FU_TABLE_BASE       = 0xC0000000   # fill_unit mem_sel table
PROG_MEM_BASE       = 0xD0000000   # scheduler program memory


# ---------------------------------------------------------------------------
# AXI address helpers
# ---------------------------------------------------------------------------
def axi_reg_addr(cfg_base: int, reg_offset: int) -> int:
    """Compose an AXI write address for a per-accelerator config register."""
    return (cfg_base & 0xFFFF0000) | (reg_offset & 0x000000FF)


def bba_addr(buf_id: int) -> int:
    """AXI write address for setting bba_mem[buf_id]."""
    return BBA_MEM_BASE | (buf_id << 2)


def fu_table_addr(buf_id: int) -> int:
    """AXI write address for setting fill_unit mem_sel_table[buf_id]."""
    return FU_TABLE_BASE | (buf_id << 2)
