# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Simon Davidson, University of Manchester
# =============================================================================
# flexman_backend.regmap — FlexMan AXI register maps & address helpers
#
# Author: Simon Davidson & Claude
# Created: 2026-06-08
# Last modified: 2026-06-08
#
# Pure-Python (no torch). These maps mirror the per-accelerator AXI config-reg
# decoders in the RTL and the scheduler's fixed control/memory address decode.
# They are HARDWARE-DEFINED — kept here, versioned with the RTL, and cross-checked
# by tools/tests/test_regmap_vs_rtl.py.
#
# Sources:
#   SNN_REG_OFFSETS / ANN_REG_OFFSETS : snnAcc/CLAUDE.md, acc_snn_processor.v
#   HU_REG_OFFSETS                    : Hadamard/hu_config_regs.v
#   scheduler control/mem addrs       : scheduler/ISA_REFERENCE.md
#
# NOTE: per-accelerator config BASE addresses (which instance sits at which AXI
# base) are TOP-MODULE / deployment specific, so they live in each front-end's
# memory_map, NOT here.
# =============================================================================
from __future__ import annotations

# ---------------------------------------------------------------------------
# Per-accelerator config-register offsets (low byte of the AXI address)
# Common shape across snnAcc and ipSnnAcc:
# ---------------------------------------------------------------------------
SNN_REG_OFFSETS = {
    "sp_act_base_addr":        0x00,
    "sp_weight_base_addr":     0x04,
    "syn_curr_base_addr":      0x08,    # shared sp + np
    "sp_weight_sz":            0x0C,
    "sp_total_timesteps":      0x14,
    "np_last_neuron_idx":      0x20,
    "np_bias_curr_base_addr":  0x28,
    "np_thresh_base_addr":     0x2C,
    "np_pot_base_addr":        0x30,
    "np_syn_curr_sz":          0x34,
    "np_bias_curr_sz":         0x38,
    "np_pot_sz":               0x3C,
    "bin_point_syn_curr":      0x40,
    "sp_in_x_len":             0x44,
    "sp_in_y_len":             0x48,
    "sp_out_x_len":            0x4C,
    "sp_out_y_len":            0x50,
    "sp_weights_per_word":     0x54,
    "sp_rows_per_neuron":      0x58,
    "sp_weight_idx_sz":        0x5C,
    "np_spike_base_addr":      0x64,
    "np_syn_curr_decay_mult":  0x68,
    "np_pot_decay_mult":       0x6C,
    "sp_weight_mode":          0x70,    # 00=full, 01=sparse, 10=conv
    "sp_x_kernel_len":         0x74,
    "sp_y_kernel_len":         0x78,
    "sp_x_kernel_step":        0x7C,
    "sp_y_kernel_step":        0x80,
    "sp_x_kernel_offset":      0x84,
    "sp_y_kernel_offset":      0x88,
    "sp_index_sz":             0x8C,
    "sp_tuple_sz":             0x90,
    "sp_sparse_count":         0x94,
    "np_mode":                 0x98,    # [0]=sub_on_fire [1]=clear_syn [2]=clear_pot
    "sp_skip_neuron":          0x9C,
}

# annAcc differs at 0x98 (sp_act_sz instead of np_mode) and adds 0xA0, 0xA4;
# it has no np_mode and no syn-curr decay.
ANN_REG_OFFSETS = dict(SNN_REG_OFFSETS)
ANN_REG_OFFSETS["sp_act_sz"]        = 0x98   # overrides np_mode (annAcc-specific)
ANN_REG_OFFSETS["np_thresh_op"]     = 0xA0   # 00=RELU 01=LUT 10=ABS
ANN_REG_OFFSETS["np_out_bin_point"] = 0xA4   # output requant; shift = bin_point_syn_curr - this
del ANN_REG_OFFSETS["np_mode"]
del ANN_REG_OFFSETS["np_syn_curr_decay_mult"]

# Hadamard config-register offsets (Hadamard/hu_config_regs.v reg_sel = addr[7:0]).
# Computes R = Z*(A-B) + B + mode*R_prev over a stream of `stream_len` elements.
HU_REG_OFFSETS = {
    "mode":             0x00,   # [0]: 1 = update (+R_prev), 0 = init
    "stream_len":       0x01,   # number of vector elements
    "src_a_base_addr":  0x04,
    "src_a_elem_sz":    0x05,
    "src_a_bin_point":  0x06,
    "src_b_base_addr":  0x08,
    "src_b_elem_sz":    0x09,
    "src_b_bin_point":  0x0A,
    "src_z_base_addr":  0x0C,
    "src_z_elem_sz":    0x0D,
    "src_z_bin_point":  0x0E,
    "src_r_base_addr":  0x10,
    "src_r_elem_sz":    0x11,
    "src_r_bin_point":  0x12,
}

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
