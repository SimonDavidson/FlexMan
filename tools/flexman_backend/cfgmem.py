# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Simon Davidson, University of Manchester
# =============================================================================
# flexman_backend.cfgmem — config_manager cfg_mem packing
#
# Author: Simon Davidson & Claude
# Created: 2026-06-08
# Last modified: 2026-06-17
#
# Pure-Python (no torch). The config_manager streams WORDS_PER_CONFIG (=15) words
# from cfg_mem[cfg_id*WPC ..] to a target accelerator on every TASK dispatch. It
# carries NO register address — the application top's glue (e.g. cfg_cnt_N)
# forms the accelerator address as CFG_BASE | (word_index << 2). So cfg_mem word i
# lands at the accelerator register at byte-offset i*4 (offsets 0x00..0x38).
#
# The full per-task config is delivered in a PACKED layout (regmap.PACKED_*),
# mirrored by the RTL decode, so a multi-layer accelerator (snnAcc running
# recurrent pass1/pass2/readout) gets its own in_x/out_x/rows_per_neuron/modes per
# task. Words 0-8 are full-width addresses + decay mults; S0-S3 carry two 16-bit
# size lanes each; M0-M1 bit-pack the mode/slice-size fields (headroom per field).
# Conv/sparse params (>=0x3C) stay boot-only for the rare conv layer.
# =============================================================================
from __future__ import annotations

from . import regmap

# cfg_mem stride = the RTL config_manager WORDS_PER_CONFIG, which must be a power
# of 2 (it forms the read address as {config_id, word_cnt}). The packed layout uses
# 15 words (regmap.PACKED_WORDS_PER_CONFIG), so the stride is 16 with one spare word
# (word 15 / offset 0x3C, unused -> future headroom).
WORDS_PER_CONFIG = 16


def pack_cfg_words(cfg: dict, acc_type: str = "snn") -> list[int]:
    """Return the packed cfg_mem words for one cfg_id from a {reg_name: value} dict.

    Layout is `regmap.PACKED_*` (the single source of truth, mirrored by the RTL
    decode): 9 full-width address/decay words, then S0-S3 (two 16-bit size lanes
    each), then M0-M1 (bit-packed mode/slice-size fields).

    `acc_type` selects the layout: {"snn","ipsnn","ann"} share `regmap.PACKED_*`
    (a field absent from `cfg` packs as 0, so annAcc leaves np_mode/syn_curr_decay
    at 0 and snnAcc/ipSnnAcc leave act_sz/thresh_op at 0); "fmisnn" uses
    `regmap.PACKED_FMI_*` (12 addr words incl. the six adaptive-LIF memory bases,
    3 size words, 1 mode word = a full 16-word stride).
    Keys not in the packed layout (conv/sparse boot regs) are ignored.
    """
    if acc_type in ("snn", "ipsnn", "ann"):
        addr_words = regmap.PACKED_ADDR_WORDS
        size_words = regmap.PACKED_SIZE_WORDS
        mode_words = regmap.PACKED_MODE_WORDS
    elif acc_type == "fmisnn":
        addr_words = regmap.PACKED_FMI_ADDR_WORDS
        size_words = regmap.PACKED_FMI_SIZE_WORDS
        mode_words = regmap.PACKED_FMI_MODE_WORDS
    else:
        raise ValueError(f"unknown acc_type {acc_type!r}")
    words = []
    # full-width address (+ decay multiplier) words, one per word
    for name in addr_words:
        words.append(cfg.get(name, 0) & 0xFFFFFFFF)
    # size words: two 16-bit lanes each (low [15:0], high [31:16])
    for low_field, high_field in size_words:
        low  = cfg.get(low_field, 0) & 0xFFFF
        high = (cfg.get(high_field, 0) & 0xFFFF) if high_field else 0
        words.append(low | (high << 16))
    # packed mode / slice-size words
    for fields in mode_words:
        word = 0
        for name, lsb, width in fields:
            word |= (cfg.get(name, 0) & ((1 << width) - 1)) << lsb
        words.append(word & 0xFFFFFFFF)
    # pad to the power-of-2 cfg_mem stride (snn/ann/ipsnn: word 15 / offset 0x3C
    # is spare; fmisnn fills all 16 words)
    words += [0] * (WORDS_PER_CONFIG - len(words))
    return words


def pack_hu_cfg_words(cfg: dict) -> list[int]:
    """Return the packed cfg_mem words for one Hadamard cfg_id.

    The Hadamard register map is ONE 32-bit register per word: each field's byte
    offset in `regmap.HU_REG_OFFSETS` is `word_index * 4`, matching the
    config_manager stream (word i -> byte i*4). So the packed block is simply
    word[offset>>2] = field value; fields absent from `cfg` pack as 0 (the reset
    value, e.g. mode=0). Used for Hadamard TASKs (R = Z*(A-B)+B+mode*R_prev).
    """
    words = [0] * WORDS_PER_CONFIG
    for name, off in regmap.HU_REG_OFFSETS.items():
        words[off >> 2] = cfg.get(name, 0) & 0xFFFFFFFF
    return words


def cfgmem_axi_writes(cfg_id: int, words, cfg_mem_base: int = regmap.CFG_MEM_BASE):
    """Map a config's 16 words to (axi_addr, data) writes into cfg_mem.

    cfg_mem is word-indexed; word K = cfg_id*WORDS_PER_CONFIG + i, AXI byte
    address = cfg_mem_base | (K << 2).
    """
    base = cfg_id * WORDS_PER_CONFIG
    return [(cfg_mem_base | ((base + i) << 2), w & 0xFFFFFFFF)
            for i, w in enumerate(words)]
