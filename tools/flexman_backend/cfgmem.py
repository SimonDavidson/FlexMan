# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Simon Davidson, University of Manchester
# =============================================================================
# flexman_backend.cfgmem — config_manager cfg_mem packing
#
# Author: Simon Davidson & Claude
# Created: 2026-06-08
# Last modified: 2026-06-08
#
# Pure-Python (no torch). The config_manager streams WORDS_PER_CONFIG (=16) words
# from cfg_mem[cfg_id*WPC ..] to a target accelerator on every TASK dispatch. It
# carries NO register address — the top's glue (siren_detector_top.v: cfg_cnt_N)
# forms the accelerator address as CFG_BASE | (word_index << 2). So cfg_mem word i
# lands at the accelerator register at byte-offset i*4 (offsets 0x00..0x3C only).
#
# Registers at offset >= 0x40 are NOT per-task reconfigurable; they are set once
# via direct AXI boot config and held. The per-task registers the deployments vary
# are reachable because the RTL aliases them INTO the window:
#   word 4 (0x10): snn/ipsnn -> {np_mode[3:1], sp_skip_neuron[0]};  ann -> skip[0]
#   word 6 (0x18): snn/ipsnn -> np_syn_curr_decay_mult              ann -> (unused)
#   word 7 (0x1C): np_pot_decay_mult (alias of 0x6C)
# (Verified against snnAcc/ipsnn/annAcc acc_*_processor.v config decoders.)
# =============================================================================
from __future__ import annotations

from . import regmap

WORDS_PER_CONFIG = 16            # siren_detector_top.v parameter

# Sentinel for the packed control word (word 4).
_PACK = object()

# word index -> register name in the config dict (None = unused/zero).
_LAYOUT_SNN = [
    "sp_act_base_addr", "sp_weight_base_addr", "syn_curr_base_addr", "sp_weight_sz",
    _PACK, "sp_total_timesteps", "np_syn_curr_decay_mult", "np_pot_decay_mult",
    "np_last_neuron_idx", None, "np_bias_curr_base_addr", "np_thresh_base_addr",
    "np_pot_base_addr", "np_syn_curr_sz", "np_bias_curr_sz", "np_pot_sz",
]
# annAcc: word4 carries skip_neuron only (no np_mode); word6 unused (no syn decay).
_LAYOUT_ANN = [
    "sp_act_base_addr", "sp_weight_base_addr", "syn_curr_base_addr", "sp_weight_sz",
    "sp_skip_neuron", "sp_total_timesteps", None, "np_pot_decay_mult",
    "np_last_neuron_idx", None, "np_bias_curr_base_addr", "np_thresh_base_addr",
    "np_pot_base_addr", "np_syn_curr_sz", "np_bias_curr_sz", "np_pot_sz",
]

_LAYOUTS = {"snn": _LAYOUT_SNN, "ipsnn": _LAYOUT_SNN, "ann": _LAYOUT_ANN}


def pack_cfg_words(cfg: dict, acc_type: str) -> list[int]:
    """Return the 16 cfg_mem words for one cfg_id from a {reg_name: value} dict.

    `acc_type` in {"snn", "ipsnn", "ann"}. Registers absent from `cfg` default to
    0; out-of-window registers in `cfg` are ignored (they are not in cfg_mem and
    must be delivered via direct boot config instead).
    """
    if acc_type not in _LAYOUTS:
        raise ValueError(f"unknown acc_type {acc_type!r}")
    words = []
    for slot in _LAYOUTS[acc_type]:
        if slot is None:
            words.append(0)
        elif slot is _PACK:
            words.append((((cfg.get("np_mode", 0) & 0x7) << 1)
                          | (cfg.get("sp_skip_neuron", 0) & 0x1)))
        else:
            words.append(cfg.get(slot, 0) & 0xFFFFFFFF)
    return words


def cfgmem_axi_writes(cfg_id: int, words, cfg_mem_base: int = regmap.CFG_MEM_BASE):
    """Map a config's 16 words to (axi_addr, data) writes into cfg_mem.

    cfg_mem is word-indexed; word K = cfg_id*WORDS_PER_CONFIG + i, AXI byte
    address = cfg_mem_base | (K << 2).
    """
    base = cfg_id * WORDS_PER_CONFIG
    return [(cfg_mem_base | ((base + i) << 2), w & 0xFFFFFFFF)
            for i, w in enumerate(words)]
