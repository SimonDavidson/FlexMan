# SPDX-License-Identifier: MIT
# Author: Simon Davidson & Claude
# Created: 2026-06-08
# Last modified: 2026-06-08
# Unit tests for flexman_backend.cfgmem (no torch, no pytest required).
from flexman_backend import cfgmem


def test_snn_layout_positions():
    cfg = {
        "sp_act_base_addr": 0x2A0, "sp_weight_base_addr": 0x111,
        "syn_curr_base_addr": 0x2E3, "sp_weight_sz": 3,
        "sp_total_timesteps": 1, "np_syn_curr_decay_mult": 0xD2E7CC00,
        "np_pot_decay_mult": 0x70ACB100, "np_last_neuron_idx": 0x1F,
        "np_bias_curr_base_addr": 5, "np_thresh_base_addr": 6, "np_pot_base_addr": 7,
        "np_syn_curr_sz": 5, "np_bias_curr_sz": 5, "np_pot_sz": 5,
        "np_mode": 0, "sp_skip_neuron": 1,
        "bin_point_syn_curr": 0xDEAD,   # out-of-window — must be ignored
    }
    w = cfgmem.pack_cfg_words(cfg, "snn")
    assert len(w) == 16
    assert w[0] == 0x2A0 and w[1] == 0x111 and w[2] == 0x2E3 and w[3] == 3
    assert w[4] == 1                          # (np_mode<<1)|skip = 0|1
    assert w[5] == 1
    assert w[6] == 0xD2E7CC00 and w[7] == 0x70ACB100   # decays aliased into window
    assert w[8] == 0x1F and w[9] == 0          # word9 unused
    assert w[15] == 5
    assert 0xDEAD not in w                      # bin_point (out-of-window) absent


def test_snn_packed_control_word():
    assert cfgmem.pack_cfg_words({"np_mode": 0, "sp_skip_neuron": 1}, "snn")[4] == 1
    assert cfgmem.pack_cfg_words({"np_mode": 2, "sp_skip_neuron": 0}, "snn")[4] == 4
    assert cfgmem.pack_cfg_words({"np_mode": 0b101, "sp_skip_neuron": 1}, "snn")[4] == 0b1011


def test_ann_layout_skip_only_word6_unused():
    w = cfgmem.pack_cfg_words({"sp_skip_neuron": 1, "np_mode": 3, "np_pot_decay_mult": 0xABCD}, "ann")
    assert w[4] == 1            # ann word4 = skip only (np_mode NOT packed)
    assert w[6] == 0            # ann word6 unused
    assert w[7] == 0xABCD       # pot decay still aliased into word7


def test_axi_write_addresses():
    words = list(range(16))
    writes = cfgmem.cfgmem_axi_writes(2, words)          # cfg_id 2
    assert writes[0] == (0xA0000000 | ((2 * 16 + 0) << 2), 0)    # 0xA0000080
    assert writes[0][0] == 0xA0000080
    assert writes[15][0] == (0xA0000000 | ((2 * 16 + 15) << 2))  # 0xA00000BC
    assert writes[15][0] == 0xA00000BC
