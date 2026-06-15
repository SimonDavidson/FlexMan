# SPDX-License-Identifier: MIT
# Author: Simon Davidson & Claude
# Created: 2026-06-08
# Last modified: 2026-06-11
# Unit tests for flexman_backend.cfgmem packed per-task layout (no torch/pytest).
from flexman_backend import cfgmem


def test_packed_length():
    from flexman_backend import regmap
    assert regmap.PACKED_WORDS_PER_CONFIG == 15        # meaningful packed words
    assert cfgmem.WORDS_PER_CONFIG == 16               # cfg_mem stride (power of 2)
    w = cfgmem.pack_cfg_words({}, "snn")
    assert len(w) == 16
    assert w[15] == 0                                  # word 15 is spare


def test_address_words():
    cfg = dict(sp_act_base_addr=0x2A0, sp_weight_base_addr=0x111,
               syn_curr_base_addr=0x2E3, np_bias_curr_base_addr=4,
               np_thresh_base_addr=5, np_pot_base_addr=6, np_spike_base_addr=7,
               np_syn_curr_decay_mult=0xD2E7CC00, np_pot_decay_mult=0x70ACB100)
    w = cfgmem.pack_cfg_words(cfg, "snn")
    assert w[0:9] == [0x2A0, 0x111, 0x2E3, 4, 5, 6, 7, 0xD2E7CC00, 0x70ACB100]


def test_size_lanes():
    cfg = dict(sp_in_x_len=512, sp_in_y_len=1, sp_out_x_len=128, sp_out_y_len=2,
               sp_rows_per_neuron=32, np_last_neuron_idx=127, sp_total_timesteps=8)
    w = cfgmem.pack_cfg_words(cfg, "snn")
    assert w[9]  == (512 | (1 << 16))     # S0: in_x | in_y
    assert w[10] == (128 | (2 << 16))     # S1: out_x | out_y
    assert w[11] == (32  | (127 << 16))   # S2: rows_per_neuron | last_neuron_idx
    assert w[12] == 8                     # S3: total_timesteps (high lane spare = 0)


def test_size_16bit_headroom():
    # future-proof: each size lane holds up to 65535
    w = cfgmem.pack_cfg_words(dict(sp_in_x_len=0xFFFF, sp_in_y_len=0xABCD), "snn")
    assert w[9] == (0xFFFF | (0xABCD << 16))


def test_mode_word_m0():
    cfg = dict(sp_weight_sz=3, np_syn_curr_sz=5, np_bias_curr_sz=5, np_pot_sz=5,
               sp_act_sz=3, np_thresh_op=2, sp_weight_mode=1)
    m0 = cfgmem.pack_cfg_words(cfg, "ann")[13]
    assert (m0 >> 0)  & 0xF == 3    # weight_sz
    assert (m0 >> 4)  & 0xF == 5    # syn_curr_sz
    assert (m0 >> 8)  & 0xF == 5    # bias_sz
    assert (m0 >> 12) & 0xF == 5    # pot_sz
    assert (m0 >> 16) & 0xF == 3    # act_sz
    assert (m0 >> 20) & 0xF == 2    # thresh_op (4-bit headroom)
    assert (m0 >> 24) & 0xF == 1    # weight_mode (4-bit headroom)


def test_mode_word_m1():
    cfg = dict(sp_skip_neuron=1, np_mode=0b101, sp_weights_per_word=4,
               bin_point_syn_curr=16, np_out_bin_point=4)
    m1 = cfgmem.pack_cfg_words(cfg, "snn")[14]
    assert (m1 >> 0)  & 0x3  == 1
    assert (m1 >> 2)  & 0xF  == 0b101
    assert (m1 >> 6)  & 0xF  == 4
    assert (m1 >> 10) & 0x3F == 16
    assert (m1 >> 16) & 0x3F == 4


def test_field_overflow_masked():
    # an over-wide value is truncated to its slot, not bleeding into neighbours
    m0 = cfgmem.pack_cfg_words(dict(sp_weight_sz=0xFF, np_syn_curr_sz=1), "snn")[13]
    assert (m0 >> 0) & 0xF == 0xF
    assert (m0 >> 4) & 0xF == 1


def test_out_of_layout_ignored():
    w = cfgmem.pack_cfg_words(dict(sp_x_kernel_len=0xDEAD), "snn")  # conv boot reg
    assert 0xDEAD not in w


def test_fmi_packed_length():
    from flexman_backend import regmap
    assert regmap.PACKED_FMI_WORDS_PER_CONFIG == 16    # fills the whole stride
    w = cfgmem.pack_cfg_words({}, "fmisnn")
    assert len(w) == 16


def test_fmi_address_words():
    cfg = dict(sp_act_base_addr=1, sp_weight_base_addr=2, syn_curr_base_addr=3,
               np_thresh_base_addr=4, np_pot_base_addr=5, np_spike_base_addr=6,
               np_dcy_syn_base_addr=7, np_dcy_mem_base_addr=8, np_ada_base_addr=9,
               np_b_eff_base_addr=10, np_dcy_ada_base_addr=11,
               np_scl_ada_base_addr=12)
    w = cfgmem.pack_cfg_words(cfg, "fmisnn")
    assert w[0:12] == list(range(1, 13))


def test_fmi_size_lanes():
    cfg = dict(sp_in_x_len=257, sp_out_x_len=128, sp_rows_per_neuron=32,
               np_last_neuron_idx=127, sp_total_timesteps=8)
    w = cfgmem.pack_cfg_words(cfg, "fmisnn")
    assert w[12] == (257 | (128 << 16))   # S0: in_x | out_x (fmi is 1-D)
    assert w[13] == (32  | (127 << 16))   # S1: rows_per_neuron | last_neuron_idx
    assert w[14] == 8                     # S2: total_timesteps (high lane spare)


def test_fmi_mode_word():
    cfg = dict(sp_skip_neuron=1, np_mode=0b101, sp_weights_per_word=4,
               bin_point_syn_curr=16, sp_weight_sz=3, np_syn_curr_sz=5,
               np_pot_sz=5, sp_weight_mode=1, np_has_ada=1)
    m0 = cfgmem.pack_cfg_words(cfg, "fmisnn")[15]
    assert (m0 >> 0)  & 0x3  == 1        # skip_neuron
    assert (m0 >> 2)  & 0xF  == 0b101    # np_mode
    assert (m0 >> 6)  & 0xF  == 4        # weights_per_word
    assert (m0 >> 10) & 0x3F == 16       # bin_point_syn_curr
    assert (m0 >> 16) & 0xF  == 3        # weight_sz
    assert (m0 >> 20) & 0xF  == 5        # syn_curr_sz
    assert (m0 >> 24) & 0xF  == 5        # pot_sz
    assert (m0 >> 28) & 0x3  == 1        # weight_mode
    assert (m0 >> 30) & 0x1  == 1        # has_ada


def test_unknown_acc_type():
    try:
        cfgmem.pack_cfg_words({}, "bogus")
        raise AssertionError("expected ValueError")
    except ValueError:
        pass


def test_axi_write_addresses():
    writes = cfgmem.cfgmem_axi_writes(2, list(range(16)))         # cfg_id 2, WPC=16
    assert len(writes) == 16
    assert writes[0][0]  == (0xA0000000 | ((2 * 16 + 0)  << 2))
    assert writes[15][0] == (0xA0000000 | ((2 * 16 + 15) << 2))


def test_hu_cfg_words_flat_layout():
    # Hadamard: one 32-bit register per word; field lands at word (offset>>2).
    from flexman_backend import regmap
    cfg = dict(mode=1, stream_len=400,
               src_a_base_addr=0x100, src_a_elem_sz=5, src_a_bin_point=15,
               src_r_base_addr=0x200, src_r_elem_sz=4, src_r_bin_point=15)
    w = cfgmem.pack_hu_cfg_words(cfg)
    assert len(w) == 16
    for name, off in regmap.HU_REG_OFFSETS.items():
        # config_manager streams word i -> byte i*4, so every HU field MUST be on
        # the word*4 grid (else it is unreachable per-task) and within the stride.
        assert off % 4 == 0, f"{name} offset {off:#x} not word-aligned"
        assert (off >> 2) < cfgmem.WORDS_PER_CONFIG
        assert w[off >> 2] == cfg.get(name, 0)


def test_hu_cfg_words_defaults_zero():
    assert cfgmem.pack_hu_cfg_words({}) == [0] * 16     # all reset (mode=0, ...)
