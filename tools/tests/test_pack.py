# SPDX-License-Identifier: MIT
# Author: Simon Davidson & Claude
# Created: 2026-06-08
# Last modified: 2026-06-08
# Unit tests for flexman_backend.quant packing (no torch, no pytest required).
from flexman_backend import quant


def test_readout_sanity_case():
    # out=1, in=4 (the most-padded layer): each input -> one word whose only
    # nonzero byte is the LSB (output 0). Mirrors quantise_weights.py:233.
    rows = [[5, -3, 7, -1]]
    words, wpi = quant.pack_weights_column_major(rows, elem_bits=8)
    assert wpi == 1
    assert words == [5, (-3) & 0xFF, 7, (-1) & 0xFF]   # [0x05, 0xfd, 0x07, 0xff]
    assert all((w >> 8) == 0 for w in words)


def test_lsb_first_within_word():
    # out=2, in=2, 8-bit: word for input k packs [o0, o1] in low two bytes.
    rows = [[1, 2], [3, 4]]   # [out=2][in=2]
    words, wpi = quant.pack_weights_column_major(rows, elem_bits=8)
    assert wpi == 1
    assert words == [1 | (3 << 8), 2 | (4 << 8)]       # [0x0301, 0x0402]


def test_int16_packing_signed():
    rows = [[1, 2], [3, 4]]
    words, wpi = quant.pack_weights_column_major(rows, elem_bits=16)
    assert wpi == 1
    assert words == [1 | (3 << 16), 2 | (4 << 16)]
    # signed two's-complement, odd out_features -> pad
    w2, wpi2 = quant.pack_weights_column_major([[-1]], elem_bits=16)
    assert wpi2 == 1 and w2 == [0xFFFF]


def test_pack_bias_sign_extend():
    assert quant.pack_bias([5, -3]) == [5, 0xFFFFFFFD]
    assert quant.pack_table([1, 2, 3], elem_bits=8)[0] == (1 | 2 << 8 | 3 << 16)


def test_fold_bias_column():
    rows = [[1, 2], [3, 4]]
    folded = quant.fold_bias_column(rows, [10, 20])
    assert folded == [[1, 2, 10], [3, 4, 20]]


def test_quantise_to_int_nested():
    out = quant.quantise_to_int([[0.5, -0.5], [300.0, -300.0]], step=1.0, bits=8)
    assert out == [[0, 0], [127, -128]]   # rounds then clamps to int8 range
