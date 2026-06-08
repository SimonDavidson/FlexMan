# SPDX-License-Identifier: MIT
# Author: Simon Davidson & Claude
# Created: 2026-06-08
# Last modified: 2026-06-08
# Unit tests for flexman_backend.lut (no torch, no pytest required).
from flexman_backend import lut


def _s16(v):
    return v - (1 << 16) if v >= (1 << 15) else v


def test_sigmoid_unsigned_monotone_midpoint():
    s = lut.sigmoid_table(entries=256, in_frac_bits=4, out_frac_bits=15, out_bits=16)
    assert len(s) == 256
    assert all(s[i] <= s[i + 1] for i in range(255))      # monotone non-decreasing
    assert s[128] == (1 << 14)                            # sigmoid(0)=0.5 at Q.15
    assert 0 <= s[0] < s[128] < s[-1] <= 0xFFFF           # endpoints saturate toward 0 / ~1


def test_tanh_signed_odd():
    t = lut.tanh_table(entries=256, in_frac_bits=4, out_frac_bits=14, out_bits=16)
    assert len(t) == 256
    assert _s16(t[128]) == 0                              # tanh(0)=0
    assert _s16(t[0]) < 0 and _s16(t[-1]) > 0             # signed, odd
    assert all(_s16(t[i]) <= _s16(t[i + 1]) for i in range(255))
    assert _s16(t[0]) == -(1 << 14) and _s16(t[-1]) == (1 << 14)   # saturate to -1 / +1


def test_index_window_convention():
    # entry idx corresponds to x = (idx - entries/2) / 2^in_frac_bits
    s = lut.function_table(lambda x: x, entries=256, in_frac_bits=4,
                           out_frac_bits=4, out_bits=32, signed=True)
    # idx=128 -> x=0 -> 0 ; idx=129 -> x=1/16 -> round(1/16 * 16)=1
    assert s[128] == 0
    assert s[129] == 1
    assert (s[127] - (1 << 32)) == -1
