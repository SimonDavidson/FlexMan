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


# --- §5.3 thresh_mem image emission -----------------------------------------
import os
import tempfile
from flexman_backend import emit


def _read_hex(path):
    with open(path) as f:
        return [int(line.strip(), 16) for line in f if line.strip()]


def _unpack(words, base, n_entries, elem_bits):
    """Inverse of pack_table at a word base: returns raw `elem_bits`-masked entries."""
    epw = 32 // elem_bits
    mask = (1 << elem_bits) - 1
    return [(words[base + k // epw] >> ((k % epw) * elem_bits)) & mask
            for k in range(n_entries)]


def test_emit_lut_image_roundtrip_and_bases():
    # Two 256-entry int16 tables (unsigned sigmoid + signed tanh), as deployed.
    sig = lut.sigmoid_table(entries=256, in_frac_bits=5, out_frac_bits=15, out_bits=16)
    tnh = lut.tanh_table(entries=256, in_frac_bits=5, out_frac_bits=14, out_bits=16)
    with tempfile.TemporaryDirectory() as d:
        path = os.path.join(d, "thresh_mem.hex")
        bases = emit.emit_lut_image([sig, tnh], path, elem_bits=16)
        # 256 entries / 2-per-word = 128 words each -> contiguous bases 0, 128.
        assert bases == [0, 128]
        words = _read_hex(path)
        assert len(words) == 256                       # 128 + 128, dense
        # each entry round-trips bit-exactly (tables already stored masked).
        assert _unpack(words, bases[0], 256, 16) == [v & 0xFFFF for v in sig]
        assert _unpack(words, bases[1], 256, 16) == [v & 0xFFFF for v in tnh]


def test_emit_lut_image_matches_emulate_lut():
    # The emitter must reproduce the emulator's own Lut tables bit-for-bit, so the
    # RTL thresh_mem == the golden oracle. Use the DEPLOY convention in_frac=idx-3.
    import math
    from flexman_backend.emulate import Lut
    idx_bits = 8
    in_frac = idx_bits - 3                              # == RTL localparam F
    sig = Lut.build(lambda x: 1.0 / (1.0 + math.exp(-x)),
                    in_frac=in_frac, out_frac=15, idx_bits=idx_bits, signed=False)
    tnh = Lut.build(math.tanh, in_frac=in_frac, out_frac=14, idx_bits=idx_bits, signed=True)
    with tempfile.TemporaryDirectory() as d:
        path = os.path.join(d, "thresh_mem.hex")
        bases = emit.emit_lut_image([sig.table, tnh.table], path, elem_bits=16)
        words = _read_hex(path)
        assert _unpack(words, bases[0], 256, 16) == [v & 0xFFFF for v in sig.table]
        assert _unpack(words, bases[1], 256, 16) == [v & 0xFFFF for v in tnh.table]
