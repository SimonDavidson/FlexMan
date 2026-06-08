# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Simon Davidson, University of Manchester
# =============================================================================
# flexman_backend.lut — activation lookup-table generation (sigmoid / tanh)
#
# Author: Simon Davidson & Claude
# Created: 2026-06-08
# Last modified: 2026-06-08
#
# Pure-Python (no torch). Generates fixed-point function tables for the annAcc
# LUT activation path (np_thresh_op = LUT), to be loaded into thresh_mem. See
# ~/work/jabra/nsnet2_flexman_mapping.md §5: the table is indexed by a
# sign-centred, saturated window of the accumulator —
#   idx = clamp( (syn_curr >> (bin_point_syn_curr - F)) + entries/2, 0, entries-1 )
# so entry `idx` corresponds to input x = (idx - entries/2) / 2^F.
#
# `in_frac_bits` (F) sets the input domain: domain = ±(entries/2)/2^F.
# Defaults (entries=256, F=4) give a ±8 domain — sigmoid/tanh are saturated well
# inside it. Output is fixed-point at `out_frac_bits`, signed for tanh.
#
# NOTE: the exact thresh_mem packing (entries-per-word for a given lut_out_sz)
# is confirmed against the LUT cache at integration; use pack_table(elem_bits).
# =============================================================================
from __future__ import annotations
import math
from typing import Callable


def function_table(fn: Callable[[float], float],
                   entries: int = 256,
                   in_frac_bits: int = 4,
                   out_frac_bits: int = 14,
                   out_bits: int = 16,
                   signed: bool = False) -> list[int]:
    """Build a fixed-point table of `fn` over a sign-centred input window.

    entry idx -> x = (idx - entries/2) / 2^in_frac_bits
    value     -> clamp(round(fn(x) * 2^out_frac_bits)) stored as a two's-complement
                 `out_bits`-wide word (unsigned range if signed=False).
    """
    mid = entries // 2
    if signed:
        lo, hi = -(1 << (out_bits - 1)), (1 << (out_bits - 1)) - 1
    else:
        lo, hi = 0, (1 << out_bits) - 1
    mask = (1 << out_bits) - 1
    table: list[int] = []
    for idx in range(entries):
        x = (idx - mid) / (1 << in_frac_bits)
        q = int(round(fn(x) * (1 << out_frac_bits)))
        q = max(lo, min(hi, q))
        table.append(q & mask)
    return table


def sigmoid_table(entries: int = 256, in_frac_bits: int = 4,
                  out_frac_bits: int = 15, out_bits: int = 16) -> list[int]:
    """Logistic sigmoid in [0, 1] — unsigned output."""
    return function_table(lambda x: 1.0 / (1.0 + math.exp(-x)),
                          entries, in_frac_bits, out_frac_bits, out_bits, signed=False)


def tanh_table(entries: int = 256, in_frac_bits: int = 4,
               out_frac_bits: int = 14, out_bits: int = 16) -> list[int]:
    """Hyperbolic tangent in [-1, 1] — signed output (needs the signed LUT path)."""
    return function_table(math.tanh,
                          entries, in_frac_bits, out_frac_bits, out_bits, signed=True)
