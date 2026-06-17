# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Simon Davidson, University of Manchester
# =============================================================================
# flexman_backend.quant — fixed-point encoding & memory-image packing
#
# Author: Simon Davidson & Claude
# Created: 2026-06-08
# Last modified: 2026-06-17
#
# Pure-Python (no torch). Operates on plain Python integers / nested int lists so
# the back-end is testable without a deep-learning stack. Front-ends own the
# torch dependency: they load a checkpoint, quantise tensors to integers, call
# `.tolist()`, and pass the result here.
#
# The column-major packing convention matches weight_generator.v (out_elem_count
# resets per input neuron) and is the same layout the reference front-end
# tooling proved against the standalone testbenches.
# =============================================================================
from __future__ import annotations
import math
from typing import Callable, Sequence


def log2_int(x: float) -> int:
    """Return -k for step_size = 2^-k. Raises if x is not an exact power of two."""
    lg = math.log2(x)
    k = int(round(lg))
    if abs(lg - k) > 1e-6:
        raise ValueError(f"{x} is not an exact power of 2")
    return k


def q032(x: float) -> int:
    """Convert a float in [0, 1) to a Q0.32 unsigned word; clip at the boundaries."""
    if x >= 1.0:
        return 0xFFFFFFFF
    if x <= 0.0:
        return 0
    return int(round(x * (1 << 32))) & 0xFFFFFFFF


def sign_extend(v: int, bits: int = 32) -> int:
    """Return v as a `bits`-wide unsigned word, two's-complement encoded."""
    return v & ((1 << bits) - 1)


def quantise_to_int(values, step: float, bits: int = 8):
    """Round-to-nearest, clamp to a signed `bits`-wide integer range.

    `values` may be a scalar or an (arbitrarily nested) list of floats; the
    nesting is preserved. No torch — front-ends that hold torch tensors should
    pass `tensor.tolist()`.
    """
    lo, hi = -(1 << (bits - 1)), (1 << (bits - 1)) - 1

    def q(v):
        if isinstance(v, (list, tuple)):
            return [q(e) for e in v]
        return max(lo, min(hi, int(round(v / step))))

    return q(values)


def pack_weights_column_major(rows: Sequence[Sequence[int]],
                              elem_bits: int = 8) -> tuple[list[int], int]:
    """Pack a 2-D signed-int weight matrix `[out_features][in_features]` into
    column-major 32-bit words.

    For each input neuron k, emit `words_per_input = ceil(out_features /
    elems_per_word)` consecutive words; word g of input k packs outputs
    `[g*epw .. g*epw+epw-1]` LSB-first (element 0 in the low bits). The last word
    of each input is zero-padded if out_features is not a multiple of epw.

    Returns (words, words_per_input). Mirrors weight_generator.v.
    """
    out_f = len(rows)
    in_f = len(rows[0]) if out_f else 0
    assert all(len(r) == in_f for r in rows), "ragged weight matrix"
    epw = 32 // elem_bits
    mask = (1 << elem_bits) - 1
    pad = (-out_f) % epw
    words_per_input = (out_f + pad) // epw
    words: list[int] = []
    for k in range(in_f):
        for g in range(words_per_input):
            w = 0
            for j in range(epw):
                o = g * epw + j
                val = rows[o][k] if o < out_f else 0
                w |= (val & mask) << (elem_bits * j)
            words.append(w)
    return words, words_per_input


def pack_bias(values: Sequence[int], bits: int = 32) -> list[int]:
    """Biases stored 1-per-word in their two's-complement integer representation."""
    return [sign_extend(int(v), bits) for v in values]


def pack_table(values: Sequence[int], elem_bits: int = 32) -> list[int]:
    """Pack a flat 1-D table (e.g. a LUT) into 32-bit words, elems-per-word
    LSB-first, zero-padding the final word. Mirrors a single weight column."""
    epw = 32 // elem_bits
    mask = (1 << elem_bits) - 1
    words: list[int] = []
    for i in range(0, len(values), epw):
        w = 0
        for j in range(epw):
            if i + j < len(values):
                w |= (int(values[i + j]) & mask) << (elem_bits * j)
        words.append(w)
    return words


def fold_bias_column(rows: Sequence[Sequence[int]],
                     bias_weights: Sequence[int]) -> list[list[int]]:
    """Fold a per-output bias into the weight matrix as one extra constant-1
    input column: `y = W·x + b` becomes `y = [W | b] · [x ; 1]`.

    `bias_weights[o]` is the bias for output o ALREADY EXPRESSED at the weight
    scale (front-end's responsibility — it must equal round(b_o / weight_step)
    given the convention that the appended activation input carries the integer
    value 1 at the activation scale). Returns new rows with in_features + 1.
    """
    assert len(rows) == len(bias_weights), "bias length must match out_features"
    return [list(row) + [int(bias_weights[o])] for o, row in enumerate(rows)]
