# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Simon Davidson, University of Manchester
# =============================================================================
# flexman_backend.mempool — shared-pool allocator for buffer layout
#
# Author: Simon Davidson & Claude
# Created: 2026-06-08
# Last modified: 2026-06-08
#
# Pure-Python (no torch). The FlexMan RTL pools every accelerator's act / spike /
# syn_curr ports into ONE physical memory, so those regions must occupy disjoint
# logical word ranges and a producer's write base must EQUAL its consumer's read
# base (that coincidence IS the inter-accelerator dataflow). `SharedPool` is the
# allocation mechanism; the specific regions are a front-end (deployment) concern.
# =============================================================================
from __future__ import annotations
from dataclasses import dataclass


@dataclass
class MemRegion:
    """A named region inside a physical memory: base word + length (words)."""
    base:  int   # word index within this memory
    words: int   # length in 32-bit words


class SharedPool:
    """Sequential, word-aligned allocator over a fixed-capacity shared pool.

    Allocations are disjoint by construction (bump pointer); `check_disjoint`
    is a guard against any manually-placed regions overlapping.
    """

    def __init__(self, capacity_words: int) -> None:
        self.capacity = capacity_words
        self._cursor = 0
        self.regions: list[tuple[str, int, int]] = []   # (name, base, words)

    def alloc(self, name: str, words: int) -> int:
        """Allocate `words` and return the base word index."""
        base = self._cursor
        self._cursor += words
        if self._cursor > self.capacity:
            raise ValueError(
                f"shared pool overflow allocating {name}: "
                f"need {self._cursor} words > {self.capacity} capacity"
            )
        self.regions.append((name, base, words))
        return base

    def check_disjoint(self) -> None:
        """Fail loudly if any two allocated regions overlap."""
        spans = sorted((base, base + words, name) for name, base, words in self.regions)
        for (b0, e0, n0), (b1, e1, n1) in zip(spans, spans[1:]):
            if e0 > b1:
                raise ValueError(
                    f"shared-pool overlap: {n0} [{b0}:{e0}) intersects {n1} [{b1}:{e1})"
                )
