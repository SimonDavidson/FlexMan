# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Simon Davidson, University of Manchester
# =============================================================================
# flexman_backend.emit — output emitters (hex memory images, AXI .vh includes)
#
# Author: Simon Davidson & Claude
# Created: 2026-06-08
# Last modified: 2026-06-15
#
# Pure-Python (no torch). `write_hex` produces $readmemh-loadable images; the
# `Emitter` builds a Verilog include of axi_write(addr, data) statements. The
# Emitter handles mechanics only — the front-end supplies the file header text,
# keeping deployment-specific wording out of the back-end.
# `emit_lut_image` packs activation LUT tables into a thresh_mem .hex image
# (§5.3) — generic table mechanics only; the front-end chooses which functions
# and scales (keeping application specifics out of the back-end).
# =============================================================================
from __future__ import annotations
from pathlib import Path
from typing import Sequence

from . import regmap
from .quant import pack_table


def write_hex(path, words: Sequence[int]) -> None:
    """Write a Verilog $readmemh-compatible .hex file (one 32-bit word per line)."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w") as f:
        for w in words:
            f.write(f"{w & 0xFFFFFFFF:08x}\n")


def emit_lut_image(tables: Sequence[Sequence[int]], path, elem_bits: int = 16,
                   bases: Sequence[int] | None = None) -> list[int]:
    """Pack one or more flat activation LUT tables into a single thresh_mem image
    and write it as a $readmemh .hex (annAcc LUT load path, §5.3).

    Each table is packed `32 // elem_bits` entries-per-word, LSB-first, via
    `pack_table` (two's-complement masked to elem_bits — correct for signed tanh
    and unsigned sigmoid alike). Tables are laid out at ascending 32-bit WORD
    bases: contiguous from 0 by default, or at the explicit `bases` given. Any
    gap or tail is zero-filled so the image is a dense word list.

    Returns the list of word bases — each is the `np_thresh_base` to configure
    for the gate task that uses that table. The caller records the matching
    `out_frac` per table (the activation bin point) in its own manifest.
    """
    packed = [pack_table(list(t), elem_bits) for t in tables]
    if bases is None:
        bases, nxt = [], 0
        for p in packed:
            bases.append(nxt)
            nxt += len(p)
    bases = list(bases)
    end = max((b + len(p) for b, p in zip(bases, packed)), default=0)
    image = [0] * end
    for b, p in zip(bases, packed):
        image[b:b + len(p)] = p
    write_hex(path, image)
    return bases


class Emitter:
    """Accumulates `axi_write(addr, data);` lines for a static-config include."""

    def __init__(self) -> None:
        self.lines: list[str] = []

    def comment(self, txt: str) -> None:
        self.lines.append(f"        // {txt}")

    def blank(self) -> None:
        self.lines.append("")

    def section(self, title: str) -> None:
        self.blank()
        self.lines.append(f"        // " + "=" * 64)
        self.lines.append(f"        // {title}")
        self.lines.append(f"        // " + "=" * 64)

    def write(self, addr: int, data: int, note: str = "") -> None:
        c = f"  // {note}" if note else ""
        self.lines.append(
            f"        axi_write(32'h{addr:08X}, 32'h{data & 0xFFFFFFFF:08X});{c}"
        )

    def reg(self, cfg_base: int, regs: dict, name: str, value: int, note: str = "") -> None:
        """Emit an AXI write to a named per-accelerator config register."""
        addr = regmap.axi_reg_addr(cfg_base, regs[name])
        self.write(addr, value & 0xFFFFFFFF,
                   f"{name} = 0x{value & 0xFFFFFFFF:08X}" + (f"  ({note})" if note else ""))

    def render(self, header: str) -> str:
        """Compose the final file: `header` (verbatim) followed by the body.

        `header` must end with the blank separator line as required for
        byte-exact reproduction of an existing include.
        """
        return header + "\n".join(self.lines) + "\n"
