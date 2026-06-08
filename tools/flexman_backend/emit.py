# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Simon Davidson, University of Manchester
# =============================================================================
# flexman_backend.emit — output emitters (hex memory images, AXI .vh includes)
#
# Author: Simon Davidson & Claude
# Created: 2026-06-08
# Last modified: 2026-06-08
#
# Pure-Python (no torch). `write_hex` produces $readmemh-loadable images; the
# `Emitter` builds a Verilog include of axi_write(addr, data) statements. The
# Emitter handles mechanics only — the front-end supplies the file header text,
# keeping deployment-specific wording out of the back-end.
# =============================================================================
from __future__ import annotations
from pathlib import Path
from typing import Sequence

from . import regmap


def write_hex(path, words: Sequence[int]) -> None:
    """Write a Verilog $readmemh-compatible .hex file (one 32-bit word per line)."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w") as f:
        for w in words:
            f.write(f"{w & 0xFFFFFFFF:08x}\n")


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
