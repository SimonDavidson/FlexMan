# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Simon Davidson, University of Manchester
# =============================================================================
# flexman_backend.isa — FlexMan scheduler ISA assembler
#
# Author: Simon Davidson & Claude
# Created: 2026-06-08
# Last modified: 2026-06-23
#
# Pure-Python (no torch). Bit-encodings mirror scheduler/ISA_REFERENCE.md and
# tb_scheduler.v. This is the single source of truth for ISA encoding, shared by
# every deployment front-end. See scheduler/ISA_REFERENCE.md for field layouts.
# =============================================================================
from __future__ import annotations
from dataclasses import dataclass
from typing import Iterable

# Opcodes (bits [2:0])
OP_TASK    = 0b000
OP_JUMP    = 0b001
OP_STOP    = 0b010
OP_CHECK   = 0b011
OP_NXT     = 0b100
OP_FILL    = 0b101
OP_LOOP    = 0b110
OP_LOOPEND = 0b111

# Slot modes
M_UNUSED = 0b00
M_SRC    = 0b01
M_RW     = 0b10
M_TGT    = 0b11


def tw1(acc: int, cfg: int, colour: int,
        m0: int, b0: int, m1: int, b1: int, m2: int, b2: int) -> int:
    """TASK word 1: opcode + acc_id + cfg_id + colour + slots 0..2 (short).

    cfg_id is a 7-bit field [11:5] (up to 128 configs); colour and the three
    short slots are shifted up by 2 bits relative to the legacy 5-bit layout.
    """
    assert acc < 4 and cfg < 128 and colour < 2
    return (
        OP_TASK
        | (acc    & 0x3)   << 3
        | (cfg    & 0x7F)  << 5
        | (colour & 0x1)   << 12
        | (m0     & 0x3)   << 13
        | (b0     & 0xF)   << 15
        | (m1     & 0x3)   << 19
        | (b1     & 0xF)   << 21
        | (m2     & 0x3)   << 25
        | (b2     & 0xF)   << 27
    )


def tw2(m3: int, b3: int, n3: int,
        m4: int, b4: int, n4: int,
        m5: int, b5: int, n5: int) -> int:
    """TASK word 2: slots 3..5 (long), with sentinel 2'b00 in low bits."""
    return (
        0
        | (m3 & 0x3)   << 2
        | (b3 & 0xF)   << 4
        | (n3 & 0x7)   << 8
        | (m4 & 0x3)   << 11
        | (b4 & 0xF)   << 13
        | (n4 & 0x7)   << 17
        | (m5 & 0x3)   << 20
        | (b5 & 0xF)   << 22
        | (n5 & 0x7)   << 26
    )


def stop_inst() -> int:
    return OP_STOP


def jump_inst(target_word: int) -> int:
    return OP_JUMP | (target_word & ((1 << 29) - 1)) << 3


def nxt_inst(nxt_in: bool, nxt_out: bool) -> int:
    return OP_NXT | (1 << 4 if nxt_in else 0) | (1 << 5 if nxt_out else 0)


def loop_inst(loop_id: int, count: int) -> int:
    """Hardware loop; total iterations = count + 1."""
    return OP_LOOP | (loop_id & 0x7) << 3 | (count & ((1 << 26) - 1)) << 6


def loopend_inst(loop_id: int) -> int:
    return OP_LOOPEND | (loop_id & 0x7) << 3


def fill_w1(buf_id: int, colour: int, ntgt: int, block_size: int) -> int:
    return (
        OP_FILL
        | (buf_id     & 0xF)     << 3
        | (colour     & 0x1)     << 8
        | (ntgt       & 0x7)     << 9
        | (block_size & 0xFFFFF) << 12
    )


def check_inst(buf_id: int, mode: int, skip_addr: int) -> int:
    """CHECK: mode 1 = finish-on-success, 0 = skip-on-success (jump to skip_addr)."""
    return (
        OP_CHECK
        | (mode      & 0x1)   << 3
        | (buf_id    & 0xF)   << 4
        | (skip_addr & 0x3FF) << 12
    )


@dataclass
class Inst:
    words: tuple[int, ...]
    comment: str


def emit(insts: Iterable[Inst]) -> tuple[list[int], list[str]]:
    """Flatten instructions to (word list, annotated listing lines)."""
    words: list[int] = []
    notes: list[str] = []
    addr = 0
    for inst in insts:
        first = True
        for w in inst.words:
            if first:
                notes.append(f"{addr:04x}: {w:08x}   ; {inst.comment}")
                first = False
            else:
                notes.append(f"{addr:04x}: {w:08x}     (cont.)")
            words.append(w)
            addr += 1
    return words, notes
