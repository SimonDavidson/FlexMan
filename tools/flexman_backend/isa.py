# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Simon Davidson, University of Manchester
# =============================================================================
# flexman_backend.isa — FlexMan scheduler ISA assembler
#
# Author: Simon Davidson & Claude
# Created: 2026-06-08
# Last modified: 2026-08-20
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

# ---------------------------------------------------------------------------
# Wide-ntgt TASK form (2026-08-20)
#
# The narrow TASK packs three long slots x [mode(2)+id(4)+ntgt(4)] into word 2's
# 30 usable bits -- exactly full, so ntgt cannot exceed 15. Monarch at nblocks=40
# needs usage counts up to 85.
#
# A wide TASK is selected by TASK word 1 bit 31, which has always been reserved
# and zero (ISA_REFERENCE.md:117), so this ADDS a second instruction form without
# altering the existing one: every previously generated image stays byte-valid.
# Verified against all committed images -- 142 TASK instructions, bit 31 clear in
# every one.
#
# Slots stay CONTIGUOUS: nothing straddles a word boundary, and the wide word
# packs slots at the same SLOT_LONG_SZ stride the scheduler uses internally, so
# its decode is a straight slice copy.
#
#   W2  [1:0]   sentinel 2'b00
#       [14:2]  slot 3   {mode, id, ntgt}
#       [27:15] slot 4
#       [31:28] spare
#   W3  [12:0]  slot 5
#       [31:13] spare
MODE_SZ           = 2
BUFF_INDX_SZ      = 4
NTGT_SZ_NARROW    = 4
NTGT_SZ_WIDE      = 7
NTGT_MAX_NARROW   = (1 << NTGT_SZ_NARROW) - 1    # 15
NTGT_MAX_WIDE     = (1 << NTGT_SZ_WIDE) - 1      # 127
SLOT_LONG_SZ_WIDE = MODE_SZ + BUFF_INDX_SZ + NTGT_SZ_WIDE   # 13


def _slot_long_wide(mode: int, buf: int, ntgt: int) -> int:
    """One long slot at the wide stride, lsb-first: {ntgt, id, mode}."""
    assert mode < 4, mode
    assert buf < (1 << BUFF_INDX_SZ), buf
    assert ntgt <= NTGT_MAX_WIDE, f"ntgt {ntgt} exceeds wide field ({NTGT_MAX_WIDE})"
    return ((mode & 0x3)
            | (buf  & 0xF)  << MODE_SZ
            | (ntgt & 0x7F) << (MODE_SZ + BUFF_INDX_SZ))


def tw1(acc: int, cfg: int, colour: int,
        m0: int, b0: int, m1: int, b1: int, m2: int, b2: int,
        wide: bool = False) -> int:
    """TASK word 1: opcode + acc_id + cfg_id + colour + slots 0..2 (short).

    cfg_id is a 7-bit field [11:5] (up to 128 configs); colour and the three
    short slots are shifted up by 2 bits relative to the legacy 5-bit layout.

    wide=True sets bit 31, selecting the three-word wide-ntgt form (see the
    module header). Default False keeps the exact legacy encoding.
    """
    assert acc < 4 and cfg < 128 and colour < 2
    return (
        OP_TASK
        | (1 << 31 if wide else 0)
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
    """TASK word 2: slots 3..5 (long), with sentinel 2'b00 in low bits.

    ntgt (usage count) is a 4-bit field per slot (up to 15); slots 4 and 5 shift
    up relative to the legacy 3-bit layout, filling word-2 exactly to bit 31.
    """
    assert max(n3, n4, n5) < 16 and max(b3, b4, b5) < 16
    return (
        0
        | (m3 & 0x3)   << 2
        | (b3 & 0xF)   << 4
        | (n3 & 0xF)   << 8
        | (m4 & 0x3)   << 12
        | (b4 & 0xF)   << 14
        | (n4 & 0xF)   << 18
        | (m5 & 0x3)   << 22
        | (b5 & 0xF)   << 24
        | (n5 & 0xF)   << 28
    )


def tw2_wide(m3: int, b3: int, n3: int,
             m4: int, b4: int, n4: int) -> int:
    """Wide TASK word 2: sentinel + long slots 3 and 4 at the SLOT_LONG_SZ stride."""
    return (_slot_long_wide(m3, b3, n3) << 2
            | _slot_long_wide(m4, b4, n4) << (2 + SLOT_LONG_SZ_WIDE))


def tw3_wide(m5: int, b5: int, n5: int) -> int:
    """Wide TASK word 3: long slot 5 at bit 0; [31:13] spare."""
    return _slot_long_wide(m5, b5, n5)


def task_words(acc: int, cfg: int, colour: int,
               m0: int, b0: int, m1: int, b1: int, m2: int, b2: int,
               m3: int, b3: int, n3: int,
               m4: int, b4: int, n4: int,
               m5: int, b5: int, n5: int,
               force_wide: bool = False) -> tuple:
    """Assemble one TASK, choosing the narrow (2-word) or wide (3-word) form.

    Narrow is used whenever every usage count fits the 4-bit field, so programs
    grow only where they must. force_wide=True emits the wide form regardless,
    which is what the nblocks=4 dual-encoding equivalence test uses: that variant
    fits BOTH forms, so the same schedule can be run through each decode path and
    required to be bit-exact.
    """
    if force_wide or max(n3, n4, n5) > NTGT_MAX_NARROW:
        return (tw1(acc, cfg, colour, m0, b0, m1, b1, m2, b2, wide=True),
                tw2_wide(m3, b3, n3, m4, b4, n4),
                tw3_wide(m5, b5, n5))
    return (tw1(acc, cfg, colour, m0, b0, m1, b1, m2, b2),
            tw2(m3, b3, n3, m4, b4, n4, m5, b5, n5))


def fill_w1_wide(buf_id: int, colour: int, ntgt: int, block_size: int) -> int:
    """FILL word 1, wide-ntgt form: ntgt is 7-bit [15:9], block_size 16-bit [31:16].

    NOTE, and it matters: unlike TASK there is no per-instruction selector, so the
    form is fixed by the hardware's WIDE_NTGT parameter for the WHOLE BUILD. A
    WIDE_NTGT=1 build needs a program whose FILLs use THIS encoder. That is the one
    part of this work which is not backward compatible, which is why WIDE_NTGT
    defaults to 0 and only tops that regenerate their program turn it on.

    FILL word 1 bit 7 IS free (buf_id occupies [6:3], colour sits at [8]) and could
    carry a per-instruction selector like TASK's bit 31 — but it is deliberately NOT
    spent. That bit is free only because BUFF_INDX_SZ=4; at 32 buffers buf_id becomes
    [7:3] and consumes it. The ceiling is already binding: the N=2 multi-lane program
    uses 15 of 16 buffer ids (5 singletons + 5 per-lane roles, so N=3 would need 20).
    Keeping bit 7 reserved preserves the ability to widen buf_id; the price is that
    FILL's widened ntgt is build-wide, which costs nothing real because only builds
    that regenerate their programs set WIDE_NTGT=1.

    block_size at 16 bits still covers 65,536 words; the largest fill measured in
    any current schedule is 632.
    """
    assert ntgt <= NTGT_MAX_WIDE, f"ntgt {ntgt} exceeds wide field ({NTGT_MAX_WIDE})"
    assert buf_id < 16 and block_size < (1 << 16), (buf_id, block_size)
    return (
        OP_FILL
        | (buf_id     & 0xF)    << 3
        | (colour     & 0x1)    << 8
        | (ntgt       & 0x7F)   << 9
        | (block_size & 0xFFFF) << 16
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
    """FILL word 1: ntgt is a 4-bit field [12:9] (up to 15); block_size shifts up
    to [31:13] (19-bit, up to 512K words)."""
    assert ntgt < 16 and buf_id < 16
    return (
        OP_FILL
        | (buf_id     & 0xF)     << 3
        | (colour     & 0x1)     << 8
        | (ntgt       & 0xF)     << 9
        | (block_size & 0x7FFFF) << 13
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
