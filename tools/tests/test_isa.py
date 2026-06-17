# SPDX-License-Identifier: MIT
# Author: Simon Davidson & Claude
# Created: 2026-06-08
# Last modified: 2026-06-17
# Unit tests for flexman_backend.isa (no torch, no pytest required).
from flexman_backend import isa


def test_simple_opcodes():
    assert isa.stop_inst() == 0b010
    assert isa.nxt_inst(True, True) == (0b100 | (1 << 4) | (1 << 5))   # 0x34
    assert isa.nxt_inst(True, False) == (0b100 | (1 << 4))            # 0x14
    assert isa.loopend_inst(0) == 0b111
    assert isa.loopend_inst(3) == (0b111 | (3 << 3))


def test_loop_count_semantics():
    # total iterations = count + 1; count occupies bits [31:6]
    assert isa.loop_inst(0, 298) == (0b110 | (298 << 6))             # 0x4a86
    assert isa.loop_inst(5, 0) == (0b110 | (5 << 3))


def test_fill_w1_fields():
    # buf_id[6:3], colour[8], ntgt[11:9], block_size[31:12]
    w = isa.fill_w1(buf_id=2, colour=1, ntgt=1, block_size=64)
    assert (w & 0x7) == isa.OP_FILL
    assert ((w >> 3) & 0xF) == 2
    assert ((w >> 8) & 0x1) == 1
    assert ((w >> 9) & 0x7) == 1
    assert ((w >> 12) & 0xFFFFF) == 64


def test_task_word_roundtrip():
    # A feature-extraction TASK: acc=2, cfg=0, TGT buf 0 in slot5 ntgt=1
    w1 = isa.tw1(2, 0, 0, isa.M_UNUSED, 0, isa.M_UNUSED, 0, isa.M_UNUSED, 0)
    w2 = isa.tw2(isa.M_UNUSED, 0, 0, isa.M_UNUSED, 0, 0, isa.M_TGT, 0, 1)
    assert (w1 & 0x7) == isa.OP_TASK
    assert ((w1 >> 3) & 0x3) == 2          # acc_id
    assert ((w1 >> 5) & 0x1F) == 0         # cfg_id
    assert (w2 & 0x3) == 0                 # sentinel
    assert ((w2 >> 20) & 0x3) == isa.M_TGT  # slot5 mode
    assert ((w2 >> 26) & 0x7) == 1          # slot5 #targets


def test_emit_addresses_and_listing():
    insts = [isa.Inst((0x11111111,), "one-word"),
             isa.Inst((0x22222222, 0x33333333), "two-word")]
    words, notes = isa.emit(insts)
    assert words == [0x11111111, 0x22222222, 0x33333333]
    assert notes[0].startswith("0000: 11111111")
    assert "(cont.)" in notes[2]
