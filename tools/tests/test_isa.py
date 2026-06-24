# SPDX-License-Identifier: MIT
# Author: Simon Davidson & Claude
# Created: 2026-06-08
# Last modified: 2026-06-24
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
    # buf_id[6:3], colour[8], ntgt[12:9] (4-bit), block_size[31:13]
    w = isa.fill_w1(buf_id=2, colour=1, ntgt=13, block_size=64)
    assert (w & 0x7) == isa.OP_FILL
    assert ((w >> 3) & 0xF) == 2
    assert ((w >> 8) & 0x1) == 1
    assert ((w >> 9) & 0xF) == 13          # ntgt > 7 exercises the widened field
    assert ((w >> 13) & 0x7FFFF) == 64


def test_task_word_roundtrip():
    # A feature-extraction TASK: acc=2, cfg=0, TGT buf 0 in slot5 ntgt=1
    w1 = isa.tw1(2, 0, 0, isa.M_UNUSED, 0, isa.M_UNUSED, 0, isa.M_UNUSED, 0)
    w2 = isa.tw2(isa.M_UNUSED, 0, 0, isa.M_UNUSED, 0, 0, isa.M_TGT, 0, 1)
    assert (w1 & 0x7) == isa.OP_TASK
    assert ((w1 >> 3) & 0x3) == 2          # acc_id
    assert ((w1 >> 5) & 0x7F) == 0         # cfg_id (7-bit field)
    assert (w2 & 0x3) == 0                 # sentinel
    assert ((w2 >> 22) & 0x3) == isa.M_TGT  # slot5 mode
    assert ((w2 >> 28) & 0xF) == 1          # slot5 #targets (4-bit field)


def test_task_word2_field_positions():
    # ntgt widened to 4 bits per long slot; slots 4/5 shifted up, filling to bit 31.
    # H1's N=2 usage (13) exercises the >7 range the 3-bit field could not hold.
    w2 = isa.tw2(isa.M_SRC, 3, 5, isa.M_RW, 10, 13, isa.M_TGT, 15, 7)
    assert (w2 & 0x3) == 0                   # sentinel [1:0]
    assert ((w2 >> 2) & 0x3) == isa.M_SRC    # slot3 mode
    assert ((w2 >> 4) & 0xF) == 3            # slot3 id
    assert ((w2 >> 8) & 0xF) == 5            # slot3 ntgt
    assert ((w2 >> 12) & 0x3) == isa.M_RW    # slot4 mode
    assert ((w2 >> 14) & 0xF) == 10          # slot4 id
    assert ((w2 >> 18) & 0xF) == 13          # slot4 ntgt (>7)
    assert ((w2 >> 22) & 0x3) == isa.M_TGT   # slot5 mode
    assert ((w2 >> 24) & 0xF) == 15          # slot5 id
    assert ((w2 >> 28) & 0xF) == 7           # slot5 ntgt (tops at bit 31)


def test_task_word1_field_positions():
    # cfg_id widened to 7 bits [11:5]; colour [12]; short slots shifted up by 2.
    w1 = isa.tw1(acc=1, cfg=100, colour=1,
                 m0=isa.M_SRC, b0=3, m1=isa.M_RW, b1=10, m2=isa.M_TGT, b2=15)
    assert (w1 & 0x7) == isa.OP_TASK
    assert ((w1 >> 3) & 0x3) == 1            # acc_id
    assert ((w1 >> 5) & 0x7F) == 100         # cfg_id (>31 exercises the widened field)
    assert ((w1 >> 12) & 0x1) == 1           # colour
    assert ((w1 >> 13) & 0x3) == isa.M_SRC   # slot0 mode
    assert ((w1 >> 15) & 0xF) == 3           # slot0 id
    assert ((w1 >> 19) & 0x3) == isa.M_RW    # slot1 mode
    assert ((w1 >> 21) & 0xF) == 10          # slot1 id
    assert ((w1 >> 25) & 0x3) == isa.M_TGT   # slot2 mode
    assert ((w1 >> 27) & 0xF) == 15          # slot2 id
    assert (w1 >> 31) == 0                    # reserved (bit 31)


def test_emit_addresses_and_listing():
    insts = [isa.Inst((0x11111111,), "one-word"),
             isa.Inst((0x22222222, 0x33333333), "two-word")]
    words, notes = isa.emit(insts)
    assert words == [0x11111111, 0x22222222, 0x33333333]
    assert notes[0].startswith("0000: 11111111")
    assert "(cont.)" in notes[2]
