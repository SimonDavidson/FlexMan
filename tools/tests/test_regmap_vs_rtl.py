# SPDX-License-Identifier: MIT
# Author: Simon Davidson & Claude
# Created: 2026-06-08
# Last modified: 2026-06-12
#
# ANTI-DRIFT CHECK — the reason the back-end lives in the FlexMan repo.
# Parses the packed per-task config decoders (case (sys_addr_i[7:0]) ...)
# straight out of the RTL and asserts the regmap.PACKED_* layout matches:
# every packed field is decoded at the right word offset and bit lsb, the
# boot-only conv/sparse registers sit at their BOOT_REG_OFFSETS, and no stale
# in-window offsets remain. An RTL layout change that isn't mirrored in
# regmap.py fails here.
import re
from pathlib import Path

from flexman_backend import regmap

REPO = Path(__file__).resolve().parents[2]   # .../FlexMan

# Fields each accelerator variant does NOT decode (packs as 0 / ignored).
SKIP_FIELDS = {
    "snn":   {"sp_act_sz", "np_thresh_op", "np_out_bin_point", "act_signed",
              "np_lut_window", "np_out_signed"},
    "ipsnn": {"sp_act_sz", "np_thresh_op", "np_out_bin_point", "act_signed",
              "np_lut_window", "np_out_signed"},
    "ann":   {"np_syn_curr_decay_mult", "np_mode"},
}

CASES = [
    ("snn",   "snnAcc/acc_snn_processor.v"),
    ("ipsnn", "ipSnnAcc/acc_snn_processor.v"),
    ("ann",   "annAcc/acc_snn_processor.v"),
]

# fmiSnnAcc has its own 16-word packed layout (regmap.PACKED_FMI_*).
FMI_RTL = "fmiSnnAcc/acc_fmiSnn_processor.v"

_RE_CASE   = re.compile(r"8'h([0-9A-Fa-f]{1,2})\s*:")
_RE_ASSIGN = re.compile(r"([A-Za-z_]\w*?)_r\s*<=\s*sys_data_i\s*\[([^\]]+)\]")


def _slice_lsb(slice_text: str) -> int:
    """lsb of a sys_data_i bit-slice: '31:0' / '15:10' -> lo, 'X-1:0' -> 0,
    '16 +: W' -> 16, single bit 'N' -> N."""
    s = slice_text.strip()
    m = re.match(r"^(\d+)\s*\+:", s)
    if m:
        return int(m.group(1))
    if ":" in s:
        lo = s.rsplit(":", 1)[1].strip()
        return int(lo) if lo.isdigit() else 0   # parameterised lo is always 0
    return int(s) if s.isdigit() else 0


def _rtl_decode(rel_path):
    """Return {offset: {field_name: lsb}} parsed from the config decoder."""
    decode: dict[int, dict[str, int]] = {}
    offset = None
    in_case = False
    for line in (REPO / rel_path).read_text().splitlines():
        if "case (sys_addr_i[7:0])" in line:
            in_case = True
            continue
        if not in_case:
            continue
        if "endcase" in line:
            break
        m = _RE_CASE.search(line)
        if m:
            offset = int(m.group(1), 16)
        m = _RE_ASSIGN.search(line)
        if m and offset is not None:
            decode.setdefault(offset, {})[m.group(1)] = _slice_lsb(m.group(2))
    return decode


def _expected_fields(acc):
    """Return {offset: {field_name: lsb}} built from regmap.PACKED_*."""
    skip = SKIP_FIELDS[acc]
    exp: dict[int, dict[str, int]] = {}
    for i, name in enumerate(regmap.PACKED_ADDR_WORDS):
        if name not in skip:
            exp[i * 4] = {name: 0}
    s_base = len(regmap.PACKED_ADDR_WORDS)
    for i, (low, high) in enumerate(regmap.PACKED_SIZE_WORDS):
        fields = {}
        if low and low not in skip:
            fields[low] = 0
        if high and high not in skip:
            fields[high] = 16
        exp[(s_base + i) * 4] = fields
    m_base = s_base + len(regmap.PACKED_SIZE_WORDS)
    for i, packing in enumerate(regmap.PACKED_MODE_WORDS):
        exp[(m_base + i) * 4] = {name: lsb for name, lsb, _w in packing
                                 if name not in skip}
    return exp


def test_packed_layout_matches_rtl_decode():
    for acc, rel_path in CASES:
        decode = _rtl_decode(rel_path)
        assert len(decode) > 8, f"parsed too few decode lines from {rel_path}"
        problems = []
        for off, fields in _expected_fields(acc).items():
            for name, lsb in fields.items():
                got = decode.get(off, {})
                if name not in got:
                    problems.append(f"{name} not decoded at 0x{off:02X}")
                elif got[name] != lsb:
                    problems.append(
                        f"{name}@0x{off:02X}: lsb {got[name]} != regmap {lsb}")
        assert not problems, f"{rel_path}: packed-layout drift: {problems}"


def test_boot_regs_match_rtl_decode():
    for acc, rel_path in CASES:
        decode = _rtl_decode(rel_path)
        missing = [(name, hex(off))
                   for name, off in regmap.BOOT_REG_OFFSETS.items()
                   if name not in decode.get(off, {})]
        assert not missing, f"{rel_path}: boot regs not decoded: {missing}"


def test_no_stale_offsets_in_rtl():
    # Window offsets must be exactly the 15 packed words (minus the variant's
    # skipped whole words); anything else <0x3C is a stale scattered-decode
    # leftover. Out-of-window offsets must all be known boot regs.
    boot_offs = set(regmap.BOOT_REG_OFFSETS.values())
    for acc, rel_path in CASES:
        decode = _rtl_decode(rel_path)
        expected_window = {off for off, fields in _expected_fields(acc).items()
                           if fields}
        got_window = {off for off in decode if off < 0x3C}
        assert got_window == expected_window, (
            f"{rel_path}: in-window decode mismatch: "
            f"unexpected={sorted(hex(o) for o in got_window - expected_window)} "
            f"missing={sorted(hex(o) for o in expected_window - got_window)}")
        stray = {off for off in decode if off >= 0x3C} - boot_offs
        assert not stray, (
            f"{rel_path}: non-boot out-of-window offsets decoded: "
            f"{sorted(hex(o) for o in stray)}")


def _expected_fields_fmi():
    """Return {offset: {field_name: lsb}} built from regmap.PACKED_FMI_*."""
    exp: dict[int, dict[str, int]] = {}
    for i, name in enumerate(regmap.PACKED_FMI_ADDR_WORDS):
        exp[i * 4] = {name: 0}
    s_base = len(regmap.PACKED_FMI_ADDR_WORDS)
    for i, (low, high) in enumerate(regmap.PACKED_FMI_SIZE_WORDS):
        fields = {low: 0}
        if high:
            fields[high] = 16
        exp[(s_base + i) * 4] = fields
    m_base = s_base + len(regmap.PACKED_FMI_SIZE_WORDS)
    for i, packing in enumerate(regmap.PACKED_FMI_MODE_WORDS):
        exp[(m_base + i) * 4] = {name: lsb for name, lsb, _w in packing}
    return exp


def test_fmi_packed_layout_matches_rtl_decode():
    decode = _rtl_decode(FMI_RTL)
    assert len(decode) > 8, f"parsed too few decode lines from {FMI_RTL}"
    problems = []
    for off, fields in _expected_fields_fmi().items():
        for name, lsb in fields.items():
            got = decode.get(off, {})
            if name not in got:
                problems.append(f"{name} not decoded at 0x{off:02X}")
            elif got[name] != lsb:
                problems.append(
                    f"{name}@0x{off:02X}: lsb {got[name]} != regmap {lsb}")
    assert not problems, f"{FMI_RTL}: packed-layout drift: {problems}"


def test_fmi_boot_regs_match_rtl_decode():
    decode = _rtl_decode(FMI_RTL)
    missing = [(name, hex(off))
               for name, off in regmap.BOOT_REG_OFFSETS_FMI.items()
               if name not in decode.get(off, {})]
    assert not missing, f"{FMI_RTL}: boot regs not decoded: {missing}"


def test_fmi_no_stale_offsets_in_rtl():
    # fmi fills the whole 16-word window (0x00..0x3C); anything else must be a
    # known fmi boot reg.
    decode = _rtl_decode(FMI_RTL)
    expected_window = set(_expected_fields_fmi())
    got_window = {off for off in decode if off < 0x40}
    assert got_window == expected_window, (
        f"{FMI_RTL}: in-window decode mismatch: "
        f"unexpected={sorted(hex(o) for o in got_window - expected_window)} "
        f"missing={sorted(hex(o) for o in expected_window - got_window)}")
    stray = ({off for off in decode if off >= 0x40}
             - set(regmap.BOOT_REG_OFFSETS_FMI.values()))
    assert not stray, (
        f"{FMI_RTL}: non-boot out-of-window offsets decoded: "
        f"{sorted(hex(o) for o in stray)}")


def test_hadamard_offsets_decoded_by_rtl():
    # HU keeps its scattered per-register decode (hu_config_regs.v).
    pairs = set()
    rx = re.compile(r"8'h([0-9A-Fa-f]{1,2})\s*:\s*([A-Za-z_]\w*)")
    for line in (REPO / "Hadamard/hu_config_regs.v").read_text().splitlines():
        m = rx.search(line)
        if m:
            sig = m.group(2)
            if sig.endswith("_r_o"):
                sig = sig[:-4]
            pairs.add((sig, int(m.group(1), 16)))
    assert len(pairs) > 8, "parsed too few decode lines from hu_config_regs.v"
    missing = [(name, hex(off)) for name, off in regmap.HU_REG_OFFSETS.items()
               if (name, off) not in pairs]
    assert not missing, f"hu_config_regs.v: HU regmap drift: {missing}"


def test_known_anchor_offsets():
    # Spot anchors so a wholesale parse failure can't pass silently.
    assert regmap.PACKED_WORDS_PER_CONFIG == 15
    assert regmap.PACKED_ADDR_WORDS[0] == "sp_act_base_addr"
    assert regmap.PACKED_ADDR_WORDS.index("np_pot_decay_mult") == 8   # 0x20
    assert regmap.PACKED_SIZE_WORDS[2] == ("sp_rows_per_neuron", "np_last_neuron_idx")
    m1 = {name: (lsb, w) for name, lsb, w in regmap.PACKED_MODE_WORDS[1]}
    assert m1["bin_point_syn_curr"] == (10, 6)
    assert m1["np_out_bin_point"] == (16, 6)
    assert regmap.BOOT_REG_OFFSETS["sp_weight_idx_sz"] == 0x5C
    assert regmap.HU_REG_OFFSETS["src_z_base_addr"] == 0x20   # w8 (one reg/word)
    # fmi anchors
    assert regmap.PACKED_FMI_WORDS_PER_CONFIG == 16
    assert regmap.PACKED_FMI_ADDR_WORDS[6] == "np_dcy_syn_base_addr"   # 0x18
    assert regmap.PACKED_FMI_ADDR_WORDS[11] == "np_scl_ada_base_addr"  # 0x2C
    assert regmap.PACKED_FMI_SIZE_WORDS[0] == ("sp_in_x_len", "sp_out_x_len")
    fmi_m0 = {name: (lsb, w) for name, lsb, w in regmap.PACKED_FMI_MODE_WORDS[0]}
    assert fmi_m0["np_has_ada"] == (30, 1)
    assert regmap.BOOT_REG_OFFSETS_FMI["sp_weight_idx_sz"] == 0x5C
