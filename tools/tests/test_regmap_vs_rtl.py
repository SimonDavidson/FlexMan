# SPDX-License-Identifier: MIT
# Author: Simon Davidson & Claude
# Created: 2026-06-08
# Last modified: 2026-06-08
#
# ANTI-DRIFT CHECK — the reason the back-end lives in the FlexMan repo.
# Parses the `8'hXX: <signal> <= ...` config-register decoders straight out of the
# RTL and asserts every offset the tool-chain emits is decoded by the hardware to
# the matching register. A separate-repo tool-chain cannot do this; here, an RTL
# register/offset change that isn't mirrored in regmap.py fails the test.
import re
from pathlib import Path

from flexman_backend import regmap

REPO = Path(__file__).resolve().parents[2]   # .../FlexMan
_RE = re.compile(r"8'h([0-9A-Fa-f]{1,2})\s*:\s*([A-Za-z_]\w*)")

# (python offset map, verilog decoder, LHS signal suffix stripped to match keys)
CASES = [
    (regmap.ANN_REG_OFFSETS, "annAcc/acc_snn_processor.v", "_r"),
    (regmap.SNN_REG_OFFSETS, "snnAcc/acc_snn_processor.v", "_r"),
    (regmap.HU_REG_OFFSETS,  "Hadamard/hu_config_regs.v",  "_r_o"),
]


def _rtl_pairs(rel_path, suffix):
    """Return the set of (register_base_name, offset_int) decoded by the RTL."""
    pairs = set()
    for line in (REPO / rel_path).read_text().splitlines():
        m = _RE.search(line)
        if not m:
            continue
        off = int(m.group(1), 16)
        sig = m.group(2)
        if sig.endswith(suffix):
            sig = sig[: -len(suffix)]
        pairs.add((sig, off))
    return pairs


def test_every_python_offset_is_decoded_by_rtl():
    for pymap, rel_path, suffix in CASES:
        pairs = _rtl_pairs(rel_path, suffix)
        assert len(pairs) > 8, f"parsed too few decode lines from {rel_path}"
        missing = [(name, hex(off)) for name, off in pymap.items()
                   if (name, off) not in pairs]
        assert not missing, (
            f"{rel_path}: regmap entries not matched by RTL decode (drift!): {missing}"
        )


def test_known_anchor_offsets():
    # Spot anchors so a wholesale parse failure can't pass silently.
    assert regmap.ANN_REG_OFFSETS["np_thresh_op"] == 0xA0
    assert regmap.ANN_REG_OFFSETS["np_out_bin_point"] == 0xA4
    assert "np_mode" not in regmap.ANN_REG_OFFSETS
    assert regmap.SNN_REG_OFFSETS["np_mode"] == 0x98
    assert regmap.HU_REG_OFFSETS["src_z_base_addr"] == 0x0C
