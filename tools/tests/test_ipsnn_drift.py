# SPDX-License-Identifier: MIT
# Author: Simon Davidson & Claude
# Created: 2026-06-11
# Last modified: 2026-06-12
#
# ANTI-DRIFT CHECK — modules shared verbatim across the variant accelerator
# directories (weight_generator, dataline_cache_with_xy, packer, ...) must stay
# byte-identical to the snnAcc/ reference copies. ipSnnAcc and fmiSnnAcc are held
# to the full shared set; annAcc keeps deliberately different weight_generator /
# dataline_cache variants (FE-specific), so it is held only to the common subset.
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]   # .../FlexMan

# Files shared verbatim, snnAcc/ is the reference copy.
SHARED_VS_SNN = {
    "ipSnnAcc":  ["weight_generator.v", "dataline_cache_with_xy.v", "packer.v",
                  "act_index_generator.v", "slice_and_align.v", "sram_bram.v"],
    "fmiSnnAcc": ["weight_generator.v", "dataline_cache_with_xy.v", "packer.v",
                  "act_index_generator.v", "slice_and_align.v", "sram_bram.v"],
    # annAcc has FE-specific weight_generator / dataline_cache variants
    "annAcc":    ["packer.v", "act_index_generator.v", "slice_and_align.v",
                  "sram_bram.v"],
}


def test_shared_modules_byte_identical_to_snnacc():
    problems = []
    for variant, files in SHARED_VS_SNN.items():
        for f in files:
            ref  = (REPO / "snnAcc" / f).read_bytes()
            copy = (REPO / variant / f).read_bytes()
            if ref != copy:
                problems.append(f"{variant}/{f} != snnAcc/{f}")
    assert not problems, (
        "shared-module copies have drifted from snnAcc/ — apply the fix to "
        "every variant directory: " + "; ".join(problems))
