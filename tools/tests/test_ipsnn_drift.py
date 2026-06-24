# SPDX-License-Identifier: MIT
# Author: Simon Davidson & Claude
# Created: 2026-06-11
# Last modified: 2026-06-24
#
# ANTI-DRIFT CHECK — modules duplicated verbatim across the variant accelerator
# directories must stay byte-identical to the snnAcc/ reference copies.
#
# The 2026-06 refactor HOISTED dataline_cache_with_xy.v / packer.v /
# slice_and_align.v / sram_bram.v into shared/ — a single canonical copy consumed
# by every variant — so those are no longer a duplication/drift risk. They are
# checked instead by test_hoisted_modules_are_single_copy_in_shared (must live
# only in shared/). The files that remain per-variant — weight_generator.v and
# act_index_generator.v — are still held to the snnAcc/ reference. annAcc keeps a
# deliberately FE-specific weight_generator, so it is held only to the common
# act_index_generator.v.
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]   # .../FlexMan

# Files still duplicated per-variant; snnAcc/ is the reference copy.
SHARED_VS_SNN = {
    "ipSnnAcc":  ["weight_generator.v", "act_index_generator.v"],
    "fmiSnnAcc": ["weight_generator.v", "act_index_generator.v"],
    # annAcc has a deliberately FE-specific weight_generator
    "annAcc":    ["act_index_generator.v"],
}

# Files hoisted to shared/: a single canonical copy (present in shared/, absent
# from every variant dir) so there is nothing left to drift.
SHARED_HOISTED = ["dataline_cache_with_xy.v", "packer.v",
                  "slice_and_align.v", "sram_bram.v"]
VARIANT_DIRS = ["snnAcc", "ipSnnAcc", "fmiSnnAcc", "annAcc"]


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


def test_hoisted_modules_are_single_copy_in_shared():
    # Each hoisted module must exist in shared/ and NOT linger as a stale
    # per-variant duplicate (which would silently re-introduce drift).
    problems = []
    for f in SHARED_HOISTED:
        if not (REPO / "shared" / f).is_file():
            problems.append(f"shared/{f} missing (expected the canonical copy)")
        for d in VARIANT_DIRS:
            if (REPO / d / f).exists():
                problems.append(f"{d}/{f} is a stale duplicate — consume shared/{f} instead")
    assert not problems, (
        "hoisted-module single-copy invariant broken: " + "; ".join(problems))
