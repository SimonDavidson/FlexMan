# SPDX-License-Identifier: MIT
# Author: Simon Davidson & Claude
# Created: 2026-06-11
# Last modified: 2026-06-11
#
# ANTI-DRIFT CHECK — the siren_detector e2e keeps ipsnn_-prefixed COPIES of the
# ipSnnAcc modules (needed because the closed-loop build elaborates snnAcc and
# ipSnnAcc module names side by side). Between 2026-05-30 and 2026-06-11 those
# copies received the six-RTL-bug fixes and the packed config decode while the
# standalone ipSnnAcc/ directory silently did not. This gate makes that class
# of drift a test failure:
#
#   1. each siren_detector/ipsnn_*.v must equal its ipSnnAcc/*.v counterpart
#      after normalising the module-name prefix, and
#   2. the modules that are shared verbatim across variant directories
#      (weight_generator, dataline_cache_with_xy, packer, ...) must stay
#      byte-identical to the snnAcc/ reference copies.
#
# annAcc keeps deliberately different weight_generator / dataline_cache
# variants (FE-specific), so it is only held to the four truly common files.
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]   # .../FlexMan

# (ipSnnAcc file, siren_detector prefixed copy)
PREFIX_PAIRS = [
    ("ipSnnAcc/acc_snn_processor.v",      "siren_detector/acc_ipsnn_processor.v"),
    ("ipSnnAcc/spike_processing.v",       "siren_detector/ipsnn_spike_processing.v"),
    ("ipSnnAcc/neuron_processing.v",      "siren_detector/ipsnn_neuron_processing.v"),
    ("ipSnnAcc/syn_curr_update.v",        "siren_detector/ipsnn_syn_curr_update.v"),
    ("ipSnnAcc/update_state_for_neuron.v","siren_detector/ipsnn_update_state_for_neuron.v"),
]

# The exact rename set used when the copies were made (siren -> ipSnnAcc).
RENAMES = [
    ("acc_ipsnn_processor",          "acc_snn_processor"),
    ("ipsnn_spike_processing",       "spike_processing"),
    ("ipsnn_neuron_processing",      "neuron_processing"),
    ("ipsnn_syn_curr_update",        "syn_curr_update"),
    ("ipsnn_update_state_for_neuron","update_state_for_neuron"),
]

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


def _normalised(rel_path: str) -> str:
    text = (REPO / rel_path).read_text()
    for old, new in RENAMES:
        text = text.replace(old, new)
    return text


def _first_diff_line(a: str, b: str) -> str:
    for n, (la, lb) in enumerate(zip(a.splitlines(), b.splitlines()), start=1):
        if la != lb:
            return f"line {n}: {la!r} != {lb!r}"
    return "files differ in length only"


def test_ipsnn_dir_matches_siren_copies():
    problems = []
    for ipsnn_rel, siren_rel in PREFIX_PAIRS:
        ours = (REPO / ipsnn_rel).read_text()
        theirs = _normalised(siren_rel)
        if ours != theirs:
            problems.append(
                f"{ipsnn_rel} != {siren_rel} (prefix-normalised); "
                f"{_first_diff_line(theirs, ours)}")
    assert not problems, (
        "ipSnnAcc/ has drifted from the siren_detector ipsnn copies — port the "
        "change to both (or re-sync): " + "; ".join(problems))


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
