# F8 fix — neuron_processing packer write-strobe (APPROVED 2026-06-21)

Authors: Simon Davidson & Claude · Created 2026-06-21 · Last modified 2026-06-21

Stage 1 (hw-plan) done + 4-lens critique folded. Simon approved 2026-06-21 with:
full per-variant TB guard; add `!buffer_full` guard to packer.v (all 6 copies);
clone tb_packer_cadence into fmiSnnAccMC.

## Root cause (CONFIRMED)
Every neuron_processing variant wires the writeback packers `.pak_write_i(result_valid)`.
`result_valid` is a LEVEL held high for many cycles whenever any packer is
back-pressured (`result_taken = result_valid & ~all *_full`). `packer.v`
OR-accumulates `output_buffer` on EVERY cycle `pak_write_i` is high, so under
sustained writeback memory back-pressure it (a) OR-accumulates the next neuron's
field into a not-yet-flushed sub-32-bit word, and (b) for the 1-bit spike packer
and 32-bit packers, re-accumulates/re-flushes a word it already flushed —
overwriting the correct word. Affects the spike packer even in all-32-bit configs.
Unreachable by existing tbs (all hard-wire write-side mem_wait=0).

Evidence: fmiSnnAcc/tb_packer_cadence.v — pak_write_i=result_valid → 70 fails;
=result_taken → 0 fails (628 checks).

## Fix
### RTL — driver (one line per packer; ada keeps `& has_ada_i`)
- snnAcc/neuron_processing.v        505/527/549  result_valid→result_taken (aligned col)
- ipSnnAcc/neuron_processing.v      503/525/547  result_valid→result_taken (aligned col)
- annAcc/neuron_processing.v        404/429      result_valid→result_taken (spike+pot only)
- fmiSnnAcc/neuron_processing.v     610/624/638  →result_taken; 656 →result_taken & has_ada_i
- fmiSnnAccMC/neuron_processing.v   identical (byte-identical to fmiSnnAcc)

### RTL — defensive guard in packer.v (all 6 copies)
output_buffer accumulate: `else if (pak_write_i)` → `else if (pak_write_i && !buffer_full)`.
Behaviour-identical given the driver fix (full ⇒ result_taken=0 ⇒ pak_write_i=0);
closes D1 at the root. 5 SNN/ANN copies identical (line 106); Hadamard differs
(line 104, same logic) — apply equivalent, run Hadamard tbs.

### Verification (full per-variant guard)
- verif/sram_bfm.vh: add wait-honouring write macros (commit gated by ~wait).
- All 5 tb_neuron_processing.v: inject write-side back-pressure (at least the
  write-only spike mem; +syn/pot/ada with read-first/wait-on-write on the shared
  pin), keep an N>32 case. Each MUST fail on as-built RTL, pass on fixed RTL
  (verify discriminating by running against as-built before applying the fix).
- fmiSnnAcc/tb_packer_cadence.v: DELETE the as-built (use_taken=0) pass (check_eq_u
  hard-prints bare FAIL that run_regression.sh greps); keep result_taken path; add
  a "exactly one accumulate per accepted neuron" strobe-count assertion; pinned seed.
- Clone tb_packer_cadence.v + tbPackerCadence.bsh into fmiSnnAccMC.
- STA sanity on fmiSnnAcc (has_ada) — result_taken now feeds packer flop-enables
  behind the ~*_full cone (added depth/fan-out).
- HARD GATE: every variant unit+acc-level+shared tbPacker green; Hadamard green;
  ./run_regression.sh clean vs baseline.

### Docs
- verif/FINDINGS.md: F8 SUSPECTED→FIXED 2026-06-21; corrected (broader than
  sub-32-bit; spike packer in all-32-bit); supersede D1. No SPDX-header date edits
  in the RTL (those files have no date line) — record in commit message + FINDINGS.

## Not cycle-identical caveat
At 32-bit/zero-wait result_valid≠result_taken cycle-for-cycle; existing 32-bit
suites stay green because the as-built re-OR is masked (idempotent before clear).
Correctness rests on memory-image equivalence + regressions, not identical waveform.
