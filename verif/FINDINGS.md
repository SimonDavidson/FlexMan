# FlexMan verification findings

Confirmed issues surfaced by the aggressive test suite (`verif/` library + per-module
testbenches). Each entry: what, where, evidence, impact, status.

Authors: Simon Davidson & Claude · Created 2026-06-07 · Last modified 2026-06-30

---

## F1 — Negative synaptic-current decay is wrong in snnAcc / ipSnnAcc / annAcc (signed×unsigned multiply)

**Where:** `snnAcc/update_state_for_neuron.v` (`mul_result = mul_a * mul_b`, mul_a
`wire signed`, mul_b `wire` unsigned, no `$signed` cast). Same form in
`ipSnnAcc/update_state_for_neuron.v` and `annAcc/update_state_for_neuron.v`.

**What:** Decaying a *negative* value with the Q0.32 decay multiplier uses a
signed×unsigned multiply, so Verilog treats the signed operand as unsigned. The
high 32 bits of the product are then wrong for negative inputs.

**Evidence (`verif/tb_np_ref_selftest.v`, 2026-06-07):**
```
decay(-1000, 0.5):  DUT syn_curr_o = 2147483148 (0x7ffffe0c)
                    ideal signed   =       -500 (0xfffffe0c)
```
A current of −1000 decayed by 0.5 should be −500; the RTL yields ~+2.1e9.

**Why it's almost certainly a bug, not intent:** `neuron_model.md` states all values
are right-justified two's-complement and decays are `x·mult >> 32` — i.e. signed.
The **FMI** variant (`fmiSnnAcc/update_state_for_neuron.v`) does the same decay with
an explicit `$signed(mul1_a) * $signed({1'b0, mul1_b})`, i.e. it was deliberately
written to be signed-correct — implying the snn/ann form is an oversight.

**Impact:** The recurrent (snnAcc) and encoder (ipSnnAcc) layers can hold negative
synaptic currents (`W·x` with negative weights). Those currents are decayed every
timestep; a negative current would be corrupted into a large positive injection,
distorting membrane dynamics. feature_extraction (annAcc) is bias-free and applies
RELU/ABS to the accumulator, but its `decayed_act` path also uses the unsigned form
(decays the non-negative threshold result, so unaffected in practice).

**Suggested fix:** cast the data operand to signed in the snn/ann multiply, matching
the FMI form: `mul_result = $signed(mul_a) * $signed({1'b0, mul_b});` (or declare
`mul_b` so the product is evaluated signed). Needs Simon's sign-off — flagged, not
auto-fixed.

**Status:** FIXED 2026-06-07. Applied the fmiSnnAcc form in all three modules:
`assign mul_result = $signed(mul_a) * $signed({1'b0, mul_b});`
(`snnAcc`/`ipSnnAcc`/`annAcc` update_state). The goldens (`np_ref_lif`,
`np_ann_decay`) now use `np_decay_signed`, and the directed negative-decay probe
is a HARD check (`decay(-1000,0.5) == -500`). Side-effect: the saturation-quirk
decayed value legitimately changes from `+2^30` to `-2^30` (decaying the clamped
`0x80000000` signed) — test expectations updated. Verified: all neuron unit tests
(snn/ipsnn/ann/fmi) and all four accelerator-level integration tests PASS.
annAcc is functionally unchanged (its `act_out` is non-negative) but kept
consistent. Positive-operand behaviour is unchanged everywhere.

---

## F2 — ipSnnAcc tb_update_state_for_neuron FAILS in the baseline (X on spike path)

**Where:** `ipSnnAcc/update_state_for_neuron.v` + `ipSnnAcc/tb_update_state_for_neuron.v`.

**What:** `ipSnnAcc/update_state_for_neuron.v` is still the OLDER **2-cycle** pipeline
(`state_cycle2_r`), whereas `snnAcc` was upgraded to the **3-cycle** timing-closed
version. The ipSnnAcc unit test still uses fixed-timing sampling (one edge after
valid) rather than polling `result_valid_o`, and currently reports:
```
FAIL T2 potential_o reset to 0: got X
FAIL T3 potential_o = 0 on spike:  got X
```
i.e. `potential_o` is X on the spike path. The snnAcc equivalent PASSES.

**Impact:** Either a stale test (sampling the 2-cycle pipeline at the wrong instant)
or a genuine uninitialised-register bug on the ipSnnAcc spike path. Project memory
claims ipSnnAcc tests passed 2026-05-08; this is the first run since under the new
regression harness.

**Status:** RESOLVED 2026-06-07 — NOT an RTL bug. The new
`ipSnnAcc/tb_update_state_for_neuron` (pipeline-depth-agnostic `VT_CONSUME`
driver + `np_ref_lif` golden) PASSES: 9012 checks, 0 failures over 3000 random
vectors; `tb_neuron_processing` PASSES too (282 checks). The 2-cycle pipeline is
functionally correct. The old failure was a STALE-TEST timing bug — fixed-cycle
sampling read `potential_o` one edge after `neuron_valid` (before `result_valid`
on the spike path), so it latched X. The polling driver samples at the right
instant.

---

## F3 — flexman_fpga_wrap.v out of sync with flexman.v (RESOLVED)

**Where:** `top_module/flexman_fpga_wrap.v` instantiating `top_module/flexman.v`.

**What:** `elab_fpga_wrap.bsh` fails elaboration:
```
*E,CUVPOM (flexman_fpga_wrap.v,385): Port name 's0_act_mem_req_o' of instance
 'u_flexman' is invalid ...   (and many more s0_*_mem_* ports)
```
The FPGA wrapper connects per-accelerator memory ports (`s0_act_mem_req_o`, etc.)
that no longer exist on the current `flexman.v` — the top-level memory interface was
reworked but the wrapper was not updated to match.

**Impact:** The FPGA synthesis wrapper cannot be built against the current top level.
`elab_flexman.bsh` (the core top) still elaborates clean, so only the wrapper is stale.

**Status:** RESOLVED 2026-06-23 — the FPGA wrapper was re-synced to `flexman.v`'s
reworked top-level memory interface. `elab_fpga_wrap.bsh` now elaborates clean
(exit 0, 0 errors, the `s0_*_mem_*` CUVPOM port mismatches are gone), and
`run_regression.sh` scores `top_module/elab_fpga_wrap` PASS(elab). Fixed outside
the test-coverage uplift (it was an RTL-sync change on the top-level interface).

---

## Baseline regression snapshot (2026-06-07, before coverage uplift)

`./run_regression.sh` over the pre-existing suite: **40 passed / 7 failed / 47 total.**
The 7 failures, by category:

| Test | Kind | Note |
|------|------|------|
| annAcc/tbSpikeProc | stale .bsh | tb instantiates `spike_processing`; annAcc module is `ann_spike_processing` |
| scheduler/myBuffState | stale .bsh | references missing `restore_buff.tcl` (GUI) |
| scheduler/myProgCache | stale .bsh | wrong path to `constants.v` |
| scheduler/mySchProg | stale .bsh | references `tb_scheduler_prog.v` (now under old_tb/) |
| ipSnnAcc/tbUpdateState | RTL/test (F2) | X on spike path (2-cycle pipeline) |
| ipSnnAcc/tbNeuronProc | RTL/test (F2) | same X propagates to neuron level |
| top_module/elab_fpga_wrap | RTL (F3) | wrapper port mismatch |

The four stale-.bsh failures are test-infrastructure rot and will be repaired as their
modules are reworked (scheduler in Phase 2C, annAcc spike in Phase 4). F2/F3 are
reported for decision.

---

## F4 — hu_compute only multiplies the TOP BYTE of Z for 16/32-bit elements

**Where:** `Hadamard/hu_compute.v`, `z_chunk` extraction + the ST_MUL recurrence.

**What:** The iterative multiply is meant to process Z in 8-bit chunks (1 cycle for
≤8-bit, 2 for 16-bit, 4 for 32-bit). But the chunk source is:
```verilog
wire [7:0] z_chunk;
assign z_chunk = z_r[DATA_BITS-1 -: 8] >> ({mul_count, 3'b000});
```
`z_r[31:24]` is only the **top byte** of Z; right-shifting that 8-bit value by
`mul_count*8` yields **0** for every `mul_count ≥ 1`. So in 16-bit and 32-bit Z
modes, chunks 1..(mul_total-1) are all zero and **only the top byte of Z** ever
multiplies `amb`. A 32-bit Z therefore contributes at most its high 8 bits; the low
24 bits are silently dropped. (For ≤8-bit Z, mul_total==1 and the single chunk is
correct, so 1..8-bit modes are unaffected.)

To get the chunk at position `mul_count` the code should index the *full* `z_r`,
e.g. `z_r >> (DATA_BITS - 8 - mul_count*8)` (left-aligned) and mask to 8 bits — not
shift the already-isolated top byte.

**Evidence:** `Hadamard/tb_hu_compute.v` (2026-06-07). The SW golden deliberately
reproduces this exact expression and the DUT matches **bit-for-bit over 20045
checks** including directed 16-bit/32-bit cases (D7/D8) and thousands of random
vectors with `elem_sz_z ∈ {0..5}`. The match across 16/32-bit sizes is itself the
proof that the high chunks contribute nothing — an idealised full-width multiply
golden would diverge there.

**Impact:** Hadamard results are wrong (loss of Z precision / magnitude) whenever Z
elements are wider than 8 bits. The existing `tb_hadamard_unit` only exercises
8-bit elements, so it never caught this. Functionally fine for ≤8-bit Z.

**Status:** FIXED 2026-06-07 (with F5, and a deeper Z-scaling root cause). See the
combined fix note under F5 below.

---

## F5 — hu_compute accumulator shift applies to the WHOLE running sum (operator precedence)

**Where:** `Hadamard/hu_compute.v`, ST_MUL accumulator update.

**What:** The partial-product accumulate is written:
```verilog
accumulator <= accumulator + (z_chunk_sext * amb_sext) << ({mul_count, 3'b000});
```
In Verilog, `+` binds **tighter** than `<<`, so this parses as
`(accumulator + (z_chunk*amb)) << (mul_count*8)` — the entire running sum is shifted
left each ST_MUL cycle, not just the new partial product. The evident intent (a
chunked multiply that shifts each partial product into place) would need
`accumulator + ((z_chunk*amb) << (mul_count*8))`. Combined with F4 (higher chunks
are zero), the only cycle that matters is `mul_count==0` (shift 0), so for the
current single-effective-chunk behaviour the result still comes out as `z_top*amb`;
the mis-shift is latent but would corrupt results immediately if F4 were fixed
without also fixing the parenthesisation.

**Evidence:** modelled exactly in `tb_hu_compute.v`'s golden (`(acc + chunk*amb) <<
(mc*8)`); 20045/20045 checks pass, confirming the as-built precedence.

**Impact:** Latent. Harmless today only because F4 zeroes the higher chunks. Both F4
and F5 must be fixed together for a correct wide-Z multiply.

**Status:** FIXED 2026-06-07, together with F4 and a DEEPER root cause uncovered
while planning the fix:

**Root cause (bigger than F4/F5):** Z was never aligned to the internal binary
point (HALF=24) the way A and B are, so `Z*(A-B)` landed at binary point
`bp_z+24` while B/R_prev were added at 48 and `out_shift` assumed 48 — the
multiply term was shifted into oblivion. hu_compute effectively output `B
(+R_prev)`, IGNORING `Z*(A-B)` (directed Z=2,A=5,B=1 gave R=1, not 9). The old
unit test passed only because its golden was a characterisation model
(DUT==golden), never checking true arithmetic; `tb_hadamard_unit` only used
Z=0 / A=B cases, so it never multiplied either.

**Fix (`Hadamard/hu_compute.v`):**
- F4: chunk the RIGHT-aligned integer `z_val` LSB-first; sign-extend the top
  chunk only, zero-extend lower chunks (exact two's-complement `z_val*amb`).
- F5: shift the partial product, not the running sum
  (`acc + ((chunk*amb) << (mc*8))`).
- Z-scaling: add B/R_prev at `<< bp_z` (was `<< HALF`) and
  `out_shift = HALF + bp_z - bp_r` (was `48 - bp_r`).
- Rounding: round-half-up before the output shift (Simon's choice).
- Robustness: `mul_total` derived from the latched `esz_z_r`.

**Verification:** `Hadamard/tb_hu_compute.v` golden rewritten to the IDEAL math;
directed cases now ASSERT the true value (D4=9, D5=127+over, D6=-128+under,
D7=260, D8=1993, D9=6 at bp=2) plus 4000 random vectors — 20056 checks, 0
failures. `tb_hadamard_unit` still PASSES (its cases have a zero multiply term).

**Residual (documented, not fixed):** WIDE=48 / ACC=84 hold for tested ranges;
a large 32-bit element at a small binary point can overflow the 48-bit aligned
intermediate — a pre-existing width limit, flagged for a future dimensioning
pass.

---

## F6 — full-mode spike_processing accumulates with ALL-ZERO activations (RESOLVED)

**Where:** `snnAcc/spike_processing.v` — `act_data_gated_valid = act_data_valid &
act_data_out[ACT_BITS-1]` and the act-index "ignore non-spike" path.

**What:** With every activation bit set to 0 (no input spikes), full-mode
spike_processing still accumulates the weights into syn_curr as if all inputs
spiked. On a fresh-reset 2×2→2×2 run with uniform weight w=5, every output
syn_curr became 20 (= 4·w) instead of 0.

**Evidence (`snnAcc/tb_spike_processing.v`, 2026-06-07):** the all-spike golden
`syn_curr[j] = init[j] + nin·w` matches the DUT bit-for-bit across grids/widths
(820 checks). The dedicated no-spike probe shows the DUT accumulating `nin·w`
with all-zero activations (reproducible after reset + read-data reg init). The
old `tb_spike_processing` used all-1 activations, so the spike value never varied
and this was never exercised.

**Open question:** bug, or intended "full = dense, process every input" semantics
(where the activation gate is only meant to matter in sparse/event mode)? The RTL
top-level gating reads as if it should suppress zero activations in all modes.

**Impact:** If a dense layer is ever fed genuinely sparse (mostly-zero) binary
activations expecting event-driven suppression, every input would be processed
regardless. Needs Simon's intent confirmation.

**Status:** RESOLVED 2026-06-23 — settled as a real CORRECTNESS bug (NOT a
performance issue): all-zero activations produced `syn_curr = nin·w` instead of 0
(wrong value, not merely wasted work). Fixed by the activation-gate rework
(`act_data_gated_valid` / `act_ignore_non_spike`, 2026-06-10) which suppresses
non-spike accumulation in full mode too. The snnAcc `tb_spike_processing` no-spike
probe now reports "full mode correctly suppresses accumulation for zero activations"
(syn=0; 820 checks, 0 failures, re-confirmed 2026-06-23), and the gating line is
identical across all five variants (incl. fmiSnnAccMC). The earlier bug-vs-intent
open question is closed: it was a bug, now fixed.

---

## New tests added (2026-06-07 coverage uplift — shared/ + Hadamard/)

First-ever unit tests for six previously-untested modules; all PASS against
as-built RTL:

| Test | Module | Checks | SW-golden approach |
|------|--------|-------:|--------------------|
| shared/tbBramSp     | bram_sp     |  4009 | read-first `model[]` array, OLD value sampled pre-write each cycle |
| shared/tbBramSdp    | bram_sdp    |  4005 | independent r/w addr; collision (raddr==waddr) returns OLD |
| shared/tbBramTdp    | bram_tdp    |  8009 | two ports, both OLD reads sampled before either write commits; en-gated dout HOLD modelled; same-addr dual-write avoided (HW race) |
| shared/tbSharedPool | shared_pool | 66077 | full priority-arbiter replica + 1-cycle registered rdsel read-return; banks modelled w/ bank_wait gating |
| Hadamard/tbHuCompute| hu_compute  | 20045 | 84-bit bit-exact datapath model incl. F4/F5 quirks; directed clamp/binpoint + random |
| Hadamard/tbHuConfigRegs | hu_config_regs | 62 | per-offset write/readback + width-trunc, combinational ack, addr-match gating, reset values |

All six use `verif/checks.vh` (+ `vt_driver.vh` for hu_compute). The three BRAM tbs
and shared_pool run at `timescale 10ps/1ps` (matching the RTL); the two Hadamard tbs
at `1ns/1ps`. No real bugs found in the BRAM primitives, shared_pool, or
hu_config_regs — those four are clean. hu_compute surfaced F4 + F5 above.

## New tests added (2026-06-07 coverage uplift — snnAcc datapath utils)

| Test | Module | Checks | Note |
|------|--------|-------:|------|
| snnAcc/tbPacker (NEW)      | packer                | 8135  | first-ever unit test |
| snnAcc/tbSliceAlign        | slice_and_align       | 64008 | all 6 sizes × every index |
| snnAcc/tbDatalineCache     | dataline_cache_with_xy| 5895  | hit/miss, invalidation, mem_wait |
| snnAcc/tbSynCurrUpdate     | syn_curr_update       | 1408  | full+sparse accumulate, sign bounds |
| snnAcc/tbActIndexGen       | act_index_generator   | 2250  | random grids + back-pressure |
| snnAcc/tbWeightGen         | weight_generator      | 88    | full/sparse golden + conv smoke |

## Design observations (non-bugs, worth an RTL comment) — snnAcc datapath

These are latent constraints, not live bugs (current callers respect them), but
worth documenting in the RTL:

- **D1 `packer` has no write-while-full guard.** `output_buffer` OR-accumulates on
  `pak_write_i` even when `buffer_full` and the writeback is stalled on `pot_wait`.
  Safe only because `neuron_processing` gates `result_taken` on `~packer_full`. A
  defensive internal guard (or a comment) would harden it.
  **RESOLVED 2026-06-21 (see F8):** this was not merely latent — it was the F8 bug.
  `packer.v` (all 6 copies) now guards the accumulate with
  `pak_write_i && !buffer_full`, and the driver now strobes on `result_taken`.
- **D2 `weight_generator.finished_pass_o` is a one-delta combinational pulse** —
  must be sampled AT the posedge, not at `#1` after (sampling late intermittently
  misses the conv pulse). Same hazard class as the `syn_curr_update`/conv
  `finished_pass`. Recommend documenting as a general rule for these `finished_*`
  pulses.
- **D3 `dataline_cache_with_xy` can alias across a slice-size change** — the single
  entry compares word indices shifted by the *current* `slice_sz_i`, so a stale
  entry can falsely hit if two requests share a base but differ in slice size.
  Harmless (slice size is fixed per task), but a latent constraint.

---

## F7 — hadamard_unit output addressing is broken for multi-word and 32-bit streams (CONFIRMED)

**Where:** `Hadamard/hadamard_unit.v` (packer element-index wiring) +
`Hadamard/dataline_cache_with_xy.v` (`slice_idx` case).

**What:** `hadamard_unit.v` feeds the output `packer` the CACHE's per-word slice index
as the element index:
```verilog
assign elem_index = {{(PIN_BITS-5){1'b0}}, a_idx};   // a_idx = slice index WITHIN a 32-bit word
```
The packer derives the output word address from this index (`offset = idx >> (5-out_sz)`).
Two consequences:
- **(a) Multi-word output overwrites word 0.** `a_idx` counts 0..(elems_per_word-1) and
  RESETS at each new input word, so every output word computes the same low offset —
  a stream spanning >1 output word writes all words to base+0; only the last survives.
- **(b) 32-bit elements emit an X address.** `dataline_cache_with_xy.v`'s `slice_idx`
  case (lines ~204-217) has entries for 1/2/4/8/16-bit but **no 32-bit (`'b101`) case**,
  so `slice_idx` defaults to `'hx`; that X flows through `a_idx` into the packer address.

**Why it matters:** The compute datapath is correct (F4/F5 fixed; `tb_hu_compute` D8 =
1993 verifies 32-bit multiply at unit level). But the ACCELERATOR only produces correct
output for a single output word of ≤16-bit elements. snnAcc avoids both by feeding the
packer its GLOBAL `neuron_counter_r`, not the cache slice index.

**Fix:** drive the packer's `pak_index_i` from a global stream-element counter (0..
stream_len-1), like snnAcc — not from `a_idx`. Also add the `'b101` case to
`dataline_cache_with_xy`'s `slice_idx` (=> 0 for 32-bit).

**Evidence:** `Hadamard/tb_hadamard_unit.v` (2026-06-08) verifies real multiplies
(R=Z*(A-B)+B+mode*R_prev: directed 17/±clamp/mode1/bp/16-bit + 60 random), 1511 checks,
0 failures — but ONLY because every task is sized to one output word and elem_sz=5 is
excluded from scoring (the test header documents how to lift those caps once F7 is fixed).

**Status:** FIXED 2026-06-15 (branch `hadamard-f7-multiword` off `verif-coverage-uplift`).

**Fix as implemented:** rather than a free-running counter in `hadamard_unit` (a naive
count-on-`take` desyncs by one — `take` is gated by compute-ready while the stream's `index`
advances on cache-ready), the fix reuses the stream generator's EXISTING global position:
- `stream_generator.v`: expose its `index` register as a new output `data_global_idx_o`
  (combinationally aligned with `data_o`/`data_idx_o`, exactly like the slice index).
- `hadamard_unit.v`: declare `a_gidx`, connect it to `u_sg_a.data_global_idx_o`, and drive
  the packer via `assign elem_index = a_gidx` (was `{..,a_idx}`). No new register; the
  packer (`offset = index >> (5-out_sz)`) is unchanged.
- `dataline_cache_with_xy.v`: add the `'b101` (32-bit) `slice_idx` case (→ 0).
All three files are Hadamard-local copies (no cross-accelerator impact). Unconditional fix
(no config gate): for a single output word the global index equals the old `a_idx`, so all
prior single-word coverage is bit-identical.

**Verification:** `Hadamard/tb_hadamard_unit.v` caps lifted; added DW1 (16-bit/3-word),
DW2 (32-bit/3-word), DW3 (8-bit/2-word) directed cases (assert values in NON-ZERO output
words) and a multi-word + 32-bit random loop. **2164 checks, 0 failures** (was 1511 single-
word). Negative control: reverting to the old `a_idx` wiring makes DW1/DW2/DW3 and the
multi-word random elements FAIL (words 1+ read 0; only the last word survives) — confirming
the cases genuinely exercise multi-word addressing. `tb_hu_compute` 20056/0 and
`tb_hu_config_regs` unchanged; `run_regression.sh Hadamard` = 3 passed / 0 failed.

---

## F8 — neuron_processing packers strobe on held result_valid, corrupting writeback under back-pressure (CONFIRMED → FIXED)

**Where:** all five `neuron_processing.v` variants (`snnAcc`, `ipSnnAcc`, `annAcc`,
`fmiSnnAcc`, `fmiSnnAccMC`) + the shared `packer.v`.

**Root cause (broader than the original sub-32-bit framing):** every writeback packer was
wired `.pak_write_i(result_valid)`. `result_valid` is a **level** held high for many
cycles whenever any packer is back-pressured (`result_taken = result_valid & ~packer_full
& ~*_wb_full`). `packer.v` OR-accumulates `output_buffer` on **every** cycle `pak_write_i`
is high, so under sustained writeback memory back-pressure it:
- **(a) sub-32-bit:** OR-accumulates the *next* neuron's field into a full, not-yet-flushed
  word (the field-level corruption originally reported); and
- **(b) the 1-bit spike packer / 32-bit packers:** after a word flushes while
  `result_valid` is still held, re-accumulates the current neuron and **re-flushes**,
  overwriting the correct word — this hits **even in all-32-bit configs** (e.g. FMI),
  where the spike packer is the corruption site, so F8 is *not* sub-32-bit-only.

Every other consumer in the system (caches' `slice_data_taken_i`, the neuron counter)
advances on the one-cycle `result_taken`/`neuron_taken` accept — only the packers latched
the held level.

**Why untested before:** every neuron_processing unit/integration test (all variants)
hard-wired the **write-side** `*_mem_wait` to `1'b0`, so a held writeback was structurally
unreachable. (Read-side back-pressure was exercised; write-side never.)

**Evidence:** new `fmiSnnAcc/tb_packer_cadence.v` (+ `tbPackerCadence.bsh`, cloned to
`fmiSnnAccMC`) drives the REAL `update_state_for_neuron` + REAL `packer` ×4 wired exactly
as `neuron_processing`, with wait-honouring writeback SRAMs and injectable write
back-pressure across the full slice-size matrix. Over an identical matrix
`pak_write_i = result_valid` → **70 failures**; `pak_write_i = result_taken` → **0
failures** (783 checks incl. an "exactly one accumulate per accepted neuron" assertion).

**Fix (2026-06-21):**
- Driver: drive every packer's `pak_write_i` from the existing one-cycle `result_taken`
  instead of `result_valid` (the ada packer keeps `& has_ada_i`). One line per packer in
  all five `neuron_processing.v`. No new wire/constant/port; no combinational loop
  (`result_valid`/`*_full` are registered, `pak_full_o` is registered `buffer_full`).
  Behaviour-identical when not back-pressured, so all existing 32-bit suites stay green.
- Defensive: `packer.v` (all 6 copies incl. Hadamard) now guards the accumulate with
  `else if (pak_write_i && !buffer_full)` so a future level-strobe reuse cannot
  re-introduce the OR-while-full case. **This supersedes design observation D1** (the
  write-while-full guard is now in the primitive); the held-flush re-accumulate is closed
  by the `result_taken` driver fix (the guard alone does not close it).

**Verification:** `tb_packer_cadence` 783/0 (fmiSnnAcc + fmiSnnAccMC). All existing suites
green post-fix across every variant + Hadamard (`tbNeuronProc`, acc-level, `tbPacker`,
`tbHuCompute`/`tbHadamard`). The acc-level `tb_neuron_processing` (fmiSnnAcc/MC) additionally
gained write-side back-pressure as **integration robustness coverage** ("still correct
under a stalled writeback"); note a 32-bit acc-level test is **not** a reliable F8
discriminator (the 32-bit re-OR is idempotent with a frozen counter, and the narrow
spike-flush race is masked by the shared read/write `mem_wait` stalling the producer) —
`tb_packer_cadence` is the canonical, reliable F8 regression.

---

## F9 — snnAcc full mode mis-indexes weight rows for GAPPED (sparse) activations (CONFIRMED → FIXED)

**Where:** `snnAcc/spike_processing.v` act-gating / `act_index_generator.v` →
`weight_generator.v` `weight_row_base_addr = act_data_idx_i * rows_per_neuron_i +
weight_base`. Surfaced 2026-06-30 by the new `snnAcc/tb_acc_snn_stress.v` full-chip
testbench (also applies to the `ipSnnAcc`/`annAcc`/`fmi*` variants, which share the
spike-processing front-end — not yet re-checked there).

**What:** When a non-spiking input sits in the MIDDLE of the input grid, the
weight-row index `act_data_idx` presented to `weight_generator` does **not** skip the
gap. Each spiking input is given a row equal to its **spike rank**, not its grid index.
So a full-connectivity layer driven by a sparse spike vector (the normal SNN case)
accumulates the **wrong weight rows**.

**Evidence (directed `gap_probe` in `tb_acc_snn_stress.v`):** 2×2 input grid, input 2
SILENT, inputs {0,1,3} spiking, distinct known weights `W[i][j]=10*(i+1)+j`, skip mode
(syn = raw accumulation, no decay):
```
spikes at grid {0,1,3}  ->  architecturally-correct rows {0,1,3}
                            syn = 70 73 76 79   (W[0]+W[1]+W[3])
DUT reads compacted rows {0,1,2}
                            syn = 60 63 66 69   (W[0]+W[1]+W[2])
```
A `weight_mem_rd_o` trace (compile `tb_acc_snn_stress.v` with `-define GAP_PROBE_DBG`)
shows `act_data_idx = 0,1,2` / `weight_row_base = 8,9,10` for the three spikes —
i.e. the silent input 2 did not advance the row index, and the grid-3 spike inherited
row 2.

**Why it's almost certainly a bug, not intent:** weights are STATIC and indexed by
input-neuron (grid) position; the firing set is a RUNTIME property. A weight row cannot
be addressed by dynamic spike rank. The architecture formula
`weight_row_base = act_data_idx * rows_per_neuron + base` is correct **iff**
`act_data_idx` is the grid index. (Likely also affects conv-mode projection, which uses
`act_data_x/y`, and sparse-mode tuple rows, which use `act_data_idx` — both untested
with gaps.)

**Why untested before:** `tb_spike_processing` only drives **all-spike or no-spike**
(its uniform-weight golden is mapping-independent and never gaps the stream with distinct
weights); `tb_acc_snn_processor` only gaps the **last** input (Test 4, a termination
regression). A non-spike in the middle with distinct weights was never exercised end to
end until `tb_acc_snn_stress`.

**Impact:** Correctness of every full/sparse layer fed a genuinely sparse spike vector
(most SNN layers). Not a flow-control/alignment bug — it reproduces identically with and
without stalls (the stall-invariance oracle still passes, since the mis-indexing is
deterministic).

**Root cause (confirmed):** `weight_generator.v` `weight_index_valid_full` was missing
the `& act_data_valid_i` gate that the sparse/conv arms already carried. Full mode
free-ran the weight cache on non-spiking inputs; with a per-input-varying weight base
(1-weight-per-word FC) the slice index advanced past an in-flight fetch and the cache
served the PREVIOUS input's weight — an off-by-one-low that flips phase at each act-word
boundary, presenting as the spike-rank-not-grid-index compaction this test observed.

**Status:** FIXED 2026-06-30 (parallel session). `0e6521d` fixed `fmiSnnAccMC` (verified
by its T13 128/128); `8e672ac` propagated the `& act_data_valid_i` gate to the three
byte-identical `weight_generator.v` copies (`snnAcc`/`ipSnnAcc`/`fmiSnnAcc`); `annAcc`
already carried the gate — so all five variants are fixed. **Re-verified here** by
`snnAcc/tb_acc_snn_stress.v`: `rand_common` flipped back to random GAPPED spike patterns
(now a scored hard check across full/sparse/conv × all weight sizes × skip/np_mode ×
stalls) and the directed `gap_probe` (mid-grid non-spike) converted from a soft NOTE to a
scored `check_eq` — spikes at grid {0,1,3} now read rows {0,1,3} (syn 70,73,76,79).
**1414 checks, 0 failures.** The fix commit itself cites this testbench
(`tbAccSNNStress 1410/1410`) as part of its verification.

**Per-variant end-to-end confirmation (2026-06-30):** focused F9 stress TBs ported
to all four other variants (full-chip top, `sp_skip_neuron=1`, gapped activations +
all weight sizes + full/sparse(/conv) + per-port stalls, connectivity golden +
stall-invariance + a directed mid-grid-gap `gap_probe`). All green:
`ipSnnAcc/tb_acc_ipsnn_stress` 340/0 (8-bit MAC), `annAcc/tb_acc_ann_stress` 340/0
(8-bit MAC), `fmiSnnAcc/tb_acc_fmiSnn_stress` 280/0 (1-bit spike, 1-D),
`fmiSnnAccMC/tb_acc_fmiSnnMC_stress` 220/0 (1-chan legacy spike). So the gate fix
is confirmed end-to-end in every variant, not just snnAcc.

---

## Notes (annAcc neuron_processing test, not bugs)

- **N1 LUT activation uses the full LUT slice width, not 8 bits.** With `LUT_SLICE_BITS=32`,
  `ann_update_state_for_neuron` zero-extends the entire 32-bit LUT word as the activation,
  not just the low 8 bits. A true 8-bit LUT must set `LUT_SLICE_BITS=8` (or guarantee
  entries ≤ 8 bits), else high bits leak into the activation.
- **N2 `acc_busy_o` overlaps the finish pulse by one cycle.** `acc_finished_o`/
  `neuron_proc_finished_o` pulse while `acc_busy_o` (running_r) is still high; busy clears
  the next edge. Relevant to scheduler integration (finish and busy overlap for one cycle).
