# FlexMan verification findings

Confirmed issues surfaced by the aggressive test suite (`verif/` library + per-module
testbenches). Each entry: what, where, evidence, impact, status.

Authors: Simon Davidson & Claude · Created 2026-06-07 · Last modified 2026-06-07

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

## F3 — flexman_fpga_wrap.v out of sync with flexman.v (won't elaborate)

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

**Status:** OPEN — reported. Out of scope for the test-coverage uplift (it is an RTL
sync issue), flagged for Simon.

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

## F6 — full-mode spike_processing accumulates with ALL-ZERO activations (spike gating ineffective)

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

**Status:** OPEN — reported as a non-fatal NOTE in the test (kept green pending
decision). Surfaced via partial spike patterns having no effect, then confirmed
with the all-zero probe.

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
- **D2 `weight_generator.finished_pass_o` is a one-delta combinational pulse** —
  must be sampled AT the posedge, not at `#1` after (sampling late intermittently
  misses the conv pulse). Same hazard class as the `syn_curr_update`/conv
  `finished_pass`. Recommend documenting as a general rule for these `finished_*`
  pulses.
- **D3 `dataline_cache_with_xy` can alias across a slice-size change** — the single
  entry compares word indices shifted by the *current* `slice_sz_i`, so a stale
  entry can falsely hit if two requests share a base but differ in slice size.
  Harmless (slice size is fixed per task), but a latent constraint.
