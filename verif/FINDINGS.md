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

**Status:** OPEN — reported, awaiting decision. Captured as a directed "spec-intent"
NOTE in the self-test; will become a hard directed FAIL in the Phase-2A
`tb_update_state_for_neuron` once the fix is agreed.

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

**Status:** OPEN — to be diagnosed in Phase 4 by rewriting the test with the
pipeline-depth-agnostic `VT_CONSUME` driver + `np_ref_lif` golden. If the RTL is
correct it will pass; if not, it pins a real ipSnnAcc bug. The same X propagates
into `ipSnnAcc/tb_neuron_processing` (baseline FAIL: `pot_sram[128] got X`).

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
