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
