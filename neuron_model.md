# FlexMan neuron model — equations as implemented

Canonical reference for what the FlexMan accelerators actually compute, derived
from the RTL (not the training framework). Verified 2026-06-02/03 against
`snnAcc/update_state_for_neuron.v`, `snnAcc/syn_curr_update.v`,
`annAcc/ann_update_state_for_neuron.v` and `annAcc/neuron_processing.v`.

> ⚠️ One deliberate divergence from Benjamin's reference is called out in
> [§ Bias placement](#bias-placement). Everything else matches eq (a)
> (decay-then-add).

## Symbols

| Symbol | Meaning | Config register |
|--------|---------|-----------------|
| `i`  | synaptic current (accumulator) | — |
| `v`  | carried membrane potential | — |
| `z`  | spike output (0/1) | — |
| `b`  | bias | `np_bias_curr_*` |
| `α`  | syn-curr decay (Q0.32) | `np_syn_curr_decay_mult` |
| `β`  | membrane decay (Q0.32) | `np_pot_decay_mult` |
| `θ`  | threshold integer = `round(1.0 · 2^bin_point)` | `np_thresh_*` (loaded as a value) |
| `bin_point` | fractional bits of the accumulator | `bin_point_syn_curr` |

Decays are applied as `x·mult >> DECAY_BITS` (Q0.32 multiply, keep the high
bits). All values are right-justified two's-complement; the binary point is
measured up from the LSB.

## LIF neuron — encoder (ipSnnAcc) and recurrent (snnAcc)

Per timestep `t`, per neuron:

```
1.  iₜ = α·iₜ₋₁ + (W·x)ₜ
        encoder    : (W·x)ₜ = W_enc · a_fe            a_fe = feature-extraction output (multi-bit)
        recurrent  : (W·x)ₜ = W_in · z_enc + W_rec · zₜ₋₁   (both inputs are SPIKES, step 1.0)
2.  pₜ = vₜ₋₁ + b + iₜ                 ← bias added at the MEMBRANE (see § Bias placement)
3.  zₜ = 1 if pₜ ≥ θ else 0
4.  vₜ = { 0          if zₜ and reset-to-zero      (np_mode[0]=0 — the siren net)
         { β·pₜ − θ   if zₜ and subtract-on-fire   (np_mode[0]=1)
         { β·pₜ       if not zₜ
```

The recurrent feedback uses the previous **spike** `zₜ₋₁`, not the potential
(`recurrent_linear`'s activation quantizer step is 1.0 → binary). The recurrent
layer runs as **two passes**: pass 1 (`sp_skip_neuron=1`) accumulates `W_in·z_enc`
into `i` with no neuron update; pass 2 (`sp_skip_neuron=0`) adds `W_rec·zₜ₋₁` then
runs steps 2–4 once. So α/bias/threshold are applied exactly once per timestep.

## Implementation notes (why the RTL looks different)

- **Decay-then-add via pre-decay storage.** syn_curr memory holds the *already
  decayed* current `α·iₜ`; next timestep's MAC adds the fresh `W·x` on top, so the
  effective recurrence is `iₜ = α·iₜ₋₁ + (W·x)ₜ` (eq a). The membrane uses the same
  trick: the stored value is `β·pₜ`, giving `pₜ = β·pₜ₋₁ + b + iₜ` between spikes.
- **Threshold is a raw integer compare** on the *pre-decay* sum `pₜ`
  (`update_state_for_neuron.v:152`), with `β` applied when carrying forward
  (step 4). `1.0` is placed at the right scale by loading `θ = 2^bin_point`
  (encoder 2048 = 2¹¹, recurrent 64 = 2⁶) — there is no dequantise shift in HW.
- **Saturation** on the membrane sum is to the internal 32-bit width
  (`update_state_for_neuron.v:87-93`), independent of the threshold.

## Readout (snnAcc, LI)

Same as the LIF neuron but `θ = 0x7FFFFFFF` (never fires) and the **output is the
potential `pₜ`** (read by the host), not a spike. Sub-threshold dynamics are
identical to the LIF layer.

## annAcc — feature_extraction (feedforward, ABS)

No recurrence or decay (`sp_total_timesteps=1`). The output is the requantised
activation:

```
iₜ   = Σ W_fe · xₜ                                       (xₜ = raw-audio window)
actₜ = saturate_N( round( |iₜ| / 2^(bin_point_syn_curr − np_out_bin_point) ) )
       RELU : max(0, iₜ) instead of |iₜ|
       LUT  : LUT[iₜ]
```

`N = np_pot_sz` (output element width). The requant (round → shift →
unsigned-saturate) lives in `annAcc/neuron_processing.v` §4b; `np_out_bin_point=0`
disables it (legacy low-bits behaviour). For the siren net `bin_point_syn_curr=16`,
`np_out_bin_point=4` ⇒ shift 12, output at step 2⁻⁴.

## <a name="bias-placement"></a>Bias placement — implemented vs reference ⚠️

| | Bias enters | ⇒ shaped by |
|---|---|---|
| **FlexMan (implemented)** | the **membrane** `pₜ = vₜ₋₁ + b + iₜ` | `β` only |
| **Benjamin's reference** | the **current** `iₜ = α·iₜ₋₁ + W·x + b` | `α` then `β` (×1/(1−α) larger) |

So our bias contribution is too small by `1/(1−α)` (×5.68 recurrent, ×1.56
encoder). Pending fix (after Benjamin confirms, meeting 2026-06-03): pre-scale the
stored bias by `1/(1−α)` per layer (DC-exact, small transient residual), or an RTL
change to inject bias on the current path (exact). feature_extraction is
bias-free, so unaffected. See memory `project-benjamin-clarifications`.
