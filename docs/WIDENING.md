<!--
Authors: Simon Davidson & Claude
Created: 2026-11-21
Last modified: 2026-11-21
-->
# snnAcc lane widening (`NUM_LANES`)

`snnAcc` can optionally process 2 or 4 neurons per cycle instead of 1. This is controlled by the
`NUM_LANES` parameter of `acc_snn_processor` and is **off by default** (`NUM_LANES = 1`).

## What it does

The scalar neuron update time-multiplexes one multiplier between the synaptic-current decay and the
potential decay, so it takes 3 cycles per neuron however fast memory is. Widening replicates that
update across L lanes in lockstep. Narrow (8/16-bit) state packs 4 or 2 elements into each 32-bit
word, so one fetched word feeds up to 4 or 2 lanes with no banking and no extra memory ports.

A one-word prefetch per read stream is essential: without it a fetch bubble per group caps the
speedup well below L.

## Enabling it

`acc_snn_processor #(.NUM_LANES(L), .PREFETCH(P), ...)` selects, with a `generate`, between the
original `spike_processing`/`neuron_processing` (`NUM_LANES == 1`) and the wide variants
(`NUM_LANES > 1`). The memory ports and the syn_curr arbiter are unchanged.
At `NUM_LANES = 1` the original modules are instantiated unchanged.

## Constraints (important)

- **Narrow state only.** The wide path needs 8- or 16-bit `syn_curr`/`potential`, with
  `NUM_LANES <= 32 / element_bits`. The scalar path (`NUM_LANES = 1`) is validated only with 32-bit
  state: driving it with narrow state processes only part of a layer. `NUM_LANES > 1` is therefore a
  different operating mode, not a drop-in speedup at the same precision.
- **Full connectivity only** in the wide spike path. Sparse and convolutional layers are not
  supported by `spike_processing_wide` (it flags this in simulation and drains without writing).
- **At least one spiking input.** With all-zero input activations the wide spike path does not
  finish its task (known limitation, not yet fixed).
- **16-bit at 4 lanes is not built** (two words per group would be needed on write-back).
- Whether the compile flow (`tools/flexman_backend`) can generate narrow-state SNN programs has not
  been checked; the config packer can encode the slice sizes, but this is untested end to end.

## Verification (Xcelium)

All pass; `./run_regression.sh snnAcc` gives 18/18 (the 12 existing tests plus the six below).

| Script | Covers |
|---|---|
| `tbPackerWide.bsh` | write-back packer: 1/8/16-bit, 1/2/4 lanes |
| `tbSynCurrAccumWide.bsh` | wide synaptic-current accumulator |
| `tbNpLineFetch.bsh` | plain and prefetching read front-ends |
| `tbNeuronProcWide.bsh` | wide neuron update vs a lane- and width-independent golden |
| `tbSpikeProcWide.bsh` | wide spike processing, L = 1, 2, PF = 0, 1 |
| `tbAccSnnWide.bsh` | end to end: 32-bit scalar baseline, and wide at 8/16-bit, 2/4 lanes |

## Measured results (June prototype)

These were measured on the original prototype, before it was merged onto the current tree, and have
**not yet been re-measured** on this merge.

Neuron-processing stage only, N = 64 neurons, with prefetch (cycles, start to finish):

| width | L=1 | L=2 | L=4 |
|---|---|---|---|
| 8-bit | 195 | 99 (1.97x) | 51 (3.82x) |
| 16-bit | 195 | 99 (1.97x) | n/a |

Cycle model, fitting the data to within one cycle: with prefetch, `cycles ~ 3 * ceil(N/L) + 3`;
without it, `3 * ceil(N/L) + 3 * ceil(N / (32/width)) + 1`.

Out-of-context synthesis on a Xilinx UltraScale+ part: the only resource that scales with L is DSP
(2 per lane); LUTs stay roughly flat and BRAM stays 0. Post-route Fmax was roughly flat across L.
