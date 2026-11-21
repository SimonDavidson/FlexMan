<!--
Authors: Simon Davidson & Claude
Created: 2026-11-21
Last modified: 2026-11-21
-->
# annAcc output-stationary lanes (`NLANE`)

`annAcc` can optionally compute 2 or 4 outputs of a dense layer per cycle. This is controlled by the
`NLANE` and `MAC_ACT_BITS` parameters of `ann_processor` and is **off by default** (`NLANE = 1`).

## Why the original path is limited

Full-mode `spike_processing` is input-stationary: every MAC does a read-modify-write on the 32-bit
`syn_curr` accumulator through a single port, so 2 cycles/MAC is a hard floor however fast the
multiplier is. Replicating multipliers cannot help; the accumulator has to move out of memory.

## What it does

For each tile of `NLANE` consecutive outputs the engine (`ann_tile_sp`) reads one 32-bit weight
word and (at word boundaries) one activation word per input, multiplies the activation into
`NLANE` lane MACs (`ann_lane_mac`) and adds into register accumulators. Each tile result is then
added into `syn_curr` with a single read-modify-write per output. The accumulator bank is
double-buffered so the drain of tile *t* overlaps the compute of tile *t*+1.

The weight word is read straight from the weight port (one pipelined request per cycle), not through
`dataline_cache_with_xy`, which can deliver at most 0.5 word/cycle. The tile assumes read data
returns the cycle after a request is accepted, the same contract the cache already relies on.

Results are bit-identical to the original path (signed 32-bit wrap-around addition is associative).

## Enabling it

`ann_processor #(.NLANE(L), .MAC_ACT_BITS(B), ...)`. With `NLANE = 1` the `generate` in
`acc_snn_processor.v` wires `ann_spike_processing` straight to the ports and `ann_tile_path` is not
elaborated; the design is cycle-identical to before. With `NLANE > 1`, `ann_tile_path` chooses per
task at dispatch between the tile engine and the original `ann_spike_processing`.

`MAC_ACT_BITS` is the widest activation the lane MACs accept. At 8, each lane needs one DSP
instead of two. Wider activations fall back to the original path.

## Constraints

- **Dense (full) connectivity only.** Sparse and convolutional layers run on the original path.
- **8- and 16-bit weights only**, with `NLANE <= weights per word` (so 16-bit weights need `NLANE <= 2`).
  Other weight widths fall back to the original path.
- **Zero activations are not skipped** (the original path skips them). The original path wins only
  above roughly 88% zero activations at `NLANE = 4` (analytical estimate, not measured).
- **The neuron-update stage is unchanged** (about 3-4 cycles/neuron), so it limits small layers.
- The tile is not interruptible and its start/finish latency differs from the original by a few cycles per task.
- Fmax has not been measured in a full FPGA flow; the multiplier-to-accumulator path is the likely limiter.
- Throughput under memory back-pressure has not been measured (correctness under stalls is tested).

## Verification (Xcelium)

| Script | Covers |
|---|---|
| `tbAccAnnTile.bsh` | 10 directed + 60 random scenarios, bit-exact vs a golden, with and without random stalls; `NLANE` 1/2/4, `REG_RETURN` 0/2, `MAC_ACT_BITS` 8/32 |
| `tbAccAnnStress.bsh` | existing stress test (all weight sizes, sparse, conv, gapped activations, stalls) at `NLANE` 1/2/4 |

`tb_ann_tile_perf.v` is the performance variant (`-define NLANE=.. -define MACB=.. -define RR=..`).

## Measured results (prototype, ideal memories, 8-bit weights)

Spike stage alone: 3.99 MACs/cycle at `NLANE = 4` (0.251-0.271 cycles/MAC) against 0.44-0.49 for the
original path, i.e. 7.6-9.0x. Whole layer (including the unchanged neuron stage) 4.8-8.4x.
Out-of-context Vivado synthesis (xcku3p-3, spike stage, no BRAM): original 1069 LUT / 4 DSP;
`NLANE = 4` adds +1070 LUT / +8 DSP (`MAC_ACT_BITS = 32`) or +900 LUT / +4 DSP (`MAC_ACT_BITS = 8`).
These were measured on the scratch prototype and are being re-measured on the merged tree.
