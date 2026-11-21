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

## Measured results (merged tree, 2026-11-21)

Cycles, `ann_processor` full chip, ideal 1-cycle memories, 8-bit weights and activations, ReLU
(`tb_ann_tile_perf.v`; `NLANE = 1` is the original design). Spike stage (SP) and whole layer:

| Layer | `REG_RETURN` | `NLANE=1` SP / total | `NLANE=4` SP / total | SP speed-up | Layer speed-up |
|---|---|---|---|---|---|
| 16 x 32 | 0 | 1156 / 1284 | 139 / 267 | 8.3x | 4.8x |
| 64 x 64 | 0 | 9232 / 9488 | 1035 / 1291 | 8.9x | 7.3x |
| 200 x 64 | 0 | 28850 / 29106 | 3211 / 3467 | 9.0x | 8.4x |
| 16 x 32 | 2 | 1058 / 1156 | 139 / 237 | 7.6x | 4.9x |
| 64 x 64 | 2 | 8322 / 8516 | 1035 / 1229 | 8.0x | 6.9x |

The spike stage reaches 3.99 MACs/cycle asymptotically at `NLANE = 4` (0.25 cycles/MAC) against
0.44-0.49 for the original path. The neuron stage is unchanged (4.0 cycles/neuron at
`REG_RETURN = 0`, about 3.0 at `REG_RETURN = 2`), so it limits small layers. `REG_RETURN = 2` is
the fairer baseline. 64 x 64 with 16-bit weights: 2055 SP cycles at `NLANE = 2`; `NLANE = 4`
falls back to the original path (10256), as designed.

Out-of-context Vivado synthesis of the whole `ann_processor` (xcku3p-3, no BRAM, 5 ns clock):

| Config | LUTs | Registers | DSPs |
|---|---|---|---|
| `NLANE=1` (original) | 2586 | 964 | 8 |
| `NLANE=2`, `MAC_ACT_BITS=8` | 3335 | 1293 | 10 |
| `NLANE=4`, `MAC_ACT_BITS=8` | 3475 | 1352 | 12 |
| `NLANE=4`, `MAC_ACT_BITS=32` | 3643 | 1481 | 16 |

Post-synthesis worst slack is -1.055 ns at 5 ns (about 165 MHz, before place and route) in every
configuration, including `NLANE = 1`. The worst path is in `neuron_processing`
(`neuron_counter_r` to the update multiplier), not in the tile, so the lanes do not set Fmax at
this stage. Place-and-route Fmax has not been measured. Throughput under memory back-pressure
has not been measured.
