# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Verilog RTL design for an **SNN (Spiking Neural Network) Hardware Accelerator**. Implements a two-stage pipeline: `spike_processing` accumulates synaptic currents from input spikes × weights, then `neuron_processing` integrates those currents to update neuron state and emit output spikes.

The sibling directories (`5_test_write_config_regs_...`, `6_about_to_add_xy_...`, etc.) are snapshot variants at earlier development stages — the canonical design lives here in `snnAcc/`.

## Simulation

Simulator is **Xcelium** (Cadence). Run from this directory:

```bash
bash tbAccSNN.bsh
```

This invokes:
```
xrun -sv -gui -timescale 1ns/1ps -access wrc -top tb_acc_snn_processor \
  acc_snn_processor.v spike_processing.v act_index_generator.v \
  weight_generator.v syn_curr_update.v update_state_for_neuron.v \
  neuron_processing.v dataline_cache_with_xy.v slice_and_align.v \
  tb_acc_snn_processor.v
```

Waveforms: `waves.shm/` and `tb_acc_snn_processor.vcd`. Open with SimVision. Build artefacts: `xcelium.d/`, `xrun.log`, `xrun.history`, `xrun.key`.

## Architecture

### Pipeline

```
[Input spikes / weights]
        │
        ▼
 spike_processing      ─── reads: act_mem, weight_mem
        │              ─── writes: syn_curr_mem
        ▼  (sp_acc_finished triggers neuron_processing)
 neuron_processing     ─── reads: syn_curr_mem, bias_mem, thresh_mem, pot_mem
        │              ─── writes: pot_mem, spike_mem (output)
        ▼
  [Output spikes / potentials]
```

The synaptic current memory (`syn_curr_mem`) is **shared** between both stages. Arbitration is in `acc_snn_processor` with **fixed priority to `neuron_processing`**. Grants are cycle-by-cycle (no burst lock). The losing module sees `mem_wait_i` asserted until the bus is free.

### Modules

| File | Module | Role |
|---|---|---|
| `acc_snn_processor.v` | `acc_snn_processor` | Top-level: instantiates both stages, arbitrates syn_curr_mem, hosts AXI config registers |
| `spike_processing.v` | `spike_processing` | Stage 1: drives act_index_gen, act_cache, weight_gen, syn_curr_update |
| `neuron_processing.v` | `neuron_processing` | Stage 2: cycles over neurons, reads currents/bias/thresh/pot, writes updated state |
| `act_index_generator.v` | `act_index_generator` | Generates activation memory addresses; supports full/sparse/conv via `weight_mode_i` |
| `weight_generator.v` | `weight_generator` | Computes weight memory addresses from input spike position and kernel parameters |
| `dataline_cache_with_xy.v` | `dataline_cache_with_xy` | Single-entry cache for any memory interface; extracts sub-word slices via `slice_and_align` |
| `syn_curr_update.v` | `syn_curr_update` | Read-modify-write: fetches 32-bit syn_curr word, adds sign-extended weight, writes back |
| `update_state_for_neuron.v` | `update_state_for_neuron` | 2-cycle pipeline: cycle 1 = integrate + decay syn_curr; cycle 2 = threshold + decay potential |
| `slice_and_align.v` | `slice_and_align` | Extracts a variable-width sub-field from a 32-bit word; generates `last_slice_o` |
| `constants.v` | — | Global `` `define `` constants for bus widths (included by all modules) |
| `tb_acc_snn_processor.v` | `tb_acc_snn_processor` | Testbench: 256×32-bit SRAM models, random data, 1-cycle read latency, 1000-cycle timeout |

### AXI Config Register Map (`acc_snn_processor`)

Address match: `sys_addr_i[31:16] == TGT_CONFIG_BASE_ADDR[31:16]`. Register select on `sys_addr_i[7:0]`:

| Offset | Register |
|--------|----------|
| `0x00` | `sp_act_base_addr_r` |
| `0x04` | `sp_weight_base_addr_r` |
| `0x08` | `syn_curr_base_addr_r` (shared) |
| `0x0C` | `sp_weight_sz_r` |
| `0x14` | `sp_total_timesteps_r` |
| `0x20` | `np_last_neuron_idx_r` |
| `0x28` | `np_bias_curr_base_addr_r` |
| `0x2C` | `np_thresh_base_addr_r` |
| `0x30` | `np_pot_base_addr_r` |
| `0x34` | `np_syn_curr_sz_r` |
| `0x38` | `np_bias_curr_sz_r` |
| `0x3C` | `np_pot_sz_r` |
| `0x40` | `bin_point_syn_curr_r` |
| `0x64` | `np_spike_base_addr_r` |
| `0x44–0x5C` | Grid/kernel size registers (in/out X/Y, stride, packing) |
| `0x70` | `sp_weight_mode_r` (00=full, 01=sparse, 10=conv) |
| `0x74–0x88` | Conv kernel: `sp_x_kernel_len_r`, `sp_y_kernel_len_r`, `sp_x_kernel_step_r`, `sp_y_kernel_step_r`, `sp_x_kernel_offset_r`, `sp_y_kernel_offset_r` |
| `0x8C–0x94` | Sparse: `sp_index_sz_r`, `sp_tuple_sz_r`, `sp_sparse_count_r` |

`sys_ack_o` is asserted combinationally on the same cycle as `sys_req_i` when the address matches.

### Key Design Patterns

**Valid/taken handshake** — the universal flow-control protocol. Producer asserts `*_valid_o`; consumer asserts `*_taken_i` to consume. Data is held stable until taken. Used across all inter-module interfaces (act_index, weight_index, weight_value, slice_data, neuron result).

**`dataline_cache_with_xy` slice encoding** — `slice_sz_i` selects element width packed in a 32-bit word:

| `slice_sz_i` | Element width |
|---|---|
| `3'b000` | 1-bit |
| `3'b001` | 2-bit |
| `3'b010` | 4-bit |
| `3'b011` | 8-bit |
| `3'b100` | 16-bit |
| `3'b101` | 32-bit |

The cache is one entry deep. A memory fetch is issued when `sys_req_i` is asserted and the address misses. Data arrives the next cycle (1-cycle SRAM latency); `sys_wait_o` back-pressures the upstream index generator during a miss.

**`update_state_for_neuron` 2-cycle pipeline** — Cycle 1: `mul_a = syn_curr_i`, decayed syn_curr is registered. Cycle 2: `mul_a = potential_r`, decayed potential is the final output. `result_valid_o` is only high on cycle 2. If a spike fires, `potential_o` is zeroed (reset-to-zero model).

**Memory interface convention** — synchronous SRAM: address/write-data/read-data/write-enable/chip-select. Reads return data one cycle after the address (matches the testbench SRAM model).

### Connectivity Modes

| `weight_mode_i` | Mode | Status |
|---|---|---|
| `2'b00` | Full (dense) connectivity | Implemented |
| `2'b01` | Sparse | Implemented |
| `2'b10` | Convolutional | Implemented |

`weight_mode` is now a config register (`sp_weight_mode_r`, offset `0x70`) driven from the testbench. In conv mode each input neuron projects to a 2-D kernel of output neurons:

```
out_x = act_x * x_kernel_step - x_kernel_offset + kx
out_y = act_y * y_kernel_step - y_kernel_offset + ky   (for kx,ky in 0..kernel_len-1)
```

Out-of-bounds projections are silently skipped (`weight_index_valid_o` suppressed for that beat) but the kernel-position counter still advances so the kernel-weight slice pointer stays in lockstep. Kernel weights live as one shared block at `weight_base_addr_i` (no `act_data_idx_i` term in conv mode), packed row-major.

`syn_curr_update` addresses syn_curr memory via `base + y * out_x_len + x`, so conv contributions accumulate into the projected output neuron. A `finished_pass_weight_i` input from `weight_generator` drops `syn_curr_update_running` cleanly even when the very last kernel slot is OOB.

**Sparse mode (`2'b01`)** — each weight memory entry is a `(output_index, weight_value)` tuple. Both `index_sz` and `weight_sz` are independently chosen powers of two (1/2/4/8/16 bits each, encoded as 3-bit log2 values). The tuple packs them back-to-back with trailing zeros to reach the next power-of-two boundary:

```
[ index_sz bits | weight_sz bits | trailing zeros ]
 ←————————————— tuple_sz bits ——————————————————→
```

`tuple_sz` must be set to the smallest power of two ≥ `index_sz + weight_sz`. Example: 8-bit index + 2-bit weight → 10 bits needed → `tuple_sz = 16`. The three sparse config registers (offsets `0x8C`/`0x90`/`0x94`) are `sp_index_sz_r`, `sp_tuple_sz_r`, `sp_sparse_count_r` (tuples per input neuron), all set independently.

The cache slice size is muxed in `weight_generator` (`tuple_sz_i` in sparse, `weight_sz_i` otherwise). `spike_processing` contains the tuple unpacker: a flat case on `index_sz_i` determines the weight start bit (immediately after the index field), and a nested case on `weight_sz_i` sign-extends the weight into `WEIGHT_BITS`. The parsed output index drives `sparse_index_i` on `syn_curr_update`, which uses it directly for addressing (overriding the `y * out_x_len + x` formula). The cache (`dataline_cache_with_xy`) tracks `base_addr_i` so it invalidates when the input neuron changes — needed for full and sparse modes (conv shares one kernel block).

Sparse mode requires `WEIGHT_SLICE_SZ ≥ tuple_sz_enc` (the weight-bus port must hold a full tuple). When the bus is wider than the actual weight, `spike_processing` right-justifies and sign-extends both the sparse-extracted weight and the non-sparse weight before passing them to `syn_curr_update`.

### Known Incomplete Areas

Several modules contain `// TODO` / `// XXXXX` stubs that are under active development:

- **`neuron_processing.v`**: `get_next_bias_curr_value` and `get_next_pot_value` are `1'b0`; `syn_curr_mem_wr_o` and `pot_mem_wr_o` write-backs are `1'b0`. Spike output is implemented via `spike_packer0` (`packer.v`): accumulates 1-bit spike values into 32-bit words and writes through `spike_mem_*`.
- **`update_state_for_neuron.v`**: `mul_result` uses `*` (inferred multiplier); signed/unsigned mixing on `mul_b` is intentional (Q0.32 decay factor).
