# CLAUDE.md — Hadamard

Hadamard transform accelerator module within the FlexMan system. Intended to be dispatched by the `scheduler` as one of the hardware accelerator targets.

## Modules

| File | Module | Role |
|---|---|---|
| `hadamard_unit.v` | `hadamard_unit` | Top-level accelerator |
| `hu_compute.v` | `hu_compute` | Compute datapath |
| `hu_config_regs.v` | `hu_config_regs` | AXI config register bank |
| `stream_generator.v` | `stream_generator` | Input data streaming |
| `dataline_cache_with_xy.v` | `dataline_cache_with_xy` | Memory cache (shared utility, also in snnAcc) |
| `packer.v` | `packer` | Output packing (shared utility, also in snnAcc) |
| `slice_and_align.v` | `slice_and_align` | Sub-word extraction (shared utility, also in snnAcc) |

## Notes

- `constants.v` duplicates most of `snnAcc/constants.v` — pending merge into `shared/constants.v`.
- `dataline_cache_with_xy.v`, `packer.v`, `slice_and_align.v` are copies of files in `snnAcc/` — these should move to `shared/` when the shared directory is established.
- Architecture details and simulation commands TBD.
