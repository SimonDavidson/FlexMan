# CLAUDE.md — FlexMan

FlexMan is a flexible hardware accelerator management system for ANN/SNN neural networks. The `scheduler` dispatches layer tasks to hardware accelerators (`snnAcc`, `Hadamard`, and future modules), managing buffer dependencies so tasks only run when inputs are ready and outputs are free.

## Directory Layout

```
FlexMan/
  scheduler/      ← task scheduler: instruction fetch/decode, buffer state, dispatch
  snnAcc/         ← SNN accelerator: spike processing + neuron processing pipeline
  Hadamard/       ← Hadamard transform accelerator
  new_module/     ← TBD
  top_module/     ← TBD: top-level integration wrapper
  shared/         ← shared constants, interfaces (in progress — see below)
```

Each sub-directory has its own `CLAUDE.md` with module-level detail. This file covers cross-cutting concerns only.

## Inter-Module Relationships

- `scheduler` dispatches `TASK` instructions to accelerators via a system bus interface.
- `snnAcc` and `Hadamard` are accelerators managed by `scheduler`; they signal completion back via the system bus.
- `top_module` (TBD) will instantiate `scheduler` plus all accelerators and wire the system bus.
- Accelerator configuration is pre-loaded by the scheduler's config register bank before each `TASK` dispatch.

## Shared Constants (action required)

`snnAcc/constants.v` and `scheduler/constants.v` are nearly identical with minor additions in each:
- Scheduler adds: `` `PROG_BITS ``, `` `PROG_DATA_BITS ``
- snnAcc adds: `` `TGT_ACC_SZ ``, `` `SCH_ENTRY_SZ ``

These should be merged into `shared/constants.v` and all `` `include `` directives updated. **Do not do this as part of a feature branch** — it touches every module and should be a dedicated integration step.

## Signal Naming Convention

Consistent across all modules:
- `_i` suffix — input port
- `_o` suffix — output port
- `_r` suffix — registered config value
- `mem_wait_i` / `mem_wait_o` — back-pressure on memory interfaces

## Simulation

Each sub-module has its own simulation script (see its `CLAUDE.md`). Integration simulation via `top_module/` is TBD. All modules use **Xcelium** (`xrun -sv`).

## Simulator Artefacts (gitignored)

`xcelium.d/`, `xrun.log`, `xrun.history`, `xrun.key`, `waves.shm/`, `*.vcd`
