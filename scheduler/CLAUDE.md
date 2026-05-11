# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

**FlexMan** is a hardware scheduler for ANN/SNN neural network accelerators. One or more
hardware accelerators each execute a single layer of neurons; the scheduler reads a program of
instructions from external memory and dispatches layer tasks to the accelerators, managing
buffer dependencies so that each task only runs when its inputs are ready and its output buffer
is free.

## Simulation Commands

All simulations use **Xcelium (`xrun`)** with SystemVerilog (`-sv`) and a GUI (`-gui`). The three
entry-point scripts:

```bash
# Full scheduler integration test
bash mySch.bsh

# Scheduler table module only
bash myTable.bsh

# Buffer/accelerator state tracking module only
bash myBuffState.bsh
```

Each script invokes `xrun` directly with the relevant source files and a `restore.tcl` /
`restore_buff.tcl` script that restores the waveform view.

## Architecture

### Module Hierarchy

```
scheduler.v          ← top-level instruction fetch/decode FSM, AXI interface
  prog_cache.v       ← program memory cache (reduces latency for instruction fetch)
  sch_table.v        ← holds up to 16 pending tasks (spec); RTL currently implements 4
    sch_entry.v      ← one instance per slot; asserts ready_to_execute when all
                        resources (source buffers full + matching colour, target buffer
                        free, accelerator free) are satisfied
  sch_buffer_state.v ← tracks NUM_BUFFERS (16 in RTL, 32 per spec) buffers
                        + NUM_HW_ACCELERATORS (2) accelerators
    buffer_state_entry.v    ← per-buffer: busy / full / colour / consumer-count state
    acc_hw_buffer_tracker.v ← per-accelerator: which buffers in use, completion signalling
```

### Data Flow

1. `scheduler.v` fetches 32-bit (or 64-bit for FILL) instructions from `prog_cache.v`.
2. TASK entries are pushed into `sch_table.v` in program order.
3. The spec defines the table as 16 entries; **only the top 4 entries are eligible for dispatch** at any time.
4. Each `sch_entry.v` checks three conditions every cycle (see Dispatch Conditions below).
5. When an entry becomes ready, `sch_table.v` selects it and `scheduler.v` sends it to the target accelerator.
6. `sch_buffer_state.v` updates buffer and accelerator state on dispatch and completion,
   broadcasting changes back to all `sch_entry` instances.

### Dispatch Conditions (all three must hold)

1. The target accelerator is free (`acc_busy` flag clear).
2. All source buffers are marked **full** and their **colour matches** the colour field in the task.
3. The destination buffer is marked **free** (not busy, not full).

### Buffer State (four fields per buffer)

| Field | Meaning |
|-------|---------|
| **busy** | Set when a task targets this buffer but hasn't completed yet. Stalls any new task that would also write here (out-of-order hazard). |
| **full** | Set when the writing task has completed. Enables waiting consumer tasks to proceed. |
| **colour** | 1-bit tag. A consumer task only claims a full buffer if its colour matches. Prevents a delayed task from consuming data meant for another task. |
| **counter** | Loaded from the `#Targets` field of the producing TASK. Decremented each time a consumer task is dispatched. When it reaches 0, `full` is cleared, colour is flipped, and the buffer is free again. |

### Key Parameters

| Parameter | RTL value | Spec target |
|-----------|-----------|-------------|
| `NUM_HW_ACCELERATORS` | 2 | 2 |
| `NUM_BUFFERS` | 16 | 32 |
| `NUM_SCH_ENTRIES` | 4 | 16 (top 4 eligible for dispatch) |
| `PROG_ADDR_BITS` | 10 | — |
| `PROG_DATA_BITS` | 32 | 32 |
| `SCH_ENTRY_SZ` | 32 | 32 |
| `CFG_ID_SZ` | 5 | — |
| `TGT_COUNT_SZ` | 3 | — |

## Instruction Set Architecture

Opcodes are **4-bit `[3:0]`**. Bit 2 of the opcode set indicates a two-word (64-bit) instruction
(currently only FILL). All instructions are 32-bit aligned.

### TASK (opcode `0b0000`)

Dispatch a layer computation to a hardware accelerator.

| Bits  | Field |
|-------|-------|
| [2:0] | Opcode |
| [7:3] | Target buffer |
| [8]   | Colour |
| [11:9] | #Targets (consumer count for target buffer) |
| [16:12] | Config ID |
| [19:17] | Accelerator ID |
| [24:20] | Source buffer 1 |
| [29:25] | Source buffer 2 |

### JUMP (opcode `0b0001`)

Unconditional absolute jump. Address is in units of 32-bit words (32-bit aligned).

| Bits  | Field |
|-------|-------|
| [2:0] | Opcode |
| [31:3] | Jump address |

### STOP (opcode `0b0010`)

Stop fetching. Wait for all outstanding tasks to complete, then signal *finished* to the host
CPU via the system bus. A new program resets all buffer and accelerator state.

| Bits  | Field |
|-------|-------|
| [3:0] | Opcode (only meaningful bits) |

### CHECK (opcode `0b0011`)

Conditional branch for dynamic neural networks (DyNN). The success/fail indicator comes from
the output of the unit nominated in bits `[10:4]`.

| Bits   | Field |
|--------|-------|
| [2:0]  | Opcode |
| [3]    | Mode: `1` = finish on success, `0` = skip on success |
| [10:4] | Check unit ID |
| [11]   | Colour |
| [31:12] | Skip address (used in skip-on-success mode) |

- **Finish on success**: terminates execution; all tasks of the matching colour are cancelled;
  waits for a CONTINUE signal from an external agent before resuming.
- **Skip on success**: jumps to the skip address, bypassing subsequent layers.
- **On fail (either mode)**: continues with the next instruction in sequence.

### FILL (opcode `0b0101`, 64-bit — two 32-bit words)

Pre-loads a buffer from external memory. Used to seed recurrent networks (e.g. GRU).

**Word 1:**

| Bits   | Field |
|--------|-------|
| [2:0]  | Opcode |
| [7:3]  | Target buffer |
| [8]    | Colour |
| [11:9] | #Targets |
| [31:12] | Block size (number of 32-bit words to load) |

**Word 2:**

| Bits   | Field |
|--------|-------|
| [1:0]  | `0b00` (must be zero) |
| [31:2] | Block start address in external memory (32-bit aligned) |

## AXI Memory Map

The accelerator is memory-mapped into a 32-bit address space. Upper 1 MB is used for
configuration; the lower 32 MB gives access to buffer contents.

### Buffer data access (bottom 32 MB, read/write)

| Bits   | Field |
|--------|-------|
| [31:25] | `0b0000000` |
| [24:20] | Buffer ID (0–31) |
| [19:2]  | Buffer offset (1 M of 32-bit words) |
| [1:0]   | `0b00` (32-bit aligned only) |

### Control commands (write-only, `[31:29] = 0b111`, `[28:25] = 0b0000`)

| Reg ID `[24:20]` | Name | Effect |
|------------------|------|--------|
| `0b00000` | LOAD_PC | Set program start address |
| `0b00001` | START | Begin execution at loaded address |
| `0b00010` | CONTINUE | Resume after finish-on-success halt |
| `0b00011` | PAUSE | Halt instruction fetch and table additions |
| `0b00100` | UNPAUSE | Resume after PAUSE |

### Status registers (read-only, `[31:30] = 0b11`, `[29:26] = 0b0001`)

| Reg ID `[25:20]` | Name | Contents |
|------------------|------|----------|
| `0b000000` | CURRENT_PC | Current fetch pointer |
| `0b000001` | ACTIVE | Bits [2:0] = per-accelerator busy; bit 31 = whole system busy; bit 30 = paused on early exit |
| `0b000001` | BUFF_FULL | Per-buffer full flags |
| `0b000010` | BUFF_BUSY | Per-buffer busy flags |
| `0b000011` | BUFF_COLOUR | Per-buffer colour flags |
| `0b100000`–`0b111111` | Per-buffer status (0–31) | Bit 0 = colour, bit 1 = busy, bit 2 = full, bits [5:3] = remaining consumer count |

### Buffer memory map (write-only, `[31:30] = 0b11`, `[29:26] = 0b0010`)

Two writes per buffer: field-select bit `[25]` = 0 sets start address (multiples of 256 bytes);
field-select `[25]` = 1 sets buffer size in bytes (must be multiple of 32).

### Config register bank (write-only, `[31:30] = 0b11`, `[29:26] = 0b0011`)

Two consecutive 32-bit writes: first selects config ID and register ID (`[19:12]`); second
supplies the new value. Config is loaded into the accelerator at the cycle a TASK is dispatched.
Config must be pre-loaded before the TASK instruction runs.

## Signal Naming Convention

- `_i` suffix — input port
- `_o` suffix — output port
