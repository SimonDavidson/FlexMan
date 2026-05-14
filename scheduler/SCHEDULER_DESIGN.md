# FlexMan Scheduler — Technical Design Summary

## Overview

The FlexMan scheduler is a hardware task-dispatch unit designed to orchestrate layer-by-layer
computation across one or more heterogeneous neural-network accelerators (SNN, ANN, Hadamard,
and future variants). Its central problem is a dependency-driven one: in a network with shared
intermediate buffers, no layer may begin until its input data is ready and its output buffer is
free. The scheduler solves this in hardware, freeing the host CPU from cycle-by-cycle resource
management and enabling multiple layers to be in-flight simultaneously across different
accelerators.

The design is implemented in synthesisable SystemVerilog and is currently parameterised for
16 buffers and up to 4 compute accelerators plus one fill-unit slot (`NUM_HW_ACCELERATORS = 5`,
`TGT_ACC_SZ = 3`), with the scheduler table holding up to 4 concurrent pending tasks. TASK
instructions encode a 2-bit accelerator ID in bits [4:3] of word 1, so they can address compute
accelerators 0–3; the fill unit occupies the highest slot (`NUM_HW_ACCELERATORS - 1 = 4`) and
is only reachable via FILL instructions.

---

## Module Hierarchy

```
scheduler.v              — instruction fetch/decode FSM; top-level control
  sch_table.v            — pending-task queue (4 entries); dispatch arbitration
    sch_entry.v × 4      — per-slot readiness logic; fires combinatorially each cycle
  sch_buffer_state.v     — tracks all buffer and accelerator state
    buffer_state_entry.v × 16  — per-buffer: free/full/colour/consumer-count registers
    acc_hw_buffer_tracker.v × 2 — per-accelerator: which buffers in use; busy flag
```

---

## Fetch and Decode

`scheduler.v` contains a small instruction-fetch processor. A 10-bit program counter
addresses a flat 32-bit word memory through a simple request/acknowledge handshake
(`prog_mem_req_o` / `prog_mem_wait_i`). To prevent stalls from causing a fetched word to be
lost, a one-word holding register (`held_inst_word_r`) captures any word that arrives when the
downstream pipeline is not yet ready to consume it. The active instruction is therefore always
taken from whichever of the live memory bus or the holding register is valid.

Decode is combinatorial: a `case` on `inst_word[2:0]` asserts one of eight one-hot flags
(`inst_is_task`, `inst_is_jump`, etc.).

The key handshake signal is `inst_consumed`, which advances the PC and allows a new fetch.
Each instruction type has its own condition for being consumed:

| Instruction    | Consumed when                                                    |
|----------------|------------------------------------------------------------------|
| TASK (word 1)  | A free slot exists in the scheduler table; PC advances to word 2 |
| TASK (word 2)  | Immediately after word 1 is accepted; entry loaded into table    |
| JUMP           | Immediately (unconditional)                                      |
| LOOP / LOOPEND | Immediately                                                      |
| CHECK          | The nominated result buffer is full (i.e. the producing task has completed) |
| NXT            | The task table is completely empty                               |
| FILL           | On completion                                                    |

### Two-Word TASK Fetch

TASK is a 64-bit instruction split across two consecutive 32-bit words. When word 1 is
recognised (`inst_word[2:0] == 3'b000`) and a table slot is free, the fetch engine sets a
`task_w2_pending_r` flag and latches word 1 into `task_w1_r`. Normal instruction decode is
suppressed while `task_w2_pending_r` is set, so the following word cannot be misinterpreted
as a new instruction. When the next word arrives it is latched into `task_w2_r` and consumed
immediately (`inst_consumed_w2`), clearing the pending flag. `load_new_entry` fires on word 2
arrival — not word 1 — so the scheduler table entry is only created once both words are held
and the full entry can be packed.

The NXT instruction is therefore a full pipeline barrier: the fetch engine stalls until every
in-flight task has completed and the table drains to zero, then pulses `nxt_input_pulse_o`
and/or `nxt_output_pulse_o` for one cycle before continuing. This is the natural synchronisation
point between timesteps in a temporal SNN or between passes in a recurrent ANN.

### Control Flow

Three control-flow mechanisms are implemented beyond sequential fetch:

**JUMP** — absolute branch, word-addressed, unconditional.

**LOOP / LOOPEND** — eight independent hardware loop counters (`loop_counter_r[0:7]`), each
paired with a saved restart address (`loop_restart_r`). LOOP initialises the counter from
`inst_word[31:6]` (26-bit range) and records PC+1 as the restart target. LOOPEND decrements
the counter and jumps back if it is non-zero, otherwise falls through. This supports the
repeated timestep loops common in SNN inference without requiring the host CPU to manage
iteration state.

**CHECK** — conditional branch for dynamic neural networks (DyNN). The CHECK instruction
stalls until the nominated buffer is full (i.e. the task that produces the branch decision has
completed), then reads the result from the `target_status_r` register file. A result of 0
means success. Two modes are supported: *skip-on-success* (jump to a given address, bypassing
subsequent layers) and *finish-on-success* (terminate execution and signal the host CPU via
the system bus, used when an early-exit criterion is met in a conditional-computation network).

---

## Task Table and Dispatch

### Table Structure

`sch_table.v` holds up to four pending TASK entries in a compact shift-register queue.
New entries always load into the highest-index slot (slot 3). Occupied slots compact downward
toward slot 0 as earlier entries are dispatched: the shift logic detects gaps and propagates
entries toward the head in the same cycle that a dispatch frees a slot, so the table stays
densely packed.

The fetch engine is stalled (`table_slot_free_o` deasserted) only when slot 3 is already
occupied. As long as the tail slot is free, the fetch can load a new TASK even while the
remaining three slots hold pending work. This gives a consistent look-ahead window of up to
four tasks beyond whatever is currently executing.

### Readiness and Arbitration

Each `sch_entry` instance evaluates readiness **combinatorially every cycle** across all six
buffer slots and the target accelerator. Each slot carries a 2-bit mode field that controls
which checks are applied:

| Mode                  | Full+colour required | Free required |
|-----------------------|----------------------|---------------|
| UNUSED     (`00`)     | No                   | No            |
| SOURCE     (`01`)     | Yes                  | No            |
| READ-WRITE (`10`)     | Yes                  | No            |
| TARGET     (`11`)     | No                   | Yes           |

For READ-WRITE slots the "not busy" condition is implicitly captured by the full check: a busy
buffer has `full=0`, so the full check fails. A separate free check would incorrectly block
dispatch because a full buffer always has `free=0`.

For each slot the entry computes two single-bit signals:
- `slot_src_ok[s]` — true if the slot does not need a full buffer, or if `buffers_full_i[id]`
  is set **and** `buffers_colour_i[id]` matches the stored entry colour.
- `slot_tgt_ok[s]` — true if the slot does not need a free buffer, or if `buffers_free_i[id]`
  is set. Only TARGET slots set this requirement; RW slots do not.

`ready_to_execute_o` is the AND of all twelve slot signals plus the accelerator-free check
(entry acc_id decoded to one-hot, ANDed with `~acc_busy_i`). This is entirely combinatorial:
any change to buffer state or accelerator state propagates to the readiness output within the
same cycle, so dispatch latency after a dependency clears is exactly one cycle.

The table then selects the **lowest-index** ready entry for dispatch. Because entries compact
toward slot 0 over time, this approximates FIFO ordering among tasks that become ready
simultaneously, preserving program-order priority while still allowing any ready task to fire
regardless of whether earlier-issued tasks are still waiting.

When `dispatch_to_acc_o` fires, `scheduler.v` drives `start_new_block_o`, `target_acc_o`,
and `buffer_info_o` to the appropriate accelerator. The selected entry is simultaneously
removed from the table.

---

## Buffer State Tracking

`sch_buffer_state.v` maintains the full resource picture and broadcasts it to the table every
cycle. It instantiates one `buffer_state_entry` per buffer and one `acc_hw_buffer_tracker`
per accelerator.

### Per-Buffer State (`buffer_state_entry.v`)

Each buffer holds four registered fields:

| Field | Register | Set by | Cleared by |
|-------|----------|--------|-----------|
| **free** | `buff_free_r` | Reset; last consumer dispatched | TARGET/RW dispatch; external mark-full |
| **full** | `buff_full_r` | Accelerator completion; external mark-full | RW dispatch (full→busy); last consumer dispatched |
| **colour** | `buff_colour_r` | Task dispatch (from TASK colour field) | — (changes only on new dispatch) |
| **consumer count** | `buff_usage_count_r` | Accelerator completion (from stored ntgt); external mark-full | Decremented each time a SOURCE consumer dispatches |

Three distinct dispatch strobes drive state transitions:

- **`buff_new_tgt_i`** (TARGET mode): clears `free`; `full` stays 0. Usage count is loaded
  from the stored `#Targets` when the accelerator completes.
- **`buff_rw_claim_i`** (READ-WRITE mode): clears both `free` and `full` simultaneously,
  transitioning the buffer from full to busy. The old consumer count is discarded.
- **`buff_content_consumed_i`** (SOURCE mode): decrements the usage counter. When the counter
  reaches 1 and this strobe fires, `buff_no_longer_needed` asserts, clearing `full` and
  restoring `free` in the same clock edge.

On accelerator completion, `buff_now_full_i` marks the buffer full and loads `buff_now_usage_count_i`
with the `#Targets` stored in the accelerator tracker at dispatch time. This applies to both
TARGET and READ-WRITE output slots.

The colour field is broadcast via `buffers_colour_o` and is now actively checked in `sch_entry`:
a SOURCE or READ-WRITE slot is only considered satisfied when `buffers_full_i[id]` is set
**and** `buffers_colour_i[id]` matches the task's colour field.

### Per-Accelerator State (`acc_hw_buffer_tracker.v`)

Each accelerator tracker stores the full six-slot descriptor for its currently executing task:
for each slot it retains the buffer ID, 2-bit mode, and (for output slots) the `#targets`
count. These are loaded on dispatch (`new_task_i`) and held until the accelerator signals
completion. The tracker also maintains an `acc_free_r` busy flag, set on dispatch and cleared
on the `task_finished_i` completion signal. The stored `(id, mode, ntgt)` tuples are broadcast
back to `sch_buffer_state` as `slot_buff_o`, `slot_mode_o`, and `slot_ntgt_o` so the
completion logic can drive the correct per-buffer strobes without re-reading the original
instruction.

### Completion Handling

Accelerators signal completion by asserting `acc_finished_i`. Multiple accelerators may finish
within the same cycle; to avoid a conflict when updating shared buffer state, completions are
queued in a one-hot pending register (`acc_process_pending_r`) and serviced one per cycle.
The priority encoder picks the lowest-index pending accelerator, reads its stored six-slot
descriptor from the tracker, and iterates over all six slots: any slot whose mode is `MODE_RW`
or `MODE_TGT` causes `buff_now_full[id]` to be asserted and the corresponding `buff_now_ntgt`
to be loaded from the slot's stored `#targets`. SOURCE and UNUSED slots are ignored at
completion time — SOURCE buffers were already decremented at dispatch. The pending bit is
cleared the cycle after it is serviced.

The accelerator also drives an `acc_result_i` bit on completion. This is captured into a
per-buffer result register file (`target_status_r`) indexed by the completing task's target
buffer. The CHECK instruction reads from this register file to decide branch direction, which
means CHECK reliably sees the result of the immediately preceding layer that wrote to the
nominated buffer.

### External Buffer Seeding

A separate `mark_buff_as_full_i` / `full_buff_id_i` / `full_buff_usage_i` interface allows
an external agent (host CPU or DMA controller) to mark a buffer as full without a preceding
TASK dispatch. This is used to inject input data (e.g. the initial hidden state of a GRU, or
the first spike/rate frame of an SNN inference) before the program begins executing.

---

## Temporal SNN Operation

For a rate-coded or time-stepped SNN running over T timesteps, the typical program structure
is:

1. A LOOP instruction setting the iteration count to T.
2. A NXT instruction to block until all layers have completed and then advance the input
   pointer (consuming the next input frame).
3. TASK instructions for all layers in the network, which the scheduler will dispatch
   out-of-order as their buffer dependencies are satisfied.
4. A NXT instruction (or combined NXT) to advance the output pointer after all layers complete.
5. A LOOPEND instruction to iterate.
6. STOP.

Recurrent connectivity (e.g. the feedback path in an LIF network or the hidden state in a GRU)
is handled through the buffer colour mechanism: alternate passes use alternating colour values
so the scheduler can distinguish old hidden-state data (to be consumed) from newly computed
data (being produced concurrently) in the same physical buffer pair. The FILL instruction seeds
the initial recurrent state.

---

## Implementation Notes

- The FILL instruction dispatches through the normal scheduler table mechanism: it creates a
  table entry with the destination buffer as a TARGET slot and targets the fill-unit accelerator
  (`FILL_ACC_ID = NUM_HW_ACCELERATORS - 1`). Completion is signalled via `acc_finished_i` like
  any other accelerator. No fill-unit RTL exists yet; in simulation a fill-unit stub must be
  provided or FILL entries will never complete and the table will stall.
- TASK is a two-word (64-bit) instruction. Slots 0–2 (word 1) carry only mode and buffer ID;
  they must therefore be UNUSED or SOURCE. Slots 3–5 (word 2) carry mode, buffer ID, and
  `#targets`, and support all four modes. Attempting to use READ-WRITE or TARGET in slots 0–2
  will produce incorrect behaviour because no `#targets` field is decoded for those slots.
- The table capacity (4 entries) is below the 16-entry spec target. The shift-register
  compaction structure generalises directly to larger tables by increasing `NUM_SCH_ENTRIES`.
- All readiness evaluation is purely combinatorial and resolves within a single clock cycle,
  so dispatch latency after a dependency clears is exactly one cycle.
