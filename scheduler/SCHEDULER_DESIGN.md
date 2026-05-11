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
16 buffers and 2 hardware accelerators, with the scheduler table holding up to 4 concurrent
pending tasks.

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

| Instruction | Consumed when |
|-------------|---------------|
| TASK | A free slot exists in the scheduler table |
| JUMP | Immediately (unconditional) |
| LOOP / LOOPEND | Immediately |
| CHECK | The nominated result buffer is full (i.e. the producing task has completed) |
| NXT | The task table is completely empty |
| FILL | On DMA completion *(stub — not yet implemented)* |

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

Each `sch_entry` instance evaluates three conditions **combinatorially every cycle**, without
waiting for an explicit check request:

1. **Source buffers full** — constructs a one-hot mask of required source buffers from the
   stored buffer IDs, ANDs with the broadcast `buffers_full_i` vector, and asserts
   `got_all_inbuffers` when no required buffer is still empty.

2. **Target buffer free** — same one-hot construction for the destination buffer, checked
   against `buffers_free_i`.

3. **Accelerator free** — decodes the stored accelerator ID into a one-hot and checks it
   against the broadcast `acc_busy_i` vector.

`ready_to_execute_o` is the AND of all three. This is purely combinatorial: state changes in
any buffer or accelerator propagate to the readiness signals within the same cycle, without
registering.

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
| **free** | `buff_free_r` | Reset; last consumer dispatched | Task dispatch targeting this buffer; external mark-full |
| **full** | `buff_full_r` | Accelerator completion signal; external mark-full | Last consumer dispatched |
| **colour** | `buff_colour_r` | Task dispatch (from TASK colour field) | — (colour changes only on new dispatch) |
| **consumer count** | `buff_usage_count_r` | Task dispatch (#Targets field); external mark-full | Decremented each time a consumer task dispatches |

The critical lifecycle is: a TASK dispatch sets the target buffer's `free` to 0 and loads the
consumer count; on accelerator completion, `full` is set to 1; each time a downstream TASK
consuming this buffer is dispatched, the counter decrements; when it reaches 1 and the last
consumer dispatches, `full` clears and `free` returns to 1 in the same clock edge
(`buff_no_longer_needed`).

Note that in the current RTL, the colour field is tracked per-buffer and broadcast via
`buffers_colour_o`, but the `sch_entry` readiness logic does not yet filter `buffers_full_i`
by colour: any task whose source buffer is full will see it as available regardless of colour
match. The colour mechanism is architecturally complete in the state-tracking layer but the
dispatch gate in `sch_entry` treats full as sufficient. This is a known gap relative to the
spec.

### Per-Accelerator State (`acc_hw_buffer_tracker.v`)

Each accelerator tracker stores the target buffer ID and up to three source buffer IDs for its
currently executing task. These are loaded on dispatch (`new_task_i`) and held until the
accelerator signals completion. The tracker also maintains an `acc_free_r` busy flag, set on
dispatch and cleared on the `task_finished_i` completion signal.

### Completion Handling

Accelerators signal completion by asserting `acc_finished_i`. Multiple accelerators may finish
within the same cycle; to avoid a conflict when updating shared buffer state, completions are
queued in a one-hot pending register (`acc_process_pending_r`) and serviced one per cycle.
The priority encoder picks the lowest-index pending accelerator, looks up its buffer IDs from
the tracker, and drives the appropriate `buff_now_full` and `buff_content_consumed` strobes
into the buffer state entries. The pending bit is cleared the cycle after it is serviced.

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

- The FILL instruction's completion logic is stubbed (`fill_complete = 1'b0`), so FILL
  instructions stall indefinitely in the current RTL.
- The third source buffer field (`src3`) is wired as a duplicate of `src2` in the instruction
  decode path (`d[ENTRY_SBUFF3] = inst_word[29:25]`) — effectively only two source buffers
  are decoded from the instruction word, with the third slot tracking the same buffer as the
  second.
- The table capacity (4 entries) is below the 16-entry spec target. The shift-register
  compaction structure generalises directly to larger tables by increasing `NUM_SCH_ENTRIES`.
- All readiness evaluation is purely combinatorial and resolves within a single clock cycle,
  so dispatch latency after a dependency clears is exactly one cycle.
