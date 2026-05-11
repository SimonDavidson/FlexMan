# FlexMan Scheduler — Programmer's ISA Reference

This document describes the instruction set and memory-mapped interface for the FlexMan
scheduler. It is intended for software engineers writing programs that run on the accelerator
or tools that generate such programs.

---

## Execution Model

The scheduler reads a flat array of 32-bit words from external memory (the *program*). It
fetches instructions sequentially and dispatches layer-computation tasks to one or more
hardware accelerators (e.g. SNN, Hadamard). Tasks are held in a scheduler table and run
out-of-order, subject to buffer dependency constraints. The program controls ordering through
the buffer system; direct sequencing is not otherwise enforced between TASK instructions.

### Buffers

There are 16 hardware buffers (buffer IDs 0–15). Each buffer is associated with four pieces
of state maintained by the scheduler:

| Field | Meaning |
|-------|---------|
| **busy** | A task is writing to this buffer but has not yet finished. No other write task may target this buffer until it clears. |
| **full** | The writing task has completed. Consumer tasks may now read this buffer. |
| **colour** | A 1-bit tag. A consumer task only claims a full buffer if its colour field matches. Prevents a stale or reordered task from consuming data intended for a later task. |
| **counter** | The number of consumers still to read this buffer (loaded from the TASK `#Targets` field). Decremented each time a consumer is dispatched. When it reaches zero, `full` clears, colour flips, and the buffer is free again. |

### Dispatch Conditions

A TASK entry in the scheduler table fires only when **all three** conditions hold simultaneously:

1. The target accelerator is free.
2. All source buffers are **full** and have a **colour matching** the task's colour field.
3. The destination buffer is **free** (neither busy nor full).

---

## Instruction Encoding

All instructions are 32-bit words and 32-bit aligned, **except FILL**, which occupies two
consecutive 32-bit words (64 bits total). The opcode occupies bits `[2:0]`. When bit 2 of the
opcode is set, the instruction is two words long (currently only FILL uses this).

### Opcode Table

| Opcode (binary) | Mnemonic | Description |
|-----------------|----------|-------------|
| `000` | TASK | Dispatch a computation to an accelerator |
| `001` | JUMP | Unconditional absolute branch |
| `010` | STOP | End program; wait for outstanding tasks |
| `011` | CHECK | Conditional branch (dynamic neural networks) |
| `100` | NXT | Advance input/output data window (barrier) |
| `101` | FILL | Pre-load a buffer from external memory (2 words) |

---

## Instruction Reference

### TASK — `opcode 0b000`

Dispatches a single layer computation to a hardware accelerator. The instruction specifies
which accelerator to use, which configuration to load into it, which source buffers to read,
and which destination buffer to write.

```
 31      30      29      25    24      20    19    17   16    12   11     9    8      7     3   2    0
 ┌───────────────────────────┬──────────────┬──────────┬─────────┬────────┬────────┬──────────────┬──────┐
 │    Source buffer 2 [4:0]  │ Src buf 1[4:0]│  Acc ID  │ Cfg ID  │#Targets│ Colour │  Dst buf[4:0]│  op  │
 └───────────────────────────┴──────────────┴──────────┴─────────┴────────┴────────┴──────────────┴──────┘
```

| Bits    | Field | Notes |
|---------|-------|-------|
| [2:0]   | Opcode | `3'b000` |
| [7:3]   | Destination buffer | Buffer ID this task writes to |
| [8]     | Colour | Must match source buffer colours; becomes the colour written to the destination |
| [11:9]  | #Targets | Number of consumer tasks that will read the destination buffer |
| [16:12] | Config ID | Selects a pre-loaded configuration register set for the accelerator |
| [19:17] | Accelerator ID | Selects the target hardware accelerator |
| [24:20] | Source buffer 1 | First input buffer |
| [29:25] | Source buffer 2 | Second input buffer (set equal to source 1 if unused) |
| [31:30] | Reserved | Set to zero |

**Notes:**
- Configuration for the accelerator (weights, dimensions, etc.) must be written to the config
  register bank **before** the TASK instruction is fetched. See the AXI map section.
- The destination buffer's `busy` flag is set immediately on dispatch; `full` is set when the
  accelerator signals completion.
- #Targets controls how many times the destination buffer can be consumed before it is freed.
  Set to 1 for a buffer read by exactly one downstream task.

---

### JUMP — `opcode 0b001`

Unconditional absolute branch. The address is in units of 32-bit words (i.e. word index, not
byte address).

```
 31                              3   2    0
 ┌──────────────────────────────────┬──────┐
 │          Jump address [28:0]     │  op  │
 └──────────────────────────────────┴──────┘
```

| Bits   | Field | Notes |
|--------|-------|-------|
| [2:0]  | Opcode | `3'b001` |
| [31:3] | Target word address | Absolute address; jumps here next cycle |

---

### STOP — `opcode 0b010`

Halts instruction fetch. The scheduler waits for all outstanding tasks in the table to
complete, then signals *finished* to the host CPU via the system bus. A subsequent START
command (see AXI map) resets all buffer and accelerator state and begins a new program.

```
 31              4   3    0
 ┌───────────────────┬──────┐
 │      (ignored)    │  op  │
 └───────────────────┴──────┘
```

| Bits  | Field | Notes |
|-------|-------|-------|
| [3:0] | Opcode | `4'b0010`; only the low bits are meaningful |

---

### CHECK — `opcode 0b011`

Conditional branch for dynamic neural networks (DyNN). Checks a success/fail signal from a
nominated unit and either terminates early or skips a block of instructions.

```
 31              12   11    10       4   3         2    0
 ┌────────────────────┬──────┬────────┬───┬──────────────┐
 │   Skip address     │colour│unit ID │mode│      op      │
 └────────────────────┴──────┴────────┴───┴──────────────┘
```

| Bits    | Field | Notes |
|---------|-------|-------|
| [2:0]   | Opcode | `3'b011` |
| [3]     | Mode | `1` = finish on success; `0` = skip on success |
| [10:4]  | Check unit ID | Selects which unit's result to inspect |
| [11]    | Colour | Identifies which task-colour group to cancel on early exit |
| [31:12] | Skip address | Word address to jump to in skip-on-success mode |

**Behaviour:**

- **Finish on success (`mode=1`):** Terminates execution. All tasks of the matching colour are
  cancelled. Scheduler halts and waits for a CONTINUE command from the host before resuming.
- **Skip on success (`mode=0`):** Jumps to the skip address, bypassing subsequent instructions
  (e.g. skipping layers in a conditional branch of a DyNN).
- **On fail (either mode):** Continues with the next instruction in sequence.

---

### NXT — `opcode 0b100`

Advances the time-window pointer for input data, output data, or both. Acts as a **barrier**:
the scheduler stalls until every entry currently in the task table has completed, then fires
the selected pulse(s) for exactly one cycle before continuing.

```
 31              6   5        4   3         2    0
 ┌────────────────────┬────────┬───┬──────────────┐
 │      (reserved)    │nxt_out │rsv│   op          │
 │                    │nxt_in  │   │               │
 └────────────────────┴────────┴───┴──────────────┘
```

| Bits  | Field | Notes |
|-------|-------|-------|
| [2:0] | Opcode | `3'b100` |
| [3]   | Reserved | Set to zero |
| [4]   | `nxt_in` | Pulse `nxt_input_pulse_o` — advance the input read pointer |
| [5]   | `nxt_out` | Pulse `nxt_output_pulse_o` — advance the output write pointer |
| [31:6] | Reserved | Set to zero |

Both bits 4 and 5 may be set simultaneously to advance input and output in the same cycle.

---

### FILL — `opcode 0b101` (two words)

Pre-loads a buffer from a block of external memory. Used to seed recurrent network state
(e.g. GRU hidden state). This is the only two-word instruction; the low two bits of word 2
must be `0b00` (the hardware uses these as a sentinel).

**Word 1:**

```
 31              12   11     9   8      7     3   2    0
 ┌────────────────────┬────────┬──────────────────┬──────┐
 │   Block size[19:0] │#Targets│ colour │ dst buf  │  op  │
 └────────────────────┴────────┴──────────────────┴──────┘
```

| Bits    | Field | Notes |
|---------|-------|-------|
| [2:0]   | Opcode | `3'b101` |
| [7:3]   | Destination buffer | Buffer ID to fill |
| [8]     | Colour | Colour to apply to the destination buffer |
| [11:9]  | #Targets | Consumer count for this buffer |
| [31:12] | Block size | Number of 32-bit words to load |

**Word 2:**

```
 31                              2   1    0
 ┌──────────────────────────────────┬──────┐
 │   Block start address [31:2]     │ 0b00 │
 └──────────────────────────────────┴──────┘
```

| Bits   | Field | Notes |
|--------|-------|-------|
| [1:0]  | Must be `0b00` | Alignment sentinel; also ensures the address is 32-bit aligned |
| [31:2] | Start address | Word-aligned byte address in external memory |

---

## AXI Memory-Mapped Interface

The scheduler is accessed through a 32-bit AXI address space. The upper 1 MB is reserved for
control and configuration; the lower 32 MB provides direct access to buffer contents.

### Buffer Data (read/write) — addresses `0x0000_0000`–`0x01FF_FFFF`

| Bits    | Field |
|---------|-------|
| [31:25] | `7'b000_0000` |
| [24:20] | Buffer ID (0–31) |
| [19:2]  | Word offset within buffer (up to 1 M words) |
| [1:0]   | `2'b00` (32-bit aligned access only) |

### Control Commands (write-only)

Address qualifier: bits `[31:29] = 3'b111`, bits `[28:25] = 4'b0000`.

| Reg ID `[24:20]` | Name | Action |
|------------------|------|--------|
| `5'b00000` | LOAD_PC | Set program start address (written as word index) |
| `5'b00001` | START | Begin execution at the loaded address; resets all state |
| `5'b00010` | CONTINUE | Resume after a finish-on-success halt |
| `5'b00011` | PAUSE | Halt instruction fetch (in-flight tasks complete normally) |
| `5'b00100` | UNPAUSE | Resume after PAUSE |

### Status Registers (read-only)

Address qualifier: bits `[31:30] = 2'b11`, bits `[29:26] = 4'b0001`.

| Reg ID `[25:20]` | Name | Contents |
|------------------|------|----------|
| `6'b00_0000` | CURRENT_PC | Current fetch word address |
| `6'b00_0001` | ACTIVE | `[2:0]` = per-accelerator busy; `[30]` = paused on early exit; `[31]` = system busy |
| `6'b00_0001` | BUFF_FULL | Per-buffer full flags (one bit per buffer) |
| `6'b00_0010` | BUFF_BUSY | Per-buffer busy flags |
| `6'b00_0011` | BUFF_COLOUR | Per-buffer colour flags |
| `6'b10_0000`–`6'b11_1111` | Per-buffer status (buffers 0–31) | `[0]`=colour, `[1]`=busy, `[2]`=full, `[5:3]`=remaining consumer count |

### Buffer Memory Map (write-only)

Address qualifier: bits `[31:30] = 2'b11`, bits `[29:26] = 4'b0010`.

Two writes per buffer, selected by bit `[25]`:
- `[25]=0` — set buffer start address (must be a multiple of 256 bytes)
- `[25]=1` — set buffer size in bytes (must be a multiple of 32)

### Config Register Bank (write-only)

Address qualifier: bits `[31:30] = 2'b11`, bits `[29:26] = 4'b0011`.

Two consecutive 32-bit writes per configuration entry:
1. First write: Config ID in `[19:16]`, register ID within that config in `[15:8]`.
2. Second write: The new register value.

Configuration must be written **before** the TASK instruction that references it is fetched
and dispatched. The values are pushed into the accelerator at dispatch time.

---

## Programming Notes

1. **Buffer lifecycle.** Write a TASK whose destination buffer is B with `#Targets = N`. That
   buffer becomes busy on dispatch and full on completion. Each of the N downstream TASKs that
   lists B as a source decrements the counter; after the Nth consumer is dispatched, B is
   automatically freed and its colour flipped.

2. **Colour usage.** Colour is a 1-bit field. Use it to distinguish two generations of data in
   the same buffer. Alternate 0 and 1 in successive passes through a recurrent loop; the
   consumer tasks will only fire when the colour matches, preventing a slow pass from consuming
   data produced for a later pass.

3. **Configuration must precede TASK.** The config register bank has no ordering guarantee
   relative to in-flight tasks. Write all accelerator configuration via the AXI interface
   before the program starts, or before the TASK that needs it is reached.

4. **FILL seeding.** Use FILL to pre-populate a recurrent buffer (e.g. initial hidden state h₀)
   before the first iteration. Its `#Targets` and colour work identically to a TASK's
   destination buffer, so downstream TASKs can treat it like any other full buffer.

5. **NXT as a synchronisation barrier.** NXT stalls the fetch pipeline until the task table
   drains. Use it at the end of each timestep in a temporal SNN to ensure all layers have
   completed before advancing the input window.

6. **STOP and restart.** After STOP, the host writes START (with a new LOAD_PC if needed) to
   begin a new program. START resets all buffer and accelerator state; any data in buffers
   from the previous run is no longer tracked.
