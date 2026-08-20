# FlexMan Scheduler — Programmer's ISA Reference

This document describes the instruction set and memory-mapped interface for the FlexMan
scheduler. It is intended for software engineers writing programs that run on the accelerator
or tools that generate such programs.

---

## Execution Model

The scheduler reads a flat array of 32-bit words from external memory (the *program*). It
fetches instructions sequentially and dispatches layer-computation tasks to one or more
hardware accelerators (e.g. SNN, Hadamard). Tasks are held in a scheduler table and dispatched
**strictly in program order** — a task waits until it is the oldest entry in the table before it
can dispatch, and only then when its buffer dependencies are satisfied. (A task may still
*complete* out of order, on whichever accelerator finishes first.) In-order dispatch, together
with the per-buffer state, is what guarantees correct read/write ordering on shared buffers
without register renaming or a reorder buffer. The program therefore controls ordering through
both program order and the buffer system: producers must precede their consumers in the program,
and the compiler should interleave independent tasks so accelerators stay busy while a dependent
task waits at the head of the table.

Hardware loop counters (LOOP / LOOPEND) allow compact programs with repeated structure
(e.g. iterating over timesteps in a spiking network). Eight independent counters are
available, enabling up to 8 levels of nesting provided each level uses a distinct loop ID.

### Buffers

There are 16 hardware buffers (buffer IDs 0–15). Each buffer is associated with four pieces
of state maintained by the scheduler:

| Field | Meaning |
|-------|---------|
| **busy** | A task has claimed this buffer but has not yet completed. No other task may claim it until busy clears. |
| **full** | The task that owns this buffer has completed. Consumer tasks may now read it. |
| **colour** | A 1-bit tag, set on a producer's dispatch and checked on a consumer's dispatch (consumer colour must match). **Vestigial:** with strict in-order dispatch it is redundant for generation separation, and the hardware never auto-manages it (it does *not* flip when the buffer frees — the program would have to alternate it). Current programs leave it 0. |
| **counter** | The number of consumers still to read this buffer (loaded from the TASK `#Targets` field). Decremented each time a consumer **completes** (not when it is dispatched — accelerators stream the buffer live until they finish). When it reaches zero, `full` clears and the buffer becomes free again. |

### Buffer Slot Modes

Each buffer slot in a TASK instruction carries a 2-bit mode field:

| Mode | Encoding | Dispatch condition | Action on dispatch | Action on completion |
|------|----------|-------------------|--------------------|----------------------|
| **UNUSED** | `2'b00` | None (slot skipped) | None | None |
| **SOURCE** | `2'b01` | Buffer must be **full** and colour must match | None | Decrement usage counter (frees buffer when it hits zero) |
| **READ-WRITE** | `2'b10` | Buffer must be **full** and colour must match | Mark buffer **busy** (clears both full and free) | Mark buffer **full** with stored #Targets |
| **TARGET** | `2'b11` | Buffer must be **free** | Reserve buffer (clears free) | Mark buffer **full** with stored #Targets |

The READ-WRITE mode supports in-place buffers, such as the synaptic current and membrane
potential accumulators in SNN layers. These must be fully computed before the task starts,
go busy during execution, and are refilled with the updated values on completion. They then
behave as ordinary full buffers, consumable by downstream tasks.

### Dispatch Conditions

A TASK entry in the scheduler table fires only when **all** of the following hold simultaneously:

1. The target accelerator is free.
2. Every SOURCE or READ-WRITE slot has its buffer **full** with a **colour matching** the task's colour field.
3. Every READ-WRITE or TARGET slot has its buffer **free** (not busy, not full).

---

## Instruction Encoding

All instructions are 32-bit aligned. Most are a single 32-bit word; **TASK** and **FILL** each
occupy two consecutive 32-bit words (64 bits total). The opcode occupies bits `[2:0]` of the
first word. The second word of a two-word instruction always has `[1:0] = 2'b00` as a sentinel.

### Opcode Table

| Opcode (binary) | Mnemonic | Description |
|-----------------|----------|-------------|
| `000` | TASK | Dispatch a computation to an accelerator |
| `001` | JUMP | Unconditional absolute branch |
| `010` | STOP | End program; wait for outstanding tasks |
| `011` | CHECK | Conditional branch (dynamic neural networks) |
| `100` | NXT | Advance input/output data window (barrier) |
| `101` | FILL | Pre-load a buffer from external memory (2 words) |
| `110` | LOOP | Initialise a hardware loop counter |
| `111` | LOOPEND | Close a hardware loop |

---

## Instruction Reference

### TASK — `opcode 0b000` (two words)

Dispatches a single layer computation to a hardware accelerator. TASK is a **two-word
instruction**: the scheduler fetches and decodes both words before creating a table entry.
Up to six buffer slots are specified, each with an independent mode field.

**Word 1:** (`cfg_id` is a 7-bit field `[11:5]`; colour and the three short slots are
shifted up by 2 bits relative to the legacy 5-bit layout.)

```
 31  30   27  26  25  24   21  20  19  18   15  14  13  12  11         5   4   3   2    0
 ┌────┬───────┬──────┬───────┬──────┬───────┬──────┬──────┬─────────────┬──────┬──────┐
 │rsv │slot2  │slot2 │slot1  │slot1 │slot0  │slot0 │colour│   cfg_id     │acc_id│  op  │
 │    │buf_id │ mode │buf_id │ mode │buf_id │ mode │      │   [11:5]     │      │      │
 └────┴───────┴──────┴───────┴──────┴───────┴──────┴──────┴─────────────┴──────┴──────┘
```

| Bits    | Field | Notes |
|---------|-------|-------|
| [2:0]   | Opcode | `3'b000` |
| [4:3]   | Accelerator ID | 2-bit field; selects the target hardware accelerator |
| [11:5]  | Config ID | 7-bit field; selects a pre-loaded configuration register set (up to 128) |
| [12]    | Colour | Applied to all SOURCE and READ-WRITE slots; written to TARGET/RW outputs |
| [14:13] | Slot 0 mode | 2-bit mode (see table above) |
| [18:15] | Slot 0 buffer ID | 4-bit buffer ID |
| [20:19] | Slot 1 mode | 2-bit mode |
| [24:21] | Slot 1 buffer ID | 4-bit buffer ID |
| [26:25] | Slot 2 mode | 2-bit mode |
| [30:27] | Slot 2 buffer ID | 4-bit buffer ID |
| [31]    | **Wide-form selector** | `0` = narrow, two-word TASK (the encoding below). `1` = **wide**, three-word TASK — see *Wide TASK form*. Was documented as reserved/zero, and every program image generated before 2026-08-20 has it clear, which is what makes the wide form a pure extension. |

**Word 2:**

```
 31    28  27   24  23  22  21    18  17   14  13  12  11    8   7    4   3   2   1   0
 ┌────────┬───────┬──────┬────────┬───────┬──────┬────────┬───────┬──────┬──────┐
 │ slot5  │slot5  │slot5 │ slot4  │slot4  │slot4 │ slot3  │slot3  │slot3 │ 0b00 │
 │ #tgts  │buf_id │ mode │ #tgts  │buf_id │ mode │ #tgts  │buf_id │ mode │      │
 └────────┴───────┴──────┴────────┴───────┴──────┴────────┴───────┴──────┴──────┘
```

| Bits    | Field | Notes |
|---------|-------|-------|
| [1:0]   | Sentinel | Must be `2'b00` |
| [3:2]   | Slot 3 mode | 2-bit mode |
| [7:4]   | Slot 3 buffer ID | 4-bit buffer ID |
| [11:8]  | Slot 3 #Targets | **4-bit** consumer count, max 15 (used for READ-WRITE and TARGET modes) |
| [13:12] | Slot 4 mode | 2-bit mode |
| [17:14] | Slot 4 buffer ID | 4-bit buffer ID |
| [21:18] | Slot 4 #Targets | **4-bit** consumer count, max 15 |
| [23:22] | Slot 5 mode | 2-bit mode |
| [27:24] | Slot 5 buffer ID | 4-bit buffer ID |
| [31:28] | Slot 5 #Targets | **4-bit** consumer count, max 15 |

The `#Targets` (usage count) field was widened 3→4 bits (max 7→15), packing word 2 exactly to
bit 31 (no reserved bits). The 4-bit field is the natural ceiling for three long slots in a 32-bit
word; it covers a recurrent buffer shared by N output-partition lanes (read by 6N+1 consumers — 13
at N=2). 5-bit would need 33 bits — it does not fit.

### Wide TASK form (2026-08-20)

Block-diagonal (Monarch) networks at higher block counts need usage counts well past
15 — 21 at nblocks=8, 45 at 20, **85 at 40**. Three long slots at 7-bit `#Targets`
need 3×13 = 39 bits, which will not fit word 2. Rather than re-balance the existing
format (losing a long slot, which Bosch uses) or grow every TASK by a word, a
**second TASK form** is selected per instruction by **word 1 bit 31**:

| | words | long slots | `#Targets` |
|---|---|---|---|
| narrow (`w1[31]=0`) | 2 | 3 | 4-bit, max 15 |
| **wide** (`w1[31]=1`) | 3 | 3 | **7-bit, max 127** |

Word 1 is otherwise **identical**. Words 2 and 3 pack long slots at the
`SLOT_LONG_SZ = mode(2) + buf_id(BUFF_INDX_SZ) + #tgts(TGT_COUNT_SZ)` stride, in the
same `{mode, buf_id, #tgts}` lsb-first order the scheduler uses internally — so a
slot is never split across a word, and decode is a straight slice copy that
rescales with the parameters.

**Word 2 (wide)** — `[1:0]` sentinel `2'b00`, slot 3 at bit 2, slot 4 at bit
`2+SLOT_LONG_SZ`, `[31:2+2*SLOT_LONG_SZ]` spare (4 bits at TGT_COUNT_SZ=7).
**Word 3 (wide)** — slot 5 at bit 0, `[31:SLOT_LONG_SZ]` spare (19 bits).

Both forms may be **mixed freely in one program**: an assembler should emit the
narrow form whenever every count fits 4 bits, so programs grow only where needed.

Enabled by the scheduler's `WIDE_NTGT` parameter (default 0). With `WIDE_NTGT=0`,
bit 31 is ignored exactly as before and the decode is bit-identical.

**FILL is different, and this is the one backward-incompatible part.** FILL word 1
has no spare bit for a per-instruction selector, so its widened `#Targets` is a
**build-wide** choice governed by the same `WIDE_NTGT` parameter:

| | `#Targets` | Block size |
|---|---|---|
| `WIDE_NTGT=0` | `[12:9]`, 4-bit | `[31:13]`, 19-bit (512K words) |
| `WIDE_NTGT=1` | `[15:9]`, **7-bit** | `[31:16]`, 16-bit (65,536 words) |

A `WIDE_NTGT=1` build therefore requires a program whose FILLs use the wide
encoding; a legacy image will not run on it. 16 bits of block size remains ample —
the largest fill in any current schedule is 632 words.

**Why FILL has no per-instruction selector, when bit 7 is free.** FILL word 1 bit 7
*is* unused — `buf_id` occupies `[6:3]` and `colour` sits at `[8]`, and it is clear
in all 111 FILL instructions across every committed image. It could carry a
selector exactly like TASK's bit 31. It is **deliberately reserved instead**,
because it is free only while `BUFF_INDX_SZ = 4`: at 32 buffers `buf_id` becomes
`[7:3]` and consumes it. That ceiling is already binding — the N=2 multi-lane
program uses **15 of the 16** buffer ids (5 singleton buffers + 5 per-lane roles,
so N=3 would need 20). Spending bit 7 would cap the design at 16 buffers and block
multi-lane beyond N=2. Making FILL build-wide costs far less, since only builds
that regenerate their programs ever set `WIDE_NTGT=1`.

**Notes:**

- Slots 0–2 are **short slots**: they carry only mode and buffer ID, with no #Targets field.
  They must only use mode `00` (UNUSED) or `01` (SOURCE). Using READ-WRITE or TARGET mode on
  a short slot will result in a zero #Targets count at completion, corrupting buffer state.
- Slots 3–5 are **long slots**: they carry mode, buffer ID, and #Targets, and support all four
  modes.
- Configuration for the accelerator must be written to the config register bank **before** the
  TASK instruction is fetched. See the AXI map section.
- The `#Targets` field of each output slot (READ-WRITE or TARGET) controls how many times that
  buffer can be consumed before it is freed. Set to 1 for a buffer read by exactly one
  downstream task.

**Typical SNN layer mapping:**

| Slot | Mode | Buffer |
|------|------|--------|
| 0 | SOURCE (`01`) | Input activations / spikes |
| 1 | SOURCE (`01`) | Weights |
| 2 | SOURCE (`01`) or UNUSED (`00`) | Bias currents |
| 3 | READ-WRITE (`10`) | Synaptic currents (in-place accumulator) |
| 4 | READ-WRITE (`10`) | Membrane potentials (in-place accumulator) |
| 5 | TARGET (`11`) | Output spikes |

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
 31          22   21           12   11    8   7     4   3   2    0
 ┌──────────────┬────────────────┬──────────┬──────────┬───┬──────┐
 │  (reserved)  │  skip address  │(reserved)│ buff ID  │mod│  op  │
 └──────────────┴────────────────┴──────────┴──────────┴───┴──────┘
```

| Bits    | Field | Notes |
|---------|-------|-------|
| [2:0]   | Opcode | `3'b011` |
| [3]     | Mode | `1` = finish on success; `0` = skip on success |
| [7:4]   | Check buffer ID | Selects the buffer whose result determines success or fail |
| [11:8]  | Reserved | Set to zero |
| [21:12] | Skip address | Word address to jump to in skip-on-success mode |
| [31:22] | Reserved | Set to zero |

The scheduler stalls on a CHECK until the nominated buffer is **full** (i.e. the task that
writes it has completed). Success is indicated by a zero result from the accelerator; fail by
a non-zero result.

**Behaviour:**

- **Finish on success (`mode=1`):** Stops instruction fetch and waits for all outstanding tasks
  to complete, then signals *finished* to the host (identical to STOP). The host must issue
  a CONTINUE command before execution can resume.
- **Skip on success (`mode=0`):** Jumps to the skip address, bypassing subsequent instructions
  (e.g. skipping layers in a conditional branch of a DyNN).
- **On fail (either mode):** Continues with the next instruction in sequence.

---

### NXT — `opcode 0b100`

Advances the time-window pointer for input data, output data, or both. Acts as a **completion
barrier**: the scheduler stalls until the task table is empty *and* every accelerator is idle
(all in-flight tasks have completed — not merely dispatched), then fires the selected pulse(s)
for exactly one cycle before continuing. Waiting for completion ensures the window pointer is
not advanced while an in-flight task is still reading/writing the current window.

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

Fills a buffer with a 32-bit constant value. Used to seed recurrent network state
(e.g. GRU hidden state h₀). Like TASK, this is a two-word instruction. The fill_unit
accelerator (acc_id = 4) performs the fill; the scheduler dispatches it as a normal
accelerator task so instruction fetch continues while the fill runs.

**Word 1:**

```
 31              13   12     9   8   7   6    3   2    0
 ┌────────────────────┬────────┬───┬───┬──────┬──────────┐
 │   Block size[18:0] │#Targets│clr│rsv│buf_id│   op    │
 └────────────────────┴────────┴───┴───┴──────┴──────────┘
```

| Bits    | Field | Notes |
|---------|-------|-------|
| [2:0]   | Opcode | `3'b101` |
| [6:3]   | Destination buffer | 4-bit buffer ID (0–15) |
| [7]     | Reserved | Set to zero |
| [8]     | Colour | Colour to apply to the destination buffer |
| [12:9]  | #Targets | **4-bit** consumer count, max 15 (widened with the TASK ntgt field) |
| [31:13] | Block size | Number of 32-bit words to load (19-bit, up to 512K) |

**Word 2:**

```
 31                                          0
 ┌────────────────────────────────────────────┐
 │              fill_value [31:0]             │
 └────────────────────────────────────────────┘
```

| Bits   | Field | Notes |
|--------|-------|-------|
| [31:0] | fill_value | 32-bit constant written to every word of the target buffer |

The base address of the target buffer is read by fill_unit from bba_mem using the
buffer ID in word 1. No alignment sentinel is needed: `task_w2_pending_r` in the
scheduler already guarantees this is word 2 of a FILL instruction.

---

### LOOP — `opcode 0b110`

Initialises a hardware loop counter. There are 8 independent counters (IDs 0–7); each stores
a 26-bit down-count and a restart address.

```
 31                         6   5    3   2    0
 ┌─────────────────────────────┬──────┬──────┐
 │         Count [25:0]        │ loop │  op  │
 │                             │  ID  │      │
 └─────────────────────────────┴──────┴──────┘
```

| Bits    | Field | Notes |
|---------|-------|-------|
| [2:0]   | Opcode | `3'b110` |
| [5:3]   | Loop ID | Selects one of 8 independent loop counters (0–7) |
| [31:6]  | Count | Number of times LOOPEND will jump back (total iterations = Count + 1) |

The LOOP instruction records `PC + 1` as the restart address for this loop ID, loads the
counter with `Count`, and continues fetching at `PC + 1` without stalling.

---

### LOOPEND — `opcode 0b111`

Closes a hardware loop. The loop ID must match the enclosing LOOP.

```
 31                         6   5    3   2    0
 ┌─────────────────────────────┬──────┬──────┐
 │          (reserved)         │ loop │  op  │
 │                             │  ID  │      │
 └─────────────────────────────┴──────┴──────┘
```

| Bits    | Field | Notes |
|---------|-------|-------|
| [2:0]   | Opcode | `3'b111` |
| [5:3]   | Loop ID | Must match the loop ID of the enclosing LOOP |
| [31:6]  | Reserved | Set to zero |

**Behaviour:**

LOOPEND is a **completion barrier**: it stalls until the scheduler table is empty *and* every
accelerator is idle (no task still in flight) before it acts. Only once the current iteration's
tasks have all **completed** does it:

- **Counter > 0:** Decrement the counter and jump to the saved restart address (the instruction
  immediately after the matching LOOP). The body executes one more time.
- **Counter = 0:** Fall through to the next instruction in sequence.

This barrier makes one loop iteration a global synchronisation boundary: the next iteration
(e.g. the next SNN timestep) cannot begin while the previous iteration's tasks are still
running. It closes cross-iteration write-after-read hazards on read-write state buffers
(membrane potentials, synaptic currents) that are reused every timestep — the next timestep's
producer can never race the current timestep's consumers, because the table is fully drained
first. (`NXT` uses the same completion-drain condition, so an `NXT` inside the loop body and the
`LOOPEND` at its end both enforce it.)

Loops may be nested up to 8 levels deep provided each level uses a distinct loop ID. There is
no hardware enforcement of proper nesting; mis-matched IDs produce undefined behaviour.

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
Reg ID is carried in bits `[24:20]`; bits `[19:0]` are ignored.  
Canonical addresses (bits `[19:0]` = 0):

| Reg ID `[24:20]` | Canonical address | Name | Action |
|------------------|-------------------|------|--------|
| `5'b00000` | `0xE000_0000` | LOAD_PC | Set program start address; write value is a word index |
| `5'b00001` | `0xE010_0000` | START | Begin execution at the loaded address. Flushes the fetch/decode front end **only** — buffer, table and accelerator state are NOT cleared (see SOFT_RESET) |
| `5'b00010` | `0xE020_0000` | CONTINUE | Resume after a finish-on-success halt |
| `5'b00011` | `0xE030_0000` | PAUSE | Halt instruction fetch; in-flight tasks complete normally |
| `5'b00100` | `0xE040_0000` | UNPAUSE | Resume after PAUSE |
| `5'b00101` | `0xE050_0000` | MARK_BUFF_FULL | Mark a buffer as pre-filled (see below) |
| `5'b00110` | `0xE060_0000` | SOFT_RESET | Clear buffer state, the task table and the front end — **not** memory. Makes the design re-runnable without a logic reset or an image reload |

**MARK_BUFF_FULL data word:**

| Bits   | Field |
|--------|-------|
| [3:0]  | Buffer ID (0–15) |
| [7:4]  | Usage count (#consumers), 4-bit (max 15) |
| [31:8] | Reserved (set to zero) |

Marks the specified buffer as full without going through the task dispatch path. Used to
pre-seed buffers that the host has loaded directly via the buffer data port before issuing
START. The buffer transitions to the full state with the given usage count; downstream tasks
may then consume it exactly that many times before it is freed. The buffer colour is left at
its reset default (0); write a separate MARK_BUFF_FULL with the same buffer ID if you need to
re-seed with a different colour in a later pass.

### Status Registers (read-only)

Address qualifier: bits `[31:30] = 2'b11`, bits `[29:26] = 4'b0001`.  
Reg ID is carried in bits `[25:20]`.  
Canonical addresses (bits `[19:0]` = 0):

| Reg ID `[25:20]` | Canonical address | Name | Contents |
|------------------|-------------------|------|----------|
| `6'b00_0000` | `0xC400_0000` | CURRENT_PC | Current fetch word address |
| `6'b00_0001` | `0xC410_0000` | ACTIVE | `[NUM_HW_ACCELERATORS-1:0]` = per-accelerator busy; `[30]` = paused on early exit; `[31]` = stopping (STOP seen, draining) |
| `6'b00_0010` | `0xC420_0000` | BUFF_FULL | Per-buffer full flags, one bit per buffer (bit N = buffer N) |
| `6'b00_0011` | `0xC430_0000` | BUFF_BUSY | Per-buffer busy flags (busy = task dispatched, not yet complete) |
| `6'b00_0100` | `0xC440_0000` | BUFF_COLOUR | Per-buffer colour flags |

### Program Memory (write-only) — addresses `0xD000_0000`–`0xD0FF_FFFF`

Address qualifier: bits `[31:24] = 8'hD0` (i.e. `(addr & 0xFF00_0000) == 0xD000_0000`).

The host loads the program by writing 32-bit words sequentially into this range. The word
index within program memory is derived from the byte address as:

```
word_index = (addr & 0x00FF_FFFF) >> 2
```

No alignment sentinel is required; simply write consecutive 32-bit-aligned addresses starting
at `0xD000_0000`. The scheduler fetches from the same memory, so all writes must complete
before START is issued.

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

1. **Buffer lifecycle (TARGET mode).** Assign a slot mode of TARGET (`11`) and set `#Targets = N`.
   The buffer becomes busy on dispatch and full on completion. Each of the N downstream TASKs
   that lists it as a SOURCE decrements the counter when that consumer **completes**; after the
   Nth consumer completes, the buffer is automatically freed. (A consumer
   holds the buffer until it finishes, so a later writer cannot overwrite data still being read.)

2. **Read-Write buffer lifecycle.** Assign a slot mode of READ-WRITE (`10`) with `#Targets = N`.
   On dispatch the buffer transitions from full to busy (old data consumed); on completion it
   becomes full again with the updated data. The `#Targets` count is reset to N at completion,
   so downstream tasks can consume the new data exactly N times before the buffer is freed.

3. **Colour usage.** Colour is a 1-bit field. Use it to distinguish two generations of data in
   the same buffer. Alternate 0 and 1 in successive passes through a recurrent loop; the
   consumer tasks will only fire when the colour matches, preventing a slow pass from consuming
   data produced for a later pass.

4. **Configuration must precede TASK.** The config register bank has no ordering guarantee
   relative to in-flight tasks. Write all accelerator configuration via the AXI interface
   before the program starts, or before the TASK that needs it is reached.

5. **FILL seeding.** Use FILL to pre-populate a recurrent buffer (e.g. initial hidden state h₀)
   before the first iteration. Its `#Targets` and colour work identically to a TASK's
   destination buffer, so downstream TASKs can treat it like any other full buffer.

6. **Host pre-seeding with MARK_BUFF_FULL.** If the host has already placed data directly into
   a buffer via the buffer data port (addresses `0x0000_0000`–`0x01FF_FFFF`), issue
   MARK_BUFF_FULL (`0xE050_0000`) with the buffer ID and consumer count before START. The
   scheduler will then treat that buffer as full when the program runs, allowing the first
   wave of TASKs to dispatch immediately. This differs from FILL, which performs a DMA
   transfer at runtime; MARK_BUFF_FULL simply updates the scheduler's state machine.

7. **NXT as a synchronisation barrier.** NXT stalls the fetch pipeline until the task table
   drains. Use it at the end of each timestep in a temporal SNN to ensure all layers have
   completed before advancing the input window.

8. **STOP and restart.** After STOP, the host issues SOFT_RESET, then re-seeds any
   host-filled buffers with MARK_BUFF_FULL, then LOAD_PC + START.

   **START ALONE IS NOT ENOUGH, and this document previously said otherwise.** START
   flushes the fetch/decode front end but leaves buffer, table and accelerator state
   intact. So a bare re-START does re-fetch and re-create the task entry — the PC visibly
   moves — but that entry can never become ready, because its target buffers are still
   FULL from the previous run and a TARGET slot requires the buffer FREE. The machine sits
   with one valid-but-never-ready entry and makes no progress. (Observed on silicon during
   the KV260 bring-up, then reproduced off-board; the incorrect claim here is part of why
   it was surprising.)

   SOFT_RESET clears buffer state, the task table, the loop counters and the front end
   while leaving program, config and weight memories untouched — so a long campaign loads
   its images once per session rather than once per run. Issue it only when no accelerator
   is busy (after STOP, or PAUSE until `acc_busy == 0`); clearing the trackers mid-task
   would let a later completion update buffers using slot state that no longer exists.
   A wedged machine reports `acc_busy == 0`, so using it for recovery is safe.

9. **Hardware loops.** Use LOOP / LOOPEND to repeat a block of instructions without the
   overhead of a JUMP. A LOOP with `Count = N` causes the body to execute `N + 1` times in
   total (the counter is decremented on each backward edge; the body falls through when the
   counter reaches zero). Each of the 8 loop counters is independent; assign a distinct loop
   ID to each nesting level. A typical SNN timestep loop places LOOP at the start of the
   timestep body, the NXT that advances the time window at the end of the body, and LOOPEND
   immediately after that NXT — so the window advances on every iteration (NXT is inside the
   loop, not after it). LOOPEND itself is a completion barrier (waits for all dispatched tasks
   to finish), so each timestep is fully drained before the next begins.
