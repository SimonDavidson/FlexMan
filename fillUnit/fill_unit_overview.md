# fill_unit — Overview

`fill_unit` is a FlexMan accelerator (acc_id = 4) that writes a 32-bit constant value to every
word of a scheduler-managed buffer. It is dispatched by the scheduler when a `FILL` instruction
is decoded and runs in parallel with computation accelerators — the fetch pipeline keeps moving
while the fill is in progress.

## How It Fits Into the System

The scheduler decodes a two-word `FILL` instruction. Word 1 carries the target buffer ID,
colour, number of targets, and block size (in 32-bit words). Word 2 carries the 32-bit fill
constant. The scheduler pushes a FILL entry into the dispatch table (slot 3 = TARGET mode, so
the buffer goes busy immediately) and signals `start_new_block_i` to `fill_unit` while setting
`fill_value_o` and `fill_block_size_o`. Any subsequent task that reads the filled buffer will
stall in the scheduler table until `fill_unit` asserts `acc_finished_o`, which clears the
buffer's busy flag and marks it full.

## What fill_unit Does

1. **On dispatch** (`start_new_block_i`): latches the buffer ID (extracted from the scheduler
   entry), the fill value, and the block size; looks up the per-buffer memory-select entry from
   its internal AXI-initialised table; issues a read to the external buffer-base-address store
   (`fu_bba_*` ports) to find where the buffer lives in memory.

2. **BBA wait**: holds in `ST_BBA_WAIT` until the base-address response arrives (one or more
   cycles depending on memory latency).

3. **Fill loop** (`ST_FILLING`): each cycle that the selected memory bus is free, asserts the
   write strobe, drives the current word address, and drives the fill constant as the write
   data. The word address increments by one each accepted cycle. Fill pauses automatically if
   the bus signals back-pressure (`*_wait_i = 1`).

4. **Done** (`ST_DONE`): after the last word is written, pulses `acc_finished_o` for one cycle
   and drops `acc_busy_o`. The scheduler sees the pulse, marks the buffer full, and enables any
   waiting consumer tasks to dispatch.

## Memory Selection

Before any `FILL` instruction runs, software writes a one-hot 25-bit enable word into
`fill_unit`'s AXI-addressable `mem_sel_table` (one entry per buffer). The set bit selects
which of the 25 memory buses (weight, act, syn_curr, pot, spike, bias_curr, thresh for each of
the three SNN/ANN accelerator slots, plus Hadamard src_a/b/z/r) the fill targets.

## Arbitration

For the 10 buses that accelerators also write (syn_curr × 3, pot × 3, spike × 3, hd_src_r × 1)
the accelerator has priority: `flexman.v` multiplexes the bus so the accelerator always wins,
and `fill_unit` sees `*_wait_i` asserted until the bus is free. For the 15 buses that
accelerators only read (weight, act, bias_curr, thresh, hd_src_a/b/z), `fill_unit` has
dedicated write ports connected directly to the top-level memory interface with no arbitration
required (dual-port SRAM model).

## Key Parameters

| Parameter | Value | Meaning |
|-----------|-------|---------|
| `FILL_ACC_ID` | 4 | Accelerator slot index in the scheduler |
| `NUM_BUFFERS` | 16 | Number of buffer table entries |
| `FILL_MEM_COUNT` | 25 | Number of memory buses that can be targeted |
| `ADDR_SIZE` | 30 | Word-address width |
