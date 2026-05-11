# FlexMan

FlexMan is a flexible hardware accelerator management system for ANN/SNN neural networks, implemented in Verilog RTL.

The `scheduler` reads a program of instructions from external memory and dispatches layer tasks to hardware accelerators, managing buffer dependencies so that each task only runs when its inputs are ready and its output buffer is free.

## Repository Layout

```
FlexMan/
  scheduler/        task scheduler: instruction fetch/decode, buffer state, dispatch
  snnAcc/           SNN accelerator: spike processing + neuron processing pipeline
  Hadamard/         Hadamard transform accelerator
  annAcc/           ANN accelerator
  ipSnnAcc/         IP-packaged SNN accelerator variant
  config_manager/   configuration register bank
  shared/           shared constants and interfaces (in progress)
  top_module/       top-level integration wrapper (TBD)
```


The scheduler dispatches `TASK` instructions to accelerators via a system bus interface. Accelerators signal completion back via the same bus. The scheduler pre-loads configuration registers before each `TASK` dispatch, so accelerators begin execution fully configured.

```
external memory
      │  (AXI)
      ▼
  scheduler ──TASK──► snnAcc
      │       └────► Hadamard
      │       └────► annAcc
      └── buffer dependency tracking (blocks dispatch until inputs ready)
```

## Simulation

All modules use **Xcelium** (`xrun -sv`). Run simulations from the relevant sub-directory.

**Scheduler:**
```bash
cd scheduler
bash mySch.bsh          # full scheduler integration
bash myTable.bsh        # scheduler table module only
bash myBuffState.bsh    # buffer/accelerator state tracking only
```

**SNN Accelerator:**
```bash
cd snnAcc
bash tbAccSNN.bsh
```

## Signal Naming Conventions

| Suffix | Meaning |
|--------|---------|
| `_i`   | input port |
| `_o`   | output port |
| `_r`   | registered (config) value |
| `mem_wait_i` / `mem_wait_o` | back-pressure on memory interfaces |

## License

MIT — see [LICENSE](LICENSE).

## Contact

Simon Davidson, University of Manchester — simon.davidson@manchester.ac.uk
