<!--
  Author: Simon Davidson & Claude
  Created: 2026-06-08
  Last modified: 2026-06-08
-->
# FlexMan deployment toolchain

A **unified back-end** for compiling neural-network deployments onto the FlexMan
accelerator, plus **distinct per-application front-ends**.

```
FlexMan/tools/
  flexman_backend/      ← shared, HARDWARE-DEFINED, pure-Python (no torch)
    isa.py        scheduler ISA assembler (TASK/LOOP/NXT/FILL/...)
    regmap.py     AXI register-offset maps (snnAcc/annAcc/Hadamard) + addr helpers
    quant.py      q032/log2 + column-major weight packing, bias pack, bias-fold
    emit.py       .hex images + axi_write(...) .vh include emitter
    mempool.py    SharedPool allocator + MemRegion
    lut.py        sigmoid/tanh activation-table generation
  tests/          unit tests + test_regmap_vs_rtl.py (anti-drift: parses the .v)
```

## Design rule: share what tracks the hardware; separate what tracks the network

The back-end mirrors the RTL (register decoders, ISA bit-layouts, packing
conventions) and is **versioned in this repo with that RTL**, so an ISA or
register change updates one place and every front-end picks it up.
`tests/test_regmap_vs_rtl.py` parses the `8'hXX:` cases out of the Verilog and
asserts the maps match — the check a separate-repo toolchain cannot do.

Application-specific logic (which layer maps to which accelerator, the
quantization source, the per-frame schedule) lives in each front-end, NOT here:

- **Bosch siren** front-end: in the Bosch repo (`~/work/Bosch/`).
- **JABRA NsNet2** front-end: in the JABRA workspace (`~/work/jabra/frontends/nsnet2/`).

## The back-end is torch-free

No `import torch` anywhere in `flexman_backend/`. It operates on plain Python
ints / nested int lists, so `tools/tests/` run under a bare interpreter. Front-ends
load checkpoints and quantise tensors, then pass `tensor.tolist()` to the packers.

## Install (editable) into a front-end's venv

```bash
<venv>/bin/pip install -e /home/mbasssd3/work/FlexMan/tools
```

## Run the tests (no torch needed)

```bash
python3 -m pytest /home/mbasssd3/work/FlexMan/tools/tests -q
# or, without pytest:
python3 /home/mbasssd3/work/FlexMan/tools/tests/run_all.py
```
