#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Author: Simon Davidson & Claude
# Created: 2026-06-08
# Last modified: 2026-06-08
#
# Minimal test runner — runs every test_* function in this directory under a bare
# interpreter (no pytest, no torch needed). Exit non-zero on any failure.
import importlib
import sys
import traceback
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))   # make flexman_backend importable in-tree


def main() -> int:
    modules = sorted(p.stem for p in HERE.glob("test_*.py"))
    passed = failed = 0
    for modname in modules:
        mod = importlib.import_module(modname)
        for fname in sorted(n for n in dir(mod) if n.startswith("test_")):
            fn = getattr(mod, fname)
            if not callable(fn):
                continue
            try:
                fn()
                passed += 1
                print(f"  PASS  {modname}.{fname}")
            except Exception:
                failed += 1
                print(f"  FAIL  {modname}.{fname}")
                traceback.print_exc()
    print(f"\n{passed} passed, {failed} failed")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
