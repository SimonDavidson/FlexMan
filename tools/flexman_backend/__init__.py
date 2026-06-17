# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Simon Davidson, University of Manchester
# =============================================================================
# flexman_backend — unified, hardware-defined deployment back-end for FlexMan
#
# Author: Simon Davidson & Claude
# Created: 2026-06-08
# Last modified: 2026-06-17
#
# Pure-Python (no torch). Shared by every deployment front-end. Tracks the RTL
# it is versioned alongside; see
# tools/README.md and tools/tests/test_regmap_vs_rtl.py.
# =============================================================================
from __future__ import annotations

from . import isa, regmap, quant, emit, mempool, lut, cfgmem, emulate

__all__ = ["isa", "regmap", "quant", "emit", "mempool", "lut", "cfgmem", "emulate"]
