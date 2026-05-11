// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
`define OP_LEN     16

`define PROG_BITS  10
`define PROG_DATA_BITS 32

`define ROW_BITS   10
`define COL_BITS   10
`define CFG_BITS    8
`define WTA_BITS   16		// Weight memory address
`define WTD_BITS   32		// Weight memory data
`define WGT_BITS   32		// Weight operand (pipe) width
`define SIZ_BITS    6		// Index for memory data bits
`define ACT_BITS   32		// Activation datum size
`define PIN_BITS   10		// Output index size
`define POT_BITS   32		// Internal potential data size
`define PRODUCT_BITS 64         // Bits produced by multiplier

`define MRPT_BITS  16		// CNN subsequence repeat counter
`define BRST_BITS  10		// Memory burst size

`define ADDR_SIZE  30

`define BINPOINT_SZ 4
`define POT_OUT_SZ_SZ 3
`define GUARDBITS_SZ 3
`define ACC_SHIFT_SZ 5
`define TGT_ACC_SZ   1
`define SCH_ENTRY_SZ 32

`define DATAWORD_SZ 32
