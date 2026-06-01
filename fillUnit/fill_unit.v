// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
`timescale 10ps/1ps
`include "../shared/constants.v"

// fill_unit: constant-value block-fill accelerator for the FlexMan scheduler.
//
// Triggered by the scheduler when a FILL instruction is dispatched (acc_id =
// FILL_ACC_ID = NUM_HW_ACCELERATORS-1).  Writes fill_value_i to every word
// in the nominated buffer, starting at the base address read from the BBA
// memory and for fill_block_size_i words.  The target buffer remains busy in
// the scheduler until acc_finished_o pulses.
//
// Memory-select table (mem_sel_table):
//   One entry per buffer, initialised via AXI writes at system start-up.
//   Each entry is a NUM_MEM_TYPES-bit one-hot value indicating which of the 25
//   external memory buses this buffer lives in.  The AXI address encodes the
//   buffer ID: addr[BUFF_INDX_SZ+1:2] selects the row.
//
// Memory index assignments (matches flexman.v arbitration mux ordering):
//   0  s0_weight   7  s1_weight  14  a0_weight  21  hd_src_a
//   1  s0_act      8  s1_act     15  a0_act     22  hd_src_b
//   2  s0_syn_curr 9  s1_syn_curr 16 a0_syn_curr 23 hd_src_z
//   3  s0_bias_curr 10 s1_bias_curr 17 a0_bias_curr 24 hd_src_r
//   4  s0_thresh   11 s1_thresh  18  a0_thresh
//   5  s0_pot      12 s1_pot     19  a0_pot
//   6  s0_spike    13 s1_spike   20  a0_spike
//
// BBA access: a dedicated second read port on bba_mem is provided to
// fill_unit.  The host must write matching base addresses to both
// config_manager's BBA table and the bba_mem physical memory; the fill_unit
// reads the same bba_mem to obtain the start word-address for its write burst.

module fill_unit #(
    // AXI address region for the mem_sel table
    parameter [31:0] FU_TABLE_ADDR      = 32'hC000_0000,
    parameter [31:0] FU_TABLE_ADDR_MASK = 32'hFF00_0000,

    parameter NUM_BUFFERS   = 16,
    parameter BUFF_INDX_SZ  = 4,
    parameter ADDR_SIZE     = `ADDR_SIZE,   // 30
    parameter DATA_SZ       = 32,
    parameter NUM_MEM_TYPES = 26
) (
    input wire clk,
    input wire reset,

    // ── AXI slave (mem_sel table initialisation) ─────────────────────────────
    input  wire        sys_req_i,
    output wire        sys_ack_o,
    input  wire [31:0] sys_addr_i,
    input  wire [31:0] sys_data_i,

    // ── Scheduler dispatch interface ─────────────────────────────────────────
    input  wire                     start_new_block_i,  // one-cycle dispatch strobe
    output reg                      acc_busy_o,          // held while fill in progress
    output reg                      acc_finished_o,      // one-cycle completion pulse

    // ── Dispatch parameters (valid at start_new_block_i) ─────────────────────
    input  wire [BUFF_INDX_SZ-1:0]  buff_id_i,          // target buffer (slot 3 id)
    input  wire [DATA_SZ-1:0]        fill_value_i,       // constant to write
    input  wire [19:0]               fill_block_size_i,  // number of 32-bit words

    // ── BBA memory read port (second port; 1-cycle latency) ──────────────────
    output reg                       fu_bba_rd_o,
    input  wire                      fu_bba_wait_i,
    output wire [31:0]               fu_bba_addr_o,
    input  wire [31:0]               fu_bba_data_i,

    // ── snnAcc0 memory write ports ───────────────────────────────────────────
    output wire                  s0_weight_wr_o,
    output wire [ADDR_SIZE-1:0]  s0_weight_addr_o,
    output wire [DATA_SZ-1:0]    s0_weight_data_o,
    input  wire                  s0_weight_wait_i,

    output wire                  s0_bias_curr_wr_o,
    output wire [ADDR_SIZE-1:0]  s0_bias_curr_addr_o,
    output wire [DATA_SZ-1:0]    s0_bias_curr_data_o,
    input  wire                  s0_bias_curr_wait_i,

    output wire                  s0_thresh_wr_o,
    output wire [ADDR_SIZE-1:0]  s0_thresh_addr_o,
    output wire [DATA_SZ-1:0]    s0_thresh_data_o,
    input  wire                  s0_thresh_wait_i,

    output wire                  s0_pot_wr_o,
    output wire [ADDR_SIZE-1:0]  s0_pot_addr_o,
    output wire [DATA_SZ-1:0]    s0_pot_data_o,
    input  wire                  s0_pot_wait_i,

    // ── snnAcc1 memory write ports ───────────────────────────────────────────
    output wire                  s1_weight_wr_o,
    output wire [ADDR_SIZE-1:0]  s1_weight_addr_o,
    output wire [DATA_SZ-1:0]    s1_weight_data_o,
    input  wire                  s1_weight_wait_i,

    output wire                  s1_bias_curr_wr_o,
    output wire [ADDR_SIZE-1:0]  s1_bias_curr_addr_o,
    output wire [DATA_SZ-1:0]    s1_bias_curr_data_o,
    input  wire                  s1_bias_curr_wait_i,

    output wire                  s1_thresh_wr_o,
    output wire [ADDR_SIZE-1:0]  s1_thresh_addr_o,
    output wire [DATA_SZ-1:0]    s1_thresh_data_o,
    input  wire                  s1_thresh_wait_i,

    output wire                  s1_pot_wr_o,
    output wire [ADDR_SIZE-1:0]  s1_pot_addr_o,
    output wire [DATA_SZ-1:0]    s1_pot_data_o,
    input  wire                  s1_pot_wait_i,

    // ── annAcc memory write ports ─────────────────────────────────────────────
    output wire                  a0_weight_wr_o,
    output wire [ADDR_SIZE-1:0]  a0_weight_addr_o,
    output wire [DATA_SZ-1:0]    a0_weight_data_o,
    input  wire                  a0_weight_wait_i,

    output wire                  a0_bias_curr_wr_o,
    output wire [ADDR_SIZE-1:0]  a0_bias_curr_addr_o,
    output wire [DATA_SZ-1:0]    a0_bias_curr_data_o,
    input  wire                  a0_bias_curr_wait_i,

    output wire                  a0_thresh_wr_o,
    output wire [ADDR_SIZE-1:0]  a0_thresh_addr_o,
    output wire [DATA_SZ-1:0]    a0_thresh_data_o,
    input  wire                  a0_thresh_wait_i,

    output wire                  a0_pot_wr_o,
    output wire [ADDR_SIZE-1:0]  a0_pot_addr_o,
    output wire [DATA_SZ-1:0]    a0_pot_data_o,
    input  wire                  a0_pot_wait_i,

    // ── Shared activation/spike/syn_curr pool — act/spike/syn_curr (all accs)
    //    and the Hadamard buffers now fill through this single port; the top
    //    routes the address LSBs to pick the interleaved bank. ────────────────
    output wire                  shared_data_wr_o,
    output wire [ADDR_SIZE-1:0]  shared_data_addr_o,
    output wire [DATA_SZ-1:0]    shared_data_data_o,
    input  wire                  shared_data_wait_i
);

// ─── Memory index localparams ─────────────────────────────────────────────────
// One-hot bit positions in the mem_sel table.  act / spike / syn_curr (all
// accelerators) and the Hadamard buffers now fill via IDX_SHARED_DATA, so their
// former per-acc indices are removed.  The remaining indices KEEP their original
// values so the host's mem_sel encoding is unchanged; the vacated positions
// (1,2,6,8,9,13,15,16,20-24) are simply reserved/unused.
localparam IDX_S0_WEIGHT    =  0;
localparam IDX_S0_BIAS_CURR =  3;
localparam IDX_S0_THRESH    =  4;
localparam IDX_S0_POT       =  5;
localparam IDX_S1_WEIGHT    =  7;
localparam IDX_S1_BIAS_CURR = 10;
localparam IDX_S1_THRESH    = 11;
localparam IDX_S1_POT       = 12;
localparam IDX_A0_WEIGHT    = 14;
localparam IDX_A0_BIAS_CURR = 17;
localparam IDX_A0_THRESH    = 18;
localparam IDX_A0_POT       = 19;
// Shared activation/spike/syn_curr + Hadamard pool (top routes the address LSBs
// to pick the interleaved bank).
localparam IDX_SHARED_DATA  = 25;

// ─── FSM states ───────────────────────────────────────────────────────────────
localparam ST_IDLE     = 2'd0;
localparam ST_BBA_WAIT = 2'd1;   // waiting for BBA read data (1-cycle latency)
localparam ST_FILLING  = 2'd2;
localparam ST_DONE     = 2'd3;

// ─── Internal registers ───────────────────────────────────────────────────────
reg [1:0]                     state_r;
reg [BUFF_INDX_SZ-1:0]        buff_id_r;
reg [DATA_SZ-1:0]              fill_value_r;
reg [19:0]                    block_size_r;
reg [19:0]                    cnt_r;
reg [ADDR_SIZE-1:0]           curr_addr_r;
reg [NUM_MEM_TYPES-1:0]       mem_sel_r;

// mem_sel table: one word per buffer, AXI-initialised
reg [NUM_MEM_TYPES-1:0] mem_sel_table [0:NUM_BUFFERS-1];

// ─── AXI slave — mem_sel table writes ─────────────────────────────────────────
wire table_addr_match = ((sys_addr_i & FU_TABLE_ADDR_MASK) ==
                          (FU_TABLE_ADDR  & FU_TABLE_ADDR_MASK));
assign sys_ack_o = sys_req_i & table_addr_match;

integer ti;
always @(posedge clk) begin
    if (sys_req_i & table_addr_match) begin
        // Buffer ID from word-indexed address bits above byte-lane bits
        ti = (sys_addr_i & ~FU_TABLE_ADDR_MASK) >> 2;
        if (ti < NUM_BUFFERS)
            mem_sel_table[ti] <= sys_data_i[NUM_MEM_TYPES-1:0];
    end
end

// ─── BBA address (always driven from latched buffer ID) ───────────────────────
assign fu_bba_addr_o = {{(32-BUFF_INDX_SZ){1'b0}}, buff_id_r};

// ─── Wait mux: one-hot select from selected memory's wait signal ──────────────
wire selected_wait = (mem_sel_r[IDX_S0_WEIGHT]    & s0_weight_wait_i)   |
                     (mem_sel_r[IDX_S0_BIAS_CURR]  & s0_bias_curr_wait_i)|
                     (mem_sel_r[IDX_S0_THRESH]     & s0_thresh_wait_i)   |
                     (mem_sel_r[IDX_S0_POT]        & s0_pot_wait_i)      |
                     (mem_sel_r[IDX_S1_WEIGHT]     & s1_weight_wait_i)   |
                     (mem_sel_r[IDX_S1_BIAS_CURR]  & s1_bias_curr_wait_i)|
                     (mem_sel_r[IDX_S1_THRESH]     & s1_thresh_wait_i)   |
                     (mem_sel_r[IDX_S1_POT]        & s1_pot_wait_i)      |
                     (mem_sel_r[IDX_A0_WEIGHT]     & a0_weight_wait_i)   |
                     (mem_sel_r[IDX_A0_BIAS_CURR]  & a0_bias_curr_wait_i)|
                     (mem_sel_r[IDX_A0_THRESH]     & a0_thresh_wait_i)   |
                     (mem_sel_r[IDX_A0_POT]        & a0_pot_wait_i)      |
                     (mem_sel_r[IDX_SHARED_DATA]   & shared_data_wait_i);

// ─── FSM ──────────────────────────────────────────────────────────────────────
always @(posedge clk) begin
    if (reset) begin
        state_r        <= ST_IDLE;
        acc_busy_o     <= 1'b0;
        acc_finished_o <= 1'b0;
        fu_bba_rd_o    <= 1'b0;
        buff_id_r      <= 'b0;
        fill_value_r   <= 'b0;
        block_size_r   <= 'b0;
        cnt_r          <= 'b0;
        curr_addr_r    <= 'b0;
        mem_sel_r      <= 'b0;
    end else begin
        acc_finished_o <= 1'b0;  // default: no pulse

        case (state_r)
            ST_IDLE: begin
                if (start_new_block_i) begin
                    buff_id_r    <= buff_id_i;
                    fill_value_r <= fill_value_i;
                    block_size_r <= fill_block_size_i;
                    mem_sel_r    <= mem_sel_table[buff_id_i];
                    acc_busy_o   <= 1'b1;
                    fu_bba_rd_o  <= 1'b1;
                    state_r      <= ST_BBA_WAIT;
                end
            end

            ST_BBA_WAIT: begin
                if (!fu_bba_wait_i) begin
                    // BBA read accepted; data arrives this cycle (1-cycle SRAM)
                    fu_bba_rd_o <= 1'b0;
                    curr_addr_r <= fu_bba_data_i[ADDR_SIZE-1:0];
                    cnt_r       <= 'b0;
                    state_r     <= ST_FILLING;
                end
            end

            ST_FILLING: begin
                if (!selected_wait) begin
                    curr_addr_r <= curr_addr_r + 1'b1;
                    cnt_r       <= cnt_r + 1'b1;
                    if (cnt_r == block_size_r - 1'b1) begin
                        state_r <= ST_DONE;
                    end
                end
            end

            ST_DONE: begin
                acc_finished_o <= 1'b1;
                acc_busy_o     <= 1'b0;
                state_r        <= ST_IDLE;
            end

            default: state_r <= ST_IDLE;
        endcase
    end
end

// ─── Write strobe and bus drivers ─────────────────────────────────────────────
wire do_write = (state_r == ST_FILLING) & ~selected_wait;

// All address and data outputs are shared (same value regardless of selection)
wire [ADDR_SIZE-1:0] wr_addr = curr_addr_r;
wire [DATA_SZ-1:0]   wr_data = fill_value_r;

assign s0_weight_wr_o    = do_write & mem_sel_r[IDX_S0_WEIGHT];
assign s0_weight_addr_o  = wr_addr;
assign s0_weight_data_o  = wr_data;

assign s0_bias_curr_wr_o  = do_write & mem_sel_r[IDX_S0_BIAS_CURR];
assign s0_bias_curr_addr_o = wr_addr;
assign s0_bias_curr_data_o = wr_data;

assign s0_thresh_wr_o    = do_write & mem_sel_r[IDX_S0_THRESH];
assign s0_thresh_addr_o  = wr_addr;
assign s0_thresh_data_o  = wr_data;

assign s0_pot_wr_o       = do_write & mem_sel_r[IDX_S0_POT];
assign s0_pot_addr_o     = wr_addr;
assign s0_pot_data_o     = wr_data;

assign s1_weight_wr_o    = do_write & mem_sel_r[IDX_S1_WEIGHT];
assign s1_weight_addr_o  = wr_addr;
assign s1_weight_data_o  = wr_data;

assign s1_bias_curr_wr_o  = do_write & mem_sel_r[IDX_S1_BIAS_CURR];
assign s1_bias_curr_addr_o = wr_addr;
assign s1_bias_curr_data_o = wr_data;

assign s1_thresh_wr_o    = do_write & mem_sel_r[IDX_S1_THRESH];
assign s1_thresh_addr_o  = wr_addr;
assign s1_thresh_data_o  = wr_data;

assign s1_pot_wr_o       = do_write & mem_sel_r[IDX_S1_POT];
assign s1_pot_addr_o     = wr_addr;
assign s1_pot_data_o     = wr_data;

assign a0_weight_wr_o    = do_write & mem_sel_r[IDX_A0_WEIGHT];
assign a0_weight_addr_o  = wr_addr;
assign a0_weight_data_o  = wr_data;

assign a0_bias_curr_wr_o  = do_write & mem_sel_r[IDX_A0_BIAS_CURR];
assign a0_bias_curr_addr_o = wr_addr;
assign a0_bias_curr_data_o = wr_data;

assign a0_thresh_wr_o    = do_write & mem_sel_r[IDX_A0_THRESH];
assign a0_thresh_addr_o  = wr_addr;
assign a0_thresh_data_o  = wr_data;

assign a0_pot_wr_o       = do_write & mem_sel_r[IDX_A0_POT];
assign a0_pot_addr_o     = wr_addr;
assign a0_pot_data_o     = wr_data;

assign shared_data_wr_o   = do_write & mem_sel_r[IDX_SHARED_DATA];
assign shared_data_addr_o = wr_addr;
assign shared_data_data_o = wr_data;

endmodule
