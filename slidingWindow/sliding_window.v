// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
`timescale 10ps/1ps
`include "../shared/constants.v"

// ───────────────────────────────────────────────────────────────────────────
// sliding_window — timestep windowing interposer (test harness model)
//
// Models the external, application-owned input/output buffer that sits between
// FlexMan and a flat data memory. It does NOT contain the memory; it interposes
// on a single downstream memory port and offsets FlexMan's addresses by a base
// pointer that advances one block per timestep.
//
// The SAME module is instantiated twice in the test wrapper: one instance on the
// input stream (NXT input pulse) and one on the output spikes (NXT output pulse).
// It is symmetric read/write — the input instance is driven mostly by reads, the
// output instance mostly by writes, but both support either.
//
// Two upstream clients share the single downstream memory port:
//
//   1. FlexMan side (WINDOWED) — fm_* port, standard FlexMan memory handshake.
//      FlexMan issues addresses 0 .. block_length-1 within the current timestep;
//      this module forwards them at  base_ptr_r + fm_addr_i.
//
//   2. Host AXI side (ABSOLUTE) — sw_sys_* slave. The host statically loads test
//      data and reads back output spikes addressing the whole flat memory
//      directly (no base offset). Also writes the two config registers.
//
// Config registers (word offsets within SW_REG region, host AXI):
//      offset 0 → base_ptr_r      (word address of current timestep block)
//      offset 1 → block_length_r  (block size in words)
// base_ptr_r is loaded by the host initially, advances by block_length_r on each
// nxt_pulse_i, and clears to 0 on reset.
//
// Arbitration onto the single downstream port is STRICT priority with FlexMan
// highest, so compute never stalls; the host is back-pressured (no ack/rvalid)
// until FlexMan is idle, then retries. In a test harness the host loads before
// the run and reads after, so contention is incidental. (Mirrors the strict-
// priority + registered read-return scheme of shared/shared_pool.v.)
//
// Reads are 1-cycle synchronous (downstream memory latency): the owner of the
// accepted read is registered and the returned data is routed back to FlexMan or
// the host one cycle later. Only one grant per cycle ⇒ at most one read in flight.
// ───────────────────────────────────────────────────────────────────────────

module sliding_window #(
    parameter [31:0] SW_REG_ADDR  = 32'hE000_0000,  // base_ptr/block_len register region
    parameter [31:0] SW_REG_MASK  = 32'hFFFF_0000,
    parameter [31:0] SW_MEM_ADDR  = 32'hE001_0000,  // windowed data-memory region (host view)
    parameter [31:0] SW_MEM_MASK  = 32'hFFFF_0000,
    parameter        ADDR_W       = `ADDR_SIZE,     // 30
    parameter        DATA_W       = 32
)(
    input  wire              clk,
    input  wire              reset,

    // Timestep advance (wire to nxt_input_pulse_o or nxt_output_pulse_o)
    input  wire              nxt_pulse_i,

    // ── Host AXI slave (config registers + absolute memory load/readback) ──
    input  wire              sw_sys_req_i,
    input  wire              sw_sys_we_i,      // 1 = write, 0 = read
    input  wire [31:0]       sw_sys_addr_i,
    input  wire [DATA_W-1:0] sw_sys_wdata_i,
    output wire              sw_sys_ack_o,
    output wire [DATA_W-1:0] sw_sys_rdata_o,
    output wire              sw_sys_rvalid_o,  // 1-cycle-later read-data strobe

    // ── FlexMan-side slave (windowed; standard FlexMan mem handshake) ──
    input  wire              fm_rd_i,
    input  wire              fm_wr_i,
    input  wire [ADDR_W-1:0] fm_addr_i,
    input  wire [DATA_W-1:0] fm_wdata_i,
    output wire [DATA_W-1:0] fm_rdata_o,
    output wire              fm_wait_o,

    // ── Downstream single memory port (to external test RAM) ──
    output wire              mem_rd_o,
    output wire              mem_wr_o,
    output wire [ADDR_W-1:0] mem_addr_o,
    output wire [DATA_W-1:0] mem_wdata_o,
    input  wire [DATA_W-1:0] mem_rdata_i,
    input  wire              mem_wait_i
);

    // ─── AXI address decode (masked compare, like config_manager.v) ──────────
    wire        reg_match = (sw_sys_addr_i & SW_REG_MASK) == (SW_REG_ADDR & SW_REG_MASK);
    wire        mem_match = (sw_sys_addr_i & SW_MEM_MASK) == (SW_MEM_ADDR & SW_MEM_MASK);

    wire [31:0] reg_off       = (sw_sys_addr_i & ~SW_REG_MASK) >> 2;  // 0=base, 1=blocklen
    wire [31:0] host_mem_word = (sw_sys_addr_i & ~SW_MEM_MASK) >> 2;  // absolute word addr

    wire        reg_access = sw_sys_req_i & reg_match;               // host reg read or write
    wire        reg_wr     = reg_access   & sw_sys_we_i;
    wire        reg_rd     = reg_access   & ~sw_sys_we_i;

    wire        host_mem_req = sw_sys_req_i & mem_match;
    wire        host_mem_rd  = host_mem_req & ~sw_sys_we_i;
    wire        host_mem_wr  = host_mem_req &  sw_sys_we_i;

    // ─── Config registers ────────────────────────────────────────────────────
    reg [ADDR_W-1:0] base_ptr_r;
    reg [ADDR_W-1:0] block_length_r;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            base_ptr_r     <= {ADDR_W{1'b0}};
            block_length_r <= {ADDR_W{1'b0}};
        end else begin
            // Host write wins over NXT advance in the (impossible) same cycle:
            // NXT only fires when FlexMan and host are idle.
            if (reg_wr & (reg_off == 32'd0))
                base_ptr_r <= sw_sys_wdata_i[ADDR_W-1:0];
            else if (nxt_pulse_i)
                base_ptr_r <= base_ptr_r + block_length_r;

            if (reg_wr & (reg_off == 32'd1))
                block_length_r <= sw_sys_wdata_i[ADDR_W-1:0];
        end
    end

    // ─── FlexMan request (windowed address) ──────────────────────────────────
    wire             fm_req       = fm_rd_i | fm_wr_i;
    wire [ADDR_W-1:0] fm_phys_addr = fm_addr_i + base_ptr_r;

    // ─── Strict-priority arbitration onto the single downstream port ─────────
    wire fm_grant   = fm_req;                  // FlexMan always wins
    wire host_grant = host_mem_req & ~fm_req;  // host only when FlexMan idle

    assign mem_rd_o    = fm_grant ? fm_rd_i : (host_grant ? host_mem_rd : 1'b0);
    assign mem_wr_o    = fm_grant ? fm_wr_i : (host_grant ? host_mem_wr : 1'b0);
    assign mem_addr_o  = fm_grant ? fm_phys_addr : host_mem_word[ADDR_W-1:0];
    assign mem_wdata_o = fm_grant ? fm_wdata_i  : sw_sys_wdata_i;

    // ─── Back-pressure ───────────────────────────────────────────────────────
    // FlexMan always wins arbitration, so it only stalls on downstream wait.
    assign fm_wait_o = fm_req & mem_wait_i;

    // Host acks: register accesses are contention-free; memory accesses ack only
    // when granted and accepted by the downstream memory.
    assign sw_sys_ack_o = reg_access
                        | (host_mem_req & host_grant & ~mem_wait_i);

    // ─── Read-data return (register owner, route one cycle later) ────────────
    // Accepted reads this cycle (exactly one possible — single downstream port).
    wire fm_rd_acc      = fm_grant   & fm_rd_i     & ~mem_wait_i;
    wire host_mem_rd_acc = host_grant & host_mem_rd & ~mem_wait_i;

    reg                  host_mem_rd_pend_r;
    reg                  host_reg_rd_pend_r;
    reg [DATA_W-1:0]     reg_rd_data_r;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            host_mem_rd_pend_r <= 1'b0;
            host_reg_rd_pend_r <= 1'b0;
            reg_rd_data_r      <= {DATA_W{1'b0}};
        end else begin
            host_mem_rd_pend_r <= host_mem_rd_acc;
            host_reg_rd_pend_r <= reg_rd;          // reg read acked now, data next cycle
            reg_rd_data_r      <= (reg_off == 32'd0)
                                ? {{(DATA_W-ADDR_W){1'b0}}, base_ptr_r}
                                : {{(DATA_W-ADDR_W){1'b0}}, block_length_r};
        end
    end

    // FlexMan reads its data the cycle after acceptance (wait-based handshake,
    // same timing contract as shared_pool). Only fm's read is in flight then.
    assign fm_rdata_o = mem_rdata_i;

    assign sw_sys_rdata_o  = host_reg_rd_pend_r ? reg_rd_data_r : mem_rdata_i;
    assign sw_sys_rvalid_o = host_mem_rd_pend_r | host_reg_rd_pend_r;

endmodule
