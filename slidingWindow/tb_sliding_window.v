// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
`timescale 10ps/1ps
`include "../shared/constants.v"

// ================================================================
// tb_sliding_window
//
// Verifies sliding_window.v: the timestep-windowing interposer.
//
// Simulation
// ----------
//   xrun -sv -exit -timescale 10ps/1ps -access wrc -top tb_sliding_window \
//     ../shared/constants.v sliding_window.v tb_sliding_window.v
//   (or: bash sim_slide.bsh)
//
// A behavioural single-port RAM model (1-cycle synchronous read, optional wait
// injection) sits on the downstream port; the host-AXI and FlexMan-side ports
// are driven by tasks.
//
// Tests
// -----
//   T1  Host load / readback: write a pattern to absolute words, read it back.
//   T2  Windowed read advances on NXT: FlexMan reads window 0..3; NXT; the same
//       window addresses now return the next timestep block (base += block_len).
//   T3  Windowed write + host readback: FlexMan writes a block, NXT, writes the
//       next block; host reads the absolute addresses and checks placement.
//   T4  Arbitration + read routing: simultaneous FlexMan + host requests —
//       FlexMan wins, host is back-pressured then serviced; data routes correctly.
//   T5  Back-pressure: assert downstream wait; verify fm_wait_o and that the read
//       completes with the correct data once wait releases (no request dropped).
// ================================================================

module tb_sliding_window;

    // ─── Parameters (match sliding_window defaults) ──────────────────────────
    localparam [31:0] SW_REG_ADDR = 32'hE000_0000;
    localparam [31:0] SW_MEM_ADDR = 32'hE001_0000;
    localparam        ADDR_W      = `ADDR_SIZE;   // 30
    localparam        DATA_W      = 32;
    localparam        MEM_WORDS   = 4096;

    // Config-register host addresses
    localparam [31:0] REG_BASE_PTR = SW_REG_ADDR | 32'h0; // word 0
    localparam [31:0] REG_BLOCKLEN = SW_REG_ADDR | 32'h4; // word 1

    // ─── DUT I/O ─────────────────────────────────────────────────────────────
    reg                clk, reset;
    reg                nxt_pulse_i;

    reg                sw_sys_req_i, sw_sys_we_i;
    reg  [31:0]        sw_sys_addr_i;
    reg  [DATA_W-1:0]  sw_sys_wdata_i;
    wire               sw_sys_ack_o, sw_sys_rvalid_o;
    wire [DATA_W-1:0]  sw_sys_rdata_o;

    reg                fm_rd_i, fm_wr_i;
    reg  [ADDR_W-1:0]  fm_addr_i;
    reg  [DATA_W-1:0]  fm_wdata_i;
    wire [DATA_W-1:0]  fm_rdata_o;
    wire               fm_wait_o;

    wire               mem_rd_o, mem_wr_o;
    wire [ADDR_W-1:0]  mem_addr_o;
    wire [DATA_W-1:0]  mem_wdata_o;
    wire [DATA_W-1:0]  mem_rdata_i;
    wire               mem_wait_i;

    integer errors = 0;

    // ─── DUT ─────────────────────────────────────────────────────────────────
    sliding_window #(
        .SW_REG_ADDR (SW_REG_ADDR),
        .SW_MEM_ADDR (SW_MEM_ADDR),
        .ADDR_W      (ADDR_W),
        .DATA_W      (DATA_W)
    ) dut (
        .clk(clk), .reset(reset), .nxt_pulse_i(nxt_pulse_i),
        .sw_sys_req_i(sw_sys_req_i), .sw_sys_we_i(sw_sys_we_i),
        .sw_sys_addr_i(sw_sys_addr_i), .sw_sys_wdata_i(sw_sys_wdata_i),
        .sw_sys_ack_o(sw_sys_ack_o), .sw_sys_rdata_o(sw_sys_rdata_o),
        .sw_sys_rvalid_o(sw_sys_rvalid_o),
        .fm_rd_i(fm_rd_i), .fm_wr_i(fm_wr_i), .fm_addr_i(fm_addr_i),
        .fm_wdata_i(fm_wdata_i), .fm_rdata_o(fm_rdata_o), .fm_wait_o(fm_wait_o),
        .mem_rd_o(mem_rd_o), .mem_wr_o(mem_wr_o), .mem_addr_o(mem_addr_o),
        .mem_wdata_o(mem_wdata_o), .mem_rdata_i(mem_rdata_i), .mem_wait_i(mem_wait_i)
    );

    // ─── Behavioural single-port RAM model ───────────────────────────────────
    reg [DATA_W-1:0] ram [0:MEM_WORDS-1];
    reg [ADDR_W-1:0] ram_rd_addr_r;
    reg              wait_force;

    assign mem_wait_i = wait_force;
    assign mem_rdata_i = ram[ram_rd_addr_r];

    always @(posedge clk) begin
        if (!mem_wait_i) begin
            if (mem_wr_o) ram[mem_addr_o] <= mem_wdata_o;
            ram_rd_addr_r <= mem_addr_o;
        end
    end

    // ─── Clock ───────────────────────────────────────────────────────────────
    initial clk = 0;
    always #10 clk = ~clk;

    // ─── Helpers ─────────────────────────────────────────────────────────────
    task expect_eq(input [DATA_W-1:0] got, input [DATA_W-1:0] exp, input [255:0] label);
    begin
        if (got === exp) $display("  PASS %0s : 0x%08x", label, got);
        else begin
            $display("  FAIL %0s : got 0x%08x exp 0x%08x", label, got, exp);
            errors = errors + 1;
        end
    end
    endtask

    // Host AXI write (reg or absolute memory). Holds req until accepted.
    task host_write(input [31:0] addr, input [DATA_W-1:0] data);
    begin
        @(negedge clk);
        sw_sys_req_i = 1; sw_sys_we_i = 1; sw_sys_addr_i = addr; sw_sys_wdata_i = data;
        @(posedge clk);
        while (!sw_sys_ack_o) @(posedge clk);
        @(negedge clk);
        sw_sys_req_i = 0; sw_sys_we_i = 0;
    end
    endtask

    // Host AXI read (reg or absolute memory). Returns data via rvalid one cycle
    // after acceptance.
    task host_read(input [31:0] addr, output [DATA_W-1:0] data);
    begin
        @(negedge clk);
        sw_sys_req_i = 1; sw_sys_we_i = 0; sw_sys_addr_i = addr;
        @(posedge clk);
        while (!sw_sys_ack_o) @(posedge clk);
        @(negedge clk);
        sw_sys_req_i = 0;
        if (!sw_sys_rvalid_o) @(negedge clk);
        data = sw_sys_rdata_o;
    end
    endtask

    // FlexMan windowed read. Data valid one cycle after acceptance.
    task fm_read(input [ADDR_W-1:0] addr, output [DATA_W-1:0] data);
    begin
        @(negedge clk);
        fm_rd_i = 1; fm_addr_i = addr;
        @(posedge clk);
        while (fm_wait_o) @(posedge clk);
        @(negedge clk);
        fm_rd_i = 0;
        data = fm_rdata_o;
    end
    endtask

    // FlexMan windowed write.
    task fm_write(input [ADDR_W-1:0] addr, input [DATA_W-1:0] data);
    begin
        @(negedge clk);
        fm_wr_i = 1; fm_addr_i = addr; fm_wdata_i = data;
        @(posedge clk);
        while (fm_wait_o) @(posedge clk);
        @(negedge clk);
        fm_wr_i = 0;
    end
    endtask

    // One-cycle NXT pulse (advance pointer once).
    task do_nxt;
    begin
        @(negedge clk); nxt_pulse_i = 1;
        @(negedge clk); nxt_pulse_i = 0;
    end
    endtask

    // ─── Stimulus ────────────────────────────────────────────────────────────
    reg [DATA_W-1:0] rd;
    integer i;

    initial begin
        // init
        reset = 1; nxt_pulse_i = 0;
        sw_sys_req_i = 0; sw_sys_we_i = 0; sw_sys_addr_i = 0; sw_sys_wdata_i = 0;
        fm_rd_i = 0; fm_wr_i = 0; fm_addr_i = 0; fm_wdata_i = 0;
        wait_force = 0; ram_rd_addr_r = 0;
        for (i = 0; i < MEM_WORDS; i = i + 1) ram[i] = 32'h0;
        repeat (3) @(posedge clk);
        @(negedge clk); reset = 0;
        @(negedge clk);

        // ── T1: host load / readback ─────────────────────────────────────────
        $display("\nT1 — host load / readback");
        for (i = 0; i < 4; i = i + 1)
            host_write(SW_MEM_ADDR | (i << 2), 32'hA0000000 + i);
        for (i = 0; i < 4; i = i + 1) begin
            host_read(SW_MEM_ADDR | (i << 2), rd);
            expect_eq(rd, 32'hA0000000 + i, "T1 readback");
        end

        // ── T2: windowed read advances on NXT ────────────────────────────────
        $display("\nT2 — windowed read advances on NXT");
        host_write(REG_BASE_PTR, 32'd0);   // base = 0
        host_write(REG_BLOCKLEN, 32'd8);   // block_length = 8 words
        // timestep 0 block at absolute 0..3, timestep 1 block at absolute 8..11
        for (i = 0; i < 4; i = i + 1) begin
            host_write(SW_MEM_ADDR | (i << 2),       32'hB0000000 + i); // t0
            host_write(SW_MEM_ADDR | ((i+8) << 2),   32'hB1000000 + i); // t1
        end
        for (i = 0; i < 4; i = i + 1) begin
            fm_read(i, rd);
            expect_eq(rd, 32'hB0000000 + i, "T2 t0 window");
        end
        do_nxt;  // base -> 8
        for (i = 0; i < 4; i = i + 1) begin
            fm_read(i, rd);
            expect_eq(rd, 32'hB1000000 + i, "T2 t1 window");
        end

        // ── T3: windowed write + host readback ───────────────────────────────
        $display("\nT3 — windowed write + host readback");
        host_write(REG_BASE_PTR, 32'd64);  // base = 64
        host_write(REG_BLOCKLEN, 32'd8);
        for (i = 0; i < 4; i = i + 1) fm_write(i, 32'hC0000000 + i); // -> abs 64..67
        do_nxt;                                                      // base -> 72
        for (i = 0; i < 4; i = i + 1) fm_write(i, 32'hD0000000 + i); // -> abs 72..75
        for (i = 0; i < 4; i = i + 1) begin
            host_read(SW_MEM_ADDR | ((64+i) << 2), rd);
            expect_eq(rd, 32'hC0000000 + i, "T3 block0 abs");
        end
        for (i = 0; i < 4; i = i + 1) begin
            host_read(SW_MEM_ADDR | ((72+i) << 2), rd);
            expect_eq(rd, 32'hD0000000 + i, "T3 block1 abs");
        end

        // ── T4: arbitration + read routing ───────────────────────────────────
        $display("\nT4 — arbitration + read routing");
        host_write(REG_BASE_PTR, 32'd0);
        host_write(SW_MEM_ADDR | (0   << 2), 32'h11110000); // FlexMan target (window 0)
        host_write(SW_MEM_ADDR | (200 << 2), 32'h22220000); // host target (abs 200)
        // Drive FlexMan read (window 0) and host read (abs 200) simultaneously.
        @(negedge clk);
        fm_rd_i = 1; fm_addr_i = 0;
        sw_sys_req_i = 1; sw_sys_we_i = 0; sw_sys_addr_i = SW_MEM_ADDR | (200 << 2);
        @(posedge clk);                       // edge 1: FlexMan wins
        if (fm_wait_o)  begin $display("  FAIL T4: FlexMan stalled (should win)"); errors=errors+1; end
        if (sw_sys_ack_o) begin $display("  FAIL T4: host acked while FlexMan active"); errors=errors+1; end
        @(negedge clk);
        fm_rd_i = 0;                          // drop FlexMan; host should win next
        expect_eq(fm_rdata_o, 32'h11110000, "T4 fm data");
        @(posedge clk);                       // edge 2: host wins
        while (!sw_sys_ack_o) @(posedge clk);
        @(negedge clk);
        sw_sys_req_i = 0;
        if (!sw_sys_rvalid_o) @(negedge clk);
        expect_eq(sw_sys_rdata_o, 32'h22220000, "T4 host data");

        // ── T5: downstream back-pressure ─────────────────────────────────────
        $display("\nT5 — downstream back-pressure");
        host_write(REG_BASE_PTR, 32'd0);
        host_write(SW_MEM_ADDR | (5 << 2), 32'hBEEF0005);
        @(negedge clk);
        wait_force = 1;
        fm_rd_i = 1; fm_addr_i = 5;
        @(posedge clk);
        if (!fm_wait_o) begin $display("  FAIL T5: fm_wait_o low under back-pressure"); errors=errors+1; end
        repeat (3) @(posedge clk);
        if (!fm_wait_o) begin $display("  FAIL T5: fm_wait_o released early"); errors=errors+1; end
        @(negedge clk); wait_force = 0;       // release
        @(posedge clk);                        // accepted here
        while (fm_wait_o) @(posedge clk);
        @(negedge clk);
        fm_rd_i = 0;
        expect_eq(fm_rdata_o, 32'hBEEF0005, "T5 data after wait");

        // ── Summary ──────────────────────────────────────────────────────────
        repeat (2) @(posedge clk);
        $display("\n========================================");
        if (errors == 0) $display("ALL TESTS PASSED");
        else             $display("%0d CHECK(S) FAILED", errors);
        $display("========================================\n");
        $finish;
    end

    // Safety timeout
    initial begin
        #200000;
        $display("TIMEOUT — simulation did not finish");
        $finish;
    end

endmodule
