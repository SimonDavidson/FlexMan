// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
`timescale 1ns/1ps
`include "../shared/constants.v"

// Test plan (8-bit elements, bp=0, stream_len=4, all base addrs=0):
//   Test 1: mode 0, Z=0        → R = B               (0*(A-B)+B = B)
//   Test 2: mode 1, Z=0        → R = B + R_prev       (0*(A-B)+B+R_prev)
//   Test 3: mode 0, A=B, Z≠0   → R = B               (Z*0+B = B)

module tb_hadamard_unit;

localparam TGT_ACC_ID   = 0;
localparam TGT_CFG_BASE = 16'h1D1D;
localparam DATA_BITS    = 32;
localparam ADDR_SZ      = `ADDR_SIZE;
localparam PIN_BITS     = `PIN_BITS;
localparam TGT_ACC_SZ   = `TGT_ACC_SZ;
localparam SCH_ENTRY_SZ = `SCH_ENTRY_SZ;
localparam NUM_ELEM_SZ  = 16;

reg clk, reset;

reg            hu_sys_req_i;
wire           hu_sys_ack_o;
reg  [31:0]    hu_sys_addr_i;
reg  [31:0]    hu_sys_data_i;

reg  [TGT_ACC_SZ-1:0]   hu_target_acc_i;
reg  [SCH_ENTRY_SZ-1:0] hu_buffer_info_i;
reg                      hu_start_new_block_i;
wire                     hu_acc_busy_o;
wire                     hu_acc_finished_o;

wire [ADDR_SZ-1:0]   src_a_mem_addr_o;
wire                 src_a_mem_rd_o;
reg  [DATA_BITS-1:0] src_a_mem_data_i;
reg                  src_a_mem_wait_i;

wire [ADDR_SZ-1:0]   src_b_mem_addr_o;
wire                 src_b_mem_rd_o;
reg  [DATA_BITS-1:0] src_b_mem_data_i;
reg                  src_b_mem_wait_i;

wire [ADDR_SZ-1:0]   src_z_mem_addr_o;
wire                 src_z_mem_rd_o;
reg  [DATA_BITS-1:0] src_z_mem_data_i;
reg                  src_z_mem_wait_i;

wire [ADDR_SZ-1:0]   src_r_mem_addr_o;
wire                 src_r_mem_rd_o;
reg  [DATA_BITS-1:0] src_r_mem_data_i;
reg                  src_r_mem_wait_i;

wire                 src_r_mem_wr_o;
wire [ADDR_SZ-1:0]   src_r_mem_wr_addr_o;
reg                  src_r_mem_wr_wait_i;
wire [DATA_BITS-1:0] src_r_mem_data_o;

reg [DATA_BITS-1:0] mem_a [0:255];
reg [DATA_BITS-1:0] mem_b [0:255];
reg [DATA_BITS-1:0] mem_z [0:255];
reg [DATA_BITS-1:0] mem_r [0:255];

integer i, errors;

hadamard_unit #(
    .TGT_ACC_ID           (TGT_ACC_ID),
    .TGT_CONFIG_BASE_ADDR (TGT_CFG_BASE),
    .MAX_STREAM_LEN       (1024),
    .ADDR_SZ              (ADDR_SZ),
    .ACT_SZ               (`POT_OUT_SZ_SZ),
    .TGT_ACC_SZ           (TGT_ACC_SZ),
    .SCH_ENTRY_SZ         (SCH_ENTRY_SZ),
    .PIN_BITS             (PIN_BITS),
    .NUM_ELEM_SZ          (NUM_ELEM_SZ)
) dut (
    .clk                  (clk),
    .reset                (reset),
    .hu_sys_req_i         (hu_sys_req_i),
    .hu_sys_ack_o         (hu_sys_ack_o),
    .hu_sys_addr_i        (hu_sys_addr_i),
    .hu_sys_data_i        (hu_sys_data_i),
    .hu_start_new_block_i (hu_start_new_block_i),
    .hu_target_acc_i      (hu_target_acc_i),
    .hu_buffer_info_i     (hu_buffer_info_i),
    .hu_acc_busy_o        (hu_acc_busy_o),
    .hu_acc_finished_o    (hu_acc_finished_o),
    .src_a_mem_rd_o       (src_a_mem_rd_o),
    .src_a_mem_wait_i     (src_a_mem_wait_i),
    .src_a_mem_addr_o     (src_a_mem_addr_o),
    .src_a_mem_data_i     (src_a_mem_data_i),
    .src_b_mem_rd_o       (src_b_mem_rd_o),
    .src_b_mem_wait_i     (src_b_mem_wait_i),
    .src_b_mem_addr_o     (src_b_mem_addr_o),
    .src_b_mem_data_i     (src_b_mem_data_i),
    .src_z_mem_rd_o       (src_z_mem_rd_o),
    .src_z_mem_wait_i     (src_z_mem_wait_i),
    .src_z_mem_addr_o     (src_z_mem_addr_o),
    .src_z_mem_data_i     (src_z_mem_data_i),
    .src_r_mem_rd_o       (src_r_mem_rd_o),
    .src_r_mem_wait_i     (src_r_mem_wait_i),
    .src_r_mem_addr_o     (src_r_mem_addr_o),
    .src_r_mem_data_i     (src_r_mem_data_i),
    .src_r_mem_wr_o       (src_r_mem_wr_o),
    .src_r_mem_wr_addr_o  (src_r_mem_wr_addr_o),
    .src_r_mem_wr_wait_i  (src_r_mem_wr_wait_i),
    .src_r_mem_data_o     (src_r_mem_data_o)
);

always #5 clk = ~clk;


// 1-cycle read latency memory models
always @(posedge clk) begin
    if (src_a_mem_rd_o && !src_a_mem_wait_i) src_a_mem_data_i <= mem_a[src_a_mem_addr_o[7:0]];
    if (src_b_mem_rd_o && !src_b_mem_wait_i) src_b_mem_data_i <= mem_b[src_b_mem_addr_o[7:0]];
    if (src_z_mem_rd_o && !src_z_mem_wait_i) src_z_mem_data_i <= mem_z[src_z_mem_addr_o[7:0]];
    if (src_r_mem_rd_o && !src_r_mem_wait_i) src_r_mem_data_i <= mem_r[src_r_mem_addr_o[7:0]];
end

// Capture R write-back
always @(posedge clk)
    if (src_r_mem_wr_o && !src_r_mem_wr_wait_i)
        mem_r[src_r_mem_wr_addr_o[7:0]] <= src_r_mem_data_o;

// Write one config register; verify combinational ACK on same cycle as req
task cfg_write;
    input [31:0] addr;
    input [31:0] data;
    begin
        @(posedge clk); #1;
        hu_sys_req_i  = 1'b1;
        hu_sys_addr_i = addr;
        hu_sys_data_i = data;
        @(negedge clk);
        if (!hu_sys_ack_o) begin
            $display("FAIL  ACK not combinational for addr=0x%08X", addr);
            errors = errors + 1;
        end
        @(posedge clk); #1;
        hu_sys_req_i = 1'b0;
    end
endtask

task start_task;
    begin
        @(posedge clk); #1;
        hu_start_new_block_i = 1'b1;
        @(posedge clk); #1;
        hu_start_new_block_i = 1'b0;
    end
endtask

task wait_done;
    begin
        fork
            begin : wdog
                repeat (2000) @(posedge clk);
                $display("TIMEOUT waiting for hu_acc_finished_o");
                $finish;
            end
            begin : done
                @(posedge hu_acc_finished_o);
                disable wdog;
            end
        join
        repeat (2) @(posedge clk);
    end
endtask

task check_r;
    input [7:0]  waddr;
    input [31:0] expected;
    input integer tnum;
    begin
        if (mem_r[waddr] !== expected) begin
            $display("FAIL  test %0d: mem_r[0x%02X]=0x%08X  expected=0x%08X",
                     tnum, waddr, mem_r[waddr], expected);
            errors = errors + 1;
        end else
            $display("PASS  test %0d: mem_r[0x%02X]=0x%08X", tnum, waddr, mem_r[waddr]);
    end
endtask

initial begin
    clk                  = 0;
    reset                = 1;
    hu_sys_req_i         = 0;
    hu_sys_addr_i        = 0;
    hu_sys_data_i        = 0;
    hu_start_new_block_i = 0;
    hu_target_acc_i      = TGT_ACC_ID[TGT_ACC_SZ-1:0];
    hu_buffer_info_i     = 0;
    src_a_mem_wait_i     = 0;
    src_b_mem_wait_i     = 0;
    src_z_mem_wait_i     = 0;
    src_r_mem_wait_i     = 0;
    src_r_mem_wr_wait_i  = 0;
    src_a_mem_data_i     = 0;
    src_b_mem_data_i     = 0;
    src_z_mem_data_i     = 0;
    src_r_mem_data_i     = 0;
    errors               = 0;

    for (i = 0; i < 256; i = i + 1) begin
        mem_a[i] = 0; mem_b[i] = 0; mem_z[i] = 0; mem_r[i] = 0;
    end
    mem_r[0] = 32'h02020202;  // pre-load R_prev for test 2 before cache is filled

    $dumpfile("tb_hadamard_unit.vcd");
    $dumpvars(0, tb_hadamard_unit);

    repeat (2) @(posedge clk);
    #1 reset = 0;
    repeat (2) @(posedge clk);

    // Common config: 8-bit elements, bp=0, stream_len=4, all streams at base 0
    // (each stream uses its own separate memory array in this testbench)
    cfg_write(32'h1D1D_0001, 32'd4);  // stream_len = 4
    cfg_write(32'h1D1D_0004, 32'd0);  // A base
    cfg_write(32'h1D1D_0005, 32'd3);  // A elem_sz = 8-bit
    cfg_write(32'h1D1D_0006, 32'd0);  // A bin_point = 0
    cfg_write(32'h1D1D_0008, 32'd0);  // B base
    cfg_write(32'h1D1D_0009, 32'd3);  // B elem_sz = 8-bit
    cfg_write(32'h1D1D_000A, 32'd0);  // B bin_point = 0
    cfg_write(32'h1D1D_000C, 32'd0);  // Z base
    cfg_write(32'h1D1D_000D, 32'd3);  // Z elem_sz = 8-bit
    cfg_write(32'h1D1D_000E, 32'd0);  // Z bin_point = 0
    cfg_write(32'h1D1D_0010, 32'd0);  // R base
    cfg_write(32'h1D1D_0011, 32'd3);  // R elem_sz = 8-bit
    cfg_write(32'h1D1D_0012, 32'd0);  // R bin_point = 0

    // -------------------------------------------------------------------
    // Test 1: mode=0, Z=0, A=8, B=4  →  R = 0*(8-4)+4 = 4 per element
    // -------------------------------------------------------------------
    cfg_write(32'h1D1D_0000, 32'd0);          // mode = 0 (init)
    mem_a[0] = 32'h08080808;                  // A = 8 (little-endian, 4×8-bit)
    mem_b[0] = 32'h04040404;                  // B = 4
    mem_z[0] = 32'h00000000;                  // Z = 0

    start_task;
    wait_done;
    check_r(8'h00, 32'h04040404, 1);

    // -------------------------------------------------------------------
    // Test 2: mode=1, Z=0, A=8, B=4, R_prev=2  →  R = 0+4+2 = 6 per element
    // -------------------------------------------------------------------
    cfg_write(32'h1D1D_0000, 32'd1);          // mode = 1 (update)
    mem_a[0] = 32'h08080808;
    mem_b[0] = 32'h04040404;
    mem_z[0] = 32'h00000000;
    mem_r[0] = 32'h02020202;                  // R_prev = 2

    start_task;
    wait_done;
    check_r(8'h00, 32'h06060606, 2);

    // -------------------------------------------------------------------
    // Test 3: mode=0, A=B, Z≠0  →  R = Z*0+B = B = 4 per element
    // -------------------------------------------------------------------
    cfg_write(32'h1D1D_0000, 32'd0);          // mode = 0
    mem_a[0] = 32'h04040404;                  // A = B = 4
    mem_b[0] = 32'h04040404;
    mem_z[0] = 32'h20202020;                  // Z = 32 (nonzero, but A-B=0)

    start_task;
    wait_done;
    check_r(8'h00, 32'h04040404, 3);

    $display("");
    if (errors == 0)
        $display("All tests PASSED");
    else
        $display("%0d test(s) FAILED", errors);

    $finish;
end

endmodule
