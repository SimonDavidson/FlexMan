// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
`timescale 1ns/1ps
`include "../shared/constants.v"

module tb_stream_generator;

localparam MAX_STREAM_LEN    = 1024;
localparam IN_DATA_BITS      = 32;
localparam SLICE_SIZE_SZ     = `POT_OUT_SZ_SZ;   // 3
localparam SLICE_DATA_IDX_SZ = 5;
localparam OUT_DATA_BITS     = 32;
localparam IDX_BITS          = $clog2(MAX_STREAM_LEN);  // 10

localparam SLICE_SZ   = 3;    // 8-bit elements
localparam STREAM_LEN = 100;

// Clock and reset
reg clk;
reg reset;

// Control inputs
reg  [IDX_BITS:0]            stream_len_i;   // 11-bit: 1..MAX_STREAM_LEN
reg  [SLICE_SIZE_SZ-1:0]     slice_sz_i;
reg  [`ADDR_SIZE-1:0]        base_addr_i;

// Status
wire                         busy_o;

// Memory interface
wire [`ADDR_SIZE-1:0]        mem_addr_o;
reg  [IN_DATA_BITS-1:0]      mem_data_i;
wire                         mem_req_o;
reg                          mem_wait_i;

// Control
reg                          start_task_i;

// Output element interface
wire                         data_valid_o;
wire [OUT_DATA_BITS-1:0]     data_o;
wire [SLICE_DATA_IDX_SZ-1:0] data_idx_o;
wire                         data_last_o;
reg                          data_taken_i;

// Memory model: 256 x 32-bit words
// With 8-bit elements and base_addr=0, max word address = (STREAM_LEN-1)/4 = 24
reg [IN_DATA_BITS-1:0] mem [0:255];

integer i;
integer element_count;

// DUT
stream_generator #(
    .MAX_STREAM_LEN    (MAX_STREAM_LEN),
    .IN_DATA_BITS      (IN_DATA_BITS),
    .SLICE_SIZE_SZ     (SLICE_SIZE_SZ),
    .SLICE_DATA_IDX_SZ (SLICE_DATA_IDX_SZ),
    .OUT_DATA_BITS     (OUT_DATA_BITS)
) dut (
    .clk          (clk),
    .reset        (reset),
    .start_task_i (start_task_i),
    .stream_len_i (stream_len_i),
    .slice_sz_i   (slice_sz_i),
    .base_addr_i  (base_addr_i),
    .busy_o       (busy_o),
    .mem_addr_o   (mem_addr_o),
    .mem_data_i   (mem_data_i),
    .mem_req_o    (mem_req_o),
    .mem_wait_i   (mem_wait_i),
    .data_valid_o (data_valid_o),
    .data_o       (data_o),
    .data_idx_o   (data_idx_o),
    .data_last_o  (data_last_o),
    .data_taken_i (data_taken_i)
);

// 10 ns clock
always #5 clk = ~clk;

// Random stall: assert mem_wait_i ~25% of cycles
always @(posedge clk)
    if (reset)
        mem_wait_i <= 0;
    else
        mem_wait_i <= ($random % 4 == 0);  // 1-in-4 chance

// Memory model: return data one cycle after a non-stalled request
always @(posedge clk)
    if (mem_req_o && !mem_wait_i)
        mem_data_i <= mem[mem_addr_o[7:0]];

initial begin
    clk           = 0;
    reset         = 1;
    start_task_i  = 0;
    stream_len_i  = STREAM_LEN;
    slice_sz_i    = SLICE_SZ;
    base_addr_i   = 0;
    mem_wait_i    = 0;  // overridden each cycle by stall generator after reset
    data_taken_i  = 1;
    mem_data_i    = 0;
    element_count = 0;

    // Fill memory with random data
    for (i = 0; i < 256; i = i + 1)
        mem[i] = $random;

    $display("Memory contents (words 0-24):");
    for (i = 0; i <= 24; i = i + 1)
        $display("  mem[%2d] = 0x%08X", i, mem[i]);
    $display("");

    $dumpfile("tb_stream_generator.vcd");
    $dumpvars(0, tb_stream_generator);

    // Reset for 2 cycles
    repeat (2) @(posedge clk);
    #1 reset = 0;

    // Pulse start for one cycle
    @(posedge clk); #1;
    start_task_i = 1;
    @(posedge clk); #1;
    start_task_i = 0;

    // Timeout watchdog races against completion
    fork
        begin : watchdog
            repeat (2000) @(posedge clk);
            $display("TIMEOUT after 2000 cycles");
            $finish;
        end
        begin : done
            wait (!busy_o);
            repeat (2) @(posedge clk);
            $display("");
            $display("Finished. Elements received: %0d / %0d  =>  %s",
                     element_count, STREAM_LEN,
                     (element_count == STREAM_LEN) ? "PASS" : "FAIL");
            disable watchdog;
            $finish;
        end
    join
end

// Log each accepted output element
always @(posedge clk) begin
    if (data_valid_o && data_taken_i) begin
        $display("t=%5t  elem[%3d]  in_word_addr=%2d  slice_idx=%1d  data=0x%08X  last=%b",
                 $time, element_count,
                 mem_addr_o[7:0],   // word address that was fetched
                 data_idx_o,
                 data_o,
                 data_last_o);
        element_count = element_count + 1;
    end
end

endmodule
