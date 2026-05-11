/////////////////////////////////////////////////////////////////////
//
// sram_bram
//
// Single-port synchronous SRAM written so that FPGA tools infer it
// as block RAM. One-cycle registered read; same-cycle write.
//
//   we=1, re=0 : write wdata into mem[addr] on posedge clk
//   we=0, re=1 : present mem[addr] on rdata one cycle after addr
//   we=1, re=1 : write-then-read (rdata receives the OLD contents,
//                because the read assignment uses the same blocking
//                semantics as the write — see write-first vs read-
//                first below). Typical SNN access patterns never
//                assert both at the same time on the same cycle.
//
// If INIT_FILE is non-empty, $readmemh loads it into the array at
// time 0. Default is "" (no init).
//
/////////////////////////////////////////////////////////////////////

module sram_bram # (
    parameter DATA_W    = 32,
    parameter ADDR_W    = 12,           // 4096 entries -> 16 KB at 32-bit
    parameter INIT_FILE = ""
)(
    input  wire              clk,
    input  wire              we,
    input  wire              re,
    input  wire [ADDR_W-1:0] addr,
    input  wire [DATA_W-1:0] wdata,
    output reg  [DATA_W-1:0] rdata
);

    localparam DEPTH = 1 << ADDR_W;

    reg [DATA_W-1:0] mem [0:DEPTH-1];

    initial begin
        if (INIT_FILE != "") begin
            $readmemh(INIT_FILE, mem);
        end
    end

    always @(posedge clk) begin
        if (we) mem[addr] <= wdata;
        if (re) rdata     <= mem[addr];
    end

endmodule
