`timescale 1ns/1ps

`include "constants.v"

// ====================================================================
//  tb_acc_snn_processor
//
//  Testbench for the acc_snn_processor top-level module.
//
//  Memory models
//  -------------
//  One synchronous SRAM model (256 x data-width) is instantiated per
//  memory interface.  All memories are pre-filled with random data at
//  the start of the simulation.  Each model has 1-cycle read latency
//  and no wait states (wait outputs are tied low).
//
//  Stimulus
//  --------
//  After reset, start_new_block_i is pulsed for exactly one clock
//  cycle to initiate spike_processing.  neuron_processing is then
//  triggered automatically when spike_processing completes.
//  All other inputs are held at zero throughout.
//
//  Termination
//  -----------
//  The simulation ends after 1000 clock cycles or when $finish is
//  called from the timeout block.
// ====================================================================

// CONFIG PARAMS START
`define TGT_ACC_ID 'h0
`define TGT_CONFIG_BASE_ADDR 32'hFFFFFFFF
// For spike_processing:
`define NUM_TIMESTEPS 32
`define IN_DATA_SZ 32
`define X_INPUT_SZ 5
`define Y_INPUT_SZ 5
`define X_OUTPUT_SZ 5
`define Y_OUTPUT_SZ 5
`define X_KERNEL_SZ 3
`define Y_KERNEL_SZ 3
`define X_STEP_SZ   3
`define Y_STEP_SZ   3
`define ELEMS_PER_ROW   4
`define ROWS_PER_NEURON 4
`define ELEM_SZ 8
`define WEIGHT_SLICE_SZ 5
`define WEIGHT_IDX_SZ 10
`define WEIGHT_DATA_IDX_SZ 5
`define ACT_SLICE_SZ 3      // Don't change
`define ACT_IDX_SZ 10
`define ACT_DATA_IDX_SZ 5
`define SYN_CURR_IDX_SZ 10
`define SYN_CURR_DATA_IDX_SZ 5
`define SYN_CURR_SLICE_SZ 3
`define SYN_CURR_SLICE_BITS 32
`define BIAS_CURR_IDX_SZ 10
`define BIAS_CURR_DATA_IDX_SZ 5
`define BIAS_CURR_SLICE_SZ 3
`define BIAS_CURR_SLICE_BITS 8

// For neuron_processing:
`define SYN_CURR_IDX_SZ 10
`define SYN_CURR_DATA_IDX_SZ 5
`define SYN_CURR_SLICE_SZ 3
`define SYN_CURR_SLICE_BITS 32
`define BIAS_CURR_IDX_SZ 10
`define BIAS_CURR_DATA_IDX_SZ 5
`define BIAS_CURR_SLICE_SZ 3
`define BIAS_CURR_SLICE_BITS 8
`define POT_IDX_SZ 10
`define POT_DATA_IDX_SZ 5
`define POT_SLICE_SZ 3   
`define POT_SLICE_BITS 32
`define SPIKE_IDX_SZ 10
`define SPIKE_DATA_IDX_SZ 5
`define SPIKE_SLICE_SZ 3
`define SPIKE_SLICE_BITS 8
// CONFIG PARAMS END

module tb_acc_snn_processor;

    // ----------------------------------------------------------------
    // Parameters
    // ----------------------------------------------------------------
    localparam CLK_PERIOD = 10;          // 10 ns -> 100 MHz
    localparam TIMEOUT_CYCLES = 1000;
    localparam MEM_DEPTH  = 8192;
    localparam MEM_ADDR_W = `ADDR_SIZE;

    // ----------------------------------------------------------------
    // Clock & reset
    // ----------------------------------------------------------------
    reg clk;
    reg reset;

    initial clk = 1'b0;
    always #(CLK_PERIOD/2) clk = ~clk;

    // ----------------------------------------------------------------
    // DUT input registers (all tied to 0 unless driven by stimulus)
    // ----------------------------------------------------------------

    // AXI config
    reg                    sys_req_i   = 1'b0;
    reg             [31:0] sys_addr_i  = 32'b0;
    reg             [31:0] sys_data_i  = 32'b0;

    // Scheduler
    reg                      start_new_block_i = 1'b0;
    reg   [`TGT_ACC_SZ-1:0]  target_acc_i      = {`TGT_ACC_SZ{1'b0}};
    reg [`SCH_ENTRY_SZ-1:0]  buffer_info_i     = {`SCH_ENTRY_SZ{1'b0}};

    // Buffer addresses – spike_processing
    reg [`PIN_BITS-1:0] sp_src1_buff_addr_i  = {`PIN_BITS{1'b0}};
    reg [`PIN_BITS-1:0] sp_src2_buff_addr_i  = {`PIN_BITS{1'b0}};
    reg [`PIN_BITS-1:0] sp_src3_buff_addr_i  = {`PIN_BITS{1'b0}};
    reg [`PIN_BITS-1:0] sp_tgt_buff_addr_i   = {`PIN_BITS{1'b0}};
    reg [`PIN_BITS-1:0] sp_weight_row_len_i  = {`PIN_BITS{1'b0}};

    // Buffer addresses – neuron_processing
    reg [`PIN_BITS-1:0] np_src1_buff_addr_i  = {`PIN_BITS{1'b0}};
    reg [`PIN_BITS-1:0] np_src2_buff_addr_i  = {`PIN_BITS{1'b0}};
    reg [`PIN_BITS-1:0] np_src3_buff_addr_i  = {`PIN_BITS{1'b0}};
    reg [`PIN_BITS-1:0] np_tgt_buff_addr_i   = {`PIN_BITS{1'b0}};
    reg [`PIN_BITS-1:0] np_weight_row_len_i  = {`PIN_BITS{1'b0}};

    // ----------------------------------------------------------------
    // DUT output wires
    // ----------------------------------------------------------------
    wire                    sys_ack_o;

    wire                    spike_proc_finished_o;
    wire                    acc_busy_o;
    wire                    acc_finished_o;

    // Memory interface wires (DUT -> memory models)
    wire                    weight_mem_rd_o;
    wire [`ADDR_SIZE-1:0]   weight_mem_addr_o;

    wire                    act_mem_req_o;
    wire [`ADDR_SIZE-1:0]   act_mem_addr_o;

    wire                    syn_curr_mem_wr_o;
    wire                    syn_curr_mem_rd_o;
    wire [`ADDR_SIZE-1:0]   syn_curr_mem_addr_o;
    wire  [`POT_BITS-1:0]   syn_curr_mem_data_o;

    wire                    bias_curr_mem_rd_o;
    wire [`ADDR_SIZE-1:0]   bias_curr_mem_addr_o;

    wire                    thresh_mem_rd_o;
    wire [`ADDR_SIZE-1:0]   thresh_mem_addr_o;

    wire                    pot_mem_wr_o;
    wire                    pot_mem_rd_o;
    wire [`ADDR_SIZE-1:0]   pot_mem_addr_o;
    wire  [`POT_BITS-1:0]   pot_mem_data_o;

    wire                    spike_mem_wr_o;
    wire [`ADDR_SIZE-1:0]   spike_mem_addr_o;
    wire  [`ACT_BITS-1:0]   spike_mem_data_o;

    // Memory interface wires (memory models -> DUT)
    wire  [`WTD_BITS-1:0]   weight_mem_data_i;
    wire  [`ACT_BITS-1:0]   act_mem_data_i;
    wire  [`POT_BITS-1:0]   syn_curr_mem_data_i;
    wire  [`WTD_BITS-1:0]   bias_curr_mem_data_i;
    wire  [`WTD_BITS-1:0]   thresh_mem_data_i;
    wire  [`POT_BITS-1:0]   pot_mem_data_i;

    // All wait signals tied low (no wait states)
    wire weight_mem_wait_i    = 1'b0;
    wire act_mem_wait_i       = 1'b0;
    wire syn_curr_mem_wait_i  = 1'b0;
    wire bias_curr_mem_wait_i = 1'b0;
    wire thresh_mem_wait_i    = 1'b0;
    wire pot_mem_wait_i       = 1'b0;
    wire spike_mem_wait_i     = 1'b0;

    // ----------------------------------------------------------------
    // DUT instantiation
    // ----------------------------------------------------------------
    acc_snn_processor # (
    .TGT_ACC_ID(`TGT_ACC_ID),
    .TGT_CONFIG_BASE_ADDR(`TGT_CONFIG_BASE_ADDR),
    .SP_NUM_TIMESTEPS(`NUM_TIMESTEPS),
    .SP_X_INPUT_SZ(`X_INPUT_SZ),
    .SP_Y_INPUT_SZ(`Y_INPUT_SZ),
    .SP_X_OUTPUT_SZ(`X_OUTPUT_SZ),
    .SP_Y_OUTPUT_SZ(`Y_OUTPUT_SZ),
    .SP_X_KERNEL_SZ(`X_KERNEL_SZ),
    .SP_Y_KERNEL_SZ(`Y_KERNEL_SZ),
    .SP_X_KERNEL_OFF_SZ(`X_STEP_SZ), //??
    .SP_Y_KERNEL_OFF_SZ(`Y_STEP_SZ), //??
    .SP_X_STEP_SZ(`X_STEP_SZ),
    .SP_Y_STEP_SZ(`Y_STEP_SZ),
    .SP_ELEMS_PER_ROW(`ELEMS_PER_ROW),
    .SP_ROWS_PER_NEURON(`ROWS_PER_NEURON),
    .SP_TIMESTEP_SZ(10), // ??
    .SP_IN_DATA_BITS(32), // ??
    .SP_ELEM_SZ(`ELEM_SZ),
    .SP_ACT_SLICE_SZ(`ACT_SLICE_SZ),
    .SP_ACT_DATA_IDX_SZ(`ACT_DATA_IDX_SZ),
    .SP_WEIGHT_ENTRY_BITS(8), // ??
    .SP_WEIGHT_IDX_SZ(`WEIGHT_IDX_SZ),
    .SP_WEIGHT_SLICE_SZ(`WEIGHT_SLICE_SZ),
    .SP_WEIGHT_DATA_IDX_SZ(`WEIGHT_DATA_IDX_SZ),
    .SP_SYN_CURR_IDX_SZ(`SYN_CURR_IDX_SZ),
    .SP_SYN_CURR_DATA_IDX_SZ(`SYN_CURR_DATA_IDX_SZ),
    .SP_SYN_CURR_SLICE_SZ(`SYN_CURR_SLICE_SZ),
    .SP_SYN_CURR_SLICE_BITS(`SYN_CURR_SLICE_BITS),
    .SP_BIAS_CURR_IDX_SZ(`BIAS_CURR_IDX_SZ),
    .SP_BIAS_CURR_DATA_IDX_SZ(`BIAS_CURR_DATA_IDX_SZ),
    .SP_BIAS_CURR_SLICE_SZ(`BIAS_CURR_SLICE_SZ),

    .NP_NUM_TIMESTEPS(`NUM_TIMESTEPS),
    .NP_TIMESTEP_SZ(10), // ??
    .NP_IN_DATA_BITS(32), // ??
    .NP_NEURON_IDX_SZ(10),  // ??
    .NP_SYN_CURR_IDX_SZ(`SYN_CURR_IDX_SZ),
    .NP_SYN_CURR_DATA_IDX_SZ(`SYN_CURR_DATA_IDX_SZ),
    .NP_SYN_CURR_SLICE_SZ(`SYN_CURR_SLICE_SZ),
    .NP_SYN_CURR_SLICE_BITS(`SYN_CURR_SLICE_BITS),
    .NP_BIAS_CURR_IDX_SZ(`BIAS_CURR_IDX_SZ),
    .NP_BIAS_CURR_DATA_IDX_SZ(`BIAS_CURR_DATA_IDX_SZ),
    .NP_BIAS_CURR_SLICE_SZ(`BIAS_CURR_SLICE_SZ),
    .NP_BIAS_CURR_SLICE_BITS(`BIAS_CURR_SLICE_BITS),
    .NP_POT_IDX_SZ(`POT_IDX_SZ),
    .NP_POT_DATA_IDX_SZ(`POT_DATA_IDX_SZ),
    .NP_POT_SLICE_SZ(`POT_SLICE_SZ),
    .NP_POT_SLICE_BITS(`POT_SLICE_BITS),
    .NP_SPIKE_IDX_SZ(`SPIKE_IDX_SZ),
    .NP_SPIKE_DATA_IDX_SZ(`SPIKE_DATA_IDX_SZ),
    .NP_SPIKE_SLICE_SZ(`SPIKE_SLICE_SZ),
    .NP_SPIKE_SLICE_BITS(`SPIKE_SLICE_BITS),
    .MEM_ADDR_BITS(`ADDR_SIZE)
    )
    u_dut (
        .clk                        (clk),
        .reset                      (reset),

        .sys_req_i                  (sys_req_i),
        .sys_ack_o                  (sys_ack_o),
        .sys_addr_i                 (sys_addr_i),
        .sys_data_i                 (sys_data_i),

        .start_new_block_i          (start_new_block_i),
        .target_acc_i               (target_acc_i),
        .buffer_info_i              (buffer_info_i),
        .spike_proc_finished_o      (spike_proc_finished_o),
        .acc_busy_o                 (acc_busy_o),
        .acc_finished_o             (acc_finished_o),

        .sp_src1_buff_addr_i        (sp_src1_buff_addr_i),
        .sp_src2_buff_addr_i        (sp_src2_buff_addr_i),
        .sp_src3_buff_addr_i        (sp_src3_buff_addr_i),
        .sp_tgt_buff_addr_i         (sp_tgt_buff_addr_i),
        .sp_weight_row_len_i        (sp_weight_row_len_i),

        .np_src1_buff_addr_i        (np_src1_buff_addr_i),
        .np_src2_buff_addr_i        (np_src2_buff_addr_i),
        .np_src3_buff_addr_i        (np_src3_buff_addr_i),
        .np_tgt_buff_addr_i         (np_tgt_buff_addr_i),
        .np_weight_row_len_i        (np_weight_row_len_i),

        .weight_mem_rd_o            (weight_mem_rd_o),
        .weight_mem_wait_i          (weight_mem_wait_i),
        .weight_mem_addr_o          (weight_mem_addr_o),
        .weight_mem_data_i          (weight_mem_data_i),

        .act_mem_req_o              (act_mem_req_o),
        .act_mem_wait_i             (act_mem_wait_i),
        .act_mem_addr_o             (act_mem_addr_o),
        .act_mem_data_i             (act_mem_data_i),

        .syn_curr_mem_wr_o          (syn_curr_mem_wr_o),
        .syn_curr_mem_rd_o          (syn_curr_mem_rd_o),
        .syn_curr_mem_wait_i        (syn_curr_mem_wait_i),
        .syn_curr_mem_addr_o        (syn_curr_mem_addr_o),
        .syn_curr_mem_data_o        (syn_curr_mem_data_o),
        .syn_curr_mem_data_i        (syn_curr_mem_data_i),

        .bias_curr_mem_rd_o         (bias_curr_mem_rd_o),
        .bias_curr_mem_wait_i       (bias_curr_mem_wait_i),
        .bias_curr_mem_addr_o       (bias_curr_mem_addr_o),
        .bias_curr_mem_data_i       (bias_curr_mem_data_i),

        .thresh_mem_rd_o            (thresh_mem_rd_o),
        .thresh_mem_wait_i          (thresh_mem_wait_i),
        .thresh_mem_addr_o          (thresh_mem_addr_o),
        .thresh_mem_data_i          (thresh_mem_data_i),

        .pot_mem_wr_o               (pot_mem_wr_o),
        .pot_mem_rd_o               (pot_mem_rd_o),
        .pot_mem_wait_i             (pot_mem_wait_i),
        .pot_mem_addr_o             (pot_mem_addr_o),
        .pot_mem_data_o             (pot_mem_data_o),
        .pot_mem_data_i             (pot_mem_data_i),

        .spike_mem_wr_o             (spike_mem_wr_o),
        .spike_mem_wait_i           (spike_mem_wait_i),
        .spike_mem_addr_o           (spike_mem_addr_o),
        .spike_mem_data_o           (spike_mem_data_o)
    );

    // ----------------------------------------------------------------
    // Synchronous SRAM model (1-cycle read latency, no wait states)
    //
    // Usage:
    //   sram_model #(.DATA_W(W), .DEPTH(D)) u_name (
    //       .clk(clk), .we(we), .re(re),
    //       .addr(addr), .wdata(wdata), .rdata(rdata));
    //
    // On a read  (re=1): rdata is presented on the NEXT rising edge.
    // On a write (we=1): wdata is written on the current rising edge.
    // ----------------------------------------------------------------
    // Weight memory  (read-only from DUT, `WTD_BITS wide)
    sram_model #(.DATA_W(`WTD_BITS), .DEPTH(MEM_DEPTH)) u_weight_mem (
        .clk   (clk),
        .we    (1'b0),
        .re    (weight_mem_rd_o),
        .addr  (weight_mem_addr_o[7:0]),
        .wdata ({`WTD_BITS{1'b0}}),
        .rdata (weight_mem_data_i)
    );

    // Activation memory  (read-only from DUT, `ACT_BITS wide)
    sram_model #(.DATA_W(`ACT_BITS), .DEPTH(MEM_DEPTH)) u_act_mem (
        .clk   (clk),
        .we    (1'b0),
        .re    (act_mem_req_o),
        .addr  (act_mem_addr_o[7:0]),
        .wdata ({`ACT_BITS{1'b0}}),
        .rdata (act_mem_data_i)
    );

    // Synaptic current memory  (read/write, `POT_BITS wide)
    sram_model #(.DATA_W(`POT_BITS), .DEPTH(MEM_DEPTH)) u_syn_curr_mem (
        .clk   (clk),
        .we    (syn_curr_mem_wr_o),
        .re    (syn_curr_mem_rd_o),
        .addr  (syn_curr_mem_addr_o[7:0]),
        .wdata (syn_curr_mem_data_o),
        .rdata (syn_curr_mem_data_i)
    );

    // Bias current memory  (read-only, `WTD_BITS wide)
    sram_model #(.DATA_W(`WTD_BITS), .DEPTH(MEM_DEPTH)) u_bias_curr_mem (
        .clk   (clk),
        .we    (1'b0),
        .re    (bias_curr_mem_rd_o),
        .addr  (bias_curr_mem_addr_o[7:0]),
        .wdata ({`WTD_BITS{1'b0}}),
        .rdata (bias_curr_mem_data_i)
    );

    // Threshold memory  (read-only, `WTD_BITS wide)
    sram_model #(.DATA_W(`WTD_BITS), .DEPTH(MEM_DEPTH)) u_thresh_mem (
        .clk   (clk),
        .we    (1'b0),
        .re    (thresh_mem_rd_o),
        .addr  (thresh_mem_addr_o[7:0]),
        .wdata ({`WTD_BITS{1'b0}}),
        .rdata (thresh_mem_data_i)
    );

    // Potential memory  (read/write, `POT_BITS wide)
    sram_model #(.DATA_W(`POT_BITS), .DEPTH(MEM_DEPTH)) u_pot_mem (
        .clk   (clk),
        .we    (pot_mem_wr_o),
        .re    (pot_mem_rd_o),
        .addr  (pot_mem_addr_o[7:0]),
        .wdata (pot_mem_data_o),
        .rdata (pot_mem_data_i)
    );

    // Output spike memory  (write-only from DUT; rdata unused)
    // Declared as read/write so the model is reusable; re is tied low.
    wire [`ACT_BITS-1:0] spike_mem_rdata_unused;
    sram_model #(.DATA_W(`ACT_BITS), .DEPTH(MEM_DEPTH)) u_spike_mem (
        .clk   (clk),
        .we    (spike_mem_wr_o),
        .re    (1'b0),
        .addr  (spike_mem_addr_o[7:0]),
        .wdata (spike_mem_data_o),
        .rdata (spike_mem_rdata_unused)
    );

    // ----------------------------------------------------------------
    // Cycle counter & timeout
    // ----------------------------------------------------------------
    integer cycle_count;

    initial cycle_count = 0;

    always @(posedge clk) begin
        cycle_count <= cycle_count + 1;
        if (cycle_count >= TIMEOUT_CYCLES) begin
            $display("[TB] TIMEOUT: %0d cycles elapsed. Ending simulation.", TIMEOUT_CYCLES);
            $finish;
        end
    end

    // ----------------------------------------------------------------
    // Config-write task
    //
    // Drives one write transaction on the AXI config bus.
    // sys_ack_o is combinational so no explicit wait is needed;
    // we deassert sys_req_i one negedge after asserting it.
    // ----------------------------------------------------------------
    task cfg_write;
        input [31:0] addr;
        input [31:0] data;
        begin
            @(negedge clk);
            sys_req_i  = 1'b1;
            sys_addr_i = addr;
            sys_data_i = data;
            @(negedge clk);
            sys_req_i  = 1'b0;
        end
    endtask

    // ----------------------------------------------------------------
    // Pre-load activation memory with all-spike pattern.
    // Each input element is 1-bit (slice_sz=0), 32 elements per 32-bit word.
    // 0xFFFFFFFF in every word = every input neuron fires every timestep.
    // ----------------------------------------------------------------
    integer i_init;
    initial begin
        #1;
        for (i_init = 0; i_init < MEM_DEPTH; i_init = i_init + 1)
            u_act_mem.mem[i_init] = 32'hFFFFFFFF;
    end

    // ----------------------------------------------------------------
    // Pre-load weight memory with sparse (idx, weight) tuples.
    //
    // weight_sz=8, index_sz=8, tuple_sz=16, 2 tuples per 32-bit word,
    // sparse_count=2 tuples per input neuron, rows_per_neuron=1.
    //
    // Tuple layout (16-bit, left-justified): [15:8]=idx, [7:0]=weight.
    // Slice index 0 reads lower 16 bits, slice index 1 reads upper 16 bits.
    // For input N: tuple0 at [15:0] = (idx=N,        wgt=N+1)
    //              tuple1 at [31:16]= (idx=(N+1)&15, wgt=N+2)
    // Each output cell ends up with exactly 2 hits per timestep.
    // ----------------------------------------------------------------
    integer w_init;
    initial begin
        #1;
        for (w_init = 0; w_init < MEM_DEPTH; w_init = w_init + 1)
            u_weight_mem.mem[w_init] = 32'h0;
        for (w_init = 0; w_init < 16; w_init = w_init + 1) begin
            u_weight_mem.mem[w_init] =
                  ((((w_init + 1) & 8'h0F) & 8'hFF) << 24)  // tuple1 idx
                | (((w_init + 2) & 8'hFF)            << 16) // tuple1 wgt
                | (( w_init      & 8'hFF)            <<  8) // tuple0 idx
                | (((w_init + 1) & 8'hFF)            <<  0);// tuple0 wgt
        end
    end

    // ----------------------------------------------------------------
    // Stimulus
    // ----------------------------------------------------------------
    initial begin
        // Initialise reset
        reset = 1'b1;

        // Hold reset for 5 clock cycles
        repeat (5) @(posedge clk);
        @(negedge clk);   // de-assert on falling edge to avoid setup issues
        reset = 1'b0;

        // Wait two cycles after reset before writing config registers
        repeat (2) @(posedge clk);

        // ---- spike_processing config registers ----
        // Address tag = TGT_CONFIG_BASE_ADDR[31:16] = 16'hFFFF
        cfg_write(32'hFFFF_0000, 32'h0000_1000); // sp_act_base_addr_r
        cfg_write(32'hFFFF_0004, 32'h0000_2000); // sp_weight_base_addr_r
        cfg_write(32'hFFFF_0008, 32'h0000_1000); // syn_curr_base_addr_r (shared)
        cfg_write(32'hFFFF_000C, 32'h0000_0003); // sp_weight_sz_r       = 3
        cfg_write(32'hFFFF_0014, 32'h0000_000A); // sp_total_timesteps_r = 10
        cfg_write(32'hFFFF_0044, 32'h0000_0004); // sp_in_x_len_r        = 4
        cfg_write(32'hFFFF_0048, 32'h0000_0004); // sp_in_y_len_r        = 4
        cfg_write(32'hFFFF_004C, 32'h0000_0004); // sp_out_x_len_r       = 4
        cfg_write(32'hFFFF_0050, 32'h0000_0004); // sp_out_y_len_r       = 4
        cfg_write(32'hFFFF_0054, 32'h0000_0002); // sp_weights_per_word_r = 2 (2 x 16-bit tuples per 32-bit word)
        cfg_write(32'hFFFF_0058, 32'h0000_0001); // sp_rows_per_neuron_r = 1 (one packed word per input)
        cfg_write(32'hFFFF_005C, 32'h0000_000A); // sp_weight_idx_sz_r   = 10

        // ---- sparse mode config registers ----
        cfg_write(32'hFFFF_0070, 32'h0000_0001); // sp_weight_mode_r     = 2'b01 (sparse)
        cfg_write(32'hFFFF_008C, 32'h0000_0003); // sp_index_sz_r        = 3 (8-bit indices)
        cfg_write(32'hFFFF_0090, 32'h0000_0004); // sp_tuple_sz_r        = 4 (16-bit tuples)
        cfg_write(32'hFFFF_0094, 32'h0000_0002); // sp_sparse_count_r    = 2 (2 tuples per input)

        // ---- shared config registers ----
        cfg_write(32'hFFFF_0040, 32'h0000_0008); // bin_point_syn_curr_r = 8

        // ---- neuron_processing config registers ----
        cfg_write(32'hFFFF_0020, 32'h0000_000F); // np_last_neuron_idx_r = 15 (16 outputs in 4x4 grid)
        cfg_write(32'hFFFF_0028, 32'h0000_2000); // np_bias_curr_base_addr_r
        cfg_write(32'hFFFF_002C, 32'h0000_1000); // np_thresh_base_addr_r
        cfg_write(32'hFFFF_0030, 32'h0000_2000); // np_pot_base_addr_r
        cfg_write(32'hFFFF_0064, 32'h0000_3000); // np_spike_base_addr_r
        cfg_write(32'hFFFF_0034, 32'h0000_0003); // np_syn_curr_sz_r     = 3 (8-bit elements)
        cfg_write(32'hFFFF_0038, 32'h0000_0003); // np_bias_curr_sz_r    = 3 (8-bit elements)
        cfg_write(32'hFFFF_003C, 32'h0000_0003); // np_pot_sz_r          = 3 (8-bit elements)
        cfg_write(32'hFFFF_0068, 32'hC000_0000); // np_syn_curr_decay_mult_r = 0.75 (Q0.32)
        cfg_write(32'hFFFF_006C, 32'hE000_0000); // np_pot_decay_mult_r      = 0.875 (Q0.32)

        $display("[TB] Config registers written at cycle %0d", cycle_count);

        // Pulse start_new_block_i for exactly one cycle to kick off spike_processing
        @(negedge clk);
        start_new_block_i = 1'b1;
        @(negedge clk);
        start_new_block_i = 1'b0;

        $display("[TB] start_new_block_i pulsed at cycle %0d", cycle_count);
    end

    // ----------------------------------------------------------------
    // Logging: print key status signals every cycle
    // ----------------------------------------------------------------
    always @(posedge clk) begin
        if (!reset) begin
            $display("[%4d ns | cyc %0d] busy=%b sp_finished=%b acc_finished=%b | syn_curr_rd=%b syn_curr_wr=%b syn_curr_addr=0x%h",
                $time,
                cycle_count,
                acc_busy_o,
                spike_proc_finished_o,
                acc_finished_o,
                syn_curr_mem_rd_o,
                syn_curr_mem_wr_o,
                syn_curr_mem_addr_o);
        end
    end

    // ----------------------------------------------------------------
    // Optional: VCD waveform dump
    // ----------------------------------------------------------------
    initial begin
        $dumpfile("tb_acc_snn_processor.vcd");
        $dumpvars(0, tb_acc_snn_processor);
    end

endmodule


// ====================================================================
//  sram_model
//
//  Simple synchronous SRAM.
//    - 1-cycle read latency: data appears on rdata one cycle after
//      re is asserted.
//    - Write is registered on the rising edge when we=1.
//    - Initialised with random data in an initial block.
// ====================================================================
module sram_model #(
    parameter DATA_W = 8,
    parameter DEPTH  = 256
)(
    input  wire              clk,
    input  wire              we,
    input  wire              re,
    input  wire        [7:0] addr,   // log2(256) = 8 bits
    input  wire [DATA_W-1:0] wdata,
    output reg  [DATA_W-1:0] rdata
);
    reg [DATA_W-1:0] mem [0:DEPTH-1];

    integer i;
    initial begin
        // Fill with random data
        for (i = 0; i < DEPTH; i = i + 1)
	    mem[i] = 'h01010101;
            //mem[i] = $random;
        rdata = {DATA_W{1'b0}};
    end

    always @(posedge clk) begin
        if (we)
            mem[addr] <= wdata;
        if (re)
            rdata <= mem[addr];
    end

endmodule
