/////////////////////////////////////////////////////////////////////
//
// wrapped_acc_snn_processor_plus_memories
//
// Wraps acc_snn_processor and attaches one on-chip 16 KB inferred
// BRAM to each of its seven memory interfaces:
//
//   act_mem       (RO from DUT)
//   weight_mem    (RO from DUT)
//   syn_curr_mem  (RW, internally arbitrated by acc_snn_processor —
//                  neuron_processing has fixed priority)
//   bias_curr_mem (RO from DUT)
//   thresh_mem    (RO from DUT)
//   pot_mem       (RW from DUT)
//   spike_mem     (WO from DUT — the network's output)
//
// All memory-bus signals stay inside the wrapper. To keep the
// design observable to synthesis (so Vivado does not prune the BRAMs
// as dead logic), the wrapper exposes a small read-side port on the
// spike memory: assert dbg_spike_rd_i with dbg_spike_addr_i and the
// stored 32-bit spike word arrives on dbg_spike_data_o one cycle
// later (registered read, just like every other BRAM port). DUT
// writes have priority — if spike_mem_wr_o and dbg_spike_rd_i
// happen on the same cycle, the address bus goes to the writer and
// the read returns the just-written word. Typical use: poll
// acc_finished_o, then walk the spike memory.
//
// The DUT's *_mem_wait_i inputs are tied low because the BRAM
// always responds in one cycle.
//
// Each BRAM is 4096 x 32-bit (16 KB) by default; see BRAM_ADDR_W.
// Each BRAM accepts an INIT_FILE parameter (default "") so the
// wrapper can be pre-loaded via $readmemh at time 0.
//
// Default parameter values mirror what tb_acc_snn_processor.v
// currently overrides into the DUT — instantiate with no overrides
// to get exactly the testbench configuration.
//
/////////////////////////////////////////////////////////////////////

`include "constants.v"

module wrapped_acc_snn_processor_plus_memories # (

    //----------------------------------------------------------------
    // BRAM sizing / init
    //----------------------------------------------------------------
    parameter BRAM_ADDR_W          = 12,            // 4096 entries -> 16 KB at 32-bit
    // RO BRAMs (we tied to 0) need non-empty INIT_FILE, otherwise
    // Vivado constant-propagates mem[]=0 and prunes both the BRAM
    // and the downstream cone of logic that consumes the read data.
    parameter ACT_INIT_FILE        = "sram_init_random.hex",
    parameter WEIGHT_INIT_FILE     = "sram_init_random.hex",
    parameter SYN_CURR_INIT_FILE   = "",
    parameter BIAS_CURR_INIT_FILE  = "sram_init_random.hex",
    parameter THRESH_INIT_FILE     = "sram_init_random.hex",
    parameter POT_INIT_FILE        = "",
    parameter SPIKE_INIT_FILE      = "",

    //----------------------------------------------------------------
    // Top-level forwards (defaults match tb_acc_snn_processor.v)
    //----------------------------------------------------------------
    parameter TGT_ACC_ID              = 'h0,
    parameter TGT_CONFIG_BASE_ADDR    = 32'hFFFFFFFF,

    //--- spike_processing ---
    parameter SP_NUM_TIMESTEPS        = 32,
    parameter SP_X_INPUT_SZ           = 5,
    parameter SP_Y_INPUT_SZ           = 5,
    parameter SP_X_OUTPUT_SZ          = 5,
    parameter SP_Y_OUTPUT_SZ          = 5,
    parameter SP_X_KERNEL_SZ          = 3,
    parameter SP_Y_KERNEL_SZ          = 3,
    parameter SP_X_KERNEL_OFF_SZ      = 3,
    parameter SP_Y_KERNEL_OFF_SZ      = 3,
    parameter SP_X_STEP_SZ            = 3,
    parameter SP_Y_STEP_SZ            = 3,
    parameter SP_ELEMS_PER_ROW        = 4,
    parameter SP_ROWS_PER_NEURON      = 4,
    parameter SP_TIMESTEP_SZ          = 10,
    parameter SP_IN_DATA_BITS         = 32,
    parameter SP_ELEM_SZ              = 8,
    parameter SP_ACT_SLICE_SZ         = 3,
    parameter SP_ACT_DATA_IDX_SZ      = 5,
    parameter SP_WEIGHT_ENTRY_BITS    = 8,
    parameter SP_WEIGHT_IDX_SZ        = 10,
    parameter SP_WEIGHT_SLICE_SZ      = 5,
    parameter SP_WEIGHT_DATA_IDX_SZ   = 5,
    parameter SP_SYN_CURR_IDX_SZ      = 10,
    parameter SP_SYN_CURR_DATA_IDX_SZ = 5,
    parameter SP_SYN_CURR_SLICE_SZ    = 3,
    parameter SP_SYN_CURR_SLICE_BITS  = 32,
    parameter SP_BIAS_CURR_IDX_SZ     = 10,
    parameter SP_BIAS_CURR_DATA_IDX_SZ= 5,
    parameter SP_BIAS_CURR_SLICE_SZ   = 3,
    parameter SP_BIAS_CURR_SLICE_BITS = 8,

    //--- neuron_processing ---
    parameter NP_NUM_TIMESTEPS        = 32,
    parameter NP_TIMESTEP_SZ          = 10,
    parameter NP_IN_DATA_BITS         = 32,
    parameter NP_NEURON_IDX_SZ        = 10,
    parameter NP_SYN_CURR_IDX_SZ      = 10,
    parameter NP_SYN_CURR_DATA_IDX_SZ = 5,
    parameter NP_SYN_CURR_SLICE_SZ    = 3,
    parameter NP_SYN_CURR_SLICE_BITS  = 32,
    parameter NP_BIAS_CURR_IDX_SZ     = 10,
    parameter NP_BIAS_CURR_DATA_IDX_SZ= 5,
    parameter NP_BIAS_CURR_SLICE_SZ   = 3,
    parameter NP_BIAS_CURR_SLICE_BITS = 8,
    parameter NP_POT_IDX_SZ           = 10,
    parameter NP_POT_DATA_IDX_SZ      = 5,
    parameter NP_POT_SLICE_SZ         = 3,
    parameter NP_POT_SLICE_BITS       = 32,
    parameter NP_SPIKE_IDX_SZ         = 10,
    parameter NP_SPIKE_DATA_IDX_SZ    = 5,
    parameter NP_SPIKE_SLICE_SZ       = 3,
    parameter NP_SPIKE_SLICE_BITS     = 8,

    parameter MEM_ADDR_BITS           = `ADDR_SIZE
)(
    input  wire clk,
    input  wire reset,

    //================================================================
    // AXI config interface
    //================================================================
    input  wire        sys_req_i,
    output wire        sys_ack_o,
    input  wire [31:0] sys_addr_i,
    input  wire [31:0] sys_data_i,

    //================================================================
    // Scheduler interface
    //================================================================
    input  wire                     start_new_block_i,
    input  wire  [`TGT_ACC_SZ-1:0]  target_acc_i,
    input  wire [`SCH_ENTRY_SZ-1:0] buffer_info_i,
    output wire                     spike_proc_finished_o,
    output wire                     acc_busy_o,
    output wire                     acc_finished_o,

    //================================================================
    // Buffer-address interface – spike_processing
    //================================================================
    input  wire [`PIN_BITS-1:0] sp_src1_buff_addr_i,
    input  wire [`PIN_BITS-1:0] sp_src2_buff_addr_i,
    input  wire [`PIN_BITS-1:0] sp_src3_buff_addr_i,
    input  wire [`PIN_BITS-1:0] sp_tgt_buff_addr_i,
    input  wire [`PIN_BITS-1:0] sp_weight_row_len_i,

    //================================================================
    // Buffer-address interface – neuron_processing
    //================================================================
    input  wire [`PIN_BITS-1:0] np_src1_buff_addr_i,
    input  wire [`PIN_BITS-1:0] np_src2_buff_addr_i,
    input  wire [`PIN_BITS-1:0] np_src3_buff_addr_i,
    input  wire [`PIN_BITS-1:0] np_tgt_buff_addr_i,
    input  wire [`PIN_BITS-1:0] np_weight_row_len_i,

    //================================================================
    // Spike-memory readback (host visibility into the network output)
    //================================================================
    input  wire                    dbg_spike_rd_i,
    input  wire [BRAM_ADDR_W-1:0]  dbg_spike_addr_i,
    output wire    [`ACT_BITS-1:0] dbg_spike_data_o
);

    //----------------------------------------------------------------
    // DUT <-> on-chip memory wires
    //----------------------------------------------------------------
    wire                  weight_mem_rd;
    wire [`ADDR_SIZE-1:0] weight_mem_addr;
    wire  [`WTD_BITS-1:0] weight_mem_data;

    wire                  act_mem_req;
    wire [`ADDR_SIZE-1:0] act_mem_addr;
    wire  [`ACT_BITS-1:0] act_mem_data;

    wire                  syn_curr_mem_wr;
    wire                  syn_curr_mem_rd;
    wire [`ADDR_SIZE-1:0] syn_curr_mem_addr;
    wire  [`POT_BITS-1:0] syn_curr_mem_wdata;
    wire  [`POT_BITS-1:0] syn_curr_mem_rdata;

    wire                  bias_curr_mem_rd;
    wire [`ADDR_SIZE-1:0] bias_curr_mem_addr;
    wire  [`WTD_BITS-1:0] bias_curr_mem_data;

    wire                  thresh_mem_rd;
    wire [`ADDR_SIZE-1:0] thresh_mem_addr;
    wire  [`WTD_BITS-1:0] thresh_mem_data;

    wire                  pot_mem_wr;
    wire                  pot_mem_rd;
    wire [`ADDR_SIZE-1:0] pot_mem_addr;
    wire  [`POT_BITS-1:0] pot_mem_wdata;
    wire  [`POT_BITS-1:0] pot_mem_rdata;

    wire                  spike_mem_wr;
    wire [`ADDR_SIZE-1:0] spike_mem_addr;
    wire  [`ACT_BITS-1:0] spike_mem_wdata;

    //----------------------------------------------------------------
    // acc_snn_processor instance
    //----------------------------------------------------------------
    acc_snn_processor # (
        .TGT_ACC_ID              (TGT_ACC_ID),
        .TGT_CONFIG_BASE_ADDR    (TGT_CONFIG_BASE_ADDR),
        .SP_NUM_TIMESTEPS        (SP_NUM_TIMESTEPS),
        .SP_X_INPUT_SZ           (SP_X_INPUT_SZ),
        .SP_Y_INPUT_SZ           (SP_Y_INPUT_SZ),
        .SP_X_OUTPUT_SZ          (SP_X_OUTPUT_SZ),
        .SP_Y_OUTPUT_SZ          (SP_Y_OUTPUT_SZ),
        .SP_X_KERNEL_SZ          (SP_X_KERNEL_SZ),
        .SP_Y_KERNEL_SZ          (SP_Y_KERNEL_SZ),
        .SP_X_KERNEL_OFF_SZ      (SP_X_KERNEL_OFF_SZ),
        .SP_Y_KERNEL_OFF_SZ      (SP_Y_KERNEL_OFF_SZ),
        .SP_X_STEP_SZ            (SP_X_STEP_SZ),
        .SP_Y_STEP_SZ            (SP_Y_STEP_SZ),
        .SP_ELEMS_PER_ROW        (SP_ELEMS_PER_ROW),
        .SP_ROWS_PER_NEURON      (SP_ROWS_PER_NEURON),
        .SP_TIMESTEP_SZ          (SP_TIMESTEP_SZ),
        .SP_IN_DATA_BITS         (SP_IN_DATA_BITS),
        .SP_ELEM_SZ              (SP_ELEM_SZ),
        .SP_ACT_SLICE_SZ         (SP_ACT_SLICE_SZ),
        .SP_ACT_DATA_IDX_SZ      (SP_ACT_DATA_IDX_SZ),
        .SP_WEIGHT_ENTRY_BITS    (SP_WEIGHT_ENTRY_BITS),
        .SP_WEIGHT_IDX_SZ        (SP_WEIGHT_IDX_SZ),
        .SP_WEIGHT_SLICE_SZ      (SP_WEIGHT_SLICE_SZ),
        .SP_WEIGHT_DATA_IDX_SZ   (SP_WEIGHT_DATA_IDX_SZ),
        .SP_SYN_CURR_IDX_SZ      (SP_SYN_CURR_IDX_SZ),
        .SP_SYN_CURR_DATA_IDX_SZ (SP_SYN_CURR_DATA_IDX_SZ),
        .SP_SYN_CURR_SLICE_SZ    (SP_SYN_CURR_SLICE_SZ),
        .SP_SYN_CURR_SLICE_BITS  (SP_SYN_CURR_SLICE_BITS),
        .SP_BIAS_CURR_IDX_SZ     (SP_BIAS_CURR_IDX_SZ),
        .SP_BIAS_CURR_DATA_IDX_SZ(SP_BIAS_CURR_DATA_IDX_SZ),
        .SP_BIAS_CURR_SLICE_SZ   (SP_BIAS_CURR_SLICE_SZ),
        .SP_BIAS_CURR_SLICE_BITS (SP_BIAS_CURR_SLICE_BITS),
        .NP_NUM_TIMESTEPS        (NP_NUM_TIMESTEPS),
        .NP_TIMESTEP_SZ          (NP_TIMESTEP_SZ),
        .NP_IN_DATA_BITS         (NP_IN_DATA_BITS),
        .NP_NEURON_IDX_SZ        (NP_NEURON_IDX_SZ),
        .NP_SYN_CURR_IDX_SZ      (NP_SYN_CURR_IDX_SZ),
        .NP_SYN_CURR_DATA_IDX_SZ (NP_SYN_CURR_DATA_IDX_SZ),
        .NP_SYN_CURR_SLICE_SZ    (NP_SYN_CURR_SLICE_SZ),
        .NP_SYN_CURR_SLICE_BITS  (NP_SYN_CURR_SLICE_BITS),
        .NP_BIAS_CURR_IDX_SZ     (NP_BIAS_CURR_IDX_SZ),
        .NP_BIAS_CURR_DATA_IDX_SZ(NP_BIAS_CURR_DATA_IDX_SZ),
        .NP_BIAS_CURR_SLICE_SZ   (NP_BIAS_CURR_SLICE_SZ),
        .NP_BIAS_CURR_SLICE_BITS (NP_BIAS_CURR_SLICE_BITS),
        .NP_POT_IDX_SZ           (NP_POT_IDX_SZ),
        .NP_POT_DATA_IDX_SZ      (NP_POT_DATA_IDX_SZ),
        .NP_POT_SLICE_SZ         (NP_POT_SLICE_SZ),
        .NP_POT_SLICE_BITS       (NP_POT_SLICE_BITS),
        .NP_SPIKE_IDX_SZ         (NP_SPIKE_IDX_SZ),
        .NP_SPIKE_DATA_IDX_SZ    (NP_SPIKE_DATA_IDX_SZ),
        .NP_SPIKE_SLICE_SZ       (NP_SPIKE_SLICE_SZ),
        .NP_SPIKE_SLICE_BITS     (NP_SPIKE_SLICE_BITS),
        .MEM_ADDR_BITS           (MEM_ADDR_BITS)
    ) u_acc (
        .clk                    (clk),
        .reset                  (reset),

        .sys_req_i              (sys_req_i),
        .sys_ack_o              (sys_ack_o),
        .sys_addr_i             (sys_addr_i),
        .sys_data_i             (sys_data_i),

        .start_new_block_i      (start_new_block_i),
        .target_acc_i           (target_acc_i),
        .buffer_info_i          (buffer_info_i),
        .spike_proc_finished_o  (spike_proc_finished_o),
        .acc_busy_o             (acc_busy_o),
        .acc_finished_o         (acc_finished_o),

        .sp_src1_buff_addr_i    (sp_src1_buff_addr_i),
        .sp_src2_buff_addr_i    (sp_src2_buff_addr_i),
        .sp_src3_buff_addr_i    (sp_src3_buff_addr_i),
        .sp_tgt_buff_addr_i     (sp_tgt_buff_addr_i),
        .sp_weight_row_len_i    (sp_weight_row_len_i),

        .np_src1_buff_addr_i    (np_src1_buff_addr_i),
        .np_src2_buff_addr_i    (np_src2_buff_addr_i),
        .np_src3_buff_addr_i    (np_src3_buff_addr_i),
        .np_tgt_buff_addr_i     (np_tgt_buff_addr_i),
        .np_weight_row_len_i    (np_weight_row_len_i),

        .weight_mem_rd_o        (weight_mem_rd),
        .weight_mem_wait_i      (1'b0),
        .weight_mem_addr_o      (weight_mem_addr),
        .weight_mem_data_i      (weight_mem_data),

        .act_mem_req_o          (act_mem_req),
        .act_mem_wait_i         (1'b0),
        .act_mem_addr_o         (act_mem_addr),
        .act_mem_data_i         (act_mem_data),

        .syn_curr_mem_wr_o      (syn_curr_mem_wr),
        .syn_curr_mem_rd_o      (syn_curr_mem_rd),
        .syn_curr_mem_wait_i    (1'b0),
        .syn_curr_mem_addr_o    (syn_curr_mem_addr),
        .syn_curr_mem_data_o    (syn_curr_mem_wdata),
        .syn_curr_mem_data_i    (syn_curr_mem_rdata),

        .bias_curr_mem_rd_o     (bias_curr_mem_rd),
        .bias_curr_mem_wait_i   (1'b0),
        .bias_curr_mem_addr_o   (bias_curr_mem_addr),
        .bias_curr_mem_data_i   (bias_curr_mem_data),

        .thresh_mem_rd_o        (thresh_mem_rd),
        .thresh_mem_wait_i      (1'b0),
        .thresh_mem_addr_o      (thresh_mem_addr),
        .thresh_mem_data_i      (thresh_mem_data),

        .pot_mem_wr_o           (pot_mem_wr),
        .pot_mem_rd_o           (pot_mem_rd),
        .pot_mem_wait_i         (1'b0),
        .pot_mem_addr_o         (pot_mem_addr),
        .pot_mem_data_o         (pot_mem_wdata),
        .pot_mem_data_i         (pot_mem_rdata),

        .spike_mem_wr_o         (spike_mem_wr),
        .spike_mem_wait_i       (1'b0),
        .spike_mem_addr_o       (spike_mem_addr),
        .spike_mem_data_o       (spike_mem_wdata)
    );

    //----------------------------------------------------------------
    // On-chip BRAMs (16 KB each at 32-bit; address truncated to low
    // BRAM_ADDR_W bits of the DUT's `ADDR_SIZE-wide bus)
    //----------------------------------------------------------------
    sram_bram # (
        .DATA_W   (`ACT_BITS),
        .ADDR_W   (BRAM_ADDR_W),
        .INIT_FILE(ACT_INIT_FILE)
    ) u_act_mem (
        .clk   (clk),
        .we    (1'b0),
        .re    (act_mem_req),
        .addr  (act_mem_addr[BRAM_ADDR_W-1:0]),
        .wdata ({`ACT_BITS{1'b0}}),
        .rdata (act_mem_data)
    );

    sram_bram # (
        .DATA_W   (`WTD_BITS),
        .ADDR_W   (BRAM_ADDR_W),
        .INIT_FILE(WEIGHT_INIT_FILE)
    ) u_weight_mem (
        .clk   (clk),
        .we    (1'b0),
        .re    (weight_mem_rd),
        .addr  (weight_mem_addr[BRAM_ADDR_W-1:0]),
        .wdata ({`WTD_BITS{1'b0}}),
        .rdata (weight_mem_data)
    );

    sram_bram # (
        .DATA_W   (`POT_BITS),
        .ADDR_W   (BRAM_ADDR_W),
        .INIT_FILE(SYN_CURR_INIT_FILE)
    ) u_syn_curr_mem (
        .clk   (clk),
        .we    (syn_curr_mem_wr),
        .re    (syn_curr_mem_rd),
        .addr  (syn_curr_mem_addr[BRAM_ADDR_W-1:0]),
        .wdata (syn_curr_mem_wdata),
        .rdata (syn_curr_mem_rdata)
    );

    sram_bram # (
        .DATA_W   (`WTD_BITS),
        .ADDR_W   (BRAM_ADDR_W),
        .INIT_FILE(BIAS_CURR_INIT_FILE)
    ) u_bias_curr_mem (
        .clk   (clk),
        .we    (1'b0),
        .re    (bias_curr_mem_rd),
        .addr  (bias_curr_mem_addr[BRAM_ADDR_W-1:0]),
        .wdata ({`WTD_BITS{1'b0}}),
        .rdata (bias_curr_mem_data)
    );

    sram_bram # (
        .DATA_W   (`WTD_BITS),
        .ADDR_W   (BRAM_ADDR_W),
        .INIT_FILE(THRESH_INIT_FILE)
    ) u_thresh_mem (
        .clk   (clk),
        .we    (1'b0),
        .re    (thresh_mem_rd),
        .addr  (thresh_mem_addr[BRAM_ADDR_W-1:0]),
        .wdata ({`WTD_BITS{1'b0}}),
        .rdata (thresh_mem_data)
    );

    sram_bram # (
        .DATA_W   (`POT_BITS),
        .ADDR_W   (BRAM_ADDR_W),
        .INIT_FILE(POT_INIT_FILE)
    ) u_pot_mem (
        .clk   (clk),
        .we    (pot_mem_wr),
        .re    (pot_mem_rd),
        .addr  (pot_mem_addr[BRAM_ADDR_W-1:0]),
        .wdata (pot_mem_wdata),
        .rdata (pot_mem_rdata)
    );

    // DUT writes have priority on the spike_mem address bus; debug
    // reads share the single port and only get a usable address when
    // the DUT is not writing this cycle.
    wire [BRAM_ADDR_W-1:0] spike_mem_addr_mux =
        spike_mem_wr ? spike_mem_addr[BRAM_ADDR_W-1:0]
                     : dbg_spike_addr_i;

    sram_bram # (
        .DATA_W   (`ACT_BITS),
        .ADDR_W   (BRAM_ADDR_W),
        .INIT_FILE(SPIKE_INIT_FILE)
    ) u_spike_mem (
        .clk   (clk),
        .we    (spike_mem_wr),
        .re    (dbg_spike_rd_i),
        .addr  (spike_mem_addr_mux),
        .wdata (spike_mem_wdata),
        .rdata (dbg_spike_data_o)
    );

endmodule
