// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
`timescale 10ps/1ps
`include "../shared/constants.v"

// Reads config words and buffer base addresses from external RAMs and pushes
// them to the target accelerator's register bank, then signals completion.
// Both FSMs run in parallel: one streams config words, the other streams BBA
// entries (target first, then sources).  cm_config_finished_o is a single-
// cycle pulse when both are done.
//
// NOTES:
//   - WORDS_PER_CONFIG must be a power of 2 and >= 2 (LOG2_WPC used as width).
//   - NUM_ACC must be >= 2 (TGT_ACC_ID_SZ = $clog2(NUM_ACC) defaults to 1).
//   - ADDR_SZ must equal 32 for the AXI write-address arithmetic to be correct.
//   - $clog2 in parameter defaults is supported by Vivado/Quartus in Verilog mode.

module config_manager #(
    // AXI write-slave address decode — fixed at build time, not run-time
    parameter [31:0] TGT_CONFIG_MEM_ADDR      = 32'hffff_ffff,
    parameter [31:0] TGT_CONFIG_MEM_ADDR_MASK  = 32'hffff_0000,
    parameter [31:0] TGT_BUFF_STORE_ADDR       = 32'hffff_ffff,
    parameter [31:0] TGT_BUFF_STORE_ADDR_MASK  = 32'hffff_0000,
    // Memory dimensions
    parameter        CONFIG_MEM_SZ             = 32,
    parameter        WORDS_PER_CONFIG          = 4,
    parameter        NUM_BUFFERS               = 32,
    parameter        BUFF_INDEX_SZ             = 5,
    // Accelerator interface sizing
    parameter        NUM_ACC                   = 2,
    parameter        TGT_ACC_ID_SZ             = $clog2(NUM_ACC),
    parameter        CFG_ID_SZ                 = 5,
    // Data / address widths
    parameter        DATAWORD_SZ               = `DATAWORD_SZ,
    parameter        ADDR_SZ                   = 32,
    parameter        SCH_ENTRY_SZ              = 32,
    // Number of source buffers; target is always sent first (index 0)
    parameter        NUM_SOURCES               = 3
) (
    input  wire                        clk,
    input  wire                        reset,

    // System bus — AXI write slave (populates config and BBA tables)
    input  wire                        cm_sys_req_i,
    output wire                        cm_sys_ack_o,
    input  wire [ADDR_SZ-1:0]          cm_sys_addr_i,
    input  wire [DATAWORD_SZ-1:0]      cm_sys_data_i,

    // Flow control from scheduler
    input  wire                        start_new_block_i,
    input  wire [TGT_ACC_ID_SZ-1:0]    target_acc_i,
    input  wire [CFG_ID_SZ-1:0]        config_id_i,
    input  wire [SCH_ENTRY_SZ-1:0]     buffer_info_i,

    // One-cycle pulse when both FSMs have finished for this block
    output wire                        cm_config_finished_o,

    // High while either FSM is mid-push. Scheduler back-pressures dispatch
    // against this so a second start_new_block_i pulse can't be silently
    // dropped during an in-flight push.
    output wire                        cm_busy_o,

    // Config memory — read port
    output wire                        cfg_mem_rd_o,
    input  wire                        cfg_mem_wait_i,
    output wire [ADDR_SZ-1:0]          cfg_mem_addr_o,
    input  wire [DATAWORD_SZ-1:0]      cfg_mem_data_i,

    // Config memory — write port (AXI slave path)
    output wire                        cfg_mem_wr_o,
    output wire [ADDR_SZ-1:0]          cfg_mem_wr_addr_o,
    output wire [DATAWORD_SZ-1:0]      cfg_mem_wr_data_o,

    // Buffer base-address memory — read port
    output wire                        bba_mem_rd_o,
    input  wire                        bba_mem_wait_i,
    output wire [ADDR_SZ-1:0]          bba_mem_addr_o,
    input  wire [DATAWORD_SZ-1:0]      bba_mem_data_i,

    // Buffer base-address memory — write port (AXI slave path)
    output wire                        bba_mem_wr_o,
    output wire [ADDR_SZ-1:0]          bba_mem_wr_addr_o,
    output wire [DATAWORD_SZ-1:0]      bba_mem_wr_data_o,

    // Accelerator config-word interface
    output wire [TGT_ACC_ID_SZ-1:0]    cm_tgt_acc_o,
    output wire [NUM_ACC-1:0]          cm_config_wr_o,
    input  wire [NUM_ACC-1:0]          cm_config_wait_i,
    output wire [DATAWORD_SZ-1:0]      cm_config_data_o,

    // Accelerator buffer-base-address interface
    output wire [NUM_ACC-1:0]          cm_buff_base_wr_o,
    input  wire [NUM_ACC-1:0]          cm_buff_base_wait_i,
    output wire [DATAWORD_SZ-1:0]      cm_buff_base_data_o
);

// ─── Derived constants ────────────────────────────────────────────────────────
localparam LOG2_WPC      = $clog2(WORDS_PER_CONFIG);
localparam NUM_BBA_SENDS = NUM_SOURCES + 1;       // target + sources
localparam BBA_CNT_SZ    = $clog2(NUM_BBA_SENDS);

// Bit positions of buffer IDs packed into buffer_info_i (mirrors scheduler)
localparam SBUFF1_MSB = BUFF_INDEX_SZ - 1;
localparam SBUFF2_LSB = BUFF_INDEX_SZ;
localparam SBUFF2_MSB = 2*BUFF_INDEX_SZ - 1;
localparam SBUFF3_LSB = 2*BUFF_INDEX_SZ;
localparam SBUFF3_MSB = 3*BUFF_INDEX_SZ - 1;
localparam TBUFF_LSB  = 3*BUFF_INDEX_SZ;
localparam TBUFF_MSB  = 4*BUFF_INDEX_SZ - 1;

// FSM state encodings — shared between both FSMs
localparam FSM_IDLE      = 2'd0;
localparam FSM_MEM_RD    = 2'd1;
localparam FSM_WAIT_DATA = 2'd2;
localparam FSM_ACC_WR    = 2'd3;

// ─── Latch scheduler inputs on start_new_block_i ──────────────────────────────
reg [TGT_ACC_ID_SZ-1:0]  tgt_acc_r;
reg [CFG_ID_SZ-1:0]      config_id_r;
reg [BUFF_INDEX_SZ-1:0]  src1_buff_r;
reg [BUFF_INDEX_SZ-1:0]  src2_buff_r;
reg [BUFF_INDEX_SZ-1:0]  src3_buff_r;
reg [BUFF_INDEX_SZ-1:0]  tgt_buff_r;

always @(posedge clk or posedge reset) begin
    if (reset) begin
        tgt_acc_r   <= {TGT_ACC_ID_SZ{1'b0}};
        config_id_r <= {CFG_ID_SZ{1'b0}};
        src1_buff_r <= {BUFF_INDEX_SZ{1'b0}};
        src2_buff_r <= {BUFF_INDEX_SZ{1'b0}};
        src3_buff_r <= {BUFF_INDEX_SZ{1'b0}};
        tgt_buff_r  <= {BUFF_INDEX_SZ{1'b0}};
    end else if (start_new_block_i) begin
        tgt_acc_r   <= target_acc_i;
        config_id_r <= config_id_i;
        src1_buff_r <= buffer_info_i[SBUFF1_MSB:0];
        src2_buff_r <= buffer_info_i[SBUFF2_MSB:SBUFF2_LSB];
        src3_buff_r <= buffer_info_i[SBUFF3_MSB:SBUFF3_LSB];
        tgt_buff_r  <= buffer_info_i[TBUFF_MSB:TBUFF_LSB];
    end
end

// ─── System bus address decode and write routing ──────────────────────────────
wire cfg_addr_match;
wire bba_addr_match;

assign cfg_addr_match = ((cm_sys_addr_i & TGT_CONFIG_MEM_ADDR_MASK) ==
                          (TGT_CONFIG_MEM_ADDR & TGT_CONFIG_MEM_ADDR_MASK));
assign bba_addr_match = ((cm_sys_addr_i & TGT_BUFF_STORE_ADDR_MASK) ==
                          (TGT_BUFF_STORE_ADDR & TGT_BUFF_STORE_ADDR_MASK));

assign cm_sys_ack_o      = cm_sys_req_i & (cfg_addr_match | bba_addr_match);

assign cfg_mem_wr_o      = cm_sys_req_i & cfg_addr_match;
assign cfg_mem_wr_addr_o = (cm_sys_addr_i & ~TGT_CONFIG_MEM_ADDR_MASK) >> 2;
assign cfg_mem_wr_data_o = cm_sys_data_i;

assign bba_mem_wr_o      = cm_sys_req_i & bba_addr_match;
assign bba_mem_wr_addr_o = (cm_sys_addr_i & ~TGT_BUFF_STORE_ADDR_MASK) >> 2;
assign bba_mem_wr_data_o = cm_sys_data_i;

// ─── Config parameters FSM ────────────────────────────────────────────────────
// Streams WORDS_PER_CONFIG words from config_mem starting at address
// (config_id * WORDS_PER_CONFIG) to the target accelerator's config registers.

reg [1:0]             cfg_state_r;
reg [LOG2_WPC-1:0]    cfg_word_cnt_r;
reg [DATAWORD_SZ-1:0] cfg_data_r;
reg                   cfg_done_r;

always @(posedge clk or posedge reset) begin
    if (reset) begin
        cfg_state_r    <= FSM_IDLE;
        cfg_word_cnt_r <= {LOG2_WPC{1'b0}};
        cfg_data_r     <= {DATAWORD_SZ{1'b0}};
        cfg_done_r     <= 1'b0;
    end else begin
        case (cfg_state_r)
            FSM_IDLE: begin
                if (start_new_block_i) begin
                    cfg_word_cnt_r <= {LOG2_WPC{1'b0}};
                    cfg_done_r     <= 1'b0;
                    cfg_state_r    <= FSM_MEM_RD;
                end
            end
            FSM_MEM_RD: begin
                if (!cfg_mem_wait_i)
                    cfg_state_r <= FSM_WAIT_DATA;
            end
            FSM_WAIT_DATA: begin
                cfg_data_r  <= cfg_mem_data_i;
                cfg_state_r <= FSM_ACC_WR;
            end
            FSM_ACC_WR: begin
                if (!cm_config_wait_i[tgt_acc_r]) begin
                    if (cfg_word_cnt_r == WORDS_PER_CONFIG - 1) begin
                        cfg_done_r  <= 1'b1;
                        cfg_state_r <= FSM_IDLE;
                    end else begin
                        cfg_word_cnt_r <= cfg_word_cnt_r + 1'b1;
                        cfg_state_r    <= FSM_MEM_RD;
                    end
                end
            end
            default: cfg_state_r <= FSM_IDLE;
        endcase
    end
end

// Read address = config_id concatenated with word counter (= config_id*WPC + cnt)
assign cfg_mem_rd_o   = (cfg_state_r == FSM_MEM_RD);
assign cfg_mem_addr_o = {{(ADDR_SZ-CFG_ID_SZ-LOG2_WPC){1'b0}}, config_id_r, cfg_word_cnt_r};

// ─── BBA FSM ──────────────────────────────────────────────────────────────────
// Streams NUM_BBA_SENDS base addresses from bba_mem to the target accelerator:
// index 0 = target buffer, 1..NUM_SOURCES = source buffers.

reg [1:0]             bba_state_r;
reg [BBA_CNT_SZ-1:0]  bba_send_cnt_r;
reg [DATAWORD_SZ-1:0] bba_data_r;
reg                   bba_done_r;

// Mux buffer ID from send counter — drives bba_mem read address
reg [BUFF_INDEX_SZ-1:0] bba_buff_id_w;
always @(*) begin
    case (bba_send_cnt_r)
        2'd0:    bba_buff_id_w = tgt_buff_r;
        2'd1:    bba_buff_id_w = src1_buff_r;
        2'd2:    bba_buff_id_w = src2_buff_r;
        default: bba_buff_id_w = src3_buff_r;
    endcase
end

always @(posedge clk or posedge reset) begin
    if (reset) begin
        bba_state_r    <= FSM_IDLE;
        bba_send_cnt_r <= {BBA_CNT_SZ{1'b0}};
        bba_data_r     <= {DATAWORD_SZ{1'b0}};
        bba_done_r     <= 1'b0;
    end else begin
        case (bba_state_r)
            FSM_IDLE: begin
                if (start_new_block_i) begin
                    bba_send_cnt_r <= {BBA_CNT_SZ{1'b0}};
                    bba_done_r     <= 1'b0;
                    bba_state_r    <= FSM_MEM_RD;
                end
            end
            FSM_MEM_RD: begin
                if (!bba_mem_wait_i)
                    bba_state_r <= FSM_WAIT_DATA;
            end
            FSM_WAIT_DATA: begin
                bba_data_r  <= bba_mem_data_i;
                bba_state_r <= FSM_ACC_WR;
            end
            FSM_ACC_WR: begin
                if (!cm_buff_base_wait_i[tgt_acc_r]) begin
                    if (bba_send_cnt_r == NUM_BBA_SENDS - 1) begin
                        bba_done_r  <= 1'b1;
                        bba_state_r <= FSM_IDLE;
                    end else begin
                        bba_send_cnt_r <= bba_send_cnt_r + 1'b1;
                        bba_state_r    <= FSM_MEM_RD;
                    end
                end
            end
            default: bba_state_r <= FSM_IDLE;
        endcase
    end
end

// Read address = buffer ID used as direct index into bba_mem
assign bba_mem_rd_o  = (bba_state_r == FSM_MEM_RD);
assign bba_mem_addr_o = {{(ADDR_SZ-BUFF_INDEX_SZ){1'b0}}, bba_buff_id_w};

// ─── Completion pulse — one cycle on rising edge of (cfg_done & bba_done) ─────
wire both_done;
reg  both_done_prev_r;

assign both_done = cfg_done_r & bba_done_r;

always @(posedge clk or posedge reset) begin
    if (reset)
        both_done_prev_r <= 1'b0;
    else
        both_done_prev_r <= both_done;
end

assign cm_config_finished_o = both_done & ~both_done_prev_r;

// Busy while either FSM is not in IDLE. Used by the scheduler as
// back-pressure on `dispatch_to_acc_o` — a second start_new_block_i
// pulse must not arrive while we're still pushing the previous one.
assign cm_busy_o = (cfg_state_r != FSM_IDLE) | (bba_state_r != FSM_IDLE);

// ─── Static output assignments ────────────────────────────────────────────────
assign cm_tgt_acc_o        = tgt_acc_r;
assign cm_config_data_o    = cfg_data_r;
assign cm_buff_base_data_o = bba_data_r;

// One-hot write strobes: bit [tgt_acc_r] asserted in the relevant ACC_WR state
assign cm_config_wr_o    = (cfg_state_r == FSM_ACC_WR)
                           ? ({{(NUM_ACC-1){1'b0}}, 1'b1} << tgt_acc_r)
                           : {NUM_ACC{1'b0}};

assign cm_buff_base_wr_o = (bba_state_r == FSM_ACC_WR)
                           ? ({{(NUM_ACC-1){1'b0}}, 1'b1} << tgt_acc_r)
                           : {NUM_ACC{1'b0}};

endmodule
