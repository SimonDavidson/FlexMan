/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
`include "constants.v"

module hadamard_unit #(
    parameter TGT_ACC_ID           = 0,
    parameter TGT_CONFIG_BASE_ADDR = 16'h1D1D,
    parameter MAX_STREAM_LEN       = 1024,
    parameter ADDR_SZ              = `ADDR_SIZE,
    parameter ACT_SZ               = `POT_OUT_SZ_SZ,
    parameter TGT_ACC_SZ           = `TGT_ACC_SZ,
    parameter SCH_ENTRY_SZ         = `SCH_ENTRY_SZ,
    parameter PIN_BITS             = `PIN_BITS,
    parameter NUM_ELEM_SZ          = 16
)(
    input  wire                    clk,
    input  wire                    reset,

    /* Configuration interface */
    input  wire                    hu_sys_req_i,
    output wire                    hu_sys_ack_o,
    input  wire [31:0]             hu_sys_addr_i,
    input  wire [31:0]             hu_sys_data_i,

    /* Flow control */
    input  wire                    hu_start_new_block_i,
    input  wire [TGT_ACC_SZ-1:0]  hu_target_acc_i,
    input  wire [SCH_ENTRY_SZ-1:0] hu_buffer_info_i,  /* reserved, unused */
    output reg                     hu_acc_busy_o,
    output wire                    hu_acc_finished_o,

    /* Source A memory interface */
    output wire                    src_a_mem_rd_o,
    input  wire                    src_a_mem_wait_i,
    output wire [ADDR_SZ-1:0]      src_a_mem_addr_o,
    input  wire [31:0]             src_a_mem_data_i,

    /* Source B memory interface */
    output wire                    src_b_mem_rd_o,
    input  wire                    src_b_mem_wait_i,
    output wire [ADDR_SZ-1:0]      src_b_mem_addr_o,
    input  wire [31:0]             src_b_mem_data_i,

    /* Source Z memory interface */
    output wire                    src_z_mem_rd_o,
    input  wire                    src_z_mem_wait_i,
    output wire [ADDR_SZ-1:0]      src_z_mem_addr_o,
    input  wire [31:0]             src_z_mem_data_i,

    /* Source R read memory interface */
    output wire                    src_r_mem_rd_o,
    input  wire                    src_r_mem_wait_i,
    output wire [ADDR_SZ-1:0]      src_r_mem_addr_o,
    input  wire [31:0]             src_r_mem_data_i,

    /* Source R write memory interface */
    output wire                    src_r_mem_wr_o,
    output wire [ADDR_SZ-1:0]      src_r_mem_wr_addr_o,
    input  wire                    src_r_mem_wr_wait_i,
    output wire [31:0]             src_r_mem_data_o
);

/* -------------------------------------------------------------------------
 * Internal parameters
 * ---------------------------------------------------------------------- */
localparam IDX_BITS   = $clog2(MAX_STREAM_LEN);
localparam DATA_BITS  = 32;

/* -------------------------------------------------------------------------
 * Config register outputs
 * ---------------------------------------------------------------------- */
wire                    mode_r;
wire [NUM_ELEM_SZ-1:0]  stream_len_r;
wire [ADDR_SZ-1:0]      src_a_base_addr_r, src_b_base_addr_r;
wire [ADDR_SZ-1:0]      src_z_base_addr_r, src_r_base_addr_r;
wire [ACT_SZ-1:0]       src_a_elem_sz_r,   src_b_elem_sz_r;
wire [ACT_SZ-1:0]       src_z_elem_sz_r,   src_r_elem_sz_r;
wire [4:0]              src_a_bin_point_r, src_b_bin_point_r;
wire [4:0]              src_z_bin_point_r, src_r_bin_point_r;

hu_config_regs #(
    .TGT_CONFIG_BASE_ADDR (TGT_CONFIG_BASE_ADDR),
    .ADDR_SZ              (ADDR_SZ),
    .ACT_SZ               (ACT_SZ),
    .NUM_ELEM_SZ          (NUM_ELEM_SZ)
) u_cfg (
    .clk                  (clk),
    .reset                (reset),
    .hu_sys_req_i         (hu_sys_req_i),
    .hu_sys_ack_o         (hu_sys_ack_o),
    .hu_sys_addr_i        (hu_sys_addr_i),
    .hu_sys_data_i        (hu_sys_data_i),
    .mode_r_o             (mode_r),
    .stream_len_r_o       (stream_len_r),
    .src_a_base_addr_r_o  (src_a_base_addr_r),
    .src_a_elem_sz_r_o    (src_a_elem_sz_r),
    .src_a_bin_point_r_o  (src_a_bin_point_r),
    .src_b_base_addr_r_o  (src_b_base_addr_r),
    .src_b_elem_sz_r_o    (src_b_elem_sz_r),
    .src_b_bin_point_r_o  (src_b_bin_point_r),
    .src_z_base_addr_r_o  (src_z_base_addr_r),
    .src_z_elem_sz_r_o    (src_z_elem_sz_r),
    .src_z_bin_point_r_o  (src_z_bin_point_r),
    .src_r_base_addr_r_o  (src_r_base_addr_r),
    .src_r_elem_sz_r_o    (src_r_elem_sz_r),
    .src_r_bin_point_r_o  (src_r_bin_point_r)
);

/* -------------------------------------------------------------------------
 * Start decode
 * ---------------------------------------------------------------------- */
wire start_pulse;
assign start_pulse = hu_start_new_block_i &
                     (hu_target_acc_i == TGT_ACC_ID[TGT_ACC_SZ-1:0]);

always @(posedge clk)
    if (reset)
        hu_acc_busy_o <= 1'b0;
    else if (start_pulse)
        hu_acc_busy_o <= 1'b1;
    else if (hu_acc_finished_o)
        hu_acc_busy_o <= 1'b0;

/* -------------------------------------------------------------------------
 * Stream generator: A
 * ---------------------------------------------------------------------- */
wire                 a_valid, a_last, a_busy;
wire [DATA_BITS-1:0] a_data;
wire [4:0]           a_idx;
wire                 a_taken;

stream_generator #(
    .MAX_STREAM_LEN    (MAX_STREAM_LEN),
    .IN_DATA_BITS      (DATA_BITS),
    .SLICE_SIZE_SZ     (ACT_SZ),
    .SLICE_DATA_IDX_SZ (5),
    .OUT_DATA_BITS     (DATA_BITS)
) u_sg_a (
    .clk          (clk),
    .reset        (reset),
    .start_task_i (start_pulse),
    .stream_len_i (stream_len_r[IDX_BITS:0]),
    .slice_sz_i   (src_a_elem_sz_r),
    .base_addr_i  (src_a_base_addr_r),
    .busy_o       (a_busy),
    .mem_addr_o   (src_a_mem_addr_o),
    .mem_data_i   (src_a_mem_data_i),
    .mem_req_o    (src_a_mem_rd_o),
    .mem_wait_i   (src_a_mem_wait_i),
    .data_valid_o (a_valid),
    .data_o       (a_data),
    .data_idx_o   (a_idx),
    .data_last_o  (a_last),
    .data_taken_i (a_taken)
);

/* -------------------------------------------------------------------------
 * Stream generator: B
 * ---------------------------------------------------------------------- */
wire                 b_valid, b_last, b_busy;
wire [DATA_BITS-1:0] b_data;
wire [4:0]           b_idx;
wire                 b_taken;

stream_generator #(
    .MAX_STREAM_LEN    (MAX_STREAM_LEN),
    .IN_DATA_BITS      (DATA_BITS),
    .SLICE_SIZE_SZ     (ACT_SZ),
    .SLICE_DATA_IDX_SZ (5),
    .OUT_DATA_BITS     (DATA_BITS)
) u_sg_b (
    .clk          (clk),
    .reset        (reset),
    .start_task_i (start_pulse),
    .stream_len_i (stream_len_r[IDX_BITS:0]),
    .slice_sz_i   (src_b_elem_sz_r),
    .base_addr_i  (src_b_base_addr_r),
    .busy_o       (b_busy),
    .mem_addr_o   (src_b_mem_addr_o),
    .mem_data_i   (src_b_mem_data_i),
    .mem_req_o    (src_b_mem_rd_o),
    .mem_wait_i   (src_b_mem_wait_i),
    .data_valid_o (b_valid),
    .data_o       (b_data),
    .data_idx_o   (b_idx),
    .data_last_o  (b_last),
    .data_taken_i (b_taken)
);

/* -------------------------------------------------------------------------
 * Stream generator: Z
 * ---------------------------------------------------------------------- */
wire                 z_valid, z_last, z_busy;
wire [DATA_BITS-1:0] z_data;
wire [4:0]           z_idx;
wire                 z_taken;

stream_generator #(
    .MAX_STREAM_LEN    (MAX_STREAM_LEN),
    .IN_DATA_BITS      (DATA_BITS),
    .SLICE_SIZE_SZ     (ACT_SZ),
    .SLICE_DATA_IDX_SZ (5),
    .OUT_DATA_BITS     (DATA_BITS)
) u_sg_z (
    .clk          (clk),
    .reset        (reset),
    .start_task_i (start_pulse),
    .stream_len_i (stream_len_r[IDX_BITS:0]),
    .slice_sz_i   (src_z_elem_sz_r),
    .base_addr_i  (src_z_base_addr_r),
    .busy_o       (z_busy),
    .mem_addr_o   (src_z_mem_addr_o),
    .mem_data_i   (src_z_mem_data_i),
    .mem_req_o    (src_z_mem_rd_o),
    .mem_wait_i   (src_z_mem_wait_i),
    .data_valid_o (z_valid),
    .data_o       (z_data),
    .data_idx_o   (z_idx),
    .data_last_o  (z_last),
    .data_taken_i (z_taken)
);

/* -------------------------------------------------------------------------
 * Stream generator: R (read, for R_prev in update mode)
 * ---------------------------------------------------------------------- */
wire                 r_valid, r_last, r_busy;
wire [DATA_BITS-1:0] r_data;
wire [4:0]           r_idx;
wire                 r_taken;

stream_generator #(
    .MAX_STREAM_LEN    (MAX_STREAM_LEN),
    .IN_DATA_BITS      (DATA_BITS),
    .SLICE_SIZE_SZ     (ACT_SZ),
    .SLICE_DATA_IDX_SZ (5),
    .OUT_DATA_BITS     (DATA_BITS)
) u_sg_r (
    .clk          (clk),
    .reset        (reset),
    .start_task_i (start_pulse),
    .stream_len_i (stream_len_r[IDX_BITS:0]),
    .slice_sz_i   (src_r_elem_sz_r),
    .base_addr_i  (src_r_base_addr_r),
    .busy_o       (r_busy),
    .mem_addr_o   (src_r_mem_addr_o),
    .mem_data_i   (src_r_mem_data_i),
    .mem_req_o    (src_r_mem_rd_o),
    .mem_wait_i   (src_r_mem_wait_i),
    .data_valid_o (r_valid),
    .data_o       (r_data),
    .data_idx_o   (r_idx),
    .data_last_o  (r_last),
    .data_taken_i (r_taken)
);

/* -------------------------------------------------------------------------
 * Element synchroniser
 * When mode=1: wait for all four streams to have valid data.
 * When mode=0: wait for A, B, Z only; pass zero for R_prev.
 * ---------------------------------------------------------------------- */
wire compute_ready;
wire all_valid;

assign all_valid = a_valid & b_valid & z_valid & (mode_r ? r_valid : 1'b1);

/* Fire take when all required streams are valid and compute is ready */
wire take;
assign take = all_valid & compute_ready;

assign a_taken = take;
assign b_taken = take;
assign z_taken = take;
assign r_taken = take & mode_r;  /* only advance R stream in update mode   */

/* -------------------------------------------------------------------------
 * Compute unit
 * ---------------------------------------------------------------------- */
wire                 pak_write;
wire                 pak_full;
wire [DATA_BITS-1:0] pak_data;
wire [PIN_BITS-1:0]  pak_index;
wire                 pak_last;

/* Element index for packer: use A's index (all streams are synchronised) */
wire [PIN_BITS-1:0] elem_index;
assign elem_index = {{(PIN_BITS-5){1'b0}}, a_idx};

hu_compute #(
    .DATA_BITS  (DATA_BITS),
    .ACT_SZ     (ACT_SZ),
    .BINPT_SZ   (5),
    .PIN_BITS   (PIN_BITS)
) u_compute (
    .clk            (clk),
    .reset          (reset),
    .valid_i        (take),
    .ready_o        (compute_ready),
    .a_i            (a_data),
    .b_i            (b_data),
    .z_i            (z_data),
    .r_prev_i       (mode_r ? r_data : {DATA_BITS{1'b0}}),
    .index_i        (elem_index),
    .last_i         (a_last),
    .bin_point_a_i  (src_a_bin_point_r),
    .bin_point_b_i  (src_b_bin_point_r),
    .bin_point_z_i  (src_z_bin_point_r),
    .bin_point_r_i  (src_r_bin_point_r),
    .elem_sz_ab_i   (src_a_elem_sz_r),
    .elem_sz_z_i    (src_z_elem_sz_r),
    .elem_sz_r_i    (src_r_elem_sz_r),
    .mode_i         (mode_r),
    .pak_write_o    (pak_write),
    .pak_full_i     (pak_full),
    .pak_data_o     (pak_data),
    .pak_index_o    (pak_index),
    .pak_last_o     (pak_last),
    .over_r_o       (),
    .under_r_o      ()
);

/* -------------------------------------------------------------------------
 * Packer
 * ---------------------------------------------------------------------- */
packer u_packer (
    .clk                (clk),
    .reset              (reset),
    .busy_o             (),
    .finish_o           (hu_acc_finished_o),
    .pak_write_i        (pak_write),
    .pak_full_o         (pak_full),
    .pak_colour_i       (1'b0),
    .pak_last_i         (pak_last),
    .pak_index_i        (pak_index),
    .pak_acc_data_i     (pak_data),
    .pot_wr_o           (src_r_mem_wr_o),
    .pot_wait_i         (src_r_mem_wr_wait_i),
    .pot_addr_o         (src_r_mem_wr_addr_o),
    .pot_data_o         (src_r_mem_data_o),
    .pak_colour_sel_o   (),
    .pak_out_sz_i       (src_r_elem_sz_r),
    .pak_colour_bs_o    (),
    .pak_out_base_addr_i(src_r_base_addr_r)
);

endmodule
/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
