##############################################################################
# FlexMan FPGA Wrapper — Xilinx xc7s50fgga484-1 Constraints
# top_module/flexman_fpga_wrap.xdc
#
# NOTE ON I/O COUNT:
#   flexman_fpga_wrap exposes 292 pins (clk + control + 32-bit AXI bus +
#   3× 32-bit spike read-back buses + program-write port).  The xc7s50fgga484-1
#   has 250 user I/O, so not all ports can be brought to package pins
#   simultaneously.  Typical approaches:
#     (a) Connect the AXI, program-write, and spike buses to an on-chip
#         MicroBlaze or AXI master instead of direct package I/O.
#     (b) Reduce spike read-back to a single muxed port.
#   Leave LOC/PACKAGE_PIN blank for any ports not connected to package pins —
#   Vivado will DRC-error on unplaced I/O during implementation; comment out
#   or remove ports that stay internal to the FPGA fabric.
#
# IOSTANDARD:
#   All assignments below use LVCMOS33.  Change to LVCMOS18 or LVCMOS25 if
#   your board's VCCO on the relevant bank is 1.8V or 2.5V.
#
# USAGE:
#   Fill in each PACKAGE_PIN value from the xc7s50fgga484-1 package pinout
#   (UG475 / Vivado "Package Pins" tab).  Remove the leading '#' on the
#   set_property lines as you assign each pin.
##############################################################################

# ── Clock ─────────────────────────────────────────────────────────────────────
# Replace <CLK_PIN> with your board oscillator's package pin.
# Adjust -period to match your clock frequency (10.000 ns = 100 MHz).
set_property -dict {PACKAGE_PIN <CLK_PIN> IOSTANDARD LVCMOS33} [get_ports clk]
create_clock -period 10.000 -name sys_clk -waveform {0.000 5.000} [get_ports clk]

# ── Resets and test control ───────────────────────────────────────────────────
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports reset]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports test_stall_pipe]

# Treat reset as asynchronous (driven by a push-button or host GPIO).
set_false_path -from [get_ports reset]

# ── AXI host bus (98 pins) ────────────────────────────────────────────────────
# In most deployments these connect to an on-chip AXI master, not package pins.
# Uncomment and assign if you are driving them from external logic.
#
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports sys_req_i]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports sys_ack_o]
#
# sys_addr_i[31:0] — 32 pins
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {sys_addr_i[31]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {sys_addr_i[30]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {sys_addr_i[29]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {sys_addr_i[28]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {sys_addr_i[27]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {sys_addr_i[26]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {sys_addr_i[25]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {sys_addr_i[24]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {sys_addr_i[23]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {sys_addr_i[22]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {sys_addr_i[21]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {sys_addr_i[20]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {sys_addr_i[19]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {sys_addr_i[18]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {sys_addr_i[17]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {sys_addr_i[16]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {sys_addr_i[15]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {sys_addr_i[14]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {sys_addr_i[13]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {sys_addr_i[12]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {sys_addr_i[11]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {sys_addr_i[10]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {sys_addr_i[9]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {sys_addr_i[8]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {sys_addr_i[7]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {sys_addr_i[6]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {sys_addr_i[5]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {sys_addr_i[4]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {sys_addr_i[3]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {sys_addr_i[2]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {sys_addr_i[1]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {sys_addr_i[0]}]
#
# sys_data_i[31:0] — 32 pins
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {sys_data_i[31]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {sys_data_i[30]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {sys_data_i[29]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {sys_data_i[28]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {sys_data_i[27]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {sys_data_i[26]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {sys_data_i[25]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {sys_data_i[24]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {sys_data_i[23]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {sys_data_i[22]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {sys_data_i[21]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {sys_data_i[20]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {sys_data_i[19]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {sys_data_i[18]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {sys_data_i[17]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {sys_data_i[16]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {sys_data_i[15]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {sys_data_i[14]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {sys_data_i[13]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {sys_data_i[12]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {sys_data_i[11]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {sys_data_i[10]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {sys_data_i[9]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {sys_data_i[8]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {sys_data_i[7]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {sys_data_i[6]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {sys_data_i[5]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {sys_data_i[4]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {sys_data_i[3]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {sys_data_i[2]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {sys_data_i[1]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {sys_data_i[0]}]
#
# sys_data_o[31:0] — 32 pins
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {sys_data_o[31]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {sys_data_o[30]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {sys_data_o[29]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {sys_data_o[28]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {sys_data_o[27]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {sys_data_o[26]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {sys_data_o[25]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {sys_data_o[24]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {sys_data_o[23]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {sys_data_o[22]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {sys_data_o[21]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {sys_data_o[20]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {sys_data_o[19]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {sys_data_o[18]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {sys_data_o[17]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {sys_data_o[16]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {sys_data_o[15]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {sys_data_o[14]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {sys_data_o[13]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {sys_data_o[12]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {sys_data_o[11]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {sys_data_o[10]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {sys_data_o[9]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {sys_data_o[8]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {sys_data_o[7]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {sys_data_o[6]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {sys_data_o[5]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {sys_data_o[4]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {sys_data_o[3]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {sys_data_o[2]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {sys_data_o[1]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {sys_data_o[0]}]

# ── Program control (11 pins) ─────────────────────────────────────────────────
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports start_program_i]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {program_addr_i[9]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {program_addr_i[8]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {program_addr_i[7]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {program_addr_i[6]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {program_addr_i[5]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {program_addr_i[4]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {program_addr_i[3]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {program_addr_i[2]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {program_addr_i[1]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {program_addr_i[0]}]

# ── Program memory write port (43 pins) ───────────────────────────────────────
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports prog_wr_en_i]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {prog_wr_addr_i[9]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {prog_wr_addr_i[8]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {prog_wr_addr_i[7]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {prog_wr_addr_i[6]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {prog_wr_addr_i[5]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {prog_wr_addr_i[4]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {prog_wr_addr_i[3]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {prog_wr_addr_i[2]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {prog_wr_addr_i[1]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {prog_wr_addr_i[0]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {prog_wr_data_i[31]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {prog_wr_data_i[30]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {prog_wr_data_i[29]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {prog_wr_data_i[28]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {prog_wr_data_i[27]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {prog_wr_data_i[26]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {prog_wr_data_i[25]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {prog_wr_data_i[24]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {prog_wr_data_i[23]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {prog_wr_data_i[22]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {prog_wr_data_i[21]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {prog_wr_data_i[20]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {prog_wr_data_i[19]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {prog_wr_data_i[18]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {prog_wr_data_i[17]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {prog_wr_data_i[16]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {prog_wr_data_i[15]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {prog_wr_data_i[14]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {prog_wr_data_i[13]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {prog_wr_data_i[12]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {prog_wr_data_i[11]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {prog_wr_data_i[10]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {prog_wr_data_i[9]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {prog_wr_data_i[8]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {prog_wr_data_i[7]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {prog_wr_data_i[6]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {prog_wr_data_i[5]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {prog_wr_data_i[4]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {prog_wr_data_i[3]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {prog_wr_data_i[2]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {prog_wr_data_i[1]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {prog_wr_data_i[0]}]

# ── Buffer pre-fill and NXT pulses (9 pins) ───────────────────────────────────
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports mark_buff_as_full_i]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {full_buff_id_i[3]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {full_buff_id_i[2]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {full_buff_id_i[1]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {full_buff_id_i[0]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {full_buff_usage_i[2]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {full_buff_usage_i[1]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {full_buff_usage_i[0]}]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports nxt_input_pulse_o]
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports nxt_output_pulse_o]

# ── Config manager status (1 pin) ─────────────────────────────────────────────
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports cm_config_finished_o]

# ── Spike read-back (126 pins across 3 accelerators) ─────────────────────────
# At ACC_MEM_DEPTH=1024 the address buses are [9:0].
# If ACC_MEM_DEPTH is changed, adjust the bit indices below accordingly.
#
# snnAcc0 spike
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {s0_spike_rd_addr_i[9]}]
#  ... (bits [8:0] follow same pattern)
#set_property -dict {PACKAGE_PIN <PIN> IOSTANDARD LVCMOS33} [get_ports {s0_spike_rd_data_o[31]}]
#  ... (bits [30:0] follow same pattern)
#
# snnAcc1 spike  (s1_spike_rd_addr_i / s1_spike_rd_data_o — same pattern)
# annAcc  spike  (a0_spike_rd_addr_i / a0_spike_rd_data_o — same pattern)

# ── Timing constraints ────────────────────────────────────────────────────────
# Input setup/hold budget relative to sys_clk.
# 2.0 ns max / 0.5 ns min is a conservative starting point for 100 MHz;
# tighten once you know your PCB trace lengths and source register timing.
set_input_delay  -clock sys_clk -max 2.000 \
    [get_ports -filter {DIRECTION == IN && NAME !~ "clk"}]
set_input_delay  -clock sys_clk -min 0.500 \
    [get_ports -filter {DIRECTION == IN && NAME !~ "clk"}]

# Output delay budget.
set_output_delay -clock sys_clk -max 2.000 \
    [get_ports -filter {DIRECTION == OUT}]
set_output_delay -clock sys_clk -min 0.500 \
    [get_ports -filter {DIRECTION == OUT}]

# ── Configuration ─────────────────────────────────────────────────────────────
# CFGBVS must match the voltage on bank 0 (pin VCCO_0).
# Set to VCCO (tracks VCCO_0) and specify the voltage.
# Change to GND / 1.8 if your board powers bank 0 at 1.8V.
set_property CFGBVS        VCCO [current_design]
set_property CONFIG_VOLTAGE 3.3  [current_design]
