##############################################################################
# FMI FPGA Wrapper — Xilinx xc7s50fgga484-1 Constraints
# top_module/fmi_top_fpga_wrap.xdc
# Authors: Simon Davidson & Claude   Created: 2026-06-30   Last modified: 2026-06-30
#
# SCOPE: synthesis (utilisation + timing) only. No package-pin placement is
# required for synth_design — Vivado only DRC-errors on unplaced I/O during
# IMPLEMENTATION. The only constraint that matters for a synth-stage Fmax read
# is the primary clock below. Pin/IOSTANDARD assignments are left as a commented
# template for the later place-and-route milestone.
##############################################################################

# ── Primary clock: 200 MHz (5.000 ns) target ─────────────────────────────────
create_clock -period 5.000 -name clk [get_ports clk]

# Input/output delays (budget ~2 ns of the 5 ns period to board/IO). Enable for
# I/O timing once real pins are assigned; harmless to leave for synth-stage.
# set_input_delay  -clock clk 2.000 [all_inputs]
# set_output_delay -clock clk 2.000 [all_outputs]

##############################################################################
# Pin placement template (for IMPLEMENTATION, not synthesis) — fill from the
# xc7s50fgga484-1 package pinout and uncomment as pins are assigned. NOTE: the
# wrapper exposes well over the 250 user I/O of this device once the 32-bit
# sys_* bus + loader bus are on package pins, so a real board will drive these
# from an on-chip AXI master / MicroBlaze rather than direct I/O.
##############################################################################
# set_property -dict {PACKAGE_PIN <CLK_PIN>   IOSTANDARD LVCMOS33} [get_ports clk]
# set_property -dict {PACKAGE_PIN <RST_PIN>   IOSTANDARD LVCMOS33} [get_ports reset]
