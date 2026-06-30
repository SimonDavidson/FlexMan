##############################################################################
# FMI FPGA Wrapper — RELAXED clock for true-Fmax probe (xc7s50fgga484-1)
# top_module/fmi_top_fpga_wrap_relaxed.xdc
# Authors: Simon Davidson & Claude   Created: 2026-06-30   Last modified: 2026-06-30
#
# 40 ns (25 MHz) target — close to the ~26 MHz synth estimate so that
# Fmax = 40 ns - WNS reads off precisely. Used only to confirm the real
# achievable frequency; the 200 MHz target lives in fmi_top_fpga_wrap.xdc.
##############################################################################
create_clock -period 40.000 -name clk [get_ports clk]
