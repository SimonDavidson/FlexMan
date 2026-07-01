##############################################################################
# FMI FPGA Wrapper — xczu7ev-ffvf1516-3-e (Zynq UltraScale+ ZU7EV) constraints
# top_module/fmi_top_fpga_wrap_zu7ev.xdc
# Authors: Simon Davidson & Claude   Created: 2026-06-30   Last modified: 2026-06-30
#
# Cross-part comparison vs the Spartan-7 xc7s50 build. Aggressive 10 ns (100 MHz)
# clock so the router pushes hard on the fast -3 UltraScale+ fabric; the true
# achievable Fmax reads off as 10 ns - WNS. Synth/P&R timing probe only — pins
# unplaced (auto-placed for impl; the critical path is internal fabric).
##############################################################################
create_clock -period 10.000 -name clk [get_ports clk]
