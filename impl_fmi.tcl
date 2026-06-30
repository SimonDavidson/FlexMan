# impl_fmi.tcl
# Authors: Simon Davidson & Claude   Created: 2026-06-30   Last modified: 2026-06-30
#
# Place & route the FMI design from the 40 ns synthesis checkpoint to obtain the
# TRUE post-route Fmax (= 40 ns - WNS) and real placed/routed resource + timing.
# The critical path is internal fabric (weight_generator -> data-pool BRAM), so
# auto-placed I/O does not affect it.
#   source ~/work/setup_vivado_2025.2.1.sh
#   vivado -mode batch -source impl_fmi.tcl

set root_dir [file dirname [file normalize [info script]]]
set rdir     [file join $root_dir vivado_fmi_relaxed reports]

open_checkpoint $root_dir/vivado_fmi_relaxed/fmi_top_relaxed.runs/synth_1/fmi_top_fpga_wrap.dcp

# Re-apply the 40 ns clock: the project-mode synth checkpoint does not carry the
# create_clock, so without this P&R runs un-timed and WNS reports NA.
read_xdc $root_dir/top_module/fmi_top_fpga_wrap_relaxed.xdc

# Timing-characterisation run: allow auto I/O placement without real pin /
# IOSTANDARD constraints (these are bitstream-stage DRCs, not relevant here).
set_property SEVERITY {Warning} [get_drc_checks NSTD-1]
set_property SEVERITY {Warning} [get_drc_checks UCIO-1]

opt_design
place_design
route_design

file mkdir $rdir
report_utilization    -file $rdir/fmi_postroute_utilization.rpt
report_timing_summary -file $rdir/fmi_postroute_timing.rpt -max_paths 20 -delay_type max
report_drc            -file $rdir/fmi_postroute_drc.rpt

puts "==================== POST-ROUTE TIMING (Fmax = 40 ns - WNS) ===================="
report_timing_summary -max_paths 1 -delay_type max
puts "==================== POST-ROUTE UTILISATION ===================="
report_utilization

write_checkpoint -force $root_dir/vivado_fmi_relaxed/fmi_postroute.dcp
puts "DONE post-route."
exit 0
