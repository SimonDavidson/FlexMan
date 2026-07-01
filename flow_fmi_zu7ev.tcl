# flow_fmi_zu7ev.tcl
# Authors: Simon Davidson & Claude   Created: 2026-06-30   Last modified: 2026-06-30
#
# Cross-part probe: synth + place & route the SAME FMI design (fmi_top_fpga_wrap)
# on xczu7ev-ffvf1516-3-e (Zynq UltraScale+ ZU7EV, fastest -3 grade), to compare
# against the Spartan-7 xc7s50 build. Synth then P&R in ONE process (open_run
# in-memory) so the create_clock constraint stays applied (a project-mode synth
# .dcp does not carry it across a separate open_checkpoint).
#   source ~/work/setup_vivado_2025.2.1.sh
#   vivado -mode batch -source flow_fmi_zu7ev.tcl

set project_name "fmi_top_zu7ev"
set fpga_part    "xczu7ev-ffvf1517-3-e"

set root_dir    [file dirname [file normalize [info script]]]
set project_dir [file join $root_dir vivado_fmi_zu7ev]
set report_dir  [file join $project_dir reports]

create_project $project_name $project_dir -part $fpga_part -force

set src_files [list \
    $root_dir/top_module/fmi_top_fpga_wrap.v \
    $root_dir/top_module/fmi_top.v \
    $root_dir/shared/shared_pool.v \
    $root_dir/shared/bram_sp.v \
    $root_dir/shared/bram_sdp.v \
    $root_dir/shared/bram_dist.v \
    $root_dir/scheduler/scheduler.v \
    $root_dir/scheduler/sch_table.v \
    $root_dir/scheduler/sch_entry.v \
    $root_dir/scheduler/sch_buffer_state.v \
    $root_dir/scheduler/buffer_state_entry.v \
    $root_dir/scheduler/acc_hw_buffer_tracker.v \
    $root_dir/scheduler/prog_cache.v \
    $root_dir/config_manager/config_manager.v \
    $root_dir/fmiSnnAccMC/acc_fmiSnnMC_processor.v \
    $root_dir/fmiSnnAccMC/spike_processing.v \
    $root_dir/fmiSnnAccMC/act_index_generator.v \
    $root_dir/fmiSnnAccMC/weight_generator.v \
    $root_dir/fmiSnnAccMC/syn_curr_update.v \
    $root_dir/fmiSnnAccMC/update_state_for_neuron.v \
    $root_dir/fmiSnnAccMC/neuron_processing.v \
    $root_dir/shared/dataline_cache_with_xy.v \
    $root_dir/shared/slice_and_align.v \
    $root_dir/shared/packer.v \
]

add_files -norecurse $src_files
foreach f $src_files { set_property file_type SystemVerilog [get_files $f] }
set_property include_dirs [list $root_dir/shared] [current_fileset]

set_property top fmi_top_fpga_wrap [current_fileset]
update_compile_order -fileset sources_1

add_files -fileset constrs_1 -norecurse $root_dir/top_module/fmi_top_fpga_wrap_zu7ev.xdc

set_property STEPS.SYNTH_DESIGN.ARGS.FLATTEN_HIERARCHY rebuilt [get_runs synth_1]

# ── Synthesise ────────────────────────────────────────────────────────────────
puts "INFO: synth on $fpga_part (100 MHz probe)..."
launch_runs synth_1 -jobs 4
wait_on_run synth_1
if {[get_property PROGRESS [get_runs synth_1]] ne "100%"} {
    puts "ERROR: synth failed. See $project_dir/${project_name}.runs/synth_1/runme.log"
    exit 1
}

file mkdir $report_dir
# Open the synthesised design in-memory (constraints from constrs_1 apply).
open_run synth_1 -name synth_1
report_utilization -file $report_dir/fmi_zu7ev_synth_utilization.rpt

# ── Place & route in-memory (timing-driven; auto I/O) ─────────────────────────
set_property SEVERITY {Warning} [get_drc_checks NSTD-1]
set_property SEVERITY {Warning} [get_drc_checks UCIO-1]
opt_design
place_design
route_design

report_utilization    -file $report_dir/fmi_zu7ev_postroute_utilization.rpt
report_timing_summary -file $report_dir/fmi_zu7ev_postroute_timing.rpt -max_paths 20 -delay_type max

puts "==================== ZU7EV POST-ROUTE TIMING (Fmax = 10 - WNS) ===================="
report_timing_summary -max_paths 1 -delay_type max
puts "==================== ZU7EV POST-ROUTE UTILISATION ===================="
report_utilization
write_checkpoint -force $project_dir/fmi_zu7ev_postroute.dcp
puts "DONE zu7ev."
exit 0
