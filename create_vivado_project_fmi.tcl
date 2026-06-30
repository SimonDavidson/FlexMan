# create_vivado_project_fmi.tcl
# Authors: Simon Davidson & Claude   Created: 2026-06-30   Last modified: 2026-06-30
#
# Vivado synthesis of the FMI model (fmi_top via fmi_top_fpga_wrap) for
# utilisation + timing on xc7s50fgga484-1 at a 200 MHz target.
#
# Usage:
#   source ~/work/setup_vivado_2025.2.1.sh
#   vivado -mode batch -source create_vivado_project_fmi.tcl
#
# Creates vivado_fmi/, synthesises, and writes:
#   vivado_fmi/reports/fmi_synth_utilization.rpt
#   vivado_fmi/reports/fmi_synth_timing.rpt

set project_name "fmi_top"
set fpga_part    "xc7s50fgga484-1"

set root_dir    [file dirname [file normalize [info script]]]
set project_dir [file join $root_dir vivado_fmi]
set report_dir  [file join $project_dir reports]

create_project $project_name $project_dir -part $fpga_part -force

# ── Source files (fmi_top set from sim_fmi_top.bsh + BRAMs + wrapper) ──────────
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
# `include "../shared/constants.v" resolves relative to each file's own dir.
set_property include_dirs [list $root_dir/shared] [current_fileset]

set_property top fmi_top_fpga_wrap [current_fileset]
update_compile_order -fileset sources_1

add_files -fileset constrs_1 -norecurse $root_dir/top_module/fmi_top_fpga_wrap.xdc

# Optimise across boundaries but keep per-module usage in the report.
set_property STEPS.SYNTH_DESIGN.ARGS.FLATTEN_HIERARCHY rebuilt [get_runs synth_1]

# ── Synthesise ────────────────────────────────────────────────────────────────
puts "INFO: launching synthesis (part $fpga_part, top fmi_top_fpga_wrap, 200 MHz)..."
launch_runs synth_1 -jobs 4
wait_on_run synth_1

set st [get_property STATUS   [get_runs synth_1]]
set pr [get_property PROGRESS [get_runs synth_1]]
puts "INFO: synth_1 STATUS='$st' PROGRESS='$pr'"

if {$pr ne "100%"} {
    puts "ERROR: synthesis did not complete. See vivado_fmi/${project_name}.runs/synth_1/runme.log"
    exit 1
}

# ── Reports ───────────────────────────────────────────────────────────────────
file mkdir $report_dir
open_run synth_1 -name synth_1

report_utilization    -file $report_dir/fmi_synth_utilization.rpt
report_timing_summary -file $report_dir/fmi_synth_timing.rpt -max_paths 10 -delay_type max

puts "==================== UTILISATION (summary) ===================="
report_utilization
puts "==================== TIMING (WNS / Fmax) ====================="
report_timing_summary -max_paths 1 -delay_type max
puts ""
puts "DONE. Reports in: $report_dir"
exit 0
