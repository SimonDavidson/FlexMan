# create_vivado_project.tcl
# Sets up a Vivado project for FlexMan FPGA synthesis.
#
# Usage (either method):
#   vivado -mode batch -source create_vivado_project.tcl
#   # — or — open Vivado, then in the Tcl console:
#   source /path/to/FlexMan/create_vivado_project.tcl
#
# The project is created in vivado/ relative to this script's location.
# Change fpga_part below to match your target device.

# ── Configuration — edit these to suit your board ─────────────────────────────
set project_name "flexman"
set fpga_part    "xc7s50fgga484-1"    ;# Spartan-7 50T (FGGA484 package)
# Other common 7-series parts:
#   xc7a100tcsg324-1  Artix-7 100T (Nexys A7-100T / Arty A7-100T)
#   xc7a35tcpg236-1   Artix-7 35T  (Arty A7-35T, Basys 3)
#   xc7k325tffg900-2  Kintex-7 325T
#   xc7z020clg484-1   Zynq-7020    (Zedboard, PYNQ-Z2)

# ── Paths ─────────────────────────────────────────────────────────────────────
set root_dir    [file dirname [file normalize [info script]]]
set project_dir [file join $root_dir vivado]

# ── Create project ────────────────────────────────────────────────────────────
create_project $project_name $project_dir -part $fpga_part -force

# ── Source files (same set as elab_fpga_wrap.bsh) ────────────────────────────
set src_files [list \
    $root_dir/top_module/flexman_fpga_wrap.v \
    $root_dir/top_module/flexman.v \
    $root_dir/shared/bram_sp.v \
    $root_dir/shared/bram_sdp.v \
    $root_dir/shared/bram_tdp.v \
    $root_dir/scheduler/scheduler.v \
    $root_dir/scheduler/sch_table.v \
    $root_dir/scheduler/sch_entry.v \
    $root_dir/scheduler/sch_buffer_state.v \
    $root_dir/scheduler/buffer_state_entry.v \
    $root_dir/scheduler/acc_hw_buffer_tracker.v \
    $root_dir/scheduler/prog_cache.v \
    $root_dir/config_manager/config_manager.v \
    $root_dir/snnAcc/acc_snn_processor.v \
    $root_dir/snnAcc/spike_processing.v \
    $root_dir/snnAcc/neuron_processing.v \
    $root_dir/snnAcc/act_index_generator.v \
    $root_dir/snnAcc/weight_generator.v \
    $root_dir/snnAcc/syn_curr_update.v \
    $root_dir/snnAcc/update_state_for_neuron.v \
    $root_dir/snnAcc/dataline_cache_with_xy.v \
    $root_dir/snnAcc/slice_and_align.v \
    $root_dir/snnAcc/packer.v \
    $root_dir/annAcc/acc_snn_processor.v \
    $root_dir/annAcc/spike_processing.v \
    $root_dir/annAcc/neuron_processing.v \
    $root_dir/annAcc/syn_curr_update.v \
    $root_dir/annAcc/update_state_for_neuron.v \
    $root_dir/Hadamard/hadamard_unit.v \
    $root_dir/Hadamard/hu_compute.v \
    $root_dir/Hadamard/hu_config_regs.v \
    $root_dir/Hadamard/stream_generator.v \
    $root_dir/fillUnit/fill_unit.v \
]

add_files -norecurse $src_files

# All files use SystemVerilog syntax (compiled with xrun -sv).
# This enables $clog2, always_ff, etc. in Vivado's parser.
foreach f $src_files {
    set_property file_type SystemVerilog [get_files $f]
}

# `include "../shared/constants.v" resolves relative to each file's directory;
# add shared/ explicitly so Vivado can also find it via include_dirs if needed.
set_property include_dirs [list $root_dir/shared] [current_fileset]

# ── Top module ────────────────────────────────────────────────────────────────
set_property top flexman_fpga_wrap [current_fileset]
update_compile_order -fileset sources_1

# ── Constraints file ─────────────────────────────────────────────────────────
add_files -fileset constrs_1 -norecurse \
    $root_dir/top_module/flexman_fpga_wrap.xdc

# ── Synthesis run ─────────────────────────────────────────────────────────────
# Flatten hierarchy to "rebuilt" so Vivado can optimise across module boundaries
# while still reporting per-module resource usage after synthesis.
set_property STEPS.SYNTH_DESIGN.ARGS.FLATTEN_HIERARCHY rebuilt [get_runs synth_1]

# ── Summary ───────────────────────────────────────────────────────────────────
puts ""
puts "Project created: $project_dir/$project_name.xpr"
puts "Top:  flexman_fpga_wrap"
puts "Part: $fpga_part"
puts ""
puts "Next steps:"
puts "  Synthesise:    launch_runs synth_1 -jobs 4 ; wait_on_run synth_1"
puts "  Implement:     launch_runs impl_1  -jobs 4 ; wait_on_run impl_1"
puts "  Generate bit:  open_run impl_1 ; write_bitstream -force flexman.bit"
