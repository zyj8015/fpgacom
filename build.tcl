# TD5.6.2 build in one process; hardware clocks follow the registered PLL/SDC.
cd [file dirname [file normalize [info script]]]
set out [file normalize ../../build/td562]
if {[info exists env(FUNC1_TD_BUILD_OUTPUT)] && $env(FUNC1_TD_BUILD_OUTPUT) ne ""} {
    set out [file normalize $env(FUNC1_TD_BUILD_OUTPUT)]
}
set no_bit 0
if {[info exists env(FUNC1_TD_NO_BIT)] && $env(FUNC1_TD_NO_BIT) eq "1"} {
    set no_bit 1
}
file mkdir $out
open_project func1.al
import_device eagle_s20.db -package EG4S20BG256
elaborate -top top
read_adc ../user_source/constraints_source/pin.adc
optimize_rtl
read_sdc ../user_source/constraints_source/timing.sdc
read_sdc -ip rfifo_32_32_512 ../user_source/hdl_source/IP/afifo_32_16_256.tcl
read_sdc -ip wfifo_32_32_512 ../user_source/hdl_source/IP/afifo_16_32_256.tcl
optimize_gate -maparea $out/func1_gate.area
legalize_phy_inst
set_param place fix_hold on
set_param route fix_hold on
set_param route effort high
set_param route opt_timing high
place
route
export_db $out/func1_pr.db
report_area -io_info -file $out/func1_phy.area
report_timing -mode FINAL -net_info -rpt_autogen true -file $out/func1_phy.timing
set timing_file [open $out/func1_phy.timing r]
set timing_text [read $timing_file]
close $timing_file
if {![regexp {Timing violations: ([0-9]+) setup errors, and ([0-9]+) hold errors} $timing_text unused setup_errors hold_errors]} {
    error "Cannot find setup/hold summary in the final timing report."
}
if {$setup_errors != 0 || $hold_errors != 0} {
    error "Final timing failed: $setup_errors setup and $hold_errors hold errors. See $out/func1_phy.timing"
}
puts "PHOTO_TIMING_PASS setup=0 hold=0"
if {$no_bit} {
    puts "PHOTO_CHECK_COMPLETE $out/func1_phy.timing"
    exit 0
}
export_bid $out/func1_inst.bid
bitgen -bit $out/func1.bit
puts "PHOTO_BUILD_COMPLETE $out/func1.bit"
exit
