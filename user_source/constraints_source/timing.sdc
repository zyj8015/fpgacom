# TD5.6.2 generated clocks, 720p60 video PLL uses CLKC2 internal feedback.
create_clock -name clk -period 20 -waveform {0 10} [get_ports {clk}]
create_generated_clock -name sd_card_clk -source [get_ports {clk}] -multiply_by 2 [get_pins {sys_pll_m0/pll_inst.clkc[0]}]
create_generated_clock -name ext_mem_clk -source [get_ports {clk}] -multiply_by 2.5 [get_pins {sys_pll_m0/pll_inst.clkc[1]}]
create_generated_clock -name ext_mem_clk_sft -source [get_ports {clk}] -multiply_by 2.5 -invert [get_pins {sys_pll_m0/pll_inst.clkc[2]}]
# TD5.6.2 accepts a decimal multiplier, but multiply/divide options are exclusive.
# 50 MHz * 1.485 = 74.25 MHz; 50 MHz * 7.425 = 371.25 MHz.
create_generated_clock -name video_clk -source [get_ports {clk}] -multiply_by 1.485 [get_pins {video_pll_m0/pll_inst.clkc[0]}]
create_generated_clock -name hdmi_5x_clk -source [get_ports {clk}] -multiply_by 7.425 [get_pins {video_pll_m0/pll_inst.clkc[1]}]
# Separate PLL domains exchange video through the dual-clock FIFO and
# request/ack synchronizers, and audio/status through held-data mailboxes.
# Do not apply this exception within either PLL domain: pixel-to-serial and
# SD-to-SDRAM paths still require normal timing closure.
set_clock_groups -name system_video_cdc -asynchronous -group [get_clocks {sd_card_clk ext_mem_clk ext_mem_clk_sft}] -group [get_clocks {video_clk hdmi_5x_clk}]
# The50MHz control domain only enters video_clk through the three-flop
# asynchronous-assert/synchronous-release video_reset_sync chain (top RTL).
# Its asynchronous reset entrance has no phase-based recovery requirement.
# Local video_reset_sync -> video logic recovery/removal and all video-to-5x
# functional paths remain timed. Review this exception if adding50MHz data
# paths into video; it is justified by the present reset-only crossing.
set_false_path -from [get_clocks {clk}] -to [get_clocks {video_clk}]
# The shared video reset asynchronously enters the HDMI lane serializer reset
# pins. These SR entrances have no meaningful phase relationship; every lane
# D-input/data path remains timed. The physical pin pattern survives TD's
# post-gate net aliasing and prevents its unstable automatic hold repair.
set_false_path -from [get_clocks {video_clk}] -through [get_pins {*u*_lane_lvds_8_1/*.sr}] -to [get_clocks {hdmi_5x_clk}]
# rst_all_reg feeds ONLY the asynchronous reset pins of the three reset
# synchronizer chains in top. Release into SD/memory is through three local
# flip-flops. Exclude this root-reset source only, not other50MHz data and
# not the synchronized reset outputs or functional SD<->memory paths.
# get_regs retains the logical source across optimizer replication.
set_false_path -from [get_regs {rst_all_reg*}] -to [get_clocks {sd_card_clk ext_mem_clk}]
# The memory-domain FIFO clear asserts the generated FIFO's two write-side
# reset synchronizer stages asynchronously. Exclude ONLY their SR entrance;
# raw-clear gating of write_ready and all synchronized release/data paths
# stay timed. Register-to-register exceptions do not select SR in TD5.6.2.
set_false_path -from [get_regs {*frame_fifo_write*/*fifo_aclr*}] -through [get_pins {*write_buf*/*asy_w_rst*.sr}] -to [get_clocks {sd_card_clk}]
