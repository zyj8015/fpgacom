`timescale 1ns/1ps
// Optional USER-RUN handshake tests; not an SD/SDRAM/HDMI integration test.
module tb_slide_transitions;
reg clk=0,rst=1,vs=0,de=0;
always #5 clk=~clk;
reg [5:0] page=1;
reg [8:0] command=0;
wire [1:0] reply;
wire [23:0] rgb;
integer effect,k;
album_teaching_scene #(.ENABLE_TRANSITIONS(1)) dut(
 .clk(clk),.rst(rst),.hs_in(1'b0),.vs_in(vs),.de_in(de),.rgb_in(24'h408080),
 .frame_valid(1'b1),.visible_page(page),.narration({1'b0,page,1'b0,16'd0}),.sidebar_visible(1'b0),
 .transition_command(command),.transition_reply(reply),.hs_out(),.vs_out(),.de_out(),.rgb_out(rgb));
task frame;
 begin
  @(negedge clk);vs=1;repeat(3) @(negedge clk);vs=0;
  repeat(20) @(negedge clk);
 end
endtask
initial begin
 repeat(5) @(negedge clk);rst=0;frame();repeat(20) frame();
 for(effect=0;effect<4;effect=effect+1) begin
  command={1'b1,~command[7],1'b0,6'(effect+5)};
  frame();
  if(dut.transition_kind!=effect || reply[1]) $fatal(1,"effect mapping/early acknowledgment");
  for(k=0;k<7;k=k+1) begin frame();if(reply[1]) $fatal(1,"covered before full cover");end
  frame();
  if(reply!=={1'b1,command[7]} || dut.transition_level!=16) $fatal(1,"missing covered ticket");
  repeat(3) frame();
  if(dut.transition_level!=16) $fatal(1,"cover lost while waiting for commit");
  page=effect+5;command[8]=0;frame();
  if(reply[1] || dut.transition_state!=3 || dut.transition_level!=16) $fatal(1,"commit reveal mismatch");
  repeat(8) frame();
  if(dut.transition_level!=0 || dut.transition_state!=0) $fatal(1,"manual reveal failed");
 end
 // Cancel and immediately retarget: the earlier ticket must not acknowledge it.
 command={1'b1,~command[7],1'b1,6'd9};frame();repeat(4) frame();
 command[8]=0;frame();
 command={1'b1,~command[7],1'b0,6'd10};frame();
 if(reply[1]) $fatal(1,"stale covered response reused after retarget");
 repeat(20) frame();
 if(reply!=={1'b1,command[7]}) $fatal(1,"retarget did not settle");
 command[8]=0;repeat(24) frame();
 if(reply[1] || dut.transition_level!=0) $fatal(1,"cancel did not restore current page");
 // Full six-bit page identity: page16 must not restart its entrance every VS.
 page=16;frame();repeat(20) frame();
 if(dut.transition_level!=0) $fatal(1,"page16 identity truncated");
 $display("SLIDE_TRANSITIONS_PASS: four effects, cover/commit, tickets, cancel, page16");$finish;
end
initial begin #2000000;$fatal(1,"transition test timeout");end
endmodule

module tb_album_transition_control;
reg clk=0,rst=1,done_level=0,kn=0,kp=0,ka=0;
always #5 clk=~clk;
reg [1:0] reply=0;
wire [8:0] command;
wire [4:0] page;
integer n,starts=0;
sd_card_bmp #(.CLK_FREQ_HZ(1000),.ENABLE_WAV(1),.ENABLE_PREFETCH(1),
 .AUTO_AUDIO_GAP_MS(30),.ENABLE_TRANSITIONS(1)) dut(
 .clk(clk),.rst(rst),.key_next(1'b1),.key_previous(1'b1),.key_auto(1'b1),
 .bmp_width(16'd1280),.bmp_height(16'd720),.write_finish_toggle(1'b0),
 .write_req_ack(1'b0),.write_ready(1'b1),.active_display_buf(2'd0),
 .SD_MISO(1'b1),.pcm_ready(1'b1),.pcm_last_played(1'b0),
 .image_index(page),.transition_command(command),.transition_reply(reply));
initial begin
 force dut.sd_init_done=1'b1;force dut.bmp_ready=1'b0;force dut.scan_done=1'b0;
 force dut.scan_found_valid=1'b0;force dut.img_found_count=6'd3;force dut.catalog_sorted=1'b1;
 force dut.wav_done=done_level;force dut.audio_fault=4'd0;
 force dut.key_next_press=kn;force dut.key_prev_press=kp;force dut.key_auto_press=ka;
end
always @(posedge clk) if(!rst && dut.track_start) starts=starts+1;
task clocks;
 input integer count;begin repeat(count) @(negedge clk);end
endtask
task cache;
 input [4:0] index;begin
  dut.cache_valid=1;dut.cache_idx=index;dut.cache_buf=1;
  dut.cache_track={27'd0,index}+32'd1;
  dut.cache_width=1280;dut.cache_height=720;dut.cache_crop_width=1280;dut.cache_crop_height=720;
 end
endtask
initial begin
 clocks(5);rst=0;clocks(3);
 dut.first_image_committed=1;dut.display_valid=1;dut.img_idx=0;dut.track_number=1;
 cache(1);ka=1;clocks(1);ka=0;clocks(3);done_level=1;
 for(n=0;n<29;n=n+1) begin clocks(1);if(command[8] || page!=0) $fatal(1,"auto gap bypassed");end
 clocks(8);
 if(!command[8] || !command[6] || command[5:0]!=2 || page!=0) $fatal(1,"auto cover request missing");
 reply={1'b1,~command[7]};clocks(5);
 if(page!=0 || starts!=0) $fatal(1,"stale ticket committed a page");
 reply={1'b1,command[7]};clocks(3);
 if(page!=1 || starts!=1 || command[8]) $fatal(1,"matching cover did not commit once");
 // Manual navigation ignores the automatic post-audio pause.
 reply=0;done_level=0;cache(0);kp=1;clocks(1);kp=0;clocks(3);
 if(!command[8] || command[6] || command[5:0]!=1) $fatal(1,"manual navigation delayed by auto gap");
 reply={1'b1,command[7]};clocks(3);
 if(page!=0 || starts!=2 || dut.auto_play_en) $fatal(1,"manual commit/audio/mode mismatch");
 // Video acknowledgment timeout retains the current frame, records a fault.
 reply=0;cache(1);kn=1;clocks(1);kn=0;clocks(3);
 if(!command[8]) $fatal(1,"new manual transition missing");
 dut.load_timeout_cnt=2000;clocks(2);
 if(command[8] || dut.next_req_pending || page!=0 || !dut.prefetch_blocked || dut.fault_code!=1)
  $fatal(1,"cover timeout did not preserve current frame");
 $display("ALBUM_TRANSITION_CONTROL_PASS: auto gap, manual bypass, tickets, audio once, timeout");$finish;
end
initial begin #2000000;$fatal(1,"album transition control timeout");end
endmodule
