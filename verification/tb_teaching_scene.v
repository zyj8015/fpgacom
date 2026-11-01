`timescale 1ns/1ps
// USER-RUN scene test, not a physical timing/resource or HDMI sink check.
module tb_teaching_scene #(parameter FAST_CAPTION_RASTER=0);
reg clk=0,rst=1,vs=0,de=0;
always #5 clk=~clk;
reg [5:0] page=1;
reg epoch=0;
reg [15:0] cs=0;
reg [23:0] rgb=24'h808080;
wire ode,ovs;
wire [23:0] out_rgb;
album_teaching_scene #(.STATUS_X(4),.STATUS_Y(4)) dut(.clk(clk),.rst(rst),.hs_in(1'b0),.vs_in(vs),.de_in(de),
 .rgb_in(rgb),.frame_valid(1'b1),.visible_page(page),.narration({epoch,page,1'b1,cs}),.sidebar_visible(1'b0),
 .hs_out(),.vs_out(ovs),.de_out(ode),.rgb_out(out_rgb));
integer xx,yy,p,white_pixels=0,dark_pixels=0,teacher_pixels=0,check_mode=0;
reg pattern_enable=0;
reg [8:0] de_expected=0,vs_expected=0;
integer xp[0:8],yp[0:8];
reg [23:0] original[0:8];
integer i;
always @(posedge clk) begin
 if(rst) begin de_expected<=0;vs_expected<=0;end
 else begin
  de_expected<={de_expected[7:0],de};vs_expected<={vs_expected[7:0],vs};
  xp[0]<=xx;yp[0]<=yy;original[0]<=rgb;
  for(i=1;i<9;i=i+1) begin xp[i]<=xp[i-1];yp[i]<=yp[i-1];original[i]<=original[i-1];end
  #1;
  if(ode!==de_expected[8] || ovs!==vs_expected[8]) $fatal(1,"nine-stage marker alignment failed");
  if(ode) begin
   if(yp[8]>=672) begin
    if(out_rgb===24'hffffff) white_pixels=white_pixels+1;
    else if(check_mode==0 && out_rgb===24'h202020) dark_pixels=dark_pixels+1;
   end else if(check_mode==0 && out_rgb!==original[8]) $fatal(1,"page %0d changed outside caption",page);
   if(check_mode==1 && xp[8]==600 && yp[8]==160 && out_rgb!==24'h3f3f3f)
    $fatal(1,"real 3x3 blur/dim result mismatch: %h",out_rgb);
   if(check_mode==1 && ((xp[8]==4 && yp[8]==4) || (xp[8]==227 && yp[8]==67)) && out_rgb!==original[8])
    $fatal(1,"HUD mismatch x%0d y%0d got%h expected%h",xp[8],yp[8],out_rgb,original[8]);
   if(check_mode==2 && xp[8]==600 && yp[8]==160 && out_rgb!==original[8])
    $fatal(1,"background did not recover");
   if(check_mode!=0 && xp[8]>=1128 && xp[8]<1272 && yp[8]>=432 && yp[8]<648 &&
      out_rgb!==original[8] && out_rgb!==24'h3f3f3f) teacher_pixels=teacher_pixels+1;
  end
 end
end
task frame_pulse;
 begin
  @(negedge clk);de=0;vs=1;
  repeat(3) @(negedge clk);vs=0;
  repeat(3) @(negedge clk);
 end
endtask
task raster;
 begin
  for(yy=0;yy<720;yy=yy+1) begin
   // Fast unit mode still advances all720row boundaries, but fully scans only
   // the caption rows during caption tests. Focus/blur tests retain full frames.
   for(xx=0;xx<((FAST_CAPTION_RASTER && check_mode==0 && yy<672) ? 1 : 1280);xx=xx+1) begin
    @(negedge clk);de=1;
    rgb=pattern_enable ? ((xx+yy)%2 ? 24'hffffff : 24'd0) : 24'h808080;
    if(pattern_enable && xx>=4 && xx<260 && yy>=4 && yy<68) rgb=24'h4080a0;
   end
   @(negedge clk);de=0;
  end
  repeat(9) @(negedge clk);
 end
endtask
initial begin
 repeat(5) @(negedge clk);rst=0;
 for(p=1;p<=12;p=p+1) begin
  page=p;epoch=~epoch;cs=0;white_pixels=0;dark_pixels=0;
  frame_pulse();
  if(dut.scroll!=0) $fatal(1,"caption phase did not reset on page change");
  // Initial frame has no text: all opening words start beyond the right edge.
  raster();
  if(white_pixels!=0 || dark_pixels<100) $fatal(1,"caption did not start offscreen page %0d",p);
  // Advance a few seconds without raster traffic, then inspect entered text.
  repeat(160) frame_pulse();white_pixels=0;dark_pixels=0;raster();
  if(white_pixels<100 || dark_pixels<100) $fatal(1,"missing Chinese caption page %0d",p);
 end
 page=8;epoch=~epoch;cs=0;frame_pulse();
 cs=650;
 repeat(40) frame_pulse();
 if(dut.focus_mix!=16 || !dut.teacher_shown || dut.entry_age!=32)
  $fatal(1,"focus/teacher entrance did not settle");
 check_mode=1;pattern_enable=1;teacher_pixels=0;raster();
 if(teacher_pixels<100) $fatal(1,"missing teacher overlay");
 cs=900;repeat(20) frame_pulse();
 if(dut.focus_mix!=0 || !dut.teacher_shown) $fatal(1,"focus recovery/teacher retention failed");
 check_mode=2;raster();
 page=2;epoch=~epoch;cs=0;frame_pulse();
 if(dut.teacher_shown || dut.focus_mix!=0 || dut.scroll!=0)
  $fatal(1,"page state leaked");
 // All six specified focus pages have their own window and row count.
 for(p=6;p<=11;p=p+1) begin
  page=p;epoch=~epoch;cs=0;frame_pulse();frame_pulse();
  cs=dut.demo_start+10;repeat(40) frame_pulse();
  if(dut.focus_mix!=16 || !dut.teacher_shown) $fatal(1,"missing focus page%0d",p);
  if((p==6 || p==9) && dut.focus_lines!=3) $fatal(1,"three-line focus truncated");
  cs=dut.demo_end+10;repeat(20) frame_pulse();
  if(dut.focus_mix!=0 || !dut.teacher_shown) $fatal(1,"focus exit page%0d",p);
 end
 $display("TEACHING_SCENE_PASS: 12 Chinese captions, pipeline, focus, real blur, teacher, reset");
 $finish;
end
initial begin #350000000; $fatal(1,"teaching scene timeout");end
endmodule
