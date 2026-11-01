`timescale 1ns/1ps
// Optional user-run test: full active frame, OSD pipeline, coherent status,
// AXIS SOF/TLAST. No SDRAM, PLL analog behavior or HDMI sink is modeled.
module tb_video_720p_osd;
reg clk=0,sd_clk=0,rst=1;
always #6.734 clk=~clk;
always #5 sd_clk=~sd_clk;
wire hs,vs,de,req;
wire ohs,ovs,ode;
wire [23:0] rgb;
wire [87:0] status;
wire user,valid,last;
wire [23:0] axis_rgb;
wire sample_ready,audio_valid,acr_valid;
wire [23:0] audio_left,audio_right;
wire [19:0] acr_cts,acr_n;
reg sample_send=0;
integer sample_phase=0,video_cycles=0,last_acr=-1,acr_packets=0;
album_pcm_hdmi_bridge #(.PIXEL_HZ(74250000)) audio_bridge(
    .sd_clk(sd_clk),.sd_rst(rst),.video_clk(clk),.video_rst(rst),
    .sample_send(sample_send),.sample_data(16'h1000),.sample_last(1'b0),.mute(1'b0),
    .sample_ready(sample_ready),.last_played_pulse(),.audio_valid(audio_valid),
    .audio_left(audio_left),.audio_right(audio_right),
    .acr_valid(acr_valid),.acr_cts(acr_cts),.acr_n(acr_n));
always @(posedge sd_clk) begin
    sample_send<=0;
    if(rst) sample_phase=0;
    else if(sample_phase>=100000000-48000) begin
        sample_phase=sample_phase-(100000000-48000);
        if(!sample_ready) $fatal(1,"audio mailbox busy at next sample");
        sample_send<=1;
    end else sample_phase=sample_phase+48000;
end
video_timing_data #(.H_ACTIVE(1280),.H_FP(110),.H_SYNC(40),.H_BP(220),
    .V_ACTIVE(720),.V_FP(5),.V_SYNC(5),.V_BP(20),.HS_POL(1),.VS_POL(1)) timing(
    .video_clk(clk),.rst(rst),.read_req(req),.read_req_ack(1'b1),.hs(hs),.vs(vs),.de(de));
album_status_cdc #(.WIDTH(88)) cdc(.source_clk(sd_clk),.source_rst(rst),
    .source_data({16'd1719,16'd967,1'b1,1'b1,1'b0,4'd0,6'd1,6'd12,16'd1722,16'd967,5'd0}),
    .dest_clk(clk),.dest_rst(rst),.dest_data(status));
album_video_osd osd(.clk(clk),.rst(rst),.hs_in(hs),.vs_in(vs),.de_in(de),
    .rgb_in(24'h804020),.status(status),.hs_out(ohs),.vs_out(ovs),.de_out(ode),.rgb_out(rgb));
video_rgb_to_axis_640x480 #(.ACTIVE_WIDTH(1280)) axis(
    .I_clk(clk),.I_rst(rst),.I_vs(ovs),.I_de(ode),.I_rgb(rgb),
    .O_video_user(user),.O_video_valid(valid),.O_video_last(last),.O_video_data(axis_rgb));
integer pixels=0,lines=0,x,y;
reg seen_vs=0,checking=0;
always @(posedge clk) if(!rst && osd.de_in && osd.x==16 && osd.frame_status[55]) begin
    case(osd.y)
        16:if(osd.text_line!=="IN  1722x0967   ") $fatal(1,"IN row mismatch");
        32:if(osd.text_line!=="OUT 1280x0720   ") $fatal(1,"OUT row mismatch");
        48:if(osd.text_line!=="PAGE 01/12      ") $fatal(1,"PAGE row mismatch");
        64:if(osd.text_line!=="MODE AUTO       ") $fatal(1,"MODE row mismatch");
    endcase
end
always @(posedge clk) if(!rst) begin
    video_cycles=video_cycles+1;
    if(acr_valid) begin
        if(acr_cts!=74250 || acr_n!=6144) $fatal(1,"720p ACR mismatch");
        if(last_acr>=0 && video_cycles-last_acr!=74250) $fatal(1,"ACR cadence mismatch");
        last_acr=video_cycles; acr_packets=acr_packets+1;
    end
    if(audio_valid && (audio_left!==24'h100000 || audio_right!==24'h100000))
        $fatal(1,"720p audio mailbox data mismatch");
    if(vs) seen_vs=1;
    if(valid) begin
        if(user && seen_vs && !checking) begin checking=1; pixels=0; lines=0; end
        if(checking) begin
            x=pixels%1280; y=pixels/1280;
            if(user !== (pixels==0)) $fatal(1,"SOF mismatch pixel %0d",pixels);
            if(last !== (x==1279)) $fatal(1,"TLAST mismatch x=%0d",x);
            if(last) lines=lines+1;
            if((x<16 || x>=240 || y<16 || y>=80) && axis_rgb!==24'h804020)
                $fatal(1,"RGB outside OSD changed at %0d,%0d",x,y);
            if(x==16 && y==20 && axis_rgb!==24'ha18858) $fatal(1,"OSD border mismatch");
            if(x==20 && y==18 && axis_rgb!==24'h382c22) $fatal(1,"OSD font mismatch");
            if(x==28 && y==18 && axis_rgb!==24'hd4bba0) $fatal(1,"OSD background mismatch");
            pixels=pixels+1;
            if(pixels==921600) begin
                if(acr_packets<10) $fatal(1,"Insufficient ACR checks");
                if(lines!=720 || osd.width_bcd!==16'h1722 || osd.height_bcd!==16'h0967)
                    $fatal(1,"Frame geometry/BCD mismatch");
                if(osd.page_ones!="1" || osd.count_tens!="1" || osd.count_ones!="2")
                    $fatal(1,"Page metadata mismatch");
                $display("VIDEO_720P_OSD_PASS: frame, AXIS markers, overlay and metadata");
                $finish;
            end
        end
    end else if(last || user) $fatal(1,"AXIS markers outside valid");
end
initial begin repeat(8) @(negedge clk); rst=0; end
initial begin #100000000; $fatal(1,"720p test timeout"); end
endmodule
