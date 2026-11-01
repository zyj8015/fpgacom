`timescale 1ns/1ps
// Optional USER-RUN tests. Timing is shortened through parameters only.
module tb_picture_buttons;
reg clk=0,rst=1,button=1;
always #5 clk=~clk;
wire sp,lp,pp,down;
wire [7:0] settings;
integer shorts=0,longs=0;
album_short_long_button #(.CLK_FREQ_HZ(1000)) key_dut(
    .clk(clk),.rst(rst),.button_in(button),.short_pulse(sp),
    .long_pulse(lp),.press_pulse(pp),.button_down(down));
album_picture_controls #(.CLK_FREQ_HZ(1000)) ui(
    .clk(clk),.rst(rst),.button_in(button),.settings(settings));
always @(posedge clk) begin
    if(sp) shorts=shorts+1;
    if(lp) longs=longs+1;
end
task ticks;
    input integer n;
    begin repeat(n) @(negedge clk); end
endtask
task short_click;
    begin button=0; ticks(80); button=1; ticks(35); end
endtask
initial begin
    ticks(5); rst=0; ticks(5);
    if(settings!=={1'b0,1'b0,3'd2,3'd2}) $fatal(1,"default settings mismatch");
    // Neither initial contact bounce nor release bounce is a second gesture.
    button=0; ticks(4); button=1; ticks(4); button=0; ticks(80);
    button=1; ticks(4); button=0; ticks(4); button=1; ticks(35);
    if(shorts!=1 || longs!=0 || settings[5:3]!=3 || !settings[7])
        $fatal(1,"debounced short/brightness mismatch");
    button=0; ticks(1700);
    if(longs!=1 || shorts!=1 || !settings[6] || settings[5:3]!=3 || !settings[7])
        $fatal(1,"long must toggle once without adjusting or auto repeating");
    button=1; ticks(35);
    if(shorts!=1 || longs!=1) $fatal(1,"release after long emitted short");
    short_click(); short_click(); short_click();
    if(settings[2:0]!=0 || settings[5:3]!=3) $fatal(1,"contrast wrap/parameter isolation failed");
    ticks(1900);
    if(!settings[7]) $fatal(1,"sidebar hidden too early");
    ticks(150);
    if(settings[7] || settings[2:0]!=0) $fatal(1,"sidebar timeout changed settings");
    button=0; ticks(780); // stable hold below 800 ms; release debounce excluded
    button=1; ticks(35);
    if(longs!=1 || shorts!=5 || settings[2:0]!=1) $fatal(1,"short near long threshold failed");
    rst=1; ticks(4); rst=0; ticks(5);
    if(settings!=={1'b0,1'b0,3'd2,3'd2}) $fatal(1,"reset failed");
    $display("PICTURE_BUTTONS_PASS: bounce, one-shot long, short release, levels, timeout");
    $finish;
end
initial begin #200000; $fatal(1,"button test timeout"); end
endmodule

module tb_picture_pixels;
reg clk=0,rst=1,hs=0,vs=0,de=0;
always #5 clk=~clk;
reg [23:0] rgb=0;
reg [7:0] settings={1'b0,1'b0,3'd2,3'd2};
wire [23:0] out_rgb;
wire out_de;
wire [87:0] status={32'd0,1'b1,1'b0,1'b0,4'd0,6'd1,6'd12,16'd1280,16'd720,5'd0};
album_video_osd #(.ENABLE_ADJUST(1)) dut(
    .clk(clk),.rst(rst),.hs_in(hs),.vs_in(vs),.de_in(de),.rgb_in(rgb),
    .status(status),.picture_settings(settings),.hs_out(),.vs_out(),.de_out(out_de),.rgb_out(out_rgb));
integer b,c,v,xx,yy,m,level,number;
reg monitor=0;
reg [1:0] expected_de=0,expected_monitor=0;
reg [23:0] reference_rgb=0,expected_rgb[0:1];
function [7:0] channel;
    input integer value,brightness,contrast;
    integer q;
    begin
        q=(value-128)*(contrast+2)+512+(brightness-2)*128;
        if(q<0) channel=0;
        else if(q>1020) channel=255;
        else channel=q/4;
    end
endfunction
function [23:0] reference;
    input [23:0] value;
    input integer brightness,contrast;
    begin reference={channel(value[23:16],brightness,contrast),
        channel(value[15:8],brightness,contrast),channel(value[7:0],brightness,contrast)}; end
endfunction
// Character selection, glyph lookup and composition are three matched stages.
always @(posedge clk) begin
    if(rst) begin
        expected_de=0; expected_monitor=0;expected_rgb[0]=0;expected_rgb[1]=0;
    end
    else begin
        #1;
        if(out_de!==expected_de[1]) $fatal(1,"DE/RGB pipeline misalignment");
        if(expected_de[1] && expected_monitor[1] && out_rgb!==expected_rgb[1])
            $fatal(1,"pixel got %h expected %h",out_rgb,expected_rgb[1]);
        expected_de={expected_de[0],de};expected_monitor={expected_monitor[0],monitor};
        expected_rgb[1]=expected_rgb[0];expected_rgb[0]=reference_rgb;
    end
end
task new_frame;
    begin
        @(negedge clk); de=0; monitor=0; vs=1;
        repeat(3) @(negedge clk); vs=0;
        repeat(3) @(negedge clk);
    end
endtask
initial begin
    repeat(5) @(negedge clk); rst=0;
    // All 25 parameter pairs, each channel spanning every 8-bit code.
    for(b=0;b<5;b=b+1) for(c=0;c<5;c=c+1) begin
        settings={1'b0,1'b0,3'(b),3'(c)}; new_frame();
        for(v=0;v<256;v=v+1) begin
            @(negedge clk); de=1; monitor=1; rgb={8'(v),8'(255-v),8'(v^85)};
            reference_rgb=reference(rgb,b,c);
            // Mid-frame mailbox changes must not alter the active frame.
            if(v==64) settings={1'b0,1'b0,3'd4,3'd0};
        end
        @(negedge clk); de=0; monitor=0;
        repeat(3) @(negedge clk);
    end
    // Raster enough lines to cover every dot. Check both parameter selectors
    // and every level; status OSD and label glyphs are excluded from this probe.
    for(m=0;m<2;m=m+1) for(level=0;level<5;level=level+1) begin
        settings={1'b1,1'(m),3'(m ? 2 : level),3'(m ? level : 2)}; new_frame();
        for(yy=0;yy<496;yy=yy+1) begin
            for(xx=0;xx<160;xx=xx+1) begin
                @(negedge clk); de=1; rgb=24'h808080; monitor=0;
                number=0;
                case(yy) 276:number=5;328:number=4;380:number=3;432:number=2;484:number=1; endcase
                if(xx==72 && number!=0) begin
                    monitor=1; reference_rgb=number<=level+1 ? 24'h40e0ff : 24'h505860;
                end
                if(xx==144 && yy>=192) begin
                    monitor=1; reference_rgb=reference(rgb,m ? 2 : level,m ? level : 2);
                end
            end
            @(negedge clk); de=0; monitor=0;
        end
    end
    settings={1'b0,1'b0,3'd2,3'd2}; new_frame();
    // After hide, the same left-side coordinates must return to the picture.
    for(yy=0;yy<496;yy=yy+1) begin
        for(xx=0;xx<160;xx=xx+1) begin
            @(negedge clk); de=1; rgb=24'h808080;
            monitor=(xx==72 && yy>=192); reference_rgb=rgb;
        end
        @(negedge clk); de=0; monitor=0;
    end
    repeat(4) @(negedge clk);
    $display("PICTURE_PIXELS_PASS: 25 settings, frame latch, clipping, dot counts, sidebar hide, latency");
    $finish;
end
initial begin #30000000; $fatal(1,"pixel test timeout"); end
endmodule
