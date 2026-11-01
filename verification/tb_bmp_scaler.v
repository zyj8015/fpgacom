`timescale 1ns/1ps
// Optional USER-RUN behavioral test. Not a synthesis or board timing check.
module tb_bmp_scaler #(
    parameter integer OUTPUT_WIDTH=640,
    parameter integer OUTPUT_HEIGHT=480,
    parameter integer FILL_MODE=0
);
reg clk=0, rst=1, start=0, valid=0;
wire ready, out_valid, done;
reg space_ready=0,space_last=0;
integer space_cycle=0;
// Exercise long stalls spanning multiple emit periods. Registered writes may
// arrive one clock after permission, as in the real FIFO with reserved space.
always @(negedge clk) begin
    space_cycle=space_cycle+1;
    space_ready=(space_cycle%521)<237;
end
always @(posedge clk) begin
    if(!rst && out_valid && !space_last) $fatal(1,"write issued without space permission");
    space_last<=space_ready;
end
always #5 clk=~clk;
reg [11:0] width;
reg [10:0] height;
reg [12:0] stride;
reg [7:0] data;
wire [11:0] actual_crop_width,actual_crop_height;
wire [23:0] rgb;
album_bmp_scaler #(.PIXEL_PERIOD(4),.OUTPUT_WIDTH(OUTPUT_WIDTH),.OUTPUT_HEIGHT(OUTPUT_HEIGHT),
    .FILL_MODE(FILL_MODE),.WRITE_BACKPRESSURE(1)) dut(
    .clk(clk),.rst(rst),.start(start),.source_width(width),.source_height(height),
    .row_stride(stride),.byte_data(data),.byte_valid(valid),.byte_ready(ready),
    .pixel_data(rgb),.pixel_valid(out_valid),.done(done),.pixel_ready(space_ready),
    .crop_width(actual_crop_width),.crop_height(actual_crop_height));
integer dw,dh,lm,bm,count=0,ox,oy,sx,sy;
integer expected_width,expected_height,expected_stride;
integer cw,ch,crop_left,crop_bottom;
integer x,y,p;
reg [23:0] expected;
function [23:0] color;
    input integer xx,yy;
    begin color={8'(yy),8'(xx),8'(xx+yy)}; end
endfunction
always @(posedge clk) if(!rst && out_valid) begin
    ox=count%OUTPUT_WIDTH; oy=count/OUTPUT_WIDTH;
    expected=0;
    if(ox>=lm && ox<lm+dw && oy>=bm && oy<bm+dh) begin
        sx=crop_left+((ox-lm)*cw)/dw; sy=crop_bottom+((oy-bm)*ch)/dh;
        expected=color(sx,sy);
    end
    if(rgb!==expected || count>=OUTPUT_WIDTH*OUTPUT_HEIGHT)
        $fatal(1,"pixel %0d got %h expected %h",count,rgb,expected);
    count=count+1;
end
task send;
    input [7:0] value;
    begin
        @(negedge clk);
        while(!ready && !done) @(negedge clk);
        if(!done) begin
            data=value; valid=1;
            @(negedge clk); valid=0;
        end
    end
endtask
task check_image;
    input integer w,h;
    reg [23:0] c;
    begin
        // Consecutive frames restart through start, without resetting DUT.
        @(negedge clk); valid=0;
        width=w; height=h; stride=(w*3+3)&~3;
        expected_width=w; expected_height=h; expected_stride=stride;
        cw=w; ch=h; crop_left=0; crop_bottom=0;
        if(FILL_MODE!=0) begin
            dw=OUTPUT_WIDTH; dh=OUTPUT_HEIGHT;
            if(w*OUTPUT_HEIGHT>=h*OUTPUT_WIDTH) cw=h*OUTPUT_WIDTH/OUTPUT_HEIGHT;
            else ch=w*OUTPUT_HEIGHT/OUTPUT_WIDTH;
            crop_left=(w-cw)/2; crop_bottom=(h-ch)/2;
        end else if(w*OUTPUT_HEIGHT>=h*OUTPUT_WIDTH) begin dw=OUTPUT_WIDTH; dh=(h*OUTPUT_WIDTH)/w; end
        else begin dh=OUTPUT_HEIGHT; dw=(w*OUTPUT_HEIGHT)/h; end
        lm=(OUTPUT_WIDTH-dw)/2; bm=(OUTPUT_HEIGHT-dh)/2; count=0; rst=0; start=1;
        @(negedge clk); start=0;
        // Header bus may change after start; this frame must use its snapshot.
        width=0; height=0; stride=0;
        for(y=0;y<h;y=y+1) begin
            for(x=0;x<w;x=x+1) if(!done) begin
                c=color(x,y); send(c[7:0]); send(c[15:8]); send(c[23:16]);
            end
            for(p=w*3;p<expected_stride;p=p+1) if(!done) send(8'ha5);
        end
        wait(done); repeat(3) @(negedge clk);
        if(count!=OUTPUT_WIDTH*OUTPUT_HEIGHT) $fatal(1,"wrong output count %0d",count);
        if(actual_crop_width!=cw || actual_crop_height!=ch) $fatal(1,"crop geometry mismatch");
        $display("PASS %0dx%0d => box %0dx%0d, padding %0d",w,h,dw,dh,expected_stride-w*3);
    end
endtask
initial begin
    repeat(4) @(negedge clk); rst=0;
    check_image(640,480);
    check_image(800,600);
    check_image(1280,720);
    check_image(1280,768);
    check_image(1600,800);
    check_image(1920,1080);
    check_image(1717,969);
    check_image(1722,967);
    check_image(1723,967);
    check_image(640,1080);
    $display("PASS all scaler cases"); $finish;
end
initial begin repeat(10) #1000000000; $fatal(1,"timeout"); end
endmodule
