`timescale 1ns/1ps
// Optional user-run test: synthetic FAT32 sectors, fragmented WAV, real
// 100 MHz source / selectable video clock, exact PCM and playback completion.
module tb_wav_player #(
 parameter [63:0] NAME_TEXT="1       ",
 parameter integer NAME_NUMBER=1,
 parameter integer VIDEO_HZ=25000000,
 parameter integer ENABLE_WAVEFORM=0
);
reg clk=0,video_clk=0,rst=1,start=0,stop=0;
always #5 clk=~clk;
always #(500000000.0/VIDEO_HZ) video_clk=~video_clk;
reg [31:0] number=NAME_NUMBER;
wire req,busy,sample_send,sample_ready,sample_last,last_played,mute,done;
wire [31:0] addr;
wire [15:0] pcm;
wire [3:0] fault;
reg [7:0] data=0;
reg valid=0,ending=0,active=0;
reg [31:0] active_addr=0;
integer pos=0,gap=0,received=0;
wire out_valid;
wire [23:0] left,right;
assign busy=active;
album_wav_player dut(
    .clk(clk),.rst(rst),.start(start),.stop(stop),.track_number(number),
    .fs_valid(1'b1),.fs_data_start(32'd100),.fs_fat_start(32'd10),
    .fs_root_cluster(32'd2),.fs_max_cluster(32'd1000),.fs_volume_end(32'd2000),
    .fs_fat_sectors(32'd8),.fs_cluster_shift(4'd0),.fs_spc(8'd1),
    .sector_req(req),.sector_addr(addr),.sector_data(data),.sector_valid(valid),
    .sector_end(ending),.sector_busy(busy),.sample_send(sample_send),
    .sample_data(pcm),.sample_last(sample_last),.sample_ready(sample_ready),
    .last_played_pulse(last_played),.mute(mute),.audio_done(done),.fault(fault),.played_cs()
);
album_pcm_hdmi_bridge #(.PIXEL_HZ(VIDEO_HZ)) bridge(
    .sd_clk(clk),.sd_rst(rst),.video_clk(video_clk),.video_rst(rst),
    .sample_send(sample_send),.sample_data(pcm),.sample_last(sample_last),
    .mute(mute),.sample_ready(sample_ready),.last_played_pulse(last_played),
    .audio_valid(out_valid),.audio_left(left),.audio_right(right),
    .acr_valid(),.acr_cts(),.acr_n()
);
generate if(ENABLE_WAVEFORM!=0) begin:g_waveform
    album_audio_waveform waveform(
        .clk(video_clk),.rst(rst),.hs_in(1'b0),.vs_in(1'b0),.de_in(1'b0),.rgb_in(24'd0),
        .frame_valid(1'b1),.visible_page(6'd1),.narration_epoch(start),
        .sample_valid(out_valid),.sample_data(left[23:8]),
        .hs_out(),.vs_out(),.de_out(),.rgb_out());
    always @(negedge video_clk) if(!rst && done) begin
        if(waveform.bin_samples<200 || waveform.bin_max!==7'sd2)
            $fatal(1,"HDMI bridge samples did not reach waveform accumulator");
        if(waveform.spectrum_overrun || waveform.spectrum_overflow)
            $fatal(1,"Spectral tap failed at actual HDMI sample cadence");
    end
end endgenerate
function [7:0] file_byte;
    input integer n;
    integer sample_number,value;
    begin
        file_byte=0;
        case(n)
            0:file_byte="R";1:file_byte="I";2:file_byte="F";3:file_byte="F";
            4:file_byte=8'h2c;5:file_byte=2; // RIFF size 556 = 36 + 520
            8:file_byte="W";9:file_byte="A";10:file_byte="V";11:file_byte="E";
            12:file_byte="f";13:file_byte="m";14:file_byte="t";15:file_byte=" ";
            16:file_byte=16;20:file_byte=1;22:file_byte=1;
            24:file_byte=8'h80;25:file_byte=8'hbb;
            28:file_byte=8'h00;29:file_byte=8'h77;30:file_byte=1;
            32:file_byte=2;34:file_byte=16;
            36:file_byte="d";37:file_byte="a";38:file_byte="t";39:file_byte="a";
            40:file_byte=8'h08;41:file_byte=2; // 260 samples
            default:begin
                if(n>=44 && n<564) begin
                    sample_number=(n-44)/2;value=1000+sample_number;
                    file_byte=((n-44)%2)==0 ? value&255 : (value>>8)&255;
                end
            end
        endcase
    end
endfunction
function [7:0] card_byte;
    input integer sector,offset;
    begin
        card_byte=0;
        if(sector==100) begin
            case(offset)
                0,1,2,3,4,5,6,7:card_byte=NAME_TEXT[63-offset*8 -:8];
                8:card_byte="W";9:card_byte="A";10:card_byte="V";
                11:card_byte=8'h20;26:card_byte=5;
                28:card_byte=8'h34;29:card_byte=2; // file length 564
                default: ;
            endcase
        end
        if(sector==103) card_byte=file_byte(offset); // cluster5
        if(sector==105) card_byte=file_byte(512+offset); // cluster7
        if(sector==10 && offset==20) card_byte=7; // FAT[5] -> 7
    end
endfunction
always @(posedge clk) begin
    valid<=0;ending<=0;
    if(rst) begin active<=0;pos<=0;gap<=0;end
    else if(active) begin
        if(pos<512) begin data<=card_byte(active_addr,pos);valid<=1;pos<=pos+1;end
        else begin ending<=1;active<=0;gap<=3;end
    end else if(gap!=0) gap<=gap-1;
    else if(req) begin active<=1;active_addr<=addr;pos<=0;end
end
always @(negedge video_clk) if(!rst && out_valid && left!=0) begin
    if(left!==((1000+received)<<8) || right!==left)
        $fatal(1,"PCM mismatch at sample %0d: %h",received,left);
    received=received+1;
end
initial begin
    repeat(10) @(negedge clk);rst=0;
    @(negedge clk);start=1;@(negedge clk);start=0;
    wait(done);@(negedge clk);
    if(received!=260 || fault!=0) $fatal(1,"End event before 260 HDMI samples, count=%0d fault=%0d",received,fault);
    @(negedge clk);number=NAME_NUMBER+1;start=1;@(negedge clk);start=0;
    wait(fault!=0);@(negedge clk);
    if(fault!=1 || !mute || done) $fatal(1,"Missing audio must mute and report missing file");
    received=0;number=NAME_NUMBER;start=1;@(negedge clk);start=0;
    wait(received==10);@(negedge clk);stop=1;@(negedge clk);stop=0;
    repeat(5000) @(negedge clk);
    if(received!=10 || !mute || done) $fatal(1,"Manual stop failed to flush/mute track");
    received=0;start=1;@(negedge clk);start=0;
    wait(done);@(negedge clk);
    if(received!=260 || fault!=0) $fatal(1,"Restart must begin at first PCM sample");
    $display("WAV_PLAYER_PASS: fragmented PCM, exact samples, playback end, missing file, stop/restart");
    $finish;
end
initial begin #20000000;$fatal(1,"WAV test timeout");end
endmodule
