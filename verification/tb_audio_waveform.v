`timescale 1ns/1ps
module tb_audio_waveform;
reg clk=0,rst=1,hs=0,vs=0,de=0,frame_valid=1,epoch=0;
reg [5:0] page=1;
reg [23:0] rgb=0;
reg sample_valid=0;
reg signed [15:0] sample_data=0;
always #5 clk=~clk;
wire ho,vo,dout;
wire [23:0] rout;
album_audio_waveform #(.ENABLE_SPECTRUM(0)) dut(
 .clk(clk),.rst(rst),.hs_in(hs),.vs_in(vs),.de_in(de),.rgb_in(rgb),
 .frame_valid(frame_valid),.visible_page(page),.narration_epoch(epoch),
 .sample_valid(sample_valid),.sample_data(sample_data),
 .hs_out(ho),.vs_out(vo),.de_out(dout),.rgb_out(rout));
integer shown_lo[0:127],shown_hi[0:127];
integer pending_lo=0,pending_hi=0,history_count=0;
integer px=0,py=0,pixels=0,frames=0;
reg check_pixels=0;
reg [23:0] expect0,expect1;
reg [2:0] marker0,marker1;
reg skip0=0,skip1=0;
integer column,lower,upper,r,g,b,h,mapped,axis_col,axis_id;
reg [34:0] axis_bitmap;
reg [23:0] golden_rgb;
function integer clamp_peak;
 input integer v;
 begin clamp_peak=v>31 ? 31 : v< -32 ? -32 : v;end
endfunction
function [34:0] axis_glyph;
 input integer id;
 begin case(id)
  0:axis_glyph=35'h3a317842e; //9
  1:axis_glyph=35'h08ca97c42; //4
  2:axis_glyph=35'h4631fc631; //H
  3:axis_glyph={5'b11111,5'b00001,5'b00010,5'b00100,5'b01000,5'b10000,5'b11111}; //Z
  4:axis_glyph=35'h3a211111f; //2
  5:axis_glyph=35'h3a33ae62e; //0
  6:axis_glyph={5'b10001,5'b10010,5'b10100,5'b11000,5'b10100,5'b10010,5'b10001}; //K
  default:axis_glyph=0;
 endcase end
endfunction
always @* begin
 golden_rgb=rgb;column=0;lower=0;upper=0;r=0;g=0;b=0;h=0;
 mapped=0;axis_col=0;axis_id=0;axis_bitmap=0;
 if(de && frame_valid && page!=0 &&
    ((px>=240 && px<528) || (px>=540 && px<828)) && py>=4 && py<68) begin
  r=(rgb[23:16]+241)/2;g=(rgb[15:8]+229)/2;b=(rgb[7:0]+203)/2;
  golden_rgb={r[7:0],g[7:0],b[7:0]};
  if(px>=256 && px<512 && py>=16 && py<64) begin
   column=(px-256)/2;
   upper=(31-clamp_peak(shown_hi[column]))*3/4;
   lower=(31-clamp_peak(shown_lo[column]))*3/4;
   if(py==39 || (px-256)%64==0) golden_rgb=24'hc6b992;
   if(column>=128-history_count && (px-256)%2==0 && py-16>=upper && py-16<=lower)
    golden_rgb=24'h287c70;
  end
  if(px>=556 && px<812 && py>=16 && py<56) begin
   column=(px-556)/2;
   h=column%64;
   mapped=(h*5+7)/8;
   if((py-16)%10==9) golden_rgb=24'hc6b992;
   if(py-16>=40-mapped && py-16<40) begin
    golden_rgb=(py-16==40-mapped) ? 24'h76a7ff : 24'h3f6fd8;
   end
   if(py-16==39) golden_rgb=24'h587bd8;
  end
  if(py>=58 && py<65 && ((px>=556 && px<588) || (px>=772 && px<812))) begin
   if(px<588) begin axis_col=px-556;axis_id=axis_col/8;end
   else begin
    axis_col=px-772;
    case(axis_col/8)
     0:axis_id=4;1:axis_id=5;2:axis_id=6;3:axis_id=2;default:axis_id=3;
    endcase
   end
   axis_bitmap=axis_glyph(axis_id);
   if(axis_col%8<5)
    if(axis_bitmap[34-(py-58)*5-axis_col%8]) golden_rgb=24'h382c22;
  end
  if(px==240 || px==527 || px==540 || px==827 || py==4 || py==67) golden_rgb=24'ha18858;
 end
end
always @(posedge clk) begin
 if(rst) begin marker0<=0;marker1<=0;expect0<=0;expect1<=0;skip0<=0;skip1<=0;end
 else begin
  marker0<={hs,vs,de};marker1<=marker0;
  expect0<=golden_rgb;expect1<=expect0;
  // Native header letters are checked for known pixels, not mirrored here.
  skip0<=((px>=256 && px<512) || (px>=556 && px<812)) && py>=7 && py<14;skip1<=skip0;
  #1;
  if(check_pixels) begin
   if({ho,vo,dout}!==marker1) $fatal(1,"RGB/video marker latency mismatch");
   if((^rout)===1'bx) $fatal(1,"Unknown rendered pixel");
   if(!skip1 && rout!==expect1)
    $fatal(1,"History pixel mismatch (%0d,%0d) got %h expected %h",px,py,rout,expect1);
   if(dout) pixels=pixels+1;
  end
 end
end
task sample;
 input integer value;
 begin
  @(negedge clk);sample_valid=1;sample_data=value;
  @(negedge clk);sample_valid=0;
 end
endtask
task collect;
 input integer kind,amplitude;
 integer n,value,q;
 begin
  pending_lo=63;pending_hi=-64;
  for(n=0;n<1536;n=n+1) begin
   case(kind)
    0:value=n[0] ? amplitude : -amplitude;
    1:value=0;
    2:value=n[0] ? 32767 : -32768;
    default:value=(n*337)%16001-8000;
   endcase
   q=value>>>9;
   if(q<pending_lo) pending_lo=q;
   if(q>pending_hi) pending_hi=q;
   sample(value);
   if(n<1535 && dut.pending_valid) $fatal(1,"Bin completed too early");
  end
  if(!dut.pending_valid || dut.bin_samples!=0) $fatal(1,"1536-sample bin not completed");
  if($signed(dut.pending_peaks[13:7])!==pending_hi ||
     $signed(dut.pending_peaks[6:0])!==pending_lo) $fatal(1,"Incorrect signed extrema");
 end
endtask
task update_model;
 integer i;
 begin
  for(i=0;i<127;i=i+1) begin shown_lo[i]=shown_lo[i+1];shown_hi[i]=shown_hi[i+1];end
  shown_lo[127]=pending_lo;shown_hi[127]=pending_hi;
  if(history_count<128) history_count=history_count+1;
 end
endtask
task check_memory;
 integer i,addr;
 begin
  if(dut.display_count!=history_count || dut.first_visible!=128-history_count)
   $fatal(1,"History count/empty leading columns wrong");
  for(i=128-history_count;i<128;i=i+1) begin
   addr=(dut.write_pointer+i)%128;
   if($signed(dut.peaks[addr][13:7])!==shown_hi[i] ||
      $signed(dut.peaks[addr][6:0])!==shown_lo[i]) $fatal(1,"Ring chronology wrong at %0d",i);
  end
 end
endtask
task publish;
 begin
  @(negedge clk);de=0;vs=1;
  @(negedge clk);vs=0;update_model();
  repeat(3) @(negedge clk);
  if(dut.pending_valid) $fatal(1,"Pending window was not consumed on VS");
  check_memory();
 end
endtask
task raster;
 integer row,col;
 begin
  if(dut.pending_valid) $fatal(1,"Publish expected pending bin before raster");
  @(negedge clk);de=0;vs=1;
  @(negedge clk);vs=0;
  repeat(400) @(negedge clk);check_pixels=1;
  for(row=0;row<720;row=row+1) begin
   for(col=0;col<1280;col=col+1) begin
    @(negedge clk);de=1;hs=0;px=col;py=row;rgb={row[7:0],col[7:0],8'h93};
   end
   @(negedge clk);de=0;hs=1;rgb=24'h123456;
   repeat(4) @(negedge clk);
  end
  repeat(4) @(negedge clk);check_pixels=0;frames=frames+1;
 end
endtask
task clear_model;
 integer i;
 begin
  history_count=0;
  for(i=0;i<128;i=i+1) begin shown_lo[i]=0;shown_hi[i]=0;end
 end
endtask
integer i,pointer_before;
reg [767:0] test_spectrum;
wire [5:0] fake_spectrum_level=test_spectrum[dut.spectrum_scan*6 +:6];
initial begin
 test_spectrum=0;
 for(i=0;i<128;i=i+1) test_spectrum[i*6 +:6]=i%64;
 // The separate spectral bench checks real PCM analysis. Here a known set
 // of heights isolates bar geometry, peak lines and marker/pixel alignment.
 force dut.spectrum_read_level=fake_spectrum_level;force dut.spectrum_ready=1'b1;
 clear_model();repeat(8) @(negedge clk);rst=0;repeat(8) @(negedge clk);
 collect(0,2048);
 if(dut.display_count!=0) $fatal(1,"History published without VS");
 publish();collect(0,12288);publish();collect(1,0);publish();raster();
 // More than 128 windows verify full history, wrap and soft/loud differences.
 for(i=0;i<132;i=i+1) begin collect(0,(i%24+2)*512);publish();end
 raster();pointer_before=dut.write_pointer;
 // New audio arrives while a complete displayed frame remains immutable.
 fork collect(2,0);raster();join
 if(dut.write_pointer!=pointer_before) $fatal(1,"History advanced during active frame");
 check_memory();publish();raster();
 // Both page and epoch changes discard old history and a partial window.
 repeat(11) sample(-32768);
 @(negedge clk);epoch=~epoch;
 repeat(3) @(negedge clk);clear_model();
 if(dut.display_count!=0 || dut.bin_samples!=0 || dut.pending_valid)
  $fatal(1,"Narration epoch did not clear history");
 collect(1,0);publish();
 @(negedge clk);page=2;
 repeat(3) @(negedge clk);clear_model();
 if(dut.display_count!=0) $fatal(1,"New page retained history");
 collect(1,0);publish();raster();
 @(negedge clk);frame_valid=0;
 repeat(3) @(negedge clk);clear_model();raster();
 if(frames!=6 || pixels!=6*921600) $fatal(1,"Incomplete raster coverage: %0d",pixels);
 // A bin finishing on the same edge as VS must not overwrite the older
 // pending bin before it is published, nor lose the newly finished bin.
 @(negedge clk);frame_valid=1;page=3;
 repeat(3) @(negedge clk);clear_model();collect(0,2048);
 repeat(1535) sample(8192);
 @(negedge clk);de=0;vs=1;sample_valid=1;sample_data=8192;
 @(negedge clk);vs=0;sample_valid=0;update_model();
 repeat(3) @(negedge clk);
 if(!dut.pending_valid || dut.pending_peaks!=={7'd16,7'd16})
  $fatal(1,"Simultaneous publish/finish dropped the new bin");
 check_memory();pending_lo=16;pending_hi=16;publish();
 $display("AUDIO_WAVEFORM_PASS: 128 filled frequency columns, blue analyzer fill/top/baseline, corner labels, waveform history/wrap/gain, page clear, six 720p rasters");
 $finish;
end
initial begin #100000000;$fatal(1,"Waveform history test timeout");end
endmodule
