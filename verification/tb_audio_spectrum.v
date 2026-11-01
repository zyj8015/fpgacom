`timescale 1ns/1ps
module tb_audio_spectrum #(
 parameter integer SAMPLE_CLOCKS=1024,CASE_COUNT=133
);
localparam integer BINS=128;
localparam integer LAST_QUIET=128;
localparam integer LAST_LOUD=129;
localparam integer LAST_SILENCE=130;
localparam integer LAST_MIXED=132;
reg clk=0,rst=1,clear=0,sample_valid=0;
reg signed [15:0] sample_data=0;
always #6.734 clk=~clk;
reg [1:0] read_bank=0;
reg [6:0] address=0;
reg locked=0;
wire [5:0] level;
wire [1:0] published;
wire ready,valid,overrun,overflow;
album_pcm_spectrum dut(.clk(clk),.rst(rst),.clear(clear),
 .sample_valid(sample_valid),.sample_data(sample_data),
 .read_bank(read_bank),.read_address(address),.read_lock(locked),
 .read_level(level),.published_bank(published),.levels_ready(ready),
 .levels_valid(valid),.overrun(overrun),.overflow(overflow));
reg [15:0] samples[0:CASE_COUNT*512-1];
reg [BINS*6-1:0] expected[0:CASE_COUNT-1];
reg [BINS*6-1:0] captured;
integer completed=0,c,n,j,dominant,best_level;
always @(negedge clk) if(!rst && !clear && (overrun || overflow))
 $fatal(1,"Spectral arithmetic/throughput fault: overrun=%b overflow=%b",overrun,overflow);
always @(negedge clk) if(!rst && !clear && valid) begin
 if(!ready || published>2) $fatal(1,"Invalid publication");
 // Hold a read bank over two COMPLETE calculation windows.
 if(completed==0 || completed>2) begin read_bank=published;locked=1;end
 if(completed==1 || completed==2) begin
  if(published==read_bank) $fatal(1,"Writer published the locked read bank");
  captured=0;
  for(j=0;j<BINS;j=j+1)
   captured[j*6 +:6]=dut.level_memory[{published,j[6:0]}];
  if(captured!==expected[completed]) $fatal(1,"Published spectrum wrong during read lock");
 end
 captured=0;
 for(j=0;j<BINS;j=j+1) begin
  address=j;
  @(posedge clk);#1;captured[j*6 +:6]=level;
  @(negedge clk);
 end
 if(captured!==(completed==1 || completed==2 ? expected[0] : expected[completed]))
  $fatal(1,"Spectrum RAM read mismatch case %0d",completed);
 if(completed<BINS) begin
  if(completed==1 || completed==2) captured=expected[completed];
  dominant=0;best_level=captured[5:0];
  for(j=1;j<BINS;j=j+1) if(captured[j*6 +:6]>best_level) begin
   best_level=captured[j*6 +:6];dominant=j;
  end
  if(dominant!=completed) $fatal(1,"Equal-amplitude tone peak at wrong frequency point");
  $display("FREQUENCY_SELECTIVITY_PASS: tone %0d -> point %0d",completed,dominant);
 end
 if(completed>=2) locked=0;
 completed=completed+1;
end
initial begin
 $readmemh("spectrum_samples.hex",samples);
 $readmemh("spectrum_expected.hex",expected);
 repeat(8) @(negedge clk);rst=0;
 for(c=0;c<CASE_COUNT;c=c+1) begin
  // Quiet/loud are deliberately consecutive without clear. Windows1/2 test
  // a display-held bank across two newer coherent publications.
  if(c!=1 && c!=2 && c!=LAST_LOUD) begin
   @(negedge clk);clear=1;repeat(2) @(negedge clk);clear=0;
  end
  for(n=0;n<512;n=n+1) begin
   @(negedge clk);sample_valid=1;sample_data=samples[c*512+n];
   @(negedge clk);sample_valid=0;
   repeat(SAMPLE_CLOCKS-2) @(negedge clk);
  end
  if(completed!=c+1) $fatal(1,"Spectrum publication/read missing before next window");
 end
 @(negedge clk);sample_valid=1;sample_data=12345;
 @(negedge clk);sample_valid=0;repeat(5) @(negedge clk);clear=1;
 repeat(3) @(negedge clk);clear=0;repeat(1500) @(negedge clk);
 if(ready || valid || dut.sample_count!=0) $fatal(1,"Spectrum clear retained stale state");
 $display("AUDIO_SPECTRUM_PASS: %0d exact 128-frequency windows, %0d clocks/sample, triple-bank coherence, no overrun/overflow, in-flight clear",completed,SAMPLE_CLOCKS);
 $finish;
end
initial begin #2000000000;$fatal(1,"Spectrum timeout");end
endmodule

