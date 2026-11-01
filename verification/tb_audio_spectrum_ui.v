`timescale 1ns/1ps
module tb_audio_spectrum_ui;
localparam integer BINS=128;
reg clk=0,rst=1,vs=0,de=0,epoch=0;
reg [BINS*6-1:0] target=0;
wire [5:0] fake_level=target[dut.spectrum_scan*6 +:6];
integer i,n;
always #5 clk=~clk;
album_audio_waveform #(.ENABLE_SPECTRUM(0)) dut(
 .clk(clk),.rst(rst),.hs_in(1'b0),.vs_in(vs),.de_in(de),.rgb_in(24'h102030),
 .frame_valid(1'b1),.visible_page(6'd1),.narration_epoch(epoch),
 .sample_valid(1'b0),.sample_data(16'd0),.hs_out(),.vs_out(),.de_out(),.rgb_out());
task frame;
 begin
  @(negedge clk);de=0;vs=1;
  @(negedge clk);vs=0;
  repeat(400) @(negedge clk);
 end
endtask
task check_height;
 input integer value;
 begin
  for(i=0;i<BINS;i=i+1)
   if(dut.height_memory[i][5:0]!==value[5:0]) $fatal(1,"Smoothed height wrong at %0d",i);
 end
endtask
initial begin
 force dut.spectrum_read_level=fake_level;force dut.spectrum_ready=1'b1;
 repeat(8) @(negedge clk);rst=0;repeat(5) @(negedge clk);
 for(i=0;i<BINS;i=i+1) target[i*6 +:6]=48;
 frame();check_height(48);
 target=0;de=1;repeat(180) @(negedge clk);check_height(48);
 for(n=1;n<=8;n=n+1) begin frame();check_height(48-n);end
 for(i=0;i<BINS;i=i+1) target[i*6 +:6]=i%64;
 epoch=1;repeat(3) @(negedge clk);
 if(dut.spectrum_filled_valid) $fatal(1,"New narration retained old filled spectrum");
 frame();
 for(i=0;i<BINS;i=i+1)
  if(dut.height_memory[i][5:0]!=i%64) $fatal(1,"128 frequency-point order wrong");
 epoch=0;repeat(3) @(negedge clk);
 for(i=0;i<BINS;i=i+1) target[i*6 +:6]=48;
 frame();target=0;
 @(negedge clk);de=0;vs=1;
 @(negedge clk);vs=0;repeat(5) @(negedge clk);
 de=1;repeat(180) @(negedge clk);
 if(!dut.spectrum_scan_busy) $fatal(1,"Update did not pause for active video");
 de=0;repeat(400) @(negedge clk);check_height(47);
 $display("AUDIO_SPECTRUM_UI_PASS: 128 filled frequencies, attack/release, order, frame stability, clear, interrupted update");
 $finish;
end
initial begin #2000000;$fatal(1,"Spectrum UI timeout");end
endmodule

