`timescale 1ns/1ps
module tb_character_id_rom;
reg clk=0;
always #5 clk=~clk;
reg [3:0] page=0;
reg [6:0] index=0;
reg [5:0] focus_index=0;
wire [8:0] character_id,focus_id;
reg [7:0] text_expected[0:2047],focus_expected[0:1023];
integer i;
album_teaching_assets dut(.clk(clk),.page(page),.character_index(index),.focus_index(focus_index),
 .subtitle_font_address(13'd0),.focus_font_address(13'd0),.focus_font_select(1'b0),
 .sprite_address(15'd0),.sprite_lane(2'd0),.character_id(character_id),.focus_id(focus_id),
 .subtitle_font_row(),.focus_font_row(),.sprite_pixel(),.text_length(),
 .demo_start_cs(),.demo_end_cs(),.teacher_x(),.teacher_y(),.focus_lines(),.scroll_step());
initial begin
 $readmemh("text_id_expected.hex",text_expected);
 $readmemh("focus_id_expected.hex",focus_expected);
 for(i=0;i<2048;i=i+1) begin
  @(negedge clk);page=i/128;index=i%128;focus_index=i%64;
  @(posedge clk);#1;
  if(character_id!=={1'b0,text_expected[i]} || focus_id!=={1'b0,focus_expected[(i/128)*64+i%64]})
   $fatal(1,"Character ROM address/one-clock latency mismatch at %0d",i);
 end
 $display("CHARACTER_ID_ROM_PASS: all2048caption and1024focus entries, invalid-page blanks, unchanged one-clock latency");
 $finish;
end
endmodule
