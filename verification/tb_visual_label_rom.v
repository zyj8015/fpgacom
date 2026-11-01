`timescale 1ns/1ps
module tb_visual_label_rom;
reg clk=0;
reg [10:0] address=0;
wire pixel;
integer i,row,col,id;
reg [34:0] bitmap;
reg expected;
always #5 clk=~clk;
album_visual_label_rom dut(.clk(clk),.address(address),.pixel(pixel));
// Golden native glyphs from the previous combinational title renderer.
function [34:0] glyph;
 input integer n;
 begin case(n)
  0:glyph=35'h4631ad6aa;1:glyph=35'h3a31fc631;
  2:glyph={5'b10001,5'b10001,5'b10001,5'b10001,5'b10001,5'b01010,5'b00100};
  3:glyph=35'h7e10f421f;4:glyph=35'h3e107043e;5:glyph=35'h7a31f4210;
  6:glyph=35'h3a308422e;7:glyph=35'h7c8421084;8:glyph=35'h7a31f5251;
  9:glyph=35'h46318c62e;10:glyph=35'h4775ac631;11:glyph=35'h42108421f;
  12:glyph=35'h3a318c62e;13:glyph=35'h4631fc631;14:glyph=35'h38842108e;
  15:glyph=35'h3a30bc62f;
  16:glyph={5'b10000,5'b01000,5'b00100,5'b00010,5'b00100,5'b01000,5'b10000};
  17:glyph=35'h3a317842e; // 9
  18:glyph=35'h08ca97c42; // 4
  19:glyph={5'b11111,5'b00001,5'b00010,5'b00100,5'b01000,5'b10000,5'b11111}; // Z
  20:glyph=35'h3a211111f; // 2
  21:glyph=35'h3a33ae62e; // 0
  22:glyph={5'b10001,5'b10010,5'b10100,5'b11000,5'b10100,5'b10010,5'b10001}; // K
  23:glyph=35'h3a10f462e; // 6
  24:glyph=35'h11842108e; // 1
  25:glyph=35'h3a317462e; // 8
  default:glyph=0;
 endcase end
endfunction
initial begin
 for(i=0;i<2048;i=i+1) begin
  @(negedge clk);address=i;id=i/64;row=(i/8)%8;col=i%8;
  bitmap=glyph(id);expected=0;
  if(row<7 && col<5) expected=bitmap[34-row*5-col];
  @(posedge clk);#1;
  if(pixel!==expected) $fatal(1,"Title ROM pixel mismatch at %0d",i);
 end
 $display("VISUAL_LABEL_ROM_PASS: all 2048 native title/frequency-label glyph and padding pixels match");
 $finish;
end
endmodule
