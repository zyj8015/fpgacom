`timescale 1ns/1ps
// Digital integration test: real album scheduler, write FIFO and frame DMA.
// Synthetic complete/cancelled pixel streams; behavioral SDRAM word capture.
// No SPI parsing, HDMI PHY, refresh controller or analog SDRAM timing model.
module tb_manual_frame_integrity #(parameter MEMORY_CANCEL=1);
reg sdclk=0,memclk=0,vidclk=0,rst=1;
always #5 sdclk=~sdclk;
always #4 memclk=~memclk;
always #7 vidclk=~vidclk;
reg kn=0,kp=0,ka=0;
reg [1:0] visible_buf=0;
wire [1:0] write_buf,display_buf;
wire display_valid,write_req,write_ack,write_ready,write_finish,cancel_req,cancel_ack;
wire write_en;
wire [31:0] write_data;
wire [4:0] page;
reg finish_toggle=0;
wire app_we,app_re;
wire [11:0] app_wa,app_ra;
wire [31:0] app_wd;
reg [31:0] memory[0:4095];
reg producer_ready=1,producer_req=0,producer_valid=0;
reg [23:0] producer_pixel=0;
reg [4:0] producer_page=0;
integer phase=0,pixel_index=0,spacing=0,loads=0,commits=0,aborts=0,tick=0;
integer j,x,y,address,expected_index;
reg [31:0] random_state=32'h4d39b7e1;
function [23:0] pattern;
 input [4:0] p;
 input integer index;
 begin pattern=24'h200000+({19'd0,p}<<14)+index;end
endfunction
sd_card_bmp #(.ENABLE_WAV(1),.ENABLE_PREFETCH(1),.OUTPUT_WIDTH(16),.OUTPUT_HEIGHT(32),
 .PIXEL_PERIOD(4),.ENABLE_WRITE_CANCEL(MEMORY_CANCEL)) dut(
 .clk(sdclk),.rst(rst),.key_next(1'b1),.key_previous(1'b1),.key_auto(1'b1),
 .bmp_width(16'd16),.bmp_height(16'd32),.image_index(page),.display_valid(display_valid),
 .write_finish_toggle(finish_toggle),.write_buf_idx(write_buf),.disp_buf_idx(display_buf),
 .write_req(write_req),.write_req_ack(write_ack),.write_ready(write_ready),
 .write_en(write_en),.write_data(write_data),.active_display_buf(visible_buf),
 .SD_MISO(1'b1),.pcm_ready(1'b1),.pcm_last_played(1'b0),
 .write_cancel(cancel_req),.write_cancel_ack(cancel_ack));
frame_read_write #(.ADDR_BITS(12),.FRAME_WIDTH(16),.FRAME_HEIGHT(32),.BURST_SIZE(64),
 .ENABLE_WRITE_CANCEL(MEMORY_CANCEL)) frame_dma(
 .rst(rst),.mem_clk(memclk),.Sdr_init_done(1'b1),.Sdr_init_ref_vld(1'b0),.Sdr_busy(1'b0),
 .App_rd_en(app_re),.App_rd_addr(app_ra),.Sdr_rd_en(1'b0),.Sdr_rd_dout(32'd0),
 .read_clk(vidclk),.read_req(1'b0),.read_req_ack(),.read_finish(),
 .read_addr_0(12'd0),.read_addr_1(12'd512),.read_addr_2(12'd0),.read_addr_3(12'd0),
 .read_addr_index(2'd0),.read_len(12'd512),.read_en(1'b0),.read_data(),
 .App_wr_en(app_we),.App_wr_addr(app_wa),.App_wr_din(app_wd),.App_wr_dm(),
 .write_clk(sdclk),.write_req(write_req),.write_req_ack(write_ack),.write_finish(write_finish),
 .write_addr_0(12'd0),.write_addr_1(12'd512),.write_addr_2(12'd0),.write_addr_3(12'd0),
 .write_addr_index(write_buf),.write_len(12'd512),.write_ready(write_ready),
 .write_en(write_en),.write_data(write_data),.write_cancel(cancel_req),.write_cancel_ack(cancel_ack));
initial begin
 force dut.sd_init_done=1'b1;force dut.bmp_ready=producer_ready;
 force dut.write_req=producer_req;force dut.bmp_data_wr_en=producer_valid;force dut.bmp_data=producer_pixel;
 force dut.scan_done=1'b0;force dut.scan_found_valid=1'b0;force dut.catalog_sorted=1'b1;
 force dut.img_found_count=6'd3;force dut.bmp_load_failed=1'b0;
 force dut.sd_sec_read=1'b0;force dut.image_sector_busy=1'b0;
 force dut.wav_done=1'b0;force dut.audio_fault=4'd0;
 force dut.bmp_source_width=16'd640;force dut.bmp_source_height=16'd480;
 force dut.bmp_crop_width=16'd640;force dut.bmp_crop_height=16'd480;
 force dut.key_next_press=kn;force dut.key_prev_press=kp;force dut.key_auto_press=ka;
 for(j=0;j<4096;j=j+1) memory[j]=32'hdeadbeef;
end
always @(posedge memclk) begin
 if(rst) finish_toggle<=0;
 else begin
  if(app_we) memory[app_wa]<=app_wd;
  if(write_finish) finish_toggle<=~finish_toggle;
  if(cancel_ack && (app_we || !frame_dma.write_fifo_aclr))
   $fatal(1,"cancel acknowledged before FIFO/commands were quiescent");
 end
end
always @(posedge sdclk) begin
 producer_valid<=0;
 if(rst || dut.load_abort) begin
  producer_ready<=1;producer_req<=0;phase<=0;pixel_index<=0;spacing<=0;
  if(!rst) aborts=aborts+1;
 end else case(phase)
  0:if(dut.load_start_pulse) begin
   if(MEMORY_CANCEL && (cancel_req || cancel_ack)) $fatal(1,"load overlapped cancel handshake");
   producer_page<=dut.load_idx;producer_req<=1;producer_ready<=0;phase<=1;loads=loads+1;
  end
  1:if(dut.write_req_ack_sync[1]) begin producer_req<=0;phase<=2;end
  2:if(!dut.write_req_ack_sync[1]) begin phase<=3;spacing<=0;end
  3:if(write_ready) begin
   if(spacing==3) begin
    producer_valid<=1;producer_pixel<=pattern(producer_page,pixel_index);spacing<=0;
    if(pixel_index==511) phase<=4;else pixel_index<=pixel_index+1;
   end else spacing<=spacing+1;
  end
  4:begin producer_ready<=1;phase<=0;pixel_index<=0;end
 endcase
 if(!rst) begin
  tick=tick+1;
  if(tick%127==0) visible_buf<=display_buf;
  if(dut.track_start) begin
   commits=commits+1;
   for(y=0;y<32;y=y+1) for(x=0;x<16;x=x+1) begin
    address=display_buf*512+y*16+x;expected_index=(31-y)*16+x;
    if(memory[address]!=={pattern(page,expected_index),8'd0})
     $fatal(1,"published incomplete/stale pixel page%0d addr%0d got%h expected%h",
            page,address,memory[address],{pattern(page,expected_index),8'd0});
   end
  end
 end
end
task clocks;
 input integer count;begin repeat(count) @(negedge sdclk);end
endtask
initial begin
 clocks(8);rst=0;
 dut.catalog_memory[0]={32'd1,32'd100};dut.catalog_memory[1]={32'd2,32'd200};
 dut.catalog_memory[2]={32'd3,32'd300};
 wait(display_valid);clocks(8);
 // Strict forward-only manual clicks, including early/repeated requests.
 for(j=0;j<24;j=j+1) begin
  clocks(400+(j%4)*600);kn=1;clocks(1);kn=0;
 end
 clocks(5000);
 // Direction changes during partial prefills and while cancel is pending.
 for(j=0;j<80;j=j+1) begin
  random_state={random_state[30:0],random_state[31]^random_state[21]^random_state[1]^random_state[0]};
  clocks(60+(random_state[11:0]%2500));
  if(random_state[17]) kn=1;else kp=1;
  clocks(1);kn=0;kp=0;
 end
 clocks(6000);
 if(commits<10 || (MEMORY_CANCEL && aborts<10)) $fatal(1,"insufficient manual/cancel coverage");
 $display("MANUAL_FRAME_INTEGRITY_PASS: cancel=%0d loads=%0d commits=%0d aborts=%0d",MEMORY_CANCEL,loads,commits,aborts);
 $finish;
end
initial begin #5000000;
 $display("DEBUG valid%0d loads%0d commits%0d phase%0d pix%0d dma%0d wcnt%0d used%0d ready%0d srcdone%0d wrdone%0d started%0d armed%0d req%0d ack%0d",
  display_valid,loads,commits,phase,pixel_index,frame_dma.frame_fifo_write_m0.state,
  frame_dma.frame_fifo_write_m0.write_cnt,frame_dma.write_buf.rdusedw,write_ready,
  dut.source_done_seen,dut.write_done_seen,dut.load_started_seen,dut.write_armed_seen,write_req,write_ack);
 $display("DEBUG dma_in=%0d q=%0d len=%0d into=%b rbusy=%b rdstate=%0d size=%0d",
  frame_dma.frame_fifo_write_m0.rdusedw,frame_dma.frame_fifo_write_m0.rdusedw_q,
  frame_dma.frame_fifo_write_m0.write_len_latch,frame_dma.frame_fifo_write_m0.into_burst,
  frame_dma.App_rd_busy,frame_dma.frame_fifo_read_m0.state,frame_dma.frame_fifo_write_m0.BURST_SIZE);
 $fatal(1,"manual integrity test timeout");end
endmodule
