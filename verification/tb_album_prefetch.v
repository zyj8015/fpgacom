`timescale 1ns/1ps
// USER-RUN controller unit test: force peripheral outputs, exercise real album
// scheduling and synchronizers. This is NOT an SD/SDRAM/audio integration test.
module tb_album_prefetch #(parameter MEMORY_CANCEL=0);
reg clk=0,rst=1;
always #5 clk=~clk;
reg [1:0] active_buf=0;
wire [1:0] disp_buf,write_buf;
wire [4:0] page;
wire valid;
wire [15:0] width;
reg kp_next=0,kp_prev=0,kp_auto=0;
reg reader_ready=1,reader_req=0,write_ack=0,finish_toggle=0;
reg allow_finish=0,done_level=0,image_busy=0;
reg [15:0] reader_width=640;
integer reader_phase=0,delay_count=0,loads=0,starts=0,saved_loads;
wire cancel_req;
reg cancel_ack=0;
integer memory_tail=0;
always @(posedge clk) begin
    if(rst) begin cancel_ack<=0;memory_tail<=0;end
    else if(dut.load_abort) begin memory_tail<=97;cancel_ack<=0;end
    else if(memory_tail!=0) memory_tail<=memory_tail-1;
    else cancel_ack<=cancel_req;
end
sd_card_bmp #(.ENABLE_WAV(1),.ENABLE_PREFETCH(1),.ENABLE_WRITE_CANCEL(MEMORY_CANCEL)) dut(
    .clk(clk),.rst(rst),.key_next(1'b1),.key_previous(1'b1),.key_auto(1'b1),
    .bmp_width(16'd1280),.bmp_height(16'd720),.display_valid(valid),
    .image_index(page),.disp_buf_idx(disp_buf),.write_buf_idx(write_buf),
    .write_finish_toggle(finish_toggle),.write_req_ack(write_ack),.write_ready(1'b1),
    .active_display_buf(active_buf),.SD_MISO(1'b1),.pcm_ready(1'b1),.pcm_last_played(1'b0),
    .display_source_width(width),.write_cancel(cancel_req),.write_cancel_ack(cancel_ack));
initial begin
    force dut.sd_init_done=1'b1;
    force dut.bmp_ready=reader_ready;
    force dut.write_req=reader_req;
    force dut.bmp_data_wr_en=1'b0;
    force dut.bmp_load_failed=1'b0;
    force dut.scan_found_valid=1'b0;
    force dut.scan_done=1'b0;
    force dut.catalog_sorted=1'b1;
    force dut.img_found_count=6'd3;
    force dut.sd_sec_read=1'b0;
    force dut.image_sector_busy=image_busy;
    force dut.bmp_source_width=reader_width;
    force dut.bmp_source_height=16'd480;
    force dut.bmp_crop_width=reader_width;
    force dut.bmp_crop_height=16'd480;
    force dut.wav_done=done_level;
    force dut.audio_fault=4'd0;
    force dut.key_next_press=kp_next;
    force dut.key_prev_press=kp_prev;
    force dut.key_auto_press=kp_auto;
end
// Stub the reader/memory handshakes; no image bytes are modeled here.
always @(posedge clk) begin
    write_ack<=reader_req;
    if(rst || dut.load_abort) begin
        reader_ready<=1; reader_req<=0; reader_phase<=0; delay_count<=0;
    end else case(reader_phase)
        0: if(dut.load_start_pulse) begin
            if(MEMORY_CANCEL && (memory_tail!=0 || cancel_req || cancel_ack))
                $fatal(1,"new load overlapped memory cancellation/drain");
            reader_ready<=0; reader_req<=1; reader_phase<=1; loads=loads+1;
            reader_width<=640+160*dut.load_idx;
            if(valid && (write_buf==disp_buf || active_buf!=disp_buf))
                $fatal(1,"load before visible-buffer release");
        end
        1: if(dut.write_req_ack_sync[1]) begin reader_req<=0; reader_phase<=2; end
        2: if(!dut.write_req_ack_sync[1] && allow_finish) begin
            if(delay_count==12) begin
                reader_ready<=1; finish_toggle<=~finish_toggle;
                reader_phase<=0; delay_count<=0;
            end else delay_count<=delay_count+1;
        end
    endcase
    if(!rst && dut.track_start) starts=starts+1;
end
task pulse;
    input integer which;
    begin
        @(negedge clk);
        case(which) 0:kp_next=1; 1:kp_prev=1; 2:kp_auto=1; endcase
        @(negedge clk); kp_next=0; kp_prev=0; kp_auto=0;
    end
endtask
task clocks;
    input integer n;
    begin repeat(n) @(negedge clk); end
endtask
initial begin
    clocks(5); rst=0;
    dut.catalog_memory[0]={32'd1,32'd100};
    dut.catalog_memory[1]={32'd2,32'd200};
    dut.catalog_memory[2]={32'd3,32'd300};
    allow_finish=1;
    wait(valid); clocks(4);
    if(page!=0 || starts!=1 || width!=640) $fatal(1,"first commit failed");
    wait(dut.cache_valid && dut.cache_idx==1 && !dut.load_busy); clocks(4);
    if(page!=0 || starts!=1 || dut.track_number!=1)
        $fatal(1,"background load published a page or audio");
    pulse(2); clocks(6); // enable auto, current narration not yet finished
    if(page!=0 || starts!=1) $fatal(1,"mode toggle interrupted narration");
    saved_loads=loads; done_level=1;
    wait(page==1); clocks(8);
    if(loads!=saved_loads || width!=800 || dut.track_number!=2)
        $fatal(1,"cached advance reloaded or used wrong metadata");
    // Deliberately hold old done level and old visible buffer. Neither a new
    // advance nor an overwrite may happen while the new track/frame starts.
    clocks(20);
    if(page!=1 || dut.next_req_pending || loads!=saved_loads)
        $fatal(1,"stale done level or missing video acknowledgment guard");
    done_level=0;
    pulse(1); // old page is still retained, so backwards hit is immediate
    wait(page==0); clocks(8);
    if(dut.auto_play_en || width!=640 || dut.track_number!=1)
        $fatal(1,"previous cache metadata/mode mismatch");
    wait(dut.load_busy && dut.load_idx==2); // backward speculation
    allow_finish=0; image_busy=1;
    pulse(0); clocks(12);
    saved_loads=loads;
    if(!dut.cancel_wait || page!=0) $fatal(1,"opposite-key cancellation failed");
    clocks(20);
    if(loads!=saved_loads) $fatal(1,"new load started before sector drain");
    image_busy=0; allow_finish=1;
    wait(page==1); clocks(6);
    if(width!=800 || dut.track_number!=2) $fatal(1,"cancel/reload wrong page");
    active_buf=disp_buf; clocks(5);
    // Missing/corrupt speculative image retains current displayed page and
    // blocks endless retries. Manual controls can subsequently request retry.
    wait(dut.load_busy); force dut.bmp_load_failed=1'b1;
    clocks(1); force dut.bmp_load_failed=1'b0; clocks(15);
    saved_loads=loads; clocks(20);
    if(page!=1 || !dut.prefetch_blocked || loads!=saved_loads)
        $fatal(1,"failed speculation corrupted display or retried forever");
    $display("ALBUM_PREFETCH_PASS: audio gating, cache hits, reverse, drain, visible buffer, failure");
    $finish;
end
initial begin #100000; $fatal(1,"prefetch unit timeout"); end
endmodule

module tb_sector_cancel;
reg clk=0,rst=1,ir=0,ar=0,cancel=0,card_end=0;
reg card_valid=0;
reg [7:0] card_data=8'h5a;
always #5 clk=~clk;
wire req,ie,ae,ib,ab;
wire [31:0] addr;
wire iv,av;
wire [7:0] idata,adata;
album_sector_arbiter #(.ENABLE_IMAGE_CANCEL(1)) dut(
    .clk(clk),.rst(rst),.image_req(ir),.image_addr(32'd11),.image_cancel(cancel),
    .audio_req(ar),.audio_addr(32'd22),.card_req(req),.card_addr(addr),
    .card_data(card_data),.card_valid(card_valid),.card_end(card_end),
    .image_data(idata),.image_valid(iv),.audio_data(adata),.audio_valid(av),
    .image_end(ie),.audio_end(ae),.image_busy(ib),.audio_busy(ab));
initial begin
    repeat(3) @(negedge clk); rst=0; ir=1; ar=1;
    @(negedge clk);
    if(!req || addr!=22 || !ab) $fatal(1,"audio priority lost");
    card_valid=1;card_data=8'ha5;
    @(negedge clk);
    if(!av || iv || adata!=8'ha5) $fatal(1,"registered audio data/valid mismatch");
    card_data=8'h3c;
    @(negedge clk);
    if(!av || iv || adata!=8'h3c) $fatal(1,"registered consecutive audio byte mismatch");
    card_valid=0;
    cancel=1;
    @(negedge clk);
    if(!ie || !req || addr!=22 || ae) $fatal(1,"ungranted image cancel damaged audio");
    cancel=0; ir=0; ar=0; card_end=1;
    @(negedge clk); card_end=0;
    repeat(5) @(negedge clk);
    ir=1;
    @(negedge clk);
    if(!req || addr!=11 || !ib) $fatal(1,"image grant failed");
    card_valid=1;card_data=8'h66;
    @(negedge clk);
    if(!iv || av || idata!=8'h66) $fatal(1,"registered image data/valid mismatch");
    card_valid=0;
    cancel=1;
    @(negedge clk); cancel=0; ir=0;
    repeat(5) begin
        @(negedge clk);
        if(!req || ie || !ib) $fatal(1,"owned sector aborted before card completion");
    end
    card_end=1; @(negedge clk);
    if(!ie || ae) $fatal(1,"owned sector not routed to image");
    card_end=0;
    repeat(5) @(negedge clk);
    if(req || ib) $fatal(1,"sector drain stuck");
    $display("SECTOR_CANCEL_PASS: audio priority, ungranted cancel, owned-sector drain");
    $finish;
end
initial begin #10000; $fatal(1,"sector cancel timeout"); end
endmodule
