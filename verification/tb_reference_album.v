`timescale 1ns/1ps
module tb_reference_album;
    reg clk=0,mem_clk=0,rst=1,key_next=1,key_auto=1;
    always #5 clk=~clk;
    always #4 mem_clk=~mem_clk;
    wire display_valid,write_req,write_en,auto_status;
    wire [1:0] write_buf,disp_buf;
    wire [31:0] write_data;
    wire [5:0] image_count;
    wire [4:0] image_index;
    wire [3:0] fault_code;
    reg req_meta=0,req_sync=0,write_ack=0,finish_toggle=0;
    reg source_complete_toggle=0,source_meta=0,source_sync=0;
    integer photo=0,pixels=0,index=0,frames=0;
    // Shorten only UI timing. The BMP files remain full-sized.
    sd_card_bmp #(.CLK_FREQ_HZ(100000),.SCAN_TARGET_COUNT(6'd0),.AUTO_INTERVAL_MS(100)) dut(
        .clk(clk),.rst(rst),.key_next(key_next),.key_previous(1'b1),.key_auto(key_auto),
        .bmp_width(16'd640),.bmp_height(16'd480),.display_valid(display_valid),.state_code(),
        .auto_status(auto_status),.image_count(image_count),.image_index(image_index),
        .fault_code(fault_code),
        .write_finish_toggle(finish_toggle),.write_buf_idx(write_buf),.disp_buf_idx(disp_buf),
        .write_req(write_req),.write_req_ack(write_ack),.write_en(write_en),.write_data(write_data),
        .SD_nCS(),.SD_DCLK(),.SD_MOSI(),.SD_MISO(1'b1)
    );
    always @(posedge mem_clk) begin
        if(rst) begin
            req_meta<=0; req_sync<=0; write_ack<=0;
            source_meta<=0; source_sync<=0; finish_toggle<=0;
        end else begin
            req_meta<=write_req; req_sync<=req_meta; write_ack<=req_sync;
            source_meta<=source_complete_toggle; source_sync<=source_meta;
            finish_toggle<=source_sync;
        end
    end
    always @(posedge clk) if(!rst && write_en) begin
        if(write_ack || write_req) $fatal(1,"Pixels before FIFO clear handshake release");
        if(display_valid && write_buf==disp_buf) $fatal(1,"Overwriting displayed buffer");
        index=dut.sd_card_top_m0.original_base(photo)+54+pixels*3;
        if(write_data!=={dut.sd_card_top_m0.disk[index+2],dut.sd_card_top_m0.disk[index+1],
                       dut.sd_card_top_m0.disk[index],8'd0})
            $fatal(1,"Frame %0d pixel %0d mismatch",frames,pixels);
        if(pixels==307199) begin
            pixels=0; frames=frames+1; photo=(photo==4)?0:photo+1;
            source_complete_toggle<=~source_complete_toggle;
        end else pixels=pixels+1;
    end
    task press_auto;
        begin
            @(negedge clk); key_auto=0; repeat(6000) @(negedge clk);
            key_auto=1; repeat(6000) @(negedge clk);
        end
    endtask
    initial begin
        repeat(10) @(negedge clk); rst=0;
        wait(display_valid); repeat(4) @(negedge clk);
        if(image_index!=0 || frames!=1 || image_count!=5)
            $fatal(1,"First frame or dynamic count failed");
        key_next=0; repeat(6000) @(negedge clk);
        key_next=1; repeat(6000) @(negedge clk);
        wait(image_index==1); repeat(4) @(negedge clk);
        press_auto;
        if(!display_valid || image_count!=5 || !auto_status)
            $fatal(1,"Mode change cleared display/catalog or failed to enable");
        wait(image_index==2); wait(image_index==3);
        wait(image_index==4); wait(image_index==0);
        repeat(4) @(negedge clk);
        if(frames!=6 || !auto_status || fault_code!=0)
            $fatal(1,"Dynamic automatic wrap failed");
        press_auto;
        if(auto_status || !display_valid) $fatal(1,"Auto pause failed");
        $display("REFERENCE_ALBUM_PASS: 5-photo manual/automatic wrap and retained display");
        $finish;
    end
    initial begin repeat(4) #1000000000; $fatal(1,"Album timeout, page=%0d fault=%0d",image_index,fault_code); end
endmodule
