`timescale 1ns/1ps
module tb_photos;
    reg clk=0,rst=1;
    always #5 clk=~clk;
    reg scan_start=0,load_start=0,load_abort=0;
    reg [31:0] load_sector=0;
    wire ready,scan_done,found_valid,write_req,pixel_valid,sd_read,sd_valid,sd_end,sd_init;
    wire [31:0] found_sector,sd_addr;
    wire [5:0] found_total;
    wire [23:0] pixel;
    wire [7:0] sd_data;
    wire [3:0] error_code;
    reg write_ack=0;
    integer found=0,current_photo=0,pixels=0,expected_index=0;
    reg [31:0] starts[0:4];
    bmp_read dut(
        .clk(clk),.rst(rst),.ready(ready),
        .scan_start(scan_start),.scan_start_sector(32'd0),.scan_max_sector(32'd131071),
        .scan_target_count(6'd0),.scan_done(scan_done),.scan_found_valid(found_valid),
        .scan_found_sector(found_sector),.scan_found_total(found_total),
        .load_start(load_start),.load_abort(load_abort),.load_sector(load_sector),
        .sd_init_done(sd_init),.state_code(),.bmp_width(16'd640),.bmp_height(16'd480),
        .write_req(write_req),.write_req_ack(write_ack),
        .sd_sec_read(sd_read),.sd_sec_read_addr(sd_addr),.sd_sec_read_data(sd_data),
        .sd_sec_read_data_valid(sd_valid),.sd_sec_read_end(sd_end),
        .bmp_data_wr_en(pixel_valid),.bmp_data(pixel),
        .load_failed(),.error_code(error_code),.catalog_full()
    );
    sd_card_top card(
        .clk(clk),.rst(rst),.SD_nCS(),.SD_DCLK(),.SD_MOSI(),.SD_MISO(1'b1),
        .sd_init_done(sd_init),.sd_sec_read(sd_read),.sd_sec_read_addr(sd_addr),
        .sd_sec_read_data(sd_data),.sd_sec_read_data_valid(sd_valid),.sd_sec_read_end(sd_end),
        .sd_sec_write(1'b0),.sd_sec_write_addr(32'd0),.sd_sec_write_data(8'd0),
        .sd_sec_write_data_req(),.sd_sec_write_end()
    );
    always @(posedge clk) write_ack<=!rst && write_req;
    // Encoding changed without changing the public compact diagnostic codes.
    // The full photo test exercises discovery, fragmented FAT reads and abort.
    integer state_index,expected_state;
    always @(negedge clk) if(!rst) begin
        if((^dut.state)===1'bx || dut.state==0 || (dut.state & (dut.state-37'd1))!=0)
            $fatal(1,"BMP state is not one-hot");
        expected_state=0;
        for(state_index=0;state_index<37;state_index=state_index+1)
            if(dut.state[state_index]) expected_state=state_index==35 ? 20 :
                state_index==36 ? 22 : state_index>=32 ? 18 : state_index;
        if(dut.reader_state!==(sd_init ? 5'(expected_state) : 5'd31))
            $fatal(1,"BMP compact diagnostic code changed");
    end
    always @(posedge clk) begin
        if(!rst && found_valid) begin
            if(found>=5) $fatal(1,"Too many photos: deleted/invalid entries counted");
            if(found_sector!==card.first_sector(found)) $fatal(1,"Wrong directory order");
            starts[found]=found_sector; found=found+1;
        end
        if(!rst && pixel_valid) begin
            if(pixels>=307200) $fatal(1,"Extra pixel");
            expected_index=card.original_base(current_photo)+54+pixels*3;
            if(pixel!=={card.disk[expected_index+2],card.disk[expected_index+1],card.disk[expected_index]})
                $fatal(1,"Fragmented photo %0d pixel %0d mismatch",current_photo,pixels);
            pixels=pixels+1;
        end
    end
    initial begin
        repeat(8) @(negedge clk); rst=0;
        @(negedge clk); scan_start=1; @(negedge clk); scan_start=0;
        wait(scan_done); repeat(4) @(negedge clk);
        if(found!=5 || found_total!=5 || error_code!=0)
            $fatal(1,"Dynamic catalog count/error: count=%0d error=%0d",found_total,error_code);
        load_sector=starts[0]; load_start=1;
        @(negedge clk); load_start=0;
        wait(!ready); repeat(20) @(negedge clk);
        load_abort=1; @(negedge clk); load_abort=0;
        wait(ready); repeat(4) @(negedge clk);
        if(!scan_done || found_total!=5) $fatal(1,"Abort lost catalog");
        for(current_photo=0;current_photo<5;current_photo=current_photo+1) begin
            pixels=0;
            @(negedge clk); load_sector=starts[current_photo]; load_start=1;
            @(negedge clk); load_start=0;
            wait(!ready); wait(ready); repeat(4) @(negedge clk);
            if(pixels!=307200 || error_code!=0)
                $fatal(1,"Photo %0d pixel count=%0d error=%0d",current_photo,pixels,error_code);
        end
        $display("PHOTOS_PASS: 5 dynamic files, deleted/invalid filtering, active FAT, fragmented chains");
        $finish;
    end
    // Scaler deliberately spaces writes to match the board frame FIFO.
    initial begin repeat(3) #1000000000; $fatal(1,"Photo test timeout"); end
endmodule

// Optional USER-RUN regression: unreset metadata must never validate a short
// sector, even after a previous valid scan left a matching boot signature.
module tb_bmp_sector_boundaries;
    reg clk=0,rst=1,scan_start=0;
    always #5 clk=~clk;
    wire ready,scan_done,read_req,fs_valid;
    wire [31:0] read_addr;
    wire [3:0] error_code;
    reg [7:0] read_data=0;
    reg read_valid=0,read_end=0;
    reg [7:0] boot[0:511];
    integer i;
    bmp_read dut(
        .clk(clk),.rst(rst),.ready(ready),.scan_start(scan_start),
        .scan_start_sector(32'd0),.scan_max_sector(32'd0),.scan_target_count(6'd0),
        .scan_done(scan_done),.scan_found_valid(),.scan_found_sector(),.scan_found_order(),
        .scan_found_total(),.load_start(1'b0),.load_abort(1'b0),.load_sector(32'd0),
        .sd_init_done(1'b1),.state_code(),.reader_state(),
        .bmp_width(16'd640),.bmp_height(16'd480),.write_req(),
        .write_req_ack(1'b0),.write_ready(1'b1),.sd_sec_read(read_req),
        .sd_sec_read_addr(read_addr),.sd_sec_read_data(read_data),
        .sd_sec_read_data_valid(read_valid),.sd_sec_read_end(read_end),
        .bmp_data_wr_en(),.bmp_data(),.load_failed(),.error_code(error_code),
        .catalog_full(),.fs_valid(fs_valid),.fs_data_start(),.fs_fat_start(),
        .fs_root_cluster(),.fs_max_cluster(),.fs_volume_end(),.fs_fat_sectors(),
        .fs_cluster_shift(),.fs_spc(),.source_width(),.source_height(),
        .source_crop_width(),.source_crop_height());
    task start_scan;
        begin
            wait(ready);@(negedge clk);scan_start=1;
            @(negedge clk);scan_start=0;wait(read_req);
            if(read_addr!==0) $fatal(1,"probe address mismatch");
        end
    endtask
    task sector;
        input integer count;
        input integer use_boot;
        input integer end_with_last;
        integer n;
        begin
            for(n=0;n<count;n=n+1) begin
                @(negedge clk);read_valid=1;read_data=use_boot ? boot[n] : 0;
                read_end=end_with_last && n==count-1;
            end
            @(negedge clk);read_valid=0;read_end=!end_with_last;
            @(negedge clk);read_end=0;
            wait(!read_req);
        end
    endtask
    task valid_scan;
        input integer end_with_last;
        begin
            start_scan();sector(512,1,end_with_last);
            wait(read_req);
            if(!fs_valid || read_addr!==2) $fatal(1,"valid boot geometry rejected");
            sector(512,0,end_with_last);wait(scan_done);
            if(error_code!==0) $fatal(1,"valid empty directory rejected");
            repeat(4) @(negedge clk);
        end
    endtask
    initial begin
        for(i=0;i<512;i=i+1) boot[i]=0;
        boot[12]=2;boot[13]=1;boot[14]=1;boot[16]=1;
        boot[32]=8'h10;boot[33]=8'h27;boot[36]=1;boot[44]=2;
        boot[510]=8'h55;boot[511]=8'haa;
        repeat(5) @(negedge clk);rst=0;
        valid_scan(0); // conventional EOF one clock after byte511
        // Reset the control path but retain all previously captured payload.
        rst=1;repeat(3) @(negedge clk);rst=0;
        start_scan();sector(128,1,0);wait(scan_done);
        if(fs_valid!==0 || error_code!==3) $fatal(1,"stale signature accepted a short boot sector");
        repeat(4) @(negedge clk);
        valid_scan(1); // final byte and EOF asserted together
        $display("BMP_SECTOR_BOUNDARIES_PASS: reset, dense bytes, aligned EOF, short-sector rejection");
        $finish;
    end
    initial begin #2000000;$fatal(1,"sector boundary test timeout");end
endmodule
