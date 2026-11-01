`timescale 1ns/1ps
// Synthetic sector interface for optional user-run tests, not an SPI/card model.
// Five files reuse the three actual BMP payloads. Their clusters are fragmented;
// the root directory also spans two nonadjacent clusters. FAT #2 is active.
module sd_card_top(
    input clk, rst,
    output SD_nCS, SD_DCLK, SD_MOSI,
    input SD_MISO,
    output sd_init_done,
    input sd_sec_read,
    input [31:0] sd_sec_read_addr,
    output reg [7:0] sd_sec_read_data=0,
    output reg sd_sec_read_data_valid=0, sd_sec_read_end=0,
    input sd_sec_write,
    input [31:0] sd_sec_write_addr,
    input [7:0] sd_sec_write_data,
    output sd_sec_write_data_req, sd_sec_write_end
);
    reg [7:0] disk[0:5417*512-1];
    reg [7:0] mbr[0:511], boot[0:511];
    reg [7:0] dir0[0:4095], dir1[0:4095];
    integer i, pos=0, sector=0, cooldown=0;
    reg active=0;
    assign sd_init_done=!rst;
    assign SD_nCS=1; assign SD_DCLK=0; assign SD_MOSI=0;
    assign sd_sec_write_data_req=0; assign sd_sec_write_end=0;
    function integer first_cluster;
        input integer photo;
        begin first_cluster=10003+photo*1024; end
    endfunction
    function integer first_sector;
        input integer photo;
        begin first_sector=65536+(first_cluster(photo)-2)*8; end
    endfunction
    function integer original_base;
        input integer photo;
        begin
            case(photo%3)
                0: original_base=8*512;
                1: original_base=1812*512;
                default: original_base=3616*512;
            endcase
        end
    endfunction
    task set_entry;
        input integer directory, slot, photo;
        integer j, cl;
        reg [7:0] entry[0:31];
        begin
            cl=first_cluster(photo);
            for(j=0;j<32;j=j+1) entry[j]=0;
            for(j=0;j<8;j=j+1) entry[j]="I";
            entry[7]="1"+photo;
            entry[8]="B"; entry[9]="M"; entry[10]="P"; entry[11]=8'h20;
            entry[20]=(cl>>16)&255; entry[21]=(cl>>24)&255;
            entry[26]=cl&255; entry[27]=(cl>>8)&255;
            for(j=0;j<4;j=j+1) entry[28+j]=(921654>>(8*j))&255;
            for(j=0;j<32;j=j+1)
                if(directory==0) dir0[slot*32+j]=entry[j];
                else dir1[slot*32+j]=entry[j];
        end
    endtask
    function [31:0] fat_value;
        input integer cl;
        integer photo, base;
        begin
            fat_value=0;
            if(cl==2) fat_value=6000;
            else if(cl==6000) fat_value=32'h0ffffff8;
            for(photo=0;photo<5;photo=photo+1) begin
                base=first_cluster(photo);
                if(cl>=base && cl<=base+450 && ((cl-base)%2)==0)
                    fat_value=(cl==base+450) ? 32'h0ffffff8 : cl+2;
            end
        end
    endfunction
    function [7:0] card_byte;
        input integer lba, byte_pos;
        integer cl, photo, base, file_sector, entry_index;
        reg [31:0] value;
        begin
            card_byte=0;
            if(lba==0) card_byte=mbr[byte_pos];
            else if(lba==2048) card_byte=boot[byte_pos];
            else if(lba>=33808 && lba<65536) begin
                entry_index=(lba-33808)*128+byte_pos/4;
                value=fat_value(entry_index);
                card_byte=(value>>(8*(byte_pos%4)))&255;
            end else if(lba>=65536 && lba<65544)
                card_byte=dir0[(lba-65536)*512+byte_pos];
            else if(lba>=113520 && lba<113528)
                card_byte=dir1[(lba-113520)*512+byte_pos];
            else if(lba>=65536) begin
                cl=(lba-65536)/8+2;
                for(photo=0;photo<5;photo=photo+1) begin
                    base=first_cluster(photo);
                    if(cl>=base && cl<=base+450 && ((cl-base)%2)==0) begin
                        file_sector=((cl-base)/2)*8+(lba-65536)%8;
                        if(file_sector<1801)
                            card_byte=disk[original_base(photo)+file_sector*512+byte_pos];
                    end
                end
            end
        end
    endfunction
    initial begin
        $readmemh("photos.hex",disk);
        for(i=0;i<512;i=i+1) begin mbr[i]=0; boot[i]=0; end
        for(i=0;i<4096;i=i+1) begin dir0[i]=0; dir1[i]=0; end
        for(i=0;i<128;i=i+1) dir0[i*32]=8'he5;
        mbr[450]=8'h0c; mbr[455]=8'h08;
        mbr[510]=8'h55; mbr[511]=8'haa;
        boot[12]=2; boot[13]=8; boot[14]=32; boot[16]=2;
        boot[34]=8'h80; // total sectors = 0x00800000
        boot[36]=8'hf0; boot[37]=8'h7b;
        boot[40]=8'h81; boot[44]=2; // active FAT #2, root cluster 2
        boot[510]=8'h55; boot[511]=8'haa;
        set_entry(0,5,0); set_entry(0,8,1); set_entry(0,15,2);
        set_entry(1,0,3); set_entry(1,1,4); // followed by directory end
        // Deleted BMP, LFN, volume label and a directory must not be counted.
        set_entry(0,0,0); dir0[0]=8'he5;
        set_entry(0,1,0); dir0[32+11]=8'h0f;
        set_entry(0,2,0); dir0[64+11]=8'h08;
        set_entry(0,3,0); dir0[96+11]=8'h10;
        // Existing .BMP entry with an invalid header must also be skipped.
        set_entry(0,4,0); dir0[128+26]=8'h28; dir0[128+27]=8'h23; // cluster 9000
    end
    always @(negedge clk) begin
        sd_sec_read_data_valid=0; sd_sec_read_end=0;
        if(rst) begin active=0; cooldown=0; end
        else if(active) begin
            if(pos<512) begin
                sd_sec_read_data=card_byte(sector,pos);
                sd_sec_read_data_valid=1; pos=pos+1;
            end else begin sd_sec_read_end=1; active=0; cooldown=2; end
        end else if(cooldown>0) cooldown=cooldown-1;
        else if(sd_sec_read) begin
            sector=sd_sec_read_addr; pos=0; active=1;
        end
    end
endmodule
