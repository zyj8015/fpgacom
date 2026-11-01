`timescale 1ns/1ps
module tb_bmp_load_entry;
reg clk=0,rst=1,start=0,abort_load=0,geometry=0;
reg [31:0] address=0,base=0;
reg [3:0] shift=0;
wire ready,failed,read_req,write_req;
wire [3:0] error_code;
integer cases=0,i;
reg [31:0] random_word=32'h12947385;
always #5 clk=~clk;
bmp_read dut(.clk(clk),.rst(rst),.ready(ready),.scan_start(1'b0),
 .scan_start_sector(32'd0),.scan_max_sector(32'd0),.scan_target_count(6'd0),
 .scan_done(),.scan_found_valid(),.scan_found_sector(),.scan_found_order(),.scan_found_total(),
 .load_start(start),.load_abort(abort_load),.load_sector(address),.sd_init_done(1'b1),
 .state_code(),.reader_state(),.bmp_width(16'd640),.bmp_height(16'd480),
 .write_req(write_req),.write_req_ack(1'b0),.write_ready(1'b1),
 .sd_sec_read(read_req),.sd_sec_read_addr(),.sd_sec_read_data(8'd0),
 .sd_sec_read_data_valid(1'b0),.sd_sec_read_end(1'b0),.bmp_data_wr_en(),.bmp_data(),
 .load_failed(failed),.error_code(error_code),.catalog_full(),.fs_valid(),.fs_data_start(),
 .fs_fat_start(),.fs_root_cluster(),.fs_max_cluster(),.fs_volume_end(),.fs_fat_sectors(),
 .fs_cluster_shift(),.fs_spc(),.source_width(),.source_height(),.source_crop_width(),.source_crop_height());
task check_load;
 input g;
 input [31:0] b,a;
 input [3:0] sh;
 reg valid_address;
 reg [31:0] relative;
 begin
  @(negedge clk);geometry=g;base=b;shift=sh;address=a;start=1;
  valid_address=g && a>=b;relative=(a-b)>>sh;
  @(negedge clk);start=0;
  if(dut.state!==dut.ST_LOAD_ADDR1 || failed) $fatal(1,"Load capture stage incorrect");
  // Inputs can change after the request; consume the captured predicate/data.
  address=~a;base=~b;geometry=~g;
  @(negedge clk);
  if(valid_address) begin
   if(dut.state!==dut.ST_LOAD_ADDR2 || dut.load_relative_cluster!==relative || failed)
    $fatal(1,"Valid address snapshot mismatch");
   abort_load=1;
   @(negedge clk);abort_load=0;
  end else if(!ready || !failed || error_code!=3 || read_req || write_req)
   $fatal(1,"Invalid address was not rejected without card/FIFO access");
  repeat(2) @(negedge clk);
  if(!ready || failed) $fatal(1,"Reader did not return ready");
  cases=cases+1;
 end
endtask
initial begin
 force dut.geometry_valid=geometry;force dut.data_start=base;force dut.cluster_shift=shift;
 repeat(6) @(negedge clk);rst=0;repeat(3) @(negedge clk);
 check_load(0,32'd100,32'd120,0);
 check_load(1,32'd100,32'd99,3);
 check_load(1,32'd100,32'd100,0);
 check_load(1,32'd0,32'hffffffff,7);
 for(i=0;i<100;i=i+1) begin
  random_word=random_word^(random_word<<13);random_word=random_word^(random_word>>17);random_word=random_word^(random_word<<5);
  check_load(i[0],random_word,{random_word[15:0],random_word[31:16]},i%8);
 end
 @(negedge clk);geometry=1;base=100;address=120;start=1;
 @(negedge clk);start=0;abort_load=1;
 @(negedge clk);abort_load=0;
 if(!ready || failed || read_req) $fatal(1,"Abort in captured-address stage failed");
 check_load(1,32'd200,32'd216,3);
 $display("BMP_LOAD_ENTRY_PASS: %0d range/geometry/snapshot cases and abort/retry",cases);$finish;
end
initial begin #1000000;$fatal(1,"BMP load entry timeout");end
endmodule
