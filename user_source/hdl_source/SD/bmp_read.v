// FAT32 root-directory BMP reader. All filesystem work runs in FPGA logic.
// Supports MBR primary FAT32 partitions or an unpartitioned FAT32 volume,
// 512-byte sectors, power-of-two cluster sizes, and fragmented file chains.
// Input: 640..1920 x 480..1080 positive-height RGB24 BI_RGB, 40-byte DIB.
// Output: parameterized nearest-neighbour, bottom-up; fit or center-crop fill.
// Board top selects 1280x720; 640x480 defaults retain legacy test compatibility.
// A sector buffer allows the streaming scaler to pause between bytes.
module bmp_read #(
    parameter integer MAX_IMAGES = 32,
    parameter integer MAX_DIRECTORY_READS = 4096,
    parameter integer OUTPUT_WIDTH = 640,
    parameter integer OUTPUT_HEIGHT = 480,
    parameter integer FILL_MODE = 0, // 0: fit with bars; 1: center crop to fill
    parameter integer WRITE_BACKPRESSURE = 0,
    parameter integer PIXEL_PERIOD = 96
)(
    input clk, rst,
    output ready,
    input scan_start,
    input [31:0] scan_start_sector, // retained for the reference interface
    input [31:0] scan_max_sector,   // raw-sector scanning is no longer used
    input [5:0] scan_target_count,  // 0: discover until directory end/capacity
    output reg scan_done,
    output reg scan_found_valid,
    output reg [31:0] scan_found_sector,
    output reg [31:0] scan_found_order,
    output reg [5:0] scan_found_total,
    input load_start, load_abort,
    input [31:0] load_sector,
    input sd_init_done,
    output reg [3:0] state_code,
    output [4:0] reader_state,
    input [15:0] bmp_width, bmp_height,
    output reg write_req,
    input write_req_ack,
    input write_ready,
    output reg sd_sec_read,
    output reg [31:0] sd_sec_read_addr,
    input [7:0] sd_sec_read_data,
    input sd_sec_read_data_valid, sd_sec_read_end,
    output bmp_data_wr_en,
    output [23:0] bmp_data,
    output reg load_failed,
    output reg [3:0] error_code,
    output reg catalog_full,
    output fs_valid,
    output [31:0] fs_data_start,fs_fat_start,fs_root_cluster,
    output [31:0] fs_max_cluster,fs_volume_end,fs_fat_sectors,
    output [3:0] fs_cluster_shift, output [7:0] fs_spc,
    output [15:0] source_width,source_height,
    output [15:0] source_crop_width,source_crop_height
);
localparam [5:0] IDLE=0, PROBE=1, PROBE_EVAL=2, BOOT=3, BOOT_EVAL=4,
    GEOMETRY=5, GEOMETRY_CHECK=6, CLUSTER_ADDR=7, CLUSTER_ISSUE=8,
    DIR_BEGIN=9, DIR_READ=10, DIR_EVAL=11, DIR_NEXT=12, RETURN_DIR=13,
    HEADER_READ=14, HEADER_EVAL=15, WRITE_WAIT=16, WRITE_RELEASE=17,
    DATA_READ=18, DATA_EVAL=19, FAT_ADDR=20, FAT_READ=21, FAT_EVAL=22,
    LOAD_ADDR1=23, LOAD_ADDR2=24, CLUSTER_SUM=25, CLUSTER_CHECK=26,
    PART_FLAGS=27, PART_PAIRS=28, PART_PICK=29, PART_CHECK=30, PART_APPLY=31,
    DATA_FETCH=32, DATA_FEED=33, DATA_FINISH=34, FAT_USE=35, FAT_APPLY=36;
// Explicit one-hot FSM: TD project has infer_fsm disabled. Direct bit tests
// avoid distributing a binary state bit through many equality decoders.
localparam [36:0] ST_IDLE=(37'd1 << IDLE),
    ST_PROBE=(37'd1 << PROBE),
    ST_PROBE_EVAL=(37'd1 << PROBE_EVAL),
    ST_BOOT=(37'd1 << BOOT),
    ST_BOOT_EVAL=(37'd1 << BOOT_EVAL),
    ST_GEOMETRY=(37'd1 << GEOMETRY),
    ST_GEOMETRY_CHECK=(37'd1 << GEOMETRY_CHECK),
    ST_CLUSTER_ADDR=(37'd1 << CLUSTER_ADDR),
    ST_CLUSTER_ISSUE=(37'd1 << CLUSTER_ISSUE),
    ST_DIR_BEGIN=(37'd1 << DIR_BEGIN),
    ST_DIR_READ=(37'd1 << DIR_READ),
    ST_DIR_EVAL=(37'd1 << DIR_EVAL),
    ST_DIR_NEXT=(37'd1 << DIR_NEXT),
    ST_RETURN_DIR=(37'd1 << RETURN_DIR),
    ST_HEADER_READ=(37'd1 << HEADER_READ),
    ST_HEADER_EVAL=(37'd1 << HEADER_EVAL),
    ST_WRITE_WAIT=(37'd1 << WRITE_WAIT),
    ST_WRITE_RELEASE=(37'd1 << WRITE_RELEASE),
    ST_DATA_READ=(37'd1 << DATA_READ),
    ST_DATA_EVAL=(37'd1 << DATA_EVAL),
    ST_FAT_ADDR=(37'd1 << FAT_ADDR),
    ST_FAT_READ=(37'd1 << FAT_READ),
    ST_FAT_EVAL=(37'd1 << FAT_EVAL),
    ST_LOAD_ADDR1=(37'd1 << LOAD_ADDR1),
    ST_LOAD_ADDR2=(37'd1 << LOAD_ADDR2),
    ST_CLUSTER_SUM=(37'd1 << CLUSTER_SUM),
    ST_CLUSTER_CHECK=(37'd1 << CLUSTER_CHECK),
    ST_PART_FLAGS=(37'd1 << PART_FLAGS),
    ST_PART_PAIRS=(37'd1 << PART_PAIRS),
    ST_PART_PICK=(37'd1 << PART_PICK),
    ST_PART_CHECK=(37'd1 << PART_CHECK),
    ST_PART_APPLY=(37'd1 << PART_APPLY),
    ST_DATA_FETCH=(37'd1 << DATA_FETCH),
    ST_DATA_FEED=(37'd1 << DATA_FEED),
    ST_DATA_FINISH=(37'd1 << DATA_FINISH),
    ST_FAT_USE=(37'd1 << FAT_USE),
    ST_FAT_APPLY=(37'd1 << FAT_APPLY);

localparam [1:0] CL_DIR=0, CL_HEADER=1, CL_DATA=2;
localparam [5:0] IMAGE_CAPACITY=MAX_IMAGES;
reg [36:0] state;
reg [9:0] rd_cnt;
reg cancel_pending, geometry_valid;
reg geometry_bounds_valid_q;
// Capture separate byte banks for each consumer, rather than driving all
// boot/header/directory/FAT registers from one high-fanout byte bus.
// End and valid advance together; the FSM still sees a complete transaction.
reg [7:0] boot_byte,dir_byte,header_byte,fat_byte,buffer_byte;
reg boot_byte_valid,dir_byte_valid,header_byte_valid,fat_byte_valid,buffer_byte_valid;
reg read_valid_q,read_end_q,sector_complete;
always @(posedge clk) begin
    if(sd_sec_read_data_valid) begin
        if(state[PROBE] || state[BOOT]) boot_byte<=sd_sec_read_data;
        if(state[DIR_READ]) dir_byte<=sd_sec_read_data;
        if(state[HEADER_READ]) header_byte<=sd_sec_read_data;
        if(state[FAT_READ]) fat_byte<=sd_sec_read_data;
        if(state[DATA_READ]) buffer_byte<=sd_sec_read_data;
    end
end
always @(posedge clk or posedge rst) begin
    if(rst) begin
        read_valid_q<=0;read_end_q<=0;sector_complete<=0;
        boot_byte_valid<=0;dir_byte_valid<=0;header_byte_valid<=0;
        fat_byte_valid<=0;buffer_byte_valid<=0;
    end else begin
        read_valid_q<=sd_sec_read_data_valid;
        read_end_q<=sd_sec_read_end;
        boot_byte_valid<=sd_sec_read_data_valid && (state[PROBE] || state[BOOT]);
        dir_byte_valid<=sd_sec_read_data_valid && state[DIR_READ];
        header_byte_valid<=sd_sec_read_data_valid && state[HEADER_READ];
        fat_byte_valid<=sd_sec_read_data_valid && state[FAT_READ];
        buffer_byte_valid<=sd_sec_read_data_valid && state[DATA_READ];
        if(read_end_q)
            sector_complete<=rd_cnt==512 || (rd_cnt==511 && read_valid_q);
    end
end

// Boot-sector fields.
reg [15:0] boot_signature, bytes_per_sector, reserved_sectors;
reg [15:0] root_entries, fat16_sectors, ext_flags;
reg [7:0] sectors_per_cluster, fat_count;
reg [31:0] fat32_sectors, total_sectors, root_cluster;
reg [7:0] partition_type [0:3];
reg [31:0] partition_start [0:3];
reg [31:0] volume_start, data_start, fat_start, max_cluster;
reg [3:0] cluster_shift;
reg [2:0] geometry_stage;
reg [32:0] reserved_base, fat_area, data_base_sum, fat_base_sum;
reg [32:0] volume_end_sum, data_sector_count;
reg [31:0] data_cluster_count;
// Final STA found partition_start -> priority selection/zero test ->
// read-address enable paths with 10-11 LUT levels. Register every selection
// step before allowing it to control the shared sector-address register.
reg [3:0] partition_usable;
reg [31:0] partition_pair0, partition_pair1, first_partition_q;
reg first_partition_present;
wire boot_valid = boot_signature==16'haa55 && bytes_per_sector==512 &&
    reserved_sectors!=0 && root_entries==0 && fat16_sectors==0 &&
    fat32_sectors!=0 && (fat_count==1 || fat_count==2) &&
    sectors_per_cluster!=0 && sectors_per_cluster<=128 &&
    ((sectors_per_cluster & (sectors_per_cluster-8'd1))==0) &&
    (!ext_flags[7] || (ext_flags[3:0]<fat_count)) &&
    total_sectors!=0 && root_cluster>=2 && root_cluster<32'h0ffffff0;
function [3:0] cluster_log2;
    input [7:0] n;
    begin
        case(n)
            1: cluster_log2=0; 2: cluster_log2=1; 4: cluster_log2=2;
            8: cluster_log2=3; 16: cluster_log2=4; 32: cluster_log2=5;
            64: cluster_log2=6; 128: cluster_log2=7;
            default: cluster_log2=0;
        endcase
    end
endfunction

// Root-directory cursor. Candidate processing resumes at the next entry,
// rereading the sector when necessary, rather than buffering a whole sector.
reg [31:0] dir_cluster, dir_lba;
reg [7:0] dir_sector_in_cluster;
reg [9:0] dir_resume_offset;
reg [12:0] directory_reads;
reg dir_end_seen, candidate_pending;
reg [7:0] entry_first, entry_attr, entry_ext0, entry_ext1, entry_ext2;
reg [31:0] entry_cluster, entry_size;
reg [31:0] candidate_cluster, candidate_size, candidate_lba;
reg [26:0] entry_number;
reg entry_has_number;
reg [63:0] entry_name;
reg name_parse_busy,name_parse_phase;
reg [2:0] name_parse_index;
reg [7:0] name_digit;
reg [26:0] name_times10;
reg [31:0] candidate_order;
wire [31:0] completed_entry_size = {dir_byte,entry_size[23:0]};
wire entry_is_bmp = entry_first!=0 && entry_first!=8'he5 &&
    entry_first!=8'h2e && ((entry_attr & 8'h1e)==0) &&
    ((entry_ext0 & 8'hdf)=="B") && ((entry_ext1 & 8'hdf)=="M") &&
    ((entry_ext2 & 8'hdf)=="P") && entry_cluster>=2 &&
    entry_cluster<=max_cluster;

// Cluster addressing and FAT lookups are shared by the directory and image.
reg [31:0] cluster_target;
reg [1:0] cluster_kind;
reg [32:0] cluster_offset;
wire [32:0] cluster_lba_sum = {1'b0,data_start}+cluster_offset;
reg [32:0] cluster_lba_q;
reg cluster_number_bad, cluster_address_bad;
reg [31:0] chain_cluster, fat_entry;
reg [8:0] fat_entry_offset;
reg fat_for_file;
reg fat_sector_bad_q,fat_next_valid_q,fat_next_eoc_q;
reg [31:0] fat_read_address_q;
wire [31:0] fat_sector_index = chain_cluster >> 7;
wire [31:0] next_cluster = {4'd0,fat_entry[27:0]};
wire next_cluster_valid = next_cluster>=2 && next_cluster<=max_cluster &&
    next_cluster<32'h0ffffff0 && next_cluster!=chain_cluster;

// BMP header and byte stream.
reg header_for_scan;
reg [7:0] header_0, header_1;
reg [31:0] file_len, pixel_offset, width, height, dib_size, compression;
reg [15:0] planes, bit_count;
reg header_match;
reg [31:0] file_cluster, bmp_len_cnt;
reg [31:0] load_relative_sector, load_relative_cluster;
reg load_address_valid_q;
reg [7:0] file_sector_in_cluster;
// Boot geometry settles long before file data. Precompute the boundary so
// DATA_EVAL does not chain an increment, comparison and register enables.
reg [7:0] last_sector_in_cluster;
always @(posedge clk) last_sector_in_cluster<=sectors_per_cluster-8'd1;
// Header arithmetic is registered while the rest of the sector arrives.
reg [31:0] row_stride, pixel_bytes;
reg [32:0] pixel_end_offset;
// Header fields settle hundreds of byte clocks before HEADER_EVAL. Register
// the end address there instead of subtracting then comparing on every byte.
// The carry bit rejects a header whose data range wraps the32bit file size.
wire pixels_complete = {1'b0,bmp_len_cnt}>=pixel_end_offset;
reg [7:0] sector_buffer [0:511]; /* fehdl force_ram=1, ram_style="bram" */
reg [8:0] sector_index;
reg [7:0] sector_byte;
wire scaler_ready, scaler_done;
wire [11:0] scaler_crop_width,scaler_crop_height;
wire scaler_start = state[WRITE_RELEASE] && !write_req_ack;
wire scaler_byte_valid = state[DATA_FEED] && bmp_len_cnt>=pixel_offset && !pixels_complete;
album_bmp_scaler #(.OUTPUT_WIDTH(OUTPUT_WIDTH),.OUTPUT_HEIGHT(OUTPUT_HEIGHT),.FILL_MODE(FILL_MODE),
    .WRITE_BACKPRESSURE(WRITE_BACKPRESSURE),.PIXEL_PERIOD(PIXEL_PERIOD)) scaler (
    .clk(clk), .rst(rst || !sd_init_done || load_abort), .start(scaler_start),
    .source_width(width[11:0]), .source_height(height[10:0]),
    .row_stride(row_stride[12:0]), .byte_data(sector_byte),
    .byte_valid(scaler_byte_valid), .byte_ready(scaler_ready),
    .pixel_data(bmp_data), .pixel_valid(bmp_data_wr_en), .done(scaler_done),
    .pixel_ready(write_ready),
    .crop_width(scaler_crop_width),.crop_height(scaler_crop_height)
);
always @(posedge clk) begin
    if(state[DATA_READ] && buffer_byte_valid)
        sector_buffer[rd_cnt[8:0]]<=buffer_byte;
    if(state[DATA_FETCH]) sector_byte<=sector_buffer[sector_index];
end
assign source_width=width[15:0]; assign source_height=height[15:0];
assign source_crop_width={4'd0,scaler_crop_width};
assign source_crop_height={4'd0,scaler_crop_height};
assign fs_valid=geometry_valid;
assign fs_data_start=data_start; assign fs_fat_start=fat_start;
assign fs_root_cluster=root_cluster; assign fs_max_cluster=max_cluster;
assign fs_volume_end=volume_end_sum[31:0]; assign fs_fat_sectors=fat32_sectors;
assign fs_cluster_shift=cluster_shift; assign fs_spc=sectors_per_cluster;
assign ready = state[IDLE] && !cancel_pending;
wire compact_state_0 = |(state & 37'haaaaaaaa);
wire compact_state_1 = |(state & 37'h17cccccccc);
wire compact_state_2 = |(state & 37'h18f0f0f0f0);
wire compact_state_3 = |(state & 37'hff00ff00);
wire compact_state_4 = |(state & 37'h1fffff0000);
assign reader_state = !sd_init_done ? 5'd31 : {compact_state_4,compact_state_3,compact_state_2,compact_state_1,compact_state_0};

always @(posedge clk or posedge rst) begin
    if(rst) rd_cnt<=0;
    else if(!sd_init_done || read_end_q) rd_cnt<=0;
    else if(read_valid_q) rd_cnt<=rd_cnt+1'b1;
end

// All these fields are overwritten by the complete512-byte boot/probe sector.
// They are consumed only after sector_complete; resetting their data bits would
// unnecessarily connect hundreds of flops to the global reset route.
always @(posedge clk) begin
    if(boot_byte_valid) begin
        case(rd_cnt)
            11: bytes_per_sector[7:0]<=boot_byte;
            12: bytes_per_sector[15:8]<=boot_byte;
            13: sectors_per_cluster<=boot_byte;
            14: reserved_sectors[7:0]<=boot_byte;
            15: reserved_sectors[15:8]<=boot_byte;
            16: fat_count<=boot_byte;
            17: root_entries[7:0]<=boot_byte;
            18: root_entries[15:8]<=boot_byte;
            22: fat16_sectors[7:0]<=boot_byte;
            23: fat16_sectors[15:8]<=boot_byte;
            32: total_sectors[7:0]<=boot_byte;
            33: total_sectors[15:8]<=boot_byte;
            34: total_sectors[23:16]<=boot_byte;
            35: total_sectors[31:24]<=boot_byte;
            36: fat32_sectors[7:0]<=boot_byte;
            37: fat32_sectors[15:8]<=boot_byte;
            38: fat32_sectors[23:16]<=boot_byte;
            39: fat32_sectors[31:24]<=boot_byte;
            40: ext_flags[7:0]<=boot_byte;
            41: ext_flags[15:8]<=boot_byte;
            44: root_cluster[7:0]<=boot_byte;
            45: root_cluster[15:8]<=boot_byte;
            46: root_cluster[23:16]<=boot_byte;
            47: root_cluster[31:24]<={4'd0,boot_byte[3:0]};
            510: boot_signature[7:0]<=boot_byte;
            511: boot_signature[15:8]<=boot_byte;
            default: ;
        endcase
    end
end
// Fixed MBR byte offsets avoid subtract/index logic on every array write.
genvar partition_slot;
generate for(partition_slot=0;partition_slot<4;partition_slot=partition_slot+1) begin: g_partition
    always @(posedge clk) begin
        if(state[PROBE] && boot_byte_valid) begin
            case(rd_cnt)
                450+16*partition_slot: partition_type[partition_slot]<=boot_byte;
                454+16*partition_slot: partition_start[partition_slot][7:0]<=boot_byte;
                455+16*partition_slot: partition_start[partition_slot][15:8]<=boot_byte;
                456+16*partition_slot: partition_start[partition_slot][23:16]<=boot_byte;
                457+16*partition_slot: partition_start[partition_slot][31:24]<=boot_byte;
                default: ;
            endcase
        end
    end
end endgenerate

always @(posedge clk or posedge rst) begin
    if(rst) begin
        candidate_pending<=0; dir_end_seen<=0;
        entry_first<=0; entry_attr<=0; entry_ext0<=0; entry_ext1<=0; entry_ext2<=0;
        entry_cluster<=0; entry_size<=0; candidate_cluster<=0; candidate_size<=0;
        entry_number<=0; entry_has_number<=0; candidate_order<=32'hffffffff;
        entry_name<=0;name_parse_busy<=0;name_parse_phase<=0;
        name_parse_index<=0;name_digit<=0;name_times10<=0;
    end else if(state[DIR_BEGIN]) begin
        candidate_pending<=0; dir_end_seen<=0;
        name_parse_busy<=0;name_parse_phase<=0;
    end else begin
        // Complete8basename characters in16clocks after byte7. No input-mux
        // ->DSP*10->adder path, even for dense one-byte/clock sector models.
        if(name_parse_busy) begin
            if(!name_parse_phase) begin
                name_times10<=(entry_number<<3)+(entry_number<<1);
                name_digit<=entry_name[{name_parse_index,3'b000} +:8];
                name_parse_phase<=1;
            end else begin
                if(name_digit>=8'h30 && name_digit<=8'h39) begin
                    entry_number<=name_times10+{23'd0,name_digit[3:0]};entry_has_number<=1;
                end
                name_parse_phase<=0;
                if(name_parse_index==7) name_parse_busy<=0;
                else name_parse_index<=name_parse_index+1'b1;
            end
        end
        if(state[DIR_READ] && dir_byte_valid &&
                rd_cnt>=dir_resume_offset && !candidate_pending && !dir_end_seen) begin
        // Numeric order from the 8-byte FAT short basename, not directory order.
        // YL001, IMG001 and 1..12 all sort by their decimal digits.
        if(rd_cnt[4:0]==0) begin
            entry_number<=0; entry_has_number<=0;
        end
        if(rd_cnt[4:0]<8) begin
            entry_name[{rd_cnt[2:0],3'b000} +:8]<=dir_byte;
            if(rd_cnt[4:0]==7) begin name_parse_index<=0;name_parse_phase<=0;name_parse_busy<=1;end
        end
        case(rd_cnt[4:0])
            0: begin
                entry_first<=dir_byte;
                if(dir_byte==0) dir_end_seen<=1;
            end
            8: entry_ext0<=dir_byte;
            9: entry_ext1<=dir_byte;
            10: entry_ext2<=dir_byte;
            11: entry_attr<=dir_byte;
            20: entry_cluster[23:16]<=dir_byte;
            21: entry_cluster[31:24]<={4'd0,dir_byte[3:0]};
            26: entry_cluster[7:0]<=dir_byte;
            27: entry_cluster[15:8]<=dir_byte;
            28: entry_size[7:0]<=dir_byte;
            29: entry_size[15:8]<=dir_byte;
            30: entry_size[23:16]<=dir_byte;
            31: begin
                entry_size[31:24]<=dir_byte;
                if(entry_is_bmp && completed_entry_size>=54) begin
                    candidate_pending<=1;
                    candidate_cluster<=entry_cluster;
                    candidate_size<=completed_entry_size;
                    candidate_order<=entry_has_number ? {5'd0,entry_number} : 32'hffffffff;
                end
            end
            default: ;
        endcase
        end
    end
end

// Header/FAT payload registers also have no reset. Full sector completion
// gates their evaluation; stale data after reset/abort cannot validate a file.
always @(posedge clk) begin
    row_stride <= (width * 32'd3 + 32'd3) & 32'hfffffffc;
    // Bounded operands avoid a generic 32x32 multiplier.
    pixel_bytes <= row_stride[12:0] * height[10:0];
    pixel_end_offset <= {1'b0,pixel_offset}+{1'b0,pixel_bytes};
    header_match <= header_0=="B" && header_1=="M" &&
        width>=640 && width<=1920 && height>=480 && height<=1080 &&
        bmp_width==OUTPUT_WIDTH && bmp_height==OUTPUT_HEIGHT &&
        dib_size==40 && planes==1 && bit_count==24 && compression==0 &&
        pixel_offset>=54 && file_len>=pixel_offset &&
        !pixel_end_offset[32] && file_len>=pixel_end_offset[31:0];
    if(header_byte_valid) begin
        case(rd_cnt)
            0: header_0<=header_byte; 1: header_1<=header_byte;
            2: file_len[7:0]<=header_byte; 3: file_len[15:8]<=header_byte;
            4: file_len[23:16]<=header_byte; 5: file_len[31:24]<=header_byte;
            10: pixel_offset[7:0]<=header_byte;
            11: pixel_offset[15:8]<=header_byte;
            12: pixel_offset[23:16]<=header_byte;
            13: pixel_offset[31:24]<=header_byte;
            14: dib_size[7:0]<=header_byte; 15: dib_size[15:8]<=header_byte;
            16: dib_size[23:16]<=header_byte; 17: dib_size[31:24]<=header_byte;
            18: width[7:0]<=header_byte; 19: width[15:8]<=header_byte;
            20: width[23:16]<=header_byte; 21: width[31:24]<=header_byte;
            22: height[7:0]<=header_byte; 23: height[15:8]<=header_byte;
            24: height[23:16]<=header_byte; 25: height[31:24]<=header_byte;
            26: planes[7:0]<=header_byte; 27: planes[15:8]<=header_byte;
            28: bit_count[7:0]<=header_byte; 29: bit_count[15:8]<=header_byte;
            30: compression[7:0]<=header_byte; 31: compression[15:8]<=header_byte;
            32: compression[23:16]<=header_byte; 33: compression[31:24]<=header_byte;
            default: ;
        endcase
    end
end

always @(posedge clk) begin
    if(fat_byte_valid) begin
        if(rd_cnt=={1'b0,fat_entry_offset}) fat_entry[7:0]<=fat_byte;
        if(rd_cnt==({1'b0,fat_entry_offset}+10'd1)) fat_entry[15:8]<=fat_byte;
        if(rd_cnt==({1'b0,fat_entry_offset}+10'd2)) fat_entry[23:16]<=fat_byte;
        if(rd_cnt==({1'b0,fat_entry_offset}+10'd3)) fat_entry[31:24]<=fat_byte;
    end
end

// File position and scaler byte/row state persist across FAT lookups.
always @(posedge clk or posedge rst) begin
    if(rst) begin
        bmp_len_cnt<=0; sector_index<=0;
    end else begin
        if(!sd_init_done || load_abort || state[WRITE_WAIT]) begin
            bmp_len_cnt<=0; sector_index<=0;
        end else if(state[DATA_READ]) sector_index<=0;
        else if(state[DATA_FEED] && (bmp_len_cnt<pixel_offset || scaler_ready)) begin
            bmp_len_cnt<=bmp_len_cnt+1'b1;
            sector_index<=sector_index+1'b1;
        end
    end
end

always @(posedge clk or posedge rst) begin
    if(rst) begin
        state<=ST_IDLE; state_code<=0; sd_sec_read<=0; sd_sec_read_addr<=0;
        write_req<=0; scan_done<=0; scan_found_valid<=0;
        scan_found_sector<=0; scan_found_order<=0; scan_found_total<=0; load_failed<=0;
        error_code<=0; catalog_full<=0; cancel_pending<=0; geometry_valid<=0;geometry_bounds_valid_q<=0;
        volume_start<=0; data_start<=0; fat_start<=0; max_cluster<=0;
        partition_usable<=0; partition_pair0<=0; partition_pair1<=0;
        first_partition_q<=0; first_partition_present<=0;
        cluster_shift<=0; geometry_stage<=0; reserved_base<=0; fat_area<=0;
        data_base_sum<=0; fat_base_sum<=0; volume_end_sum<=0;
        data_sector_count<=0; data_cluster_count<=0;
        dir_cluster<=0; dir_lba<=0; dir_sector_in_cluster<=0;
        dir_resume_offset<=0; directory_reads<=0; candidate_lba<=0;
        cluster_target<=0; cluster_kind<=CL_DIR; cluster_offset<=0;
        cluster_lba_q<=0; cluster_number_bad<=0; cluster_address_bad<=0;
        chain_cluster<=0; fat_entry_offset<=0; fat_for_file<=0;
        fat_sector_bad_q<=0;fat_next_valid_q<=0;fat_next_eoc_q<=0;fat_read_address_q<=0;
        header_for_scan<=0; file_cluster<=0; file_sector_in_cluster<=0;
        load_relative_sector<=0; load_relative_cluster<=0;load_address_valid_q<=0;
    end else if(!sd_init_done) begin
        state<=ST_IDLE; state_code<=0; sd_sec_read<=0; write_req<=0;
        scan_done<=0; scan_found_valid<=0; scan_found_total<=0;
        cancel_pending<=0; geometry_valid<=0; error_code<=0;
        load_failed<=0; catalog_full<=0;
    end else if(load_abort) begin
        state<=ST_IDLE; state_code<=1; sd_sec_read<=0; write_req<=0;
        scan_found_valid<=0; load_failed<=0;
        cancel_pending<=sd_sec_read && !read_end_q;
    end else begin
        scan_found_valid<=0; load_failed<=0;
        // Every reachable state has exactly one bit set. Tell synthesis that
        // these branches are mutually exclusive, avoiding a37-way priority
        // chain. Keep default recovery; do not mark this case full_case.
        case(1'b1) // synopsys parallel_case
            state[IDLE]: begin
                state_code<=1; sd_sec_read<=0; write_req<=0;
                if(cancel_pending) begin
                    if(read_end_q) cancel_pending<=0;
                end else if(scan_start) begin
                    scan_done<=0; scan_found_total<=0; catalog_full<=0;
                    error_code<=0; geometry_valid<=0; volume_start<=0;
                    directory_reads<=0; sd_sec_read_addr<=0; state<=ST_PROBE;
                end else if(load_start) begin
                    error_code<=0; header_for_scan<=0;
                    // Break the32-bit range check -> deep FSM enable path.
                    // The existing address stage consumes this snapshot.
                    load_address_valid_q<=geometry_valid && load_sector>=data_start;
                    load_relative_sector<=load_sector-data_start;
                    sd_sec_read_addr<=load_sector; state<=ST_LOAD_ADDR1;
                end
            end
            state[LOAD_ADDR1]: begin
                if(load_address_valid_q) begin
                    load_relative_cluster<=load_relative_sector>>cluster_shift;
                    state<=ST_LOAD_ADDR2;
                end else begin load_failed<=1; error_code<=3;state<=ST_IDLE;end
            end
            state[LOAD_ADDR2]: begin
                file_cluster<=load_relative_cluster+32'd2;
                file_sector_in_cluster<=0; state<=ST_HEADER_READ;
            end
            state[PROBE],state[BOOT]: begin
                state_code<=2; sd_sec_read<=1;
                if(read_end_q) begin
                    sd_sec_read<=0; state<=(state[PROBE]) ? ST_PROBE_EVAL : ST_BOOT_EVAL;
                end
            end
            state[PROBE_EVAL]: begin
                if(sector_complete && boot_valid) begin geometry_stage<=0; state<=ST_GEOMETRY; end
                else if(sector_complete && boot_signature==16'haa55) state<=ST_PART_FLAGS;
                else begin scan_done<=1; error_code<=3; state<=ST_IDLE; end
            end
            state[PART_FLAGS]: begin
                partition_usable[0]<=((partition_type[0]==8'h0b || partition_type[0]==8'h0c) && partition_start[0]!=0);
                partition_usable[1]<=((partition_type[1]==8'h0b || partition_type[1]==8'h0c) && partition_start[1]!=0);
                partition_usable[2]<=((partition_type[2]==8'h0b || partition_type[2]==8'h0c) && partition_start[2]!=0);
                partition_usable[3]<=((partition_type[3]==8'h0b || partition_type[3]==8'h0c) && partition_start[3]!=0);
                state<=ST_PART_PAIRS;
            end
            state[PART_PAIRS]: begin
                partition_pair0<=partition_usable[0] ? partition_start[0] :
                    (partition_usable[1] ? partition_start[1] : 32'd0);
                partition_pair1<=partition_usable[2] ? partition_start[2] :
                    (partition_usable[3] ? partition_start[3] : 32'd0);
                state<=ST_PART_PICK;
            end
            state[PART_PICK]: begin
                first_partition_q<=partition_pair0!=0 ? partition_pair0 : partition_pair1;
                state<=ST_PART_CHECK;
            end
            state[PART_CHECK]: begin
                first_partition_present<=first_partition_q!=0;
                state<=ST_PART_APPLY;
            end
            state[PART_APPLY]: begin
                if(first_partition_present) begin
                    volume_start<=first_partition_q; sd_sec_read_addr<=first_partition_q;
                    state<=ST_BOOT;
                end else begin scan_done<=1; error_code<=3; state<=ST_IDLE; end
            end
            state[BOOT_EVAL]: begin
                if(sector_complete && boot_valid) begin geometry_stage<=0; state<=ST_GEOMETRY; end
                else begin scan_done<=1; error_code<=3; state<=ST_IDLE; end
            end
            state[GEOMETRY]: begin
                case(geometry_stage)
                    0: begin
                        reserved_base<={1'b0,volume_start}+{17'd0,reserved_sectors};
                        fat_area<=fat_count==2 ? {fat32_sectors,1'b0} : {1'b0,fat32_sectors};
                        volume_end_sum<={1'b0,volume_start}+{1'b0,total_sectors};
                        cluster_shift<=cluster_log2(sectors_per_cluster);
                    end
                    1: begin
                        data_base_sum<=reserved_base+fat_area;
                        fat_base_sum<=reserved_base+
                            ((ext_flags[7] && ext_flags[3:0]==1) ? {1'b0,fat32_sectors} : 33'd0);
                    end
                    2: begin
                        data_start<=data_base_sum[31:0]; fat_start<=fat_base_sum[31:0];
                        data_sector_count<=volume_end_sum-data_base_sum;
                    end
                    3: data_cluster_count<=data_sector_count>>cluster_shift;
                    4: max_cluster<=data_cluster_count+32'd1;
                    5: geometry_bounds_valid_q<=!volume_end_sum[32] && !data_base_sum[32] &&
                        data_base_sum<volume_end_sum && data_cluster_count!=0 &&
                        max_cluster<32'h0ffffff0 && root_cluster<=max_cluster;
                    default: ;
                endcase
                if(geometry_stage==5) state<=ST_GEOMETRY_CHECK;
                else geometry_stage<=geometry_stage+1'b1;
            end
            state[GEOMETRY_CHECK]: begin
                if(geometry_bounds_valid_q) begin
                    geometry_valid<=1; dir_cluster<=root_cluster;
                    cluster_target<=root_cluster; cluster_kind<=CL_DIR;
                    state<=ST_CLUSTER_ADDR;
                end else begin scan_done<=1; error_code<=3; state<=ST_IDLE; end
            end
            state[CLUSTER_ADDR]: begin
                cluster_offset<=({1'b0,cluster_target}-33'd2)<<cluster_shift;
                state<=ST_CLUSTER_SUM;
            end
            // The previous STA report found add -> compare -> state decoding
            // on one 100 MHz path. Each arithmetic/decision step now registers.
            state[CLUSTER_SUM]: begin
                cluster_lba_q<=cluster_lba_sum;
                cluster_number_bad<=cluster_target<2 || cluster_target>max_cluster;
                state<=ST_CLUSTER_CHECK;
            end
            state[CLUSTER_CHECK]: begin
                cluster_address_bad<=cluster_lba_q[32] ||
                    cluster_lba_q>=volume_end_sum || cluster_number_bad;
                state<=ST_CLUSTER_ISSUE;
            end
            state[CLUSTER_ISSUE]: begin
                if(cluster_address_bad) begin
                    error_code<=4;
                    if(cluster_kind==CL_DATA) load_failed<=1; else scan_done<=1;
                    state<=ST_IDLE;
                end else begin
                    sd_sec_read_addr<=cluster_lba_q[31:0];
                    case(cluster_kind)
                        CL_DIR: begin
                            dir_lba<=cluster_lba_q[31:0]; dir_sector_in_cluster<=0;
                            dir_resume_offset<=0; state<=ST_DIR_BEGIN;
                        end
                        CL_HEADER: begin
                            candidate_lba<=cluster_lba_q[31:0];
                            header_for_scan<=1; state<=ST_HEADER_READ;
                        end
                        default: begin file_sector_in_cluster<=0; state<=ST_DATA_READ; end
                    endcase
                end
            end
            state[DIR_BEGIN]: begin
                state_code<=2;
                if(directory_reads>=MAX_DIRECTORY_READS) begin
                    error_code<=7; scan_done<=1; state<=ST_IDLE;
                end else begin directory_reads<=directory_reads+1'b1; state<=ST_DIR_READ; end
            end
            state[DIR_READ]: begin
                sd_sec_read<=1;
                // Capture the resume point only when a candidate entry completes.
                if(dir_byte_valid && rd_cnt>=dir_resume_offset &&
                    !candidate_pending && !dir_end_seen && rd_cnt[4:0]==31 &&
                    entry_is_bmp && completed_entry_size>=54)
                    dir_resume_offset<=rd_cnt+10'd1;
                if(read_end_q) begin sd_sec_read<=0; state<=ST_DIR_EVAL; end
            end
            state[DIR_EVAL]: begin
                if(!sector_complete) begin scan_done<=1; error_code<=3; state<=ST_IDLE; end
                else if(candidate_pending) begin
                    cluster_target<=candidate_cluster; cluster_kind<=CL_HEADER; state<=ST_CLUSTER_ADDR;
                end else if(dir_end_seen) begin scan_done<=1; state<=ST_IDLE; end
                else state<=ST_DIR_NEXT;
            end
            state[RETURN_DIR]: begin
                if(dir_resume_offset>=512) state<=ST_DIR_NEXT;
                else begin sd_sec_read_addr<=dir_lba; state<=ST_DIR_BEGIN; end
            end
            state[DIR_NEXT]: begin
                dir_resume_offset<=0;
                if(({1'b0,dir_sector_in_cluster}+9'd1)<{1'b0,sectors_per_cluster}) begin
                    dir_lba<=dir_lba+1'b1; dir_sector_in_cluster<=dir_sector_in_cluster+1'b1;
                    sd_sec_read_addr<=dir_lba+1'b1; state<=ST_DIR_BEGIN;
                end else begin chain_cluster<=dir_cluster; fat_for_file<=0; state<=ST_FAT_ADDR; end
            end
            state[HEADER_READ]: begin
                state_code<=2; sd_sec_read<=1;
                if(read_end_q) begin sd_sec_read<=0; state<=ST_HEADER_EVAL; end
            end
            state[HEADER_EVAL]: begin
                if(!sector_complete) begin
                    if(header_for_scan) begin scan_done<=1;error_code<=3;end
                    else begin load_failed<=1;error_code<=5;end
                    state<=ST_IDLE;
                end else if(header_for_scan) begin
                    if(header_match && file_len<=candidate_size) begin
                        scan_found_valid<=1; scan_found_sector<=candidate_lba;
                        scan_found_order<=candidate_order;
                        scan_found_total<=scan_found_total+1'b1;
                        if(scan_found_total>=IMAGE_CAPACITY-1'b1 ||
                            (scan_target_count!=0 && scan_found_total+1'b1>=scan_target_count)) begin
                            scan_done<=1; catalog_full<=scan_found_total>=IMAGE_CAPACITY-1'b1;
                            state<=ST_IDLE;
                        end else state<=ST_RETURN_DIR;
                    end else state<=ST_RETURN_DIR;
                end else if(header_match) begin
                    write_req<=1; state<=ST_WRITE_WAIT;
                end else begin error_code<=5; load_failed<=1; state<=ST_IDLE; end
            end
            state[WRITE_WAIT]: begin
                state_code<=3;
                if(write_req_ack) begin write_req<=0; state<=ST_WRITE_RELEASE; end
            end
            state[WRITE_RELEASE]: begin
                state_code<=3;
                if(!write_req_ack) state<=ST_DATA_READ;
            end
            state[DATA_READ]: begin
                state_code<=4; sd_sec_read<=1;
                if(read_end_q) begin
                    sd_sec_read<=0;
                    if(rd_cnt==512 || (rd_cnt==511 && read_valid_q)) state<=ST_DATA_FETCH;
                    else begin load_failed<=1;error_code<=5;state<=ST_IDLE;end
                end
            end
            state[DATA_FETCH]: state<=ST_DATA_FEED;
            state[DATA_FEED]: begin
                if(scaler_done) state<=ST_IDLE;
                else if(pixels_complete) state<=ST_DATA_FINISH;
                else if(bmp_len_cnt<pixel_offset || scaler_ready)
                    state<=sector_index==511 ? ST_DATA_EVAL : ST_DATA_FETCH;
            end
            state[DATA_FINISH]: begin
                if(scaler_done) state<=ST_IDLE;
            end
            state[DATA_EVAL]: begin
                if(scaler_done) state<=ST_IDLE;
                else if(pixels_complete) state<=ST_DATA_FINISH;
                else if(file_sector_in_cluster<last_sector_in_cluster) begin
                    file_sector_in_cluster<=file_sector_in_cluster+1'b1;
                    sd_sec_read_addr<=sd_sec_read_addr+1'b1; state<=ST_DATA_READ;
                end else begin chain_cluster<=file_cluster; fat_for_file<=1; state<=ST_FAT_ADDR; end
            end
            state[FAT_ADDR]: begin
                state_code<=5;
                // Separate32-bit comparison/addition from request/state muxes.
                fat_sector_bad_q<=fat_sector_index>=fat32_sectors;
                fat_read_address_q<=fat_start+fat_sector_index;
                fat_entry_offset<={chain_cluster[6:0],2'b00};state<=ST_FAT_USE;
            end
            state[FAT_USE]: begin
                if(fat_sector_bad_q) begin
                    error_code<=4;
                    if(fat_for_file) load_failed<=1; else scan_done<=1;
                    state<=ST_IDLE;
                end else begin
                    sd_sec_read_addr<=fat_read_address_q;state<=ST_FAT_READ;
                end
            end
            state[FAT_READ]: begin
                sd_sec_read<=1;
                if(read_end_q) begin sd_sec_read<=0; state<=ST_FAT_EVAL; end
            end
            state[FAT_EVAL]: begin
                // fat_entry remains stable until the next FAT_READ transaction.
                fat_next_valid_q<=next_cluster_valid;
                fat_next_eoc_q<=next_cluster>=32'h0ffffff8;
                state<=ST_FAT_APPLY;
            end
            state[FAT_APPLY]: begin
                if(!sector_complete) begin
                    error_code<=4;state<=ST_IDLE;
                    if(fat_for_file) load_failed<=1;else scan_done<=1;
                end else if(fat_next_eoc_q && !fat_for_file) begin scan_done<=1; state<=ST_IDLE; end
                else if(!fat_next_valid_q) begin
                    error_code<=4;
                    if(fat_for_file) load_failed<=1; else scan_done<=1;
                    state<=ST_IDLE;
                end else begin
                    cluster_target<=next_cluster;
                    if(fat_for_file) begin file_cluster<=next_cluster; cluster_kind<=CL_DATA; end
                    else begin dir_cluster<=next_cluster; cluster_kind<=CL_DIR; end
                    state<=ST_CLUSTER_ADDR;
                end
            end
            default: begin state<=ST_IDLE; sd_sec_read<=0; write_req<=0; end
        endcase
    end
end
endmodule

// One output line of storage; no full-resolution source frame in SDRAM.
// Inputs are at least640x480 and at most1920x1080. Repeat samples/lines for
// upscaling and skip samples/lines for downscaling. FILL_MODE selects center
// cropping; legacy fit preserves all content and adds borders as necessary.
// Integer DDA: sx=crop_left+floor(dx*region_width/dw), likewise for bottom-up y.
// Byte backpressure is handled by the caller's complete-sector buffer.
module album_bmp_scaler #(
    parameter integer PIXEL_PERIOD=96, // board uses 4 with write-side FIFO feedback
    parameter integer OUTPUT_WIDTH=640,
    parameter integer OUTPUT_HEIGHT=480,
    parameter integer FILL_MODE=0,
    parameter integer WRITE_BACKPRESSURE=0
)(
    input clk, rst, start,
    input [11:0] source_width,
    input [10:0] source_height,
    input [12:0] row_stride,
    input [7:0] byte_data,
    input byte_valid,
    input pixel_ready, // permission to issue registered write pulse; FIFO reserves headroom
    output byte_ready,
    output reg [23:0] pixel_data,
    output reg pixel_valid, done,
    output [11:0] crop_width,crop_height
);
localparam [3:0] S_IDLE=0,S_PRODUCTS=1,S_CHOOSE=2,S_DIVIDE=3,S_LAYOUT=4,
    S_BEGIN=5,S_BYTES=6,S_PIXEL=7,S_XSTEP=8,S_ROWCHECK=9,S_EMIT=10,
    S_YSTEP=11,S_DONE=12;
reg [3:0] state;
// Freeze the BMP geometry for this frame; all fit/padding/sample calculations
// use the same captured header, including consecutive loads of different sizes.
reg [11:0] frame_width;
reg [10:0] frame_height;
reg [12:0] frame_stride;
reg [11:0] region_width,region_height,crop_left,crop_bottom;
assign crop_width=region_width;
assign crop_height=region_height;
reg [20:0] wide_product, tall_product, remainder;
reg [11:0] divisor;
reg [10:0] quotient, dest_width, dest_height, left_margin, bottom_margin;
reg width_limited;
reg [11:0] src_x, src_y, target_x, target_y;
reg [12:0] row_position, x_remainder;
reg [11:0] y_remainder;
reg [1:0] phase;
reg [23:0] incoming_pixel;
reg [10:0] dest_x, dest_y, output_x, output_y;
reg row_has_data;
reg [7:0] emit_div;
reg [23:0] line_pixels [0:OUTPUT_WIDTH-1]; /* fehdl force_ram=1, ram_style="bram" */
reg [23:0] line_q;
wire in_picture = output_x>=left_margin && output_x<left_margin+dest_width;
wire [10:0] line_address=output_x-left_margin;
assign byte_ready=state==S_BYTES;

// Synchronous reads and a single write port permit BRAM inference.
always @(posedge clk) begin
    if(state==S_PIXEL && src_y==target_y && src_x==target_x && dest_x<dest_width)
        line_pixels[dest_x]<=incoming_pixel;
    if(state==S_EMIT && in_picture) line_q<=line_pixels[line_address];
end

always @(posedge clk or posedge rst) begin
    if(rst) begin
        state<=S_IDLE; done<=0; pixel_valid<=0; pixel_data<=0;
        frame_width<=0; frame_height<=0; frame_stride<=0;
        region_width<=0; region_height<=0; crop_left<=0; crop_bottom<=0;
        wide_product<=0; tall_product<=0; remainder<=0; divisor<=1;
        quotient<=0; dest_width<=0; dest_height<=0; left_margin<=0; bottom_margin<=0;
        width_limited<=0; src_x<=0; src_y<=0; target_x<=0; target_y<=0;
        row_position<=0; x_remainder<=0; y_remainder<=0; phase<=0; incoming_pixel<=0;
        dest_x<=0; dest_y<=0; output_x<=0; output_y<=0; row_has_data<=0; emit_div<=0;
    end else begin
        pixel_valid<=0;
        if(start) begin
            frame_width<=source_width; frame_height<=source_height; frame_stride<=row_stride;
            state<=S_PRODUCTS; done<=0; src_x<=0; src_y<=0; target_x<=0; target_y<=0;
            row_position<=0; x_remainder<=0; y_remainder<=0; phase<=0;
            dest_x<=0; dest_y<=0; output_x<=0; output_y<=0; emit_div<=0; row_has_data<=0;
        end else case(state)
            S_PRODUCTS: begin
                wide_product<=frame_width*OUTPUT_HEIGHT;
                tall_product<=frame_height*OUTPUT_WIDTH;
                state<=S_CHOOSE;
            end
            S_CHOOSE: begin
                width_limited<=wide_product>=tall_product;
                remainder<=wide_product>=tall_product ? tall_product : wide_product;
                if(FILL_MODE!=0)
                    divisor<=wide_product>=tall_product ? OUTPUT_HEIGHT : OUTPUT_WIDTH;
                else divisor<=wide_product>=tall_product ? frame_width : {1'b0,frame_height};
                quotient<=0; state<=S_DIVIDE;
            end
            // Bounded setup division once per image, no combinational divider.
            S_DIVIDE: begin
                if(remainder>={9'd0,divisor}) begin
                    remainder<=remainder-{9'd0,divisor}; quotient<=quotient+1'b1;
                end else begin
                    if(FILL_MODE!=0) begin
                        region_width<=width_limited ? {1'b0,quotient} : frame_width;
                        region_height<=width_limited ? {1'b0,frame_height} : {1'b0,quotient};
                        dest_width<=OUTPUT_WIDTH; dest_height<=OUTPUT_HEIGHT;
                    end else begin
                        region_width<=frame_width; region_height<={1'b0,frame_height};
                        dest_width<=width_limited ? OUTPUT_WIDTH : quotient;
                        dest_height<=width_limited ? quotient : OUTPUT_HEIGHT;
                    end
                    state<=S_LAYOUT;
                end
            end
            S_LAYOUT: begin
                left_margin<=(OUTPUT_WIDTH-dest_width)>>1;
                bottom_margin<=(OUTPUT_HEIGHT-dest_height)>>1;
                crop_left<=(frame_width-region_width)>>1;
                crop_bottom<=({1'b0,frame_height}-region_height)>>1;
                state<=S_BEGIN;
            end
            S_BEGIN: begin
                target_x<=crop_left; target_y<=crop_bottom;
                state<=bottom_margin==0 ? S_BYTES : S_EMIT;
            end
            S_BYTES: if(byte_valid) begin
                row_position<=row_position+1'b1;
                state<=S_ROWCHECK;
                if(src_x<frame_width) begin
                    case(phase)
                        0: incoming_pixel[7:0]<=byte_data;
                        1: incoming_pixel[15:8]<=byte_data;
                        2: begin incoming_pixel[23:16]<=byte_data; state<=S_PIXEL; end
                        default: ;
                    endcase
                    phase<=phase==2 ? 2'd0 : phase+1'b1;
                end
            end
            S_PIXEL: begin
                if(src_y==target_y && src_x==target_x && dest_x<dest_width) begin
                    dest_x<=dest_x+1'b1;
                    x_remainder<=x_remainder+{1'b0,region_width}; state<=S_XSTEP;
                end else begin src_x<=src_x+1'b1; state<=S_ROWCHECK; end
            end
            S_XSTEP: begin
                if(x_remainder>={2'd0,dest_width}) begin
                    x_remainder<=x_remainder-{2'd0,dest_width}; target_x<=target_x+1'b1;
                end else if(target_x==src_x && dest_x<dest_width) state<=S_PIXEL;
                else begin src_x<=src_x+1'b1; state<=S_ROWCHECK; end
            end
            S_ROWCHECK: begin
                if(row_position==frame_stride) begin
                    src_y<=src_y+1'b1; src_x<=0; row_position<=0; phase<=0;
                    dest_x<=0; target_x<=crop_left; x_remainder<=0;
                    if(src_y==target_y) begin
                        row_has_data<=1; output_x<=0; emit_div<=0; state<=S_EMIT;
                    end else state<=S_BYTES;
                end else state<=S_BYTES;
            end
            S_EMIT: begin
                if(emit_div==PIXEL_PERIOD-1 && (WRITE_BACKPRESSURE==0 || pixel_ready)) begin
                    emit_div<=0; pixel_valid<=1;
                    pixel_data<=row_has_data && in_picture ? line_q : 24'd0;
                    if(output_x==OUTPUT_WIDTH-1) begin
                        output_x<=0; output_y<=output_y+1'b1;
                        if(output_y==OUTPUT_HEIGHT-1) begin done<=1; state<=S_DONE; end
                        else if(row_has_data) begin
                            dest_y<=dest_y+1'b1;
                            if(dest_y+1'b1==dest_height) row_has_data<=0;
                            else begin
                                y_remainder<=y_remainder+region_height; state<=S_YSTEP;
                            end
                        end else if(output_y+1'b1==bottom_margin) state<=S_BYTES;
                    end else output_x<=output_x+1'b1;
                end else if(emit_div!=PIXEL_PERIOD-1) emit_div<=emit_div+1'b1;
            end
            S_YSTEP: begin
                if(y_remainder>={1'd0,dest_height}) begin
                    y_remainder<=y_remainder-{1'd0,dest_height}; target_y<=target_y+1'b1;
                end else if(target_y+1'b1==src_y) state<=S_EMIT;
                else state<=S_BYTES;
            end
            S_DONE: ;
            default: state<=S_IDLE;
        endcase
    end
end
endmodule
