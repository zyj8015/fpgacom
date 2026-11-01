module sd_card_bmp #(
    parameter integer CLK_FREQ_HZ = 100_000_000,
    parameter integer ENABLE_WAV = 0, // top enables WAV; legacy image tests keep timer mode
    parameter [31:0] SCAN_START_SECTOR = 32'd0,
    parameter [31:0] SCAN_MAX_SECTOR = 32'd131071,
    parameter [5:0] SCAN_TARGET_COUNT = 6'd0,
    parameter integer MAX_IMAGES = 32,
    parameter integer AUTO_INTERVAL_MS = 3000,
    parameter integer LOAD_TIMEOUT_SECONDS = 8,
    parameter integer OUTPUT_WIDTH=640,
    parameter integer OUTPUT_HEIGHT=480,
    parameter integer FILL_MODE=0,
    parameter integer WRITE_BACKPRESSURE=0,
    parameter integer PIXEL_PERIOD=96,
    parameter integer ENABLE_PREFETCH=0,
    parameter integer COMBINED_PAGE_KEY=0,
    parameter integer AUTO_AUDIO_GAP_MS=0,
    parameter integer ENABLE_TRANSITIONS=0,
    parameter integer ENABLE_WRITE_CANCEL=0
)(
    input clk, rst, key_next, key_previous, key_auto,
    input [15:0] bmp_width, bmp_height,
    output reg display_valid,
    output [3:0] state_code,
    output [4:0] reader_state,
    output auto_status,
    output [5:0] image_count,
    output [4:0] image_index,
    output reg [3:0] fault_code,
    input write_finish_toggle,
    output reg [1:0] write_buf_idx, disp_buf_idx,
    output write_req,
    input write_req_ack,
    input write_ready,
    input [1:0] active_display_buf, // video frame-latched buffer; synchronized below
    output write_en,
    output [31:0] write_data,
    output SD_nCS, SD_DCLK, SD_MOSI,
    input SD_MISO,
    output pcm_send, output [15:0] pcm_data, output pcm_last,pcm_mute,
    input pcm_ready,pcm_last_played,
    output [3:0] audio_fault,
    output reg [15:0] display_source_width,display_source_height,
    output audio_complete,
    output reg [15:0] display_crop_width,display_crop_height,
    output [15:0] audio_elapsed_cs,
    output reg page_epoch,
    output [8:0] transition_command, // request, ticket, automatic, destination page
    input [1:0] transition_reply,   // coherent CDC: covered, matching ticket
    output write_cancel,
    input write_cancel_ack
);
localparam [31:0] AUTO_CYCLES=(CLK_FREQ_HZ/1000)*AUTO_INTERVAL_MS;
localparam [31:0] AUTO_GAP_CYCLES=(CLK_FREQ_HZ/1000)*AUTO_AUDIO_GAP_MS;
localparam [31:0] LOAD_IDLE_CYCLES=CLK_FREQ_HZ*LOAD_TIMEOUT_SECONDS;
wire key_next_press, key_prev_press, key_auto_press;
wire page_gesture_start;
wire sd_sec_read;
wire [31:0] sd_sec_read_addr;
wire [7:0] sd_sec_read_data;
wire sd_sec_read_data_valid, sd_sec_read_end, sd_init_done;
wire bmp_data_wr_en, bmp_ready, scan_done, scan_found_valid;
wire card_req,card_valid,card_end;
wire [31:0] card_addr;
wire [7:0] card_data;
wire audio_req,audio_valid,audio_end,audio_sector_busy;
wire image_sector_busy;
wire [31:0] audio_addr;
wire [7:0] audio_data;
wire wav_done;
wire [15:0] wav_elapsed_cs;
wire fs_valid;
wire [31:0] fs_data_start,fs_fat_start,fs_root_cluster;
wire [31:0] fs_max_cluster,fs_volume_end,fs_fat_sectors;
wire [3:0] fs_cluster_shift;
wire [7:0] fs_spc;
reg track_start,track_stop;
assign audio_elapsed_cs=track_start ? 16'd0 : wav_elapsed_cs;
reg [31:0] track_number,load_track_number;
wire [23:0] bmp_data;
wire [15:0] bmp_source_width,bmp_source_height;
wire [15:0] bmp_crop_width,bmp_crop_height;
assign audio_complete=wav_done;
wire [31:0] scan_found_sector, scan_found_order;
wire [5:0] scan_found_total;
wire bmp_load_failed, catalog_full;
wire [3:0] bmp_error;
reg scan_start_pulse, load_start_pulse, scan_kicked;
reg previous_pending;
reg catalog_sorted;
reg [3:0] sort_phase;
reg sort_swap;
reg [4:0] sort_index;
reg [5:0] sort_limit;
reg [31:0] sort_left_sector, sort_right_sector, sort_left_key, sort_right_key;

reg first_image_committed, auto_play_en, next_req_pending, load_busy, load_abort;
reg [31:0] load_sector, auto_cnt, load_timeout_cnt;
reg [5:0] img_found_count;
reg [4:0] img_idx, load_idx;
reg [1:0] pending_buf_idx, load_select_phase;
// One packed catalog, one synchronous read and one write port. No reset
// on memory contents: only slots below img_found_count are valid.
reg [63:0] catalog_memory [0:MAX_IMAGES-1]; /* fehdl force_ram=1, ram_style="bram" */
reg [63:0] catalog_read_data;
reg catalog_write_en;
reg [4:0] catalog_write_addr;
reg [63:0] catalog_write_data;
wire [4:0] catalog_read_addr = catalog_sorted ? load_idx :
    ((sort_phase==3 || sort_phase==4) ? sort_index+5'd1 : sort_index);
reg source_done_seen, write_done_seen, load_started_seen, write_armed_seen;
reg cache_valid, prefetch_previous, prefetch_blocked, cancel_wait, narration_armed, request_is_auto;
(* ASYNC_REG="TRUE" *) reg [1:0] cancel_ack_sync;
assign write_cancel=ENABLE_WRITE_CANCEL!=0 && cancel_wait;
always @(posedge clk or posedge rst) begin
    if(rst) cancel_ack_sync<=0;
    else cancel_ack_sync<={cancel_ack_sync[0],write_cancel_ack};
end
reg cache_commit_candidate,commit_guard;
reg transition_request,transition_ticket;
reg [5:0] transition_page;
reg transition_automatic;
assign transition_command={transition_request,transition_ticket,transition_automatic,transition_page};
wire transition_commit_ok=ENABLE_TRANSITIONS==0 || !first_image_committed ||
    (transition_request && transition_reply[1] && transition_reply[0]==transition_ticket);
reg [4:0] cache_idx, requested_idx;
reg [1:0] cache_buf;
reg [31:0] cache_track;
reg [15:0] cache_width,cache_height,cache_crop_width,cache_crop_height;
(* ASYNC_REG="TRUE" *) reg [1:0] active_buf_meta,active_buf_sync;
reg ack_last;
(* ASYNC_REG="TRUE" *) reg [1:0] write_req_ack_sync;
(* ASYNC_REG="TRUE" *) reg [2:0] wrfin_tgl_sync;
wire write_finish_pulse=wrfin_tgl_sync[2]^wrfin_tgl_sync[1];
wire [5:0] incremented_index={1'b0,img_idx}+6'd1;
wire [4:0] next_from_current =
    incremented_index>=img_found_count ? 5'd0 : incremented_index[4:0];
wire [4:0] previous_from_current=img_idx==0 ? img_found_count[4:0]-5'd1 : img_idx-5'd1;
wire [4:0] prefetch_idx = !auto_play_en && prefetch_previous ? previous_from_current : next_from_current;
wire auto_tick=auto_cnt>=AUTO_CYCLES-1'b1;
// Cut target comparison out of the metadata/audio commit enable path. Every
// event that changes its inputs inserts a guard cycle, so the registered
// candidate can never commit a cache tag from before a retarget/cancellation.
always @(posedge clk or posedge rst) begin
    if(rst) cache_commit_candidate<=0;
    else cache_commit_candidate<=sd_init_done && !load_busy && !cancel_wait && cache_valid &&
        (!first_image_committed || (next_req_pending && cache_idx==requested_idx));
end
assign write_en=bmp_data_wr_en;
assign write_data={bmp_data,8'd0};
assign auto_status=auto_play_en;
assign image_count=img_found_count;
assign image_index=img_idx;

// Serialize swap writes; two dynamic writes per cycle prevented a simple
// RAM implementation and expanded the previous catalog into logic/muxes.
always @* begin
    catalog_write_en=0; catalog_write_addr=0; catalog_write_data=0;
    if(!rst && sd_init_done) begin
        if(scan_found_valid && img_found_count<MAX_IMAGES) begin
            catalog_write_en=1; catalog_write_addr=img_found_count[4:0];
            catalog_write_data={scan_found_order,scan_found_sector};
        end else if(sort_swap && sort_phase==6) begin
            catalog_write_en=1; catalog_write_addr=sort_index;
            catalog_write_data={sort_right_key,sort_right_sector};
        end else if(sort_swap && sort_phase==7) begin
            catalog_write_en=1; catalog_write_addr=sort_index+5'd1;
            catalog_write_data={sort_left_key,sort_left_sector};
        end
    end
end
always @(posedge clk) begin
    if(catalog_write_en) catalog_memory[catalog_write_addr]<=catalog_write_data;
    catalog_read_data<=catalog_memory[catalog_read_addr];
end
// Stable bubble sort with explicit synchronous-RAM read latency, registered
// comparison, and one swap write per cycle. At most 32 initialized entries.
always @(posedge clk or posedge rst) begin
    if(rst) begin
        catalog_sorted<=0; sort_phase<=0; sort_index<=0; sort_limit<=0; sort_swap<=0;
        sort_left_sector<=0; sort_right_sector<=0; sort_left_key<=0; sort_right_key<=0;
    end else if(!sd_init_done || scan_start_pulse) begin
        catalog_sorted<=0; sort_phase<=0; sort_index<=0; sort_limit<=0; sort_swap<=0;
    end else if(!catalog_sorted && scan_done && !scan_found_valid) begin
        case(sort_phase)
            0: begin
                if(img_found_count<2) catalog_sorted<=1;
                else begin sort_limit<=img_found_count; sort_index<=0; sort_phase<=1; end
            end
            1: sort_phase<=2; // issue/wait for left synchronous read
            2: begin
                sort_left_sector<=catalog_read_data[31:0];
                sort_left_key<=catalog_read_data[63:32]; sort_phase<=3;
            end
            3: sort_phase<=4; // issue/wait for right synchronous read
            4: begin
                sort_right_sector<=catalog_read_data[31:0];
                sort_right_key<=catalog_read_data[63:32]; sort_phase<=5;
            end
            5: begin sort_swap<=sort_left_key>sort_right_key; sort_phase<=6; end
            6: sort_phase<=7; // write left slot
            7: sort_phase<=8; // write right slot
            8: begin
                sort_swap<=0;
                if({1'b0,sort_index}+6'd2>=sort_limit) begin
                    if(sort_limit<=2) begin catalog_sorted<=1; sort_phase<=0; end
                    else begin sort_limit<=sort_limit-1'b1; sort_index<=0; sort_phase<=1; end
                end else begin sort_index<=sort_index+1'b1; sort_phase<=1; end
            end
            default: sort_phase<=0;
        endcase
    end
end

always @(posedge clk or posedge rst) begin
    if(rst) begin write_req_ack_sync<=0; wrfin_tgl_sync<=0; active_buf_meta<=0; active_buf_sync<=0; end
    else begin
        write_req_ack_sync<={write_req_ack_sync[0],write_req_ack};
        wrfin_tgl_sync<={wrfin_tgl_sync[1:0],write_finish_toggle};
        active_buf_meta<=active_display_buf; active_buf_sync<=active_buf_meta;
    end
end

generate if(COMBINED_PAGE_KEY!=0) begin: g_combined_page
    album_short_long_button #(.CLK_FREQ_HZ(CLK_FREQ_HZ),.DEBOUNCE_MS(20),.LONG_MS(800)) u_page_button(
        .clk(clk),.rst(rst),.button_in(key_next),.short_pulse(key_next_press),
        .long_pulse(key_prev_press),.press_pulse(page_gesture_start),.button_down());
end else begin: g_separate_page
assign page_gesture_start=1'b0;
key_press_debounce #(.CLK_FREQ_HZ(CLK_FREQ_HZ),.DEBOUNCE_MS(20)) u_key_next(
    .clk(clk),.rst(rst),.button_in(key_next),.press_pulse(key_next_press));
key_press_debounce #(.CLK_FREQ_HZ(CLK_FREQ_HZ),.DEBOUNCE_MS(20)) u_key_previous(
    .clk(clk),.rst(rst),.button_in(key_previous),.press_pulse(key_prev_press));
end endgenerate
key_press_debounce #(.CLK_FREQ_HZ(CLK_FREQ_HZ),.DEBOUNCE_MS(50)) u_key_auto(
    .clk(clk),.rst(rst),.button_in(key_auto),.press_pulse(key_auto_press));

always @(posedge clk or posedge rst) begin
    if(rst) auto_play_en<=0;
    else if((ENABLE_WAV!=0 && audio_fault!=0) || (ENABLE_PREFETCH!=0 && prefetch_blocked) ||
            key_next_press || key_prev_press || page_gesture_start) auto_play_en<=0;
    else if(key_auto_press) auto_play_en<=~auto_play_en;
end

// One displayed frame plus one tagged cache. Background completion never
// publishes a page or starts its WAV. Only a requested cache hit commits it.
always @(posedge clk or posedge rst) begin
    if(rst) begin
        ack_last<=0; scan_start_pulse<=0; load_start_pulse<=0;
        track_start<=0; track_stop<=0; track_number<=0; load_track_number<=0;
        display_source_width<=0; display_source_height<=0;
        display_crop_width<=0; display_crop_height<=0;
        load_sector<=0; scan_kicked<=0; first_image_committed<=0;
        auto_cnt<=0; img_found_count<=0; img_idx<=0; load_idx<=0;
        pending_buf_idx<=0; write_buf_idx<=0; disp_buf_idx<=0;
        next_req_pending<=0; previous_pending<=0; load_busy<=0; load_timeout_cnt<=0; load_abort<=0;
        source_done_seen<=0; write_done_seen<=0; load_started_seen<=0;
        write_armed_seen<=0; display_valid<=0; fault_code<=0; load_select_phase<=0;
        cache_valid<=0; cache_idx<=0; cache_buf<=0; cache_track<=0;
        cache_width<=0; cache_height<=0; cache_crop_width<=0; cache_crop_height<=0;
        requested_idx<=0; prefetch_previous<=0; prefetch_blocked<=0; cancel_wait<=0; narration_armed<=0;
        request_is_auto<=0; page_epoch<=0;
        transition_request<=0;transition_ticket<=0;transition_page<=0;transition_automatic<=0;
        commit_guard<=1;
    end else begin
        ack_last<=write_req_ack_sync[1];
        scan_start_pulse<=0; load_start_pulse<=0; load_abort<=0;
        track_start<=0; track_stop<=0;
        commit_guard<=0;
        if(!sd_init_done) begin
            display_source_width<=0; display_source_height<=0;
            display_crop_width<=0; display_crop_height<=0;
            scan_kicked<=0; first_image_committed<=0; auto_cnt<=0;
            img_found_count<=0; img_idx<=0; load_idx<=0;
            pending_buf_idx<=0; write_buf_idx<=0; disp_buf_idx<=0;
            next_req_pending<=0; previous_pending<=0; load_busy<=0; load_timeout_cnt<=0;
            source_done_seen<=0; write_done_seen<=0; load_started_seen<=0;
            write_armed_seen<=0; display_valid<=0; fault_code<=0; load_select_phase<=0;
            cache_valid<=0; requested_idx<=0; prefetch_previous<=0;
            prefetch_blocked<=0; cancel_wait<=0; narration_armed<=0; request_is_auto<=0; page_epoch<=0;
            transition_request<=0;transition_ticket<=0;transition_page<=0;transition_automatic<=0;
            commit_guard<=1;
        end else begin
            // audio_done is a level. Do not reuse the previous track's done
            // level before the WAV player has accepted the new start pulse.
            if(!wav_done && !track_start) narration_armed<=1;
            if(scan_found_valid && img_found_count<MAX_IMAGES)
                img_found_count<=img_found_count+1'b1;
            if(!scan_kicked && bmp_ready) begin
                scan_start_pulse<=1; scan_kicked<=1;
            end
            if(scan_done && img_found_count==0)
                fault_code<=bmp_error!=0 ? bmp_error : 4'd2;

            // Both producers must release the cancelled transaction: draining
            // only the SD sector does not prove the frame writer/FIFO is idle.
            if(cancel_wait && bmp_ready && !sd_sec_read && !image_sector_busy && !load_abort &&
               (ENABLE_WRITE_CANCEL==0 || cancel_ack_sync[1]))
                cancel_wait<=0;
            if(load_busy) begin
                case(load_select_phase)
                    1: load_select_phase<=2;
                    2: begin
                        load_sector<=catalog_read_data[31:0];
                        load_track_number<=catalog_read_data[63:32]; load_select_phase<=3;
                    end
                    3: begin load_start_pulse<=1; load_select_phase<=0; end
                    default: ;
                endcase
                if(!bmp_ready) load_started_seen<=1;
                if(load_started_seen && bmp_ready) source_done_seen<=1;
                if(write_req && write_req_ack_sync[1]) write_armed_seen<=1;
                if(write_armed_seen && write_finish_pulse) write_done_seen<=1;
                if(sd_sec_read_data_valid || ack_last!=write_req_ack_sync[1] || write_finish_pulse)
                    load_timeout_cnt<=0;
                else if(load_timeout_cnt<LOAD_IDLE_CYCLES)
                    load_timeout_cnt<=load_timeout_cnt+1'b1;
                if(bmp_load_failed || load_timeout_cnt>=LOAD_IDLE_CYCLES) begin
                    load_busy<=0; load_abort<=1; cancel_wait<=1; load_timeout_cnt<=0;
                    load_select_phase<=0; source_done_seen<=0; write_done_seen<=0;
                    load_started_seen<=0; write_armed_seen<=0; cache_valid<=0;
                    prefetch_blocked<=1; next_req_pending<=0;
                    transition_request<=0;
                    commit_guard<=1;
                    fault_code<=bmp_load_failed ? bmp_error : 4'd1;
                end else if(source_done_seen && write_done_seen) begin
                    load_busy<=0; load_timeout_cnt<=0;
                    source_done_seen<=0; write_done_seen<=0;
                    load_started_seen<=0; write_armed_seen<=0;
                    cache_valid<=1; cache_idx<=load_idx; cache_buf<=pending_buf_idx;
                    cache_track<=load_track_number;
                    cache_width<=bmp_source_width; cache_height<=bmp_source_height;
                    cache_crop_width<=bmp_crop_width; cache_crop_height<=bmp_crop_height;
                    commit_guard<=1;
                end
            end else load_timeout_cnt<=0;

            // Capture targets relative to the committed page, not the load.
            if(key_next_press || key_prev_press) begin
                requested_idx<=key_prev_press ? previous_from_current : next_from_current;
                next_req_pending<=first_image_committed;
                request_is_auto<=0;
                transition_request<=0;
                commit_guard<=1;
                previous_pending<=key_prev_press; prefetch_previous<=key_prev_press;
                track_stop<=1; fault_code<=0; prefetch_blocked<=0; auto_cnt<=0;
                if(load_busy && load_idx!=(key_prev_press ? previous_from_current : next_from_current)) begin
                    load_busy<=0; load_abort<=1; cancel_wait<=1; cache_valid<=0;
                    load_select_phase<=0; load_start_pulse<=0;
                    source_done_seen<=0; write_done_seen<=0;
                    load_started_seen<=0; write_armed_seen<=0;
                end
            end else if(page_gesture_start) begin
                // Suspend auto on a stable page-button press, before waiting
                // for release/long threshold; do not stop the narration yet.
                auto_cnt<=0;
                commit_guard<=1;
                if(request_is_auto) begin next_req_pending<=0;transition_request<=0;end
            end else if(key_auto_press) begin
                prefetch_previous<=0; auto_cnt<=0;
                commit_guard<=1;
                if(auto_play_en && request_is_auto) begin next_req_pending<=0;transition_request<=0;end
                if(ENABLE_PREFETCH!=0 && !auto_play_en && !next_req_pending &&
                   load_busy && load_idx!=next_from_current) begin
                    // Switching from backward manual navigation to auto should
                    // not finish an irrelevant speculative previous-page load.
                    load_busy<=0; load_abort<=1; cancel_wait<=1; cache_valid<=0;
                    load_select_phase<=0; load_start_pulse<=0;
                    source_done_seen<=0; write_done_seen<=0;
                    load_started_seen<=0; write_armed_seen<=0;
                end
                // Mode toggles preserve the current narration and manual queue.
                if(ENABLE_WAV==0 && !auto_play_en && first_image_committed) begin
                    requested_idx<=next_from_current; next_req_pending<=1; request_is_auto<=1;
                end
            end else begin
                if(ENABLE_WAV!=0) begin
                    if(auto_play_en && display_valid && wav_done && narration_armed && !track_start && !next_req_pending &&
                       img_found_count>1 && audio_fault==0 && fault_code==0) begin
                        // Only automatic navigation waits after audible completion.
                        // Manual requests above bypass this timer and cancel it.
                        if(AUTO_AUDIO_GAP_MS==0 || auto_cnt>=AUTO_GAP_CYCLES-1'b1) begin
                            auto_cnt<=0;requested_idx<=next_from_current;next_req_pending<=1;request_is_auto<=1;
                            commit_guard<=1;
                        end else auto_cnt<=auto_cnt+1'b1;
                    end else auto_cnt<=0;
                end else if(catalog_sorted && auto_play_en && display_valid &&
                            !next_req_pending && img_found_count>1) begin
                    if(auto_tick) begin
                        auto_cnt<=0; requested_idx<=next_from_current; next_req_pending<=1; request_is_auto<=1;
                        commit_guard<=1;
                    end else auto_cnt<=auto_cnt+1'b1;
                end else auto_cnt<=0;

                // Retain the old page as a cache until it is overwritten.
                if(cache_commit_candidate && !commit_guard) begin
                  if(!transition_commit_ok) begin
                    // Prepare the cover only after the complete destination
                    // frame is cached. Keep the visible buffer untouched.
                    if(load_timeout_cnt>=CLK_FREQ_HZ*2) begin
                        transition_request<=0;next_req_pending<=0;prefetch_blocked<=1;
                        fault_code<=4'd1;load_timeout_cnt<=0;
                        commit_guard<=1;
                    end else begin
                      load_timeout_cnt<=load_timeout_cnt+1'b1;
                      if(!transition_request) begin
                        transition_request<=1;transition_ticket<=~transition_ticket;
                        transition_page<={1'b0,cache_idx}+6'd1;
                        transition_automatic<=request_is_auto;
                      end
                    end
                  end else begin
                    transition_request<=0;
                    commit_guard<=1;
                    disp_buf_idx<=cache_buf; img_idx<=cache_idx;
                    display_source_width<=cache_width; display_source_height<=cache_height;
                    display_crop_width<=cache_crop_width; display_crop_height<=cache_crop_height;
                    track_number<=cache_track; track_start<=1; narration_armed<=0;
                    page_epoch<=~page_epoch;
                    cache_valid<=display_valid; cache_idx<=img_idx; cache_buf<=disp_buf_idx;
                    cache_track<=track_number;
                    cache_width<=display_source_width; cache_height<=display_source_height;
                    cache_crop_width<=display_crop_width; cache_crop_height<=display_crop_height;
                    display_valid<=1; first_image_committed<=1; fault_code<=0;
                    next_req_pending<=0; prefetch_blocked<=0; auto_cnt<=0; request_is_auto<=0;
                  end
                end else if(catalog_sorted && bmp_ready && !load_busy && !cancel_wait && !load_abort &&
                            img_found_count!=0 && (first_image_committed || !cache_valid) &&
                            (ENABLE_WRITE_CANCEL==0 || !cancel_ack_sync[1]) &&
                            (ENABLE_PREFETCH==0 || !display_valid || active_buf_sync==disp_buf_idx)) begin
                    if((!first_image_committed && !prefetch_blocked) ||
                       (next_req_pending && (!cache_valid || cache_idx!=requested_idx)) ||
                       (ENABLE_PREFETCH!=0 && !next_req_pending && !prefetch_blocked &&
                        img_found_count>1 && (!cache_valid || cache_idx!=prefetch_idx))) begin
                        load_idx<=!first_image_committed ? 5'd0 :
                                  (next_req_pending ? requested_idx : prefetch_idx);
                        pending_buf_idx<=display_valid ? (disp_buf_idx ^ 2'b01) : 2'd0;
                        write_buf_idx<=display_valid ? (disp_buf_idx ^ 2'b01) : 2'd0;
                        cache_valid<=0; load_busy<=1; load_select_phase<=1; load_timeout_cnt<=0;
                        commit_guard<=1;
                        source_done_seen<=0; write_done_seen<=0;
                        load_started_seen<=0; write_armed_seen<=0;
                        if(!first_image_committed || next_req_pending) track_stop<=1;
                    end
                end
            end
        end
    end
end

bmp_read #(.MAX_IMAGES(MAX_IMAGES),.OUTPUT_WIDTH(OUTPUT_WIDTH),.OUTPUT_HEIGHT(OUTPUT_HEIGHT),
    .FILL_MODE(FILL_MODE),.WRITE_BACKPRESSURE(WRITE_BACKPRESSURE),
    .PIXEL_PERIOD(PIXEL_PERIOD)) bmp_read_m0(
    .clk(clk),.rst(rst),.ready(bmp_ready),
    .scan_start(scan_start_pulse),.scan_start_sector(SCAN_START_SECTOR),
    .scan_max_sector(SCAN_MAX_SECTOR),.scan_target_count(SCAN_TARGET_COUNT),
    .scan_done(scan_done),.scan_found_valid(scan_found_valid),
    .scan_found_sector(scan_found_sector),.scan_found_order(scan_found_order),
    .scan_found_total(scan_found_total),
    .load_start(load_start_pulse),.load_abort(load_abort),.load_sector(load_sector),
    .sd_init_done(sd_init_done),.state_code(state_code),.reader_state(reader_state),
    .bmp_width(bmp_width),.bmp_height(bmp_height),.write_req(write_req),
    .write_req_ack(write_req_ack_sync[1]),.write_ready(write_ready),.sd_sec_read(sd_sec_read),
    .sd_sec_read_addr(sd_sec_read_addr),.sd_sec_read_data(sd_sec_read_data),
    .sd_sec_read_data_valid(sd_sec_read_data_valid),.sd_sec_read_end(sd_sec_read_end),
    .bmp_data_wr_en(bmp_data_wr_en),.bmp_data(bmp_data),
    .load_failed(bmp_load_failed),.error_code(bmp_error),.catalog_full(catalog_full),
    .fs_valid(fs_valid),.fs_data_start(fs_data_start),.fs_fat_start(fs_fat_start),
    .fs_root_cluster(fs_root_cluster),.fs_max_cluster(fs_max_cluster),
    .fs_volume_end(fs_volume_end),.fs_fat_sectors(fs_fat_sectors),
    .fs_cluster_shift(fs_cluster_shift),.fs_spc(fs_spc),
    .source_width(bmp_source_width),.source_height(bmp_source_height),
    .source_crop_width(bmp_crop_width),.source_crop_height(bmp_crop_height)
);
generate if(ENABLE_WAV!=0) begin: g_wav
    album_sector_arbiter #(.ENABLE_IMAGE_CANCEL(1)) u_sector_arbiter(
        .clk(clk),.rst(rst),.image_req(sd_sec_read),.image_addr(sd_sec_read_addr),
        .image_cancel(load_abort),
        .audio_req(audio_req),.audio_addr(audio_addr),.card_req(card_req),.card_addr(card_addr),
        .card_data(card_data),.card_valid(card_valid),.card_end(card_end),
        .image_data(sd_sec_read_data),.image_valid(sd_sec_read_data_valid),.image_end(sd_sec_read_end),
        .audio_data(audio_data),.audio_valid(audio_valid),.audio_end(audio_end),.audio_busy(audio_sector_busy),
        .image_busy(image_sector_busy)
    );
    album_wav_player #(.CLOCK_HZ(CLK_FREQ_HZ)) u_wav_player(
        .clk(clk),.rst(rst),.start(track_start),.stop(track_stop),.track_number(track_number),
        .fs_valid(fs_valid),.fs_data_start(fs_data_start),.fs_fat_start(fs_fat_start),
        .fs_root_cluster(fs_root_cluster),.fs_max_cluster(fs_max_cluster),
        .fs_volume_end(fs_volume_end),.fs_fat_sectors(fs_fat_sectors),
        .fs_cluster_shift(fs_cluster_shift),.fs_spc(fs_spc),
        .sector_req(audio_req),.sector_addr(audio_addr),.sector_data(audio_data),
        .sector_valid(audio_valid),.sector_end(audio_end),.sector_busy(audio_sector_busy),
        .sample_send(pcm_send),.sample_data(pcm_data),.sample_last(pcm_last),
        .sample_ready(pcm_ready),.last_played_pulse(pcm_last_played),
        .mute(pcm_mute),.audio_done(wav_done),.fault(audio_fault),.played_cs(wav_elapsed_cs)
    );
end else begin: g_legacy_image_test
    assign card_req=sd_sec_read; assign card_addr=sd_sec_read_addr;
    assign sd_sec_read_data=card_data; assign sd_sec_read_data_valid=card_valid;
    assign sd_sec_read_end=card_end;
    assign pcm_send=0; assign pcm_data=0; assign pcm_last=0; assign pcm_mute=1;
    assign wav_done=0; assign audio_fault=0; assign wav_elapsed_cs=0;
    assign image_sector_busy=0;
end endgenerate
sd_card_top sd_card_top_m0(
    .clk(clk),.rst(rst),.SD_nCS(SD_nCS),.SD_DCLK(SD_DCLK),.SD_MOSI(SD_MOSI),
    .SD_MISO(SD_MISO),.sd_init_done(sd_init_done),.sd_sec_read(card_req),
    .sd_sec_read_addr(card_addr),.sd_sec_read_data(card_data),
    .sd_sec_read_data_valid(card_valid),.sd_sec_read_end(card_end),
    .sd_sec_write(1'b0),.sd_sec_write_addr(32'd0),.sd_sec_write_data(8'd0),
    .sd_sec_write_data_req(),.sd_sec_write_end()
);
endmodule

// Debounced short-on-release / one-shot long-on-hold. A long gesture never
// emits a short event on release. Durations start after stable press detection.
module album_short_long_button #(
    parameter integer CLK_FREQ_HZ=100000000,
    parameter integer DEBOUNCE_MS=20,
    parameter integer LONG_MS=800
)(
    input clk,rst,button_in,
    output reg short_pulse,long_pulse,press_pulse,
    output button_down
);
localparam integer DB_CYCLES=(CLK_FREQ_HZ/1000)*DEBOUNCE_MS;
localparam integer HOLD_CYCLES=(CLK_FREQ_HZ/1000)*LONG_MS;
(* ASYNC_REG="TRUE" *) reg sync0,sync1;
reg stable,long_sent;
reg [31:0] debounce_count,hold_count;
assign button_down=!stable;
always @(posedge clk or posedge rst) begin
    if(rst) begin
        sync0<=1; sync1<=1; stable<=1; long_sent<=0;
        debounce_count<=0; hold_count<=0;
        short_pulse<=0; long_pulse<=0; press_pulse<=0;
    end else begin
        sync0<=button_in; sync1<=sync0;
        short_pulse<=0; long_pulse<=0; press_pulse<=0;
        if(sync1==stable) debounce_count<=0;
        else if(debounce_count>=DB_CYCLES-1) begin
            stable<=sync1; debounce_count<=0;
            if(!sync1) begin hold_count<=0; long_sent<=0; press_pulse<=1; end
            else if(!long_sent) short_pulse<=1;
        end else debounce_count<=debounce_count+1'b1;
        // Stop counting during a tentative release; release debounce must not
        // turn a just-short-enough press into a long gesture.
        if(!stable && !sync1 && !long_sent) begin
            if(hold_count>=HOLD_CYCLES-1) begin long_sent<=1; long_pulse<=1; end
            else hold_count<=hold_count+1'b1;
        end
    end
end
endmodule

module key_press_debounce #(
    parameter integer CLK_FREQ_HZ = 100_000_000,
    parameter integer ENABLE_WAV = 0, // top enables WAV; legacy image tests keep timer mode
    parameter integer DEBOUNCE_MS = 20
)(
    input  wire clk,
    input  wire rst,
    input  wire button_in,    // 默认：松开=1，按下=0
    output reg  press_pulse
);

localparam integer DEBOUNCE_CYCLES = (CLK_FREQ_HZ / 1000) * DEBOUNCE_MS;

reg button_sync0;
reg button_sync1;
reg button_stable;
reg [31:0] cnt;

always @(posedge clk or posedge rst) begin
    if (rst) begin
        button_sync0  <= 1'b1;
        button_sync1  <= 1'b1;
        button_stable <= 1'b1;
        cnt           <= 32'd0;
        press_pulse   <= 1'b0;
    end else begin
        button_sync0 <= button_in;
        button_sync1 <= button_sync0;
        press_pulse  <= 1'b0;

        if (button_sync1 == button_stable) begin
            cnt <= 32'd0;
        end else begin
            if (cnt >= DEBOUNCE_CYCLES - 1) begin
                if (button_stable && !button_sync1)
                    press_pulse <= 1'b1;
                button_stable <= button_sync1;
                cnt           <= 32'd0;
            end else begin
                cnt <= cnt + 32'd1;
            end
        end
    end
end

endmodule


// Audio helper definitions stay with this already-registered source.
// TD GUI saves of an older file list must not turn them into black boxes.

// Whole-sector arbitration. Client requests remain asserted until their end
// pulse; no SPI transaction is interrupted when a page/track is cancelled.
module album_sector_arbiter #(parameter integer ENABLE_IMAGE_CANCEL=0)(
    input clk,rst,
    input image_req, input [31:0] image_addr,
    input image_cancel,
    input audio_req, input [31:0] audio_addr,
    output reg card_req, output reg [31:0] card_addr,
    input [7:0] card_data, input card_valid,card_end,
    output reg [7:0] image_data,audio_data,
    output reg image_valid,audio_valid,audio_end,
    output image_end,audio_busy,image_busy
);
reg [1:0] state;
reg owner;
reg cancelled_image_end;
reg image_real_end;
wire cancel_image=ENABLE_IMAGE_CANCEL!=0 && image_cancel;
// Separate clocked client buses cut the SPI->all clients combinational fanout.
// Data has no reset; valid/end are reset and delayed by the same single cycle.
always @(posedge clk) begin
    if(state==1 && !owner && card_valid) image_data<=card_data;
    if(state==1 && owner && card_valid) audio_data<=card_data;
end
always @(posedge clk or posedge rst) begin
    if(rst) begin image_valid<=0;audio_valid<=0;image_real_end<=0;audio_end<=0;end
    else begin
        image_valid<=state==1 && !owner && card_valid;
        audio_valid<=state==1 && owner && card_valid;
        image_real_end<=state==1 && !owner && card_end;
        audio_end<=state==1 && owner && card_end;
    end
end
assign image_end=image_real_end || cancelled_image_end;
assign audio_busy=state==1 && owner;
assign image_busy=state!=0 && !owner; // includes transaction drain/release states
// An image request may be waiting behind audio and not yet granted. In that
// case the parser still needs a drain acknowledgment, but no card transaction
// exists to produce one. Owned image sectors always finish on the real card.
always @(posedge clk or posedge rst) begin
    if(rst) cancelled_image_end<=0;
    else cancelled_image_end<=cancel_image && !(state==1 && !owner);
end
always @(posedge clk or posedge rst) begin
    if(rst) begin state<=0; owner<=0; card_req<=0; card_addr<=0; end
    else case(state)
        0: begin
            card_req<=0;
            // Audio requests only when its buffer has room for a full sector.
            if(audio_req) begin owner<=1; card_addr<=audio_addr; card_req<=1; state<=1; end
            else if(image_req && !cancel_image) begin owner<=0; card_addr<=image_addr; card_req<=1; state<=1; end
        end
        1: if(card_end) begin card_req<=0; state<=2; end
        2: state<=3; // allow the owner and SPI controller to release requests
        3: if(!card_end) state<=0;
        default: begin state<=0; card_req<=0; end
    endcase
end
endmodule

// One sample mailbox, not an unsynchronized PCM bus. The source holds the
// complete word until its request is acknowledged. 48 kHz is much slower
// than the 100/25 MHz clocks, leaving ample round-trip settling time.
module album_pcm_hdmi_bridge #(
    parameter integer PIXEL_HZ=25000000
)(
    input sd_clk,sd_rst,video_clk,video_rst,
    input sample_send, input [15:0] sample_data, input sample_last,
    input mute,
    output sample_ready, output last_played_pulse,
    output reg audio_valid, output reg [23:0] audio_left,audio_right,
    output reg acr_valid, output [19:0] acr_cts,acr_n
);
reg request_toggle,ack_toggle,done_toggle;
reg [16:0] held_sample;
(* ASYNC_REG="TRUE" *) reg [1:0] ack_sync,done_sync,request_sync,mute_sync;
reg done_seen,request_seen;
reg [16:0] acr_count;
assign sample_ready=request_toggle==ack_sync[1];
assign last_played_pulse=done_sync[1]^done_seen;
assign acr_n=20'd6144;
assign acr_cts=PIXEL_HZ/1000; // N=6144, Fs=48k: CTS=Fpixel/1000
always @(posedge sd_clk or posedge sd_rst) begin
    if(sd_rst) begin
        request_toggle<=0; held_sample<=0; ack_sync<=0; done_sync<=0; done_seen<=0;
    end else begin
        ack_sync<={ack_sync[0],ack_toggle};
        done_sync<={done_sync[0],done_toggle}; done_seen<=done_sync[1];
        if(sample_send && sample_ready) begin
            held_sample<={sample_last,sample_data}; request_toggle<=~request_toggle;
        end
    end
end
always @(posedge video_clk or posedge video_rst) begin
    if(video_rst) begin
        request_sync<=0; mute_sync<=3; request_seen<=0; ack_toggle<=0;
        done_toggle<=0; audio_valid<=0; audio_left<=0; audio_right<=0;
        acr_count<=0; acr_valid<=0;
    end else begin
        request_sync<={request_sync[0],request_toggle};
        mute_sync<={mute_sync[0],mute};
        audio_valid<=0; acr_valid<=0;
        if(acr_count==PIXEL_HZ/1000-1) begin acr_count<=0; acr_valid<=1; end
        else acr_count<=acr_count+1'b1;
        if(request_sync[1]!=request_seen) begin
            request_seen<=request_sync[1]; ack_toggle<=request_sync[1];
            audio_valid<=1;
            audio_left<=mute_sync[1] ? 24'd0 : {held_sample[15:0],8'd0};
            audio_right<=mute_sync[1] ? 24'd0 : {held_sample[15:0],8'd0};
            if(held_sample[16] && !mute_sync[1]) done_toggle<=~done_toggle;
        end
    end
end
endmodule

// Read-only FAT32 WAV stream for the converter's canonical 44-byte PCM WAV.
// Searches root entries by decimal digits in the short basename; supports
// fragmented root and file chains. Accepts 48kHz/16bit/mono PCM only.
module album_wav_player #(
    parameter integer CLOCK_HZ=100000000,
    parameter integer FIFO_BITS=11
)(
    input clk,rst,start,stop, input [31:0] track_number,
    input fs_valid, input [31:0] fs_data_start,fs_fat_start,fs_root_cluster,
    input [31:0] fs_max_cluster,fs_volume_end,fs_fat_sectors,
    input [3:0] fs_cluster_shift, input [7:0] fs_spc,
    output reg sector_req, output reg [31:0] sector_addr,
    input [7:0] sector_data, input sector_valid,sector_end,sector_busy,
    output sample_send, output [15:0] sample_data, output sample_last,
    input sample_ready,last_played_pulse,
    output mute, output reg audio_done, output reg [3:0] fault,
    output reg [15:0] played_cs
);
localparam [4:0] IDLE=0,CANCEL=1,ADDR_MUL=2,ADDR_SUM=3,ADDR_CHECK=4,
    ADDR_USE=5,ISSUE=6,WAIT_SECTOR=7,DIR_EVAL=8,FAT_ADDR=9,
    FAT_USE=10,FAT_EVAL=11,HEADER_SUM=12,HEADER_CHECK=13,
    HEADER_APPLY=14,ADVANCE_FILE=15,SPACE=16,DATA_EVAL=17,DRAIN=18,FAILED=19,FAT_APPLY=20;
localparam [1:0] KIND_DIR=0,KIND_HEADER=1,KIND_DATA=2,KIND_FAT=3;
localparam integer FIFO_DEPTH=1<<FIFO_BITS;
reg [4:0] state,return_state;
reg [1:0] read_kind,cluster_kind;
reg restart_pending;
reg [2:0] cancel_wait;
reg [31:0] requested_number;
reg [31:0] cluster_target,dir_cluster,file_cluster,dir_lba,file_lba;
reg [7:0] dir_sector,file_sector;
reg [32:0] cluster_offset,cluster_sum;
reg address_bad;
reg [9:0] byte_index;
reg [12:0] directory_reads;
reg dir_end_seen,candidate_pending;
reg [7:0] entry_first,entry_attr,ext0,ext1,ext2;
reg [31:0] entry_cluster,entry_size;
reg [26:0] entry_number;
reg entry_number_valid;
reg [63:0] entry_name;
reg name_parse_busy,name_parse_phase;
reg [2:0] name_parse_index;
reg [7:0] name_digit;
reg [26:0] name_times10;
reg [31:0] wav_file_size;
reg [31:0] chain_cluster,fat_entry,fat_address;
reg [8:0] fat_offset;
reg fat_for_file,fat_address_bad,next_cluster_valid_q;
wire [31:0] next_cluster={4'd0,fat_entry[27:0]};
wire next_cluster_valid=next_cluster>=2 && next_cluster<=fs_max_cluster &&
    next_cluster<32'h0ffffff0 && next_cluster!=chain_cluster;
wire entry_match=entry_first!=0 && entry_first!=8'he5 && entry_first!=8'h2e &&
    (entry_attr & 8'h1e)==0 && (ext0 & 8'hdf)=="W" &&
    (ext1 & 8'hdf)=="A" && (ext2 & 8'hdf)=="V" &&
    entry_number_valid && {5'd0,entry_number}==requested_number &&
    entry_cluster>=2 && entry_cluster<=fs_max_cluster;

reg [31:0] riff_magic,wave_magic,fmt_magic,fmt_size,data_magic,data_size;
reg [15:0] format_tag,channels,bits_per_sample,block_align;
reg [31:0] sample_rate,byte_rate,file_position,data_received;
reg [32:0] expected_file_size;
reg header_ok,format_good,reader_done;
reg [7:0] low_byte;
reg half_sample;
reg [31:0] total_samples,consumed_samples;
reg playing,output_enabled;
reg [26:0] sample_phase;
wire sample_tick=sample_phase>=CLOCK_HZ-48000;
localparam [26:0] PHASE_ADD=27'd48000;
localparam [26:0] PHASE_SUB=CLOCK_HZ-48000;

reg [15:0] pcm_memory [0:FIFO_DEPTH-1]; /* fehdl force_ram=1, ram_style="bram" */
reg [FIFO_BITS-1:0] wr_ptr,rd_ptr;
reg [FIFO_BITS:0] used_samples;
reg [15:0] head_sample;
wire payload_byte=state==WAIT_SECTOR && sector_valid &&
    (read_kind==KIND_HEADER || read_kind==KIND_DATA) &&
    file_position>=44 && data_received<data_size;
wire payload_pair=payload_byte && half_sample;
wire fifo_write=payload_pair && used_samples<FIFO_DEPTH && !start && !stop;
wire fifo_pop=sample_tick && sample_ready && playing && used_samples!=0 && !start && !stop;
assign sample_send=sample_tick && sample_ready;
assign sample_data=fifo_pop ? head_sample : 16'd0;
assign sample_last=fifo_pop && consumed_samples+32'd1==total_samples;
assign mute=!output_enabled || start || stop;
reg [8:0] centisecond_samples;
// Count consumed valid PCM samples, not SD-read time or speculative loading.
always @(posedge clk or posedge rst) begin
    if(rst) begin played_cs<=0; centisecond_samples<=0; end
    else if(start || stop) begin played_cs<=0; centisecond_samples<=0; end
    else if(fifo_pop) begin
        if(centisecond_samples==479) begin
            centisecond_samples<=0;
            if(played_cs!=16'hffff) played_cs<=played_cs+1'b1;
        end else centisecond_samples<=centisecond_samples+1'b1;
    end
end
always @(posedge clk) begin
    if(fifo_write) pcm_memory[wr_ptr]<={sector_data,low_byte};
    head_sample<=pcm_memory[rd_ptr];
end
always @(posedge clk or posedge rst) begin
    if(rst) sample_phase<=0;
    else if(sample_tick) sample_phase<=sample_phase-PHASE_SUB;
    else sample_phase<=sample_phase+PHASE_ADD;
end

always @(posedge clk or posedge rst) begin
    if(rst) begin
        state<=IDLE; return_state<=IDLE; read_kind<=0; cluster_kind<=0;
        restart_pending<=0; cancel_wait<=0; requested_number<=0;
        sector_req<=0; sector_addr<=0; cluster_target<=0;
        dir_cluster<=0; file_cluster<=0; dir_lba<=0; file_lba<=0;
        dir_sector<=0; file_sector<=0; cluster_offset<=0; cluster_sum<=0;
        address_bad<=0; byte_index<=0; directory_reads<=0;
        dir_end_seen<=0; candidate_pending<=0;
        entry_first<=0; entry_attr<=0; ext0<=0; ext1<=0; ext2<=0;
        entry_cluster<=0; entry_size<=0; entry_number<=0; entry_number_valid<=0;
        entry_name<=0; name_parse_busy<=0; name_parse_phase<=0;
        name_parse_index<=0; name_digit<=0; name_times10<=0;
        wav_file_size<=0; chain_cluster<=0; fat_entry<=0; fat_address<=0;
        fat_offset<=0; fat_for_file<=0; fat_address_bad<=0;next_cluster_valid_q<=0;
        riff_magic<=0; wave_magic<=0; fmt_magic<=0; fmt_size<=0;
        data_magic<=0; data_size<=0; format_tag<=0; channels<=0;
        bits_per_sample<=0; block_align<=0; sample_rate<=0; byte_rate<=0;
        file_position<=0; data_received<=0; expected_file_size<=0;
        header_ok<=0; format_good<=0; reader_done<=0;
        low_byte<=0; half_sample<=0; total_samples<=0; consumed_samples<=0;
        playing<=0; output_enabled<=0; wr_ptr<=0; rd_ptr<=0; used_samples<=0;
        audio_done<=0; fault<=0;
    end else if(start || stop) begin
        // Drain an already granted sector; the arbiter retains SPI ownership.
        state<=CANCEL; sector_req<=0; cancel_wait<=0;
        restart_pending<=start; if(start) requested_number<=track_number;
        format_good<=0; reader_done<=0; header_ok<=0;
        file_position<=0; data_received<=0; data_size<=0;
        playing<=0; output_enabled<=0; total_samples<=0; consumed_samples<=0;
        wr_ptr<=0; rd_ptr<=0; used_samples<=0; half_sample<=0;
        audio_done<=0; fault<=0;
        name_parse_busy<=0; name_parse_phase<=0;
    end else begin
        // Capture the eight basename bytes, then parse one character in two
        // clocks. Finishes16 clocks after byte7, before the byte31 match even
        // for a one-byte-per-clock sector model. Avoid mux -> DSP*10 -> add.
        if(name_parse_busy) begin
            if(!name_parse_phase) begin
                name_times10<=(entry_number<<3)+(entry_number<<1);
                name_digit<=entry_name[{name_parse_index,3'b000} +:8];
                name_parse_phase<=1;
            end else begin
                if(name_digit>=8'h30 && name_digit<=8'h39) begin
                    entry_number<=name_times10+{23'd0,name_digit[3:0]};
                    entry_number_valid<=1;
                end
                name_parse_phase<=0;
                if(name_parse_index==7) name_parse_busy<=0;
                else name_parse_index<=name_parse_index+1'b1;
            end
        end
        if(fifo_write) wr_ptr<=wr_ptr+1'b1;
        if(fifo_pop) begin
            rd_ptr<=rd_ptr+1'b1; consumed_samples<=consumed_samples+1'b1;
            if(sample_last) playing<=0;
        end
        case({fifo_write,fifo_pop})
            2'b10:used_samples<=used_samples+1'b1;
            2'b01:used_samples<=used_samples-1'b1;
            default: ;
        endcase
        if(format_good && !playing && !output_enabled &&
           (used_samples>=512 || (reader_done && used_samples!=0))) begin
            playing<=1; output_enabled<=1;
        end
        // Playback completion is returned by the HDMI clock domain, after
        // the last valid sample has been handed to its audio interface.
        if(last_played_pulse && output_enabled && total_samples!=0 &&
           consumed_samples==total_samples) audio_done<=1;

        if(state==WAIT_SECTOR && sector_valid) begin
            byte_index<=byte_index+1'b1;
            if(read_kind==KIND_DIR && !candidate_pending && !dir_end_seen) begin
                if(byte_index[4:0]==0) begin entry_number<=0; entry_number_valid<=0; end
                if(byte_index[4:0]<8) begin
                    entry_name[{byte_index[2:0],3'b000} +:8]<=sector_data;
                    if(byte_index[4:0]==7) begin
                        name_parse_index<=0; name_parse_phase<=0; name_parse_busy<=1;
                    end
                end
                case(byte_index[4:0])
                    0:begin entry_first<=sector_data; if(sector_data==0) dir_end_seen<=1; end
                    8:ext0<=sector_data; 9:ext1<=sector_data; 10:ext2<=sector_data;
                    11:entry_attr<=sector_data;
                    20:entry_cluster[23:16]<=sector_data;
                    21:entry_cluster[31:24]<={4'd0,sector_data[3:0]};
                    26:entry_cluster[7:0]<=sector_data; 27:entry_cluster[15:8]<=sector_data;
                    28:entry_size[7:0]<=sector_data; 29:entry_size[15:8]<=sector_data;
                    30:entry_size[23:16]<=sector_data;
                    31:begin
                        entry_size[31:24]<=sector_data;
                        if(entry_match && {sector_data,entry_size[23:0]}>=44)
                            candidate_pending<=1;
                    end
                    default: ;
                endcase
            end
            if(read_kind==KIND_FAT) begin
                if(byte_index=={1'b0,fat_offset}) fat_entry[7:0]<=sector_data;
                if(byte_index=={1'b0,fat_offset}+10'd1) fat_entry[15:8]<=sector_data;
                if(byte_index=={1'b0,fat_offset}+10'd2) fat_entry[23:16]<=sector_data;
                if(byte_index=={1'b0,fat_offset}+10'd3) fat_entry[31:24]<=sector_data;
            end
            if(read_kind==KIND_HEADER || read_kind==KIND_DATA) begin
                file_position<=file_position+1'b1;
                if(payload_byte) begin
                    data_received<=data_received+1'b1;
                    if(!half_sample) low_byte<=sector_data;
                    half_sample<=~half_sample;
                end
            end
            if(read_kind==KIND_HEADER) begin
                case(byte_index)
                    0,1,2,3:riff_magic<={sector_data,riff_magic[31:8]};
                    8,9,10,11:wave_magic<={sector_data,wave_magic[31:8]};
                    12,13,14,15:fmt_magic<={sector_data,fmt_magic[31:8]};
                    16,17,18,19:fmt_size<={sector_data,fmt_size[31:8]};
                    20:format_tag[7:0]<=sector_data;21:format_tag[15:8]<=sector_data;
                    22:channels[7:0]<=sector_data;23:channels[15:8]<=sector_data;
                    24,25,26,27:sample_rate<={sector_data,sample_rate[31:8]};
                    28,29,30,31:byte_rate<={sector_data,byte_rate[31:8]};
                    32:block_align[7:0]<=sector_data;33:block_align[15:8]<=sector_data;
                    34:bits_per_sample[7:0]<=sector_data;35:bits_per_sample[15:8]<=sector_data;
                    36,37,38,39:data_magic<={sector_data,data_magic[31:8]};
                    40,41,42,43:data_size<={sector_data,data_size[31:8]};
                    default: ;
                endcase
            end
        end

        case(state)
            IDLE: begin
                sector_req<=0;
                if(restart_pending && fs_valid) begin
                    restart_pending<=0; dir_cluster<=fs_root_cluster;
                    cluster_target<=fs_root_cluster; cluster_kind<=KIND_DIR;
                    directory_reads<=0; state<=ADDR_MUL;
                end
            end
            CANCEL: begin
                sector_req<=0;
                if(cancel_wait<4) cancel_wait<=cancel_wait+1'b1;
                else if(!sector_busy) state<=IDLE;
            end
            ADDR_MUL: begin
                cluster_offset<=({1'b0,cluster_target}-33'd2)<<fs_cluster_shift;
                state<=ADDR_SUM;
            end
            ADDR_SUM: begin cluster_sum<={1'b0,fs_data_start}+cluster_offset; state<=ADDR_CHECK; end
            ADDR_CHECK: begin
                address_bad<=cluster_sum[32] || cluster_sum>={1'b0,fs_volume_end} ||
                    cluster_target<2 || cluster_target>fs_max_cluster;
                state<=ADDR_USE;
            end
            ADDR_USE: begin
                if(address_bad) begin fault<=4; state<=FAILED; end
                else begin
                    sector_addr<=cluster_sum[31:0]; read_kind<=cluster_kind;
                    if(cluster_kind==KIND_DIR) begin
                        dir_lba<=cluster_sum[31:0]; dir_sector<=0; return_state<=DIR_EVAL;
                    end else begin
                        file_lba<=cluster_sum[31:0]; file_sector<=0;
                        return_state<=cluster_kind==KIND_HEADER ? HEADER_SUM : DATA_EVAL;
                    end
                    state<=cluster_kind==KIND_DATA ? SPACE : ISSUE;
                end
            end
            ISSUE: begin
                byte_index<=0;
                if(read_kind==KIND_DIR) begin
                    candidate_pending<=0; dir_end_seen<=0; directory_reads<=directory_reads+1'b1;
                end
                if(read_kind==KIND_DIR && directory_reads>=4096) begin fault<=3; state<=FAILED; end
                else begin sector_req<=1; state<=WAIT_SECTOR; end
            end
            WAIT_SECTOR: if(sector_end) begin sector_req<=0; state<=return_state; end
            DIR_EVAL: begin
                if(candidate_pending) begin
                    file_cluster<=entry_cluster; wav_file_size<=entry_size;
                    cluster_target<=entry_cluster; cluster_kind<=KIND_HEADER; state<=ADDR_MUL;
                end else if(dir_end_seen) begin fault<=1; state<=FAILED; end
                else if({1'b0,dir_sector}+9'd1<{1'b0,fs_spc}) begin
                    dir_sector<=dir_sector+1'b1; dir_lba<=dir_lba+1'b1;
                    sector_addr<=dir_lba+1'b1; read_kind<=KIND_DIR; state<=ISSUE;
                end else begin chain_cluster<=dir_cluster; fat_for_file<=0; state<=FAT_ADDR; end
            end
            FAT_ADDR: begin
                fat_address<=fs_fat_start+(chain_cluster>>7);
                fat_address_bad<=(chain_cluster>>7)>=fs_fat_sectors;
                fat_offset<={chain_cluster[6:0],2'b00}; state<=FAT_USE;
            end
            FAT_USE: begin
                if(fat_address_bad) begin fault<=4; state<=FAILED; end
                else begin sector_addr<=fat_address; read_kind<=KIND_FAT; return_state<=FAT_EVAL; state<=ISSUE; end
            end
            FAT_EVAL: begin
                // The complete FAT entry is stable throughout these two phases.
                // Register validation before it enables32-bit target/state writes.
                next_cluster_valid_q<=next_cluster_valid;state<=FAT_APPLY;
            end
            FAT_APPLY: begin
                if(!next_cluster_valid_q) begin fault<=fat_for_file ? 4'd5 : 4'd1; state<=FAILED; end
                else begin
                    cluster_target<=next_cluster;
                    if(fat_for_file) begin file_cluster<=next_cluster; cluster_kind<=KIND_DATA; end
                    else begin dir_cluster<=next_cluster; cluster_kind<=KIND_DIR; end
                    state<=ADDR_MUL;
                end
            end
            HEADER_SUM: begin expected_file_size<={1'b0,data_size}+33'd44; state<=HEADER_CHECK; end
            HEADER_CHECK: begin
                header_ok<=riff_magic==32'h46464952 && wave_magic==32'h45564157 &&
                    fmt_magic==32'h20746d66 && fmt_size==16 && format_tag==1 && channels==1 &&
                    sample_rate==48000 && byte_rate==96000 && block_align==2 && bits_per_sample==16 &&
                    data_magic==32'h61746164 && data_size!=0 && !data_size[0] &&
                    !expected_file_size[32] && expected_file_size=={1'b0,wav_file_size};
                state<=HEADER_APPLY;
            end
            HEADER_APPLY: begin
                if(!header_ok) begin fault<=2; state<=FAILED; end
                else begin format_good<=1; total_samples<=data_size>>1; state<=DATA_EVAL; end
            end
            DATA_EVAL: begin
                if(data_received>=data_size) begin reader_done<=1; state<=DRAIN; end
                else state<=ADVANCE_FILE;
            end
            ADVANCE_FILE: begin
                if({1'b0,file_sector}+9'd1<{1'b0,fs_spc}) begin
                    file_sector<=file_sector+1'b1; file_lba<=file_lba+1'b1;
                    sector_addr<=file_lba+1'b1; read_kind<=KIND_DATA;
                    return_state<=DATA_EVAL; state<=SPACE;
                end else begin chain_cluster<=file_cluster; fat_for_file<=1; state<=FAT_ADDR; end
            end
            SPACE: if(used_samples<=FIFO_DEPTH-256) state<=ISSUE;
            DRAIN: sector_req<=0;
            FAILED: begin sector_req<=0; playing<=0; output_enabled<=0; end
            default: state<=IDLE;
        endcase
        if((payload_pair && used_samples>=FIFO_DEPTH) ||
           (sample_tick && playing && (used_samples==0 || !sample_ready))) begin
            fault<=6; state<=FAILED; sector_req<=0; playing<=0; output_enabled<=0;
        end
    end
end
endmodule
