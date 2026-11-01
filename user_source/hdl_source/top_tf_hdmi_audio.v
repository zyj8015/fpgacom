
module top(
    input                       clk,
    input                       rst_n,          // Physical KEY4 / C1: active-low reset
    input                       key1,           // Physical KEY1 / A2: auto toggle
    input                       key2,           // B2: short next, 800 ms hold previous
    input                       key3,           // B1: short adjust, 800 ms hold select parameter

	output [7:0]                seg_sel,
	output [7:0]                seg_data,

    // HDMI TMDS
    output                      HDMI_CLK_P,
    output                      HDMI_D2_P,
    output                      HDMI_D1_P,
    output                      HDMI_D0_P,

    // HDMI DDC
    output                      HDMI_DDC_SCL,
    inout                       HDMI_DDC_SDA,

    // TF card SPI
    output                      sd_ncs,
    output                      sd_dclk,
    output                      sd_mosi,
    input                       sd_miso
);

// Dwell after each completed image load. Later audio playback must supply
// duration/track-end events; this timer alone is not audio synchronization.
parameter integer ALBUM_DWELL_MS = 3000;
parameter MEM_DATA_BITS = 32;
parameter ADDR_BITS     = 21;
parameter BUSRT_BITS    = 10;
parameter FRAME_PIXELS  = 24'd921600;   // 1280*720; two buffers = 7,372,800 bytes
parameter BUF0_ADDR     = 24'd0;
parameter BUF1_ADDR     = FRAME_PIXELS;

wire Sdr_init_done;
wire Sdr_init_ref_vld;
wire Sdr_busy;

wire sd_card_clk;
wire ext_mem_clk;
wire ext_mem_clk_sft;
wire video_clk;
wire hdmi_5x_clk;

wire hs;
wire vs;
wire de;

wire [23:0] vout_data_raw;
wire [23:0] vout_data;
wire [23:0] osd_rgb;
wire osd_hs,osd_vs,osd_de;
wire osd_frame_valid;
wire [1:0] osd_frame_buffer;
wire [15:0] shown_source_width,shown_source_height;
wire [15:0] shown_crop_width,shown_crop_height;
wire narration_complete;
wire [15:0] narration_elapsed_cs;
wire narration_epoch;
wire [5:0] osd_frame_page;
wire osd_sidebar_visible;
wire [8:0] transition_command_sd,transition_command_video;
wire [1:0] transition_reply_video,transition_reply_sd;
wire [23:0] scene_rgb,video_narration;
wire scene_hs,scene_vs,scene_de;
wire [23:0] waveform_rgb;
wire waveform_hs,waveform_vs,waveform_de;
wire        display_valid;

wire [3:0]  state_code;
wire [4:0]  reader_state;

wire        video_read_req;
wire        video_read_req_ack;
wire        video_read_en;
wire [31:0] video_read_data;

wire        sd_card_write_en;
wire        sd_card_write_ready;
wire        sd_card_write_cancel,sd_card_write_cancel_ack;
wire [31:0] sd_card_write_data;
wire        sd_card_write_req;
wire        sd_card_write_req_ack;
wire        frame_write_finish;
reg         frame_write_toggle_mem;

wire [1:0]  write_buf_idx;
wire [1:0]  disp_buf_idx;

wire App_rd_en;
wire [ADDR_BITS-1:0] App_rd_addr;
wire Sdr_rd_en;
wire [MEM_DATA_BITS-1:0] Sdr_rd_dout;
wire App_wr_en;
wire [ADDR_BITS-1:0] App_wr_addr;
wire [MEM_DATA_BITS-1:0] App_wr_din;
wire [3:0] App_wr_dm;

wire hs_0;
wire vs_0;
wire de_0;

// HDMI 1.4b 相关
wire        axis_s_user;
wire        axis_s_valid;
wire        axis_s_last;
wire [23:0] axis_s_data;
wire        axis_s_ready;

wire        edid_trig;
wire        edid_valid;
wire [7:0]  edid_data;

// WAV samples cross through a held-data mailbox; ACR remains in video_clk.
wire        audio_valid;
wire [23:0] audio_left_data;
wire [23:0] audio_right_data;
wire        acr_valid;
wire [19:0] acr_cts;
wire [19:0] acr_n;
wire        pcm_send,pcm_last,pcm_mute,pcm_ready,pcm_last_played;
wire [15:0] pcm_data;
wire [3:0] wav_fault;
wire        auto_status;
wire [5:0]  image_count;
wire [4:0]  image_index;
wire [3:0]  album_fault;
wire [6:0] page_segments, page_tens_segments;
reg [7:0] display_digit_right, display_digit_left;
wire [5:0] shown_page = display_valid ? ({1'b0,image_index}+6'd1) : 6'd0;
reg [5:0] display_page = 6'd0;
reg display_auto_mode = 1'b0;
wire album_auto_key = key1;
// Active-low dp/gfedcba: lowercase o uses c,d,e,g; n uses c,e,g.
wire [7:0] album_digit_right = display_auto_mode ? 8'b10101011 : {1'b1,page_segments};
wire [7:0] album_digit_left = display_auto_mode ? 8'b10100011 : {1'b1,page_tens_segments};

function [3:0] decimal_tens;
    input [5:0] value;
    begin
        if(value>=30) decimal_tens=3;
        else if(value>=20) decimal_tens=2;
        else if(value>=10) decimal_tens=1;
        else decimal_tens=0;
    end
endfunction
function [3:0] decimal_ones;
    input [5:0] value;
    begin
        if(value>=30) decimal_ones=value-6'd30;
        else if(value>=20) decimal_ones=value-6'd20;
        else if(value>=10) decimal_ones=value-6'd10;
        else decimal_ones=value[3:0];
    end
endfunction

wire [9:0]  tmds_ch0_data;
wire [9:0]  tmds_ch1_data;
wire [9:0]  tmds_ch2_data;
wire [9:0]  tmds_clk_data;

// 统一复位：TF 图像链路 + HDMI 链路
wire sys_pll_lock;
wire video_pll_lock;
wire rst_all;
wire rst_mem;
wire rst_sd;
wire rst_video;

reg [19:0] rst_cnt = 20'd0;
reg rst_all_reg = 1'b1;

// Manual pp.18-19: all four physical keys are active low. KEY4 is
// dedicated reset; KEY1 no longer shares the reset/auto input.
(* ASYNC_REG="TRUE" *) reg [1:0] reset_button_sync = 2'b11;
always @(posedge clk) reset_button_sync<={reset_button_sync[0],rst_n};
wire reset_pressed = !reset_button_sync[1];

always @(posedge clk) begin
    if (reset_pressed) begin
        rst_cnt <= 20'd0;
        rst_all_reg <= 1'b1;
    end else if (!rst_all_reg) begin
        // 一旦复位释放，忽略后续微小的 PLL 抖动，防止系统在使用时随机复位
        rst_all_reg <= 1'b0;
    end else if (sys_pll_lock && video_pll_lock) begin
        // 等待 PLL 连续稳定 20ms @ 50MHz
        if (rst_cnt < 20'd1_000_000) begin
            rst_cnt <= rst_cnt + 1'b1;
            rst_all_reg <= 1'b1;
        end else begin
            rst_all_reg <= 1'b0;
        end
    end else begin
        rst_cnt <= 20'd0;
        rst_all_reg <= 1'b1;
    end
end

assign rst_all = rst_all_reg;

// Album status is generated by sd_card_clk (100 MHz). Keep the complete
// page calculation, decimal decode and scan in that SAME domain. The old
// 100 MHz status -> 50 MHz display path required hold-fix buffers in the
// failed route. No unsynchronized status bus is sampled by clk now.
always @(posedge sd_card_clk or posedge rst_sd) begin
    if(rst_sd) begin
        display_page<=0; display_auto_mode<=0;
        display_digit_right<=8'hff; display_digit_left<=8'hff;
    end else begin
        display_page<=shown_page; display_auto_mode<=auto_status;
        // Register decoded glyphs separately from page arithmetic, keeping
        // the decimal/segment logic out of the scanner's output path.
        display_digit_right<=album_digit_right;
        display_digit_left<=album_digit_left;
    end
end

// STA also reported a clk(50 MHz) -> ext_mem_clk(125 MHz) reset-control
// violation inside the SDRAM core. Assert reset asynchronously, but release
// the memory subsystem only through memory-clocked registers.
(* ASYNC_REG="TRUE" *) reg [2:0] mem_reset_sync = 3'b111;
always @(posedge ext_mem_clk or posedge rst_all) begin
    if(rst_all) mem_reset_sync<=3'b111;
    else mem_reset_sync<={mem_reset_sync[1:0],1'b0};
end
assign rst_mem=mem_reset_sync[2];

// 保持你原来的 TF / SDRAM / video 时钟
// SD controller reset asserts immediately, but releases only on its own
// 100 MHz clock; avoid driving reader controls directly from a 50 MHz flop.
(* ASYNC_REG="TRUE" *) reg [2:0] sd_reset_sync = 3'b111;
always @(posedge sd_card_clk or posedge rst_all) begin
    if(rst_all) sd_reset_sync<=3'b111;
    else sd_reset_sync<={sd_reset_sync[1:0],1'b0};
end
assign rst_sd=sd_reset_sync[2];
(* ASYNC_REG="TRUE" *) reg [2:0] video_reset_sync=3'b111;
always @(posedge video_clk or posedge rst_all) begin
    if(rst_all) video_reset_sync<=3'b111;
    else video_reset_sync<={video_reset_sync[1:0],1'b0};
end
assign rst_video=video_reset_sync[2];

sys_pll sys_pll_m0(
    .refclk     (clk),
    .reset      (1'b0),
    .extlock    (sys_pll_lock),
    .clk0_out   (sd_card_clk),
    .clk1_out   (ext_mem_clk),
    .clk2_out   (ext_mem_clk_sft)
);

video_pll video_pll_m0(
    .refclk     (clk),
    .reset      (1'b0),
    .extlock    (video_pll_lock),
    .clk0_out   (video_clk),
    .clk1_out   (hdmi_5x_clk)
);

// mem_clk 域把 write_finish 单拍转成 toggle，供 sd_card_clk 域可靠同步
always @(posedge ext_mem_clk or posedge rst_mem) begin
    if (rst_mem)
        frame_write_toggle_mem <= 1'b0;
    else if (frame_write_finish)
        frame_write_toggle_mem <= ~frame_write_toggle_mem;
end

// ===================== TF 多图扫描与缓存（双缓冲） =====================
sd_card_bmp #(
    .CLK_FREQ_HZ       (100_000_000),
    .ENABLE_WAV        (1),
    .OUTPUT_WIDTH      (1280),
    .OUTPUT_HEIGHT     (720),
    .FILL_MODE         (1),
    .WRITE_BACKPRESSURE(1),
    .PIXEL_PERIOD      (4),
    .ENABLE_PREFETCH   (1),
    .COMBINED_PAGE_KEY (1),
    .AUTO_AUDIO_GAP_MS (1000),
    .ENABLE_TRANSITIONS(1),
    .ENABLE_WRITE_CANCEL(1),
    .SCAN_START_SECTOR (32'd0),
    .SCAN_MAX_SECTOR   (32'd131071),
    .SCAN_TARGET_COUNT (6'd0),          // Discover valid root BMPs automatically.
    .MAX_IMAGES        (32),
    .AUTO_INTERVAL_MS  (ALBUM_DWELL_MS)
) sd_card_bmp_m0(
    .clk               (sd_card_clk),
    .rst               (rst_sd),
    .key_next          (key2),
    .key_previous      (1'b1),
    .key_auto          (album_auto_key),
    .state_code        (state_code),
    .reader_state      (reader_state),
    .auto_status       (auto_status),
    .image_count       (image_count),
    .image_index       (image_index),
    .fault_code        (album_fault),
    .bmp_width         (16'd1280),
    .bmp_height        (16'd720),
    .display_valid     (display_valid),

    .write_finish_toggle(frame_write_toggle_mem),
    .write_buf_idx     (write_buf_idx),
    .disp_buf_idx      (disp_buf_idx),

    .write_req         (sd_card_write_req),
    .write_req_ack     (sd_card_write_req_ack),
    .write_en          (sd_card_write_en),
    .write_ready       (sd_card_write_ready),
    .active_display_buf(osd_frame_buffer),
    .write_data        (sd_card_write_data),
    .SD_nCS            (sd_ncs),
    .SD_DCLK           (sd_dclk),
    .SD_MOSI           (sd_mosi),
    .SD_MISO           (sd_miso),
    .pcm_send(pcm_send),.pcm_data(pcm_data),.pcm_last(pcm_last),.pcm_mute(pcm_mute),
    .pcm_ready(pcm_ready),.pcm_last_played(pcm_last_played),.audio_fault(wav_fault),
    .display_source_width(shown_source_width),.display_source_height(shown_source_height),
    .display_crop_width(shown_crop_width),.display_crop_height(shown_crop_height),
    .audio_complete(narration_complete),.audio_elapsed_cs(narration_elapsed_cs),.page_epoch(narration_epoch),
    .transition_command(transition_command_sd),.transition_reply(transition_reply_sd),
    .write_cancel(sd_card_write_cancel),.write_cancel_ack(sd_card_write_cancel_ack)
);

// Correct the displayed order and select polarity per board feedback:
// FPGA selects are active LOW; rightmost pair mapping is handled in seg_scan.
// Units/n -> SEL7/D11, tens/o -> SEL6/E11; other six selects stay HIGH/off.
seg_scan #(.ACTIVE_DIGITS(2),.CLK_FREQ(100000000)) seg_scan_m0(
    .clk(sd_card_clk), .rst_n(~rst_sd), .seg_sel(seg_sel), .seg_data(seg_data),
    .seg_data_0(display_digit_right), .seg_data_1(display_digit_left),
    .seg_data_2(8'hff), .seg_data_3(8'hff),
    .seg_data_4(8'hff), .seg_data_5(8'hff)
);
seg_decoder u_page_seg(.bin_data(decimal_ones(display_page)),.seg_data(page_segments));
seg_decoder u_page_tens_seg(.bin_data(decimal_tens(display_page)),.seg_data(page_tens_segments));

// ===================== 原图像时序与帧缓存 =====================
video_timing_data #(.H_ACTIVE(1280),.H_FP(110),.H_SYNC(40),.H_BP(220),
    .V_ACTIVE(720),.V_FP(5),.V_SYNC(5),.V_BP(20),.HS_POL(1),.VS_POL(1)) video_timing_data_m0(
    .video_clk         (video_clk),
    .rst               (rst_video),
    .read_req          (video_read_req),
    .read_req_ack      (video_read_req_ack),
    .hs                (hs_0),
    .vs                (vs_0),
    .de                (de_0)
);

video_delay video_delay_m0(
    .video_clk         (video_clk),
    .rst               (rst_video),
    .read_en           (video_read_en),
    .read_data         (video_read_data[31:8]),
    .hs                (hs_0),
    .vs                (vs_0),
    .de                (de_0),
    .hs_r              (hs),
    .vs_r              (vs),
    .de_r              (de),
    .vout_data         (vout_data_raw)
);

// 首图提交前黑屏；提交后一直显示当前显示缓冲区内容
wire [87:0] video_status;
album_status_cdc #(.WIDTH(88)) u_status_cdc(
    .source_clk(sd_card_clk),.source_rst(rst_sd),
    .source_data({shown_crop_width,shown_crop_height,display_valid,auto_status,narration_complete,wav_fault,
        shown_page,image_count,shown_source_width,shown_source_height,disp_buf_idx,3'd0}),
    .dest_clk(video_clk),.dest_rst(rst_video),.dest_data(video_status)
);
assign vout_data = osd_frame_valid ? vout_data_raw : 24'd0;
wire [7:0] picture_settings,video_picture_settings;
album_picture_controls u_picture_controls(
    .clk(sd_card_clk),.rst(rst_sd),.button_in(key3),.settings(picture_settings));
album_status_cdc #(.WIDTH(8)) u_picture_settings_cdc(
    .source_clk(sd_card_clk),.source_rst(rst_sd),.source_data(picture_settings),
    .dest_clk(video_clk),.dest_rst(rst_video),.dest_data(video_picture_settings));
album_status_cdc #(.WIDTH(24)) u_narration_cdc(
    .source_clk(sd_card_clk),.source_rst(rst_sd),
    .source_data({narration_epoch,shown_page,!pcm_mute,narration_elapsed_cs}),
    .dest_clk(video_clk),.dest_rst(rst_video),.dest_data(video_narration));
album_video_osd #(.ENABLE_ADJUST(1),.X_POS(4),.Y_POS(4)) u_album_osd(
    .clk(video_clk),.rst(rst_video),.hs_in(hs),.vs_in(vs),.de_in(de),
    .rgb_in(vout_data),.status(video_status),.picture_settings(video_picture_settings),
    .hs_out(osd_hs),.vs_out(osd_vs),.de_out(osd_de),.rgb_out(osd_rgb),
    .frame_valid(osd_frame_valid),.frame_buffer(osd_frame_buffer),.frame_page(osd_frame_page),
    .sidebar_visible(osd_sidebar_visible)
);
album_status_cdc #(.WIDTH(9)) u_transition_command_cdc(
    .source_clk(sd_card_clk),.source_rst(rst_sd),.source_data(transition_command_sd),
    .dest_clk(video_clk),.dest_rst(rst_video),.dest_data(transition_command_video));
album_status_cdc #(.WIDTH(2)) u_transition_reply_cdc(
    .source_clk(video_clk),.source_rst(rst_video),.source_data(transition_reply_video),
    .dest_clk(sd_card_clk),.dest_rst(rst_sd),.dest_data(transition_reply_sd));
album_teaching_scene #(.STATUS_X(4),.STATUS_Y(4),.ENABLE_TRANSITIONS(1)) u_teaching_scene(
    .clk(video_clk),.rst(rst_video),.hs_in(osd_hs),.vs_in(osd_vs),.de_in(osd_de),.rgb_in(osd_rgb),
    .frame_valid(osd_frame_valid),.visible_page(osd_frame_page),.narration(video_narration),
    .sidebar_visible(osd_sidebar_visible),
    .transition_command(transition_command_video),.transition_reply(transition_reply_video),
    .hs_out(scene_hs),.vs_out(scene_vs),.de_out(scene_de),.rgb_out(scene_rgb));

frame_read_write #(
    .WRITE_V_FLIP     (1),
    .FRAME_WIDTH      (1280),
    .FRAME_HEIGHT     (720),
    .ENABLE_WRITE_CANCEL(1)
) frame_read_write_m0(
    .mem_clk           (ext_mem_clk),
    .rst               (rst_mem),
    .Sdr_init_done     (Sdr_init_done),
    .Sdr_init_ref_vld  (Sdr_init_ref_vld),
    .Sdr_busy          (Sdr_busy),

    .App_rd_en         (App_rd_en),
    .App_rd_addr       (App_rd_addr),
    .Sdr_rd_en         (Sdr_rd_en),
    .Sdr_rd_dout       (Sdr_rd_dout),

    .read_clk          (video_clk),
    .read_req          (video_read_req),
    .read_req_ack      (video_read_req_ack),
    .read_finish       (),
    .read_addr_0       (BUF0_ADDR),
    .read_addr_1       (BUF1_ADDR),
    .read_addr_2       (24'd0),
    .read_addr_3       (24'd0),
    .read_addr_index   (osd_frame_buffer),
    .read_len          (FRAME_PIXELS),
    .read_en           (video_read_en),
    .read_data         (video_read_data),

    .App_wr_en         (App_wr_en),
    .App_wr_addr       (App_wr_addr),
    .App_wr_din        (App_wr_din),
    .App_wr_dm         (App_wr_dm),

    .write_clk         (sd_card_clk),
    .write_req         (sd_card_write_req),
    .write_req_ack     (sd_card_write_req_ack),
    .write_finish      (frame_write_finish),
    .write_addr_0      (BUF0_ADDR),
    .write_addr_1      (BUF1_ADDR),
    .write_addr_2      (24'd0),
    .write_addr_3      (24'd0),
    .write_addr_index  (write_buf_idx),
    .write_len         (FRAME_PIXELS),
    .write_en          (sd_card_write_en),
    .write_ready       (sd_card_write_ready),
    .write_data        (sd_card_write_data),
    .write_cancel      (sd_card_write_cancel),
    .write_cancel_ack  (sd_card_write_cancel_ack)
);

sdram U3(
    .Clk               (ext_mem_clk),
    .Clk_sft           (ext_mem_clk_sft),
    .Rst               (rst_mem),
    .Sdr_init_done     (Sdr_init_done),
    .Sdr_init_ref_vld  (Sdr_init_ref_vld),
    .Sdr_busy          (Sdr_busy),
    .App_wr_en         (App_wr_en),
    .App_wr_addr       (App_wr_addr),
    .App_wr_dm         (App_wr_dm),
    .App_wr_din        (App_wr_din),
    .App_rd_en         (App_rd_en),
    .App_rd_addr       (App_rd_addr),
    .Sdr_rd_en         (Sdr_rd_en),
    .Sdr_rd_dout       (Sdr_rd_dout)
);

// ===================== RGB/DE 转 AXIS 视频 =====================
album_pcm_hdmi_bridge #(.PIXEL_HZ(74250000)) u_wav_hdmi_bridge(
    .sd_clk(sd_card_clk),.sd_rst(rst_sd),.video_clk(video_clk),.video_rst(rst_video),
    .sample_send(pcm_send),.sample_data(pcm_data),.sample_last(pcm_last),.mute(pcm_mute),
    .sample_ready(pcm_ready),.last_played_pulse(pcm_last_played),
    .audio_valid(audio_valid),.audio_left(audio_left_data),.audio_right(audio_right_data),
    .acr_valid(acr_valid),.acr_cts(acr_cts),.acr_n(acr_n)
);

// Tap the final, mute-resolved HDMI PCM stream in its existing video domain.
// The overlay never back-pressures playback or touches SD/SDRAM scheduling.
album_audio_waveform u_audio_waveform(
    .clk(video_clk),.rst(rst_video),
    .hs_in(scene_hs),.vs_in(scene_vs),.de_in(scene_de),.rgb_in(scene_rgb),
    .frame_valid(osd_frame_valid),.visible_page(osd_frame_page),
    .narration_epoch(video_narration[23]),
    .sample_valid(audio_valid),.sample_data(audio_left_data[23:8]),
    .hs_out(waveform_hs),.vs_out(waveform_vs),.de_out(waveform_de),.rgb_out(waveform_rgb));

video_rgb_to_axis_640x480 #(.ACTIVE_WIDTH(1280)) u_video_rgb_to_axis_640x480(
    .I_clk         (video_clk),
    .I_rst         (rst_video),
    .I_vs          (waveform_vs),
    .I_de          (waveform_de),
    .I_rgb         (waveform_rgb),
    .O_video_user  (axis_s_user),
    .O_video_valid (axis_s_valid),
    .O_video_last  (axis_s_last),
    .O_video_data  (axis_s_data)
);

// 上电后自动打一拍，触发一次 EDID 读取
startup_pulse #(
    .CNT_MAX(20'd100000)
) u_startup_pulse (
    .I_clk   (video_clk),
    .I_rst   (rst_video),
    .O_pulse (edid_trig)
);

// ===================== HDMI 1.4b 发射 =====================
hdmi_1_4b_transmitter_core_wrapper #(
    .DEVICE                 ( "EG"       ),
    .HTOTAL                 ( 1650       ),
    .HSA                    ( 40         ),
    .HFP                    ( 110        ),
    .HBP                    ( 220        ),
    .HACTIVE                ( 1280       ),
    .VTOTAL                 ( 750        ),
    .VSA                    ( 5          ),
    .VFP                    ( 5          ),
    .VBP                    ( 20         ),
    .VACTIVE                ( 720        ),
    .VIDEO_VIC              ( 4          ),
    .VIDEO_TPG              ( "Disable"  ),
    .VIDEO_FORMAT           ( "RGB"      ),
    .AUDIO_SAMPLE_RATE      ( "48K"      ),
    .IIC_SCL_DIV            ( 743        )
) u_hdmi_1_4b_transmitter_core_wrapper(
    .I_pixel_clk        (video_clk),
    .I_rst              (rst_video),
    .I_edid_read_trig   (edid_trig),
    .O_edid_read_valid  (edid_valid),
    .O_edid_read_data   (edid_data),

    .I_axis_s_user      (axis_s_user),
    .I_axis_s_valid     (axis_s_valid),
    .I_axis_s_last      (axis_s_last),
    .I_axis_s_data      (axis_s_data),
    .O_axis_s_ready     (axis_s_ready),

    .I_audio_valid      (audio_valid),
    .I_audio_left_data  (audio_left_data),
    .I_audio_right_data (audio_right_data),
    .I_acr_valid        (acr_valid),
    .I_acr_cts          (acr_cts),
    .I_acr_n            (acr_n),

    .O_video_locked     (),
    .O_ddc_scl          (HDMI_DDC_SCL),
    .IO_ddc_sda         (HDMI_DDC_SDA),

    .O_ch0_tmds_data    (tmds_ch0_data),
    .O_ch1_tmds_data    (tmds_ch1_data),
    .O_ch2_tmds_data    (tmds_ch2_data),
    .O_clk_tmds_data    (tmds_clk_data)
);

hdmi_phy_wrapper #(
    .DEVICE ( "EG" )
) u_hdmi2phy_wrapper(
    .I_pixel_clk        (video_clk),
    .I_serial_clk       (hdmi_5x_clk),
    .I_rst              (rst_video),
    .I_tmds_channel_0   (tmds_ch0_data),
    .I_tmds_channel_1   (tmds_ch1_data),
    .I_tmds_channel_2   (tmds_ch2_data),
    .I_tmds_channel_clk (tmds_clk_data),
    .O_tmds_ch0_p       (HDMI_D0_P),
    .O_tmds_ch1_p       (HDMI_D1_P),
    .O_tmds_ch2_p       (HDMI_D2_P),
    .O_tmds_clk_p       (HDMI_CLK_P)
);

endmodule

// Keep the test-audio definition with its caller: TD may retain an older
// project source list when the .al file is edited outside the GUI.
// Internal test scale, entirely synchronous to the 25 MHz HDMI pixel clock.
// 48000 stereo samples/s; ACR N=6144, CTS=25000. No I2S CDC or audio PLL.
module hdmi_test_audio #(
    parameter integer NOTE_SAMPLES=24000 // >=512; shorten only in a testbench
)(
    input wire clk,
    input wire rst,
    input wire play_enable,
    output reg audio_valid,
    output reg [23:0] audio_left,
    output reg [23:0] audio_right,
    output reg acr_valid,
    output wire [19:0] acr_n,
    output wire [19:0] acr_cts
);
    localparam [24:0] PIXEL_HZ=25'd25000000;
    localparam [24:0] SAMPLE_HZ=25'd48000;
    reg [24:0] sample_phase;
    wire sample_tick=(sample_phase>=PIXEL_HZ-SAMPLE_HZ);
    reg [14:0] acr_count;
    assign acr_n=20'd6144;
    assign acr_cts=20'd25000;

    reg [31:0] tone_phase;
    reg [2:0] note_index;
    reg [14:0] note_sample;
    reg [31:0] note_step;
    always @* begin
        case(note_index)
            0: note_step=32'd23409862;
            1: note_step=32'd26276681;
            2: note_step=32'd29494578;
            3: note_step=32'd31248410;
            4: note_step=32'd35075155;
            5: note_step=32'd39370534;
            6: note_step=32'd44191930;
            default: note_step=32'd46819716;
        endcase
    end

    wire [5:0] quarter_index=tone_phase[30] ?
        6'd32-{1'b0,tone_phase[29:25]} : {1'b0,tone_phase[29:25]};
    reg [15:0] magnitude;
    always @* begin
        case(quarter_index)
            0:magnitude=0;       1:magnitude=1608;
            2:magnitude=3212;    3:magnitude=4808;
            4:magnitude=6393;    5:magnitude=7962;
            6:magnitude=9512;    7:magnitude=11039;
            8:magnitude=12539;   9:magnitude=14010;
            10:magnitude=15446;  11:magnitude=16846;
            12:magnitude=18204;  13:magnitude=19519;
            14:magnitude=20787;  15:magnitude=22005;
            16:magnitude=23170;  17:magnitude=24279;
            18:magnitude=25329;  19:magnitude=26319;
            20:magnitude=27245;  21:magnitude=28105;
            22:magnitude=28898;  23:magnitude=29621;
            24:magnitude=30273;  25:magnitude=30852;
            26:magnitude=31356;  27:magnitude=31785;
            28:magnitude=32137;  29:magnitude=32412;
            30:magnitude=32609;  31:magnitude=32728;
            default:magnitude=32767;
        endcase
    end
    wire signed [16:0] signed_sine=tone_phase[31] ?
        -$signed({1'b0,magnitude}) : $signed({1'b0,magnitude});
    reg [7:0] envelope;
    always @* begin
        if(note_sample<15'd256) envelope=note_sample[7:0];
        else if(note_sample>=NOTE_SAMPLES-256) envelope=NOTE_SAMPLES-1-note_sample;
        else envelope=8'd255;
    end

    reg signed [16:0] sample_sine;
    reg signed [8:0] sample_gain;
    reg signed [25:0] sample_product;
    reg sample_pending, product_pending;
    always @(posedge clk or posedge rst) begin
        if(rst) begin
            sample_phase<=0; acr_count<=0; acr_valid<=0;
            tone_phase<=0; note_index<=0; note_sample<=0;
            sample_sine<=0; sample_gain<=0; sample_product<=0;
            sample_pending<=0; product_pending<=0;
            audio_valid<=0; audio_left<=0; audio_right<=0;
        end else begin
            // The fixed rational accumulator has no long-term sample-rate drift.
            if(sample_tick) sample_phase<=sample_phase-(PIXEL_HZ-SAMPLE_HZ);
            else sample_phase<=sample_phase+SAMPLE_HZ;
            acr_valid<=0;
            if(acr_count==15'd24999) begin acr_count<=0; acr_valid<=1; end
            else acr_count<=acr_count+1'b1;

            // Keep silence samples and ACR running while the first photo loads.
            sample_pending<=sample_tick;
            product_pending<=sample_pending;
            sample_product<=sample_sine*sample_gain;
            audio_valid<=product_pending;
            if(product_pending) begin
                // Full envelope peak is about 1/8 of signed 24-bit full scale.
                audio_left<=sample_product >>> 3;
                audio_right<=sample_product >>> 3;
            end
            if(!play_enable) begin
                tone_phase<=0; note_index<=0; note_sample<=0;
            end
            if(sample_tick) begin
                sample_sine<=play_enable ? signed_sine : 17'sd0;
                sample_gain<=play_enable ? $signed({1'b0,envelope}) : 9'sd0;
                if(play_enable) begin
                    if(note_sample==NOTE_SAMPLES-1) begin
                        note_sample<=0; note_index<=note_index+1'b1; tone_phase<=0;
                    end else begin
                        note_sample<=note_sample+1'b1; tone_phase<=tone_phase+note_step;
                    end
                end
            end
        end
    end
endmodule



// Held-data toggle mailbox: coherent status bus, never independent bit sampling.
module album_status_cdc #(parameter integer WIDTH=56)(
    input source_clk,source_rst,input [WIDTH-1:0] source_data,
    input dest_clk,dest_rst,output reg [WIDTH-1:0] dest_data
);
reg [WIDTH-1:0] held;
reg request_toggle,ack_toggle,seen;
(* ASYNC_REG="TRUE" *) reg [1:0] ack_sync,request_sync;
always @(posedge source_clk or posedge source_rst) begin
    if(source_rst) begin held<=0; request_toggle<=0; ack_sync<=0; end
    else begin
        ack_sync<={ack_sync[0],ack_toggle};
        if(request_toggle==ack_sync[1] && source_data!=held) begin
            held<=source_data; request_toggle<=~request_toggle;
        end
    end
end
always @(posedge dest_clk or posedge dest_rst) begin
    if(dest_rst) begin request_sync<=0; seen<=0; ack_toggle<=0; dest_data<=0; end
    else begin
        request_sync<={request_sync[0],request_toggle};
        if(request_sync[1]!=seen) begin
            dest_data<=held; seen<=request_sync[1]; ack_toggle<=request_sync[1];
        end
    end
end
endmodule

// KEY3 settings remain in the SD clock domain. Pack both parameters and UI
// state into one held-data mailbox; video applies the snapshot at frame start.
module album_picture_controls #(
    parameter integer CLK_FREQ_HZ=100000000,
    parameter integer HIDE_MS=2000
)(
    input clk,rst,button_in,
    output [7:0] settings
);
localparam integer VISIBLE_CYCLES=(CLK_FREQ_HZ/1000)*HIDE_MS;
wire short_event,long_event,press_event,button_down;
reg [2:0] brightness,contrast;
reg contrast_selected;
reg [31:0] visible_count;
album_short_long_button #(.CLK_FREQ_HZ(CLK_FREQ_HZ),.DEBOUNCE_MS(20),.LONG_MS(800)) u_adjust_button(
    .clk(clk),.rst(rst),.button_in(button_in),.short_pulse(short_event),
    .long_pulse(long_event),.press_pulse(press_event),.button_down(button_down));
assign settings={visible_count!=0,contrast_selected,brightness,contrast};
always @(posedge clk or posedge rst) begin
    if(rst) begin
        brightness<=2; contrast<=2; contrast_selected<=0; visible_count<=0;
    end else begin
        if(button_down || short_event || long_event || press_event) visible_count<=VISIBLE_CYCLES;
        else if(visible_count!=0) visible_count<=visible_count-1'b1;
        if(long_event) contrast_selected<=~contrast_selected;
        else if(short_event) begin
            if(contrast_selected) contrast<=contrast==4 ? 3'd0 : contrast+1'b1;
            else brightness<=brightness==4 ? 3'd0 : brightness+1'b1;
        end
    end
end
endmodule

// 1280x720 overlay; four 16-pixel rows, 256x64 dimmed panel at top left.
// All RGB and sync signals traverse the same two registered pixel stages.
module album_video_osd #(
    parameter integer X_POS=16,
    parameter integer Y_POS=16,
    parameter integer ENABLE_ADJUST=0
)(
    input clk,rst,hs_in,vs_in,de_in,input [23:0] rgb_in,
    input [87:0] status,
    input [7:0] picture_settings, // visible, contrast-selected, brightness[2:0], contrast[2:0]
    output reg hs_out,vs_out,de_out,output reg [23:0] rgb_out,
    output frame_valid,output [1:0] frame_buffer,output [5:0] frame_page,output sidebar_visible
);
reg vs_last,de_last;
reg [10:0] x,y;
reg [87:0] frame_status;
reg [7:0] frame_settings;
assign sidebar_visible=ENABLE_ADJUST!=0 && frame_settings[7];
// Buffer selection and its metadata are captured together at VS rising.
// SDRAM read_req is issued at VS falling, five lines later in 720p.
assign frame_valid=frame_status[55];
assign frame_buffer=frame_status[4:3];
assign frame_page=frame_status[48:43];
reg [15:0] width_shift,height_shift,width_bcd,height_bcd;
reg [15:0] width_work,height_work;
reg [4:0] convert_left;
reg [7:0] page_tens,page_ones,count_tens,count_ones;
reg [127:0] text_line;
reg [7:0] character_s1;
reg [34:0] glyph_s2;
reg [2:0] glyph_x_s1,glyph_y_s1;
reg [2:0] glyph_x_s2,glyph_y_s2;
reg panel_s1,ink_area_s1,border_s1,hs_s1,vs_s1,de_s1,hud_s1;
reg panel_s2,ink_area_s2,border_s2,hs_s2,vs_s2,de_s2,hud_s2;
reg [23:0] rgb_s1;
reg [23:0] adjusted_rgb_s2;
reg signed [11:0] red_q4_s1,green_q4_s1,blue_q4_s1,bias_q4_s1;
reg dot_s1,dot_on_s1;
reg dot_s2,dot_on_s2;
// Contrast factors in quarters: 2,3,4,5,6. Brightness offsets:
// -64,-32,0,+32,+64. Default level 2/2 is an exact identity for all RGB values.
function signed [11:0] contrast_q4;
    input [7:0] value;
    input [2:0] level;
    reg signed [11:0] centered;
    begin
        centered=$signed({1'b0,value})-12'sd128;
        case(level)
            0:contrast_q4=centered<<<1;
            1:contrast_q4=(centered<<<1)+centered;
            3:contrast_q4=(centered<<<2)+centered;
            4:contrast_q4=(centered<<<2)+(centered<<<1);
            default:contrast_q4=centered<<<2;
        endcase
    end
endfunction
function signed [11:0] brightness_bias;
    input [2:0] level;
    begin
        case(level)
            0:brightness_bias=12'sd256;
            1:brightness_bias=12'sd384;
            3:brightness_bias=12'sd640;
            4:brightness_bias=12'sd768;
            default:brightness_bias=12'sd512;
        endcase
    end
endfunction
function [7:0] clip_q4;
    input signed [12:0] value;
    begin
        if(value<0) clip_q4=0;
        else if(value>13'sd1020) clip_q4=255;
        else clip_q4=value[9:2];
    end
endfunction
wire [23:0] adjusted_rgb=ENABLE_ADJUST==0 ? rgb_s1 : !frame_status[55] ? 24'd0 : {
    clip_q4($signed({red_q4_s1[11],red_q4_s1})+$signed({bias_q4_s1[11],bias_q4_s1})),
    clip_q4($signed({green_q4_s1[11],green_q4_s1})+$signed({bias_q4_s1[11],bias_q4_s1})),
    clip_q4($signed({blue_q4_s1[11],blue_q4_s1})+$signed({bias_q4_s1[11],bias_q4_s1}))};
// Temporary sidebar occupies x0..143, y192..527, below the four status rows.
wire sidebar=ENABLE_ADJUST!=0 && de_in && frame_settings[7] && x<144 && y>=192 && y<528;
wire side_label=sidebar && x>=8 && x<136 && y>=208 && y<224;
wire [10:0] side_x=x-11'd8,side_y=y-11'd208;
wire [2:0] side_col=side_x[6:4];
wire [127:0] side_line=frame_settings[6] ? "CONTRAST        " : "BRIGHT          ";
wire [7:0] side_character=side_line[127-side_col*8 -:8];
wire [2:0] selected_level=frame_settings[6] ? frame_settings[2:0] : frame_settings[5:3];
wire [10:0] dot_dx=x>=72 ? x-11'd72 : 11'd72-x;
reg [2:0] dot_number;
reg [10:0] dot_dy;
reg [3:0] dot_radius;
always @* begin
    dot_number=0; dot_dy=11;
    if(y>=266 && y<=286) begin dot_number=5; dot_dy=y>=276 ? y-11'd276 : 11'd276-y; end
    else if(y>=318 && y<=338) begin dot_number=4; dot_dy=y>=328 ? y-11'd328 : 11'd328-y; end
    else if(y>=370 && y<=390) begin dot_number=3; dot_dy=y>=380 ? y-11'd380 : 11'd380-y; end
    else if(y>=422 && y<=442) begin dot_number=2; dot_dy=y>=432 ? y-11'd432 : 11'd432-y; end
    else if(y>=474 && y<=494) begin dot_number=1; dot_dy=y>=484 ? y-11'd484 : 11'd484-y; end
    // Integer radius-10 circle, using a small row lookup instead of squaring.
    case(dot_dy)
        0:dot_radius=10;
        1,2,3,4:dot_radius=9;
        5,6:dot_radius=8;
        7:dot_radius=7;
        8:dot_radius=6;
        9:dot_radius=4;
        default:dot_radius=0;
    endcase
end
wire panel=de_in && x>=X_POS && x<X_POS+224 && y>=Y_POS && y<Y_POS+64;
wire [10:0] local_x=x-X_POS;
wire [10:0] local_y=y-Y_POS;
wire [3:0] text_col=local_x[7:4];
wire [2:0] text_row=local_y[6:4];
wire [7:0] character=text_line[127-text_col*8 -: 8];
// Constant slices, rather than a subtract/multiply/35-bit variable shift.
function [4:0] glyph_row;
    input [34:0] bitmap;
    input [2:0] row;
    begin
        case(row)
            0:glyph_row=bitmap[34:30];1:glyph_row=bitmap[29:25];
            2:glyph_row=bitmap[24:20];3:glyph_row=bitmap[19:15];
            4:glyph_row=bitmap[14:10];5:glyph_row=bitmap[9:5];
            6:glyph_row=bitmap[4:0];default:glyph_row=0;
        endcase
    end
endfunction
wire [4:0] font_row=glyph_row(glyph_s2,glyph_y_s2);
wire ink=ink_area_s2 && font_row[4-glyph_x_s2];
function [7:0] digit;
    input [3:0] n;
    begin digit=8'd48+{4'd0,n}; end
endfunction
function [7:0] paper_channel;
    input [7:0] background,paper;
    reg [9:0] sum;
    begin sum={2'd0,background}+{1'b0,paper,1'b0}+{2'd0,paper};paper_channel=sum[9:2];end
endfunction
wire [23:0] hud_paper={paper_channel(adjusted_rgb_s2[23:16],8'hf1),
    paper_channel(adjusted_rgb_s2[15:8],8'he5),paper_channel(adjusted_rgb_s2[7:0],8'hcb)};
function [7:0] tens;
    input [5:0] n;
    begin
        if(n>=30) tens="3"; else if(n>=20) tens="2";
        else if(n>=10) tens="1"; else tens="0";
    end
endfunction
function [7:0] ones;
    input [5:0] n;
    begin
        if(n>=30) ones=8'd48+n-6'd30; else if(n>=20) ones=8'd48+n-6'd20;
        else if(n>=10) ones=8'd48+n-6'd10; else ones=8'd48+n;
    end
endfunction
function [15:0] bcd_adjust;
    input [15:0] n;
    reg [15:0] r;
    integer i;
    begin
        r=n;
        for(i=0;i<4;i=i+1) if(n[i*4 +:4]>=5) r[i*4 +:4]=n[i*4 +:4]+4'd3;
        bcd_adjust=r;
    end
endfunction
function [34:0] glyph;
    input [7:0] ch;
    begin
        case(ch)
            8'd48: glyph=35'h3a33ae62e;
            8'd49: glyph=35'h11842108e;
            8'd50: glyph=35'h3a211111f;
            8'd51: glyph=35'h78217043e;
            8'd52: glyph=35'h08ca97c42;
            8'd53: glyph=35'h7e10f043e;
            8'd54: glyph=35'h3a10f462e;
            8'd55: glyph=35'h7c2222108;
            8'd56: glyph=35'h3a317462e;
            8'd57: glyph=35'h3a317842e;
            8'd65: glyph=35'h3a31fc631;
            8'd66: glyph=35'h7a31f463e;
            8'd67: glyph=35'h3a308422e;
            8'd68: glyph=35'h7a318c63e;
            8'd69: glyph=35'h7e10f421f;
            8'd70: glyph=35'h7e10f4210;
            8'd71: glyph=35'h3a30bc62f;
            8'd72: glyph=35'h4631fc631;
            8'd73: glyph=35'h38842108e;
            8'd76: glyph=35'h42108421f;
            8'd77: glyph=35'h4775ac631;
            8'd78: glyph=35'h47359c631;
            8'd79: glyph=35'h3a318c62e;
            8'd80: glyph=35'h7a31f4210;
            8'd82: glyph=35'h7a31f5251;
            8'd83: glyph=35'h3e107043e;
            8'd84: glyph=35'h7c8421084;
            8'd85: glyph=35'h46318c62e;
            8'd87: glyph=35'h4631ad6aa;
            8'd89: glyph=35'h462a21084;
            8'd120: glyph=35'h001151151;
            8'd47: glyph=35'h044222110;
            default: glyph=0;
        endcase
    end
endfunction
always @* begin
    case(text_row)
        0:text_line={"IN  ",digit(width_bcd[15:12]),digit(width_bcd[11:8]),
            digit(width_bcd[7:4]),digit(width_bcd[3:0]),"x",
            digit(height_bcd[15:12]),digit(height_bcd[11:8]),
            digit(height_bcd[7:4]),digit(height_bcd[3:0]),"   "};
        1:text_line="OUT 1280x0720   ";
        2:text_line={"PAGE ",page_tens,page_ones,"/",count_tens,count_ones,"      "};
        3:text_line=frame_status[54] ? "MODE AUTO       " : "MODE MANUAL     ";
        default:text_line=128'd0;
    endcase
end
always @(posedge clk or posedge rst) begin
    if(rst) begin
        x<=0; y<=0; vs_last<=0; de_last<=0; frame_status<=0;
        frame_settings<={1'b0,1'b0,3'd2,3'd2};
        red_q4_s1<=0; green_q4_s1<=0; blue_q4_s1<=0; bias_q4_s1<=512;
        dot_s1<=0; dot_on_s1<=0;
        width_shift<=0; height_shift<=0; width_bcd<=0; height_bcd<=0;
        width_work<=0; height_work<=0; convert_left<=0;
        page_tens<="0"; page_ones<="0"; count_tens<="0"; count_ones<="0";
        character_s1<=0;glyph_s2<=0; glyph_x_s1<=0; glyph_y_s1<=0;
        glyph_x_s2<=0;glyph_y_s2<=0;adjusted_rgb_s2<=0;
        panel_s2<=0;ink_area_s2<=0;border_s2<=0;hud_s1<=0;hud_s2<=0;
        hs_s2<=0;vs_s2<=0;de_s2<=0;dot_s2<=0;dot_on_s2<=0;
        panel_s1<=0; ink_area_s1<=0; border_s1<=0;
        hs_s1<=0; vs_s1<=0; de_s1<=0; rgb_s1<=0;
        hs_out<=0; vs_out<=0; de_out<=0; rgb_out<=0;
    end else begin
        vs_last<=vs_in; de_last<=de_in;
        if(de_in) x<=x+1'b1; else x<=0;
        if(vs_in && !vs_last) begin
            x<=0; y<=0; frame_status<=status;
            if(ENABLE_ADJUST!=0) frame_settings<=picture_settings;
            width_shift<=status[36:21]; height_shift<=status[20:5];
            width_work<=0; height_work<=0; convert_left<=16;
            page_tens<=tens(status[48:43]); page_ones<=ones(status[48:43]);
            count_tens<=tens(status[42:37]); count_ones<=ones(status[42:37]);
        end else begin
            if(de_last && !de_in) y<=y+1'b1;
            if(convert_left!=0) begin
                width_work<=(bcd_adjust(width_work)<<1)|{15'd0,width_shift[15]};
                height_work<=(bcd_adjust(height_work)<<1)|{15'd0,height_shift[15]};
                width_shift<=width_shift<<1; height_shift<=height_shift<<1;
                convert_left<=convert_left-1'b1;
                if(convert_left==1) begin
                    width_bcd<=(bcd_adjust(width_work)<<1)|{15'd0,width_shift[15]};
                    height_bcd<=(bcd_adjust(height_work)<<1)|{15'd0,height_shift[15]};
                end
            end
        end
        red_q4_s1<=contrast_q4(rgb_in[23:16],frame_settings[2:0]);
        green_q4_s1<=contrast_q4(rgb_in[15:8],frame_settings[2:0]);
        blue_q4_s1<=contrast_q4(rgb_in[7:0],frame_settings[2:0]);
        bias_q4_s1<=brightness_bias(frame_settings[5:3]);
        // Stage1 chooses only the character. Stage2 decodes its glyph, cutting
        // the measured text-row/column -> character mux -> font-table path.
        character_s1<=side_label ? side_character : character;
        glyph_x_s1<=side_label ? side_x[3:1] : local_x[3:1];
        glyph_y_s1<=side_label ? side_y[3:1] : local_y[3:1];
        ink_area_s1<=(panel && local_x[3:1]<5 && local_y[3:1]<7) ||
                     (side_label && side_x[3:1]<5 && side_y[3:1]<7);
        panel_s1<=panel || sidebar;
        hud_s1<=panel;
        dot_s1<=sidebar && dot_number!=0 && dot_dy<=10 && dot_dx<={7'd0,dot_radius};
        dot_on_s1<=dot_number<={1'b0,selected_level}+4'd1;
        border_s1<=(panel && (x==X_POS || x==X_POS+223 || y==Y_POS || y==Y_POS+63)) ||
                    (sidebar && (x==0 || x==143 || y==192 || y==527));
        hs_s1<=hs_in; vs_s1<=vs_in; de_s1<=de_in; rgb_s1<=rgb_in;
        glyph_s2<=glyph(character_s1);glyph_x_s2<=glyph_x_s1;glyph_y_s2<=glyph_y_s1;
        ink_area_s2<=ink_area_s1;panel_s2<=panel_s1;border_s2<=border_s1;hud_s2<=hud_s1;
        dot_s2<=dot_s1;dot_on_s2<=dot_on_s1;adjusted_rgb_s2<=adjusted_rgb;
        hs_s2<=hs_s1;vs_s2<=vs_s1;de_s2<=de_s1;
        // Stage3 composites the matching picture, glyph and UI masks.
        hs_out<=hs_s2; vs_out<=vs_s2; de_out<=de_s2;
        if(!de_s2) rgb_out<=0;
        else if(border_s2) rgb_out<=hud_s2 ? 24'ha18858 : 24'h4080a0;
        else if(ink) rgb_out<=hud_s2 ? 24'h382c22 : 24'hffffff;
        else if(dot_s2) rgb_out<=dot_on_s2 ? 24'h40e0ff : 24'h505860;
        else if(hud_s2) rgb_out<=hud_paper;
        else if(panel_s2) rgb_out<={2'd0,adjusted_rgb_s2[23:18],2'd0,adjusted_rgb_s2[15:10],2'd0,adjusted_rgb_s2[7:2]};
        else rgb_out<=adjusted_rgb_s2;
    end
end
endmodule

// Signed min/max history of the played PCM, not a frequency spectrum.
// 1536 samples/column at 48 kHz = 32 ms; 128 columns retain 4.096 seconds.
// New columns enter on the right. The circular history advances only on VS,
// and RAM is never written during an active frame. No full-frame buffer.
module album_audio_waveform #(
    parameter integer BIN_SAMPLES=1536,
    parameter integer X_POS=240,Y_POS=4,
    parameter integer SPECTRUM_X_POS=540,ENABLE_SPECTRUM=1
)(
    input clk,rst,hs_in,vs_in,de_in,input [23:0] rgb_in,
    input frame_valid,input [5:0] visible_page,input narration_epoch,
    input sample_valid,input signed [15:0] sample_data,
    output reg hs_out,vs_out,de_out,output reg [23:0] rgb_out
);
reg [10:0] x,y;
reg de_last,vs_last;
reg [5:0] page_seen;
reg epoch_seen;
reg [15:0] bin_samples;
reg signed [6:0] bin_min,bin_max;
reg [6:0] write_pointer;
reg [7:0] display_count,first_visible;
reg [13:0] pending_peaks;
reg pending_valid;
reg [13:0] peaks[0:127]; /* fehdl force_ram=1, ram_style="bram" */
reg [13:0] peaks_q;
wire signed [6:0] pcm=sample_data[15:9];
wire signed [6:0] next_min=bin_samples==0 || pcm<bin_min ? pcm : bin_min;
wire signed [6:0] next_max=bin_samples==0 || pcm>bin_max ? pcm : bin_max;
wire clear_track=!frame_valid || visible_page==0 || visible_page!=page_seen ||
                 narration_epoch!=epoch_seen;
wire [5:0] spectrum_read_level;
wire [1:0] spectrum_published_bank;
wire spectrum_ready,spectrum_update,spectrum_overrun,spectrum_overflow;
reg [1:0] spectrum_read_bank;
reg [6:0] spectrum_scan;
reg spectrum_scan_busy,spectrum_scan_first,spectrum_filled_valid,spectrum_scan_phase;
album_pcm_spectrum spectrum(
    .clk(clk),.rst(rst),.clear(clear_track || ENABLE_SPECTRUM==0),
    .sample_valid(sample_valid && ENABLE_SPECTRUM!=0),.sample_data(sample_data),
    .read_bank(spectrum_read_bank),.read_address(spectrum_scan),.read_lock(spectrum_scan_busy),
    .read_level(spectrum_read_level),.published_bank(spectrum_published_bank),
    .levels_ready(spectrum_ready),.levels_valid(spectrum_update),
    .overrun(spectrum_overrun),.overflow(spectrum_overflow));
// One 128x6 RAM stores the smoothed height for every measured frequency.
// Attack is immediate and release decays one unit per video frame.
reg [11:0] height_memory[0:127]; /* fehdl force_ram=1, ram_style="bram" */
reg [11:0] height_q;
wire [10:0] spectrum_x=x-(SPECTRUM_X_POS+16);
wire [6:0] spectrum_column=spectrum_x[7:1];
wire [5:0] old_height=spectrum_scan_first ? 6'd0 : height_q[5:0];
wire [5:0] next_height=spectrum_read_level>=old_height ? spectrum_read_level :
                       old_height==0 ? 6'd0 : old_height-1'b1;
wire blank_update=spectrum_scan_busy && !de_in && !de_last && !clear_track;
function [5:0] display_height;
    input [5:0] value;
    reg [8:0] scaled;
    begin scaled={2'd0,value}+{1'b0,value,2'b0}+9'd7;display_height=scaled[8:3];end
endfunction
wire blank_read=spectrum_scan_busy && !de_in && !de_last;
wire [6:0] height_read_address=blank_read ? spectrum_scan : spectrum_column;
always @(posedge clk) begin
    height_q<=height_memory[height_read_address];
    if(!rst && blank_update && spectrum_scan_phase)
        height_memory[spectrum_scan]<={display_height(next_height),next_height};
end
always @(posedge clk or posedge rst) begin
    if(rst) begin
        spectrum_read_bank<=0;spectrum_scan<=0;spectrum_scan_busy<=0;
        spectrum_scan_phase<=0;spectrum_scan_first<=1;spectrum_filled_valid<=0;
    end else if(clear_track) begin
        spectrum_read_bank<=0;spectrum_scan<=0;spectrum_scan_busy<=0;
        spectrum_scan_phase<=0;spectrum_scan_first<=1;spectrum_filled_valid<=0;
    end else if(vs_in && !vs_last) begin
        spectrum_read_bank<=spectrum_published_bank;spectrum_scan<=0;
        spectrum_scan_busy<=spectrum_ready;spectrum_scan_phase<=0;
        spectrum_scan_first<=!spectrum_filled_valid;
    end else if(blank_update) begin
        if(!spectrum_scan_phase) spectrum_scan_phase<=1;
        else begin
            spectrum_scan_phase<=0;
            if(spectrum_scan==127) begin
                spectrum_scan_busy<=0;spectrum_filled_valid<=1;
            end else spectrum_scan<=spectrum_scan+1'b1;
        end
    end
end
wire finish_bin=sample_valid && bin_samples==BIN_SAMPLES-1;
wire publish=vs_in && !vs_last && pending_valid && !clear_track;
wire active=frame_valid && visible_page!=0;
// Match the four-row OSD height: 64px panel, 48px plot plus native title.
wire wave_panel=active && x>=X_POS && x<X_POS+288 && y>=Y_POS && y<Y_POS+64;
wire spectrum_panel=active && x>=SPECTRUM_X_POS && x<SPECTRUM_X_POS+288 && y>=Y_POS && y<Y_POS+64;
wire panel=wave_panel || spectrum_panel;
wire [10:0] plot_x=x-(X_POS+16),plot_y=y-(Y_POS+12);
wire plot=wave_panel && x>=X_POS+16 && x<X_POS+272 && y>=Y_POS+12 && y<Y_POS+60;
wire spectrum_plot=spectrum_panel && x>=SPECTRUM_X_POS+16 && x<SPECTRUM_X_POS+272 &&
                   y>=Y_POS+12 && y<Y_POS+52;
wire [6:0] history_column=plot_x[7:1];
wire [6:0] read_address=write_pointer+history_column;
// The pointer denotes the oldest slot in a full history. Before it fills,
// mask the unused leading columns; the latest sample is still on the right.
// No reset/clear loop on RAM. Its 128x14 logical capacity is unchanged.
always @(posedge clk) begin
    if(!rst && publish) peaks[write_pointer]<=pending_peaks;
    peaks_q<=peaks[plot ? read_address : 7'd0];
end
always @(posedge clk or posedge rst) begin
    if(rst) begin
        x<=0;y<=0;de_last<=0;vs_last<=0;page_seen<=0;epoch_seen<=0;
        bin_samples<=0;bin_min<=0;bin_max<=0;
        write_pointer<=0;display_count<=0;first_visible<=128;
        pending_peaks<=0;pending_valid<=0;
    end else begin
        de_last<=de_in;vs_last<=vs_in;
        if(de_in) x<=x+1'b1;else x<=0;
        if(de_last && !de_in) y<=y+1'b1;
        if(vs_in && !vs_last) begin x<=0;y<=0;end
        page_seen<=visible_page;epoch_seen<=narration_epoch;
        if(clear_track) begin
            bin_samples<=0;bin_min<=0;bin_max<=0;
            write_pointer<=0;display_count<=0;first_visible<=128;
            pending_peaks<=0;pending_valid<=0;
        end else begin
            if(publish) begin
                write_pointer<=write_pointer+1'b1;pending_valid<=0;
                if(display_count<128) begin
                    display_count<=display_count+1'b1;
                    first_visible<=first_visible-1'b1;
                end
            end
            if(sample_valid) begin
                bin_min<=next_min;bin_max<=next_max;
                if(finish_bin) begin
                    bin_samples<=0;pending_peaks<={next_max,next_min};pending_valid<=1;
                end else bin_samples<=bin_samples+1'b1;
            end
            // A simultaneous publication consumes the previous pending bin;
            // finish_bin above retains the new one for the following VS.
            // If video VS stops, keep the latest completed bin rather than
            // stalling audio or accumulating an unbounded visual queue.
        end
    end
end
// RAM-read stage and composition stage: all video markers receive the same
// two-clock delay. Spectrum smoothing runs separately in blanking.
reg hs_s0,vs_s0,de_s0,panel_s0,plot_s0,border_s0,grid_s0,valid_s0;
reg [23:0] rgb_s0;
reg [5:0] plot_row_s0;
reg stroke_s0;
reg spectrum_plot_s0,spectrum_valid_s0;
// Titles retain the same 2048x1 native ROM. Frequency labels occupy only
// the lower left/right corners, never the title or plotted line.
wire [10:0] label_x=spectrum_panel ? spectrum_x : plot_x;
wire [10:0] title_y=y-(Y_POS+3);
wire [10:0] axis_y=y-(Y_POS+54);
wire axis_left=spectrum_panel && spectrum_x<32 && axis_y<7;
wire axis_right=spectrum_panel && spectrum_x>=216 && spectrum_x<256 && axis_y<7;
wire [10:0] axis_x=axis_right ? spectrum_x-216 : spectrum_x;
reg [4:0] label_id;
always @* begin
 label_id=31;
 if(axis_left) case(axis_x[4:3])
  0:label_id=17;1:label_id=18;2:label_id=13;3:label_id=19; // 94HZ
 endcase
 else if(axis_right) case(axis_x[5:3])
  0:label_id=20;1:label_id=21;2:label_id=22;3:label_id=13;4:label_id=19;default:; // 20KHZ
 endcase
 else if(spectrum_panel) case(label_x[7:3])
  0:label_id=4;1:label_id=5;2:label_id=3;3:label_id=6;
  4:label_id=7;5:label_id=8;6:label_id=9;7:label_id=10;
  9:label_id=24;10:label_id=20;11:label_id=25;12:label_id=5;13:label_id=7;default:; // SPECTRUM 128PT
 endcase else case(label_x[7:3])
  0:label_id=0;1:label_id=1;2:label_id=2;3:label_id=3;default:;
 endcase
end
wire axis_label=axis_left || axis_right;
wire label_area=(axis_label && axis_x[2:0]<5) ||
                (panel && label_x<256 && title_y<7 && label_x[2:0]<5);
wire [2:0] label_row=axis_label ? axis_y[2:0] : title_y[2:0];
wire [2:0] label_col=axis_label ? axis_x[2:0] : label_x[2:0];
wire [10:0] label_address=label_area ? {label_id,label_row,label_col} : 11'd0;
wire ink_s0;
reg label_area_s0;
album_visual_label_rom label_rom(.clk(clk),.address(label_address),.pixel(ink_s0));
// Fixed display gain x2, with saturation, preserves relative loudness.
// Normally seven-bit peaks would shift right once to fit the 64px plot;
// retaining their value and clamping to six bits doubles that visual gain.
// It never changes PCM or renormalizes a quiet word to look like a loud one.
function [5:0] display_peak;
    input signed [6:0] value;
    begin
        if(value>7'sd31) display_peak=6'd31;
        else if(value< -7'sd32) display_peak=6'b100000;
        else display_peak=value[5:0];
    end
endfunction
function [5:0] compact_row;
    input [5:0] value;
    reg [7:0] scaled;
    begin scaled={2'd0,value}+{1'b0,value,1'b0};compact_row=scaled[7:2];end
endfunction
wire [5:0] peak_top=compact_row(6'd31-display_peak(peaks_q[13:7]));
wire [5:0] peak_bottom=compact_row(6'd31-display_peak(peaks_q[6:0]));
wire waveform=plot_s0 && valid_s0 && stroke_s0 &&
              plot_row_s0>=peak_top && plot_row_s0<=peak_bottom;
// Filled analyzer: each measured frequency owns two columns and fills
// continuously from the baseline to its smoothed magnitude. The connecting
// top edge is highlighted; interpolation does not add fake frequency bins.
wire [5:0] filled_height=height_q[11:6];
wire spectrum_fill=spectrum_plot_s0 && spectrum_valid_s0 && filled_height!=0 &&
                   {1'b0,plot_row_s0}>=7'd40-{1'b0,filled_height};
wire spectrum_top=spectrum_plot_s0 && spectrum_valid_s0 && filled_height!=0 &&
                  {1'b0,plot_row_s0}==7'd40-{1'b0,filled_height};
wire spectrum_baseline=spectrum_plot_s0 && spectrum_valid_s0 && plot_row_s0==6'd39;
function [7:0] paper_channel;
    input [7:0] background,paper;
    reg [8:0] sum;
    begin sum={1'b0,background}+{1'b0,paper};paper_channel=sum[8:1];end
endfunction
wire [23:0] paper_rgb={paper_channel(rgb_s0[23:16],8'hf1),
                      paper_channel(rgb_s0[15:8],8'he5),paper_channel(rgb_s0[7:0],8'hcb)};
always @(posedge clk or posedge rst) begin
    if(rst) begin
        hs_s0<=0;vs_s0<=0;de_s0<=0;rgb_s0<=0;
        panel_s0<=0;plot_s0<=0;border_s0<=0;grid_s0<=0;label_area_s0<=0;
        valid_s0<=0;plot_row_s0<=0;stroke_s0<=0;
        spectrum_plot_s0<=0;spectrum_valid_s0<=0;
        hs_out<=0;vs_out<=0;de_out<=0;rgb_out<=0;
    end else begin
        hs_s0<=hs_in;vs_s0<=vs_in;de_s0<=de_in;rgb_s0<=rgb_in;
        panel_s0<=panel;plot_s0<=plot;
        valid_s0<={1'b0,history_column}>=first_visible && !clear_track;
        plot_row_s0<=plot_y[5:0];stroke_s0<=!plot_x[0];
        border_s0<=panel && (x==X_POS || x==X_POS+287 || x==SPECTRUM_X_POS ||
                            x==SPECTRUM_X_POS+287 || y==Y_POS || y==Y_POS+63);
        grid_s0<=(plot && (plot_y==23 || plot_x[5:0]==0)) ||
                 (spectrum_plot && (plot_y==9 || plot_y==19 || plot_y==29 || plot_y==39));
        spectrum_plot_s0<=spectrum_plot;
        spectrum_valid_s0<=spectrum_filled_valid && !clear_track;
        label_area_s0<=label_area;
        hs_out<=hs_s0;vs_out<=vs_s0;de_out<=de_s0;
        if(!de_s0) rgb_out<=rgb_s0;
        else if(border_s0) rgb_out<=24'ha18858;
        else if(label_area_s0 && ink_s0) rgb_out<=24'h382c22;
        else if(waveform) rgb_out<=24'h287c70;
        else if(grid_s0) rgb_out<=24'hc6b992;
        else if(spectrum_top) rgb_out<=24'h76a7ff;
        else if(spectrum_fill) rgb_out<=24'h3f6fd8;
        else if(spectrum_baseline) rgb_out<=24'h587bd8;
        else if(panel_s0) rgb_out<=paper_rgb;
        else rgb_out<=rgb_s0;
    end
end
endmodule

// Native title bitmaps: 32 cells x 8 rows x 8 columns, unused cells blank.
module album_visual_label_rom(input clk,input [10:0] address,output reg pixel);
reg memory[0:2047]; /* fehdl force_ram=1, ram_style="bram" */
integer i;
initial begin
 for(i=0;i<2048;i=i+1) memory[i]=0;
 memory[0]=1'b1;
 memory[4]=1'b1;
 memory[8]=1'b1;
 memory[12]=1'b1;
 memory[16]=1'b1;
 memory[20]=1'b1;
 memory[24]=1'b1;
 memory[26]=1'b1;
 memory[28]=1'b1;
 memory[32]=1'b1;
 memory[34]=1'b1;
 memory[36]=1'b1;
 memory[40]=1'b1;
 memory[42]=1'b1;
 memory[44]=1'b1;
 memory[49]=1'b1;
 memory[51]=1'b1;
 memory[65]=1'b1;
 memory[66]=1'b1;
 memory[67]=1'b1;
 memory[72]=1'b1;
 memory[76]=1'b1;
 memory[80]=1'b1;
 memory[84]=1'b1;
 memory[88]=1'b1;
 memory[89]=1'b1;
 memory[90]=1'b1;
 memory[91]=1'b1;
 memory[92]=1'b1;
 memory[96]=1'b1;
 memory[100]=1'b1;
 memory[104]=1'b1;
 memory[108]=1'b1;
 memory[112]=1'b1;
 memory[116]=1'b1;
 memory[128]=1'b1;
 memory[132]=1'b1;
 memory[136]=1'b1;
 memory[140]=1'b1;
 memory[144]=1'b1;
 memory[148]=1'b1;
 memory[152]=1'b1;
 memory[156]=1'b1;
 memory[160]=1'b1;
 memory[164]=1'b1;
 memory[169]=1'b1;
 memory[171]=1'b1;
 memory[178]=1'b1;
 memory[192]=1'b1;
 memory[193]=1'b1;
 memory[194]=1'b1;
 memory[195]=1'b1;
 memory[196]=1'b1;
 memory[200]=1'b1;
 memory[208]=1'b1;
 memory[216]=1'b1;
 memory[217]=1'b1;
 memory[218]=1'b1;
 memory[219]=1'b1;
 memory[224]=1'b1;
 memory[232]=1'b1;
 memory[240]=1'b1;
 memory[241]=1'b1;
 memory[242]=1'b1;
 memory[243]=1'b1;
 memory[244]=1'b1;
 memory[257]=1'b1;
 memory[258]=1'b1;
 memory[259]=1'b1;
 memory[260]=1'b1;
 memory[264]=1'b1;
 memory[272]=1'b1;
 memory[281]=1'b1;
 memory[282]=1'b1;
 memory[283]=1'b1;
 memory[292]=1'b1;
 memory[300]=1'b1;
 memory[304]=1'b1;
 memory[305]=1'b1;
 memory[306]=1'b1;
 memory[307]=1'b1;
 memory[320]=1'b1;
 memory[321]=1'b1;
 memory[322]=1'b1;
 memory[323]=1'b1;
 memory[328]=1'b1;
 memory[332]=1'b1;
 memory[336]=1'b1;
 memory[340]=1'b1;
 memory[344]=1'b1;
 memory[345]=1'b1;
 memory[346]=1'b1;
 memory[347]=1'b1;
 memory[352]=1'b1;
 memory[360]=1'b1;
 memory[368]=1'b1;
 memory[385]=1'b1;
 memory[386]=1'b1;
 memory[387]=1'b1;
 memory[392]=1'b1;
 memory[396]=1'b1;
 memory[400]=1'b1;
 memory[408]=1'b1;
 memory[416]=1'b1;
 memory[424]=1'b1;
 memory[428]=1'b1;
 memory[433]=1'b1;
 memory[434]=1'b1;
 memory[435]=1'b1;
 memory[448]=1'b1;
 memory[449]=1'b1;
 memory[450]=1'b1;
 memory[451]=1'b1;
 memory[452]=1'b1;
 memory[458]=1'b1;
 memory[466]=1'b1;
 memory[474]=1'b1;
 memory[482]=1'b1;
 memory[490]=1'b1;
 memory[498]=1'b1;
 memory[512]=1'b1;
 memory[513]=1'b1;
 memory[514]=1'b1;
 memory[515]=1'b1;
 memory[520]=1'b1;
 memory[524]=1'b1;
 memory[528]=1'b1;
 memory[532]=1'b1;
 memory[536]=1'b1;
 memory[537]=1'b1;
 memory[538]=1'b1;
 memory[539]=1'b1;
 memory[544]=1'b1;
 memory[546]=1'b1;
 memory[552]=1'b1;
 memory[555]=1'b1;
 memory[560]=1'b1;
 memory[564]=1'b1;
 memory[576]=1'b1;
 memory[580]=1'b1;
 memory[584]=1'b1;
 memory[588]=1'b1;
 memory[592]=1'b1;
 memory[596]=1'b1;
 memory[600]=1'b1;
 memory[604]=1'b1;
 memory[608]=1'b1;
 memory[612]=1'b1;
 memory[616]=1'b1;
 memory[620]=1'b1;
 memory[625]=1'b1;
 memory[626]=1'b1;
 memory[627]=1'b1;
 memory[640]=1'b1;
 memory[644]=1'b1;
 memory[648]=1'b1;
 memory[649]=1'b1;
 memory[651]=1'b1;
 memory[652]=1'b1;
 memory[656]=1'b1;
 memory[658]=1'b1;
 memory[660]=1'b1;
 memory[664]=1'b1;
 memory[666]=1'b1;
 memory[668]=1'b1;
 memory[672]=1'b1;
 memory[676]=1'b1;
 memory[680]=1'b1;
 memory[684]=1'b1;
 memory[688]=1'b1;
 memory[692]=1'b1;
 memory[704]=1'b1;
 memory[712]=1'b1;
 memory[720]=1'b1;
 memory[728]=1'b1;
 memory[736]=1'b1;
 memory[744]=1'b1;
 memory[752]=1'b1;
 memory[753]=1'b1;
 memory[754]=1'b1;
 memory[755]=1'b1;
 memory[756]=1'b1;
 memory[769]=1'b1;
 memory[770]=1'b1;
 memory[771]=1'b1;
 memory[776]=1'b1;
 memory[780]=1'b1;
 memory[784]=1'b1;
 memory[788]=1'b1;
 memory[792]=1'b1;
 memory[796]=1'b1;
 memory[800]=1'b1;
 memory[804]=1'b1;
 memory[808]=1'b1;
 memory[812]=1'b1;
 memory[817]=1'b1;
 memory[818]=1'b1;
 memory[819]=1'b1;
 memory[832]=1'b1;
 memory[836]=1'b1;
 memory[840]=1'b1;
 memory[844]=1'b1;
 memory[848]=1'b1;
 memory[852]=1'b1;
 memory[856]=1'b1;
 memory[857]=1'b1;
 memory[858]=1'b1;
 memory[859]=1'b1;
 memory[860]=1'b1;
 memory[864]=1'b1;
 memory[868]=1'b1;
 memory[872]=1'b1;
 memory[876]=1'b1;
 memory[880]=1'b1;
 memory[884]=1'b1;
 memory[897]=1'b1;
 memory[898]=1'b1;
 memory[899]=1'b1;
 memory[906]=1'b1;
 memory[914]=1'b1;
 memory[922]=1'b1;
 memory[930]=1'b1;
 memory[938]=1'b1;
 memory[945]=1'b1;
 memory[946]=1'b1;
 memory[947]=1'b1;
 memory[961]=1'b1;
 memory[962]=1'b1;
 memory[963]=1'b1;
 memory[968]=1'b1;
 memory[972]=1'b1;
 memory[976]=1'b1;
 memory[984]=1'b1;
 memory[986]=1'b1;
 memory[987]=1'b1;
 memory[988]=1'b1;
 memory[992]=1'b1;
 memory[996]=1'b1;
 memory[1000]=1'b1;
 memory[1004]=1'b1;
 memory[1009]=1'b1;
 memory[1010]=1'b1;
 memory[1011]=1'b1;
 memory[1012]=1'b1;
 memory[1024]=1'b1;
 memory[1033]=1'b1;
 memory[1042]=1'b1;
 memory[1051]=1'b1;
 memory[1058]=1'b1;
 memory[1065]=1'b1;
 memory[1072]=1'b1;
 // Frequency range header: native 5x7 glyphs 9,4,Z,2,0,K.
 memory[1089]=1'b1;
 memory[1090]=1'b1;
 memory[1091]=1'b1;
 memory[1096]=1'b1;
 memory[1100]=1'b1;
 memory[1104]=1'b1;
 memory[1108]=1'b1;
 memory[1113]=1'b1;
 memory[1114]=1'b1;
 memory[1115]=1'b1;
 memory[1116]=1'b1;
 memory[1124]=1'b1;
 memory[1132]=1'b1;
 memory[1137]=1'b1;
 memory[1138]=1'b1;
 memory[1139]=1'b1;
 memory[1155]=1'b1;
 memory[1162]=1'b1;
 memory[1163]=1'b1;
 memory[1169]=1'b1;
 memory[1171]=1'b1;
 memory[1176]=1'b1;
 memory[1179]=1'b1;
 memory[1184]=1'b1;
 memory[1185]=1'b1;
 memory[1186]=1'b1;
 memory[1187]=1'b1;
 memory[1188]=1'b1;
 memory[1195]=1'b1;
 memory[1203]=1'b1;
 memory[1216]=1'b1;
 memory[1217]=1'b1;
 memory[1218]=1'b1;
 memory[1219]=1'b1;
 memory[1220]=1'b1;
 memory[1228]=1'b1;
 memory[1235]=1'b1;
 memory[1242]=1'b1;
 memory[1249]=1'b1;
 memory[1256]=1'b1;
 memory[1264]=1'b1;
 memory[1265]=1'b1;
 memory[1266]=1'b1;
 memory[1267]=1'b1;
 memory[1268]=1'b1;
 memory[1281]=1'b1;
 memory[1282]=1'b1;
 memory[1283]=1'b1;
 memory[1288]=1'b1;
 memory[1292]=1'b1;
 memory[1300]=1'b1;
 memory[1307]=1'b1;
 memory[1314]=1'b1;
 memory[1321]=1'b1;
 memory[1328]=1'b1;
 memory[1329]=1'b1;
 memory[1330]=1'b1;
 memory[1331]=1'b1;
 memory[1332]=1'b1;
 memory[1345]=1'b1;
 memory[1346]=1'b1;
 memory[1347]=1'b1;
 memory[1352]=1'b1;
 memory[1356]=1'b1;
 memory[1360]=1'b1;
 memory[1363]=1'b1;
 memory[1364]=1'b1;
 memory[1368]=1'b1;
 memory[1370]=1'b1;
 memory[1372]=1'b1;
 memory[1376]=1'b1;
 memory[1377]=1'b1;
 memory[1380]=1'b1;
 memory[1384]=1'b1;
 memory[1388]=1'b1;
 memory[1393]=1'b1;
 memory[1394]=1'b1;
 memory[1395]=1'b1;
 memory[1408]=1'b1;
 memory[1412]=1'b1;
 memory[1416]=1'b1;
 memory[1419]=1'b1;
 memory[1424]=1'b1;
 memory[1426]=1'b1;
 memory[1432]=1'b1;
 memory[1433]=1'b1;
 memory[1440]=1'b1;
 memory[1442]=1'b1;
 memory[1448]=1'b1;
 memory[1451]=1'b1;
 memory[1456]=1'b1;
 memory[1460]=1'b1;
 // Native digit 6 for the 64PT title.
 memory[1473]=1'b1;
 memory[1474]=1'b1;
 memory[1475]=1'b1;
 memory[1480]=1'b1;
 memory[1488]=1'b1;
 memory[1496]=1'b1;
 memory[1497]=1'b1;
 memory[1498]=1'b1;
 memory[1499]=1'b1;
 memory[1504]=1'b1;
 memory[1508]=1'b1;
 memory[1512]=1'b1;
 memory[1516]=1'b1;
 memory[1521]=1'b1;
 memory[1522]=1'b1;
 memory[1523]=1'b1;
 // Native 1 and 8 for the 128PT title.
 memory[1538]=1'b1;
 memory[1545]=1'b1;
 memory[1546]=1'b1;
 memory[1554]=1'b1;
 memory[1562]=1'b1;
 memory[1570]=1'b1;
 memory[1578]=1'b1;
 memory[1585]=1'b1;
 memory[1586]=1'b1;
 memory[1587]=1'b1;
 memory[1601]=1'b1;
 memory[1602]=1'b1;
 memory[1603]=1'b1;
 memory[1608]=1'b1;
 memory[1612]=1'b1;
 memory[1616]=1'b1;
 memory[1620]=1'b1;
 memory[1625]=1'b1;
 memory[1626]=1'b1;
 memory[1627]=1'b1;
 memory[1632]=1'b1;
 memory[1636]=1'b1;
 memory[1640]=1'b1;
 memory[1644]=1'b1;
 memory[1649]=1'b1;
 memory[1650]=1'b1;
 memory[1651]=1'b1;
end
always @(posedge clk) pixel<=memory[address];
endmodule

// Selected DFT coefficients using a serial Goertzel recurrence. The 128
// nonuniform frequencies span 93.75..19968.75 Hz at Fs=48 kHz, N=512.
// Q14 constants; signed 10-bit input; one shared pipelined 26x16 multiply.
// Reference: ST AN5647, real=cos(w)*s1-s2, imag=sin(w)*s1.
module album_pcm_spectrum(
    input clk,rst,clear,sample_valid,input signed [15:0] sample_data,
    input [1:0] read_bank,input [6:0] read_address,input read_lock,
    output reg [5:0] read_level,output reg [1:0] published_bank,
    output reg levels_ready,levels_valid,
    output reg overrun,overflow
);
localparam [3:0] IDLE=0,LOAD=1,WAIT_MUL=2,ROUND_WAIT=3,UPDATE=4,
    COS_LOAD=5,COS_WAIT=6,COS_ROUND=7,COS_SAVE=8,SIN_LOAD=9,SIN_WAIT=10,
    SIN_ROUND=11,SIN_SAVE=12,MAG_ADD=13,LOG_SAVE=14;
reg [3:0] state;
reg [6:0] band;
reg [8:0] sample_count;
reg first_sample,last_sample;
reg signed [9:0] input_sample;
// Narrow synchronous memories use spare 9K blocks, avoiding distributed read muxes.
reg [17:0] state_lo[0:127]; /* fehdl force_ram=1, ram_style="bram" */
reg [17:0] state_mid[0:127]; /* fehdl force_ram=1, ram_style="bram" */
reg [17:0] state_hi[0:127]; /* fehdl force_ram=1, ram_style="bram" */
reg [17:0] state_lo_q,state_mid_q,state_hi_q;
wire [53:0] state_word={state_hi_q,state_mid_q,state_lo_q};
wire signed [25:0] state_value1=state_word[25:0],state_value2=state_word[51:26];
wire advance_band=(state==UPDATE || state==LOG_SAVE) && band!=127;
wire [6:0] state_read_address=advance_band ? band+1'b1 :
    state==IDLE || (state==UPDATE && band==127) ? 7'd0 : band;
wire signed [27:0] recurrence;
wire [53:0] state_write_word={2'd0,(first_sample ? 26'd0 : state_value1),recurrence[25:0]};
always @(posedge clk) begin
    state_lo_q<=state_lo[state_read_address];state_mid_q<=state_mid[state_read_address];
    state_hi_q<=state_hi[state_read_address];
    if(!rst && !clear && state==UPDATE) begin
        state_lo[band]<=state_write_word[17:0];state_mid[band]<=state_write_word[35:18];
        state_hi[band]<=state_write_word[53:36];
    end
end
reg signed [25:0] mul_value,real_part;
reg signed [15:0] mul_coefficient;
reg signed [41:0] product;
reg signed [27:0] feedback;
reg signed [27:0] drive;
reg [25:0] mag_high,mag_low;
reg [26:0] magnitude;
// Three narrow coefficient ROMs share the existing band prefetch.
// Keeping constants in RAM avoids a large 64-way combinational LUT mux.
reg [15:0] coefficient_feedback[0:127]; /* fehdl force_ram=1, ram_style="bram" */
reg [15:0] coefficient_cos[0:127]; /* fehdl force_ram=1, ram_style="bram" */
reg [15:0] coefficient_sin[0:127]; /* fehdl force_ram=1, ram_style="bram" */
reg signed [15:0] feedback_q,cos_q,sin_q;
initial begin
 coefficient_feedback[0]=16'h7ffe;
 coefficient_cos[0]=16'h3fff;
 coefficient_sin[0]=16'h00c9;
 coefficient_feedback[1]=16'h7ff6;
 coefficient_cos[1]=16'h3ffb;
 coefficient_sin[1]=16'h0192;
 coefficient_feedback[2]=16'h7fea;
 coefficient_cos[2]=16'h3ff5;
 coefficient_sin[2]=16'h025b;
 coefficient_feedback[3]=16'h7fd9;
 coefficient_cos[3]=16'h3fec;
 coefficient_sin[3]=16'h0324;
 coefficient_feedback[4]=16'h7fc2;
 coefficient_cos[4]=16'h3fe1;
 coefficient_sin[4]=16'h03ed;
 coefficient_feedback[5]=16'h7fa7;
 coefficient_cos[5]=16'h3fd4;
 coefficient_sin[5]=16'h04b5;
 coefficient_feedback[6]=16'h7f87;
 coefficient_cos[6]=16'h3fc4;
 coefficient_sin[6]=16'h057e;
 coefficient_feedback[7]=16'h7f62;
 coefficient_cos[7]=16'h3fb1;
 coefficient_sin[7]=16'h0646;
 coefficient_feedback[8]=16'h7f38;
 coefficient_cos[8]=16'h3f9c;
 coefficient_sin[8]=16'h070e;
 coefficient_feedback[9]=16'h7f0a;
 coefficient_cos[9]=16'h3f85;
 coefficient_sin[9]=16'h07d6;
 coefficient_feedback[10]=16'h7ed6;
 coefficient_cos[10]=16'h3f6b;
 coefficient_sin[10]=16'h089d;
 coefficient_feedback[11]=16'h7e9d;
 coefficient_cos[11]=16'h3f4f;
 coefficient_sin[11]=16'h0964;
 coefficient_feedback[12]=16'h7e60;
 coefficient_cos[12]=16'h3f30;
 coefficient_sin[12]=16'h0a2b;
 coefficient_feedback[13]=16'h7e1e;
 coefficient_cos[13]=16'h3f0f;
 coefficient_sin[13]=16'h0af1;
 coefficient_feedback[14]=16'h7dd6;
 coefficient_cos[14]=16'h3eeb;
 coefficient_sin[14]=16'h0bb7;
 coefficient_feedback[15]=16'h7d8a;
 coefficient_cos[15]=16'h3ec5;
 coefficient_sin[15]=16'h0c7c;
 coefficient_feedback[16]=16'h7d3a;
 coefficient_cos[16]=16'h3e9d;
 coefficient_sin[16]=16'h0d41;
 coefficient_feedback[17]=16'h7ce4;
 coefficient_cos[17]=16'h3e72;
 coefficient_sin[17]=16'h0e06;
 coefficient_feedback[18]=16'h7c89;
 coefficient_cos[18]=16'h3e45;
 coefficient_sin[18]=16'h0eca;
 coefficient_feedback[19]=16'h7c2a;
 coefficient_cos[19]=16'h3e15;
 coefficient_sin[19]=16'h0f8d;
 coefficient_feedback[20]=16'h7bc6;
 coefficient_cos[20]=16'h3de3;
 coefficient_sin[20]=16'h1050;
 coefficient_feedback[21]=16'h7b5d;
 coefficient_cos[21]=16'h3daf;
 coefficient_sin[21]=16'h1112;
 coefficient_feedback[22]=16'h7aef;
 coefficient_cos[22]=16'h3d78;
 coefficient_sin[22]=16'h11d3;
 coefficient_feedback[23]=16'h7a7d;
 coefficient_cos[23]=16'h3d3f;
 coefficient_sin[23]=16'h1294;
 coefficient_feedback[24]=16'h7a06;
 coefficient_cos[24]=16'h3d03;
 coefficient_sin[24]=16'h1354;
 coefficient_feedback[25]=16'h798a;
 coefficient_cos[25]=16'h3cc5;
 coefficient_sin[25]=16'h1413;
 coefficient_feedback[26]=16'h790a;
 coefficient_cos[26]=16'h3c85;
 coefficient_sin[26]=16'h14d2;
 coefficient_feedback[27]=16'h7885;
 coefficient_cos[27]=16'h3c42;
 coefficient_sin[27]=16'h1590;
 coefficient_feedback[28]=16'h77fb;
 coefficient_cos[28]=16'h3bfd;
 coefficient_sin[28]=16'h164c;
 coefficient_feedback[29]=16'h776c;
 coefficient_cos[29]=16'h3bb6;
 coefficient_sin[29]=16'h1709;
 coefficient_feedback[30]=16'h76d9;
 coefficient_cos[30]=16'h3b6d;
 coefficient_sin[30]=16'h17c4;
 coefficient_feedback[31]=16'h7642;
 coefficient_cos[31]=16'h3b21;
 coefficient_sin[31]=16'h187e;
 coefficient_feedback[32]=16'h75a6;
 coefficient_cos[32]=16'h3ad3;
 coefficient_sin[32]=16'h1937;
 coefficient_feedback[33]=16'h7505;
 coefficient_cos[33]=16'h3a82;
 coefficient_sin[33]=16'h19ef;
 coefficient_feedback[34]=16'h7460;
 coefficient_cos[34]=16'h3a30;
 coefficient_sin[34]=16'h1aa7;
 coefficient_feedback[35]=16'h73b6;
 coefficient_cos[35]=16'h39db;
 coefficient_sin[35]=16'h1b5d;
 coefficient_feedback[36]=16'h7308;
 coefficient_cos[36]=16'h3984;
 coefficient_sin[36]=16'h1c12;
 coefficient_feedback[37]=16'h7255;
 coefficient_cos[37]=16'h392b;
 coefficient_sin[37]=16'h1cc6;
 coefficient_feedback[38]=16'h719e;
 coefficient_cos[38]=16'h38cf;
 coefficient_sin[38]=16'h1d79;
 coefficient_feedback[39]=16'h70e3;
 coefficient_cos[39]=16'h3871;
 coefficient_sin[39]=16'h1e2b;
 coefficient_feedback[40]=16'h7023;
 coefficient_cos[40]=16'h3812;
 coefficient_sin[40]=16'h1edc;
 coefficient_feedback[41]=16'h6f5f;
 coefficient_cos[41]=16'h37b0;
 coefficient_sin[41]=16'h1f8c;
 coefficient_feedback[42]=16'h6e97;
 coefficient_cos[42]=16'h374b;
 coefficient_sin[42]=16'h203a;
 coefficient_feedback[43]=16'h6dca;
 coefficient_cos[43]=16'h36e5;
 coefficient_sin[43]=16'h20e7;
 coefficient_feedback[44]=16'h6cf9;
 coefficient_cos[44]=16'h367d;
 coefficient_sin[44]=16'h2193;
 coefficient_feedback[45]=16'h6c24;
 coefficient_cos[45]=16'h3612;
 coefficient_sin[45]=16'h223d;
 coefficient_feedback[46]=16'h6b4b;
 coefficient_cos[46]=16'h35a5;
 coefficient_sin[46]=16'h22e7;
 coefficient_feedback[47]=16'h6a6e;
 coefficient_cos[47]=16'h3537;
 coefficient_sin[47]=16'h238e;
 coefficient_feedback[48]=16'h698c;
 coefficient_cos[48]=16'h34c6;
 coefficient_sin[48]=16'h2435;
 coefficient_feedback[49]=16'h68a7;
 coefficient_cos[49]=16'h3453;
 coefficient_sin[49]=16'h24da;
 coefficient_feedback[50]=16'h67bd;
 coefficient_cos[50]=16'h33df;
 coefficient_sin[50]=16'h257e;
 coefficient_feedback[51]=16'h66d0;
 coefficient_cos[51]=16'h3368;
 coefficient_sin[51]=16'h2620;
 coefficient_feedback[52]=16'h65de;
 coefficient_cos[52]=16'h32ef;
 coefficient_sin[52]=16'h26c1;
 coefficient_feedback[53]=16'h64e9;
 coefficient_cos[53]=16'h3274;
 coefficient_sin[53]=16'h2760;
 coefficient_feedback[54]=16'h63ef;
 coefficient_cos[54]=16'h31f8;
 coefficient_sin[54]=16'h27fe;
 coefficient_feedback[55]=16'h62f2;
 coefficient_cos[55]=16'h3179;
 coefficient_sin[55]=16'h289a;
 coefficient_feedback[56]=16'h61f1;
 coefficient_cos[56]=16'h30f9;
 coefficient_sin[56]=16'h2935;
 coefficient_feedback[57]=16'h60ec;
 coefficient_cos[57]=16'h3076;
 coefficient_sin[57]=16'h29ce;
 coefficient_feedback[58]=16'h5fe4;
 coefficient_cos[58]=16'h2ff2;
 coefficient_sin[58]=16'h2a65;
 coefficient_feedback[59]=16'h5ed7;
 coefficient_cos[59]=16'h2f6c;
 coefficient_sin[59]=16'h2afb;
 coefficient_feedback[60]=16'h5dc8;
 coefficient_cos[60]=16'h2ee4;
 coefficient_sin[60]=16'h2b8f;
 coefficient_feedback[61]=16'h5cb4;
 coefficient_cos[61]=16'h2e5a;
 coefficient_sin[61]=16'h2c21;
 coefficient_feedback[62]=16'h5b9d;
 coefficient_cos[62]=16'h2dcf;
 coefficient_sin[62]=16'h2cb2;
 coefficient_feedback[63]=16'h5a82;
 coefficient_cos[63]=16'h2d41;
 coefficient_sin[63]=16'h2d41;
 coefficient_feedback[64]=16'h5964;
 coefficient_cos[64]=16'h2cb2;
 coefficient_sin[64]=16'h2dcf;
 coefficient_feedback[65]=16'h5843;
 coefficient_cos[65]=16'h2c21;
 coefficient_sin[65]=16'h2e5a;
 coefficient_feedback[66]=16'h571e;
 coefficient_cos[66]=16'h2b8f;
 coefficient_sin[66]=16'h2ee4;
 coefficient_feedback[67]=16'h55f6;
 coefficient_cos[67]=16'h2afb;
 coefficient_sin[67]=16'h2f6c;
 coefficient_feedback[68]=16'h54ca;
 coefficient_cos[68]=16'h2a65;
 coefficient_sin[68]=16'h2ff2;
 coefficient_feedback[69]=16'h539b;
 coefficient_cos[69]=16'h29ce;
 coefficient_sin[69]=16'h3076;
 coefficient_feedback[70]=16'h5269;
 coefficient_cos[70]=16'h2935;
 coefficient_sin[70]=16'h30f9;
 coefficient_feedback[71]=16'h5134;
 coefficient_cos[71]=16'h289a;
 coefficient_sin[71]=16'h3179;
 coefficient_feedback[72]=16'h4ffb;
 coefficient_cos[72]=16'h27fe;
 coefficient_sin[72]=16'h31f8;
 coefficient_feedback[73]=16'h4ec0;
 coefficient_cos[73]=16'h2760;
 coefficient_sin[73]=16'h3274;
 coefficient_feedback[74]=16'h4d81;
 coefficient_cos[74]=16'h26c1;
 coefficient_sin[74]=16'h32ef;
 coefficient_feedback[75]=16'h4afb;
 coefficient_cos[75]=16'h257e;
 coefficient_sin[75]=16'h33df;
 coefficient_feedback[76]=16'h49b4;
 coefficient_cos[76]=16'h24da;
 coefficient_sin[76]=16'h3453;
 coefficient_feedback[77]=16'h471d;
 coefficient_cos[77]=16'h238e;
 coefficient_sin[77]=16'h3537;
 coefficient_feedback[78]=16'h45cd;
 coefficient_cos[78]=16'h22e7;
 coefficient_sin[78]=16'h35a5;
 coefficient_feedback[79]=16'h4326;
 coefficient_cos[79]=16'h2193;
 coefficient_sin[79]=16'h367d;
 coefficient_feedback[80]=16'h4074;
 coefficient_cos[80]=16'h203a;
 coefficient_sin[80]=16'h374b;
 coefficient_feedback[81]=16'h3f17;
 coefficient_cos[81]=16'h1f8c;
 coefficient_sin[81]=16'h37b0;
 coefficient_feedback[82]=16'h3c57;
 coefficient_cos[82]=16'h1e2b;
 coefficient_sin[82]=16'h3871;
 coefficient_feedback[83]=16'h398d;
 coefficient_cos[83]=16'h1cc6;
 coefficient_sin[83]=16'h392b;
 coefficient_feedback[84]=16'h36ba;
 coefficient_cos[84]=16'h1b5d;
 coefficient_sin[84]=16'h39db;
 coefficient_feedback[85]=16'h354e;
 coefficient_cos[85]=16'h1aa7;
 coefficient_sin[85]=16'h3a30;
 coefficient_feedback[86]=16'h326e;
 coefficient_cos[86]=16'h1937;
 coefficient_sin[86]=16'h3ad3;
 coefficient_feedback[87]=16'h2f87;
 coefficient_cos[87]=16'h17c4;
 coefficient_sin[87]=16'h3b6d;
 coefficient_feedback[88]=16'h2c99;
 coefficient_cos[88]=16'h164c;
 coefficient_sin[88]=16'h3bfd;
 coefficient_feedback[89]=16'h29a4;
 coefficient_cos[89]=16'h14d2;
 coefficient_sin[89]=16'h3c85;
 coefficient_feedback[90]=16'h26a8;
 coefficient_cos[90]=16'h1354;
 coefficient_sin[90]=16'h3d03;
 coefficient_feedback[91]=16'h23a7;
 coefficient_cos[91]=16'h11d3;
 coefficient_sin[91]=16'h3d78;
 coefficient_feedback[92]=16'h209f;
 coefficient_cos[92]=16'h1050;
 coefficient_sin[92]=16'h3de3;
 coefficient_feedback[93]=16'h1d93;
 coefficient_cos[93]=16'h0eca;
 coefficient_sin[93]=16'h3e45;
 coefficient_feedback[94]=16'h1a83;
 coefficient_cos[94]=16'h0d41;
 coefficient_sin[94]=16'h3e9d;
 coefficient_feedback[95]=16'h15e2;
 coefficient_cos[95]=16'h0af1;
 coefficient_sin[95]=16'h3f0f;
 coefficient_feedback[96]=16'h12c8;
 coefficient_cos[96]=16'h0964;
 coefficient_sin[96]=16'h3f4f;
 coefficient_feedback[97]=16'h0fab;
 coefficient_cos[97]=16'h07d6;
 coefficient_sin[97]=16'h3f85;
 coefficient_feedback[98]=16'h0afb;
 coefficient_cos[98]=16'h057e;
 coefficient_sin[98]=16'h3fc4;
 coefficient_feedback[99]=16'h07d9;
 coefficient_cos[99]=16'h03ed;
 coefficient_sin[99]=16'h3fe1;
 coefficient_feedback[100]=16'h04b6;
 coefficient_cos[100]=16'h025b;
 coefficient_sin[100]=16'h3ff5;
 coefficient_feedback[101]=16'h0000;
 coefficient_cos[101]=16'h0000;
 coefficient_sin[101]=16'h4000;
 coefficient_feedback[102]=16'hfcdc;
 coefficient_cos[102]=16'hfe6e;
 coefficient_sin[102]=16'h3ffb;
 coefficient_feedback[103]=16'hf827;
 coefficient_cos[103]=16'hfc13;
 coefficient_sin[103]=16'h3fe1;
 coefficient_feedback[104]=16'hf374;
 coefficient_cos[104]=16'hf9ba;
 coefficient_sin[104]=16'h3fb1;
 coefficient_feedback[105]=16'hf055;
 coefficient_cos[105]=16'hf82a;
 coefficient_sin[105]=16'h3f85;
 coefficient_feedback[106]=16'hebab;
 coefficient_cos[106]=16'hf5d5;
 coefficient_sin[106]=16'h3f30;
 coefficient_feedback[107]=16'he707;
 coefficient_cos[107]=16'hf384;
 coefficient_sin[107]=16'h3ec5;
 coefficient_feedback[108]=16'he26d;
 coefficient_cos[108]=16'hf136;
 coefficient_sin[108]=16'h3e45;
 coefficient_feedback[109]=16'hdddc;
 coefficient_cos[109]=16'heeee;
 coefficient_sin[109]=16'h3daf;
 coefficient_feedback[110]=16'hd958;
 coefficient_cos[110]=16'hecac;
 coefficient_sin[110]=16'h3d03;
 coefficient_feedback[111]=16'hd4e1;
 coefficient_cos[111]=16'hea70;
 coefficient_sin[111]=16'h3c42;
 coefficient_feedback[112]=16'hd079;
 coefficient_cos[112]=16'he83c;
 coefficient_sin[112]=16'h3b6d;
 coefficient_feedback[113]=16'hcc21;
 coefficient_cos[113]=16'he611;
 coefficient_sin[113]=16'h3a82;
 coefficient_feedback[114]=16'hc7db;
 coefficient_cos[114]=16'he3ee;
 coefficient_sin[114]=16'h3984;
 coefficient_feedback[115]=16'hc3a9;
 coefficient_cos[115]=16'he1d5;
 coefficient_sin[115]=16'h3871;
 coefficient_feedback[116]=16'hbe32;
 coefficient_cos[116]=16'hdf19;
 coefficient_sin[116]=16'h36e5;
 coefficient_feedback[117]=16'hba33;
 coefficient_cos[117]=16'hdd19;
 coefficient_sin[117]=16'h35a5;
 coefficient_feedback[118]=16'hb505;
 coefficient_cos[118]=16'hda82;
 coefficient_sin[118]=16'h33df;
 coefficient_feedback[119]=16'hb140;
 coefficient_cos[119]=16'hd8a0;
 coefficient_sin[119]=16'h3274;
 coefficient_feedback[120]=16'hac65;
 coefficient_cos[120]=16'hd632;
 coefficient_sin[120]=16'h3076;
 coefficient_feedback[121]=16'ha8e2;
 coefficient_cos[121]=16'hd471;
 coefficient_sin[121]=16'h2ee4;
 coefficient_feedback[122]=16'ha463;
 coefficient_cos[122]=16'hd231;
 coefficient_sin[122]=16'h2cb2;
 coefficient_feedback[123]=16'ha01c;
 coefficient_cos[123]=16'hd00e;
 coefficient_sin[123]=16'h2a65;
 coefficient_feedback[124]=16'h9c11;
 coefficient_cos[124]=16'hce08;
 coefficient_sin[124]=16'h27fe;
 coefficient_feedback[125]=16'h9843;
 coefficient_cos[125]=16'hcc21;
 coefficient_sin[125]=16'h257e;
 coefficient_feedback[126]=16'h94b5;
 coefficient_cos[126]=16'hca5b;
 coefficient_sin[126]=16'h22e7;
 coefficient_feedback[127]=16'h9169;
 coefficient_cos[127]=16'hc8b5;
 coefficient_sin[127]=16'h203a;
end
// Three result banks fit in one 9K RAM. While the display holds one,
// the analyzer can publish two newer coherent results.
reg [5:0] level_memory[0:383]; /* fehdl force_ram=1, ram_style="bram" */
reg [1:0] work_bank;
function [1:0] free_bank;
 input [1:0] published,locked_bank;
 input locked;
 begin
  if(published!=0 && (!locked || locked_bank!=0)) free_bank=0;
  else if(published!=1 && (!locked || locked_bank!=1)) free_bank=1;
  else free_bank=2;
 end
endfunction
always @(posedge clk) begin
 feedback_q<=coefficient_feedback[state_read_address];
 cos_q<=coefficient_cos[state_read_address];sin_q<=coefficient_sin[state_read_address];
 read_level<=level_memory[{read_bank,read_address}];
 if(!rst && !clear && state==LOG_SAVE)
  level_memory[{work_bank,band}]<=log_height(magnitude);
end
wire signed [16:0] rounded_input=$signed({sample_data[15],sample_data})+17'sd32;
wire signed [9:0] quantized_input=rounded_input>17'sd32767 ? 10'sd511 : rounded_input[15:6];
assign recurrence=feedback+drive;
wire signed [27:0] real_result=feedback-{{2{state_value2[25]}},state_value2};
wire [25:0] abs_real=real_part[25] ? -real_part : real_part;
wire [25:0] abs_imag=feedback[25] ? -feedback[25:0] : feedback[25:0];
// A display approximation to complex magnitude, max + min/2. It is not PSD
// calibration or dBFS. Four display steps per octave with a fixed noise floor.
function [5:0] log_height;
    input [26:0] value;
    integer i;
    reg [6:0] h;
    begin
        h=0;
        // Select a constant exponent/mantissa first; add one only ONCE.
        // The former per-branch +1 inferred many independent carry chains.
        for(i=4;i<19;i=i+1)
            if(value[i]) h=((i-4)*4)|{5'd0,value[i-1 -:2]};
        if(value<27'd16) log_height=0;
        else if(value>=27'd524288) log_height=63;
        else log_height=h[5:0]+6'd1;
    end
endfunction
always @(posedge clk) product<=mul_value*mul_coefficient;
// A separate rounding register avoids systematic negative truncation bias
// and keeps the feedback adder off the multiplier/rounding critical path.
always @(posedge clk) feedback<=$signed(product[41:14])+$signed({27'd0,product[13]});
always @(posedge clk or posedge rst) begin
    if(rst) begin
        state<=IDLE;band<=0;sample_count<=0;first_sample<=0;last_sample<=0;
        input_sample<=0;mul_value<=0;mul_coefficient<=0;real_part<=0;drive<=0;
        mag_high<=0;mag_low<=0;magnitude<=0;work_bank<=1;published_bank<=0;levels_ready<=0;
        levels_valid<=0;overrun<=0;overflow<=0;
    end else if(clear) begin
        state<=IDLE;band<=0;sample_count<=0;first_sample<=0;last_sample<=0;
        input_sample<=0;mul_value<=0;mul_coefficient<=0;real_part<=0;drive<=0;
        mag_high<=0;mag_low<=0;magnitude<=0;work_bank<=1;published_bank<=0;levels_ready<=0;
        levels_valid<=0;overrun<=0;overflow<=0;
    end else begin
        levels_valid<=0;
        if(sample_valid && state!=IDLE) overrun<=1;
        case(state)
            IDLE:if(sample_valid) begin
                input_sample<=quantized_input;band<=0;
                first_sample<=sample_count==0;last_sample<=sample_count==511;
                if(sample_count==511) sample_count<=0;
                else sample_count<=sample_count+1'b1;
                state<=LOAD;
            end
            LOAD:begin
                mul_value<=first_sample ? 26'sd0 : state_value1;
                mul_coefficient<=feedback_q;
                drive<=first_sample ? {{18{input_sample[9]}},input_sample} :
                       $signed({{18{input_sample[9]}},input_sample})-
                       $signed({{2{state_value2[25]}},state_value2});
                state<=WAIT_MUL;
            end
            WAIT_MUL:state<=ROUND_WAIT;
            ROUND_WAIT:state<=UPDATE;
            UPDATE:begin
                if(recurrence[27:25]!={3{recurrence[25]}}) overflow<=1;
                if(band==127) begin
                    band<=0;state<=last_sample ? COS_LOAD : IDLE;
                end else begin band<=band+1'b1;state<=LOAD;end
            end
            COS_LOAD:begin
                if(band==0) work_bank<=free_bank(published_bank,read_bank,read_lock);
                mul_value<=state_value1;mul_coefficient<=cos_q;state<=COS_WAIT;
            end
            COS_WAIT:state<=COS_ROUND;
            COS_ROUND:state<=COS_SAVE;
            COS_SAVE:begin real_part<=real_result[25:0];state<=SIN_LOAD;end
            SIN_LOAD:begin
                mul_value<=state_value1;mul_coefficient<=sin_q;state<=SIN_WAIT;
            end
            SIN_WAIT:state<=SIN_ROUND;
            SIN_ROUND:state<=SIN_SAVE;
            SIN_SAVE:begin
                if(abs_real>=abs_imag) begin mag_high<=abs_real;mag_low<=abs_imag;end
                else begin mag_high<=abs_imag;mag_low<=abs_real;end
                state<=MAG_ADD;
            end
            MAG_ADD:begin magnitude<={1'b0,mag_high}+{2'd0,mag_low[25:1]};state<=LOG_SAVE;end
            LOG_SAVE:begin
                if(band==127) begin
                    published_bank<=work_bank;levels_ready<=1;levels_valid<=1;state<=IDLE;
                end else begin band<=band+1'b1;state<=COS_LOAD;end
            end
            default:state<=IDLE;
        endcase
    end
end
endmodule

// BEGIN GENERATED TEACHING ASSETS
// Runtime scene engine. Kept in registered top source by asset generator.
// Two coordinate + six scene + one transition stages; markers follow RGB.
module album_teaching_scene #(
 parameter integer STATUS_X=16,STATUS_Y=16,ENABLE_TRANSITIONS=0
)(
 input clk,rst,hs_in,vs_in,de_in,input [23:0] rgb_in,
 input frame_valid,input [5:0] visible_page,input [23:0] narration,input sidebar_visible,
 input [8:0] transition_command,output reg [1:0] transition_reply,
 output reg hs_out,vs_out,de_out,output reg [23:0] rgb_out
);
reg [10:0] raw_x,raw_y;
reg [10:0] x_pre0,y_pre0,x_pre1,y_pre1;
reg [23:0] rgb_pre0,rgb_pre1;
reg [1:0] hs_pre,vs_pre,de_pre;
wire [10:0] x=x_pre1,y=y_pre1;
wire pixel_de=de_pre[1],pixel_hs=hs_pre[1],pixel_vs=vs_pre[1];
wire [23:0] pixel_rgb=rgb_pre1;
reg de_last,vs_last;
reg [5:0] page_frame;
reg epoch_seen,enabled_frame;
reg [11:0] scroll;
reg [4:0] focus_mix;
reg [5:0] entry_age;
reg teacher_shown;
reg [7:0] frame_count;
wire timeline_match=narration[22:17]==visible_page;
wire [15:0] demo_start,demo_end;
wire [10:0] teacher_x,teacher_y;
wire [3:0] scroll_step;
wire [1:0] focus_lines;
wire [6:0] text_length;
wire highlight_now=enabled_frame && timeline_match && narration[16] && focus_lines!=0 &&
 narration[15:0]>=demo_start && narration[15:0]<demo_end;
wire page_changed=visible_page!=page_frame;
wire new_page=page_changed || (timeline_match && narration[23]!=epoch_seen);
always @(posedge clk or posedge rst) begin
 if(rst) begin
  raw_x<=0;raw_y<=0;de_last<=0;vs_last<=0;page_frame<=0;epoch_seen<=0;enabled_frame<=0;
  scroll<=0;focus_mix<=0;entry_age<=0;teacher_shown<=0;frame_count<=0;
 end else begin
  vs_last<=vs_in;de_last<=de_in;
  if(de_in) raw_x<=raw_x+1'b1; else raw_x<=0;
  if(de_last && !de_in) raw_y<=raw_y+1'b1;
  if(vs_in && !vs_last) begin
   raw_x<=0;raw_y<=0;page_frame<=visible_page;
   enabled_frame<=frame_valid && visible_page>=1 && visible_page<=12;
   if(timeline_match) epoch_seen<=narration[23];
   if(new_page || !frame_valid) begin
    scroll<=0;focus_mix<=0;entry_age<=0;teacher_shown<=0;frame_count<=0;
   end else begin
    frame_count<=frame_count+1'b1;
    if(scroll+{8'd0,scroll_step}>=(text_length*12'd32+12'd1280)) scroll<=0;
    else scroll<=scroll+{8'd0,scroll_step};
    if(highlight_now) begin
     if(focus_mix<16) focus_mix<=focus_mix+1'b1;
    end else begin
     if(focus_mix!=0) focus_mix<=focus_mix-1'b1;
    end
    if(enabled_frame && timeline_match && narration[16] && focus_lines!=0 && narration[15:0]>=demo_start)
     teacher_shown<=1;
    if(teacher_shown && entry_age<32) entry_age<=entry_age+1'b1;
   end
  end
 end
end
// Cover the current image, acknowledge the matching ticket, then uncover the
// committed image. No third frame buffer or simultaneous dual-frame reads.
// Controls change at VS only; cancel restores the still-current picture.
localparam [1:0] FX_IDLE=0,FX_COVER=1,FX_WAIT=2,FX_REVEAL=3;
reg [1:0] transition_state,transition_kind;
reg [4:0] transition_level;
reg transition_seen,transition_token,transition_auto;
reg [2:0] cancel_frames;
wire [4:0] transition_step=transition_auto ? 5'd1 : 5'd2;
function [1:0] page_effect;
 input [5:0] page;
 begin page_effect=page[1:0]-2'd1;end
endfunction
always @(posedge clk or posedge rst) begin
 if(rst) begin
  transition_state<=FX_IDLE;transition_kind<=0;transition_level<=0;
  transition_seen<=0;transition_token<=0;transition_auto<=0;cancel_frames<=0;transition_reply<=0;
 end else if(vs_in && !vs_last) begin
  if(ENABLE_TRANSITIONS==0 || !frame_valid) begin
   transition_state<=FX_IDLE;transition_level<=0;transition_seen<=0;transition_reply<=0;cancel_frames<=0;
  end else begin
   if(!transition_command[8]) transition_seen<=0;
   if(transition_command[8] && (!transition_seen || transition_command[7]!=transition_token)) begin
    transition_seen<=1;transition_token<=transition_command[7];transition_auto<=transition_command[6];
    transition_kind<=page_effect(transition_command[5:0]);
    transition_state<=FX_COVER;transition_reply<=0;cancel_frames<=0;
   end else if(page_changed) begin
    transition_state<=FX_REVEAL;transition_level<=16;transition_reply<=0;cancel_frames<=0;
    transition_kind<=page_effect(visible_page);
   end else case(transition_state)
    FX_COVER: begin
     if(!transition_command[8]) begin transition_state<=FX_REVEAL;transition_reply<=0;end
     else if(transition_level>=16-transition_step) begin
      transition_level<=16;transition_state<=FX_WAIT;transition_reply<={1'b1,transition_token};
     end else transition_level<=transition_level+transition_step;
    end
    FX_WAIT: begin
     // Allow independent coherent status/command CDCs several frame edges to
     // converge after commit. A true cancelled command restores the old page.
     if(!transition_command[8]) begin
      if(cancel_frames==3) begin transition_state<=FX_REVEAL;transition_reply<=0;end
      else cancel_frames<=cancel_frames+1'b1;
     end else cancel_frames<=0;
    end
    FX_REVEAL: begin
     transition_reply<=0;
     if(transition_level<=transition_step) begin transition_level<=0;transition_state<=FX_IDLE;end
     else transition_level<=transition_level-transition_step;
    end
    default: begin transition_level<=0;transition_reply<=0;end
   endcase
  end
 end
end
// First and subsequent passes enter at x1280, completely outside the right
// edge, then move left. Reset on every committed page/narration session.
// A full-screen gap separates repeats. Position updates only in blanking.
reg signed [13:0] ticker_sum_pre,ticker_position_pre;
// No resets on data registers: valid markers mask startup values. This avoids
// adding reset fanout while separating coordinate and character-ROM clocks.
always @(posedge clk) begin
 x_pre0<=raw_x;y_pre0<=raw_y;x_pre1<=x_pre0;y_pre1<=y_pre0;
 rgb_pre0<=rgb_in;rgb_pre1<=rgb_pre0;
 ticker_sum_pre<=$signed({3'd0,raw_x})+$signed({2'd0,scroll})-14'sd1280;
 // Phase0..text_width+1279 guarantees no second copy within this scanline.
 // Keep the second coordinate register for the existing eight-clock pipeline.
 ticker_position_pre<=ticker_sum_pre;
end
always @(posedge clk or posedge rst) begin
 if(rst) begin de_pre<=0;hs_pre<=0;vs_pre<=0;end
 else begin
  de_pre<={de_pre[0],de_in};hs_pre<={hs_pre[0],hs_in};vs_pre<={vs_pre[0],vs_in};
 end
end
wire signed [13:0] ticker_position=ticker_position_pre;
wire sub_area=enabled_frame && pixel_de && y>=672;
wire sub_text=sub_area && y>=680 && y<712 && ticker_position>=0 &&
 ticker_position<$signed({2'd0,text_length,5'd0});
// Decode the character independently of clipping. submask_pipe already masks
// every out-of-strip/out-of-text pixel at composition, so an invalid pixel's
// speculative glyph is never displayed. This removes the range-comparator ->
// index mux -> large character lookup chain without adding a pixel clock.
wire [6:0] character_index=ticker_position[11:5];
wire [10:0] caption_y=y-11'd680;
wire [8:0] character_id,focus_id;
wire [31:0] font_row,focus_row;
wire [5:0] sprite_pixel;
// Native32px focus glyphs stay at exact2x throughout the fade. Shift-only
// addressing preserves even strokes and removes two variable-scale products.
reg [10:0] teacher_left;
reg [10:0] focus_top,focus_bottom,card_top,card_bottom;
wire [10:0] focus_dx=x-11'd256,focus_dy=y-focus_top;
wire [8:0] focus_u=focus_dx[9:1];
wire [6:0] focus_v=focus_dy[7:1];
wire focus_text=enabled_frame && focus_mix!=0 && x>=256 && x<1024 && y>=focus_top && y<focus_bottom;
wire [5:0] focus_index=({4'd0,focus_v[6:5]}<<3)+({4'd0,focus_v[6:5]}<<2)+{2'd0,focus_u[8:5]};
//96x144 source ->144x216 output:12.5% larger than the previous128x192.
// Exact bounded3/2 nearest-neighbor mapping, with no larger sprite ROM.
wire [10:0] teacher_dx=x-teacher_left,teacher_dy=y-teacher_y;
wire [7:0] teacher_dy8=teacher_dy[7:0],teacher_dx8=teacher_dx[7:0];
// floor(171*dy/256)==floor(2*dy/3) for0<=dy<216.
wire [15:0] teacher_v_scaled=({8'd0,teacher_dy8}<<7)+({8'd0,teacher_dy8}<<5)+
 ({8'd0,teacher_dy8}<<3)+({8'd0,teacher_dy8}<<1)+{8'd0,teacher_dy8};
wire [7:0] teacher_v=teacher_v_scaled[15:8];
wire teacher_area=enabled_frame && teacher_shown && pixel_de &&
 x>=teacher_left && x<teacher_left+144 && y>=teacher_y && y<teacher_y+216;
reg [1:0] pose;
// Geometry changes once per frame and settles while DE is low. Register it
// before per-pixel ROM addressing. The render-pipeline latency is unchanged.
always @(posedge clk or posedge rst) begin
 if(rst) begin teacher_left<=1304;pose<=0;focus_top<=296;focus_bottom<=424;card_top<=264;card_bottom<=456;end
 else if(!pixel_de) begin
  focus_top<=11'd360-({9'd0,focus_lines}<<5);
  focus_bottom<=11'd360+({9'd0,focus_lines}<<5);
  card_top<=11'd328-({9'd0,focus_lines}<<5);
  card_bottom<=11'd392+({9'd0,focus_lines}<<5);
  teacher_left<=teacher_x+((11'd32-{5'd0,entry_age})<<2)+(11'd32-{5'd0,entry_age});
  pose<=entry_age<20 ? 2'd1 : focus_mix!=0 ? (frame_count[3] ? 2'd3 : 2'd2) : 2'd0;
 end
end
wire [14:0] sprite_base=pose==0 ? 15'd0 : pose==1 ? 15'd4608 : pose==2 ? 15'd9216 : 15'd13824;
// floor(57*dx/256)==floor(2*dx/9) for0<=dx<144: packed3-pixel word.
// The residual0..8 selects floor(residual/3), avoiding general division.
wire [15:0] teacher_word_scaled=({8'd0,teacher_dx8}<<6)-({8'd0,teacher_dx8}<<3)+{8'd0,teacher_dx8};
wire [4:0] sprite_word_x=teacher_word_scaled[12:8];
wire [8:0] teacher_residual={teacher_dx8,1'b0}-(({4'd0,sprite_word_x}<<3)+{4'd0,sprite_word_x});
wire [1:0] sprite_lane=teacher_residual<3 ? 2'd0 : teacher_residual<6 ? 2'd1 : 2'd2;
wire [14:0] sprite_address=teacher_area ? sprite_base+{2'd0,teacher_v,5'd0}+{10'd0,sprite_word_x} : 15'd0;
reg [4:0] sub_row0,sub_col0,sub_col1,sub_col2,focus_row0,focus_col0,focus_col1,focus_col2;
reg [2:0] focusmask_pipe;
album_teaching_assets assets(
 .clk(clk),.page(page_frame[3:0]),.character_index(character_index),.focus_index(focus_index),
 .subtitle_font_address({character_id[7:0],sub_row0}),
 .focus_font_address({focus_id[7:0],focus_row0}),.sprite_address(sprite_address),.sprite_lane(sprite_lane),
 .focus_font_select(focusmask_pipe[0]),
 .character_id(character_id),.focus_id(focus_id),.subtitle_font_row(font_row),.focus_font_row(focus_row),
 .sprite_pixel(sprite_pixel),.text_length(text_length),.demo_start_cs(demo_start),.demo_end_cs(demo_end),
 .teacher_x(teacher_x),.teacher_y(teacher_y),.focus_lines(focus_lines),.scroll_step(scroll_step));
// Real separable 3x3 spatial blur, weights [1,2,1] in both directions.
// Two previous scanlines; clamp unavailable top/left neighbors to edge pixels.
reg [23:0] line1[0:1279]; /* fehdl force_ram=1, ram_style="bram" */
reg [23:0] line2[0:1279]; /* fehdl force_ram=1, ram_style="bram" */
reg [23:0] q1,q2,rgb0,rgb1,rgb2,vertical1,blur2,left1,left2,bg3,bg4;
reg [10:0] x0,y0,x1;
reg [4:0] de_pipe,hs_pipe,vs_pipe,sub_pipe,submask_pipe,teacher_pipe;
reg [5:0] sprite2,sprite3,sprite4;
reg sub_ink3,sub_ink4,focus_ink3;
reg [3:0] effect0,effect1,effect2,effect3;
wire preserve_ui=(x>=STATUS_X && x<STATUS_X+224 && y>=STATUS_Y && y<STATUS_Y+64) ||
 (sidebar_visible && x<144 && y>=192 && y<528);
wire card=enabled_frame && focus_mix!=0 && x>=224 && x<1056 && y>=card_top && y<card_bottom;
// Warm paper with a fine ochre keyline and short traditional corner rules.
wire card_border=card && (
 (((x==236 || x==1043) && y>=card_top+12 && y<card_bottom-12) ||
  ((y==card_top+12 || y==card_bottom-13) && x>=236 && x<1044)) ||
 ((x<226 || x>=1054) && (y<card_top+24 || y>=card_bottom-24)) ||
 ((y<card_top+2 || y>=card_bottom-2) && (x<248 || x>=1032)));
wire shadow=enabled_frame && focus_mix!=0 && !card && x>=232 && x<1064 && y>=card_top+8 && y<card_bottom+8;
reg [10:0] wipe_edge;
reg [5:0] blind_edge;
always @(posedge clk or posedge rst) begin
 if(rst) begin wipe_edge<=0;blind_edge<=0;end
 else if(!pixel_de) begin
  //720p raster thresholds settle during blanking, off the RGB critical path.
  if(transition_state==FX_REVEAL) begin
   wipe_edge<=({6'd0,(5'd16-transition_level)}<<6)+({6'd0,(5'd16-transition_level)}<<4);
   blind_edge<={1'b0,(5'd16-transition_level)}<<1;
  end else begin
   wipe_edge<=({6'd0,transition_level}<<6)+({6'd0,transition_level}<<4);
   blind_edge<={1'b0,transition_level}<<1;
  end
 end
end
wire [3:0] dissolve_order=x[3:0]^y[3:0]^{x[6:5],y[6:5]};
reg [4:0] transition_alpha;
reg [4:0] trans0,trans1,trans2,trans3,trans4,trans5;
always @* begin
 transition_alpha=0;
 if(ENABLE_TRANSITIONS!=0 && frame_valid && !preserve_ui) begin
  case(transition_kind)
   0:transition_alpha=transition_level;
   1:transition_alpha=(transition_state==FX_REVEAL ? x>=wipe_edge : x<wipe_edge) ? 5'd16 : 5'd0;
   2:transition_alpha=(transition_state==FX_REVEAL ? {1'b0,y[4:0]}>=blind_edge :
                       {1'b0,y[4:0]}<blind_edge) ? 5'd16 : 5'd0;
   3:transition_alpha={1'b0,dissolve_order}<transition_level ? 5'd16 : 5'd0;
   default:transition_alpha=0;
  endcase
 end
end
function [7:0] filter_channel;
 input [7:0] a,b,c;
 reg [9:0] sum;
 begin sum={2'd0,a}+{1'd0,b,1'b0}+{2'd0,c};filter_channel=sum[9:2];end
endfunction
function [23:0] filter3;
 input [23:0] a,b,c;
 begin filter3={filter_channel(a[23:16],b[23:16],c[23:16]),
 filter_channel(a[15:8],b[15:8],c[15:8]),filter_channel(a[7:0],b[7:0],c[7:0])};end
endfunction
function [7:0] mix_channel;
 input [7:0] foreground,background;
 input [4:0] alpha;
 reg signed [9:0] difference;
 reg signed [15:0] sum;
 begin
  difference=$signed({2'd0,foreground})-$signed({2'd0,background});
  sum=$signed({4'd0,background,4'd0})+difference*$signed({1'b0,alpha});
  mix_channel=sum[11:4];
 end
endfunction
function [23:0] mix_rgb;
 input [23:0] foreground,background;
 input [4:0] alpha;
 begin mix_rgb={mix_channel(foreground[23:16],background[23:16],alpha),
 mix_channel(foreground[15:8],background[15:8],alpha),mix_channel(foreground[7:0],background[7:0],alpha)};end
endfunction
wire [23:0] teacher_rgb;
album_teacher_palette palette(.index(sprite4[4:0]),.rgb(teacher_rgb));
wire [4:0] teacher_alpha=entry_age>=16 ? 5'd16 : entry_age[4:0];
reg hs_composite,vs_composite,de_composite;
reg [23:0] rgb_composite;
always @(posedge clk) begin
 if(pixel_de) begin q1<=line1[x];q2<=line2[x];line1[x]<=pixel_rgb;end
 if(de_pipe[0]) line2[x0]<=q1;
end
always @(posedge clk or posedge rst) begin
 if(rst) begin
  de_pipe<=0;hs_pipe<=0;vs_pipe<=0;sub_pipe<=0;submask_pipe<=0;teacher_pipe<=0;focusmask_pipe<=0;
  rgb0<=0;rgb1<=0;rgb2<=0;vertical1<=0;blur2<=0;left1<=0;left2<=0;bg3<=0;bg4<=0;
  x0<=0;y0<=0;x1<=0;sub_row0<=0;sub_col0<=0;sub_col1<=0;sub_col2<=0;
  focus_row0<=0;focus_col0<=0;focus_col1<=0;focus_col2<=0;
  sprite2<=0;sprite3<=0;sprite4<=0;sub_ink3<=0;sub_ink4<=0;
  focus_ink3<=0;effect0<=0;effect1<=0;effect2<=0;effect3<=0;
  de_composite<=0;hs_composite<=0;vs_composite<=0;rgb_composite<=0;
  trans0<=0;trans1<=0;trans2<=0;trans3<=0;trans4<=0;trans5<=0;
 end else begin
  de_pipe<={de_pipe[3:0],pixel_de};hs_pipe<={hs_pipe[3:0],pixel_hs};vs_pipe<={vs_pipe[3:0],pixel_vs};
  sub_pipe<={sub_pipe[3:0],sub_area};submask_pipe<={submask_pipe[3:0],sub_text};
  teacher_pipe<={teacher_pipe[3:0],teacher_area};focusmask_pipe<={focusmask_pipe[1:0],focus_text};
  x0<=x;y0<=y;x1<=x0;rgb0<=pixel_rgb;rgb1<=rgb0;rgb2<=rgb1;
  sub_row0<=caption_y[4:0];sub_col0<=ticker_position[4:0];sub_col1<=sub_col0;sub_col2<=sub_col1;
  focus_row0<=focus_v[4:0];focus_col0<=focus_u[4:0];focus_col1<=focus_col0;focus_col2<=focus_col1;
  vertical1<=filter3(rgb0,y0==0 ? rgb0 : q1,y0<2 ? (y0==0 ? rgb0 : q1) : q2);
  if(de_pipe[1]) begin
   blur2<=filter3(vertical1,x1==0 ? vertical1 : left1,x1==0 ? vertical1 : x1==1 ? left1 : left2);
   left1<=vertical1;left2<=left1;
  end
  effect0<={preserve_ui,shadow,card_border,card};effect1<=effect0;effect2<=effect1;effect3<=effect2;
  trans0<=transition_alpha;trans1<=trans0;trans2<=trans1;trans3<=trans2;trans4<=trans3;trans5<=trans4;
  // Span ROM adds a memory access. Reuse the previous ink/sprite delay slots
  // so the external six-stage RGB/HS/VS/DE latency remains unchanged.
  sprite2<=sprite_pixel;sprite3<=sprite2;sprite4<=sprite3;
  sub_ink3<=submask_pipe[2] && font_row[31-sub_col2];sub_ink4<=sub_ink3;
  focus_ink3<=focusmask_pipe[2] && focus_row[31-focus_col2];
  bg3<=effect2[3] ? rgb2 : mix_rgb({1'b0,blur2[23:17],1'b0,blur2[15:9],1'b0,blur2[7:1]},rgb2,focus_mix);
  if(focus_ink3) bg4<=mix_rgb(24'h382c22,bg3,focus_mix);
  else if(effect3[1]) bg4<=mix_rgb(24'ha18858,bg3,focus_mix);
  else if(effect3[0]) bg4<=mix_rgb(24'hf1e5cb,bg3,focus_mix);
  else if(effect3[2]) bg4<=mix_rgb(24'd0,bg3,{1'b0,focus_mix[4:1]});
  else bg4<=bg3;
  hs_composite<=hs_pipe[4];vs_composite<=vs_pipe[4];de_composite<=de_pipe[4];
  if(!de_pipe[4]) rgb_composite<=0;
  else if(sub_ink4) rgb_composite<=24'hffffff;
  else if(sub_pipe[4]) rgb_composite<={2'd0,bg4[23:18],2'd0,bg4[15:10],2'd0,bg4[7:2]};
  else if(teacher_pipe[4] && sprite4[5]) rgb_composite<=mix_rgb(teacher_rgb,bg4,teacher_alpha);
  else rgb_composite<=bg4;
 end
end
// Separate registered transition blend avoids chaining teacher alpha and
// page alpha in one pixel period. HDMI timing markers gain the same clock.
always @(posedge clk or posedge rst) begin
 if(rst) begin hs_out<=0;vs_out<=0;de_out<=0;rgb_out<=0;end
 else begin
  hs_out<=hs_composite;vs_out<=vs_composite;de_out<=de_composite;
  rgb_out<=de_composite ? mix_rgb(24'hf1e5cb,rgb_composite,trans5) : 24'd0;
 end
end
endmodule

// Generated by tools/prepare_teaching_assets.py; do not edit ROM literals.
module album_teaching_assets(
 input clk,input [3:0] page,input [6:0] character_index,
 input [5:0] focus_index,input [12:0] subtitle_font_address,focus_font_address,input focus_font_select,
 input [14:0] sprite_address,input [1:0] sprite_lane,
 output reg [8:0] character_id,focus_id,
 output [31:0] subtitle_font_row,focus_font_row,
 output [5:0] sprite_pixel,output reg [6:0] text_length,
 output reg [15:0] demo_start_cs,demo_end_cs,output [10:0] teacher_x,teacher_y,
 output reg [1:0] focus_lines,output [3:0] scroll_step
);
reg [3:0] text_id_lo [0:2047]; /* fehdl force_ram=1, ram_style="bram" */
initial begin
text_id_lo[0]=4'h0; text_id_lo[1]=4'h0; text_id_lo[2]=4'h0; text_id_lo[3]=4'h0; text_id_lo[4]=4'h0; text_id_lo[5]=4'h0; text_id_lo[6]=4'h0; text_id_lo[7]=4'h0;
text_id_lo[8]=4'h0; text_id_lo[9]=4'h0; text_id_lo[10]=4'h0; text_id_lo[11]=4'h0; text_id_lo[12]=4'h0; text_id_lo[13]=4'h0; text_id_lo[14]=4'h0; text_id_lo[15]=4'h0;
text_id_lo[16]=4'h0; text_id_lo[17]=4'h0; text_id_lo[18]=4'h0; text_id_lo[19]=4'h0; text_id_lo[20]=4'h0; text_id_lo[21]=4'h0; text_id_lo[22]=4'h0; text_id_lo[23]=4'h0;
text_id_lo[24]=4'h0; text_id_lo[25]=4'h0; text_id_lo[26]=4'h0; text_id_lo[27]=4'h0; text_id_lo[28]=4'h0; text_id_lo[29]=4'h0; text_id_lo[30]=4'h0; text_id_lo[31]=4'h0;
text_id_lo[32]=4'h0; text_id_lo[33]=4'h0; text_id_lo[34]=4'h0; text_id_lo[35]=4'h0; text_id_lo[36]=4'h0; text_id_lo[37]=4'h0; text_id_lo[38]=4'h0; text_id_lo[39]=4'h0;
text_id_lo[40]=4'h0; text_id_lo[41]=4'h0; text_id_lo[42]=4'h0; text_id_lo[43]=4'h0; text_id_lo[44]=4'h0; text_id_lo[45]=4'h0; text_id_lo[46]=4'h0; text_id_lo[47]=4'h0;
text_id_lo[48]=4'h0; text_id_lo[49]=4'h0; text_id_lo[50]=4'h0; text_id_lo[51]=4'h0; text_id_lo[52]=4'h0; text_id_lo[53]=4'h0; text_id_lo[54]=4'h0; text_id_lo[55]=4'h0;
text_id_lo[56]=4'h0; text_id_lo[57]=4'h0; text_id_lo[58]=4'h0; text_id_lo[59]=4'h0; text_id_lo[60]=4'h0; text_id_lo[61]=4'h0; text_id_lo[62]=4'h0; text_id_lo[63]=4'h0;
text_id_lo[64]=4'h0; text_id_lo[65]=4'h0; text_id_lo[66]=4'h0; text_id_lo[67]=4'h0; text_id_lo[68]=4'h0; text_id_lo[69]=4'h0; text_id_lo[70]=4'h0; text_id_lo[71]=4'h0;
text_id_lo[72]=4'h0; text_id_lo[73]=4'h0; text_id_lo[74]=4'h0; text_id_lo[75]=4'h0; text_id_lo[76]=4'h0; text_id_lo[77]=4'h0; text_id_lo[78]=4'h0; text_id_lo[79]=4'h0;
text_id_lo[80]=4'h0; text_id_lo[81]=4'h0; text_id_lo[82]=4'h0; text_id_lo[83]=4'h0; text_id_lo[84]=4'h0; text_id_lo[85]=4'h0; text_id_lo[86]=4'h0; text_id_lo[87]=4'h0;
text_id_lo[88]=4'h0; text_id_lo[89]=4'h0; text_id_lo[90]=4'h0; text_id_lo[91]=4'h0; text_id_lo[92]=4'h0; text_id_lo[93]=4'h0; text_id_lo[94]=4'h0; text_id_lo[95]=4'h0;
text_id_lo[96]=4'h0; text_id_lo[97]=4'h0; text_id_lo[98]=4'h0; text_id_lo[99]=4'h0; text_id_lo[100]=4'h0; text_id_lo[101]=4'h0; text_id_lo[102]=4'h0; text_id_lo[103]=4'h0;
text_id_lo[104]=4'h0; text_id_lo[105]=4'h0; text_id_lo[106]=4'h0; text_id_lo[107]=4'h0; text_id_lo[108]=4'h0; text_id_lo[109]=4'h0; text_id_lo[110]=4'h0; text_id_lo[111]=4'h0;
text_id_lo[112]=4'h0; text_id_lo[113]=4'h0; text_id_lo[114]=4'h0; text_id_lo[115]=4'h0; text_id_lo[116]=4'h0; text_id_lo[117]=4'h0; text_id_lo[118]=4'h0; text_id_lo[119]=4'h0;
text_id_lo[120]=4'h0; text_id_lo[121]=4'h0; text_id_lo[122]=4'h0; text_id_lo[123]=4'h0; text_id_lo[124]=4'h0; text_id_lo[125]=4'h0; text_id_lo[126]=4'h0; text_id_lo[127]=4'h0;
text_id_lo[128]=4'hd; text_id_lo[129]=4'h7; text_id_lo[130]=4'h0; text_id_lo[131]=4'he; text_id_lo[132]=4'h9; text_id_lo[133]=4'he; text_id_lo[134]=4'hc; text_id_lo[135]=4'hd;
text_id_lo[136]=4'h9; text_id_lo[137]=4'h0; text_id_lo[138]=4'h0; text_id_lo[139]=4'h0; text_id_lo[140]=4'h2; text_id_lo[141]=4'h7; text_id_lo[142]=4'h7; text_id_lo[143]=4'h7;
text_id_lo[144]=4'h4; text_id_lo[145]=4'h5; text_id_lo[146]=4'he; text_id_lo[147]=4'h1; text_id_lo[148]=4'h9; text_id_lo[149]=4'h3; text_id_lo[150]=4'hc; text_id_lo[151]=4'h0;
text_id_lo[152]=4'h8; text_id_lo[153]=4'he; text_id_lo[154]=4'hb; text_id_lo[155]=4'hc; text_id_lo[156]=4'h3; text_id_lo[157]=4'hb; text_id_lo[158]=4'hc; text_id_lo[159]=4'hc;
text_id_lo[160]=4'h7; text_id_lo[161]=4'hb; text_id_lo[162]=4'h1; text_id_lo[163]=4'he; text_id_lo[164]=4'h7; text_id_lo[165]=4'h6; text_id_lo[166]=4'h0; text_id_lo[167]=4'h0;
text_id_lo[168]=4'h0; text_id_lo[169]=4'h0; text_id_lo[170]=4'h0; text_id_lo[171]=4'h0; text_id_lo[172]=4'h0; text_id_lo[173]=4'h0; text_id_lo[174]=4'h0; text_id_lo[175]=4'h0;
text_id_lo[176]=4'h0; text_id_lo[177]=4'h0; text_id_lo[178]=4'h0; text_id_lo[179]=4'h0; text_id_lo[180]=4'h0; text_id_lo[181]=4'h0; text_id_lo[182]=4'h0; text_id_lo[183]=4'h0;
text_id_lo[184]=4'h0; text_id_lo[185]=4'h0; text_id_lo[186]=4'h0; text_id_lo[187]=4'h0; text_id_lo[188]=4'h0; text_id_lo[189]=4'h0; text_id_lo[190]=4'h0; text_id_lo[191]=4'h0;
text_id_lo[192]=4'h0; text_id_lo[193]=4'h0; text_id_lo[194]=4'h0; text_id_lo[195]=4'h0; text_id_lo[196]=4'h0; text_id_lo[197]=4'h0; text_id_lo[198]=4'h0; text_id_lo[199]=4'h0;
text_id_lo[200]=4'h0; text_id_lo[201]=4'h0; text_id_lo[202]=4'h0; text_id_lo[203]=4'h0; text_id_lo[204]=4'h0; text_id_lo[205]=4'h0; text_id_lo[206]=4'h0; text_id_lo[207]=4'h0;
text_id_lo[208]=4'h0; text_id_lo[209]=4'h0; text_id_lo[210]=4'h0; text_id_lo[211]=4'h0; text_id_lo[212]=4'h0; text_id_lo[213]=4'h0; text_id_lo[214]=4'h0; text_id_lo[215]=4'h0;
text_id_lo[216]=4'h0; text_id_lo[217]=4'h0; text_id_lo[218]=4'h0; text_id_lo[219]=4'h0; text_id_lo[220]=4'h0; text_id_lo[221]=4'h0; text_id_lo[222]=4'h0; text_id_lo[223]=4'h0;
text_id_lo[224]=4'h0; text_id_lo[225]=4'h0; text_id_lo[226]=4'h0; text_id_lo[227]=4'h0; text_id_lo[228]=4'h0; text_id_lo[229]=4'h0; text_id_lo[230]=4'h0; text_id_lo[231]=4'h0;
text_id_lo[232]=4'h0; text_id_lo[233]=4'h0; text_id_lo[234]=4'h0; text_id_lo[235]=4'h0; text_id_lo[236]=4'h0; text_id_lo[237]=4'h0; text_id_lo[238]=4'h0; text_id_lo[239]=4'h0;
text_id_lo[240]=4'h0; text_id_lo[241]=4'h0; text_id_lo[242]=4'h0; text_id_lo[243]=4'h0; text_id_lo[244]=4'h0; text_id_lo[245]=4'h0; text_id_lo[246]=4'h0; text_id_lo[247]=4'h0;
text_id_lo[248]=4'h0; text_id_lo[249]=4'h0; text_id_lo[250]=4'h0; text_id_lo[251]=4'h0; text_id_lo[252]=4'h0; text_id_lo[253]=4'h0; text_id_lo[254]=4'h0; text_id_lo[255]=4'h0;
text_id_lo[256]=4'hf; text_id_lo[257]=4'h0; text_id_lo[258]=4'ha; text_id_lo[259]=4'hd; text_id_lo[260]=4'h7; text_id_lo[261]=4'hf; text_id_lo[262]=4'hf; text_id_lo[263]=4'hf;
text_id_lo[264]=4'h9; text_id_lo[265]=4'he; text_id_lo[266]=4'h8; text_id_lo[267]=4'h1; text_id_lo[268]=4'h5; text_id_lo[269]=4'hf; text_id_lo[270]=4'h2; text_id_lo[271]=4'he;
text_id_lo[272]=4'hb; text_id_lo[273]=4'hb; text_id_lo[274]=4'h5; text_id_lo[275]=4'hf; text_id_lo[276]=4'ha; text_id_lo[277]=4'he; text_id_lo[278]=4'h0; text_id_lo[279]=4'h0;
text_id_lo[280]=4'h6; text_id_lo[281]=4'h0; text_id_lo[282]=4'h0; text_id_lo[283]=4'h0; text_id_lo[284]=4'h0; text_id_lo[285]=4'h0; text_id_lo[286]=4'h0; text_id_lo[287]=4'h0;
text_id_lo[288]=4'h0; text_id_lo[289]=4'h0; text_id_lo[290]=4'h0; text_id_lo[291]=4'h0; text_id_lo[292]=4'h0; text_id_lo[293]=4'h0; text_id_lo[294]=4'h0; text_id_lo[295]=4'h0;
text_id_lo[296]=4'h0; text_id_lo[297]=4'h0; text_id_lo[298]=4'h0; text_id_lo[299]=4'h0; text_id_lo[300]=4'h0; text_id_lo[301]=4'h0; text_id_lo[302]=4'h0; text_id_lo[303]=4'h0;
text_id_lo[304]=4'h0; text_id_lo[305]=4'h0; text_id_lo[306]=4'h0; text_id_lo[307]=4'h0; text_id_lo[308]=4'h0; text_id_lo[309]=4'h0; text_id_lo[310]=4'h0; text_id_lo[311]=4'h0;
text_id_lo[312]=4'h0; text_id_lo[313]=4'h0; text_id_lo[314]=4'h0; text_id_lo[315]=4'h0; text_id_lo[316]=4'h0; text_id_lo[317]=4'h0; text_id_lo[318]=4'h0; text_id_lo[319]=4'h0;
text_id_lo[320]=4'h0; text_id_lo[321]=4'h0; text_id_lo[322]=4'h0; text_id_lo[323]=4'h0; text_id_lo[324]=4'h0; text_id_lo[325]=4'h0; text_id_lo[326]=4'h0; text_id_lo[327]=4'h0;
text_id_lo[328]=4'h0; text_id_lo[329]=4'h0; text_id_lo[330]=4'h0; text_id_lo[331]=4'h0; text_id_lo[332]=4'h0; text_id_lo[333]=4'h0; text_id_lo[334]=4'h0; text_id_lo[335]=4'h0;
text_id_lo[336]=4'h0; text_id_lo[337]=4'h0; text_id_lo[338]=4'h0; text_id_lo[339]=4'h0; text_id_lo[340]=4'h0; text_id_lo[341]=4'h0; text_id_lo[342]=4'h0; text_id_lo[343]=4'h0;
text_id_lo[344]=4'h0; text_id_lo[345]=4'h0; text_id_lo[346]=4'h0; text_id_lo[347]=4'h0; text_id_lo[348]=4'h0; text_id_lo[349]=4'h0; text_id_lo[350]=4'h0; text_id_lo[351]=4'h0;
text_id_lo[352]=4'h0; text_id_lo[353]=4'h0; text_id_lo[354]=4'h0; text_id_lo[355]=4'h0; text_id_lo[356]=4'h0; text_id_lo[357]=4'h0; text_id_lo[358]=4'h0; text_id_lo[359]=4'h0;
text_id_lo[360]=4'h0; text_id_lo[361]=4'h0; text_id_lo[362]=4'h0; text_id_lo[363]=4'h0; text_id_lo[364]=4'h0; text_id_lo[365]=4'h0; text_id_lo[366]=4'h0; text_id_lo[367]=4'h0;
text_id_lo[368]=4'h0; text_id_lo[369]=4'h0; text_id_lo[370]=4'h0; text_id_lo[371]=4'h0; text_id_lo[372]=4'h0; text_id_lo[373]=4'h0; text_id_lo[374]=4'h0; text_id_lo[375]=4'h0;
text_id_lo[376]=4'h0; text_id_lo[377]=4'h0; text_id_lo[378]=4'h0; text_id_lo[379]=4'h0; text_id_lo[380]=4'h0; text_id_lo[381]=4'h0; text_id_lo[382]=4'h0; text_id_lo[383]=4'h0;
text_id_lo[384]=4'ha; text_id_lo[385]=4'h2; text_id_lo[386]=4'h9; text_id_lo[387]=4'h3; text_id_lo[388]=4'hc; text_id_lo[389]=4'h0; text_id_lo[390]=4'h4; text_id_lo[391]=4'h4;
text_id_lo[392]=4'h8; text_id_lo[393]=4'hd; text_id_lo[394]=4'h6; text_id_lo[395]=4'hb; text_id_lo[396]=4'h3; text_id_lo[397]=4'hd; text_id_lo[398]=4'h2; text_id_lo[399]=4'h2;
text_id_lo[400]=4'hc; text_id_lo[401]=4'hb; text_id_lo[402]=4'hb; text_id_lo[403]=4'he; text_id_lo[404]=4'h9; text_id_lo[405]=4'h8; text_id_lo[406]=4'hf; text_id_lo[407]=4'h7;
text_id_lo[408]=4'h5; text_id_lo[409]=4'hc; text_id_lo[410]=4'h0; text_id_lo[411]=4'h5; text_id_lo[412]=4'hc; text_id_lo[413]=4'h4; text_id_lo[414]=4'h5; text_id_lo[415]=4'he;
text_id_lo[416]=4'h9; text_id_lo[417]=4'h5; text_id_lo[418]=4'h8; text_id_lo[419]=4'h0; text_id_lo[420]=4'h0; text_id_lo[421]=4'h6; text_id_lo[422]=4'h0; text_id_lo[423]=4'h0;
text_id_lo[424]=4'h0; text_id_lo[425]=4'h0; text_id_lo[426]=4'h0; text_id_lo[427]=4'h0; text_id_lo[428]=4'h0; text_id_lo[429]=4'h0; text_id_lo[430]=4'h0; text_id_lo[431]=4'h0;
text_id_lo[432]=4'h0; text_id_lo[433]=4'h0; text_id_lo[434]=4'h0; text_id_lo[435]=4'h0; text_id_lo[436]=4'h0; text_id_lo[437]=4'h0; text_id_lo[438]=4'h0; text_id_lo[439]=4'h0;
text_id_lo[440]=4'h0; text_id_lo[441]=4'h0; text_id_lo[442]=4'h0; text_id_lo[443]=4'h0; text_id_lo[444]=4'h0; text_id_lo[445]=4'h0; text_id_lo[446]=4'h0; text_id_lo[447]=4'h0;
text_id_lo[448]=4'h0; text_id_lo[449]=4'h0; text_id_lo[450]=4'h0; text_id_lo[451]=4'h0; text_id_lo[452]=4'h0; text_id_lo[453]=4'h0; text_id_lo[454]=4'h0; text_id_lo[455]=4'h0;
text_id_lo[456]=4'h0; text_id_lo[457]=4'h0; text_id_lo[458]=4'h0; text_id_lo[459]=4'h0; text_id_lo[460]=4'h0; text_id_lo[461]=4'h0; text_id_lo[462]=4'h0; text_id_lo[463]=4'h0;
text_id_lo[464]=4'h0; text_id_lo[465]=4'h0; text_id_lo[466]=4'h0; text_id_lo[467]=4'h0; text_id_lo[468]=4'h0; text_id_lo[469]=4'h0; text_id_lo[470]=4'h0; text_id_lo[471]=4'h0;
text_id_lo[472]=4'h0; text_id_lo[473]=4'h0; text_id_lo[474]=4'h0; text_id_lo[475]=4'h0; text_id_lo[476]=4'h0; text_id_lo[477]=4'h0; text_id_lo[478]=4'h0; text_id_lo[479]=4'h0;
text_id_lo[480]=4'h0; text_id_lo[481]=4'h0; text_id_lo[482]=4'h0; text_id_lo[483]=4'h0; text_id_lo[484]=4'h0; text_id_lo[485]=4'h0; text_id_lo[486]=4'h0; text_id_lo[487]=4'h0;
text_id_lo[488]=4'h0; text_id_lo[489]=4'h0; text_id_lo[490]=4'h0; text_id_lo[491]=4'h0; text_id_lo[492]=4'h0; text_id_lo[493]=4'h0; text_id_lo[494]=4'h0; text_id_lo[495]=4'h0;
text_id_lo[496]=4'h0; text_id_lo[497]=4'h0; text_id_lo[498]=4'h0; text_id_lo[499]=4'h0; text_id_lo[500]=4'h0; text_id_lo[501]=4'h0; text_id_lo[502]=4'h0; text_id_lo[503]=4'h0;
text_id_lo[504]=4'h0; text_id_lo[505]=4'h0; text_id_lo[506]=4'h0; text_id_lo[507]=4'h0; text_id_lo[508]=4'h0; text_id_lo[509]=4'h0; text_id_lo[510]=4'h0; text_id_lo[511]=4'h0;
text_id_lo[512]=4'hb; text_id_lo[513]=4'hb; text_id_lo[514]=4'h2; text_id_lo[515]=4'hf; text_id_lo[516]=4'hd; text_id_lo[517]=4'h8; text_id_lo[518]=4'h9; text_id_lo[519]=4'hc;
text_id_lo[520]=4'ha; text_id_lo[521]=4'h4; text_id_lo[522]=4'h4; text_id_lo[523]=4'hb; text_id_lo[524]=4'h6; text_id_lo[525]=4'h9; text_id_lo[526]=4'h9; text_id_lo[527]=4'h3;
text_id_lo[528]=4'he; text_id_lo[529]=4'h1; text_id_lo[530]=4'h3; text_id_lo[531]=4'he; text_id_lo[532]=4'h8; text_id_lo[533]=4'hb; text_id_lo[534]=4'h3; text_id_lo[535]=4'h7;
text_id_lo[536]=4'h5; text_id_lo[537]=4'h8; text_id_lo[538]=4'hd; text_id_lo[539]=4'h5; text_id_lo[540]=4'hd; text_id_lo[541]=4'h7; text_id_lo[542]=4'h5; text_id_lo[543]=4'h9;
text_id_lo[544]=4'h9; text_id_lo[545]=4'h5; text_id_lo[546]=4'hb; text_id_lo[547]=4'h6; text_id_lo[548]=4'h6; text_id_lo[549]=4'h0; text_id_lo[550]=4'h0; text_id_lo[551]=4'h0;
text_id_lo[552]=4'h0; text_id_lo[553]=4'h0; text_id_lo[554]=4'h0; text_id_lo[555]=4'h0; text_id_lo[556]=4'h0; text_id_lo[557]=4'h0; text_id_lo[558]=4'h0; text_id_lo[559]=4'h0;
text_id_lo[560]=4'h0; text_id_lo[561]=4'h0; text_id_lo[562]=4'h0; text_id_lo[563]=4'h0; text_id_lo[564]=4'h0; text_id_lo[565]=4'h0; text_id_lo[566]=4'h0; text_id_lo[567]=4'h0;
text_id_lo[568]=4'h0; text_id_lo[569]=4'h0; text_id_lo[570]=4'h0; text_id_lo[571]=4'h0; text_id_lo[572]=4'h0; text_id_lo[573]=4'h0; text_id_lo[574]=4'h0; text_id_lo[575]=4'h0;
text_id_lo[576]=4'h0; text_id_lo[577]=4'h0; text_id_lo[578]=4'h0; text_id_lo[579]=4'h0; text_id_lo[580]=4'h0; text_id_lo[581]=4'h0; text_id_lo[582]=4'h0; text_id_lo[583]=4'h0;
text_id_lo[584]=4'h0; text_id_lo[585]=4'h0; text_id_lo[586]=4'h0; text_id_lo[587]=4'h0; text_id_lo[588]=4'h0; text_id_lo[589]=4'h0; text_id_lo[590]=4'h0; text_id_lo[591]=4'h0;
text_id_lo[592]=4'h0; text_id_lo[593]=4'h0; text_id_lo[594]=4'h0; text_id_lo[595]=4'h0; text_id_lo[596]=4'h0; text_id_lo[597]=4'h0; text_id_lo[598]=4'h0; text_id_lo[599]=4'h0;
text_id_lo[600]=4'h0; text_id_lo[601]=4'h0; text_id_lo[602]=4'h0; text_id_lo[603]=4'h0; text_id_lo[604]=4'h0; text_id_lo[605]=4'h0; text_id_lo[606]=4'h0; text_id_lo[607]=4'h0;
text_id_lo[608]=4'h0; text_id_lo[609]=4'h0; text_id_lo[610]=4'h0; text_id_lo[611]=4'h0; text_id_lo[612]=4'h0; text_id_lo[613]=4'h0; text_id_lo[614]=4'h0; text_id_lo[615]=4'h0;
text_id_lo[616]=4'h0; text_id_lo[617]=4'h0; text_id_lo[618]=4'h0; text_id_lo[619]=4'h0; text_id_lo[620]=4'h0; text_id_lo[621]=4'h0; text_id_lo[622]=4'h0; text_id_lo[623]=4'h0;
text_id_lo[624]=4'h0; text_id_lo[625]=4'h0; text_id_lo[626]=4'h0; text_id_lo[627]=4'h0; text_id_lo[628]=4'h0; text_id_lo[629]=4'h0; text_id_lo[630]=4'h0; text_id_lo[631]=4'h0;
text_id_lo[632]=4'h0; text_id_lo[633]=4'h0; text_id_lo[634]=4'h0; text_id_lo[635]=4'h0; text_id_lo[636]=4'h0; text_id_lo[637]=4'h0; text_id_lo[638]=4'h0; text_id_lo[639]=4'h0;
text_id_lo[640]=4'hb; text_id_lo[641]=4'hb; text_id_lo[642]=4'h2; text_id_lo[643]=4'h5; text_id_lo[644]=4'h9; text_id_lo[645]=4'h5; text_id_lo[646]=4'hf; text_id_lo[647]=4'h4;
text_id_lo[648]=4'h4; text_id_lo[649]=4'hc; text_id_lo[650]=4'hd; text_id_lo[651]=4'h0; text_id_lo[652]=4'h8; text_id_lo[653]=4'he; text_id_lo[654]=4'h1; text_id_lo[655]=4'h3;
text_id_lo[656]=4'he; text_id_lo[657]=4'h9; text_id_lo[658]=4'h8; text_id_lo[659]=4'h5; text_id_lo[660]=4'hf; text_id_lo[661]=4'h5; text_id_lo[662]=4'h4; text_id_lo[663]=4'h5;
text_id_lo[664]=4'h8; text_id_lo[665]=4'h7; text_id_lo[666]=4'h5; text_id_lo[667]=4'hc; text_id_lo[668]=4'h6; text_id_lo[669]=4'h0; text_id_lo[670]=4'h0; text_id_lo[671]=4'h0;
text_id_lo[672]=4'h0; text_id_lo[673]=4'h0; text_id_lo[674]=4'h0; text_id_lo[675]=4'h0; text_id_lo[676]=4'h0; text_id_lo[677]=4'h0; text_id_lo[678]=4'h0; text_id_lo[679]=4'h0;
text_id_lo[680]=4'h0; text_id_lo[681]=4'h0; text_id_lo[682]=4'h0; text_id_lo[683]=4'h0; text_id_lo[684]=4'h0; text_id_lo[685]=4'h0; text_id_lo[686]=4'h0; text_id_lo[687]=4'h0;
text_id_lo[688]=4'h0; text_id_lo[689]=4'h0; text_id_lo[690]=4'h0; text_id_lo[691]=4'h0; text_id_lo[692]=4'h0; text_id_lo[693]=4'h0; text_id_lo[694]=4'h0; text_id_lo[695]=4'h0;
text_id_lo[696]=4'h0; text_id_lo[697]=4'h0; text_id_lo[698]=4'h0; text_id_lo[699]=4'h0; text_id_lo[700]=4'h0; text_id_lo[701]=4'h0; text_id_lo[702]=4'h0; text_id_lo[703]=4'h0;
text_id_lo[704]=4'h0; text_id_lo[705]=4'h0; text_id_lo[706]=4'h0; text_id_lo[707]=4'h0; text_id_lo[708]=4'h0; text_id_lo[709]=4'h0; text_id_lo[710]=4'h0; text_id_lo[711]=4'h0;
text_id_lo[712]=4'h0; text_id_lo[713]=4'h0; text_id_lo[714]=4'h0; text_id_lo[715]=4'h0; text_id_lo[716]=4'h0; text_id_lo[717]=4'h0; text_id_lo[718]=4'h0; text_id_lo[719]=4'h0;
text_id_lo[720]=4'h0; text_id_lo[721]=4'h0; text_id_lo[722]=4'h0; text_id_lo[723]=4'h0; text_id_lo[724]=4'h0; text_id_lo[725]=4'h0; text_id_lo[726]=4'h0; text_id_lo[727]=4'h0;
text_id_lo[728]=4'h0; text_id_lo[729]=4'h0; text_id_lo[730]=4'h0; text_id_lo[731]=4'h0; text_id_lo[732]=4'h0; text_id_lo[733]=4'h0; text_id_lo[734]=4'h0; text_id_lo[735]=4'h0;
text_id_lo[736]=4'h0; text_id_lo[737]=4'h0; text_id_lo[738]=4'h0; text_id_lo[739]=4'h0; text_id_lo[740]=4'h0; text_id_lo[741]=4'h0; text_id_lo[742]=4'h0; text_id_lo[743]=4'h0;
text_id_lo[744]=4'h0; text_id_lo[745]=4'h0; text_id_lo[746]=4'h0; text_id_lo[747]=4'h0; text_id_lo[748]=4'h0; text_id_lo[749]=4'h0; text_id_lo[750]=4'h0; text_id_lo[751]=4'h0;
text_id_lo[752]=4'h0; text_id_lo[753]=4'h0; text_id_lo[754]=4'h0; text_id_lo[755]=4'h0; text_id_lo[756]=4'h0; text_id_lo[757]=4'h0; text_id_lo[758]=4'h0; text_id_lo[759]=4'h0;
text_id_lo[760]=4'h0; text_id_lo[761]=4'h0; text_id_lo[762]=4'h0; text_id_lo[763]=4'h0; text_id_lo[764]=4'h0; text_id_lo[765]=4'h0; text_id_lo[766]=4'h0; text_id_lo[767]=4'h0;
text_id_lo[768]=4'h2; text_id_lo[769]=4'h7; text_id_lo[770]=4'ha; text_id_lo[771]=4'h4; text_id_lo[772]=4'hf; text_id_lo[773]=4'ha; text_id_lo[774]=4'hb; text_id_lo[775]=4'h0;
text_id_lo[776]=4'h8; text_id_lo[777]=4'h6; text_id_lo[778]=4'ha; text_id_lo[779]=4'hc; text_id_lo[780]=4'h5; text_id_lo[781]=4'hc; text_id_lo[782]=4'he; text_id_lo[783]=4'h4;
text_id_lo[784]=4'hf; text_id_lo[785]=4'ha; text_id_lo[786]=4'h4; text_id_lo[787]=4'hf; text_id_lo[788]=4'ha; text_id_lo[789]=4'hd; text_id_lo[790]=4'h5; text_id_lo[791]=4'h8;
text_id_lo[792]=4'hb; text_id_lo[793]=4'h1; text_id_lo[794]=4'h7; text_id_lo[795]=4'ha; text_id_lo[796]=4'hd; text_id_lo[797]=4'h6; text_id_lo[798]=4'h5; text_id_lo[799]=4'he;
text_id_lo[800]=4'h7; text_id_lo[801]=4'h9; text_id_lo[802]=4'h4; text_id_lo[803]=4'h4; text_id_lo[804]=4'he; text_id_lo[805]=4'h6; text_id_lo[806]=4'hb; text_id_lo[807]=4'h1;
text_id_lo[808]=4'hc; text_id_lo[809]=4'hb; text_id_lo[810]=4'h6; text_id_lo[811]=4'h1; text_id_lo[812]=4'h6; text_id_lo[813]=4'h0; text_id_lo[814]=4'h0; text_id_lo[815]=4'h0;
text_id_lo[816]=4'h0; text_id_lo[817]=4'h0; text_id_lo[818]=4'h0; text_id_lo[819]=4'h0; text_id_lo[820]=4'h0; text_id_lo[821]=4'h0; text_id_lo[822]=4'h0; text_id_lo[823]=4'h0;
text_id_lo[824]=4'h0; text_id_lo[825]=4'h0; text_id_lo[826]=4'h0; text_id_lo[827]=4'h0; text_id_lo[828]=4'h0; text_id_lo[829]=4'h0; text_id_lo[830]=4'h0; text_id_lo[831]=4'h0;
text_id_lo[832]=4'h0; text_id_lo[833]=4'h0; text_id_lo[834]=4'h0; text_id_lo[835]=4'h0; text_id_lo[836]=4'h0; text_id_lo[837]=4'h0; text_id_lo[838]=4'h0; text_id_lo[839]=4'h0;
text_id_lo[840]=4'h0; text_id_lo[841]=4'h0; text_id_lo[842]=4'h0; text_id_lo[843]=4'h0; text_id_lo[844]=4'h0; text_id_lo[845]=4'h0; text_id_lo[846]=4'h0; text_id_lo[847]=4'h0;
text_id_lo[848]=4'h0; text_id_lo[849]=4'h0; text_id_lo[850]=4'h0; text_id_lo[851]=4'h0; text_id_lo[852]=4'h0; text_id_lo[853]=4'h0; text_id_lo[854]=4'h0; text_id_lo[855]=4'h0;
text_id_lo[856]=4'h0; text_id_lo[857]=4'h0; text_id_lo[858]=4'h0; text_id_lo[859]=4'h0; text_id_lo[860]=4'h0; text_id_lo[861]=4'h0; text_id_lo[862]=4'h0; text_id_lo[863]=4'h0;
text_id_lo[864]=4'h0; text_id_lo[865]=4'h0; text_id_lo[866]=4'h0; text_id_lo[867]=4'h0; text_id_lo[868]=4'h0; text_id_lo[869]=4'h0; text_id_lo[870]=4'h0; text_id_lo[871]=4'h0;
text_id_lo[872]=4'h0; text_id_lo[873]=4'h0; text_id_lo[874]=4'h0; text_id_lo[875]=4'h0; text_id_lo[876]=4'h0; text_id_lo[877]=4'h0; text_id_lo[878]=4'h0; text_id_lo[879]=4'h0;
text_id_lo[880]=4'h0; text_id_lo[881]=4'h0; text_id_lo[882]=4'h0; text_id_lo[883]=4'h0; text_id_lo[884]=4'h0; text_id_lo[885]=4'h0; text_id_lo[886]=4'h0; text_id_lo[887]=4'h0;
text_id_lo[888]=4'h0; text_id_lo[889]=4'h0; text_id_lo[890]=4'h0; text_id_lo[891]=4'h0; text_id_lo[892]=4'h0; text_id_lo[893]=4'h0; text_id_lo[894]=4'h0; text_id_lo[895]=4'h0;
text_id_lo[896]=4'h0; text_id_lo[897]=4'h1; text_id_lo[898]=4'h3; text_id_lo[899]=4'hc; text_id_lo[900]=4'h9; text_id_lo[901]=4'h4; text_id_lo[902]=4'hb; text_id_lo[903]=4'hb;
text_id_lo[904]=4'h1; text_id_lo[905]=4'h5; text_id_lo[906]=4'hf; text_id_lo[907]=4'h5; text_id_lo[908]=4'h8; text_id_lo[909]=4'h2; text_id_lo[910]=4'he; text_id_lo[911]=4'h4;
text_id_lo[912]=4'h7; text_id_lo[913]=4'h3; text_id_lo[914]=4'h9; text_id_lo[915]=4'h0; text_id_lo[916]=4'hb; text_id_lo[917]=4'h2; text_id_lo[918]=4'h5; text_id_lo[919]=4'ha;
text_id_lo[920]=4'h5; text_id_lo[921]=4'h8; text_id_lo[922]=4'ha; text_id_lo[923]=4'h5; text_id_lo[924]=4'h3; text_id_lo[925]=4'hc; text_id_lo[926]=4'h7; text_id_lo[927]=4'hb;
text_id_lo[928]=4'h1; text_id_lo[929]=4'h9; text_id_lo[930]=4'h7; text_id_lo[931]=4'h6; text_id_lo[932]=4'h0; text_id_lo[933]=4'h0; text_id_lo[934]=4'h0; text_id_lo[935]=4'h0;
text_id_lo[936]=4'h0; text_id_lo[937]=4'h0; text_id_lo[938]=4'h0; text_id_lo[939]=4'h0; text_id_lo[940]=4'h0; text_id_lo[941]=4'h0; text_id_lo[942]=4'h0; text_id_lo[943]=4'h0;
text_id_lo[944]=4'h0; text_id_lo[945]=4'h0; text_id_lo[946]=4'h0; text_id_lo[947]=4'h0; text_id_lo[948]=4'h0; text_id_lo[949]=4'h0; text_id_lo[950]=4'h0; text_id_lo[951]=4'h0;
text_id_lo[952]=4'h0; text_id_lo[953]=4'h0; text_id_lo[954]=4'h0; text_id_lo[955]=4'h0; text_id_lo[956]=4'h0; text_id_lo[957]=4'h0; text_id_lo[958]=4'h0; text_id_lo[959]=4'h0;
text_id_lo[960]=4'h0; text_id_lo[961]=4'h0; text_id_lo[962]=4'h0; text_id_lo[963]=4'h0; text_id_lo[964]=4'h0; text_id_lo[965]=4'h0; text_id_lo[966]=4'h0; text_id_lo[967]=4'h0;
text_id_lo[968]=4'h0; text_id_lo[969]=4'h0; text_id_lo[970]=4'h0; text_id_lo[971]=4'h0; text_id_lo[972]=4'h0; text_id_lo[973]=4'h0; text_id_lo[974]=4'h0; text_id_lo[975]=4'h0;
text_id_lo[976]=4'h0; text_id_lo[977]=4'h0; text_id_lo[978]=4'h0; text_id_lo[979]=4'h0; text_id_lo[980]=4'h0; text_id_lo[981]=4'h0; text_id_lo[982]=4'h0; text_id_lo[983]=4'h0;
text_id_lo[984]=4'h0; text_id_lo[985]=4'h0; text_id_lo[986]=4'h0; text_id_lo[987]=4'h0; text_id_lo[988]=4'h0; text_id_lo[989]=4'h0; text_id_lo[990]=4'h0; text_id_lo[991]=4'h0;
text_id_lo[992]=4'h0; text_id_lo[993]=4'h0; text_id_lo[994]=4'h0; text_id_lo[995]=4'h0; text_id_lo[996]=4'h0; text_id_lo[997]=4'h0; text_id_lo[998]=4'h0; text_id_lo[999]=4'h0;
text_id_lo[1000]=4'h0; text_id_lo[1001]=4'h0; text_id_lo[1002]=4'h0; text_id_lo[1003]=4'h0; text_id_lo[1004]=4'h0; text_id_lo[1005]=4'h0; text_id_lo[1006]=4'h0; text_id_lo[1007]=4'h0;
text_id_lo[1008]=4'h0; text_id_lo[1009]=4'h0; text_id_lo[1010]=4'h0; text_id_lo[1011]=4'h0; text_id_lo[1012]=4'h0; text_id_lo[1013]=4'h0; text_id_lo[1014]=4'h0; text_id_lo[1015]=4'h0;
text_id_lo[1016]=4'h0; text_id_lo[1017]=4'h0; text_id_lo[1018]=4'h0; text_id_lo[1019]=4'h0; text_id_lo[1020]=4'h0; text_id_lo[1021]=4'h0; text_id_lo[1022]=4'h0; text_id_lo[1023]=4'h0;
text_id_lo[1024]=4'h2; text_id_lo[1025]=4'hf; text_id_lo[1026]=4'hd; text_id_lo[1027]=4'h8; text_id_lo[1028]=4'h9; text_id_lo[1029]=4'hc; text_id_lo[1030]=4'ha; text_id_lo[1031]=4'he;
text_id_lo[1032]=4'hd; text_id_lo[1033]=4'h3; text_id_lo[1034]=4'hd; text_id_lo[1035]=4'hd; text_id_lo[1036]=4'h7; text_id_lo[1037]=4'h3; text_id_lo[1038]=4'h2; text_id_lo[1039]=4'h3;
text_id_lo[1040]=4'h6; text_id_lo[1041]=4'h0; text_id_lo[1042]=4'he; text_id_lo[1043]=4'h3; text_id_lo[1044]=4'h4; text_id_lo[1045]=4'hb; text_id_lo[1046]=4'h1; text_id_lo[1047]=4'h6;
text_id_lo[1048]=4'h5; text_id_lo[1049]=4'h0; text_id_lo[1050]=4'h2; text_id_lo[1051]=4'h8; text_id_lo[1052]=4'h0; text_id_lo[1053]=4'he; text_id_lo[1054]=4'h9; text_id_lo[1055]=4'h3;
text_id_lo[1056]=4'ha; text_id_lo[1057]=4'h3; text_id_lo[1058]=4'hf; text_id_lo[1059]=4'ha; text_id_lo[1060]=4'hd; text_id_lo[1061]=4'h1; text_id_lo[1062]=4'hb; text_id_lo[1063]=4'ha;
text_id_lo[1064]=4'hd; text_id_lo[1065]=4'h9; text_id_lo[1066]=4'h0; text_id_lo[1067]=4'h6; text_id_lo[1068]=4'h0; text_id_lo[1069]=4'h0; text_id_lo[1070]=4'h0; text_id_lo[1071]=4'h0;
text_id_lo[1072]=4'h0; text_id_lo[1073]=4'h0; text_id_lo[1074]=4'h0; text_id_lo[1075]=4'h0; text_id_lo[1076]=4'h0; text_id_lo[1077]=4'h0; text_id_lo[1078]=4'h0; text_id_lo[1079]=4'h0;
text_id_lo[1080]=4'h0; text_id_lo[1081]=4'h0; text_id_lo[1082]=4'h0; text_id_lo[1083]=4'h0; text_id_lo[1084]=4'h0; text_id_lo[1085]=4'h0; text_id_lo[1086]=4'h0; text_id_lo[1087]=4'h0;
text_id_lo[1088]=4'h0; text_id_lo[1089]=4'h0; text_id_lo[1090]=4'h0; text_id_lo[1091]=4'h0; text_id_lo[1092]=4'h0; text_id_lo[1093]=4'h0; text_id_lo[1094]=4'h0; text_id_lo[1095]=4'h0;
text_id_lo[1096]=4'h0; text_id_lo[1097]=4'h0; text_id_lo[1098]=4'h0; text_id_lo[1099]=4'h0; text_id_lo[1100]=4'h0; text_id_lo[1101]=4'h0; text_id_lo[1102]=4'h0; text_id_lo[1103]=4'h0;
text_id_lo[1104]=4'h0; text_id_lo[1105]=4'h0; text_id_lo[1106]=4'h0; text_id_lo[1107]=4'h0; text_id_lo[1108]=4'h0; text_id_lo[1109]=4'h0; text_id_lo[1110]=4'h0; text_id_lo[1111]=4'h0;
text_id_lo[1112]=4'h0; text_id_lo[1113]=4'h0; text_id_lo[1114]=4'h0; text_id_lo[1115]=4'h0; text_id_lo[1116]=4'h0; text_id_lo[1117]=4'h0; text_id_lo[1118]=4'h0; text_id_lo[1119]=4'h0;
text_id_lo[1120]=4'h0; text_id_lo[1121]=4'h0; text_id_lo[1122]=4'h0; text_id_lo[1123]=4'h0; text_id_lo[1124]=4'h0; text_id_lo[1125]=4'h0; text_id_lo[1126]=4'h0; text_id_lo[1127]=4'h0;
text_id_lo[1128]=4'h0; text_id_lo[1129]=4'h0; text_id_lo[1130]=4'h0; text_id_lo[1131]=4'h0; text_id_lo[1132]=4'h0; text_id_lo[1133]=4'h0; text_id_lo[1134]=4'h0; text_id_lo[1135]=4'h0;
text_id_lo[1136]=4'h0; text_id_lo[1137]=4'h0; text_id_lo[1138]=4'h0; text_id_lo[1139]=4'h0; text_id_lo[1140]=4'h0; text_id_lo[1141]=4'h0; text_id_lo[1142]=4'h0; text_id_lo[1143]=4'h0;
text_id_lo[1144]=4'h0; text_id_lo[1145]=4'h0; text_id_lo[1146]=4'h0; text_id_lo[1147]=4'h0; text_id_lo[1148]=4'h0; text_id_lo[1149]=4'h0; text_id_lo[1150]=4'h0; text_id_lo[1151]=4'h0;
text_id_lo[1152]=4'hc; text_id_lo[1153]=4'h2; text_id_lo[1154]=4'h2; text_id_lo[1155]=4'ha; text_id_lo[1156]=4'h1; text_id_lo[1157]=4'h2; text_id_lo[1158]=4'hc; text_id_lo[1159]=4'h0;
text_id_lo[1160]=4'h6; text_id_lo[1161]=4'h2; text_id_lo[1162]=4'hc; text_id_lo[1163]=4'h7; text_id_lo[1164]=4'hb; text_id_lo[1165]=4'hd; text_id_lo[1166]=4'hf; text_id_lo[1167]=4'hb;
text_id_lo[1168]=4'h1; text_id_lo[1169]=4'he; text_id_lo[1170]=4'hc; text_id_lo[1171]=4'h6; text_id_lo[1172]=4'ha; text_id_lo[1173]=4'he; text_id_lo[1174]=4'hb; text_id_lo[1175]=4'h1;
text_id_lo[1176]=4'hf; text_id_lo[1177]=4'hc; text_id_lo[1178]=4'hf; text_id_lo[1179]=4'h3; text_id_lo[1180]=4'he; text_id_lo[1181]=4'hb; text_id_lo[1182]=4'hf; text_id_lo[1183]=4'h8;
text_id_lo[1184]=4'hb; text_id_lo[1185]=4'h1; text_id_lo[1186]=4'h1; text_id_lo[1187]=4'h0; text_id_lo[1188]=4'hb; text_id_lo[1189]=4'h7; text_id_lo[1190]=4'h0; text_id_lo[1191]=4'h8;
text_id_lo[1192]=4'h5; text_id_lo[1193]=4'h7; text_id_lo[1194]=4'hb; text_id_lo[1195]=4'he; text_id_lo[1196]=4'h1; text_id_lo[1197]=4'h6; text_id_lo[1198]=4'hc; text_id_lo[1199]=4'h4;
text_id_lo[1200]=4'hb; text_id_lo[1201]=4'h1; text_id_lo[1202]=4'h7; text_id_lo[1203]=4'h5; text_id_lo[1204]=4'hf; text_id_lo[1205]=4'h6; text_id_lo[1206]=4'h0; text_id_lo[1207]=4'h0;
text_id_lo[1208]=4'h0; text_id_lo[1209]=4'h0; text_id_lo[1210]=4'h0; text_id_lo[1211]=4'h0; text_id_lo[1212]=4'h0; text_id_lo[1213]=4'h0; text_id_lo[1214]=4'h0; text_id_lo[1215]=4'h0;
text_id_lo[1216]=4'h0; text_id_lo[1217]=4'h0; text_id_lo[1218]=4'h0; text_id_lo[1219]=4'h0; text_id_lo[1220]=4'h0; text_id_lo[1221]=4'h0; text_id_lo[1222]=4'h0; text_id_lo[1223]=4'h0;
text_id_lo[1224]=4'h0; text_id_lo[1225]=4'h0; text_id_lo[1226]=4'h0; text_id_lo[1227]=4'h0; text_id_lo[1228]=4'h0; text_id_lo[1229]=4'h0; text_id_lo[1230]=4'h0; text_id_lo[1231]=4'h0;
text_id_lo[1232]=4'h0; text_id_lo[1233]=4'h0; text_id_lo[1234]=4'h0; text_id_lo[1235]=4'h0; text_id_lo[1236]=4'h0; text_id_lo[1237]=4'h0; text_id_lo[1238]=4'h0; text_id_lo[1239]=4'h0;
text_id_lo[1240]=4'h0; text_id_lo[1241]=4'h0; text_id_lo[1242]=4'h0; text_id_lo[1243]=4'h0; text_id_lo[1244]=4'h0; text_id_lo[1245]=4'h0; text_id_lo[1246]=4'h0; text_id_lo[1247]=4'h0;
text_id_lo[1248]=4'h0; text_id_lo[1249]=4'h0; text_id_lo[1250]=4'h0; text_id_lo[1251]=4'h0; text_id_lo[1252]=4'h0; text_id_lo[1253]=4'h0; text_id_lo[1254]=4'h0; text_id_lo[1255]=4'h0;
text_id_lo[1256]=4'h0; text_id_lo[1257]=4'h0; text_id_lo[1258]=4'h0; text_id_lo[1259]=4'h0; text_id_lo[1260]=4'h0; text_id_lo[1261]=4'h0; text_id_lo[1262]=4'h0; text_id_lo[1263]=4'h0;
text_id_lo[1264]=4'h0; text_id_lo[1265]=4'h0; text_id_lo[1266]=4'h0; text_id_lo[1267]=4'h0; text_id_lo[1268]=4'h0; text_id_lo[1269]=4'h0; text_id_lo[1270]=4'h0; text_id_lo[1271]=4'h0;
text_id_lo[1272]=4'h0; text_id_lo[1273]=4'h0; text_id_lo[1274]=4'h0; text_id_lo[1275]=4'h0; text_id_lo[1276]=4'h0; text_id_lo[1277]=4'h0; text_id_lo[1278]=4'h0; text_id_lo[1279]=4'h0;
text_id_lo[1280]=4'h1; text_id_lo[1281]=4'he; text_id_lo[1282]=4'h2; text_id_lo[1283]=4'h9; text_id_lo[1284]=4'h6; text_id_lo[1285]=4'h9; text_id_lo[1286]=4'ha; text_id_lo[1287]=4'h0;
text_id_lo[1288]=4'hb; text_id_lo[1289]=4'h1; text_id_lo[1290]=4'hf; text_id_lo[1291]=4'h2; text_id_lo[1292]=4'hd; text_id_lo[1293]=4'he; text_id_lo[1294]=4'h7; text_id_lo[1295]=4'h8;
text_id_lo[1296]=4'h6; text_id_lo[1297]=4'hd; text_id_lo[1298]=4'h3; text_id_lo[1299]=4'hc; text_id_lo[1300]=4'h9; text_id_lo[1301]=4'h7; text_id_lo[1302]=4'h7; text_id_lo[1303]=4'he;
text_id_lo[1304]=4'ha; text_id_lo[1305]=4'h1; text_id_lo[1306]=4'ha; text_id_lo[1307]=4'h3; text_id_lo[1308]=4'h8; text_id_lo[1309]=4'h0; text_id_lo[1310]=4'ha; text_id_lo[1311]=4'h8;
text_id_lo[1312]=4'h4; text_id_lo[1313]=4'h2; text_id_lo[1314]=4'hc; text_id_lo[1315]=4'hd; text_id_lo[1316]=4'he; text_id_lo[1317]=4'he; text_id_lo[1318]=4'h1; text_id_lo[1319]=4'h9;
text_id_lo[1320]=4'h9; text_id_lo[1321]=4'h2; text_id_lo[1322]=4'h0; text_id_lo[1323]=4'h2; text_id_lo[1324]=4'ha; text_id_lo[1325]=4'he; text_id_lo[1326]=4'h0; text_id_lo[1327]=4'ha;
text_id_lo[1328]=4'h7; text_id_lo[1329]=4'ha; text_id_lo[1330]=4'h3; text_id_lo[1331]=4'h8; text_id_lo[1332]=4'h5; text_id_lo[1333]=4'hb; text_id_lo[1334]=4'h6; text_id_lo[1335]=4'h0;
text_id_lo[1336]=4'h0; text_id_lo[1337]=4'h0; text_id_lo[1338]=4'h0; text_id_lo[1339]=4'h0; text_id_lo[1340]=4'h0; text_id_lo[1341]=4'h0; text_id_lo[1342]=4'h0; text_id_lo[1343]=4'h0;
text_id_lo[1344]=4'h0; text_id_lo[1345]=4'h0; text_id_lo[1346]=4'h0; text_id_lo[1347]=4'h0; text_id_lo[1348]=4'h0; text_id_lo[1349]=4'h0; text_id_lo[1350]=4'h0; text_id_lo[1351]=4'h0;
text_id_lo[1352]=4'h0; text_id_lo[1353]=4'h0; text_id_lo[1354]=4'h0; text_id_lo[1355]=4'h0; text_id_lo[1356]=4'h0; text_id_lo[1357]=4'h0; text_id_lo[1358]=4'h0; text_id_lo[1359]=4'h0;
text_id_lo[1360]=4'h0; text_id_lo[1361]=4'h0; text_id_lo[1362]=4'h0; text_id_lo[1363]=4'h0; text_id_lo[1364]=4'h0; text_id_lo[1365]=4'h0; text_id_lo[1366]=4'h0; text_id_lo[1367]=4'h0;
text_id_lo[1368]=4'h0; text_id_lo[1369]=4'h0; text_id_lo[1370]=4'h0; text_id_lo[1371]=4'h0; text_id_lo[1372]=4'h0; text_id_lo[1373]=4'h0; text_id_lo[1374]=4'h0; text_id_lo[1375]=4'h0;
text_id_lo[1376]=4'h0; text_id_lo[1377]=4'h0; text_id_lo[1378]=4'h0; text_id_lo[1379]=4'h0; text_id_lo[1380]=4'h0; text_id_lo[1381]=4'h0; text_id_lo[1382]=4'h0; text_id_lo[1383]=4'h0;
text_id_lo[1384]=4'h0; text_id_lo[1385]=4'h0; text_id_lo[1386]=4'h0; text_id_lo[1387]=4'h0; text_id_lo[1388]=4'h0; text_id_lo[1389]=4'h0; text_id_lo[1390]=4'h0; text_id_lo[1391]=4'h0;
text_id_lo[1392]=4'h0; text_id_lo[1393]=4'h0; text_id_lo[1394]=4'h0; text_id_lo[1395]=4'h0; text_id_lo[1396]=4'h0; text_id_lo[1397]=4'h0; text_id_lo[1398]=4'h0; text_id_lo[1399]=4'h0;
text_id_lo[1400]=4'h0; text_id_lo[1401]=4'h0; text_id_lo[1402]=4'h0; text_id_lo[1403]=4'h0; text_id_lo[1404]=4'h0; text_id_lo[1405]=4'h0; text_id_lo[1406]=4'h0; text_id_lo[1407]=4'h0;
text_id_lo[1408]=4'h7; text_id_lo[1409]=4'hc; text_id_lo[1410]=4'h2; text_id_lo[1411]=4'he; text_id_lo[1412]=4'h2; text_id_lo[1413]=4'hf; text_id_lo[1414]=4'hb; text_id_lo[1415]=4'h1;
text_id_lo[1416]=4'h3; text_id_lo[1417]=4'he; text_id_lo[1418]=4'h2; text_id_lo[1419]=4'h4; text_id_lo[1420]=4'ha; text_id_lo[1421]=4'hb; text_id_lo[1422]=4'h1; text_id_lo[1423]=4'h4;
text_id_lo[1424]=4'h5; text_id_lo[1425]=4'hb; text_id_lo[1426]=4'h1; text_id_lo[1427]=4'h5; text_id_lo[1428]=4'h5; text_id_lo[1429]=4'hb; text_id_lo[1430]=4'h1; text_id_lo[1431]=4'h2;
text_id_lo[1432]=4'he; text_id_lo[1433]=4'hb; text_id_lo[1434]=4'h1; text_id_lo[1435]=4'h5; text_id_lo[1436]=4'h1; text_id_lo[1437]=4'h0; text_id_lo[1438]=4'h6; text_id_lo[1439]=4'h1;
text_id_lo[1440]=4'h3; text_id_lo[1441]=4'hd; text_id_lo[1442]=4'h5; text_id_lo[1443]=4'h8; text_id_lo[1444]=4'h2; text_id_lo[1445]=4'hc; text_id_lo[1446]=4'h5; text_id_lo[1447]=4'h9;
text_id_lo[1448]=4'he; text_id_lo[1449]=4'h1; text_id_lo[1450]=4'h4; text_id_lo[1451]=4'he; text_id_lo[1452]=4'h4; text_id_lo[1453]=4'hd; text_id_lo[1454]=4'he; text_id_lo[1455]=4'h3;
text_id_lo[1456]=4'h6; text_id_lo[1457]=4'h2; text_id_lo[1458]=4'he; text_id_lo[1459]=4'h6; text_id_lo[1460]=4'h0; text_id_lo[1461]=4'h0; text_id_lo[1462]=4'h0; text_id_lo[1463]=4'h0;
text_id_lo[1464]=4'h0; text_id_lo[1465]=4'h0; text_id_lo[1466]=4'h0; text_id_lo[1467]=4'h0; text_id_lo[1468]=4'h0; text_id_lo[1469]=4'h0; text_id_lo[1470]=4'h0; text_id_lo[1471]=4'h0;
text_id_lo[1472]=4'h0; text_id_lo[1473]=4'h0; text_id_lo[1474]=4'h0; text_id_lo[1475]=4'h0; text_id_lo[1476]=4'h0; text_id_lo[1477]=4'h0; text_id_lo[1478]=4'h0; text_id_lo[1479]=4'h0;
text_id_lo[1480]=4'h0; text_id_lo[1481]=4'h0; text_id_lo[1482]=4'h0; text_id_lo[1483]=4'h0; text_id_lo[1484]=4'h0; text_id_lo[1485]=4'h0; text_id_lo[1486]=4'h0; text_id_lo[1487]=4'h0;
text_id_lo[1488]=4'h0; text_id_lo[1489]=4'h0; text_id_lo[1490]=4'h0; text_id_lo[1491]=4'h0; text_id_lo[1492]=4'h0; text_id_lo[1493]=4'h0; text_id_lo[1494]=4'h0; text_id_lo[1495]=4'h0;
text_id_lo[1496]=4'h0; text_id_lo[1497]=4'h0; text_id_lo[1498]=4'h0; text_id_lo[1499]=4'h0; text_id_lo[1500]=4'h0; text_id_lo[1501]=4'h0; text_id_lo[1502]=4'h0; text_id_lo[1503]=4'h0;
text_id_lo[1504]=4'h0; text_id_lo[1505]=4'h0; text_id_lo[1506]=4'h0; text_id_lo[1507]=4'h0; text_id_lo[1508]=4'h0; text_id_lo[1509]=4'h0; text_id_lo[1510]=4'h0; text_id_lo[1511]=4'h0;
text_id_lo[1512]=4'h0; text_id_lo[1513]=4'h0; text_id_lo[1514]=4'h0; text_id_lo[1515]=4'h0; text_id_lo[1516]=4'h0; text_id_lo[1517]=4'h0; text_id_lo[1518]=4'h0; text_id_lo[1519]=4'h0;
text_id_lo[1520]=4'h0; text_id_lo[1521]=4'h0; text_id_lo[1522]=4'h0; text_id_lo[1523]=4'h0; text_id_lo[1524]=4'h0; text_id_lo[1525]=4'h0; text_id_lo[1526]=4'h0; text_id_lo[1527]=4'h0;
text_id_lo[1528]=4'h0; text_id_lo[1529]=4'h0; text_id_lo[1530]=4'h0; text_id_lo[1531]=4'h0; text_id_lo[1532]=4'h0; text_id_lo[1533]=4'h0; text_id_lo[1534]=4'h0; text_id_lo[1535]=4'h0;
text_id_lo[1536]=4'hc; text_id_lo[1537]=4'h7; text_id_lo[1538]=4'hf; text_id_lo[1539]=4'h5; text_id_lo[1540]=4'h2; text_id_lo[1541]=4'h5; text_id_lo[1542]=4'h6; text_id_lo[1543]=4'he;
text_id_lo[1544]=4'he; text_id_lo[1545]=4'hf; text_id_lo[1546]=4'hf; text_id_lo[1547]=4'h5; text_id_lo[1548]=4'h4; text_id_lo[1549]=4'h6; text_id_lo[1550]=4'h0; text_id_lo[1551]=4'h0;
text_id_lo[1552]=4'h0; text_id_lo[1553]=4'h0; text_id_lo[1554]=4'h0; text_id_lo[1555]=4'h0; text_id_lo[1556]=4'h0; text_id_lo[1557]=4'h0; text_id_lo[1558]=4'h0; text_id_lo[1559]=4'h0;
text_id_lo[1560]=4'h0; text_id_lo[1561]=4'h0; text_id_lo[1562]=4'h0; text_id_lo[1563]=4'h0; text_id_lo[1564]=4'h0; text_id_lo[1565]=4'h0; text_id_lo[1566]=4'h0; text_id_lo[1567]=4'h0;
text_id_lo[1568]=4'h0; text_id_lo[1569]=4'h0; text_id_lo[1570]=4'h0; text_id_lo[1571]=4'h0; text_id_lo[1572]=4'h0; text_id_lo[1573]=4'h0; text_id_lo[1574]=4'h0; text_id_lo[1575]=4'h0;
text_id_lo[1576]=4'h0; text_id_lo[1577]=4'h0; text_id_lo[1578]=4'h0; text_id_lo[1579]=4'h0; text_id_lo[1580]=4'h0; text_id_lo[1581]=4'h0; text_id_lo[1582]=4'h0; text_id_lo[1583]=4'h0;
text_id_lo[1584]=4'h0; text_id_lo[1585]=4'h0; text_id_lo[1586]=4'h0; text_id_lo[1587]=4'h0; text_id_lo[1588]=4'h0; text_id_lo[1589]=4'h0; text_id_lo[1590]=4'h0; text_id_lo[1591]=4'h0;
text_id_lo[1592]=4'h0; text_id_lo[1593]=4'h0; text_id_lo[1594]=4'h0; text_id_lo[1595]=4'h0; text_id_lo[1596]=4'h0; text_id_lo[1597]=4'h0; text_id_lo[1598]=4'h0; text_id_lo[1599]=4'h0;
text_id_lo[1600]=4'h0; text_id_lo[1601]=4'h0; text_id_lo[1602]=4'h0; text_id_lo[1603]=4'h0; text_id_lo[1604]=4'h0; text_id_lo[1605]=4'h0; text_id_lo[1606]=4'h0; text_id_lo[1607]=4'h0;
text_id_lo[1608]=4'h0; text_id_lo[1609]=4'h0; text_id_lo[1610]=4'h0; text_id_lo[1611]=4'h0; text_id_lo[1612]=4'h0; text_id_lo[1613]=4'h0; text_id_lo[1614]=4'h0; text_id_lo[1615]=4'h0;
text_id_lo[1616]=4'h0; text_id_lo[1617]=4'h0; text_id_lo[1618]=4'h0; text_id_lo[1619]=4'h0; text_id_lo[1620]=4'h0; text_id_lo[1621]=4'h0; text_id_lo[1622]=4'h0; text_id_lo[1623]=4'h0;
text_id_lo[1624]=4'h0; text_id_lo[1625]=4'h0; text_id_lo[1626]=4'h0; text_id_lo[1627]=4'h0; text_id_lo[1628]=4'h0; text_id_lo[1629]=4'h0; text_id_lo[1630]=4'h0; text_id_lo[1631]=4'h0;
text_id_lo[1632]=4'h0; text_id_lo[1633]=4'h0; text_id_lo[1634]=4'h0; text_id_lo[1635]=4'h0; text_id_lo[1636]=4'h0; text_id_lo[1637]=4'h0; text_id_lo[1638]=4'h0; text_id_lo[1639]=4'h0;
text_id_lo[1640]=4'h0; text_id_lo[1641]=4'h0; text_id_lo[1642]=4'h0; text_id_lo[1643]=4'h0; text_id_lo[1644]=4'h0; text_id_lo[1645]=4'h0; text_id_lo[1646]=4'h0; text_id_lo[1647]=4'h0;
text_id_lo[1648]=4'h0; text_id_lo[1649]=4'h0; text_id_lo[1650]=4'h0; text_id_lo[1651]=4'h0; text_id_lo[1652]=4'h0; text_id_lo[1653]=4'h0; text_id_lo[1654]=4'h0; text_id_lo[1655]=4'h0;
text_id_lo[1656]=4'h0; text_id_lo[1657]=4'h0; text_id_lo[1658]=4'h0; text_id_lo[1659]=4'h0; text_id_lo[1660]=4'h0; text_id_lo[1661]=4'h0; text_id_lo[1662]=4'h0; text_id_lo[1663]=4'h0;
text_id_lo[1664]=4'h0; text_id_lo[1665]=4'h0; text_id_lo[1666]=4'h0; text_id_lo[1667]=4'h0; text_id_lo[1668]=4'h0; text_id_lo[1669]=4'h0; text_id_lo[1670]=4'h0; text_id_lo[1671]=4'h0;
text_id_lo[1672]=4'h0; text_id_lo[1673]=4'h0; text_id_lo[1674]=4'h0; text_id_lo[1675]=4'h0; text_id_lo[1676]=4'h0; text_id_lo[1677]=4'h0; text_id_lo[1678]=4'h0; text_id_lo[1679]=4'h0;
text_id_lo[1680]=4'h0; text_id_lo[1681]=4'h0; text_id_lo[1682]=4'h0; text_id_lo[1683]=4'h0; text_id_lo[1684]=4'h0; text_id_lo[1685]=4'h0; text_id_lo[1686]=4'h0; text_id_lo[1687]=4'h0;
text_id_lo[1688]=4'h0; text_id_lo[1689]=4'h0; text_id_lo[1690]=4'h0; text_id_lo[1691]=4'h0; text_id_lo[1692]=4'h0; text_id_lo[1693]=4'h0; text_id_lo[1694]=4'h0; text_id_lo[1695]=4'h0;
text_id_lo[1696]=4'h0; text_id_lo[1697]=4'h0; text_id_lo[1698]=4'h0; text_id_lo[1699]=4'h0; text_id_lo[1700]=4'h0; text_id_lo[1701]=4'h0; text_id_lo[1702]=4'h0; text_id_lo[1703]=4'h0;
text_id_lo[1704]=4'h0; text_id_lo[1705]=4'h0; text_id_lo[1706]=4'h0; text_id_lo[1707]=4'h0; text_id_lo[1708]=4'h0; text_id_lo[1709]=4'h0; text_id_lo[1710]=4'h0; text_id_lo[1711]=4'h0;
text_id_lo[1712]=4'h0; text_id_lo[1713]=4'h0; text_id_lo[1714]=4'h0; text_id_lo[1715]=4'h0; text_id_lo[1716]=4'h0; text_id_lo[1717]=4'h0; text_id_lo[1718]=4'h0; text_id_lo[1719]=4'h0;
text_id_lo[1720]=4'h0; text_id_lo[1721]=4'h0; text_id_lo[1722]=4'h0; text_id_lo[1723]=4'h0; text_id_lo[1724]=4'h0; text_id_lo[1725]=4'h0; text_id_lo[1726]=4'h0; text_id_lo[1727]=4'h0;
text_id_lo[1728]=4'h0; text_id_lo[1729]=4'h0; text_id_lo[1730]=4'h0; text_id_lo[1731]=4'h0; text_id_lo[1732]=4'h0; text_id_lo[1733]=4'h0; text_id_lo[1734]=4'h0; text_id_lo[1735]=4'h0;
text_id_lo[1736]=4'h0; text_id_lo[1737]=4'h0; text_id_lo[1738]=4'h0; text_id_lo[1739]=4'h0; text_id_lo[1740]=4'h0; text_id_lo[1741]=4'h0; text_id_lo[1742]=4'h0; text_id_lo[1743]=4'h0;
text_id_lo[1744]=4'h0; text_id_lo[1745]=4'h0; text_id_lo[1746]=4'h0; text_id_lo[1747]=4'h0; text_id_lo[1748]=4'h0; text_id_lo[1749]=4'h0; text_id_lo[1750]=4'h0; text_id_lo[1751]=4'h0;
text_id_lo[1752]=4'h0; text_id_lo[1753]=4'h0; text_id_lo[1754]=4'h0; text_id_lo[1755]=4'h0; text_id_lo[1756]=4'h0; text_id_lo[1757]=4'h0; text_id_lo[1758]=4'h0; text_id_lo[1759]=4'h0;
text_id_lo[1760]=4'h0; text_id_lo[1761]=4'h0; text_id_lo[1762]=4'h0; text_id_lo[1763]=4'h0; text_id_lo[1764]=4'h0; text_id_lo[1765]=4'h0; text_id_lo[1766]=4'h0; text_id_lo[1767]=4'h0;
text_id_lo[1768]=4'h0; text_id_lo[1769]=4'h0; text_id_lo[1770]=4'h0; text_id_lo[1771]=4'h0; text_id_lo[1772]=4'h0; text_id_lo[1773]=4'h0; text_id_lo[1774]=4'h0; text_id_lo[1775]=4'h0;
text_id_lo[1776]=4'h0; text_id_lo[1777]=4'h0; text_id_lo[1778]=4'h0; text_id_lo[1779]=4'h0; text_id_lo[1780]=4'h0; text_id_lo[1781]=4'h0; text_id_lo[1782]=4'h0; text_id_lo[1783]=4'h0;
text_id_lo[1784]=4'h0; text_id_lo[1785]=4'h0; text_id_lo[1786]=4'h0; text_id_lo[1787]=4'h0; text_id_lo[1788]=4'h0; text_id_lo[1789]=4'h0; text_id_lo[1790]=4'h0; text_id_lo[1791]=4'h0;
text_id_lo[1792]=4'h0; text_id_lo[1793]=4'h0; text_id_lo[1794]=4'h0; text_id_lo[1795]=4'h0; text_id_lo[1796]=4'h0; text_id_lo[1797]=4'h0; text_id_lo[1798]=4'h0; text_id_lo[1799]=4'h0;
text_id_lo[1800]=4'h0; text_id_lo[1801]=4'h0; text_id_lo[1802]=4'h0; text_id_lo[1803]=4'h0; text_id_lo[1804]=4'h0; text_id_lo[1805]=4'h0; text_id_lo[1806]=4'h0; text_id_lo[1807]=4'h0;
text_id_lo[1808]=4'h0; text_id_lo[1809]=4'h0; text_id_lo[1810]=4'h0; text_id_lo[1811]=4'h0; text_id_lo[1812]=4'h0; text_id_lo[1813]=4'h0; text_id_lo[1814]=4'h0; text_id_lo[1815]=4'h0;
text_id_lo[1816]=4'h0; text_id_lo[1817]=4'h0; text_id_lo[1818]=4'h0; text_id_lo[1819]=4'h0; text_id_lo[1820]=4'h0; text_id_lo[1821]=4'h0; text_id_lo[1822]=4'h0; text_id_lo[1823]=4'h0;
text_id_lo[1824]=4'h0; text_id_lo[1825]=4'h0; text_id_lo[1826]=4'h0; text_id_lo[1827]=4'h0; text_id_lo[1828]=4'h0; text_id_lo[1829]=4'h0; text_id_lo[1830]=4'h0; text_id_lo[1831]=4'h0;
text_id_lo[1832]=4'h0; text_id_lo[1833]=4'h0; text_id_lo[1834]=4'h0; text_id_lo[1835]=4'h0; text_id_lo[1836]=4'h0; text_id_lo[1837]=4'h0; text_id_lo[1838]=4'h0; text_id_lo[1839]=4'h0;
text_id_lo[1840]=4'h0; text_id_lo[1841]=4'h0; text_id_lo[1842]=4'h0; text_id_lo[1843]=4'h0; text_id_lo[1844]=4'h0; text_id_lo[1845]=4'h0; text_id_lo[1846]=4'h0; text_id_lo[1847]=4'h0;
text_id_lo[1848]=4'h0; text_id_lo[1849]=4'h0; text_id_lo[1850]=4'h0; text_id_lo[1851]=4'h0; text_id_lo[1852]=4'h0; text_id_lo[1853]=4'h0; text_id_lo[1854]=4'h0; text_id_lo[1855]=4'h0;
text_id_lo[1856]=4'h0; text_id_lo[1857]=4'h0; text_id_lo[1858]=4'h0; text_id_lo[1859]=4'h0; text_id_lo[1860]=4'h0; text_id_lo[1861]=4'h0; text_id_lo[1862]=4'h0; text_id_lo[1863]=4'h0;
text_id_lo[1864]=4'h0; text_id_lo[1865]=4'h0; text_id_lo[1866]=4'h0; text_id_lo[1867]=4'h0; text_id_lo[1868]=4'h0; text_id_lo[1869]=4'h0; text_id_lo[1870]=4'h0; text_id_lo[1871]=4'h0;
text_id_lo[1872]=4'h0; text_id_lo[1873]=4'h0; text_id_lo[1874]=4'h0; text_id_lo[1875]=4'h0; text_id_lo[1876]=4'h0; text_id_lo[1877]=4'h0; text_id_lo[1878]=4'h0; text_id_lo[1879]=4'h0;
text_id_lo[1880]=4'h0; text_id_lo[1881]=4'h0; text_id_lo[1882]=4'h0; text_id_lo[1883]=4'h0; text_id_lo[1884]=4'h0; text_id_lo[1885]=4'h0; text_id_lo[1886]=4'h0; text_id_lo[1887]=4'h0;
text_id_lo[1888]=4'h0; text_id_lo[1889]=4'h0; text_id_lo[1890]=4'h0; text_id_lo[1891]=4'h0; text_id_lo[1892]=4'h0; text_id_lo[1893]=4'h0; text_id_lo[1894]=4'h0; text_id_lo[1895]=4'h0;
text_id_lo[1896]=4'h0; text_id_lo[1897]=4'h0; text_id_lo[1898]=4'h0; text_id_lo[1899]=4'h0; text_id_lo[1900]=4'h0; text_id_lo[1901]=4'h0; text_id_lo[1902]=4'h0; text_id_lo[1903]=4'h0;
text_id_lo[1904]=4'h0; text_id_lo[1905]=4'h0; text_id_lo[1906]=4'h0; text_id_lo[1907]=4'h0; text_id_lo[1908]=4'h0; text_id_lo[1909]=4'h0; text_id_lo[1910]=4'h0; text_id_lo[1911]=4'h0;
text_id_lo[1912]=4'h0; text_id_lo[1913]=4'h0; text_id_lo[1914]=4'h0; text_id_lo[1915]=4'h0; text_id_lo[1916]=4'h0; text_id_lo[1917]=4'h0; text_id_lo[1918]=4'h0; text_id_lo[1919]=4'h0;
text_id_lo[1920]=4'h0; text_id_lo[1921]=4'h0; text_id_lo[1922]=4'h0; text_id_lo[1923]=4'h0; text_id_lo[1924]=4'h0; text_id_lo[1925]=4'h0; text_id_lo[1926]=4'h0; text_id_lo[1927]=4'h0;
text_id_lo[1928]=4'h0; text_id_lo[1929]=4'h0; text_id_lo[1930]=4'h0; text_id_lo[1931]=4'h0; text_id_lo[1932]=4'h0; text_id_lo[1933]=4'h0; text_id_lo[1934]=4'h0; text_id_lo[1935]=4'h0;
text_id_lo[1936]=4'h0; text_id_lo[1937]=4'h0; text_id_lo[1938]=4'h0; text_id_lo[1939]=4'h0; text_id_lo[1940]=4'h0; text_id_lo[1941]=4'h0; text_id_lo[1942]=4'h0; text_id_lo[1943]=4'h0;
text_id_lo[1944]=4'h0; text_id_lo[1945]=4'h0; text_id_lo[1946]=4'h0; text_id_lo[1947]=4'h0; text_id_lo[1948]=4'h0; text_id_lo[1949]=4'h0; text_id_lo[1950]=4'h0; text_id_lo[1951]=4'h0;
text_id_lo[1952]=4'h0; text_id_lo[1953]=4'h0; text_id_lo[1954]=4'h0; text_id_lo[1955]=4'h0; text_id_lo[1956]=4'h0; text_id_lo[1957]=4'h0; text_id_lo[1958]=4'h0; text_id_lo[1959]=4'h0;
text_id_lo[1960]=4'h0; text_id_lo[1961]=4'h0; text_id_lo[1962]=4'h0; text_id_lo[1963]=4'h0; text_id_lo[1964]=4'h0; text_id_lo[1965]=4'h0; text_id_lo[1966]=4'h0; text_id_lo[1967]=4'h0;
text_id_lo[1968]=4'h0; text_id_lo[1969]=4'h0; text_id_lo[1970]=4'h0; text_id_lo[1971]=4'h0; text_id_lo[1972]=4'h0; text_id_lo[1973]=4'h0; text_id_lo[1974]=4'h0; text_id_lo[1975]=4'h0;
text_id_lo[1976]=4'h0; text_id_lo[1977]=4'h0; text_id_lo[1978]=4'h0; text_id_lo[1979]=4'h0; text_id_lo[1980]=4'h0; text_id_lo[1981]=4'h0; text_id_lo[1982]=4'h0; text_id_lo[1983]=4'h0;
text_id_lo[1984]=4'h0; text_id_lo[1985]=4'h0; text_id_lo[1986]=4'h0; text_id_lo[1987]=4'h0; text_id_lo[1988]=4'h0; text_id_lo[1989]=4'h0; text_id_lo[1990]=4'h0; text_id_lo[1991]=4'h0;
text_id_lo[1992]=4'h0; text_id_lo[1993]=4'h0; text_id_lo[1994]=4'h0; text_id_lo[1995]=4'h0; text_id_lo[1996]=4'h0; text_id_lo[1997]=4'h0; text_id_lo[1998]=4'h0; text_id_lo[1999]=4'h0;
text_id_lo[2000]=4'h0; text_id_lo[2001]=4'h0; text_id_lo[2002]=4'h0; text_id_lo[2003]=4'h0; text_id_lo[2004]=4'h0; text_id_lo[2005]=4'h0; text_id_lo[2006]=4'h0; text_id_lo[2007]=4'h0;
text_id_lo[2008]=4'h0; text_id_lo[2009]=4'h0; text_id_lo[2010]=4'h0; text_id_lo[2011]=4'h0; text_id_lo[2012]=4'h0; text_id_lo[2013]=4'h0; text_id_lo[2014]=4'h0; text_id_lo[2015]=4'h0;
text_id_lo[2016]=4'h0; text_id_lo[2017]=4'h0; text_id_lo[2018]=4'h0; text_id_lo[2019]=4'h0; text_id_lo[2020]=4'h0; text_id_lo[2021]=4'h0; text_id_lo[2022]=4'h0; text_id_lo[2023]=4'h0;
text_id_lo[2024]=4'h0; text_id_lo[2025]=4'h0; text_id_lo[2026]=4'h0; text_id_lo[2027]=4'h0; text_id_lo[2028]=4'h0; text_id_lo[2029]=4'h0; text_id_lo[2030]=4'h0; text_id_lo[2031]=4'h0;
text_id_lo[2032]=4'h0; text_id_lo[2033]=4'h0; text_id_lo[2034]=4'h0; text_id_lo[2035]=4'h0; text_id_lo[2036]=4'h0; text_id_lo[2037]=4'h0; text_id_lo[2038]=4'h0; text_id_lo[2039]=4'h0;
text_id_lo[2040]=4'h0; text_id_lo[2041]=4'h0; text_id_lo[2042]=4'h0; text_id_lo[2043]=4'h0; text_id_lo[2044]=4'h0; text_id_lo[2045]=4'h0; text_id_lo[2046]=4'h0; text_id_lo[2047]=4'h0;
end
reg [3:0] text_id_hi [0:2047]; /* fehdl force_ram=1, ram_style="bram" */
initial begin
text_id_hi[0]=4'h0; text_id_hi[1]=4'h0; text_id_hi[2]=4'h0; text_id_hi[3]=4'h0; text_id_hi[4]=4'h0; text_id_hi[5]=4'h0; text_id_hi[6]=4'h0; text_id_hi[7]=4'h0;
text_id_hi[8]=4'h0; text_id_hi[9]=4'h0; text_id_hi[10]=4'h0; text_id_hi[11]=4'h0; text_id_hi[12]=4'h0; text_id_hi[13]=4'h0; text_id_hi[14]=4'h0; text_id_hi[15]=4'h0;
text_id_hi[16]=4'h0; text_id_hi[17]=4'h0; text_id_hi[18]=4'h0; text_id_hi[19]=4'h0; text_id_hi[20]=4'h0; text_id_hi[21]=4'h0; text_id_hi[22]=4'h0; text_id_hi[23]=4'h0;
text_id_hi[24]=4'h0; text_id_hi[25]=4'h0; text_id_hi[26]=4'h0; text_id_hi[27]=4'h0; text_id_hi[28]=4'h0; text_id_hi[29]=4'h0; text_id_hi[30]=4'h0; text_id_hi[31]=4'h0;
text_id_hi[32]=4'h0; text_id_hi[33]=4'h0; text_id_hi[34]=4'h0; text_id_hi[35]=4'h0; text_id_hi[36]=4'h0; text_id_hi[37]=4'h0; text_id_hi[38]=4'h0; text_id_hi[39]=4'h0;
text_id_hi[40]=4'h0; text_id_hi[41]=4'h0; text_id_hi[42]=4'h0; text_id_hi[43]=4'h0; text_id_hi[44]=4'h0; text_id_hi[45]=4'h0; text_id_hi[46]=4'h0; text_id_hi[47]=4'h0;
text_id_hi[48]=4'h0; text_id_hi[49]=4'h0; text_id_hi[50]=4'h0; text_id_hi[51]=4'h0; text_id_hi[52]=4'h0; text_id_hi[53]=4'h0; text_id_hi[54]=4'h0; text_id_hi[55]=4'h0;
text_id_hi[56]=4'h0; text_id_hi[57]=4'h0; text_id_hi[58]=4'h0; text_id_hi[59]=4'h0; text_id_hi[60]=4'h0; text_id_hi[61]=4'h0; text_id_hi[62]=4'h0; text_id_hi[63]=4'h0;
text_id_hi[64]=4'h0; text_id_hi[65]=4'h0; text_id_hi[66]=4'h0; text_id_hi[67]=4'h0; text_id_hi[68]=4'h0; text_id_hi[69]=4'h0; text_id_hi[70]=4'h0; text_id_hi[71]=4'h0;
text_id_hi[72]=4'h0; text_id_hi[73]=4'h0; text_id_hi[74]=4'h0; text_id_hi[75]=4'h0; text_id_hi[76]=4'h0; text_id_hi[77]=4'h0; text_id_hi[78]=4'h0; text_id_hi[79]=4'h0;
text_id_hi[80]=4'h0; text_id_hi[81]=4'h0; text_id_hi[82]=4'h0; text_id_hi[83]=4'h0; text_id_hi[84]=4'h0; text_id_hi[85]=4'h0; text_id_hi[86]=4'h0; text_id_hi[87]=4'h0;
text_id_hi[88]=4'h0; text_id_hi[89]=4'h0; text_id_hi[90]=4'h0; text_id_hi[91]=4'h0; text_id_hi[92]=4'h0; text_id_hi[93]=4'h0; text_id_hi[94]=4'h0; text_id_hi[95]=4'h0;
text_id_hi[96]=4'h0; text_id_hi[97]=4'h0; text_id_hi[98]=4'h0; text_id_hi[99]=4'h0; text_id_hi[100]=4'h0; text_id_hi[101]=4'h0; text_id_hi[102]=4'h0; text_id_hi[103]=4'h0;
text_id_hi[104]=4'h0; text_id_hi[105]=4'h0; text_id_hi[106]=4'h0; text_id_hi[107]=4'h0; text_id_hi[108]=4'h0; text_id_hi[109]=4'h0; text_id_hi[110]=4'h0; text_id_hi[111]=4'h0;
text_id_hi[112]=4'h0; text_id_hi[113]=4'h0; text_id_hi[114]=4'h0; text_id_hi[115]=4'h0; text_id_hi[116]=4'h0; text_id_hi[117]=4'h0; text_id_hi[118]=4'h0; text_id_hi[119]=4'h0;
text_id_hi[120]=4'h0; text_id_hi[121]=4'h0; text_id_hi[122]=4'h0; text_id_hi[123]=4'h0; text_id_hi[124]=4'h0; text_id_hi[125]=4'h0; text_id_hi[126]=4'h0; text_id_hi[127]=4'h0;
text_id_hi[128]=4'h4; text_id_hi[129]=4'h5; text_id_hi[130]=4'h5; text_id_hi[131]=4'hd; text_id_hi[132]=4'h1; text_id_hi[133]=4'h4; text_id_hi[134]=4'h6; text_id_hi[135]=4'h1;
text_id_hi[136]=4'h0; text_id_hi[137]=4'h4; text_id_hi[138]=4'hc; text_id_hi[139]=4'h8; text_id_hi[140]=4'h8; text_id_hi[141]=4'h8; text_id_hi[142]=4'h9; text_id_hi[143]=4'h0;
text_id_hi[144]=4'hd; text_id_hi[145]=4'hd; text_id_hi[146]=4'hc; text_id_hi[147]=4'h0; text_id_hi[148]=4'h5; text_id_hi[149]=4'hb; text_id_hi[150]=4'h2; text_id_hi[151]=4'h3;
text_id_hi[152]=4'h0; text_id_hi[153]=4'hd; text_id_hi[154]=4'h6; text_id_hi[155]=4'h3; text_id_hi[156]=4'hc; text_id_hi[157]=4'hd; text_id_hi[158]=4'hb; text_id_hi[159]=4'hc;
text_id_hi[160]=4'h9; text_id_hi[161]=4'h9; text_id_hi[162]=4'h3; text_id_hi[163]=4'h8; text_id_hi[164]=4'h6; text_id_hi[165]=4'h0; text_id_hi[166]=4'h0; text_id_hi[167]=4'h0;
text_id_hi[168]=4'h0; text_id_hi[169]=4'h0; text_id_hi[170]=4'h0; text_id_hi[171]=4'h0; text_id_hi[172]=4'h0; text_id_hi[173]=4'h0; text_id_hi[174]=4'h0; text_id_hi[175]=4'h0;
text_id_hi[176]=4'h0; text_id_hi[177]=4'h0; text_id_hi[178]=4'h0; text_id_hi[179]=4'h0; text_id_hi[180]=4'h0; text_id_hi[181]=4'h0; text_id_hi[182]=4'h0; text_id_hi[183]=4'h0;
text_id_hi[184]=4'h0; text_id_hi[185]=4'h0; text_id_hi[186]=4'h0; text_id_hi[187]=4'h0; text_id_hi[188]=4'h0; text_id_hi[189]=4'h0; text_id_hi[190]=4'h0; text_id_hi[191]=4'h0;
text_id_hi[192]=4'h0; text_id_hi[193]=4'h0; text_id_hi[194]=4'h0; text_id_hi[195]=4'h0; text_id_hi[196]=4'h0; text_id_hi[197]=4'h0; text_id_hi[198]=4'h0; text_id_hi[199]=4'h0;
text_id_hi[200]=4'h0; text_id_hi[201]=4'h0; text_id_hi[202]=4'h0; text_id_hi[203]=4'h0; text_id_hi[204]=4'h0; text_id_hi[205]=4'h0; text_id_hi[206]=4'h0; text_id_hi[207]=4'h0;
text_id_hi[208]=4'h0; text_id_hi[209]=4'h0; text_id_hi[210]=4'h0; text_id_hi[211]=4'h0; text_id_hi[212]=4'h0; text_id_hi[213]=4'h0; text_id_hi[214]=4'h0; text_id_hi[215]=4'h0;
text_id_hi[216]=4'h0; text_id_hi[217]=4'h0; text_id_hi[218]=4'h0; text_id_hi[219]=4'h0; text_id_hi[220]=4'h0; text_id_hi[221]=4'h0; text_id_hi[222]=4'h0; text_id_hi[223]=4'h0;
text_id_hi[224]=4'h0; text_id_hi[225]=4'h0; text_id_hi[226]=4'h0; text_id_hi[227]=4'h0; text_id_hi[228]=4'h0; text_id_hi[229]=4'h0; text_id_hi[230]=4'h0; text_id_hi[231]=4'h0;
text_id_hi[232]=4'h0; text_id_hi[233]=4'h0; text_id_hi[234]=4'h0; text_id_hi[235]=4'h0; text_id_hi[236]=4'h0; text_id_hi[237]=4'h0; text_id_hi[238]=4'h0; text_id_hi[239]=4'h0;
text_id_hi[240]=4'h0; text_id_hi[241]=4'h0; text_id_hi[242]=4'h0; text_id_hi[243]=4'h0; text_id_hi[244]=4'h0; text_id_hi[245]=4'h0; text_id_hi[246]=4'h0; text_id_hi[247]=4'h0;
text_id_hi[248]=4'h0; text_id_hi[249]=4'h0; text_id_hi[250]=4'h0; text_id_hi[251]=4'h0; text_id_hi[252]=4'h0; text_id_hi[253]=4'h0; text_id_hi[254]=4'h0; text_id_hi[255]=4'h0;
text_id_hi[256]=4'h2; text_id_hi[257]=4'h1; text_id_hi[258]=4'h0; text_id_hi[259]=4'h4; text_id_hi[260]=4'hc; text_id_hi[261]=4'h2; text_id_hi[262]=4'hd; text_id_hi[263]=4'h9;
text_id_hi[264]=4'h0; text_id_hi[265]=4'h9; text_id_hi[266]=4'h3; text_id_hi[267]=4'h7; text_id_hi[268]=4'h0; text_id_hi[269]=4'h9; text_id_hi[270]=4'h1; text_id_hi[271]=4'h9;
text_id_hi[272]=4'h8; text_id_hi[273]=4'hc; text_id_hi[274]=4'h0; text_id_hi[275]=4'h9; text_id_hi[276]=4'h0; text_id_hi[277]=4'h9; text_id_hi[278]=4'hc; text_id_hi[279]=4'h8;
text_id_hi[280]=4'h0; text_id_hi[281]=4'h0; text_id_hi[282]=4'h0; text_id_hi[283]=4'h0; text_id_hi[284]=4'h0; text_id_hi[285]=4'h0; text_id_hi[286]=4'h0; text_id_hi[287]=4'h0;
text_id_hi[288]=4'h0; text_id_hi[289]=4'h0; text_id_hi[290]=4'h0; text_id_hi[291]=4'h0; text_id_hi[292]=4'h0; text_id_hi[293]=4'h0; text_id_hi[294]=4'h0; text_id_hi[295]=4'h0;
text_id_hi[296]=4'h0; text_id_hi[297]=4'h0; text_id_hi[298]=4'h0; text_id_hi[299]=4'h0; text_id_hi[300]=4'h0; text_id_hi[301]=4'h0; text_id_hi[302]=4'h0; text_id_hi[303]=4'h0;
text_id_hi[304]=4'h0; text_id_hi[305]=4'h0; text_id_hi[306]=4'h0; text_id_hi[307]=4'h0; text_id_hi[308]=4'h0; text_id_hi[309]=4'h0; text_id_hi[310]=4'h0; text_id_hi[311]=4'h0;
text_id_hi[312]=4'h0; text_id_hi[313]=4'h0; text_id_hi[314]=4'h0; text_id_hi[315]=4'h0; text_id_hi[316]=4'h0; text_id_hi[317]=4'h0; text_id_hi[318]=4'h0; text_id_hi[319]=4'h0;
text_id_hi[320]=4'h0; text_id_hi[321]=4'h0; text_id_hi[322]=4'h0; text_id_hi[323]=4'h0; text_id_hi[324]=4'h0; text_id_hi[325]=4'h0; text_id_hi[326]=4'h0; text_id_hi[327]=4'h0;
text_id_hi[328]=4'h0; text_id_hi[329]=4'h0; text_id_hi[330]=4'h0; text_id_hi[331]=4'h0; text_id_hi[332]=4'h0; text_id_hi[333]=4'h0; text_id_hi[334]=4'h0; text_id_hi[335]=4'h0;
text_id_hi[336]=4'h0; text_id_hi[337]=4'h0; text_id_hi[338]=4'h0; text_id_hi[339]=4'h0; text_id_hi[340]=4'h0; text_id_hi[341]=4'h0; text_id_hi[342]=4'h0; text_id_hi[343]=4'h0;
text_id_hi[344]=4'h0; text_id_hi[345]=4'h0; text_id_hi[346]=4'h0; text_id_hi[347]=4'h0; text_id_hi[348]=4'h0; text_id_hi[349]=4'h0; text_id_hi[350]=4'h0; text_id_hi[351]=4'h0;
text_id_hi[352]=4'h0; text_id_hi[353]=4'h0; text_id_hi[354]=4'h0; text_id_hi[355]=4'h0; text_id_hi[356]=4'h0; text_id_hi[357]=4'h0; text_id_hi[358]=4'h0; text_id_hi[359]=4'h0;
text_id_hi[360]=4'h0; text_id_hi[361]=4'h0; text_id_hi[362]=4'h0; text_id_hi[363]=4'h0; text_id_hi[364]=4'h0; text_id_hi[365]=4'h0; text_id_hi[366]=4'h0; text_id_hi[367]=4'h0;
text_id_hi[368]=4'h0; text_id_hi[369]=4'h0; text_id_hi[370]=4'h0; text_id_hi[371]=4'h0; text_id_hi[372]=4'h0; text_id_hi[373]=4'h0; text_id_hi[374]=4'h0; text_id_hi[375]=4'h0;
text_id_hi[376]=4'h0; text_id_hi[377]=4'h0; text_id_hi[378]=4'h0; text_id_hi[379]=4'h0; text_id_hi[380]=4'h0; text_id_hi[381]=4'h0; text_id_hi[382]=4'h0; text_id_hi[383]=4'h0;
text_id_hi[384]=4'h5; text_id_hi[385]=4'h0; text_id_hi[386]=4'h5; text_id_hi[387]=4'hb; text_id_hi[388]=4'h2; text_id_hi[389]=4'h3; text_id_hi[390]=4'h0; text_id_hi[391]=4'h0;
text_id_hi[392]=4'h2; text_id_hi[393]=4'ha; text_id_hi[394]=4'h2; text_id_hi[395]=4'h3; text_id_hi[396]=4'h0; text_id_hi[397]=4'hc; text_id_hi[398]=4'h9; text_id_hi[399]=4'h5;
text_id_hi[400]=4'hb; text_id_hi[401]=4'h8; text_id_hi[402]=4'hc; text_id_hi[403]=4'hd; text_id_hi[404]=4'hb; text_id_hi[405]=4'hb; text_id_hi[406]=4'hc; text_id_hi[407]=4'h1;
text_id_hi[408]=4'h0; text_id_hi[409]=4'h2; text_id_hi[410]=4'h3; text_id_hi[411]=4'h0; text_id_hi[412]=4'hd; text_id_hi[413]=4'hd; text_id_hi[414]=4'h0; text_id_hi[415]=4'h5;
text_id_hi[416]=4'hd; text_id_hi[417]=4'h0; text_id_hi[418]=4'hc; text_id_hi[419]=4'hd; text_id_hi[420]=4'ha; text_id_hi[421]=4'h0; text_id_hi[422]=4'h0; text_id_hi[423]=4'h0;
text_id_hi[424]=4'h0; text_id_hi[425]=4'h0; text_id_hi[426]=4'h0; text_id_hi[427]=4'h0; text_id_hi[428]=4'h0; text_id_hi[429]=4'h0; text_id_hi[430]=4'h0; text_id_hi[431]=4'h0;
text_id_hi[432]=4'h0; text_id_hi[433]=4'h0; text_id_hi[434]=4'h0; text_id_hi[435]=4'h0; text_id_hi[436]=4'h0; text_id_hi[437]=4'h0; text_id_hi[438]=4'h0; text_id_hi[439]=4'h0;
text_id_hi[440]=4'h0; text_id_hi[441]=4'h0; text_id_hi[442]=4'h0; text_id_hi[443]=4'h0; text_id_hi[444]=4'h0; text_id_hi[445]=4'h0; text_id_hi[446]=4'h0; text_id_hi[447]=4'h0;
text_id_hi[448]=4'h0; text_id_hi[449]=4'h0; text_id_hi[450]=4'h0; text_id_hi[451]=4'h0; text_id_hi[452]=4'h0; text_id_hi[453]=4'h0; text_id_hi[454]=4'h0; text_id_hi[455]=4'h0;
text_id_hi[456]=4'h0; text_id_hi[457]=4'h0; text_id_hi[458]=4'h0; text_id_hi[459]=4'h0; text_id_hi[460]=4'h0; text_id_hi[461]=4'h0; text_id_hi[462]=4'h0; text_id_hi[463]=4'h0;
text_id_hi[464]=4'h0; text_id_hi[465]=4'h0; text_id_hi[466]=4'h0; text_id_hi[467]=4'h0; text_id_hi[468]=4'h0; text_id_hi[469]=4'h0; text_id_hi[470]=4'h0; text_id_hi[471]=4'h0;
text_id_hi[472]=4'h0; text_id_hi[473]=4'h0; text_id_hi[474]=4'h0; text_id_hi[475]=4'h0; text_id_hi[476]=4'h0; text_id_hi[477]=4'h0; text_id_hi[478]=4'h0; text_id_hi[479]=4'h0;
text_id_hi[480]=4'h0; text_id_hi[481]=4'h0; text_id_hi[482]=4'h0; text_id_hi[483]=4'h0; text_id_hi[484]=4'h0; text_id_hi[485]=4'h0; text_id_hi[486]=4'h0; text_id_hi[487]=4'h0;
text_id_hi[488]=4'h0; text_id_hi[489]=4'h0; text_id_hi[490]=4'h0; text_id_hi[491]=4'h0; text_id_hi[492]=4'h0; text_id_hi[493]=4'h0; text_id_hi[494]=4'h0; text_id_hi[495]=4'h0;
text_id_hi[496]=4'h0; text_id_hi[497]=4'h0; text_id_hi[498]=4'h0; text_id_hi[499]=4'h0; text_id_hi[500]=4'h0; text_id_hi[501]=4'h0; text_id_hi[502]=4'h0; text_id_hi[503]=4'h0;
text_id_hi[504]=4'h0; text_id_hi[505]=4'h0; text_id_hi[506]=4'h0; text_id_hi[507]=4'h0; text_id_hi[508]=4'h0; text_id_hi[509]=4'h0; text_id_hi[510]=4'h0; text_id_hi[511]=4'h0;
text_id_hi[512]=4'h8; text_id_hi[513]=4'hc; text_id_hi[514]=4'h0; text_id_hi[515]=4'h6; text_id_hi[516]=4'h6; text_id_hi[517]=4'h9; text_id_hi[518]=4'h9; text_id_hi[519]=4'h8;
text_id_hi[520]=4'h9; text_id_hi[521]=4'h0; text_id_hi[522]=4'h0; text_id_hi[523]=4'h7; text_id_hi[524]=4'hd; text_id_hi[525]=4'h8; text_id_hi[526]=4'h8; text_id_hi[527]=4'h8;
text_id_hi[528]=4'h4; text_id_hi[529]=4'hd; text_id_hi[530]=4'h0; text_id_hi[531]=4'hd; text_id_hi[532]=4'hb; text_id_hi[533]=4'hc; text_id_hi[534]=4'h7; text_id_hi[535]=4'ha;
text_id_hi[536]=4'h0; text_id_hi[537]=4'h2; text_id_hi[538]=4'ha; text_id_hi[539]=4'h0; text_id_hi[540]=4'h2; text_id_hi[541]=4'h4; text_id_hi[542]=4'h0; text_id_hi[543]=4'h3;
text_id_hi[544]=4'h3; text_id_hi[545]=4'h0; text_id_hi[546]=4'h7; text_id_hi[547]=4'hd; text_id_hi[548]=4'h0; text_id_hi[549]=4'h0; text_id_hi[550]=4'h0; text_id_hi[551]=4'h0;
text_id_hi[552]=4'h0; text_id_hi[553]=4'h0; text_id_hi[554]=4'h0; text_id_hi[555]=4'h0; text_id_hi[556]=4'h0; text_id_hi[557]=4'h0; text_id_hi[558]=4'h0; text_id_hi[559]=4'h0;
text_id_hi[560]=4'h0; text_id_hi[561]=4'h0; text_id_hi[562]=4'h0; text_id_hi[563]=4'h0; text_id_hi[564]=4'h0; text_id_hi[565]=4'h0; text_id_hi[566]=4'h0; text_id_hi[567]=4'h0;
text_id_hi[568]=4'h0; text_id_hi[569]=4'h0; text_id_hi[570]=4'h0; text_id_hi[571]=4'h0; text_id_hi[572]=4'h0; text_id_hi[573]=4'h0; text_id_hi[574]=4'h0; text_id_hi[575]=4'h0;
text_id_hi[576]=4'h0; text_id_hi[577]=4'h0; text_id_hi[578]=4'h0; text_id_hi[579]=4'h0; text_id_hi[580]=4'h0; text_id_hi[581]=4'h0; text_id_hi[582]=4'h0; text_id_hi[583]=4'h0;
text_id_hi[584]=4'h0; text_id_hi[585]=4'h0; text_id_hi[586]=4'h0; text_id_hi[587]=4'h0; text_id_hi[588]=4'h0; text_id_hi[589]=4'h0; text_id_hi[590]=4'h0; text_id_hi[591]=4'h0;
text_id_hi[592]=4'h0; text_id_hi[593]=4'h0; text_id_hi[594]=4'h0; text_id_hi[595]=4'h0; text_id_hi[596]=4'h0; text_id_hi[597]=4'h0; text_id_hi[598]=4'h0; text_id_hi[599]=4'h0;
text_id_hi[600]=4'h0; text_id_hi[601]=4'h0; text_id_hi[602]=4'h0; text_id_hi[603]=4'h0; text_id_hi[604]=4'h0; text_id_hi[605]=4'h0; text_id_hi[606]=4'h0; text_id_hi[607]=4'h0;
text_id_hi[608]=4'h0; text_id_hi[609]=4'h0; text_id_hi[610]=4'h0; text_id_hi[611]=4'h0; text_id_hi[612]=4'h0; text_id_hi[613]=4'h0; text_id_hi[614]=4'h0; text_id_hi[615]=4'h0;
text_id_hi[616]=4'h0; text_id_hi[617]=4'h0; text_id_hi[618]=4'h0; text_id_hi[619]=4'h0; text_id_hi[620]=4'h0; text_id_hi[621]=4'h0; text_id_hi[622]=4'h0; text_id_hi[623]=4'h0;
text_id_hi[624]=4'h0; text_id_hi[625]=4'h0; text_id_hi[626]=4'h0; text_id_hi[627]=4'h0; text_id_hi[628]=4'h0; text_id_hi[629]=4'h0; text_id_hi[630]=4'h0; text_id_hi[631]=4'h0;
text_id_hi[632]=4'h0; text_id_hi[633]=4'h0; text_id_hi[634]=4'h0; text_id_hi[635]=4'h0; text_id_hi[636]=4'h0; text_id_hi[637]=4'h0; text_id_hi[638]=4'h0; text_id_hi[639]=4'h0;
text_id_hi[640]=4'h8; text_id_hi[641]=4'hc; text_id_hi[642]=4'h0; text_id_hi[643]=4'h8; text_id_hi[644]=4'h3; text_id_hi[645]=4'ha; text_id_hi[646]=4'h5; text_id_hi[647]=4'h0;
text_id_hi[648]=4'h0; text_id_hi[649]=4'h7; text_id_hi[650]=4'h0; text_id_hi[651]=4'h2; text_id_hi[652]=4'h1; text_id_hi[653]=4'hb; text_id_hi[654]=4'he; text_id_hi[655]=4'h0;
text_id_hi[656]=4'hd; text_id_hi[657]=4'hb; text_id_hi[658]=4'hb; text_id_hi[659]=4'ha; text_id_hi[660]=4'h5; text_id_hi[661]=4'h0; text_id_hi[662]=4'ha; text_id_hi[663]=4'h0;
text_id_hi[664]=4'hd; text_id_hi[665]=4'h6; text_id_hi[666]=4'h0; text_id_hi[667]=4'h7; text_id_hi[668]=4'h0; text_id_hi[669]=4'h0; text_id_hi[670]=4'h0; text_id_hi[671]=4'h0;
text_id_hi[672]=4'h0; text_id_hi[673]=4'h0; text_id_hi[674]=4'h0; text_id_hi[675]=4'h0; text_id_hi[676]=4'h0; text_id_hi[677]=4'h0; text_id_hi[678]=4'h0; text_id_hi[679]=4'h0;
text_id_hi[680]=4'h0; text_id_hi[681]=4'h0; text_id_hi[682]=4'h0; text_id_hi[683]=4'h0; text_id_hi[684]=4'h0; text_id_hi[685]=4'h0; text_id_hi[686]=4'h0; text_id_hi[687]=4'h0;
text_id_hi[688]=4'h0; text_id_hi[689]=4'h0; text_id_hi[690]=4'h0; text_id_hi[691]=4'h0; text_id_hi[692]=4'h0; text_id_hi[693]=4'h0; text_id_hi[694]=4'h0; text_id_hi[695]=4'h0;
text_id_hi[696]=4'h0; text_id_hi[697]=4'h0; text_id_hi[698]=4'h0; text_id_hi[699]=4'h0; text_id_hi[700]=4'h0; text_id_hi[701]=4'h0; text_id_hi[702]=4'h0; text_id_hi[703]=4'h0;
text_id_hi[704]=4'h0; text_id_hi[705]=4'h0; text_id_hi[706]=4'h0; text_id_hi[707]=4'h0; text_id_hi[708]=4'h0; text_id_hi[709]=4'h0; text_id_hi[710]=4'h0; text_id_hi[711]=4'h0;
text_id_hi[712]=4'h0; text_id_hi[713]=4'h0; text_id_hi[714]=4'h0; text_id_hi[715]=4'h0; text_id_hi[716]=4'h0; text_id_hi[717]=4'h0; text_id_hi[718]=4'h0; text_id_hi[719]=4'h0;
text_id_hi[720]=4'h0; text_id_hi[721]=4'h0; text_id_hi[722]=4'h0; text_id_hi[723]=4'h0; text_id_hi[724]=4'h0; text_id_hi[725]=4'h0; text_id_hi[726]=4'h0; text_id_hi[727]=4'h0;
text_id_hi[728]=4'h0; text_id_hi[729]=4'h0; text_id_hi[730]=4'h0; text_id_hi[731]=4'h0; text_id_hi[732]=4'h0; text_id_hi[733]=4'h0; text_id_hi[734]=4'h0; text_id_hi[735]=4'h0;
text_id_hi[736]=4'h0; text_id_hi[737]=4'h0; text_id_hi[738]=4'h0; text_id_hi[739]=4'h0; text_id_hi[740]=4'h0; text_id_hi[741]=4'h0; text_id_hi[742]=4'h0; text_id_hi[743]=4'h0;
text_id_hi[744]=4'h0; text_id_hi[745]=4'h0; text_id_hi[746]=4'h0; text_id_hi[747]=4'h0; text_id_hi[748]=4'h0; text_id_hi[749]=4'h0; text_id_hi[750]=4'h0; text_id_hi[751]=4'h0;
text_id_hi[752]=4'h0; text_id_hi[753]=4'h0; text_id_hi[754]=4'h0; text_id_hi[755]=4'h0; text_id_hi[756]=4'h0; text_id_hi[757]=4'h0; text_id_hi[758]=4'h0; text_id_hi[759]=4'h0;
text_id_hi[760]=4'h0; text_id_hi[761]=4'h0; text_id_hi[762]=4'h0; text_id_hi[763]=4'h0; text_id_hi[764]=4'h0; text_id_hi[765]=4'h0; text_id_hi[766]=4'h0; text_id_hi[767]=4'h0;
text_id_hi[768]=4'h8; text_id_hi[769]=4'h8; text_id_hi[770]=4'h1; text_id_hi[771]=4'hc; text_id_hi[772]=4'h4; text_id_hi[773]=4'h6; text_id_hi[774]=4'h9; text_id_hi[775]=4'h6;
text_id_hi[776]=4'h8; text_id_hi[777]=4'h1; text_id_hi[778]=4'h2; text_id_hi[779]=4'h0; text_id_hi[780]=4'h8; text_id_hi[781]=4'hb; text_id_hi[782]=4'hd; text_id_hi[783]=4'hb;
text_id_hi[784]=4'h3; text_id_hi[785]=4'h1; text_id_hi[786]=4'hc; text_id_hi[787]=4'h4; text_id_hi[788]=4'h6; text_id_hi[789]=4'h0; text_id_hi[790]=4'h6; text_id_hi[791]=4'h1;
text_id_hi[792]=4'h9; text_id_hi[793]=4'h3; text_id_hi[794]=4'h9; text_id_hi[795]=4'h3; text_id_hi[796]=4'hc; text_id_hi[797]=4'h6; text_id_hi[798]=4'h9; text_id_hi[799]=4'hd;
text_id_hi[800]=4'h6; text_id_hi[801]=4'h7; text_id_hi[802]=4'h1; text_id_hi[803]=4'hb; text_id_hi[804]=4'hd; text_id_hi[805]=4'h7; text_id_hi[806]=4'h9; text_id_hi[807]=4'h3;
text_id_hi[808]=4'hb; text_id_hi[809]=4'h1; text_id_hi[810]=4'hb; text_id_hi[811]=4'h2; text_id_hi[812]=4'h0; text_id_hi[813]=4'h0; text_id_hi[814]=4'h0; text_id_hi[815]=4'h0;
text_id_hi[816]=4'h0; text_id_hi[817]=4'h0; text_id_hi[818]=4'h0; text_id_hi[819]=4'h0; text_id_hi[820]=4'h0; text_id_hi[821]=4'h0; text_id_hi[822]=4'h0; text_id_hi[823]=4'h0;
text_id_hi[824]=4'h0; text_id_hi[825]=4'h0; text_id_hi[826]=4'h0; text_id_hi[827]=4'h0; text_id_hi[828]=4'h0; text_id_hi[829]=4'h0; text_id_hi[830]=4'h0; text_id_hi[831]=4'h0;
text_id_hi[832]=4'h0; text_id_hi[833]=4'h0; text_id_hi[834]=4'h0; text_id_hi[835]=4'h0; text_id_hi[836]=4'h0; text_id_hi[837]=4'h0; text_id_hi[838]=4'h0; text_id_hi[839]=4'h0;
text_id_hi[840]=4'h0; text_id_hi[841]=4'h0; text_id_hi[842]=4'h0; text_id_hi[843]=4'h0; text_id_hi[844]=4'h0; text_id_hi[845]=4'h0; text_id_hi[846]=4'h0; text_id_hi[847]=4'h0;
text_id_hi[848]=4'h0; text_id_hi[849]=4'h0; text_id_hi[850]=4'h0; text_id_hi[851]=4'h0; text_id_hi[852]=4'h0; text_id_hi[853]=4'h0; text_id_hi[854]=4'h0; text_id_hi[855]=4'h0;
text_id_hi[856]=4'h0; text_id_hi[857]=4'h0; text_id_hi[858]=4'h0; text_id_hi[859]=4'h0; text_id_hi[860]=4'h0; text_id_hi[861]=4'h0; text_id_hi[862]=4'h0; text_id_hi[863]=4'h0;
text_id_hi[864]=4'h0; text_id_hi[865]=4'h0; text_id_hi[866]=4'h0; text_id_hi[867]=4'h0; text_id_hi[868]=4'h0; text_id_hi[869]=4'h0; text_id_hi[870]=4'h0; text_id_hi[871]=4'h0;
text_id_hi[872]=4'h0; text_id_hi[873]=4'h0; text_id_hi[874]=4'h0; text_id_hi[875]=4'h0; text_id_hi[876]=4'h0; text_id_hi[877]=4'h0; text_id_hi[878]=4'h0; text_id_hi[879]=4'h0;
text_id_hi[880]=4'h0; text_id_hi[881]=4'h0; text_id_hi[882]=4'h0; text_id_hi[883]=4'h0; text_id_hi[884]=4'h0; text_id_hi[885]=4'h0; text_id_hi[886]=4'h0; text_id_hi[887]=4'h0;
text_id_hi[888]=4'h0; text_id_hi[889]=4'h0; text_id_hi[890]=4'h0; text_id_hi[891]=4'h0; text_id_hi[892]=4'h0; text_id_hi[893]=4'h0; text_id_hi[894]=4'h0; text_id_hi[895]=4'h0;
text_id_hi[896]=4'h6; text_id_hi[897]=4'ha; text_id_hi[898]=4'h2; text_id_hi[899]=4'h9; text_id_hi[900]=4'h7; text_id_hi[901]=4'h1; text_id_hi[902]=4'h1; text_id_hi[903]=4'h9;
text_id_hi[904]=4'h3; text_id_hi[905]=4'h7; text_id_hi[906]=4'ha; text_id_hi[907]=4'h0; text_id_hi[908]=4'h4; text_id_hi[909]=4'h9; text_id_hi[910]=4'hd; text_id_hi[911]=4'hb;
text_id_hi[912]=4'h6; text_id_hi[913]=4'h1; text_id_hi[914]=4'h7; text_id_hi[915]=4'he; text_id_hi[916]=4'h0; text_id_hi[917]=4'hd; text_id_hi[918]=4'h5; text_id_hi[919]=4'h2;
text_id_hi[920]=4'h6; text_id_hi[921]=4'h1; text_id_hi[922]=4'hd; text_id_hi[923]=4'hb; text_id_hi[924]=4'hd; text_id_hi[925]=4'ha; text_id_hi[926]=4'h9; text_id_hi[927]=4'h9;
text_id_hi[928]=4'h3; text_id_hi[929]=4'h4; text_id_hi[930]=4'hd; text_id_hi[931]=4'h0; text_id_hi[932]=4'h0; text_id_hi[933]=4'h0; text_id_hi[934]=4'h0; text_id_hi[935]=4'h0;
text_id_hi[936]=4'h0; text_id_hi[937]=4'h0; text_id_hi[938]=4'h0; text_id_hi[939]=4'h0; text_id_hi[940]=4'h0; text_id_hi[941]=4'h0; text_id_hi[942]=4'h0; text_id_hi[943]=4'h0;
text_id_hi[944]=4'h0; text_id_hi[945]=4'h0; text_id_hi[946]=4'h0; text_id_hi[947]=4'h0; text_id_hi[948]=4'h0; text_id_hi[949]=4'h0; text_id_hi[950]=4'h0; text_id_hi[951]=4'h0;
text_id_hi[952]=4'h0; text_id_hi[953]=4'h0; text_id_hi[954]=4'h0; text_id_hi[955]=4'h0; text_id_hi[956]=4'h0; text_id_hi[957]=4'h0; text_id_hi[958]=4'h0; text_id_hi[959]=4'h0;
text_id_hi[960]=4'h0; text_id_hi[961]=4'h0; text_id_hi[962]=4'h0; text_id_hi[963]=4'h0; text_id_hi[964]=4'h0; text_id_hi[965]=4'h0; text_id_hi[966]=4'h0; text_id_hi[967]=4'h0;
text_id_hi[968]=4'h0; text_id_hi[969]=4'h0; text_id_hi[970]=4'h0; text_id_hi[971]=4'h0; text_id_hi[972]=4'h0; text_id_hi[973]=4'h0; text_id_hi[974]=4'h0; text_id_hi[975]=4'h0;
text_id_hi[976]=4'h0; text_id_hi[977]=4'h0; text_id_hi[978]=4'h0; text_id_hi[979]=4'h0; text_id_hi[980]=4'h0; text_id_hi[981]=4'h0; text_id_hi[982]=4'h0; text_id_hi[983]=4'h0;
text_id_hi[984]=4'h0; text_id_hi[985]=4'h0; text_id_hi[986]=4'h0; text_id_hi[987]=4'h0; text_id_hi[988]=4'h0; text_id_hi[989]=4'h0; text_id_hi[990]=4'h0; text_id_hi[991]=4'h0;
text_id_hi[992]=4'h0; text_id_hi[993]=4'h0; text_id_hi[994]=4'h0; text_id_hi[995]=4'h0; text_id_hi[996]=4'h0; text_id_hi[997]=4'h0; text_id_hi[998]=4'h0; text_id_hi[999]=4'h0;
text_id_hi[1000]=4'h0; text_id_hi[1001]=4'h0; text_id_hi[1002]=4'h0; text_id_hi[1003]=4'h0; text_id_hi[1004]=4'h0; text_id_hi[1005]=4'h0; text_id_hi[1006]=4'h0; text_id_hi[1007]=4'h0;
text_id_hi[1008]=4'h0; text_id_hi[1009]=4'h0; text_id_hi[1010]=4'h0; text_id_hi[1011]=4'h0; text_id_hi[1012]=4'h0; text_id_hi[1013]=4'h0; text_id_hi[1014]=4'h0; text_id_hi[1015]=4'h0;
text_id_hi[1016]=4'h0; text_id_hi[1017]=4'h0; text_id_hi[1018]=4'h0; text_id_hi[1019]=4'h0; text_id_hi[1020]=4'h0; text_id_hi[1021]=4'h0; text_id_hi[1022]=4'h0; text_id_hi[1023]=4'h0;
text_id_hi[1024]=4'h0; text_id_hi[1025]=4'h6; text_id_hi[1026]=4'h6; text_id_hi[1027]=4'h9; text_id_hi[1028]=4'h9; text_id_hi[1029]=4'h8; text_id_hi[1030]=4'h9; text_id_hi[1031]=4'hd;
text_id_hi[1032]=4'h9; text_id_hi[1033]=4'h7; text_id_hi[1034]=4'hb; text_id_hi[1035]=4'h2; text_id_hi[1036]=4'h4; text_id_hi[1037]=4'h0; text_id_hi[1038]=4'hc; text_id_hi[1039]=4'h9;
text_id_hi[1040]=4'h9; text_id_hi[1041]=4'h7; text_id_hi[1042]=4'hd; text_id_hi[1043]=4'h3; text_id_hi[1044]=4'h9; text_id_hi[1045]=4'h9; text_id_hi[1046]=4'h3; text_id_hi[1047]=4'h6;
text_id_hi[1048]=4'h9; text_id_hi[1049]=4'he; text_id_hi[1050]=4'h0; text_id_hi[1051]=4'hc; text_id_hi[1052]=4'hd; text_id_hi[1053]=4'h5; text_id_hi[1054]=4'hd; text_id_hi[1055]=4'h0;
text_id_hi[1056]=4'h7; text_id_hi[1057]=4'h4; text_id_hi[1058]=4'h4; text_id_hi[1059]=4'h6; text_id_hi[1060]=4'h0; text_id_hi[1061]=4'h3; text_id_hi[1062]=4'h9; text_id_hi[1063]=4'h3;
text_id_hi[1064]=4'hc; text_id_hi[1065]=4'h6; text_id_hi[1066]=4'hb; text_id_hi[1067]=4'h0; text_id_hi[1068]=4'h0; text_id_hi[1069]=4'h0; text_id_hi[1070]=4'h0; text_id_hi[1071]=4'h0;
text_id_hi[1072]=4'h0; text_id_hi[1073]=4'h0; text_id_hi[1074]=4'h0; text_id_hi[1075]=4'h0; text_id_hi[1076]=4'h0; text_id_hi[1077]=4'h0; text_id_hi[1078]=4'h0; text_id_hi[1079]=4'h0;
text_id_hi[1080]=4'h0; text_id_hi[1081]=4'h0; text_id_hi[1082]=4'h0; text_id_hi[1083]=4'h0; text_id_hi[1084]=4'h0; text_id_hi[1085]=4'h0; text_id_hi[1086]=4'h0; text_id_hi[1087]=4'h0;
text_id_hi[1088]=4'h0; text_id_hi[1089]=4'h0; text_id_hi[1090]=4'h0; text_id_hi[1091]=4'h0; text_id_hi[1092]=4'h0; text_id_hi[1093]=4'h0; text_id_hi[1094]=4'h0; text_id_hi[1095]=4'h0;
text_id_hi[1096]=4'h0; text_id_hi[1097]=4'h0; text_id_hi[1098]=4'h0; text_id_hi[1099]=4'h0; text_id_hi[1100]=4'h0; text_id_hi[1101]=4'h0; text_id_hi[1102]=4'h0; text_id_hi[1103]=4'h0;
text_id_hi[1104]=4'h0; text_id_hi[1105]=4'h0; text_id_hi[1106]=4'h0; text_id_hi[1107]=4'h0; text_id_hi[1108]=4'h0; text_id_hi[1109]=4'h0; text_id_hi[1110]=4'h0; text_id_hi[1111]=4'h0;
text_id_hi[1112]=4'h0; text_id_hi[1113]=4'h0; text_id_hi[1114]=4'h0; text_id_hi[1115]=4'h0; text_id_hi[1116]=4'h0; text_id_hi[1117]=4'h0; text_id_hi[1118]=4'h0; text_id_hi[1119]=4'h0;
text_id_hi[1120]=4'h0; text_id_hi[1121]=4'h0; text_id_hi[1122]=4'h0; text_id_hi[1123]=4'h0; text_id_hi[1124]=4'h0; text_id_hi[1125]=4'h0; text_id_hi[1126]=4'h0; text_id_hi[1127]=4'h0;
text_id_hi[1128]=4'h0; text_id_hi[1129]=4'h0; text_id_hi[1130]=4'h0; text_id_hi[1131]=4'h0; text_id_hi[1132]=4'h0; text_id_hi[1133]=4'h0; text_id_hi[1134]=4'h0; text_id_hi[1135]=4'h0;
text_id_hi[1136]=4'h0; text_id_hi[1137]=4'h0; text_id_hi[1138]=4'h0; text_id_hi[1139]=4'h0; text_id_hi[1140]=4'h0; text_id_hi[1141]=4'h0; text_id_hi[1142]=4'h0; text_id_hi[1143]=4'h0;
text_id_hi[1144]=4'h0; text_id_hi[1145]=4'h0; text_id_hi[1146]=4'h0; text_id_hi[1147]=4'h0; text_id_hi[1148]=4'h0; text_id_hi[1149]=4'h0; text_id_hi[1150]=4'h0; text_id_hi[1151]=4'h0;
text_id_hi[1152]=4'h0; text_id_hi[1153]=4'hd; text_id_hi[1154]=4'hb; text_id_hi[1155]=4'h2; text_id_hi[1156]=4'h3; text_id_hi[1157]=4'h4; text_id_hi[1158]=4'h2; text_id_hi[1159]=4'hb;
text_id_hi[1160]=4'h0; text_id_hi[1161]=4'h0; text_id_hi[1162]=4'h4; text_id_hi[1163]=4'h6; text_id_hi[1164]=4'ha; text_id_hi[1165]=4'h3; text_id_hi[1166]=4'h1; text_id_hi[1167]=4'h9;
text_id_hi[1168]=4'h3; text_id_hi[1169]=4'hd; text_id_hi[1170]=4'h7; text_id_hi[1171]=4'hc; text_id_hi[1172]=4'h4; text_id_hi[1173]=4'hd; text_id_hi[1174]=4'h2; text_id_hi[1175]=4'hb;
text_id_hi[1176]=4'h8; text_id_hi[1177]=4'h9; text_id_hi[1178]=4'ha; text_id_hi[1179]=4'h0; text_id_hi[1180]=4'hd; text_id_hi[1181]=4'h5; text_id_hi[1182]=4'h0; text_id_hi[1183]=4'h1;
text_id_hi[1184]=4'h9; text_id_hi[1185]=4'h3; text_id_hi[1186]=4'h1; text_id_hi[1187]=4'hb; text_id_hi[1188]=4'h0; text_id_hi[1189]=4'h3; text_id_hi[1190]=4'h1; text_id_hi[1191]=4'h7;
text_id_hi[1192]=4'hc; text_id_hi[1193]=4'h6; text_id_hi[1194]=4'h6; text_id_hi[1195]=4'hd; text_id_hi[1196]=4'h6; text_id_hi[1197]=4'h3; text_id_hi[1198]=4'h9; text_id_hi[1199]=4'h7;
text_id_hi[1200]=4'h9; text_id_hi[1201]=4'h3; text_id_hi[1202]=4'h9; text_id_hi[1203]=4'h4; text_id_hi[1204]=4'h1; text_id_hi[1205]=4'h0; text_id_hi[1206]=4'h0; text_id_hi[1207]=4'h0;
text_id_hi[1208]=4'h0; text_id_hi[1209]=4'h0; text_id_hi[1210]=4'h0; text_id_hi[1211]=4'h0; text_id_hi[1212]=4'h0; text_id_hi[1213]=4'h0; text_id_hi[1214]=4'h0; text_id_hi[1215]=4'h0;
text_id_hi[1216]=4'h0; text_id_hi[1217]=4'h0; text_id_hi[1218]=4'h0; text_id_hi[1219]=4'h0; text_id_hi[1220]=4'h0; text_id_hi[1221]=4'h0; text_id_hi[1222]=4'h0; text_id_hi[1223]=4'h0;
text_id_hi[1224]=4'h0; text_id_hi[1225]=4'h0; text_id_hi[1226]=4'h0; text_id_hi[1227]=4'h0; text_id_hi[1228]=4'h0; text_id_hi[1229]=4'h0; text_id_hi[1230]=4'h0; text_id_hi[1231]=4'h0;
text_id_hi[1232]=4'h0; text_id_hi[1233]=4'h0; text_id_hi[1234]=4'h0; text_id_hi[1235]=4'h0; text_id_hi[1236]=4'h0; text_id_hi[1237]=4'h0; text_id_hi[1238]=4'h0; text_id_hi[1239]=4'h0;
text_id_hi[1240]=4'h0; text_id_hi[1241]=4'h0; text_id_hi[1242]=4'h0; text_id_hi[1243]=4'h0; text_id_hi[1244]=4'h0; text_id_hi[1245]=4'h0; text_id_hi[1246]=4'h0; text_id_hi[1247]=4'h0;
text_id_hi[1248]=4'h0; text_id_hi[1249]=4'h0; text_id_hi[1250]=4'h0; text_id_hi[1251]=4'h0; text_id_hi[1252]=4'h0; text_id_hi[1253]=4'h0; text_id_hi[1254]=4'h0; text_id_hi[1255]=4'h0;
text_id_hi[1256]=4'h0; text_id_hi[1257]=4'h0; text_id_hi[1258]=4'h0; text_id_hi[1259]=4'h0; text_id_hi[1260]=4'h0; text_id_hi[1261]=4'h0; text_id_hi[1262]=4'h0; text_id_hi[1263]=4'h0;
text_id_hi[1264]=4'h0; text_id_hi[1265]=4'h0; text_id_hi[1266]=4'h0; text_id_hi[1267]=4'h0; text_id_hi[1268]=4'h0; text_id_hi[1269]=4'h0; text_id_hi[1270]=4'h0; text_id_hi[1271]=4'h0;
text_id_hi[1272]=4'h0; text_id_hi[1273]=4'h0; text_id_hi[1274]=4'h0; text_id_hi[1275]=4'h0; text_id_hi[1276]=4'h0; text_id_hi[1277]=4'h0; text_id_hi[1278]=4'h0; text_id_hi[1279]=4'h0;
text_id_hi[1280]=4'h4; text_id_hi[1281]=4'h3; text_id_hi[1282]=4'h0; text_id_hi[1283]=4'h1; text_id_hi[1284]=4'h5; text_id_hi[1285]=4'hc; text_id_hi[1286]=4'hc; text_id_hi[1287]=4'h2;
text_id_hi[1288]=4'h4; text_id_hi[1289]=4'he; text_id_hi[1290]=4'h7; text_id_hi[1291]=4'h8; text_id_hi[1292]=4'h5; text_id_hi[1293]=4'hd; text_id_hi[1294]=4'h7; text_id_hi[1295]=4'hd;
text_id_hi[1296]=4'h8; text_id_hi[1297]=4'h7; text_id_hi[1298]=4'h0; text_id_hi[1299]=4'h1; text_id_hi[1300]=4'h7; text_id_hi[1301]=4'hb; text_id_hi[1302]=4'h6; text_id_hi[1303]=4'hd;
text_id_hi[1304]=4'h2; text_id_hi[1305]=4'h9; text_id_hi[1306]=4'h8; text_id_hi[1307]=4'h5; text_id_hi[1308]=4'h5; text_id_hi[1309]=4'he; text_id_hi[1310]=4'hb; text_id_hi[1311]=4'h6;
text_id_hi[1312]=4'h6; text_id_hi[1313]=4'h4; text_id_hi[1314]=4'h5; text_id_hi[1315]=4'h7; text_id_hi[1316]=4'hd; text_id_hi[1317]=4'ha; text_id_hi[1318]=4'hc; text_id_hi[1319]=4'ha;
text_id_hi[1320]=4'h7; text_id_hi[1321]=4'h6; text_id_hi[1322]=4'h4; text_id_hi[1323]=4'hb; text_id_hi[1324]=4'hb; text_id_hi[1325]=4'hd; text_id_hi[1326]=4'h9; text_id_hi[1327]=4'ha;
text_id_hi[1328]=4'h6; text_id_hi[1329]=4'h6; text_id_hi[1330]=4'h7; text_id_hi[1331]=4'h1; text_id_hi[1332]=4'h2; text_id_hi[1333]=4'hb; text_id_hi[1334]=4'h0; text_id_hi[1335]=4'h0;
text_id_hi[1336]=4'h0; text_id_hi[1337]=4'h0; text_id_hi[1338]=4'h0; text_id_hi[1339]=4'h0; text_id_hi[1340]=4'h0; text_id_hi[1341]=4'h0; text_id_hi[1342]=4'h0; text_id_hi[1343]=4'h0;
text_id_hi[1344]=4'h0; text_id_hi[1345]=4'h0; text_id_hi[1346]=4'h0; text_id_hi[1347]=4'h0; text_id_hi[1348]=4'h0; text_id_hi[1349]=4'h0; text_id_hi[1350]=4'h0; text_id_hi[1351]=4'h0;
text_id_hi[1352]=4'h0; text_id_hi[1353]=4'h0; text_id_hi[1354]=4'h0; text_id_hi[1355]=4'h0; text_id_hi[1356]=4'h0; text_id_hi[1357]=4'h0; text_id_hi[1358]=4'h0; text_id_hi[1359]=4'h0;
text_id_hi[1360]=4'h0; text_id_hi[1361]=4'h0; text_id_hi[1362]=4'h0; text_id_hi[1363]=4'h0; text_id_hi[1364]=4'h0; text_id_hi[1365]=4'h0; text_id_hi[1366]=4'h0; text_id_hi[1367]=4'h0;
text_id_hi[1368]=4'h0; text_id_hi[1369]=4'h0; text_id_hi[1370]=4'h0; text_id_hi[1371]=4'h0; text_id_hi[1372]=4'h0; text_id_hi[1373]=4'h0; text_id_hi[1374]=4'h0; text_id_hi[1375]=4'h0;
text_id_hi[1376]=4'h0; text_id_hi[1377]=4'h0; text_id_hi[1378]=4'h0; text_id_hi[1379]=4'h0; text_id_hi[1380]=4'h0; text_id_hi[1381]=4'h0; text_id_hi[1382]=4'h0; text_id_hi[1383]=4'h0;
text_id_hi[1384]=4'h0; text_id_hi[1385]=4'h0; text_id_hi[1386]=4'h0; text_id_hi[1387]=4'h0; text_id_hi[1388]=4'h0; text_id_hi[1389]=4'h0; text_id_hi[1390]=4'h0; text_id_hi[1391]=4'h0;
text_id_hi[1392]=4'h0; text_id_hi[1393]=4'h0; text_id_hi[1394]=4'h0; text_id_hi[1395]=4'h0; text_id_hi[1396]=4'h0; text_id_hi[1397]=4'h0; text_id_hi[1398]=4'h0; text_id_hi[1399]=4'h0;
text_id_hi[1400]=4'h0; text_id_hi[1401]=4'h0; text_id_hi[1402]=4'h0; text_id_hi[1403]=4'h0; text_id_hi[1404]=4'h0; text_id_hi[1405]=4'h0; text_id_hi[1406]=4'h0; text_id_hi[1407]=4'h0;
text_id_hi[1408]=4'h2; text_id_hi[1409]=4'hb; text_id_hi[1410]=4'ha; text_id_hi[1411]=4'h6; text_id_hi[1412]=4'h0; text_id_hi[1413]=4'h1; text_id_hi[1414]=4'h9; text_id_hi[1415]=4'h3;
text_id_hi[1416]=4'h0; text_id_hi[1417]=4'hd; text_id_hi[1418]=4'h2; text_id_hi[1419]=4'h8; text_id_hi[1420]=4'h2; text_id_hi[1421]=4'h9; text_id_hi[1422]=4'h3; text_id_hi[1423]=4'h3;
text_id_hi[1424]=4'h0; text_id_hi[1425]=4'h9; text_id_hi[1426]=4'h3; text_id_hi[1427]=4'h7; text_id_hi[1428]=4'h0; text_id_hi[1429]=4'h9; text_id_hi[1430]=4'h3; text_id_hi[1431]=4'h4;
text_id_hi[1432]=4'hd; text_id_hi[1433]=4'h6; text_id_hi[1434]=4'h8; text_id_hi[1435]=4'h3; text_id_hi[1436]=4'h6; text_id_hi[1437]=4'he; text_id_hi[1438]=4'h7; text_id_hi[1439]=4'h5;
text_id_hi[1440]=4'ha; text_id_hi[1441]=4'h8; text_id_hi[1442]=4'h0; text_id_hi[1443]=4'ha; text_id_hi[1444]=4'h7; text_id_hi[1445]=4'hb; text_id_hi[1446]=4'ha; text_id_hi[1447]=4'h2;
text_id_hi[1448]=4'hd; text_id_hi[1449]=4'h7; text_id_hi[1450]=4'h5; text_id_hi[1451]=4'h1; text_id_hi[1452]=4'h2; text_id_hi[1453]=4'hd; text_id_hi[1454]=4'hd; text_id_hi[1455]=4'h6;
text_id_hi[1456]=4'h4; text_id_hi[1457]=4'h4; text_id_hi[1458]=4'h0; text_id_hi[1459]=4'h0; text_id_hi[1460]=4'h0; text_id_hi[1461]=4'h0; text_id_hi[1462]=4'h0; text_id_hi[1463]=4'h0;
text_id_hi[1464]=4'h0; text_id_hi[1465]=4'h0; text_id_hi[1466]=4'h0; text_id_hi[1467]=4'h0; text_id_hi[1468]=4'h0; text_id_hi[1469]=4'h0; text_id_hi[1470]=4'h0; text_id_hi[1471]=4'h0;
text_id_hi[1472]=4'h0; text_id_hi[1473]=4'h0; text_id_hi[1474]=4'h0; text_id_hi[1475]=4'h0; text_id_hi[1476]=4'h0; text_id_hi[1477]=4'h0; text_id_hi[1478]=4'h0; text_id_hi[1479]=4'h0;
text_id_hi[1480]=4'h0; text_id_hi[1481]=4'h0; text_id_hi[1482]=4'h0; text_id_hi[1483]=4'h0; text_id_hi[1484]=4'h0; text_id_hi[1485]=4'h0; text_id_hi[1486]=4'h0; text_id_hi[1487]=4'h0;
text_id_hi[1488]=4'h0; text_id_hi[1489]=4'h0; text_id_hi[1490]=4'h0; text_id_hi[1491]=4'h0; text_id_hi[1492]=4'h0; text_id_hi[1493]=4'h0; text_id_hi[1494]=4'h0; text_id_hi[1495]=4'h0;
text_id_hi[1496]=4'h0; text_id_hi[1497]=4'h0; text_id_hi[1498]=4'h0; text_id_hi[1499]=4'h0; text_id_hi[1500]=4'h0; text_id_hi[1501]=4'h0; text_id_hi[1502]=4'h0; text_id_hi[1503]=4'h0;
text_id_hi[1504]=4'h0; text_id_hi[1505]=4'h0; text_id_hi[1506]=4'h0; text_id_hi[1507]=4'h0; text_id_hi[1508]=4'h0; text_id_hi[1509]=4'h0; text_id_hi[1510]=4'h0; text_id_hi[1511]=4'h0;
text_id_hi[1512]=4'h0; text_id_hi[1513]=4'h0; text_id_hi[1514]=4'h0; text_id_hi[1515]=4'h0; text_id_hi[1516]=4'h0; text_id_hi[1517]=4'h0; text_id_hi[1518]=4'h0; text_id_hi[1519]=4'h0;
text_id_hi[1520]=4'h0; text_id_hi[1521]=4'h0; text_id_hi[1522]=4'h0; text_id_hi[1523]=4'h0; text_id_hi[1524]=4'h0; text_id_hi[1525]=4'h0; text_id_hi[1526]=4'h0; text_id_hi[1527]=4'h0;
text_id_hi[1528]=4'h0; text_id_hi[1529]=4'h0; text_id_hi[1530]=4'h0; text_id_hi[1531]=4'h0; text_id_hi[1532]=4'h0; text_id_hi[1533]=4'h0; text_id_hi[1534]=4'h0; text_id_hi[1535]=4'h0;
text_id_hi[1536]=4'h6; text_id_hi[1537]=4'h9; text_id_hi[1538]=4'h2; text_id_hi[1539]=4'h1; text_id_hi[1540]=4'h3; text_id_hi[1541]=4'h8; text_id_hi[1542]=4'ha; text_id_hi[1543]=4'h7;
text_id_hi[1544]=4'hd; text_id_hi[1545]=4'hb; text_id_hi[1546]=4'hb; text_id_hi[1547]=4'h2; text_id_hi[1548]=4'h4; text_id_hi[1549]=4'h0; text_id_hi[1550]=4'h0; text_id_hi[1551]=4'h0;
text_id_hi[1552]=4'h0; text_id_hi[1553]=4'h0; text_id_hi[1554]=4'h0; text_id_hi[1555]=4'h0; text_id_hi[1556]=4'h0; text_id_hi[1557]=4'h0; text_id_hi[1558]=4'h0; text_id_hi[1559]=4'h0;
text_id_hi[1560]=4'h0; text_id_hi[1561]=4'h0; text_id_hi[1562]=4'h0; text_id_hi[1563]=4'h0; text_id_hi[1564]=4'h0; text_id_hi[1565]=4'h0; text_id_hi[1566]=4'h0; text_id_hi[1567]=4'h0;
text_id_hi[1568]=4'h0; text_id_hi[1569]=4'h0; text_id_hi[1570]=4'h0; text_id_hi[1571]=4'h0; text_id_hi[1572]=4'h0; text_id_hi[1573]=4'h0; text_id_hi[1574]=4'h0; text_id_hi[1575]=4'h0;
text_id_hi[1576]=4'h0; text_id_hi[1577]=4'h0; text_id_hi[1578]=4'h0; text_id_hi[1579]=4'h0; text_id_hi[1580]=4'h0; text_id_hi[1581]=4'h0; text_id_hi[1582]=4'h0; text_id_hi[1583]=4'h0;
text_id_hi[1584]=4'h0; text_id_hi[1585]=4'h0; text_id_hi[1586]=4'h0; text_id_hi[1587]=4'h0; text_id_hi[1588]=4'h0; text_id_hi[1589]=4'h0; text_id_hi[1590]=4'h0; text_id_hi[1591]=4'h0;
text_id_hi[1592]=4'h0; text_id_hi[1593]=4'h0; text_id_hi[1594]=4'h0; text_id_hi[1595]=4'h0; text_id_hi[1596]=4'h0; text_id_hi[1597]=4'h0; text_id_hi[1598]=4'h0; text_id_hi[1599]=4'h0;
text_id_hi[1600]=4'h0; text_id_hi[1601]=4'h0; text_id_hi[1602]=4'h0; text_id_hi[1603]=4'h0; text_id_hi[1604]=4'h0; text_id_hi[1605]=4'h0; text_id_hi[1606]=4'h0; text_id_hi[1607]=4'h0;
text_id_hi[1608]=4'h0; text_id_hi[1609]=4'h0; text_id_hi[1610]=4'h0; text_id_hi[1611]=4'h0; text_id_hi[1612]=4'h0; text_id_hi[1613]=4'h0; text_id_hi[1614]=4'h0; text_id_hi[1615]=4'h0;
text_id_hi[1616]=4'h0; text_id_hi[1617]=4'h0; text_id_hi[1618]=4'h0; text_id_hi[1619]=4'h0; text_id_hi[1620]=4'h0; text_id_hi[1621]=4'h0; text_id_hi[1622]=4'h0; text_id_hi[1623]=4'h0;
text_id_hi[1624]=4'h0; text_id_hi[1625]=4'h0; text_id_hi[1626]=4'h0; text_id_hi[1627]=4'h0; text_id_hi[1628]=4'h0; text_id_hi[1629]=4'h0; text_id_hi[1630]=4'h0; text_id_hi[1631]=4'h0;
text_id_hi[1632]=4'h0; text_id_hi[1633]=4'h0; text_id_hi[1634]=4'h0; text_id_hi[1635]=4'h0; text_id_hi[1636]=4'h0; text_id_hi[1637]=4'h0; text_id_hi[1638]=4'h0; text_id_hi[1639]=4'h0;
text_id_hi[1640]=4'h0; text_id_hi[1641]=4'h0; text_id_hi[1642]=4'h0; text_id_hi[1643]=4'h0; text_id_hi[1644]=4'h0; text_id_hi[1645]=4'h0; text_id_hi[1646]=4'h0; text_id_hi[1647]=4'h0;
text_id_hi[1648]=4'h0; text_id_hi[1649]=4'h0; text_id_hi[1650]=4'h0; text_id_hi[1651]=4'h0; text_id_hi[1652]=4'h0; text_id_hi[1653]=4'h0; text_id_hi[1654]=4'h0; text_id_hi[1655]=4'h0;
text_id_hi[1656]=4'h0; text_id_hi[1657]=4'h0; text_id_hi[1658]=4'h0; text_id_hi[1659]=4'h0; text_id_hi[1660]=4'h0; text_id_hi[1661]=4'h0; text_id_hi[1662]=4'h0; text_id_hi[1663]=4'h0;
text_id_hi[1664]=4'h0; text_id_hi[1665]=4'h0; text_id_hi[1666]=4'h0; text_id_hi[1667]=4'h0; text_id_hi[1668]=4'h0; text_id_hi[1669]=4'h0; text_id_hi[1670]=4'h0; text_id_hi[1671]=4'h0;
text_id_hi[1672]=4'h0; text_id_hi[1673]=4'h0; text_id_hi[1674]=4'h0; text_id_hi[1675]=4'h0; text_id_hi[1676]=4'h0; text_id_hi[1677]=4'h0; text_id_hi[1678]=4'h0; text_id_hi[1679]=4'h0;
text_id_hi[1680]=4'h0; text_id_hi[1681]=4'h0; text_id_hi[1682]=4'h0; text_id_hi[1683]=4'h0; text_id_hi[1684]=4'h0; text_id_hi[1685]=4'h0; text_id_hi[1686]=4'h0; text_id_hi[1687]=4'h0;
text_id_hi[1688]=4'h0; text_id_hi[1689]=4'h0; text_id_hi[1690]=4'h0; text_id_hi[1691]=4'h0; text_id_hi[1692]=4'h0; text_id_hi[1693]=4'h0; text_id_hi[1694]=4'h0; text_id_hi[1695]=4'h0;
text_id_hi[1696]=4'h0; text_id_hi[1697]=4'h0; text_id_hi[1698]=4'h0; text_id_hi[1699]=4'h0; text_id_hi[1700]=4'h0; text_id_hi[1701]=4'h0; text_id_hi[1702]=4'h0; text_id_hi[1703]=4'h0;
text_id_hi[1704]=4'h0; text_id_hi[1705]=4'h0; text_id_hi[1706]=4'h0; text_id_hi[1707]=4'h0; text_id_hi[1708]=4'h0; text_id_hi[1709]=4'h0; text_id_hi[1710]=4'h0; text_id_hi[1711]=4'h0;
text_id_hi[1712]=4'h0; text_id_hi[1713]=4'h0; text_id_hi[1714]=4'h0; text_id_hi[1715]=4'h0; text_id_hi[1716]=4'h0; text_id_hi[1717]=4'h0; text_id_hi[1718]=4'h0; text_id_hi[1719]=4'h0;
text_id_hi[1720]=4'h0; text_id_hi[1721]=4'h0; text_id_hi[1722]=4'h0; text_id_hi[1723]=4'h0; text_id_hi[1724]=4'h0; text_id_hi[1725]=4'h0; text_id_hi[1726]=4'h0; text_id_hi[1727]=4'h0;
text_id_hi[1728]=4'h0; text_id_hi[1729]=4'h0; text_id_hi[1730]=4'h0; text_id_hi[1731]=4'h0; text_id_hi[1732]=4'h0; text_id_hi[1733]=4'h0; text_id_hi[1734]=4'h0; text_id_hi[1735]=4'h0;
text_id_hi[1736]=4'h0; text_id_hi[1737]=4'h0; text_id_hi[1738]=4'h0; text_id_hi[1739]=4'h0; text_id_hi[1740]=4'h0; text_id_hi[1741]=4'h0; text_id_hi[1742]=4'h0; text_id_hi[1743]=4'h0;
text_id_hi[1744]=4'h0; text_id_hi[1745]=4'h0; text_id_hi[1746]=4'h0; text_id_hi[1747]=4'h0; text_id_hi[1748]=4'h0; text_id_hi[1749]=4'h0; text_id_hi[1750]=4'h0; text_id_hi[1751]=4'h0;
text_id_hi[1752]=4'h0; text_id_hi[1753]=4'h0; text_id_hi[1754]=4'h0; text_id_hi[1755]=4'h0; text_id_hi[1756]=4'h0; text_id_hi[1757]=4'h0; text_id_hi[1758]=4'h0; text_id_hi[1759]=4'h0;
text_id_hi[1760]=4'h0; text_id_hi[1761]=4'h0; text_id_hi[1762]=4'h0; text_id_hi[1763]=4'h0; text_id_hi[1764]=4'h0; text_id_hi[1765]=4'h0; text_id_hi[1766]=4'h0; text_id_hi[1767]=4'h0;
text_id_hi[1768]=4'h0; text_id_hi[1769]=4'h0; text_id_hi[1770]=4'h0; text_id_hi[1771]=4'h0; text_id_hi[1772]=4'h0; text_id_hi[1773]=4'h0; text_id_hi[1774]=4'h0; text_id_hi[1775]=4'h0;
text_id_hi[1776]=4'h0; text_id_hi[1777]=4'h0; text_id_hi[1778]=4'h0; text_id_hi[1779]=4'h0; text_id_hi[1780]=4'h0; text_id_hi[1781]=4'h0; text_id_hi[1782]=4'h0; text_id_hi[1783]=4'h0;
text_id_hi[1784]=4'h0; text_id_hi[1785]=4'h0; text_id_hi[1786]=4'h0; text_id_hi[1787]=4'h0; text_id_hi[1788]=4'h0; text_id_hi[1789]=4'h0; text_id_hi[1790]=4'h0; text_id_hi[1791]=4'h0;
text_id_hi[1792]=4'h0; text_id_hi[1793]=4'h0; text_id_hi[1794]=4'h0; text_id_hi[1795]=4'h0; text_id_hi[1796]=4'h0; text_id_hi[1797]=4'h0; text_id_hi[1798]=4'h0; text_id_hi[1799]=4'h0;
text_id_hi[1800]=4'h0; text_id_hi[1801]=4'h0; text_id_hi[1802]=4'h0; text_id_hi[1803]=4'h0; text_id_hi[1804]=4'h0; text_id_hi[1805]=4'h0; text_id_hi[1806]=4'h0; text_id_hi[1807]=4'h0;
text_id_hi[1808]=4'h0; text_id_hi[1809]=4'h0; text_id_hi[1810]=4'h0; text_id_hi[1811]=4'h0; text_id_hi[1812]=4'h0; text_id_hi[1813]=4'h0; text_id_hi[1814]=4'h0; text_id_hi[1815]=4'h0;
text_id_hi[1816]=4'h0; text_id_hi[1817]=4'h0; text_id_hi[1818]=4'h0; text_id_hi[1819]=4'h0; text_id_hi[1820]=4'h0; text_id_hi[1821]=4'h0; text_id_hi[1822]=4'h0; text_id_hi[1823]=4'h0;
text_id_hi[1824]=4'h0; text_id_hi[1825]=4'h0; text_id_hi[1826]=4'h0; text_id_hi[1827]=4'h0; text_id_hi[1828]=4'h0; text_id_hi[1829]=4'h0; text_id_hi[1830]=4'h0; text_id_hi[1831]=4'h0;
text_id_hi[1832]=4'h0; text_id_hi[1833]=4'h0; text_id_hi[1834]=4'h0; text_id_hi[1835]=4'h0; text_id_hi[1836]=4'h0; text_id_hi[1837]=4'h0; text_id_hi[1838]=4'h0; text_id_hi[1839]=4'h0;
text_id_hi[1840]=4'h0; text_id_hi[1841]=4'h0; text_id_hi[1842]=4'h0; text_id_hi[1843]=4'h0; text_id_hi[1844]=4'h0; text_id_hi[1845]=4'h0; text_id_hi[1846]=4'h0; text_id_hi[1847]=4'h0;
text_id_hi[1848]=4'h0; text_id_hi[1849]=4'h0; text_id_hi[1850]=4'h0; text_id_hi[1851]=4'h0; text_id_hi[1852]=4'h0; text_id_hi[1853]=4'h0; text_id_hi[1854]=4'h0; text_id_hi[1855]=4'h0;
text_id_hi[1856]=4'h0; text_id_hi[1857]=4'h0; text_id_hi[1858]=4'h0; text_id_hi[1859]=4'h0; text_id_hi[1860]=4'h0; text_id_hi[1861]=4'h0; text_id_hi[1862]=4'h0; text_id_hi[1863]=4'h0;
text_id_hi[1864]=4'h0; text_id_hi[1865]=4'h0; text_id_hi[1866]=4'h0; text_id_hi[1867]=4'h0; text_id_hi[1868]=4'h0; text_id_hi[1869]=4'h0; text_id_hi[1870]=4'h0; text_id_hi[1871]=4'h0;
text_id_hi[1872]=4'h0; text_id_hi[1873]=4'h0; text_id_hi[1874]=4'h0; text_id_hi[1875]=4'h0; text_id_hi[1876]=4'h0; text_id_hi[1877]=4'h0; text_id_hi[1878]=4'h0; text_id_hi[1879]=4'h0;
text_id_hi[1880]=4'h0; text_id_hi[1881]=4'h0; text_id_hi[1882]=4'h0; text_id_hi[1883]=4'h0; text_id_hi[1884]=4'h0; text_id_hi[1885]=4'h0; text_id_hi[1886]=4'h0; text_id_hi[1887]=4'h0;
text_id_hi[1888]=4'h0; text_id_hi[1889]=4'h0; text_id_hi[1890]=4'h0; text_id_hi[1891]=4'h0; text_id_hi[1892]=4'h0; text_id_hi[1893]=4'h0; text_id_hi[1894]=4'h0; text_id_hi[1895]=4'h0;
text_id_hi[1896]=4'h0; text_id_hi[1897]=4'h0; text_id_hi[1898]=4'h0; text_id_hi[1899]=4'h0; text_id_hi[1900]=4'h0; text_id_hi[1901]=4'h0; text_id_hi[1902]=4'h0; text_id_hi[1903]=4'h0;
text_id_hi[1904]=4'h0; text_id_hi[1905]=4'h0; text_id_hi[1906]=4'h0; text_id_hi[1907]=4'h0; text_id_hi[1908]=4'h0; text_id_hi[1909]=4'h0; text_id_hi[1910]=4'h0; text_id_hi[1911]=4'h0;
text_id_hi[1912]=4'h0; text_id_hi[1913]=4'h0; text_id_hi[1914]=4'h0; text_id_hi[1915]=4'h0; text_id_hi[1916]=4'h0; text_id_hi[1917]=4'h0; text_id_hi[1918]=4'h0; text_id_hi[1919]=4'h0;
text_id_hi[1920]=4'h0; text_id_hi[1921]=4'h0; text_id_hi[1922]=4'h0; text_id_hi[1923]=4'h0; text_id_hi[1924]=4'h0; text_id_hi[1925]=4'h0; text_id_hi[1926]=4'h0; text_id_hi[1927]=4'h0;
text_id_hi[1928]=4'h0; text_id_hi[1929]=4'h0; text_id_hi[1930]=4'h0; text_id_hi[1931]=4'h0; text_id_hi[1932]=4'h0; text_id_hi[1933]=4'h0; text_id_hi[1934]=4'h0; text_id_hi[1935]=4'h0;
text_id_hi[1936]=4'h0; text_id_hi[1937]=4'h0; text_id_hi[1938]=4'h0; text_id_hi[1939]=4'h0; text_id_hi[1940]=4'h0; text_id_hi[1941]=4'h0; text_id_hi[1942]=4'h0; text_id_hi[1943]=4'h0;
text_id_hi[1944]=4'h0; text_id_hi[1945]=4'h0; text_id_hi[1946]=4'h0; text_id_hi[1947]=4'h0; text_id_hi[1948]=4'h0; text_id_hi[1949]=4'h0; text_id_hi[1950]=4'h0; text_id_hi[1951]=4'h0;
text_id_hi[1952]=4'h0; text_id_hi[1953]=4'h0; text_id_hi[1954]=4'h0; text_id_hi[1955]=4'h0; text_id_hi[1956]=4'h0; text_id_hi[1957]=4'h0; text_id_hi[1958]=4'h0; text_id_hi[1959]=4'h0;
text_id_hi[1960]=4'h0; text_id_hi[1961]=4'h0; text_id_hi[1962]=4'h0; text_id_hi[1963]=4'h0; text_id_hi[1964]=4'h0; text_id_hi[1965]=4'h0; text_id_hi[1966]=4'h0; text_id_hi[1967]=4'h0;
text_id_hi[1968]=4'h0; text_id_hi[1969]=4'h0; text_id_hi[1970]=4'h0; text_id_hi[1971]=4'h0; text_id_hi[1972]=4'h0; text_id_hi[1973]=4'h0; text_id_hi[1974]=4'h0; text_id_hi[1975]=4'h0;
text_id_hi[1976]=4'h0; text_id_hi[1977]=4'h0; text_id_hi[1978]=4'h0; text_id_hi[1979]=4'h0; text_id_hi[1980]=4'h0; text_id_hi[1981]=4'h0; text_id_hi[1982]=4'h0; text_id_hi[1983]=4'h0;
text_id_hi[1984]=4'h0; text_id_hi[1985]=4'h0; text_id_hi[1986]=4'h0; text_id_hi[1987]=4'h0; text_id_hi[1988]=4'h0; text_id_hi[1989]=4'h0; text_id_hi[1990]=4'h0; text_id_hi[1991]=4'h0;
text_id_hi[1992]=4'h0; text_id_hi[1993]=4'h0; text_id_hi[1994]=4'h0; text_id_hi[1995]=4'h0; text_id_hi[1996]=4'h0; text_id_hi[1997]=4'h0; text_id_hi[1998]=4'h0; text_id_hi[1999]=4'h0;
text_id_hi[2000]=4'h0; text_id_hi[2001]=4'h0; text_id_hi[2002]=4'h0; text_id_hi[2003]=4'h0; text_id_hi[2004]=4'h0; text_id_hi[2005]=4'h0; text_id_hi[2006]=4'h0; text_id_hi[2007]=4'h0;
text_id_hi[2008]=4'h0; text_id_hi[2009]=4'h0; text_id_hi[2010]=4'h0; text_id_hi[2011]=4'h0; text_id_hi[2012]=4'h0; text_id_hi[2013]=4'h0; text_id_hi[2014]=4'h0; text_id_hi[2015]=4'h0;
text_id_hi[2016]=4'h0; text_id_hi[2017]=4'h0; text_id_hi[2018]=4'h0; text_id_hi[2019]=4'h0; text_id_hi[2020]=4'h0; text_id_hi[2021]=4'h0; text_id_hi[2022]=4'h0; text_id_hi[2023]=4'h0;
text_id_hi[2024]=4'h0; text_id_hi[2025]=4'h0; text_id_hi[2026]=4'h0; text_id_hi[2027]=4'h0; text_id_hi[2028]=4'h0; text_id_hi[2029]=4'h0; text_id_hi[2030]=4'h0; text_id_hi[2031]=4'h0;
text_id_hi[2032]=4'h0; text_id_hi[2033]=4'h0; text_id_hi[2034]=4'h0; text_id_hi[2035]=4'h0; text_id_hi[2036]=4'h0; text_id_hi[2037]=4'h0; text_id_hi[2038]=4'h0; text_id_hi[2039]=4'h0;
text_id_hi[2040]=4'h0; text_id_hi[2041]=4'h0; text_id_hi[2042]=4'h0; text_id_hi[2043]=4'h0; text_id_hi[2044]=4'h0; text_id_hi[2045]=4'h0; text_id_hi[2046]=4'h0; text_id_hi[2047]=4'h0;
end
reg [7:0] focus_id_rom [0:1023]; /* fehdl force_ram=1, ram_style="bram" */
initial begin
focus_id_rom[0]=8'h0; focus_id_rom[1]=8'h0; focus_id_rom[2]=8'h0; focus_id_rom[3]=8'h0; focus_id_rom[4]=8'h0; focus_id_rom[5]=8'h0; focus_id_rom[6]=8'h0; focus_id_rom[7]=8'h0;
focus_id_rom[8]=8'h0; focus_id_rom[9]=8'h0; focus_id_rom[10]=8'h0; focus_id_rom[11]=8'h0; focus_id_rom[12]=8'h0; focus_id_rom[13]=8'h0; focus_id_rom[14]=8'h0; focus_id_rom[15]=8'h0;
focus_id_rom[16]=8'h0; focus_id_rom[17]=8'h0; focus_id_rom[18]=8'h0; focus_id_rom[19]=8'h0; focus_id_rom[20]=8'h0; focus_id_rom[21]=8'h0; focus_id_rom[22]=8'h0; focus_id_rom[23]=8'h0;
focus_id_rom[24]=8'h0; focus_id_rom[25]=8'h0; focus_id_rom[26]=8'h0; focus_id_rom[27]=8'h0; focus_id_rom[28]=8'h0; focus_id_rom[29]=8'h0; focus_id_rom[30]=8'h0; focus_id_rom[31]=8'h0;
focus_id_rom[32]=8'h0; focus_id_rom[33]=8'h0; focus_id_rom[34]=8'h0; focus_id_rom[35]=8'h0; focus_id_rom[36]=8'h0; focus_id_rom[37]=8'h0; focus_id_rom[38]=8'h0; focus_id_rom[39]=8'h0;
focus_id_rom[40]=8'h0; focus_id_rom[41]=8'h0; focus_id_rom[42]=8'h0; focus_id_rom[43]=8'h0; focus_id_rom[44]=8'h0; focus_id_rom[45]=8'h0; focus_id_rom[46]=8'h0; focus_id_rom[47]=8'h0;
focus_id_rom[48]=8'h0; focus_id_rom[49]=8'h0; focus_id_rom[50]=8'h0; focus_id_rom[51]=8'h0; focus_id_rom[52]=8'h0; focus_id_rom[53]=8'h0; focus_id_rom[54]=8'h0; focus_id_rom[55]=8'h0;
focus_id_rom[56]=8'h0; focus_id_rom[57]=8'h0; focus_id_rom[58]=8'h0; focus_id_rom[59]=8'h0; focus_id_rom[60]=8'h0; focus_id_rom[61]=8'h0; focus_id_rom[62]=8'h0; focus_id_rom[63]=8'h0;
focus_id_rom[64]=8'h0; focus_id_rom[65]=8'h0; focus_id_rom[66]=8'h0; focus_id_rom[67]=8'h0; focus_id_rom[68]=8'h0; focus_id_rom[69]=8'h0; focus_id_rom[70]=8'h0; focus_id_rom[71]=8'h0;
focus_id_rom[72]=8'h0; focus_id_rom[73]=8'h0; focus_id_rom[74]=8'h0; focus_id_rom[75]=8'h0; focus_id_rom[76]=8'h0; focus_id_rom[77]=8'h0; focus_id_rom[78]=8'h0; focus_id_rom[79]=8'h0;
focus_id_rom[80]=8'h0; focus_id_rom[81]=8'h0; focus_id_rom[82]=8'h0; focus_id_rom[83]=8'h0; focus_id_rom[84]=8'h0; focus_id_rom[85]=8'h0; focus_id_rom[86]=8'h0; focus_id_rom[87]=8'h0;
focus_id_rom[88]=8'h0; focus_id_rom[89]=8'h0; focus_id_rom[90]=8'h0; focus_id_rom[91]=8'h0; focus_id_rom[92]=8'h0; focus_id_rom[93]=8'h0; focus_id_rom[94]=8'h0; focus_id_rom[95]=8'h0;
focus_id_rom[96]=8'h0; focus_id_rom[97]=8'h0; focus_id_rom[98]=8'h0; focus_id_rom[99]=8'h0; focus_id_rom[100]=8'h0; focus_id_rom[101]=8'h0; focus_id_rom[102]=8'h0; focus_id_rom[103]=8'h0;
focus_id_rom[104]=8'h0; focus_id_rom[105]=8'h0; focus_id_rom[106]=8'h0; focus_id_rom[107]=8'h0; focus_id_rom[108]=8'h0; focus_id_rom[109]=8'h0; focus_id_rom[110]=8'h0; focus_id_rom[111]=8'h0;
focus_id_rom[112]=8'h0; focus_id_rom[113]=8'h0; focus_id_rom[114]=8'h0; focus_id_rom[115]=8'h0; focus_id_rom[116]=8'h0; focus_id_rom[117]=8'h0; focus_id_rom[118]=8'h0; focus_id_rom[119]=8'h0;
focus_id_rom[120]=8'h0; focus_id_rom[121]=8'h0; focus_id_rom[122]=8'h0; focus_id_rom[123]=8'h0; focus_id_rom[124]=8'h0; focus_id_rom[125]=8'h0; focus_id_rom[126]=8'h0; focus_id_rom[127]=8'h0;
focus_id_rom[128]=8'h0; focus_id_rom[129]=8'h0; focus_id_rom[130]=8'h0; focus_id_rom[131]=8'h0; focus_id_rom[132]=8'h0; focus_id_rom[133]=8'h0; focus_id_rom[134]=8'h0; focus_id_rom[135]=8'h0;
focus_id_rom[136]=8'h0; focus_id_rom[137]=8'h0; focus_id_rom[138]=8'h0; focus_id_rom[139]=8'h0; focus_id_rom[140]=8'h0; focus_id_rom[141]=8'h0; focus_id_rom[142]=8'h0; focus_id_rom[143]=8'h0;
focus_id_rom[144]=8'h0; focus_id_rom[145]=8'h0; focus_id_rom[146]=8'h0; focus_id_rom[147]=8'h0; focus_id_rom[148]=8'h0; focus_id_rom[149]=8'h0; focus_id_rom[150]=8'h0; focus_id_rom[151]=8'h0;
focus_id_rom[152]=8'h0; focus_id_rom[153]=8'h0; focus_id_rom[154]=8'h0; focus_id_rom[155]=8'h0; focus_id_rom[156]=8'h0; focus_id_rom[157]=8'h0; focus_id_rom[158]=8'h0; focus_id_rom[159]=8'h0;
focus_id_rom[160]=8'h0; focus_id_rom[161]=8'h0; focus_id_rom[162]=8'h0; focus_id_rom[163]=8'h0; focus_id_rom[164]=8'h0; focus_id_rom[165]=8'h0; focus_id_rom[166]=8'h0; focus_id_rom[167]=8'h0;
focus_id_rom[168]=8'h0; focus_id_rom[169]=8'h0; focus_id_rom[170]=8'h0; focus_id_rom[171]=8'h0; focus_id_rom[172]=8'h0; focus_id_rom[173]=8'h0; focus_id_rom[174]=8'h0; focus_id_rom[175]=8'h0;
focus_id_rom[176]=8'h0; focus_id_rom[177]=8'h0; focus_id_rom[178]=8'h0; focus_id_rom[179]=8'h0; focus_id_rom[180]=8'h0; focus_id_rom[181]=8'h0; focus_id_rom[182]=8'h0; focus_id_rom[183]=8'h0;
focus_id_rom[184]=8'h0; focus_id_rom[185]=8'h0; focus_id_rom[186]=8'h0; focus_id_rom[187]=8'h0; focus_id_rom[188]=8'h0; focus_id_rom[189]=8'h0; focus_id_rom[190]=8'h0; focus_id_rom[191]=8'h0;
focus_id_rom[192]=8'h0; focus_id_rom[193]=8'h0; focus_id_rom[194]=8'h0; focus_id_rom[195]=8'h0; focus_id_rom[196]=8'h0; focus_id_rom[197]=8'h0; focus_id_rom[198]=8'h0; focus_id_rom[199]=8'h0;
focus_id_rom[200]=8'h0; focus_id_rom[201]=8'h0; focus_id_rom[202]=8'h0; focus_id_rom[203]=8'h0; focus_id_rom[204]=8'h0; focus_id_rom[205]=8'h0; focus_id_rom[206]=8'h0; focus_id_rom[207]=8'h0;
focus_id_rom[208]=8'h0; focus_id_rom[209]=8'h0; focus_id_rom[210]=8'h0; focus_id_rom[211]=8'h0; focus_id_rom[212]=8'h0; focus_id_rom[213]=8'h0; focus_id_rom[214]=8'h0; focus_id_rom[215]=8'h0;
focus_id_rom[216]=8'h0; focus_id_rom[217]=8'h0; focus_id_rom[218]=8'h0; focus_id_rom[219]=8'h0; focus_id_rom[220]=8'h0; focus_id_rom[221]=8'h0; focus_id_rom[222]=8'h0; focus_id_rom[223]=8'h0;
focus_id_rom[224]=8'h0; focus_id_rom[225]=8'h0; focus_id_rom[226]=8'h0; focus_id_rom[227]=8'h0; focus_id_rom[228]=8'h0; focus_id_rom[229]=8'h0; focus_id_rom[230]=8'h0; focus_id_rom[231]=8'h0;
focus_id_rom[232]=8'h0; focus_id_rom[233]=8'h0; focus_id_rom[234]=8'h0; focus_id_rom[235]=8'h0; focus_id_rom[236]=8'h0; focus_id_rom[237]=8'h0; focus_id_rom[238]=8'h0; focus_id_rom[239]=8'h0;
focus_id_rom[240]=8'h0; focus_id_rom[241]=8'h0; focus_id_rom[242]=8'h0; focus_id_rom[243]=8'h0; focus_id_rom[244]=8'h0; focus_id_rom[245]=8'h0; focus_id_rom[246]=8'h0; focus_id_rom[247]=8'h0;
focus_id_rom[248]=8'h0; focus_id_rom[249]=8'h0; focus_id_rom[250]=8'h0; focus_id_rom[251]=8'h0; focus_id_rom[252]=8'h0; focus_id_rom[253]=8'h0; focus_id_rom[254]=8'h0; focus_id_rom[255]=8'h0;
focus_id_rom[256]=8'h0; focus_id_rom[257]=8'h0; focus_id_rom[258]=8'h0; focus_id_rom[259]=8'h0; focus_id_rom[260]=8'h0; focus_id_rom[261]=8'h0; focus_id_rom[262]=8'h0; focus_id_rom[263]=8'h0;
focus_id_rom[264]=8'h0; focus_id_rom[265]=8'h0; focus_id_rom[266]=8'h0; focus_id_rom[267]=8'h0; focus_id_rom[268]=8'h0; focus_id_rom[269]=8'h0; focus_id_rom[270]=8'h0; focus_id_rom[271]=8'h0;
focus_id_rom[272]=8'h0; focus_id_rom[273]=8'h0; focus_id_rom[274]=8'h0; focus_id_rom[275]=8'h0; focus_id_rom[276]=8'h0; focus_id_rom[277]=8'h0; focus_id_rom[278]=8'h0; focus_id_rom[279]=8'h0;
focus_id_rom[280]=8'h0; focus_id_rom[281]=8'h0; focus_id_rom[282]=8'h0; focus_id_rom[283]=8'h0; focus_id_rom[284]=8'h0; focus_id_rom[285]=8'h0; focus_id_rom[286]=8'h0; focus_id_rom[287]=8'h0;
focus_id_rom[288]=8'h0; focus_id_rom[289]=8'h0; focus_id_rom[290]=8'h0; focus_id_rom[291]=8'h0; focus_id_rom[292]=8'h0; focus_id_rom[293]=8'h0; focus_id_rom[294]=8'h0; focus_id_rom[295]=8'h0;
focus_id_rom[296]=8'h0; focus_id_rom[297]=8'h0; focus_id_rom[298]=8'h0; focus_id_rom[299]=8'h0; focus_id_rom[300]=8'h0; focus_id_rom[301]=8'h0; focus_id_rom[302]=8'h0; focus_id_rom[303]=8'h0;
focus_id_rom[304]=8'h0; focus_id_rom[305]=8'h0; focus_id_rom[306]=8'h0; focus_id_rom[307]=8'h0; focus_id_rom[308]=8'h0; focus_id_rom[309]=8'h0; focus_id_rom[310]=8'h0; focus_id_rom[311]=8'h0;
focus_id_rom[312]=8'h0; focus_id_rom[313]=8'h0; focus_id_rom[314]=8'h0; focus_id_rom[315]=8'h0; focus_id_rom[316]=8'h0; focus_id_rom[317]=8'h0; focus_id_rom[318]=8'h0; focus_id_rom[319]=8'h0;
focus_id_rom[320]=8'h0; focus_id_rom[321]=8'h0; focus_id_rom[322]=8'h0; focus_id_rom[323]=8'h0; focus_id_rom[324]=8'h0; focus_id_rom[325]=8'h0; focus_id_rom[326]=8'h0; focus_id_rom[327]=8'h0;
focus_id_rom[328]=8'h0; focus_id_rom[329]=8'h0; focus_id_rom[330]=8'h0; focus_id_rom[331]=8'h0; focus_id_rom[332]=8'h0; focus_id_rom[333]=8'h0; focus_id_rom[334]=8'h0; focus_id_rom[335]=8'h0;
focus_id_rom[336]=8'h0; focus_id_rom[337]=8'h0; focus_id_rom[338]=8'h0; focus_id_rom[339]=8'h0; focus_id_rom[340]=8'h0; focus_id_rom[341]=8'h0; focus_id_rom[342]=8'h0; focus_id_rom[343]=8'h0;
focus_id_rom[344]=8'h0; focus_id_rom[345]=8'h0; focus_id_rom[346]=8'h0; focus_id_rom[347]=8'h0; focus_id_rom[348]=8'h0; focus_id_rom[349]=8'h0; focus_id_rom[350]=8'h0; focus_id_rom[351]=8'h0;
focus_id_rom[352]=8'h0; focus_id_rom[353]=8'h0; focus_id_rom[354]=8'h0; focus_id_rom[355]=8'h0; focus_id_rom[356]=8'h0; focus_id_rom[357]=8'h0; focus_id_rom[358]=8'h0; focus_id_rom[359]=8'h0;
focus_id_rom[360]=8'h0; focus_id_rom[361]=8'h0; focus_id_rom[362]=8'h0; focus_id_rom[363]=8'h0; focus_id_rom[364]=8'h0; focus_id_rom[365]=8'h0; focus_id_rom[366]=8'h0; focus_id_rom[367]=8'h0;
focus_id_rom[368]=8'h0; focus_id_rom[369]=8'h0; focus_id_rom[370]=8'h0; focus_id_rom[371]=8'h0; focus_id_rom[372]=8'h0; focus_id_rom[373]=8'h0; focus_id_rom[374]=8'h0; focus_id_rom[375]=8'h0;
focus_id_rom[376]=8'h0; focus_id_rom[377]=8'h0; focus_id_rom[378]=8'h0; focus_id_rom[379]=8'h0; focus_id_rom[380]=8'h0; focus_id_rom[381]=8'h0; focus_id_rom[382]=8'h0; focus_id_rom[383]=8'h0;
focus_id_rom[384]=8'h0; focus_id_rom[385]=8'h1a; focus_id_rom[386]=8'hc4; focus_id_rom[387]=8'h4f; focus_id_rom[388]=8'h6a; focus_id_rom[389]=8'hd; focus_id_rom[390]=8'h65; focus_id_rom[391]=8'h18;
focus_id_rom[392]=8'h9b; focus_id_rom[393]=8'h31; focus_id_rom[394]=8'h97; focus_id_rom[395]=8'h0; focus_id_rom[396]=8'h0; focus_id_rom[397]=8'h3a; focus_id_rom[398]=8'hcd; focus_id_rom[399]=8'h66;
focus_id_rom[400]=8'h95; focus_id_rom[401]=8'hde; focus_id_rom[402]=8'h67; focus_id_rom[403]=8'h79; focus_id_rom[404]=8'h14; focus_id_rom[405]=8'hb4; focus_id_rom[406]=8'hde; focus_id_rom[407]=8'h0;
focus_id_rom[408]=8'h0; focus_id_rom[409]=8'h0; focus_id_rom[410]=8'h76; focus_id_rom[411]=8'h9b; focus_id_rom[412]=8'h31; focus_id_rom[413]=8'hbc; focus_id_rom[414]=8'h1b; focus_id_rom[415]=8'hb6;
focus_id_rom[416]=8'h21; focus_id_rom[417]=8'h6; focus_id_rom[418]=8'h0; focus_id_rom[419]=8'h0; focus_id_rom[420]=8'h0; focus_id_rom[421]=8'h0; focus_id_rom[422]=8'h0; focus_id_rom[423]=8'h0;
focus_id_rom[424]=8'h0; focus_id_rom[425]=8'h0; focus_id_rom[426]=8'h0; focus_id_rom[427]=8'h0; focus_id_rom[428]=8'h0; focus_id_rom[429]=8'h0; focus_id_rom[430]=8'h0; focus_id_rom[431]=8'h0;
focus_id_rom[432]=8'h0; focus_id_rom[433]=8'h0; focus_id_rom[434]=8'h0; focus_id_rom[435]=8'h0; focus_id_rom[436]=8'h0; focus_id_rom[437]=8'h0; focus_id_rom[438]=8'h0; focus_id_rom[439]=8'h0;
focus_id_rom[440]=8'h0; focus_id_rom[441]=8'h0; focus_id_rom[442]=8'h0; focus_id_rom[443]=8'h0; focus_id_rom[444]=8'h0; focus_id_rom[445]=8'h0; focus_id_rom[446]=8'h0; focus_id_rom[447]=8'h0;
focus_id_rom[448]=8'h0; focus_id_rom[449]=8'h23; focus_id_rom[450]=8'h9c; focus_id_rom[451]=8'h79; focus_id_rom[452]=8'h14; focus_id_rom[453]=8'h1b; focus_id_rom[454]=8'h9b; focus_id_rom[455]=8'h31;
focus_id_rom[456]=8'h75; focus_id_rom[457]=8'haf; focus_id_rom[458]=8'h5; focus_id_rom[459]=8'h0; focus_id_rom[460]=8'h0; focus_id_rom[461]=8'h0; focus_id_rom[462]=8'h48; focus_id_rom[463]=8'h92;
focus_id_rom[464]=8'hde; focus_id_rom[465]=8'hb4; focus_id_rom[466]=8'h67; focus_id_rom[467]=8'h13; focus_id_rom[468]=8'h79; focus_id_rom[469]=8'h0; focus_id_rom[470]=8'h0; focus_id_rom[471]=8'h0;
focus_id_rom[472]=8'h0; focus_id_rom[473]=8'h0; focus_id_rom[474]=8'h0; focus_id_rom[475]=8'h0; focus_id_rom[476]=8'h0; focus_id_rom[477]=8'h0; focus_id_rom[478]=8'h0; focus_id_rom[479]=8'h0;
focus_id_rom[480]=8'h0; focus_id_rom[481]=8'h0; focus_id_rom[482]=8'h0; focus_id_rom[483]=8'h0; focus_id_rom[484]=8'h0; focus_id_rom[485]=8'h0; focus_id_rom[486]=8'h0; focus_id_rom[487]=8'h0;
focus_id_rom[488]=8'h0; focus_id_rom[489]=8'h0; focus_id_rom[490]=8'h0; focus_id_rom[491]=8'h0; focus_id_rom[492]=8'h0; focus_id_rom[493]=8'h0; focus_id_rom[494]=8'h0; focus_id_rom[495]=8'h0;
focus_id_rom[496]=8'h0; focus_id_rom[497]=8'h0; focus_id_rom[498]=8'h0; focus_id_rom[499]=8'h0; focus_id_rom[500]=8'h0; focus_id_rom[501]=8'h0; focus_id_rom[502]=8'h0; focus_id_rom[503]=8'h0;
focus_id_rom[504]=8'h0; focus_id_rom[505]=8'h0; focus_id_rom[506]=8'h0; focus_id_rom[507]=8'h0; focus_id_rom[508]=8'h0; focus_id_rom[509]=8'h0; focus_id_rom[510]=8'h0; focus_id_rom[511]=8'h0;
focus_id_rom[512]=8'h0; focus_id_rom[513]=8'h0; focus_id_rom[514]=8'h7a; focus_id_rom[515]=8'h43; focus_id_rom[516]=8'h4f; focus_id_rom[517]=8'h6a; focus_id_rom[518]=8'hd; focus_id_rom[519]=8'h31;
focus_id_rom[520]=8'h9b; focus_id_rom[521]=8'h0; focus_id_rom[522]=8'h0; focus_id_rom[523]=8'h0; focus_id_rom[524]=8'h0; focus_id_rom[525]=8'h0; focus_id_rom[526]=8'h0; focus_id_rom[527]=8'h3a;
focus_id_rom[528]=8'hcd; focus_id_rom[529]=8'h69; focus_id_rom[530]=8'hb0; focus_id_rom[531]=8'h6; focus_id_rom[532]=8'h0; focus_id_rom[533]=8'h0; focus_id_rom[534]=8'h0; focus_id_rom[535]=8'h0;
focus_id_rom[536]=8'h0; focus_id_rom[537]=8'h0; focus_id_rom[538]=8'h0; focus_id_rom[539]=8'h0; focus_id_rom[540]=8'h0; focus_id_rom[541]=8'h0; focus_id_rom[542]=8'h0; focus_id_rom[543]=8'h0;
focus_id_rom[544]=8'h0; focus_id_rom[545]=8'h0; focus_id_rom[546]=8'h0; focus_id_rom[547]=8'h0; focus_id_rom[548]=8'h0; focus_id_rom[549]=8'h0; focus_id_rom[550]=8'h0; focus_id_rom[551]=8'h0;
focus_id_rom[552]=8'h0; focus_id_rom[553]=8'h0; focus_id_rom[554]=8'h0; focus_id_rom[555]=8'h0; focus_id_rom[556]=8'h0; focus_id_rom[557]=8'h0; focus_id_rom[558]=8'h0; focus_id_rom[559]=8'h0;
focus_id_rom[560]=8'h0; focus_id_rom[561]=8'h0; focus_id_rom[562]=8'h0; focus_id_rom[563]=8'h0; focus_id_rom[564]=8'h0; focus_id_rom[565]=8'h0; focus_id_rom[566]=8'h0; focus_id_rom[567]=8'h0;
focus_id_rom[568]=8'h0; focus_id_rom[569]=8'h0; focus_id_rom[570]=8'h0; focus_id_rom[571]=8'h0; focus_id_rom[572]=8'h0; focus_id_rom[573]=8'h0; focus_id_rom[574]=8'h0; focus_id_rom[575]=8'h0;
focus_id_rom[576]=8'h0; focus_id_rom[577]=8'h0; focus_id_rom[578]=8'h5b; focus_id_rom[579]=8'hf; focus_id_rom[580]=8'h18; focus_id_rom[581]=8'h9b; focus_id_rom[582]=8'h31; focus_id_rom[583]=8'h11;
focus_id_rom[584]=8'hb0; focus_id_rom[585]=8'h0; focus_id_rom[586]=8'h0; focus_id_rom[587]=8'h0; focus_id_rom[588]=8'h0; focus_id_rom[589]=8'h0; focus_id_rom[590]=8'hb; focus_id_rom[591]=8'h37;
focus_id_rom[592]=8'h10; focus_id_rom[593]=8'h78; focus_id_rom[594]=8'hc5; focus_id_rom[595]=8'h67; focus_id_rom[596]=8'h6b; focus_id_rom[597]=8'hde; focus_id_rom[598]=8'h0; focus_id_rom[599]=8'h0;
focus_id_rom[600]=8'h0; focus_id_rom[601]=8'h61; focus_id_rom[602]=8'h36; focus_id_rom[603]=8'h9c; focus_id_rom[604]=8'h74; focus_id_rom[605]=8'h9b; focus_id_rom[606]=8'h31; focus_id_rom[607]=8'h97;
focus_id_rom[608]=8'h45; focus_id_rom[609]=8'h1f; focus_id_rom[610]=8'h6; focus_id_rom[611]=8'h0; focus_id_rom[612]=8'h0; focus_id_rom[613]=8'h0; focus_id_rom[614]=8'h0; focus_id_rom[615]=8'h0;
focus_id_rom[616]=8'h0; focus_id_rom[617]=8'h0; focus_id_rom[618]=8'h0; focus_id_rom[619]=8'h0; focus_id_rom[620]=8'h0; focus_id_rom[621]=8'h0; focus_id_rom[622]=8'h0; focus_id_rom[623]=8'h0;
focus_id_rom[624]=8'h0; focus_id_rom[625]=8'h0; focus_id_rom[626]=8'h0; focus_id_rom[627]=8'h0; focus_id_rom[628]=8'h0; focus_id_rom[629]=8'h0; focus_id_rom[630]=8'h0; focus_id_rom[631]=8'h0;
focus_id_rom[632]=8'h0; focus_id_rom[633]=8'h0; focus_id_rom[634]=8'h0; focus_id_rom[635]=8'h0; focus_id_rom[636]=8'h0; focus_id_rom[637]=8'h0; focus_id_rom[638]=8'h0; focus_id_rom[639]=8'h0;
focus_id_rom[640]=8'h0; focus_id_rom[641]=8'h0; focus_id_rom[642]=8'h0; focus_id_rom[643]=8'h1c; focus_id_rom[644]=8'h79; focus_id_rom[645]=8'hb7; focus_id_rom[646]=8'h67; focus_id_rom[647]=8'hde;
focus_id_rom[648]=8'h0; focus_id_rom[649]=8'h0; focus_id_rom[650]=8'h0; focus_id_rom[651]=8'h0; focus_id_rom[652]=8'h0; focus_id_rom[653]=8'h0; focus_id_rom[654]=8'h0; focus_id_rom[655]=8'h2a;
focus_id_rom[656]=8'h2e; focus_id_rom[657]=8'h91; focus_id_rom[658]=8'h8a; focus_id_rom[659]=8'h53; focus_id_rom[660]=8'h58; focus_id_rom[661]=8'h0; focus_id_rom[662]=8'h0; focus_id_rom[663]=8'h0;
focus_id_rom[664]=8'h0; focus_id_rom[665]=8'h0; focus_id_rom[666]=8'h0; focus_id_rom[667]=8'h0; focus_id_rom[668]=8'h0; focus_id_rom[669]=8'h0; focus_id_rom[670]=8'h0; focus_id_rom[671]=8'h0;
focus_id_rom[672]=8'h0; focus_id_rom[673]=8'h0; focus_id_rom[674]=8'h0; focus_id_rom[675]=8'h0; focus_id_rom[676]=8'h0; focus_id_rom[677]=8'h0; focus_id_rom[678]=8'h0; focus_id_rom[679]=8'h0;
focus_id_rom[680]=8'h0; focus_id_rom[681]=8'h0; focus_id_rom[682]=8'h0; focus_id_rom[683]=8'h0; focus_id_rom[684]=8'h0; focus_id_rom[685]=8'h0; focus_id_rom[686]=8'h0; focus_id_rom[687]=8'h0;
focus_id_rom[688]=8'h0; focus_id_rom[689]=8'h0; focus_id_rom[690]=8'h0; focus_id_rom[691]=8'h0; focus_id_rom[692]=8'h0; focus_id_rom[693]=8'h0; focus_id_rom[694]=8'h0; focus_id_rom[695]=8'h0;
focus_id_rom[696]=8'h0; focus_id_rom[697]=8'h0; focus_id_rom[698]=8'h0; focus_id_rom[699]=8'h0; focus_id_rom[700]=8'h0; focus_id_rom[701]=8'h0; focus_id_rom[702]=8'h0; focus_id_rom[703]=8'h0;
focus_id_rom[704]=8'h0; focus_id_rom[705]=8'h27; focus_id_rom[706]=8'hbc; focus_id_rom[707]=8'ha2; focus_id_rom[708]=8'h6e; focus_id_rom[709]=8'h2; focus_id_rom[710]=8'h1f; focus_id_rom[711]=8'h9b;
focus_id_rom[712]=8'h31; focus_id_rom[713]=8'h3; focus_id_rom[714]=8'h0; focus_id_rom[715]=8'h0; focus_id_rom[716]=8'h0; focus_id_rom[717]=8'h76; focus_id_rom[718]=8'h51; focus_id_rom[719]=8'ha3;
focus_id_rom[720]=8'h8d; focus_id_rom[721]=8'h5; focus_id_rom[722]=8'ha8; focus_id_rom[723]=8'h72; focus_id_rom[724]=8'hbc; focus_id_rom[725]=8'ha5; focus_id_rom[726]=8'h29; focus_id_rom[727]=8'h0;
focus_id_rom[728]=8'h0; focus_id_rom[729]=8'h0; focus_id_rom[730]=8'h0; focus_id_rom[731]=8'h0; focus_id_rom[732]=8'h0; focus_id_rom[733]=8'h0; focus_id_rom[734]=8'h0; focus_id_rom[735]=8'h0;
focus_id_rom[736]=8'h0; focus_id_rom[737]=8'h0; focus_id_rom[738]=8'h0; focus_id_rom[739]=8'h0; focus_id_rom[740]=8'h0; focus_id_rom[741]=8'h0; focus_id_rom[742]=8'h0; focus_id_rom[743]=8'h0;
focus_id_rom[744]=8'h0; focus_id_rom[745]=8'h0; focus_id_rom[746]=8'h0; focus_id_rom[747]=8'h0; focus_id_rom[748]=8'h0; focus_id_rom[749]=8'h0; focus_id_rom[750]=8'h0; focus_id_rom[751]=8'h0;
focus_id_rom[752]=8'h0; focus_id_rom[753]=8'h0; focus_id_rom[754]=8'h0; focus_id_rom[755]=8'h0; focus_id_rom[756]=8'h0; focus_id_rom[757]=8'h0; focus_id_rom[758]=8'h0; focus_id_rom[759]=8'h0;
focus_id_rom[760]=8'h0; focus_id_rom[761]=8'h0; focus_id_rom[762]=8'h0; focus_id_rom[763]=8'h0; focus_id_rom[764]=8'h0; focus_id_rom[765]=8'h0; focus_id_rom[766]=8'h0; focus_id_rom[767]=8'h0;
focus_id_rom[768]=8'h0; focus_id_rom[769]=8'h0; focus_id_rom[770]=8'h0; focus_id_rom[771]=8'h0; focus_id_rom[772]=8'h0; focus_id_rom[773]=8'h0; focus_id_rom[774]=8'h0; focus_id_rom[775]=8'h0;
focus_id_rom[776]=8'h0; focus_id_rom[777]=8'h0; focus_id_rom[778]=8'h0; focus_id_rom[779]=8'h0; focus_id_rom[780]=8'h0; focus_id_rom[781]=8'h0; focus_id_rom[782]=8'h0; focus_id_rom[783]=8'h0;
focus_id_rom[784]=8'h0; focus_id_rom[785]=8'h0; focus_id_rom[786]=8'h0; focus_id_rom[787]=8'h0; focus_id_rom[788]=8'h0; focus_id_rom[789]=8'h0; focus_id_rom[790]=8'h0; focus_id_rom[791]=8'h0;
focus_id_rom[792]=8'h0; focus_id_rom[793]=8'h0; focus_id_rom[794]=8'h0; focus_id_rom[795]=8'h0; focus_id_rom[796]=8'h0; focus_id_rom[797]=8'h0; focus_id_rom[798]=8'h0; focus_id_rom[799]=8'h0;
focus_id_rom[800]=8'h0; focus_id_rom[801]=8'h0; focus_id_rom[802]=8'h0; focus_id_rom[803]=8'h0; focus_id_rom[804]=8'h0; focus_id_rom[805]=8'h0; focus_id_rom[806]=8'h0; focus_id_rom[807]=8'h0;
focus_id_rom[808]=8'h0; focus_id_rom[809]=8'h0; focus_id_rom[810]=8'h0; focus_id_rom[811]=8'h0; focus_id_rom[812]=8'h0; focus_id_rom[813]=8'h0; focus_id_rom[814]=8'h0; focus_id_rom[815]=8'h0;
focus_id_rom[816]=8'h0; focus_id_rom[817]=8'h0; focus_id_rom[818]=8'h0; focus_id_rom[819]=8'h0; focus_id_rom[820]=8'h0; focus_id_rom[821]=8'h0; focus_id_rom[822]=8'h0; focus_id_rom[823]=8'h0;
focus_id_rom[824]=8'h0; focus_id_rom[825]=8'h0; focus_id_rom[826]=8'h0; focus_id_rom[827]=8'h0; focus_id_rom[828]=8'h0; focus_id_rom[829]=8'h0; focus_id_rom[830]=8'h0; focus_id_rom[831]=8'h0;
focus_id_rom[832]=8'h0; focus_id_rom[833]=8'h0; focus_id_rom[834]=8'h0; focus_id_rom[835]=8'h0; focus_id_rom[836]=8'h0; focus_id_rom[837]=8'h0; focus_id_rom[838]=8'h0; focus_id_rom[839]=8'h0;
focus_id_rom[840]=8'h0; focus_id_rom[841]=8'h0; focus_id_rom[842]=8'h0; focus_id_rom[843]=8'h0; focus_id_rom[844]=8'h0; focus_id_rom[845]=8'h0; focus_id_rom[846]=8'h0; focus_id_rom[847]=8'h0;
focus_id_rom[848]=8'h0; focus_id_rom[849]=8'h0; focus_id_rom[850]=8'h0; focus_id_rom[851]=8'h0; focus_id_rom[852]=8'h0; focus_id_rom[853]=8'h0; focus_id_rom[854]=8'h0; focus_id_rom[855]=8'h0;
focus_id_rom[856]=8'h0; focus_id_rom[857]=8'h0; focus_id_rom[858]=8'h0; focus_id_rom[859]=8'h0; focus_id_rom[860]=8'h0; focus_id_rom[861]=8'h0; focus_id_rom[862]=8'h0; focus_id_rom[863]=8'h0;
focus_id_rom[864]=8'h0; focus_id_rom[865]=8'h0; focus_id_rom[866]=8'h0; focus_id_rom[867]=8'h0; focus_id_rom[868]=8'h0; focus_id_rom[869]=8'h0; focus_id_rom[870]=8'h0; focus_id_rom[871]=8'h0;
focus_id_rom[872]=8'h0; focus_id_rom[873]=8'h0; focus_id_rom[874]=8'h0; focus_id_rom[875]=8'h0; focus_id_rom[876]=8'h0; focus_id_rom[877]=8'h0; focus_id_rom[878]=8'h0; focus_id_rom[879]=8'h0;
focus_id_rom[880]=8'h0; focus_id_rom[881]=8'h0; focus_id_rom[882]=8'h0; focus_id_rom[883]=8'h0; focus_id_rom[884]=8'h0; focus_id_rom[885]=8'h0; focus_id_rom[886]=8'h0; focus_id_rom[887]=8'h0;
focus_id_rom[888]=8'h0; focus_id_rom[889]=8'h0; focus_id_rom[890]=8'h0; focus_id_rom[891]=8'h0; focus_id_rom[892]=8'h0; focus_id_rom[893]=8'h0; focus_id_rom[894]=8'h0; focus_id_rom[895]=8'h0;
focus_id_rom[896]=8'h0; focus_id_rom[897]=8'h0; focus_id_rom[898]=8'h0; focus_id_rom[899]=8'h0; focus_id_rom[900]=8'h0; focus_id_rom[901]=8'h0; focus_id_rom[902]=8'h0; focus_id_rom[903]=8'h0;
focus_id_rom[904]=8'h0; focus_id_rom[905]=8'h0; focus_id_rom[906]=8'h0; focus_id_rom[907]=8'h0; focus_id_rom[908]=8'h0; focus_id_rom[909]=8'h0; focus_id_rom[910]=8'h0; focus_id_rom[911]=8'h0;
focus_id_rom[912]=8'h0; focus_id_rom[913]=8'h0; focus_id_rom[914]=8'h0; focus_id_rom[915]=8'h0; focus_id_rom[916]=8'h0; focus_id_rom[917]=8'h0; focus_id_rom[918]=8'h0; focus_id_rom[919]=8'h0;
focus_id_rom[920]=8'h0; focus_id_rom[921]=8'h0; focus_id_rom[922]=8'h0; focus_id_rom[923]=8'h0; focus_id_rom[924]=8'h0; focus_id_rom[925]=8'h0; focus_id_rom[926]=8'h0; focus_id_rom[927]=8'h0;
focus_id_rom[928]=8'h0; focus_id_rom[929]=8'h0; focus_id_rom[930]=8'h0; focus_id_rom[931]=8'h0; focus_id_rom[932]=8'h0; focus_id_rom[933]=8'h0; focus_id_rom[934]=8'h0; focus_id_rom[935]=8'h0;
focus_id_rom[936]=8'h0; focus_id_rom[937]=8'h0; focus_id_rom[938]=8'h0; focus_id_rom[939]=8'h0; focus_id_rom[940]=8'h0; focus_id_rom[941]=8'h0; focus_id_rom[942]=8'h0; focus_id_rom[943]=8'h0;
focus_id_rom[944]=8'h0; focus_id_rom[945]=8'h0; focus_id_rom[946]=8'h0; focus_id_rom[947]=8'h0; focus_id_rom[948]=8'h0; focus_id_rom[949]=8'h0; focus_id_rom[950]=8'h0; focus_id_rom[951]=8'h0;
focus_id_rom[952]=8'h0; focus_id_rom[953]=8'h0; focus_id_rom[954]=8'h0; focus_id_rom[955]=8'h0; focus_id_rom[956]=8'h0; focus_id_rom[957]=8'h0; focus_id_rom[958]=8'h0; focus_id_rom[959]=8'h0;
focus_id_rom[960]=8'h0; focus_id_rom[961]=8'h0; focus_id_rom[962]=8'h0; focus_id_rom[963]=8'h0; focus_id_rom[964]=8'h0; focus_id_rom[965]=8'h0; focus_id_rom[966]=8'h0; focus_id_rom[967]=8'h0;
focus_id_rom[968]=8'h0; focus_id_rom[969]=8'h0; focus_id_rom[970]=8'h0; focus_id_rom[971]=8'h0; focus_id_rom[972]=8'h0; focus_id_rom[973]=8'h0; focus_id_rom[974]=8'h0; focus_id_rom[975]=8'h0;
focus_id_rom[976]=8'h0; focus_id_rom[977]=8'h0; focus_id_rom[978]=8'h0; focus_id_rom[979]=8'h0; focus_id_rom[980]=8'h0; focus_id_rom[981]=8'h0; focus_id_rom[982]=8'h0; focus_id_rom[983]=8'h0;
focus_id_rom[984]=8'h0; focus_id_rom[985]=8'h0; focus_id_rom[986]=8'h0; focus_id_rom[987]=8'h0; focus_id_rom[988]=8'h0; focus_id_rom[989]=8'h0; focus_id_rom[990]=8'h0; focus_id_rom[991]=8'h0;
focus_id_rom[992]=8'h0; focus_id_rom[993]=8'h0; focus_id_rom[994]=8'h0; focus_id_rom[995]=8'h0; focus_id_rom[996]=8'h0; focus_id_rom[997]=8'h0; focus_id_rom[998]=8'h0; focus_id_rom[999]=8'h0;
focus_id_rom[1000]=8'h0; focus_id_rom[1001]=8'h0; focus_id_rom[1002]=8'h0; focus_id_rom[1003]=8'h0; focus_id_rom[1004]=8'h0; focus_id_rom[1005]=8'h0; focus_id_rom[1006]=8'h0; focus_id_rom[1007]=8'h0;
focus_id_rom[1008]=8'h0; focus_id_rom[1009]=8'h0; focus_id_rom[1010]=8'h0; focus_id_rom[1011]=8'h0; focus_id_rom[1012]=8'h0; focus_id_rom[1013]=8'h0; focus_id_rom[1014]=8'h0; focus_id_rom[1015]=8'h0;
focus_id_rom[1016]=8'h0; focus_id_rom[1017]=8'h0; focus_id_rom[1018]=8'h0; focus_id_rom[1019]=8'h0; focus_id_rom[1020]=8'h0; focus_id_rom[1021]=8'h0; focus_id_rom[1022]=8'h0; focus_id_rom[1023]=8'h0;
end
reg [23:0] font_span_rom [0:255]; /* fehdl force_ram=1, ram_style="bram" */
initial begin
font_span_rom[0]=24'h0; font_span_rom[1]=24'h6d; font_span_rom[2]=24'h18e3; font_span_rom[3]=24'h50e3; font_span_rom[4]=24'h8875; font_span_rom[5]=24'ha0d5; font_span_rom[6]=24'hd0f4; font_span_rom[7]=24'h10b61;
font_span_rom[8]=24'h1e361; font_span_rom[9]=24'h2b84c; font_span_rom[10]=24'h2cac3; font_span_rom[11]=24'h37b02; font_span_rom[12]=24'h43ae3; font_span_rom[13]=24'h4f322; font_span_rom[14]=24'h5bb02; font_span_rom[15]=24'h67b41;
font_span_rom[16]=24'h74b42; font_span_rom[17]=24'h81b22; font_span_rom[18]=24'h8e284; font_span_rom[19]=24'h98303; font_span_rom[20]=24'ha4322; font_span_rom[21]=24'hb0b41; font_span_rom[22]=24'hbdb22; font_span_rom[23]=24'hca341;
font_span_rom[24]=24'hd7322; font_span_rom[25]=24'he3b41; font_span_rom[26]=24'hf0b61; font_span_rom[27]=24'hfe322; font_span_rom[28]=24'h10ab22; font_span_rom[29]=24'h117342; font_span_rom[30]=24'h124341; font_span_rom[31]=24'h131342;
font_span_rom[32]=24'h13e341; font_span_rom[33]=24'h14b342; font_span_rom[34]=24'h158342; font_span_rom[35]=24'h165322; font_span_rom[36]=24'h171b61; font_span_rom[37]=24'h17f341; font_span_rom[38]=24'h18c321; font_span_rom[39]=24'h198b21;
font_span_rom[40]=24'h1a5302; font_span_rom[41]=24'h1b1342; font_span_rom[42]=24'h1be303; font_span_rom[43]=24'h1ca341; font_span_rom[44]=24'h1d7361; font_span_rom[45]=24'h1e4b61; font_span_rom[46]=24'h1f2341; font_span_rom[47]=24'h1ff322;
font_span_rom[48]=24'h20bb41; font_span_rom[49]=24'h218b22; font_span_rom[50]=24'h225341; font_span_rom[51]=24'h232361; font_span_rom[52]=24'h23fb21; font_span_rom[53]=24'h24c322; font_span_rom[54]=24'h258b22; font_span_rom[55]=24'h265342;
font_span_rom[56]=24'h272323; font_span_rom[57]=24'h27eb42; font_span_rom[58]=24'h28bb23; font_span_rom[59]=24'h298342; font_span_rom[60]=24'h2a5342; font_span_rom[61]=24'h2b2342; font_span_rom[62]=24'h2bf322; font_span_rom[63]=24'h2cbb22;
font_span_rom[64]=24'h2d8303; font_span_rom[65]=24'h2e4322; font_span_rom[66]=24'h2f0b61; font_span_rom[67]=24'h2fe341; font_span_rom[68]=24'h30b322; font_span_rom[69]=24'h317b61; font_span_rom[70]=24'h325341; font_span_rom[71]=24'h332321;
font_span_rom[72]=24'h33eae3; font_span_rom[73]=24'h34a322; font_span_rom[74]=24'h356b21; font_span_rom[75]=24'h363342; font_span_rom[76]=24'h370361; font_span_rom[77]=24'h37db23; font_span_rom[78]=24'h38a303; font_span_rom[79]=24'h396322;
font_span_rom[80]=24'h3a2b22; font_span_rom[81]=24'h3af361; font_span_rom[82]=24'h3bcb41; font_span_rom[83]=24'h3c9b22; font_span_rom[84]=24'h3d6322; font_span_rom[85]=24'h3e2b41; font_span_rom[86]=24'h3efb61; font_span_rom[87]=24'h3fd341;
font_span_rom[88]=24'h40a361; font_span_rom[89]=24'h417b61; font_span_rom[90]=24'h425302; font_span_rom[91]=24'h431361; font_span_rom[92]=24'h43eb22; font_span_rom[93]=24'h44b361; font_span_rom[94]=24'h458b22; font_span_rom[95]=24'h465342;
font_span_rom[96]=24'h472323; font_span_rom[97]=24'h47eb23; font_span_rom[98]=24'h48b361; font_span_rom[99]=24'h498b22; font_span_rom[100]=24'h4a5322; font_span_rom[101]=24'h4b1b61; font_span_rom[102]=24'h4bf341; font_span_rom[103]=24'h4cc322;
font_span_rom[104]=24'h4d8b41; font_span_rom[105]=24'h4e5b41; font_span_rom[106]=24'h4f2b21; font_span_rom[107]=24'h4ff302; font_span_rom[108]=24'h50b321; font_span_rom[109]=24'h517b02; font_span_rom[110]=24'h523b02; font_span_rom[111]=24'h52fb22;
font_span_rom[112]=24'h53c341; font_span_rom[113]=24'h549340; font_span_rom[114]=24'h556341; font_span_rom[115]=24'h563323; font_span_rom[116]=24'h56fb03; font_span_rom[117]=24'h57bb21; font_span_rom[118]=24'h588303; font_span_rom[119]=24'h594361;
font_span_rom[120]=24'h5a1b41; font_span_rom[121]=24'h5aeb03; font_span_rom[122]=24'h5bab60; font_span_rom[123]=24'h5c8341; font_span_rom[124]=24'h5d5303; font_span_rom[125]=24'h5e1323; font_span_rom[126]=24'h5edb41; font_span_rom[127]=24'h5fab42;
font_span_rom[128]=24'h607b61; font_span_rom[129]=24'h615361; font_span_rom[130]=24'h622b61; font_span_rom[131]=24'h630361; font_span_rom[132]=24'h63db41; font_span_rom[133]=24'h64ab40; font_span_rom[134]=24'h657b41; font_span_rom[135]=24'h664b41;
font_span_rom[136]=24'h671b61; font_span_rom[137]=24'h67f361; font_span_rom[138]=24'h68cb21; font_span_rom[139]=24'h699380; font_span_rom[140]=24'h6a7342; font_span_rom[141]=24'h6b4361; font_span_rom[142]=24'h6c1b22; font_span_rom[143]=24'h6ce360;
font_span_rom[144]=24'h6dbb41; font_span_rom[145]=24'h6e8b41; font_span_rom[146]=24'h6f5b41; font_span_rom[147]=24'h702b03; font_span_rom[148]=24'h70eb03; font_span_rom[149]=24'h71ab61; font_span_rom[150]=24'h728341; font_span_rom[151]=24'h735341;
font_span_rom[152]=24'h742361; font_span_rom[153]=24'h74fb22; font_span_rom[154]=24'h75c342; font_span_rom[155]=24'h769360; font_span_rom[156]=24'h776b22; font_span_rom[157]=24'h783360; font_span_rom[158]=24'h790b60; font_span_rom[159]=24'h79e341;
font_span_rom[160]=24'h7ab360; font_span_rom[161]=24'h7b8b41; font_span_rom[162]=24'h7c5b41; font_span_rom[163]=24'h7d2b21; font_span_rom[164]=24'h7df341; font_span_rom[165]=24'h7ec302; font_span_rom[166]=24'h7f8360; font_span_rom[167]=24'h805b40;
font_span_rom[168]=24'h812b23; font_span_rom[169]=24'h81f360; font_span_rom[170]=24'h82cb61; font_span_rom[171]=24'h83a380; font_span_rom[172]=24'h848361; font_span_rom[173]=24'h855b61; font_span_rom[174]=24'h863380; font_span_rom[175]=24'h871380;
font_span_rom[176]=24'h87f361; font_span_rom[177]=24'h88cb41; font_span_rom[178]=24'h899b41; font_span_rom[179]=24'h8a6b61; font_span_rom[180]=24'h8b4361; font_span_rom[181]=24'h8c1b41; font_span_rom[182]=24'h8ceb61; font_span_rom[183]=24'h8dc361;
font_span_rom[184]=24'h8e9b42; font_span_rom[185]=24'h8f6b61; font_span_rom[186]=24'h904342; font_span_rom[187]=24'h911361; font_span_rom[188]=24'h91eb41; font_span_rom[189]=24'h92bb22; font_span_rom[190]=24'h938361; font_span_rom[191]=24'h945b61;
font_span_rom[192]=24'h953361; font_span_rom[193]=24'h960b23; font_span_rom[194]=24'h96d322; font_span_rom[195]=24'h979b41; font_span_rom[196]=24'h986b41; font_span_rom[197]=24'h993b41; font_span_rom[198]=24'h9a0b23; font_span_rom[199]=24'h9ad361;
font_span_rom[200]=24'h9bab61; font_span_rom[201]=24'h9c8322; font_span_rom[202]=24'h9d4b42; font_span_rom[203]=24'h9e1b61; font_span_rom[204]=24'h9ef2e3; font_span_rom[205]=24'h9fab02; font_span_rom[206]=24'ha06b61; font_span_rom[207]=24'ha14361;
font_span_rom[208]=24'ha21b61; font_span_rom[209]=24'ha2f361; font_span_rom[210]=24'ha3cb61; font_span_rom[211]=24'ha4a361; font_span_rom[212]=24'ha57b03; font_span_rom[213]=24'ha63b23; font_span_rom[214]=24'ha70323; font_span_rom[215]=24'ha7cb03;
font_span_rom[216]=24'ha88b23; font_span_rom[217]=24'ha95341; font_span_rom[218]=24'haa2361; font_span_rom[219]=24'haafb61; font_span_rom[220]=24'habd303; font_span_rom[221]=24'hac9360; font_span_rom[222]=24'had68f5; font_span_rom[223]=24'hada1e9;
font_span_rom[224]=24'hae1a69; font_span_rom[225]=24'haeb2a3; font_span_rom[226]=24'h0; font_span_rom[227]=24'h0; font_span_rom[228]=24'h0; font_span_rom[229]=24'h0; font_span_rom[230]=24'h0; font_span_rom[231]=24'h0;
font_span_rom[232]=24'h0; font_span_rom[233]=24'h0; font_span_rom[234]=24'h0; font_span_rom[235]=24'h0; font_span_rom[236]=24'h0; font_span_rom[237]=24'h0; font_span_rom[238]=24'h0; font_span_rom[239]=24'h0;
font_span_rom[240]=24'h0; font_span_rom[241]=24'h0; font_span_rom[242]=24'h0; font_span_rom[243]=24'h0; font_span_rom[244]=24'h0; font_span_rom[245]=24'h0; font_span_rom[246]=24'h0; font_span_rom[247]=24'h0;
font_span_rom[248]=24'h0; font_span_rom[249]=24'h0; font_span_rom[250]=24'h0; font_span_rom[251]=24'h0; font_span_rom[252]=24'h0; font_span_rom[253]=24'h0; font_span_rom[254]=24'h0; font_span_rom[255]=24'h0;
end
reg [31:0] font_rom [0:5610]; /* fehdl force_ram=1, ram_style="bram" */
initial begin
font_rom[0]=32'h38000; font_rom[1]=32'h38000; font_rom[2]=32'h38000; font_rom[3]=32'h330; font_rom[4]=32'h630; font_rom[5]=32'h630; font_rom[6]=32'h670; font_rom[7]=32'he60;
font_rom[8]=32'hc60; font_rom[9]=32'hc60; font_rom[10]=32'h6300000; font_rom[11]=32'h6300000; font_rom[12]=32'h6700000; font_rom[13]=32'hc600000; font_rom[14]=32'hc600000; font_rom[15]=32'hc600000;
font_rom[16]=32'h8400000; font_rom[17]=32'h3838700; font_rom[18]=32'h3838780; font_rom[19]=32'h1838300; font_rom[20]=32'h4000000; font_rom[21]=32'he000000; font_rom[22]=32'h7000000; font_rom[23]=32'h3800000;
font_rom[24]=32'h1c00000; font_rom[25]=32'h800000; font_rom[26]=32'h3800000; font_rom[27]=32'hfc00000; font_rom[28]=32'hcc00000; font_rom[29]=32'hc600000; font_rom[30]=32'hcc00000; font_rom[31]=32'hfc00000;
font_rom[32]=32'h3800000; font_rom[33]=32'h730; font_rom[34]=32'h660; font_rom[35]=32'h660; font_rom[36]=32'hcc0; font_rom[37]=32'hcc0; font_rom[38]=32'h19c0; font_rom[39]=32'h1980;
font_rom[40]=32'h3b80; font_rom[41]=32'h3300; font_rom[42]=32'h7300; font_rom[43]=32'h6600; font_rom[44]=32'h6600; font_rom[45]=32'hcc00; font_rom[46]=32'hcc00; font_rom[47]=32'hcc00;
font_rom[48]=32'h6600; font_rom[49]=32'h6600; font_rom[50]=32'h7300; font_rom[51]=32'h3300; font_rom[52]=32'h3b80; font_rom[53]=32'h1980; font_rom[54]=32'h19c0; font_rom[55]=32'hcc0;
font_rom[56]=32'hcc0; font_rom[57]=32'h660; font_rom[58]=32'h660; font_rom[59]=32'h730; font_rom[60]=32'hee00000; font_rom[61]=32'h6600000; font_rom[62]=32'h6700000; font_rom[63]=32'h3300000;
font_rom[64]=32'h3300000; font_rom[65]=32'h1980000; font_rom[66]=32'h1980000; font_rom[67]=32'h1cc0000; font_rom[68]=32'hcc0000; font_rom[69]=32'hee0000; font_rom[70]=32'h660000; font_rom[71]=32'h670000;
font_rom[72]=32'h330000; font_rom[73]=32'h330000; font_rom[74]=32'h330000; font_rom[75]=32'h670000; font_rom[76]=32'h660000; font_rom[77]=32'hee0000; font_rom[78]=32'hcc0000; font_rom[79]=32'h1cc0000;
font_rom[80]=32'h1980000; font_rom[81]=32'h1980000; font_rom[82]=32'h3300000; font_rom[83]=32'h3300000; font_rom[84]=32'h6700000; font_rom[85]=32'h6600000; font_rom[86]=32'hee00000; font_rom[87]=32'h3ffffffc;
font_rom[88]=32'h3ffffffc; font_rom[89]=32'h7ffffe0; font_rom[90]=32'h7ffffe0; font_rom[91]=32'h0; font_rom[92]=32'h0; font_rom[93]=32'h0; font_rom[94]=32'h0; font_rom[95]=32'h0;
font_rom[96]=32'h0; font_rom[97]=32'h0; font_rom[98]=32'h3ffffc0; font_rom[99]=32'h3ffffc0; font_rom[100]=32'h0; font_rom[101]=32'h0; font_rom[102]=32'h0; font_rom[103]=32'h0;
font_rom[104]=32'h0; font_rom[105]=32'h0; font_rom[106]=32'h0; font_rom[107]=32'h0; font_rom[108]=32'h0; font_rom[109]=32'h1ffffff8; font_rom[110]=32'h1ffffff8; font_rom[111]=32'h70000;
font_rom[112]=32'h70000; font_rom[113]=32'h70000; font_rom[114]=32'h70000; font_rom[115]=32'h70000; font_rom[116]=32'h70000; font_rom[117]=32'h70000; font_rom[118]=32'h7ffe0; font_rom[119]=32'h7ffe0;
font_rom[120]=32'h70000; font_rom[121]=32'h70000; font_rom[122]=32'h70000; font_rom[123]=32'h70000; font_rom[124]=32'h70000; font_rom[125]=32'h70000; font_rom[126]=32'h70000; font_rom[127]=32'h70000;
font_rom[128]=32'h70000; font_rom[129]=32'h70000; font_rom[130]=32'h70000; font_rom[131]=32'h70000; font_rom[132]=32'h70000; font_rom[133]=32'h1ffffff8; font_rom[134]=32'h1ffffff8; font_rom[135]=32'h1ffffff8;
font_rom[136]=32'h1ffffff8; font_rom[137]=32'h70000; font_rom[138]=32'h30000; font_rom[139]=32'h30000; font_rom[140]=32'h30000; font_rom[141]=32'h34000; font_rom[142]=32'h3f000; font_rom[143]=32'h37800;
font_rom[144]=32'h31e00; font_rom[145]=32'h30f00; font_rom[146]=32'h303c0; font_rom[147]=32'h301e0; font_rom[148]=32'h300e0; font_rom[149]=32'h30060; font_rom[150]=32'h30000; font_rom[151]=32'h30000;
font_rom[152]=32'h30000; font_rom[153]=32'h30000; font_rom[154]=32'h30000; font_rom[155]=32'h30000; font_rom[156]=32'h30000; font_rom[157]=32'h30000; font_rom[158]=32'hc00000; font_rom[159]=32'hc00000;
font_rom[160]=32'hc00000; font_rom[161]=32'hfffff8; font_rom[162]=32'hfffff8; font_rom[163]=32'hc00000; font_rom[164]=32'h1c00000; font_rom[165]=32'h1c00000; font_rom[166]=32'h1c00000; font_rom[167]=32'h1800000;
font_rom[168]=32'h1ffffe0; font_rom[169]=32'h1ffffe0; font_rom[170]=32'h60; font_rom[171]=32'h60; font_rom[172]=32'h60; font_rom[173]=32'h60; font_rom[174]=32'h3ffffe60; font_rom[175]=32'h3ffffe60;
font_rom[176]=32'h60; font_rom[177]=32'h60; font_rom[178]=32'he0; font_rom[179]=32'hc0; font_rom[180]=32'h1c0; font_rom[181]=32'h3ffc0; font_rom[182]=32'h1ff80; font_rom[183]=32'h30180;
font_rom[184]=32'h3830180; font_rom[185]=32'h3830180; font_rom[186]=32'h3830180; font_rom[187]=32'h3830180; font_rom[188]=32'h3830180; font_rom[189]=32'h1ffffff8; font_rom[190]=32'h1ffffff8; font_rom[191]=32'h3830180;
font_rom[192]=32'h3830180; font_rom[193]=32'h3830180; font_rom[194]=32'h3830180; font_rom[195]=32'h3830180; font_rom[196]=32'h3830180; font_rom[197]=32'h3830180; font_rom[198]=32'h383ff80; font_rom[199]=32'h383ff80;
font_rom[200]=32'h3830180; font_rom[201]=32'h3830180; font_rom[202]=32'h3800000; font_rom[203]=32'h3800000; font_rom[204]=32'h3800000; font_rom[205]=32'h3fffff8; font_rom[206]=32'h3fffff8; font_rom[207]=32'h1c000;
font_rom[208]=32'h3c000; font_rom[209]=32'h7e000; font_rom[210]=32'h66000; font_rom[211]=32'he7000; font_rom[212]=32'h1c3800; font_rom[213]=32'h381c00; font_rom[214]=32'h700e00; font_rom[215]=32'he00700;
font_rom[216]=32'h3c00380; font_rom[217]=32'h78181e0; font_rom[218]=32'h1f0180f8; font_rom[219]=32'h1c018038; font_rom[220]=32'h18018018; font_rom[221]=32'h18000; font_rom[222]=32'h18000; font_rom[223]=32'h18000;
font_rom[224]=32'h18000; font_rom[225]=32'h18000; font_rom[226]=32'h18000; font_rom[227]=32'h18000; font_rom[228]=32'h18000; font_rom[229]=32'h18000; font_rom[230]=32'h18000; font_rom[231]=32'h18000;
font_rom[232]=32'h18000; font_rom[233]=32'h38000; font_rom[234]=32'h3038000; font_rom[235]=32'h3c38000; font_rom[236]=32'h1e30000; font_rom[237]=32'h430000; font_rom[238]=32'h30000; font_rom[239]=32'hffffff0;
font_rom[240]=32'hffffff0; font_rom[241]=32'h70070; font_rom[242]=32'h70070; font_rom[243]=32'h60070; font_rom[244]=32'h60070; font_rom[245]=32'he6070; font_rom[246]=32'he7070; font_rom[247]=32'hc3870;
font_rom[248]=32'h1c1c70; font_rom[249]=32'h180e70; font_rom[250]=32'h380460; font_rom[251]=32'h700060; font_rom[252]=32'he00060; font_rom[253]=32'he00060; font_rom[254]=32'h1c000e0; font_rom[255]=32'h78001e0;
font_rom[256]=32'hf007fc0; font_rom[257]=32'h1e007f80; font_rom[258]=32'h8000000; font_rom[259]=32'h10000; font_rom[260]=32'h38000; font_rom[261]=32'h1c000; font_rom[262]=32'h8000; font_rom[263]=32'hfffffe0;
font_rom[264]=32'hfffffe0; font_rom[265]=32'h1c0; font_rom[266]=32'h380; font_rom[267]=32'h700; font_rom[268]=32'he00; font_rom[269]=32'h1c00; font_rom[270]=32'h3800; font_rom[271]=32'hf000;
font_rom[272]=32'h1e000; font_rom[273]=32'h3c000; font_rom[274]=32'hf0000; font_rom[275]=32'h1e0000; font_rom[276]=32'h780000; font_rom[277]=32'hf00000; font_rom[278]=32'h3c00000; font_rom[279]=32'h7c00000;
font_rom[280]=32'hef00000; font_rom[281]=32'h1c7c0000; font_rom[282]=32'h183ffff8; font_rom[283]=32'h1007fff8; font_rom[284]=32'h7ffffe0; font_rom[285]=32'h7ffffe0; font_rom[286]=32'h0; font_rom[287]=32'h0;
font_rom[288]=32'h0; font_rom[289]=32'h0; font_rom[290]=32'h0; font_rom[291]=32'h0; font_rom[292]=32'h0; font_rom[293]=32'h0; font_rom[294]=32'h0; font_rom[295]=32'h0;
font_rom[296]=32'h0; font_rom[297]=32'h0; font_rom[298]=32'h0; font_rom[299]=32'h0; font_rom[300]=32'h0; font_rom[301]=32'h0; font_rom[302]=32'h1ffffff8; font_rom[303]=32'h1ffffff8;
font_rom[304]=32'h7ffffe0; font_rom[305]=32'h7ffffe0; font_rom[306]=32'h1c000; font_rom[307]=32'h1c000; font_rom[308]=32'h1c000; font_rom[309]=32'h1c000; font_rom[310]=32'h1c000; font_rom[311]=32'h1c000;
font_rom[312]=32'h1c000; font_rom[313]=32'h1ffffff8; font_rom[314]=32'h1ffffff8; font_rom[315]=32'h1c000; font_rom[316]=32'h1c000; font_rom[317]=32'h1c000; font_rom[318]=32'h1c000; font_rom[319]=32'h1c000;
font_rom[320]=32'h1c000; font_rom[321]=32'h1c000; font_rom[322]=32'h1c000; font_rom[323]=32'h1c000; font_rom[324]=32'h1c000; font_rom[325]=32'h18000; font_rom[326]=32'h7f8000; font_rom[327]=32'h7f8000;
font_rom[328]=32'h10000; font_rom[329]=32'h38000; font_rom[330]=32'h18000; font_rom[331]=32'h1ffffff8; font_rom[332]=32'h1ffffff8; font_rom[333]=32'h0; font_rom[334]=32'h800; font_rom[335]=32'h300e00;
font_rom[336]=32'h700700; font_rom[337]=32'he00380; font_rom[338]=32'h1c00dc0; font_rom[339]=32'h3900ce0; font_rom[340]=32'h7301c70; font_rom[341]=32'h1e381820; font_rom[342]=32'h8183800; font_rom[343]=32'hc7000;
font_rom[344]=32'he6000; font_rom[345]=32'h7e000; font_rom[346]=32'h3c000; font_rom[347]=32'h7e000; font_rom[348]=32'h1ff800; font_rom[349]=32'hfc3e00; font_rom[350]=32'h7f00fe0; font_rom[351]=32'h1f8001f8;
font_rom[352]=32'hc000030; font_rom[353]=32'h30000; font_rom[354]=32'h38000; font_rom[355]=32'h18000; font_rom[356]=32'h1ffffff8; font_rom[357]=32'h1ffffff8; font_rom[358]=32'h0; font_rom[359]=32'h3ffff80;
font_rom[360]=32'h3ffff80; font_rom[361]=32'h3800180; font_rom[362]=32'h3800180; font_rom[363]=32'h3ffff80; font_rom[364]=32'h3ffff80; font_rom[365]=32'h1000180; font_rom[366]=32'h0; font_rom[367]=32'h7ffffc0;
font_rom[368]=32'h7ffffc0; font_rom[369]=32'h3f80; font_rom[370]=32'h1f000; font_rom[371]=32'h18000; font_rom[372]=32'h3ffffff8; font_rom[373]=32'h1ffffff8; font_rom[374]=32'h18000; font_rom[375]=32'h18000;
font_rom[376]=32'h18000; font_rom[377]=32'h3f8000; font_rom[378]=32'h3f0000; font_rom[379]=32'h30000; font_rom[380]=32'h38000; font_rom[381]=32'h18000; font_rom[382]=32'h1ffffff8; font_rom[383]=32'h1ffffff8;
font_rom[384]=32'h0; font_rom[385]=32'h0; font_rom[386]=32'h3ffffc0; font_rom[387]=32'h3ffffc0; font_rom[388]=32'h30000c0; font_rom[389]=32'h30000c0; font_rom[390]=32'h30000c0; font_rom[391]=32'h30000c0;
font_rom[392]=32'h3ffffc0; font_rom[393]=32'h3ffffc0; font_rom[394]=32'h30180c0; font_rom[395]=32'h18000; font_rom[396]=32'h418400; font_rom[397]=32'he18700; font_rom[398]=32'h1c18380; font_rom[399]=32'h78181c0;
font_rom[400]=32'he0180f0; font_rom[401]=32'h1c018038; font_rom[402]=32'h183f8010; font_rom[403]=32'h1f0000; font_rom[404]=32'h10000; font_rom[405]=32'h38000; font_rom[406]=32'h18000; font_rom[407]=32'h1ffffff8;
font_rom[408]=32'h1ffffff8; font_rom[409]=32'h0; font_rom[410]=32'h1ffff80; font_rom[411]=32'h1ffff80; font_rom[412]=32'h1800180; font_rom[413]=32'h1800180; font_rom[414]=32'h1ffff80; font_rom[415]=32'h1ffff80;
font_rom[416]=32'h1800180; font_rom[417]=32'h0; font_rom[418]=32'h1ffffff8; font_rom[419]=32'h1ffffff8; font_rom[420]=32'h18000018; font_rom[421]=32'h18000018; font_rom[422]=32'h1bffff98; font_rom[423]=32'h1ffff80;
font_rom[424]=32'hc000; font_rom[425]=32'hc000; font_rom[426]=32'hc000; font_rom[427]=32'h1c000; font_rom[428]=32'h3f8000; font_rom[429]=32'h1f0000; font_rom[430]=32'h18000; font_rom[431]=32'h18000;
font_rom[432]=32'h18000; font_rom[433]=32'h18000; font_rom[434]=32'h18000; font_rom[435]=32'h38000; font_rom[436]=32'h38000; font_rom[437]=32'h38000; font_rom[438]=32'h3c000; font_rom[439]=32'h7c000;
font_rom[440]=32'h7e000; font_rom[441]=32'h66000; font_rom[442]=32'he6000; font_rom[443]=32'hc3000; font_rom[444]=32'h1c3800; font_rom[445]=32'h181800; font_rom[446]=32'h381c00; font_rom[447]=32'h700e00;
font_rom[448]=32'hf00f00; font_rom[449]=32'h1e00780; font_rom[450]=32'h3c003c0; font_rom[451]=32'h78001e0; font_rom[452]=32'h1f0000f8; font_rom[453]=32'h1c000078; font_rom[454]=32'h8000010; font_rom[455]=32'h1c000;
font_rom[456]=32'h3c000; font_rom[457]=32'h7e000; font_rom[458]=32'he6000; font_rom[459]=32'hc7000; font_rom[460]=32'h1c3800; font_rom[461]=32'h781c00; font_rom[462]=32'he00f00; font_rom[463]=32'h1c20780;
font_rom[464]=32'h78301e0; font_rom[465]=32'h1f0380f8; font_rom[466]=32'h3c018038; font_rom[467]=32'h1001c018; font_rom[468]=32'h8000; font_rom[469]=32'h0; font_rom[470]=32'h7ffff80; font_rom[471]=32'h7ffff80;
font_rom[472]=32'h380; font_rom[473]=32'h700; font_rom[474]=32'he00; font_rom[475]=32'h1e00; font_rom[476]=32'h1c00; font_rom[477]=32'h3800; font_rom[478]=32'h7000; font_rom[479]=32'he000;
font_rom[480]=32'h4000; font_rom[481]=32'h601800; font_rom[482]=32'h601800; font_rom[483]=32'he01800; font_rom[484]=32'hc01800; font_rom[485]=32'hc01800; font_rom[486]=32'h1801800; font_rom[487]=32'h1801800;
font_rom[488]=32'h3801800; font_rom[489]=32'h3801800; font_rom[490]=32'h79ffffc; font_rom[491]=32'h79ffffc; font_rom[492]=32'hf801800; font_rom[493]=32'h1f801800; font_rom[494]=32'h1b801800; font_rom[495]=32'h13801800;
font_rom[496]=32'h13801800; font_rom[497]=32'h3801800; font_rom[498]=32'h3801800; font_rom[499]=32'h3801800; font_rom[500]=32'h3801800; font_rom[501]=32'h3801800; font_rom[502]=32'h3801800; font_rom[503]=32'h3801800;
font_rom[504]=32'h38ffff8; font_rom[505]=32'h38ffff8; font_rom[506]=32'h3800000; font_rom[507]=32'h3800000; font_rom[508]=32'h40c000; font_rom[509]=32'he0e200; font_rom[510]=32'hc0e780; font_rom[511]=32'h1c0e1c0;
font_rom[512]=32'h180e0e0; font_rom[513]=32'h380e040; font_rom[514]=32'h380e000; font_rom[515]=32'h780fff8; font_rom[516]=32'hfbffff8; font_rom[517]=32'hfbff000; font_rom[518]=32'h1f806000; font_rom[519]=32'h3b806000;
font_rom[520]=32'h13807000; font_rom[521]=32'h3807000; font_rom[522]=32'h3803000; font_rom[523]=32'h3803000; font_rom[524]=32'h3803800; font_rom[525]=32'h3801800; font_rom[526]=32'h3801c18; font_rom[527]=32'h3801c18;
font_rom[528]=32'h3800e18; font_rom[529]=32'h3800718; font_rom[530]=32'h38007f8; font_rom[531]=32'h38003f0; font_rom[532]=32'h38000e0; font_rom[533]=32'h40000; font_rom[534]=32'h60c0180; font_rom[535]=32'h60e0180;
font_rom[536]=32'h6070180; font_rom[537]=32'h6030180; font_rom[538]=32'h6038180; font_rom[539]=32'h6018180; font_rom[540]=32'h6010180; font_rom[541]=32'h6000180; font_rom[542]=32'h6000180; font_rom[543]=32'h6000180;
font_rom[544]=32'h6000180; font_rom[545]=32'h6000380; font_rom[546]=32'h6000300; font_rom[547]=32'h6000300; font_rom[548]=32'h6020700; font_rom[549]=32'h60e0700; font_rom[550]=32'h63e0f80; font_rom[551]=32'h6f81fc0;
font_rom[552]=32'hfe039e0; font_rom[553]=32'hf8070f0; font_rom[554]=32'he01e078; font_rom[555]=32'h407c038; font_rom[556]=32'hf0010; font_rom[557]=32'hc0000; font_rom[558]=32'hc40000; font_rom[559]=32'hc60000;
font_rom[560]=32'hc71ff0; font_rom[561]=32'h1c39ff0; font_rom[562]=32'h1810030; font_rom[563]=32'h3800030; font_rom[564]=32'h30c0030; font_rom[565]=32'h70c0030; font_rom[566]=32'h70c0030; font_rom[567]=32'hf0c0030;
font_rom[568]=32'hf0c0030; font_rom[569]=32'h1b0c0030; font_rom[570]=32'h3b0c0030; font_rom[571]=32'h130c0030; font_rom[572]=32'h30c0030; font_rom[573]=32'h30c0030; font_rom[574]=32'h30c0030; font_rom[575]=32'h30c0030;
font_rom[576]=32'h30c0030; font_rom[577]=32'h30c0030; font_rom[578]=32'h30c0030; font_rom[579]=32'h30c0030; font_rom[580]=32'h30c0070; font_rom[581]=32'h30c0ff0; font_rom[582]=32'h30c0fe0; font_rom[583]=32'h30c0000;
font_rom[584]=32'hc03800; font_rom[585]=32'hc03800; font_rom[586]=32'h1c07c00; font_rom[587]=32'h180ec00; font_rom[588]=32'h181c600; font_rom[589]=32'h3838300; font_rom[590]=32'h3070380; font_rom[591]=32'h30e01e0;
font_rom[592]=32'h71c0070; font_rom[593]=32'h7380038; font_rom[594]=32'hf238018; font_rom[595]=32'hf038300; font_rom[596]=32'h1b038300; font_rom[597]=32'h3b030300; font_rom[598]=32'h13030300; font_rom[599]=32'h3030300;
font_rom[600]=32'h3030300; font_rom[601]=32'h3030300; font_rom[602]=32'h3030300; font_rom[603]=32'h3070300; font_rom[604]=32'h3060300; font_rom[605]=32'h3060300; font_rom[606]=32'h30e0300; font_rom[607]=32'h31c0300;
font_rom[608]=32'h3380300; font_rom[609]=32'h3100300; font_rom[610]=32'hc18000; font_rom[611]=32'hc18000; font_rom[612]=32'hc38000; font_rom[613]=32'h1c3fff8; font_rom[614]=32'h187fff8; font_rom[615]=32'h38e0000;
font_rom[616]=32'h30c6000; font_rom[617]=32'h71c6000; font_rom[618]=32'h7386000; font_rom[619]=32'hf106000; font_rom[620]=32'hf1ffff0; font_rom[621]=32'h1f1ffff0; font_rom[622]=32'h3b006030; font_rom[623]=32'h13006030;
font_rom[624]=32'h13006030; font_rom[625]=32'h3006030; font_rom[626]=32'h300e030; font_rom[627]=32'h300c030; font_rom[628]=32'h301c030; font_rom[629]=32'h3018030; font_rom[630]=32'h3038030; font_rom[631]=32'h3070070;
font_rom[632]=32'h30e0060; font_rom[633]=32'h31c1fe0; font_rom[634]=32'h3381fc0; font_rom[635]=32'h3100000; font_rom[636]=32'hc00000; font_rom[637]=32'hc00000; font_rom[638]=32'h1dffffc; font_rom[639]=32'h19ffffc;
font_rom[640]=32'h1800060; font_rom[641]=32'h3800060; font_rom[642]=32'h3000060; font_rom[643]=32'h7000060; font_rom[644]=32'h70ff860; font_rom[645]=32'hf0ff860; font_rom[646]=32'hf0c1860; font_rom[647]=32'h1f0c1860;
font_rom[648]=32'h3b0c1860; font_rom[649]=32'h130c1860; font_rom[650]=32'h130c1860; font_rom[651]=32'h30c1860; font_rom[652]=32'h30c1860; font_rom[653]=32'h30ff860; font_rom[654]=32'h30ff860; font_rom[655]=32'h30c0060;
font_rom[656]=32'h3000060; font_rom[657]=32'h3000060; font_rom[658]=32'h3000060; font_rom[659]=32'h30000e0; font_rom[660]=32'h3001fe0; font_rom[661]=32'h3001f80; font_rom[662]=32'hc18000; font_rom[663]=32'hc18000;
font_rom[664]=32'hc30000; font_rom[665]=32'h1c3fffc; font_rom[666]=32'h187fff8; font_rom[667]=32'h3867000; font_rom[668]=32'h30c7000; font_rom[669]=32'h71c7000; font_rom[670]=32'h7187000; font_rom[671]=32'hf387000;
font_rom[672]=32'hf107ff0; font_rom[673]=32'h1b007ff0; font_rom[674]=32'h3b007000; font_rom[675]=32'h13007000; font_rom[676]=32'h3007000; font_rom[677]=32'h3007000; font_rom[678]=32'h3007000; font_rom[679]=32'h3007ff8;
font_rom[680]=32'h3007ff8; font_rom[681]=32'h3007000; font_rom[682]=32'h3007000; font_rom[683]=32'h3007000; font_rom[684]=32'h3007000; font_rom[685]=32'h3007000; font_rom[686]=32'h3007000; font_rom[687]=32'h3007000;
font_rom[688]=32'hc02000; font_rom[689]=32'hc07000; font_rom[690]=32'h1803800; font_rom[691]=32'h1801000; font_rom[692]=32'h1bffff8; font_rom[693]=32'h33ffff8; font_rom[694]=32'h3007000; font_rom[695]=32'h700f000;
font_rom[696]=32'h700f000; font_rom[697]=32'hf01f020; font_rom[698]=32'hf03b870; font_rom[699]=32'h1f0318e0; font_rom[700]=32'h3b071bc0; font_rom[701]=32'h130f1f00; font_rom[702]=32'h31f0e00; font_rom[703]=32'h33f0e00;
font_rom[704]=32'h3770600; font_rom[705]=32'h3e70700; font_rom[706]=32'h3470380; font_rom[707]=32'h30701c0; font_rom[708]=32'h30738e0; font_rom[709]=32'h307f078; font_rom[710]=32'h307e038; font_rom[711]=32'h3078010;
font_rom[712]=32'h3070000; font_rom[713]=32'h3020000; font_rom[714]=32'hc18300; font_rom[715]=32'hc18300; font_rom[716]=32'hc18300; font_rom[717]=32'h1dffff0; font_rom[718]=32'h19ffff0; font_rom[719]=32'h3818300;
font_rom[720]=32'h3018300; font_rom[721]=32'h7018300; font_rom[722]=32'h7018300; font_rom[723]=32'hf3ffff8; font_rom[724]=32'h1f3ffff8; font_rom[725]=32'h1b000000; font_rom[726]=32'h1b000000; font_rom[727]=32'h130fffe0;
font_rom[728]=32'h30fffe0; font_rom[729]=32'h30c0060; font_rom[730]=32'h30c0060; font_rom[731]=32'h30fffe0; font_rom[732]=32'h30fffe0; font_rom[733]=32'h30c0060; font_rom[734]=32'h30c0060; font_rom[735]=32'h30c0060;
font_rom[736]=32'h30fffe0; font_rom[737]=32'h30fffe0; font_rom[738]=32'h30c0060; font_rom[739]=32'hc03000; font_rom[740]=32'hc03000; font_rom[741]=32'h1803000; font_rom[742]=32'h1bffff8; font_rom[743]=32'h1bffff8;
font_rom[744]=32'h3003000; font_rom[745]=32'h3003000; font_rom[746]=32'h30fffe0; font_rom[747]=32'h70fffe0; font_rom[748]=32'h70c0060; font_rom[749]=32'hf0c0060; font_rom[750]=32'hf0c0060; font_rom[751]=32'h1f0fffe0;
font_rom[752]=32'h3b0fffe0; font_rom[753]=32'h130c0060; font_rom[754]=32'h30c0060; font_rom[755]=32'h30fffe0; font_rom[756]=32'h30fffe0; font_rom[757]=32'h30c0060; font_rom[758]=32'h30c0060; font_rom[759]=32'h30fffe0;
font_rom[760]=32'h30fffe0; font_rom[761]=32'h30c0060; font_rom[762]=32'h30c0060; font_rom[763]=32'h37ffffc; font_rom[764]=32'h37ffffc; font_rom[765]=32'h3000000; font_rom[766]=32'h1800000; font_rom[767]=32'h1b00000;
font_rom[768]=32'h1b1fffc; font_rom[769]=32'h331fff8; font_rom[770]=32'h3300600; font_rom[771]=32'h3300600; font_rom[772]=32'h7300e00; font_rom[773]=32'h7307ff0; font_rom[774]=32'h73e7ff0; font_rom[775]=32'hf3e4030;
font_rom[776]=32'hf304630; font_rom[777]=32'h1b304630; font_rom[778]=32'h1b304630; font_rom[779]=32'h13304630; font_rom[780]=32'h3304630; font_rom[781]=32'h3304630; font_rom[782]=32'h3304630; font_rom[783]=32'h3304630;
font_rom[784]=32'h3334630; font_rom[785]=32'h33f4f30; font_rom[786]=32'h33c0f80; font_rom[787]=32'h3301dc0; font_rom[788]=32'h30038e0; font_rom[789]=32'h3007030; font_rom[790]=32'h301e018; font_rom[791]=32'h3008008;
font_rom[792]=32'hc01800; font_rom[793]=32'hcc1830; font_rom[794]=32'h1cc1830; font_rom[795]=32'h18c1830; font_rom[796]=32'h18c1830; font_rom[797]=32'h30ffff0; font_rom[798]=32'h30ffff0; font_rom[799]=32'h7020830;
font_rom[800]=32'h7070c00; font_rom[801]=32'hf060600; font_rom[802]=32'hf0ffff8; font_rom[803]=32'h1f0ffff8; font_rom[804]=32'h3b1c0c00; font_rom[805]=32'h133c0c00; font_rom[806]=32'h33ffff8; font_rom[807]=32'h36ffff0;
font_rom[808]=32'h30c0c00; font_rom[809]=32'h30c0c00; font_rom[810]=32'h30ffff8; font_rom[811]=32'h30ffff0; font_rom[812]=32'h30c0c00; font_rom[813]=32'h30c0c00; font_rom[814]=32'h30c0c00; font_rom[815]=32'h30ffff8;
font_rom[816]=32'h30ffff8; font_rom[817]=32'h1c000; font_rom[818]=32'h3c000; font_rom[819]=32'h7e000; font_rom[820]=32'he7000; font_rom[821]=32'h1c3800; font_rom[822]=32'h381c00; font_rom[823]=32'hf00e00;
font_rom[824]=32'h1e00780; font_rom[825]=32'h78001c0; font_rom[826]=32'hf0000f0; font_rom[827]=32'h1dfffff8; font_rom[828]=32'h19ffff98; font_rom[829]=32'h18000; font_rom[830]=32'h18000; font_rom[831]=32'h18000;
font_rom[832]=32'h18000; font_rom[833]=32'h3ffffc0; font_rom[834]=32'h3ffffc0; font_rom[835]=32'h18000; font_rom[836]=32'h18000; font_rom[837]=32'h18000; font_rom[838]=32'h18000; font_rom[839]=32'h18000;
font_rom[840]=32'hffffff0; font_rom[841]=32'hffffff0; font_rom[842]=32'h400200; font_rom[843]=32'he00700; font_rom[844]=32'h700e00; font_rom[845]=32'h780e00; font_rom[846]=32'h381c00; font_rom[847]=32'h103800;
font_rom[848]=32'hffffff0; font_rom[849]=32'hffffff0; font_rom[850]=32'h0; font_rom[851]=32'h0; font_rom[852]=32'h0; font_rom[853]=32'h0; font_rom[854]=32'h0; font_rom[855]=32'h0;
font_rom[856]=32'h7ffffc0; font_rom[857]=32'h7ffffc0; font_rom[858]=32'h0; font_rom[859]=32'h0; font_rom[860]=32'h0; font_rom[861]=32'h0; font_rom[862]=32'h0; font_rom[863]=32'h0;
font_rom[864]=32'h1ffffff8; font_rom[865]=32'h1ffffff8; font_rom[866]=32'hc3000; font_rom[867]=32'hc3000; font_rom[868]=32'hc3000; font_rom[869]=32'h3ffffe0; font_rom[870]=32'h3ffffe0; font_rom[871]=32'h30c3060;
font_rom[872]=32'h30c3060; font_rom[873]=32'h30c3060; font_rom[874]=32'h30c3060; font_rom[875]=32'h30c3060; font_rom[876]=32'h3ffffe0; font_rom[877]=32'h3ffffe0; font_rom[878]=32'h30c3060; font_rom[879]=32'h30c3060;
font_rom[880]=32'h30c3060; font_rom[881]=32'h30c3060; font_rom[882]=32'h30c3060; font_rom[883]=32'h1ffffff8; font_rom[884]=32'h1ffffff8; font_rom[885]=32'h0; font_rom[886]=32'h181800; font_rom[887]=32'h781e00;
font_rom[888]=32'h1f00f80; font_rom[889]=32'h7c001e0; font_rom[890]=32'h1e000078; font_rom[891]=32'h8000010; font_rom[892]=32'hffffff0; font_rom[893]=32'hffffff0; font_rom[894]=32'hc000030; font_rom[895]=32'hc600030;
font_rom[896]=32'hce00030; font_rom[897]=32'he00000; font_rom[898]=32'hffffc0; font_rom[899]=32'hffffc0; font_rom[900]=32'hc00000; font_rom[901]=32'h1c00000; font_rom[902]=32'h1800000; font_rom[903]=32'h1ffffe0;
font_rom[904]=32'h1ffffe0; font_rom[905]=32'h60; font_rom[906]=32'h60; font_rom[907]=32'h60; font_rom[908]=32'h3ffffe60; font_rom[909]=32'h3ffffe60; font_rom[910]=32'he0; font_rom[911]=32'he0;
font_rom[912]=32'hc0; font_rom[913]=32'h1c0; font_rom[914]=32'hffc0; font_rom[915]=32'hff00; font_rom[916]=32'h3000; font_rom[917]=32'h7800; font_rom[918]=32'h8007800; font_rom[919]=32'h1c00cc00;
font_rom[920]=32'he01ce00; font_rom[921]=32'h7038600; font_rom[922]=32'h3870300; font_rom[923]=32'h10e41c0; font_rom[924]=32'h1ce0e0; font_rom[925]=32'h787078; font_rom[926]=32'hf0383c; font_rom[927]=32'h401808;
font_rom[928]=32'h1000; font_rom[929]=32'h2000000; font_rom[930]=32'h33fffe0; font_rom[931]=32'h33fffe0; font_rom[932]=32'h70000e0; font_rom[933]=32'h60001c0; font_rom[934]=32'h6000380; font_rom[935]=32'he0c0700;
font_rom[936]=32'he0f1e00; font_rom[937]=32'hc03fc00; font_rom[938]=32'h1c00f800; font_rom[939]=32'h1c003e00; font_rom[940]=32'h8000f00; font_rom[941]=32'h200; font_rom[942]=32'h3000; font_rom[943]=32'h8003000;
font_rom[944]=32'h1c003000; font_rom[945]=32'hc3ffff0; font_rom[946]=32'he3ffff0; font_rom[947]=32'h6003000; font_rom[948]=32'h71fffe0; font_rom[949]=32'h31fffe0; font_rom[950]=32'h2003060; font_rom[951]=32'h3060;
font_rom[952]=32'h7ffff8; font_rom[953]=32'h7ffff8; font_rom[954]=32'h3060; font_rom[955]=32'h2003060; font_rom[956]=32'h31fffe0; font_rom[957]=32'h71fffe0; font_rom[958]=32'h600c000; font_rom[959]=32'h6018000;
font_rom[960]=32'h67ffff8; font_rom[961]=32'he7ffff8; font_rom[962]=32'hc060300; font_rom[963]=32'hc0e0700; font_rom[964]=32'h1c07fe00; font_rom[965]=32'h1800fc00; font_rom[966]=32'h807ff80; font_rom[967]=32'h7f83f0;
font_rom[968]=32'h380060; font_rom[969]=32'h400000; font_rom[970]=32'h413ff8; font_rom[971]=32'h847bff8; font_rom[972]=32'h187e0030; font_rom[973]=32'h1c780860; font_rom[974]=32'hc618ce0; font_rom[975]=32'he418ec0;
font_rom[976]=32'h6618780; font_rom[977]=32'h47f8380; font_rom[978]=32'h3e0180; font_rom[979]=32'h603ff8; font_rom[980]=32'h603ff8; font_rom[981]=32'h47f8318; font_rom[982]=32'h6ff8310; font_rom[983]=32'hecc3300;
font_rom[984]=32'hc8c3300; font_rom[985]=32'hc0833f8; font_rom[986]=32'hdffb3f0; font_rom[987]=32'hcffb300; font_rom[988]=32'hc183300; font_rom[989]=32'h1c1c3300; font_rom[990]=32'h183e3b00; font_rom[991]=32'h18377f00;
font_rom[992]=32'h18626f00; font_rom[993]=32'h18c0c7f8; font_rom[994]=32'h19c1c1f8; font_rom[995]=32'h808000; font_rom[996]=32'h18000; font_rom[997]=32'h18000; font_rom[998]=32'h70180e0; font_rom[999]=32'h70180e0;
font_rom[1000]=32'h70180e0; font_rom[1001]=32'h70180e0; font_rom[1002]=32'h70180e0; font_rom[1003]=32'h70180e0; font_rom[1004]=32'h70180e0; font_rom[1005]=32'h70180e0; font_rom[1006]=32'h7ffffe0; font_rom[1007]=32'h7ffffe0;
font_rom[1008]=32'h180e0; font_rom[1009]=32'h18000; font_rom[1010]=32'h18000; font_rom[1011]=32'h6018060; font_rom[1012]=32'h6018060; font_rom[1013]=32'h6018060; font_rom[1014]=32'h6018060; font_rom[1015]=32'h6018060;
font_rom[1016]=32'h6018060; font_rom[1017]=32'h6018060; font_rom[1018]=32'h6018060; font_rom[1019]=32'h7ffffe0; font_rom[1020]=32'h7ffffe0; font_rom[1021]=32'h60; font_rom[1022]=32'h82000; font_rom[1023]=32'h1c3000;
font_rom[1024]=32'h383000; font_rom[1025]=32'h301800; font_rom[1026]=32'h701c00; font_rom[1027]=32'he00c00; font_rom[1028]=32'h1c00600; font_rom[1029]=32'h3800700; font_rom[1030]=32'h38003c0; font_rom[1031]=32'hf0001e0;
font_rom[1032]=32'h1e0000f8; font_rom[1033]=32'h1c000038; font_rom[1034]=32'h13ffff10; font_rom[1035]=32'h3ffff00; font_rom[1036]=32'h180300; font_rom[1037]=32'h180300; font_rom[1038]=32'h180300; font_rom[1039]=32'h380300;
font_rom[1040]=32'h300300; font_rom[1041]=32'h700700; font_rom[1042]=32'he00700; font_rom[1043]=32'h1c00e00; font_rom[1044]=32'h783fe00; font_rom[1045]=32'h1f03fc00; font_rom[1046]=32'hc000000; font_rom[1047]=32'h3000000;
font_rom[1048]=32'h3000000; font_rom[1049]=32'h30ffff0; font_rom[1050]=32'h30ffff0; font_rom[1051]=32'h3006030; font_rom[1052]=32'h3006030; font_rom[1053]=32'h3006030; font_rom[1054]=32'h3fc6030; font_rom[1055]=32'h1ffc6030;
font_rom[1056]=32'h1f806030; font_rom[1057]=32'h3006030; font_rom[1058]=32'h3006030; font_rom[1059]=32'h3006030; font_rom[1060]=32'h3006030; font_rom[1061]=32'h3006030; font_rom[1062]=32'h304e070; font_rom[1063]=32'h30cc070;
font_rom[1064]=32'h33cc070; font_rom[1065]=32'h379c070; font_rom[1066]=32'h3e18060; font_rom[1067]=32'h3c38060; font_rom[1068]=32'h7030060; font_rom[1069]=32'h20700e0; font_rom[1070]=32'h1e3fe0; font_rom[1071]=32'h3c3fc0;
font_rom[1072]=32'h100000; font_rom[1073]=32'h30; font_rom[1074]=32'hfff8030; font_rom[1075]=32'hfff8c30; font_rom[1076]=32'hc018c30; font_rom[1077]=32'hc018c30; font_rom[1078]=32'hc018c30; font_rom[1079]=32'hc018c30;
font_rom[1080]=32'hfff8c30; font_rom[1081]=32'hfff8c30; font_rom[1082]=32'hc018c30; font_rom[1083]=32'hc00c30; font_rom[1084]=32'hc00c30; font_rom[1085]=32'hc00c30; font_rom[1086]=32'h1fff8c30; font_rom[1087]=32'h1fff8c30;
font_rom[1088]=32'hc18c30; font_rom[1089]=32'h1c18c30; font_rom[1090]=32'h1818c30; font_rom[1091]=32'h1818c30; font_rom[1092]=32'h3818030; font_rom[1093]=32'h3038030; font_rom[1094]=32'h7030070; font_rom[1095]=32'he7f0ff0;
font_rom[1096]=32'h1c7e0fe0; font_rom[1097]=32'h18000000; font_rom[1098]=32'h30; font_rom[1099]=32'h30; font_rom[1100]=32'h1fffe030; font_rom[1101]=32'h1fffec30; font_rom[1102]=32'hc00c30; font_rom[1103]=32'h1c00c30;
font_rom[1104]=32'h1860c30; font_rom[1105]=32'h3070c30; font_rom[1106]=32'h7030c30; font_rom[1107]=32'he7f8c30; font_rom[1108]=32'hfffcc30; font_rom[1109]=32'hf80cc30; font_rom[1110]=32'h8c30; font_rom[1111]=32'h300c30;
font_rom[1112]=32'h300c30; font_rom[1113]=32'h300c30; font_rom[1114]=32'h1fffcc30; font_rom[1115]=32'h1fffcc30; font_rom[1116]=32'h300c30; font_rom[1117]=32'h300c30; font_rom[1118]=32'h300030; font_rom[1119]=32'h33c030;
font_rom[1120]=32'h7ffe030; font_rom[1121]=32'h1fff0030; font_rom[1122]=32'h1e0007f0; font_rom[1123]=32'h7e0; font_rom[1124]=32'h200000; font_rom[1125]=32'h700030; font_rom[1126]=32'h300030; font_rom[1127]=32'h100030;
font_rom[1128]=32'h1fffe630; font_rom[1129]=32'h1fffe630; font_rom[1130]=32'hc00630; font_rom[1131]=32'h1800630; font_rom[1132]=32'h1820630; font_rom[1133]=32'h3070630; font_rom[1134]=32'h7060630; font_rom[1135]=32'h60c0630;
font_rom[1136]=32'hffcc630; font_rom[1137]=32'hff8c630; font_rom[1138]=32'h31c630; font_rom[1139]=32'h618630; font_rom[1140]=32'hc30630; font_rom[1141]=32'h3870630; font_rom[1142]=32'h70e0630; font_rom[1143]=32'he0e0630;
font_rom[1144]=32'h1c1e0630; font_rom[1145]=32'h18730030; font_rom[1146]=32'he18030; font_rom[1147]=32'h1c1e030; font_rom[1148]=32'h780e070; font_rom[1149]=32'hf0047f0; font_rom[1150]=32'h40007e0; font_rom[1151]=32'h200400;
font_rom[1152]=32'h300e00; font_rom[1153]=32'h381c00; font_rom[1154]=32'h181800; font_rom[1155]=32'h1ffffff8; font_rom[1156]=32'h1ffffff8; font_rom[1157]=32'h0; font_rom[1158]=32'h0; font_rom[1159]=32'hfff0c60;
font_rom[1160]=32'hfff0c60; font_rom[1161]=32'he030c60; font_rom[1162]=32'he030c60; font_rom[1163]=32'hfff0c60; font_rom[1164]=32'hfff0c60; font_rom[1165]=32'he030c60; font_rom[1166]=32'he030c60; font_rom[1167]=32'he030c60;
font_rom[1168]=32'hfff0c60; font_rom[1169]=32'hfff0c60; font_rom[1170]=32'he030c60; font_rom[1171]=32'he030060; font_rom[1172]=32'he030060; font_rom[1173]=32'he030060; font_rom[1174]=32'he3f0fe0; font_rom[1175]=32'he3e07c0;
font_rom[1176]=32'h60000; font_rom[1177]=32'h60000; font_rom[1178]=32'h60000; font_rom[1179]=32'h60000; font_rom[1180]=32'h60000; font_rom[1181]=32'hfffffe0; font_rom[1182]=32'hfffffe0; font_rom[1183]=32'he0060;
font_rom[1184]=32'he0060; font_rom[1185]=32'he0060; font_rom[1186]=32'he0060; font_rom[1187]=32'he0060; font_rom[1188]=32'hc0060; font_rom[1189]=32'hc0060; font_rom[1190]=32'h1c0060; font_rom[1191]=32'h1c0060;
font_rom[1192]=32'h380060; font_rom[1193]=32'h380060; font_rom[1194]=32'h700060; font_rom[1195]=32'he000e0; font_rom[1196]=32'h1c000e0; font_rom[1197]=32'h78001c0; font_rom[1198]=32'hf00ffc0; font_rom[1199]=32'h1e00ff80;
font_rom[1200]=32'h8000000; font_rom[1201]=32'h606000; font_rom[1202]=32'h606000; font_rom[1203]=32'he06000; font_rom[1204]=32'he06020; font_rom[1205]=32'hc06070; font_rom[1206]=32'h1c060f0; font_rom[1207]=32'h18061e0;
font_rom[1208]=32'h38063c0; font_rom[1209]=32'h7806780; font_rom[1210]=32'h7806f00; font_rom[1211]=32'hf807e00; font_rom[1212]=32'h1d807800; font_rom[1213]=32'h1980f000; font_rom[1214]=32'h1983e000; font_rom[1215]=32'h1187e000;
font_rom[1216]=32'h19f6000; font_rom[1217]=32'h1bc6018; font_rom[1218]=32'h190601c; font_rom[1219]=32'h1806018; font_rom[1220]=32'h1806018; font_rom[1221]=32'h1806018; font_rom[1222]=32'h1807038; font_rom[1223]=32'h1807ff0;
font_rom[1224]=32'h1803fe0; font_rom[1225]=32'h1800000; font_rom[1226]=32'h600; font_rom[1227]=32'h1c600; font_rom[1228]=32'hffc600; font_rom[1229]=32'h1fff0600; font_rom[1230]=32'hff00600; font_rom[1231]=32'h300600;
font_rom[1232]=32'h300600; font_rom[1233]=32'h300600; font_rom[1234]=32'h300600; font_rom[1235]=32'h300600; font_rom[1236]=32'h300600; font_rom[1237]=32'h1ffffff8; font_rom[1238]=32'h1ffffff8; font_rom[1239]=32'h300600;
font_rom[1240]=32'h300600; font_rom[1241]=32'h300600; font_rom[1242]=32'h700600; font_rom[1243]=32'h700600; font_rom[1244]=32'h600600; font_rom[1245]=32'h600600; font_rom[1246]=32'he00600; font_rom[1247]=32'h1c00600;
font_rom[1248]=32'h3800600; font_rom[1249]=32'h7000600; font_rom[1250]=32'h1e000600; font_rom[1251]=32'hc000600; font_rom[1252]=32'h7fffff8; font_rom[1253]=32'h7fffff8; font_rom[1254]=32'h700e000; font_rom[1255]=32'h700c000;
font_rom[1256]=32'h700c000; font_rom[1257]=32'h71fffe0; font_rom[1258]=32'h71fffe0; font_rom[1259]=32'h7180060; font_rom[1260]=32'h7180060; font_rom[1261]=32'h61fffe0; font_rom[1262]=32'h61fffe0; font_rom[1263]=32'h6180060;
font_rom[1264]=32'h6180060; font_rom[1265]=32'h61fffe0; font_rom[1266]=32'h61fffe0; font_rom[1267]=32'h6003000; font_rom[1268]=32'h6043100; font_rom[1269]=32'h60c3380; font_rom[1270]=32'he1c31c0; font_rom[1271]=32'hc3830e0;
font_rom[1272]=32'hc703070; font_rom[1273]=32'h1ce07038; font_rom[1274]=32'h18c7f018; font_rom[1275]=32'h38c7e010; font_rom[1276]=32'h10000000; font_rom[1277]=32'h18000; font_rom[1278]=32'h18000; font_rom[1279]=32'h18000;
font_rom[1280]=32'h18000; font_rom[1281]=32'h7ffffe0; font_rom[1282]=32'h7ffffe0; font_rom[1283]=32'h18000; font_rom[1284]=32'h18000; font_rom[1285]=32'h18000; font_rom[1286]=32'h18000; font_rom[1287]=32'h18000;
font_rom[1288]=32'h18000; font_rom[1289]=32'h1ffffff8; font_rom[1290]=32'h1ffffff8; font_rom[1291]=32'h70000; font_rom[1292]=32'he0000; font_rom[1293]=32'h1c0800; font_rom[1294]=32'h380c00; font_rom[1295]=32'h300e00;
font_rom[1296]=32'h600700; font_rom[1297]=32'he00180; font_rom[1298]=32'h1c0ff80; font_rom[1299]=32'h7ffffc0; font_rom[1300]=32'h3fe0060; font_rom[1301]=32'h2000070; font_rom[1302]=32'h20; font_rom[1303]=32'h1ffdfff0;
font_rom[1304]=32'h1ffdfff0; font_rom[1305]=32'hcc070; font_rom[1306]=32'hcc070; font_rom[1307]=32'h80cc060; font_rom[1308]=32'h1c1cc060; font_rom[1309]=32'hc18c060; font_rom[1310]=32'h618c060; font_rom[1311]=32'h71860e0;
font_rom[1312]=32'h31860c0; font_rom[1313]=32'h3b060c0; font_rom[1314]=32'h1b061c0; font_rom[1315]=32'hf03180; font_rom[1316]=32'he03380; font_rom[1317]=32'h603b00; font_rom[1318]=32'hf01f00; font_rom[1319]=32'hf00e00;
font_rom[1320]=32'h1980e00; font_rom[1321]=32'h39c1f00; font_rom[1322]=32'h70c3b80; font_rom[1323]=32'he0ef1e0; font_rom[1324]=32'h1c05c0f0; font_rom[1325]=32'h1c078038; font_rom[1326]=32'h180f0010; font_rom[1327]=32'h40000;
font_rom[1328]=32'h60c00; font_rom[1329]=32'h3060e00; font_rom[1330]=32'h7060780; font_rom[1331]=32'h60e03c0; font_rom[1332]=32'he0e0080; font_rom[1333]=32'hc0c0000; font_rom[1334]=32'hffffff8; font_rom[1335]=32'hffffff8;
font_rom[1336]=32'h1c0000; font_rom[1337]=32'h180000; font_rom[1338]=32'h180000; font_rom[1339]=32'h3fffc0; font_rom[1340]=32'h3fffc0; font_rom[1341]=32'h7c01c0; font_rom[1342]=32'h660180; font_rom[1343]=32'he60380;
font_rom[1344]=32'hc30300; font_rom[1345]=32'h1c38700; font_rom[1346]=32'h181ce00; font_rom[1347]=32'h300fc00; font_rom[1348]=32'h7007800; font_rom[1349]=32'he01fc00; font_rom[1350]=32'h1c07cf00; font_rom[1351]=32'h183f03f0;
font_rom[1352]=32'h11f800f8; font_rom[1353]=32'hc00030; font_rom[1354]=32'hfe0; font_rom[1355]=32'hfffffe0; font_rom[1356]=32'h7ffe000; font_rom[1357]=32'h1030100; font_rom[1358]=32'h3830380; font_rom[1359]=32'h1838380;
font_rom[1360]=32'hc18700; font_rom[1361]=32'h818e00; font_rom[1362]=32'h1ffffff8; font_rom[1363]=32'h1ffffff8; font_rom[1364]=32'h1c000038; font_rom[1365]=32'h1c000038; font_rom[1366]=32'h1c000038; font_rom[1367]=32'h1dffff38;
font_rom[1368]=32'h1ffff00; font_rom[1369]=32'h600700; font_rom[1370]=32'h300e00; font_rom[1371]=32'h381c00; font_rom[1372]=32'h1c3800; font_rom[1373]=32'he7000; font_rom[1374]=32'h7c000; font_rom[1375]=32'h7e000;
font_rom[1376]=32'h3efc00; font_rom[1377]=32'h3f83fe0; font_rom[1378]=32'h1fc007f8; font_rom[1379]=32'h1c000070; font_rom[1380]=32'h18000; font_rom[1381]=32'h18000; font_rom[1382]=32'h18000; font_rom[1383]=32'h18000;
font_rom[1384]=32'h18000; font_rom[1385]=32'h3ffffffc; font_rom[1386]=32'h3ffffffc; font_rom[1387]=32'h18000; font_rom[1388]=32'h18000; font_rom[1389]=32'h18000; font_rom[1390]=32'h18000; font_rom[1391]=32'h18000;
font_rom[1392]=32'h18000; font_rom[1393]=32'h3ffffc0; font_rom[1394]=32'h3ffffc0; font_rom[1395]=32'h30000c0; font_rom[1396]=32'h30000c0; font_rom[1397]=32'h30000c0; font_rom[1398]=32'h30000c0; font_rom[1399]=32'h30000c0;
font_rom[1400]=32'h30000c0; font_rom[1401]=32'h30000c0; font_rom[1402]=32'h3ffffc0; font_rom[1403]=32'h3ffffc0; font_rom[1404]=32'h30000c0; font_rom[1405]=32'h30000c0; font_rom[1406]=32'h600000; font_rom[1407]=32'h600000;
font_rom[1408]=32'he00000; font_rom[1409]=32'hffffe0; font_rom[1410]=32'h1ffffe0; font_rom[1411]=32'h1800060; font_rom[1412]=32'h3000060; font_rom[1413]=32'h7000060; font_rom[1414]=32'hfffe060; font_rom[1415]=32'h1dffe060;
font_rom[1416]=32'h19806060; font_rom[1417]=32'h9806060; font_rom[1418]=32'h1806060; font_rom[1419]=32'h1806060; font_rom[1420]=32'h1806060; font_rom[1421]=32'h1806060; font_rom[1422]=32'h1806060; font_rom[1423]=32'h1ffe060;
font_rom[1424]=32'h1ffe060; font_rom[1425]=32'h1800060; font_rom[1426]=32'h18000e0; font_rom[1427]=32'he0; font_rom[1428]=32'h1c0; font_rom[1429]=32'h1ffc0; font_rom[1430]=32'h1ff00; font_rom[1431]=32'h1c000;
font_rom[1432]=32'h3c000; font_rom[1433]=32'h7e000; font_rom[1434]=32'he3000; font_rom[1435]=32'h3c1800; font_rom[1436]=32'h700e00; font_rom[1437]=32'h1e00700; font_rom[1438]=32'h3c001c0; font_rom[1439]=32'hffffff0;
font_rom[1440]=32'h1cffff38; font_rom[1441]=32'h18000018; font_rom[1442]=32'h0; font_rom[1443]=32'h0; font_rom[1444]=32'h0; font_rom[1445]=32'h3ffffc0; font_rom[1446]=32'h3ffffc0; font_rom[1447]=32'h38001c0;
font_rom[1448]=32'h38001c0; font_rom[1449]=32'h38001c0; font_rom[1450]=32'h38001c0; font_rom[1451]=32'h38001c0; font_rom[1452]=32'h38001c0; font_rom[1453]=32'h3ffffc0; font_rom[1454]=32'h3ffffc0; font_rom[1455]=32'h38001c0;
font_rom[1456]=32'hffffff0; font_rom[1457]=32'hffffff0; font_rom[1458]=32'hc000070; font_rom[1459]=32'hc000030; font_rom[1460]=32'hc000030; font_rom[1461]=32'hcffff30; font_rom[1462]=32'hcffff30; font_rom[1463]=32'hc000030;
font_rom[1464]=32'hc000030; font_rom[1465]=32'hc000030; font_rom[1466]=32'hc3ffc30; font_rom[1467]=32'hc3ffc30; font_rom[1468]=32'hc300c30; font_rom[1469]=32'hc300c30; font_rom[1470]=32'hc300c30; font_rom[1471]=32'hc300c30;
font_rom[1472]=32'hc3ffc30; font_rom[1473]=32'hc3ffc30; font_rom[1474]=32'hc300030; font_rom[1475]=32'hc000030; font_rom[1476]=32'hc000070; font_rom[1477]=32'hc000070; font_rom[1478]=32'hc000fe0; font_rom[1479]=32'hc000fc0;
font_rom[1480]=32'hc0000; font_rom[1481]=32'h1c0000; font_rom[1482]=32'h3fffe0; font_rom[1483]=32'h7fffe0; font_rom[1484]=32'he001c0; font_rom[1485]=32'h1800380; font_rom[1486]=32'h3100700; font_rom[1487]=32'he381e00;
font_rom[1488]=32'h1c1c3800; font_rom[1489]=32'h80ef000; font_rom[1490]=32'h7c000; font_rom[1491]=32'hf0000; font_rom[1492]=32'h3c0000; font_rom[1493]=32'h1ffffe0; font_rom[1494]=32'h7ffffe0; font_rom[1495]=32'h1ec00060;
font_rom[1496]=32'h18c00060; font_rom[1497]=32'hc00060; font_rom[1498]=32'hc00060; font_rom[1499]=32'hc00060; font_rom[1500]=32'hc00060; font_rom[1501]=32'hc00060; font_rom[1502]=32'hffffe0; font_rom[1503]=32'hffffe0;
font_rom[1504]=32'hc00060; font_rom[1505]=32'h1e0; font_rom[1506]=32'h3fff0; font_rom[1507]=32'h3ffffc0; font_rom[1508]=32'h3ff8000; font_rom[1509]=32'h3000000; font_rom[1510]=32'h3000000; font_rom[1511]=32'h3000000;
font_rom[1512]=32'h3000000; font_rom[1513]=32'h3fffff8; font_rom[1514]=32'h3fffff8; font_rom[1515]=32'h3000000; font_rom[1516]=32'h3000000; font_rom[1517]=32'h3000000; font_rom[1518]=32'h7000000; font_rom[1519]=32'h7000000;
font_rom[1520]=32'h73fffe0; font_rom[1521]=32'h73fffe0; font_rom[1522]=32'h6300060; font_rom[1523]=32'h6300060; font_rom[1524]=32'h6300060; font_rom[1525]=32'h6300060; font_rom[1526]=32'he300060; font_rom[1527]=32'hc300060;
font_rom[1528]=32'h1c3fffe0; font_rom[1529]=32'h183fffe0; font_rom[1530]=32'h38300060; font_rom[1531]=32'h10300060; font_rom[1532]=32'h1c000; font_rom[1533]=32'h3c000; font_rom[1534]=32'h6e000; font_rom[1535]=32'h1c7000;
font_rom[1536]=32'h383800; font_rom[1537]=32'h731e00; font_rom[1538]=32'h1e38700; font_rom[1539]=32'h3c1c3c0; font_rom[1540]=32'hf0081f0; font_rom[1541]=32'h1c000078; font_rom[1542]=32'h19ffff18; font_rom[1543]=32'h1ffff00;
font_rom[1544]=32'he00; font_rom[1545]=32'h1c00; font_rom[1546]=32'h3800; font_rom[1547]=32'h1000; font_rom[1548]=32'h3ffffc0; font_rom[1549]=32'h3ffffc0; font_rom[1550]=32'h30000c0; font_rom[1551]=32'h30000c0;
font_rom[1552]=32'h30000c0; font_rom[1553]=32'h30000c0; font_rom[1554]=32'h30000c0; font_rom[1555]=32'h3ffffc0; font_rom[1556]=32'h3ffffc0; font_rom[1557]=32'h30000c0; font_rom[1558]=32'h3f0; font_rom[1559]=32'hff1fff0;
font_rom[1560]=32'hff1fc00; font_rom[1561]=32'hc318000; font_rom[1562]=32'hc318000; font_rom[1563]=32'hc318000; font_rom[1564]=32'hc318000; font_rom[1565]=32'hc318000; font_rom[1566]=32'hc318000; font_rom[1567]=32'hc31fff8;
font_rom[1568]=32'hc31fff8; font_rom[1569]=32'hc318180; font_rom[1570]=32'hc318180; font_rom[1571]=32'hc318180; font_rom[1572]=32'hc318180; font_rom[1573]=32'hc318180; font_rom[1574]=32'hc318180; font_rom[1575]=32'hff18180;
font_rom[1576]=32'hff38180; font_rom[1577]=32'hc030180; font_rom[1578]=32'hc070180; font_rom[1579]=32'h60180; font_rom[1580]=32'he0180; font_rom[1581]=32'h1c0180; font_rom[1582]=32'h80180; font_rom[1583]=32'h10000;
font_rom[1584]=32'h18000; font_rom[1585]=32'h1c000; font_rom[1586]=32'h1ffffff8; font_rom[1587]=32'h1ffffff8; font_rom[1588]=32'h0; font_rom[1589]=32'h0; font_rom[1590]=32'h3ffffc0; font_rom[1591]=32'h3ffffc0;
font_rom[1592]=32'h30000c0; font_rom[1593]=32'h30000c0; font_rom[1594]=32'h30000c0; font_rom[1595]=32'h30000c0; font_rom[1596]=32'h3ffffc0; font_rom[1597]=32'h3ffffc0; font_rom[1598]=32'h30fc0c0; font_rom[1599]=32'h1cc020;
font_rom[1600]=32'h3860e0; font_rom[1601]=32'h7063c0; font_rom[1602]=32'hf03780; font_rom[1603]=32'h3f01e00; font_rom[1604]=32'hf301c00; font_rom[1605]=32'h1c30ce00; font_rom[1606]=32'h1833c780; font_rom[1607]=32'h7f81f0;
font_rom[1608]=32'h7c00f8; font_rom[1609]=32'h200010; font_rom[1610]=32'h1800; font_rom[1611]=32'h3000; font_rom[1612]=32'h3000; font_rom[1613]=32'hfc7fff0; font_rom[1614]=32'hfc7fff0; font_rom[1615]=32'hc460030;
font_rom[1616]=32'hc460030; font_rom[1617]=32'hc460030; font_rom[1618]=32'hc460030; font_rom[1619]=32'hc467f30; font_rom[1620]=32'hc467f30; font_rom[1621]=32'hc464330; font_rom[1622]=32'hc464330; font_rom[1623]=32'hc464330;
font_rom[1624]=32'hc464330; font_rom[1625]=32'hc464330; font_rom[1626]=32'hc464330; font_rom[1627]=32'hc464330; font_rom[1628]=32'hc467f30; font_rom[1629]=32'hfc67f30; font_rom[1630]=32'hfc64030; font_rom[1631]=32'hc460030;
font_rom[1632]=32'hc460030; font_rom[1633]=32'h60070; font_rom[1634]=32'h607e0; font_rom[1635]=32'h607e0; font_rom[1636]=32'h1800; font_rom[1637]=32'h1800; font_rom[1638]=32'h1ffff8; font_rom[1639]=32'hfcffff0;
font_rom[1640]=32'hfc01800; font_rom[1641]=32'h8c01800; font_rom[1642]=32'h8c7ffe0; font_rom[1643]=32'h8c7ffe0; font_rom[1644]=32'h8c00000; font_rom[1645]=32'h8dffff8; font_rom[1646]=32'h8dffff8; font_rom[1647]=32'h8d80018;
font_rom[1648]=32'h8dbff98; font_rom[1649]=32'h8d9ff98; font_rom[1650]=32'h8c00000; font_rom[1651]=32'h8c7ffe0; font_rom[1652]=32'h8c7ffe0; font_rom[1653]=32'h8c60060; font_rom[1654]=32'h8c60060; font_rom[1655]=32'hfc7ffe0;
font_rom[1656]=32'hfc7ff80; font_rom[1657]=32'h8018380; font_rom[1658]=32'h800c300; font_rom[1659]=32'h3ffff8; font_rom[1660]=32'h1ffff8; font_rom[1661]=32'h3001c00; font_rom[1662]=32'h3001c00; font_rom[1663]=32'h3061c00;
font_rom[1664]=32'h3061c30; font_rom[1665]=32'h3061ff0; font_rom[1666]=32'h1fe61ff0; font_rom[1667]=32'h1fe7ff30; font_rom[1668]=32'h30ffc30; font_rom[1669]=32'h31f9c30; font_rom[1670]=32'h31e1c30; font_rom[1671]=32'h3061c30;
font_rom[1672]=32'h3061c30; font_rom[1673]=32'h3061c70; font_rom[1674]=32'h3061fe0; font_rom[1675]=32'h3061fe0; font_rom[1676]=32'h3061c08; font_rom[1677]=32'h3361c1c; font_rom[1678]=32'h3f61c1c; font_rom[1679]=32'hfc60018;
font_rom[1680]=32'h3e060018; font_rom[1681]=32'h10070038; font_rom[1682]=32'h3fff0; font_rom[1683]=32'h1ffe0; font_rom[1684]=32'h3000000; font_rom[1685]=32'h30fffe0; font_rom[1686]=32'h30fffe0; font_rom[1687]=32'h30003c0;
font_rom[1688]=32'h3000f00; font_rom[1689]=32'h3001c00; font_rom[1690]=32'h1ff07800; font_rom[1691]=32'h1ff1e000; font_rom[1692]=32'h3078000; font_rom[1693]=32'h307fff8; font_rom[1694]=32'h307fff0; font_rom[1695]=32'h300c630;
font_rom[1696]=32'h3018630; font_rom[1697]=32'h3018630; font_rom[1698]=32'h3338630; font_rom[1699]=32'h3f30c30; font_rom[1700]=32'h3e70c30; font_rom[1701]=32'h1f8e1830; font_rom[1702]=32'h1c1c1830; font_rom[1703]=32'h10383030;
font_rom[1704]=32'h707030; font_rom[1705]=32'h20e070; font_rom[1706]=32'h1cfe0; font_rom[1707]=32'h38fc0; font_rom[1708]=32'h30000; font_rom[1709]=32'h30300c0; font_rom[1710]=32'h30300c0; font_rom[1711]=32'h30300c0;
font_rom[1712]=32'h31ffff8; font_rom[1713]=32'h31ffff8; font_rom[1714]=32'h30300c0; font_rom[1715]=32'h30300c0; font_rom[1716]=32'h1fe3ffc0; font_rom[1717]=32'h1fe3ffc0; font_rom[1718]=32'h30300c0; font_rom[1719]=32'h30300c0;
font_rom[1720]=32'h303ffc0; font_rom[1721]=32'h303ffc0; font_rom[1722]=32'h30300c0; font_rom[1723]=32'h30300c0; font_rom[1724]=32'h33ffffc; font_rom[1725]=32'h33ffff8; font_rom[1726]=32'h30c1180; font_rom[1727]=32'h36c31c0;
font_rom[1728]=32'h3ec70e0; font_rom[1729]=32'hfcce060; font_rom[1730]=32'h3e0c4020; font_rom[1731]=32'h100c0000; font_rom[1732]=32'hffff8; font_rom[1733]=32'hffff8; font_rom[1734]=32'h1801800; font_rom[1735]=32'h1801800;
font_rom[1736]=32'h1801800; font_rom[1737]=32'h3001800; font_rom[1738]=32'h3fe1800; font_rom[1739]=32'h3fe1800; font_rom[1740]=32'h6061a00; font_rom[1741]=32'h6061f00; font_rom[1742]=32'h6061f80; font_rom[1743]=32'he0619c0;
font_rom[1744]=32'he0618e0; font_rom[1745]=32'h1f0c1870; font_rom[1746]=32'h1b0c1838; font_rom[1747]=32'h3b0c1810; font_rom[1748]=32'h11981800; font_rom[1749]=32'h1981800; font_rom[1750]=32'hf81800; font_rom[1751]=32'hf01800;
font_rom[1752]=32'h701800; font_rom[1753]=32'hf01800; font_rom[1754]=32'h1f81800; font_rom[1755]=32'h39e1800; font_rom[1756]=32'h707e000; font_rom[1757]=32'he03fff8; font_rom[1758]=32'h1c003ff8; font_rom[1759]=32'h8000000;
font_rom[1760]=32'h30000; font_rom[1761]=32'h60000; font_rom[1762]=32'he0000; font_rom[1763]=32'h1fffe0; font_rom[1764]=32'h7fffe0; font_rom[1765]=32'he001c0; font_rom[1766]=32'h1c00780; font_rom[1767]=32'h7900e00;
font_rom[1768]=32'he183c00; font_rom[1769]=32'h40cf000; font_rom[1770]=32'h7c000; font_rom[1771]=32'hf3000; font_rom[1772]=32'h7ce000; font_rom[1773]=32'h3f1fff0; font_rom[1774]=32'h1f83fff0; font_rom[1775]=32'hc0e0070;
font_rom[1776]=32'h3c00e0; font_rom[1777]=32'h7001c0; font_rom[1778]=32'h1e20780; font_rom[1779]=32'h7830e00; font_rom[1780]=32'h603bc00; font_rom[1781]=32'h1f000; font_rom[1782]=32'h3e000; font_rom[1783]=32'h1f8000;
font_rom[1784]=32'h1fc0000; font_rom[1785]=32'hfe00000; font_rom[1786]=32'h6000000; font_rom[1787]=32'h18000; font_rom[1788]=32'h18000; font_rom[1789]=32'h18000; font_rom[1790]=32'h18000; font_rom[1791]=32'h18000;
font_rom[1792]=32'h18000; font_rom[1793]=32'h1ffffff8; font_rom[1794]=32'h1ffffff8; font_rom[1795]=32'h38000; font_rom[1796]=32'h38000; font_rom[1797]=32'h3c000; font_rom[1798]=32'h3c000; font_rom[1799]=32'h3c000;
font_rom[1800]=32'h66000; font_rom[1801]=32'h66000; font_rom[1802]=32'he3000; font_rom[1803]=32'h1c3800; font_rom[1804]=32'h381c00; font_rom[1805]=32'h380e00; font_rom[1806]=32'hf00700; font_rom[1807]=32'h1e003c0;
font_rom[1808]=32'h3c001f0; font_rom[1809]=32'hf000078; font_rom[1810]=32'h1e000010; font_rom[1811]=32'h8000000; font_rom[1812]=32'hffffff0; font_rom[1813]=32'hffffff0; font_rom[1814]=32'h18000; font_rom[1815]=32'h18000;
font_rom[1816]=32'h18000; font_rom[1817]=32'h18000; font_rom[1818]=32'h18000; font_rom[1819]=32'h18000; font_rom[1820]=32'h18000; font_rom[1821]=32'h1ffffff8; font_rom[1822]=32'h1ffffff8; font_rom[1823]=32'h3c000;
font_rom[1824]=32'h3c000; font_rom[1825]=32'h7c000; font_rom[1826]=32'h66000; font_rom[1827]=32'he7000; font_rom[1828]=32'h1c3000; font_rom[1829]=32'h381800; font_rom[1830]=32'h781e00; font_rom[1831]=32'hf00f00;
font_rom[1832]=32'h3c003c0; font_rom[1833]=32'hf8001f0; font_rom[1834]=32'h1e000078; font_rom[1835]=32'h8000010; font_rom[1836]=32'h1c000; font_rom[1837]=32'hc1c000; font_rom[1838]=32'hc1c000; font_rom[1839]=32'h1c1c000;
font_rom[1840]=32'h181c000; font_rom[1841]=32'h3fffff0; font_rom[1842]=32'h3fffff0; font_rom[1843]=32'h701c000; font_rom[1844]=32'he018000; font_rom[1845]=32'hc018000; font_rom[1846]=32'h4018000; font_rom[1847]=32'h18000;
font_rom[1848]=32'h1ffffff8; font_rom[1849]=32'h1ffffff8; font_rom[1850]=32'h3c000; font_rom[1851]=32'h3c000; font_rom[1852]=32'h76000; font_rom[1853]=32'he7000; font_rom[1854]=32'hc3800; font_rom[1855]=32'h381c00;
font_rom[1856]=32'h700e00; font_rom[1857]=32'h1e00780; font_rom[1858]=32'hf8001f0; font_rom[1859]=32'h1f0000f8; font_rom[1860]=32'h8000010; font_rom[1861]=32'h3800000; font_rom[1862]=32'h383fff8; font_rom[1863]=32'h303fff8;
font_rom[1864]=32'h3000070; font_rom[1865]=32'h30000e0; font_rom[1866]=32'h3ff803c0; font_rom[1867]=32'h3ff80700; font_rom[1868]=32'h6180600; font_rom[1869]=32'h6180600; font_rom[1870]=32'h6180600; font_rom[1871]=32'h6180600;
font_rom[1872]=32'he37fffc; font_rom[1873]=32'hc37fffc; font_rom[1874]=32'hc300600; font_rom[1875]=32'hc700600; font_rom[1876]=32'h6600600; font_rom[1877]=32'h3e00600; font_rom[1878]=32'h1e00600; font_rom[1879]=32'h1e00600;
font_rom[1880]=32'h1f80600; font_rom[1881]=32'h31c0600; font_rom[1882]=32'h7080e00; font_rom[1883]=32'he000e00; font_rom[1884]=32'h1c01fc00; font_rom[1885]=32'h1001f800; font_rom[1886]=32'h2001000; font_rom[1887]=32'h3003800;
font_rom[1888]=32'h7001800; font_rom[1889]=32'h6001800; font_rom[1890]=32'h60ffff8; font_rom[1891]=32'h60ffff8; font_rom[1892]=32'h60c0018; font_rom[1893]=32'h1fec0018; font_rom[1894]=32'h1fef0000; font_rom[1895]=32'h463f7f0;
font_rom[1896]=32'hc43f7f0; font_rom[1897]=32'hcc63630; font_rom[1898]=32'hcc63630; font_rom[1899]=32'hcc43630; font_rom[1900]=32'hccc6630; font_rom[1901]=32'h18cc6630; font_rom[1902]=32'h18db6630; font_rom[1903]=32'h1dbbe630;
font_rom[1904]=32'hf90c7f0; font_rom[1905]=32'h380c6e0; font_rom[1906]=32'h3c1c608; font_rom[1907]=32'h3e18618; font_rom[1908]=32'h6730618; font_rom[1909]=32'he270618; font_rom[1910]=32'h1c0e0618; font_rom[1911]=32'h181c07f8;
font_rom[1912]=32'h100803e0; font_rom[1913]=32'h10000; font_rom[1914]=32'h38000; font_rom[1915]=32'h1c000; font_rom[1916]=32'hc000; font_rom[1917]=32'hffffff8; font_rom[1918]=32'hffffff8; font_rom[1919]=32'hc000038;
font_rom[1920]=32'hc000038; font_rom[1921]=32'hc000038; font_rom[1922]=32'hcffff38; font_rom[1923]=32'hffff00; font_rom[1924]=32'hf00; font_rom[1925]=32'h3c00; font_rom[1926]=32'hf000; font_rom[1927]=32'h1c000;
font_rom[1928]=32'h1c000; font_rom[1929]=32'h1ffffffc; font_rom[1930]=32'h1ffffffc; font_rom[1931]=32'h1c000; font_rom[1932]=32'h1c000; font_rom[1933]=32'h1c000; font_rom[1934]=32'h1c000; font_rom[1935]=32'h1c000;
font_rom[1936]=32'h1c000; font_rom[1937]=32'h3f8000; font_rom[1938]=32'h3f0000; font_rom[1939]=32'h7e0; font_rom[1940]=32'h1ff7ffe0; font_rom[1941]=32'h1ff7ff80; font_rom[1942]=32'h663180; font_rom[1943]=32'he63180;
font_rom[1944]=32'hc63180; font_rom[1945]=32'h1863180; font_rom[1946]=32'h1863180; font_rom[1947]=32'h3063180; font_rom[1948]=32'h3063180; font_rom[1949]=32'h3163180; font_rom[1950]=32'h3f63180; font_rom[1951]=32'h3e63180;
font_rom[1952]=32'h1f8631c0; font_rom[1953]=32'h1f0630c0; font_rom[1954]=32'h130630c0; font_rom[1955]=32'h30630c0; font_rom[1956]=32'h30c36e0; font_rom[1957]=32'h30c3660; font_rom[1958]=32'h30c3660; font_rom[1959]=32'h31c3670;
font_rom[1960]=32'h3183f30; font_rom[1961]=32'h3387f38; font_rom[1962]=32'h1f707b18; font_rom[1963]=32'h1e202108; font_rom[1964]=32'h20080; font_rom[1965]=32'h10701c0; font_rom[1966]=32'h3838180; font_rom[1967]=32'h1c1c380;
font_rom[1968]=32'hc08700; font_rom[1969]=32'h800200; font_rom[1970]=32'hffffff8; font_rom[1971]=32'hffffff8; font_rom[1972]=32'hc000038; font_rom[1973]=32'hc000038; font_rom[1974]=32'hcffff38; font_rom[1975]=32'hffff00;
font_rom[1976]=32'he00; font_rom[1977]=32'h3c00; font_rom[1978]=32'hf000; font_rom[1979]=32'h1c000; font_rom[1980]=32'h1ffffff8; font_rom[1981]=32'h1ffffff8; font_rom[1982]=32'h1c000; font_rom[1983]=32'h1c000;
font_rom[1984]=32'h1c000; font_rom[1985]=32'h1c000; font_rom[1986]=32'h1c000; font_rom[1987]=32'h3f8000; font_rom[1988]=32'h3f8000; font_rom[1989]=32'h18000; font_rom[1990]=32'h1c000; font_rom[1991]=32'hc000;
font_rom[1992]=32'h1ffffff8; font_rom[1993]=32'h1ffffff8; font_rom[1994]=32'h1c000038; font_rom[1995]=32'h1c407038; font_rom[1996]=32'h1ce07038; font_rom[1997]=32'h387000; font_rom[1998]=32'h1c7000; font_rom[1999]=32'h30e7000;
font_rom[2000]=32'h3c46000; font_rom[2001]=32'he06000; font_rom[2002]=32'h706000; font_rom[2003]=32'h106000; font_rom[2004]=32'h6000; font_rom[2005]=32'h1ffffff8; font_rom[2006]=32'h1ffffff8; font_rom[2007]=32'h1c000;
font_rom[2008]=32'h3e000; font_rom[2009]=32'h7f800; font_rom[2010]=32'h1e3e00; font_rom[2011]=32'h7c0780; font_rom[2012]=32'h7f001f0; font_rom[2013]=32'h1f800078; font_rom[2014]=32'h1c000010; font_rom[2015]=32'h10000;
font_rom[2016]=32'h38000; font_rom[2017]=32'h18000; font_rom[2018]=32'h1ffffff8; font_rom[2019]=32'h1ffffff8; font_rom[2020]=32'h1c018038; font_rom[2021]=32'h1c818138; font_rom[2022]=32'h1dc183b8; font_rom[2023]=32'he18700;
font_rom[2024]=32'h718e00; font_rom[2025]=32'h218400; font_rom[2026]=32'h3ffffc0; font_rom[2027]=32'h3ffffc0; font_rom[2028]=32'h30000c0; font_rom[2029]=32'h30000c0; font_rom[2030]=32'h3ffffc0; font_rom[2031]=32'h3ffffc0;
font_rom[2032]=32'h30000c0; font_rom[2033]=32'h30000c0; font_rom[2034]=32'h3ffffc0; font_rom[2035]=32'h3ffffc0; font_rom[2036]=32'h30000c0; font_rom[2037]=32'h30000c0; font_rom[2038]=32'h30000c0; font_rom[2039]=32'h30000c0;
font_rom[2040]=32'h3003fc0; font_rom[2041]=32'h3001f80; font_rom[2042]=32'h10000; font_rom[2043]=32'h38000; font_rom[2044]=32'h1c000; font_rom[2045]=32'h1ffffff8; font_rom[2046]=32'h1ffffff8; font_rom[2047]=32'h1c000038;
font_rom[2048]=32'h1c000038; font_rom[2049]=32'h3ffffc0; font_rom[2050]=32'h3ffffc0; font_rom[2051]=32'h60000; font_rom[2052]=32'h1e0040; font_rom[2053]=32'h3e01c0; font_rom[2054]=32'hf30380; font_rom[2055]=32'h7c78e00;
font_rom[2056]=32'h1f0ffc00; font_rom[2057]=32'h81cfc00; font_rom[2058]=32'h78e600; font_rom[2059]=32'h1e1e700; font_rom[2060]=32'h7836380; font_rom[2061]=32'h1f0e61c0; font_rom[2062]=32'h81c61e0; font_rom[2063]=32'h706078;
font_rom[2064]=32'h1e06038; font_rom[2065]=32'h780e010; font_rom[2066]=32'h1e3fc000; font_rom[2067]=32'h81f8000; font_rom[2068]=32'h10000; font_rom[2069]=32'h18000; font_rom[2070]=32'h1c000; font_rom[2071]=32'h8000;
font_rom[2072]=32'hffffff0; font_rom[2073]=32'hffffff0; font_rom[2074]=32'hc000030; font_rom[2075]=32'hcc00030; font_rom[2076]=32'hcc00000; font_rom[2077]=32'hc0ffe0; font_rom[2078]=32'hfeffe0; font_rom[2079]=32'hfe6060;
font_rom[2080]=32'hc06060; font_rom[2081]=32'hc03060; font_rom[2082]=32'hc030e0; font_rom[2083]=32'h1fffb0c0; font_rom[2084]=32'h1fff38c0; font_rom[2085]=32'h1980; font_rom[2086]=32'h601d80; font_rom[2087]=32'h6660f00;
font_rom[2088]=32'h6670f00; font_rom[2089]=32'he638f00; font_rom[2090]=32'hc611f80; font_rom[2091]=32'h186079c0; font_rom[2092]=32'h860e0f0; font_rom[2093]=32'h3e3c078; font_rom[2094]=32'h3c10010; font_rom[2095]=32'h10000;
font_rom[2096]=32'h38000; font_rom[2097]=32'h1c000; font_rom[2098]=32'hffffff8; font_rom[2099]=32'hffffff8; font_rom[2100]=32'hc181818; font_rom[2101]=32'h181800; font_rom[2102]=32'h7ffffe0; font_rom[2103]=32'h7ffffe0;
font_rom[2104]=32'h181800; font_rom[2105]=32'h181800; font_rom[2106]=32'h7ffffe0; font_rom[2107]=32'h7ffffe0; font_rom[2108]=32'h181800; font_rom[2109]=32'h181800; font_rom[2110]=32'h1ffffff8; font_rom[2111]=32'h1ffffff8;
font_rom[2112]=32'h300c00; font_rom[2113]=32'he00600; font_rom[2114]=32'h1ce0380; font_rom[2115]=32'h70fe1e0; font_rom[2116]=32'h1e00f878; font_rom[2117]=32'h18300018; font_rom[2118]=32'h7f0000; font_rom[2119]=32'h7f000;
font_rom[2120]=32'hfe00; font_rom[2121]=32'hc00; font_rom[2122]=32'hc0; font_rom[2123]=32'hc0; font_rom[2124]=32'hc0; font_rom[2125]=32'h1ffc00c0; font_rom[2126]=32'h1ffc00c0; font_rom[2127]=32'hc00c0;
font_rom[2128]=32'h1dfffc; font_rom[2129]=32'h819fffc; font_rom[2130]=32'hc1800c0; font_rom[2131]=32'he1800c0; font_rom[2132]=32'h718c0c0; font_rom[2133]=32'h3b0c0c0; font_rom[2134]=32'h1f0e0c0; font_rom[2135]=32'h1f070c0;
font_rom[2136]=32'he030c0; font_rom[2137]=32'he038c0; font_rom[2138]=32'hf010c0; font_rom[2139]=32'h1f800c0; font_rom[2140]=32'h39c00c0; font_rom[2141]=32'h70c00c0; font_rom[2142]=32'he0e00c0; font_rom[2143]=32'h1c0400c0;
font_rom[2144]=32'h18003fc0; font_rom[2145]=32'h10003f80; font_rom[2146]=32'h1806000; font_rom[2147]=32'h180e000; font_rom[2148]=32'h181c000; font_rom[2149]=32'h183fff0; font_rom[2150]=32'h1187fff0; font_rom[2151]=32'h198e0070;
font_rom[2152]=32'h1d9c80e0; font_rom[2153]=32'hd91c3c0; font_rom[2154]=32'hf80ef80; font_rom[2155]=32'h5807e00; font_rom[2156]=32'h180f800; font_rom[2157]=32'h187e1c0; font_rom[2158]=32'h19f81c0; font_rom[2159]=32'h19801c0;
font_rom[2160]=32'h39ffffc; font_rom[2161]=32'h79ffffc; font_rom[2162]=32'hf8101c0; font_rom[2163]=32'hd8301c0; font_rom[2164]=32'h198181c0; font_rom[2165]=32'h3981c1c0; font_rom[2166]=32'h1180e1c0; font_rom[2167]=32'h18041c0;
font_rom[2168]=32'h18001c0; font_rom[2169]=32'h18001c0; font_rom[2170]=32'h1803f80; font_rom[2171]=32'h1803f00; font_rom[2172]=32'h1800000; font_rom[2173]=32'h18000; font_rom[2174]=32'h6018060; font_rom[2175]=32'h6018060;
font_rom[2176]=32'h6018060; font_rom[2177]=32'h6018060; font_rom[2178]=32'h7ffffe0; font_rom[2179]=32'h7ffffe0; font_rom[2180]=32'h60; font_rom[2181]=32'h1c0000; font_rom[2182]=32'h180000; font_rom[2183]=32'h3fffe0;
font_rom[2184]=32'h7fffe0; font_rom[2185]=32'hc000c0; font_rom[2186]=32'h1800180; font_rom[2187]=32'h7000380; font_rom[2188]=32'he100700; font_rom[2189]=32'h1c380e00; font_rom[2190]=32'h80e3c00; font_rom[2191]=32'h77000;
font_rom[2192]=32'h3e000; font_rom[2193]=32'h7c000; font_rom[2194]=32'h3f0000; font_rom[2195]=32'h3f80000; font_rom[2196]=32'h1fe00000; font_rom[2197]=32'he000000; font_rom[2198]=32'h1c000; font_rom[2199]=32'h301c060;
font_rom[2200]=32'h301c060; font_rom[2201]=32'h301c060; font_rom[2202]=32'h301c060; font_rom[2203]=32'h3ffffe0; font_rom[2204]=32'h3ffffe0; font_rom[2205]=32'h60; font_rom[2206]=32'h0; font_rom[2207]=32'h7fffff8;
font_rom[2208]=32'h7fffff8; font_rom[2209]=32'h7000000; font_rom[2210]=32'h7000000; font_rom[2211]=32'h7000000; font_rom[2212]=32'h73fffe0; font_rom[2213]=32'h73fffe0; font_rom[2214]=32'h7006000; font_rom[2215]=32'h7006000;
font_rom[2216]=32'h6006000; font_rom[2217]=32'h6fffff8; font_rom[2218]=32'h6fffff8; font_rom[2219]=32'h6006000; font_rom[2220]=32'hc006000; font_rom[2221]=32'hc006000; font_rom[2222]=32'h1c006000; font_rom[2223]=32'h18006000;
font_rom[2224]=32'h10006000; font_rom[2225]=32'h180c020; font_rom[2226]=32'h180c070; font_rom[2227]=32'h180c0e0; font_rom[2228]=32'h180c1c0; font_rom[2229]=32'hff8c380; font_rom[2230]=32'hff8c700; font_rom[2231]=32'hd98ce00;
font_rom[2232]=32'hd98dc00; font_rom[2233]=32'hd98c800; font_rom[2234]=32'hd98c000; font_rom[2235]=32'hd9ffff8; font_rom[2236]=32'hd9ffff8; font_rom[2237]=32'hd98c600; font_rom[2238]=32'hd98c600; font_rom[2239]=32'hd98c600;
font_rom[2240]=32'hd98c300; font_rom[2241]=32'hd98c380; font_rom[2242]=32'hd98c180; font_rom[2243]=32'hdf8c1c0; font_rom[2244]=32'hdb0c4e0; font_rom[2245]=32'h180dc70; font_rom[2246]=32'h180fc38; font_rom[2247]=32'h181f018;
font_rom[2248]=32'h181e000; font_rom[2249]=32'h8000; font_rom[2250]=32'hc00000; font_rom[2251]=32'hc00000; font_rom[2252]=32'h1c00000; font_rom[2253]=32'h1fffff0; font_rom[2254]=32'h3fffff0; font_rom[2255]=32'h7006000;
font_rom[2256]=32'h6006000; font_rom[2257]=32'he006000; font_rom[2258]=32'h1c006000; font_rom[2259]=32'h9fffff0; font_rom[2260]=32'h1fffff0; font_rom[2261]=32'h1806000; font_rom[2262]=32'h1806000; font_rom[2263]=32'h1806000;
font_rom[2264]=32'h1806000; font_rom[2265]=32'h1806000; font_rom[2266]=32'h1ffffffc; font_rom[2267]=32'h1ffffffc; font_rom[2268]=32'h6000; font_rom[2269]=32'h6000; font_rom[2270]=32'h6000; font_rom[2271]=32'h6000;
font_rom[2272]=32'h6000; font_rom[2273]=32'h6000; font_rom[2274]=32'h6000; font_rom[2275]=32'h6000; font_rom[2276]=32'hffffff0; font_rom[2277]=32'hffffff0; font_rom[2278]=32'h300e00; font_rom[2279]=32'h300e00;
font_rom[2280]=32'h300e00; font_rom[2281]=32'h300e00; font_rom[2282]=32'h300e00; font_rom[2283]=32'h300e00; font_rom[2284]=32'h300e00; font_rom[2285]=32'h300e00; font_rom[2286]=32'h1ffffff8; font_rom[2287]=32'h1ffffff8;
font_rom[2288]=32'h300e00; font_rom[2289]=32'h300e00; font_rom[2290]=32'h300e00; font_rom[2291]=32'h700e00; font_rom[2292]=32'h600e00; font_rom[2293]=32'h600e00; font_rom[2294]=32'he00e00; font_rom[2295]=32'hc00e00;
font_rom[2296]=32'h1c00e00; font_rom[2297]=32'h3800e00; font_rom[2298]=32'hf000e00; font_rom[2299]=32'h1e000e00; font_rom[2300]=32'h8000e00; font_rom[2301]=32'h1fe3ffe0; font_rom[2302]=32'h1fe3ffe0; font_rom[2303]=32'h630060;
font_rom[2304]=32'h630060; font_rom[2305]=32'h630060; font_rom[2306]=32'h63ffe0; font_rom[2307]=32'hfe3ffe0; font_rom[2308]=32'hfe31860; font_rom[2309]=32'hc601800; font_rom[2310]=32'h18001800; font_rom[2311]=32'h1807fff0;
font_rom[2312]=32'h1807fff0; font_rom[2313]=32'h1fe61830; font_rom[2314]=32'h1fe61830; font_rom[2315]=32'h661830; font_rom[2316]=32'h67fff0; font_rom[2317]=32'h67fff0; font_rom[2318]=32'h661800; font_rom[2319]=32'h601860;
font_rom[2320]=32'he01870; font_rom[2321]=32'hc01830; font_rom[2322]=32'hc1fff8; font_rom[2323]=32'hfcffff8; font_rom[2324]=32'hf8f801c; font_rom[2325]=32'h8; font_rom[2326]=32'h20; font_rom[2327]=32'h70;
font_rom[2328]=32'hfffe0e0; font_rom[2329]=32'hfffc1c0; font_rom[2330]=32'h3860380; font_rom[2331]=32'h3860700; font_rom[2332]=32'h3861e00; font_rom[2333]=32'h3863c00; font_rom[2334]=32'h3861010; font_rom[2335]=32'h3860038;
font_rom[2336]=32'h3860070; font_rom[2337]=32'h38600e0; font_rom[2338]=32'h1fffe3c0; font_rom[2339]=32'h1fffe700; font_rom[2340]=32'h3860e00; font_rom[2341]=32'h3063c00; font_rom[2342]=32'h3061008; font_rom[2343]=32'h3060018;
font_rom[2344]=32'h3060038; font_rom[2345]=32'h3060070; font_rom[2346]=32'h30600e0; font_rom[2347]=32'h60603c0; font_rom[2348]=32'h6060780; font_rom[2349]=32'he060e00; font_rom[2350]=32'h1c063c00; font_rom[2351]=32'h18067800;
font_rom[2352]=32'h10062000; font_rom[2353]=32'h60; font_rom[2354]=32'hfffc060; font_rom[2355]=32'hfffc0c0; font_rom[2356]=32'hc00c1c0; font_rom[2357]=32'hfffc700; font_rom[2358]=32'hc01ce00; font_rom[2359]=32'hc00dc10;
font_rom[2360]=32'hfffd838; font_rom[2361]=32'hfffc070; font_rom[2362]=32'h7000e0; font_rom[2363]=32'h1fffe1c0; font_rom[2364]=32'h1fffe380; font_rom[2365]=32'h700; font_rom[2366]=32'hfff8e00; font_rom[2367]=32'he019c08;
font_rom[2368]=32'hc018818; font_rom[2369]=32'hc018038; font_rom[2370]=32'hfff8070; font_rom[2371]=32'he3180e0; font_rom[2372]=32'h23201c0; font_rom[2373]=32'h7338380; font_rom[2374]=32'he31c700; font_rom[2375]=32'h1c309e00;
font_rom[2376]=32'h13f03800; font_rom[2377]=32'h1c01000; font_rom[2378]=32'h403000; font_rom[2379]=32'he03800; font_rom[2380]=32'hc01800; font_rom[2381]=32'h1c01800; font_rom[2382]=32'h3800000; font_rom[2383]=32'h70ffff8;
font_rom[2384]=32'he0ffff8; font_rom[2385]=32'h1c201800; font_rom[2386]=32'h18701800; font_rom[2387]=32'h10e01800; font_rom[2388]=32'h1c01800; font_rom[2389]=32'h1c01800; font_rom[2390]=32'h3801800; font_rom[2391]=32'h787fff0;
font_rom[2392]=32'hf87fff0; font_rom[2393]=32'h1f801800; font_rom[2394]=32'h19801800; font_rom[2395]=32'h11801800; font_rom[2396]=32'h1801800; font_rom[2397]=32'h1801800; font_rom[2398]=32'h1801800; font_rom[2399]=32'h1801800;
font_rom[2400]=32'h19ffffc; font_rom[2401]=32'h19ffffc; font_rom[2402]=32'h1800000; font_rom[2403]=32'h10000; font_rom[2404]=32'h38000; font_rom[2405]=32'h18000; font_rom[2406]=32'h1ffffff8; font_rom[2407]=32'h1ffffff8;
font_rom[2408]=32'h1c3000; font_rom[2409]=32'h1c3000; font_rom[2410]=32'h19c3100; font_rom[2411]=32'h19c3380; font_rom[2412]=32'h39c31c0; font_rom[2413]=32'h71c30e0; font_rom[2414]=32'he1c3070; font_rom[2415]=32'h1c1c3038;
font_rom[2416]=32'h81c3010; font_rom[2417]=32'h1c3000; font_rom[2418]=32'h10000; font_rom[2419]=32'h38000; font_rom[2420]=32'h6618080; font_rom[2421]=32'h671c0c0; font_rom[2422]=32'he70e0e0; font_rom[2423]=32'hc704670;
font_rom[2424]=32'h1c700630; font_rom[2425]=32'h18700638; font_rom[2426]=32'h18700618; font_rom[2427]=32'h18300e00; font_rom[2428]=32'h3ffc00; font_rom[2429]=32'h1ff800; font_rom[2430]=32'hc3000; font_rom[2431]=32'hc3000;
font_rom[2432]=32'hc3000; font_rom[2433]=32'h1ffc3ff8; font_rom[2434]=32'h1ffc3ff8; font_rom[2435]=32'hc3000; font_rom[2436]=32'hc3000; font_rom[2437]=32'hffc3ff0; font_rom[2438]=32'hffc3ff0; font_rom[2439]=32'hc3000;
font_rom[2440]=32'hc3000; font_rom[2441]=32'h1ffc3ff8; font_rom[2442]=32'h1ffc3ff8; font_rom[2443]=32'hc3000; font_rom[2444]=32'hc3000; font_rom[2445]=32'h10000; font_rom[2446]=32'h6738040; font_rom[2447]=32'h671c0e0;
font_rom[2448]=32'he70e070; font_rom[2449]=32'hc704230; font_rom[2450]=32'hc700738; font_rom[2451]=32'h1c700618; font_rom[2452]=32'h18300618; font_rom[2453]=32'h18300e00; font_rom[2454]=32'h3ffc00; font_rom[2455]=32'h1ff800;
font_rom[2456]=32'h3001800; font_rom[2457]=32'h31ffff8; font_rom[2458]=32'h31ffff8; font_rom[2459]=32'h3801800; font_rom[2460]=32'h3cffff0; font_rom[2461]=32'h36ffff0; font_rom[2462]=32'h1b201800; font_rom[2463]=32'h1b001800;
font_rom[2464]=32'h1b3ffffc; font_rom[2465]=32'h1b1ffff8; font_rom[2466]=32'h1b000000; font_rom[2467]=32'h130ffff0; font_rom[2468]=32'h130ffff0; font_rom[2469]=32'h30c0070; font_rom[2470]=32'h30c0070; font_rom[2471]=32'h30ffff0;
font_rom[2472]=32'h30ffff0; font_rom[2473]=32'h30c0070; font_rom[2474]=32'h30c0070; font_rom[2475]=32'h30ffff0; font_rom[2476]=32'h30ffff0; font_rom[2477]=32'h30c0070; font_rom[2478]=32'h30c0060; font_rom[2479]=32'h30c0fe0;
font_rom[2480]=32'h30c0fc0; font_rom[2481]=32'h1800000; font_rom[2482]=32'h181ffe0; font_rom[2483]=32'h181ffe0; font_rom[2484]=32'h1818060; font_rom[2485]=32'h1ff9ffe0; font_rom[2486]=32'h1ff9ffe0; font_rom[2487]=32'h1818060;
font_rom[2488]=32'h3818060; font_rom[2489]=32'h3f18060; font_rom[2490]=32'h7f9ffe0; font_rom[2491]=32'h799ffe0; font_rom[2492]=32'hd818060; font_rom[2493]=32'hd818060; font_rom[2494]=32'h19818060; font_rom[2495]=32'h1181ffe0;
font_rom[2496]=32'h1181ffe0; font_rom[2497]=32'h1818060; font_rom[2498]=32'h20000; font_rom[2499]=32'h6330040; font_rom[2500]=32'h6318260; font_rom[2501]=32'he30c770; font_rom[2502]=32'hc304638; font_rom[2503]=32'h1c300618;
font_rom[2504]=32'h18300e10; font_rom[2505]=32'h83ffc00; font_rom[2506]=32'h1ff800; font_rom[2507]=32'hc0c00; font_rom[2508]=32'h1fc0c00; font_rom[2509]=32'h1ffc0c00; font_rom[2510]=32'hfc00c00; font_rom[2511]=32'hc10c30;
font_rom[2512]=32'hc38c30; font_rom[2513]=32'hc18c60; font_rom[2514]=32'h1ffcccc0; font_rom[2515]=32'h1ffc9e80; font_rom[2516]=32'h1c01e00; font_rom[2517]=32'h1f03300; font_rom[2518]=32'h3f83300; font_rom[2519]=32'h6dc6180;
font_rom[2520]=32'hcc4c0c0; font_rom[2521]=32'h18c38070; font_rom[2522]=32'h10c70038; font_rom[2523]=32'hc20010; font_rom[2524]=32'h10040; font_rom[2525]=32'h67180c0; font_rom[2526]=32'h670c260; font_rom[2527]=32'he70e770;
font_rom[2528]=32'hc704638; font_rom[2529]=32'h1c700618; font_rom[2530]=32'h18300e10; font_rom[2531]=32'h83ffc00; font_rom[2532]=32'h1ff800; font_rom[2533]=32'h10000; font_rom[2534]=32'h38000; font_rom[2535]=32'h18000;
font_rom[2536]=32'hffffff0; font_rom[2537]=32'h7ffffe0; font_rom[2538]=32'h600600; font_rom[2539]=32'h301c00; font_rom[2540]=32'h1ffffff8; font_rom[2541]=32'h1ffffff8; font_rom[2542]=32'h0; font_rom[2543]=32'h3ffffc0;
font_rom[2544]=32'h3ffffc0; font_rom[2545]=32'h30000c0; font_rom[2546]=32'h3ffffc0; font_rom[2547]=32'h3ffffc0; font_rom[2548]=32'h30000c0; font_rom[2549]=32'h3ffffc0; font_rom[2550]=32'h3ffffc0; font_rom[2551]=32'h30000;
font_rom[2552]=32'h4618020; font_rom[2553]=32'hc61c670; font_rom[2554]=32'hc608638; font_rom[2555]=32'h18700618; font_rom[2556]=32'h183ffe08; font_rom[2557]=32'h1ff800; font_rom[2558]=32'h1980; font_rom[2559]=32'h19c0;
font_rom[2560]=32'h1840; font_rom[2561]=32'h7fffff8; font_rom[2562]=32'h7fffff8; font_rom[2563]=32'h6001800; font_rom[2564]=32'h6ffd860; font_rom[2565]=32'h67fcc60; font_rom[2566]=32'h6000cc0; font_rom[2567]=32'h67fcdc0;
font_rom[2568]=32'h67fc780; font_rom[2569]=32'h660c708; font_rom[2570]=32'h660c718; font_rom[2571]=32'he7fdf18; font_rom[2572]=32'hc7ff998; font_rom[2573]=32'h1c0031f8; font_rom[2574]=32'h180100f0; font_rom[2575]=32'h6738000;
font_rom[2576]=32'h671c440; font_rom[2577]=32'he708660; font_rom[2578]=32'hc300630; font_rom[2579]=32'h1c300e38; font_rom[2580]=32'h83ffc10; font_rom[2581]=32'h1ff800; font_rom[2582]=32'h6000; font_rom[2583]=32'he6300;
font_rom[2584]=32'h3fe6380; font_rom[2585]=32'h1ffc61c0; font_rom[2586]=32'h1ec060e0; font_rom[2587]=32'hc07070; font_rom[2588]=32'hc07020; font_rom[2589]=32'hc07000; font_rom[2590]=32'h3ffffffc; font_rom[2591]=32'h3ffffffc;
font_rom[2592]=32'he03000; font_rom[2593]=32'hc03020; font_rom[2594]=32'hc03070; font_rom[2595]=32'hc730e0; font_rom[2596]=32'hff31c0; font_rom[2597]=32'h1ffc3b80; font_rom[2598]=32'h1fe01f00; font_rom[2599]=32'h18c01e18;
font_rom[2600]=32'hc01c1c; font_rom[2601]=32'hc07c18; font_rom[2602]=32'hc0fe18; font_rom[2603]=32'hc3c618; font_rom[2604]=32'hcf07b8; font_rom[2605]=32'h1fc403f0; font_rom[2606]=32'hf8000f0; font_rom[2607]=32'hfe0;
font_rom[2608]=32'hfffffe0; font_rom[2609]=32'hffffe00; font_rom[2610]=32'h18000; font_rom[2611]=32'h18000; font_rom[2612]=32'h18000; font_rom[2613]=32'h18000; font_rom[2614]=32'h7ffffe0; font_rom[2615]=32'h7ffffe0;
font_rom[2616]=32'h18000; font_rom[2617]=32'h18000; font_rom[2618]=32'h18000; font_rom[2619]=32'h18000; font_rom[2620]=32'h18000; font_rom[2621]=32'h1ffffffc; font_rom[2622]=32'h1ffffffc; font_rom[2623]=32'h18000;
font_rom[2624]=32'h18000; font_rom[2625]=32'h18000; font_rom[2626]=32'h18000; font_rom[2627]=32'h18000; font_rom[2628]=32'h38000; font_rom[2629]=32'h7f8000; font_rom[2630]=32'h7f0000; font_rom[2631]=32'h1800000;
font_rom[2632]=32'h1800000; font_rom[2633]=32'h183fff0; font_rom[2634]=32'h183fff0; font_rom[2635]=32'h1830030; font_rom[2636]=32'h1ffb0030; font_rom[2637]=32'h1ffb0030; font_rom[2638]=32'h1830030; font_rom[2639]=32'h1830030;
font_rom[2640]=32'h1830030; font_rom[2641]=32'h1830030; font_rom[2642]=32'h1fb0030; font_rom[2643]=32'h1ffb0030; font_rom[2644]=32'h1fc30030; font_rom[2645]=32'h1f830030; font_rom[2646]=32'h1830030; font_rom[2647]=32'h1830030;
font_rom[2648]=32'h1830030; font_rom[2649]=32'h1830030; font_rom[2650]=32'h1830030; font_rom[2651]=32'h183fff0; font_rom[2652]=32'h383fff0; font_rom[2653]=32'h1f830030; font_rom[2654]=32'h1f030030; font_rom[2655]=32'h300c000;
font_rom[2656]=32'h300c000; font_rom[2657]=32'h300c000; font_rom[2658]=32'h300c000; font_rom[2659]=32'h300c000; font_rom[2660]=32'h1fe7ff80; font_rom[2661]=32'h1fe7ff80; font_rom[2662]=32'h300c180; font_rom[2663]=32'h300c180;
font_rom[2664]=32'h300c180; font_rom[2665]=32'h300c180; font_rom[2666]=32'h305c180; font_rom[2667]=32'h3efc180; font_rom[2668]=32'hfe78180; font_rom[2669]=32'h3f81e180; font_rom[2670]=32'h1b01f180; font_rom[2671]=32'h303b980;
font_rom[2672]=32'h3031988; font_rom[2673]=32'h30701cc; font_rom[2674]=32'h30700dc; font_rom[2675]=32'h30e00d8; font_rom[2676]=32'h31c00f8; font_rom[2677]=32'h33c00f8; font_rom[2678]=32'h1f380070; font_rom[2679]=32'h1e100000;
font_rom[2680]=32'h3018180; font_rom[2681]=32'h3018180; font_rom[2682]=32'h3018180; font_rom[2683]=32'h31ffffc; font_rom[2684]=32'h31ffffc; font_rom[2685]=32'h1fe18180; font_rom[2686]=32'h1fe18180; font_rom[2687]=32'h3018180;
font_rom[2688]=32'h3000000; font_rom[2689]=32'h30ffff0; font_rom[2690]=32'h30ffff0; font_rom[2691]=32'h36c1830; font_rom[2692]=32'h3ec1830; font_rom[2693]=32'h1f8c1830; font_rom[2694]=32'h3f0c1830; font_rom[2695]=32'h130c1830;
font_rom[2696]=32'h30ffff0; font_rom[2697]=32'h30ffff0; font_rom[2698]=32'h30c1830; font_rom[2699]=32'h30c1830; font_rom[2700]=32'h30c1830; font_rom[2701]=32'h30c1830; font_rom[2702]=32'h30c1830; font_rom[2703]=32'h30ffff0;
font_rom[2704]=32'h3f0ffff0; font_rom[2705]=32'h1e0c0030; font_rom[2706]=32'h38000; font_rom[2707]=32'h18000; font_rom[2708]=32'h1c000; font_rom[2709]=32'h8000; font_rom[2710]=32'h1ffffff8; font_rom[2711]=32'h1ffffff8;
font_rom[2712]=32'hc00300; font_rom[2713]=32'he00700; font_rom[2714]=32'h600600; font_rom[2715]=32'h600600; font_rom[2716]=32'h700e00; font_rom[2717]=32'h300c00; font_rom[2718]=32'h300c00; font_rom[2719]=32'h181800;
font_rom[2720]=32'h1c3800; font_rom[2721]=32'hc3000; font_rom[2722]=32'h67000; font_rom[2723]=32'h7e000; font_rom[2724]=32'h3c000; font_rom[2725]=32'h7e000; font_rom[2726]=32'hff800; font_rom[2727]=32'h3c3e00;
font_rom[2728]=32'h1f00f80; font_rom[2729]=32'hfc007f0; font_rom[2730]=32'h1f0000f8; font_rom[2731]=32'hc000030; font_rom[2732]=32'h1006000; font_rom[2733]=32'h1806000; font_rom[2734]=32'h1c0e000; font_rom[2735]=32'h81fff8;
font_rom[2736]=32'h1ffdfff8; font_rom[2737]=32'h1fff8000; font_rom[2738]=32'h6070000; font_rom[2739]=32'h6020070; font_rom[2740]=32'h6000ff0; font_rom[2741]=32'h601ff80; font_rom[2742]=32'h7f9ec00; font_rom[2743]=32'h7f98c00;
font_rom[2744]=32'h6198c10; font_rom[2745]=32'h6198638; font_rom[2746]=32'h61986f0; font_rom[2747]=32'h61987c0; font_rom[2748]=32'h6118380; font_rom[2749]=32'h6318300; font_rom[2750]=32'h6318380; font_rom[2751]=32'h6318180;
font_rom[2752]=32'hc3180c0; font_rom[2753]=32'hc3184e0; font_rom[2754]=32'h1c319c78; font_rom[2755]=32'h1bf1f83c; font_rom[2756]=32'h1be1e008; font_rom[2757]=32'h10010000; font_rom[2758]=32'h7ffffe0; font_rom[2759]=32'h7ffffe0;
font_rom[2760]=32'h38000; font_rom[2761]=32'h30000; font_rom[2762]=32'h30000; font_rom[2763]=32'h30000; font_rom[2764]=32'h30000; font_rom[2765]=32'h30000; font_rom[2766]=32'h30000; font_rom[2767]=32'h1ffffff8;
font_rom[2768]=32'h1ffffff8; font_rom[2769]=32'h60000; font_rom[2770]=32'h60000; font_rom[2771]=32'he6000; font_rom[2772]=32'hc6000; font_rom[2773]=32'h1c6000; font_rom[2774]=32'h186018; font_rom[2775]=32'h386018;
font_rom[2776]=32'h706018; font_rom[2777]=32'he06018; font_rom[2778]=32'h1c06018; font_rom[2779]=32'h3807038; font_rom[2780]=32'hf007ff8; font_rom[2781]=32'h1c001fe0; font_rom[2782]=32'h8000000; font_rom[2783]=32'h3ffffc0;
font_rom[2784]=32'h3ffffc0; font_rom[2785]=32'h38001c0; font_rom[2786]=32'h38001c0; font_rom[2787]=32'h38001c0; font_rom[2788]=32'h38001c0; font_rom[2789]=32'h38001c0; font_rom[2790]=32'h38001c0; font_rom[2791]=32'h38001c0;
font_rom[2792]=32'h38001c0; font_rom[2793]=32'h3ffffc0; font_rom[2794]=32'h3ffffc0; font_rom[2795]=32'h38001c0; font_rom[2796]=32'h38001c0; font_rom[2797]=32'h38001c0; font_rom[2798]=32'h38001c0; font_rom[2799]=32'h38001c0;
font_rom[2800]=32'h38001c0; font_rom[2801]=32'h38001c0; font_rom[2802]=32'h38001c0; font_rom[2803]=32'h3ffffc0; font_rom[2804]=32'h3ffffc0; font_rom[2805]=32'h38001c0; font_rom[2806]=32'h38001c0; font_rom[2807]=32'hc0;
font_rom[2808]=32'hc0; font_rom[2809]=32'hc0; font_rom[2810]=32'h1ff800c0; font_rom[2811]=32'h1ff800c0; font_rom[2812]=32'h1c1800c0; font_rom[2813]=32'h1c1bfffc; font_rom[2814]=32'h1c1bfffc; font_rom[2815]=32'h1c1800c0;
font_rom[2816]=32'h1c1800c0; font_rom[2817]=32'h1c1880c0; font_rom[2818]=32'h1ff9c0c0; font_rom[2819]=32'h1ff8c0c0; font_rom[2820]=32'h1c18e0c0; font_rom[2821]=32'h1c1860c0; font_rom[2822]=32'h1c1870c0; font_rom[2823]=32'h1c1820c0;
font_rom[2824]=32'h1c1800c0; font_rom[2825]=32'h1c1800c0; font_rom[2826]=32'h1c1800c0; font_rom[2827]=32'h1ff800c0; font_rom[2828]=32'h1ff801c0; font_rom[2829]=32'h1c1801c0; font_rom[2830]=32'h1c187f80; font_rom[2831]=32'h7f80;
font_rom[2832]=32'h1ffff80; font_rom[2833]=32'h1ffff80; font_rom[2834]=32'h1800180; font_rom[2835]=32'h1800180; font_rom[2836]=32'h1ffff80; font_rom[2837]=32'h1ffff80; font_rom[2838]=32'h1800180; font_rom[2839]=32'h1800180;
font_rom[2840]=32'h1ffff80; font_rom[2841]=32'h1ffff80; font_rom[2842]=32'h1800180; font_rom[2843]=32'h0; font_rom[2844]=32'h1ffffff8; font_rom[2845]=32'h1ffffff8; font_rom[2846]=32'h1c000; font_rom[2847]=32'hc1c000;
font_rom[2848]=32'hc1ffe0; font_rom[2849]=32'h181ffe0; font_rom[2850]=32'h3c1c000; font_rom[2851]=32'h3e1c000; font_rom[2852]=32'h77dc000; font_rom[2853]=32'he3ffffc; font_rom[2854]=32'h3c07fff8; font_rom[2855]=32'h10000000;
font_rom[2856]=32'h8000; font_rom[2857]=32'h1c000; font_rom[2858]=32'hc060; font_rom[2859]=32'hfe0cff0; font_rom[2860]=32'hfe3ff80; font_rom[2861]=32'hc6ff040; font_rom[2862]=32'hc6c70c0; font_rom[2863]=32'hc603380;
font_rom[2864]=32'hc603f18; font_rom[2865]=32'hc601e18; font_rom[2866]=32'hc607e18; font_rom[2867]=32'hfe1e798; font_rom[2868]=32'hfef83f8; font_rom[2869]=32'hc6400f0; font_rom[2870]=32'hc600000; font_rom[2871]=32'hc6ffff8;
font_rom[2872]=32'hc6ffff8; font_rom[2873]=32'hc60c600; font_rom[2874]=32'hc60c600; font_rom[2875]=32'hc60c608; font_rom[2876]=32'hfe1c618; font_rom[2877]=32'hfe18618; font_rom[2878]=32'hc038618; font_rom[2879]=32'hc070718;
font_rom[2880]=32'he03f8; font_rom[2881]=32'h3c01f0; font_rom[2882]=32'h100000; font_rom[2883]=32'h200400; font_rom[2884]=32'h300e00; font_rom[2885]=32'h181c00; font_rom[2886]=32'hffffff0; font_rom[2887]=32'hffffff0;
font_rom[2888]=32'hc3000; font_rom[2889]=32'h30c3080; font_rom[2890]=32'h3cc33c0; font_rom[2891]=32'h8c3100; font_rom[2892]=32'hc3000; font_rom[2893]=32'h1ffffff8; font_rom[2894]=32'h1ffffff8; font_rom[2895]=32'h0;
font_rom[2896]=32'h0; font_rom[2897]=32'h3ffffc0; font_rom[2898]=32'h3ffffc0; font_rom[2899]=32'h38001c0; font_rom[2900]=32'h38001c0; font_rom[2901]=32'h3ffffc0; font_rom[2902]=32'h3ffffc0; font_rom[2903]=32'h38001c0;
font_rom[2904]=32'h38001c0; font_rom[2905]=32'h38001c0; font_rom[2906]=32'h3ffffc0; font_rom[2907]=32'h3ffffc0; font_rom[2908]=32'h38001c0; font_rom[2909]=32'h3ffffc0; font_rom[2910]=32'h3ffffc0; font_rom[2911]=32'h30000c0;
font_rom[2912]=32'h3ffffc0; font_rom[2913]=32'h3ffffc0; font_rom[2914]=32'h30000c0; font_rom[2915]=32'h3ffffc0; font_rom[2916]=32'h3ffffc0; font_rom[2917]=32'h30100c0; font_rom[2918]=32'h18000; font_rom[2919]=32'h1ffffff8;
font_rom[2920]=32'hffffff0; font_rom[2921]=32'h0; font_rom[2922]=32'h3ffffc0; font_rom[2923]=32'h3ffffc0; font_rom[2924]=32'h30000c0; font_rom[2925]=32'h30000c0; font_rom[2926]=32'h3ffffc0; font_rom[2927]=32'h3ffffc0;
font_rom[2928]=32'h3318cc0; font_rom[2929]=32'hf18f00; font_rom[2930]=32'h3c183e0; font_rom[2931]=32'hf1f80f8; font_rom[2932]=32'hc1f0010; font_rom[2933]=32'h800; font_rom[2934]=32'h1c00; font_rom[2935]=32'hc00;
font_rom[2936]=32'hfe7fff0; font_rom[2937]=32'hfe7fff0; font_rom[2938]=32'hc608040; font_rom[2939]=32'hc61c0e0; font_rom[2940]=32'hc60c0c0; font_rom[2941]=32'hc60e180; font_rom[2942]=32'hc604380; font_rom[2943]=32'hc6ffff8;
font_rom[2944]=32'hc6ffff8; font_rom[2945]=32'hfe00000; font_rom[2946]=32'hfe00000; font_rom[2947]=32'hc63ffe0; font_rom[2948]=32'hc63ffe0; font_rom[2949]=32'hc630060; font_rom[2950]=32'hc630060; font_rom[2951]=32'hc630060;
font_rom[2952]=32'hc63ffe0; font_rom[2953]=32'hc63ffe0; font_rom[2954]=32'hfe30060; font_rom[2955]=32'hfe30060; font_rom[2956]=32'hc630060; font_rom[2957]=32'hc03ffe0; font_rom[2958]=32'h3ffe0; font_rom[2959]=32'h30060;
font_rom[2960]=32'h181800; font_rom[2961]=32'h181800; font_rom[2962]=32'h1ffffff8; font_rom[2963]=32'h1ffffff8; font_rom[2964]=32'h181800; font_rom[2965]=32'h3ffffc0; font_rom[2966]=32'h3ffffc0; font_rom[2967]=32'h30000c0;
font_rom[2968]=32'h3ffffc0; font_rom[2969]=32'h38001c0; font_rom[2970]=32'h30000c0; font_rom[2971]=32'h3ffffc0; font_rom[2972]=32'h38e01c0; font_rom[2973]=32'h1c0000; font_rom[2974]=32'h1ffffff8; font_rom[2975]=32'hffffff0;
font_rom[2976]=32'he00300; font_rom[2977]=32'h3ffffe0; font_rom[2978]=32'h7c003f8; font_rom[2979]=32'h1cc00318; font_rom[2980]=32'h10ffff00; font_rom[2981]=32'hc00300; font_rom[2982]=32'hc00300; font_rom[2983]=32'hffff00;
font_rom[2984]=32'hc00300; font_rom[2985]=32'hc00300; font_rom[2986]=32'h1ffffff8; font_rom[2987]=32'h1ffffff8; font_rom[2988]=32'h18000; font_rom[2989]=32'h18000; font_rom[2990]=32'h3ffffc0; font_rom[2991]=32'h3ffffc0;
font_rom[2992]=32'h30180c0; font_rom[2993]=32'h30180c0; font_rom[2994]=32'h3ffffc0; font_rom[2995]=32'h3ffffc0; font_rom[2996]=32'h30180c0; font_rom[2997]=32'h30180c0; font_rom[2998]=32'h30180c0; font_rom[2999]=32'h3ffffc0;
font_rom[3000]=32'h3ffffc0; font_rom[3001]=32'h18300c0; font_rom[3002]=32'h1c30000; font_rom[3003]=32'he70000; font_rom[3004]=32'h7e0000; font_rom[3005]=32'h3e0000; font_rom[3006]=32'h7ff000; font_rom[3007]=32'h3e3fff8;
font_rom[3008]=32'h1f801ff8; font_rom[3009]=32'h1c000000; font_rom[3010]=32'h1ffff80; font_rom[3011]=32'h1ffff80; font_rom[3012]=32'h1800180; font_rom[3013]=32'h1800180; font_rom[3014]=32'h1800180; font_rom[3015]=32'h1800180;
font_rom[3016]=32'h1800180; font_rom[3017]=32'h1ffff80; font_rom[3018]=32'h1ffff80; font_rom[3019]=32'h1800180; font_rom[3020]=32'h1800180; font_rom[3021]=32'h1800180; font_rom[3022]=32'h1800180; font_rom[3023]=32'h1800180;
font_rom[3024]=32'h1ffff80; font_rom[3025]=32'h1ffff80; font_rom[3026]=32'h1800180; font_rom[3027]=32'h3800180; font_rom[3028]=32'h3000180; font_rom[3029]=32'h3000180; font_rom[3030]=32'h7000180; font_rom[3031]=32'h6000180;
font_rom[3032]=32'he007f80; font_rom[3033]=32'h1c007f00; font_rom[3034]=32'h8000000; font_rom[3035]=32'h18000; font_rom[3036]=32'h18000; font_rom[3037]=32'h18000; font_rom[3038]=32'h1ffffff8; font_rom[3039]=32'h1ffffff8;
font_rom[3040]=32'h18000; font_rom[3041]=32'h18000; font_rom[3042]=32'h18000; font_rom[3043]=32'h7ffffe0; font_rom[3044]=32'h7ffffe0; font_rom[3045]=32'h7018060; font_rom[3046]=32'h7018060; font_rom[3047]=32'h7018060;
font_rom[3048]=32'h7018060; font_rom[3049]=32'h7ffffe0; font_rom[3050]=32'h7ffffe0; font_rom[3051]=32'h707e060; font_rom[3052]=32'hff000; font_rom[3053]=32'h19b800; font_rom[3054]=32'h319c00; font_rom[3055]=32'he18700;
font_rom[3056]=32'h3c183c0; font_rom[3057]=32'hf0180f0; font_rom[3058]=32'h1e018078; font_rom[3059]=32'h18018018; font_rom[3060]=32'h18000; font_rom[3061]=32'h1800000; font_rom[3062]=32'h187ffe0; font_rom[3063]=32'h187ffe0;
font_rom[3064]=32'h18001c0; font_rom[3065]=32'h1800780; font_rom[3066]=32'h1ff81e00; font_rom[3067]=32'h1ff83c00; font_rom[3068]=32'h380f000; font_rom[3069]=32'h381c000; font_rom[3070]=32'h3878000; font_rom[3071]=32'h7cffff8;
font_rom[3072]=32'h7effff0; font_rom[3073]=32'h5f0c630; font_rom[3074]=32'hdb9c630; font_rom[3075]=32'hd998630; font_rom[3076]=32'h19938c30; font_rom[3077]=32'h39830c30; font_rom[3078]=32'h11861c30; font_rom[3079]=32'h18e1830;
font_rom[3080]=32'h19c3830; font_rom[3081]=32'h1b87030; font_rom[3082]=32'h1a06030; font_rom[3083]=32'h180c070; font_rom[3084]=32'h1838fe0; font_rom[3085]=32'h1870fc0; font_rom[3086]=32'h1820000; font_rom[3087]=32'h1800000;
font_rom[3088]=32'h18001f0; font_rom[3089]=32'h183fff0; font_rom[3090]=32'h183fe00; font_rom[3091]=32'h1830000; font_rom[3092]=32'h1ffb0000; font_rom[3093]=32'h1ff30000; font_rom[3094]=32'h1830000; font_rom[3095]=32'h3830000;
font_rom[3096]=32'h3830000; font_rom[3097]=32'h3c3fffc; font_rom[3098]=32'h7e3fffc; font_rom[3099]=32'h7f30300; font_rom[3100]=32'hfbb0300; font_rom[3101]=32'hd9b0300; font_rom[3102]=32'hd930300; font_rom[3103]=32'h19830300;
font_rom[3104]=32'h19830300; font_rom[3105]=32'h11830300; font_rom[3106]=32'h1870300; font_rom[3107]=32'h1860300; font_rom[3108]=32'h1860300; font_rom[3109]=32'h18e0300; font_rom[3110]=32'h18c0300; font_rom[3111]=32'h19c0300;
font_rom[3112]=32'h1980300; font_rom[3113]=32'h1900300; font_rom[3114]=32'h200c000; font_rom[3115]=32'h780c000; font_rom[3116]=32'h1e0c000; font_rom[3117]=32'h41c000; font_rom[3118]=32'h180fff00; font_rom[3119]=32'h1e0fff00;
font_rom[3120]=32'h7018300; font_rom[3121]=32'h3018300; font_rom[3122]=32'h18308; font_rom[3123]=32'he3830c; font_rom[3124]=32'h1c3031c; font_rom[3125]=32'h7860398; font_rom[3126]=32'hf0e01f8; font_rom[3127]=32'h1c3c00f0;
font_rom[3128]=32'h8118000; font_rom[3129]=32'h18000; font_rom[3130]=32'h1ffffff8; font_rom[3131]=32'h1ffffff8; font_rom[3132]=32'hdf000; font_rom[3133]=32'h19b800; font_rom[3134]=32'h319c00; font_rom[3135]=32'he18e00;
font_rom[3136]=32'h3c18780; font_rom[3137]=32'hf0181f0; font_rom[3138]=32'h1e018078; font_rom[3139]=32'h8018010; font_rom[3140]=32'h18000; font_rom[3141]=32'h300e000; font_rom[3142]=32'h30ff000; font_rom[3143]=32'h30f07f0;
font_rom[3144]=32'h30c07f0; font_rom[3145]=32'h30c0630; font_rom[3146]=32'h1fec6630; font_rom[3147]=32'h1fec6630; font_rom[3148]=32'h30c6630; font_rom[3149]=32'h30c6630; font_rom[3150]=32'h70c6630; font_rom[3151]=32'h7cc6630;
font_rom[3152]=32'h7cc6630; font_rom[3153]=32'hf6c6630; font_rom[3154]=32'hf3c6630; font_rom[3155]=32'h1b2c6630; font_rom[3156]=32'h1b0de630; font_rom[3157]=32'h3b0fe630; font_rom[3158]=32'h130f6630; font_rom[3159]=32'h30cc630;
font_rom[3160]=32'h300c6f0; font_rom[3161]=32'h300c6e0; font_rom[3162]=32'h3018600; font_rom[3163]=32'h3038600; font_rom[3164]=32'h3070600; font_rom[3165]=32'h30e0600; font_rom[3166]=32'h31c0600; font_rom[3167]=32'h3180600;
font_rom[3168]=32'h600600; font_rom[3169]=32'h600600; font_rom[3170]=32'h600600; font_rom[3171]=32'hffe7ff8; font_rom[3172]=32'hffe7ff8; font_rom[3173]=32'he00e00; font_rom[3174]=32'h1f81f80; font_rom[3175]=32'h37e3fe0;
font_rom[3176]=32'h66e76f0; font_rom[3177]=32'he62e638; font_rom[3178]=32'h1c61c610; font_rom[3179]=32'h8608600; font_rom[3180]=32'h0; font_rom[3181]=32'hffffff0; font_rom[3182]=32'h7fffff0; font_rom[3183]=32'h18060;
font_rom[3184]=32'hc18060; font_rom[3185]=32'hc18000; font_rom[3186]=32'hc1ffc0; font_rom[3187]=32'h181ffc0; font_rom[3188]=32'h1c18000; font_rom[3189]=32'h3e18000; font_rom[3190]=32'h3718000; font_rom[3191]=32'h6398000;
font_rom[3192]=32'h1e1ffff8; font_rom[3193]=32'h1c07fff8; font_rom[3194]=32'h10000000; font_rom[3195]=32'h18000; font_rom[3196]=32'h18000; font_rom[3197]=32'h8038000; font_rom[3198]=32'h1c030000; font_rom[3199]=32'he030000;
font_rom[3200]=32'h707fff0; font_rom[3201]=32'h387fff0; font_rom[3202]=32'h1ce0030; font_rom[3203]=32'h8c3060; font_rom[3204]=32'h1c3060; font_rom[3205]=32'h1830e0; font_rom[3206]=32'h8830c0; font_rom[3207]=32'h1c07000;
font_rom[3208]=32'h1c07800; font_rom[3209]=32'h3807800; font_rom[3210]=32'h300f800; font_rom[3211]=32'h700cc00; font_rom[3212]=32'h601ce00; font_rom[3213]=32'he018600; font_rom[3214]=32'hc038700; font_rom[3215]=32'h1c070380;
font_rom[3216]=32'h180e01c0; font_rom[3217]=32'h81c00e0; font_rom[3218]=32'h780078; font_rom[3219]=32'hf00038; font_rom[3220]=32'h400010; font_rom[3221]=32'h181800; font_rom[3222]=32'h181800; font_rom[3223]=32'h181800;
font_rom[3224]=32'h181800; font_rom[3225]=32'h181800; font_rom[3226]=32'h6181810; font_rom[3227]=32'h6181878; font_rom[3228]=32'h61818f0; font_rom[3229]=32'h61819c0; font_rom[3230]=32'h61fdf80; font_rom[3231]=32'h61fde00;
font_rom[3232]=32'h6181c00; font_rom[3233]=32'h6181800; font_rom[3234]=32'h6181800; font_rom[3235]=32'h6181800; font_rom[3236]=32'h6181800; font_rom[3237]=32'h6181800; font_rom[3238]=32'h6181800; font_rom[3239]=32'h6181818;
font_rom[3240]=32'h6181818; font_rom[3241]=32'h6181818; font_rom[3242]=32'h6199818; font_rom[3243]=32'h67f9818; font_rom[3244]=32'hfff1838; font_rom[3245]=32'h1fe01ff8; font_rom[3246]=32'h1c000ff0; font_rom[3247]=32'h6200;
font_rom[3248]=32'h6700; font_rom[3249]=32'h1ffc61c0; font_rom[3250]=32'h1ffc6080; font_rom[3251]=32'h3006000; font_rom[3252]=32'h3006000; font_rom[3253]=32'h3006070; font_rom[3254]=32'h3007ff0; font_rom[3255]=32'h3fbff80;
font_rom[3256]=32'h7fbe000; font_rom[3257]=32'h6186000; font_rom[3258]=32'h61870f8; font_rom[3259]=32'hc307ff8; font_rom[3260]=32'hc37ff00; font_rom[3261]=32'h1b33b000; font_rom[3262]=32'h1bf03020; font_rom[3263]=32'h10e03070;
font_rom[3264]=32'h6039e0; font_rom[3265]=32'hc01b80; font_rom[3266]=32'hc01f08; font_rom[3267]=32'h1803c18; font_rom[3268]=32'h380fc18; font_rom[3269]=32'h703ee18; font_rom[3270]=32'he0f8738; font_rom[3271]=32'h1c0c03f0;
font_rom[3272]=32'h80000f0; font_rom[3273]=32'hc0000; font_rom[3274]=32'h1f0000; font_rom[3275]=32'h7e000; font_rom[3276]=32'hf000; font_rom[3277]=32'h2000; font_rom[3278]=32'h0; font_rom[3279]=32'h3ffc000;
font_rom[3280]=32'h3ffc020; font_rom[3281]=32'h1c0f0; font_rom[3282]=32'h1c1e0; font_rom[3283]=32'h1e780; font_rom[3284]=32'h1ff1ff00; font_rom[3285]=32'h1ff1fc00; font_rom[3286]=32'h31f800; font_rom[3287]=32'h71d800;
font_rom[3288]=32'h61cc00; font_rom[3289]=32'he1ce00; font_rom[3290]=32'hc1c700; font_rom[3291]=32'h1c1c380; font_rom[3292]=32'h381c1c0; font_rom[3293]=32'h701c0e0; font_rom[3294]=32'he01c078; font_rom[3295]=32'h1c018038;
font_rom[3296]=32'h18018010; font_rom[3297]=32'h3f8000; font_rom[3298]=32'h3f0000; font_rom[3299]=32'h4002000; font_rom[3300]=32'he007000; font_rom[3301]=32'h7003800; font_rom[3302]=32'h3c01800; font_rom[3303]=32'h1c01000;
font_rom[3304]=32'h9ffffc; font_rom[3305]=32'h1ffffc; font_rom[3306]=32'h10003000; font_rom[3307]=32'h1c003000; font_rom[3308]=32'he003000; font_rom[3309]=32'h7003000; font_rom[3310]=32'h3803c00; font_rom[3311]=32'h1003e00;
font_rom[3312]=32'h3780; font_rom[3313]=32'h31c0; font_rom[3314]=32'h10030f0; font_rom[3315]=32'h1803070; font_rom[3316]=32'h3803010; font_rom[3317]=32'h3803000; font_rom[3318]=32'h3003000; font_rom[3319]=32'h7003000;
font_rom[3320]=32'h7003000; font_rom[3321]=32'h6003000; font_rom[3322]=32'he003000; font_rom[3323]=32'he003000; font_rom[3324]=32'hc003000; font_rom[3325]=32'h4003000; font_rom[3326]=32'h8000000; font_rom[3327]=32'he000000;
font_rom[3328]=32'hf1ffff0; font_rom[3329]=32'h79ffff0; font_rom[3330]=32'h3980030; font_rom[3331]=32'h1180030; font_rom[3332]=32'h180030; font_rom[3333]=32'h10000000; font_rom[3334]=32'h1c03ff00; font_rom[3335]=32'h1e03ff00;
font_rom[3336]=32'hf030300; font_rom[3337]=32'h7030300; font_rom[3338]=32'h2030300; font_rom[3339]=32'h30300; font_rom[3340]=32'h30300; font_rom[3341]=32'h3030300; font_rom[3342]=32'h3070300; font_rom[3343]=32'h3070300;
font_rom[3344]=32'h3060300; font_rom[3345]=32'h7060308; font_rom[3346]=32'h60e031c; font_rom[3347]=32'h61c0318; font_rom[3348]=32'he3c0318; font_rom[3349]=32'he780318; font_rom[3350]=32'hcf003f8; font_rom[3351]=32'h1c4001f0;
font_rom[3352]=32'hc000000; font_rom[3353]=32'h4003000; font_rom[3354]=32'he003000; font_rom[3355]=32'h7807000; font_rom[3356]=32'h3cffff0; font_rom[3357]=32'h1cffff0; font_rom[3358]=32'h8c0030; font_rom[3359]=32'hc0030;
font_rom[3360]=32'h100c0030; font_rom[3361]=32'h1c0c0030; font_rom[3362]=32'h1e0c0030; font_rom[3363]=32'h78c0030; font_rom[3364]=32'h38c0030; font_rom[3365]=32'h10ffff0; font_rom[3366]=32'hffff0; font_rom[3367]=32'hc0030;
font_rom[3368]=32'h10c0030; font_rom[3369]=32'h18c0030; font_rom[3370]=32'h38c0030; font_rom[3371]=32'h30c0030; font_rom[3372]=32'h70c0030; font_rom[3373]=32'h70c0030; font_rom[3374]=32'h60c0030; font_rom[3375]=32'he0ffff0;
font_rom[3376]=32'he0ffff0; font_rom[3377]=32'h40c0030; font_rom[3378]=32'h2000; font_rom[3379]=32'h8007000; font_rom[3380]=32'he007000; font_rom[3381]=32'hf003800; font_rom[3382]=32'h7801000; font_rom[3383]=32'h3bffff8;
font_rom[3384]=32'h13ffff8; font_rom[3385]=32'h3800; font_rom[3386]=32'h10003800; font_rom[3387]=32'h1c003800; font_rom[3388]=32'h1e003800; font_rom[3389]=32'hf003800; font_rom[3390]=32'h7003800; font_rom[3391]=32'h2003800;
font_rom[3392]=32'h1ffff0; font_rom[3393]=32'h1ffff0; font_rom[3394]=32'h2003800; font_rom[3395]=32'h3003800; font_rom[3396]=32'h3003800; font_rom[3397]=32'h7003800; font_rom[3398]=32'h6003800; font_rom[3399]=32'h6003800;
font_rom[3400]=32'he003800; font_rom[3401]=32'hc003800; font_rom[3402]=32'hc7ffff8; font_rom[3403]=32'h1c7ffff8; font_rom[3404]=32'h1c000000; font_rom[3405]=32'h8000000; font_rom[3406]=32'h4000000; font_rom[3407]=32'he0ffff0;
font_rom[3408]=32'h78ffff0; font_rom[3409]=32'h3cc0030; font_rom[3410]=32'h8c0030; font_rom[3411]=32'hc0030; font_rom[3412]=32'hc0030; font_rom[3413]=32'h100c0030; font_rom[3414]=32'h1c0ffff0; font_rom[3415]=32'he0ffff0;
font_rom[3416]=32'h78c0030; font_rom[3417]=32'h10c0030; font_rom[3418]=32'hc0030; font_rom[3419]=32'hc0030; font_rom[3420]=32'h30c0030; font_rom[3421]=32'h30ffff0; font_rom[3422]=32'h30ffff0; font_rom[3423]=32'h70c0030;
font_rom[3424]=32'h70c0030; font_rom[3425]=32'h60c0030; font_rom[3426]=32'h60c0030; font_rom[3427]=32'he0c0030; font_rom[3428]=32'he0ffff0; font_rom[3429]=32'hc0ffff0; font_rom[3430]=32'hc0c0030; font_rom[3431]=32'hc0c0030;
font_rom[3432]=32'hc000000; font_rom[3433]=32'he0003f0; font_rom[3434]=32'hf1ffff0; font_rom[3435]=32'h39ffe00; font_rom[3436]=32'h1980000; font_rom[3437]=32'h180000; font_rom[3438]=32'h180070; font_rom[3439]=32'h10180ff0;
font_rom[3440]=32'h1c18ffc0; font_rom[3441]=32'h1e18fe00; font_rom[3442]=32'hf18c600; font_rom[3443]=32'h718c610; font_rom[3444]=32'h218c638; font_rom[3445]=32'h18c6f0; font_rom[3446]=32'h18c7c0; font_rom[3447]=32'h218c780;
font_rom[3448]=32'h718c300; font_rom[3449]=32'h618c380; font_rom[3450]=32'h638c180; font_rom[3451]=32'h630c1c0; font_rom[3452]=32'he30c0e0; font_rom[3453]=32'hc30c4e0; font_rom[3454]=32'hc70cc78; font_rom[3455]=32'hc60fc38;
font_rom[3456]=32'h1ce0f010; font_rom[3457]=32'h1cc1c000; font_rom[3458]=32'h8808000; font_rom[3459]=32'hc000000; font_rom[3460]=32'he1ffff8; font_rom[3461]=32'h79ffff8; font_rom[3462]=32'h1d80038; font_rom[3463]=32'h980038;
font_rom[3464]=32'h188238; font_rom[3465]=32'h1c700; font_rom[3466]=32'h180383c0; font_rom[3467]=32'h1c0f00e0; font_rom[3468]=32'hf1c3870; font_rom[3469]=32'h3983810; font_rom[3470]=32'h1003800; font_rom[3471]=32'h3ffff8;
font_rom[3472]=32'h3ffff8; font_rom[3473]=32'h2007800; font_rom[3474]=32'h3007c00; font_rom[3475]=32'h300be00; font_rom[3476]=32'h703bb00; font_rom[3477]=32'h60739c0; font_rom[3478]=32'h60e38e0; font_rom[3479]=32'he3c3878;
font_rom[3480]=32'hc703838; font_rom[3481]=32'hc203818; font_rom[3482]=32'h1c003800; font_rom[3483]=32'h8003800; font_rom[3484]=32'h1800; font_rom[3485]=32'hc001800; font_rom[3486]=32'he001800; font_rom[3487]=32'h73ffff8;
font_rom[3488]=32'h39ffff8; font_rom[3489]=32'h1001800; font_rom[3490]=32'h1ffff0; font_rom[3491]=32'hffff0; font_rom[3492]=32'h10001800; font_rom[3493]=32'h1c001800; font_rom[3494]=32'he3ffffc; font_rom[3495]=32'h73ffff8;
font_rom[3496]=32'h2000000; font_rom[3497]=32'h1ffff0; font_rom[3498]=32'h1ffff0; font_rom[3499]=32'h1c0030; font_rom[3500]=32'h21c0030; font_rom[3501]=32'h31ffff0; font_rom[3502]=32'h71ffff0; font_rom[3503]=32'h71c0030;
font_rom[3504]=32'h61c0030; font_rom[3505]=32'h61ffff0; font_rom[3506]=32'he1ffff0; font_rom[3507]=32'he1c0030; font_rom[3508]=32'hc1c0070; font_rom[3509]=32'h1c1c0fe0; font_rom[3510]=32'hc1c0fe0; font_rom[3511]=32'h8030300;
font_rom[3512]=32'h1c030300; font_rom[3513]=32'he7ffff8; font_rom[3514]=32'h77ffff8; font_rom[3515]=32'h2030300; font_rom[3516]=32'h30300; font_rom[3517]=32'h0; font_rom[3518]=32'h107ffff8; font_rom[3519]=32'h3c7ffff8;
font_rom[3520]=32'he018600; font_rom[3521]=32'h7018600; font_rom[3522]=32'h2018600; font_rom[3523]=32'h3ffff8; font_rom[3524]=32'h3ffff8; font_rom[3525]=32'h318618; font_rom[3526]=32'h6318618; font_rom[3527]=32'h7338618;
font_rom[3528]=32'h633c718; font_rom[3529]=32'h633ef98; font_rom[3530]=32'he366df8; font_rom[3531]=32'he3e3cf8; font_rom[3532]=32'hc3c1858; font_rom[3533]=32'hc383018; font_rom[3534]=32'h1c302030; font_rom[3535]=32'h1c3003f0;
font_rom[3536]=32'h83003e0; font_rom[3537]=32'h8000000; font_rom[3538]=32'he7ffff8; font_rom[3539]=32'h73ffff8; font_rom[3540]=32'h3818600; font_rom[3541]=32'h1018600; font_rom[3542]=32'h3ffff0; font_rom[3543]=32'h3ffff0;
font_rom[3544]=32'h18318630; font_rom[3545]=32'h1c318630; font_rom[3546]=32'hf3ffff0; font_rom[3547]=32'h73ffff0; font_rom[3548]=32'h300030; font_rom[3549]=32'h0; font_rom[3550]=32'hfffc0; font_rom[3551]=32'hfffe0;
font_rom[3552]=32'h3000000; font_rom[3553]=32'h7000000; font_rom[3554]=32'h67ffff8; font_rom[3555]=32'h67ffff8; font_rom[3556]=32'h6003000; font_rom[3557]=32'h60e3180; font_rom[3558]=32'hc1e31e0; font_rom[3559]=32'hc7830f0;
font_rom[3560]=32'hcf03038; font_rom[3561]=32'h1c43f008; font_rom[3562]=32'hc03e000; font_rom[3563]=32'h18000; font_rom[3564]=32'h18000; font_rom[3565]=32'h18000; font_rom[3566]=32'h1fff8; font_rom[3567]=32'h1fff8;
font_rom[3568]=32'h18000; font_rom[3569]=32'h18000; font_rom[3570]=32'h18000; font_rom[3571]=32'h3ffffe0; font_rom[3572]=32'h3ffffe0; font_rom[3573]=32'h30000e0; font_rom[3574]=32'h30000e0; font_rom[3575]=32'h30000e0;
font_rom[3576]=32'h30000e0; font_rom[3577]=32'h30000e0; font_rom[3578]=32'h30000e0; font_rom[3579]=32'h3ffffe0; font_rom[3580]=32'h3ffffe0; font_rom[3581]=32'h30000e0; font_rom[3582]=32'h0; font_rom[3583]=32'h2182040;
font_rom[3584]=32'h71870e0; font_rom[3585]=32'he1c3070; font_rom[3586]=32'h1c0c3830; font_rom[3587]=32'h1c0c1838; font_rom[3588]=32'h80c1810; font_rom[3589]=32'h3fffff0; font_rom[3590]=32'h3fffff0; font_rom[3591]=32'h301c030;
font_rom[3592]=32'h300c030; font_rom[3593]=32'h300c030; font_rom[3594]=32'h300c030; font_rom[3595]=32'h300c030; font_rom[3596]=32'h3fffff0; font_rom[3597]=32'h3fffff0; font_rom[3598]=32'h300c030; font_rom[3599]=32'h300c030;
font_rom[3600]=32'h300c030; font_rom[3601]=32'h300c030; font_rom[3602]=32'h700c030; font_rom[3603]=32'h7fffff0; font_rom[3604]=32'h7fffff0; font_rom[3605]=32'h600c030; font_rom[3606]=32'h600c030; font_rom[3607]=32'h600c030;
font_rom[3608]=32'he00c030; font_rom[3609]=32'hc00c030; font_rom[3610]=32'h1c00c070; font_rom[3611]=32'h1800cff0; font_rom[3612]=32'h800cfe0; font_rom[3613]=32'h1ffffff8; font_rom[3614]=32'h1ffffff8; font_rom[3615]=32'h0;
font_rom[3616]=32'h0; font_rom[3617]=32'hc7ffe30; font_rom[3618]=32'hc7ffe30; font_rom[3619]=32'hc618630; font_rom[3620]=32'hc618630; font_rom[3621]=32'hc618630; font_rom[3622]=32'hc618630; font_rom[3623]=32'hc7ffe30;
font_rom[3624]=32'hc7ffe30; font_rom[3625]=32'hc618630; font_rom[3626]=32'hc618630; font_rom[3627]=32'hc618630; font_rom[3628]=32'hc7ffe30; font_rom[3629]=32'hc7ffe30; font_rom[3630]=32'hc600630; font_rom[3631]=32'hc600630;
font_rom[3632]=32'hc000030; font_rom[3633]=32'hc000030; font_rom[3634]=32'hffffff0; font_rom[3635]=32'hffffff0; font_rom[3636]=32'h30; font_rom[3637]=32'h4000; font_rom[3638]=32'he000; font_rom[3639]=32'h6000;
font_rom[3640]=32'h1fffff8; font_rom[3641]=32'h1fffff8; font_rom[3642]=32'h1800000; font_rom[3643]=32'h19800000; font_rom[3644]=32'h19bffff0; font_rom[3645]=32'h199ffff0; font_rom[3646]=32'h1d8601c0; font_rom[3647]=32'hd83c700;
font_rom[3648]=32'hd807c00; font_rom[3649]=32'h19ffff8; font_rom[3650]=32'h19ffff8; font_rom[3651]=32'h7981818; font_rom[3652]=32'h1f981818; font_rom[3653]=32'h1f9ffff8; font_rom[3654]=32'h131ffff8; font_rom[3655]=32'h3181818;
font_rom[3656]=32'h3181818; font_rom[3657]=32'h3181818; font_rom[3658]=32'h71ffff8; font_rom[3659]=32'h61ffff8; font_rom[3660]=32'he181818; font_rom[3661]=32'h1c181838; font_rom[3662]=32'h18181bf0; font_rom[3663]=32'h81813e0;
font_rom[3664]=32'h60000; font_rom[3665]=32'h70000; font_rom[3666]=32'h60000; font_rom[3667]=32'he0000; font_rom[3668]=32'h7ffffe0; font_rom[3669]=32'h7ffffe0; font_rom[3670]=32'h6000060; font_rom[3671]=32'h6000060;
font_rom[3672]=32'h6000060; font_rom[3673]=32'h6000060; font_rom[3674]=32'h6000060; font_rom[3675]=32'h6000060; font_rom[3676]=32'h6000060; font_rom[3677]=32'h7ffffe0; font_rom[3678]=32'h7ffffe0; font_rom[3679]=32'h6000060;
font_rom[3680]=32'h6000060; font_rom[3681]=32'h6000060; font_rom[3682]=32'h6000060; font_rom[3683]=32'h6000060; font_rom[3684]=32'h6000060; font_rom[3685]=32'h6000060; font_rom[3686]=32'h7ffffe0; font_rom[3687]=32'h7ffffe0;
font_rom[3688]=32'h6000060; font_rom[3689]=32'h6000060; font_rom[3690]=32'hc01800; font_rom[3691]=32'h1c03800; font_rom[3692]=32'h1803000; font_rom[3693]=32'h1803000; font_rom[3694]=32'hffc7ff8; font_rom[3695]=32'hffc7ff0;
font_rom[3696]=32'hc0ce030; font_rom[3697]=32'hc0cc030; font_rom[3698]=32'hc0dc030; font_rom[3699]=32'hc0d8030; font_rom[3700]=32'hc0ca030; font_rom[3701]=32'hc0c7030; font_rom[3702]=32'hc0c3830; font_rom[3703]=32'hffc1830;
font_rom[3704]=32'hffc1c30; font_rom[3705]=32'hc0c0e30; font_rom[3706]=32'hc0c0e30; font_rom[3707]=32'hc0c0430; font_rom[3708]=32'hc0c0030; font_rom[3709]=32'hc0c0030; font_rom[3710]=32'hc0c0030; font_rom[3711]=32'hc0c0030;
font_rom[3712]=32'hffc0070; font_rom[3713]=32'hffc0070; font_rom[3714]=32'hc0c3fe0; font_rom[3715]=32'hc0c3fc0; font_rom[3716]=32'h1800000; font_rom[3717]=32'h1800000; font_rom[3718]=32'h183fff0; font_rom[3719]=32'h183fff0;
font_rom[3720]=32'h1830030; font_rom[3721]=32'h1830030; font_rom[3722]=32'h1ffb0030; font_rom[3723]=32'h1ffb0030; font_rom[3724]=32'h1830030; font_rom[3725]=32'h383fff0; font_rom[3726]=32'h383fff0; font_rom[3727]=32'h3a30030;
font_rom[3728]=32'h7f30030; font_rom[3729]=32'hfbb0030; font_rom[3730]=32'hd9b0030; font_rom[3731]=32'h1d8b0030; font_rom[3732]=32'h1983fff0; font_rom[3733]=32'h1183fff0; font_rom[3734]=32'h11830030; font_rom[3735]=32'h1830030;
font_rom[3736]=32'h1830030; font_rom[3737]=32'h1830030; font_rom[3738]=32'h1830030; font_rom[3739]=32'h183fff0; font_rom[3740]=32'h183fff0; font_rom[3741]=32'h1830030; font_rom[3742]=32'h1830030; font_rom[3743]=32'h1fe0;
font_rom[3744]=32'hfffffe0; font_rom[3745]=32'h7ff8000; font_rom[3746]=32'h70000; font_rom[3747]=32'h7ffffe0; font_rom[3748]=32'h7ffffe0; font_rom[3749]=32'hc0000; font_rom[3750]=32'h1ffffff8; font_rom[3751]=32'h1ffffff8;
font_rom[3752]=32'h700000; font_rom[3753]=32'h600000; font_rom[3754]=32'hffffc0; font_rom[3755]=32'h1ffffc0; font_rom[3756]=32'h3c000c0; font_rom[3757]=32'h6ffffc0; font_rom[3758]=32'h1cffffc0; font_rom[3759]=32'h18c000c0;
font_rom[3760]=32'h10c000c0; font_rom[3761]=32'hffffc0; font_rom[3762]=32'hffffc0; font_rom[3763]=32'hc000c0; font_rom[3764]=32'hc000c0; font_rom[3765]=32'hffffc0; font_rom[3766]=32'hffffc0; font_rom[3767]=32'hc000c0;
font_rom[3768]=32'hff1fff0; font_rom[3769]=32'hff1fff0; font_rom[3770]=32'hc318070; font_rom[3771]=32'hc318070; font_rom[3772]=32'hc318070; font_rom[3773]=32'hc318070; font_rom[3774]=32'hc31fff0; font_rom[3775]=32'hff1fff0;
font_rom[3776]=32'hff18070; font_rom[3777]=32'hc318070; font_rom[3778]=32'hc318070; font_rom[3779]=32'hc31fff0; font_rom[3780]=32'hc31fff0; font_rom[3781]=32'hc319870; font_rom[3782]=32'hff18c08; font_rom[3783]=32'hff18c38;
font_rom[3784]=32'hc318e70; font_rom[3785]=32'hc3187c0; font_rom[3786]=32'hc318780; font_rom[3787]=32'hc318300; font_rom[3788]=32'hc318180; font_rom[3789]=32'hff184e0; font_rom[3790]=32'hff19c70; font_rom[3791]=32'hc33f838;
font_rom[3792]=32'hc33e010; font_rom[3793]=32'h18000; font_rom[3794]=32'h10000; font_rom[3795]=32'h38000; font_rom[3796]=32'h38000; font_rom[3797]=32'h1ffffff8; font_rom[3798]=32'h1ffffff8; font_rom[3799]=32'h200400;
font_rom[3800]=32'h37c1cc0; font_rom[3801]=32'h30ff0c0; font_rom[3802]=32'h307e0c0; font_rom[3803]=32'h33f7cc0; font_rom[3804]=32'h37004c0; font_rom[3805]=32'h30000c0; font_rom[3806]=32'h3ffffc0; font_rom[3807]=32'h3ffffc0;
font_rom[3808]=32'h380c0; font_rom[3809]=32'h30000; font_rom[3810]=32'h7ffffe0; font_rom[3811]=32'h7ffffe0; font_rom[3812]=32'h60c0060; font_rom[3813]=32'h60c2060; font_rom[3814]=32'h6183860; font_rom[3815]=32'h6381c60;
font_rom[3816]=32'h63ffe60; font_rom[3817]=32'h63fcc60; font_rom[3818]=32'h6000060; font_rom[3819]=32'h60003e0; font_rom[3820]=32'h60003c0; font_rom[3821]=32'h781800; font_rom[3822]=32'h1ff81800; font_rom[3823]=32'h1f801800;
font_rom[3824]=32'h1801800; font_rom[3825]=32'h1801800; font_rom[3826]=32'h1841810; font_rom[3827]=32'h1861838; font_rom[3828]=32'h1ffe1870; font_rom[3829]=32'h1ffb18e0; font_rom[3830]=32'h18319c0; font_rom[3831]=32'h3819980;
font_rom[3832]=32'h3819d00; font_rom[3833]=32'h7a0bc00; font_rom[3834]=32'h7f03400; font_rom[3835]=32'hfb03600; font_rom[3836]=32'hd983600; font_rom[3837]=32'h1d906200; font_rom[3838]=32'h19806300; font_rom[3839]=32'h1180c380;
font_rom[3840]=32'h1181c180; font_rom[3841]=32'h18380c0; font_rom[3842]=32'h18700e0; font_rom[3843]=32'h18e0070; font_rom[3844]=32'h19c0038; font_rom[3845]=32'h1880010; font_rom[3846]=32'h10000; font_rom[3847]=32'h38000;
font_rom[3848]=32'h18000; font_rom[3849]=32'hffffff0; font_rom[3850]=32'h7ffffe0; font_rom[3851]=32'he00600; font_rom[3852]=32'h700e00; font_rom[3853]=32'h301c00; font_rom[3854]=32'h1ffffff8; font_rom[3855]=32'h1ffffff8;
font_rom[3856]=32'h0; font_rom[3857]=32'h3ffffc0; font_rom[3858]=32'h3ffffc0; font_rom[3859]=32'h30000c0; font_rom[3860]=32'h30000c0; font_rom[3861]=32'h3ffffc0; font_rom[3862]=32'h3ffffc0; font_rom[3863]=32'h30000c0;
font_rom[3864]=32'h30000c0; font_rom[3865]=32'h3ffffc0; font_rom[3866]=32'h3ffffd8; font_rom[3867]=32'h183018; font_rom[3868]=32'h383018; font_rom[3869]=32'h703818; font_rom[3870]=32'h3e01ff8; font_rom[3871]=32'h1fc00fe0;
font_rom[3872]=32'h1e000000; font_rom[3873]=32'h10000; font_rom[3874]=32'h38000; font_rom[3875]=32'h18000; font_rom[3876]=32'hffffff0; font_rom[3877]=32'hffffff0; font_rom[3878]=32'h600600; font_rom[3879]=32'h600e00;
font_rom[3880]=32'h300c00; font_rom[3881]=32'h1ffffff8; font_rom[3882]=32'h1ffffff8; font_rom[3883]=32'h0; font_rom[3884]=32'h3ffffc0; font_rom[3885]=32'h3ffffc0; font_rom[3886]=32'h30000c0; font_rom[3887]=32'h30000c0;
font_rom[3888]=32'h3ffffc0; font_rom[3889]=32'h3ffffc0; font_rom[3890]=32'h30000c0; font_rom[3891]=32'h30000c0; font_rom[3892]=32'h3ffffc0; font_rom[3893]=32'h3ffffc0; font_rom[3894]=32'h30180c0; font_rom[3895]=32'h18000;
font_rom[3896]=32'h1ffffff8; font_rom[3897]=32'h1ffffff8; font_rom[3898]=32'h18000; font_rom[3899]=32'h18000; font_rom[3900]=32'h1c01c00; font_rom[3901]=32'h1801800; font_rom[3902]=32'h3ffbff8; font_rom[3903]=32'h3ff3ff8;
font_rom[3904]=32'h6606600; font_rom[3905]=32'hc60c700; font_rom[3906]=32'h18304300; font_rom[3907]=32'h10200100; font_rom[3908]=32'h7ffffe0; font_rom[3909]=32'h7ffffe0; font_rom[3910]=32'h180e0; font_rom[3911]=32'h180e0;
font_rom[3912]=32'h3ffffe0; font_rom[3913]=32'h7ffffe0; font_rom[3914]=32'h60180e0; font_rom[3915]=32'h6018000; font_rom[3916]=32'h6018000; font_rom[3917]=32'h6018000; font_rom[3918]=32'hffffff0; font_rom[3919]=32'h7fffff0;
font_rom[3920]=32'hf8030; font_rom[3921]=32'h3d8030; font_rom[3922]=32'hf18070; font_rom[3923]=32'h3c18fe0; font_rom[3924]=32'h1f018fc0; font_rom[3925]=32'h18018000; font_rom[3926]=32'h1c01c00; font_rom[3927]=32'h1801800;
font_rom[3928]=32'h3803800; font_rom[3929]=32'h3ffbff8; font_rom[3930]=32'h7ff7ff8; font_rom[3931]=32'he406400; font_rom[3932]=32'hc60c600; font_rom[3933]=32'h18304300; font_rom[3934]=32'h10218200; font_rom[3935]=32'h18000;
font_rom[3936]=32'hffffff0; font_rom[3937]=32'h7ffffe0; font_rom[3938]=32'h18000; font_rom[3939]=32'h18000; font_rom[3940]=32'h1ffffffc; font_rom[3941]=32'h1ffffff8; font_rom[3942]=32'h600; font_rom[3943]=32'h600;
font_rom[3944]=32'h1ffffff8; font_rom[3945]=32'hffffff0; font_rom[3946]=32'h600600; font_rom[3947]=32'h700600; font_rom[3948]=32'h380600; font_rom[3949]=32'h1c0600; font_rom[3950]=32'h80e00; font_rom[3951]=32'h1fc00;
font_rom[3952]=32'h1f800; font_rom[3953]=32'hc01c00; font_rom[3954]=32'h1801800; font_rom[3955]=32'h3ffbff8; font_rom[3956]=32'h3ff3ff8; font_rom[3957]=32'h6406400; font_rom[3958]=32'hc614600; font_rom[3959]=32'h18338300;
font_rom[3960]=32'h10218200; font_rom[3961]=32'h3ffffe0; font_rom[3962]=32'h3ffffe0; font_rom[3963]=32'h3000060; font_rom[3964]=32'h3000060; font_rom[3965]=32'h3ffffe0; font_rom[3966]=32'h3ffffe0; font_rom[3967]=32'h7000000;
font_rom[3968]=32'h6000000; font_rom[3969]=32'h6fffff0; font_rom[3970]=32'h6fffff0; font_rom[3971]=32'h6c30c30; font_rom[3972]=32'hec30c30; font_rom[3973]=32'hcfffff0; font_rom[3974]=32'h1cfffff0; font_rom[3975]=32'h18c30c30;
font_rom[3976]=32'h18c30c30; font_rom[3977]=32'h10c30ff0; font_rom[3978]=32'hc001e0; font_rom[3979]=32'h300000; font_rom[3980]=32'hc33ffe0; font_rom[3981]=32'hc33ffe0; font_rom[3982]=32'hc31c0c0; font_rom[3983]=32'hc30c380;
font_rom[3984]=32'hc307700; font_rom[3985]=32'hc303e00; font_rom[3986]=32'hc30ff00; font_rom[3987]=32'hc37e7f8; font_rom[3988]=32'hc330070; font_rom[3989]=32'hf0000; font_rom[3990]=32'h3c0200; font_rom[3991]=32'h1e01f00;
font_rom[3992]=32'h1fff800; font_rom[3993]=32'h1ffe000; font_rom[3994]=32'h78200; font_rom[3995]=32'h1c0300; font_rom[3996]=32'hf0ffc0; font_rom[3997]=32'h7fffc60; font_rom[3998]=32'h3e0c070; font_rom[3999]=32'h20c420;
font_rom[4000]=32'he0cf00; font_rom[4001]=32'h3c0c7c0; font_rom[4002]=32'hf81c1f0; font_rom[4003]=32'h1e1f8070; font_rom[4004]=32'h81f0010; font_rom[4005]=32'h806000; font_rom[4006]=32'h1c06000; font_rom[4007]=32'h180e000;
font_rom[4008]=32'h380c000; font_rom[4009]=32'h301fff0; font_rom[4010]=32'h611fff0; font_rom[4011]=32'h63b8030; font_rom[4012]=32'hc330030; font_rom[4013]=32'h18760030; font_rom[4014]=32'h1fee0030; font_rom[4015]=32'h1fc44030;
font_rom[4016]=32'h180e030; font_rom[4017]=32'h3806030; font_rom[4018]=32'h3007030; font_rom[4019]=32'h6003830; font_rom[4020]=32'hef81830; font_rom[4021]=32'h1ff81c30; font_rom[4022]=32'hf000830; font_rom[4023]=32'h30;
font_rom[4024]=32'h30; font_rom[4025]=32'h3c0030; font_rom[4026]=32'h7fc0070; font_rom[4027]=32'h1fc00070; font_rom[4028]=32'h18007fe0; font_rom[4029]=32'h3fc0; font_rom[4030]=32'h1830300; font_rom[4031]=32'h1830300;
font_rom[4032]=32'h3830300; font_rom[4033]=32'h3030300; font_rom[4034]=32'h7030300; font_rom[4035]=32'h6030300; font_rom[4036]=32'hc230300; font_rom[4037]=32'hc630300; font_rom[4038]=32'h18e30300; font_rom[4039]=32'h1fc30300;
font_rom[4040]=32'h1f830300; font_rom[4041]=32'h1830380; font_rom[4042]=32'h3038380; font_rom[4043]=32'h607c380; font_rom[4044]=32'hc07c780; font_rom[4045]=32'hfe6e780; font_rom[4046]=32'h1fc676c0; font_rom[4047]=32'h1c0e26c0;
font_rom[4048]=32'hc0ec0; font_rom[4049]=32'h4c0c60; font_rom[4050]=32'h3dc1c60; font_rom[4051]=32'hf381870; font_rom[4052]=32'h1c383830; font_rom[4053]=32'h10707038; font_rom[4054]=32'he0601c; font_rom[4055]=32'h402008;
font_rom[4056]=32'h800000; font_rom[4057]=32'h1c7ffc0; font_rom[4058]=32'h187ffc0; font_rom[4059]=32'h38001c0; font_rom[4060]=32'h3000380; font_rom[4061]=32'h6200700; font_rom[4062]=32'h6300e00; font_rom[4063]=32'hc603e00;
font_rom[4064]=32'h1860f7c0; font_rom[4065]=32'h1fc3e1f0; font_rom[4066]=32'h1f9f0078; font_rom[4067]=32'h30c0018; font_rom[4068]=32'h6000000; font_rom[4069]=32'hff00000; font_rom[4070]=32'h1fe7ffe0; font_rom[4071]=32'h1e07ffe0;
font_rom[4072]=32'h1800; font_rom[4073]=32'h1800; font_rom[4074]=32'h1800; font_rom[4075]=32'h701800; font_rom[4076]=32'h7f01800; font_rom[4077]=32'h1f801800; font_rom[4078]=32'h181ffff8; font_rom[4079]=32'h1ffff8;
font_rom[4080]=32'h1800c00; font_rom[4081]=32'h1800c00; font_rom[4082]=32'h1800c00; font_rom[4083]=32'h3000c00; font_rom[4084]=32'h3000c00; font_rom[4085]=32'h60ffff8; font_rom[4086]=32'h60ffff8; font_rom[4087]=32'hc200c00;
font_rom[4088]=32'h1c700c00; font_rom[4089]=32'h18e00c00; font_rom[4090]=32'h1fc00c00; font_rom[4091]=32'h1fc7fff8; font_rom[4092]=32'h187fff0; font_rom[4093]=32'h3000000; font_rom[4094]=32'h7000000; font_rom[4095]=32'h6000000;
font_rom[4096]=32'hff7fff0; font_rom[4097]=32'h1ff7fff0; font_rom[4098]=32'h1f070030; font_rom[4099]=32'h70030; font_rom[4100]=32'h70030; font_rom[4101]=32'h770030; font_rom[4102]=32'hff70030; font_rom[4103]=32'h1f87fff0;
font_rom[4104]=32'h1807fff0; font_rom[4105]=32'h70030; font_rom[4106]=32'h70030; font_rom[4107]=32'h1807000; font_rom[4108]=32'h1807000; font_rom[4109]=32'h3007020; font_rom[4110]=32'h3007070; font_rom[4111]=32'h607ff60;
font_rom[4112]=32'h607ffc0; font_rom[4113]=32'hc3071c0; font_rom[4114]=32'h1c307380; font_rom[4115]=32'h18607700; font_rom[4116]=32'h1feffff8; font_rom[4117]=32'h1fcffff8; font_rom[4118]=32'h1803800; font_rom[4119]=32'h3007000;
font_rom[4120]=32'h601e000; font_rom[4121]=32'hc63ffe0; font_rom[4122]=32'h1fefffe0; font_rom[4123]=32'h1f9f8060; font_rom[4124]=32'h18118060; font_rom[4125]=32'h1ffe0; font_rom[4126]=32'h1ffe0; font_rom[4127]=32'h318060;
font_rom[4128]=32'h7f18060; font_rom[4129]=32'h1fc18060; font_rom[4130]=32'h1c01ffe0; font_rom[4131]=32'h1ffe0; font_rom[4132]=32'h18060; font_rom[4133]=32'hffffff0; font_rom[4134]=32'he1c3870; font_rom[4135]=32'hc081030;
font_rom[4136]=32'hffffff0; font_rom[4137]=32'he000070; font_rom[4138]=32'h6000000; font_rom[4139]=32'h6060000; font_rom[4140]=32'h1fff7fe0; font_rom[4141]=32'he0e3fe0; font_rom[4142]=32'h6060060; font_rom[4143]=32'h7fe3060;
font_rom[4144]=32'h6663060; font_rom[4145]=32'h603060; font_rom[4146]=32'hfff2060; font_rom[4147]=32'hc633ff0; font_rom[4148]=32'h8633ff0; font_rom[4149]=32'hfff0030; font_rom[4150]=32'hc630030; font_rom[4151]=32'h637fb0;
font_rom[4152]=32'h607f30; font_rom[4153]=32'h1fff8030; font_rom[4154]=32'he00030; font_rom[4155]=32'h600030; font_rom[4156]=32'h601ff0; font_rom[4157]=32'h601fe0; font_rom[4158]=32'h400200; font_rom[4159]=32'h700e00;
font_rom[4160]=32'h381c00; font_rom[4161]=32'h181800; font_rom[4162]=32'h1ffffff8; font_rom[4163]=32'h1ffffff8; font_rom[4164]=32'h18000; font_rom[4165]=32'h18000; font_rom[4166]=32'h7ffffe0; font_rom[4167]=32'h7ffffe0;
font_rom[4168]=32'h18000; font_rom[4169]=32'h18000; font_rom[4170]=32'h1ffffff8; font_rom[4171]=32'h1ffffff8; font_rom[4172]=32'h0; font_rom[4173]=32'h18000; font_rom[4174]=32'h18000; font_rom[4175]=32'h18000;
font_rom[4176]=32'h1ffffff8; font_rom[4177]=32'h1ffffff8; font_rom[4178]=32'h3c000; font_rom[4179]=32'h66000; font_rom[4180]=32'h1c3800; font_rom[4181]=32'h781e00; font_rom[4182]=32'h3f00fc0; font_rom[4183]=32'h1fc003f8;
font_rom[4184]=32'h1c000078; font_rom[4185]=32'hc000; font_rom[4186]=32'hc000; font_rom[4187]=32'hff1c000; font_rom[4188]=32'hff1fff8; font_rom[4189]=32'hc33fff0; font_rom[4190]=32'hc370000; font_rom[4191]=32'hc3ffff0;
font_rom[4192]=32'hc37fff0; font_rom[4193]=32'hc330030; font_rom[4194]=32'hff30030; font_rom[4195]=32'hff3fff0; font_rom[4196]=32'hc33fff0; font_rom[4197]=32'hc330030; font_rom[4198]=32'hc330030; font_rom[4199]=32'hc33fff0;
font_rom[4200]=32'hc31fff0; font_rom[4201]=32'hff0c030; font_rom[4202]=32'hff1c000; font_rom[4203]=32'hc31fff0; font_rom[4204]=32'hc33ffe0; font_rom[4205]=32'hc37c0e0; font_rom[4206]=32'hc3e61c0; font_rom[4207]=32'hc343f80;
font_rom[4208]=32'hc301f00; font_rom[4209]=32'h1830ffc0; font_rom[4210]=32'h1be7e1f8; font_rom[4211]=32'h13e70038; font_rom[4212]=32'h20000; font_rom[4213]=32'h70000; font_rom[4214]=32'h60000; font_rom[4215]=32'h60000;
font_rom[4216]=32'h7ffffe0; font_rom[4217]=32'h7ffffe0; font_rom[4218]=32'h70000e0; font_rom[4219]=32'h70000e0; font_rom[4220]=32'h70000e0; font_rom[4221]=32'h70000e0; font_rom[4222]=32'h70000e0; font_rom[4223]=32'h7ffffe0;
font_rom[4224]=32'h7ffffe0; font_rom[4225]=32'h70000e0; font_rom[4226]=32'h70000e0; font_rom[4227]=32'h70000e0; font_rom[4228]=32'h70000e0; font_rom[4229]=32'h7ffffe0; font_rom[4230]=32'h7ffffe0; font_rom[4231]=32'h70000e0;
font_rom[4232]=32'h70000e0; font_rom[4233]=32'h70000e0; font_rom[4234]=32'h70000e0; font_rom[4235]=32'h70000e0; font_rom[4236]=32'h7ffffe0; font_rom[4237]=32'h7ffffe0; font_rom[4238]=32'h70000e0; font_rom[4239]=32'h70000e0;
font_rom[4240]=32'h1c000; font_rom[4241]=32'h3e000; font_rom[4242]=32'hf7000; font_rom[4243]=32'h1e3800; font_rom[4244]=32'h381c00; font_rom[4245]=32'hf00f00; font_rom[4246]=32'h3e003c0; font_rom[4247]=32'hffffff0;
font_rom[4248]=32'h1e7ffe78; font_rom[4249]=32'h18018018; font_rom[4250]=32'h18000; font_rom[4251]=32'h18000; font_rom[4252]=32'h1ffffff8; font_rom[4253]=32'h1ffffff8; font_rom[4254]=32'h18000; font_rom[4255]=32'h18000;
font_rom[4256]=32'h18000; font_rom[4257]=32'h1ffff80; font_rom[4258]=32'h1ffff80; font_rom[4259]=32'h1800180; font_rom[4260]=32'h1800180; font_rom[4261]=32'h1800180; font_rom[4262]=32'h1800180; font_rom[4263]=32'h1800180;
font_rom[4264]=32'h1ffff80; font_rom[4265]=32'h1ffff80; font_rom[4266]=32'h1800180; font_rom[4267]=32'h30000; font_rom[4268]=32'h70000; font_rom[4269]=32'h60000; font_rom[4270]=32'h1ffff80; font_rom[4271]=32'h1ffff80;
font_rom[4272]=32'h1800180; font_rom[4273]=32'h1820180; font_rom[4274]=32'h1870180; font_rom[4275]=32'h1838180; font_rom[4276]=32'h181c180; font_rom[4277]=32'h180c180; font_rom[4278]=32'h1804180; font_rom[4279]=32'h1800180;
font_rom[4280]=32'h1ffffffc; font_rom[4281]=32'h1ffffffc; font_rom[4282]=32'h1800180; font_rom[4283]=32'h1820180; font_rom[4284]=32'h1870180; font_rom[4285]=32'h1838180; font_rom[4286]=32'h381e180; font_rom[4287]=32'h3004180;
font_rom[4288]=32'h3000180; font_rom[4289]=32'h7000180; font_rom[4290]=32'h6000180; font_rom[4291]=32'he007f80; font_rom[4292]=32'h1c003f00; font_rom[4293]=32'h8000000; font_rom[4294]=32'h10000; font_rom[4295]=32'h30000;
font_rom[4296]=32'h18000; font_rom[4297]=32'h18000; font_rom[4298]=32'h3ffffc0; font_rom[4299]=32'h3ffffc0; font_rom[4300]=32'h30000c0; font_rom[4301]=32'h30000c0; font_rom[4302]=32'h30000c0; font_rom[4303]=32'h3ffffc0;
font_rom[4304]=32'h3ffffc0; font_rom[4305]=32'h30000c0; font_rom[4306]=32'h30000c0; font_rom[4307]=32'h30000c0; font_rom[4308]=32'h3ffffc0; font_rom[4309]=32'h3ffffc0; font_rom[4310]=32'h3060020; font_rom[4311]=32'h3030070;
font_rom[4312]=32'h30301e0; font_rom[4313]=32'h3018380; font_rom[4314]=32'h301cf00; font_rom[4315]=32'h300fc00; font_rom[4316]=32'h3007800; font_rom[4317]=32'h3023c00; font_rom[4318]=32'h73e0f00; font_rom[4319]=32'h7fc07e0;
font_rom[4320]=32'h7c001f8; font_rom[4321]=32'h6000038; font_rom[4322]=32'h300c00; font_rom[4323]=32'h300c00; font_rom[4324]=32'h300c00; font_rom[4325]=32'h300c00; font_rom[4326]=32'h1ffffff8; font_rom[4327]=32'h1ffffff8;
font_rom[4328]=32'h300c00; font_rom[4329]=32'h300c00; font_rom[4330]=32'h300c00; font_rom[4331]=32'h0; font_rom[4332]=32'h0; font_rom[4333]=32'hfffffc0; font_rom[4334]=32'hfffffc0; font_rom[4335]=32'h1800c0;
font_rom[4336]=32'h1800c0; font_rom[4337]=32'h1800c0; font_rom[4338]=32'h1800c0; font_rom[4339]=32'h1800c0; font_rom[4340]=32'h1800c0; font_rom[4341]=32'h1800c0; font_rom[4342]=32'h1801c0; font_rom[4343]=32'h1801c0;
font_rom[4344]=32'h187f80; font_rom[4345]=32'h187f00; font_rom[4346]=32'h180000; font_rom[4347]=32'h180000; font_rom[4348]=32'h180000; font_rom[4349]=32'h180000; font_rom[4350]=32'h300c00; font_rom[4351]=32'h300c00;
font_rom[4352]=32'h300c00; font_rom[4353]=32'h1ffffff8; font_rom[4354]=32'h1ffffff8; font_rom[4355]=32'h300c00; font_rom[4356]=32'h300c00; font_rom[4357]=32'h318c00; font_rom[4358]=32'h18000; font_rom[4359]=32'h18000;
font_rom[4360]=32'h1ffffff8; font_rom[4361]=32'h1ffffff8; font_rom[4362]=32'h18000; font_rom[4363]=32'h18000; font_rom[4364]=32'h18000; font_rom[4365]=32'h3ffffc0; font_rom[4366]=32'h3ffffc0; font_rom[4367]=32'h30000c0;
font_rom[4368]=32'h30000c0; font_rom[4369]=32'h30000c0; font_rom[4370]=32'h30000c0; font_rom[4371]=32'h30000c0; font_rom[4372]=32'h30000c0; font_rom[4373]=32'h3ffffc0; font_rom[4374]=32'h3ffffc0; font_rom[4375]=32'h30000c0;
font_rom[4376]=32'h30000c0; font_rom[4377]=32'h300c00; font_rom[4378]=32'h300c00; font_rom[4379]=32'h1ffffff8; font_rom[4380]=32'h1ffffff8; font_rom[4381]=32'h300c00; font_rom[4382]=32'h4310c00; font_rom[4383]=32'he030000;
font_rom[4384]=32'h307ffc0; font_rom[4385]=32'h187ffc0; font_rom[4386]=32'h8f01c0; font_rom[4387]=32'h101b8380; font_rom[4388]=32'h1839c700; font_rom[4389]=32'he307c00; font_rom[4390]=32'h7207c00; font_rom[4391]=32'h201efc0;
font_rom[4392]=32'hf83fc; font_rom[4393]=32'h7c0038; font_rom[4394]=32'h32fffc0; font_rom[4395]=32'h30fffc0; font_rom[4396]=32'h70c00c0; font_rom[4397]=32'h60c00c0; font_rom[4398]=32'h60c00c0; font_rom[4399]=32'he0c00c0;
font_rom[4400]=32'hc0fffc0; font_rom[4401]=32'h1c0fffc0; font_rom[4402]=32'h80c00c0; font_rom[4403]=32'h30000; font_rom[4404]=32'h30000; font_rom[4405]=32'h3fff0; font_rom[4406]=32'h3ffe0; font_rom[4407]=32'h30000;
font_rom[4408]=32'h30000; font_rom[4409]=32'h7fffff8; font_rom[4410]=32'h7fffff8; font_rom[4411]=32'h6030018; font_rom[4412]=32'h6031f18; font_rom[4413]=32'h6ffff00; font_rom[4414]=32'h6ff0060; font_rom[4415]=32'h6030060;
font_rom[4416]=32'h603ffe0; font_rom[4417]=32'h601ff80; font_rom[4418]=32'h6000000; font_rom[4419]=32'h6431840; font_rom[4420]=32'h6e318e0; font_rom[4421]=32'h67319c0; font_rom[4422]=32'he331980; font_rom[4423]=32'hc131900;
font_rom[4424]=32'hc031800; font_rom[4425]=32'h1c031800; font_rom[4426]=32'h19fffff8; font_rom[4427]=32'h39fffff8; font_rom[4428]=32'h10000000; font_rom[4429]=32'h1804040; font_rom[4430]=32'h180e0c0; font_rom[4431]=32'h18060c0;
font_rom[4432]=32'h1802180; font_rom[4433]=32'h183fff8; font_rom[4434]=32'h1ff3fff8; font_rom[4435]=32'h1ff30c18; font_rom[4436]=32'h19930c18; font_rom[4437]=32'h19930c18; font_rom[4438]=32'h1993fff8; font_rom[4439]=32'h1993fff8;
font_rom[4440]=32'h19930c18; font_rom[4441]=32'h19930c18; font_rom[4442]=32'h19930c18; font_rom[4443]=32'h1ff3fff8; font_rom[4444]=32'h1ff3fff8; font_rom[4445]=32'h19830c18; font_rom[4446]=32'h1800c00; font_rom[4447]=32'h1b00c00;
font_rom[4448]=32'h1b7fffc; font_rom[4449]=32'h1b7fffc; font_rom[4450]=32'h1b00c00; font_rom[4451]=32'h7f80c00; font_rom[4452]=32'h3ff80c00; font_rom[4453]=32'h1c180c00; font_rom[4454]=32'h180c00; font_rom[4455]=32'hc00;
font_rom[4456]=32'h300; font_rom[4457]=32'h1fff8300; font_rom[4458]=32'h1fff0300; font_rom[4459]=32'h300; font_rom[4460]=32'h300; font_rom[4461]=32'h7fe3ff0; font_rom[4462]=32'h7fe3ff0; font_rom[4463]=32'h4023330;
font_rom[4464]=32'h4023330; font_rom[4465]=32'h7fe3330; font_rom[4466]=32'h7fe3330; font_rom[4467]=32'h4023330; font_rom[4468]=32'h3330; font_rom[4469]=32'hfffb330; font_rom[4470]=32'hfffbff0; font_rom[4471]=32'h909bff0;
font_rom[4472]=32'h919b300; font_rom[4473]=32'h9998300; font_rom[4474]=32'h8b18330; font_rom[4475]=32'hffd8330; font_rom[4476]=32'hbfd8330; font_rom[4477]=32'h8618310; font_rom[4478]=32'h8618318; font_rom[4479]=32'h86183f8;
font_rom[4480]=32'h8617ff8; font_rom[4481]=32'h87f7c18; font_rom[4482]=32'h80e0008; font_rom[4483]=32'h400000; font_rom[4484]=32'h600000; font_rom[4485]=32'he3fff0; font_rom[4486]=32'h1c3fff0; font_rom[4487]=32'h3800000;
font_rom[4488]=32'h7000000; font_rom[4489]=32'he000000; font_rom[4490]=32'h1c200000; font_rom[4491]=32'h18700000; font_rom[4492]=32'h10e00000; font_rom[4493]=32'hcffffc; font_rom[4494]=32'h1cffffc; font_rom[4495]=32'h3800300;
font_rom[4496]=32'h7800300; font_rom[4497]=32'hf800300; font_rom[4498]=32'h1d800300; font_rom[4499]=32'h19800300; font_rom[4500]=32'h11800300; font_rom[4501]=32'h1800300; font_rom[4502]=32'h1800300; font_rom[4503]=32'h1800300;
font_rom[4504]=32'h1800300; font_rom[4505]=32'h1800300; font_rom[4506]=32'h1800700; font_rom[4507]=32'h180ff00; font_rom[4508]=32'h180fe00; font_rom[4509]=32'h18000; font_rom[4510]=32'h18000; font_rom[4511]=32'h18000;
font_rom[4512]=32'hffffff8; font_rom[4513]=32'hffffff0; font_rom[4514]=32'h18000; font_rom[4515]=32'h18000; font_rom[4516]=32'h18000; font_rom[4517]=32'h7ffffe0; font_rom[4518]=32'h7ffffe0; font_rom[4519]=32'h18000;
font_rom[4520]=32'h18000; font_rom[4521]=32'h1ffffff8; font_rom[4522]=32'h1ffffff8; font_rom[4523]=32'h78000; font_rom[4524]=32'hec060; font_rom[4525]=32'h1cc0e0; font_rom[4526]=32'h3861c0; font_rom[4527]=32'h703780;
font_rom[4528]=32'h1f03e00; font_rom[4529]=32'h3b01c00; font_rom[4530]=32'hf300e00; font_rom[4531]=32'h1c306780; font_rom[4532]=32'h1833e3e0; font_rom[4533]=32'h3fc0f8; font_rom[4534]=32'h3e0030; font_rom[4535]=32'h300000;
font_rom[4536]=32'h10000c0; font_rom[4537]=32'h38000c0; font_rom[4538]=32'h18000c0; font_rom[4539]=32'h18000c0; font_rom[4540]=32'hc0; font_rom[4541]=32'h1ff7fffc; font_rom[4542]=32'h1ff7fffc; font_rom[4543]=32'h6000c0;
font_rom[4544]=32'he000c0; font_rom[4545]=32'hc000c0; font_rom[4546]=32'h18080c0; font_rom[4547]=32'h388c0c0; font_rom[4548]=32'h79ce0c0; font_rom[4549]=32'hfb860c0; font_rom[4550]=32'h1df070c0; font_rom[4551]=32'h19e030c0;
font_rom[4552]=32'h11f820c0; font_rom[4553]=32'h1bc00c0; font_rom[4554]=32'h18800c0; font_rom[4555]=32'h18000c0; font_rom[4556]=32'h18000c0; font_rom[4557]=32'h18000c0; font_rom[4558]=32'h18000c0; font_rom[4559]=32'h18001c0;
font_rom[4560]=32'h1803f80; font_rom[4561]=32'h1803f00; font_rom[4562]=32'h1800000; font_rom[4563]=32'h1800000; font_rom[4564]=32'h300fff0; font_rom[4565]=32'h3f8fff0; font_rom[4566]=32'h7f81830; font_rom[4567]=32'h6381830;
font_rom[4568]=32'hc303030; font_rom[4569]=32'h18603070; font_rom[4570]=32'h1ffee7e0; font_rom[4571]=32'h1fffc7c0; font_rom[4572]=32'hc468000; font_rom[4573]=32'hc463300; font_rom[4574]=32'hc463300; font_rom[4575]=32'hffe6300;
font_rom[4576]=32'hffe7ff8; font_rom[4577]=32'hc467ff8; font_rom[4578]=32'hc46c300; font_rom[4579]=32'hc46c300; font_rom[4580]=32'hffec300; font_rom[4581]=32'hffe0300; font_rom[4582]=32'hc47fff8; font_rom[4583]=32'hc46fff8;
font_rom[4584]=32'hc460300; font_rom[4585]=32'h1c460300; font_rom[4586]=32'h18060300; font_rom[4587]=32'h187c0300; font_rom[4588]=32'h10380300; font_rom[4589]=32'h4038180; font_rom[4590]=32'he038180; font_rom[4591]=32'h7038180;
font_rom[4592]=32'h3838180; font_rom[4593]=32'h1838180; font_rom[4594]=32'h11ffff8; font_rom[4595]=32'h1ffff8; font_rom[4596]=32'h38180; font_rom[4597]=32'h38180; font_rom[4598]=32'h3f038180; font_rom[4599]=32'h3f038180;
font_rom[4600]=32'h7030180; font_rom[4601]=32'h7030180; font_rom[4602]=32'h7030180; font_rom[4603]=32'h73ffff8; font_rom[4604]=32'h73ffff8; font_rom[4605]=32'h7030180; font_rom[4606]=32'h7030180; font_rom[4607]=32'h7030180;
font_rom[4608]=32'h7070180; font_rom[4609]=32'h7660180; font_rom[4610]=32'h7c60180; font_rom[4611]=32'h78c0180; font_rom[4612]=32'h71c0180; font_rom[4613]=32'h4380180; font_rom[4614]=32'h300180; font_rom[4615]=32'h100180;
font_rom[4616]=32'h2000000; font_rom[4617]=32'h703ff80; font_rom[4618]=32'h383ff80; font_rom[4619]=32'h3c30180; font_rom[4620]=32'h1c30180; font_rom[4621]=32'h830180; font_rom[4622]=32'h70180; font_rom[4623]=32'h701c0;
font_rom[4624]=32'he00fc; font_rom[4625]=32'h1f1c00f8; font_rom[4626]=32'h1f180000; font_rom[4627]=32'h3000000; font_rom[4628]=32'h30fffe0; font_rom[4629]=32'h30fffe0; font_rom[4630]=32'h30700c0; font_rom[4631]=32'h30301c0;
font_rom[4632]=32'h3038180; font_rom[4633]=32'h301c300; font_rom[4634]=32'h310e700; font_rom[4635]=32'h3707e00; font_rom[4636]=32'h3f07c00; font_rom[4637]=32'h3c0fe00; font_rom[4638]=32'h783ef80; font_rom[4639]=32'h21f83f0;
font_rom[4640]=32'h7e00f8; font_rom[4641]=32'h300010; font_rom[4642]=32'h4000000; font_rom[4643]=32'h60001f0; font_rom[4644]=32'h707fff0; font_rom[4645]=32'h387fe00; font_rom[4646]=32'h1c60000; font_rom[4647]=32'h1c60000;
font_rom[4648]=32'h860000; font_rom[4649]=32'h60000; font_rom[4650]=32'h60000; font_rom[4651]=32'h1f07fff8; font_rom[4652]=32'h1f07fff8; font_rom[4653]=32'h3060600; font_rom[4654]=32'h3060600; font_rom[4655]=32'h3060600;
font_rom[4656]=32'h3060600; font_rom[4657]=32'h3064600; font_rom[4658]=32'h306fe00; font_rom[4659]=32'h3263f00; font_rom[4660]=32'h3660fc0; font_rom[4661]=32'h3ee07f8; font_rom[4662]=32'h3cc0678; font_rom[4663]=32'h78c0610;
font_rom[4664]=32'h71c0600; font_rom[4665]=32'h2180600; font_rom[4666]=32'h380600; font_rom[4667]=32'h300600; font_rom[4668]=32'h200600; font_rom[4669]=32'h4000000; font_rom[4670]=32'hc1ffff0; font_rom[4671]=32'he1ffff0;
font_rom[4672]=32'h7000030; font_rom[4673]=32'h3800030; font_rom[4674]=32'h1800030; font_rom[4675]=32'h1000030; font_rom[4676]=32'h3fff30; font_rom[4677]=32'h3fff30; font_rom[4678]=32'h3f000030; font_rom[4679]=32'h3f000030;
font_rom[4680]=32'h3000030; font_rom[4681]=32'h30ffc30; font_rom[4682]=32'h30ffc30; font_rom[4683]=32'h30c0c30; font_rom[4684]=32'h30c0c30; font_rom[4685]=32'h30c0c30; font_rom[4686]=32'h30c0c30; font_rom[4687]=32'h30c0c30;
font_rom[4688]=32'h30ffc30; font_rom[4689]=32'h32ffc30; font_rom[4690]=32'h36c0030; font_rom[4691]=32'h7ec0030; font_rom[4692]=32'h7800030; font_rom[4693]=32'h7000ff0; font_rom[4694]=32'h2000fe0; font_rom[4695]=32'h4000000;
font_rom[4696]=32'he1ffff0; font_rom[4697]=32'h71ffff0; font_rom[4698]=32'h380c000; font_rom[4699]=32'h380c000; font_rom[4700]=32'h101c000; font_rom[4701]=32'h1fffc0; font_rom[4702]=32'h1fffc0; font_rom[4703]=32'h180c0;
font_rom[4704]=32'h3f0180c0; font_rom[4705]=32'h3f0180c0; font_rom[4706]=32'h33ffffc; font_rom[4707]=32'h33ffff8; font_rom[4708]=32'h3000000; font_rom[4709]=32'h3000000; font_rom[4710]=32'h30fffe0; font_rom[4711]=32'h30fffe0;
font_rom[4712]=32'h30c0060; font_rom[4713]=32'h30c0060; font_rom[4714]=32'h32c0060; font_rom[4715]=32'h36c0060; font_rom[4716]=32'h7ec0060; font_rom[4717]=32'h7cfffe0; font_rom[4718]=32'h70fffe0; font_rom[4719]=32'h60c0060;
font_rom[4720]=32'h4020100; font_rom[4721]=32'he060180; font_rom[4722]=32'h7030380; font_rom[4723]=32'h3038300; font_rom[4724]=32'h3818600; font_rom[4725]=32'h1c08e00; font_rom[4726]=32'h8fffe0; font_rom[4727]=32'hfffe0;
font_rom[4728]=32'hc0060; font_rom[4729]=32'hc0060; font_rom[4730]=32'h3f0c0060; font_rom[4731]=32'h3f0c0060; font_rom[4732]=32'h70c0060; font_rom[4733]=32'h70c0060; font_rom[4734]=32'h70fffe0; font_rom[4735]=32'h70fffe0;
font_rom[4736]=32'h70cce60; font_rom[4737]=32'h701ce00; font_rom[4738]=32'h7018e00; font_rom[4739]=32'h7318e18; font_rom[4740]=32'h7e18e18; font_rom[4741]=32'h7c38e18; font_rom[4742]=32'h7870e18; font_rom[4743]=32'h70e0618;
font_rom[4744]=32'h21c07f8; font_rom[4745]=32'h7803f0; font_rom[4746]=32'h200000; font_rom[4747]=32'h20060; font_rom[4748]=32'h4060060; font_rom[4749]=32'he060060; font_rom[4750]=32'h63fe060; font_rom[4751]=32'h73fe060;
font_rom[4752]=32'h3b06060; font_rom[4753]=32'h1306060; font_rom[4754]=32'h3feff8; font_rom[4755]=32'h3feff8; font_rom[4756]=32'h306060; font_rom[4757]=32'h3e306060; font_rom[4758]=32'h3e3fec60; font_rom[4759]=32'h63fec60;
font_rom[4760]=32'h6306660; font_rom[4761]=32'h6306660; font_rom[4762]=32'h6306660; font_rom[4763]=32'h6ffe360; font_rom[4764]=32'h67fe260; font_rom[4765]=32'h601e060; font_rom[4766]=32'h6436060; font_rom[4767]=32'h6c66060;
font_rom[4768]=32'h7cc6060; font_rom[4769]=32'hf1c6060; font_rom[4770]=32'h4706060; font_rom[4771]=32'he06060; font_rom[4772]=32'hc7e7c0; font_rom[4773]=32'h7c7c0; font_rom[4774]=32'h818100; font_rom[4775]=32'h1c18380;
font_rom[4776]=32'hf18700; font_rom[4777]=32'h618400; font_rom[4778]=32'hffffff0; font_rom[4779]=32'hffffff0; font_rom[4780]=32'hc000030; font_rom[4781]=32'hc000030; font_rom[4782]=32'hcffff30; font_rom[4783]=32'hffff00;
font_rom[4784]=32'hc00300; font_rom[4785]=32'hffff00; font_rom[4786]=32'hffff00; font_rom[4787]=32'h0; font_rom[4788]=32'h0; font_rom[4789]=32'h3ffffc0; font_rom[4790]=32'h3ffffc0; font_rom[4791]=32'h30000c0;
font_rom[4792]=32'h30180c0; font_rom[4793]=32'h30180c0; font_rom[4794]=32'h30180c0; font_rom[4795]=32'h30360c0; font_rom[4796]=32'h3077cc0; font_rom[4797]=32'h3e1f00; font_rom[4798]=32'h3f803e0; font_rom[4799]=32'h1fe000f0;
font_rom[4800]=32'he000030; font_rom[4801]=32'h7fffff8; font_rom[4802]=32'h7fffff8; font_rom[4803]=32'h6000000; font_rom[4804]=32'h6000000; font_rom[4805]=32'h6000000; font_rom[4806]=32'h63fffe0; font_rom[4807]=32'h63fffe0;
font_rom[4808]=32'h6000000; font_rom[4809]=32'h6000000; font_rom[4810]=32'h6000000; font_rom[4811]=32'h6000000; font_rom[4812]=32'h7fffff8; font_rom[4813]=32'h7fffff8; font_rom[4814]=32'h638c020; font_rom[4815]=32'h638e070;
font_rom[4816]=32'h63861e0; font_rom[4817]=32'h63873c0; font_rom[4818]=32'h6383f00; font_rom[4819]=32'h6381e00; font_rom[4820]=32'hc381c00; font_rom[4821]=32'hc384e00; font_rom[4822]=32'h1c3bc780; font_rom[4823]=32'h183fc1f0;
font_rom[4824]=32'h383e0078; font_rom[4825]=32'h10100018; font_rom[4826]=32'h4000000; font_rom[4827]=32'he0ffff0; font_rom[4828]=32'h70ffff0; font_rom[4829]=32'h3800000; font_rom[4830]=32'h3800000; font_rom[4831]=32'h1000000;
font_rom[4832]=32'h0; font_rom[4833]=32'h0; font_rom[4834]=32'h3ffff8; font_rom[4835]=32'h1f3ffff8; font_rom[4836]=32'h1f00e000; font_rom[4837]=32'h300c000; font_rom[4838]=32'h301c200; font_rom[4839]=32'h3018700;
font_rom[4840]=32'h3030300; font_rom[4841]=32'h3070180; font_rom[4842]=32'h30601c0; font_rom[4843]=32'h30e07e0; font_rom[4844]=32'h31fffe0; font_rom[4845]=32'h31ff070; font_rom[4846]=32'h7880020; font_rom[4847]=32'hfc00000;
font_rom[4848]=32'h1cf00000; font_rom[4849]=32'h187ffff8; font_rom[4850]=32'h101ffff8; font_rom[4851]=32'h2000; font_rom[4852]=32'h4007000; font_rom[4853]=32'he003800; font_rom[4854]=32'h7001000; font_rom[4855]=32'h3bffff8;
font_rom[4856]=32'h1bffff8; font_rom[4857]=32'h1000000; font_rom[4858]=32'h80; font_rom[4859]=32'h1c0; font_rom[4860]=32'h1f0c0180; font_rom[4861]=32'h1f070380; font_rom[4862]=32'h3038300; font_rom[4863]=32'h301c600;
font_rom[4864]=32'h300fe00; font_rom[4865]=32'h3007c00; font_rom[4866]=32'h3003c00; font_rom[4867]=32'h3007e00; font_rom[4868]=32'h301e780; font_rom[4869]=32'h303c3c0; font_rom[4870]=32'h31f01e0; font_rom[4871]=32'h33c0070;
font_rom[4872]=32'h7100020; font_rom[4873]=32'hf800000; font_rom[4874]=32'h1ce00000; font_rom[4875]=32'h187ffff8; font_rom[4876]=32'h101ffff8; font_rom[4877]=32'h4003800; font_rom[4878]=32'he007800; font_rom[4879]=32'h6006c00;
font_rom[4880]=32'h700ce00; font_rom[4881]=32'h3818700; font_rom[4882]=32'h18383c0; font_rom[4883]=32'h10f01e0; font_rom[4884]=32'h1e00f8; font_rom[4885]=32'h3fffb8; font_rom[4886]=32'h33ff80; font_rom[4887]=32'h1f003000;
font_rom[4888]=32'h1f003000; font_rom[4889]=32'h3003000; font_rom[4890]=32'h31ffff0; font_rom[4891]=32'h31ffff0; font_rom[4892]=32'h3003000; font_rom[4893]=32'h3023100; font_rom[4894]=32'h3063380; font_rom[4895]=32'h30e31c0;
font_rom[4896]=32'h31c30e0; font_rom[4897]=32'h3383070; font_rom[4898]=32'h713f020; font_rom[4899]=32'hf83e000; font_rom[4900]=32'h1ce00000; font_rom[4901]=32'h187ffff8; font_rom[4902]=32'h101ffff8; font_rom[4903]=32'h1000;
font_rom[4904]=32'h4003800; font_rom[4905]=32'hc001800; font_rom[4906]=32'he1ffff0; font_rom[4907]=32'h71ffff0; font_rom[4908]=32'h3980030; font_rom[4909]=32'h1180030; font_rom[4910]=32'h1ffff0; font_rom[4911]=32'h1ffff0;
font_rom[4912]=32'h180030; font_rom[4913]=32'h1f180000; font_rom[4914]=32'h1f1ffff0; font_rom[4915]=32'h31ffff0; font_rom[4916]=32'h3363310; font_rom[4917]=32'h3363310; font_rom[4918]=32'h3363310; font_rom[4919]=32'h337fff0;
font_rom[4920]=32'h367fff0; font_rom[4921]=32'h3663310; font_rom[4922]=32'h3e63310; font_rom[4923]=32'h3463310; font_rom[4924]=32'h70633f0; font_rom[4925]=32'hf8600f0; font_rom[4926]=32'h1de00000; font_rom[4927]=32'h18fffff8;
font_rom[4928]=32'h101ffff8; font_rom[4929]=32'h1fff1ff8; font_rom[4930]=32'h1fff1ff8; font_rom[4931]=32'h1831838; font_rom[4932]=32'h1831830; font_rom[4933]=32'h1831830; font_rom[4934]=32'h1831860; font_rom[4935]=32'h1831860;
font_rom[4936]=32'h1fff18e0; font_rom[4937]=32'h1fff18c0; font_rom[4938]=32'h18318c0; font_rom[4939]=32'h18319c0; font_rom[4940]=32'h18318e0; font_rom[4941]=32'h1831870; font_rom[4942]=32'h1831838; font_rom[4943]=32'h1fff1818;
font_rom[4944]=32'h1fff1818; font_rom[4945]=32'h3031818; font_rom[4946]=32'h3031818; font_rom[4947]=32'h3031a38; font_rom[4948]=32'h7031bf0; font_rom[4949]=32'h6071be0; font_rom[4950]=32'he061800; font_rom[4951]=32'h1cfe1800;
font_rom[4952]=32'h18fc1800; font_rom[4953]=32'h10001800; font_rom[4954]=32'h600000; font_rom[4955]=32'h600000; font_rom[4956]=32'h701ff8; font_rom[4957]=32'h301ff8; font_rom[4958]=32'h1fffd818; font_rom[4959]=32'h1fffd830;
font_rom[4960]=32'h2021830; font_rom[4961]=32'h7071860; font_rom[4962]=32'h3061860; font_rom[4963]=32'h38c18c0; font_rom[4964]=32'h10c18c0; font_rom[4965]=32'h1fffd9c0; font_rom[4966]=32'h1fffd8e0; font_rom[4967]=32'h1870;
font_rom[4968]=32'h1838; font_rom[4969]=32'h1818; font_rom[4970]=32'h7ff1818; font_rom[4971]=32'h7ff1818; font_rom[4972]=32'h6031818; font_rom[4973]=32'h6031818; font_rom[4974]=32'h6031a38; font_rom[4975]=32'h6031bf0;
font_rom[4976]=32'h6031be0; font_rom[4977]=32'h7ff1800; font_rom[4978]=32'h7ff1800; font_rom[4979]=32'h6031800; font_rom[4980]=32'h6031800; font_rom[4981]=32'hc00000; font_rom[4982]=32'hc00000; font_rom[4983]=32'hc01ff8;
font_rom[4984]=32'hc09ff8; font_rom[4985]=32'h1ffdd838; font_rom[4986]=32'hfff9830; font_rom[4987]=32'hc31830; font_rom[4988]=32'hc71860; font_rom[4989]=32'hce1860; font_rom[4990]=32'h1fffd8c0; font_rom[4991]=32'h1fffd8c0;
font_rom[4992]=32'h7018c0; font_rom[4993]=32'he01860; font_rom[4994]=32'h1c01870; font_rom[4995]=32'h7ff1838; font_rom[4996]=32'hfff1818; font_rom[4997]=32'h1f031818; font_rom[4998]=32'h1f031818; font_rom[4999]=32'h3ff1818;
font_rom[5000]=32'h3ff1818; font_rom[5001]=32'h3031a38; font_rom[5002]=32'h3031bf0; font_rom[5003]=32'h3031be0; font_rom[5004]=32'h3ff1800; font_rom[5005]=32'h3ff1800; font_rom[5006]=32'h3031800; font_rom[5007]=32'h3031800;
font_rom[5008]=32'hc000000; font_rom[5009]=32'he3ffff8; font_rom[5010]=32'h73ffff8; font_rom[5011]=32'h380c600; font_rom[5012]=32'h80c600; font_rom[5013]=32'hc600; font_rom[5014]=32'h3ffff0; font_rom[5015]=32'h103ffff0;
font_rom[5016]=32'h1c30c630; font_rom[5017]=32'he30c630; font_rom[5018]=32'h7318630; font_rom[5019]=32'h2318630; font_rom[5020]=32'h338630; font_rom[5021]=32'h3307f0; font_rom[5022]=32'h23f03f0; font_rom[5023]=32'h3be0030;
font_rom[5024]=32'h3300030; font_rom[5025]=32'h73ffff0; font_rom[5026]=32'h73ffff0; font_rom[5027]=32'h6300030; font_rom[5028]=32'h6300030; font_rom[5029]=32'he300030; font_rom[5030]=32'hc3ffff0; font_rom[5031]=32'h1c3ffff0;
font_rom[5032]=32'h4300030; font_rom[5033]=32'h1ffefff0; font_rom[5034]=32'h1ffefff0; font_rom[5035]=32'h120c030; font_rom[5036]=32'h120c030; font_rom[5037]=32'h120fff0; font_rom[5038]=32'h120fff0; font_rom[5039]=32'h1ffec030;
font_rom[5040]=32'h1ffec030; font_rom[5041]=32'h1926fff0; font_rom[5042]=32'h19267ff0; font_rom[5043]=32'h19264600; font_rom[5044]=32'h1b266600; font_rom[5045]=32'h1b26c600; font_rom[5046]=32'h1f3efff8; font_rom[5047]=32'h1e1efff0;
font_rom[5048]=32'h18078600; font_rom[5049]=32'h18078600; font_rom[5050]=32'h1ffe0600; font_rom[5051]=32'h1ffefff0; font_rom[5052]=32'h1806fff0; font_rom[5053]=32'h18060600; font_rom[5054]=32'h18060600; font_rom[5055]=32'h1ffe0600;
font_rom[5056]=32'h1ffffffc; font_rom[5057]=32'h1807fff8; font_rom[5058]=32'h18060000; font_rom[5059]=32'h780000; font_rom[5060]=32'h1ffbfff0; font_rom[5061]=32'h1fc3fff0; font_rom[5062]=32'h180c070; font_rom[5063]=32'h1988c060;
font_rom[5064]=32'h199860c0; font_rom[5065]=32'hd9831c0; font_rom[5066]=32'hdb03b80; font_rom[5067]=32'hdb01f00; font_rom[5068]=32'h1803f00; font_rom[5069]=32'h1ff8fbf0; font_rom[5070]=32'h1ffbecf8; font_rom[5071]=32'h1830c18;
font_rom[5072]=32'h3800c00; font_rom[5073]=32'h3e00c00; font_rom[5074]=32'h7f3fff0; font_rom[5075]=32'h7bbfff0; font_rom[5076]=32'hd980c00; font_rom[5077]=32'hd880c00; font_rom[5078]=32'h19800c00; font_rom[5079]=32'h1987fffc;
font_rom[5080]=32'h1187fffc; font_rom[5081]=32'h1800c00; font_rom[5082]=32'h1800c00; font_rom[5083]=32'h1800c00; font_rom[5084]=32'h1800c00; font_rom[5085]=32'h1800c00; font_rom[5086]=32'h7ffffe0; font_rom[5087]=32'h7ffffe0;
font_rom[5088]=32'h6018060; font_rom[5089]=32'h6018060; font_rom[5090]=32'h6018060; font_rom[5091]=32'h7ffffe0; font_rom[5092]=32'h7ffffe0; font_rom[5093]=32'h6018060; font_rom[5094]=32'h6018060; font_rom[5095]=32'h6018060;
font_rom[5096]=32'h6018060; font_rom[5097]=32'h7ffffe0; font_rom[5098]=32'h7ffffe0; font_rom[5099]=32'h6018060; font_rom[5100]=32'h18000; font_rom[5101]=32'h18000; font_rom[5102]=32'hffffff0; font_rom[5103]=32'hffffff0;
font_rom[5104]=32'h18000; font_rom[5105]=32'h18000; font_rom[5106]=32'h18000; font_rom[5107]=32'h3ffffffc; font_rom[5108]=32'h3ffffffc; font_rom[5109]=32'h1e0; font_rom[5110]=32'hfffffe0; font_rom[5111]=32'h7ffc000;
font_rom[5112]=32'h18000; font_rom[5113]=32'h1ffffff8; font_rom[5114]=32'h1ffffff8; font_rom[5115]=32'h18000; font_rom[5116]=32'h3ffffc0; font_rom[5117]=32'h3ffffc0; font_rom[5118]=32'h30180c0; font_rom[5119]=32'h30180c0;
font_rom[5120]=32'h3ffffc0; font_rom[5121]=32'h30180c0; font_rom[5122]=32'h30180c0; font_rom[5123]=32'h30180c0; font_rom[5124]=32'h3ffffc0; font_rom[5125]=32'h3ffffc0; font_rom[5126]=32'h30180c0; font_rom[5127]=32'h18000;
font_rom[5128]=32'h7ffffe0; font_rom[5129]=32'h7ffffe0; font_rom[5130]=32'h18000; font_rom[5131]=32'h3ffffff8; font_rom[5132]=32'h1ffffff8; font_rom[5133]=32'h3001c00; font_rom[5134]=32'h3001c00; font_rom[5135]=32'h6003c00;
font_rom[5136]=32'h6003600; font_rom[5137]=32'h7f07600; font_rom[5138]=32'hff06300; font_rom[5139]=32'hc00c180; font_rom[5140]=32'h1801c180; font_rom[5141]=32'h180390c0; font_rom[5142]=32'h17e31860; font_rom[5143]=32'h17ee1c38;
font_rom[5144]=32'h30c0c1c; font_rom[5145]=32'h3080408; font_rom[5146]=32'h3000000; font_rom[5147]=32'h307ffe0; font_rom[5148]=32'h3ff7ffe0; font_rom[5149]=32'h1ff000e0; font_rom[5150]=32'h30000c0; font_rom[5151]=32'h3000180;
font_rom[5152]=32'h3018380; font_rom[5153]=32'h311c700; font_rom[5154]=32'h330ee00; font_rom[5155]=32'h3f03c00; font_rom[5156]=32'h3e01c00; font_rom[5157]=32'h3c00f00; font_rom[5158]=32'h3000380; font_rom[5159]=32'h1000100;
font_rom[5160]=32'hc00000; font_rom[5161]=32'hc00080; font_rom[5162]=32'hc003c0; font_rom[5163]=32'hc00780; font_rom[5164]=32'hc01e00; font_rom[5165]=32'hc03c00; font_rom[5166]=32'hc0f000; font_rom[5167]=32'hc7c000;
font_rom[5168]=32'hcf0000; font_rom[5169]=32'hcc0000; font_rom[5170]=32'hc00000; font_rom[5171]=32'h1ffffff8; font_rom[5172]=32'h1ffffff8; font_rom[5173]=32'h1c18000; font_rom[5174]=32'hc1c000; font_rom[5175]=32'hc0c000;
font_rom[5176]=32'hc0e000; font_rom[5177]=32'hc07000; font_rom[5178]=32'hc03800; font_rom[5179]=32'hc01c00; font_rom[5180]=32'hc01e00; font_rom[5181]=32'hc10700; font_rom[5182]=32'hc703c0; font_rom[5183]=32'h1ff01f0;
font_rom[5184]=32'h1fc0078; font_rom[5185]=32'h1e00010; font_rom[5186]=32'h800000; font_rom[5187]=32'h1000000; font_rom[5188]=32'h3800000; font_rom[5189]=32'h1c7fff0; font_rom[5190]=32'he7fff0; font_rom[5191]=32'h400070;
font_rom[5192]=32'h70; font_rom[5193]=32'h6000070; font_rom[5194]=32'h6000070; font_rom[5195]=32'h6000070; font_rom[5196]=32'h6000070; font_rom[5197]=32'h6000070; font_rom[5198]=32'h6000070; font_rom[5199]=32'h6000070;
font_rom[5200]=32'h6000070; font_rom[5201]=32'h6000070; font_rom[5202]=32'h6000070; font_rom[5203]=32'h6000070; font_rom[5204]=32'h6000070; font_rom[5205]=32'h6000070; font_rom[5206]=32'h6000070; font_rom[5207]=32'h6000070;
font_rom[5208]=32'h6000070; font_rom[5209]=32'h6000070; font_rom[5210]=32'h6000060; font_rom[5211]=32'h6001fe0; font_rom[5212]=32'h6001fc0; font_rom[5213]=32'h6000000; font_rom[5214]=32'h2000000; font_rom[5215]=32'h3000000;
font_rom[5216]=32'h38ffff0; font_rom[5217]=32'h1cffff0; font_rom[5218]=32'h800030; font_rom[5219]=32'h30; font_rom[5220]=32'hc401f30; font_rom[5221]=32'hcefff30; font_rom[5222]=32'hc77f030; font_rom[5223]=32'hc206030;
font_rom[5224]=32'hd006030; font_rom[5225]=32'hd8fffb0; font_rom[5226]=32'hdcfffb0; font_rom[5227]=32'hcc06030; font_rom[5228]=32'hc406030; font_rom[5229]=32'hc006030; font_rom[5230]=32'hc47ff30; font_rom[5231]=32'hc67ff30;
font_rom[5232]=32'hc660330; font_rom[5233]=32'hcc60330; font_rom[5234]=32'hcc60330; font_rom[5235]=32'hcc60330; font_rom[5236]=32'hd87ff30; font_rom[5237]=32'hd87fe30; font_rom[5238]=32'hd860030; font_rom[5239]=32'hc0003f0;
font_rom[5240]=32'hc0003e0; font_rom[5241]=32'h2000000; font_rom[5242]=32'h3000000; font_rom[5243]=32'h38ffff0; font_rom[5244]=32'h1cffff0; font_rom[5245]=32'h800030; font_rom[5246]=32'hc004030; font_rom[5247]=32'hcfec430;
font_rom[5248]=32'hcfe6e30; font_rom[5249]=32'hc067830; font_rom[5250]=32'hc443130; font_rom[5251]=32'hcec33b0; font_rom[5252]=32'hc781e30; font_rom[5253]=32'hc3ffc30; font_rom[5254]=32'hc67ee30; font_rom[5255]=32'hcc18730;
font_rom[5256]=32'hd8181f0; font_rom[5257]=32'hd3ffcb0; font_rom[5258]=32'hc3ffc30; font_rom[5259]=32'hc030030; font_rom[5260]=32'hc07c030; font_rom[5261]=32'hc0ef030; font_rom[5262]=32'hc3c3c30; font_rom[5263]=32'hcf00e30;
font_rom[5264]=32'hcc00230; font_rom[5265]=32'hc000030; font_rom[5266]=32'hc0007f0; font_rom[5267]=32'hc0003c0; font_rom[5268]=32'h8400; font_rom[5269]=32'h18e00; font_rom[5270]=32'h18600; font_rom[5271]=32'h1fe18700;
font_rom[5272]=32'h1fe38200; font_rom[5273]=32'h63fff8; font_rom[5274]=32'h67fff8; font_rom[5275]=32'h8670600; font_rom[5276]=32'h1c670600; font_rom[5277]=32'hc6f0600; font_rom[5278]=32'hcff0600; font_rom[5279]=32'h6dbfff0;
font_rom[5280]=32'h6dbfff0; font_rom[5281]=32'h3c30600; font_rom[5282]=32'h3830600; font_rom[5283]=32'h1830600; font_rom[5284]=32'h1830600; font_rom[5285]=32'h3c3fff0; font_rom[5286]=32'h3c3fff0; font_rom[5287]=32'h7e30600;
font_rom[5288]=32'h6630600; font_rom[5289]=32'he730600; font_rom[5290]=32'hc330600; font_rom[5291]=32'h1823fff8; font_rom[5292]=32'h1803fff8; font_rom[5293]=32'h10030000; font_rom[5294]=32'h30000; font_rom[5295]=32'h1ffffff8;
font_rom[5296]=32'h1ffffff8; font_rom[5297]=32'h18000; font_rom[5298]=32'h18000; font_rom[5299]=32'h18000; font_rom[5300]=32'hffffff0; font_rom[5301]=32'hffffff0; font_rom[5302]=32'hc018030; font_rom[5303]=32'hc018030;
font_rom[5304]=32'hcc19030; font_rom[5305]=32'hce19c30; font_rom[5306]=32'hc718e30; font_rom[5307]=32'hc198730; font_rom[5308]=32'hc098130; font_rom[5309]=32'hc018030; font_rom[5310]=32'hc819030; font_rom[5311]=32'hce1b830;
font_rom[5312]=32'hc718e30; font_rom[5313]=32'hc398730; font_rom[5314]=32'hc098230; font_rom[5315]=32'hc018030; font_rom[5316]=32'hc018070; font_rom[5317]=32'hc018fe0; font_rom[5318]=32'hc018fc0; font_rom[5319]=32'h7ffffe0;
font_rom[5320]=32'h18000; font_rom[5321]=32'h18000; font_rom[5322]=32'h1ffffff8; font_rom[5323]=32'h1c018038; font_rom[5324]=32'h18018018; font_rom[5325]=32'h19fdbf98; font_rom[5326]=32'h18018018; font_rom[5327]=32'h18018018;
font_rom[5328]=32'h1fdbf80; font_rom[5329]=32'h18000; font_rom[5330]=32'h0; font_rom[5331]=32'h600600; font_rom[5332]=32'h600600; font_rom[5333]=32'h600600; font_rom[5334]=32'h1ffefff0; font_rom[5335]=32'h1ffefff0;
font_rom[5336]=32'hf80f00; font_rom[5337]=32'h1fe3f80; font_rom[5338]=32'h36476c0; font_rom[5339]=32'he61c670; font_rom[5340]=32'h1c638638; font_rom[5341]=32'h10620608; font_rom[5342]=32'h600600; font_rom[5343]=32'h600600;
font_rom[5344]=32'h7ffffe0; font_rom[5345]=32'h18000; font_rom[5346]=32'hffffff8; font_rom[5347]=32'hc018038; font_rom[5348]=32'h9fdbf98; font_rom[5349]=32'h8018018; font_rom[5350]=32'h9fdbf98; font_rom[5351]=32'h0;
font_rom[5352]=32'h40fffe0; font_rom[5353]=32'he0c0060; font_rom[5354]=32'h30c0060; font_rom[5355]=32'h18fffe0; font_rom[5356]=32'h10c0060; font_rom[5357]=32'hc0060; font_rom[5358]=32'h1f0fffe0; font_rom[5359]=32'h1f060000;
font_rom[5360]=32'h31ffff0; font_rom[5361]=32'h3703030; font_rom[5362]=32'h3683830; font_rom[5363]=32'h308ef30; font_rom[5364]=32'h3098130; font_rom[5365]=32'h3efff30; font_rom[5366]=32'h7c00070; font_rom[5367]=32'h7001fe0;
font_rom[5368]=32'h2000f80; font_rom[5369]=32'h1ffffff8; font_rom[5370]=32'h1ffffff8; font_rom[5371]=32'h30000; font_rom[5372]=32'h30000; font_rom[5373]=32'h60000; font_rom[5374]=32'hffffff0; font_rom[5375]=32'hffffff0;
font_rom[5376]=32'hc181870; font_rom[5377]=32'hc181870; font_rom[5378]=32'hc181870; font_rom[5379]=32'hc181870; font_rom[5380]=32'hc1ff870; font_rom[5381]=32'hc1ff870; font_rom[5382]=32'hc181870; font_rom[5383]=32'hc181870;
font_rom[5384]=32'hc181870; font_rom[5385]=32'hc1ff870; font_rom[5386]=32'hc1ff870; font_rom[5387]=32'hc181870; font_rom[5388]=32'hc181870; font_rom[5389]=32'hc181870; font_rom[5390]=32'hffffff0; font_rom[5391]=32'hffffff0;
font_rom[5392]=32'hc000070; font_rom[5393]=32'h7ffff80; font_rom[5394]=32'h7ffff80; font_rom[5395]=32'h6000180; font_rom[5396]=32'h6001180; font_rom[5397]=32'h6203980; font_rom[5398]=32'h6703180; font_rom[5399]=32'h6307180;
font_rom[5400]=32'h6186180; font_rom[5401]=32'h61ce180; font_rom[5402]=32'h60ec180; font_rom[5403]=32'h607c180; font_rom[5404]=32'h6078180; font_rom[5405]=32'h6038180; font_rom[5406]=32'h6078180; font_rom[5407]=32'h607c180;
font_rom[5408]=32'h60ee180; font_rom[5409]=32'h61c6188; font_rom[5410]=32'h638719c; font_rom[5411]=32'he303998; font_rom[5412]=32'hc7019d8; font_rom[5413]=32'hce01dd8; font_rom[5414]=32'h1c4008d8; font_rom[5415]=32'h180000f8;
font_rom[5416]=32'h18000070; font_rom[5417]=32'h10000000; font_rom[5418]=32'h3004000; font_rom[5419]=32'h300c000; font_rom[5420]=32'h300c000; font_rom[5421]=32'h600c000; font_rom[5422]=32'h7f9fff8; font_rom[5423]=32'h7f9fff8;
font_rom[5424]=32'hc338038; font_rom[5425]=32'hc331830; font_rom[5426]=32'h1c671870; font_rom[5427]=32'h1b661860; font_rom[5428]=32'h3b0e18e0; font_rom[5429]=32'h13041800; font_rom[5430]=32'h3001c00; font_rom[5431]=32'h3001c00;
font_rom[5432]=32'h3003c00; font_rom[5433]=32'h3003e00; font_rom[5434]=32'h3003600; font_rom[5435]=32'h3007700; font_rom[5436]=32'h3006300; font_rom[5437]=32'h310e380; font_rom[5438]=32'h331c1c0; font_rom[5439]=32'h3f380e0;
font_rom[5440]=32'h7c70070; font_rom[5441]=32'h71e0038; font_rom[5442]=32'h21c0018; font_rom[5443]=32'h180000; font_rom[5444]=32'h3006200; font_rom[5445]=32'h3006700; font_rom[5446]=32'h6006380; font_rom[5447]=32'h60061c0;
font_rom[5448]=32'h7f06080; font_rom[5449]=32'hff06000; font_rom[5450]=32'hc3060f0; font_rom[5451]=32'h1830fff0; font_rom[5452]=32'h187fff00; font_rom[5453]=32'h336f6000; font_rom[5454]=32'h13606000; font_rom[5455]=32'h3407038;
font_rom[5456]=32'h3003ff8; font_rom[5457]=32'h303ffc0; font_rom[5458]=32'h31ff020; font_rom[5459]=32'h31c3060; font_rom[5460]=32'h30030e0; font_rom[5461]=32'h30039c0; font_rom[5462]=32'h3001b80; font_rom[5463]=32'h3001e00;
font_rom[5464]=32'h3001c18; font_rom[5465]=32'h3307c1c; font_rom[5466]=32'h3e1ee18; font_rom[5467]=32'h3c78718; font_rom[5468]=32'h39f03b8; font_rom[5469]=32'h70803f0; font_rom[5470]=32'h20000f0; font_rom[5471]=32'h200400;
font_rom[5472]=32'h700e00; font_rom[5473]=32'h380c00; font_rom[5474]=32'h181800; font_rom[5475]=32'h1ffffff8; font_rom[5476]=32'h1ffffff8; font_rom[5477]=32'h30000; font_rom[5478]=32'h30000; font_rom[5479]=32'h30000;
font_rom[5480]=32'h3ffffc0; font_rom[5481]=32'h3ffffc0; font_rom[5482]=32'h30000c0; font_rom[5483]=32'h30000c0; font_rom[5484]=32'h3ffffc0; font_rom[5485]=32'h3ffffc0; font_rom[5486]=32'h30000c0; font_rom[5487]=32'h30000c0;
font_rom[5488]=32'h30000c0; font_rom[5489]=32'h3ffffc0; font_rom[5490]=32'h3ffffc0; font_rom[5491]=32'h30000c0; font_rom[5492]=32'h30000c0; font_rom[5493]=32'h30000c0; font_rom[5494]=32'h3ffffc0; font_rom[5495]=32'h3ffffc0;
font_rom[5496]=32'h30000c0; font_rom[5497]=32'h30000c0; font_rom[5498]=32'h1feffc00; font_rom[5499]=32'h1feffff8; font_rom[5500]=32'h6613f8; font_rom[5501]=32'hc67f018; font_rom[5502]=32'hc463230; font_rom[5503]=32'hc461330;
font_rom[5504]=32'hcc7f1e0; font_rom[5505]=32'h8c630e0; font_rom[5506]=32'h8c610e0; font_rom[5507]=32'h18cff9f0; font_rom[5508]=32'h18cff718; font_rom[5509]=32'h1fe01208; font_rom[5510]=32'h1fe007e0; font_rom[5511]=32'h67ffc0;
font_rom[5512]=32'h671c00; font_rom[5513]=32'h7e01c18; font_rom[5514]=32'h3f60de78; font_rom[5515]=32'h18639be0; font_rom[5516]=32'h6f1980; font_rom[5517]=32'h6c59c0; font_rom[5518]=32'h60d870; font_rom[5519]=32'h639838;
font_rom[5520]=32'hfce1818; font_rom[5521]=32'hf881800; font_rom[5522]=32'h10000; font_rom[5523]=32'h38000; font_rom[5524]=32'h18000; font_rom[5525]=32'h1ffffff8; font_rom[5526]=32'h1ffffff8; font_rom[5527]=32'h0;
font_rom[5528]=32'h1ffff80; font_rom[5529]=32'h1ffff80; font_rom[5530]=32'h1800180; font_rom[5531]=32'h1800180; font_rom[5532]=32'h1ffff80; font_rom[5533]=32'h1ffff80; font_rom[5534]=32'h1800180; font_rom[5535]=32'h0;
font_rom[5536]=32'hffffff0; font_rom[5537]=32'hffffff0; font_rom[5538]=32'hc000030; font_rom[5539]=32'hc000030; font_rom[5540]=32'hc3ffc30; font_rom[5541]=32'hc3ffc30; font_rom[5542]=32'hc300c30; font_rom[5543]=32'hc300c30;
font_rom[5544]=32'hc3ffc30; font_rom[5545]=32'hc3ffc30; font_rom[5546]=32'hc300030; font_rom[5547]=32'hc0007f0; font_rom[5548]=32'hc0007e0; font_rom[5549]=32'hc00000; font_rom[5550]=32'hc00000; font_rom[5551]=32'h1c00000;
font_rom[5552]=32'h1800000; font_rom[5553]=32'h1800000; font_rom[5554]=32'h1800000; font_rom[5555]=32'h3000000; font_rom[5556]=32'hc00000; font_rom[5557]=32'h1e00000; font_rom[5558]=32'he00000; font_rom[5559]=32'h0;
font_rom[5560]=32'h0; font_rom[5561]=32'h0; font_rom[5562]=32'h0; font_rom[5563]=32'h0; font_rom[5564]=32'h0; font_rom[5565]=32'h0; font_rom[5566]=32'h0; font_rom[5567]=32'h0;
font_rom[5568]=32'he00000; font_rom[5569]=32'h1e00000; font_rom[5570]=32'hc00000; font_rom[5571]=32'hc00000; font_rom[5572]=32'h1e00000; font_rom[5573]=32'he00000; font_rom[5574]=32'h0; font_rom[5575]=32'h0;
font_rom[5576]=32'h0; font_rom[5577]=32'h0; font_rom[5578]=32'h0; font_rom[5579]=32'h0; font_rom[5580]=32'h0; font_rom[5581]=32'h0; font_rom[5582]=32'h0; font_rom[5583]=32'hc00000;
font_rom[5584]=32'hc00000; font_rom[5585]=32'h1c00000; font_rom[5586]=32'h1800000; font_rom[5587]=32'h1800000; font_rom[5588]=32'h1800000; font_rom[5589]=32'h3000000; font_rom[5590]=32'h3f00000; font_rom[5591]=32'hffc0000;
font_rom[5592]=32'he3c0000; font_rom[5593]=32'h80e0000; font_rom[5594]=32'he0000; font_rom[5595]=32'he0000; font_rom[5596]=32'he0000; font_rom[5597]=32'hc0000; font_rom[5598]=32'h1c0000; font_rom[5599]=32'h380000;
font_rom[5600]=32'h700000; font_rom[5601]=32'he00000; font_rom[5602]=32'h1c00000; font_rom[5603]=32'h1c00000; font_rom[5604]=32'h1c00000; font_rom[5605]=32'h0; font_rom[5606]=32'h0; font_rom[5607]=32'h0;
font_rom[5608]=32'h1c00000; font_rom[5609]=32'h1c00000; font_rom[5610]=32'h1800000;
end
reg [23:0] sprite_span_rom [0:575]; /* fehdl force_ram=1, ram_style="bram" */
initial begin
sprite_span_rom[0]=24'h0; sprite_span_rom[1]=24'h0; sprite_span_rom[2]=24'h0; sprite_span_rom[3]=24'h0; sprite_span_rom[4]=24'h0; sprite_span_rom[5]=24'h0; sprite_span_rom[6]=24'h0; sprite_span_rom[7]=24'h0;
sprite_span_rom[8]=24'h0; sprite_span_rom[9]=24'h0; sprite_span_rom[10]=24'h0; sprite_span_rom[11]=24'h0; sprite_span_rom[12]=24'h0; sprite_span_rom[13]=24'h0; sprite_span_rom[14]=24'h0; sprite_span_rom[15]=24'h0;
sprite_span_rom[16]=24'h0; sprite_span_rom[17]=24'h0; sprite_span_rom[18]=24'h0; sprite_span_rom[19]=24'h0; sprite_span_rom[20]=24'h0; sprite_span_rom[21]=24'h0; sprite_span_rom[22]=24'h0; sprite_span_rom[23]=24'h0;
sprite_span_rom[24]=24'h0; sprite_span_rom[25]=24'h0; sprite_span_rom[26]=24'h0; sprite_span_rom[27]=24'h0; sprite_span_rom[28]=24'h0; sprite_span_rom[29]=24'h0; sprite_span_rom[30]=24'h0; sprite_span_rom[31]=24'h0;
sprite_span_rom[32]=24'h0; sprite_span_rom[33]=24'h0; sprite_span_rom[34]=24'h0; sprite_span_rom[35]=24'h0; sprite_span_rom[36]=24'h0; sprite_span_rom[37]=24'h0; sprite_span_rom[38]=24'h0; sprite_span_rom[39]=24'h0;
sprite_span_rom[40]=24'h0; sprite_span_rom[41]=24'h0; sprite_span_rom[42]=24'h0; sprite_span_rom[43]=24'h0; sprite_span_rom[44]=24'h0; sprite_span_rom[45]=24'h0; sprite_span_rom[46]=24'h0; sprite_span_rom[47]=24'h0;
sprite_span_rom[48]=24'h53; sprite_span_rom[49]=24'h1092; sprite_span_rom[50]=24'h3092; sprite_span_rom[51]=24'h50d1; sprite_span_rom[52]=24'h80d1; sprite_span_rom[53]=24'hb0d1; sprite_span_rom[54]=24'he10f; sprite_span_rom[55]=24'h1210f;
sprite_span_rom[56]=24'h160f0; sprite_span_rom[57]=24'h198f0; sprite_span_rom[58]=24'h1d10f; sprite_span_rom[59]=24'h2114e; sprite_span_rom[60]=24'h2614e; sprite_span_rom[61]=24'h2b14d; sprite_span_rom[62]=24'h3014d; sprite_span_rom[63]=24'h3516d;
sprite_span_rom[64]=24'h3a96d; sprite_span_rom[65]=24'h4018c; sprite_span_rom[66]=24'h4618c; sprite_span_rom[67]=24'h4c18c; sprite_span_rom[68]=24'h521ac; sprite_span_rom[69]=24'h589ac; sprite_span_rom[70]=24'h5f1ac; sprite_span_rom[71]=24'h659ac;
sprite_span_rom[72]=24'h6c1cb; sprite_span_rom[73]=24'h731cb; sprite_span_rom[74]=24'h7a1cb; sprite_span_rom[75]=24'h811cb; sprite_span_rom[76]=24'h881cb; sprite_span_rom[77]=24'h8f1cb; sprite_span_rom[78]=24'h961cb; sprite_span_rom[79]=24'h9d1cb;
sprite_span_rom[80]=24'ha41cb; sprite_span_rom[81]=24'hab18c; sprite_span_rom[82]=24'hb118c; sprite_span_rom[83]=24'hb718c; sprite_span_rom[84]=24'hbd16d; sprite_span_rom[85]=24'hc296d; sprite_span_rom[86]=24'hc814d; sprite_span_rom[87]=24'hcd14d;
sprite_span_rom[88]=24'hd214d; sprite_span_rom[89]=24'hd712d; sprite_span_rom[90]=24'hdb92d; sprite_span_rom[91]=24'he012d; sprite_span_rom[92]=24'he48d0; sprite_span_rom[93]=24'he78b0; sprite_span_rom[94]=24'hea090; sprite_span_rom[95]=24'hec0af;
sprite_span_rom[96]=24'hee8cf; sprite_span_rom[97]=24'hf18cf; sprite_span_rom[98]=24'hf48ee; sprite_span_rom[99]=24'hf80ee; sprite_span_rom[100]=24'hfb8ee; sprite_span_rom[101]=24'hff0ee; sprite_span_rom[102]=24'h10290e; sprite_span_rom[103]=24'h10692d;
sprite_span_rom[104]=24'h10b12d; sprite_span_rom[105]=24'h10f92d; sprite_span_rom[106]=24'h11412d; sprite_span_rom[107]=24'h11892d; sprite_span_rom[108]=24'h11d12d; sprite_span_rom[109]=24'h12192d; sprite_span_rom[110]=24'h12612d; sprite_span_rom[111]=24'h12a92d;
sprite_span_rom[112]=24'h12f12d; sprite_span_rom[113]=24'h13392d; sprite_span_rom[114]=24'h13812d; sprite_span_rom[115]=24'h13c92d; sprite_span_rom[116]=24'h14112d; sprite_span_rom[117]=24'h14592d; sprite_span_rom[118]=24'h14a12d; sprite_span_rom[119]=24'h14e92d;
sprite_span_rom[120]=24'h15312d; sprite_span_rom[121]=24'h15792d; sprite_span_rom[122]=24'h15c12d; sprite_span_rom[123]=24'h16092d; sprite_span_rom[124]=24'h16512d; sprite_span_rom[125]=24'h16992d; sprite_span_rom[126]=24'h16e12d; sprite_span_rom[127]=24'h17290e;
sprite_span_rom[128]=24'h17690e; sprite_span_rom[129]=24'h17a92e; sprite_span_rom[130]=24'h17f12e; sprite_span_rom[131]=24'h18394d; sprite_span_rom[132]=24'h18894d; sprite_span_rom[133]=24'h18d94d; sprite_span_rom[134]=24'h19294d; sprite_span_rom[135]=24'h19794d;
sprite_span_rom[136]=24'h19c94d; sprite_span_rom[137]=24'h1a194d; sprite_span_rom[138]=24'h1a694d; sprite_span_rom[139]=24'h1ab94d; sprite_span_rom[140]=24'h1b090e; sprite_span_rom[141]=24'h1b48ee; sprite_span_rom[142]=24'h1b80af; sprite_span_rom[143]=24'h0;
sprite_span_rom[144]=24'h0; sprite_span_rom[145]=24'h0; sprite_span_rom[146]=24'h0; sprite_span_rom[147]=24'h0; sprite_span_rom[148]=24'h0; sprite_span_rom[149]=24'h0; sprite_span_rom[150]=24'h0; sprite_span_rom[151]=24'h0;
sprite_span_rom[152]=24'h0; sprite_span_rom[153]=24'h0; sprite_span_rom[154]=24'h0; sprite_span_rom[155]=24'h0; sprite_span_rom[156]=24'h0; sprite_span_rom[157]=24'h0; sprite_span_rom[158]=24'h0; sprite_span_rom[159]=24'h0;
sprite_span_rom[160]=24'h0; sprite_span_rom[161]=24'h0; sprite_span_rom[162]=24'h0; sprite_span_rom[163]=24'h0; sprite_span_rom[164]=24'h0; sprite_span_rom[165]=24'h0; sprite_span_rom[166]=24'h0; sprite_span_rom[167]=24'h0;
sprite_span_rom[168]=24'h0; sprite_span_rom[169]=24'h0; sprite_span_rom[170]=24'h0; sprite_span_rom[171]=24'h0; sprite_span_rom[172]=24'h0; sprite_span_rom[173]=24'h0; sprite_span_rom[174]=24'h0; sprite_span_rom[175]=24'h0;
sprite_span_rom[176]=24'h0; sprite_span_rom[177]=24'h0; sprite_span_rom[178]=24'h0; sprite_span_rom[179]=24'h0; sprite_span_rom[180]=24'h0; sprite_span_rom[181]=24'h0; sprite_span_rom[182]=24'h0; sprite_span_rom[183]=24'h0;
sprite_span_rom[184]=24'h0; sprite_span_rom[185]=24'h0; sprite_span_rom[186]=24'h0; sprite_span_rom[187]=24'h0; sprite_span_rom[188]=24'h0; sprite_span_rom[189]=24'h0; sprite_span_rom[190]=24'h0; sprite_span_rom[191]=24'h0;
sprite_span_rom[192]=24'h0; sprite_span_rom[193]=24'h1ba870; sprite_span_rom[194]=24'h1bc090; sprite_span_rom[195]=24'h1be0af; sprite_span_rom[196]=24'h1c08cf; sprite_span_rom[197]=24'h1c38cf; sprite_span_rom[198]=24'h1c690d; sprite_span_rom[199]=24'h1ca90d;
sprite_span_rom[200]=24'h1ce8ee; sprite_span_rom[201]=24'h1d210d; sprite_span_rom[202]=24'h1d612c; sprite_span_rom[203]=24'h1da94b; sprite_span_rom[204]=24'h1df96b; sprite_span_rom[205]=24'h1e516b; sprite_span_rom[206]=24'h1ea96a; sprite_span_rom[207]=24'h1f016a;
sprite_span_rom[208]=24'h1f596a; sprite_span_rom[209]=24'h1fb189; sprite_span_rom[210]=24'h2011a9; sprite_span_rom[211]=24'h2079a9; sprite_span_rom[212]=24'h20e1a9; sprite_span_rom[213]=24'h2149a9; sprite_span_rom[214]=24'h21b1a9; sprite_span_rom[215]=24'h2219c8;
sprite_span_rom[216]=24'h2289c8; sprite_span_rom[217]=24'h22f9c8; sprite_span_rom[218]=24'h2369c8; sprite_span_rom[219]=24'h23d9c8; sprite_span_rom[220]=24'h2449c8; sprite_span_rom[221]=24'h24b9c8; sprite_span_rom[222]=24'h2529c8; sprite_span_rom[223]=24'h2599c8;
sprite_span_rom[224]=24'h2609a9; sprite_span_rom[225]=24'h2671a9; sprite_span_rom[226]=24'h26d9a9; sprite_span_rom[227]=24'h274189; sprite_span_rom[228]=24'h27a189; sprite_span_rom[229]=24'h280189; sprite_span_rom[230]=24'h286189; sprite_span_rom[231]=24'h28c169;
sprite_span_rom[232]=24'h291969; sprite_span_rom[233]=24'h297149; sprite_span_rom[234]=24'h29c168; sprite_span_rom[235]=24'h2a1948; sprite_span_rom[236]=24'h2a6948; sprite_span_rom[237]=24'h2ab967; sprite_span_rom[238]=24'h2b1167; sprite_span_rom[239]=24'h2b6908;
sprite_span_rom[240]=24'h2ba9a8; sprite_span_rom[241]=24'h2c11a8; sprite_span_rom[242]=24'h2c7988; sprite_span_rom[243]=24'h2cd988; sprite_span_rom[244]=24'h2d3988; sprite_span_rom[245]=24'h2d9988; sprite_span_rom[246]=24'h2df988; sprite_span_rom[247]=24'h2e5968;
sprite_span_rom[248]=24'h2eb168; sprite_span_rom[249]=24'h2f0968; sprite_span_rom[250]=24'h2f6168; sprite_span_rom[251]=24'h2fb968; sprite_span_rom[252]=24'h301168; sprite_span_rom[253]=24'h306968; sprite_span_rom[254]=24'h30c168; sprite_span_rom[255]=24'h311968;
sprite_span_rom[256]=24'h317188; sprite_span_rom[257]=24'h31d188; sprite_span_rom[258]=24'h323188; sprite_span_rom[259]=24'h329188; sprite_span_rom[260]=24'h32f188; sprite_span_rom[261]=24'h335188; sprite_span_rom[262]=24'h33b188; sprite_span_rom[263]=24'h341188;
sprite_span_rom[264]=24'h34712b; sprite_span_rom[265]=24'h34b94a; sprite_span_rom[266]=24'h35094a; sprite_span_rom[267]=24'h35594a; sprite_span_rom[268]=24'h35a94a; sprite_span_rom[269]=24'h35f94a; sprite_span_rom[270]=24'h36494a; sprite_span_rom[271]=24'h36992a;
sprite_span_rom[272]=24'h36e12a; sprite_span_rom[273]=24'h37292a; sprite_span_rom[274]=24'h37712a; sprite_span_rom[275]=24'h37b92a; sprite_span_rom[276]=24'h38012a; sprite_span_rom[277]=24'h38494a; sprite_span_rom[278]=24'h38994a; sprite_span_rom[279]=24'h38e94a;
sprite_span_rom[280]=24'h39394a; sprite_span_rom[281]=24'h39894a; sprite_span_rom[282]=24'h39d94a; sprite_span_rom[283]=24'h3a292a; sprite_span_rom[284]=24'h3a712a; sprite_span_rom[285]=24'h3ab8eb; sprite_span_rom[286]=24'h3af0cb; sprite_span_rom[287]=24'h0;
sprite_span_rom[288]=24'h0; sprite_span_rom[289]=24'h0; sprite_span_rom[290]=24'h0; sprite_span_rom[291]=24'h0; sprite_span_rom[292]=24'h0; sprite_span_rom[293]=24'h0; sprite_span_rom[294]=24'h0; sprite_span_rom[295]=24'h0;
sprite_span_rom[296]=24'h0; sprite_span_rom[297]=24'h0; sprite_span_rom[298]=24'h0; sprite_span_rom[299]=24'h0; sprite_span_rom[300]=24'h0; sprite_span_rom[301]=24'h0; sprite_span_rom[302]=24'h0; sprite_span_rom[303]=24'h0;
sprite_span_rom[304]=24'h0; sprite_span_rom[305]=24'h0; sprite_span_rom[306]=24'h0; sprite_span_rom[307]=24'h0; sprite_span_rom[308]=24'h0; sprite_span_rom[309]=24'h0; sprite_span_rom[310]=24'h0; sprite_span_rom[311]=24'h0;
sprite_span_rom[312]=24'h0; sprite_span_rom[313]=24'h0; sprite_span_rom[314]=24'h0; sprite_span_rom[315]=24'h0; sprite_span_rom[316]=24'h0; sprite_span_rom[317]=24'h0; sprite_span_rom[318]=24'h0; sprite_span_rom[319]=24'h0;
sprite_span_rom[320]=24'h0; sprite_span_rom[321]=24'h0; sprite_span_rom[322]=24'h0; sprite_span_rom[323]=24'h0; sprite_span_rom[324]=24'h0; sprite_span_rom[325]=24'h0; sprite_span_rom[326]=24'h0; sprite_span_rom[327]=24'h0;
sprite_span_rom[328]=24'h0; sprite_span_rom[329]=24'h0; sprite_span_rom[330]=24'h0; sprite_span_rom[331]=24'h0; sprite_span_rom[332]=24'h0; sprite_span_rom[333]=24'h0; sprite_span_rom[334]=24'h0; sprite_span_rom[335]=24'h0;
sprite_span_rom[336]=24'h3b2057; sprite_span_rom[337]=24'h3b3096; sprite_span_rom[338]=24'h3b5096; sprite_span_rom[339]=24'h3b70b6; sprite_span_rom[340]=24'h3b98b6; sprite_span_rom[341]=24'h3bc0f4; sprite_span_rom[342]=24'h3bf8f4; sprite_span_rom[343]=24'h3c30d5;
sprite_span_rom[344]=24'h3c6113; sprite_span_rom[345]=24'h3ca132; sprite_span_rom[346]=24'h3ce932; sprite_span_rom[347]=24'h3d3171; sprite_span_rom[348]=24'h3d8991; sprite_span_rom[349]=24'h3de951; sprite_span_rom[350]=24'h3e3970; sprite_span_rom[351]=24'h3e9190;
sprite_span_rom[352]=24'h3ef190; sprite_span_rom[353]=24'h3f5190; sprite_span_rom[354]=24'h3fb190; sprite_span_rom[355]=24'h4011af; sprite_span_rom[356]=24'h4079af; sprite_span_rom[357]=24'h40e1cf; sprite_span_rom[358]=24'h4151cf; sprite_span_rom[359]=24'h41c1cf;
sprite_span_rom[360]=24'h4231ee; sprite_span_rom[361]=24'h42a9ee; sprite_span_rom[362]=24'h4321ee; sprite_span_rom[363]=24'h439ae6; sprite_span_rom[364]=24'h4452e6; sprite_span_rom[365]=24'h450ac7; sprite_span_rom[366]=24'h45baa7; sprite_span_rom[367]=24'h466288;
sprite_span_rom[368]=24'h470288; sprite_span_rom[369]=24'h47a269; sprite_span_rom[370]=24'h483a69; sprite_span_rom[371]=24'h48d22a; sprite_span_rom[372]=24'h495a2a; sprite_span_rom[373]=24'h49e20b; sprite_span_rom[374]=24'h4a61eb; sprite_span_rom[375]=24'h4ad9eb;
sprite_span_rom[376]=24'h4b51cb; sprite_span_rom[377]=24'h4bc1ac; sprite_span_rom[378]=24'h4c298d; sprite_span_rom[379]=24'h4c898d; sprite_span_rom[380]=24'h4ce94d; sprite_span_rom[381]=24'h4d394d; sprite_span_rom[382]=24'h4d896d; sprite_span_rom[383]=24'h4de16d;
sprite_span_rom[384]=24'h4e396d; sprite_span_rom[385]=24'h4e916d; sprite_span_rom[386]=24'h4ee98d; sprite_span_rom[387]=24'h4f498d; sprite_span_rom[388]=24'h4fa98d; sprite_span_rom[389]=24'h50098d; sprite_span_rom[390]=24'h5069ad; sprite_span_rom[391]=24'h50d1ad;
sprite_span_rom[392]=24'h5139ad; sprite_span_rom[393]=24'h51a1ad; sprite_span_rom[394]=24'h5209ad; sprite_span_rom[395]=24'h5271ad; sprite_span_rom[396]=24'h52d9ad; sprite_span_rom[397]=24'h5341ad; sprite_span_rom[398]=24'h53a98e; sprite_span_rom[399]=24'h54098e;
sprite_span_rom[400]=24'h54698e; sprite_span_rom[401]=24'h54c931; sprite_span_rom[402]=24'h551131; sprite_span_rom[403]=24'h555931; sprite_span_rom[404]=24'h55a131; sprite_span_rom[405]=24'h55e931; sprite_span_rom[406]=24'h563131; sprite_span_rom[407]=24'h567931;
sprite_span_rom[408]=24'h56c111; sprite_span_rom[409]=24'h570111; sprite_span_rom[410]=24'h574130; sprite_span_rom[411]=24'h578930; sprite_span_rom[412]=24'h57d150; sprite_span_rom[413]=24'h582150; sprite_span_rom[414]=24'h587150; sprite_span_rom[415]=24'h58c150;
sprite_span_rom[416]=24'h591150; sprite_span_rom[417]=24'h596150; sprite_span_rom[418]=24'h59b150; sprite_span_rom[419]=24'h5a0150; sprite_span_rom[420]=24'h5a5170; sprite_span_rom[421]=24'h5aa970; sprite_span_rom[422]=24'h5b0150; sprite_span_rom[423]=24'h5b5150;
sprite_span_rom[424]=24'h5ba150; sprite_span_rom[425]=24'h5bf111; sprite_span_rom[426]=24'h5c30d2; sprite_span_rom[427]=24'h5c6055; sprite_span_rom[428]=24'h0; sprite_span_rom[429]=24'h0; sprite_span_rom[430]=24'h0; sprite_span_rom[431]=24'h0;
sprite_span_rom[432]=24'h0; sprite_span_rom[433]=24'h0; sprite_span_rom[434]=24'h0; sprite_span_rom[435]=24'h0; sprite_span_rom[436]=24'h0; sprite_span_rom[437]=24'h0; sprite_span_rom[438]=24'h0; sprite_span_rom[439]=24'h0;
sprite_span_rom[440]=24'h0; sprite_span_rom[441]=24'h0; sprite_span_rom[442]=24'h0; sprite_span_rom[443]=24'h0; sprite_span_rom[444]=24'h0; sprite_span_rom[445]=24'h0; sprite_span_rom[446]=24'h0; sprite_span_rom[447]=24'h0;
sprite_span_rom[448]=24'h0; sprite_span_rom[449]=24'h0; sprite_span_rom[450]=24'h0; sprite_span_rom[451]=24'h0; sprite_span_rom[452]=24'h0; sprite_span_rom[453]=24'h0; sprite_span_rom[454]=24'h0; sprite_span_rom[455]=24'h0;
sprite_span_rom[456]=24'h0; sprite_span_rom[457]=24'h0; sprite_span_rom[458]=24'h0; sprite_span_rom[459]=24'h0; sprite_span_rom[460]=24'h0; sprite_span_rom[461]=24'h0; sprite_span_rom[462]=24'h0; sprite_span_rom[463]=24'h0;
sprite_span_rom[464]=24'h0; sprite_span_rom[465]=24'h0; sprite_span_rom[466]=24'h0; sprite_span_rom[467]=24'h0; sprite_span_rom[468]=24'h0; sprite_span_rom[469]=24'h0; sprite_span_rom[470]=24'h0; sprite_span_rom[471]=24'h0;
sprite_span_rom[472]=24'h0; sprite_span_rom[473]=24'h0; sprite_span_rom[474]=24'h0; sprite_span_rom[475]=24'h0; sprite_span_rom[476]=24'h0; sprite_span_rom[477]=24'h0; sprite_span_rom[478]=24'h0; sprite_span_rom[479]=24'h0;
sprite_span_rom[480]=24'h0; sprite_span_rom[481]=24'h0; sprite_span_rom[482]=24'h0; sprite_span_rom[483]=24'h0; sprite_span_rom[484]=24'h0; sprite_span_rom[485]=24'h5c7073; sprite_span_rom[486]=24'h5c8aa2; sprite_span_rom[487]=24'h5d32a2;
sprite_span_rom[488]=24'h5ddaa2; sprite_span_rom[489]=24'h5e82c2; sprite_span_rom[490]=24'h5f32a3; sprite_span_rom[491]=24'h5fdaa3; sprite_span_rom[492]=24'h6082a3; sprite_span_rom[493]=24'h612aa3; sprite_span_rom[494]=24'h61d284; sprite_span_rom[495]=24'h627264;
sprite_span_rom[496]=24'h630a64; sprite_span_rom[497]=24'h63a284; sprite_span_rom[498]=24'h644285; sprite_span_rom[499]=24'h64e285; sprite_span_rom[500]=24'h658245; sprite_span_rom[501]=24'h661245; sprite_span_rom[502]=24'h66a265; sprite_span_rom[503]=24'h673a65;
sprite_span_rom[504]=24'h67d265; sprite_span_rom[505]=24'h686a46; sprite_span_rom[506]=24'h68fa46; sprite_span_rom[507]=24'h698a46; sprite_span_rom[508]=24'h6a1a46; sprite_span_rom[509]=24'h6aaa46; sprite_span_rom[510]=24'h6b3a27; sprite_span_rom[511]=24'h6bc227;
sprite_span_rom[512]=24'h6c4a27; sprite_span_rom[513]=24'h6cd227; sprite_span_rom[514]=24'h6d5a27; sprite_span_rom[515]=24'h6de207; sprite_span_rom[516]=24'h6e6207; sprite_span_rom[517]=24'h6ee207; sprite_span_rom[518]=24'h6f6207; sprite_span_rom[519]=24'h6fe207;
sprite_span_rom[520]=24'h7061e7; sprite_span_rom[521]=24'h70d9e7; sprite_span_rom[522]=24'h7151e7; sprite_span_rom[523]=24'h71c9c7; sprite_span_rom[524]=24'h7239a7; sprite_span_rom[525]=24'h72a187; sprite_span_rom[526]=24'h730167; sprite_span_rom[527]=24'h735947;
sprite_span_rom[528]=24'h73a9c7; sprite_span_rom[529]=24'h7419e7; sprite_span_rom[530]=24'h7491c7; sprite_span_rom[531]=24'h7501c7; sprite_span_rom[532]=24'h7571c7; sprite_span_rom[533]=24'h75e1c7; sprite_span_rom[534]=24'h7651c7; sprite_span_rom[535]=24'h76c1c7;
sprite_span_rom[536]=24'h7731c7; sprite_span_rom[537]=24'h77a1c7; sprite_span_rom[538]=24'h7811c7; sprite_span_rom[539]=24'h7881a8; sprite_span_rom[540]=24'h78e9a8; sprite_span_rom[541]=24'h79516b; sprite_span_rom[542]=24'h79a96b; sprite_span_rom[543]=24'h7a016b;
sprite_span_rom[544]=24'h7a596b; sprite_span_rom[545]=24'h7ab16b; sprite_span_rom[546]=24'h7b096b; sprite_span_rom[547]=24'h7b616b; sprite_span_rom[548]=24'h7bb96b; sprite_span_rom[549]=24'h7c116b; sprite_span_rom[550]=24'h7c698a; sprite_span_rom[551]=24'h7cc98a;
sprite_span_rom[552]=24'h7d298a; sprite_span_rom[553]=24'h7d896a; sprite_span_rom[554]=24'h7de16a; sprite_span_rom[555]=24'h7e392a; sprite_span_rom[556]=24'h7e814a; sprite_span_rom[557]=24'h7ed14a; sprite_span_rom[558]=24'h7f214a; sprite_span_rom[559]=24'h7f714a;
sprite_span_rom[560]=24'h7fc169; sprite_span_rom[561]=24'h801989; sprite_span_rom[562]=24'h807989; sprite_span_rom[563]=24'h80d989; sprite_span_rom[564]=24'h813989; sprite_span_rom[565]=24'h819989; sprite_span_rom[566]=24'h81f989; sprite_span_rom[567]=24'h82596a;
sprite_span_rom[568]=24'h82b189; sprite_span_rom[569]=24'h831189; sprite_span_rom[570]=24'h83714a; sprite_span_rom[571]=24'h83c12a; sprite_span_rom[572]=24'h8408eb; sprite_span_rom[573]=24'h8440eb; sprite_span_rom[574]=24'h847830; sprite_span_rom[575]=24'h0;
end
reg [14:0] sprite_rom [0:4239]; /* fehdl force_ram=1, ram_style="bram" */
initial begin
sprite_rom[0]=15'h6000; sprite_rom[1]=15'h1b; sprite_rom[2]=15'h5f00; sprite_rom[3]=15'h66f7; sprite_rom[4]=15'h777a; sprite_rom[5]=15'h1e; sprite_rom[6]=15'h62d7; sprite_rom[7]=15'h6738;
sprite_rom[8]=15'h6f5a; sprite_rom[9]=15'h7fdd; sprite_rom[10]=15'h5c00; sprite_rom[11]=15'h77b5; sprite_rom[12]=15'h633a; sprite_rom[13]=15'h6b37; sprite_rom[14]=15'h77bc; sprite_rom[15]=15'h1d;
sprite_rom[16]=15'h5c00; sprite_rom[17]=15'h77fb; sprite_rom[18]=15'h6fdf; sprite_rom[19]=15'h677b; sprite_rom[20]=15'h7779; sprite_rom[21]=15'h3bc; sprite_rom[22]=15'h7400; sprite_rom[23]=15'h3256;
sprite_rom[24]=15'h7ad1; sprite_rom[25]=15'h77ff; sprite_rom[26]=15'h775a; sprite_rom[27]=15'h35e; sprite_rom[28]=15'h4400; sprite_rom[29]=15'h0; sprite_rom[30]=15'h5000; sprite_rom[31]=15'h1ccb;
sprite_rom[32]=15'h4969; sprite_rom[33]=15'h7ed2; sprite_rom[34]=15'h777d; sprite_rom[35]=15'h6fbf; sprite_rom[36]=15'h3000; sprite_rom[37]=15'h5292; sprite_rom[38]=15'h3255; sprite_rom[39]=15'h256e;
sprite_rom[40]=15'h498c; sprite_rom[41]=15'h444c; sprite_rom[42]=15'h7bfa; sprite_rom[43]=15'h73bf; sprite_rom[44]=15'h5800; sprite_rom[45]=15'h52b7; sprite_rom[46]=15'h52b5; sprite_rom[47]=15'h4a92;
sprite_rom[48]=15'h2651; sprite_rom[49]=15'h7f4f; sprite_rom[50]=15'h6f7f; sprite_rom[51]=15'h5ef7; sprite_rom[52]=15'h7378; sprite_rom[53]=15'h77bd; sprite_rom[54]=15'h5fbd; sprite_rom[55]=15'h4e94;
sprite_rom[56]=15'h7a8d; sprite_rom[57]=15'h3df; sprite_rom[58]=15'h5ad7; sprite_rom[59]=15'h6f57; sprite_rom[60]=15'h6f5b; sprite_rom[61]=15'h6b5b; sprite_rom[62]=15'h777b; sprite_rom[63]=15'h6fbe;
sprite_rom[64]=15'h5296; sprite_rom[65]=15'h2b4; sprite_rom[66]=15'h5800; sprite_rom[67]=15'h6f36; sprite_rom[68]=15'h673b; sprite_rom[69]=15'h6718; sprite_rom[70]=15'h6738; sprite_rom[71]=15'h6f7a;
sprite_rom[72]=15'h7bbb; sprite_rom[73]=15'h7b9f; sprite_rom[74]=15'h52ba; sprite_rom[75]=15'h4db4; sprite_rom[76]=15'h5eb8; sprite_rom[77]=15'h675b; sprite_rom[78]=15'h62f9; sprite_rom[79]=15'h6338;
sprite_rom[80]=15'h6317; sprite_rom[81]=15'h6b39; sprite_rom[82]=15'h737b; sprite_rom[83]=15'h7bdd; sprite_rom[84]=15'h1e; sprite_rom[85]=15'h56a0; sprite_rom[86]=15'h5c00; sprite_rom[87]=15'h6f35;
sprite_rom[88]=15'h6b19; sprite_rom[89]=15'h673c; sprite_rom[90]=15'h6b9c; sprite_rom[91]=15'h677b; sprite_rom[92]=15'h6319; sprite_rom[93]=15'h6f7a; sprite_rom[94]=15'h77dc; sprite_rom[95]=15'h3bb;
sprite_rom[96]=15'h5700; sprite_rom[97]=15'h6b99; sprite_rom[98]=15'h6f5a; sprite_rom[99]=15'h737d; sprite_rom[100]=15'h739e; sprite_rom[101]=15'h6b5b; sprite_rom[102]=15'h633a; sprite_rom[103]=15'h6f58;
sprite_rom[104]=15'h73bc; sprite_rom[105]=15'h77be; sprite_rom[106]=15'h62a0; sprite_rom[107]=15'h737a; sprite_rom[108]=15'h77bc; sprite_rom[109]=15'h7bfd; sprite_rom[110]=15'h77de; sprite_rom[111]=15'h73bd;
sprite_rom[112]=15'h673a; sprite_rom[113]=15'h6b39; sprite_rom[114]=15'h777b; sprite_rom[115]=15'h77fd; sprite_rom[116]=15'h1b; sprite_rom[117]=15'h6ef5; sprite_rom[118]=15'h6f79; sprite_rom[119]=15'h6f7b;
sprite_rom[120]=15'h7fdc; sprite_rom[121]=15'h77df; sprite_rom[122]=15'h6f7c; sprite_rom[123]=15'h6b7b; sprite_rom[124]=15'h6319; sprite_rom[125]=15'h737a; sprite_rom[126]=15'h7bdd; sprite_rom[127]=15'h39b;
sprite_rom[128]=15'h5c00; sprite_rom[129]=15'h6b76; sprite_rom[130]=15'h6b38; sprite_rom[131]=15'h6b5a; sprite_rom[132]=15'h7bba; sprite_rom[133]=15'h6b5d; sprite_rom[134]=15'h6b5b; sprite_rom[135]=15'h7359;
sprite_rom[136]=15'h631a; sprite_rom[137]=15'h7779; sprite_rom[138]=15'h7bbd; sprite_rom[139]=15'h33e; sprite_rom[140]=15'h5800; sprite_rom[141]=15'h5f3a; sprite_rom[142]=15'h6738; sprite_rom[143]=15'h7339;
sprite_rom[144]=15'h775b; sprite_rom[145]=15'h6f79; sprite_rom[146]=15'h6b39; sprite_rom[147]=15'h673a; sprite_rom[148]=15'h673b; sprite_rom[149]=15'h6b59; sprite_rom[150]=15'h779b; sprite_rom[151]=15'h6bbe;
sprite_rom[152]=15'h5f00; sprite_rom[153]=15'h5ef7; sprite_rom[154]=15'h5f18; sprite_rom[155]=15'h7ff9; sprite_rom[156]=15'h6b9e; sprite_rom[157]=15'h7bdd; sprite_rom[158]=15'h671b; sprite_rom[159]=15'h6739;
sprite_rom[160]=15'h6758; sprite_rom[161]=15'h6738; sprite_rom[162]=15'h777a; sprite_rom[163]=15'h63fe; sprite_rom[164]=15'h5ee0; sprite_rom[165]=15'h5af7; sprite_rom[166]=15'h62f7; sprite_rom[167]=15'h5afc;
sprite_rom[168]=15'h62f6; sprite_rom[169]=15'h7379; sprite_rom[170]=15'h635f; sprite_rom[171]=15'h6739; sprite_rom[172]=15'h7318; sprite_rom[173]=15'h6739; sprite_rom[174]=15'h777a; sprite_rom[175]=15'h67fe;
sprite_rom[176]=15'h1c; sprite_rom[177]=15'h5ec0; sprite_rom[178]=15'h5ef9; sprite_rom[179]=15'h5af6; sprite_rom[180]=15'h1510; sprite_rom[181]=15'h2905; sprite_rom[182]=15'h414a; sprite_rom[183]=15'h5f15;
sprite_rom[184]=15'h6317; sprite_rom[185]=15'h6ad7; sprite_rom[186]=15'h6b5d; sprite_rom[187]=15'h77bb; sprite_rom[188]=15'h77de; sprite_rom[189]=15'h1b; sprite_rom[190]=15'h6ed8; sprite_rom[191]=15'h6319;
sprite_rom[192]=15'h2af7; sprite_rom[193]=15'h14a5; sprite_rom[194]=15'h14a5; sprite_rom[195]=15'h14a5; sprite_rom[196]=15'h5e88; sprite_rom[197]=15'h5ef6; sprite_rom[198]=15'h5ef7; sprite_rom[199]=15'h6f7c;
sprite_rom[200]=15'h777c; sprite_rom[201]=15'h77dd; sprite_rom[202]=15'h19; sprite_rom[203]=15'h6756; sprite_rom[204]=15'h5f19; sprite_rom[205]=15'h1619; sprite_rom[206]=15'h14a5; sprite_rom[207]=15'h1485;
sprite_rom[208]=15'h14a5; sprite_rom[209]=15'h5d45; sprite_rom[210]=15'h5ef7; sprite_rom[211]=15'h6738; sprite_rom[212]=15'h6bba; sprite_rom[213]=15'h737b; sprite_rom[214]=15'h7bdd; sprite_rom[215]=15'h39b;
sprite_rom[216]=15'h5800; sprite_rom[217]=15'h6777; sprite_rom[218]=15'h6719; sprite_rom[219]=15'h14b5; sprite_rom[220]=15'h1085; sprite_rom[221]=15'hc84; sprite_rom[222]=15'h1084; sprite_rom[223]=15'h54a5;
sprite_rom[224]=15'h62f9; sprite_rom[225]=15'h6b39; sprite_rom[226]=15'h7799; sprite_rom[227]=15'h737b; sprite_rom[228]=15'h7fdd; sprite_rom[229]=15'h37c; sprite_rom[230]=15'h5800; sprite_rom[231]=15'h633d;
sprite_rom[232]=15'h6339; sprite_rom[233]=15'h14aa; sprite_rom[234]=15'hc84; sprite_rom[235]=15'hc63; sprite_rom[236]=15'h1063; sprite_rom[237]=15'h4044; sprite_rom[238]=15'h62f8; sprite_rom[239]=15'h6b38;
sprite_rom[240]=15'h773a; sprite_rom[241]=15'h779e; sprite_rom[242]=15'h7fde; sprite_rom[243]=15'h37c; sprite_rom[244]=15'h62c0; sprite_rom[245]=15'h633b; sprite_rom[246]=15'h5358; sprite_rom[247]=15'h423;
sprite_rom[248]=15'hc83; sprite_rom[249]=15'hc63; sprite_rom[250]=15'hc63; sprite_rom[251]=15'h2084; sprite_rom[252]=15'h62f6; sprite_rom[253]=15'h6738; sprite_rom[254]=15'h6b5b; sprite_rom[255]=15'h7bfe;
sprite_rom[256]=15'h7ffe; sprite_rom[257]=15'h3bb; sprite_rom[258]=15'h76e0; sprite_rom[259]=15'h6338; sprite_rom[260]=15'h5318; sprite_rom[261]=15'h4693; sprite_rom[262]=15'hc44; sprite_rom[263]=15'hc63;
sprite_rom[264]=15'h863; sprite_rom[265]=15'h421; sprite_rom[266]=15'h5ef0; sprite_rom[267]=15'h6719; sprite_rom[268]=15'h6fba; sprite_rom[269]=15'h7fdb; sprite_rom[270]=15'h7fff; sprite_rom[271]=15'h3db;
sprite_rom[272]=15'h7700; sprite_rom[273]=15'h6338; sprite_rom[274]=15'h16ba; sprite_rom[275]=15'h4d44; sprite_rom[276]=15'h914; sprite_rom[277]=15'hc63; sprite_rom[278]=15'h1043; sprite_rom[279]=15'h2d69;
sprite_rom[280]=15'h5eca; sprite_rom[281]=15'h6718; sprite_rom[282]=15'h7b99; sprite_rom[283]=15'h7bbd; sprite_rom[284]=15'h7bff; sprite_rom[285]=15'h3db; sprite_rom[286]=15'h7340; sprite_rom[287]=15'h6317;
sprite_rom[288]=15'h61c; sprite_rom[289]=15'h508; sprite_rom[290]=15'h904; sprite_rom[291]=15'hc63; sprite_rom[292]=15'h5102; sprite_rom[293]=15'h4e74; sprite_rom[294]=15'h5e93; sprite_rom[295]=15'h6317;
sprite_rom[296]=15'h7739; sprite_rom[297]=15'h7bde; sprite_rom[298]=15'h7bde; sprite_rom[299]=15'h1b; sprite_rom[300]=15'h6b80; sprite_rom[301]=15'h6738; sprite_rom[302]=15'h515c; sprite_rom[303]=15'h5fdf;
sprite_rom[304]=15'h102a; sprite_rom[305]=15'hc63; sprite_rom[306]=15'h14a3; sprite_rom[307]=15'h422; sprite_rom[308]=15'h4021; sprite_rom[309]=15'h5ef7; sprite_rom[310]=15'h6f5a; sprite_rom[311]=15'h7bdd;
sprite_rom[312]=15'h7fde; sprite_rom[313]=15'h1a; sprite_rom[314]=15'h380; sprite_rom[315]=15'h6359; sprite_rom[316]=15'h7eb6; sprite_rom[317]=15'h7ed4; sprite_rom[318]=15'h55f; sprite_rom[319]=15'hc63;
sprite_rom[320]=15'h1443; sprite_rom[321]=15'h5ad4; sprite_rom[322]=15'h954; sprite_rom[323]=15'h62f0; sprite_rom[324]=15'h6b99; sprite_rom[325]=15'h7bbb; sprite_rom[326]=15'h6ffe; sprite_rom[327]=15'h1b;
sprite_rom[328]=15'h3c0; sprite_rom[329]=15'h671c; sprite_rom[330]=15'h52b5; sprite_rom[331]=15'h528d; sprite_rom[332]=15'h63f; sprite_rom[333]=15'hc63; sprite_rom[334]=15'h5043; sprite_rom[335]=15'h7ff7;
sprite_rom[336]=15'h4fff; sprite_rom[337]=15'h6201; sprite_rom[338]=15'h7739; sprite_rom[339]=15'h7bbd; sprite_rom[340]=15'h6bde; sprite_rom[341]=15'h1f; sprite_rom[342]=15'h6780; sprite_rom[343]=15'he17;
sprite_rom[344]=15'h7bf2; sprite_rom[345]=15'h63f; sprite_rom[346]=15'hc64; sprite_rom[347]=15'h20a2; sprite_rom[348]=15'h69d4; sprite_rom[349]=15'h7f5f; sprite_rom[350]=15'h5471; sprite_rom[351]=15'h6f5d;
sprite_rom[352]=15'h7bde; sprite_rom[353]=15'h77be; sprite_rom[354]=15'h5c00; sprite_rom[355]=15'h697; sprite_rom[356]=15'h7b8f; sprite_rom[357]=15'h63e; sprite_rom[358]=15'hc64; sprite_rom[359]=15'h443;
sprite_rom[360]=15'h7ffc; sprite_rom[361]=15'h52df; sprite_rom[362]=15'h391f; sprite_rom[363]=15'h7fdb; sprite_rom[364]=15'h7bff; sprite_rom[365]=15'h77be; sprite_rom[366]=15'h4000; sprite_rom[367]=15'h16d4;
sprite_rom[368]=15'h56a6; sprite_rom[369]=15'h8f5; sprite_rom[370]=15'hc63; sprite_rom[371]=15'h843; sprite_rom[372]=15'h7fd9; sprite_rom[373]=15'h2ebf; sprite_rom[374]=15'h2c30; sprite_rom[375]=15'h57b8;
sprite_rom[376]=15'h7a10; sprite_rom[377]=15'h37f; sprite_rom[378]=15'h22b6; sprite_rom[379]=15'h4148; sprite_rom[380]=15'h104a; sprite_rom[381]=15'hc63; sprite_rom[382]=15'h483; sprite_rom[383]=15'h56b4;
sprite_rom[384]=15'h15b7; sprite_rom[385]=15'h4045; sprite_rom[386]=15'h2abc; sprite_rom[387]=15'h548a; sprite_rom[388]=15'h3be; sprite_rom[389]=15'h2218; sprite_rom[390]=15'h1508; sprite_rom[391]=15'h1085;
sprite_rom[392]=15'hc65; sprite_rom[393]=15'hc83; sprite_rom[394]=15'h5268; sprite_rom[395]=15'h1470; sprite_rom[396]=15'h5048; sprite_rom[397]=15'h42b8; sprite_rom[398]=15'h510a; sprite_rom[399]=15'h1f;
sprite_rom[400]=15'h22b8; sprite_rom[401]=15'h1508; sprite_rom[402]=15'h1065; sprite_rom[403]=15'hc63; sprite_rom[404]=15'h1483; sprite_rom[405]=15'h20a5; sprite_rom[406]=15'h2108; sprite_rom[407]=15'h5545;
sprite_rom[408]=15'h42d4; sprite_rom[409]=15'h5caa; sprite_rom[410]=15'h3818; sprite_rom[411]=15'h14a4; sprite_rom[412]=15'hc64; sprite_rom[413]=15'h862; sprite_rom[414]=15'h1483; sprite_rom[415]=15'h2105;
sprite_rom[416]=15'h1508; sprite_rom[417]=15'h2a68; sprite_rom[418]=15'h2ab0; sprite_rom[419]=15'h262; sprite_rom[420]=15'h5418; sprite_rom[421]=15'h1045; sprite_rom[422]=15'h1443; sprite_rom[423]=15'h20a8;
sprite_rom[424]=15'h1465; sprite_rom[425]=15'h20a5; sprite_rom[426]=15'hd08; sprite_rom[427]=15'h1208; sprite_rom[428]=15'h26b0; sprite_rom[429]=15'h15; sprite_rom[430]=15'h19; sprite_rom[431]=15'h4d5;
sprite_rom[432]=15'hc83; sprite_rom[433]=15'h1505; sprite_rom[434]=15'hc64; sprite_rom[435]=15'h14a5; sprite_rom[436]=15'h2023; sprite_rom[437]=15'h63d5; sprite_rom[438]=15'h2f7; sprite_rom[439]=15'h1a;
sprite_rom[440]=15'h3800; sprite_rom[441]=15'hc21; sprite_rom[442]=15'h863; sprite_rom[443]=15'hc63; sprite_rom[444]=15'h422; sprite_rom[445]=15'h6e68; sprite_rom[446]=15'h37f; sprite_rom[447]=15'h1c;
sprite_rom[448]=15'h1b; sprite_rom[449]=15'h0; sprite_rom[450]=15'h974; sprite_rom[451]=15'h421; sprite_rom[452]=15'h1061; sprite_rom[453]=15'h5165; sprite_rom[454]=15'h3fb; sprite_rom[455]=15'h0;
sprite_rom[456]=15'h19; sprite_rom[457]=15'h4170; sprite_rom[458]=15'h4150; sprite_rom[459]=15'h2d0; sprite_rom[460]=15'h0; sprite_rom[461]=15'h0; sprite_rom[462]=15'h19; sprite_rom[463]=15'h6400;
sprite_rom[464]=15'h2950; sprite_rom[465]=15'h5a70; sprite_rom[466]=15'h0; sprite_rom[467]=15'h6400; sprite_rom[468]=15'h4ec0; sprite_rom[469]=15'h2910; sprite_rom[470]=15'h4e70; sprite_rom[471]=15'h16;
sprite_rom[472]=15'h5800; sprite_rom[473]=15'h4694; sprite_rom[474]=15'h394a; sprite_rom[475]=15'h4e6f; sprite_rom[476]=15'h293; sprite_rom[477]=15'h56a0; sprite_rom[478]=15'h3273; sprite_rom[479]=15'h316a;
sprite_rom[480]=15'h4a4f; sprite_rom[481]=15'h24d1; sprite_rom[482]=15'h16; sprite_rom[483]=15'h556e; sprite_rom[484]=15'h35f4; sprite_rom[485]=15'h352b; sprite_rom[486]=15'h3e6d; sprite_rom[487]=15'h44c;
sprite_rom[488]=15'h9; sprite_rom[489]=15'h4c00; sprite_rom[490]=15'h5561; sprite_rom[491]=15'h3db4; sprite_rom[492]=15'h35a7; sprite_rom[493]=15'h35ef; sprite_rom[494]=15'h186b; sprite_rom[495]=15'h221;
sprite_rom[496]=15'h1c00; sprite_rom[497]=15'h5583; sprite_rom[498]=15'h31f4; sprite_rom[499]=15'h35ac; sprite_rom[500]=15'h3589; sprite_rom[501]=15'h18c9; sprite_rom[502]=15'he3; sprite_rom[503]=15'h620;
sprite_rom[504]=15'h5189; sprite_rom[505]=15'h31f5; sprite_rom[506]=15'h258c; sprite_rom[507]=15'h35c4; sprite_rom[508]=15'h10c9; sprite_rom[509]=15'h4c46; sprite_rom[510]=15'h10d6; sprite_rom[511]=15'h45ae;
sprite_rom[512]=15'h3235; sprite_rom[513]=15'h9ac; sprite_rom[514]=15'h31e7; sprite_rom[515]=15'h18c9; sprite_rom[516]=15'h2c66; sprite_rom[517]=15'h184e; sprite_rom[518]=15'h2dae; sprite_rom[519]=15'h3295;
sprite_rom[520]=15'h4ed; sprite_rom[521]=15'h31a7; sprite_rom[522]=15'h1ce9; sprite_rom[523]=15'h10c3; sprite_rom[524]=15'h14; sprite_rom[525]=15'h5400; sprite_rom[526]=15'h1ce2; sprite_rom[527]=15'h25ee;
sprite_rom[528]=15'h3eb4; sprite_rom[529]=15'h849; sprite_rom[530]=15'h31a9; sprite_rom[531]=15'h196b; sprite_rom[532]=15'h8c6; sprite_rom[533]=15'h11; sprite_rom[534]=15'h2c00; sprite_rom[535]=15'h2503;
sprite_rom[536]=15'h324e; sprite_rom[537]=15'h46b4; sprite_rom[538]=15'h8c3; sprite_rom[539]=15'h35eb; sprite_rom[540]=15'h96e; sprite_rom[541]=15'hc46; sprite_rom[542]=15'h9; sprite_rom[543]=15'h660;
sprite_rom[544]=15'h2463; sprite_rom[545]=15'h4e51; sprite_rom[546]=15'h46b3; sprite_rom[547]=15'h2527; sprite_rom[548]=15'h35f0; sprite_rom[549]=15'h930; sprite_rom[550]=15'h18c6; sprite_rom[551]=15'h266;
sprite_rom[552]=15'h2120; sprite_rom[553]=15'h256b; sprite_rom[554]=15'h524f; sprite_rom[555]=15'h52b2; sprite_rom[556]=15'h462f; sprite_rom[557]=15'h3df4; sprite_rom[558]=15'h1c6e; sprite_rom[559]=15'h10c7;
sprite_rom[560]=15'h1c4; sprite_rom[561]=15'h2451; sprite_rom[562]=15'h2d27; sprite_rom[563]=15'h5a71; sprite_rom[564]=15'h7752; sprite_rom[565]=15'h4a51; sprite_rom[566]=15'h4df4; sprite_rom[567]=15'h256e;
sprite_rom[568]=15'h2527; sprite_rom[569]=15'hc3; sprite_rom[570]=15'h1cee; sprite_rom[571]=15'h18c6; sprite_rom[572]=15'h5a89; sprite_rom[573]=15'h62b5; sprite_rom[574]=15'h5274; sprite_rom[575]=15'h4a56;
sprite_rom[576]=15'h18eb; sprite_rom[577]=15'h2927; sprite_rom[578]=15'h4c29; sprite_rom[579]=15'h18f1; sprite_rom[580]=15'h1846; sprite_rom[581]=15'h45ad; sprite_rom[582]=15'h20aa; sprite_rom[583]=15'h5a94;
sprite_rom[584]=15'h1134; sprite_rom[585]=15'h18c3; sprite_rom[586]=15'hc44; sprite_rom[587]=15'h4887; sprite_rom[588]=15'hc53; sprite_rom[589]=15'h2c83; sprite_rom[590]=15'h220d; sprite_rom[591]=15'h28a1;
sprite_rom[592]=15'h3e90; sprite_rom[593]=15'h196d; sprite_rom[594]=15'hc46; sprite_rom[595]=15'h18c6; sprite_rom[596]=15'h50c6; sprite_rom[597]=15'h184e; sprite_rom[598]=15'h34c6; sprite_rom[599]=15'h94d;
sprite_rom[600]=15'h2942; sprite_rom[601]=15'h2865; sprite_rom[602]=15'h2dad; sprite_rom[603]=15'h8c4; sprite_rom[604]=15'h18c6; sprite_rom[605]=15'h54e6; sprite_rom[606]=15'h1c90; sprite_rom[607]=15'h3526;
sprite_rom[608]=15'h2a0c; sprite_rom[609]=15'h16b4; sprite_rom[610]=15'h20a5; sprite_rom[611]=15'h318b; sprite_rom[612]=15'h18c6; sprite_rom[613]=15'h18c6; sprite_rom[614]=15'h126; sprite_rom[615]=15'h2491;
sprite_rom[616]=15'h3586; sprite_rom[617]=15'h4e49; sprite_rom[618]=15'h42d6; sprite_rom[619]=15'h2021; sprite_rom[620]=15'h318a; sprite_rom[621]=15'h1c69; sprite_rom[622]=15'h1cc6; sprite_rom[623]=15'h226;
sprite_rom[624]=15'h20f3; sprite_rom[625]=15'h31a7; sprite_rom[626]=15'h4629; sprite_rom[627]=15'h4e76; sprite_rom[628]=15'h4eb3; sprite_rom[629]=15'h3530; sprite_rom[630]=15'h18cc; sprite_rom[631]=15'h1ce7;
sprite_rom[632]=15'h266; sprite_rom[633]=15'h1913; sprite_rom[634]=15'h31ab; sprite_rom[635]=15'h45c9; sprite_rom[636]=15'h2e36; sprite_rom[637]=15'h47df; sprite_rom[638]=15'h352e; sprite_rom[639]=15'h8ed;
sprite_rom[640]=15'h1ce7; sprite_rom[641]=15'h269; sprite_rom[642]=15'h10f4; sprite_rom[643]=15'h2dac; sprite_rom[644]=15'h45c9; sprite_rom[645]=15'h2236; sprite_rom[646]=15'h4fd6; sprite_rom[647]=15'h312e;
sprite_rom[648]=15'hd6d; sprite_rom[649]=15'h24e6; sprite_rom[650]=15'h269; sprite_rom[651]=15'h1c53; sprite_rom[652]=15'h25ed; sprite_rom[653]=15'h4d6b; sprite_rom[654]=15'h1a34; sprite_rom[655]=15'h57f4;
sprite_rom[656]=15'h2d2d; sprite_rom[657]=15'h1dac; sprite_rom[658]=15'h24e4; sprite_rom[659]=15'h287; sprite_rom[660]=15'h2c30; sprite_rom[661]=15'h1ded; sprite_rom[662]=15'h4d6b; sprite_rom[663]=15'h1a33;
sprite_rom[664]=15'h4e4b; sprite_rom[665]=15'h1d6f; sprite_rom[666]=15'h2dad; sprite_rom[667]=15'h1ce4; sprite_rom[668]=15'h2c7; sprite_rom[669]=15'h34ce; sprite_rom[670]=15'h19ec; sprite_rom[671]=15'h51ab;
sprite_rom[672]=15'h19d2; sprite_rom[673]=15'h4ce9; sprite_rom[674]=15'h1d73; sprite_rom[675]=15'h398d; sprite_rom[676]=15'h18c7; sprite_rom[677]=15'he; sprite_rom[678]=15'h352e; sprite_rom[679]=15'h1d8d;
sprite_rom[680]=15'h5249; sprite_rom[681]=15'h11d2; sprite_rom[682]=15'h44e9; sprite_rom[683]=15'h1d74; sprite_rom[684]=15'h358c; sprite_rom[685]=15'h18c9; sprite_rom[686]=15'h14; sprite_rom[687]=15'h318e;
sprite_rom[688]=15'h252f; sprite_rom[689]=15'h4e8b; sprite_rom[690]=15'hdd2; sprite_rom[691]=15'h2cc9; sprite_rom[692]=15'h1dd4; sprite_rom[693]=15'h35a9; sprite_rom[694]=15'h24ee; sprite_rom[695]=15'h16;
sprite_rom[696]=15'h31b3; sprite_rom[697]=15'h252f; sprite_rom[698]=15'h4a92; sprite_rom[699]=15'h9d2; sprite_rom[700]=15'h2c67; sprite_rom[701]=15'h2693; sprite_rom[702]=15'h31eb; sprite_rom[703]=15'h492f;
sprite_rom[704]=15'h14; sprite_rom[705]=15'h31f6; sprite_rom[706]=15'h352f; sprite_rom[707]=15'h4a54; sprite_rom[708]=15'h9d1; sprite_rom[709]=15'h2c66; sprite_rom[710]=15'h4692; sprite_rom[711]=15'h31e9;
sprite_rom[712]=15'h522f; sprite_rom[713]=15'h2b2; sprite_rom[714]=15'h3660; sprite_rom[715]=15'h5233; sprite_rom[716]=15'h4a54; sprite_rom[717]=15'hdd1; sprite_rom[718]=15'h2466; sprite_rom[719]=15'h526f;
sprite_rom[720]=15'h3e71; sprite_rom[721]=15'h4293; sprite_rom[722]=15'h273; sprite_rom[723]=15'h3ec0; sprite_rom[724]=15'h4e93; sprite_rom[725]=15'h4e32; sprite_rom[726]=15'hdcf; sprite_rom[727]=15'h2466;
sprite_rom[728]=15'h4e4f; sprite_rom[729]=15'h4e74; sprite_rom[730]=15'h2dd4; sprite_rom[731]=15'h232; sprite_rom[732]=15'h5800; sprite_rom[733]=15'h2e75; sprite_rom[734]=15'h4df2; sprite_rom[735]=15'hdcf;
sprite_rom[736]=15'h1c66; sprite_rom[737]=15'h462f; sprite_rom[738]=15'h4e93; sprite_rom[739]=15'h394b; sprite_rom[740]=15'h5231; sprite_rom[741]=15'h2535; sprite_rom[742]=15'h4db1; sprite_rom[743]=15'h9cf;
sprite_rom[744]=15'h1866; sprite_rom[745]=15'h3dee; sprite_rom[746]=15'h21ef; sprite_rom[747]=15'h2cc6; sprite_rom[748]=15'h4a2e; sprite_rom[749]=15'h2493; sprite_rom[750]=15'h4db1; sprite_rom[751]=15'h9cf;
sprite_rom[752]=15'h1866; sprite_rom[753]=15'h45ee; sprite_rom[754]=15'h1dec; sprite_rom[755]=15'h2cc6; sprite_rom[756]=15'h3e2b; sprite_rom[757]=15'h244e; sprite_rom[758]=15'h4db2; sprite_rom[759]=15'h9cf;
sprite_rom[760]=15'h1084; sprite_rom[761]=15'h49ee; sprite_rom[762]=15'h1dec; sprite_rom[763]=15'h24c6; sprite_rom[764]=15'h3e2b; sprite_rom[765]=15'h14; sprite_rom[766]=15'h2c47; sprite_rom[767]=15'h45b2;
sprite_rom[768]=15'h971; sprite_rom[769]=15'hcc3; sprite_rom[770]=15'h49ee; sprite_rom[771]=15'h25ec; sprite_rom[772]=15'h18c3; sprite_rom[773]=15'h45cb; sprite_rom[774]=15'h12; sprite_rom[775]=15'h4c00;
sprite_rom[776]=15'h2c81; sprite_rom[777]=15'h39b1; sprite_rom[778]=15'hcc6; sprite_rom[779]=15'h863; sprite_rom[780]=15'h49eb; sprite_rom[781]=15'h2dac; sprite_rom[782]=15'h18c3; sprite_rom[783]=15'h3dcb;
sprite_rom[784]=15'hd; sprite_rom[785]=15'h4400; sprite_rom[786]=15'h38c1; sprite_rom[787]=15'h45ef; sprite_rom[788]=15'h10c2; sprite_rom[789]=15'h863; sprite_rom[790]=15'h4a2b; sprite_rom[791]=15'h35ad;
sprite_rom[792]=15'h1083; sprite_rom[793]=15'h39c9; sprite_rom[794]=15'h26d; sprite_rom[795]=15'h2c00; sprite_rom[796]=15'h3cc2; sprite_rom[797]=15'h39ef; sprite_rom[798]=15'h10c2; sprite_rom[799]=15'h1862;
sprite_rom[800]=15'h4e29; sprite_rom[801]=15'h398d; sprite_rom[802]=15'h1866; sprite_rom[803]=15'h2dc6; sprite_rom[804]=15'h1ef; sprite_rom[805]=15'h12a0; sprite_rom[806]=15'h44e6; sprite_rom[807]=15'h39ef;
sprite_rom[808]=15'h18c2; sprite_rom[809]=15'h1862; sprite_rom[810]=15'h38c7; sprite_rom[811]=15'h358f; sprite_rom[812]=15'h1067; sprite_rom[813]=15'h2dc6; sprite_rom[814]=15'h5591; sprite_rom[815]=15'h660;
sprite_rom[816]=15'h44e6; sprite_rom[817]=15'h2def; sprite_rom[818]=15'h18c2; sprite_rom[819]=15'h1863; sprite_rom[820]=15'h2c47; sprite_rom[821]=15'h3d91; sprite_rom[822]=15'h1869; sprite_rom[823]=15'h3964;
sprite_rom[824]=15'h4df1; sprite_rom[825]=15'h46c0; sprite_rom[826]=15'h496b; sprite_rom[827]=15'h39ec; sprite_rom[828]=15'h18c3; sprite_rom[829]=15'h1863; sprite_rom[830]=15'h2467; sprite_rom[831]=15'h3d91;
sprite_rom[832]=15'h82b; sprite_rom[833]=15'h3922; sprite_rom[834]=15'h324e; sprite_rom[835]=15'h2ea0; sprite_rom[836]=15'h49ce; sprite_rom[837]=15'h2e72; sprite_rom[838]=15'h18c3; sprite_rom[839]=15'h1883;
sprite_rom[840]=15'h1cc7; sprite_rom[841]=15'h4a32; sprite_rom[842]=15'h2d71; sprite_rom[843]=15'h2e4e; sprite_rom[844]=15'h5249; sprite_rom[845]=15'h620; sprite_rom[846]=15'h4126; sprite_rom[847]=15'h194e;
sprite_rom[848]=15'h18c4; sprite_rom[849]=15'h18c6; sprite_rom[850]=15'h1843; sprite_rom[851]=15'h2dcb; sprite_rom[852]=15'h39c9; sprite_rom[853]=15'h2a0b; sprite_rom[854]=15'h2a1; sprite_rom[855]=15'h2680;
sprite_rom[856]=15'h2d02; sprite_rom[857]=15'h3a2e; sprite_rom[858]=15'h869; sprite_rom[859]=15'h1822; sprite_rom[860]=15'h39cb; sprite_rom[861]=15'h8cb; sprite_rom[862]=15'h842; sprite_rom[863]=15'h4d21;
sprite_rom[864]=15'h13; sprite_rom[865]=15'h5694; sprite_rom[866]=15'h2449; sprite_rom[867]=15'h2e31; sprite_rom[868]=15'h5609; sprite_rom[869]=15'h24f1; sprite_rom[870]=15'h3a2e; sprite_rom[871]=15'h2d2b;
sprite_rom[872]=15'h291; sprite_rom[873]=15'h5000; sprite_rom[874]=15'h421; sprite_rom[875]=15'h4da1; sprite_rom[876]=15'h53b4; sprite_rom[877]=15'h421; sprite_rom[878]=15'h44c1; sprite_rom[879]=15'h16;
sprite_rom[880]=15'h4e74; sprite_rom[881]=15'h2b3; sprite_rom[882]=15'h0; sprite_rom[883]=15'h39d1; sprite_rom[884]=15'h26f; sprite_rom[885]=15'h5c00; sprite_rom[886]=15'h62f7; sprite_rom[887]=15'h7b9a;
sprite_rom[888]=15'h5ed7; sprite_rom[889]=15'h6f77; sprite_rom[890]=15'h737b; sprite_rom[891]=15'h3fe; sprite_rom[892]=15'h5800; sprite_rom[893]=15'h6796; sprite_rom[894]=15'h5f19; sprite_rom[895]=15'h7759;
sprite_rom[896]=15'h7bbd; sprite_rom[897]=15'h5800; sprite_rom[898]=15'h7ffd; sprite_rom[899]=15'h6b5e; sprite_rom[900]=15'h6b7b; sprite_rom[901]=15'h77bb; sprite_rom[902]=15'h1c; sprite_rom[903]=15'h7000;
sprite_rom[904]=15'h4e97; sprite_rom[905]=15'h7bf8; sprite_rom[906]=15'h677e; sprite_rom[907]=15'h7b99; sprite_rom[908]=15'h3bb; sprite_rom[909]=15'h2e80; sprite_rom[910]=15'h13; sprite_rom[911]=15'h4c00;
sprite_rom[912]=15'h1c47; sprite_rom[913]=15'h62cb; sprite_rom[914]=15'h73ff; sprite_rom[915]=15'h7b7a; sprite_rom[916]=15'h37e; sprite_rom[917]=15'h4c00; sprite_rom[918]=15'h5294; sprite_rom[919]=15'h2db5;
sprite_rom[920]=15'h3129; sprite_rom[921]=15'h318c; sprite_rom[922]=15'h7ece; sprite_rom[923]=15'h7b9e; sprite_rom[924]=15'h35e; sprite_rom[925]=15'h6380; sprite_rom[926]=15'h52b6; sprite_rom[927]=15'h3654;
sprite_rom[928]=15'h4e4e; sprite_rom[929]=15'h4ccc; sprite_rom[930]=15'h7bff; sprite_rom[931]=15'h37f; sprite_rom[932]=15'h5ef7; sprite_rom[933]=15'h6b17; sprite_rom[934]=15'h7bdc; sprite_rom[935]=15'h63bd;
sprite_rom[936]=15'h3ab7; sprite_rom[937]=15'h3232; sprite_rom[938]=15'h7ff4; sprite_rom[939]=15'h3be; sprite_rom[940]=15'h5ac0; sprite_rom[941]=15'h6717; sprite_rom[942]=15'h6f7a; sprite_rom[943]=15'h6f7b;
sprite_rom[944]=15'h777b; sprite_rom[945]=15'h6fdd; sprite_rom[946]=15'h4ad6; sprite_rom[947]=15'h7ef2; sprite_rom[948]=15'h1f; sprite_rom[949]=15'h5c00; sprite_rom[950]=15'h6ed5; sprite_rom[951]=15'h637c;
sprite_rom[952]=15'h6738; sprite_rom[953]=15'h6738; sprite_rom[954]=15'h6f7a; sprite_rom[955]=15'h777b; sprite_rom[956]=15'h5fff; sprite_rom[957]=15'h52b4; sprite_rom[958]=15'h15; sprite_rom[959]=15'h56c0;
sprite_rom[960]=15'h6f9a; sprite_rom[961]=15'h5f1b; sprite_rom[962]=15'h6758; sprite_rom[963]=15'h6338; sprite_rom[964]=15'h6f59; sprite_rom[965]=15'h737b; sprite_rom[966]=15'h7fbc; sprite_rom[967]=15'h7bde;
sprite_rom[968]=15'h5295; sprite_rom[969]=15'h233; sprite_rom[970]=15'h6ef5; sprite_rom[971]=15'h6f3a; sprite_rom[972]=15'h6b3c; sprite_rom[973]=15'h6f7c; sprite_rom[974]=15'h6b7c; sprite_rom[975]=15'h6319;
sprite_rom[976]=15'h6f59; sprite_rom[977]=15'h77bc; sprite_rom[978]=15'h7bdf; sprite_rom[979]=15'h0; sprite_rom[980]=15'h55f5; sprite_rom[981]=15'h5400; sprite_rom[982]=15'h6b97; sprite_rom[983]=15'h6f5a;
sprite_rom[984]=15'h739d; sprite_rom[985]=15'h739d; sprite_rom[986]=15'h6b7b; sprite_rom[987]=15'h633a; sprite_rom[988]=15'h6f37; sprite_rom[989]=15'h777b; sprite_rom[990]=15'h77de; sprite_rom[991]=15'h1e;
sprite_rom[992]=15'h5ec0; sprite_rom[993]=15'h737b; sprite_rom[994]=15'h77bc; sprite_rom[995]=15'h7bfd; sprite_rom[996]=15'h7bde; sprite_rom[997]=15'h6fbd; sprite_rom[998]=15'h673a; sprite_rom[999]=15'h6b17;
sprite_rom[1000]=15'h779b; sprite_rom[1001]=15'h7bbd; sprite_rom[1002]=15'h39c; sprite_rom[1003]=15'h72f6; sprite_rom[1004]=15'h6b59; sprite_rom[1005]=15'h6f7a; sprite_rom[1006]=15'h7fdb; sprite_rom[1007]=15'h7bdf;
sprite_rom[1008]=15'h779d; sprite_rom[1009]=15'h677c; sprite_rom[1010]=15'h6739; sprite_rom[1011]=15'h6f7a; sprite_rom[1012]=15'h7bbb; sprite_rom[1013]=15'h35f; sprite_rom[1014]=15'h5c00; sprite_rom[1015]=15'h5f76;
sprite_rom[1016]=15'h6759; sprite_rom[1017]=15'h6739; sprite_rom[1018]=15'h7bbb; sprite_rom[1019]=15'h6f7c; sprite_rom[1020]=15'h6b5b; sprite_rom[1021]=15'h6f7b; sprite_rom[1022]=15'h5f39; sprite_rom[1023]=15'h6f58;
sprite_rom[1024]=15'h7b9b; sprite_rom[1025]=15'h6fbe; sprite_rom[1026]=15'h5800; sprite_rom[1027]=15'h62f7; sprite_rom[1028]=15'h6739; sprite_rom[1029]=15'h7399; sprite_rom[1030]=15'h6b9a; sprite_rom[1031]=15'h6b99;
sprite_rom[1032]=15'h6b5a; sprite_rom[1033]=15'h6f39; sprite_rom[1034]=15'h5f3b; sprite_rom[1035]=15'h6b37; sprite_rom[1036]=15'h77bb; sprite_rom[1037]=15'h6bde; sprite_rom[1038]=15'h1c; sprite_rom[1039]=15'h5b00;
sprite_rom[1040]=15'h5ef7; sprite_rom[1041]=15'h6719; sprite_rom[1042]=15'h7bff; sprite_rom[1043]=15'h6b5e; sprite_rom[1044]=15'h77de; sprite_rom[1045]=15'h6739; sprite_rom[1046]=15'h6739; sprite_rom[1047]=15'h677b;
sprite_rom[1048]=15'h6f59; sprite_rom[1049]=15'h779c; sprite_rom[1050]=15'h7bdd; sprite_rom[1051]=15'h1a; sprite_rom[1052]=15'h5ec0; sprite_rom[1053]=15'h5ef8; sprite_rom[1054]=15'h7317; sprite_rom[1055]=15'h5ad7;
sprite_rom[1056]=15'h7398; sprite_rom[1057]=15'h7bde; sprite_rom[1058]=15'h671c; sprite_rom[1059]=15'h6739; sprite_rom[1060]=15'h6778; sprite_rom[1061]=15'h6738; sprite_rom[1062]=15'h737a; sprite_rom[1063]=15'h7fdd;
sprite_rom[1064]=15'h1a; sprite_rom[1065]=15'h6af8; sprite_rom[1066]=15'h5f19; sprite_rom[1067]=15'h42d7; sprite_rom[1068]=15'h210a; sprite_rom[1069]=15'h420a; sprite_rom[1070]=15'h5a91; sprite_rom[1071]=15'h5efd;
sprite_rom[1072]=15'h6738; sprite_rom[1073]=15'h6ef8; sprite_rom[1074]=15'h6718; sprite_rom[1075]=15'h737a; sprite_rom[1076]=15'h7fdd; sprite_rom[1077]=15'h39b; sprite_rom[1078]=15'h6756; sprite_rom[1079]=15'h6319;
sprite_rom[1080]=15'h1557; sprite_rom[1081]=15'h14a5; sprite_rom[1082]=15'h14a5; sprite_rom[1083]=15'h28a5; sprite_rom[1084]=15'h5af4; sprite_rom[1085]=15'h5ef7; sprite_rom[1086]=15'h6ad7; sprite_rom[1087]=15'h675d;
sprite_rom[1088]=15'h777a; sprite_rom[1089]=15'h7bde; sprite_rom[1090]=15'h37c; sprite_rom[1091]=15'h5800; sprite_rom[1092]=15'h6357; sprite_rom[1093]=15'h6319; sprite_rom[1094]=15'h14b0; sprite_rom[1095]=15'h14a5;
sprite_rom[1096]=15'h14a5; sprite_rom[1097]=15'h14a5; sprite_rom[1098]=15'h5eca; sprite_rom[1099]=15'h5ed7; sprite_rom[1100]=15'h5ef7; sprite_rom[1101]=15'h73bc; sprite_rom[1102]=15'h779c; sprite_rom[1103]=15'h7bbd;
sprite_rom[1104]=15'h35d; sprite_rom[1105]=15'h5b00; sprite_rom[1106]=15'h631c; sprite_rom[1107]=15'h5338; sprite_rom[1108]=15'h14a5; sprite_rom[1109]=15'h10a5; sprite_rom[1110]=15'h1084; sprite_rom[1111]=15'h14a5;
sprite_rom[1112]=15'h6685; sprite_rom[1113]=15'h5ef7; sprite_rom[1114]=15'h6738; sprite_rom[1115]=15'h6b7a; sprite_rom[1116]=15'h6f9c; sprite_rom[1117]=15'h7bbc; sprite_rom[1118]=15'h33d; sprite_rom[1119]=15'h6ae0;
sprite_rom[1120]=15'h631b; sprite_rom[1121]=15'h22d9; sprite_rom[1122]=15'h10a4; sprite_rom[1123]=15'h1084; sprite_rom[1124]=15'h1084; sprite_rom[1125]=15'h1484; sprite_rom[1126]=15'h6203; sprite_rom[1127]=15'h6317;
sprite_rom[1128]=15'h6739; sprite_rom[1129]=15'h73ba; sprite_rom[1130]=15'h6f7a; sprite_rom[1131]=15'h7bbd; sprite_rom[1132]=15'h33e; sprite_rom[1133]=15'h7ae0; sprite_rom[1134]=15'h6339; sprite_rom[1135]=15'h2298;
sprite_rom[1136]=15'h1023; sprite_rom[1137]=15'hc64; sprite_rom[1138]=15'hc63; sprite_rom[1139]=15'h1083; sprite_rom[1140]=15'h5905; sprite_rom[1141]=15'h6317; sprite_rom[1142]=15'h6b38; sprite_rom[1143]=15'h7b79;
sprite_rom[1144]=15'h737c; sprite_rom[1145]=15'h7bdd; sprite_rom[1146]=15'h733f; sprite_rom[1147]=15'h7b00; sprite_rom[1148]=15'h6339; sprite_rom[1149]=15'h5216; sprite_rom[1150]=15'he35; sprite_rom[1151]=15'hc83;
sprite_rom[1152]=15'hc63; sprite_rom[1153]=15'h1084; sprite_rom[1154]=15'h4c45; sprite_rom[1155]=15'h62f8; sprite_rom[1156]=15'h6f38; sprite_rom[1157]=15'h775a; sprite_rom[1158]=15'h77bf; sprite_rom[1159]=15'h7fde;
sprite_rom[1160]=15'h7b3f; sprite_rom[1161]=15'h7378; sprite_rom[1162]=15'h6f18; sprite_rom[1163]=15'h454; sprite_rom[1164]=15'h5288; sprite_rom[1165]=15'hc63; sprite_rom[1166]=15'hc63; sprite_rom[1167]=15'h821;
sprite_rom[1168]=15'h2023; sprite_rom[1169]=15'h62f7; sprite_rom[1170]=15'h6738; sprite_rom[1171]=15'h6f7d; sprite_rom[1172]=15'h7ffd; sprite_rom[1173]=15'h7ffe; sprite_rom[1174]=15'h33e; sprite_rom[1175]=15'h7379;
sprite_rom[1176]=15'h7318; sprite_rom[1177]=15'h402b; sprite_rom[1178]=15'h202a; sprite_rom[1179]=15'hc68; sprite_rom[1180]=15'h863; sprite_rom[1181]=15'h568a; sprite_rom[1182]=15'h4e94; sprite_rom[1183]=15'h5ef5;
sprite_rom[1184]=15'h6738; sprite_rom[1185]=15'h77db; sprite_rom[1186]=15'h7fbb; sprite_rom[1187]=15'h7bff; sprite_rom[1188]=15'h37d; sprite_rom[1189]=15'h360; sprite_rom[1190]=15'h6f18; sprite_rom[1191]=15'h7eea;
sprite_rom[1192]=15'h16bf; sprite_rom[1193]=15'hc83; sprite_rom[1194]=15'hc63; sprite_rom[1195]=15'h214a; sprite_rom[1196]=15'h2904; sprite_rom[1197]=15'h5eb0; sprite_rom[1198]=15'h6717; sprite_rom[1199]=15'h7b99;
sprite_rom[1200]=15'h7bbd; sprite_rom[1201]=15'h7bde; sprite_rom[1202]=15'h3dc; sprite_rom[1203]=15'h380; sprite_rom[1204]=15'h5f1b; sprite_rom[1205]=15'h4797; sprite_rom[1206]=15'h57f6; sprite_rom[1207]=15'hc62;
sprite_rom[1208]=15'hc63; sprite_rom[1209]=15'h28a2; sprite_rom[1210]=15'h94e; sprite_rom[1211]=15'h5941; sprite_rom[1212]=15'h6718; sprite_rom[1213]=15'h7759; sprite_rom[1214]=15'h7bde; sprite_rom[1215]=15'h7bde;
sprite_rom[1216]=15'h3bb; sprite_rom[1217]=15'h5f20; sprite_rom[1218]=15'h51d5; sprite_rom[1219]=15'h7354; sprite_rom[1220]=15'hc65; sprite_rom[1221]=15'h863; sprite_rom[1222]=15'h7ec8; sprite_rom[1223]=15'h5fff;
sprite_rom[1224]=15'h402b; sprite_rom[1225]=15'h6317; sprite_rom[1226]=15'h6f5b; sprite_rom[1227]=15'h7bfe; sprite_rom[1228]=15'h7bde; sprite_rom[1229]=15'h1a; sprite_rom[1230]=15'h7380; sprite_rom[1231]=15'h5830;
sprite_rom[1232]=15'h5fff; sprite_rom[1233]=15'hc63; sprite_rom[1234]=15'h863; sprite_rom[1235]=15'h526a; sprite_rom[1236]=15'h7ff6; sprite_rom[1237]=15'h127f; sprite_rom[1238]=15'h5f10; sprite_rom[1239]=15'h73b8;
sprite_rom[1240]=15'h7bbc; sprite_rom[1241]=15'h6ffe; sprite_rom[1242]=15'h1d; sprite_rom[1243]=15'h6000; sprite_rom[1244]=15'h5031; sprite_rom[1245]=15'h5bfe; sprite_rom[1246]=15'hc81; sprite_rom[1247]=15'h1063;
sprite_rom[1248]=15'h5e61; sprite_rom[1249]=15'h73f4; sprite_rom[1250]=15'h2bf6; sprite_rom[1251]=15'h76a3; sprite_rom[1252]=15'h7779; sprite_rom[1253]=15'h7bdd; sprite_rom[1254]=15'h6fbe; sprite_rom[1255]=15'h1f;
sprite_rom[1256]=15'h5b60; sprite_rom[1257]=15'h38b0; sprite_rom[1258]=15'h42b5; sprite_rom[1259]=15'hc81; sprite_rom[1260]=15'h1063; sprite_rom[1261]=15'h7ea1; sprite_rom[1262]=15'h7bff; sprite_rom[1263]=15'h538d;
sprite_rom[1264]=15'h7601; sprite_rom[1265]=15'h7ffe; sprite_rom[1266]=15'h7bdf; sprite_rom[1267]=15'h73bd; sprite_rom[1268]=15'h5780; sprite_rom[1269]=15'h2110; sprite_rom[1270]=15'h116a; sprite_rom[1271]=15'hc83;
sprite_rom[1272]=15'h1063; sprite_rom[1273]=15'h6281; sprite_rom[1274]=15'h639c; sprite_rom[1275]=15'h1607; sprite_rom[1276]=15'h6202; sprite_rom[1277]=15'h579e; sprite_rom[1278]=15'h7ff5; sprite_rom[1279]=15'h35f;
sprite_rom[1280]=15'h6360; sprite_rom[1281]=15'h2110; sprite_rom[1282]=15'hca5; sprite_rom[1283]=15'hca5; sprite_rom[1284]=15'hc63; sprite_rom[1285]=15'h5562; sprite_rom[1286]=15'h3ab5; sprite_rom[1287]=15'h14a1;
sprite_rom[1288]=15'h7285; sprite_rom[1289]=15'h2956; sprite_rom[1290]=15'h7ea1; sprite_rom[1291]=15'h3bb; sprite_rom[1292]=15'h300; sprite_rom[1293]=15'h20b1; sprite_rom[1294]=15'h10a8; sprite_rom[1295]=15'h423;
sprite_rom[1296]=15'hc62; sprite_rom[1297]=15'h28a5; sprite_rom[1298]=15'h1610; sprite_rom[1299]=15'h14a5; sprite_rom[1300]=15'h66c5; sprite_rom[1301]=15'h4215; sprite_rom[1302]=15'h7a28; sprite_rom[1303]=15'h1d;
sprite_rom[1304]=15'h320; sprite_rom[1305]=15'h1434; sprite_rom[1306]=15'h8a5; sprite_rom[1307]=15'h220a; sprite_rom[1308]=15'h1065; sprite_rom[1309]=15'h2105; sprite_rom[1310]=15'h2105; sprite_rom[1311]=15'h4a5;
sprite_rom[1312]=15'h52b3; sprite_rom[1313]=15'h2955; sprite_rom[1314]=15'h7aa5; sprite_rom[1315]=15'h300; sprite_rom[1316]=15'h600; sprite_rom[1317]=15'h4a5; sprite_rom[1318]=15'h42aa; sprite_rom[1319]=15'h950;
sprite_rom[1320]=15'h20a5; sprite_rom[1321]=15'h2108; sprite_rom[1322]=15'h2ca5; sprite_rom[1323]=15'h5554; sprite_rom[1324]=15'h515; sprite_rom[1325]=15'h3eb; sprite_rom[1326]=15'h300; sprite_rom[1327]=15'h3800;
sprite_rom[1328]=15'hc21; sprite_rom[1329]=15'h4203; sprite_rom[1330]=15'h910; sprite_rom[1331]=15'h14a5; sprite_rom[1332]=15'h1505; sprite_rom[1333]=15'h40a5; sprite_rom[1334]=15'h5428; sprite_rom[1335]=15'h4451;
sprite_rom[1336]=15'h4000; sprite_rom[1337]=15'h0; sprite_rom[1338]=15'h0; sprite_rom[1339]=15'h491; sprite_rom[1340]=15'h28a2; sprite_rom[1341]=15'h1068; sprite_rom[1342]=15'h1484; sprite_rom[1343]=15'h425;
sprite_rom[1344]=15'h7288; sprite_rom[1345]=15'h5ebb; sprite_rom[1346]=15'h2d6; sprite_rom[1347]=15'h4210; sprite_rom[1348]=15'h10; sprite_rom[1349]=15'h0; sprite_rom[1350]=15'h3800; sprite_rom[1351]=15'h423;
sprite_rom[1352]=15'h421; sprite_rom[1353]=15'hc41; sprite_rom[1354]=15'h5208; sprite_rom[1355]=15'h7bf8; sprite_rom[1356]=15'h67de; sprite_rom[1357]=15'h2950; sprite_rom[1358]=15'h2a8a; sprite_rom[1359]=15'h0;
sprite_rom[1360]=15'h0; sprite_rom[1361]=15'h2a00; sprite_rom[1362]=15'h2949; sprite_rom[1363]=15'h414a; sprite_rom[1364]=15'h17; sprite_rom[1365]=15'h0; sprite_rom[1366]=15'h6700; sprite_rom[1367]=15'h4000;
sprite_rom[1368]=15'hd10; sprite_rom[1369]=15'h28a5; sprite_rom[1370]=15'h0; sprite_rom[1371]=15'h0; sprite_rom[1372]=15'h0; sprite_rom[1373]=15'h42c0; sprite_rom[1374]=15'h4150; sprite_rom[1375]=15'h15;
sprite_rom[1376]=15'h0; sprite_rom[1377]=15'h360; sprite_rom[1378]=15'h4000; sprite_rom[1379]=15'h20aa; sprite_rom[1380]=15'h4028; sprite_rom[1381]=15'h0; sprite_rom[1382]=15'h0; sprite_rom[1383]=15'h0;
sprite_rom[1384]=15'h2a76; sprite_rom[1385]=15'h414a; sprite_rom[1386]=15'h292; sprite_rom[1387]=15'h0; sprite_rom[1388]=15'h1b; sprite_rom[1389]=15'h8a0; sprite_rom[1390]=15'h5045; sprite_rom[1391]=15'h0;
sprite_rom[1392]=15'h0; sprite_rom[1393]=15'h4e34; sprite_rom[1394]=15'h2a14; sprite_rom[1395]=15'h3dca; sprite_rom[1396]=15'h5253; sprite_rom[1397]=15'h1600; sprite_rom[1398]=15'h5102; sprite_rom[1399]=15'h0;
sprite_rom[1400]=15'h4400; sprite_rom[1401]=15'h4da2; sprite_rom[1402]=15'h2993; sprite_rom[1403]=15'h458b; sprite_rom[1404]=15'h4652; sprite_rom[1405]=15'h289; sprite_rom[1406]=15'h0; sprite_rom[1407]=15'h0;
sprite_rom[1408]=15'h0; sprite_rom[1409]=15'h16; sprite_rom[1410]=15'h4000; sprite_rom[1411]=15'h3504; sprite_rom[1412]=15'h15; sprite_rom[1413]=15'h5c0; sprite_rom[1414]=15'h4da7; sprite_rom[1415]=15'h39af;
sprite_rom[1416]=15'h35e7; sprite_rom[1417]=15'h31b2; sprite_rom[1418]=15'h5041; sprite_rom[1419]=15'h0; sprite_rom[1420]=15'h0; sprite_rom[1421]=15'h5400; sprite_rom[1422]=15'h16; sprite_rom[1423]=15'h3a80;
sprite_rom[1424]=15'h2d45; sprite_rom[1425]=15'h28f; sprite_rom[1426]=15'h1c2b; sprite_rom[1427]=15'h3deb; sprite_rom[1428]=15'h1ded; sprite_rom[1429]=15'h3d8d; sprite_rom[1430]=15'h25af; sprite_rom[1431]=15'h2443;
sprite_rom[1432]=15'h0; sprite_rom[1433]=15'h0; sprite_rom[1434]=15'h56a0; sprite_rom[1435]=15'h3a40; sprite_rom[1436]=15'h2d50; sprite_rom[1437]=15'h396c; sprite_rom[1438]=15'h24c4; sprite_rom[1439]=15'h25cf;
sprite_rom[1440]=15'h352f; sprite_rom[1441]=15'h25ac; sprite_rom[1442]=15'h25ab; sprite_rom[1443]=15'h4c4; sprite_rom[1444]=15'h12; sprite_rom[1445]=15'h0; sprite_rom[1446]=15'h5ea0; sprite_rom[1447]=15'h39c0;
sprite_rom[1448]=15'h2550; sprite_rom[1449]=15'h38ef; sprite_rom[1450]=15'h2c69; sprite_rom[1451]=15'h1d6f; sprite_rom[1452]=15'h316c; sprite_rom[1453]=15'h98c; sprite_rom[1454]=15'h2dac; sprite_rom[1455]=15'h8c6;
sprite_rom[1456]=15'h9; sprite_rom[1457]=15'h0; sprite_rom[1458]=15'h2b5; sprite_rom[1459]=15'h2d36; sprite_rom[1460]=15'h25d1; sprite_rom[1461]=15'h18ef; sprite_rom[1462]=15'h38eb; sprite_rom[1463]=15'h10ef;
sprite_rom[1464]=15'h318d; sprite_rom[1465]=15'h84c; sprite_rom[1466]=15'h2dac; sprite_rom[1467]=15'h10c6; sprite_rom[1468]=15'h282; sprite_rom[1469]=15'h0; sprite_rom[1470]=15'h14; sprite_rom[1471]=15'h24f5;
sprite_rom[1472]=15'h3171; sprite_rom[1473]=15'h18ed; sprite_rom[1474]=15'h3d69; sprite_rom[1475]=15'h2c6f; sprite_rom[1476]=15'h358f; sprite_rom[1477]=15'h827; sprite_rom[1478]=15'h2dad; sprite_rom[1479]=15'h10c6;
sprite_rom[1480]=15'h1c3; sprite_rom[1481]=15'h5400; sprite_rom[1482]=15'h15; sprite_rom[1483]=15'h24d4; sprite_rom[1484]=15'h30f0; sprite_rom[1485]=15'h18ed; sprite_rom[1486]=15'h4569; sprite_rom[1487]=15'h34ef;
sprite_rom[1488]=15'h25ec; sprite_rom[1489]=15'h1862; sprite_rom[1490]=15'h35ae; sprite_rom[1491]=15'h10e7; sprite_rom[1492]=15'h54e6; sprite_rom[1493]=15'h56a0; sprite_rom[1494]=15'h24d4; sprite_rom[1495]=15'h30ee;
sprite_rom[1496]=15'h18ed; sprite_rom[1497]=15'h4569; sprite_rom[1498]=15'h35d1; sprite_rom[1499]=15'hdad; sprite_rom[1500]=15'h24c6; sprite_rom[1501]=15'h39af; sprite_rom[1502]=15'h18e9; sprite_rom[1503]=15'h4486;
sprite_rom[1504]=15'h5ea0; sprite_rom[1505]=15'h24d4; sprite_rom[1506]=15'h34cb; sprite_rom[1507]=15'h1ced; sprite_rom[1508]=15'h4969; sprite_rom[1509]=15'h4a74; sprite_rom[1510]=15'h2572; sprite_rom[1511]=15'h2d29;
sprite_rom[1512]=15'h458f; sprite_rom[1513]=15'h10ea; sprite_rom[1514]=15'h2442; sprite_rom[1515]=15'h2b5; sprite_rom[1516]=15'h24f3; sprite_rom[1517]=15'h34e9; sprite_rom[1518]=15'h1ced; sprite_rom[1519]=15'h49cb;
sprite_rom[1520]=15'h3e75; sprite_rom[1521]=15'h3def; sprite_rom[1522]=15'h4e2f; sprite_rom[1523]=15'h4592; sprite_rom[1524]=15'hccb; sprite_rom[1525]=15'h4849; sprite_rom[1526]=15'h2f5; sprite_rom[1527]=15'h24f3;
sprite_rom[1528]=15'h3526; sprite_rom[1529]=15'h1ccd; sprite_rom[1530]=15'h4a2b; sprite_rom[1531]=15'h4a96; sprite_rom[1532]=15'h35ef; sprite_rom[1533]=15'h522f; sprite_rom[1534]=15'h49b3; sprite_rom[1535]=15'h244e;
sprite_rom[1536]=15'h6d27; sprite_rom[1537]=15'h2bb; sprite_rom[1538]=15'h24f3; sprite_rom[1539]=15'h3586; sprite_rom[1540]=15'h1cce; sprite_rom[1541]=15'h4a4b; sprite_rom[1542]=15'h52b6; sprite_rom[1543]=15'h4e53;
sprite_rom[1544]=15'h5694; sprite_rom[1545]=15'h4d93; sprite_rom[1546]=15'h24ee; sprite_rom[1547]=15'h7e81; sprite_rom[1548]=15'h1d3; sprite_rom[1549]=15'h2532; sprite_rom[1550]=15'h35c6; sprite_rom[1551]=15'h24cd;
sprite_rom[1552]=15'h4a6b; sprite_rom[1553]=15'h4ed6; sprite_rom[1554]=15'h4e52; sprite_rom[1555]=15'h5ab4; sprite_rom[1556]=15'h4973; sprite_rom[1557]=15'h96e; sprite_rom[1558]=15'h67cb; sprite_rom[1559]=15'h5526;
sprite_rom[1560]=15'h25d1; sprite_rom[1561]=15'h35e6; sprite_rom[1562]=15'h24ce; sprite_rom[1563]=15'h4a8e; sprite_rom[1564]=15'h4e76; sprite_rom[1565]=15'h39cb; sprite_rom[1566]=15'h5a92; sprite_rom[1567]=15'h3d33;
sprite_rom[1568]=15'h2d31; sprite_rom[1569]=15'h47f7; sprite_rom[1570]=15'h4482; sprite_rom[1571]=15'h25d2; sprite_rom[1572]=15'h35a7; sprite_rom[1573]=15'h24cd; sprite_rom[1574]=15'h4a8e; sprite_rom[1575]=15'h4295;
sprite_rom[1576]=15'h1064; sprite_rom[1577]=15'h5a91; sprite_rom[1578]=15'h3934; sprite_rom[1579]=15'h4153; sprite_rom[1580]=15'h2f1a; sprite_rom[1581]=15'h1ce7; sprite_rom[1582]=15'h1973; sprite_rom[1583]=15'h35ab;
sprite_rom[1584]=15'h24cc; sprite_rom[1585]=15'h4a73; sprite_rom[1586]=15'h1e74; sprite_rom[1587]=15'h1cc6; sprite_rom[1588]=15'h5249; sprite_rom[1589]=15'h4533; sprite_rom[1590]=15'h2211; sprite_rom[1591]=15'h4605;
sprite_rom[1592]=15'h18e9; sprite_rom[1593]=15'h14; sprite_rom[1594]=15'h1934; sprite_rom[1595]=15'h3dae; sprite_rom[1596]=15'h24eb; sprite_rom[1597]=15'h4a73; sprite_rom[1598]=15'h1e54; sprite_rom[1599]=15'h18c6;
sprite_rom[1600]=15'h4e23; sprite_rom[1601]=15'h3933; sprite_rom[1602]=15'h1551; sprite_rom[1603]=15'h4505; sprite_rom[1604]=15'h1ced; sprite_rom[1605]=15'h13; sprite_rom[1606]=15'h2575; sprite_rom[1607]=15'h3daf;
sprite_rom[1608]=15'h252b; sprite_rom[1609]=15'h4a75; sprite_rom[1610]=15'h1e33; sprite_rom[1611]=15'h18c6; sprite_rom[1612]=15'h4dc2; sprite_rom[1613]=15'h2cf3; sprite_rom[1614]=15'h1513; sprite_rom[1615]=15'h3e02;
sprite_rom[1616]=15'h192d; sprite_rom[1617]=15'h12; sprite_rom[1618]=15'h3a55; sprite_rom[1619]=15'h3daf; sprite_rom[1620]=15'h392b; sprite_rom[1621]=15'h4e36; sprite_rom[1622]=15'h1a32; sprite_rom[1623]=15'h18c3;
sprite_rom[1624]=15'h49c2; sprite_rom[1625]=15'h1d33; sprite_rom[1626]=15'h2ab5; sprite_rom[1627]=15'h2e90; sprite_rom[1628]=15'h198d; sprite_rom[1629]=15'h11; sprite_rom[1630]=15'h5240; sprite_rom[1631]=15'h45ed;
sprite_rom[1632]=15'h5529; sprite_rom[1633]=15'h4db5; sprite_rom[1634]=15'h1a32; sprite_rom[1635]=15'h18c3; sprite_rom[1636]=15'h49c2; sprite_rom[1637]=15'h2532; sprite_rom[1638]=15'h53fa; sprite_rom[1639]=15'h2e30;
sprite_rom[1640]=15'h19ac; sprite_rom[1641]=15'h10; sprite_rom[1642]=15'h4e80; sprite_rom[1643]=15'h3e2f; sprite_rom[1644]=15'h288; sprite_rom[1645]=15'h49f4; sprite_rom[1646]=15'h1a32; sprite_rom[1647]=15'h1863;
sprite_rom[1648]=15'h4962; sprite_rom[1649]=15'h4d32; sprite_rom[1650]=15'h4eff; sprite_rom[1651]=15'h25c9; sprite_rom[1652]=15'h1dec; sprite_rom[1653]=15'hb; sprite_rom[1654]=15'h36a0; sprite_rom[1655]=15'h4651;
sprite_rom[1656]=15'h15; sprite_rom[1657]=15'h4df3; sprite_rom[1658]=15'h1232; sprite_rom[1659]=15'hc64; sprite_rom[1660]=15'h4962; sprite_rom[1661]=15'h4572; sprite_rom[1662]=15'h5237; sprite_rom[1663]=15'h2d69;
sprite_rom[1664]=15'h31e9; sprite_rom[1665]=15'h2a6; sprite_rom[1666]=15'h5000; sprite_rom[1667]=15'h2d4; sprite_rom[1668]=15'h0; sprite_rom[1669]=15'h4e31; sprite_rom[1670]=15'h1231; sprite_rom[1671]=15'hc64;
sprite_rom[1672]=15'h4962; sprite_rom[1673]=15'h9d2; sprite_rom[1674]=15'h5169; sprite_rom[1675]=15'h2d2d; sprite_rom[1676]=15'h3de7; sprite_rom[1677]=15'h266; sprite_rom[1678]=15'h4deb; sprite_rom[1679]=15'h11f1;
sprite_rom[1680]=15'hc83; sprite_rom[1681]=15'h4522; sprite_rom[1682]=15'h19f1; sprite_rom[1683]=15'h4d26; sprite_rom[1684]=15'h2cd2; sprite_rom[1685]=15'h3da2; sprite_rom[1686]=15'h229; sprite_rom[1687]=15'h5400;
sprite_rom[1688]=15'h4e2b; sprite_rom[1689]=15'hdf1; sprite_rom[1690]=15'hcc2; sprite_rom[1691]=15'h4522; sprite_rom[1692]=15'h1e31; sprite_rom[1693]=15'h44e6; sprite_rom[1694]=15'h1d34; sprite_rom[1695]=15'h3d82;
sprite_rom[1696]=15'h22d; sprite_rom[1697]=15'h4c00; sprite_rom[1698]=15'h4e4b; sprite_rom[1699]=15'h11cf; sprite_rom[1700]=15'hcc2; sprite_rom[1701]=15'h4522; sprite_rom[1702]=15'h2631; sprite_rom[1703]=15'h38c6;
sprite_rom[1704]=15'h19f3; sprite_rom[1705]=15'h3d86; sprite_rom[1706]=15'h26d; sprite_rom[1707]=15'h4000; sprite_rom[1708]=15'h4e2b; sprite_rom[1709]=15'h11cf; sprite_rom[1710]=15'hcc2; sprite_rom[1711]=15'h4522;
sprite_rom[1712]=15'h2e31; sprite_rom[1713]=15'h2484; sprite_rom[1714]=15'h2e91; sprite_rom[1715]=15'h3d82; sprite_rom[1716]=15'h28f; sprite_rom[1717]=15'h2c00; sprite_rom[1718]=15'h4ded; sprite_rom[1719]=15'h11cf;
sprite_rom[1720]=15'h18c2; sprite_rom[1721]=15'h4522; sprite_rom[1722]=15'h3232; sprite_rom[1723]=15'h2464; sprite_rom[1724]=15'h4e6e; sprite_rom[1725]=15'h4567; sprite_rom[1726]=15'h2f2; sprite_rom[1727]=15'h2400;
sprite_rom[1728]=15'h4dee; sprite_rom[1729]=15'h19cf; sprite_rom[1730]=15'h18c2; sprite_rom[1731]=15'h3d02; sprite_rom[1732]=15'h39f2; sprite_rom[1733]=15'h1866; sprite_rom[1734]=15'h524b; sprite_rom[1735]=15'h4a32;
sprite_rom[1736]=15'h13; sprite_rom[1737]=15'h12a0; sprite_rom[1738]=15'h4def; sprite_rom[1739]=15'h19cf; sprite_rom[1740]=15'h18c2; sprite_rom[1741]=15'h3ce2; sprite_rom[1742]=15'h3df2; sprite_rom[1743]=15'h1867;
sprite_rom[1744]=15'h55cb; sprite_rom[1745]=15'h4a74; sprite_rom[1746]=15'h16; sprite_rom[1747]=15'h1260; sprite_rom[1748]=15'h4def; sprite_rom[1749]=15'h19cf; sprite_rom[1750]=15'h18c2; sprite_rom[1751]=15'h3ce2;
sprite_rom[1752]=15'h3e32; sprite_rom[1753]=15'hc69; sprite_rom[1754]=15'h4d49; sprite_rom[1755]=15'h5a94; sprite_rom[1756]=15'h1e20; sprite_rom[1757]=15'h4df1; sprite_rom[1758]=15'h19cf; sprite_rom[1759]=15'h1862;
sprite_rom[1760]=15'h3ce2; sprite_rom[1761]=15'h3e2f; sprite_rom[1762]=15'hc6b; sprite_rom[1763]=15'h4549; sprite_rom[1764]=15'h2b3; sprite_rom[1765]=15'h2560; sprite_rom[1766]=15'h4def; sprite_rom[1767]=15'h19cd;
sprite_rom[1768]=15'h1862; sprite_rom[1769]=15'h38c2; sprite_rom[1770]=15'h364f; sprite_rom[1771]=15'hc6d; sprite_rom[1772]=15'h2d66; sprite_rom[1773]=15'h253; sprite_rom[1774]=15'h2cc0; sprite_rom[1775]=15'h4e2f;
sprite_rom[1776]=15'h19cf; sprite_rom[1777]=15'h1842; sprite_rom[1778]=15'h3863; sprite_rom[1779]=15'h364f; sprite_rom[1780]=15'hc8f; sprite_rom[1781]=15'h2524; sprite_rom[1782]=15'h5633; sprite_rom[1783]=15'h3854;
sprite_rom[1784]=15'h4a2f; sprite_rom[1785]=15'h19d2; sprite_rom[1786]=15'h1842; sprite_rom[1787]=15'h3c46; sprite_rom[1788]=15'h3252; sprite_rom[1789]=15'h8f1; sprite_rom[1790]=15'h2522; sprite_rom[1791]=15'h4a51;
sprite_rom[1792]=15'h3c32; sprite_rom[1793]=15'h3a2d; sprite_rom[1794]=15'h1929; sprite_rom[1795]=15'h1842; sprite_rom[1796]=15'h2446; sprite_rom[1797]=15'h322e; sprite_rom[1798]=15'h931; sprite_rom[1799]=15'h24e2;
sprite_rom[1800]=15'h3e6b; sprite_rom[1801]=15'h44ce; sprite_rom[1802]=15'h3a4c; sprite_rom[1803]=15'h1d22; sprite_rom[1804]=15'hc42; sprite_rom[1805]=15'hcc6; sprite_rom[1806]=15'h3224; sprite_rom[1807]=15'h96f;
sprite_rom[1808]=15'h24c3; sprite_rom[1809]=15'h3e67; sprite_rom[1810]=15'h13; sprite_rom[1811]=15'h44e9; sprite_rom[1812]=15'h2e4c; sprite_rom[1813]=15'h1d26; sprite_rom[1814]=15'hc42; sprite_rom[1815]=15'h10c6;
sprite_rom[1816]=15'h3623; sprite_rom[1817]=15'h9ad; sprite_rom[1818]=15'h2c63; sprite_rom[1819]=15'h4a27; sprite_rom[1820]=15'hf; sprite_rom[1821]=15'h4521; sprite_rom[1822]=15'h2e4d; sprite_rom[1823]=15'h1ce3;
sprite_rom[1824]=15'hc42; sprite_rom[1825]=15'h18c3; sprite_rom[1826]=15'h35c2; sprite_rom[1827]=15'h11ed; sprite_rom[1828]=15'h2464; sprite_rom[1829]=15'h4d67; sprite_rom[1830]=15'h2ad; sprite_rom[1831]=15'h4a31;
sprite_rom[1832]=15'h262c; sprite_rom[1833]=15'h1ce3; sprite_rom[1834]=15'hc63; sprite_rom[1835]=15'h18c3; sprite_rom[1836]=15'h3d62; sprite_rom[1837]=15'h1e2c; sprite_rom[1838]=15'h1842; sprite_rom[1839]=15'h5169;
sprite_rom[1840]=15'h24f; sprite_rom[1841]=15'h4a00; sprite_rom[1842]=15'h2652; sprite_rom[1843]=15'h1d26; sprite_rom[1844]=15'hc63; sprite_rom[1845]=15'h18c3; sprite_rom[1846]=15'h4d62; sprite_rom[1847]=15'h3a4f;
sprite_rom[1848]=15'h2d29; sprite_rom[1849]=15'h4a32; sprite_rom[1850]=15'h1f2; sprite_rom[1851]=15'h1c53; sprite_rom[1852]=15'h196b; sprite_rom[1853]=15'h1cc3; sprite_rom[1854]=15'hc86; sprite_rom[1855]=15'h18e6;
sprite_rom[1856]=15'h38c2; sprite_rom[1857]=15'h2d6e; sprite_rom[1858]=15'h420e; sprite_rom[1859]=15'h2571; sprite_rom[1860]=15'h2f6; sprite_rom[1861]=15'h8f4; sprite_rom[1862]=15'h45c6; sprite_rom[1863]=15'h1d6e;
sprite_rom[1864]=15'hc42; sprite_rom[1865]=15'h1842; sprite_rom[1866]=15'h25ab; sprite_rom[1867]=15'h10e7; sprite_rom[1868]=15'h18c2; sprite_rom[1869]=15'h4629; sprite_rom[1870]=15'h4c00; sprite_rom[1871]=15'h1e75;
sprite_rom[1872]=15'h4566; sprite_rom[1873]=15'h1d2b; sprite_rom[1874]=15'h4dc6; sprite_rom[1875]=15'h2d6e; sprite_rom[1876]=15'h3a31; sprite_rom[1877]=15'h24e7; sprite_rom[1878]=15'h311; sprite_rom[1879]=15'h560;
sprite_rom[1880]=15'hc21; sprite_rom[1881]=15'h5a91; sprite_rom[1882]=15'h27a0; sprite_rom[1883]=15'h421; sprite_rom[1884]=15'h5561; sprite_rom[1885]=15'h19; sprite_rom[1886]=15'h4c00; sprite_rom[1887]=15'h4e52;
sprite_rom[1888]=15'h15; sprite_rom[1889]=15'h4c00; sprite_rom[1890]=15'h39ae; sprite_rom[1891]=15'h291; sprite_rom[1892]=15'h6320; sprite_rom[1893]=15'h378; sprite_rom[1894]=15'h5b00; sprite_rom[1895]=15'h66f6;
sprite_rom[1896]=15'h735a; sprite_rom[1897]=15'h3dd; sprite_rom[1898]=15'h6eb7; sprite_rom[1899]=15'h6338; sprite_rom[1900]=15'h6f58; sprite_rom[1901]=15'h7bbd; sprite_rom[1902]=15'h7b76; sprite_rom[1903]=15'h675d;
sprite_rom[1904]=15'h6f18; sprite_rom[1905]=15'h7b9b; sprite_rom[1906]=15'h1d; sprite_rom[1907]=15'h5bbb; sprite_rom[1908]=15'h7bb7; sprite_rom[1909]=15'h67bd; sprite_rom[1910]=15'h7739; sprite_rom[1911]=15'h37e;
sprite_rom[1912]=15'h231; sprite_rom[1913]=15'h0; sprite_rom[1914]=15'h1d94; sprite_rom[1915]=15'h7269; sprite_rom[1916]=15'h77fe; sprite_rom[1917]=15'h6f5a; sprite_rom[1918]=15'h6f9e; sprite_rom[1919]=15'h4e94;
sprite_rom[1920]=15'h3ab4; sprite_rom[1921]=15'h1d29; sprite_rom[1922]=15'h398d; sprite_rom[1923]=15'h768e; sprite_rom[1924]=15'h77bf; sprite_rom[1925]=15'h6bbe; sprite_rom[1926]=15'h56fc; sprite_rom[1927]=15'h4674;
sprite_rom[1928]=15'h49cd; sprite_rom[1929]=15'h2cef; sprite_rom[1930]=15'h7bf6; sprite_rom[1931]=15'h6fde; sprite_rom[1932]=15'h5c00; sprite_rom[1933]=15'h5ef7; sprite_rom[1934]=15'h7b99; sprite_rom[1935]=15'h7bde;
sprite_rom[1936]=15'h52f9; sprite_rom[1937]=15'h324f; sprite_rom[1938]=15'h7ecc; sprite_rom[1939]=15'h73df; sprite_rom[1940]=15'h6000; sprite_rom[1941]=15'h5ed6; sprite_rom[1942]=15'h6f59; sprite_rom[1943]=15'h6f7b;
sprite_rom[1944]=15'h737c; sprite_rom[1945]=15'h77bd; sprite_rom[1946]=15'h56da; sprite_rom[1947]=15'h6251; sprite_rom[1948]=15'h3ff; sprite_rom[1949]=15'h5ac0; sprite_rom[1950]=15'h6f58; sprite_rom[1951]=15'h5ef9;
sprite_rom[1952]=15'h671a; sprite_rom[1953]=15'h6f7a; sprite_rom[1954]=15'h737b; sprite_rom[1955]=15'h7ffe; sprite_rom[1956]=15'h5297; sprite_rom[1957]=15'h2d4; sprite_rom[1958]=15'h6000; sprite_rom[1959]=15'h6af6;
sprite_rom[1960]=15'h635a; sprite_rom[1961]=15'h66f7; sprite_rom[1962]=15'h6318; sprite_rom[1963]=15'h6739; sprite_rom[1964]=15'h6f79; sprite_rom[1965]=15'h779c; sprite_rom[1966]=15'h7b9e; sprite_rom[1967]=15'h52bd;
sprite_rom[1968]=15'h4654; sprite_rom[1969]=15'h56e0; sprite_rom[1970]=15'h6359; sprite_rom[1971]=15'h6b99; sprite_rom[1972]=15'h6f9a; sprite_rom[1973]=15'h6f7b; sprite_rom[1974]=15'h633a; sprite_rom[1975]=15'h6f37;
sprite_rom[1976]=15'h737b; sprite_rom[1977]=15'h7bdc; sprite_rom[1978]=15'h1e; sprite_rom[1979]=15'h3a80; sprite_rom[1980]=15'h15; sprite_rom[1981]=15'h66d7; sprite_rom[1982]=15'h6b7a; sprite_rom[1983]=15'h77bb;
sprite_rom[1984]=15'h77dc; sprite_rom[1985]=15'h6f7c; sprite_rom[1986]=15'h675a; sprite_rom[1987]=15'h66f8; sprite_rom[1988]=15'h737a; sprite_rom[1989]=15'h7bbd; sprite_rom[1990]=15'h37e; sprite_rom[1991]=15'h5c00;
sprite_rom[1992]=15'h6f76; sprite_rom[1993]=15'h737b; sprite_rom[1994]=15'h7bbd; sprite_rom[1995]=15'h7bfe; sprite_rom[1996]=15'h77be; sprite_rom[1997]=15'h6b7c; sprite_rom[1998]=15'h6739; sprite_rom[1999]=15'h737a;
sprite_rom[2000]=15'h779d; sprite_rom[2001]=15'h6fdf; sprite_rom[2002]=15'h5c00; sprite_rom[2003]=15'h6b5b; sprite_rom[2004]=15'h6b39; sprite_rom[2005]=15'h777b; sprite_rom[2006]=15'h7bff; sprite_rom[2007]=15'h739d;
sprite_rom[2008]=15'h6f9c; sprite_rom[2009]=15'h673a; sprite_rom[2010]=15'h6b38; sprite_rom[2011]=15'h779b; sprite_rom[2012]=15'h77fe; sprite_rom[2013]=15'h1b; sprite_rom[2014]=15'h62e0; sprite_rom[2015]=15'h6717;
sprite_rom[2016]=15'h6739; sprite_rom[2017]=15'h775a; sprite_rom[2018]=15'h6b5d; sprite_rom[2019]=15'h6f7b; sprite_rom[2020]=15'h6f5a; sprite_rom[2021]=15'h673c; sprite_rom[2022]=15'h6b18; sprite_rom[2023]=15'h777a;
sprite_rom[2024]=15'h7bde; sprite_rom[2025]=15'h39b; sprite_rom[2026]=15'h5ef8; sprite_rom[2027]=15'h6717; sprite_rom[2028]=15'h7b38; sprite_rom[2029]=15'h6f7e; sprite_rom[2030]=15'h775a; sprite_rom[2031]=15'h6b5c;
sprite_rom[2032]=15'h673a; sprite_rom[2033]=15'h6b9a; sprite_rom[2034]=15'h6b38; sprite_rom[2035]=15'h77bb; sprite_rom[2036]=15'h7bbd; sprite_rom[2037]=15'h33f; sprite_rom[2038]=15'h5ef6; sprite_rom[2039]=15'h5ef6;
sprite_rom[2040]=15'h7ffb; sprite_rom[2041]=15'h73fd; sprite_rom[2042]=15'h7bdc; sprite_rom[2043]=15'h679f; sprite_rom[2044]=15'h673a; sprite_rom[2045]=15'h7339; sprite_rom[2046]=15'h6b5a; sprite_rom[2047]=15'h6f7a;
sprite_rom[2048]=15'h77bd; sprite_rom[2049]=15'h6f5f; sprite_rom[2050]=15'h5c00; sprite_rom[2051]=15'h5f37; sprite_rom[2052]=15'h62f7; sprite_rom[2053]=15'h4297; sprite_rom[2054]=15'h5a91; sprite_rom[2055]=15'h62f7;
sprite_rom[2056]=15'h67fe; sprite_rom[2057]=15'h6739; sprite_rom[2058]=15'h6739; sprite_rom[2059]=15'h631c; sprite_rom[2060]=15'h6f79; sprite_rom[2061]=15'h7bbc; sprite_rom[2062]=15'h6fbf; sprite_rom[2063]=15'h5ae0;
sprite_rom[2064]=15'h673b; sprite_rom[2065]=15'h5758; sprite_rom[2066]=15'h14a8; sprite_rom[2067]=15'h14a5; sprite_rom[2068]=15'h2908; sprite_rom[2069]=15'h66b0; sprite_rom[2070]=15'h6717; sprite_rom[2071]=15'h5f18;
sprite_rom[2072]=15'h67ba; sprite_rom[2073]=15'h6f59; sprite_rom[2074]=15'h7bbc; sprite_rom[2075]=15'h6bde; sprite_rom[2076]=15'h6ec0; sprite_rom[2077]=15'h6339; sprite_rom[2078]=15'h2ad9; sprite_rom[2079]=15'h14a5;
sprite_rom[2080]=15'h14a5; sprite_rom[2081]=15'h14a5; sprite_rom[2082]=15'h5945; sprite_rom[2083]=15'h5ed7; sprite_rom[2084]=15'h5ef7; sprite_rom[2085]=15'h77b7; sprite_rom[2086]=15'h737b; sprite_rom[2087]=15'h7bbd;
sprite_rom[2088]=15'h67de; sprite_rom[2089]=15'h1d; sprite_rom[2090]=15'h6f37; sprite_rom[2091]=15'h6339; sprite_rom[2092]=15'h1618; sprite_rom[2093]=15'h14a5; sprite_rom[2094]=15'h1084; sprite_rom[2095]=15'h14a5;
sprite_rom[2096]=15'h50a5; sprite_rom[2097]=15'h5ef9; sprite_rom[2098]=15'h6317; sprite_rom[2099]=15'h7739; sprite_rom[2100]=15'h739b; sprite_rom[2101]=15'h77bb; sprite_rom[2102]=15'h73de; sprite_rom[2103]=15'h1b;
sprite_rom[2104]=15'h67d7; sprite_rom[2105]=15'h6f39; sprite_rom[2106]=15'h1494; sprite_rom[2107]=15'h1084; sprite_rom[2108]=15'h1084; sprite_rom[2109]=15'h1084; sprite_rom[2110]=15'h2885; sprite_rom[2111]=15'h6318;
sprite_rom[2112]=15'h6738; sprite_rom[2113]=15'h6f39; sprite_rom[2114]=15'h6f7d; sprite_rom[2115]=15'h779b; sprite_rom[2116]=15'h77de; sprite_rom[2117]=15'h19; sprite_rom[2118]=15'h5c00; sprite_rom[2119]=15'h679b;
sprite_rom[2120]=15'h6359; sprite_rom[2121]=15'h44a; sprite_rom[2122]=15'hc83; sprite_rom[2123]=15'hc63; sprite_rom[2124]=15'h1463; sprite_rom[2125]=15'h1484; sprite_rom[2126]=15'h5f35; sprite_rom[2127]=15'h6739;
sprite_rom[2128]=15'h673a; sprite_rom[2129]=15'h73dd; sprite_rom[2130]=15'h77bc; sprite_rom[2131]=15'h77fe; sprite_rom[2132]=15'h19; sprite_rom[2133]=15'h6000; sprite_rom[2134]=15'h671e; sprite_rom[2135]=15'h579a;
sprite_rom[2136]=15'h4691; sprite_rom[2137]=15'hc24; sprite_rom[2138]=15'hc63; sprite_rom[2139]=15'h1063; sprite_rom[2140]=15'hc84; sprite_rom[2141]=15'h5f10; sprite_rom[2142]=15'h6739; sprite_rom[2143]=15'h6b7a;
sprite_rom[2144]=15'h7fdb; sprite_rom[2145]=15'h7bbd; sprite_rom[2146]=15'h77ff; sprite_rom[2147]=15'h1a; sprite_rom[2148]=15'h6c00; sprite_rom[2149]=15'h631e; sprite_rom[2150]=15'h2b7b; sprite_rom[2151]=15'h50c1;
sprite_rom[2152]=15'h8d4; sprite_rom[2153]=15'hc63; sprite_rom[2154]=15'h443; sprite_rom[2155]=15'h10a1; sprite_rom[2156]=15'h62a1; sprite_rom[2157]=15'h6738; sprite_rom[2158]=15'h7779; sprite_rom[2159]=15'h7b9b;
sprite_rom[2160]=15'h7bff; sprite_rom[2161]=15'h73ff; sprite_rom[2162]=15'h1c; sprite_rom[2163]=15'h2b7; sprite_rom[2164]=15'h0; sprite_rom[2165]=15'h0; sprite_rom[2166]=15'h0; sprite_rom[2167]=15'h0;
sprite_rom[2168]=15'h0; sprite_rom[2169]=15'h0; sprite_rom[2170]=15'h0; sprite_rom[2171]=15'h6c00; sprite_rom[2172]=15'h6300; sprite_rom[2173]=15'h16fc; sprite_rom[2174]=15'h948; sprite_rom[2175]=15'h948;
sprite_rom[2176]=15'hc63; sprite_rom[2177]=15'h4502; sprite_rom[2178]=15'h56b4; sprite_rom[2179]=15'h6293; sprite_rom[2180]=15'h6737; sprite_rom[2181]=15'h7739; sprite_rom[2182]=15'h739e; sprite_rom[2183]=15'h7ffd;
sprite_rom[2184]=15'h73fe; sprite_rom[2185]=15'h1e; sprite_rom[2186]=15'h56e0; sprite_rom[2187]=15'h15; sprite_rom[2188]=15'h0; sprite_rom[2189]=15'h0; sprite_rom[2190]=15'h0; sprite_rom[2191]=15'h0;
sprite_rom[2192]=15'h0; sprite_rom[2193]=15'h0; sprite_rom[2194]=15'h7000; sprite_rom[2195]=15'h5f60; sprite_rom[2196]=15'h42dd; sprite_rom[2197]=15'h73ff; sprite_rom[2198]=15'h104a; sprite_rom[2199]=15'hc63;
sprite_rom[2200]=15'h3942; sprite_rom[2201]=15'h108a; sprite_rom[2202]=15'h4d08; sprite_rom[2203]=15'h6318; sprite_rom[2204]=15'h6b39; sprite_rom[2205]=15'h7bdd; sprite_rom[2206]=15'h7bde; sprite_rom[2207]=15'h73de;
sprite_rom[2208]=15'h1d; sprite_rom[2209]=15'h2b5; sprite_rom[2210]=15'h0; sprite_rom[2211]=15'h0; sprite_rom[2212]=15'h0; sprite_rom[2213]=15'h0; sprite_rom[2214]=15'h0; sprite_rom[2215]=15'h0;
sprite_rom[2216]=15'h7400; sprite_rom[2217]=15'h63a0; sprite_rom[2218]=15'h72bc; sprite_rom[2219]=15'h7e98; sprite_rom[2220]=15'h91e; sprite_rom[2221]=15'hc63; sprite_rom[2222]=15'h843; sprite_rom[2223]=15'h420a;
sprite_rom[2224]=15'h1028; sprite_rom[2225]=15'h6314; sprite_rom[2226]=15'h6b58; sprite_rom[2227]=15'h7bbb; sprite_rom[2228]=15'h7bde; sprite_rom[2229]=15'h6ffe; sprite_rom[2230]=15'h1d; sprite_rom[2231]=15'h56e0;
sprite_rom[2232]=15'h15; sprite_rom[2233]=15'h0; sprite_rom[2234]=15'h0; sprite_rom[2235]=15'h0; sprite_rom[2236]=15'h0; sprite_rom[2237]=15'h0; sprite_rom[2238]=15'h0; sprite_rom[2239]=15'h7000;
sprite_rom[2240]=15'h56f9; sprite_rom[2241]=15'h5692; sprite_rom[2242]=15'h61f; sprite_rom[2243]=15'hc63; sprite_rom[2244]=15'h4c43; sprite_rom[2245]=15'h7fff; sprite_rom[2246]=15'h2afe; sprite_rom[2247]=15'h5e65;
sprite_rom[2248]=15'h6f38; sprite_rom[2249]=15'h7b7a; sprite_rom[2250]=15'h7fff; sprite_rom[2251]=15'h6bdf; sprite_rom[2252]=15'h5ab5; sprite_rom[2253]=15'h0; sprite_rom[2254]=15'h0; sprite_rom[2255]=15'h0;
sprite_rom[2256]=15'h0; sprite_rom[2257]=15'h0; sprite_rom[2258]=15'h0; sprite_rom[2259]=15'h0; sprite_rom[2260]=15'h22fb; sprite_rom[2261]=15'h7fee; sprite_rom[2262]=15'hd1c; sprite_rom[2263]=15'hc63;
sprite_rom[2264]=15'h50c2; sprite_rom[2265]=15'h7ed5; sprite_rom[2266]=15'h7fb9; sprite_rom[2267]=15'h502a; sprite_rom[2268]=15'h6b1d; sprite_rom[2269]=15'h779d; sprite_rom[2270]=15'h7fdd; sprite_rom[2271]=15'h7b5e;
sprite_rom[2272]=15'h56e0; sprite_rom[2273]=15'h15; sprite_rom[2274]=15'h0; sprite_rom[2275]=15'h0; sprite_rom[2276]=15'h0; sprite_rom[2277]=15'h0; sprite_rom[2278]=15'h0; sprite_rom[2279]=15'h0;
sprite_rom[2280]=15'h2b16; sprite_rom[2281]=15'h7b89; sprite_rom[2282]=15'hc78; sprite_rom[2283]=15'hc63; sprite_rom[2284]=15'h4824; sprite_rom[2285]=15'h7e96; sprite_rom[2286]=15'h61df; sprite_rom[2287]=15'h2118;
sprite_rom[2288]=15'h6fd6; sprite_rom[2289]=15'h7bdc; sprite_rom[2290]=15'h7bde; sprite_rom[2291]=15'h37e; sprite_rom[2292]=15'h56b5; sprite_rom[2293]=15'h0; sprite_rom[2294]=15'h0; sprite_rom[2295]=15'h0;
sprite_rom[2296]=15'h0; sprite_rom[2297]=15'h0; sprite_rom[2298]=15'h0; sprite_rom[2299]=15'h42bb; sprite_rom[2300]=15'h5aa8; sprite_rom[2301]=15'h1033; sprite_rom[2302]=15'hc63; sprite_rom[2303]=15'h5424;
sprite_rom[2304]=15'h7fff; sprite_rom[2305]=15'h2d3f; sprite_rom[2306]=15'h61f; sprite_rom[2307]=15'h7bd4; sprite_rom[2308]=15'h7fff; sprite_rom[2309]=15'h7fde; sprite_rom[2310]=15'h39d; sprite_rom[2311]=15'h5700;
sprite_rom[2312]=15'h15; sprite_rom[2313]=15'h0; sprite_rom[2314]=15'h0; sprite_rom[2315]=15'h0; sprite_rom[2316]=15'h0; sprite_rom[2317]=15'h0; sprite_rom[2318]=15'h42bc; sprite_rom[2319]=15'h4148;
sprite_rom[2320]=15'h1026; sprite_rom[2321]=15'hc64; sprite_rom[2322]=15'h4c24; sprite_rom[2323]=15'h7398; sprite_rom[2324]=15'h1c3b; sprite_rom[2325]=15'h1050; sprite_rom[2326]=15'h7734; sprite_rom[2327]=15'h5e97;
sprite_rom[2328]=15'h7bff; sprite_rom[2329]=15'h1a; sprite_rom[2330]=15'h6eb6; sprite_rom[2331]=15'h0; sprite_rom[2332]=15'h0; sprite_rom[2333]=15'h0; sprite_rom[2334]=15'h0; sprite_rom[2335]=15'h0;
sprite_rom[2336]=15'h2afb; sprite_rom[2337]=15'h1508; sprite_rom[2338]=15'h1465; sprite_rom[2339]=15'h1062; sprite_rom[2340]=15'h2423; sprite_rom[2341]=15'h5ab5; sprite_rom[2342]=15'h1431; sprite_rom[2343]=15'h14a4;
sprite_rom[2344]=15'h5395; sprite_rom[2345]=15'h50a; sprite_rom[2346]=15'h6ff7; sprite_rom[2347]=15'h7bc0; sprite_rom[2348]=15'h43dc; sprite_rom[2349]=15'h0; sprite_rom[2350]=15'h0; sprite_rom[2351]=15'h0;
sprite_rom[2352]=15'h0; sprite_rom[2353]=15'h2819; sprite_rom[2354]=15'h2105; sprite_rom[2355]=15'h20a5; sprite_rom[2356]=15'h846; sprite_rom[2357]=15'h1483; sprite_rom[2358]=15'h420a; sprite_rom[2359]=15'h1505;
sprite_rom[2360]=15'h2845; sprite_rom[2361]=15'h56f7; sprite_rom[2362]=15'h1610; sprite_rom[2363]=15'h77d4; sprite_rom[2364]=15'h4ffe; sprite_rom[2365]=15'h2905; sprite_rom[2366]=15'h13; sprite_rom[2367]=15'h0;
sprite_rom[2368]=15'h0; sprite_rom[2369]=15'h4419; sprite_rom[2370]=15'h14a1; sprite_rom[2371]=15'h5445; sprite_rom[2372]=15'h2a15; sprite_rom[2373]=15'h14a3; sprite_rom[2374]=15'h2108; sprite_rom[2375]=15'h2108;
sprite_rom[2376]=15'h5465; sprite_rom[2377]=15'h56b4; sprite_rom[2378]=15'h154a; sprite_rom[2379]=15'h3f6; sprite_rom[2380]=15'h7e8a; sprite_rom[2381]=15'h67d; sprite_rom[2382]=15'h201; sprite_rom[2383]=15'h0;
sprite_rom[2384]=15'h0; sprite_rom[2385]=15'h18; sprite_rom[2386]=15'h142b; sprite_rom[2387]=15'h4024; sprite_rom[2388]=15'h4210; sprite_rom[2389]=15'h14a5; sprite_rom[2390]=15'h2108; sprite_rom[2391]=15'h1508;
sprite_rom[2392]=15'h4e05; sprite_rom[2393]=15'h52ca; sprite_rom[2394]=15'h4048; sprite_rom[2395]=15'h2a00; sprite_rom[2396]=15'h53f3; sprite_rom[2397]=15'h4041; sprite_rom[2398]=15'h14; sprite_rom[2399]=15'h0;
sprite_rom[2400]=15'h18; sprite_rom[2401]=15'h5c0; sprite_rom[2402]=15'h2021; sprite_rom[2403]=15'h2a10; sprite_rom[2404]=15'h1483; sprite_rom[2405]=15'h20a5; sprite_rom[2406]=15'h8a5; sprite_rom[2407]=15'h126a;
sprite_rom[2408]=15'h3ae2; sprite_rom[2409]=15'h222; sprite_rom[2410]=15'h4000; sprite_rom[2411]=15'h7e03; sprite_rom[2412]=15'h2055; sprite_rom[2413]=15'h51ab; sprite_rom[2414]=15'h0; sprite_rom[2415]=15'h0;
sprite_rom[2416]=15'h4400; sprite_rom[2417]=15'h828; sprite_rom[2418]=15'h8a8; sprite_rom[2419]=15'hc64; sprite_rom[2420]=15'h863; sprite_rom[2421]=15'h4502; sprite_rom[2422]=15'h63f7; sprite_rom[2423]=15'h5b15;
sprite_rom[2424]=15'h5550; sprite_rom[2425]=15'h2031; sprite_rom[2426]=15'h318b; sprite_rom[2427]=15'h227; sprite_rom[2428]=15'h0; sprite_rom[2429]=15'h0; sprite_rom[2430]=15'h2a20; sprite_rom[2431]=15'h421;
sprite_rom[2432]=15'h1041; sprite_rom[2433]=15'h4105; sprite_rom[2434]=15'h7b75; sprite_rom[2435]=15'h7fbe; sprite_rom[2436]=15'h37e; sprite_rom[2437]=15'h2954; sprite_rom[2438]=15'h352b; sprite_rom[2439]=15'h1027;
sprite_rom[2440]=15'h26b; sprite_rom[2441]=15'h0; sprite_rom[2442]=15'h0; sprite_rom[2443]=15'h4200; sprite_rom[2444]=15'h4215; sprite_rom[2445]=15'h5e0a; sprite_rom[2446]=15'h0; sprite_rom[2447]=15'h0;
sprite_rom[2448]=15'h18; sprite_rom[2449]=15'h4273; sprite_rom[2450]=15'h352a; sprite_rom[2451]=15'h88b; sprite_rom[2452]=15'h1c26; sprite_rom[2453]=15'h522b; sprite_rom[2454]=15'h14; sprite_rom[2455]=15'h0;
sprite_rom[2456]=15'h2a16; sprite_rom[2457]=15'h4d4a; sprite_rom[2458]=15'h16; sprite_rom[2459]=15'h0; sprite_rom[2460]=15'h1b; sprite_rom[2461]=15'h45b2; sprite_rom[2462]=15'h34ee; sprite_rom[2463]=15'h8cc;
sprite_rom[2464]=15'h4c6; sprite_rom[2465]=15'h843; sprite_rom[2466]=15'h1cc4; sprite_rom[2467]=15'h4e2b; sprite_rom[2468]=15'h29d3; sprite_rom[2469]=15'h4a0a; sprite_rom[2470]=15'h2d3; sprite_rom[2471]=15'h4171;
sprite_rom[2472]=15'h34ec; sprite_rom[2473]=15'hcec; sprite_rom[2474]=15'h1cc2; sprite_rom[2475]=15'h18c3; sprite_rom[2476]=15'h18c3; sprite_rom[2477]=15'h4de7; sprite_rom[2478]=15'h29d3; sprite_rom[2479]=15'h49aa;
sprite_rom[2480]=15'h5273; sprite_rom[2481]=15'h392f; sprite_rom[2482]=15'h34cb; sprite_rom[2483]=15'h18ec; sprite_rom[2484]=15'h2063; sprite_rom[2485]=15'h1866; sprite_rom[2486]=15'h1cc6; sprite_rom[2487]=15'h4de7;
sprite_rom[2488]=15'h2df2; sprite_rom[2489]=15'h3d8b; sprite_rom[2490]=15'h2653; sprite_rom[2491]=15'hb; sprite_rom[2492]=15'h392f; sprite_rom[2493]=15'h344b; sprite_rom[2494]=15'h192d; sprite_rom[2495]=15'h1c66;
sprite_rom[2496]=15'hcc9; sprite_rom[2497]=15'h24e8; sprite_rom[2498]=15'h45a9; sprite_rom[2499]=15'h31ed; sprite_rom[2500]=15'h3de9; sprite_rom[2501]=15'h2df1; sprite_rom[2502]=15'h1c1; sprite_rom[2503]=15'h2cf1;
sprite_rom[2504]=15'h344b; sprite_rom[2505]=15'hd2d; sprite_rom[2506]=15'h1866; sprite_rom[2507]=15'h929; sprite_rom[2508]=15'h1d28; sprite_rom[2509]=15'h25a9; sprite_rom[2510]=15'h31ec; sprite_rom[2511]=15'h358c;
sprite_rom[2512]=15'h31ab; sprite_rom[2513]=15'h4827; sprite_rom[2514]=15'h2cd3; sprite_rom[2515]=15'h3449; sprite_rom[2516]=15'hd2d; sprite_rom[2517]=15'hcc7; sprite_rom[2518]=15'h2527; sprite_rom[2519]=15'h2566;
sprite_rom[2520]=15'h1a2c; sprite_rom[2521]=15'h2de9; sprite_rom[2522]=15'h25ac; sprite_rom[2523]=15'h35e6; sprite_rom[2524]=15'h1c49; sprite_rom[2525]=15'h2c74; sprite_rom[2526]=15'h3449; sprite_rom[2527]=15'h92d;
sprite_rom[2528]=15'hce7; sprite_rom[2529]=15'h2d26; sprite_rom[2530]=15'h2d69; sprite_rom[2531]=15'h9cf; sprite_rom[2532]=15'h3164; sprite_rom[2533]=15'h9ad; sprite_rom[2534]=15'h35a6; sprite_rom[2535]=15'h8e9;
sprite_rom[2536]=15'he; sprite_rom[2537]=15'h24d5; sprite_rom[2538]=15'h3449; sprite_rom[2539]=15'h96d; sprite_rom[2540]=15'h18e6; sprite_rom[2541]=15'h2926; sprite_rom[2542]=15'h39cb; sprite_rom[2543]=15'h58f;
sprite_rom[2544]=15'h3189; sprite_rom[2545]=15'h8ed; sprite_rom[2546]=15'h3da6; sprite_rom[2547]=15'h18eb; sprite_rom[2548]=15'h282; sprite_rom[2549]=15'h24f6; sprite_rom[2550]=15'h3449; sprite_rom[2551]=15'h96c;
sprite_rom[2552]=15'h1ce6; sprite_rom[2553]=15'h24e6; sprite_rom[2554]=15'h364b; sprite_rom[2555]=15'h1d6f; sprite_rom[2556]=15'h358d; sprite_rom[2557]=15'h1869; sprite_rom[2558]=15'h3de7; sprite_rom[2559]=15'h8f0;
sprite_rom[2560]=15'h166; sprite_rom[2561]=15'h1d37; sprite_rom[2562]=15'h3449; sprite_rom[2563]=15'hd6c; sprite_rom[2564]=15'h1ce6; sprite_rom[2565]=15'h24e7; sprite_rom[2566]=15'h2eae; sprite_rom[2567]=15'h4652;
sprite_rom[2568]=15'h3df1; sprite_rom[2569]=15'h1ce9; sprite_rom[2570]=15'h3deb; sprite_rom[2571]=15'h2d31; sprite_rom[2572]=15'h3847; sprite_rom[2573]=15'h1da0; sprite_rom[2574]=15'h34c9; sprite_rom[2575]=15'h198c;
sprite_rom[2576]=15'h1ce6; sprite_rom[2577]=15'h2527; sprite_rom[2578]=15'h3294; sprite_rom[2579]=15'h4a93; sprite_rom[2580]=15'h3def; sprite_rom[2581]=15'h45f1; sprite_rom[2582]=15'h4a74; sprite_rom[2583]=15'h2d72;
sprite_rom[2584]=15'h1969; sprite_rom[2585]=15'h14; sprite_rom[2586]=15'h1e60; sprite_rom[2587]=15'h34e7; sprite_rom[2588]=15'h19ac; sprite_rom[2589]=15'h1ce6; sprite_rom[2590]=15'h3929; sprite_rom[2591]=15'h3680;
sprite_rom[2592]=15'h4ab4; sprite_rom[2593]=15'h3def; sprite_rom[2594]=15'h4e4f; sprite_rom[2595]=15'h4e93; sprite_rom[2596]=15'h18ed; sprite_rom[2597]=15'h1ce7; sprite_rom[2598]=15'hb; sprite_rom[2599]=15'h2e80;
sprite_rom[2600]=15'h3527; sprite_rom[2601]=15'h1dcc; sprite_rom[2602]=15'h24c7; sprite_rom[2603]=15'h5529; sprite_rom[2604]=15'h3640; sprite_rom[2605]=15'h56d4; sprite_rom[2606]=15'h4e94; sprite_rom[2607]=15'h42b3;
sprite_rom[2608]=15'h2885; sprite_rom[2609]=15'h25b1; sprite_rom[2610]=15'h1ce6; sprite_rom[2611]=15'hb; sprite_rom[2612]=15'h3aa0; sprite_rom[2613]=15'h3d69; sprite_rom[2614]=15'h1dcc; sprite_rom[2615]=15'h2ce7;
sprite_rom[2616]=15'h268; sprite_rom[2617]=15'h3600; sprite_rom[2618]=15'h52d3; sprite_rom[2619]=15'h4e73; sprite_rom[2620]=15'h42d3; sprite_rom[2621]=15'hc85; sprite_rom[2622]=15'h35ab; sprite_rom[2623]=15'h18c6;
sprite_rom[2624]=15'he; sprite_rom[2625]=15'h3ec0; sprite_rom[2626]=15'h3d86; sprite_rom[2627]=15'h1dcc; sprite_rom[2628]=15'h2527; sprite_rom[2629]=15'hb; sprite_rom[2630]=15'h3d60; sprite_rom[2631]=15'h4ed4;
sprite_rom[2632]=15'h252e; sprite_rom[2633]=15'h5a91; sprite_rom[2634]=15'h865; sprite_rom[2635]=15'h35a8; sprite_rom[2636]=15'hccb; sprite_rom[2637]=15'h9; sprite_rom[2638]=15'h4c00; sprite_rom[2639]=15'h356b;
sprite_rom[2640]=15'h25ed; sprite_rom[2641]=15'h2529; sprite_rom[2642]=15'h16; sprite_rom[2643]=15'h3cf4; sprite_rom[2644]=15'h4ed4; sprite_rom[2645]=15'h869; sprite_rom[2646]=15'h5a8b; sprite_rom[2647]=15'h4154;
sprite_rom[2648]=15'h31b0; sprite_rom[2649]=15'h1ced; sprite_rom[2650]=15'h286; sprite_rom[2651]=15'h5000; sprite_rom[2652]=15'h3632; sprite_rom[2653]=15'h25ed; sprite_rom[2654]=15'h5529; sprite_rom[2655]=15'h0;
sprite_rom[2656]=15'h44f3; sprite_rom[2657]=15'h3a94; sprite_rom[2658]=15'h18e6; sprite_rom[2659]=15'h5167; sprite_rom[2660]=15'h52b4; sprite_rom[2661]=15'h25d1; sprite_rom[2662]=15'h1d2d; sprite_rom[2663]=15'h227;
sprite_rom[2664]=15'h5000; sprite_rom[2665]=15'h3672; sprite_rom[2666]=15'h25ef; sprite_rom[2667]=15'h2ab; sprite_rom[2668]=15'h0; sprite_rom[2669]=15'h452e; sprite_rom[2670]=15'h3a54; sprite_rom[2671]=15'h18e3;
sprite_rom[2672]=15'h4966; sprite_rom[2673]=15'h4e73; sprite_rom[2674]=15'h25cb; sprite_rom[2675]=15'h1d8d; sprite_rom[2676]=15'h1c9; sprite_rom[2677]=15'h3653; sprite_rom[2678]=15'h41d1; sprite_rom[2679]=15'h16;
sprite_rom[2680]=15'h0; sprite_rom[2681]=15'h3d69; sprite_rom[2682]=15'h3a54; sprite_rom[2683]=15'h18c2; sprite_rom[2684]=15'h4922; sprite_rom[2685]=15'h5253; sprite_rom[2686]=15'h25cb; sprite_rom[2687]=15'h25ac;
sprite_rom[2688]=15'h1c7; sprite_rom[2689]=15'h3df5; sprite_rom[2690]=15'h2b3; sprite_rom[2691]=15'h0; sprite_rom[2692]=15'h5400; sprite_rom[2693]=15'h3d84; sprite_rom[2694]=15'h2e54; sprite_rom[2695]=15'hcc2;
sprite_rom[2696]=15'h4922; sprite_rom[2697]=15'h51d3; sprite_rom[2698]=15'h2571; sprite_rom[2699]=15'h31ac; sprite_rom[2700]=15'h166; sprite_rom[2701]=15'h5680; sprite_rom[2702]=15'h0; sprite_rom[2703]=15'h0;
sprite_rom[2704]=15'h4c00; sprite_rom[2705]=15'h3dc2; sprite_rom[2706]=15'h2e34; sprite_rom[2707]=15'h8c2; sprite_rom[2708]=15'h44e3; sprite_rom[2709]=15'h4df2; sprite_rom[2710]=15'h24f3; sprite_rom[2711]=15'h35ab;
sprite_rom[2712]=15'h167; sprite_rom[2713]=15'h3800; sprite_rom[2714]=15'h3de4; sprite_rom[2715]=15'h2df3; sprite_rom[2716]=15'h8c2; sprite_rom[2717]=15'h3ce6; sprite_rom[2718]=15'h45f2; sprite_rom[2719]=15'h2594;
sprite_rom[2720]=15'h35a9; sprite_rom[2721]=15'h1cd; sprite_rom[2722]=15'h1c00; sprite_rom[2723]=15'h3de7; sprite_rom[2724]=15'h2df3; sprite_rom[2725]=15'hcc2; sprite_rom[2726]=15'h3cc6; sprite_rom[2727]=15'h3232;
sprite_rom[2728]=15'h2673; sprite_rom[2729]=15'h35a7; sprite_rom[2730]=15'h22f; sprite_rom[2731]=15'h660; sprite_rom[2732]=15'h3de9; sprite_rom[2733]=15'h2df3; sprite_rom[2734]=15'hcc2; sprite_rom[2735]=15'h3cc3;
sprite_rom[2736]=15'h3252; sprite_rom[2737]=15'h3e91; sprite_rom[2738]=15'h35a6; sprite_rom[2739]=15'h26f; sprite_rom[2740]=15'h620; sprite_rom[2741]=15'h45e9; sprite_rom[2742]=15'h2df2; sprite_rom[2743]=15'h10c2;
sprite_rom[2744]=15'h3cc3; sprite_rom[2745]=15'h3251; sprite_rom[2746]=15'h520e; sprite_rom[2747]=15'h3d2b; sprite_rom[2748]=15'h2cf; sprite_rom[2749]=15'h920; sprite_rom[2750]=15'h45eb; sprite_rom[2751]=15'h2df2;
sprite_rom[2752]=15'h18c2; sprite_rom[2753]=15'h3c63; sprite_rom[2754]=15'h326f; sprite_rom[2755]=15'h4d6f; sprite_rom[2756]=15'h4a32; sprite_rom[2757]=15'h2f1; sprite_rom[2758]=15'h10d5; sprite_rom[2759]=15'h3dee;
sprite_rom[2760]=15'h2df2; sprite_rom[2761]=15'h18c2; sprite_rom[2762]=15'h3c63; sprite_rom[2763]=15'h326f; sprite_rom[2764]=15'h412d; sprite_rom[2765]=15'h4a75; sprite_rom[2766]=15'h13; sprite_rom[2767]=15'h1852;
sprite_rom[2768]=15'h458e; sprite_rom[2769]=15'h2df2; sprite_rom[2770]=15'h18c2; sprite_rom[2771]=15'h3863; sprite_rom[2772]=15'h326f; sprite_rom[2773]=15'h2ced; sprite_rom[2774]=15'h4eb4; sprite_rom[2775]=15'h16;
sprite_rom[2776]=15'h1c4e; sprite_rom[2777]=15'h498f; sprite_rom[2778]=15'h2df1; sprite_rom[2779]=15'h1861; sprite_rom[2780]=15'h3863; sprite_rom[2781]=15'h326f; sprite_rom[2782]=15'h252d; sprite_rom[2783]=15'h292;
sprite_rom[2784]=15'h1c67; sprite_rom[2785]=15'h498f; sprite_rom[2786]=15'h2def; sprite_rom[2787]=15'h1841; sprite_rom[2788]=15'h3863; sprite_rom[2789]=15'h364d; sprite_rom[2790]=15'h192d; sprite_rom[2791]=15'h5650;
sprite_rom[2792]=15'h5000; sprite_rom[2793]=15'h24c2; sprite_rom[2794]=15'h498f; sprite_rom[2795]=15'h2def; sprite_rom[2796]=15'hc42; sprite_rom[2797]=15'h3842; sprite_rom[2798]=15'h364d; sprite_rom[2799]=15'hd6d;
sprite_rom[2800]=15'h4e4e; sprite_rom[2801]=15'h4400; sprite_rom[2802]=15'h24c2; sprite_rom[2803]=15'h498f; sprite_rom[2804]=15'h2def; sprite_rom[2805]=15'hc62; sprite_rom[2806]=15'h3c42; sprite_rom[2807]=15'h3e4d;
sprite_rom[2808]=15'hd8c; sprite_rom[2809]=15'h4669; sprite_rom[2810]=15'h2400; sprite_rom[2811]=15'h2c82; sprite_rom[2812]=15'h498f; sprite_rom[2813]=15'h2def; sprite_rom[2814]=15'hc42; sprite_rom[2815]=15'h3842;
sprite_rom[2816]=15'h3e4d; sprite_rom[2817]=15'hdcc; sprite_rom[2818]=15'h4647; sprite_rom[2819]=15'h14; sprite_rom[2820]=15'he80; sprite_rom[2821]=15'h3886; sprite_rom[2822]=15'h49af; sprite_rom[2823]=15'h2def;
sprite_rom[2824]=15'hc42; sprite_rom[2825]=15'h3842; sprite_rom[2826]=15'h3e2d; sprite_rom[2827]=15'h19cc; sprite_rom[2828]=15'h4a06; sprite_rom[2829]=15'h12; sprite_rom[2830]=15'ha20; sprite_rom[2831]=15'h3cc6;
sprite_rom[2832]=15'h45ad; sprite_rom[2833]=15'h2ded; sprite_rom[2834]=15'hc62; sprite_rom[2835]=15'h3862; sprite_rom[2836]=15'h462d; sprite_rom[2837]=15'h19ac; sprite_rom[2838]=15'h4963; sprite_rom[2839]=15'hd;
sprite_rom[2840]=15'hd60; sprite_rom[2841]=15'h44e6; sprite_rom[2842]=15'h45ec; sprite_rom[2843]=15'h2e2f; sprite_rom[2844]=15'hc62; sprite_rom[2845]=15'h3842; sprite_rom[2846]=15'h462d; sprite_rom[2847]=15'h1dec;
sprite_rom[2848]=15'h4923; sprite_rom[2849]=15'h26f; sprite_rom[2850]=15'h10c0; sprite_rom[2851]=15'h4526; sprite_rom[2852]=15'h39ec; sprite_rom[2853]=15'h1e2e; sprite_rom[2854]=15'hc63; sprite_rom[2855]=15'h3862;
sprite_rom[2856]=15'h4632; sprite_rom[2857]=15'h25ec; sprite_rom[2858]=15'h44e3; sprite_rom[2859]=15'h1f1; sprite_rom[2860]=15'h1853; sprite_rom[2861]=15'h4546; sprite_rom[2862]=15'h39ec; sprite_rom[2863]=15'h1cc4;
sprite_rom[2864]=15'hc63; sprite_rom[2865]=15'h1cc2; sprite_rom[2866]=15'h4527; sprite_rom[2867]=15'h2dec; sprite_rom[2868]=15'h38c3; sprite_rom[2869]=15'h51b1; sprite_rom[2870]=15'h1cce; sprite_rom[2871]=15'h3d66;
sprite_rom[2872]=15'h2e2c; sprite_rom[2873]=15'h1cc3; sprite_rom[2874]=15'h863; sprite_rom[2875]=15'h18c2; sprite_rom[2876]=15'h4522; sprite_rom[2877]=15'h35ad; sprite_rom[2878]=15'h2cc3; sprite_rom[2879]=15'h49f1;
sprite_rom[2880]=15'h1829; sprite_rom[2881]=15'h3dc6; sprite_rom[2882]=15'h2e2c; sprite_rom[2883]=15'h1cc3; sprite_rom[2884]=15'h863; sprite_rom[2885]=15'h18c2; sprite_rom[2886]=15'h44e3; sprite_rom[2887]=15'h39ad;
sprite_rom[2888]=15'h24c4; sprite_rom[2889]=15'h3e51; sprite_rom[2890]=15'h2636; sprite_rom[2891]=15'h3629; sprite_rom[2892]=15'h262c; sprite_rom[2893]=15'h18c3; sprite_rom[2894]=15'hc63; sprite_rom[2895]=15'h18c2;
sprite_rom[2896]=15'h38e3; sprite_rom[2897]=15'h3daf; sprite_rom[2898]=15'h1cc6; sprite_rom[2899]=15'h364e; sprite_rom[2900]=15'h13; sprite_rom[2901]=15'h42c0; sprite_rom[2902]=15'h4a50; sprite_rom[2903]=15'h2651;
sprite_rom[2904]=15'h18c4; sprite_rom[2905]=15'hc63; sprite_rom[2906]=15'h18c3; sprite_rom[2907]=15'h38e3; sprite_rom[2908]=15'h458f; sprite_rom[2909]=15'h24c9; sprite_rom[2910]=15'h462e; sprite_rom[2911]=15'h15;
sprite_rom[2912]=15'h11c0; sprite_rom[2913]=15'h4529; sprite_rom[2914]=15'hd72; sprite_rom[2915]=15'h1ce4; sprite_rom[2916]=15'hc63; sprite_rom[2917]=15'hcc4; sprite_rom[2918]=15'h38c2; sprite_rom[2919]=15'h4632;
sprite_rom[2920]=15'h4a10; sprite_rom[2921]=15'h5dd0; sprite_rom[2922]=15'h5a0; sprite_rom[2923]=15'h1ce4; sprite_rom[2924]=15'h2dc9; sprite_rom[2925]=15'h1047; sprite_rom[2926]=15'h886; sprite_rom[2927]=15'h1c42;
sprite_rom[2928]=15'h24e9; sprite_rom[2929]=15'h2129; sprite_rom[2930]=15'h2d29; sprite_rom[2931]=15'h12e; sprite_rom[2932]=15'h4c00; sprite_rom[2933]=15'h4dce; sprite_rom[2934]=15'h39d1; sprite_rom[2935]=15'h25d1;
sprite_rom[2936]=15'h426; sprite_rom[2937]=15'h4a27; sprite_rom[2938]=15'h4631; sprite_rom[2939]=15'h8cb; sprite_rom[2940]=15'h2441; sprite_rom[2941]=15'h15; sprite_rom[2942]=15'h5800; sprite_rom[2943]=15'h421;
sprite_rom[2944]=15'h4961; sprite_rom[2945]=15'h5293; sprite_rom[2946]=15'h577; sprite_rom[2947]=15'h2421; sprite_rom[2948]=15'h4e92; sprite_rom[2949]=15'h271; sprite_rom[2950]=15'h256e; sprite_rom[2951]=15'h522b;
sprite_rom[2952]=15'h0; sprite_rom[2953]=15'h436; sprite_rom[2954]=15'h1821; sprite_rom[2955]=15'h12; sprite_rom[2956]=15'h5000; sprite_rom[2957]=15'h293; sprite_rom[2958]=15'h5ef7; sprite_rom[2959]=15'h62f7;
sprite_rom[2960]=15'h3bb; sprite_rom[2961]=15'h17; sprite_rom[2962]=15'h0; sprite_rom[2963]=15'h0; sprite_rom[2964]=15'h0; sprite_rom[2965]=15'h0; sprite_rom[2966]=15'h0; sprite_rom[2967]=15'h0;
sprite_rom[2968]=15'h0; sprite_rom[2969]=15'h0; sprite_rom[2970]=15'h0; sprite_rom[2971]=15'h0; sprite_rom[2972]=15'h0; sprite_rom[2973]=15'h0; sprite_rom[2974]=15'h0; sprite_rom[2975]=15'h0;
sprite_rom[2976]=15'h0; sprite_rom[2977]=15'h5800; sprite_rom[2978]=15'h5f16; sprite_rom[2979]=15'h6f38; sprite_rom[2980]=15'h779c; sprite_rom[2981]=15'h1f; sprite_rom[2982]=15'h2a0; sprite_rom[2983]=15'h0;
sprite_rom[2984]=15'h0; sprite_rom[2985]=15'h0; sprite_rom[2986]=15'h0; sprite_rom[2987]=15'h0; sprite_rom[2988]=15'h0; sprite_rom[2989]=15'h0; sprite_rom[2990]=15'h0; sprite_rom[2991]=15'h0;
sprite_rom[2992]=15'h0; sprite_rom[2993]=15'h0; sprite_rom[2994]=15'h0; sprite_rom[2995]=15'h0; sprite_rom[2996]=15'h0; sprite_rom[2997]=15'h0; sprite_rom[2998]=15'h5ae0; sprite_rom[2999]=15'h67be;
sprite_rom[3000]=15'h62f9; sprite_rom[3001]=15'h779a; sprite_rom[3002]=15'h3fd; sprite_rom[3003]=15'h56e0; sprite_rom[3004]=15'h0; sprite_rom[3005]=15'h0; sprite_rom[3006]=15'h0; sprite_rom[3007]=15'h0;
sprite_rom[3008]=15'h0; sprite_rom[3009]=15'h0; sprite_rom[3010]=15'h0; sprite_rom[3011]=15'h0; sprite_rom[3012]=15'h0; sprite_rom[3013]=15'h0; sprite_rom[3014]=15'h0; sprite_rom[3015]=15'h0;
sprite_rom[3016]=15'h0; sprite_rom[3017]=15'h3620; sprite_rom[3018]=15'h0; sprite_rom[3019]=15'h6780; sprite_rom[3020]=15'h7af7; sprite_rom[3021]=15'h6b7d; sprite_rom[3022]=15'h6f59; sprite_rom[3023]=15'h7bbd;
sprite_rom[3024]=15'h5400; sprite_rom[3025]=15'h15; sprite_rom[3026]=15'h0; sprite_rom[3027]=15'h0; sprite_rom[3028]=15'h0; sprite_rom[3029]=15'h0; sprite_rom[3030]=15'h0; sprite_rom[3031]=15'h0;
sprite_rom[3032]=15'h0; sprite_rom[3033]=15'h0; sprite_rom[3034]=15'h0; sprite_rom[3035]=15'h0; sprite_rom[3036]=15'h0; sprite_rom[3037]=15'h0; sprite_rom[3038]=15'h5b40; sprite_rom[3039]=15'h5695;
sprite_rom[3040]=15'h3274; sprite_rom[3041]=15'h4d27; sprite_rom[3042]=15'h77fd; sprite_rom[3043]=15'h673b; sprite_rom[3044]=15'h6fbc; sprite_rom[3045]=15'h1e; sprite_rom[3046]=15'h15; sprite_rom[3047]=15'h0;
sprite_rom[3048]=15'h0; sprite_rom[3049]=15'h0; sprite_rom[3050]=15'h0; sprite_rom[3051]=15'h0; sprite_rom[3052]=15'h0; sprite_rom[3053]=15'h0; sprite_rom[3054]=15'h0; sprite_rom[3055]=15'h0;
sprite_rom[3056]=15'h0; sprite_rom[3057]=15'h5c00; sprite_rom[3058]=15'h5ad6; sprite_rom[3059]=15'h62f7; sprite_rom[3060]=15'h5b19; sprite_rom[3061]=15'h25b2; sprite_rom[3062]=15'h2569; sprite_rom[3063]=15'h7ef3;
sprite_rom[3064]=15'h737e; sprite_rom[3065]=15'h77dc; sprite_rom[3066]=15'h1b; sprite_rom[3067]=15'h2b7; sprite_rom[3068]=15'h0; sprite_rom[3069]=15'h0; sprite_rom[3070]=15'h0; sprite_rom[3071]=15'h0;
sprite_rom[3072]=15'h0; sprite_rom[3073]=15'h0; sprite_rom[3074]=15'h0; sprite_rom[3075]=15'h0; sprite_rom[3076]=15'h0; sprite_rom[3077]=15'h5c00; sprite_rom[3078]=15'h5ad6; sprite_rom[3079]=15'h7357;
sprite_rom[3080]=15'h6b3a; sprite_rom[3081]=15'h737b; sprite_rom[3082]=15'h5b1c; sprite_rom[3083]=15'h3593; sprite_rom[3084]=15'h3d31; sprite_rom[3085]=15'h6ffd; sprite_rom[3086]=15'h7bbb; sprite_rom[3087]=15'h1a;
sprite_rom[3088]=15'h56a0; sprite_rom[3089]=15'h0; sprite_rom[3090]=15'h0; sprite_rom[3091]=15'h0; sprite_rom[3092]=15'h0; sprite_rom[3093]=15'h0; sprite_rom[3094]=15'h0; sprite_rom[3095]=15'h0;
sprite_rom[3096]=15'h0; sprite_rom[3097]=15'h0; sprite_rom[3098]=15'h56dc; sprite_rom[3099]=15'h6f57; sprite_rom[3100]=15'h635a; sprite_rom[3101]=15'h6739; sprite_rom[3102]=15'h6f7b; sprite_rom[3103]=15'h7bbb;
sprite_rom[3104]=15'h56dd; sprite_rom[3105]=15'h2653; sprite_rom[3106]=15'h7f4d; sprite_rom[3107]=15'h7fbd; sprite_rom[3108]=15'h1b; sprite_rom[3109]=15'h5400; sprite_rom[3110]=15'h15; sprite_rom[3111]=15'h0;
sprite_rom[3112]=15'h0; sprite_rom[3113]=15'h0; sprite_rom[3114]=15'h0; sprite_rom[3115]=15'h0; sprite_rom[3116]=15'h0; sprite_rom[3117]=15'h0; sprite_rom[3118]=15'h6000; sprite_rom[3119]=15'h72f5;
sprite_rom[3120]=15'h6b1b; sprite_rom[3121]=15'h633c; sprite_rom[3122]=15'h6739; sprite_rom[3123]=15'h6718; sprite_rom[3124]=15'h6b5a; sprite_rom[3125]=15'h7fdb; sprite_rom[3126]=15'h4a36; sprite_rom[3127]=15'h75ac;
sprite_rom[3128]=15'h7fdf; sprite_rom[3129]=15'h1b; sprite_rom[3130]=15'h2f4; sprite_rom[3131]=15'h0; sprite_rom[3132]=15'h0; sprite_rom[3133]=15'h0; sprite_rom[3134]=15'h0; sprite_rom[3135]=15'h0;
sprite_rom[3136]=15'h0; sprite_rom[3137]=15'h0; sprite_rom[3138]=15'h5ae0; sprite_rom[3139]=15'h6f7a; sprite_rom[3140]=15'h6f3c; sprite_rom[3141]=15'h6f7e; sprite_rom[3142]=15'h6b7c; sprite_rom[3143]=15'h675a;
sprite_rom[3144]=15'h6f39; sprite_rom[3145]=15'h737b; sprite_rom[3146]=15'h57be; sprite_rom[3147]=15'h51b2; sprite_rom[3148]=15'h77ff; sprite_rom[3149]=15'h1c; sprite_rom[3150]=15'h297; sprite_rom[3151]=15'h0;
sprite_rom[3152]=15'h0; sprite_rom[3153]=15'h0; sprite_rom[3154]=15'h0; sprite_rom[3155]=15'h0; sprite_rom[3156]=15'h0; sprite_rom[3157]=15'h0; sprite_rom[3158]=15'h72d7; sprite_rom[3159]=15'h735a;
sprite_rom[3160]=15'h77bc; sprite_rom[3161]=15'h7bbe; sprite_rom[3162]=15'h77bd; sprite_rom[3163]=15'h6f7b; sprite_rom[3164]=15'h6719; sprite_rom[3165]=15'h737a; sprite_rom[3166]=15'h7fbc; sprite_rom[3167]=15'h5279;
sprite_rom[3168]=15'h7ff7; sprite_rom[3169]=15'h56a0; sprite_rom[3170]=15'h0; sprite_rom[3171]=15'h0; sprite_rom[3172]=15'h0; sprite_rom[3173]=15'h0; sprite_rom[3174]=15'h0; sprite_rom[3175]=15'h0;
sprite_rom[3176]=15'h5c00; sprite_rom[3177]=15'h5f16; sprite_rom[3178]=15'h6b38; sprite_rom[3179]=15'h737a; sprite_rom[3180]=15'h7ffe; sprite_rom[3181]=15'h77df; sprite_rom[3182]=15'h6b7d; sprite_rom[3183]=15'h635a;
sprite_rom[3184]=15'h6f58; sprite_rom[3185]=15'h779c; sprite_rom[3186]=15'h5b9f; sprite_rom[3187]=15'h5296; sprite_rom[3188]=15'h5400; sprite_rom[3189]=15'h15; sprite_rom[3190]=15'h0; sprite_rom[3191]=15'h0;
sprite_rom[3192]=15'h0; sprite_rom[3193]=15'h0; sprite_rom[3194]=15'h0; sprite_rom[3195]=15'h5c00; sprite_rom[3196]=15'h5ef7; sprite_rom[3197]=15'h6739; sprite_rom[3198]=15'h6b59; sprite_rom[3199]=15'h7ffc;
sprite_rom[3200]=15'h77be; sprite_rom[3201]=15'h77bd; sprite_rom[3202]=15'h675a; sprite_rom[3203]=15'h6b37; sprite_rom[3204]=15'h779b; sprite_rom[3205]=15'h7bde; sprite_rom[3206]=15'h541f; sprite_rom[3207]=15'h293;
sprite_rom[3208]=15'h14; sprite_rom[3209]=15'h0; sprite_rom[3210]=15'h0; sprite_rom[3211]=15'h0; sprite_rom[3212]=15'h0; sprite_rom[3213]=15'h0; sprite_rom[3214]=15'h5ee0; sprite_rom[3215]=15'h62f8;
sprite_rom[3216]=15'h6319; sprite_rom[3217]=15'h679b; sprite_rom[3218]=15'h6fdb; sprite_rom[3219]=15'h6f5a; sprite_rom[3220]=15'h6f7b; sprite_rom[3221]=15'h677c; sprite_rom[3222]=15'h6b39; sprite_rom[3223]=15'h77bb;
sprite_rom[3224]=15'h7bbd; sprite_rom[3225]=15'h1d; sprite_rom[3226]=15'h3280; sprite_rom[3227]=15'h14; sprite_rom[3228]=15'h2b7; sprite_rom[3229]=15'h0; sprite_rom[3230]=15'h0; sprite_rom[3231]=15'h0;
sprite_rom[3232]=15'h0; sprite_rom[3233]=15'h0; sprite_rom[3234]=15'h6af6; sprite_rom[3235]=15'h5ef7; sprite_rom[3236]=15'h7b77; sprite_rom[3237]=15'h77df; sprite_rom[3238]=15'h6b5a; sprite_rom[3239]=15'h6b5b;
sprite_rom[3240]=15'h6b5b; sprite_rom[3241]=15'h6bbb; sprite_rom[3242]=15'h6739; sprite_rom[3243]=15'h737a; sprite_rom[3244]=15'h7bbc; sprite_rom[3245]=15'h39d; sprite_rom[3246]=15'h5000; sprite_rom[3247]=15'h15;
sprite_rom[3248]=15'h77c0; sprite_rom[3249]=15'h0; sprite_rom[3250]=15'h0; sprite_rom[3251]=15'h0; sprite_rom[3252]=15'h0; sprite_rom[3253]=15'h5800; sprite_rom[3254]=15'h6777; sprite_rom[3255]=15'h5ef8;
sprite_rom[3256]=15'h56fb; sprite_rom[3257]=15'h76d5; sprite_rom[3258]=15'h779c; sprite_rom[3259]=15'h6bde; sprite_rom[3260]=15'h6739; sprite_rom[3261]=15'h6f59; sprite_rom[3262]=15'h6319; sprite_rom[3263]=15'h6f59;
sprite_rom[3264]=15'h7b9d; sprite_rom[3265]=15'h35f; sprite_rom[3266]=15'h6c00; sprite_rom[3267]=15'h1c; sprite_rom[3268]=15'h0; sprite_rom[3269]=15'h0; sprite_rom[3270]=15'h0; sprite_rom[3271]=15'h5ae0;
sprite_rom[3272]=15'h673c; sprite_rom[3273]=15'h6319; sprite_rom[3274]=15'h2113; sprite_rom[3275]=15'h4105; sprite_rom[3276]=15'h62d5; sprite_rom[3277]=15'h73fd; sprite_rom[3278]=15'h6738; sprite_rom[3279]=15'h6b39;
sprite_rom[3280]=15'h633b; sprite_rom[3281]=15'h6f59; sprite_rom[3282]=15'h7bbc; sprite_rom[3283]=15'h77be; sprite_rom[3284]=15'h7c00; sprite_rom[3285]=15'h2bf; sprite_rom[3286]=15'h0; sprite_rom[3287]=15'h0;
sprite_rom[3288]=15'h0; sprite_rom[3289]=15'h72d8; sprite_rom[3290]=15'h673a; sprite_rom[3291]=15'h4338; sprite_rom[3292]=15'h2105; sprite_rom[3293]=15'h14a5; sprite_rom[3294]=15'h28a5; sprite_rom[3295]=15'h7af0;
sprite_rom[3296]=15'h6717; sprite_rom[3297]=15'h6739; sprite_rom[3298]=15'h6b5c; sprite_rom[3299]=15'h777a; sprite_rom[3300]=15'h77bd; sprite_rom[3301]=15'h6bbe; sprite_rom[3302]=15'h1e; sprite_rom[3303]=15'h4000;
sprite_rom[3304]=15'h6d9; sprite_rom[3305]=15'he; sprite_rom[3306]=15'h0; sprite_rom[3307]=15'h0; sprite_rom[3308]=15'h7336; sprite_rom[3309]=15'h6339; sprite_rom[3310]=15'h169b; sprite_rom[3311]=15'h14a5;
sprite_rom[3312]=15'h14a5; sprite_rom[3313]=15'h14a5; sprite_rom[3314]=15'h5545; sprite_rom[3315]=15'h62f8; sprite_rom[3316]=15'h6739; sprite_rom[3317]=15'h6399; sprite_rom[3318]=15'h6f59; sprite_rom[3319]=15'h777b;
sprite_rom[3320]=15'h77de; sprite_rom[3321]=15'h1b; sprite_rom[3322]=15'h4000; sprite_rom[3323]=15'h5be8; sprite_rom[3324]=15'h141; sprite_rom[3325]=15'h0; sprite_rom[3326]=15'h0; sprite_rom[3327]=15'h63d7;
sprite_rom[3328]=15'h6718; sprite_rom[3329]=15'h1455; sprite_rom[3330]=15'h1085; sprite_rom[3331]=15'h1084; sprite_rom[3332]=15'h14a4; sprite_rom[3333]=15'h40a5; sprite_rom[3334]=15'h5ed8; sprite_rom[3335]=15'h5ef7;
sprite_rom[3336]=15'h6f77; sprite_rom[3337]=15'h6b38; sprite_rom[3338]=15'h779b; sprite_rom[3339]=15'h7fde; sprite_rom[3340]=15'h1a; sprite_rom[3341]=15'h5d48; sprite_rom[3342]=15'h2834; sprite_rom[3343]=15'h0;
sprite_rom[3344]=15'h6000; sprite_rom[3345]=15'h63d9; sprite_rom[3346]=15'h6338; sprite_rom[3347]=15'h1634; sprite_rom[3348]=15'hc82; sprite_rom[3349]=15'hc63; sprite_rom[3350]=15'h1083; sprite_rom[3351]=15'h2885;
sprite_rom[3352]=15'h5ef8; sprite_rom[3353]=15'h5ef7; sprite_rom[3354]=15'h7b37; sprite_rom[3355]=15'h6b3a; sprite_rom[3356]=15'h779b; sprite_rom[3357]=15'h7ffe; sprite_rom[3358]=15'h19; sprite_rom[3359]=15'h204a;
sprite_rom[3360]=15'h557; sprite_rom[3361]=15'h10; sprite_rom[3362]=15'h6000; sprite_rom[3363]=15'h5f3d; sprite_rom[3364]=15'h5398; sprite_rom[3365]=15'h5665; sprite_rom[3366]=15'hc47; sprite_rom[3367]=15'hc63;
sprite_rom[3368]=15'h1063; sprite_rom[3369]=15'h28a4; sprite_rom[3370]=15'h6337; sprite_rom[3371]=15'h6318; sprite_rom[3372]=15'h7319; sprite_rom[3373]=15'h739d; sprite_rom[3374]=15'h7bbc; sprite_rom[3375]=15'h7bdf;
sprite_rom[3376]=15'h3da; sprite_rom[3377]=15'h10b0; sprite_rom[3378]=15'h6d5; sprite_rom[3379]=15'ha; sprite_rom[3380]=15'h6400; sprite_rom[3381]=15'h5f00; sprite_rom[3382]=15'h231c; sprite_rom[3383]=15'h2821;
sprite_rom[3384]=15'hc74; sprite_rom[3385]=15'hc63; sprite_rom[3386]=15'h1083; sprite_rom[3387]=15'h1484; sprite_rom[3388]=15'h6315; sprite_rom[3389]=15'h6739; sprite_rom[3390]=15'h6f39; sprite_rom[3391]=15'h777c;
sprite_rom[3392]=15'h77bd; sprite_rom[3393]=15'h7bde; sprite_rom[3394]=15'h3da; sprite_rom[3395]=15'h1460; sprite_rom[3396]=15'h23f7; sprite_rom[3397]=15'h2a4; sprite_rom[3398]=15'h6c00; sprite_rom[3399]=15'h5f60;
sprite_rom[3400]=15'h42be; sprite_rom[3401]=15'h16b9; sprite_rom[3402]=15'h908; sprite_rom[3403]=15'hc63; sprite_rom[3404]=15'h443; sprite_rom[3405]=15'h862; sprite_rom[3406]=15'h5f13; sprite_rom[3407]=15'h6739;
sprite_rom[3408]=15'h6b3a; sprite_rom[3409]=15'h6b7e; sprite_rom[3410]=15'h779b; sprite_rom[3411]=15'h7bdd; sprite_rom[3412]=15'h3db; sprite_rom[3413]=15'h2a60; sprite_rom[3414]=15'h160a; sprite_rom[3415]=15'h5164;
sprite_rom[3416]=15'h6c00; sprite_rom[3417]=15'h6400; sprite_rom[3418]=15'h7ebc; sprite_rom[3419]=15'h67fd; sprite_rom[3420]=15'hc65; sprite_rom[3421]=15'hc63; sprite_rom[3422]=15'h2d03; sprite_rom[3423]=15'h50b;
sprite_rom[3424]=15'h630a; sprite_rom[3425]=15'h6739; sprite_rom[3426]=15'h6b5a; sprite_rom[3427]=15'h6fdd; sprite_rom[3428]=15'h77bb; sprite_rom[3429]=15'h7bdd; sprite_rom[3430]=15'h39d; sprite_rom[3431]=15'h14b0;
sprite_rom[3432]=15'h2d45; sprite_rom[3433]=15'h16; sprite_rom[3434]=15'h7000; sprite_rom[3435]=15'h56b8; sprite_rom[3436]=15'h7ed4; sprite_rom[3437]=15'h1030; sprite_rom[3438]=15'hc63; sprite_rom[3439]=15'h5205;
sprite_rom[3440]=15'h4e94; sprite_rom[3441]=15'h5ed0; sprite_rom[3442]=15'h6738; sprite_rom[3443]=15'h6b9a; sprite_rom[3444]=15'h77dc; sprite_rom[3445]=15'h7bbb; sprite_rom[3446]=15'h7fde; sprite_rom[3447]=15'h3bc;
sprite_rom[3448]=15'h22a0; sprite_rom[3449]=15'h2542; sprite_rom[3450]=15'hf; sprite_rom[3451]=15'h0; sprite_rom[3452]=15'h2e1c; sprite_rom[3453]=15'h777e; sprite_rom[3454]=15'hc4e; sprite_rom[3455]=15'hc63;
sprite_rom[3456]=15'h445; sprite_rom[3457]=15'h1c21; sprite_rom[3458]=15'h62b0; sprite_rom[3459]=15'h6717; sprite_rom[3460]=15'h77b9; sprite_rom[3461]=15'h7fbb; sprite_rom[3462]=15'h7bbe; sprite_rom[3463]=15'h7fff;
sprite_rom[3464]=15'h3da; sprite_rom[3465]=15'h4000; sprite_rom[3466]=15'h2545; sprite_rom[3467]=15'h26c; sprite_rom[3468]=15'h0; sprite_rom[3469]=15'h2e1e; sprite_rom[3470]=15'h5fff; sprite_rom[3471]=15'hc61;
sprite_rom[3472]=15'hc63; sprite_rom[3473]=15'h5204; sprite_rom[3474]=15'h1615; sprite_rom[3475]=15'h5941; sprite_rom[3476]=15'h6717; sprite_rom[3477]=15'h7b7a; sprite_rom[3478]=15'h7b9d; sprite_rom[3479]=15'h7fff;
sprite_rom[3480]=15'h7bff; sprite_rom[3481]=15'h1b; sprite_rom[3482]=15'h4260; sprite_rom[3483]=15'h1d50; sprite_rom[3484]=15'h4d8f; sprite_rom[3485]=15'h6800; sprite_rom[3486]=15'h2d57; sprite_rom[3487]=15'h4f17;
sprite_rom[3488]=15'hc81; sprite_rom[3489]=15'h863; sprite_rom[3490]=15'h7ea8; sprite_rom[3491]=15'h73ff; sprite_rom[3492]=15'h4030; sprite_rom[3493]=15'h6717; sprite_rom[3494]=15'h7759; sprite_rom[3495]=15'h77df;
sprite_rom[3496]=15'h7ffe; sprite_rom[3497]=15'h77ff; sprite_rom[3498]=15'h1e; sprite_rom[3499]=15'h3640; sprite_rom[3500]=15'h1d54; sprite_rom[3501]=15'h98d; sprite_rom[3502]=15'h6c15; sprite_rom[3503]=15'h2154;
sprite_rom[3504]=15'h1ab5; sprite_rom[3505]=15'hc62; sprite_rom[3506]=15'h863; sprite_rom[3507]=15'h3aa3; sprite_rom[3508]=15'h5fde; sprite_rom[3509]=15'h157f; sprite_rom[3510]=15'h6335; sprite_rom[3511]=15'h735a;
sprite_rom[3512]=15'h7bde; sprite_rom[3513]=15'h7fde; sprite_rom[3514]=15'h6fdf; sprite_rom[3515]=15'h1e; sprite_rom[3516]=15'h2e40; sprite_rom[3517]=15'h19d1; sprite_rom[3518]=15'h98d; sprite_rom[3519]=15'h7349;
sprite_rom[3520]=15'h2115; sprite_rom[3521]=15'hd0a; sprite_rom[3522]=15'hc85; sprite_rom[3523]=15'h463; sprite_rom[3524]=15'h67ef; sprite_rom[3525]=15'h47ff; sprite_rom[3526]=15'h1377; sprite_rom[3527]=15'h66ea;
sprite_rom[3528]=15'h6f9b; sprite_rom[3529]=15'h7bfd; sprite_rom[3530]=15'h7bde; sprite_rom[3531]=15'h6fde; sprite_rom[3532]=15'h2e20; sprite_rom[3533]=15'h1971; sprite_rom[3534]=15'h1dad; sprite_rom[3535]=15'h7e41;
sprite_rom[3536]=15'h20b0; sprite_rom[3537]=15'h10a5; sprite_rom[3538]=15'hc85; sprite_rom[3539]=15'h483; sprite_rom[3540]=15'h7ff3; sprite_rom[3541]=15'h37df; sprite_rom[3542]=15'h43ee; sprite_rom[3543]=15'h76a3;
sprite_rom[3544]=15'h77b9; sprite_rom[3545]=15'h7fbc; sprite_rom[3546]=15'h7fff; sprite_rom[3547]=15'h7b9f; sprite_rom[3548]=15'h25c0; sprite_rom[3549]=15'h1931; sprite_rom[3550]=15'h1dad; sprite_rom[3551]=15'h60c3;
sprite_rom[3552]=15'h1453; sprite_rom[3553]=15'hc85; sprite_rom[3554]=15'h821; sprite_rom[3555]=15'h483; sprite_rom[3556]=15'h5ef0; sprite_rom[3557]=15'h6f8; sprite_rom[3558]=15'h2ee9; sprite_rom[3559]=15'h7a82;
sprite_rom[3560]=15'h777d; sprite_rom[3561]=15'h7bde; sprite_rom[3562]=15'h7ffe; sprite_rom[3563]=15'h3bd; sprite_rom[3564]=15'h25c0; sprite_rom[3565]=15'h1cee; sprite_rom[3566]=15'h1dad; sprite_rom[3567]=15'h3843;
sprite_rom[3568]=15'h1516; sprite_rom[3569]=15'h2045; sprite_rom[3570]=15'h2215; sprite_rom[3571]=15'h1461; sprite_rom[3572]=15'h5688; sprite_rom[3573]=15'h535; sprite_rom[3574]=15'h8aa; sprite_rom[3575]=15'h7282;
sprite_rom[3576]=15'h7fff; sprite_rom[3577]=15'h7fdf; sprite_rom[3578]=15'h7bde; sprite_rom[3579]=15'h3fd; sprite_rom[3580]=15'h2560; sprite_rom[3581]=15'h24ce; sprite_rom[3582]=15'h1dad; sprite_rom[3583]=15'h1862;
sprite_rom[3584]=15'h670; sprite_rom[3585]=15'h2045; sprite_rom[3586]=15'h4215; sprite_rom[3587]=15'h144a; sprite_rom[3588]=15'h2905; sprite_rom[3589]=15'h14a8; sprite_rom[3590]=15'h14a5; sprite_rom[3591]=15'h66a5;
sprite_rom[3592]=15'h52b8; sprite_rom[3593]=15'h7ff7; sprite_rom[3594]=15'h6fff; sprite_rom[3595]=15'h1e; sprite_rom[3596]=15'h1d60; sprite_rom[3597]=15'h2c4e; sprite_rom[3598]=15'h1d8d; sprite_rom[3599]=15'h1c62;
sprite_rom[3600]=15'h41c7; sprite_rom[3601]=15'h1061; sprite_rom[3602]=15'h4150; sprite_rom[3603]=15'h144a; sprite_rom[3604]=15'h2108; sprite_rom[3605]=15'h2148; sprite_rom[3606]=15'h8a5; sprite_rom[3607]=15'h6370;
sprite_rom[3608]=15'h2150; sprite_rom[3609]=15'h7f45; sprite_rom[3610]=15'h777f; sprite_rom[3611]=15'h1d60; sprite_rom[3612]=15'h304e; sprite_rom[3613]=15'h198d; sprite_rom[3614]=15'h1863; sprite_rom[3615]=15'h4849;
sprite_rom[3616]=15'h833; sprite_rom[3617]=15'h294a; sprite_rom[3618]=15'h1465; sprite_rom[3619]=15'h2105; sprite_rom[3620]=15'h1508; sprite_rom[3621]=15'h2c85; sprite_rom[3622]=15'h5e96; sprite_rom[3623]=15'h4210;
sprite_rom[3624]=15'h7ec5; sprite_rom[3625]=15'h3db; sprite_rom[3626]=15'h1d60; sprite_rom[3627]=15'h302d; sprite_rom[3628]=15'h198c; sprite_rom[3629]=15'hcc6; sprite_rom[3630]=15'h10e9; sprite_rom[3631]=15'he94;
sprite_rom[3632]=15'h861; sprite_rom[3633]=15'h1084; sprite_rom[3634]=15'h14a5; sprite_rom[3635]=15'h14a5; sprite_rom[3636]=15'h5145; sprite_rom[3637]=15'h5a8a; sprite_rom[3638]=15'h294a; sprite_rom[3639]=15'h7f68;
sprite_rom[3640]=15'h1f; sprite_rom[3641]=15'h1dc0; sprite_rom[3642]=15'h342b; sprite_rom[3643]=15'h198c; sprite_rom[3644]=15'h886; sprite_rom[3645]=15'h18e7; sprite_rom[3646]=15'h5a67; sprite_rom[3647]=15'h4ae;
sprite_rom[3648]=15'h821; sprite_rom[3649]=15'h442; sprite_rom[3650]=15'h821; sprite_rom[3651]=15'h2a8a; sprite_rom[3652]=15'h4ec2; sprite_rom[3653]=15'hc65; sprite_rom[3654]=15'h3f5; sprite_rom[3655]=15'h1dc0;
sprite_rom[3656]=15'h382b; sprite_rom[3657]=15'h118c; sprite_rom[3658]=15'h8c7; sprite_rom[3659]=15'hd28; sprite_rom[3660]=15'h38c7; sprite_rom[3661]=15'h4ed5; sprite_rom[3662]=15'h2a14; sprite_rom[3663]=15'h414a;
sprite_rom[3664]=15'h5eb4; sprite_rom[3665]=15'h73fc; sprite_rom[3666]=15'h2ef5; sprite_rom[3667]=15'h5a2b; sprite_rom[3668]=15'h1a20; sprite_rom[3669]=15'h3829; sprite_rom[3670]=15'hd8c; sprite_rom[3671]=15'h8c6;
sprite_rom[3672]=15'hd27; sprite_rom[3673]=15'h2927; sprite_rom[3674]=15'h5e6b; sprite_rom[3675]=15'h2a98; sprite_rom[3676]=15'h5610; sprite_rom[3677]=15'h7fff; sprite_rom[3678]=15'h7bbe; sprite_rom[3679]=15'h39c;
sprite_rom[3680]=15'h1e20; sprite_rom[3681]=15'h3cc9; sprite_rom[3682]=15'h98c; sprite_rom[3683]=15'h18e6; sprite_rom[3684]=15'h1966; sprite_rom[3685]=15'h1d28; sprite_rom[3686]=15'h522b; sprite_rom[3687]=15'h2a13;
sprite_rom[3688]=15'h460a; sprite_rom[3689]=15'h16; sprite_rom[3690]=15'h6000; sprite_rom[3691]=15'h2640; sprite_rom[3692]=15'h34e7; sprite_rom[3693]=15'h96d; sprite_rom[3694]=15'h1926; sprite_rom[3695]=15'h1d66;
sprite_rom[3696]=15'h1d27; sprite_rom[3697]=15'h528d; sprite_rom[3698]=15'h2952; sprite_rom[3699]=15'h4dae; sprite_rom[3700]=15'h2b4; sprite_rom[3701]=15'h2660; sprite_rom[3702]=15'h3526; sprite_rom[3703]=15'h96d;
sprite_rom[3704]=15'h1d26; sprite_rom[3705]=15'h2523; sprite_rom[3706]=15'h2569; sprite_rom[3707]=15'h4a6f; sprite_rom[3708]=15'h2d6d; sprite_rom[3709]=15'h4e4b; sprite_rom[3710]=15'h3a53; sprite_rom[3711]=15'h16;
sprite_rom[3712]=15'h0; sprite_rom[3713]=15'h0; sprite_rom[3714]=15'h428a; sprite_rom[3715]=15'h2e60; sprite_rom[3716]=15'h35a2; sprite_rom[3717]=15'h96d; sprite_rom[3718]=15'h2526; sprite_rom[3719]=15'h2d26;
sprite_rom[3720]=15'h2d69; sprite_rom[3721]=15'h362f; sprite_rom[3722]=15'h258f; sprite_rom[3723]=15'h49ef; sprite_rom[3724]=15'h19ef; sprite_rom[3725]=15'h2a6; sprite_rom[3726]=15'h0; sprite_rom[3727]=15'h2000;
sprite_rom[3728]=15'h1610; sprite_rom[3729]=15'h10; sprite_rom[3730]=15'h2e80; sprite_rom[3731]=15'h31e7; sprite_rom[3732]=15'hd2d; sprite_rom[3733]=15'h2526; sprite_rom[3734]=15'h2d27; sprite_rom[3735]=15'h31c9;
sprite_rom[3736]=15'h3d2f; sprite_rom[3737]=15'h312f; sprite_rom[3738]=15'h49ed; sprite_rom[3739]=15'h198d; sprite_rom[3740]=15'h122; sprite_rom[3741]=15'h0; sprite_rom[3742]=15'h1100; sprite_rom[3743]=15'h290a;
sprite_rom[3744]=15'h2ea0; sprite_rom[3745]=15'h31a9; sprite_rom[3746]=15'h112f; sprite_rom[3747]=15'h2526; sprite_rom[3748]=15'h2ce9; sprite_rom[3749]=15'h3e2e; sprite_rom[3750]=15'h3c4e; sprite_rom[3751]=15'h3189;
sprite_rom[3752]=15'h356d; sprite_rom[3753]=15'h196f; sprite_rom[3754]=15'h4446; sprite_rom[3755]=15'h0; sprite_rom[3756]=15'h2830; sprite_rom[3757]=15'h40aa; sprite_rom[3758]=15'h26c0; sprite_rom[3759]=15'h31a9;
sprite_rom[3760]=15'h18ed; sprite_rom[3761]=15'h2526; sprite_rom[3762]=15'h2d29; sprite_rom[3763]=15'h3df3; sprite_rom[3764]=15'h2487; sprite_rom[3765]=15'h318c; sprite_rom[3766]=15'h304c; sprite_rom[3767]=15'h1d6d;
sprite_rom[3768]=15'h1c86; sprite_rom[3769]=15'h5814; sprite_rom[3770]=15'h8ae; sprite_rom[3771]=15'h201; sprite_rom[3772]=15'h46c0; sprite_rom[3773]=15'h31ae; sprite_rom[3774]=15'h18ef; sprite_rom[3775]=15'h2526;
sprite_rom[3776]=15'h2d29; sprite_rom[3777]=15'h39f3; sprite_rom[3778]=15'h34c2; sprite_rom[3779]=15'h358d; sprite_rom[3780]=15'h3443; sprite_rom[3781]=15'h25ad; sprite_rom[3782]=15'h18e6; sprite_rom[3783]=15'h4acb;
sprite_rom[3784]=15'h204a; sprite_rom[3785]=15'h14; sprite_rom[3786]=15'h4ec0; sprite_rom[3787]=15'h3593; sprite_rom[3788]=15'h1cef; sprite_rom[3789]=15'h2526; sprite_rom[3790]=15'h3929; sprite_rom[3791]=15'h39f3;
sprite_rom[3792]=15'h3584; sprite_rom[3793]=15'h1dac; sprite_rom[3794]=15'h3482; sprite_rom[3795]=15'h25ad; sprite_rom[3796]=15'h1844; sprite_rom[3797]=15'h39ee; sprite_rom[3798]=15'h40aa; sprite_rom[3799]=15'h14;
sprite_rom[3800]=15'h4c00; sprite_rom[3801]=15'h3d92; sprite_rom[3802]=15'h1cee; sprite_rom[3803]=15'h2527; sprite_rom[3804]=15'h4129; sprite_rom[3805]=15'h49f1; sprite_rom[3806]=15'h364e; sprite_rom[3807]=15'h8ed;
sprite_rom[3808]=15'h3d03; sprite_rom[3809]=15'h25cd; sprite_rom[3810]=15'h3903; sprite_rom[3811]=15'h35ab; sprite_rom[3812]=15'h454a; sprite_rom[3813]=15'hc; sprite_rom[3814]=15'h5000; sprite_rom[3815]=15'h3d92;
sprite_rom[3816]=15'h1ccd; sprite_rom[3817]=15'h2529; sprite_rom[3818]=15'h5129; sprite_rom[3819]=15'h5251; sprite_rom[3820]=15'h4a52; sprite_rom[3821]=15'h2d71; sprite_rom[3822]=15'h3e0b; sprite_rom[3823]=15'h1dcd;
sprite_rom[3824]=15'h1d46; sprite_rom[3825]=15'h31e7; sprite_rom[3826]=15'h460a; sprite_rom[3827]=15'h289; sprite_rom[3828]=15'h5000; sprite_rom[3829]=15'h45af; sprite_rom[3830]=15'h250b; sprite_rom[3831]=15'h2529;
sprite_rom[3832]=15'h6267; sprite_rom[3833]=15'h526f; sprite_rom[3834]=15'h31b1; sprite_rom[3835]=15'h35ac; sprite_rom[3836]=15'h4a91; sprite_rom[3837]=15'h25cd; sprite_rom[3838]=15'h20e7; sprite_rom[3839]=15'h31a7;
sprite_rom[3840]=15'h3a30; sprite_rom[3841]=15'h1e9; sprite_rom[3842]=15'h5000; sprite_rom[3843]=15'h4a2d; sprite_rom[3844]=15'h252b; sprite_rom[3845]=15'h2529; sprite_rom[3846]=15'h5413; sprite_rom[3847]=15'h5a8e;
sprite_rom[3848]=15'h4674; sprite_rom[3849]=15'h4a51; sprite_rom[3850]=15'h4692; sprite_rom[3851]=15'h2e2c; sprite_rom[3852]=15'h1d29; sprite_rom[3853]=15'h31e7; sprite_rom[3854]=15'h322b; sprite_rom[3855]=15'h5929;
sprite_rom[3856]=15'h4674; sprite_rom[3857]=15'h1d28; sprite_rom[3858]=15'h5629; sprite_rom[3859]=15'h5000; sprite_rom[3860]=15'h5a8d; sprite_rom[3861]=15'h4e95; sprite_rom[3862]=15'h5273; sprite_rom[3863]=15'h46d4;
sprite_rom[3864]=15'h3a2c; sprite_rom[3865]=15'h1926; sprite_rom[3866]=15'h31e7; sprite_rom[3867]=15'h2dc9; sprite_rom[3868]=15'h50eb; sprite_rom[3869]=15'h5400; sprite_rom[3870]=15'h4e53; sprite_rom[3871]=15'h15;
sprite_rom[3872]=15'h4c00; sprite_rom[3873]=15'h5a8f; sprite_rom[3874]=15'h4673; sprite_rom[3875]=15'h4e51; sprite_rom[3876]=15'h4ad4; sprite_rom[3877]=15'h462f; sprite_rom[3878]=15'h18e8; sprite_rom[3879]=15'h35a6;
sprite_rom[3880]=15'h2dc7; sprite_rom[3881]=15'h44e9; sprite_rom[3882]=15'h4400; sprite_rom[3883]=15'h5aaf; sprite_rom[3884]=15'h1d73; sprite_rom[3885]=15'h4947; sprite_rom[3886]=15'h4ab3; sprite_rom[3887]=15'h4dcf;
sprite_rom[3888]=15'h18e9; sprite_rom[3889]=15'h35a6; sprite_rom[3890]=15'h2d69; sprite_rom[3891]=15'h34c9; sprite_rom[3892]=15'h16; sprite_rom[3893]=15'h2c00; sprite_rom[3894]=15'h568f; sprite_rom[3895]=15'h18f1;
sprite_rom[3896]=15'h40c4; sprite_rom[3897]=15'h4ed4; sprite_rom[3898]=15'h456f; sprite_rom[3899]=15'h18ce; sprite_rom[3900]=15'h35a6; sprite_rom[3901]=15'h2ceb; sprite_rom[3902]=15'h2c69; sprite_rom[3903]=15'h14;
sprite_rom[3904]=15'h2680; sprite_rom[3905]=15'h4e91; sprite_rom[3906]=15'h1ccb; sprite_rom[3907]=15'h18c6; sprite_rom[3908]=15'h4e92; sprite_rom[3909]=15'h396f; sprite_rom[3910]=15'h1cf3; sprite_rom[3911]=15'h35a6;
sprite_rom[3912]=15'h2ccc; sprite_rom[3913]=15'h2c69; sprite_rom[3914]=15'h14; sprite_rom[3915]=15'h2e20; sprite_rom[3916]=15'h4e91; sprite_rom[3917]=15'h1c8b; sprite_rom[3918]=15'h18c6; sprite_rom[3919]=15'h4e51;
sprite_rom[3920]=15'h218d; sprite_rom[3921]=15'h1dd4; sprite_rom[3922]=15'h3564; sprite_rom[3923]=15'h24ef; sprite_rom[3924]=15'h2c69; sprite_rom[3925]=15'h14; sprite_rom[3926]=15'h35c0; sprite_rom[3927]=15'h4a8f;
sprite_rom[3928]=15'h1c4b; sprite_rom[3929]=15'h10c3; sprite_rom[3930]=15'h4e4e; sprite_rom[3931]=15'h1d8d; sprite_rom[3932]=15'h1e51; sprite_rom[3933]=15'h3d67; sprite_rom[3934]=15'h196d; sprite_rom[3935]=15'h2c4b;
sprite_rom[3936]=15'h12; sprite_rom[3937]=15'h38e0; sprite_rom[3938]=15'h4a8f; sprite_rom[3939]=15'h1849; sprite_rom[3940]=15'h1066; sprite_rom[3941]=15'h4dee; sprite_rom[3942]=15'h1dad; sprite_rom[3943]=15'h2e8b;
sprite_rom[3944]=15'h3d29; sprite_rom[3945]=15'h9ec; sprite_rom[3946]=15'h34cb; sprite_rom[3947]=15'h12; sprite_rom[3948]=15'h3cd4; sprite_rom[3949]=15'h4691; sprite_rom[3950]=15'h1849; sprite_rom[3951]=15'h1066;
sprite_rom[3952]=15'h4deb; sprite_rom[3953]=15'h19ad; sprite_rom[3954]=15'h4a49; sprite_rom[3955]=15'h3d29; sprite_rom[3956]=15'h25ed; sprite_rom[3957]=15'h3526; sprite_rom[3958]=15'h12; sprite_rom[3959]=15'h3cee;
sprite_rom[3960]=15'h4671; sprite_rom[3961]=15'h1849; sprite_rom[3962]=15'hc66; sprite_rom[3963]=15'h4deb; sprite_rom[3964]=15'h1daf; sprite_rom[3965]=15'h5567; sprite_rom[3966]=15'h392e; sprite_rom[3967]=15'h39af;
sprite_rom[3968]=15'h3522; sprite_rom[3969]=15'h12; sprite_rom[3970]=15'h3d29; sprite_rom[3971]=15'h4652; sprite_rom[3972]=15'h1849; sprite_rom[3973]=15'hc66; sprite_rom[3974]=15'h4deb; sprite_rom[3975]=15'h25af;
sprite_rom[3976]=15'h4d23; sprite_rom[3977]=15'h2d36; sprite_rom[3978]=15'h3d8f; sprite_rom[3979]=15'h44eb; sprite_rom[3980]=15'h12; sprite_rom[3981]=15'h5400; sprite_rom[3982]=15'h3d62; sprite_rom[3983]=15'h4652;
sprite_rom[3984]=15'h1048; sprite_rom[3985]=15'h866; sprite_rom[3986]=15'h49eb; sprite_rom[3987]=15'h2d91; sprite_rom[3988]=15'h2d23; sprite_rom[3989]=15'h26b7; sprite_rom[3990]=15'h35b1; sprite_rom[3991]=15'h4e51;
sprite_rom[3992]=15'h12; sprite_rom[3993]=15'h4800; sprite_rom[3994]=15'h35c1; sprite_rom[3995]=15'h4653; sprite_rom[3996]=15'hc47; sprite_rom[3997]=15'h866; sprite_rom[3998]=15'h462b; sprite_rom[3999]=15'h3192;
sprite_rom[4000]=15'h2ce6; sprite_rom[4001]=15'h53d3; sprite_rom[4002]=15'h35ee; sprite_rom[4003]=15'h4a92; sprite_rom[4004]=15'h13; sprite_rom[4005]=15'h3400; sprite_rom[4006]=15'h35c2; sprite_rom[4007]=15'h3e53;
sprite_rom[4008]=15'hc46; sprite_rom[4009]=15'h886; sprite_rom[4010]=15'h3e29; sprite_rom[4011]=15'h3d92; sprite_rom[4012]=15'h24c6; sprite_rom[4013]=15'h2ce; sprite_rom[4014]=15'h3e35; sprite_rom[4015]=15'h466d;
sprite_rom[4016]=15'h14; sprite_rom[4017]=15'h1800; sprite_rom[4018]=15'h35e6; sprite_rom[4019]=15'h3e53; sprite_rom[4020]=15'hc46; sprite_rom[4021]=15'h8c6; sprite_rom[4022]=15'h3de9; sprite_rom[4023]=15'h3d92;
sprite_rom[4024]=15'h1c69; sprite_rom[4025]=15'h28b; sprite_rom[4026]=15'h4ee0; sprite_rom[4027]=15'h45af; sprite_rom[4028]=15'h660; sprite_rom[4029]=15'h31e8; sprite_rom[4030]=15'h3df3; sprite_rom[4031]=15'h846;
sprite_rom[4032]=15'h864; sprite_rom[4033]=15'h3de9; sprite_rom[4034]=15'h35b3; sprite_rom[4035]=15'h1869; sprite_rom[4036]=15'h5a49; sprite_rom[4037]=15'h0; sprite_rom[4038]=15'h2d6; sprite_rom[4039]=15'h9c0;
sprite_rom[4040]=15'h35e9; sprite_rom[4041]=15'h39f3; sprite_rom[4042]=15'h846; sprite_rom[4043]=15'h863; sprite_rom[4044]=15'h35e9; sprite_rom[4045]=15'h35f3; sprite_rom[4046]=15'h188c; sprite_rom[4047]=15'h5207;
sprite_rom[4048]=15'hcc0; sprite_rom[4049]=15'h35eb; sprite_rom[4050]=15'h39f3; sprite_rom[4051]=15'h846; sprite_rom[4052]=15'h863; sprite_rom[4053]=15'h3627; sprite_rom[4054]=15'h31f2; sprite_rom[4055]=15'h10ce;
sprite_rom[4056]=15'h4d66; sprite_rom[4057]=15'h16; sprite_rom[4058]=15'h1853; sprite_rom[4059]=15'h35eb; sprite_rom[4060]=15'h39f3; sprite_rom[4061]=15'h844; sprite_rom[4062]=15'h864; sprite_rom[4063]=15'h35e7;
sprite_rom[4064]=15'h31f2; sprite_rom[4065]=15'hcef; sprite_rom[4066]=15'h4926; sprite_rom[4067]=15'h13; sprite_rom[4068]=15'h184e; sprite_rom[4069]=15'h35ee; sprite_rom[4070]=15'h39f3; sprite_rom[4071]=15'h866;
sprite_rom[4072]=15'hc64; sprite_rom[4073]=15'h3de7; sprite_rom[4074]=15'h3231; sprite_rom[4075]=15'hd2f; sprite_rom[4076]=15'h40e6; sprite_rom[4077]=15'h292; sprite_rom[4078]=15'h18c7; sprite_rom[4079]=15'h3dae;
sprite_rom[4080]=15'h39b3; sprite_rom[4081]=15'hc66; sprite_rom[4082]=15'h864; sprite_rom[4083]=15'h35e6; sprite_rom[4084]=15'h362f; sprite_rom[4085]=15'hd6d; sprite_rom[4086]=15'h2cc4; sprite_rom[4087]=15'h1f3;
sprite_rom[4088]=15'h5000; sprite_rom[4089]=15'h1cc1; sprite_rom[4090]=15'h45b1; sprite_rom[4091]=15'h3a53; sprite_rom[4092]=15'hc66; sprite_rom[4093]=15'h863; sprite_rom[4094]=15'h35e6; sprite_rom[4095]=15'h362f;
sprite_rom[4096]=15'h19ac; sprite_rom[4097]=15'h2484; sprite_rom[4098]=15'h4df1; sprite_rom[4099]=15'h3800; sprite_rom[4100]=15'h24c2; sprite_rom[4101]=15'h4591; sprite_rom[4102]=15'h2570; sprite_rom[4103]=15'h846;
sprite_rom[4104]=15'h863; sprite_rom[4105]=15'h35c6; sprite_rom[4106]=15'h3e2f; sprite_rom[4107]=15'h1dac; sprite_rom[4108]=15'h1c84; sprite_rom[4109]=15'h364b; sprite_rom[4110]=15'h16; sprite_rom[4111]=15'h1c00;
sprite_rom[4112]=15'h24c4; sprite_rom[4113]=15'h4591; sprite_rom[4114]=15'h248e; sprite_rom[4115]=15'h846; sprite_rom[4116]=15'h863; sprite_rom[4117]=15'h35c4; sprite_rom[4118]=15'h45cf; sprite_rom[4119]=15'h25ec;
sprite_rom[4120]=15'h18c6; sprite_rom[4121]=15'h3e2b; sprite_rom[4122]=15'h12; sprite_rom[4123]=15'h680; sprite_rom[4124]=15'h2cc6; sprite_rom[4125]=15'h458f; sprite_rom[4126]=15'h24cb; sprite_rom[4127]=15'h847;
sprite_rom[4128]=15'hc63; sprite_rom[4129]=15'h45c3; sprite_rom[4130]=15'h45d2; sprite_rom[4131]=15'h35ec; sprite_rom[4132]=15'h18c6; sprite_rom[4133]=15'h4969; sprite_rom[4134]=15'h2ac; sprite_rom[4135]=15'h5c0;
sprite_rom[4136]=15'h38c7; sprite_rom[4137]=15'h458f; sprite_rom[4138]=15'h24cb; sprite_rom[4139]=15'h846; sprite_rom[4140]=15'h863; sprite_rom[4141]=15'h3926; sprite_rom[4142]=15'h452e; sprite_rom[4143]=15'h3dad;
sprite_rom[4144]=15'h1886; sprite_rom[4145]=15'h4547; sprite_rom[4146]=15'h1f1; sprite_rom[4147]=15'he60; sprite_rom[4148]=15'h38c2; sprite_rom[4149]=15'h498f; sprite_rom[4150]=15'h24c9; sprite_rom[4151]=15'h846;
sprite_rom[4152]=15'h863; sprite_rom[4153]=15'hce6; sprite_rom[4154]=15'h38e3; sprite_rom[4155]=15'h3d8f; sprite_rom[4156]=15'h18c7; sprite_rom[4157]=15'h2d26; sprite_rom[4158]=15'h5193; sprite_rom[4159]=15'h5800;
sprite_rom[4160]=15'h4571; sprite_rom[4161]=15'h49ad; sprite_rom[4162]=15'h24c9; sprite_rom[4163]=15'h866; sprite_rom[4164]=15'h843; sprite_rom[4165]=15'h18e6; sprite_rom[4166]=15'h2d06; sprite_rom[4167]=15'h3d91;
sprite_rom[4168]=15'hc4b; sprite_rom[4169]=15'h3922; sprite_rom[4170]=15'h3233; sprite_rom[4171]=15'h4e11; sprite_rom[4172]=15'h49af; sprite_rom[4173]=15'h24c9; sprite_rom[4174]=15'h866; sprite_rom[4175]=15'h843;
sprite_rom[4176]=15'h18e6; sprite_rom[4177]=15'h2ce4; sprite_rom[4178]=15'h3d91; sprite_rom[4179]=15'h2d31; sprite_rom[4180]=15'h4270; sprite_rom[4181]=15'h5271; sprite_rom[4182]=15'h5400; sprite_rom[4183]=15'h2d27;
sprite_rom[4184]=15'h4671; sprite_rom[4185]=15'h20e7; sprite_rom[4186]=15'hc66; sprite_rom[4187]=15'h863; sprite_rom[4188]=15'h18e6; sprite_rom[4189]=15'h24e4; sprite_rom[4190]=15'h4a51; sprite_rom[4191]=15'h39d1;
sprite_rom[4192]=15'h2631; sprite_rom[4193]=15'h2c7; sprite_rom[4194]=15'h5000; sprite_rom[4195]=15'h1121; sprite_rom[4196]=15'h1dca; sprite_rom[4197]=15'h24c4; sprite_rom[4198]=15'h10c6; sprite_rom[4199]=15'hc63;
sprite_rom[4200]=15'h18e6; sprite_rom[4201]=15'h2462; sprite_rom[4202]=15'h2569; sprite_rom[4203]=15'h1ce6; sprite_rom[4204]=15'h45c7; sprite_rom[4205]=15'h14; sprite_rom[4206]=15'h574; sprite_rom[4207]=15'h41c8;
sprite_rom[4208]=15'h192e; sprite_rom[4209]=15'h18c6; sprite_rom[4210]=15'hc86; sprite_rom[4211]=15'h1866; sprite_rom[4212]=15'h2d6b; sprite_rom[4213]=15'h847; sprite_rom[4214]=15'h842; sprite_rom[4215]=15'h289;
sprite_rom[4216]=15'h4c00; sprite_rom[4217]=15'h36b4; sprite_rom[4218]=15'h462b; sprite_rom[4219]=15'h447; sprite_rom[4220]=15'h421; sprite_rom[4221]=15'h4dc4; sprite_rom[4222]=15'h45f1; sprite_rom[4223]=15'h3631;
sprite_rom[4224]=15'h4dcd; sprite_rom[4225]=15'h4e0; sprite_rom[4226]=15'h2c21; sprite_rom[4227]=15'h4e31; sprite_rom[4228]=15'h4e32; sprite_rom[4229]=15'h6d6; sprite_rom[4230]=15'h421; sprite_rom[4231]=15'h51c7;
sprite_rom[4232]=15'h2e60; sprite_rom[4233]=15'h3969; sprite_rom[4234]=15'h2d3; sprite_rom[4235]=15'h0; sprite_rom[4236]=15'h680; sprite_rom[4237]=15'h422; sprite_rom[4238]=15'h226; sprite_rom[4239]=15'h5294;
end
reg [23:0] font_span_q,sprite_span_q;
reg [4:0] font_y_q,sprite_x_q; reg [1:0] sprite_lane0,sprite_lane_q;
reg [31:0] font_bits_q; reg font_valid_q,sprite_valid_q; reg [14:0] sprite_word_q;
wire [5:0] font_offset={1'b0,font_y_q}-{1'b0,font_span_q[4:0]};
wire [5:0] sprite_offset={1'b0,sprite_x_q}-{1'b0,sprite_span_q[4:0]};
wire font_hit=font_y_q>=font_span_q[4:0] && font_offset<font_span_q[10:5];
wire sprite_hit=sprite_x_q>=sprite_span_q[4:0] && sprite_offset<sprite_span_q[10:5];
wire [12:0] font_address=font_span_q[23:11]+{7'd0,font_offset};
wire [12:0] sprite_address_compact=sprite_span_q[23:11]+{7'd0,sprite_offset};
wire [12:0] selected_font_address=focus_font_select ? focus_font_address : subtitle_font_address;
assign subtitle_font_row=font_valid_q ? font_bits_q : 32'd0;
wire [4:0] sprite_q=sprite_lane_q==0 ? sprite_word_q[4:0] : sprite_lane_q==1 ? sprite_word_q[9:5] : sprite_word_q[14:10];
assign sprite_pixel=sprite_valid_q ? {sprite_q!=0,sprite_q} : 6'd0;
assign focus_font_row=subtitle_font_row;
assign teacher_x=11'd1128; assign teacher_y=11'd432;
assign scroll_step=4'd3;
always @* begin demo_start_cs=0;demo_end_cs=0;focus_lines=0;case(page)
4'd6:begin demo_start_cs=16'd350;demo_end_cs=16'd930;focus_lines=2'd3;end
4'd7:begin demo_start_cs=16'd80;demo_end_cs=16'd510;focus_lines=2'd2;end
4'd8:begin demo_start_cs=16'd480;demo_end_cs=16'd820;focus_lines=2'd2;end
4'd9:begin demo_start_cs=16'd650;demo_end_cs=16'd1180;focus_lines=2'd3;end
4'd10:begin demo_start_cs=16'd300;demo_end_cs=16'd750;focus_lines=2'd2;end
4'd11:begin demo_start_cs=16'd100;demo_end_cs=16'd1060;focus_lines=2'd2;end
default:;endcase end
always @* begin text_length=0; case(page)
1:text_length=7'd38;
2:text_length=7'd25;
3:text_length=7'd38;
4:text_length=7'd37;
5:text_length=7'd29;
6:text_length=7'd45;
7:text_length=7'd36;
8:text_length=7'd44;
9:text_length=7'd54;
10:text_length=7'd55;
11:text_length=7'd52;
12:text_length=7'd14;
default:; endcase end
always @(posedge clk) begin
 character_id<={1'b0,text_id_hi[{page,character_index}],text_id_lo[{page,character_index}]};
 focus_id<={1'b0,focus_id_rom[{page,focus_index}]};
 font_span_q<=font_span_rom[selected_font_address[12:5]]; font_y_q<=selected_font_address[4:0];
 font_bits_q<=font_rom[font_hit ? font_address : 13'd0]; font_valid_q<=font_hit;
 sprite_span_q<=sprite_span_rom[sprite_address[14:5]]; sprite_x_q<=sprite_address[4:0];
 sprite_lane0<=sprite_lane; sprite_lane_q<=sprite_lane0;
 sprite_word_q<=sprite_rom[sprite_hit ? sprite_address_compact : 13'd0]; sprite_valid_q<=sprite_hit;
end
endmodule
module album_teacher_palette(input [4:0] index,output reg [23:0] rgb);
always @* case(index)
5'd0:rgb=24'h000000;
5'd1:rgb=24'hfbefe4;
5'd2:rgb=24'hf5e8db;
5'd3:rgb=24'hf5e5d9;
5'd4:rgb=24'hf4e2d5;
5'd5:rgb=24'hf9ddcf;
5'd6:rgb=24'headed1;
5'd7:rgb=24'he4dace;
5'd8:rgb=24'hebd2c4;
5'd9:rgb=24'hd8cfc3;
5'd10:rgb=24'he2c2b4;
5'd11:rgb=24'hcdc1b6;
5'd12:rgb=24'hbfc3b7;
5'd13:rgb=24'hbabcb1;
5'd14:rgb=24'hbcb5a9;
5'd15:rgb=24'haeb3a8;
5'd16:rgb=24'hc2a497;
5'd17:rgb=24'haaa498;
5'd18:rgb=24'h9da194;
5'd19:rgb=24'h939084;
5'd20:rgb=24'h8f8074;
5'd21:rgb=24'h876759;
5'd22:rgb=24'h675951;
5'd23:rgb=24'h594540;
5'd24:rgb=24'h513e3a;
5'd25:rgb=24'h4a3c39;
5'd26:rgb=24'h443937;
5'd27:rgb=24'h403433;
5'd28:rgb=24'h3d2f2e;
5'd29:rgb=24'h352c2b;
5'd30:rgb=24'h2f2625;
5'd31:rgb=24'h261b1a;
default:rgb=0; endcase endmodule
// END GENERATED TEACHING ASSETS
