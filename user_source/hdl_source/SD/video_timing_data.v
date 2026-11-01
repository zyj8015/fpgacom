module video_timing_data
#(
	parameter DATA_WIDTH = 16,
    parameter H_ACTIVE=640,H_FP=16,H_SYNC=96,H_BP=48,
    parameter V_ACTIVE=480,V_FP=10,V_SYNC=2,V_BP=33,
    parameter HS_POL=0,VS_POL=0
)
(
	input                       video_clk,          // Video pixel clock
	input                       rst,
	output reg                  read_req,           // Start reading a frame of data
	input                       read_req_ack,       // Read request response
	//output                      read_en,            // Read data enable
	//input[DATA_WIDTH - 1:0]     read_data,          // Read data
	output                      hs,                 // horizontal synchronization
	output                      vs,                 // vertical synchronization
	output                      de                  // video valid
	//output[DATA_WIDTH - 1:0]    vout_data           // video data
);
wire                   video_hs;
wire                   video_vs;
wire                   video_de;
//delay video_hs video_vs  video_de 2 clock cycles
reg                    video_hs_d0;
reg                    video_vs_d0;
reg                    video_de_d0;
reg                    video_hs_d1;
reg                    video_vs_d1;
reg                    video_de_d1;
// SDRAM request acknowledgment originates in mem_clk, not video_clk.
(* ASYNC_REG="TRUE" *) reg [1:0] read_ack_sync;
always @(posedge video_clk or posedge rst) begin
    if(rst) read_ack_sync<=0;
    else read_ack_sync<={read_ack_sync[0],read_req_ack};
end

//reg[DATA_WIDTH - 1:0]  vout_data_r;
//assign read_en = video_de;
assign hs = video_hs_d1;
assign vs = video_vs_d1;
assign de = video_de_d1;
//assign vout_data = vout_data_r;
always@(posedge video_clk or posedge rst)
begin
	if(rst == 1'b1)
	begin
		video_hs_d0 <= 1'b0;
		video_vs_d0 <= 1'b0;
		video_de_d0 <= 1'b0;
        video_hs_d1<=0; video_vs_d1<=0; video_de_d1<=0;
	end
	else
	begin
		//delay video_hs video_vs  video_de 2 clock cycles
		video_hs_d0 <= video_hs;
		video_vs_d0 <= video_vs;
		video_de_d0 <= video_de;
		video_hs_d1 <= video_hs_d0;
		video_vs_d1 <= video_vs_d0;
		video_de_d1 <= video_de_d0;
	end
end
/*
always@(posedge video_clk or posedge rst)
begin
	if(rst == 1'b1)
		vout_data_r <= {DATA_WIDTH{1'b0}};
	else if(video_de_d0)
		vout_data_r <= read_data;
	else
		vout_data_r <= {DATA_WIDTH{1'b0}};
end
*/
always@(posedge video_clk or posedge rst)
begin
	if(rst == 1'b1)
		read_req <= 1'b0;
	else if(video_vs_d0 & ~video_vs) //vertical synchronization edge (the rising or falling edges are OK)
		read_req <= 1'b1;
	else if(read_ack_sync[1])
		read_req <= 1'b0;
end

color_bar #(.H_ACTIVE(H_ACTIVE),.H_FP(H_FP),.H_SYNC(H_SYNC),.H_BP(H_BP),
    .V_ACTIVE(V_ACTIVE),.V_FP(V_FP),.V_SYNC(V_SYNC),.V_BP(V_BP),
    .HS_POL(HS_POL),.VS_POL(VS_POL)) color_bar_m0(
	.clk(video_clk),
	.rst(rst),
	.hs(video_hs),
	.vs(video_vs),
	.de(video_de),
	.rgb_r(),
	.rgb_g(),
	.rgb_b()
);
endmodule
