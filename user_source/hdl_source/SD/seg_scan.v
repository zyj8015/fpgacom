// Board-feedback correction: high-active scan lit all eight positions and
// reversed the requested pair. Drive active-LOW FPGA digit controls instead.
// Only two positions are used: logical 0 units/n -> SEL7/D11,
// logical 1 tens/o -> SEL6/E11. All other selects remain high (off).
module seg_scan #(
    parameter ACTIVE_DIGITS=2,
    parameter SCAN_FREQ=200,
    parameter CLK_FREQ=50000000,
    parameter SCAN_COUNT=CLK_FREQ/(SCAN_FREQ*ACTIVE_DIGITS)-1
)(
    input clk, rst_n,
    output reg [7:0] seg_sel,
    output reg [7:0] seg_data,
    input [7:0] seg_data_0, seg_data_1, seg_data_2,
    input [7:0] seg_data_3, seg_data_4, seg_data_5
);
reg [31:0] scan_timer;
reg [2:0] scan_sel;
reg enable_pending;
reg [7:0] selected_segments;
always @* begin
    case(scan_sel)
        0: selected_segments=seg_data_0;
        1: selected_segments=seg_data_1;
        2: selected_segments=seg_data_2;
        3: selected_segments=seg_data_3;
        4: selected_segments=seg_data_4;
        5: selected_segments=seg_data_5;
        default: selected_segments=8'hff;
    endcase
end
always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        scan_timer<=0; scan_sel<=0; enable_pending<=0;
        seg_sel<=8'hff; seg_data<=8'hff;
    end else if(scan_timer>=SCAN_COUNT) begin
        // Break before make: disable all digit supplies before changing data.
        seg_sel<=8'hff; scan_timer<=0; enable_pending<=0;
        scan_sel<=scan_sel==ACTIVE_DIGITS-1 ? 3'd0 : scan_sel+1'b1;
    end else begin
        scan_timer<=scan_timer+1'b1;
        if(!enable_pending) begin
            seg_sel<=8'hff; seg_data<=selected_segments; enable_pending<=1;
        end else seg_sel<=~(8'h80 >> scan_sel);
    end
end
endmodule
