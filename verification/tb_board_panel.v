`timescale 1ns/1ps
// Optional user-run test: no FPGA vendor primitives or physical card required.
module tb_board_panel;
    reg clk=0, rst=1, next_button=1, previous_button=1;
    always #5 clk=~clk;
    wire next_event, previous_event;
    wire [7:0] selects, segments;
    integer next_count=0, previous_count=0, left_seen=0, right_seen=0;
    key_press_debounce #(.CLK_FREQ_HZ(1000),.DEBOUNCE_MS(3)) next_filter(
        .clk(clk),.rst(rst),.button_in(next_button),.press_pulse(next_event));
    key_press_debounce #(.CLK_FREQ_HZ(1000),.DEBOUNCE_MS(3)) previous_filter(
        .clk(clk),.rst(rst),.button_in(previous_button),.press_pulse(previous_event));
    seg_scan #(.ACTIVE_DIGITS(2),.SCAN_COUNT(8)) display_driver(
        .clk(clk),.rst_n(~rst),.seg_sel(selects),.seg_data(segments),
        .seg_data_0(8'hf9),.seg_data_1(8'hc0),
        .seg_data_2(8'hff),.seg_data_3(8'hff),
        .seg_data_4(8'hff),.seg_data_5(8'hff));
    always @(negedge clk) begin
        if(!rst) begin
            if(next_event) next_count=next_count+1;
            if(previous_event) previous_count=previous_count+1;
            if(selects!==8'hff && selects!==8'hbf && selects!==8'h7f)
                $fatal(1,"Only active-low SEL6/SEL7 may be enabled");
            if(selects==8'h7f) begin
                if(segments!==8'hf9) $fatal(1,"SEL7 must show units 1");
                right_seen=right_seen+1;
            end
            if(selects==8'hbf) begin
                if(segments!==8'hc0) $fatal(1,"SEL6 must show tens 0");
                left_seen=left_seen+1;
            end
        end
    end
    initial begin
        repeat(3) @(posedge clk); #1; rst=0;
        @(posedge clk); #1; next_button=0;
        repeat(25) @(posedge clk); #1;
        if(next_count!=1 || previous_count!=0)
            $fatal(1,"Next hold must emit one independent press event");
        next_button=1;
        repeat(10) @(posedge clk); #1; previous_button=0;
        repeat(25) @(posedge clk); #1;
        if(next_count!=1 || previous_count!=1)
            $fatal(1,"Previous must not emit next or repeat while held");
        previous_button=1;
        repeat(10) @(posedge clk); #1;
        if(next_count!=1 || previous_count!=1 || left_seen==0 || right_seen==0)
            $fatal(1,"Release must not emit a second page request; both digits needed");
        $display("BOARD_PANEL_PASS: separate keys; active-low SEL7 units / SEL6 tens");
        $finish;
    end
endmodule
