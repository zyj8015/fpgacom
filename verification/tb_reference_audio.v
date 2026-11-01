`timescale 1ns/1ps
module tb_reference_audio;
    reg clk=0, rst=1, play=0, collect=0;
    always #20 clk=~clk;
    wire valid, acr_valid;
    wire [23:0] left_pcm, right_pcm;
    wire [19:0] cts,n;
    reg [23:0] first_loop[0:8191];
    integer samples=0, nonzero=0, packets=0, cycles=0;
    integer last_sample=0, last_acr=0, positive=0, negative=0;
    // Two complete eight-note loops, shortened only by this test parameter.
    hdmi_test_audio #(.NOTE_SAMPLES(1024)) dut(
        .clk(clk), .rst(rst), .play_enable(play), .audio_valid(valid),
        .audio_left(left_pcm), .audio_right(right_pcm),
        .acr_valid(acr_valid), .acr_n(n), .acr_cts(cts)
    );
    always @(posedge clk) if(!rst) begin
        cycles=cycles+1;
        if(acr_valid) begin
            if(n!=6144 || cts!=25000) $fatal(1,"Incorrect fixed ACR");
            if(last_acr!=0 && cycles-last_acr!=25000) $fatal(1,"ACR cadence changed");
            last_acr=cycles; packets=packets+1;
        end
        if(valid) begin
            if(last_sample!=0 && cycles-last_sample!=520 && cycles-last_sample!=521)
                $fatal(1,"PCM cadence changed");
            last_sample=cycles;
            if(!play && (left_pcm!==0 || right_pcm!==0)) $fatal(1,"Sound before play_enable");
            if(collect) begin
                if((^left_pcm===1'bx) || left_pcm!==right_pcm) $fatal(1,"Unknown/unpaired stereo sample");
                if($signed(left_pcm)>1050000 || $signed(left_pcm)<-1050000) $fatal(1,"Audio overflow");
                if(samples<8192) first_loop[samples]=left_pcm;
                else if(left_pcm!==first_loop[samples-8192]) $fatal(1,"Second loop differs at %0d",samples-8192);
                if(left_pcm!=0) nonzero=nonzero+1;
                if($signed(left_pcm)>0) positive=positive+1;
                if($signed(left_pcm)<0) negative=negative+1;
                samples=samples+1;
            end
        end
    end
    initial begin
        #1000; rst=0;
        #1000000;
        @(negedge clk); play=1;
        repeat(4) @(negedge clk); collect=1;
        wait(samples==16384);
        @(negedge clk);
        if(nonzero<14000 || positive==0 || negative==0 || packets<300)
            $fatal(1,"Missing test scale samples");
        $display("TEST_AUDIO_PASS: 2 complete loops identical; exact sample/ACR cadence; initial silence");
        $finish;
    end
    initial begin #400000000; $fatal(1,"Audio loop timeout"); end
endmodule
