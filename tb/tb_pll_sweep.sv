`timescale 1ns / 1ps
//
// tb_pll_sweep.sv (2026-09-18) -- runs several pll.sv KP_SHIFT/KI_SHIFT
// combinations SIMULTANEOUSLY against the same shared 19006Hz reference
// stimulus (same setup as tb_pll_convergence.sv), logging each instance's
// dphase once per strb -- one sim run instead of one compile+run per
// combination. Loop gains aren't analytically well-known here (loop/
// phase-detector/VCO gain constants), so this sweeps and compares
// empirically rather than trusting the Kp/sqrt(Ki) damping-ratio formula
// directly.
//
module tb_pll_sweep;

localparam real CLK_PERIOD_NS  = 1000.0/30.0; // 30MHz dsp_clk
localparam int  STRB_DIV       = 125;         // dsp_clk/125 = 240kHz
localparam real FS_HZ          = 30_000_000.0 / STRB_DIV;
localparam real PILOT_FREQ_HZ  = 19006.0;
localparam real PI             = 3.14159265358979323846;
localparam int  AMPLITUDE      = 20000;
localparam int  N_SAMPLES      = 240000; // ~1s @ 240kHz

logic clk = 0;
logic rstb = 0;
logic signed [15:0] i_data = '0;
logic strb;

// Four configurations under test, sharing the same stimulus.
logic signed [15:0] vs_a, vc_a, vs_b, vc_b, vs_c, vc_c, vs_d, vc_d;
logic dummy1, dummy2; // unused vco2/vco3 outputs, wired but not logged
logic signed [15:0] junk16;
logic valid_a, valid_b, valid_c, valid_d;

pll #(.KP_SHIFT(12), .KI_SHIFT(25)) dutA ( // original
    .clk(clk), .rstb(rstb), .i_data(i_data), .strb(strb),
    .o_vco1_sin(vs_a), .o_vco1_cos(vc_a),
    .o_vco2_sin(), .o_vco2_cos(), .o_vco3_sin(), .o_vco3_cos(),
    .valid(valid_a)
);

pll #(.KP_SHIFT(11), .KI_SHIFT(25)) dutB ( // current deployed
    .clk(clk), .rstb(rstb), .i_data(i_data), .strb(strb),
    .o_vco1_sin(vs_b), .o_vco1_cos(vc_b),
    .o_vco2_sin(), .o_vco2_cos(), .o_vco3_sin(), .o_vco3_cos(),
    .valid(valid_b)
);

pll #(.KP_SHIFT(10), .KI_SHIFT(25)) dutC ( // more aggressive Kp
    .clk(clk), .rstb(rstb), .i_data(i_data), .strb(strb),
    .o_vco1_sin(vs_c), .o_vco1_cos(vc_c),
    .o_vco2_sin(), .o_vco2_cos(), .o_vco3_sin(), .o_vco3_cos(),
    .valid(valid_c)
);

pll #(.KP_SHIFT(11), .KI_SHIFT(26)) dutD ( // current Kp, weaker Ki
    .clk(clk), .rstb(rstb), .i_data(i_data), .strb(strb),
    .o_vco1_sin(vs_d), .o_vco1_cos(vc_d),
    .o_vco2_sin(), .o_vco2_cos(), .o_vco3_sin(), .o_vco3_cos(),
    .valid(valid_d)
);

always #(CLK_PERIOD_NS/2) clk = ~clk;

int div_cnt;
always_ff @(posedge clk) begin
    if(~rstb) begin
        div_cnt <= '0;
        strb    <= 1'b0;
    end else if(div_cnt == STRB_DIV-1) begin
        div_cnt <= '0;
        strb    <= 1'b1;
    end else begin
        div_cnt <= div_cnt + 1'b1;
        strb    <= 1'b0;
    end
end

real ref_phase;
always_ff @(posedge clk) begin
    if(~rstb) begin
        ref_phase <= 0.0;
        i_data    <= '0;
    end else if(strb) begin
        ref_phase <= ref_phase + 2.0*PI*PILOT_FREQ_HZ/FS_HZ;
        i_data    <= $rtoi(AMPLITUDE * $sin(ref_phase));
    end
end

integer logfile;
int log_count;

initial begin
    logfile = $fopen("pll_sweep_log.csv", "w");
    $fwrite(logfile, "n,time_ns,dphase_a,dphase_b,dphase_c,dphase_d\n");
    log_count = 0;
    // Sampled on a shared strb-derived cadence rather than any one
    // instance's own `valid` (each instance's internal pipeline timing
    // is identical since they share strb, so this is safe and avoids
    // needing to pick one instance's valid as "the" clock for all four).
    forever begin
        @(posedge valid_b);
        $fwrite(logfile, "%0d,%0t,%0d,%0d,%0d,%0d\n",
            log_count, $time, dutA.dphase, dutB.dphase, dutC.dphase, dutD.dphase);
        log_count++;
        if(log_count >= N_SAMPLES) begin
            $fclose(logfile);
            $display("tb_pll_sweep: logged %0d samples (~1s @ 240kHz) to pll_sweep_log.csv", log_count);
            $finish;
        end
    end
end

initial begin
    repeat(10) @(posedge clk);
    rstb = 1'b1;
end

endmodule
