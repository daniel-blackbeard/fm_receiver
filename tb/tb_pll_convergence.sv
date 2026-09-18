`timescale 1ns / 1ps
//
// tb_pll_convergence.sv (2026-09-18) -- minimal convergence/divergence
// check for pll.sv's Costas loop: a pure, constant 19006Hz reference
// tone (6Hz above F_INIT's ~19000Hz initial guess) generated directly
// here via real-valued math (no external .hex stimulus), for the full
// clean run -- no step, no noise, nothing else going on. Logs exactly
// dphase (the loop's tracked frequency), phase_err, pilot_mix (raw
// mixer product) and pilot_dc (its filtered version) on every `valid`
// pulse, for 240000 samples (~1s at the design's own 240kHz PLL rate).
//
// Convergence: dphase should settle near F_INIT's 19006Hz equivalent
// and stay there. Divergence: dphase keeps moving and never settles.
//
module tb_pll_convergence;

localparam real CLK_PERIOD_NS  = 1000.0/30.0; // 30MHz dsp_clk
localparam int  STRB_DIV       = 125;         // dsp_clk/125 = 240kHz, matches pll.sv's own assumption
localparam real FS_HZ          = 30_000_000.0 / STRB_DIV;
localparam real PILOT_FREQ_HZ  = 19006.0;     // reference tone: 6Hz above F_INIT's ~19000Hz guess
localparam real PI             = 3.14159265358979323846;
localparam int  AMPLITUDE      = 20000;       // moderate signal, well clear of 16-bit full scale
localparam int  N_SAMPLES      = 240000;      // ~1s @ 240kHz

logic clk = 0;
logic rstb = 0;
logic signed [15:0] i_data = '0;
logic strb;

logic signed [15:0] o_vco1_sin, o_vco1_cos;
logic signed [15:0] o_vco2_sin, o_vco2_cos;
logic signed [15:0] o_vco3_sin, o_vco3_cos;
logic valid;

pll dut (
    .clk(clk), .rstb(rstb), .i_data(i_data), .strb(strb),
    .o_vco1_sin(o_vco1_sin), .o_vco1_cos(o_vco1_cos),
    .o_vco2_sin(o_vco2_sin), .o_vco2_cos(o_vco2_cos),
    .o_vco3_sin(o_vco3_sin), .o_vco3_cos(o_vco3_cos),
    .valid(valid)
);

always #(CLK_PERIOD_NS/2) clk = ~clk;

// strb every STRB_DIV cycles once out of reset.
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

// Reference tone: real-valued phase accumulator, advanced once per strb,
// evaluated fresh every strobe -- constant 19006Hz for the whole run.
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
    logfile = $fopen("pll_convergence_log.csv", "w");
    $fwrite(logfile, "n,time_ns,dphase,phase_err,pilot_mix,pilot_dc\n");
    log_count = 0;
    forever begin
        @(posedge valid);
        $fwrite(logfile, "%0d,%0t,%0d,%0d,%0d,%0d\n",
            log_count, $time, dut.dphase, dut.phase_err, dut.pilot_mix, dut.pilot_dc);
        log_count++;
        if(log_count >= N_SAMPLES) begin
            $fclose(logfile);
            $display("tb_pll_convergence: logged %0d samples (~1s @ 240kHz) to pll_convergence_log.csv", log_count);
            $finish;
        end
    end
end

initial begin
    repeat(10) @(posedge clk);
    rstb = 1'b1;
end

endmodule
