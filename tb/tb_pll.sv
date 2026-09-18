`timescale 1ns / 1ps
//
// Stimulus-driven CSV logger for pll.sv, not self-checking -- feeds a
// 19kHz pilot (tools/gen_pll_stim.py, phase-continuous step to 19005Hz
// at t=50ms for 200ms) and logs phase_err/dphase/pilot_dc once per PLL
// iteration (on valid) to output/pll_tb_log.csv for offline analysis.
//
module tb_pll;

localparam real CLK_PERIOD_NS = 1000.0/30.0; // 30MHz dsp_clk
localparam int  STRB_DIV      = 125;         // dsp_clk/125 = 240kHz
localparam int  N_SAMPLES     = 240000;      // 1s @ 240kHz

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

logic signed [15:0] stim_mem [0:N_SAMPLES-1];
initial $readmemh("pll_stim.hex", stim_mem);

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

// i_data updates with strb, held stable until the next one.
int sample_idx;
always_ff @(posedge clk) begin
    if(~rstb) begin
        sample_idx <= 0;
        i_data     <= '0;
    end else if(strb && sample_idx < N_SAMPLES) begin
        i_data     <= stim_mem[sample_idx];
        sample_idx <= sample_idx + 1;
    end
end

integer logfile;
int log_count;

initial begin
    logfile = $fopen("../output/pll_tb_log.csv", "w");
    $fwrite(logfile, "n,time_ns,phase_err,dphase,pilot_dc,pilot_mix,vco1_sin,vco1_cos\n");
    log_count = 0;
    forever begin
        @(posedge valid);
        $fwrite(logfile, "%0d,%0t,%0d,%0d,%0d,%0d,%0d,%0d\n",
            log_count, $time, dut.phase_err, dut.dphase, dut.pilot_dc,
            dut.pilot_mix, o_vco1_sin, o_vco1_cos);
        log_count++;
        if(log_count >= N_SAMPLES) begin
            $fclose(logfile);
            $display("tb_pll: logged %0d rows to output/pll_tb_log.csv", log_count);
            $finish;
        end
    end
end

initial begin
    repeat(10) @(posedge clk);
    rstb = 1'b1;
end

endmodule
