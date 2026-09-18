`timescale 1ns / 1ps
//
// tb_pll_trace.sv (2026-09-18) -- fine-grained, EVERY-dsp_clk-cycle trace
// of pll.sv's internals for the first ~12 strobes (~8000 cycles), same
// 19006Hz reference tone as tb_pll_convergence.sv. Purpose: verify the
// RTL's arithmetic against an independent Python re-implementation of
// the same difference equations for the first several iterations, to
// separate "the coded math doesn't match what was intended" (an
// implementation bug) from "the math is coded correctly but the
// resulting closed-loop dynamics just don't converge" (a design issue).
//
module tb_pll_trace;

localparam real CLK_PERIOD_NS  = 1000.0/30.0; // 30MHz dsp_clk
localparam int  STRB_DIV       = 125;         // dsp_clk/125 = 240kHz
localparam real FS_HZ          = 30_000_000.0 / STRB_DIV;
localparam real PILOT_FREQ_HZ  = 19006.0;
localparam real PI             = 3.14159265358979323846;
localparam int  AMPLITUDE      = 20000;
localparam int  N_CYCLES       = 8000; // ~12-13 strobes at 625 cycles/strobe

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
int cyc;

always_ff @(posedge clk) begin
    if(~rstb) cyc <= 0;
    else      cyc <= cyc + 1;
end

initial begin
    logfile = $fopen("pll_trace_log.csv", "w");
    $fwrite(logfile,
        "cyc,strb,i_data,vco1_sin,vco1_cos,vco1_ready,vco1_ready_d1,vco1_ready_d2,vco1_ready_d3,pilot_mix,pilot_dc1,pilot_dc2,pilot_dc,loop_accum,phase_err,dphase,active_vco,cordic_valid,ph_accum1\n");
    wait(rstb);
    repeat(N_CYCLES) begin
        @(posedge clk);
        $fwrite(logfile, "%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d\n",
            cyc, strb, i_data, o_vco1_sin, o_vco1_cos,
            dut.vco1_ready, dut.vco1_ready_d1, dut.vco1_ready_d2, dut.vco1_ready_d3,
            dut.pilot_mix, dut.pilot_dc1, dut.pilot_dc2, dut.pilot_dc,
            dut.loop_accum, dut.phase_err, dut.dphase, dut.active_vco, dut.cordic_valid, dut.ph_accum1);
    end
    $fclose(logfile);
    $display("tb_pll_trace: logged %0d cycles to pll_trace_log.csv", N_CYCLES);
    $finish;
end

initial begin
    repeat(10) @(posedge clk);
    rstb = 1'b1;
end

endmodule
