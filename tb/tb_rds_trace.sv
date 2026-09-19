`timescale 1ns / 1ps
//
// tb_rds_trace.sv (2026-09-19) -- cycle-accurate trace of rds.sv: feeds a
// clean, moderate-frequency baseband tone (well inside the RDS/CIC
// passband) at the 240kHz strobe rate rds.sv expects, and logs EVERY
// dsp_clk cycle (strb, counter, com5, valid) for a few dozen decimation
// events. Purpose: (1) show directly whether `valid` lines up with the
// cycle com5 actually gets its fresh value, instead of just arguing about
// it, and (2) let the decimated output samples (captured whenever valid
// pulses) be checked in Python for whether the CIC's shaping/gain looks
// sane on a real, non-trivial input.
//
module tb_rds_trace;

localparam real CLK_PERIOD_NS = 1000.0/30.0; // 30MHz dsp_clk
localparam int  STRB_DIV      = 125;         // dsp_clk/125 = 240kHz, matches rds.sv's expected input rate
localparam real FS_HZ         = 30_000_000.0 / STRB_DIV;
localparam real TONE_HZ       = 1000.0;      // well inside RDS/CIC passband, easy to eyeball
localparam real PI            = 3.14159265358979323846;
localparam int  AMPLITUDE     = 20000;
localparam int  N_DECIM_EVENTS = 60;         // ~60 decimated (48kHz-ish) output samples
localparam int  N_CYCLES      = N_DECIM_EVENTS * 5 * STRB_DIV + 2000; // margin for startup

logic clk = 0;
logic rstb = 0;
logic signed [15:0] i_mpx_data = '0;
logic strb;

logic signed [15:0] o_rds_data;
logic valid;

rds dut (
    .clk(clk), .rstb(rstb),
    .i_mpx_data(i_mpx_data), .strb(strb),
    .o_rds_data(o_rds_data), .valid(valid)
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
        ref_phase   <= 0.0;
        i_mpx_data  <= '0;
    end else if(strb) begin
        ref_phase  <= ref_phase + 2.0*PI*TONE_HZ/FS_HZ;
        i_mpx_data <= $rtoi(AMPLITUDE * $sin(ref_phase));
    end
end

// Full cycle trace -- small enough to just read directly.
integer cyclog;
integer cyc;
logic signed [27:0] com5_prev;
always_ff @(posedge clk) begin
    if(~rstb) cyc <= 0;
    else      cyc <= cyc + 1;
end

initial begin
    cyclog = $fopen("rds_cycle_trace.csv", "w");
    $fwrite(cyclog, "cyc,strb,counter,com5,com5_changed,valid,o_rds_data\n");
    com5_prev = 0;
    wait(rstb);
    forever begin
        @(posedge clk);
        $fwrite(cyclog, "%0d,%0d,%0d,%0d,%0d,%0d,%0d\n",
            cyc, strb, dut.counter, dut.com5, (dut.com5 !== com5_prev), valid, o_rds_data);
        com5_prev = dut.com5;
    end
end

// Decimated-sample log -- one row per `valid` pulse, for shaping/gain checks.
integer samplelog;
int sample_n;
initial begin
    samplelog = $fopen("rds_samples.csv", "w");
    $fwrite(samplelog, "n,time_ns,o_rds_data\n");
    sample_n = 0;
    forever begin
        @(posedge valid);
        $fwrite(samplelog, "%0d,%0t,%0d\n", sample_n, $time, o_rds_data);
        sample_n++;
        if (sample_n >= N_DECIM_EVENTS) begin
            $fclose(samplelog);
            $fclose(cyclog);
            $display("tb_rds_trace: logged %0d decimated samples, %0d cycles total.", sample_n, cyc);
            $finish;
        end
    end
end

initial begin
    repeat(10) @(posedge clk);
    rstb = 1'b1;
end

// Safety net in case valid never fires enough times.
initial begin
    wait(rstb);
    wait(cyc > N_CYCLES);
    $display("tb_rds_trace: hit cycle cap (%0d) with only %0d samples logged -- valid may not be firing as expected.", N_CYCLES, sample_n);
    $fclose(samplelog);
    $fclose(cyclog);
    $finish;
end

endmodule
