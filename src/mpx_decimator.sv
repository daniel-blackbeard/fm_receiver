`timescale 1ns / 1ps
//
// mpx_decimator.sv (2026-09-15) -- single-stage (N=1) CIC decimator,
// decimate-by-5, for narrowing pc_console.py's FFT window on mpx_data
// (1.2MHz -> 240kHz, Nyquist 600kHz -> 120kHz). Deliberately hardcoded,
// not parametric/reusable (see cic_dec.sv's parametrization attempt
// earlier this session, reverted -- that generality wasn't the right
// fit) and NOT I/Q (mpx_data is a single real signal, not a complex
// pair).
//
// Critical difference from cic_dec.sv: this module's input is NOT fresh
// every clk cycle -- mpx_data only updates once per disc_done pulse
// (~once every 25 dsp_clk cycles upstream, via the CIC+FIR+CORDIC
// chain). The integrator, the comb, and the R=5 decimation counter all
// run on/count `valid` (wire to disc_done), NOT raw clk cycles -- a
// naive port of cic_dec.sv's continuous per-cycle counting would
// silently integrate the same stale sample ~17-25x too many times
// between real updates.
//
// Sizing (same Hogenauer-bound reasoning as cic_dec.sv's 2026-09-15 fix,
// N=1 here): GROWTH=ceil(log2(R^N))=ceil(log2(5))=3 bits.
// ACC_WIDTH=IN_WIDTH+GROWTH=16+3=19 bits. OUT_SHIFT=IN_WIDTH-OUT_WIDTH+
// GROWTH=16-16+3=3 -- tightest shift with no overflow for a sustained
// full-scale input: 32768*5=163840, >>3=20480 (fits +-32767);
// >>2=40960 would NOT fit.
//
// Timing convention mirrors cic_dec.sv's own proven pattern exactly
// (comb reads the integrator's pre-this-cycle value, i.e. before this
// cycle's own addition applies -- a labeling/boundary convention that
// doesn't affect steady-state gain, only where exactly the transient
// settles after reset, same as the original).
module mpx_decimator (
    input  logic               clk,
    input  logic               rstb,
    input  logic signed [15:0] data_in,
    input  logic               valid,      // one-cycle pulse (disc_done): data_in is a fresh sample THIS cycle
    output logic signed [15:0] data_out,
    output logic               strb        // registered, one-cycle-delayed: "data_out is valid now" (same contract as cic_dec.sv's strb)
);

localparam int R         = 5;
localparam int ACC_WIDTH = 19;  // 16 + ceil(log2(5^1)) = 16+3
localparam int OUT_SHIFT = 3;   // tightest no-overflow shift, see header derivation

logic signed [ACC_WIDTH-1:0] accum, accum_last;
logic signed [ACC_WIDTH-1:0] dec_raw;
logic [2:0] valid_cnt;  // counts 0..R-1, only advances on `valid`

logic decim_event;
assign decim_event = valid && (valid_cnt == R-1);

always_ff @(posedge clk) begin
    if (~rstb) begin
        accum      <= '0;
        accum_last <= '0;
        dec_raw    <= '0;
        valid_cnt  <= '0;
        strb       <= 1'b0;
    end else begin
        strb <= decim_event;

        if (valid) begin
            valid_cnt <= (valid_cnt == R-1) ? '0 : valid_cnt + 1'b1;
            accum     <= accum + ACC_WIDTH'(data_in);

            if (decim_event) begin
                dec_raw    <= accum - accum_last;
                accum_last <= accum;
            end
        end
    end
end

logic signed [ACC_WIDTH-1:0] out_rounded;
assign out_rounded = (dec_raw + (ACC_WIDTH)'(1 <<< (OUT_SHIFT-1))) >>> OUT_SHIFT;

always_ff @(posedge clk) begin
    if (~rstb)     data_out <= '0;
    else if (strb) data_out <= out_rounded[15:0];
end

endmodule
