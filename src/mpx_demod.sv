module mpx_demod (
    input  logic               clk,
    input  logic               rstb,
    input  logic signed [15:0] i_mpx_data,
    input  logic               strb,
    input  logic signed [15:0] i_vco1_sin,
    input  logic signed [15:0] i_vco1_cos,
    input  logic signed [15:0] i_vco2_sin,
    input  logic signed [15:0] i_vco2_cos,
    output logic               valid,
    output logic signed [15:0] o_audio_r,
    output logic signed [15:0] o_audio_l,
    // Pre-matrix-combine taps for diagnosis: mono (L+R, no downconversion
    // involved) vs ster (L-R, downconverted against vco2) -- a bad PLL
    // lock shows up far more clearly here than diluted into L/R.
    output logic signed [15:0] o_mono,
    output logic signed [15:0] o_ster
);

// mono (L+R) is already at baseband in the composite, no mixing needed.

// Stereo (L-R) downconversion: one DSP slice, mpx_data x 38kHz cosine.
// Product is Q0.15xQ0.15=Q0.30 in 32 bits; >>>15 brings it back to a
// plain Q0.15 16-bit value for mpx_fir's i_mpx_ster input.
logic signed [31:0] mpx_ster_mix;
logic signed [15:0] mpx_ster_i;
always_ff @(posedge clk) begin
    if(~rstb) mpx_ster_mix <= '0;
    else if(strb) mpx_ster_mix <= i_mpx_data * i_vco2_cos;
end
assign mpx_ster_i = mpx_ster_mix >>> 15;

// Mirrors mpx_ster_mix's 1-cycle latency so mpx_fir sees mono/ster
// aligned to the same original sample, not one strobe apart.
logic signed [15:0] mpx_mono_d1;
logic strb_d1;
always_ff @(posedge clk) begin
    if(~rstb) begin
        mpx_mono_d1 <= '0;
        strb_d1     <= '0;
    end else begin
        if(strb) mpx_mono_d1 <= i_mpx_data;
        strb_d1 <= strb;
    end
end

logic fir_valid;
logic signed [15:0] fir_audio_mono, fir_audio_ster;

mpx_fir u_mpx_fir (
    .clk          (clk),
    .rstb         (rstb),
    .i_mpx_mono   (mpx_mono_d1),
    .i_mpx_ster   (mpx_ster_i),
    .strb         (strb_d1),
    .o_audio_mono (fir_audio_mono),
    .o_audio_ster (fir_audio_ster),
    .valid        (fir_valid)
);

// L = mono+ster = 2L, R = mono-ster = 2R -- 17-bit sum/diff (two 16-bit
// signed values can need the extra bit), >>>1 back to the L/R scale.
logic signed [16:0] audio_sum, audio_diff;
assign audio_sum  = 17'(fir_audio_mono) + 17'(fir_audio_ster);
assign audio_diff = 17'(fir_audio_mono) - 17'(fir_audio_ster);

always_ff @(posedge clk) begin
    if(~rstb) begin
        o_audio_l <= '0;
        o_audio_r <= '0;
        o_mono    <= '0;
        o_ster    <= '0;
        valid     <= '0;
    end else if(fir_valid) begin
        o_audio_l <= audio_sum  >>> 1;
        o_audio_r <= audio_diff >>> 1;
        o_mono    <= fir_audio_mono;
        o_ster    <= fir_audio_ster;
        valid     <= 1'b1;
    end else begin
        valid <= 1'b0;
    end
end

endmodule
