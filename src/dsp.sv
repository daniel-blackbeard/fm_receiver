module dsp (
    input  logic               clk,
    input  logic               rstb,
    input  logic               valid,
    input  logic signed [11:0] rx_data_i1,
    input  logic signed [11:0] rx_data_q1,
    input  logic signed [11:0] rx_data_i2,
    input  logic signed [11:0] rx_data_q2,
    output logic signed [15:0] o_data1,
    output logic signed [15:0] o_data2,
    output logic               o_valid,
    // Register settings
    input  logic        [31:0] phase_step,
    input  logic        [31:0] cfg0,
    // debug ports
    output logic        [63:0] debug
);
// notes: 2:39 - 18.42KHz
//        2:42 - 16.90KHz
//        2:49 - 14.18KHz
//        2:55 - 11.82KHz
//        Estimated drift: 6.88Hz/s, always downwards
// cfg0[31:0] documentation:
// cfg0[ 0] : AFC loop enable
// cfg0[28] : DISCRIMINATOR reset
// cfg0[29] : DECIMATOR reset
// cfg0[30] : PLL reset
// cfg0[31] : DEMODULATOR reset

logic signed [11:0] sin_s, cos_s;
logic signed [23:0] p1, p2;
logic signed [24:0] I_out_ext, Q_out_ext;
logic signed [15:0] I_out_dec, Q_out_dec;
logic signed [15:0] I_out_fir, Q_out_fir;

logic strb, fir_done, disc_done;
logic signed [15:0] mpx_data, mpx_angle, mpx_angle_last;
logic signed [31:0] mpx_data_lowfreq, phase_step_corrected;

// --- CFG0 enables -----------------------------------------------------------------
assign phase_step_corrected = cfg0[0] ? phase_step + mpx_data_lowfreq : phase_step;

// ----------------------------------------------------------------------------------

always_ff @(posedge clk) begin
    p1        <= rx_data_i1 * cos_s;
    I_out_ext <= p1 + (rx_data_q1 * sin_s);
end

always_ff @(posedge clk) begin
    p2        <= rx_data_q1 * cos_s;
    Q_out_ext <= p2 - (rx_data_i1 * sin_s);
end

synth_core u_synth_nco(
    .clk(clk), 
    .rstb(rstb), 
    .phase_step(phase_step_corrected),
    .I(cos_s), 
    .Q(sin_s)
);

// --- CIC -------------------------------------------------------------------------
cic_dec u_cic_stage (
    .clk(clk),
    .rstb(rstb),
    .rx_i(I_out_ext[23 -:12]),
    .rx_q(Q_out_ext[23 -:12]),
    .dec_i(I_out_dec),
    .dec_q(Q_out_dec),
    .strb(strb)
);

fir_time_multiplexed u_iq_fir(
  .clk(clk),
  .rstb(rstb),
  .data_in_i(I_out_dec),
  .data_in_q(Q_out_dec),
  .valid(strb),
  .data_out_i(I_out_fir),
  .data_out_q(Q_out_fir),
  .done(fir_done)
);



// --- DISC -------------------------------------------------------------------------
cordic_vector u_discriminator (
    .clk   (clk),
    .rstb  (rstb & ~cfg0[28]),
    .strb  (fir_done),
    .I_in  (I_out_fir),
    .Q_in  (Q_out_fir),
    .phase (mpx_angle),
    .valid (disc_done)
);


always_ff @(posedge clk) begin
    if(fir_done) mpx_angle_last <= mpx_angle;
    if(disc_done) mpx_data <= (mpx_angle - mpx_angle_last);
end

logic signed [31:0] mpx_data_accum;
always_ff @(posedge clk) begin
    // fs = 1.2MHz, k=15, pole at ~5.8Hz
    if(disc_done) mpx_data_accum <= mpx_data_accum + mpx_data - (mpx_data_accum >>> 15);
end

assign mpx_data_lowfreq = mpx_data_accum >>> 7;


logic signed [15:0] mpx_data_desc;
logic               mpx_valid;

mpx_decimator u_mpx_dec (
    .clk      (clk),
    .rstb     (rstb & ~cfg0[29]),
    .data_in  (mpx_data),
    .valid    (disc_done),
    .data_out (mpx_data_desc),
    .strb     (mpx_valid)
);

logic        pll_valid;
logic signed [15:0] pll_vco1_sin, pll_vco1_cos;
logic signed [15:0] pll_vco2_sin, pll_vco2_cos;
logic signed [15:0] pll_vco3_sin, pll_vco3_cos;

pll u_pll (
    .clk        (clk),
    .rstb       (rstb & ~cfg0[30]),
    .i_data     (mpx_data_desc),
    .strb       (mpx_valid),
    .o_vco1_sin (pll_vco1_sin),
    .o_vco1_cos (pll_vco1_cos),
    .o_vco2_sin (pll_vco2_sin),
    .o_vco2_cos (pll_vco2_cos),
    .o_vco3_sin (pll_vco3_sin),
    .o_vco3_cos (pll_vco3_cos),
    .valid      (pll_valid)
);

logic               mpx_demod_valid;
logic signed [15:0] mpx_demod_audio_r, mpx_demod_audio_l, mpx_demod_rds_data;
logic signed [15:0] mpx_demod_mono, mpx_demod_ster;

mpx_demod u_mpx_demod (
    .clk         (clk),
    .rstb        (rstb & ~cfg0[31]),
    .i_mpx_data  (mpx_data_desc),
    .strb        (pll_valid),
    .i_vco1_sin  (pll_vco1_sin),
    .i_vco1_cos  (pll_vco1_cos),
    .i_vco2_sin  (pll_vco2_sin),
    .i_vco2_cos  (pll_vco2_cos),
    .i_vco3_sin  (pll_vco3_sin),
    .i_vco3_cos  (pll_vco3_cos),
    .valid       (mpx_demod_valid),
    .o_audio_r   (mpx_demod_audio_r),
    .o_audio_l   (mpx_demod_audio_l),
    .o_rds_data  (mpx_demod_rds_data),
    .o_mono      (mpx_demod_mono),
    .o_ster      (mpx_demod_ster)
);

assign o_valid = mpx_demod_valid;
assign debug   = {mpx_demod_mono, mpx_demod_ster, mpx_demod_rds_data, pll_vco1_sin};

endmodule