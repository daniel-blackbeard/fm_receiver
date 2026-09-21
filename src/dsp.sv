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

// We reduce the rate to 240KHz
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
logic signed [15:0] mpx_demod_audio_r, mpx_demod_audio_l;
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
    .valid       (mpx_demod_valid),
    .o_audio_r   (mpx_demod_audio_r),
    .o_audio_l   (mpx_demod_audio_l),
    .o_mono      (mpx_demod_mono),
    .o_ster      (mpx_demod_ster)
);

// RDS downconversion: one DSP slice, mpx_data x 57kHz cosine, same
// Q0.30->Q0.15 scaling convention as mpx_demod.sv's stereo mixer (this
// used to live there; moved here since rds.sv's CIC needs the raw,
// pre-filter mix, not mpx_demod's old (now-removed) weak single-pole
// filtered version).
logic signed [31:0] rds_mix;
logic signed [15:0] rds_mix_i;
logic pll_valid_d1;

always_ff @(posedge clk) begin
    if(~rstb) rds_mix <= '0;
    else if(pll_valid) rds_mix <= mpx_data_desc * pll_vco3_cos;
end
assign rds_mix_i = rds_mix >>> 15;

always_ff @(posedge clk) begin
    if(~rstb) pll_valid_d1 <= '0;
    else      pll_valid_d1 <= pll_valid;
end

logic        [15:0] rds_pi;
logic        [63:0] rds_ps;
logic       [511:0] rds_rt;
logic               rds_locked;

// Only the decoded results (S8) and the lock flag are used; every earlier
// stage (S0..S7) is left unconnected on purpose.
rds u_rds (
    .clk           (clk),
    .rstb          (rstb),
    .i_mpx_data    (rds_mix_i),
    .strb          (pll_valid_d1),
    .o_rds_data    (),
    .valid         (),
    .o_lpf_data    (),
    .o_lpf_valid   (),
    .o_nco_phase   (),
    .o_nco_valid   (),
    .o_soft        (),
    .o_biphase     (),
    .o_soft_valid  (),
    .o_data_bit    (),
    .o_data_valid  (),
    .o_syndrome    (),
    .o_offset_type (),
    .o_syn_valid   (),
    .o_block_data  (),
    .o_block_type  (),
    .o_block_valid (),
    .o_locked      (rds_locked),
    .o_pi          (rds_pi),
    .o_ps          (rds_ps),
    .o_rt          (rds_rt)
);

// ---- Decoded RDS over debug slot 3 (ch1_i), one 16-bit word per o_valid ----
// Each word is {1'b0, addr[6:0], byte[7:0]}: a free-running scan of the decoded
// registers, so the PC needs no framing -- every word says where it belongs and
// a lost word is just refreshed on the next sweep (75 words = 1.56ms @ 48kHz).
//   addr 0..1   PI (high, low)            addr 2..9   PS chars 0..7
//   addr 10..73 RadioText chars 0..63     addr 74     status (bit0 = lock)
// Matched by tools/pc_console.py (RDS_ADDR_* constants).
localparam logic [6:0] RDS_LAST_ADDR = 7'd74;

logic  [6:0] rds_scan_addr;
logic  [6:0] rds_ps_idx, rds_rt_idx;
logic  [7:0] rds_scan_byte;
logic [15:0] rds_dbg_word;

assign rds_ps_idx = rds_scan_addr - 7'd2;
assign rds_rt_idx = rds_scan_addr - 7'd10;

always_comb begin : RDS_SCAN_MUX
    if      (rds_scan_addr == 7'd0)  rds_scan_byte = rds_pi[15:8];
    else if (rds_scan_addr == 7'd1)  rds_scan_byte = rds_pi[7:0];
    else if (rds_scan_addr <  7'd10) rds_scan_byte = rds_ps[63  - 8*rds_ps_idx -: 8];
    else if (rds_scan_addr <  7'd74) rds_scan_byte = rds_rt[511 - 8*rds_rt_idx -: 8];
    else                             rds_scan_byte = {7'b0, rds_locked};
end

always_ff @(posedge clk) begin : RDS_SCAN
    if(~rstb) begin
        rds_scan_addr <= '0;
        rds_dbg_word  <= '0;
    end else if(mpx_demod_valid) begin
        rds_dbg_word  <= {1'b0, rds_scan_addr, rds_scan_byte};
        rds_scan_addr <= (rds_scan_addr == RDS_LAST_ADDR) ? 7'd0 : rds_scan_addr + 7'd1;
    end
end

assign o_valid = mpx_demod_valid;
assign debug   = {mpx_demod_audio_r, mpx_demod_audio_l, rds_dbg_word, pll_vco1_sin};

endmodule