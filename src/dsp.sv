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
    // debug ports
    output logic        [63:0] debug
);

logic signed [11:0] sin_s, cos_s;
logic signed [23:0] p1, p2;
logic signed [24:0] I_out_ext, Q_out_ext;
logic signed [15:0] I_out_dec, Q_out_dec;
logic signed [15:0] I_out_fir, Q_out_fir;

logic strb, fir_done;
logic signed [15:0] mpx_data, mpx_angle, mpx_angle_last;

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
    .phase_step(phase_step), 
    .I(cos_s), 
    .Q(sin_s)
);
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

logic disc_done;

cordic_vector u_discriminator (
    .clk   (clk),
    .rstb  (rstb),
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

logic signed [15:0] mpx_data_desc;
logic               mpx_valid;

mpx_decimator u_mpx_dec (
    .clk      (clk),
    .rstb     (rstb),
    .data_in  (mpx_data),
    .valid    (disc_done),
    .data_out (mpx_data_desc),
    .strb     (mpx_valid)
);


assign o_valid = mpx_valid;
assign debug   = {mpx_data_desc, 16'b0, 16'b0, 16'b0};
endmodule