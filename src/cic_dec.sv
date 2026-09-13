module cic_dec (
    input  logic               clk,
    input  logic               rstb,
    input  logic signed [11:0] rx_i,
    input  logic signed [11:0] rx_q,
    output logic signed [15:0] dec_i,
    output logic signed [15:0] dec_q,
    output logic               strb   // registered, one cycle delayed from the internal decimation event -- see comment below
);

logic signed [25:0] accum1_i, accum1_q;
logic signed [25:0] accum2_i, accum2_q;
logic signed [25:0] accum3_i, accum3_q, accum3_i_last, accum3_q_last;
logic signed [25:0] dec1_i, dec1_q, dec1_i_last, dec1_q_last;
logic signed [25:0] dec2_i, dec2_q, dec2_i_last, dec2_q_last;
logic signed [25:0] dec3_i, dec3_q;
logic         [4:0] strb_cnt;

// Internal decimation-event trigger (un-delayed): fires the same cycle
// the comb math below runs. dec3_i/dec3_q (hence dec_i/dec_q, the gain-
// corrected outputs derived from them below) don't reflect that math's
// result until the FOLLOWING cycle (non-blocking assignment), so the
// exported `strb` port is instead a registered, one-cycle-delayed copy
// of this trigger. Its contract to a consumer is "dec_i/dec_q are valid
// now," not "the comb math is happening now, ask again next cycle" --
// nothing downstream ever needs its own extra delay to use this module
// correctly.
logic strb_internal;
assign strb_internal = (strb_cnt == 5'b0);

always_ff @(posedge clk) begin
    if(~rstb) begin
        accum1_i <= '0;
        accum1_q <= '0;
        accum2_i <= '0;
        accum2_q <= '0;
        accum3_i <= '0;
        accum3_q <= '0;
        accum3_i_last <= '0;
        accum3_q_last <= '0;
        dec1_i_last   <= '0;
        dec1_q_last   <= '0;
        dec2_i_last   <= '0;
        dec2_q_last   <= '0;
        dec1_i   <= '0;
        dec1_q   <= '0;
        dec2_i   <= '0;
        dec2_q   <= '0;
        dec3_i   <= '0;
        dec3_q   <= '0;
        strb_cnt <= '0;
        strb     <= '0;
    end else begin
        strb_cnt <= strb_cnt == 5'b11000 ?               '0 : strb_cnt + 1;
        strb     <= strb_internal;
        if(strb_internal) begin
            dec1_i <= accum3_i - accum3_i_last;
            dec1_q <= accum3_q - accum3_q_last;
            accum3_i_last <= accum3_i;
            accum3_q_last <= accum3_q;

            dec2_i <= dec1_i - dec1_i_last;
            dec2_q <= dec1_q - dec1_q_last;
            dec1_i_last <= dec1_i;
            dec1_q_last <= dec1_q;

            dec3_i <= dec2_i - dec2_i_last;
            dec3_q <= dec2_q - dec2_q_last;
            dec2_i_last <= dec2_i;
            dec2_q_last <= dec2_q;
        end

        accum1_i <= accum1_i + rx_i;
        accum1_q <= accum1_q + rx_q;
        accum2_i <= accum2_i + accum1_i;
        accum2_q <= accum2_q + accum1_q;
        accum3_i <= accum3_i + accum2_i;
        accum3_q <= accum3_q + accum2_q;
    end
end

// Gain correction (R^3 = 25^3, full 14-bit compensating shift) plus
// round-to-nearest (adding half the discarded LSB weight, 2^13, before
// truncating) -- a bit-select can't apply directly to a parenthesized
// expression in Verilog, hence the explicit wide temporaries below.
logic signed [25:0] dec_i_rounded, dec_q_rounded;
assign dec_i_rounded = (dec3_i + 26'sd8192) >>> 14;
assign dec_q_rounded = (dec3_q + 26'sd8192) >>> 14;
assign dec_i = dec_i_rounded[15:0];
assign dec_q = dec_q_rounded[15:0];

endmodule