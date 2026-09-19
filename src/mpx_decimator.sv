`timescale 1ns / 1ps

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
