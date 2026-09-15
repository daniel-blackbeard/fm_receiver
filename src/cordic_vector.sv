`timescale 1ns / 1ps
module cordic_vector (
    input  logic        clk,
    input  logic        rstb,
    (* mark_debug = "true" *) input  logic        strb,
    input  logic [15:0] I_in,
    input  logic [15:0] Q_in,
    output logic [15:0] phase,
    (* mark_debug = "true" *) output logic        valid
);

logic [13:0] atan_lut [0:15];
initial begin
    atan_lut[0]  = 14'd8192;
    atan_lut[1]  = 14'd4836;
    atan_lut[2]  = 14'd2556;
    atan_lut[3]  = 14'd1297;
    atan_lut[4]  = 14'd651;
    atan_lut[5]  = 14'd326;
    atan_lut[6]  = 14'd163;
    atan_lut[7]  = 14'd81;
    atan_lut[8]  = 14'd41;
    atan_lut[9]  = 14'd20;
    atan_lut[10] = 14'd10;
    atan_lut[11] = 14'd5;
    atan_lut[12] = 14'd3;
    atan_lut[13] = 14'd1;
    atan_lut[14] = 14'd1;
    atan_lut[15] = 14'd0;
end

typedef enum logic [1:0] {IDLE, RUNNING, DONE} state_t;
(* mark_debug = "true" *) state_t state;

logic signed [17:0] x, y;
// 16 bits, not 15 -- the vectoring recurrence's convergence range (sum of
// the full atan LUT, ~16881 in this Q16-angle scale) exceeds 90 deg
// (16384) by design margin, and z transiently -- and, for an input near
// exactly +-90 deg, even at steady state -- reaches into that margin.
// 15 bits (max magnitude 16383) overflows there; 16 bits (32767) doesn't.
logic signed [15:0] z;
(* mark_debug = "true" *) logic [3:0] iter;
logic               fold;

logic d;
assign d = ~y[17];

logic signed [17:0] shift_x, shift_y;
assign shift_x = x >>> iter;
assign shift_y = y >>> iter;

logic signed [15:0] delta_z;
assign delta_z = $signed({2'b00, atan_lut[iter]});

logic               x_negated;
logic signed [15:0] x_pre, y_pre;
assign x_negated = I_in[15];
assign x_pre = x_negated ? -I_in : I_in;
assign y_pre = x_negated ? -Q_in : Q_in;

always_ff @(posedge clk) begin
    if(~rstb) begin
        state <= IDLE;
        valid <= '0;
        x     <= '0;
        y     <= '0;
        z     <= '0;
        iter  <= '0;
        fold  <= '0;
        phase <= '0;
    end else begin
        valid <= 1'b0;

        case(state)
            IDLE : begin
                if(strb) begin
                    x     <= 18'(x_pre);
                    y     <= 18'(y_pre);
                    z     <= '0;
                    fold  <= x_negated;
                    iter  <= '0;
                    state <= RUNNING;
                end
            end
            RUNNING: begin
                if(!d) begin
                    x <= x - shift_y;
                    y <= y + shift_x;
                    z <= z - delta_z;
                end else begin
                    x <= x + shift_y;
                    y <= y - shift_x;
                    z <= z + delta_z;
                end

                if(iter == 15) state <= DONE;
                else iter <= iter + 1;
            end
            DONE : begin
                phase <= fold ? (z + 16'h8000) : z;
                valid <= 1'b1;
                state <= IDLE;
            end
            default : state <= IDLE;
        endcase
    end
end

endmodule