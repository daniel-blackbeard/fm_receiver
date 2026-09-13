module cordic (
    input  logic        clk,
    input  logic        rstb,
    input  logic        strb,
    input  logic [15:0] phase,
    output logic [15:0] sin,
    output logic [15:0] cos,
    output logic        valid
);

localparam signed [17:0] X_INIT = 18'sd9949;

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
state_t state;

logic signed [17:0] x, y;
logic signed [14:0] z;
logic         [3:0] iter;
logic         [1:0] quad_reg;

logic  [1:0] quadrant;
logic [13:0] fold_angle;

assign quadrant = phase[15:14];
assign fold_angle = quadrant[0] ? ~phase[13:0] : phase[13:0];

logic d;
assign d = z[14];

logic signed [17:0] shift_x, shift_y;
assign shift_x = x >>> iter;
assign shift_y = y >>> iter;

logic signed [14:0] delta_z;
assign delta_z = $signed({1'b0, atan_lut[iter]});

always_ff @(posedge clk) begin
    if(~rstb) begin
        state    <= IDLE;
        valid    <= '0;
        x        <= '0;
        y        <= '0;
        z        <= '0;
        iter     <= '0;
        quad_reg <= '0;
        sin      <= '0;
        cos      <= '0;
    end else begin
        valid <= 1'b0;

        case(state)
            IDLE : begin
                if(strb) begin
                    x        <= X_INIT;
                    y        <= '0;
                    z        <= {1'b0, fold_angle};
                    quad_reg <= quadrant;
                    iter     <= '0;
                    state    <= RUNNING;
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
                unique case (quad_reg)
                    2'b00 : begin sin <=  $signed(y[15:0]); cos <=  $signed(x[15:0]); end
                    2'b01 : begin sin <=  $signed(y[15:0]); cos <= -$signed(x[15:0]); end
                    2'b10 : begin sin <= -$signed(y[15:0]); cos <= -$signed(x[15:0]); end
                    2'b11 : begin sin <= -$signed(y[15:0]); cos <=  $signed(x[15:0]); end
                endcase
                valid <= 1'b1;
                state <= IDLE;
            end
            default : state <= IDLE;
        endcase
    end
end

endmodule