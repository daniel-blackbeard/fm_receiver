`timescale 1ns / 1ps

module synth_core(
  input  logic               clk,
  input  logic               rstb,
  input  logic        [31:0] phase_step,
  output logic signed [11:0] I,
  output logic signed [11:0] Q
);

logic [10:0] sin, cos;
logic [31:0] phase_accum;
logic  [9:0] addr;
logic  [1:0] quadrant;
logic signed [11:0] cos_s, sin_s;

always @(posedge clk) begin
  if(~rstb) begin
    phase_accum <= '0;
    quadrant    <= '0;
  end else begin
    phase_accum <= phase_accum + phase_step;
    quadrant    <= phase_accum[31:30];
  end
end

always_comb begin
  case(phase_accum[31:30])
    2'b00: addr = phase_accum[29:20];
    2'b01: addr = 10'h3FF  - phase_accum[29:20];
    2'b10: addr = phase_accum[29:20];
    2'b11: addr = 10'h3FF  - phase_accum[29:20];
  endcase
  
  case(quadrant)
    2'b00: begin I =  cos_s; Q =  sin_s; end
    2'b01: begin I = -cos_s; Q =  sin_s; end
    2'b10: begin I = -cos_s; Q = -sin_s; end
    2'b11: begin I =  cos_s; Q = -sin_s; end
  endcase
end

assign cos_s = {1'b0, cos};
assign sin_s = {1'b0, sin};

rom_sincos u_sincos_lut(.clk(clk), .addr(addr), .sin(sin), .cos(cos));

endmodule
