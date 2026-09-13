`timescale 1ns / 1ps

module rom_sincos(
  input  logic        clk,
  input  logic [9:0]  addr,
  output logic [10:0] sin,
  output logic [10:0] cos
);

logic [9:0] addr_neg;

(* rom_style = "block" *)
logic [10:0] mem[0:1023];
initial $readmemh("sine_table.hex", mem);

always_ff @(posedge clk) sin <= mem[addr];
always_ff @(posedge clk) cos <= mem[addr_neg];

assign addr_neg = ~addr;

endmodule
