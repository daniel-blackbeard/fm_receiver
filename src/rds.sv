module rds (
    input  logic               clk,
    input  logic               rstb,
    input  logic signed [15:0] i_mpx_data,
    input  logic               strb,
    output logic signed [15:0] o_rds_data,
    output logic               valid
);

localparam int OUT_SHIFT = 12;

logic signed [27:0] out_rounded;
logic signed [27:0] int1, int2, int3, int4, int5;
logic signed [27:0] com1, com2, com3, com4, com5;
logic signed [27:0] int5_d1, com1_d1, com2_d1, com3_d1, com4_d1;
logic         [2:0] counter;

logic decim_event;
assign decim_event = strb && (counter == 3'b0);

// First thing: decimate by 5 to match audio rate to use the same channel
always_ff @(posedge clk) begin
    if(~rstb) begin
        valid      <= '0;
        int1       <= '0;
        int2       <= '0;
        int3       <= '0;
        int4       <= '0;
        int5       <= '0;
        com1       <= '0;
        com2       <= '0;
        com3       <= '0;
        com4       <= '0;
        com5       <= '0;
        int5_d1    <= '0;
        com1_d1    <= '0;
        com2_d1    <= '0;
        com3_d1    <= '0;
        com4_d1    <= '0;
        counter    <= '0;
    end else begin
        valid <= decim_event;
        if(strb) begin
            counter <= (counter == 3'b100) ? 3'b0 : counter + 3'b1;
            int1 <= int1 + i_mpx_data;
            int2 <= int2 + int1;
            int3 <= int3 + int2;
            int4 <= int4 + int3;
            int5 <= int5 + int4;
            if(counter == 3'b0) begin
                com1 <= int5 - int5_d1; int5_d1 <= int5;
                com2 <= com1 - com1_d1; com1_d1 <= com1;
                com3 <= com2 - com2_d1; com2_d1 <= com2;
                com4 <= com3 - com3_d1; com3_d1 <= com3;
                com5 <= com4 - com4_d1; com4_d1 <= com4;
            end
        end
    end
end

assign out_rounded = (com5 + (28'sd1 <<< (OUT_SHIFT-1))) >>> OUT_SHIFT;
assign o_rds_data  = out_rounded[15:0];

endmodule