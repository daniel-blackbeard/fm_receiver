`timescale 1ns / 1ps
// Throwaway debug harness (2026-09-15): dumps fir_time_multiplexed's
// internal pipeline every cycle for the first ~2 conversions of a small
// dynamic chirp, to find the exact mechanism behind Q_out_fir sticking
// at +32767 (see tb_fir_dynamic.sv, which reproduced this). Not meant to
// be a permanent regression test -- just instrumentation.
module tb_fir_debug_trace;
    localparam real CLK_PERIOD_NS = 33.33;
    localparam int  R = 25;
    localparam int  NUM_TAPS = 48;

    logic clk;
    initial clk = 0;
    always #(CLK_PERIOD_NS/2.0) clk = ~clk;

    logic rstb;
    logic signed [15:0] data_in_i, data_in_q;
    logic fir_valid;
    logic signed [15:0] I_out_fir, Q_out_fir;
    logic fir_done;

    fir_time_multiplexed #(.NUM_TAPS(NUM_TAPS)) u_fir (
        .clk(clk), .rstb(rstb),
        .data_in_i(data_in_i), .data_in_q(data_in_q), .valid(fir_valid),
        .data_out_i(I_out_fir), .data_out_q(Q_out_fir), .done(fir_done)
    );

    initial begin
        data_in_i <= '0; data_in_q <= '0; fir_valid <= 1'b0; rstb <= 1'b0;
        repeat (5) @(posedge clk);
        rstb <= 1'b1;
        @(posedge clk);

        // Two conversions: p=0 (small chirp, near cos(0)=1) then p=1.
        for (int p = 0; p < 2; p++) begin
            real phase, rad;
            int i_val, q_val;
            phase = 2.0*3.14159265*0.001*(p+1);
            i_val = $rtoi($cos(phase) * 200.0);
            q_val = $rtoi($sin(phase) * 200.0);
            for (int c = 0; c < R; c++) begin
                data_in_i <= i_val[15:0];
                data_in_q <= q_val[15:0];
                fir_valid <= (c == 0);
                @(posedge clk);
                $display("t=%0t p=%0d c=%0d state=%0d mac_step=%0d tap_idx=%0d fetch=%0b/%0b/%0b mult=%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d partial_sum=%0d accum_i=%0d accum_q=%0d I_out=%0d Q_out=%0d done=%0b",
                    $time, p, c, u_fir.state, u_fir.mac_step, u_fir.tap_idx,
                    u_fir.fetch, u_fir.fetch_d1, u_fir.fetch_d2,
                    u_fir.mult[0], u_fir.mult[1], u_fir.mult[2], u_fir.mult[3],
                    u_fir.mult[4], u_fir.mult[5], u_fir.mult[6], u_fir.mult[7],
                    u_fir.partial_sum, u_fir.accum_i, u_fir.accum_q,
                    I_out_fir, Q_out_fir, fir_done);
            end
        end
        $finish;
    end
endmodule
