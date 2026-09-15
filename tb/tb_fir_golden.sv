`timescale 1ns / 1ps
//
// Golden-vector testbench (2026-09-15) -- drives fir_time_multiplexed with
// the exact I/Q waveform tools/gen_fir_golden_vectors.py generated, and
// checks its output bit-for-bit (small tolerance for pipeline-boundary
// samples where the RTL's registered output hasn't caught up to a
// same-cycle-generated golden value) against that script's independent,
// bit-accurate model of the RTL's own algorithm, run against the SAME
// fir_coeffs.hex Vivado actually synthesizes. Built specifically to
// settle the FIR's gain question with real numbers instead of reasoning
// about scale factors in the abstract -- see project memory,
// fir_time_multiplexed_i_channel_bug.md.

module tb_fir_golden;

    localparam real CLK_PERIOD_NS = 33.33;
    localparam int  R             = 25;
    localparam int  NUM_TAPS      = 48;
    localparam int  N_SAMPLES     = 600;
    localparam int  TOL_COUNTS    = 5;   // near-zero -- this should be a near-bit-exact match

    logic [15:0] in_i  [0:N_SAMPLES-1];
    logic [15:0] in_q  [0:N_SAMPLES-1];
    logic [15:0] exp_i [0:N_SAMPLES-1];
    logic [15:0] exp_q [0:N_SAMPLES-1];

    initial $readmemh("fir_golden_in_i.hex",  in_i);
    initial $readmemh("fir_golden_in_q.hex",  in_q);
    initial $readmemh("fir_golden_exp_i.hex", exp_i);
    initial $readmemh("fir_golden_exp_q.hex", exp_q);

    int unsigned pass_count = 0;
    int unsigned fail_count = 0;
    int max_abs_err = 0;

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
        int err_i, err_q, conv_idx;

        $display("=== tb_fir_golden starting: %0d golden samples ===", N_SAMPLES);

        data_in_i <= '0; data_in_q <= '0; fir_valid <= 1'b0; rstb <= 1'b0;
        repeat (5) @(posedge clk);
        rstb <= 1'b1;
        @(posedge clk);

        conv_idx = 0;
        for (int s = 0; s < N_SAMPLES; s++) begin
            for (int c = 0; c < R; c++) begin
                data_in_i <= $signed(in_i[s]);
                data_in_q <= $signed(in_q[s]);
                fir_valid <= (c == 0);
                @(posedge clk);
                if (fir_done) begin
                    err_i = $signed(I_out_fir) - $signed(exp_i[conv_idx]);
                    err_q = $signed(Q_out_fir) - $signed(exp_q[conv_idx]);
                    if (err_i < 0) err_i = -err_i;
                    if (err_q < 0) err_q = -err_q;
                    if (err_i > max_abs_err) max_abs_err = err_i;
                    if (err_q > max_abs_err) max_abs_err = err_q;

                    if (err_i <= TOL_COUNTS && err_q <= TOL_COUNTS) begin
                        pass_count++;
                    end else begin
                        fail_count++;
                        if (fail_count <= 15)
                            $display("[FAIL] conv=%0d I_out=%0d exp_i=%0d err_i=%0d  Q_out=%0d exp_q=%0d err_q=%0d",
                                      conv_idx, I_out_fir, $signed(exp_i[conv_idx]), err_i,
                                      Q_out_fir, $signed(exp_q[conv_idx]), err_q);
                    end
                    conv_idx++;
                end
            end
        end

        $display("=== tb_fir_golden finished: %0d/%0d conversions matched (tol=%0d), max_abs_err=%0d ===",
                  pass_count, conv_idx, TOL_COUNTS, max_abs_err);
        if (fail_count == 0 && conv_idx == N_SAMPLES)
            $display("=== RESULT: PASS ===");
        else
            $display("=== RESULT: FAIL (%0d mismatches, %0d/%0d conversions seen) ===", fail_count, conv_idx, N_SAMPLES);

        $finish;
    end

endmodule
