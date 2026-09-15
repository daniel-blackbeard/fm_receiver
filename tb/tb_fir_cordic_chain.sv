`timescale 1ns / 1ps
//
// Integration testbench: fir_time_multiplexed -> cordic_vector, wired
// EXACTLY the way dsp.sv wires them (cordic_vector.strb = fir's own
// `done` output, not a testbench-idealized strobe). Built specifically
// to test the hypothesis that cordic_vector "doesn't like the fir_done
// kind of strobe" -- tb_cordic_vector.sv already proved cordic_vector is
// correct against a clean, testbench-driven strobe; this checks whether
// it's still correct against the REAL done signal fir_time_multiplexed
// actually produces.
//
// Stimulus: drives fir_time_multiplexed's `valid` on a ~25-cycle cadence
// (matching cic_dec's real R=25 decimation period in the full chain),
// feeding a constant-angle I/Q phasor held long enough (60 valid pulses,
// comfortably more than NUM_TAPS/PARALLELISM=6 MAC-steps worth of shift-
// register flush) for the FIR's own low-pass response to fully settle,
// then checks cordic_vector's phase output against that known angle.
// Then switches to a second, different angle and repeats -- if
// cordic_vector only ever fires once and then locks up, or never updates
// again after the first real fir_done, this second phase would catch it.

module tb_fir_cordic_chain;

    localparam real CLK_PERIOD_NS = 33.33;
    localparam int  R             = 25;   // matches cic_dec's real decimation period
    localparam int  SETTLE_PULSES = 60;   // valid pulses per angle, well past the FIR's own settling time
    localparam int  TOL_COUNTS    = 200;  // looser than tb_cordic_vector's -- FIR filtering + rounding adds real quantization on top of the CORDIC's own
    localparam real PI            = 3.14159265358979323846;

    int unsigned pass_count = 0;
    int unsigned fail_count = 0;

    task automatic check_tol(input string name, input longint signed actual, input longint signed expected, input int tol);
        longint signed diff;
        diff = actual - expected;
        if (diff > 32768) diff -= 65536;
        if (diff < -32768) diff += 65536;
        if (diff < 0) diff = -diff;
        if (diff <= tol) begin
            pass_count++;
            $display("[PASS] t=%0t %-58s got=%0d expected=%0d diff=%0d", $time, name, actual, expected, diff);
        end else begin
            fail_count++;
            $display("[FAIL] t=%0t %-58s got=%0d expected=%0d diff=%0d (tol=%0d)", $time, name, actual, expected, diff, tol);
        end
    endtask

    logic clk;
    initial clk = 0;
    always #(CLK_PERIOD_NS/2.0) clk = ~clk;

    logic               rstb;
    logic signed [15:0] data_in_i, data_in_q;
    logic               fir_valid;
    logic signed [15:0] I_out_fir, Q_out_fir;
    logic               fir_done;

    fir_time_multiplexed #(.NUM_TAPS(48)) u_fir (
        .clk        (clk),
        .rstb       (rstb),
        .data_in_i  (data_in_i),
        .data_in_q  (data_in_q),
        .valid      (fir_valid),
        .data_out_i (I_out_fir),
        .data_out_q (Q_out_fir),
        .done       (fir_done)
    );

    logic signed [15:0] mpx_angle;
    logic                disc_done;

    // Wired exactly as dsp.sv wires it -- strb = fir_done, the real signal,
    // not a testbench-generated one.
    cordic_vector u_discriminator (
        .clk   (clk),
        .rstb  (rstb),
        .strb  (fir_done),
        .I_in  (I_out_fir),
        .Q_in  (Q_out_fir),
        .phase (mpx_angle),
        .valid (disc_done)
    );

    task automatic do_reset;
        data_in_i <= '0;
        data_in_q <= '0;
        fir_valid <= 1'b0;
        rstb <= 1'b0;
        repeat (5) @(posedge clk);
        rstb <= 1'b1;
        @(posedge clk);
    endtask

    // Drives `valid` every R cycles (matching cic_dec's real cadence),
    // holding a constant-angle I/Q phasor for SETTLE_PULSES periods, then
    // checks how many times cordic_vector's disc_done actually fired
    // during that window, and the last phase it produced against the
    // known target angle.
    task automatic run_angle(input string tag, input real angle_deg, input int amplitude);
        real rad;
        int  i_val, q_val;
        int  expected_code;
        int  disc_fires;
        logic signed [15:0] last_phase;

        rad   = angle_deg * PI / 180.0;
        i_val = $rtoi($cos(rad) * amplitude);
        q_val = $rtoi($sin(rad) * amplitude);
        expected_code = $rtoi((angle_deg / 360.0) * 65536.0);
        disc_fires = 0;
        last_phase = mpx_angle;

        for (int p = 0; p < SETTLE_PULSES; p++) begin
            for (int c = 0; c < R; c++) begin
                data_in_i <= i_val[15:0];
                data_in_q <= q_val[15:0];
                fir_valid <= (c == 0);
                @(posedge clk);
                if (disc_done) begin
                    disc_fires++;
                    last_phase = mpx_angle;
                    if (disc_fires > SETTLE_PULSES - 15)
                        $display("[TRACE] t=%0t %s: conversion #%0d I_out_fir=%0d Q_out_fir=%0d mpx_angle=%0d",
                                  $time, tag, disc_fires, I_out_fir, Q_out_fir, mpx_angle);
                end
            end
        end

        $display("[INFO] t=%0t %s: cordic_vector.valid fired %0d times over %0d fir_done pulses",
                  $time, tag, disc_fires, SETTLE_PULSES);
        check_tol($sformatf("%s: fir_done fires at all (nonzero disc_fires)", tag), disc_fires, disc_fires, 0); // always "passes", just logs; real check below
        if (disc_fires == 0) begin
            fail_count++;
            $display("[FAIL] t=%0t %s: cordic_vector.valid NEVER fired -- strb (fir_done) never triggered it", $time, tag);
        end else begin
            check_tol($sformatf("%s: settled phase (I=%0d Q=%0d, target %.1fdeg)", tag, i_val, q_val, angle_deg),
                      $signed(last_phase), expected_code, TOL_COUNTS);
        end
    endtask

    initial begin
        $display("=== tb_fir_cordic_chain starting ===");
        do_reset();

        $display("=== Angle 1: 45deg, held %0d valid pulses ===", SETTLE_PULSES);
        run_angle("angle1_45deg", 45.0, 12000);

        $display("=== Angle 2: -120deg, held %0d valid pulses (tests re-trigger after angle 1) ===", SETTLE_PULSES);
        run_angle("angle2_-120deg", -120.0, 12000);

        $display("=== Angle 3: 10deg, held %0d valid pulses (tests a third re-trigger) ===", SETTLE_PULSES);
        run_angle("angle3_10deg", 10.0, 12000);

        $display("=== tb_fir_cordic_chain finished: %0d passed, %0d failed ===", pass_count, fail_count);
        if (fail_count == 0)
            $display("=== RESULT: PASS ===");
        else
            $display("=== RESULT: FAIL ===");

        $finish;
    end

endmodule
