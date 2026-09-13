`timescale 1ns / 1ps
//
// Self-checking testbench for cic_dec (src/cic_dec.sv): a 3-stage
// integrator/comb CIC decimator, decimate-by-25, 12-bit signed IQ in,
// 16-bit signed IQ out (dec_i/dec_q = internal 26-bit dec3_i/dec3_q[25:10],
// i.e. an arithmetic-right-shift-by-10 truncation, NOT a full R^N=15625
// gain-normalizing shift -- see the DC steady-state check below, which
// checks the DUT abides by exactly this convention).
//
// Verification strategy: rather than mirror cic_dec.sv's own structure
// (which would just prove the DUT matches a copy of itself, not that
// either is actually a correct CIC), this drives an INDEPENDENT reference
// model -- ref_model registers below -- built the same conceptual way
// (integrate, then comb, then decimate) but:
//   - written fresh, not copy-pasted from the DUT
//   - 64-bit (longint) non-wrapping arithmetic throughout, instead of the
//     DUT's fixed 26-bit wraparound registers, so a real bit-width/
//     overflow bug in the DUT would show up as a mismatch here rather
//     than being silently reproduced
//   - the final truncation is parameterized (ACC_SHIFT) rather than a
//     hardcoded bit-select, so a wrong shift amount is also caught
// and cross-checked a second, completely independent way against the
// closed-form fact that a CIC(N,R) filter's steady-state DC gain is
// exactly R^N (a basic property of the filter, unrelated to how either
// implementation computes it) -- see run_dc_test().
//
// Coverage:
//   - Reset state (dec_i/dec_q == 0, matching the DUT's explicit reset).
//   - Strobe period: exactly R=25 clk cycles between strb pulses, over
//     several periods, checked purely from the black-box strb output.
//   - Zero input -> zero output, for several decimation periods.
//   - DC/step response: several constant inputs (including the extreme
//     +2047/-2048 codes), each from a clean reset, checked against the
//     closed-form steady-state code dc*R^3>>>ACC_SHIFT once the ~3-strobe
//     group delay has passed, and checked to be genuinely constant (not
//     still drifting) across several consecutive strobes.
//   - A single deterministic impulse (I and Q different, non-symmetric
//     values so a channel swap would be caught), full transient traced
//     against the live reference model -- also human-eyeballable in the
//     log for the characteristic CIC impulse-response shape.
//   - 300 decimation periods (7500 clk cycles) of randomized 12-bit IQ,
//     checked against the live reference model at every decimated output.
//
// Current status (2026-09-13, run against cic_dec.sv as of this date):
// 851 pass, 25 FAIL -- and the failures are real, not testbench noise.
// Two confirmed bugs in cic_dec.sv, not fixed here (their file, not
// this testbench's job):
//   1. Incomplete reset: dec3_i/dec3_q/accum3_i_last/accum3_q_last/
//      dec1_i_last/dec1_q_last/dec2_i_last/dec2_q_last are never reset
//      (only accum1/2/3, dec1, dec2, dec_i, dec_q, strb_cnt are).
//   2. dec_i/dec_q are driven by BOTH a continuous assign (dec3_i[25-:16])
//      AND a procedural reset assignment (dec_i<='0) -- a genuine
//      multi-driver conflict (XSIM warns "might have multiple concurrent
//      drivers" at elaboration), which makes the procedural reset dead:
//      it's immediately overridden by the continuous assign the instant
//      dec3_i holds anything.
// Together these mean the first ~5 decimated outputs after ANY reset
// are garbage (confirmed cleanly: a zero-input test produces nonzero
// output at strobes #1-5, which has no legitimate settling-time excuse
// -- integrating zero forever must stay zero). All 25 current failures
// are confined to strobes #1-5 of the zero-input/impulse/random phases,
// which start fresh from reset; DC-test SKIP_STROBES=7 already steers
// clear of this same window, and once past it the DUT matches both the
// live reference model and the closed-form DC-gain formula exactly,
// every time -- i.e. the actual CIC math is correct, only the reset is
// incomplete. Expect this failure count to go to 0 once cic_dec.sv's
// reset list is completed and the dec_i/dec_q driver conflict resolved
// (e.g. make dec_i/dec_q plain continuous assigns, drop the procedural
// reset of them entirely, since dec3_i itself being properly reset
// would make it redundant anyway).
//
// Known open items, not this testbench's job to fix:
//   - cic_dec.sv has no input valid/enable port -- this testbench always
//     drives a fresh sample every single clk cycle, matching the DUT's
//     actual interface as given; it does not check behavior if the DUT
//     is ever changed to add one.
//   - The DUT's output convention (R^N gain, only >>>10, not the full
//     >>>14 that would give a ~unity-gain 16-bit output) is checked as
//     given, not flagged as right or wrong -- see the module-level
//     comment above. If that convention changes, ACC_SHIFT below must
//     change to match.

module tb_cic_dec;

    // ------------------------------------------------------------------
    // Parameters -- mirror cic_dec.sv's fixed (non-parameterized) values.
    // ------------------------------------------------------------------
    localparam real CLK_PERIOD_NS = 33.56; // matches this project's ~29.8MHz dsp_clk convention; value doesn't affect correctness here
    localparam int  IN_WIDTH   = 12;
    localparam int  OUT_WIDTH  = 16;
    localparam int  R          = 25;  // decimation ratio (strb_cnt counts 0..24)
    localparam int  ACC_SHIFT  = 10;  // matches dec_i = dec3_i[25 -:16]
    localparam longint DC_GAIN = 25 ** 3; // R^N, N=3 stages -- CIC's closed-form DC gain

    localparam logic signed [IN_WIDTH-1:0] IN_MAX = (1 << (IN_WIDTH-1)) - 1; //  2047
    localparam logic signed [IN_WIDTH-1:0] IN_MIN = -(1 << (IN_WIDTH-1));    // -2048

    localparam int RANDOM_PERIODS = 300;
    localparam int RANDOM_SEED    = 32'hCAFE_1234;

    // ------------------------------------------------------------------
    // Scoreboard
    // ------------------------------------------------------------------
    int unsigned pass_count = 0;
    int unsigned fail_count = 0;

    task automatic check_signed(input string name, input longint signed actual, input longint signed expected);
        if (actual === expected) begin
            pass_count++;
            $display("[PASS] t=%0t %-58s got=%0d", $time, name, actual);
        end else begin
            fail_count++;
            $display("[FAIL] t=%0t %-58s got=%0d expected=%0d", $time, name, actual, expected);
        end
    endtask

    // ------------------------------------------------------------------
    // Clock
    // ------------------------------------------------------------------
    logic clk;
    initial clk = 0;
    always #(CLK_PERIOD_NS/2.0) clk = ~clk;

    // ------------------------------------------------------------------
    // DUT
    // ------------------------------------------------------------------
    logic                        rstb;
    logic signed [IN_WIDTH-1:0]  rx_i, rx_q;
    logic signed [OUT_WIDTH-1:0] dec_i, dec_q;
    logic                        strb;

    cic_dec dut (
        .clk   (clk),
        .rstb  (rstb),
        .rx_i  (rx_i),
        .rx_q  (rx_q),
        .dec_i (dec_i),
        .dec_q (dec_q),
        .strb  (strb)
    );

    // ------------------------------------------------------------------
    // Independent reference model -- see header comment. Same rx_i/rx_q/
    // clk/rstb as the DUT, structured as integrate-every-cycle / comb-on-
    // strobe / decimate-by-R, but wide non-wrapping arithmetic and a
    // parameterized final shift instead of a hardcoded bit-select.
    // ------------------------------------------------------------------
    longint signed ref_acc1_i, ref_acc1_q;
    longint signed ref_acc2_i, ref_acc2_q;
    longint signed ref_acc3_i, ref_acc3_q, ref_acc3_i_last, ref_acc3_q_last;
    longint signed ref_c1_i, ref_c1_q, ref_c1_i_last, ref_c1_q_last;
    longint signed ref_c2_i, ref_c2_q, ref_c2_i_last, ref_c2_q_last;
    longint signed ref_c3_i, ref_c3_q;
    logic [4:0] ref_strb_cnt;
    logic       ref_strb;

    assign ref_strb = (ref_strb_cnt == 0);

    always_ff @(posedge clk) begin
        if (!rstb) begin
            ref_acc1_i <= 0; ref_acc1_q <= 0;
            ref_acc2_i <= 0; ref_acc2_q <= 0;
            ref_acc3_i <= 0; ref_acc3_q <= 0;
            ref_acc3_i_last <= 0; ref_acc3_q_last <= 0;
            ref_c1_i <= 0; ref_c1_q <= 0; ref_c1_i_last <= 0; ref_c1_q_last <= 0;
            ref_c2_i <= 0; ref_c2_q <= 0; ref_c2_i_last <= 0; ref_c2_q_last <= 0;
            ref_c3_i <= 0; ref_c3_q <= 0;
            ref_strb_cnt <= '0;
        end else begin
            ref_strb_cnt <= (ref_strb_cnt == R-1) ? '0 : ref_strb_cnt + 1;

            if (ref_strb) begin
                ref_c1_i <= ref_acc3_i - ref_acc3_i_last;
                ref_acc3_i_last <= ref_acc3_i;
                ref_c1_q <= ref_acc3_q - ref_acc3_q_last;
                ref_acc3_q_last <= ref_acc3_q;

                ref_c2_i <= ref_c1_i - ref_c1_i_last;
                ref_c1_i_last <= ref_c1_i;
                ref_c2_q <= ref_c1_q - ref_c1_q_last;
                ref_c1_q_last <= ref_c1_q;

                ref_c3_i <= ref_c2_i - ref_c2_i_last;
                ref_c2_i_last <= ref_c2_i;
                ref_c3_q <= ref_c2_q - ref_c2_q_last;
                ref_c2_q_last <= ref_c2_q;
            end

            ref_acc1_i <= ref_acc1_i + longint'(rx_i);
            ref_acc1_q <= ref_acc1_q + longint'(rx_q);
            ref_acc2_i <= ref_acc2_i + ref_acc1_i;
            ref_acc2_q <= ref_acc2_q + ref_acc1_q;
            ref_acc3_i <= ref_acc3_i + ref_acc2_i;
            ref_acc3_q <= ref_acc3_q + ref_acc2_q;
        end
    end

    // Continuous, combinational -- mirrors the DUT's own
    // "assign dec_i = dec3_i[25 -:16]" timing exactly (no extra register
    // stage), so ref_dec_i/ref_dec_q become valid on the exact same cycle
    // as the DUT's dec_i/dec_q.
    function automatic logic signed [OUT_WIDTH-1:0] shift_trunc(input longint signed wide);
        longint signed shifted;
        shifted = wide >>> ACC_SHIFT;
        return shifted[OUT_WIDTH-1:0];
    endfunction

    logic signed [OUT_WIDTH-1:0] ref_dec_i, ref_dec_q;
    assign ref_dec_i = shift_trunc(ref_c3_i);
    assign ref_dec_q = shift_trunc(ref_c3_q);

    // One-cycle-delayed strobe: dec3_i (and ref_c3_i) update ON the edge
    // where strb is sampled high, so the freshly-computed value is only
    // visible starting the cycle AFTER that edge. Used purely to time the
    // per-decimated-output check()/logging below -- correctness doesn't
    // depend on it (dec_i/ref_dec_i hold steady and agree continuously
    // regardless), it just keeps the log to one line per real decimated
    // sample instead of one per clk cycle.
    logic strb_d1;
    always_ff @(posedge clk) strb_d1 <= strb;

    // ------------------------------------------------------------------
    // Reset (synchronous in the DUT -- "if(~rstb)" inside the clocked
    // block -- so rstb must be low across a real posedge, not just
    // glitched between edges).
    // ------------------------------------------------------------------
    task automatic do_reset;
        rx_i <= '0;
        rx_q <= '0;
        rstb <= 1'b0;
        repeat (5) @(posedge clk);
        rstb <= 1'b1;
        @(posedge clk);
    endtask

    // ------------------------------------------------------------------
    // Strobe period: drives arbitrary content (irrelevant to this check)
    // while measuring the clk-cycle gap between `num_periods` consecutive
    // strb pulses -- independently validates the decimation ratio R
    // purely from the black-box strb output.
    // ------------------------------------------------------------------
    task automatic check_strobe_period(input int num_periods);
        int gap;
        integer seed;
        seed = 32'hBEEF_0001;
        do begin
            rx_i <= $random(seed);
            rx_q <= '0;
            @(posedge clk);
        end while (!strb);
        for (int p = 0; p < num_periods; p++) begin
            gap = 0;
            do begin
                rx_i <= $random(seed);
                rx_q <= '0;
                @(posedge clk);
                gap++;
            end while (!strb);
            check_signed($sformatf("Strobe period #%0d (clk cycles between strb pulses)", p), gap, R);
        end
    endtask

    // ------------------------------------------------------------------
    // DC/step test: clean reset, drive a constant for long enough to
    // clear the ~3-strobe group delay, then check several consecutive
    // decimated outputs are (a) all identical (genuinely settled, not
    // still drifting) and (b) equal to the closed-form steady-state code
    // dc*R^3 >>> ACC_SHIFT -- computed independently of both the DUT and
    // the reference model, straight from the CIC DC-gain identity.
    // ------------------------------------------------------------------
    task automatic run_dc_test(input string tag, input logic signed [IN_WIDTH-1:0] dc_val);
        localparam int SKIP_STROBES  = 7; // empirically >=6 needed (see tb run notes); N=3 group delay alone (3) is not enough margin
        localparam int CHECK_STROBES = 8;
        localparam int TOTAL_CYCLES  = (SKIP_STROBES + CHECK_STROBES + 2) * R;
        logic signed [OUT_WIDTH-1:0] expected, first_seen;
        int strobes_seen;

        do_reset();
        expected = shift_trunc(longint'(dc_val) * DC_GAIN);
        strobes_seen = 0;

        for (int c = 0; c < TOTAL_CYCLES; c++) begin
            rx_i <= dc_val;
            rx_q <= dc_val;
            @(posedge clk);
            if (strb_d1) begin
                strobes_seen++;
                if (strobes_seen == SKIP_STROBES + 1) first_seen = dec_i;
                if (strobes_seen > SKIP_STROBES && strobes_seen <= SKIP_STROBES + CHECK_STROBES) begin
                    check_signed($sformatf("%s: I steady-state strobe #%0d == closed-form dc*R^3>>>%0d", tag, strobes_seen, ACC_SHIFT),
                                 dec_i, expected);
                    check_signed($sformatf("%s: Q steady-state strobe #%0d == closed-form dc*R^3>>>%0d", tag, strobes_seen, ACC_SHIFT),
                                 dec_q, expected);
                    check_signed($sformatf("%s: I strobe #%0d matches first settled sample (genuinely constant)", tag, strobes_seen),
                                 dec_i, first_seen);
                    check_signed($sformatf("%s: I strobe #%0d DUT matches live ref_model too", tag, strobes_seen),
                                 dec_i, ref_dec_i);
                    check_signed($sformatf("%s: ref_model strobe #%0d == closed-form (calibration check)", tag, strobes_seen),
                                 ref_dec_i, expected);
                end
            end
        end
    endtask

    // ------------------------------------------------------------------
    // Main test sequence
    // ------------------------------------------------------------------
    initial begin
        $display("=== tb_cic_dec starting ===");

        // ---- Reset state ----
        do_reset();
        check_signed("Reset: dec_i == 0", dec_i, 0);
        check_signed("Reset: dec_q == 0", dec_q, 0);

        // ---- Strobe period ----
        $display("=== Strobe period (expect exactly R=%0d clk cycles) ===", R);
        check_strobe_period(10);

        // ---- Zero input -> zero output ----
        $display("=== Zero input settles to zero output ===");
        do_reset();
        begin
            int strobes;
            strobes = 0;
            while (strobes < 6) begin
                rx_i <= '0;
                rx_q <= '0;
                @(posedge clk);
                if (strb_d1) begin
                    strobes++;
                    check_signed($sformatf("Zero-input strobe #%0d: I == 0", strobes), dec_i, 0);
                    check_signed($sformatf("Zero-input strobe #%0d: Q == 0", strobes), dec_q, 0);
                end
            end
        end

        // ---- DC / step response, including extreme codes ----
        $display("=== DC steady-state response ===");
        run_dc_test("DC=+1",             1);
        run_dc_test("DC=-1",            -1);
        run_dc_test("DC=+1000",          1000);
        run_dc_test("DC=-1000",         -1000);
        run_dc_test("DC=+2047 (IN_MAX)", IN_MAX);
        run_dc_test("DC=-2048 (IN_MIN)", IN_MIN);

        // ---- Single impulse, full transient traced against the live
        //      reference model (also human-eyeballable in the log) ----
        $display("=== Impulse response (I != Q, checked against live ref_model) ===");
        do_reset();
        begin
            localparam int IMPULSE_TRACE_CYCLES = 6 * R;
            int strobes;
            strobes = 0;
            for (int c = 0; c < IMPULSE_TRACE_CYCLES; c++) begin
                if (c == 0) begin
                    rx_i <= 777;
                    rx_q <= -333;
                end else begin
                    rx_i <= '0;
                    rx_q <= '0;
                end
                @(posedge clk);
                if (strb_d1) begin
                    strobes++;
                    check_signed($sformatf("Impulse strobe #%0d: I matches live ref_model", strobes), dec_i, ref_dec_i);
                    check_signed($sformatf("Impulse strobe #%0d: Q matches live ref_model", strobes), dec_q, ref_dec_q);
                    $display("[INFO] t=%0t Impulse strobe #%0d: dec_i=%0d dec_q=%0d", $time, strobes, dec_i, dec_q);
                end
            end
        end

        // ---- Randomized bulk comparison against the live reference
        //      model, every decimated output, for RANDOM_PERIODS strobes
        //      (RANDOM_PERIODS*R clk cycles). ----
        $display("=== Randomized IQ vs. live reference model: %0d decimation periods ===", RANDOM_PERIODS);
        do_reset();
        begin
            int strobes;
            integer seed;
            seed = RANDOM_SEED;
            strobes = 0;
            while (strobes < RANDOM_PERIODS) begin
                rx_i <= $random(seed);
                rx_q <= $random(seed);
                @(posedge clk);
                if (strb_d1) begin
                    strobes++;
                    check_signed($sformatf("Random strobe #%0d: I matches live ref_model", strobes), dec_i, ref_dec_i);
                    check_signed($sformatf("Random strobe #%0d: Q matches live ref_model", strobes), dec_q, ref_dec_q);
                end
            end
        end

        // ------------------------------------------------------------------
        $display("=== tb_cic_dec finished: %0d passed, %0d failed ===", pass_count, fail_count);
        if (fail_count == 0)
            $display("=== RESULT: PASS ===");
        else
            $display("=== RESULT: FAIL ===");

        $finish;
    end

endmodule
