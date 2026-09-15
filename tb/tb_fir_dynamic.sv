`timescale 1ns / 1ps
//
// Dynamic-input testbench for fir_time_multiplexed (2026-09-15) -- built
// specifically to close a real gap flagged by the user: every prior FIR
// testbench (this project's original one, and the one used to verify the
// OUT_SHIFT 7->9 overflow fix) drives a CONSTANT, held input for many
// cycles. Real antenna data changes every sample. A bug specific to
// continuously-varying input (e.g. in the shared-datapath drain/pipeline
// timing, which was verified correct by hand cycle-by-cycle bookkeeping
// but never stress-tested in simulation against non-constant data) would
// be invisible to every test run so far.
//
// Reference model: an independent real-arithmetic direct-form FIR
// convolution (NOT a copy of the RTL's own per-tap-shift-then-sum
// structure), maintaining its own sample history and computing
// y = sum(taps[k] * x_hist[k]) / 2^15 each conversion -- matches the RTL
// to within a few counts of quantization noise for a correct
// implementation (confirmed against the earlier settled-DC case, see
// project memory fir_time_multiplexed_i_channel_bug.md), and would be
// off by thousands of counts (a 16-bit wrap is 65536) if the RTL
// actually drops/duplicates/corrupts a conversion under dynamic input.
//
// Drives a chirp (linearly swept frequency) I/Q phasor, changing every
// valid pulse, at three amplitude tiers, and reports: max tracking
// error against the reference, and how often the RTL output lands
// exactly on the 16-bit rails (+32767/-32768) -- a real overflow/
// pipeline bug would show up as either a large tracking error or
// suspiciously frequent rail-hits; correct behavior at a given
// amplitude tier should show neither.

module tb_fir_dynamic;

    localparam real CLK_PERIOD_NS = 33.33;
    localparam int  R             = 25;    // matches cic_dec's real decimation cadence
    localparam int  NUM_TAPS      = 48;
    localparam int  N_PULSES      = 400;   // valid pulses per amplitude tier
    localparam real PI            = 3.14159265358979323846;
    localparam int  TOL_COUNTS    = 300;   // generous -- catching real bugs (1000s of counts), not micro-tuning quantization

    int unsigned pass_count = 0;
    int unsigned fail_count = 0;
    int unsigned rail_hits  = 0;
    int unsigned total_checks = 0;

    logic [13:0] taps_raw [0:NUM_TAPS-1];  // unused placeholder, real taps read below into `taps`
    logic signed [15:0] taps [0:NUM_TAPS-1];
    initial $readmemh("fir_coeffs.hex", taps);

    logic clk;
    initial clk = 0;
    always #(CLK_PERIOD_NS/2.0) clk = ~clk;

    logic               rstb;
    logic signed [15:0] data_in_i, data_in_q;
    logic               fir_valid;
    logic signed [15:0] I_out_fir, Q_out_fir;
    logic               fir_done;

    fir_time_multiplexed #(.NUM_TAPS(NUM_TAPS)) u_fir (
        .clk        (clk),
        .rstb       (rstb),
        .data_in_i  (data_in_i),
        .data_in_q  (data_in_q),
        .valid      (fir_valid),
        .data_out_i (I_out_fir),
        .data_out_q (Q_out_fir),
        .done       (fir_done)
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

    // Independent reference model: real-arithmetic direct-form convolution.
    real i_hist [0:NUM_TAPS-1];
    real q_hist [0:NUM_TAPS-1];

    function automatic real ref_convolve(real hist [0:NUM_TAPS-1]);
        real acc;
        acc = 0.0;
        for (int k = 0; k < NUM_TAPS; k++) begin
            acc += real'($signed(taps[k])) * hist[k];
        end
        return acc / 131072.0;  // taps are Q15-ish fixed point (2^15), plus the RTL's own PROD_SHIFT(8)+OUT_SHIFT(9)-15=2 extra bits of scaling -- 2^(15+2)=2^17
    endfunction

    task automatic push_hist(ref real hist [0:NUM_TAPS-1], input real x);
        for (int k = NUM_TAPS-1; k > 0; k--) hist[k] = hist[k-1];
        hist[0] = x;
    endtask

    task automatic run_chirp(input string tag, input real amplitude);
        real freq_start, freq_end, freq, phase, rad;
        int  i_val, q_val;
        real ref_i, ref_q;
        real err_i, err_q;

        freq_start = 0.001;  // cycles per valid pulse
        freq_end   = 0.45;

        phase = 0.0;

        for (int p = 0; p < N_PULSES; p++) begin
            freq = freq_start + (freq_end - freq_start) * (real'(p) / real'(N_PULSES));
            phase += 2.0 * PI * freq;
            i_val = $rtoi($cos(phase) * amplitude);
            q_val = $rtoi($sin(phase) * amplitude);

            push_hist(i_hist, real'(i_val));
            push_hist(q_hist, real'(q_val));
            ref_i = ref_convolve(i_hist);
            ref_q = ref_convolve(q_hist);

            for (int c = 0; c < R; c++) begin
                data_in_i <= i_val[15:0];
                data_in_q <= q_val[15:0];
                fir_valid <= (c == 0);
                @(posedge clk);
                if (fir_done) begin
                    total_checks++;
                    err_i = real'(I_out_fir) - ref_i;
                    err_q = real'(Q_out_fir) - ref_q;
                    if (err_i < 0) err_i = -err_i;
                    if (err_q < 0) err_q = -err_q;

                    if (I_out_fir == 16'sh7FFF || I_out_fir == 16'sh8000) rail_hits++;
                    if (Q_out_fir == 16'sh7FFF || Q_out_fir == 16'sh8000) rail_hits++;

                    if (err_i <= TOL_COUNTS && err_q <= TOL_COUNTS) begin
                        pass_count++;
                    end else begin
                        fail_count++;
                        if (fail_count <= 10)
                            $display("[FAIL] t=%0t %s p=%0d I_out_fir=%0d ref_i=%.1f err_i=%.1f  Q_out_fir=%0d ref_q=%.1f err_q=%.1f",
                                      $time, tag, p, I_out_fir, ref_i, err_i, Q_out_fir, ref_q, err_q);
                    end
                end
            end
        end

        $display("[INFO] %s: %0d checks this tier, rail_hits_so_far=%0d", tag, total_checks, rail_hits);
    endtask

    initial begin
        $display("=== tb_fir_dynamic starting ===");
        do_reset();
        for (int k = 0; k < NUM_TAPS; k++) begin
            i_hist[k] = 0.0;
            q_hist[k] = 0.0;
        end

        $display("=== Tier 1: small amplitude (200), chirp sweep ===");
        run_chirp("small_200", 200.0);

        $display("=== Tier 2: medium amplitude (2000), chirp sweep ===");
        run_chirp("medium_2000", 2000.0);

        $display("=== Tier 3: large amplitude (12000), chirp sweep ===");
        run_chirp("large_12000", 12000.0);

        $display("=== tb_fir_dynamic finished: %0d passed, %0d failed, %0d checks total, %0d rail hits (%.2f%%) ===",
                  pass_count, fail_count, total_checks, rail_hits, 100.0*real'(rail_hits)/real'(total_checks*2));
        if (fail_count == 0)
            $display("=== RESULT: PASS ===");
        else
            $display("=== RESULT: FAIL ===");

        $finish;
    end

endmodule
