`timescale 1ns / 1ps
//
// tb_rds_golden.sv -- one-pass golden-reference testbench for src/rds.sv.
//
// Feeds rds.sv the synthetic 240 kHz stream tools/gen_rds_golden.py produced
// (exactly what dsp.sv hands it: mixed-down MPX, 16-bit, one single-cycle
// `strb` per sample, dsp_clk = 30 MHz / STRB_DIV=125 -> 240 kHz) and checks
// EVERY stage of the RDS chain against that script's models, all in the same
// run. Stages not built yet simply FAIL (or don't compile until their port
// exists) -- work the module until the summary table is all PASS.
//
// Regenerate vectors:   C:\msys64\ucrt64\bin\python.exe tools\gen_rds_golden.py
// Run:                  scripts\run_sim_rds.bat
// Faster early runs:    xsim ... -testplusarg STRB_DIV=16   (pre-CIC-rate pipelines only!)
//
// ---- PORT CONTRACT (rds.sv must provide exactly these) ---------------------
//  in : clk, rstb, i_mpx_data[15:0] signed, strb
//
//  S0 CIC        o_rds_data[15:0] s   valid          one item per decimation event (exact)
//  S1 low-pass   o_lpf_data[15:0]  s  o_lpf_valid    one per S0 item, |err| <= TOL_S1
//  S2 NCO        o_nco_phase[31:0] u  o_nco_valid    one per S1 item; phase word paired with
//                                                    sample k = NCO_PHASE0 + k*NCO_STEP (mod 2^32),
//                                                    i.e. the value BEFORE that sample's increment (exact)
//  S3 biphase    o_soft[23:0] s, o_biphase  o_soft_valid
//                                                    one per completed symbol. soft = sum(S1 samples with
//                                                    phase[31]==0) - sum(S1 samples with phase[31]==1) over
//                                                    the samples of one symbol (symbol = phase>>32 of the
//                                                    unwrapped accumulator; emitted when the first sample of
//                                                    the NEXT symbol arrives). |err| <= TOL_S3.
//                                                    o_biphase = (soft > 0), exact from SYM_SKIP
//  S4 diff dec   o_data_bit   o_data_valid           o_biphase[n] ^ o_biphase[n-1] (prev = 0 at reset)
//  S5 syndrome   o_syndrome[9:0]  o_syn_valid        one per data bit; (26,10) syndrome of the LAST 26 bits
//                                                    (sliding window, oldest bit = MSB), valid from bit 25
//  S6 offset     o_offset_type[2:0]   (o_syn_valid)  0 none, 1 A, 2 B, 3 C, 4 C', 5 D -- which offset word
//                                                    the window ending at this bit matches
//  S7 block sync o_block_data[15:0], o_block_type[2:0]  o_block_valid   o_locked
//                                                    lock after LOCK_RUN consecutive blocks 26 bits apart whose
//                                                    types follow A->B->(C|C')->D->A; the block completing the
//                                                    run is the first event, then one event per block while
//                                                    locked; an illegal successor drops lock and re-hunts
//  S8 content    o_pi[15:0], o_ps[63:0] (char 0 = [63:56]), o_rt[511:0] (char 0 = [511:504]), unseen = 0x00
//                                                    final values, checked after the stream ends
//
//  Every *_valid is a single-cycle pulse; the data is sampled in that cycle.
//  Items are matched by ORDER, not latency, so pipeline depth is free.
//  Outputs of stages S1+ may lag by up to SLACK_Sx trailing items (no more
//  input arrives to flush them) and extra items beyond the golden count are
//  ignored; items before the first golden mismatch are all that is reported.
//
module tb_rds_golden;

`include "rds_gold_cfg.svh"

localparam real CLK_PERIOD_NS = 1000.0 / 30.0;   // 30 MHz dsp_clk
localparam bit  CHECK_RT      = 1'b1;            // set 0 until RadioText is implemented
localparam int  MAX_PRINT     = 5;               // mismatch lines printed per stage

int strb_div = 125;                               // +STRB_DIV=n overrides

// ---- golden memories ------------------------------------------------------
logic        [15:0] in_mem  [0:N_IN-1];
logic signed [15:0] g_s0    [0:N_S0-1];
logic signed [15:0] g_s1    [0:N_S1-1];
logic        [31:0] g_s2    [0:N_S2-1];
logic signed [31:0] g_s3s   [0:N_SYM-1];
logic         [7:0] g_s3b   [0:N_SYM-1];
logic         [7:0] g_s4    [0:N_BIT-1];
logic        [15:0] g_s5    [0:N_BIT-1];
logic         [7:0] g_s6    [0:N_BIT-1];
logic        [15:0] g_s7d   [0:N_BLK-1];
logic         [7:0] g_s7t   [0:N_BLK-1];

initial begin
    $readmemh("rds_gold_in.hex",       in_mem);
    $readmemh("rds_gold_s0.hex",       g_s0);
    $readmemh("rds_gold_s1.hex",       g_s1);
    $readmemh("rds_gold_s2.hex",       g_s2);
    $readmemh("rds_gold_s3_soft.hex",  g_s3s);
    $readmemh("rds_gold_s3_bit.hex",   g_s3b);
    $readmemh("rds_gold_s4.hex",       g_s4);
    $readmemh("rds_gold_s5.hex",       g_s5);
    $readmemh("rds_gold_s6.hex",       g_s6);
    $readmemh("rds_gold_s7_data.hex",  g_s7d);
    $readmemh("rds_gold_s7_type.hex",  g_s7t);
end

// ---- DUT ------------------------------------------------------------------
logic clk = 1'b0;
logic rstb = 1'b0;
logic signed [15:0] i_mpx_data = '0;
logic strb = 1'b0;

logic signed [15:0] o_rds_data;   logic valid;
logic signed [15:0] o_lpf_data;   logic o_lpf_valid;
logic        [31:0] o_nco_phase;  logic o_nco_valid;
logic signed [23:0] o_soft;       logic o_biphase;  logic o_soft_valid;
logic               o_data_bit;   logic o_data_valid;
logic        [9:0]  o_syndrome;   logic [2:0] o_offset_type;  logic o_syn_valid;
logic        [15:0] o_block_data; logic [2:0] o_block_type;   logic o_block_valid;  logic o_locked;
logic        [15:0] o_pi;
logic        [63:0] o_ps;
logic       [511:0] o_rt;

rds dut (
    .clk(clk), .rstb(rstb), .i_mpx_data(i_mpx_data), .strb(strb),
    .o_rds_data(o_rds_data),     .valid(valid),
    .o_lpf_data(o_lpf_data),     .o_lpf_valid(o_lpf_valid),
    .o_nco_phase(o_nco_phase),   .o_nco_valid(o_nco_valid),
    .o_soft(o_soft),             .o_biphase(o_biphase),   .o_soft_valid(o_soft_valid),
    .o_data_bit(o_data_bit),     .o_data_valid(o_data_valid),
    .o_syndrome(o_syndrome),     .o_offset_type(o_offset_type), .o_syn_valid(o_syn_valid),
    .o_block_data(o_block_data), .o_block_type(o_block_type),
    .o_block_valid(o_block_valid), .o_locked(o_locked),
    .o_pi(o_pi), .o_ps(o_ps), .o_rt(o_rt)
);

always #(CLK_PERIOD_NS/2.0) clk = ~clk;

// ---- stimulus: one single-cycle strobe every strb_div clocks -------------------
initial begin
    if ($value$plusargs("STRB_DIV=%d", strb_div)) ;
    $display("=== tb_rds_golden: %0d input samples, strobe every %0d clocks ===", N_IN, strb_div);
    repeat (10) @(posedge clk);
    rstb <= 1'b1;
    repeat (5) @(posedge clk);
    for (int n = 0; n < N_IN; n++) begin
        i_mpx_data <= in_mem[n];
        strb       <= 1'b1;
        @(posedge clk);
        strb       <= 1'b0;
        repeat (strb_div - 1) @(posedge clk);
        if (n % 20000 == 19999) $display("  ... %0d / %0d input samples", n + 1, N_IN);
    end
    repeat (4 * strb_div + 2000) @(posedge clk);
    final_report();
    $finish;
end

// ---- bookkeeping ---------------------------------------------------------------
localparam int NST = 9;
string stage_name [NST] = '{"S0 CIC decimator      ", "S1 low-pass filter    ", "S2 symbol NCO phase   ",
                            "S3 biphase soft/bit   ", "S4 differential decode", "S5 (26,10) syndrome  ",
                            "S6 offset-word match  ", "S7 block sync / lock  ", "S8 PI / PS / RadioText"};
int cnt   [NST];   // valid pulses seen
int nchk  [NST];   // items actually compared
int nerr  [NST];   // mismatches
int nexp  [NST];   // golden item count
int slack [NST];
int maxerr[NST];   // largest tolerance-stage error seen (info only)

initial begin
    for (int i = 0; i < NST; i++) begin cnt[i] = 0; nchk[i] = 0; nerr[i] = 0; maxerr[i] = 0; end
    nexp[0] = N_S0;  nexp[1] = N_S1;  nexp[2] = N_S2;  nexp[3] = N_SYM;  nexp[4] = N_BIT;
    nexp[5] = N_BIT; nexp[6] = N_BIT; nexp[7] = N_BLK; nexp[8] = 1;
    slack[0] = SLACK_S0; slack[1] = SLACK_S1; slack[2] = SLACK_S2; slack[3] = SLACK_S3;
    slack[4] = SLACK_S4; slack[5] = SLACK_S5; slack[6] = SLACK_S6; slack[7] = SLACK_S7; slack[8] = 0;
end

function automatic bit has_x(input logic [511:0] v);
    return (^v === 1'bx);
endfunction

task automatic fail(input int st, input int idx, input string msg);
    nerr[st]++;
    if (nerr[st] <= MAX_PRINT) $display("[FAIL %s] item %0d @ %0t: %s", stage_name[st], idx, $time, msg);
endtask

// Waveform aids (S1): golden expected value / error / item index for the item
// being compared, held until the next S1 item. Add to the wave next to o_lpf_data.
logic signed [15:0] dbg_s1_exp = '0;
int                 dbg_s1_err = 0;
int                 dbg_s1_idx = -1;

// ---- checkers (sample DUT outputs at the clock edge: sees pre-NBA, i.e. consistent, values) ----
always @(posedge clk) if (rstb) begin
    int i, d;

    // S0 ------------------------------------------------------------------
    if (valid === 1'b1) begin
        i = cnt[0];
        if (i < N_S0) begin
            nchk[0]++;
            if (has_x(512'(o_rds_data))) fail(0, i, "o_rds_data is X/Z");
            else if (o_rds_data !== g_s0[i])
                fail(0, i, $sformatf("got %0d, expected %0d", o_rds_data, g_s0[i]));
        end
        cnt[0]++;
    end

    // S1 ------------------------------------------------------------------
    if (o_lpf_valid === 1'b1) begin
        i = cnt[1];
        if (i < N_S1) begin
            nchk[1]++;
            if (has_x(512'(o_lpf_data))) fail(1, i, "o_lpf_data is X/Z");
            else begin
                d = int'(o_lpf_data) - int'(g_s1[i]);
                dbg_s1_exp <= g_s1[i]; dbg_s1_err <= d; dbg_s1_idx <= i;
                if (d < 0) d = -d;
                if (d > maxerr[1]) maxerr[1] = d;
                if (d > TOL_S1)
                    fail(1, i, $sformatf("got %0d, expected %0d (|err| %0d > TOL_S1 %0d)", o_lpf_data, g_s1[i], d, TOL_S1));
            end
        end
        cnt[1]++;
    end

    // S2 ------------------------------------------------------------------
    if (o_nco_valid === 1'b1) begin
        i = cnt[2];
        if (i < N_S2) begin
            nchk[2]++;
            if (has_x(512'(o_nco_phase))) fail(2, i, "o_nco_phase is X/Z");
            else if (o_nco_phase !== g_s2[i])
                fail(2, i, $sformatf("got 0x%08h, expected 0x%08h", o_nco_phase, g_s2[i]));
        end
        cnt[2]++;
    end

    // S3 ------------------------------------------------------------------
    if (o_soft_valid === 1'b1) begin
        i = cnt[3];
        if (i < N_SYM) begin
            nchk[3]++;
            if (has_x(512'(o_soft)) || has_x(512'(o_biphase))) fail(3, i, "o_soft/o_biphase is X/Z");
            else begin
                d = int'(o_soft) - int'(g_s3s[i]);
                if (d < 0) d = -d;
                if (d > maxerr[3]) maxerr[3] = d;
                if (d > TOL_S3)
                    fail(3, i, $sformatf("soft got %0d, expected %0d (|err| %0d > TOL_S3 %0d)", o_soft, g_s3s[i], d, TOL_S3));
                else if (i >= SYM_SKIP && i < SYM_END && o_biphase !== g_s3b[i][0])
                    fail(3, i, $sformatf("o_biphase got %0b, expected %0b (soft %0d)", o_biphase, g_s3b[i][0], o_soft));
            end
        end
        cnt[3]++;
    end

    // S4 ------------------------------------------------------------------
    if (o_data_valid === 1'b1) begin
        i = cnt[4];
        if (i < N_BIT) begin
            nchk[4]++;
            if (has_x(512'(o_data_bit))) fail(4, i, "o_data_bit is X/Z");
            else if (i >= SYM_SKIP && i < SYM_END && o_data_bit !== g_s4[i][0])
                fail(4, i, $sformatf("got %0b, expected %0b", o_data_bit, g_s4[i][0]));
        end
        cnt[4]++;
    end

    // S5 / S6 share o_syn_valid -----------------------------------------------
    if (o_syn_valid === 1'b1) begin
        i = cnt[5];
        if (i < N_BIT) begin
            nchk[5]++; nchk[6]++;
            if (i >= S5_FROM && i < SYM_END) begin
                if (has_x(512'(o_syndrome))) fail(5, i, "o_syndrome is X/Z");
                else if (o_syndrome !== g_s5[i][9:0])
                    fail(5, i, $sformatf("got 0x%03h, expected 0x%03h", o_syndrome, g_s5[i][9:0]));
                if (has_x(512'(o_offset_type))) fail(6, i, "o_offset_type is X/Z");
                else if (o_offset_type !== g_s6[i][2:0])
                    fail(6, i, $sformatf("got %0d, expected %0d", o_offset_type, g_s6[i][2:0]));
            end
        end
        cnt[5]++; cnt[6]++;
    end

    // S7 ------------------------------------------------------------------
    if (o_block_valid === 1'b1) begin
        i = cnt[7];
        if (i < N_BLK) begin
            nchk[7]++;
            if (has_x(512'(o_block_data)) || has_x(512'(o_block_type))) fail(7, i, "block data/type is X/Z");
            else if (o_block_data !== g_s7d[i] || o_block_type !== g_s7t[i][2:0])
                fail(7, i, $sformatf("got data 0x%04h type %0d, expected data 0x%04h type %0d",
                                     o_block_data, o_block_type, g_s7d[i], g_s7t[i][2:0]));
        end
        cnt[7]++;
    end
end

// ---- final report --------------------------------------------------------------
task automatic final_report();
    int nfail = 0;
    bit pass [NST];

    // S8: final register values, plus lock still held at the end
    nchk[8] = 3 + int'(CHECK_RT);
    if (o_locked !== 1'b1) fail(8, 0, $sformatf("o_locked = %b at end of stream, expected 1", o_locked));
    if (o_pi !== EXP_PI) fail(8, 0, $sformatf("o_pi got 0x%04h, expected 0x%04h", o_pi, EXP_PI));
    if (o_ps !== EXP_PS) fail(8, 0, $sformatf("o_ps got 0x%016h, expected 0x%016h", o_ps, EXP_PS));
    if (CHECK_RT && o_rt !== EXP_RT) fail(8, 0, $sformatf("o_rt got 0x%0128h, expected 0x%0128h", o_rt, EXP_RT));
    cnt[8] = 1;

    $display("");
    $display("=== tb_rds_golden summary ===");
    $display("  %-24s | %8s | %8s | %8s | %8s | result", "stage", "items", "expected", "compared", "mismatch");
    for (int s = 0; s < NST; s++) begin
        bit short_count = (cnt[s] < nexp[s] - slack[s]);
        pass[s] = (nerr[s] == 0) && !short_count;
        if (!pass[s]) nfail++;
        $display("  %-24s | %8d | %8d | %8d | %8d | %s%s", stage_name[s], cnt[s], nexp[s], nchk[s], nerr[s],
                 pass[s] ? "PASS" : "FAIL",
                 (short_count && s != 8) ? $sformatf("  (only %0d of >= %0d items produced)", cnt[s], nexp[s] - slack[s]) : "");
    end
    $display("  max |err| seen: S1 %0d (tol %0d), S3 soft %0d (tol %0d)", maxerr[1], TOL_S1, maxerr[3], TOL_S3);
    if (nfail == 0) $display("=== RESULT: PASS (all %0d stages) ===", NST);
    else            $display("=== RESULT: FAIL (%0d of %0d stages failing) ===", nfail, NST);
endtask

endmodule
