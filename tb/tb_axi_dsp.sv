`timescale 1ns / 1ps
//
// Self-checking testbench for axi_dsp (src/axi_dsp.sv), against the real
// S_AXI_HP0 slave interface fm_receiver.sv exposes.
//
// Contract checked:
//   - clk_dsp ~29.8MHz (hardware-measured AD9361 RX sample rate),
//     clk_fpga 100MHz (fclk0) -- non-integer ratio genuinely exercises
//     the CDC. i_valid gates real data, at most 1-in-8 clk_dsp cycles
//     (decimation); no i_ready, dropping is acceptable if axi_dsp can't
//     capture a pulse.
//   - WDATA is always full 64 bits real data (WSTRB=8'hFF, AWSIZE=3'b011,
//     8 bytes/beat), no padding.
//   - No fixed burst length or bank-to-burst ratio assumed: axi_dsp may
//     issue any number of AXI3-legal bursts (<=16 beats) as long as every
//     sample appears exactly once, in arrival order.
//   - Expected content comes from a plain ordered FIFO of every sample
//     fed to the DSP inputs (spec-driven, not mirroring the DUT's own
//     bank/addressing logic) -- see git history for an earlier version
//     that mirrored internal bookkeeping and would have hidden a real
//     dropped-sample bug.
//   - Packing order WDATA={ch0_i,ch0_q,ch1_i,ch1_q} confirmed from the
//     module's own port order. Only ch0_i carries the checked counter
//     pattern; ch0_q/ch1_i/ch1_q are zero (real per-channel stimulus is a
//     follow-up).
//   - No read channel modeled: axi_dsp is write-only, fm_receiver.sv ties
//     off AR/R separately.
//
// Coverage: reset state; several consecutive bursts (AWSIZE/AWBURST/
// AWLOCK, no 4KB-boundary crossing, WLAST placement, AWID==WID, WSTRB,
// FIFO-order content); AXI backpressure on AWREADY/WREADY/BVALID; AWID/WID
// driven (not X); AWADDR checked against an independently-tracked
// circular write pointer (BANK_BASE=0x02000000, REGION_SIZE=1MB) advanced
// from observed AWLEN/AWSIZE only -- a dedicated long sweep (~8192 bursts,
// removed after passing, see private/sample_streaming_plan.md) confirmed
// the wrap-to-0 behavior; axi_dsp's pl_wen/pl_index/pl_update wired into a
// real axi_notifications instance, checking one notification per 8 bursts
// with the index advancing 0,1,2, read back over its own AXI bus.
//
// Known open items, not this testbench's job to fix:
//   - If axi_dsp's internal buffering is a single-bit "pending" flag
//     rather than a counter/FIFO, a second buffer-full condition arriving
//     before the first drains could silently overwrite it. Not exercised
//     as an edge case, but the FIFO-order check would catch the symptom
//     (skipped/duplicated samples) if it ever happened.
//   - The burst FSM returns to BM_IDLE and can fire a new AW immediately
//     without checking BVALID/BREADY first, so back-to-back outstanding
//     bursts look structurally possible on real hardware. This testbench
//     always lets one burst's B phase finish first, so that overlap is
//     untested -- worth a dedicated test once it's a priority.

module tb_axi_dsp;

    // ------------------------------------------------------------------
    // Parameters
    // ------------------------------------------------------------------
    // clk_dsp: the faster of the two real, hardware-confirmed AD9361 RX
    // sample rates (see header comment) -- ~29.8MHz measured, period
    // ~33.56ns. clk_fpga: 100MHz, matching fclk0 elsewhere in this
    // project. Non-integer ratio between them (inherent to the real
    // measured rate, not a testbench contrivance) genuinely exercises
    // the CDC rather than coincidentally sampling in phase.
    localparam real DSP_CLK_PERIOD_NS  = 33.56;
    localparam real FPGA_CLK_PERIOD_NS = 10.0;
    localparam int  TIMEOUT_CYCLES     = 500;  // AXI handshake guard (fpga_clk cycles)
    localparam int  DECIMATION         = 8;    // minimum stated decimation: 1 real sample per 8 clk_dsp cycles
    localparam int  MAX_BEATS_PER_BURST = 16;  // AXI3 hard ceiling (4-bit AWLEN)

    // Circular DDR write region -- must be kept in sync by hand with
    // axi_dsp.sv's own BANK_BASE/REGION_SIZE localparams, since the module
    // doesn't expose them as ports/parameters this testbench can read.
    // Agreed 2026-09-05: 1MB reserved at 0x02000000, comfortably clear of
    // every other DDR region this project already uses (GEM descriptors/
    // frame buffers, this program's own load window).
    localparam logic [31:0] BANK_BASE   = 32'h0200_0000;
    localparam logic [31:0] REGION_SIZE = 32'h0010_0000; // 1 MByte

    // ------------------------------------------------------------------
    // DUT signals — named and widthed to match the REAL S_AXI_HP0 slave
    // interface (see fm_receiver.sv's s_axi_hp0_* declarations, prefix
    // dropped) and the agreed DSP-side interface, not axi_dsp.sv's
    // actual current ports. Wired explicitly below (no .* ), so any
    // mismatch against the real module shows up as a clean elaboration
    // error rather than a silently-unconnected port.
    // ------------------------------------------------------------------
    logic        clk_fpga;
    logic        clk_dsp;
    logic        rstb_dsp;
    logic        rstb_fpga;

    logic        i_valid;
    logic [15:0] ch0_i;
    logic [15:0] ch0_q;
    logic [15:0] ch1_i;
    logic [15:0] ch1_q;

    logic [31:0] awaddr;
    logic  [1:0] awburst;
    logic  [3:0] awcache;
    logic  [5:0] awid;
    logic  [3:0] awlen;
    logic  [1:0] awlock;
    logic  [2:0] awprot;
    logic  [3:0] awqos;
    logic        awready;
    logic  [2:0] awsize;
    logic        awvalid;

    logic [63:0] wdata;
    logic  [5:0] wid;
    logic        wlast;
    logic        wready;
    logic  [7:0] wstrb;
    logic        wvalid;

    logic  [5:0] bid;
    logic        bready;
    logic  [1:0] bresp;
    logic        bvalid;

    // PL-to-PS notification port (feeds axi_notifications below) --
    // agreed 2026-09-05/06 (private/sample_streaming_plan.md): pl_index
    // always selects register 0 (axi_dsp's own notification word);
    // pl_update's bit0 is the ready flag, bits[10:1] the 10-bit index of
    // which 1KB slice of the 1MB region just got written (0-1023, since
    // 1MB/1KB = 1024 exactly).
    logic [31:0] pl_update;
    logic  [1:0] pl_index;
    logic        pl_wen;

    // ------------------------------------------------------------------
    // Scoreboard
    // ------------------------------------------------------------------
    int unsigned pass_count = 0;
    int unsigned fail_count = 0;

    // Independently-tracked expected DDR write pointer (byte offset within
    // the 1MB region) -- advanced here purely from AWADDR/AWLEN/AWSIZE as
    // observed on the bus, never by reading axi_dsp's own wr_offset. Same
    // spec-driven principle as sample_fifo below: this checks the DUT's
    // externally observable address sequencing against the agreed
    // contract, not against a copy of its own internal bookkeeping.
    logic [31:0] expected_wr_offset;

    task automatic check(input string name, input logic [63:0] actual, input logic [63:0] expected);
        if (actual === expected) begin
            pass_count++;
            $display("[PASS] t=%0t %-52s got=0x%016h", $time, name, actual);
        end else begin
            fail_count++;
            $display("[FAIL] t=%0t %-52s got=0x%016h expected=0x%016h", $time, name, actual, expected);
        end
    endtask

    task automatic check_bit(input string name, input logic actual, input logic expected);
        if (actual === expected) begin
            pass_count++;
            $display("[PASS] t=%0t %-52s got=%0b", $time, name, actual);
        end else begin
            fail_count++;
            $display("[FAIL] t=%0t %-52s got=%0b expected=%0b", $time, name, actual, expected);
        end
    endtask

    // WID must match its AW's AWID per AXI3, and on real silicon an
    // undriven ID net is not "X" -- it's whatever the synthesis tool
    // decides, which is a real correctness risk, not a sim artifact.
    // X===X silently passes the equality checks above, so this catches
    // "never actually driven" explicitly instead of letting it hide.
    task automatic check_not_x(input string name, input logic [63:0] actual);
        check_bit(name, (^actual) !== 1'bx, 1'b1);
    endtask

    // Verifies one burst's AWADDR against the independently-tracked
    // circular write pointer, verifies the burst never overruns the
    // reserved 1MB region, then advances/wraps that pointer -- the same
    // wrap RULE the agreed contract states (advance by this burst's byte
    // count, fold to 0 at the region edge), applied to an independent
    // counter, not a read of the DUT's own wr_offset register.
    task automatic check_wr_pointer(
        input  string      tag,
        input  logic [31:0] addr,
        input  int          num_beats,
        input  logic  [2:0] size,
        output bit          did_wrap
    );
        logic [31:0] burst_bytes;
        burst_bytes = num_beats * (32'h1 << size);

        check($sformatf("%s: AWADDR matches expected DDR write pointer", tag),
              {32'h0, addr}, {32'h0, BANK_BASE + expected_wr_offset});
        check_bit($sformatf("%s: burst stays inside the reserved 1MB region (no overrun)", tag),
                  (expected_wr_offset + burst_bytes) <= REGION_SIZE, 1'b1);

        did_wrap = (expected_wr_offset + burst_bytes) >= REGION_SIZE;
        if (did_wrap) expected_wr_offset = 32'h0;
        else          expected_wr_offset = expected_wr_offset + burst_bytes;
    endtask

    // ------------------------------------------------------------------
    // DUT
    // ------------------------------------------------------------------
    axi_dsp dut (
        .clk_fpga  (clk_fpga),
        .clk_dsp   (clk_dsp),
        .rstb_dsp  (rstb_dsp),
        .rstb_fpga (rstb_fpga),

        .i_valid (i_valid),
        .ch0_i   (ch0_i),
        .ch0_q   (ch0_q),
        .ch1_i   (ch1_i),
        .ch1_q   (ch1_q),

        .awid    (awid),
        .awaddr  (awaddr),
        .awlen   (awlen),
        .awsize  (awsize),
        .awburst (awburst),
        .awlock  (awlock),
        .awcache (awcache),
        .awprot  (awprot),
        .awqos   (awqos),
        .awvalid (awvalid),
        .awready (awready),

        .wid    (wid),
        .wdata  (wdata),
        .wstrb  (wstrb),
        .wlast  (wlast),
        .wvalid (wvalid),
        .wready (wready),

        .bid    (bid),
        .bresp  (bresp),
        .bvalid (bvalid),
        .bready (bready),

        .pl_update (pl_update),
        .pl_index  (pl_index),
        .pl_wen    (pl_wen)
    );

    // ------------------------------------------------------------------
    // axi_notifications (src/axi_notifications.sv) -- the real consumer
    // of axi_dsp's pl_wen/pl_index/pl_update port. Wired here (not just
    // watched as raw axi_dsp output pins) so the check below verifies the
    // composed pipeline end-to-end: read back what actually landed in the
    // register through its own AXI-facing bus, the same interface real
    // firmware will use, rather than trusting axi_dsp's output ports in
    // isolation. This testbench drives axi_notifications' peripheral bus
    // directly (no axi_if in this testbench, same reasoning as driving
    // axi_dsp's AXI3 signals directly via the BFM above) -- only the read
    // side is exercised here, since this test never needs to ack/clear
    // the flag itself.
    // ------------------------------------------------------------------
    localparam logic [7:0] NOTIF_PERIPH_ID = 8'h04;

    logic        notif_ren;
    logic [31:0] notif_raddr;
    logic        notif_rdone;
    logic [31:0] notif_rdata;
    logic        notif_rnoaddr;
    logic [127:0] notif_reg_out_unused;

    axi_notifications #(
        .PERIPH_ID (NOTIF_PERIPH_ID)
    ) notif (
        .clk  (clk_fpga),
        .rstb (rstb_fpga),

        .wen    (1'b0),
        .w_addr (32'h0),
        .w_data (32'h0),
        .w_strb (4'h0),
        .w_done (),
        .w_no_addr (),

        .ren    (notif_ren),
        .r_addr (notif_raddr),
        .r_done (notif_rdone),
        .r_data (notif_rdata),
        .r_no_addr (notif_rnoaddr),

        .pl_wen    (pl_wen),
        .pl_windex (pl_index),
        .pl_wdata  (pl_update),

        .reg_out (notif_reg_out_unused)
    );

    // Background count of every pl_wen pulse axi_dsp issues -- purely
    // diagnostic (the real check below is the AXI-side readback), but
    // cheap insurance: if the readback checks ever pass despite a wrong
    // pulse cadence (e.g. because a checkpoint happened to land after an
    // extra, unexpected pulse already overwrote the register with the
    // "right" next value), this count makes that visible instead of
    // hiding it.
    int unsigned pl_wen_count;
    always_ff @(posedge clk_fpga or negedge rstb_fpga) begin
        if (!rstb_fpga) pl_wen_count <= 0;
        else if (pl_wen) pl_wen_count <= pl_wen_count + 1;
    end

    // Reads one of axi_notifications' 4 registers directly over its
    // peripheral bus (combinational r_done/r_data, no CDC -- see
    // axi_notifications.sv's header comment).
    task automatic notif_read(input logic [1:0] idx, output logic [31:0] data, output bit done);
        notif_raddr = {8'h0, NOTIF_PERIPH_ID, 12'h0, idx, 2'b00};
        notif_ren   = 1'b1;
        @(posedge clk_fpga);
        data = notif_rdata;
        done = notif_rdone;
        notif_ren = 1'b0;
        @(posedge clk_fpga);
    endtask

    // ------------------------------------------------------------------
    // Clocks
    // ------------------------------------------------------------------
    initial clk_dsp = 0;
    always #(DSP_CLK_PERIOD_NS/2.0) clk_dsp = ~clk_dsp;

    initial clk_fpga = 0;
    always #(FPGA_CLK_PERIOD_NS/2.0) clk_fpga = ~clk_fpga;

    // ------------------------------------------------------------------
    // Sample producer (dsp_clk domain): a free-running 16-bit counter on
    // ch0_i, advancing and pulsing i_valid for exactly one clk_dsp cycle
    // every DECIMATION cycles -- modeling real data arriving no faster
    // than 1-in-8, tested at that minimum (tightest) cadence. ch0_q/
    // ch1_i/ch1_q stay at a constant zero (see header comment). Every
    // valid sample is pushed onto sample_fifo in strict arrival order,
    // with zero knowledge of the DUT's own internal bookkeeping -- the
    // checker below pops exactly as many entries as a burst's AWLEN
    // claims and compares them in order; any mismatch means the DUT
    // dropped, duplicated, or reordered a sample, regardless of which
    // internal mechanism caused it.
    // ------------------------------------------------------------------
    logic [15:0] sample_counter;
    int          decim_ctr;
    logic [15:0] sample_fifo [$];

    always_ff @(posedge clk_dsp or negedge rstb_dsp) begin
        if (!rstb_dsp) begin
            sample_counter <= '0;
            decim_ctr      <= 0;
            i_valid        <= 1'b0;
        end else begin
            if (decim_ctr == DECIMATION - 1) begin
                decim_ctr      <= 0;
                i_valid        <= 1'b1;
                // push the value ch0_i will actually show once i_valid is
                // visible (this same edge also advances sample_counter,
                // and ch0_i tracks it combinationally) -- not the
                // pre-increment value read here, which sample_counter
                // will have already moved past by the time i_valid=1.
                sample_fifo.push_back(sample_counter + 16'b1);
                sample_counter <= sample_counter + 16'b1;
            end else begin
                decim_ctr <= decim_ctr + 1;
                i_valid   <= 1'b0;
            end
        end
    end

    always_comb ch0_i = sample_counter;
    always_comb ch0_q = '0;
    always_comb ch1_i = '0;
    always_comb ch1_q = '0;

    // ------------------------------------------------------------------
    // Reset
    // ------------------------------------------------------------------
    task automatic do_reset;
        expected_wr_offset = 32'h0;
        awready <= 1'b0;
        wready  <= 1'b0;
        bvalid  <= 1'b0;
        bresp   <= 2'b00;
        bid     <= '0;

        rstb_dsp  <= 1'b0;
        rstb_fpga <= 1'b0;
        repeat (5) @(posedge clk_fpga);
        repeat (5) @(posedge clk_dsp);
        rstb_dsp  <= 1'b1;
        rstb_fpga <= 1'b1;
        @(posedge clk_fpga);
    endtask

    // ------------------------------------------------------------------
    // AXI3 write slave BFM (fpga_clk domain): accepts exactly one burst
    // per call, of whatever length the DUT itself requests via AWLEN
    // (no fixed/assumed burst length), applying the requested per-phase
    // delays. BID always echoes back whatever AWID this burst's AW
    // carried, matching a real HP-port slave's behavior.
    // ------------------------------------------------------------------
    task automatic accept_one_burst(
        output logic [31:0] addr_out,
        output logic  [3:0] len_out,
        output logic  [2:0] size_out,
        output logic  [1:0] burst_out,
        output logic  [1:0] lock_out,
        output logic  [5:0] awid_out,
        ref    logic [63:0] beats [$],
        ref    logic  [5:0] wids  [$],
        output int unsigned wlast_beat_idx,
        output bit           wlast_seen,
        output bit           timed_out,
        input  int           aw_ready_delay = 0,
        input  int           w_ready_delay  = 0,
        input  int           bvalid_delay   = 0
    );
        int cnt;
        int num_beats;
        timed_out      = 0;
        wlast_seen     = 0;
        wlast_beat_idx = 0;
        beats.delete();
        wids.delete();

        // ---- AW ----
        awready <= 1'b0;
        cnt = 0;
        do begin
            @(posedge clk_fpga);
            cnt++;
        end while (!awvalid && cnt < TIMEOUT_CYCLES);
        if (cnt >= TIMEOUT_CYCLES) begin
            timed_out = 1;
            return;
        end

        repeat (aw_ready_delay) @(posedge clk_fpga);
        addr_out  = awaddr;
        len_out   = awlen;
        size_out  = awsize;
        burst_out = awburst;
        lock_out  = awlock;
        awid_out  = awid;
        awready <= 1'b1;
        @(posedge clk_fpga);
        awready <= 1'b0;

        num_beats = int'(len_out) + 1;

        // AXI protocol ceiling: no burst may cross a 4KB address boundary
        // (UG585 ch.9 confirms this is a general AXI rule, not
        // DMAC-specific). AWSIZE is asserted elsewhere to always be 8
        // bytes/beat; computed generically here anyway in case that
        // assumption ever changes.
        begin
            int unsigned bytes_per_beat = 1 << size_out;
            int unsigned total_bytes    = num_beats * bytes_per_beat;
            int unsigned page_offset    = addr_out & 32'hFFF;
            check_bit($sformatf("Burst @0x%08h: does not cross a 4KB boundary", addr_out),
                      (page_offset + total_bytes) <= 32'h1000, 1'b1);
        end

        // ---- W: exactly num_beats beats, whatever the DUT claimed ----
        wready <= 1'b0;
        repeat (w_ready_delay) @(posedge clk_fpga);
        wready <= 1'b1;

        for (int i = 0; i < num_beats; i++) begin
            cnt = 0;
            do begin
                @(posedge clk_fpga);
                cnt++;
            end while (!wvalid && cnt < TIMEOUT_CYCLES);
            if (cnt >= TIMEOUT_CYCLES) begin
                timed_out = 1;
                wready <= 1'b0;
                return;
            end
            beats.push_back(wdata);
            wids.push_back(wid);
            check($sformatf("Burst beat %0d: WSTRB (always full-width)", i), {56'h0, wstrb}, {56'h0, 8'hFF});
            if (wlast) begin
                wlast_seen     = 1;
                wlast_beat_idx = i;
            end else if (i == num_beats - 1) begin
                fail_count++;
                $display("[FAIL] t=%0t Burst beat %0d: expected WLAST on final beat, not seen", $time, i);
            end
        end
        wready <= 1'b0;

        // ---- B: echo the AWID this burst carried, per AXI3 ----
        bvalid <= 1'b0;
        repeat (bvalid_delay) @(posedge clk_fpga);
        bid    <= awid_out;
        bresp  <= 2'b00;
        bvalid <= 1'b1;
        cnt = 0;
        do begin
            @(posedge clk_fpga);
            cnt++;
        end while (!bready && cnt < TIMEOUT_CYCLES);
        if (cnt >= TIMEOUT_CYCLES) begin
            timed_out = 1;
        end
        bvalid <= 1'b0;
        @(posedge clk_fpga);
    endtask

    // ------------------------------------------------------------------
    // One burst, checked against the next entries in sample_fifo (see
    // header comment on why this is FIFO-order, not a re-derivation of
    // the DUT's own addressing/bank bookkeeping). No assumption about
    // burst length or how many bursts make up a "bank" -- whatever AWLEN
    // the DUT issues, that many samples get popped and checked.
    // ------------------------------------------------------------------
    task automatic do_burst_and_check(
        input string tag,
        input int aw_ready_delay = 0,
        input int w_ready_delay  = 0,
        input int bvalid_delay   = 0
    );
        logic [31:0] addr;
        logic  [3:0] len;
        logic  [2:0] size;
        logic  [1:0] burst;
        logic  [1:0] lock;
        logic  [5:0] this_awid;
        logic [63:0] beats [$];
        logic  [5:0] wids  [$];
        int unsigned wlast_idx;
        bit wlast_seen;
        bit timed_out;
        logic [15:0] expected_sample;
        int num_beats;

        accept_one_burst(addr, len, size, burst, lock, this_awid, beats, wids, wlast_idx, wlast_seen, timed_out,
                          aw_ready_delay, w_ready_delay, bvalid_delay);

        if (timed_out) begin
            fail_count++;
            $display("[FAIL] t=%0t %-52s burst timed out", $time, tag);
            return;
        end

        num_beats = int'(len) + 1;
        check_not_x($sformatf("%s: AWID is actually driven (not X)", tag), {58'h0, this_awid});
        check_bit($sformatf("%s: AWLEN within AXI3 legal range (<=16 beats)", tag), (num_beats <= MAX_BEATS_PER_BURST), 1'b1);
        check(    $sformatf("%s: AWSIZE (always 8 bytes/beat, full bus width)", tag), {61'h0, size},  {61'h0, 3'b011});
        check(    $sformatf("%s: AWBURST (INCR)", tag), {62'h0, burst}, {62'h0, 2'b01});
        check(    $sformatf("%s: AWLOCK (normal access)", tag), {62'h0, lock}, {62'h0, 2'b00});
        check_bit($sformatf("%s: WLAST observed exactly once", tag), wlast_seen, 1'b1);
        check(    $sformatf("%s: WLAST on final beat", tag), {56'h0, wlast_idx}, {56'h0, num_beats - 1});
        $display("[INFO] t=%0t %-52s AWADDR=0x%08h AWLEN+1=%0d beats", $time, tag, addr, num_beats);

        begin
            bit unused_wrap;
            check_wr_pointer(tag, addr, num_beats, size, unused_wrap);
        end

        for (int i = 0; i < num_beats; i++) begin
            check_not_x($sformatf("%s: beat %0d WID is actually driven (not X)", tag, i), {58'h0, wids[i]});
            check($sformatf("%s: beat %0d WID matches this burst's AWID", tag, i),
                  {58'h0, wids[i]}, {58'h0, this_awid});

            // Packing-order assumption (see header comment): ch0_i in
            // WDATA[63:48], ch0_q/ch1_i/ch1_q (all driven to 0 here) fill
            // the rest.
            check($sformatf("%s: beat %0d WDATA[47:0] (ch0_q/ch1_i/ch1_q, all zero)", tag, i),
                  {16'h0, beats[i][47:0]}, 64'h0);

            if (sample_fifo.size() == 0) begin
                fail_count++;
                $display("[FAIL] t=%0t %-52s beat %0d: no unconsumed sample left in sample_fifo -- DUT produced more beats than samples were ever driven", $time, tag, i);
                continue;
            end
            expected_sample = sample_fifo.pop_front();
            check($sformatf("%s: beat %0d ch0_i (next unconsumed FIFO entry, assumed WDATA[63:48])", tag, i),
                  {48'h0, beats[i][63:48]},
                  {48'h0, expected_sample});
        end
    endtask

    // ------------------------------------------------------------------
    // Main test sequence
    // ------------------------------------------------------------------
    initial begin
        $display("=== tb_axi_dsp starting ===");

        do_reset();

        check_bit("Reset: AW_VALID low",  awvalid, 1'b0);
        check_bit("Reset: W_VALID low",   wvalid,  1'b0);
        check_bit("Reset: B_READY high",  bready,  1'b1);

        // ---- axi_notifications: one notification every 8 bursts,
        //      LSB=1, index = how many notifications fired before this
        //      one, read back over axi_notifications' own AXI bus. Runs 3
        //      full cycles (24 bursts) so the index is seen to actually
        //      advance (0, 1, 2), not just checked once.
        //
        //      Must run FIRST, right after reset: expected_index assumes
        //      wr_offset starts at 0 relative to this section's own burst
        //      count, which only holds if nothing else has advanced it
        //      yet. ----
        $display("=== axi_notifications: one notification every 8 bursts ===");
        begin
            localparam int NUM_NOTIFICATIONS = 3;
            logic [31:0] notif_data;
            bit          notif_done;
            logic [9:0]  expected_index;
            logic [31:0] expected_update;

            for (int notif_n = 0; notif_n < NUM_NOTIFICATIONS; notif_n++) begin
                for (int b = 0; b < 8; b++) begin
                    do_burst_and_check($sformatf("Notif cycle %0d burst %0d", notif_n, b));
                end

                expected_index  = notif_n[9:0];
                expected_update = {21'h0, expected_index, 1'b1};

                notif_read(2'd0, notif_data, notif_done);
                check_bit($sformatf("Notification %0d: axi_notifications register 0 read claimed", notif_n),
                          notif_done, 1'b1);
                check($sformatf("Notification %0d: register 0 == {index=%0d, ready=1}", notif_n, expected_index),
                      {32'h0, notif_data}, {32'h0, expected_update});
            end

            check($sformatf("Notification test: total pl_wen pulses (diagnostic, expect %0d)", NUM_NOTIFICATIONS),
                  {32'h0, pl_wen_count}, {32'h0, 32'(NUM_NOTIFICATIONS)});
        end

        // ---- Directed: several consecutive bursts, AXI side kept fast
        //      (no delays) so it stays caught up with the i_valid-gated
        //      producer. No assumption about burst length or how many
        //      bursts make up a "bank" -- see header comment. ----
        $display("=== Several consecutive bursts, no backpressure ===");
        for (int n = 0; n < 8; n++) begin
            do_burst_and_check($sformatf("Burst %0d", n));
        end

        // ---- AXI-side backpressure: AWREADY, WREADY, BVALID each
        //      independently delayed ----
        $display("=== AXI backpressure ===");
        do_burst_and_check("Backpressure: AWREADY delay 3", 3, 0, 0);
        do_burst_and_check("Backpressure: WREADY delay 2",  0, 2, 0);
        do_burst_and_check("Backpressure: BVALID delay 4",  0, 0, 4);

        // ------------------------------------------------------------------
        $display("=== tb_axi_dsp finished: %0d passed, %0d failed ===", pass_count, fail_count);
        if (fail_count == 0)
            $display("=== RESULT: PASS ===");
        else
            $display("=== RESULT: FAIL ===");

        $finish;
    end

endmodule
