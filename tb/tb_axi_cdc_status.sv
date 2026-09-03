`timescale 1ns / 1ps
//
// Self-checking testbench for axi_if + axi_cdc_status (src/axi_if.sv,
// src/axi_cdc_status.sv) cascaded together, i.e. how fm_receiver.sv wires
// them: axi_if turns AXI3 into the wen/ren peripheral bus, axi_cdc_status
// is the one peripheral hanging off it in this test (PERIPH_ID =
// TEST_PERIPH_ID below).
//
// axi_cdc_status is read-only from the AXI master's point of view — its
// 64 x 8-bit status_reg lanes are written entirely from the dsp_clk side,
// crossing into the fclk0/AXI domain via a 4-phase request/ack handshake
// (see src/axi_cdc_status.sv's header comment for the exact mechanism).
// This testbench drives BOTH clock domains independently, deliberately at
// a non-integer-multiple ratio (10ns fclk0 vs 7ns dsp_clk) so the
// synchronizers are genuinely exercised rather than getting lucky on an
// aligned-edge relationship.
//
// Coverage in this version:
//   - Directed dsp-side writes to all 64 lanes, read back correctly via
//     AXI (crossing the handshake for real, not white-box shortcuts)
//   - AXI write attempts always DECERR (this is read-only memory from
//     that side), and never touch status_reg
//   - PERIPH_ID mismatch -> DECERR promptly on read
//   - Back-to-back AXI reads, no idle gap forced between them
//   - Concurrent dsp-side write racing an in-flight AXI read of the same
//     lane -- the read must land on either the old or the new value,
//     never a torn/corrupted mix
//   - Randomized stress interleaving dsp writes and AXI reads across all
//     64 lanes
//   - AXI-side backpressure (RREADY delayed) on a read
//
// Scope not covered: the reverse direction (AXI write) doesn't exist by
// design, so there's no WSTRB/staggering coverage to speak of here — see
// tb_axi_fm.sv for that on the peripheral that actually supports it.

module tb_axi_cdc_status;

    // ------------------------------------------------------------------
    // Parameters
    // ------------------------------------------------------------------
    localparam int CLK_PERIOD_NS     = 10;  // 100 MHz, matches fclk0
    localparam int DSP_CLK_PERIOD_NS = 7;   // deliberately non-integer-multiple of CLK_PERIOD_NS
    localparam int NUM_REGS          = 64;
    localparam int TIMEOUT_CYCLES    = 200;
    localparam int CDC_WAIT_TIMEOUT  = 200; // fclk0 cycles to wait for a handshake round trip
    localparam int NUM_RANDOM_ITER   = 800; // keeps ~same per-lane density as the original 200/16

    localparam logic [7:0] TEST_PERIPH_ID  = 8'h03; // matches axi_cdc_status in fm_receiver.sv
    localparam logic [7:0] WRONG_PERIPH_ID = 8'h01; // axi_registers' ID -- nobody here claims it

    // ------------------------------------------------------------------
    // DUT signals — names match axi_if's/axi_cdc_status's ports exactly
    // so `.*` wires everything up automatically.
    // ------------------------------------------------------------------
    logic         clk;
    logic         rstb;

    logic [31:0]  m_axi_gp0_araddr;
    logic [1:0]   m_axi_gp0_arburst;
    logic [3:0]   m_axi_gp0_arcache;
    logic [11:0]  m_axi_gp0_arid;
    logic [3:0]   m_axi_gp0_arlen;
    logic [1:0]   m_axi_gp0_arlock;
    logic [2:0]   m_axi_gp0_arprot;
    logic [3:0]   m_axi_gp0_arqos;
    logic         m_axi_gp0_arready;
    logic [2:0]   m_axi_gp0_arsize;
    logic         m_axi_gp0_arvalid;

    logic [31:0]  m_axi_gp0_awaddr;
    logic [1:0]   m_axi_gp0_awburst;
    logic [3:0]   m_axi_gp0_awcache;
    logic [11:0]  m_axi_gp0_awid;
    logic [3:0]   m_axi_gp0_awlen;
    logic [1:0]   m_axi_gp0_awlock;
    logic [2:0]   m_axi_gp0_awprot;
    logic [3:0]   m_axi_gp0_awqos;
    logic         m_axi_gp0_awready;
    logic [2:0]   m_axi_gp0_awsize;
    logic         m_axi_gp0_awvalid;

    logic [11:0]  m_axi_gp0_bid;
    logic         m_axi_gp0_bready;
    logic [1:0]   m_axi_gp0_bresp;
    logic         m_axi_gp0_bvalid;

    logic [31:0]  m_axi_gp0_rdata;
    logic [11:0]  m_axi_gp0_rid;
    logic         m_axi_gp0_rlast;
    logic         m_axi_gp0_rready;
    logic [1:0]   m_axi_gp0_rresp;
    logic         m_axi_gp0_rvalid;

    logic [31:0]  m_axi_gp0_wdata;
    logic [11:0]  m_axi_gp0_wid;
    logic         m_axi_gp0_wlast;
    logic         m_axi_gp0_wready;
    logic [3:0]   m_axi_gp0_wstrb;
    logic         m_axi_gp0_wvalid;

    // Peripheral bus between axi_if and axi_cdc_status.
    logic        wen;
    logic [31:0] w_addr;
    logic [31:0] w_data;
    logic [3:0]  w_strb;
    logic        w_done;
    logic        w_no_addr;

    logic        ren;
    logic [31:0] r_addr;
    logic        r_done;
    logic [31:0] r_data;
    logic        r_no_addr;

    // DSP-side port
    logic         dsp_clk;
    logic         dsp_rstb;
    logic         dsp_wen;
    logic [5:0]   dsp_windex;
    logic [7:0]   dsp_wdata;
    logic [511:0] dsp_reg_out;

    // ------------------------------------------------------------------
    // Scoreboard
    // ------------------------------------------------------------------
    int unsigned pass_count = 0;
    int unsigned fail_count = 0;

    // Golden model: what every lane SHOULD contain right now.
    logic [7:0] shadow [0:NUM_REGS-1];

    // ------------------------------------------------------------------
    // DUT
    // ------------------------------------------------------------------
    axi_if dut (.*);

    axi_cdc_status #(
        .PERIPH_ID (TEST_PERIPH_ID)
    ) dut_cdc (.*);

    // ------------------------------------------------------------------
    // Clocks — deliberately non-integer-multiple periods so the CDC
    // synchronizers see every possible edge alignment over the course of
    // a run, not just one fixed phase relationship.
    // ------------------------------------------------------------------
    initial clk = 0;
    always #(CLK_PERIOD_NS/2.0) clk = ~clk;

    initial dsp_clk = 0;
    always #(DSP_CLK_PERIOD_NS/2.0) dsp_clk = ~dsp_clk;

    // ------------------------------------------------------------------
    // Address helper. addr[31:24] stands in for whatever M_AXI_GP0 fixes
    // it to on real hardware (0x40-0x7F) -- non-zero here so these tests
    // exercise a genuinely reachable address, not one that only works
    // because axi_cdc_status ignores those bits.
    // ------------------------------------------------------------------
    localparam logic [7:0] GP0_TOP_BYTE = 8'h40;

    function automatic logic [31:0] reg_addr(input int idx);
        return {GP0_TOP_BYTE, TEST_PERIPH_ID, 16'h0} | (idx << 2);
    endfunction

    // ------------------------------------------------------------------
    // Checker
    // ------------------------------------------------------------------
    task automatic check(input string name, input logic [31:0] actual, input logic [31:0] expected);
        if (actual === expected) begin
            pass_count++;
            $display("[PASS] t=%0t %-52s got=0x%08h", $time, name, actual);
        end else begin
            fail_count++;
            $display("[FAIL] t=%0t %-52s got=0x%08h expected=0x%08h", $time, name, actual, expected);
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

    // ------------------------------------------------------------------
    // Reset — both domains independently, since this standalone
    // testbench validates the module's own dsp_rstb behavior even though
    // fm_receiver.sv currently ties it to 1'b1 (no real dsp-domain reset
    // wired up yet at the top level).
    // ------------------------------------------------------------------
    task automatic do_reset;
        m_axi_gp0_awvalid <= 1'b0;
        m_axi_gp0_awaddr  <= '0;
        m_axi_gp0_awid    <= '0;
        m_axi_gp0_awlen   <= '0;
        m_axi_gp0_awsize  <= 3'b010;
        m_axi_gp0_awburst <= 2'b01;
        m_axi_gp0_awlock  <= '0;
        m_axi_gp0_awcache <= '0;
        m_axi_gp0_awprot  <= '0;
        m_axi_gp0_awqos   <= '0;

        m_axi_gp0_wvalid  <= 1'b0;
        m_axi_gp0_wdata   <= '0;
        m_axi_gp0_wstrb   <= '0;
        m_axi_gp0_wid     <= '0;
        m_axi_gp0_wlast   <= 1'b1;

        m_axi_gp0_bready  <= 1'b0;

        m_axi_gp0_arvalid <= 1'b0;
        m_axi_gp0_araddr  <= '0;
        m_axi_gp0_arid    <= '0;
        m_axi_gp0_arlen   <= '0;
        m_axi_gp0_arsize  <= 3'b010;
        m_axi_gp0_arburst <= 2'b01;
        m_axi_gp0_arlock  <= '0;
        m_axi_gp0_arcache <= '0;
        m_axi_gp0_arprot  <= '0;
        m_axi_gp0_arqos   <= '0;

        m_axi_gp0_rready  <= 1'b0;

        dsp_wen    <= 1'b0;
        dsp_windex <= '0;
        dsp_wdata  <= '0;

        for (int i = 0; i < NUM_REGS; i++) shadow[i] = 8'h0;

        rstb     <= 1'b0;
        dsp_rstb <= 1'b0;
        repeat (5) @(posedge clk);
        rstb <= 1'b1;
        repeat (5) @(posedge dsp_clk);
        dsp_rstb <= 1'b1;
        @(posedge clk);
    endtask

    // ------------------------------------------------------------------
    // dsp-side write: one lane, one dsp_clk cycle, native domain so no
    // handshake needed here.
    // ------------------------------------------------------------------
    task automatic dsp_write(input int idx, input logic [7:0] data);
        @(posedge dsp_clk);
        dsp_wen    <= 1'b1;
        dsp_windex <= idx[5:0];
        dsp_wdata  <= data;
        @(posedge dsp_clk);
        dsp_wen <= 1'b0;
        shadow[idx] = data;
    endtask

    // ------------------------------------------------------------------
    // AXI3 write/read transactions — identical in structure to
    // tb_axi_fm.sv's, since axi_cdc_status sits behind the same AXI3
    // slave interface (axi_if).
    // ------------------------------------------------------------------
    task automatic axi_write(
        input  logic [31:0] addr,
        input  logic [31:0] data,
        input  logic [3:0]  strb,
        input  logic [11:0] id,
        output logic [1:0]  bresp_out,
        output logic [11:0] bid_out,
        output bit          timed_out,
        input  int          aw_delay     = 0,
        input  int          w_delay      = 0,
        input  int          bready_delay = 0,
        input  string       tag          = "axi_write"
    );
        int aw_cnt, w_cnt, b_cnt;
        bit aw_done, w_done_l;
        timed_out = 0;

        fork
            begin : AW_DRV
                repeat (aw_delay) @(posedge clk);
                m_axi_gp0_awaddr  <= addr;
                m_axi_gp0_awid    <= id;
                m_axi_gp0_awlen   <= 4'h0;
                m_axi_gp0_awsize  <= 3'b010;
                m_axi_gp0_awburst <= 2'b01;
                m_axi_gp0_awlock  <= 2'b00;
                m_axi_gp0_awcache <= 4'h0;
                m_axi_gp0_awprot  <= 3'h0;
                m_axi_gp0_awqos   <= 4'h0;
                m_axi_gp0_awvalid <= 1'b1;
                aw_cnt = 0;
                do begin
                    @(posedge clk);
                    aw_cnt++;
                end while (!m_axi_gp0_awready && aw_cnt < TIMEOUT_CYCLES);
                aw_done = (aw_cnt < TIMEOUT_CYCLES);
                m_axi_gp0_awvalid <= 1'b0;
            end
            begin : W_DRV
                repeat (w_delay) @(posedge clk);
                m_axi_gp0_wdata  <= data;
                m_axi_gp0_wstrb  <= strb;
                m_axi_gp0_wid    <= id;
                m_axi_gp0_wlast  <= 1'b1;
                m_axi_gp0_wvalid <= 1'b1;
                w_cnt = 0;
                do begin
                    @(posedge clk);
                    w_cnt++;
                end while (!m_axi_gp0_wready && w_cnt < TIMEOUT_CYCLES);
                w_done_l = (w_cnt < TIMEOUT_CYCLES);
                m_axi_gp0_wvalid <= 1'b0;
            end
        join

        if (!aw_done || !w_done_l) begin
            timed_out = 1;
            bresp_out = 2'bxx;
            bid_out   = 12'hxxx;
            return;
        end

        m_axi_gp0_bready <= 1'b0;
        repeat (bready_delay) @(posedge clk);

        m_axi_gp0_bready <= 1'b1;
        b_cnt = 0;
        do begin
            @(posedge clk);
            b_cnt++;
        end while (!m_axi_gp0_bvalid && b_cnt < TIMEOUT_CYCLES);

        if (b_cnt >= TIMEOUT_CYCLES) begin
            timed_out = 1;
            bresp_out = 2'bxx;
            bid_out   = 12'hxxx;
        end else begin
            bresp_out = m_axi_gp0_bresp;
            bid_out   = m_axi_gp0_bid;
        end
        m_axi_gp0_bready <= 1'b0;
        @(posedge clk);
    endtask

    task automatic axi_read(
        input  logic [31:0] addr,
        input  logic [11:0] id,
        output logic [31:0] data_out,
        output logic [1:0]  rresp_out,
        output logic [11:0] rid_out,
        output bit          timed_out,
        input  int          rready_delay = 0,
        input  string       tag          = "axi_read"
    );
        int ar_cnt, r_cnt;
        logic [31:0] rdata_seen;
        logic [1:0]  rresp_seen;
        logic [11:0] rid_seen;
        bit          rdata_captured;
        timed_out      = 0;
        rdata_captured = 0;

        m_axi_gp0_araddr  <= addr;
        m_axi_gp0_arid    <= id;
        m_axi_gp0_arlen   <= 4'h0;
        m_axi_gp0_arsize  <= 3'b010;
        m_axi_gp0_arburst <= 2'b01;
        m_axi_gp0_arlock  <= 2'b00;
        m_axi_gp0_arcache <= 4'h0;
        m_axi_gp0_arprot  <= 3'h0;
        m_axi_gp0_arqos   <= 4'h0;
        m_axi_gp0_arvalid <= 1'b1;

        ar_cnt = 0;
        do begin
            @(posedge clk);
            ar_cnt++;
        end while (!m_axi_gp0_arready && ar_cnt < TIMEOUT_CYCLES);
        m_axi_gp0_arvalid <= 1'b0;

        if (ar_cnt >= TIMEOUT_CYCLES) begin
            timed_out = 1;
            data_out  = 32'hxxxxxxxx;
            rresp_out = 2'bxx;
            rid_out   = 12'hxxx;
            return;
        end

        m_axi_gp0_rready <= 1'b0;
        for (int i = 0; i < rready_delay; i++) begin
            @(posedge clk);
            if (m_axi_gp0_rvalid) begin
                if (!rdata_captured) begin
                    rdata_seen     = m_axi_gp0_rdata;
                    rresp_seen     = m_axi_gp0_rresp;
                    rid_seen       = m_axi_gp0_rid;
                    rdata_captured = 1;
                end else begin
                    check($sformatf("%s: RDATA stable during RREADY hold", tag),
                          m_axi_gp0_rdata, rdata_seen);
                    check_bit($sformatf("%s: RRESP stable during RREADY hold", tag),
                              (m_axi_gp0_rresp == rresp_seen), 1'b1);
                    check($sformatf("%s: RID stable during RREADY hold", tag),
                          {20'h0, m_axi_gp0_rid}, {20'h0, rid_seen});
                end
            end
        end

        m_axi_gp0_rready <= 1'b1;
        r_cnt = 0;
        do begin
            @(posedge clk);
            r_cnt++;
        end while (!m_axi_gp0_rvalid && r_cnt < CDC_WAIT_TIMEOUT);

        if (r_cnt >= CDC_WAIT_TIMEOUT) begin
            timed_out = 1;
            data_out  = 32'hxxxxxxxx;
            rresp_out = 2'bxx;
            rid_out   = 12'hxxx;
        end else begin
            data_out  = m_axi_gp0_rdata;
            rresp_out = m_axi_gp0_rresp;
            rid_out   = m_axi_gp0_rid;
        end
        m_axi_gp0_rready <= 1'b0;
        @(posedge clk);
    endtask

    // Convenience wrappers
    task automatic do_read_and_check(input string tag, input int idx, input logic [11:0] id, input int rready_delay = 0);
        logic [31:0] data;
        logic [1:0]  rresp;
        logic [11:0] rid;
        bit          timed_out;

        axi_read(reg_addr(idx), id, data, rresp, rid, timed_out, rready_delay, tag);
        if (timed_out) begin
            fail_count++;
            $display("[FAIL] t=%0t %-52s read timed out (idx %0d)", $time, tag, idx);
            return;
        end
        check_bit($sformatf("%s: RRESP OKAY", tag), (rresp == 2'b00), 1'b1);
        check($sformatf("%s: RID echoes ARID", tag), {20'h0, rid}, {20'h0, id});
        check($sformatf("%s: RDATA lane %0d", tag, idx), data, {24'h0, shadow[idx]});
    endtask

    task automatic do_write_attempt_and_check_decerr(input string tag, input logic [31:0] addr, input logic [31:0] data);
        logic [1:0]  bresp;
        logic [11:0] bid;
        bit          timed_out;
        logic [511:0] before_regs = dsp_reg_out;

        axi_write(addr, data, 4'hF, 12'hD00, bresp, bid, timed_out, 0, 0, 0, tag);
        if (timed_out) begin
            fail_count++;
            $display("[FAIL] t=%0t %-52s write timed out (should DECERR promptly)", $time, tag);
            return;
        end
        check_bit($sformatf("%s: BRESP DECERR", tag), (bresp == 2'b11), 1'b1);
        check($sformatf("%s: dsp_reg_out untouched [511:480]", tag), dsp_reg_out[511:480], before_regs[511:480]);
        check($sformatf("%s: dsp_reg_out untouched [31:0]",   tag), dsp_reg_out[31:0],   before_regs[31:0]);
    endtask

    // ------------------------------------------------------------------
    // Main test sequence
    // ------------------------------------------------------------------
    initial begin
        $display("=== tb_axi_cdc_status starting ===");

        do_reset();
        check("Reset: dsp_reg_out all zero (low word)",  dsp_reg_out[31:0],  32'h0);
        check("Reset: dsp_reg_out all zero (high word)", dsp_reg_out[511:480], 32'h0);
        for (int i = 0; i < NUM_REGS; i++)
            do_read_and_check($sformatf("Reset: read lane %0d", i), i, 12'h000 + i);

        // ---- Directed: dsp-side write every lane, read back via AXI ----
        $display("=== Directed: dsp writes, AXI reads ===");
        for (int i = 0; i < NUM_REGS; i++) begin
            logic [7:0] pattern = 8'h10 * (i + 1) + i;
            dsp_write(i, pattern);
            do_read_and_check($sformatf("Directed read lane %0d", i), i, 12'h100 + i);
        end

        // ---- AXI write attempts: always DECERR, never touch storage ----
        $display("=== AXI write attempts (always DECERR) section ===");
        do_write_attempt_and_check_decerr("Write attempt lane 0", reg_addr(0), 32'hFFFF_FFFF);
        do_write_attempt_and_check_decerr("Write attempt lane 7", reg_addr(7), 32'h0000_0000);
        for (int i = 0; i < NUM_REGS; i++)
            do_read_and_check($sformatf("Post-write-attempt read lane %0d", i), i, 12'h140 + i);

        // ---- PERIPH_ID mismatch: DECERR promptly on read ----
        $display("=== PERIPH_ID mismatch (DECERR) section ===");
        begin
            logic [31:0] unmapped = {GP0_TOP_BYTE, WRONG_PERIPH_ID, 16'h0000};
            logic [31:0] rdata;
            logic [1:0]  rresp;
            logic [11:0] rid;
            bit          timed_out;

            axi_read(unmapped, 12'hB00, rdata, rresp, rid, timed_out, 0, "Unmapped read");
            if (!timed_out) begin
                check_bit("Unmapped read: RRESP DECERR", (rresp == 2'b11), 1'b1);
            end else begin
                fail_count++;
                $display("[FAIL] t=%0t Unmapped read unexpectedly timed out (should DECERR promptly)", $time);
            end
        end

        // ---- Back-to-back reads, no idle gap forced ----
        $display("=== Back-to-back reads ===");
        for (int i = 0; i < NUM_REGS; i++)
            do_read_and_check($sformatf("Back-to-back read lane %0d", i), i, 12'h200 + i);

        // ---- Concurrent dsp write racing an in-flight AXI read of the
        //      same lane: the read must land on either the old or the
        //      new value, never a torn/corrupted mix. ----
        $display("=== Concurrent dsp-write-vs-AXI-read race section ===");
        for (int n = 0; n < 20; n++) begin
            int idx = n % NUM_REGS;
            logic [7:0] old_val = shadow[idx];
            logic [7:0] new_val = old_val ^ 8'hFF;
            logic [31:0] data;
            logic [1:0]  rresp;
            logic [11:0] rid;
            bit          timed_out;

            fork
                begin
                    axi_read(reg_addr(idx), 12'h300 + n, data, rresp, rid, timed_out, 0, "Race read");
                end
                begin
                    dsp_write(idx, new_val);
                end
            join

            if (!timed_out) begin
                check_bit($sformatf("Race[%0d] lane %0d: RRESP OKAY", n, idx), (rresp == 2'b00), 1'b1);
                check_bit($sformatf("Race[%0d] lane %0d: read is old or new value, not torn", n, idx),
                           (data == {24'h0, old_val}) || (data == {24'h0, new_val}), 1'b1);
            end else begin
                fail_count++;
                $display("[FAIL] t=%0t Race[%0d] lane %0d read timed out", $time, n, idx);
            end
        end
        for (int i = 0; i < NUM_REGS; i++)
            do_read_and_check($sformatf("Post-race read lane %0d", i), i, 12'h340 + i);

        // ---- Randomized stress: interleave dsp writes and AXI reads
        //      across all 64 lanes ----
        $display("=== Randomized stress: %0d iterations ===", NUM_RANDOM_ITER);
        for (int n = 0; n < NUM_RANDOM_ITER; n++) begin
            int         idx      = $urandom_range(0, NUM_REGS-1);
            logic [7:0] data     = $urandom_range(0, 255);
            bit         do_write = $urandom_range(0, 1);

            if (do_write)
                dsp_write(idx, data);
            else
                do_read_and_check($sformatf("Stress[%0d] read lane %0d", n, idx), idx, 12'h400 + (n % 4095));
        end

        // ---- AXI-side backpressure: RREADY delayed on a read ----
        $display("=== AXI backpressure section ===");
        do_read_and_check("Backpressure: RREADY delayed by 5", 3, 12'h900, 5);
        for (int d = 1; d <= 8; d++)
            do_read_and_check($sformatf("Backpressure: RREADY delayed by %0d", d), 4, 12'h910 + d, d);

        // ---- Final sweep ----
        for (int i = 0; i < NUM_REGS; i++)
            do_read_and_check($sformatf("Final sweep lane %0d", i), i, 12'h500 + i);

        // ------------------------------------------------------------------
        $display("=== tb_axi_cdc_status finished: %0d passed, %0d failed ===", pass_count, fail_count);
        if (fail_count == 0)
            $display("=== RESULT: PASS ===");
        else
            $display("=== RESULT: FAIL ===");

        $finish;
    end

endmodule
