`timescale 1ns / 1ps
//
// Self-checking testbench for axi_if + axi_registers (src/axi_if.sv,
// src/axi_registers.sv) cascaded together, i.e. exactly how
// fm_receiver.sv wires them: axi_if turns AXI3 into the wen/ren
// peripheral bus, axi_registers is the one peripheral hanging off it in
// this test (PERIPH_ID = TEST_PERIPH_ID below).
//
// Drives the pair as an AXI3 master (mirroring what PS7's M_AXI_GP0
// actually is) and checks both the write path (via direct reg_out
// observation) and the read path (via AXI RDATA), against a golden
// "shadow" register model tracked in this testbench. Every check is a
// pass/fail assertion — run to completion and read the final summary,
// no waveform inspection required.
//
// Address map assumed (see reg_addr()/reg_out_slice() below if this needs
// changing): addr[23:16] = TEST_PERIPH_ID (axi_registers only reacts on a
// match, else axi_if replies DECERR), addr[4:2] selects one of 8
// word-aligned 32-bit registers, reg_out[31:0] = register 0 through
// reg_out[255:224] = register 7.
//
// Coverage in this version:
//   - AW/W same-cycle and staggered-cycle handshakes (either order, 0-8
//     cycle skew, directed + randomized)
//   - WSTRB byte-lane behavior, including the all-zero no-op case
//   - Response-channel backpressure: BREADY/RREADY withheld for several
//     cycles, with BRESP/BID and RDATA/RRESP/RID checked for stability
//     across the entire hold, not just at the end
//   - Address decode aliasing within the 8-register window, above
//     addr[4:2] (white-box, documents actual behavior rather than
//     asserting an unimplemented finer-grained SLVERR)
//   - PERIPH_ID mismatch -> axi_if replies DECERR promptly (no waiting on
//     a done that will never come), reg_out left untouched
//   - Reset asserted mid-transaction, with recovery verified
//   - A same-clock-edge concurrent read + write to the same register
//     (white-box, synchronizes on axi_if's internal aw_seen/w_seen),
//     documenting which value a same-edge read observes
//
// Scope still not covered: multiple outstanding (pipelined) transactions
// with distinct IDs in flight at once — this slave is intentionally
// non-pipelined (single outstanding transaction), so that's out of scope
// by design rather than a gap.

module tb_axi_fm;

    // ------------------------------------------------------------------
    // Parameters
    // ------------------------------------------------------------------
    localparam int CLK_PERIOD_NS   = 10;   // 100 MHz, matches fclk0
    localparam int NUM_REGS        = 8;
    localparam int TIMEOUT_CYCLES  = 200;
    localparam int NUM_RANDOM_ITER = 200;

    localparam logic [7:0] TEST_PERIPH_ID = 8'h01; // matches axi_registers in fm_receiver.sv
    localparam logic [7:0] WRONG_PERIPH_ID = 8'h02; // an address nobody claims, for DECERR coverage

    // ------------------------------------------------------------------
    // DUT signals — names match axi_fm's ports exactly so `.*` wires
    // everything up automatically.
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

    logic [255:0] reg_out;

    // Peripheral bus between axi_if and axi_registers — named to match
    // both modules' port lists exactly so `.*` wires everything up
    // automatically on both instances below.
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

    // ------------------------------------------------------------------
    // Scoreboard
    // ------------------------------------------------------------------
    int unsigned pass_count = 0;
    int unsigned fail_count = 0;

    // Golden model: what every register SHOULD contain right now.
    logic [31:0] shadow [0:NUM_REGS-1];

    // ------------------------------------------------------------------
    // DUT
    // ------------------------------------------------------------------
    axi_if dut (.*);

    axi_registers #(
        .PERIPH_ID (TEST_PERIPH_ID)
    ) dut_regs (.*);

    // ------------------------------------------------------------------
    // Clock
    // ------------------------------------------------------------------
    initial clk = 0;
    always #(CLK_PERIOD_NS/2) clk = ~clk;

    // ------------------------------------------------------------------
    // Address map helpers (adjust here if the real mapping differs)
    // ------------------------------------------------------------------
    // addr[31:24] is a stand-in for whatever M_AXI_GP0 fixes it to on real
    // hardware (0x40-0x7F) -- deliberately non-zero here so these tests
    // exercise a genuinely reachable address, not one that only works
    // because axi_registers ignores those bits.
    localparam logic [7:0] GP0_TOP_BYTE = 8'h40;

    function automatic logic [31:0] reg_addr(input int idx);
        return {GP0_TOP_BYTE, TEST_PERIPH_ID, 16'h0} | (idx << 2);
    endfunction

    function automatic logic [31:0] reg_out_slice(input logic [255:0] regs, input int idx);
        return regs[idx*32 +: 32];
    endfunction

    // AXI WSTRB byte-lane merge: only byte lanes with their strb bit set
    // are updated, the rest keep their previous value.
    function automatic logic [31:0] apply_wstrb(
        input logic [31:0] old_data,
        input logic [31:0] new_data,
        input logic [3:0]  strb
    );
        logic [31:0] result;
        result = old_data;
        for (int b = 0; b < 4; b++) begin
            if (strb[b]) result[b*8 +: 8] = new_data[b*8 +: 8];
        end
        return result;
    endfunction

    // ------------------------------------------------------------------
    // Checker
    // ------------------------------------------------------------------
    task automatic check(input string name, input logic [31:0] actual, input logic [31:0] expected);
        if (actual === expected) begin
            pass_count++;
            $display("[PASS] %-48s got=0x%08h", name, actual);
        end else begin
            fail_count++;
            $display("[FAIL] %-48s got=0x%08h expected=0x%08h", name, actual, expected);
        end
    endtask

    task automatic check_bit(input string name, input logic actual, input logic expected);
        if (actual === expected) begin
            pass_count++;
            $display("[PASS] %-48s got=%0b", name, actual);
        end else begin
            fail_count++;
            $display("[FAIL] %-48s got=%0b expected=%0b", name, actual, expected);
        end
    endtask

    // ------------------------------------------------------------------
    // Reset
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

        for (int i = 0; i < NUM_REGS; i++) shadow[i] = 32'h0;

        rstb <= 1'b0;
        repeat (5) @(posedge clk);
        rstb <= 1'b1;
        @(posedge clk);
    endtask

    // ------------------------------------------------------------------
    // AXI3 write transaction. AW and W are driven concurrently (as a
    // real master may legally do), each with its own handshake wait, then
    // the B response is awaited. Every wait is timeout-guarded so a stuck
    // DUT fails the transaction instead of hanging the simulation.
    //
    // aw_delay/w_delay: cycles to wait (from task entry) before raising
    // AWVALID/WVALID respectively. A real AXI master is free to stagger
    // these in either order with any gap.
    //
    // bready_delay: cycles to hold BREADY low after AW/W complete before
    // accepting the response. While held low, BVALID must stay asserted
    // and BRESP/BID must stay stable — this is checked every cycle of the
    // hold, not just once.
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
        bit aw_done, w_done;
        logic [1:0]  bresp_seen;
        logic [11:0] bid_seen;
        bit          bresp_captured;
        timed_out      = 0;
        bresp_captured = 0;

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
                w_done = (w_cnt < TIMEOUT_CYCLES);
                m_axi_gp0_wvalid <= 1'b0;
            end
        join

        if (!aw_done || !w_done) begin
            timed_out = 1;
            bresp_out = 2'bxx;
            bid_out   = 12'hxxx;
            return;
        end

        // Hold BREADY low for bready_delay cycles first. AXI requires the
        // slave to keep BVALID/BRESP/BID stable for as long as it takes
        // the master to accept — check that on every cycle of the hold,
        // not just trust it.
        m_axi_gp0_bready <= 1'b0;
        for (int i = 0; i < bready_delay; i++) begin
            @(posedge clk);
            if (m_axi_gp0_bvalid) begin
                if (!bresp_captured) begin
                    bresp_seen     = m_axi_gp0_bresp;
                    bid_seen       = m_axi_gp0_bid;
                    bresp_captured = 1;
                end else begin
                    check_bit($sformatf("%s: BRESP stable during BREADY hold", tag),
                              (m_axi_gp0_bresp == bresp_seen), 1'b1);
                    check_bit($sformatf("%s: BID stable during BREADY hold", tag),
                              (m_axi_gp0_bid == bid_seen), 1'b1);
                end
            end
        end

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

    // ------------------------------------------------------------------
    // AXI3 read transaction. rready_delay mirrors bready_delay above:
    // hold RREADY low for N cycles, checking RDATA/RRESP/RID stay stable
    // throughout while RVALID is asserted.
    // ------------------------------------------------------------------
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
        end while (!m_axi_gp0_rvalid && r_cnt < TIMEOUT_CYCLES);

        if (r_cnt >= TIMEOUT_CYCLES) begin
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

    // Convenience wrappers that also update/check against the shadow model.
    task automatic do_write_and_check(
        input string       tag,
        input int          idx,
        input logic [31:0] data,
        input logic [3:0]  strb,
        input logic [11:0] id,
        input int          aw_delay     = 0,
        input int          w_delay      = 0,
        input int          bready_delay = 0
    );
        logic [1:0]  bresp;
        logic [11:0] bid;
        bit          timed_out;

        axi_write(reg_addr(idx), data, strb, id, bresp, bid, timed_out,
                  aw_delay, w_delay, bready_delay, tag);

        if (timed_out) begin
            fail_count++;
            $display("[FAIL] %-48s write timed out (reg %0d)", tag, idx);
            return;
        end

        shadow[idx] = apply_wstrb(shadow[idx], data, strb);

        check_bit($sformatf("%s: BRESP OKAY",        tag), (bresp == 2'b00), 1'b1);
        check(    $sformatf("%s: BID echoes AWID",    tag), {20'h0, bid},     {20'h0, id});
        check(    $sformatf("%s: reg_out[%0d] after write", tag, idx),
                  reg_out_slice(reg_out, idx), shadow[idx]);
    endtask

    task automatic do_read_and_check(
        input string       tag,
        input int          idx,
        input logic [11:0] id,
        input int          rready_delay = 0
    );
        logic [31:0] data;
        logic [1:0]  rresp;
        logic [11:0] rid;
        bit          timed_out;

        axi_read(reg_addr(idx), id, data, rresp, rid, timed_out, rready_delay, tag);

        if (timed_out) begin
            fail_count++;
            $display("[FAIL] %-48s read timed out (reg %0d)", tag, idx);
            return;
        end

        check_bit($sformatf("%s: RRESP OKAY",     tag), (rresp == 2'b00), 1'b1);
        check(    $sformatf("%s: RID echoes ARID", tag), {20'h0, rid},     {20'h0, id});
        check(    $sformatf("%s: RDATA reg %0d",   tag, idx), data, shadow[idx]);
    endtask

    // ------------------------------------------------------------------
    // Main test sequence
    // ------------------------------------------------------------------
    initial begin
        $display("=== tb_axi_fm starting ===");

        do_reset();
        check("Reset: reg_out all zero", reg_out[31:0], 32'h0);
        for (int i = 0; i < NUM_REGS; i++)
            check($sformatf("Reset: reg_out[%0d]", i), reg_out_slice(reg_out, i), 32'h0);

        // ---- Directed: write a distinct pattern to every register ----
        for (int i = 0; i < NUM_REGS; i++) begin
            logic [31:0] pattern = (32'h1000_0000 * (i + 1)) ^ (32'h1111_1111 * i);
            do_write_and_check($sformatf("Directed write reg %0d", i), i, pattern, 4'hF, 12'h100 + i);
        end

        // ---- Directed: read back every register ----
        for (int i = 0; i < NUM_REGS; i++)
            do_read_and_check($sformatf("Directed read reg %0d", i), i, 12'h200 + i);

        // ---- WSTRB byte-lane test on every register ----
        for (int i = 0; i < NUM_REGS; i++) begin
            do_write_and_check($sformatf("WSTRB baseline reg %0d", i), i, 32'hAABBCCDD, 4'hF, 12'h300 + i);
            do_write_and_check($sformatf("WSTRB byte0-only reg %0d", i), i, 32'h11111111, 4'b0001, 12'h301 + i);
            do_write_and_check($sformatf("WSTRB byte2+3-only reg %0d", i), i, 32'h22223333, 4'b1100, 12'h302 + i);
            do_read_and_check($sformatf("WSTRB verify via read reg %0d", i), i, 12'h303 + i);
        end

        // ---- WSTRB = 0: a legal AXI write with no byte lanes selected —
        //      handshake must still complete (BRESP OKAY), but reg_out
        //      must be bit-for-bit unchanged. ----
        $display("=== WSTRB = 0 no-op section ===");
        for (int i = 0; i < NUM_REGS; i++) begin
            logic [31:0] beforee = reg_out_slice(reg_out, i);
            do_write_and_check($sformatf("WSTRB=0 no-op reg %0d", i), i, 32'hFFFF_FFFF, 4'b0000, 12'h380 + i);
            check($sformatf("WSTRB=0 reg %0d truly unchanged", i), reg_out_slice(reg_out, i), beforee);
        end

        // ---- Back-to-back writes, no idle gap injected between them ----
        for (int i = 0; i < NUM_REGS; i++)
            do_write_and_check($sformatf("Back-to-back write reg %0d", i), i, 32'hDEAD_0000 + i, 4'hF, 12'h400 + i);
        for (int i = 0; i < NUM_REGS; i++)
            do_read_and_check($sformatf("Back-to-back read reg %0d", i), i, 12'h500 + i);

        // ---- Randomized stress ----
        $display("=== Randomized stress: %0d iterations ===", NUM_RANDOM_ITER);
        for (int n = 0; n < NUM_RANDOM_ITER; n++) begin
            int          idx      = $urandom_range(0, NUM_REGS-1);
            logic [31:0] data     = $urandom;
            logic [3:0]  strb     = $urandom_range(1, 15); // never all-zero here; covered above
            logic [11:0] id       = $urandom;
            bit          do_read_back = $urandom_range(0, 1);

            do_write_and_check($sformatf("Stress[%0d] write reg %0d", n, idx), idx, data, strb, id);

            if (do_read_back)
                do_read_and_check($sformatf("Stress[%0d] read reg %0d", n, idx), idx, id);
        end

        // ---- Staggered AW/W: AW and W raised on different cycles instead
        //      of always together. A real AXI master is free to do this in
        //      either order with any gap — a slave that implicitly assumes
        //      they always arrive on the same cycle can pass every test
        //      above and still be broken for this legal case. ----
        $display("=== Staggered AW/W section ===");

        do_write_and_check("Staggered: AW leads W by 5", 0, 32'hA1A1_A1A1, 4'hF, 12'h600, 0, 5);
        do_write_and_check("Staggered: W leads AW by 5", 1, 32'hB2B2_B2B2, 4'hF, 12'h601, 5, 0);
        do_write_and_check("Staggered: AW/W same cycle", 2, 32'hC3C3_C3C3, 4'hF, 12'h602, 0, 0);

        for (int d = 1; d <= 8; d++) begin
            do_write_and_check($sformatf("Staggered: AW leads W by %0d", d),
                                3, 32'hD000_0000 + d, 4'hF, 12'h610 + d, 0, d);
            do_write_and_check($sformatf("Staggered: W leads AW by %0d", d),
                                4, 32'hE000_0000 + d, 4'hF, 12'h620 + d, d, 0);
        end

        $display("=== Randomized staggered stress: %0d iterations ===", NUM_RANDOM_ITER);
        for (int n = 0; n < NUM_RANDOM_ITER; n++) begin
            int          idx          = $urandom_range(0, NUM_REGS-1);
            logic [31:0] data         = $urandom;
            logic [3:0]  strb         = $urandom_range(1, 15);
            logic [11:0] id           = $urandom;
            int          aw_delay     = $urandom_range(0, 8);
            int          w_delay      = $urandom_range(0, 8);
            bit          do_read_back = $urandom_range(0, 1);

            do_write_and_check($sformatf("StaggerStress[%0d] reg %0d aw_d=%0d w_d=%0d",
                                          n, idx, aw_delay, w_delay),
                                idx, data, strb, id, aw_delay, w_delay);

            if (do_read_back)
                do_read_and_check($sformatf("StaggerStress[%0d] read reg %0d", n, idx), idx, id);
        end

        // ---- Response-channel backpressure: BREADY/RREADY withheld for
        //      several cycles. Directed cases first, then randomized,
        //      folded into the same write/read helpers so the stability
        //      checks inside axi_write/axi_read run automatically. ----
        $display("=== Response backpressure section ===");

        do_write_and_check("Backpressure: BREADY delayed by 5", 5, 32'h5A5A_5A5A, 4'hF, 12'h900, 0, 0, 5);
        do_read_and_check("Backpressure: RREADY delayed by 5",  5, 12'h901, 5);

        for (int d = 1; d <= 8; d++) begin
            do_write_and_check($sformatf("Backpressure: BREADY delayed by %0d", d),
                                6, 32'h6000_0000 + d, 4'hF, 12'h910 + d, 0, 0, d);
            do_read_and_check($sformatf("Backpressure: RREADY delayed by %0d", d),
                               6, 12'h920 + d, d);
        end

        $display("=== Randomized backpressure stress: %0d iterations ===", NUM_RANDOM_ITER);
        for (int n = 0; n < NUM_RANDOM_ITER; n++) begin
            int          idx           = $urandom_range(0, NUM_REGS-1);
            logic [31:0] data          = $urandom;
            logic [3:0]  strb          = $urandom_range(1, 15);
            logic [11:0] id            = $urandom;
            int          aw_delay      = $urandom_range(0, 4);
            int          w_delay       = $urandom_range(0, 4);
            int          bready_delay  = $urandom_range(0, 8);
            int          rready_delay  = $urandom_range(0, 8);
            bit          do_read_back  = $urandom_range(0, 1);

            do_write_and_check($sformatf("BackpressureStress[%0d] reg %0d bready_d=%0d",
                                          n, idx, bready_delay),
                                idx, data, strb, id, aw_delay, w_delay, bready_delay);

            if (do_read_back)
                do_read_and_check($sformatf("BackpressureStress[%0d] read reg %0d rready_d=%0d",
                                             n, idx, rready_delay),
                                   idx, id, rready_delay);
        end

        // ---- Address decode aliasing within the 8-register window. This
        //      peripheral decodes with addr[4:2] only — anything set
        //      above bit 4 (but still under the matching PERIPH_ID byte)
        //      is currently NOT range-checked, so it aliases back into
        //      the 8-register window instead of producing a finer-grained
        //      SLVERR. This test documents that actual, current behavior
        //      (known limitation, not yet a bug fix target) so a future
        //      change in decode behavior shows up as a deliberate,
        //      visible diff here instead of silently. ----
        $display("=== Address decode aliasing section (documents current behavior) ===");
        begin
            logic [31:0] alias_pattern_1 = 32'h600D_0001;
            logic [31:0] alias_pattern_2 = 32'h600D_0002;
            logic [1:0]  bresp;
            logic [11:0] bid;
            bit          timed_out;

            // {TEST_PERIPH_ID, 0x20} -> addr[4:2] = 3'b000 -> aliases register 0
            axi_write({GP0_TOP_BYTE, TEST_PERIPH_ID, 16'h0020}, alias_pattern_1, 4'hF, 12'hA00, bresp, bid, timed_out);
            if (!timed_out) begin
                shadow[0] = alias_pattern_1;
                check_bit("Alias +0x20: BRESP OKAY", (bresp == 2'b00), 1'b1);
                check("Alias +0x20 write lands in reg_out[0] (aliasing, documented)",
                      reg_out_slice(reg_out, 0), alias_pattern_1);
            end else begin
                fail_count++;
                $display("[FAIL] Alias +0x20 write timed out");
            end

            // {TEST_PERIPH_ID, 0x100} -> addr[4:2] = 3'b000 -> also aliases register 0
            axi_write({GP0_TOP_BYTE, TEST_PERIPH_ID, 16'h0100}, alias_pattern_2, 4'hF, 12'hA01, bresp, bid, timed_out);
            if (!timed_out) begin
                shadow[0] = alias_pattern_2;
                check_bit("Alias +0x100: BRESP OKAY", (bresp == 2'b00), 1'b1);
                check("Alias +0x100 write lands in reg_out[0] (aliasing, documented)",
                      reg_out_slice(reg_out, 0), alias_pattern_2);
            end else begin
                fail_count++;
                $display("[FAIL] Alias +0x100 write timed out");
            end
        end

        // ---- PERIPH_ID mismatch: an address nobody claims must come
        //      back DECERR promptly (axi_if must not hang waiting on a
        //      done that will never come), and must not touch reg_out. ----
        $display("=== PERIPH_ID mismatch (DECERR) section ===");
        begin
            logic [31:0] unmapped_addr = {GP0_TOP_BYTE, WRONG_PERIPH_ID, 16'h0000};
            logic [31:0] before_reg0   = reg_out_slice(reg_out, 0);
            logic [1:0]  bresp;
            logic [11:0] bid;
            bit          timed_out;
            logic [31:0] rdata;
            logic [1:0]  rresp;
            logic [11:0] rid;
            bit          rtimed_out;

            axi_write(unmapped_addr, 32'hDEAD_BEEF, 4'hF, 12'hB00, bresp, bid, timed_out, 0, 0, 0, "Unmapped write");
            if (!timed_out) begin
                check_bit("Unmapped write: BRESP DECERR", (bresp == 2'b11), 1'b1);
                check("Unmapped write: reg_out[0] untouched", reg_out_slice(reg_out, 0), before_reg0);
            end else begin
                fail_count++;
                $display("[FAIL] Unmapped write unexpectedly timed out (should DECERR promptly, not hang)");
            end

            axi_read(unmapped_addr, 12'hB01, rdata, rresp, rid, rtimed_out, 0, "Unmapped read");
            if (!rtimed_out) begin
                check_bit("Unmapped read: RRESP DECERR", (rresp == 2'b11), 1'b1);
            end else begin
                fail_count++;
                $display("[FAIL] Unmapped read unexpectedly timed out (should DECERR promptly, not hang)");
            end
        end

        // ---- Reset asserted mid-transaction: get the write FSM to
        //      capture AW (aw_done=1) but withhold W, then assert reset,
        //      then confirm clean recovery and that a fresh transaction
        //      works normally afterward. ----
        $display("=== Reset mid-transaction section ===");
        begin
            m_axi_gp0_awaddr  <= reg_addr(0);
            m_axi_gp0_awid    <= 12'h700;
            m_axi_gp0_awlen   <= 4'h0;
            m_axi_gp0_awsize  <= 3'b010;
            m_axi_gp0_awburst <= 2'b01;
            m_axi_gp0_awlock  <= 2'b00;
            m_axi_gp0_awcache <= 4'h0;
            m_axi_gp0_awprot  <= 3'h0;
            m_axi_gp0_awqos   <= 4'h0;
            m_axi_gp0_awvalid <= 1'b1;

            // Wait for the AW handshake only — W is deliberately never
            // driven, leaving the FSM sitting in W_IDLE with aw_done set
            // and w_done clear.
            while (!m_axi_gp0_awready) @(posedge clk);
            @(posedge clk);
            m_axi_gp0_awvalid <= 1'b0;

            repeat (3) @(posedge clk); // sit mid-transaction for a bit

            rstb <= 1'b0;
            repeat (5) @(posedge clk);
            rstb <= 1'b1;
            @(posedge clk);

            check_bit("Reset mid-txn: AWREADY high after recovery", m_axi_gp0_awready, 1'b1);
            check_bit("Reset mid-txn: WREADY high after recovery",  m_axi_gp0_wready,  1'b1);
            check_bit("Reset mid-txn: BVALID low after recovery",   m_axi_gp0_bvalid,  1'b0);

            for (int i = 0; i < NUM_REGS; i++) shadow[i] = 32'h0;
            check("Reset mid-txn: reg_out cleared", reg_out[31:0], 32'h0);
            for (int i = 0; i < NUM_REGS; i++)
                check($sformatf("Reset mid-txn: reg_out[%0d] cleared", i),
                      reg_out_slice(reg_out, i), 32'h0);
        end

        do_write_and_check("Reset mid-txn: fresh write after recovery", 0, 32'hF00D_F00D, 4'hF, 12'h701);
        do_read_and_check("Reset mid-txn: fresh read after recovery",   0, 12'h702);

        // ---- Same-clock-edge concurrent read + write to the same
        //      register. White-box: synchronizes on axi_if's internal
        //      aw_seen/w_seen (both already latched, about to drive
        //      w_state to W_REQ on the next edge) so the AR handshake
        //      lands on that identical posedge -- putting both wen (write)
        //      and ren (read) live on the same cycle one edge later, which
        //      is exactly the scenario axi_registers must resolve itself
        //      now that axi_if no longer arbitrates between them (see
        //      point 3 of the axi_if/axi_registers split). Documents
        //      (rather than merely assumes) that a same-edge read observes
        //      the PRE-write value — NBA semantics evaluate all right-hand
        //      sides against pre-edge state before any left-hand side is
        //      updated, so this is expected, not a race bug. ----
        $display("=== Concurrent same-edge read/write section (white-box) ===");
        begin
            int          idx     = 5;
            logic [31:0] pre_val = shadow[idx];
            logic [31:0] new_val = 32'hCAFE_BABE;
            logic [31:0] read_result;
            logic [1:0]  rresp_tmp;
            logic [11:0] rid_tmp;
            logic [1:0]  bresp_tmp;
            logic [11:0] bid_tmp;
            bit          w_timeout;

            fork
                // Write branch: ordinary same-cycle AW+W write.
                begin
                    axi_write(reg_addr(idx), new_val, 4'hF, 12'h800, bresp_tmp, bid_tmp, w_timeout);
                end
                // Read branch: wait until aw_seen & w_seen are both set
                // (the last W_IDLE cycle before axi_if drives w_state to
                // W_REQ), then fire AR so its handshake lands on that same
                // clock edge — R_IDLE's capture and the W_IDLE->W_REQ
                // transition then land together, putting ren and wen both
                // live on the following cycle.
                begin
                    wait (dut.aw_seen && dut.w_seen);
                    m_axi_gp0_araddr  <= reg_addr(idx);
                    m_axi_gp0_arid    <= 12'h801;
                    m_axi_gp0_arlen   <= 4'h0;
                    m_axi_gp0_arsize  <= 3'b010;
                    m_axi_gp0_arburst <= 2'b01;
                    m_axi_gp0_arlock  <= 2'b00;
                    m_axi_gp0_arcache <= 4'h0;
                    m_axi_gp0_arprot  <= 3'h0;
                    m_axi_gp0_arqos   <= 4'h0;
                    m_axi_gp0_arvalid <= 1'b1;
                    @(posedge clk); // same edge as the write's commit
                    m_axi_gp0_arvalid <= 1'b0;
                    m_axi_gp0_rready  <= 1'b1;
                    // Poll for RVALID rather than assuming a fixed edge
                    // count: axi_if's R_REQ->R_RESP transition schedules
                    // RDATA/RVALID via NBA on the same edge it samples
                    // r_done, so reading RDATA immediately after landing
                    // on THAT edge would see the stale pre-update value
                    // (a real bug this caught: it was picking up leftover
                    // RDATA/RID from the previous read transaction).
                    // Checking after a later edge, once RVALID has had a
                    // full cycle to settle, is what axi_read() already
                    // does correctly -- mirror that here.
                    do @(posedge clk); while (!m_axi_gp0_rvalid);
                    read_result = m_axi_gp0_rdata;
                    rresp_tmp   = m_axi_gp0_rresp;
                    rid_tmp     = m_axi_gp0_rid;
                    m_axi_gp0_rready <= 1'b0;
                end
            join

            shadow[idx] = new_val;

            check($sformatf("Concurrent R/W reg %0d: read sees pre-write value", idx),
                  read_result, pre_val);
            check($sformatf("Concurrent R/W reg %0d: reg_out has new value after", idx),
                  reg_out_slice(reg_out, idx), new_val);
            check_bit($sformatf("Concurrent R/W reg %0d: RRESP OKAY", idx), (rresp_tmp == 2'b00), 1'b1);
            check($sformatf("Concurrent R/W reg %0d: RID echoes ARID", idx), {20'h0, rid_tmp}, {20'h0, 12'h801});
        end

        // ---- Final sweep: confirm every register still holds exactly
        //      what the shadow model expects after the whole run ----
        for (int i = 0; i < NUM_REGS; i++)
            check($sformatf("Final sweep reg_out[%0d]", i), reg_out_slice(reg_out, i), shadow[i]);

        // ------------------------------------------------------------------
        $display("=== tb_axi_fm finished: %0d passed, %0d failed ===", pass_count, fail_count);
        if (fail_count == 0)
            $display("=== RESULT: PASS ===");
        else
            $display("=== RESULT: FAIL ===");

        $finish;
    end

endmodule