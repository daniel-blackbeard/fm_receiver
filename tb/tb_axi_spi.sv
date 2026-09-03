`timescale 1ns / 1ps
//
// Self-checking testbench for axi_if + axi_spi (src/axi_if.sv,
// src/axi_spi.sv) cascaded together, exactly how fm_receiver.sv wires them.
// axi_spi.sv itself is the user's own module to write ([[tired_day_rtl_handoff]]
// in project memory); this testbench exists to establish/verify the contract
// it needs to meet.
//
// axi_spi is an AXI-peripheral-bus-to-SPI-master bridge that lets the CPU
// launch a single, fixed-length AD9361 SPI transaction: a 16-bit
// instruction word followed by exactly one 8-bit data byte, 24 SPI_CLK
// edges total, always (see ad9361_registers.md) — no bursts, no variable
// byte counts, one request in flight at a time.
//
// SPI-side timing is fixed by the chip, taken from the AD9361 Reference
// Manual (UG-570): SPI_ENB (chip select) is active low, asserted before
// the first SPI_CLK edge and held through the last; SPI_CLK idles low;
// data is launched on the rising edge and sampled on the falling edge by
// both sides (SPI Mode 1: CPOL=0, CPHA=1); MSB-first; max 50 MHz.
//
// Peripheral-bus contract — a *direct* model, no staged CTRL/ADDR/WDATA/
// STATUS registers: axi_if's wen/ren is a level held for the whole
// transaction (see src/axi_if.sv's port-list comment — W_REQ/R_REQ drive
// wen/ren continuously until w_done/r_done comes back), so a peripheral
// is free to just sit on it for as long as the real work takes. Here:
//   - A single AXI WRITE launches one AD9361 SPI write. addr[23:16] must
//     match TEST_PERIPH_ID; addr[11:2] is the AD9361 register address
//     (addr[15:12] don't-care, addr[1:0] always 0 -- shifted up by 2 so
//     the address stays word-aligned regardless of the AD9361 register
//     number, which isn't restricted to multiples of 4; this was a real
//     bug caught on real hardware: an unshifted addr[9:0] placement
//     produces an unaligned pointer for most registers, which ARM
//     faults on before the access ever reaches the AXI bus at all).
//     w_data[7:0] is the byte to write. w_done is held low until the
//     full 24-edge SPI transaction actually completes on the wire.
//   - A single AXI READ launches one AD9361 SPI read the same way, using
//     r_addr[11:2] directly. r_done is held low until the real SPI read
//     completes, and r_data[7:0] is the captured byte.
// No polling, no launch bit, no status register — the AXI transaction
// itself blocks (via axi_if's unconditional wait on done) for as long as
// the SPI side takes.
//
// Coverage in this version:
//   - Directed single-register read/write round trips, including the
//     address-space and data-space boundaries (0x000/0x3FF, 0x00/0xFF)
//   - Every observed SPI transaction checked against the expected 16-bit
//     instruction word (R/W bit, the N[2:0] byte-count field pinned at
//     000, the unused [11:10] field pinned at 00, the 10-bit address)
//   - Exactly 24 total SPI_CLK edges per transaction, counted
//     independently of the phase-based bit sampling below, so this
//     verifies what the DUT actually toggled rather than how many edges
//     this BFM's own loop chose to wait for
//   - SPI_CLK idle-low framing immediately before/after each transaction
//   - Back-to-back requests with no idle gap forced between them
//   - Randomized stress across the full 10-bit address / 8-bit data space
//   - AXI-side backpressure (BREADY/RREADY delayed) directly on the write
//     and read transactions themselves — since axi_if blocks on done
//     regardless, this also exercises holding the bus for a full ~100
//     cycle real transaction
//   - PERIPH_ID mismatch -> DECERR promptly, no SPI activity triggered
//
// Scope not covered: multi-byte bursts (N>1) — out of scope by design,
// this bridge only ever does 1-byte transfers.

module tb_axi_spi;

    // ------------------------------------------------------------------
    // Parameters
    // ------------------------------------------------------------------
    localparam int CLK_PERIOD_NS   = 10;    // 100 MHz, matches fclk0
    localparam int TIMEOUT_CYCLES  = 200;   // AXI handshake guard
    localparam int SPI_WAIT_TIMEOUT = 5000; // cycles to wait for the SPI
                                             // side to see a transaction
    localparam int NUM_RANDOM_ITER = 200;

    localparam logic [7:0] TEST_PERIPH_ID  = 8'h02; // reserved for axi_spi in fm_receiver.sv
    localparam logic [7:0] WRONG_PERIPH_ID = 8'h01; // axi_registers' ID — nobody here claims it

    // Peripheral-bus address for a given AD9361 SPI register: PERIPH_ID
    // in addr[23:16], the 10-bit SPI address passed straight through in
    // the low bits, everything in between don't-care (zeroed here).
    // addr[31:24] stands in for whatever M_AXI_GP0 fixes it to on real
    // hardware (0x40-0x7F) -- non-zero here so this exercises a genuinely
    // reachable address, not one that only works because axi_spi ignores
    // those bits.
    localparam logic [7:0] GP0_TOP_BYTE = 8'h40;

    // a lands at addr[11:2], not addr[9:0] directly -- shifted up by 2 so
    // the resulting address is always word-aligned regardless of a's
    // value (AD9361 register numbers are arbitrary, not restricted to
    // multiples of 4; a real ARM store to an unaligned Strongly-Ordered
    // address is an immediate Alignment Fault, caught on real hardware).
    function automatic logic [31:0] spi_addr(input logic [9:0] a);
        return {GP0_TOP_BYTE, TEST_PERIPH_ID, 4'h0, a, 2'b00};
    endfunction

    // ------------------------------------------------------------------
    // DUT signals — names match axi_if's and axi_spi's ports exactly so
    // `.*` wires everything up automatically on both instances below.
    // ------------------------------------------------------------------
    logic clk;
    logic rstb;

    logic spi_miso;
    logic spi_mosi;
    logic spi_ce;
    logic spi_ck;

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

    // Peripheral bus between axi_if and axi_spi — same contract as
    // axi_registers uses (see src/axi_if.sv's port-list comment).
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

    // ------------------------------------------------------------------
    // DUT
    // ------------------------------------------------------------------
    axi_if dut (.*);

    axi_spi #(
        .PERIPH_ID (TEST_PERIPH_ID)
    ) dut_spi (.*);

    // ------------------------------------------------------------------
    // Clock
    // ------------------------------------------------------------------
    initial clk = 0;
    always #(CLK_PERIOD_NS/2) clk = ~clk;

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
    // AD9361 SPI slave model (BFM): a 1024x8 shadow register file behind
    // the spi_ce/spi_ck/spi_mosi/spi_miso pins. Phase-based sampling
    // below follows SPI Mode 1 (CPOL=0, CPHA=1): the master (DUT) is
    // expected to change MOSI on the rising edge and this model samples
    // it on the falling edge; for a read, this model drives MISO on the
    // rising edge for the DUT to sample on the falling edge — matching
    // UG-570's "launched on the rising edge, sampled on the falling edge
    // by both the BBP and the AD9361."
    //
    // Independent of that phase-based loop, a free-running counter
    // further below tallies every completed SPI_CLK cycle (falling
    // edges) seen while CS is low, so the "exactly 24 clocks" contract
    // is checked against what the DUT actually toggled on the wire,
    // not against how many edges this model's own loop happened to
    // wait for.
    // ------------------------------------------------------------------
    logic [7:0] ad9361_shadow [0:1023];

    logic        last_rw;
    logic [2:0]  last_n_field;
    logic [1:0]  last_unused_field;
    logic [9:0]  last_addr;
    logic [7:0]  last_data;
    bit          last_ck_idle_ok;
    int unsigned last_edges;
    int unsigned spi_txn_seen_count = 0;

    // Counts falling edges only (= completed clock cycles), so a
    // correct 24-clock transaction reads 24, not 48 — counting both
    // edges of each clock would double-count every cycle.
    int unsigned spi_edge_count = 0;
    always @(negedge spi_ce) spi_edge_count = 0;
    always @(negedge spi_ck) if (!spi_ce) spi_edge_count++;

    initial begin
        spi_miso = 1'b0;
        forever begin
            automatic logic [15:0] instr;
            automatic logic [7:0]  data_byte;
            automatic bit          idle_ok;

            @(negedge spi_ce);
            idle_ok = (spi_ck === 1'b0);

            instr = '0;
            for (int b = 0; b < 16; b++) begin
                @(negedge spi_ck);
                instr = {instr[14:0], spi_mosi};
            end

            if (instr[15]) begin
                // Write: master drives MOSI, sampled on falling edges.
                data_byte = '0;
                for (int b = 0; b < 8; b++) begin
                    @(negedge spi_ck);
                    data_byte = {data_byte[6:0], spi_mosi};
                end
                ad9361_shadow[instr[9:0]] = data_byte;
            end else begin
                // Read: this model drives MISO, changing on rising edges.
                data_byte = ad9361_shadow[instr[9:0]];
                for (int b = 7; b >= 0; b--) begin
                    @(posedge spi_ck);
                    spi_miso <= data_byte[b];
                end
            end

            @(posedge spi_ce);
            spi_miso <= 1'b0;

            last_rw          = instr[15];
            last_n_field      = instr[14:12];
            last_unused_field = instr[11:10];
            last_addr         = instr[9:0];
            last_data         = data_byte;
            last_ck_idle_ok   = idle_ok && (spi_ck === 1'b0);
            last_edges        = spi_edge_count;
            spi_txn_seen_count++;
        end
    end

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

        for (int i = 0; i < 1024; i++) ad9361_shadow[i] = 8'h00;

        rstb <= 1'b0;
        repeat (5) @(posedge clk);
        rstb <= 1'b1;
        @(posedge clk);
    endtask

    // ------------------------------------------------------------------
    // AXI3 write transaction — identical in structure to tb_axi_fm's,
    // since axi_spi's AXI-facing port list is the same AXI3 slave
    // interface. AW and W driven concurrently, B awaited; every wait is
    // timeout-guarded. Note BRESP only arrives once axi_spi's w_done
    // asserts, which for a matched address doesn't happen until the real
    // 24-edge SPI transaction completes — so a successful call here has
    // already waited out the whole SPI transfer.
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

        m_axi_gp0_bready <= 1'b0;
        repeat (bready_delay) @(posedge clk);

        m_axi_gp0_bready <= 1'b1;
        b_cnt = 0;
        do begin
            @(posedge clk);
            b_cnt++;
        end while (!m_axi_gp0_bvalid && b_cnt < SPI_WAIT_TIMEOUT);

        if (b_cnt >= SPI_WAIT_TIMEOUT) begin
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
    // AXI3 read transaction — same structure as tb_axi_fm's axi_read.
    // Same note as axi_write above: RVALID doesn't arrive for a matched
    // address until the real SPI read transaction completes.
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
        timed_out = 0;

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
        repeat (rready_delay) @(posedge clk);

        m_axi_gp0_rready <= 1'b1;
        r_cnt = 0;
        do begin
            @(posedge clk);
            r_cnt++;
        end while (!m_axi_gp0_rvalid && r_cnt < SPI_WAIT_TIMEOUT);

        if (r_cnt >= SPI_WAIT_TIMEOUT) begin
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

    // ------------------------------------------------------------------
    // One AXI write == one complete AD9361 SPI write, checked both at
    // the AXI level (BRESP) and black-box on the SPI pins (via the BFM's
    // last_* capture and the shadow register file).
    // ------------------------------------------------------------------
    task automatic do_spi_write_and_check(input string tag, input logic [9:0] addr, input logic [7:0] data);
        logic [1:0]  bresp;
        logic [11:0] bid;
        bit          timed_out;
        int unsigned seen_before;

        seen_before = spi_txn_seen_count;
        axi_write(spi_addr(addr), {24'h0, data}, 4'hF, 12'h010, bresp, bid, timed_out, 0, 0, 0, tag);

        if (timed_out) begin
            fail_count++;
            $display("[FAIL] t=%0t %-52s write timed out (addr 0x%03h)", $time, tag, addr);
            return;
        end

        check_bit($sformatf("%s: BRESP OKAY",               tag), (bresp == 2'b00), 1'b1);
        check(    $sformatf("%s: SPI txn count advanced",   tag), {24'h0, spi_txn_seen_count}, {24'h0, seen_before + 1});
        check_bit($sformatf("%s: SPI saw RW=write",         tag), last_rw, 1'b1);
        check_bit($sformatf("%s: N field is 000 (1 byte)",  tag), (last_n_field == 3'b000), 1'b1);
        check_bit($sformatf("%s: unused field is 00",       tag), (last_unused_field == 2'b00), 1'b1);
        check(    $sformatf("%s: SPI saw ADDR",              tag), {22'h0, last_addr}, {22'h0, addr});
        check(    $sformatf("%s: SPI saw WDATA",             tag), {24'h0, last_data}, {24'h0, data});
        check(    $sformatf("%s: exactly 24 SPI_CLK edges",  tag), last_edges, 32'd24);
        check_bit($sformatf("%s: SPI_CLK idle-low framing",  tag), last_ck_idle_ok, 1'b1);
        check(    $sformatf("%s: shadow register updated",   tag), {24'h0, ad9361_shadow[addr]}, {24'h0, data});
    endtask

    // ------------------------------------------------------------------
    // One AXI read == one complete AD9361 SPI read.
    // ------------------------------------------------------------------
    task automatic do_spi_read_and_check(input string tag, input logic [9:0] addr);
        logic [31:0] data_out;
        logic [1:0]  rresp;
        logic [11:0] rid;
        bit          timed_out;
        logic [7:0]  expected;
        int unsigned seen_before;

        expected    = ad9361_shadow[addr];
        seen_before = spi_txn_seen_count;

        axi_read(spi_addr(addr), 12'h013, data_out, rresp, rid, timed_out, 0, tag);

        if (timed_out) begin
            fail_count++;
            $display("[FAIL] t=%0t %-52s read timed out (addr 0x%03h)", $time, tag, addr);
            return;
        end

        check_bit($sformatf("%s: RRESP OKAY",                   tag), (rresp == 2'b00), 1'b1);
        check(    $sformatf("%s: SPI txn count advanced",       tag), {24'h0, spi_txn_seen_count}, {24'h0, seen_before + 1});
        check_bit($sformatf("%s: SPI saw RW=read",              tag), last_rw, 1'b0);
        check_bit($sformatf("%s: N field is 000 (1 byte)",      tag), (last_n_field == 3'b000), 1'b1);
        check_bit($sformatf("%s: unused field is 00",           tag), (last_unused_field == 2'b00), 1'b1);
        check(    $sformatf("%s: SPI saw ADDR",                  tag), {22'h0, last_addr}, {22'h0, addr});
        check(    $sformatf("%s: exactly 24 SPI_CLK edges",      tag), last_edges, 32'd24);
        check_bit($sformatf("%s: SPI_CLK idle-low framing",      tag), last_ck_idle_ok, 1'b1);
        check(    $sformatf("%s: AXI read data matches shadow",  tag), {24'h0, data_out[7:0]}, {24'h0, expected});
    endtask

    // ------------------------------------------------------------------
    // Main test sequence
    // ------------------------------------------------------------------
    initial begin
        $display("=== tb_axi_spi starting ===");

        do_reset();

        check_bit("Reset: SPI_ENB idle high (deasserted)", spi_ce, 1'b1);
        check_bit("Reset: SPI_CLK idle low",                spi_ck, 1'b0);
        check(    "Reset: no SPI transaction yet", {24'h0, spi_txn_seen_count}, 32'h0);

        // ---- Directed: quick-reference bring-up registers from
        //      ad9361_registers.md, write then read back ----
        $display("=== Directed: quick-reference bring-up registers ===");
        do_spi_write_and_check("Write REG_SPI_CONF (0x000)",   10'h000, 8'h00);
        do_spi_write_and_check("Write REG_PRODUCT_ID-ish (0x037)", 10'h037, 8'hA8);
        do_spi_write_and_check("Write REG_STATE (0x017)",      10'h017, 8'h05);
        do_spi_write_and_check("Write REG_ENSM_MODE (0x013)",  10'h013, 8'h01);
        do_spi_read_and_check ("Read back REG_SPI_CONF (0x000)",   10'h000);
        do_spi_read_and_check ("Read back REG_PRODUCT_ID-ish (0x037)", 10'h037);
        do_spi_read_and_check ("Read back REG_STATE (0x017)",  10'h017);
        do_spi_read_and_check ("Read back REG_ENSM_MODE (0x013)", 10'h013);

        // ---- Directed: address-space boundaries ----
        $display("=== Directed: address boundaries ===");
        do_spi_write_and_check("Write addr 0x000 (lowest)",  10'h000, 8'h11);
        do_spi_read_and_check ("Read addr 0x000 (lowest)",   10'h000);
        do_spi_write_and_check("Write addr 0x3FF (highest)", 10'h3FF, 8'hEE);
        do_spi_read_and_check ("Read addr 0x3FF (highest)",  10'h3FF);

        // ---- Directed: data-space boundaries ----
        do_spi_write_and_check("Write data 0x00", 10'h100, 8'h00);
        do_spi_read_and_check ("Read data 0x00",  10'h100);
        do_spi_write_and_check("Write data 0xFF", 10'h101, 8'hFF);
        do_spi_read_and_check ("Read data 0xFF",  10'h101);

        // ---- Back-to-back requests, no idle gap forced between them ----
        $display("=== Back-to-back requests ===");
        for (int i = 0; i < 8; i++)
            do_spi_write_and_check($sformatf("Back-to-back write %0d", i), 10'h200 + i, 8'h10 + i);
        for (int i = 0; i < 8; i++)
            do_spi_read_and_check($sformatf("Back-to-back read %0d", i), 10'h200 + i);

        // ---- Randomized stress across the full address/data space ----
        $display("=== Randomized stress: %0d iterations ===", NUM_RANDOM_ITER);
        for (int n = 0; n < NUM_RANDOM_ITER; n++) begin
            logic [9:0] addr     = $urandom_range(0, 1023);
            logic [7:0] data     = $urandom_range(0, 255);
            bit         do_write = $urandom_range(0, 1);

            if (do_write)
                do_spi_write_and_check($sformatf("Stress[%0d] write", n), addr, data);
            else
                do_spi_read_and_check($sformatf("Stress[%0d] read", n), addr);
        end

        // ---- AXI-side backpressure, directly on the write/read
        //      transaction itself — axi_if already blocks on done
        //      regardless, so this also exercises holding BVALID/RVALID
        //      off across a full ~100-cycle real SPI transaction ----
        $display("=== AXI backpressure ===");
        begin
            logic [1:0]  bresp;
            logic [11:0] bid;
            bit          wto;
            logic [31:0] data_out;
            logic [1:0]  rresp;
            logic [11:0] rid;
            bit          rto;
            int unsigned seen_before;

            seen_before = spi_txn_seen_count;
            axi_write(spi_addr(10'h150), {24'h0, 8'h5A}, 4'hF, 12'h020, bresp, bid, wto, 0, 0, 5, "Backpressure: write BREADY delay 5");
            if (!wto) begin
                check_bit("Backpressure: BRESP OKAY",             (bresp == 2'b00), 1'b1);
                check(    "Backpressure: write landed in shadow", {24'h0, ad9361_shadow[10'h150]}, {24'h0, 8'h5A});
                check(    "Backpressure: SPI txn count advanced", {24'h0, spi_txn_seen_count}, {24'h0, seen_before + 1});
            end else begin
                fail_count++;
                $display("[FAIL] t=%0t Backpressure write timed out", $time);
            end

            seen_before = spi_txn_seen_count;
            axi_read(spi_addr(10'h150), 12'h021, data_out, rresp, rid, rto, 5, "Backpressure: read RREADY delay 5");
            if (!rto) begin
                check_bit("Backpressure: RRESP OKAY",           (rresp == 2'b00), 1'b1);
                check(    "Backpressure: RDATA matches shadow", {24'h0, data_out[7:0]}, {24'h0, ad9361_shadow[10'h150]});
                check(    "Backpressure: SPI txn count advanced", {24'h0, spi_txn_seen_count}, {24'h0, seen_before + 1});
            end else begin
                fail_count++;
                $display("[FAIL] t=%0t Backpressure read timed out", $time);
            end
        end

        // ---- PERIPH_ID mismatch: an address nobody here claims (axi_spi
        //      is TEST_PERIPH_ID, this uses axi_registers' ID instead)
        //      must come back DECERR promptly and must not touch the SPI
        //      pins at all. ----
        $display("=== PERIPH_ID mismatch (DECERR) section ===");
        begin
            logic [31:0] unmapped_addr = {GP0_TOP_BYTE, WRONG_PERIPH_ID, 16'h0000};
            logic [1:0]  bresp;
            logic [11:0] bid;
            bit          timed_out;
            int unsigned seen_before = spi_txn_seen_count;

            axi_write(unmapped_addr, 32'h0000_0003, 4'hF, 12'hC00, bresp, bid, timed_out, 0, 0, 0, "Unmapped write");
            if (!timed_out) begin
                check_bit("Unmapped write: BRESP DECERR", (bresp == 2'b11), 1'b1);
            end else begin
                fail_count++;
                $display("[FAIL] t=%0t Unmapped write unexpectedly timed out (should DECERR promptly, not hang)", $time);
            end

            check("Unmapped write: no SPI transaction triggered",
                  {24'h0, spi_txn_seen_count}, {24'h0, seen_before});
        end

        // ------------------------------------------------------------------
        $display("=== tb_axi_spi finished: %0d passed, %0d failed ===", pass_count, fail_count);
        if (fail_count == 0)
            $display("=== RESULT: PASS ===");
        else
            $display("=== RESULT: FAIL ===");

        $finish;
    end

endmodule
