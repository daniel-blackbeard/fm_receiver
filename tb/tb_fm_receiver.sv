`timescale 1ns / 1ps
//
// Integration testbench for the PL-side peripheral cluster fm_receiver.sv
// wires up: axi_if + axi_registers (PERIPH_ID 8'h01) + axi_spi (PERIPH_ID
// 8'h02) + axi_cdc_status (PERIPH_ID 8'h03), sharing one broadcast wen/ren
// bus with the OR-dones/AND-no_addrs/mux-rdata combining logic
// fm_receiver.sv itself uses (see src/fm_receiver.sv around
// u_axi_if/u_axi_registers/u_axi_spi/u_axi_cdc_status and src/axi_if.sv's
// port-list comment for the contract). axi_cdc_status also needs its own
// dsp_clk domain driven (deliberately at a non-integer-multiple ratio to
// fclk0, same reasoning as tb_axi_cdc_status.sv) since its storage lives
// there, crossing to AXI via a request/ack handshake -- see
// src/axi_cdc_status.sv's header comment.
//
// Deliberately does NOT instantiate fm_receiver (the top module) or
// design_1_wrapper (the PS7 block design) underneath it: M_AXI_GP0 is
// driven entirely by the PS7 hard macro, which never spontaneously issues
// AXI transactions in plain RTL simulation without real ARM software
// running via a full cosim bridge -- nothing here could inject fake master
// traffic onto it without hierarchically forcing the PS7's own output
// ports, which pulls in Xilinx's encrypted PS7 sim models for no real
// benefit. This testbench instead reproduces exactly the peripheral
// wiring fm_receiver.sv does (same modules, same combining logic) and
// drives it directly as an AXI3 master -- that wiring is the actual thing
// this level needs to verify, since axi_if+axi_registers and
// axi_if+axi_spi are each already thoroughly covered individually by
// tb_axi_fm.sv and tb_axi_spi.sv.
//
// Coverage in this version -- intentionally scoped to what's NEW at this
// integration level, not re-covering per-peripheral edge cases (WSTRB,
// AW/W staggering, backpressure stability, reset-mid-transaction, etc.)
// already exhaustively tested standalone in tb_axi_fm.sv/tb_axi_spi.sv:
//   - Smoke coverage that each peripheral still works normally with the
//     other one also present on the shared bus
//   - Cross-peripheral isolation: a request to one peripheral must not
//     touch the other's state (reg_out untouched by SPI traffic and vice
//     versa), and must not leak the other's stale r_data through the mux
//   - Interleaved back-and-forth access, stressing the combining logic
//     switching contexts every transaction
//   - An address matching NONE of the three peripherals' PERIPH_IDs ->
//     DECERR promptly on both write and read, with none of them reacting
//     -- this is the real AND-of-no_addr test: each individual testbench
//     only ever had one real peripheral in the mix, so that reduces
//     trivially there
//   - axi_cdc_status smoke coverage through the shared bus: dsp-side
//     writes read back correctly via AXI, AXI write attempts always
//     DECERR (it's read-only from that side) and never touch dsp_reg_out
//     or the other two peripherals' state
//
// See [[cdc_status_axi_architecture_plan]] in project memory for the
// broader CDC-status peripheral context.

module tb_fm_receiver;

    // ------------------------------------------------------------------
    // Parameters
    // ------------------------------------------------------------------
    localparam int CLK_PERIOD_NS    = 10;   // 100 MHz, matches fclk0
    localparam int NUM_REGS         = 8;
    localparam int NUM_CDC_REGS     = 64;
    localparam int TIMEOUT_CYCLES   = 200;
    localparam int SPI_WAIT_TIMEOUT = 5000; // a real SPI txn takes ~96 clk cycles

    localparam logic [7:0] REGS_PERIPH_ID = 8'h01;
    localparam logic [7:0] SPI_PERIPH_ID  = 8'h02;
    localparam logic [7:0] CDC_PERIPH_ID  = 8'h03;
    localparam logic [7:0] UNMAPPED_PERIPH_ID = 8'h04; // nobody claims this one
    localparam int DSP_CLK_PERIOD_NS = 7; // deliberately non-integer-multiple of CLK_PERIOD_NS

    // ------------------------------------------------------------------
    // AXI3 master signals (this testbench IS M_AXI_GP0)
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

    // ------------------------------------------------------------------
    // Shared peripheral bus (fanned out to both peripherals) -- same
    // literal port names as axi_if.sv/axi_registers.sv/axi_spi.sv so `.*`
    // wires the broadcast side automatically on every instance below.
    // ------------------------------------------------------------------
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

    // Per-peripheral done/no_addr/rdata -- each peripheral drives its own,
    // combined into the shared w_done/w_no_addr/r_done/r_data/r_no_addr
    // above via the same strategy fm_receiver.sv uses.
    logic        regs_wdone, regs_wnoaddr, regs_rdone, regs_rnoaddr;
    logic [31:0] regs_rdata;

    logic        spi_wdone, spi_wnoaddr, spi_rdone, spi_rnoaddr;
    logic [31:0] spi_rdata;

    logic        cdc_wdone, cdc_wnoaddr, cdc_rdone, cdc_rnoaddr;
    logic [31:0] cdc_rdata;

    assign w_done    = regs_wdone   | spi_wdone   | cdc_wdone;
    assign w_no_addr  = regs_wnoaddr & spi_wnoaddr & cdc_wnoaddr;
    assign r_done    = regs_rdone   | spi_rdone   | cdc_rdone;
    assign r_no_addr  = regs_rnoaddr & spi_rnoaddr & cdc_rnoaddr;
    assign r_data    = regs_rdone ? regs_rdata : (spi_rdone ? spi_rdata : cdc_rdata);

    logic [255:0] reg_out;

    logic spi_miso;
    logic spi_mosi;
    logic spi_ce;
    logic spi_ck;

    logic         dsp_clk;
    logic         dsp_rstb;
    logic         dsp_wen;
    logic [5:0]   dsp_windex;
    logic [7:0]   dsp_wdata;
    logic [511:0] dsp_reg_out;
    logic [7:0]   dsp_shadow [0:NUM_CDC_REGS-1];

    // ------------------------------------------------------------------
    // Scoreboard
    // ------------------------------------------------------------------
    int unsigned pass_count = 0;
    int unsigned fail_count = 0;

    // Golden models
    logic [31:0] regs_shadow [0:NUM_REGS-1];

    // ------------------------------------------------------------------
    // DUT: the same peripheral cluster fm_receiver.sv instantiates
    // ------------------------------------------------------------------
    axi_if dut (.*);

    axi_registers #(
        .PERIPH_ID (REGS_PERIPH_ID)
    ) dut_regs (
        .*,
        .w_done    (regs_wdone),
        .w_no_addr (regs_wnoaddr),
        .r_done    (regs_rdone),
        .r_data    (regs_rdata),
        .r_no_addr (regs_rnoaddr),
        .reg_out   (reg_out)
    );

    axi_spi #(
        .PERIPH_ID (SPI_PERIPH_ID)
    ) dut_spi (
        .*,
        .w_done    (spi_wdone),
        .w_no_addr (spi_wnoaddr),
        .r_done    (spi_rdone),
        .r_data    (spi_rdata),
        .r_no_addr (spi_rnoaddr)
    );

    axi_cdc_status #(
        .PERIPH_ID (CDC_PERIPH_ID)
    ) dut_cdc (
        .*,
        .w_done    (cdc_wdone),
        .w_no_addr (cdc_wnoaddr),
        .r_done    (cdc_rdone),
        .r_data    (cdc_rdata),
        .r_no_addr (cdc_rnoaddr)
    );

    // ------------------------------------------------------------------
    // Clocks — fclk0 and dsp_clk deliberately at a non-integer-multiple
    // ratio, same reasoning as tb_axi_cdc_status.sv: exercises every
    // edge-alignment phase rather than getting lucky on one fixed
    // relationship.
    // ------------------------------------------------------------------
    initial clk = 0;
    always #(CLK_PERIOD_NS/2) clk = ~clk;

    initial dsp_clk = 0;
    always #(DSP_CLK_PERIOD_NS/2.0) dsp_clk = ~dsp_clk;

    // ------------------------------------------------------------------
    // Address helpers. addr[31:24] stands in for whatever M_AXI_GP0 fixes
    // it to on real hardware (0x40-0x7F) -- non-zero here so these tests
    // exercise genuinely reachable addresses, not ones that only work
    // because the peripherals ignore those bits. PERIPH_ID lives in
    // addr[23:16]; each peripheral's own internal address is addr[15:0].
    // ------------------------------------------------------------------
    localparam logic [7:0] GP0_TOP_BYTE = 8'h40;

    function automatic logic [31:0] reg_addr(input int idx);
        return {GP0_TOP_BYTE, REGS_PERIPH_ID, 16'h0} | (idx << 2);
    endfunction

    function automatic logic [31:0] reg_out_slice(input logic [255:0] regs, input int idx);
        return regs[idx*32 +: 32];
    endfunction

    // a lands at addr[11:2], not addr[9:0] directly -- see
    // tb_axi_spi.sv's spi_addr() comment for why (word-alignment).
    function automatic logic [31:0] spi_addr(input logic [9:0] a);
        return {GP0_TOP_BYTE, SPI_PERIPH_ID, 4'h0, a, 2'b00};
    endfunction

    function automatic logic [31:0] cdc_addr(input int idx);
        return {GP0_TOP_BYTE, CDC_PERIPH_ID, 16'h0} | (idx << 2);
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
    // AD9361 SPI slave model (BFM) -- same shape as tb_axi_spi.sv's: a
    // 1024x8 shadow register file behind spi_ce/spi_ck/spi_mosi/spi_miso,
    // SPI Mode 1 (CPOL=0, CPHA=1), MSB-first, 16-bit instruction + 1 data
    // byte = 24 SPI_CLK edges always.
    // ------------------------------------------------------------------
    logic [7:0] ad9361_shadow [0:1023];

    logic        last_rw;
    logic [9:0]  last_addr;
    logic [7:0]  last_data;
    bit          last_ck_idle_ok;
    int unsigned last_edges;
    int unsigned spi_txn_seen_count = 0;

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
                data_byte = '0;
                for (int b = 0; b < 8; b++) begin
                    @(negedge spi_ck);
                    data_byte = {data_byte[6:0], spi_mosi};
                end
                ad9361_shadow[instr[9:0]] = data_byte;
            end else begin
                data_byte = ad9361_shadow[instr[9:0]];
                for (int b = 7; b >= 0; b--) begin
                    @(posedge spi_ck);
                    spi_miso <= data_byte[b];
                end
            end

            @(posedge spi_ce);
            spi_miso <= 1'b0;

            last_rw         = instr[15];
            last_addr       = instr[9:0];
            last_data       = data_byte;
            last_ck_idle_ok = idle_ok && (spi_ck === 1'b0);
            last_edges      = spi_edge_count;
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

        dsp_wen    <= 1'b0;
        dsp_windex <= '0;
        dsp_wdata  <= '0;

        for (int i = 0; i < NUM_REGS; i++) regs_shadow[i] = 32'h0;
        for (int i = 0; i < 1024; i++) ad9361_shadow[i] = 8'h00;
        for (int i = 0; i < NUM_CDC_REGS; i++) dsp_shadow[i] = 8'h0;

        rstb     <= 1'b0;
        dsp_rstb <= 1'b0;
        repeat (5) @(posedge clk);
        rstb <= 1'b1;
        repeat (5) @(posedge dsp_clk);
        dsp_rstb <= 1'b1;
        @(posedge clk);
    endtask

    // dsp-side write: native domain, no handshake needed.
    task automatic dsp_write(input int idx, input logic [7:0] data);
        @(posedge dsp_clk);
        dsp_wen    <= 1'b1;
        dsp_windex <= idx[5:0];
        dsp_wdata  <= data;
        @(posedge dsp_clk);
        dsp_wen <= 1'b0;
        dsp_shadow[idx] = data;
    endtask

    // ------------------------------------------------------------------
    // AXI3 write/read transactions -- same structure as tb_axi_fm.sv's,
    // reused as-is since both peripherals sit behind the identical AXI3
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
    // Convenience wrappers
    // ------------------------------------------------------------------
    task automatic do_regs_write_and_check(input string tag, input int idx, input logic [31:0] data);
        logic [1:0]  bresp;
        logic [11:0] bid;
        bit          timed_out;

        axi_write(reg_addr(idx), data, 4'hF, 12'h100, bresp, bid, timed_out, 0, 0, 0, tag);
        if (timed_out) begin
            fail_count++;
            $display("[FAIL] t=%0t %-52s write timed out (reg %0d)", $time, tag, idx);
            return;
        end
        regs_shadow[idx] = data;
        check_bit($sformatf("%s: BRESP OKAY", tag), (bresp == 2'b00), 1'b1);
        check($sformatf("%s: reg_out[%0d]", tag, idx), reg_out_slice(reg_out, idx), regs_shadow[idx]);
    endtask

    task automatic do_regs_read_and_check(input string tag, input int idx);
        logic [31:0] data;
        logic [1:0]  rresp;
        logic [11:0] rid;
        bit          timed_out;

        axi_read(reg_addr(idx), 12'h200, data, rresp, rid, timed_out, 0, tag);
        if (timed_out) begin
            fail_count++;
            $display("[FAIL] t=%0t %-52s read timed out (reg %0d)", $time, tag, idx);
            return;
        end
        check_bit($sformatf("%s: RRESP OKAY", tag), (rresp == 2'b00), 1'b1);
        check($sformatf("%s: RDATA reg %0d", tag, idx), data, regs_shadow[idx]);
    endtask

    task automatic do_spi_write_and_check(input string tag, input logic [9:0] addr, input logic [7:0] data);
        logic [1:0]  bresp;
        logic [11:0] bid;
        bit          timed_out;
        int unsigned seen_before;

        seen_before = spi_txn_seen_count;
        axi_write(spi_addr(addr), {24'h0, data}, 4'hF, 12'h300, bresp, bid, timed_out, 0, 0, 0, tag);
        if (timed_out) begin
            fail_count++;
            $display("[FAIL] t=%0t %-52s write timed out (addr 0x%03h)", $time, tag, addr);
            return;
        end
        check_bit($sformatf("%s: BRESP OKAY", tag), (bresp == 2'b00), 1'b1);
        check($sformatf("%s: SPI txn count advanced", tag), {24'h0, spi_txn_seen_count}, {24'h0, seen_before + 1});
        check_bit($sformatf("%s: SPI saw RW=write", tag), last_rw, 1'b1);
        check($sformatf("%s: SPI saw ADDR", tag), {22'h0, last_addr}, {22'h0, addr});
        check($sformatf("%s: SPI saw WDATA", tag), {24'h0, last_data}, {24'h0, data});
        check($sformatf("%s: exactly 24 SPI_CLK edges", tag), last_edges, 32'd24);
        check($sformatf("%s: shadow register updated", tag), {24'h0, ad9361_shadow[addr]}, {24'h0, data});
    endtask

    task automatic do_spi_read_and_check(input string tag, input logic [9:0] addr);
        logic [31:0] data_out;
        logic [1:0]  rresp;
        logic [11:0] rid;
        bit          timed_out;
        logic [7:0]  expected;
        int unsigned seen_before;

        expected    = ad9361_shadow[addr];
        seen_before = spi_txn_seen_count;

        axi_read(spi_addr(addr), 12'h301, data_out, rresp, rid, timed_out, 0, tag);
        if (timed_out) begin
            fail_count++;
            $display("[FAIL] t=%0t %-52s read timed out (addr 0x%03h)", $time, tag, addr);
            return;
        end
        check_bit($sformatf("%s: RRESP OKAY", tag), (rresp == 2'b00), 1'b1);
        check($sformatf("%s: SPI txn count advanced", tag), {24'h0, spi_txn_seen_count}, {24'h0, seen_before + 1});
        check($sformatf("%s: AXI read data matches shadow", tag), {24'h0, data_out[7:0]}, {24'h0, expected});
    endtask

    task automatic do_cdc_read_and_check(input string tag, input int idx);
        logic [31:0] data;
        logic [1:0]  rresp;
        logic [11:0] rid;
        bit          timed_out;

        axi_read(cdc_addr(idx), 12'h400, data, rresp, rid, timed_out, 0, tag);
        if (timed_out) begin
            fail_count++;
            $display("[FAIL] t=%0t %-52s read timed out (lane %0d)", $time, tag, idx);
            return;
        end
        check_bit($sformatf("%s: RRESP OKAY", tag), (rresp == 2'b00), 1'b1);
        check($sformatf("%s: RDATA lane %0d", tag, idx), data, {24'h0, dsp_shadow[idx]});
    endtask

    task automatic do_cdc_write_attempt_and_check_decerr(input string tag, input int idx);
        logic [1:0]  bresp;
        logic [11:0] bid;
        bit          timed_out;

        axi_write(cdc_addr(idx), 32'hFFFF_FFFF, 4'hF, 12'hD00, bresp, bid, timed_out, 0, 0, 0, tag);
        if (timed_out) begin
            fail_count++;
            $display("[FAIL] t=%0t %-52s write timed out (should DECERR promptly)", $time, tag);
            return;
        end
        check_bit($sformatf("%s: BRESP DECERR", tag), (bresp == 2'b11), 1'b1);
    endtask

    // ------------------------------------------------------------------
    // Main test sequence
    // ------------------------------------------------------------------
    initial begin
        $display("=== tb_fm_receiver starting ===");

        do_reset();
        check("Reset: reg_out all zero", reg_out[31:0], 32'h0);
        check("Reset: no SPI transaction yet", {24'h0, spi_txn_seen_count}, 32'h0);
        check("Reset: dsp_reg_out all zero", dsp_reg_out[31:0], 32'h0);

        // ---- Smoke: each peripheral still works normally with the other
        //      one also present on the shared bus ----
        $display("=== Smoke: axi_registers through the shared bus ===");
        for (int i = 0; i < NUM_REGS; i++)
            do_regs_write_and_check($sformatf("Smoke regs write %0d", i), i, 32'hA000_0000 + i);
        for (int i = 0; i < NUM_REGS; i++)
            do_regs_read_and_check($sformatf("Smoke regs read %0d", i), i);

        $display("=== Smoke: axi_spi through the shared bus ===");
        do_spi_write_and_check("Smoke spi write 0x010", 10'h010, 8'h5A);
        do_spi_read_and_check ("Smoke spi read 0x010",  10'h010);
        do_spi_write_and_check("Smoke spi write 0x020", 10'h020, 8'hA5);
        do_spi_read_and_check ("Smoke spi read 0x020",  10'h020);

        $display("=== Smoke: axi_cdc_status through the shared bus ===");
        for (int i = 0; i < NUM_CDC_REGS; i++)
            dsp_write(i, 8'hC0 + i);
        for (int i = 0; i < NUM_CDC_REGS; i++)
            do_cdc_read_and_check($sformatf("Smoke cdc read lane %0d", i), i);
        do_cdc_write_attempt_and_check_decerr("Smoke cdc write attempt lane 0", 0);

        // ---- Cross-peripheral isolation ----
        $display("=== Isolation: regs traffic must not touch SPI ===");
        begin
            int unsigned seen_before = spi_txn_seen_count;
            do_regs_write_and_check("Isolation regs write 0", 0, 32'hDEAD_0000);
            do_regs_read_and_check ("Isolation regs read 0",  0);
            check("Isolation: no SPI activity from regs traffic",
                  {24'h0, spi_txn_seen_count}, {24'h0, seen_before});
        end

        $display("=== Isolation: SPI traffic must not touch reg_out ===");
        begin
            logic [31:0] before_reg3 = reg_out_slice(reg_out, 3);
            do_spi_write_and_check("Isolation spi write 0x030", 10'h030, 8'h33);
            do_spi_read_and_check ("Isolation spi read 0x030",  10'h030);
            check("Isolation: reg_out[3] untouched by SPI traffic",
                  reg_out_slice(reg_out, 3), before_reg3);
        end

        $display("=== Isolation: regs/SPI traffic must not touch dsp_reg_out ===");
        begin
            logic [511:0] before_dsp = dsp_reg_out;
            do_regs_write_and_check("Isolation regs write 1 (vs cdc)", 1, 32'hFEED_0001);
            do_spi_write_and_check ("Isolation spi write 0x050 (vs cdc)", 10'h050, 8'h77);
            check("Isolation: dsp_reg_out untouched by regs/SPI traffic",
                  dsp_reg_out[31:0], before_dsp[31:0]);
        end

        $display("=== Isolation: cdc write attempts must not touch regs/SPI ===");
        begin
            logic [31:0] before_reg4 = reg_out_slice(reg_out, 4);
            int unsigned seen_before = spi_txn_seen_count;
            do_cdc_write_attempt_and_check_decerr("Isolation cdc write attempt lane 5", 5);
            check("Isolation: reg_out[4] untouched by cdc write attempt",
                  reg_out_slice(reg_out, 4), before_reg4);
            check("Isolation: no SPI activity from cdc write attempt",
                  {24'h0, spi_txn_seen_count}, {24'h0, seen_before});
        end

        // ---- r_data mux correctness: a regs read must return regs data,
        //      never a leaked stale spi_rdata, and vice versa. Write
        //      deliberately different, non-zero-extending-collision
        //      patterns to each side first. ----
        $display("=== r_data mux correctness ===");
        do_spi_write_and_check ("Mux setup: spi write 0x040", 10'h040, 8'hC3);
        do_spi_read_and_check  ("Mux setup: spi read 0x040 (primes spi_rdata)", 10'h040);
        do_regs_write_and_check("Mux setup: regs write reg 1", 1, 32'h1234_5678);
        do_regs_read_and_check ("Mux: regs read reg 1 returns regs data, not spi's", 1);

        do_regs_write_and_check("Mux setup: regs write reg 2", 2, 32'h0000_00C3); // shares SPI's byte value on purpose
        do_regs_read_and_check ("Mux: regs read reg 2 (0xC3 byte, but from regs)", 2);
        do_spi_write_and_check ("Mux setup: spi write 0x041", 10'h041, 8'h78);
        do_spi_read_and_check  ("Mux: spi read 0x041 returns spi data, not regs'", 10'h041);

        dsp_write(9, 8'hC3); // shares the same byte value as reg 2 above, on purpose
        do_cdc_read_and_check ("Mux: cdc read lane 9 (0xC3 byte, but from cdc)", 9);
        do_regs_read_and_check("Mux: regs read reg 2 still returns regs data, not cdc's", 2);

        // ---- Interleaved back-and-forth: alternate regs/spi ops with no
        //      pattern, stressing the combining logic switching contexts
        //      every transaction. ----
        $display("=== Interleaved access ===");
        for (int i = 0; i < 6; i++) begin
            do_regs_write_and_check($sformatf("Interleave regs write %0d", i), i % NUM_REGS, 32'hB000_0000 + i);
            do_spi_write_and_check ($sformatf("Interleave spi write %0d", i), 10'h100 + i, 8'h50 + i);
            dsp_write(i, 8'h90 + i);
            do_regs_read_and_check ($sformatf("Interleave regs read %0d", i), i % NUM_REGS);
            do_spi_read_and_check  ($sformatf("Interleave spi read %0d", i), 10'h100 + i);
            do_cdc_read_and_check  ($sformatf("Interleave cdc read %0d", i), i);
        end

        // ---- Unmapped PERIPH_ID: matches neither peripheral. This is the
        //      real AND-of-no_addr test -- both regs_wnoaddr/spi_wnoaddr
        //      (etc.) must independently be true for axi_if to see
        //      w_no_addr/r_no_addr asserted at all. ----
        $display("=== Unmapped PERIPH_ID (DECERR) section ===");
        begin
            logic [31:0] unmapped   = {GP0_TOP_BYTE, UNMAPPED_PERIPH_ID, 16'h0000};
            logic [31:0] before_reg0 = reg_out_slice(reg_out, 0);
            logic [511:0] before_dsp = dsp_reg_out;
            logic [1:0]  bresp;
            logic [11:0] bid;
            bit          wtimed_out;
            logic [31:0] rdata;
            logic [1:0]  rresp;
            logic [11:0] rid;
            bit          rtimed_out;
            int unsigned seen_before = spi_txn_seen_count;

            axi_write(unmapped, 32'hDEAD_BEEF, 4'hF, 12'hC00, bresp, bid, wtimed_out, 0, 0, 0, "Unmapped write");
            if (!wtimed_out) begin
                check_bit("Unmapped write: BRESP DECERR", (bresp == 2'b11), 1'b1);
                check("Unmapped write: reg_out[0] untouched", reg_out_slice(reg_out, 0), before_reg0);
                check("Unmapped write: no SPI activity", {24'h0, spi_txn_seen_count}, {24'h0, seen_before});
                check("Unmapped write: dsp_reg_out untouched", dsp_reg_out[31:0], before_dsp[31:0]);
            end else begin
                fail_count++;
                $display("[FAIL] t=%0t Unmapped write unexpectedly timed out (should DECERR promptly)", $time);
            end

            axi_read(unmapped, 12'hC01, rdata, rresp, rid, rtimed_out, 0, "Unmapped read");
            if (!rtimed_out) begin
                check_bit("Unmapped read: RRESP DECERR", (rresp == 2'b11), 1'b1);
                check("Unmapped read: no SPI activity", {24'h0, spi_txn_seen_count}, {24'h0, seen_before});
            end else begin
                fail_count++;
                $display("[FAIL] t=%0t Unmapped read unexpectedly timed out (should DECERR promptly)", $time);
            end
        end

        // ---- Final sweep ----
        for (int i = 0; i < NUM_REGS; i++)
            check($sformatf("Final sweep reg_out[%0d]", i), reg_out_slice(reg_out, i), regs_shadow[i]);
        for (int i = 0; i < NUM_CDC_REGS; i++)
            do_cdc_read_and_check($sformatf("Final sweep cdc lane %0d", i), i);

        // ------------------------------------------------------------------
        $display("=== tb_fm_receiver finished: %0d passed, %0d failed ===", pass_count, fail_count);
        if (fail_count == 0)
            $display("=== RESULT: PASS ===");
        else
            $display("=== RESULT: FAIL ===");

        $finish;
    end

endmodule
