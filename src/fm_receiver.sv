module fm_receiver (
    input  logic clk_i,

    inout  wire [14:0] DDR_addr,
    inout  wire [2:0]  DDR_ba,
    inout  wire        DDR_cas_n,
    inout  wire        DDR_ck_n,
    inout  wire        DDR_ck_p,
    inout  wire        DDR_cke,
    inout  wire        DDR_cs_n,
    inout  wire [3:0]  DDR_dm,
    inout  wire [31:0] DDR_dq,
    inout  wire [3:0]  DDR_dqs_n,
    inout  wire [3:0]  DDR_dqs_p,
    inout  wire        DDR_odt,
    inout  wire        DDR_ras_n,
    inout  wire        DDR_reset_n,
    inout  wire        DDR_we_n,
    inout  wire        FIXED_IO_ddr_vrn,
    inout  wire        FIXED_IO_ddr_vrp,
    inout  wire [53:0] FIXED_IO_mio,
    inout  wire        FIXED_IO_ps_clk,
    inout  wire        FIXED_IO_ps_porb,
    inout  wire        FIXED_IO_ps_srstb,

    output logic gpio_3p3_1,
    output logic gpio_3p3_2,

    // AD9361 SPI bus — plain PL bank I/O (not a PS7 hard peripheral),
    // pin assignments already fixed in constraints/constraints.xdc:
    // spi_csn=R17, spi_clk=V18, spi_mosi=P16, spi_miso=V17.
    output logic spi_csn,
    output logic spi_clk,
    output logic spi_mosi,
    input  logic spi_miso,

    // AD3961 flags
    output       enable,
    output       gpio_resetb,
    output       txnrx,

    // AD9361 RX digital interface (LVDS) -- pins fixed in
    // constraints/constraints.xdc. RX only for now; TX interface pins
    // are constrained too but not wired up yet.
    input  logic       rx_clk_in_p,
    input  logic       rx_clk_in_n,
    input  logic       rx_frame_in_p,
    input  logic       rx_frame_in_n,
    input  logic [5:0] rx_data_in_p,
    input  logic [5:0] rx_data_in_n
);

logic [31:0] counter;
wire         fclk0;
wire         fclk0_rstn;   // active-low; unused for now — free-running counter needs no reset
logic        dsp_clk;      // ad3961_if_rx's recovered RX clock -- declared here (rather than
                            // down by its instantiation) so it's available to axi_cdc_status,
                            // instantiated earlier in the file
logic [15:0] data_rx0_i, data_rx0_q, data_rx1_i, data_rx1_q; // decimated I/Q, see "Glue logic" below --
                            // declared here so axi_dsp's instantiation (earlier in the file) can use them
logic        dec_done;      // decimation-stage pulse, same forward-declaration reasoning as above

// M_AXI_GP0 — PS7's AXI3 master into the PL. Internal wires, not top-level
// ports: this bus never leaves the chip. It goes straight into axi_if
// (below), which turns it into the wen/ren peripheral bus fanned out to
// axi_registers/axi_spi. Directions below are from design_1_wrapper's
// perspective (PS7 as AXI master); axi_if, as the slave, has every
// direction flipped relative to this list.
wire [31:0] m_axi_gp0_araddr;
wire [1:0]  m_axi_gp0_arburst;
wire [3:0]  m_axi_gp0_arcache;
wire [11:0] m_axi_gp0_arid;
wire [3:0]  m_axi_gp0_arlen;
wire [1:0]  m_axi_gp0_arlock;
wire [2:0]  m_axi_gp0_arprot;
wire [3:0]  m_axi_gp0_arqos;
wire        m_axi_gp0_arready;
wire [2:0]  m_axi_gp0_arsize;
wire        m_axi_gp0_arvalid;
wire [31:0] m_axi_gp0_awaddr;
wire [1:0]  m_axi_gp0_awburst;
wire [3:0]  m_axi_gp0_awcache;
wire [11:0] m_axi_gp0_awid;
wire [3:0]  m_axi_gp0_awlen;
wire [1:0]  m_axi_gp0_awlock;
wire [2:0]  m_axi_gp0_awprot;
wire [3:0]  m_axi_gp0_awqos;
wire        m_axi_gp0_awready;
wire [2:0]  m_axi_gp0_awsize;
wire        m_axi_gp0_awvalid;
wire [11:0] m_axi_gp0_bid;
wire        m_axi_gp0_bready;
wire [1:0]  m_axi_gp0_bresp;
wire        m_axi_gp0_bvalid;
wire [31:0] m_axi_gp0_rdata;
wire [11:0] m_axi_gp0_rid;
wire        m_axi_gp0_rlast;
wire        m_axi_gp0_rready;
wire [1:0]  m_axi_gp0_rresp;
wire        m_axi_gp0_rvalid;
wire [31:0] m_axi_gp0_wdata;
wire [11:0] m_axi_gp0_wid;
wire        m_axi_gp0_wlast;
wire        m_axi_gp0_wready;
wire [3:0]  m_axi_gp0_wstrb;
wire        m_axi_gp0_wvalid;

// S_AXI_HP0 -- opposite direction from M_AXI_GP0 above: PS7 is the AXI
// *slave* here, so the master-role signals (everything except the
// *ready outputs) are driven INTO design_1_wrapper from whatever PL
// master eventually exists, not out of it. Placeholder tie-offs live
// right after the design_1_wrapper/axi_if instantiations below, driving
// every master-role signal to a safe idle value until a real master
// (the planned RX-sample-to-DDR streaming engine) replaces them --
// see ps7_configure.tcl's S_AXI_HP0 section for the PS7-side setup.
// Widths and directions taken directly from the generated
// design_1_wrapper.v (64-bit data, 6-bit ID, confirmed after enabling
// S_AXI_HP0 -- not guessed): ID width is 6 bits here, not 12 like GP0's,
// and there's no separate ACLK port since it's wired internally in the
// BD (looped back to the same FCLK_CLK0 fm_receiver runs on).
wire [31:0] s_axi_hp0_araddr;
wire [1:0]  s_axi_hp0_arburst;
wire [3:0]  s_axi_hp0_arcache;
wire [5:0]  s_axi_hp0_arid;
wire [3:0]  s_axi_hp0_arlen;
wire [1:0]  s_axi_hp0_arlock;
wire [2:0]  s_axi_hp0_arprot;
wire [3:0]  s_axi_hp0_arqos;
wire        s_axi_hp0_arready;
wire [2:0]  s_axi_hp0_arsize;
wire        s_axi_hp0_arvalid;
wire [31:0] s_axi_hp0_awaddr;
wire [1:0]  s_axi_hp0_awburst;
wire [3:0]  s_axi_hp0_awcache;
wire [5:0]  s_axi_hp0_awid;
wire [3:0]  s_axi_hp0_awlen;
wire [1:0]  s_axi_hp0_awlock;
wire [2:0]  s_axi_hp0_awprot;
wire [3:0]  s_axi_hp0_awqos;
wire        s_axi_hp0_awready;
wire [2:0]  s_axi_hp0_awsize;
wire        s_axi_hp0_awvalid;
wire [5:0]  s_axi_hp0_bid;
wire        s_axi_hp0_bready;
wire [1:0]  s_axi_hp0_bresp;
wire        s_axi_hp0_bvalid;
wire [63:0] s_axi_hp0_rdata;
wire [5:0]  s_axi_hp0_rid;
wire        s_axi_hp0_rlast;
wire        s_axi_hp0_rready;
wire [1:0]  s_axi_hp0_rresp;
wire        s_axi_hp0_rvalid;
wire [63:0] s_axi_hp0_wdata;
wire [5:0]  s_axi_hp0_wid;
wire        s_axi_hp0_wlast;
wire        s_axi_hp0_wready;
wire [7:0]  s_axi_hp0_wstrb;
wire        s_axi_hp0_wvalid;

// Peripheral bus: axi_if (below) fans this out identically to every
// peripheral in parallel -- no interconnect, no address-range table.
// Each peripheral decodes p_waddr[23:16]/p_raddr[23:16] against its own
// PERIPH_ID and only reacts on a match; see src/axi_if.sv's port-list
// comment for the full contract. wen/ren/addr/data/strb are broadcast
// identically to both peripherals below; each peripheral's own
// done/no_addr/rdata come back on separate wires and get combined into
// the final p_* signals feeding axi_if: OR the dones, AND the no_addrs,
// mux rdata by whichever peripheral's rdone is high -- safe since
// PERIPH_ID match is mutually exclusive by construction (see
// src/axi_if.sv's port-list comment).
wire        p_wen;
wire [31:0] p_waddr;
wire [31:0] p_wdata;
wire [3:0]  p_wstrb;
wire        p_wdone;
wire        p_wnoaddr;

wire        p_ren;
wire [31:0] p_raddr;
wire        p_rdone;
wire [31:0] p_rdata;
wire        p_rnoaddr;

// axi_registers (PERIPH_ID 8'h01)
wire        regs_wdone;
wire        regs_wnoaddr;
wire        regs_rdone;
wire [31:0] regs_rdata;
wire        regs_rnoaddr;

// axi_spi (PERIPH_ID 8'h02)
wire        spi_wdone;
wire        spi_wnoaddr;
wire        spi_rdone;
wire [31:0] spi_rdata;
wire        spi_rnoaddr;

// axi_cdc_status (PERIPH_ID 8'h03) -- read-only from AXI, so cdc_wdone is
// always low and cdc_wnoaddr always high (see src/axi_cdc_status.sv);
// included in the combine below anyway for uniformity and so a future
// peripheral joining this list is a pure copy-paste.
wire        cdc_wdone;
wire        cdc_wnoaddr;
wire        cdc_rdone;
wire [31:0] cdc_rdata;
wire        cdc_rnoaddr;

// axi_notifications (PERIPH_ID 8'h04)
wire        notif_wdone;
wire        notif_wnoaddr;
wire        notif_rdone;
wire [31:0] notif_rdata;
wire        notif_rnoaddr;

assign p_wdone   = regs_wdone   | spi_wdone   | cdc_wdone   | notif_wdone;
assign p_wnoaddr = regs_wnoaddr & spi_wnoaddr & cdc_wnoaddr & notif_wnoaddr;
assign p_rdone   = regs_rdone   | spi_rdone   | cdc_rdone   | notif_rdone;
assign p_rnoaddr = regs_rnoaddr & spi_rnoaddr & cdc_rnoaddr & notif_rnoaddr;
assign p_rdata   = regs_rdone ? regs_rdata : (spi_rdone ? spi_rdata : (cdc_rdone ? cdc_rdata : notif_rdata));

wire [255:0] regmap;
wire         blink_en;
wire [7:0]   blink_speed;
logic        adc_r1_mode_ps;

// RM 0x00
assign blink_en       = regmap[0];
// RM 0x04
assign blink_speed    = regmap[39:32];
// RM 0x08
assign enable         = regmap[64];
assign txnrx          = regmap[65];
assign gpio_resetb    = regmap[66];

// adc_r1_mode_ps is registered here in fclk0, NOT a plain assign from
// regmap[67] directly -- the dsp_clk-domain CDC synchronizer taps this
// register instead of the raw regmap wire, so our cross-domain read
// never adds fanout/routing load onto axi_registers' own internal
// readback path for this word. (Suspected cause of regmap word 0x08
// reading back with its low byte mangled on UART, back when this word
// lived in axi_fm: that module's AXI-read datapath for this word may
// have been timing-sensitive, and this bit was the only one in that
// byte with an external consumer beyond the register module itself.)
always_ff @( posedge fclk0 ) begin
    adc_r1_mode_ps <= regmap[67];
end

always_ff @( posedge fclk0 ) begin
    counter <= counter + 32'b1;
end

assign gpio_3p3_1 = blink_en & counter[blink_speed[4:0]];
//assign gpio_3p3_1 = regmap[0];

design_1_wrapper ps_u (
    .DDR_addr          (DDR_addr),
    .DDR_ba            (DDR_ba),
    .DDR_cas_n         (DDR_cas_n),
    .DDR_ck_n          (DDR_ck_n),
    .DDR_ck_p          (DDR_ck_p),
    .DDR_cke           (DDR_cke),
    .DDR_cs_n          (DDR_cs_n),
    .DDR_dm            (DDR_dm),
    .DDR_dq            (DDR_dq),
    .DDR_dqs_n         (DDR_dqs_n),
    .DDR_dqs_p         (DDR_dqs_p),
    .DDR_odt           (DDR_odt),
    .DDR_ras_n         (DDR_ras_n),
    .DDR_reset_n       (DDR_reset_n),
    .DDR_we_n          (DDR_we_n),
    .FIXED_IO_ddr_vrn  (FIXED_IO_ddr_vrn),
    .FIXED_IO_ddr_vrp  (FIXED_IO_ddr_vrp),
    .FIXED_IO_mio      (FIXED_IO_mio),
    .FIXED_IO_ps_clk   (FIXED_IO_ps_clk),
    .FIXED_IO_ps_porb  (FIXED_IO_ps_porb),
    .FIXED_IO_ps_srstb (FIXED_IO_ps_srstb),
    .fclk0             (fclk0),
    .fclk0_rstn        (fclk0_rstn),
    .m_axi_gp0_araddr  (m_axi_gp0_araddr),
    .m_axi_gp0_arburst (m_axi_gp0_arburst),
    .m_axi_gp0_arcache (m_axi_gp0_arcache),
    .m_axi_gp0_arid    (m_axi_gp0_arid),
    .m_axi_gp0_arlen   (m_axi_gp0_arlen),
    .m_axi_gp0_arlock  (m_axi_gp0_arlock),
    .m_axi_gp0_arprot  (m_axi_gp0_arprot),
    .m_axi_gp0_arqos   (m_axi_gp0_arqos),
    .m_axi_gp0_arready (m_axi_gp0_arready),
    .m_axi_gp0_arsize  (m_axi_gp0_arsize),
    .m_axi_gp0_arvalid (m_axi_gp0_arvalid),
    .m_axi_gp0_awaddr  (m_axi_gp0_awaddr),
    .m_axi_gp0_awburst (m_axi_gp0_awburst),
    .m_axi_gp0_awcache (m_axi_gp0_awcache),
    .m_axi_gp0_awid    (m_axi_gp0_awid),
    .m_axi_gp0_awlen   (m_axi_gp0_awlen),
    .m_axi_gp0_awlock  (m_axi_gp0_awlock),
    .m_axi_gp0_awprot  (m_axi_gp0_awprot),
    .m_axi_gp0_awqos   (m_axi_gp0_awqos),
    .m_axi_gp0_awready (m_axi_gp0_awready),
    .m_axi_gp0_awsize  (m_axi_gp0_awsize),
    .m_axi_gp0_awvalid (m_axi_gp0_awvalid),
    .m_axi_gp0_bid     (m_axi_gp0_bid),
    .m_axi_gp0_bready  (m_axi_gp0_bready),
    .m_axi_gp0_bresp   (m_axi_gp0_bresp),
    .m_axi_gp0_bvalid  (m_axi_gp0_bvalid),
    .m_axi_gp0_rdata   (m_axi_gp0_rdata),
    .m_axi_gp0_rid     (m_axi_gp0_rid),
    .m_axi_gp0_rlast   (m_axi_gp0_rlast),
    .m_axi_gp0_rready  (m_axi_gp0_rready),
    .m_axi_gp0_rresp   (m_axi_gp0_rresp),
    .m_axi_gp0_rvalid  (m_axi_gp0_rvalid),
    .m_axi_gp0_wdata   (m_axi_gp0_wdata),
    .m_axi_gp0_wid     (m_axi_gp0_wid),
    .m_axi_gp0_wlast   (m_axi_gp0_wlast),
    .m_axi_gp0_wready  (m_axi_gp0_wready),
    .m_axi_gp0_wstrb   (m_axi_gp0_wstrb),
    .m_axi_gp0_wvalid  (m_axi_gp0_wvalid),

    .s_axi_hp0_araddr  (s_axi_hp0_araddr),
    .s_axi_hp0_arburst (s_axi_hp0_arburst),
    .s_axi_hp0_arcache (s_axi_hp0_arcache),
    .s_axi_hp0_arid    (s_axi_hp0_arid),
    .s_axi_hp0_arlen   (s_axi_hp0_arlen),
    .s_axi_hp0_arlock  (s_axi_hp0_arlock),
    .s_axi_hp0_arprot  (s_axi_hp0_arprot),
    .s_axi_hp0_arqos   (s_axi_hp0_arqos),
    .s_axi_hp0_arready (s_axi_hp0_arready),
    .s_axi_hp0_arsize  (s_axi_hp0_arsize),
    .s_axi_hp0_arvalid (s_axi_hp0_arvalid),
    .s_axi_hp0_awaddr  (s_axi_hp0_awaddr),
    .s_axi_hp0_awburst (s_axi_hp0_awburst),
    .s_axi_hp0_awcache (s_axi_hp0_awcache),
    .s_axi_hp0_awid    (s_axi_hp0_awid),
    .s_axi_hp0_awlen   (s_axi_hp0_awlen),
    .s_axi_hp0_awlock  (s_axi_hp0_awlock),
    .s_axi_hp0_awprot  (s_axi_hp0_awprot),
    .s_axi_hp0_awqos   (s_axi_hp0_awqos),
    .s_axi_hp0_awready (s_axi_hp0_awready),
    .s_axi_hp0_awsize  (s_axi_hp0_awsize),
    .s_axi_hp0_awvalid (s_axi_hp0_awvalid),
    .s_axi_hp0_bid     (s_axi_hp0_bid),
    .s_axi_hp0_bready  (s_axi_hp0_bready),
    .s_axi_hp0_bresp   (s_axi_hp0_bresp),
    .s_axi_hp0_bvalid  (s_axi_hp0_bvalid),
    .s_axi_hp0_rdata   (s_axi_hp0_rdata),
    .s_axi_hp0_rid     (s_axi_hp0_rid),
    .s_axi_hp0_rlast   (s_axi_hp0_rlast),
    .s_axi_hp0_rready  (s_axi_hp0_rready),
    .s_axi_hp0_rresp   (s_axi_hp0_rresp),
    .s_axi_hp0_rvalid  (s_axi_hp0_rvalid),
    .s_axi_hp0_wdata   (s_axi_hp0_wdata),
    .s_axi_hp0_wid     (s_axi_hp0_wid),
    .s_axi_hp0_wlast   (s_axi_hp0_wlast),
    .s_axi_hp0_wready  (s_axi_hp0_wready),
    .s_axi_hp0_wstrb   (s_axi_hp0_wstrb),
    .s_axi_hp0_wvalid  (s_axi_hp0_wvalid)
);

// S_AXI_HP0 read channel: axi_dsp is write-only, PL never reads back
// what it streams out, so tie off AR/R permanently.
assign s_axi_hp0_arvalid = 1'b0;
assign s_axi_hp0_araddr  = 32'd0;
assign s_axi_hp0_arburst = 2'd0;
assign s_axi_hp0_arcache = 4'd0;
assign s_axi_hp0_arid    = 6'd0;
assign s_axi_hp0_arlen   = 4'd0;
assign s_axi_hp0_arlock  = 2'd0;
assign s_axi_hp0_arprot  = 3'd0;
assign s_axi_hp0_arqos   = 4'd0;
assign s_axi_hp0_arsize  = 3'd0;
assign s_axi_hp0_rready  = 1'b1;

// axi_dsp: the DSP-sample-to-DDR streaming master, driving S_AXI_HP0 and
// axi_notifications' PL write port.
wire [31:0] axi_dsp_pl_update;
wire [1:0]  axi_dsp_pl_index;
wire        axi_dsp_pl_wen;
wire [1:0]  axi_dsp_dbg_state;
wire        axi_dsp_dbg_pending;
wire        axi_dsp_dbg_trigger;
wire        axi_dsp_dbg_drain_bank;
wire [19:0] axi_dsp_dbg_wr_offset;

// dsp_clk-domain reset synchronizer for axi_dsp's rstb_dsp: fclk0_rstn
// crossed into dsp_clk via async-assert/sync-release (asserts immediately,
// releases after 2 dsp_clk cycles so the release edge can't cause
// metastability downstream).
logic rstb_dsp_meta, rstb_dsp_sync;
always_ff @(posedge dsp_clk or negedge fclk0_rstn) begin
    if (!fclk0_rstn) begin
        rstb_dsp_meta <= 1'b0;
        rstb_dsp_sync <= 1'b0;
    end else begin
        rstb_dsp_meta <= 1'b1;
        rstb_dsp_sync <= rstb_dsp_meta;
    end
end

axi_dsp u_axi_dsp (
    .clk_fpga  (fclk0),
    .clk_dsp   (dsp_clk),
    .rstb_dsp  (rstb_dsp_sync),
    .rstb_fpga (fclk0_rstn),

    .i_valid (dec_done),           // placeholder -- no real DSP sample pipeline feeds this yet
    .ch0_i (data_rx0_i),            // placeholder -- real RX0 I data not wired yet
    .ch0_q (data_rx0_q),            // placeholder -- real RX0 Q data not wired yet
    .ch1_i (data_rx1_i),            // placeholder -- real RX1 I data not wired yet
    .ch1_q (data_rx1_q),            // placeholder -- real RX1 Q data not wired yet

    .awid    (s_axi_hp0_awid),
    .awaddr  (s_axi_hp0_awaddr),
    .awlen   (s_axi_hp0_awlen),
    .awsize  (s_axi_hp0_awsize),
    .awburst (s_axi_hp0_awburst),
    .awlock  (s_axi_hp0_awlock),
    .awcache (s_axi_hp0_awcache),
    .awprot  (s_axi_hp0_awprot),
    .awqos   (s_axi_hp0_awqos),
    .awvalid (s_axi_hp0_awvalid),
    .awready (s_axi_hp0_awready),

    .wid     (s_axi_hp0_wid),
    .wdata   (s_axi_hp0_wdata),
    .wstrb   (s_axi_hp0_wstrb),
    .wlast   (s_axi_hp0_wlast),
    .wvalid  (s_axi_hp0_wvalid),
    .wready  (s_axi_hp0_wready),

    .bid     (s_axi_hp0_bid),
    .bresp   (s_axi_hp0_bresp),
    .bvalid  (s_axi_hp0_bvalid),
    .bready  (s_axi_hp0_bready),

    .pl_update (axi_dsp_pl_update),
    .pl_index  (axi_dsp_pl_index),
    .pl_wen    (axi_dsp_pl_wen),

    .dbg_state      (axi_dsp_dbg_state),
    .dbg_pending    (axi_dsp_dbg_pending),
    .dbg_trigger    (axi_dsp_dbg_trigger),
    .dbg_drain_bank (axi_dsp_dbg_drain_bank),
    .dbg_wr_offset  (axi_dsp_dbg_wr_offset)
);

axi_if u_axi_if (
    .clk  (fclk0),
    .rstb (fclk0_rstn),

    .m_axi_gp0_araddr  (m_axi_gp0_araddr),
    .m_axi_gp0_arburst (m_axi_gp0_arburst),
    .m_axi_gp0_arcache (m_axi_gp0_arcache),
    .m_axi_gp0_arid    (m_axi_gp0_arid),
    .m_axi_gp0_arlen   (m_axi_gp0_arlen),
    .m_axi_gp0_arlock  (m_axi_gp0_arlock),
    .m_axi_gp0_arprot  (m_axi_gp0_arprot),
    .m_axi_gp0_arqos   (m_axi_gp0_arqos),
    .m_axi_gp0_arready (m_axi_gp0_arready),
    .m_axi_gp0_arsize  (m_axi_gp0_arsize),
    .m_axi_gp0_arvalid (m_axi_gp0_arvalid),

    .m_axi_gp0_awaddr  (m_axi_gp0_awaddr),
    .m_axi_gp0_awburst (m_axi_gp0_awburst),
    .m_axi_gp0_awcache (m_axi_gp0_awcache),
    .m_axi_gp0_awid    (m_axi_gp0_awid),
    .m_axi_gp0_awlen   (m_axi_gp0_awlen),
    .m_axi_gp0_awlock  (m_axi_gp0_awlock),
    .m_axi_gp0_awprot  (m_axi_gp0_awprot),
    .m_axi_gp0_awqos   (m_axi_gp0_awqos),
    .m_axi_gp0_awready (m_axi_gp0_awready),
    .m_axi_gp0_awsize  (m_axi_gp0_awsize),
    .m_axi_gp0_awvalid (m_axi_gp0_awvalid),

    .m_axi_gp0_bid    (m_axi_gp0_bid),
    .m_axi_gp0_bready (m_axi_gp0_bready),
    .m_axi_gp0_bresp  (m_axi_gp0_bresp),
    .m_axi_gp0_bvalid (m_axi_gp0_bvalid),

    .m_axi_gp0_rdata  (m_axi_gp0_rdata),
    .m_axi_gp0_rid    (m_axi_gp0_rid),
    .m_axi_gp0_rlast  (m_axi_gp0_rlast),
    .m_axi_gp0_rready (m_axi_gp0_rready),
    .m_axi_gp0_rresp  (m_axi_gp0_rresp),
    .m_axi_gp0_rvalid (m_axi_gp0_rvalid),

    .m_axi_gp0_wdata  (m_axi_gp0_wdata),
    .m_axi_gp0_wid    (m_axi_gp0_wid),
    .m_axi_gp0_wlast  (m_axi_gp0_wlast),
    .m_axi_gp0_wready (m_axi_gp0_wready),
    .m_axi_gp0_wstrb  (m_axi_gp0_wstrb),
    .m_axi_gp0_wvalid (m_axi_gp0_wvalid),

    .wen    (p_wen),
    .w_addr (p_waddr),
    .w_data (p_wdata),
    .w_strb (p_wstrb),
    .w_done (p_wdone),
    .w_no_addr (p_wnoaddr),

    .ren    (p_ren),
    .r_addr (p_raddr),
    .r_done (p_rdone),
    .r_data (p_rdata),
    .r_no_addr (p_rnoaddr)
);

axi_registers #(
    .PERIPH_ID (8'h01)
) u_axi_registers (
    .clk  (fclk0),
    .rstb (fclk0_rstn),

    .wen    (p_wen),
    .w_addr (p_waddr),
    .w_data (p_wdata),
    .w_strb (p_wstrb),
    .w_done (regs_wdone),
    .w_no_addr (regs_wnoaddr),

    .ren    (p_ren),
    .r_addr (p_raddr),
    .r_done (regs_rdone),
    .r_data (regs_rdata),
    .r_no_addr (regs_rnoaddr),

    .reg_out (regmap)
);

// AD9361 SPI bus. axi_spi's spi_ce/spi_ck naming is generic
// (chip-enable/clock); mapped here to this board's spi_csn/spi_clk pins.
axi_spi #(
    .PERIPH_ID (8'h02)
) u_axi_spi (
    .clk  (fclk0),
    .rstb (fclk0_rstn),

    .wen    (p_wen),
    .w_addr (p_waddr),
    .w_data (p_wdata),
    .w_strb (p_wstrb),
    .w_done (spi_wdone),
    .w_no_addr (spi_wnoaddr),

    .ren    (p_ren),
    .r_addr (p_raddr),
    .r_done (spi_rdone),
    .r_data (spi_rdata),
    .r_no_addr (spi_rnoaddr),

    .spi_miso (spi_miso),
    .spi_mosi (spi_mosi),
    .spi_ce   (spi_csn),
    .spi_ck   (spi_clk)
);

// DSP-side write port for axi_cdc_status. axi_cdc_status only accepts one
// 8-bit lane per dsp_clk cycle (dsp_wdata into 1-of-64 lanes selected by
// dsp_windex), but there are several live status values to keep mirrored
// (RX data snapshot, valid count, error count, raw pre-decode diagnostic
// taps, and the 32-bit heartbeat counter split across 4 lanes) --
// cdc_mirror_* below is a small round-robin sequencer that rewrites each
// target lane in turn, one per dsp_clk cycle, reusing the single-lane
// port exactly as designed rather than changing axi_cdc_status.sv's
// interface. dsp_rstb is tied to 1'b1
// for now: dsp_clk-domain logic elsewhere in this file (dsp_counter, the
// adc_r1_mode sync chain) has no reset wired up yet either -- a real
// dsp-domain reset is a separate, not-yet-decided piece of work.
wire [511:0] cdc_dsp_reg_out;
wire         cdc_dsp_wen;
logic [5:0]  cdc_dsp_windex;
logic [7:0]  cdc_dsp_wdata;

axi_cdc_status #(
    .PERIPH_ID (8'h03)
) u_axi_cdc_status (
    .clk  (fclk0),
    .rstb (fclk0_rstn),

    .wen    (p_wen),
    .w_addr (p_waddr),
    .w_data (p_wdata),
    .w_strb (p_wstrb),
    .w_done (cdc_wdone),
    .w_no_addr (cdc_wnoaddr),

    .ren    (p_ren),
    .r_addr (p_raddr),
    .r_done (cdc_rdone),
    .r_data (cdc_rdata),
    .r_no_addr (cdc_rnoaddr),

    .dsp_clk    (dsp_clk),
    .dsp_rstb   (1'b1),
    .dsp_wen    (cdc_dsp_wen),
    .dsp_windex (cdc_dsp_windex),
    .dsp_wdata  (cdc_dsp_wdata),
    .dsp_reg_out (cdc_dsp_reg_out)
);

// Debug tap: sticky "ever seen since boot" bits for S_AXI_HP0's write
// handshake, mirrored into axi_notifications register 1 (spare). Kept
// for bring-up visibility.
logic seen_awvalid, seen_awready, seen_wvalid, seen_wready, seen_bvalid, seen_bready;
always_ff @(posedge fclk0 or negedge fclk0_rstn) begin
    if (!fclk0_rstn) begin
        seen_awvalid <= 1'b0;
        seen_awready <= 1'b0;
        seen_wvalid  <= 1'b0;
        seen_wready  <= 1'b0;
        seen_bvalid  <= 1'b0;
        seen_bready  <= 1'b0;
    end else begin
        if (s_axi_hp0_awvalid) seen_awvalid <= 1'b1;
        if (s_axi_hp0_awready) seen_awready <= 1'b1;
        if (s_axi_hp0_wvalid)  seen_wvalid  <= 1'b1;
        if (s_axi_hp0_wready)  seen_wready  <= 1'b1;
        if (s_axi_hp0_bvalid)  seen_bvalid  <= 1'b1;
        if (s_axi_hp0_bready)  seen_bready  <= 1'b1;
    end
end

wire [31:0] dbg_hp0_status = {26'b0, seen_bready, seen_bvalid, seen_wready, seen_wvalid, seen_awready, seen_awvalid};

// Second half of the debug tap: axi_dsp's internal FSM/bookkeeping state
// (register 2, spare). trigger is sticky since it's a single-cycle pulse
// that an async UART read would otherwise almost never catch live.
logic seen_trigger;
always_ff @(posedge fclk0 or negedge fclk0_rstn) begin
    if (!fclk0_rstn) seen_trigger <= 1'b0;
    else if (axi_dsp_dbg_trigger) seen_trigger <= 1'b1;
end

wire [31:0] dbg_axi_dsp_status = {7'b0, axi_dsp_dbg_wr_offset, axi_dsp_dbg_drain_bank,
                                   seen_trigger, axi_dsp_dbg_pending, axi_dsp_dbg_state};

// Debug taps share axi_notifications' single PL write port with axi_dsp's
// real notification: axi_dsp's write (register 0) takes unconditional
// priority when it fires; otherwise registers 1/2 alternate each cycle.
logic dbg_toggle;
always_ff @(posedge fclk0 or negedge fclk0_rstn) begin
    if (!fclk0_rstn) dbg_toggle <= 1'b0;
    else              dbg_toggle <= ~dbg_toggle;
end

wire        notif_pl_wen_muxed   = 1'b1;
wire [1:0]  notif_pl_index_muxed = axi_dsp_pl_wen ? axi_dsp_pl_index : (dbg_toggle ? 2'd2 : 2'd1);
wire [31:0] notif_pl_data_muxed  = axi_dsp_pl_wen ? axi_dsp_pl_update
                                    : (dbg_toggle ? dbg_axi_dsp_status : dbg_hp0_status);

// axi_notifications: PL-to-PS status regmap (see
// src/axi_notifications.sv and private/sample_streaming_plan.md).
wire [127:0] notif_reg_out;

axi_notifications #(
    .PERIPH_ID (8'h04)
) u_axi_notifications (
    .clk  (fclk0),
    .rstb (fclk0_rstn),

    .wen    (p_wen),
    .w_addr (p_waddr),
    .w_data (p_wdata),
    .w_strb (p_wstrb),
    .w_done (notif_wdone),
    .w_no_addr (notif_wnoaddr),

    .ren    (p_ren),
    .r_addr (p_raddr),
    .r_done (notif_rdone),
    .r_data (notif_rdata),
    .r_no_addr (notif_rnoaddr),

    .pl_wen    (notif_pl_wen_muxed),
    .pl_windex (notif_pl_index_muxed),
    .pl_wdata  (notif_pl_data_muxed),

    .reg_out (notif_reg_out)
);

// AD9361 RX digital interface -- ad3961_if_rx's own recovered RX clock
// (looped in from rx_clk_in_p/n) and decoded ADC samples. Nothing
// downstream consumes these yet (no DSP chain built) -- wired through
// as a complete, connected instance rather than dangling ports, ready
// for whatever consumes it next.
logic        adc_r1_mode;
logic        adc_r1_mode_dsp;
logic        adc_valid;
logic [11:0] adc_data_i1;
logic [11:0] adc_data_q1;
logic [11:0] adc_data_i2;
logic [11:0] adc_data_q2;
logic        adc_status;
logic [11:0] dbg_rx_data;
logic [3:0]  dbg_rx_frame_s;

logic        [ 7:0] dsp_reserved1;
logic        [ 7:0] dsp_reserved2;
logic        [ 7:0] dsp_reserved3;

// DSP side synchronizers
always_ff @( posedge dsp_clk ) begin
    adc_r1_mode_dsp <= adc_r1_mode_ps;
    adc_r1_mode     <= adc_r1_mode_dsp;
end

ad3961_if_rx u_ad3961_if_rx (
    .dsp_clk       (dsp_clk),

    .rx_clk_in_p   (rx_clk_in_p),
    .rx_clk_in_n   (rx_clk_in_n),
    .rx_frame_in_p (rx_frame_in_p),
    .rx_frame_in_n (rx_frame_in_n),
    .rx_data_in_p  (rx_data_in_p),
    .rx_data_in_n  (rx_data_in_n),

    .adc_r1_mode   (adc_r1_mode),
    .adc_valid     (adc_valid),
    .adc_data_i1   (adc_data_i1),
    .adc_data_q1   (adc_data_q1),
    .adc_data_i2   (adc_data_i2),
    .adc_data_q2   (adc_data_q2),
    .adc_status    (adc_status),

    .dbg_rx_data     (dbg_rx_data),
    .dbg_rx_frame_s  (dbg_rx_frame_s)
);

// Glue logic: decimation stage x8 (declarations moved up to axi_dsp's
// own declaration block above -- same reasoning as dsp_clk's forward
// declaration: xvlog requires declare-before-use even across a module
// instantiation's port map, not just within a single always_ff block;
// synth_design tolerates the original order fine, xvlog doesn't)
logic  [2:0] dec_counter;

always_ff @( posedge dsp_clk ) begin
    dec_counter <= dec_counter + 3'b1;
    if(dec_done) begin
        data_rx0_i <= adc_data_i1;
        data_rx0_q <= adc_data_q1;
        data_rx1_i <= adc_data_i2;
        data_rx1_q <= adc_data_q2;
    end else begin
        data_rx0_i <= data_rx0_i + adc_data_i1;
        data_rx0_q <= data_rx0_q + adc_data_q1;
        data_rx1_i <= data_rx1_i + adc_data_i2;
        data_rx1_q <= data_rx1_q + adc_data_q2;
    end
end

assign dec_done = dec_counter == 3'b0;

assign gpio_3p3_2 = adc_status;

logic [31:0] dsp_counter;
always_ff @( posedge dsp_clk ) begin
    dsp_counter <= dsp_counter + 32'b1;
end

// ------------------------------------------------------------------
// Sample valid/error counters + a decimated raw-data tap, all feeding
// axi_cdc_status for readback over AXI (PERIPH_ID 0x03) -- see
// README.md's "Sample counters" section. Plain wrapping 8-bit counters,
// not the earlier decaying-count idea: simpler, and a rolling count is
// easier to reason about from a single UART snapshot than a leaky
// integrator's steady-state value would be.
// ------------------------------------------------------------------
logic [7:0] valid_count;
always_ff @( posedge dsp_clk ) begin
    if (adc_valid) valid_count <= valid_count + 8'd1;
end

logic [7:0] error_count;
always_ff @( posedge dsp_clk ) begin
    if (~adc_status) error_count <= error_count + 8'd1;
end

// One raw RX sample (adc_data_i1's 8 MSBs -- axi_cdc_status's lanes are
// only 8 bits, so the low 4 bits of the 12-bit ADC sample are dropped)
// captured every 47th valid sample rather than every one, so this
// register doesn't just keep landing on the same phase of whatever
// periodic pattern is streaming (BIST/PRBS or otherwise) -- 47 doesn't
// divide any of the standard PRBS periods (127, 511, 2047, 32767, ...)
// or the frame-decode cycle, so the sampled phase keeps moving.
logic [5:0] rx_decim_cnt;
logic [7:0] rx_data_snapshot;
always_ff @( posedge dsp_clk ) begin
    if (adc_valid) begin
        if (rx_decim_cnt == 6'd46) begin
            rx_decim_cnt     <= 6'd0;
            rx_data_snapshot <= adc_data_i1[11:4];
        end else begin
            rx_decim_cnt <= rx_decim_cnt + 6'd1;
        end
    end
end

// Round-robin mirror into axi_cdc_status: one lane rewritten per dsp_clk
// cycle, cycling through reg0 (RX data), reg1 (valid count), reg2 (error
// count), reg3 (raw pre-decode rx_frame_s -- diagnostic, see below),
// reg4-7 (dsp_counter, 32 bits split across 4 lanes, a dsp_clk-domain
// heartbeat/"blinky" replacement), reg8-9 (raw pre-decode rx_data --
// diagnostic). A full cycle is 10 dsp_clk cycles, far faster than any of
// these values actually change, so the lag between a value updating and
// its mirrored lane catching up is never meaningful here.
//
// reg3/reg8-9 exist to answer a question upstream of frame lock itself:
// coarse REG_RX_CLOCK_DATA_DELAY sweeps (see [[project_trajectory]] in
// project memory) produced zero change in valid_count across the whole
// register range, which doesn't fit a simple timing-margin story -- these
// mirror rx_frame_s/rx_data exactly as the frame-match logic in
// ad3961_if_rx.sv sees them, ungated by adc_valid, so a raw capture can
// show whether the LVDS interface is toggling near the expected pattern
// (supports the timing theory) or is dead/stuck (points elsewhere).
//
// reg10-63 are deliberately left unused here -- reserved for the
// correlator/histogram dsp_clk-domain modules planned for next session
// (see [[project_trajectory]]), so they can mirror their own output into
// axi_cdc_status the same way without another interface change.
logic [3:0] cdc_mirror_seq;
always_ff @( posedge dsp_clk ) begin
    cdc_mirror_seq <= (cdc_mirror_seq == 4'd9) ? 4'd0 : cdc_mirror_seq + 4'd1;
end

assign cdc_dsp_wen = 1'b1;

always_comb begin
    case (cdc_mirror_seq)
        4'd0: begin cdc_dsp_windex = 6'd0; cdc_dsp_wdata = rx_data_snapshot;   end
        4'd1: begin cdc_dsp_windex = 6'd1; cdc_dsp_wdata = valid_count;       end
        4'd2: begin cdc_dsp_windex = 6'd2; cdc_dsp_wdata = error_count;       end
        4'd3: begin cdc_dsp_windex = 6'd3; cdc_dsp_wdata = {4'b0, dbg_rx_frame_s}; end
        4'd4: begin cdc_dsp_windex = 6'd4; cdc_dsp_wdata = dsp_counter[7:0];  end
        4'd5: begin cdc_dsp_windex = 6'd5; cdc_dsp_wdata = dsp_counter[15:8]; end
        4'd6: begin cdc_dsp_windex = 6'd6; cdc_dsp_wdata = dsp_counter[23:16]; end
        4'd7: begin cdc_dsp_windex = 6'd7; cdc_dsp_wdata = dsp_counter[31:24]; end
        4'd8: begin cdc_dsp_windex = 6'd8; cdc_dsp_wdata = dbg_rx_data[7:0];  end
        default: begin cdc_dsp_windex = 6'd9; cdc_dsp_wdata = {4'b0, dbg_rx_data[11:8]}; end
    endcase
end

endmodule
