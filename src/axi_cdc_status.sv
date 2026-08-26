module axi_cdc_status #(
    parameter logic [7:0] PERIPH_ID = 8'h03
)(
    // ------------------------------------------------------------------
    // AXI-facing peripheral bus (fclk0 domain) — see src/axi_if.sv's
    // port-list comment for the full contract. This peripheral is
    // read-only from the AXI master's point of view: a write matching
    // PERIPH_ID still comes back DECERR (not a silent no-op) since there
    // is nothing here for software to legitimately write.
    // ------------------------------------------------------------------
    input  logic          clk,
    input  logic          rstb,

    input  logic          wen,
    input  logic  [31:0]  w_addr,
    input  logic  [31:0]  w_data,
    input  logic  [3:0]   w_strb,
    output logic          w_done,
    output logic          w_no_addr,

    input  logic          ren,
    input  logic  [31:0]  r_addr,
    output logic          r_done,
    output logic  [31:0]  r_data,
    output logic          r_no_addr,

    // ------------------------------------------------------------------
    // DSP-facing side (dsp_clk domain) — freely read/written by
    // dsp_clk-domain logic, no handshake needed here since it's the
    // storage's own native clock. dsp_reg_out is a plain combinational
    // tap (packed [511:0], 64 x 8-bit lanes, same slicing convention as
    // axi_registers.sv's reg_out) for any other dsp_clk logic to read
    // directly; dsp_wen/dsp_windex/dsp_wdata is a one-register-per-cycle
    // synchronous write port.
    // ------------------------------------------------------------------
    input  logic          dsp_clk,
    input  logic          dsp_rstb,

    input  logic          dsp_wen,
    input  logic  [5:0]   dsp_windex,
    input  logic  [7:0]   dsp_wdata,

    output logic [511:0]  dsp_reg_out
);

    // ------------------------------------------------------------------
    // Storage — lives entirely in the dsp_clk domain. 64 x 8-bit lanes,
    // packed the same way axi_registers.sv packs its 8 x 32-bit lanes.
    // Sized 4x the lane count actually in use as of the RX-lock
    // investigation (see [[project_trajectory]] in project memory) so
    // upcoming dsp_clk-domain diagnostic modules (correlator, histogram)
    // have room to mirror their own output without another interface
    // change.
    // ------------------------------------------------------------------
    logic [511:0] status_reg;

    always_ff @(posedge dsp_clk or negedge dsp_rstb) begin
        if (~dsp_rstb) begin
            status_reg <= 512'h0;
        end
        else if (dsp_wen) begin
            status_reg[dsp_windex*8 +: 8] <= dsp_wdata;
        end
    end

    assign dsp_reg_out = status_reg;

    // ------------------------------------------------------------------
    // AXI write side — always DECERR. w_addr is intentionally never
    // even inspected: any write reaching this PERIPH_ID is unsupported,
    // not just an unmapped sub-address.
    // ------------------------------------------------------------------
    assign w_no_addr = wen;
    assign w_done    = 1'b0;

    // ------------------------------------------------------------------
    // AXI read side — address match is combinational (same reasoning as
    // axi_registers.sv/axi_spi.sv: axi_if polls r_no_addr every cycle,
    // so a mismatch must be visible the same cycle ren appears, not one
    // clock later). r_idx selects one of the 64 lanes (addr[7:2]); not
    // range-checked above that (same documented aliasing behavior as
    // axi_registers.sv).
    // ------------------------------------------------------------------
    logic       r_match;
    logic [5:0] r_idx;
    assign r_match  = (r_addr[23:16] == PERIPH_ID);
    assign r_idx    = r_addr[7:2];
    assign r_no_addr = ren & ~r_match;

    // ------------------------------------------------------------------
    // Read handshake — toggle-synchronized, not level/ack: axi_if never
    // guarantees an idle gap between back-to-back requests, so a
    // level-based ack (raise on request seen, drop once request drops)
    // can have its retraction still in flight through the synchronizer
    // when the next request's pulse arrives, silently skipping that
    // request's snapshot capture and leaking the previous one's stale
    // value through instead. A toggle handshake has no minimum-pulse-
    // width requirement and is immune to that: each request becomes a
    // single edge-detected toggle-flip, unambiguous regardless of
    // timing, and axi_if can't present a new request until r_done has
    // already fired for the current one (see below) -- so there's
    // structurally never more than one toggle-flip in flight at once.
    //
    //   fclk0: r_req = ren & r_match (level, held until r_done responds)
    //          edge-detected -> flips r_req_toggle once per new request
    //     -> 2FF sync into dsp_clk
    //   dsp_clk: synced toggle changing value = "new request arrived" ->
    //            snapshot status_reg[r_idx], flip r_ack_toggle_dsp
    //     -> 2FF sync back into fclk0
    //   fclk0: synced ack toggle changing value = r_done pulses for one
    //          cycle, r_data = the (already-settled) synchronized
    //          snapshot
    // ------------------------------------------------------------------
    logic r_req;
    assign r_req = ren & r_match;

    logic r_req_d;
    logic r_req_toggle;
    always_ff @(posedge clk or negedge rstb) begin
        if (~rstb) begin
            r_req_d      <= 1'b0;
            r_req_toggle <= 1'b0;
        end
        else begin
            r_req_d <= r_req;
            if (r_req && !r_req_d) r_req_toggle <= ~r_req_toggle;
        end
    end

    logic r_req_toggle_meta, r_req_toggle_dsp, r_req_toggle_dsp_d;
    always_ff @(posedge dsp_clk or negedge dsp_rstb) begin
        if (~dsp_rstb) begin
            r_req_toggle_meta  <= 1'b0;
            r_req_toggle_dsp   <= 1'b0;
            r_req_toggle_dsp_d <= 1'b0;
        end
        else begin
            r_req_toggle_meta  <= r_req_toggle;
            r_req_toggle_dsp   <= r_req_toggle_meta;
            r_req_toggle_dsp_d <= r_req_toggle_dsp;
        end
    end

    logic r_new_req_dsp;
    assign r_new_req_dsp = (r_req_toggle_dsp != r_req_toggle_dsp_d);

    logic [7:0] r_snapshot_dsp;
    logic       r_ack_toggle_dsp;
    always_ff @(posedge dsp_clk or negedge dsp_rstb) begin
        if (~dsp_rstb) begin
            r_snapshot_dsp   <= 8'h0;
            r_ack_toggle_dsp <= 1'b0;
        end
        else if (r_new_req_dsp) begin
            r_snapshot_dsp   <= status_reg[r_idx*8 +: 8];
            r_ack_toggle_dsp <= ~r_ack_toggle_dsp;
        end
    end

    logic r_ack_toggle_meta, r_ack_toggle, r_ack_toggle_d;
    always_ff @(posedge clk or negedge rstb) begin
        if (~rstb) begin
            r_ack_toggle_meta <= 1'b0;
            r_ack_toggle      <= 1'b0;
            r_ack_toggle_d    <= 1'b0;
        end
        else begin
            r_ack_toggle_meta <= r_ack_toggle_dsp;
            r_ack_toggle      <= r_ack_toggle_meta;
            r_ack_toggle_d    <= r_ack_toggle;
        end
    end

    assign r_done = (r_ack_toggle != r_ack_toggle_d) & r_match;
    assign r_data = {24'b0, r_snapshot_dsp};

endmodule
