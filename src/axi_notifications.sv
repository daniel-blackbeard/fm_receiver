// axi_notifications — small AXI-lite regmap with a second, PL-facing write
// port, so PL logic can report status to PS with no DMA/interrupt
// machinery. First user: axi_dsp's DDR-write-pointer notification (see
// private/sample_streaming_plan.md) — reg0 bit0 = a new batch landed in
// DDR, bits[10:1] = which 1KB slice (0-1023, doubles as both count and
// address since 1MB/1KB = 1024 exactly). Regs 1-3 are spare for future
// PL status.
//
// No CDC needed: axi_dsp's burst engine and this module's AXI side are
// both on fclk0 (unlike axi_cdc_status, whose hardware side is genuinely
// on dsp_clk).
//
// Same-cycle collision (pl_wen and an AXI write hitting the same index):
// pl_wen wins. Not a structural guarantee, just a timing margin (PS would
// have to fall an entire notification period behind), but free to build
// in, and the alternative risks silently losing a real batch.
module axi_notifications #(
    parameter logic [7:0] PERIPH_ID = 8'h04
)(
    input  logic          clk,
    input  logic          rstb,

    // Peripheral bus from axi_if — see src/axi_if.sv's port-list comment
    // for the full contract. addr[23:16] is compared against PERIPH_ID;
    // addr[3:2] selects one of the 4 word registers, higher internal bits
    // are not range-checked (aliases back into the 4-register window,
    // same documented behavior as axi_registers.sv).
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

    // PL-facing write port, fclk0 domain (see header comment). Always a
    // full-word overwrite of reg_out[pl_windex] -- no per-byte strobe on
    // this side, unlike the AXI side, since every known PL writer sets a
    // complete value each time.
    input  logic          pl_wen,
    input  logic  [1:0]   pl_windex,
    input  logic  [31:0]  pl_wdata,

    output logic [127:0]  reg_out            // 4 words of 32 bits, packed array
);

    // ------------------------------------------------------------------
    // Address match — combinational, resolves the same cycle wen/ren is
    // presented, so w_done/r_done can also fire same-cycle (matches
    // axi_registers.sv's latency profile; axi_if itself tolerates a
    // later r_done just fine, see axi_cdc_status.sv, but there's no
    // reason to add latency here when nothing requires it).
    // ------------------------------------------------------------------
    logic w_match, r_match;
    assign w_match = (w_addr[23:16] == PERIPH_ID);
    assign r_match = (r_addr[23:16] == PERIPH_ID);

    assign w_no_addr = wen & ~w_match;
    assign r_no_addr = ren & ~r_match;
    assign w_done    = wen & w_match;
    assign r_done    = ren & r_match;

    logic [1:0]  w_idx, r_idx;
    logic [31:0] w_mask;
    assign w_idx  = w_addr[3:2];
    assign r_idx  = r_addr[3:2];
    assign w_mask = {{8{w_strb[3]}}, {8{w_strb[2]}}, {8{w_strb[1]}}, {8{w_strb[0]}}};

    assign r_data = (ren & r_match) ? reg_out[r_idx*32 +: 32] : 32'h0;

    always_ff @(posedge clk or negedge rstb) begin
        if (~rstb) begin
            reg_out <= 128'h0;
        end
        else begin
            // PL write, unconditional on index -- see header comment for
            // why this goes first/wins.
            if (pl_wen) begin
                reg_out[pl_windex*32 +: 32] <= pl_wdata;
            end

            // AXI write -- suppressed only for the exact index pl_wen is
            // writing this same cycle; every other index (or every cycle
            // pl_wen is low) behaves exactly like axi_registers.sv.
            if (wen & w_match & !(pl_wen && (pl_windex == w_idx))) begin
                reg_out[w_idx*32 +: 32] <= (w_data & w_mask) |
                                            (reg_out[w_idx*32 +: 32] & ~w_mask);
            end
        end
    end

endmodule
