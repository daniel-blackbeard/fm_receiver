module axi_registers #(
    parameter logic [7:0] PERIPH_ID = 8'h01
)(
    input  logic          clk,
    input  logic          rstb,

    // Peripheral bus from axi_if — see the port-list comment in
    // src/axi_if.sv for the full contract. addr[23:16] is compared
    // against PERIPH_ID; addr[4:2] selects one of the 8 word registers,
    // higher internal bits are not range-checked (aliases back into the
    // 8-register window, same documented behavior as the old axi_fm).
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

    output logic [255:0]  reg_out            // 8 words of 32 bits, packed array
);

    // ------------------------------------------------------------------
    // Address match — combinational, resolves the same cycle wen/ren is
    // presented, so w_done/r_done can also fire same-cycle (1-cycle
    // register writes/reads, same latency profile as the old axi_fm).
    // ------------------------------------------------------------------
    logic w_match, r_match;
    assign w_match = (w_addr[23:16] == PERIPH_ID);
    assign r_match = (r_addr[23:16] == PERIPH_ID);

    assign w_no_addr = wen & ~w_match;
    assign r_no_addr = ren & ~r_match;
    assign w_done    = wen & w_match;
    assign r_done    = ren & r_match;

    logic [2:0]  w_idx, r_idx;
    logic [31:0] w_mask;
    assign w_idx  = w_addr[4:2];
    assign r_idx  = r_addr[4:2];
    assign w_mask = {{8{w_strb[3]}}, {8{w_strb[2]}}, {8{w_strb[1]}}, {8{w_strb[0]}}};

    assign r_data = (ren & r_match) ? reg_out[r_idx*32 +: 32] : 32'h0;

    always_ff @(posedge clk or negedge rstb) begin
        if (~rstb) begin
            reg_out <= 256'h0;
        end
        else if (wen & w_match) begin
            reg_out[w_idx*32 +: 32] <= (w_data & w_mask) |
                                        (reg_out[w_idx*32 +: 32] & ~w_mask);
        end
    end

endmodule
