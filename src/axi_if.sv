module axi_if(
    input  logic          clk,
    input  logic          rstb,

    // Read address channel
    input  logic  [31:0]  m_axi_gp0_araddr,
    input  logic  [1:0]   m_axi_gp0_arburst,
    input  logic  [3:0]   m_axi_gp0_arcache,
    input  logic  [11:0]  m_axi_gp0_arid,
    input  logic  [3:0]   m_axi_gp0_arlen,
    input  logic  [1:0]   m_axi_gp0_arlock,
    input  logic  [2:0]   m_axi_gp0_arprot,
    input  logic  [3:0]   m_axi_gp0_arqos,
    output logic          m_axi_gp0_arready,
    input  logic  [2:0]   m_axi_gp0_arsize,
    input  logic          m_axi_gp0_arvalid,

    // Write address channel
    input  logic  [31:0]  m_axi_gp0_awaddr,
    input  logic  [1:0]   m_axi_gp0_awburst,
    input  logic  [3:0]   m_axi_gp0_awcache,
    input  logic  [11:0]  m_axi_gp0_awid,
    input  logic  [3:0]   m_axi_gp0_awlen,
    input  logic  [1:0]   m_axi_gp0_awlock,
    input  logic  [2:0]   m_axi_gp0_awprot,
    input  logic  [3:0]   m_axi_gp0_awqos,
    output logic          m_axi_gp0_awready,
    input  logic  [2:0]   m_axi_gp0_awsize,
    input  logic          m_axi_gp0_awvalid,

    // Write response channel
    output logic  [11:0]  m_axi_gp0_bid,
    input  logic          m_axi_gp0_bready,
    output logic  [1:0]   m_axi_gp0_bresp,
    output logic          m_axi_gp0_bvalid,

    // Read data channel
    output logic  [31:0]  m_axi_gp0_rdata,
    output logic  [11:0]  m_axi_gp0_rid,
    output logic          m_axi_gp0_rlast,
    input  logic          m_axi_gp0_rready,
    output logic  [1:0]   m_axi_gp0_rresp,
    output logic          m_axi_gp0_rvalid,

    // Write data channel
    input  logic  [31:0]  m_axi_gp0_wdata,
    input  logic  [11:0]  m_axi_gp0_wid,
    input  logic          m_axi_gp0_wlast,
    output logic          m_axi_gp0_wready,
    input  logic  [3:0]   m_axi_gp0_wstrb,
    input  logic          m_axi_gp0_wvalid,

    // ------------------------------------------------------------------
    // Peripheral-facing bus. Write and read sides are fully independent
    // (mirrors AXI's own AW/AR separation) -- axi_if imposes no ordering
    // between a pending write and a pending read; that's left entirely to
    // whatever peripheral is listening. addr is the full, unmasked 32-bit
    // AXI address: addr[23:16] is the peripheral-select field (each
    // peripheral compares it against its own PERIPH_ID and only reacts on
    // a match), addr[15:0] is that peripheral's own internal address.
    // addr[31:24] is deliberately NOT part of the match -- on real
    // hardware it's fixed by which PS7 AXI master port the transaction
    // came in on (M_AXI_GP0 forces addr[31:24] into 0x40-0x7F; nothing
    // here needs to know or care), so no peripheral ever inspects it. The
    // whole bus fans out identically to every peripheral in parallel (no
    // interconnect/address-range table needed here) -- *_no_addr coming
    // back tells axi_if nobody claimed the address, so it can reply with
    // DECERR immediately instead of waiting on a *_done that will never
    // come.
    // ------------------------------------------------------------------
    output logic          wen,
    output logic  [31:0]  w_addr,
    output logic  [31:0]  w_data,
    output logic  [3:0]   w_strb,
    input  logic          w_done,
    input  logic          w_no_addr,

    output logic          ren,
    output logic  [31:0]  r_addr,
    input  logic          r_done,
    input  logic  [31:0]  r_data,
    input  logic          r_no_addr
);

    localparam logic [1:0] RESP_OKAY   = 2'b00;
    localparam logic [1:0] RESP_DECERR = 2'b11; // no peripheral claimed the address

    // ------------------------------------------------------------------
    // FSM state encodings
    // ------------------------------------------------------------------
    typedef enum logic [1:0] {
        W_IDLE = 2'b00,
        W_REQ  = 2'b01, // wen asserted, waiting for w_done or w_no_addr
        W_RESP = 2'b10
    } w_state_t;

    typedef enum logic [1:0] {
        R_IDLE = 2'b00,
        R_REQ  = 2'b01, // ren asserted, waiting for r_done or r_no_addr
        R_RESP = 2'b10
    } r_state_t;

    w_state_t w_state, w_state_next;
    r_state_t r_state, r_state_next;

    // ------------------------------------------------------------------
    // Write-side captured transaction state. aw_seen/w_seen track the AW
    // and W channel handshakes independently (they may arrive on
    // different cycles) -- named to avoid clashing with the peripheral's
    // own w_done input.
    // ------------------------------------------------------------------
    logic        aw_seen, w_seen;
    logic [31:0] ltch_waddr;
    logic [11:0] ltch_id;
    logic [31:0] ltch_data;
    logic [3:0]  ltch_strb;

    // ==================================================================
    // Write FSM — next-state logic
    // ==================================================================
    always_comb begin : W_NEXT_STATE_LOGIC
        case (w_state)
            W_IDLE:  w_state_next = (aw_seen & w_seen) ? W_REQ : W_IDLE;
            W_REQ:   w_state_next = (w_done | w_no_addr) ? W_RESP : W_REQ;
            W_RESP:  w_state_next = (m_axi_gp0_bready & m_axi_gp0_bvalid) ? W_IDLE : W_RESP;
            default: w_state_next = W_IDLE;
        endcase
    end

    // ==================================================================
    // Write FSM — state/output/register logic
    // ==================================================================
    always_ff @(posedge clk or negedge rstb) begin : W_STATE_LOGIC
        if (~rstb) begin
            w_state         <= W_IDLE;
            aw_seen         <= 1'b0;
            w_seen          <= 1'b0;
            ltch_waddr      <= 32'h0;
            ltch_id         <= 12'h0;
            ltch_data       <= 32'h0;
            ltch_strb       <= 4'h0;
            m_axi_gp0_bid   <= 12'h0;
            m_axi_gp0_bresp <= RESP_OKAY;
        end
        else begin
            w_state <= w_state_next;

            case (w_state)
                // Independently capture AW and W the instant each one
                // handshakes — never re-sample the bus on later cycles.
                W_IDLE: begin
                    if (m_axi_gp0_awvalid & m_axi_gp0_awready) begin
                        aw_seen    <= 1'b1;
                        ltch_waddr <= m_axi_gp0_awaddr;
                        ltch_id    <= m_axi_gp0_awid;
                    end
                    if (m_axi_gp0_wvalid & m_axi_gp0_wready) begin
                        w_seen    <= 1'b1;
                        ltch_data <= m_axi_gp0_wdata;
                        ltch_strb <= m_axi_gp0_wstrb;
                    end
                end

                // wen is held (see assign below) until a peripheral claims
                // the address (w_done) or none does (w_no_addr) — latch
                // the response the instant it arrives.
                W_REQ: begin
                    if (w_done | w_no_addr) begin
                        m_axi_gp0_bid   <= ltch_id;
                        m_axi_gp0_bresp <= w_done ? RESP_OKAY : RESP_DECERR;
                    end
                end

                // Hold BVALID until the master accepts the response,
                // then release aw_seen/w_seen to allow the next transaction.
                W_RESP: begin
                    if (m_axi_gp0_bready & m_axi_gp0_bvalid) begin
                        aw_seen <= 1'b0;
                        w_seen  <= 1'b0;
                    end
                end

                default: ;
            endcase
        end
    end

    assign m_axi_gp0_awready = ~aw_seen;
    assign m_axi_gp0_wready  = ~w_seen;
    assign m_axi_gp0_bvalid  = (w_state == W_RESP);

    assign wen    = (w_state == W_REQ);
    assign w_addr = ltch_waddr;
    assign w_data = ltch_data;
    assign w_strb = ltch_strb;

    // ------------------------------------------------------------------
    // Read-side captured transaction state
    // ------------------------------------------------------------------
    logic [31:0] ltch_raddr;
    logic [11:0] ltch_rid;

    // ==================================================================
    // Read FSM — next-state logic
    // ==================================================================
    always_comb begin : R_NEXT_STATE_LOGIC
        case (r_state)
            R_IDLE:  r_state_next = (m_axi_gp0_arvalid & m_axi_gp0_arready) ? R_REQ : R_IDLE;
            R_REQ:   r_state_next = (r_done | r_no_addr) ? R_RESP : R_REQ;
            R_RESP:  r_state_next = (m_axi_gp0_rready & m_axi_gp0_rvalid) ? R_IDLE : R_RESP;
            default: r_state_next = R_IDLE;
        endcase
    end

    // ==================================================================
    // Read FSM — state/output/register logic
    // ==================================================================
    always_ff @(posedge clk or negedge rstb) begin : R_STATE_LOGIC
        if (~rstb) begin
            r_state         <= R_IDLE;
            ltch_raddr      <= 32'h0;
            ltch_rid        <= 12'h0;
            m_axi_gp0_rdata <= 32'h0;
            m_axi_gp0_rresp <= RESP_OKAY;
            m_axi_gp0_rid   <= 12'h0;
        end
        else begin
            r_state <= r_state_next;

            case (r_state)
                // Latch AR the instant it handshakes — ren/r_addr only
                // present it from R_REQ onward (see assigns below).
                R_IDLE: begin
                    if (m_axi_gp0_arvalid & m_axi_gp0_arready) begin
                        ltch_raddr <= m_axi_gp0_araddr;
                        ltch_rid   <= m_axi_gp0_arid;
                    end
                end

                // ren is held until a peripheral claims the address
                // (r_done, with r_data valid alongside it) or none does
                // (r_no_addr) — latch the response the instant it arrives.
                R_REQ: begin
                    if (r_done | r_no_addr) begin
                        m_axi_gp0_rdata <= r_done ? r_data : 32'h0;
                        m_axi_gp0_rresp <= r_done ? RESP_OKAY : RESP_DECERR;
                        m_axi_gp0_rid   <= ltch_rid;
                    end
                end

                R_RESP: ; // nothing to do — waiting for RREADY

                default: ;
            endcase
        end
    end

    assign m_axi_gp0_arready = (r_state == R_IDLE);
    assign m_axi_gp0_rvalid  = (r_state == R_RESP);
    assign m_axi_gp0_rlast   = 1'b1; // always single-beat

    assign ren    = (r_state == R_REQ);
    assign r_addr = ltch_raddr;

endmodule
