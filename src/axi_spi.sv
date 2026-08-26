module axi_spi #(
    parameter logic [7:0] PERIPH_ID = 8'h02
)(
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

    // SPI ports
    input  logic spi_miso,
    output logic spi_mosi,
    output logic spi_ce,
    output logic spi_ck
);

    logic [9:0] addr_reg;
    logic [7:0] wdata_reg;
    logic       ctrl_rw_reg;

    assign wdata_reg = w_data[7:0];

    // ------------------------------------------------------------------
    // Address match — combinational. axi_if holds wen/ren as a level and
    // polls w_no_addr/r_no_addr (and w_done/r_done, declared further
    // below) every cycle, so these must react the same cycle wen/ren
    // first appears, not one clock later.
    // ------------------------------------------------------------------
    logic w_match, r_match;
    assign w_match = (w_addr[23:16] == PERIPH_ID);
    assign r_match = (r_addr[23:16] == PERIPH_ID);

    assign w_no_addr = wen & ~w_match;
    assign r_no_addr = ren & ~r_match;

    // ------------------------------------------------------------------
    // FSM state encodings
    // ------------------------------------------------------------------
    typedef enum logic [1:0] {
        SPI_IDLE,
        SPI_DRIVE,  // half-period before the next rising edge
        SPI_SAMPLE, // half-period before the next falling edge
        SPI_DONE
    } spi_state_t;

    spi_state_t spi_state;

    // ------------------------------------------------------------------
    // New-request edge detection. wen/ren are held for the whole
    // transaction (possibly ~100 clk cycles), so launching off the
    // level would re-trigger every single cycle they're asserted —
    // including the brief SPI_IDLE window right after a transaction
    // finishes but before axi_if has noticed done and dropped wen/ren.
    // Only the rising edge means "this is a new request".
    // ------------------------------------------------------------------
    logic wen_d, ren_d;
    logic wen_pulse, ren_pulse;
    assign wen_pulse = wen & ~wen_d;
    assign ren_pulse = ren & ~ren_d;

    // ------------------------------------------------------------------
    // Write-side captured transaction state
    // ------------------------------------------------------------------
    logic        spi_start;    // 1-cycle pulse into the SPI engine

    // ------------------------------------------------------------------
    // SPI engine state. Fixed-length transaction only: 16-bit
    // instruction word + 1 data byte = 24 SPI_CLK edges, always (see
    // ad9361_registers.md). SPI Mode 1 (CPOL=0, CPHA=1) per the AD9361
    // Reference Manual (UG-570): SPI_ENB (CS) active low, SPI_CLK idles
    // low, data launched on the rising edge and sampled on the falling
    // edge by both sides, MSB-first, instruction word =
    // {RW, 3'b000, 2'b00, ADDR[9:0]}.
    // ------------------------------------------------------------------
    localparam int SPI_HALF_PERIOD = 2; // clk cycles per SPI_CLK half-period
                                         // -> SPI_CLK = clk / (2*SPI_HALF_PERIOD)
                                         //    = 100 MHz / 4 = 25 MHz, comfortably
                                         //    under the AD9361's 50 MHz SPI limit

    logic [3:0]  spi_half_cnt;
    logic [4:0]  spi_bit_cnt;   // which edge (0..23) is being completed
    logic [23:0] spi_shift_out; // {RW, 3'b000, 2'b00, ADDR[9:0], WDATA[7:0]}
    logic [7:0]  spi_shift_in;
    logic        spi_rw_latched;

    // ------------------------------------------------------------------
    // done/no_addr are the request/ack handshake axi_if polls every
    // cycle. This is the fix for the bug the user was chasing: these
    // used to be registered (`<=`), which put them one clock behind
    // wen/ren first appearing. Since axi_if samples done combinationally
    // the same cycle it enters its wait state, a registered done was
    // always showing a *stale* value at that exact moment — leftover
    // from reset (done defaulted high) or from the previous transaction
    // — so every request got acked one operation too early, and the
    // real SPI transfer for it only completed in the background,
    // showing up a full operation late on the next check. Driving these
    // combinationally off wen/ren + w_match/r_match + the engine's own
    // (registered) state removes that window: done can only ever read 1
    // while the engine is actually sitting in SPI_DONE for a matching,
    // still-held request.
    // ------------------------------------------------------------------
    assign w_done = wen & w_match & (spi_state == SPI_DONE) & spi_rw_latched;
    assign r_done = ren & r_match & (spi_state == SPI_DONE) & ~spi_rw_latched;

    // r_data reads spi_shift_in directly rather than staging it through
    // an extra registered copy: the last read bit lands in spi_shift_in
    // on the very same edge spi_state first becomes SPI_DONE (both are
    // set by the same SPI_SAMPLE case branch below), so it's already
    // valid for the whole SPI_DONE cycle that r_done is combinationally
    // high for. An extra copy-on-DONE register would land one edge
    // later than the done signal it's paired with — the same class of
    // bug as above, just on the read-data path.
    assign r_data = {24'b0, spi_shift_in};

    // ==================================================================
    // SPI engine: shared by both AXI write (launches a transaction) and
    // AXI read — a matching, newly-arrived wen/ren pulse launches one
    // fixed-length SPI transaction directly, no separate launch/status
    // registers. wen/ren stay held by axi_if until w_done/r_done (above)
    // goes high, which only happens once this engine actually reaches
    // SPI_DONE for that same request.
    // ==================================================================
    always_ff @(posedge clk or negedge rstb) begin : SPI_ENGINE
        if (~rstb) begin
            spi_state      <= SPI_IDLE;
            spi_half_cnt   <= '0;
            spi_bit_cnt    <= '0;
            spi_shift_out  <= '0;
            spi_shift_in   <= '0;
            spi_rw_latched <= 1'b0;
            spi_ce         <= 1'b1; // deasserted (active low)
            spi_ck         <= 1'b0; // idle low
            spi_mosi       <= 1'b0;
            wen_d          <= 1'b0;
            ren_d          <= 1'b0;
            spi_start      <= 1'b0;
            ctrl_rw_reg    <= 1'b0;
            addr_reg       <= '0;
        end
        else begin
            wen_d <= wen;
            ren_d <= ren;

            // Write has priority over a concurrent read pulse.
            if (wen_pulse & w_match) begin
                ctrl_rw_reg <= 1'b1;
                addr_reg    <= w_addr[11:2];
                spi_start   <= 1'b1;
            end
            else if (ren_pulse & r_match) begin
                ctrl_rw_reg <= 1'b0;
                addr_reg    <= r_addr[11:2];
                spi_start   <= 1'b1;
            end

            case (spi_state)
                SPI_IDLE: begin
                    spi_ce <= 1'b1;
                    spi_ck <= 1'b0;
                    if (spi_start) begin
                        spi_rw_latched <= ctrl_rw_reg;
                        spi_shift_out  <= {ctrl_rw_reg, 3'b000, 2'b00, addr_reg, wdata_reg};
                        spi_shift_in   <= 8'h0;
                        spi_bit_cnt    <= 5'h0;
                        spi_half_cnt   <= 4'h0;
                        spi_ce         <= 1'b0;        // assert CS before the first edge
                        spi_mosi       <= ctrl_rw_reg; // pre-drive bit[23] (the R/W bit)
                        spi_state      <= SPI_DRIVE;
                        spi_start      <= '0;
                    end
                end

                // Half-period before a rising edge. On this phase's first
                // cycle (one full clk cycle after the previous falling
                // edge — never on bit 0, which was already pre-driven at
                // launch), shift and drive the next MOSI bit. Doing this
                // here rather than at the tail of SPI_SAMPLE below matters:
                // updating MOSI on the exact same edge that SPI_SAMPLE
                // uses as its sample point is a zero-margin race (no hold
                // time) — this way MOSI changes a full cycle after the
                // sample point and is stable with a full cycle of setup
                // margin before the next rising edge.
                SPI_DRIVE: begin
                    if (spi_half_cnt == 4'h0 && spi_bit_cnt != 5'd0) begin
                        spi_shift_out <= {spi_shift_out[22:0], 1'b0};
                        if (spi_rw_latched || (spi_bit_cnt < 5'd16))
                            spi_mosi <= spi_shift_out[22];
                    end

                    if (spi_half_cnt == SPI_HALF_PERIOD-1) begin
                        spi_half_cnt <= '0;
                        spi_ck       <= 1'b1; // rising edge
                        spi_state    <= SPI_SAMPLE;
                    end
                    else begin
                        spi_half_cnt <= spi_half_cnt + 1'b1;
                    end
                end

                // Half-period before a falling edge: the sample point for
                // both sides. On the edge, sample MISO (data phase of a
                // read only) and advance the bit count. MOSI for the next
                // bit is prepared later, in SPI_DRIVE above — not here.
                SPI_SAMPLE: begin
                    if (spi_half_cnt == SPI_HALF_PERIOD-1) begin
                        spi_half_cnt <= '0;
                        spi_ck       <= 1'b0; // falling edge

                        if (!spi_rw_latched && (spi_bit_cnt >= 5'd16))
                            spi_shift_in <= {spi_shift_in[6:0], spi_miso};

                        spi_state   <= (spi_bit_cnt == 5'd23) ? SPI_DONE : SPI_DRIVE;
                        spi_bit_cnt <= spi_bit_cnt + 1'b1;
                    end
                    else begin
                        spi_half_cnt <= spi_half_cnt + 1'b1;
                    end
                end

                SPI_DONE: begin
                    spi_ce    <= 1'b1; // deassert CS after the last falling edge
                    spi_state <= SPI_IDLE;
                end

                default: spi_state <= SPI_IDLE;
            endcase
        end
    end

endmodule
