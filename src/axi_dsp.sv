module axi_dsp (
    input  logic        clk_fpga,
    input  logic        clk_dsp,
    input  logic        rstb_dsp,    // active-low reset, dsp_clk domain
    input  logic        rstb_fpga,   // active-low reset, fpga_clk domain

    input  logic        i_valid,
    input  logic [15:0] ch0_i,
    input  logic [15:0] ch0_q,
    input  logic [15:0] ch1_i,
    input  logic [15:0] ch1_q,

    // AXI Interface
    output logic [31:0] awaddr,
    output logic  [5:0] awid,
    output logic  [3:0] awlen,
    output logic  [3:0] awcache,
    output logic  [3:0] awqos,
    output logic  [2:0] awsize,
    output logic  [2:0] awprot,
    output logic  [1:0] awlock,
    output logic  [1:0] awburst,
    output logic        awvalid,
    input  logic        awready,

    output logic [63:0] wdata,
    output logic  [5:0] wid,
    output logic  [7:0] wstrb,
    output logic        wlast,
    output logic        wvalid,
    input  logic        wready,

    input  logic        bvalid,
    output logic        bready,
    input  logic  [1:0] bresp,
    input  logic  [5:0] bid,
    output logic [31:0] pl_update,
    output logic  [1:0] pl_index,
    output logic        pl_wen,

    // Debug tap: internal FSM/bookkeeping state (all clk_fpga domain,
    // no CDC needed), mirrored into axi_notifications for bring-up.
    output logic  [1:0] dbg_state,
    output logic        dbg_pending,
    output logic        dbg_trigger,
    output logic        dbg_drain_bank,
    output logic [19:0] dbg_wr_offset,
    output logic        dbg_timeout // pulses once per watchdog recovery, see below
);

    logic [63:0] buffer [0:31]; // 32 samples of 16 bits

    localparam int          BANK_SAMPLES = 16;
    localparam logic [31:0] BANK_BASE    = 32'h0200_0000;
    localparam logic [31:0] REGION_SIZE  = 32'h0010_0000; // 1 MByte
    localparam logic [31:0] BURST_BYTES  = BANK_SAMPLES * 8; // 128 bytes/burst

    // ---------------- DSP domain ----------------
    logic [4:0] addr;
    logic dsp_ready;
    logic dsp_flag, dsp_ff1_flag;
    logic former_bank;
    logic fpga_flag; // declared here (moved up from the FPGA-domain block
                      // below) since it's read in the DSP-domain block
                      // right after this -- Vivado's elaborator requires
                      // declare-before-use, unlike some other tools

    logic [63:0] i_data;
    assign i_data = {ch0_i, ch0_q, ch1_i, ch1_q};

    always_ff @(posedge clk_dsp or negedge rstb_dsp) begin
        if (!rstb_dsp) begin
            addr         <= '0;
            dsp_ready    <= '0;
            dsp_flag     <= '0;
            dsp_ff1_flag <= '0;
            former_bank  <= '0;
        end else begin
            if(i_valid) begin
                addr         <= addr + 5'b1;
                buffer[addr] <= i_data;
                former_bank  <= addr[4];

                if (former_bank != addr[4]) dsp_ready <= 1'b1;
                if (dsp_flag)               dsp_ready <= 1'b0;
            end

            // Ungated (unlike dsp_ready above): a synchronizer must
            // sample every dsp_clk cycle, not just once per i_valid.
            dsp_ff1_flag <= fpga_flag;
            dsp_flag     <= dsp_ff1_flag;
        end
    end

    // ---------------- FPGA domain ----------------
    logic fpga_ff1_ready;
    logic fpga_flag_d;
    logic trigger;

    always_ff @(posedge clk_fpga or negedge rstb_fpga) begin
        if (!rstb_fpga) begin
            fpga_ff1_ready <= '0;
            fpga_flag      <= '0;
            fpga_flag_d    <= '0;
        end else begin
            fpga_ff1_ready <= dsp_ready;
            fpga_flag      <= fpga_ff1_ready;
            fpga_flag_d    <= fpga_flag;
        end
    end

    assign trigger = fpga_flag & ~fpga_flag_d;

    typedef enum logic [1:0] {BM_IDLE, BM_AW, BM_DATA} burst_state_t;
    burst_state_t state;

    logic         pending;
    logic         drain_bank;
    logic  [3:0]  rd_ptr;
    logic  [3:0]  beat_cnt;
    logic [31:0]  wr_offset;   // byte offset within the 1MB region, wraps
    logic  [2:0]  pl_addr_last;
    logic  [2:0]  pl_addr_curr;
    logic  [9:0]  batch_addr;

    // Watchdog: recovers from a hung S_AXI_HP0 response -- without it the
    // FSM parks in BM_DATA forever with wvalid held and wready never
    // returned. AXI3 forbids retracting AWVALID/WVALID mid-handshake in
    // normal operation, but a genuinely hung slave leaves no other option
    // than abandoning that one burst -- same "drop rather than block"
    // stance as the rest of this design (sample_streaming_plan.md). 16
    // bits gives ~655us margin at fclk0's 100MHz over any legitimate
    // response delay. Counts only while waiting; reset on every real
    // state entry/beat.
    localparam logic [15:0] WATCHDOG_LIMIT = 16'hFFFF;
    logic [15:0] wd_cnt;
    logic        wd_timeout;
    assign wd_timeout = (wd_cnt == WATCHDOG_LIMIT);

    always_ff @(posedge clk_fpga or negedge rstb_fpga) begin
        if (!rstb_fpga) begin
            state        <= BM_IDLE;
            pending      <= '0;
            drain_bank   <= '0;
            rd_ptr       <= '0;
            awvalid      <= '0;
            awlock       <= '0;
            wvalid       <= '0;
            wlast        <= '0;
            bready       <= 1'b1;
            wr_offset    <= '0;
            pl_update    <= '0;
            pl_addr_last <= '0;
            pl_addr_curr <= '0;
            batch_addr   <= '0;
            wd_cnt       <= '0;
            dbg_timeout  <= '0;
        end else begin

            if (trigger) pending <= 1'b1;

            batch_addr <= wr_offset[19:10];
            pl_addr_last <= pl_addr_curr;
            pl_update    <= {21'b0, batch_addr, 1'b1};
            pl_addr_curr <= wr_offset[9:7];

            dbg_timeout <= 1'b0; // single-cycle pulse, see fm_receiver.sv's sticky mirror

            case (state)

                BM_IDLE: begin
                    wd_cnt <= '0;
                    if (pending) begin
                        pending  <= 1'b0;
                        awaddr   <= BANK_BASE + wr_offset;
                        awlen    <= BANK_SAMPLES - 1;
                        awsize   <= 3'b011;
                        awburst  <= 2'b01;
                        awvalid  <= 1'b1;
                        rd_ptr   <= '0;
                        state    <= BM_AW;

                        // advance the circular write pointer for the *next* burst
                        if (wr_offset + BURST_BYTES >= REGION_SIZE)
                            wr_offset <= '0;
                        else
                            wr_offset <= wr_offset + BURST_BYTES;
                    end
                end

                BM_AW: begin
                    if (awvalid && awready) begin
                        awvalid  <= 1'b0;
                        beat_cnt <= BANK_SAMPLES;
                        wdata    <= buffer[{drain_bank, 4'd0}];
                        wstrb    <= 8'b11111111;
                        wvalid   <= 1'b1;
                        state    <= BM_DATA;
                        wd_cnt   <= '0;
                    end else if (wd_timeout) begin
                        // AWREADY never came -- abandon this burst, go idle.
                        awvalid     <= 1'b0;
                        state       <= BM_IDLE;
                        wd_cnt      <= '0;
                        dbg_timeout <= 1'b1;
                    end else begin
                        wd_cnt <= wd_cnt + 16'd1;
                    end
                end

                BM_DATA: begin
                    if (wvalid && wready) begin
                        wd_cnt <= '0;
                        if (beat_cnt == 1) begin
                            wvalid     <= 1'b0;
                            wlast      <= 1'b0;
                            drain_bank <= ~drain_bank;
                            state      <= BM_IDLE;
                        end else begin
                            rd_ptr   <= rd_ptr + 1;
                            beat_cnt <= beat_cnt - 1;
                            wdata    <= buffer[{drain_bank, rd_ptr + 4'd1}];
                            wlast    <= (beat_cnt == 2);
                        end
                    end else if (wd_timeout) begin
                        // WREADY never came -- abandon the rest of this
                        // burst. Beats already landed stay in DDR as a
                        // partial write; accepted.
                        wvalid      <= 1'b0;
                        wlast       <= 1'b0;
                        drain_bank  <= ~drain_bank;
                        state       <= BM_IDLE;
                        wd_cnt      <= '0;
                        dbg_timeout <= 1'b1;
                    end else begin
                        wd_cnt <= wd_cnt + 16'd1;
                    end
                end
            endcase
        end
    end

    assign awid     = 6'b0;
    assign wid      = 6'b0;
    assign awprot   = 3'b0;
    assign awqos    = 4'b0;
    assign awcache  = 4'b0011;
    assign pl_index = 2'b0;
    assign pl_wen   = (pl_addr_curr == 3'b0) & (pl_addr_last == 3'b111);

    assign dbg_state      = state;
    assign dbg_pending    = pending;
    assign dbg_trigger    = trigger;
    assign dbg_drain_bank = drain_bank;
    assign dbg_wr_offset  = wr_offset[19:0];

endmodule