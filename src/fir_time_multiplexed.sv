module fir_time_multiplexed #(
  // Shared-datapath I/Q design below needs 2*(NUM_TAPS/PARALLELISM + 2)
  // clk cycles per decimated sample (both channels' MAC passes, each
  // with a 2-cycle pipeline drain -- see TOTAL_STEPS below) to fit
  // inside the CIC's 25-cycle decimation period. Comfortably true for
  // the tap counts being evaluated (32/48/64), not for the original 128.
  parameter int NUM_TAPS = 48   // must be a multiple of PARALLELISM (8, fixed below)
)(
  input  logic                     clk,
  input  logic                     rstb,

  input  logic signed [15:0]       data_in_i,
  input  logic signed [15:0]       data_in_q,
  input  logic                     valid,       // one-cycle pulse: data_in_i/data_in_q are fresh samples this cycle

  output logic signed [15:0]       data_out_i,
  output logic signed [15:0]       data_out_q,
  output logic                     done         // one-cycle pulse once BOTH data_out_i/data_out_q are valid
);

  localparam int WIDTH         = 16;  // project-standard datapath width (data and taps both)
  localparam int PARALLELISM   = 8;   // fixed: 8 DSP48-friendly multiplies per cycle, shared between I and Q
  localparam int PROD_SHIFT    = 8;   // fixed-point right-shift applied to each product before accumulation
  localparam int PROD_WIDTH    = 2*WIDTH - PROD_SHIFT;            // 24 @ WIDTH=16
  localparam int ACC_WIDTH     = PROD_WIDTH + $clog2(NUM_TAPS);   // overflow-safe accumulator width

  localparam int OUT_SHIFT     = 8;
  localparam int NUM_MAC_STEPS = NUM_TAPS / PARALLELISM;

  localparam int TOTAL_STEPS = NUM_MAC_STEPS + 2;

  // ------------------------------------------------------------
  // Taps: loaded once, shared by both channels (I and Q always use
  // identical coefficients for this filter). Never written anywhere
  // else, so no reset needed.
  // ------------------------------------------------------------
  logic signed [WIDTH-1:0] taps [NUM_TAPS];
  initial $readmemh("fir_coeffs.hex", taps);

  // ------------------------------------------------------------
  // Internal signals
  // ------------------------------------------------------------
  logic signed [WIDTH-1:0]   shift_reg_i [NUM_TAPS];
  logic signed [WIDTH-1:0]   shift_reg_q [NUM_TAPS];
  logic signed [WIDTH-1:0]   sample_s    [PARALLELISM];
  logic signed [WIDTH-1:0]   tap_s       [PARALLELISM];
  logic signed [2*WIDTH-1:0] mult        [PARALLELISM];  // exact WIDTHxWIDTH product width, no padding needed

  logic signed [ACC_WIDTH-1:0] partial_sum;
  logic signed [ACC_WIDTH-1:0] accum_i, accum_q;

  logic [$clog2(NUM_TAPS)-1:0]    tap_idx;   // current tap group base index (0, PARALLELISM, 2*PARALLELISM, ...)
  logic [$clog2(TOTAL_STEPS)-1:0] mac_step;  // cycle counter within the current channel's MAC pass


  logic fetch, fetch_d1, fetch_d2;

  typedef enum logic [1:0] {IDLE, MAC_I, MAC_Q, DONE} state_t;
  state_t state, next_state;

  assign fetch = (state == MAC_I || state == MAC_Q) && (mac_step < NUM_MAC_STEPS);

  // ------------------------------------------------------------
  // Shift registers: both channels load together on `valid`
  // ------------------------------------------------------------
  always_ff @(posedge clk) begin
    if (~rstb) begin
      for (int i = 0; i < NUM_TAPS; i++) begin
        shift_reg_i[i] <= '0;
        shift_reg_q[i] <= '0;
      end
    end else if (valid) begin
      for (int i = 0; i < NUM_TAPS-1; i++) begin
        shift_reg_i[i] <= shift_reg_i[i+1];
        shift_reg_q[i] <= shift_reg_q[i+1];
      end
      shift_reg_i[NUM_TAPS-1] <= data_in_i;
      shift_reg_q[NUM_TAPS-1] <= data_in_q;
    end
  end

  // ------------------------------------------------------------
  // FSM: IDLE -> MAC_I -> MAC_Q -> DONE -> IDLE. Each MAC_x state lasts
  // TOTAL_STEPS cycles (NUM_MAC_STEPS fetch cycles + 2 drain cycles).
  // ------------------------------------------------------------
  always_ff @(posedge clk) begin
    if (~rstb) state <= IDLE;
    else       state <= next_state;
  end

  always_comb begin
    next_state = state;
    unique case (state)
      IDLE : if (valid)                     next_state = MAC_I;
      MAC_I: if (mac_step == TOTAL_STEPS-1) next_state = MAC_Q;
      MAC_Q: if (mac_step == TOTAL_STEPS-1) next_state = DONE;
      DONE :                                next_state = IDLE;
    endcase
  end

  // ------------------------------------------------------------
  // tap_idx / mac_step. tap_idx only advances while fetch is true (so
  // it holds through the 2 drain cycles); mac_step counts every cycle
  // of MAC_I/MAC_Q so the FSM above can detect the end of TOTAL_STEPS.
  // Both reset at IDLE->MAC_I and again at MAC_I->MAC_Q, so the two
  // channels' passes are structurally identical.
  // ------------------------------------------------------------
  always_ff @(posedge clk) begin
    if (~rstb) begin
      tap_idx  <= '0;
      mac_step <= '0;
    end else begin
      unique case (state)
        IDLE: if (valid) begin
          tap_idx  <= '0;
          mac_step <= '0;
        end
        MAC_I, MAC_Q: begin
          if (mac_step == TOTAL_STEPS-1) begin
            tap_idx  <= '0;
            mac_step <= '0;
          end else begin
            if (fetch) tap_idx <= tap_idx + PARALLELISM;
            mac_step <= mac_step + 1'b1;
          end
        end
        default: ;
      endcase
    end
  end

  // ------------------------------------------------------------
  // Parallel sample selection + tap lookup. sample_s picks the active
  // channel's shift register; tap_s is a single shared lookup (I and Q
  // always use the same taps).
  // ------------------------------------------------------------
  generate
    for (genvar p = 0; p < PARALLELISM; p++) begin : SEL
      assign sample_s[p] = (state == MAC_Q) ? shift_reg_q[tap_idx + p] : shift_reg_i[tap_idx + p];
      assign tap_s[p]    = taps[(NUM_TAPS-1) - (tap_idx + p)];
    end
  endgenerate

  // ------------------------------------------------------------
  // MAC loop: one shared set of 8 multiplies + adder tree, used for
  // both channels' passes in turn (sample_s/tap_s are already
  // WIDTH-bit signed, so a plain `*` gives the exact full-precision
  // product -- no manual sign-extension padding needed). accum_i/
  // accum_q only accumulate when fetch_d2 confirms the arriving
  // partial_sum is real -- see the drain-bug fix above.
  // ------------------------------------------------------------
  always_ff @(posedge clk) begin
    if (~rstb) begin
      fetch_d1    <= 1'b0;
      fetch_d2    <= 1'b0;
      accum_i     <= '0;
      accum_q     <= '0;
      partial_sum <= '0;
      data_out_i  <= '0;
      data_out_q  <= '0;
      done        <= 1'b0;
    end else begin
      done     <= 1'b0;
      fetch_d1 <= fetch;
      fetch_d2 <= fetch_d1;

      unique case (state)
        IDLE: if (valid) begin
          accum_i     <= '0;
          accum_q     <= '0;
          partial_sum <= '0;
        end

        MAC_I, MAC_Q: begin
          if (fetch) begin
            for (int p = 0; p < PARALLELISM; p++) begin
              (* use_dsp = "yes" *)
              mult[p] <= sample_s[p] * tap_s[p];
            end
          end

          partial_sum <= ($signed(mult[0][2*WIDTH-1 -: PROD_WIDTH]) + $signed(mult[1][2*WIDTH-1 -: PROD_WIDTH]))
                       + ($signed(mult[2][2*WIDTH-1 -: PROD_WIDTH]) + $signed(mult[3][2*WIDTH-1 -: PROD_WIDTH]))
                       + ($signed(mult[4][2*WIDTH-1 -: PROD_WIDTH]) + $signed(mult[5][2*WIDTH-1 -: PROD_WIDTH]))
                       + ($signed(mult[6][2*WIDTH-1 -: PROD_WIDTH]) + $signed(mult[7][2*WIDTH-1 -: PROD_WIDTH]));

          if (fetch_d2) begin
            if (state == MAC_I) accum_i <= accum_i + partial_sum;
            else                 accum_q <= accum_q + partial_sum;
          end
        end

        DONE: begin
          data_out_i <= accum_i[OUT_SHIFT +: WIDTH];
          data_out_q <= accum_q[OUT_SHIFT +: WIDTH];
          done       <= 1'b1;
        end

        default: ;
      endcase
    end
  end

endmodule
