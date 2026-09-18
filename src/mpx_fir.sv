module mpx_fir (
    input  logic               clk,
    input  logic               rstb,
    input  logic signed [15:0] i_mpx_mono,
    input  logic signed [15:0] i_mpx_ster,
    input  logic               strb,
    output logic signed [15:0] o_audio_mono,
    output logic signed [15:0] o_audio_ster,
    output logic               valid
);

typedef enum logic [1:0] {ACCUM, MONO, STER, DONE} state_t;

// 8 taps/cycle: MONO+STER together take ~65 cycles, well inside one
// 125-cycle strb period -- a new strb can never land mid-computation,
// so the shift below never races against an in-progress MAC pass.
localparam int PARALLELISM = 8;

state_t state;
logic  [2:0] counter;
logic  [7:0] tap_idx;
logic signed [15:0] buffer_mono [0:255];
logic signed [15:0] buffer_ster [0:255];
logic signed [35:0] mono_accum, ster_accum;
logic signed [15:0] taps   [0:255];
// PLACEHOLDER coefficients (15kHz passband / 18kHz stopband, ~240kHz),
// just to give synthesis something real -- proper filter design TBD.
initial $readmemh("mpx_fir_coeffs.hex", taps);

// Combinational 8-way MAC for the current tap group.
logic signed [35:0] sum8_mono, sum8_ster;
always_comb begin
    sum8_mono = '0;
    sum8_ster = '0;
    for (int p = 0; p < PARALLELISM; p++) begin
        sum8_mono += taps[tap_idx+p] * buffer_mono[tap_idx+p];
        sum8_ster += taps[tap_idx+p] * buffer_ster[tap_idx+p];
    end
end

always_ff @(posedge clk) begin
    if(~rstb) begin
        tap_idx      <= '0;
        o_audio_mono <= '0;
        o_audio_ster <= '0;
        counter      <= '0;
        state        <= ACCUM;
        valid        <= '0;
        mono_accum   <= '0;
        ster_accum   <= '0;
    end else begin
        if(strb) begin
            // ==3'b101 (5), not 4: the transition check below reads
            // counter the cycle *after* a strobe updates it, so a
            // threshold of 4 only ever waited for 4 real strobes, not 5.
            counter <= counter == 3'b101 ? 3'b0 : counter + 3'b1;
            for (int i = 0; i < 255; i++) begin
                buffer_mono[i] <= buffer_mono[i+1];
                buffer_ster[i] <= buffer_ster[i+1];
            end
            buffer_mono[255] <= i_mpx_mono;
            buffer_ster[255] <= i_mpx_ster;
        end

        // Transition checks compare tap_idx/counter directly (not a
        // registered "done" flag derived from them) -- a derived flag
        // would lag one cycle behind, re-running the last tap group.
        case (state)
            ACCUM : state <= (counter == 3'b101)          ? MONO : ACCUM;
            MONO  : state <= (tap_idx == 256-PARALLELISM) ? STER : MONO;
            STER  : state <= (tap_idx == 256-PARALLELISM) ? DONE : STER;
            DONE  : state <= ACCUM;
        endcase

        if(state == ACCUM) begin
            tap_idx    <= '0;
            mono_accum <= '0;
            ster_accum <= '0;
            valid      <= 1'b0;
        end

        if(state == MONO) begin
            tap_idx    <= tap_idx + PARALLELISM;
            mono_accum <= mono_accum + sum8_mono;
        end

        if(state == STER) begin
            tap_idx    <= tap_idx + PARALLELISM;
            ster_accum <= ster_accum + sum8_ster;
        end

        if(state == DONE) begin
            // counter free-runs on every strb regardless of state, but
            // normally no strb lands during MONO/STER/DONE (65 cycles,
            // strobes ~125 apart) -- so without this it's still stuck
            // at 4 when ACCUM is re-entered, retriggering MONO one
            // cycle later with zero new samples instead of waiting for
            // the next 4 real strobes. Reset here, once, on the way out.
            counter      <= '0;
            valid        <= 1'b1;
            o_audio_mono <= mono_accum >>> 20;
            o_audio_ster <= ster_accum >>> 20;
        end
    end
end

endmodule