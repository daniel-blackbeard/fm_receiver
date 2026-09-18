module pll #(
    // Loop filter gains as shift amounts (Kp=2^-KP_SHIFT, Ki=2^-KI_SHIFT).
    // Defaults are the deployed values chosen from tb_pll_sweep.sv's
    // comparison (2026-09-18): smoothest/lowest-overshoot of the sweep,
    // still a good (~0.31s) settle time -- see that sweep's plot/numbers
    // for the other options and their trade-offs.
    parameter int KP_SHIFT = 11,
    parameter int KI_SHIFT = 26
)(
    input  logic               clk,
    input  logic               rstb,
    input  logic signed [15:0] i_data,
    input  logic               strb,
    output logic signed [15:0] o_vco1_sin,
    output logic signed [15:0] o_vco1_cos,
    output logic signed [15:0] o_vco2_sin,
    output logic signed [15:0] o_vco2_cos,
    output logic signed [15:0] o_vco3_sin,
    output logic signed [15:0] o_vco3_cos,
    output logic               valid
);

// dphase reset value: 19kHz at the 240kHz PLL sample rate (dsp_clk/125).
// 32-bit accumulator/dphase for fine frequency resolution (0.000056Hz/LSB);
// only the top 16 bits go to the CORDIC, which is all the angle needs.
localparam [31:0] F_INIT = 32'd340018244;

logic [31:0] ph_accum1, ph_accum2, ph_accum3, dphase;

// 1- and 2-cycle delayed strb: ph_accum1 is fresh one cycle after strb,
// ph_accum2 fresh two cycles after -- these gate the harmonic pipeline.
logic strb_d1, strb_d2;
always_ff @(posedge clk) begin
    if(~rstb) begin
        strb_d1 <= '0;
        strb_d2 <= '0;
    end else begin
        strb_d1 <= strb;
        strb_d2 <= strb_d1;
    end
end

// Fundamental on strb, 2nd harmonic one cycle later, 3rd harmonic the
// cycle after -- each needs the previous cycle's fresh accumulator.
always_ff @(posedge clk) begin
    if(~rstb) begin
        ph_accum1 <= '0;
        ph_accum2 <= '0;
        ph_accum3 <= '0;
    end else begin
        if(strb)    ph_accum1 <= ph_accum1 + dphase;
        if(strb_d1) ph_accum2 <= ph_accum1 + ph_accum1;
        if(strb_d2) ph_accum3 <= ph_accum2 + ph_accum1;
    end
end

typedef enum logic [1:0] {VCO1, VCO2, VCO3} vco_sel_t;
vco_sel_t active_vco;

logic        cordic_strb, cordic_valid;
logic [15:0] cordic_phase;
logic signed [15:0] cordic_sin, cordic_cos;

// Feeds whichever harmonic is currently loaded into the shared CORDIC --
// top 16 bits only, the wide accumulator is for frequency resolution.
always_comb begin
    unique case(active_vco)
        VCO1: cordic_phase = ph_accum1[31:16];
        VCO2: cordic_phase = ph_accum2[31:16];
        VCO3: cordic_phase = ph_accum3[31:16];
        default: cordic_phase = ph_accum1[31:16];
    endcase
end

logic vco1_ready;

// Sequences the shared CORDIC through VCO1->VCO2->VCO3: strb_d1 kicks
// off VCO1, then each cordic_valid both captures that VCO's result and
// (except after VCO3) immediately restarts CORDIC on the next harmonic.
always_ff @(posedge clk) begin
    if(~rstb) begin
        active_vco  <= VCO1;
        cordic_strb <= '0;
        vco1_ready  <= '0;
        valid       <= '0;
        o_vco1_sin <= '0; o_vco1_cos <= '0;
        o_vco2_sin <= '0; o_vco2_cos <= '0;
        o_vco3_sin <= '0; o_vco3_cos <= '0;
    end else begin
        cordic_strb <= strb_d1;
        vco1_ready  <= '0;
        valid       <= '0;

        if(cordic_valid) begin
            unique case(active_vco)
                VCO1: begin
                    o_vco1_sin  <= cordic_sin;
                    o_vco1_cos  <= cordic_cos;
                    active_vco  <= VCO2;
                    cordic_strb <= 1'b1;
                    vco1_ready  <= 1'b1;
                end
                VCO2: begin
                    o_vco2_sin  <= cordic_sin;
                    o_vco2_cos  <= cordic_cos;
                    active_vco  <= VCO3;
                    cordic_strb <= 1'b1;
                end
                VCO3: begin
                    o_vco3_sin <= cordic_sin;
                    o_vco3_cos <= cordic_cos;
                    active_vco <= VCO1;
                    valid      <= 1'b1;
                end
                default: active_vco <= VCO1;
            endcase
        end
    end
end

// Mixer: downconverts the pilot to DC, one DSP slice, fires once vco1_sin is fresh.
logic signed [31:0] pilot_mix;
always_ff @(posedge clk) begin
    if(~rstb) pilot_mix <= '0;
    else if(vco1_ready) pilot_mix <= i_data * o_vco1_sin;
end

// One cycle behind vco1_ready so the filter below reads the fresh pilot_mix, not stale.
logic vco1_ready_d1;
always_ff @(posedge clk) begin
    if(~rstb) vco1_ready_d1 <= '0;
    else      vco1_ready_d1 <= vco1_ready;
end

// Cheap 3-pole filter.
logic signed [31:0] pilot_dc, pilot_dc1, pilot_dc2;
always_ff @(posedge clk) begin
    if(~rstb) begin
        pilot_dc  <= '0;
        pilot_dc1 <= '0;
        pilot_dc2 <= '0;
    end else begin
        if(vco1_ready_d1) begin
            // 32'sd... (signed), not 32'd... -- an unsigned literal here
            // contaminates the WHOLE expression to unsigned arithmetic
            // (SV rule: any unsigned operand makes the whole expression
            // unsigned), so pilot_mix-pilot_dc1 going negative (~half of
            // real operation) got reinterpreted as a huge unsigned value
            // and >>> silently became a logical (not arithmetic) shift on
            // it -- confirmed 2026-09-18 via hand-verified trace mismatch
            // against a golden Python model (cyc 897->898: correct signed
            // math gives pilot_dc1=3468787, the bug gives the logged
            // 20246003, bit-exact match to the unsigned reinterpretation).
            pilot_dc1 <= pilot_dc1 + ((pilot_mix - pilot_dc1 + 32'sd128) >>> 8);
            pilot_dc2 <= pilot_dc2 + ((pilot_dc1 - pilot_dc2 + 32'sd256) >>> 9);
            pilot_dc  <= pilot_dc  + ((pilot_dc2 - pilot_dc  + 32'sd512) >>> 10);
        end
    end
end

// One cycle behind vco1_ready_d1 so the PI filter reads the fresh pilot_dc, not stale.
logic vco1_ready_d2;
always_ff @(posedge clk) begin
    if(~rstb) vco1_ready_d2 <= '0;
    else      vco1_ready_d2 <= vco1_ready_d1;
end

// PI loop filter: Ki=2^-KI_SHIFT (pure accumulator), Kp=2^-KP_SHIFT.
// Damping ratio ~ Kp/sqrt(Ki) -- gains chosen by sweeping in sim
// (tb_pll_sweep.sv), not derived analytically (loop gain constants
// aren't known accurately enough for that to be trustworthy).
logic signed [31:0] phase_err;
logic signed [55:0] loop_accum;
always_ff @(posedge clk) begin
    if(~rstb) begin
        loop_accum <= '0;
        phase_err  <= '0;
    end else if(vco1_ready_d2) begin
        loop_accum <= loop_accum + pilot_dc;
        phase_err  <= (pilot_dc >>> KP_SHIFT) + (loop_accum >>> KI_SHIFT);
    end
end

// One cycle behind vco1_ready_d2 so dphase's update below reads the fresh phase_err.
logic vco1_ready_d3;
always_ff @(posedge clk) begin
    if(~rstb) vco1_ready_d3 <= '0;
    else      vco1_ready_d3 <= vco1_ready_d2;
end

// Closes the loop: dphase tracks F_INIT minus phase_err (sign flipped
// 2026-09-18 to test correction polarity), unshifted now that the wide
// accumulator itself carries the fractional-LSB precision.
always_ff @(posedge clk) begin
    if(~rstb) dphase <= F_INIT;
    else if(vco1_ready_d3) dphase <= F_INIT - phase_err;
end

cordic vco_shared(
    .clk(clk),
    .rstb(rstb),
    .strb(cordic_strb),
    .phase(cordic_phase),
    .sin(cordic_sin),
    .cos(cordic_cos),
    .valid(cordic_valid)
);

endmodule
