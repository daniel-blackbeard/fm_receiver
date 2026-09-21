module rds (
    input  logic               clk,
    input  logic               rstb,
    input  logic signed [15:0] i_mpx_data,
    input  logic               strb,
    output logic signed [15:0] o_rds_data,
    output logic               valid,
    output logic signed [15:0] o_lpf_data,
    output logic               o_lpf_valid,
    output logic        [31:0] o_nco_phase,
    output logic               o_nco_valid,
    output logic signed [23:0] o_soft,
    output logic               o_biphase,
    output logic               o_soft_valid,
    output logic               o_data_bit,
    output logic               o_data_valid,
    output logic         [9:0] o_syndrome,
    output logic         [2:0] o_offset_type,
    output logic               o_syn_valid,
    output logic        [15:0] o_block_data,
    output logic         [2:0] o_block_type,
    output logic               o_block_valid,
    output logic               o_locked,
    output logic        [15:0] o_pi,
    output logic        [63:0] o_ps,
    output logic       [511:0] o_rt
);

localparam int OUT_SHIFT = 12;

logic signed [27:0] out_rounded;
logic signed [27:0] int1, int2, int3, int4, int5;
logic signed [27:0] com1, com2, com3, com4, com5;
logic signed [27:0] int5_d1, com1_d1, com2_d1, com3_d1, com4_d1;
logic         [2:0] counter;

logic decim_event;
assign decim_event = strb && (counter == 3'b0);

// First thing: decimate by 5 to match audio rate to use the same channel
always_ff @(posedge clk) begin
    if(~rstb) begin
        valid      <= '0;
        int1       <= '0;
        int2       <= '0;
        int3       <= '0;
        int4       <= '0;
        int5       <= '0;
        com1       <= '0;
        com2       <= '0;
        com3       <= '0;
        com4       <= '0;
        com5       <= '0;
        int5_d1    <= '0;
        com1_d1    <= '0;
        com2_d1    <= '0;
        com3_d1    <= '0;
        com4_d1    <= '0;
        counter    <= '0;
    end else begin
        valid <= decim_event;
        if(strb) begin
            counter <= (counter == 3'b100) ? 3'b0 : counter + 3'b1;
            int1 <= int1 + i_mpx_data;
            int2 <= int2 + int1;
            int3 <= int3 + int2;
            int4 <= int4 + int3;
            int5 <= int5 + int4;
            if(counter == 3'b0) begin
                com1 <= int5 - int5_d1; int5_d1 <= int5;
                com2 <= com1 - com1_d1; com1_d1 <= com1;
                com3 <= com2 - com2_d1; com2_d1 <= com2;
                com4 <= com3 - com3_d1; com3_d1 <= com3;
                com5 <= com4 - com4_d1; com4_d1 <= com4;
            end
        end
    end
end

assign out_rounded = (com5 + (28'sd1 <<< (OUT_SHIFT-1))) >>> OUT_SHIFT;
assign o_rds_data  = out_rounded[15:0];

// -------------- IIR Butter 2nd order ---------------//

logic signed [15:0] rds_data_m1, rds_data_m2;
logic signed [17:0] iir_tap;
logic signed [19:0] lpf_data_m1, lpf_data_m2, lpf_data_new, iir_value;
logic signed [39:0] iir_accum;
logic [2:0] rds_iir_fsm;

localparam iir_b0 =  18'sd1417;
localparam iir_b1 =  18'sd2834;
localparam iir_b2 =  18'sd1417;
localparam iir_a1 =  18'sd101130;
localparam iir_a2 = -18'sd41262;

assign lpf_data_new = (iir_accum + 40'sd32768) >>> 16;

always_ff @(posedge clk) begin : IIR_BUTTER2
    if(~rstb) begin
        o_lpf_data  <= '0;
        rds_data_m1 <= '0;
        rds_data_m2 <= '0;
        lpf_data_m1 <= '0;
        lpf_data_m2 <= '0;
        o_lpf_valid <= '0;
        rds_iir_fsm <= '0;
        iir_tap     <= '0;
        iir_value   <= '0;
        iir_accum   <= '0;
    end else begin
        o_lpf_valid <= 1'b0;
        case(rds_iir_fsm)
            3'b000 : begin rds_iir_fsm <= valid ? 3'b001 : 3'b000; iir_accum<= 32'b0; end
            3'b001 : begin rds_iir_fsm <= 3'b010; iir_tap <= iir_b0; iir_value <= o_rds_data  <<< 4; end
            3'b010 : begin rds_iir_fsm <= 3'b011; iir_tap <= iir_b1; iir_value <= rds_data_m1 <<< 4; iir_accum <= iir_accum + iir_tap * iir_value; end
            3'b011 : begin rds_iir_fsm <= 3'b100; iir_tap <= iir_b2; iir_value <= rds_data_m2 <<< 4; iir_accum <= iir_accum + iir_tap * iir_value; end
            3'b100 : begin rds_iir_fsm <= 3'b101; iir_tap <= iir_a1; iir_value <= lpf_data_m1;       iir_accum <= iir_accum + iir_tap * iir_value; end
            3'b101 : begin rds_iir_fsm <= 3'b110; iir_tap <= iir_a2; iir_value <= lpf_data_m2;       iir_accum <= iir_accum + iir_tap * iir_value; end
            3'b110 : begin rds_iir_fsm <= 3'b111;                                                    iir_accum <= iir_accum + iir_tap * iir_value; end
            3'b111 : begin
                         rds_data_m1 <= o_rds_data; rds_data_m2 <= rds_data_m1;
                         lpf_data_m1 <= lpf_data_new; lpf_data_m2 <= lpf_data_m1;
                         o_lpf_data  <= (lpf_data_new + 4'sd8) >>> 4;
                         rds_iir_fsm <= 3'b000;
                         o_lpf_valid <= 1'b1;
                     end
        endcase
    end
end

// ----------------- NCO symbol ------------------//
localparam logic [31:0] NCO_STEP   = 32'h06555555;  // hardcoded for this use
localparam logic [31:0] NCO_PHASE0 = 32'hB6C00000;

logic [31:0] nco_acc;
always_ff @(posedge clk) begin
    if(~rstb) begin
        nco_acc <= NCO_PHASE0; o_nco_valid <= 1'b0;
    end else begin
        o_nco_valid <= o_lpf_valid;
        if(o_lpf_valid) begin
            o_nco_phase <= nco_acc;
            nco_acc     <= nco_acc + NCO_STEP;
        end
    end
end

// ----------------- Biphase soft/bit ------------------//

logic nco_acc_msb;
logic signed [23:0] acc_soft;
always_ff @(posedge clk) begin : BIPHASE_SOFT_BIT
    if(~rstb) begin
        nco_acc_msb  <= '0;
        acc_soft     <= '0;
        o_soft       <= '0;
        o_soft_valid <= '0;
        o_biphase    <= '0;
    end else begin
        o_soft_valid <= 1'b0;
        if(o_lpf_valid) begin
            nco_acc_msb <= nco_acc[31];
            if(nco_acc_msb & ~nco_acc[31]) begin
                o_soft   <= acc_soft;
                o_soft_valid <= 1'b1;
                o_biphase    <= ~acc_soft[21];
                acc_soft <= o_lpf_data;
            end else begin
                acc_soft <= nco_acc[31] ? acc_soft - o_lpf_data : acc_soft + o_lpf_data;
            end
        end 
    end
end

// ----------------- Differential Decode ------------------//
logic biph_prev;
always_ff @(posedge clk) begin : DIFF_DEC
    if(~rstb) begin
        biph_prev  <= '0;
        o_data_bit <= '0;
    end else begin
        o_data_valid <= o_soft_valid;
        if(o_soft_valid) begin
            biph_prev  <= o_biphase;
            o_data_bit <= o_biphase ^ biph_prev;
        end
    end
end

// ------------------- Syndrome --------------------//
logic [25:0] syn_barrel;
logic [9:0] lfsr, lfsr_calc;
assign o_syndrome = lfsr;
assign lfsr_calc = {lfsr[8], lfsr[7] ^ lfsr[9], lfsr[6] ^ lfsr[9], lfsr[5], 
                             lfsr[4] ^ lfsr[9], lfsr[3] ^ lfsr[9], lfsr[2] ^ lfsr[9], lfsr[1], 
                             lfsr[0],           lfsr[9] ^ o_data_bit};

always_ff @(posedge clk) begin : SYNDROME
    if(~rstb) begin
        syn_barrel  <= '0;
        lfsr        <= '0;
        o_syn_valid <= '0;
    end else begin
        o_syn_valid <= o_data_valid;
        if(o_data_valid) begin
            syn_barrel <= {syn_barrel[24:0], o_data_bit};
            lfsr <= syn_barrel[25] ? lfsr_calc ^ 10'hEE : lfsr_calc;
        end
    end
end

// ------------------- Match Offset --------------------//

localparam OW_A = 10'h0FC;
localparam OW_B = 10'h198;
localparam OW_C = 10'h168;
localparam OW_Z = 10'h350;
localparam OW_D = 10'h1B4;

always_comb begin : OFFSET_MATCH
    case(o_syndrome)
        OW_A    : o_offset_type  <= 3'b001;
        OW_B    : o_offset_type  <= 3'b010;
        OW_C    : o_offset_type  <= 3'b011;
        OW_Z    : o_offset_type  <= 3'b100;
        OW_D    : o_offset_type  <= 3'b101;
        default : o_offset_type  <= 3'b000;
    endcase
end

// ----------------- Block Sync/Lock ------------------//
// Hunting : lock when the offset types at bits i-52, i-26 and i (now) all hit and
//           follow the legal group order A>B>C|C'>D>A. The block that completes the
//           run is the first one emitted.
// Locked  : every 26 bits the next block must be a legal successor of the previous
//           one -> emit it; otherwise drop lock and hunt again with an empty history.

localparam logic [2:0] OFFSET_A  = 3'd1;
localparam logic [2:0] OFFSET_B  = 3'd2;
localparam logic [2:0] OFFSET_C  = 3'd3;
localparam logic [2:0] OFFSET_CP = 3'd4;
localparam logic [2:0] OFFSET_D  = 3'd5;

logic [155:0] type_hist;     // offset type of the last 52 bits, newest in [2:0]
logic   [4:0] blk_cnt;       // bits elapsed since the last block while locked
logic   [2:0] last_type;

logic [2:0] t_now, t_m26, t_m52;
logic       ok_52_26, ok_26_now, ok_last_now;

assign t_now = o_offset_type;
assign t_m26 = type_hist[77:75];      // type 26 bits ago
assign t_m52 = type_hist[155:153];    // type 52 bits ago

always_comb begin : SYNC_SEQUENCE_CHECK
    ok_52_26    = ((t_m52 == OFFSET_A) & (t_m26 == OFFSET_B))
                | ((t_m52 == OFFSET_B) & ((t_m26 == OFFSET_C) | (t_m26 == OFFSET_CP)))
                | (((t_m52 == OFFSET_C) | (t_m52 == OFFSET_CP)) & (t_m26 == OFFSET_D))
                | ((t_m52 == OFFSET_D) & (t_m26 == OFFSET_A));
    ok_26_now   = ((t_m26 == OFFSET_A) & (t_now == OFFSET_B))
                | ((t_m26 == OFFSET_B) & ((t_now == OFFSET_C) | (t_now == OFFSET_CP)))
                | (((t_m26 == OFFSET_C) | (t_m26 == OFFSET_CP)) & (t_now == OFFSET_D))
                | ((t_m26 == OFFSET_D) & (t_now == OFFSET_A));
    ok_last_now = ((last_type == OFFSET_A) & (t_now == OFFSET_B))
                | ((last_type == OFFSET_B) & ((t_now == OFFSET_C) | (t_now == OFFSET_CP)))
                | (((last_type == OFFSET_C) | (last_type == OFFSET_CP)) & (t_now == OFFSET_D))
                | ((last_type == OFFSET_D) & (t_now == OFFSET_A));
end

always_ff @(posedge clk) begin : SYNC_LOCK
    if(~rstb) begin
        type_hist     <= '0;
        blk_cnt       <= '0;
        last_type     <= '0;
        o_locked      <= 1'b0;
        o_block_valid <= 1'b0;
        o_block_data  <= '0;
        o_block_type  <= '0;
    end else begin
        o_block_valid <= 1'b0;
        if(o_syn_valid) begin
            type_hist <= {type_hist[152:0], t_now};
            if(o_locked) begin
                if(blk_cnt == 5'd25) begin              // this bit ends the next block
                    blk_cnt <= '0;
                    if(ok_last_now) begin
                        o_block_data  <= syn_barrel[25 -: 16];   // oldest 16 bits of the window
                        o_block_type  <= t_now;
                        o_block_valid <= 1'b1;
                        last_type     <= t_now;
                    end else begin
                        o_locked  <= 1'b0;
                        type_hist <= '0;
                    end
                end else begin
                    blk_cnt <= blk_cnt + 5'd1;
                end
            end else if(ok_52_26 & ok_26_now) begin
                o_locked      <= 1'b1;
                blk_cnt       <= '0;
                last_type     <= t_now;
                o_block_data  <= syn_barrel[25 -: 16];
                o_block_type  <= t_now;
                o_block_valid <= 1'b1;
            end
        end
    end
end

// ----------------- PI/PS/RadioText ------------------//

logic  [3:0] rt_state;
logic [11:0] b_segment;
logic [15:0] c_data;

always_ff @(posedge clk) begin : PS_PI_RADIOTEXT
    if(~rstb) begin
        rt_state  <= 4'b0;
        b_segment <= '0;
        c_data    <= '0;
        o_pi      <= '0;
        o_ps      <= '0;
        o_rt      <= '0;
    end else begin
        case(rt_state)
            4'b0000 : begin if(o_block_valid & (o_block_type == 3'b001)) begin rt_state <= 4'b0001; o_pi <= o_block_data; end end // A received
            4'b0001 : begin if(o_block_valid) rt_state <= (o_block_type == 3'b010) ? 4'b0010 : 4'b0000;                            end      // B received
            4'b0010 : begin rt_state <= o_block_data[15:12] == 4'b0010 ? 4'b0100 : 4'b0011; b_segment <= o_block_data[11:0];       end
            4'b0011 : begin if(o_block_valid) begin rt_state <= ((o_block_type == 3'b011) | (o_block_type == 3'b100)) ? 4'b0101 : 4'b0000; c_data <= o_block_data; end end
            4'b0100 : begin if(o_block_valid) begin rt_state <= ((o_block_type == 3'b011) | (o_block_type == 3'b100)) ? 4'b0110 : 4'b0000; c_data <= o_block_data; end end
            4'b0101 : begin if(o_block_valid) begin rt_state <= (o_block_type == 3'b101) ? 4'b0111 : 4'b0000;      end end // C received
            4'b0110 : begin if(o_block_valid) begin rt_state <= (o_block_type == 3'b101) ? 4'b1000 : 4'b0000;      end end // C received
            4'b0111 : begin rt_state <= 4'b0000; o_ps[63 - 16*b_segment[1:0] -: 16] <= o_block_data;                   end
            4'b1000 : begin rt_state <= 4'b0000; o_rt[511 - 32*b_segment[3:0] -: 32] <= {c_data, o_block_data};        end
            default : begin rt_state <= 4'b0000; end
        endcase
    end
end

//assign o_pi          = '0;
//assign o_ps          = '0;
//assign o_rt          = '0;


endmodule