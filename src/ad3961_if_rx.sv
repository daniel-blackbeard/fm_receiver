module ad3961_if_rx (
    output logic       dsp_clk,
    // AD3961 side
    input  logic       rx_clk_in_p,
    input  logic       rx_clk_in_n,
    input  logic       rx_frame_in_p,
    input  logic       rx_frame_in_n,
    input  logic [5:0] rx_data_in_p,
    input  logic [5:0] rx_data_in_n,

    // PL side
    input  logic        adc_r1_mode,
    output logic        adc_valid,
    output logic [11:0] adc_data_i1,
    output logic [11:0] adc_data_q1,
    output logic [11:0] adc_data_i2,
    output logic [11:0] adc_data_q2,
    output logic        adc_status,

    // Raw pre-decode taps for RX-lock diagnosis -- rx_data/rx_frame_s as
    // seen by the frame-match logic itself, ungated by adc_valid/
    // adc_r1_mode, so they're visible even when frame lock never
    // acquires. Plain wire taps on already-registered internal signals,
    // no new logic.
    output logic [11:0] dbg_rx_data,
    output logic [3:0]  dbg_rx_frame_s
);



logic data_clk, data_clk_ibuff;
assign dsp_clk = data_clk;

logic        [ 5:0]  rx_data_n_s_d   = 'd0;
logic                rx_frame_n_s_d  = 'd0;
logic        [11:0]  rx_data         = 'd0;
logic        [ 1:0]  rx_frame        = 'd0;
logic        [11:0]  rx_data_d       = 'd0;
logic        [ 1:0]  rx_frame_d      = 'd0;

logic                rx_error_r1     = 'd0;
logic                rx_valid_r1     = 'd0;
logic                rx_error_r2     = 'd0;
logic                rx_valid_r2     = 'd0;
logic signed [11:0]  rx_data_i_r1    = 'd0;
logic signed [11:0]  rx_data_q_r1    = 'd0;
logic signed [11:0]  rx_data_i1_r2   = 'd0;
logic signed [11:0]  rx_data_q1_r2   = 'd0;
logic signed [11:0]  rx_data_i2_r2   = 'd0;
logic signed [11:0]  rx_data_q2_r2   = 'd0;

logic        [ 5:0]  rx_data_p_s;
logic        [ 5:0]  rx_data_n_s;
logic        [ 5:0]  rx_data_ibuf;
logic                rx_frame_p_s;
logic                rx_frame_n_s;
logic                rx_frame_ibuf;

logic        [ 3:0]  rx_frame_s;

// Register the sampled frame data
assign rx_frame_s = {rx_frame_d, rx_frame};
assign dbg_rx_frame_s = rx_frame_s;
assign dbg_rx_data    = rx_data;
always @ (posedge data_clk)begin
    rx_frame_n_s_d  <= rx_frame_n_s;
    rx_frame_d      <= rx_frame;  
    rx_frame        <= {rx_frame_n_s_d,rx_frame_p_s};
end

// Register the sampled data
always @ (posedge data_clk)begin
    rx_data_n_s_d   <= rx_data_n_s;
    rx_data_d       <= rx_data;
    rx_data         <= {rx_data_n_s_d,rx_data_p_s};
end

// Single-RF receive data path; expected frame matches only I/Q MSB
always @ (posedge data_clk)
begin
    rx_error_r1 <= ((rx_frame_s==4'b1100)||(rx_frame_s==4'b0011)) ? 1'b0:1'b1;
    rx_valid_r1 <= (rx_frame_s==4'b1100) ? 1'b1:1'b0;
    if (rx_frame_s==4'b1100)
        begin
            rx_data_i_r1 <= {rx_data_d[11:6],rx_data[11:6]};
            rx_data_q_r1 <= {rx_data_d[ 5:0],rx_data[ 5:0]};
        end
end

// Dual-RF receive data path; expected frame applies only to RF-1's I/Q MSB and LSB (this is the one used)
always @ (posedge data_clk)
begin
    rx_error_r2<=((rx_frame_s==4'b1111)||(rx_frame_s==4'b1100)||(rx_frame_s==4'b0000)||(rx_frame_s== 4'b0011)) ? 1'b0 : 1'b1;
    rx_valid_r2<=(rx_frame_s==4'b0000) ? 1'b1 : 1'b0;
    if(rx_frame_s==4'b1111)
        begin
            rx_data_i1_r2 <= {rx_data_d[11:6],rx_data[11:6]};
            rx_data_q1_r2 <= {rx_data_d[ 5:0],rx_data[ 5:0]};
        end
    if(rx_frame_s==4'b0000)
        begin
            rx_data_i2_r2 <= {rx_data_d[11:6],rx_data[11:6]};
            rx_data_q2_r2 <= {rx_data_d[ 5:0],rx_data[ 5:0]};
        end
end

// Select the output data according to the receive mode
always @ (posedge data_clk)
begin
    if(adc_r1_mode == 1'b1)
        begin
            adc_valid   <= rx_valid_r1;
            adc_data_i1 <= rx_data_i_r1;
            adc_data_q1 <= rx_data_q_r1;
            adc_data_i2 <= 12'd0;
            adc_data_q2 <= 12'd0;
            adc_status  <= ~rx_error_r1;
        end
    else
        begin
            adc_valid   <= rx_valid_r2;
            adc_data_i1 <= rx_data_i1_r2;
            adc_data_q1 <= rx_data_q1_r2;
            adc_data_i2 <= rx_data_i2_r2;
            adc_data_q2 <= rx_data_q2_r2;
            adc_status  <= ~rx_error_r2;
        end
end  

// Clock conversion from LVDS to fabric
IBUFDS IBUFDS_data_clk_inst (
    .I          (rx_clk_in_p),
    .IB         (rx_clk_in_n),
    .O          (data_clk_ibuff)
);
// Clock enabling stage (wired to always enabled for now)
BUFGCE BUFGCE_data_clk_inst (
    .CE         (1'b1 ),
    .I          (data_clk_ibuff),
    .O          (data_clk)
);

// Convert receive data from differential to single-ended, IDDR sampling
genvar l_inst;

generate
    for (l_inst=0; l_inst<=5; l_inst=l_inst+1) begin: g_rx_data
        IBUFDS i_rx_data_ibuf(
            .I(rx_data_in_p[l_inst]),
            .IB(rx_data_in_n[l_inst]),
            .O(rx_data_ibuf[l_inst])
        );
        
        IDDR #(
            .DDR_CLK_EDGE("OPPOSITE_EDGE"),
            .INIT_Q1(1'b0),
            .INIT_Q2(1'b0),
            .SRTYPE("SYNC")
        )     
        i_rx_data_iddr  (
            .R(1'b0),
            .S(1'b0),
            .C(data_clk),
            .CE(1'b1),
            .D(rx_data_ibuf[l_inst]),
            .Q1(rx_data_p_s[l_inst]),
            .Q2(rx_data_n_s[l_inst])
        );
    end 
endgenerate    

// Convert receive frame clock from differential to single-ended, IDDR sampling
IBUFDS i_rx_frame_ibuf (
    .I              (rx_frame_in_p),
    .IB             (rx_frame_in_n),
    .O              (rx_frame_ibuf)
);

IDDR #(
    .DDR_CLK_EDGE   ("OPPOSITE_EDGE"),
    .INIT_Q1        (1'b0),
    .INIT_Q2        (1'b0),
    .SRTYPE         ("SYNC")  
)
i_rx_frame_iddr ( 
    .R              (1'b0                   ),
    .S              (1'b0                   ),
    .CE             (1'b1                   ),
    .C              (data_clk               ),
    .D              (rx_frame_ibuf          ),
    .Q1             (rx_frame_p_s           ),
    .Q2             (rx_frame_n_s           )
);

endmodule