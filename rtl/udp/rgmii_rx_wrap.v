// rgmii_rx_wrap.v  ——  RGMII → GMII风格字节流
module rgmii_rx_wrap #(
  parameter DEVICE           = "EG4",   // 传给你的 IDDR
  parameter RISE_LOW_NIBBLE  = 1        // 1: 上沿=低4位；0: 上沿=高4位（如发现半字节颠倒改这里）
)(
  input  wire        phy_rxc,     // RGMII RX clock (125/25/2.5 MHz)
  input  wire        phy_rx_ctl,  // RGMII RX_CTL
  input  wire [3:0]  phy_rxd,     // RGMII RXD
  input  wire        rst,         // 与 IDDR 配套的复位

  output reg  [7:0]  gmii_rxd,    // 每个上升沿出一个字节
  output reg         gmii_rx_dv,  // 数据有效
  output reg         gmii_rx_er   // 错误
);

  wire [3:0] d_rise, d_fall;
  wire       c_rise, c_fall;

  genvar i;
  generate
    for (i=0;i<4;i=i+1) begin: G_IDDR_D
      // 你的语义：q0=posedge, q1=negedge
      IDDR #(.ASYNCRST("ENABLE"), .PIPEMODE("PIPED"), .DEVICE(DEVICE)) u_iddr_d (
        .q0 (d_rise[i]),   // 上升沿为低位
        .q1 (d_fall[i]),   // 下降沿为高位
        .clk(phy_rxc),
        .d  (phy_rxd[i]),
        .rst(rst)
      );
    end
    IDDR #(.ASYNCRST("ENABLE"), .PIPEMODE("PIPED"), .DEVICE(DEVICE)) u_iddr_ctl (
      .q0 (c_rise),        // 上升沿
      .q1 (c_fall),        // 下降沿
      .clk(phy_rxc),
      .d  (phy_rx_ctl),
      .rst(rst)
    );
  endgenerate

  wire [3:0] lo = RISE_LOW_NIBBLE ? d_rise : d_fall;    //上升沿采低四位
  wire [3:0] hi = RISE_LOW_NIBBLE ? d_fall : d_rise;    //下降沿采高四位

  always @(posedge phy_rxc or posedge rst) begin
    if (rst) begin
      gmii_rxd   <= 8'h00;
      gmii_rx_dv <= 1'b0;
      gmii_rx_er <= 1'b0;
    end else begin
      gmii_rxd   <= {hi, lo};
      gmii_rx_dv <= c_rise;          // RX_DV = RX_CTL(rise)
      gmii_rx_er <= c_rise ^ c_fall; // RX_ER = rise ^ fall
    end
  end

endmodule
