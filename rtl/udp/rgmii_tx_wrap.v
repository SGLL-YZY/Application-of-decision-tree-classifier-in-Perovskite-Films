// rgmii_tx_wrap.v  ——  GMII风格字节流 → RGMII
module rgmii_tx_wrap #(
  parameter DEVICE          = "EG4",
  parameter RISE_LOW_NIBBLE = 1    // 1: 上沿=低4位；与 RX 保持一致
)(
  input  wire        tx_clk,       // 125/25/2.5 MHz 数据时钟（0°）
  input  wire        tx_clk_90,    // 125/25/2.5 MHz 90° 相移（用于 TXC，如你的板卡做法）
  input  wire        rst,

  input  wire [7:0]  gmii_txd,
  input  wire        gmii_tx_en,
  input  wire        gmii_tx_er,
  input  wire        speed_10_100, // 1: 10/100M；0: 1G

  output wire        phy_txc,
  output wire        phy_tx_ctl,
  output wire [3:0]  phy_txd
);

  // 半字节选择（上沿发 lo）
  wire [3:0] lo = RISE_LOW_NIBBLE ? gmii_txd[3:0] : gmii_txd[7:4];
  wire [3:0] hi_1g = RISE_LOW_NIBBLE ? gmii_txd[7:4] : gmii_txd[3:0];
  wire [3:0] fall_nib = speed_10_100 ? lo : hi_1g; // 10/100M 下降沿复用低半字节

  // TXC：ODDR 产生 50% 方波（建议用 tx_clk_90，匹配你原工程）
  ODDR #(.ASYNCRST("ENABLE"), .DEVICE(DEVICE)) u_oddr_txc (
    .q   (phy_txc),
    .clk (tx_clk_90),
    .d0  (1'b1),   // 上升沿输出 1
    .d1  (1'b0),   // 下降沿输出 0
    .rst (rst)
  );

  // TX_CTL = TX_EN ^ TX_ER（RGMII v2.0）
  wire tx_en_to_ddr = gmii_tx_en;            // 你的工程还有个门控，这里先保持简单
  wire tx_ctl_rise  = tx_en_to_ddr;          // 上沿送 TX_EN
  wire tx_ctl_fall  = tx_en_to_ddr ^ gmii_tx_er; // 下降沿送 TX_EN ^ TX_ER

  ODDR #(.ASYNCRST("ENABLE"), .DEVICE(DEVICE)) u_oddr_ctl (
    .q   (phy_tx_ctl),
    .clk (tx_clk),
    .d0  (tx_ctl_rise),   // 上沿
    .d1  (tx_ctl_fall),   // 下降沿
    .rst (rst)
  );

  // 数据 TXD[3:0]
  genvar i;
  generate
    for (i=0;i<4;i=i+1) begin : G_ODDR_D
      ODDR #(.ASYNCRST("ENABLE"), .DEVICE(DEVICE)) u_oddr_d (
        .q   (phy_txd[i]),
        .clk (tx_clk),
        .d0  (lo[i]),       // 上沿：低半字节
        .d1  (fall_nib[i]), // 下降沿：1G=高半字节；10/100=低半字节
        .rst (rst)
      );
    end
  endgenerate

endmodule
