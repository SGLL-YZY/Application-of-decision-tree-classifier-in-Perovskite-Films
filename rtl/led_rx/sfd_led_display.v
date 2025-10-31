// sfd_led_display.v
module sfd_led_display #(
  parameter integer CLK_HZ    = 25_000_000, // 这里用 clk_25m
  parameter integer WINDOW_MS = 100         // 刷新窗口 100ms
)(
  input  wire       clk,        // 显示时钟，例如 clk_25m
  input  wire       rst_n,
  // 来自 RGMII RX 域
  input  wire       rx_clk,     // phy1_rgmii_rx_clk
  input  wire       sfd_hit_rx, // eth_sfd_probe 的输出（rx_clk 域）
  // 4 个 LED，高电平亮
  output reg  [3:0] led
);
  // --- CDC: toggle-同步-边沿检测 ---
  reg toggle_rx;
  always @(posedge rx_clk or negedge rst_n)
    if(!rst_n) toggle_rx <= 1'b0;
    else if (sfd_hit_rx) toggle_rx <= ~toggle_rx;

  reg [2:0] sync_ff;
  always @(posedge clk or negedge rst_n)
    if(!rst_n) sync_ff <= 3'b000;
    else       sync_ff <= {sync_ff[1:0], toggle_rx};

  wire sfd_pulse = sync_ff[2] ^ sync_ff[1]; // clk 域单拍

  // --- 100ms 窗口累加显示 ---
  localparam integer TICK_CYC = (CLK_HZ/1000)*WINDOW_MS;
  reg [31:0] tick_cnt;
  reg [3:0]  bucket;

  always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
      tick_cnt <= 0;
      bucket   <= 0;
      led      <= 4'b0000;
    end else begin
      // 事件计数（最多 15）
      if (sfd_pulse && bucket != 4'd15)
        bucket <= bucket + 1'b1;

      // 定时刷新，显示并清零
      if (tick_cnt == TICK_CYC-1) begin
        tick_cnt <= 0;
        led      <= bucket;   // 直接显示 0~15
        bucket   <= 0;
      end else begin
        tick_cnt <= tick_cnt + 1'b1;
      end
    end
  end
endmodule
