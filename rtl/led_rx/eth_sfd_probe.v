// eth_sfd_probe.v
module eth_sfd_probe(
  input  wire       clk,    // 用 phy1_rgmii_rx_clk
  input  wire       rst_n,
  input  wire       vld,    // gmii_rx_dv
  input  wire [7:0] din,    // gmii_rxd
  output reg        sfd_hit // 单拍脉冲：检测到 SFD=0xD5
);
  reg [2:0] pre_cnt;
  always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
      pre_cnt <= 3'd0;
      sfd_hit <= 1'b0;
    end else begin
      sfd_hit <= 1'b0;
      if (vld) begin
        if (pre_cnt < 3'd7) begin
          pre_cnt <= (din==8'h55) ? pre_cnt+1'b1 : 3'd0;
        end else begin
          if (din==8'hD5) begin
            sfd_hit <= 1'b1;
            pre_cnt <= 3'd0;
          end else begin
            pre_cnt <= (din==8'h55) ? 3'd1 : 3'd0;
          end
        end
      end else begin
        pre_cnt <= 3'd0;
      end
    end
  end
endmodule
