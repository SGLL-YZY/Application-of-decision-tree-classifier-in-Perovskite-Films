// 简易 LED 拉伸：任何 strobe=1 → led 拉高约100ms
module led_stretch #(parameter CLK_HZ=125_000_000, MS=100) (
  input  wire clk, input wire rst_n, input wire strobe,
  output wire led
);
  localparam integer CNT_MAX = CLK_HZ/1000*MS;
  reg [$clog2(CNT_MAX):0] cnt;
  always @(posedge clk or negedge rst_n) begin
    if(!rst_n) cnt <= 0;
    else if(strobe) cnt <= CNT_MAX;
    else if(cnt!=0) cnt <= cnt - 1;
  end
  assign led = (cnt != 0);
endmodule
