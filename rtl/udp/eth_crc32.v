module eth_crc32 (
  input  wire       clk,
  input  wire       rst,
  input  wire       clr,      // 置初值 0xFFFF_FFFF（在SFD后对帧体起算）
  input  wire       en,       // 输入使能（对每个字节拉高1拍）
  input  wire [7:0] din,      // 帧体字节（从DA开始，到payload结束）
  output reg [31:0] crc       // 当前CRC寄存器（反向取反前）
);
  // 反射多项式 0xEDB88320，按低位先行更新
  function [31:0] crc32_update8;
    input [31:0] c;
    input [7:0]  d;
    integer i;
    reg [31:0] x;
    reg [7:0]  b;
  begin
    x = c; b = d;
    for (i=0;i<8;i=i+1) begin
      if ((x[0] ^ b[0]) == 1'b1) x = (x >> 1) ^ 32'hEDB88320;
      else                       x = (x >> 1);
      b = b >> 1;
    end
    crc32_update8 = x;
  end
  endfunction

  always @(posedge clk) begin
    if (rst)         crc <= 32'hFFFF_FFFF;
    else if (clr)    crc <= 32'hFFFF_FFFF;
    else if (en)     crc <= crc32_update8(crc, din);
  end
endmodule
