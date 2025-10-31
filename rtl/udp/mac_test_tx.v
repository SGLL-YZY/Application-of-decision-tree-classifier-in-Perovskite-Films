// IPv4/UDP 广播发包（IP 头校验和 OK，UDP 校验和置 0，带以太网 FCS）
// 1000BASE-T: 125MHz GMII
module udp_broadcast_tx #(
  parameter [47:0] SRC_MAC       = 48'h02_11_22_33_44_55,
  parameter [31:0] SRC_IP        = 32'hC0A8_F001,     // 192.168.240.1
  parameter [15:0] SRC_PORT      = 16'd6001,
  parameter [47:0] DST_MAC       = 48'hFF_FF_FF_FF_FF_FF, // L2 广播
  parameter [31:0] DST_IP        = 32'hC0A8_F0FF,     // 192.168.240.255
  parameter [15:0] DST_PORT      = 16'd6002,
  parameter integer PAYLOAD_BYTES= 512                // <= 1472
)(
  input  wire       clk,       // 125 MHz
  input  wire       rst,       // 同步高复位
  output reg  [7:0] gmii_txd,
  output reg        gmii_tx_en,
  output wire       gmii_tx_er
);
  assign gmii_tx_er = 1'b0;

  // 以太网 FCS（CRC32，反射，多项式0xEDB88320）
  reg [31:0] crc;
  function [31:0] crc32_d8;
    input [31:0] c;
    input [7:0]  d;
    integer i;
    reg [31:0] cur;
    reg [7:0]   b;
  begin
    cur = c; b = d;
    for (i=0;i<8;i=i+1) begin
      if ((cur[0]^b[0])==1'b1) cur = (cur>>1) ^ 32'hedb88320;
      else                     cur = (cur>>1);
      b = b>>1;
    end
    crc32_d8 = cur;
  end
  endfunction

  // IPv4 头字段
  localparam [7:0]  IP_VER_IHL = 8'h45;
  localparam [7:0]  IP_TOS     = 8'h00;
  localparam [15:0] IP_ID      = 16'h0000;
  localparam [15:0] IP_FRAG    = 16'h0000; // 不分片
  localparam [7:0]  IP_TTL     = 8'd64;
  localparam [7:0]  IP_PROTO   = 8'd17;    // UDP

  // UDP/IP 长度
  localparam integer UDP_HDR_BYTES = 8;
  localparam integer IP_HDR_BYTES  = 20;
  localparam integer L3_BYTES      = IP_HDR_BYTES + UDP_HDR_BYTES + PAYLOAD_BYTES;

  // IPv4 头校验和（一次性求和）
  function [15:0] ip_checksum;
    input [15:0] w0,w1,w2,w3,w4,w5,w6,w7,w8,w9;
    reg [31:0] s;
  begin
    s = w0+w1+w2+w3+w4+w5+w6+w7+w8+w9;
    s = (s[15:0] + s[31:16]);
    s = (s[15:0] + (s>>16));
    ip_checksum = ~s[15:0];
  end
  endfunction

  wire [15:0] ip_tot_len = L3_BYTES[15:0];
  wire [15:0] ip_chk = ip_checksum(
      {IP_VER_IHL,IP_TOS},
      ip_tot_len,
      IP_ID,
      IP_FRAG,
      {IP_TTL,IP_PROTO},
      16'h0000,
      SRC_IP[31:16],
      SRC_IP[15:0],
      DST_IP[31:16],
      DST_IP[15:0]
  );

  wire [15:0] udp_len = (UDP_HDR_BYTES + PAYLOAD_BYTES);

  // 状态机
  localparam S_IDLE=0,S_PREAM=1,S_SFD=2,S_ETH=3,S_IP=4,S_UDP=5,S_PAY=6,S_FCS=7,S_IFG=8;
  reg [3:0] st;

  // 计数
  reg [5:0] pre_cnt;
  reg [5:0] eth_cnt;   // 14
  reg [5:0] ip_cnt;    // 20
  reg [3:0] udp_cnt;   // 8
  reg [15:0] pay_cnt;  // payload
  reg [2:0] fcs_cnt;   // 4
  reg [5:0] ifg_cnt;   // 12

  // 多路选择：以太网头（DST MAC + SRC MAC + 0x0800）
  reg [7:0] eth_byte;
  always @(*) begin
    case (eth_cnt)
      0: eth_byte = DST_MAC[47:40];
      1: eth_byte = DST_MAC[39:32];
      2: eth_byte = DST_MAC[31:24];
      3: eth_byte = DST_MAC[23:16];
      4: eth_byte = DST_MAC[15:8 ];
      5: eth_byte = DST_MAC[7 :0 ];
      6: eth_byte = SRC_MAC[47:40];
      7: eth_byte = SRC_MAC[39:32];
      8: eth_byte = SRC_MAC[31:24];
      9: eth_byte = SRC_MAC[23:16];
      10:eth_byte = SRC_MAC[15:8 ];
      11:eth_byte = SRC_MAC[7 :0 ];
      12:eth_byte = 8'h08;              // EtherType = 0x0800 (IPv4)
      13:eth_byte = 8'h00;
      default: eth_byte = 8'h00;
    endcase
  end

  // IPv4 头逐字节
  reg [7:0] ip_byte;
  always @(*) begin
    case (ip_cnt)
      0:  ip_byte = IP_VER_IHL;
      1:  ip_byte = IP_TOS;
      2:  ip_byte = ip_tot_len[15:8];
      3:  ip_byte = ip_tot_len[7:0];
      4:  ip_byte = IP_ID[15:8];
      5:  ip_byte = IP_ID[7:0];
      6:  ip_byte = IP_FRAG[15:8];
      7:  ip_byte = IP_FRAG[7:0];
      8:  ip_byte = IP_TTL;
      9:  ip_byte = IP_PROTO;
      10: ip_byte = ip_chk[15:8];
      11: ip_byte = ip_chk[7:0];
      12: ip_byte = SRC_IP[31:24];
      13: ip_byte = SRC_IP[23:16];
      14: ip_byte = SRC_IP[15:8];
      15: ip_byte = SRC_IP[7:0];
      16: ip_byte = DST_IP[31:24];
      17: ip_byte = DST_IP[23:16];
      18: ip_byte = DST_IP[15:8];
      19: ip_byte = DST_IP[7:0];
      default: ip_byte = 8'h00;
    endcase
  end

  // UDP 头逐字节（校验和=0）
  reg [7:0] udp_byte;
  always @(*) begin
    case (udp_cnt)
      0: udp_byte = SRC_PORT[15:8];
      1: udp_byte = SRC_PORT[7:0];
      2: udp_byte = DST_PORT[15:8];
      3: udp_byte = DST_PORT[7:0];
      4: udp_byte = udp_len[15:8];
      5: udp_byte = udp_len[7:0];
      6: udp_byte = 8'h00;              // checksum = 0
      7: udp_byte = 8'h00;
      default: udp_byte = 8'h00;
    endcase
  end

  // 简单负载（递增）
  wire [7:0] pay_byte = pay_cnt[7:0];

  // FCS 输出（小端）
  wire [31:0] fcs_val = ~crc;
  reg  [7:0]  fcs_byte;
  always @(*) begin
    case (fcs_cnt)
      0: fcs_byte = fcs_val[7:0];
      1: fcs_byte = fcs_val[15:8];
      2: fcs_byte = fcs_val[23:16];
      3: fcs_byte = fcs_val[31:24];
      default: fcs_byte = 8'h00;
    endcase
  end

  // 主状态机
  always @(posedge clk) begin
    if (rst) begin
      st<=S_IDLE; gmii_tx_en<=1'b0; gmii_txd<=8'h00;
      pre_cnt<=0; eth_cnt<=0; ip_cnt<=0; udp_cnt<=0;
      pay_cnt<=0; fcs_cnt<=0; ifg_cnt<=0; crc<=32'hFFFF_FFFF;
    end else begin
      case (st)
        S_IDLE: begin
          gmii_tx_en<=1'b1; gmii_txd<=8'h55;
          pre_cnt<=1; crc<=32'hFFFF_FFFF; st<=S_PREAM;
        end
        S_PREAM: begin
          gmii_txd<=8'h55; pre_cnt<=pre_cnt+1;
          if (pre_cnt==6) st<=S_SFD;
        end
        S_SFD: begin
          gmii_txd<=8'hD5;
          // SFD 不计入 CRC；紧接着开始计算 CRC
          crc<=32'hFFFF_FFFF;
          eth_cnt<=0; st<=S_ETH;
        end
        S_ETH: begin
          gmii_txd<=eth_byte;
          crc<=crc32_d8(crc, eth_byte);
          eth_cnt<=eth_cnt+1;
          if (eth_cnt==13) begin ip_cnt<=0; st<=S_IP; end
        end
        S_IP: begin
          gmii_txd<=ip_byte;
          crc<=crc32_d8(crc, ip_byte);
          ip_cnt<=ip_cnt+1;
          if (ip_cnt==19) begin udp_cnt<=0; st<=S_UDP; end
        end
        S_UDP: begin
          gmii_txd<=udp_byte;
          crc<=crc32_d8(crc, udp_byte);
          udp_cnt<=udp_cnt+1;
          if (udp_cnt==7) begin pay_cnt<=0; st<=S_PAY; end
        end
        S_PAY: begin
          gmii_txd<=pay_byte;
          crc<=crc32_d8(crc, pay_byte);
          pay_cnt<=pay_cnt+1;
          if (pay_cnt==PAYLOAD_BYTES-1) begin fcs_cnt<=0; st<=S_FCS; end
        end
        S_FCS: begin
          gmii_txd<=fcs_byte; fcs_cnt<=fcs_cnt+1;
          if (fcs_cnt==3) begin gmii_tx_en<=1'b0; ifg_cnt<=0; st<=S_IFG; end
        end
        S_IFG: begin
          ifg_cnt<=ifg_cnt+1;
          if (ifg_cnt==12-1) st<=S_IDLE; // 连续发
        end
      endcase
    end
  end
endmodule
