`timescale 1ns/1ps
module udp_line_tx_from_fifo #(
  // ====== MAC/IP/UDP ======
  parameter [47:0] SRC_MAC   = 48'h02_11_22_33_44_55,
  parameter [31:0] SRC_IP    = 32'hC0A8_F001,      // 192.168.240.1
  parameter [15:0] SRC_PORT  = 16'd6001,
  parameter [47:0] DST_MAC   = 48'hFF_FF_FF_FF_FF_FF,
  parameter [31:0] DST_IP    = 32'hC0A8_F002,      // 192.168.240.2
  parameter [15:0] DST_PORT  = 16'd6002,

  // ====== 图像 & 分包 ======
  parameter integer IMG_W        = 640,
  parameter integer IMG_H        = 480,
  parameter integer BYTES_PER_PX = 2,
  parameter integer LINE_BYTES   = IMG_W*BYTES_PER_PX, // 1280
  parameter integer HDR_BYTES    = 16                   // 业务头 16B
)(
  input  wire        clk,
  input  wire        rst,

  // 源 FIFO（8bit，FWFT/Show-Ahead）
  input  wire [7:0]  fifo_dout,
  input  wire        fifo_empty,
  input  wire [15:0] fifo_rdusedw,   // 单位：Byte
  output reg         fifo_re,

  // GMII TX
  output reg  [7:0]  gmii_txd,
  output reg         gmii_tx_en,
  output wire        gmii_tx_er,

  // ====== 调试探针（探针1）======
  output reg         dbg_bad_len,      // 粘滞：发现负载 != 1280 -> 置1
  output reg [15:0]  dbg_pay_bytes,    // 本包实际发出的负载字节计数
  output reg         dbg_pkt_toggle    // 每包结束翻转一次
);
  assign gmii_tx_er = 1'b0;

  // ====== 常量（方案A：一行一包）======
  // UDP payload = 16(头) + 1280(像素) = 1296
  localparam [15:0] UDP_LEN_CONST = 16'd8  + HDR_BYTES + LINE_BYTES; // 1304
  localparam [15:0] IP_LEN_CONST  = 16'd20 + UDP_LEN_CONST;          // 1324

  // 以太/IP/UDP 常量
  localparam [15:0] ETH_TYPE_IPV4 = 16'h0800;
  localparam [7:0]  IP_VER_IHL    = 8'h45;
  localparam [7:0]  IP_TOS        = 8'h00;
  localparam [15:0] IP_ID         = 16'h0000;
  localparam [15:0] IP_FRAG       = 16'h0000;
  localparam [7:0]  IP_TTL        = 8'd64;
  localparam [7:0]  IP_PROTO_UDP  = 8'd17;

  // CRC32（Ethernet FCS，反射式 0xEDB88320）
  function [31:0] crc32_d8(input [31:0] c, input [7:0] d);
    integer i; reg [31:0] cur; reg [7:0] b;
  begin
    cur=c; b=d;
    for(i=0;i<8;i=i+1) begin
      if ((cur[0]^b[0])) cur=(cur>>1)^32'hedb88320; else cur=(cur>>1);
      b=b>>1;
    end
    crc32_d8=cur;
  end endfunction

  // IPv4 头校验和（10个16b词）
  function [15:0] ip_checksum(
    input [15:0] w0,input [15:0] w1,input [15:0] w2,input [15:0] w3,input [15:0] w4,
    input [15:0] w5,input [15:0] w6,input [15:0] w7,input [15:0] w8,input [15:0] w9
  ); reg [31:0] s;
  begin
    s=w0+w1+w2+w3+w4+w5+w6+w7+w8+w9;
    s=(s[15:0]+s[31:16]); s=(s[15:0]+(s>>16));
    ip_checksum=~s[15:0];
  end endfunction

  // ====== 行/帧计数 ======
  reg [15:0] frame_id=0;
  reg [15:0] line_idx=0;

  // ====== 本包头字段（定值寄存）======
  reg [15:0] ip_tot_len_r, ip_chk_r;

  // ====== 发送状态机 ======
  localparam S_IDLE=0,S_PREAM=1,S_SFD=2,S_ETH=3,S_IP=4,S_UDP=5,S_HDR=6,S_PAY=7,S_FCS=8,S_IFG=9,S_END=10; 
  reg [3:0] st=0;

  reg [5:0] pre_cnt, eth_cnt, ip_cnt;
  reg [3:0] udp_cnt;
  reg [4:0] hdr_cnt;
  reg [15:0] payload_left;
  reg [2:0] fcs_cnt;
  reg [5:0] ifg_cnt;
  reg [31:0] crc;

  // —— FWFT：滑移缓冲
  reg  [7:0] pay_byte;

  // ====== 起包条件（首包必须等 FIFO ≥ 1280 字节）======
  wire can_start = (fifo_rdusedw >= LINE_BYTES) && !fifo_empty;

  // 以太头字节
  reg [7:0] eth_byte;
  always @(*) begin
    case (eth_cnt)
      0: eth_byte=DST_MAC[47:40];  1: eth_byte=DST_MAC[39:32];
      2: eth_byte=DST_MAC[31:24];  3: eth_byte=DST_MAC[23:16];
      4: eth_byte=DST_MAC[15:8 ];  5: eth_byte=DST_MAC[7:0];
      6: eth_byte=SRC_MAC[47:40];  7: eth_byte=SRC_MAC[39:32];
      8: eth_byte=SRC_MAC[31:24];  9: eth_byte=SRC_MAC[23:16];
      10:eth_byte=SRC_MAC[15:8 ]; 11: eth_byte=SRC_MAC[7:0];
      12:eth_byte=8'h08;          13: eth_byte=8'h00;
      default: eth_byte=8'h00;
    endcase
  end

  // IP 头字节
  reg [7:0] ip_byte;
  always @(*) begin
    case (ip_cnt)
      0: ip_byte=IP_VER_IHL;     1: ip_byte=IP_TOS;
      2: ip_byte=ip_tot_len_r[15:8]; 3: ip_byte=ip_tot_len_r[7:0];
      4: ip_byte=IP_ID[15:8];    5: ip_byte=IP_ID[7:0];
      6: ip_byte=IP_FRAG[15:8];  7: ip_byte=IP_FRAG[7:0];
      8: ip_byte=IP_TTL;         9: ip_byte=IP_PROTO_UDP;
      10:ip_byte=ip_chk_r[15:8]; 11:ip_byte=ip_chk_r[7:0];
      12:ip_byte=SRC_IP[31:24];  13:ip_byte=SRC_IP[23:16];
      14:ip_byte=SRC_IP[15:8];   15:ip_byte=SRC_IP[7:0];
      16:ip_byte=DST_IP[31:24];  17:ip_byte=DST_IP[23:16];
      18:ip_byte=DST_IP[15:8];   19:ip_byte=DST_IP[7:0];
      default: ip_byte=8'h00;
    endcase
  end

  // UDP 头字节（长度固定常量）
  reg [7:0] udp_byte;
  always @(*) begin
    case (udp_cnt)
      0: udp_byte=SRC_PORT[15:8];     1: udp_byte=SRC_PORT[7:0];
      2: udp_byte=DST_PORT[15:8];     3: udp_byte=DST_PORT[7:0];
      4: udp_byte=UDP_LEN_CONST[15:8];5: udp_byte=UDP_LEN_CONST[7:0];
      6: udp_byte=8'h00;              7: udp_byte=8'h00;  // checksum=0
      default: udp_byte=8'h00;
    endcase
  end

  // 16B 业务头（小端）
  reg [7:0] hdr_byte;
  wire [7:0] flags = {6'b0,(line_idx==0),(1'b1)}; // bit0=SOL, bit1=SOF；一行一包
  always @(*) begin
    case (hdr_cnt)
      0:  hdr_byte = 8'h5A;
      1:  hdr_byte = 8'hA5;
      2:  hdr_byte = 8'h01;
      3:  hdr_byte = flags;
      4:  hdr_byte = frame_id[7:0];
      5:  hdr_byte = frame_id[15:8];
      6:  hdr_byte = line_idx[7:0];
      7:  hdr_byte = line_idx[15:8];
      8:  hdr_byte = 8'h00;                 // pkt_idx L
      9:  hdr_byte = 8'h00;                 // pkt_idx H
      10: hdr_byte = LINE_BYTES[7:0];       // data_len L
      11: hdr_byte = LINE_BYTES[15:8];      // data_len H
      12: hdr_byte = 8'h00;
      13: hdr_byte = 8'h00;
      14: hdr_byte = 8'h00;
      15: hdr_byte = 8'h00;
      default: hdr_byte = 8'h00;
    endcase
  end

  // FCS
  wire [31:0] fcs_val = ~crc;
  reg  [7:0]  fcs_byte;
  always @(*) begin
    case (fcs_cnt)
      0:fcs_byte=fcs_val[7:0];
      1:fcs_byte=fcs_val[15:8];
      2:fcs_byte=fcs_val[23:16];
      3:fcs_byte=fcs_val[31:24];
      default:fcs_byte=8'h00;
    endcase
  end

  // ====== 主时序 ======
  always @(posedge clk) begin
    if (rst) begin
      st<=S_IDLE; gmii_tx_en<=1'b0; gmii_txd<=8'h00; fifo_re<=1'b0;
      pre_cnt<=0; eth_cnt<=0; ip_cnt<=0; udp_cnt<=0; hdr_cnt<=0;
      payload_left<=0; fcs_cnt<=0; ifg_cnt<=0; crc<=32'hFFFF_FFFF;
      frame_id<=0; line_idx<=0; pay_byte<=8'h00;
      ip_tot_len_r<=0; ip_chk_r<=0;
      dbg_bad_len<=1'b0; dbg_pay_bytes<=16'd0; dbg_pkt_toggle<=1'b0;
    end else begin
      fifo_re <= 1'b0;

      case (st)
        // ---------- IDLE：等一整行都在 FIFO 中 ----------
        S_IDLE: begin
          if (can_start) begin
            ip_tot_len_r <= IP_LEN_CONST;
            ip_chk_r     <= ip_checksum(
                              {IP_VER_IHL,IP_TOS},
                              IP_LEN_CONST,
                              IP_ID,
                              IP_FRAG,
                              {IP_TTL,IP_PROTO_UDP},
                              16'h0000,
                              SRC_IP[31:16], SRC_IP[15:0],
                              DST_IP[31:16], DST_IP[15:0]
                            );
            if (line_idx==0) frame_id <= frame_id + 16'd1;

            gmii_tx_en <= 1'b1;
            gmii_txd   <= 8'h55;     // preamble 第1字节
            pre_cnt    <= 6'd1;
            crc        <= 32'hFFFF_FFFF;
            st         <= S_PREAM;
          end
        end

        // ---------- 7×0x55 ----------
        S_PREAM: begin
          gmii_txd<=8'h55; pre_cnt<=pre_cnt+1;
          if (pre_cnt==6) st<=S_SFD;
        end

        // ---------- 0xD5 ----------
        S_SFD: begin
          gmii_txd<=8'hD5;
          crc<=32'hFFFF_FFFF;  // CRC 从 SFD 后开始计算
          eth_cnt<=0; st<=S_ETH;
        end

        // ---------- Ethernet Header ----------
        S_ETH: begin
          gmii_txd<=eth_byte; crc<=crc32_d8(crc,eth_byte); eth_cnt<=eth_cnt+1;
          if (eth_cnt==13) begin ip_cnt<=0; st<=S_IP; end
        end

        // ---------- IP Header ----------
        S_IP: begin
          gmii_txd<=ip_byte; crc<=crc32_d8(crc,ip_byte); ip_cnt<=ip_cnt+1;
          if (ip_cnt==19) begin udp_cnt<=0; st<=S_UDP; end
        end

        // ---------- UDP Header ----------
        S_UDP: begin
          gmii_txd<=udp_byte; crc<=crc32_d8(crc,udp_byte); udp_cnt<=udp_cnt+1;
          if (udp_cnt==7) begin
            hdr_cnt<=0;
            st<=S_HDR;          // ★ FWFT: 这里不要预取
          end
        end

        // ---------- 16B 业务头（FWFT 对齐：先锁首字节，最后一拍再弹出） ----------
        S_HDR: begin
          gmii_txd<=hdr_byte; crc<=crc32_d8(crc,hdr_byte); hdr_cnt<=hdr_cnt+1;

          if (hdr_cnt==HDR_BYTES-2) begin
            pay_byte <= fifo_dout;         // 先“看见并锁住”第1个像素
          end

          if (hdr_cnt==HDR_BYTES-1) begin
            payload_left  <= LINE_BYTES;
            dbg_pay_bytes <= 16'd0;        // 计数清零
            fifo_re       <= 1'b1;         // ★ 最后一拍才弹出 → 下一拍 dout=第2个像素
            st            <= S_PAY;
          end
        end

        // ---------- PAY：每拍发一个、读一个（FWFT） ----------
        S_PAY: begin
          gmii_txd <= pay_byte;              // 发第 k 个字节
          crc      <= crc32_d8(crc, pay_byte);
          dbg_pay_bytes <= dbg_pay_bytes + 16'd1;

          if (payload_left > 16'd1) begin
            payload_left <= payload_left - 16'd1;
            pay_byte     <= fifo_dout;       // 顶上第 k+1 个字节
            fifo_re      <= 1'b1;            // 预取第 k+2 个字节
          end else begin
            // 最后 1 个字节（本拍已发）
            payload_left <= 16'd0;
            fcs_cnt      <= 3'd0;
            st           <= S_FCS;
          end
        end

        // ---------- FCS ----------
        S_FCS: begin
        gmii_txd <= fcs_byte;
        fcs_cnt  <= fcs_cnt + 1;
        if (fcs_cnt==3) begin
            // ★ 保持 gmii_tx_en=1，下一拍到 S_END 再拉低
            // 统计/翻转保持不变（如果你有 dbg_xx 就放在这里）
            dbg_pkt_toggle <= ~dbg_pkt_toggle;
            if (dbg_pay_bytes != LINE_BYTES) dbg_bad_len <= 1'b1;
            st <= S_END;
        end
        end
        // ★ 新增尾态：下一拍再把 tx_en 拉低，然后进入 IFG
        S_END: begin
        gmii_tx_en <= 1'b0;   // 现在才退出帧
        ifg_cnt    <= 0;
        st         <= S_IFG;
        end
        // ---------- IFG ≥ 12B ----------
        S_IFG: begin
          ifg_cnt <= ifg_cnt + 6'd1;
          if (ifg_cnt==6'd11) begin
            line_idx <= (line_idx==IMG_H-1) ? 16'd0 : (line_idx+16'd1);
            st<=S_IDLE;
          end
        end

      endcase
    end
  end
endmodule
