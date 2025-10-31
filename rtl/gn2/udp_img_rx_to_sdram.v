// ===============================================================
// udp_img_rx_to_sdram.v
//   收 UDP(每行一个包)：'P2FV' + u16(line_no,BE) + u16(len=IMG_W*2,BE) + payload(IMG_W*2)
//   校验通过则写 SDRAM：RGB565（高字节在前），line_no==0 触发 wr_load，最后一行触发 frame_done 翻转
//   特性：
//    - 以太/IP/UDP 逐层解析，默认严格过滤（MAC/IP/PORT），允许广播
//    - 可选支持 802.1Q VLAN（TPID 0x8100）
//    - UDP 长度严格等于 8+IMG_LINE_BYTES（可通过参数开关）
//    - 首像素高字节在 S_HEADCHK 抓取，避免丢第 1 字节导致行错位
//    - 行末用像素计数自检，不够/多了即打 line_err 脉冲
//    - 提供调试脉冲：dbg_ipv4_seen / dbg_udp_seen / dbg_port_hit
// ===============================================================
module udp_img_rx_to_sdram #(
  // ---- 配置参数 ----
  parameter [47:0] LOCAL_MAC      = 48'h02_11_22_33_44_55,
  parameter [31:0] LOCAL_IP       = 32'hC0A8_F001,   // 192.168.240.1
  parameter [31:0] BCAST_IP       = 32'hC0A8_F0FF,   // 192.168.240.255
  parameter [15:0] IMG_PORT       = 16'd6004,
  parameter integer IMG_W         = 640,
  parameter integer IMG_H         = 480,

  // ---- 调试/容错开关（按需改）----
  parameter        FILTER_MAC     = 1,   // 1=目的MAC 必须等于 LOCAL_MAC 或广播
  parameter        FILTER_IP_DST  = 1,   // 1=目的IP 必须 LOCAL_IP/BCAST_IP/0xFFFFFFFF
  parameter        STRICT_UDPLEN  = 1,   // 1=强制 UDP.len == 8 + IMG_W*2
  parameter        ACCEPT_VLAN    = 1    // 1=支持 802.1Q：0x8100 + TCI + EtherType
)(
  input  wire        clk,
  input  wire        rst_n,
  input  wire        en,              // 允许写（门控）；为 0 时仅解析不写
  input  wire [7:0]  gmii_rxd,
  input  wire        gmii_rx_dv,

  // 写端口（到 SDRAM 写口）
  output reg         wr_en,
  output reg [15:0]  wr_data,
  output reg         wr_load,

  // 帧完成（翻转位）
  output reg         frame_done,      // TOGGLE（480 行到齐）

  // 调试/状态脉冲（1 拍）
  output reg         hit_port,        // 命中 port（S_UDP 结束）
  output reg         bad_header,      // 行头异常（magic/pay_len/line_no）
  output reg         drop_pkt,        // 被丢弃的帧（MAC/IP/PORT/长度不符）
  output reg         line_err,        // 行像素数不符（少/多像素）

  // 分层命中调试脉冲（1 拍）
  output reg         dbg_ipv4_seen,   // 以太层→IP 层
  output reg         dbg_udp_seen,    // IP 层→UDP 层
  output reg         dbg_port_hit     // 端口命中 IMG_PORT
);

  // ==================== 常量/局部量 ====================
  localparam integer IMG_LINE_BYTES = (IMG_W*2);

  // ---------- 状态机 ----------
  localparam [3:0]
    S_IDLE    = 4'd0,
    S_PREAM   = 4'd1,
    S_ETH     = 4'd2,
    S_VLAN    = 4'd3,   // 0x8100 VLAN：TCI(2B) + EtherType(2B)
    S_IP      = 4'd4,
    S_UDP     = 4'd5,
    S_HEAD    = 4'd6,
    S_HEADCHK = 4'd7,
    S_PAY     = 4'd8,
    S_DROP    = 4'd9;

  reg [3:0]  st;
  reg [15:0] idx;

  // 以太层
  reg [47:0] dst_mac, src_mac;
  reg [15:0] eth_type;
  reg [1:0]  vlan_cnt;      // 读取 VLAN 4 字节计数（TCI 2B + EtherType 2B）
  reg [7:0]  pre_cnt;

  // IP 层
  reg [7:0]  ip_ver_ihl, ip_proto;
  reg [31:0] ip_dst;

  // UDP 头
  reg [15:0] udp_dst_port, udp_len;
  reg [15:0] pay_left;      // UDP 负载剩余（字节），包含 8B 行头 + 像素

  // 行头缓冲
  reg [7:0]  head[0:7];
  reg [3:0]  hcnt;

  // 行 & 像素
  reg [15:0] line_no, pay_len;
  reg [10:0] pix_cnt;       // 0..(IMG_W-1)
  reg        byte_phase;    // 0:等高字节，1:等低字节
  reg [15:0] pix_word;

  // dv 缓存（沿检测）
  reg dv_d1;
  wire dv_rise =  gmii_rx_dv & ~dv_d1;
  wire dv_fall = ~gmii_rx_dv &  dv_d1;

  integer i;

  // ==================== 任务：解析行头 ====================
  task prepare_line;
    reg [31:0] magic;
  begin
    magic   = {head[0],head[1],head[2],head[3]}; // 'P2FV'
    line_no = {head[4],head[5]};
    pay_len = {head[6],head[7]};

    // 默认：新行开始前的复位
    byte_phase <= 1'b0;
    pix_cnt    <= 11'd0;

    // 头校验
    if (magic==32'h50324656 &&                             // 'P2FV'
        pay_len==IMG_LINE_BYTES &&                         // 长度=1280
        line_no < IMG_H) begin                             // 0..479
      // 行0触发切换写缓冲
      if (en && line_no==16'd0) wr_load <= 1'b1;
    end else begin
      bad_header <= 1'b1;  // ★ 发一个脉冲
    end
  end
  endtask

  // ==================== 主时序 ====================
  always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
      st<=S_IDLE; idx<=16'd0;
      dv_d1<=1'b0; pre_cnt<=8'd0; vlan_cnt<=2'd0;

      dst_mac<=48'd0; src_mac<=48'd0; eth_type<=16'd0;
      ip_ver_ihl<=8'd0; ip_proto<=8'd0; ip_dst<=32'd0;

      udp_dst_port<=16'd0; udp_len<=16'd0; pay_left<=16'd0;
      hcnt<=4'd0; line_no<=16'd0; pay_len<=16'd0;

      wr_en<=1'b0; wr_data<=16'h0000; wr_load<=1'b0;
      frame_done<=1'b0;

      hit_port<=1'b0; bad_header<=1'b0; drop_pkt<=1'b0; line_err<=1'b0;
      dbg_ipv4_seen<=1'b0; dbg_udp_seen<=1'b0; dbg_port_hit<=1'b0;

      pix_cnt<=11'd0; byte_phase<=1'b0; pix_word<=16'd0;
    end else begin
      // 默认值（单拍脉冲/写使能/切换/错误在本拍内赋值）
      dv_d1        <= gmii_rx_dv;
      wr_en        <= 1'b0;
      wr_load      <= 1'b0;

      hit_port     <= 1'b0;
      bad_header   <= 1'b0;
      drop_pkt     <= 1'b0;
      line_err     <= 1'b0;

      dbg_ipv4_seen<= 1'b0;
      dbg_udp_seen <= 1'b0;
      dbg_port_hit <= 1'b0;

      case(st)
        // ----------------- 等待帧开始 -----------------
        S_IDLE: begin
          if (dv_rise) begin
            if (gmii_rxd==8'h55) begin
              pre_cnt <= 8'd1; st<=S_PREAM;
            end else begin
              // 没有前导也尝试按以太头起步（容错）
              idx<=16'd1; dst_mac[47:40]<=gmii_rxd; st<=S_ETH;
            end
          end
        end

        // ----------------- 前导码 55...55 D5 -----------------
        S_PREAM: begin
          if (gmii_rx_dv) begin
            if (pre_cnt<8'd7) begin
              if (gmii_rxd==8'h55) pre_cnt<=pre_cnt+8'd1;
              else pre_cnt<=8'd0; // 出错就重新计
            end else begin
              if (gmii_rxd==8'hD5) begin idx<=16'd0; st<=S_ETH; end
            end
          end else st<=S_IDLE;
        end

        // ----------------- 以太头（含目的/源 MAC + EtherType） -----------------
        S_ETH: begin
          if (gmii_rx_dv) begin
            case(idx)
              16'd0:  dst_mac[47:40] <= gmii_rxd;
              16'd1:  dst_mac[39:32] <= gmii_rxd;
              16'd2:  dst_mac[31:24] <= gmii_rxd;
              16'd3:  dst_mac[23:16] <= gmii_rxd;
              16'd4:  dst_mac[15:8 ] <= gmii_rxd;
              16'd5:  dst_mac[7:0  ] <= gmii_rxd;

              16'd6:  src_mac[47:40] <= gmii_rxd;
              16'd7:  src_mac[39:32] <= gmii_rxd;
              16'd8:  src_mac[31:24] <= gmii_rxd;
              16'd9:  src_mac[23:16] <= gmii_rxd;
              16'd10: src_mac[15:8 ] <= gmii_rxd;
              16'd11: src_mac[7:0  ] <= gmii_rxd;

              16'd12: eth_type[15:8] <= gmii_rxd;
              16'd13: eth_type[7:0 ] <= gmii_rxd;
            endcase
            idx <= idx + 16'd1;

            if (idx==16'd13) begin
              // VLAN 0x8100 处理
              if (ACCEPT_VLAN && eth_type==16'h8100) begin
                st <= S_VLAN; vlan_cnt <= 2'd0; // 接下来再读 4B：TCI(2)+EtherType(2)
              end else begin
                // 常规：以太类型 = IPv4 ?
                if (eth_type==16'h0800) begin
                  if (!FILTER_MAC || (dst_mac==LOCAL_MAC || dst_mac==48'hFF_FF_FF_FF_FF_FF)) begin
                    idx<=16'd0; st<=S_IP; dbg_ipv4_seen<=1'b1; // ★
                  end else begin
                    st<=S_DROP; drop_pkt<=1'b1;
                  end
                end else begin
                  st<=S_DROP; drop_pkt<=1'b1;
                end
              end
            end
          end else st<=S_IDLE;
        end

        // ----------------- VLAN：读取 TCI(2B) + EtherType(2B) -----------------
        S_VLAN: begin
          if (gmii_rx_dv) begin
            vlan_cnt <= vlan_cnt + 2'd1;
            if (vlan_cnt==2'd2) eth_type[15:8] <= gmii_rxd; // 先高字节
            if (vlan_cnt==2'd3) begin
              eth_type[7:0] <= gmii_rxd;
              // VLAN 后 EtherType 仍需是 IPv4
              if (eth_type==16'h0800) begin
                if (!FILTER_MAC || (dst_mac==LOCAL_MAC || dst_mac==48'hFF_FF_FF_FF_FF_FF)) begin
                  idx<=16'd0; st<=S_IP; dbg_ipv4_seen<=1'b1; // ★
                end else begin
                  st<=S_DROP; drop_pkt<=1'b1;
                end
              end else begin
                st<=S_DROP; drop_pkt<=1'b1;
              end
            end
          end else st<=S_IDLE;
        end

        // ----------------- IP 头（IHL=5） -----------------
        S_IP: begin
          if (gmii_rx_dv) begin
            case(idx)
              16'd0:  ip_ver_ihl <= gmii_rxd;      // 一般为 0x45
              16'd9:  ip_proto   <= gmii_rxd;      // 17=UDP
              16'd16: ip_dst[31:24] <= gmii_rxd;
              16'd17: ip_dst[23:16] <= gmii_rxd;
              16'd18: ip_dst[15:8 ] <= gmii_rxd;
              16'd19: begin
                ip_dst[7:0] <= gmii_rxd;
                // 末字节当拍比较，避免晚一拍
                if (ip_ver_ihl[7:4]==4'h4 && ip_ver_ihl[3:0]==4'h5 && // IPv4, IHL=5
                    ip_proto==8'd17) begin                           // UDP
                  if (!FILTER_IP_DST ||
                      ({ip_dst[31:8], gmii_rxd}==LOCAL_IP  ||
                       {ip_dst[31:8], gmii_rxd}==BCAST_IP  ||
                       {ip_dst[31:8], gmii_rxd}==32'hFFFF_FFFF)) begin
                    idx<=16'd0; st<=S_UDP; dbg_udp_seen<=1'b1; // ★
                  end else begin
                    st<=S_DROP; drop_pkt<=1'b1;
                  end
                end else begin
                  st<=S_DROP; drop_pkt<=1'b1;
                end
              end
              default: ;
            endcase
            if (idx!=16'd19) idx<=idx+16'd1;
          end else st<=S_IDLE;
        end

        // ----------------- UDP 头（8 字节） -----------------
        S_UDP: begin
          if (gmii_rx_dv) begin
            case(idx)
              16'd2: udp_dst_port[15:8] <= gmii_rxd;
              16'd3: udp_dst_port[7:0 ] <= gmii_rxd;
              16'd4: udp_len[15:8]      <= gmii_rxd;
              16'd5: udp_len[7:0 ]      <= gmii_rxd;
            endcase
            idx<=idx+16'd1;

            if (idx==16'd7) begin
              // 负载剩余：扣掉 8 字节 UDP 头
              pay_left <= (udp_len>=16'd8) ? (udp_len - 16'd8) : 16'd0;
              hcnt     <= 4'd0;

              // 端口判定 + 长度严格判等（可开关）
              if ( (!STRICT_UDPLEN || udp_len==(16'd8 + IMG_LINE_BYTES)) &&
                   (udp_dst_port==IMG_PORT) ) begin
                hit_port     <= 1'b1;    // ★ 脉冲
                dbg_port_hit <= 1'b1;    // ★ 脉冲
                st           <= S_HEAD;
              end else begin
                st <= S_DROP; drop_pkt <= 1'b1;
              end
            end
          end else st<=S_IDLE;
        end

        // ----------------- 行头 8B -----------------
        S_HEAD: begin
          if (gmii_rx_dv) begin
            if (pay_left!=16'd0) begin
              pay_left   <= pay_left - 16'd1;  // 这 8B 也属于负载
              head[hcnt] <= gmii_rxd;
              if (hcnt==4'd7) begin
                hcnt <= 4'd0; st<=S_HEADCHK;   // 下一拍解析
              end else begin
                hcnt <= hcnt + 4'd1;
              end
            end else begin
              st<=S_DROP; // 长度不够
            end
          end else st<=S_IDLE;
        end

        // ----------------- 头检查 + 抓第一字节为高字节 -----------------
        S_HEADCHK: begin
          // 解析头，可能产生 wr_load/bad_header 脉冲
          prepare_line();

          // ★ 抓住第一个 payload 字节（高字节）
          if (gmii_rx_dv && en && !bad_header && (pay_left!=16'd0)) begin
            pix_word[15:8] <= gmii_rxd;     // 高字节
            byte_phase     <= 1'b1;         // 下一拍等低字节 -> 立即写
            pay_left       <= pay_left - 16'd1;
          end
          st <= S_PAY;
        end

        // ----------------- 像素负载（RGB565，高字节先） -----------------
        S_PAY: begin
          if (gmii_rx_dv) begin
            if (pay_left!=16'd0) begin
              pay_left <= pay_left - 16'd1;

              if (en && !bad_header) begin
                if (!byte_phase) begin
                  pix_word[15:8] <= gmii_rxd;  // 高字节
                  byte_phase     <= 1'b1;
                end else begin
                  pix_word[7:0]  <= gmii_rxd;  // 低字节
                  byte_phase     <= 1'b0;

                  wr_data        <= {pix_word[15:8], gmii_rxd};
                  wr_en          <= 1'b1;       // 写一个像素
                  pix_cnt        <= pix_cnt + 11'd1;

                  // 包最后一个字节（当前拍看到 pay_left==1）
                  if (pay_left==16'd1) begin
                    // 本拍我们正好写完最后一个像素 ⇒ 旧值应为 W-1
                    if (pix_cnt != (IMG_W-1)) line_err <= 1'b1;  // ★ 不等于 W-1 触发
                    if (line_no == IMG_H-1) frame_done <= ~frame_done; // ★ 最后一行翻转
                  end
                end
              end

            end else begin
              st <= S_DROP; // 异常：负载提前结束
            end

          end else begin
            st<=S_IDLE;
          end
        end

        // ----------------- 丢弃整帧 -----------------
        S_DROP: begin
          if (dv_fall) st<=S_IDLE;
        end

        default: st<=S_IDLE;
      endcase
    end
  end

endmodule
