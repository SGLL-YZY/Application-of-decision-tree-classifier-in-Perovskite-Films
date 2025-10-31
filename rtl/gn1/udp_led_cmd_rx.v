// udp_led_cmd_rx.v  (LED + SEG + BOTH + EN门控)
// 兼容原 LED!，新增 SEG!、BOTH；新增使能输入 en：en=1 时才执行写入
module udp_led_cmd_rx #(
  parameter [47:0] LOCAL_MAC   = 48'h02_11_22_33_44_55,
  parameter [31:0] LOCAL_IP    = 32'hC0A8_F001,   // 192.168.240.1
  parameter [31:0] BCAST_IP    = 32'hC0A8_F0FF,   // 192.168.240.255
  parameter [15:0] LISTEN_PORT = 16'd6003
)(
  input  wire        clk,       // GMII RX 时钟域（rx_clk_g）
  input  wire        rst_n,
  input  wire        en,        // ★ 新增：模式使能（1=允许更新输出；0=仅解析不写）
  input  wire [7:0]  gmii_rxd,
  input  wire        gmii_rx_dv,

  output reg  [3:0]  led_o,     // 低4位LED
  output reg [15:0]  dled_o,    // 数码管并行输出 {seg_en[15:8], seg_pat[7:0]}

  // 调试探针
  output reg         hit_port,
  output reg         hit_cmd,
  output reg         err_magic
);

  localparam S_IDLE=0, S_PREAM=1, S_ETH=2, S_IP=3, S_UDP=4, S_PAY=5, S_DROP=6;
  reg [2:0]  st;
  reg [15:0] idx;

  // 以太/IPv4/UDP 字段
  reg [47:0] dst_mac, src_mac;
  reg [15:0] eth_type;

  reg [7:0]  ip_ver_ihl, ip_proto;
  reg [31:0] ip_dst;

  reg [15:0] udp_dst_port, udp_len;
  reg [15:0] pay_left;

  // 负载前 8 字节缓冲（为支持 BOTH）
  reg [7:0] pay_buf[0:7];
  reg [3:0] pay_cnt;   // 0..8

  // 前导码相关
  reg [2:0] pre_cnt;

  // RX_DV 边沿
  reg dv_d1;
  wire dv_rise = gmii_rx_dv & ~dv_d1;
  wire dv_fall = ~gmii_rx_dv & dv_d1;

  integer i;

  // === 解析并（在 en=1 时）执行命令 ===
  task apply_command;
    reg [31:0] magic;
    reg [3:0]  led_new;
    reg [15:0] dled_new;
    begin
      magic = {pay_buf[0], pay_buf[1], pay_buf[2], pay_buf[3]}; // 大端拼

      if (magic == 32'h4C454421) begin
        // "LED!" + mask + value
        if (pay_cnt >= 6) begin
          hit_cmd <= 1'b1; // 收到合法命令
          led_new = (led_o & ~pay_buf[4][3:0]) | (pay_buf[5][3:0] & pay_buf[4][3:0]);
          if (en) led_o <= led_new; // ★ en=1 才写
        end else begin
          err_magic <= 1'b1; // 长度不够
        end
      end

      else if (magic == 32'h53454721) begin
        // "SEG!" + seg_en + seg_pat
        if (pay_cnt >= 6) begin
          hit_cmd <= 1'b1;
          dled_new = {pay_buf[4], pay_buf[5]}; // {seg_en, seg_pat}
          if (en) dled_o <= dled_new;          // ★ en=1 才写
        end else begin
          err_magic <= 1'b1;
        end
      end

      else if (magic == 32'h424F5448) begin
        // "BOTH" + mask + value + seg_en + seg_pat
        if (pay_cnt >= 8) begin
          hit_cmd  <= 1'b1;
          led_new  = (led_o & ~pay_buf[4][3:0]) | (pay_buf[5][3:0] & pay_buf[4][3:0]);
          dled_new = {pay_buf[6], pay_buf[7]};
          if (en) begin
            led_o  <= led_new;   // ★ en=1 才写
            dled_o <= dled_new;  // ★ en=1 才写
          end
        end else begin
          err_magic <= 1'b1;
        end
      end

      else begin
        err_magic <= 1'b1; // 未知 magic
      end
    end
  endtask

  always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
      st<=S_IDLE; idx<=0; dv_d1<=0; pre_cnt<=0;
      dst_mac<=0; src_mac<=0; eth_type<=0;
      ip_ver_ihl<=0; ip_proto<=0; ip_dst<=0;
      udp_dst_port<=0; udp_len<=0; pay_left<=0; pay_cnt<=0;
      for(i=0;i<8;i=i+1) pay_buf[i]<=8'h00;
      led_o<=4'b0000; 
      dled_o<=16'hFFFF;  // 多数数码管低有效：默认全灭；如为高有效请改默认值
      hit_port<=1'b0; hit_cmd<=1'b0; err_magic<=1'b0;
    end else begin
      dv_d1    <= gmii_rx_dv;
      hit_port <= 1'b0;
      hit_cmd  <= 1'b0;

      case(st)
        // 等待帧起
        S_IDLE: begin
          if (dv_rise) begin
            if (gmii_rxd == 8'h55) begin
              pre_cnt <= 3'd1;
              st      <= S_PREAM;
            end else begin
              idx <= 16'd1;
              dst_mac[47:40] <= gmii_rxd;
              st  <= S_ETH;
            end
          end
        end

        // 跳过 7×0x55 + 0xD5
        S_PREAM: begin
          if (gmii_rx_dv) begin
            if (pre_cnt < 3'd7) begin
              if (gmii_rxd == 8'h55) pre_cnt <= pre_cnt + 3'd1;
              else pre_cnt <= 3'd0;
            end else begin
              if (gmii_rxd == 8'hD5) begin
                idx <= 16'd0;
                st  <= S_ETH;
              end
            end
          end else st <= S_IDLE;
        end

        // 以太头 14B
        S_ETH: begin
          if (gmii_rx_dv) begin
            case(idx)
              0:dst_mac[47:40]<=gmii_rxd;  1:dst_mac[39:32]<=gmii_rxd;
              2:dst_mac[31:24]<=gmii_rxd;  3:dst_mac[23:16]<=gmii_rxd;
              4:dst_mac[15:8 ]<=gmii_rxd;  5:dst_mac[7:0 ]<=gmii_rxd;
              6:src_mac[47:40]<=gmii_rxd;  7:src_mac[39:32]<=gmii_rxd;
              8:src_mac[31:24]<=gmii_rxd;  9:src_mac[23:16]<=gmii_rxd;
              10:src_mac[15:8]<=gmii_rxd;  11:src_mac[7:0]<=gmii_rxd;
              12:eth_type[15:8]<=gmii_rxd; 13:eth_type[7:0]<=gmii_rxd;
            endcase
            idx <= idx + 16'd1;

            if (idx==13) begin
              if (eth_type==16'h0800 &&
                 (dst_mac==LOCAL_MAC || dst_mac==48'hFF_FF_FF_FF_FF_FF)) begin
                idx<=16'd0; st<=S_IP;
              end else st<=S_DROP;
            end
          end else st<=S_IDLE;
        end

        // IP 头（IHL=5）
        S_IP: begin
          if (gmii_rx_dv) begin
            case(idx)
              0: ip_ver_ihl<=gmii_rxd;    // 0x45
              9: ip_proto  <=gmii_rxd;    // 17
              16:ip_dst[31:24]<=gmii_rxd;
              17:ip_dst[23:16]<=gmii_rxd;
              18:ip_dst[15:8 ]<=gmii_rxd;
              19:ip_dst[7:0  ]<=gmii_rxd;
            endcase
            idx <= idx + 16'd1;

            if (idx==19) begin
              if (ip_ver_ihl==8'h45 && ip_proto==8'd17 &&
                 (ip_dst==LOCAL_IP || ip_dst==BCAST_IP || ip_dst==32'hFFFF_FFFF)) begin
                idx<=16'd0; st<=S_UDP;
              end else st<=S_DROP;
            end
          end else st<=S_IDLE;
        end

        // UDP 头
        S_UDP: begin
          if (gmii_rx_dv) begin
            case(idx)
              2: udp_dst_port[15:8]<=gmii_rxd;
              3: udp_dst_port[7:0 ]<=gmii_rxd;
              4: udp_len[15:8]     <=gmii_rxd;
              5: udp_len[7:0 ]     <=gmii_rxd;
            endcase
            idx <= idx + 16'd1;

            if (idx==7) begin
              pay_left <= (udp_len>=16'd8)?(udp_len-16'd8):16'd0;
              pay_cnt  <= 4'd0;
              if (udp_dst_port==LISTEN_PORT) begin
                hit_port <= 1'b1;
                st <= S_PAY;
              end else st<=S_DROP;
            end
          end else st<=S_IDLE;
        end

        // 负载（前 8B 最多缓存，尾部执行）
        S_PAY: begin
          if (gmii_rx_dv) begin
            if (pay_left!=16'd0) begin
              pay_left <= pay_left - 16'd1;
              if (pay_cnt<4'd8) begin
                pay_buf[pay_cnt] <= gmii_rxd;
                pay_cnt <= pay_cnt + 4'd1;
              end
            end else begin
              apply_command();   // ★ 在这里完成解析与（条件）写入
              st <= S_DROP;
            end
          end else st<=S_IDLE;
        end

        // 丢弃到帧尾
        S_DROP: begin
          if (dv_fall) st <= S_IDLE;
        end
      endcase
    end
  end
endmodule
