module Ethernet(    
    input                sys_clk             ,
    input                sys_rst_n           ,
    //摄像头接口      
    input                cam_pclk            ,
    input                cam_vsync           ,
    input                cam_href            ,
    input       [7:0]    cam_data            ,
    output               cam_rst_n           ,
    output               cam_pwdn            ,
    output               cam_scl             ,
    inout                cam_sda             ,
    //按键控制       
    input                SW1                 ,
    input                SW2                 ,
    input                SW3                 ,
    input                SW4                 , 
    //功能1        
    output               LED0                ,
    output               LED1                ,
    output               LED2                ,
    output               LED3                ,
    output      [15:0]   dled                ,
    //功能2                                
    output               vga_hs              ,
    output               vga_vs              ,
    output      [11:0]   vga_rgb             ,
    //功能4
    input                phy1_rgmii_rx_clk   ,
    input                phy1_rgmii_rx_ctl   ,
    input [3:0]          phy1_rgmii_rx_data  ,
    output wire          phy1_rgmii_tx_clk   ,
    output wire          phy1_rgmii_tx_ctl   ,
    output wire [3:0]    phy1_rgmii_tx_data
    );

//功能1参数
parameter  BCAST_IP           = 32'hC0A8_F0FF;          // 192.168.240.255
parameter  LISTEN_PORT        = 16'd6003;

//功能2参数
parameter  IMG_PORT           = 16'd6004;        // PC-FPGA 图片端口

//功能4参数
parameter  LOCAL_UDP_PORT_NUM = 16'd6001;               //PS侧
parameter  LOCAL_IP_ADDRESS   = 32'hc0a8f001;           //PS侧 : 192.168.240.1
parameter  LOCAL_MAC_ADDRESS  = 48'h02_11_22_33_44_55;  //PS侧
parameter  DST_UDP_PORT_NUM   = 16'd6002;               //PC侧 
parameter  DST_IP_ADDRESS     = 32'hc0a8f002;           //PC侧 : 192.168.240.2
parameter  DST_MAC_ADDRESS    = 48'hFF_FF_FF_FF_FF_FF;  //PC侧

//公共参数
parameter  SLAVE_ADDR = 7'h3c         ;  //OV5640的器件地址7'h3c
parameter  BIT_CTRL   = 1'b1          ;  //OV5640的字节地址为16位  0:8位 1:16位
parameter  CLK_FREQ   = 26'd25_000_000;  //i2c_dri模块的驱动时钟频率 65MHz
parameter  I2C_FREQ   = 18'd250_000   ;  //I2C的SCL时钟频率,不超过400KHz
parameter  CMOS_H_PIXEL = 24'd640     ;  //CMOS水平方向像素个数,用于设置SDRAM缓存大小
parameter  CMOS_V_PIXEL = 24'd480     ;  //CMOS垂直方向像素个数,用于设置SDRAM缓存大小

//功能选择
wire gn1 = (~SW1) & ( SW2) & ( SW3);
wire gn2 = ( SW1) & (~SW2) & ( SW3);
wire gn4 = ( SW1) & ( SW2) & (~SW3);

//wire define
wire   [3:0]          led_cmd         ;
wire                  clk_100m        ;
wire                  clk_100m_shift  ;
wire                  clk_25m         ;
wire                  locked          ;
wire                  rst_n           ;

wire                  i2c_exec        ;
wire   [23:0]         i2c_data        ;        
wire                  cam_init_done   ;
wire                  i2c_done        ;
wire                  i2c_dri_clk     ;
                                      
wire                  wr_en           ;
wire   [15:0]         wr_data         ;
wire                  rd_en           ;
wire   [15:0]         rd_data         ;
wire                  sdram_init_done ;
wire                  sys_init_done   ;

wire                  clk_125m        ;
wire                  clk_125m_shift  ;

assign  cam_rst_n = 1'b1;
assign  cam_pwdn = 1'b0;

//时钟
wire rx_clk_g;    //公共时钟
wire clk_125    = clk_125m;
wire clk_125_90 = clk_125m_shift;

//SDRAM适配
wire         sdram_clk  ;  
wire         sdram_cke  ;  
wire         sdram_cs_n ; 
wire         sdram_ras_n;
wire         sdram_cas_n;
wire         sdram_we_n ; 
wire  [ 1:0] sdram_ba   ;   
wire  [12:0] sdram_addr ; 
wire  [15:0] sdram_data ; 
wire  [ 1:0] sdram_dqm  ;

wire         sdram_clk_TD  ;  
wire         sdram_cke_TD  ;  
wire         sdram_cs_n_TD ; 
wire         sdram_ras_n_TD;
wire         sdram_cas_n_TD;
wire         sdram_we_n_TD ; 
wire  [ 1:0] sdram_ba_TD   ;   
wire  [10:0] sdram_addr_TD ; 
wire  [31:0] sdram_data_TD ; 
wire  [ 3:0] sdram_dqm_TD  ;

assign sdram_clk_TD = sdram_clk;
assign sdram_cke_TD = sdram_cke;
assign sdram_cs_n_TD = sdram_cs_n;
assign sdram_ras_n_TD = sdram_ras_n;
assign sdram_cas_n_TD = sdram_cas_n;
assign sdram_we_n_TD = sdram_we_n;
assign sdram_ba_TD = sdram_ba;
assign sdram_addr_TD = sdram_addr[10:0];
assign sdram_data_TD[15:0]  = sdram_data;    
assign sdram_data_TD[31:16] = 16'hzzzz;      
assign sdram_dqm_TD = {2'b11,sdram_dqm};

//
assign  rst_n = sys_rst_n & locked;
assign  sys_init_done = sdram_init_done & cam_init_done;

//功能2图像数据接口
wire         img_wr_en, img_wr_load;
wire [15:0]  img_wr_data;

wire [7:0] gmii_rxd;
wire       gmii_rx_dv;
//

wire        cam_we     = wr_en; 
wire [7:0]  fifo_dout;
wire        fifo_empty;
wire [12:0] fifo_rdusedw;
wire        fifo_re;

//功能4相关
wire [7:0]  gmii_txd;
wire        gmii_tx_en, gmii_tx_er;
wire [3:0]  tree_out;
wire        frame_val_flag;
wire        cmos_frame_href;
            
wire        lable_start;
wire [19:0] feature0   ;
wire [19:0] feature1   ;
wire        data_valid ;
wire [15:0] data       ;

//SDRAM相关读写限制
reg fd_s1, fd_s2;
always @(posedge clk_25m or negedge rst_n) begin
  if (!rst_n) begin fd_s1<=1'b0; fd_s2<=1'b0; end
  else begin fd_s1 <= frame_done_from_rx; fd_s2 <= fd_s1; end
end
wire img_frame_done_sync = fd_s1 ^ fd_s2; 

reg frame_ready;

always @(posedge clk_25m or negedge rst_n) begin
  if (!rst_n) frame_ready <= 1'b0;
  else if (img_frame_done_sync) frame_ready <= 1'b1; 
end

wire new_frame_pulse = img_frame_done_sync;  

reg pending_swap;
reg vs_d1, vs_d2;
always @(posedge clk_25m or negedge rst_n) begin
  if(!rst_n) begin
    pending_swap <= 1'b0;
    vs_d1 <= 1'b0; vs_d2 <= 1'b0;
  end else begin
    vs_d1 <= vga_vs; vs_d2 <= vs_d1;
    if(new_frame_pulse)            pending_swap <= 1'b1;       
    else if(vs_d1 & ~vs_d2)        pending_swap <= 1'b0;       
  end
end
wire vs_rise = (vs_d1 & ~vs_d2);
wire rd_load_vsync = vs_rise & pending_swap;



//公共模块
pll_clk pll_clk_inst(
  .refclk   (sys_clk),
  .reset    (~sys_rst_n),
  .extlock  (locked),
  .clk0_out (clk_100m),
  .clk1_out (clk_100m_shift), //    偏移75
  .clk2_out (clk_25m),
  .clk3_out (clk_125m), 
  .clk4_out (clk_125m_shift)  //    偏移90
);


EG_LOGIC_BUFG u_rxbufg (
  .o(rx_clk_g),
  .i(phy1_rgmii_rx_clk)
);



//功能1相关模块
rgmii_rx_wrap #(.DEVICE("EG4"), .RISE_LOW_NIBBLE(1)) u_rx (
  .phy_rxc    (rx_clk_g),
  .phy_rx_ctl (phy1_rgmii_rx_ctl),
  .phy_rxd    (phy1_rgmii_rx_data),
  .rst        (~rst_n),
  .gmii_rxd   (gmii_rxd),
  .gmii_rx_dv (gmii_rx_dv)
);


udp_led_cmd_rx #(
  .LOCAL_MAC   (LOCAL_MAC_ADDRESS),
  .LOCAL_IP    (LOCAL_IP_ADDRESS),
  .BCAST_IP    (BCAST_IP),           
  .LISTEN_PORT (LISTEN_PORT)         
) U_RX_LED (
  .clk(rx_clk_g), 
  .rst_n(rst_n), 
  .en(gn1),               //功能1使能
  .gmii_rxd(gmii_rxd), 
  .gmii_rx_dv(gmii_rx_dv),
  .led_o(led_cmd), 
  .dled_o(dled)
);

assign LED0 = led_cmd[0];
assign LED1 = led_cmd[1];
assign LED2 = led_cmd[2];
assign LED3 = led_cmd[3];



//功能2相关模块
//udp接收信息发给SDRAM
udp_img_rx_to_sdram #(     
  .LOCAL_MAC (LOCAL_MAC_ADDRESS),
  .LOCAL_IP  (LOCAL_IP_ADDRESS),
  .BCAST_IP  (BCAST_IP),
  .IMG_PORT  (IMG_PORT),
  .IMG_W     (CMOS_H_PIXEL), 
  .IMG_H     (CMOS_V_PIXEL), 
  .STRICT_UDPLEN(0)
) U_IMG_RX (
  .clk        (rx_clk_g),
  .rst_n      (rst_n),
  .en         (gn2),      //功能2使能
  .gmii_rxd   (gmii_rxd),
  .gmii_rx_dv (gmii_rx_dv),

  .wr_en      (img_wr_en),
  .wr_data    (img_wr_data),
  .wr_load    (img_wr_load),
  .frame_done (frame_done_from_rx) 
);


//传给SDRAM
sdram_top u_sdram_top(
    .ref_clk            (clk_100m),                  
    .out_clk            (clk_100m_shift),            
    .rst_n              (rst_n),                     
                                                        
    //用户写端口                                        
    .wr_clk             (rx_clk_g),            
    .wr_en              (img_wr_en),
    .wr_data            (img_wr_data),
    .wr_min_addr        (24'd0),
    .wr_max_addr        (CMOS_H_PIXEL*CMOS_V_PIXEL - 1),  // 640*480
    .wr_len             (10'd256),
    .wr_load            (~rst_n | img_wr_load),       

    //用户读端口                                        
    .rd_clk             (clk_25m),                    
    .rd_en              (rd_en),                      
    .rd_data            (rd_data),                    
    .rd_min_addr        (24'd0),                      
    .rd_max_addr        (CMOS_H_PIXEL*CMOS_V_PIXEL - 1), 
    .rd_len             (10'd256),               
    .rd_load            (~rst_n | rd_load_vsync),

    //用户控制端口                                
    .sdram_read_valid   (frame_ready),               
    .sdram_pingpang_en  (1'b0),                      
    .sdram_init_done    (sdram_init_done),           

    //SDRAM 芯片接口                                
    .sdram_clk          (sdram_clk),                 
    .sdram_cke          (sdram_cke),                 
    .sdram_cs_n         (sdram_cs_n),                
    .sdram_ras_n        (sdram_ras_n),               
    .sdram_cas_n        (sdram_cas_n),               
    .sdram_we_n         (sdram_we_n),                
    .sdram_ba           (sdram_ba),                  
    .sdram_addr         (sdram_addr),                
    .sdram_data         (sdram_data),                
    .sdram_dqm          (sdram_dqm)                  
    );

//适配安陆SDRAM
SDRAM sdram_inst(
	.clk   (sdram_clk_TD)   ,
	.ras_n (sdram_ras_n_TD) ,
	.cas_n (sdram_cas_n_TD) ,
	.we_n  (sdram_we_n_TD)  ,
	.addr  (sdram_addr_TD)  ,
	.ba    (sdram_ba_TD)    ,
	.dq    (sdram_data_TD)  ,
	.cs_n  (sdram_cs_n_TD)  ,
	.dm0   (sdram_dqm_TD[0]),
	.dm1   (sdram_dqm_TD[1]),
	.dm2   (sdram_dqm_TD[2]),
	.dm3   (sdram_dqm_TD[3]),
	.cke   (sdram_cke_TD)   
);


//读的数据传给VGA
vga_driver u_vga_driver(
    .vga_clk            (clk_25m),    
    .sys_rst_n          (rst_n),    
    
    .vga_hs             (vga_hs),       
    .vga_vs             (vga_vs),       
    .vga_rgb444            (vga_rgb),      
        
    .pixel_data         (rd_data), 
    .data_req           (rd_en),   
    .pixel_xpos         (), 
    .pixel_ypos         ()
    ); 



//功能4相关模块
i2c_ov5640_rgb565_cfg 
   #(
     .CMOS_H_PIXEL      (CMOS_H_PIXEL),
     .CMOS_V_PIXEL      (CMOS_V_PIXEL)
    )   
   u_i2c_cfg(   
    .clk                (i2c_dri_clk),
    .rst_n              (rst_n),
    .i2c_done           (i2c_done),
    .i2c_exec           (i2c_exec),
    .i2c_data           (i2c_data),
    .init_done          (cam_init_done)
    );    


i2c_dri
   #(
    .SLAVE_ADDR         (SLAVE_ADDR), 
    .CLK_FREQ           (CLK_FREQ  ),              
    .I2C_FREQ           (I2C_FREQ  )                
    )   
   u_i2c_dri(   
    .clk                (clk_25m   ),
    .rst_n              (rst_n     ),   
        
    .i2c_exec           (i2c_exec  ),   
    .bit_ctrl           (BIT_CTRL  ),   
    .i2c_rh_wl          (1'b0),     
    .i2c_addr           (i2c_data[23:8]),   
    .i2c_data_w         (i2c_data[7:0]),   
    .i2c_data_r         (),   
    .i2c_done           (i2c_done  ),   
    .scl                (cam_scl   ),   
    .sda                (cam_sda   ),   
        
    .dri_clk            (i2c_dri_clk)     
);


cmos_capture_data u_cmos_capture_data(  
    .rst_n              (rst_n & sys_init_done), 
        
    .cam_pclk           (cam_pclk),
    .cam_vsync          (cam_vsync),
    .cam_href           (cam_href),
    .cam_data           (cam_data),
        
    .frame_val_flag     (frame_val_flag),
    .cmos_frame_vsync   (),
    .cmos_frame_href    (cmos_frame_href),
    .cmos_frame_valid   (wr_en),    
    .cmos_frame_data    (wr_data)   
    );

Pretreatment Pretreatment_inst(
    .cam_pclk         (cam_pclk),
    .rst_n            (rst_n),      
    .frame_val_flag   (frame_val_flag),
    .cmos_frame_href  (cmos_frame_href),
    .cmos_frame_valid (wr_en),
    .cmos_frame_data  (wr_data),
    .tree_out         (tree_out),
    .SW4              (SW4),

    .lable_start      (lable_start),
    .feature0         (feature0),
    .feature1         (feature1),
    .data_valid       (data_valid),
    .data             (data)
);

tree tree_inst(
    .cam_pclk   (cam_pclk),
    .rst_n      (rst_n),   
    .feature0   (feature0),
    .feature1   (feature1),
    .lable_start(lable_start),

    .tree_out   (tree_out)
);


dcfifo_16xN U_CAM_FIFO (
  .di         (data),    
  .clkw       (cam_pclk),
  .wrst       (~rst_n),
  .we         (data_valid & gn4),

  .clkr       (clk_125),
  .rrst       (~rst_n),
  .re         (fifo_re),
  .dout       (fifo_dout),
  .empty_flag (fifo_empty),
  .rdusedw    (fifo_rdusedw)
);


udp_line_tx_from_fifo #(
  .SRC_MAC  (LOCAL_MAC_ADDRESS),
  .SRC_IP   (LOCAL_IP_ADDRESS),   
  .SRC_PORT (LOCAL_UDP_PORT_NUM),
  .DST_MAC  (DST_MAC_ADDRESS),
  .DST_IP   (DST_IP_ADDRESS),  
  .DST_PORT (DST_UDP_PORT_NUM),
  .IMG_W    (CMOS_H_PIXEL),
  .IMG_H    (CMOS_V_PIXEL)
) U_LINE_TX (
  .clk          (clk_125),
  .rst          (~rst_n),
  .fifo_dout    (fifo_dout),
  .fifo_empty   (fifo_empty),
  .fifo_rdusedw ({3'b000, fifo_rdusedw}),
  .fifo_re      (fifo_re),
  .gmii_txd     (gmii_txd),
  .gmii_tx_en   (gmii_tx_en),
  .gmii_tx_er   (gmii_tx_er)
);


rgmii_tx_wrap #(.DEVICE("EG4"), .RISE_LOW_NIBBLE(1)) u_tx (
  .tx_clk      (clk_125),
  .tx_clk_90   (clk_125_90),
  .rst         (~rst_n),
  .gmii_txd    (gmii_txd),
  .gmii_tx_en  (gmii_tx_en),
  .gmii_tx_er  (gmii_tx_er),
  .speed_10_100(1'b0),
  .phy_txc     (phy1_rgmii_tx_clk),
  .phy_tx_ctl  (phy1_rgmii_tx_ctl),
  .phy_txd     (phy1_rgmii_tx_data)
);


endmodule 