module Pretreatment(
    input               cam_pclk,
    input               rst_n,      
    input               frame_val_flag,
    input               cmos_frame_href,
    input               cmos_frame_valid,
    input  [15:0]       cmos_frame_data,
    input  [3:0]        tree_out,
    input               SW4,
    
    output              lable_start,
    output reg [19:0]   feature0,
    output reg [19:0]   feature1,
    output reg          data_valid,
    output reg [15:0]   data
);

reg  cmos_frame_href_reg ;
wire cmos_frame_href_low;
assign cmos_frame_href_low = (!cmos_frame_href & cmos_frame_href_reg);


reg        cmos_frame_valid_reg;
reg [15:0] cmos_frame_data_reg ;

always @(posedge cam_pclk or negedge rst_n) begin
    if(!rst_n) begin
        cmos_frame_href_reg   <= 1'b0;
        cmos_frame_valid_reg  <= 1'b0;
        cmos_frame_data_reg   <= 16'd0;
    end
    else begin
        cmos_frame_href_reg   <= cmos_frame_href;
        cmos_frame_valid_reg  <= cmos_frame_valid;
        cmos_frame_data_reg   <= cmos_frame_data;
    end
end

reg [9:0] cnt_h;
reg [9:0] cnt_v;

always @(posedge cam_pclk or negedge rst_n) begin
    if (!rst_n) begin
        cnt_h <= 10'd0;
        cnt_v <= 10'd0;
    end else begin
        if (cmos_frame_href_low) begin
            cnt_h <= 10'd0;
            cnt_v <= (cnt_v == 10'd479) ? 10'd0 : (cnt_v + 10'd1);
        end else if (frame_val_flag && cmos_frame_valid) begin
            cnt_h <= cnt_h + 10'd1;
        end
    end
end


wire [7:0] cmos_frame_data_reg_R = {cmos_frame_data_reg[15:11],3'b000};
wire [7:0] cmos_frame_data_reg_G = {cmos_frame_data_reg[10:5],2'b00};
wire [7:0] cmos_frame_data_reg_B = {cmos_frame_data_reg[4:0],3'b000};

wire [7:0] cmos_frame_data_reg_grey;
assign cmos_frame_data_reg_grey = (cmos_frame_data_reg_R >> 2) + (cmos_frame_data_reg_G >> 1) + (cmos_frame_data_reg_B >> 2);

wire cmos_frame_data_reg_Bina = (cmos_frame_data_reg_grey < 8'd105) ? 1'b1 : 1'b0; //0-1形式输出

wire cmos_frame_href_high = (cmos_frame_href & !cmos_frame_href_reg);

always @(posedge cam_pclk or negedge rst_n) begin
    if(!rst_n) begin
        feature0 <= 20'd0;
        feature1 <= 20'd0;
    end
    else if(cnt_h == 10'd639 && cnt_v == 10'd479)begin
        feature0 <= 20'd0;
        feature1 <= 20'd0;
    end
    else if((cnt_h > 10'd169) && (cnt_h < 10'd470) && (cnt_v > 10'd89) && (cnt_v < 10'd390))begin
        if(frame_val_flag == 1'b1 && cmos_frame_valid_reg == 1'b1 && cmos_frame_data_reg_Bina == 1'b1)
            feature0 <= feature0 + 20'd1;
        else if(frame_val_flag == 1'b1 && cmos_frame_valid_reg == 1'b1 && cmos_frame_data_reg_Bina == 1'b0)
            feature1 <= feature1 + 20'd1;
    end
end

assign lable_start = (cnt_h == 10'd499 && cnt_v == 10'd399 && cmos_frame_valid_reg) ? 1'b1 : 1'b0;

localparam integer BOX_X0 = 10'd169;
localparam integer BOX_Y0 = 10'd89;
localparam integer BOX_W  = 10'd301;
localparam integer BOX_H  = 10'd301; 
localparam integer BOX_X1 = BOX_X0 + BOX_W;   // 结束坐标（含）
localparam integer BOX_Y1 = BOX_Y0 + BOX_H;   // 结束坐标（含）

wire in_top_edge    = (cnt_v == BOX_Y0) && (cnt_h >= BOX_X0) && (cnt_h <= BOX_X1);
wire in_bottom_edge = (cnt_v == BOX_Y1) && (cnt_h >= BOX_X0) && (cnt_h <= BOX_X1);
wire in_left_edge   = (cnt_h == BOX_X0) && (cnt_v >= BOX_Y0) && (cnt_v <= BOX_Y1);
wire in_right_edge  = (cnt_h == BOX_X1) && (cnt_v >= BOX_Y0) && (cnt_v <= BOX_Y1);
wire in_box_edge    = in_top_edge | in_bottom_edge | in_left_edge | in_right_edge;


parameter   BLACK   =   16'h0000,   //黑色
            WHITE   =   16'hFFFF;   //白色

reg [31:0] char_data0 [0:31];
reg [31:0] char_data1 [0:31];
reg [31:0] char_data2 [0:31];
reg [31:0] char_data3 [0:31];

always @(posedge cam_pclk or negedge rst_n) begin
    if(!rst_n) begin
        data <= BLACK;
    end
    else if((cnt_h >= 0) && (cnt_h < 32) && (cnt_v >= 0) && (cnt_v < 32) && (cmos_frame_valid_reg == 1'b1) && (!SW4)) begin
        case (tree_out)
            4'b0001: if (char_data0[cnt_v][31 - cnt_h]) data <= BLACK; else data <= WHITE;
            4'b0010: if (char_data1[cnt_v][31 - cnt_h]) data <= BLACK; else data <= WHITE;
            4'b0100: if (char_data2[cnt_v][31 - cnt_h]) data <= BLACK; else data <= WHITE;
            4'b1000: if (char_data3[cnt_v][31 - cnt_h]) data <= BLACK; else data <= WHITE;
            default: data = BLACK; // 默认显示黑色
        endcase
    end
    else if(in_box_edge == 1'b1 && (!SW4))
        data <= BLACK;
    else
        data <= cmos_frame_data_reg;
end

always @(posedge cam_pclk or negedge rst_n)
    if(!rst_n)
        data_valid <= 1'b0;
    else
        data_valid <= cmos_frame_valid_reg;


// 初始化类别"A"的32x32字模数据
initial begin
    char_data0[0]  = 32'b00000000_00000000_00000000_00000000;
    char_data0[1]  = 32'b00000000_00000000_00000000_00000000;
    char_data0[2]  = 32'b00000000_00000011_11000000_00000000;
    char_data0[3]  = 32'b00000000_00001110_01110000_00000000;
    char_data0[4]  = 32'b00000000_00111000_00111000_00000000;
    char_data0[5]  = 32'b00000000_01110000_00011100_00000000;
    char_data0[6]  = 32'b00000000_11100000_00011100_00000000;
    char_data0[7]  = 32'b00000001_11100000_00001110_00000000;
    char_data0[8]  = 32'b00000001_11000000_00000011_10000000;
    char_data0[9]  = 32'b00000011_10000000_00000011_10000000;
    char_data0[10] = 32'b00000111_00000000_00000001_11000000;
    char_data0[11] = 32'b00001110_00000000_00000000_11100000;
    char_data0[12] = 32'b00001110_00000000_00000000_11100000;
    char_data0[13] = 32'b00001110_00000000_00000000_11100000;
    char_data0[14] = 32'b00001110_00000000_00000000_11100000;
    char_data0[15] = 32'b00001110_00000000_00000000_11100000;
    char_data0[16] = 32'b00011111_11111111_11111111_11110000;
    char_data0[17] = 32'b01011111_11111111_11111111_11110000;
    char_data0[18] = 32'b00011100_00000000_00000000_01110000;
    char_data0[19] = 32'b00111000_00000000_00000000_00111000;
    char_data0[20] = 32'b00111000_00000000_00000000_00111000;
    char_data0[21] = 32'b00111000_00000000_00000000_00111000;
    char_data0[22] = 32'b00111000_00000000_00000000_00111000;
    char_data0[23] = 32'b01110000_00000000_00000000_00011100;
    char_data0[24] = 32'b01110000_00000000_00000000_00011100;
    char_data0[25] = 32'b01110000_00000000_00000000_00011100;
    char_data0[26] = 32'b01110000_00000000_00000000_00011100;
    char_data0[27] = 32'b01110000_00000000_00000000_00011100;
    char_data0[28] = 32'b01110000_00000000_00000000_00011100;
    char_data0[29] = 32'b01110000_00000000_00000000_00011100;
    char_data0[30] = 32'b00000000_00000000_00000000_00000000;
    char_data0[31] = 32'b00000000_00000000_00000000_00000000;
end

// 初始化类别"B"的32x32字模数据
initial begin
    char_data1[0]  = 32'b00000000_00000000_00000000_00000000;
    char_data1[1]  = 32'b00000000_00000000_00000000_00000000;
    char_data1[2]  = 32'b00001111_11111111_11111111_11100000;
    char_data1[3]  = 32'b00001111_11111111_11111111_11110000;
    char_data1[4]  = 32'b00001111_11111111_11111111_11110000;
    char_data1[5]  = 32'b00001111_00000000_00000000_11110000;
    char_data1[6]  = 32'b00001111_00000000_00000000_11110000;
    char_data1[7]  = 32'b00001111_00000000_00000000_11110000;
    char_data1[8]  = 32'b00001111_00000000_00000000_11110000;
    char_data1[9]  = 32'b00001111_00000000_00000000_11110000;
    char_data1[10] = 32'b00001111_00000000_00000000_11110000;
    char_data1[11] = 32'b00001111_00000000_00000000_11110000;
    char_data1[12] = 32'b00001111_00000000_00000000_11110000;
    char_data1[13] = 32'b00001111_00000000_00000000_11100000;
    char_data1[14] = 32'b00001111_11111111_11111111_11000000;
    char_data1[15] = 32'b00001111_11111111_11111111_11000000;
    char_data1[16] = 32'b00001111_11111111_11111111_11100000;
    char_data1[17] = 32'b01001111_00000000_00000000_11110000;
    char_data1[18] = 32'b00001111_00000000_00000000_11110000;
    char_data1[19] = 32'b00001111_00000000_00000000_11110000;
    char_data1[20] = 32'b00001111_00000000_00000000_11110000;
    char_data1[21] = 32'b00001111_00000000_00000000_11110000;
    char_data1[22] = 32'b00001111_00000000_00000000_11110000;
    char_data1[23] = 32'b00001111_00000000_00000000_11110000;
    char_data1[24] = 32'b00001111_00000000_00000000_11110000;
    char_data1[25] = 32'b00001111_11111111_11111111_11110000;
    char_data1[26] = 32'b00001111_11111111_11111111_11110000;
    char_data1[27] = 32'b00001111_11111111_11111111_11100000;
    char_data1[28] = 32'b00000000_00000000_00000000_00000000;
    char_data1[29] = 32'b00000000_00000000_00000000_00000000;
    char_data1[30] = 32'b00000000_00000000_00000000_00000000;
    char_data1[31] = 32'b00000000_00000000_00000000_00000000;
end

// 初始化类别"C"的32x32字模数据
initial begin
    char_data2[0]  = 32'b00000000_00000000_00000000_00000000;
    char_data2[1]  = 32'b00000000_00000000_00000000_00000000;
    char_data2[2]  = 32'b00001111_11111111_11111111_11110000;
    char_data2[3]  = 32'b00001111_11111111_11111111_11110000;
    char_data2[4]  = 32'b00001111_11111111_11111111_11110000;
    char_data2[5]  = 32'b00001111_00000000_00000000_00000000;
    char_data2[6]  = 32'b00001111_00000000_00000000_00000000;
    char_data2[7]  = 32'b00001111_00000000_00000000_00000000;
    char_data2[8]  = 32'b00001111_00000000_00000000_00000000;
    char_data2[9]  = 32'b00001111_00000000_00000000_00000000;
    char_data2[10] = 32'b00001111_00000000_00000000_00000000;
    char_data2[11] = 32'b00001111_00000000_00000000_00000000;
    char_data2[12] = 32'b00001111_00000000_00000000_00000000;
    char_data2[13] = 32'b00001111_00000000_00000000_00000000;
    char_data2[14] = 32'b00001111_00000000_00000000_00000000;
    char_data2[15] = 32'b00001111_00000000_00000000_00000000;
    char_data2[16] = 32'b00001111_00000000_00000000_00000000;
    char_data2[17] = 32'b01001111_00000000_00000000_00000000;
    char_data2[18] = 32'b00001111_00000000_00000000_00000000;
    char_data2[19] = 32'b00001111_00000000_00000000_00000000;
    char_data2[20] = 32'b00001111_00000000_00000000_00000000;
    char_data2[21] = 32'b00001111_00000000_00000000_00000000;
    char_data2[22] = 32'b00001111_00000000_00000000_00000000;
    char_data2[23] = 32'b00001111_00000000_00000000_00000000;
    char_data2[24] = 32'b00001111_11111111_11111111_11110000;
    char_data2[25] = 32'b00001111_11111111_11111111_11110000;
    char_data2[26] = 32'b00001111_11111111_11111111_11110000;
    char_data2[27] = 32'b00000000_00000000_00000000_00000000;
    char_data2[28] = 32'b00000000_00000000_00000000_00000000;
    char_data2[29] = 32'b00000000_00000000_00000000_00000000;
    char_data2[30] = 32'b00000000_00000000_00000000_00000000;
    char_data2[31] = 32'b00000000_00000000_00000000_00000000;
end

// 初始化类别默认的32x32字模数据
initial begin
    char_data3[0]  = 32'b00000000_00000000_00000000_00000000;
    char_data3[1]  = 32'b00000000_00000000_00000000_00000000;
    char_data3[2]  = 32'b00000000_00000000_00000000_00000000;
    char_data3[3]  = 32'b00000000_00000000_00000000_00000000;
    char_data3[4]  = 32'b00000000_00000000_00000000_00000000;
    char_data3[5]  = 32'b00000000_00000000_00000000_00000000;
    char_data3[6]  = 32'b00000000_00000000_00000000_00000000;
    char_data3[7]  = 32'b00000000_00000000_00000000_00000000;
    char_data3[8]  = 32'b00000000_00000000_00000000_00000000;
    char_data3[9]  = 32'b00000000_00000000_00000000_00000000;
    char_data3[10] = 32'b00000000_00000000_00000000_00000000;
    char_data3[11] = 32'b00000000_00000000_00000000_00000000;
    char_data3[12] = 32'b00000000_00000000_00000000_00000000;
    char_data3[13] = 32'b00000000_00000000_00000000_00000000;
    char_data3[14] = 32'b00000000_00000000_00000000_00000000;
    char_data3[15] = 32'b00000000_00000000_00000000_00000000;
    char_data3[16] = 32'b00000000_00000000_00000000_00000000;
    char_data3[17] = 32'b00000000_00000000_00000000_00000000;
    char_data3[18] = 32'b00000000_00000000_00000000_00000000;
    char_data3[19] = 32'b00000000_00000000_00000000_00000000;
    char_data3[20] = 32'b00000000_00000000_00000000_00000000;
    char_data3[21] = 32'b00000000_00000000_00000000_00000000;
    char_data3[22] = 32'b00000000_00000000_00000000_00000000;
    char_data3[23] = 32'b00000000_00000000_00000000_00000000;
    char_data3[24] = 32'b00000000_00000000_00000000_00000000;
    char_data3[25] = 32'b00000000_00000000_00000000_00000000;
    char_data3[26] = 32'b00000000_00000000_00000000_00000000;
    char_data3[27] = 32'b00000000_00000000_00000000_00000000;
    char_data3[28] = 32'b00000000_00000000_00000000_00000000;
    char_data3[29] = 32'b00000000_00000000_00000000_00000000;
    char_data3[30] = 32'b00000000_00000000_00000000_00000000;
    char_data3[31] = 32'b00000000_00000000_00000000_00000000;
end

endmodule
