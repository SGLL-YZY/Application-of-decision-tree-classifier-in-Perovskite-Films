module sdram_cmd(
    input             clk,              //系统时钟
    input             rst_n,            //低电平复位信号

    input      [23:0] sys_wraddr,       //写SDRAM时地址（以16bit为单位的线性地址）
    input      [23:0] sys_rdaddr,       //读SDRAM时地址
    input      [ 9:0] sdram_wr_burst,   //突发写SDRAM字个数（16bit为1个）
    input      [ 9:0] sdram_rd_burst,   //突发读SDRAM字个数
    
    input      [ 4:0] init_state,       //SDRAM初始化状态
    input      [ 3:0] work_state,       //SDRAM工作状态
    input      [ 9:0] cnt_clk,          //延时计数器 
    input             sdram_rd_wr,      //读/写控制, 0=写, 1=读
    
    output            sdram_cke,        //SDRAM时钟有效信号
    output            sdram_cs_n,       //SDRAM片选信号
    output            sdram_ras_n,      //SDRAM行地址选通脉冲
    output            sdram_cas_n,      //SDRAM列地址选通脉冲
    output            sdram_we_n,       //SDRAM写允许位
    output reg [ 1:0] sdram_ba,         //SDRAM L-Bank地址线
    output reg [12:0] sdram_addr        //SDRAM地址总线（A12..A0）
    );

`include "sdram_para.v"                 //包含SDRAM参数定义

// ===================================================
//            ★ 关键几何：2M x 32bit, 4 Banks ★
//   Row = 11bit,  Col = 8bit,  Bank = 2bit
//   线性地址切片：{bank[23:22], row[20:10], col8[8:1], half[0]}
//   我们只用 32bit 的低半 16bit ⇒ half 固定为 0（A0=0）
//   READ/WRITE 时 A10=AP，这里保持 0（不用自动预充电，因为状态机有 W_PRE）
// ===================================================
localparam integer BANK_BITS = 2;
localparam integer ROW_BITS  = 11;
localparam integer COL_BITS  = 8;

wire [4:0]  sdram_cmd_r;
reg  [4:0]  sdram_cmd_q;

assign {sdram_cke,sdram_cs_n,sdram_ras_n,sdram_cas_n,sdram_we_n} = sdram_cmd_q;

// 选读/写地址
wire [23:0] sys_addr = sdram_rd_wr ? sys_rdaddr : sys_wraddr;

// 帧内跨 Bank + 帧间乒乓
wire [1:0] bank = { sys_addr[23], 1'b0 };   // ★ 帧内固定 bank=0，帧间乒乓用 bit23
wire [10:0] row  = sys_addr[18:8];                 // 11 位行
wire [7:0]  col8 = sys_addr[7:0];                  // 8 位列

// 地址拼接（输出到 13 根地址线 A12..A0；顶层只会接出 A10..A0 到 32bit 颗粒）
wire        ap_bit   = 1'b0;                  // 现阶段不用自动预充电，保持 0
wire [12:0] act_addr = { 2'b00, row };        // ACT:   A12..A11=0, A10..A0 = row[10:0]
wire [12:0] rw_addr  = { 3'b000, ap_bit, col8, 1'b0 };  // ★ A0=0，列=col8 放在 A8..A1

// SDRAM 操作指令控制
always @ (posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        sdram_cmd_q <= `CMD_INIT;
        sdram_ba    <= 2'b11;
        sdram_addr  <= 13'h1fff;
    end
    else begin
        case(init_state)
            // 初始化阶段
            `I_NOP,`I_TRP,`I_TRF,`I_TRSC: begin
                sdram_cmd_q <= `CMD_NOP;
                sdram_ba    <= 2'b11;
                sdram_addr  <= 13'h1fff;    
            end
            `I_PRE: begin               // 预充电（all banks）
                sdram_cmd_q <= `CMD_PRGE;
                sdram_ba    <= 2'b11;
                sdram_addr  <= 13'h1fff;
            end 
            `I_AR: begin               // 自动刷新
                sdram_cmd_q <= `CMD_A_REF;
                sdram_ba    <= 2'b11;
                sdram_addr  <= 13'h1fff;                        
            end                 
            `I_MRS: begin              // 模式寄存器设置
                sdram_cmd_q <= `CMD_LMR;
                sdram_ba    <= 2'b00;
                sdram_addr  <= {
                    3'b000,    // N/A
                    1'b0,      // A9: Write burst = Programmed burst length (0)
                    2'b00,     // A8..A7: N/A for x32, 置0
                    3'b011,    // A6..A4: CAS Latency = 3
                    1'b0,      // A3: Burst Type = Sequential
                    3'b111     // A2..A0: Burst Length = Full Page（保持你的原设置）
                };
            end 

            // 初始化完成 → 工作阶段
            `I_DONE: begin
                case(work_state)
                    `W_IDLE,`W_TRCD,`W_CL,`W_TWR,`W_TRP,`W_TRFC: begin
                        sdram_cmd_q <= `CMD_NOP;
                        sdram_ba    <= 2'b11;
                        sdram_addr  <= 13'h1fff;
                    end

                    // ★★★ 行激活：用 11 位 row ★★★
                    `W_ACTIVE: begin
                        sdram_cmd_q <= `CMD_ACTIVE;
                        sdram_ba    <= bank;
                        sdram_addr  <= act_addr;     // {2'b00, row}
                    end

                    // ★★★ 读命令：A10=AP=0，列 8 位，A0=0 固定低半字 ★★★
                    `W_READ: begin
                        sdram_cmd_q <= `CMD_READ;
                        sdram_ba    <= bank;
                        sdram_addr  <= rw_addr;      // {3'b000, ap(0), col8, 1'b0}
                    end

                    `W_RD: begin
                        if(`end_rdburst)
                            sdram_cmd_q <= `CMD_B_STOP;
                        else begin
                            sdram_cmd_q <= `CMD_NOP;
                            sdram_ba    <= 2'b11;
                            sdram_addr  <= 13'h1fff;
                        end
                    end

                    // ★★★ 写命令：同读命令的列地址打包 ★★★
                    `W_WRITE: begin
                        sdram_cmd_q <= `CMD_WRITE;
                        sdram_ba    <= bank;
                        sdram_addr  <= rw_addr;      // {3'b000, ap(0), col8, 1'b0}
                    end

                    `W_WD: begin
                        if(`end_wrburst)
                            sdram_cmd_q <= `CMD_B_STOP;
                        else begin
                            sdram_cmd_q <= `CMD_NOP;
                            sdram_ba    <= 2'b11;
                            sdram_addr  <= 13'h1fff;
                        end
                    end

                    `W_PRE: begin     // 预充电（A10=1）
                        sdram_cmd_q <= `CMD_PRGE;
                        sdram_ba    <= bank;
                        sdram_addr  <= 13'h0400;     // A10=1
                    end

                    `W_AR: begin
                        sdram_cmd_q <= `CMD_A_REF;
                        sdram_ba    <= 2'b11;
                        sdram_addr  <= 13'h1fff;
                    end

                    default: begin
                        sdram_cmd_q <= `CMD_NOP;
                        sdram_ba    <= 2'b11;
                        sdram_addr  <= 13'h1fff;
                    end
                endcase
            end

            default: begin
                sdram_cmd_q <= `CMD_NOP;
                sdram_ba    <= 2'b11;
                sdram_addr  <= 13'h1fff;
            end
        endcase
    end
end

endmodule
