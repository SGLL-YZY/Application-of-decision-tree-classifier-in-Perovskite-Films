`timescale 1ns/1ps
module wrfifo (
    input              aclr,      // 高有效
    input      [15:0]  data,
    input              rdclk,
    input              rdreq,
    input              wrclk,
    input              wrreq,
    output     [15:0]  q,
    output     [9:0]   rdusedw
);
    wire wrfull_unused, rdempty_unused;
    wire [9:0] wrusedw_unused;
    async_fifo_core #(.WIDTH(16), .ADDR(10)) u_core (
        .aclr(aclr),
        .wrclk(wrclk), .wrreq(wrreq), .di(data),
        .wrfull(wrfull_unused), .wrusedw(wrusedw_unused),
        .rdclk(rdclk), .rdreq(rdreq), .do(q),
        .rdempty(rdempty_unused), .rdusedw(rdusedw)
    );
endmodule
