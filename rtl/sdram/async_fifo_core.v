`timescale 1ns/1ps
// ------------------------------------------------------
// Vendor-agnostic async FIFO core (Verilog-2001版, 无 '0 用法)
// - WIDTH: data width
// - ADDR : log2(DEPTH); DEPTH must be power-of-two
// - Non-showahead read: q updates on (rdreq && !empty)
// - rdusedw is in rdclk domain; wrusedw is in wrclk domain
// ------------------------------------------------------
module async_fifo_core #(
    parameter integer WIDTH = 16,
    parameter integer ADDR  = 10
)(
    input                   aclr,          // async reset, active-high

    // write domain
    input                   wrclk,
    input                   wrreq,
    input      [WIDTH-1:0]  di,
    output                  wrfull,
    output reg [ADDR-1:0]   wrusedw,      // in wrclk domain

    // read domain
    input                   rdclk,
    input                   rdreq,
    output reg [WIDTH-1:0]  do,
    output                  rdempty,
    output reg [ADDR-1:0]   rdusedw       // in rdclk domain
);
    localparam integer DEPTH = (1<<ADDR);

    // memory
    (* ram_style = "block" *) reg [WIDTH-1:0] mem [0:DEPTH-1];

    // binary/gray pointers (one extra MSB for wrap)
    reg [ADDR:0] wr_bin, rd_bin;
    reg [ADDR:0] wr_gray, rd_gray;

    // cross-domain sync (gray)
    reg [ADDR:0] rd_gray_wrclk_d1, rd_gray_wrclk_d2;
    reg [ADDR:0] wr_gray_rdclk_d1, wr_gray_rdclk_d2;

    // gray<->bin functions
    integer i;
    function [ADDR:0] bin2gray;
        input [ADDR:0] b;
        begin
            bin2gray = (b >> 1) ^ b;
        end
    endfunction

    function [ADDR:0] gray2bin;
        input [ADDR:0] g;
        begin
            gray2bin[ADDR] = g[ADDR];
            for (i = ADDR-1; i >= 0; i = i-1)
                gray2bin[i] = gray2bin[i+1] ^ g[i];
        end
    endfunction

    // ---------------- write domain ----------------
    wire [ADDR:0] rd_gray_w = rd_gray_wrclk_d2;
    wire [ADDR:0] rd_bin_w  = gray2bin(rd_gray_w);

    // +1 (带位宽)
    wire [ADDR:0] ONE_ADDRP1 = {{ADDR{1'b0}}, 1'b1};

    assign wrfull = ( bin2gray(wr_bin + ONE_ADDRP1)
                    == {~rd_gray_w[ADDR:ADDR-1], rd_gray_w[ADDR-2:0]} );

    always @(posedge wrclk or posedge aclr) begin
        if (aclr) begin
            wr_bin  <= { (ADDR+1){1'b0} };
            wr_gray <= { (ADDR+1){1'b0} };
        end else begin
            if (wrreq && !wrfull) begin
                mem[wr_bin[ADDR-1:0]] <= di;
                wr_bin  <= wr_bin + ONE_ADDRP1;
                wr_gray <= bin2gray(wr_bin + ONE_ADDRP1);
            end
        end
    end

    // sync read gray into write domain
    always @(posedge wrclk or posedge aclr) begin
        if (aclr) begin
            rd_gray_wrclk_d1 <= { (ADDR+1){1'b0} };
            rd_gray_wrclk_d2 <= { (ADDR+1){1'b0} };
        end else begin
            rd_gray_wrclk_d1 <= rd_gray;
            rd_gray_wrclk_d2 <= rd_gray_wrclk_d1;
        end
    end

    // wrusedw in wrclk domain
    always @(posedge wrclk or posedge aclr) begin
        if (aclr) wrusedw <= { ADDR{1'b0} };
        else      wrusedw <= wr_bin[ADDR-1:0] - rd_bin_w[ADDR-1:0];
    end

    // ---------------- read domain ----------------
    wire [ADDR:0] wr_gray_r = wr_gray_rdclk_d2;
    wire [ADDR:0] wr_bin_r  = gray2bin(wr_gray_r);

    assign rdempty = (rd_gray == wr_gray_r);

    always @(posedge rdclk or posedge aclr) begin
        if (aclr) begin
            rd_bin  <= { (ADDR+1){1'b0} };
            rd_gray <= { (ADDR+1){1'b0} };
            do      <= { WIDTH{1'b0} };
        end else begin
            if (rdreq && !rdempty) begin
                do     <= mem[rd_bin[ADDR-1:0]];
                rd_bin <= rd_bin + ONE_ADDRP1;
                rd_gray<= bin2gray(rd_bin + ONE_ADDRP1);
            end
        end
    end

    // sync write gray into read domain
    always @(posedge rdclk or posedge aclr) begin
        if (aclr) begin
            wr_gray_rdclk_d1 <= { (ADDR+1){1'b0} };
            wr_gray_rdclk_d2 <= { (ADDR+1){1'b0} };
        end else begin
            wr_gray_rdclk_d1 <= wr_gray;
            wr_gray_rdclk_d2 <= wr_gray_rdclk_d1;
        end
    end

    // rdusedw in rdclk domain
    always @(posedge rdclk or posedge aclr) begin
        if (aclr) rdusedw <= { ADDR{1'b0} };
        else      rdusedw <= wr_bin_r[ADDR-1:0] - rd_bin[ADDR-1:0];
    end
endmodule
