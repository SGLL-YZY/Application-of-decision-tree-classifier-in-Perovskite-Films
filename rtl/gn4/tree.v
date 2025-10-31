module tree(
    input              cam_pclk,
    input              rst_n,   
    input   [19:0]     feature0,
    input   [19:0]     feature1,
    input              lable_start,
    
    output reg [3:0]   tree_out
);

always @(posedge cam_pclk or negedge rst_n) begin
    if (!rst_n) begin
        tree_out <= 4'b1000; 
    end 
    else if (lable_start) begin
        if (feature0 <= 20'd200) begin
            tree_out <= 4'b1000;
        end 
        else if (feature0 <= 20'd9644) begin 
            if (feature0 <= 20'd2991) 
                tree_out <= 4'b0001;
            else                       
                tree_out <= 4'b0001;
        end 
        else begin
        if (feature0 <= 20'd18528) 
            tree_out <= 4'b0010;
        else                        
            tree_out <= 4'b0100;
        end
    end
end

endmodule