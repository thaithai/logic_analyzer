module timestamp_counter (
    input  clk,          
    input  rst_n,        // Reset tích cực mức thấp
    //input  enable,       // Cho phép đếm
    input clk_scale,
    output reg [7:0]  timestamp     // Giá trị mốc thời gian 8-bit
);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            timestamp <= 8'd0;
        end else if (clk_scale) begin
            timestamp <= timestamp + 1'b1; 
        end
    end
endmodule