module clock_divider (
    input  clk,         // Clock gốc từ PLL
    input  rst_n,          // Reset tích cực mức thấp
    input [15:0] scale,
    output reg  clk_scale         // Clock sau khi chia tần
);

    reg [31:0] counter;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            counter <= 32'd0;
            clk_scale <= 1'b0;
        end else begin
            if (counter >= scale) begin
                counter <= 32'd0;
                clk_scale <= 1'b1;
            end else begin
                counter <= counter + 1'b1;
                clk_scale <= 1'b0;
            end
        end
    end
endmodule