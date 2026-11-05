module input_sync #(parameter DATA_WIDTH = 8)(
    input clk,
    input rst_n,
    input [DATA_WIDTH-1:0] in,
    output reg [DATA_WIDTH-1:0] in_sync
);
    reg [DATA_WIDTH-1:0] sync_stage;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sync_stage <= {DATA_WIDTH{1'b0}};
            in_sync     <= {DATA_WIDTH{1'b0}};
        end else begin
            sync_stage <= in;          
            in_sync     <= sync_stage;
        end
    end
endmodule