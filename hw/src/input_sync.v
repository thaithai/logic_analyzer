// module input_sync #(parameter DATA_WIDTH = 8)(
//     input clk,
//     input rst_n,
//     input [DATA_WIDTH-1:0] in,
//     output reg [DATA_WIDTH-1:0] in_sync
// );
//     reg [DATA_WIDTH-1:0] sync_stage;

//     always @(posedge clk or negedge rst_n) begin
//         if (!rst_n) begin
//             sync_stage <= {DATA_WIDTH{1'b0}};
//             in_sync     <= {DATA_WIDTH{1'b0}};
//         end else begin
//             sync_stage <= in;          
//             // in_sync     <= sync_stage;
//             in_sync <= 8'b01100110;
//         end
//     end
// endmodule

module input_sync #(
    parameter DATA_WIDTH = 8,
    parameter DIV_BITS   = 8   // Tại 50 MHz: 2^8 * 20 ns = 5.12 us mỗi bước đếm
)(
    input                       clk,
    input                       rst_n,
    input      [DATA_WIDTH-1:0] in,
    output reg [DATA_WIDTH-1:0] in_sync
);

    localparam CNT_WIDTH = DIV_BITS + DATA_WIDTH;
    reg [CNT_WIDTH-1:0] counter;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            counter <= {CNT_WIDTH{1'b0}};
            in_sync <= {DATA_WIDTH{1'b0}};
        end else begin
            counter <= counter + 1'b1;
            in_sync <= counter[CNT_WIDTH-1 : DIV_BITS];
        end
    end

endmodule