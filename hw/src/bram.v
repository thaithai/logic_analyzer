module bram #(
    parameter DATA_WIDTH = 16,
    parameter MEM_DEPTH  = 8192
)(
    input  wire                        clk,

    // Cổng ghi (Write Port)
    input  wire                        we,
    input  wire [$clog2(MEM_DEPTH)-1:0] waddr,
    input  wire [DATA_WIDTH-1:0]       wdata,

    // Cổng đọc đồng bộ (Synchronous Read Port)
    input  wire [$clog2(MEM_DEPTH)-1:0] raddr,
    output reg  [DATA_WIDTH-1:0]       rdata
);

    // Ép trình tổng hợp Gowin sử dụng khối BSRAM chuyên dụng
    (* syn_ramstyle = "block_ram" *) reg [DATA_WIDTH-1:0] mem [0:MEM_DEPTH-1];

    // Không dùng tín hiệu reset (rst_n) trong khối này để đảm bảo suy diễn ra BSRAM
    always @(posedge clk) begin
        if (we) begin
            mem[waddr] <= wdata;
        end
        rdata <= mem[raddr];
    end

endmodule