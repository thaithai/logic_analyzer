module controller (
    input  wire       clk,
    input  wire       rst_n,
    
    // Nhận lệnh từ UART RX
    input  wire [7:0] pc_command,   // Byte lệnh nhận từ PC
    input  wire       cmd_valid,    // Xung báo lệnh từ PC hợp lệ
    
    // Ngõ ra điều khiển cấu hình cho Trigger
    output reg        enable,
    output reg        edge_detect,
    output reg [2:0]  channel
);

    // Giải mã lệnh đơn giản dựa trên byte truyền từ PC:
    // Ví dụ cấu trúc byte lệnh: [7: Bit Enable] [6: Edge Detect] [5..3: Reserved] [2..0: Channel Select]
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            enable      <= 1'b0;
            edge_detect <= 1'b1; // Mặc định bắt sườn lên
            channel     <= 3'd0; // Mặc định kênh 0
        end else if (cmd_valid) begin
            enable      <= pc_command[7];
            edge_detect <= pc_command[6];
            channel     <= pc_command[2:0];
        end
    end
endmodule