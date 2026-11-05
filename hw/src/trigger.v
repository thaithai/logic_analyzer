module trigger (
    input  clk,
    input  rst_n,
    
    // Giao tiếp cấu hình từ Controller
    input  enable,       // Bật/tắt tính năng trigger
    input  edge_detect,  // 1: Cạnh lên (Rising), 0: Cạnh xuống (Falling)
    input  [2:0] channel,      // Chọn kênh cần theo dõi (0 đến 7 cho bus 8-bit)
    
    // Dữ liệu và Timestamp đầu vào
    input  [7:0] data,         // Dữ liệu từ input_sync
    input  [7:0] timestamp,    // Timestamp hiện tại
    
    // Ngõ ra kết nối với Buffer
    output reg        trigger_detect, // Cờ báo đã phát hiện trigger// cần check
    output reg [7:0]  config_out    // Gói cấu hình chuyển sang buffer (nếu cần)
);

    reg prev_bit;
    wire target_bit;

    // Trích xuất bit cần quan sát dựa trên channel (3 bit chọn 1 trong 8 đường data)
    assign target_bit = data[channel];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            trigger_detect <= 1'b0;
            prev_bit       <= 1'b0;
            config_out     <= 8'd0;
        end else begin
            prev_bit <= target_bit;
            
            if (enable && !trigger_detect) begin
                // Phát hiện sườn lên hoặc sườn xuống tùy theo cấu hình edge_detect
                if (edge_detect && (~prev_bit & target_bit)) begin
                    trigger_detect <= 1'b1; // Sườn lên (0 -> 1)
                end else if (!edge_detect && (prev_bit & ~target_bit)) begin
                    trigger_detect <= 1'b1; // Sườn xuống (1 -> 0)
                end
            end else if (!enable) begin
                trigger_detect <= 1'b0; // Reset trạng thái trigger khi disable
            end
        end
    end
endmodule