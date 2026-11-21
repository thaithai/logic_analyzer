module trigger #(parameter DATA_WIDTH = 8)(
    input  clk,
    input  rst_n,

    // 00: logic 0, 01: logic 1, 10: cạnh lên, 11: cạnh xuống
    input  enable,
    input  [1:0] trigger_condition,
    input  [2:0] channel,

    // Dữ liệu và tín hiệu lấy mẫu
    input  [DATA_WIDTH-1:0] data,
    input  clk_scale,

    // Ngõ ra kết nối với Buffer
    output event_pulse,
    output reg trigger_detect
);

    reg  [DATA_WIDTH-1:0] prev_data;
    wire       selected_bit;
    reg        trigger_match;

    assign selected_bit = data[channel];

    // Báo có thay đổi dữ liệu tại thời điểm lấy mẫu.
    assign event_pulse = (data != prev_data) && clk_scale;

    // Kiểm tra điều kiện trigger trên kênh đã chọn.
    always @(*) begin
        case (trigger_condition)
            2'b00:   trigger_match = ~selected_bit;              // Logic 0
            2'b01:   trigger_match =  selected_bit;               // Logic 1
            2'b10:   trigger_match =  selected_bit & ~prev_data[channel]; // Cạnh lên
            2'b11:   trigger_match = ~selected_bit &  prev_data[channel]; // Cạnh xuống
            default: trigger_match = 1'b0;
        endcase
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            prev_data     <= 8'b0;
            trigger_detect <= 1'b0;
        end else begin
            // Cập nhật dữ liệu trước ở mỗi chu kỳ lấy mẫu,
            // kể cả khi trigger đang bị tắt.
            if (clk_scale)
                prev_data <= data;

            if (!enable) begin
                trigger_detect <= 1'b0;
            end else if (clk_scale && trigger_match) begin
                trigger_detect <= 1'b1;
            end
        end
    end

endmodule