// Note: cần sửa lại toàn bộ uart
// 
module uart_rx #( // xem lại sau khi check uart rx (mục tiêu uart rx là để giải mã lệnh cấu hình từ PC)
    parameter CLKS_PER_BIT = 434
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire       rx,          // Chân nhận dữ liệu nối tiếp UART RX từ phần cứng
    output reg        rx_dv,       // Cờ báo có dữ liệu mới nhận hợp lệ (1 chu kỳ clock)
    output reg  [7:0] rx_data      // Dữ liệu 8-bit nhận được
);

    localparam [2:0] 
        S_IDLE      = 3'b000,
        S_RX_START  = 3'b001,
        S_RX_DATA   = 3'b010,
        S_RX_STOP   = 3'b011,
        S_CLEANUP   = 3'b100;

    reg [2:0]  state;
    reg [15:0] clk_count;
    reg [2:0]  bit_index;
    reg        rx_sync_1, rx_sync_2; // Đồng bộ hóa tín hiệu RX bất đồng bộ

    // Lọc Metastability cho chân RX bất đồng bộ từ bên ngoài vào
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_sync_1 <= 1'b1;
            rx_sync_2 <= 1'b1;
        end else begin
            rx_sync_1 <= rx;
            rx_sync_2 <= rx_sync_1;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= S_IDLE;
            rx_dv     <= 1'b0;
            rx_data   <= 8'd0;
            clk_count <= 16'd0;
            bit_index <= 3'd0;
        end else begin
            rx_dv <= 1'b0; // Mặc định xung valid chỉ kéo dài 1 chu kỳ

            case (state)
                S_IDLE: begin
                    clk_count <= 16'd0;
                    bit_index <= 3'd0;
                    // Phát hiện sườn âm của chân RX (Bắt đầu bit Start = 0)
                    if (rx_sync_2 == 1'b0) begin
                        state <= S_RX_START;
                    end
                end

                S_RX_START: begin
                    // Đợi đến giữa chu kỳ của bit Start để lấy mẫu chính xác
                    if (clk_count == (CLKS_PER_BIT / 2)) begin
                        if (rx_sync_2 == 1'b0) begin
                            clk_count <= 16'd0;
                            state     <= S_RX_DATA;
                        end else begin
                            state     <= S_IDLE; // Nhiễu giả mạo, quay về IDLE
                        end
                    end else begin
                        clk_count <= clk_count + 1'b1;
                    end
                end

                S_RX_DATA: begin
                    if (clk_count < CLKS_PER_BIT - 1) begin
                        clk_count <= clk_count + 1'b1;
                    end else begin
                        clk_count             <= 16'd0;
                        rx_data[bit_index]    <= rx_sync_2; // Lấy mẫu từng bit từ LSB đến MSB
                        
                        if (bit_index < 3'd7) begin
                            bit_index <= bit_index + 1'b1;
                        end else begin
                            bit_index <= 3'd0;
                            state     <= S_RX_STOP;
                        end
                    end
                end

                S_RX_STOP: begin
                    // Đợi hết chu kỳ bit Stop
                    if (clk_count < CLKS_PER_BIT - 1) begin
                        clk_count <= clk_count + 1'b1;
                    end else begin
                        rx_dv     <= 1'b1; // Bật cờ báo có dữ liệu mới
                        clk_count <= 16'd0;
                        state     <= S_CLEANUP;
                    end
                end

                S_CLEANUP: begin
                    state <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end
endmodule