module uart_tx #( // cần xem lại logic của uart (ưu tiên uart tx để truyền data lên PC)
    parameter CLKS_PER_BIT = 434 // Tính bằng: Tần số Clock / Baudrate (Ví dụ: 50MHz / 115200 ≈ 434)
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire       tx_start,    // Xung kích hoạt bắt đầu gửi 1 byte
    input  wire [7:0] tx_data,     // Dữ liệu 8-bit cần gửi
    output reg        tx_busy,     // Báo bận (đang trong quá trình truyền)
    output reg        tx,          // Chân nối tiếp UART TX ra phần cứng
    output reg        tx_done      // Xung báo đã truyền xong 1 byte (1 chu kỳ clock)
);

    localparam [2:0] 
        S_IDLE  = 3'b000,
        S_START = 3'b001,
        S_DATA  = 3'b010,
        S_STOP  = 3'b011;

    reg [2:0]  state;
    reg [15:0] clk_count;
    reg [2:0]  bit_index;
    reg [7:0]  tx_data_reg;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state       <= S_IDLE;
            tx          <= 1'b1;
            tx_busy     <= 1'b0;
            tx_done     <= 1'b0;
            clk_count   <= 16'd0;
            bit_index   <= 3'd0;
            tx_data_reg <= 8'd0;
        end else begin
            tx_done <= 1'b0; // Mặc định xung done chỉ tồn tại 1 chu kỳ

            case (state)
                S_IDLE: begin
                    tx      <= 1'b1;
                    tx_busy <= 1'b0;
                    clk_count <= 16'd0;
                    bit_index <= 3'd0;

                    if (tx_start) begin
                        tx_data_reg <= tx_data;
                        tx_busy     <= 1'b1;
                        state       <= S_START;
                    end
                end

                S_START: begin
                    tx <= 1'b0; // Bit Start (mức 0)
                    if (clk_count < CLKS_PER_BIT - 1) begin
                        clk_count <= clk_count + 1'b1;
                    end else begin
                        clk_count <= 16'd0;
                        state     <= S_DATA;
                    end
                end

                S_DATA: begin
                    tx <= tx_data_reg[bit_index]; // Truyền từ LSB đến MSB
                    if (clk_count < CLKS_PER_BIT - 1) begin
                        clk_count <= clk_count + 1'b1;
                    end else begin
                        clk_count <= 16'd0;
                        if (bit_index < 3'd7) begin
                            bit_index <= bit_index + 1'b1;
                        end else begin
                            bit_index <= 3'd0;
                            state     <= S_STOP;
                        end
                    end
                end

                S_STOP: begin
                    tx <= 1'b1; // Bit Stop (mức 1)
                    if (clk_count < CLKS_PER_BIT - 1) begin
                        clk_count <= clk_count + 1'b1;
                    end else begin
                        clk_count <= 16'd0;
                        tx_busy   <= 1'b0;
                        tx_done   <= 1'b1;
                        state     <= S_IDLE;
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end
endmodule