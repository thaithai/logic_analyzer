module data_converter #( // cần bổ sung và xem lại khối convert
    parameter DATA_WIDTH   = 8,
    parameter CLKS_PER_BIT = 434 // Tính toán dựa trên tần số clock hệ thống và baudrate
                                 // Ví dụ: Clock 50MHz, Baudrate 115200 -> 50,000,000 / 115200 ≈ 434
)(
    input  wire                   clk, 
    input  wire                   rst_n,
    
    // --- Giao tiếp nhận dữ liệu từ Buffer ---
    input  wire [DATA_WIDTH-1:0]  din,       // Dữ liệu nhận từ buffer
    input  wire                   valid,     // Báo dữ liệu từ buffer hợp lệ
    output reg                    ready,     // Báo converter sẵn sàng nhận byte tiếp theo
    
    // --- Giao tiếp phần cứng UART TX ra chân FPGA ---
    output reg                    tx         // Chân truyền dữ liệu nối tiếp UART
);

    // --- Định nghĩa trạng thái Máy trạng thái (FSM) UART TX ---
    localparam [2:0] 
        S_IDLE  = 3'b000,
        S_START = 3'b001,
        S_DATA  = 3'b010,
        S_STOP  = 3'b011,
        S_CLEAN = 3'b100;

    reg [2:0]  state;
    reg [15:0] clk_count;
    reg [2:0]  bit_index;
    reg [7:0]  tx_data_reg;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state       <= S_IDLE;
            tx          <= 1'b1; // Đường truyền UART mặc định ở mức cao (Idle)
            ready       <= 1'b0;
            clk_count   <= 16'd0;
            bit_index   <= 3'd0;
            tx_data_reg <= 8'd0;
        end else begin
            case (state)
                S_IDLE: begin
                    tx    <= 1'b1;
                    ready <= 1'b1; // Sẵn sàng nhận dữ liệu mới
                    
                    if (valid && ready) begin
                        tx_data_reg <= din;
                        ready       <= 1'b0; // Hạ ready để khóa buffer, bắt đầu gửi UART
                        state       <= S_START;
                        clk_count   <= 16'd0;
                    end
                end

                S_START: begin
                    tx <= 1'b0; // Phát Bit Start (mức thấp 0)
                    if (clk_count < CLKS_PER_BIT - 1) begin
                        clk_count <= clk_count + 1'b1;
                    end else begin
                        clk_count <= 16'd0;
                        state     <= S_DATA;
                        bit_index <= 3'd0;
                    end
                end

                S_DATA: begin
                    tx <= tx_data_reg[bit_index]; // Truyền lần lượt 8 bit từ LSB đến MSB
                    if (clk_count < CLKS_PER_BIT - 1) begin
                        clk_count <= clk_count + 1'b1;
                    end else begin
                        clk_count <= 16'd0;
                        if (bit_index < 3'd7) begin
                            bit_index <= bit_index + 1'b1;
                        end else begin
                            state <= S_STOP;
                        end
                    end
                end

                S_STOP: begin
                    tx <= 1'b1; // Phát Bit Stop (mức cao 1)
                    if (clk_count < CLKS_PER_BIT - 1) begin
                        clk_count <= clk_count + 1'b1;
                    end else begin
                        clk_count <= 16'd0;
                        state     <= S_CLEAN;
                    end
                end

                S_CLEAN: begin
                    ready <= 1'b1; // Báo cho buffer biết đã truyền xong 1 byte, có thể gửi byte tiếp theo
                    state <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule