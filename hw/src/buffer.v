module buffer #( // cần bổ sung thêm các signal và xem lại logic của ping pong buffer
    parameter DATA_WIDTH = 8, 
    parameter MEM_DEPTH  = 16
)(
    input                       clk,
    input                       rst_n,
    input [DATA_WIDTH-1:0]      in_sync,
    input                       trigger,
    input clk_scale,
    input [DATA_WIDTH-1:0]  timestamp,
    
    output reg  [DATA_WIDTH-1:0]    data_out,
    output reg  [DATA_WIDTH-1:0]    timestamp,
    output reg                      valid     // Bổ sung tên và kiểu dữ liệu cho cổng output
);

    // Tính toán độ rộng địa chỉ dựa trên MEM_DEPTH (ví dụ: MEM_DEPTH = 16 -> ADDR_WIDTH = 4)
    localparam ADDR_WIDTH = $clog2(MEM_DEPTH);

    // Khai báo bộ nhớ RAM
    reg [2*DATA_WIDTH-1:0] mem [0:MEM_DEPTH-1];

    // Các trạng thái của FSM
    localparam [1:0] 
        S_RECORD  = 2'b00,
        S_TRIGGER = 2'b01,
        S_READOUT = 2'b10;

    reg [1:0] state, next_state;

    reg [ADDR_WIDTH-1:0] wr_ptr;
    reg [ADDR_WIDTH-1:0] rd_ptr;
    reg [3:0]            post_counter; // Đếm số mẫu ghi thêm sau trigger

    // 1. Khối ghi dữ liệu liên tục (Circular Buffer)
    always @(posedge clk) begin
        if (state == S_RECORD || state == S_TRIGGER) begin
            mem[wr_ptr] <= in_sync;
        end
    end

    // 2. Máy trạng thái điều khiển (FSM Sequential)
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= S_RECORD;
            wr_ptr       <= {ADDR_WIDTH{1'b0}};
            rd_ptr       <= {ADDR_WIDTH{1'b0}};
            post_counter <= 4'd0;
        end else begin
            state <= next_state;

            case (state)
                S_RECORD: begin
                    wr_ptr <= wr_ptr + 1'b1;
                    if (trigger) begin
                        post_counter <= 4'd0;
                    end
                end

                S_TRIGGER: begin
                    wr_ptr <= wr_ptr + 1'b1;
                    if (post_counter < (MEM_DEPTH / 2)) begin
                        post_counter <= post_counter + 1'b1;
                    end else begin
                        // Đặt con trỏ đọc bắt đầu từ vị trí cũ nhất trong buffer vòng
                        rd_ptr <= wr_ptr + 1'b1; 
                    end
                end

                S_READOUT: begin
                    rd_ptr <= rd_ptr + 1'b1;
                end
            endcase
        end
    end

    // 3. Logic chuyển đổi trạng thái (FSM Combinational)
    always @* begin
        next_state = state;
        case (state)
            S_RECORD: begin
                if (trigger) next_state = S_TRIGGER;
            end
            S_TRIGGER: begin
                if (post_counter >= (MEM_DEPTH / 2)) next_state = S_READOUT;
            end
            S_READOUT: begin
                // Đọc đủ 1 vòng bộ nhớ thì quay lại ghi tiếp
                if (rd_ptr == wr_ptr) next_state = S_RECORD;
            end
        endcase
    end

    // 4. Khối xuất dữ liệu ra ngoài
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            data_out <= {DATA_WIDTH{1'b0}};
            valid    <= 1'b0;
        end else if (state == S_READOUT) begin
            data_out <= mem[rd_ptr];
            valid    <= 1'b1;
        end else begin
            valid    <= 1'b0;
        end
    end

endmodule