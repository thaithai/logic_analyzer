module controller #(
    parameter MEM_DEPTH = 256
)(
    input  wire        clk,
    input  wire        rst_n,

    // UART RX interface (Gói lệnh 1 byte từ PC)
    input  wire [7:0]  pc_command,
    input  wire        cmd_valid,

    // Trigger configuration
    output reg         enable,
    output reg  [1:0]  trigger_condition, // 00: Cạnh lên, 01: Cạnh xuống, 10: Mức 0, 11: Mức 1
    output reg  [2:0]  channel,           // Kênh trigger (có thể mở rộng thêm nếu cần)

    // Sample clock configuration
    output reg  [15:0] scale,             // Hệ số chia tần cho clock_divider

    // Buffer configuration
    output reg  [$clog2(MEM_DEPTH):0] pre_capture,
    output reg  [$clog2(MEM_DEPTH):0] post_capture
);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            enable            <= 1'b0;
            trigger_condition <= 2'b00; // Mặc định: Cạnh lên
            channel           <= 3'd0;   // Mặc định: Kênh 0
            scale             <= 16'd50; // Mặc định: 1 MHz (50MHz / 50)
            
            pre_capture       <= MEM_DEPTH / 2;
            post_capture      <= MEM_DEPTH - (MEM_DEPTH / 2);
        end 
        else begin

            if (cmd_valid) begin
                // 1. [7:5] Giải mã tần số lấy mẫu (Scale)
                case (pc_command[7:5])
                    3'b000: scale <= 16'd500; // 0.1 MHz
                    3'b001: scale <= 16'd100; // 0.5 MHz
                    3'b010: scale <= 16'd50;  // 1.0 MHz
                    3'b011: scale <= 16'd10;  // 5.0 MHz
                    3'b100: scale <= 16'd5;   // 10.0 MHz
                    3'b101: scale <= 16'd2;   // 25.0 MHz
                    3'b110: scale <= 16'd1;   // 50.0 MHz
                    3'b111: scale <= 16'd1;   // 100.0 MHz
                    default: scale <= 16'd50;
                endcase

                // 2. [4] Trạng thái Enable Trigger
                enable <= pc_command[4];

                // 3. [3:2] Phân chia Pre-trigger và Post-trigger dựa trên MEM_DEPTH
                case (pc_command[3:2])
                    2'b00: begin // 25% Pre, 75% Post
                        pre_capture  <= MEM_DEPTH / 4;
                        post_capture <= MEM_DEPTH - (MEM_DEPTH / 4);
                    end
                    2'b01: begin // 50% Pre, 50% Post
                        pre_capture  <= MEM_DEPTH / 2;
                        post_capture <= MEM_DEPTH - (MEM_DEPTH / 2);
                    end
                    2'b10: begin // 75% Pre, 25% Post
                        pre_capture  <= (MEM_DEPTH * 3) / 4;
                        post_capture <= MEM_DEPTH - ((MEM_DEPTH * 3) / 4);
                    end
                    2'b11: begin // 90% Pre, 10% Post
                        pre_capture  <= (MEM_DEPTH * 9) / 10;
                        post_capture <= MEM_DEPTH - ((MEM_DEPTH * 9) / 10);
                    end
                endcase

                // 4. [1:0] 4 trạng thái trigger (Cạnh lên, Cạnh xuống, Mức 0, Mức 1)
                trigger_condition <= pc_command[1:0];
            end
        end
    end

endmodule