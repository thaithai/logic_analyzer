module logic_analyzer_top #(
    parameter DATA_WIDTH = 8, 
    parameter MEM_DEPTH = 8192
)(
    input  wire                  clk,       // Clock gốc từ PLL (VD: 50MHz)
    input  wire                  rst_n,     // Reset tích cực mức thấp
    input  wire [DATA_WIDTH-1:0] data_in,   // Dữ liệu đầu vào cần phân tích (8 kênh)
    input  wire                  rx,        // Chân nhận UART từ PC
    output wire                  tx         // Chân truyền UART lên PC
);

    // ==========================================
    // 1. Tín hiệu kết nối nội bộ (Internal Wires)
    // ==========================================
    
    // Tín hiệu UART RX -> Controller
    wire       cmd_valid;
    wire [7:0] cmd_data;

    // Tín hiệu từ Controller cấu hình hệ thống
    wire       enable;
    wire [1:0] trigger_condition;
    wire [2:0] channel;
    wire [15:0] scale;
    wire [$clog2(MEM_DEPTH)-1:0] pre_capture;
    wire [$clog2(MEM_DEPTH)-1:0] post_capture;

    // Tín hiệu đồng bộ và tạo xung
    wire                  clk_scale;
    wire [DATA_WIDTH-1:0] data_sync;
    wire [7:0]            timestamp;

    // Tín hiệu Trigger
    wire trigger_detect;
    wire event_pulse;

    // Tín hiệu truyền dữ liệu (Buffer -> Converter -> UART TX)
    wire [15:0] buffer_data_out; // Gồm 8-bit data + 8-bit timestamp
    wire        buffer_tx_req;   // Yêu cầu truyền từ buffer
    wire        tx_start;
    wire [7:0]  tx_data;
    wire        tx_busy;
    wire        tx_done;

    // ==========================================
    // 2. Instantiate các module con
    // ==========================================

    // Khối đồng bộ tín hiệu đầu vào
    input_sync #(
        .DATA_WIDTH(DATA_WIDTH)
    ) input_sync_inst (
        .clk(clk),
        .rst_n(rst_n),
        .data_in(data_in),
        .data_out(data_sync)
    );

    // Khối chia xung nhịp lấy mẫu
    clock_divider clock_divider_inst (
        .clk(clk),
        .rst_n(rst_n),
        .scale(scale),
        .clk_scale(clk_scale)
    );

    // Khối đếm thời gian
    timestamp_counter timestamp_counter_inst (
        .clk(clk),
        .rst_n(rst_n),
        .clk_scale(clk_scale),
        .timestamp(timestamp)
    );

    // Khối phát hiện điều kiện Trigger
    trigger trigger_inst (
        .clk(clk),
        .rst_n(rst_n),
        .enable(enable),
        .trigger_condition(trigger_condition),
        .channel(channel),
        .data(data_sync),
        .clk_scale(clk_scale),
        .event_pulse(event_pulse),
        .trigger_detect(trigger_detect)
    );

    // Khối giải mã lệnh từ PC
    controller #(
        .MEM_DEPTH(MEM_DEPTH)
    ) controller_inst (
        .clk(clk),
        .rst_n(rst_n),
        .pc_command(cmd_data), 
        .cmd_valid(cmd_valid), 
        .enable(enable),
        .trigger_condition(trigger_condition),
        .channel(channel),
        .scale(scale),
        .pre_capture(pre_capture),
        .post_capture(post_capture)
    );

    // Khối lưu trữ dữ liệu (BRAM/Circular Buffer)
    buffer #(
        .MEM_DEPTH(MEM_DEPTH)
    ) buffer_inst (
        .clk(clk),
        .rst_n(rst_n),
        .data(data_sync),
        .timestamp(timestamp),
        .event_pulse(event_pulse),
        .trigger_detect(trigger_detect),
        .pre_capture(pre_capture),
        .post_capture(post_capture),
        .tx_req(buffer_tx_req),
        .data_out(buffer_data_out)
    );

    // Khối chuyển đổi dữ liệu 16-bit thành 2 byte 8-bit để gửi UART
    converter #(
        .DATA_WIDTH(DATA_WIDTH)
    ) converter_inst (
        .clk(clk),
        .rst_n(rst_n),
        .buffer_tx_req(buffer_tx_req),
        .buffer_data_in(buffer_data_out),
        .tx_busy(tx_busy),
        .tx_start(tx_start),
        .tx_data(tx_data)
    );

    // Khối giao tiếp UART
    uart #(
        .CLKS_PER_BIT(434) // ~115200 baudrate với 50MHz clock
    ) uart_inst (
        .clk(clk),
        .rst_n(rst_n),
        // Giao diện truyền (TX)
        .tx_start(tx_start),
        .tx_data(tx_data),
        .tx(tx),
        .tx_busy(tx_busy),
        .tx_done(tx_done),
        // Giao diện nhận (RX)
        .rx(rx),
        .rx_dv(cmd_valid),
        .rx_data(cmd_data)
    );

endmodule