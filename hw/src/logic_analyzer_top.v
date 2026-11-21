module logic_analyzer_top #(
    parameter DATA_WIDTH   = 8,
    parameter MEM_DEPTH    = 8192,
    parameter CLKS_PER_BIT = 434   // 50MHz / 115200 baud
)(
    input  wire                  clk,
    input  wire                  rst_n,

    // Ngõ vào 8 kênh tín hiệu logic cần đo
    input  wire [DATA_WIDTH-1:0] data_in,

    // Giao tiếp UART với PC
    input  wire                  rx,
    output wire                  tx
);

    // 1. Tín hiệu đồng bộ ngõ vào
    wire [DATA_WIDTH-1:0] data_sync;

    // 2. Tín hiệu điều khiển từ Controller
    wire                        enable;
    wire [1:0]                  trigger_condition;
    wire [2:0]                  channel;
    wire [15:0]                 scale;
    wire [$clog2(MEM_DEPTH):0]  pre_capture;
    wire [$clog2(MEM_DEPTH):0]  post_capture;

    // 3. Tín hiệu Clock chia tần & Timestamp
    wire       clk_scale;
    wire [7:0] timestamp;

    // 4. Tín hiệu Trigger -> Buffer
    wire event_pulse;
    wire trigger_detect;

    // 5. Tín hiệu bắt tay (Handshake) Buffer <-> Converter
    wire                    buffer_ready;
    wire                    buffer_tx_req;
    wire [2*DATA_WIDTH-1:0] buffer_data_out;

    // 6. Tín hiệu giao tiếp UART (TX & RX)
    wire       tx_start;
    wire [7:0] tx_data;
    wire       tx_busy;
    wire       rx_dv;
    wire [7:0] rx_data;

    // =========================================================================
    // 1. Khối đồng bộ hóa ngõ vào (Chống Metastability)
    // =========================================================================
    input_sync #(
        .DATA_WIDTH(DATA_WIDTH)
    ) u_input_sync (
        .clk     (clk),
        .rst_n   (rst_n),
        .in      (data_in),
        .in_sync (data_sync)
    );

    // =========================================================================
    // 2. Khối giải mã lệnh cấu hình từ PC
    // =========================================================================
    controller #(
        .MEM_DEPTH(MEM_DEPTH)
    ) u_controller (
        .clk               (clk),
        .rst_n             (rst_n),
        .pc_command        (rx_data),
        .cmd_valid         (rx_dv),
        .enable            (enable),
        .trigger_condition (trigger_condition),
        .channel           (channel),
        .scale             (scale),
        .pre_capture       (pre_capture),
        .post_capture      (post_capture)
    );

    // =========================================================================
    // 3. Khối chia tần số lấy mẫu & bộ đếm Timestamp
    // =========================================================================
    clock_divider u_clock_divider (
        .clk       (clk),
        .rst_n     (rst_n),
        .scale     (scale),
        .clk_scale (clk_scale)
    );

    timestamp_counter u_timestamp_counter (
        .clk       (clk),
        .rst_n     (rst_n),
        .clk_scale (clk_scale),
        .timestamp (timestamp)
    );

    // =========================================================================
    // 4. Khối phát hiện sự kiện thay đổi & điều kiện Trigger
    // =========================================================================
    trigger #(
        .DATA_WIDTH(DATA_WIDTH)
    ) u_trigger (
        .clk               (clk),
        .rst_n             (rst_n),
        .enable            (enable),
        .trigger_condition (trigger_condition),
        .channel           (channel),
        .data              (data_sync),
        .clk_scale         (clk_scale),
        .event_pulse       (event_pulse),
        .trigger_detect    (trigger_detect)
    );

    // =========================================================================
    // 5. Khối bộ nhớ đệm Pre/Post Trigger (Gọi module bram bên trong)
    // =========================================================================
    buffer #(
        .DATA_WIDTH(DATA_WIDTH),
        .MEM_DEPTH (MEM_DEPTH)
    ) u_buffer (
        .clk            (clk),
        .rst_n          (rst_n),
        .pre_capture    (pre_capture),
        .post_capture   (post_capture),
        .data           (data_sync),
        .timestamp      (timestamp),
        .event_pulse    (event_pulse),
        .trigger_detect (trigger_detect),
        .tx_ready       (buffer_ready),
        .tx_req         (buffer_tx_req),
        .data_out       (buffer_data_out)
    );

    // =========================================================================
    // 6. Khối đóng gói 16-bit {Timestamp, Data} thành 2 byte UART
    // =========================================================================
    converter #(
        .DATA_WIDTH(DATA_WIDTH)
    ) u_converter (
        .clk            (clk),
        .rst_n          (rst_n),
        .buffer_tx_req  (buffer_tx_req),
        .buffer_data_in (buffer_data_out),
        .buffer_ready   (buffer_ready),
        .tx_busy        (tx_busy),
        .tx_start       (tx_start),
        .tx_data        (tx_data)
    );

    // =========================================================================
    // 7. Khối truyền nhận nối tiếp UART (TX & RX)
    // =========================================================================
    uart #(
        .CLKS_PER_BIT(CLKS_PER_BIT)
    ) u_uart (
        .clk      (clk),
        .rst_n    (rst_n),
        .tx_start (tx_start),
        .tx_data  (tx_data),
        .tx_busy  (tx_busy),
        .tx_done  (),
        .tx       (tx),
        .rx       (rx),
        .rx_dv    (rx_dv),
        .rx_data  (rx_data)
    );

endmodule