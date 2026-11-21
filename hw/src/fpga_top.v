module fpga_top (
    input  wire       clk_27m,  // Nối với bộ clock mặc định 27 MHz trên board (Pin 45)
    input  wire       rst_n,    // Nút nhấn KEY1 (Pin 15)
    input  wire [7:0] data_in,  // 8 kênh đo tín hiệu logic (Pin 16..23)
    input  wire       rx,       // UART RX (Pin 28)
    output wire       tx        // UART TX (Pin 27)
);

    // Dây xung nhịp 50 MHz nội bộ sau khi nhân tần từ 27 MHz
    wire clk_50m;

    // 1. Bộ nhân tần từ thạch anh mặc định 27 MHz lên 50 MHz
    Gowin_PLLVR u_pll (
        .clkout (clk_50m),
        .clkin  (clk_27m)
    );

    // 2. Cấp xung 50 MHz trực tiếp vào cổng clk của logic_analyzer_top
    logic_analyzer_top #(
        .DATA_WIDTH   (8),
        .MEM_DEPTH    (8192),
        .CLKS_PER_BIT (434)
    ) u_logic_analyzer (
        .clk     (clk_50m),
        .rst_n   (rst_n),
        .data_in (data_in),
        .rx      (rx),
        .tx      (tx)
    );

endmodule