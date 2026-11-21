module uart #(
    parameter CLKS_PER_BIT = 434
)(
    input  wire       clk,
    input  wire       rst_n,

    // UART TX interface
    input  wire       tx_start,
    input  wire [7:0] tx_data,
    output wire       tx_busy,
    output wire       tx_done,
    output wire       tx,

    // UART RX interface
    input  wire       rx,
    output wire       rx_dv,
    output wire [7:0] rx_data
);

    uart_tx #(
        .CLKS_PER_BIT(CLKS_PER_BIT)
    ) u_uart_tx (
        .clk        (clk),
        .rst_n      (rst_n),

        .tx_start   (tx_start),
        .tx_data    (tx_data),

        .tx_busy    (tx_busy),
        .tx         (tx),
        .tx_done    (tx_done)
    );
    
    uart_rx #(
        .CLKS_PER_BIT(CLKS_PER_BIT)
    ) u_uart_rx (
        .clk        (clk),
        .rst_n      (rst_n),

        .rx         (rx),

        .rx_dv      (rx_dv),
        .rx_data    (rx_data)
    );

endmodule