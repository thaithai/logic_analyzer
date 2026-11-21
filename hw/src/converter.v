module converter #(
    parameter DATA_WIDTH = 8
)(
    input  wire                  clk,
    input  wire                  rst_n,

    // Buffer interface
    input  wire                  buffer_tx_req,
    input  wire [2*DATA_WIDTH-1:0] buffer_data_in, // 16-bit: [15:8] timestamp, [7:0] data

    // UART interface
    input  wire                  tx_busy,
    output reg                   tx_start,
    output reg  [DATA_WIDTH-1:0] tx_data
);

    // Trạng thái FSM chuyển đổi 16-bit thành 2 byte UART
    localparam [1:0] 
        IDLE  = 2'b00,
        SEND1 = 2'b01,
        WAIT1 = 2'b10,
        SEND2 = 2'b11;

    reg [1:0] state;
    reg [2*DATA_WIDTH-1:0] saved_data;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state      <= IDLE;
            tx_start   <= 1'b0;
            tx_data    <= 8'd0;
            saved_data <= '0;
        end else begin
            tx_start <= 1'b0; // Mặc định tạo xung hẹp cho tx_start

            case (state)
                IDLE: begin
                    if (buffer_tx_req) begin
                        saved_data <= buffer_data_in;
                        state      <= SEND1;
                    end
                end

                SEND1: begin
                    if (!tx_busy) begin
                        tx_data  <= saved_data[2*DATA_WIDTH-1:DATA_WIDTH]; // Gửi byte cao (Timestamp)
                        tx_start <= 1'b1;
                        state    <= WAIT1;
                    end
                end

                WAIT1: begin
                    if (tx_busy) begin
                        state <= SEND2;
                    end
                end

                SEND2: begin
                    if (!tx_busy) begin
                        tx_data  <= saved_data[DATA_WIDTH-1:0];         // Gửi byte thấp (Data)
                        tx_start <= 1'b1;
                        state    <= IDLE;
                    end
                end

                default: state <= IDLE;
            endcase
        end
    end

endmodule