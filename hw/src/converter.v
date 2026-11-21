module converter #(
    parameter DATA_WIDTH = 8
)(
    input  wire                    clk,
    input  wire                    rst_n,

    input  wire                    buffer_tx_req,
    input  wire [2*DATA_WIDTH-1:0] buffer_data_in,
    output wire                    buffer_ready,

    input  wire                    tx_busy,
    output reg                     tx_start,
    output reg  [DATA_WIDTH-1:0]   tx_data
);

    localparam [2:0]
        IDLE      = 3'd0,
        LOAD_BYTE = 3'd1,
        WAIT_BUSY = 3'd2,
        WAIT_DONE = 3'd3;

    reg [2:0] state;
    reg [2:0] byte_idx;
    reg [DATA_WIDTH-1:0] sample_byte;

    assign buffer_ready = (state == IDLE);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state       <= IDLE;
            byte_idx    <= 3'd0;
            tx_start    <= 1'b0;
            tx_data     <= {DATA_WIDTH{1'b0}};
            sample_byte <= {DATA_WIDTH{1'b0}};
        end else begin
            tx_start <= 1'b0;

            case (state)
                IDLE: begin
                    if (buffer_tx_req) begin
                        sample_byte <= buffer_data_in[DATA_WIDTH-1:0]; // Lấy 8-bit dữ liệu kênh
                        byte_idx    <= 3'd0;
                        state       <= LOAD_BYTE;
                    end
                end

                LOAD_BYTE: begin
                    case (byte_idx)
                        3'd0: tx_data <= 8'h5A;        // DEVICE_SYNC[0]
                        3'd1: tx_data <= 8'hA5;        // DEVICE_SYNC[1]
                        3'd2: tx_data <= 8'h10;        // PKT_DATA
                        3'd3: tx_data <= 8'h01;        // LEN_LO = 1
                        3'd4: tx_data <= 8'h00;        // LEN_HI = 0
                        3'd5: tx_data <= sample_byte;  // Payload 1 byte mẫu
                        default: tx_data <= 8'h00;
                    endcase
                    tx_start <= 1'b1;
                    state    <= WAIT_BUSY;
                end

                WAIT_BUSY: begin
                    if (tx_busy) state <= WAIT_DONE;
                end

                WAIT_DONE: begin
                    if (!tx_busy) begin
                        if (byte_idx == 3'd5) begin
                            state <= IDLE;
                        end else begin
                            byte_idx <= byte_idx + 1'b1;
                            state    <= LOAD_BYTE;
                        end
                    end
                end

                default: state <= IDLE;
            endcase
        end
    end

endmodule