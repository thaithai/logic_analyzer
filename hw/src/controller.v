module controller #(
    parameter MEM_DEPTH = 8192
)(
    input  wire        clk,
    input  wire        rst_n,

    input  wire [7:0]  pc_command,
    input  wire        cmd_valid,

    output reg         enable,
    output reg  [1:0]  trigger_condition, // 00: Mức 0, 01: Mức 1, 10: Cạnh lên, 11: Cạnh xuống
    output reg  [2:0]  channel,
    output reg  [15:0] scale,

    output reg  [$clog2(MEM_DEPTH):0] pre_capture,
    output reg  [$clog2(MEM_DEPTH):0] post_capture
);

    localparam [2:0]
        ST_SYNC1   = 3'd0,
        ST_SYNC2   = 3'd1,
        ST_CMD     = 3'd2,
        ST_CFG_PAY = 3'd3;

    reg [2:0]  rx_state;
    reg [3:0]  byte_idx;
    reg [31:0] sample_rate_buf;
    reg [15:0] pre_buf;
    reg [15:0] post_buf;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_state          <= ST_SYNC1;
            byte_idx          <= 4'd0;
            sample_rate_buf   <= 32'd1_000_000;
            pre_buf           <= 16'd128;
            post_buf          <= 16'd128;

            enable            <= 1'b0;
            trigger_condition <= 2'b10; // Cạnh lên
            channel           <= 3'd0;
            scale             <= 16'd50;
            pre_capture       <= 128;
            post_capture      <= 128;
        end else if (cmd_valid) begin
            case (rx_state)
                ST_SYNC1: begin
                    if (pc_command == 8'hA5) rx_state <= ST_SYNC2;
                end

                ST_SYNC2: begin
                    if (pc_command == 8'h5A) rx_state <= ST_CMD;
                    else if (pc_command == 8'hA5) rx_state <= ST_SYNC2;
                    else rx_state <= ST_SYNC1;
                end

                ST_CMD: begin
                    case (pc_command)
                        8'h01: begin // CMD_CONFIG (10 byte payload tiếp theo)
                            byte_idx <= 4'd0;
                            rx_state <= ST_CFG_PAY;
                        end
                        8'h02: begin // CMD_RUN
                            enable   <= 1'b1;
                            rx_state <= ST_SYNC1;
                        end
                        8'h03: begin // CMD_STOP
                            enable   <= 1'b0;
                            rx_state <= ST_SYNC1;
                        end
                        default: rx_state <= ST_SYNC1;
                    endcase
                end

                ST_CFG_PAY: begin
                    case (byte_idx)
                        4'd0: sample_rate_buf[7:0]   <= pc_command;
                        4'd1: sample_rate_buf[15:8]  <= pc_command;
                        4'd2: sample_rate_buf[23:16] <= pc_command;
                        4'd3: begin
                            sample_rate_buf[31:24] <= pc_command;
                            case ({pc_command, sample_rate_buf[23:0]})
                                32'd100_000:   scale <= 16'd499;
                                32'd500_000:   scale <= 16'd99;
                                32'd1_000_000: scale <= 16'd49;
                                32'd5_000_000: scale <= 16'd9;
                                32'd10_000_000: scale <= 16'd4;
                                32'd25_000_000: scale <= 16'd1;
                                default:       scale <= 16'd0; // 50 MS/s
                            endcase
                        end
                        4'd4: channel <= pc_command[2:0];
                        4'd5: begin
                            // Ánh xạ mã trigger từ LA_gui.py sang mã của trigger.v
                            case (pc_command)
                                8'd1:    trigger_condition <= 2'b10; // Cạnh lên
                                8'd2:    trigger_condition <= 2'b11; // Cạnh xuống
                                8'd3:    trigger_condition <= 2'b01; // Mức HIGH
                                8'd4:    trigger_condition <= 2'b00; // Mức LOW
                                default: trigger_condition <= 2'b01; // Không trigger -> kích ngay khi bit=1
                            endcase
                        end
                        4'd6: pre_buf[7:0]   <= pc_command;
                        4'd7: begin
                            pre_buf[15:8] <= pc_command;
                            pre_capture   <= {pc_command, pre_buf[7:0]};
                        end
                        4'd8: post_buf[7:0]  <= pc_command;
                        4'd9: begin
                            post_buf[15:8] <= pc_command;
                            post_capture   <= {pc_command, post_buf[7:0]};
                            rx_state       <= ST_SYNC1;
                        end
                    endcase
                    byte_idx <= byte_idx + 1'b1;
                end

                default: rx_state <= ST_SYNC1;
            endcase
        end
    end

endmodule