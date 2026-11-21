module buffer #( //đọc lại buffer
    parameter DATA_WIDTH = 8,
    parameter MEM_DEPTH  = 16
)(
    input                         clk,
    input                         rst_n,

    input      [DATA_WIDTH-1:0]   data,
    input      [DATA_WIDTH-1:0]   i_timestamp,
    input                         event_pulse,
    input                         trigger_detect,

    output reg [DATA_WIDTH-1:0]   data_out,
    output reg [DATA_WIDTH-1:0]   o_timestamp,
    output reg                    valid,
    output reg                    o_done
);
    localparam PRE_DEPTH  = MEM_DEPTH / 2;
    localparam POST_DEPTH = MEM_DEPTH - PRE_DEPTH;
    localparam COUNT_WIDTH = $clog2(MEM_DEPTH + 1);

    // Trạng thái ghi
    localparam [1:0]
        C_PRE  = 2'b00,
        C_POST = 2'b01,
        C_WAIT = 2'b10;

    // Hai bank độc lập, mỗi mẫu gồm {timestamp, data}.
    reg [2*DATA_WIDTH-1:0] mem0 [0:MEM_DEPTH-1];
    reg [2*DATA_WIDTH-1:0] mem1 [0:MEM_DEPTH-1];

    // Quản lý bank và luồng ghi
    reg                   cap_bank;
    reg [1:0]             cap_state;
    reg [COUNT_WIDTH-1:0] pre_count;
    reg [COUNT_WIDTH-1:0] post_count;

    // Bank đã ghi xong và số mẫu cần đọc trong mỗi bank
    reg                   ready0;
    reg                   ready1;
    reg [COUNT_WIDTH-1:0] len0;
    reg [COUNT_WIDTH-1:0] len1;

    // Quản lý luồng đọc
    reg                   read_active;
    reg                   read_bank;
    reg [COUNT_WIDTH-1:0] read_addr;
    reg [COUNT_WIDTH-1:0] read_count;
    reg [COUNT_WIDTH-1:0] read_total;

    // Bank chỉ khả dụng nếu không chứa dữ liệu chờ đọc
    // và hiện không được reader sử dụng.
    wire bank0_free = !ready0 && !(read_active && (read_bank == 1'b0));
    wire bank1_free = !ready1 && !(read_active && (read_bank == 1'b1));

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cap_bank   <= 1'b0;
            cap_state  <= C_PRE;
            pre_count  <= 0;
            post_count <= 0;

            ready0 <= 1'b0;
            ready1 <= 1'b0;
            len0   <= 0;
            len1   <= 0;

            read_active <= 1'b0;
            read_bank   <= 1'b0;
            read_addr   <= 0;
            read_count  <= 0;
            read_total  <= 0;

            data_out    <= 0;
            o_timestamp <= 0;
            valid       <= 1'b0;
            o_done      <= 1'b0;
        end else begin
            valid  <= 1'b0;
            o_done <= 1'b0;

            // ------------------------------------------------
            // Ghi: PRE / POST
            // ------------------------------------------------
            case (cap_state)
                C_PRE: begin
                    if (event_pulse) begin
                        if (!cap_bank)
                            mem0[pre_count] <= {i_timestamp, data};
                        else
                            mem1[pre_count] <= {i_timestamp, data};

                        if (trigger_detect) begin
                            // Mẫu trigger được tính vào PRE.
                            pre_count  <= pre_count + 1'b1;
                            post_count <= 0;
                            cap_state  <= C_POST;
                        end else if (pre_count == PRE_DEPTH - 1) begin
                            // PRE đầy nhưng không có trigger: đưa bank đi đọc.
                            if (!cap_bank) begin
                                len0  <= PRE_DEPTH;
                                ready0 <= 1'b1;
                            end else begin
                                len1  <= PRE_DEPTH;
                                ready1 <= 1'b1;
                            end

                            cap_bank  <= ~cap_bank;
                            cap_state <= C_WAIT;
                        end else begin
                            pre_count <= pre_count + 1'b1;
                        end
                    end
                end

                C_POST: begin
                    if (event_pulse) begin
                        if (!cap_bank)
                            mem0[pre_count + post_count] <= {i_timestamp, data};
                        else
                            mem1[pre_count + post_count] <= {i_timestamp, data};

                        if (post_count == POST_DEPTH - 1) begin
                            // Đã ghi đủ POST. Tổng số mẫu gồm PRE thực tế và POST.
                            if (!cap_bank) begin
                                len0   <= pre_count + POST_DEPTH;
                                ready0 <= 1'b1;
                            end else begin
                                len1   <= pre_count + POST_DEPTH;
                                ready1 <= 1'b1;
                            end

                            cap_bank  <= ~cap_bank;
                            cap_state <= C_WAIT;
                        end else begin
                            post_count <= post_count + 1'b1;
                        end
                    end
                end

                C_WAIT: begin
                    // Chọn bank đang trống để bắt đầu lần ghi tiếp theo.
                    if (bank0_free) begin
                        cap_bank   <= 1'b0;
                        cap_state  <= C_PRE;
                        pre_count  <= 0;
                        post_count <= 0;
                    end else if (bank1_free) begin
                        cap_bank   <= 1'b1;
                        cap_state  <= C_PRE;
                        pre_count  <= 0;
                        post_count <= 0;
                    end
                end

                default: begin
                    cap_state <= C_WAIT;
                end
            endcase

            // ------------------------------------------------
            // Đọc: đọc một bank đã ghi xong trong lúc bank kia ghi
            // ------------------------------------------------
            if (!read_active) begin
                if (ready0) begin
                    read_active <= 1'b1;
                    read_bank   <= 1'b0;
                    read_addr   <= 0;
                    read_count  <= 0;
                    read_total  <= len0;
                    ready0      <= 1'b0;
                end else if (ready1) begin
                    read_active <= 1'b1;
                    read_bank   <= 1'b1;
                    read_addr   <= 0;
                    read_count  <= 0;
                    read_total  <= len1;
                    ready1      <= 1'b0;
                end
            end else begin
                if (!read_bank) begin
                    data_out    <= mem0[read_addr][DATA_WIDTH-1:0];
                    o_timestamp <= mem0[read_addr][2*DATA_WIDTH-1:DATA_WIDTH];
                end else begin
                    data_out    <= mem1[read_addr][DATA_WIDTH-1:0];
                    o_timestamp <= mem1[read_addr][2*DATA_WIDTH-1:DATA_WIDTH];
                end

                valid <= 1'b1;

                if (read_count == read_total - 1'b1) begin
                    read_active <= 1'b0;
                    read_addr   <= 0;
                    read_count  <= 0;
                    o_done      <= 1'b1;
                end else begin
                    read_addr  <= read_addr + 1'b1;
                    read_count <= read_count + 1'b1;
                end
            end
        end
    end

endmodule