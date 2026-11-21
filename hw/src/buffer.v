module buffer #(
    parameter DATA_WIDTH = 8,
    parameter MEM_DEPTH  = 8192
)(
    input                         clk,
    input                         rst_n,

    // Cấu hình từ controller động
    input      [$clog2(MEM_DEPTH):0] pre_capture,
    input      [$clog2(MEM_DEPTH):0] post_capture,

    // Dữ liệu đầu vào
    input      [DATA_WIDTH-1:0]   data,
    input      [7:0]              timestamp,
    input                         event_pulse,
    input                         trigger_detect,

    // Giao tiếp với converter (Handshake)
    input                         tx_ready,
    output reg                    tx_req,
    output reg [2*DATA_WIDTH-1:0] data_out
);

    localparam ADDR_WIDTH = $clog2(MEM_DEPTH);
    localparam CNT_WIDTH  = $clog2(MEM_DEPTH + 1);

    localparam [2:0]
        S_PRE      = 3'd0, // Ghi vòng tròn liên tục trước khi có trigger
        S_POST     = 3'd1, // Ghi nốt số mẫu sau khi phát hiện trigger
        S_READ_MEM = 3'd2, // Cấp địa chỉ đọc cho BRAM
        S_WAIT_MEM = 3'd3, // Chờ 1 chu kỳ trễ đọc của BRAM
        S_SEND_REQ = 3'd4, // Đẩy dữ liệu sang converter khi tx_ready = 1
        S_HOLD_REQ = 3'd5; // Hạ tx_req và kiểm tra hoàn tất gói tin

    // Tín hiệu kết nối với module bram
    reg                     mem_we;
    reg  [ADDR_WIDTH-1:0]   mem_waddr;
    reg  [2*DATA_WIDTH-1:0] mem_wdata;
    reg  [ADDR_WIDTH-1:0]   mem_raddr;
    wire [2*DATA_WIDTH-1:0] mem_rdata;

    bram #(
        .DATA_WIDTH(2 * DATA_WIDTH),
        .MEM_DEPTH (MEM_DEPTH)
    ) u_bram (
        .clk   (clk),
        .we    (mem_we),
        .waddr (mem_waddr),
        .wdata (mem_wdata),
        .raddr (mem_raddr),
        .rdata (mem_rdata)
    );

    // Thanh ghi quản lý bộ đệm vòng (Ring Buffer) & Handshake UART
    reg [2:0]            state;
    reg                  armed;
    reg [ADDR_WIDTH-1:0] wr_ptr;
    reg [ADDR_WIDTH-1:0] rd_ptr;
    reg [CNT_WIDTH-1:0]  pre_count;
    reg [CNT_WIDTH-1:0]  post_count;
    reg [CNT_WIDTH-1:0]  dump_total;
    reg [CNT_WIDTH-1:0]  dump_count;

    wire [ADDR_WIDTH-1:0] next_wr_ptr   = (wr_ptr == MEM_DEPTH - 1) ? {ADDR_WIDTH{1'b0}} : (wr_ptr + 1'b1);
    wire [CNT_WIDTH-1:0]  eff_pre_count = (event_pulse && (pre_count < pre_capture)) ? (pre_count + 1'b1) : pre_count;
    wire [ADDR_WIDTH-1:0] eff_wr_ptr    = event_pulse ? next_wr_ptr : wr_ptr;

    wire [CNT_WIDTH:0] start_addr_ext = ({1'b0, eff_wr_ptr} >= {1'b0, eff_pre_count}) ?
                                        ({1'b0, eff_wr_ptr} - {1'b0, eff_pre_count}) :
                                        (MEM_DEPTH + {1'b0, eff_wr_ptr} - {1'b0, eff_pre_count});

    wire [CNT_WIDTH:0]   total_samples_ext = {1'b0, eff_pre_count} + {1'b0, post_capture};
    wire [CNT_WIDTH-1:0] total_samples     = (total_samples_ext > MEM_DEPTH) ? MEM_DEPTH[CNT_WIDTH-1:0] : total_samples_ext[CNT_WIDTH-1:0];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state      <= S_PRE;
            armed      <= 1'b1;
            wr_ptr     <= {ADDR_WIDTH{1'b0}};
            rd_ptr     <= {ADDR_WIDTH{1'b0}};
            pre_count  <= {CNT_WIDTH{1'b0}};
            post_count <= {CNT_WIDTH{1'b0}};
            dump_total <= {CNT_WIDTH{1'b0}};
            dump_count <= {CNT_WIDTH{1'b0}};

            mem_we     <= 1'b0;
            mem_waddr  <= {ADDR_WIDTH{1'b0}};
            mem_wdata  <= {(2*DATA_WIDTH){1'b0}};
            mem_raddr  <= {ADDR_WIDTH{1'b0}};

            tx_req     <= 1'b0;
            data_out   <= {(2*DATA_WIDTH){1'b0}};
        end else begin
            mem_we <= 1'b0;
            tx_req <= 1'b0;

            if (!trigger_detect) begin
                armed <= 1'b1;
            end

            case (state)
                S_PRE: begin
                    if (event_pulse) begin
                        mem_we    <= 1'b1;
                        mem_waddr <= wr_ptr;
                        mem_wdata <= {timestamp, data};
                        wr_ptr    <= next_wr_ptr;

                        if (pre_count < pre_capture) begin
                            pre_count <= pre_count + 1'b1;
                        end
                    end

                    if (trigger_detect && armed) begin
                        armed      <= 1'b0;
                        rd_ptr     <= start_addr_ext[ADDR_WIDTH-1:0];
                        dump_total <= total_samples;
                        dump_count <= {CNT_WIDTH{1'b0}};
                        post_count <= {CNT_WIDTH{1'b0}};

                        if (post_capture == {CNT_WIDTH{1'b0}}) begin
                            if (total_samples == {CNT_WIDTH{1'b0}}) begin
                                pre_count <= {CNT_WIDTH{1'b0}};
                                state     <= S_PRE;
                            end else begin
                                state     <= S_READ_MEM;
                            end
                        end else begin
                            state <= S_POST;
                        end
                    end
                end

                S_POST: begin
                    if (event_pulse) begin
                        mem_we    <= 1'b1;
                        mem_waddr <= wr_ptr;
                        mem_wdata <= {timestamp, data};
                        wr_ptr    <= next_wr_ptr;

                        if (post_count + 1'b1 >= post_capture) begin
                            state <= S_READ_MEM;
                        end else begin
                            post_count <= post_count + 1'b1;
                        end
                    end
                end

                S_READ_MEM: begin
                    mem_raddr <= rd_ptr;
                    state     <= S_WAIT_MEM;
                end

                S_WAIT_MEM: begin
                    state <= S_SEND_REQ;
                end

                S_SEND_REQ: begin
                    if (tx_ready) begin
                        data_out   <= mem_rdata;
                        tx_req     <= 1'b1;
                        rd_ptr     <= (rd_ptr == MEM_DEPTH - 1) ? {ADDR_WIDTH{1'b0}} : (rd_ptr + 1'b1);
                        dump_count <= dump_count + 1'b1;
                        state      <= S_HOLD_REQ;
                    end
                end

                S_HOLD_REQ: begin
                    if (dump_count >= dump_total) begin
                        pre_count  <= {CNT_WIDTH{1'b0}};
                        post_count <= {CNT_WIDTH{1'b0}};
                        state      <= S_PRE;
                    end else begin
                        state <= S_READ_MEM;
                    end
                end

                default: state <= S_PRE;
            endcase
        end
    end

endmodule