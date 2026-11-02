module uart #(
    parameter   CNT_1S = 32'd49_999_999 // 50MHz 下约 1 秒发送一帧
) (
    input               clk_50M,
    input               rst_n,

    input               fpga_rx,
    output              fpga_tx,

    input       [15:0]  data_in,
    input       [31:0]  encoder_in,

    output reg  [15:0]  data_out
);

parameter UART_BPS     = 21'd9600;
parameter CLK_FREQ     = 26'd50_000_000;
parameter SEN_CNT_MAX  = 20'd999_999;   // 约 20ms 发一个字节
parameter SEN_BYTE_CNT = 4'd9;          // 实际发送 10 个字节：0~9

wire        pi_flag;
wire [7:0]  pi_data;
wire        pi_done;

//----------------------------------------------------
// UART 发送模块例化
//----------------------------------------------------
uart_tx #(
    .UART_BPS   (UART_BPS),
    .CLK_FREQ   (CLK_FREQ)
) uart_tx_inst (
    .sys_clk    (clk_50M),
    .sys_rst_n  (rst_n),
    .pi_data    (pi_data),
    .pi_flag    (pi_flag),
    .tx         (fpga_tx),
    .pi_done    (pi_done)
);

//----------------------------------------------------
// 周期计数器：每隔 CNT_1S 产生一个 frame_start 脉冲
// 上电复位后自动开始，不需要按键
//----------------------------------------------------
reg [31:0] period_cnt;
reg        frame_start;

always @(posedge clk_50M or negedge rst_n) begin
    if (!rst_n) begin
        period_cnt  <= 32'd0;
        frame_start <= 1'b0;
    end else begin
        if (period_cnt == CNT_1S - 1) begin
            period_cnt  <= 32'd0;
            frame_start <= 1'b1;
        end else begin
            period_cnt  <= period_cnt + 1'b1;
            frame_start <= 1'b0;
        end
    end
end

//----------------------------------------------------
// 发送控制
//----------------------------------------------------
reg [21:0] send_cnt;
reg [4:0]  tx_byte_cnt;
reg        send_valid;
wire       send_en;

assign send_en  = (send_cnt == SEN_CNT_MAX - 1) ? 1'b1 : 1'b0;
assign pi_flag  = send_en;

reg [7:0] pi_data_r;
assign pi_data = pi_data_r;

// 每帧开始时锁存一次数据，保证整帧使用同一组编码器值
reg [15:0] data_in_reg;
reg [31:0] encoder_in_reg;

always @(posedge clk_50M or negedge rst_n) begin
    if (!rst_n) begin
        data_in_reg    <= 16'd0;
        encoder_in_reg <= 32'h0000_4E20;
    end else if (frame_start) begin
        data_in_reg    <= data_in;
        encoder_in_reg <= encoder_in;
    end
end

// 发送状态控制：每帧发送 10 个字节，发完后停止，等待下一个 frame_start
always @(posedge clk_50M or negedge rst_n) begin
    if (!rst_n) begin
        send_valid  <= 1'b0;
        tx_byte_cnt <= 5'd0;
        send_cnt    <= 22'd0;
    end else begin
        if (frame_start) begin
            send_valid  <= 1'b1;
            tx_byte_cnt <= 5'd0;
            send_cnt    <= 22'd0;
        end else if (send_valid) begin
            if (send_cnt < SEN_CNT_MAX - 1) begin
                send_cnt <= send_cnt + 1'b1;
            end else begin
                send_cnt <= 22'd0;

                if (tx_byte_cnt >= SEN_BYTE_CNT) begin
                    // 一帧发送完成
                    send_valid  <= 1'b0;
                    tx_byte_cnt <= 5'd0;
                end else begin
                    tx_byte_cnt <= tx_byte_cnt + 1'b1;
                end
            end
        end
    end
end

// 根据 tx_byte_cnt 输出当前要发送的字节
always @(posedge clk_50M or negedge rst_n) begin
    if (!rst_n) begin
        pi_data_r <= 8'd0;
    end else if (send_en) begin
        case (tx_byte_cnt)
            0: pi_data_r <= 8'hAA;
            1: pi_data_r <= 8'h55;
            2: pi_data_r <= data_in_reg[15:8];
            3: pi_data_r <= data_in_reg[7:0];
            4: pi_data_r <= encoder_in_reg[31:24];
            5: pi_data_r <= encoder_in_reg[23:16];
            6: pi_data_r <= encoder_in_reg[15:8];
            7: pi_data_r <= encoder_in_reg[7:0];
            8: pi_data_r <= 8'h0D;
            9: pi_data_r <= 8'h0A;
            default: pi_data_r <= 8'h00;
        endcase
    end
end

endmodule