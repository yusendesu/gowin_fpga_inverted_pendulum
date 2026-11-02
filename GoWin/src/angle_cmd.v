//=============================================================================
// angle_cmd.v
// ESP32 旋转角度命令接收 / 解帧
//
// 帧格式（5 字节，无校验）：
//     AA 55 | ANGLE_H ANGLE_L | 0D
//   - ANGLE_H/ANGLE_L 为有符号 16 bit，单位：度，范围 -180 ~ +180；
//   - 默认大端（ANGLE_H 先到），与 interfacetest 示例的 uart.v 一致；
//     如果 ESP32 发的是小端，把 MSB_FIRST 改成 0；
//   - 角度超出 ±180 时按边界截断，避免异常帧把摆臂目标拉飞；
//   - 特殊值 0x7FFF = 开始正弦摆动，0x8000 = 停止正弦摆动（不改变帧格式）。
//
// 解帧状态机对数据字节完全透明（数据字节可以是 AA/55/0D），只在帧尾位置
// 检查 0x0D。若帧尾不对，会把当前字节当作可能的 AA 重新同步。
//
// 串口字节由 uart_rx.v 给出（参考 D:\project\fpga\Gowin\interfacetest\src）。
// 帧内字节超时（默认 20 ms）会复位状态机，防止丢字节后卡死。
//=============================================================================
module angle_cmd #(
    parameter integer CLK_FREQ          = 50_000_000,
    parameter integer UART_BPS          = 115200, // 与 ESP32 发送波特率一致
    parameter integer FRAME_TIMEOUT_MS  = 20,     // 帧内字节超时
    parameter         MSB_FIRST         = 1'b1    // 1: 高字节先到
) (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               tick_1ms,
    input  wire               rx,

    output reg  signed [15:0] angle_deg,  // 解析出的角度（-180 ~ +180）
    output reg                valid,      // 普通角度帧，单周期脉冲
    output reg                sine_start, // 0x7FFF：开始正弦摆动
    output reg                sine_stop   // 0x8000：停止正弦摆动
);

    //-------------------------------------------------------------------------
    // UART 字节接收
    //-------------------------------------------------------------------------
    wire [7:0] rx_data;
    wire       rx_valid;

    uart_rx #(
        .UART_BPS (UART_BPS),
        .CLK_FREQ (CLK_FREQ)
    ) u_uart_rx (
        .sys_clk  (clk),
        .sys_rst_n(rst_n),
        .rx       (rx),
        .po_data  (rx_data),
        .po_flag  (rx_valid)
    );

    //-------------------------------------------------------------------------
    // 解帧
    //-------------------------------------------------------------------------
    localparam [2:0] S_HEAD0  = 3'd0,
                     S_HEAD1  = 3'd1,
                     S_DATA_H = 3'd2,
                     S_DATA_L = 3'd3,
                     S_TAIL   = 3'd4;

    reg  [2:0] state;
    reg  [7:0] data_h;
    reg  [7:0] data_l;
    reg  [4:0] timeout_ms;

    wire [15:0] raw_bits = MSB_FIRST ? {data_h, data_l} : {data_l, data_h};
    wire signed [16:0] raw_s = $signed(raw_bits);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state      <= S_HEAD0;
            data_h     <= 8'd0;
            data_l     <= 8'd0;
            timeout_ms <= 5'd0;
            angle_deg  <= 16'sd0;
            valid      <= 1'b0;
            sine_start <= 1'b0;
            sine_stop  <= 1'b0;
        end else begin
            valid      <= 1'b0;   // 默认只脉冲一个周期
            sine_start <= 1'b0;
            sine_stop  <= 1'b0;

            if (rx_valid) begin
                timeout_ms <= 5'd0;
                case (state)
                    S_HEAD0: begin
                        if (rx_data == 8'hAA)
                            state <= S_HEAD1;
                        else
                            state <= S_HEAD0;
                    end
                    S_HEAD1: begin
                        if (rx_data == 8'h55)
                            state <= S_DATA_H;
                        else if (rx_data == 8'hAA)
                            state <= S_HEAD1;   // 连续 AA，重新当帧头
                        else
                            state <= S_HEAD0;
                    end
                    S_DATA_H: begin
                        data_h <= rx_data;
                        state  <= S_DATA_L;
                    end
                    S_DATA_L: begin
                        data_l <= rx_data;
                        state  <= S_TAIL;
                    end
                    S_TAIL: begin
                        state <= S_HEAD0;
                        if (rx_data == 8'h0D) begin
                            if (raw_bits == 16'h7FFF) begin
                                sine_start <= 1'b1;
                            end else if (raw_bits == 16'h8000) begin
                                sine_stop <= 1'b1;
                            end else begin
                                if (raw_s > 17'sd180)
                                    angle_deg <= 16'sd180;
                                else if (raw_s < -17'sd180)
                                    angle_deg <= -16'sd180;
                                else
                                    angle_deg <= raw_s[15:0];
                                valid <= 1'b1;
                            end
                        end else if (rx_data == 8'hAA) begin
                            state <= S_HEAD1;   // 帧尾丢失，当前字节可能是新帧头
                        end
                    end
                    default: state <= S_HEAD0;
                endcase
            end else if (tick_1ms && (state != S_HEAD0)) begin
                if (timeout_ms >= (FRAME_TIMEOUT_MS - 1))
                    state <= S_HEAD0;
                else
                    timeout_ms <= timeout_ms + 1'b1;
            end
        end
    end

endmodule
