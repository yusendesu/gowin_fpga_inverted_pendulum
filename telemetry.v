//=============================================================================
// telemetry.v
// 倒立摆调试遥测发送模块（复用工程已有的 uart_tx 物理层）
//
// 设计目标：
//   1. 100 Hz 连续发送，不再出现旧版 uart.v “每 0.1 s 只发 5 个字节就被
//      下一帧复位打断”的问题；
//   2. 一帧同时给出角度传感器原始值、编码器原始值、角度目标、位置误差、
//      两个 PID 输出、PWM 占空比、运行状态和故障标志；
//   3. 带序号、毫秒时间戳和 CRC-16/CCITT-FALSE，主机可以检测丢帧/错帧；
//   4. 帧头仍以 AA 55 开始，前 8 字节保持旧版“角度 + 32 位编码器”的
//      位置，方便老的串口助手继续看前两个量。
//
// 依赖：uart_tx.v
//=============================================================================
module telemetry #(
    parameter integer CLK_FREQ    = 50_000_000,
    parameter integer UART_BPS    = 115200,
    parameter integer FRAME_MS    = 10,   // 10 ms -> 100 Hz
    parameter integer ENCODER_PPR = 11,   // 旋转编码器线数（四倍频后 44 计数/圈）
    parameter integer ADC_BITS    = 12    // 角度数据位宽
)(
    input  wire               clk,
    input  wire               rst_n,
    input  wire               tick_1ms,       // 1 ms 单周期脉冲

    output wire               uart_tx,        // 接 fpga_tx

    //---- 每帧锁存的数据 ----
    input  wire [11:0]        angle_raw,      // ADC 12 bit 原始角度
    input  wire signed [15:0] angle_target,   // 角度环目标（ADC 码值）
    input  wire [15:0]        encoder_pos,    // 编码器 16 bit 原始计数
    input  wire signed [15:0] loc_error,      // 位置环误差 = target - pos

    input  wire signed [7:0]  angle_pid_out,
    input  wire signed [7:0]  loc_pid_out,
    input  wire signed [7:0]  loc_cmd,        // 位置环限幅后的角度目标偏移
    input  wire signed [7:0]  motor_out,
    input  wire [7:0]         duty,

    input  wire               run,
    input  wire               swing,          // 1 = 自动起摆中
    input  wire               in_window,
    input  wire               loc_sat,
    input  wire               angle_sat
);

    //-------------------------------------------------------------------------
    // 协议常量
    //-------------------------------------------------------------------------
    localparam integer FRAME_BYTES = 34;
    localparam integer CRC_LAST    = 29;      // CRC 覆盖 byte 0..29
    localparam [7:0]   PROTO_VER   = 8'h02;
    localparam [7:0]   PPR_BYTE    = ENCODER_PPR;
    localparam [7:0]   ADC_BYTE    = ADC_BITS;

    // 低波特率时一帧可能超过 FRAME_MS；自动把帧周期放长，避免 overrun
    // 和主机按错误采样率分析。115200 时 MIN=4，仍按 FRAME_MS=10 发。
    localparam integer MIN_FRAME_MS = (FRAME_BYTES * 10 * 1000) / UART_BPS + 2;
    localparam integer FRAME_DIV    = (FRAME_MS > MIN_FRAME_MS) ? FRAME_MS : MIN_FRAME_MS;

    //-------------------------------------------------------------------------
    // UART 发送物理层：直接复用工程已有模块
    //-------------------------------------------------------------------------
    reg  [7:0] pi_data_r;
    reg        pi_flag;
    wire       tx_done;

    uart_tx #(
        .UART_BPS (UART_BPS),
        .CLK_FREQ (CLK_FREQ)
    ) u_tx (
        .sys_clk   (clk),
        .sys_rst_n (rst_n),
        .pi_data   (pi_data_r),
        .pi_flag   (pi_flag),
        .tx        (uart_tx),
        .pi_done   (tx_done)
    );

    //-------------------------------------------------------------------------
    // 毫秒时间戳与帧节拍
    //-------------------------------------------------------------------------
    reg [31:0] t_ms;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) t_ms <= 32'd0;
        else if (tick_1ms) t_ms <= t_ms + 1'b1;
    end

    reg [15:0] ms_div;
    wire frame_tick = tick_1ms && (ms_div == (FRAME_DIV - 1));

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) ms_div <= 16'd0;
        else if (tick_1ms) ms_div <= frame_tick ? 16'd0 : (ms_div + 1'b1);
    end

    //-------------------------------------------------------------------------
    // 每帧锁存寄存器
    //-------------------------------------------------------------------------
    reg [11:0]        angle_lat;
    reg signed [15:0] target_lat;
    reg signed [15:0] locerr_lat;
    reg signed [15:0] angleerr_lat;
    reg        [15:0] pos_lat;
    reg        [15:0] pos_prev;
    reg signed [15:0] delta_lat;
    reg signed [7:0]  apid_lat;
    reg signed [7:0]  lpid_lat;
    reg signed [7:0]  lcmd_lat;
    reg signed [7:0]  motor_lat;
    reg        [7:0]  duty_lat;
    reg        [7:0]  flags_lat;
    reg        [7:0]  seq;
    reg        [7:0]  seq_lat;
    reg        [31:0] t_lat;

    //-------------------------------------------------------------------------
    // 发送状态
    //-------------------------------------------------------------------------
    reg        frame_active;
    reg        tx_busy;
    reg        overrun_flag;
    reg [5:0]  byte_idx;
    reg [15:0] crc;

    wire signed [15:0] angle_act    = $signed({4'b0000, angle_raw});
    wire signed [15:0] angle_err_now = angle_target - angle_act;
    wire signed [15:0] pos_delta_now = $signed(encoder_pos - pos_prev);

    wire frame_ending = tx_done && (byte_idx == (FRAME_BYTES - 1));

    // CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF
    function [15:0] crc16_step;
        input [15:0] c_in;
        input [7:0]  d;
        integer k;
        reg [15:0] c;
        begin
            c = c_in ^ {d, 8'h00};
            for (k = 0; k < 8; k = k + 1)
                c = c[15] ? ((c << 1) ^ 16'h1021) : (c << 1);
            crc16_step = c;
        end
    endfunction

    //-------------------------------------------------------------------------
    // 当前要发送的字节
    //-------------------------------------------------------------------------
    reg [7:0] cur_byte;
    always @(*) begin
        case (byte_idx)
            6'd0:  cur_byte = 8'hAA;
            6'd1:  cur_byte = 8'h55;
            6'd2:  cur_byte = {4'h0, angle_lat[11:8]};
            6'd3:  cur_byte = angle_lat[7:0];
            6'd4:  cur_byte = 8'h00;
            6'd5:  cur_byte = 8'h00;
            6'd6:  cur_byte = pos_lat[15:8];
            6'd7:  cur_byte = pos_lat[7:0];
            6'd8:  cur_byte = PROTO_VER;
            6'd9:  cur_byte = seq_lat;
            6'd10: cur_byte = t_lat[7:0];
            6'd11: cur_byte = t_lat[15:8];
            6'd12: cur_byte = t_lat[23:16];
            6'd13: cur_byte = t_lat[31:24];
            6'd14: cur_byte = target_lat[7:0];
            6'd15: cur_byte = target_lat[15:8];
            6'd16: cur_byte = locerr_lat[7:0];
            6'd17: cur_byte = locerr_lat[15:8];
            6'd18: cur_byte = apid_lat;
            6'd19: cur_byte = lpid_lat;
            6'd20: cur_byte = motor_lat;
            6'd21: cur_byte = duty_lat;
            6'd22: cur_byte = flags_lat;
            6'd23: cur_byte = lcmd_lat;
            6'd24: cur_byte = angleerr_lat[7:0];
            6'd25: cur_byte = angleerr_lat[15:8];
            6'd26: cur_byte = delta_lat[7:0];
            6'd27: cur_byte = delta_lat[15:8];
            6'd28: cur_byte = PPR_BYTE;
            6'd29: cur_byte = ADC_BYTE;
            6'd30: cur_byte = crc[7:0];
            6'd31: cur_byte = crc[15:8];
            6'd32: cur_byte = 8'h0D;
            6'd33: cur_byte = 8'h0A;
            default: cur_byte = 8'h00;
        endcase
    end

    //-------------------------------------------------------------------------
    // 主状态机：锁存 / 逐字节发送
    //-------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pi_data_r    <= 8'd0;
            pi_flag      <= 1'b0;
            frame_active <= 1'b0;
            tx_busy      <= 1'b0;
            overrun_flag <= 1'b0;
            byte_idx     <= 6'd0;
            crc          <= 16'hFFFF;
            seq          <= 8'd0;
            seq_lat      <= 8'd0;
            t_lat        <= 32'd0;
            angle_lat    <= 12'd0;
            target_lat   <= 16'sd0;
            locerr_lat   <= 16'sd0;
            angleerr_lat <= 16'sd0;
            pos_lat      <= 16'd0;
            pos_prev     <= 16'd32768;   // 编码器上电中心，避免首帧 delta 跳变
            delta_lat    <= 16'sd0;
            apid_lat     <= 8'sd0;
            lpid_lat     <= 8'sd0;
            lcmd_lat     <= 8'sd0;
            motor_lat    <= 8'sd0;
            duty_lat     <= 8'd0;
            flags_lat    <= 8'd0;
        end else begin
            pi_flag <= 1'b0;

            // 1) 上一字节发送完成
            if (tx_done) begin
                tx_busy <= 1'b0;
                if (frame_active) begin
                    if (byte_idx == (FRAME_BYTES - 1))
                        frame_active <= 1'b0;
                    else
                        byte_idx <= byte_idx + 1'b1;
                end
            end

            // 2) 帧节拍：锁存整帧数据；若上一帧还没发完则记录 overrun
            if (frame_tick) begin
                if (!frame_active || frame_ending) begin
                    angle_lat    <= angle_raw;
                    target_lat   <= angle_target;
                    locerr_lat   <= loc_error;
                    angleerr_lat <= angle_err_now;
                    pos_lat      <= encoder_pos;
                    delta_lat    <= pos_delta_now;
                    apid_lat     <= angle_pid_out;
                    lpid_lat     <= loc_pid_out;
                    lcmd_lat     <= loc_cmd;
                    motor_lat    <= motor_out;
                    duty_lat     <= duty;
                    // flags: [6]=swing [5]=0 [4]=overrun [3]=angle_sat
                    //        [2]=loc_sat [1]=in_window [0]=run
                    flags_lat    <= {swing, 1'b0, overrun_flag, angle_sat, loc_sat, in_window, run};
                    t_lat        <= t_ms + 1'b1;
                    seq_lat      <= seq;
                    seq          <= seq + 1'b1;
                    pos_prev     <= encoder_pos;
                    byte_idx     <= 6'd0;
                    crc          <= 16'hFFFF;
                    frame_active <= 1'b1;
                    overrun_flag <= 1'b0;
                end else begin
                    overrun_flag <= 1'b1;
                end
            end

            // 3) 启动下一个字节
            if (frame_active && !tx_busy && !pi_flag) begin
                pi_data_r <= cur_byte;
                pi_flag   <= 1'b1;
                tx_busy   <= 1'b1;
                if (byte_idx <= CRC_LAST)
                    crc <= crc16_step(crc, cur_byte);
            end
        end
    end

endmodule
