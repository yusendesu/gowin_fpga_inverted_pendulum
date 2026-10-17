//=============================================================================
// top.v
// 一阶倒立摆主控（GW2A 系列）
//   - 角度：AD9200 / 3PA1030 10bit 并行 ADC（WDD35D4 角度传感器）
//   - 位置：增量式编码器 x2（四倍频，中心值 32768）
//   - 控制：位置环(外环) + 角度环(内环) 增量式 PID（pid_dual_loop.v）
//   - 驱动：TB6612 x2（两路电机同方向、同占空比）
//   - 交互：ESP32 通过 UART 9600 8N1 发送命令，FPGA 周期回传状态
//
// ESP32 -> FPGA 命令帧（6 字节）：
//   AA 55 CMD DL DH XOR          XOR = CMD ^ DL ^ DH
//   CMD=01 START        启动闭环（仅当角度在安全范围内；启动时锁存当前位置）
//   CMD=02 STOP         停止闭环，电机停转
//   CMD=03 SET_UPRIGHT  DATA=竖直时 theta_raw 的 12bit 原始值（0~4095）
//   CMD=04 SET_DIR      DATA=0 正常，1 电机方向取反（调线序用）
//   CMD=05 CLEAR_FAULT  清除 fault 标志（不会自动启动）
//   CMD=06 CAP_UPRIGHT  以当前 theta_raw 作为竖直校准值
//   CMD=07 SET_POS_DIR  DATA=0 编码器正常，1 编码器计数取反（位置环方向）
//
// FPGA -> ESP32 遥测帧（10 字节，约 0.5s 一帧）：
//   AA 55 TH TL E1H E1L E2H E2L 0D 0A
//   TH/TL = {run, fault, motor_sel[1:0], theta_raw[11:0]}
//       theta     = {TH[3:0], TL}      // 12bit 角度原始值
//       run       = TH[7]
//       fault     = TH[6]
//       motor_sel = TH[5:4]           // 00 停 01 正转 10 反转 11 刹车
//   E1 = enc_pos1（16bit，中心 32768），E2 = enc_pos2
//=============================================================================
module top #(
    //==================== PID 参数 ====================
    parameter signed [15:0] KP_POS = 16'sd10,
    parameter signed [15:0] KI_POS = 16'sd2,
    parameter signed [15:0] KD_POS = 16'sd30,
    parameter signed [15:0] KP_ANG = 16'sd40,
    parameter signed [15:0] KI_ANG = 16'sd3,
    parameter signed [15:0] KD_ANG = 16'sd25,

    //==================== 控制周期 ====================
    // 50MHz * 1ms - 1 = 1kHz。
    // 注意：adc.v 每 3ms 才更新一次 theta_out。如果角度环出现 3ms 周期的
    // 抖动，可把该参数改为 32'd149_999，让 PID 与 ADC 更新同步（333Hz）。
    parameter [31:0] CNT_CYCLE = 32'd49_999,

    //==================== 安全保护 ====================
    // 相对竖直位置的最大允许偏差，单位：12bit ADC 码值。
    // WDD35D4 约 345°/1024(10bit)，12bit 下约 11.9 码/度，360 码 ≈ 30°。
    parameter [12:0] SAFE_ANGLE = 13'd360,
    // 倾倒确认时间：50MHz * 1ms - 1，连续超过 SAFE_ANGLE 达 1ms 立即停机
    parameter [19:0] FALL_CNT_MAX = 20'd49_999,

    // 上电默认电机方向（1=取反），也可以用 CMD=04 在线修改
    parameter DIR_INV_DEFAULT = 1'b0,
    // 上电默认编码器计数方向（1=取反），也可以用 CMD=07 在线修改
    parameter POS_INV_DEFAULT = 1'b0
)(
    input           clk_50m,        // 50MHz 板载时钟
    input           rst_n,          // 复位，低有效

    // ESP32 串口
    output          uart_tx,        // FPGA -> ESP32
    input           uart_rx,        // ESP32 -> FPGA

    // ADC（AD9200 / 3PA1030）接口
    output          adc_clk,        // ADC 采样时钟
    output          adc_oe_n,       // 输出使能，低有效
    input   [9:0]   adc_data_in,    // D9~D0 并行数据

    // 编码器
    input           enc1_a,
    input           enc1_b,
    input           enc2_a,
    input           enc2_b,

    // TB6612 电机 A
    output          AN1,
    output          AN2,
    output          PWMA,

    // TB6612 电机 B
    output          BN1,
    output          BN2,
    output          PWMB
);

//=============================================================================
// 命令定义
//=============================================================================
localparam [7:0] CMD_START       = 8'h01;
localparam [7:0] CMD_STOP        = 8'h02;
localparam [7:0] CMD_SET_UPRIGHT = 8'h03;
localparam [7:0] CMD_SET_DIR     = 8'h04;
localparam [7:0] CMD_CLEAR_FAULT = 8'h05;
localparam [7:0] CMD_CAP_UPRIGHT = 8'h06;
localparam [7:0] CMD_SET_POS_DIR = 8'h07;

// 接收状态机
localparam [2:0] S_IDLE = 3'd0;
localparam [2:0] S_HEAD = 3'd1;
localparam [2:0] S_CMD  = 3'd2;
localparam [2:0] S_DL   = 3'd3;
localparam [2:0] S_DH   = 3'd4;
localparam [2:0] S_CHK  = 3'd5;

//=============================================================================
// 传感器采集
//=============================================================================
wire [11:0] theta_raw;          // 3ms 平均后的角度，0~4092
wire [15:0] enc_pos1, enc_pos2; // 编码器位置，中心 32768

adc adc_inst (
    .clk_50m     (clk_50m    ),
    .rst_n       (rst_n      ),
    .adc_clk     (adc_clk    ),
    .adc_data_in (adc_data_in),
    .theta_out   (theta_raw  )
);

// 新版 adc.v 不再有 adc_stby_n 端口，OE 直接常开（低有效）
assign adc_oe_n = 1'b0;

encoder encoder_1 (
    .clk_50m (clk_50m ),
    .rst_n   (rst_n   ),
    .enc_a   (enc1_a  ),
    .enc_b   (enc1_b  ),
    .pos     (enc_pos1)
);

encoder encoder_2 (
    .clk_50m (clk_50m ),
    .rst_n   (rst_n   ),
    .enc_a   (enc2_a  ),
    .enc_b   (enc2_b  ),
    .pos     (enc_pos2)
);

//=============================================================================
// ESP32 命令接收与解析
//=============================================================================
wire [7:0] rx_byte;
wire       rx_valid;

uart_rx #(
    .UART_BPS (21'd9600       ),
    .CLK_FREQ (26'd50_000_000 )
) uart_rx_inst (
    .sys_clk   (clk_50m ),
    .sys_rst_n (rst_n   ),
    .rx        (uart_rx ),
    .po_data   (rx_byte ),
    .po_flag   (rx_valid)
);

reg [2:0]  cmd_state;
reg [7:0]  cmd_code;
reg [7:0]  cmd_dl;
reg [7:0]  cmd_dh;
reg [19:0] cmd_timeout;         // 半帧超时：20ms 没有新字节则丢弃半帧

reg        start_req;
reg        stop_req;
reg        clear_req;

reg        run;                 // 闭环运行
reg        fault;               // 倾倒或启动条件不满足
reg [11:0] ang_upright;         // 竖直位置对应的 ADC 原始值，默认中值 2048
reg        dir_inv;             // 电机方向取反
reg        pos_inv;             // 编码器计数方向取反

// 位置反馈（停止状态下可通过 CMD=07 取反，围绕 32768 中心翻转）
wire [15:0] pos_fb_used = pos_inv ? (16'd0 - enc_pos1) : enc_pos1;

always @(posedge clk_50m or negedge rst_n) begin
    if(!rst_n) begin
        cmd_state   <= S_IDLE;
        cmd_code    <= 8'd0;
        cmd_dl      <= 8'd0;
        cmd_dh      <= 8'd0;
        cmd_timeout <= 20'd0;
        start_req   <= 1'b0;
        stop_req    <= 1'b0;
        clear_req   <= 1'b0;
        ang_upright <= 12'd2048;
        dir_inv     <= DIR_INV_DEFAULT;
        pos_inv     <= POS_INV_DEFAULT;
    end else begin
        // 请求信号默认拉低，只在解析到完整命令时产生一个时钟脉冲
        start_req <= 1'b0;
        stop_req  <= 1'b0;
        clear_req <= 1'b0;

        if(rx_valid) begin
            cmd_timeout <= 20'd0;
            case(cmd_state)
                S_IDLE: begin
                    if(rx_byte == 8'hAA)
                        cmd_state <= S_HEAD;
                end

                S_HEAD: begin
                    if(rx_byte == 8'h55)
                        cmd_state <= S_CMD;
                    else if(rx_byte != 8'hAA)
                        cmd_state <= S_IDLE;
                end

                S_CMD: begin
                    cmd_code  <= rx_byte;
                    cmd_state <= S_DL;
                end

                S_DL: begin
                    cmd_dl    <= rx_byte;
                    cmd_state <= S_DH;
                end

                S_DH: begin
                    cmd_dh    <= rx_byte;
                    cmd_state <= S_CHK;
                end

                S_CHK: begin
                    cmd_state <= S_IDLE;
                    if(rx_byte == (cmd_code ^ cmd_dl ^ cmd_dh)) begin
                        case(cmd_code)
                            CMD_START:       start_req <= 1'b1;
                            CMD_STOP:        stop_req  <= 1'b1;
                            CMD_CLEAR_FAULT: clear_req <= 1'b1;

                            CMD_SET_UPRIGHT: begin
                                // 0~4095 的 12bit 原始角度；运行中不允许改，防止目标跳变
                                if(!run && cmd_dh[7:4] == 4'h0)
                                    ang_upright <= {cmd_dh[3:0], cmd_dl};
                            end

                            CMD_SET_DIR: begin
                                if(!run && (cmd_dl == 8'd0 || cmd_dl == 8'd1))
                                    dir_inv <= cmd_dl[0];
                            end

                            CMD_SET_POS_DIR: begin
                                if(!run && (cmd_dl == 8'd0 || cmd_dl == 8'd1))
                                    pos_inv <= cmd_dl[0];
                            end

                            CMD_CAP_UPRIGHT: begin
                                if(!run)
                                    ang_upright <= theta_raw;
                            end

                            default: ;
                        endcase
                    end
                end

                default: cmd_state <= S_IDLE;
            endcase
        end else if(cmd_state != S_IDLE) begin
            // 20ms 内没有收到下一个字节，认为半帧丢失，回到空闲重新同步
            if(cmd_timeout >= 20'd999_999) begin
                cmd_state   <= S_IDLE;
                cmd_timeout <= 20'd0;
            end else begin
                cmd_timeout <= cmd_timeout + 1'b1;
            end
        end else begin
            cmd_timeout <= 20'd0;
        end
    end
end

//=============================================================================
// 运行状态 / 安全保护
//=============================================================================
reg [15:0] pos_set;             // 位置目标，START 时锁存
reg [4:0]  pid_clr_cnt;         // PID 复位展宽
reg [19:0] fall_cnt;            // 倾倒确认计数

// 相对竖直位置的偏差（有符号）
wire signed [12:0] ang_dev;
wire        [12:0] ang_abs;
wire               angle_ok;

assign ang_dev  = $signed({1'b0, theta_raw}) - $signed({1'b0, ang_upright});
assign ang_abs  = (ang_dev < 13'sd0) ? -ang_dev : ang_dev;
assign angle_ok = (ang_abs < SAFE_ANGLE);

// PID 复位：START/STOP/FAULT 后拉低约 31 个时钟，清空积分和历史
wire pid_rst_n = rst_n & (pid_clr_cnt == 5'd0);
wire pid_en    = run & ~fault & (pid_clr_cnt == 5'd0);

always @(posedge clk_50m or negedge rst_n) begin
    if(!rst_n) begin
        run         <= 1'b0;
        fault       <= 1'b0;
        pos_set     <= 16'd0;
        pid_clr_cnt <= 5'd0;
        fall_cnt    <= 20'd0;
    end else begin
        //-------------------------------------------------------------
        // 1. PID 复位展宽计数（先自减，后面的命令赋值优先级更高）
        //-------------------------------------------------------------
        if(pid_clr_cnt != 5'd0)
            pid_clr_cnt <= pid_clr_cnt - 1'b1;

        //-------------------------------------------------------------
        // 2. 倾倒保护：连续 1ms 超出安全角度立即停机
        //-------------------------------------------------------------
        if(run && !angle_ok) begin
            if(fall_cnt >= (FALL_CNT_MAX - 1'b1)) begin
                run         <= 1'b0;
                fault       <= 1'b1;
                pid_clr_cnt <= 5'd31;
                fall_cnt    <= 20'd0;
            end else begin
                fall_cnt <= fall_cnt + 1'b1;
            end
        end else begin
            fall_cnt <= 20'd0;
        end

        //-------------------------------------------------------------
        // 3. 启停命令
        //-------------------------------------------------------------
        if(start_req) begin
            if(angle_ok) begin
                // 以当前小车位置作为位置环目标，避免启动瞬间位置环猛拉
                pos_set     <= pos_fb_used;
                run         <= 1'b1;
                fault       <= 1'b0;
                pid_clr_cnt <= 5'd31;
                fall_cnt    <= 20'd0;
            end else begin
                // 角度不在安全范围内，拒绝启动并置 fault，等待 ESP32 处理
                run         <= 1'b0;
                fault       <= 1'b1;
                pid_clr_cnt <= 5'd31;
            end
        end else if(stop_req) begin
            run         <= 1'b0;
            pid_clr_cnt <= 5'd31;
            fall_cnt    <= 20'd0;
        end else if(clear_req) begin
            fault <= 1'b0;
        end
    end
end

//=============================================================================
// 双闭环 PID
//=============================================================================
// 把“竖直=ang_upright”平移到“竖直=2048”，pid_dual_loop 内部固定以 2048
// 为角度目标，这样无需修改 PID 模块即可完成传感器零位校准。
wire [11:0] ang_trim = ang_upright - 12'd2048;
wire [11:0] theta_fb = theta_raw - ang_trim;

wire [11:0] theta_set_dbg;      // 角度环目标（调试用，不参与对外输出）
wire [1:0]  motor_sel_raw;
wire [7:0]  duty_raw;

pid_dual_loop #(
    .KP_POS   (KP_POS   ),
    .KI_POS   (KI_POS   ),
    .KD_POS   (KD_POS   ),
    .KP_ANG   (KP_ANG   ),
    .KI_ANG   (KI_ANG   ),
    .KD_ANG   (KD_ANG   ),
    .CNT_CYCLE(CNT_CYCLE)
) pid_dual_loop_inst (
    .clk_50m   (clk_50m       ),
    .rst_n     (pid_rst_n     ),
    .pid_en    (pid_en        ),
    .pos_fb    (pos_fb_used   ),
    .pos_set   (pos_set       ),
    .theta_fb  (theta_fb      ),
    .theta_set (theta_set_dbg ),
    .motor_sel (motor_sel_raw ),
    .duty_out  (duty_raw      )
);

//=============================================================================
// 电机输出（停止时强制 00 + 占空比 0，避免 PID 残值驱动电机）
//=============================================================================
wire drive_en = run & ~fault;

wire [1:0] motor_sel_inv = (motor_sel_raw == 2'b01) ? 2'b10 :
                           (motor_sel_raw == 2'b10) ? 2'b01 :
                                                      motor_sel_raw;

wire [1:0] motor_sel = drive_en ? (dir_inv ? motor_sel_inv : motor_sel_raw)
                                : 2'b00;
wire [7:0] duty_out  = drive_en ? duty_raw : 8'd0;

motor_ctrl motor_ctrl_A (
    .clk_50m     (clk_50m  ),
    .rst_n       (rst_n    ),
    .motor_a_sel (motor_sel),
    .duty_out    (duty_out ),
    .ain1        (AN1      ),
    .ain2        (AN2      ),
    .pwm         (PWMA     )
);

motor_ctrl motor_ctrl_B (
    .clk_50m     (clk_50m  ),
    .rst_n       (rst_n    ),
    .motor_a_sel (motor_sel),
    .duty_out    (duty_out ),
    .ain1        (BN1      ),
    .ain2        (BN2      ),
    .pwm         (PWMB     )
);

//=============================================================================
// 遥测：角度 + 状态 + 两路编码器
//=============================================================================
wire [15:0] telem_data = {run, fault, motor_sel, theta_raw};
wire [15:0] uart_data_out;

uart #(
    .CNT_1S (32'd24_999_999)     // 0.5s 启动一帧（9600 波特率下一帧约 210ms）
) uart_inst (
    .clk_50M    (clk_50m               ),
    .rst_n      (rst_n                 ),
    .fpga_rx    (uart_rx               ),
    .fpga_tx    (uart_tx               ),
    .data_in    (telem_data            ),
    .encoder_in ({enc_pos1, enc_pos2}  ),
    .data_out   (uart_data_out         )
);

endmodule
