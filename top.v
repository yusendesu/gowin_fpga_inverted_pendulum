//=============================================================================
// top.v
// 旋转式倒立摆（Furuta pendulum）主控：
//   - 编码器 = 摆臂在水平面内的转角 φ（16 bit 计数）
//   - ADC    = 摆杆相对竖直的倾角 θ（12 bit）
//   - 内环：摆杆倾角环 5 ms（200 Hz），PD + 小积分
//   - 外环：摆臂转角环 50 ms（20 Hz），PD，输出是摆杆倾角目标的偏移
//   - 串口：115200 bps，100 Hz 二进制遥测（见 telemetry.v）
//
// 自动起摆（本次新增）：
//   1. 角度传感器在 0 或 4095 附近（自然下垂）持续 HANG_CONFIRM_MS，
//      自动进入 S_SWING，触发一次起摆；
//   2. swing_up.v 用“能量泵 + 起始踢动 + v_need 限速”把摆杆送到顶部；
//   3. 摆杆进入 CAPTURE_ANGLE / CAPTURE_VEL 窗口后，立即切到 S_BALANCE，
//      由原有角度环 + 摆臂转角环接管（目标在切换瞬间锁存为当前转角）；
//   4. 起摆超时或平衡中倒下会停机并撤销自动触发（AUTO_RETRY=0 默认），
//      需再按 K1 才会重新起摆；把 AUTO_RETRY 改成 1 可让下垂后自动重试，
//      避免无人看守时无限次反复踢摆。
//
// K1 语义：
//   - 运行中按 K1：立即停止并撤销自动触发；
//   - 静止且摆杆在平衡窗口内按 K1：直接进入平衡；
//   - 静止且摆杆下垂时按 K1：立即起摆；
//   - 其它静止位置按 K1：重新允许自动起摆。
//
// 注意：代码里的 Location / loc_* 是历史命名，指的都是“摆臂转角环”，
//       不是小车直线位置；loc_cmd 的单位是摆杆倾角 ADC counts，
//       loc_error 的单位是编码器 counts，两者不能直接相减。
//
// 历史改动（保留）：
//   1. 修复摆臂转角环实际 18 ms 而不是 50 ms 的计数器位宽错误；
//   2. 摆臂转角环 Kd 按 50 ms 重新折算，输出权限限到 ±60；
//   3. 平衡点统一为 CENTER_ANGLE，不再出现 2010/2063 两套值；
//   4. 启动时把当前摆臂转角锁为外环目标，避免一启动就“回上电零点”；
//   5. PID 增加 run/积分限幅/微分低通，停机清零，启动无微分冲击；
//   6. 串口改为真正能发完的 100 Hz 遥测帧。
//=============================================================================
module top (
    input  wire        clk_50m,
    input  wire        rst_n,
    // ADC
    input  wire [9:0]  adc_data_in,
    output wire        adc_clk,
    // 编码器
    input  wire        enc_a,
    input  wire        enc_b,
    // 按键（K1 启停/手动起摆；K2/K3 保留）
    input  wire        key1,
    input  wire        key2,
    input  wire        key3,
    // 电机
    output wire        pwm_out,       // PWM 调速（接 ENA）
    output wire        in1,           // IN1
    output wire        in2,           // IN2
    // 串口
    output wire        fpga_tx,
    input  wire        fpga_rx,
    // LED
    output wire        led
);

    //==================== 机械 / 传感器参数 ====================
    // 平衡点：把摆杆扶到静止竖直，看串口 angle_raw，把该值填到这里。
    // 自然下垂时 angle_raw 在 0 / 4095 附近，竖直时约为 2053。
    localparam [11:0] CENTER_ANGLE = 12'd2053;
    localparam [11:0] CENTER_RANGE = 12'd500;
    localparam [11:0] CENTER_LOW   = CENTER_ANGLE - CENTER_RANGE;  // 1553
    localparam [11:0] CENTER_HIGH  = CENTER_ANGLE + CENTER_RANGE;  // 2553

    // 旋转编码器 11 线，四倍频后 44 counts/圈（编码器轴）。
    // 旋转式：编码器测的是摆臂转角；若编码器在电机轴上，摆臂一圈计数 = 44 × 减速比。
    localparam integer ENCODER_PPR = 11;
    localparam integer ADC_BITS    = 12;

    //==================== 自动起摆参数 ====================
    // 自然下垂判定：0 或 4095 两侧各 HANG_BAND counts 以内。
    localparam [11:0] HANG_BAND        = 12'd240;    // 约 ±7°
    localparam [15:0] HANG_CONFIRM_MS  = 16'd500;   // 持续 0.5 s 才触发
    localparam        AUTO_ARM_ON_RESET = 1'b1;     // 上电即允许自动起摆
    localparam        AUTO_RETRY        = 1'b0;     // 超时/倒杆后是否自动重试

    // 起摆驱动：先踢一下，再能量泵
    localparam [7:0]  SWING_DUTY          = 8'd80;   // 能量泵占空比 %
    localparam [7:0]  SWING_KICK_DUTY     = 8'd35;   // 起始踢动占空比 %
    localparam [15:0] SWING_KICK_MS       = 16'd100; // 起始踢动时间 ms
    localparam [11:0] SWING_CAPTURE_ANGLE = 12'd300; // 距平衡点 ±300 counts 捕获
    localparam [15:0] SWING_CAPTURE_VEL   = 16'd8;  // counts / 5 ms
    localparam [15:0] SWING_TIMEOUT_MS    = 16'd4000;// 单次起摆超时
    localparam [7:0]  SWING_VNEED_OFFSET  = 8'd0;    // 加大可补偿摩擦
    localparam        SWING_POLARITY      = 1'b1;    // 方向反了改成 1'b0

    //==================== PID 参数（16.16 定点） ====================
    // 摆杆倾角环 5 ms（内环）
    localparam signed [31:0] ANGLE_KP = 32'sd19661;   // 0.30
    localparam signed [31:0] ANGLE_KI = 32'sd655;     // 0.01
    localparam signed [31:0] ANGLE_KD = 32'sd26214;   // 0.40

    // 摆臂转角环 50 ms（外环）：原来 4.00 是按 18 ms 跑的，折算到 50 ms 为 1.44；
    // 这里取 1.50，等效微分作用与原来接近。
    localparam signed [31:0] LOC_KP   = 32'sd29492;   // 0.40
    localparam signed [31:0] LOC_KI   = 32'sd0;       // 0
    localparam signed [31:0] LOC_KD   = 32'sd140000;   // 1.50 188304

    // 摆臂转角环输出权限：单位是摆杆倾角 ADC counts（不是编码器 counts）。
    // 旧版 ±100 过大，容易让摆臂大幅摆动。
    localparam signed [7:0]  LOC_OUT_LIMIT = 8'sd60;

    // 积分限幅（单位：误差码值）与微分低通移位
    localparam [15:0] ANGLE_INT_MAX  = 16'd500;
    localparam [15:0] LOC_INT_MAX    = 16'd1000;
    localparam [4:0]  ANGLE_DIFF_LPF = 5'd1;
    localparam [4:0]  LOC_DIFF_LPF   = 5'd1;

    //==================== 状态机 ====================
    localparam [1:0] S_IDLE    = 2'd0;
    localparam [1:0] S_SWING   = 2'd1;
    localparam [1:0] S_BALANCE = 2'd2;

    reg  [1:0]  state;
    reg         armed;          // 允许自动触发；停机/超时后清零
    reg  [15:0] hang_ms;        // 连续自然下垂计时

    //==================== 内部信号 ====================
    wire [11:0] Angle;
    wire [15:0] pos;
    wire key1_flag, key2_flag, key3_flag;

    // 1 ms 时基
    reg  [16:0] cnt_1ms;
    wire        tick_1ms;

    // 分频计数
    reg  [5:0] cnt_5ms;
    reg  [5:0] cnt_50ms;
    wire       update_angle, update_loc;

    wire swinging  = (state == S_SWING);
    wire balancing = (state == S_BALANCE);
    wire active    = swinging || balancing;

    // 摆臂转角目标：进入平衡瞬间锁存当前摆臂转角
    reg signed [15:0] LocationTarget;

    // 摆臂转角 = 编码器原始值 - 上电中心（16 位模运算后再按有符号解释）
    wire [15:0]        loc_bits = pos - 16'd32768;
    wire signed [15:0] Location = $signed(loc_bits);

    // PID 输出
    wire signed [7:0] angle_pid_out;
    wire signed [7:0] loc_pid_out;

    // 摆臂转角环输出限幅（单位：摆杆倾角 ADC counts）
    wire signed [7:0] loc_cmd = (loc_pid_out >  LOC_OUT_LIMIT) ?  LOC_OUT_LIMIT :
                                (loc_pid_out < -LOC_OUT_LIMIT) ? -LOC_OUT_LIMIT :
                                loc_pid_out;
    wire signed [15:0] loc_cmd_ext = {{8{loc_cmd[7]}}, loc_cmd};

    // 摆杆倾角目标 = 平衡点 - 摆臂转角环输出
    wire signed [15:0] AngleTarget = $signed({4'd0, CENTER_ANGLE}) - loc_cmd_ext;
    wire signed [15:0] angle_actual = $signed({4'd0, Angle});
    wire signed [15:0] angle_error  = AngleTarget - angle_actual;
    wire signed [15:0] loc_error    = LocationTarget - Location;   // 摆臂转角误差

    // 状态标志
    wire angle_in_window = (Angle > CENTER_LOW) && (Angle < CENTER_HIGH);
    wire loc_sat         = (loc_pid_out >= LOC_OUT_LIMIT) || (loc_pid_out <= -LOC_OUT_LIMIT);
    wire angle_sat       = (angle_pid_out >= 8'sd100) || (angle_pid_out <= -8'sd100);

    // 自然下垂：0 或 4095 附近
    wire hang_now = (Angle <= HANG_BAND) || ((12'd4095 - Angle) <= HANG_BAND);

    // 起摆模块输出
    wire signed [7:0] swing_drive;
    wire              swing_captured;
    wire              swing_timeout;

    // 电机驱动：起摆时用 swing_drive，平衡时用角度环输出，其它时候刹车
    wire signed [7:0] motor_out = balancing ? angle_pid_out :
                                  swinging  ? swing_drive   : 8'sd0;
    wire [7:0]        pwm_duty  = motor_out[7] ? (~motor_out + 1'b1) : motor_out;

    // IN1/IN2：运行且输出非零时按符号驱动，否则刹车
    assign in1 = (active && motor_out != 8'sd0) ? ~motor_out[7] : 1'b1;
    assign in2 = (active && motor_out != 8'sd0) ?  motor_out[7] : 1'b1;

    assign led = active;

    //==================== 子模块 ====================
    adc u_adc (
        .clk_50m     (clk_50m),
        .rst_n       (rst_n),
        .adc_clk     (adc_clk),
        .adc_data_in (adc_data_in),
        .theta_out   (Angle)
    );

    encoder u_encoder (
        .clk_50m (clk_50m),
        .rst_n   (rst_n),
        .enc_a   (enc_a),
        .enc_b   (enc_b),
        .pos     (pos)
    );

    key_filter u_key1 (.sys_clk(clk_50m), .sys_rst_n(rst_n), .key_in(key1), .key_flag(key1_flag));
    key_filter u_key2 (.sys_clk(clk_50m), .sys_rst_n(rst_n), .key_in(key2), .key_flag(key2_flag));
    key_filter u_key3 (.sys_clk(clk_50m), .sys_rst_n(rst_n), .key_in(key3), .key_flag(key3_flag));

    pwm_20k_1pct u_pwm (
        .clk       (clk_50m),
        .rstn      (rst_n),
        .pid_duty  (pwm_duty),
        .pwm       (pwm_out)
    );

    swing_up #(
        .CENTER_ANGLE  (CENTER_ANGLE),
        .SWING_DUTY    (SWING_DUTY),
        .KICK_DUTY     (SWING_KICK_DUTY),
        .KICK_MS       (SWING_KICK_MS),
        .CAPTURE_ANGLE (SWING_CAPTURE_ANGLE),
        .CAPTURE_VEL   (SWING_CAPTURE_VEL),
        .TIMEOUT_MS    (SWING_TIMEOUT_MS),
        .VEL_DEADBAND  (16'd2),
        .VNEED_OFFSET  (SWING_VNEED_OFFSET),
        .POLARITY      (SWING_POLARITY)
    ) u_swing (
        .clk      (clk_50m),
        .rst_n    (rst_n),
        .active   (swinging),
        .update   (update_angle),
        .tick_1ms (tick_1ms),
        .angle    (Angle),
        .drive    (swing_drive),
        .captured (swing_captured),
        .timeout  (swing_timeout)
    );

    // 100 Hz 遥测，115200 bps
    telemetry #(
        .CLK_FREQ    (50_000_000),
        .UART_BPS    (115200),
        .FRAME_MS    (10),
        .ENCODER_PPR (ENCODER_PPR),
        .ADC_BITS    (ADC_BITS)
    ) u_telemetry (
        .clk           (clk_50m),
        .rst_n         (rst_n),
        .tick_1ms      (tick_1ms),
        .uart_tx       (fpga_tx),
        .angle_raw     (Angle),
        .angle_target  (AngleTarget),
        .encoder_pos   (pos),
        .loc_error     (loc_error),
        .angle_pid_out (angle_pid_out),
        .loc_pid_out   (loc_pid_out),
        .loc_cmd       (loc_cmd),
        .motor_out     (motor_out),
        .duty          (pwm_duty),
        .run           (balancing),
        .swing         (swinging),
        .in_window     (angle_in_window),
        .loc_sat       (loc_sat),
        .angle_sat     (angle_sat)
    );

    pid_core #(
        .DIFF_LPF_SHIFT (ANGLE_DIFF_LPF)
    ) u_angle_pid (
        .clk            (clk_50m),
        .rst_n          (rst_n),
        .update         (update_angle),
        .run            (balancing),
        .actual         (angle_actual),
        .target         (AngleTarget),
        .Kp             (ANGLE_KP),
        .Ki             (ANGLE_KI),
        .Kd             (ANGLE_KD),
        .int_max        (ANGLE_INT_MAX),
        .out            (angle_pid_out)
    );

    pid_core #(
        .DIFF_LPF_SHIFT (LOC_DIFF_LPF)
    ) u_loc_pid (
        .clk            (clk_50m),
        .rst_n          (rst_n),
        .update         (update_loc),
        .run            (balancing),
        .actual         (Location),
        .target         (LocationTarget),
        .Kp             (LOC_KP),
        .Ki             (LOC_KI),
        .Kd             (LOC_KD),
        .int_max        (LOC_INT_MAX),
        .out            (loc_pid_out)
    );

    //==================== 1 ms 时基与分频 ====================
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) cnt_1ms <= 17'd0;
        else if (tick_1ms) cnt_1ms <= 17'd0;
        else cnt_1ms <= cnt_1ms + 1'b1;
    end
    assign tick_1ms = (cnt_1ms == 17'd49999);

    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            cnt_5ms  <= 6'd0;
            cnt_50ms <= 6'd0;
        end else if (tick_1ms) begin
            cnt_5ms  <= (cnt_5ms  == 5'd4)  ? 6'd0 : cnt_5ms  + 1'b1;
            cnt_50ms <= (cnt_50ms == 6'd49) ? 6'd0 : cnt_50ms + 1'b1;
        end
    end
    assign update_angle = tick_1ms && (cnt_5ms  == 5'd4);
    assign update_loc   = tick_1ms && (cnt_50ms == 6'd49);

    //==================== 自然下垂计时 ====================
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n)
            hang_ms <= 16'd0;
        else if (tick_1ms) begin
            if ((state != S_IDLE) || !armed || !hang_now)
                hang_ms <= 16'd0;
            else if (hang_ms != HANG_CONFIRM_MS)
                hang_ms <= hang_ms + 1'b1;
        end
    end

    //==================== 启停 / 起摆 / 平衡状态机 ====================
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            state          <= S_IDLE;
            armed          <= AUTO_ARM_ON_RESET;
            LocationTarget <= 16'sd0;
        end else if (key1_flag) begin
            if (state != S_IDLE) begin
                // 运行中：立即停止并撤销自动触发
                state <= S_IDLE;
                armed <= 1'b0;
            end else if (angle_in_window) begin
                // 摆杆已立住：直接进入平衡，锁当前摆臂转角
                state          <= S_BALANCE;
                armed          <= 1'b1;
                LocationTarget <= Location;
            end else if (hang_now) begin
                // 自然下垂：立即起摆，不必再等确认时间
                state <= S_SWING;
                armed <= 1'b1;
            end else begin
                // 其它位置：重新允许自动起摆
                armed <= 1'b1;
            end
        end else if (armed && (state == S_IDLE) &&
                     (hang_ms >= HANG_CONFIRM_MS)) begin
            // 自然下垂持续足够久：自动触发一次起摆
            state <= S_SWING;
        end else if (state == S_SWING) begin
            if (swing_captured) begin
                // 起摆完成，立即交给原有平衡逻辑
                state          <= S_BALANCE;
                LocationTarget <= Location;
            end else if (swing_timeout) begin
                // 本次尝试失败：停机；AUTO_RETRY=1 时保留下一次自动触发
                state <= S_IDLE;
                armed <= AUTO_RETRY;
            end
        end else if (state == S_BALANCE) begin
            // 倒杆保护：只在平衡时生效，起摆过程中不检查
            if (!angle_in_window) begin
                state <= S_IDLE;
                armed <= AUTO_RETRY;
            end
        end
    end

endmodule
