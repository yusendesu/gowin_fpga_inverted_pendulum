//=============================================================================
// rotate_ctrl.v
// 平衡状态下的摆臂轨迹生成：
//   1) 增量角度移动：梯形速度斜坡（ramp_move.v）；
//   2) 正弦摆动（sine_en 电平：1=运行，0=停止渐出）。
// 两者都输出：
//   - target：位置环目标；
//   - v_ref ：Q16 counts/ms，速度前馈；
//   - rotating：轨迹未收敛/正在摆动，用于清空并保持 PID 积分。
//
// 增量移动已从“位置误差 S 曲线 + PD”改为确定性的梯形斜坡：
//   - 加速段按 A_MAX 增加速度，匀速段保持 V_MAX；
//   - 根据剩余距离 v^2/(2a) 提前减速，末端吸附；
//   - 运动中新命令累加到目标，不再反复把速度清零；
//   - 单次连续行程、跟踪误差、移动时间都有上限，防止“越转越快转飞”。
// 详细说明见 docs/rotate-cmd.md 与 docs/ramp-move.md。
//
// 通信帧格式不变：AA 55 | ANGLE_H ANGLE_L | 0D
//   普通角度：-180..180
//   正弦开关：拨码开关电平，低有效
//=============================================================================
module rotate_ctrl #(
    parameter integer COUNTS_PER_REV      = 1320,
    parameter         DIR_SIGN            = 1'b1,

    // 梯形斜坡参数（Q16）
    parameter integer RAMP_V_MAX           = 6554,   // 0.10 count/ms ≈ 27°/s
    parameter integer RAMP_A_MAX           = 66,     // 0.001 count/ms² ≈ 1000 counts/s²
    parameter integer RAMP_SETTLE_COUNTS   = 2,      // 末端吸附门限（counts）
    parameter integer RAMP_SETTLE_V        = 328,    // 末端吸附速度门限（0.005 count/ms）
    parameter integer RAMP_MAX_TRAVEL_DEG  = 360,    // 一次连续增量最多转多少度
    parameter integer RAMP_TRACK_ERR_COUNTS = 200,   // 跟踪误差保护门限（counts）
    parameter integer RAMP_TRACK_TIMEOUT_MS = 500,   // 持续超差多久后放弃本次移动
    parameter integer RAMP_MOVE_TIMEOUT_MS  = 20000, // 单次移动绝对超时

    // 正弦摆动参数
    parameter integer SINE_AMP_DEG       = 10,
    parameter integer SINE_FREQ_MILLIHZ  = 500,    // 0.5 Hz
    parameter integer SINE_GAIN_STEP     = 131     // 0.5 s 渐入/渐出
) (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               tick_1ms,

    input  wire               balancing,
    input  wire               load,
    input  wire signed [15:0] load_value,
    input  wire signed [15:0] location,

    input  wire               lock,
    input  wire               sine_en,      // 拨码开关电平：1=允许正弦，0=停止
    input  wire               cmd_valid,
    input  wire signed [15:0] cmd_angle,

    output wire signed [15:0] target,
    output wire signed [31:0] v_ref,
    output wire               rotating,
    output wire               sine_on,      // 正弦摆动进行中（给 LED 指示用）
    output wire               move_fault    // 增量移动触发跟踪/超时保护
);

    //-------------------------------------------------------------------------
    // 角度 -> counts（ramp_move 内部自己换算，这里只给正弦中心用）
    //-------------------------------------------------------------------------
    localparam integer MAX_TRAVEL_COUNTS =
        (COUNTS_PER_REV * RAMP_MAX_TRAVEL_DEG + 359) / 360;

    localparam signed [31:0] POS_LIMIT_Q  = 32'sd32760 * 32'sd65536;
    localparam signed [39:0] POS_LIMIT_40 = {{8{POS_LIMIT_Q[31]}}, POS_LIMIT_Q};

    wire signed [31:0] location_q = {location, 16'd0};
    wire signed [31:0] load_q     = {load_value, 16'd0};

    //-------------------------------------------------------------------------
    // 梯形斜坡：增量移动
    //-------------------------------------------------------------------------
    wire signed [15:0] ramp_target;
    wire signed [31:0] ramp_v;
    wire               ramp_rotating;
    wire               ramp_fault;

    reg  signed [31:0] center_q;       // 正弦中心（Q16）
    reg  signed [15:0] center_counts;  // 正弦中心（整数 counts）
    reg                center_load;    // 单周期：让斜坡把当前位置装成中心

    wire ramp_load        = load | (center_load && !lock);
    wire signed [15:0] ramp_load_value = load ? load_value : center_counts;

    ramp_move #(
        .COUNTS_PER_REV       (COUNTS_PER_REV),
        .DIR_SIGN             (DIR_SIGN),
        .V_MAX_Q16            (RAMP_V_MAX),
        .A_MAX_Q16            (RAMP_A_MAX),
        .SETTLE_COUNTS        (RAMP_SETTLE_COUNTS),
        .SETTLE_V_Q16         (RAMP_SETTLE_V),
        .MAX_TRAVEL_COUNTS    (MAX_TRAVEL_COUNTS),
        .TRACK_ERR_COUNTS     (RAMP_TRACK_ERR_COUNTS),
        .TRACK_TIMEOUT_MS     (RAMP_TRACK_TIMEOUT_MS),
        .MOVE_TIMEOUT_MS      (RAMP_MOVE_TIMEOUT_MS)
    ) u_ramp (
        .clk        (clk),
        .rst_n      (rst_n),
        .tick_1ms   (tick_1ms),
        .balancing  (balancing),
        .load       (ramp_load),
        .load_value (ramp_load_value),
        .location   (location),
        .lock       (lock),
        .cmd_valid  (cmd_valid),
        .cmd_angle  (cmd_angle),
        .target     (ramp_target),
        .v_ref      (ramp_v),
        .rotating   (ramp_rotating),
        .fault      (ramp_fault)
    );

    assign move_fault = ramp_fault;

    //-------------------------------------------------------------------------
    // 正弦轨迹 + 渐入/渐出
    //-------------------------------------------------------------------------
    // 注意：不能直接写 (COUNTS_PER_REV*SINE_AMP_DEG*65536+359)/360，
    // 1320*60*65536 = 5.19e9 超过 32 位有符号整数，常量表达式会回绕。
    // 先算 counts 再乘 Q16 就不会溢出。
    localparam signed [31:0] SINE_AMP_COUNTS =
        (COUNTS_PER_REV * SINE_AMP_DEG + 359) / 360;
    localparam signed [31:0] SINE_AMP_Q16 = SINE_AMP_COUNTS * 65536;
    localparam [63:0] PHASE_INC_64 =
        (64'd1 << 32) * SINE_FREQ_MILLIHZ / 1000000;
    localparam [31:0] PHASE_INC = PHASE_INC_64[31:0];
    localparam signed [31:0] OMEGA_Q16 =
        (411775 * SINE_FREQ_MILLIHZ) / 1000000; // 2*pi*65536 ≈ 411775
    localparam signed [31:0] SINE_V_AMP_Q16 = SINE_AMP_COUNTS * OMEGA_Q16;

    reg              sine_active;
    reg              sine_run;
    reg              sine_inhibit;   // 收到角度命令后禁止正弦，直到拨码先关一次
    reg  [31:0]      phase;
    reg  [15:0]      gain;

    wire [7:0] sin_idx = phase[31:24];
    wire [7:0] cos_idx = phase[31:24] + 8'd64;
    wire signed [15:0] sin_val, cos_val;

    sine_lut u_sin (.idx(sin_idx), .val(sin_val));
    sine_lut u_cos (.idx(cos_idx), .val(cos_val));

    wire signed [63:0] sine_pos_raw =
        ($signed(SINE_AMP_Q16) * $signed(sin_val)) >>> 15;
    wire signed [63:0] sine_pos_scaled =
        (sine_pos_raw * $signed({16'd0, gain})) >>> 16;
    wire signed [31:0] sine_pos_off = sine_pos_scaled[31:0];

    // 中心 + 正弦偏移可能越过 16 位范围，先扩到 40 位再对称限幅
    wire signed [39:0] sine_pos_ext =
        {{8{center_q[31]}}, center_q} + {{8{sine_pos_off[31]}}, sine_pos_off};
    wire signed [39:0] sine_pos_lim =
        (sine_pos_ext >  POS_LIMIT_40) ?  POS_LIMIT_40 :
        (sine_pos_ext < -POS_LIMIT_40) ? -POS_LIMIT_40 : sine_pos_ext;
    wire signed [31:0] sine_pos_q = sine_pos_lim[31:0];

    wire signed [63:0] sine_v_raw =
        ($signed(SINE_V_AMP_Q16) * $signed(cos_val)) >>> 15;
    wire signed [63:0] sine_v_scaled =
        (sine_v_raw * $signed({16'd0, gain})) >>> 16;
    wire signed [31:0] sine_v_q = sine_v_scaled[31:0];

    //-------------------------------------------------------------------------
    // 输出选择
    //-------------------------------------------------------------------------
    wire signed [31:0] ramp_pos_q = {ramp_target, 16'd0};
    wire signed [31:0] active_pos_q = sine_active ? sine_pos_q : ramp_pos_q;

    wire signed [31:0] out_abs = active_pos_q[31] ? -active_pos_q : active_pos_q;
    wire signed [31:0] out_round = (out_abs + 32'sd32768) >>> 16;
    assign target = active_pos_q[31] ? -out_round[15:0] : out_round[15:0];
    assign v_ref  = sine_active ? sine_v_q : ramp_v;

    assign rotating = balancing && (sine_active || ramp_rotating);
    assign sine_on  = sine_active;

    //-------------------------------------------------------------------------
    // 时序
    //-------------------------------------------------------------------------
    wire sine_start_ok = sine_en && !sine_inhibit && balancing && !sine_active;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            center_q      <= 32'sd0;
            center_counts <= 16'sd0;
            center_load   <= 1'b0;
            sine_active   <= 1'b0;
            sine_run      <= 1'b0;
            sine_inhibit  <= 1'b0;
            phase         <= 32'd0;
            gain          <= 16'd0;
        end else begin
            center_load <= 1'b0;

            if (load) begin
                center_q      <= load_q;
                center_counts <= load_value;
                sine_active   <= 1'b0;
                sine_run      <= 1'b0;
                sine_inhibit  <= 1'b0;
                phase         <= 32'd0;
                gain          <= 16'd0;
            end else if (!balancing) begin
                center_q      <= 32'sd0;
                center_counts <= 16'sd0;
                sine_active   <= 1'b0;
                sine_run      <= 1'b0;
                sine_inhibit  <= 1'b0;
                phase         <= 32'd0;
                gain          <= 16'd0;
            end else if (lock) begin
                center_q      <= location_q;
                center_counts <= location;
                sine_active   <= 1'b0;
                sine_run      <= 1'b0;
                phase         <= 32'd0;
                gain          <= 16'd0;
            end else if (cmd_valid) begin
                // 普通角度命令优先：退出正弦，禁止开关电平立刻把正弦重新拉起
                sine_active  <= 1'b0;
                sine_run     <= 1'b0;
                sine_inhibit <= 1'b1;
                gain         <= 16'd0;
            end else if (sine_start_ok) begin
                // 拨码开关为电平：允许且尚未运行时，锁存中心并开始正弦
                sine_active   <= 1'b1;
                sine_run      <= 1'b1;
                phase         <= 32'd0;
                gain          <= 16'd0;
                center_q      <= {ramp_target, 16'd0};
                center_counts <= ramp_target;
                center_load   <= 1'b1;
            end else if (tick_1ms) begin
                // 拨码关闭时在这里清禁止标志，并让下面的渐出逻辑继续跑；
                // 不能单独用 else if (!sine_en) 挡住 tick，否则永远渐出不结束。
                if (!sine_en)
                    sine_inhibit <= 1'b0;

                if (sine_active) begin
                    phase <= phase + PHASE_INC;
                    if (sine_run && sine_en) begin
                        if (gain >= (16'd65535 - SINE_GAIN_STEP))
                            gain <= 16'd65535;
                        else
                            gain <= gain + SINE_GAIN_STEP;
                    end else begin
                        if (gain > SINE_GAIN_STEP) begin
                            gain <= gain - SINE_GAIN_STEP;
                        end else begin
                            gain        <= 16'd0;
                            sine_active <= 1'b0;
                            center_load <= 1'b1;   // 回到中心并让斜坡接住
                        end
                    end
                end
            end
        end
    end

endmodule
