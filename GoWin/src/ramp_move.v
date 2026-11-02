//=============================================================================
// ramp_move.v
// 增量式角度移动的“梯形速度斜坡”轨迹发生器（1 ms 更新）。
//
// 旧实现（rotate_ctrl 里的 S 曲线）本质上还是“位置误差 -> PD -> 速度”，
// 而且每收到一帧增量命令都会把轨迹重新从当前实际位置起步：
//   - 连续发帧时轨迹被反复打断，速度前馈跟着一跳一跳；
//   - 没有行程上限、没有跟踪误差保护，外环一旦跟不上目标就会一直往前跑；
//   - 命令只按“当前实际位置 + 本帧角度”重算，无法把多帧增量合并成一条平滑轨迹。
// 实测表现就是“越转越快，最后转飞”。
//
// 本模块改成确定性的梯形速度曲线：
//
//        v
//        ^
//   Vmax |     ______________
//        |    /              \
//        |   /                \
//        |  /                  \
//        +-/--------------------\-------> t
//         Aaccel             Adecel
//
//   - 加速段：每个 1 ms 增加 A_MAX，直到 V_MAX；
//   - 匀速段：保持 V_MAX；
//   - 减速段：根据剩余距离 v^2/(2a) 提前开始减速，保证停在目标点；
//   - 末端：剩余 <= SETTLE_COUNTS 且速度足够小时直接贴到终点，消除拖尾。
//
// 增量命令语义：
//   - 空闲时收到一帧：以“当前实际位置 location”为基准，目标 += 本帧角度；
//   - 运动过程中收到新帧：把本帧角度**累加**到当前目标，不打断速度曲线；
//   - 一次连续移动的总行程被限制在 MAX_TRAVEL_COUNTS 内，防止命令刷屏导致
//     目标无限累加（这是“越转越快”的另一条路径）；
//   - 运动中若“轨迹目标 - 实际位置”超过 TRACK_ERR_COUNTS 并持续
//     TRACK_TIMEOUT_MS，说明外环已经拉不住，立即把目标拉回实际位置并停机，
//     不再继续把目标往前推。
//
// 所有位置/速度均为 16.16 定点：
//   - 位置：counts << 16
//   - 速度：counts/ms << 16
//   - 加速度：counts/ms^2 << 16
//=============================================================================
module ramp_move #(
    parameter integer COUNTS_PER_REV      = 1320,
    parameter         DIR_SIGN            = 1'b1,

    // 梯形曲线参数（Q16）
    parameter integer V_MAX_Q16           = 6554,   // 0.10 count/ms ≈ 27°/s
    parameter integer A_MAX_Q16           = 66,     // 0.001 count/ms² ≈ 1000 counts/s²

    // 末端吸附
    parameter integer SETTLE_COUNTS       = 2,      // 剩余 <= 2 counts 且速度足够小就贴终点
    parameter integer SETTLE_V_Q16        = 328,    // 0.005 count/ms

    // 安全限制
    parameter integer MAX_TRAVEL_COUNTS   = 1320,   // 一次连续增量移动最多 1 圈
    parameter integer TRACK_ERR_COUNTS    = 200,    // 目标超前实际超过 200 counts（约 54°）开始计时
    parameter integer TRACK_TIMEOUT_MS    = 500,    // 持续 500 ms 仍未追上 -> 放弃本次移动
    parameter integer MOVE_TIMEOUT_MS     = 20000   // 单次移动绝对超时
) (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               tick_1ms,

    input  wire               balancing,
    input  wire               load,          // 单周期：把目标装成 load_value
    input  wire signed [15:0] load_value,
    input  wire signed [15:0] location,      // 当前实际摆臂转角（编码器 counts）
    input  wire               lock,          // K2：把当前位置锁为新的目标

    input  wire               cmd_valid,     // 单周期：收到一帧增量角度
    input  wire signed [15:0] cmd_angle,     // 单位：度，-180..+180

    output wire signed [15:0] target,        // 位置环目标（counts）
    output wire signed [31:0] v_ref,         // 速度前馈（Q16 counts/ms）
    output wire               rotating,      // 轨迹未收敛（给外环清积分用）
    output reg                fault          // 跟踪超差/超时保护触发（保持到下一次命令）
);

    //-------------------------------------------------------------------------
    // 参数换算
    //-------------------------------------------------------------------------
    localparam signed [31:0] V_MAX      = V_MAX_Q16;
    localparam signed [31:0] A_MAX      = A_MAX_Q16;
    localparam signed [31:0] SETTLE_Q   = SETTLE_COUNTS * 65536;
    localparam signed [31:0] SETTLE_V   = SETTLE_V_Q16;
    localparam signed [31:0] TRACK_ERR_Q = TRACK_ERR_COUNTS * 65536;

    // 16 位编码器可表示 ±32768 counts；留 8 counts 余量，避免 32 位 Q16 加爆。
    localparam signed [31:0] POS_LIMIT_Q = 32'sd32760 * 32'sd65536;
    localparam signed [39:0] POS_LIMIT_40 = {{8{POS_LIMIT_Q[31]}}, POS_LIMIT_Q};

    // 角度 -> Q16 counts
    localparam signed [31:0] COUNTS_PER_DEG_Q16 =
        (COUNTS_PER_REV * 65536 + 359) / 360;

    wire signed [63:0] delta_full = $signed(cmd_angle) * $signed(COUNTS_PER_DEG_Q16);
    wire signed [63:0] delta_abs  = delta_full[63] ? -delta_full : delta_full;
    wire signed [63:0] delta_mag  = (delta_abs + 64'sd32768) >>> 16;
    wire signed [63:0] delta_trunc = delta_full[63] ? -delta_mag : delta_mag;
    // delta_trunc 是整数 counts，必须再左移 16 位才是 Q16 目标增量
    wire signed [31:0] delta_counts =
        DIR_SIGN ? delta_trunc[31:0] : -delta_trunc[31:0];
    wire signed [31:0] delta_q = delta_counts <<< 16;

    wire signed [31:0] location_q = {location, 16'd0};

    localparam signed [31:0] MAX_TRAVEL_Q = MAX_TRAVEL_COUNTS * 65536;
    localparam signed [39:0] MAX_TRAVEL_40 = {{8{MAX_TRAVEL_Q[31]}}, MAX_TRAVEL_Q};
    localparam signed [39:0] TRACK_ERR_40 = {{8{TRACK_ERR_Q[31]}}, TRACK_ERR_Q};

    //-------------------------------------------------------------------------
    // 轨迹状态
    //-------------------------------------------------------------------------
    reg signed [31:0] pos_q;       // 轨迹当前位置
    reg signed [31:0] goal_q;      // 本段最终目标
    reg signed [31:0] base_q;      // 本段起点（用于总行程限幅）
    reg signed [31:0] v_q;         // 当前速度
    reg               moving;
    reg        [15:0] move_ms;
    reg        [15:0] track_ms;

    // 新命令的基准：
    //   - 运动中把本帧增量累加到“当前目标”；
    //   - 空闲时以实际位置为基准重新起步；
    //   - 总行程限幅始终以本段起点 base_q 为基准。
    wire signed [31:0] eff_base      = moving ? base_q : location_q;
    wire signed [31:0] cmd_goal_base = moving ? goal_q : location_q;
    wire signed [31:0] cmd_start     = moving ? pos_q  : location_q;

    wire signed [39:0] eff_base_40   = {{8{eff_base[31]}}, eff_base};
    wire signed [39:0] goal_base_40  = {{8{cmd_goal_base[31]}}, cmd_goal_base};
    wire signed [39:0] delta_40      = {{8{delta_q[31]}}, delta_q};
    wire signed [39:0] goal_raw      = goal_base_40 + delta_40;
    wire signed [39:0] rel_hi        = eff_base_40 + MAX_TRAVEL_40;
    wire signed [39:0] rel_lo        = eff_base_40 - MAX_TRAVEL_40;

    // 同时限制“相对本段起点的行程”和“16 位编码器绝对范围”
    wire signed [39:0] goal_hi = (rel_hi < POS_LIMIT_40) ? rel_hi : POS_LIMIT_40;
    wire signed [39:0] goal_lo = (rel_lo > -POS_LIMIT_40) ? rel_lo : -POS_LIMIT_40;
    wire signed [39:0] goal_lim = (goal_raw > goal_hi) ? goal_hi :
                                  (goal_raw < goal_lo) ? goal_lo : goal_raw;
    wire signed [31:0] goal_next = goal_lim[31:0];

    //-------------------------------------------------------------------------
    // 轨迹运算
    //-------------------------------------------------------------------------
    wire signed [31:0] err_q   = goal_q - pos_q;
    wire        [31:0] err_abs = err_q[31] ? -err_q : err_q;
    wire        [31:0] v_abs   = v_q[31]   ? -v_q   : v_q;
    wire               dir_neg = err_q[31];

    // 制动距离判据：v^2 + A*v >= 2*A*|err|
    // 离散减速时每 ms 减 A，实际停止距离约为 v^2/(2A)+v/2，
    // 所以把 v*A 也加进来，避免“刚好停在目标上时还有速度”。
    wire signed [63:0] v2        = $signed(v_abs) * $signed(v_abs);
    wire signed [63:0] v_times_a = $signed(v_abs) * $signed(A_MAX);
    wire signed [63:0] brake_now = v2 + v_times_a;
    wire signed [63:0] stop_dist = ($signed(err_abs) * $signed(A_MAX)) <<< 1;
    wire               decel_now = brake_now >= stop_dist;

    // 速度方向错时先减到 0，再朝目标方向加速
    wire v_wrong = (v_q != 32'sd0) && (v_q[31] != dir_neg);

    wire signed [31:0] a_step  = dir_neg ? -A_MAX : A_MAX;
    wire signed [31:0] v_acc   = v_q + a_step;
    wire signed [31:0] v_decel = v_q - a_step;
    wire signed [31:0] v_zero  = v_q + (v_q[31] ? A_MAX : -A_MAX);
    wire signed [31:0] v_cruise = dir_neg ? -V_MAX : V_MAX;

    wire signed [31:0] v_next =
        (err_q == 32'sd0)       ? 32'sd0 :
        v_wrong                 ? ((v_abs <= A_MAX) ? 32'sd0 : v_zero) :
        decel_now               ? ((v_abs <= A_MAX) ? 32'sd0 : v_decel) :
        ((v_abs + A_MAX) >= V_MAX) ? v_cruise : v_acc;

    wire signed [39:0] pos_40     = {{8{pos_q[31]}}, pos_q};
    wire signed [39:0] v_next_40  = {{8{v_next[31]}}, v_next};
    wire signed [39:0] goal_40    = {{8{goal_q[31]}}, goal_q};
    wire signed [39:0] pos_step_40 = pos_40 + v_next_40;

    wire pass_goal = dir_neg ? (pos_step_40 <= goal_40) : (pos_step_40 >= goal_40);
    wire settle    = (err_abs <= SETTLE_Q) && (v_abs <= SETTLE_V);

    wire signed [31:0] pos_next = (pass_goal || settle) ? goal_q : pos_step_40[31:0];
    wire signed [31:0] v_final  = (pass_goal || settle) ? 32'sd0 : v_next;

    // 跟踪保护：轨迹目标不能长期跑在实际位置前面
    wire signed [39:0] loc_40_t  = {{8{location_q[31]}}, location_q};
    wire signed [39:0] track_err = pos_40 - loc_40_t;
    wire        [39:0] track_abs = track_err[39] ? -track_err : track_err;
    wire               track_over = track_abs > TRACK_ERR_40;

    //-------------------------------------------------------------------------
    // 输出
    //-------------------------------------------------------------------------
    wire signed [31:0] pos_abs   = pos_q[31] ? -pos_q : pos_q;
    wire signed [31:0] pos_round = (pos_abs + 32'sd32768) >>> 16;
    wire signed [15:0] pos_out   = pos_q[31] ? -pos_round[15:0] : pos_round[15:0];

    assign target   = balancing ? pos_out : 16'sd0;
    // 跟踪超差暂停时不要把旧速度继续前馈出去，否则目标停住、摆杆还在被
    // 速度前馈往前压，反而更难追上。
    assign v_ref    = (balancing && moving && !track_over) ? v_q : 32'sd0;
    assign rotating = balancing && moving;

    //-------------------------------------------------------------------------
    // 时序
    //-------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pos_q    <= 32'sd0;
            goal_q   <= 32'sd0;
            base_q   <= 32'sd0;
            v_q      <= 32'sd0;
            moving   <= 1'b0;
            move_ms  <= 16'd0;
            track_ms <= 16'd0;
            fault    <= 1'b0;
        end else if (load) begin
            // 进入平衡 / 重新装载：目标 = 当前实际位置，速度归零
            // 注意：load 与 balancing 的上升沿可能在同一拍（top 的 enter_balance），
            // 所以 load 必须优先于 !balancing，否则第一个目标会被清成 0。
            pos_q    <= location_q;
            goal_q   <= location_q;
            base_q   <= location_q;
            v_q      <= 32'sd0;
            moving   <= 1'b0;
            move_ms  <= 16'd0;
            track_ms <= 16'd0;
            fault    <= 1'b0;
        end else if (!balancing) begin
            pos_q    <= 32'sd0;
            goal_q   <= 32'sd0;
            base_q   <= 32'sd0;
            v_q      <= 32'sd0;
            moving   <= 1'b0;
            move_ms  <= 16'd0;
            track_ms <= 16'd0;
            fault    <= 1'b0;
        end else if (lock) begin
            pos_q    <= location_q;
            goal_q   <= location_q;
            base_q   <= location_q;
            v_q      <= 32'sd0;
            moving   <= 1'b0;
            move_ms  <= 16'd0;
            track_ms <= 16'd0;
            fault    <= 1'b0;
        end else if (cmd_valid) begin
            // 空闲：从实际位置起步；运动中：把增量累加到原目标，速度不重置
            if (!moving) begin
                pos_q  <= location_q;
                base_q <= location_q;
            end
            goal_q <= goal_next;

            if (goal_next == cmd_start) begin
                v_q    <= 32'sd0;
                moving <= 1'b0;
            end else begin
                moving <= 1'b1;
            end
            move_ms  <= 16'd0;
            track_ms <= 16'd0;
            fault    <= 1'b0;
        end else if (tick_1ms && moving) begin
            if (track_over) begin
                // 目标超前实际太多：先暂停轨迹让外环追，持续超时就放弃本次移动
                if (track_ms >= (TRACK_TIMEOUT_MS - 1)) begin
                    pos_q    <= location_q;
                    goal_q   <= location_q;
                    base_q   <= location_q;
                    v_q      <= 32'sd0;
                    moving   <= 1'b0;
                    move_ms  <= 16'd0;
                    track_ms <= 16'd0;
                    fault    <= 1'b1;
                end else begin
                    track_ms <= track_ms + 1'b1;
                    v_q      <= 32'sd0;   // 暂停期间速度归零，避免前馈继续推
                end
            end else begin
                track_ms <= 16'd0;
                if (move_ms >= (MOVE_TIMEOUT_MS - 1)) begin
                    // 绝对超时：不再继续推目标
                    pos_q    <= location_q;
                    goal_q   <= location_q;
                    base_q   <= location_q;
                    v_q      <= 32'sd0;
                    moving   <= 1'b0;
                    move_ms  <= 16'd0;
                    fault    <= 1'b1;
                end else begin
                    move_ms <= move_ms + 1'b1;
                    if (pass_goal || settle) begin
                        pos_q   <= goal_q;
                        v_q     <= 32'sd0;
                        moving  <= 1'b0;
                        move_ms <= 16'd0;
                    end else begin
                        pos_q <= pos_next;
                        v_q   <= v_final;
                    end
                end
            end
        end
    end

endmodule
