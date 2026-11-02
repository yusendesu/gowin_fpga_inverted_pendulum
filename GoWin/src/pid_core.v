//=============================================================================
// pid_core.v  (v2)
// 16.16 定点 PID，带：
//   - run 使能：停机时彻底清零，启动沿预置 error1，消除微分冲击；
//   - int_max 积分限幅（单位：误差码值，不是 16.16 原值）；
//   - hold_int 旋转/移动期间清空并保持积分为 0（防止旧位置积分把摆臂拉回）；
//   - 输出饱和时条件积分（抗饱和）；
//   - D 项对“测量值”微分，目标阶跃不会产生微分冲击；
//   - DIFF_LPF_SHIFT 一阶低通微分（模块参数；0 = 不滤波，1 = alpha 1/2，2 = 1/4 ...）。
//
// 与旧版的区别仅在于可靠性/数值安全，控制结构仍是标准位置式 PID：
//   out = Kp*e + Ki*sum(e) + Kd*(e - e1)
// 参数均为 16.16 定点。
//=============================================================================
module pid_core #(
    parameter integer DIFF_LPF_SHIFT = 1   // 微分一阶低通移位，0=不滤波
) (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               update,          // 单周期更新脉冲
    input  wire               run,             // 0: 清零并停止积分
    input  wire               hold_int,        // 1: 清空并保持积分为0（旋转/移动时）
    input  wire signed [15:0] actual,
    input  wire signed [15:0] target,
    input  wire signed [31:0] Kp,              // 16.16
    input  wire signed [31:0] Ki,              // 16.16
    input  wire signed [31:0] Kd,              // 16.16
    input  wire        [15:0] int_max,         // 积分限幅，单位：误差码值
    output reg  signed [7:0]  out              // -100 ~ +100
);

    //-------------------------------------------------------------------------
    // 误差（16.16）
    //-------------------------------------------------------------------------
    wire signed [31:0] actual_fixed = {actual, 16'd0};
    wire signed [31:0] target_fixed = {target, 16'd0};
    wire signed [31:0] error0_fixed = target_fixed - actual_fixed;

    reg signed [31:0] actual1_fixed;  // 上一拍测量值，用于 D 项对测量微分
    reg signed [31:0] error_int;
    reg signed [31:0] diff_filtered;

    //-------------------------------------------------------------------------
    // run 上升沿检测
    //-------------------------------------------------------------------------
    reg run_d;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) run_d <= 1'b0;
        else        run_d <= run;
    end
    wire run_rise = run & ~run_d;

    // hold_int 上升沿：在旋转开始时立即清掉旧位置的积分
    reg hold_int_d;
    wire hold_rise = hold_int & ~hold_int_d;

    //-------------------------------------------------------------------------
    // 积分：33 位累加，防 32 位回绕；对称限幅
    //-------------------------------------------------------------------------
    wire signed [32:0] int_acc = {error_int[31], error_int} +
                                 {error0_fixed[31], error0_fixed};
    wire signed [32:0] int_lim = {1'b0, int_max, 16'd0};   // int_max 个码值
    wire signed [32:0] int_acc_lim = (int_acc >  int_lim) ?  int_lim :
                                     (int_acc < -int_lim) ? -int_lim : int_acc;
    wire signed [31:0] int_clamped = int_acc_lim[31:0];

    // 输出已饱和且误差同向时不再积分
    wire int_hold = ((out >=  8'sd100) && (error0_fixed > 0)) ||
                    ((out <= -8'sd100) && (error0_fixed < 0));
    wire signed [31:0] int_candidate = (Ki != 32'sd0) ? int_clamped : 32'sd0;
    // hold_int=1 时积分被清空并保持 0，而不是保留旧位置的积分
    wire signed [31:0] int_next = hold_int ? 32'sd0 :
                                  (int_hold ? error_int : int_candidate);

    //-------------------------------------------------------------------------
    // 微分 + 可选一阶低通
    //-------------------------------------------------------------------------
    // 对测量微分：error = target - actual，d(error)/dt = -d(actual)/dt，
    // 所以用 actual1 - actual 作为 D 的原始差分。目标阶跃时 actual 不变，D=0。
    wire signed [31:0] diff_raw  = actual1_fixed - actual_fixed;
    wire signed [31:0] diff_next = (DIFF_LPF_SHIFT == 0) ? diff_raw :
        diff_filtered + ((diff_raw - diff_filtered) >>> DIFF_LPF_SHIFT);

    //-------------------------------------------------------------------------
    // 乘加与限幅
    //-------------------------------------------------------------------------
    wire signed [63:0] mult_p = Kp * error0_fixed;
    wire signed [63:0] mult_i = Ki * int_next;
    wire signed [63:0] mult_d = Kd * diff_next;
    wire signed [63:0] sum64  = mult_p + mult_i + mult_d;

    // 32.32 -> 四舍五入到整数（对称，避免负小数被 floor 成 -1 的死区）
    wire signed [63:0] sum_abs = sum64[63] ? -sum64 : sum64;
    wire signed [63:0] sum_abs_round = sum_abs + 64'sd2147483648;  // +0.5 个整数
    wire signed [31:0] sum_mag = sum_abs_round[63:32];
    wire signed [31:0] integer_part = sum64[63] ? -sum_mag : sum_mag;

    wire signed [7:0] out_next = (integer_part >  32'sd100) ?  8'sd100 :
                                 (integer_part < -32'sd100) ? -8'sd100 :
                                 integer_part[7:0];

    //-------------------------------------------------------------------------
    // 时序更新
    //-------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n || !run) begin
            actual1_fixed  <= 32'sd0;
            error_int     <= 32'sd0;
            diff_filtered <= 32'sd0;
            out           <= 8'sd0;
            hold_int_d    <= 1'b0;
        end else begin
            hold_int_d <= hold_int;
            if (run_rise) begin
                // 启动瞬间让 D 项为 0，避免目标/位置突然锁存造成踢动
                actual1_fixed  <= actual_fixed;
                error_int     <= 32'sd0;
                diff_filtered <= 32'sd0;
                out           <= 8'sd0;
            end else if (hold_rise) begin
                // 移动开始：清掉旧积分；D 只对当前测量速度，不受目标阶跃影响
                actual1_fixed  <= actual_fixed;
                error_int     <= 32'sd0;
                diff_filtered <= diff_next;
                out           <= out_next;
            end else if (update) begin
                actual1_fixed  <= actual_fixed;
                error_int     <= int_next;
                diff_filtered <= diff_next;
                out           <= out_next;
            end
        end
    end

endmodule
