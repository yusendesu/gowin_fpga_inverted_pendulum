//=============================================================================
// pid_core.v  (v2)
// 16.16 定点 PID，带：
//   - run 使能：停机时彻底清零，启动沿预置 error1，消除微分冲击；
//   - int_max 积分限幅（单位：误差码值，不是 16.16 原值）；
//   - 输出饱和时条件积分（抗饱和）；
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

    reg signed [31:0] error1_fixed;
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
    wire signed [31:0] int_next = int_hold ? error_int : int_candidate;

    //-------------------------------------------------------------------------
    // 微分 + 可选一阶低通
    //-------------------------------------------------------------------------
    wire signed [31:0] diff_raw  = error0_fixed - error1_fixed;
    wire signed [31:0] diff_next = (DIFF_LPF_SHIFT == 0) ? diff_raw :
        diff_filtered + ((diff_raw - diff_filtered) >>> DIFF_LPF_SHIFT);

    //-------------------------------------------------------------------------
    // 乘加与限幅
    //-------------------------------------------------------------------------
    wire signed [63:0] mult_p = Kp * error0_fixed;
    wire signed [63:0] mult_i = Ki * int_next;
    wire signed [63:0] mult_d = Kd * diff_next;
    wire signed [63:0] sum64  = mult_p + mult_i + mult_d;

    // 32.32 -> 取整数部分
    wire signed [31:0] integer_part = sum64[63:32];

    wire signed [7:0] out_next = (integer_part >  32'sd100) ?  8'sd100 :
                                 (integer_part < -32'sd100) ? -8'sd100 :
                                 integer_part[7:0];

    //-------------------------------------------------------------------------
    // 时序更新
    //-------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n || !run) begin
            error1_fixed  <= 32'sd0;
            error_int     <= 32'sd0;
            diff_filtered <= 32'sd0;
            out           <= 8'sd0;
        end else if (run_rise) begin
            // 启动瞬间让 D 项为 0，避免目标/位置突然锁存造成踢动
            error1_fixed  <= error0_fixed;
            error_int     <= 32'sd0;
            diff_filtered <= 32'sd0;
            out           <= 8'sd0;
        end else if (update) begin
            error1_fixed  <= error0_fixed;
            error_int     <= int_next;
            diff_filtered <= diff_next;
            out           <= out_next;
        end
    end

endmodule
