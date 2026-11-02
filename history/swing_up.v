//=============================================================================
// swing_up.v
// 旋转倒立摆自动起摆控制器（能量泵 + 顶部捕获）
//
// 角度定义（与 top.v 一致）：
//   angle 是 12 bit 绝对角度，0/4095 附近 = 摆杆自然下垂，
//   CENTER_ANGLE（默认 2053）= 摆杆竖直向上。
//   e = angle - CENTER_ANGLE（按 12 bit 回绕，范围 -2048..+2047）。
//
// 起摆思路：
//   1) 起摆开始先给一个固定方向、固定时间的“踢一下”，打破静止死点；
//   2) 之后按能量泵方向驱动：
//          u = +sign(v * cos(e))
//      v = de/dt（每 5 ms 的差分，带回绕）。
//      在自然角度符号约定下（平衡环能稳定），这个方向总是给摆杆注入能量；
//   3) v_need(e) 是从当前角度升到竖直所需的最小速度。只有 |v| < v_need 时才
//      继续泵能量，因此摆杆能量接近“刚好到顶”时会自动停止，不会越摆越快；
//   4) |e| 进入 CAPTURE_ANGLE 且 |v| <= CAPTURE_VEL 连续 CAPTURE_TICKS 次，
//      输出 captured，交给 top.v 原有平衡环。
//
// v_need 查表：
//   v_need(e) = sqrt(2*K*(1-cos(pi*|e|/2048)))，K = (g/l)*(T*cnts_per_rad)^2，
//   T = 5 ms，cnts_per_rad = 2048/pi。表按 l = 0.30 m（K≈347）生成，索引
//   |e|/64（0..32）。若摆杆长度不同，可按同一公式重新生成 LUT。
//
// 说明：
//   - 能量泵方向由 POLARITY 控制；若平衡环方向正确，POLARITY=1 即可。
//   - 本模块只管“把摆杆送进捕获窗口”，不做平衡；停机时 drive 立即回 0。
//=============================================================================
module swing_up #(
    parameter [11:0] CENTER_ANGLE  = 12'd2053,
    parameter [7:0]  SWING_DUTY    = 8'd30,    // 能量泵占空比（%）
    parameter [7:0]  KICK_DUTY     = 8'd35,    // 起始踢动占空比（%）
    parameter [15:0] KICK_MS       = 16'd100,  // 起始踢动持续时间（ms）
    parameter [11:0] CAPTURE_ANGLE = 12'd300,  // 进入此角度（counts）才可能捕获
    parameter [15:0] CAPTURE_VEL   = 16'd12,   // 捕获速度门限（counts / 5 ms）
    parameter [15:0] TIMEOUT_MS    = 16'd4000, // 单次起摆超时
    parameter [15:0] VEL_DEADBAND  = 16'd2,    // 速度死区（counts / 5 ms）
    parameter [7:0]  VNEED_OFFSET  = 8'd0,     // v_need 整体加一点，补偿摩擦
    parameter        POLARITY      = 1'b1      // 能量泵方向：1 或 0
) (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               active,       // 高 = top 处于起摆状态
    input  wire               update,       // 5 ms 单周期脉冲
    input  wire               tick_1ms,     // 1 ms 单周期脉冲
    input  wire [11:0]        angle,        // 12 bit 角度原始值
    output reg  signed [7:0]  drive,        // 带符号 PWM 命令（-100..100）
    output reg                captured,     // 单周期脉冲：已到捕获窗口
    output reg                timeout       // 单周期脉冲：本次起摆超时
);

    localparam [3:0] CAPTURE_TICKS = 4'd3;  // 连续满足 3 次 5 ms 才捕获

    //-------------------------------------------------------------------------
    // 角度误差、绝对值、cos 符号
    //-------------------------------------------------------------------------
    wire [11:0]       e_bits = angle - CENTER_ANGLE;   // 12 bit 回绕
    wire signed [11:0] e_s    = $signed(e_bits);
    wire [11:0]       e_abs  = e_s[11] ? (12'd0 - e_bits) : e_bits;
    wire              cos_neg = (e_abs > 12'd1024);    // |e|>90° -> cos<0

    //-------------------------------------------------------------------------
    // v_need 查表（l = 0.30 m，margin = 0）
    //-------------------------------------------------------------------------
    function [7:0] vneed_lut;
        input [5:0] idx;
        begin
            case (idx)
                6'd0:  vneed_lut = 8'd0;   6'd1:  vneed_lut = 8'd2;
                6'd2:  vneed_lut = 8'd4;   6'd3:  vneed_lut = 8'd5;
                6'd4:  vneed_lut = 8'd7;   6'd5:  vneed_lut = 8'd9;
                6'd6:  vneed_lut = 8'd11;  6'd7:  vneed_lut = 8'd13;
                6'd8:  vneed_lut = 8'd14;  6'd9:  vneed_lut = 8'd16;
                6'd10: vneed_lut = 8'd18;  6'd11: vneed_lut = 8'd19;
                6'd12: vneed_lut = 8'd21;  6'd13: vneed_lut = 8'd22;
                6'd14: vneed_lut = 8'd24;  6'd15: vneed_lut = 8'd25;
                6'd16: vneed_lut = 8'd26;  6'd17: vneed_lut = 8'd28;
                6'd18: vneed_lut = 8'd29;  6'd19: vneed_lut = 8'd30;
                6'd20: vneed_lut = 8'd31;  6'd21: vneed_lut = 8'd32;
                6'd22: vneed_lut = 8'd33;  6'd23: vneed_lut = 8'd34;
                6'd24: vneed_lut = 8'd34;  6'd25: vneed_lut = 8'd35;
                6'd26: vneed_lut = 8'd36;  6'd27: vneed_lut = 8'd36;
                6'd28: vneed_lut = 8'd37;  6'd29: vneed_lut = 8'd37;
                6'd30: vneed_lut = 8'd37;  6'd31: vneed_lut = 8'd37;
                default: vneed_lut = 8'd37;
            endcase
        end
    endfunction

    wire [13:0] v_need_ext = {6'd0, vneed_lut(e_abs[11:6])} + {6'd0, VNEED_OFFSET};

    //-------------------------------------------------------------------------
    // 速度估计：5 ms 差分 -> 一阶低通
    // 差分必须先按 12 bit 回绕，否则经过 0/4095 时会出现 ±4096 的假速度。
    //-------------------------------------------------------------------------
    reg  signed [11:0] e_prev;
    reg  signed [13:0] v_filt;
    wire signed [11:0] v_raw      = $signed(e_bits) - e_prev;
    wire signed [13:0] v_raw_ext  = {{2{v_raw[11]}}, v_raw};
    wire signed [13:0] v_abs_s    = v_filt[13] ? (~v_filt + 1'b1) : v_filt;
    wire        [13:0] v_abs      = v_abs_s;

    wire          pump_ok = (v_abs > VEL_DEADBAND) && (v_abs < v_need_ext) &&
                            (e_abs > CAPTURE_ANGLE);
    wire          kick_ok = (v_abs < v_need_ext) && (e_abs > CAPTURE_ANGLE);
    wire          at_top  = (e_abs < CAPTURE_ANGLE) &&
                            (v_abs <= CAPTURE_VEL);

    //-------------------------------------------------------------------------
    // 方向：u = +sign(v*cos(e))，再用 POLARITY 全局取反
    //-------------------------------------------------------------------------
    wire prod_neg = v_filt[13] ^ cos_neg;
    wire dir_neg  = prod_neg ^ ~POLARITY;

    wire signed [7:0] swing_duty_s = SWING_DUTY;
    wire signed [7:0] kick_duty_s  = KICK_DUTY;
    wire signed [7:0] pump_drive   = dir_neg ? -swing_duty_s : swing_duty_s;
    wire signed [7:0] kick_drive   = POLARITY ? kick_duty_s : -kick_duty_s;

    //-------------------------------------------------------------------------
    // 计时/状态
    //-------------------------------------------------------------------------
    reg        active_d;
    wire       active_rise = active & ~active_d;

    reg [15:0] active_ms;
    reg [15:0] kick_ms;
    reg [3:0]  cap_cnt;
    wire       kick_win = (kick_ms < KICK_MS);
    wire       drive_en = kick_win ? kick_ok : pump_ok;
    wire signed [7:0] drive_cmd = kick_win ? kick_drive : pump_drive;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) active_d <= 1'b0;
        else        active_d <= active;
    end

    // 速度/角度历史：停机时持续跟踪，起摆上升沿对齐，避免第一次差分误判
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            e_prev <= 12'sd0;
            v_filt <= 14'sd0;
        end else if (!active || active_rise) begin
            e_prev <= e_s;
            v_filt <= 14'sd0;
        end else if (update) begin
            e_prev <= e_s;
            v_filt <= v_filt + ((v_raw_ext - v_filt) >>> 1);
        end
    end

    // 毫秒计时（超时、踢动）
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n || !active) begin
            active_ms <= 16'd0;
            kick_ms   <= 16'd0;
        end else if (active_rise) begin
            active_ms <= 16'd0;
            kick_ms   <= 16'd0;
        end else if (tick_1ms) begin
            if (active_ms != TIMEOUT_MS)
                active_ms <= active_ms + 1'b1;
            if (kick_ms != KICK_MS)
                kick_ms <= kick_ms + 1'b1;
        end
    end

    // 超时脉冲
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) timeout <= 1'b0;
        else        timeout <= active && tick_1ms &&
                               (active_ms == (TIMEOUT_MS - 1'b1));
    end

    // 捕获：连续 CAPTURE_TICKS 次 5 ms 都在窗口内
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n || !active || active_rise) begin
            captured <= 1'b0;
            cap_cnt  <= 4'd0;
        end else if (update) begin
            if (at_top) begin
                if (cap_cnt == CAPTURE_TICKS - 1'b1) begin
                    captured <= 1'b1;
                    cap_cnt  <= 4'd0;
                end else begin
                    captured <= 1'b0;
                    cap_cnt  <= cap_cnt + 1'b1;
                end
            end else begin
                captured <= 1'b0;
                cap_cnt  <= 4'd0;
            end
        end else begin
            captured <= 1'b0;
        end
    end

    // 驱动输出
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n || !active)
            drive <= 8'sd0;
        else if (update)
            drive <= drive_en ? drive_cmd : 8'sd0;
    end

endmodule
