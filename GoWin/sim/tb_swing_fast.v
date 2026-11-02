`timescale 1ns / 1ps
//------------------------------------------------------------------------------
// tb_swing_fast.v
// swing_up.v 的快速闭环验证（不跑 50 MHz 的 top）：
//   - 1 MHz 时钟，手动产生 1 ms / 5 ms 脉冲；
//   - 同一个摆杆-支点加速度模型；
//   - 验证：踢动 -> 能量泵 -> v_need 限速 -> captured。
// 时钟降到 1 MHz 只是为了仿真速度；模块只认 update/tick_1ms 脉冲，
// 控制周期仍是 5 ms，和硬件一致。
//------------------------------------------------------------------------------
module tb_swing_fast;

    localparam real PI     = 3.14159265358979;
    localparam real G      = 9.81;
    localparam real L      = 0.30;
    localparam real A_FULL = 40.0;
    localparam real FRIC   = 0.6;
    localparam real CPR    = 2048.0 / PI;
    localparam real CENTER = 2053.0;
    localparam real DT     = 1.0e-4;

    reg         clk = 0;
    reg         rst_n = 0;
    reg         active = 1;
    reg  [11:0] angle = 12'd5;
    reg         tick_1ms = 0;
    reg         update = 0;
    wire signed [7:0] drive;
    wire        captured, timeout;
    reg         got_capture = 0;

    always #500 clk = ~clk;   // 1 MHz

    swing_up #(
        .CENTER_ANGLE  (12'd2053),
        .SWING_DUTY    (8'd80),
        .KICK_DUTY     (8'd35),
        .KICK_MS       (16'd100),
        .CAPTURE_ANGLE (12'd300),
        .CAPTURE_VEL   (16'd8),
        .TIMEOUT_MS    (16'd4000),
        .VEL_DEADBAND  (16'd2),
        .VNEED_OFFSET  (8'd0),
        .POLARITY      (1'b1)
    ) dut (
        .clk(clk), .rst_n(rst_n), .active(active),
        .update(update), .tick_1ms(tick_1ms), .angle(angle),
        .drive(drive), .captured(captured), .timeout(timeout)
    );

    // 1 ms / 5 ms 节拍
    integer ms_cnt = 0;
    integer ms5_cnt = 0;
    always @(posedge clk) begin
        if (!rst_n) begin
            ms_cnt  <= 0;
            ms5_cnt <= 0;
            tick_1ms <= 0;
            update   <= 0;
        end else begin
            if (ms_cnt == 999) begin
                ms_cnt    <= 0;
                tick_1ms  <= 1;
                if (ms5_cnt == 4) begin
                    ms5_cnt <= 0;
                    update  <= 1;
                end else begin
                    ms5_cnt <= ms5_cnt + 1;
                    update  <= 0;
                end
            end else begin
                ms_cnt   <= ms_cnt + 1;
                tick_1ms <= 0;
                update   <= 0;
            end
        end
    end

    // 摆杆模型
    real theta = PI;
    real omega = 0.0;
    real alpha, u;
    integer theta_counts;

    always #100_000 begin
        if (rst_n) begin
            u     = $itor(drive) / 100.0;
            alpha = (G / L) * $sin(theta) + (A_FULL / L) * u * $cos(theta)
                    - FRIC * omega;
            omega = omega + alpha * DT;
            theta = theta + omega * DT;
            while (theta >  PI) theta = theta - 2.0 * PI;
            while (theta < -PI) theta = theta + 2.0 * PI;

            theta_counts = $rtoi(theta * CPR + CENTER);
            theta_counts = theta_counts % 4096;
            if (theta_counts < 0) theta_counts = theta_counts + 4096;
            angle = theta_counts[11:0];
        end
    end

    // 观测
    integer next_log = 0;
    always @(posedge clk) begin
        if (rst_n && active && ($realtime / 1.0e6 >= next_log)) begin
            $display("t=%5.1f ms  theta=%7.2f deg  angle=%4d  e=%5d  v=%5.1f  drive=%4d",
                     $realtime / 1.0e6, theta * 180.0 / PI, angle,
                     $signed(angle - 12'd2053), $itor($signed(dut.v_filt)), $signed(drive));
            next_log = next_log + 100;
        end
        if (rst_n && captured) begin
            $display("CAPTURED t=%5.1f ms angle=%0d e=%0d v=%0.1f",
                     $realtime / 1.0e6, angle, $signed(angle - 12'd2053),
                     $itor($signed(dut.v_filt)));
            got_capture <= 1;
            active <= 0;
        end
        if (rst_n && timeout) begin
            $display("TIMEOUT t=%5.1f ms", $realtime / 1.0e6);
            active <= 0;
        end
    end

    initial begin
        $display("==== tb_swing_fast: active=1 from t=0, pendulum hanging ====");
        #200 rst_n = 1;
        #2_500_000_000;   // 2.5 s
        $display("==== END t=%5.1f ms theta=%7.2f got_capture=%b ====",
                 $realtime / 1.0e6, theta * 180.0 / PI, got_capture);
        $finish;
    end

endmodule
