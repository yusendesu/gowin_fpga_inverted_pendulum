`timescale 1ns / 1ps
//------------------------------------------------------------------------------
// tb_swing.v
// 闭环行为级验证：top + swing_up + 一个“摆杆 + 支点加速度”模型。
//
// 模型：
//   theta  = 摆杆相对竖直向上的角度（rad），pi = 自然下垂
//   theta'' = (g/l)*sin(theta) + (A_FULL/l)*u*cos(theta) - friction*theta'
//   u = top.motor_out / 100
// 符号按“现有平衡环能稳定”的约定选取（与 Python 模型一致）。
//
// 验证内容：
//   1. 上电自然下垂，500 ms 后自动进入起摆；
//   2. 起摆能量泵能把摆杆送到顶部；
//   3. captured 后 top 立即进入平衡，且不再出 ±500 counts 窗口；
//   4. 超时/倒杆会停机。
//------------------------------------------------------------------------------
module tb_swing;

    localparam real PI      = 3.14159265358979;
    localparam real G       = 9.81;
    localparam real L       = 0.30;
    localparam real A_FULL  = 40.0;    // 100% 占空比对应的支点加速度
    localparam real FRIC    = 0.6;     // 摆杆粘性阻尼
    localparam real CPR     = 2048.0 / PI;
    localparam real CENTER  = 2053.0;
    localparam real DT      = 1.0e-4;  // 100 us 积分步长

    reg         clk_50m = 0;
    reg         rst_n   = 0;
    reg  [9:0]  adc_data_in = 10'd0;
    reg         enc_a = 0, enc_b = 0;
    reg         key1 = 1, key2 = 1, key3 = 1;
    reg         fpga_rx = 1;
    wire        adc_clk, pwm_out, in1, in2, fpga_tx, led;

    always #10 clk_50m = ~clk_50m;   // 50 MHz

    top dut (
        .clk_50m(clk_50m), .rst_n(rst_n),
        .adc_data_in(adc_data_in), .adc_clk(adc_clk),
        .enc_a(enc_a), .enc_b(enc_b),
        .key1(key1), .key2(key2), .key3(key3), .sine_sw(1'b1),
        .pwm_out(pwm_out), .in1(in1), .in2(in2),
        .fpga_tx(fpga_tx), .fpga_rx(fpga_rx), .led(led)
    );

    //--------------------------------------------------------------------------
    // 摆杆模型
    //--------------------------------------------------------------------------
    real    theta = PI;      // 自然下垂
    real    omega = 0.0;
    real    alpha, u;
    integer theta_counts;
    integer i;

    always #100_000 begin : plant
        if (rst_n) begin
            u     = $itor(dut.motor_out) / 100.0;
            alpha = (G / L) * $sin(theta) + (A_FULL / L) * u * $cos(theta)
                    - FRIC * omega;
            omega = omega + alpha * DT;
            theta = theta + omega * DT;
            while (theta >  PI) theta = theta - 2.0 * PI;
            while (theta < -PI) theta = theta + 2.0 * PI;

            theta_counts = $rtoi(theta * CPR + CENTER);
            theta_counts = theta_counts % 4096;
            if (theta_counts < 0) theta_counts = theta_counts + 4096;
            adc_data_in = theta_counts[11:2];
        end
    end

    //--------------------------------------------------------------------------
    // 观测
    //--------------------------------------------------------------------------
    reg [1:0] state_d = 2'd0;
    always @(posedge clk_50m) begin
        if (rst_n && (dut.state != state_d)) begin
            $display("[%0t ms] STATE %0d -> %0d  angle=%0d theta=%7.2f deg  armed=%b",
                     $realtime / 1.0e6, state_d, dut.state, dut.Angle,
                     theta * 180.0 / PI, dut.armed);
            state_d <= dut.state;
        end
        if (rst_n && dut.swing_captured)
            $display("[%0t ms] CAPTURED  angle=%0d v_filt=%0d  e=%0d",
                     $realtime / 1.0e6, dut.Angle,
                     $rtoi($signed(dut.u_swing.v_filt)), dut.Angle - 2053);
        if (rst_n && dut.swing_timeout)
            $display("[%0t ms] SWING TIMEOUT", $realtime / 1.0e6);
    end

    // 起摆期间每 100 ms 打印一次
    integer next_log = 0;
    always @(posedge clk_50m) begin
        if (rst_n && dut.swinging && ($realtime / 1.0e6 >= next_log)) begin
            $display("  t=%0t ms swing angle=%0d theta=%7.2f drive=%0d v=%0d",
                     $realtime / 1.0e6, dut.Angle, theta * 180.0 / PI,
                     $signed(dut.swing_drive), $rtoi($signed(dut.u_swing.v_filt)));
            next_log = next_log + 100;
        end
    end

    //--------------------------------------------------------------------------
    // 激励
    //--------------------------------------------------------------------------
    initial begin
        $display("==== tb_swing: power up with pendulum hanging ====");
        #200 rst_n = 1;
        // 为了缩短 50 MHz 仿真时间，直接跳过 500 ms 的 hang 确认
        #2_000_000;
        force dut.hang_ms = 16'd499;
        #2_100_000;
        release dut.hang_ms;
        #2_500_000_000;   // 约 2.5 s：起摆 + 平衡观察
        $display("==== END t=%0t ms state=%0d angle=%0d theta=%7.2f armed=%b motor=%0d ====",
                 $realtime / 1.0e6, dut.state, dut.Angle,
                 theta * 180.0 / PI, dut.armed, $signed(dut.motor_out));
        $finish;
    end

    // 防止仿真无限长
    initial begin
        #3_500_000_000;
        $display("==== WATCHDOG: simulation overran 3.5 s ====");
        $finish;
    end

endmodule
