`timescale 1ns / 1ps
//------------------------------------------------------------------------------
// tb_top.v
// 复现 top + pid_core 的积分饱和问题：
//   阶段1 (0~100ms): 摆杆下垂 (Angle=0)，RunState=0，但 PID 仍在每 5ms 积分
//   阶段2 (100ms 起): 摆杆扶到平衡点附近 (Angle=2096，仅偏 8 个码值)，
//                     按 K1 启动，观察输出是否直接满占空比
//   加 +reset_int 参数可在 100ms 时清零积分，作为对照
//------------------------------------------------------------------------------
module tb_top;

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

    integer i;

    // 每 5ms（角度环更新）打印一次
    always @(posedge clk_50m) begin
        if (rst_n && dut.update_angle)
            $display("t=%7.1fms  Angle=%4d  err=%5d  err_int=%11d  out=%4d  run=%b  motor=%4d  duty=%3d  in1/in2=%b%b",
                     $realtime/1.0e6, dut.Angle,
                     $signed(dut.AngleTarget) - $signed(dut.angle_actual),
                     $signed(dut.u_angle_pid.error_int),
                     $signed(dut.angle_pid_out),
                     dut.balancing, $signed(dut.motor_out), dut.pwm_duty, in1, in2);
    end

    initial begin
        $display("==== Phase 1: pole down 100ms, RunState=0, PID keeps integrating ====");
        #200 rst_n = 1;

        #100_000_000;  // 100ms

        if ($test$plusargs("reset_int")) begin
            @(negedge clk_50m);
            dut.u_angle_pid.error_int = 32'sd0;
            dut.u_angle_pid.actual1_fixed = dut.u_angle_pid.actual_fixed;
            dut.u_angle_pid.diff_filtered = 32'sd0;
            dut.u_angle_pid.out = 8'sd0;
            $display("#### CONTROL: angle integral cleared @ %7.1fms ####", $realtime/1.0e6);
        end

        // 扶到平衡点附近 2096（偏 8 个码值，刚出死区）
        adc_data_in = 10'd524;   // 524*4 = 2096
        $display("==== Phase 2: pole near balance, press K1 in 5ms ====");
        #5_000_000;
        key1 = 0;  #25_000_000;  key1 = 1;   // 消抖 20ms，产生一次启停脉冲
        #60_000_000;
        $display("==== END t=%.1fms: motor=%0d duty=%0d run=%b ====",
                 $realtime/1.0e6, $signed(dut.motor_out), dut.pwm_duty, dut.balancing);
        $finish;
    end

endmodule
