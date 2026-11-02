// tb_sine_amp.v
// 检查 rotate_ctrl 正弦目标的实际幅值：
//   SINE_AMP_DEG=60、1320 counts/rev 时，target 应在 ±220 counts（±60°）。
// 之前 SINE_AMP_Q16 常量按 32 位回绕，60° 只剩 10.4°（±38 counts），本测试用于回归。
`timescale 1ns/1ps
module tb_sine_amp;
    reg clk = 0;
    reg rst_n = 0;
    reg tick = 0;
    reg balancing = 0, load = 0, lock = 0;
    reg sine_en = 0, cmd_valid = 0;
    reg signed [15:0] load_value = 0, location = 0, cmd_angle = 0;

    wire signed [15:0] target;
    wire signed [31:0] v_ref;
    wire rotating;

    rotate_ctrl #(
        .COUNTS_PER_REV(1320),
        .SINE_AMP_DEG(60),
        .SINE_FREQ_MILLIHZ(500),
        .SINE_GAIN_STEP(131)
    ) dut (
        .clk(clk), .rst_n(rst_n), .tick_1ms(tick),
        .balancing(balancing), .load(load), .load_value(load_value),
        .location(location), .lock(lock),
        .sine_en(sine_en),
        .cmd_valid(cmd_valid), .cmd_angle(cmd_angle),
        .target(target), .v_ref(v_ref), .rotating(rotating),
        .sine_on(), .move_fault()
    );

    always #50 clk = ~clk;   // 10 MHz 假时钟：tick 高 1 拍 = 100 ns

    integer i;
    integer tmax = -32768, tmin = 32767;
    integer vmax = 0;

    initial begin
        #200 rst_n = 1;
        #200 balancing = 1;
        #200 sine_en = 1;

        // 3000 个 tick（RTL 内部按 1 ms/个），覆盖 0.5 s 渐入 + 1.5 个正弦周期
        for (i = 0; i < 3000; i = i + 1) begin
            tick = 1; #100; tick = 0; #900;
            if (target > tmax) tmax = target;
            if (target < tmin) tmin = target;
            if (v_ref > vmax) vmax = v_ref;
            if (v_ref < -vmax) vmax = -v_ref;
            if (i % 250 == 0)
                $display("t=%4d ms  target=%6d counts (%.2f deg)  v_ref=%6d",
                         i, target, target * 360.0 / 1320.0, v_ref);
        end
        $display("MAX=%0d  MIN=%0d  peak-to-peak=%0d counts = %.2f deg, |v_ref|max=%0d",
                 tmax, tmin, tmax - tmin, (tmax - tmin) * 360.0 / 1320.0, vmax);
        if ((tmax > 215) && (tmax < 225) && (tmin < -215) && (tmin > -225))
            $display("PASS: 正弦幅值 = ±%0.1f deg", tmax * 360.0 / 1320.0);
        else
            $display("FAIL: 正弦幅值不对，期望 ±220 counts");
        $finish;
    end
endmodule
