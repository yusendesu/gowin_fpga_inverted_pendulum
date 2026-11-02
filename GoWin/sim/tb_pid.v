`timescale 1ns / 1ps
// pid_core 积分限幅 / 停机清零 / 启动无微分冲击 单元测试
module tb_pid;

    reg               clk = 0, rst_n = 0, update = 0, run = 0, hold_int = 0;
    reg signed [15:0] actual = 0, target = 16'sd2088;
    wire signed [7:0] out;

    always #10 clk = ~clk;

    pid_core #(
        .DIFF_LPF_SHIFT (5'd1)
    ) dut (
        .clk(clk), .rst_n(rst_n), .update(update), .run(run),
        .hold_int(hold_int),
        .actual(actual), .target(target),
        .Kp(32'sd0), .Ki(32'sd590), .Kd(32'sd26870),
        .int_max(16'sd500), .out(out)
    );

    task do_update;
        begin
            @(negedge clk); update = 1'b1;
            @(negedge clk); update = 1'b0;
        end
    endtask

    integer i;
    initial begin
        #100 rst_n = 1;
        run = 1;
        repeat (50) do_update;
        $display("run 50 updates (err=2088): err_int=%0d (%0d counts) out=%0d",
                 $signed(dut.error_int), $signed(dut.error_int)/65536, $signed(out));

        run = 0;
        repeat (3) @(negedge clk);
        $display("after stop              : err_int=%0d out=%0d",
                 $signed(dut.error_int), $signed(out));

        actual = 16'sd2092;   // error = -4
        run = 1;
        repeat (5) do_update;
        $display("restart (err=-4)        : err_int=%0d out=%0d",
                 $signed(dut.error_int), $signed(out));

        // hold_int 上升沿应立即清掉旧积分
        hold_int = 1'b1;
        do_update;
        if (dut.error_int !== 32'sd0) begin
            $display("FAIL clear_int: err_int=%0d (want 0)", $signed(dut.error_int));
        end else begin
            $display("PASS clear_int on hold_int rise: err_int=0");
        end
        hold_int = 1'b0;

        // hold_int: 清 0 后保持
        run = 0;
        repeat (3) @(negedge clk);
        actual = 16'sd0;
        target = 16'sd100;
        hold_int = 1'b1;
        run = 1;
        repeat (10) do_update;
        if (dut.error_int !== 32'sd0) begin
            $display("FAIL hold_int: err_int=%0d (want 0)", $signed(dut.error_int));
        end else begin
            $display("PASS hold_int: err_int=0");
        end

        hold_int = 1'b0;
        repeat (10) do_update;
        if (dut.error_int <= 32'sd0) begin
            $display("FAIL release hold: err_int=%0d", $signed(dut.error_int));
        end else begin
            $display("PASS release hold: err_int=%0d", $signed(dut.error_int));
        end

        // D 对测量微分：目标阶跃但 actual 不动时，D 项不应产生冲击
        run = 0;
        repeat (3) @(negedge clk);
        actual = 16'sd0;
        target = 16'sd0;
        hold_int = 1'b0;
        run = 1;
        repeat (2) do_update;
        target = 16'sd40;      // 阶跃 40 counts
        do_update;
        if (out !== 8'sd0) begin
            $display("FAIL D on measurement: target step produced out=%0d", $signed(out));
        end else begin
            $display("PASS D on measurement: target step D kick = 0");
        end
        $finish;
    end

endmodule
