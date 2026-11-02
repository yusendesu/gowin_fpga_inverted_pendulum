`timescale 1ns / 1ps
//------------------------------------------------------------------------------
// tb_rotate.v
// rotate_ctrl 单元测试（1 MHz 时钟，只为加快仿真）：
//   - load 后 target = 当前转角；
//   - +90° -> 梯形斜坡到 +330 counts（1320 counts/rev）；
//   - -45° -> -165 counts；
//   - K2 lock 把当前位置设为目标；
//   - 非平衡状态命令被忽略。
// 跟踪保护由 tb_ramp_move 单独验证，这里把门限设大，避免 location 不跟随
// 导致测试提前放弃。
//------------------------------------------------------------------------------
module tb_rotate;

    reg         clk = 0;
    reg         rst_n = 0;
    reg         tick_1ms = 0;
    integer     ms_cnt = 0;

    reg         balancing = 0;
    reg         load = 0;
    reg  signed [15:0] load_value = 16'sd1000;
    reg  signed [15:0] location = 16'sd1000;
    reg         lock = 0;
    reg         sine_en = 0;
    reg         cmd_valid = 0;
    reg  signed [15:0] cmd_angle = 16'sd0;

    wire signed [15:0] target;

    always #500 clk = ~clk;   // 1 MHz

    // 1 ms tick
    always @(posedge clk) begin
        if (!rst_n) begin
            ms_cnt   <= 0;
            tick_1ms <= 0;
        end else if (ms_cnt == 999) begin
            ms_cnt   <= 0;
            tick_1ms <= 1;
        end else begin
            ms_cnt   <= ms_cnt + 1;
            tick_1ms <= 0;
        end
    end

    rotate_ctrl #(
        .COUNTS_PER_REV        (1320),
        .DIR_SIGN              (1'b1),
        .RAMP_TRACK_ERR_COUNTS (10000)
    ) dut (
        .clk       (clk),
        .rst_n     (rst_n),
        .tick_1ms  (tick_1ms),
        .balancing (balancing),
        .load      (load),
        .load_value(load_value),
        .location  (location),
        .lock      (lock),
        .sine_en   (sine_en),
        .cmd_valid (cmd_valid),
        .cmd_angle (cmd_angle),
        .target    (target),
        .v_ref     (),
        .rotating  (),
        .sine_on   (),
        .move_fault()
    );

    // 1 ms tick 不再使用（保留空余时间给测试等待）

    integer errors = 0;

    task wait_target;
        input signed [15:0] want;
        input integer       timeout_ms;
        input [8*24-1:0]    tag;
        integer t;
        begin
            t = 0;
            while ((target !== want) && (t < timeout_ms)) begin
                #1_000_000;   // 1 ms
                t = t + 1;
            end
            if (target !== want) begin
                $display("FAIL [%0s]: target=%0d (want %0d) after %0d ms",
                         tag, target, want, t);
                errors = errors + 1;
            end else begin
                $display("PASS [%0s]: target=%0d after %0d ms", tag, target, t);
            end
        end
    endtask

    initial begin
        $display("==== tb_rotate ====");
        #200 rst_n = 1;
        #10_000;

        // 进入平衡
        load = 1'b1;
        #2000;
        load = 1'b0;
        balancing = 1'b1;
        #5000;
        if (target !== 16'sd1000) begin
            $display("FAIL load: target=%0d (want 1000)", target);
            errors = errors + 1;
        end else
            $display("PASS load: target=1000");

        // +90° -> 330 counts，约 3.3 s（0.1 count/ms）
        cmd_angle = 16'sd90;
        @(negedge clk);
        cmd_valid = 1'b1;
        @(negedge clk);
        cmd_valid = 1'b0;
        wait_target(16'sd1330, 6000, "+90 ramp");

        // 摆臂实际到了新位置，再发 -45° -> -165 counts
        location = 16'sd1330;
        #5000;
        cmd_angle = -16'sd45;
        @(negedge clk);
        cmd_valid = 1'b1;
        @(negedge clk);
        cmd_valid = 1'b0;
        wait_target(16'sd1165, 3000, "-45 ramp");

        // K2 锁存：把当前位置 1200 设为位置环目标
        location = 16'sd1200;
        lock = 1'b1;
        #2000;
        lock = 1'b0;
        #2000;
        if (target !== 16'sd1200) begin
            $display("FAIL lock: target=%0d (want 1200)", target);
            errors = errors + 1;
        end else
            $display("PASS lock current position: target=1200");

        // 正弦摆动：±10°、0.5 Hz、中心 1200；sine_en 是电平
        sine_en = 1'b1;
        #500_000_000;   // 500 ms，约四分之一周期，接近正峰值
        if ((target < 16'sd1210) || (target > 16'sd1260)) begin
            $display("FAIL sine start: target=%0d (want 1210..1260)", target);
            errors = errors + 1;
        end else
            $display("PASS sine start: target=%0d", target);

        sine_en = 1'b0;
        #700_000_000;   // 增益渐出后回到中心
        if (target !== 16'sd1200) begin
            $display("FAIL sine stop: target=%0d (want 1200)", target);
            errors = errors + 1;
        end else
            $display("PASS sine stop: target=1200");

        // 非平衡时命令应被忽略，target 清 0
        balancing = 1'b0;
        #5000;
        cmd_angle = 16'sd90;
        cmd_valid = 1'b1;
        #2000;
        cmd_valid = 1'b0;
        #5000;
        if (target !== 16'sd0) begin
            $display("FAIL ignore: target=%0d (want 0)", target);
            errors = errors + 1;
        end else
            $display("PASS ignore while not balancing");

        if (errors == 0)
            $display("==== ALL TESTS PASSED ====");
        else
            $display("==== %0d TEST(S) FAILED ====", errors);
        $finish;
    end

endmodule
