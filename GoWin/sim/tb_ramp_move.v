`timescale 1ns / 1ps
//------------------------------------------------------------------------------
// tb_ramp_move.v
// ramp_move 单元测试（1 MHz 时钟，1 ms tick）：
//   1) +90° 斜坡最终到 330 counts，速度不超过 V_MAX，结束后 v_ref=0；
//   2) 再发 -45°，从 330 回到 165；
//   3) 运动过程中累加两帧 +20°/+40°，最终 220 counts（不是从实际位置重新起步）；
//   4) 连续刷 +180° 命令，总行程被 MAX_TRAVEL 限到 1320 counts（360°）；
//   5) 实际位置卡住不跟随，跟踪误差保护在超时后把目标拉回实际位置并置 fault。
//------------------------------------------------------------------------------
module tb_ramp_move;

    reg         clk = 0;
    reg         rst_n = 0;
    reg         tick_1ms = 0;
    integer     ms_cnt = 0;

    reg         balancing = 0;
    reg         load = 0;
    reg  signed [15:0] load_value = 16'sd0;
    reg  signed [15:0] location = 16'sd0;
    reg         lock = 0;
    reg         cmd_valid = 0;
    reg  signed [15:0] cmd_angle = 16'sd0;

    wire signed [15:0] target;
    wire signed [31:0] v_ref;
    wire               rotating;
    wire               fault;

    // 正常测试让实际位置跟随目标，模拟外环完美跟踪；
    // 跟踪保护测试时 follow=0，位置卡住。
    reg follow = 1;

    ramp_move #(
        .COUNTS_PER_REV    (1320),
        .DIR_SIGN          (1'b1),
        .V_MAX_Q16         (6554),   // 0.10 count/ms
        .A_MAX_Q16         (66),     // 0.001 count/ms^2
        .SETTLE_COUNTS     (2),
        .SETTLE_V_Q16      (328),
        .MAX_TRAVEL_COUNTS (1320),   // 360°
        .TRACK_ERR_COUNTS  (50),     // 测试用较小门限
        .TRACK_TIMEOUT_MS  (20),
        .MOVE_TIMEOUT_MS   (20000)
    ) dut (
        .clk       (clk),
        .rst_n     (rst_n),
        .tick_1ms  (tick_1ms),
        .balancing (balancing),
        .load      (load),
        .load_value(load_value),
        .location  (location),
        .lock      (lock),
        .cmd_valid (cmd_valid),
        .cmd_angle (cmd_angle),
        .target    (target),
        .v_ref     (v_ref),
        .rotating  (rotating),
        .fault     (fault)
    );

    always #500 clk = ~clk;   // 1 MHz

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

    // 模拟实际位置跟随；只在 tick 后更新，和真实编码器一样有 1 ms 延迟
    always @(posedge clk) begin
        if (rst_n && tick_1ms && follow)
            location <= target;
    end

    integer errors = 0;
    integer max_target = 0;
    integer max_v = 0;

    always @(posedge clk) begin
        if (rst_n) begin
            if (target > max_target) max_target <= target;
            if (v_ref > max_v) max_v <= v_ref;
            if (-v_ref > max_v) max_v <= -v_ref;
        end
    end

    task do_load;
        input signed [15:0] v;
        begin
            follow    = 1'b0;
            load_value = v;
            location  = v;
            @(negedge clk);
            load = 1'b1;
            @(negedge clk);   // 只保持一个时钟，和真实单周期脉冲一致
            load = 1'b0;
            #2000;
            follow = 1'b1;
        end
    endtask

    task send_cmd;
        input signed [15:0] deg;
        begin
            cmd_angle = deg;
            @(negedge clk);
            cmd_valid = 1'b1;
            @(negedge clk);   // cmd_valid 是单周期脉冲，不能保持多个时钟
            cmd_valid = 1'b0;
            #2000;
        end
    endtask

    task wait_target;
        input signed [15:0] want;
        input integer       timeout_ms;
        input [8*28-1:0]    tag;
        integer t;
        begin
            t = 0;
            while ((target !== want) && (t < timeout_ms)) begin
                #1_000_000;
                t = t + 1;
            end
            if (target !== want) begin
                $display("FAIL [%0s]: target=%0d (want %0d) after %0d ms, fault=%b",
                         tag, target, want, t, fault);
                errors = errors + 1;
            end else begin
                $display("PASS [%0s]: target=%0d after %0d ms", tag, target, t);
            end
        end
    endtask

    task check_idle;
        input [8*28-1:0] tag;
        integer t;
        begin
            // target 是四舍五入后的整数，可能比轨迹提前 1 ms 到终点；
            // 等 rotating 真正回 0 再检查 v_ref。
            t = 0;
            while ((rotating !== 1'b0) && (t < 500)) begin
                #1_000_000;
                t = t + 1;
            end
            #1000;
            if (rotating !== 1'b0 || v_ref !== 32'sd0) begin
                $display("FAIL [%0s]: rotating=%b v_ref=%0d (want 0/0)",
                         tag, rotating, v_ref);
                errors = errors + 1;
            end else
                $display("PASS [%0s]: rotating=0 v_ref=0 (settle %0d ms)", tag, t);
        end
    endtask

    initial begin
        $display("==== tb_ramp_move ====");
        #200 rst_n = 1;
        #5000;

        balancing = 1'b1;
        #5000;

        do_load(16'sd0);
        if (target !== 16'sd0) begin
            $display("FAIL load: target=%0d", target);
            errors = errors + 1;
        end else
            $display("PASS load: target=0");

        // 1) +90° -> 330 counts
        max_target = 0;
        max_v = 0;
        send_cmd(16'sd90);
        wait_target(16'sd330, 6000, "+90 ramp");
        check_idle("+90 settle");
        if (max_v > 6554) begin
            $display("FAIL +90: |v_ref|max=%0d > 6554", max_v);
            errors = errors + 1;
        end else
            $display("PASS +90: |v_ref|max=%0d", max_v);
        if (max_target > 330) begin
            $display("FAIL +90 overshoot: max_target=%0d", max_target);
            errors = errors + 1;
        end else
            $display("PASS +90: no overshoot (max_target=%0d)", max_target);

        // 2) -45°：实际位置 330 -> 165
        do_load(16'sd330);
        max_target = 0;
        send_cmd(-16'sd45);
        wait_target(16'sd165, 3000, "-45 ramp");
        check_idle("-45 settle");

        // 3) 运动中累加命令：+20° 后 100 ms 再 +40°，最终 220 counts
        do_load(16'sd0);
        send_cmd(16'sd20);
        #100_000_000;             // 100 ms，仍在运动中
        if (rotating !== 1'b1) begin
            $display("FAIL accumulate setup: not moving at 100 ms");
            errors = errors + 1;
        end
        send_cmd(16'sd40);
        wait_target(16'sd220, 5000, "accumulate +20/+40");
        check_idle("accumulate settle");

        // 4) 连续 +180° 刷帧：总行程应卡在 1320 counts（360°）
        do_load(16'sd0);
        max_target = 0;
        send_cmd(16'sd180);
        #10_000_000;
        send_cmd(16'sd180);
        #10_000_000;
        send_cmd(16'sd180);
        wait_target(16'sd1320, 20000, "travel clamp 360deg");
        if (max_target > 1320) begin
            $display("FAIL travel clamp: max_target=%0d > 1320", max_target);
            errors = errors + 1;
        end else
            $display("PASS travel clamp: max_target=%0d", max_target);
        check_idle("clamp settle");

        // 5) 跟踪保护：目标在走，实际位置卡住 -> 20 ms 后放弃并回到 0
        do_load(16'sd0);
        follow   = 1'b0;
        location = 16'sd0;
        #5000;
        send_cmd(16'sd90);
        #1_500_000_000;           // 1.5 s，足够触发 20 ms 跟踪超时
        if (fault !== 1'b1) begin
            $display("FAIL track guard: fault=%b (want 1)", fault);
            errors = errors + 1;
        end else if (target !== 16'sd0 || rotating !== 1'b0) begin
            $display("FAIL track guard abort: target=%0d rotating=%b (want 0/0)",
                     target, rotating);
            errors = errors + 1;
        end else
            $display("PASS track guard: fault=1 target=0");

        if (errors == 0)
            $display("==== ALL TESTS PASSED ====");
        else
            $display("==== %0d TEST(S) FAILED ====", errors);
        $finish;
    end

endmodule
