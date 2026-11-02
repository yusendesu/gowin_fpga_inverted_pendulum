`timescale 1ns / 1ps
//------------------------------------------------------------------------------
// tb_top_logic.v
// top 状态机 / 衔接逻辑的快速验证（不跑完整起摆过程）：
//   1. 自然下垂 + 强制 hang_ms 接近门限 -> 自动进入 S_SWING；
//   2. 摆杆在顶部时给一个 captured 脉冲 -> 进入 S_BALANCE，锁存 LocationTarget；
//   3. K1 停止 -> IDLE + armed=0；
//   4. 摆杆立住时按 K1 -> 直接平衡；
//   5. 自然下垂时按 K1 -> 立即起摆；
//   6. 起摆超时 -> IDLE + armed=0。
//
// 起摆算法本身由 tb_swing_fast.v 闭环验证；这里只验证 top 的状态迁移。
//------------------------------------------------------------------------------
module tb_top_logic;

    reg         clk_50m = 0;
    reg         rst_n = 0;
    reg  [9:0]  adc_data_in = 10'd0;
    reg         enc_a = 0, enc_b = 0;
    reg         key1 = 1, key2 = 1, key3 = 1;
    reg         fpga_rx = 1;
    wire        adc_clk, pwm_out, in1, in2, fpga_tx, led;

    localparam [9:0] ADC_HANG = 10'd0;    // angle 0
    localparam [9:0] ADC_UP   = 10'd513;  // 513*4 = 2052，平衡点附近

    always #10 clk_50m = ~clk_50m;

    top dut (
        .clk_50m(clk_50m), .rst_n(rst_n),
        .adc_data_in(adc_data_in), .adc_clk(adc_clk),
        .enc_a(enc_a), .enc_b(enc_b),
        .key1(key1), .key2(key2), .key3(key3), .sine_sw(1'b1),
        .pwm_out(pwm_out), .in1(in1), .in2(in2),
        .fpga_tx(fpga_tx), .fpga_rx(fpga_rx), .led(led)
    );

    integer errors = 0;

    task check_state;
        input [1:0] want_state;
        input       want_armed;
        input [8*24-1:0] tag;
        begin
            #1000;
            if ((dut.state !== want_state) || (dut.armed !== want_armed)) begin
                $display("FAIL [%0s]: state=%0d armed=%b (want %0d/%b) angle=%0d",
                         tag, dut.state, dut.armed, want_state, want_armed, dut.Angle);
                errors = errors + 1;
            end else begin
                $display("PASS [%0s]: state=%0d armed=%b angle=%0d",
                         tag, dut.state, dut.armed, dut.Angle);
            end
        end
    endtask

    task press_k1;
        begin
            key1 = 0;
            #25_000_000;   // 25 ms，覆盖 20 ms 消抖
            key1 = 1;
            #2_000_000;
        end
    endtask

    initial begin
        $display("==== tb_top_logic ====");
        #200 rst_n = 1;
        adc_data_in = ADC_HANG;
        #12_000_000;   // 12 ms：先确认 hang 计数器在正常累加
        if ((dut.hang_ms < 16'd8) || (dut.hang_ms > 16'd13) || !dut.hang_now) begin
            $display("FAIL hang counter: hang_ms=%0d hang_now=%b angle=%0d",
                     dut.hang_ms, dut.hang_now, dut.Angle);
            errors = errors + 1;
        end else begin
            $display("PASS hang counter: hang_ms=%0d angle=%0d", dut.hang_ms, dut.Angle);
        end

        // 1) 自动起摆：把 hang_ms 直接推到门限前，省掉 500 ms 等待
        force dut.hang_ms = 16'd499;
        #2_100_000;   // 2.1 ms，跨过 1 ms tick
        release dut.hang_ms;
        #2_000_000;   // 再等一次 1 ms tick 让 499 -> 500 并触发
        check_state(2'd1, 1'b1, "auto hang -> SWING");

        // 2) 捕获衔接：先把角度放到平衡点附近，再给 captured
        adc_data_in = ADC_UP;
        #2_000_000;
        force dut.swing_captured = 1'b1;
        #200;
        release dut.swing_captured;
        #2_000;       // 等状态迁移 + 一次 PID 更新
        check_state(2'd2, 1'b1, "captured -> BALANCE");
        if (dut.loc_target !== dut.Location) begin
            $display("FAIL loc_target=%0d Location=%0d",
                     $signed(dut.loc_target), $signed(dut.Location));
            errors = errors + 1;
        end

        // 3) K1 停止
        press_k1;
        check_state(2'd0, 1'b0, "K1 stop");

        // 4) 摆杆立住时按 K1 -> 直接平衡
        adc_data_in = ADC_UP;
        #2_000_000;
        press_k1;
        check_state(2'd2, 1'b1, "K1 upright -> BALANCE");
        press_k1;
        check_state(2'd0, 1'b0, "K1 stop again");

        // 5) 自然下垂时按 K1 -> 立即起摆
        adc_data_in = ADC_HANG;
        #2_000_000;
        press_k1;
        check_state(2'd1, 1'b1, "K1 hanging -> SWING");

        // 6) 起摆超时 -> 停机并撤销自动触发
        force dut.swing_timeout = 1'b1;
        #200;
        release dut.swing_timeout;
        #2_000;
        check_state(2'd0, 1'b0, "timeout -> IDLE/disarmed");

        if (errors == 0)
            $display("==== ALL TESTS PASSED ====");
        else
            $display("==== %0d TEST(S) FAILED ====", errors);
        $finish;
    end

endmodule
