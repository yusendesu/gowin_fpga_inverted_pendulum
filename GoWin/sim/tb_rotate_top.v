`timescale 1ns / 1ps
//------------------------------------------------------------------------------
// tb_rotate_top.v
// top 级联调（命令串口 -> angle_cmd -> rotate_ctrl -> 外环目标）：
//   - 强制 state=S_BALANCE，模拟已经平衡；
//   - 115200 bps 发送 AA 55 00 5A 0D（+90°）；
//   - 检查梯形斜坡 loc_target 最终到 330 counts（1320 counts/rev）；
//   - 再发 FF D3（-45°），检查目标到 -165。
// 测试用快速斜坡参数，缩短 50 MHz 仿真时间。
//------------------------------------------------------------------------------
module tb_rotate_top;

    reg         clk_50m = 0;
    reg         rst_n = 0;
    reg  [9:0]  adc_data_in = 10'd513;   // 2052，平衡点附近
    reg         enc_a = 0, enc_b = 0;
    reg         key1 = 1, key2 = 1, key3 = 1;
    reg         fpga_rx = 1;
    wire        adc_clk, pwm_out, in1, in2, fpga_tx, led;

    localparam integer BIT_NS = 8681;    // 115200 bps

    always #10 clk_50m = ~clk_50m;

    top #(
        .UART_RX_BPS           (115200),
        .ARM_COUNTS_PER_REV    (1320),
        .RAMP_V_MAX            (655360),  // 测试用快速斜坡：10 counts/ms
        .RAMP_A_MAX            (655360),  // 10 counts/ms^2
        .RAMP_SETTLE_COUNTS    (2),
        .RAMP_SETTLE_V         (328),
        .RAMP_MAX_TRAVEL_DEG   (360),
        .RAMP_TRACK_ERR_COUNTS (30000),   // 测试中 Location 不跟随，关闭跟踪保护
        .RAMP_TRACK_TIMEOUT_MS (500),
        .RAMP_MOVE_TIMEOUT_MS  (20000),
        .CMD_MSB_FIRST         (1'b1),
        .ROTATE_SIGN           (1'b1)
    ) dut (
        .clk_50m(clk_50m), .rst_n(rst_n),
        .adc_data_in(adc_data_in), .adc_clk(adc_clk),
        .enc_a(enc_a), .enc_b(enc_b),
        .key1(key1), .key2(key2), .key3(key3), .sine_sw(1'b1),
        .pwm_out(pwm_out), .in1(in1), .in2(in2),
        .fpga_tx(fpga_tx), .fpga_rx(fpga_rx), .led(led)
    );

    integer errors = 0;

    task send_byte;
        input [7:0] b;
        integer i;
        begin
            fpga_rx = 1'b0;
            #BIT_NS;
            for (i = 0; i < 8; i = i + 1) begin
                fpga_rx = b[i];
                #BIT_NS;
            end
            fpga_rx = 1'b1;
            #BIT_NS;
            #(BIT_NS / 2);
        end
    endtask

    task send_frame;
        input [15:0] angle;
        begin
            send_byte(8'hAA);
            send_byte(8'h55);
            send_byte(angle[15:8]);
            send_byte(angle[7:0]);
            send_byte(8'h0D);
        end
    endtask

    task wait_target;
        input signed [15:0] want;
        input integer       timeout_ms;
        input [8*24-1:0]    tag;
        integer t;
        begin
            t = 0;
            while ((dut.loc_target !== want) && (t < timeout_ms)) begin
                #1_000_000;
                t = t + 1;
            end
            if (dut.loc_target !== want) begin
                $display("FAIL [%0s]: loc_target=%0d (want %0d) after %0d ms",
                         tag, dut.loc_target, want, t);
                errors = errors + 1;
            end else begin
                $display("PASS [%0s]: loc_target=%0d after %0d ms",
                         tag, dut.loc_target, t);
            end
        end
    endtask

    initial begin
        $display("==== tb_rotate_top ====");
        #200 rst_n = 1;

        // 等 ADC 平均稳定，然后强制进入平衡并加载当前转角
        #2_000_000;
        force dut.state = 2'd2;
        #1_000_000;
        force dut.enter_balance = 1'b1;
        #200;
        release dut.enter_balance;
        #1_000_000;

        if (dut.loc_target !== 16'sd0) begin
            $display("FAIL load: loc_target=%0d (want 0)", dut.loc_target);
            errors = errors + 1;
        end else
            $display("PASS load: loc_target=0");

        // +90° -> 330 counts，目标直接阶跃
        send_frame(16'sd90);
        wait_target(16'sd330, 100, "+90 command");

        // -45°：Location 仍为 0，所以终点是 -165
        send_frame(-16'sd45);
        wait_target(-16'sd165, 100, "-45 command");

        if (errors == 0)
            $display("==== ALL TESTS PASSED ====");
        else
            $display("==== %0d TEST(S) FAILED ====", errors);
        $finish;
    end

endmodule
