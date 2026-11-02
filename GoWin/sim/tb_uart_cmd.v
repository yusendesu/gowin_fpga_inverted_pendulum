`timescale 1ns / 1ps
//------------------------------------------------------------------------------
// tb_uart_cmd.v
// angle_cmd 解帧测试：
//   帧：AA 55 + 16 bit 有符号角度（大端）+ 0D，115200 bps。
//   覆盖：+90/-90、数据字节为 AA/55/0D、超范围截断。
//------------------------------------------------------------------------------
module tb_uart_cmd;

    reg         clk = 0;
    reg         rst_n = 0;
    reg         rx = 1;
    reg         tick_1ms = 0;
    wire signed [15:0] angle_deg;
    wire        valid;
    wire        sine_start;
    wire        sine_stop;

    localparam integer BIT_NS = 8681;   // 1/115200 s ≈ 8.68 us

    always #10 clk = ~clk;   // 50 MHz

    angle_cmd #(
        .CLK_FREQ         (50_000_000),
        .UART_BPS         (115200),
        .FRAME_TIMEOUT_MS (20),
        .MSB_FIRST        (1'b1)
    ) dut (
        .clk      (clk),
        .rst_n    (rst_n),
        .tick_1ms (tick_1ms),
        .rx       (rx),
        .angle_deg(angle_deg),
        .valid    (valid),
        .sine_start(sine_start),
        .sine_stop (sine_stop)
    );

    integer errors = 0;
    reg        got_valid = 0;
    reg signed [15:0] got_angle = 0;
    reg        got_sine_start = 0;
    reg        got_sine_stop  = 0;

    always @(posedge clk) begin
        if (valid) begin
            got_valid <= 1'b1;
            got_angle <= angle_deg;
        end
        if (sine_start) got_sine_start <= 1'b1;
        if (sine_stop)  got_sine_stop  <= 1'b1;
    end

    task send_byte;
        input [7:0] b;
        integer i;
        begin
            rx = 1'b0;
            #BIT_NS;
            for (i = 0; i < 8; i = i + 1) begin
                rx = b[i];
                #BIT_NS;
            end
            rx = 1'b1;
            #BIT_NS;
            #(BIT_NS / 2);   // 字节间空闲
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

    task check_frame;
        input signed [15:0] want;
        input [8*24-1:0]   tag;
        begin
            #2_000_000;   // 2 ms，等 valid 脉冲
            if (!got_valid) begin
                $display("FAIL [%0s]: no valid (angle=%0d)", tag, angle_deg);
                errors = errors + 1;
            end else if (got_angle !== want) begin
                $display("FAIL [%0s]: angle=%0d (want %0d)", tag, got_angle, want);
                errors = errors + 1;
            end else begin
                $display("PASS [%0s]: angle=%0d", tag, got_angle);
            end
            got_valid = 1'b0;
            got_angle = 16'sd0;
        end
    endtask

    initial begin
        $display("==== tb_uart_cmd ====");
        #200 rst_n = 1;
        #20_000;

        send_frame(16'sd90);
        check_frame(16'sd90, "+90");

        send_frame(-16'sd90);
        check_frame(-16'sd90, "-90");

        // 数据字节里出现 0xAA / 0x55 / 0x0D
        send_frame(-16'sd86);       // FF AA
        check_frame(-16'sd86, "-86 data=AA");

        send_frame(16'sd85);        // 00 55
        check_frame(16'sd85, "+85 data=55");

        send_frame(16'sd13);        // 00 0D
        check_frame(16'sd13, "+13 data=0D");

        // 超范围截断
        send_frame(16'sd181);
        check_frame(16'sd180, "clamp +181");
        send_frame(-16'sd181);
        check_frame(-16'sd180, "clamp -181");

        // 特殊帧：0x7FFF 开始正弦，0x8000 停止；不应产生普通 valid
        send_frame(16'h7FFF);
        #2_000_000;
        if (!got_sine_start || got_valid) begin
            $display("FAIL sine start: start=%0d valid=%0d", got_sine_start, got_valid);
            errors = errors + 1;
        end else
            $display("PASS sine start frame");
        got_sine_start = 1'b0;
        got_valid = 1'b0;

        send_frame(16'h8000);
        #2_000_000;
        if (!got_sine_stop || got_valid) begin
            $display("FAIL sine stop: stop=%0d valid=%0d", got_sine_stop, got_valid);
            errors = errors + 1;
        end else
            $display("PASS sine stop frame");
        got_sine_stop = 1'b0;
        got_valid = 1'b0;

        if (errors == 0)
            $display("==== ALL TESTS PASSED ====");
        else
            $display("==== %0d TEST(S) FAILED ====", errors);
        $finish;
    end

endmodule
