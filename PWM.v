module pwm_20k_1pct (
    input  wire       clk,        // 50MHz 系统唯一时钟
    input  wire       rstn,      // 低电平复位
    input  wire [7:0] pid_duty,   // PID 输出，0~100 整数
    output reg        pwm         // 20kHz PWM 输出
);

    localparam integer PERIOD    = 2500; // 50MHz / 20kHz = 2500
    localparam integer DUTY_STEP = 25;   // 2500 * 1% = 25

    reg  [11:0] cnt;          // 0 ~ 2499
    reg  [7:0]  duty_reg;     // 当前 PWM 周期使用的占空比
    wire [7:0]  duty_lim;     // 限幅到 0~100
    wire [12:0] duty_rate;    // duty_reg * 25，最大 2500

    // PID 输出限幅，防止异常值超过 100
    assign duty_lim = (pid_duty > 8'd100) ? 8'd100 : pid_duty;

    // 高电平阈值：duty=0 -> 0，duty=50 -> 1250，duty=100 -> 2500
    assign duty_rate = duty_reg * DUTY_STEP;

    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            cnt      <= 12'd0;
            duty_reg <= 8'd0;
            pwm      <= 1'b0;
        end
        else begin
            // 20kHz 周期计数：0 ~ 2499
            if (cnt >= PERIOD - 1) begin
                cnt      <= 12'd0;
                duty_reg <= duty_lim;   // 只在周期边界锁存 PID 输出
            end
            else begin
                cnt <= cnt + 1'b1;
            end

            // 产生 PWM
            if (cnt < duty_rate)
                pwm <= 1'b1;
            else
                pwm <= 1'b0;
        end
    end

endmodule