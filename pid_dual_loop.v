module pid_dual_loop #(
    //=========== 位置环(外环)参数 ===========
    parameter   KP_POS  = 16'sd10 ,
    parameter   KI_POS  = 16'sd2  ,
    parameter   KD_POS  = 16'sd30 ,
    //=========== 角度环(内环)参数 ===========
    parameter   KP_ANG  = 16'sd40 ,
    parameter   KI_ANG  = 16'sd3  ,
    parameter   KD_ANG  = 16'sd25 ,
    //=========== 控制周期参数 ===========
    parameter   CNT_CYCLE   = 32'd49_999
)(
    input               clk_50m ,
    input               rst_n   ,
    input               pid_en  ,
    input   [15:0]      pos_fb  ,
    input   [15:0]      pos_set ,
    input   [11:0]      theta_fb,
    output  reg [11:0]  theta_set,
    output  reg [1:0]   motor_sel,
    output  reg [7:0]   duty_out
);

//=========== 控制周期定时 ===========
reg [31:0]  cnt_ctl;
reg         ctl_pulse;
always @(posedge clk_50m or negedge rst_n) begin
    if(!rst_n) begin
        cnt_ctl <= 32'd0;
        ctl_pulse <= 1'b0;
    end else begin
        if(cnt_ctl == CNT_CYCLE) begin
            cnt_ctl <= 32'd0;
            ctl_pulse <= 1'b1;
        end else begin
            cnt_ctl <= cnt_ctl + 32'd1;
            ctl_pulse <= 1'b0;
        end
    end
end

//=========== 内部变量 ===========
reg signed [15:0] err_pos, err_pos1, err_pos2;
reg signed [15:0] err_ang, err_ang1, err_ang2;
reg signed [31:0] dkp_pos, dki_pos, dkd_pos;
reg signed [31:0] u_inc_pos, accu_pos;
reg signed [31:0] dkp_ang, dki_ang, dkd_ang;
reg signed [31:0] u_inc_ang, accu_ang;
reg signed [31:0] abs_accu_ang;

// 限幅常量
localparam signed [15:0] POS_MAX_OFFSET = 16'sd200;
localparam signed [15:0] POS_MIN_OFFSET = -16'sd200;
localparam signed [31:0] ANG_MAX_OUT = 32'sd255;
localparam signed [31:0] ANG_MIN_OUT = -32'sd255;

//=========== 主控制算法（阻塞赋值，保证顺序）===========
always @(posedge clk_50m or negedge rst_n) begin
    if(!rst_n) begin
        err_pos  <= 'd0; err_pos1 <= 'd0; err_pos2 <= 'd0;
        err_ang  <= 'd0; err_ang1 <= 'd0; err_ang2 <= 'd0;
        dkp_pos  <= 'd0; dki_pos  <= 'd0; dkd_pos  <= 'd0;
        dkp_ang  <= 'd0; dki_ang  <= 'd0; dkd_ang  <= 'd0;
        u_inc_pos<= 'd0; u_inc_ang<= 'd0;
        accu_pos <= 'd0; accu_ang <= 'd0;
        theta_set <= 12'd2048;
        motor_sel <= 2'b00;
        duty_out  <= 8'd0;
    end else if(ctl_pulse && pid_en) begin
        // -------- 位置环 --------
        // 计算当前误差（组合计算，立即使用）
        err_pos2 = err_pos1;
        err_pos1 = err_pos;
        err_pos  = $signed({1'b0, pos_set}) - $signed({1'b0, pos_fb});

        // 增量式PID三项
        dkp_pos = (err_pos - err_pos1) * KP_POS;
        dki_pos = err_pos * KI_POS;
        dkd_pos = (err_pos - 2*err_pos1 + err_pos2) * KD_POS;
        u_inc_pos = dkp_pos + dki_pos + dkd_pos;
        accu_pos = accu_pos + u_inc_pos;

        // 位置环输出限幅（角度偏移）
        if(accu_pos > POS_MAX_OFFSET) accu_pos = POS_MAX_OFFSET;
        else if(accu_pos < POS_MIN_OFFSET) accu_pos = POS_MIN_OFFSET;

        // 计算角度目标
        theta_set = 12'd2048 + accu_pos[11:0]; // 由于限幅在±200，安全

        // -------- 角度环 --------
        // 更新误差历史
        err_ang2 = err_ang1;
        err_ang1 = err_ang;
        err_ang  = $signed({1'b0, theta_set}) - $signed({1'b0, theta_fb});

        // 增量式PID三项
        dkp_ang = (err_ang - err_ang1) * KP_ANG;
        dki_ang = err_ang * KI_ANG;
        dkd_ang = (err_ang - 2*err_ang1 + err_ang2) * KD_ANG;
        u_inc_ang = dkp_ang + dki_ang + dkd_ang;
        accu_ang = accu_ang + u_inc_ang;

        // 角度环输出限幅（饱和）
        if(accu_ang > ANG_MAX_OUT) accu_ang = ANG_MAX_OUT;
        else if(accu_ang < ANG_MIN_OUT) accu_ang = ANG_MIN_OUT;

        // 占空比和方向
        if(accu_ang > 0) begin
            motor_sel = 2'b01; // CW
            duty_out = (accu_ang[7:0] > 8'd255) ? 8'd255 : accu_ang[7:0];
        end else if(accu_ang < 0) begin
            motor_sel = 2'b10; // CCW
            abs_accu_ang = ~accu_ang + 1;                    // 绝对值
            duty_out = (abs_accu_ang > 8'd255) ? 8'd255 : abs_accu_ang[7:0];
        end else begin
            motor_sel = 2'b00; // Stop
            duty_out = 8'd0;
        end
    end
end

endmodule