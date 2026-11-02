module adc(
    input           clk_50m,        // 50MHz 板载时钟
    input           rst_n,          // 复位，低有效
    // AD9200 硬件接口
    output          adc_clk,        // ADC 采样时钟输出，12.5MHz
    input   [9:0]   adc_data_in,    // D9~D0 并行数据
    output  [11:0]  theta_out       // 平均后的角度值，更新周期 1ms (1kHz)
);

// 时钟分频：50MHz -> 12.5MHz
reg [1:0] clk_div_cnt;
always @(posedge clk_50m or negedge rst_n) begin
    if (!rst_n)
        clk_div_cnt <= 2'b0;
    else
        clk_div_cnt <= clk_div_cnt + 1'b1;
end
assign adc_clk = clk_div_cnt[1];    // 50MHz / 4 = 12.5MHz

// 用 adc_clk 下降沿采样 ADC 数据
reg [9:0] adc_data_r;
always @(negedge adc_clk or negedge rst_n) begin
    if (!rst_n)
        adc_data_r <= 10'd0;
    else
        adc_data_r <= adc_data_in;
end

// 同步到 clk_50m 域
reg [9:0] adc_data_sync;
always @(posedge clk_50m or negedge rst_n) begin
    if (!rst_n)
        adc_data_sync <= 10'd0;
    else
        adc_data_sync <= adc_data_r;
end

// 将 10 位数据扩展为 12 位（左移 2 位）
wire [11:0] theta_12bit = {adc_data_sync, 2'b0};

// 输出寄存器与累加变量
reg [11:0] theta_out_reg;
reg [15:0] theta_sum;
reg [16:0] cnt_1ms;      // 1ms 计数器：50000 × 20ns = 1ms
reg [11:0] cnt_62_5us;   // 62.5us 计数器：3125 × 20ns = 62.5us
reg [3:0]  cnt_add;      // 累加次数计数（保留，未使用）

assign theta_out = theta_out_reg;

// 1ms 计数器 (0 ~ 49999)
always @(posedge clk_50m or negedge rst_n) begin
    if (!rst_n)
        cnt_1ms <= 0;
    else if (cnt_1ms == 17'd49999)
        cnt_1ms <= 0;
    else
        cnt_1ms <= cnt_1ms + 1;
end

// 62.5us 计数器 (0 ~ 3124)
always @(posedge clk_50m or negedge rst_n) begin
    if (!rst_n)
        cnt_62_5us <= 0;
    else if (cnt_62_5us == 12'd3124)
        cnt_62_5us <= 0;
    else
        cnt_62_5us <= cnt_62_5us + 1;
end

// 每 62.5us 累加一次 ADC 数据，每 1ms 求平均并输出
always @(posedge clk_50m or negedge rst_n) begin
    if (!rst_n) begin
        theta_sum     <= 0;
        cnt_add       <= 0;
        theta_out_reg <= 0;
    end else if (cnt_62_5us == 12'd3123) begin   // 62.5us 累加（提前一时钟）
        theta_sum <= theta_sum + theta_12bit;
        cnt_add   <= cnt_add + 1;
    end else if (cnt_1ms == 17'd49999) begin     // 1ms 输出
        theta_out_reg <= theta_sum >> 4;        // 1ms 内累加 16 次，右移 4 位求平均
        theta_sum     <= 0;
        cnt_add       <= 0;
    end else begin
        theta_out_reg <= theta_out_reg;
        theta_sum     <= theta_sum;
        cnt_add       <= cnt_add;
    end
end

endmodule