module adc(
    input           clk_50m,        // 50MHz 板载时钟
    input           rst_n,          // 复位，低有效
    // AD9200 硬件接口
    output          adc_clk,        // ADC 采样时钟输出，12.5MHz
    input   [9:0]   adc_data_in,    // D9~D0 并行数据
    output  [11:0]  theta_out       // 平均后的角度值
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

// 用 adc_clk 下降沿采样 ADC 数据，避开数据变化沿
reg [9:0] adc_data_r;
always @(negedge adc_clk or negedge rst_n) begin
    if (!rst_n)
        adc_data_r <= 10'd0;
    else
        adc_data_r <= adc_data_in;
end

// 将采样数据同步到 clk_50m 域，避免跨时钟域亚稳态
reg [9:0] adc_data_sync;
always @(posedge clk_50m or negedge rst_n) begin
    if (!rst_n)
        adc_data_sync <= 10'd0;
    else
        adc_data_sync <= adc_data_r;
end

// 将 10 位数据扩展为 12 位（直接二进制格式，左移 2 位）
wire [11:0] theta_12bit = {adc_data_sync, 2'b0};

// 输出寄存器与累加变量
reg [11:0] theta_out_reg;
reg [15:0] theta_sum;
reg [17:0] cnt_3ms;
reg [13:0] cnt_187us;
reg [3:0]  cnt_add;      // 累加次数计数（未使用，保留原逻辑）

assign theta_out = theta_out_reg;

// 3ms 计数器：150000 × 20ns = 3ms
always @(posedge clk_50m or negedge rst_n) begin
    if (!rst_n)
        cnt_3ms <= 0;
    else if (cnt_3ms == 149999)
        cnt_3ms <= 0;
    else
        cnt_3ms <= cnt_3ms + 1;
end

// 187.5us 计数器：9375 × 20ns = 187.5us
always @(posedge clk_50m or negedge rst_n) begin
    if (!rst_n)
        cnt_187us <= 0;
    else if (cnt_187us == 9374)
        cnt_187us <= 0;
    else
        cnt_187us <= cnt_187us + 1;
end

// 每 187.5us 累加一次 ADC 数据，每 3ms 求平均并输出
always @(posedge clk_50m or negedge rst_n) begin
    if (!rst_n) begin
        theta_sum     <= 0;
        cnt_add       <= 0;
        theta_out_reg <= 0;
    end else if (cnt_187us == 9373) begin
        theta_sum <= theta_sum + theta_12bit;
        cnt_add   <= cnt_add + 1;
    end else if (cnt_3ms == 149999) begin
        theta_out_reg <= theta_sum >> 4;   // 3ms 内累加 16 次，右移 4 位求平均
        theta_sum     <= 0;
        cnt_add       <= 0;
    end else begin
        theta_out_reg <= theta_out_reg;
        theta_sum     <= theta_sum;
        cnt_add       <= cnt_add;
    end
end

endmodule