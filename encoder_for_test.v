//该文件仅用于测试编码器模块的质量，请勿直接加入项目中！
module encoder_for_test (
    input           clk_50m,
    input           rst_n,
    input           enc_a, enc_b,
    output  [15:0]  pos,
    output          a_phase_final,
    output          b_phase_final
);

reg [1:0] enc_sync_a, enc_sync_b;
reg [1:0] state_now, state_last;
reg [1:0] state_filt;
reg [15:0] filter_cnt;
reg [15:0] pos_reg;
reg        init_done;

assign pos = pos_reg;
//将处理后的A相和B相的信号输出到a_phase_final和b_phase_final上
assign a_phase_final=state_filt[1];
assign b_phase_final=state_filt[0];
parameter FILTER_CNT = 500;  // 连续稳定 500 个周期才确认，约 10us @50MHz

// 两级同步，防亚稳态
always @(posedge clk_50m or negedge rst_n) begin
    if(!rst_n) begin
        enc_sync_a <= 2'd0;
        enc_sync_b <= 2'd0;
        state_now  <= 2'd0;
    end else begin
        enc_sync_a <= {enc_sync_a[0], enc_a};
        enc_sync_b <= {enc_sync_b[0], enc_b};
        state_now  <= {enc_sync_a[1], enc_sync_b[1]};
    end
end

// 数字滤波：连续 FILTER_CNT 个周期 state_now 与 state_filt 不同才更新
always @(posedge clk_50m or negedge rst_n) begin
    if(!rst_n) begin
        state_filt <= 2'b00;
        filter_cnt <= 16'd0;
    end else begin
        if(state_now == state_filt) begin
            filter_cnt <= 16'd0;
        end else if(filter_cnt >= FILTER_CNT - 1) begin
            state_filt <= state_now;
            filter_cnt <= 16'd0;
        end else begin
            filter_cnt <= filter_cnt + 1'b1;
        end
    end
end

//四倍频方向判断，使用滤波后的 state_filt
always @(posedge clk_50m or negedge rst_n) begin
    if(!rst_n) begin
        pos_reg    <= 16'd32768;  // 中心偏置
        state_last <= 2'b00;
        init_done  <= 1'b0;
    end else begin
        if(!init_done) begin
            // 等待第一次滤波状态稳定，将 state_last 同步为 state_filt，不计数
            state_last <= state_filt;
            if(state_now == state_filt && filter_cnt == 0)
                init_done <= 1'b1;
        end else begin
            state_last <= state_filt;
            case({state_last, state_filt})
                4'b00_01: pos_reg <= pos_reg - 1;
                4'b01_11: pos_reg <= pos_reg - 1;
                4'b11_10: pos_reg <= pos_reg - 1;
                4'b10_00: pos_reg <= pos_reg - 1;

                4'b00_10: pos_reg <= pos_reg + 1;
                4'b10_11: pos_reg <= pos_reg + 1;
                4'b11_01: pos_reg <= pos_reg + 1;
                4'b01_00: pos_reg <= pos_reg + 1;

                default: pos_reg <= pos_reg;
            endcase
        end
    end
end

endmodule