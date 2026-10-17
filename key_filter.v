module key_filter #(

	parameter CNT_MAX = 20'd1000_000
	
)(

	input		sys_clk		,	
	input		sys_rst_n	,	
	input		key_in		,	
	
	output	reg	key_flag

);

//============================================================
// 1. 异步按键输入两级同步
//============================================================
reg key_in_d0;
reg key_in_d1;

always @(posedge sys_clk or negedge sys_rst_n) begin
	if(sys_rst_n == 1'b0) begin
		key_in_d0 <= 1'b1;
		key_in_d1 <= 1'b1;
	end
	else begin
		key_in_d0 <= key_in;
		key_in_d1 <= key_in_d0;
	end
end

wire key_in_sync = key_in_d1;

//============================================================
// 2. 消抖计数器
//    按键低有效：key_in_sync == 1 时清零
//    持续为 0 时计数，到 CNT_MAX 后保持
//============================================================
reg [19:0] cnt_20ms;

always @(posedge sys_clk or negedge sys_rst_n) begin
	if(sys_rst_n == 1'b0)
		cnt_20ms <= 20'd0;
	else if(key_in_sync == 1'b1)
		cnt_20ms <= 20'd0;
	else if(cnt_20ms == CNT_MAX)
		cnt_20ms <= cnt_20ms;
	else
		cnt_20ms <= cnt_20ms + 1'b1;
end

//============================================================
// 3. 稳定按下后输出一个 sys_clk 周期脉冲
//============================================================
always @(posedge sys_clk or negedge sys_rst_n) begin
	if(sys_rst_n == 1'b0)
		key_flag <= 1'b0;
	else if(cnt_20ms == CNT_MAX - 1'b1 && key_in_sync == 1'b0)
		key_flag <= 1'b1;
	else
		key_flag <= 1'b0;
end

endmodule