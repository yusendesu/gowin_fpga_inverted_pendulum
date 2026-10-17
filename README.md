# gowin_fpga_inverted_pendulum
使用高云的GW2A-18的Verilog HDL一阶倒立摆控制程序/Control program for a first-order inverted pendulum using GoWin's GW2A-18

有关key_filter说明：
key_filter并非将输入的开关波形滤波，而是检测到一个有效按下信号后产生一个极窄（20ns）的脉冲，详见key_filter_sim.jpg
