# gowin_fpga_inverted_pendulum

基于高云（Gowin）GW2A FPGA 的**旋转式倒立摆（Furuta pendulum）**完整工程：
FPGA 负责全部实时控制（传感采集、自动起摆、双环平衡、旋转指令、正弦摆动、遥测），
ESP32（MicroPython）负责人机交互（ST7789 表盘界面 + 手机网页控制）。

## 目录结构

```
gowin_fpga_inverted_pendulum/
├── GoWin/       # FPGA 工程（Gowin IDE 综合布局布线 + iverilog 仿真）
├── exp32/       # ESP32 MicroPython 端（屏显界面 + 网页控制 + UART 通信）
└── history/     # 旧版本归档（v1_basic）
```

## 系统架构

```
 WDD35D4 角度传感器 ──► ADC（12bit，摆杆倾角 θ）
 11 线编码器 ×4 倍频 ×30 减速 ──► 编码器（摆臂转角 φ，1320 counts/圈）
                                    │
             FPGA (GW2A)            ▼
  ┌───────────────────────────────────────────────────┐
  │ 状态机 S_IDLE / S_SWING / S_BALANCE                │
  │  自动起摆 → 双环平衡 → 旋转指令 / 正弦摆动           │
  │  内环：摆杆倾角环 5 ms（200 Hz）PD + 小积分          │
  │  外环：摆臂转角环 50 ms（20 Hz）PD + 速度/摩擦前馈    │
  │  遥测：100 Hz UART 帧                              │
  └───────────────────────────────────────────────────┘
        │ UART 115200 8N1                     ▲
        ▼                                     │
        ESP32 (MicroPython)
  ST7789 表盘界面 + 手机网页 ──► 旋转角指令
```

## GoWin/ FPGA 工程

### 控制功能

- **自动起摆**：摆杆自然下垂并保持 0.5 s 后自动进入起摆；`swing_up.v` 用
  “起始踢动 + 能量泵 + v_need 限速 + 顶部捕获”把摆杆送上竖直，捕获成功后
  无缝交接给平衡环（交接瞬间外环目标锁存为当前摆臂转角）。
- **双环平衡**：内环摆杆倾角环（5 ms / 200 Hz，PD + 小积分）+ 外环摆臂转角环
  （50 ms / 20 Hz，PD）；`pid_core.v` 为通用 16.16 定点 PID（积分限幅、
  微分低通、停机/轨迹运动时积分清零）。
- **旋转指令**：平衡状态下接收 ESP32 角度命令，`ramp_move.v` 生成梯形速度
  斜坡轨迹（加速 → 匀速 → 减速 → 末端吸附），带 360° 行程限幅、跟踪误差
  与移动超时三重保护，并叠加速度前馈与摩擦前馈。
- **正弦摆动**：拨码开关 C8（低电平 ON）使能，默认幅度 8°、频率 0.5 Hz，
  增益渐入渐出，运动中可用角度命令打断。
- **按键**：K1 启停 / 手动起摆，K2 平衡时锁存当前位置，K3 保留。
- **遥测**：`telemetry.v` 以 100 Hz 上报角度、角度目标、编码器位置、内外环
  PID 输出、占空比与状态 flags（run / swing / in_window / loc_sat /
  angle_sat / move_fault 等）。

### 源文件（src/）

| 文件 | 说明 |
| --- | --- |
| `top.v` | 顶层：状态机、参数、PID 例化、速度/摩擦前馈、电机输出 |
| `adc.v` | 12bit 角度采集（3 ms 滑动平均） |
| `encoder.v` | 编码器同步滤波、四倍频计数（中心值 32768） |
| `pid_core.v` | 通用 16.16 定点 PID（可配微分低通、积分限幅、hold_int 清零） |
| `swing_up.v` | 自动起摆：踢动 + 能量泵 + v_need 限速 + 顶部捕获 |
| `rotate_ctrl.v` | 平衡状态转角控制：斜坡目标、正弦摆动、K2 锁存 |
| `ramp_move.v` | 梯形速度斜坡轨迹生成 + 行程/跟踪/超时保护 |
| `sine_lut.v` | 正弦查表 |
| `angle_cmd.v` | 角度命令解帧（`AA 55 + 16bit + 0D`，超时重同步） |
| `uart_rx.v` / `uart_tx.v` | UART 接收 / 发送 |
| `telemetry.v` | 100 Hz 遥测组帧 |
| `key_filter.v` | 按键消抖 |
| `PWM.v` | 20 kHz PWM 输出（1% 分辨率） |
| `pendulum.cst` | 引脚约束（器件与引脚以实际板卡原理图为准） |

### 串口协议（115200 8N1）

ESP32 → FPGA（仅在平衡状态生效，命令帧 5 字节）：

```
AA 55  ANGLE_H  ANGLE_L  0D
```

`ANGLE` 为有符号 16 bit 角度，单位度，范围 -180~+180，默认大端
（小端可将 `top.v` 的 `CMD_MSB_FIRST` 改为 0）。

FPGA → ESP32：100 Hz 遥测帧（34 字节，含 `angle_raw` / `encoder_pos` /
PID 输出 / flags，CRC16 校验），帧格式见 `exp32/uart.py` 与
`GoWin/src/telemetry.v`。

### 仿真与文档

- `sim/`：iverilog 测试平台（`tb_*.v`）与实测日志（`run_*.log`）；
  `sim/swing/` 含 Python 控制律模型（`model.py`）、v_need LUT 生成脚本
  （`make_vneed_lut.py`）与结果分析脚本。
- `docs/`：功能文档——`swing-up.md`（自动起摆）、`rotate-cmd.md`（旋转命令）、
  `ramp-move.md`（梯形斜坡改造）、`fix-report.md`（PID 修复记录）。
- `tools/`：报告 / 图片生成脚本。
- `figures/`：硬件与仿真图片。
- `backup/`：历次改动前的文件备份（按时间/功能命名）。

编译：Gowin IDE 打开 `pendulum.gprj`（顶层 `top`，约束 `src/pendulum.cst`）；
仿真：iverilog，命令见各 `tb_*.v` 文件头部。

## exp32/ ESP32 端（MicroPython）

| 文件 | 说明 |
| --- | --- |
| `boot.py` | 启动：按内存敏感顺序初始化 UART → WiFi → SPI → HTTP → 主循环 |
| `board.py` | 硬件 SPI 总线（HSPI，20 MHz） |
| `uart.py` | FPGA 遥测帧接收（CRC16 校验，只解析最新帧）+ 旋转指令发送 |
| `ui.py` | ST7789 界面：角度表盘 + 旋转角设置页 |
| `st7789.py` | ST7789 驱动 |
| `vga1_16x32.py` | 16x32 点阵字体 |
| `web.py` | 非阻塞 HTTP 服务（`GET /` 控制页、`GET /s?x=` 下发旋转角） |
| `www/index.html` | 手机控制页面 |
| `wifi.py` | WiFi 连接 |
| `tmp.py` | 主循环：按键扫描、刷屏、UART / HTTP poll |

使用：ESP32 上电后连接 WiFi，屏幕底部显示 IP；手机浏览器打开
`http://<IP>/` 即可在平衡状态下下发旋转角度；屏幕表盘实时显示摆杆角度。

## history/ 历史归档

存放上一版本（`v1_basic`）的 Verilog 源码与说明，仅作归档参考，
不再参与当前工程编译。
