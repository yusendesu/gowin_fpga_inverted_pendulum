# ESP32 角度旋转命令（发挥部分）

> 目标：倒立摆保持平衡的同时，按 ESP32 的指令顺时针/逆时针旋转指定角度。
> 命令只改变**摆臂转角环的目标**，平衡环（摆杆角度环 + 摆臂转角环）结构不变。
>
> **本次改动**：增量移动从“位置误差 S 曲线 + PD”改为**梯形速度斜坡**
> （`src/ramp_move.v`），并加入总行程、跟踪误差、移动时间三重保护。
> 详细分析和整定见 [ramp-move.md](ramp-move.md)。

新增/修改文件：

- `src/uart_rx.v`：8N1 UART 接收
- `src/angle_cmd.v`：`AA 55 | ANGLE_H ANGLE_L | 0D` 解帧
- `src/ramp_move.v`：**新增**，梯形速度斜坡、增量合并、安全保护
- `src/rotate_ctrl.v`：调用 `ramp_move`，保留正弦摆动
- `src/top.v`、`pendulum.gprj`：参数和例化

---

## 1. 帧格式

ESP32 → FPGA，5 字节：

```text
0xAA  0x55  ANGLE_H  ANGLE_L  0x0D
```

| 字段 | 说明 |
|---|---|
| `AA 55` | 帧头 |
| `ANGLE_H` `ANGLE_L` | 有符号 16 bit，单位度，-180~+180 |
| `0D` | 帧尾 |

默认 **大端**（高字节先到），与 `interfacetest/src/uart.v` 的发送顺序一致。
如果 ESP32 发的是小端，把 `top.v` 的 `CMD_MSB_FIRST` 改成 `1'b0`。

示例：

| 角度 | 字节 |
|---:|---|
| +90° | `AA 55 00 5A 0D` |
| -90° | `AA 55 FF A6 0D` |
| +180° | `AA 55 00 B4 0D` |
| -180° | `AA 55 FF 4C 0D` |
| 0° | `AA 55 00 00 0D` |

波特率 **115200**，与 FPGA 遥测 TX 一致；ESP32 用同一个 UART 即可
（TX/RX 都是 115200）。如果以后要改回 9600，改 `top.v` 的 `UART_RX_BPS`。

解帧规则：

- 帧头/帧尾之外的字节不检查，因此数据字节可以是 `AA`、`55`、`0D`；
- 帧尾不是 `0D` 时，会把当前字节当作可能的 `AA` 重新同步；
- 帧内字节超过 20 ms 未到齐，状态机复位（防止丢字节后卡死）；
- 角度超过 ±180 时按 ±180 截断。

---

## 2. 旋转行为

### 2.1 一帧 = 一次相对增量

收到一帧且当前处于 `S_BALANCE` 时：

```text
空闲/已停稳：目标终点 = 当前实际摆臂转角 Location + 命令角度对应 counts
正在移动  ：目标终点 = 当前目标 + 命令角度对应 counts
```

然后 `ramp_move` 生成一条**梯形速度曲线**的位置目标 `loc_target`
（加速 → 匀速 → 按剩余距离减速 → 末端吸附），位置环 PID 跟踪这条轨迹；
同时叠加**速度前馈**和**摩擦前馈**：

```text
loc_cmd = loc_pid_out + clamp(KV_FF * v_ref, ±FF_LIMIT) + Ff
```

几个约定：

- **一帧 = 一次相对增量**。先发 +90，等停稳再发 +90，会再转 90；
- 如果上一帧还没走完就收到新帧，新角度**累加**到当前目标，速度曲线不中断；
- 一次连续移动的总行程限制在 `RAMP_MAX_TRAVEL_DEG`（默认 360°）内；
- 只有在 `S_BALANCE` 时命令生效；`S_IDLE` / `S_SWING` 时收到的命令直接丢弃；
- 轨迹收敛后外环会保持新的摆臂转角，不会自己转回去。

例：

```text
先发 +20°，100 ms 后再发 +40°  =>  最终转 60°
```

### 2.2 为什么改成斜坡

旧版增量路径虽然有一段 S 曲线，但轨迹仍由位置误差 PD 生成，而且每帧都从
当前实际位置重新起步、把速度清零。连续发帧时轨迹被反复打断，速度前馈一跳一跳；
再加上没有行程和跟踪误差上限，外环一旦跟不上，目标会继续往前推，表现为
“越转越快，最后转飞”。

新结构把**轨迹生成**和**外环 PID**分开：

- 轨迹：`ramp_move` 自己按加速度/速度/剩余距离走，输出有界、可预测；
- PID：只跟踪轨迹，不再决定轨迹形状；
- 轨迹走完 `rotating` 回 0，外环积分从 0 重新开始。

### 2.3 轨迹运动期间清积分

`rotate_ctrl` 的 `rotating` 在斜坡运动期间为 1：

- `pid_core` 的 `hold_int` 上升沿**立即把积分清 0**；
- 运动期间积分保持 0，P/D 和前馈正常工作；
- 轨迹收敛、`rotating` 回 0 后，积分从 0 重新积分。

`top.v` 用 `rotating` 驱动两个 PID 的 `hold_int`
（若想临时对比旧行为，可把 `HOLD_INT_ON_ROTATE` 设为 `1'b0`）。

### 2.4 K2 锁存当前位置

平衡时按一下 K2：

```text
loc_target <= 当前 Location
```

位置环立刻把当前位置当作新的平衡目标，之后 PID 保持这个点。
锁存只改目标，不清积分；`S_IDLE` / `S_SWING` 时按 K2 被忽略。
K2 需要按键低有效、上拉，并在 `pendulum.cst` 中补 `IO_LOC`。

### 2.5 正弦摆动（拨码开关 C8）

拨码开关 **C8，低电平有效，电平使能**：

- 拨到 ON（C8 接地，`sine_en=1`）→ 持续允许正弦；
- 拨到 OFF（C8 上拉为高，`sine_en=0`）→ 停止并渐出；
- 进入平衡时如果开关已经 ON，会自动开始；
- 10 ms 去抖；
- `sine_en` 只在“允许且尚未运行”时启动一次，不会每个时钟复位相位/增益；
- 正弦中心 = 当前斜坡目标（K2 锁存的平衡位置）；
- 幅度 `SINE_AMP_DEG`（`top.v` 默认 8°）；
- 频率 `SINE_FREQ_MILLIHZ`（默认 500 = 0.5 Hz）；
- 增益 0→1 渐入，约 0.5 s，避免起步冲击；
- 同时输出 `v_ref` 给速度前馈。

`rotate_ctrl` 的 `sine_on` 在正弦进行中（含渐出）为 1，`top.v` 把它送到
`led_sine`（**B16，板上 LED1**，高电平点亮），停止并渐出结束后熄灭；
原来 `led`（C14）仍是“起摆/平衡运行”指示，两者互不影响。

**命令与正弦的优先级**：收到普通角度命令会退出正弦，并置 `sine_inhibit`。
此时即使拨码还 ON，正弦也不会立刻重新启动；必须先把拨码关一次再开。
这样命令不会被电平开关覆盖。

停止时：

- 增益 1→0 渐出，约 0.5 s；
- 回到正弦中心并保持；
- `S_IDLE` / `S_SWING` 时拨码开关不会启动正弦。

命令角度 -> counts：

```text
counts = angle_deg * ARM_COUNTS_PER_REV / 360
```

用 Q16 定点实现（`COUNTS_PER_DEG_Q16`），并向上取整，减少低分辨率时的
截断误差。1320 counts/圈时：90° = 330 counts，45° = 165 counts，
180° = 660 counts。

---

## 3. 增量移动的安全保护

### 3.1 总行程限幅 `RAMP_MAX_TRAVEL_DEG`

默认 360°。从“停稳”到“再次停稳”算一段，目标被夹在：

```text
[base - MAX_TRAVEL, base + MAX_TRAVEL]
```

- 单帧最大 ±180°，正常一帧不会触发；
- ESP32 连续刷帧时，最多再转一圈就停住，不会无限累积；
- 停稳后再发命令，会以新的实际位置为 `base` 开始新的一段；
- 想允许更大行程，改 `RAMP_MAX_TRAVEL_DEG`（建议不超过 3000°，
  避免 Q16 32 位溢出）。

### 3.2 跟踪误差保护 `RAMP_TRACK_ERR_COUNTS` / `RAMP_TRACK_TIMEOUT_MS`

每个 1 ms 检查：

```text
|轨迹目标 - 实际位置| > RAMP_TRACK_ERR_COUNTS ?
```

- 默认 200 counts ≈ 54°；
- 超差时**暂停轨迹推进**，并把速度前馈清零，让外环先追；
- 连续超差 500 ms，说明外环确实拉不住：
  - `target`、`goal`、`base` 全部拉回当前实际位置；
  - 速度清零；
  - `fault` 置 1（保持到下一次命令/装载）。
- 这是防止“目标一直往前跑、外环永远追不上”的最后一道闸。

### 3.3 移动超时 `RAMP_MOVE_TIMEOUT_MS`

默认 20 s。按默认 27°/s，转满 360° 约 13.3 s，20 s 有足够余量。
如果电机堵转、编码器不动、外环失效，超时后同样把目标拉回实际位置并置 `fault`。

`fault` 已接到遥测帧的 `flags` 字节 **bit5**（`move_fault`），
正常为 0，触发后保持到下一次角度命令或重新进入平衡。串口助手看到
`loc_target` 突然回到实际位置时，先看这一位。

### 3.4 速度前馈限幅 `FF_LIMIT`

```text
loc_ff_v = clamp((KV_FF * v_ref) >> 16, ±FF_LIMIT)
```

默认 `FF_LIMIT = 12` 个摆杆倾角 ADC counts。即使 `KV_FF` 被误设得很大，
前馈也不会把摆杆压到很大的角度。摩擦前馈 `FRIC_FF` 仍然单独叠加。

---

## 4. 参数（`src/top.v` 的 module 参数）

| 参数 | 默认 | 说明 |
|---|---:|---|
| `UART_RX_BPS` | 115200 | ESP32 发送波特率（与遥测 TX 一致） |
| `ARM_COUNTS_PER_REV` | 1320 | 摆臂转一圈的编码器 counts（11×4×30） |
| `RAMP_V_MAX` | 6554 | 斜坡最大速度，Q16 counts/ms（0.10 count/ms ≈ 27°/s） |
| `RAMP_A_MAX` | 66 | 斜坡加速度，Q16 counts/ms²（≈1000 counts/s²） |
| `RAMP_SETTLE_COUNTS` | 2 | 末端吸附距离（counts） |
| `RAMP_SETTLE_V` | 328 | 末端吸附速度（0.005 count/ms） |
| `RAMP_MAX_TRAVEL_DEG` | 360 | 一次连续增量最大行程 |
| `RAMP_TRACK_ERR_COUNTS` | 200 | 跟踪超差门限（counts，约 54°） |
| `RAMP_TRACK_TIMEOUT_MS` | 500 | 连续超差多久后放弃本次移动 |
| `RAMP_MOVE_TIMEOUT_MS` | 20000 | 单次移动绝对超时 |
| `SINE_AMP_DEG` | 8 | 正弦幅度（度） |
| `SINE_FREQ_MILLIHZ` | 500 | 正弦频率（毫赫兹，500=0.5 Hz） |
| `SINE_GAIN_STEP` | 66 | 正弦渐入/渐出增益步长，约 1 s |
| `KV_FF` | 100 | 速度前馈，Q16 倾角 counts / (count/ms) |
| `FF_LIMIT` | 12 | 速度前馈输出限幅（倾角 counts） |
| `FRIC_FF` | 4 | 摩擦前馈倾角 counts |
| `FRIC_V_THRESH` | 328 | 摩擦前馈速度门限，Q16 |
| `FRIC_ERR_THRESH` | 2 | 静止摩擦前馈误差门限，counts |
| `CMD_MSB_FIRST` | 1 | 1 = 大端，0 = 小端 |
| `ROTATE_SIGN` | 1 | 1 = 正角度使 `Location` 增大；方向相反时改 0 |

### 4.1 `ARM_COUNTS_PER_REV` 怎么定

编码器 11 线，四倍频 = 44 counts/编码器圈；电机 30 减速比，所以摆臂转一圈：

```text
44 × 30 = 1320 counts
```

`top.v` 里 `ARM_COUNTS_PER_REV = 1320` 已经按这个值设置。如果以后换电机/
减速比，按 `44 × N` 修改。

不确定时用串口遥测标定：记下旋转前的 `encoder_pos`，发一帧 +180°，转完后
再读 `encoder_pos`，则

```text
ARM_COUNTS_PER_REV ≈ 2 × |Δencoder_pos|
```

（因为 180° 是半圈；注意 16 bit 回绕。）

### 4.2 斜坡参数怎么调

1. **先确认方向**：发 +90°，摆臂应朝你定义的顺时针转；反了改 `ROTATE_SIGN`。
2. **只调 `RAMP_V_MAX`**：从 3277（13.6°/s）开始，确认能平稳转 90°，再逐步加到 6554。
3. **再调 `RAMP_A_MAX`**：默认 66 比较温和；想更跟手可加到 131，但起步/停止冲击更大。
4. **最后调 `KV_FF`**：先设 0，确认纯位置环能转到位；再从 50 开始加。
   加上前馈反而更容易倒，说明符号不对或太大，先回 0。
5. **不要一上来改大 `RAMP_MAX_TRAVEL_DEG`**，先把 360° 内调稳。

### 4.3 方向

`ROTATE_SIGN = 1` 时，正角度让 `Location` 增大。实际“顺时针”是哪个方向取决于
编码器 A/B 相序和机械安装：

- 发 +90，摆臂转向你定义的顺时针 → 保持 1；
- 转向相反 → 把 `ROTATE_SIGN` 改成 `1'b0`（不要同时改 `in1/in2`，否则平衡环
  方向也会被破坏）。

---

## 5. 验证

### 5.1 解帧（`tb_uart_cmd.v`）

115200 bps 发送多种帧，覆盖数据字节为 `AA`/`55`/`0D` 和超范围截断：

```powershell
iverilog -g2005 -o sim\tb_uart_cmd.vvp -s tb_uart_cmd `
  sim\tb_uart_cmd.v src\uart_rx.v src\angle_cmd.v
vvp sim\tb_uart_cmd.vvp
```

实测（`sim/swing/run_uart_cmd.log`）：

```text
PASS [+90]: angle=90
PASS [-90]: angle=-90
PASS [-86 data=AA]: angle=-86
PASS [+85 data=55]: angle=85
PASS [+13 data=0D]: angle=13
PASS [clamp +181]: angle=180
PASS [clamp -181]: angle=-180
PASS sine start frame
PASS sine stop frame
==== ALL TESTS PASSED ====
```

### 5.2 斜坡单元（`tb_ramp_move.v`）

1 MHz 时钟，1320 counts/rev，验证速度上限、无过冲、命令累加、行程限幅、
跟踪保护：

```powershell
iverilog -g2005 -o sim\tb_ramp_move.vvp -s tb_ramp_move `
  sim\tb_ramp_move.v src\ramp_move.v
vvp sim\tb_ramp_move.vvp
```

实测（`sim/swing/run_ramp.log`）：

```text
PASS load: target=0
PASS [+90 ramp]: target=330 after 3366 ms
PASS [+90 settle]: rotating=0 v_ref=0
PASS +90: |v_ref|max=6554
PASS +90: no overshoot (max_target=330)
PASS [-45 ramp]: target=165 after 1716 ms
PASS [accumulate +20/+40]: target=220 after 2166 ms
PASS [travel clamp 360deg]: target=1320 after 13247 ms
PASS track guard: fault=1 target=0
==== ALL TESTS PASSED ====
```

### 5.3 斜坡 + 正弦（`tb_rotate.v`）

```powershell
iverilog -g2005 -o sim\tb_rotate.vvp -s tb_rotate `
  sim\tb_rotate.v src\rotate_ctrl.v src\ramp_move.v src\sine_lut.v
vvp sim\tb_rotate.vvp
```

实测（`sim/swing/run_rotate.log`）：

```text
PASS load: target=1000
PASS [+90 ramp]: target=1330 after 3366 ms
PASS [-45 ramp]: target=1165 after 1753 ms
PASS lock current position: target=1200
PASS sine start: target=1237
PASS sine stop: target=1200
PASS ignore while not balancing
==== ALL TESTS PASSED ====
```

### 5.4 top 级联（`tb_rotate_top.v`）

在完整 top 中强制 `S_BALANCE`，通过 `fpga_rx` 发帧，检查
`angle_cmd -> rotate_ctrl -> loc_target`。测试用快速斜坡参数缩短仿真：

```powershell
iverilog -g2005 -o sim\tb_rotate_top.vvp -s tb_rotate_top `
  sim\tb_rotate_top.v src\PWM.v src\adc.v src\encoder.v src\key_filter.v `
  src\pid_core.v src\swing_up.v src\uart_rx.v src\angle_cmd.v `
  src\rotate_ctrl.v src\ramp_move.v src\sine_lut.v src\telemetry.v `
  src\top.v src\uart_tx.v
vvp sim\tb_rotate_top.vvp
```

实测（`sim/swing/run_rotate_top.log`）：

```text
PASS load: loc_target=0
PASS [+90 command]: loc_target=330 after 34 ms
PASS [-45 command]: loc_target=-165 after 18 ms
==== ALL TESTS PASSED ====
```

### 5.5 PID 积分清零（`tb_pid.v`）

```powershell
iverilog -g2005 -o sim\tb_pid.vvp -s tb_pid sim\tb_pid.v src\pid_core.v
vvp sim\tb_pid.vvp
```

实测（`sim/swing/run_pid.log`）：

```text
PASS clear_int on hold_int rise: err_int=0
PASS hold_int: err_int=0
PASS release hold: err_int=32768000
PASS D on measurement: target step D kick = 0
```

---

## 6. 上机调试顺序

1. 先用串口助手/ESP32 发一帧 `AA 55 00 5A 0D`（+90°），确认 `fpga_rx`
   接线正确、波特率匹配。
2. 让系统进入平衡（自动起摆或 K1 手动），再发命令。命令只在平衡时生效。
3. 观察摆臂转动方向：
   - 方向相反 → 改 `ROTATE_SIGN`；
   - 转动角度明显不对 → 标定 `ARM_COUNTS_PER_REV`；
   - 转得太猛/摆杆倒 → 降低 `RAMP_V_MAX` / `RAMP_A_MAX`，或降 `LOC_KP`/`LOC_KD`。
4. 确认一帧只转一次：转完后不要重复发送同一帧，否则会继续叠加。
5. 如果 ESP32 需要在旋转完成后收到确认，目前协议没有回包；可以读遥测里的
   `loc_error`/`encoder_pos` 判断是否到位，或后续再扩展协议。
6. 如果转到一半 `loc_target` 突然回到实际位置：跟踪保护触发。先看
   `loc_error`、`loc_pid_out`、`loc_sat`，通常是外环跟不上或方向不对；
   先把 `RAMP_V_MAX` 降到 3277 再试。详见 [ramp-move.md](ramp-move.md)。

---

## 7. 文件清单

| 文件 | 说明 |
|---|---|
| `src/uart_rx.v` | 8N1 UART 接收 |
| `src/angle_cmd.v` | 5 字节帧解析、超时重同步、±180 截断 |
| `src/ramp_move.v` | **新增**：梯形速度斜坡、增量合并、行程/跟踪/超时保护 |
| `src/rotate_ctrl.v` | 调用 `ramp_move`；正弦摆动、K2 锁存 |
| `src/sine_lut.v` | 256 点正弦 LUT |
| `src/top.v` | 例化上面模块；位置环 + 速度/摩擦前馈；参数 |
| `pendulum.gprj` | 加入 `ramp_move.v` |
| `sim/tb_uart_cmd.v` | 解帧测试 |
| `sim/tb_ramp_move.v` | **新增**：斜坡/累加/限幅/跟踪保护测试 |
| `sim/tb_rotate.v` | 斜坡 + 正弦 + 方向 + 锁存测试 |
| `sim/tb_rotate_top.v` | top 级联测试 |
| `sim/tb_sine_amp.v` | 正弦幅值回归 |
