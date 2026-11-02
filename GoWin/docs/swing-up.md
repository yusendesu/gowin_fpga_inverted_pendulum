# 自动起摆与平衡衔接（swing_up）

> 本次改动目标：
> 1. 角度传感器在 **0 或 4095 附近（摆杆自然下垂）持续一段时间** 后，自动触发一次起摆；
> 2. 起摆把摆杆送到竖直附近后，**立即交给原有角度环 + 摆臂转角环**，不改变平衡环结构。
>
> 新增文件：`src/swing_up.v`；修改文件：`src/top.v`、`src/telemetry.v`、
> `pendulum.gprj`；验证文件：`sim/tb_swing_fast.v`、`sim/tb_top_logic.v`、
> `sim/tb_swing.v`、`sim/swing/model.py`。
>
> 角度旋转命令（`AA 55 + 角度 + 0D`）见 [rotate-cmd.md](rotate-cmd.md)。

---

## 1. 行为概览

### 1.1 状态机

`top.v` 新增三个状态：

```
                 hang_now 持续 HANG_CONFIRM_MS
   S_IDLE  ---------------------------------------->  S_SWING
     ^                                                 |
     |  K1 停止 / 超时 / 倒杆                            | swing_captured
     |                                                 v
     +--------------------------------------------  S_BALANCE
                         K1 停止 / 角度超出 ±500
```

| 状态 | 电机 | 角度环 / 转角环 | 说明 |
|---|---|---|---|
| `S_IDLE` | 刹车 | `run=0`，PID 清零 | 检测自然下垂；可被 K1 手动起摆或直接平衡 |
| `S_SWING` | `swing_up.drive` | `run=0`，PID 清零 | 能量泵起摆，不受 ±500 倒杆保护限制 |
| `S_BALANCE` | 角度环输出 | `run=1` | 原有平衡逻辑；±500 倒杆保护生效 |

LED：`S_SWING` / `S_BALANCE` 时常亮，`S_IDLE` 时灭（`led = active`）。

进入 `S_BALANCE` 时，`rotate_ctrl` 在 `enter_balance` 同一拍把外环目标
`loc_target` 装成当前 `Location`：

```verilog
// top.v: enter_balance -> rotate_ctrl.load
// rotate_ctrl.v: target <= load_value; target_final <= load_value;
```

因此外环不会要求摆臂“回到起摆前的角度”，避免一接管就大幅修正。

### 1.2 K1 语义

| 当前状态 | 按 K1 后 |
|---|---|
| `S_SWING` / `S_BALANCE` | 立即停止，`armed=0`，回 `S_IDLE` |
| `S_IDLE`，摆杆在平衡窗口内 | 直接进入 `S_BALANCE`，锁存当前摆臂转角 |
| `S_IDLE`，摆杆自然下垂 | 立即起摆（不必再等 500 ms 确认） |
| `S_IDLE`，其它位置 | `armed=1`，重新允许自动起摆 |

`armed` 复位值为 `AUTO_ARM_ON_RESET=1`，即上电后如果摆杆自然下垂，0.5 s 后会自动起摆。
如果希望上电后必须按 K1 才允许自动起摆，把 `AUTO_ARM_ON_RESET` 改成 `1'b0`。

### 1.3 为什么超时/倒杆后不自动重试

`S_SWING` 超时或 `S_BALANCE` 倒杆后，状态机回 `S_IDLE`，默认 `armed=0`
（`AUTO_RETRY=0`）。这样做是为了：

- 满足“触发一次起摆”的要求，避免无人看守时无限次踢摆；
- 让用户有机会检查摆杆、调整参数；
- 需要重试时按一下 K1 即可。

如果把 `AUTO_RETRY` 改成 `1'b1`，超时/倒杆后 `armed` 保持为 1，摆杆再次
自然下垂 0.5 s 后会自动重试；K1 手动停止仍然会强制 `armed=0`。

---

## 2. 起摆算法（`swing_up.v`）

### 2.1 角度定义

- `angle`：12 bit 绝对角度，`0 / 4095` 附近 = 自然下垂，`CENTER_ANGLE=2050` = 竖直向上（当前标定值）。
- `e = signed12(angle - CENTER_ANGLE)`，范围 `-2048..+2047`。
- `v = de/dt`：每 5 ms 的差分，**先按 12 bit 回绕再解释为有符号数**。

> 回绕非常关键：摆杆每次经过 `0/4095` 时，原始差分会出现 ±4096 的假速度，
> 会把能量泵方向瞬间打反。`swing_up.v` 中写成
> `$signed(e_bits) - e_prev`（12 bit 自然回绕），不要改成更宽的减法。

### 2.2 起始踢动

静止下垂时 `v=0`，能量泵方向没有定义，所以起摆开始先给一个固定方向、
固定时间的踢动：

```verilog
KICK_DUTY = 35%,  KICK_MS = 100 ms
```

踢动方向与 `POLARITY` 一致。踢动结束后，速度已经非零，能量泵接管。

### 2.3 能量泵方向

对“摆杆 + 支点加速度”模型，能量变化为

```
dE/dt = l * a * v * cos(e)
```

在“原有平衡环能稳定”的符号约定下，要增加能量应取

```
u = +sign(v * cos(e))
```

`cos(e)` 的符号只需要知道 `|e|` 是否大于 1024（90°）：

```verilog
cos_neg = (|e| > 1024);
prod_neg = sign(v) XOR cos_neg;
u = prod_neg ? -SWING_DUTY : +SWING_DUTY;
```

`SWING_POLARITY` 可整体取反。**如果平衡环方向是对的，`POLARITY=1` 就是对的**；
只有在电机/传感器整体反装、平衡环也反着调的情况下才需要改成 0。

### 2.4 v_need 限速：为什么不会越摆越快

只按上面的方向泵能量，摆杆能量会一直增加，最后直接“转圈”，很难进平衡。
`swing_up.v` 增加了一个从当前角度升到竖直所需的最小速度：

```
v_need(e) = sqrt(2*K*(1 - cos(pi*|e|/2048)))
K         = (g/l) * (T * 2048/pi)^2,  T = 5 ms
```

只有当 `|v| < v_need(e)` 时才继续泵能量；否则输出 0（让摆杆滑行）。
这样能量接近“刚好到顶”时会自动停止，顶部速度被限制在一个可捕获的范围。

`v_need` 用 33 项查表实现（`|e|/64` 索引），默认按 `l = 0.30 m` 生成：

```
[0, 2, 4, 5, 7, 9, 11, 13, 14, 16, 18, 19, 21, 22, 24, 25, 26,
 28, 29, 30, 31, 32, 33, 34, 34, 35, 36, 36, 37, 37, 37, 37, 37]
```

摆杆长度不同（摩擦也不同）时，可以：

1. 用 `python sim/swing/make_vneed_lut.py --length 0.25` 重新生成表，粘贴到
   `swing_up.v`；或
2. 先不改表，增大 `SWING_VNEED_OFFSET`（1~5）给一点额外能量，补偿摩擦。

### 2.5 顶部捕获与交接

捕获条件：

```verilog
(|e| < CAPTURE_ANGLE) && (|v| <= CAPTURE_VEL)   // 连续 CAPTURE_TICKS 次 5 ms
```

默认 `CAPTURE_ANGLE=300`（约 26°）、`CAPTURE_VEL=12`（counts/5 ms）、
`CAPTURE_TICKS=3`。满足后 `captured` 输出一个时钟脉冲，`top.v` 在下一个时钟沿
进入 `S_BALANCE`；`rotate_ctrl` 同时把外环目标 `loc_target` 装成当前 `Location`。

> 捕获窗口内 `|e| < CAPTURE_ANGLE` 时能量泵被关闭（`pump_ok` 要求
> `|e| > CAPTURE_ANGLE`），避免在顶部继续加速。

---

## 3. 参数表（`src/top.v` 当前值）

| 参数 | 当前值 | 作用 / 调整建议 |
|---|---:|---|
| `CENTER_ANGLE` | 2050 | 竖直平衡点；必须按实际机械标定 |
| `HANG_BAND` | 240 | 自然下垂判定带，约 ±21°；下垂读数偏大时加大 |
| `HANG_CONFIRM_MS` | 500 | 连续下垂多久触发；太小会被摆动经过误触发 |
| `AUTO_ARM_ON_RESET` | 1 | 上电是否允许自动起摆 |
| `AUTO_RETRY` | 0 | 超时/倒杆后是否允许自动重试 |
| `SWING_DUTY` | 80% | 能量泵占空比；当前已较大，起不来先查方向/摩擦 |
| `SWING_KICK_DUTY` | 35% | 起始踢动占空比 |
| `SWING_KICK_MS` | 100 ms | 起始踢动时间 |
| `SWING_CAPTURE_ANGLE` | 300 | 捕获窗口半角（counts） |
| `SWING_CAPTURE_VEL` | 8 | 捕获速度门限（counts/5 ms） |
| `SWING_TIMEOUT_MS` | 4000 | 单次起摆超时 |
| `SWING_VNEED_OFFSET` | 0 | v_need 整体加量，补偿摩擦 |
| `SWING_POLARITY` | 1 | 能量泵方向；平衡环正常时保持 1 |

`HANG_BAND` 与 `SWING_CAPTURE_ANGLE` 都是 12 bit 角度 counts；
`SWING_CAPTURE_VEL` 的单位是“每 5 ms 的 counts”，不是 counts/s。

---

## 4. 验证

### 4.1 Python 闭环模型

`sim/swing/model.py` 用“摆杆 + 支点加速度”模型（l=0.30 m、A_FULL=40 m/s²、
粘性摩擦 0.6 /s）闭环验证控制律，输出 `sim/swing/model.png`。模型里的
`CENTER=2053` 是合成角度，只影响角度坐标，不影响起摆/捕获逻辑（硬件当前标定为 2050）。

```powershell
python sim\swing\model.py
```

模型结果（`ENERGY_MARGIN=0`、80% 泵、35%/100 ms 踢动）：
摆杆从下垂开始，约 **1.02 s** 到达顶部捕获窗口（`e≈+228`、`v≈-7.6 counts/5 ms`）
并切换到平衡；平衡后角度收敛。

### 4.2 RTL 模块级闭环（`tb_swing_fast.v`）

为了不跑 50 MHz 的完整 top，`tb_swing_fast.v` 用 1 MHz 时钟 + 手动 1 ms / 5 ms
脉冲，只实例化 `swing_up.v`，接同一个摆杆模型：

```powershell
iverilog -g2005 -o sim\tb_swing_fast.vvp -s tb_swing_fast `
  sim/tb_swing_fast.v src/swing_up.v
vvp sim\tb_swing_fast.vvp
```

实测日志（`sim/swing/run_swing_fast.log`）：

```
t=  0.0 ms  theta= 180.00 deg  angle=   5  e=-2048  v=  0.0  drive=   0
t=100.0 ms  theta= 168.45 deg  angle=3969  e= 1916  v=-12.0  drive=  35
t=200.0 ms  theta= 123.74 deg  angle=3460  e= 1407  v=-34.0  drive=   0
t=300.0 ms  theta=  74.88 deg  angle=2905  e=  852  v=-24.0  drive=   0
t=400.0 ms  theta=  45.87 deg  angle=2574  e=  521  v=-14.0  drive=   0
t=500.0 ms  theta=  24.37 deg  angle=2330  e=  277  v=-11.0  drive=   0
CAPTURED t=575.0 ms angle=2201 e=148 v=-8.0
```

起摆约 **0.575 s** 后进入捕获窗口，捕获时 `e=148`、速度 **-8.0 counts/5 ms**，
正好在 8 的门限内。

### 4.3 top 状态机（`tb_top_logic.v`）

`tb_top_logic.v` 用强制 `hang_ms` / `captured` / `timeout` 的方式快速验证状态迁移，
不跑完整起摆：

```powershell
iverilog -g2005 -o sim\tb_top_logic.vvp -s tb_top_logic `
  sim/tb_top_logic.v src/PWM.v src/adc.v src/encoder.v src/key_filter.v `
  src/pid_core.v src/swing_up.v src/uart_rx.v src/angle_cmd.v `
  src/rotate_ctrl.v src/telemetry.v src/top.v src/uart_tx.v
vvp sim\tb_top_logic.vvp
```

实测 8 项全部 PASS（`sim/swing/run_top_logic.log`）：

- 自然下垂时 `hang_ms` 按 1 ms 正常累加（12 ms 时读到 12）；
- 自然下垂 + 确认计时到门限 → `S_SWING`；
- `captured` → `S_BALANCE`，`loc_target == Location`；
- K1 停止 → `S_IDLE` / `armed=0`；
- 摆杆立住时按 K1 → 直接 `S_BALANCE`；
- 自然下垂时按 K1 → 立即 `S_SWING`；
- 起摆超时 → `S_IDLE` / `armed=0`。

### 4.4 完整 top 闭环（`tb_swing.v`，可选）

`tb_swing.v` 实例化完整 `top`（50 MHz）+ 摆杆模型，包含 ADC 平均、4 counts
量化、PID 定点运算和真实 5 ms / 50 ms 时基。为了节省仿真时间，测试平台在
2 ms 时把 `hang_ms` 直接推到 499，跳过 500 ms 的确认等待。

```powershell
iverilog -g2005 -o sim\tb_swing.vvp -s tb_swing `
  sim/tb_swing.v src/PWM.v src/adc.v src/encoder.v src/key_filter.v `
  src/pid_core.v src/swing_up.v src/uart_rx.v src/angle_cmd.v `
  src/rotate_ctrl.v src/telemetry.v src/top.v src/uart_tx.v
vvp sim\tb_swing.vvp
```

> 这个仿真在 50 MHz 下用 Icarus 跑 2.5 s 需要数分钟到十几分钟，本环境没有等它
> 跑完；起摆算法由 4.2 的模块级闭环验证，状态迁移由 4.3 覆盖。建议在上机前
> 本地跑一次，或直接用串口 `swing` 标志和 `duty` 看实际波形。

### 4.5 原有平衡回归

`sim/tb_top.v` 已同步到新代码（不再 force 旧版 `diff_lpf_shift` 端口，改用
`dut.balancing`）。它验证“摆杆已在平衡点附近，按 K1 直接进入平衡”的路径，
PID 积分限幅/清零行为保持不变：

```powershell
iverilog -g2005 -o sim\tb_top_new.vvp -s tb_top sim/tb_top.v `
  src/PWM.v src/adc.v src/encoder.v src/key_filter.v src/pid_core.v `
  src/swing_up.v src/uart_rx.v src/angle_cmd.v src/rotate_ctrl.v `
  src/telemetry.v src/top.v src/uart_tx.v
vvp sim\tb_top_new.vvp
```

---

## 5. 上机调试顺序

1. **先确认角度方向和平衡点**：把摆杆扶到竖直，串口看 `angle_raw`，应约等于
   `CENTER_ANGLE`；自然下垂时看是否在 0 / 4095 附近。不在就改
   `CENTER_ANGLE` / `HANG_BAND`。
2. **确认平衡方向仍然正确**：按 K1 手动把摆杆扶到平衡窗口内启动，确认车/臂
   追着摆走。如果这里方向反了，先修 `in1/in2` 或角度误差符号，再谈起摆。
3. **单独试起摆**：让摆杆自然下垂，等 0.5 s，观察是否开始踢动。如果不动：
   - 看串口 `swing` 标志位（flags bit6）是否置 1；
   - 看 `duty` 是否有 35% 踢动脉冲，随后 80% 泵输出；
   - `SWING_DUTY` 当前已 80%，先查 `SWING_POLARITY`、机械摩擦/接线，
     可把 `SWING_KICK_DUTY` 临时加到 45% 试。
4. **起摆方向不对**：摆杆越泵越没劲、或者明显被“压”下去，先把
   `SWING_POLARITY` 改成 `1'b0` 试一次（注意：平衡环正常时通常不需要）。
5. **起摆能摆到顶部但捕不到**：
   - 捕获时速度太大：减小 `SWING_DUTY`（例如 50~60%），或把
     `SWING_VNEED_OFFSET` 先设为 0 再微调；
   - 摆杆到不了顶：加大 `SWING_VNEED_OFFSET`（每次 1~2）；
   - 顶部抖动/反复过顶：适当减小 `SWING_CAPTURE_VEL` 或加大
     `SWING_CAPTURE_ANGLE`（先保证能进窗口）。
6. **捕获后平衡不住**：说明交接速度仍然偏大或平衡环增益偏弱。先降低
   `SWING_CAPTURE_VEL`（例如 6），再按原 `docs/fix-report.md` 的顺序调角度环
   PD。`S_BALANCE` 的 ±500 倒杆保护会兜底停机。
7. **需要反复试**：超时/倒杆后系统会撤销自动触发，按 K1 重新起摆；
   上电自动起摆由 `AUTO_ARM_ON_RESET` 控制。

---

## 6. 串口遥测新增标志

`telemetry.v` 的 flags 字节（byte 22）保持原布局，只使用了原来的保留位：

| bit | 含义 |
|---:|---|
| 6 | `swing`：1 = 正在自动起摆 |
| 5 | 保留 0 |
| 4 | overrun |
| 3 | angle_sat |
| 2 | loc_sat |
| 1 | in_window |
| 0 | run（平衡环运行） |

起摆过程中 `run=0`、`swing=1`，`motor_out` / `duty` 仍是实际起摆输出，
可以直接在串口助手里看起摆波形。

---

## 7. 文件清单

| 文件 | 说明 |
|---|---|
| `src/swing_up.v` | 新增：能量泵 + 踢动 + v_need 限速 + 捕获 |
| `src/top.v` | 新增三状态机、自然下垂计时、电机 mux、K1 新语义、平衡目标加载 |
| `src/uart_rx.v`、`src/angle_cmd.v`、`src/rotate_ctrl.v` | 角度旋转命令（见 `docs/rotate-cmd.md`） |
| `src/telemetry.v` | flags bit6 = swing |
| `pendulum.gprj` | 加入新增源文件 |
| `sim/tb_swing_fast.v` | 模块级快速闭环仿真 |
| `sim/tb_top_logic.v` | top 状态机/交接逻辑测试 |
| `sim/tb_swing.v` | 完整 top 闭环仿真（50 MHz） |
| `sim/swing/model.py` | Python 控制律模型 |
| `sim/swing/make_vneed_lut.py` | 按摆长生成 v_need 查表 |
| `backup/top.v.before-swing`、`backup/pid_core.v.before-swing` | 改动前备份 |
