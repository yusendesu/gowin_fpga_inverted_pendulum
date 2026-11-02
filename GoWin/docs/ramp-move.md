# 增量移动改为梯形速度斜坡（ramp_move）

> 目标：解决平衡状态下按 ESP32 增量角度命令移动时“越转越快、最后转飞”的问题。
> 结论：把**轨迹生成**从位置误差 PD 中拆出来，改成确定性的梯形速度斜坡，
> 并加入行程、跟踪误差、移动时间三重保护。外层摆臂转角 PID 只负责跟踪，
> 不再决定“下一拍目标在哪”。

相关源码：

- `src/ramp_move.v`：新增，梯形斜坡 + 增量合并 + 安全保护
- `src/rotate_ctrl.v`：改为调用 `ramp_move`，保留正弦摆动
- `src/top.v`：参数改为 `RAMP_*`，速度前馈增加 `FF_LIMIT` 限幅
- `sim/tb_ramp_move.v`：斜坡单元测试
- `sim/tb_rotate.v`、`sim/tb_rotate_top.v`、`sim/tb_sine_amp.v`：回归测试

---

## 1. 旧增量移动为什么容易转飞

旧版 `rotate_ctrl.v` 里虽然有一段“S 曲线”，但它仍然是用**位置误差**算加速度：

```text
a = KP * (target - pos) - KD * v
v = clamp(v + a, ±V_MAX)
pos = pos + v
```

这条轨迹本身能收敛，但增量命令的处理方式是：

```verilog
// 每收到一帧：
target = Location + delta;   // 以“当前实际位置”重新起步
pos    = Location;
v      = 0;
```

由此带来四个问题：

1. **连续发帧时轨迹被反复打断**
   - 每一帧都把速度清 0、从当前实际位置重新规划；
   - 速度前馈 `v_ref` 跟着一跳一跳，外环看到的是脉冲式速度指令；
   - 帧率越高，实际运动越像“走走停停”，而不是一条平滑轨迹。

2. **没有总行程上限**
   - 协议是相对增量，ESP32 连续发 `+10°`、`+10°`……目标就会一直往前推；
   - 只要帧还在来，摆臂就没有“终点”，最终可能把线缆/机械行程走完。

3. **没有跟踪误差保护**
   - 如果外环（摆臂转角 PID + 摆杆角度环）因为摩擦、饱和、方向不对而跟不上，
     轨迹目标仍然按自己的速度往前走；
   - 位置误差越来越大，外环输出饱和，摆杆被压得越来越斜，表现就是
     “越转越快，最后转飞”。

4. **正弦开关会抢走命令**
   - 旧版 `sine_en` 是电平。普通角度命令把 `sine_active` 清 0 后，
     下一拍 `sine_en` 仍为 1，会立刻重新启动正弦，把刚收到的角度命令覆盖掉。
   - 本次同时加了 `sine_inhibit`：收到角度命令后，必须先关一次拨码，
     正弦才允许再次启动。

另外两个时序优先级问题也一并修了：

- `load`（进入平衡那一拍）必须优先于 `!balancing`。旧 `ramp_move` 初版把
  `!balancing` 放在前面，导致进入平衡时第一个目标被清成 0；
- `!sine_en` 不能单独作为一个 `else if` 分支挡住 `tick_1ms`，否则正弦渐出
  永远不会结束。

---

## 2. 新算法：梯形速度斜坡

### 2.1 速度曲线

```text
        v
        ^
   Vmax |     ______________
        |    /              \
        |   /                \
        |  /                  \
        +-/--------------------\-------> t
         Aaccel             Adecel
```

每个 1 ms：

1. **加速段**：`v += A_MAX`，直到 `|v| = V_MAX`；
2. **匀速段**：保持 `V_MAX`；
3. **减速段**：当剩余距离只剩制动距离时，`v -= A_MAX`；
4. **末端吸附**：剩余 `<= SETTLE_COUNTS` 且 `|v| <= SETTLE_V` 时直接贴终点。

速度方向和目标方向相反时（例如运动中收到反向命令），先把速度减到 0，
再朝新方向加速，不会直接反向跳变。

### 2.2 什么时候开始减速

连续时间下制动距离是

```text
d_brake = v^2 / (2a)
```

离散实现里每 1 ms 减 `A`，实际停止距离约为 `v^2/(2A) + v/2`。
代码用下面的判据提前进入减速：

```text
v^2 + A*v  >=  2*A*|err|
```

两边都是 Q16 定点，`v` 和 `err` 的单位分别是 `counts/ms` 和 `counts`。
这样不需要除法/开方，只需要乘法和 64 位比较，FPGA 里很省资源。

### 2.3 增量命令怎么合并

| 当前状态 | 收到一帧角度 |
|---|---|
| 空闲/已停稳 | `目标 = 当前实际位置 Location + 本帧角度` |
| 正在移动 | `目标 = 当前目标 + 本帧角度`，**不打断速度** |

所以：

```text
先发 +20°，100 ms 后再发 +40°  =>  最终走 60°，而不是 40°
```

`tb_ramp_move` 里专门验证了这一点：最终 `target=220 counts`（60°），
而不是从当前位置重新起步的 147 counts（40°）。

### 2.4 外层 PID 的角色

`ramp_move` 输出：

- `target`：位置环目标；
- `v_ref`：速度前馈（Q16 counts/ms）；
- `rotating`：轨迹未收敛，给外环清积分。

外层 `pid_core` 仍然跟踪 `target`，但它的输出不再反过来决定轨迹形状。
这样即使外环有摩擦、饱和、参数偏软，轨迹本身也是**有界、可预测**的：
不会因为误差大就自己加速。

---

## 3. 三重安全保护

### 3.1 总行程限幅 `RAMP_MAX_TRAVEL_DEG`

一次连续增量移动（从停稳到再次停稳）的总行程限制在默认 **360°**：

```text
目标被夹在 [base - MAX_TRAVEL, base + MAX_TRAVEL]
```

- 单帧最大 ±180°，正常一帧不会触发；
- ESP32 连续刷帧时，最多再转一圈就停住，不会无限累积；
- 停稳后再发命令，会以新的实际位置为 `base` 开始新的一段；
- 想允许更大行程，改 `RAMP_MAX_TRAVEL_DEG`（注意 Q16 32 位上限，建议不超过
  3000°）。

### 3.2 跟踪误差保护 `RAMP_TRACK_ERR_COUNTS` / `RAMP_TRACK_TIMEOUT_MS`

每个 1 ms 检查：

```text
|轨迹目标 - 实际位置| > RAMP_TRACK_ERR_COUNTS ?
```

- 默认 200 counts ≈ 54°；
- 超差时**暂停轨迹推进**，并把 `v_ref` 清零，让外环先追；
- 连续超差 500 ms，说明外环确实拉不住：
  - `target`、`goal`、`base` 全部拉回当前实际位置；
  - 速度清零；
  - `fault` 置 1（保持到下一次命令/装载）。
- 这是防止“目标一直往前跑、外环永远追不上”的最后一道闸。
- `fault` 已接到遥测 `flags` 字节的 **bit5**，可以直接在串口助手里看到。

### 3.3 移动超时 `RAMP_MOVE_TIMEOUT_MS`

默认 20 s。按默认 27°/s，转满 360° 约 13.3 s，20 s 有足够余量。
如果电机堵转、编码器不动、外环失效，超时后同样把目标拉回实际位置并置 `fault`。

### 3.4 速度前馈限幅 `FF_LIMIT`

`top.v` 里速度前馈：

```text
loc_ff_v = clamp((KV_FF * v_ref) >> 16, ±FF_LIMIT)
```

默认 `FF_LIMIT = 12` 个摆杆倾角 ADC counts。即使 `KV_FF` 被误设得很大，
前馈也不会把摆杆压到很大的角度。摩擦前馈 `FRIC_FF` 仍然单独叠加。

---

## 4. 参数与整定

### 4.1 `src/top.v` 参数

| 参数 | 默认 | 说明 |
|---|---:|---|
| `RAMP_V_MAX` | 6554 | 最大速度，Q16 counts/ms；6554 = 0.10 count/ms ≈ 27°/s |
| `RAMP_A_MAX` | 66 | 加速度，Q16 counts/ms²；66 ≈ 1000 counts/s² |
| `RAMP_SETTLE_COUNTS` | 2 | 末端吸附距离，2 counts ≈ 0.55° |
| `RAMP_SETTLE_V` | 328 | 末端吸附速度，0.005 count/ms |
| `RAMP_MAX_TRAVEL_DEG` | 360 | 一次连续增量最大行程 |
| `RAMP_TRACK_ERR_COUNTS` | 200 | 跟踪超差门限，200 counts ≈ 54° |
| `RAMP_TRACK_TIMEOUT_MS` | 500 | 连续超差多久后放弃 |
| `RAMP_MOVE_TIMEOUT_MS` | 20000 | 单次移动绝对超时 |
| `KV_FF` | 100 | 速度前馈增益 |
| `FF_LIMIT` | 12 | 速度前馈限幅（倾角 counts） |
| `FRIC_FF` | 4 | 摩擦前馈（倾角 counts） |

### 4.2 整定顺序

1. **先确认方向**：发 +90°，摆臂应朝你定义的顺时针转；反了改 `ROTATE_SIGN`。
2. **只调 `RAMP_V_MAX`**：从较小值开始（例如 3277 = 13.6°/s），确认能平稳转 90°；
   逐步加到 6554。速度越大，摆杆越容易被“甩”倒。
3. **再调 `RAMP_A_MAX`**：默认 66 比较温和。想更跟手可加到 131（≈2000 counts/s²），
   但起步/停止的冲击也会变大。
4. **最后调 `KV_FF`**：先设 0，确认纯位置环能转到位；再从 50 开始加。
   如果加上前馈反而更容易倒，说明符号不对或太大，先回 0。
5. **`RAMP_MAX_TRAVEL_DEG` 不建议一上来改大**。先把 360° 内的移动调稳。

### 4.3 参数换算

```text
1 count    = 360/1320 ≈ 0.273°
1 count/ms = 1000 counts/s ≈ 272.7°/s
```

默认 `RAMP_V_MAX=6554`：

```text
6554 / 65536 = 0.10 count/ms = 100 counts/s ≈ 27.3°/s
```

默认 `RAMP_A_MAX=66`：

```text
66 / 65536 = 0.001007 count/ms² = 1007 counts/s² ≈ 274°/s²
```

---

## 5. 验证结果

### 5.1 `tb_ramp_move`（单元）

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

覆盖：

- 斜坡速度不超过 `V_MAX`；
- 不冲过终点；
- 运动中累加命令；
- 总行程 360° 限幅；
- 实际位置卡住时跟踪保护生效。

### 5.2 `tb_rotate`（rotate_ctrl + 正弦）

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

### 5.3 `tb_rotate_top`（串口命令 -> top）

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

### 5.4 正弦幅值回归

```powershell
iverilog -g2005 -o sim\tb_sine_amp.vvp -s tb_sine_amp `
  sim\tb_sine_amp.v src\rotate_ctrl.v src\ramp_move.v src\sine_lut.v
vvp sim\tb_sine_amp.vvp
```

实测（`sim/swing/run_sine_amp.log`）：

```text
MAX=220  MIN=-220  peak-to-peak=440 counts = 120.00 deg
PASS: 正弦幅值 = ±60.0 deg
```

---

## 6. 上机排查

| 现象 | 优先检查 |
|---|---|
| 一收到命令就反向 | `ROTATE_SIGN`；先发 +90° 看 `Location` 增大还是减小 |
| 能转但明显转过头 | `RAMP_V_MAX` 太大或外环 `LOC_KP/LOC_KD` 太弱；先把速度降到 3277 |
| 转到一半停住、`loc_target` 回到原地 | 跟踪保护触发：外环没跟上。看串口 `loc_error`、`loc_pid_out`、`loc_sat`；先降 `RAMP_V_MAX`，再检查外环方向/增益 |
| 连续发帧只转一圈就停 | 正常，`RAMP_MAX_TRAVEL_DEG=360` 在限幅；停稳后再发下一段 |
| 发完命令后正弦又自己启动 | 已加 `sine_inhibit`；需要先把拨码关一次再开 |
| 进入平衡瞬间目标变成 0 | 检查 `load` 是否仍然优先于 `!balancing`（当前代码已保证） |
| 摆杆在移动中明显前倾/后仰 | `KV_FF` 符号或大小；先设 0，确认纯位置环；`FF_LIMIT` 限制前馈不超过 ±12 |
| 电机堵转/编码器不动 | 20 s 移动超时后 `target` 会回到实际位置，遥测 `flags` bit5（`move_fault`）=1 |

---

## 7. 文件清单

| 文件 | 说明 |
|---|---|
| `src/ramp_move.v` | 新增：梯形斜坡、增量合并、行程/跟踪/超时保护 |
| `src/rotate_ctrl.v` | 改为调用 `ramp_move`；正弦逻辑修复（禁止标志、渐出） |
| `src/top.v` | `RAMP_*` 参数、`FF_LIMIT`、例化 `move_fault` |
| `pendulum.gprj` | 加入 `src/ramp_move.v` |
| `sim/tb_ramp_move.v` | 新增单元测试 |
| `sim/tb_rotate.v` | 适配新接口与斜坡时序 |
| `sim/tb_rotate_top.v` | 适配 `RAMP_*` 参数 |
| `sim/tb_sine_amp.v` | 适配 `sine_en` 接口 |
| `backup/*.before-ramp` | 改动前的 `rotate_ctrl.v`、`top.v` 备份 |
