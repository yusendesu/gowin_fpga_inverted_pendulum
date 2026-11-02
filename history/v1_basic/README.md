# gowin_fpga_inverted_pendulum

本readme.md由DeepSeek-V4.1总结
基于高云 GW2A 系列 FPGA 的一阶倒立摆控制器。

- 角度：AD9200 / 3PA1030 10bit 并行 ADC + WDD35D4 角度传感器
- 位置：增量式编码器（四倍频，中心值 32768）
- 控制：位置环（外环）+ 角度环（内环）增量式 PID
- 驱动：TB6612 × 2，两路电机同方向、同占空比
- 交互：ESP32 通过 UART 控制启停、校准和方向，FPGA 周期回传遥测

当前版本已去掉 4 个板载按键，全部通过 ESP32 串口控制。

---

## 1. 文件说明

| 文件 | 说明 |
| --- | --- |
| `top.v` | **顶层**：传感器采集、UART 命令解析、启停/安全保护、PID 与电机输出、遥测 |
| `adc.v` | ADC 采集与 3ms 滑动平均，输出 12bit 角度 `theta_out` |
| `encoder.v` | 编码器两级同步、数字滤波、四倍频计数，中心值 32768 |
| `pid_dual_loop.v` | 位置环 + 角度环增量式 PID（1kHz 控制周期） |
| `motor_ctrl.v` | TB6612 方向控制 + 约 10kHz PWM |
| `uart.v` | 遥测发送（10 字节帧） |
| `uart_tx.v` / `uart_rx.v` | UART 收发底层模块 |
| `fo_rip_top.cst` | 当前顶层使用的引脚约束（已移除按键） |
| `AD_test.cst` | ADC 单独测试用约束，**不是**当前顶层的约束 |
| `key_filter.v` | 本版本未使用 |
| `PWM.v` | 未使用（`motor_ctrl.v` 内部已包含 PWM） |
| `encoder_for_test.v` | 仅编码器测试用，**不要加入工程** |

---

## 2. 控制结构

```
                 ┌──────────────┐
 WDD35D4 ─ADC──► │ theta_raw    │
                 │ 3ms 平均     │
                 └──────┬───────┘
                        │ theta_fb（竖直=2048）
                        ▼
 ESP32 ─UART─► 命令解析  ┌─────────────────────────────┐
                        │ pid_dual_loop               │
 encoder1 ──────► pos_fb │ 位置环 → theta_set → 角度环 │──► motor_sel/duty
                        └─────────────────────────────┘         │
                                                                 ▼
                                                        TB6612 A/B → 电机
```

启动时位置环目标锁存为当前编码器位置，因此上电/启动瞬间不会因为位置误差猛拉电机。
停止、倾倒保护、重新启动时会复位 PID 的积分和历史状态。

---

## 3. 引脚连接

以 `fo_rip_top.cst` 为准：

| 信号 | FPGA 引脚 | 说明 |
| --- | --- | --- |
| `clk_50m` | M19 | 50MHz 时钟 |
| `rst_n` | AB3 | 复位，低有效 |
| `uart_tx` | K20 | FPGA → ESP32 RX |
| `uart_rx` | L20 | ESP32 TX → FPGA |
| `adc_clk` | G18 | ADC 采样时钟 |
| `adc_oe_n` | H20 | ADC 输出使能，顶层常驱 0（低有效） |
| `adc_data_in[9:0]` | E19/G17/G19/H19/H18/D19/J19/J20/M21/L21 | ADC 并行数据 |
| `enc1_a` / `enc1_b` | Y20 / AA20 | 编码器 1 |
| `enc2_a` / `enc2_b` | W18 / W17 | 编码器 2 |
| `AN1` / `AN2` / `PWMA` | Y17 / Y19 / Y18 | 电机 A |
| `BN1` / `BN2` / `PWMB` | V15 / AA17 / V14 | 电机 B |

> 注意：`AD_test.cst` 里 `BN1`/`BN2` 的引脚和 `fo_rip_top.cst` 是**反过来**的
> （`BN2=V15, BN1=AA17`）。如果 B 电机转向和 A 电机相反，先检查这两个约束，
> 必要时对调 `BN1`/`BN2`。最终工程请使用 `fo_rip_top.cst`。

---

## 4. ESP32 串口协议

- 波特率：**9600**，8 数据位，无校验，1 停止位（8N1）
- 电平：3.3V，不需要流控
- ESP32 `TX` 接 FPGA `L20`，ESP32 `RX` 接 FPGA `K20`

### 4.1 ESP32 → FPGA 命令帧（6 字节）

```
0xAA  0x55  CMD  DL  DH  XOR
XOR = CMD ^ DL ^ DH
```

| CMD | 名称 | DATA | 说明 |
| --- | --- | --- | --- |
| `0x01` | START | 0 | 启动闭环。只有角度在安全范围内才接受，同时锁存当前位置 |
| `0x02` | STOP | 0 | 停止闭环，电机停转 |
| `0x03` | SET_UPRIGHT | 0~4095 | 设置“竖直”对应的 `theta_raw` 原始值（仅停止时有效） |
| `0x04` | SET_DIR | 0/1 | 电机方向取反。角度环推反了就用它（仅停止时有效） |
| `0x05` | CLEAR_FAULT | 0 | 清除 fault 标志，不会自动启动 |
| `0x06` | CAP_UPRIGHT | 0 | 把当前 `theta_raw` 作为竖直校准值（仅停止时有效，推荐） |
| `0x07` | SET_POS_DIR | 0/1 | 编码器计数取反。位置环推反了就用它（仅停止时有效） |

未识别命令或校验错误会被丢弃；半帧超过 20ms 会自动回到空闲重新同步。

### 4.2 FPGA → ESP32 遥测帧（10 字节，约 0.5s 一帧）

```
0xAA  0x55  TH  TL  E1H  E1L  E2H  E2L  0x0D  0x0A
```

`TH/TL = {run, fault, motor_sel[1:0], theta_raw[11:0]}`

| 字段 | 位置 | 说明 |
| --- | --- | --- |
| `theta` | `TH[3:0]`,`TL` | 12bit 角度原始值，0~4092 |
| `run` | `TH[7]` | 1=闭环运行 |
| `fault` | `TH[6]` | 1=倾倒保护或启动被拒绝 |
| `motor_sel` | `TH[5:4]` | 00 停 / 01 正转 / 10 反转 / 11 刹车 |
| `E1` | `E1H`,`E1L` | `enc_pos1`，16bit，中心 32768 |
| `E2` | `E2H`,`E2L` | `enc_pos2`，16bit，中心 32768 |

### 4.3 收发示例

Arduino / ESP32：

```cpp
// Serial2.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN); // 按实际 ESP32 引脚修改

void sendCmd(uint8_t cmd, uint16_t data) {
    uint8_t f[6] = {
        0xAA, 0x55, cmd,
        (uint8_t)(data & 0xFF), (uint8_t)(data >> 8),
        (uint8_t)(cmd ^ (data & 0xFF) ^ (data >> 8))
    };
    Serial2.write(f, sizeof(f));
}

// 竖直校准：保持摆杆竖直，然后：
sendCmd(0x06, 0);   // CAP_UPRIGHT
sendCmd(0x01, 0);   // START

// 停止：
sendCmd(0x02, 0);
```

Python（USB-TTL 调试）：

```python
import serial

ser = serial.Serial("COM5", 9600, timeout=1)

def send(cmd, data=0):
    dl, dh = data & 0xFF, (data >> 8) & 0xFF
    ser.write(bytes([0xAA, 0x55, cmd, dl, dh, cmd ^ dl ^ dh]))

def read_frame():
    buf = b""
    while True:
        buf += ser.read(1)
        if len(buf) >= 10:
            if buf[0] == 0xAA and buf[1] == 0x55 and buf[8] == 0x0D and buf[9] == 0x0A:
                t = (buf[2] << 8) | buf[3]
                return {
                    "theta": t & 0x0FFF,
                    "run": (t >> 15) & 1,
                    "fault": (t >> 14) & 1,
                    "motor_sel": (t >> 12) & 3,
                    "enc1": (buf[4] << 8) | buf[5],
                    "enc2": (buf[6] << 8) | buf[7],
                }
            buf = buf[1:]
```

---

## 5. 上电与调试流程

1. 上电后等待至少 **100ms**（ADC 需要 3ms 才输出第一个平均值；第一帧遥测约在 0.5s 时发出）。
2. 读取遥测，手动摆动摆杆，确认 `theta` 会跟着变化。
3. 把摆杆扶到竖直位置，发送 `CAP_UPRIGHT`（`0x06`）。
   - 也可以先读遥测，把竖直时的 `theta` 用 `SET_UPRIGHT`（`0x03`）发下去。
4. 发送 `START`（`0x01`）。
   - 如果遥测 `fault=1`，说明角度超出 `SAFE_ANGLE`，重新校准或检查传感器。
5. 如果电机把摆杆推倒的方向相反：先 `STOP`，发 `SET_DIR 1`（`0x04`），再 `START`。
6. 先只调角度环：把 `top.v` 中 `KP_POS/KI_POS/KD_POS` 设为 0，
   确认能原地平衡，再逐步加位置环。
7. 如果位置环让小车往远离目标的方向跑：`STOP`，发 `SET_POS_DIR 1`（`0x07`），再 `START`。

### 建议的调试顺序

```
CAP_UPRIGHT → START（只角度环）→ 调 KP_ANG/KD_ANG
           → 加位置环 KP_POS/KD_POS → 加 KI_POS
```

---

## 6. 参数说明

都在 `top.v` 的模块参数里，可直接在工程中修改：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `KP_POS/KI_POS/KD_POS` | 10/2/30 | 位置环增量式 PID |
| `KP_ANG/KI_ANG/KD_ANG` | 40/3/25 | 角度环增量式 PID |
| `CNT_CYCLE` | 49999 | 控制周期，1kHz |
| `SAFE_ANGLE` | 360 | 安全角度，约 30°（12bit 码值） |
| `FALL_CNT_MAX` | 49999 | 超过安全角度持续 1ms 后停机 |
| `DIR_INV_DEFAULT` | 0 | 上电默认电机方向取反 |
| `POS_INV_DEFAULT` | 0 | 上电默认编码器计数取反 |

角度换算：WDD35D4 约 345°/1024（10bit），`theta_raw = adc_data << 2`，
所以 12bit 下约 **11.9 码/度**，360 码 ≈ 30°。

> `adc.v` 每 **3ms** 更新一次 `theta_out`，而 PID 默认 1kHz。
> 如果角度环出现明显的 3ms 周期抖动，可把 `CNT_CYCLE` 改成 `32'd149_999`，
> 让 PID 与 ADC 更新同步（333Hz）。

---

## 7. 已知限制 / 注意事项

1. **遥测只有约 2Hz**：`uart.v` 每字节间隔 20ms，10 字节一帧约 210ms。
   如果需要更快的上位机波形，把 `uart.v` 的 `SEN_CNT_MAX` 改小
   （例如 `20'd100_000`，2ms），同时把 `top.v` 里 `uart` 的 `CNT_1S`
   改到大于一帧时间（例如 `32'd4_999_999`，100ms），即可到约 10Hz。
2. `adc.v` 的 3ms 平均会带来相位滞后，高速摆动时注意这一点。
3. B 电机方向：`fo_rip_top.cst` 与 `AD_test.cst` 的 `BN1/BN2` 不一致，见第 3 节。
4. `.cst` 文件头写的器件是 `GW2A-55PG484`，而实际板卡是 GW2A-18；
   如果引脚位置对不上，请以实际板卡原理图为准重新约束。
5. 旧版 `top.v` 的 `adc_stby_n` 已删除：新版 `adc.v` 没有该端口，
   且 `fo_rip_top.cst` 也没有该引脚。若你的板子 STBY 没有硬件上拉，
   需要在约束里补一个引脚并驱动为 1。
6. 两路电机使用同一组方向和占空比；如果机械上两个电机是镜像安装的，
   需要单独取反其中一路（改 `motor_ctrl` 的输入或交换电机线）。
7. `key_filter.v`、`PWM.v`、`encoder_for_test.v` 不参与当前顶层编译。

---

## 8. 工程编译

1. 在 Gowin IDE 中新建/打开工程，器件选实际板卡型号。
2. 添加：`top.v`、`adc.v`、`encoder.v`、`pid_dual_loop.v`、`motor_ctrl.v`、
   `uart.v`、`uart_tx.v`、`uart_rx.v`。
   **不要**添加 `encoder_for_test.v`。
3. 顶层设为 `top`。
4. 物理约束使用 `fo_rip_top.cst`（不是 `AD_test.cst`）。
5. 建议加 50MHz 时钟约束（`.sdc`）：

   ```tcl
   create_clock -name clk_50m -period 20 -waveform {0 10} [get_ports {clk_50m}]
   ```

6. 综合、布局布线、下载。
