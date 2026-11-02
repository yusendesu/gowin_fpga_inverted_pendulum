# -*- coding: utf-8 -*-
"""生成报告插图（2-1、2-4、2-5、2-6、2-7）。"""
import os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch, Rectangle

plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei"]
plt.rcParams["axes.unicode_minus"] = False

OUT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "figures"))
os.makedirs(OUT, exist_ok=True)

BLUE = "#DCE9F7"
PURPLE = "#EDE4F5"
ORANGE = "#FBE9D7"
GREEN = "#DFF0DB"
RED = "#F8D7DA"
GRAY = "#F2F2F2"
EDGE = "#2F3E4E"


def box(ax, x, y, w, h, text, fc=BLUE, fs=11, ec=EDGE, lw=1.2, bold=False, ls=1.65):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.0,rounding_size=0.02",
                                linewidth=lw, edgecolor=ec, facecolor=fc))
    ax.text(x + w / 2, y + h / 2, text, ha="center", va="center", fontsize=fs,
            color="#111111", fontweight="bold" if bold else "normal", linespacing=ls)


def arrow(ax, x1, y1, x2, y2, ls="-", color="#222222", lw=1.2, style="-|>", rad=0.0):
    ax.add_patch(FancyArrowPatch((x1, y1), (x2, y2), arrowstyle=style, mutation_scale=14,
                                 linewidth=lw, color=color, linestyle=ls,
                                 connectionstyle="arc3,rad=%s" % rad, shrinkA=0, shrinkB=0))


def label(ax, x, y, text, fs=9, ha="center", va="center", rot=0, color="#222222"):
    ax.text(x, y, text, fontsize=fs, ha=ha, va=va, rotation=rot, color=color, linespacing=1.5,
            bbox=dict(boxstyle="round,pad=0.2", fc="white", ec="none", alpha=0.9))


def canvas(w, h, xlim=100, ylim=100):
    fig = plt.figure(figsize=(w, h), dpi=200)
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, xlim)
    ax.set_ylim(0, ylim)
    ax.axis("off")
    return fig, ax


def save(fig, name):
    path = os.path.join(OUT, name)
    fig.savefig(path, dpi=200, facecolor="white")
    plt.close(fig)
    print("saved", path)


# ---------------------------------------------------------------- 图 2-1
def fig_2_1():
    fig, ax = canvas(8.5, 5.4)
    box(ax, 25, 86, 50, 11, "ESP32 人机交互单元\nST7789 240×320 屏 · 4 个按键 · Wi-Fi 手机网页", PURPLE, 11)

    arrow(ax, 50, 86, 50, 79, style="<|-|>", lw=1.4)
    ax.text(51.5, 84.5, "UART 115200 8N1", fontsize=9.5, ha="left", va="center")
    ax.text(51.5, 81.4, "命令 5 B ／ 遥测 34 B @100 Hz", fontsize=9.5, ha="left", va="center")

    ax.add_patch(Rectangle((3, 44), 94, 35, linewidth=1.4, edgecolor=EDGE, facecolor="#FAFBFC"))
    ax.text(5, 76.2, "FPGA  GW2A-18（50 MHz 单时钟域）", fontsize=12.5, fontweight="bold",
            ha="left", va="center")

    box(ax, 6, 58, 21, 14, "角度采集 adc.v\n12.5 MHz 采样\n1 ms 内 16 点平均", BLUE, 10)
    box(ax, 29, 58, 21, 14, "编码器解码 encoder.v\n两级同步 + 100 μs 滤波\n四倍频计数", BLUE, 10)
    box(ax, 52, 58, 21, 14, "串级 PID（pid_core×2）\n倾角环 200 Hz\n转角环 20 Hz", BLUE, 10)
    box(ax, 75, 58, 20, 14, "起摆 swing_up.v\n踢动 + 能量泵\nv_need 限速与捕获", BLUE, 10)

    box(ax, 6, 46.5, 44, 8.5, "运行状态机 / 倒杆保护 / 输出选择（top.v）", GRAY, 10)
    box(ax, 52, 46.5, 43, 8.5, "旋转轨迹 ramp_move.v + sine_lut.v + 前馈", GRAY, 10)

    box(ax, 4, 8, 23, 16, "WDD35D4 角度传感器\n\nAD9200 10 bit 并行 ADC", BLUE, 10)
    box(ax, 31, 8, 23, 16, "增量式编码器 11 线\n30∶1 减速\n1320 counts／摆臂圈", BLUE, 10)
    box(ax, 60, 8, 18, 16, "TB6612FNG\n单路 H 桥\n20 kHz PWM", ORANGE, 10)
    box(ax, 82, 8, 14, 16, "直流减速\n电机\n摆臂 + 摆杆", ORANGE, 10)

    arrow(ax, 15.5, 24, 15.5, 44)
    arrow(ax, 42.5, 24, 42.5, 44)
    arrow(ax, 69, 44, 69, 24)
    arrow(ax, 78, 16, 82, 16)
    label(ax, 17.0, 34, "10 bit 并行", 9, ha="left")
    label(ax, 44.0, 34, "A / B 相", 9, ha="left")
    label(ax, 70.5, 34, "PWM + 方向", 9, ha="left")

    ax.add_patch(FancyArrowPatch((89, 8), (89, 5.2), arrowstyle="-", linewidth=1.0,
                                 linestyle=(0, (4, 3)), color="#7A5C2E"))
    ax.add_patch(FancyArrowPatch((89, 5.2), (15.5, 5.2), arrowstyle="-", linewidth=1.0,
                                 linestyle=(0, (4, 3)), color="#7A5C2E"))
    ax.add_patch(FancyArrowPatch((15.5, 5.2), (15.5, 8), arrowstyle="-|>", linewidth=1.0,
                                 linestyle=(0, (4, 3)), color="#7A5C2E", mutation_scale=12))
    ax.text(52, 2.2, "机械耦合：摆杆倾角 θ → 电位器；摆臂转角 φ → 编码器",
            fontsize=8.6, ha="center", va="center", color="#7A5C2E")
    save(fig, "fig_2_1.png")


# ---------------------------------------------------------------- 图 2-4
def fig_2_4():
    fig, ax = canvas(8.6, 5.4)
    box(ax, 27, 90, 46, 7.5, "top.v（顶层：模块例化、状态机、输出选择）", GRAY, 11.5, bold=True)

    groups = [
        (1.5, 59, 23, 25, "感知层", "adc.v：12.5 MHz 采样、\n16 点平均\nencoder.v：两级同步、\n100 μs 滤波、四倍频\nkey_filter ×3：20 ms", BLUE),
        (26.0, 59, 23, 25, "控制层", "pid_core：倾角环 5 ms\npid_core：转角环 50 ms\nswing_up.v：踢动、能量泵、\nv_need 限速、顶部捕获", BLUE),
        (50.5, 59, 23, 25, "轨迹层", "rotate_ctrl.v\nramp_move.v：梯形斜坡\nsine_lut ×2：0.5 Hz 正弦\n速度前馈 / 摩擦前馈", GREEN),
        (75.0, 59, 23.5, 25, "通信层", "angle_cmd.v + uart_rx.v\n5 B 命令解帧\ntelemetry.v + uart_tx.v\n34 B 帧、CRC16、100 Hz", PURPLE),
    ]
    for x, y, w, h, title, body, fc in groups:
        ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0,rounding_size=0.02",
                                    linewidth=1.2, edgecolor=EDGE, facecolor=fc))
        ax.plot([x, x + w], [y + h - 5, y + h - 5], color=EDGE, lw=0.8)
        ax.text(x + w / 2, y + h - 2.5, title, fontsize=11, fontweight="bold",
                ha="center", va="center")
        ax.text(x + w / 2, y + (h - 5.5) / 2, body, fontsize=8.0, ha="center", va="center",
                linespacing=1.7)
        arrow(ax, x + w / 2, 90, x + w / 2, y + h, lw=1.0)

    box(ax, 1.5, 33, 46, 17, "执行与安全（top.v 内）\npwm_20k_1pct.v：20 kHz、1% 步进、周期边界锁存\n"
        "三状态机 S_IDLE / S_SWING / S_BALANCE\n倒杆越界与起摆超时保护、停机强制输出归零", ORANGE, 9.2)
    box(ax, 51, 33, 47.5, 17, "时基与同步（top.v 内）\n50 MHz 唯一时钟；1 ms 时基分频出 5 ms / 50 ms 使能\n"
        "ADC 数据在采样时钟下降沿锁存后同步入 50 MHz 域\n按键 20 ms 消抖、拨码开关 10 ms 去抖", BLUE, 9.2)
    arrow(ax, 24.5, 59, 24.5, 50, lw=1.0)
    arrow(ax, 74.5, 59, 74.5, 50, lw=1.0)

    box(ax, 3, 14, 94, 12, "数据通路：adc.v → 倾角环 → 输出选择 → PWM；encoder.v → 转角环 → 倾角目标；\n"
        "angle_cmd → rotate_ctrl → 转角环目标；各环状态 → telemetry → ESP32", GRAY, 9.4)
    arrow(ax, 50, 33, 50, 26, lw=1.0)
    save(fig, "fig_2_4.png")


# ---------------------------------------------------------------- 图 2-5
def fig_2_5():
    fig, ax = canvas(6.4, 6.9)
    box(ax, 20, 90, 60, 7.5, "上电 boot.py：按“抢连续内存”顺序初始化", GRAY, 10.5, bold=True)
    steps = [
        "① UART2（GPIO21/22，115200，rxbuf 1 KB）",
        "② Wi-Fi 连接（STA 连热点 / AP 自建热点）",
        "③ 硬件 SPI（20 MHz，ST7789 240×320）",
        "④ HTTP 监听 :80 ／ import tmp",
    ]
    prev_bottom = 90
    y = 78
    for s in steps:
        box(ax, 24, y, 52, 7, s, BLUE, 9.5)
        arrow(ax, 50, prev_bottom, 50, y + 7, lw=1.0)
        prev_bottom = y
        y -= 9.5

    box(ax, 18, 36, 64, 8, "主循环（约 5 ms 一轮，全程非阻塞）", GRAY, 10.5, bold=True)
    arrow(ax, 50, prev_bottom, 50, 44, lw=1.0)

    box(ax, 1, 22, 31, 9, "4 个按键\n中断记沿 + 30 ms 消抖", BLUE, 8.8)
    box(ax, 34.5, 22, 31, 9, "uart.poll()\n只取最新一帧、CRC16", BLUE, 8.8)
    box(ax, 68, 22, 31, 9, "web.poll()\n单连接、每轮 ≤512 B", BLUE, 8.8)
    for x in (16.5, 50, 83.5):
        arrow(ax, x, 36, x, 31, lw=1.0)

    box(ax, 1, 5, 31, 11, "表盘界面\n指针 30 Hz 刷新\n底部常驻 IP", GREEN, 8.8)
    box(ax, 34.5, 5, 31, 11, "旋转角设置\n符号/百/十/个位\n长按 MENU 下发", GREEN, 8.8)
    box(ax, 68, 5, 31, 11, "手机网页 GET /s?x=±N\n复用 uart 组帧\n回 JSON {ok:1}", PURPLE, 8.8)
    for x in (16.5, 50, 83.5):
        arrow(ax, x, 22, x, 16, lw=1.0)
    ax.text(50, 2.4, "wifi.maintain()：掉线后每 10 s 重连一次，不阻塞刷屏与收帧",
            fontsize=8.4, ha="center", va="center", color="#444444")
    save(fig, "fig_2_5.png")


# ---------------------------------------------------------------- 图 2-6
def fig_2_6():
    fig, ax = canvas(9.0, 4.6, 100, 100)
    Y = 74
    HB = 13

    box(ax, 1, Y - HB / 2, 10, HB, "φ_set\n目标转角", GRAY, 8.6)
    ax.add_patch(plt.Circle((15.5, Y), 3.0, fill=False, ec=EDGE, lw=1.2))
    ax.text(15.5, Y, "+", fontsize=11, ha="center", va="center")
    arrow(ax, 11, Y, 12.5, Y)

    box(ax, 20, Y - HB / 2, 17, HB, "转角环 PID\n20 Hz（50 ms）\n0.40 / 0 / 1.50", BLUE, 8.6)
    arrow(ax, 18.5, Y, 20, Y)
    box(ax, 39, Y - HB / 2, 9, HB, "限幅\n±60 码", GRAY, 8.4)
    arrow(ax, 37, Y, 39, Y)

    ax.add_patch(plt.Circle((52, Y), 3.0, fill=False, ec=EDGE, lw=1.2))
    ax.text(52, Y, "+", fontsize=11, ha="center", va="center")
    arrow(ax, 48, Y, 49, Y)
    label(ax, 48.9, Y + 4.6, "θ_ref", 8.2, ha="right")

    box(ax, 56, Y - HB / 2, 18, HB, "倾角环 PID\n200 Hz（5 ms）\n0.30 / 0.01 / 0.40", BLUE, 8.6)
    arrow(ax, 55, Y, 56, Y)
    box(ax, 77, Y - HB / 2, 9, HB, "限幅\n±100", GRAY, 8.4)
    arrow(ax, 74, Y, 77, Y)
    box(ax, 89, Y - HB / 2, 10, HB, "PWM\n20 kHz\n1% 步进", ORANGE, 8.4)
    arrow(ax, 86, Y, 89, Y)

    box(ax, 64, 36, 22, 11, "TB6612FNG + 直流减速电机\n30∶1 减速器 → 摆臂转角 φ", ORANGE, 8.4)
    box(ax, 36, 36, 24, 11, "摆杆（倾角 θ）\n欠驱动、开环不稳定", GREEN, 8.8)
    arrow(ax, 94, 67.5, 94, 41.5, lw=1.2)
    arrow(ax, 94, 41.5, 86, 41.5, lw=1.2)
    arrow(ax, 64, 41.5, 60, 41.5, lw=1.2)

    # θ 测量：摆杆 → 传感器 → 倾角环相加点
    box(ax, 36, 20, 24, 10, "角度传感器 + AD9200\n1 ms 16 点平均", BLUE, 8.6)
    arrow(ax, 48, 36, 48, 30, lw=1.2)
    arrow(ax, 60, 25, 62, 25, style="-", lw=1.2)
    arrow(ax, 62, 25, 62, 55, style="-", lw=1.2)
    arrow(ax, 62, 55, 52, 55, style="-", lw=1.2)
    arrow(ax, 52, 55, 52, 71, lw=1.2)
    label(ax, 57.2, 57.6, "θ 反馈", 8.2)

    # φ 测量：电机尾轴编码器 → 转角环相加点
    box(ax, 64, 20, 22, 10, "增量式编码器\n1320 counts／圈 → φ", BLUE, 8.6)
    arrow(ax, 75, 36, 75, 30, lw=1.2)
    arrow(ax, 75, 20, 75, 10, lw=1.2)
    arrow(ax, 75, 10, 15.5, 10, lw=1.2)
    arrow(ax, 15.5, 10, 15.5, 71, lw=1.2)
    label(ax, 16.8, 46, "φ 反馈", 8.2, ha="left", rot=90)
    ax.text(13.4, Y - 5.0, "−", fontsize=11, ha="center", va="center")

    box(ax, 4, 86, 36, 10, "ramp_move 梯形斜坡 + 速度前馈\n27°/s，1000 counts/s²，限幅 ±12 码", PURPLE, 8.6)
    arrow(ax, 40, 91, 52, 91, ls="--", lw=1.0, style="-")
    arrow(ax, 52, 91, 52, 77.2, ls="--", lw=1.0)
    label(ax, 53.4, 84, "前馈叠加", 8.2, ha="left")
    save(fig, "fig_2_6.png")


# ---------------------------------------------------------------- 图 2-7
def fig_2_7():
    fig, ax = canvas(7.0, 4.6)
    box(ax, 16, 74, 38, 16, "S_IDLE  待机\n电机刹车 · 两级 PID 清零\n检测自然下垂 / 等待 K1", GRAY, 10)
    box(ax, 16, 44, 38, 16, "S_SWING  起摆\n35%×100 ms 踢动 + 能量泵\nv_need 限速，不受倒杆保护", ORANGE, 10)
    box(ax, 16, 14, 38, 16, "S_BALANCE  平衡\n倾角环 200 Hz · 转角环 20 Hz\n斜坡轨迹与前馈使能", GREEN, 10)

    arrow(ax, 35, 74, 35, 60)
    label(ax, 36.5, 67, "自动触发：\n自然下垂 0.5 s 或按 K1", 8.6, ha="left")
    arrow(ax, 35, 44, 35, 30)
    label(ax, 36.5, 37, "捕获成功：|e|<300 码\n|v|≤8（counts/5 ms）×3 次", 8.6, ha="left")

    arrow(ax, 16, 52, 6, 52, style="-", lw=1.2)
    arrow(ax, 6, 52, 6, 82, style="-", lw=1.2)
    arrow(ax, 6, 82, 16, 82, style="-|>", lw=1.2)
    label(ax, 2.6, 67, "起摆超时 4 s（armed=0）", 8.0, ha="center", rot=90)

    arrow(ax, 54, 22, 92, 22, style="-", lw=1.2)
    arrow(ax, 92, 22, 92, 82, style="-", lw=1.2)
    arrow(ax, 92, 82, 54, 82, style="-|>", lw=1.2)
    label(ax, 96.0, 55, "倒杆越界 ±500 码或按 K1", 8.0, ha="center", rot=90)
    save(fig, "fig_2_7.png")


# ---------------------------------------------------------------- 图 2-8 网络化控制
def fig_2_8():
    fig, ax = canvas(7.8, 4.4)

    ax.text(50, 95.5,
            "上行：遥测 34 B @100 Hz（全状态 + 序号 + 毫秒时标 + CRC16）　下行：命令 5 B（目标角度 / 模式）",
            fontsize=9.0, ha="center", va="center")

    # 三条纵向分层
    ax.add_patch(Rectangle((2, 24), 28, 66, facecolor="#F7FBFE", edgecolor="#9DB4C8",
                           linewidth=1.0, linestyle=(0, (5, 3))))
    ax.add_patch(Rectangle((36, 24), 28, 66, facecolor="#FBF8FE", edgecolor="#B4A6C8",
                           linewidth=1.0, linestyle=(0, (5, 3))))
    ax.add_patch(Rectangle((70, 24), 28, 66, facecolor="#F8F9FA", edgecolor="#A8A8A8",
                           linewidth=1.0, linestyle=(0, (5, 3))))
    ax.text(16, 86.2, "现场实时层\n（确定性）", fontsize=11.5, fontweight="bold", ha="center", va="center")
    ax.text(50, 86.2, "边缘网关层\n（尽力而为）", fontsize=11.5, fontweight="bold", ha="center", va="center")
    ax.text(84, 86.2, "网络与云端", fontsize=11.5, fontweight="bold", ha="center", va="center")

    box(ax, 4, 62, 24, 14, "FPGA GW2A-18 · 50 MHz\n双环 5 ms / 50 ms\nPWM 20 kHz、1% 步进", BLUE, 9.6, ls=1.45)
    box(ax, 4, 45, 24, 14, "指令整形 + 看门狗\n±180° → 斜坡 → ±60 码\n倒杆保护与状态机", ORANGE, 9.6, ls=1.45)
    box(ax, 4, 28, 24, 14, "遥测锁存\n34 B ／ 10 ms\n（序号 + 毫秒时标）", BLUE, 9.6, ls=1.45)

    box(ax, 38, 62, 24, 14, "ESP32 · MicroPython\nUART 115200\n34 B 解帧 · CRC16", PURPLE, 9.6, ls=1.45)
    box(ax, 38, 45, 24, 14, "HTTP / WebSocket / MQTT\n单连接、非阻塞\n页面流式发送", PURPLE, 9.6, ls=1.45)
    box(ax, 38, 28, 24, 14, "指令翻译与二次整形\n5 B 帧 · 断线 10 s 重连\n二次限幅", PURPLE, 9.6, ls=1.45)

    box(ax, 72, 62, 24, 14, "Wi-Fi / 5G /\n工业以太网", GRAY, 9.8)
    box(ax, 72, 45, 24, 14, "手机网页 · 上位机\n曲线 / 回放\n参数整定", GREEN, 9.6, ls=1.45)
    box(ax, 72, 28, 24, 14, "时序数据库\n数字孪生 / MQTT Broker", GREEN, 9.6, ls=1.45)

    arrow(ax, 33, 56, 33, 62, style="<|-|>", lw=1.4)
    arrow(ax, 67, 56, 67, 62, style="<|-|>", lw=1.4)

    # AI 接入层
    ax.add_patch(FancyBboxPatch((2, 5), 96, 13, boxstyle="round,pad=0,rounding_size=0.02",
                                linewidth=1.2, edgecolor="#7A5C2E", facecolor="#FDF6E9"))
    ax.text(50, 14.4, "AI 控制接入（设计）：仿真训练 → 策略评估 → 参考轨迹 / 前馈 / 增益组下发",
            fontsize=10.2, ha="center", va="center", fontweight="bold")
    ax.text(50, 9.4, "影子模式先跑、安全包络兜底：输出仍经限幅、斜坡、看门狗，可随时回退到 PID",
            fontsize=9.2, ha="center", va="center")
    arrow(ax, 50, 18, 50, 24, ls="--", lw=1.0)
    arrow(ax, 84, 18, 84, 24, ls="--", lw=1.0)
    ax.text(51.2, 21.2, "策略 / 参数", fontsize=8.2, ha="left", va="center",
            bbox=dict(boxstyle="round,pad=0.15", fc="white", ec="none"))

    ax.text(50, 1.4, "网络延迟与丢包不进 5 ms 内环：断链时保持当前目标，本地闭环继续平衡",
            fontsize=9.2, ha="center", va="center", color="#8A3B2E")
    save(fig, "fig_2_8.png")


if __name__ == "__main__":
    fig_2_1()
    fig_2_4()
    fig_2_5()
    fig_2_6()
    fig_2_7()
    fig_2_8()





