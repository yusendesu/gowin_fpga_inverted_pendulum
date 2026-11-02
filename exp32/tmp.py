"""
倒立摆位置环主程序。

按键（全部接 GND，内部上拉，按下为低电平）：
    MENU: 表盘界面按下 -> 打开旋转角设置
          设置界面短按 -> 退出设置
          设置界面按住 >= 1 秒后松开 -> 通过 UART 发送旋转角，
                                       短暂显示 SENT 后返回表盘
    UP:   设置界面中增大当前字符
    DOWN: 设置界面中减小当前字符
    ALT:  切换当前编辑的字符（符号/百位/十位/个位）

性能要点（摆杆快速转动时界面卡顿、按键失灵的原因）：
    * 屏幕改走硬件 SPI。14/13/12/15 正好是 ESP32 HSPI 的默认引脚，
      能走 IOMUX 全速；原来的 SoftSPI 每帧要传几十 KB，实测只能跑到
      几帧每秒，主循环被拖到一百多毫秒一轮，按键自然扫不到。
    * 按键的按下/松开沿由 Pin 中断记录时间戳，主循环只负责取事件，
      再快的点按也不会因为一次刷屏而整次丢掉。
    * SENT 提示不再用 while 自旋等待 600 ms，改成主循环里倒计时，
      等待期间照常扫键、收帧。

UART 接收和发送逻辑见 uart.py。

手机网页控制：
    热点连接在 boot.py 里、import tmp 之前完成（顺序原因见 boot.py 的
    注释），这里只取结果；屏幕底部会常驻显示 IP。
    手机浏览器打开 http://<IP>/ 就是控制页，按钮走 GET /s?x=角度，
    由 web.py 处理。整条路径都是非阻塞的，每次 poll 最多收发几百字节，
    不会拖慢刷屏和串口收帧。
"""

import time

import board
import st7789
from machine import Pin, SPI, SoftSPI

import vga1_16x32 as big_font

import ui
import uart
import web
import wifi


# ---------------------------------------------------------------------------
# 硬件配置（按实际接线修改）
# ---------------------------------------------------------------------------

# ST7789 SPI 屏幕
PIN_SCK = 14
PIN_MOSI = 13
PIN_MISO = 12
PIN_CS = 15
PIN_DC = 2
PIN_RESET = 4
PIN_BACKLIGHT = 32

# SPI 驱动：
#   "hard" = 硬件 SPI（HSPI），快，正常用这个
#   "soft" = 软件 SPI，慢，但时序容错高（就是以前能亮的那套）
# 排查用：屏一点不亮时切到 "soft"，能亮就说明屏和接线没问题，
# 问题出在硬件 SPI 的时序/频率上。
SPI_DRIVER = "hard"
# HSPI = SPI(1)，默认引脚就是 14/13/12/15（可以跑 IOMUX 全速）。
# 先用 10 MHz 跑通，稳定后再试 20000000 / 26666666 / 40000000；
# 出花屏、错位、黑屏就退回上一个能用的值。
SPI_ID = 1
SPI_BAUD = 20000000
SPI_SOFT_BAUD = 2000000
# ST7789 的参考时序是 mode 0；个别模块认 mode 3，可以改成 (1, 1) 试。
SPI_POLARITY = 0
SPI_PHASE = 0

# 按键
PIN_MENU = 16
PIN_UP = 17
PIN_DOWN = 18
PIN_ALT = 19

BUTTON_DEBOUNCE_MS = 30
MENU_LONG_MS = 1000
# 发送成功后保持显示 SENT 的时长
SENT_HOLD_MS = 600

MODE_ANGLE = 0
MODE_ROTATION = 1

# 设置界面的四个字符位置
SLOT_SIGN = 0
SLOT_HUNDREDS = 1
SLOT_TENS = 2
SLOT_UNITS = 3
SLOT_COUNT = 4


class Button:
    """上拉、低有效、带消抖的按键。

    按下沿和松开沿都在 Pin 中断里记录时间戳，主循环只调用 scan() 把事件
    取出来。这样即使一次刷屏花掉几十毫秒，期间发生的点按也不会丢
    （旧版只在主循环里轮询，采样不到就整次丢掉）。

    scan() 里另外保留了轮询兜底：万一中断被消抖过滤或调度不及时，就按
    当前电平再判一次，保证不会卡在“一直按下”的状态。
    """

    def __init__(self, pin_number, name):
        self.name = name
        self.pin = Pin(pin_number, Pin.IN, Pin.PULL_UP)

        now = time.ticks_ms()
        self._edge_ms = now          # 上一次被消抖接受的边沿时刻
        self._fall_ms = now          # 最近一次按下沿时刻
        self._rise_ms = now          # 最近一次松开沿时刻
        self._fall_count = 0
        self._rise_count = 0
        self._seen_fall = 0
        self._seen_rise = 0

        self._raw = 1                # 轮询兜底用
        self._raw_since = now
        self._down = False
        self._press_ms = now
        self._swallow = False

        self.pressed = False
        self.released = False
        self.hold_ms = 0

        self.pin.irq(
            handler=self._on_edge,
            trigger=Pin.IRQ_FALLING | Pin.IRQ_RISING,
        )

    def _on_edge(self, pin):
        """中断里只记时间戳，不绘制、不分配内存。"""
        now = time.ticks_ms()
        if time.ticks_diff(now, self._edge_ms) < BUTTON_DEBOUNCE_MS:
            return
        self._edge_ms = now
        if pin.value() == 0:
            self._fall_ms = now
            self._fall_count += 1
        else:
            self._rise_ms = now
            self._rise_count += 1

    def scan(self, now):
        self.pressed = False
        self.released = False

        # 1) 中断记下的按下沿。
        if self._fall_count != self._seen_fall:
            self._seen_fall = self._fall_count
            if not self._down:
                self._down = True
                self._press_ms = self._fall_ms
                self.pressed = True

        # 2) 中断记下的松开沿（带上准确的按住时长）。
        if self._rise_count != self._seen_rise:
            self._seen_rise = self._rise_count
            if self._down:
                self._down = False
                self.hold_ms = time.ticks_diff(self._rise_ms, self._press_ms)
                self.released = True

        # 3) 轮询兜底。
        raw = self.pin.value()
        if self._down:
            # 已经松开，但松开沿被消抖吃掉了：按电平收尾。
            if raw == 1 and time.ticks_diff(now, self._press_ms) >= BUTTON_DEBOUNCE_MS:
                self._down = False
                self.hold_ms = time.ticks_diff(now, self._press_ms)
                self.released = True
        elif raw != self._raw:
            self._raw = raw
            self._raw_since = now
        elif raw == 0 and time.ticks_diff(now, self._raw_since) >= BUTTON_DEBOUNCE_MS:
            # 中断没有记录到按下沿：按持续低电平补一次。
            self._down = True
            self._press_ms = self._raw_since
            self.pressed = True
        self._raw = raw

        if self._swallow and self.released:
            self._swallow = False
            self.released = False

    def swallow_release(self):
        """忽略当前这一次松手。

        用于“按下就切屏”的按键：不吞掉的话，松手会被设置界面当成短按，
        刚进去又立刻退出来。
        """
        if self.released:
            self.released = False
        else:
            self._swallow = True


def adjust_rotation(magnitude, positive, slot, delta):
    """调整旋转角，返回 (magnitude, positive)。"""
    if slot == SLOT_SIGN:
        if delta > 0:
            positive = True
        elif delta < 0:
            positive = False
        return magnitude, positive

    weights = (100, 10, 1)
    magnitude += delta * weights[slot - 1]
    if magnitude < 0:
        magnitude = 0
    elif magnitude > 180:
        magnitude = 180
    return magnitude, positive


def make_spi():
    """按 SPI_DRIVER 创建 SPI 总线。

    14/13/12 是 ESP32 HSPI 的默认引脚，硬件 SPI 走它们能用 IOMUX 全速。

    时序：ST7789 的 4 线 SPI 参考时序是 mode 0（SCL 空闲低、上升沿锁存
    SDIN）。原来的 SoftSPI 用 polarity=1/phase=0 也能亮，是因为软件 SPI
    在拉低时钟之前就把数据放到 MOSI 上了，两个边沿上数据都是稳的；
    换成硬件 SPI 后，硬件严格按 mode 2 走（上升沿换数据、下降沿采样），
    和 ST7789 的锁存时刻对不上，屏收到的都是错位数据 —— 整屏不亮。
    """
    if SPI_DRIVER == "soft":
        # 软件 SPI 保持原来那套（已实测能亮），方便和硬件 SPI 做对比。
        return SoftSPI(
            baudrate=SPI_SOFT_BAUD,
            polarity=1,
            phase=0,
            sck=Pin(PIN_SCK),
            mosi=Pin(PIN_MOSI),
            miso=Pin(PIN_MISO),
        )
    return SPI(
        SPI_ID,
        baudrate=SPI_BAUD,
        polarity=SPI_POLARITY,
        phase=SPI_PHASE,
        sck=Pin(PIN_SCK),
        mosi=Pin(PIN_MOSI),
        miso=Pin(PIN_MISO),
    )


def make_display():
    """创建 ST7789 对象；旋转 0 时驱动窗口为 240x320。

    SPI 总线由 boot.py 用 board.open_spi() 提前建好（要趁内部 RAM 还完整
    的时候抢 DMA 通道），这里只取现成的对象，不再自己 make_spi()。
    """
    return st7789.ST7789(
        board.spi(),
        320,
        240,
        reset=Pin(PIN_RESET, Pin.OUT),
        dc=Pin(PIN_DC, Pin.OUT),
        cs=Pin(PIN_CS, Pin.OUT),
        backlight=Pin(PIN_BACKLIGHT, Pin.OUT),
        rotation=0,
    )


def _boot_screen(tft, title, detail="", color=st7789.WHITE):
    """开机时在屏上显示网络状态。

    这段内容会被随后的 ui.init() 整屏覆盖，只用来告诉人“正在连 / 连上了
    / 没连上”，以及顺手把 IP 亮出来给人抄。
    """
    tft.fill(st7789.BLACK)
    tft.text(big_font, title, 8, 96, st7789.WHITE)
    if detail:
        tft.text(big_font, detail, 8, 140, color)


def main():
    if SPI_DRIVER == "soft":
        print("[boot] display: soft spi %d" % SPI_SOFT_BAUD)
    else:
        print("[boot] display: hard spi %d" % SPI_BAUD)
    tft = make_display()
    print("[boot] display ok")

    # 屏的 SPI 总线已经占掉一个 DMA 通道和一块描述符。UART 驱动也要内部
    # RAM，必须抢在 ui.init() 那 18 KB 缓冲之前装 —— 排在它后面实测就是：
    #     E uart: UART driver malloc error
    #     OSError: [Errno 1] EPERM: ESP_FAIL
    # 但这个 try 不要放开：装不上也让板子把表盘点亮，别陷在崩溃重启里。
    try:
        uart.open_hw()
    except OSError as err:
        print("[boot] uart driver fail:", err)

    # socket 已经在 boot.py 里建好了：必须趁内部 RAM 完整时建，
    # 排在 ui.init() 那 18 KB 之后就是 [Errno 105] ENOBUFS。
    web_ok = web.is_ready()

    # WiFi 已经在 boot.py 里连好了：必须早于本文件和 ui 的导入，
    # 否则 import 出来的几十 KB 会把 WiFi 驱动要的内部 DRAM 挤掉，
    # 报 OSError: WiFi Out of Memory。这里只取结果。
    net_ok = wifi.is_up()
    net_ip = wifi.ip()
    if net_ok:
        print("[boot] wifi ok", net_ip)
        _boot_screen(tft, "READY", net_ip, st7789.GREEN)
        time.sleep_ms(900)
    else:
        print("[boot] wifi fail")
        _boot_screen(tft, "NO WIFI", "check hotspot", st7789.RED)
        time.sleep_ms(2000)

    view = ui.init(tft)
    print("[boot] ui ok")

    link = None
    try:
        link = uart.init(view)
        print("[boot] uart ok")
    except OSError as err:
        # 驱动装不上时退化成“只能看不能发”，别让它把整个程序带停
        print("[boot] uart unavailable:", err)

    if web_ok and link is not None:
        web.attach(link)

    if net_ok and web_ok:
        # 表盘底部常驻显示 IP，现场排障全靠它
        view.set_footer(net_ip)

    menu = Button(PIN_MENU, "menu")
    up = Button(PIN_UP, "up")
    down = Button(PIN_DOWN, "down")
    alt = Button(PIN_ALT, "alt")
    buttons = (menu, up, down, alt)

    mode = MODE_ANGLE
    magnitude = 0
    positive = True
    slot = SLOT_SIGN
    status = None
    sent_deadline = None
    # 热点重启后 IP 可能变，屏幕底部的 IP 跟着更新（每秒查一次，很便宜）
    net_check_ms = time.ticks_ms()
    footer_ip = net_ip

    while True:
        now = time.ticks_ms()
        for button in buttons:
            button.scan(now)

        if link is not None:
            link.poll()
        web.poll()
        # 掉线后每 10 秒重踢一次连接；connect() 不阻塞，不会卡住这一轮。
        wifi.maintain()

        if time.ticks_diff(now, net_check_ms) >= 0:
            net_check_ms = time.ticks_add(now, 1000)
            if wifi.is_up():
                current_ip = wifi.ip()
                if current_ip != footer_ip:
                    # 重连后地址变了，把新的 IP 换上，别让人对着旧地址输
                    footer_ip = current_ip
                    view.set_footer(current_ip)

        if (sent_deadline is not None
                and time.ticks_diff(now, sent_deadline) >= 0):
            # SENT 显示时间到，回表盘。
            sent_deadline = None
            mode = MODE_ANGLE
            status = None
            view.show_angle_screen()

        elif mode == MODE_ANGLE:
            if menu.pressed:
                mode = MODE_ROTATION
                # 这次按下对应的松手不算“短按退出”。
                menu.swallow_release()
                status = None
                view.show_rotation_screen(
                    magnitude, positive, slot, status
                )
        else:
            changed = False

            if alt.pressed:
                slot = (slot + 1) % SLOT_COUNT
                status = None
                changed = True

            if up.pressed:
                old = (magnitude, positive)
                magnitude, positive = adjust_rotation(
                    magnitude, positive, slot, 1
                )
                if (magnitude, positive) != old:
                    status = None
                    changed = True

            if down.pressed:
                old = (magnitude, positive)
                magnitude, positive = adjust_rotation(
                    magnitude, positive, slot, -1
                )
                if (magnitude, positive) != old:
                    status = None
                    changed = True

            if menu.released:
                if menu.hold_ms >= MENU_LONG_MS:
                    signed_angle = magnitude if positive else -magnitude
                    if link is not None:
                        link.send_rotation(signed_angle)
                    # 显示 SENT，由主循环计时返回，不再阻塞等待。
                    view.show_rotation_screen(
                        magnitude, positive, slot, "SENT"
                    )
                    sent_deadline = time.ticks_add(now, SENT_HOLD_MS)
                    changed = False
                else:
                    mode = MODE_ANGLE
                    status = None
                    sent_deadline = None
                    view.show_angle_screen()
                    changed = False

            if changed:
                # 修改数值时取消 SENT 的自动返回。
                sent_deadline = None
                view.show_rotation_screen(
                    magnitude, positive, slot, status
                )

        time.sleep_ms(5)


if __name__ == "__main__":
    main()

