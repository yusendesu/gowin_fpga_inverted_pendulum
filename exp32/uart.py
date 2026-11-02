"""
UART 通信：接收摆杆状态帧，更新表盘指针；发送旋转角设置指令。

接收帧为 34 字节：
    [0:2]   0xAA 0x55
    [2:4]   angle_raw，12 位绝对角度，高字节在前（0..4095）
    ...
    [6:8]   encoder_pos，高字节在前
    ...
    [30:32] CRC16/CCITT-FALSE，低字节在前，覆盖 byte 0..29
    [32]    0x0D
    [33]    0x0A

角度来源：
    USE_ANGLE_RAW = True 时使用 [2:4] 的 12 位绝对角度；
    设为 False 时改用 [6:8] 的 1320 线编码器。
    实测当前传感器 [6:8] 固定为 0x7FFF（约 -0.27°），不会随摆杆变化，
    而 [2:4] 会随摆杆位置变化，所以默认使用 angle_raw。

12 位绝对角度参数：
    一圈 4096 个计数。
    角度 = (angle_raw - 0) % 4096 / 4096 * 360
    并归一化到 [-180, 180)。

编码器参数（USE_ANGLE_RAW = False 时使用）：
    一圈 1320 个脉冲，零位计数值为 32768。
    角度 = (encoder_pos - 32768) % 1320 / 1320 * 360
    并归一化到 [-180, 180)。

旋转指令（5 字节）：
    0xAA 0x55 sign angle 0x0D
    sign  : 0x00 正，0x01 负
    angle : 0x00..0xB4（0..180）

性能要点：
    * 发送端 100 Hz 发帧，界面只需要 30 Hz。主循环偶尔被刷屏拖慢时，
      缓冲区里会积压很多旧帧；这里只解析“最新一帧”，旧数据直接丢，
      不会越积越多把主循环拖死。
    * CRC 用 256 项查表，一次校验 30 次循环（原来 240 次）。
    * 逐帧打印会阻塞串口（没人读的时候会直接卡死主循环），默认关闭。

用法：
    import uart
    link = uart.init(view)
    link.poll()                 # 在主循环中反复调用
    link.send_rotation(-90)     # 需要时发送
"""

import time

from machine import UART, Pin


# ---------------------------------------------------------------------------
# 硬件配置
# ---------------------------------------------------------------------------

UART_ID = 2
UART_BAUD = 115200
UART_TX_PIN = 21
UART_RX_PIN = 22
# 接收缓冲。默认只有 256 字节（约 7 帧），主循环卡一下就会溢出丢数据。
UART_RX_BUFFER = 1024

# ---------------------------------------------------------------------------
# 协议与机械参数
# ---------------------------------------------------------------------------

RX_FRAME_LEN = 34
RX_HEADER = b"\xAA\x55"
# 缓冲区最多保留 4 帧，多出来的旧帧直接扔掉。
RX_MAX_BUFFER = RX_FRAME_LEN * 4

ENCODER_PPR = 1320
ENCODER_ZERO = 32768
ENCODER_DIRECTION = 1       # 安装方向相反时改为 -1

# 角度来源：True = [2:4] 的 12 位绝对角度；False = [6:8] 的 1320 线编码器。
USE_ANGLE_RAW = False
ANGLE_RAW_COUNTS = 4096
ANGLE_RAW_ZERO = 0          # 摆杆在 0° 时的原始值，可按实际零点修改
ANGLE_RAW_DIRECTION = 1     # 方向相反时改为 -1

# 串口每 10 ms 发一帧，界面不需要按 100 Hz 刷新
UI_UPDATE_MS = 33

ROT_SIGN_POSITIVE = 0x00
ROT_SIGN_NEGATIVE = 0x01

# 调试开关。
# DEBUG     = True 时每 DEBUG_MS 打印一行统计（不会明显影响主循环）。
# DEBUG_RAW = True 时逐帧打印原始十六进制，只适合短时间排查，很慢。
DEBUG = False
DEBUG_RAW = False
DEBUG_MS = 500


# ---------------------------------------------------------------------------
# 尽早装驱动
# ---------------------------------------------------------------------------

_hw = None


def open_hw():
    """尽早把 UART 驱动装上，返回 UART 对象。

    必须在 import ui、分配 UI 大缓冲之前调用（也就是在 boot.py 里）。

    UART 驱动的接收环形缓冲要从 **DMA 可访问的内部 RAM** 里分配，而
    MicroPython 的堆是按需从同一块内部 RAM 增长的。等 WiFi、字库、字节码、
    UI 缓冲吃掉 80 多 KB 之后再装驱动，ESP-IDF 就 malloc 不到：

        E (22639) uart: UART driver malloc error
        OSError: [Errno 1] EPERM: ESP_FAIL

    注意 ESP_FAIL 被 MicroPython 笼统地翻成了 EPERM，看着像权限问题，
    其实是没有连续内存。gc.mem_free() 报的几十 KB 是堆内部的空闲量，
    和这里能不能拿到 DMA 内存是两回事。
    """
    global _hw
    if _hw is not None:
        return _hw

    try:
        _hw = UART(
            UART_ID,
            baudrate=UART_BAUD,
            tx=Pin(UART_TX_PIN),
            rx=Pin(UART_RX_PIN),
            rxbuf=UART_RX_BUFFER,
            timeout=0,
        )
    except TypeError:
        # 个别固件不支持 rxbuf 参数，退回默认缓冲。
        _hw = UART(
            UART_ID,
            baudrate=UART_BAUD,
            tx=Pin(UART_TX_PIN),
            rx=Pin(UART_RX_PIN),
            timeout=0,
        )
    print("[uart] driver ok")
    return _hw


# ---------------------------------------------------------------------------
# CRC16/CCITT-FALSE：poly=0x1021, init=0xFFFF，不反射
# ---------------------------------------------------------------------------

def _make_crc_table():
    table = []
    for value in range(256):
        crc = value << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
        table.append(crc)
    return table


_CRC_TABLE = _make_crc_table()


def crc16_ccitt(data, start=0, end=30):
    """CRC-16/CCITT-FALSE，查表实现。"""
    crc = 0xFFFF
    table = _CRC_TABLE
    for i in range(start, end):
        crc = ((crc << 8) & 0xFFFF) ^ table[((crc >> 8) ^ data[i]) & 0xFF]
    return crc


def _hex(data):
    """把字节串格式化为空格分隔的大写十六进制。"""
    return " ".join(["%02X" % byte for byte in data])


def encoder_to_angle(encoder_pos):
    """把 16 位编码器原始计数转换为 [-180, 180) 的机械角度。"""
    counts = (encoder_pos - ENCODER_ZERO) % ENCODER_PPR
    angle = (counts * 1080.0 / ENCODER_PPR) % 360.0
    if angle >= 180.0:
        angle -= 360.0
    return angle * ENCODER_DIRECTION


def angle_raw_to_angle(raw):
    """把 12 位绝对角度原始值转换为 [-180, 180) 的机械角度。"""
    counts = (raw - ANGLE_RAW_ZERO) % ANGLE_RAW_COUNTS
    angle = counts * 360.0 / ANGLE_RAW_COUNTS
    if angle >= 180.0:
        angle -= 360.0
    return angle * ANGLE_RAW_DIRECTION


def build_rotation_command(angle):
    """构造旋转角设置指令。angle 为带符号角度，范围 -180..180。"""
    try:
        value = int(round(float(angle) / 2))
    except (TypeError, ValueError):
        value = 0

    if value > 180:
        value = 180
        
    u16 = value & 0xFFFF
    high = (u16 >> 8) & 0xFF
    low = u16 & 0xFF

    return bytes((0xAA, 0x55, high, low, 0x0D))

class UARTLink:
    """串口接收解析与发送。"""

    def __init__(self, view, uart=None):
        self.view = view
        if uart is None:
            try:
                self.uart = UART(
                    UART_ID,
                    baudrate=UART_BAUD,
                    tx=Pin(UART_TX_PIN),
                    rx=Pin(UART_RX_PIN),
                    rxbuf=UART_RX_BUFFER,
                    timeout=0,
                )
            except TypeError:
                # 个别固件不支持 rxbuf 参数，退回默认缓冲。
                self.uart = UART(
                    UART_ID,
                    baudrate=UART_BAUD,
                    tx=Pin(UART_TX_PIN),
                    rx=Pin(UART_RX_PIN),
                    timeout=0,
                )
        else:
            self.uart = uart

        self._buf = bytearray()
        self._last_ui_ms = time.ticks_ms() - UI_UPDATE_MS
        self._debug_ms = time.ticks_ms()
        self.angle = 0.0
        self.angle_raw = 0
        self.rx_frames = 0
        self.bad_frames = 0
        self.last_encoder = ENCODER_ZERO

    # ------------------------------------------------------------------
    # 接收
    # ------------------------------------------------------------------

    def poll(self):
        """读取当前可用的全部串口数据，用其中最新的一帧刷新界面。"""
        count = self.uart.any()
        if not count:
            return

        data = self.uart.read(count)
        if not data:
            return

        if DEBUG_RAW:
            print("UART RX [%d]:" % len(data), _hex(data))

        buf = self._buf
        buf.extend(data)
        if len(buf) > RX_MAX_BUFFER:
            # 积压的都是过时的帧，只留尾部重新找帧头。
            self._buf = buf[-RX_MAX_BUFFER:]

        self._parse()

        if DEBUG:
            now = time.ticks_ms()
            if time.ticks_diff(now, self._debug_ms) >= DEBUG_MS:
                self._debug_ms = now
                print("UART rx=%d bad=%d angle=%.1f"
                      % (self.rx_frames, self.bad_frames, self.angle))

    def _parse(self):
        """从缓冲区里挑出最后一条完整且校验通过的帧。"""
        buf = self._buf
        length = len(buf)
        if length < RX_FRAME_LEN:
            return

        start = -1
        i = length - RX_FRAME_LEN
        while i >= 0:
            if buf[i] == 0xAA and buf[i + 1] == 0x55:
                if self._decode(buf, i):
                    start = i
                    break
                self.bad_frames += 1
            i -= 1

        if start < 0:
            # 没有完整有效帧：只保留最后一个可能是帧头的字节。
            if buf[length - 1] == 0xAA:
                self._buf = bytearray((0xAA,))
            else:
                self._buf = bytearray()
            return

        self._buf = buf[start + RX_FRAME_LEN:]
        self._update_ui()

    def _decode(self, buf, start):
        """校验并解析 start 处的一帧，成功返回 True。"""
        if buf[start + 32] != 0x0D or buf[start + 33] != 0x0A:
            return False

        expected = buf[start + 30] | (buf[start + 31] << 8)
        if expected != crc16_ccitt(buf, start, start + 30):
            return False

        raw = (buf[start + 2] << 8) | buf[start + 3]
        encoder = (buf[start + 6] << 8) | buf[start + 7]
        self.angle_raw = raw
        self.last_encoder = encoder
        if USE_ANGLE_RAW:
            self.angle = angle_raw_to_angle(raw)
        else:
            self.angle = encoder_to_angle(encoder)
        self.rx_frames += 1
        return True

    def _update_ui(self):
        now = time.ticks_ms()
        if time.ticks_diff(now, self._last_ui_ms) >= UI_UPDATE_MS:
            self._last_ui_ms = now
            self.view.set_angle(self.angle)

    # ------------------------------------------------------------------
    # 发送
    # ------------------------------------------------------------------

    def send_rotation(self, angle):
        """发送旋转角设置指令，返回实际写入的字节串。"""
        command = build_rotation_command(angle)
        if DEBUG or DEBUG_RAW:
            print("UART TX [%d]:" % len(command), _hex(command))
        self.uart.write(command)
        return command


# ---------------------------------------------------------------------------
# 模块级便捷接口
# ---------------------------------------------------------------------------

_link = None


def init(view, hw=None):
    """创建串口对象并返回 UARTLink；调用前先初始化 ui。

    hw 一般不用传：boot.py 已经用 open_hw() 把驱动装好了。没装过的话
    这里会自己装（REPL 里单独调试时就是这条路）。
    """
    global _link
    if hw is None:
        hw = _hw
    _link = UARTLink(view, hw)
    return _link


def poll():
    """在主循环中反复调用。"""
    if _link is not None:
        _link.poll()


def send_rotation(angle):
    """发送旋转角设置指令。"""
    if _link is None:
        raise RuntimeError("uart.init(view) must be called first")
    return _link.send_rotation(angle)
