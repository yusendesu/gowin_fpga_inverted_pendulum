"""
倒立摆位置环 UI（240 x 320 竖屏）。

包含两个界面：
    1. 角度表盘：圆形刻度盘 + 指针，下方显示当前角度。
    2. 旋转角设置：正负号、百位、十位、个位四个可编辑字符。

用法::

    import st7789
    import ui

    tft = st7789.ST7789(..., 320, 240, rotation=0)  # 旋转后为 240x320
    view = ui.init(tft)

    view.set_angle(12.5)                         # 更新指针
    view.show_rotation_screen(90, True, 1)       # 打开设置界面
    view.show_angle_screen()                     # 返回表盘

角度单位为度，0° 指向上方，正值顺时针，负值逆时针。
"""

import gc
import math

import st7789
import vga1_16x32 as big_font


# ---------------------------------------------------------------------------
# 配色（RGB565）
# ---------------------------------------------------------------------------

_BG = st7789.color565(7, 11, 18)             # 页面背景
_PANEL = st7789.color565(13, 21, 32)         # 设置项底色
_PANEL_ACTIVE = st7789.color565(7, 39, 43)   # 当前设置项底色
_RULE = st7789.color565(32, 47, 65)          # 分隔线
_RING = st7789.color565(40, 57, 78)          # 表盘外圈
_TICK = st7789.color565(85, 107, 134)        # 小刻度
_TICK_MAJOR = st7789.color565(176, 200, 228) # 大刻度
_ACCENT = st7789.color565(0, 216, 190)       # 指针/强调色
_ACCENT_DIM = st7789.color565(0, 122, 112)
_TEXT = st7789.color565(238, 245, 255)       # 主数值
_MUTED = st7789.color565(103, 124, 148)      # 标签
_HUB = st7789.color565(222, 244, 246)        # 指针轴心

# 指针几何（像素）
_NEEDLE_BASE_R = 4.0        # 指针根部离圆心
_NEEDLE_HALF_BASE = 3.8     # 指针根部半宽

_MODE_ANGLE = 0
_MODE_ROTATION = 1

# 5x7 点阵，供设置界面标题、提示和刻度方位使用
_SMALL_FONT = {
    "0": (0x0E, 0x11, 0x13, 0x15, 0x19, 0x11, 0x0E),
    "1": (0x04, 0x0C, 0x04, 0x04, 0x04, 0x04, 0x0E),
    "2": (0x0E, 0x11, 0x01, 0x02, 0x04, 0x08, 0x1F),
    "3": (0x1F, 0x02, 0x04, 0x02, 0x01, 0x11, 0x0E),
    "4": (0x02, 0x06, 0x0A, 0x12, 0x1F, 0x02, 0x02),
    "5": (0x1F, 0x10, 0x1E, 0x01, 0x01, 0x11, 0x0E),
    "6": (0x06, 0x08, 0x10, 0x1E, 0x11, 0x11, 0x0E),
    "7": (0x1F, 0x01, 0x02, 0x04, 0x08, 0x08, 0x08),
    "8": (0x0E, 0x11, 0x11, 0x0E, 0x11, 0x11, 0x0E),
    "9": (0x0E, 0x11, 0x11, 0x0F, 0x01, 0x01, 0x0E),
    "-": (0x00, 0x00, 0x00, 0x1F, 0x00, 0x00, 0x00),
    "+": (0x00, 0x04, 0x04, 0x1F, 0x04, 0x04, 0x00),
    "A": (0x0E, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11),
    "B": (0x1E, 0x11, 0x11, 0x1E, 0x11, 0x11, 0x1E),
    "C": (0x0E, 0x11, 0x10, 0x10, 0x10, 0x11, 0x0E),
    "D": (0x1E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x1E),
    "E": (0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x1F),
    "F": (0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x10),
    "G": (0x0E, 0x11, 0x10, 0x17, 0x11, 0x11, 0x0E),
    "H": (0x11, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11),
    "I": (0x0E, 0x04, 0x04, 0x04, 0x04, 0x04, 0x0E),
    "J": (0x07, 0x02, 0x02, 0x02, 0x02, 0x12, 0x0C),
    "K": (0x11, 0x12, 0x14, 0x18, 0x14, 0x12, 0x11),
    "L": (0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x1F),
    "M": (0x11, 0x1B, 0x15, 0x15, 0x11, 0x11, 0x11),
    "N": (0x11, 0x19, 0x15, 0x13, 0x11, 0x11, 0x11),
    "O": (0x0E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E),
    "P": (0x1E, 0x11, 0x11, 0x1E, 0x10, 0x10, 0x10),
    "Q": (0x0E, 0x11, 0x11, 0x11, 0x15, 0x12, 0x0D),
    "R": (0x1E, 0x11, 0x11, 0x1E, 0x14, 0x12, 0x11),
    "S": (0x0F, 0x10, 0x10, 0x0E, 0x01, 0x01, 0x1E),
    "T": (0x1F, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04),
    "U": (0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E),
    "V": (0x11, 0x11, 0x11, 0x11, 0x11, 0x0A, 0x04),
    "W": (0x11, 0x11, 0x11, 0x15, 0x15, 0x1B, 0x11),
    "X": (0x11, 0x11, 0x0A, 0x04, 0x0A, 0x11, 0x11),
    "Y": (0x11, 0x11, 0x0A, 0x04, 0x04, 0x04, 0x04),
    "Z": (0x1F, 0x01, 0x02, 0x04, 0x08, 0x10, 0x1F),
    ":": (0x00, 0x04, 0x04, 0x00, 0x04, 0x04, 0x00),
    "a": (0x00, 0x00, 0x0E, 0x01, 0x0F, 0x11, 0x0F),
    "c": (0x00, 0x00, 0x0E, 0x11, 0x10, 0x11, 0x0E),
    "d": (0x01, 0x01, 0x0F, 0x11, 0x11, 0x13, 0x0D),
    "e": (0x00, 0x00, 0x0E, 0x11, 0x1F, 0x10, 0x0E),
    "g": (0x0F, 0x11, 0x11, 0x0F, 0x01, 0x01, 0x0E),
    "h": (0x10, 0x10, 0x16, 0x19, 0x11, 0x11, 0x11),
    "i": (0x04, 0x00, 0x0C, 0x04, 0x04, 0x04, 0x0E),
    "l": (0x0C, 0x04, 0x04, 0x04, 0x04, 0x04, 0x0E),
    "m": (0x00, 0x00, 0x1A, 0x15, 0x15, 0x15, 0x15),
    "n": (0x00, 0x00, 0x16, 0x19, 0x11, 0x11, 0x11),
    "o": (0x00, 0x00, 0x0E, 0x11, 0x11, 0x11, 0x0E),
    "r": (0x00, 0x00, 0x16, 0x19, 0x10, 0x10, 0x10),
    "s": (0x00, 0x00, 0x0F, 0x10, 0x0E, 0x01, 0x1E),
    "t": (0x08, 0x08, 0x1C, 0x08, 0x08, 0x09, 0x06),
    "u": (0x00, 0x00, 0x11, 0x11, 0x11, 0x13, 0x0D),
    "x": (0x00, 0x00, 0x11, 0x0A, 0x04, 0x0A, 0x11),
}


def _text_width(text, scale=1):
    """5x7 点阵字符串的像素宽度。"""
    return len(text) * 5 * scale + (len(text) - 1) * scale


def _draw_small(display, text, x, y, color, scale=1):
    """绘制 5x7 小字，同色横向连续像素合并成一次 fill_rect。"""
    for char in text:
        rows = _SMALL_FONT.get(char)
        if rows:
            for row, bits in enumerate(rows):
                col = 0
                while col < 5:
                    if bits & (1 << (4 - col)):
                        run = 1
                        while col + run < 5 and (bits & (1 << (4 - col - run))):
                            run += 1
                        display.fill_rect(
                            x + col * scale, y + row * scale,
                            run * scale, scale, color
                        )
                        col += run
                    else:
                        col += 1
        x += 6 * scale


def _circle_ring(display, cx, cy, radius, thickness, color):
    """用横向色块快速画一个空心圆环。"""
    inner = radius - thickness
    inner_sq = inner * inner
    for dy in range(-radius, radius + 1):
        outer_w = int(math.sqrt(radius * radius - dy * dy))
        if abs(dy) < inner:
            inner_w = int(math.sqrt(inner_sq - dy * dy))
        else:
            inner_w = -1
        y = cy + dy
        if inner_w < 0:
            display.fill_rect(cx - outer_w, y, outer_w * 2 + 1, 1, color)
        else:
            w = outer_w - inner_w
            if w > 0:
                display.fill_rect(cx - outer_w, y, w, 1, color)
                display.fill_rect(cx + inner_w + 1, y, w, 1, color)


class AngleUI:
    """240x320 的倒立摆界面。"""

    def __init__(self, display):
        if display is None:
            raise ValueError("display is required")

        self.display = display
        self.width = display.width
        self.height = display.height

        self._mode = _MODE_ANGLE
        self._angle = 0.0
        self._last_angle = None
        self._last_text = None
        self._needle_rect = None
        self._gauge_dirty = False
        # 表盘底部常驻的一行小字（显示 IP 用）
        self._footer = None

        self._rotation_magnitude = 0
        self._rotation_positive = True
        self._rotation_cursor = 0
        self._rotation_status = None

        self._layout()
        # 先分配最大的指针缓冲区，再分配临时缓冲区，减少堆碎片导致的
        # MemoryError。芯片刚上电时堆最完整，顺序反过来更容易失败。
        self._init_needle_buffer()
        # 所有原子绘制（数字、设置项）共用一块 RGB565 临时缓冲区。
        self._scratch = bytearray(self._scratch_pixels * 2)
        self._prepare_static()
        self._draw_gauge_static()
        self.set_angle(0.0)

    # ------------------------------------------------------------------
    # 布局
    # ------------------------------------------------------------------

    def _layout(self):
        w = self.width
        h = self.height

        if h >= w + 40:
            # 240x320 主布局。
            radius = min((w - 24) // 2, 102)
            self.cx = w // 2
            self.cy = 18 + radius
            self.circle_bottom = self.cy + radius
            self.big_y = h - 56
        else:
            # 兼容其他分辨率。
            radius = min((w - 16) // 2, (h - 68) // 2, 100)
            self.cx = w // 2
            self.cy = 6 + radius
            self.circle_bottom = self.cy + radius
            self.big_y = min(h - 40, self.circle_bottom + 22)

        self.radius = max(radius, 24)
        self.tick_inner = self.radius - 10
        self.minor_len = max(3, self.radius // 22)
        self.major_len = max(6, self.radius // 11)
        self.needle_len = max(10, int(self.tick_inner * 0.68))
        self.hub_radius = max(4, self.radius // 14)

        if self.big_y < self.circle_bottom + 30:
            self.big_y = self.circle_bottom + 30
        self.label_y = self.big_y - 18

        self.readout_w = big_font.WIDTH * 6 + 13
        if self.readout_w > w - 8:
            self.readout_w = w - 8

        # 表盘数字：整块重绘后一次 blit，避免逐行刷新。
        self.readout_clear_w = self.readout_w + 6
        self.readout_clear_h = big_font.HEIGHT + 4
        self.readout_clear_x = (w - self.readout_clear_w) // 2
        self.readout_y = self.big_y - 2

        # 旋转设置界面的四个字符格。
        self.cell_w = 48
        self.cell_gap = 4
        self.cell_y = 96
        self.cell_h = 86
        self.cell_label_gap = 9
        self.cell_block_h = self.cell_h + self.cell_label_gap + 7
        total_w = self.cell_w * 4 + self.cell_gap * 3
        self.cell_x0 = (w - total_w) // 2
        self.rotation_labels = ("SIGN", "100", "10", "1")

        # 临时缓冲区按最大绘制块分配（四个字符格最大）。
        self._scratch_pixels = self.cell_w * self.cell_block_h
        readout_pixels = self.readout_clear_w * self.readout_clear_h
        if readout_pixels > self._scratch_pixels:
            self._scratch_pixels = readout_pixels
        status_pixels = 160 * 18
        if status_pixels > self._scratch_pixels:
            self._scratch_pixels = status_pixels

    # ------------------------------------------------------------------
    # RAM 缓冲绘制：拼好一整块后只 blit 一次
    # ------------------------------------------------------------------

    def _scratch_fill(self, width, height, color):
        buf = self._scratch
        hi = color >> 8
        lo = color & 0xFF
        for idx in range(0, width * height * 2, 2):
            buf[idx] = hi
            buf[idx + 1] = lo

    def _scratch_rect(self, stride, x, y, width, height, color):
        if width <= 0 or height <= 0:
            return
        buf = self._scratch
        hi = color >> 8
        lo = color & 0xFF
        for yy in range(y, y + height):
            idx = (yy * stride + x) * 2
            for _ in range(width):
                buf[idx] = hi
                buf[idx + 1] = lo
                idx += 2

    def _scratch_big_char(self, stride, char, x, y, color, scale=1):
        code = ord(char)
        if not (big_font.FIRST <= code < big_font.LAST):
            return

        data = big_font.FONT
        base = (code - big_font.FIRST) * 64
        buf = self._scratch
        hi = color >> 8
        lo = color & 0xFF
        for row in range(32):
            band = row // 8
            in_band = row % 8
            offset = base + band * 16 + in_band * 2
            bits = (data[offset] << 8) | data[offset + 1]
            for bit in range(16):
                if bits & (1 << (15 - bit)):
                    px = x + bit * scale
                    for sy in range(scale):
                        idx = ((y + row * scale + sy) * stride + px) * 2
                        for _ in range(scale):
                            buf[idx] = hi
                            buf[idx + 1] = lo
                            idx += 2

    def _scratch_small_text(self, stride, text, x, y, color, scale=1):
        buf = self._scratch
        hi = color >> 8
        lo = color & 0xFF
        for char in text:
            rows = _SMALL_FONT.get(char)
            if rows:
                for row, bits in enumerate(rows):
                    for col in range(5):
                        if bits & (1 << (4 - col)):
                            px = x + col * scale
                            for sy in range(scale):
                                idx = ((y + row * scale + sy) * stride + px) * 2
                                for _ in range(scale):
                                    buf[idx] = hi
                                    buf[idx + 1] = lo
                                    idx += 2
            x += 6 * scale

    def _scratch_ring(self, stride, cx, cy, radius, color):
        buf = self._scratch
        hi = color >> 8
        lo = color & 0xFF
        r_sq = radius * radius
        inner_sq = (radius - 1) * (radius - 1)
        for dy in range(-radius, radius + 1):
            width = int(math.sqrt(r_sq - dy * dy))
            for dx in range(-width, width + 1):
                if dx * dx + dy * dy >= inner_sq:
                    idx = ((cy + dy) * stride + cx + dx) * 2
                    buf[idx] = hi
                    buf[idx + 1] = lo

    def _scratch_blit(self, x, y, width, height):
        self.display.blit_buffer(
            memoryview(self._scratch)[:width * height * 2],
            x, y, width, height
        )

    # ------------------------------------------------------------------
    # 表盘静态内容
    # ------------------------------------------------------------------

    def _prepare_static(self):
        """预计算静态表盘的刻度和标签位置。

        表盘只在初始化、或者从设置界面切回来时重画一次；位置先算好，
        重画时不再做三角函数。
        """
        ticks = []
        for deg in range(0, 360, 10):
            major = (deg % 30) == 0
            r1 = self.tick_inner
            r2 = r1 + (self.major_len if major else self.minor_len)
            rad = math.radians(deg)
            sin_a = math.sin(rad)
            cos_a = math.cos(rad)
            ticks.append((
                int(self.cx + sin_a * r1 + 0.5),
                int(self.cy - cos_a * r1 + 0.5),
                int(self.cx + sin_a * r2 + 0.5),
                int(self.cy - cos_a * r2 + 0.5),
                _TICK_MAJOR if major else _TICK,
            ))
        self._ticks = ticks

        labels = []
        if self.radius >= 80:
            label_r = self.tick_inner - 12
            for text, deg in (
                ("0", 0), ("90", 90), ("-180", 180), ("-90", 270)
            ):
                rad = math.radians(deg)
                tw = _text_width(text)
                labels.append((
                    text,
                    int(self.cx + math.sin(rad) * label_r - tw / 2),
                    int(self.cy - math.cos(rad) * label_r - 3),
                ))
        self._labels = labels
        self._angle_label_w = _text_width("ANGLE", 2)

    def _draw_gauge_static(self):
        d = self.display
        d.fill(_BG)
        _circle_ring(d, self.cx, self.cy, self.radius, 2, _RING)

        for x1, y1, x2, y2, color in self._ticks:
            d.line(x1, y1, x2, y2, color)

        for text, lx, ly in self._labels:
            _draw_small(d, text, lx, ly, _MUTED)

        _draw_small(d, "ANGLE", (self.width - self._angle_label_w) // 2,
                    self.label_y, _MUTED, 2)

        self._draw_footer()

    def _draw_footer(self):
        """在屏幕最底部画那行常驻小字。"""
        if self._footer:
            _draw_small(
                self.display, self._footer, 4, self.height - 14, _MUTED
            )

    def set_footer(self, text):
        """设置底部常驻小字（例如 WiFi IP）。

        切到旋转角设置界面时整屏会被重绘，所以只在表盘模式下直接画；
        回到表盘时 _draw_gauge_static() 会自动补上。
        """
        self._footer = text
        if self._mode == _MODE_ANGLE:
            self._draw_footer()

    # ------------------------------------------------------------------
    # 指针精灵：只重画“指针 + 轴心”的包围盒
    #
    # 指针绕圆心转，任何角度下“指针 + 轴心”的包围盒最大也就 70x60 像素
    # 左右（约 7 KB）。旧版固定一块 127x127（32 KB），每帧多传 4~5 倍
    # 的数据，SoftSPI 下就是几百毫秒的差距。
    # ------------------------------------------------------------------

    def _needle_rect_for(self, degrees):
        """指针（含轴心）相对圆心的包围盒 (x, y, w, h)。"""
        radians = math.radians(degrees)
        ux = math.sin(radians)
        uy = -math.cos(radians)
        px = -uy
        py = ux

        bx = ux * _NEEDLE_BASE_R
        by = uy * _NEEDLE_BASE_R
        xs = (bx + px * _NEEDLE_HALF_BASE,
              bx - px * _NEEDLE_HALF_BASE,
              ux * self.needle_len)
        ys = (by + py * _NEEDLE_HALF_BASE,
              by - py * _NEEDLE_HALF_BASE,
              uy * self.needle_len)

        hub = self.hub_radius + 1
        x0 = int(math.floor(min(min(xs), -hub))) - 1
        y0 = int(math.floor(min(min(ys), -hub))) - 1
        x1 = int(math.ceil(max(max(xs), hub))) + 1
        y1 = int(math.ceil(max(max(ys), hub))) + 1
        return (x0, y0, x1 - x0 + 1, y1 - y0 + 1)

    def _init_needle_buffer(self):
        # 扫一圈（1 度步长）取最大包围盒，按它分配精灵缓冲。
        max_pixels = 0
        for deg in range(0, 360):
            rect = self._needle_rect_for(deg)
            pixels = rect[2] * rect[3]
            if pixels > max_pixels:
                max_pixels = pixels
        self._sprite_cap = int(max_pixels * 1.1) + 32
        self._sprite = bytearray(self._sprite_cap * 2)

        self.bg_hi = _BG >> 8
        self.bg_lo = _BG & 0xFF

        # 背景色块：清精灵时用切片赋值整段拷贝，交给 C 代码做，
        # 不再逐像素跑 Python 循环。
        chunk = bytearray(2048)
        for idx in range(0, len(chunk), 2):
            chunk[idx] = self.bg_hi
            chunk[idx + 1] = self.bg_lo
        self._bg_chunk = chunk

    def _sprite_clear(self, pixels):
        """把精灵缓冲的前 pixels 个像素填成背景色。"""
        n = pixels * 2
        buf = self._sprite
        chunk = self._bg_chunk
        step = len(chunk)
        off = 0
        while off + step <= n:
            buf[off:off + step] = chunk
            off += step
        if off < n:
            buf[off:n] = chunk[:n - off]

    def _render_needle(self, degrees, rect, stride):
        """把指针和轴心画进精灵缓冲（局部坐标，行跨度 stride）。"""
        x0, y0, w, h = rect
        radians = math.radians(degrees)
        ux = math.sin(radians)
        uy = -math.cos(radians)
        px = -uy
        py = ux

        base_x = ux * _NEEDLE_BASE_R
        base_y = uy * _NEEDLE_BASE_R
        points = (
            (base_x + px * _NEEDLE_HALF_BASE,
             base_y + py * _NEEDLE_HALF_BASE),
            (base_x - px * _NEEDLE_HALF_BASE,
             base_y - py * _NEEDLE_HALF_BASE),
            (ux * self.needle_len, uy * self.needle_len),
        )

        buf = self._sprite
        hi = _ACCENT >> 8
        lo = _ACCENT & 0xFF

        # 实心三角：按扫描线求左右边界，整行连续写入。
        min_y = int(min(p[1] for p in points))
        max_y = int(max(p[1] for p in points) + 0.999)
        for y in range(min_y, max_y + 1):
            sample_y = y + 0.5
            xs = []
            for edge in range(3):
                ax, ay = points[edge]
                nx, ny = points[(edge + 1) % 3]
                if ay == ny:
                    continue
                if (ay <= sample_y < ny) or (ny <= sample_y < ay):
                    xs.append(ax + (sample_y - ay) * (nx - ax) / (ny - ay))
            row = y - y0
            if len(xs) < 2 or row < 0 or row >= h:
                continue
            left = int(min(xs) + 0.5) - x0
            right = int(max(xs) + 0.5) - x0
            if left < 0:
                left = 0
            if right >= w:
                right = w - 1
            if left > right:
                continue
            idx = (row * stride + left) * 2
            for _ in range(right - left + 1):
                buf[idx] = hi
                buf[idx + 1] = lo
                idx += 2

        # 中间的高亮细线
        x0l = int(-x0 + ux * (_NEEDLE_BASE_R + 1) + 0.5)
        y0l = int(-y0 + uy * (_NEEDLE_BASE_R + 1) + 0.5)
        x1l = int(-x0 + ux * (self.needle_len - 1.5) + 0.5)
        y1l = int(-y0 + uy * (self.needle_len - 1.5) + 0.5)
        dx = abs(x1l - x0l)
        sx = 1 if x0l < x1l else -1
        dy = -abs(y1l - y0l)
        sy = 1 if y0l < y1l else -1
        err = dx + dy
        hi = _TEXT >> 8
        lo = _TEXT & 0xFF
        while True:
            if 0 <= x0l < w and 0 <= y0l < h:
                idx = (y0l * stride + x0l) * 2
                buf[idx] = hi
                buf[idx + 1] = lo
            if x0l == x1l and y0l == y1l:
                break
            e2 = 2 * err
            if e2 >= dy:
                err += dy
                x0l += sx
            if e2 <= dx:
                err += dx
                y0l += sy

        # 轴心（叠在指针根部上）
        r = self.hub_radius
        r_sq = r * r
        inner_sq = (r - 2) * (r - 2)
        ccx = -x0
        ccy = -y0
        for hdy in range(-r, r + 1):
            half_w = int(math.sqrt(r_sq - hdy * hdy))
            sy2 = ccy + hdy
            if sy2 < 0 or sy2 >= h:
                continue
            row_idx = sy2 * stride
            for hdx in range(-half_w, half_w + 1):
                sx2 = ccx + hdx
                if sx2 < 0 or sx2 >= w:
                    continue
                if hdx * hdx + hdy * hdy <= inner_sq:
                    color = _HUB
                else:
                    color = _ACCENT_DIM
                idx = (row_idx + sx2) * 2
                buf[idx] = color >> 8
                buf[idx + 1] = color & 0xFF

    # ------------------------------------------------------------------
    # 表盘数值和界面切换
    # ------------------------------------------------------------------

    def _draw_readout(self, value):
        if abs(value) < 0.05:
            text = "0.0"
        else:
            text = "%+.1f" % value

        if text == self._last_text:
            return

        clear_w = self.readout_clear_w
        clear_h = self.readout_clear_h
        font_w = len(text) * big_font.WIDTH
        total_w = font_w + 7 + 7
        tx = (self.width - total_w) // 2 - self.readout_clear_x

        if self._last_text is not None and len(text) == len(self._last_text):
            # 位数没变（绝大多数情况）：只重画真正变化的那个字符。
            old = self._last_text
            for i in range(len(text)):
                if text[i] == old[i]:
                    continue
                gx = tx + i * big_font.WIDTH
                self._scratch_fill(big_font.WIDTH, clear_h, _BG)
                self._scratch_big_char(
                    big_font.WIDTH, text[i], 0, 2, _TEXT, 1
                )
                self._scratch_blit(
                    self.readout_clear_x + gx, self.readout_y,
                    big_font.WIDTH, clear_h
                )
            self._last_text = text
            return

        # 位数变了（例如 +9.9 -> +10.0），整块重画，顺便重画度数符号。
        self._scratch_fill(clear_w, clear_h, _BG)
        ty = 2
        for i in range(len(text)):
            self._scratch_big_char(
                clear_w, text[i], tx + i * big_font.WIDTH, ty, _TEXT, 1
            )
        self._scratch_ring(clear_w, tx + font_w + 8, ty + 6, 3, _ACCENT)

        # 整块数字（含背景和度数符号）一次写入，不会逐行刷新。
        self._scratch_blit(
            self.readout_clear_x, self.readout_y, clear_w, clear_h
        )
        self._last_text = text

    def show_angle_screen(self):
        """切换回角度表盘，并立即刷新为最新角度。"""
        self._mode = _MODE_ANGLE
        self._last_angle = None
        self._last_text = None
        # 只有被设置界面盖住过才需要重画静态表盘；平时刷新指针不动它。
        if self._gauge_dirty:
            self._gauge_dirty = False
            self._draw_gauge_static()
        self._needle_rect = None
        self.set_angle(self._angle)

    def show_rotation_screen(self, magnitude=0, positive=True,
                             cursor=0, status=None):
        """打开旋转角设置界面。

        第一次进入时绘制整页；之后修改数字或按 ALT 切换时，只重绘
        发生变化字符附近的小块区域。

        参数:
            magnitude: 0..180 的角度幅值
            positive:  True 显示 +，False 显示 -
            cursor:    当前选中的字符，0=符号，1=百位，2=十位，3=个位
            status:    可选状态文字，例如发送成功后传入 "SENT"
        """
        try:
            magnitude = int(magnitude)
        except (TypeError, ValueError):
            magnitude = 0
        if magnitude < 0:
            magnitude = -magnitude
        if magnitude > 180:
            magnitude = 180

        positive = bool(positive)
        cursor = int(cursor) % 4
        status = status if status else None

        if self._mode != _MODE_ROTATION:
            self._mode = _MODE_ROTATION
            self._gauge_dirty = True
            self._rotation_magnitude = magnitude
            self._rotation_positive = positive
            self._rotation_cursor = cursor
            self._rotation_status = status
            self._draw_rotation_screen(status)
            return

        old_digits = self._rotation_digits(
            self._rotation_magnitude, self._rotation_positive
        )
        new_digits = self._rotation_digits(magnitude, positive)
        old_cursor = self._rotation_cursor
        new_cursor = cursor

        full_cells = []
        glyph_cells = []
        for i in range(4):
            selected_changed = (i == old_cursor) != (i == new_cursor)
            if selected_changed:
                full_cells.append(i)
            elif old_digits[i] != new_digits[i]:
                glyph_cells.append(i)

        old_status = self._rotation_status
        self._rotation_magnitude = magnitude
        self._rotation_positive = positive
        self._rotation_cursor = cursor
        self._rotation_status = status

        # ALT 改变选中框，重绘受影响的字符格。
        for i in full_cells:
            self._draw_rotation_cell(i, new_digits[i])

        # 数字本身变化时，只更新 32x64 的字形区域。
        for i in glyph_cells:
            self._draw_rotation_glyph(i, new_digits[i])

        # 发送状态只刷新状态文字区域。
        if status != old_status:
            self._draw_rotation_status(status)

    def _rotation_digits(self, magnitude, positive):
        return (
            "+" if positive else "-",
            str(magnitude // 100),
            str((magnitude // 10) % 10),
            str(magnitude % 10),
        )

    def _draw_rotation_cell(self, index, digit):
        """绘制一个完整的字符格（含选中背景、边框和底部标签）。"""
        stride = self.cell_w
        height = self.cell_block_h
        selected = (index == self._rotation_cursor)

        self._scratch_fill(stride, height, _BG)
        panel = _PANEL_ACTIVE if selected else _PANEL
        border = _ACCENT if selected else _RULE
        self._scratch_rect(stride, 0, 0, stride, self.cell_h, panel)
        self._scratch_rect(stride, 0, 0, stride, 1, border)
        self._scratch_rect(stride, 0, self.cell_h - 1, stride, 1, border)
        self._scratch_rect(stride, 0, 0, 1, self.cell_h, border)
        self._scratch_rect(stride, stride - 1, 0, 1, self.cell_h, border)
        if selected:
            self._scratch_rect(stride, 0, self.cell_h - 3,
                               stride, 3, _ACCENT)

        glyph_x = (stride - big_font.WIDTH * 2) // 2
        self._scratch_big_char(
            stride, digit, glyph_x, 11,
            _ACCENT if selected else _TEXT, 2
        )

        label = self.rotation_labels[index]
        label_w = _text_width(label)
        self._scratch_small_text(
            stride, label, (stride - label_w) // 2,
            self.cell_h + self.cell_label_gap,
            _ACCENT if selected else _MUTED
        )

        cell_x = self.cell_x0 + index * (self.cell_w + self.cell_gap)
        self._scratch_blit(cell_x, self.cell_y, stride, height)

    def _draw_rotation_glyph(self, index, digit):
        """只重绘字符笔画及其底色，用于 UP/DOWN 修改数字。"""
        width = big_font.WIDTH * 2
        height = big_font.HEIGHT * 2
        selected = (index == self._rotation_cursor)
        self._scratch_fill(
            width, height, _PANEL_ACTIVE if selected else _PANEL
        )
        self._scratch_big_char(
            width, digit, 0, 0, _ACCENT if selected else _TEXT, 2
        )

        cell_x = self.cell_x0 + index * (self.cell_w + self.cell_gap)
        glyph_x = cell_x + (self.cell_w - width) // 2
        self._scratch_blit(glyph_x, self.cell_y + 11, width, height)

    def _draw_rotation_status(self, status):
        """只刷新发送状态区域。"""
        width = 160
        height = 18
        x = (self.width - width) // 2
        y = 214
        self._scratch_fill(width, height, _BG)
        if status:
            status_w = _text_width(status, 2)
            self._scratch_small_text(
                width, status, (width - status_w) // 2, 4, _ACCENT, 2
            )
        self._scratch_blit(x, y, width, height)

    def _draw_rotation_screen(self, status=None):
        d = self.display
        w = self.width
        d.fill(_BG)

        # 标题
        title = "ROTATION"
        title_w = _text_width(title, 2)
        _draw_small(d, title, (w - title_w) // 2, 38, _TEXT, 2)
        subtitle = "SET ANGLE"
        sub_w = _text_width(subtitle)
        _draw_small(d, subtitle, (w - sub_w) // 2, 68, _MUTED)
        d.fill_rect((w - 26) // 2, 61, 26, 2, _ACCENT_DIM)

        digits = self._rotation_digits(
            self._rotation_magnitude, self._rotation_positive
        )
        for i in range(4):
            self._draw_rotation_cell(i, digits[i])

        self._draw_rotation_status(status)

        hints = (
            "alt: next digit",
            "short menu: cancel",
            "hold menu: execute",
        )
        y = 250
        for hint in hints:
            hint_w = _text_width(hint, 2)
            _draw_small(d, hint, (w - hint_w) // 2, y, _MUTED, 2)
            y += 22

    # ------------------------------------------------------------------
    # 对外更新接口
    # ------------------------------------------------------------------

    def set_angle(self, angle):
        """更新指针位置，角度单位为度。"""
        try:
            value = float(angle)
        except (TypeError, ValueError):
            return

        if value != value:  # NaN
            return
#         if value > 180.0:
#             value = 180.0
#         elif value < -180.0:
#             value = -180.0

        self._angle = value
        if self._mode != _MODE_ANGLE:
            return
        if self._last_angle is not None and value == self._last_angle:
            return

        rect = self._needle_rect_for(value)
        stride = rect[2]
        pixels = stride * rect[3]
        if pixels > self._sprite_cap:
            # 理论上不会发生（初始化时已按最大包围盒分配），
            # 真遇到就扩一次，绝不静默丢帧。
            self._sprite_cap = pixels
            self._sprite = bytearray(pixels * 2)

        # 先在 RAM 里画好新指针，再擦旧位置、刷新的，屏幕上的空档最短。
        self._sprite_clear(pixels)
        self._render_needle(value, rect, stride)

        if self._needle_rect is not None:
            ox, oy, ow, oh = self._needle_rect
            self.display.fill_rect(
                self.cx + ox, self.cy + oy, ow, oh, _BG
            )

        self.display.blit_buffer(
            memoryview(self._sprite)[:pixels * 2],
            self.cx + rect[0], self.cy + rect[1], stride, rect[3]
        )
        self._needle_rect = rect
        self._draw_readout(value)
        self._last_angle = value


# ---------------------------------------------------------------------------
# 模块级便捷接口
# ---------------------------------------------------------------------------

_ui = None


def init(display):
    """初始化界面并返回 AngleUI 对象。"""
    global _ui
    # 重跑 main 时先丢掉上一轮界面对象的引用，再回收，避免两份大缓冲区
    # 同时占用堆内存。
    _ui = None
    gc.collect()
    _ui = AngleUI(display)
    return _ui


def set_angle(angle):
    """把指针移动到指定角度；调用前需要先执行 init(tft)。"""
    if _ui is None:
        raise RuntimeError("ui.init(tft) must be called before ui.set_angle()")
    _ui.set_angle(angle)


def show_angle_screen():
    """返回角度表盘界面。"""
    if _ui is None:
        raise RuntimeError("ui.init(tft) must be called first")
    _ui.show_angle_screen()


def show_rotation_screen(magnitude=0, positive=True, cursor=0, status=None):
    """打开旋转角设置界面。"""
    if _ui is None:
        raise RuntimeError("ui.init(tft) must be called first")
    _ui.show_rotation_screen(magnitude, positive, cursor, status)


def set_footer(text):
    """在表盘底部显示一行小字（例如 WiFi IP）。"""
    if _ui is None:
        raise RuntimeError("ui.init(tft) must be called first")
    _ui.set_footer(text)


# ---------------------------------------------------------------------------
# 直接运行 ui.py 时的显示调试
#
# 下面引脚与 main.py 当前接线保持一致：
#   SCK=14 MOSI=13 MISO=12 CS=15 DC=2 RST=4 BL=32
# 14/13/12 正好是 ESP32 HSPI 的默认引脚，所以用硬件 SPI 全速驱动。
# 程序会循环演示表盘扫角和旋转角设置界面。
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    import time

    from machine import Pin, SPI, SoftSPI

    # 与 main.py 保持一致：屏一点不亮时把 USE_HARD_SPI 改成 False，
    # 用软件 SPI 确认屏和接线没问题，再回来查硬件 SPI。
    USE_HARD_SPI = True
    SPI_BAUD = 10000000

    if USE_HARD_SPI:
        print("ui debug: hard spi %d" % SPI_BAUD)
        # ST7789 在 SCL 上升沿锁存数据，硬件 SPI 必须用 mode 0；
        # 用 polarity=1/phase=0 的话硬件会在上升沿同时改数据，屏会不亮。
        spi = SPI(
            1,
            baudrate=SPI_BAUD,
            polarity=0,
            phase=0,
            sck=Pin(14),
            mosi=Pin(13),
            miso=Pin(12),
        )
    else:
        print("ui debug: soft spi")
        spi = SoftSPI(
            baudrate=2000000,
            polarity=1,
            phase=0,
            sck=Pin(14),
            mosi=Pin(13),
            miso=Pin(12),
        )
    tft = st7789.ST7789(
        spi,
        320,
        240,
        reset=Pin(4, Pin.OUT),
        dc=Pin(2, Pin.OUT),
        cs=Pin(15, Pin.OUT),
        backlight=Pin(32, Pin.OUT),
        rotation=0,
    )
    print("ui debug: display ok")

    view = AngleUI(tft)
    print("ui debug: running")

    while True:
        # 指针从 -180° 扫到 +180°。
        view.show_angle_screen()
        for angle in range(-180, 181, 5):
            view.set_angle(angle)
            time.sleep_ms(30)
        time.sleep_ms(500)

        # 依次高亮符号、百位、十位、个位。
        for cursor in range(4):
            view.show_rotation_screen(90, True, cursor)
            time.sleep_ms(700)

        # 演示正负号和发送成功状态。
        view.show_rotation_screen(180, False, 3, "SENT")
        time.sleep_ms(1500)

