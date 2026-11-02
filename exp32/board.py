"""
板级外设：SPI 总线。

单独抽成模块，只依赖 machine，不导入任何界面/字库代码 —— 这样
boot.py 可以在 `import tmp` 之前就把 SPI 总线建好，趁内部 RAM 还完整
的时候把 DMA 通道和描述符抢下来。

背景：MicroPython 的堆是按需从内部 DRAM 增长的，而 SPI 的 DMA 描述符、
UART 的环形缓冲、WiFi 的收发缓冲、lwIP 的 socket 都要从同一块内部 RAM
里分连续内存。等到 ui、字库、字节码把堆撑大几十 KB 之后再去要，就会
碰到 `OSError: out of memory` / `ENOBUFS` / 甚至 IDF 直接 abort。

用法（boot.py 里，必须在 import tmp 之前）::

    import board
    board.open_spi()

之后 tmp.py 里的 make_display() 用 board.spi() 拿同一个总线对象。
"""

from machine import Pin, SPI


# ---------------------------------------------------------------------------
# 接线（和 tmp.py 原来那套保持一致）
# ---------------------------------------------------------------------------

PIN_SCK = 14
PIN_MOSI = 13
PIN_MISO = 12

# HSPI = SPI(1)，默认引脚就是 14/13/12/15，可以跑 IOMUX 全速。
SPI_ID = 1
SPI_BAUD = 20000000
# ST7789 的参考时序是 mode 0。
SPI_POLARITY = 0
SPI_PHASE = 0

_bus = None


def open_spi():
    """创建硬件 SPI 总线；重复调用返回同一个对象。"""
    global _bus
    if _bus is None:
        _bus = SPI(
            SPI_ID,
            baudrate=SPI_BAUD,
            polarity=SPI_POLARITY,
            phase=SPI_PHASE,
            sck=Pin(PIN_SCK),
            mosi=Pin(PIN_MOSI),
            miso=Pin(PIN_MISO),
        )
    return _bus


def spi():
    """拿到已经建好的总线；没建过就现在建一个。"""
    return _bus if _bus is not None else open_spi()
