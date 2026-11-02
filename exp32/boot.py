# This file is executed on every boot (including wake-boot from deepsleep)

# 关掉 IDF 的串口日志。它自己也要占 UART0 的发送缓冲，而我们正在为
# 几百字节的内部 RAM 打架。
import esp
esp.osdebug(None)

# ---------------------------------------------------------------------------
# 这个文件的核心目标只有一个：把吃“内部 RAM 连续内存”的东西，全部抢在
# `import tmp` 之前初始化完。
#
# MicroPython 的堆是按需从内部 DRAM 增长的。import tmp 会连带拉进 st7789、
# ui、字库、_SMALL_FONT，几十 KB 的 Python 对象一落地，内部 RAM 就碎了。
# 之后再去要连续内存，就按顺序碰到这些死法：
#
#   OSError: WiFi Out of Memory               WiFi 排晚了
#   E uart: UART driver malloc error → EPERM  UART 排晚了
#   OSError: out of memory                    SPI 总线排晚了
#   [Errno 105] ENOBUFS                       监听 socket 排晚了
#
# 实测过的顺序：UART 一定要在屏之前；SPI 和 socket 一定要在 import tmp
# 之前。WiFi 要一大块，排第二（前面只让出 1 KB 给 UART）。
# ---------------------------------------------------------------------------

import uart
uart.open_hw()

import wifi
wifi.boot_connect()

import board
board.open_spi()

import web
try:
    web.init()
    print("[boot] http ok")
except OSError as err:
    # 112 = 端口被上一轮占着；105 / 12 = 内部 RAM 不够
    print("[boot] http fail:", err)

import tmp
tmp.main()
