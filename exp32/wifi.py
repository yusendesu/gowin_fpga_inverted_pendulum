"""
网络。两种组网方式任选（见下面的 AP_MODE）：

    1. 板子当客户端连手机热点（STA）：手机开热点，板子连上去，
       手机浏览器访问板子的 IP。
    2. 板子自己开热点（AP）：手机 WiFi 里连板子，浏览器打开
       http://192.168.4.1/ 。演示时更稳，不受手机热点各种行为影响。

用法::

    import wifi
    if wifi.boot_connect():        # 开机时在 boot.py 里调用
        print(wifi.ip())

!! 顺序很重要 !!
    boot_connect() 必须在 import ui / st7789 / uart 之前调用（也就是放在
    boot.py 里、import tmp 之前）。MicroPython 的堆是按需从内部 DRAM 增长
    的，WiFi 驱动初始化也要在同一块内部 DRAM 里要一大块连续内存。等 ui、
    字库、字节码把堆撑大几十 KB 之后再 active(True)，ESP-IDF 就分不到
    内存，直接抛：

        OSError: WiFi Out of Memory

    这里的 146 KB mem_free 是 MicroPython 堆内部的空闲量，和 IDF 能不能
    拿到连续 DRAM 是两回事，别被它误导。

!! 常见的错误码 !!
    OSError: WiFi Out of Memory
        驱动初始化拿不到内部 DRAM，多半是初始化顺序问题（见上）。

    RuntimeError: Wifi Unknown Error 0x0102
        = ESP_ERR_INVALID_ARG，esp_wifi_set_config() 直接拒绝了参数：
        SSID 为空或超过 32 字节，密码为 1~7 字节（合法值只有 0 或 8~64）。
        注意单位是“字节”，一个汉字算 3 字节。

    status() == 2 / 3 / 4
        密码错 / 找不到 AP（多半是 5 GHz 或热点没开）/ 建连失败（WPA3）。

演示要点：
    * 手机热点必须是 2.4 GHz。ESP32-WROOM 不支持 5 GHz；iPhone 要打开
      个人热点里的“最大兼容性”。
    * 热点名和密码写死在这里，用纯 ASCII，别用中文。
    * 连不上不会卡死整个程序：main 里连不上照样能按键操作，只是网页
      打不开。掉线后由 maintain() 每 RETRY_MS 重踢一次连接。
    * connect() 本身不阻塞（真正的建连在后台跑），所以重连不会拖慢
      主循环。
"""

import gc
import time

import network


# ---------------------------------------------------------------------------
# 配置（改成你的热点）
# ---------------------------------------------------------------------------

SSID = "demo-esp32"
PASSWORD = "12345678"

# ---------------------------------------------------------------------------
# 组网方式：这一行决定谁当热点
# ---------------------------------------------------------------------------
# False → 板子当客户端，连手机热点。需要热点正常工作，而且手机能访问热点
#         下的其它设备（有些机型开了"客户端隔离"就不行）。
# True  → 板子自己开热点，手机直接连板子。演示时更稳，不用管 5 GHz、
#         客户端隔离、DHCP 抽风、热点自动关闭这些事。
#
# 切成 True 之后：手机 WiFi 里选 AP_SSID，密码 AP_PASSWORD，
# 浏览器打开 http://192.168.4.1/
AP_MODE = False
AP_SSID = "pendulum"
AP_PASSWORD = "12345678"        # 至少 8 位

# 想用固定 IP 就把下面这行改成四元组，否则保持 None 走 DHCP。
# 手机热点通常是 192.168.43.x，网关 192.168.43.1（安卓）。
# 实测你现在拿到的是 192.168.43.74，所以 .50 是空闲的：
# STATIC_IP = ("192.168.43.50", "255.255.255.0",
#              "192.168.43.1", "192.168.43.1")
STATIC_IP = None

# 掉线后的重连间隔（毫秒）
RETRY_MS = 10000
# 等待 isconnected() 时的轮询间隔
STEP_MS = 200

# WLAN.status() 的终态失败码：2 密码错、3 找不到 AP、4 建连失败。
# 遇到这几个不用再干等了。
_FAILED = (2, 3, 4)

_wlan = None
_last_retry = 0


def start():
    """打开 STA 接口并返回 WLAN 对象；重复调用是安全的。

    软复位（Thonny 的 Run、Ctrl-D）不会重置 ESP-IDF，只重置 MicroPython
    的 VM：上一轮的 WiFi 驱动还开着，但 Python 对象全是新的。这时
    active() 返回 True，如果直接跳过初始化，拿到的就是一个“看着开着、
    配置还是上一轮的”接口，connect() 会抛：

        RuntimeError: Wifi Unknown Error 0x0102

    所以发现已经开着就先关掉再开，重新要一份干净的。
    """
    global _wlan
    if _wlan is None:
        _wlan = network.WLAN(network.AP_IF if AP_MODE else network.STA_IF)

    try:
        if _wlan.active():
            print("[wifi] driver left over, re-init")
            _wlan.active(False)
            time.sleep_ms(200)
    except Exception:
        pass

    if AP_MODE:
        _wlan.active(True)
        _config_ap(_wlan)
        return _wlan

    _wlan.active(True)
    _apply_ip(_wlan)
    return _wlan


def _config_ap(w):
    """设置热点名、密码和加密方式。

    必须显式设 authmode：ESP-IDF 的 AP 默认是 OPEN，只给 password 的话
    它会被直接忽略，热点以“开放网络”启动，手机看到的就是一个没有小锁的
    SSID，而且往往还连不上。不同固件对 ssid / essid 的拼写不一样，两个都试。
    """
    for ssid_key in ("ssid", "essid"):
        cfg = {
            "password": AP_PASSWORD,
            "authmode": network.AUTH_WPA_WPA2_PSK,
        }
        cfg[ssid_key] = AP_SSID
        try:
            w.config(**cfg)
        except (ValueError, TypeError):
            continue                # 这个固件不认 ssid，换 essid 再试
        except Exception as err:
            print("[wifi] ap config fail:", err)
            return
        break
    else:
        print("[wifi] ap config rejected")
        return

    # 把实际生效的值打出来：authmode 应该是 4（WPA/WPA2 混合）
    try:
        print("[wifi] ap authmode:", w.config("authmode"))
    except Exception:
        pass


def _apply_ip(w):
    """填了 STATIC_IP 就设静态地址；失败就当没填，继续走 DHCP。"""
    if STATIC_IP:
        try:
            w.ifconfig(STATIC_IP)
        except OSError:
            pass


def _check_credentials():
    """连之前先把两个参数验一遍，把 0x0102 换成一句人话。

    驱动校验的是字节数，不是字符数：一个汉字 3 字节，一个 emoji 4 字节。
    """
    try:
        ssid_len = len(SSID.encode())
        pwd_len = len(PASSWORD.encode())
    except AttributeError:
        # SSID 被写成了非字符串，比如漏了引号
        print("[wifi] SSID/PASSWORD 必须是字符串")
        return False

    print("[wifi] ssid %d bytes, pwd %d bytes" % (ssid_len, pwd_len))

    if ssid_len < 1 or ssid_len > 32:
        print("[wifi] SSID 长度非法：要求 1-32 字节，现在是", ssid_len)
        return False
    if pwd_len > 64:
        print("[wifi] 密码太长：要求最多 64 字节，现在是", pwd_len)
        return False
    if 0 < pwd_len < 8:
        print("[wifi] 密码太短：合法值只有 0 或 8-64 字节，现在是", pwd_len)
        return False
    return True


def boot_connect(timeout_ms=15000):
    """开机时调用，而且必须早于 import ui / st7789 之类的模块。

    任何异常（驱动 OOM、参数非法）都只打印一行并返回 False，
    绝不让开机流程挂掉 —— 按键和屏幕照常可用，只是没有网页。
    """
    try:
        # 这两个数字用来区分“Python 堆不够”和“IDF 拿不到连续 DRAM”。
        # 报 WiFi Out of Memory 时这里通常还剩 140 KB 以上，属于后者。
        print("[wifi] heap before:", gc.mem_free())
        ok = connect(timeout_ms)
        print("[wifi] heap after:", gc.mem_free())
        return ok
    except Exception as err:
        # 不能只捕 OSError：参数非法抛的是 RuntimeError，
        # 漏掉它整个开机流程就停了，屏幕停在黑屏。
        print("[wifi] init failed:", err)
        return False


def connect(timeout_ms=20000):
    """发起连接并等待，返回是否连上。AP 模式下等于“等接口拿到地址”。"""
    global _last_retry

    if AP_MODE:
        w = start()
        start_ms = time.ticks_ms()
        while True:
            try:
                if w.ifconfig()[0] != "0.0.0.0":
                    return True
            except Exception:
                pass
            if time.ticks_diff(time.ticks_ms(), start_ms) >= timeout_ms:
                return False
            time.sleep_ms(STEP_MS)

    if not _check_credentials():
        _last_retry = time.ticks_ms()
        return False

    w = start()
    _apply_ip(w)

    try:
        w.connect(SSID, PASSWORD)
    except Exception as err:
        print("[wifi] connect rejected:", err)
        _last_retry = time.ticks_ms()
        return False

    if wait(timeout_ms):
        return True

    try:
        print("[wifi] status:", w.status())
    except Exception:
        pass
    _last_retry = time.ticks_ms()
    return False


def wait(timeout_ms=20000):
    """等连上；遇到密码错/找不到 AP 立刻返回，不干等到超时。"""
    w = _wlan
    if w is None:
        return False

    start_ms = time.ticks_ms()
    try:
        while not w.isconnected():
            if w.status() in _FAILED:
                return False
            if time.ticks_diff(time.ticks_ms(), start_ms) >= timeout_ms:
                return False
            time.sleep_ms(STEP_MS)
    except Exception:
        return False
    return True


def is_up():
    """网络是否就绪。接口没起来时不抛异常，直接算没连上。"""
    w = _wlan
    if w is None:
        return False
    try:
        if AP_MODE:
            # AP 模式下 isconnected() 的意思是“有客户端连进来”，
            # 这里要的是“接口自己有地址”
            return w.active() and w.ifconfig()[0] != "0.0.0.0"
        return w.isconnected()
    except Exception:
        return False


def ip():
    """返回当前 IP；没连上时是 '0.0.0.0'。"""
    w = _wlan
    if w is None:
        return "0.0.0.0"
    try:
        return w.ifconfig()[0]
    except Exception:
        return "0.0.0.0"


def maintain():
    """主循环里调用：掉线时每 RETRY_MS 重踢一次连接。

    connect() 只是把建连请求丢给后台，不会阻塞，所以可以放心在
    主循环里调用。接口根本没起来（开机就 OOM）时什么都不做，
    那种情况只能复位重来。
    """
    global _last_retry

    if AP_MODE:
        return                          # 热点模式没有“重连”这回事

    w = _wlan
    if w is None:
        return

    try:
        if not w.active():
            return
        if w.isconnected():
            return
        if w.status() == 1:             # 1 = 还在连，别打断
            return

        now = time.ticks_ms()
        if time.ticks_diff(now, _last_retry) < RETRY_MS:
            return
        _last_retry = now
        w.connect(SSID, PASSWORD)
    except Exception:
        # 这里绝不能把异常放出去，否则整个刷屏主循环就停了
        _last_retry = time.ticks_ms()
