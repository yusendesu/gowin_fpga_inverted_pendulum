"""
极简 HTTP 服务：给手机一个页面，能下发旋转角。

设计目标（配合 tmp.py 那个 5 ms 的主循环）：
    * 全程非阻塞。所有 socket 都是 settimeout(0)，每次 poll() 最多收发
      CHUNK 字节，卡顿在毫秒级。不会拖慢刷屏，也不会让串口缓冲溢出。
    * 一次只服务一个连接。页面是单文件，浏览器只开一条连接。
    * 页面放 flash，按 512 字节流式发送，不整页读进内存（省几 KB 堆）。
    * 请求缓冲固定 1 KB。收全请求头（\\r\\n\\r\\n）就处理，不解析 body；
      读超时 800 ms，专门对付浏览器的“预连接”空连接；超长垃圾请求直接断开。
    * poll() 里兜住所有异常：任何网络意外都不许把刷屏主循环带崩。

路由：
    GET /             控制页面（www/index.html）
    GET /s?x=-90      下发旋转角，-180..180，复用 uart 的组帧函数
    GET /favicon.ico  返回 204，省掉一次 404

用法::

    import web
    web.init(link)     # link = uart.init(view)
    # 然后在主循环里反复调用
    web.poll()
"""

import os
import socket
import time


# ---------------------------------------------------------------------------
# 配置
# ---------------------------------------------------------------------------

PAGE_FILE = "www/index.html"
PORT = 80

# 一次 poll 最多收发的字节数；也是文件分块的大小
CHUNK = 512
# 请求缓冲上限（要装得下整个请求头，浏览器会带 User-Agent 等）
REQ_MAX = 1024

# 等请求头的超时。手机上 RTT 才几十毫秒，800 ms 足够；
# 这个值小一点，浏览器的“预连接”空连接才不会把服务占住。
READ_TIMEOUT_MS = 800
# 回响应期间的超时，客户端收得慢也不能一直挂着
SEND_TIMEOUT_MS = 5000
# 页面缓存时间，避免演示时反复拉页面
PAGE_CACHE_S = 300

# 非阻塞 socket 的“再试一次”
_EAGAIN = 11

# 调试开关：打印每次连接、每条请求行、以及被捕获的异常。
# 排障期开着，确认能用之后改成 False（打印会占串口时间）。
DEBUG = True

_ST_READ = 0
_ST_SEND = 1


def _parse_int(data):
    """把 b'-90' 这样的字节串解析成 int，非法返回 None。

    不用 int(bytes) 是为了不依赖各版本对 bytes 的转换语义，
    而且省掉一次异常抛出。
    """
    if not data:
        return None

    sign = 1
    i = 0
    if data[0] == 45:               # '-'
        sign = -1
        i = 1
    elif data[0] == 43:             # '+'
        i = 1

    if i >= len(data):
        return None

    value = 0
    while i < len(data):
        digit = data[i] - 48
        if digit < 0 or digit > 9:
            return None
        value = value * 10 + digit
        i += 1
    return sign * value


class WebServer:
    """单连接、非阻塞的 HTTP 服务。"""

    def __init__(self, link=None):
        self.link = link

        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            srv.bind(("0.0.0.0", PORT))
            srv.listen(2)
            srv.settimeout(0.0)
        except OSError:
            # 失败时显式关掉，免得重试几次把 socket 池吃光
            try:
                srv.close()
            except OSError:
                pass
            raise
        self._srv = srv

        self._conn = None
        self._state = _ST_READ
        self._deadline = 0

        self._req = bytearray(REQ_MAX)
        self._req_len = 0

        self._hdr = b""
        self._hdr_off = 0
        self._body = None
        self._body_off = 0
        self._file = None

        self.requests = 0
        self.commands = 0

    def attach(self, link):
        """UART 起来之后再把 link 挂上来。

        监听 socket 必须建在 ui.init() 之前（那时内部 RAM 还没被 UI 缓冲
        吃掉），而 link 要等 ui.init() 之后才存在，所以拆成两步。
        """
        self.link = link

    # ------------------------------------------------------------------
    # 主循环入口
    # ------------------------------------------------------------------

    def poll(self):
        """在主循环里反复调用；每次最多推进 CHUNK 字节。"""
        try:
            self._poll()
        except Exception as err:
            # 以前这里静默吞掉，什么都看不见。现在打出来。
            if DEBUG:
                print("[web] err", err)
            self._close()

    def _poll(self):
        if self._conn is None:
            self._accept()
            if self._conn is None:
                return

        if self._state == _ST_READ:
            self._read()
        else:
            self._send()

        self._check_timeout()

    # ------------------------------------------------------------------
    # 连接管理
    # ------------------------------------------------------------------

    def _accept(self):
        """接受一个连接。忙的时候不 accept，让新连接在 backlog 里排队。"""
        try:
            conn, addr = self._srv.accept()
        except OSError:
            return
        if conn is None:
            return

        if DEBUG:
            print("[web] conn", addr)

        try:
            conn.settimeout(0.0)
        except OSError:
            pass

        self._conn = conn
        self._state = _ST_READ
        self._req_len = 0
        self._hdr = b""
        self._hdr_off = 0
        self._body = None
        self._body_off = 0
        self._file = None
        self._deadline = time.ticks_add(time.ticks_ms(), READ_TIMEOUT_MS)

    def _close(self):
        """收尾：关连接、关文件、清状态。"""
        if self._conn is not None:
            try:
                self._conn.close()
            except OSError:
                pass
            self._conn = None
        if self._file is not None:
            try:
                self._file.close()
            except OSError:
                pass
            self._file = None

        self._hdr = b""
        self._hdr_off = 0
        self._body = None
        self._body_off = 0
        self._state = _ST_READ
        self._req_len = 0

    def _check_timeout(self):
        if self._conn is None:
            return
        if time.ticks_diff(time.ticks_ms(), self._deadline) > 0:
            self._close()

    # ------------------------------------------------------------------
    # 收请求
    # ------------------------------------------------------------------

    def _read(self):
        conn = self._conn
        room = REQ_MAX - self._req_len
        if room <= 0:
            self._close()
            return

        try:
            data = conn.recv(room)
        except OSError as err:
            if err.args[0] == _EAGAIN:
                return                  # 还没数据，下一轮再看
            self._close()
            return

        if not data:
            self._close()               # 对端关了
            return

        self._req[self._req_len:self._req_len + len(data)] = data
        self._req_len += len(data)

        head = bytes(self._req[:self._req_len])
        if head.find(b"\r\n\r\n") < 0:
            return                      # 请求头还没收全

        line_end = head.find(b"\r\n")
        if line_end < 0:
            self._close()
            return

        self.requests += 1
        self._route(head[:line_end])

    def _route(self, line):
        """只解析请求行：GET /s?x=-90 HTTP/1.1"""
        parts = line.split(b" ")
        if len(parts) < 2:
            self._close()
            return

        path = parts[1]
        if DEBUG:
            print("[web] req", line)
        if path == b"/s" or path.startswith(b"/s?"):
            self._handle_command(path)
        elif path == b"/" or path.startswith(b"/index"):
            self._serve_page()
        elif path.startswith(b"/favicon"):
            self._respond(b"204 NC", b"text/plain", body=b"")
        else:
            self._respond(b"404 NF", b"text/plain", body=b"not found")

    def _handle_command(self, path):
        """GET /s?x=-90 —— 下发旋转角。复用 uart 的组帧，别自己造协议。"""
        if self.link is None:
            self._respond(b"503 NA", b"application/json",
                          body=b'{"ok":0}')
            return

        angle = 0
        q = path.find(b"?")
        if q >= 0:
            for pair in path[q + 1:].split(b"&"):
                if pair.startswith(b"x="):
                    value = _parse_int(pair[2:])
                    if value is not None:
                        angle = value
                    break

        if angle > 180:
            angle = 180
        elif angle < -180:
            angle = -180

        try:
            self.link.send_rotation(angle)
        except OSError:
            self._respond(b"500 ERR", b"application/json",
                          body=b'{"ok":0}')
            return

        self.commands += 1
        self._respond(b"200 OK", b"application/json",
                      body=b'{"ok":1,"x":%d}' % angle)

    def _serve_page(self):
        """页面从 flash 流式发，不整页读进内存。"""
        try:
            size = os.stat(PAGE_FILE)[6]
            handle = open(PAGE_FILE, "rb")
        except OSError:
            self._respond(
                b"500 ERR", b"text/plain",
                body=b"missing www/index.html on the board",
            )
            return

        self._respond(b"200 OK", b"text/html; charset=utf-8",
                      f=handle, size=size, cache_s=PAGE_CACHE_S)

    # ------------------------------------------------------------------
    # 回响应
    # ------------------------------------------------------------------

    def _respond(self, code, ctype, body=None, f=None, size=0, cache_s=0):
        """准备响应头。body 是要发的内存数据，f 是要流式读的文件。"""
        length = size if f is not None else len(body)
        cache = b"no-store" if cache_s <= 0 else b"max-age=%d" % cache_s

        self._hdr = (
            b"HTTP/1.1 %s\r\n"
            b"Content-Type: %s\r\n"
            b"Content-Length: %d\r\n"
            b"Cache-Control: %s\r\n"
            b"Connection: close\r\n"
            b"\r\n"
        ) % (code, ctype, length, cache)

        self._hdr_off = 0
        self._body = body
        self._body_off = 0
        self._file = f
        self._state = _ST_SEND
        self._deadline = time.ticks_add(time.ticks_ms(), SEND_TIMEOUT_MS)

    def _send(self):
        """把响应头、内存 body、文件内容依次推出去，每轮最多 CHUNK 字节。"""
        budget = CHUNK

        while budget > 0:
            if self._hdr_off < len(self._hdr):
                sent = self._try_send(memoryview(self._hdr)[self._hdr_off:])
                if sent == 0:
                    return
                self._hdr_off += sent
                budget -= sent
                continue

            if self._body is not None and self._body_off < len(self._body):
                sent = self._try_send(
                    memoryview(self._body)[self._body_off:]
                )
                if sent == 0:
                    return
                self._body_off += sent
                budget -= sent
                continue

            if self._file is not None:
                chunk = self._file.read(CHUNK)
                if not chunk:
                    self._close()
                    return
                self._body = chunk
                self._body_off = 0
                budget = 0              # 这一块发完就让出主循环
                continue

            self._close()
            return

    def _try_send(self, view):
        """发一次；返回实际发出的字节数，0 表示对方暂时收不下或已断开。"""
        try:
            sent = self._conn.send(view)
        except OSError as err:
            if err.args[0] == _EAGAIN:
                return 0                # TCP 缓冲满了，下一轮再发
            self._close()
            return 0

        if not sent or sent <= 0:
            self._close()
            return 0

        # 有进展就把超时往后推，客户端慢也不算卡死
        self._deadline = time.ticks_add(time.ticks_ms(), SEND_TIMEOUT_MS)
        return sent


# ---------------------------------------------------------------------------
# 模块级便捷接口（和 uart.py 保持一致的用法）
# ---------------------------------------------------------------------------

_server = None


def init(link=None):
    """创建 HTTP 服务（只建监听 socket）；调用前不需要 uart。

    要尽早调用：socket 也要内部 RAM，排在 ui.init() 的 18 KB 缓冲之后
    就会报 [Errno 105] ENOBUFS。
    """
    global _server
    _server = WebServer(link)
    return _server


def attach(link):
    """把 uart 的 link 挂上来；UART 初始化完成之后调用。"""
    if _server is not None:
        _server.attach(link)


def is_ready():
    """监听 socket 是否已经建好（boot.py 里建，tmp.main 里查）。"""
    return _server is not None


def poll():
    """在主循环中反复调用。"""
    if _server is not None:
        _server.poll()
