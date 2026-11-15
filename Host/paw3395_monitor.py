#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PAW3395 + STM32F103RCT6 上位机接收工具
=====================================
依赖：pyserial（pip install pyserial）、tkinter（Python 自带）

功能：
  1. 选择串口 / 波特率，连接或断开
  2. 解析 STM32 通过 USART1 发来的 15 字节二进制帧（AA 55 LEN SEQ payload CHK）
  3. 实时显示 ΔX/ΔY/SQUAL/CPI/MOTION/SHUTTER、帧率与错误统计
  4. 绘制累计运动轨迹 + ΔX/ΔY 波形
  5. 显示原始十六进制字节流

帧格式（小端，共 15 字节）：
  0      0xAA  帧头1
  1      0x55  帧头2
  2      10    payload 长度
  3      SEQ   帧序号
  4-5    ΔX    int16
  6-7    ΔY    int16
  8      SQUAL uint8
  9-10   CPI   uint16
  11     MOTION 原始字节（bit7=MOT, bit3=Lift_Stat, bit1:0=OP_Mode）
  12-13  SHUTTER uint16
  14     CHK   前 14 字节累加和的低 8 位
"""

import queue
import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox
from collections import deque

try:
    import serial
    from serial.tools import list_ports
except ImportError:  # 未安装 pyserial 时给出明确提示
    serial = None
    list_ports = None

FRAME_HEAD0 = 0xAA
FRAME_HEAD1 = 0x55
PAYLOAD_LEN = 10
FRAME_LEN = 4 + PAYLOAD_LEN + 1

DEFAULT_BAUD = "460800"
BAUD_LIST = ["115200", "230400", "460800", "921600", "1000000"]
TRAJ_POINTS = 3000      # 轨迹最多保留点数（1kHz 时约 3 秒）
WAVE_POINTS = 500       # 波形最多保留点数（1kHz 时约 0.5 秒）

# 性能相关：数据接收与界面刷新解耦，界面按固定频率刷新
UI_TICK_MS = 100        # 界面刷新周期(ms)：数据再快也只按这个频率刷
RAW_MAX_PENDING = 2048  # 原始数据缓存上限(字节)，超出丢弃最旧的
RAW_FLUSH_BYTES = 256   # 每次刷新最多往文本框写多少字节
RAW_MAX_LINES = 400     # 文本框最多保留行数
RAW_KEEP_LINES = 300    # 超出上限后保留的行数


# ----------------------------------------------------------------------------
# 帧解析（字节流 -> 完整帧）
# ----------------------------------------------------------------------------
class FrameParser(object):
    def __init__(self):
        self.buf = bytearray()
        self.bad_cnt = 0        # 长度错 / 校验错的帧数

    def feed(self, data):
        """输入任意长度字节流，返回本次解析出的完整帧列表"""
        self.buf.extend(data)
        frames = []

        while True:
            idx = self.buf.find(b"\xAA\x55")
            if idx < 0:
                # 没有帧头：只保留最后 1 字节（可能是 0xAA）
                if len(self.buf) > 1:
                    del self.buf[:-1]
                break
            if idx > 0:
                del self.buf[:idx]          # 丢弃帧头前的噪声
            if len(self.buf) < 4:
                break

            plen = self.buf[2]
            total = 4 + plen + 1
            if plen != PAYLOAD_LEN:
                del self.buf[0]             # 长度不对，滑动 1 字节重新同步
                self.bad_cnt += 1
                continue
            if len(self.buf) < total:
                break                       # 半帧，等后续数据

            frame = bytes(self.buf[:total])
            if (sum(frame[:-1]) & 0xFF) != frame[-1]:
                del self.buf[0]             # 校验错，滑动 1 字节重新同步
                self.bad_cnt += 1
                continue

            del self.buf[:total]
            frames.append(frame)

        return frames

    def reset(self):
        self.buf.clear()
        self.bad_cnt = 0


def decode(frame):
    """把 15 字节帧解析成字典"""
    return {
        "raw": frame,
        "seq": frame[3],
        "dx": int.from_bytes(frame[4:6], "little", signed=True),
        "dy": int.from_bytes(frame[6:8], "little", signed=True),
        "squal": frame[8],
        "cpi": int.from_bytes(frame[9:11], "little"),
        "motion": frame[11],
        "shutter": int.from_bytes(frame[12:14], "little"),
    }


# ----------------------------------------------------------------------------
# 串口接收线程
# ----------------------------------------------------------------------------
class SerialReader(threading.Thread):
    def __init__(self, port, baud, out_queue, parser):
        super().__init__(daemon=True)
        self.port = port
        self.baud = baud
        self.out_queue = out_queue
        self.parser = parser
        self._stop = threading.Event()
        self.ser = None
        self.error = None

    def run(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.05)
        except Exception as exc:                      # 串口打不开
            self.error = str(exc)
            self.out_queue.put(("error", str(exc)))
            return

        self.out_queue.put(("open", self.port))
        try:
            while not self._stop.is_set():
                data = self.ser.read(4096)
                if not data:
                    continue
                self.out_queue.put(("bytes", data))
                for frame in self.parser.feed(data):
                    self.out_queue.put(("frame", decode(frame)))
        except Exception as exc:
            self.out_queue.put(("error", str(exc)))
        finally:
            try:
                if self.ser is not None and self.ser.is_open:
                    self.ser.close()
            except Exception:
                pass
            self.out_queue.put(("closed", None))

    def stop(self):
        self._stop.set()


# ----------------------------------------------------------------------------
# 主界面
# ----------------------------------------------------------------------------
class MonitorApp(object):
    def __init__(self, root):
        self.root = root
        self.root.title("PAW3395 上位机 - STM32F103RCT6 USART1 数据接收")
        self.root.geometry("1060x760")
        self.root.minsize(980, 660)

        self.rx_queue = queue.Queue()
        self.parser = FrameParser()
        self.reader = None
        self.frame_cnt = 0
        self.rx_bytes = 0
        self.last_seq = None
        self.lost_cnt = 0
        # 帧率统计窗口：必须比最高帧率大，否则显示会被截顶
        self.rate_ts = deque(maxlen=4000)

        self.traj = deque(maxlen=TRAJ_POINTS)   # (x, y) 累计轨迹
        self.pos_x = 0
        self.pos_y = 0
        self.wave_x = deque(maxlen=WAVE_POINTS)
        self.wave_y = deque(maxlen=WAVE_POINTS)

        # 数据与界面解耦用：接收线程只写这些普通变量，
        # 界面按固定频率（UI_TICK_MS）刷一次，避免每帧都去动 Tk 控件
        self.last_frame = None          # 最近一帧，供界面显示
        self.raw_pending = bytearray()  # 待显示的原始字节
        self.raw_dropped = 0            # 原始显示跟不上的丢字节数
        self.last_draw_cnt = -1

        self._build_ui()
        self._refresh_ports()
        self.root.after(10, self._poll_queue)
        self.root.after(UI_TICK_MS, self._tick_ui)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    # ---------------- 界面 ----------------
    def _build_ui(self):
        top = ttk.Frame(self.root, padding=8)
        top.pack(side=tk.TOP, fill=tk.X)

        ttk.Label(top, text="串口:").pack(side=tk.LEFT)
        self.cmb_port = ttk.Combobox(top, width=22, state="readonly")
        self.cmb_port.pack(side=tk.LEFT, padx=(4, 8))

        ttk.Button(top, text="刷新", width=6,
                   command=self._refresh_ports).pack(side=tk.LEFT)

        ttk.Label(top, text="波特率:").pack(side=tk.LEFT, padx=(12, 0))
        self.cmb_baud = ttk.Combobox(top, width=10, state="readonly",
                                     values=BAUD_LIST)
        self.cmb_baud.set(DEFAULT_BAUD)
        self.cmb_baud.pack(side=tk.LEFT, padx=(4, 12))

        self.btn_conn = ttk.Button(top, text="连接", width=10,
                                   command=self._toggle_connect)
        self.btn_conn.pack(side=tk.LEFT)
        ttk.Button(top, text="清零", width=8,
                   command=self._clear).pack(side=tk.LEFT, padx=6)

        self.lbl_state = ttk.Label(top, text="未连接", foreground="#b00")
        self.lbl_state.pack(side=tk.LEFT, padx=12)

        body = ttk.Frame(self.root, padding=(8, 0, 8, 8))
        body.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        # 左侧：数值面板（只放窄控件，避免把右侧画布挤没）
        left = ttk.LabelFrame(body, text="解析结果", padding=8)
        left.pack(side=tk.LEFT, fill=tk.Y)

        self.vars = {}
        rows = [
            ("seq", "帧序号 SEQ"), ("dx", "ΔX"), ("dy", "ΔY"),
            ("posx", "绝对坐标 X"), ("posy", "绝对坐标 Y"),
            ("posmm", "位移 X,Y (mm)"),
            ("squal", "SQUAL 表面质量"), ("cpi", "CPI"),
            ("motion", "MOTION 原始值"), ("shutter", "SHUTTER"),
        ]
        for i, (key, text) in enumerate(rows):
            ttk.Label(left, text=text + ":", width=14, anchor="w").grid(
                row=i, column=0, sticky="w", pady=2)
            var = tk.StringVar(value="-")
            ttk.Label(left, textvariable=var, width=15, anchor="w",
                      font=("Consolas", 11, "bold")).grid(
                row=i, column=1, sticky="w", pady=2)
            self.vars[key] = var

        ttk.Separator(left, orient="horizontal").grid(
            row=len(rows), column=0, columnspan=2, sticky="ew", pady=6)

        stat_rows = [
            ("rate", "接收帧率"), ("total", "累计帧数"),
            ("rxbytes", "已收字节"), ("lost", "丢帧(序号跳变)"),
            ("bad", "坏帧(长度/校验)"),
        ]
        for j, (key, text) in enumerate(stat_rows):
            ttk.Label(left, text=text + ":", width=14, anchor="w").grid(
                row=len(rows) + 1 + j, column=0, sticky="w", pady=2)
            var = tk.StringVar(value="0")
            ttk.Label(left, textvariable=var, width=15, anchor="w").grid(
                row=len(rows) + 1 + j, column=1, sticky="w", pady=2)
            self.vars[key] = var

        self.raw_show = tk.BooleanVar(value=False)
        ttk.Checkbutton(left, text="显示原始字节", variable=self.raw_show).grid(
            row=len(rows) + 1 + len(stat_rows), column=0, columnspan=2,
            sticky="w", pady=(10, 0))

        # 右侧：轨迹图（自适应拉伸）+ 波形图 + 原始数据
        right = ttk.Frame(body)
        right.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(8, 0))

        traj_box = ttk.LabelFrame(right, text="绝对坐标轨迹 (X 向右, Y 向下)",
                                  padding=4)
        traj_box.pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        self.cv_traj = tk.Canvas(traj_box, bg="#101418", highlightthickness=0,
                                 width=460, height=320)
        self.cv_traj.pack(fill=tk.BOTH, expand=True)

        wave_box = ttk.LabelFrame(
            right, text="ΔX / ΔY 波形 (最近 %d 帧)" % WAVE_POINTS, padding=4)
        wave_box.pack(side=tk.TOP, fill=tk.X, pady=(6, 0))
        self.cv_wave = tk.Canvas(wave_box, bg="#101418", highlightthickness=0,
                                 width=460, height=130)
        self.cv_wave.pack(fill=tk.X)

        raw_box = ttk.LabelFrame(right, text="原始字节 (HEX  |  ASCII)",
                                 padding=4)
        raw_box.pack(side=tk.TOP, fill=tk.X, pady=(6, 0))
        self.txt_raw = tk.Text(raw_box, height=8, wrap=tk.NONE,
                               font=("Consolas", 9))
        self.txt_raw.pack(side=tk.LEFT, fill=tk.X, expand=True)
        self.raw_lines = 0

    # ---------------- 串口 ----------------
    def _refresh_ports(self):
        devs, usb_devs = [], []
        if list_ports is not None:
            for p in list_ports.comports():
                devs.append(p.device)
                # 电脑上常有一堆“蓝牙链接上的标准串行”，这里优先选真正的 USB 串口
                if "USB" in (p.description or "").upper():
                    usb_devs.append(p.device)
        self.cmb_port["values"] = devs
        if self.cmb_port.get() in devs:
            return                          # 当前选择仍然有效，保持不变
        if usb_devs:
            self.cmb_port.set(usb_devs[0])  # 默认选 USB 串口，别默认选蓝牙
        elif devs:
            self.cmb_port.set(devs[0])

    def _toggle_connect(self):
        if self.reader is not None:
            self._disconnect()
        else:
            self._connect()

    def _connect(self):
        if serial is None:
            messagebox.showerror("缺少依赖", "未安装 pyserial，请执行：\n"
                                             "pip install pyserial")
            return
        port = self.cmb_port.get()
        if not port:
            messagebox.showwarning("提示", "请选择串口")
            return
        try:
            baud = int(self.cmb_baud.get())
        except ValueError:
            messagebox.showwarning("提示", "波特率无效")
            return

        self.parser.reset()
        self.reader = SerialReader(port, baud, self.rx_queue, self.parser)
        self.reader.start()
        self.btn_conn.config(text="断开")

    def _disconnect(self):
        if self.reader is not None:
            self.reader.stop()
            self.reader = None
        self.btn_conn.config(text="连接")
        self.lbl_state.config(text="未连接", foreground="#b00")

    def _on_close(self):
        self._disconnect()
        self.root.destroy()

    # ---------------- 数据流 ----------------
    def _poll_queue(self):
        """只做“搬数据”，绝不碰 Tk 控件；界面刷新交给 _tick_ui"""
        pending = 0
        try:
            while True:
                kind, payload = self.rx_queue.get_nowait()
                pending += 1
                if kind == "frame":
                    self._on_frame(payload)
                elif kind == "bytes":
                    self.rx_bytes += len(payload)
                    if self.raw_show.get():
                        # 只缓存，不做任何界面操作；超过 2KB 就丢最旧的
                        self.raw_pending.extend(payload)
                        if len(self.raw_pending) > RAW_MAX_PENDING:
                            drop = len(self.raw_pending) - RAW_MAX_PENDING
                            del self.raw_pending[:drop]
                            self.raw_dropped += drop
                    else:
                        self.raw_pending.clear()
                elif kind == "open":
                    self.lbl_state.config(
                        text="已连接 %s" % payload, foreground="#0a0")
                elif kind == "closed":
                    self.lbl_state.config(text="未连接", foreground="#b00")
                elif kind == "error":
                    self.lbl_state.config(text="错误: %s" % payload,
                                          foreground="#b00")
                    self.btn_conn.config(text="连接")
                    self.reader = None
                if pending >= 2000:     # 防止一次吃太多，留时间给界面
                    break
        except queue.Empty:
            pass
        self.root.after(10, self._poll_queue)

    def _on_frame(self, f):
        """纯计算，不做任何 Tk 调用"""
        self.frame_cnt += 1
        self.rate_ts.append(time.time())

        if self.last_seq is not None:
            gap = (f["seq"] - self.last_seq - 1) & 0xFF
            if gap:
                self.lost_cnt += gap
        self.last_seq = f["seq"]

        self.pos_x += f["dx"]
        self.pos_y += f["dy"]
        self.traj.append((self.pos_x, self.pos_y))
        self.wave_x.append(f["dx"])
        self.wave_y.append(f["dy"])
        self.last_frame = f

    def _flush_raw(self):
        """把缓存的原始字节按限量刷到文本框，避免一次性插入太多把界面卡死"""
        if not self.raw_pending:
            return
        chunk = bytes(self.raw_pending[:RAW_FLUSH_BYTES])
        del self.raw_pending[:RAW_FLUSH_BYTES]

        lines = []
        for i in range(0, len(chunk), 16):
            seg = chunk[i:i + 16]
            hexs = " ".join("%02X" % b for b in seg)
            text = "".join(chr(b) if 32 <= b < 127 else "." for b in seg)
            lines.append("%-47s |%s|" % (hexs, text))
            self.raw_lines += 1

        if self.raw_dropped:
            lines.append("... 已省略 %d 字节（显示跟不上接收速率）..."
                         % self.raw_dropped)
            self.raw_dropped = 0

        self.txt_raw.insert(tk.END, "\n".join(lines) + "\n")
        if self.raw_lines > RAW_MAX_LINES:      # 限制显示长度
            cut = self.raw_lines - RAW_KEEP_LINES
            self.txt_raw.delete("1.0", "%d.0" % cut)
            self.raw_lines = RAW_KEEP_LINES
        self.txt_raw.see(tk.END)

    # ---------------- 周期刷新 ----------------
    def _tick_ui(self):
        # 1) 数值面板：只取“最近一帧”，每 UI_TICK_MS 刷一次就够了
        f = self.last_frame
        if f is not None:
            self.vars["seq"].set(str(f["seq"]))
            self.vars["dx"].set(str(f["dx"]))
            self.vars["dy"].set(str(f["dy"]))
            self.vars["squal"].set(str(f["squal"]))
            self.vars["cpi"].set(str(f["cpi"]))
            self.vars["motion"].set("0x%02X" % f["motion"])
            self.vars["shutter"].set(str(f["shutter"]))

        # 2) 绝对坐标（上电/清零以来的累计位移），始终刷新
        self.vars["posx"].set(str(self.pos_x))
        self.vars["posy"].set(str(self.pos_y))
        cpi = f["cpi"] if f is not None else 0
        if cpi > 0:
            # 1 英寸 = CPI 个计数，1 英寸 = 25.4 mm
            self.vars["posmm"].set("%.2f, %.2f" % (self.pos_x * 25.4 / cpi,
                                                   self.pos_y * 25.4 / cpi))
        else:
            self.vars["posmm"].set("-")

        # 3) 统计信息
        now = time.time()
        while self.rate_ts and now - self.rate_ts[0] > 1.0:
            self.rate_ts.popleft()
        self.vars["rate"].set("%d 帧/秒" % len(self.rate_ts))
        self.vars["total"].set(str(self.frame_cnt))
        self.vars["rxbytes"].set(str(self.rx_bytes))
        self.vars["lost"].set(str(self.lost_cnt))
        self.vars["bad"].set(str(self.parser.bad_cnt))

        # 4) 原始数据显示（限量）
        self._flush_raw()

        # 5) 图形：没有新数据就不重画
        if self.frame_cnt != self.last_draw_cnt:
            self.last_draw_cnt = self.frame_cnt
            self._draw_traj()
            self._draw_wave()

        self.root.after(UI_TICK_MS, self._tick_ui)

    def _draw_traj(self):
        cv = self.cv_traj
        cv.delete("all")
        w = cv.winfo_width()
        h = cv.winfo_height()
        if w < 10 or h < 10:
            return
        if len(self.traj) < 2:
            cv.create_text(w / 2, h / 2, text="等待数据...", fill="#6b7280")
            return

        # 把原点(0,0)也算进显示范围，这样“绝对坐标”才有参照物
        xs = [p[0] for p in self.traj]
        ys = [p[1] for p in self.traj]
        min_x, max_x = min(min(xs), 0), max(max(xs), 0)
        min_y, max_y = min(min(ys), 0), max(max(ys), 0)

        # 位移范围太小时（没动 / 被抗零漂死区压成 0），所有点会重合在一个像素上，
        # 看起来就像“没显示图像”——这里给出明确文字提示
        if (max_x - min_x) < 8 and (max_y - min_y) < 8:
            cv.create_text(w / 2, h / 2, text="静止中（无位移）",
                           fill="#6b7280")
            cv.create_text(8, 8, anchor="nw", fill="#e5e7eb",
                           text="绝对坐标  X=%d  Y=%d" % (self.pos_x, self.pos_y))
            return

        span_x = max(max_x - min_x, 1)
        span_y = max(max_y - min_y, 1)
        scale = min((w - 40) / span_x, (h - 40) / span_y)

        def to_px(x, y):
            return (20 + (x - min_x) * scale,
                    20 + (y - min_y) * scale)

        # 原点十字：上电/清零时的位置
        ox, oy = to_px(0, 0)
        cv.create_line(ox - 6, oy, ox + 6, oy, fill="#4b5563")
        cv.create_line(ox, oy - 6, ox, oy + 6, fill="#4b5563")

        pts = []
        for (x, y) in self.traj:
            px, py = to_px(x, y)
            pts.extend((px, py))
        cv.create_line(*pts, fill="#22d3ee", width=2)
        px, py = to_px(self.traj[-1][0], self.traj[-1][1])
        cv.create_oval(px - 4, py - 4, px + 4, py + 4, fill="#f59e0b",
                       outline="")
        cv.create_text(8, 8, anchor="nw", fill="#e5e7eb",
                       text="绝对坐标  X=%d  Y=%d" % (self.pos_x, self.pos_y))
        cv.create_text(8, 26, anchor="nw", fill="#9ca3af",
                       text="范围 X[%d,%d] Y[%d,%d]   (十字 = 清零位置)" %
                            (min_x, max_x, min_y, max_y))

    def _draw_wave(self):
        cv = self.cv_wave
        cv.delete("all")
        w = cv.winfo_width()
        h = cv.winfo_height()
        if w < 10 or h < 10:
            return
        if len(self.wave_x) < 2:
            cv.create_text(w / 2, h / 2, text="等待数据...", fill="#6b7280")
            return

        peak = max(1, max(max(abs(v) for v in self.wave_x),
                          max(abs(v) for v in self.wave_y)))
        mid = h / 2

        def line(vals, color):
            step = w / float(WAVE_POINTS - 1)
            pts = []
            for i, v in enumerate(vals):
                pts.extend((i * step, mid - (v / peak) * (h / 2 - 6)))
            if len(pts) >= 4:
                cv.create_line(*pts, fill=color, width=1)

        cv.create_line(0, mid, w, mid, fill="#374151")
        line(self.wave_x, "#22d3ee")
        line(self.wave_y, "#f472b6")
        cv.create_text(6, 6, anchor="nw", fill="#9ca3af",
                       text="峰值 ±%d   (青=ΔX 粉=ΔY)" % peak)

    def _clear(self):
        self.frame_cnt = 0
        self.rx_bytes = 0
        self.lost_cnt = 0
        self.last_seq = None
        self.last_frame = None
        self.raw_pending.clear()
        self.raw_dropped = 0
        self.parser.bad_cnt = 0
        self.traj.clear()
        self.wave_x.clear()
        self.wave_y.clear()
        self.pos_x = 0
        self.pos_y = 0
        self.txt_raw.delete("1.0", tk.END)
        self.raw_lines = 0
        self.rate_ts.clear()
        for var in self.vars.values():
            var.set("-")


def main():
    root = tk.Tk()
    try:
        root.call("tk", "scaling", 1.2)
    except tk.TclError:
        pass
    MonitorApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
