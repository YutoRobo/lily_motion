#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Lily MCU Config Editor for Python 2
===================================

Python 2環境でSocketCANのPythonバインディング差を避けるため、
Linuxの can-utils (`candump`, `cansend`) を利用してCAN通信する。

必要条件:
  - Python 2.7
  - Tkinter
  - can-utils
  - can0 が事前にUP済み
  - `candump can0` が動作すること

MCU Config protocol:
  Request ID  = 0x080 | axis
  Response ID = 0x180 | axis

  Byte0: Command      READ=0x01
  Byte1: Config Type  HW=0x01 / SW=0x02
  Byte2: Parameter ID
  Byte3: Result
  Byte4-7: Value (little endian 32-bit)

この版はREAD / WRITE / SAVEに対応する。WRITE/SAVEはMCU側の状態制限に従う。
"""

from __future__ import print_function

import argparse
import math
import os
import re
import struct
import subprocess
import sys
import threading
import time

try:
    import Queue as queue
except ImportError:
    import queue

try:
    import Tkinter as tk
    import ttk
    import tkMessageBox as messagebox
except ImportError:
    import tkinter as tk
    from tkinter import ttk
    from tkinter import messagebox


CONFIG_CMD_BASE = 0x080
CONFIG_RESPONSE_BASE = 0x180

CONFIG_CMD_READ = 0x01
CONFIG_CMD_WRITE = 0x02
CONFIG_CMD_SAVE = 0x03

CONFIG_TYPE_HW = 0x01
CONFIG_TYPE_SW = 0x02

RESULT_TEXT = {
    0x00: "OK",
    0x01: "INVALID_PARAM",
    0x02: "INVALID_VALUE",
    0x03: "INVALID_STATE",
    0x04: "SAVE_ERROR",
    0x05: "SAVE_NOT_IMPLEMENTED",
    0x06: "STORAGE_ERROR",
}

# key, section, config type, param id, value type, label, display unit
PARAMETERS = [
    ("gear_ratio",      "HW", CONFIG_TYPE_HW, 0x01, "f32", u"Gear ratio",           ""),
    ("motor_direction", "HW", CONFIG_TYPE_HW, 0x02, "i32", u"Motor direction",      ""),
    ("joint_min_rad",    "HW", CONFIG_TYPE_HW, 0x03, "f32", u"Joint minimum",        "deg"),
    ("joint_max_rad",    "HW", CONFIG_TYPE_HW, 0x04, "f32", u"Joint maximum",        "deg"),
    ("can_termination",  "HW", CONFIG_TYPE_HW, 0x05, "u32", u"CAN termination",      ""),

    ("kp",               "SW", CONFIG_TYPE_SW, 0x01, "i32", u"Kp",                   ""),
    ("ki",               "SW", CONFIG_TYPE_SW, 0x02, "i32", u"Ki",                   ""),
    ("kd",               "SW", CONFIG_TYPE_SW, 0x03, "i32", u"Kd",                   ""),
    ("pos_jump_rad",     "SW", CONFIG_TYPE_SW, 0x04, "f32", u"Position jump limit",  "deg"),
    ("pos_error_rad",    "SW", CONFIG_TYPE_SW, 0x05, "f32", u"Position error limit", "deg"),
    ("interp_ms",        "SW", CONFIG_TYPE_SW, 0x06, "u32", u"Interpolation time",   "ms"),
    ("torque_target",    "SW", CONFIG_TYPE_SW, 0x07, "i32", u"Torque ramp target",   ""),
    ("torque_ramp_ms",   "SW", CONFIG_TYPE_SW, 0x08, "u32", u"Torque ramp duration", "ms"),
]

PARAM_BY_KEY = dict((p[0], p) for p in PARAMETERS)


def normalize_interface(name):
    # Accidental full-width digits such as "can０" -> "can0"
    trans = {
        u"０": u"0", u"１": u"1", u"２": u"2", u"３": u"3", u"４": u"4",
        u"５": u"5", u"６": u"6", u"７": u"7", u"８": u"8", u"９": u"9",
    }
    try:
        if not isinstance(name, unicode):
            name = name.decode("utf-8")
    except NameError:
        pass
    for src, dst in trans.items():
        name = name.replace(src, dst)
    try:
        return name.encode("ascii")
    except Exception:
        return name


def parse_axis_spec(text):
    axes = set()
    for part in text.split(","):
        part = part.strip()
        if not part:
            continue
        if "-" in part:
            a, b = part.split("-", 1)
            start = int(a, 0)
            end = int(b, 0)
            if end < start:
                start, end = end, start
            for axis in range(start, end + 1):
                axes.add(axis)
        else:
            axes.add(int(part, 0))

    axes = sorted(axes)
    for axis in axes:
        if axis < 0 or axis > 0x7F:
            raise ValueError("Axis must be 0..127")
    return axes


def raw4_from_list(values):
    return "".join(chr(v & 0xFF) for v in values)


def unpack_value(values4, value_type):
    raw = raw4_from_list(values4)
    if value_type == "f32":
        return struct.unpack("<f", raw)[0]
    if value_type == "i32":
        return struct.unpack("<i", raw)[0]
    if value_type == "u32":
        return struct.unpack("<I", raw)[0]
    raise ValueError("Unknown value type: %s" % value_type)


def display_value(key, value):
    if value is None:
        return "-"
    if key in ("joint_min_rad", "joint_max_rad", "pos_jump_rad", "pos_error_rad"):
        deg = value * 180.0 / math.pi
        return "%.3f deg  (%.6f rad)" % (deg, value)
    if key == "gear_ratio":
        return "%.4g" % value
    if key == "motor_direction":
        if value == 1:
            return "+1"
        if value == -1:
            return "-1"
        return str(value)
    if key == "can_termination":
        return "ON (1)" if value else "OFF (0)"
    return str(value)


def pack_value(value, value_type):
    if value_type == "f32":
        raw = struct.pack("<f", float(value))
    elif value_type == "i32":
        raw = struct.pack("<i", int(value))
    elif value_type == "u32":
        ivalue = int(value)
        if ivalue < 0:
            raise ValueError("unsigned value cannot be negative")
        raw = struct.pack("<I", ivalue)
    else:
        raise ValueError("Unknown value type: %s" % value_type)
    return [ord(c) for c in raw]


def edit_value_from_wire(key, value):
    if value is None:
        return ""
    if key in ("joint_min_rad", "joint_max_rad", "pos_jump_rad", "pos_error_rad"):
        return "%.6f" % (value * 180.0 / math.pi)
    if key == "gear_ratio":
        return "%.6g" % value
    return str(value)


def wire_value_from_edit(key, text, value_type):
    text = text.strip()
    if not text:
        raise ValueError("value is empty")
    if key in ("joint_min_rad", "joint_max_rad", "pos_jump_rad", "pos_error_rad"):
        return float(text) * math.pi / 180.0
    if value_type == "f32":
        return float(text)
    if "." in text or "e" in text.lower():
        raise ValueError("integer value required")
    return int(text, 0)


def command_exists(name):
    devnull = open(os.devnull, "w")
    try:
        return subprocess.call(
            ["which", name],
            stdout=devnull,
            stderr=devnull
        ) == 0
    finally:
        devnull.close()


class CanUtilsCAN(object):
    """
    PythonのAF_CANを直接使わず、can-utilsを使用する。
    candumpを1プロセス常駐させ、cansendで要求を送信する。
    """

    CLASSIC_RE = re.compile(
        r"^\s*(\S+)\s+([0-9A-Fa-f]{3,8})\s+\[(\d+)\]\s*(.*)$"
    )
    COMPACT_RE = re.compile(
        r"^\s*(?:\([^)]+\)\s+)?(\S+)\s+([0-9A-Fa-f]{3,8})#([0-9A-Fa-f]*)\s*$"
    )

    def __init__(self, interface):
        self.interface = normalize_interface(interface)
        self.proc = None
        self.frames = queue.Queue()
        self.stop_event = threading.Event()
        self.reader = None
        self.lock = threading.Lock()

    def open(self):
        if not command_exists("candump"):
            raise RuntimeError("candump が見つかりません。can-utilsを確認してください。")
        if not command_exists("cansend"):
            raise RuntimeError("cansend が見つかりません。can-utilsを確認してください。")

        self.proc = subprocess.Popen(
            ["candump", self.interface],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            bufsize=1,
            universal_newlines=True
        )

        # candumpが即終了していないか確認
        time.sleep(0.10)
        rc = self.proc.poll()
        if rc is not None:
            err = self.proc.stderr.read()
            raise RuntimeError("candump起動失敗: %s" % err.strip())

        self.reader = threading.Thread(target=self._reader_loop)
        self.reader.daemon = True
        self.reader.start()

    def close(self):
        self.stop_event.set()
        if self.proc is not None:
            try:
                self.proc.terminate()
            except Exception:
                pass
            try:
                self.proc.wait()
            except Exception:
                pass
            self.proc = None

    def _reader_loop(self):
        while not self.stop_event.is_set():
            if self.proc is None:
                return
            line = self.proc.stdout.readline()
            if not line:
                if self.proc.poll() is not None:
                    return
                time.sleep(0.01)
                continue

            parsed = self._parse_line(line)
            if parsed is not None:
                self.frames.put(parsed)

    def _parse_line(self, line):
        # Typical:
        #   can0  18B   [8]  01 02 01 00 F4 01 00 00
        m = self.CLASSIC_RE.match(line)
        if m:
            can_id = int(m.group(2), 16)
            dlc = int(m.group(3))
            rest = m.group(4).strip()
            values = []
            if rest:
                for token in rest.split():
                    if len(token) == 2:
                        try:
                            values.append(int(token, 16))
                        except ValueError:
                            pass
            return (can_id, dlc, values[:dlc])

        # Also accept compact log format:
        #   (timestamp) can0 18B#01020100F4010000
        m = self.COMPACT_RE.match(line)
        if m:
            can_id = int(m.group(2), 16)
            hexdata = m.group(3)
            if len(hexdata) % 2:
                return None
            values = []
            for i in range(0, len(hexdata), 2):
                values.append(int(hexdata[i:i+2], 16))
            return (can_id, len(values), values)

        return None

    def _drain_queue(self):
        while True:
            try:
                self.frames.get_nowait()
            except queue.Empty:
                return

    def send_frame(self, can_id, data):
        if len(data) != 8:
            raise ValueError("CAN payload must be 8 bytes")
        payload = "".join("%02X" % (b & 0xFF) for b in data)
        frame = "%03X#%s" % (can_id, payload)

        p = subprocess.Popen(
            ["cansend", self.interface, frame],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )
        out, err = p.communicate()
        if p.returncode != 0:
            raise RuntimeError("cansend failed: %s" % err.strip())

    def _transaction(self, axis, command, config_type, param_id, value_bytes=None, timeout_sec=0.35):
        request_id = CONFIG_CMD_BASE | axis
        response_id = CONFIG_RESPONSE_BASE | axis
        if value_bytes is None:
            value_bytes = [0, 0, 0, 0]
        request = [command, config_type, param_id, 0] + list(value_bytes)

        with self.lock:
            self._drain_queue()
            self.send_frame(request_id, request)
            deadline = time.time() + timeout_sec
            while True:
                remain = deadline - time.time()
                if remain <= 0:
                    raise RuntimeError("TIMEOUT")
                try:
                    can_id, dlc, data = self.frames.get(timeout=remain)
                except queue.Empty:
                    raise RuntimeError("TIMEOUT")
                if can_id != response_id or dlc < 8 or len(data) < 8:
                    continue
                if data[0] != command or data[1] != config_type or data[2] != param_id:
                    continue
                return data[3], data[4:8]

    def read_parameter(self, axis, config_type, param_id, value_type, timeout_sec=0.25):
        result, value_bytes = self._transaction(
            axis, CONFIG_CMD_READ, config_type, param_id, [0, 0, 0, 0], timeout_sec
        )
        if result != 0:
            raise RuntimeError(RESULT_TEXT.get(result, "RESULT_0x%02X" % result))
        return unpack_value(value_bytes, value_type)

    def write_parameter(self, axis, config_type, param_id, value_type, value, timeout_sec=0.40):
        sent = pack_value(value, value_type)
        result, echo_bytes = self._transaction(
            axis, CONFIG_CMD_WRITE, config_type, param_id, sent, timeout_sec
        )
        if result != 0:
            raise RuntimeError(RESULT_TEXT.get(result, "RESULT_0x%02X" % result))
        return unpack_value(echo_bytes, value_type)

    def save_config(self, axis, config_type, timeout_sec=1.20):
        result, _ = self._transaction(
            axis, CONFIG_CMD_SAVE, config_type, 0x00, [0, 0, 0, 0], timeout_sec
        )
        if result != 0:
            raise RuntimeError(RESULT_TEXT.get(result, "RESULT_0x%02X" % result))
        return True


class LilyConfigViewer(tk.Tk):
    def __init__(self, interface, axes):
        tk.Tk.__init__(self)

        self.interface = normalize_interface(interface)
        self.axes = axes
        self.can = CanUtilsCAN(self.interface)
        self.events = queue.Queue()
        self.stop_event = threading.Event()
        self.worker = None
        self.axis_data = dict((a, {}) for a in axes)
        self.axis_status = dict((a, u"未取得") for a in axes)
        self.selected_param_key = None
        self.hw_reboot_required_axes = set()

        self.title("Lily MCU Config Editor (Python 2)")
        self.geometry("1180x700")
        self.minsize(900, 560)

        self._build_ui()

        try:
            self.can.open()
            self.connection_var.set("%s: 接続済み" % self.interface)
        except Exception as e:
            self.connection_var.set("%s: 接続失敗" % self.interface)
            messagebox.showerror(
                u"CAN接続エラー",
                u"%s を開けませんでした。\n\n%s\n\n"
                u"`candump %s` と `cansend` が使用できるか確認してください。"
                % (self.interface, str(e), self.interface)
            )

        self.protocol("WM_DELETE_WINDOW", self._on_close)
        self.after(50, self._poll_events)

    def _build_ui(self):
        top = ttk.Frame(self, padding=8)
        top.pack(fill=tk.X)

        self.connection_var = tk.StringVar()
        self.connection_var.set("%s: 未接続" % self.interface)
        ttk.Label(top, textvariable=self.connection_var).pack(side=tk.LEFT, padx=(0, 14))

        ttk.Label(top, text="Axis").pack(side=tk.LEFT)

        self.axis_var = tk.StringVar()
        self.axis_var.set(str(self.axes[0]))
        self.axis_combo = ttk.Combobox(
            top,
            width=6,
            textvariable=self.axis_var,
            values=[str(a) for a in self.axes],
            state="readonly"
        )
        self.axis_combo.pack(side=tk.LEFT, padx=(4, 10))
        self.axis_combo.bind("<<ComboboxSelected>>", self._on_axis_selected)

        self.btn_selected = ttk.Button(
            top, text=u"選択軸を更新", command=self.refresh_selected
        )
        self.btn_selected.pack(side=tk.LEFT, padx=4)

        self.btn_all = ttk.Button(
            top, text=u"全軸を更新", command=self.refresh_all
        )
        self.btn_all.pack(side=tk.LEFT, padx=4)

        self.btn_stop = ttk.Button(
            top, text=u"停止", command=self.stop_refresh, state=tk.DISABLED
        )
        self.btn_stop.pack(side=tk.LEFT, padx=4)

        self.progress = ttk.Progressbar(
            top, orient=tk.HORIZONTAL, mode="determinate", length=180
        )
        self.progress.pack(side=tk.RIGHT, padx=(10, 0))

        self.status_var = tk.StringVar()
        self.status_var.set(u"READは常時可。WRITE/SAVEはMCU側でaliment_standby時のみ許可されます。")
        ttk.Label(
            self, textvariable=self.status_var, padding=(10, 0, 10, 6)
        ).pack(fill=tk.X)

        body = ttk.Panedwindow(self, orient=tk.HORIZONTAL)
        body.pack(fill=tk.BOTH, expand=True, padx=8, pady=(0, 8))

        left = ttk.Frame(body)
        body.add(left, weight=3)
        ttk.Label(left, text=u"軸一覧").pack(anchor=tk.W, pady=(0, 4))

        cols = ("axis", "status", "gear", "dir", "min", "max", "kp", "ki", "kd")
        self.summary = ttk.Treeview(
            left, columns=cols, show="headings", height=20
        )

        headers = {
            "axis": "Axis", "status": "Status", "gear": "Gear", "dir": "Dir",
            "min": "Min [deg]", "max": "Max [deg]",
            "kp": "Kp", "ki": "Ki", "kd": "Kd"
        }
        widths = {
            "axis": 55, "status": 95, "gear": 70, "dir": 55,
            "min": 85, "max": 85, "kp": 65, "ki": 65, "kd": 65
        }

        for c in cols:
            self.summary.heading(c, text=headers[c])
            self.summary.column(c, width=widths[c], minwidth=45, anchor=tk.CENTER)

        sy = ttk.Scrollbar(left, orient=tk.VERTICAL, command=self.summary.yview)
        sx = ttk.Scrollbar(left, orient=tk.HORIZONTAL, command=self.summary.xview)
        self.summary.configure(yscrollcommand=sy.set, xscrollcommand=sx.set)

        self.summary.pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        sx.pack(side=tk.BOTTOM, fill=tk.X)
        sy.pack(side=tk.RIGHT, fill=tk.Y)

        self.summary.bind("<<TreeviewSelect>>", self._on_summary_select)

        for axis in self.axes:
            self.summary.insert(
                "", tk.END, iid=str(axis),
                values=(axis, u"未取得", "-", "-", "-", "-", "-", "-", "-")
            )

        right = ttk.Frame(body)
        body.add(right, weight=2)

        self.detail_title_var = tk.StringVar()
        self.detail_title_var.set("Axis %s Detail" % self.axis_var.get())
        ttk.Label(right, textvariable=self.detail_title_var).pack(
            anchor=tk.W, pady=(0, 4)
        )

        dcols = ("section", "parameter", "value", "unit")
        self.detail = ttk.Treeview(
            right, columns=dcols, show="headings", height=20
        )
        self.detail.heading("section", text="Type")
        self.detail.heading("parameter", text="Parameter")
        self.detail.heading("value", text="Value")
        self.detail.heading("unit", text="Unit")
        self.detail.column("section", width=55, anchor=tk.CENTER)
        self.detail.column("parameter", width=175, anchor=tk.W)
        self.detail.column("value", width=220, anchor=tk.W)
        self.detail.column("unit", width=60, anchor=tk.CENTER)

        dy = ttk.Scrollbar(right, orient=tk.VERTICAL, command=self.detail.yview)
        self.detail.configure(yscrollcommand=dy.set)
        self.detail.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        dy.pack(side=tk.RIGHT, fill=tk.Y)
        self.detail.bind("<<TreeviewSelect>>", self._on_parameter_selected)

        editor = ttk.LabelFrame(right, text=u"選択パラメータ編集", padding=8)
        editor.pack(fill=tk.X, pady=(8, 0))
        self.edit_param_var = tk.StringVar()
        self.edit_param_var.set(u"パラメータを選択してください")
        ttk.Label(editor, textvariable=self.edit_param_var).grid(row=0, column=0, columnspan=3, sticky=tk.W)
        ttk.Label(editor, text=u"書込値").grid(row=1, column=0, sticky=tk.W, pady=(6, 0))
        self.edit_value_var = tk.StringVar()
        self.edit_entry = ttk.Entry(editor, textvariable=self.edit_value_var, width=22)
        self.edit_entry.grid(row=1, column=1, sticky=tk.EW, padx=6, pady=(6, 0))
        self.edit_unit_var = tk.StringVar()
        ttk.Label(editor, textvariable=self.edit_unit_var).grid(row=1, column=2, sticky=tk.W, pady=(6, 0))
        self.btn_write = ttk.Button(editor, text=u"WRITE（RAM反映）", command=self.write_selected, state=tk.DISABLED)
        self.btn_write.grid(row=2, column=0, columnspan=3, sticky=tk.EW, pady=(8, 0))
        self.echo_var = tk.StringVar()
        self.echo_var.set(u"Echo: -")
        ttk.Label(editor, textvariable=self.echo_var).grid(row=3, column=0, columnspan=3, sticky=tk.W, pady=(5, 0))
        editor.columnconfigure(1, weight=1)

        save_frame = ttk.LabelFrame(right, text=u"不揮発保存", padding=8)
        save_frame.pack(fill=tk.X, pady=(8, 0))
        self.btn_save_hw = ttk.Button(save_frame, text=u"HardwareConfig SAVE", command=lambda: self.save_selected_config(CONFIG_TYPE_HW))
        self.btn_save_hw.pack(fill=tk.X, pady=2)
        self.btn_save_sw = ttk.Button(save_frame, text=u"SoftwareConfig SAVE", command=lambda: self.save_selected_config(CONFIG_TYPE_SW))
        self.btn_save_sw.pack(fill=tk.X, pady=2)
        self.reboot_var = tk.StringVar()
        self.reboot_var.set(u"HardwareConfig SAVE後は電源再投入が必要です。")
        ttk.Label(save_frame, textvariable=self.reboot_var, wraplength=360).pack(anchor=tk.W, pady=(5, 0))

        self._show_axis_detail(self.axes[0])

    def _set_busy(self, busy):
        self.btn_selected.configure(state=tk.DISABLED if busy else tk.NORMAL)
        self.btn_all.configure(state=tk.DISABLED if busy else tk.NORMAL)
        self.axis_combo.configure(state="disabled" if busy else "readonly")
        self.btn_stop.configure(state=tk.NORMAL if busy else tk.DISABLED)
        self.btn_save_hw.configure(state=tk.DISABLED if busy else tk.NORMAL)
        self.btn_save_sw.configure(state=tk.DISABLED if busy else tk.NORMAL)
        if busy:
            self.btn_write.configure(state=tk.DISABLED)
        else:
            self.btn_write.configure(state=tk.NORMAL if self.selected_param_key else tk.DISABLED)

    def _start_worker(self, axes, discovery_mode):
        return self._start_custom_worker(self._worker_refresh_axes, (axes, discovery_mode))

    def _start_custom_worker(self, target, args):
        if self.worker is not None and self.worker.is_alive():
            return False
        self.stop_event.clear()
        self._set_busy(True)
        self.worker = threading.Thread(target=target, args=args)
        self.worker.daemon = True
        self.worker.start()
        return True

    def refresh_selected(self):
        axis = int(self.axis_var.get())
        self.progress["maximum"] = len(PARAMETERS)
        self.progress["value"] = 0
        self._start_worker([axis], False)

    def refresh_all(self):
        self.progress["maximum"] = max(1, len(self.axes))
        self.progress["value"] = 0
        self._start_worker(list(self.axes), True)

    def stop_refresh(self):
        self.stop_event.set()
        self.status_var.set(u"停止要求を送信しました。")

    def _worker_refresh_axes(self, axes, discovery_mode):
        try:
            for index, axis in enumerate(axes):
                if self.stop_event.is_set():
                    break

                self.events.put(("axis_status", axis, u"読取中"))
                self.events.put(("status", u"Axis %d を読取中..." % axis))
                values = {}

                # 全軸探索では最初にgear_ratioだけ短いtimeoutで存在確認
                if discovery_mode:
                    p = PARAM_BY_KEY["gear_ratio"]
                    try:
                        values[p[0]] = self.can.read_parameter(
                            axis, p[2], p[3], p[4], timeout_sec=0.15
                        )
                    except Exception:
                        self.events.put(("axis_data", axis, values, u"応答なし"))
                        self.events.put(("progress", index + 1))
                        continue

                ok = True
                error_text = ""

                for p in PARAMETERS:
                    if self.stop_event.is_set():
                        ok = False
                        error_text = u"停止"
                        break

                    key, section, cfg_type, param_id, value_type, label, unit = p
                    if key in values:
                        continue

                    try:
                        values[key] = self.can.read_parameter(
                            axis, cfg_type, param_id, value_type, timeout_sec=0.25
                        )
                    except Exception as e:
                        ok = False
                        error_text = str(e)
                        break

                    if not discovery_mode:
                        self.events.put(("progress", len(values)))

                status = "OK" if ok else error_text
                self.events.put(("axis_data", axis, values, status))

                if discovery_mode:
                    self.events.put(("progress", index + 1))

            self.events.put(("done",))
        except Exception as e:
            self.events.put(("fatal", str(e)))

    def _poll_events(self):
        try:
            while True:
                event = self.events.get_nowait()
                kind = event[0]

                if kind == "axis_status":
                    _, axis, status = event
                    self.axis_status[axis] = status
                    self._update_summary_row(axis)

                elif kind == "axis_data":
                    _, axis, values, status = event
                    self.axis_data[axis].update(values)
                    self.axis_status[axis] = status
                    self._update_summary_row(axis)
                    if str(axis) == self.axis_var.get():
                        self._show_axis_detail(axis)

                elif kind == "progress":
                    self.progress["value"] = event[1]

                elif kind == "status":
                    self.status_var.set(event[1])

                elif kind == "write_ok":
                    _, axis, key, echo_value, readback = event
                    self.axis_data[axis][key] = readback
                    self.axis_status[axis] = "OK"
                    self.echo_var.set(u"Echo: %s" % display_value(key, echo_value))
                    self._update_summary_row(axis)
                    if str(axis) == self.axis_var.get():
                        self._show_axis_detail(axis)
                        self.selected_param_key = key
                        self._select_detail_key(key)
                        self.edit_value_var.set(edit_value_from_wire(key, readback))

                elif kind == "save_ok":
                    _, axis, config_type = event
                    if config_type == CONFIG_TYPE_HW:
                        self.hw_reboot_required_axes.add(axis)
                        self.reboot_var.set(u"Axis %d: HardwareConfig SAVE成功。電源再投入が必要です。" % axis)
                        messagebox.showinfo(u"SAVE成功", u"HardwareConfigを保存しました。\n電源再投入が必要です。")
                    else:
                        messagebox.showinfo(u"SAVE成功", u"SoftwareConfigを保存しました。")
                    self._update_summary_row(axis)

                elif kind == "operation_error":
                    _, title, text = event
                    self.status_var.set(u"%s: %s" % (title, text))
                    messagebox.showerror(title, text)
                    self._set_busy(False)

                elif kind == "fatal":
                    self.status_var.set(u"エラー: %s" % event[1])
                    messagebox.showerror(u"通信エラー", event[1])
                    self._set_busy(False)

                elif kind == "done":
                    self._set_busy(False)
                    self.status_var.set(event[1] if len(event) > 1 else u"更新完了。")

        except queue.Empty:
            pass

        self.after(50, self._poll_events)

    def _update_summary_row(self, axis):
        d = self.axis_data.get(axis, {})

        def deg_or_dash(key):
            if key not in d:
                return "-"
            return "%.2f" % (d[key] * 180.0 / math.pi)

        values = (
            axis,
            ((self.axis_status.get(axis, u"未取得") + u" / 要再起動") if (axis in self.hw_reboot_required_axes and self.axis_status.get(axis, u"未取得") == "OK") else self.axis_status.get(axis, u"未取得")),
            ("%.4g" % d["gear_ratio"]) if "gear_ratio" in d else "-",
            str(d["motor_direction"]) if "motor_direction" in d else "-",
            deg_or_dash("joint_min_rad"),
            deg_or_dash("joint_max_rad"),
            str(d["kp"]) if "kp" in d else "-",
            str(d["ki"]) if "ki" in d else "-",
            str(d["kd"]) if "kd" in d else "-",
        )
        self.summary.item(str(axis), values=values)

    def _show_axis_detail(self, axis):
        self.detail_title_var.set("Axis %d Detail" % axis)
        for item in self.detail.get_children():
            self.detail.delete(item)
        data = self.axis_data.get(axis, {})
        for p in PARAMETERS:
            key, section, cfg_type, param_id, value_type, label, unit = p
            value = display_value(key, data[key]) if key in data else "-"
            shown_unit = unit
            if key in ("joint_min_rad", "joint_max_rad", "pos_jump_rad", "pos_error_rad", "can_termination"):
                shown_unit = ""
            self.detail.insert("", tk.END, iid=key, values=(section, label, value, shown_unit))
        if self.selected_param_key and self.detail.exists(self.selected_param_key):
            self._select_detail_key(self.selected_param_key)

    def _select_detail_key(self, key):
        try:
            self.detail.selection_set(key)
            self.detail.see(key)
        except Exception:
            pass

    def _on_parameter_selected(self, _event=None):
        selected = self.detail.selection()
        if not selected:
            return
        key = selected[0]
        if key not in PARAM_BY_KEY:
            return
        self.selected_param_key = key
        p = PARAM_BY_KEY[key]
        self.edit_param_var.set(u"%s / %s" % (p[1], p[5]))
        self.edit_unit_var.set(p[6])
        axis = int(self.axis_var.get())
        if key in self.axis_data.get(axis, {}):
            self.edit_value_var.set(edit_value_from_wire(key, self.axis_data[axis][key]))
        else:
            self.edit_value_var.set("")
        if self.worker is None or not self.worker.is_alive():
            self.btn_write.configure(state=tk.NORMAL)

    def write_selected(self):
        if not self.selected_param_key:
            return
        axis = int(self.axis_var.get())
        p = PARAM_BY_KEY[self.selected_param_key]
        key, section, config_type, param_id, value_type, label, unit = p
        try:
            value = wire_value_from_edit(key, self.edit_value_var.get(), value_type)
            pack_value(value, value_type)
        except Exception as e:
            messagebox.showerror(u"入力値エラー", str(e))
            return

        if config_type == CONFIG_TYPE_HW:
            if not messagebox.askyesno(
                u"HardwareConfig WRITE確認",
                u"Axis %d の %s を %s %s にWRITEします。\n\nHardwareConfigはSAVE後に電源再投入が必要です。\nWRITEしますか？" %
                (axis, label, self.edit_value_var.get(), unit)
            ):
                return

        self.echo_var.set(u"Echo: 送信中...")
        self._start_custom_worker(self._worker_write, (axis, p, value))

    def _worker_write(self, axis, p, value):
        key, section, config_type, param_id, value_type, label, unit = p
        try:
            self.events.put(("status", u"Axis %d %s WRITE中..." % (axis, label)))
            echo_value = self.can.write_parameter(axis, config_type, param_id, value_type, value)
            readback = self.can.read_parameter(axis, config_type, param_id, value_type, timeout_sec=0.30)
            self.events.put(("write_ok", axis, key, echo_value, readback))
            self.events.put(("done", u"WRITE成功。Echo確認 + READ back完了。"))
        except Exception as e:
            self.events.put(("operation_error", u"WRITE失敗", str(e)))

    def save_selected_config(self, config_type):
        axis = int(self.axis_var.get())
        name = u"HardwareConfig" if config_type == CONFIG_TYPE_HW else u"SoftwareConfig"
        extra = u"\n\n保存後は電源再投入が必要です。" if config_type == CONFIG_TYPE_HW else u""
        if not messagebox.askyesno(
            u"SAVE確認",
            u"Axis %d の%sをFlashへ保存します。%s\n\nSAVEしますか？" % (axis, name, extra)
        ):
            return
        self._start_custom_worker(self._worker_save, (axis, config_type))

    def _worker_save(self, axis, config_type):
        name = u"HardwareConfig" if config_type == CONFIG_TYPE_HW else u"SoftwareConfig"
        try:
            self.events.put(("status", u"Axis %d %s SAVE中..." % (axis, name)))
            self.can.save_config(axis, config_type)
            self.events.put(("save_ok", axis, config_type))
            self.events.put(("done", u"%s SAVE成功。" % name))
        except Exception as e:
            self.events.put(("operation_error", u"SAVE失敗", str(e)))

    def _on_axis_selected(self, _event=None):
        axis = int(self.axis_var.get())
        self._show_axis_detail(axis)
        self.summary.selection_set(str(axis))
        self.summary.see(str(axis))
        if axis in self.hw_reboot_required_axes:
            self.reboot_var.set(u"Axis %d: HardwareConfig SAVE済み。電源再投入が必要です。" % axis)
        else:
            self.reboot_var.set(u"HardwareConfig SAVE後は電源再投入が必要です。")
        if self.selected_param_key:
            self._on_parameter_selected()

    def _on_summary_select(self, _event=None):
        selected = self.summary.selection()
        if not selected:
            return
        axis = int(selected[0])
        self.axis_var.set(str(axis))
        self._show_axis_detail(axis)
        if axis in self.hw_reboot_required_axes:
            self.reboot_var.set(u"Axis %d: HardwareConfig SAVE済み。電源再投入が必要です。" % axis)
        else:
            self.reboot_var.set(u"HardwareConfig SAVE後は電源再投入が必要です。")

    def _on_close(self):
        self.stop_event.set()
        self.can.close()
        self.destroy()


def main():
    parser = argparse.ArgumentParser(description="Lily MCU Config Editor (Python 2)")
    parser.add_argument(
        "--interface", default="can0",
        help="CAN interface (default: can0)"
    )
    parser.add_argument(
        "--axes", default="0-23",
        help="Axes: 0-23 / 11 / 0,1,2,11 (default: 0-23)"
    )
    args = parser.parse_args()

    try:
        axes = parse_axis_spec(args.axes)
    except Exception as e:
        print("Invalid --axes: %s" % e, file=sys.stderr)
        return 2

    if not axes:
        print("No axes specified.", file=sys.stderr)
        return 2

    app = LilyConfigViewer(args.interface, axes)
    app.mainloop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
