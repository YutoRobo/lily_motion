#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Lily MCU position-debug realtime viewer with native Tkinter controls.

Expected CAN telemetry per axis:
  CAN ID   : 0x500 | axis
  DLC      : 8
  byte 0-3 : internal position command [joint rad], float32 little-endian
  byte 4-7 : actual position          [joint rad], float32 little-endian

Operation:
  1. Start this viewer. candump starts in receive-only mode.
  2. Enter Duration [s] in the Tkinter UI.
  3. Press START.
  4. The first selected-axis telemetry frame after START is t=0.
  5. Capture continues for the specified duration, then stops automatically.
  6. The plot stays visible. Press START again for another run.

This program NEVER sends CAN frames.
Python 2.7 / Ubuntu 18.04 compatible.
"""
from __future__ import division, print_function

import argparse
import collections
import csv
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
    import tkMessageBox as messagebox
except ImportError:
    import tkinter as tk
    from tkinter import messagebox

import matplotlib
matplotlib.use('TkAgg')
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
try:
    from matplotlib.backends.backend_tkagg import NavigationToolbar2TkAgg as NavigationToolbar
except ImportError:
    from matplotlib.backends.backend_tkagg import NavigationToolbar2Tk as NavigationToolbar


CANDUMP_RE = re.compile(
    r'^\s*\((?P<timestamp>[0-9]+(?:\.[0-9]+)?)\)\s+'
    r'(?P<interface>\S+)\s+'
    r'(?P<canid>[0-9A-Fa-f]+)\s+'
    r'\[(?P<dlc>\d+)\]\s+'
    r'(?P<data>(?:[0-9A-Fa-f]{2}(?:\s+|$))+)$'
)

CANDUMP_HASH_RE = re.compile(
    r'^\s*\((?P<timestamp>[0-9]+(?:\.[0-9]+)?)\)\s+'
    r'(?P<interface>\S+)\s+'
    r'(?P<canid>[0-9A-Fa-f]+)#(?P<data>[0-9A-Fa-f]*)\s*$'
)

JOINT_NAMES = ('base', 'thigh', 'tibia')


def parse_args(argv=None):
    p = argparse.ArgumentParser(
        description='Realtime plot of 0x500|axis MCU position debug telemetry')
    p.add_argument('--interface', default='can0',
                   help='SocketCAN interface (default: can0)')
    p.add_argument('--leg-index', type=int, default=3,
                   help='zero-based leg index 0..7 (default: 3)')
    p.add_argument('--axes', default='',
                   help='optional explicit comma-separated axes, e.g. 9,10,11')
    p.add_argument('--duration-sec', type=float, default=5.0,
                   help='initial Duration [s] shown in UI (default: 5)')
    p.add_argument('--window-sec', type=float, default=5.0,
                   help='minimum visible x-axis window [s] (default: 5)')
    p.add_argument('--refresh-hz', type=float, default=20.0,
                   help='UI plot refresh rate only (default: 20 Hz)')
    p.add_argument('--expected-telemetry-hz', type=float, default=100.0,
                   help='used for ring-buffer sizing only (default: 100 Hz)')
    p.add_argument('--csv', default='',
                   help='CSV path for first run; later runs get _002, _003, ...')
    p.add_argument('--no-csv', action='store_true',
                   help='disable CSV logging')
    p.add_argument('--error-limit-deg', type=float, default=4.0,
                   help='error reference line in degrees (default: 4)')
    return p.parse_args(argv)


def resolve_axes(args):
    if args.axes:
        axes = [int(x.strip()) for x in args.axes.split(',') if x.strip()]
    else:
        if not 0 <= args.leg_index <= 7:
            raise ValueError('--leg-index must be in 0..7')
        first = args.leg_index * 3
        axes = [first, first + 1, first + 2]
    if not axes:
        raise ValueError('no axes selected')
    for axis in axes:
        if not 0 <= axis <= 23:
            raise ValueError('axis must be in 0..23')
    return axes


def axis_label(axis):
    return 'axis%d %s' % (axis, JOINT_NAMES[axis % 3])


def is_finite(v):
    return not (math.isnan(v) or math.isinf(v))


def decode_float_le(b4):
    return struct.unpack('<f', struct.pack('4B', *b4))[0]


def parse_candump_line(line):
    line = line.rstrip('\r\n')
    m = CANDUMP_RE.match(line)
    if m:
        data = [int(x, 16) for x in m.group('data').split()]
        return (float(m.group('timestamp')),
                int(m.group('canid'), 16),
                data)

    m = CANDUMP_HASH_RE.match(line)
    if m:
        hx = m.group('data')
        if len(hx) % 2:
            return None
        data = [int(hx[i:i + 2], 16) for i in range(0, len(hx), 2)]
        return (float(m.group('timestamp')),
                int(m.group('canid'), 16),
                data)
    return None


class CandumpReader(threading.Thread):
    def __init__(self, interface, out_queue, stop_event):
        threading.Thread.__init__(self)
        self.daemon = True
        self.interface = interface
        self.out_queue = out_queue
        self.stop_event = stop_event
        self.proc = None
        self.error = None

    def run(self):
        cmd = ['candump', '-L', self.interface]
        try:
            self.proc = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                universal_newlines=True,
                bufsize=1)
        except Exception as exc:
            self.error = 'failed to start candump: %s' % exc
            return

        while not self.stop_event.is_set():
            line = self.proc.stdout.readline()
            if not line:
                if self.proc.poll() is not None:
                    err = ''
                    try:
                        err = self.proc.stderr.read().strip()
                    except Exception:
                        pass
                    self.error = 'candump exited: %s' % err
                    break
                time.sleep(0.01)
                continue
            self.out_queue.put(line)

    def terminate(self):
        if self.proc is not None and self.proc.poll() is None:
            try:
                self.proc.terminate()
            except Exception:
                pass


class ViewerApp(object):
    def __init__(self, root, args, axes):
        self.root = root
        self.args = args
        self.axes_ids = axes
        self.canid_to_axis = dict((0x500 | a, a) for a in axes)

        self.line_queue = queue.Queue()
        self.stop_event = threading.Event()
        self.reader = CandumpReader(args.interface, self.line_queue, self.stop_event)

        maxlen = max(1000, int(max(args.window_sec, args.duration_sec, 1.0)
                               * max(args.expected_telemetry_hz, 1.0) * 2.0))
        self.samples = dict((a, collections.deque(maxlen=maxlen)) for a in axes)
        self.sample_count = dict((a, 0) for a in axes)
        self.max_abs_error = dict((a, 0.0) for a in axes)

        self.measurement_active = False
        self.measurement_complete = False
        self.measurement_t0 = None
        self.measurement_duration = float(args.duration_sec)
        self.measurement_index = 0
        self.last_rx_wall = None
        self.status_override = ''

        self.csv_file = None
        self.csv_writer = None
        self.csv_path = None

        self._build_ui()
        self.reader.start()

        refresh_ms = int(max(20.0, 1000.0 / max(args.refresh_hz, 1.0)))
        self.refresh_ms = refresh_ms
        self.root.after(self.refresh_ms, self._periodic_update)
        self.root.protocol('WM_DELETE_WINDOW', self._on_close)

    def _build_ui(self):
        self.root.title('Lily MCU Position Debug Viewer')
        try:
            self.root.geometry('1200x900')
        except Exception:
            pass

        # Native Tkinter control area: always visible above the graph.
        control = tk.Frame(self.root, bd=2, relief=tk.GROOVE)
        control.pack(side=tk.TOP, fill=tk.X, padx=6, pady=6)

        tk.Label(control, text='Interface:').pack(side=tk.LEFT, padx=(6, 2))
        tk.Label(control, text=self.args.interface).pack(side=tk.LEFT, padx=(0, 12))

        tk.Label(control, text='Axes:').pack(side=tk.LEFT, padx=(2, 2))
        tk.Label(control, text=','.join(str(a) for a in self.axes_ids)).pack(
            side=tk.LEFT, padx=(0, 18))

        tk.Label(control, text='Duration [s]:').pack(side=tk.LEFT, padx=(2, 4))
        self.duration_var = tk.StringVar()
        self.duration_var.set('%.3f' % self.measurement_duration)
        self.duration_entry = tk.Entry(control, width=10,
                                       textvariable=self.duration_var)
        self.duration_entry.pack(side=tk.LEFT, padx=(0, 12))

        self.start_button = tk.Button(
            control, text='START', width=12, command=self._start_measurement)
        self.start_button.pack(side=tk.LEFT, padx=4)

        self.stop_button = tk.Button(
            control, text='STOP', width=12, command=self._stop_measurement)
        self.stop_button.pack(side=tk.LEFT, padx=4)

        self.clear_button = tk.Button(
            control, text='CLEAR', width=10, command=self._clear_display)
        self.clear_button.pack(side=tk.LEFT, padx=4)

        self.status_var = tk.StringVar()
        self.status_var.set('IDLE - set Duration and press START')
        status_frame = tk.Frame(self.root)
        status_frame.pack(side=tk.TOP, fill=tk.X, padx=8, pady=(0, 4))
        tk.Label(status_frame, textvariable=self.status_var,
                 anchor='w', justify=tk.LEFT).pack(side=tk.LEFT, fill=tk.X, expand=True)

        # Matplotlib graph embedded in Tkinter.
        n_pos = len(self.axes_ids)
        self.fig = Figure(figsize=(11, 8), dpi=100)
        self.pos_axes = []
        self.pos_lines = {}

        for i, axis in enumerate(self.axes_ids):
            ax = self.fig.add_subplot(n_pos + 1, 1, i + 1)
            cmd_line, = ax.plot([], [], label='command')
            act_line, = ax.plot([], [], label='actual')
            ax.set_ylabel('rad')
            ax.set_title(axis_label(axis))
            ax.grid(True)
            ax.legend(loc='upper right')
            self.pos_axes.append(ax)
            self.pos_lines[axis] = (cmd_line, act_line)

        self.err_ax = self.fig.add_subplot(n_pos + 1, 1, n_pos + 1)
        self.err_lines = {}
        for axis in self.axes_ids:
            line, = self.err_ax.plot([], [], label=axis_label(axis))
            self.err_lines[axis] = line
        limit_rad = math.radians(self.args.error_limit_deg)
        self.err_ax.axhline(limit_rad, linestyle='--', linewidth=1.0,
                            label='+%.1f deg limit' % self.args.error_limit_deg)
        self.err_ax.axhline(-limit_rad, linestyle='--', linewidth=1.0,
                            label='-%.1f deg limit' % self.args.error_limit_deg)
        self.err_ax.set_xlabel('time [s]')
        self.err_ax.set_ylabel('cmd - actual [rad]')
        self.err_ax.set_title('tracking error')
        self.err_ax.grid(True)
        self.err_ax.legend(loc='upper right', ncol=2)

        self.fig.tight_layout()

        graph_frame = tk.Frame(self.root)
        graph_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        self.canvas = FigureCanvasTkAgg(self.fig, master=graph_frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        try:
            toolbar = NavigationToolbar(self.canvas, graph_frame)
            toolbar.update()
            toolbar.pack(side=tk.BOTTOM, fill=tk.X)
        except Exception:
            pass

    def _validate_duration(self):
        try:
            value = float(self.duration_var.get().strip())
        except Exception:
            messagebox.showerror('Invalid Duration',
                                 'Duration [s] must be numeric.')
            return None
        if not is_finite(value) or value <= 0.0:
            messagebox.showerror('Invalid Duration',
                                 'Duration [s] must be greater than 0.')
            return None
        return value

    def _start_measurement(self):
        duration = self._validate_duration()
        if duration is None:
            return

        if self.measurement_active:
            self._finish_measurement('RESTARTED')

        self.measurement_index += 1
        self.measurement_duration = duration
        self._clear_measurement_data()
        self._clear_input_queue()
        self._close_csv()
        self._open_csv()

        self.measurement_active = True
        self.measurement_complete = False
        self.measurement_t0 = None
        self.status_override = 'ARMED - waiting for first selected-axis telemetry frame'
        self.status_var.set(self.status_override)

    def _stop_measurement(self):
        if self.measurement_active:
            self._finish_measurement('STOPPED by user')
        else:
            self.status_override = 'IDLE - no measurement is running'
            self.status_var.set(self.status_override)

    def _clear_display(self):
        if self.measurement_active:
            self._finish_measurement('STOPPED by CLEAR')
        self._clear_measurement_data()
        self.status_override = 'CLEARED - set Duration and press START'
        self.status_var.set(self.status_override)
        self._refresh_plot()

    def _finish_measurement(self, reason):
        self.measurement_active = False
        self.measurement_complete = (reason == 'COMPLETE')
        self._close_csv()
        if reason == 'COMPLETE':
            self.status_override = 'COMPLETE - measurement finished automatically'
        else:
            self.status_override = reason
        self.status_var.set(self.status_override)

    def _clear_measurement_data(self):
        for axis in self.axes_ids:
            self.samples[axis].clear()
            self.sample_count[axis] = 0
            self.max_abs_error[axis] = 0.0
        self.measurement_t0 = None
        self.measurement_complete = False

    def _clear_input_queue(self):
        while True:
            try:
                self.line_queue.get_nowait()
            except queue.Empty:
                break

    def _make_csv_path(self):
        if self.args.no_csv:
            return None
        if self.args.csv:
            base, ext = os.path.splitext(self.args.csv)
            if not ext:
                ext = '.csv'
            if self.measurement_index <= 1:
                return base + ext
            return '%s_%03d%s' % (base, self.measurement_index, ext)
        stamp = time.strftime('%Y%m%d_%H%M%S')
        return 'position_debug_%s_%03d.csv' % (stamp, self.measurement_index)

    def _open_csv(self):
        self.csv_path = self._make_csv_path()
        if self.csv_path is None:
            return
        mode = 'wb' if sys.version_info[0] < 3 else 'w'
        self.csv_file = open(self.csv_path, mode)
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'timestamp_epoch_sec', 'time_sec', 'axis',
            'command_rad', 'actual_rad', 'error_rad',
            'command_deg', 'actual_deg', 'error_deg'
        ])
        self.csv_file.flush()

    def _close_csv(self):
        if self.csv_file is not None:
            try:
                self.csv_file.flush()
                self.csv_file.close()
            except Exception:
                pass
        self.csv_file = None
        self.csv_writer = None

    def _store_sample(self, axis, ts, command, actual):
        if self.measurement_t0 is None:
            self.measurement_t0 = ts
            self.status_override = ''

        rel_t = ts - self.measurement_t0
        if rel_t < -0.001:
            return

        if rel_t > self.measurement_duration:
            self._finish_measurement('COMPLETE')
            return

        error = command - actual
        self.samples[axis].append((rel_t, command, actual, error))
        self.sample_count[axis] += 1
        if abs(error) > self.max_abs_error[axis]:
            self.max_abs_error[axis] = abs(error)

        if self.csv_writer is not None:
            self.csv_writer.writerow([
                '%.9f' % ts,
                '%.9f' % rel_t,
                axis,
                '%.9f' % command,
                '%.9f' % actual,
                '%.9f' % error,
                '%.6f' % math.degrees(command),
                '%.6f' % math.degrees(actual),
                '%.6f' % math.degrees(error),
            ])

    def _consume_can(self):
        while True:
            try:
                line = self.line_queue.get_nowait()
            except queue.Empty:
                break

            parsed = parse_candump_line(line)
            if parsed is None:
                continue
            ts, canid, data = parsed
            axis = self.canid_to_axis.get(canid)
            if axis is None or len(data) < 8:
                continue

            try:
                command = decode_float_le(data[0:4])
                actual = decode_float_le(data[4:8])
            except Exception:
                continue
            if not is_finite(command) or not is_finite(actual):
                continue

            self.last_rx_wall = time.time()

            # Monitoring is always active, but data are retained only during STARTed measurement.
            if not self.measurement_active:
                continue

            self._store_sample(axis, ts, command, actual)

    def _refresh_plot(self):
        latest_t = 0.0
        for axis in self.axes_ids:
            data = list(self.samples[axis])
            if data:
                latest_t = max(latest_t, data[-1][0])

        x_max = max(self.args.window_sec, self.measurement_duration,
                    latest_t if latest_t > 0.0 else 0.0)
        if x_max <= 0.0:
            x_max = self.args.window_sec

        for idx, axis in enumerate(self.axes_ids):
            data = list(self.samples[axis])
            if data:
                t = [r[0] for r in data]
                cmd = [r[1] for r in data]
                act = [r[2] for r in data]
                self.pos_lines[axis][0].set_data(t, cmd)
                self.pos_lines[axis][1].set_data(t, act)
                self.pos_axes[idx].set_xlim(0.0, x_max)
                self.pos_axes[idx].relim()
                self.pos_axes[idx].autoscale_view(scalex=False, scaley=True)
            else:
                self.pos_lines[axis][0].set_data([], [])
                self.pos_lines[axis][1].set_data([], [])
                self.pos_axes[idx].set_xlim(0.0, x_max)

        for axis in self.axes_ids:
            data = list(self.samples[axis])
            if data:
                t = [r[0] for r in data]
                err = [r[3] for r in data]
                self.err_lines[axis].set_data(t, err)
            else:
                self.err_lines[axis].set_data([], [])

        self.err_ax.set_xlim(0.0, x_max)
        self.err_ax.relim()
        self.err_ax.autoscale_view(scalex=False, scaley=True)
        self.canvas.draw_idle()

    def _update_status(self):
        if self.reader.error:
            self.status_var.set('ERROR - ' + self.reader.error)
            return

        if self.measurement_active:
            if self.measurement_t0 is None:
                status = 'ARMED - waiting for first selected-axis telemetry frame'
            else:
                latest = 0.0
                for axis in self.axes_ids:
                    if self.samples[axis]:
                        latest = max(latest, self.samples[axis][-1][0])
                status = 'MEASURING %.3f / %.3f s' % (
                    latest, self.measurement_duration)
            stats = []
            for axis in self.axes_ids:
                stats.append('axis%d n=%d max|e|=%.4fdeg' % (
                    axis,
                    self.sample_count[axis],
                    math.degrees(self.max_abs_error[axis])))
            self.status_var.set(status + '    ' + ' | '.join(stats))
            return

        if self.status_override:
            detail = self.status_override
        else:
            detail = 'IDLE - set Duration and press START'

        if self.csv_path:
            detail += '    CSV: %s' % self.csv_path
        self.status_var.set(detail)

    def _periodic_update(self):
        if self.stop_event.is_set():
            return
        try:
            self._consume_can()
            self._refresh_plot()
            self._update_status()
            if self.csv_file is not None:
                self.csv_file.flush()
        except Exception as exc:
            self.status_var.set('ERROR in UI update: %s' % exc)
        self.root.after(self.refresh_ms, self._periodic_update)

    def _on_close(self):
        self.stop_event.set()
        self._close_csv()
        self.reader.terminate()
        try:
            self.root.destroy()
        except Exception:
            pass


def main(argv=None):
    args = parse_args(argv)
    try:
        axes = resolve_axes(args)
    except Exception as exc:
        print('ERROR: %s' % exc, file=sys.stderr)
        return 2

    if args.duration_sec <= 0.0 or not is_finite(args.duration_sec):
        print('ERROR: --duration-sec must be > 0', file=sys.stderr)
        return 2
    if args.refresh_hz <= 0.0:
        print('ERROR: --refresh-hz must be > 0', file=sys.stderr)
        return 2

    print('Lily MCU Position Debug Viewer')
    print(' interface : %s' % args.interface)
    print(' axes      : %s' % ','.join(str(a) for a in axes))
    print(' CAN IDs   : %s' % ', '.join('0x%03X' % (0x500 | a) for a in axes))
    print(' receive-only: yes')
    print(' UI: enter Duration [s] and press START')

    root = tk.Tk()
    app = ViewerApp(root, args, axes)
    root.mainloop()
    return 0


if __name__ == '__main__':
    sys.exit(main())
