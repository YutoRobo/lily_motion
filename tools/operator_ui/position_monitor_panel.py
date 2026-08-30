#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division, print_function

import argparse
import collections
import csv
import math
import os
import sys
import time

try:
    import Queue as queue
except ImportError:
    import queue

try:
    import Tkinter as tk
    import tkFileDialog as filedialog
    import tkMessageBox as messagebox
except ImportError:
    import tkinter as tk
    from tkinter import filedialog, messagebox

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
TOOLS_DIR = os.path.dirname(THIS_DIR)
DIAGNOSTICS_DIR = os.path.join(TOOLS_DIR, 'diagnostics')
if DIAGNOSTICS_DIR not in sys.path:
    sys.path.insert(0, DIAGNOSTICS_DIR)

import realtime_position_debug_viewer_ui as viewer


DEFAULT_HISTORY_POINTS_PER_AXIS = 100000
MAX_PLOT_POINTS_PER_TRACE = 10000
MAX_CAN_LINES_PER_TICK = 1200
IDLE_DRAIN_LINES_PER_TICK = 5000
ACTIVE_PLOT_REFRESH_SEC = 0.50
ACTIVE_STATUS_REFRESH_SEC = 0.25
IDLE_STATUS_REFRESH_SEC = 0.50
CSV_FLUSH_SEC = 1.0
CSV_REQUIRED_COLUMNS = (
    'time_sec', 'axis', 'command_rad', 'actual_rad')


def _open_csv_for_read(path):
    if sys.version_info[0] < 3:
        return open(path, 'rb')
    return open(path, 'r', newline='')


def read_monitor_csv(path):
    """Read a Monitor CSV into plot-ready per-axis samples.

    The loader intentionally uses the four canonical wire-independent columns
    written by the Monitor. error/degree columns are recomputed from command and
    actual so copied or post-processed CSVs cannot introduce inconsistent plots.
    """
    if not path or not os.path.isfile(path):
        raise ValueError('CSV file does not exist: %s' % path)

    samples = {}
    total_rows = 0
    max_time = 0.0

    csv_file = _open_csv_for_read(path)
    try:
        reader = csv.DictReader(csv_file)
        fieldnames = reader.fieldnames or []
        missing = [name for name in CSV_REQUIRED_COLUMNS
                   if name not in fieldnames]
        if missing:
            raise ValueError(
                'Unsupported Monitor CSV. Missing column(s): %s' %
                ', '.join(missing))

        for line_number, row in enumerate(reader, 2):
            try:
                axis = int(row['axis'])
                rel_t = float(row['time_sec'])
                command = float(row['command_rad'])
                actual = float(row['actual_rad'])
            except Exception as exc:
                raise ValueError(
                    'Invalid CSV value at line %d: %s' %
                    (line_number, exc))

            if axis < 0 or axis > 23:
                raise ValueError(
                    'Invalid axis at line %d: %d (expected 0..23)' %
                    (line_number, axis))
            if (not viewer.is_finite(rel_t) or
                    not viewer.is_finite(command) or
                    not viewer.is_finite(actual)):
                raise ValueError(
                    'Non-finite CSV value at line %d' % line_number)
            if rel_t < -0.001:
                raise ValueError(
                    'Negative time_sec at line %d: %.9f' %
                    (line_number, rel_t))

            error = command - actual
            samples.setdefault(axis, []).append(
                (max(0.0, rel_t), command, actual, error))
            total_rows += 1
            max_time = max(max_time, rel_t)
    finally:
        csv_file.close()

    if total_rows <= 0:
        raise ValueError('Monitor CSV contains no data rows.')

    axes = sorted(samples.keys())
    for axis in axes:
        samples[axis].sort(key=lambda item: item[0])

    return {
        'path': os.path.abspath(path),
        'axes': axes,
        'samples': samples,
        'row_count': total_rows,
        'duration_sec': max(0.0, max_time),
    }


class EmbeddedViewerHost(tk.Frame):
    """Tk Frame adapter for the existing standalone ViewerApp."""

    def title(self, *unused_args, **unused_kwargs):
        return None

    def geometry(self, *unused_args, **unused_kwargs):
        return None

    def protocol(self, *unused_args, **unused_kwargs):
        return None


class EmbeddedViewerApp(viewer.ViewerApp):
    """ViewerApp adapted for a shared operator Tk main loop.

    Integration rules:

    1. Retain a long telemetry history even when Duration is changed after
       startup. CSV and retained samples keep every selected-axis sample.
    2. Never monopolize the Tk thread. CAN work per tick is bounded and live
       Matplotlib redraw is throttled.
    3. Once measurement stops/completes, automatic plot refresh is disabled.
       This both removes idle redraw load and allows the Matplotlib toolbar's
       Pan/Zoom interaction to remain stable instead of being overwritten by
       the next autoscale pass.
    4. Keep the Matplotlib toolbar above the plot so Home/Pan/Zoom stay visible
       even when the integrated window height is limited.
    5. Offline CSV display never sends CAN and stops the receive-only candump
       reader after loading, leaving the graph static for analysis.
    """

    def __init__(self, root, args, axes):
        args.refresh_hz = 20.0
        viewer.ViewerApp.__init__(self, root, args, axes)
        self._move_toolbar_above_canvas()
        self.history_points_per_axis = DEFAULT_HISTORY_POINTS_PER_AXIS
        self.max_plot_points_per_trace = MAX_PLOT_POINTS_PER_TRACE
        self._last_plot_wall = 0.0
        self._last_status_wall = 0.0
        self._last_csv_flush_wall = 0.0
        self._plot_dirty = False
        self._final_plot_pending = False
        self.offline_source_path = None
        self._ensure_history_capacity(self.measurement_duration)

    def _move_toolbar_above_canvas(self):
        """Repack the existing Matplotlib toolbar above the canvas."""
        try:
            canvas_widget = self.canvas.get_tk_widget()
            graph_frame = canvas_widget.master
            packed = list(graph_frame.pack_slaves())
            toolbar_widgets = [widget for widget in packed
                               if widget is not canvas_widget]
            if not toolbar_widgets:
                return

            for widget in toolbar_widgets:
                widget.pack_forget()
            canvas_widget.pack_forget()

            for widget in toolbar_widgets:
                widget.pack(side=tk.TOP, fill=tk.X)
            canvas_widget.pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        except Exception:
            pass

    def _required_history_points(self, duration_sec):
        expected_hz = max(float(self.args.expected_telemetry_hz), 1.0)
        duration_sec = max(float(duration_sec), 1.0)
        duration_based = int(math.ceil(duration_sec * expected_hz * 2.0))
        return max(DEFAULT_HISTORY_POINTS_PER_AXIS, duration_based)

    def _ensure_history_capacity(self, duration_sec):
        required = self._required_history_points(duration_sec)
        for axis in self.axes_ids:
            current = self.samples[axis]
            current_maxlen = current.maxlen or 0
            if current_maxlen >= required:
                continue
            self.samples[axis] = collections.deque(current, maxlen=required)
        self.history_points_per_axis = max(
            self.samples[axis].maxlen for axis in self.axes_ids)

    def _start_measurement(self):
        duration = self._validate_duration()
        if duration is None:
            return
        self.offline_source_path = None
        self._ensure_history_capacity(duration)
        self._last_plot_wall = 0.0
        self._last_status_wall = 0.0
        self._last_csv_flush_wall = 0.0
        self._plot_dirty = True
        self._final_plot_pending = False
        result = viewer.ViewerApp._start_measurement(self)
        self._refresh_plot()
        self._plot_dirty = False
        return result

    def _finish_measurement(self, reason):
        result = viewer.ViewerApp._finish_measurement(self, reason)
        self._final_plot_pending = True
        self._plot_dirty = True
        return result

    def _store_sample(self, axis, ts, command, actual):
        before = self.sample_count.get(axis, 0)
        viewer.ViewerApp._store_sample(self, axis, ts, command, actual)
        if self.sample_count.get(axis, 0) != before:
            self._plot_dirty = True

    def _clear_display(self):
        result = viewer.ViewerApp._clear_display(self)
        self._plot_dirty = False
        self._final_plot_pending = False
        if self.offline_source_path:
            self.status_var.set(
                'OFFLINE CSV cleared - use LOAD CSV or APPLY TARGET for live mode')
        return result

    def load_offline_dataset(self, dataset):
        """Replace live history with an already parsed Monitor CSV dataset."""
        if self.measurement_active:
            viewer.ViewerApp._finish_measurement(
                self, 'STOPPED for offline CSV load')
        self._close_csv()
        self._clear_measurement_data()

        duration = max(float(dataset['duration_sec']), 0.001)
        self.measurement_duration = duration
        self.duration_var.set('%.3f' % duration)
        self._ensure_history_capacity(duration)

        for axis in self.axes_ids:
            rows = list(dataset['samples'].get(axis, []))
            required = max(
                self.samples[axis].maxlen or 0,
                len(rows) + 1,
                DEFAULT_HISTORY_POINTS_PER_AXIS)
            if (self.samples[axis].maxlen or 0) < required:
                self.samples[axis] = collections.deque(maxlen=required)
            for row in rows:
                self.samples[axis].append(row)
            self.sample_count[axis] = len(rows)
            self.max_abs_error[axis] = max(
                [abs(row[3]) for row in rows] or [0.0])

        self.measurement_active = False
        self.measurement_complete = True
        self.measurement_t0 = 0.0
        self.offline_source_path = dataset['path']
        self._plot_dirty = False
        self._final_plot_pending = False

        # Offline analysis does not need candump. Stop it immediately so a PC
        # without a CAN interface does no continuing Monitor work after load.
        try:
            self.reader.terminate()
        except Exception:
            pass
        self.stop_event.set()

        try:
            self.start_button.configure(state=tk.DISABLED)
            self.stop_button.configure(state=tk.DISABLED)
            self.duration_entry.configure(state=tk.DISABLED)
        except Exception:
            pass

        self._refresh_plot()
        stats = []
        for axis in self.axes_ids:
            stats.append(
                'axis%d n=%d max|e|=%.4fdeg' % (
                    axis,
                    self.sample_count[axis],
                    math.degrees(self.max_abs_error[axis])))
        self.status_override = (
            'OFFLINE CSV - %s | %.3f s | %s' % (
                os.path.basename(dataset['path']),
                dataset['duration_sec'],
                ' | '.join(stats)))
        self.status_var.set(self.status_override)

    def _decimated(self, data):
        count = len(data)
        limit = int(self.max_plot_points_per_trace)
        if count <= limit or limit < 2:
            return data
        stride = int(math.ceil(float(count) / float(limit)))
        reduced = data[::stride]
        if reduced and reduced[-1] is not data[-1]:
            reduced.append(data[-1])
        return reduced

    def _drain_idle_lines(self):
        """Discard queued raw CAN text cheaply while no measurement is armed."""
        drained = 0
        while drained < IDLE_DRAIN_LINES_PER_TICK:
            try:
                self.line_queue.get_nowait()
            except queue.Empty:
                break
            drained += 1
        return drained

    def _consume_can_bounded(self):
        """Process at most MAX_CAN_LINES_PER_TICK raw candump records."""
        processed = 0
        while processed < MAX_CAN_LINES_PER_TICK:
            try:
                line = self.line_queue.get_nowait()
            except queue.Empty:
                break
            processed += 1

            parsed = viewer.parse_candump_line(line)
            if parsed is None:
                continue
            ts, canid, data = parsed
            axis = self.canid_to_axis.get(canid)
            if axis is None or len(data) < 8:
                continue

            try:
                command = viewer.decode_float_le(data[0:4])
                actual = viewer.decode_float_le(data[4:8])
            except Exception:
                continue
            if not viewer.is_finite(command) or not viewer.is_finite(actual):
                continue

            self.last_rx_wall = time.time()
            self._store_sample(axis, ts, command, actual)

        return processed

    def _refresh_plot(self):
        latest_t = 0.0
        axis_data = {}
        for axis in self.axes_ids:
            full = list(self.samples[axis])
            if full:
                latest_t = max(latest_t, full[-1][0])
            axis_data[axis] = self._decimated(full)

        x_max = max(self.args.window_sec, self.measurement_duration,
                    latest_t if latest_t > 0.0 else 0.0)
        if x_max <= 0.0:
            x_max = self.args.window_sec

        for idx, axis in enumerate(self.axes_ids):
            data = axis_data[axis]
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
            data = axis_data[axis]
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

    def _periodic_update(self):
        """Acquire continuously but redraw only when it provides new information."""
        if self.stop_event.is_set():
            return

        try:
            now = time.time()

            if self.measurement_active:
                self._consume_can_bounded()

                if (self._plot_dirty and
                        now - self._last_plot_wall >= ACTIVE_PLOT_REFRESH_SEC):
                    self._last_plot_wall = now
                    self._refresh_plot()
                    self._plot_dirty = False

                if now - self._last_status_wall >= ACTIVE_STATUS_REFRESH_SEC:
                    self._last_status_wall = now
                    self._update_status()
            else:
                self._drain_idle_lines()

                if self._final_plot_pending:
                    self._refresh_plot()
                    self._plot_dirty = False
                    self._final_plot_pending = False
                    self._last_plot_wall = now

                if now - self._last_status_wall >= IDLE_STATUS_REFRESH_SEC:
                    self._last_status_wall = now
                    self._update_status()

            if (self.csv_file is not None and
                    now - self._last_csv_flush_wall >= CSV_FLUSH_SEC):
                self._last_csv_flush_wall = now
                self.csv_file.flush()
        except Exception as exc:
            self.status_var.set('ERROR in embedded monitor update: %s' % exc)

        self.root.after(self.refresh_ms, self._periodic_update)


class PositionMonitorPanel(object):
    """Embedded receive-only MCU position telemetry monitor."""

    def __init__(self, root, can_interface='can0', default_leg_index=3):
        self.root = root
        self.can_interface = can_interface
        self.viewer_host = None
        self.viewer_app = None

        default_leg_index = int(default_leg_index)
        if default_leg_index < 0 or default_leg_index > 7:
            default_leg_index = 3

        self.leg_var = tk.StringVar(value=str(default_leg_index + 1))
        self.target_status_var = tk.StringVar(value='')

        self._build_shell()
        self._create_viewer(default_leg_index)

    def _build_shell(self):
        selector = tk.Frame(self.root, bd=2, relief=tk.GROOVE)
        selector.pack(side=tk.TOP, fill=tk.X, padx=6, pady=6)

        tk.Label(selector, text='Monitor target leg').pack(
            side=tk.LEFT, padx=(6, 4))
        values = [str(i) for i in range(1, 9)]
        self.leg_menu = tk.OptionMenu(selector, self.leg_var, *values)
        self.leg_menu.pack(side=tk.LEFT, padx=(0, 8))

        self.apply_button = tk.Button(
            selector, text='APPLY TARGET', command=self.apply_target)
        self.apply_button.pack(side=tk.LEFT, padx=(0, 6))

        self.load_csv_button = tk.Button(
            selector, text='LOAD CSV...', command=self.load_csv)
        self.load_csv_button.pack(side=tk.LEFT, padx=(0, 12))

        tk.Label(
            selector,
            text='Live: receive-only CAN. Offline CSV: no CAN output.',
            fg='#006600').pack(side=tk.LEFT, padx=4)

        tk.Label(
            selector, textvariable=self.target_status_var,
            anchor='e').pack(side=tk.RIGHT, padx=8)

        hint = tk.Label(
            self.root,
            text='After STOP/COMPLETE or LOAD CSV: use the toolbar above the plots for Pan / Zoom / Home.',
            anchor='w', fg='#444444')
        hint.pack(side=tk.TOP, fill=tk.X, padx=10, pady=(0, 2))

        self.viewer_container = tk.Frame(self.root)
        self.viewer_container.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

    def _viewer_args(self, leg_index):
        args = viewer.parse_args([])
        args.interface = self.can_interface
        args.leg_index = int(leg_index)
        args.axes = ''
        return args

    def _mount_viewer(self, args, axes):
        self._destroy_viewer()
        host = EmbeddedViewerHost(self.viewer_container)
        host.pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        app = EmbeddedViewerApp(host, args, axes)
        self.viewer_host = host
        self.viewer_app = app
        return app

    def _create_viewer(self, leg_index):
        args = self._viewer_args(leg_index)
        axes = viewer.resolve_axes(args)
        app = self._mount_viewer(args, axes)
        self.target_status_var.set(
            'LIVE | Leg %d | axes %s | %s | history %d pts/axis' % (
                leg_index + 1,
                ','.join(str(axis) for axis in axes),
                self.can_interface,
                app.history_points_per_axis))

    def _create_offline_viewer(self, dataset):
        axes = list(dataset['axes'])
        leg_index = max(0, min(7, axes[0] // 3))
        args = self._viewer_args(leg_index)
        args.axes = ','.join(str(axis) for axis in axes)
        args.no_csv = True
        args.csv = ''
        args.duration_sec = max(dataset['duration_sec'], 0.001)
        args.window_sec = max(args.window_sec, args.duration_sec)
        app = self._mount_viewer(args, axes)
        app.load_offline_dataset(dataset)

        if (len(axes) == 3 and
                axes[0] % 3 == 0 and
                axes == list(range(axes[0], axes[0] + 3))):
            self.leg_var.set(str(axes[0] // 3 + 1))

        self.target_status_var.set(
            'OFFLINE | %s | axes %s | rows %d' % (
                os.path.basename(dataset['path']),
                ','.join(str(axis) for axis in axes),
                dataset['row_count']))

    def load_csv(self):
        if self.viewer_app is not None and self.viewer_app.measurement_active:
            messagebox.showwarning(
                'Measurement active',
                'Stop the current monitor measurement before loading a CSV.')
            return
        path = filedialog.askopenfilename(
            parent=self.root,
            title='Load Lily Monitor CSV',
            filetypes=[('CSV', '*.csv'), ('All files', '*')])
        if path:
            self.open_csv_path(path)

    def open_csv_path(self, path):
        """Load a CSV path without opening a dialog; useful for offline launch."""
        if self.viewer_app is not None and self.viewer_app.measurement_active:
            raise ValueError(
                'Stop the current monitor measurement before loading a CSV.')
        try:
            dataset = read_monitor_csv(path)
            self._create_offline_viewer(dataset)
        except Exception as exc:
            messagebox.showerror(
                'LOAD CSV failed',
                'Could not load Monitor CSV:\n%s\n\n%s' % (path, exc))
            return False
        return True

    def apply_target(self):
        if self.viewer_app is not None and self.viewer_app.measurement_active:
            messagebox.showwarning(
                'Measurement active',
                'Stop the current monitor measurement before changing the target leg.')
            return

        try:
            leg_number = int(self.leg_var.get().strip())
        except Exception:
            messagebox.showerror('Invalid leg', 'Leg must be an integer from 1 to 8.')
            return
        if leg_number < 1 or leg_number > 8:
            messagebox.showerror('Invalid leg', 'Leg must be in 1..8.')
            return

        try:
            self._create_viewer(leg_number - 1)
        except Exception as exc:
            messagebox.showerror(
                'Monitor target failed',
                'Could not create position monitor for Leg %d: %s' % (
                    leg_number, exc))

    def _destroy_viewer(self):
        app = self.viewer_app
        self.viewer_app = None
        if app is not None:
            try:
                app.stop_event.set()
            except Exception:
                pass
            try:
                app._close_csv()
            except Exception:
                pass
            try:
                app.reader.terminate()
            except Exception:
                pass

        host = self.viewer_host
        self.viewer_host = None
        if host is not None:
            try:
                host.destroy()
            except Exception:
                pass

    def close(self):
        self._destroy_viewer()


def main(argv=None):
    parser = argparse.ArgumentParser(
        description='Lily Position Monitor / offline Monitor CSV viewer')
    parser.add_argument(
        '--csv', default='',
        help='optional Monitor CSV to open immediately in offline mode')
    parser.add_argument(
        '--interface', default='can0',
        help='live SocketCAN interface when using START (default: can0)')
    parser.add_argument(
        '--monitor-leg', type=int, default=4,
        help='initial one-based live monitor leg 1..8 (default: 4)')
    args = parser.parse_args(argv)

    if args.monitor_leg < 1 or args.monitor_leg > 8:
        raise SystemExit('--monitor-leg must be in 1..8')

    root = tk.Tk()
    root.title('Lily Position Monitor')
    try:
        root.geometry('1200x900')
    except Exception:
        pass

    panel = PositionMonitorPanel(
        root,
        can_interface=args.interface,
        default_leg_index=args.monitor_leg - 1)

    if args.csv:
        root.after(50, lambda: panel.open_csv_path(args.csv))

    def close_handler():
        panel.close()
        root.destroy()

    root.protocol('WM_DELETE_WINDOW', close_handler)
    root.mainloop()
    return 0


if __name__ == '__main__':
    sys.exit(main())
