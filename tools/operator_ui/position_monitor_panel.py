#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division, print_function

import collections
import math
import os
import sys

try:
    import Tkinter as tk
    import tkMessageBox as messagebox
except ImportError:
    import tkinter as tk
    from tkinter import messagebox

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
TOOLS_DIR = os.path.dirname(THIS_DIR)
DIAGNOSTICS_DIR = os.path.join(TOOLS_DIR, 'diagnostics')
if DIAGNOSTICS_DIR not in sys.path:
    sys.path.insert(0, DIAGNOSTICS_DIR)

import realtime_position_debug_viewer_ui as viewer


DEFAULT_HISTORY_POINTS_PER_AXIS = 100000
MAX_PLOT_POINTS_PER_TRACE = 10000


class EmbeddedViewerHost(tk.Frame):
    """Tk Frame adapter for the existing standalone ViewerApp."""

    def title(self, *unused_args, **unused_kwargs):
        return None

    def geometry(self, *unused_args, **unused_kwargs):
        return None

    def protocol(self, *unused_args, **unused_kwargs):
        return None


class EmbeddedViewerApp(viewer.ViewerApp):
    """ViewerApp with large retained history and bounded plotting cost.

    The standalone ViewerApp sizes its deque once from the initial Duration.
    In the Operator UI the operator often changes Duration after startup, so a
    longer measurement could otherwise silently discard older samples.  This
    adapter keeps a large history per axis and expands it again when a requested
    Duration needs more room.  Plotting is decimated only for rendering; CSV and
    retained samples keep every received measurement sample.
    """

    def __init__(self, root, args, axes):
        viewer.ViewerApp.__init__(self, root, args, axes)
        self.history_points_per_axis = DEFAULT_HISTORY_POINTS_PER_AXIS
        self.max_plot_points_per_trace = MAX_PLOT_POINTS_PER_TRACE
        self._ensure_history_capacity(self.measurement_duration)

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
        self._ensure_history_capacity(duration)
        return viewer.ViewerApp._start_measurement(self)

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
        self.apply_button.pack(side=tk.LEFT, padx=(0, 12))

        tk.Label(
            selector,
            text='Receive-only: CAN telemetry 0x500|axis. No CAN frames are sent.',
            fg='#006600').pack(side=tk.LEFT, padx=4)

        tk.Label(
            selector, textvariable=self.target_status_var,
            anchor='e').pack(side=tk.RIGHT, padx=8)

        self.viewer_container = tk.Frame(self.root)
        self.viewer_container.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

    def _viewer_args(self, leg_index):
        args = viewer.parse_args([])
        args.interface = self.can_interface
        args.leg_index = int(leg_index)
        args.axes = ''
        return args

    def _create_viewer(self, leg_index):
        self._destroy_viewer()

        args = self._viewer_args(leg_index)
        axes = viewer.resolve_axes(args)

        host = EmbeddedViewerHost(self.viewer_container)
        host.pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        app = EmbeddedViewerApp(host, args, axes)

        self.viewer_host = host
        self.viewer_app = app
        self.target_status_var.set(
            'Leg %d | axes %s | %s | history %d pts/axis' % (
                leg_index + 1,
                ','.join(str(axis) for axis in axes),
                self.can_interface,
                app.history_points_per_axis))

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
