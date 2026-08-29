#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division, print_function

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


class EmbeddedViewerHost(tk.Frame):
    """Tk Frame adapter for the existing standalone ViewerApp.

    ViewerApp only needs normal Tk container methods plus title/geometry/protocol,
    which are window methods on Tk/Toplevel.  For embedding, these window-only
    methods intentionally become no-ops so the ViewerApp can build all controls
    and plots inside a Notebook tab without owning the application window.
    """

    def title(self, *unused_args, **unused_kwargs):
        return None

    def geometry(self, *unused_args, **unused_kwargs):
        return None

    def protocol(self, *unused_args, **unused_kwargs):
        return None


class PositionMonitorPanel(object):
    """Embedded receive-only MCU position telemetry monitor.

    The underlying reader remains the existing candump-based ViewerApp and never
    sends CAN frames.  Selecting another leg reconstructs only the viewer panel;
    it does not touch the Operator StateMachine or motion publisher.
    """

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
        app = viewer.ViewerApp(host, args, axes)

        self.viewer_host = host
        self.viewer_app = app
        self.target_status_var.set(
            'Leg %d | axes %s | %s' % (
                leg_index + 1,
                ','.join(str(axis) for axis in axes),
                self.can_interface))

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
