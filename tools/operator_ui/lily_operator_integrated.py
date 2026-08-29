#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division, print_function

import argparse
import os
import sys

try:
    import Tkinter as tk
    import ttk
    import tkMessageBox as messagebox
except ImportError:
    import tkinter as tk
    from tkinter import ttk, messagebox

import can
import rospy

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
TOOLS_DIR = os.path.dirname(THIS_DIR)
ROOT = os.path.dirname(TOOLS_DIR)
STATEMACHINE_DIR = os.path.join(TOOLS_DIR, 'can_interface', 'statemachine')

for path in (ROOT, THIS_DIR, STATEMACHINE_DIR):
    if path not in sys.path:
        sys.path.insert(0, path)

from state_machine import StateMachine
from lily_operator_ui import OperatorLegControlUI, MotionPanel
from position_monitor_panel import PositionMonitorPanel


DEFAULT_CAN_INTERFACE = 'socketcan'
DEFAULT_CAN_CHANNEL = 'can0'
DEFAULT_CAN_BITRATE = 500000
STATE_MACHINE_PERIOD_MS = 33


class LegacyPanelHost(tk.Frame):
    """Frame adapter allowing the maintained LegControlUI to live in a tab."""

    def title(self, *unused_args, **unused_kwargs):
        return None


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description='Integrated Lily Operator UI + CAN StateMachine')
    parser.add_argument(
        '--can-interface',
        default=os.environ.get('LILY_CAN_INTERFACE', DEFAULT_CAN_INTERFACE))
    parser.add_argument(
        '--can-channel',
        default=os.environ.get('LILY_CAN_CHANNEL', DEFAULT_CAN_CHANNEL))
    parser.add_argument(
        '--can-bitrate', type=int,
        default=int(os.environ.get('LILY_CAN_BITRATE', str(DEFAULT_CAN_BITRATE))))
    parser.add_argument(
        '--monitor-leg', type=int, default=4,
        help='initial one-based monitor leg 1..8 (default: 4)')
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    if args.monitor_leg < 1 or args.monitor_leg > 8:
        raise SystemExit('--monitor-leg must be in 1..8')

    rospy.init_node('lily_operator', anonymous=False)

    try:
        bus = can.interface.Bus(
            interface=args.can_interface,
            channel=args.can_channel,
            bitrate=args.can_bitrate)
    except Exception as exc:
        rospy.logfatal(
            'CAN bus init failed: interface=%s channel=%s bitrate=%d error=%s',
            args.can_interface, args.can_channel, args.can_bitrate, exc)
        raise SystemExit(
            'CAN bus init failed on %s: %s' % (args.can_channel, exc))

    sm = StateMachine(bus)

    # Keep the same receive architecture as the maintained StateMachine runner.
    can_listener = can.BufferedReader()
    can_listener.on_message_received = sm.can_callback
    notifier = can.Notifier(bus, [can_listener])

    root = tk.Tk()
    root.title('Lily Operator UI')
    try:
        root.geometry('1500x950')
    except Exception:
        pass

    # Persistent status / STOP area remains visible on every tab.
    header = tk.Frame(root, bd=2, relief=tk.GROOVE)
    header.pack(side=tk.TOP, fill=tk.X, padx=6, pady=(6, 2))
    tk.Label(
        header, text='Lily Operator',
        font=('Helvetica', 14, 'bold')).pack(side=tk.LEFT, padx=(8, 18))
    system_status_var = tk.StringVar(value='Starting...')
    tk.Label(
        header, textvariable=system_status_var,
        anchor='w').pack(side=tk.LEFT, fill=tk.X, expand=True)

    notebook = ttk.Notebook(root)
    notebook.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=6, pady=(2, 6))

    control_tab = tk.Frame(notebook)
    motion_tab = tk.Frame(notebook)
    monitor_tab = tk.Frame(notebook)
    notebook.add(control_tab, text='Control')
    notebook.add(motion_tab, text='Motion')
    notebook.add(monitor_tab, text='Monitor')

    # Existing LegControlUI is mounted unchanged inside a Frame adapter.
    control_host = LegacyPanelHost(control_tab)
    control_host.pack(side=tk.TOP, fill=tk.BOTH, expand=True)
    leg_ui = OperatorLegControlUI(control_host)

    # JSONL LOAD/CHECK/SEND is isolated in its own tab.
    motion_panel = MotionPanel(motion_tab, leg_ui)

    # Existing receive-only candump/Matplotlib viewer is embedded in Monitor.
    monitor_panel = PositionMonitorPanel(
        monitor_tab,
        can_interface=args.can_channel,
        default_leg_index=args.monitor_leg - 1)

    tk.Button(
        header,
        text='STOP',
        command=leg_ui.send_stop,
        font=('Helvetica', 12, 'bold'),
        width=10).pack(side=tk.RIGHT, padx=8, pady=3)

    shutting_down = [False]

    def refresh_header_status():
        online = sum(
            1 for leg in leg_ui.legs
            if leg.state != 'Disconnected')
        run_text = 'RUN' if sm.is_run else 'STANDBY'
        can_text = 'OK' if sm.can_interface_ok else 'ERROR'
        system_status_var.set(
            'CAN %s: %s @ %d | axes online %d/24 | %s' % (
                args.can_channel, can_text, args.can_bitrate,
                online, run_text))

    def state_machine_tick():
        if shutting_down[0] or rospy.is_shutdown():
            return
        try:
            sm.execute()
        except Exception as exc:
            rospy.logerr('StateMachine execute failed: %s', exc)
        refresh_header_status()
        root.after(STATE_MACHINE_PERIOD_MS, state_machine_tick)

    def shutdown_backend():
        if shutting_down[0]:
            return
        shutting_down[0] = True
        try:
            monitor_panel.close()
        except Exception:
            pass
        try:
            notifier.stop()
        except Exception:
            pass
        try:
            bus.shutdown()
        except Exception:
            pass
        try:
            rospy.signal_shutdown('Lily Operator UI closed')
        except Exception:
            pass

    def close_handler():
        if not motion_panel.on_close():
            return
        if sm.is_run:
            messagebox.showwarning(
                'RUN is active',
                'RUN is still active. Finish the motion session and use STOP '
                'before closing the integrated Operator UI.')
            return
        shutdown_backend()
        root.destroy()

    root.protocol('WM_DELETE_WINDOW', close_handler)
    root.after(STATE_MACHINE_PERIOD_MS, state_machine_tick)

    rospy.loginfo(
        'Integrated Lily Operator started: CAN interface=%s channel=%s bitrate=%d',
        args.can_interface, args.can_channel, args.can_bitrate)

    try:
        root.mainloop()
    finally:
        shutdown_backend()


if __name__ == '__main__':
    main()
