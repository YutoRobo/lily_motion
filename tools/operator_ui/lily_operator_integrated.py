#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division, print_function

import argparse
import os
import sys

try:
    import Tkinter as tk
    import tkMessageBox as messagebox
except ImportError:
    import tkinter as tk
    from tkinter import messagebox

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


DEFAULT_CAN_INTERFACE = 'socketcan'
DEFAULT_CAN_CHANNEL = 'can0'
DEFAULT_CAN_BITRATE = 500000
STATE_MACHINE_PERIOD_MS = 33


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
    return parser.parse_args(argv)


def any_axis_running(leg_ui):
    return any(leg.state == 'Running' for leg in leg_ui.legs)


def main(argv=None):
    args = parse_args(argv)
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
    leg_ui = OperatorLegControlUI(root)
    motion_panel = MotionPanel(root, leg_ui)
    root.title('Lily Operator UI')

    shutting_down = [False]

    def state_machine_tick():
        if shutting_down[0] or rospy.is_shutdown():
            return
        try:
            sm.execute()
        except Exception as exc:
            rospy.logerr('StateMachine execute failed: %s', exc)
        root.after(STATE_MACHINE_PERIOD_MS, state_machine_tick)

    def shutdown_backend():
        if shutting_down[0]:
            return
        shutting_down[0] = True
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
        if any_axis_running(leg_ui):
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
