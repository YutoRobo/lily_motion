#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function

try:
    import Tkinter as tk
except ImportError:
    import tkinter as tk

import rospy

from lily_operator_ui import MotionPanel


POSITION_LENGTH = 24


class _GazeboLeg(object):
    """Minimal leg state used only to reuse MotionPanel safety/continuity logic."""

    def __init__(self, leg_id):
        self.leg_id = int(leg_id)
        self.use = True
        self.state = 'Running'


class GazeboMotionHost(object):
    """MotionPanel adapter with no CAN StateMachine or hardware controls."""

    def __init__(self):
        self.legs = [_GazeboLeg(i) for i in range(POSITION_LENGTH)]
        self.motion_check_active = False
        self.operator_motion_active = False

    def update_motion_check_ui(self):
        return None

    def update_leg_ui(self, _leg_id):
        return None



def main():
    rospy.init_node('lily_operator_gazebo_ui', anonymous=False)

    root = tk.Tk()
    root.title('Lily Operator - Gazebo')
    try:
        root.geometry('1200x360')
    except Exception:
        pass

    header = tk.Frame(root, bd=2, relief=tk.GROOVE)
    header.pack(side=tk.TOP, fill=tk.X, padx=6, pady=(6, 2))
    tk.Label(
        header,
        text='Lily Operator - Gazebo',
        font=('Helvetica', 14, 'bold')).pack(side=tk.LEFT, padx=(8, 18))
    tk.Label(
        header,
        text='CAN/StateMachine OFF | /cmdForJetson -> Gazebo MCU interpolator -> joint controllers',
        anchor='w').pack(side=tk.LEFT, fill=tk.X, expand=True)

    body = tk.Frame(root)
    body.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=6, pady=(2, 6))

    host = GazeboMotionHost()
    motion_panel = MotionPanel(body, host)

    note = tk.Label(
        body,
        text=('Gazebo model and joint controllers must already be running. '
              'SEND remains protected by the existing /cmdForJetson topology check.'),
        anchor='w', justify=tk.LEFT, fg='#444444')
    note.pack(side=tk.TOP, fill=tk.X, padx=5, pady=(2, 5))

    def close_handler():
        if not motion_panel.on_close():
            return
        try:
            rospy.signal_shutdown('Gazebo Operator UI closed')
        except Exception:
            pass
        root.destroy()

    root.protocol('WM_DELETE_WINDOW', close_handler)
    root.mainloop()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
