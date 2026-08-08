# -*- coding: utf-8 -*-
from __future__ import division

import imp
import inspect
import os
import Queue
import sys
import types
import unittest


ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
UI_PATH = os.path.join(ROOT, "tools", "can_interface", "initUI", "ui.py")


class FakeRospy(types.ModuleType):
    def __init__(self):
        types.ModuleType.__init__(self, "rospy")
        self.warnings = []

    def logwarn(self, fmt, *args):
        self.warnings.append(fmt % args if args else fmt)


class TkAccessForbidden(object):
    def after(self, *args, **kwargs):
        raise AssertionError("ROS callback touched Tk root.after")


class Msg(object):
    def __init__(self, data):
        self.data = data


def load_ui_module():
    fake_tk = types.ModuleType("Tkinter")
    fake_rospy = FakeRospy()
    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")
    std_msgs_msg.String = type("String", (object,), {})

    replacements = {
        "Tkinter": fake_tk,
        "rospy": fake_rospy,
        "std_msgs": std_msgs,
        "std_msgs.msg": std_msgs_msg,
    }
    previous = dict((name, sys.modules.get(name)) for name in replacements)
    try:
        for name, module in replacements.items():
            sys.modules[name] = module
        loaded = imp.load_source("leg_control_ui_under_test", UI_PATH)
    finally:
        for name, old_module in previous.items():
            if old_module is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = old_module

    loaded.rospy = fake_rospy
    return loaded


class CanUiThreadHandoffTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.module = load_ui_module()

    def make_ui_without_tk_init(self):
        ui = self.module.LegControlUI.__new__(self.module.LegControlUI)
        ui.root = TkAccessForbidden()
        ui.ui_event_queue = Queue.Queue()
        return ui

    def test_ros_callbacks_only_enqueue_plain_events(self):
        ui = self.make_ui_without_tk_init()

        ui.status_callback(Msg("10,Connected"))
        ui.use_status_callback(Msg("10,1"))
        ui.motion_check_status_callback(Msg("active:axis10"))
        ui.diagnostic_status_callback(Msg("10|Diagnostic RUN sent"))
        ui.diagnostic_targets_callback(Msg("10|axis10;11|axis11"))

        events = []
        while not ui.ui_event_queue.empty():
            events.append(ui.ui_event_queue.get_nowait())

        self.assertEqual(
            ["status", "use_status", "motion_check_status",
             "diagnostic_status", "diagnostic_targets"],
            [event[0] for event in events])
        self.assertEqual(("status", 10, "Connected"), events[0])
        self.assertEqual(("use_status", 10, True), events[1])
        self.assertEqual(("diagnostic_status", 10, "Diagnostic RUN sent"),
                         events[3])
        self.assertEqual([10, 11], events[4][1])
        self.assertEqual({10: "axis10", 11: "axis11"}, events[4][2])

    def test_drain_dispatches_on_caller_thread_and_is_bounded(self):
        ui = self.make_ui_without_tk_init()
        applied = []

        ui._apply_status_update = lambda axis, state: applied.append(
            ("status", axis, state))
        ui._apply_use_status_update = lambda axis, active: applied.append(
            ("use_status", axis, active))
        ui._apply_motion_check_status = lambda status: applied.append(
            ("motion_check_status", status))
        ui._apply_diagnostic_status = lambda axis, status: applied.append(
            ("diagnostic_status", axis, status))
        ui._apply_diagnostic_targets = lambda targets, labels: applied.append(
            ("diagnostic_targets", targets, labels))

        ui.ui_event_queue.put(("status", 9, "Connected"))
        ui.ui_event_queue.put(("use_status", 9, True))
        ui.ui_event_queue.put(("motion_check_status", "idle"))
        ui.ui_event_queue.put(("diagnostic_status", 9, "ready"))
        ui.ui_event_queue.put(("diagnostic_targets", [9], {9: "axis9"}))

        self.assertEqual(3, ui._drain_ui_events(max_events=3))
        self.assertEqual(2, ui.ui_event_queue.qsize())
        self.assertEqual(
            [("status", 9, "Connected"),
             ("use_status", 9, True),
             ("motion_check_status", "idle")],
            applied)

        self.assertEqual(2, ui._drain_ui_events(max_events=3))
        self.assertEqual(0, ui.ui_event_queue.qsize())
        self.assertEqual("diagnostic_status", applied[3][0])
        self.assertEqual("diagnostic_targets", applied[4][0])

    def test_queue_is_created_before_ros_subscribers(self):
        with open(UI_PATH, "r") as handle:
            source = handle.read()
        queue_pos = source.index("self.ui_event_queue = Queue.Queue()")
        subscriber_pos = source.index("rospy.Subscriber(")
        self.assertLess(queue_pos, subscriber_pos)

    def test_ros_callback_methods_do_not_call_tk_after(self):
        callback_names = (
            "status_callback",
            "use_status_callback",
            "motion_check_status_callback",
            "diagnostic_status_callback",
            "diagnostic_targets_callback",
        )
        for name in callback_names:
            source = inspect.getsource(getattr(self.module.LegControlUI, name))
            self.assertNotIn("root.after", source)
            self.assertNotIn(".config(", source)
            self.assertNotIn(".set(", source)

    def test_invalid_ros_messages_do_not_enqueue(self):
        ui = self.make_ui_without_tk_init()
        ui.status_callback(Msg("bad"))
        ui.use_status_callback(Msg("10,not-an-int"))
        ui.diagnostic_status_callback(Msg("bad"))
        self.assertTrue(ui.ui_event_queue.empty())


if __name__ == "__main__":
    unittest.main()
