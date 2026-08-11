# -*- coding: utf-8 -*-
from __future__ import print_function

import imp
import os
import sys
import types
import unittest


class FakePublisher(object):
    def __init__(self, *args, **kwargs):
        self.messages = []

    def publish(self, value):
        self.messages.append(value)


class FakeRospy(types.ModuleType):
    def __init__(self):
        types.ModuleType.__init__(self, "rospy")
        self.Publisher = FakePublisher
        self.Subscriber = lambda *args, **kwargs: None

    def loginfo(self, *args):
        pass

    def logwarn(self, *args):
        pass

    def logerr(self, *args):
        pass


class FakeCan(types.ModuleType):
    class Message(object):
        def __init__(self, arbitration_id, data, is_extended_id=False):
            self.arbitration_id = arbitration_id
            self.data = list(data)
            self.is_extended_id = is_extended_id


class FakeBus(object):
    def __init__(self):
        self.sent = []

    def send(self, msg):
        self.sent.append(msg)


def load_state_machine():
    fake_rospy = FakeRospy()
    fake_can = FakeCan("can")
    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")
    std_msgs_msg.String = type("String", (object,), {})
    sensor_msgs = types.ModuleType("sensor_msgs")
    sensor_msgs_msg = types.ModuleType("sensor_msgs.msg")
    sensor_msgs_msg.JointState = type("JointState", (object,), {})
    modules = {
        "rospy": fake_rospy,
        "can": fake_can,
        "std_msgs": std_msgs,
        "std_msgs.msg": std_msgs_msg,
        "sensor_msgs": sensor_msgs,
        "sensor_msgs.msg": sensor_msgs_msg,
    }
    old = {}
    for name, module in modules.items():
        old[name] = sys.modules.get(name)
        sys.modules[name] = module
    try:
        root = os.path.dirname(os.path.dirname(__file__))
        path = os.path.join(root, "tools", "can_interface", "statemachine",
                            "state_machine.py")
        module = imp.load_source("can_state_machine_per_axis_home_test", path)
    finally:
        for name, previous in old.items():
            if previous is None:
                del sys.modules[name]
            else:
                sys.modules[name] = previous
    return module


class PerAxisSetHomeTest(unittest.TestCase):
    def setUp(self):
        self.module = load_state_machine()
        self.bus = FakeBus()
        self.sm = self.module.StateMachine(self.bus)
        for axis in (10, 11, 12):
            self.assertTrue(self.sm.handle_use_selection(axis, True))
            self.sm.handle_connection_ping(axis, now=0.0)
        self.assertEqual([10, 11, 12], self.sm.handle_alignment_request(now=1.0))

    def test_aligned_axis_can_set_home_while_other_axes_are_still_aligning(self):
        self.assertTrue(self.sm.handle_alignment_result(10, 1, now=2.0))
        self.assertTrue(self.sm.legs[11].alignment_in_progress)
        self.assertTrue(self.sm.legs[12].alignment_in_progress)

        self.assertTrue(self.sm.handle_set_home(10))
        self.assertTrue(self.sm.legs[10].homed_in_current_session)
        self.assertEqual("Homed", self.sm.legs[10].state_str)
        self.assertFalse(self.sm.handle_set_home(11))
        self.assertFalse(self.sm.legs[11].homed_in_current_session)

        set_home_ids = [m.arbitration_id for m in self.bus.sent
                        if 0x300 <= m.arbitration_id < 0x318]
        self.assertEqual([0x30A], set_home_ids)

    def test_run_still_requires_all_active_axes_aligned_and_homed(self):
        self.assertTrue(self.sm.handle_alignment_result(10, 1, now=2.0))
        self.assertTrue(self.sm.handle_set_home(10))
        self.assertFalse(self.sm.handle_run_request())
        self.assertIn("leg11_missing_aligned", self.sm._run_blocking_reasons())
        self.assertIn("leg12_missing_aligned", self.sm._run_blocking_reasons())


if __name__ == "__main__":
    unittest.main()
