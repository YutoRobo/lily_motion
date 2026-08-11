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
        self.logs = []
        self.Publisher = FakePublisher
        self.Subscriber = lambda *args, **kwargs: None

    def _log(self, level, fmt, *args):
        self.logs.append((level, fmt % args if args else fmt))

    def loginfo(self, fmt, *args):
        self._log("info", fmt, *args)

    def logwarn(self, fmt, *args):
        self._log("warn", fmt, *args)

    def logerr(self, fmt, *args):
        self._log("error", fmt, *args)


class FakeCan(types.ModuleType):
    class CanError(Exception):
        pass

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
        module = imp.load_source(
            "can_state_machine_alignment_heartbeat_race_test", path)
    finally:
        for name, previous in old.items():
            if previous is None:
                del sys.modules[name]
            else:
                sys.modules[name] = previous
    return module, fake_rospy


class AlignmentHeartbeatRaceTest(unittest.TestCase):
    def setUp(self):
        self.module, self.rospy = load_state_machine()
        self.bus = FakeBus()
        self.sm = self.module.StateMachine(self.bus)
        self.assertTrue(self.sm.handle_use_selection(10, True))
        self.sm.handle_connection_ping(10, now=0.0)
        self.assertEqual([10], self.sm.handle_alignment_request(now=1.0))

    def test_heartbeat_during_alignment_keeps_request_alive(self):
        leg = self.sm.legs[10]
        deadline = leg.alignment_deadline
        generation = leg.alignment_request_generation
        self.assertTrue(leg.alignment_in_progress)
        self.assertEqual("Aligning", leg.state_str)

        self.sm.handle_connection_ping(10, now=2.0)

        self.assertTrue(leg.connected)
        self.assertEqual(2.0, leg.last_seen)
        self.assertTrue(leg.alignment_in_progress)
        self.assertEqual(deadline, leg.alignment_deadline)
        self.assertEqual(generation, leg.alignment_request_generation)
        self.assertFalse(leg.initialization_error_latched)
        self.assertEqual("Aligning", leg.state_str)

        self.assertTrue(self.sm.handle_alignment_result(10, 1, now=3.0))
        self.assertTrue(leg.aligned_in_current_session)
        self.assertFalse(leg.alignment_in_progress)
        self.assertEqual("Aligned", leg.state_str)

        self.sm.execute(now=100.0)
        self.assertTrue(leg.aligned_in_current_session)
        self.assertEqual("Aligned", leg.state_str)

    def test_run_heartbeat_safety_behavior_is_unchanged(self):
        self.assertTrue(self.sm.handle_alignment_result(10, 1, now=2.0))
        self.assertTrue(self.sm.handle_set_home(10))
        self.assertTrue(self.sm.handle_run_request())

        self.sm.handle_connection_ping(10, now=50.0)

        self.assertFalse(self.sm.is_run)
        self.assertFalse(self.sm.legs[10].aligned_in_current_session)
        self.assertFalse(self.sm.legs[10].homed_in_current_session)


if __name__ == "__main__":
    unittest.main()
