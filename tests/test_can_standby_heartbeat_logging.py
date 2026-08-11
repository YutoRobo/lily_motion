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
    class Message(object):
        def __init__(self, arbitration_id, data, is_extended_id=False):
            self.arbitration_id = arbitration_id
            self.data = list(data)
            self.is_extended_id = is_extended_id


class FakeBus(object):
    def send(self, msg):
        pass


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
            "can_state_machine_standby_heartbeat_logging_test", path)
    finally:
        for name, previous in old.items():
            if previous is None:
                del sys.modules[name]
            else:
                sys.modules[name] = previous
    return module, fake_rospy


class StandbyHeartbeatLoggingTest(unittest.TestCase):
    def setUp(self):
        self.module, self.rospy = load_state_machine()
        self.sm = self.module.StateMachine(FakeBus())
        self.rospy.logs[:] = []

    def standby_info_logs(self):
        return [message for level, message in self.rospy.logs
                if level == "info" and "standby heartbeat" in message]

    def test_repeated_standby_heartbeat_is_not_logged_repeatedly(self):
        self.sm.handle_connection_ping(10, now=0.0)
        self.assertEqual(1, len(self.standby_info_logs()))
        self.assertIn("discovered", self.standby_info_logs()[0])

        self.sm.handle_connection_ping(10, now=1.0)
        self.sm.handle_connection_ping(10, now=2.0)
        self.assertEqual(1, len(self.standby_info_logs()))

    def test_recovery_heartbeat_is_logged_once(self):
        self.sm.handle_connection_ping(10, now=0.0)
        self.sm.legs[10].awaiting_heartbeat = True

        self.sm.handle_connection_ping(10, now=1.0)
        self.assertEqual(2, len(self.standby_info_logs()))
        self.assertIn("recovered", self.standby_info_logs()[-1])


if __name__ == "__main__":
    unittest.main()
