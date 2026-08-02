# -*- coding: utf-8 -*-
from __future__ import division

import imp
import math
import os
import sys
import types
import unittest


ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CAN_DIR = os.path.join(ROOT, "tools", "can_interface")
STATE_DIR = os.path.join(CAN_DIR, "statemachine")
PUBLISHER_PATH = os.path.join(
    ROOT, "tools", "publish_cmdforjetson_single_axis_test.py")


class FakePublisher(object):
    def __init__(self, *args, **kwargs):
        self.messages = []

    def publish(self, value):
        self.messages.append(value)


class FakeRospy(types.ModuleType):
    def __init__(self):
        types.ModuleType.__init__(self, "rospy")
        self.logs = []
        self.subscribers = []
        self.Publisher = FakePublisher

    def Subscriber(self, topic, unused_type, callback):
        self.subscribers.append((topic, callback))
        return object()

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
    def __init__(self):
        self.sent = []

    def send(self, msg):
        self.sent.append(msg)


def load_unified_state_machine():
    fake_rospy = FakeRospy()
    fake_can = FakeCan("can")
    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")
    std_msgs_msg.String = type("String", (object,), {})
    sensor_msgs = types.ModuleType("sensor_msgs")
    sensor_msgs_msg = types.ModuleType("sensor_msgs.msg")
    sensor_msgs_msg.JointState = type("JointState", (object,), {})
    replacements = {
        "rospy": fake_rospy,
        "can": fake_can,
        "std_msgs": std_msgs,
        "std_msgs.msg": std_msgs_msg,
        "sensor_msgs": sensor_msgs,
        "sensor_msgs.msg": sensor_msgs_msg,
    }
    old_modules = dict((name, sys.modules.get(name))
                       for name in replacements)
    old_state_machine = sys.modules.get("state_machine")
    old_paths = list(sys.path)
    try:
        sys.path.insert(0, CAN_DIR)
        sys.path.insert(0, STATE_DIR)
        for name, module in replacements.items():
            sys.modules[name] = module
        base = imp.load_source(
            "state_machine", os.path.join(STATE_DIR, "state_machine.py"))
        sys.modules["state_machine"] = base
        unified = imp.load_source(
            "unified_state_machine_under_test",
            os.path.join(STATE_DIR, "unified_state_machine.py"))
    finally:
        sys.path[:] = old_paths
        for name, previous in old_modules.items():
            if previous is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = previous
        if old_state_machine is None:
            sys.modules.pop("state_machine", None)
        else:
            sys.modules["state_machine"] = old_state_machine
    return unified, fake_rospy


class UnifiedCmdForJetsonStateMachineTest(unittest.TestCase):
    def setUp(self):
        self.module, self.rospy = load_unified_state_machine()
        self.bus = FakeBus()
        self.sm = self.module.StateMachine(self.bus)
        self.assertTrue(self.sm.handle_use_selection(10, True))

    def test_run_is_sent_only_to_use_true_axes(self):
        self.assertTrue(self.sm.send_run_start_command())
        self.assertEqual([0x60A],
                         [message.arbitration_id for message in self.bus.sent])

    def test_position_is_sent_only_to_use_true_axes(self):
        positions = [float("nan")] * 24
        positions[10] = 0.010
        self.assertIsNone(self.sm._position_limit_violation_all(positions))
        self.assertTrue(self.sm.send_position_command_all(positions))
        self.assertEqual([0x40A],
                         [message.arbitration_id for message in self.bus.sent])
        self.assertAlmostEqual(
            0.010, self.sm.legs[10].last_logical_position_command_rad)
        self.assertIsNone(self.sm.legs[11].last_logical_position_command_rad)

    def test_extra_use_axis_rejects_nan_guard(self):
        self.assertTrue(self.sm.handle_use_selection(11, True))
        positions = [float("nan")] * 24
        positions[10] = 0.010
        violation = self.sm._position_limit_violation_all(positions)
        self.assertIn("position[11] is non-finite", violation)

    def test_coordinate_callback_uses_same_active_only_can_path(self):
        self.sm.is_run = True
        positions = [float("nan")] * 24
        positions[10] = -0.010
        msg = type("JointStateValue", (object,), {"position": positions})()
        self.sm.coordinate_callback(msg)
        self.assertEqual([0x40A],
                         [message.arbitration_id for message in self.bus.sent])

    def test_legacy_external_axis_topic_is_ignored(self):
        msg = type("StringValue", (object,), {"data": "position:10:0.01"})()
        self.assertFalse(self.sm.external_axis_command_callback(msg))
        self.assertEqual([], self.bus.sent)
        self.assertTrue(any(
            "/can/axis_command ignored" in text
            for level, text in self.rospy.logs if level == "warn"))


class SingleAxisPublisherPureTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.publisher = imp.load_source(
            "cmdforjetson_single_axis_publisher_under_test", PUBLISHER_PATH)

    def test_offsets_include_start_peak_and_return_to_zero(self):
        offsets, peak_index = self.publisher.build_offsets(
            "plus", 0.020, 0.005)
        self.assertEqual(0.0, offsets[0])
        self.assertEqual(0.020, offsets[peak_index])
        self.assertEqual(0.0, offsets[-1])
        self.assertEqual(
            [0.0, 0.005, 0.010, 0.015, 0.020,
             0.015, 0.010, 0.005, 0.0], offsets)

    def test_negative_direction(self):
        offsets, unused_peak = self.publisher.build_offsets(
            "minus", 0.010, 0.005)
        self.assertEqual([0.0, -0.005, -0.010, -0.005, 0.0], offsets)

    def test_position_vector_has_one_finite_axis(self):
        positions = self.publisher.build_position(10, 0.125)
        self.assertEqual(24, len(positions))
        self.assertEqual(0.125, positions[10])
        self.assertEqual(1, sum(
            0 if math.isnan(value) else 1 for value in positions))

    def test_invalid_axis_and_step_are_rejected(self):
        with self.assertRaises(ValueError):
            self.publisher.build_position(24, 0.0)
        with self.assertRaises(ValueError):
            self.publisher.build_offsets("plus", 0.020, 0.003)


if __name__ == "__main__":
    unittest.main()
