# -*- coding: utf-8 -*-
from __future__ import print_function

import os
import sys
import time
import unittest

sys.path.insert(0, os.path.dirname(__file__))
from test_can_legacy_alignment_retry import FakeBus, load_state_machine

EMULATOR_DIR = os.path.join(
    os.path.dirname(__file__), "..", "tools", "can_interface", "emulator")
if EMULATOR_DIR not in sys.path:
    sys.path.insert(0, EMULATOR_DIR)
from multi_actuator_emulator import MultiActuatorEmulator
from scenario import ActuatorScenario


class Frame(object):
    def __init__(self, arbitration_id, data):
        self.arbitration_id = arbitration_id
        self.data = list(data)


class CanBridge(object):
    def __init__(self, axes, scenarios=None):
        self.module, self.rospy = load_state_machine()
        self.pc_bus = FakeBus()
        self.sm = self.module.StateMachine(self.pc_bus)
        self.emulator_frames = []
        self.emulator = MultiActuatorEmulator(
            axes, self.emulator_transmit, scenarios=scenarios,
            heartbeat_period=1.0, align_delay=0.1, reset_delay=0.2,
            now=self.base)
        self.pc_frame_index = 0

    @property
    def base(self):
        if not hasattr(self, "_base"):
            self._base = time.time()
        return self._base

    def emulator_transmit(self, arbitration_id, data):
        frame = Frame(arbitration_id, data)
        self.emulator_frames.append(frame)
        self.sm.can_callback(frame)

    def drain_pc_to_emulator(self, now):
        while self.pc_frame_index < len(self.pc_bus.sent):
            frame = self.pc_bus.sent[self.pc_frame_index]
            self.pc_frame_index += 1
            self.emulator.receive(frame, now)

    def tick(self, now):
        self.emulator.tick(now)

    def discover(self, now=None):
        self.tick(self.base if now is None else now)

    def align(self, now=None):
        now = self.base + 0.01 if now is None else now
        sent = self.sm.handle_alignment_request(now=now)
        self.drain_pc_to_emulator(now)
        self.tick(now + 0.11)
        return sent

    def home_all(self):
        for axis in self.sm._active_joint_ids():
            self.sm.handle_set_home(axis)
        self.drain_pc_to_emulator(self.base + 1.0)


class MultiActuatorIntegrationTest(unittest.TestCase):
    def test_c_d_connected_use_auto_on_and_three_axis_align(self):
        bridge = CanBridge((10, 11, 12))
        bridge.discover()
        for axis in (10, 11, 12):
            self.assertTrue(bridge.sm.legs[axis].connected)
            self.assertIn(axis, bridge.sm.active_joints)
        self.assertEqual([10, 11, 12], bridge.align())
        self.assertTrue(all(
            bridge.sm.legs[axis].aligned_in_current_session
            for axis in (10, 11, 12)))

    def test_c_manual_use_off_survives_later_heartbeat(self):
        bridge = CanBridge((10,))
        bridge.discover()
        bridge.sm.handle_use_selection(10, False)
        bridge.tick(bridge.base + 1.1)
        self.assertTrue(bridge.sm.legs[10].connected)
        self.assertNotIn(10, bridge.sm.active_joints)

    def test_e_partial_failure_retry_preserves_successful_axes(self):
        scenarios = {
            10: ActuatorScenario(),
            11: ActuatorScenario(fail_attempts=(1,)),
            12: ActuatorScenario(),
        }
        bridge = CanBridge((10, 11, 12), scenarios)
        bridge.discover()
        bridge.align()
        self.assertTrue(bridge.sm.legs[10].aligned_in_current_session)
        self.assertFalse(bridge.sm.legs[11].aligned_in_current_session)
        self.assertTrue(bridge.sm.legs[12].aligned_in_current_session)
        bridge.tick(bridge.base + 0.5)
        before = len(bridge.pc_bus.sent)
        sent = bridge.sm.handle_alignment_request(now=bridge.base + 0.51)
        self.assertEqual([11], sent)
        bridge.drain_pc_to_emulator(bridge.base + 0.51)
        self.assertEqual(
            [0x00B],
            [m.arbitration_id for m in bridge.pc_bus.sent[before:]])
        bridge.tick(bridge.base + 0.62)
        self.assertTrue(bridge.sm.alignment_complete())
        self.assertTrue(bridge.sm.legs[10].aligned_in_current_session)
        self.assertTrue(bridge.sm.legs[12].aligned_in_current_session)

    def test_m_reset_after_run_invalidates_pc_session(self):
        bridge = CanBridge(
            (10,), {10: ActuatorScenario(reset_after_run_sec=0.2)})
        bridge.discover()
        bridge.align()
        bridge.home_all()
        self.assertTrue(bridge.sm.handle_run_request())
        bridge.drain_pc_to_emulator(bridge.base + 1.1)
        self.assertTrue(bridge.sm.legs[10].running_in_current_session)
        bridge.tick(bridge.base + 1.31)
        leg = bridge.sm.legs[10]
        self.assertFalse(leg.aligned_in_current_session)
        self.assertFalse(leg.homed_in_current_session)
        self.assertFalse(leg.run_command_sent_in_current_session)
        self.assertFalse(bridge.sm.is_run)

    def test_o_24_axis_normal_position_fanout(self):
        bridge = CanBridge(range(24))
        bridge.discover()
        self.assertEqual(list(range(24)), bridge.align())
        bridge.home_all()
        self.assertTrue(bridge.sm.handle_run_request())
        bridge.drain_pc_to_emulator(bridge.base + 1.1)
        before = len(bridge.pc_bus.sent)
        msg = type("JointStateValue", (object,), {
            "position": [axis * 0.001 for axis in range(24)]})()
        bridge.sm.coordinate_callback(msg)
        bridge.drain_pc_to_emulator(bridge.base + 1.2)
        self.assertEqual(
            list(range(0x400, 0x418)),
            [m.arbitration_id for m in bridge.pc_bus.sent[before:]])
        for axis in range(24):
            self.assertAlmostEqual(
                axis * 0.001,
                bridge.emulator.actuators[axis].last_position_command_rad,
                places=6)

    def test_p_use_false_sends_no_align_diagnostic_or_position(self):
        bridge = CanBridge((10, 11))
        bridge.discover()
        bridge.sm.handle_use_selection(11, False)
        self.assertEqual([10], bridge.sm.handle_alignment_request(
            now=bridge.base + 0.01))
        ids = [m.arbitration_id for m in bridge.pc_bus.sent]
        self.assertNotIn(0x00B, ids)
        ok, unused_reasons = bridge.sm.submit_axis_command(
            "external", 11, "diagnostic_run", now=bridge.base + 0.02)
        self.assertFalse(ok)
        ok, unused_reasons = bridge.sm.submit_axis_command(
            "external", 11, "position", value=0.005,
            now=bridge.base + 0.03)
        self.assertFalse(ok)
        self.assertNotIn(0x60B, [m.arbitration_id for m in bridge.pc_bus.sent])
        self.assertNotIn(0x40B, [m.arbitration_id for m in bridge.pc_bus.sent])


class FrameMessage(object):
    def __init__(self, text):
        self.data = text


if __name__ == "__main__":
    unittest.main()
