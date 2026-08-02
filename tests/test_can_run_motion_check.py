# -*- coding: utf-8 -*-
from __future__ import print_function

import os
import struct
import sys
import unittest

sys.path.insert(0, os.path.dirname(__file__))
from test_can_legacy_alignment_retry import FakeBus, load_state_machine


class RunMotionCheckTest(unittest.TestCase):
    def setUp(self):
        self.module, self.rospy = load_state_machine()
        self.bus = FakeBus()
        self.sm = self.module.StateMachine(self.bus)
        self.sm.handle_use_selection(10, True)
        self.sm.handle_connection_ping(10, now=0.0)
        self.sm.handle_alignment_request(now=1.0)
        self.sm.handle_alignment_result(10, 1, now=2.0)
        self.sm.handle_set_home(10)
        self.sm.handle_run_request()
        self.bus.sent = []

    def position_frames(self):
        return [msg for msg in self.bus.sent
                if 0x400 <= msg.arbitration_id <= 0x417]

    def position_values(self):
        return [struct.unpack("<f", bytes(bytearray(msg.data[4:8])))[0]
                for msg in self.position_frames()]

    def drive_commands(self, count):
        while len(self.position_frames()) < count:
            due = self.sm.motion_check_next_send_time
            self.sm.execute(now=due + 1e-6)

    def drive_complete(self):
        for unused in range(20):
            if not self.sm.motion_check_active:
                return
            due = (self.sm.motion_check_complete_time
                   if self.sm.motion_check_complete_time is not None
                   else self.sm.motion_check_next_send_time)
            self.sm.execute(now=due + 1e-6)
        self.fail("motion check did not complete")

    def assert_values_close(self, expected):
        actual = self.position_values()
        self.assertEqual(len(expected), len(actual))
        for want, got in zip(expected, actual):
            self.assertAlmostEqual(want, got, places=6)

    def test_a_positive_normal_completion(self):
        self.assertTrue(self.sm.start_run_motion_check(10, 1, now=0.0))
        self.drive_complete()
        self.assert_values_close(
            [0.005, 0.010, 0.015, 0.020,
             0.015, 0.010, 0.005, 0.000])
        self.assertTrue(all(
            msg.arbitration_id == 0x40A for msg in self.position_frames()))
        self.assertEqual(0.0,
                         self.sm.legs[10].last_logical_position_command_rad)
        self.assertFalse(self.sm.motion_check_active)

    def test_b_negative_normal_completion(self):
        self.assertTrue(self.sm.start_run_motion_check(10, -1, now=0.0))
        self.drive_complete()
        self.assert_values_close(
            [-0.005, -0.010, -0.015, -0.020,
             -0.015, -0.010, -0.005, 0.000])

    def test_c_nonzero_q0(self):
        self.sm.legs[10].last_logical_position_command_rad = 0.100
        self.assertTrue(self.sm.start_run_motion_check(10, 1, now=0.0))
        self.drive_complete()
        self.assert_values_close(
            [0.105, 0.110, 0.115, 0.120,
             0.115, 0.110, 0.105, 0.100])

    def test_d_use_false_rejected_without_can(self):
        self.sm.active_joints.discard(10)
        self.assertFalse(self.sm.start_run_motion_check(10, 1, now=0.0))
        self.assertEqual([], self.position_frames())

    def test_e_unaligned_rejected_without_can(self):
        self.sm.legs[10].aligned_in_current_session = False
        self.assertFalse(self.sm.start_run_motion_check(10, 1, now=0.0))
        self.assertEqual([], self.position_frames())

    def test_f_unhomed_rejected_without_can(self):
        self.sm.legs[10].homed_in_current_session = False
        self.assertFalse(self.sm.start_run_motion_check(10, 1, now=0.0))
        self.assertEqual([], self.position_frames())

    def test_g_not_running_rejected_without_can(self):
        self.sm.legs[10].running_in_current_session = False
        self.assertFalse(self.sm.start_run_motion_check(10, 1, now=0.0))
        self.assertEqual([], self.position_frames())

    def test_h_unknown_q0_rejected_without_can(self):
        self.sm.legs[10].last_logical_position_command_rad = None
        self.assertFalse(self.sm.start_run_motion_check(10, 1, now=0.0))
        self.assertIn(
            "last_position_unknown",
            self.sm.motion_check_status_pub.messages[-1])
        self.assertEqual([], self.position_frames())

    def test_i_runtime_error_stops_without_automatic_return(self):
        self.sm.start_run_motion_check(10, 1, now=0.0)
        self.sm.execute(now=0.1)
        self.sm.execute(now=0.2)
        before = len(self.position_frames())
        frame = type("CanFrame", (object,), {
            "arbitration_id": 0x0EE,
            "data": [10, 5, 0, 0, 0, 0, 0, 2]})()
        self.sm.can_callback(frame)
        for now in (0.3, 0.4, 1.0, 2.0):
            self.sm.execute(now=now)
        self.assertEqual(before, len(self.position_frames()))
        self.assertFalse(self.sm.motion_check_active)
        self.assertFalse(self.sm.is_run)

    def test_j_unexpected_heartbeat_aborts_without_return(self):
        self.sm.start_run_motion_check(10, 1, now=0.0)
        self.sm.execute(now=0.1)
        before = len(self.position_frames())
        self.sm.handle_connection_ping(10, now=0.2)
        self.sm.execute(now=1.0)
        self.assertEqual(before, len(self.position_frames()))
        self.assertFalse(self.sm.motion_check_active)
        self.assertFalse(self.sm.legs[10].aligned_in_current_session)
        self.assertFalse(self.sm.legs[10].homed_in_current_session)

    def test_k_ui_cancel_returns_in_step_increments(self):
        self.sm.start_run_motion_check(10, 1, now=0.0)
        self.drive_commands(3)
        cancel_time = self.sm.motion_check_next_send_time - 0.01
        self.assertTrue(self.sm.cancel_run_motion_check(now=cancel_time))
        self.drive_complete()
        self.assert_values_close(
            [0.005, 0.010, 0.015, 0.010, 0.005, 0.000])
        self.assertFalse(self.sm.motion_check_active)
        self.assertEqual(0.0,
                         self.sm.legs[10].last_logical_position_command_rad)

    def test_l_second_start_is_rejected(self):
        self.assertTrue(self.sm.start_run_motion_check(10, 1, now=0.0))
        self.assertFalse(self.sm.start_run_motion_check(10, -1, now=0.0))
        self.drive_complete()
        self.assertEqual(8, len(self.position_frames()))

    def test_m_other_axes_are_untouched(self):
        before = [(leg.aligned_in_current_session,
                   leg.homed_in_current_session,
                   leg.last_logical_position_command_rad)
                  for leg in self.sm.legs]
        self.sm.start_run_motion_check(10, 1, now=0.0)
        self.drive_complete()
        self.assertTrue(all(
            msg.arbitration_id == 0x40A for msg in self.position_frames()))
        after = [(leg.aligned_in_current_session,
                  leg.homed_in_current_session,
                  leg.last_logical_position_command_rad)
                 for leg in self.sm.legs]
        for axis in range(24):
            if axis != 10:
                self.assertEqual(before[axis], after[axis])


if __name__ == "__main__":
    unittest.main()
