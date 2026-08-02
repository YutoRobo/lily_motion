# -*- coding: utf-8 -*-
from __future__ import print_function
import os
import struct
import unittest
from test_can_legacy_alignment_retry import FakeBus, load_state_machine

class DiagnosticRunTest(unittest.TestCase):
    def setUp(self):
        self.module, self.rospy = load_state_machine()
        self.bus = FakeBus()
        self.sm = self.module.StateMachine(self.bus)

    def ready(self, axis=10):
        self.sm.handle_use_selection(axis, True)
        self.sm.handle_connection_ping(axis, now=0.0)
        self.sm.handle_alignment_request([axis], now=1.0)
        self.sm.handle_alignment_result(axis, 1, now=2.0)
        self.sm.handle_set_home(axis)
        self.bus.sent = []

    def test_a_target_candidates_are_use_discovered_and_nonfatal(self):
        self.sm.handle_use_selection(10, True)
        self.sm.handle_connection_ping(10, now=0.0)
        self.sm.handle_use_selection(11, False)
        self.sm.handle_connection_ping(11, now=0.0)
        self.sm.handle_use_selection(12, True)
        self.assertEqual([10], self.sm.get_diagnostic_target_axes())

    def test_b_other_use_axes_do_not_gate_selected_axis(self):
        self.ready(10)
        self.sm.active_joints.update((11, 12))
        self.sm.legs[11].discovered_once_in_current_session = True
        self.sm.legs[12].discovered_once_in_current_session = True
        allowed, reasons = self.sm.can_start_diagnostic_run(10, now=3.0)
        self.assertTrue(allowed, reasons)

    def test_c_selected_axis_run_sends_only_one_id_and_sets_flags(self):
        self.ready(10)
        self.assertTrue(self.sm.start_diagnostic_run(10, now=3.0))
        self.assertEqual([0x60A], [m.arbitration_id for m in self.bus.sent])
        self.assertTrue(self.sm.legs[10].diagnostic_run_command_sent)
        self.assertTrue(self.sm.legs[10].run_command_sent_in_current_session)
        self.assertFalse(self.sm.is_run)

    def test_d_run_send_failure_blocks_motion(self):
        self.ready(10)
        self.bus.fail = True
        self.assertFalse(self.sm.start_diagnostic_run(10, now=3.0))
        self.assertFalse(self.sm.legs[10].diagnostic_run_command_sent)
        self.assertTrue(self.sm.send_error_latched)
        self.assertFalse(self.sm.start_motion_check(10, 1, now=4.0))

    def test_e_motion_check_after_diagnostic_run_sends_only_axis(self):
        self.ready(10)
        self.assertTrue(self.sm.start_diagnostic_run(10, now=3.0))
        self.bus.sent = []
        self.assertTrue(self.sm.start_motion_check(10, 1, now=4.0))
        for unused in range(20):
            if not self.sm.motion_check_active:
                break
            due = (self.sm.motion_check_complete_time if self.sm.motion_check_complete_time is not None else self.sm.motion_check_next_send_time)
            self.sm.execute(now=due + 1e-6)
        frames = [m for m in self.bus.sent if 0x400 <= m.arbitration_id <= 0x417]
        self.assertEqual(8, len(frames))
        self.assertTrue(all(m.arbitration_id == 0x40A for m in frames))
        values = [struct.unpack("<f", bytes(bytearray(m.data[4:8])))[0] for m in frames]
        self.assertAlmostEqual(0.0, values[-1], places=6)

    def test_f_g_h_selected_axis_gates(self):
        self.ready(10)
        for attr, reason in (("aligned_in_current_session", "not_aligned"), ("homed_in_current_session", "not_homed")):
            old = getattr(self.sm.legs[10], attr)
            setattr(self.sm.legs[10], attr, False)
            allowed, reasons = self.sm.can_start_diagnostic_run(10, now=3.0)
            self.assertFalse(allowed)
            self.assertTrue(any(reason in item for item in reasons))
            setattr(self.sm.legs[10], attr, old)
        self.sm.active_joints.discard(10)
        self.assertFalse(self.sm.can_start_diagnostic_run(10, now=3.0)[0])
        self.assertEqual([], self.bus.sent)

    def test_i_heartbeat_freshness_is_not_a_diagnostic_gate(self):
        self.ready(10)
        self.sm.execute(now=10.0)
        self.assertIn(10, self.sm.get_diagnostic_target_axes())
        self.assertTrue(self.sm.can_start_diagnostic_run(10, now=10.0)[0])

    def test_j_unexpected_heartbeat_invalidates_diagnostic_session(self):
        self.ready(10)
        self.sm.start_diagnostic_run(10, now=3.0)
        self.sm.start_motion_check(10, 1, now=4.0)
        self.sm.handle_connection_ping(10, now=4.1)
        leg = self.sm.legs[10]
        self.assertFalse(leg.diagnostic_run_command_sent)
        self.assertFalse(leg.run_command_sent_in_current_session)
        self.assertFalse(leg.aligned_in_current_session)
        self.assertFalse(leg.homed_in_current_session)
        self.assertFalse(self.sm.motion_check_active)

    def test_k_ui_has_no_can_packing_or_motion_sequence(self):
        path = os.path.join(os.path.dirname(__file__), "..", "tools", "can_interface", "initUI", "ui.py")
        with open(path) as stream:
            source = stream.read()
        self.assertIn("diagnostic_run:{}", source)
        self.assertIn("/ui/diagnostic_targets", source)
        self.assertNotIn("0x600", source)
        self.assertNotIn("0x400", source)
        self.assertNotIn("struct.pack", source)
        self.assertNotIn("time.sleep", source)
        self.assertNotIn("build_motion_values", source)
        self.assertNotIn('leg.state == "Running"', source)

    def test_l_normal_all_axis_run_is_unchanged(self):
        self.ready(10)
        self.assertTrue(self.sm.handle_run_request())
        ids = [m.arbitration_id for m in self.bus.sent if 0x600 <= m.arbitration_id <= 0x617]
        self.assertEqual(list(range(0x600, 0x618)), ids)

if __name__ == "__main__":
    unittest.main()
