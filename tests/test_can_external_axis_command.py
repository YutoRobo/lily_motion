# -*- coding: utf-8 -*-
from __future__ import print_function

import os
import unittest

from test_can_legacy_alignment_retry import FakeBus, load_state_machine


class ExternalAxisCommandTest(unittest.TestCase):
    def make_ready(self, axis):
        module, unused_rospy = load_state_machine()
        bus = FakeBus()
        sm = module.StateMachine(bus)
        sm.handle_use_selection(axis, True)
        sm.handle_connection_ping(axis, now=0.0)
        sm.handle_alignment_request([axis], now=1.0)
        sm.handle_alignment_result(axis, 1, now=2.0)
        sm.handle_set_home(axis)
        bus.sent = []
        return sm, bus

    def message(self, text):
        return type("StringValue", (object,), {"data": text})()

    def test_a_ui_is_not_imported_for_external_axis11_path(self):
        self.assertNotIn("tools.can_interface.initUI.ui", __import__("sys").modules)
        sm, bus = self.make_ready(11)
        sm.external_axis_command_callback(self.message("diagnostic_run:11"))
        for offset in (0.005, 0.010, 0.005, 0.000):
            sm.external_axis_command_callback(
                self.message("position_offset:11:%.3f" % offset))
        self.assertEqual(
            [0x60B, 0x40B, 0x40B, 0x40B, 0x40B],
            [frame.arbitration_id for frame in bus.sent])

    def test_b_c_d_e_representative_axis_ids(self):
        for axis, run_id, position_id in (
                (0, 0x600, 0x400),
                (10, 0x60A, 0x40A),
                (11, 0x60B, 0x40B),
                (23, 0x617, 0x417)):
            sm, bus = self.make_ready(axis)
            ok, reasons = sm.submit_axis_command(
                "external", axis, "diagnostic_run", now=3.0)
            self.assertTrue(ok, reasons)
            ok, reasons = sm.submit_axis_command(
                "external", axis, "position_offset", 0.005, now=3.1)
            self.assertTrue(ok, reasons)
            self.assertEqual(
                [run_id, position_id],
                [frame.arbitration_id for frame in bus.sent])

    def test_f_all_24_axes_are_parameterized_without_other_axis_frames(self):
        for axis in range(24):
            sm, bus = self.make_ready(axis)
            self.assertTrue(sm.submit_axis_command(
                "external", axis, "diagnostic_run", now=3.0)[0])
            self.assertTrue(sm.submit_axis_command(
                "external", axis, "position_offset", 0.005, now=3.1)[0])
            self.assertEqual(
                [0x600 | axis, 0x400 | axis],
                [frame.arbitration_id for frame in bus.sent])

    def test_g_use_false_rejects_run_and_position_without_can(self):
        sm, bus = self.make_ready(11)
        sm.active_joints.discard(11)
        self.assertFalse(sm.submit_axis_command(
            "external", 11, "diagnostic_run", now=3.0)[0])
        self.assertFalse(sm.submit_axis_command(
            "external", 11, "position", 0.005, now=3.1)[0])
        self.assertEqual([], bus.sent)

    def test_h_axis10_does_not_send_align_when_motion_starts(self):
        sm, bus = self.make_ready(10)
        self.assertTrue(sm.submit_axis_command(
            "external", 10, "diagnostic_run", now=3.0)[0])
        self.assertTrue(sm.submit_axis_command(
            "external", 10, "position_offset", 0.005, now=3.1)[0])
        ids = [frame.arbitration_id for frame in bus.sent]
        self.assertEqual([0x60A, 0x40A], ids)
        self.assertNotIn(0x00A, ids)

    def test_i_ui_and_external_sources_use_same_common_api(self):
        sm_ui, bus_ui = self.make_ready(11)
        sm_ext, bus_ext = self.make_ready(11)
        ui_result = sm_ui.submit_axis_command(
            "ui", 11, "diagnostic_run", now=3.0)
        ext_result = sm_ext.submit_axis_command(
            "external", 11, "diagnostic_run", now=3.0)
        self.assertEqual(ui_result, ext_result)
        self.assertEqual(
            [m.arbitration_id for m in bus_ui.sent],
            [m.arbitration_id for m in bus_ext.sent])
        ui_position = sm_ui.submit_axis_command(
            "ui", 11, "position_offset", 0.005, now=3.1)
        ext_position = sm_ext.submit_axis_command(
            "external", 11, "position_offset", 0.005, now=3.1)
        self.assertEqual(ui_position, ext_position)
        self.assertEqual(
            [m.arbitration_id for m in bus_ui.sent],
            [m.arbitration_id for m in bus_ext.sent])

    def test_j_no_ui_only_flag_is_required_by_external_position(self):
        sm, bus = self.make_ready(11)
        sm.submit_axis_command("external", 11, "diagnostic_run", now=3.0)
        self.assertFalse(hasattr(sm, "ui_button_pressed"))
        self.assertFalse(hasattr(sm, "selected_ui_axis"))
        ok, reasons = sm.submit_axis_command(
            "external", 11, "position_offset", -0.005, now=3.1)
        self.assertTrue(ok, reasons)
        self.assertEqual(0x40B, bus.sent[-1].arbitration_id)

    def test_external_publisher_uses_ros_only_not_can_or_ui(self):
        path = os.path.join(os.path.dirname(__file__), "..", "tools",
                            "can_interface",
                            "publish_single_axis_external_test.py")
        with open(path) as stream:
            source = stream.read()
        self.assertIn("/can/axis_command", source)
        self.assertIn("position_offset", source)
        self.assertNotIn("import can", source)
        self.assertNotIn("can.interface", source)
        self.assertNotIn("Bus(", source)
        self.assertNotIn("initUI", source)
        self.assertNotIn("ui.py", source)

    def test_static_runtime_has_no_axis10_or_literal_axis10_ids(self):
        root = os.path.join(os.path.dirname(__file__), "..", "tools",
                            "can_interface")
        needles = ("0x00A", "0x40A", "0x60A", "axis == 10", "axis = 10")
        for current, unused_dirs, files in os.walk(root):
            if "testdata" in current:
                continue
            for filename in files:
                if not filename.endswith(".py"):
                    continue
                path = os.path.join(current, filename)
                with open(path) as stream:
                    source = stream.read()
                for needle in needles:
                    self.assertNotIn(needle, source, (needle, path))


if __name__ == "__main__":
    unittest.main()
