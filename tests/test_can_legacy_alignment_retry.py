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
        self.fail = False

    def send(self, msg):
        if self.fail:
            raise RuntimeError("offline send failure")
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
        path = os.path.join(
            os.path.dirname(__file__), "..", "tools", "can_interface",
            "statemachine", "state_machine.py")
        module = imp.load_source("can_state_machine_under_test", path)
    finally:
        for name, previous in old.items():
            if previous is None:
                del sys.modules[name]
            else:
                sys.modules[name] = previous
    return module, fake_rospy


class LegacyAlignmentRetryTest(unittest.TestCase):
    ACTIVE = (10, 11, 12)

    def setUp(self):
        self.module, self.rospy = load_state_machine()
        self.bus = FakeBus()
        self.sm = self.module.StateMachine(self.bus)
        for leg_id in self.ACTIVE:
            self.assertTrue(self.sm.handle_use_selection(leg_id, True))

    def heartbeat_active(self, ids=None, now=0.0):
        for leg_id in (self.ACTIVE if ids is None else ids):
            self.sm.handle_connection_ping(leg_id, now=now)

    def start_active_alignment(self, now=1.0):
        self.heartbeat_active(now=0.0)
        self.assertEqual(list(self.ACTIVE),
                         self.sm.handle_alignment_request(now=now))

    def align_success(self, ids=None, now=2.0):
        for leg_id in (self.ACTIVE if ids is None else ids):
            self.assertTrue(self.sm.handle_alignment_result(leg_id, 1, now=now))

    def home_active(self):
        for leg_id in self.ACTIVE:
            self.assertTrue(self.sm.handle_set_home(leg_id))

    def test_a_initial_alignment_all_success(self):
        self.start_active_alignment()
        self.align_success()
        self.assertTrue(self.sm.alignment_complete())
        self.assertEqual([], self.sm._home_blocking_reasons())
        self.assertTrue(all(self.sm.legs[i].aligned for i in self.ACTIVE))

    def test_b_one_axis_failure_keeps_successful_axes(self):
        self.start_active_alignment()
        self.align_success((10, 12))
        self.sm.handle_mcu_error(11, 8)
        self.assertTrue(self.sm.legs[10].aligned)
        self.assertFalse(self.sm.legs[11].aligned)
        self.assertTrue(self.sm.legs[12].aligned)
        self.assertFalse(self.sm.alignment_complete())
        self.assertIn("pending=[11]", self.sm.alignment_status_summary())
        self.assertTrue(self.sm._home_blocking_reasons())

    def test_c_retry_sends_only_recovered_failed_axis(self):
        self.start_active_alignment()
        self.align_success((10, 12))
        self.sm.handle_mcu_error(11, 8)
        before = len(self.bus.sent)
        self.sm.handle_connection_ping(11, now=3.0)
        self.assertEqual([11], self.sm.handle_alignment_request(now=4.0))
        self.assertEqual([11], [m.arbitration_id for m in self.bus.sent[before:]])
        self.assertTrue(self.sm.legs[10].aligned)
        self.assertTrue(self.sm.legs[12].aligned)
        self.assertTrue(self.sm.handle_alignment_result(11, 1, now=5.0))
        self.assertTrue(self.sm.alignment_complete())

    def test_d_retry_waits_for_heartbeat_recovery(self):
        self.start_active_alignment()
        self.align_success((10, 12))
        self.sm.handle_mcu_error(11, 8)
        before = len(self.bus.sent)
        self.assertEqual([], self.sm.handle_alignment_request(now=4.0))
        self.assertEqual(before, len(self.bus.sent))
        self.assertTrue(self.sm.legs[10].aligned)
        self.assertTrue(self.sm.legs[12].aligned)

    def test_e_use_false_does_not_gate_or_receive_commands(self):
        self.start_active_alignment()
        self.align_success()
        self.home_active()
        self.assertFalse(self.sm.legs[13].connected)
        self.assertTrue(self.sm.handle_run_request())
        run_ids = [m.arbitration_id for m in self.bus.sent if 0x600 <= m.arbitration_id < 0x618]
        self.assertEqual([0x60A, 0x60B, 0x60C], run_ids)
        before = len(self.bus.sent)
        msg = type("JointStateValue", (object,), {"position": [0.0] * 24})()
        self.sm.coordinate_callback(msg)
        self.assertEqual([0x40A, 0x40B, 0x40C],
                         [m.arbitration_id for m in self.bus.sent[before:]])

    def test_f_expired_old_alignment_result_is_rejected(self):
        self.start_active_alignment(now=1.0)
        old_generation = self.sm.legs[11].alignment_request_generation
        self.sm.execute(now=1.0 + self.module.ALIGNMENT_TIMEOUT_SEC + 1.0)
        self.assertFalse(self.sm.handle_alignment_result(11, 1, now=40.0))
        self.assertFalse(self.sm.legs[11].aligned)
        self.assertEqual(old_generation,
                         self.sm.legs[11].alignment_request_generation)

    def test_g_unsolicited_result_for_successful_axis_is_ignored(self):
        self.start_active_alignment()
        self.align_success((10,))
        generation = self.sm.legs[10].alignment_request_generation
        self.assertFalse(self.sm.handle_alignment_result(10, 0, now=3.0))
        self.assertTrue(self.sm.legs[10].aligned)
        self.assertEqual(generation,
                         self.sm.legs[10].alignment_request_generation)

    def test_h_heartbeat_stops_after_alignment_without_timeout(self):
        self.start_active_alignment()
        self.align_success()
        self.sm.execute(now=100.0)
        self.assertTrue(all(self.sm.legs[i].aligned for i in self.ACTIVE))
        self.assertEqual([], self.sm._home_blocking_reasons())

    def test_i_unexpected_heartbeat_invalidates_axis_and_stops_run(self):
        self.start_active_alignment()
        self.align_success()
        self.home_active()
        self.assertTrue(self.sm.handle_run_request())
        self.sm.handle_connection_ping(10, now=50.0)
        self.assertFalse(self.sm.is_run)
        self.assertFalse(self.sm.legs[10].aligned_in_current_session)
        self.assertFalse(self.sm.legs[10].homed_in_current_session)
        self.assertTrue(self.sm.legs[11].aligned_in_current_session)
        self.assertTrue(self.sm._run_blocking_reasons())

    def test_j_run_gate_uses_only_use_axes_and_requires_completion(self):
        self.heartbeat_active()
        self.sm.handle_alignment_request(now=1.0)
        self.align_success((10, 11))
        self.assertFalse(self.sm.handle_run_request())
        self.assertIn("leg12_missing_aligned", self.sm._run_blocking_reasons())
        self.assertFalse(any("leg13" in reason
                             for reason in self.sm._run_blocking_reasons()))


class ErrorFrameClassificationTest(unittest.TestCase):
    ACTIVE = (10, 11, 12)

    def setUp(self):
        self.module, self.rospy = load_state_machine()
        self.bus = FakeBus()
        self.sm = self.module.StateMachine(self.bus)
        for leg_id in self.ACTIVE:
            self.sm.handle_use_selection(leg_id, True)

    def error_frame(self, leg_id, error_id, low_state=2, extension=None):
        extension = [0, 0, 0, 0, 0] if extension is None else list(extension)
        data = [leg_id, low_state] + extension + [error_id]
        msg = type("CanFrame", (object,), {
            "arbitration_id": 0x0EE, "data": data})()
        self.sm.can_callback(msg)

    def prepare_partial_alignment(self):
        for leg_id in self.ACTIVE:
            self.sm.handle_connection_ping(leg_id, now=0.0)
        self.sm.handle_alignment_request(now=1.0)
        self.sm.handle_alignment_result(10, 1, now=2.0)
        self.sm.handle_alignment_result(12, 1, now=2.0)

    def prepare_running(self):
        for leg_id in self.ACTIVE:
            self.sm.handle_connection_ping(leg_id, now=0.0)
        self.sm.handle_alignment_request(now=1.0)
        for leg_id in self.ACTIVE:
            self.sm.handle_alignment_result(leg_id, 1, now=2.0)
        for leg_id in self.ACTIVE:
            self.sm.handle_set_home(leg_id)
        self.assertTrue(self.sm.handle_run_request())

    def test_a_active_aliment_error_is_retryable_not_global(self):
        self.prepare_partial_alignment()
        self.error_frame(11, 8)
        self.assertFalse(self.sm.error_latched)
        self.assertFalse(self.sm.legs[11].aligned_in_current_session)
        self.assertTrue(self.sm.legs[10].aligned_in_current_session)
        self.assertTrue(self.sm.legs[12].aligned_in_current_session)
        self.assertTrue(self.sm.legs[11].awaiting_heartbeat)

    def test_b_active_aliment_stop_error_is_retryable_not_global(self):
        self.prepare_partial_alignment()
        self.error_frame(11, 9)
        self.assertFalse(self.sm.error_latched)
        self.assertEqual("ALIMENT_STOP_ERR", self.sm.legs[11].last_error_name)
        self.assertTrue(self.sm.legs[11].awaiting_heartbeat)

    def test_c_active_fault_yet_initialise_is_retryable_not_global(self):
        self.prepare_partial_alignment()
        self.error_frame(11, 12)
        self.assertFalse(self.sm.error_latched)
        self.assertEqual("FAULT_YET_INITIALISE_ERR",
                         self.sm.legs[11].last_error_name)
        self.assertTrue(self.sm.legs[11].initialization_error_latched)

    def test_d_retry_success_clears_axis_initialization_error(self):
        self.prepare_partial_alignment()
        self.error_frame(11, 8)
        self.sm.handle_connection_ping(11, now=3.0)
        self.assertEqual([11], self.sm.handle_alignment_request(now=4.0))
        self.assertTrue(self.sm.handle_alignment_result(11, 1, now=5.0))
        self.assertFalse(self.sm.legs[11].initialization_error_latched)
        self.assertTrue(self.sm.alignment_complete())
        self.assertEqual([], self.sm._home_blocking_reasons())

    def test_e_inactive_initialization_error_does_not_gate(self):
        for leg_id in self.ACTIVE:
            self.sm.handle_connection_ping(leg_id, now=0.0)
        self.sm.handle_alignment_request(now=1.0)
        for leg_id in self.ACTIVE:
            self.sm.handle_alignment_result(leg_id, 1, now=2.0)
        for leg_id in self.ACTIVE:
            self.sm.handle_set_home(leg_id)
        self.error_frame(13, 8)
        self.assertFalse(self.sm.error_latched)
        self.assertEqual([], self.sm._home_blocking_reasons())
        self.assertEqual([], self.sm._run_blocking_reasons())
        self.assertTrue(self.sm.handle_run_request())

    def test_f_inactive_runtime_error_latches_and_stops(self):
        self.prepare_running()
        self.error_frame(13, 2)
        self.assertTrue(self.sm.error_latched)
        self.assertFalse(self.sm.is_run)
        self.assertEqual("POS_CMD_JUMP_ERR",
                         self.sm.global_error_details[-1]["error_name"])

    def test_g_active_fault_yet_error_latches_and_stops(self):
        self.prepare_running()
        self.error_frame(10, 7)
        self.assertTrue(self.sm.error_latched)
        self.assertFalse(self.sm.is_run)
        self.assertEqual("FAULT_YET_ERR",
                         self.sm.global_error_details[-1]["error_name"])

    def test_h_invalid_dlc_changes_no_state(self):
        before = (self.sm.error_latched,
                  self.sm.legs[10].last_error_code,
                  self.sm.legs[10].state_str)
        msg = type("CanFrame", (object,), {
            "arbitration_id": 0x0EE,
            "data": [10, 2, 0, 0, 0, 0, 8]})()
        self.sm.can_callback(msg)
        self.assertEqual(before, (self.sm.error_latched,
                                 self.sm.legs[10].last_error_code,
                                 self.sm.legs[10].state_str))

    def test_i_error_id_names_match_main_c_enum(self):
        expected = [
            "NOMINAL", "POS_CMD_NOT_READY", "POS_CMD_JUMP_ERR",
            "POS_CMD_OUT_OF_RANGE_ERR", "POS_OUT_OF_RANGE_ERR",
            "POS_ERROR_OUT_OF_RANGE_ERR", "POS_CMD_LUG_ERR",
            "FAULT_YET_ERR", "ALIMENT_ERR", "ALIMENT_STOP_ERR",
            "ACT_POS_OUT_OF_RANGE_ERR", "OTHER_AXIS_ERR",
            "FAULT_YET_INITIALISE_ERR", "OTHER"]
        self.assertEqual(expected, [
            self.module.error_id_name(value) for value in range(14)])

    def test_unknown_low_freq_state_is_accepted_and_named(self):
        self.error_frame(10, 8, low_state=9)
        self.assertEqual("unknown_state_9",
                         self.sm.legs[10].last_error_low_freq_state_name)
        self.assertTrue(self.sm.legs[10].awaiting_heartbeat)

    def test_unknown_error_id_is_system_fatal(self):
        self.error_frame(10, 14)
        self.assertTrue(self.sm.error_latched)
        self.assertEqual("unknown_error_14",
                         self.sm.global_error_details[-1]["error_name"])

    def test_nonzero_extension_is_saved_and_processed(self):
        self.error_frame(10, 8, extension=[1, 2, 3, 4, 5])
        self.assertEqual((1, 2, 3, 4, 5),
                         self.sm.legs[10].last_error_extension)
        self.assertTrue(self.sm.legs[10].awaiting_heartbeat)

    def test_invalid_axis_changes_no_state(self):
        msg = type("CanFrame", (object,), {
            "arbitration_id": 0x0EE,
            "data": [24, 2, 0, 0, 0, 0, 0, 8]})()
        self.sm.can_callback(msg)
        self.assertFalse(self.sm.error_latched)
        self.assertFalse(any(
            leg.last_error_code is not None for leg in self.sm.legs))


class FirstDiscoveryUseSelectionTest(unittest.TestCase):
    def setUp(self):
        self.module, self.rospy = load_state_machine()
        self.bus = FakeBus()
        self.sm = self.module.StateMachine(self.bus)

    def test_a_first_connected_enables_use_and_publishes_ui_sync(self):
        self.sm.handle_connection_ping(10, now=0.0)
        leg = self.sm.legs[10]
        self.assertTrue(leg.connected)
        self.assertTrue(leg.discovered_once_in_current_session)
        self.assertTrue(self.sm._is_active(10))
        self.assertTrue(leg.use_selection_initialized)
        self.assertEqual("10,1", self.sm.use_status_pub.messages[-1])

    def test_b_repeated_heartbeat_keeps_use_without_reinitializing(self):
        self.sm.handle_connection_ping(10, now=0.0)
        message_count = len(self.sm.use_status_pub.messages)
        self.sm.handle_connection_ping(10, now=1.0)
        self.sm.handle_connection_ping(10, now=2.0)
        self.assertTrue(self.sm._is_active(10))
        self.assertEqual(message_count, len(self.sm.use_status_pub.messages))
        self.assertEqual("Connected", self.sm.legs[10].state_str)

    def test_c_manual_off_survives_repeated_heartbeat(self):
        self.sm.handle_connection_ping(10, now=0.0)
        self.assertTrue(self.sm.handle_use_selection(10, False))
        self.sm.handle_connection_ping(10, now=1.0)
        self.assertTrue(self.sm.legs[10].connected)
        self.assertFalse(self.sm._is_active(10))
        self.assertTrue(self.sm.legs[10].use_manually_selected)
        self.assertEqual("10,0", self.sm.use_status_pub.messages[-1])

    def test_d_timeout_and_reconnect_preserve_manual_off(self):
        self.sm.handle_connection_ping(10, now=0.0)
        self.sm.handle_use_selection(10, False)
        self.sm.execute(now=4.0)
        self.assertFalse(self.sm.legs[10].connected)
        self.assertFalse(self.sm._is_active(10))
        self.sm.handle_connection_ping(10, now=5.0)
        self.assertTrue(self.sm.legs[10].connected)
        self.assertFalse(self.sm._is_active(10))

    def test_e_manual_off_before_discovery_has_priority(self):
        self.assertTrue(self.sm.handle_use_selection(10, False))
        self.assertTrue(self.sm.legs[10].use_selection_initialized)
        self.sm.handle_connection_ping(10, now=0.0)
        self.assertTrue(self.sm.legs[10].connected)
        self.assertFalse(self.sm._is_active(10))
        self.assertEqual("10,0", self.sm.use_status_pub.messages[-1])

    def test_f_new_axis_after_alignment_start_is_not_auto_added(self):
        self.sm.handle_connection_ping(10, now=0.0)
        self.assertEqual([10], self.sm.handle_alignment_request(now=1.0))
        original_active = self.sm._active_joint_ids()
        self.sm.handle_connection_ping(11, now=2.0)
        self.assertTrue(self.sm.legs[11].connected)
        self.assertTrue(self.sm.legs[11].discovered_once_in_current_session)
        self.assertFalse(self.sm._is_active(11))
        self.assertEqual(original_active, self.sm._active_joint_ids())

    def test_g_connected_use_false_axis_does_not_gate(self):
        self.sm.handle_connection_ping(10, now=0.0)
        self.sm.handle_connection_ping(13, now=0.0)
        self.sm.handle_use_selection(13, False)
        self.sm.handle_alignment_request(now=1.0)
        self.assertTrue(self.sm.handle_alignment_result(10, 1, now=2.0))
        self.assertTrue(self.sm.handle_set_home(10))
        self.assertEqual([], self.sm._home_blocking_reasons())
        self.assertEqual([], self.sm._run_blocking_reasons())
        self.assertTrue(self.sm.legs[13].connected)
        self.assertFalse(self.sm._is_active(13))

    def test_h_auto_on_and_manual_off_publish_matching_ui_values(self):
        self.sm.handle_connection_ping(10, now=0.0)
        self.assertEqual("10,1", self.sm.use_status_pub.messages[-1])
        self.sm.handle_use_selection(10, False)
        self.assertFalse(self.sm._is_active(10))
        self.assertEqual("10,0", self.sm.use_status_pub.messages[-1])


if __name__ == "__main__":
    unittest.main()
