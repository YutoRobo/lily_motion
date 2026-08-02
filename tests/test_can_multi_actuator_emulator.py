# -*- coding: utf-8 -*-
from __future__ import print_function

import os
import struct
import sys
import unittest

EMULATOR_DIR = os.path.join(
    os.path.dirname(__file__), "..", "tools", "can_interface", "emulator")
if EMULATOR_DIR not in sys.path:
    sys.path.insert(0, EMULATOR_DIR)

from multi_actuator_emulator import (
    MultiActuatorEmulator, build_scenarios, parse_fail_at_specs,
    validate_channel)
from scenario import ActuatorScenario, parse_axis_spec
from virtual_actuator import (
    ALIMENT, ALIMENT_STANDBY, GET_HOME, RUN, RUN_STANDBY,
    VirtualActuator)


class Frame(object):
    def __init__(self, arbitration_id, data):
        self.arbitration_id = arbitration_id
        self.data = list(data)


def position_payload(value):
    return [0, 0, 0, 0] + list(struct.pack("<f", float(value)))


class MultiActuatorEmulatorUnitTest(unittest.TestCase):
    def test_a_axis_specification(self):
        self.assertEqual([10], parse_axis_spec("10"))
        self.assertEqual([10, 11, 12], parse_axis_spec("10,11,12"))
        self.assertEqual(list(range(24)), parse_axis_spec("0-23"))
        self.assertEqual(
            [0, 1, 2, 10, 12, 13, 14, 15, 23],
            parse_axis_spec("0-2,10,12-15,23"))
        for invalid in ("", "10,10", "24", "-1", "12-10", "a", "1,,2"):
            with self.assertRaises(ValueError, msg=invalid):
                parse_axis_spec(invalid)

    def test_specific_alignment_attempt_parser(self):
        self.assertEqual({11: set((2, 3))},
                         parse_fail_at_specs(("11:2", "11:3")))
        scenario = build_scenarios(
            (11,), fail_at={11: set((2,))})[11]
        self.assertFalse(scenario.alignment_should_fail(1))
        self.assertTrue(scenario.alignment_should_fail(2))
        for invalid in ("11:0", "11:x", "24:1", "11"):
            with self.assertRaises(ValueError):
                parse_fail_at_specs((invalid,))

    def test_can0_is_rejected_before_open(self):
        with self.assertRaises(ValueError):
            validate_channel("can0")
        self.assertEqual("vcan0", validate_channel("vcan0"))
        with self.assertRaises(ValueError):
            validate_channel("eth0")

    def test_b_heartbeat_only_in_standby_and_align_success_stops_it(self):
        sent = []
        actuator = VirtualActuator(
            10, lambda can_id, data: sent.append(Frame(can_id, data)),
            heartbeat_period=1.0, align_delay=0.2, now=0.0)
        actuator.tick(0.0)
        self.assertEqual(0x0FF, sent[-1].arbitration_id)
        self.assertEqual([10, 0, 0, 0, 0, 0, 0, 0], sent[-1].data)
        actuator.receive(0x00A, [0] * 8, now=0.1)
        self.assertEqual(ALIMENT, actuator.low_freq_state)
        before = len(sent)
        actuator.tick(0.2)
        self.assertEqual(before, len(sent))
        actuator.tick(0.31)
        self.assertEqual(GET_HOME, actuator.low_freq_state)
        self.assertEqual(0x10A, sent[-1].arbitration_id)
        before = len(sent)
        actuator.tick(5.0)
        self.assertEqual(before, len(sent))

    def test_b_align_failure_reset_restarts_heartbeat(self):
        sent = []
        actuator = VirtualActuator(
            11, lambda can_id, data: sent.append(Frame(can_id, data)),
            scenario=ActuatorScenario(fail_attempts=(1,), initialization_error_id=8),
            align_delay=0.1, reset_delay=0.2, now=0.0)
        actuator.tick(0.0)
        actuator.receive(0x00B, [0] * 8, now=0.01)
        actuator.tick(0.11)
        self.assertEqual(0x0EE, sent[-1].arbitration_id)
        self.assertEqual([11, 7, 0, 0, 0, 0, 0, 8], sent[-1].data)
        actuator.tick(0.32)
        self.assertEqual(ALIMENT_STANDBY, actuator.low_freq_state)
        self.assertEqual(1, actuator.reset_count)
        self.assertEqual(0x0FF, sent[-1].arbitration_id)
        self.assertEqual(11, sent[-1].data[0])

    def test_d_f_alignment_independence_and_no_success_replay(self):
        sent = []
        emulator = MultiActuatorEmulator(
            (10, 11, 12),
            lambda can_id, data: sent.append(Frame(can_id, data)),
            now=0.0, align_delay=0.1)
        for axis in (10, 11, 12):
            emulator.receive(Frame(axis, [0] * 8), now=0.0)
        emulator.tick(0.11)
        self.assertEqual(
            [0x10A, 0x10B, 0x10C],
            [f.arbitration_id for f in sent])
        before = len(sent)
        emulator.receive(Frame(0x00A, [0] * 8), now=0.2)
        emulator.tick(1.0)
        self.assertEqual(before, len(sent))
        self.assertEqual(GET_HOME, emulator.actuators[10].low_freq_state)

    def test_g_h_i_home_run_and_position_acceptance(self):
        sent = []
        actuator = VirtualActuator(
            10, lambda can_id, data: sent.append(Frame(can_id, data)),
            now=0.0, align_delay=0.0)
        actuator.receive(0x40A, position_payload(0.2), now=0.0)
        self.assertIsNone(actuator.last_position_command_rad)
        actuator.receive(0x00A, [0] * 8, now=0.1)
        actuator.tick(0.1)
        self.assertEqual(GET_HOME, actuator.low_freq_state)
        actuator.receive(0x20A, position_payload(0.1), now=0.2)
        self.assertAlmostEqual(0.1, actuator.last_position_command_rad)
        actuator.receive(0x40A, position_payload(0.2), now=0.3)
        self.assertAlmostEqual(0.2, actuator.last_position_command_rad)
        actuator.receive(0x30A, [0] * 8, now=0.4)
        self.assertEqual(RUN_STANDBY, actuator.low_freq_state)
        actuator.receive(0x40A, position_payload(0.3), now=0.5)
        self.assertAlmostEqual(0.2, actuator.last_position_command_rad)
        actuator.receive(0x60A, [0] * 8, now=0.6)
        self.assertEqual(RUN, actuator.low_freq_state)
        actuator.receive(0x40A, position_payload(0.3), now=0.7)
        self.assertAlmostEqual(0.3, actuator.last_position_command_rad)

    def test_j_representative_axis_ids(self):
        for axis in (0, 10, 11, 23):
            sent = []
            actuator = VirtualActuator(
                axis, lambda can_id, data: sent.append(Frame(can_id, data)),
                now=0.0, align_delay=0.0)
            actuator.receive(0x000 | axis, [0] * 8, now=0.0)
            actuator.tick(0.0)
            actuator.receive(0x200 | axis, position_payload(0.0), now=0.1)
            actuator.receive(0x300 | axis, [0] * 8, now=0.2)
            actuator.receive(0x600 | axis, [0] * 8, now=0.3)
            actuator.receive(0x400 | axis, position_payload(0.005), now=0.4)
            ids = [r.arbitration_id for r in actuator.received_frame_history]
            self.assertEqual(
                [0x000 | axis, 0x200 | axis, 0x300 | axis,
                 0x600 | axis, 0x400 | axis], ids)
            self.assertEqual(0x100 | axis, sent[-1].arbitration_id)

    def test_k_all_24_axes_keep_independent_state(self):
        emulator = MultiActuatorEmulator(
            range(24), lambda unused_id, unused_data: None,
            now=0.0, align_delay=0.0)
        for axis in range(24):
            emulator.receive(Frame(axis, [0] * 8), now=0.0)
        emulator.tick(0.0)
        self.assertEqual(24, len(emulator.actuators))
        self.assertTrue(all(
            actuator.low_freq_state == GET_HOME
            for actuator in emulator.actuators.values()))
        self.assertEqual(
            list(range(24)),
            sorted(actuator.axis for actuator in emulator.actuators.values()))

    def test_l_error_payload_and_injection(self):
        sent = []
        actuator = VirtualActuator(
            12, lambda can_id, data: sent.append(Frame(can_id, data)), now=0.0)
        actuator.inject_error(2, now=1.0)
        self.assertEqual(0x0EE, sent[-1].arbitration_id)
        self.assertEqual([12, 1, 0, 0, 0, 0, 0, 2], sent[-1].data)

    def test_reset_after_run_returns_to_standby(self):
        actuator = VirtualActuator(
            10, lambda unused_id, unused_data: None,
            scenario=ActuatorScenario(reset_after_run_sec=2.0),
            now=0.0, align_delay=0.0)
        actuator.receive(0x00A, [0] * 8, now=0.0)
        actuator.tick(0.0)
        actuator.receive(0x30A, [0] * 8, now=0.1)
        actuator.receive(0x60A, [0] * 8, now=0.2)
        actuator.tick(2.21)
        self.assertEqual(ALIMENT_STANDBY, actuator.low_freq_state)
        self.assertEqual(1, actuator.reset_count)


if __name__ == "__main__":
    unittest.main()
