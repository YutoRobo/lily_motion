# -*- coding: utf-8 -*-
from __future__ import division

import os
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from lily_motion_v3.command_timing import (
    resample_transport_records,
    simulate_linear_actuator_records,
    timing_relationship,
)


def make_records(values):
    return [
        {
            'frame_index': i,
            'joint_command_rad': [float(value)] * 24,
        }
        for i, value in enumerate(values)
    ]


class CommandTimingTest(unittest.TestCase):
    def test_shared_transport_factor_two(self):
        records = resample_transport_records(
            make_records([0.0, 2.0, 4.0]), factor=2)
        values = [record['joint_command_rad'][0] for record in records]
        self.assertEqual([0.0, 1.0, 2.0, 3.0, 4.0], values)

    def test_timing_relationship_is_not_hard_coded(self):
        self.assertEqual(
            'matched', timing_relationship(0.100, 0.100))
        self.assertEqual(
            'hold_after_interpolation', timing_relationship(0.120, 0.100))
        self.assertEqual(
            'new_target_before_interpolation_complete',
            timing_relationship(0.080, 0.100))

    def test_matched_actuator_timing_is_continuous(self):
        records = simulate_linear_actuator_records(
            make_records([0.0, 1.0, 2.0]),
            target_period_sec=1.0,
            interpolation_duration_sec=1.0,
            update_period_sec=0.5)
        values = [record['joint_command_rad'][0] for record in records]
        self.assertEqual([0.0, 0.0, 0.0, 0.5, 1.0, 1.5, 2.0], values)
        self.assertAlmostEqual(3.0, records[-1]['actuator_time_sec'])

    def test_shorter_interpolation_duration_creates_hold(self):
        records = simulate_linear_actuator_records(
            make_records([0.0, 1.0, 2.0]),
            target_period_sec=1.0,
            interpolation_duration_sec=0.5,
            update_period_sec=0.25)
        samples = dict(
            (round(record['actuator_time_sec'], 6),
             record['joint_command_rad'][0])
            for record in records)
        self.assertAlmostEqual(1.0, samples[1.5])
        self.assertAlmostEqual(1.0, samples[1.75])
        self.assertAlmostEqual(1.0, samples[2.0])

    def test_longer_interpolation_duration_exposes_current_mcu_restart(self):
        records = simulate_linear_actuator_records(
            make_records([0.0, 1.0, 2.0]),
            target_period_sec=1.0,
            interpolation_duration_sec=1.5,
            update_period_sec=0.5)
        samples = dict(
            (round(record['actuator_time_sec'], 6),
             record['joint_command_rad'][0])
            for record in records)
        self.assertAlmostEqual(1.0 / 3.0, samples[1.5])
        # At t=2.0 a new target arrives before the previous interpolation has
        # completed.  The current MCU semantics restart from the old target,
        # therefore the commanded value becomes exactly 1.0 here.
        self.assertAlmostEqual(1.0, samples[2.0])

    def test_non_integer_period_ratio_keeps_exact_final_target(self):
        records = simulate_linear_actuator_records(
            make_records([0.0, 1.0]),
            target_period_sec=1.0,
            interpolation_duration_sec=0.7,
            update_period_sec=0.4)
        self.assertAlmostEqual(1.7, records[-1]['actuator_time_sec'])
        self.assertAlmostEqual(1.0, records[-1]['joint_command_rad'][0])

    def test_invalid_timing_is_rejected(self):
        with self.assertRaises(ValueError):
            timing_relationship(0.0, 0.1)
        with self.assertRaises(ValueError):
            simulate_linear_actuator_records(
                make_records([0.0, 1.0]),
                target_period_sec=0.1,
                interpolation_duration_sec=0.1,
                update_period_sec=0.0)


if __name__ == '__main__':
    unittest.main()
