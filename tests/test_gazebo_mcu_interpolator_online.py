# -*- coding: utf-8 -*-
from __future__ import division

import os
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from lily_motion_v3.gazebo_actuator_interpolator import (
    OnlineLinearActuatorInterpolator,
)


def vector(value):
    return [float(value)] * 24


class OnlineGazeboMcuInterpolatorTest(unittest.TestCase):
    def test_no_output_before_first_target(self):
        emulator = OnlineLinearActuatorInterpolator(0.100, expected_length=24)
        self.assertIsNone(emulator.command_at(0.0))

    def test_first_target_is_treated_as_held_start_pose(self):
        emulator = OnlineLinearActuatorInterpolator(0.100, expected_length=24)
        emulator.set_target(vector(2.0), 1.0)
        self.assertAlmostEqual(2.0, emulator.command_at(1.0)[0])
        self.assertAlmostEqual(2.0, emulator.command_at(1.5)[0])

    def test_second_target_interpolates_by_elapsed_time(self):
        emulator = OnlineLinearActuatorInterpolator(0.100, expected_length=24)
        emulator.set_target(vector(0.0), 0.0)
        emulator.set_target(vector(1.0), 0.100)
        self.assertAlmostEqual(0.0, emulator.command_at(0.100)[0])
        self.assertAlmostEqual(0.5, emulator.command_at(0.150)[0])
        self.assertAlmostEqual(1.0, emulator.command_at(0.200)[0])
        self.assertAlmostEqual(1.0, emulator.command_at(1.000)[0])

    def test_new_target_before_completion_restarts_from_old_target(self):
        emulator = OnlineLinearActuatorInterpolator(0.200, expected_length=24)
        emulator.set_target(vector(0.0), 0.0)
        emulator.set_target(vector(1.0), 0.100)
        self.assertAlmostEqual(0.5, emulator.command_at(0.200)[0])
        emulator.set_target(vector(2.0), 0.200)
        # Current MCU semantics restart from the old target value (1.0), not
        # from the partially interpolated output (0.5).
        self.assertAlmostEqual(1.0, emulator.command_at(0.200)[0])
        self.assertAlmostEqual(1.5, emulator.command_at(0.300)[0])
        self.assertAlmostEqual(2.0, emulator.command_at(0.400)[0])

    def test_timing_parameter_is_changeable(self):
        emulator = OnlineLinearActuatorInterpolator(0.050, expected_length=24)
        emulator.set_target(vector(0.0), 0.0)
        emulator.set_target(vector(1.0), 0.100)
        self.assertAlmostEqual(0.5, emulator.command_at(0.125)[0])
        self.assertAlmostEqual(1.0, emulator.command_at(0.150)[0])

    def test_invalid_target_is_rejected_without_losing_old_target(self):
        emulator = OnlineLinearActuatorInterpolator(0.100, expected_length=24)
        emulator.set_target(vector(1.0), 0.0)
        with self.assertRaises(ValueError):
            emulator.set_target([0.0] * 23, 0.1)
        self.assertAlmostEqual(1.0, emulator.command_at(0.2)[0])


if __name__ == '__main__':
    unittest.main()
