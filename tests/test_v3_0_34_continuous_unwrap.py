# -*- coding: utf-8 -*-
from __future__ import print_function
import math
import unittest
from lily_motion_v3.command_resampler import unwrap_continuous_command_records, boundary_transition_diagnostics

class TestContinuousUnwrap(unittest.TestCase):
    def test_unwrap_reduces_equivalent_jump(self):
        recs = [
            {'joint_command_rad':[math.radians(179.0)], 'roll_index':0, 'frame_index':0},
            {'joint_command_rad':[math.radians(-179.0)], 'roll_index':1, 'frame_index':1},
        ]
        raw = boundary_transition_diagnostics(recs)['worst_boundary']['max_abs_delta_deg']
        out = unwrap_continuous_command_records(recs)
        fixed = boundary_transition_diagnostics(out)['worst_boundary']['max_abs_delta_deg']
        self.assertGreater(raw, 300.0)
        self.assertLess(fixed, 5.0)

if __name__ == '__main__':
    unittest.main()
